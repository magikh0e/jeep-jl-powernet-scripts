#!/bin/bash
#
# horn.sh -- honk the JEEP horn via UDS Service 0x2F on DID $D0AD.
#
# Original author:   jmccorm  (discovered DID $D0AD on the JEEP platform
#                              and authored the five-tap burst bash recipe
#                              this script is built on top of)
# Updates / polish:  magikh0e
# Last updated:      05.2026
#
# Same UDS service-path template as 3rd_brakelight.sh: NM wake -> Extended
# Diagnostic Session -> IOControlByIdentifier (Service 0x2F) -> takeover
# state, then revert. Only the DID changes: $D1B3 (brake light) -> $D0AD
# (horn). Demonstrates that this is a reusable pattern for any actuator
# the BCM / target ECU exposes through IOControlByIdentifier.
#
# Three modes, picked by CLI flag:
#
#   default          one short beep (~300 ms)
#   --press DUR_MS   hold the horn for DUR_MS milliseconds
#   --burst N        N quick taps (alternating ON/OFF on $TAP_MS / $GAP_MS)
#
# On exit (normal or SIGINT) the script issues an explicit state-OFF and
# returnControlToECU so the horn is NEVER left held by a crashed or
# interrupted run. This matters more here than for the brake light --
# a stuck horn at 3am is a different category of mistake.
#
# USAGE
#     ./horn.sh                              short beep (~300 ms)
#     ./horn.sh --press 1000                 hold for 1000 ms
#     ./horn.sh --burst 5                    5 quick taps
#     ./horn.sh --burst 5 --tap-ms 50 --gap-ms 100
#     ./horn.sh --verbose
#
# OPTIONS
#     --press DUR_MS   Sustained press for DUR_MS milliseconds (default 300)
#     --burst N        N quick taps; alternates ON / OFF
#     --tap-ms MS      ON duration per burst tap (default 25)
#     --gap-ms MS      OFF duration between burst taps (default 25)
#     -v, --verbose    Echo every cansend invocation as it's issued
#     -h, --help       Print usage and exit
#
# REQUIRES
#     - can-utils (cansend, ip)         apt install can-utils
#     - Two CAN interfaces up:          $WAKE_BUS and $UDS_BUS
#     - Physical CAN access. On 2018+ FCA / Stellantis vehicles the Secure
#       Gateway Module blocks UDS writes from the OBD-II port; use the
#       13-way connectors behind the glovebox. See:
#         https://magikh0e.pl/pubCarHacking/secure-gateway-module.html
#
# PLATFORM NOTE
#     DID $D0AD is the horn identifier on the platform this script was
#     developed against. DIDs are NOT portable across FCA model years --
#     verify with wiTECH or the JL Wrangler RE spreadsheet before running
#     on a different platform:
#         https://docs.google.com/spreadsheets/d/16ypMADKinBBnH1pOY4-gMmVRjeR85fYplpV12aCHJC4/edit?gid=303439516#gid=303439516
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#uds
#     -- UDS service-path walkthrough; this script is one example of the
#     general $D1B3 / $D0AD / ... template.
#
# CAVEAT
#     Honks in public places are socially loaded. Bench-test before
#     vehicle-test, and please run it in a driveway or empty lot during
#     daylight hours, not 2am in a dense neighbourhood.

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

WAKE_BUS=can0                # Network Management (NM) wake bus
UDS_BUS=can1                 # UDS request/response bus

ECU_REQ_ID=620               # UDS REQUEST arbitration ID (tester -> ECU)
ECU_RES_ID=628               # UDS RESPONSE arbitration ID (ECU -> tester)
                             # FCA convention: response = request + 0x08

DID_HORN=D0AD                # Data Identifier -- horn
CTRL_SHORT_TERM=03           # IOCtrl control byte: shortTermAdjustment
CTRL_RETURN=00               # IOCtrl control byte: returnControlToECU

DEFAULT_PRESS_MS=300         # Default single-beep duration
DEFAULT_TAP_MS=25            # Default burst tap ON duration
DEFAULT_GAP_MS=25            # Default burst tap OFF duration

SETUP_DELAY=0.1              # Settle time after wake / session-control

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

VERBOSE=0
MODE=press                   # press | burst
PRESS_MS=$DEFAULT_PRESS_MS
BURST_N=1
TAP_MS=$DEFAULT_TAP_MS
GAP_MS=$DEFAULT_GAP_MS
CLEANED=0

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
horn.sh -- honk the JEEP horn via UDS

USAGE
    $0                              short beep (~${DEFAULT_PRESS_MS} ms)
    $0 --press DUR_MS               sustained press for DUR_MS milliseconds
    $0 --burst N                    N quick taps
    $0 --burst N --tap-ms MS --gap-ms MS    custom burst timing

OPTIONS
    --press DUR_MS   Sustained press for DUR_MS milliseconds (default $DEFAULT_PRESS_MS)
    --burst N        Quick-tap burst with N taps
    --tap-ms MS      ON duration per burst tap (default $DEFAULT_TAP_MS)
    --gap-ms MS      OFF duration between burst taps (default $DEFAULT_GAP_MS)
    -v, --verbose    Echo every cansend invocation
    -h, --help       Print this message

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html#uds
EOF
    exit 0
}

# Validate a non-negative integer arg.
need_int() {
    local val="$1" name="$2"
    if ! [[ "$val" =~ ^[0-9]+$ ]]; then
        echo "ERROR: $name must be a non-negative integer (got: $val)" >&2
        exit 2
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --press)   shift; need_int "${1:-}" "--press"; MODE=press; PRESS_MS="$1" ;;
        --burst)   shift; need_int "${1:-}" "--burst"; MODE=burst; BURST_N="$1" ;;
        --tap-ms)  shift; need_int "${1:-}" "--tap-ms"; TAP_MS="$1" ;;
        --gap-ms)  shift; need_int "${1:-}" "--gap-ms"; GAP_MS="$1" ;;
        -v|--verbose) VERBOSE=1 ;;
        -h|--help)    usage ;;
        *)
            echo "unknown arg: $1" >&2
            echo "usage: $0 [--press DUR_MS | --burst N] [--tap-ms MS] [--gap-ms MS] [--verbose] [--help]" >&2
            exit 2
            ;;
    esac
    shift
done

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

send() {
    local bus="$1" frame="$2" label="$3"
    [ "$VERBOSE" = "1" ] && echo "  cansend $bus $frame  # $label"
    cansend "$bus" "$frame"
}

# Sleep for an integer number of milliseconds. `sleep` accepts fractional
# seconds on GNU coreutils so this is portable enough for Linux hosts.
sleep_ms() {
    local ms="$1"
    awk -v ms="$ms" 'BEGIN { printf "%.3f", ms / 1000 }' \
        | xargs sleep
}

# IOControlByIdentifier on the horn DID. $1 = "01" (honk) or "00" (silent).
iocontrol_horn() {
    local state="$1" label="$2"
    local frame
    printf -v frame '052F%s%s%s0000' "$DID_HORN" "$CTRL_SHORT_TERM" "$state"
    send "$UDS_BUS" "$ECU_REQ_ID#$frame" "$label"
}

return_control() {
    local frame
    printf -v frame '042F%s%s000000' "$DID_HORN" "$CTRL_RETURN"
    send "$UDS_BUS" "$ECU_REQ_ID#$frame" "returnControlToECU"
}

# ---------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------

cleanup() {
    [ "$CLEANED" = "1" ] && return 0
    CLEANED=1
    echo
    echo "[cleanup] silencing horn and returning control to ECU"
    iocontrol_horn 00 "Horn OFF (cleanup)" || true
    return_control || true
    exit 0
}
trap cleanup INT TERM

# ---------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------

for iface in "$WAKE_BUS" "$UDS_BUS"; do
    if ! ip link show "$iface" >/dev/null 2>&1; then
        echo "ERROR: CAN interface $iface not found" >&2
        echo "       Bring it up first, e.g.:" >&2
        echo "         ip link set $iface up type can bitrate 500000" >&2
        exit 1
    fi
done

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

# 1. NM (Network Management) wake frame. Not part of UDS itself --
#    this knocks ECUs out of low-power sleep so they're listening when
#    we start the diagnostic session below.
echo "[1] Wake $WAKE_BUS via NM frame"
send "$WAKE_BUS" "2D3#0700000000000000" "NM wake frame"
sleep "$SETUP_DELAY"

# 2. Enter Extended Diagnostic Session.
echo "[2] Enter Extended Diagnostic Session on $UDS_BUS"
send "$UDS_BUS" "$ECU_REQ_ID#0210030000000000" "DiagnosticSessionControl extended"
sleep "$SETUP_DELAY"

# 3. Drive the horn according to the chosen mode.
case "$MODE" in
    press)
        echo "[3] Honk: sustained press for ${PRESS_MS} ms"
        iocontrol_horn 01 "Horn ON"
        sleep_ms "$PRESS_MS"
        iocontrol_horn 00 "Horn OFF"
        ;;
    burst)
        echo "[3] Honk: $BURST_N quick taps (${TAP_MS}ms on / ${GAP_MS}ms off)"
        for ((i = 1; i <= BURST_N; i++)); do
            iocontrol_horn 01 "Horn ON  (tap $i/$BURST_N)"
            sleep_ms "$TAP_MS"
            iocontrol_horn 00 "Horn OFF (tap $i/$BURST_N)"
            sleep_ms "$GAP_MS"
        done
        ;;
esac

# 4. Hand control back to the ECU. Even on a clean run -- the cleanup
#    trap will do the same on Ctrl+C, but the normal-exit path here
#    keeps the bus state symmetric whether or not we were interrupted.
cleanup

# REVISION NOTES  (2026-05-16)
#     - Original bash recipe by jmccorm: DID $D0AD discovery on the JEEP
#       platform, the NM-wake + Session-Control + IOControl sequence,
#       and the five-tap burst pattern. This rewrite preserves jmccorm's
#       exact byte sequences and burst-timing logic verbatim; everything
#       below is refactor / polish on top.
#     - Pattern matches the rewritten 3rd_brakelight.sh (Originally
#       created 11.2023, last updated 05.2026) -- only the DID changes
#       ($D1B3 -> $D0AD) and the CLI offers press/burst modes instead
#       of the brake-light's on/off loop.
#     - Added shebang + `set -eu`, named constants for buses / IDs / DIDs
#       / control bytes / delays
#     - SIGINT trap silences the horn and returnsControlToECU on exit
#       (matters more than the brake light -- a stuck horn is socially /
#       legally louder than a stuck cargo light)
#     - sleep_ms helper for millisecond-precision delays (awk + xargs sleep)
#     - Pre-flight check for both CAN interfaces
#     - CLI: --press DUR_MS, --burst N, --tap-ms MS, --gap-ms MS,
#       --verbose, --help
