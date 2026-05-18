#!/bin/bash
#
# getVehicleTime.sh -- read the JEEP dashboard clock from the CAN-IHS bus.
#
# Originally created: 11.2023
# Last updated:       05.2026
#
# JEEP / FCA vehicles broadcast the wall clock on CAN message $350 once per
# second, on both CAN-C and CAN-IHS. This script captures one $350 frame
# from CAN-IHS, decodes the 7-byte raw-hex payload (NOT BCD), and prints
# the result in the requested format.
#
# USAGE
#     ./getVehicleTime.sh [--format iso|us|epoch|json] [--retries N]
#                        [--verbose] [--help]
#
# OPTIONS
#     -f, --format <fmt>   Output format (default: iso)
#         iso     2026-05-16 09:21:05    (ISO 8601, unambiguous, default)
#         us      5/16/2026 09:21:05     (matches legacy script output)
#         epoch   1747387265              (Unix timestamp via `date`)
#         json    {"year":2026,...}       (one-line JSON for piping to jq)
#     -r, --retries <N>    Attempts before giving up (default 2). $350 is
#                          1Hz; one well-timed miss is common. Each
#                          attempt waits up to $TIMEOUT_SEC seconds.
#                          Attempts 2+ send an NM wake frame first in
#                          case the vehicle has dozed off since the
#                          last broadcast.
#     -v, --verbose        Echo the raw candump line and the byte-by-byte
#                          decode alongside the formatted output.
#     -h, --help           Print this message and exit.
#
# REQUIRES
#     - can-utils (candump)    apt install can-utils
#     - CAN interface up and configured: $CAN_IHS
#     - Vehicle must be awake. $350 stops broadcasting in sleep mode --
#       absence of $350 is itself the sleep-state signal.
#
# PAYLOAD LAYOUT  (raw hex, NOT BCD)
#     byte 0   seconds            0x00..0x3B
#     byte 1   minutes            0x00..0x3B
#     byte 2   hours              0x00..0x17
#     bytes 3-4 year (big-endian) e.g. 0x07E7 = 2023
#     byte 5   month              0x01..0x0C
#     byte 6   day                0x01..0x1F
#     all-0xFF = clock not yet initialized (battery disconnect, no
#               radio-set yet)
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-350
#     -- full $350 layout, broadcast cadence, sentinel semantics.
#
# PLATFORM NOTE
#     $350 is one of the more portable FCA IDs (the dashboard clock has
#     to render somewhere) but byte order has been observed to differ on
#     some non-JEEP platforms. Verify with candump on your own vehicle
#     before trusting the output.
#
# REVISION NOTES  (2026-05-16)
#     - Added shebang + `set -eu`
#     - Timeout bumped to 3s + retry loop (default 2 attempts). $350
#       is 1Hz; a 1s timeout had a real chance of missing the window.
#     - Replaced brittle column-based `cut -c30-43` / `cut -c1-11,13-20`
#       parsing with `${line#*#}` + parameter-expansion byte slicing.
#       No more dependency on candump's exact output column positions.
#     - `[ $X == "Y" ]` -> `[[ ]]` throughout
#     - `timeout -s 9` -> default SIGTERM (lets candump flush stdout)
#     - Default output format is now ISO 8601 (unambiguous); legacy
#       m/d/yyyy still available via `--format us`
#     - CLI: --format iso|us|epoch|json, --retries N, --verbose, --help
#     - Pre-flight CAN interface check with bitrate hint on failure
#     - Distinct error paths for "no frame" vs "all-FF uninitialised"
#     Original preserved as getVehicleTime.legacy.txt.
#
# REVISION NOTES  (2026-05-16, second pass)
#     - Wake-bus-then-retry: on attempt 2+, send the standard NM wake
#       frame ($2D3) before re-listening. Matches the pattern in
#       jmccorm's later getVehicleTime variant; if the vehicle dozed
#       off between attempts, this can be the difference between
#       success and failure. Harmless when the vehicle is already
#       awake (the wake frame is the same one 3rd_brakelight.sh and
#       horn.sh send before their UDS sessions). First attempt skips
#       the wake to keep the success-case latency unchanged.

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

CAN_IHS=can0                # CAN-IHS interface
CAN_ID=350                  # Message ID to capture
TIMEOUT_SEC=3               # Per-attempt timeout. Must be > 1s since
                            # $350 is broadcast at 1Hz; tight timeouts
                            # frequently miss the next emission.
DEFAULT_RETRIES=2           # Total attempts before giving up

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

OUTPUT_FORMAT=iso
VERBOSE=0
RETRIES=$DEFAULT_RETRIES

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
getVehicleTime.sh -- read the JEEP dashboard clock from message \$350 on CAN-IHS

USAGE
    $0 [--format iso|us|epoch|json] [--retries N] [--verbose] [--help]

OPTIONS
    -f, --format FMT   Output format (default: iso)
                       iso, us, epoch, json
    -r, --retries N    Attempts before giving up (default: $DEFAULT_RETRIES)
    -v, --verbose      Echo raw candump line and per-byte decode
    -h, --help         Print this message

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-350
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -f|--format)
            shift
            case "${1:-}" in
                iso|us|epoch|json) OUTPUT_FORMAT="$1" ;;
                *) echo "ERROR: --format must be iso|us|epoch|json (got: ${1:-<empty>})" >&2; exit 2 ;;
            esac
            ;;
        -r|--retries)
            shift
            RETRIES="${1:-}"
            if ! [[ "$RETRIES" =~ ^[0-9]+$ ]] || [[ "$RETRIES" -lt 1 ]]; then
                echo "ERROR: --retries must be a positive integer (got: ${RETRIES:-<empty>})" >&2
                exit 2
            fi
            ;;
        -v|--verbose) VERBOSE=1 ;;
        -h|--help)    usage ;;
        *)
            echo "unknown arg: $1" >&2
            echo "usage: $0 [--format FMT] [--retries N] [--verbose] [--help]" >&2
            exit 2
            ;;
    esac
    shift
done

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

# Pretty error + exit. Used when no $350 frame is captured.
fail_no_frame() {
    cat >&2 <<EOF
ERROR: No valid \$$CAN_ID frame seen on $CAN_IHS within $RETRIES attempt(s)
       of $TIMEOUT_SEC s each. Possible causes:

         - Vehicle is asleep (ignition off, doors closed > ~10 min).
           \$$CAN_ID stops broadcasting in sleep mode.
         - Wrong CAN interface (current: $CAN_IHS). Try a different one.
         - CAN bitrate mismatch on the interface. CAN-IHS is 125 kbps.
         - Cable / adapter problem -- run plain 'candump $CAN_IHS' to see
           if ANY traffic is flowing.
EOF
    exit 1
}

# Pretty error + exit. Used when $350 carries the all-FF sentinel.
fail_uninitialized() {
    cat >&2 <<EOF
ERROR: \$$CAN_ID payload is all 0xFF -- the dashboard clock has never
       been initialized on this vehicle. This happens after a battery
       disconnect or fresh radio swap when the user hasn't set the time
       yet. Set the time via the head unit and re-run.
EOF
    exit 1
}

# Capture one frame of $CAN_ID from $CAN_IHS. Returns the raw candump
# line on stdout, or empty string on miss. Uses default SIGTERM (not
# SIGKILL) so candump has a chance to flush its line buffer.
capture_one() {
    timeout "$TIMEOUT_SEC" candump -L "$CAN_IHS,0${CAN_ID}:0fff" 2>/dev/null | tail -1
}

# Send the standard NM (Network Management) wake frame on $CAN_IHS.
# Called between retry attempts to rouse the bus if the vehicle has
# dozed off. Harmless when the vehicle is already awake. Failures are
# tolerated -- if the bus is fully down the cansend itself will fail,
# but we still want to fall through to the next capture attempt
# rather than abort.
wake_bus() {
    [[ $VERBOSE -eq 1 ]] && echo "  sending NM wake frame ($CAN_IHS \$2D3)" >&2
    cansend "$CAN_IHS" "2D3#0700000000000000" 2>/dev/null || true
    sleep 0.2  # let modules respond to the wake before we re-listen
}

# Extract the payload (everything after the first '#') from a candump
# line. Uses bash parameter expansion so we don't depend on column
# positions or external tools.
payload_of() {
    local line="$1"
    [[ "$line" == *"#"* ]] || { echo ""; return; }
    echo "${line#*#}"
}

# Decode and print. Expects the 14-hex-char payload (7 bytes) as $1.
decode_and_print() {
    local p="$1"

    # All-FF sentinel: clock has never been initialized.
    if [[ "${p:0:14}" == "FFFFFFFFFFFFFF" ]]; then
        fail_uninitialized
    fi

    # Per-byte slice using parameter expansion. Each ${p:N:2} grabs 2
    # hex chars starting at offset N. See the PAYLOAD LAYOUT block in
    # the header for what each byte means.
    local sec_hex="${p:0:2}"
    local min_hex="${p:2:2}"
    local hr_hex="${p:4:2}"
    local yr_hex="${p:6:4}"   # 2 bytes glued together, big-endian
    local mo_hex="${p:10:2}"
    local dy_hex="${p:12:2}"

    # Convert hex to decimal. The leading-zero pattern "0x07" is fine
    # for $((16#..)), no octal-interpretation trap like with $((0x..)).
    local sec=$((16#$sec_hex))
    local min=$((16#$min_hex))
    local hr=$((16#$hr_hex))
    local yr=$((16#$yr_hex))
    local mo=$((16#$mo_hex))
    local dy=$((16#$dy_hex))

    if [[ $VERBOSE -eq 1 ]]; then
        printf '  raw payload : %s\n' "$p" >&2
        printf '  bytes       : ss=%s mm=%s hh=%s YYYY=%s MM=%s DD=%s\n' \
            "$sec_hex" "$min_hex" "$hr_hex" "$yr_hex" "$mo_hex" "$dy_hex" >&2
        printf '  decoded     : %04d-%02d-%02d %02d:%02d:%02d\n' \
            "$yr" "$mo" "$dy" "$hr" "$min" "$sec" >&2
    fi

    case "$OUTPUT_FORMAT" in
        iso)
            printf '%04d-%02d-%02d %02d:%02d:%02d\n' \
                "$yr" "$mo" "$dy" "$hr" "$min" "$sec"
            ;;
        us)
            # Matches the legacy script's m/d/yyyy h:m:s output.
            printf '%d/%d/%d %02d:%02d:%02d\n' \
                "$mo" "$dy" "$yr" "$hr" "$min" "$sec"
            ;;
        epoch)
            # `date -d` interprets the ISO-style string and emits Unix
            # epoch seconds. Note: this uses the LOCAL timezone of the
            # host running the script -- vehicle clock is assumed to
            # match local time, which is true on most dashboards.
            date -d "$(printf '%04d-%02d-%02d %02d:%02d:%02d' \
                "$yr" "$mo" "$dy" "$hr" "$min" "$sec")" +%s
            ;;
        json)
            printf '{"year":%d,"month":%d,"day":%d,"hours":%d,"minutes":%d,"seconds":%d}\n' \
                "$yr" "$mo" "$dy" "$hr" "$min" "$sec"
            ;;
    esac
}

# ---------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------

if ! ip link show "$CAN_IHS" >/dev/null 2>&1; then
    echo "ERROR: CAN interface $CAN_IHS not found" >&2
    echo "       Bring it up first, e.g.:" >&2
    echo "         ip link set $CAN_IHS up type can bitrate 125000" >&2
    exit 1
fi

# ---------------------------------------------------------------------
# Main: capture with retry
# ---------------------------------------------------------------------

raw=""
attempt=0
while [[ $attempt -lt $RETRIES ]]; do
    attempt=$((attempt + 1))
    # First attempt: just listen. Subsequent attempts: wake the bus
    # first in case the vehicle has dozed off since the last $350
    # broadcast. Harmless if it's already awake; potentially the
    # difference between success and failure if it isn't.
    if [[ $attempt -gt 1 ]]; then
        wake_bus
    fi
    [[ $VERBOSE -eq 1 ]] && echo "  attempt $attempt/$RETRIES (timeout ${TIMEOUT_SEC}s)" >&2
    raw=$(capture_one)
    if [[ -n "$raw" ]]; then
        break
    fi
done

if [[ -z "$raw" ]]; then
    fail_no_frame
fi

payload=$(payload_of "$raw")
if [[ -z "$payload" ]]; then
    fail_no_frame
fi

[[ $VERBOSE -eq 1 ]] && echo "  raw candump : $raw" >&2
decode_and_print "$payload"
