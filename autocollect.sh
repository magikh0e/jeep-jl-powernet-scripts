#!/bin/bash
#
# autocollect.sh -- engine-state event framework for an in-vehicle Pi.
#
# Original author:   jmccorm
# Updates / polish:  magikh0e
# Last updated:      05.2026
#
# Monitors $077 broadcast on CAN-C for engine-state transitions and
# fires user-customizable hook functions on each event:
#
#     vehiclepoweredon   -- vehicle powered on (TIP / remote start)
#     enginestarted      -- engine running, observation stable
#     oneminute          -- once per minute while engine running
#     tenminute          -- once per ten minutes while engine running
#     onehour            -- once per hour while engine running
#     engineshutdown     -- engine off, last chance to flush state
#
# Each hook is a no-op stub by default; populate with your own work
# (start a logger, take a GPS fix, trigger SD-card flush, send a
# heartbeat, etc.) and the framework handles the timing.
#
# RELATED SCRIPTS
#     Blackbox_monitor.sh is a more focused start-recorder / stop-
#     recorder script keyed on $122 (CAN-IHS ignition state). This
#     script is the more general framework keyed on $077 (CAN-C
#     engine running state). They can run alongside each other --
#     they watch different message IDs on different buses for
#     different things. Use Blackbox for "I just want a dashcam-
#     style recorder"; use autocollect for "I want event hooks at
#     multiple rates."
#
# USAGE
#     ./autocollect.sh [--once] [--debug LEVEL] [--verbose] [--help]
#
# OPTIONS
#     --once          Exit after the first OFF->RUN or RUN->OFF
#                     transition (for testing the framework without
#                     waiting for a real drive cycle).
#     --debug LEVEL   off|on|raw  (default: on).
#                       off  silent except for hook output
#                       on   event + hook-dispatch notices
#                       raw  also dumps every observed $077 state
#     -v, --verbose   Alias for --debug raw
#     -h, --help      Print this message and exit
#
# REQUIRES
#     - can-utils (candump)     apt install can-utils
#     - $CAN_C interface up at 500 kbps:
#         ip link set $CAN_C up type can bitrate 500000
#
# BACKGROUND -- $077 STATE ENCODING (CAN-C)
#     Message $077 broadcasts engine / vehicle state on CAN-C. The
#     first two bytes of the payload encode the state as a 16-bit
#     hex value; observed codes on the JEEP platform this script
#     was developed against:
#
#         0x0422   Engine running (most common code)
#         0x4421   Engine running (alternate / mode-dependent)
#         0x5D21   Remote start in progress
#         < 0x0400 Vehicle off / accessory / pre-crank
#         > 0x0399 Vehicle on (engine may or may not be running)
#
#     Bytes 2-7 of the $077 payload aren't decoded by this script;
#     they likely carry additional state we haven't characterised
#     (sub-mode, transmission state, etc).
#
#     See <a href="bus-message-reference.html#id-077"> on the BMR for
#     the full byte-level reference once it's filled out further.
#
# WHAT THE POLISHED VERSION FIXED vs. THE LEGACY RECIPE
#     - Original used decimal-string comparison via `-gt` / `-lt`
#       against hex strings, which crashes whenever the state byte
#       contains a non-decimal hex digit (A-F). The original
#       hard-coded a `5D21 -> 5555` workaround for the one observed
#       case. This rewrite converts via $((16#$state)) so all hex
#       values compare correctly without per-state special-cases.
#     - Added shebang + `set -eu`, CLI flags, pre-flight check,
#       SIGINT trap, structured logging, named constants for the
#       state thresholds.
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-077
#     https://magikh0e.pl/pubCarHacking/scripts/Blackbox_monitor.txt
#         Sibling: simpler RUN/OFF transition handler on $122.

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

CAN_C=can1                              # CAN-C interface
CAN_ID=077                              # Message ID broadcasting state

# State thresholds (as decimal integers; the script converts the
# raw hex string from the bus via $((16#$state)) before comparing,
# so these can stay as ordinary integers).
RUN_THRESHOLD_HEX=0x0400                # State >= this means vehicle on
ENGINE_RUNNING_STATES=(0x0422 0x4421)   # Engine actually running

# Hook dispatch cadence. Each modulo's offset is the legacy script's
# choice -- keeps the hooks from all firing on the same second.
ONEMINUTE_OFFSET=45                     # fire when SECONDS % 60 == 45
TENMINUTE_OFFSET=599                    # fire when SECONDS % 600 == 599
ONEHOUR_OFFSET=35970                    # fire when SECONDS % 36000 == 35970

CANDUMP_RETRY_DELAY=5                   # Backoff between pipe restarts

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

DEBUG=on                # off|on|raw
ONCE=0
VEHICLE_ON=0
ENGINE_RUNNING=0
LAST_SECOND=-1          # rate-limit gate (init to impossible value)
CLEANED=0

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
autocollect.sh -- engine-state event framework (watches \$$CAN_ID on $CAN_C)

USAGE
    $0 [--once] [--debug off|on|raw] [--verbose] [--help]

OPTIONS
    --once          Exit after first state transition (testing)
    --debug LEVEL   off|on|raw  (default: on)
    -v, --verbose   Alias for --debug raw
    -h, --help      Print this message

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-077
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --once) ONCE=1 ;;
        --debug)
            shift
            case "${1:-}" in
                off|on|raw) DEBUG="$1" ;;
                *) echo "ERROR: --debug must be off|on|raw" >&2; exit 2 ;;
            esac
            ;;
        -v|--verbose) DEBUG=raw ;;
        -h|--help)    usage ;;
        *)
            echo "unknown arg: $1" >&2
            echo "usage: $0 [--once] [--debug LEVEL] [--verbose] [--help]" >&2
            exit 2
            ;;
    esac
    shift
done

# ---------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------

log()       { echo "$(date) AUTOCOLLECT: $*"; }
debug()     { [[ "$DEBUG" != "off" ]] && log "$*" || true; }
raw_log()   { [[ "$DEBUG" == "raw" ]] && log "$*" || true; }

# ---------------------------------------------------------------------
# USER-DEFINED HOOK FUNCTIONS
# ---------------------------------------------------------------------
# Each hook runs in the background (`&`) so a slow user hook doesn't
# block the main monitoring loop. If your hook needs to be serial,
# remove the `&` from the dispatch calls in the main loop below.

# Called when the vehicle has been powered on (TIP button or remote
# start). Engine may or may not be running yet at this point.
vehiclepoweredon() {
    log "Vehicle power-on items go here."
}

# Called once when the engine first becomes RUNNING. Will not fire
# again unless the engine first goes OFF and then RUNNING again.
enginestarted() {
    log "Post engine-start items go here."
}

# Called once per minute while the engine is running.
oneminute() {
    log "One-minute items go here."
}

# Called once per ten minutes while the engine is running.
tenminute() {
    log "Ten-minute items go here."
}

# Called once per hour while the engine is running.
onehour() {
    log "One-hour items go here."
}

# Called when the engine has just shut down. Last chance to flush
# state, sync I/O, etc. before the Pi loses 12V power.
engineshutdown() {
    log "Engine shutdown items go here."
    # Flush any cached / pending I/O (SD card writes especially).
    sync; sync; sleep 5; sync; sync
}

# ---------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------

cleanup() {
    [[ "$CLEANED" -eq 1 ]] && return 0
    CLEANED=1
    log ""
    log "[cleanup] tearing down on signal"
    exit 0
}
trap cleanup INT TERM

# ---------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------

if ! ip link show "$CAN_C" >/dev/null 2>&1; then
    log "ERROR: CAN interface $CAN_C not found"
    log "       Bring it up first: ip link set $CAN_C up type can bitrate 500000"
    exit 1
fi

if ! command -v candump >/dev/null 2>&1; then
    log "ERROR: candump not found. Install: apt install can-utils"
    exit 1
fi

# ---------------------------------------------------------------------
# Helper -- check whether a given hex state is in the running list
# ---------------------------------------------------------------------
is_engine_running() {
    local state="$1" running
    for running in "${ENGINE_RUNNING_STATES[@]}"; do
        [[ $state -eq $running ]] && return 0
    done
    return 1
}

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

log "Script started, waiting for events on \$$CAN_ID via $CAN_C"
log "Hook dispatch cadence: oneminute=45s tenminute=599s onehour=35970s"

# Outer reconnect loop: if the candump pipe dies (CAN interface drops,
# vehicle goes fully asleep so the bus stops carrying any traffic),
# back off and reconnect.
while true; do
    candump -T 1000 -L "$CAN_C,0${CAN_ID}:0FFF" 2>/dev/null \
    | while read -r TIME BUS FRAME; do

        # Rate-limit decode work to once per second. We don't care
        # about every $077 broadcast -- state transitions are slow.
        NOW=$SECONDS
        [[ "$NOW" -eq "$LAST_SECOND" ]] && continue
        LAST_SECOND=$NOW

        # Extract the first 4 hex chars (2 bytes) of payload.
        payload="${FRAME##*#}"
        state_hex="${payload:0:4}"
        if ! [[ "$state_hex" =~ ^[0-9A-Fa-f]{4}$ ]]; then
            raw_log "  skip: non-hex state '$state_hex' in $FRAME"
            continue
        fi
        state=$((16#$state_hex))

        raw_log "  state=0x$state_hex ($state) vehicle_on=$VEHICLE_ON engine_running=$ENGINE_RUNNING"

        # ---- Engine-running transitions ----------------------------
        if is_engine_running "$state"; then
            if [[ $ENGINE_RUNNING -eq 0 ]]; then
                debug "EVENT: engine started"
                enginestarted &
                ENGINE_RUNNING=1
            fi

            # Periodic hooks while engine is running. Modulo offsets
            # are jmccorm's original choices, kept verbatim so anyone
            # porting a populated copy of this script doesn't lose
            # their existing tuning.
            (( SECONDS % 60 == ONEMINUTE_OFFSET ))    && oneminute &
            (( SECONDS % 600 == TENMINUTE_OFFSET ))   && tenminute &
            (( SECONDS % 36000 == ONEHOUR_OFFSET ))   && onehour &
        fi

        # ---- Vehicle-on transition --------------------------------
        if (( state >= RUN_THRESHOLD_HEX )); then
            if [[ $VEHICLE_ON -eq 0 ]]; then
                debug "EVENT: vehicle powered on (state 0x$state_hex)"
                vehiclepoweredon &
                VEHICLE_ON=1
            fi
        fi

        # ---- Engine-off transition --------------------------------
        if (( state < RUN_THRESHOLD_HEX )); then
            if [[ $ENGINE_RUNNING -eq 1 ]]; then
                debug "EVENT: engine shut down"
                engineshutdown &
                # The legacy script broke out + exited here too. We
                # honour that intent: shutdown is terminal for this
                # invocation, callers can re-launch via systemd.
                wait
                [[ "$ONCE" -eq 1 ]] && cleanup
                cleanup
            fi
            ENGINE_RUNNING=0
            VEHICLE_ON=0
            [[ "$ONCE" -eq 1 ]] && cleanup
        fi
    done

    log "[reconnect] candump pipe ended; retrying in ${CANDUMP_RETRY_DELAY}s"
    sleep "$CANDUMP_RETRY_DELAY"
done

# REVISION NOTES  (2026-05-16)
#     - Original by jmccorm: discovered the $077 state encoding, the
#       hook-function event framework design, and the multi-rate
#       (1m / 10m / 1h) dispatch via SECONDS modulo offsets.
#     - This rewrite preserves jmccorm's hook function names and
#       signatures exactly so anyone with a populated copy of the
#       legacy script can drop their bodies into this version with
#       zero changes. Same goes for the modulo offsets (45 / 599 /
#       35970) -- if you tuned those they survive verbatim.
#     - FIXED: legacy script used decimal `-gt` / `-lt` against
#       hex strings, which crashes whenever the state byte contains
#       any non-decimal hex digit (A-F). Original hard-coded a
#       `5D21 -> 5555` workaround for the one observed case. New
#       version converts via $((16#$state)) so all hex states
#       compare correctly without per-state special-cases.
#     - Added shebang + `set -eu`, SIGINT trap, named state-threshold
#       constants, structured log/debug/raw_log helpers, CLI flags
#       (--once / --debug / --verbose / --help), pre-flight CAN +
#       candump checks, outer reconnect loop on pipe death.
#     - State decode uses a list-based check (is_engine_running)
#       instead of inline OR-chains, so adding new RUNNING states
#       observed on other platforms is a one-array-entry change.
