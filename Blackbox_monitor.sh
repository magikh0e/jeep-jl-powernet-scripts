#!/bin/bash
#
# Blackbox_monitor.sh -- ignition-state event monitor for a Raspberry Pi
# in-vehicle setup. Watches $122 on CAN-IHS, starts a black-box CAN
# recorder when the ignition goes to RUN/START, kills it when the vehicle
# returns to OFF/ACCESSORY, and shifts the CPU governor between
# powersave (idle) and ondemand (running) so the Pi sips power when the
# car is sleeping and has full punch when it's not.
#
# Originally created: 11.2023
# Last updated:       05.2026
#
# Optional companion: an autohvac trigger fires once per RUN transition
# IF the script was already running before the ignition came on (so a
# script restart on a running engine doesn't spuriously kick the HVAC).
#
# USAGE
#     ./Blackbox_monitor.sh [--once] [--no-cpu] [--debug LEVEL]
#                          [--verbose] [--help]
#
# OPTIONS
#     --once          Exit after the first OFF->RUN or RUN->OFF transition
#                     (useful for testing). Default: loop forever.
#     --no-cpu        Skip CPU-governor changes. Use on hosts that
#                     don't expose /sys/devices/system/cpu/cpufreq/.
#     --debug LEVEL   off|on|raw. Default: on.
#                       off  silent except for one-line action notices
#                       on   action + event detail to stderr
#                       raw  also dumps every $122 frame received
#     -v, --verbose   Alias for --debug raw
#     -h, --help      Print this message and exit
#
# REQUIRES
#     - can-utils (candump)             apt install can-utils
#     - $CAN_IHS up at 125 kbps         ip link set $CAN_IHS up type can bitrate 125000
#     - $RECORDER_CMD and $HVAC_CMD     these are external scripts/binaries
#                                       you wire to whatever recorder /
#                                       HVAC integration you use
#
# BACKGROUND
#     Message $122 broadcasts on CAN-IHS every ~100 ms while the
#     vehicle is awake; it carries the virtual ignition switch state.
#     The first 4 bytes encode RUN/ACCESSORY/OFF/START in a way that
#     the script reduces to a single threshold test:
#
#         payload >= 0x04000000    => RUN / START   (engine running)
#         payload <  0x04000000    => OFF / ACCESSORY
#
#     Bit 26 (0x04000000) is the run-state flag. Finer discrimination
#     between OFF / ACCESSORY / RUN / START would require decoding the
#     lower bytes -- not done here because the bit-26 threshold is
#     sufficient for the start/stop automation this script drives.
#
#     Absence of $122 entirely is the sleep-state signal: the vehicle
#     stops sending it when fully asleep, so the candump pipe blocks
#     until the vehicle wakes up.
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-122
#     -- full $122 description, broadcast cadence, sleep semantics.
#
# CAVEAT
#     Tested on JEEP platforms. CAN-IHS message IDs vary by FCA model
#     year -- if $122 doesn't behave as expected on your vehicle,
#     confirm with candump before relying on this for automation.
#
# REVISION NOTES  (2026-05-16)
#     - Added shebang + `set -eu`
#     - Magic threshold 67108863 -> `RUN_THRESHOLD=$((16#04000000))`
#       (named, hex-readable, with comment about bit 26)
#     - Hardcoded /home/pi/bin/dump, /home/pi/bin/autohvac and the
#       sysfs governor path are now overridable via $RECORDER_CMD,
#       $HVAC_CMD, $CPU_FREQ_PATH at the top
#     - Trap for INT/TERM: stops the recorder + restores idle governor
#       before exit (was leaking the recorder on Ctrl+C)
#     - Outer reconnect loop with backoff so the script survives the
#       candump pipe dying (CAN drop, vehicle going deep-sleep, etc.)
#     - Edge-only state transitions via on_run_start / on_run_stop
#       handlers; main loop is now just edge detection
#     - Input validation: skips empty / non-hex payloads instead of
#       crashing $((16#$payload))
#     - DEBUG tiers renamed off|on|raw (was true|false|raw); structured
#       log helpers `log` / `debug` / `raw_log` replace repeated guards
#     - Pre-flight checks for $CAN_IHS interface AND `candump` binary
#     - CLI: --once, --no-cpu, --debug LEVEL, --verbose, --help
#     - set_governor / trigger_hvac no-op gracefully on non-Pi hosts
#       (missing sysfs path, missing $HVAC_CMD binary, etc.)
#     Original preserved as Blackbox_monitor.legacy.txt.

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

CAN_IHS=can0                            # CAN-IHS interface
CAN_ID=122                              # Message ID to watch (ignition state)
WIFI_DEV=wlan0                          # WiFi device (unused here, kept for
                                        # compatibility with legacy script)

RUN_THRESHOLD=$((16#04000000))          # 67108864. payload >= this == RUN/START
                                        # (bit 26 of the big-endian 4-byte head)

CPU_FREQ_PATH=/sys/devices/system/cpu/cpufreq/policy0/scaling_governor
IDLE_GOVERNOR=powersave                 # When vehicle is OFF
RUN_GOVERNOR=ondemand                   # When vehicle is RUN/START

RECORDER_CMD="/home/pi/bin/dump any"    # Black-box recorder command
RECORDER_LOG=/dev/null                  # Redirect recorder stdout/stderr here
HVAC_CMD=/home/pi/bin/autohvac          # Remote-start HVAC trigger (optional)

INITIAL_DELAY=5                         # Settle period before main loop
RUN_TO_DUMP_DELAY=2                     # Wait between starting dump and pgrep
RUN_TO_HVAC_DELAY=5                     # Wait between dump start and HVAC trigger
OFF_TO_KILL_DELAY=2                     # Grace before killing recorder
CANDUMP_RETRY_DELAY=5                   # Backoff between candump pipe restarts

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

DEBUG=on                # off|on|raw
ONCE=0
NO_CPU=0
DUMP_PID=0              # Tracked recorder PID (0 = not running)
LAST_IGNITION=99        # 99 = "not seen yet" sentinel (engine state pre-startup
                        # is unknown, so the first OFF->RUN edge doesn't fire
                        # the HVAC trigger; we don't know if it was already on)
CLEANED=0

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
Blackbox_monitor.sh -- ignition-state event monitor (watches \$$CAN_ID on $CAN_IHS)

USAGE
    $0 [--once] [--no-cpu] [--debug off|on|raw] [--verbose] [--help]

OPTIONS
    --once          Exit after first state transition (for testing)
    --no-cpu        Skip CPU-governor changes
    --debug LEVEL   off|on|raw  (default: on)
    -v, --verbose   Alias for --debug raw
    -h, --help      Print this message

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-122
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --once)    ONCE=1 ;;
        --no-cpu)  NO_CPU=1 ;;
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
            echo "usage: $0 [--once] [--no-cpu] [--debug LEVEL] [--verbose] [--help]" >&2
            exit 2
            ;;
    esac
    shift
done

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

# Logging tiers, all to stderr so stdout stays clean for piping.
log()       { echo "$*" >&2; }
debug()     { [[ "$DEBUG" != "off" ]] && log "$*" || true; }
raw_log()   { [[ "$DEBUG" == "raw" ]] && log "$*" || true; }

# Set the CPU governor. No-op if --no-cpu, or the sysfs node doesn't
# exist (running on a non-Pi host).
set_governor() {
    local gov="$1"
    if [[ "$NO_CPU" -eq 1 ]]; then return 0; fi
    if [[ ! -w "$CPU_FREQ_PATH" ]]; then
        debug "  set_governor: $CPU_FREQ_PATH not writable, skipping"
        return 0
    fi
    debug "  set_governor: $gov"
    echo "$gov" > "$CPU_FREQ_PATH"
}

# Start the recorder. Captures the real PID (after nohup/setsid fork)
# via pgrep so we can kill it cleanly later.
start_recorder() {
    if [[ "$DUMP_PID" -ne 0 ]]; then
        debug "  start_recorder: already running as PID $DUMP_PID, skipping"
        return 0
    fi
    debug "  start_recorder: $RECORDER_CMD > $RECORDER_LOG"
    # shellcheck disable=SC2086
    nohup $RECORDER_CMD > "$RECORDER_LOG" 2>&1 &
    local nohup_pid=$!
    sleep "$RUN_TO_DUMP_DELAY"
    # `nohup` (or whatever wrapper) usually forks once; the real
    # recorder process lives as a child of $nohup_pid. pgrep -P finds
    # the child PID so signals route to the right process on teardown.
    DUMP_PID=$(pgrep -P "$nohup_pid" | head -1)
    if [[ -z "$DUMP_PID" ]]; then
        DUMP_PID="$nohup_pid"
    fi
    log "  DUMP STARTED, PID: $DUMP_PID"
}

# Stop the recorder. Sends TERM to the recorder's process group first,
# then a direct TERM to its PID as a fallback. Tolerates already-dead
# processes (we may have been racing with the recorder exiting on its
# own).
stop_recorder() {
    if [[ "$DUMP_PID" -eq 0 ]]; then
        debug "  stop_recorder: no recorder tracked, skipping"
        return 0
    fi
    sleep "$OFF_TO_KILL_DELAY"
    log "  ENGINE OFF, KILLING DUMP (PID $DUMP_PID)"
    pkill -TERM -P "$DUMP_PID" 2>/dev/null || true
    kill -TERM "$DUMP_PID" 2>/dev/null || true
    DUMP_PID=0
}

# Trigger the HVAC routine. Fire-and-forget; the script doesn't wait
# for it or track its PID. Per the legacy comment: "exits if the
# vehicle was not remote started. Either way, terminates shortly after
# starting and doesn't need monitoring."
trigger_hvac() {
    if [[ ! -x "$HVAC_CMD" ]]; then
        debug "  trigger_hvac: $HVAC_CMD not executable, skipping"
        return 0
    fi
    debug "  trigger_hvac: $HVAC_CMD &"
    "$HVAC_CMD" &
}

# State-transition handlers.
on_run_start() {
    debug ""
    log   "EVENT : VEHICLE HAS BEEN TURNED ON"
    set_governor "$RUN_GOVERNOR"
    start_recorder
    # Only fire HVAC if we OBSERVED the OFF->RUN edge. If LAST_IGNITION
    # is still 99 we never saw the engine in OFF/ACC -- it was already
    # running when the script came up. Don't ghost-fire HVAC in that
    # case.
    if [[ "$LAST_IGNITION" != "99" ]]; then
        sleep "$RUN_TO_HVAC_DELAY"
        log "ACTION: STARTING HVAC AUTOMATION FOR REMOTE-START"
        trigger_hvac
    else
        debug "  skipping HVAC trigger -- LAST_IGNITION was 99 (cold start)"
    fi
}

on_run_stop() {
    debug ""
    log   "EVENT : VEHICLE HAS BEEN TURNED OFF"
    stop_recorder
    set_governor "$IDLE_GOVERNOR"
}

# ---------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------

cleanup() {
    [[ "$CLEANED" -eq 1 ]] && return 0
    CLEANED=1
    log ""
    log "[cleanup] tearing down on signal"
    stop_recorder
    set_governor "$IDLE_GOVERNOR"
    exit 0
}
trap cleanup INT TERM

# ---------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------

if ! ip link show "$CAN_IHS" >/dev/null 2>&1; then
    log "ERROR: CAN interface $CAN_IHS not found"
    log "       Bring it up first, e.g.:"
    log "         ip link set $CAN_IHS up type can bitrate 125000"
    exit 1
fi

if ! command -v candump >/dev/null 2>&1; then
    log "ERROR: candump not found. Install: apt install can-utils"
    exit 1
fi

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

# Initial settle delay then start in IDLE_GOVERNOR. The legacy script
# did this before the main loop too.
sleep 1
set_governor "$IDLE_GOVERNOR"

log "[start] watching \$$CAN_ID on $CAN_IHS (threshold >= 0x$(printf %08X "$RUN_THRESHOLD"))"

# Outer retry loop: if the candump pipe ever dies (CAN interface drops,
# adapter unplugged, vehicle goes deep enough into sleep that the bus
# stops carrying ANY traffic), back off and reconnect instead of
# exiting silently.
while true; do
    candump -L "$CAN_IHS,0${CAN_ID}:0FFFF" 2>/dev/null \
    | while read -r TIME BUS IGNITION_FRAME; do

        raw_log "CAN-IHS data: $IGNITION_FRAME"

        # Parse: extract everything after '#' as the payload, then
        # interpret as a hex integer. Reject empty or non-hex payloads
        # so the int conversion doesn't blow up on a malformed line.
        if [[ "$IGNITION_FRAME" != *"#"* ]]; then
            raw_log "  skipping malformed line (no '#'): $IGNITION_FRAME"
            continue
        fi
        payload="${IGNITION_FRAME#*#}"
        if ! [[ "$payload" =~ ^[0-9A-Fa-f]+$ ]]; then
            raw_log "  skipping non-hex payload: $payload"
            continue
        fi
        IGNITION=$((16#$payload))

        raw_log "  IGNITION=$IGNITION  LAST_IGNITION=$LAST_IGNITION"

        # Edge-trigger on OFF -> RUN
        if [[ $IGNITION -ge $RUN_THRESHOLD ]] \
           && [[ $LAST_IGNITION -lt $RUN_THRESHOLD ]]; then
            on_run_start
            [[ "$ONCE" -eq 1 ]] && cleanup
        fi

        # Edge-trigger on RUN -> OFF
        if [[ $IGNITION -lt $RUN_THRESHOLD ]] \
           && [[ $LAST_IGNITION -ge $RUN_THRESHOLD ]] \
           && [[ $LAST_IGNITION -ne 99 ]]; then
            on_run_stop
            [[ "$ONCE" -eq 1 ]] && cleanup
        fi

        LAST_IGNITION=$IGNITION
    done

    log "[reconnect] candump pipe ended; retrying in ${CANDUMP_RETRY_DELAY}s"
    sleep "$CANDUMP_RETRY_DELAY"
done
