#!/bin/bash
#
# Remote_WiFi.sh -- toggle the Raspberry Pi's WiFi radio from the fob.
#
# Originally created: 01.2022 (both auto and toggle modes are from
#                              jmccorm's original 2022 work; they were
#                              two coexisting variants of the same idea)
# Last updated:       05.2026
#
# Listens for $1C0 (Remote Lock / Unlock) on CAN-IHS and uses fob
# button events to control the Pi's WiFi radio. Two operating modes
# are supported via --mode:
#
#   --mode auto    (default)
#       Lock          -> WiFi DOWN  (interface goes offline)
#       2nd Unlock    -> WiFi UP    (interface comes back online)
#       1st Unlock    -> no-op      (single unlock leaves WiFi off)
#       Idle          -> no-op      (just bookkeeping)
#
#       Effect: WiFi is silent any time the vehicle is locked. The
#       double-tap unlock is the trigger to bring it back up,
#       avoiding access-point broadcasts every time you walk past
#       the car. Locked = secure, double-unlock = convenience.
#
#   --mode toggle
#       Lock          -> no-op      (lock does NOT touch WiFi)
#       2nd Unlock    -> TOGGLE     (flip WiFi state -- off-to-on or
#                                    on-to-off, alternating each press)
#       1st Unlock    -> no-op      (still need the double-tap)
#       Idle          -> no-op
#
#       Effect: WiFi state is independent of lock state. The
#       double-tap unlock is a single-button latch for WiFi on/off.
#       Useful when you don't want a "lock and walk away" to kill
#       your SSH session, or when you want manual control over
#       when the radio is broadcasting.
#
#   Both modes are from jmccorm's original 2022 work -- they were two
#   coexisting variants of the same idea, packaged here behind one
#   --mode switch.
#
# USAGE
#     ./Remote_WiFi.sh [--mode auto|toggle] [--once] [--dry-run]
#                      [--debug LEVEL] [--verbose] [--help]
#
# OPTIONS
#     --mode MODE     auto|toggle  (default: auto)
#                       auto    lock disables WiFi, 2nd-unlock enables
#                       toggle  lock no-op, 2nd-unlock flips WiFi state
#     --once          Exit after first event handled (useful for testing)
#     --dry-run       Log everything but DON'T touch the WiFi interface
#     --debug LEVEL   off|on|raw  (default: on)
#                       off  silent except action notices
#                       on   action + event detail to stderr
#                       raw  also dumps every $1C0 frame received
#     -v, --verbose   Alias for --debug raw
#     -h, --help      Print this message and exit
#
# REQUIRES
#     - can-utils (candump)     apt install can-utils
#     - iproute2 (ip)           usually preinstalled
#     - $CAN_IHS up at 125 kbps ip link set $CAN_IHS up type can bitrate 125000
#     - $WIFI_DEV               check with: ip link show
#
# PAYLOAD MAP  (observed on the JEEP platform this script was developed
#               against; bytes vary by year/model -- see BMR caveats)
#
#     1C0#21 00 00 90 00 00      Lock         byte0=0x21  byte3=0x90 active
#     1C0#23 00 00 90 00 00      1st Unlock   byte0=0x23  byte3=0x90 active
#     1C0#24 00 00 90 00 00      2nd Unlock   byte0=0x24  byte3=0x90 active
#     1C0#00 00 00 80 00 00      Idle         byte0=0x00  byte3=0x80 idle
#
#     byte 0 = command code (0x21/0x23/0x24/0x00)
#     byte 3 = active/idle flag (0x90 active, 0x80 idle)
#     other bytes appear to carry fob ID / button-hold metadata --
#     not positively identified on this platform.
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-1c0
#     -- full $1C0 description with observed payloads + caveats.
#
# PLATFORM NOTE
#     The four payloads above are JEEP-specific. FCA reassigns CAN-IHS
#     message IDs aggressively between platforms (and even between
#     model years on the same platform). On a non-JEEP FCA vehicle,
#     verify with candump while pressing fob buttons before trusting
#     these byte patterns.
#
# CAVEAT
#     If the script dies (panic, kill -9, host reboot) while the radio
#     is down, the radio stays down. On a fresh boot, systemd's
#     network setup brings $WIFI_DEV back up; if you're running this
#     under systemd it should self-heal on restart. Otherwise: bring
#     it back manually with: ip link set $WIFI_DEV up
#
# REVISION NOTES  (2026-05-16)
#     - Added shebang + `set -eu`
#     - Deprecated `ifconfig` / `iwconfig` replaced with `ip link`
#       (iproute2, universally installed on modern distros)
#     - Magic 1C0 payloads extracted to named constants: $PAYLOAD_LOCK,
#       $PAYLOAD_UNLOCK_1, $PAYLOAD_UNLOCK_2, $PAYLOAD_IDLE
#     - Four parallel `if` blocks collapsed to a single `case`
#       statement dispatching to on_lock / on_unlock_1 / on_unlock_2 /
#       on_idle handlers
#     - Edge filter: act only when the payload CHANGES (idle messages
#       broadcast at ~100ms cadence; the legacy LASTREMOTE sentinel
#       used a literal that didn't match the actual payload format)
#     - **Unknown payloads logged** as candidates for new handlers --
#       handy when porting to a different FCA platform
#     - SIGINT/SIGTERM trap that deliberately does NOT bounce WiFi on
#       exit (if you last locked the car, the radio should stay down)
#     - Outer reconnect loop with backoff on candump pipe death
#     - `[ $X == "..." ]` -> `[[ ]]` throughout, with proper quoting
#     - Strict frame validation: only $CAN_ID frames reach dispatch
#     - Pre-flight: $CAN_IHS, $WIFI_DEV, and `candump` binary all
#       checked before main loop
#     - CLI: --once, --dry-run, --debug LEVEL, --verbose, --help
#     Original preserved as Remote_WiFi.legacy.txt.
#
# REVISION NOTES  (2026-05-16, --mode addition)
#     - Added --mode auto|toggle.  Both modes are from jmccorm's
#       original 2022 work; previously this script implemented only
#       the auto behaviour, with the toggle variant living in a
#       separate file.  Default is auto so existing systemd units
#       and cron jobs keep their previous behaviour.
#     - Toggle mode introduces a LOCKED_STATE bookkeeping variable;
#       the 2nd-unlock handler flips it and dispatches wifi_up /
#       wifi_down accordingly. Lock handler becomes a debug-only
#       no-op in toggle mode.
#     - Toggle-mode default initial state is LOCKED_STATE=0 ("WiFi
#       currently on"), matching jmccorm's variant. First 2nd-unlock
#       press therefore turns WiFi OFF.
#     - Both toggle-mode and auto-mode dispatch use `ip link` instead
#       of the original `iwconfig` / `ifconfig`.

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

CAN_IHS=can0                            # CAN-IHS interface
CAN_ID=1C0                              # Message ID to watch (RKE / fob)
WIFI_DEV=wlan0                          # WiFi interface to toggle

# Payload codes (everything after '#' in the candump line). Matched
# verbatim; case matters since candump emits uppercase hex.
PAYLOAD_LOCK=210000900000               # Lock button
PAYLOAD_UNLOCK_1=230000900000           # 1st unlock button (single press)
PAYLOAD_UNLOCK_2=240000900000           # 2nd unlock button (double tap)
PAYLOAD_IDLE=000000800000               # Idle / no recent command

# Toggle delays (legacy script's incantation -- empirically these
# delays were necessary to avoid race conditions with the WiFi stack).
WIFI_UP_SETTLE=4                        # After bringing the link up,
                                        # before further config
INITIAL_DELAY=5                         # Settle period before main loop
CANDUMP_RETRY_DELAY=5                   # Backoff between pipe restarts

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

DEBUG=on
ONCE=0
DRY_RUN=0
MODE=auto                               # auto | toggle  (see header MODES)
LAST_PAYLOAD="__init__"                 # Sentinel so the first observed
                                        # frame always counts as an edge
LOCKED_STATE=0                          # Toggle-mode bookkeeping: 0 ==
                                        # "WiFi currently on", 1 == "off".
                                        # In auto mode this is updated but
                                        # not consulted for dispatch.
CLEANED=0

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
Remote_WiFi.sh -- toggle WiFi from the fob via \$$CAN_ID on $CAN_IHS

USAGE
    $0 [--mode auto|toggle] [--once] [--dry-run]
       [--debug off|on|raw] [--verbose] [--help]

OPTIONS
    --mode MODE     auto|toggle  (default: auto)
                      auto    lock disables WiFi, 2nd-unlock enables
                      toggle  lock no-op, 2nd-unlock flips WiFi state
    --once          Exit after first event handled (testing)
    --dry-run       Log actions but don't touch the WiFi interface
    --debug LEVEL   off|on|raw  (default: on)
    -v, --verbose   Alias for --debug raw
    -h, --help      Print this message

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-1c0
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)
            shift
            case "${1:-}" in
                auto|toggle) MODE="$1" ;;
                *) echo "ERROR: --mode must be auto or toggle" >&2; exit 2 ;;
            esac
            ;;
        --once)    ONCE=1 ;;
        --dry-run) DRY_RUN=1 ;;
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
            echo "usage: $0 [--mode auto|toggle] [--once] [--dry-run] [--debug LEVEL] [--verbose] [--help]" >&2
            exit 2
            ;;
    esac
    shift
done

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

log()       { echo "$*" >&2; }
debug()     { [[ "$DEBUG" != "off" ]] && log "$*" || true; }
raw_log()   { [[ "$DEBUG" == "raw" ]] && log "$*" || true; }

# Bring WiFi up/down. dry-run skips the actual `ip` call but still logs
# so you can verify event handling without disconnecting yourself
# during testing.
wifi_down() {
    if [[ "$DRY_RUN" -eq 1 ]]; then
        debug "  [dry-run] would: ip link set $WIFI_DEV down"
        return 0
    fi
    debug "  ip link set $WIFI_DEV down"
    ip link set "$WIFI_DEV" down 2>/dev/null || \
        log "  warn: could not bring $WIFI_DEV down (already down? permissions?)"
}

wifi_up() {
    if [[ "$DRY_RUN" -eq 1 ]]; then
        debug "  [dry-run] would: ip link set $WIFI_DEV up; sleep ${WIFI_UP_SETTLE}s"
        return 0
    fi
    debug "  ip link set $WIFI_DEV up"
    ip link set "$WIFI_DEV" up 2>/dev/null || \
        log "  warn: could not bring $WIFI_DEV up (no such device? permissions?)"
    sleep "$WIFI_UP_SETTLE"
}

# Event handlers. One per observed fob command. Each fires on the EDGE
# only -- repeats of the same payload (e.g. idle messages broadcast at
# 100ms cadence) are filtered out by the LAST_REMOTE check in main().
# Handlers dispatch on $MODE for the two operating modes:
#   auto    -- lock disables WiFi, 2nd-unlock enables
#   toggle  -- lock no-op, 2nd-unlock flips WiFi state
on_lock() {
    case "$MODE" in
        auto)
            log "EVENT : KEYFOB LOCK COMMAND RECEIVED"
            log "ACTION: TURNING OFF WIFI DEVICE ($WIFI_DEV)"
            wifi_down
            LOCKED_STATE=1
            ;;
        toggle)
            debug "EVENT : KEYFOB LOCK COMMAND RECEIVED"
            debug "ACTION: NONE (toggle mode -- WiFi only changes on 2nd unlock)"
            ;;
    esac
}

on_unlock_1() {
    debug "EVENT : KEYFOB 1ST UNLOCK COMMAND RECEIVED"
    debug "ACTION: NONE (waiting for 2nd unlock)"
}

on_unlock_2() {
    case "$MODE" in
        auto)
            log "EVENT : KEYFOB 2ND UNLOCK COMMAND RECEIVED"
            log "ACTION: TURNING ON WIFI DEVICE ($WIFI_DEV)"
            wifi_up
            LOCKED_STATE=0
            ;;
        toggle)
            log "EVENT : KEYFOB 2ND UNLOCK COMMAND RECEIVED"
            # Flip the locked-state bookkeeping and act on the new value.
            LOCKED_STATE=$(( 1 - LOCKED_STATE ))
            if [[ "$LOCKED_STATE" -eq 0 ]]; then
                log "ACTION: TURNING ON WIFI DEVICE ($WIFI_DEV)"
                wifi_up
            else
                log "ACTION: TURNING OFF WIFI DEVICE ($WIFI_DEV)"
                wifi_down
            fi
            ;;
    esac
}

on_idle() {
    debug "EVENT : IDLE STATE (default) -- no action"
}

# ---------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------

cleanup() {
    [[ "$CLEANED" -eq 1 ]] && return 0
    CLEANED=1
    log ""
    log "[cleanup] tearing down on signal"
    # NB: we deliberately do NOT touch the WiFi state here. If the user
    # last locked the car, the radio is down by design -- bringing it
    # up on script exit defeats the point. Restore manually if needed:
    #   ip link set $WIFI_DEV up
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

if ! ip link show "$WIFI_DEV" >/dev/null 2>&1; then
    log "ERROR: WiFi interface $WIFI_DEV not found"
    log "       Check available interfaces with: ip link show"
    exit 1
fi

if ! command -v candump >/dev/null 2>&1; then
    log "ERROR: candump not found. Install: apt install can-utils"
    exit 1
fi

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

sleep "$INITIAL_DELAY"

log "[start] watching \$$CAN_ID on $CAN_IHS for fob events  (mode: $MODE)"
[[ "$DRY_RUN" -eq 1 ]] && log "[start] DRY RUN -- WiFi interface will NOT be touched"

# Outer retry loop: if candump dies (CAN interface drops, vehicle goes
# deep enough into sleep that the bus is silent), back off and reconnect
# instead of exiting silently.
while true; do
    candump -L "$CAN_IHS,0${CAN_ID}:0FFFF" 2>/dev/null \
    | while read -r TIME BUS FRAME; do

        raw_log "CAN-IHS data: $FRAME"

        # Skip malformed lines. Strict format: $ID#$PAYLOAD where the
        # payload is uppercase hex. candump -L emits this consistently
        # but a torn read from a dying interface could leak garbage.
        if [[ "$FRAME" != "${CAN_ID}#"* ]]; then
            raw_log "  skipping: not a \$$CAN_ID frame: $FRAME"
            continue
        fi
        payload="${FRAME#*#}"

        # Edge filter: act only when the payload changes. Idle messages
        # repeat at ~100ms cadence; without this we'd flood the log.
        if [[ "$payload" == "$LAST_PAYLOAD" ]]; then
            continue
        fi

        # Dispatch.
        case "$payload" in
            "$PAYLOAD_LOCK")     on_lock ;;
            "$PAYLOAD_UNLOCK_1") on_unlock_1 ;;
            "$PAYLOAD_UNLOCK_2") on_unlock_2 ;;
            "$PAYLOAD_IDLE")     on_idle ;;
            *)
                # Unknown payload. Log it -- this is useful when
                # porting to a different platform; new payloads here
                # are candidates for new event handlers.
                debug "EVENT : UNKNOWN \$$CAN_ID payload: $payload"
                ;;
        esac

        LAST_PAYLOAD="$payload"

        [[ "$ONCE" -eq 1 ]] && cleanup
    done

    log "[reconnect] candump pipe ended; retrying in ${CANDUMP_RETRY_DELAY}s"
    sleep "$CANDUMP_RETRY_DELAY"
done
