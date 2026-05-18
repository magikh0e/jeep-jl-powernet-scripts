#!/bin/bash
#
# autoheat.sh -- remote-start HVAC: drive driver-side temperature to MAX,
#                fan to MAX, windshield + floor vents, recirculate on,
#                A/C off.
#
# Originally created:  jmccorm
# Last updated:        05.2026 (polish by magikh0e)
#
# WHAT IT DOES
#   Sequences the JEEP's HVAC controls via $2D3 wake-bus button-press
#   frames and the $342 driver/passenger sync command to land the cabin
#   in "blow hard, hot, recirculated air" state.  Designed to run after
#   a remote-start so the cabin is already warming by the time the
#   driver gets in.  Summer counterpart is autocool.sh (same shape,
#   opposite temperature direction); a watchdog upstream
#   (Blackbox_monitor.sh / autocollect.sh) decides which one to launch
#   based on ambient temperature when the engine starts.
#
# SEQUENCE (all $2D3 frames are CAN-IHS wake-bus button presses)
#
#   1.  If the blower is off, send the HVAC-on toggle and wait
#       4.1s for the blower to actually spin up.
#   2.  Mute the stereo.
#   3.  Fan to speed 7 (max).
#   4.  Front defroster on -- side effect: clears any active
#       recirculate / A/C state to a known baseline.
#   5.  Re-select windshield + floor vents (warm air on the
#       windshield to clear frost, warm air at the feet for comfort).
#   6.  Recirculate on (warm cabin air is faster to re-warm than
#       outside winter air).  Side effect: this engages A/C; we
#       turn the A/C back off in step 8.
#   7.  Step driver temperature UP x 8 (phase 1).
#   8.  A/C OFF (clearing the side effect from step 6).
#   9.  Step driver temperature UP x 18 more (phase 2) -- ends at the
#       MAX stop.  Total 26 up-presses, enough to walk from any
#       starting temp to MAX even if the system clamps at the
#       endpoint.
#  10.  Break the passenger-side temperature sync by ratcheting the
#       passenger temp up-then-down.  Some HVAC firmware re-enables
#       driver/passenger sync after an A/C / recirc toggle; touching
#       the passenger temp tells the controller "passenger side is
#       independent now".
#  11.  Send the $342 sync command so the controller re-mirrors the
#       driver's setting (MAX) to the passenger side cleanly.
#
# USAGE
#     ./autoheat.sh                Run the sequence
#     ./autoheat.sh --help         Print this header
#
# OVERRIDABLE ENV VARS  (sensible defaults baked in)
#     WAKE_BUS         CAN-IHS interface name      (default: can0)
#     HVAC_BUS         where $342 sync goes        (default: can0)
#     FANSPEED_CMD     fan-speed helper path       (default: /home/pi/bin/fanspeed)
#     VENTMODE_CMD     vent-mode helper path       (default: /home/pi/bin/ventmode)
#     MUTE_CMD         radio mute helper path      (default: /home/pi/bin/mute)
#
# REQUIRES
#     - can-utils (cansend)                       apt install can-utils
#     - CAN-IHS interface up at 125 kbps:
#         ip link set $WAKE_BUS up type can bitrate 125000
#     - The fanspeed / ventmode / mute helper binaries above
#       (override with env vars if they live elsewhere)
#     - Vehicle in remote-start state (engine running, doors locked,
#       no driver present)
#
# REVISION NOTES  (2026-05-18)
#     - jmccorm's exact cansend payloads + sleep durations preserved
#       VERBATIM (this script is timing-tuned; the working part IS the
#       sequence of presses + waits, not the wrapper logic).
#     - FIXED: single-instance grep was only checking for "autoheat";
#       autocool could still be running and the two would fight each
#       other on the HVAC controls.  Now checks for either name and
#       excludes self via $$ filter.
#     - FIXED: bare "ventmode 02" call (every other ventmode call
#       used the absolute path); would fail with "command not found"
#       unless /home/pi/bin was in $PATH.  Now uses $VENTMODE_CMD
#       consistently.
#     - FIXED: backtick command substitution `seq 1 N` -- replaced
#       with $(seq 1 N) form.
#     - REMOVED: unused $STARTED variable.
#     - Added shebang + set -eu, SIGINT/SIGTERM trap, named
#       constants for byte sequences and sleep durations, pre-flight
#       checks for cansend + the three helper binaries.
#     - Hardcoded /home/pi/bin/* paths now overridable via env vars
#       so the script is portable to non-Pi hosts.
#     - Original preserved as autoheat.legacy.txt.
#

set -eu

# ---------------------------------------------------------------------
# Configuration (override via env vars)
# ---------------------------------------------------------------------

: "${WAKE_BUS:=can0}"
: "${HVAC_BUS:=can0}"
: "${FANSPEED_CMD:=/home/pi/bin/fanspeed}"
: "${VENTMODE_CMD:=/home/pi/bin/ventmode}"
: "${MUTE_CMD:=/home/pi/bin/mute}"

# ---------------------------------------------------------------------
# Wake-bus $2D3 button-press frames (CAN-IHS).
# Format: 2D3#07 NN NN NN BB BB BB BB
# Byte 0 = 0x07 marks this as a button-press packet.
# ---------------------------------------------------------------------

BTN_HVAC_TOGGLE='2D3#0700000000010000'         # byte 4 bit 0 -- HVAC on/off
BTN_TEMP_UP_DRIVER='2D3#0700000000040000'      # byte 4 bit 2 -- driver temp up
BTN_AC_TOGGLE='2D3#0700000000000100'           # byte 5 bit 0 -- A/C on/off
BTN_RECIRC_TOGGLE='2D3#0700000000000200'       # byte 5 bit 1 -- recirculate on/off
BTN_TEMP_UP_PASSENGER='2D3#0700000000100000'   # byte 4 bit 4 -- passenger temp up
BTN_TEMP_DOWN_PASSENGER='2D3#0700000000200000' # byte 4 bit 5 -- passenger temp down

# Driver/passenger sync command on a different ID.
CMD_HVAC_SYNC='342#0000000400'                 # ask controller to mirror driver -> passenger

# ---------------------------------------------------------------------
# Timing constants (seconds, all from jmccorm's tuning).
# ---------------------------------------------------------------------

INITIAL_BLOWER_DELAY=4.1   # let the blower spin up before more presses
MUTE_SETTLE=0.31           # after toggling the stereo mute
DEFROST_SETTLE=1.12        # after switching to defroster mode
VENT_SETTLE=1.12           # after re-selecting windshield + floor vents
RECIRC_SETTLE=1.1          # after engaging recirculation
STEP_INTERVAL=0.36         # between each temperature step
PHASE_SETTLE=0.6           # after temp-step phase / A/C toggle
SYNC_BREAK_SETTLE=1.93     # after the passenger temp-down sync-break
SYNC_SETTLE=1.61           # after the $342 sync command

# Driver-side temperature step counts.
TEMP_UP_PHASE1=8           # before A/C is turned off
TEMP_UP_PHASE2=18          # after A/C is turned off -- walk to MAX

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

log() { echo "$(date) AUTOHEAT: $*"; }

cleanup() {
    log "Interrupted -- exiting partway through the sequence."
    # No reset action here on purpose.  Whatever HVAC state we left
    # the vehicle in is fine; the driver can adjust on entry.
}
trap cleanup INT TERM

# Single-instance check: don't run if autoheat OR autocool is already
# active.  $$ filter excludes our own PID.
SELF="$$"
running=$(ps -eaf \
    | grep -E 'autoheat|autocool' \
    | grep -v grep \
    | grep -v " $SELF " \
    | wc -l)
if [[ $running -gt 0 ]]; then
    log "ERROR - autoheat or autocool already running.  Exiting."
    exit 1
fi

# Pre-flight: cansend on PATH, helper binaries executable.
if ! command -v cansend >/dev/null 2>&1; then
    echo "ERROR: cansend not found.  apt install can-utils" >&2
    exit 2
fi

for helper in "$FANSPEED_CMD" "$VENTMODE_CMD" "$MUTE_CMD"; do
    if [[ ! -x "$helper" ]]; then
        echo "ERROR: helper binary not executable: $helper" >&2
        echo "       Override path via FANSPEED_CMD / VENTMODE_CMD / MUTE_CMD env vars." >&2
        exit 2
    fi
done

# Minimal CLI: --help / -h prints the doc-block.
case "${1:-}" in
    -h|--help)
        awk '/^# autoheat\.sh/,/^$/ { print }' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
esac

# ---------------------------------------------------------------------
# Main sequence
# ---------------------------------------------------------------------

log "Initializing."

# Step 1: if the blower motor is off, wake the HVAC system up first.
FANSPEED=$("$FANSPEED_CMD")
if [[ "$FANSPEED" == "0" ]]; then
    log "Turning on HVAC system"
    cansend "$WAKE_BUS" "$BTN_HVAC_TOGGLE"
    sleep "$INITIAL_BLOWER_DELAY"
fi

# Log current settings (for postmortem / debugging).
VENTMODE=$("$VENTMODE_CMD")
FANSPEED=$("$FANSPEED_CMD")
log "Current VENT: $VENTMODE and SPEED: $FANSPEED"

# Step 2: mute the stereo.
log "Muting the radio"
"$MUTE_CMD" ON
sleep "$MUTE_SETTLE"

# Step 3: fan to MAX (speed 7).
log "Setting fan to MAX"
"$FANSPEED_CMD" 7

# Step 4: front defrost -- side effect clears recirculate / A/C state.
log "Clearing modes via the front defroster"
"$VENTMODE_CMD" 00
sleep "$DEFROST_SETTLE"

# Step 5: windshield + floor (warm windshield to clear frost,
# warm feet for comfort).
log "Directing air to windshield and floor"
"$VENTMODE_CMD" 02
sleep "$VENT_SETTLE"

# Step 6: recirculate on (faster to re-warm cabin air than outside
# winter air).  This also turns A/C on as a side effect; we turn A/C
# back off in step 8.
log "Recirculating air"
cansend "$HVAC_BUS" "$BTN_RECIRC_TOGGLE"
sleep "$RECIRC_SETTLE"

# Step 7: driver-side temperature UP, phase 1 (before A/C is turned off).
log "Stepping driver temperature up (phase 1, $TEMP_UP_PHASE1 steps)"
for _ in $(seq 1 "$TEMP_UP_PHASE1"); do
    cansend "$HVAC_BUS" "$BTN_TEMP_UP_DRIVER"
    sleep "$STEP_INTERVAL"
done
sleep "$PHASE_SETTLE"

# Step 8: turn A/C off (came on with recirculate in step 6; we want
# pure heat, not heat + dehumidification).
log "Turning OFF the AC system"
cansend "$HVAC_BUS" "$BTN_AC_TOGGLE"
sleep "$PHASE_SETTLE"

# Step 9: driver-side temperature UP, phase 2 (18 more steps to MAX).
log "Setting temperature to MAX (phase 2, $TEMP_UP_PHASE2 steps)"
for _ in $(seq 1 "$TEMP_UP_PHASE2"); do
    cansend "$HVAC_BUS" "$BTN_TEMP_UP_DRIVER"
    sleep "$STEP_INTERVAL"
done
sleep "$PHASE_SETTLE"

# Step 10: break passenger-side temperature sync by walking it up
# then back down.  Some HVAC firmware silently re-enables sync after
# an A/C / recirc toggle; this nudge forces the controller to
# acknowledge that the passenger side is independent.
log "Syncing driver/passenger settings"
cansend "$HVAC_BUS" "$BTN_TEMP_UP_PASSENGER"
sleep "$STEP_INTERVAL"
cansend "$HVAC_BUS" "$BTN_TEMP_DOWN_PASSENGER"
sleep "$SYNC_BREAK_SETTLE"

# Step 11: issue the $342 sync command so the controller mirrors the
# driver's setting (MAX) to the passenger side cleanly.
cansend "$HVAC_BUS" "$CMD_HVAC_SYNC"
sleep "$SYNC_SETTLE"

log "Exiting normally."
