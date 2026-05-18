#!/bin/bash
#
# autocool.sh -- remote-start HVAC: drive driver-side temperature to LOW,
#                fan to MAX, panel vents on, A/C on, recirculation off.
#
# Originally created:  jmccorm
# Last updated:        05.2026 (polish by magikh0e)
#
# WHAT IT DOES
#   Sequences the JEEP's HVAC controls via $2D3 wake-bus button-press
#   frames and the $342 driver/passenger sync command to land the cabin
#   in "blow hard, cold, fresh air" state.  Designed to run after a
#   remote-start so the cabin is already cooling by the time the driver
#   gets in.  Winter counterpart is autoheat.sh (same shape, opposite
#   temperature direction); a watchdog upstream
#   (Blackbox_monitor.sh / autocollect.sh) decides which one to launch
#   based on ambient temperature when the engine starts.
#
# SEQUENCE (all $2D3 frames are CAN-IHS wake-bus button presses)
#
#   1.  If the blower is off, send the HVAC-on toggle and wait
#       4.1s for the blower to actually spin up.
#   2.  Mute the stereo so HVAC chimes don't compete with whatever
#       the driver was last listening to.
#   3.  Fan to speed 7 (max).
#   4.  Front defroster on -- side effect: clears any active
#       recirculate / A/C state to a known baseline.
#   5.  Re-select panel vents (defrost was just a "reset to baseline"
#       hop, not the destination).
#   6.  Step driver temperature DOWN x 8 (phase 1).
#   7.  A/C on (side effect: also engages recirculate).
#   8.  Step driver temperature DOWN x 18 more (phase 2) -- ends at
#       the LOW stop.  Total 26 down-presses, enough to walk from any
#       starting temp to LOW even if the system clamps at the
#       endpoint.
#   9.  Recirculate off (we want fresh air, not cabin air, on a hot
#       remote-start).
#  10.  Break the passenger-side temperature sync by ratcheting the
#       passenger temp down-then-up.  Some HVAC firmware re-enables
#       driver/passenger sync after an A/C toggle; touching the
#       passenger temp tells the controller "passenger side is
#       independent now".
#  11.  Send the $342 sync command so the controller re-mirrors the
#       driver's setting (LOW) to the passenger side cleanly.
#
# USAGE
#     ./autocool.sh                Run the sequence
#     ./autocool.sh --help         Print this header
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
#       no driver present) -- the HVAC will accept these button
#       presses any time the bus is awake, but the use case is
#       remote-start specifically.
#
# REVISION NOTES  (2026-05-18)
#     - jmccorm's exact cansend payloads + sleep durations preserved
#       VERBATIM (this script is timing-tuned; the working part IS the
#       sequence of presses + waits, not the wrapper logic).
#     - FIXED: single-instance grep was checking for "autoheat" but
#       this is autocool -- so two simultaneous autocools could run
#       and fight each other.  Now checks for either name and
#       excludes self via $$ filter.
#     - FIXED: bare "ventmode 08" call on line ~56 of the legacy
#       script (every other ventmode call used the absolute path);
#       would fail with "command not found" unless /home/pi/bin was
#       in $PATH.  Now uses $VENTMODE_CMD consistently.
#     - FIXED: backtick command substitution `seq 1 N` -- replaced
#       with $(seq 1 N) form (working but obsolete).
#     - REMOVED: unused $STARTED variable (set, never referenced).
#     - Added shebang + set -eu, SIGINT/SIGTERM trap, named
#       constants for byte sequences and sleep durations, pre-flight
#       checks for cansend + the three helper binaries.
#     - Hardcoded /home/pi/bin/* paths now overridable via env vars
#       so the script is portable to non-Pi hosts.
#     - Original preserved as autocool.legacy.txt.
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
# Byte 0 = 0x07 marks this as a button-press packet.  Subsequent bytes
# are button bits -- one bit per physical control on the HVAC panel.
# ---------------------------------------------------------------------

BTN_HVAC_TOGGLE='2D3#0700000000010000'        # byte 4 bit 0 -- HVAC on/off
BTN_TEMP_DOWN_DRIVER='2D3#0700000000080000'   # byte 4 bit 3 -- driver temp down
BTN_AC_TOGGLE='2D3#0700000000000100'          # byte 5 bit 0 -- A/C on/off (also enables recirc)
BTN_RECIRC_TOGGLE='2D3#0700000000000200'      # byte 5 bit 1 -- recirculate on/off
BTN_TEMP_DOWN_PASSENGER='2D3#0700000000200000' # byte 4 bit 5 -- passenger temp down
BTN_TEMP_UP_PASSENGER='2D3#0700000000100000'  # byte 4 bit 4 -- passenger temp up

# Driver/passenger sync command on a different ID.
CMD_HVAC_SYNC='342#0000000400'                # ask controller to mirror driver -> passenger

# ---------------------------------------------------------------------
# Timing constants (seconds, all from jmccorm's tuning).
# ---------------------------------------------------------------------

INITIAL_BLOWER_DELAY=4.1   # let the blower spin up before more presses
MUTE_SETTLE=0.31           # after toggling the stereo mute
DEFROST_SETTLE=1.12        # after switching to defroster mode
STEP_INTERVAL=0.36         # between each temperature step
PHASE_SETTLE=0.6           # after temp-step phase / A/C toggle
PASSENGER_UP_SETTLE=0.63   # after the passenger temp-up sync-break
FINAL_PAUSE=1.3            # before issuing the $342 sync
SYNC_SETTLE=0.61           # after the $342 sync command

# Driver-side temperature step counts.
TEMP_DOWN_PHASE1=8         # before A/C is engaged
TEMP_DOWN_PHASE2=18        # after A/C is engaged -- walk to LOW

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

log() { echo "$(date) AUTOCOOL: $*"; }

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
log "Active VENT $VENTMODE and SPEED $FANSPEED"

# Step 2: mute the stereo so HVAC chimes don't compete with audio.
log "Muting the radio"
"$MUTE_CMD" ON
sleep "$MUTE_SETTLE"

# Step 3: fan to MAX (speed 7).
log "Setting fan to MAX"
"$FANSPEED_CMD" 7

# Step 4: front defrost -- side effect clears any active recirculate /
# A/C state, so we start the rest of the sequence from a known baseline.
log "Clearing modes with front defroster"
"$VENTMODE_CMD" 00
sleep "$DEFROST_SETTLE"

# Step 5: now select the actual destination -- panel vents.
log "Directing air to panel vents"
"$VENTMODE_CMD" 08

# Step 6: driver-side temperature DOWN, phase 1 (before A/C engages).
log "Stepping driver temperature down (phase 1, $TEMP_DOWN_PHASE1 steps)"
for _ in $(seq 1 "$TEMP_DOWN_PHASE1"); do
    cansend "$HVAC_BUS" "$BTN_TEMP_DOWN_DRIVER"
    sleep "$STEP_INTERVAL"
done
sleep "$PHASE_SETTLE"

# Step 7: A/C on.  This ALSO turns on recirculation as a side effect;
# we turn recirc back off after the temperature finishes ratcheting.
log "Turning on A/C system"
cansend "$HVAC_BUS" "$BTN_AC_TOGGLE"
sleep "$PHASE_SETTLE"

# Step 8: driver-side temperature DOWN, phase 2 (18 more steps to LOW).
log "Setting temperature to LOW (phase 2, $TEMP_DOWN_PHASE2 steps)"
for _ in $(seq 1 "$TEMP_DOWN_PHASE2"); do
    cansend "$HVAC_BUS" "$BTN_TEMP_DOWN_DRIVER"
    sleep "$STEP_INTERVAL"
done
sleep "$PHASE_SETTLE"

# Step 9: turn OFF air recirculation.  Came on with the A/C; we want
# fresh outside air pulled through the cabin on a remote-start cool-down.
log "Turning OFF air recirculation"
cansend "$HVAC_BUS" "$BTN_RECIRC_TOGGLE"

# Step 10: break passenger-side temperature sync by walking it down
# then back up.  Some HVAC firmware silently re-enables sync after an
# A/C toggle; this nudge forces the controller to acknowledge that the
# passenger side is independent.
log "Syncing driver/passenger settings"
cansend "$HVAC_BUS" "$BTN_TEMP_DOWN_PASSENGER"
sleep "$STEP_INTERVAL"
cansend "$HVAC_BUS" "$BTN_TEMP_UP_PASSENGER"
sleep "$PASSENGER_UP_SETTLE"

sleep "$FINAL_PAUSE"

# Step 11: issue the $342 sync command so the controller mirrors the
# driver's setting (LOW) to the passenger side cleanly.
cansend "$HVAC_BUS" "$CMD_HVAC_SYNC"
sleep "$SYNC_SETTLE"

log "Exiting normally."
