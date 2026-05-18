#!/bin/bash
#
# mute.sh -- read or set the radio mute state via CAN broadcast.
#
# Usage:
#   mute.sh           query current mute state (prints ON / OFF / UNKNOWN)
#   mute.sh on        mute the radio if not already muted
#   mute.sh off       unmute the radio if currently muted
#
# Originally created: 01.2022 by jmccorm
# Last updated:       05.2026 (polish by magikh0e)
#
# MECHANISM
#   READ  -- listens for $25D broadcasts on CAN-C, extracts byte 3
#            (zero-indexed) of the payload.  0x00 = OFF, 0x03 = ON.
#   WRITE -- sends the steering-wheel "MUTE pressed" signal on the
#            wake bus ($2D3 with byte 2 = 0x01).  The radio toggles
#            its mute state on receipt.  We re-read $25D to verify
#            and toggle again if the state still isn't what we
#            wanted, up to MAXTRIES total toggles.
#
# BUS QUIET HANDLING
#   If the first read sees no $25D traffic at all we assume the bus
#   is asleep, send an NM wake frame ($2D3 with byte 2 = 0x00), and
#   retry with a longer collect window.  A second silent retry uses
#   an even longer window before giving up.
#
# REQUIRES
#   - can-utils (candump, cansend)         apt install can-utils
#   - timeout (coreutils)                  usually preinstalled
#   - CAN-C interface up at 500 kbps       on can1 in this script
#   - wake bus reachable on can0           for the NM wake frame
#
# REVISION NOTES (2026-05-16)
#   - Inlined the wake-bus call (was: /home/pi/bin/wake) so the
#     script is self-contained and portable across hosts.  The
#     external helper that jmccorm's original called is now on the
#     site as wake.sh -- this script's wake_bus() function is the
#     same logic minus the random-nibble flourish.
#   - Documented the byte-offset arithmetic in request_mute().
#   - Quoted "$1" and "$0" in basename / case branches.
#   - Removed stale "FIVE times" retry comment; the cap is MAXTRIES.

usage() {
  echo ""
  echo "USAGE: $(basename "$0") [on/off]"
  echo "       (no argument -- prints current mute state)"
  echo ""
  exit 1
}

TRIES=0
MAXTRIES=3
DEBUG=false
DELAY=1.1
MUTEID=99

MUTEWANTED=$(echo "$1" | tr '[:lower:]' '[:upper:]')
[ "$MUTEWANTED" == "" ]    && MUTEWANTED=XX
[ "$MUTEWANTED" == "ON" ]  && MUTEWANTED=03
[ "$MUTEWANTED" == "OFF" ] && MUTEWANTED=00
[ "$MUTEWANTED" != "00" ] && [ "$MUTEWANTED" != "03" ] && [ "$MUTEWANTED" != "XX" ] && usage

wake_bus() {
  # NM wake frame.  $2D3 with byte 2 = 0x00 is the standard
  # "everyone wake up" signal; byte 2 = 0x01 (sent below) is the
  # mute-button press the radio listens for.
  cansend can0 2D3#0700000000000000
}

read_error() {
  echo "FAILURE: Could not read the mute status."
  echo ""
  exit 1
}

write_error() {
  echo "FAILURE: Could not change the mute status."
  echo ""
  exit 1
}

request_mute () {
  # Collect $25D broadcasts on CAN-C for $DELAY seconds, take the
  # last frame, and return its byte 3 (zero-indexed) as a hex string.
  # candump -L line format:  (ts) can1 25D#0011223344556677
  #   cut -d# -f2     ->  0011223344556677
  #   cut -c7-8       ->  "33"  (chars 7-8 are byte 3 zero-indexed)
  # Empty string if the bus is silent.
  COMMAND="timeout -s 1 $DELAY /usr/bin/candump -L can1,025D:0FFF"
  RESPONSE=$( $COMMAND | cut -d# -f2 | cut -c7-8 | tail -1)
}

# MAIN PROGRAM LOOP BEGINS HERE -------------------------------------------
# Read the mute status.  If it isn't what we want, hit the mute button.

while [ "$MUTEID" != "$MUTEWANTED" ]
  do

    # See what our mute status is (no actual request, just listening)
    request_mute

    # If no response, the bus might be asleep -- wake and retry.
    if [ "$RESPONSE" == "" ] ; then
      DELAY=1.6
      wake_bus
      request_mute
    fi

    # If still no response, listen longer one more time.
    if [ "$RESPONSE" == "" ] ; then
      DELAY=2.2
      request_mute
      [ "$RESPONSE" == "" ] && read_error
    fi

    MUTEID=$RESPONSE

    # Query-only mode: print and exit.
    [ "$MUTEWANTED" == "XX" ] && {
      [ "$MUTEID" == "03" ] && { echo ON; exit 0; }
      [ "$MUTEID" == "00" ] && { echo OFF; exit 0; }
      echo UNKNOWN ; exit 1
    }

    # If the mute status isn't what we want, hit the mute toggle button.
    [ "$DEBUG" == "true" ] && echo "MUTEID: $MUTEID   MUTEWANTED: $MUTEWANTED"
    [ "$MUTEID" != "$MUTEWANTED" ] && {
      [ "$DEBUG" == "true" ] && echo TOGGLING MUTE
      cansend can0 2D3#0700010000000000 ; sleep 0.21
    }

    # Limit toggle attempts.
    TRIES=$(( $TRIES + 1 ))
    [ "$TRIES" -gt $MAXTRIES ] && write_error

  done

# A clean exit.  Either we reported a value, made a change, or the
# requested state was already in effect.
exit 0
