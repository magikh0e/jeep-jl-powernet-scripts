#!/bin/bash
#
# battery.sh -- read the JEEP / FCA battery voltage from CAN-IHS message $2C2.
#
# Captures one $2C2 frame from CAN-IHS and decodes byte 2 (the third byte
# of the 8-byte payload) as battery voltage in tenths-of-a-volt. A raw
# 0x78 reads back as 12.0V, 0x86 as 13.4V, etc.
#
# $2C2 is broadcast frequently on CAN-IHS while the vehicle is awake.
# If the bus is asleep (no messages within 2 seconds), this script
# sends a single $2D3 wake frame and retries once before giving up.
#
# DATA SOURCE
#     $2C2 byte 2  (offset 0-indexed from start of payload)
#     decode       value / 10 = volts
#
# See the bus-message-reference for the full $2C2 entry:
#     /pubCarHacking/bus-message-reference.html#id-2c2
# and the $2D3 NM-wake message used by the retry path:
#     /pubCarHacking/bus-message-reference.html#id-2d3
#
# USAGE
#     ./battery.sh
#
# OUTPUT
#     12.4 vdc
#
# REQUIREMENTS
#     can-utils (candump, cansend), dc, timeout, awk-grade coreutils.
#     can0 already configured as the CAN-IHS interface (125 kbps).
#
# EXIT CODES
#     0   read OK, voltage printed on stdout
#     1   no $2C2 message received within timeout, or payload was FF*8
#         (bus disconnected / battery sentinel)
#

error() {
  echo "ERROR: Is the vehicle off? Battery voltage data not found."
  echo "DEBUG: A valid ID 2C2 message was not seen on CAN-IHS within 2 seconds."
  echo " "
  # Exit with a failure result code
  exit 1
}

initialize () {
COMMAND="timeout -s 9 2 /usr/bin/candump -L can0,02C2:0fff"
can2C2=$( $COMMAND | tail -1 )
}

initialize
if [ "$can2C2" == "" ] ; then
  # Wake the CAN bus and try again.
  cansend can0 2D3#0700000000000000 ; sleep 1.1
  initialize
  # Display an error and exit if no messages were received.
  [ "$can2C2" == "" ] && error
fi

if [ "$( echo "$can2C2" | cut -c30-43 )" == "FFFFFFFFFFFFFF" ] ; then error; fi
temp="$( echo "$can2C2" | cut -c34-35 )"
temp="$( printf "%d" 0x$temp )"
temp="$( echo "1 k $temp 10 / p" | dc )"

echo $temp vdc
