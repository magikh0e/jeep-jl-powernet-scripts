#!/bin/bash
#
# evic.sh -- display arbitrary text on the EVIC music-information page.
#
# Usage:
#   evic.sh LINE "text to display"
#
#     LINE = 1  input name   (top line)
#            2  artist       (middle line)
#            3  title        (bottom line)
#
#   Example:
#     ./evic.sh 2 "magikh0e"
#     ./evic.sh 3 "Pi inside the dash"
#
# Originally created: 01.2022 by jmccorm
# Last updated:       05.2026 (polish by magikh0e)
#
# WHAT IT DOES
#   The JEEP EVIC (Electronic Vehicle Information Center -- the small
#   text region in the instrument cluster between the speedometer and
#   tachometer) renders three lines of metadata for the currently
#   playing audio source.  This script overwrites any of those three
#   lines with a string of your choosing via CAN message $328 on
#   CAN-IHS.  Useful for: ad-hoc dashboard notifications, build-status
#   indicators, "hello world" demos for friends, or just confusing
#   passengers.
#
# MESSAGE FORMAT (observed)
#   The string is sent as a sequence of $328 frames followed by a
#   single all-zero terminator frame:
#
#     328 # <line>0 <seq><func> <ch>  <ch>  <ch>            data frame
#                                \---6 bytes total---/
#     328 # 00 00 00 00 00 00 00 00                          commit frame
#
#   byte 0   high nibble = line-of-string counter, counting DOWN from
#            (total_lines - 1) to 0.  i.e. a string requiring 4 frames
#            ships as 3, 2, 1, 0 in the high nibble.  Low nibble = 0.
#   byte 1   high nibble = 4 for the FIRST frame of a string, 0 for
#                          every continuation frame.
#            low  nibble = function (1 input name, 2 artist, 3 title).
#   bytes 2-7  three characters of payload encoded as UTF-16BE pairs:
#               00 XX 00 YY 00 ZZ  (high byte 0x00, low byte = ASCII).
#            Strings are right-padded with 0x00 0x00 to a multiple of
#            three characters before sending.
#
#   The all-zero terminator commits the new lines to the display.
#   Without it the EVIC keeps showing the previous content.
#
# REQUIRES
#   - can-utils (cansend)                  apt install can-utils
#   - CAN-IHS interface up at 125 kbps     on can0 in this script
#   - vehicle awake                        EVIC won't render with the
#                                          radio module off
#
# REVISION NOTES (2026-05-16)
#   - Fixed lowercase $function in the DEBUG echo (was uninitialised,
#     so debug output silently dropped the function value).
#   - Added LINE-argument validation (must be 1, 2, or 3).
#   - Documented the byte 0 / byte 1 / payload encoding in the header
#     so the message can be reproduced from this script's source.
#   - Quoted $0 in basename for filenames with spaces.

[ "$2" == "" ] && {
  echo "USAGE: $(basename "$0") [line] [text]"
  echo "  Displays [text] on the EVIC music information page."
  echo "  Valid line numbers are:  1 (input name)  2 (artist)  3 (title)"
  echo ""
  exit 1
}

FUNCTION=$1
shift

# Validate LINE / FUNCTION before encoding anything.
case "$FUNCTION" in
  1|2|3) ;;
  *)
    echo "ERROR: line must be 1, 2, or 3 (got '$FUNCTION')"
    exit 1
    ;;
esac

declare -a message
string="$@"
length=${#string}
remainder=$(( ( 3 - ( $length % 3 ) ) % 3 ))

# Encode each character as a UTF-16BE pair: 00 followed by the ASCII
# value of the character in hex.  printf's `'c` trick converts a
# single character to its numeric code point.
POSITION=1
for i in $( seq 1 $length )
do
  message[$POSITION]='00'
  POSITION=$(( $POSITION + 1 ))
  message[$POSITION]=$(printf "%02X" "'${string:$(( i - 1 )):1}'")
  POSITION=$(( $POSITION + 1 ))
done

# Right-pad to a multiple of 3 characters with 00 00 pairs.
for i in $( seq 1 $remainder )
do
  message[$POSITION]=00
  POSITION=$(( $POSITION + 1 ))
  message[$POSITION]=00
  POSITION=$(( $POSITION + 1 ))
  length=$(( $length + 1 ))
done
lines=$(( $length / 3 ))

[ "$DEBUG" == "true" ] && echo "FUNCTION: $FUNCTION"
[ "$DEBUG" == "true" ] && echo "STRING  : $string"
[ "$DEBUG" == "true" ] && echo "LENGTH  : $length characters ($lines lines)"
[ "$DEBUG" == "true" ] && echo "ENCODED : ${message[*]}"

# Send the frames in reverse order: line index counts DOWN from
# (lines - 1) to 0.  First frame uses INIT=4 in byte 1 high nibble,
# continuations use INIT=0.
POSITION=1
INIT=4
for i in $( seq $(( $lines - 1 )) -1 0 )
do
  LINE=$(printf "%x" $i)
  COMMAND="cansend can0 328#${LINE}0.${INIT}${FUNCTION}"
  for j in 1 2 3 4 5 6
  do
    COMMAND="$COMMAND.${message[$POSITION]}"
    POSITION=$(( $POSITION + 1 ))
  done
  $COMMAND
  sleep 0.002
  INIT=0
done

# Terminator frame: all zeros, commits the new lines to the display.
cansend can0 328#00.00.00.00.00.00.00.00
sleep 0.005
