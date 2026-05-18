#!/bin/bash
#
# log_reader.sh -- replay a candump log and render decoded vehicle
#                  state as one-line-per-tick text output.
#
# Original recipe: bash style matches the jmccorm-era contributions
#                  on this site; specific author for this exact
#                  script not recorded.
# Updates / polish: magikh0e
# Last updated:    05.2026
#
# Reads a candump log file (the output of `candump -L`), watches a
# curated set of message IDs, and prints one decoded line per
# $UPDATE_RATE microseconds of log time. Useful for replaying a
# recorded drive at human-readable speed without needing the
# vehicle present.
#
# DECODED FIELDS (each line):
#
#     TIME    Timestamp from the log
#     KEY     Ignition state from $122 -- Kill / Acc / Off / Strt /
#             Crnk / RRun / RAcc / Run / Unk
#     BRK     Brake pedal pressure %      ($079 byte 0 nibbles, /22.5)
#     ACCL    Accelerator pedal %         ($07B bytes 0-2, scaled)
#     VALVE   Throttle valve %            ($07B bytes 6-8, alternate;
#                                          enabled by $SHOW_ALTERNATES)
#     RPM     Engine RPM                  ($322 bytes 0-1, raw)
#     WHEEL   Steering angle + direction  ($023 bytes 2-3, ±2048 centre)
#     DIR     Compass heading             ($358 byte 0 low nibble:
#                                          0=N..7=NW, F=unknown)
#     GEAR    PRNDL                       ($340 derived)
#     ODOM    Odometer                    ($3D2 bytes 0-2, * 50/8 / 100)
#     MPH     Vehicle speed               ($322 bytes 2-3, /200)
#
# Plus three user-configurable "flag" IDs ($FLAG1 / $FLAG2 / $FLAG3)
# for ad-hoc exploration -- set the hex ID at the top of the script,
# uncomment the echo lines near the bottom, and the script will
# capture and display the raw payload for each flagged ID on every
# render tick.
#
# USAGE
#     ./log_reader.sh <candump-log-file>
#
#     The log file must be in `candump -L` format:
#     (1638323019.362674) can1 1CE#2B00300B R
#
# REQUIRES
#     - bash, grep, dc, bc (most distros)
#     - A candump -L formatted log file. Capture one with:
#         candump -L can0,can1 > drive.log
#
# REFERENCE
#     Each decoded field's underlying message ID is documented at
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html
#         #id-122   Ignition state (with the named state-code table)
#         #live-data-map (for $023, $322 byte layouts)
#     The $358 / $3D2 / $079 / $07B / $340 / $0AB IDs are decoded by
#     this script but not yet documented in the BMR; see the field
#     comments in this script's source for the byte offsets.
#
# WHAT THE POLISH FIXED VS. THE LEGACY VERSION
#     - BUG: legacy script set TMPDIR=/run/tmpfiles.d, which is
#       systemd's directory for declarative tmpfiles configuration.
#       Writing app state there could conflict with systemd's
#       tmpfiles-clean runs and is arguably wrong. The polished
#       version uses a private mktemp -d under /tmp and cleans up
#       on exit via a trap.
#     - Replaced `cat $1 | egrep PATTERN` with `grep -E PATTERN "$1"`
#       (useless use of cat, plus quoting).
#     - Backticks -> $() throughout.
#     - Added shebang, --help, and usage-on-no-args.
#     - Variables quoted where they could break on unusual input.
#     - Preserved every decode formula, every state code table, and
#       all output formatting exactly so the rendered lines are
#       byte-for-byte identical to the legacy version (modulo the
#       tmpdir path inside the script).
#
# CAVEAT
#     Does NOT use `set -e` -- the legacy script's pattern of
#     `[ test ] && action` for conditional logic exits non-zero on
#     the failure path of every test, which `set -e` would treat as
#     a fatal error and abort the run. Fixing that properly is a
#     larger refactor than this polish pass; for now the script
#     intentionally tolerates non-zero exits from those branches.

set -u

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

# How frequently (in microseconds of log time) to emit a render line.
#    .01 second =    10000
#    .1  second =   100000
#     1  second =  1000000
UPDATE_RATE=100000

# Show alternate / experimental decodes (throttle valve via $07B
# bytes 6-8; alternate DCT gear via $0AB).
SHOW_ALTERNATES=1

# Ad-hoc exploration slots. Set each to a 3-character hex message ID
# and the script will capture that ID's payload on every match. The
# captured value is available via /$TMPDIR/$flagN in the render loop.
# Uncomment the FLAG echo lines near the bottom of the script to
# print them on each rendered line.
flag1="025"
flag2="027"
flag3="02B"

# ---------------------------------------------------------------------
# CLI / arg parsing
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
log_reader.sh -- replay a candump log and render decoded vehicle state

USAGE
    $0 <candump-log-file>

The log file must be in \`candump -L\` format:
    (1638323019.362674) can1 1CE#2B00300B R

Capture one with:
    candump -L can0,can1 > drive.log

DECODED FIELDS
    TIME | KEY | BRK% | ACCL% | (VALVE%) | RPM | WHEEL | DIR | GEAR | ODOM | MPH

See script header comments for which message ID each field comes from.

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html
EOF
    exit 1
}

case "${1:-}" in
    -h|--help|"") usage ;;
esac

LOGFILE="$1"
if [[ ! -r "$LOGFILE" ]]; then
    echo "ERROR: cannot read log file '$LOGFILE'" >&2
    exit 2
fi

# ---------------------------------------------------------------------
# Per-run state directory (private, auto-cleaned)
# ---------------------------------------------------------------------
# The legacy script wrote to /run/tmpfiles.d, which is the systemd-
# managed directory for declarative tmpfiles configuration. That's the
# wrong place for application state; systemd's tmpfiles cleaner could
# nuke files there at any time, and it might interpret malformed entries
# as configuration. Use a private temp dir instead, cleaned up on exit.

TMPDIR=$(mktemp -d /tmp/log_reader.XXXXXX)
trap 'rm -rf "$TMPDIR"' EXIT

# Seed each ID's state file so the first iteration has something to
# read (avoids "empty cat" surprises in the render loop).
echo "0000"             > "$TMPDIR/0AB"
echo "023#FFFF"         > "$TMPDIR/023"
echo "F"                > "$TMPDIR/358"
echo "000001"           > "$TMPDIR/3D2"
echo "00"               > "$TMPDIR/232"
echo "0000000000000000" > "$TMPDIR/$flag1"
echo "0000000000000000" > "$TMPDIR/$flag2"
echo "0000000000000000" > "$TMPDIR/$flag3"
echo "000000000"        > "$TMPDIR/07B"
echo "000"              > "$TMPDIR/079"
echo "000000000000"     > "$TMPDIR/322"

# ---------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------

LATEST=0

grep -E " 023#| 079#| 07B#| 232#| 340#| 358#| 3D2#| 322#| 122#| 0AB#| $flag1#| $flag2#| $flag3#" "$LOGFILE" \
| while read -r a b c; do

case "$c" in
    "$flag1"#*) echo "${c:4:16}" > "$TMPDIR/$flag1" ;;
    "$flag2"#*) echo "${c:4:16}" > "$TMPDIR/$flag2" ;;
    "$flag3"#*) echo "${c:4:16}" > "$TMPDIR/$flag3" ;;
    079#*)      echo "${c:5:9}"  > "$TMPDIR/079" ;;
    07B#*)      echo "${c:5:9}"  > "$TMPDIR/07B" ;;
    232#*)      echo "${c:6:2}"  > "$TMPDIR/232" ;;
    358#*)      echo "${c:5:1}"  > "$TMPDIR/358" ;;
    3D2#*)      echo "${c:4:6}"  > "$TMPDIR/3D2" ;;
    0AB#*)      echo "${c:8:4}"  > "$TMPDIR/0AB" ;;
    122#*)      echo "${c:4:4}"  > "$TMPDIR/122" ;;
    023#*)      echo "$c"        > "$TMPDIR/023" ;;
    322#*)      echo "${c:4:12}" > "$TMPDIR/322" ;;
    340#*)      echo "$c" | cut -c9-10,19,20 > "$TMPDIR/340" ;;
esac

# Reconstruct a microsecond-since-epoch timestamp from the log line.
# Format: (1638323019.362674) so a[1..10] = seconds, a[12..17] = usec.
NOW=${a:1:10}${a:12:6}
[ "$LATEST" -eq 0 ] && LATEST=$(( NOW - 1000 ))
[ "$NOW" -gt "$LATEST" ] && {
    LATEST=$(( LATEST + UPDATE_RATE ))

    trans=$(cat "$TMPDIR/340")
    speed="${trans:2:2}"

    # Timestamp -- HH:MM:SS portion of the log line's date.
    time="${a:6:8}"
    echo -n "$time  "

    # ---- $122 ignition state -----------------------------------------
    # State codes captured from real vehicle traces:
    #   0301/0302 = Kill,   0502/1502 = Acc,  0000/0001 = Off,
    #   4501 = Strt (start), 5D01 = Crnk (crank),
    #   4401 = RRun (remote run), 0402 = Run.
    rawkey=$(cat "$TMPDIR/122")
    key="Unk${rawkey} "
    case "$rawkey" in
        0301) key="Kill" ;;
        0302) key="Kill" ;;
        0502) key="Acc " ;;
        1502) key="Acc " ;;
        0000) key="Off " ;;
        0001) key="Off " ;;
        4501) key="Strt" ;;
        5D01) key="Crnk" ;;
        4401)
            key="RRun"
            [ "${rpm1:-}" == "0.0k" ] && key="RAcc"
            ;;
        0402) key="Run " ;;
    esac
    echo -n "KEY: $key  "

    # ---- $079 brake pedal -------------------------------------------
    # First 3 hex chars of payload -> /22.5 = % pressure.
    BRAKE=$(cat "$TMPDIR/079" | cut -c1-3)
    BRAKE=$(printf "%d" 0x$BRAKE)
    BRAKE=$(echo "0k $BRAKE 22.5 / p" | dc)
    [ "$BRAKE" -gt 100 ] && BRAKE=100
    [ "$BRAKE" -lt 100 ] && BRAKE=" $BRAKE"
    [ "$BRAKE" -lt 10 ]  && BRAKE=" $BRAKE"
    echo -n "BRK:$BRAKE%"

    # ---- $07B accelerator pedal -------------------------------------
    # Two interpretations from different byte ranges:
    #   ACCEL  = bytes 0-2 (pedal position, scaled around 1990..3990)
    #   ACCEL2 = bytes 6-8 (throttle valve %, /18 from offset 2000)
    ACC=$(cat "$TMPDIR/07B")
    ACCEL=$(printf "%d" 0x${ACC:0:3})
    ACCEL2=$(printf "%d" 0x${ACC:6:3})
    [ "$ACCEL2" -lt 2000 ] && ACCEL2=2000
    ACCEL2=$(echo "0k $ACCEL2 2000 - 18 / p" | dc)
    ACCEL2=$(printf "%2d" $ACCEL2)
    [ "$ACCEL" -lt 1900 ] && ACCEL=1900
    ACCEL=$(echo "0k $ACCEL 1990 - 100 * 2000 / p" | dc)
    [ "$ACCEL" -lt 0 ]   && ACCEL=0
    [ "$ACCEL" -lt 100 ] && ACCEL=" $ACCEL"
    [ "$ACCEL" -lt 10 ]  && ACCEL=" $ACCEL"
    echo -n "  ACCL:$ACCEL% "
    [ "$SHOW_ALTERNATES" -eq 1 ] && echo -n "(VALVE $ACCEL2%) "

    # ---- $322 engine RPM (bytes 0-1) --------------------------------
    rpm1=$(cat "$TMPDIR/322" | cut -c1-4)
    rpm1=$(printf "%d" 0x$rpm1)
    [ "$rpm1" = 65535 ] && rpm1="0"
    printf " RPM: %4d  " "$rpm1"

    # ---- $023 steering wheel angle ---------------------------------
    # 16-bit BE with 0x1000 zero offset; >$1000 = left of centre.
    STEER=$(cat "$TMPDIR/023" | cut -c5-8)
    STEER=$(printf "%d" 0x$STEER)
    STEERSIGN="R "
    [[ "$STEER" -gt 4096 ]] && STEERSIGN="L "
    STEER=$(echo "4096 $STEER -p" | dc | cut -d- -f2)
    STEER=$(echo "2k $STEER 2 / p" | dc | cut -d. -f1)
    STEER=$(printf "%3d\n" "${STEER}")
    SYMBOL="°"
    [ "$STEER" -lt 2 ]    && STEERSIGN="  " && STEER="  0"
    [ "$STEER" -gt 1000 ] && STEERSIGN="IN" && STEER="VAL" && SYMBOL="D"
    echo -n "WHEEL: ${STEERSIGN}${STEER}${SYMBOL}  "

    # ---- $340 transmission state (derived speed + gear) ------------
    [ "$speed" == "FF" ] && speed="00"
    speed=$(printf "%d" 0x$speed)

    gear=$(echo $trans | cut -c2)
    [ "$gear" == "F" ] && gear="NA"
    [ "$gear" == "B" ] && gear="R " && speed="-$speed"
    [ "$gear" == "D" ] && gear="P "
    if [[ "$gear" =~ ^[1-9]+$ ]]; then
        gear="D$gear"
    fi
    [ "$gear" == "0" ] && gear="N "

    # ---- $322 vehicle speed (bytes 2-3, /200) ----------------------
    mph=$(cat "$TMPDIR/322" | cut -c5-8)
    mph=$(printf "%d" 0x$mph)
    [ "$mph" == 65535 ] && mph="0"
    [ "$gear" == "R " ] && mph="-$mph"
    mph=$(echo "2 k $mph 200 / p" | dc)
    mph=$(printf "%2.2f" $mph)

    # ---- $358 compass heading (byte 0 low nibble, 0..7 + F) --------
    compass=$(cat "$TMPDIR/358")
    case "$compass" in
        F) compass="??" ;;
        0) compass="N " ;;
        1) compass="NE" ;;
        2) compass=" E" ;;
        3) compass="SE" ;;
        4) compass="S " ;;
        5) compass="SW" ;;
        6) compass=" W" ;;
        7) compass="NW" ;;
    esac
    echo -n "DIR: $compass "

    echo -n " GEAR: $gear  "

    ## # Alternate gear display (double-clutched automatic transmission?)
    ## [ "$SHOW_ALTERNATES" -eq 1 ] && echo -n "($(cat $TMPDIR/0AB))  "

    # ---- $3D2 odometer (bytes 0-2, * 50/8 / 100 = miles) -----------
    echo -n "ODOM: "
    odometer=$(cat "$TMPDIR/3D2")
    odometer=$(printf "%d" 0x$odometer)
    odometer=$(echo " $odometer * 50 / 8 " | bc)
    if [ "$odometer" == "6" ]; then
        odometer="0"
        echo -n "??????.?mi  "
    else
        printf '%8.1f' "$(echo "$odometer / 100" | bc -l)"
        echo -n "mi  "
    fi

    printf "MPH: %5s  " "$mph"

    # ---- Ad-hoc exploration slots ----------------------------------
    # Uncomment to display the captured raw payload for each flag ID.
    # Set $flag1 / $flag2 / $flag3 at the top of this script to point
    # at the message IDs you want to spy on, then use these lines to
    # render their values alongside the rest of the per-tick output.

    # echo -n "FLAG1: $(cat $TMPDIR/$flag1)  "
    # echo -n "FLAG2: $(cat $TMPDIR/$flag2)  "
    # echo -n "FLAG3: $(cat $TMPDIR/$flag3)  "

    echo ""
}

done

# REVISION NOTES  (2026-05-16)
#     - BUG FIX: legacy script set TMPDIR=/run/tmpfiles.d, the
#       systemd-managed declarative-tmpfiles directory. Wrong place
#       for app state. Polished version uses mktemp -d in /tmp/ and
#       cleans up on exit via EXIT trap.
#     - Replaced `cat $1 | egrep PATTERN` with `grep -E PATTERN "$1"`.
#     - Backticks -> $() throughout.
#     - Quoted $LOGFILE, $TMPDIR, and other vars that could break on
#       unusual input.
#     - Added --help and usage-on-no-args; failures on unreadable
#       log file now exit 2 with a clear error message.
#     - Decode logic, all message-ID parsing, state-code tables, and
#       output formatting preserved verbatim. Render-line output is
#       byte-for-byte identical to the legacy version (modulo the
#       internal tmpdir path).
#     - Did NOT add `set -e`: the legacy script's `[ test ] && action`
#       idiom exits non-zero on every test failure, which would abort
#       the run under -e. Properly converting all of those to `if/fi`
#       would be a substantial refactor; left for a future pass.
