#!/bin/bash
#
# ridscan.sh -- sweep a range of UDS DIDs against a named module,
#               calling rid.sh once per DID.  Discovers what
#               ReadDataByIdentifier requests the module actually
#               responds to.
#
# Originally created:  jmccorm
# Last updated:        05.2026 (polish by magikh0e)
#
# WHAT IT DOES
#   Calls `rid <module> <did>` for every DID in [start, end] (inclusive,
#   inputs as 1-4 hex chars).  Prints the DID + the rid response for
#   each.  Designed to pair with rid.sh's optional archive mode -- a
#   full scan populates $RID_DATA_DIR/<module>/rid/<did>/<data>
#   directories that can be diff'd later to find which DIDs returned
#   useful values vs which returned negative-response codes.
#
#   Typical use: leave it running overnight on a known DID range
#   you suspect might hold useful configuration data, come back to a
#   harvested directory of observations.
#
# USAGE
#     ./ridscan.sh <module> [start_did] [end_did]
#     ./ridscan.sh --help
#
#     module     one of the names rid.sh recognises (bcm, sccm, radio,
#                ipcm, hvac).
#     start_did  starting 16-bit DID, hex (default: 0000)
#     end_did    ending 16-bit DID, hex   (default: FFFF)
#
# EXAMPLES
#     ./ridscan.sh bcm                  Full 0x0000-0xFFFF scan of BCM
#     ./ridscan.sh bcm F100 F1FF        Scan BCM identification range
#     ./ridscan.sh hvac 0000 02FF       Scan HVAC's configuration range
#
# RATE-LIMITING
#   Set SCAN_DELAY= (default: 0) to insert a sleep between iterations.
#   Useful when you want the scan to run gentle-on-the-bus overnight
#   instead of hammering it.  Each rid call already has its own
#   ISO-TP recv timeout (8s by default in rid.sh), so the inner
#   throttle is mostly about pacing, not avoiding bus congestion.
#
# REQUIRES
#   - rid.sh on PATH (or set RID_CMD=/path/to/rid.sh)
#   - Same dependencies as rid.sh (can-utils + can-isotp kernel module)
#
# REVISION NOTES  (2026-05-18)
#     - jmccorm's outer-loop logic (sweep range + retry-on-99 inner
#       loop) preserved verbatim.
#     - FIXED: retry-on-99 inner loop had no upper bound.  If a DID
#       structurally always returned UNKNOWN RESPONSE (rare but
#       possible for malformed module replies), the legacy script
#       would spin forever.  New version caps retries at $MAX_RETRIES
#       (default 3) per DID.
#     - FIXED: `printf "%d" 0x$1` with empty $1 = "0x" -- a bash
#       integer parse error.  Now validated before printf with a
#       proper hex regex.
#     - FIXED: backtick `` `seq $a $b` `` -> `$(seq "$a" "$b")`.
#     - Replaced positional-arg munging (`$1`, `$2` overlapping with
#       `shift`) with explicit named variables; harder to break.
#     - Added shebang + set -eu, --help flag, env-var-based rate
#       limit (SCAN_DELAY).
#     - Progress output now prints a percentage every 256 DIDs (a
#       full scan is 65,536 iterations; without progress markers
#       it's hard to know if you've sat through a wedged scan).
#     - rid.sh path now overridable via RID_CMD env var.
#     - Original preserved as ridscan.legacy.txt.
#

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

: "${SCAN_DELAY:=0}"        # seconds between iterations (0 = no delay)
: "${MAX_RETRIES:=3}"       # max retries on a single DID returning code 99
: "${RID_CMD:=rid}"         # rid binary / script (must be on PATH or absolute)
PROGRESS_EVERY=256          # print scan progress every N DIDs

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
Usage: $(basename "$0") <module> [start_did] [end_did]
       $(basename "$0") --help

Sweep UDS DIDs against a module via rid.sh.

Module names:
    bcm, sccm, radio, ipcm (alias evic), hvac
    (same set rid.sh recognises)

DID range:
    start_did    hex, 1-4 chars (default: 0000)
    end_did      hex, 1-4 chars (default: FFFF)

Env vars:
    SCAN_DELAY    seconds between iterations  (default: 0)
    MAX_RETRIES   retries on UNKNOWN RESPONSE (default: 3)
    RID_CMD       rid binary path             (default: rid)

Examples:
    $(basename "$0") bcm                     Full sweep of BCM
    $(basename "$0") bcm F100 F1FF           BCM identification range
    SCAN_DELAY=0.1 $(basename "$0") hvac     Throttled HVAC sweep
EOF
}

case "${1:-}" in
    -h|--help|"")
        usage
        exit 0
        ;;
esac

MODULE="$1"
START_HEX="${2:-0000}"
END_HEX="${3:-FFFF}"

# Validate the range arguments as hex.
for arg in "$START_HEX" "$END_HEX"; do
    if ! [[ "$arg" =~ ^[0-9A-Fa-f]{1,4}$ ]]; then
        echo "ERROR: '$arg' is not a valid hex DID (1-4 chars)" >&2
        usage >&2
        exit 2
    fi
done

START=$((16#$START_HEX))
END=$((16#$END_HEX))

if (( START > END )); then
    echo "ERROR: start ($START_HEX) > end ($END_HEX)" >&2
    exit 2
fi

# Pre-flight: rid binary reachable.
if ! command -v "$RID_CMD" >/dev/null 2>&1 && [[ ! -x "$RID_CMD" ]]; then
    echo "ERROR: rid command not found: $RID_CMD" >&2
    echo "       Override path via RID_CMD env var." >&2
    exit 2
fi

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

TOTAL=$((END - START + 1))
echo "START: $START (0x$START_HEX)   END: $END (0x$END_HEX)   TOTAL: $TOTAL DIDs"
echo "MODULE: $MODULE   RID_CMD: $RID_CMD   SCAN_DELAY: ${SCAN_DELAY}s   MAX_RETRIES: $MAX_RETRIES"
echo

count=0
for i in $(seq "$START" "$END"); do
    did_hex=$(printf "%04x" "$i")
    printf "%s ------ " "$did_hex"

    # Call rid; on UNKNOWN RESPONSE (exit 99), retry up to MAX_RETRIES.
    retries=0
    "$RID_CMD" "$MODULE" "$did_hex" || rc=$?
    rc=${rc:-0}
    while [[ "$rc" -eq 99 && "$retries" -lt "$MAX_RETRIES" ]]; do
        echo "REDO ($((retries + 1))/$MAX_RETRIES):"
        printf "%s ------ " "$did_hex"
        "$RID_CMD" "$MODULE" "$did_hex" || rc=$?
        rc=${rc:-0}
        retries=$((retries + 1))
    done
    if [[ "$rc" -eq 99 ]]; then
        echo "GIVE UP: still UNKNOWN after $MAX_RETRIES retries"
    fi

    count=$((count + 1))
    if (( count % PROGRESS_EVERY == 0 )); then
        pct=$(awk "BEGIN { printf \"%.1f\", ($count / $TOTAL) * 100 }")
        echo "  -- progress: $count / $TOTAL  (${pct}%)" >&2
    fi

    # Optional inter-iteration sleep.
    if [[ "$SCAN_DELAY" != "0" ]]; then
        sleep "$SCAN_DELAY"
    fi
done

echo
echo "Scan complete: $count DIDs queried."
