#!/bin/bash
#
# rid.sh -- ReadDataByIdentifier helper.  Sends UDS Service 0x22 to a
#           named module on the JEEP CAN bus, decodes the response,
#           and optionally archives the data on disk for later
#           cross-reference.
#
# Originally created:  jmccorm
# Last updated:        05.2026 (polish by magikh0e)
#
# WHAT IT DOES
#   Translates a friendly module name (bcm, sccm, radio, ipcm, hvac)
#   into the UDS request / response arbitration-ID pair plus the
#   correct CAN bus (CAN-C or CAN-IHS), then issues the standard UDS
#   Service 0x22 ReadDataByIdentifier query for a 16-bit DID via
#   isotpsend / isotprecv (the kernel ISO-TP stack handles all the
#   single-frame / first-frame / consecutive-frame / flow-control
#   framing for you -- no manual cansend + Flow Control like dtc.sh
#   has to do).
#
# OPTIONAL DATA ARCHIVAL
#   On a positive response (service byte 0x62 = 0x22 + 0x40), the
#   data is also written to:
#       $RID_DATA_DIR/<module>/rid/<DID>/<data>
#   The data itself is the *filename*, so a recursive `ls` over the
#   archive directory gives you "everything I've ever seen for this
#   DID on this module" without any database.  If the data string is
#   too long for a filename, it falls back to writing the data into
#   a file named "FILE" instead.  Pairs with ridscan.sh, which sweeps
#   a DID range across a module and accumulates observations.
#
#   Set RID_DATA_DIR= (default: /home/pi/modules) to relocate the
#   archive, or set RID_NO_ARCHIVE=1 to disable archival entirely.
#
# USAGE
#     ./rid.sh <module> <did>
#     ./rid.sh --help
#
#     module:  one of bcm, sccm, radio, ipcm (or evic), hvac
#     did:     1-4 hex chars (zero-padded to 4)
#
# EXAMPLES
#     ./rid.sh bcm F190         Read VIN from BCM
#     ./rid.sh bcm F18C         Read BCM serial number
#     ./rid.sh hvac 0298        Read HVAC current vent mode
#     ./rid.sh ipcm F195        Read IPCM software version
#
# EXIT CODES
#     0    success (positive response printed)
#     1    negative response from module OR DID not supported
#     2    invalid CLI args
#     3    invalid module name
#     99   unknown response shape (neither 0x62 nor 0x7F)
#
# REQUIRES
#     - can-isotp kernel module + can-utils
#         apt install can-utils
#         modprobe can-isotp   (or add to /etc/modules-load.d)
#     - Both CAN buses up with appropriate bitrates:
#         can0 (CAN-IHS) at 125 kbps
#         can1 (CAN-C)   at 500 kbps
#     - Vehicle awake (engine running OR a recent wake event)
#
# REVISION NOTES  (2026-05-18)
#     - jmccorm's exact isotpsend payloads, isotprecv invocations,
#       and per-module (SOURCE, DEST, BUS) values are preserved.
#     - FIXED: duplicate "sccm" lookup block.  Legacy had two
#       `if [ "$MODULE" == "sccm" ]` blocks: first set
#       SOURCE=763 DEST=4E3 BUS=can1, second overwrote with
#       SOURCE=123 DEST=321 BUS=can0.  Per the BMR module-ID catalog
#       and jmccorm's own `# INVALID INFORMATION - NEEDS CORRECTED`
#       comment on the first block, the FIRST (763/4E3 CAN-C) is
#       correct for the Steering Column Control Module.  Second
#       block removed.
#     - Module lookup refactored from chained `if [ "$MODULE" == ...`
#       blocks to a single `case` statement (jmccorm's own comment
#       on the legacy: "we should probably be working with arrays.
#       But this is a product of figuring things out when you go
#       along ... This needs to be revisited.").
#     - `echo $DATA | cat > FILE` collapsed to `echo "$DATA" > FILE`
#       (pointless cat, eight chars removed).
#     - Variables quoted throughout.
#     - /usr/bin/isotpsend, /usr/bin/isotprecv use PATH now instead
#       of hardcoded absolute paths.
#     - RID_DATA_DIR + RID_NO_ARCHIVE env-var overrides so the
#       script is portable to non-Pi hosts and to read-only file
#       systems.
#     - Added shebang + set -eu, --help flag, proper exit codes
#       documented in the header.
#     - Original preserved as rid.legacy.txt.
#

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

: "${RID_DATA_DIR:=/home/pi/modules}"
: "${RID_NO_ARCHIVE:=0}"
RECV_TIMEOUT=8         # seconds to wait for the UDS response

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
Usage: $(basename "$0") <module> <did>
       $(basename "$0") --help

Read a UDS DID by Identifier (Service 0x22) from a named module.

Modules:
    bcm    Body Control Module           (CAN-C  $620 / $504)
    sccm   Steering Column Control Mod   (CAN-C  $763 / $4E3)
    radio  Radio / Head Unit             (CAN-IHS $7BF / $53F)
    ipcm   Instrument Panel Cluster Mod  (CAN-C  $742 / $4C2)
           (alias: evic, ipc)
    hvac   HVAC Control Module           (CAN-IHS $783 / $503)

Examples:
    $(basename "$0") bcm F190        Read VIN
    $(basename "$0") hvac 0298       Read current vent mode

Override RID_DATA_DIR= to relocate the archive (default
/home/pi/modules), or set RID_NO_ARCHIVE=1 to disable archival.
EOF
}

case "${1:-}" in
    -h|--help)
        usage
        exit 0
        ;;
esac

if [[ $# -lt 2 ]]; then
    echo "ERROR: need module and DID arguments" >&2
    usage >&2
    exit 2
fi

MODULE=$(echo "$1" | tr '[:upper:]' '[:lower:]')
shift

# Pre-normalise the DID: pad to 4 hex chars, validate.
INPUT=$(printf "%04X" "0x$1" 2>/dev/null) || true
if [[ ! "$INPUT" =~ ^[0-9A-F]{4}$ ]]; then
    echo "ERROR: '$1' is not a valid hex DID (1-4 chars)" >&2
    exit 2
fi
ID="${INPUT:0:2} ${INPUT:2:2}"

# ---------------------------------------------------------------------
# Module lookup (was a chain of if/then blocks in the legacy version;
# `case` makes the duplicate-sccm bug structurally impossible to
# reintroduce).
# ---------------------------------------------------------------------

# Alias normalisation BEFORE the case match.
case "$MODULE" in
    ipc|evic)  MODULE="ipcm" ;;
esac

case "$MODULE" in
    bcm)
        SOURCE=620
        DEST=504
        BUS=can1   # CAN-C
        ;;
    sccm)
        SOURCE=763
        DEST=4E3
        BUS=can1   # CAN-C
        ;;
    radio)
        SOURCE=7BF
        DEST=53F
        BUS=can0   # CAN-IHS
        ;;
    ipcm)
        SOURCE=742
        DEST=4C2
        BUS=can1   # CAN-C
        ;;
    hvac)
        SOURCE=783
        DEST=503
        BUS=can0   # CAN-IHS
        ;;
    *)
        echo "INVALID MODULE SPECIFIED: $MODULE" >&2
        usage >&2
        exit 3
        ;;
esac

OPERATION=rid

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

# Archive a positive response under $RID_DATA_DIR.  Data is the
# *filename* (so a `find` over the archive shows every distinct value
# ever observed).  Falls back to writing the data into "FILE" if the
# filesystem rejects the filename for being too long.
archive_response() {
    local data="$1"
    [[ "$RID_NO_ARCHIVE" -eq 1 ]] && return

    local dir="$RID_DATA_DIR/$MODULE/$OPERATION/${ID:0:2}${ID:3:2}"
    mkdir -p "$dir"

    # `touch "<long-data>"` will fail with ENAMETOOLONG if data exceeds
    # the filesystem's max filename length.  Catch + fall back.
    if ! touch "$dir/$data" 2>/dev/null; then
        echo "$data" > "$dir/FILE"
    fi
}

# Send the UDS request and capture the response.
issue_rid() {
    # Background the send so we're listening before the response lands.
    ( sleep 0.04
      echo "22 $ID" | isotpsend -s "$SOURCE" -d "$DEST" -p 00:00 -P l "$BUS"
    ) &

    # Capture with timeout.  Some DIDs legitimately take several seconds
    # (multi-frame string returns, internal-eeprom lookups, etc.) so
    # default is 8s.
    RESPONSE=$(timeout -s 1 "$RECV_TIMEOUT" \
        isotprecv -s "$SOURCE" -d "$DEST" -p 00:00 -P l "$BUS" || true)
}

# Parse the response and act on it.  Sets exit code via OK.
handle_response() {
    local data="${RESPONSE:9}"
    local response_service="${RESPONSE:0:2}"
    local positive_service
    positive_service=$(printf "%X" $((0x22 + 0x40)))   # 62

    if [[ "$response_service" == "$positive_service" ]]; then
        echo "SUCCESS"
        echo "$data"
        archive_response "$data"
        OK=0
    elif [[ "$response_service" == "7F" ]]; then
        echo "NEGATIVE RESPONSE"
        echo "$RESPONSE"
        OK=1
    else
        echo "UNKNOWN RESPONSE"
        echo "$RESPONSE"
        OK=99
    fi
}

# ---------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------

for cmd in isotpsend isotprecv; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "ERROR: $cmd not found.  apt install can-utils + modprobe can-isotp" >&2
        exit 2
    fi
done

if ! ip link show "$BUS" >/dev/null 2>&1; then
    echo "ERROR: CAN interface $BUS not found (module $MODULE expects $BUS)" >&2
    exit 2
fi

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

OK=99
issue_rid
handle_response
exit "$OK"
