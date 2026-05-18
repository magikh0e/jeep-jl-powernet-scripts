#!/bin/bash
#
# ventmode.sh -- read or set the HVAC vent-mode (which vents the blower
#                directs air to) via UDS on CAN-IHS.
#
# Originally created:  jmccorm
# Last updated:        05.2026 (polish by magikh0e)
#
# WHAT IT DOES
#   READ  -- with no argument, queries the HVAC Module at $783 / $503
#            (CAN-IHS UDS request / response pair) for Service 0x22
#            ReadDataByIdentifier on DID $0298 (current vent-mode
#            selection).  Prints the 1-byte mode ID on stdout.
#
#   WRITE -- with a 2-char hex argument, walks the HVAC controls until
#            the read-back mode equals the requested mode.  Three
#            modes have direct-jump buttons (Auto $0F, Defrost $00,
#            cycle for the rest); the cycle button is mashed until the
#            module reports the desired state.
#
# VENT MODE IDS
#     00   Defrost only
#     02   Defrost + Feet (mix)
#     04   Feet only
#     06   Panel + Floor (mix)
#     08   Panel only
#     0F   Auto (vents + fan controlled by the HVAC module's logic)
#
# USAGE
#     ./ventmode.sh                 Read and print current vent mode
#     ./ventmode.sh 08              Set vent mode to Panel only
#     ./ventmode.sh -h | --help     Print this header
#     ./ventmode.sh -v 08           Verbose / debug mode
#
# UDS REQUEST/RESPONSE FORMAT
#     Request   can0 783 # 03 22 02 98 00 00 00 00
#                              \  /  \  /
#                               |     |
#                               |     +-- DID $0298 (vent-mode)
#                               +-------- Service 0x22 ReadDataByIdentifier
#
#     Response  can0 503 # 04 62 02 98 <ID> 00 00 00
#                              \  /  \  /  \--/
#                               |     |     |
#                               |     |     +-- 1-byte vent mode ID
#                               |     +-------- DID echo
#                               +-------------- 0x62 = 0x22 + 0x40 (positive response)
#
# OVERRIDABLE ENV VARS
#     IHS_BUS          CAN-IHS interface       (default: can0)
#     HVAC_REQ_ID      UDS request arb ID      (default: 783)
#     HVAC_RES_ID      UDS response arb ID     (default: 503)
#     MAXTRIES         max cycle-button presses to reach target  (default: 6)
#
# EXIT CODES
#     0   success (mode read or successfully written)
#     1   read error (no response from HVAC module after retries)
#     2   write error (couldn't reach target mode within MAXTRIES)
#     3   bad CLI argument (invalid mode ID)
#
# REQUIRES
#     - can-utils (cansend, candump)              apt install can-utils
#     - CAN-IHS interface up at 125 kbps:
#         ip link set $IHS_BUS up type can bitrate 125000
#     - Vehicle awake (engine running OR a recent wake event on CAN-IHS)
#
# REVISION NOTES  (2026-05-18)
#     - jmccorm's exact UDS request payload, cansend button frames,
#       and DELAY escalation (0.6 -> 1.2 -> 1.8) preserved VERBATIM.
#     - Variables now quoted ("$DELAY", "$VENTID", etc.) so weird
#       inputs don't break the loop.
#     - set -eu added.
#     - --help / -h and --verbose / -v CLI flags added; previous
#       -H / HELP behaviour still recognised for compatibility.
#     - Error messages now go to stderr (was stdout, which made it
#       hard to chain with `$(./ventmode.sh)`).
#     - grep pattern uses a literal-string match ('-F') instead of
#       backslash-escaping the '#'.
#     - Bad mode IDs ($VENTWANTED == "XX" path when arg was non-empty
#       but not 2 chars) now report a usage error to stderr with
#       exit 3 instead of silently falling into the read-only path.
#     - DELAY is a local in request_hvac_vent() now; escalation
#       happens via the caller passing larger values.  Avoids the
#       legacy script's silent global-DELAY mutation that affected
#       the final cansend / sleep at the bottom of the loop too.
#     - Original preserved as ventmode.legacy.txt.
#

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

: "${IHS_BUS:=can0}"
: "${HVAC_REQ_ID:=783}"
: "${HVAC_RES_ID:=503}"
: "${MAXTRIES:=6}"

# UDS Service 0x22 ReadDataByIdentifier request for DID $0298.
# PCI length = 3 (service byte + 2 DID bytes), padded with 0x00.
UDS_READ_VENT_REQ='0322029800000000'

# Positive response prefix: 04 62 02 98 ... (length=4, 0x62=0x22+0x40, DID echo).
UDS_READ_VENT_RES_PREFIX='#04620298'

# Direct-jump buttons (CAN-IHS wake-bus $2D3 button presses).
BTN_AUTO='2D3#0700000000020000'
BTN_DEFROST='2D3#0700000000800000'
BTN_CYCLE_MODE='2D3#0700000000000800'

# Default delays (seconds) for read-window escalation.
DELAY_FAST=0.6
DELAY_SLOW=1.2
DELAY_SLOWEST=1.8

# ---------------------------------------------------------------------
# CLI parsing
# ---------------------------------------------------------------------

VERBOSE=0
VENTWANTED=""

usage() {
    cat <<EOF
Usage: $(basename "$0") [vent_id] [-v|--verbose] [-h|--help]

Read or set the HVAC vent mode via UDS on CAN-IHS.

Vent IDs:
    00   Defrost only
    02   Defrost + Feet (mix)
    04   Feet only
    06   Panel + Floor (mix)
    08   Panel only
    0F   Auto

With no vent_id, reads and prints the current mode.
With a vent_id, walks the HVAC controls until the module reports
that mode.

Override IHS_BUS / HVAC_REQ_ID / HVAC_RES_ID / MAXTRIES via env vars.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help|-H|help|HELP)
            usage
            exit 0
            ;;
        -v|--verbose)
            VERBOSE=1
            ;;
        --)
            shift
            break
            ;;
        -*)
            echo "ERROR: unknown flag: $1" >&2
            usage >&2
            exit 3
            ;;
        *)
            if [[ -z "$VENTWANTED" ]]; then
                VENTWANTED=$(echo "$1" | tr '[:lower:]' '[:upper:]')
            else
                echo "ERROR: unexpected positional arg: $1" >&2
                exit 3
            fi
            ;;
    esac
    shift
done

# Validate the vent ID if one was given.
if [[ -n "$VENTWANTED" ]] && ! [[ "$VENTWANTED" =~ ^[0-9A-F]{2}$ ]]; then
    echo "ERROR: '$VENTWANTED' is not a valid 2-character hex vent ID" >&2
    usage >&2
    exit 3
fi

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

log() { [[ $VERBOSE -eq 1 ]] && echo "ventmode: $*" >&2 || true; }

read_error() {
    echo "FAILURE: Could not read the vent mode from the HVAC Module." >&2
    exit 1
}

write_error() {
    echo "FAILURE: Could not change the vent mode in the HVAC Module." >&2
    exit 2
}

# Query the HVAC module's current vent mode.  Sets RESPONSE in caller
# scope to either the 2-char hex mode ID or empty string on timeout.
request_hvac_vent() {
    local delay="$1"

    # Fire the UDS request after a brief delay so candump is ready.
    ( sleep 0.1; cansend "$IHS_BUS" "${HVAC_REQ_ID}#${UDS_READ_VENT_REQ}" ) &

    # Capture responses on $HVAC_RES_ID for $delay seconds, extract the
    # vent-mode byte (cut chars 9-10 = bytes 4-5 of the hex payload,
    # which is the position of the mode ID after stripping the 04 62 02 98
    # header).  -F grep for the literal positive-response prefix.
    RESPONSE=$(
        timeout -s 1 "$delay" candump -L "${IHS_BUS},${HVAC_RES_ID}:0FFF" 2>/dev/null \
            | grep -F "$UDS_READ_VENT_RES_PREFIX" \
            | cut -d# -f2 \
            | cut -c9-10 \
            | tail -1
    )
}

# ---------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------

if ! ip link show "$IHS_BUS" >/dev/null 2>&1; then
    echo "ERROR: CAN interface $IHS_BUS not found" >&2
    echo "       Bring it up: ip link set $IHS_BUS up type can bitrate 125000" >&2
    exit 2
fi

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

VENTID="99"
TRIES=0

while [[ "$VENTID" != "$VENTWANTED" ]]; do

    # Read current vent mode.  Escalate the timeout window if the
    # first attempt comes back empty.
    request_hvac_vent "$DELAY_FAST"
    if [[ -z "$RESPONSE" ]]; then
        log "no response at ${DELAY_FAST}s -- retrying with ${DELAY_SLOW}s"
        request_hvac_vent "$DELAY_SLOW"
    fi
    if [[ -z "$RESPONSE" ]]; then
        log "no response at ${DELAY_SLOW}s -- retrying with ${DELAY_SLOWEST}s"
        request_hvac_vent "$DELAY_SLOWEST"
        if [[ -z "$RESPONSE" ]]; then read_error; fi
    fi

    VENTID="$RESPONSE"
    log "current vent mode: $VENTID   target: ${VENTWANTED:-<read-only>}"

    # Read-only mode: no argument given, just print and exit.
    if [[ -z "$VENTWANTED" ]]; then
        echo "$VENTID"
        exit 0
    fi

    # If we're already at the target, no work to do.
    if [[ "$VENTID" == "$VENTWANTED" ]]; then
        break
    fi

    # Press the appropriate button to move the HVAC module toward the
    # target.  Auto and Defrost have direct-jump buttons; everything
    # else cycles through the mode list.
    case "$VENTWANTED" in
        0F)
            log "pressing Auto button"
            cansend "$IHS_BUS" "$BTN_AUTO"
            sleep "$DELAY_FAST"
            exit 0
            ;;
        00)
            log "pressing Defrost button"
            cansend "$IHS_BUS" "$BTN_DEFROST"
            sleep "$DELAY_FAST"
            exit 0
            ;;
        *)
            log "cycling HVAC mode button (TRIES=$((TRIES + 1))/$MAXTRIES)"
            cansend "$IHS_BUS" "$BTN_CYCLE_MODE"
            sleep "$DELAY_FAST"
            ;;
    esac

    TRIES=$((TRIES + 1))
    if [[ "$TRIES" -gt "$MAXTRIES" ]]; then
        write_error
    fi

done

exit 0
