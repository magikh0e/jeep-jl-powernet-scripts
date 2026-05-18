#!/bin/bash
#
# obd.sh -- query an OBD-II PID and print the response bytes.
#
# Original author:   jmccorm
# Updates / polish:  magikh0e
# Last updated:      05.2026
#
# Sends OBD-II Service 0x01 (ShowCurrentData) with a user-provided
# Parameter ID (PID) and prints the response data bytes as
# space-separated decimal values. The engine must be running for
# most PIDs to return useful data; some (like 0x42 ControlModule
# Voltage) work with just the ignition on.
#
# OBD-II vs UDS  --  This is the standardized cousin of the UDS
# scripts on this site (3rd_brakelight.sh, horn.sh, 2k.sh, etc).
# Both ride on CAN-C and both talk to the ECM via the $7E0 / $7E8
# arbitration-ID pair, but they use different service sets:
#
#     OBD-II (SAE J1979 / ISO 15031):
#       Service 0x01 ShowCurrentData       <-- this script
#       Service 0x03 ShowDTCs
#       Service 0x09 RequestVehicleInfo   (VIN, cal ID)
#       (always read-only, no session unlock needed)
#
#     UDS (ISO 14229):
#       Service 0x10 DiagnosticSessionControl
#       Service 0x22 ReadDataByIdentifier
#       Service 0x2F IOControlByIdentifier  (writes)
#       Service 0x31 RoutineControl         (writes)
#       (writes need extendedDiagnosticSession unlock first)
#
# Positive response convention is the same across both protocols:
# response service byte = request service byte + 0x40. So 0x01
# requests yield 0x41 responses; the PID is echoed in the next
# byte, and the data follows.
#
# USAGE
#     ./obd.sh <PID>  [--wait SEC] [--retry-wait SEC] [--verbose] [--help]
#
#     PID is a 1- or 2-character hex value (e.g. 0C, c, 42, 1F).
#     Single-char input is zero-padded automatically.
#
# OPTIONS
#     --wait SEC       Initial response timeout (default: 0.2)
#     --retry-wait SEC Fallback timeout if first attempt times out
#                      (default: 0.5)
#     -v, --verbose    Echo raw frames captured + decode steps to stderr
#     -h, --help       Print this message and exit
#
# OUTPUT
#     A space-separated list of decimal byte values, one byte per
#     OBD-II response data byte (after the 0x41 service-echo and
#     the PID-echo are stripped). Apply the formula listed in
#     the OBD-II PID spec to convert the bytes into engineering
#     units. See https://en.wikipedia.org/wiki/OBD-II_PIDs.
#
# EXIT CODES
#     0   Success -- decimal bytes printed.
#     1   No response after both attempts.
#     2   Bad usage (no PID given, PID not hex, etc).
#
# EXAMPLES
#     Engine RPM ($0C). Formula: ((A*256)+B)/4
#         $ ./obd.sh 0c
#         17 184
#         ((17*256)+184)/4 = (4352+184)/4 = 1134 RPM
#
#     Intake Air Temperature ($0F). Formula: A-40
#         $ ./obd.sh F
#         78
#         78-40 = 38 degC
#
#     Coolant temperature ($05). Formula: A-40
#     Vehicle speed ($0D). Formula: A (km/h)
#     MAF rate ($10). Formula: ((A*256)+B)/100
#     Throttle position ($11). Formula: A * 100/255 (%)
#
# REQUIRES
#     - can-utils (cansend, candump)    apt install can-utils
#     - CAN-C interface up at 500 kbps:
#         ip link set $CAN_C up type can bitrate 500000
#     - Engine running (or at least ignition on for some PIDs)
#
# SECURE GATEWAY MODULE NOTE
#     OBD-II Mode 01 reads pass through the SGW on 2018+ FCA
#     vehicles WITHOUT needing AutoAuth -- they're read-only and
#     standardized, so the SGW lets them through. Writes (Service
#     0x04 ClearDTCs and the UDS write services) are still gated.
#     See https://magikh0e.pl/pubCarHacking/secure-gateway-module.html
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#obd-ii
#     https://en.wikipedia.org/wiki/OBD-II_PIDs    (full PID list + formulas)
#     https://magikh0e.pl/pubCarHacking/jscan-uds-intro.html  (the UDS cousin)
#
# REVISION NOTES
#     - Original by jmccorm: the request/response flow with two-stage
#       retry, single-frame vs first-frame PCI handling, and the
#       "return raw decimal bytes, let the caller apply OBD-II
#       formulas" output convention.
#     - This rewrite preserves the exact CLI surface (positional PID
#       argument, single-char zero-padding) and the two-stage retry
#       cadence. Improvements:
#         * Removed several ineffective `2>/dev/null` redirects on
#           variable assignments (bash syntax error in spirit if not
#           in fact -- you can't redirect output of a plain `=`).
#         * Replaced `LENGTH=$(printf "%d" 0x$LENGTH)` with
#           `LENGTH=$((16#$LENGTH))` -- saves a fork, more idiomatic.
#         * Replaced `fold | paste` byte splitting with bash
#           parameter expansion.
#         * Added shebang + `set -eu`, --wait / --retry-wait / --verbose
#           / --help CLI flags, pre-flight CAN check.
#         * Fixed "REQUIIRES" typo in original.

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

CAN_C=can1                    # CAN-C interface (OBD-II ECM lives here)
ECU_REQ_ID=7E0                # OBD-II ECM request arbitration ID
ECU_RES_ID=7E8                # OBD-II ECM response arbitration ID
OBD_MODE=01                   # Service 0x01 ShowCurrentData

DEFAULT_WAIT=0.2              # Initial response capture window
DEFAULT_RETRY_WAIT=0.5        # Fallback window on first-attempt miss

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

PID=""
VERBOSE=0
WAIT="$DEFAULT_WAIT"
RETRY_WAIT="$DEFAULT_RETRY_WAIT"

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
obd.sh -- query an OBD-II PID and print the response bytes

USAGE
    $0 <PID> [--wait SEC] [--retry-wait SEC] [--verbose] [--help]

    PID is a 1- or 2-character hex value (e.g. 0C, c, 42, 1F).

OPTIONS
    --wait SEC          Initial response timeout (default: $DEFAULT_WAIT)
    --retry-wait SEC    Fallback timeout (default: $DEFAULT_RETRY_WAIT)
    -v, --verbose       Echo raw decode steps to stderr
    -h, --help          Print this message

EXAMPLES
    $0 0c    Engine RPM      formula ((A*256)+B)/4
    $0 0f    Intake air temp formula A-40 (deg C)
    $0 05    Coolant temp    formula A-40 (deg C)
    $0 0d    Vehicle speed   formula A (km/h)
    $0 42    Module voltage  formula ((A*256)+B)/1000 (V)

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html#obd-ii
    https://en.wikipedia.org/wiki/OBD-II_PIDs
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --wait)
            shift; WAIT="${1:-}"
            ;;
        --retry-wait)
            shift; RETRY_WAIT="${1:-}"
            ;;
        -v|--verbose) VERBOSE=1 ;;
        -h|--help)    usage ;;
        --) shift; break ;;
        -*)
            echo "unknown arg: $1" >&2
            echo "usage: $0 <PID> [--wait SEC] [--retry-wait SEC] [--verbose] [--help]" >&2
            exit 2
            ;;
        *)
            if [[ -z "$PID" ]]; then
                PID="$1"
            else
                echo "unexpected positional arg: $1" >&2
                exit 2
            fi
            ;;
    esac
    shift
done

if [[ -z "$PID" ]]; then
    usage
fi

# Normalise PID: uppercase, zero-pad single digit, validate hex.
PID="${PID^^}"                          # uppercase
[[ ${#PID} -eq 1 ]] && PID="0$PID"      # zero-pad
if ! [[ "$PID" =~ ^[0-9A-F]{2}$ ]]; then
    echo "ERROR: '$PID' is not a valid 2-character hex value" >&2
    exit 2
fi

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

log()       { [[ $VERBOSE -eq 1 ]] && echo "$@" >&2 || true; }

# Send Service 0x01 request for the configured $PID and capture the
# response within $1 seconds. Returns the data bytes (as a hex string)
# on stdout, or empty string on no response. Caller decides retry.
query_pid() {
    local wait="$1"

    # Fire the request after a 0.1s delay so we're already listening
    # by the time the ECM replies.
    ( sleep 0.1
      cansend "$CAN_C" "${ECU_REQ_ID}#02${OBD_MODE}${PID}0000000000"
    ) &

    # Capture matching responses on $ECU_RES_ID until $wait seconds
    # elapse. Match both single-frame ($##41PID) and first-frame
    # ($10##41PID) responses; take the LAST match if multiple arrive
    # (later one is more likely to be ours).
    local response
    response=$(
        timeout -s TERM "$wait" candump -L "$CAN_C,${ECU_RES_ID}:0FFF" 2>/dev/null \
        | grep -E "#..41${PID}|#10..41${PID}" \
        | cut -d# -f2 \
        | tail -1
    )

    [[ -z "$response" ]] && { echo ""; return; }

    log "  raw response : $response"

    # PCI byte = first 2 hex chars of the payload.
    local pci="${response:0:2}"
    local pci_dec=$((16#$pci))
    local data length

    if [[ "$pci" == "10" ]]; then
        # First Frame of a multi-frame response. The 12-bit length
        # field spans the low nibble of byte 0 and all of byte 1; we
        # take byte 1 only because nibble of byte 0 is always 0 for
        # OBD-II responses we care about. Data follows from byte 2.
        length=$((16#${response:2:2}))
        data="${response:4}"
        log "  first-frame  : length=$length bytes, data after FF header"
    else
        # Single Frame. byte 0 = PCI length.
        length=$pci_dec
        data="${response:2}"
        log "  single-frame : length=$length bytes"
    fi

    # Strip the leading "41 PID" (2 bytes = 4 hex chars) to get just
    # the data payload. Then trim to (length - 2) data bytes since
    # the PCI length includes the service byte and PID echo.
    local payload_bytes=$((length - 2))
    if (( payload_bytes <= 0 )); then
        echo ""
        return
    fi
    data="${data:4:$((payload_bytes * 2))}"

    log "  data bytes   : $data ($payload_bytes byte(s))"
    echo "$data"
}

# Print a hex string as space-separated decimal byte values.
hex_to_decimal_bytes() {
    local hex="$1" i out=""
    for ((i = 0; i < ${#hex}; i += 2)); do
        local byte="${hex:i:2}"
        out+="$((16#$byte)) "
    done
    echo "${out% }"   # trim trailing space
}

# ---------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------

if ! ip link show "$CAN_C" >/dev/null 2>&1; then
    echo "ERROR: CAN interface $CAN_C not found" >&2
    echo "       Bring it up: ip link set $CAN_C up type can bitrate 500000" >&2
    exit 2
fi

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

log "query: Service 0x${OBD_MODE} PID 0x${PID} on $CAN_C ($ECU_REQ_ID -> $ECU_RES_ID)"
log "  attempt 1 (wait ${WAIT}s)"
data=$(query_pid "$WAIT")

if [[ -z "$data" ]]; then
    log "  attempt 2 (wait ${RETRY_WAIT}s)"
    data=$(query_pid "$RETRY_WAIT")
fi

if [[ -z "$data" ]]; then
    echo "NO RESPONSE" >&2
    exit 1
fi

hex_to_decimal_bytes "$data"
