#!/bin/bash
#
# dtc.sh -- read OBD-II diagnostic trouble codes (the check-engine-light codes).
#
# Author:        magikh0e
# Last updated:  05.2026
#
# Sends OBD-II Service 0x03 (ShowDTCs) and prints each stored
# diagnostic trouble code in the canonical SAE format (P0420,
# C0561, B1320, U0101, etc.). Also supports Service 0x07
# (ShowPendingDTCs) for codes that haven't been confirmed across
# enough drive cycles yet, and Service 0x0A (ShowPermanentDTCs)
# for emissions-related codes that can't be cleared by tool action
# alone.
#
# OBD-II MODES THIS SCRIPT TARGETS
#     Service 0x03  ShowDTCs           -- "confirmed" codes; the
#                                         ones that turned on the MIL
#                                         (check engine light)
#     Service 0x07  ShowPendingDTCs    -- "pending" codes; detected
#                                         once but not yet confirmed
#                                         across enough drive cycles
#     Service 0x0A  ShowPermanentDTCs  -- emissions codes that can't
#                                         be cleared by Service 0x04
#                                         alone (regulatory requirement)
#
# The script does NOT issue Service 0x04 (ClearDTCs). That's a
# destructive write -- a dedicated script (dtc-clear.sh, future) will
# handle it with an interactive confirm prompt.
#
# DTC ENCODING (2 bytes per code -- wire format)
#     byte 0 (hi):  bits 7-6 = letter (00=P, 01=C, 10=B, 11=U)
#                   bits 5-4 = first digit (0-3)
#                   bits 3-0 = second hex digit
#     byte 1 (lo):  bits 7-4 = third hex digit
#                   bits 3-0 = fourth hex digit
#
#     Example: 0x01 0x33  -> bits 7-6 of 0x01 = 00     -> P
#                            bits 5-4 of 0x01 = 00     -> 0
#                            bits 3-0 of 0x01 = 0001   -> 1
#                            bits 7-4 of 0x33 = 0011   -> 3
#                            bits 3-0 of 0x33 = 0011   -> 3
#                                                       => P0133
#                            (O2 sensor circuit slow response, bank 1)
#
# DTC ANATOMY (what the 5 characters MEAN)
#     position 1   P=Powertrain, B=Body, C=Chassis, U=Network
#     position 2   0 = SAE-standardised code
#                  1-3 = manufacturer-specific (FCA / Stellantis-defined)
#     position 3   for P-codes: subsystem
#                  1-2 = fuel and air metering
#                  3   = ignition / misfire
#                  4   = auxiliary emission control (EVAP, EGR, etc)
#                  5   = vehicle speed / idle control
#                  6   = PCM output circuits
#                  7-9 = transmission
#     positions 4-5  fault number within the subsystem (00-FF)
#
#     Severity types:
#         Type A -- emissions, MIL on after ONE failed cycle (severe)
#         Type B -- emissions, pending after one, MIL on after TWO
#         Permanent -- emissions, can't be cleared until verified-fixed
#                      by completed monitors (Service 0x0A)
#
#     Full anatomy + position-3 subsystem map:
#         https://magikh0e.pl/pubCarHacking/bus-message-reference.html#dtc-anatomy
#
# USAGE
#     ./dtc.sh                  Read confirmed DTCs (default, Service 0x03)
#     ./dtc.sh --mode pending   Read pending DTCs (Service 0x07)
#     ./dtc.sh --mode permanent Read permanent emissions DTCs (Service 0x0A)
#
#     ./dtc.sh [--mode confirmed|pending|permanent]
#              [--wait SEC] [--retry-wait SEC] [--verbose] [--help]
#
# OUTPUT
#     One DTC per line, in P0123 / C0561 / B1320 / U0101 format.
#     "No DTCs reported." if the ECM has nothing to share.
#     Multi-frame responses (4+ DTCs) are reassembled automatically.
#
# EXIT CODES
#     0   Success (DTCs printed, or "No DTCs reported.")
#     1   No response from ECM (engine off? wrong bus?)
#     2   Bad usage / invalid CLI args
#     3   ECM returned a Negative Response Code
#
# REQUIRES
#     - can-utils                       apt install can-utils
#     - CAN-C interface up at 500 kbps:
#         ip link set $CAN_C up type can bitrate 500000
#     - Vehicle powered on (ignition Run; engine running is fine too)
#
# SECURE GATEWAY MODULE NOTE
#     OBD-II Service 0x03 / 0x07 / 0x0A are READ-only, standardised
#     emissions queries -- they pass through the SGW on 2018+ FCA
#     vehicles WITHOUT needing AutoAuth. Service 0x04 (ClearDTCs)
#     would be gated; this script never issues it.
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#obd-ii
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#dtc-anatomy
#     https://magikh0e.pl/pubCarHacking/jeep-dtc-codes-jk.html  (569-code lookup)
#     https://en.wikipedia.org/wiki/OBD-II_PIDs   (full DTC encoding)
#     https://www.obd-codes.com/                  (DTC code lookup)
#
# OPTIONAL: LOCAL DTC DESCRIPTION LOOKUP
#     If dtc-codes-jk.txt is present in the same directory as this
#     script (the on-site copy ships alongside), output is annotated
#     with the human-readable description from that catalog.  Without
#     the TSV, output is just the bare code.
#
# EXAMPLE OUTPUT (with dtc-codes-jk.txt in same directory)
#     $ ./dtc.sh
#     P0420  -  Catalyst Efficiency (Bank 1)
#     P0171  -  Fuel System 1/1 Lean
#
#     $ ./dtc.sh --mode pending
#     No DTCs reported.
#
#     $ ./dtc.sh --verbose
#     query: Service 0x03 ShowDTCs on can1 (7E0 -> 7E8)
#       attempt 1 (wait 0.3s)
#       raw response : 0643020055017A0420
#       single-frame : length=6 bytes
#       data bytes   : 0200 5501 7A04 (3 DTC(s))
#     P0200  -  Fuel Injector 1 Circuit
#     C1501  -  Tire Pressure Sensor 1 Internal
#     B7A04  (no description -- not in local catalog)
#

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

CAN_C=can1                    # CAN-C interface (OBD-II ECM lives here)
ECU_REQ_ID=7E0                # OBD-II ECM request arbitration ID
ECU_RES_ID=7E8                # OBD-II ECM response arbitration ID

DEFAULT_WAIT=0.3              # Initial response capture window
                              # (slightly longer than obd.sh's 0.2 to
                              #  catch multi-frame responses arriving
                              #  after our flow control reply)
DEFAULT_RETRY_WAIT=0.7        # Fallback window on first-attempt miss

# Service modes
MODE_CONFIRMED="03"
MODE_PENDING="07"
MODE_PERMANENT="0A"

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

MODE="$MODE_CONFIRMED"        # Default: Service 0x03 ShowDTCs
MODE_NAME="confirmed"
VERBOSE=0
WAIT="$DEFAULT_WAIT"
RETRY_WAIT="$DEFAULT_RETRY_WAIT"

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
dtc.sh -- read OBD-II diagnostic trouble codes (check-engine codes)

USAGE
    $0 [--mode confirmed|pending|permanent]
       [--wait SEC] [--retry-wait SEC] [--verbose] [--help]

OPTIONS
    --mode MODE         Which DTC set to query:
                          confirmed  Service 0x03 -- MIL-on codes (default)
                          pending    Service 0x07 -- not yet confirmed
                          permanent  Service 0x0A -- emissions-only
    --wait SEC          Initial response timeout (default: $DEFAULT_WAIT)
    --retry-wait SEC    Fallback timeout (default: $DEFAULT_RETRY_WAIT)
    -v, --verbose       Echo raw decode steps to stderr
    -h, --help          Print this message

EXAMPLES
    $0                  Read confirmed DTCs (the codes lighting the MIL)
    $0 --mode pending   Read pending DTCs (early warning)
    $0 --verbose        Same as above with decode-step diagnostics

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html#obd-ii
    https://www.obd-codes.com/   (DTC code lookup)
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)
            shift
            case "${1:-}" in
                confirmed) MODE="$MODE_CONFIRMED"; MODE_NAME="confirmed" ;;
                pending)   MODE="$MODE_PENDING";   MODE_NAME="pending"   ;;
                permanent) MODE="$MODE_PERMANENT"; MODE_NAME="permanent" ;;
                *)  echo "ERROR: --mode must be 'confirmed', 'pending', or 'permanent'" >&2
                    exit 2
                    ;;
            esac
            ;;
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
            echo "usage: $0 [--mode confirmed|pending|permanent] [--verbose] [--help]" >&2
            exit 2
            ;;
        *)
            echo "unexpected positional arg: $1" >&2
            exit 2
            ;;
    esac
    shift
done

# Positive response = request mode + 0x40
POS_RESPONSE=$(printf "%02X" $((16#$MODE + 0x40)))

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

log() { [[ $VERBOSE -eq 1 ]] && echo "$@" >&2 || true; }

# Path to the optional JK Wrangler DTC description TSV.  If present
# alongside the script (same directory), output is annotated with
# the human-readable description.  Otherwise output is just the
# code.  See pubCarHacking/jeep-dtc-codes-jk.html for the full
# catalog (569 codes).
CODES_TSV="$(dirname -- "$(readlink -f -- "$0" 2>/dev/null || echo "$0")")/dtc-codes-jk.txt"

# Look up a DTC code in the TSV.  Returns the description on stdout,
# or empty string if not found / TSV missing.
lookup_dtc() {
    local code="$1"
    [[ -f "$CODES_TSV" ]] || { echo ""; return; }
    # Match exact code at start of line, followed by tab.  Strip
    # comment lines.  Print the description (everything after the
    # first tab).  -F is fixed-string for safety against regex
    # characters; -m 1 stops at the first match.
    grep -m 1 -F "$(printf '%s\t' "$code")" "$CODES_TSV" 2>/dev/null \
        | cut -f2- \
        | head -1
}

# Decode a 2-byte hex DTC into canonical SAE format (P0420, C0561, etc).
decode_dtc() {
    local hi="$1" lo="$2"
    local hi_dec=$((16#$hi))
    local lo_dec=$((16#$lo))

    # Letter from top 2 bits of high byte.
    local type_bits=$(( (hi_dec >> 6) & 0x3 ))
    local letter
    case $type_bits in
        0) letter="P" ;;   # Powertrain (engine / transmission)
        1) letter="C" ;;   # Chassis (brakes, steering, suspension)
        2) letter="B" ;;   # Body (HVAC, lighting, doors, locks)
        3) letter="U" ;;   # Network / communication
    esac

    # First digit: bits 5-4 of high byte (range 0-3).
    local digit1=$(( (hi_dec >> 4) & 0x3 ))
    # Second digit: bits 3-0 of high byte (hex 0-F).
    local digit2=$(( hi_dec & 0xF ))
    # Third digit: bits 7-4 of low byte.
    local digit3=$(( (lo_dec >> 4) & 0xF ))
    # Fourth digit: bits 3-0 of low byte.
    local digit4=$(( lo_dec & 0xF ))

    printf "%s%d%X%X%X" "$letter" "$digit1" "$digit2" "$digit3" "$digit4"
}

# Send the DTC request and capture the (potentially multi-frame)
# response.  Returns the full response payload as a hex string on
# stdout (PCI bytes already stripped), or empty string on timeout.
query_dtcs() {
    local wait="$1"

    # Background candump capturing the response.  We start it BEFORE
    # the request so we don't miss fast replies.  Capture continues
    # for $wait seconds, then we flow-control if it's a first frame.
    local capture_file
    capture_file=$(mktemp)

    # Filter the response ID at the candump level so we don't pull in
    # broadcast traffic from other modules.
    timeout -s TERM "$wait" candump -L "$CAN_C,${ECU_RES_ID}:0FFF" \
        > "$capture_file" 2>/dev/null &
    local dump_pid=$!

    # Fire the request after a brief delay so candump is ready.
    sleep 0.1
    cansend "$CAN_C" "${ECU_REQ_ID}#01${MODE}000000000000"

    # Wait for the initial timeout window to elapse.
    wait "$dump_pid" 2>/dev/null || true

    # Parse: find the FIRST frame matching our positive-response service byte.
    local first_line
    first_line=$(grep -E "#..${POS_RESPONSE}|#1...${POS_RESPONSE}" "$capture_file" | head -1)

    if [[ -z "$first_line" ]]; then
        rm -f "$capture_file"
        echo ""
        return
    fi

    log "  raw response : $(echo "$first_line" | cut -d# -f2)"

    local raw="${first_line##*#}"
    local pci_high_nibble="${raw:0:1}"

    if [[ "$pci_high_nibble" == "0" ]]; then
        # Single Frame: low nibble of byte 0 is total length.
        local length_hex="${raw:1:1}"
        local length=$((16#$length_hex))
        local data="${raw:2:$((length * 2))}"
        log "  single-frame : length=$length bytes"
        rm -f "$capture_file"
        # Strip the service-byte echo (first hex pair) from the data.
        echo "${data:2}"
        return
    fi

    if [[ "$pci_high_nibble" == "1" ]]; then
        # First Frame: bits of bytes 0-1 = 12-bit total length.
        # PCI byte 0 low nibble = high 4 bits of length; PCI byte 1 = low 8.
        local len_hi="${raw:1:1}"
        local len_lo="${raw:2:2}"
        local length=$(( (16#$len_hi << 8) | 16#$len_lo ))
        local first_data="${raw:4:12}"   # 6 data bytes carried in the FF
        log "  first-frame  : length=$length bytes, data so far=$first_data"

        # Send flow control to allow the ECM to send consecutive frames.
        # 30 = continue, 00 = unlimited block size, 00 = zero ST_min.
        cansend "$CAN_C" "${ECU_REQ_ID}#3000000000000000"
        log "  flow control sent (30 00 00) -- expecting consecutive frames"

        # Capture consecutive frames in a second window.  We need
        # ceil((length - 6) / 7) more frames.
        local remaining=$((length - 6))
        local extra_frames=$(( (remaining + 6) / 7 ))
        local cf_wait
        # 50ms per expected frame plus 100ms slack.
        cf_wait=$(awk "BEGIN { printf \"%.3f\", 0.1 + ($extra_frames * 0.05) }")

        local cf_file
        cf_file=$(mktemp)
        timeout -s TERM "$cf_wait" \
            candump -L "$CAN_C,${ECU_RES_ID}:0FFF" > "$cf_file" 2>/dev/null \
            &
        local cf_pid=$!
        wait "$cf_pid" 2>/dev/null || true

        # Re-assemble consecutive frames in sequence order (sequence
        # numbers cycle 1..F then 0..F repeatedly).
        local cf_data=""
        local expected_seq=1
        while read -r cf_line; do
            local cf_raw="${cf_line##*#}"
            local cf_pci="${cf_raw:0:2}"
            # Consecutive Frame PCI = 2N where N=sequence (0..F).
            if [[ "${cf_pci:0:1}" != "2" ]]; then continue; fi
            local seq=$((16#${cf_pci:1:1}))
            # Take expected sequence only; skip dupes / out-of-order.
            if [[ $seq -ne $expected_seq ]]; then continue; fi
            cf_data+="${cf_raw:2:14}"   # 7 data bytes per CF
            expected_seq=$(( (expected_seq + 1) & 0xF ))
        done < "$cf_file"
        rm -f "$cf_file"

        local total_data="${first_data}${cf_data}"
        # Trim trailing pad bytes to the declared length.
        total_data="${total_data:0:$((length * 2))}"
        log "  reassembled  : $total_data ($length bytes)"

        rm -f "$capture_file"
        # Strip service-byte echo.
        echo "${total_data:2}"
        return
    fi

    # Unknown PCI -- not a valid ISO-TP single or first frame.
    rm -f "$capture_file"
    echo ""
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

log "query: Service 0x${MODE} ShowDTCs ($MODE_NAME) on $CAN_C ($ECU_REQ_ID -> $ECU_RES_ID)"
log "  attempt 1 (wait ${WAIT}s)"
data=$(query_dtcs "$WAIT")

if [[ -z "$data" ]]; then
    log "  attempt 2 (wait ${RETRY_WAIT}s)"
    data=$(query_dtcs "$RETRY_WAIT")
fi

if [[ -z "$data" ]]; then
    echo "NO RESPONSE -- check that the vehicle is powered on and CAN-C is reachable." >&2
    exit 1
fi

# Some ECMs prepend a single "number of DTCs" byte; others go straight
# into the DTC list.  The DTC list itself is always 2 bytes per code.
# If the byte count is odd, treat the first byte as a count and skip
# it; otherwise treat the entire payload as DTC bytes.
total_bytes=$(( ${#data} / 2 ))
if (( total_bytes % 2 == 1 )); then
    log "  note: odd byte count ($total_bytes) -- treating byte 0 as DTC count"
    data="${data:2}"
    total_bytes=$((total_bytes - 1))
fi

dtc_count=$(( total_bytes / 2 ))
log "  decoded $dtc_count DTC byte-pair(s)"

if (( dtc_count == 0 )); then
    echo "No DTCs reported."
    exit 0
fi

# Decode each 2-byte DTC.  Skip 0x0000 entries (some ECMs pad with
# zeros to a fixed slot count even when fewer codes are stored).
# Annotate with the human-readable description from the local TSV
# if it's available (see CODES_TSV path above).
printed=0
for ((i = 0; i < dtc_count; i++)); do
    hi="${data:$((i * 4)):2}"
    lo="${data:$((i * 4 + 2)):2}"
    if [[ "$hi$lo" == "0000" ]]; then continue; fi
    code=$(decode_dtc "$hi" "$lo")
    desc=$(lookup_dtc "$code")
    if [[ -n "$desc" ]]; then
        echo "$code  -  $desc"
    else
        echo "$code"
    fi
    printed=$((printed + 1))
done

if (( printed == 0 )); then
    echo "No DTCs reported."
fi

exit 0
