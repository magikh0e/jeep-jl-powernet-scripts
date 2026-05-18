#!/bin/bash
#
# getVIN.sh -- read the 17-character VIN from CAN-IHS.
#
# Original recipe:    jmccorm-era bash (same lineage as the original
#                                       getVehicleTime.sh)
# Polish / hardening: magikh0e
# Last updated:       05.2026
#
# The vehicle's VIN is broadcast across three sequential CAN-IHS
# frames on message ID $3E0:
#
#     frame 0:  00 <ch0> <ch1> <ch2> <ch3> <ch4> <ch5> <ch6>
#     frame 1:  01 <ch7> <ch8> <ch9> <chA> <chB> <chC> <chD>
#     frame 2:  02 <chE> <chF> <ch10> 00 00 00 00
#
#   - First byte of each frame is the sequence index (0, 1, or 2).
#   - Bytes 1-7 carry seven ASCII characters of the VIN (in hex).
#   - The last frame's trailing bytes are 0x00 padding because 17
#     characters don't divide evenly across 7-byte payloads
#     (7 + 7 + 3 = 17).
#   - Each frame is broadcast ~0.1 s apart; the full VIN is on the
#     bus once every ~0.3 s while the vehicle is awake.
#
# Frames can arrive in any order on a busy bus, so the script
# reassembles them by sequence index rather than receive order.
# Listens for up to $TIMEOUT_SEC seconds total.
#
# USAGE
#     ./getVIN.sh [--retries N] [--strict] [--verbose] [--help]
#
# OPTIONS
#     -r, --retries N   Attempts before giving up (default: 2).
#                       Each attempt waits up to $TIMEOUT_SEC.
#     -s, --strict      Require exactly 17 characters AND validate the
#                       VIN-format alphabet (no I, O, Q). Without
#                       --strict, prints whatever was reassembled.
#     -v, --verbose     Echo the raw candump line and per-frame decode
#                       to stderr alongside the formatted output.
#     -h, --help        Print this message and exit.
#
# EXIT CODES
#     0   Success -- VIN printed on stdout.
#     1   Reassembled string is shorter than 17 characters (--strict
#         mode only). Partial VIN printed in the "INVALID(...)" form.
#     2   No frames received within the timeout window.
#
# REQUIRES
#     - can-utils (candump)    apt install can-utils
#     - CAN interface up:      $CAN_IHS at 125 kbps
#     - Vehicle must be awake. $3E0 stops broadcasting in sleep mode.
#
# COMPATIBILITY NOTE -- $3E0 vs $380
#     The original (pre-polish) recipe had a comment referencing message
#     ID $380, but its candump filter was 03E0:0FFF -- that's exactly
#     $3E0, NOT $380. Filter wins (it's what actually runs). The $380
#     reference in older notes was likely a typo. Verify on YOUR
#     vehicle with `candump -L can0,03E0:7FF` -- if your platform
#     puts VIN on $380 instead, edit $CAN_ID below.
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-3e0
#     https://magikh0e.pl/pubCarHacking/scripts/getVehicleTime.txt
#         (sibling: similar broadcast-read pattern for $350 / RTC)
#
# PLATFORM NOTE
#     $3E0 has been observed on JL Wrangler / JT Gladiator platforms.
#     FCA-internal CAN-IHS message IDs vary by year and platform;
#     other vehicles may broadcast VIN on a different ID or omit it
#     from CAN-IHS entirely (UDS Service 0x22 with DID 0xF190 is the
#     ISO-standard way to read VIN if broadcast isn't available).

set -eu

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

CAN_IHS=can0                # CAN-IHS interface
CAN_ID=3E0                  # Message ID broadcasting VIN
TIMEOUT_SEC=2               # Per-attempt capture window (3 frames at
                            # ~0.1s spacing = 0.3s minimum; we give
                            # extra headroom)
DEFAULT_RETRIES=2           # Attempts before giving up

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

STRICT=0
VERBOSE=0
RETRIES=$DEFAULT_RETRIES

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

usage() {
    cat <<EOF
getVIN.sh -- read the 17-character VIN from CAN-IHS message \$$CAN_ID

USAGE
    $0 [--retries N] [--strict] [--verbose] [--help]

OPTIONS
    -r, --retries N   Attempts before giving up (default: $DEFAULT_RETRIES)
    -s, --strict      Require exactly 17 characters; validate VIN alphabet
    -v, --verbose     Echo raw candump line + per-frame decode to stderr
    -h, --help        Print this message

REFERENCE
    https://magikh0e.pl/pubCarHacking/bus-message-reference.html#id-3e0
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -r|--retries)
            shift
            if ! [[ "${1:-}" =~ ^[0-9]+$ ]] || [[ "${1:-0}" -lt 1 ]]; then
                echo "ERROR: --retries must be a positive integer" >&2
                exit 2
            fi
            RETRIES="$1"
            ;;
        -s|--strict)  STRICT=1 ;;
        -v|--verbose) VERBOSE=1 ;;
        -h|--help)    usage ;;
        *)
            echo "unknown arg: $1" >&2
            echo "usage: $0 [--retries N] [--strict] [--verbose] [--help]" >&2
            exit 2
            ;;
    esac
    shift
done

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

# Print 7 ASCII characters from a 14-hex-char string, skipping any
# 0x00 bytes (padding in the last frame). No external dependencies --
# just printf and bash parameter expansion. Replaces the original
# script's bindechexascii pipeline (uncommon, not packaged on most
# modern distros).
hex_to_ascii() {
    local hex="$1" out=""
    local i byte
    for ((i = 0; i < ${#hex}; i += 2)); do
        byte="${hex:i:2}"
        [[ "$byte" == "00" ]] && continue
        printf -v c '\x'"$byte"
        out+="$c"
    done
    printf '%s' "$out"
}

# Validate a VIN against ISO 3779:
#   - exactly 17 characters
#   - A-Z (excluding I, O, Q) and 0-9
# Returns 0 if valid, 1 if not.
is_valid_vin() {
    local v="$1"
    [[ ${#v} -eq 17 ]] || return 1
    [[ "$v" =~ ^[A-HJ-NPR-Z0-9]{17}$ ]] || return 1
    return 0
}

fail_no_frame() {
    cat >&2 <<EOF
ERROR: No \$$CAN_ID frames seen on $CAN_IHS within $RETRIES attempt(s)
       of $TIMEOUT_SEC s each. Possible causes:
         - Vehicle is asleep (ignition off, doors closed > ~10 min)
         - Wrong CAN interface (current: $CAN_IHS)
         - CAN bitrate mismatch (CAN-IHS is 125 kbps)
         - VIN broadcast lives on a different ID on this platform
           (try \$380 if you have older Wrangler / Dodge notes)
EOF
    echo "NOTFOUND"
    exit 2
}

# ---------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------

if ! ip link show "$CAN_IHS" >/dev/null 2>&1; then
    echo "ERROR: CAN interface $CAN_IHS not found" >&2
    echo "       Bring it up: ip link set $CAN_IHS up type can bitrate 125000" >&2
    exit 2
fi

# ---------------------------------------------------------------------
# Main: capture + reassemble
# ---------------------------------------------------------------------

attempt=0
vin=""
while [[ $attempt -lt $RETRIES ]] && [[ -z "$vin" ]]; do
    attempt=$((attempt + 1))
    [[ $VERBOSE -eq 1 ]] && echo "  attempt $attempt/$RETRIES (timeout ${TIMEOUT_SEC}s)" >&2

    # Slots indexed by sequence byte. Empty until that frame arrives.
    declare -A slot
    slot=()
    got=0   # bitmask: 1=frame 0, 2=frame 1, 4=frame 2; want 7

    # Read frames from candump until we have all three OR the
    # outer timeout fires. `read -t` per-frame keeps us responsive.
    while read -r line; do
        [[ $VERBOSE -eq 1 ]] && echo "  raw: $line" >&2
        payload="${line##*#}"
        [[ ${#payload} -lt 16 ]] && continue   # frame too short

        seq_hex="${payload:0:2}"
        if ! [[ "$seq_hex" =~ ^[0-9A-Fa-f]{2}$ ]]; then
            continue
        fi
        seq=$((16#$seq_hex))
        [[ $seq -lt 0 || $seq -gt 2 ]] && continue   # bogus sequence id

        ascii_hex="${payload:2:14}"
        chars=$(hex_to_ascii "$ascii_hex")
        slot[$seq]="$chars"
        got=$((got | (1 << seq)))

        [[ $VERBOSE -eq 1 ]] && \
            echo "  frame $seq: '$chars'  (got mask 0b$(printf '%03d' $(echo "obase=2;$got" | bc)))" >&2

        [[ $got -eq 7 ]] && break
    done < <(timeout "$TIMEOUT_SEC" candump -L "$CAN_IHS,0${CAN_ID}:07FF" 2>/dev/null)

    if [[ $got -ne 0 ]]; then
        vin="${slot[0]:-}${slot[1]:-}${slot[2]:-}"
    fi
done

# ---------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------

if [[ -z "$vin" ]]; then
    fail_no_frame
fi

if [[ $STRICT -eq 1 ]] && ! is_valid_vin "$vin"; then
    echo "INVALID($vin)"
    exit 1
fi

echo "$vin"

# REVISION NOTES  (2026-05-16)
#     - Original recipe: jmccorm-era bash (same lineage / style as the
#       original getVehicleTime.sh). Discovered the $3E0 broadcast,
#       the seq-id-in-byte-0 layout, and the cut/fold/paste reassembly
#       approach.
#     - Added shebang + `set -eu`
#     - Replaced bindechexascii (uncommon package, not in most modern
#       distros) with a pure-bash hex_to_ascii helper using printf.
#     - Replaced cut/fold/paste/sed parsing chain with parameter
#       expansion (${payload##*#}, ${ascii_hex:i:2}, etc) -- no more
#       dependency on candump's exact column positions.
#     - Reassembly is now indexed by sequence byte (0/1/2), so
#       out-of-order frames assemble correctly. Original assumed
#       arrival order matched sequence order.
#     - Added retry loop (1Hz-ish broadcast cadence + 0.35s window
#       could miss a sweep)
#     - timeout -s 9 (SIGKILL) -> default SIGTERM (lets candump flush)
#     - Pre-flight CAN interface check with bitrate hint
#     - Optional --strict mode validates against ISO 3779 VIN alphabet
#       (17 chars, A-Z minus I/O/Q, 0-9)
#     - Distinct exit codes: 0 success, 1 invalid, 2 not found
#     - Flagged the original $380-vs-$3E0 comment/filter mismatch
#       inline; filter wins, $3E0 is the actual ID being read.
