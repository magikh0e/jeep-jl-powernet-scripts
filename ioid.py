#!/usr/bin/python3
#
# ioid.py -- generic UDS Service 0x2F (IOControlByIdentifier) driver.
#
# Usage:
#   ioid [-e] MODULE IDENTIFIER DATA
#
# Examples:
#   ioid -e bcm D0AD 0301        # Blare the horn
#   ioid -e bcm D0AD 0300        # Stop blaring the horn
#   ioid -e bcm D1B3 0301        # Turn on the 3rd brake light
#   ioid -e bcm D1B3 0300        # Turn it off again
#
# Originally created: 03.2023 by Josh McCormick (jmccorm) for the
#                              Jeep Wrangler JL / Gladiator JT
#                              community
# Last updated:       05.2026 (polish by magikh0e)
#
# WHAT THIS IS
#   A small CLI wrapper around Service 0x2F (IOControlByIdentifier).
#   The hand-rolled bash scripts on this site (horn.sh,
#   3rd_brakelight.sh, 3honk.sh) each do exactly one DID; ioid.py
#   does any DID against any of the known modules with three
#   command-line arguments.  Useful for exploring the BCM IOControl
#   DID catalog without writing a new shell script per target.
#
# DATA BYTE CONVENTION (Service 0x2F short-term adjustment)
#   Most actuator DIDs accept the same two-byte data pattern:
#       0x03  control mode = shortTermAdjustment
#       0xXX  state byte   (0x01 ON, 0x00 OFF for binary actuators)
#
#   So "0301" means "force this output ON for the duration of the
#   session"; "0300" means "force it OFF".  Some DIDs accept richer
#   state encodings -- e.g., wiper speed at $D1AA may take values
#   beyond 0x00 / 0x01.
#
# EXTENDED DIAGNOSTIC SESSION (-e)
#   Most BCM IOControl writes need an Extended Diagnostic Session
#   (Service 0x10 subfunction 0x03) opened first.  Pass -e and the
#   script handles that for you.  For back-to-back ioid invocations
#   within the ECU's S3 timeout (~5 seconds) the session stays open
#   and -e is redundant; -e on every invocation is still safe.
#
# KNOWN MODULES
#   bcm     $620 / $504 on CAN-C   (verified -- the DID catalog at
#                                   uds-writes.html#known-targets
#                                   lists 40+ working targets)
#   hvac    $783 / $503 on CAN-IHS (left in by jmccorm for
#                                   exploration; no verified DIDs
#                                   yet on this module)
#   radio   $7BF / $53F on CAN-IHS (likewise -- exploratory)
#
# RELATED ON-SITE
#   uds-writes.html#known-targets       BCM IOControl DID catalog
#   bus-message-reference.html#uds      protocol reference
#   scripts/horn.txt                    bash-and-cansend equivalent
#                                       for the single horn case
#   scripts/3rd_brakelight.txt          ditto for 3rd brake light
#
# REQUIRES
#   Python 3.7+
#   pip install python-can python-can-isotp
#   SocketCAN interface(s) up at the right bitrates
#       sudo ip link set can0 up type can bitrate 125000   # CAN-IHS
#       sudo ip link set can1 up type can bitrate 500000   # CAN-C
#
# EXIT CODES
#   0   success
#   1   bad input (unknown module, malformed hex)
#   5   timeout (no response within 0.5s)
#   6   negative response  -- usually "DID not supported" (NRC 0x31)
#       or "subfunction not supported in active session" (NRC 0x7E)
#   7   unexpected response (positive response with wrong DID echo,
#       or other oddity)
#
# REVISION NOTES (2026-05-16)
#   - Replaced deprecated python-can `bustype=` with `interface=`.
#   - Simplified the module lookup -- dropped a redundant dict
#     comprehension that built a new dict just to do a containment
#     check against keys that were already lowercase.
#   - Pre-resolved MODULE_INFO ints at load time instead of calling
#     int(..., 16) at every reference site.
#   - Added a __main__ guard so the script is importable for testing.

import can
import isotp
import time
import sys
import argparse


# ---------------------------------------------------------------------
# Known modules
#
#   key        -> (txid, rxid, channel)
#                 txid  = arbitration ID we send requests on
#                 rxid  = arbitration ID we read responses on
#                 channel = SocketCAN interface name
# ---------------------------------------------------------------------

# Full catalog kept in sync with wid.py / ecureset.py.  Source of
# truth for the bus assignments is the BMR's module-IDs table:
#   https://magikh0e.pl/pubCarHacking/bus-message-reference.html#module-ids
#
# Convention on the channel column:
#   can0 = CAN-IHS  (125 kbps, body / comfort)
#   can1 = CAN-C    (500 kbps, powertrain / safety)
MODULE_INFO = {
    "sgw":        (0x47E, 0x47F, "can1"),  # Security Gateway Module       (?)
    "bcm":        (0x620, 0x504, "can0"),  # Body Control Module           (CAN-IHS)
    "rf":         (0x740, 0x4C0, "can0"),  # RF Hub                        (?)
    "ipcm":       (0x742, 0x4C2, "can0"),  # Instrument Panel Cluster      (?)
    "evic":       (0x742, 0x4C2, "can0"),  # alias for IPCM
    "tpms":       (0x743, 0x4C3, "can0"),  # Tire Pressure Monitoring      (?)
    "airbag":     (0x744, 0x4C4, "can0"),  # OCS / Airbag                  (?)
    "abs":        (0x747, 0x4C7, "can1"),  # Anti-lock Braking System      (CAN-C)
    "shift":      (0x749, 0x4C9, "can0"),  # Electronic Shifter            (?)
    "swaybar":    (0x74A, 0x4CA, "can0"),  # Sway Bar                      (CAN-IHS)
    "drivetrain": (0x74B, 0x4CB, "can1"),  # Drive train FDC               (CAN-C)
    "acc":        (0x753, 0x4D3, "can1"),  # Adaptive Cruise Control       (?)
    "parkassist": (0x75A, 0x4DA, "can0"),  # Park Assist                   (?)
    "eps":        (0x762, 0x4E2, "can1"),  # Electric Power Steering       (?)
    "scm":        (0x763, 0x4E3, "can0"),  # Steering Column Module        (?)
    "hvac":       (0x783, 0x503, "can0"),  # HVAC                          (CAN-IHS)
    "driverdoor": (0x784, 0x504, "can0"),  # Driver Door (?)               (?)
    "passdoor":   (0x785, 0x505, "can0"),  # Passenger Door (?)            (?)
    "unknown792": (0x792, 0x512, "can0"),  # Unknown module                (CAN-IHS)
    "vision":     (0x794, 0x514, "can0"),  # Central Vision Processing     (?)
    "cscm":       (0x7BC, 0x53C, "can0"),  # Integrated Center Stack       (?)
    "amp":        (0x7BE, 0x53E, "can0"),  # Amplifier                     (?)
    "radio":      (0x7BF, 0x53F, "can0"),  # Uconnect Radio Module         (CAN-IHS)
    "ecm":        (0x7E0, 0x7E8, "can1"),  # Engine Control Module / PCM   (CAN-C)
    "pcm":        (0x7E0, 0x7E8, "can1"),  # alias for ECM
    "tcm":        (0x7E1, 0x7E9, "can1"),  # Transmission Control Module   (?)
    "hybrid":     (0x7E2, 0x7EA, "can1"),  # Hybrid Control Processor      (?)
    "battery":    (0x7E7, 0x7EF, "can1"),  # Battery Pack Control Module   (?)
}


def parse_hexadecimal(hexadecimal):
    """Accept '0xABCD', '0XABCD', or 'ABCD'; return an int."""
    if hexadecimal.startswith("0x") or hexadecimal.startswith("0X"):
        hexadecimal = hexadecimal[2:]
    try:
        return int(hexadecimal, 16)
    except ValueError:
        print(f"Invalid hexadecimal value: {hexadecimal}")
        sys.exit(1)


def send_extended_diagnostic_session(can_channel, rxid, txid):
    """UDS Service 0x10 subfunction 0x03 -- Extended Diagnostic Session."""
    can_filters = [{"can_id": rxid, "can_mask": 0x7FF}]
    bus = can.interface.Bus(can_channel, interface='socketcan', can_filters=can_filters)
    address = isotp.Address(isotp.AddressingMode.Normal_11bits, txid=txid, rxid=rxid)
    params = {'tx_data_min_length': 8, 'tx_padding': 0x00}
    stack = isotp.CanStack(bus=bus, address=address, params=params)

    stack.send(bytearray([0x10, 0x03]))
    start_time = time.time()
    response = None
    while time.time() - start_time < 0.5:
        stack.process()
        response = stack.recv()
        if response is not None:
            break
    bus.shutdown()
    return response


def send_io_control_by_identifier(can_channel, rxid, txid, identifier, data):
    """UDS Service 0x2F -- IOControlByIdentifier.

    Builds and sends the request:
        2F  <DID hi> <DID lo>  <data hi> <data lo>
    Returns the raw response payload (without ISO-TP framing) or None
    on timeout.
    """
    can_filters = [{"can_id": rxid, "can_mask": 0x7FF}]
    bus = can.interface.Bus(can_channel, interface='socketcan', can_filters=can_filters)
    address = isotp.Address(isotp.AddressingMode.Normal_11bits, txid=txid, rxid=rxid)
    params = {'tx_data_min_length': 8, 'tx_padding': 0x00}
    stack = isotp.CanStack(bus=bus, address=address, params=params)

    uds_request = bytearray([
        0x2F,
        (identifier >> 8) & 0xFF,
        identifier        & 0xFF,
        (data       >> 8) & 0xFF,
        data              & 0xFF,
    ])
    stack.send(uds_request)

    start_time = time.time()
    response = None
    while time.time() - start_time < 0.5:
        stack.process()
        response = stack.recv()
        if response is not None:
            break
    bus.shutdown()
    return response


def main():
    parser = argparse.ArgumentParser(
        prog="ioid",
        description="Generic UDS Service 0x2F (IOControlByIdentifier) driver."
    )
    parser.add_argument("module_name", help="Module name (bcm | hvac | radio)")
    parser.add_argument("identifier",  help="Data Identifier in hex (e.g. D0AD)")
    parser.add_argument("data",        help="Control data in hex (e.g. 0301)")
    parser.add_argument("-e", "--extended", action="store_true",
                        help="Open Extended Diagnostic Session (0x10 sub 0x03) first")
    args = parser.parse_args()

    module     = args.module_name.lower()
    identifier = parse_hexadecimal(args.identifier)
    data       = parse_hexadecimal(args.data)

    if len(hex(identifier)[2:]) > 4 or len(hex(data)[2:]) > 4:
        print("Identifier and data should both be two-byte hexadecimal numbers.")
        sys.exit(1)

    if module not in MODULE_INFO:
        print("No such module found. Please provide a valid module name.")
        print("Pre-defined modules:", " ".join(MODULE_INFO.keys()))
        sys.exit(1)

    txid, rxid, channel = MODULE_INFO[module]

    if args.extended:
        response = send_extended_diagnostic_session(channel, rxid, txid)
        if response is None or response[0] != 0x50:
            print("Failed to enter extended diagnostic session.")
            sys.exit(1)

    response = send_io_control_by_identifier(channel, rxid, txid, identifier, data)

    if response is None:
        print("No response received within 0.5 seconds.")
        sys.exit(5)

    if response[0] == 0x7F:
        print("NO SUCH IDENTIFIER")
        sys.exit(6)

    if (response[0] == 0x6F
            and response[1] == ((identifier >> 8) & 0xFF)
            and response[2] ==  (identifier       & 0xFF)):
        print("SUCCESS")
        sys.exit(0)

    print("Response received:")
    for byte in response:
        print(f"{byte:02X}", end=" ")
    print()
    sys.exit(7)


if __name__ == "__main__":
    main()
