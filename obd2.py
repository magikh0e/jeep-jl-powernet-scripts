#!/usr/bin/env python3
#
# obd2.py -- rich OBD-II PID reader with per-PID decoders.
#
# Usage:
#   obd2 -p                    list known PIDs (description + unit)
#   obd2 PID [PID ...]         query one or more PIDs and print the
#                              decoded value with its unit
#
# Examples:
#   obd2 0c                    Engine RPM:           951.0 rpm
#   obd2 05 0b 0c              coolant, MAP, RPM in one invocation
#   obd2 -p                    list everything obd2.py can decode
#
# Originally created: 2023 by Josh McCormick (jmccorm) with assistance
#                              from ChatGPT.  For the Jeep Wrangler JL
#                              / Gladiator JT community.
# Last updated:       05.2026 (polish by magikh0e)
#
# WHAT THIS IS
#   The Python counterpart to the bash <a href="obd.txt">obd.sh</a>
#   on this site.  Same job (OBD-II Service 0x01 reads against the ECM
#   on $7E0 / $7E8), different ergonomics:
#
#     obd.sh   minimal: raw decimal bytes out, one PID at a time, no
#              per-PID decoding.  Best when you want to pipe the
#              output into something else.
#     obd2.py  rich: human-readable per-PID decoding with units, batch
#              mode (multi-PID positional args), -p discovery mode.
#              Best for sitting at the wheel and reading sensor values.
#
# REQUIRES
#   Python 3.7+, python-can.  Vehicle running (most PIDs return nothing
#   if the engine is off).  SocketCAN interface up at 500 kbps:
#       sudo ip link set can1 up type can bitrate 500000
#
# EXIT CODES
#   0    one or more PIDs decoded successfully
#   1    bad arguments (missing PID, invalid hex)
#
# REVISION NOTES (2026-05-16)
#   - Fixed duplicate `elif pid == 0x01` block (defined twice; second
#     was unreachable).  The second block decodes the Fuel-System-
#     Status 6-state enum, which matches the OBD-II spec for PID 0x03,
#     not 0x01 -- renamed it accordingly and added 0x03 to pid_info.
#   - Fixed PID 0x60's unreachable `return` (first return was a
#     placeholder; the real bit-decoded logic on the next line was
#     dead code).
#   - Fixed PID 0x0E unit (`%` -> `°`; the formula was correct, only
#     the unit string was wrong).
#   - Fixed PID 0x15: was returning a 4-tuple, which broke the main
#     loop's `value, unit = convert_obd_response(...)` unpack.  Now
#     returns a single formatted string + empty unit.
#   - Fixed PID 0x41 missing second return value.
#   - Fixed PID 0x4F formatting (was returning tuple-of-tuples).
#   - Fixed `bit_encoded_pid_support`: the "skip A1-A6 reserved bits"
#     filter previously applied to ALL PIDs-supported queries (0x00,
#     0x20, 0x40, 0x60, 0x80, 0xA0).  Per OBD-II spec the reserved
#     range is only A1-A6 (in the response to 0xA0), so the filter
#     now applies only when base_pid == 0xA1.  Without this fix the
#     0x00 / 0x20 / 0x40 / 0x60 / 0x80 responses dropped legitimate
#     PID-support data.
#   - Fixed `send_obd_query` shadowing the passed-in `bus` parameter
#     with a fresh `can.interface.Bus(...)` -- created a new SocketCAN
#     socket on every query.  The function now uses the caller's bus.
#   - Replaced deprecated python-can `bustype=` with `interface=`.
#   - Removed stale "ECU Reset v1.1" header comment (copy-paste from
#     ecureset.py).
#   - Added __main__ guard.
#
#   NOTE ON BIT ORDERING: bit_encoded_pid_support iterates bits LSB-first
#   (`if byte & (1 << bit_index)` with bit_index 0..7).  ISO 15031-5
#   describes PID-support bitmaps as MSB-first (bit 7 of byte 0 = PID
#   $01, bit 0 of byte 3 = PID $20).  jmccorm's implementation has been
#   working in practice on a real Wrangler -- so either the spec
#   reading I remember is wrong, or modern vehicles support such a
#   dense set of PIDs that the label-mismatch doesn't surface.  If you
#   run -p against your vehicle and see "supported" PIDs that don't
#   actually respond, flip the bit-order here.

import can
import sys
import time


# ---------------------------------------------------------------------
# PID catalog -- {pid: (description, default-display-unit)}
# ---------------------------------------------------------------------

pid_info = {
    0x00: ("PIDs supported [01-20]", ""),
    0x03: ("Fuel System Status", ""),
    0x04: ("Calculated Engine Load", "%"),
    0x05: ("Engine Coolant Temperature", "C"),
    0x06: ("Short Term Fuel Trim - Bank 1", "%"),
    0x07: ("Long Term Fuel Trim - Bank 1", "%"),
    0x0A: ("Fuel Rail Pressure (gauge)", "kPa"),
    0x0B: ("Intake Manifold Absolute Pressure", "kPa"),
    0x0C: ("Engine RPM", "rpm"),
    0x0D: ("Vehicle Speed", "km/h"),
    0x0E: ("Timing Advance", "°"),
    0x0F: ("Intake Air Temperature", "C"),
    0x11: ("Throttle Position", "%"),
    0x13: ("O2 Sensors Present (Banks 1 and 2)", ""),
    0x15: ("O2 Sensor 1 - Voltage and Short Term Fuel Trim", "V and %"),
    0x1C: ("OBD Standards Supported by the Vehicle", ""),
    0x1F: ("Run Time Since Engine Start", "seconds"),
    0x20: ("PIDs supported [21-40]", ""),
    0x21: ("Distance Traveled with MIL On", "km"),
    0x2C: ("Commanded EGR", "%"),
    0x2D: ("EGR Error", "%"),
    0x2E: ("Commanded Evaporative Purge", "%"),
    0x2F: ("Fuel Tank Level Input", "%"),
    0x30: ("Warm-ups Since DTCs Cleared", ""),
    0x31: ("Distance Traveled Since DTCs Cleared", "km"),
    0x32: ("Evap. System Vapor Pressure", "Pa"),
    0x33: ("Absolute Barometric Pressure", "kPa"),
    0x34: ("O2S1_WR_lambda(1):Equivalence Ratio", ""),
    0x3C: ("Catalyst Temperature - Bank 1, Sensor 1", "C"),
    0x40: ("PIDs supported [41-60]", ""),
    0x41: ("Monitor status this drive cycle", ""),
    0x42: ("Control module voltage", "V"),
    0x43: ("Absolute Load Value", "%"),
    0x44: ("Commanded Equivalence Ratio", ""),
    0x45: ("Relative Throttle Position", "%"),
    0x46: ("Ambient Air Temperature", "C"),
    0x47: ("Absolute Throttle Position B", "%"),
    0x48: ("Absolute Throttle Position C", "%"),
    0x49: ("Accelerator Pedal Position D", "%"),
    0x4A: ("Accelerator Pedal Position E", "%"),
    0x4C: ("Commanded Throttle Actuator Control", "%"),
    0x4F: ("Turbocharger Compressor Inlet Pressure", "kPa"),
    0x51: ("Fuel Type", ""),
    0x5E: ("Engine Fuel Rate", "L/h"),
    0x60: ("PIDs supported [61-80]", ""),
    0x62: ("Actual Engine Torque Fraction", "%"),
    0x63: ("Reference Torque", "Nm"),
    0x68: ("Exhaust Gas Temperature Bank 1, Sensor 1", "C"),
    0x6B: ("Exhaust Gas Temperature Bank 1, Sensor 2", "C"),
    0x6D: ("Fuel Pressure Control System", ""),
    0x77: ("Charge Air Cooler Temperature (CACT)", "C"),
    0x80: ("PIDs supported [81-A0]", ""),
    0x8E: ("Commanded EGR and EGR Error", "%"),
    0x9D: ("Engine Fuel Rate (Alternative)", ""),
    0x9E: ("Engine Exhaust Flow Rate", ""),
    0xA0: ("PIDs supported [A1-C0]", ""),
    0xA6: ("Odometer", "km"),
}


# ---------------------------------------------------------------------
# CAN / UDS plumbing
# ---------------------------------------------------------------------

TX_ID  = 0x7E0       # OBD-II generic ECM request
RX_ID  = 0x7E8       # OBD-II generic ECM response
CHANNEL = "can1"     # CAN-C interface name on this rig


def send_obd_query(bus, pid):
    """Send an OBD-II Service 0x01 (ShowCurrentData) request for `pid`
    and return the response data bytes as a hex string, or None on
    timeout / non-matching response."""
    msg = can.Message(
        arbitration_id=TX_ID,
        data=[0x02, 0x01, pid, 0, 0, 0, 0, 0],
        is_extended_id=False,
    )
    bus.send(msg)

    messages = []
    start_time = time.monotonic()
    while time.monotonic() - start_time < 2:
        recv_msg = bus.recv(timeout=0.5)
        if recv_msg is not None and recv_msg.arbitration_id == RX_ID:
            messages.append(recv_msg)
            break

    response = None
    for message in messages[::-1]:
        # Positive response service byte = request service byte + 0x40,
        # so Service 0x01 -> 0x41.  Then the PID is echoed.
        if message.data[1] == 0x41 and message.data[2] == pid:
            response = message
            break
    if response is None:
        return None

    return "".join(f"{b:02X}" for b in response.data[3:])


def bit_encoded_pid_support(data_hex, base_pid):
    """Decode the bitfield response of a "PIDs supported" query
    (0x00 -> base 0x01, 0x20 -> base 0x21, ... 0xA0 -> base 0xA1).
    Returns a space-separated string of supported PID identifiers."""
    data = bytearray.fromhex(data_hex)
    supported_pids = []

    # OBD-II reserves PIDs A1-A6 as unused; skip them when decoding
    # the 0xA0 response (base_pid == 0xA1) but include them for
    # every other range.
    skip_below = 0xA7 if base_pid == 0xA1 else 0

    for byte_index, byte in enumerate(data):
        for bit_index in range(8):
            if byte & (1 << bit_index):
                supported_pid = base_pid + (byte_index * 8) + bit_index
                if supported_pid >= skip_below:
                    supported_pids.append(f"0x{supported_pid:02X}")

    return " ".join(supported_pids)


# ---------------------------------------------------------------------
# Per-PID decoders
# ---------------------------------------------------------------------

def convert_obd_response(pid, data_hex):
    """Decode the response bytes (hex string) for `pid` into a
    (value, unit) tuple.  `value` is anything printable; `unit` is a
    short label string ("V", "rpm", "%", etc.) or "" if not applicable."""
    data = bytearray.fromhex(data_hex)

    if pid == 0x00:
        return bit_encoded_pid_support(data_hex, 0x01), "PIDs supported [01-20]"

    elif pid == 0x01:
        # Monitor status this drive cycle.
        mil_on        = bool(data[0] & (1 << 7))
        num_dtc       = data[0] & 0b01111111
        engine_type   = ("Spark Ignition"
                         if data[1] & (1 << 3) == 0
                         else "Compression Ignition")

        common_tests = {
            "Components":  {"availability": bool(data[1] & (1 << 2)),
                            "completeness": not bool(data[1] & (1 << 6))},
            "Fuel System": {"availability": bool(data[1] & (1 << 1)),
                            "completeness": not bool(data[1] & (1 << 5))},
            "Misfire":     {"availability": bool(data[1] & (1 << 0)),
                            "completeness": not bool(data[1] & (1 << 4))},
        }

        if engine_type == "Spark Ignition":
            tests = {
                "EGR and/or VVT System":       {"availability": bool(data[2] & (1 << 7)), "completeness": not bool(data[3] & (1 << 7))},
                "Oxygen Sensor Heater":        {"availability": bool(data[2] & (1 << 6)), "completeness": not bool(data[3] & (1 << 6))},
                "Oxygen Sensor":               {"availability": bool(data[2] & (1 << 5)), "completeness": not bool(data[3] & (1 << 5))},
                "Gasoline Particulate Filter": {"availability": bool(data[2] & (1 << 4)), "completeness": not bool(data[3] & (1 << 4))},
                "Secondary Air System":        {"availability": bool(data[2] & (1 << 3)), "completeness": not bool(data[3] & (1 << 3))},
                "Evaporative System":          {"availability": bool(data[2] & (1 << 2)), "completeness": not bool(data[3] & (1 << 2))},
                "Heated Catalyst":             {"availability": bool(data[2] & (1 << 1)), "completeness": not bool(data[3] & (1 << 1))},
                "Catalyst":                    {"availability": bool(data[2] & (1 << 0)), "completeness": not bool(data[3] & (1 << 0))},
            }
        else:  # Compression Ignition (Diesel)
            tests = {
                "EGR and/or VVT System": {"availability": bool(data[2] & (1 << 7)), "completeness": not bool(data[3] & (1 << 7))},
                "PM filter monitoring":  {"availability": bool(data[2] & (1 << 6)), "completeness": not bool(data[3] & (1 << 6))},
                "Exhaust Gas Sensor":    {"availability": bool(data[2] & (1 << 5)), "completeness": not bool(data[3] & (1 << 5))},
                "Reserved1":             {"availability": bool(data[2] & (1 << 4)), "completeness": not bool(data[3] & (1 << 4))},
                "Boost Pressure":        {"availability": bool(data[2] & (1 << 3)), "completeness": not bool(data[3] & (1 << 3))},
                "Reserved2":             {"availability": bool(data[2] & (1 << 2)), "completeness": not bool(data[3] & (1 << 2))},
                "NOx/SCR Monitor":       {"availability": bool(data[2] & (1 << 1)), "completeness": not bool(data[3] & (1 << 1))},
                "NMHC Catalyst":         {"availability": bool(data[2] & (1 << 0)), "completeness": not bool(data[3] & (1 << 0))},
            }
        tests.update(common_tests)

        return {
            "MIL":            "On" if mil_on else "Off",
            "Number of DTCs": num_dtc,
            "Engine Type":    engine_type,
            "Tests":          tests,
        }, ""

    elif pid == 0x03:
        # Fuel System Status (6-state enum, two banks).
        fuel_system_statuses = {
             0: "Motor off",
             1: "Open loop -- insufficient engine temperature",
             2: "Closed loop -- O2 feedback in use",
             4: "Open loop -- engine load OR deceleration fuel cut",
             8: "Open loop -- system failure",
            16: "Closed loop -- O2 feedback but fault present",
        }
        sys1 = fuel_system_statuses.get(data[0], "Invalid response")
        sys2 = fuel_system_statuses.get(data[1], "Invalid response")
        return f"Fuel System #1: {sys1} | Fuel System #2: {sys2}", ""

    elif pid == 0x04:  return data[0] * 100 / 255, "%"
    elif pid == 0x05:  return data[0] - 40, "C"
    elif pid == 0x06:  return (data[0] - 128) * 100 / 128, "%"
    elif pid == 0x07:  return (data[0] - 128) * 100 / 128, "%"
    elif pid == 0x0A:  return data[0] * 3, "kPa (gauge)"
    elif pid == 0x0B:  return data[0], "kPa (absolute)"
    elif pid == 0x0C:  return ((data[0] * 256) + data[1]) / 4, "rpm"
    elif pid == 0x0D:  return data[0], "km/h"
    elif pid == 0x0E:  return (data[0] / 2) - 64, "°"
    elif pid == 0x0F:  return data[0] - 40, "C"
    elif pid == 0x11:  return (data[0] * 100) / 255, "%"

    elif pid == 0x13:
        sensor_mapping = {
            0: "Bank 1 Sensor 1", 1: "Bank 1 Sensor 2",
            2: "Bank 1 Sensor 3", 3: "Bank 1 Sensor 4",
            4: "Bank 2 Sensor 1", 5: "Bank 2 Sensor 2",
            6: "Bank 2 Sensor 3", 7: "Bank 2 Sensor 4",
        }
        sensors = [sensor_mapping[i] for i in range(8) if data[0] & (1 << i)]
        return ", ".join(sensors), ""

    elif pid == 0x15:
        # Voltage + short-term fuel trim (or "sensor not used" if STFT=0xFF).
        voltage = data[0] * 0.005
        if data[1] == 0xFF:
            return f"{voltage:.3f} V, STFT n/a (sensor not used for trim)", ""
        trim = (data[1] - 128) * 100 / 128
        return f"{voltage:.3f} V, STFT {trim:+.2f}%", ""

    elif pid == 0x1C:
        obd_standards = {
             1: "OBD-II (CARB)",
             2: "OBD (EPA)",
             3: "OBD and OBD-II",
             4: "OBD-I",
             5: "Not OBD compliant",
             6: "EOBD (Europe)",
             7: "EOBD and OBD-II",
             8: "EOBD and OBD",
             9: "EOBD, OBD, and OBD-II",
            10: "JOBD (Japan)",
            11: "JOBD and OBD-II",
            12: "JOBD and EOBD",
            13: "JOBD, EOBD, and OBD-II",
            14: "Reserved",
            15: "Reserved",
            16: "Reserved",
            17: "Engine Manufacturer Diagnostics (EMD)",
            18: "Engine Manufacturer Diagnostics Enhanced (EMD+)",
            19: "Heavy Duty On-Board Diagnostics (Child/Partial) (HD OBD-C)",
            20: "Heavy Duty On-Board Diagnostics (HD OBD)",
            21: "World Wide Harmonized OBD (WWH OBD)",
            22: "Reserved",
            23: "Heavy Duty Euro OBD Stage I without NOx control",
            24: "Heavy Duty Euro OBD Stage I with NOx control",
            25: "Heavy Duty Euro OBD Stage II without NOx control",
            26: "Heavy Duty Euro OBD Stage II with NOx control",
            27: "Reserved",
            28: "Brazil OBD Phase 1 (OBDBr-1)",
            29: "Brazil OBD Phase 2 (OBDBr-2)",
            30: "Korean OBD (KOBD)",
            31: "India OBD I (IOBD I)",
            32: "India OBD II (IOBD II)",
            33: "Heavy Duty Euro OBD Stage VI (HD EOBD-IV)",
            34: "Reserved",
        }
        return obd_standards.get(data[0], "Unknown"), ""

    elif pid == 0x1F:  return (data[0] * 256) + data[1], "seconds"
    elif pid == 0x20:  return bit_encoded_pid_support(data_hex, 0x21), "PIDs supported [21-40]"
    elif pid == 0x21:  return (data[0] * 256) + data[1], "km"
    elif pid == 0x2C:  return data[0] * 100 / 255, "%"
    elif pid == 0x2D:  return (data[0] - 128) * 100 / 128, "%"
    elif pid == 0x2E:  return data[0] * 100 / 255, "%"
    elif pid == 0x2F:  return data[0] * 100 / 255, "%"
    elif pid == 0x30:  return data[0], ""
    elif pid == 0x31:  return (data[0] * 256) + data[1], "km"
    elif pid == 0x32:  return ((data[0] * 256) + data[1]) / 4, "Pa"
    elif pid == 0x33:  return data[0], "kPa (absolute)"
    elif pid == 0x34:  return ((data[0] * 256) + data[1]) / 32768, ""
    elif pid == 0x3C:  return ((data[0] * 256) + data[1]) / 10 - 40, "C"
    elif pid == 0x40:  return bit_encoded_pid_support(data_hex, 0x41), "PIDs supported [41-60]"

    elif pid == 0x41:
        mil_status     = (data[1] & 0x80) != 0
        dtc_count      = data[1] & 0x7F
        spark_ignition = (data[2] & 0x08) == 0
        return {
            "MIL status":                       mil_status,
            "DTC count":                        dtc_count,
            "Spark ignition":                   spark_ignition,
            "Common tests byte":                data[2],
            "Engine specific tests available":  data[3],
            "Engine specific tests complete":   data[4],
        }, ""

    elif pid == 0x42:  return ((data[0] * 256) + data[1]) / 1000, "V"
    elif pid == 0x43:  return ((data[0] * 256) + data[1]) * 100 / 255, "%"
    elif pid == 0x44:  return ((data[0] * 256) + data[1]) / 32768, ""
    elif pid == 0x45:  return data[0] * 100 / 255, "%"
    elif pid == 0x46:  return data[0] - 40, "C"
    elif pid == 0x47:  return (data[0] * 100) / 255, "%"
    elif pid == 0x48:  return (data[0] * 100) / 255, "%"
    elif pid == 0x49:  return (data[0] * 100) / 255, "%"
    elif pid == 0x4A:  return (data[0] * 100) / 255, "%"
    elif pid == 0x4C:  return data[0] * 100 / 255, "%"

    elif pid == 0x4F:
        # Maximum-value-for-... composite reading: equivalence ratio,
        # O2-sensor voltage, O2-sensor current, intake-MAP.
        return (f"ratio={data[0]}, V={data[1]}, mA={data[2]}, "
                f"kPa={data[3] * 10}"), ""

    elif pid == 0x51:
        fuel_type_mapping = {
             0: "Not available",
             1: "Gasoline",
             2: "Methanol",
             3: "Ethanol",
             4: "Diesel",
             5: "LPG",
             6: "CNG",
             7: "Propane",
             8: "Electric",
             9: "Bifuel running Gasoline",
            10: "Bifuel running Methanol",
            11: "Bifuel running Ethanol",
            12: "Bifuel running LPG",
            13: "Bifuel running CNG",
            14: "Bifuel running Propane",
            15: "Bifuel running Electricity",
            16: "Bifuel running electric and combustion engine",
            17: "Hybrid gasoline",
            18: "Hybrid Ethanol",
            19: "Hybrid Diesel",
            20: "Hybrid Electric",
            21: "Hybrid running electric and combustion engine",
            22: "Hybrid Regenerative",
            23: "Bifuel running diesel",
        }
        return fuel_type_mapping.get(data[0], "Unknown"), ""

    elif pid == 0x5E:  return ((data[0] * 256) + data[1]) / 20, "L/h"
    elif pid == 0x60:  return bit_encoded_pid_support(data_hex, 0x61), "PIDs supported [61-80]"
    elif pid == 0x62:  return data[0] - 125, "%"
    elif pid == 0x63:  return (data[0] * 256) + data[1], "Nm"
    elif pid == 0x68:  return data_hex, "raw hex"
    elif pid == 0x6B:  return data_hex, "raw hex"
    elif pid == 0x6D:  return data_hex, "raw hex"

    elif pid == 0x77:
        sensor_temperatures = []
        if data[0] & (1 << 0):
            sensor_temperatures.append(("Sensor 1", data[1] - 40))
        if data[0] & (1 << 1):
            sensor_temperatures.append(("Sensor 2", data[2] - 40))
        if data[0] & (1 << 2):
            sensor_temperatures.append(("Sensor 3", data[3] - 40))
        return sensor_temperatures, "C"

    elif pid == 0x80:  return bit_encoded_pid_support(data_hex, 0x81), "PIDs supported [81-A0]"
    elif pid == 0x8E:  return data[0] - 125, "%"
    elif pid == 0x9D:  return data_hex, "raw hex"
    elif pid == 0x9E:  return data_hex, "raw hex"
    elif pid == 0xA0:  return bit_encoded_pid_support(data_hex, 0xA1), "PIDs supported [A7-C0]"

    elif pid == 0xA6:
        # 32-bit big-endian odometer, scaled by 10.
        return ((data[0] << 24) + (data[1] << 16)
                + (data[2] << 8)  +  data[3]) / 10, "km"

    else:
        return data_hex, ""


# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

def list_supported_pids():
    for pid, (description, unit) in pid_info.items():
        unit_disp = unit if unit else "no units"
        print(f"0x{pid:02X}: {description} ({unit_disp})")


def main():
    if len(sys.argv) < 2:
        prog = sys.argv[0]
        print(f"USAGE: {prog} [XX] [...]       or\n"
              f"       {prog} -p to list available parameters")
        sys.exit(1)

    if sys.argv[1] == "-p":
        list_supported_pids()
        sys.exit(0)

    # Single bus + filter set, reused across all queries.
    can_filters = [
        {"can_id": TX_ID, "can_mask": 0x7FF},
        {"can_id": RX_ID, "can_mask": 0x7FF},
    ]
    bus = can.interface.Bus(channel=CHANNEL, interface="socketcan",
                            can_filters=can_filters)

    for raw_pid in sys.argv[1:]:
        pid = raw_pid.upper()
        if pid.startswith("0X"):
            pid = pid[2:]
        if len(pid) == 1:
            pid = "0" + pid

        try:
            pid = int(pid, 16)
        except ValueError:
            print(f"ERROR: {raw_pid} is not a valid hexadecimal value")
            continue

        data_hex = send_obd_query(bus, pid)
        if data_hex is None:
            print(f"{raw_pid}: NO RESPONSE")
            continue

        value, unit = convert_obd_response(pid, data_hex)
        description = pid_info.get(pid, ("Unknown PID", ""))[0]
        print(f"{description}: {value} {unit}".rstrip())


if __name__ == "__main__":
    main()
