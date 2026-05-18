#!/bin/bash
#
# 2k.sh -- hold engine RPM at 2000 via UDS RoutineControl.
#          Press Ctrl+C to release.
#
# Original author:   jmccorm  (cold-morning engine warm-up tool)
# Updates / polish:  magikh0e
# Last updated:      05.2026
#
# Lightly tested. Uses UDS Service 0x31 (RoutineControl) against the
# Engine Control Module ($7E0 / $7E8) to start a routine that holds
# engine RPM at the value encoded in the routine identifier:
# 0x07D0 == 2000 decimal == 2000 RPM. Cancel by exiting the script
# (Ctrl+C or kill); when TesterPresent stops arriving the ECM exits
# the diagnostic session on its own S3 timer and the RPM hold is
# released.
#
# WHAT MAKES THIS DIFFERENT FROM horn.sh / 3rd_brakelight.sh
#
#   1. Different module / ID pair.
#        BCM (horn, brake light):   $620 / $504  (FCA-internal IDs)
#        HVAC (battery V at HVAC):  $783 / $503  (FCA-internal IDs)
#      > ECM (engine):              $7E0 / $7E8  (OBD-II 11-bit IDs)
#
#   2. Different session sub-function.
#        0x03  extendedDiagnosticSession   (what brake / horn / etc use)
#      > 0x92  manufacturer-specific       (what the ECM wants for this)
#
#   3. Different service entirely.
#        0x2F  IOControlByIdentifier       (toggle an actuator state)
#      > 0x31  RoutineControl              (start / stop / poll a routine)
#
#   4. Different cleanup model.
#        horn.sh / 3rd_brakelight.sh: SIGINT trap sends an explicit
#        OFF + returnControlToECU.
#      > 2k.sh: rely on the ECM's S3 timeout. Stop TesterPresent and
#        the session expires by itself, which automatically releases
#        the RPM hold. NO explicit cancel command. That's why this
#        script's main loop is just "send TesterPresent forever";
#        Ctrl+C IS the cleanup.
#
# THE REQUEST BREAKDOWN
#
#   echo "31 05 07 D0" | isotpsend ...
#        \ /  \ /  \  /
#         |    |    +-- routine identifier 0x07D0
#         |    |        = 2000 decimal = target RPM
#         |    +------- sub 0x05 = startRoutine
#         +------------ service 0x31 = RoutineControl
#
# Two plausible interpretations of the 0x07D0:
#   (a) Routine ID 0x07D0 specifically means "hold idle at 2000 RPM"
#       and other targets are reachable via other fixed routine IDs.
#   (b) Routine ID is the target RPM value itself in big-endian
#       decimal -- 1500 RPM would be `31 05 05 DC`, 2500 RPM would
#       be `31 05 09 C4`, etc.
#
# Interpretation (b) is more elegant and matches the Chrysler habit
# of repurposing the routineIdentifier slot as a parameter; (a) is
# what ISO 14229 actually intends. Not yet tested either way. Try
# `05 DC` and watch the tach if you want to find out.
#
# SAFETY
#
#   - THE ENGINE MUST BE RUNNING. If not, this script is useless.
#   - PARK BRAKE SET. Transmission in PARK. Not just neutral.
#   - Run only when stationary. The hold persists until you Ctrl+C;
#     if you put the vehicle in gear during a hold, behaviour is
#     undefined and probably bad.
#   - Be ready to hit Ctrl+C immediately if anything sounds wrong.
#   - Don't run this indoors / in a closed garage. CO is a thing.
#   - "Lightly tested" -- jmccorm shipped this as a cold-morning
#     warmup tool and it worked, but the failure modes haven't been
#     explored. Use accordingly.
#
# REQUIRES
#   - can-utils (cansend, isotpsend)   apt install can-utils
#   - can-isotp kernel module loaded:  sudo modprobe can-isotp
#   - CAN-C interface up at 500 kbps as 'can1':
#         ip link set can1 up type can bitrate 500000
#
# REFERENCE
#   https://magikh0e.pl/pubCarHacking/jscan-uds-intro.html
#       UDS protocol intro (Service 0x22, 0x2F decoded byte-by-byte).
#       This script extends that vocabulary with Service 0x31.
#   https://magikh0e.pl/pubCarHacking/bus-message-reference.html#uds
#       Module ID pairs + the +0x40 positive-response rule.

# No wakeup needed -- if the engine isn't running, this whole thing
# is useless anyhow.

# Enter manufacturer-specific diagnostic session on the ECM.
# 0x10 = DiagnosticSessionControl, 0x92 = Chrysler-specific session
# (more rights than 0x03 extended; ECM-specific behaviour).
echo "10 92" | isotpsend -s 7E0 -d 7E8 -p 00:00 -P a can1
sleep 0.25

# TesterPresent right after the session-open. Probably redundant
# here since the next command comes within 0.25 s of the session
# response, but jmccorm left it in for safety. The S3 timer is
# short on the ECM, ~1 second on most Chrysler platforms, and a
# late TesterPresent is better than a dropped session mid-hold.
echo "3E 00" | isotpsend -s 7E0 -d 7E8 -p 00:00 -P a can1
sleep 0.25

# Start the "set engine RPM" routine with target 0x07D0 = 2000 RPM.
# Service 0x31 = RoutineControl, sub 0x05 = startRoutine, routine
# identifier = 0x07D0. The ECM should respond with 0x71 0x05 0x07
# 0xD0 on $7E8 (positive response = 0x31 + 0x40 = 0x71).
echo "31 05 07 D0" | isotpsend -s 7E0 -d 7E8 -p 00:00 -P a can1

# Hold the session open by sending TesterPresent every 0.25 s. The
# routine stays active for as long as the diagnostic session does;
# the moment we stop pinging, the ECM's S3 timer fires and the
# session expires, which automatically cancels the RPM hold.
#
# That's why there's no explicit cleanup trap. Ctrl+C kills this
# loop; loop death kills TesterPresent; missing TesterPresent
# expires the session; expired session releases the routine. The
# kernel hands cleanup back to the ECM by doing nothing.
while true
do
  sleep 0.25
  echo "3E 00" | isotpsend -s 7E0 -d 7E8 -p 00:00 -P a can1
done

# REVISION NOTES  (2026-05-16)
#     - Original by jmccorm: discovered the ECM target ($7E0/$7E8),
#       the manufacturer-specific session 0x92, the RoutineControl
#       0x31 service path, and the routine identifier 0x07D0 = 2000
#       RPM. Plus the cold-morning use case that made it a real tool
#       and not just a demo.
#     - This rewrite preserves jmccorm's exact byte sequences and
#       the rely-on-session-timeout cleanup design. Additions are
#       limited to the header doc-block (safety, interpretation of
#       the routine ID, comparison to horn/brake-light scripts) and
#       per-step comments mapping each isotpsend to its UDS decode.
#     - Deliberately did NOT add a SIGINT trap that sends an
#       explicit cancel. The session-timeout cleanup model IS the
#       cleanup -- adding a trap to send 31 02 ... (stopRoutine)
#       would be belt-and-suspenders, but it would also obscure
#       the elegant "let the protocol handle it" pattern jmccorm's
#       script demonstrates.
