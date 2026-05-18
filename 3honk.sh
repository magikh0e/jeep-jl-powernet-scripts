#!/bin/bash
#
# 3honk.sh -- honk the horn three times via UDS, using isotpsend
#             instead of hand-rolled cansend frames.
#
# Original author:   jmccorm  (also wrote the cansend-based recipe
#                              this script in horn.sh is based on)
# Updates / polish:  magikh0e
# Last updated:      05.2026
#
# A pedagogical companion to horn.sh. Same target (BCM, DID $D0AD,
# Service 0x2F IOControlByIdentifier), same expected behaviour
# (three quick honks), but instead of building 8-byte CAN frames
# by hand with cansend and padding zeros yourself, this script
# uses isotpsend -- which handles the ISO-TP framing details for
# you: PCI length byte prepended, frame padding applied per the
# -p / -P flags, multi-frame split automatically if the payload
# ever grows past 7 bytes.
#
# Why this matters: for tiny requests like "honk the horn" the
# difference is cosmetic. For anything longer than 7 UDS bytes
# (e.g. WriteDataByIdentifier with a multi-byte value, or any of
# the SecurityAccess seed/key flows), isotpsend is the right tool.
# Once you're using isotpsend for everything, the whole protocol
# stack lifts up one layer cleanly.
#
# This is the "we're almost certain to build upon later" pattern
# jmccorm describes in the JScan UDS intro tutorial:
#     https://magikh0e.pl/pubCarHacking/jscan-uds-intro.html
#
# REQUIRES
#     - can-utils (cansend, isotpsend)   apt install can-utils
#     - can-isotp kernel module loaded:  sudo modprobe can-isotp
#     - Two CAN interfaces up: WAKE bus and UDS bus (here can0 / can1)
#
# COMPANION RECEIVER WINDOWS (in another terminal)
#     isotpdump -s 620 -d 504 -c -ta -u any
#     candump any,400:400 | egrep -v "can.*4[01234]. "
#
# REFERENCE
#     https://magikh0e.pl/pubCarHacking/jscan-uds-intro.html
#         Walkthrough that gets you to the point where this script
#         makes sense.
#     https://magikh0e.pl/pubCarHacking/bus-message-reference.html#uds
#         Protocol-byte breakdown for the same UDS flow done with
#         cansend.
#     https://magikh0e.pl/pubCarHacking/scripts/horn.txt
#         The fully-polished cansend version of the same idea,
#         with SIGINT cleanup + press/burst CLI modes.
#
# NOTE: This script assumes the vehicle is already awake. If not,
# the cansend wake frame at the top will knock it awake for you
# at the cost of 0.2 seconds.

# Wake up the CAN bus if needed (at the cost of 0.2 seconds)
cansend can0 2D3#0700000000000000
sleep 0.2

# Enter Extended Diagnostic Session via Service 0x10 sub 0x03.
# isotpsend prepends the PCI length byte automatically; we just
# provide the UDS service bytes.
echo "10 03" | isotpsend -s 620 -d 504 -p 00: -P a can1

for i in 1 2 3
do
  echo HONK
  # Service 0x2F, DID $D0AD, control byte 0x03 (shortTermAdjustment),
  # state byte 0x01 (ON). isotpsend handles padding to 8 bytes.
  echo "2F D0 AD 03 01" | isotpsend -s 620 -d 504 -p 00:00 -P a can1
  sleep 0.05

  echo UNHONK
  # Same service / DID / control byte, state byte 0x00 (OFF).
  echo "2F D0 AD 03 00" | isotpsend -s 620 -d 504 -p 00:00 -P a can1
  sleep 0.1
done

# REVISION NOTES  (2026-05-16)
#     - Original bash recipe by jmccorm: the isotpsend-based "use the
#       layer-2 tool instead of hand-rolling frames" pattern, plus the
#       three-tap honk demo.
#     - This rewrite preserves jmccorm's exact isotpsend invocations
#       and the loop body verbatim. The additions are limited to the
#       header doc-block (USAGE / REQUIRES / REFERENCE / NOTE) and the
#       inline comments explaining what each isotpsend call is doing,
#       so a first-time reader can follow it without consulting man
#       pages.
#     - Did NOT add a SIGINT trap here on purpose: this script's
#       teaching value is showing the cleanest possible isotpsend
#       form. If you want the safety scaffolding (Ctrl+C silences
#       horn, returnControlToECU on exit) reach for horn.sh, which
#       is the fully polished version of the same idea built on
#       cansend.
