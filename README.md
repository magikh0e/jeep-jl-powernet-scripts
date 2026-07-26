# jeep-jl-powernet-scripts

Linux / SocketCAN scripts for talking to the CAN bus on the **2018+ Jeep Wrangler (JL) "Powernet"** platform: read sensors, drive the HVAC and EVIC, honk, hold RPM, and live-dashboard the bus. Built on the community reverse-engineering effort at jlwranglerforums.com.

> ⚠️ **Use this at your own risk.** These send real frames to a live vehicle bus. Only use them on a vehicle you own, never while driving, and expect the unexpected.

## Background

This is a repository of knowledge based around the 2018+ Jeep Wrangler JL platform CAN bus, gathered by a number of people on the jlwranglerforums.com forum. For more detail, see:

- Forum thread: [Reverse Engineering the Wrangler's CAN Bus (Powernet)](https://www.jlwranglerforums.com/forum/threads/reverse-engineering-the-wranglers-can-bus-powernet.82139/)
- Community spreadsheet of identified CAN IDs and data: [Google Sheet](https://docs.google.com/spreadsheets/d/16ypMADKinBBnH1pOY4-gMmVRjeR85fYplpV12aCHJC4)

## Requirements

- A Linux host with **SocketCAN** and **can-utils** (`cansend`, `candump`, and `isotpsend` from the isotp tools).
- A CAN adapter on each bus you want to reach: **CAN-IHS** (`can0`, 125 kbit/s) and **CAN-C** (`can1`, 500 kbit/s). A Raspberry Pi with a dual-CAN HAT is the common build.
- For the Python dashboards: `python-can` (`pip install python-can`), plus Tkinter and, for the camera view, OpenCV (`opencv-python`).

## Setup

| Script | What it does |
|---|---|
| `canup.sh` | Bring up `can0` (125k) and `can1` (500k) with a large tx queue |
| `vcan-up.sh` | Create virtual `vcan0` / `vcan1` for log playback and testing without the vehicle |

## Read / monitor

| Script | What it does |
|---|---|
| `batvolt.sh` | Battery voltage (CAN-IHS ID `2C2`) |
| `cabintemp.sh` | Cabin temperature |
| `gettime.sh` | Vehicle date and time (CAN-IHS ID `350`) |
| `showrpm.sh` | Live engine RPM (ID `322`) |
| `showevic.sh` | Dump the EVIC (dash) display text (ID `328`) |
| `rid.sh` | Read Data By Identifier (UDS `0x22`) from a module |
| `ridscan.sh` | Brute-scan a range of Data Identifiers on a module |
| `obd.sh` | Send an OBD-II PID request and print the response |
| `logreader.sh` | Maintain per-CAN-ID state files under `/tmp` (feeds the dashboards) |

## Control / actuate

| Script | What it does |
|---|---|
| `3honk.sh` | Honk the horn three times |
| `2k.sh` | Hold the engine at 2000 RPM via a UDS diagnostic session (e.g. for winching or charging); releases when the script exits |
| `fanspeed.sh` | Read or set the HVAC fan speed (`01` low to `07` max) |
| `ventmode.sh` | Read or set the HVAC vent mode (defrost / feet / panel / auto) |
| `mute.sh` | Read or toggle the radio mute |
| `evic.sh` | Show custom text on the EVIC music-info page: `evic.sh [line] [text]` |
| `dimmer.sh` | Dim a Raspberry Pi backlight from the JL dimmer switch (watches ID `291`) |
| `overheadflash.sh` | Flash the third brake / overhead courtesy light in a loop |

## Automation

| Script | What it does |
|---|---|
| `autocollect.sh` | Engine-state watcher; runs hooks on power-up, engine running, per minute / 10 min / hour, and shutdown |
| `autoheat.sh` | Set HVAC to heat on remote start |
| `autocool.sh` | Set HVAC to cool on remote start |
| `remote.sh` | Toggle the Pi's WiFi transmitter on remote lock / unlock (stays RF-quiet while driving) |

## Dashboards (Python)

| Script | What it does |
|---|---|
| `pycan.py` | Terminal (curses) live CAN data display |
| `pycan2.py` | v2 terminal display with CAN filters |
| `tkcan.py` | Tkinter GUI dashboard |
| `tkcan1.py` ... `tkcan5.py` | Tkinter dashboard iterations, some with an OpenCV backup-camera feed, dark mode, and a tach / battery / AC screen |

## Credits

All CAN IDs and behavior come from the community reverse-engineering work on the [jlwranglerforums.com thread](https://www.jlwranglerforums.com/forum/threads/reverse-engineering-the-wranglers-can-bus-powernet.82139/) and the shared [Google Sheet](https://docs.google.com/spreadsheets/d/16ypMADKinBBnH1pOY4-gMmVRjeR85fYplpV12aCHJC4). Please use this information at your own risk.
