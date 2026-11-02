# Pickup bot

ESP32 + 2x TB6612 + 4x N20 encoder motors, laptop vision control, 3D-printed chassis.
See CLAUDE.md for the full project context.

- firmware/robot_main   - the firmware: drive (web page + UDP), encoders, straight-drive assist,
                          pickup-arm servo, battery cutoff, over-the-air updates.
                          Wi-Fi + OTA passwords go in firmware/robot_main/secrets.h
- firmware/upload.py    - compile + upload: --usb COM5 (first time) or --ota car1.local (over Wi-Fi)
- laptop/bot.py         - test from the Mac: status, live wheel data, straight test, arm
- laptop/arena_drive.py - MAIN: USB webcam (~2.5 m up) watches the official 3x3 m floor -> robot drives
                          along the lanes to any station / bay (see the top of the file for setup + keys)
- laptop/arena_spec.py  - the official floor plan (blocks, lanes, stations, bays, lane graph, routes)
- laptop/lane_follow.py - route following along the 20 cm lanes (python lane_follow.py = self-test)
- ../../problemstatementcontext.txt - the competition handout (rules, floor, order-server API)
- laptop/follow_tape.py - older: iPhone camera on the robot -> follows the red tape square
- cad/                  - OpenSCAD source, print-ready OBJ files, renders
- docs/wiring.md        - every connection

Laptop setup: pip3 install -r laptop/requirements.txt   (Windows: py -m pip install -r laptop\requirements.txt)
