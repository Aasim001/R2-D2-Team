# Pickup robot — full project context

Read this whole file before doing anything. It is the hand-off from a long claude.ai chat
in which the hardware, wiring, firmware, laptop software and 3D-printed parts were designed.

## 1. What this project is

A small 4-wheel robot for a competition, built by Ashish (software engineer: strong Java /
Spring background, newer to embedded). **Current phase (2026-10-07): one robot driving the
official lanes** with an external webcam (`laptop/arena_drive.py`, section 10b). The iPhone /
`follow_tape.py` setup below is the earlier phase.

### The competition (full handout: `problemstatementcontext.txt`, one folder above `pickup-bot/`)
"Autonomous Multi-Robot Order Fulfilment Under Congestion" (Infosys hackathon).
- **Fleet of exactly 3 robots** on a 3 x 3 m floor. A mock WMS (an HTTP/JSON order server on the
  venue LAN) streams orders: fetch the QR cube from station S1-S6, report the pick with the cube
  ID read from its QR code, deliver it to the dispatch bay DZ, report the delivery. Optional
  telemetry POST.
- **Floor** (fixed, `arena_spec.py`): 4 keep-out blocks of 120 x 120 cm set 20 cm in from the
  edge. Every lane is exactly 20 cm wide (a perimeter ring plus a cross) and robots can never pass
  each other. Alcoves are 20 x 10 cm cut into the blocks, with a 20 x 20 docking square in the
  lane. DZ is a 20 x 60 dead end and P1-P3 are 20 x 30 dead ends: drive in, reverse out.
  Official frame: cm, origin bottom-left, x east, y north, heading 0 = east, CCW.
- Robot max **15 x 15 cm, 20 cm tall**. The pick must reach **~10 cm sideways** from the docking
  square. At most **10 fiducial markers per team**.
- Scoring: throughput. Zero collisions, zero keep-out incursions, zero wrong cubes, NO deadlocks,
  fairness, self-recovery. Compute preference: edge device > phone > laptop.
- External webcam: the owner says it is allowed and mountable (~2.5 m up or overhead).
- Figure 1 (floor plan) = `arenaimg.jpg` next to the handout. `arena_spec.py` was checked against
  it on 2026-10-07: A top-left, B top-right, C bottom-left, D bottom-right. S3's alcove is cut into
  Block A (north of its square) and S4's into Block D (south). The earlier assumption had these
  backwards. Nothing is assumed any more.

### Test track
Red electrical tape laid as a square on large glossy white floor tiles (wheels slip at high
speed — rubber tyres / rubber bands help).

## 2. Hardware (what the owner actually has)

| Part | Details |
|---|---|
| Microcontroller | ESP32 DevKit V1 clone, 30-pin, ESP-WROOM-32, CP2102 USB chip, USB-C, AMS1117 3.3 V regulator. Arduino board: **"ESP32 Dev Module"** (`esp32:esp32:esp32`), Arduino-ESP32 core **3.x** (uses `ledcAttach` / `ledcWrite(pin, duty)`). |
| Motor drivers | 2 x TB6612FNG red breakout boards: **front** driver = front-left (A) + front-right (B), **rear** driver = rear-left (A) + rear-right (B). |
| Motors | 4 x N20 gear motors with hall encoders, 6-wire harness. Encoder board pad order: **M1, GND, C1, C2, VCC, M2** = **white, blue, green, yellow, BLACK, red**. Black is **VCC** on this board (not ground). Encoders powered from 3V3. Gear ratio / counts-per-rev unknown (label partly read "...00RPM"). |
| Battery | 2S Li-ion (2 x 18650, 7.4 V nominal, 8.4 V full) → fuse → switch → VBAT |
| Regulator | LM2596 buck, **set to 5.0 V** → ESP32 **VIN** and servo |
| Servo | TowerPro **MG90S** (180° positional, metal gear) for the pickup arm, signal on D2 |
| Battery sense | VBAT –100 kΩ– D34 –47 kΩ– GND (8.4 V reads ≈ 2.7 V). Optional: firmware handles it missing. |
| Camera | **iPhone**, rear camera, mounted on the robot, streaming over Wi-Fi with the **IP Camera Lite** app (HTTP MJPEG, needs login; default admin/admin; port 8081 in use). |
| Laptop | MacBook (macOS) — runs the vision script and Claude Code |
| Also owned / planned | ESP32-CAM (planned separate QR-reading node), Jetson Orin Nano Super (owned, not used in the current setup) |

## 3. Wiring — FINAL. The owner cannot change it now.

### ESP32 pins (board held USB-C at the top, chip facing you; names as printed on the board)

| Left side | Goes to | Right side | Goes to |
|---|---|---|---|
| 3V3 | 3.3 V rail | VIN | 5 V from LM2596 |
| GND | GND | GND | GND |
| D15 | rear-right encoder C2 | D13 | rear driver PWMA |
| D2 | servo signal | D12 | not used (boot pin, keep free) |
| D4 | front-left encoder C1 | D14 | front driver PWMB |
| RX2 (16) | front-right encoder C1 | D27 | front driver AIN2 |
| TX2 (17) | rear-left encoder C1 | D26 | front driver AIN1 |
| D5 | rear-right encoder C1 | D25 | front driver PWMA |
| D18 | rear driver AIN1 | D33 | front driver BIN2 |
| D19 | rear driver AIN2 | D32 | front driver BIN1 |
| D21 | rear driver BIN1 | D35 | front-left encoder C2 |
| RX0 | USB serial — leave free | D34 | battery sense |
| TX0 | USB serial — leave free | VN (39) | front-right encoder C2 |
| D22 | rear driver BIN2 | VP (36) | rear-left encoder C2 |
| D23 | rear driver PWMB | EN | not used |

### Drivers (both identical except pins)
- Front: PWMA←D25, AIN1←D26, AIN2←D27, PWMB←D14, BIN1←D32, BIN2←D33
- Rear:  PWMA←D13, AIN1←D18, AIN2←D19, PWMB←D23, BIN1←D21, BIN2←D22
- Both: **STBY←3V3** (always enabled — GPIO4 is an encoder now), VCC←3V3, GND←GND,
  VM←VBAT, GND←battery −.
- Outputs: AO1/AO2 → front-left / rear-left motor (white M1 / red M2);
  BO1/BO2 → front-right / rear-right motor.

### Power
- Battery + → fuse → switch = **VBAT** → both drivers' VM, LM2596 IN+, battery-sense resistor.
- LM2596 OUT+ (5.0 V) → ESP32 VIN + servo red. OUT− → GND.
- ESP32 3V3 → STBY + VCC on both drivers + black VCC wire on all 4 encoders.
- **All grounds joined**: battery −, LM2596 IN−/OUT−, ESP32 GND, both GND pins on each driver,
  4 encoder blue wires, servo brown, 47 kΩ resistor.

### Pin gotchas
- D34, D35, VN, VP are input-only with **no internal pull-ups**; if a C2 on them never changes,
  add 10 kΩ from that pin to 3V3. D4, RX2, TX2, D5, D15 use INPUT_PULLUP.
- D2, D5, D12, D15 are boot-strapping pins. If an upload fails, unplug the servo signal from D2;
  if it hangs at "Connecting...", hold the BOOT button.
- In code: left wheels = motor index 0 (FL) and 2 (RL), right wheels = 1 (FR) and 3 (RR).

## 4. Repository layout

```
CLAUDE.md                     this file
README.md
docs/wiring.md                wiring table (same as section 3)
firmware/robot_main/          THE firmware (robot_main.ino)
firmware/old/                 superseded sketches (udp_drive: old 6-byte packet; web_control)
laptop/follow_tape.py         camera -> tape detection -> steering -> UDP to robot
laptop/bot.py                 UDP command-line tool: status, live telemetry, straight test, arm
laptop/requirements.txt       opencv-python, numpy
laptop/runs/                  created by --record (one folder per run)
cad/pickup_bot_v2.scad        parametric OpenSCAD source of the compact chassis + pickup arm
cad/obj/                      print-ready OBJ files (Bambu Studio)
cad/renders/                  PNG renders (stowed / reach / carry)
```

## 5. Firmware — `firmware/robot_main/robot_main.ino`

Compiles cleanly with esp32 core 3.1.1 (≈ 76 % flash). Not yet confirmed on the real robot
beyond the owner uploading it.

### What it does
- **Motors**: 1 kHz, 8-bit PWM. `setCommand(left, right)` with −255..255 per side.
- **Speed-up ramp**: `RAMP_PWM_PER_S = 1200` (≈ 0.15 s to full speed); stopping is instant.
  Set to 100000 for instant power.
- **Encoders**: interrupt on C1 (CHANGE), C2 read for direction, ISR uses direct register reads
  (`GPIO_IN_REG`), counts protected by a portMUX. `ENC_SIGN[]` only flips displayed counts.
- **Straight-drive assist** (`ASSIST`): **currently OFF by default (`ASSIST = false`) — the owner
  asked for it to be disabled.** Can be toggled at runtime (web page button, or UDP `assist on`).
  When on: if both sides get the same command, all 4 wheels are matched to the average encoder
  speed (learned per-wheel trims, zero-mean, separate forward/backward) and left/right travel is
  equalised (error measured in seconds of travel → no gear ratio needed). In turns, front/rear on
  each side are matched. Uses |counts|, so encoder sign doesn't matter. A wheel whose encoder stops
  counting is excluded (flag `ok` → 0) and copies its same-side partner's correction.
  Verified in a C++ simulation of mismatched motors: 26.7° drift over 3 s → under 1°.
- **Servo**: 50 Hz, 14-bit LEDC on D2, moves 1°/8 ms. Presets **not yet calibrated**:
  `ARM_STOW = 20, ARM_CARRY = 60, ARM_RELEASE = 72, ARM_REACH = 110` (servo degrees).
- **Battery**: D34 averaged; if < 6.4 V for 3 s → motors off (LOW BATTERY). If the divider isn't
  fitted the reading is ~0 and the cutoff stays disabled.
- **Fail-safes**: motors stop if no UDP command for 300 ms, or no web command for 1 s.
- **Wi-Fi**: station mode, `WiFi.setSleep(false)` (power saving adds lag), mDNS hostname `car1`
  (`car1.local`), prints `Ready. Web: http://<IP> ...` on Serial at 115200 baud.
  `WIFI_SSID` / `WIFI_PASS` are placeholders at the top — must be the 2.4 GHz network that the
  Mac and iPhone are on.

### Control interfaces
1. **Web page** `http://<robot IP>/`: hold ▲▼◀▶ (or keyboard arrows) to drive, speed slider,
   straight-assist toggle, reset trims, arm preset buttons, servo calibration slider, status box
   (battery, servo, Wi-Fi RSSI, per-wheel count / rate / pwm / trim / encoder ok).
   HTTP endpoints: `/cmd?c=f|b|l|r|s&s=180`, `/stop`, `/servo?a=90`, `/arm?p=stow|reach|carry|release`,
   `/assist?on=0|1`, `/trimreset`, `/status` (JSON).
2. **UDP port 4210**:
   - Drive packet = **8 bytes**: `'D','R'`, int16 left, int16 right (−255..255), uint16 seq,
     little-endian. Older/out-of-order seq numbers are dropped.
     (The `DR` header exists because the text command `status` is 6 bytes — the old 6-byte drive
     packet made `status` look like a random drive command.)
   - Any other packet = text command, reply goes to the sender:
     `stop` | `arm stow|reach|carry|release` | `servo <deg>` | `assist on|off` |
     `tele on|off` (10 Hz JSON telemetry to the sender) | `trim reset` | `status`.
   - Status JSON: `batt, low, servo, assist, cmd[2], rssi, pwm[4], cnt[4], rate[4], ok[4], trim[4]`.

### Over-the-air updates (added 2026-10-07)
- The firmware runs ArduinoOTA (port 3232, password from `secrets.h`).
  - `ArduinoOTA.handle()` is called ONLY while the robot is stopped (cmd and ramp all 0). The
    password check (PBKDF2, ~0.5-1 s) blocks `loop()`, which would otherwise leave the motors at
    their last PWM with the fail-safe frozen.
  - `onStart` → `stopNow()` and `otaActive`. `onError` → `stopNow()`, flushes the UDP drive
    packets queued during the update, and refreshes `lastCmdMs`. A failed update keeps the old
    firmware; a successful one restarts.
- **Motor watchdog**: an `esp_timer` every 50 ms cuts all 4 motors if `loop()` has not run for
  150 ms (any stall). `loop()` then calls `stopNow()` and waits for a fresh command, and the
  control-step `dt` is clamped to 50 ms so the ramp can't jump.
- `upload.py --ota`:
  - sends the robot `stop` first (the old udp_drive sketch would read `status` as a drive
    command, so never probe with it)
  - authenticates with `--auth=<pw>` as one token, masked in the output
  - `--ask-current` / env `OTA_CURRENT_PASSWORD` let you change the OTA password over Wi-Fi
    (log in with the old one, flash the new one)
  - reads the password ignoring comments (last `#define` wins)
  - firewall advice: allow espota.exe on Public AND Private (this PC's Wi-Fi is Public)
- OTA review (4-agent workflow, 2026-10-07) found these 6 issues; all are fixed.
- The Wi-Fi SSID/password and the OTA password now live in `firmware/robot_main/secrets.h`
  (`SECRET_WIFI_SSID`, `SECRET_WIFI_PASS`, `SECRET_OTA_PASSWORD`). The owner edits that file;
  never ask them to paste it.
- `python firmware/upload.py --usb COM5` (USB) or `--ota car1.local` / `--ota <ip>` (Wi-Fi),
  `--list`, `--compile`. It uses the IDE's bundled arduino-cli and the core's `espota.exe`, reads
  the password from `secrets.h` and masks it in its output. The FIRST upload with OTA in it must
  go over USB.
- Size with OTA: 1,028,534 bytes = 78 % of the 1.25 MB app slot (default 4 MB partition scheme,
  which has two OTA slots).

### Laptop hotspot (2026-10-07)
The owner can run the robot on this laptop's Windows Mobile Hotspot: the laptop is 192.168.137.1
and the robot gets 192.168.137.x. It is 2.4 GHz channel 11 because the laptop's own Wi-Fi is
2.4 GHz; set the band to 2.4 GHz in the hotspot settings and turn off "turn off when no devices are
connected". Find the robot by its Espressif MAC in `arp -a`, by `car1.local`, or by its serial
print. Confirm it with the text command `stop` (reply "ok stop"), not `status`.
- 2026-10-07: robot flashed over USB (COM3, CP2102) with the OTA + watchdog firmware. It joined the
  hotspot "DESKTOP-TEST" as **192.168.137.115** (MAC 8c-94-df-8b-93-40). `car1.local` does NOT
  resolve over the hotspot, so use the IP. Web `/status` and UDP both work.
  - Measured: RSSI −70 dBm; UDP round trip median 28 ms, 90th percentile 115 ms, worst 177 ms,
    10 % lost. The laptop's single Wi-Fi radio is shared between the hotspot and its own link to
    "i2web", which likely causes the spikes.
  - Battery reads 0.45 V: the sense divider sees no battery (switch off or not fitted), so the
    low-battery cut-off is inactive.
  - Windows hotspot quirk: it turned itself off when no device was connected (power saving).
    The ESP32 then waits in "Connecting to Wi-Fi...." (motors off) until it comes back.
- OTA over the hotspot: invitation and auth OK. The firewall first blocked espota's connect-back;
  the owner then allowed it (Public). Transfers then died at 7-85 % on the lossy link until
  `ArduinoOTA.setTimeout(10000)` was added. One OTA then reached 100 %, BUT the robot never came
  back on the network. On USB it was printing "ts.h" lines endlessly (the tail of the secrets.h
  warning?) with no ROM banner, even after an RTS reset. **Not explained yet. Watch serial over
  USB during the next OTA before relying on OTA.**
- Then (owner's request) the robot was switched to router **"NETGEAR37"** and flashed over USB.
  That firmware has the self-healing Wi-Fi (`connectWiFi()` retries every 10 s; `keepWiFi()` in
  `loop()` reconnects after 10 s lost). It boots cleanly, joins in ~3 s, and is at
  **192.168.1.3**. The laptop must be on NETGEAR37 too to reach it.
- **Current network (2026-10-07): "MiFi_021208"**, a mobile Wi-Fi router, 2.4 GHz. Flashed over
  USB. The robot joins in ~4 s at **192.168.43.93**. The laptop is on the same network.
  - Measured: RSSI −55 dBm; UDP round trip median 9 ms, 90th percentile 15 ms, worst 45 ms,
    0/60 lost. Much better than the laptop hotspot.
  - The robot is still on USB (COM3), which makes it the right moment to diagnose the OTA "ts.h"
    problem: send an OTA while reading the serial port.

### Live arena mapping with the real webcam (2026-10-07)
- The external webcam is camera **0** (DirectShow, 1920×1080); it is the only camera found. It
  sees the whole arena from just outside the near edge: all 4 red corners are in view, the 4 black
  blocks and the robot. No floor markers placed yet; the robot has NO ArUco marker yet (orange
  top plate).
- **Calibrated automatically from the red boundary**, without clicks. Red-pixel line fits per
  side (spread ~1 px); the faint far side was fitted column by column on "redness" R−max(G,B) ≥ 7
  (142 points, 0.3 px spread). Corners NW, NE, SE, SW = (534.7,124.4), (1312.2,148.5),
  (1808.1,739.3), (5.6,699.6) px. Near side = SOUTH (the floor is symmetric while there are no
  cut-outs). Saved as `laptop/arena_calib.json` (calibrated_by "red boundary lines (auto)").
- Webcam estimated from the rectangle: 1.34 m high, 1.24 m outside the south edge, HFOV 84°.
  CAM_HEIGHT_MM not set yet.
- Check against the tape: the black block tape is a median 0.8 cm from the official positions
  (worst 2.0 cm on the outer east/south edges); tile grout lines match the 60 cm grid to ~1 cm.
  **The practice floor matches the official dimensions.** The residual is taping tolerance plus
  some lens distortion; lens correction is still to do.
- **The camera is too low for the far half.** A 100 mm marker at robot height would look
  62×42 px on the near ring, 38×17 px at the centre, and 27×9 px on the far ring (~1.5 px per
  marker cell: unreadable). Raise it to ~2.5 m (or more overhead), then re-run this auto-mapping.
- `bot.py --host 192.168.43.93 status` works with the owner's normal Python (no OpenCV needed).
  OpenCV is still NOT installed there; my tests used the scratchpad venv.
- **Camera raised (owner: "around 210 cm"). Re-mapped automatically:**
  1. Rough corners come from the extreme endpoints of LONG red Hough segments. Taking extreme red
     pixels picked up a red object on a person outside the arena.
  2. Each side is refined by scanning across it for the reddest pixel at 200 points, then a
     RANSAC line fit (spread 0.3-0.9 px). It works around people standing in the arena.
  - Corners NW, NE, SE, SW = (615.4,124.8), (1416.4,178.0), (1723.8,1057.6), (222.3,917.5).
  - Tape vs official plan: median 0.2 cm, worst 1.6 cm.
  - The rectangle alone gives the camera 2.22 m up, 0.64 m outside the south edge, HFOV 84°,
    the SAME 84° as the independent first mapping. So `CAM_HFOV_DEG = 84` is now set; forcing
    2100 mm would imply 91°.
  - Saved in `arena_calib.json`.
  - A 10 cm mark now looks 55×56 px on the near ring, 38×28 at the centre and 29×17 on the far
    ring: readable everywhere.
- **Robot found by its yellow-orange TRIANGLE** (`ROBOT_MARK = "color"`; there is no ArUco
  marker on the robot):
  - HSV (9-32, ≥110, ≥100). The mark itself is hue ~15, S ~200; the red tape is hue 0-6.
  - Only inside the arena; plausible floor size 15-250 cm² (measured 74 cm²); triangle fill
    ≥ 0.55; prefers the patch nearest the last position.
  - The outline is un-warped to top-down at the mark's height. The direction comes from
    `minEnclosingTriangle`: the apex is the corner with the most equal sides (the triangle is
    WIDE, so "farthest point from the centre" picked a base corner).
  - Live, standing still: found in 34/36 frames, position ±0.3 mm, direction 87° ± 2° (it points
    north).
  - Frame rate was only ~4-5 fps. Cause: DirectShow ignores the MJPG request on this webcam and
    sends YUY2 1080p (USB-limited to ~5 fps). **Fix: `open_camera()` tries Media Foundation
    (CAP_MSMF) first**, which gives 30 fps MJPG 1080p. BUT MSMF with "hardware transforms" on hangs
    for minutes when opening. `arena_drive.py` sets
    `OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0` BEFORE importing cv2; any test script must
    import arena_drive before cv2. The camera now opens in ~1.4 s at 27-30 fps.
  - About 2 % of frames picked the wrong triangle corner. Fix: `prev_heading` continuity: if the
    apex is > 60° from the last direction, take the corner that continues it. Live: 267/267 frames
    found, 0 frames > 20° off, wobble ~1°.
- The webcam DISCONNECTED twice after being raised: probably the long USB cable / extension.
- New CLI options:
  - `--auto-setup`: maps the arena from the red boundary with no clicks
    (`red_boundary_corners()`). It keeps the H test result if the corners moved ≤ 15 px and saves
    `auto_setup_check.jpg`.
  - `--auto H,S2,DZ`: presses the keys by script, each step when the previous one has finished. It
    stops at the first failure (non-ARRIVED, failed H test, or a step longer than 90 s), then quits.
    This is how Claude can run driving tests.

### FIRST REAL RUNS (2026-10-07, camera ~2.2 m, orange-triangle tracking, MiFi network)
- Battery: fine according to the owner. The sense divider is simply not connected (reads 0.44 V).
- **H test** (`--auto H`): forward 105 mm, turned +38° in the 0.4 s pulse (this robot turns slowly).
  Result: **robot front = triangle tip −176°** (the triangle points to the robot's BACK);
  turn_sign 1 (wiring normal). Saved in `arena_calib.json`.
  - The old H test turned the robot out in the lane (2.8 cm keep-out warning, correctly
    flagged). It is now forward → BACK → turn right → turn LEFT, so it ends where it started.
- **`--auto S2`** from (150,147) heading 260:
  - It reversed 3 cm to J150,150, turned 180° there (several pulses, ~4 cm pivot drift, at the
    junction so no harm), then REVERSED 60 cm down the lane to S2.
  - Lane keeping: max 1.2 cm, typical 0.6 cm off the centre line.
  - **Arrived 1.2 cm from S2, heading 90.7° (wanted 90).** Zero keep-out frames, robot seen in
    100 % of frames, 15 s in total.
- The run loop processed ~17 fps (the camera gives 30). Next optimisation: colour detection in a
  small window around the last position (it searches the whole arena box every frame, ~23 ms).
- **Run A (`--auto S4,S5,P1,S6,S3`) failed on its first turn** (`runs/arena-20261007-184134`):
  - It drove north from S2 to J150,150 with perfect lane keeping (x 149.1–149.6).
  - It turned 90° right there, but the reported position jumped 7.4 cm (149.6,149.4 → 152.0,142.4).
  - The first DRIVE east saw 7.6 cm cross-track → OFF LANE.
- **Cause: the orange triangle is not over the robot's turning point.** A circle fit of the
  triangle through each recorded turn (error ≤ 0.2 cm) puts the turning point about 3–4.6 cm
  BEHIND the triangle and 1–2 cm to its left, so the robot also stopped 4–5 cm short of every
  junction. The point differs by turn direction (right turns −46/−33 fwd, +20 left; left turns
  −23 fwd, +6 left, in mm): it also slides a little while it turns, more when turning right.
- **Fix (`arena_drive.py`):**
  - The pose is now the turning point: `mark_to_centre(mark, heading, arena.turn_centre)`.
  - `arena_calib.json` `turn_centre_mm` = {left: [fwd, left], right: [...]} mm from the
    triangle; the position uses their average. It is not reset by `--auto-setup`.
  - It is learnt by the H test (one turn each way) and refined after every TURN/FINAL TURN
    while driving (`fit_turn_centre`/`learn_turn_centre`, 30 % per turn; saved at quit).
  - Replaying the recorded turns, the apparent jump drops from 7.4 to 2.4 cm.
  - `log.csv` gains `mark_x_mm,mark_y_mm` (the raw triangle); x/y are the turning point.
- The Lenovo webcam was missing 4× today; Windows then lists it as "Unknown". At least the last time
  the owner had unplugged it on purpose; the earlier ones are unexplained. Check with
  `Get-PnpDevice -Class Camera`. USB selective suspend is ON on this laptop. If it does drop out
  by itself, turning that off is worth a try:
  `powercfg /setacvalueindex SCHEME_CURRENT 2a737441-1930-4402-8d77-b2bebba308a3 48e6b7a6-50f5-4782-a5d4-53bb8f07e226 0`
  then `powercfg /setactive SCHEME_CURRENT`. After replugging, run `--auto-setup` if the camera moved.
- **Run A, 2nd try (`runs/arena-20261007-185435`), with the turning-point fix:**
  - S4: ARRIVED, 1.3 cm off.
  - S5: ARRIVED, 4.3 cm off. The error is sideways drift on the last leg.
  - P1: OFF LANE. 96 keep-out frames over the run.
  - The turn fits now agree within about 1 cm whichever way it turns, so the left/right split is
    mostly noise. The average is about −33 mm fwd, +12 mm left.
- Three new problems found and fixed:
  1. **The camera-moved check froze the loop for ~0.45 s every 3 s.** That is longer than the 300 ms
     firmware fail-safe, so the robot stopped and started all the time (gaps visible in every
     log.csv).
     - It now runs in a background thread (`CameraCheck`): loop steps stay ≤ 70 ms meanwhile.
     - It must disagree `CAMERA_MOVED_N` = 3 times in a row before stopping the robot. A person
       walking over the boundary once caused a false "moved 36 px" stop.
  2. **The triangle's angle reads wrong by an amount that depends on which way it faces.** Travel
     direction vs measured heading on straight drives: facing east +5.5…+8°, facing north ≈ 0°.
     - So it drove slightly crabwise: 3–4 cm sideways drift on east/west legs, keep-out frames,
       and the S5 sideways error.
     - Fix: `heading_trim_deg` {E, N, W, S} in calib, interpolated by heading
       (`heading_trim`). It is learnt from every 15 cm of straight DRIVE (`learn_heading_trim`:
       a wheeled robot moves the way it points). Seeded E +6.5, N −0.3.
  3. **At the far (north) edge the triangle is ~20 px:** about 1 frame in 8 picks the wrong corner
     as the tip (heading 125 instead of 10). One such frame plus the turning-point offset → a
     7 cm jump → OFF LANE.
     - Fix: the run loop drops a detection that jumps more than 30 mm + 300 mm/s·dt or
       45° + 3 rad/s·dt (at most 90°) from the last good one. After `JUMP_FRAMES` = 5 in a row
       it believes it.
- **Review workflow (11 agents) on those fixes. Confirmed and fixed:**
  - The H test took heading_offset from the untrimmed angle, so the trim counted twice (+6.5° in
    every direction after an H test facing east). It now solves offset + trim(offset) = travel
    (fixed point). The turn fit uses run()'s trimmed heading, windows open at the turn commands,
    and it blends at rate 0.5 if a turning point is already known.
  - A wrong-corner heading on the first frame could lock in.
    - Now (re)finding the robot needs `ACQUIRE_FRAMES` = 3 frames agreeing within 30°.
    - While tracking, if 7 of the last 10 frames' own shape disagrees by more than 60°, it finds
      the robot afresh. `find_robot*` return a 4th element = this frame's own direction.
  - Camera check: no longer a permanent stop. After 3 bad readings it stops and waits
    `CAMERA_WAIT_S` = 20 s. If the view comes back (2 good readings), it carries on and `--auto`
    redoes the interrupted step.
  - `red_boundary_corners` is vectorised: 130–180 ms instead of 380–460 ms, and loop steps stay
    ≤ 34 ms during a background check. Results match the old version within its own RANSAC
    spread.
  - Also: `drive_log` is capped at 90 frames, and a goal key while the robot is unseen fails
    `--auto` instead of silently skipping the goal.
  - Refuted by the review: resetting trims when the camera moves (trims relearn; reset is worse).
    The trim error looks like the far edge of the triangle being lost, so W ≈ −E: seeded
    E +5.5, W −5.5.
- **The arena is mapped at EVERY start now** (`map_boundary`: median of 5 frames, the ones within
  4 px of it averaged, so a person on the tape can't spoil it). `--no-auto-setup` skips it;
  `--auto-setup` maps and quits.
  - Camera re-mounted 2026-10-07 evening: auto-map says 2.39 m up; the owner measured 2.40 m.
    So CAM_HFOV_DEG 84 is right; CAM_HEIGHT_MM stays None.
- End-to-end sim (`scratchpad/e2e_test.py`): the real run loop, a rendered triangle on a real
  frame, a fake ESP32 over UDP, 90 ms lag, the true turning point (−33, +12) and a 6° cos-shaped
  angle error.
  - P1, S6, S3 all ARRIVED (0.9 / 0.9 / 0.6 cm). It learnt turning point (−32, +11) and W trim −6.2.
- Seen later the same evening:
  - All 4 corners shifted ~12 px within 15 min of mapping, so the camera mount moves a little.
  - The owner started taping bay/alcove CUT-OUTS into the blocks.
  - Next: re-map live during runs, confirmed against the black block tape; CUTOUTS = True once
    the taping is done.

### 2026-10-08 (robot offline; camera only)
- **The Windows Camera app holds the webcam.** While it is open, OpenCV can't open camera 0
  ("cannot open camera 0 - retrying"). Close it before running arena_drive.py.
- **Cut-outs taped by the owner.** A top-down warp of the live frame vs arena_spec shows:
  - P1, P2, P3 and the DZ slot match the plan within about 1 cm, so `arena_spec.CUTOUTS = True`.
    Both self-tests pass: bays are entered forward and left in reverse, 0 keep-out.
  - Station alcoves: S3 and S6 back walls only (the block edge in front is still continuous);
    S1, S2, S4 and S5 are not taped. That doesn't matter for driving, because alcoves stay keep-out
    for the body.
  - Reference frame of the empty arena: `laptop/tools/empty_arena_2026-10-08.png` (also the e2e
    background now).
- **Map check against the black block tape: `tape_fit_mm(frame, corners)`.**
  - It samples every 5 cm along all block edges (skipping bay openings), finds the dark tape line
    across each, and returns the median |offset − TAPE_INSET_MM| (15 mm: the tape centre sits
    1.5 cm inside the block edge). It takes about 15 ms.
  - Measured: right map 5 mm; corners 3 px off 10 mm; 6–8 px off 15–20 mm; 12 px off 20–30 mm.
  - Used at start: `auto_setup` rejects a map over `TAPE_FIT_MAX_MM` = 12.
- **Live re-map in run().**
  - `CameraCheck` returns the corners plus the tape fit of the current and the new map.
  - If 3 readings in a row are more than `REMAP_PX` = 4 px off and agree within 4 px, and the new
    map fits the tape better and within 12 mm, it calls `Arena.set_corners()`. The new corners are
    saved at quit.
  - One reading more than `CAMERA_STOP_PX` = 25 px off stops the robot at once. More than 8 px
    for 3 readings without a confirmed re-map also stops it; then it waits 20 s, as before.
  - Bug caught by the e2e test: the re-map code reused the name `new`, which run() uses as its
    "new frame" flag → crash. Renamed.
- **e2e test (`tools/e2e_test.py`, now optionally with camera drift):**
  - Run: `150 150 90 P1,DZ,P3,P2 12`, with the camera sliding 12/6 px over 60 s.
  - Result: all ARRIVED (0.3 / 0.6 / 1.0 / 1.1 cm), re-mapped live twice, 16 fps.
  - Learnt: turning point (−33, +11), true (−33, +12); trims E +6.3, W −6.0, true ±6.
- **Known limit: on-the-spot turns.**
  - Robot 13 × 15 cm → corners sweep a 19.8 cm circle in a 20 cm lane, so a pivot fits only
    centred within ~1 mm.
  - With 1–2 cm sideways error, a corner pokes out up to 1.5–2.3 cm briefly during the turn
    (sim: ring corner J10,290 and the P2 mouth, over the outer boundary).
  - Possible fixes: re-centre sideways before pivoting (needs a small S-move), arc turns, or a
    narrower robot.

### Building / uploading
- Arduino IDE: board "ESP32 Dev Module", port `/dev/cu.usbserial-…` or `/dev/cu.SLAB_USBtoUART`.
- arduino-cli: `arduino-cli compile --upload -p <port> --fqbn esp32:esp32:esp32 firmware/robot_main`
  (`arduino-cli board list` shows the port; core: `arduino-cli core install esp32:esp32` with the
  Espressif index URL `https://espressif.github.io/arduino-esp32/package_esp32_index.json`).
- USB alone powers the ESP32 but not the motors — battery switch must be on to drive.
- Multi-robot plan: three identical robots on one Wi-Fi network → change `HOSTNAME` to car2, car3.

## 6. Laptop software (MacBook, Python 3, `pip install -r laptop/requirements.txt`)

### `laptop/follow_tape.py`
Camera → red-tape detection → PD steering → UDP drive packets at ~30 Hz.

```
python3 follow_tape.py --camera http://PHONE_IP:8081 --user admin --password admin --preview
python3 follow_tape.py --camera http://PHONE_IP:8081 --user admin --password admin --car ROBOT_IP
python3 follow_tape.py ... --record          # save the run for review (see section 8)
python3 follow_tape.py --find                # list camera numbers (Continuity Camera / webcams)
```
Keys in the window: **SPACE** start/stop, **+/−** speed, **Q** quit.

- **Camera input**: `--camera` takes a camera number or a stream URL (http MJPEG or rtsp).
  For http it handles **Basic and Digest login** (`--user/--password`, `user:pass@` in the URL,
  or auto-tries admin/admin), and if given only `http://ip:port` it opens the app's web page and
  finds the MJPEG link inside it (like a browser), then tries common paths. Always keeps only the
  newest frame (no lag build-up), auto-reconnects if the stream drops, shows a "Waiting for
  camera" screen with the reason. **Status: the owner confirmed the iPhone stream works with the
  login, using the back camera.**
- **Continuity Camera did not work** for the owner (OpenCV uses an AVFoundation path that recent
  macOS deprecates for Continuity Camera). Don't go back to it unless asked.
- **Detection**: frame resized to 320x180, HSV threshold for red (two hue ranges `RED1`, `RED2`),
  ignore top `ROI_TOP` = 35 %, split the rest into `BANDS` = 6 horizontal slices; each slice with
  ≥ `MIN_PIX` red pixels gives an x position (−1..1).
- **Steering**: err = 0.5·nearest + 0.5·farthest seen slice − `CAM_OFFSET`;
  turn = (KP·err + KD·d(err)/dt)·base; forward speed reduced by up to 60 % in bends;
  when no slice sees tape → **SEARCH**: spin toward the side where it was last seen
  (handles the 90° corners). Sends (0,0) if the camera freezes > 0.3 s.
- **Tunables (current defaults)**: `BASE_SPEED 170`, `MIN_PWM 95` (N20 stall), `KP 1.3`,
  `KD 0.02`, `SEARCH_SPEED 150`, `ROI_TOP 0.35`, `CAM_OFFSET 0.0`,
  `RED1 ((0,100,60),(10,255,255))`, `RED2 ((165,100,60),(180,255,255))`.
  Recommendation given to the owner: start around speed 130 (Wi-Fi camera adds ~0.1–0.25 s lag).
- Window shows camera view (left, green dots = tape per slice) and red mask (right).

### `laptop/bot.py` (UDP tool)
```
python3 bot.py --host ROBOT_IP status
python3 bot.py --host ROBOT_IP watch               # live wheel table
python3 bot.py --host ROBOT_IP drive 180 180 3     # drive 3 s, prints left/right count difference (< ~2 % = straight)
python3 bot.py --host ROBOT_IP arm reach           # stow|reach|carry|release
python3 bot.py --host ROBOT_IP servo 90 | assist on | assist off | trim reset | stop
```

### Network
iPhone, MacBook and ESP32 must be on the **same 2.4 GHz Wi-Fi**. IPs change between sessions
(an earlier network was 10.67.197.x); the robot prints its IP on Serial at boot; `car1.local`
works on most Macs, otherwise use the IP.

## 7. 3D-printed parts (`cad/`)

- Owner prints with **Bambu Studio** and needs **OBJ** files (mm, no unit conversion).
  PLA, 0.2 mm, 3 walls, 20 % infill.
- `pickup_bot_v2.scad` (current design, all parts collision-checked in CAD): compact stacked
  chassis, **145 x 130 mm, 141.5 mm tall** with the arm stowed.
  - Lower plate: 4 N20 motor slots (shaft hole + 2 x M1.6 + zip-tie slots + encoder clearance),
    2x18650 holder guides, LM2596 spot, flat **side panel the cube rests against** while carried
    (wheels don't stick out there).
  - Middle deck: MG90S mount (shaft down), slot for the scissor's sliding pin, standoffs for a
    70 x 90 mm perfboard (ESP32 + drivers), ESP32-CAM mast tilted 35° down toward the pickup side.
  - Top plate: 76 mm AprilTag recess (arrow = robot front).
  - **Side scissor arm** (lazy tongs, 4 cells, 38 mm links, driven by the MG90S) + **one-way hinged
    flap** at the tip: extend → flap rides over the cube and drops behind it → retract → flap hooks
    the cube and drags it against the side panel → carry → extend ~15 mm and drive away to release.
    First-link servo angles from CAD: stowed 77°, carrying a 50 mm cube 54°, full reach 23°
    (reach ≈ 108 mm past the side panel). Real servo values still need calibrating.
  - OBJ parts: 1_lower_plate, 2_middle_deck, 3_top_tag_plate, 4_camera_cradle,
    5_scissor_link (x7), 6_scissor_link_servo (x1), 7_tip_block, 8_flap, 9/10 spacers, 11_wheel (x4, 34 mm).
  - Hardware: M3 screws + nyloc nuts for arm joints, M3 for plates, M2 x 30 hinge pin, M2 servo screws.
- Earlier designs (superseded): a 90 x 140 mm basic chassis, and a 150 x 148 mm version with a
  rack-and-pinion slide + 2-finger claw. The owner's current physical chassis is his own print;
  don't assume it matches the CAD exactly.

## 8. Review-and-fix loop (how to help on this Mac)

You can't watch the robot live. Work in short loops:
1. Ask the owner for a recorded run:
   `python3 follow_tape.py --camera http://PHONE_IP:8081 --user admin --password admin --car ROBOT_IP --record`
   He presses SPACE to start, Q to stop. Each run goes to `laptop/runs/<date-time>/`:
   - `summary.txt` — duration, fps, % frames with tape lost, number/length of searches,
     avg/max steering error, settings used. **Read this first.**
   - `log.csv` — every frame: t, fps, state (FOLLOW/SEARCH/STOPPED/NO CAMERA), base, err
     (−1..1, + = tape to the right), seen (0–6 slices), L, R (PWM sent), band0..band5 x-positions.
   - `robot.jsonl` — ~10 Hz robot telemetry (pwm, encoder rate, battery, encoder ok flags).
   - `frames/*.jpg` — camera view + tape mask with overlay, 3/s and at every state change.
     Look at frames around problems (e.g. the first `_SEARCH` frames after a FOLLOW stretch).
2. Change one or two tunables at a time, explain why, and ask for another run.
   Typical reads: err sign flipping every few frames → lower KP / raise KD; tape lost at corners →
   lower BASE_SPEED or ROI_TOP (look further ahead); `seen` often 1–3 on straights → check the RED
   ranges against the mask half of the frames; wheel `rate` 0 while `pwm` > 0 → motor, wiring or
   battery problem; robot turns when told to go straight → motor wired backwards (swap white/red)
   or enable straight assist.
3. Firmware changes: edit, compile, upload over USB (section 5), and tell the owner what changed.

All of this was tested in the cloud chat with fake cameras/robots (MJPEG + login servers, UDP
robot simulator). **Nothing in sections 5–6 has been verified on the real robot except: the
owner uploaded the firmware and the iPhone stream connects with login + back camera.**

## 9. Decisions made (and why)
- Pickup on the **side** (not front/back): no turning needed at the docking square.
- **Drag, don't lift**: cube can't fall; one servo is enough.
- **Scissor arm + one-way flap** chosen over a claw (straight-line reach from a small body).
- **STBY → 3V3**: GPIO4 was needed for an encoder.
- **UDP** for drive commands (low latency, newest packet wins, 300 ms fail-safe).
- **Wi-Fi IP-camera app** instead of Continuity Camera (didn't work with OpenCV here).
- **Straight assist disabled by default** at the owner's request (code kept, toggleable).

## 10. Open items / next steps
1. First real runs of the tape follower with `--record`; tune KP / KD / BASE_SPEED / ROI_TOP.
2. Check every encoder counts (web page: turn each wheel by hand → its count changes, `ok`).
   If one wheel spins backwards: swap that motor's white and red wires.
3. Calibrate the arm presets with the servo slider and put the numbers in `ARM_*`.
4. A printed iPhone mount (landscape, ~40° down tilt, low and centred over the axles).
5. Later: ESP32-CAM QR reading; three robots (car1–car3) on one network.
6. Ask organisers: does the size limit apply while carrying the cube?

## 10a. Official-floor lane driving (2026-10-07) — `arena_drive.py` + `arena_spec.py` + `lane_follow.py`

With `OFFICIAL = True` (the default), `arena_drive.py` drives ONE robot along the official lanes to a
goal chosen by key: 1-6 = S1-S6, D = DZ, 7/8/9 = P1-P3. SPACE = pause / go on (re-plans from where
the robot is), R = old roam mode.
- `arena_spec.py`: the floor in official cm. Blocks, bays (drivable dead ends), alcoves, docking
  squares, `FLOOR_MARKERS` (ids 10-13 at (35,35), (265,35), (35,230), (265,230): near the outer
  block corners, inside blocks, clear of every bay and alcove). Spread out like this, 3 mm of
  placement error costs ~0.5 cm at the ring (the first, bunched positions cost 1.3-2.7 cm).
  Lane graph (Dijkstra `route()`, `route_from_point()`), `is_keepout()`, `keepout_depth()`
  (distance to the nearest lane strip / bay), `heading_for_pick()`, `goal_node()`.
  `python arena_spec.py` prints routes and saves `arena_spec.png`.
- **`CUTOUTS = False`** (in `arena_spec.py`, set 2026-10-07): the owner's practice floor has solid
  taped blocks, with no bays or alcoves cut out. Bays then count as keep-out and are not in the
  graph (19 nodes / 22 edges). Goals D and 7/8/9 stop at the bay's MOUTH on the ring
  (`goal_node()`). Stations work as normal. Set True once the cut-outs are taped.
- `lane_follow.py`: `Follower.go(x, y, hdg, goal)` then `update(t, x, y, hdg)` → (L, R, state)
  every frame. It turns on the spot ONLY at junctions and bay mouths: a 15 cm robot turning
  mid-lane sweeps 10.6 cm > the 10 cm half-lane. Otherwise it REVERSES:
  - at the start when facing the wrong way (only as far as the first junction)
  - onto a docking square when that puts the pick side (PICK_SIDE = right, from the CAD) toward
    the alcove (only from the last junction)
  - out of bays
  On a straight it steers with a heading term + cross-track term + yaw-rate damping
  (K_HEAD 0.35, K_CROSS 0.10, K_RATE 0.10; the earlier K_HEAD 1.5 oscillated with 0.13 s of lag).
  `go()` plans the direct route AND routes via every junction, and keeps the one with the fewest
  turns where there's no room (`grade()`), then the shortest (reverse cm count ×1.3). So
  "goal ahead but the dock needs the nose the other way" goes on to a junction, turns there and
  comes back. Starting > 6 cm off the lane is refused (OFF LANE). 3-6 cm off along the lane is
  driven back in by the cross-track term, with no sideways stub.
  - Turns over 35° go fast at PIVOT_PWM and stop early (learned `latency`). Smaller turns and stop
    corrections (NUDGE) are TIMED PULSES: how far a pulse moves doesn't depend on the delay, so
    they converge at any delay. The pulse gains learn: shorter after an overshoot, longer if it
    hardly moved, more PWM if pulses don't move it at all.
  - SETTLE waits until the robot is really still, comparing poses 0.25 s apart.
  - The straight-line stop delay `stop_lat` is learned separately, from stop over/undershoot.
  - After 8 tries a 2× tolerance is accepted; after 16 tries or 20 s → TURN FAILED (stop).
  - With the outer edge straight ahead at a junction, it aims 1 cm short (EDGE_SHORT_CM), so an
    overshoot can't swing it over the edge when it turns.
  `python lane_follow.py` = self-test. `simulate()` models lag, noise, slip and the firmware ramp.
- Setup on the official floor: automatic from the 4 floor markers (Enter when all are found), or
  C → click 2 points per side in the order SOUTH, EAST, NORTH, WEST (refuses a mirrored order).
  No tape clicks (the map is exact; `USE_LIVE_TAPE` is off: on a practice floor without taped
  bays, the block tape would close them). `CAM_HEIGHT_MM` (measured) pins the focal length,
  which matters for an overhead webcam.
- A live keep-out monitor warns if the robot outline is > 5 mm over a lane edge. `--record`
  logs the official pose, the goal, keep-out frames and arrival errors.
- **Tested in simulation only**:
  - `lane_follow` self-test + stress test: 720 random legs (3 setups: CAD size, max 15 x 15,
    bad camera/Wi-Fi), 0 failures, stops ≤ 2.3 cm (3.6 cm with the bad camera).
  - Keep-out: 0.1 cm (CAD size) / 1.0 cm (15 x 15). The remaining touch is turning at a T-junction
    against the outer edge, which no software can avoid for a 15 x 15 robot.
  - End-to-end (scratchpad `sim_official.py`): rendered webcam 2.5 m up, 0.8 m outside the south
    edge. Setup → H → S2 → DZ → S5 → P3 through the real `setup()` / `run()` and a fake ESP32.
    Camera height estimated 2.50 m (true 2.50), stops 0.5-0.9 cm, docking heading ≤ 0.7°, true
    keep-out max 0.1 cm.
- The bug review (8-agent workflow) found 5 real defects, all fixed: the focal estimate for
  centred / overhead views (now least squares + overhead fallback), the `need` floor, an H-test
  turn over 180°, setup with an unusable layout, and the setup window too big for 1366×768.
- Second review (8 agents, 2026-10-07): 14 confirmed findings (9 distinct), all fixed:
  - CRITICAL: with CAM_HEIGHT_MM, height(f) is not monotonic for a side view, so the wrong root
    shifted every pose by up to 17 cm. Now all roots are found and the one nearest the
    rectangle's own focal estimate is used.
  - The floor markers were bunched in the middle (moved out).
  - The marker sheets had uneven borders: centring the sheet put the black square 7.5 mm off.
    Borders are now symmetric.
  - Off-lane sideways stub; 180° turns mid-lane / on the dock (now via-junction planning).
  - Turn limit cycle at ≥ 0.27 s delay (now pulses).
  - A robot in a bay facing out was forced to turn round in the bay.
  - `keepout_depth` over-reported at bay ends.
  - Stray setup clicks became side points; SPACE re-planned from a stale pose.
- After the fixes (simulation):
  - delay 0.13-0.5 s: 0 failures, stops ≤ 1.2 cm
  - 540 random legs: 0 failures, stops ≤ 2 cm, keep-out 0.0 cm (CAD size) / 0.8 cm (15 x 15, at a
    bay mouth against the outer edge: geometric)
  - end-to-end practice floor: stops 0.2-0.9 cm, true keep-out 0.0 cm.
- **Firmware reviewed 2026-10-07: no change needed for the lane driving** (OTA was added later that
  day at the owner's request; see section 5). Same 8-byte DR protocol and 300 ms fail-safe
  (the laptop sends ≥ 20 Hz); ASSIST stays off (it would fight the follower's steering). Compiles
  with esp32 core 3.3.8 (77 % flash) using the IDE's bundled arduino-cli. Simulated with the
  firmware's 1200 PWM/s ramp from 0 versus a proposed "start the ramp at 90": no measurable
  difference, so it was not changed. Stops coast (IN1 = IN2 = LOW); short-brake (both HIGH) is
  a possible later tweak if real stops overshoot.

## 10b. Arena mode (newer version) — `laptop/arena_drive.py`

The owner moved to a **USB webcam on a Windows PC** with a side view of the whole arena (tape
lines on the floor); the robot must drive **between the lines**. The iPhone / `follow_tape.py`
setup is kept but no longer the main path. Firmware unchanged (same 8-byte `DR` UDP packets).
(This "roam between the lines" mode is now the R key / `OFFICIAL = False`; see 10a.)

- **Robot position**: ArUco marker (`DICT_4X4_50`, id 0, `--make-tag`) flat on the top plate.
  Needs a white border ≥ 1/6 of the black square. Found in the arena crop at 1x then 2x, and
  in a crop around the last position (enlarged to ~80 px).
- **Setup** (`--setup --arena-w --arena-h`): click 2 points on EACH side of the red rectangle
  (8 clicks, going round; side 1 = arena-w). The corners are where the lines meet, so they may be
  off-screen (they are, in the owner's photos). Then click the tape (robot out of the arena), black
  AND red → `arena_calib.json` + `arena_tape_map.png`. The final preview draws the arena and the
  measured blocks over the camera picture: they must sit on the tape.
- **Measured layout** (`RING_MM`, `GAP_MM` at the top of the file; still None = not measured yet).
  The map is drawn from the numbers with the 4 blocks SOLID, and live tape detection only adds
  extras (e.g. a ball). Recommended: the reflections leave gaps in the detected tape.
  The webcam's focal length and position are computed from the corners alone. They are used
  to correct the marker's height parallax (marker ~14 cm up looks ~30 cm off from a side view).
  A field of view outside 50–100° gives a warning (usually swapped arena-w / arena-h).
- **Heading test** (`H`, once after setup): still → forward → still → right turn → still.
  It learns marker→robot-front offset and `turn_sign` (catches swapped left/right motors).
  SPACE refuses to start until it has been done.
- **Tape**: colour from the clicks (only the least floor-like pixels around each click). Black
  and coloured clicks get separate ranges; black = V ≤ min(p95 + 25, 0.33 × floor V), which keeps
  out the grout lines. Checked in the full-res camera picture, then warped to a 5 mm/px top-down
  map, then open 3×3 (removes grout and specks) and close 5×5 (joins tape broken by reflections).
  Live tape is OR-ed with the saved map. The robot's own body (and the floor it hides) is masked
  out of the live part.
- **Steering**: distance transform of the map. Each frame it tries directions −90..+90° and
  scores each by room along 300 mm (capped), minus turn / change-of-mind costs. It steers with
  PD and pivots if > 55° off; STUCK = turn on the spot. Stops if the marker is unseen > 0.3 s
  or the camera freezes.
- Tunables at the top of the file: `BASE_SPEED 140`, `KP 1.0`, `KD 0.05`, `PIVOT_DEG 55`,
  `NEED_MM` (= half robot width + 10), `LOOK_MM 300`, `TAG_HEIGHT_MM 141.5` (measure!), `CAM_W/H 1920x1080`.
- `--record` → `laptop/runs/arena-<date-time>/` (summary.txt, log.csv, frames).
- **Tested only in simulation** (rendered side-view webcam, red-tape loop lane 300 mm wide,
  simulated robot with lag, fake UDP robot, scripted keys). Results:
  - 1080p 1.1 m up / 1.3 m in front, and 720p 1.6 m up: 4 laps per minute, robot centre always
    ≥ 99 mm from tape.
  - 720p low side view: the marker is too small at the far edge → robot stops safely.
  - Not run with a real webcam or robot yet.

### Real arena (owner's webcam photos, `WIN_20261006_*.jpg` in the folder above `pickup-bot/`)
Glossy white 600 mm(?) tiles with dark grout lines. Thin red/orange tape outer rectangle. Inside it,
4 square blocks of black tape (2×2), so the corridors are a ring around the blocks plus a cross
between them. The shape is fixed; the webcam position is not decided yet. Ceiling-light reflections
cross the tape.
- From one photo (assuming a 3 × 3 m arena): corridors ≈ 215–230 mm wide, against a 150 mm
  robot. Little room for error.
- Tape pipeline on the real photo: grout rejected, black tape found (a few gaps at reflections),
  far red tape mostly too faint (fine: the arena edge counts as a wall). The guessed symmetric
  layout (ring 230, gap 215) lines up with the black tape in the overlay.
- **Camera placement is the main risk.** Simulated real layout (scratchpad `sim_real.py`):
  - webcam about 1.15 m up just outside the near edge (like the photos): the far half is about
    4 m away, the marker is about 16 × 4 px and unreadable, so the robot stops. A 134 mm marker
    alone or a 2 m webcam alone are still not enough.
  - **webcam 2 m up + 134 mm marker** (biggest that fits the top plate): drove 15.7 m in 60 s,
    covered most of the arena, marker lost 6 frames, closest centre-to-tape 86 mm (half-width
    75): it never touched tape.
  - Rerun 2026-10-07 (the first runs were killed for low memory): webcam 2.5 m up + normal marker
    drove 15.6 m per minute (min 91 mm); colour-only mode at 2 m + big marker drove 15.7 m
    (min 90 mm, no false tape).

### Next steps (2026-10-07)
1. Owner: print the markers (`--make-tag`): robot marker ~130 mm square, floor markers ~150 mm
   black, with the CENTRE OF THE BLACK SQUARE at the printed positions (the 2026-10-07 positions
   near the block corners). Mount the webcam (~2.5 m), measure CAM_HEIGHT_MM (essential if
   overhead) and TAG_HEIGHT_MM, run `--setup`, H, then try goal keys on the real floor with
   `--record`. Practice floor: CUTOUTS = False (bay goals stop in front of the bay).
2. (done) S3 / S4 alcove sides checked against Figure 1 (`arenaimg.jpg`).
3. Lens-distortion correction (webcam barrel distortion at the edges may be several cm). With
   only 2.5 cm of lane margin this is likely the next accuracy limit.
4. Then the fleet: 3 marker ids, a lane-reservation / deadlock-free planner on the arena_spec
   graph, a WMS client + mock WMS, QR reading, telemetry, dashboard.

## 11. Working with the owner
- He usually wants **complete, paste-ready files** (not diffs) plus short numbered steps.
- Keep code beginner-readable with comments; never change wiring without saying so clearly —
  **the wiring in section 3 is final**.
- Always keep the motor fail-safes.
- 3D parts → OBJ files for Bambu Studio.
- He's on a Mac (Terminal, Arduino IDE). The arena-mode PC is Windows (`py` / `python`, COM ports).

### 2026-10-08 real runs (robot back online, camera 2.40 m)
- **Results** (arrival error):
  - S4 2.1, S5 1.8, P1 1.0, S6 0.8, S3 2.5, S1 0.9, P2 2.8 cm.
  - DZ: TURN FAILED at its mouth. P3 and S2 not run yet.
  - But 120–360 keep-out frames per run: the cause is almost all the triangle's DIRECTION.
- **Constant error.** The triangle read ~7° low in every direction (it was fine on 10-07; probably
  knocked on the robot). Fixed: `heading_offset_deg` −176.4 → 176.8.
- **Position-dependent error of 5–10° on top.** Per-direction trims at 30 % / 15 cm couldn't keep up.
  - Now `TRIM_STRETCH_MM` = 60, `TRIM_LEARN` = 0.5, learning only when the heading changed less
    than `TRIM_STRAIGHT_DEG` = 8° over the stretch.
- **Direction method changed: corners → area moments** (in `find_robot_color`).
  - Principal axes come from the 2nd moments. Along each, the 3rd moment (skew) gives the thin end.
    The stronger-skew axis wins.
  - This triangle is wide: its long axis runs along the base, so the minor-axis skew points to the tip.
  - Live: same mean as the corner method (+0.9°) but 3× less spread (1.7 vs 5.6°).
  - The continuity guard picks the nearest of the 4 axis directions. `find_robot*` still return the
    frame's own direction as element 4.
- **Far NW corner / top lane** (smallest, blurriest view): the corner method flipped tips, giving
  7 cm position jumps and ~10 s of back-and-forth turning. That fed bad turning-point fits.
  - Now `learn_turn_centre` ignores turns where the robot was lost or re-found, and fits more than
    15 mm from the known point. Turning point reset to (−34, +9).
  - "Find afresh" now needs 9 of 10 disagreeing frames (was 7).
- **Glare:** the mark is glossy. Near the camera (bottom ring, DZ mouth) a ceiling reflection
  crosses it, and during turns the heading alternated about 40° → TURN FAILED.
  - Now `MARK_CLOSE_PX` = 11 closes gaps in the mask.
  - Owner advice: make the mark matte, and best add a 2nd colour dot at the front (direction =
    line between two centroids).
- **The robot hides part of the north red line** on the top lane → a false 44 px "camera moved"
  stop. The check now trusts the black tape: if the current map still fits it
  (≤ `TAPE_FIT_MAX_MM`), the camera has not moved.
- **Test scripts:** `import arena_drive` BEFORE cv2 (the MSMF option). Otherwise opening the
  camera hangs. Fixed in tools/*.py.

### 2026-10-08 afternoon: SYNC + wheel encoders
- **SYNC move before every `--auto` run** (auto-prepended; `--no-sync` skips it; key Y by hand).
  - It drives ~5 cm straight and back, never turning, only if `room_along()` says the outline
    stays clear.
  - The move direction vs the mark direction gives the heading offset (fixed point with the trims,
    like the H test).
  - It fails `--auto` if the legs move < 25 mm (battery / Wi-Fi) or the mark isn't seen.
  - If the offset changed > `SYNC_NEW_MARK_DEG` (20°), the mark was turned or replaced: it resets
    the turning point and trims, and inserts the H test if `room_to_turn()` (i.e. at a junction).
  - Real run: it caught the owner's NEW mark (a house/arrow shape, area ~79 cm², triangle fill
    0.76–0.82, still passes the shape filter), which reads 178.7° from the old triangle.
- **Encoders:** 4 hall encoders, ~13.2 counts/mm (`ENC_COUNTS_PER_MM`); the left wheels count
  backwards (`ENC_SIGN`).
  - The firmware streams `cnt/rate/ok` at ~11 Hz after `tele on` (`Car.start_telemetry`,
    `Car.turned(t0, t1)`).
  - Uses:
    1. Referee for sudden camera heading jumps: only if > `ENC_CHECK_DEG` 15° since the last good
       look, and rejected if > 20° from what the wheels turned.
    2. Rotation scale learnt from turns (`learn_enc_scale`; sim converged to the true 150 mm/rad).
    3. Stall stop (`STALL_S`): motors driven but the wheels don't turn.
- **Firmware straight-drive ASSIST: keep it OFF.**
  - With it on (one run), the owner saw the wheels spinning and jerky driving.
  - Counters after the run: FL 20.6k, RL 94.6k, FR 78.7k, RR 16.9k. Same-side wheels differed
    4×, so two diagonal wheels spun on the tiles. With assist off, they agree (840/837, 914/849).
  - The slip also made the first encoder referee reject good frames (NO ROBOT every ~3 s), hence
    "referee only on suspicious jumps".
  - The laptop now sends `assist off` at start.
- Calib reset for the new mark: turning point [0, 0] (learnt again by H / turns), trims 0,
  heading offset −1.9° (from SYNC).
- **Later runs:**
  - SYNC + H worked on the real robot: the new mark sits ~1 cm from the turning point, and its
    offset is +8° (skewed).
  - The drive to J150,10 was clean (4 keep-out frames).
  - The bottom ring (glare) still failed the turns. Near the camera the arrow mark picks the wrong
    axis in ~15 % of frames (centre washed out, ragged outline, elongation only 1.8).
    - The shape-only "find afresh" rule kept re-locking onto the wrong axis, so it is REMOVED.
    - Replaced by `motion_heading_error`: while driving straight, if the move direction is more
      than `WRONG_LOCK_DEG` 50° off the heading, it snaps `hint_heading` by the nearest multiple
      of 90° (the mark's other axis / end).
- **Last run: 20 s of TURN with ZERO movement.**
  - The firmware stopped answering UDP (ping OK), then its counters read 0 (restarted).
  - A 0.3 s pulse at PWM 130 moved only FR, by 7 mm.
  - So: flat battery / no motor power (turns went 38° → 17° → 0° during the day; no battery
    sense fitted, reads 0.44 V).
  - Added a camera-based stall stop: driven ≥ MIN_PWM for `CAM_STALL_S` 2.5 s with < 1 cm and
    < 4° movement → STALLED and `--auto` stops. The encoder stall check only works while
    telemetry flows.
- **New batteries → FULL TEST PASSED** (`runs/arena-20261008-143537`). Start: SYNC from J150,150,
  then S4 S5 P1 S6 S3 S1 P2 DZ P3 S2 in one `--auto` run, 224 s.
  - ALL TEN ARRIVED: 1.7 / 2.4 / 2.1 / 1.1 / 1.5 / 2.0 / 2.4 / 2.8 / 1.8 / 0.6 cm.
  - Robot not seen in 1.0 % of frames, no OFF LANE, no TURN FAILED.
  - Keep-out: 157/3478 frames (4.5 %), median 0.9 cm, max 1.9 cm. 82 of them on the DZ leg
    (long centre-lane drive toward the camera).
  - Learnt: turning point (−3, +4) mm (the new mark sits on it), encoder scale ~210 mm/rad,
    trims ≈ −6° in every direction (a constant: next SYNC folds it in).
  - Next ideas:
    - Seed new-mark trims to 0 but let SYNC + the first stretch fix the constant faster.
    - Re-centre sideways before pivots.
    - Check the DZ centre-lane drift.
    - Wire the battery sense (D34 divider) so a flat battery is reported instead of guessed.

### 2026-10-08 late: lane-keeping tuning (simulation only - robot offline)
- **Cause** in the passing real run: 4-wheel skid steer. A left/right difference below ~100 mm/s
  just scrubs (DZ leg: 40 PWM for ~1 s, the heading barely changed), so the old proportional
  steering wobbled 3–4 cm with a ~6 s period on the centre lane.
- **lane_follow.py changes** (old file kept as `laptop/tools/lane_follow_before_integral.py`):
  - `STEER_MIN` 30: friction compensation.
  - `K_INT` 0.3: integral steering, plus a learned per-direction imbalance (`INT_LEARN`).
  - `K_CROSS` 0.10 → 0.20, `K_HEAD` 0.35 → 0.6, `BASE_PWM` 130 → 110.
  - `MAX_APPROACH_DEG` 25: the cap is needed, or a robot 4.5 cm off re-turns forever, because the
    approach angle exceeds `REALIGN_DEG`.
- **Tuning tools:**
  - `tools/tune_lane.py` (192 combos) and `tools/tune_lane_robust.py`, using the fast simulator
    now with scrub / imbalance / heading bias and lane metrics.
  - Result: keep-out 9.9 % → 0.01 % of the time, lateral rms 1.7 → 0.4 cm, failed legs
    87/480 → 0/480.
- **Camera e2e** (scrub 80 mm/s modelled, DZ+S1): straight-line true rms 2.5 → 0.9–1.3 cm. Still
  keep-out during pivots (corner sweep: 19.8 cm diagonal in a 20 cm lane).
- **Not yet run on the real robot.**

### 2026-10-08: pickup arm software (servo not wired yet)
- **Owner built the CAD v2 arm:** side scissor arm + one-way flap on ONE MG90S per robot. Their
  Downloads copy of `pickup_bot_v2.scad` is identical to `cad/`.
  - The flap / side panel are centred along the robot (y = 65 = wheel midpoint = turning point),
    so a docking-square stop lines the arm up with the alcove.
- **Wiring:** signal D2 (firmware `SERVO_PIN`), red to LM2596 OUT+ 5.0 V, brown to OUT−, plus a
  470–1000 µF cap at the servo. See `docs/wiring.md` "Pickup-arm servo".
- **Firmware presets were WRONG for this arm** (REACH 110 = 90° from stow; the arm only swings
  ~54°). Changed to CAD guesses STOW 20 / CARRY 43 / RELEASE 50 / REACH 74. Edited in the .ino,
  NOT flashed yet.
- **arena_drive.py:**
  - Angles: `arm_deg` + `arm_calibrated` in calib. The arm refuses to move until
    `tools/arm_calibrate.py` has saved them; `auto_setup` keeps them.
  - PICK (key K / `--auto` PICK) = reach → carry. DROP (key U) = release; stows automatically once
    the robot is out of the bay.
  - `Car.arm(deg)` sends "servo N"; the telemetry "servo" angle tells when it is there.
  - `--orders S1,S4` expands to S1,PICK,DZ,DROP,S4,PICK,DZ,DROP.
  - The keep-out check includes the carried cube (`cube_corners`).
  - lane_follow: `CARRY_OFFSET_CM` 2.5 while carrying (robot + cube centred in the lane).
  - e2e: fake firmware handles "servo" and `ARM_OK=1`. The SYNC,S1,PICK,DZ,DROP,S4 sim run worked
    end to end (arm 74 → 43 at S1, 50 in DZ, 20 after leaving the bay).
- **GEOMETRY RISK:** robot ~145–150 mm + cube 50 mm ≈ 195–200 mm, inside a 200 mm lane. While
  carrying there is ~0–3 mm of margin per side, so any lateral error puts robot or cube over an
  edge. Consider a recess in the side so the cube is carried (partly) inside the body outline.
- **Still to do:** QR reading (ESP32-CAM on the mast; the 2.4 m overhead webcam can't resolve a
  4–5 cm QR), WMS client (picked/delivered POSTs), and verifying the cube actually came along
  (camera or a contact/IR sensor).

### 2026-10-30: claw calibrated on the real robot (robot 2), servo made safe
- **Robot 2** (MAC 00:70:07:a3:18:a8, USB COM13) flashed over USB. Wi-Fi = **"A34"** (a 2.4 GHz phone
  hotspot) → robot at **10.227.57.218**; the laptop must be on A34 too. `HOSTNAME` is still "car1" in the
  code, so this robot answers as car1.local. The OTA password is still the default "change-me".
- **The built mechanism is a claw (two arms closing on the cube)**, not the CAD scissor arm + flap.
  Measured with the cube in it: **a HIGHER angle CLOSES it; 58° grips the cube, 30° = open (lets go).**
  - Presets: STOW 30 / CARRY 58 / RELEASE 30 / REACH 30 (reach = open, ready to pick).
  - **Hard limit `ARM_MIN`..`ARM_MAX` = 30–60°** on every servo path (UDP `servo`, `arm`, `nudge`, web
    page). Past 60 the claw would squeeze / jam. Below 30 has not been tried. Widen only in small steps.
  - The old presets would have crushed it (REACH 74 here, 110 in `old/web_control`), which may be how it
    broke before. `old/web_control.ino` now carries a warning.
  - `arena_calib.json` has `arm_deg` + `arm_calibrated`, so PICK = 30 → 58 and DROP = 30 in
    `arena_drive.py`. `ARM_DEFAULT` holds the same values. `arm_calibrate.py`'s check now expects carry
    further closed than stow / release / reach (the old check was for the scissor arm).
- **Servo safety in the firmware:**
  - **No servo pulses at power-on** (limp, doesn't move). The first servo / arm command starts it and jumps
    straight to that angle (`setServoTarget`), because a hobby servo can't report where it is.
  - `servo off` = limp at once. `nudge <deg> [pulses 1-10]` = a few pulses toward deg, then limp: a small,
    bounded move from an unknown position. It also reads D2 back and replies with the pulses it saw and
    their width (proves the signal leaves the pin).
  - Numbers are checked (`parseNumber`): before, `servo off` or any typo became `servo 0` via `toInt()`.
  - `status` has `servo_on`; the web page shows "off (limp)" and its slider is 30–60.
  - `arena_drive.py` sends `servo <stow>` (30) at start: a jump, but inside the safe range.
- **Seen while testing:**
  - On USB power, 1–2 pulse nudges toward 25 / 30 / 40 never moved the servo, although the D2 read-back
    showed correct pulses (834 µs for 30°). After the owner changed some connections, 2-pulse nudges toward
    45 moved it. The exact fault wasn't pinned down.
  - A smooth 28° move (58 → 30) took ~1 s instead of the intended 0.22 s (1° / 8 ms), so `loop()` seems to
    take ~35 ms per pass. Not investigated yet (it also paces the 50 Hz motor control).
- **Not checked yet:** `cube_corners()` and `lane_follow.CARRY_OFFSET_CM` still assume the CAD side-panel
  carry. The claw may hold the cube somewhere else.

### 2026-10-30 / 11-02: front claw, ArUco robot, first autonomous pick attempts
- **Robot names (the owner's):** **bot 1** = MAC 00:70:07:a3:18:a8 (CH340, the claw robot, ArUco **id 11** on
  top, marker 14 cm above the floor). **bot 2** = MAC 8c:94:df:8b:93:40 (CP2102, COM12, the robot of the October
  runs). Both run the same firmware (name still "car1"). Wi-Fi "A34" (phone hotspot): its address range changes
  when the phone restarts it (10.227.57.x, then 172.22.81.x) - find robots by sending "status" to the whole /24.
  `car1.local` does not resolve there. Updates over Wi-Fi: `python firmware/upload.py --ota <ip>` (a first try
  can be cut off at "Authenticating"; retry). Full-flash backup of bot 1: `firmware/backups/`.
- **The claw is at the robot's FRONT** (`lane_follow.PICK_SIDE = "front"`): on a docking square it turns to
  FACE the alcove (`heading_for_pick(..., "front")`), opens, creeps toward the cube, closes, backs out.
  Claw (higher angle = closes): grip 58, release 30, open for the approach 20 (allowed from 2026-11-02;
  `ARM_MIN..ARM_MAX` = 20..60 in the firmware). Measured from the camera: the jaws hold a cube ~10 cm in front of
  the turning point (`CLAW_HOLD_MM`), tips at ~13 cm.
- **Wheels:** `tools/wheel_check.py IP [--camera]` (each side, encoders + camera pivot point) and the firmware's
  single-wheel test `wheel fl|fr|rl|rr <pwm> [ms]`. Bot 1's front-left wheel was dead (turned about a point
  6-7 cm to its left and slid off the arena at S6); the owner repaired it - now all 4 turn, turning point
  (+19, -11) mm from the marker, heading offset -9 deg. Bot 2: all 4 good, but front/rear are swapped on each
  side (wiring) - harmless for driving.
- **First autonomous run with all 4 wheels (2026-11-02 13:45, `--auto H,S6,PICK,DZ,DROP`):** H OK, S6 ARRIVED
  1.7 cm off (24 keep-out frames in the whole run). PICK MISSED: the claw opened only to 30 (barely wider
  than the cube), the fixed 75 mm creep coasted to 95 mm and pushed the cube 2 cm deeper. Then the claw closed to
  58 on NOTHING and the robot stalled (wheels slipping) - and **since then the claw does not move at all**
  (camera: identical at 20 / 30 / 45 deg; the servo pulses on D2 are correct) -> hardware: stripped servo or
  slipped horn. Check it before any more picks.
- **New in arena_drive.py:** `find_cube()` - the cube in an alcove from the overhead camera (top-down warp at the
  cube's top height, only pixels clearly BRIGHTER than the floor - tape off the plan fooled a darker-or-brighter
  test -, robot outline masked, centre = half a cube in from the far edge). Live: S6 cube at (29.7, 197.2) cm
  +-0.4, no false cubes. Used to stop level with the cube (`dock_shift`, `Follower.go(shift=...)`), to set the
  creep (`ahead - CLAW_HOLD_MM`, max 110 mm), and after the pick: a cube still in the alcove = PICK MISSED ->
  one retry, then stop. The claw is always released (`servo off`) when a run ends. `CREEP_STOP_EARLY_MM` 25.
- `tools/e2e_claw_test.py` (being written 2026-11-02): end-to-end sim of the ArUco robot + front claw + a cube.
- **The claw came back** after the owner restarted the robot (camera: jaw span closed 85 / 45 deg 105 / 30 deg 121 /
  20 deg 133 px, noise 13 px) - so the "dead claw" was electrical, not mechanical. Approach angle now 20.
- **Stalls at y = 180 were a TILE JOINT:** the cube (carried with its bottom on the floor) caught on it and the
  wheels slipped. Added an unstick move (`bump_step`: back off 25 mm, push at PWM 180 until 4 cm past, at most
  `BUMP_TRIES` 2, then STALLED). Better: carry the cube a few mm off the floor (mechanical).
- `run()` is wrapped: any crash / Ctrl+C stops the wheels and sends `servo off`.
- **FIRST COMPLETE AUTONOMOUS DELIVERIES (2026-11-02, bot 1, `--auto <station>,PICK,DZ,DROP,P3`):**
  - 14:22 S6 -> DZ -> P3: 82 s; S6 stop shifted 4.3 cm to line up with the cube, creep 94 mm, pick check OK,
    DZ 0.9 cm, P3 1.3 cm, keep-out 4.5 % of frames. Cube confirmed in DZ by the camera.
  - 14:27 S3 -> DZ -> P3 (the owner moved the cube to S3): 86 s; shift +1.9 cm, creep 37 mm, DZ 0.6 cm,
    P3 0.7 cm, keep-out 2.3 %. Cube confirmed in DZ at (71.2, 59.5).
- Also added (not yet run on the robot): PICK from a bay (`find_cube` / `dock_shift` handle "DZ", "P1".."P3":
  stop CLAW_HOLD + 2 cm short of the cube) and DROP at a station = PLACE (creep so the cube ends mid-alcove,
  open, back out, look). The robot outline masked by `find_cube` now includes its mirror image on the shiny floor.
- **bot 2 (2026-11-02):** carries ArUco **id 10** (arena_drive still tracks only `ROBOT_TAG_ID` 11 - one robot at a time;
  per-robot calibration is still to do). Flashed with the newest firmware (servo 20-60). Its claw sat closed and did
  NOT move for nudges toward 30 (2 and 5 pulses, read-back OK), also after a power cycle - the owner is checking it.
  Camera-guided claw calibration script: scratchpad `claw_calib_cam.py IP MARKER_ID` (nudge open, explore 20-60 in
  steps, stop as soon as the jaws stop moving) - worth moving into tools/.
- Git: this folder had no repository (it is a copy of the 10-08 state); a local repo was created 2026-11-02. The
  GitHub repo (Aasim001/R2-D2) answered "not found" without a login and has later history (10-10..10-12) - don't push
  onto it without the owner deciding how to merge.

### 2026-11-03: speed pass, wheel calibration, bot 2's claw
- **Stable snapshot first:** tag `stable-2026-11-03` (commit a97b7c0) = the code of the two full deliveries on 11-02
  (driving at BASE_PWM 110 / PIVOT_PWM 160). `git checkout stable-2026-11-03` goes back to it.
- **Measured** (not guessed):
  - Real command -> seen-motion delay, from 41 drive starts in the 11-02 runs: median 213 ms = the simulator's
    3-4 frame lag. Wi-Fi (A34 hotspot) round trip to bot 2: median 9 ms, p99 54 ms - not the bottleneck.
  - Per frame at 1080p: marker tracking 3.7 ms, top-down warp 1.3 ms, drawing the window 11.3 ms (8.5 of it in two
    numpy lines), imshow 2-6 ms, recording 2.6 ms, lost-robot search 50 ms (13 ms at 1x + 37 ms at 2x).
  - Standing still, the camera pose changes <= 0.14 cm over 0.15 s (99 %).
  - Simulator: the robot spent 25 % of a route SETTLING after stops/turns.
- **Changes:**
  - `lane_follow`: BASE_PWM 110 -> 160, PIVOT_PWM 160 -> 220 (speed sweep: fastest that stays accurate; 200+ weaves,
    255 fails legs), SETTLE_S 0.3 -> 0.15, new STILL_S 0.25 -> 0.15. `simulate()` also returns the route time.
  - `arena_drive`: the drive command is sent BEFORE drawing / showing / recording (~15 ms less delay per frame);
    `draw()` uses cv2.convertScaleAbs + copyTo (11.3 -> ~3 ms); the slow 2x marker search runs only on every 3rd
    frame without the robot (`find_robot(try_2x=)`).
  - Firmware: a new drive packet runs the control step at once (it waited for the next 20 ms tick, 10 ms on
    average); the wheel-speed filter is time-based (RATE_TAU_S) because steps are now 4-20 ms apart. Flashed to
    bot 2 over USB and used for the claw tests; NOT yet on bot 1.
- **Results (simulation):** 10-stop route 257 -> ~133 s. e2e claw test (ArUco, S6 pick -> DZ -> P3) 59.4 -> 55.8 s,
  same accuracy. e2e colour test (harder: marker 3.3 cm off the turning point) 87-89 -> 52-53 s, the same number of
  keep-out frames, but the worst touch 2.1-3.0 -> 3.5-3.7 cm, now during turns. Delay sensitivity: at 160/220 it is
  fine up to ~133 ms of pure lag (real ~113), weaves at 200 ms - so keep the delay low. NOT yet run on the real arena.
- **tools/wheel_calibrate.py** (new): each N20 alone (start PWM, speed vs PWM, both ways) and `--moves` (all four:
  forward/back/turns, left/right balance); reports crossed motor/encoder wiring. Bot 2: all within 4 %, left side
  ~1.5 % slower - no per-wheel gain needed. Its motors are counted by the OTHER wheel's encoder on the same side
  (front <-> rear crossed) - fix the wiring (case A: swap green/yellow D4/D35 <-> TX2/VP and RX2/VN <-> D5/D15, or
  case B: swap white/red front <-> rear driver outputs; which one depends on which wheel spins for `wheel fl`).
- **USB power back-feeds the motors:** with only USB plugged in, VIN -> LM2596 OUT+ leaks back to VBAT (~4 V), so the
  wheels turn weakly and the LEDs light. Keep the battery on whenever the motors or servo run.
- **Bot 2's claw** (new metal-gear servo, calibrated from a front camera view): higher angle closes (like bot 1);
  open 30, wide open 20, grip 44 (tested: crept 79 mm to a cube, closed, reversed - the cube came along); the jaws
  touch at 51 without a cube. Claw angles are now PER ROBOT: `arena_calib.json` `arm_deg_by_tag` {"11": bot 1, "10":
  bot 2}; `arena_drive.arm_angles()` picks ROBOT_TAG_ID's (none = claw locked); `tools/arm_calibrate.py IP [MARKER]`.
- Also: an iPhone hotspot is 5 GHz unless "Maximize Compatibility" is on (the ESP32 needs 2.4 GHz).
