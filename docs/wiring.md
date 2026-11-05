# Wiring (current)

Power: 2S battery + -> 3 A fuse -> switch = VBAT.
- VBAT -> VM on both TB6612 drivers, LM2596 IN+, and the 100k battery-sense resistor.
- LM2596 set to 5.0 V (set it BEFORE connecting): OUT+ -> ESP32 VIN + servo red. OUT- -> GND.
- ESP32 3V3 -> STBY + VCC on both drivers (STBY must NOT go to GPIO4 any more - D4 is an encoder), and VCC (black wire) on all 4 encoders.
- ALL grounds joined: battery -, LM2596 IN-/OUT-, both ESP32 GND, both GND pins on each driver,
  4 encoder GND (blue), servo brown, 47k resistor.

ESP32 pins as printed on the board (USB at the top, chip facing you):

| Left side | Goes to | Right side | Goes to |
|---|---|---|---|
| 3V3 | 3.3 V rail | VIN | 5 V from LM2596 |
| GND | GND | GND | GND |
| D15 | rear-right encoder C2 | D13 | rear driver PWMA |
| D2 | servo signal | D12 | not used |
| D4 | front-left encoder C1 | D14 | front driver PWMB |
| RX2 | front-right encoder C1 | D27 | front driver AIN2 |
| TX2 | rear-left encoder C1 | D26 | front driver AIN1 |
| D5 | rear-right encoder C1 | D25 | front driver PWMA |
| D18 | rear driver AIN1 | D33 | front driver BIN2 |
| D19 | rear driver AIN2 | D32 | front driver BIN1 |
| D21 | rear driver BIN1 | D35 | front-left encoder C2 |
| RX0 | USB serial (leave free) | D34 | battery sense |
| TX0 | USB serial (leave free) | VN | front-right encoder C2 |
| D22 | rear driver BIN2 | VP | rear-left encoder C2 |
| D23 | rear driver PWMB | EN | not used |

Motors (each has 6 wires: white M1, red M2, black VCC, blue GND, green C1, yellow C2):

| Motor | M1 (white) | M2 (red) | C1 (green) | C2 (yellow) |
|---|---|---|---|---|
| Front left | front AO1 | front AO2 | D4 | D35 |
| Front right | front BO1 | front BO2 | RX2 (16) | VN (39) |
| Rear left | rear AO1 | rear AO2 | TX2 (17) | VP (36) |
| Rear right | rear BO1 | rear BO2 | D5 | D15 |

## Pickup-arm servo (MG90S) - one per robot

| MG90S wire | Goes to |
|---|---|
| orange (signal) | ESP32 **D2** |
| red (+) | LM2596 **OUT+ (5.0 V)** - NOT the ESP32 5V/VIN pin, NOT the 2S battery |
| brown (-) | LM2596 **OUT-** (the common ground) |

- Solder a **470-1000 uF, >= 10 V electrolytic capacitor** across the servo's red/brown, close to the servo
  plug (stripe = minus = brown). The MG90S pulls ~0.7 A in bursts and when it stalls; without the
  capacitor that dip can reset the ESP32 (brown-out).
- Take the servo's 5 V and ground straight from the LM2596 output terminals (not daisy-chained through the
  ESP32 board), so its current doesn't run through the ESP32's ground.
- D2 is a boot pin: if a USB upload fails with "wrong boot mode", unplug the servo signal for the upload.
  (D2 also drives the board's blue LED on many DevKits - it will glow/flicker; harmless.)
- The 3.3 V signal from the ESP32 is fine for the MG90S - no level shifter needed.
- At power-on the servo gets no signal (limp, it does not move). It starts with the first servo / arm command.
- Claw angles - a higher angle closes it, but each robot's claw sits differently on its servo, so the angles are
  per robot (`arm_deg_by_tag` in laptop/arena_calib.json, by marker id):
  - bot 1 (marker 11, 2026-10-30): grip 58, open 30, wide open 20
  - bot 2 (marker 10, 2026-11-03): grip 44, open 30, wide open 20 - its jaws already TOUCH at 51 (no cube), so
    bot 1's 58 would jam it
  The firmware holds every servo command to 20-60 deg (`ARM_MIN` / `ARM_MAX` in robot_main.ino).
- Not sure where the servo is? `nudge <deg> [pulses]` sends only a few pulses, so it moves just a little.
- Fitting the claw: power the robot, send `arm stow` (the servo goes to 30 deg = open), THEN press the claw
  onto the spline in its open position. Then check / calibrate:
  `python laptop/tools/arm_calibrate.py ROBOT_IP`.

