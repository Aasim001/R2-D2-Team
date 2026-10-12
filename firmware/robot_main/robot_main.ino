// =====================================================================
//  robot_main.ino  -  ONE firmware for the pickup robot
//
//   * 4 N20 motors on 2x TB6612FNG  (STBY on BOTH drivers wired to 3V3)
//   * 4 hall encoders (C1 + C2 on every motor)
//   * Straight-drive assist: when driving straight, all 4 wheels are held
//     at the same speed and left/right travel is kept equal, so the robot
//     goes straight. In turns, the two wheels on each side are matched.
//   * MG90S pickup-arm servo (smooth moves, 4 presets)
//   * Battery voltage + low-battery motor cutoff
//   * Control from BOTH:
//       - UDP port 4210  (laptop: follow_tape.py, bot.py)   - fast
//           drive packet = 8 bytes: 'D','R', int16 left, int16 right, uint16 seq
//           anything else = text command (see handleText)
//       - Web page http://car1.local (or the IP)            - phone / laptop
//   * Over-the-air updates (OTA): after ONE upload over USB, new firmware can be sent over
//     Wi-Fi (python firmware/upload.py --ota car1.local). Password protected. Updates are only
//     accepted while the robot is STOPPED (checking the password takes ~1 s during which nothing
//     else runs); the robot restarts when it's done, or keeps the old firmware if it fails.
//   * Motor watchdog: a separate timer cuts the motors if the main loop ever stalls > 150 ms.
//
//  Board: "ESP32 Dev Module", Arduino-ESP32 core 3.x, partition scheme "Default 4MB" (has OTA room)
//  Wiring: docs/wiring.md
//  Wi-Fi name/password and the OTA password: secrets.h (edit that file, not this one)
//  Robot number (car1 / car2 / car3): upload.py --robot N (see ROBOT_NUM below)
// =====================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "esp_timer.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "secrets.h"

// ============================ SETTINGS ===============================
const char* WIFI_SSID    = SECRET_WIFI_SSID;     // 2.4 GHz network (set in secrets.h)
const char* WIFI_PASS    = SECRET_WIFI_PASS;
const char* OTA_PASSWORD = SECRET_OTA_PASSWORD;
// Which robot this is: 1, 2 or 3. Set when uploading:  python firmware/upload.py --usb COM5 --robot 2
// (uploading from the Arduino IDE instead gives robot 1, unless you change the 1 below)
#ifndef ROBOT_NUM
#define ROBOT_NUM 1
#endif
#define STR_(x) #x
#define STR(x) STR_(x)
const char* HOSTNAME  = "car" STR(ROBOT_NUM);   // car1.local, car2.local, car3.local
const uint16_t UDP_PORT = 4210;

// ---- motors: {PWM, IN1, IN2}  (index 0,2 = LEFT side, 1,3 = RIGHT side)
int MOTOR[4][3] = {
  {25, 26, 27},   // 0 front-left   front driver A
  {14, 32, 33},   // 1 front-right  front driver B
  {13, 18, 19},   // 2 rear-left    rear driver A
  {23, 21, 22},   // 3 rear-right   rear driver B
};
const char* WHEEL_NAME[4] = {"FL", "FR", "RL", "RR"};
const int PWM_FREQ = 1000;                 // Hz, 8-bit duty (0-255)

// ---- encoders: C1 (interrupt) and C2 (direction).  DRAM: read inside the ISR
DRAM_ATTR int ENC_C1[4]   = {4, 16, 17, 5};      // D4, RX2, TX2, D5
DRAM_ATTR int ENC_C2[4]   = {35, 39, 36, 15};    // D35, VN, VP, D15
DRAM_ATTR int ENC_SIGN[4] = {1, 1, 1, 1};        // -1 if a wheel counts backwards
                                                 // (only affects displayed counts)

// ---- straight-drive assist (works without knowing the gear ratio)
bool  ASSIST        = false;  // OFF at start-up; can be switched on the web page or by UDP
const float SYNC_KP = 0.8;    // instant correction for a speed mismatch
const float SYNC_KI = 2.0;    // how fast each wheel learns its trim (per second)
const float TRIM_MAX = 70;    // max PWM a wheel's trim may add/remove
const float HEAD_K   = 1.5;   // straightness: corrects the left/right travel difference
const float HEAD_MAX = 0.25;  // max straightness correction, fraction of speed
const float MIN_SYNC_RATE = 40;    // counts/s: below this the wheels are "not moving"
const float RAMP_PWM_PER_S = 1200; // speed-up rate (stopping is instant)

// ---- wheel balance: N20 motors differ by 10-20 %, so each wheel has a fixed gain (1.0 = as commanded,
// 0.9 = 10 % slower). It scales only the part of the PWM above the motors' dead zone, so small commands
// still turn the wheel. Measured by laptop/tools/wheel_balance.py (robot lifted), kept in the ESP32's flash.
// UDP: "gain" (show) | "gain rl 0.9" (set one) | "gain save" | "gain reset"
float wheelGain[4] = {1, 1, 1, 1};
const float WHEEL_DEAD_PWM = 70;   // below about this PWM an N20 doesn't turn at all
const float GAIN_MIN = 0.6, GAIN_MAX = 1.4;
Preferences prefs;

// ---- fail-safes
const unsigned long UDP_FAILSAFE_MS  = 300;   // laptop stopped sending -> stop
const unsigned long HTTP_FAILSAFE_MS = 1000;  // web page stopped sending -> stop

// ---- servo on D2 (MG90S). 2026-10-10: the owner fitted a gripping CLAW to it (see "claw" below); the presets
// below are from the earlier scissor-arm design (cad/pickup_bot_v2.scad) and are not used by the claw.
// At power-on the servo goes back to the LAST angle it had (saved in flash), ARM_STOW the very first time, so a
// claw that was left closed is not driven into itself at start-up (a stalled servo can reset the ESP32).
const int SERVO_PIN = 2;
int ARM_STOW = 20, ARM_CARRY = 43, ARM_RELEASE = 50, ARM_REACH = 74;

// ---- battery: VBAT -100k- D34 -47k- GND
const int   BATT_PIN   = 34;
const float BATT_RATIO = (100.0 + 47.0) / 47.0;
const float LOW_BATT_V = 6.4;     // 2S Li-ion: stop motors below this (3.2 V/cell)

// ============================ STATE ==================================
WebServer server(80);
WiFiUDP   udp;

volatile long encCount[4] = {0, 0, 0, 0};
volatile uint32_t encEdges[4] = {0, 0, 0, 0};   // C1 edges seen, whatever C2 says (wiring check: if this
                                                // grows but the count doesn't, C2 is the dead wire)
portMUX_TYPE  encMux = portMUX_INITIALIZER_UNLOCKED;
long  prevCount[4] = {0, 0, 0, 0};
float rateSigned[4] = {0, 0, 0, 0};   // counts/s, filtered
float rateAbs[4]    = {0, 0, 0, 0};
float segDist[4]    = {0, 0, 0, 0};   // |counts| since the current straight run began
float trim[2][4]    = {{0}};          // [0 = forward, 1 = backward][wheel]
bool  encOk[4]      = {true, true, true, true};
float stuckT[4]     = {0, 0, 0, 0};

int   cmdL = 0, cmdR = 0;             // requested side speeds  (-255..255)
float rampL = 0, rampR = 0;           // after the speed-up ramp
int   pwmOut[4] = {0, 0, 0, 0};       // what each motor actually gets
bool  straightRun = false;
int   straightDir = 0;

unsigned long lastCmdMs = 0, failsafeMs = HTTP_FAILSAFE_MS;
uint16_t lastSeq = 0;
unsigned long lastUdpDriveMs = 0;

float battV = 0;  bool battPresent = false;  bool lowBatt = false;  float lowT = 0;
int   servoNow = 0, servoTarget = 0;
const int SERVO_MS_PER_DEG = 8;       // servo speed: 0 -> 180 deg in ~1.5 s
// "Spin servo" test button: sweeps back and forth between these until pressed again (MG90S: 180 deg servo; the
// ends are kept 10 deg away from its end stops - pushing into an end stop stalls it and draws ~1 A)
const int SPIN_LO = 10, SPIN_HI = 170;
bool servoSpin = false;
// ---- claw: OPEN / CLOSE move it slowly while the button is HELD (it stops CLAW_HOLD_MS after the last command,
// like the drive buttons); GRIP squeezes gripSqueeze degrees past the point where CLOSE was let go.
// A hobby servo has no force setting - it pushes until it reaches its angle - so the grip force is set by how far
// past the object it is told to go: close until the claw touches the object, let go, press GRIP.
// (gentle ~3-5 deg, firm ~12-15 deg; more = more force and current, and a stalled MG90S draws up to ~1 A)
const int CLAW_MS_PER_DEG = 22;       // claw speed ~45 deg/s: slow enough to stop on the object
const unsigned long CLAW_HOLD_MS = 500;
int servoMsPerDeg = SERVO_MS_PER_DEG; // speed of the current move
int clawDir = 0;                      // +1 closing, -1 opening, 0 still
unsigned long clawCmdMs = 0;
int clawContact = -1;                 // servo angle where CLOSE was let go (the claw on the object), -1 = unknown
int clawCloseSign = 1;                // +1: closing = bigger servo angle, -1: smaller ("Swap open/close", saved)
int gripSqueeze = 4;                  // GRIP / PICK: degrees past the touch point (saved; SG90: keep it gentle)
// PICK / DROP (saved once with "Save open" / "Save touch"): the claw's open (= drop) angle and the angle where its
// fingers just touch the cube. PICK = touch + squeeze, DROP = open; the closing direction is touch - open.
// (Owner's claw, 2026-10-10: two geared arms on a TowerPro SG90 - plastic gears, so keep the squeeze gentle.)
int clawOpenDeg = -1, clawTouchDeg = -1;
int servoSaved = -1;                  // last angle saved to flash
// timing check (status "loopMaxUs" / "servoWrUs"): the slowest loop() pass and servo write since the last status.
// 2026-10-10: the servo moved at ~40 ms/deg instead of 8 and loop() slowed only while it moved - find out why.
volatile unsigned long loopMaxUs = 0, servoWrMaxUs = 0;

IPAddress teleIP;  uint16_t telePort = 0;  bool teleOn = false;
volatile bool otaActive = false;      // an over-the-air update is running: motors stay off
volatile unsigned long lastLoopMs = 0;  // heartbeat of loop(), checked by the motor watchdog
volatile bool wdStopped = false;      // the watchdog cut the motors (loop() had stalled)
const unsigned long LOOP_STALL_MS = 150;
esp_timer_handle_t wdTimer;

// ============================ ENCODERS ===============================
static inline int IRAM_ATTR fastRead(int pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1
                  : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}

void IRAM_ATTR encISR(void* arg) {
  int i = (int)(intptr_t)arg - 1;     // (registered as wheel + 1: a NULL argument for wheel 0 would make the
                                      //  core call the ISR with no argument at all)
  int step = (fastRead(ENC_C1[i]) == fastRead(ENC_C2[i])) ? ENC_SIGN[i] : -ENC_SIGN[i];
  portENTER_CRITICAL_ISR(&encMux);
  encCount[i] += step;
  encEdges[i]++;
  portEXIT_CRITICAL_ISR(&encMux);
}

long readCount(int i) {
  portENTER_CRITICAL(&encMux);
  long c = encCount[i];
  portEXIT_CRITICAL(&encMux);
  return c;
}

// ============================ MOTORS =================================
void motorRaw(int i, int s) {
  s = constrain(s, -255, 255);
  digitalWrite(MOTOR[i][1], s > 0 ? HIGH : LOW);
  digitalWrite(MOTOR[i][2], s < 0 ? HIGH : LOW);
  ledcWrite(MOTOR[i][0], abs(s));
  pwmOut[i] = s;
}

void setCommand(int l, int r, unsigned long failsafe) {
  cmdL = constrain(l, -255, 255);
  cmdR = constrain(r, -255, 255);
  lastCmdMs = millis();
  failsafeMs = failsafe;
}

void stopNow() {
  cmdL = cmdR = 0;
  rampL = rampR = 0;
  for (int i = 0; i < 4; i++) motorRaw(i, 0);
}

// web-page directions
void driveDir(char c, int s) {
  switch (c) {
    case 'f': setCommand( s,  s, HTTP_FAILSAFE_MS); break;
    case 'b': setCommand(-s, -s, HTTP_FAILSAFE_MS); break;
    case 'l': setCommand(-s,  s, HTTP_FAILSAFE_MS); break;
    case 'r': setCommand( s, -s, HTTP_FAILSAFE_MS); break;
    default:  stopNow(); lastCmdMs = millis(); break;
  }
}

float approach(float cur, float target, float maxUp) {
  if (target == 0) return 0;                           // stop instantly
  if ((cur > 0 && target < 0) || (cur < 0 && target > 0)) cur = 0;  // reverse: from 0
  if (fabs(target) < fabs(cur)) return target;         // slowing down: instant
  float d = target - cur;
  if (d >  maxUp) return cur + maxUp;
  if (d < -maxUp) return cur - maxUp;
  return target;
}

// ============================ CONTROL LOOP (50 Hz) ===================
void syncGroup(const int* idx, int n, const float* base, float* corr, float dt) {
  // robust average speed: ignore wheels reading far below the fastest one
  // (a dead / unplugged encoder must not drag the average down)
  float mx = 0;
  for (int j = 0; j < n; j++) { int i = idx[j]; if (encOk[i] && base[i] != 0) mx = fmaxf(mx, rateAbs[i]); }
  float sum = 0; int k = 0;
  for (int j = 0; j < n; j++) { int i = idx[j]; if (encOk[i] && base[i] != 0 && rateAbs[i] >= 0.3f * mx) { sum += rateAbs[i]; k++; } }
  if (k == 0) return;
  float avg = sum / k;
  if (avg < MIN_SYNC_RATE) return;                     // not really moving yet

  // encoder health: a wheel that reads ~nothing while the others move
  for (int j = 0; j < n; j++) {
    int i = idx[j];
    if (base[i] == 0) continue;
    if (rateAbs[i] < 0.1f * avg) {
      stuckT[i] += dt;
      if (stuckT[i] > 0.6f && encOk[i]) {
        encOk[i] = false;
        trim[0][i] = trim[1][i] = 0;
        Serial.printf("Encoder %s not counting - excluded from straight assist\n", WHEEL_NAME[i]);
      }
    } else {
      stuckT[i] = 0;
      if (!encOk[i] && rateAbs[i] > 0.5f * avg) { encOk[i] = true; Serial.printf("Encoder %s OK again\n", WHEEL_NAME[i]); }
    }
  }

  // learn each wheel's trim, then re-centre the trims so they only move
  // speed between wheels (a slow wheel gets +, a fast wheel gets -)
  float e[4] = {0, 0, 0, 0}; float tsum = 0; int tk = 0;
  for (int j = 0; j < n; j++) {
    int i = idx[j];
    if (base[i] == 0 || !encOk[i]) continue;
    e[i] = (avg - rateAbs[i]) / avg;                  // +ve = this wheel is too slow
    int d = base[i] < 0 ? 1 : 0;
    trim[d][i] = constrain(trim[d][i] + SYNC_KI * e[i] * fabsf(base[i]) * dt, -TRIM_MAX, TRIM_MAX);
    tsum += trim[d][i]; tk++;
  }
  for (int j = 0; j < n; j++) {
    int i = idx[j];
    if (base[i] == 0 || !encOk[i]) continue;
    int d = base[i] < 0 ? 1 : 0;
    trim[d][i] -= tsum / tk;
    corr[i] += trim[d][i] + SYNC_KP * e[i] * fabsf(base[i]);
  }
}

void controlStep(float dt) {
  // 1. wheel speeds from the encoders
  for (int i = 0; i < 4; i++) {
    long c = readCount(i);
    long d = c - prevCount[i];
    prevCount[i] = c;
    rateSigned[i] = 0.6 * rateSigned[i] + 0.4 * (d / dt);
    rateAbs[i] = fabs(rateSigned[i]);
    segDist[i] += labs(d);
  }

  // 2. fail-safes
  if (millis() - lastCmdMs > failsafeMs) { cmdL = cmdR = 0; }
  if (lowBatt) { cmdL = cmdR = 0; }

  // 3. speed-up ramp
  rampL = approach(rampL, cmdL, RAMP_PWM_PER_S * dt);
  rampR = approach(rampR, cmdR, RAMP_PWM_PER_S * dt);
  float base[4] = {rampL, rampR, rampL, rampR};

  // 4. is this a straight run?  (both sides asked for the same speed)
  bool straight = (cmdL == cmdR) && cmdL != 0;
  int dir = cmdL > 0 ? 1 : -1;
  if (straight && (!straightRun || dir != straightDir)) {
    for (int i = 0; i < 4; i++) segDist[i] = 0;        // new straight run: start measuring
  }
  straightRun = straight;
  straightDir = straight ? dir : 0;

  float corr[4] = {0, 0, 0, 0};
  if (ASSIST) {
    // 5. match wheel speeds: all 4 when straight, front+rear on each side in turns
    if (straight) { int all[4] = {0, 1, 2, 3}; syncGroup(all, 4, base, corr, dt); }
    else { int L[2] = {0, 2}, R[2] = {1, 3}; syncGroup(L, 2, base, corr, dt); syncGroup(R, 2, base, corr, dt); }

    // 6. keep left and right travel equal (this is what makes it go straight)
    if (straight) {
      float dl = 0, dr = 0, rl = 0, rr = 0; int nl = 0, nr = 0;
      for (int i = 0; i < 4; i++) {
        if (!encOk[i]) continue;
        if (i % 2 == 0) { dl += segDist[i]; rl += rateAbs[i]; nl++; }
        else            { dr += segDist[i]; rr += rateAbs[i]; nr++; }
      }
      if (nl && nr) {
        float avgRate = (rl / nl + rr / nr) / 2;
        if (avgRate > MIN_SYNC_RATE) {
          float aheadSec = (dl / nl - dr / nr) / avgRate;          // +ve = left side is ahead
          float h = constrain(HEAD_K * aheadSec, -HEAD_MAX, HEAD_MAX) * fabs(base[0]);
          corr[0] -= h; corr[2] -= h;                               // slow the side that is ahead
          corr[1] += h; corr[3] += h;
        }
      }
    }
  }

  // a wheel whose encoder is not counting copies the correction of the
  // other wheel on the same side (front <-> rear)
  if (ASSIST) for (int i = 0; i < 4; i++) if (!encOk[i] && encOk[i ^ 2]) corr[i] = corr[i ^ 2];

  // 7. combine; if a wheel would need more than 255, scale ALL wheels down by the same factor above the
  //    dead zone (keeps the commanded turn and the wheel balance; subtracting one amount from every wheel
  //    would stall the inner wheels of a fast turn)
  float mag[4]; float maxMag = 0;
  for (int i = 0; i < 4; i++) {
    float b = fabsf(base[i]);
    if (b > WHEEL_DEAD_PWM) b = WHEEL_DEAD_PWM + (b - WHEEL_DEAD_PWM) * wheelGain[i];   // wheel balance
    mag[i] = base[i] == 0 ? 0 : fmaxf(0.0f, b + corr[i]);
    maxMag = fmaxf(maxMag, mag[i]);
  }
  float k = maxMag > 255 ? (255 - WHEEL_DEAD_PWM) / (maxMag - WHEEL_DEAD_PWM) : 1.0f;
  for (int i = 0; i < 4; i++) {
    float m = mag[i] > WHEEL_DEAD_PWM ? WHEEL_DEAD_PWM + (mag[i] - WHEEL_DEAD_PWM) * k : mag[i];
    int mi = base[i] == 0 ? 0 : (int)constrain(m, 0.0f, 255.0f);
    motorRaw(i, base[i] < 0 ? -mi : mi);
  }
}

// ============================ SERVO ==================================
void servoWriteAngle(int deg) {
  int us = map(constrain(deg, 0, 180), 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, (uint32_t)us * 16383 / 20000);  // 14-bit at 50 Hz
}

void clawStop() {                                       // OPEN / CLOSE let go: stay where it is
  if (clawDir == 0) return;
  if (clawDir > 0) clawContact = servoNow;              // (closed until it touched -> GRIP squeezes from here)
  clawDir = 0;
  servoTarget = servoNow;
}

void updateServo() {                // 1 degree every servoMsPerDeg, by the clock: if loop() is slow it takes
  static unsigned long t = 0;       // several degrees per step, so the speed stays the same
  static unsigned long stillSince = 0;
  unsigned long now = millis();
  if (clawDir != 0 && now - clawCmdMs > CLAW_HOLD_MS) clawStop();   // button let go / page gone
  if (servoSpin && servoNow == servoTarget)            // spin test: at one end -> go to the other
    servoTarget = (servoNow >= SPIN_HI) ? SPIN_LO : SPIN_HI;
  if (servoNow == servoTarget) {
    t = now;
    if (!stillSince) stillSince = now;
    if (servoNow != servoSaved && now - stillSince > 2000) {   // still for 2 s: remember it for the next power-on
      prefs.putInt("servo", servoNow);
      servoSaved = servoNow;
    }
    return;
  }
  stillSince = 0;
  int steps = (now - t) / servoMsPerDeg;
  if (steps <= 0) return;
  t = (now - t > 100) ? now : t + (unsigned long)steps * servoMsPerDeg;   // (no backlog after a stall)
  int diff = servoTarget - servoNow;
  servoNow = abs(diff) <= steps ? servoTarget : servoNow + (diff > 0 ? steps : -steps);
  unsigned long w0 = micros();
  servoWriteAngle(servoNow);
  unsigned long w = micros() - w0;
  if (w > servoWrMaxUs) servoWrMaxUs = w;
}

void servoGo(int deg) {                                 // plain "go to this angle" (slider, servo N, presets)
  servoSpin = false;
  clawDir = 0;
  servoMsPerDeg = SERVO_MS_PER_DEG;
  servoTarget = constrain(deg, 0, 180);
}

void setSpin(bool on) {                                 // start / stop the spin test (stops where it is)
  clawDir = 0;
  servoMsPerDeg = SERVO_MS_PER_DEG;
  servoSpin = on;
  if (!on) servoTarget = servoNow;
}

int clawClosing() {                                     // +1 / -1: which way (in servo degrees) closes the claw
  if (clawOpenDeg >= 0 && clawTouchDeg >= 0 && clawOpenDeg != clawTouchDeg) return clawTouchDeg > clawOpenDeg ? 1 : -1;
  return clawCloseSign;                                 // (the saved points decide; without them the Swap setting)
}

// Save open / Save touch back off CLAW_SAVE_BACKOFF degrees: the claw keeps moving (~45 deg/s) for the moment the
// "let go" takes to arrive. The rest of the overshoot (the eye-to-finger delay) is removed with the 1-degree
// nudge buttons before saving: close until the fingers touch, nudge open until the servo stops buzzing, Save touch.
const int CLAW_SAVE_BACKOFF = 2;
// Web hold presses carry a number (n); a late, re-sent open/close from a press that was already let go is ignored.
unsigned long clawStoppedIds[4] = {0, 0, 0, 0};
int clawStoppedNext = 0;

bool clawMove(const String& d, unsigned long pressId);

// claw: "open" / "close" (hold-to-move: repeat at least every CLAW_HOLD_MS) | "stop" | "grip" | "swap"
//       | "nudgeopen" / "nudgeclose" (1 degree) | "setopen" / "settouch" (save where it is now) | "pick" | "drop".
// pressId: the web page's press number (0 = none, e.g. UDP). Returns "" or what went wrong.
String clawCommand(const String& d, unsigned long pressId) {
  if (d == "setopen" || d == "settouch") {
    clawStop();
    int closing = clawClosing();
    int a = constrain(servoNow + (d == "setopen" ? closing : -closing) * CLAW_SAVE_BACKOFF, SPIN_LO, SPIN_HI);
    if (d == "setopen") { clawOpenDeg = a;  prefs.putInt("clawopen", a); }
    else                { clawTouchDeg = a; prefs.putInt("clawtouch", a); }
    servoSpin = false;
    servoMsPerDeg = CLAW_MS_PER_DEG;
    servoTarget = a;                                    // (back off now too: no stall while setting up)
    if (clawOpenDeg >= 0 && clawTouchDeg >= 0 && clawOpenDeg != clawTouchDeg && clawClosing() != clawCloseSign) {
      clawCloseSign = clawClosing();                    // the Close button now goes the same way as PICK
      prefs.putInt("clawdir", clawCloseSign);
    }
    return "";
  }
  if (d == "nudgeopen" || d == "nudgeclose") {          // 1 degree, to find the exact touch / open point
    clawStop();
    servoSpin = false;
    servoMsPerDeg = CLAW_MS_PER_DEG;
    servoTarget = constrain(servoTarget + (d == "nudgeclose" ? 1 : -1) * clawClosing(), SPIN_LO, SPIN_HI);
    return "";
  }
  if (d == "pick" || d == "drop") {
    int to = d == "pick" ? clawTouchDeg : clawOpenDeg;
    if (to < 0) return d == "pick" ? "save the touch point first (Save touch)" : "save the open position first (Save open)";
    servoSpin = false;
    clawDir = 0;
    servoMsPerDeg = CLAW_MS_PER_DEG;
    if (d == "pick") { clawContact = clawTouchDeg; to = clawTouchDeg + clawClosing() * gripSqueeze; }
    else clawContact = -1;
    servoTarget = constrain(to, SPIN_LO, SPIN_HI);
    return "";
  }
  return clawMove(d, pressId) ? "" : "use open|close|stop|grip|swap|nudgeopen|nudgeclose|setopen|settouch|pick|drop";
}

bool clawMove(const String& d, unsigned long pressId) {
  if (d == "open" || d == "close") {
    if (pressId)
      for (int i = 0; i < 4; i++) if (clawStoppedIds[i] == pressId) return true;   // late request: already let go
    servoSpin = false;
    clawDir = (d == "close") ? 1 : -1;
    if (clawDir < 0) clawContact = -1;                  // opened: the touch point is gone
    clawCmdMs = millis();
    servoMsPerDeg = CLAW_MS_PER_DEG;
    servoTarget = (clawDir * clawClosing() > 0) ? SPIN_HI : SPIN_LO;   // keeps going until let go
  } else if (d == "stop") {
    if (pressId) { clawStoppedIds[clawStoppedNext] = pressId; clawStoppedNext = (clawStoppedNext + 1) % 4; }
    clawStop();
  } else if (d == "grip") {                             // squeeze gripSqueeze past the touch point (not more per press)
    servoSpin = false;
    clawDir = 0;
    servoMsPerDeg = CLAW_MS_PER_DEG;
    if (clawContact < 0) clawContact = clawTouchDeg >= 0 ? clawTouchDeg : servoNow;
    servoTarget = constrain(clawContact + clawClosing() * gripSqueeze, SPIN_LO, SPIN_HI);
  } else if (d == "swap") {                             // flip Open / Close; the saved points go (they set the direction)
    clawStop();
    clawCloseSign = -clawCloseSign;
    clawContact = -1;
    clawOpenDeg = clawTouchDeg = -1;
    prefs.putInt("clawdir", clawCloseSign);
    prefs.putInt("clawopen", -1);
    prefs.putInt("clawtouch", -1);
  } else return false;
  return true;
}

void servoFreeze() {                                    // "stop": every servo move stops where it is
  clawStop();
  servoSpin = false;
  servoTarget = servoNow;
}

void setGripSqueeze(int deg) {
  gripSqueeze = constrain(deg, 1, 30);
  prefs.putInt("grip", gripSqueeze);
}

bool armPreset(const String& p) {
  if      (p == "stow")    servoGo(ARM_STOW);
  else if (p == "reach")   servoGo(ARM_REACH);
  else if (p == "carry")   servoGo(ARM_CARRY);
  else if (p == "release") servoGo(ARM_RELEASE);
  else return false;
  return true;
}

// ============================ BATTERY ================================
void updateBattery(float dt) {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(BATT_PIN);
  float v = (mv / 8.0f) / 1000.0f * BATT_RATIO;
  battV = battV == 0 ? v : 0.8f * battV + 0.2f * v;
  battPresent = battV > 3.0;                            // no divider fitted -> reads ~0
  if (!battPresent) { lowBatt = false; return; }
  if (battV < LOW_BATT_V) { lowT += dt; if (lowT > 3 && !lowBatt) { lowBatt = true; Serial.println("LOW BATTERY - motors off"); } }
  else { lowT = 0; if (battV > LOW_BATT_V + 0.3) lowBatt = false; }
}

// ============================ STATUS =================================
String statusJson() {
  String j = "{\"name\":\"" + String(HOSTNAME) + "\",\"batt\":" + String(battV, 2) + ",\"low\":" + String(lowBatt) +
             ",\"servo\":" + String(servoNow) + ",\"assist\":" + String(ASSIST) +
             ",\"cmd\":[" + String(cmdL) + "," + String(cmdR) + "]" +
             ",\"rssi\":" + String(WiFi.RSSI());
  j += ",\"pwm\":[";  for (int i = 0; i < 4; i++) j += String(pwmOut[i]) + (i < 3 ? "," : "]");
  j += ",\"cnt\":[";  for (int i = 0; i < 4; i++) j += String(readCount(i)) + (i < 3 ? "," : "]");
  j += ",\"rate\":["; for (int i = 0; i < 4; i++) j += String((int)rateSigned[i]) + (i < 3 ? "," : "]");
  j += ",\"ok\":[";   for (int i = 0; i < 4; i++) j += String(encOk[i]) + (i < 3 ? "," : "]");
  j += ",\"trim\":["; for (int i = 0; i < 4; i++) j += String((int)trim[0][i]) + (i < 3 ? "," : "]");
  j += ",\"gain\":["; for (int i = 0; i < 4; i++) j += String(wheelGain[i], 3) + (i < 3 ? "," : "]");
  // encoder wiring check: C1 edges counted, and the live C1 / C2 levels as two digits (e.g. 10 = C1 high, C2 low)
  j += ",\"edges\":["; for (int i = 0; i < 4; i++) j += String(encEdges[i]) + (i < 3 ? "," : "]");
  j += ",\"lv\":[";   for (int i = 0; i < 4; i++)
    j += "\"" + String(digitalRead(ENC_C1[i])) + String(digitalRead(ENC_C2[i])) + "\"" + (i < 3 ? "," : "]");
  j += ",\"spin\":" + String(servoSpin);
  j += ",\"claw\":" + String(clawDir) + ",\"squeeze\":" + String(gripSqueeze) + ",\"clawSign\":" + String(clawCloseSign) +
       ",\"contact\":" + String(clawContact) + ",\"open\":" + String(clawOpenDeg) + ",\"touch\":" + String(clawTouchDeg) +
       ",\"target\":" + String(servoTarget);
  j += ",\"loopMaxUs\":" + String(loopMaxUs) + ",\"servoWrUs\":" + String(servoWrMaxUs);
  loopMaxUs = servoWrMaxUs = 0;                         // (the slowest since the last status)
  j += "}";
  return j;
}

// ============================ UDP ====================================
void udpReply(const String& s) {
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(s);
  udp.endPacket();
}

// ---- wheel balance (see wheelGain)
String gainText() {
  String t = "gain";
  for (int i = 0; i < 4; i++) t += String(" ") + WHEEL_NAME[i] + " " + String(wheelGain[i], 3);
  return t;
}

void saveGains() {
  for (int i = 0; i < 4; i++) { char key[3] = {'g', (char)('0' + i), 0}; prefs.putFloat(key, wheelGain[i]); }
}

void loadGains() {
  for (int i = 0; i < 4; i++) {
    char key[3] = {'g', (char)('0' + i), 0};
    float v = prefs.getFloat(key, 1.0f);
    wheelGain[i] = (v >= GAIN_MIN && v <= GAIN_MAX) ? v : 1.0f;   // (a NaN fails both tests -> 1.0)
  }
}

// text commands:  stop | arm stow|reach|carry|release | servo <deg> | spin on|off
//                 claw open|close (repeat < 0.5 s apart while moving) | claw stop|grip|swap | claw squeeze <deg>
//                 assist on|off | tele on|off | trim reset | status
//                 gain | gain fl|fr|rl|rr <0.6-1.4> | gain save | gain reset
void handleText(String s) {
  s.trim(); s.toLowerCase();
  if (s == "stop") { stopNow(); lastCmdMs = millis(); servoFreeze(); udpReply("ok stop"); }
  else if (s == "gain") udpReply(gainText());
  else if (s == "gain save") { saveGains(); udpReply("ok saved " + gainText()); }
  else if (s == "gain reset") { for (int i = 0; i < 4; i++) wheelGain[i] = 1; saveGains(); udpReply("ok reset " + gainText()); }
  else if (s.startsWith("gain ")) {                    // "gain rl 0.9"
    String w = s.substring(5);
    int sp = w.indexOf(' ');
    String nm = sp > 0 ? w.substring(0, sp) : w;
    nm.toUpperCase();
    int i = -1;
    for (int k = 0; k < 4; k++) if (nm == WHEEL_NAME[k]) i = k;
    float g = sp > 0 ? w.substring(sp + 1).toFloat() : 0;
    if (i < 0 || !(g >= GAIN_MIN && g <= GAIN_MAX)) udpReply("err use: gain fl|fr|rl|rr 0.6-1.4");   // (rejects nan)
    else { wheelGain[i] = g; udpReply("ok " + gainText() + " (not saved yet: gain save)"); }
  }
  else if (s.startsWith("arm "))   udpReply(armPreset(s.substring(4)) ? "ok " + s : "err unknown arm position");
  else if (s.startsWith("servo ")) { servoGo(s.substring(6).toInt()); udpReply("ok servo " + String(servoTarget)); }
  else if (s == "spin on" || s == "spin off") { setSpin(s == "spin on"); udpReply("ok " + s); }
  else if (s.startsWith("claw squeeze ")) { setGripSqueeze(s.substring(13).toInt()); udpReply("ok claw squeeze " + String(gripSqueeze)); }
  else if (s.startsWith("claw ")) { String e = clawCommand(s.substring(5), 0); udpReply(e.length() ? "err " + e : "ok " + s); }
  else if (s == "assist on")  { ASSIST = true;  udpReply("ok assist on"); }
  else if (s == "assist off") { ASSIST = false; udpReply("ok assist off"); }
  else if (s == "tele on")  { teleOn = true; teleIP = udp.remoteIP(); telePort = udp.remotePort(); udpReply("ok tele on"); }
  else if (s == "tele off") { teleOn = false; udpReply("ok tele off"); }
  else if (s == "trim reset") { memset(trim, 0, sizeof(trim)); for (int i = 0; i < 4; i++) encOk[i] = true; udpReply("ok trim reset"); }
  else if (s == "status") udpReply(statusJson());
  else udpReply("err unknown command");
}

void handleUdp() {
  while (udp.parsePacket() > 0) {
    uint8_t buf[64];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len == 8 && buf[0] == 'D' && buf[1] == 'R') {  // drive: "DR" + int16 L, int16 R, uint16 seq
      int16_t l, r; uint16_t seq;
      memcpy(&l, buf + 2, 2); memcpy(&r, buf + 4, 2); memcpy(&seq, buf + 6, 2);
      bool newer = (int16_t)(seq - lastSeq) > 0 || millis() - lastUdpDriveMs > UDP_FAILSAFE_MS;
      if (!newer) continue;                            // late / out-of-order packet
      lastSeq = seq;
      lastUdpDriveMs = millis();
      setCommand(l, r, UDP_FAILSAFE_MS);
    } else if (len > 0) {
      buf[len] = 0;
      handleText(String((char*)buf));
    }
  }
}

void sendTelemetry() {
  static unsigned long t = 0;
  if (!teleOn || millis() - t < 100) return;
  t = millis();
  udp.beginPacket(teleIP, telePort);
  udp.print(statusJson());
  udp.endPacket();
}

// ============================ WEB PAGE ===============================
const char PAGE[] = R"HTML(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Pickup Bot</title>
<style>
body{font-family:sans-serif;margin:14px;background:#111;color:#eee;text-align:center;user-select:none;-webkit-user-select:none}
.g{display:grid;grid-template-columns:repeat(3,84px);gap:10px;justify-content:center;margin:14px 0}
button{height:84px;border:0;border-radius:14px;font-size:28px;background:#333;color:#fff;touch-action:none}
button:active{background:#2e86de}
#s{background:#c0392b;font-size:17px}
.row{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;max-width:340px;margin:10px auto}
.row button{height:46px;font-size:15px;background:#5b4bb7}
#as{background:#1e8449}
input{width:80%}
#st{font-family:monospace;font-size:13px;text-align:left;max-width:340px;margin:12px auto;white-space:pre;background:#1c1c1c;padding:10px;border-radius:10px}
</style></head><body>
<h3>Pickup Bot</h3>
<div class="g">
 <div></div><button data-c="f">&#9650;</button><div></div>
 <button data-c="l">&#9664;</button><button id="s">STOP</button><button data-c="r">&#9654;</button>
 <div></div><button data-c="b">&#9660;</button><div></div>
</div>
<div>Speed <span id="v">180</span></div>
<input type="range" min="90" max="255" value="180" oninput="spd=this.value;v.textContent=spd">
<div class="row"><button id="as" onclick="toggleAssist()">Straight assist: ON</button><button onclick="fetch('/trimreset')">Reset trims</button></div>
<div class="row">
 <button onclick="arm('stow')">Stow</button><button onclick="arm('reach')">Reach</button>
 <button onclick="arm('carry')">Grab / carry</button><button onclick="arm('release')">Release</button>
</div>
<div class="row"><button id="sp" onclick="spin()">Spin servo</button></div>
<div>Claw: HOLD Open / Close, let go when it touches, then Grip</div>
<div class="row" style="grid-template-columns:repeat(3,1fr)"><button id="co">Open</button><button id="cc">Close</button><button onclick="claw('grip')">Grip</button></div>
<div>Grip strength <span id="gq">4</span>&deg; past the touch point (SG90: gentle 2-4, firm 8)</div>
<input type="range" min="2" max="20" value="4" id="gs" oninput="gq.textContent=this.value" onchange="fetch('/claw?squeeze='+this.value)">
<div class="row"><button onclick="claw('swap')">Swap open / close (clears Save open / touch)</button><span id="cs" style="align-self:center"></span></div>
<div>Pick &amp; drop - set once: Open wide, <b>Save open</b>. Cube in: Close until the fingers touch it, then <b>Open 1&deg;</b> until the servo stops buzzing, <b>Save touch</b>.</div>
<div class="row"><button onclick="claw('nudgeopen')">Open 1&deg;</button><button onclick="claw('nudgeclose')">Close 1&deg;</button></div>
<div class="row"><button onclick="claw('setopen')">Save open</button><button onclick="claw('settouch')">Save touch</button></div>
<div class="row"><button onclick="claw('pick')" style="background:#1e8449">PICK</button><button onclick="claw('drop')" style="background:#b9770e">DROP</button></div>
<div id="cm"></div>
<div>Servo angle <span id="sa">-</span>&deg; (calibration)</div>
<input type="range" min="0" max="180" value="20" id="sv" oninput="sa.textContent=this.value;fetch('/servo?a='+this.value)">
<div id="st">connecting...</div>
<script>
let cur='s', spd=180, assist=1;
const v=document.getElementById('v'), sa=document.getElementById('sa'), st=document.getElementById('st'), as=document.getElementById('as');
function send(){fetch(`/cmd?c=${cur}&s=${spd}`).catch(()=>{})}
function go(c){if(cur!==c){cur=c;send()}}
function arm(p){fetch('/arm?p='+p)}
const sp=document.getElementById('sp');
function showSpin(on){sp.textContent=on?'Stop servo':'Spin servo';sp.style.background=on?'#c0392b':''}
function spin(){fetch('/spin').then(r=>r.text()).then(t=>showSpin(t==='spinning'))}
const gq=document.getElementById('gq'), gs=document.getElementById('gs'), cs=document.getElementById('cs');
let clawT=null, press=Math.floor(Math.random()*1e9);   // press number: the robot ignores late repeats
const cm=document.getElementById('cm');
function claw(d,n){fetch('/claw?d='+d+(n?'&n='+n:'')).then(r=>r.text()).then(t=>{if(t!=='ok')cm.textContent=t;
 else if(['setopen','settouch','pick','drop'].includes(d))cm.textContent=d+': ok'}).catch(()=>{})}
[['co','open'],['cc','close']].forEach(([id,d])=>{const b=document.getElementById(id);   // hold to move
 b.onpointerdown=e=>{e.preventDefault();clearInterval(clawT);const n=++press;claw(d,n);clawT=setInterval(()=>claw(d,n),150)};
 b.onpointerup=b.onpointerleave=b.onpointercancel=()=>{if(clawT){clearInterval(clawT);clawT=null;claw('stop',press)}};
 b.oncontextmenu=e=>e.preventDefault();});
function toggleAssist(){fetch('/assist?on='+(assist?0:1))}
document.querySelectorAll('button[data-c]').forEach(b=>{
 b.onpointerdown=e=>{e.preventDefault();go(b.dataset.c)};
 b.onpointerup=b.onpointerleave=b.onpointercancel=()=>go('s');
 b.oncontextmenu=e=>e.preventDefault();
});
document.getElementById('s').onclick=()=>go('s');
setInterval(()=>{if(cur!=='s')send()},200);
const keys={ArrowUp:'f',ArrowDown:'b',ArrowLeft:'l',ArrowRight:'r'};
document.onkeydown=e=>{if(keys[e.key])go(keys[e.key])};
document.onkeyup=e=>{if(keys[e.key])go('s')};
document.addEventListener('visibilitychange',()=>{if(document.hidden){go('s');if(clawT){clearInterval(clawT);clawT=null;claw('stop',press)}}});
const N=['FL','FR','RL','RR'];
setInterval(()=>{fetch('/status').then(r=>r.json()).then(d=>{
 assist=d.assist; as.textContent='Straight assist: '+(assist?'ON':'OFF'); as.style.background=assist?'#1e8449':'#555';
 let t=`Battery ${d.batt.toFixed(2)} V ${d.low?' LOW - MOTORS OFF':''}\nServo   ${d.servo} deg   Wi-Fi ${d.rssi} dBm\n\nwheel  count   rate/s  pwm  trim enc\n`;
 for(let i=0;i<4;i++) t+=`${N[i]}  ${String(d.cnt[i]).padStart(7)} ${String(d.rate[i]).padStart(7)} ${String(d.pwm[i]).padStart(5)} ${String(d.trim[i]).padStart(4)}  ${d.ok[i]?'ok':'NO'}\n`;
 st.textContent=t; sa.textContent=d.servo; showSpin(d.spin);
 const svEl=document.getElementById('sv'); if(document.activeElement!==svEl)svEl.value=d.servo;
 if(document.activeElement!==gs){gs.value=d.squeeze;gq.textContent=d.squeeze}
 cs.textContent=(d.clawSign>0?'close = bigger angle':'close = smaller angle')+(d.contact>=0?', touched at '+d.contact+'°':'');
 cm.title='saved: open '+(d.open>=0?d.open+'°':'-')+', touch '+(d.touch>=0?d.touch+'°':'-');
 if(!cm.textContent)cm.textContent=cm.title;
 }).catch(()=>{st.textContent='no connection'})},400);
</script></body></html>
)HTML";

void setupWeb() {
  server.on("/", []() { server.send(200, "text/html", PAGE); });
  server.on("/cmd", []() {                              // /cmd?c=f&s=180
    char c = server.hasArg("c") ? server.arg("c")[0] : 's';
    int  s = server.hasArg("s") ? server.arg("s").toInt() : 180;
    driveDir(c, s);
    server.send(200, "text/plain", "ok");
  });
  server.on("/stop",   []() { stopNow(); lastCmdMs = millis(); server.send(200, "text/plain", "stopped"); });
  server.on("/servo",  []() { if (server.hasArg("a")) servoGo(server.arg("a").toInt()); server.send(200, "text/plain", "ok"); });
  server.on("/claw",   []() {     // /claw?d=open|close|stop|grip|swap|setopen|settouch|pick|drop   /claw?squeeze=8
    if (server.hasArg("squeeze")) setGripSqueeze(server.arg("squeeze").toInt());
    unsigned long n = server.hasArg("n") ? strtoul(server.arg("n").c_str(), nullptr, 10) : 0;
    String e = server.hasArg("d") ? clawCommand(server.arg("d"), n) : String("");
    server.send(e.length() ? 400 : 200, "text/plain", e.length() ? e : String("ok"));
  });
  server.on("/spin",   []() {                             // /spin = toggle, /spin?on=1 / ?on=0
    setSpin(server.hasArg("on") ? server.arg("on") == "1" : !servoSpin);
    server.send(200, "text/plain", servoSpin ? "spinning" : "stopped");
  });
  server.on("/arm",    []() { server.send(200, "text/plain", armPreset(server.arg("p")) ? "ok" : "unknown"); });
  server.on("/assist", []() { ASSIST = server.arg("on") == "1"; server.send(200, "text/plain", ASSIST ? "on" : "off"); });
  server.on("/trimreset", []() { memset(trim, 0, sizeof(trim)); for (int i = 0; i < 4; i++) encOk[i] = true; server.send(200, "text/plain", "ok"); });
  server.on("/status", []() { server.send(200, "application/json", statusJson()); });
  server.begin();
}

// ============================ OTA (update over Wi-Fi) ================
void setupOTA() {
  ArduinoOTA.setHostname(HOSTNAME);      // also starts mDNS as HOSTNAME.local
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setTimeout(10000);          // wait up to 10 s for a stalled chunk (default 1 s): on a weak
                                         // or busy Wi-Fi link (e.g. the laptop hotspot) TCP retransmits
                                         // can pause the transfer for a few seconds
  ArduinoOTA.onStart([]() {
    otaActive = true;
    stopNow();                           // never drive while the firmware is being replaced
    Serial.println("OTA update starting - motors off");
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static int lastTen = -1;
    int pct = total ? (int)(100ULL * done / total) : 0;
    if (pct / 10 != lastTen) { lastTen = pct / 10; Serial.printf("OTA %d%%\n", pct); }
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA done - restarting"); });
  ArduinoOTA.onError([](ota_error_t e) {
    otaActive = false;                   // failed: keep running the old firmware
    stopNow();
    while (udp.parsePacket() > 0) {}     // throw away drive packets that queued up during the update
    lastCmdMs = millis();
    Serial.printf("OTA failed (error %u) - still running the old firmware\n", (unsigned)e);
  });
  ArduinoOTA.begin();
  if (strcmp(OTA_PASSWORD, "change-me") == 0)
    Serial.println("WARNING: OTA password is still 'change-me' - set your own in secrets.h");
}

// ============================ WI-FI ==================================
// Join the network, and keep trying: if an attempt hasn't worked within 10 s, start a fresh one.
// (Calling WiFi.begin() only once can leave the ESP32 waiting forever, e.g. after a restart when
// the access point - like a laptop hotspot - briefly refuses a device it saw leave.)
void connectWiFi() {
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) { delay(250); Serial.print("."); }
    if (WiFi.status() != WL_CONNECTED) Serial.printf(" (no luck, status %d - trying again)\n", (int)WiFi.status());
  }
}

// Called from loop(): Wi-Fi lost for more than 10 s -> start a fresh connection attempt.
// (While it's lost no drive packets arrive, so the 300 ms fail-safe has already stopped the motors.)
void keepWiFi() {
  static unsigned long lostSince = 0;
  if (WiFi.status() == WL_CONNECTED) { lostSince = 0; return; }
  if (lostSince == 0) { lostSince = millis(); return; }
  if (millis() - lostSince > 10000) {
    Serial.println("Wi-Fi lost for 10 s - reconnecting");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lostSince = millis();
  }
}

// ============================ MOTOR WATCHDOG =========================
// Runs every 50 ms in its own task, independent of loop(). If loop() has not run for
// LOOP_STALL_MS (an OTA password check, a slow web client, anything), the motors are cut -
// the normal fail-safes live in loop() and can't act while it is stuck.
void motorWatchdog(void*) {
  if (!wdStopped && millis() - lastLoopMs > LOOP_STALL_MS) {
    for (int i = 0; i < 4; i++) {
      ledcWrite(MOTOR[i][0], 0);
      digitalWrite(MOTOR[i][1], LOW);
      digitalWrite(MOTOR[i][2], LOW);
    }
    wdStopped = true;
  }
}

void startWatchdog() {
  esp_timer_create_args_t args = {};
  args.callback = &motorWatchdog;
  args.name = "motor_wd";
  esp_timer_create(&args, &wdTimer);
  esp_timer_start_periodic(wdTimer, 50000);            // every 50 ms
}

// ============================ SETUP / LOOP ===========================
void setup() {
  Serial.begin(115200);
  prefs.begin("robot", false);                          // wheel balance, kept in flash
  loadGains();

  for (int i = 0; i < 4; i++) {                         // motors
    pinMode(MOTOR[i][1], OUTPUT);
    pinMode(MOTOR[i][2], OUTPUT);
    ledcAttach(MOTOR[i][0], PWM_FREQ, 8);
    motorRaw(i, 0);
  }

  lastLoopMs = millis();
  startWatchdog();                                      // motors are set up: guard them from now on

  for (int i = 0; i < 4; i++) {                         // encoders
    pinMode(ENC_C1[i], INPUT_PULLUP);
    pinMode(ENC_C2[i], ENC_C2[i] >= 34 ? INPUT : INPUT_PULLUP);   // 34-39 have no pull-up
    attachInterruptArg(digitalPinToInterrupt(ENC_C1[i]), encISR, (void*)(intptr_t)(i + 1), CHANGE);
  }

  ledcAttach(SERVO_PIN, 50, 14);                        // servo: back to its last angle (see the servo notes)
  servoSaved = constrain(prefs.getInt("servo", ARM_STOW), 0, 180);
  servoNow = servoTarget = servoSaved;
  servoWriteAngle(servoNow);
  clawCloseSign = prefs.getInt("clawdir", 1) < 0 ? -1 : 1;
  gripSqueeze = constrain(prefs.getInt("grip", 4), 1, 30);
  clawOpenDeg = prefs.getInt("clawopen", -1);
  clawTouchDeg = prefs.getInt("clawtouch", -1);
  if (clawOpenDeg > 180) clawOpenDeg = -1;
  if (clawTouchDeg > 180) clawTouchDeg = -1;

  analogSetPinAttenuation(BATT_PIN, ADC_11db);          // battery

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                                 // power saving adds 100+ ms lag
  WiFi.setAutoReconnect(true);
  connectWiFi();                                        // keeps trying until it's on the network
  Serial.printf("\nReady.  Web: http://%s  (or http://%s.local)   UDP port %d   OTA updates on\n",
                WiFi.localIP().toString().c_str(), HOSTNAME, UDP_PORT);
  setupOTA();                                           // starts mDNS (HOSTNAME.local) too
  MDNS.addService("http", "tcp", 80);
  udp.begin(UDP_PORT);
  setupWeb();
  lastCmdMs = millis();
}

void loop() {
  lastLoopMs = millis();                                // heartbeat for the motor watchdog
  static unsigned long lastUs = micros();               // timing check: the slowest pass
  unsigned long nowUs = micros();
  if (nowUs - lastUs > loopMaxUs) loopMaxUs = nowUs - lastUs;
  lastUs = nowUs;
  if (wdStopped) {                                      // loop() had stalled and the watchdog cut the
    wdStopped = false;                                  // motors: don't jump back to an old command,
    stopNow();                                          // wait for a fresh one
    lastCmdMs = millis();
  }
  // Updates over Wi-Fi only while the robot is stopped: checking the OTA password takes ~1 s
  // during which nothing else runs, so one must never start mid-drive.
  if (cmdL == 0 && cmdR == 0 && rampL == 0 && rampR == 0) ArduinoOTA.handle();
  if (otaActive) return;                                // (motors are off until it restarts)
  keepWiFi();                                           // reconnect if the network was lost
  handleUdp();
  server.handleClient();

  static unsigned long tCtl = millis();                 // motor control at 50 Hz
  if (millis() - tCtl >= 20) {
    float dt = fminf((millis() - tCtl) / 1000.0f, 0.05f);   // (after a stall: no jump in the ramp)
    tCtl = millis();
    controlStep(dt);
  }

  static unsigned long tBat = millis();                        // battery every 0.5 s
  if (millis() - tBat >= 500) { updateBattery((millis() - tBat) / 1000.0f); tBat = millis(); }

  updateServo();
  sendTelemetry();
}
