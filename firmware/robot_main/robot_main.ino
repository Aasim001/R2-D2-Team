// =====================================================================
//  robot_main.ino  -  ONE firmware for the pickup robot
//
//   * 4 N20 motors on 2x TB6612FNG  (STBY on BOTH drivers wired to 3V3)
//   * 4 hall encoders (C1 + C2 on every motor)
//   * Straight-drive assist: when driving straight, all 4 wheels are held
//     at the same speed and left/right travel is kept equal, so the robot
//     goes straight. In turns, the two wheels on each side are matched.
//   * MG90S pickup-claw servo (smooth moves, 4 presets, every angle held to a safe range)
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
// =====================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "esp_timer.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"
#include "secrets.h"

// ============================ SETTINGS ===============================
const char* WIFI_SSID    = SECRET_WIFI_SSID;     // 2.4 GHz network (set in secrets.h)
const char* WIFI_PASS    = SECRET_WIFI_PASS;
const char* OTA_PASSWORD = SECRET_OTA_PASSWORD;
const char* HOSTNAME  = "car1";            // car1.local  (car2, car3 for the other robots)
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

// ---- fail-safes
const unsigned long UDP_FAILSAFE_MS  = 300;   // laptop stopped sending -> stop
const unsigned long HTTP_FAILSAFE_MS = 1000;  // web page stopped sending -> stop

// ---- servo (pickup claw, MG90S on D2). At power-on the servo gets NO signal (limp, doesn't move). The first
// servo / arm command starts it - it then jumps to that angle from wherever it is ("nudge" first if unsure).
// Measured on the robot's claw 2026-10-30: a HIGHER angle CLOSES it. 58 grips the cube, 30 = open (lets go).
// Every servo command is held to ARM_MIN..ARM_MAX: past 60 the claw would squeeze / jam. 20 = open wide (the old
// firmware held the claw there for days; allowed 2026-11-02 for the pick approach); below 20 hasn't been tried.
const int SERVO_PIN = 2;
const int ARM_MIN = 20, ARM_MAX = 60;
int ARM_STOW = 30, ARM_CARRY = 58, ARM_RELEASE = 30, ARM_REACH = 30;   // reach = open, ready to pick

// ---- battery: VBAT -100k- D34 -47k- GND
const int   BATT_PIN   = 34;
const float BATT_RATIO = (100.0 + 47.0) / 47.0;
const float LOW_BATT_V = 6.4;     // 2S Li-ion: stop motors below this (3.2 V/cell)

// ============================ STATE ==================================
WebServer server(80);
WiFiUDP   udp;

volatile long encCount[4] = {0, 0, 0, 0};
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
int   testWheel = -1, testPwm = 0;    // single-wheel test ("wheel fl 150 2000"): which motor, its PWM ...
unsigned long testUntil = 0;          // ... and until when (then everything stops)

unsigned long lastCmdMs = 0, failsafeMs = HTTP_FAILSAFE_MS;
uint16_t lastSeq = 0;
unsigned long lastUdpDriveMs = 0;

float battV = 0;  bool battPresent = false;  bool lowBatt = false;  float lowT = 0;
int   servoNow = 0, servoTarget = 0;
bool  servoLive = false;              // false = no servo pulses sent yet since power-on (limp)

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
  int i = (int)(intptr_t)arg;
  int step = (fastRead(ENC_C1[i]) == fastRead(ENC_C2[i])) ? ENC_SIGN[i] : -ENC_SIGN[i];
  portENTER_CRITICAL_ISR(&encMux);
  encCount[i] += step;
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
  testWheel = -1;
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

  // 1b. single-wheel test: only that motor, the others off, until its time is up (or "stop" / low battery)
  if (testWheel >= 0) {
    if ((long)(millis() - testUntil) < 0 && !lowBatt) {
      for (int i = 0; i < 4; i++) motorRaw(i, i == testWheel ? testPwm : 0);
      return;
    }
    testWheel = -1;
    for (int i = 0; i < 4; i++) motorRaw(i, 0);
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

  // 7. combine; if a wheel would need more than 255, slow the others instead
  float mag[4]; float maxMag = 0;
  for (int i = 0; i < 4; i++) {
    mag[i] = base[i] == 0 ? 0 : fmaxf(0.0f, fabsf(base[i]) + corr[i]);
    maxMag = fmaxf(maxMag, mag[i]);
  }
  float over = maxMag > 255 ? maxMag - 255 : 0;
  for (int i = 0; i < 4; i++) {
    int m = base[i] == 0 ? 0 : (int)constrain(mag[i] - over, 0.0f, 255.0f);
    motorRaw(i, base[i] < 0 ? -m : m);
  }
}

// ============================ SERVO ==================================
void servoWriteAngle(int deg) {
  int us = map(constrain(deg, 0, 180), 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, (uint32_t)us * 16383 / 20000);  // 14-bit at 50 Hz
}

void servoOff() {                                       // no more pulses: the servo goes limp where it is
  ledcWrite(SERVO_PIN, 0);
  servoLive = false;
}

void updateServo() {                                    // 1 degree every 8 ms
  static unsigned long t = 0;
  if (!servoLive || servoNow == servoTarget || millis() - t < 8) return;
  t = millis();
  servoNow += (servoTarget > servoNow) ? 1 : -1;
  servoWriteAngle(servoNow);
}

// every servo / arm command goes through here
void setServoTarget(int deg) {
  servoTarget = constrain(deg, ARM_MIN, ARM_MAX);
  if (!servoLive) {                     // first command since power-on: where the servo is now is unknown,
    servoLive = true;                   // so start the pulses right at the asked angle (stepping from a
    servoNow = servoTarget;             // guess would first jerk it to the guess)
    servoWriteAngle(servoNow);
  }
}

// A few pulses toward deg, then limp again. This (analog) servo only drives while it gets pulses, so it moves
// just a little toward deg, even when where it is now is unknown: a safe first move - it can't slam into a stop.
// Meanwhile D2 is read back, to check the pulses really come out of the pin. Holds up loop() for up to ~0.25 s
// (so only with the wheels stopped). Returns what it saw.
String servoNudge(int deg, int pulses) {
  pulses = constrain(pulses, 1, 10);
  servoNow = servoTarget = constrain(deg, ARM_MIN, ARM_MAX);
  servoLive = false;                                    // (limp afterwards: no stepping)
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[SERVO_PIN]);        // so the pin can be read while the PWM drives it
  int expectUs = map(servoNow, 0, 180, 500, 2500);
  unsigned long t0 = micros(), offAt = pulses * 20000UL, rise = 0, highUs = 0;   // 50 Hz = a pulse every 20 ms
  int seen = 0;
  bool on = true, prev = false;
  servoWriteAngle(servoNow);                            // (starts with the next 20 ms period)
  while (micros() - t0 < offAt + 25000UL) {             // the burst + one more period to see it end
    if (on && micros() - t0 >= offAt) { ledcWrite(SERVO_PIN, 0); on = false; }
    bool lvl = fastRead(SERVO_PIN);
    if (lvl && !prev) { seen++; rise = micros(); }
    if (!lvl && prev) highUs += micros() - rise;
    prev = lvl;
  }
  if (on) ledcWrite(SERVO_PIN, 0);
  String r = "nudge " + String(servoNow) + " x" + String(pulses) + " - D2 read back: " + String(seen) + " pulse(s)";
  if (seen) r += " of " + String(highUs / seen) + " us (should be " + String(expectUs) + ")";
  if (prev) r += " - D2 STUCK HIGH";
  else if (!seen) r += " - NO PULSES on D2 (pin shorted to GND? PWM not running?)";
  return r;
}

bool armPreset(const String& p) {
  if      (p == "stow")    setServoTarget(ARM_STOW);
  else if (p == "reach")   setServoTarget(ARM_REACH);
  else if (p == "carry")   setServoTarget(ARM_CARRY);
  else if (p == "release") setServoTarget(ARM_RELEASE);
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
  String j = "{\"batt\":" + String(battV, 2) + ",\"low\":" + String(lowBatt) +
             ",\"servo\":" + String(servoNow) + ",\"servo_on\":" + String(servoLive) +
             ",\"assist\":" + String(ASSIST) +
             ",\"cmd\":[" + String(cmdL) + "," + String(cmdR) + "]" +
             ",\"rssi\":" + String(WiFi.RSSI());
  j += ",\"pwm\":[";  for (int i = 0; i < 4; i++) j += String(pwmOut[i]) + (i < 3 ? "," : "]");
  j += ",\"cnt\":[";  for (int i = 0; i < 4; i++) j += String(readCount(i)) + (i < 3 ? "," : "]");
  j += ",\"rate\":["; for (int i = 0; i < 4; i++) j += String((int)rateSigned[i]) + (i < 3 ? "," : "]");
  j += ",\"ok\":[";   for (int i = 0; i < 4; i++) j += String(encOk[i]) + (i < 3 ? "," : "]");
  j += ",\"trim\":["; for (int i = 0; i < 4; i++) j += String((int)trim[0][i]) + (i < 3 ? "," : "]");
  j += "}";
  return j;
}

// ============================ UDP ====================================
void udpReply(const String& s) {
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(s);
  udp.endPacket();
}

// a whole number, nothing else (String.toInt() gives 0 for "off" or "abc" - and servo 0 deg is a full swing)
bool parseNumber(String t, int& out) {
  t.trim();
  if (t.length() == 0) return false;
  for (unsigned i = 0; i < t.length(); i++) if (!isDigit(t[i])) return false;
  out = t.toInt();
  return true;
}

// "fl 150 2000" -> only the front-left motor at PWM 150 for 2 s (at most 3 s), then everything stops
bool startWheelTest(String a) {
  a.trim();
  int sp1 = a.indexOf(' ');
  if (sp1 < 0) return false;
  String name = a.substring(0, sp1), rest = a.substring(sp1 + 1);
  rest.trim();
  int idx = -1;
  for (int i = 0; i < 4; i++) { String n = WHEEL_NAME[i]; n.toLowerCase(); if (name == n) idx = i; }
  if (idx < 0) return false;
  int sp2 = rest.indexOf(' '), pwm, ms = 1000;
  String p = sp2 < 0 ? rest : rest.substring(0, sp2);
  bool neg = p.startsWith("-");
  if (!parseNumber(neg ? p.substring(1) : p, pwm)) return false;
  if (sp2 >= 0 && !parseNumber(rest.substring(sp2 + 1), ms)) return false;
  stopNow();
  testWheel = idx;
  testPwm = constrain(neg ? -pwm : pwm, -255, 255);
  testUntil = millis() + constrain(ms, 50, 3000);
  lastCmdMs = millis();
  return true;
}

// text commands:  stop | arm stow|reach|carry|release | servo <deg> | servo off | nudge <deg> [pulses]
//                 wheel fl|fr|rl|rr <pwm> [ms] | assist on|off | tele on|off | trim reset | status
void handleText(String s) {
  s.trim(); s.toLowerCase();
  if (s == "stop") { stopNow(); lastCmdMs = millis(); udpReply("ok stop"); }
  else if (s.startsWith("arm "))   udpReply(armPreset(s.substring(4)) ? "ok " + s : "err unknown arm position");
  else if (s == "servo off") { servoOff(); udpReply("ok servo off (limp)"); }
  else if (s.startsWith("wheel "))
    udpReply(startWheelTest(s.substring(6)) ? "ok " + s : "err use: wheel fl|fr|rl|rr <pwm -255..255> [ms <= 3000]");
  else if (s.startsWith("nudge ")) {                    // nudge <deg> [pulses 1-10, default 1]
    String a = s.substring(6); a.trim();
    int sp = a.indexOf(' '), deg, n = 1;
    if (!(parseNumber(sp < 0 ? a : a.substring(0, sp), deg) && (sp < 0 || parseNumber(a.substring(sp + 1), n))))
      udpReply("err use: nudge <deg> [pulses]");
    else if (cmdL || cmdR || rampL || rampR) udpReply("err stop the wheels first");
    else udpReply("ok " + servoNudge(deg, n));
  }
  else if (s.startsWith("servo ")) {
    int deg;
    if (parseNumber(s.substring(6), deg)) {
      setServoTarget(deg);
      udpReply("ok servo " + String(servoTarget) +
               (servoTarget != deg ? " (held to the safe range " + String(ARM_MIN) + "-" + String(ARM_MAX) + ")" : String("")));
    }
    else udpReply("err use: servo <deg " + String(ARM_MIN) + "-" + String(ARM_MAX) + "> | servo off");
  }
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
 <button onclick="arm('stow')">Stow</button><button onclick="arm('reach')">Open (reach)</button>
 <button onclick="arm('carry')">Grab / carry</button><button onclick="arm('release')">Release</button>
</div>
<div>Servo angle <span id="sa">-</span>&deg; (calibration)</div>
<input type="range" min="20" max="60" value="30" id="sv" oninput="sa.textContent=this.value;fetch('/servo?a='+this.value)">
<div id="st">connecting...</div>
<script>
let cur='s', spd=180, assist=1;
const v=document.getElementById('v'), sa=document.getElementById('sa'), st=document.getElementById('st'), as=document.getElementById('as');
function send(){fetch(`/cmd?c=${cur}&s=${spd}`).catch(()=>{})}
function go(c){if(cur!==c){cur=c;send()}}
function arm(p){fetch('/arm?p='+p)}
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
document.addEventListener('visibilitychange',()=>{if(document.hidden)go('s')});
const N=['FL','FR','RL','RR'];
setInterval(()=>{fetch('/status').then(r=>r.json()).then(d=>{
 assist=d.assist; as.textContent='Straight assist: '+(assist?'ON':'OFF'); as.style.background=assist?'#1e8449':'#555';
 let t=`Battery ${d.batt.toFixed(2)} V ${d.low?' LOW - MOTORS OFF':''}\nServo   ${d.servo_on?d.servo+' deg':'off (limp)'}   Wi-Fi ${d.rssi} dBm\n\nwheel  count   rate/s  pwm  trim enc\n`;
 for(let i=0;i<4;i++) t+=`${N[i]}  ${String(d.cnt[i]).padStart(7)} ${String(d.rate[i]).padStart(7)} ${String(d.pwm[i]).padStart(5)} ${String(d.trim[i]).padStart(4)}  ${d.ok[i]?'ok':'NO'}\n`;
 st.textContent=t; sa.textContent=d.servo;}).catch(()=>{st.textContent='no connection'})},400);
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
  server.on("/servo",  []() { if (server.hasArg("a")) setServoTarget(server.arg("a").toInt()); server.send(200, "text/plain", "ok"); });
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
    attachInterruptArg(digitalPinToInterrupt(ENC_C1[i]), encISR, (void*)(intptr_t)i, CHANGE);
  }

  ledcAttach(SERVO_PIN, 50, 14);                        // servo: NO pulses at power-on - it stays limp
  ledcWrite(SERVO_PIN, 0);                              // where it is (the arm may be open / half-built)
  servoNow = servoTarget = ARM_STOW;                    // and only moves on a servo / arm command

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
  if (wdStopped) {                                      // loop() had stalled and the watchdog cut the
    wdStopped = false;                                  // motors: don't jump back to an old command,
    stopNow();                                          // wait for a fresh one
    lastCmdMs = millis();
  }
  // Updates over Wi-Fi only while the robot is stopped: checking the OTA password takes ~1 s
  // during which nothing else runs, so one must never start mid-drive.
  if (cmdL == 0 && cmdR == 0 && rampL == 0 && rampR == 0 && testWheel < 0) ArduinoOTA.handle();
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
