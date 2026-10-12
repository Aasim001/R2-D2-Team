// =====================================================================
//  Pickup robot - ESP32 + 2x TB6612FNG + 4x N20 encoder motors
//  Wi-Fi drive (4 directions), encoders, MG90S pickup arm, battery level
//  Wiring = docs/wiring.md (STBY on both drivers goes to 3V3)
//  Board: ESP32 Dev Module, Arduino-ESP32 core 3.x
// =====================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ---------- Wi-Fi (2.4 GHz only) ----------
const char* WIFI_SSID = "YourWiFi";
const char* WIFI_PASS = "YourPassword";
const char* HOSTNAME  = "car1";            // http://car1.local on most laptops

// ---------- motors {PWM, IN1, IN2} ----------
int motors[4][3] = {
  {25, 26, 27},   // 0 front-left   (front driver A)
  {14, 32, 33},   // 1 front-right  (front driver B)
  {13, 18, 19},   // 2 rear-left    (rear driver A)
  {23, 21, 22},   // 3 rear-right   (rear driver B)
};

// ---------- encoders {C1, C2} ----------
int encPins[4][2] = { {4, 35}, {16, 39}, {17, 36}, {5, 15} };  // FL, FR, RL, RR
int encSign[4]    = { 1, 1, 1, 1 };       // set to -1 if a wheel counts backwards
volatile long encCount[4] = {0, 0, 0, 0};
long  encLast[4] = {0, 0, 0, 0};
float encRate[4] = {0, 0, 0, 0};          // counts per second

// ---------- pickup servo + battery ----------
const int SERVO_PIN = 2;
const int BATT_PIN  = 34;                 // 100k from battery +, 47k to GND
const float BATT_RATIO = (100.0 + 47.0) / 47.0;

// Arm positions in SERVO degrees - find yours with the slider on the page
int ARM_STOW = 20, ARM_CARRY = 60, ARM_RELEASE = 72, ARM_REACH = 110;

// ---------- driving ----------
const int PWM_FREQ = 1000;
const unsigned long DRIVE_TIMEOUT = 1000; // stop if no command for 1 s
unsigned long lastCmd = 0;
bool moving = false;

WebServer server(80);

// ===================== motors =====================
void setMotor(int i, int s) {
  s = constrain(s, -255, 255);
  digitalWrite(motors[i][1], s > 0 ? HIGH : LOW);
  digitalWrite(motors[i][2], s < 0 ? HIGH : LOW);
  ledcWrite(motors[i][0], abs(s));
}

void setSides(int left, int right) {
  setMotor(0, left);  setMotor(2, left);
  setMotor(1, right); setMotor(3, right);
  moving = (left != 0 || right != 0);
}

void drive(char c, int s) {
  switch (c) {
    case 'f': setSides( s,  s); break;    // forward
    case 'b': setSides(-s, -s); break;    // backward
    case 'l': setSides(-s,  s); break;    // turn left on the spot
    case 'r': setSides( s, -s); break;    // turn right on the spot
    default:  setSides( 0,  0); break;    // stop
  }
}

// ===================== encoders =====================
void IRAM_ATTR encISR(void* arg) {
  int i = (int)(intptr_t)arg;
  bool a = digitalRead(encPins[i][0]);
  bool b = digitalRead(encPins[i][1]);
  encCount[i] += (a == b) ? encSign[i] : -encSign[i];
}

void updateRates() {                       // every 200 ms
  static unsigned long t = 0;
  if (millis() - t < 200) return;
  float dt = (millis() - t) / 1000.0;
  t = millis();
  for (int i = 0; i < 4; i++) {
    noInterrupts(); long c = encCount[i]; interrupts();
    encRate[i] = (c - encLast[i]) / dt;
    encLast[i] = c;
  }
}

// ===================== servo (50 Hz, smooth moves) =====================
int servoNow = 0, servoTarget = 0;

void servoWriteAngle(int deg) {
  int us = map(constrain(deg, 0, 180), 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, (uint32_t)us * 16383 / 20000);   // 14-bit duty at 50 Hz
}

void updateServo() {                       // moves 1 degree every 8 ms
  static unsigned long t = 0;
  if (servoNow == servoTarget || millis() - t < 8) return;
  t = millis();
  servoNow += (servoTarget > servoNow) ? 1 : -1;
  servoWriteAngle(servoNow);
}

// ===================== battery =====================
float batteryVolts() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(BATT_PIN);
  return (mv / 8.0) / 1000.0 * BATT_RATIO;
}

// ===================== web page =====================
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
.arm{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;max-width:340px;margin:10px auto}
.arm button{height:48px;font-size:16px;background:#5b4bb7}
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
<div class="arm">
 <button onclick="arm('stow')">Stow</button><button onclick="arm('reach')">Reach</button>
 <button onclick="arm('carry')">Grab / carry</button><button onclick="arm('release')">Release</button>
</div>
<div>Servo angle <span id="sa">-</span>&deg; (for calibration)</div>
<input type="range" min="0" max="180" value="20" id="sv" oninput="sa.textContent=this.value;fetch('/servo?a='+this.value)">
<div id="st">connecting...</div>
<script>
let cur='s', spd=180;
const v=document.getElementById('v'), sa=document.getElementById('sa'), st=document.getElementById('st');
function send(){fetch(`/cmd?c=${cur}&s=${spd}`).catch(()=>{})}
function go(c){if(cur!==c){cur=c;send()}}
function arm(p){fetch('/arm?p='+p)}
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
setInterval(()=>{fetch('/status').then(r=>r.json()).then(d=>{
 st.textContent=`Battery  ${d.batt.toFixed(2)} V\nServo    ${d.servo} deg\n`+
 `FL ${d.cnt[0]} (${d.rate[0]}/s)   FR ${d.cnt[1]} (${d.rate[1]}/s)\n`+
 `RL ${d.cnt[2]} (${d.rate[2]}/s)   RR ${d.cnt[3]} (${d.rate[3]}/s)\nWi-Fi    ${d.rssi} dBm`;
 sa.textContent=d.servo;}).catch(()=>{st.textContent='no connection'})},500);
</script></body></html>
)HTML";

// ===================== web handlers =====================
void handleRoot() { server.send(200, "text/html", PAGE); }

void handleCmd() {                         // /cmd?c=f&s=180
  char c = server.hasArg("c") ? server.arg("c")[0] : 's';
  int  s = server.hasArg("s") ? server.arg("s").toInt() : 180;
  drive(c, s);
  lastCmd = millis();
  server.send(200, "text/plain", "ok");
}

void handleStop() { drive('s', 0); lastCmd = millis(); server.send(200, "text/plain", "stopped"); }

void handleServo() {                       // /servo?a=90
  if (server.hasArg("a")) servoTarget = constrain(server.arg("a").toInt(), 0, 180);
  server.send(200, "text/plain", "ok");
}

void handleArm() {                         // /arm?p=stow|reach|carry|release
  String p = server.arg("p");
  if      (p == "stow")    servoTarget = ARM_STOW;
  else if (p == "reach")   servoTarget = ARM_REACH;
  else if (p == "carry")   servoTarget = ARM_CARRY;
  else if (p == "release") servoTarget = ARM_RELEASE;
  server.send(200, "text/plain", p);
}

void handleStatus() {
  String j = "{\"batt\":" + String(batteryVolts(), 2) +
             ",\"servo\":" + String(servoNow) +
             ",\"rssi\":" + String(WiFi.RSSI()) + ",\"cnt\":[";
  for (int i = 0; i < 4; i++) { noInterrupts(); long c = encCount[i]; interrupts(); j += String(c) + (i < 3 ? "," : "],\"rate\":["); }
  for (int i = 0; i < 4; i++) j += String((int)encRate[i]) + (i < 3 ? "," : "]}");
  server.send(200, "application/json", j);
}

// ===================== setup / loop =====================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {            // motors (STBY is hard-wired to 3V3)
    pinMode(motors[i][1], OUTPUT);
    pinMode(motors[i][2], OUTPUT);
    ledcAttach(motors[i][0], PWM_FREQ, 8);
  }
  drive('s', 0);

  for (int i = 0; i < 4; i++) {            // encoders
    int c2 = encPins[i][1];
    pinMode(encPins[i][0], INPUT_PULLUP);
    pinMode(c2, (c2 >= 34) ? INPUT : INPUT_PULLUP);    // 34-39 have no pull-up
    attachInterruptArg(digitalPinToInterrupt(encPins[i][0]), encISR, (void*)(intptr_t)i, CHANGE);
  }

  ledcAttach(SERVO_PIN, 50, 14);           // servo
  servoNow = servoTarget = ARM_STOW;
  servoWriteAngle(ARM_STOW);

  analogSetPinAttenuation(BATT_PIN, ADC_11db);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.print("\nOpen http://");
  Serial.println(WiFi.localIP());
  MDNS.begin(HOSTNAME);

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/stop", handleStop);
  server.on("/servo", handleServo);
  server.on("/arm", handleArm);
  server.on("/status", handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();
  updateRates();
  updateServo();
  if (moving && millis() - lastCmd > DRIVE_TIMEOUT) drive('s', 0);   // safety stop
}
