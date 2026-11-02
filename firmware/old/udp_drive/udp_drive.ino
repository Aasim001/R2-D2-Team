// ESP32 UDP drive receiver for the laptop line follower
// Packet: 6 bytes little-endian = int16 left, int16 right (-255..255), uint16 seq
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

const char* WIFI_SSID = "YourWiFi";       // 2.4 GHz, same network as the MacBook
const char* WIFI_PASS = "YourPassword";
const char* HOSTNAME  = "car1";           // laptop finds it as car1.local
const uint16_t UDP_PORT = 4210;
const unsigned long FAILSAFE_MS = 300;    // stop if commands stop

int motors[4][3] = {                      // {PWM, IN1, IN2}
  {25, 26, 27},   // front-left
  {14, 32, 33},   // front-right
  {13, 18, 19},   // rear-left
  {23, 21, 22},   // rear-right
};

WiFiUDP udp;
unsigned long lastPkt = 0;
uint16_t lastSeq = 0;
bool stopped = true;

void setMotor(int i, int s) {
  s = constrain(s, -255, 255);
  digitalWrite(motors[i][1], s > 0 ? HIGH : LOW);
  digitalWrite(motors[i][2], s < 0 ? HIGH : LOW);
  ledcWrite(motors[i][0], abs(s));
}

void setSides(int left, int right) {
  setMotor(0, left);  setMotor(2, left);
  setMotor(1, right); setMotor(3, right);
  stopped = (left == 0 && right == 0);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(motors[i][1], OUTPUT);
    pinMode(motors[i][2], OUTPUT);
    ledcAttach(motors[i][0], 1000, 8);
  }
  setSides(0, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                   // important: Wi-Fi power saving adds 100+ ms lag
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.printf("\nIP %s  port %d\n", WiFi.localIP().toString().c_str(), UDP_PORT);
  MDNS.begin(HOSTNAME);
  udp.begin(UDP_PORT);
}

void loop() {
  uint8_t buf[8];
  while (udp.parsePacket() > 0) {         // read every waiting packet, keep the newest
    int len = udp.read(buf, sizeof(buf));
    if (len != 6) continue;
    int16_t l, r; uint16_t seq;
    memcpy(&l, buf, 2); memcpy(&r, buf + 2, 2); memcpy(&seq, buf + 4, 2);
    bool newer = (int16_t)(seq - lastSeq) > 0 || millis() - lastPkt > FAILSAFE_MS;
    if (!newer) continue;                 // late / out-of-order packet
    lastSeq = seq;
    lastPkt = millis();
    setSides(l, r);
  }
  if (!stopped && millis() - lastPkt > FAILSAFE_MS) setSides(0, 0);
}
