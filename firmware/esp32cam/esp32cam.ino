// =====================================================================
//  esp32cam.ino  -  the robot's QR camera (AI-Thinker ESP32-CAM, OV2640)
//
//   http://<cam>/                     status page
//   http://<cam>/capture              one JPEG, at the size asked for:
//          ?size=qvga|vga|svga|xga|sxga|uxga   (320x240 .. 1600x1200; default uxga - QR codes need pixels)
//          ?flash=1                   the on-board LED on while the picture is taken (lights the alcove)
//   http://<cam>:81/stream            MJPEG live view (VGA)
//   OTA updates over Wi-Fi (python firmware/upload.py --cam --ota <ip> --robot N), password from secrets.h
//
//  The QR codes on the cubes are small: at 320x240 a 4-5 cm code ~10 cm away is only ~3 px per QR square -
//  too few to decode (2026-10-10, both robots). At 1600x1200 it is ~15 px per square.
//  Board: "AI Thinker ESP32-CAM" (esp32:esp32:esp32cam, PSRAM on). Wi-Fi: robot_main/secrets.h.
// =====================================================================
#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "secrets.h"

#ifndef ROBOT_NUM
#define ROBOT_NUM 1
#endif
#define STR_(x) #x
#define STR(x) STR_(x)
const char* HOSTNAME = "cam" STR(ROBOT_NUM);      // cam1 / cam2 / cam3 = the camera on robot car1 / car2 / car3

// AI-Thinker ESP32-CAM pins
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_LED 4

httpd_handle_t mainServer = NULL, streamServer = NULL;
SemaphoreHandle_t camLock;                         // one picture at a time (stream and capture share the sensor)
framesize_t streamSize = FRAMESIZE_VGA;
bool camOk = false;                                // the camera started (else it is retried in loop())

// ---------------------------------------------------------------- camera
bool startCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM; c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_UXGA;                   // allocate for the biggest picture
  c.jpeg_quality = 10;                             // (lower = better; 10 keeps QR edges crisp)
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;                // always the newest frame
  if (esp_camera_init(&c) != ESP_OK) return false;
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, streamSize);
  return true;
}

framesize_t sizeFromName(const char* n) {
  if (!strcmp(n, "qvga")) return FRAMESIZE_QVGA;
  if (!strcmp(n, "vga"))  return FRAMESIZE_VGA;
  if (!strcmp(n, "svga")) return FRAMESIZE_SVGA;
  if (!strcmp(n, "xga"))  return FRAMESIZE_XGA;
  if (!strcmp(n, "sxga")) return FRAMESIZE_SXGA;
  return FRAMESIZE_UXGA;
}

// ---------------------------------------------------------------- http
esp_err_t indexHandler(httpd_req_t* req) {
  char page[600];
  snprintf(page, sizeof(page),
           "<html><body style='font-family:monospace;background:#111;color:#eee;padding:20px'>"
           "<h2 style='color:#4c4'>%s %s</h2><p>IP: %s</p>"
           "<p><a style='color:#8cf' href='/capture?size=vga'>Single frame (VGA)</a> | "
           "<a style='color:#8cf' href='/capture'>Single frame (UXGA, for QR)</a> | "
           "<a style='color:#8cf' href='http://%s:81/stream'>Live stream</a></p></body></html>",
           HOSTNAME, camOk ? "ready" : "- CAMERA NOT STARTED", WiFi.localIP().toString().c_str(),
           WiFi.localIP().toString().c_str());
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

esp_err_t captureHandler(httpd_req_t* req) {
  char query[64] = "", val[16] = "uxga";
  bool flash = false;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    httpd_query_key_value(query, "size", val, sizeof(val));
    char f[4] = "";
    if (httpd_query_key_value(query, "flash", f, sizeof(f)) == ESP_OK) flash = f[0] == '1';
  }
  if (!camOk) { httpd_resp_send_500(req); return ESP_FAIL; }
  xSemaphoreTake(camLock, portMAX_DELAY);
  sensor_t* s = esp_camera_sensor_get();
  framesize_t want = sizeFromName(val);
  if (flash) { digitalWrite(FLASH_LED, HIGH); delay(150); }
  s->set_framesize(s, want);
  for (int i = 0; i < 2; i++) {                    // frames already queued are from the old size / no flash
    camera_fb_t* old = esp_camera_fb_get();
    if (old) esp_camera_fb_return(old);
  }
  camera_fb_t* fb = esp_camera_fb_get();
  if (flash) digitalWrite(FLASH_LED, LOW);
  esp_err_t res;
  if (!fb) {
    httpd_resp_send_500(req);
    res = ESP_FAIL;
  } else {
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }
  s->set_framesize(s, streamSize);
  xSemaphoreGive(camLock);
  return res;
}

#define BOUNDARY "frame0123456789"
esp_err_t streamHandler(httpd_req_t* req) {
  if (!camOk) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
  char head[96];
  while (true) {
    xSemaphoreTake(camLock, portMAX_DELAY);
    camera_fb_t* fb = esp_camera_fb_get();
    xSemaphoreGive(camLock);
    if (!fb) return ESP_FAIL;
    int n = snprintf(head, sizeof(head), "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     (unsigned)fb->len);
    esp_err_t r = httpd_resp_send_chunk(req, head, n);
    if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (r != ESP_OK) return r;                     // (the viewer went away)
  }
}

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  if (httpd_start(&mainServer, &cfg) == ESP_OK) {
    httpd_uri_t index = {"/", HTTP_GET, indexHandler, NULL};
    httpd_uri_t cap = {"/capture", HTTP_GET, captureHandler, NULL};
    httpd_register_uri_handler(mainServer, &index);
    httpd_register_uri_handler(mainServer, &cap);
  }
  cfg.server_port = 81;
  cfg.ctrl_port += 1;
  if (httpd_start(&streamServer, &cfg) == ESP_OK) {
    httpd_uri_t st = {"/stream", HTTP_GET, streamHandler, NULL};
    httpd_register_uri_handler(streamServer, &st);
  }
}

// ---------------------------------------------------------------- setup / loop
// Wi-Fi + OTA come up FIRST and stay up whatever the camera does: this camera is usually updated over the air
// only, so a camera fault must never lock it out (the camera is simply retried every 10 s).

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);
  camLock = xSemaphoreCreateMutex();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(250); Serial.print("."); }
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(SECRET_OTA_PASSWORD);
  ArduinoOTA.setTimeout(10000);
  ArduinoOTA.begin();
  camOk = startCamera();
  if (!camOk) Serial.println("Camera init FAILED - check the ribbon cable (retrying every 10 s; OTA stays on)");
  startServers();
  Serial.printf("\nReady.  %s  http://%s/capture  stream http://%s:81/stream\n", HOSTNAME,
                WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
}

void loop() {
  ArduinoOTA.handle();
  static unsigned long lost = 0, camTry = 0;       // Wi-Fi lost for 10 s: try again
  if (WiFi.status() != WL_CONNECTED) {
    if (!lost) lost = millis();
    else if (millis() - lost > 10000) { WiFi.disconnect(); WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASS); lost = millis(); }
  } else lost = 0;
  if (!camOk && millis() - camTry > 10000) {       // camera failed at start: keep trying
    camTry = millis();
    esp_camera_deinit();
    camOk = startCamera();
  }
  delay(5);
}
