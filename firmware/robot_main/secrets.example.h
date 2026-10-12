// secrets.example.h - TEMPLATE. Copy it to secrets.h (same folder) and fill in your own values.
// secrets.h is in .gitignore: it never goes to GitHub.
// =====================================================================
//  secrets.h - your Wi-Fi and over-the-air (OTA) update password.
//  Edit THIS file (not robot_main.ino) and keep it to yourself: don't paste it into chats
//  and don't share it. robot_main.ino reads these three values from here.
// =====================================================================
#pragma once

#define SECRET_WIFI_SSID     "your-2.4GHz-wifi-name"     // must offer 2.4 GHz - the ESP32 has no 5 GHz
#define SECRET_WIFI_PASS     "your-wifi-password"
#define SECRET_OTA_PASSWORD  "pick-an-ota-password"             // needed to update the robot over Wi-Fi. Pick your own:
                                               // on a shared network (competition venue) anyone who
                                               // knows it could re-flash the robot.
