Full flash backups (the whole 4 MB chip: bootloader, partition table, firmware, saved settings).

bot1_full_flash_2026-10-30_mac-00-70-07-a3-18-a8.bin
  Read from the robot with MAC 00:70:07:a3:18:a8 (CH340 USB chip) on 2026-10-30, verified against the chip.
  MD5 7bff11096ffd7518fde7c18ab81e3053
  It holds robot_main.ino as of 2026-10-30: Wi-Fi "A34", servo limp at power-on, claw held to 30-60 deg
  (grip 58, open 30), nudge / servo off commands. Robot name "car1".

!! It contains the Wi-Fi password and the OTA password in plain text - don't share it or put it on GitHub.

Write it onto another ESP32 (same 4 MB "ESP32 Dev Module"), over USB:
  %LOCALAPPDATA%\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe --port COMx --baud 921600 write-flash 0 bot1_full_flash_2026-10-30_mac-00-70-07-a3-18-a8.bin
The copy is an exact clone: same Wi-Fi, same OTA password, same name "car1" (each robot keeps its own MAC
address, so they get different IP addresses).
Normally just compile from the source instead:  python firmware/upload.py --usb COMx
