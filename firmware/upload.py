#!/usr/bin/env python3
"""
Compile robot_main and upload it to the ESP32 - over USB or over Wi-Fi (OTA).

  python firmware/upload.py --list                       # which COM port is the ESP32 on? (USB)
  python firmware/upload.py --usb COM5 --robot 2         # upload over USB (needed ONCE, to get OTA onto the robot)
  python firmware/upload.py --ota car2.local --robot 2   # upload over Wi-Fi (or the robot's IP, e.g. 192.168.1.50)
  python firmware/upload.py --compile                    # only check that it compiles
  python firmware/upload.py --cam --usb COM7 --robot 1   # the robot's ESP32-CAM (firmware/esp32cam), same options

--robot 1 / 2 / 3 = which robot you are uploading to. It is built into the firmware as its name (car1, car2,
car3 - also its car1.local name for OTA), so every upload must say which robot it is for.

Uses the Arduino IDE's own arduino-cli and the ESP32 core's espota tool - nothing extra to install.

Over Wi-Fi:
  * The robot only accepts an update while it is STOPPED. Quit arena_drive.py / bot.py first;
    this script also sends the robot a 'stop' before it starts.
  * It logs in with the OTA password from robot_main/secrets.h. CHANGED that password? The robot
    still has the old one until this upload is done, so log in with the old one:
        python firmware/upload.py --ota car1.local --ask-current     (asks for it, hidden)
    or set the environment variable OTA_CURRENT_PASSWORD.
  * The first time, Windows asks whether espota.exe may use the network. Tick BOTH Private and
    Public (this PC's Wi-Fi and a venue's Wi-Fi are usually "Public"; the robot connects back to
    the PC to fetch the new firmware). If that very first try fails right after the prompt, run
    it again.
  * The robot restarts with the new firmware when it is done, or keeps the old one if it fails.
"""
import argparse, getpass, glob, os, re, shutil, socket, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_MAIN = os.path.join(HERE, "robot_main")
SKETCH = ROBOT_MAIN
BUILD = os.path.join(HERE, ".build")
FQBN = "esp32:esp32:esp32"
CAM_SKETCH = os.path.join(HERE, "esp32cam")           # --cam: the QR camera (AI-Thinker ESP32-CAM, PSRAM)
CAM_BUILD = os.path.join(HERE, ".build-cam")
CAM_FQBN = "esp32:esp32:esp32cam:PartitionScheme=min_spiffs"   # (1.9 MB app with OTA)
OTA_PORT = 3232


def find_cli():
    cands = [shutil.which("arduino-cli"),
             os.path.expandvars(r"%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"),
             r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
             "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"]
    for c in cands:
        if c and os.path.isfile(c):
            return c
    sys.exit("Can't find arduino-cli (it comes with the Arduino IDE 2). Install the IDE, or put arduino-cli on PATH.")


def find_espota():
    roots = [os.path.expandvars(r"%LOCALAPPDATA%\Arduino15"), os.path.expanduser("~/Library/Arduino15"),
             os.path.expanduser("~/.arduino15")]
    for root in roots:
        cores = sorted(glob.glob(os.path.join(root, "packages", "esp32", "hardware", "esp32", "*")),
                       key=lambda p: [int(n) if n.isdigit() else 0 for n in re.split(r"[.\-]", os.path.basename(p))])
        for core in reversed(cores):                       # newest installed ESP32 core first
            exe, py = os.path.join(core, "tools", "espota.exe"), os.path.join(core, "tools", "espota.py")
            if sys.platform == "win32" and os.path.isfile(exe):
                return [exe]
            if os.path.isfile(py):
                return [sys.executable, py]
    sys.exit("Can't find espota (part of the ESP32 board package). Install 'esp32' in the IDE's Boards Manager.")


def ota_password():
    """SECRET_OTA_PASSWORD from secrets.h - the one the compiler uses (comments ignored, last wins)."""
    try:
        text = open(os.path.join(ROBOT_MAIN, "secrets.h"), encoding="utf-8").read()
    except OSError:
        sys.exit("robot_main/secrets.h is missing - it holds the Wi-Fi and OTA passwords.")
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)                     # /* ... */ comments
    text = "\n".join(_strip_line_comment(line) for line in text.splitlines())   # // comments
    found = re.findall(r'^\s*#define\s+SECRET_OTA_PASSWORD\s+"((?:[^"\\]|\\.)*)"', text, re.M)
    if not found:
        sys.exit("No SECRET_OTA_PASSWORD in robot_main/secrets.h")
    pw = found[-1]
    if "\\" in pw or '"' in pw:
        sys.exit("Please use an OTA password without backslashes or quote marks (secrets.h).")
    return pw


def _strip_line_comment(line):
    """Cut a // comment off a line, but not a // inside "quotes"."""
    in_str, i = False, 0
    while i < len(line):
        c = line[i]
        if c == "\\" and in_str:
            i += 2
            continue
        if c == '"':
            in_str = not in_str
        elif c == "/" and not in_str and line[i:i + 2] == "//":
            return line[:i]
        i += 1
    return line


def run(cmd, secret=None):
    """Run a command, printing it first - with the password (if any) hidden."""
    shown = ["--auth=****" if secret is not None and c == "--auth=" + secret else c for c in cmd if c]
    print(">", " ".join(f'"{c}"' if " " in c else c for c in shown))
    return subprocess.call(cmd)


def tell_robot_to_stop(ip):
    """The robot only takes an update while stopped: send its (harmless) 'stop' text command."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.5)
    try:
        for _ in range(3):
            s.sendto(b"stop", (ip, 4210))
            try:
                if s.recvfrom(64)[0].startswith(b"ok"):
                    return True
            except socket.timeout:
                pass
    except OSError:
        pass
    return False




def compile_sketch(cli, robot):
    """Compile for robot 1, 2 or 3 (robot_main.ino's ROBOT_NUM -> its name car1 / car2 / car3)."""
    ROBOT_FILE = os.path.join(BUILD, "robot.txt")     # which robot the last build in BUILD is for
    os.makedirs(BUILD, exist_ok=True)
    if os.path.exists(ROBOT_FILE):
        os.remove(ROBOT_FILE)                          # (a failed compile must not leave the old robot's mark)
    if run([cli, "compile", "--fqbn", FQBN, "--output-dir", BUILD,
            "--build-property", f"compiler.cpp.extra_flags=-DROBOT_NUM={robot} -I{ROBOT_MAIN.replace(os.sep, '/')}",
            SKETCH]) != 0:
        sys.exit("Compile failed - nothing was uploaded.")
    name = os.path.basename(SKETCH) + ".ino.bin"
    bins = glob.glob(os.path.join(BUILD, name))
    if not bins:
        sys.exit(f"Compiled, but no {name} in {BUILD}")
    with open(ROBOT_FILE, "w") as f:
        f.write(str(robot))
    print(f"Compiled for robot {robot} (car{robot}).")
    return bins[0]


def last_build_robot():
    try:
        return int(open(os.path.join(BUILD, "robot.txt")).read().strip())
    except (OSError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description="Compile robot_main and upload it over USB or Wi-Fi (OTA).")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", action="store_true", help="list serial ports / boards")
    g.add_argument("--usb", metavar="PORT", help="upload over USB, e.g. COM5 (Mac: /dev/cu.usbserial-...)")
    g.add_argument("--ota", metavar="HOST", help="upload over Wi-Fi: car1.local or the robot's IP")
    g.add_argument("--compile", action="store_true", help="only compile")
    ap.add_argument("--robot", type=int, choices=(1, 2, 3),
                    help="which robot you are uploading to: 1, 2 or 3 (its name becomes car1 / car2 / car3)")
    ap.add_argument("--no-compile", action="store_true", help="upload the last build without compiling again")
    ap.add_argument("--cam", action="store_true", help="the robot's ESP32-CAM (firmware/esp32cam) instead of the robot")
    ap.add_argument("--ask-current", action="store_true",
                    help="OTA: ask for the password the robot has NOW (when you just changed it in secrets.h)")
    a = ap.parse_args()
    global SKETCH, BUILD, FQBN
    if a.cam:
        SKETCH, BUILD, FQBN = CAM_SKETCH, CAM_BUILD, CAM_FQBN
    cli = find_cli()

    if a.list:
        sys.exit(run([cli, "board", "list"]))
    if a.robot is None:
        if a.usb or a.ota:                                # a wrong number would give two robots the same name
            sys.exit("Which robot is this? Add --robot 1, 2 or 3 (it becomes the robot's name: car1 / car2 / car3).")
        a.robot = 1                                       # (--compile only: any number checks the code)
    m = re.fullmatch(r"(?:car|cam)(\d)(\.local)?", a.ota or "", re.I)
    if m and int(m.group(1)) != a.robot:
        sys.exit(f"--ota {a.ota} but --robot {a.robot}: that would rename car{m.group(1)} to car{a.robot}. "
                 "Check which robot you mean.")
    if a.no_compile:
        binfile = os.path.join(BUILD, os.path.basename(SKETCH) + ".ino.bin")
        if last_build_robot() != a.robot:
            sys.exit(f"The last build is for robot {last_build_robot()}, not robot {a.robot} - "
                     "upload without --no-compile.")
    else:
        binfile = compile_sketch(cli, a.robot)
    if a.compile:
        print("Compiled OK:", binfile)
        return
    if a.usb:
        print("Uploading over USB. If it hangs at 'Connecting...', hold the BOOT button; if it fails,"
              " unplug the servo signal wire from D2 and try again.")
        sys.exit(run([cli, "upload", "-p", a.usb, "--fqbn", FQBN, "--input-dir", BUILD, SKETCH]))

    # ---- OTA
    new_pw = ota_password()                               # goes INTO the new firmware (compiled in)
    if a.ask_current:
        auth_pw = getpass.getpass("OTA password the robot has now: ")
    else:
        auth_pw = os.environ.get("OTA_CURRENT_PASSWORD", new_pw)   # what the robot checks today
    if new_pw == "change-me":
        print("Note: the OTA password is still 'change-me' (secrets.h). It works, but pick your own before"
              " using a shared network (then upload once more with --ask-current).")
    try:
        ip = socket.gethostbyname(a.ota)
    except socket.gaierror:
        sys.exit(f"Can't find '{a.ota}' on the network. Use the robot's IP (it prints it on the serial monitor"
                 " at start-up), and check the PC is on the same Wi-Fi.")
    if not a.cam:                                         # (a camera has no motors to stop)
        print(f"Telling the robot at {ip} to stop (it only takes an update while stopped) ...")
        if not tell_robot_to_stop(ip):
            print("  (no answer to 'stop' - if arena_drive.py or bot.py is driving it, quit them first)")
        time.sleep(0.5)
    print(f"Uploading {os.path.basename(binfile)} ({os.path.getsize(binfile) // 1024} KB) over Wi-Fi to {ip} ..."
          " the robot restarts with the new firmware when it's done.")
    rc = run(find_espota() + ["-i", ip, "-p", str(OTA_PORT), "--auth=" + auth_pw, "-f", binfile, "-r"],
             secret=auth_pw)
    if rc != 0:
        print("OTA upload failed. Check:\n"
              "  - the robot is switched on, on the same Wi-Fi, and STOPPED (quit arena_drive.py / bot.py)\n"
              "  - it runs firmware WITH OTA (upload once over USB first; an ESP32-CAM built with the IDE's\n"
              "    default 'Huge APP (No OTA)' answers the invitation but never takes the update)\n"
              "  - the password: if you changed it in secrets.h since the last upload, add --ask-current\n"
              "  - Windows firewall: espota.exe must be allowed on Public AND Private networks\n"
              "    (Windows Security > Firewall > Allow an app; remove any 'Block' rule for espota)\n"
              "  - it was the first try right after the Windows firewall prompt: just run it again")
    sys.exit(rc)


if __name__ == "__main__":
    main()
