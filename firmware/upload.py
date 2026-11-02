#!/usr/bin/env python3
"""
Compile robot_main and upload it to the ESP32 - over USB or over Wi-Fi (OTA).

  python firmware/upload.py --list                 # which COM port is the ESP32 on? (USB)
  python firmware/upload.py --usb COM5             # upload over USB (needed ONCE, to get OTA onto the robot)
  python firmware/upload.py --ota car1.local       # upload over Wi-Fi (or the robot's IP, e.g. 192.168.1.50)
  python firmware/upload.py --compile              # only check that it compiles

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
SKETCH = os.path.join(HERE, "robot_main")
BUILD = os.path.join(HERE, ".build")
FQBN = "esp32:esp32:esp32"
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
        text = open(os.path.join(SKETCH, "secrets.h"), encoding="utf-8").read()
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


def compile_sketch(cli):
    os.makedirs(BUILD, exist_ok=True)
    if run([cli, "compile", "--fqbn", FQBN, "--output-dir", BUILD, SKETCH]) != 0:
        sys.exit("Compile failed - nothing was uploaded.")
    bins = glob.glob(os.path.join(BUILD, "robot_main.ino.bin"))
    if not bins:
        sys.exit(f"Compiled, but no robot_main.ino.bin in {BUILD}")
    return bins[0]


def main():
    ap = argparse.ArgumentParser(description="Compile robot_main and upload it over USB or Wi-Fi (OTA).")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", action="store_true", help="list serial ports / boards")
    g.add_argument("--usb", metavar="PORT", help="upload over USB, e.g. COM5 (Mac: /dev/cu.usbserial-...)")
    g.add_argument("--ota", metavar="HOST", help="upload over Wi-Fi: car1.local or the robot's IP")
    g.add_argument("--compile", action="store_true", help="only compile")
    ap.add_argument("--no-compile", action="store_true", help="upload the last build without compiling again")
    ap.add_argument("--ask-current", action="store_true",
                    help="OTA: ask for the password the robot has NOW (when you just changed it in secrets.h)")
    a = ap.parse_args()
    cli = find_cli()

    if a.list:
        sys.exit(run([cli, "board", "list"]))
    binfile = os.path.join(BUILD, "robot_main.ino.bin") if a.no_compile else compile_sketch(cli)
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
              "  - it runs firmware WITH OTA (upload once over USB first)\n"
              "  - the password: if you changed it in secrets.h since the last upload, add --ask-current\n"
              "  - Windows firewall: espota.exe must be allowed on Public AND Private networks\n"
              "    (Windows Security > Firewall > Allow an app; remove any 'Block' rule for espota)\n"
              "  - it was the first try right after the Windows firewall prompt: just run it again")
    sys.exit(rc)


if __name__ == "__main__":
    main()
