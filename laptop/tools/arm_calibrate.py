"""Calibrate the pickup arm (one MG90S on D2, side scissor arm + one-way flap) and save the angles in
arena_calib.json. Nothing else moves - only the servo.

usage:  python tools/arm_calibrate.py [ROBOT_IP]

Type a command and press Enter:
  40          go to 40 degrees            +5 / -2     nudge from where it is
  stow        save the CURRENT angle as the stowed pose      (arm folded in, inside the robot outline)
  reach       ... as full reach   (flap clearly past the FAR side of a cube at the back of an alcove,
                                   and not at the scissor's mechanical limit - back off 2-3 deg from it)
  carry       ... as carrying     (flap hooks the cube and holds it against the side panel - firm, but the
                                   servo must not buzz/strain: back off 1-2 deg from where it squeezes)
  release     ... as release      (carry + about 15 mm more out: the flap no longer touches the cube)
  test        run a pick (reach -> carry) and a drop (release -> stow) with the saved angles
  show        print the saved angles          done   mark calibrated + save        q   quit (no save)

Do it with a real cube: put it against the robot's side panel / in a mock alcove (10 cm deep) next to it.
"""
import os, sys, json, socket, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.43.93"
addr = (ip, ad.CAR_PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)


def status():
    sock.sendto(b"status", addr)
    try:
        while True:
            d = sock.recvfrom(4096)[0].decode(errors="replace")
            if d.startswith("{"):
                return json.loads(d)
    except (socket.timeout, ValueError, ConnectionResetError):
        return None


def go(deg):
    deg = max(0, min(180, int(round(deg))))
    sock.sendto(f"servo {deg}".encode(), addr)
    t = time.time()
    while time.time() - t < 3:                     # wait until the servo is there (1 deg / 8 ms)
        st = status()
        if st and abs(st["servo"] - deg) <= 0:
            break
        time.sleep(0.1)
    return deg


calib = json.load(open(ad.CALIB_FILE))
deg = dict(calib.get("arm_deg", ad.ARM_DEFAULT))
st = status()
if st is None:
    print(f"No answer from the robot at {ip} - is it on and on the Wi-Fi?")
    sys.exit(1)
now = st["servo"]
print(__doc__.split("Type a command")[0].strip())
print(f"\nRobot found. Servo now at {now} deg. Saved angles: {deg}"
      f"{'' if calib.get('arm_calibrated') else '  (NOT calibrated yet - these are CAD guesses)'}")
print("Commands: <deg>  +N  -N  stow  reach  carry  release  test  show  done  q")
while True:
    try:
        c = input(f"[{now:3d} deg] > ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        break
    if not c:
        continue
    if c == "q":
        break
    if c in ("stow", "reach", "carry", "release"):
        deg[c] = now
        print(f"  {c} = {now} deg")
    elif c == "show":
        print(" ", deg)
    elif c == "test":
        for name, wait in ad.PICK_STEPS + ad.DROP_STEPS + [("stow", 0.3)]:
            print(f"  {name} ({deg[name]} deg)")
            now = go(deg[name])
            time.sleep(wait)
    elif c == "done":
        order = deg["stow"] < deg["carry"] < deg["release"] < deg["reach"] or \
                deg["stow"] > deg["carry"] > deg["release"] > deg["reach"]
        if not order:
            print("  The angles should go stow -> carry -> release -> reach in one direction "
                  f"(now {deg}). Check them, then 'done' again.")
            continue
        calib = json.load(open(ad.CALIB_FILE))      # (re-read: the main program may have saved since)
        calib["arm_deg"], calib["arm_calibrated"] = deg, True
        ad.save_calib(calib)
        print(f"  Saved in {os.path.basename(ad.CALIB_FILE)}: {deg} - the arm is calibrated.")
        break
    else:
        try:
            now = go(now + int(c)) if c[0] in "+-" else go(int(c))
        except ValueError:
            print("  ? (a number, +N, -N, stow, reach, carry, release, test, show, done, q)")
