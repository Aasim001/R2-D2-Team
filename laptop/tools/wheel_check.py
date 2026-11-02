"""Wheel check: FORWARD, BACK, RIGHT, LEFT, each a short burst, and how far each of the 4 wheels turned (its
encoder). With --camera it also measures how the robot really moved (overhead webcam + its marker) and, for the
two turns, the point it turned about. The claw is not touched.

usage:  python tools/wheel_check.py ROBOT_IP [--pwm 150] [--secs 0.5] [--camera]

Put the robot on a junction (room to turn) or lift it so the wheels spin in the air (then skip --camera).
Reading it:
  * all 4 wheels should count in every move; the two wheels on one side should count about the same
  * a wheel that hardly counts while the other wheel on its side does: that motor (or its driver channel) is weak
  * a wheel that counts the opposite way to its partner on the same side: that motor's wires are swapped
  * turns: the robot should turn about a point near the marker. Far to one side = that side's wheels hardly drive
"""
import os, sys, json, math, socket, struct, time, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad                 # (first: it sets the webcam option OpenCV must see before loading)
import numpy as np

NAMES = ["FL", "FR", "RL", "RR"]
MOVES = [("FORWARD", 1, 1), ("BACK", -1, -1), ("RIGHT", 1, -1), ("LEFT", -1, 1)]

ap = argparse.ArgumentParser()
ap.add_argument("ip")
ap.add_argument("--pwm", type=int, default=150)
ap.add_argument("--secs", type=float, default=0.5)
ap.add_argument("--camera", action="store_true", help="also measure the real movement with the overhead webcam")
a = ap.parse_args()
addr = (a.ip, ad.CAR_PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)


def status():
    for _ in range(3):
        sock.sendto(b"status", addr)
        try:
            while True:
                d = sock.recvfrom(4096)[0].decode(errors="replace")
                if d.startswith("{"):
                    return json.loads(d)
        except (socket.timeout, ValueError, ConnectionResetError):
            pass
    sys.exit(f"no answer from the robot at {a.ip}")


def drive(left, right, secs):
    seq, t_end = 0, time.time() + secs
    while time.time() < t_end:
        seq = (seq + 1) & 0xFFFF
        sock.sendto(b"DR" + struct.pack("<hhH", left, right, seq), addr)
        time.sleep(0.03)
    for _ in range(3):
        seq = (seq + 1) & 0xFFFF
        sock.sendto(b"DR" + struct.pack("<hhH", 0, 0, seq), addr)
        time.sleep(0.02)


cam = arena = det = None
if a.camera:
    calib = json.load(open(ad.CALIB_FILE))
    arena, det = ad.Arena(calib), ad.make_detector()
    cam = ad.Camera(calib.get("camera", 0))
    t0 = time.time()
    while cam.latest()[0] is None and time.time() - t0 < 10:
        time.sleep(0.05)


def look(secs=0.6):
    """Marker position (map mm) and robot heading (map rad), median over secs, or None."""
    if cam is None:
        return None
    t0, last, ps, hs = time.time(), None, [], []
    while time.time() - t0 < secs:
        f, t = cam.latest()
        if f is None or t == last:
            time.sleep(0.01)
            continue
        last = t
        fd = ad.find_robot(f, det, arena)
        if fd:
            ps.append(fd[0])
            hs.append(fd[1] + (arena.heading_offset or 0.0))
    if len(ps) < 3:
        return None
    return np.median(ps, axis=0), ad.circ_mean(hs)


def pivot(p0, p1):
    """Point the robot turned about, as (forward, left) mm from the marker at the start, or None."""
    (m0, h0), (m1, h1) = p0, p1
    th = float(ad.wrap(h1 - h0))
    if abs(th) < math.radians(8):
        return None
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    C = np.linalg.solve(np.eye(2) - R, m1 - R @ m0)      # m1 - C = R (m0 - C)
    d = C - m0
    fwd = np.array([math.cos(h0), math.sin(h0)])
    left = np.array([math.sin(h0), -math.cos(h0)])       # (map y points down)
    return float(d @ fwd), float(d @ left), math.degrees(th)


sock.sendto(b"assist off", addr)
print(f"robot {a.ip}: each move {a.secs} s at PWM {a.pwm}" + ("  (+ camera)" if cam else ""))
for name, l, r in MOVES:
    before, p0 = status(), look()
    drive(l * a.pwm, r * a.pwm, a.secs)
    time.sleep(0.6)
    after, p1 = status(), look()
    d = [after["cnt"][i] - before["cnt"][i] for i in range(4)]
    mm = [ad.ENC_SIGN[i] * d[i] / ad.ENC_COUNTS_PER_MM for i in range(4)]   # + = that wheel rolled forward
    want = [l, r, l, r]
    marks = []
    for i in range(4):
        partner = i ^ 2                                  # same side: FL<->RL, FR<->RR
        if abs(mm[i]) < 2:
            marks.append("NOT TURNING")
        elif math.copysign(1, mm[i]) != want[i]:
            marks.append("WRONG WAY?")
        elif abs(mm[partner]) > 2 and abs(mm[i]) < 0.5 * abs(mm[partner]):
            marks.append("weak")
        else:
            marks.append("ok")
    print(f"\n{name:8s} wheel travel (mm, + = rolled forward):  " +
          "  ".join(f"{NAMES[i]} {mm[i]:+6.0f} {marks[i]}" for i in range(4)))
    if p0 is not None and p1 is not None:
        dm = p1[0] - p0[0]
        turn = math.degrees(float(ad.wrap(p1[1] - p0[1])))
        line = f"         camera: marker moved {np.hypot(*dm):.0f} mm, robot turned {-turn:+.0f} deg (+ = left / CCW)"
        pv = pivot(p0, p1) if l != r else None
        if pv:
            line += f", turned about a point {pv[0]:+.0f} mm forward, {pv[1]:+.0f} mm left of the marker"
        print(line)
    elif cam is not None:
        print("         camera: robot marker not seen")
if cam is not None:
    cam.running = False
print("\n(left wheels FL/RL, right wheels FR/RR. FORWARD/BACK: all 4 the same sign; RIGHT: left +, right -;"
      " LEFT: left -, right +)")
