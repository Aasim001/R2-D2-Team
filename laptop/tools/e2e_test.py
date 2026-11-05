"""End-to-end test of arena_drive.run() with the NEW code (no robot, no camera):
- background = a real frame from the current camera position (real robot's orange painted out)
- the robot is drawn as a blurred orange triangle at TAG_HEIGHT_MM (parallax right), tip pointing to its
  back (heading offset -176.4), stuck 33 mm in front of / 12 mm right of the turning point
- the triangle angle is drawn with an error of +E_ERR*cos(heading) (like the real one: east reads low)
- fake ESP32 on a local UDP port: 8-byte packets, 300 ms fail-safe, motor dead band, 90 ms camera lag
- real calib (read only: save_calib is stubbed), --auto goals, --record into the scratchpad
- optional camera drift: the whole picture slides DRIFT_PX (x) and DRIFT_PX/2 (y) between t = 10 s and 70 s
usage: python tools/e2e_test.py START_X START_Y START_HEADING STEPS [DRIFT_PX]   e.g.  150 150 90 P1,DZ 12
(background = tools/e2e_background.png, an empty-arena frame from the camera as mounted on 2026-10-08;
 replace it with a fresh empty-arena frame when the camera or the floor tape changes.
 Recordings go to tools/runs/. The real arena_calib.json is never written.)"""
import os, sys, socket, struct, threading, time, math, json, collections
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # laptop/
import arena_drive as ad
import cv2, numpy as np

SP = os.path.dirname(os.path.abspath(__file__))
sx, sy, sh, steps = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
DRIFT = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
calib = ad.load_calib()
_bg = cv2.imread(os.path.join(SP, "e2e_background.png"))     # map from the background frame itself (as at start)
calib["corners_px"], _ = ad.red_boundary_corners(_bg)
calib["focal_px"], calib["camera_mm"] = ad.camera_pose(calib["corners_px"], ad.ARENA_MM, ad.ARENA_MM, (1920, 1080))
calib0 = [list(c) for c in calib["corners_px"]]
if os.environ.get("ARM_OK"):                       # pretend the arm has been calibrated (sim only)
    calib["arm_deg_by_tag"] = {str(ad.ROBOT_TAG_ID): dict(ad.ARM_DEFAULT)}
saved = {}
ad.save_calib = lambda c: saved.update(c)          # never touch the real calib
ad.HERE = SP                                       # recordings into the scratchpad
ad.CAR_PORT = 45211
arena = arena0 = ad.Arena(calib)              # the TRUE (start) camera geometry

bg = cv2.imread(os.path.join(SP, "e2e_background.png"))
hsv = cv2.cvtColor(bg, cv2.COLOR_BGR2HSV)
orange = cv2.dilate(cv2.inRange(hsv, np.array(ad.MARK_HSV[0]), np.array(ad.MARK_HSV[1])), np.ones((25, 25), np.uint8))
x0, y0, x1, y1 = ad.Arena(calib).search_box
orange[:y0], orange[y1:], orange[:, :x0], orange[:, x1:] = 0, 0, 0, 0   # only inside the arena (keep the tape measure)
bg = cv2.inpaint(bg, orange, 5, cv2.INPAINT_TELEA)

TC = np.array([-33.0, 12.0])                       # turning point from the triangle (fwd, left) mm
OFFSET = math.radians(-176.4)                      # robot front = triangle tip + offset (map rad)
E_ERR = 6.0                                        # deg: triangle reads this much low facing east
TRACK, LAG = 150.0, 0.09
SCRUB_MMS = float(os.environ.get("SCRUB_MMS", "80"))   # (real robot: a 40-PWM ~ 100 mm/s difference hardly turned it)
import lane_follow
if "K_INT" in os.environ:
    lane_follow.K_INT = float(os.environ["K_INT"])
robot = {"c": np.array(ad.to_map(sx, sy), float), "h": -math.radians(sh), "cmd": (0, 0), "last": 0.0, "pkts": 0, "arm": []}
hist = collections.deque()
stop = threading.Event()


def wheel(p):                                      # PWM -> mm/s (dead band like the N20s)
    return math.copysign(max(0.0, abs(p) - 70) * 2.6, p)


def esp32():
    """Fake firmware: DR packets (300 ms fail-safe), and like the real one 'tele on' -> a status JSON with
    the 4 encoder counts every 100 ms (13.2 counts/mm, left wheels counting backwards)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", ad.CAR_PORT)); s.setblocking(False)
    t = time.time(); tele_to, tele_t, travel = None, 0.0, [0.0, 0.0]
    servo, servo_target = 20.0, 20.0
    while not stop.is_set():
        while True:
            try:
                d, frm = s.recvfrom(64)
            except (BlockingIOError, ConnectionResetError):
                break
            if len(d) == 8 and d[:2] == b"DR":
                l, r, _ = struct.unpack("<hhH", d[2:]); robot["cmd"], robot["last"] = (l, r), time.time(); robot["pkts"] += 1
            elif d.startswith(b"tele on"):
                tele_to = frm; s.sendto(b"ok tele on", frm)
            elif d.startswith(b"assist"):
                s.sendto(b"ok " + d, frm)
            elif d.startswith(b"servo "):
                servo_target = float(d.split()[1]); s.sendto(b"ok " + d, frm); robot["arm"].append((round(time.time() - T_START, 1), int(servo_target)))
        if time.time() - robot["last"] > 0.3:
            robot["cmd"] = (0, 0)
        now = time.time(); dt = now - t; t = now
        vl, vr = wheel(robot["cmd"][0]) * 1.05, wheel(robot["cmd"][1])    # left motor a bit stronger
        d = vr - vl                                    # skid steer: a small left/right difference just scrubs
        d = math.copysign(max(0.0, abs(d) - SCRUB_MMS), d) # the tyres sideways - the body doesn't turn
        robot["h"] += -d / TRACK * dt                  # map frame: CCW (left turn, vr > vl) lowers the angle
        robot["c"] = robot["c"] + (vl + vr) / 2 * dt * np.array([math.cos(robot["h"]), math.sin(robot["h"])])
        travel[0] += vl * dt; travel[1] += vr * dt
        servo += max(-125 * dt, min(125 * dt, servo_target - servo))   # firmware: 1 deg / 8 ms
        if tele_to and now - tele_t > 0.1:
            tele_t = now
            cnt = [int(-travel[0] * 13.2), int(travel[1] * 13.2), int(-travel[0] * 13.2), int(travel[1] * 13.2)]
            rate = [int(abs(vl) * 13.2), int(abs(vr) * 13.2)] * 2
            s.sendto(json.dumps({"cnt": cnt, "rate": rate, "ok": [1, 1, 1, 1], "servo": int(round(servo))}).encode(), tele_to)
        hist.append((now, robot["c"].copy(), robot["h"]))
        while hist and hist[0][0] < now - 1.0:
            hist.popleft()
        time.sleep(0.004)


T_START = time.time()
_shifted = {"key": None, "img": None}


def shift_now():
    k = min(1.0, max(0.0, (time.time() - T_START - 10) / 60))
    return DRIFT * k, DRIFT * k / 2


def render(c, h):
    dx, dy = shift_now()
    key = (round(dx * 4), round(dy * 4))
    if _shifted["key"] != key:                     # the camera has slid: shift the whole background
        _shifted["key"], _shifted["img"] = key, cv2.warpAffine(bg, np.float32([[1, 0, dx], [0, 1, dy]]),
                                                                (bg.shape[1], bg.shape[0]), borderMode=cv2.BORDER_REPLICATE)
    img = _shifted["img"].copy()
    fwd = np.array([math.cos(h), math.sin(h)]); left = np.array([math.sin(h), -math.cos(h)])
    mark = c - TC[0] * fwd - TC[1] * left
    o = (-math.degrees(h)) % 360
    tip = h - OFFSET + math.radians(E_ERR * math.cos(math.radians(o)))    # read as h + E*cos: east reads low
    u = np.array([math.cos(tip), math.sin(tip)]); n = np.array([-u[1], u[0]])
    tri = np.array([mark + u * 90, mark - u * 45 + n * 55, mark - u * 45 - n * 55])   # ~74 cm2, centroid at mark
    floor = arena0.drop(tri, ad.TAG_HEIGHT_MM)
    px = cv2.perspectiveTransform(floor.reshape(1, -1, 2).astype(np.float32), arena0.floor2img)[0] + [dx, dy]
    x0, y0 = np.maximum(px.min(0).astype(int) - 8, 0); x1, y1 = px.max(0).astype(int) + 8
    roi = img[y0:y1, x0:x1]
    m = np.zeros(roi.shape[:2], np.float32)
    cv2.fillPoly(m, [np.round((px - [x0, y0]) * 4).astype(np.int32)], 1.0, shift=2)
    m = cv2.GaussianBlur(m, (5, 5), 1.2)[..., None]
    img[y0:y1, x0:x1] = (roi * (1 - m) + np.array([20, 140, 230]) * m).astype(np.uint8)
    return img


class FakeCam:
    def __init__(self):
        self.frame, self.t, self.status, self.running = None, 0.0, "ok", True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            t0 = time.time()
            old = [p for p in list(hist) if p[0] <= t0 - LAG]
            c, h = (old[-1][1], old[-1][2]) if old else (robot["c"], robot["h"])
            self.frame, self.t = render(c, h), time.time()
            time.sleep(max(0, 1 / 30 - (time.time() - t0)))

    def latest(self):
        return self.frame, self.t


truth, lat = [], {"drive": [], "drive_ko": 0, "other_ko": 0}
import arena_spec as spec
def watch():                                       # true keep-out depth of the real robot body
    while not stop.is_set():
        c, h = robot["c"].copy(), robot["h"]
        d = ad.keepout_depth((c, h)); truth.append(d)
        l, r = robot["cmd"]
        o = ad.to_official(c, h)
        driving = l * r > 0 and abs(l - r) < 80    # straight-ish driving (not a pivot)
        if driving:
            hh = o[2] % 180                        # lateral error from the lane it is driving along
            e = (o[1] - min(spec.LANE_Y, key=lambda v: abs(v - o[1]))) if (hh < 45 or hh > 135) else                 (o[0] - min(spec.LANE_X, key=lambda v: abs(v - o[0])))
            if abs(e) < 8:                         # (not in a bay)
                lat["drive"].append(e)
        lat["drive_ko" if driving else "other_ko"] += d > 0.5
        time.sleep(0.05)


ad.cv2.imshow = lambda *a: None
threading.Thread(target=esp32, daemon=True).start()
threading.Thread(target=watch, daemon=True).start()
cam = FakeCam()
T0 = time.time()
try:
    ad.run(cam, calib, "127.0.0.1", record=True, auto=[s.strip() for s in steps.split(",")])
except Exception:
    import traceback; tb = sys.exc_info()[2]
    while tb.tb_next: tb = tb.tb_next
    print({k: type(v).__name__ for k, v in tb.tb_frame.f_locals.items() if k in ("found", "goal", "mode", "warn", "state", "official", "pose")}); traceback.print_exc()
cam.running = False; stop.set()
o = ad.to_official(robot["c"], robot["h"])
print(f"END: true turning point ({o[0]:.1f},{o[1]:.1f}) heading {o[2]:.0f}; packets {robot['pkts']} in {time.time()-T0:.0f} s;"
      f" worst true keep-out {max(truth):.1f} cm; frames > 0.5 cm: {sum(d > 0.5 for d in truth)}")
e = np.abs(lat["drive"]) if lat["drive"] else np.zeros(1)
print(f"TRUE lateral error while driving straight: rms {np.sqrt(np.mean(e**2)):.2f} cm, 95% {np.percentile(e, 95):.2f}, "
      f"max {e.max():.2f} cm; true keep-out samples: driving {lat['drive_ko']}, turning/stopped {lat['other_ko']}")
print("would save:", {k: saved[k] for k in ("turn_centre_mm", "heading_trim_deg") if k in saved})
if robot["arm"]:
    print("arm commands (t s, deg):", robot["arm"])
if DRIFT:
    print(f"camera drift at the end: {shift_now()[0]:.1f},{shift_now()[1]:.1f} px; saved corners moved by "
          + (str(np.round(np.array(saved['corners_px']) - np.array(calib0), 1).tolist()) if 'corners_px' in saved else "nothing (no re-map)"))
