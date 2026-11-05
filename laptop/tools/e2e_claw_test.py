"""End-to-end SIMULATION of bot 1's FRONT-CLAW pick-and-deliver through arena_drive.run() (no robot, no camera).
Copied from tools/e2e_test.py (2026-11-02) and adapted:
- background = a real frame of the empty arena (tools/e2e_background.png, real robot's orange painted out)
- the robot = ArUco DICT_4X4_50 id ad.ROBOT_TAG_ID (80 mm black square + 15 mm white border) flat at ad.TAG_HEIGHT_MM
  on a dark 130 x 150 mm body, with two dark claw jaws at its front that open / close with the servo. The marker sits
  at turning point - TC, TC = (fwd, left) mm (19, -11), and its "up" edge points at robot heading - heading_offset_deg
  (map frame), so arena_drive's pose (offset + turn_centre_mm = TC) is the true turning point
- ONE 50 mm cube: white top with dark QR squares at 50 mm, light grey sides. env CUBE_X, CUBE_Y (official cm,
  default 27.5, 200 = the back half of S6's alcove), CUBE_YAW (deg, default 0)
- fake ESP32 on a local UDP port: 8-byte DR packets, 300 ms fail-safe, motor dead band, skid-steer scrub, 90 ms camera
  lag, encoder telemetry, and the claw servo like robot_main.ino: limp at power-on, every angle held to 20-60, the
  first command after limp jumps there, then 1 deg / 8 ms; "servo off" = limp; telemetry has "servo" + "servo_on"
- CLAW model: the servo closing past 55 while the cube centre is 60-130 mm ahead of the turning point and within
  +-28 mm sideways ATTACHES it (it then rides 100 mm ahead of the turning point); <= 35 DETACHES it where it is;
  a loose cube that would end up < 60 mm ahead (within +-40 mm sideways) is PUSHED ahead (jaws / body).
  env CLAW_BROKEN=1: the claw never closes (the firmware still reports the angle it was told)
- calib: the real arena_calib.json, READ ONLY (save_calib is stubbed), with the claw calibrated (stow 30, carry 58,
  release 30, reach 20), turn_centre_mm = TC both ways and heading trims 0 (the drawn marker has no angle error)
usage: python tools/e2e_claw_test.py START_X START_Y START_HEADING STEPS [DRIFT_PX]
   e.g.  CUBE_X=27.5 CUBE_Y=200 python tools/e2e_claw_test.py 150 150 270 S6,PICK,DZ,DROP,P3
         CLAW_BROKEN=1 python tools/e2e_claw_test.py 150 150 270 S6,PICK,DZ,DROP
   other env: SERVO_DPS (servo deg/s, firmware 125; the real loop measured ~28), LAG_S (camera lag, 0.09),
   CMD_DELAY_S (Wi-Fi + firmware delay of every packet, 0; ~0.25 gives the real ~25 mm creep coast), SCRUB_MMS,
   K_INT, CUBE_SIDE_V (grey of the cube's sides, 185), E2E_RUNS_DIR (where runs/ goes; default tools/),
   RENDER_CHECK=1 (only check the drawing against find_robot / find_cube at a few poses, save pictures, quit),
   DIAG_NO_MIRROR=1 (DIAGNOSIS: run with find_cube's robot mask as it was before its floor-mirror term - patched
   in this process only, arena_drive.py is not changed)
(Nothing goes to the real robot: the fake ESP32 is on 127.0.0.1. The real arena_calib.json is never written.)"""
import os, sys, socket, struct, threading, time, math, json, collections
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # laptop/
import arena_drive as ad                           # (before cv2: it sets the MSMF camera option)
import arena_spec as spec
import lane_follow
import cv2, numpy as np

SP = os.path.dirname(os.path.abspath(__file__))
env = os.environ.get
RENDER_CHECK = env("RENDER_CHECK", "0") not in ("", "0")
if RENDER_CHECK and len(sys.argv) < 5:
    sys.argv += ["150", "150", "270", "S6"][len(sys.argv) - 1:]
sx, sy, sh, steps = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
DRIFT = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
CUBE_X, CUBE_Y, CUBE_YAW = float(env("CUBE_X", "27.5")), float(env("CUBE_Y", "200")), float(env("CUBE_YAW", "0"))
CLAW_BROKEN = env("CLAW_BROKEN", "0") not in ("", "0")
SERVO_DPS = float(env("SERVO_DPS", "125"))         # firmware: 1 deg / 8 ms
LAG = float(env("LAG_S", "0.09"))                  # camera lag (s)
CMD_DELAY = float(env("CMD_DELAY_S", "0"))         # every packet reaches the fake firmware this late (s)
CUBE_SIDE_V = int(env("CUBE_SIDE_V", "185"))       # grey level of the cube's side faces (top: 238, floor: ~145)
if env("DIAG_NO_MIRROR", "0") not in ("", "0"):    # DIAGNOSIS ONLY: find_cube without the floor-mirror robot mask
    import inspect, textwrap
    _src = textwrap.dedent(inspect.getsource(ad.find_cube))
    _old = "for z in (-ROBOT_TOP_MM / 2, 0.0, float(ROBOT_TOP_MM)):"
    if _old in _src:
        exec(_src.replace(_old, "for z in (0.0, float(ROBOT_TOP_MM)):"), ad.__dict__)   # (replaces ad.find_cube)
        print("DIAG_NO_MIRROR: arena_drive.find_cube patched IN THIS PROCESS ONLY - robot mask without its mirror image")
    else:
        print("DIAG_NO_MIRROR: find_cube has changed - NOT patched")

calib = ad.load_calib()             # (read only)
_bg = cv2.imread(os.path.join(SP, "e2e_background.png"))     # map from the background frame itself (as at start)
calib["corners_px"], _ = ad.red_boundary_corners(_bg)
calib["focal_px"], calib["camera_mm"] = ad.camera_pose(calib["corners_px"], ad.ARENA_MM, ad.ARENA_MM, (1920, 1080))
calib0 = [list(c) for c in calib["corners_px"]]
TC = np.array([19.0, -11.0])                       # turning point from the marker centre (fwd, left) mm
calib["turn_centre_mm"] = {"left": TC.tolist(), "right": TC.tolist()}   # -> arena_drive's pose = the turning point
calib["heading_trim_deg"] = {d: 0.0 for d in ad.TRIM_DIRS}              # (the drawn marker reads its angle exactly)
calib["arm_deg_by_tag"] = {str(ad.ROBOT_TAG_ID): {"stow": 30, "carry": 58, "release": 30, "reach": 20}}
if calib.get("heading_offset_deg") is None:
    calib["heading_offset_deg"] = -9.0
OFFSET = math.radians(calib["heading_offset_deg"])   # robot front = marker "up" + OFFSET (map rad)
saved = {}
ad.save_calib = lambda c: saved.update(c)          # never touch the real calib
ad.HERE = env("E2E_RUNS_DIR") or SP                # recordings: <that>/runs/
SIM_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # the fake ESP32's port: a free one the OS picks (env
SIM_SOCK.bind(("127.0.0.1", int(env("SIM_PORT", "0"))))       # SIM_PORT to fix it), so a second simulation running
ad.CAR_PORT = SIM_SOCK.getsockname()[1]                       # on this PC can't mix its packets into this one
arena = arena0 = ad.Arena(calib)                   # the TRUE (start) camera geometry

bg = _bg.copy()
hsv = cv2.cvtColor(bg, cv2.COLOR_BGR2HSV)
orange = cv2.dilate(cv2.inRange(hsv, np.array(ad.MARK_HSV[0]), np.array(ad.MARK_HSV[1])), np.ones((25, 25), np.uint8))
x0, y0, x1, y1 = arena0.search_box
orange[:y0], orange[y1:], orange[:, :x0], orange[:, x1:] = 0, 0, 0, 0   # only inside the arena (keep the tape measure)
bg = cv2.inpaint(bg, orange, 5, cv2.INPAINT_TELEA)

# ---- the robot and the cube (mm; heights above the floor)
BODY_LEN, BODY_WID, BODY_TOP = ad.ROBOT_LEN_MM, ad.ROBOT_WID_MM, ad.TAG_HEIGHT_MM   # body box; marker on its top
TAG_BLACK_MM, TAG_BORDER_MM = 80.0, 15.0           # marker: black square + white border
JAW_FROM, JAW_TO, JAW_W, JAW_H = 60.0, 130.0, 10.0, 40.0   # claw jaws: from / to mm ahead of the turning point
CUBE = float(ad.CUBE_MM)                           # 50 mm
HOLD_MM = 100.0                                    # an attached cube rides this far ahead of the turning point
GRIP_MIN, GRIP_MAX, GRIP_SIDE = 60.0, 130.0, 28.0  # the closing claw catches a cube centred in this window
ATTACH_DEG, DETACH_DEG = 55.0, 35.0                # servo: closes on the cube past this / lets go below this
PUSH_MM, PUSH_SIDE_MM = 60.0, 40.0                 # a loose cube can't get closer than this ahead (within +-40 sideways)
ARM_MIN, ARM_MAX, FW_STOW = 20, 60, 30             # robot_main.ino: every servo angle held to 20-60; power-on angle
JAW_STUCK_DEG = 30.0                               # CLAW_BROKEN: the jaws stay where they are (open)
BODY_COL, TOP_COL, JAW_COL = (40, 40, 46), (54, 54, 60), (62, 62, 66)
MARK_BLACK, MARK_WHITE = 28, 232
QR = [(14, 14, 12), (14, -14, 12), (-14, 14, 12), (-14, -14, 5), (-4, -6, 6), (5, 3, 5), (-8, 6, 4), (6, -10, 4)]
# ^ dark squares on the cube's top face: (fwd mm, left mm, size mm) from its centre

TRACK = 150.0
SCRUB_MMS = float(env("SCRUB_MMS", "80"))          # (real robot: a 40-PWM ~ 100 mm/s difference hardly turned it)
if "K_INT" in os.environ:
    lane_follow.K_INT = float(os.environ["K_INT"])
robot = {"c": np.array(ad.to_map(sx, sy), float), "h": -math.radians(sh), "cmd": (0, 0), "last": 0.0, "pkts": 0,
         "arm": [], "servo": float(FW_STOW), "servo_target": float(FW_STOW), "servo_on": False, "senders": set(),
         "frozen": []}
cube = {"p": np.array(ad.to_map(CUBE_X, CUBE_Y), float), "yaw": -math.radians(CUBE_YAW), "attached": False,
        "pushed_mm": 0.0, "events": [], "start": np.array(ad.to_map(CUBE_X, CUBE_Y), float)}
hist = collections.deque()
stop = threading.Event()
T_START = time.time()


def fwd_left(h):
    """Unit vectors forward / left of a map-frame heading (map y points down: left = (sin, -cos))."""
    return np.array([math.cos(h), math.sin(h)]), np.array([math.sin(h), -math.cos(h)])


def rect(c, h, a0, a1, b0, b1):
    """Floor corners (map mm) of a rectangle a0..a1 ahead, b0..b1 left of point c facing h."""
    f, l = fwd_left(h)
    return np.array([c + f * a + l * b for a, b in ((a0, b0), (a1, b0), (a1, b1), (a0, b1))])


def cube_corners_true(p, yaw):
    return rect(p, yaw, -CUBE / 2, CUBE / 2, -CUBE / 2, CUBE / 2)


def wheel(p):                                      # PWM -> mm/s (dead band like the N20s)
    return math.copysign(max(0.0, abs(p) - 70) * 2.6, p)


def t_rel():
    return round(time.time() - T_START, 1)


def claw_physics(c, h):
    """Attach / carry / detach / push the cube (called after every motion step of the fake robot)."""
    f, l = fwd_left(h)
    rel = cube["p"] - c
    ahead, side = float(rel @ f), float(rel @ l)
    servo, prev = robot["servo"], robot.get("servo_prev", robot["servo"])
    robot["servo_prev"] = servo
    if cube["attached"]:
        if servo <= DETACH_DEG:
            cube["attached"] = False
            o = ad.to_official(cube["p"])
            cube["events"].append(f"t {t_rel()} DETACHED (servo {servo:.0f}) at ({o[0]:.1f}, {o[1]:.1f}) cm")
        else:
            cube["p"], cube["yaw"] = c + HOLD_MM * f, h
        return
    closing = prev < ATTACH_DEG <= servo
    if closing:
        if CLAW_BROKEN:
            cube["events"].append(f"t {t_rel()} servo closed past {ATTACH_DEG:.0f} but the claw is BROKEN "
                                  f"(cube {ahead:.0f} mm ahead, {side:+.0f} mm left)")
        elif GRIP_MIN <= ahead <= GRIP_MAX and abs(side) <= GRIP_SIDE:
            cube["attached"] = True
            cube["events"].append(f"t {t_rel()} ATTACHED (cube was {ahead:.0f} mm ahead, {side:+.0f} mm left)")
            cube["p"], cube["yaw"] = c + HOLD_MM * f, h
            return
        else:
            cube["events"].append(f"t {t_rel()} claw closed on NOTHING (cube {ahead:.0f} mm ahead, {side:+.0f} mm left;"
                                  f" catches {GRIP_MIN:.0f}-{GRIP_MAX:.0f} ahead, +-{GRIP_SIDE:.0f} sideways)")
    if -20 < ahead < PUSH_MM and abs(side) <= PUSH_SIDE_MM:   # jaws / body push a loose cube ahead
        push = PUSH_MM - ahead
        if cube["pushed_mm"] == 0 or robot.get("push_t", 0) < time.time() - 1.0:
            cube["events"].append(f"t {t_rel()} PUSHING the cube (it was {ahead:.0f} mm ahead, {side:+.0f} mm left)")
        robot["push_t"] = time.time()
        cube["p"] = cube["p"] + push * f
        cube["pushed_mm"] += push


def esp32():
    """Fake firmware: DR packets (300 ms fail-safe), and like the real one 'tele on' -> a status JSON with the 4
    encoder counts (13.2 counts/mm, left wheels counting backwards) + servo angle every 100 ms; text commands."""
    s = SIM_SOCK; s.setblocking(False)
    t = time.time(); tele_to, tele_t, travel = None, 0.0, [0.0, 0.0]
    pending = collections.deque()                  # (arrival time, packet, sender): CMD_DELAY_S of Wi-Fi + firmware
    while not stop.is_set():
        while True:
            try:
                d, frm = s.recvfrom(64)
            except (BlockingIOError, ConnectionResetError):
                break
            pending.append((time.time(), d, frm))
        while pending and pending[0][0] + CMD_DELAY <= time.time():
            _, d, frm = pending.popleft()
            robot["senders"].add(frm)                  # (one run = one sender; more = another program on this port)
            if len(d) == 8 and d[:2] == b"DR":
                l, r, _ = struct.unpack("<hhH", d[2:]); robot["cmd"], robot["last"] = (l, r), time.time(); robot["pkts"] += 1
                continue
            txt = d.decode(errors="replace").strip().lower()
            reply = "err unknown command"
            if txt == "tele on":
                tele_to, reply = frm, "ok tele on"
            elif txt == "tele off":
                tele_to, reply = None, "ok tele off"
            elif txt in ("assist on", "assist off"):
                reply = "ok " + txt
            elif txt == "stop":
                robot["cmd"], reply = (0, 0), "ok stop"
            elif txt == "servo off":
                robot["servo_on"], reply = False, "ok servo off (limp)"
                robot["arm"].append((t_rel(), "off"))
            elif txt.startswith("servo "):
                arg = txt[6:].strip()
                if arg.isdigit():                  # (firmware parseNumber: digits only)
                    deg = int(arg)
                    tgt = min(ARM_MAX, max(ARM_MIN, deg))
                    robot["servo_target"] = float(tgt)
                    if not robot["servo_on"]:      # first command since limp: jump straight there
                        robot["servo_on"], robot["servo"] = True, float(tgt)
                    reply = f"ok servo {tgt}" + (f" (held to the safe range {ARM_MIN}-{ARM_MAX})" if tgt != deg else "")
                    robot["arm"].append((t_rel(), tgt if tgt == deg else f"{deg}->{tgt}"))
                else:
                    reply = f"err use: servo <deg {ARM_MIN}-{ARM_MAX}> | servo off"
                    robot["arm"].append((t_rel(), f"BAD '{txt}'"))
            s.sendto(reply.encode(), frm)
        if time.time() - robot["last"] > 0.3:
            robot["cmd"] = (0, 0)
        now = time.time(); dt = now - t; t = now
        if dt > 1.0:                                   # the whole program stood still (PC asleep / overloaded)
            robot["frozen"].append((t_rel(), round(dt, 1)))
        dt = min(dt, 0.05)
        vl, vr = wheel(robot["cmd"][0]) * 1.05, wheel(robot["cmd"][1])    # left motor a bit stronger
        d = vr - vl                                    # skid steer: a small left/right difference just scrubs
        d = math.copysign(max(0.0, abs(d) - SCRUB_MMS), d) # the tyres sideways - the body doesn't turn
        robot["h"] += -d / TRACK * dt                  # map frame: CCW (left turn, vr > vl) lowers the angle
        robot["c"] = robot["c"] + (vl + vr) / 2 * dt * np.array([math.cos(robot["h"]), math.sin(robot["h"])])
        travel[0] += vl * dt; travel[1] += vr * dt
        if robot["servo_on"]:                          # firmware: 1 deg / 8 ms toward the target
            robot["servo"] += max(-SERVO_DPS * dt, min(SERVO_DPS * dt, robot["servo_target"] - robot["servo"]))
        claw_physics(robot["c"], robot["h"])
        if tele_to and now - tele_t > 0.1:
            tele_t = now
            cnt = [int(-travel[0] * 13.2), int(travel[1] * 13.2), int(-travel[0] * 13.2), int(travel[1] * 13.2)]
            rate = [int(abs(vl) * 13.2), int(abs(vr) * 13.2)] * 2
            s.sendto(json.dumps({"cnt": cnt, "rate": rate, "ok": [1, 1, 1, 1], "servo": int(round(robot["servo"])),
                                 "servo_on": int(robot["servo_on"])}).encode(), tele_to)
        hist.append((now, robot["c"].copy(), robot["h"], robot["servo"], cube["p"].copy(), cube["yaw"]))
        while hist and hist[0][0] < now - 1.0:
            hist.popleft()
        time.sleep(0.004)


_shifted = {"key": None, "img": None}


def shift_now():
    k = min(1.0, max(0.0, (time.time() - T_START - 10) / 60))
    return DRIFT * k, DRIFT * k / 2


# ---------------------------------------------------------------- drawing
MK_PPM = 3.0                                       # marker picture: px per mm


def make_marker():
    d = cv2.aruco.getPredefinedDictionary(ad.TAG_DICT)
    img = cv2.aruco.generateImageMarker(d, ad.ROBOT_TAG_ID, int(round(TAG_BLACK_MM * MK_PPM)))   # 240 px = 6 cells
    b = int(round(TAG_BORDER_MM * MK_PPM))
    img = cv2.copyMakeBorder(img, b, b, b, b, cv2.BORDER_CONSTANT, value=255)
    g = np.where(img > 127, MARK_WHITE, MARK_BLACK).astype(np.uint8)
    return cv2.merge([g, g, g])


MK = make_marker()
MK_ONES = np.full(MK.shape[:2], 255, np.uint8)


class Canvas:
    """Draws projected 3-D boxes / the marker into a camera picture (anti-aliased), remembering the area touched."""
    def __init__(self, img, M):
        self.img, self.M, self.box = img, M, [10 ** 9, 10 ** 9, -10 ** 9, -10 ** 9]

    def px(self, pts, z):                          # floor-plan points (map mm) at height z -> picture px
        fl = arena0.drop(np.asarray(pts, float), z) if z else np.asarray(pts, float)
        return cv2.perspectiveTransform(fl.reshape(1, -1, 2).astype(np.float64), self.M)[0]

    def fill(self, pts_px, colour):
        hull = cv2.convexHull(np.asarray(pts_px, np.float32).reshape(-1, 1, 2)).reshape(-1, 2)
        cv2.fillConvexPoly(self.img, np.round(hull * 16).astype(np.int32), colour, lineType=cv2.LINE_AA, shift=4)
        self.box = [min(self.box[0], hull[:, 0].min()), min(self.box[1], hull[:, 1].min()),
                    max(self.box[2], hull[:, 0].max()), max(self.box[3], hull[:, 1].max())]

    def solid(self, foot, z0, z1, colour):         # a box standing on foot (4 corners) from z0 to z1: its silhouette
        self.fill(np.vstack([self.px(foot, z0), self.px(foot, z1)]), colour)

    def marker(self, centre, up):
        """The ArUco marker flat at TAG_HEIGHT_MM, its top edge toward map angle `up` (4x supersampled)."""
        u = np.array([math.cos(up), math.sin(up)]); r = np.array([-u[1], u[0]])   # (not mirrored, seen from above)
        S, p = MK.shape[0], MK_PPM
        m0 = (S - 1) / 2
        A = np.array([[r[0] / p, -u[0] / p, centre[0] - m0 / p * r[0] + m0 / p * u[0]],
                      [r[1] / p, -u[1] / p, centre[1] - m0 / p * r[1] + m0 / p * u[1]], [0, 0, 1]])
        k = arena0._k(ad.TAG_HEIGHT_MM); cx, cy = arena0.cam[:2]
        D = np.array([[1 / k, 0, cx * (1 - 1 / k)], [0, 1 / k, cy * (1 - 1 / k)], [0, 0, 1]])   # = arena.drop
        H = self.M @ D @ A                         # marker picture px -> camera picture px
        cor = cv2.perspectiveTransform(np.float64([[[-0.5, -0.5], [S - 0.5, -0.5], [S - 0.5, S - 0.5], [-0.5, S - 0.5]]]), H)[0]
        h_, w_ = self.img.shape[:2]
        x0, y0 = np.maximum(np.floor(cor.min(0)).astype(int) - 2, 0)
        x1, y1 = np.minimum(np.ceil(cor.max(0)).astype(int) + 3, [w_, h_])
        if x1 <= x0 or y1 <= y0:
            return
        F, w, hh = 4, x1 - x0, y1 - y0
        Hc = np.array([[F, 0, F * (0.5 - x0) - 0.5], [0, F, F * (0.5 - y0) - 0.5], [0, 0, 1]]) @ H
        col = cv2.warpPerspective(MK, Hc, (F * w, F * hh), flags=cv2.INTER_LINEAR, borderValue=0)
        cov = cv2.warpPerspective(MK_ONES, Hc, (F * w, F * hh), flags=cv2.INTER_LINEAR, borderValue=0)
        col = cv2.resize(col, (w, hh), interpolation=cv2.INTER_AREA).astype(np.float32)        # (premultiplied)
        a = cv2.resize(cov, (w, hh), interpolation=cv2.INTER_AREA).astype(np.float32)[..., None] / 255.0
        roi = self.img[y0:y1, x0:x1].astype(np.float32)
        self.img[y0:y1, x0:x1] = np.clip(roi * (1 - a) + col, 0, 255).astype(np.uint8)
        self.box = [min(self.box[0], x0), min(self.box[1], y0), max(self.box[2], x1), max(self.box[3], y1)]

    def soften(self, sigma=0.6):                   # a webcam is never pin-sharp
        h_, w_ = self.img.shape[:2]
        x0, y0 = max(int(self.box[0]) - 4, 0), max(int(self.box[1]) - 4, 0)
        x1, y1 = min(int(self.box[2]) + 5, w_), min(int(self.box[3]) + 5, h_)
        if x1 > x0 and y1 > y0:
            self.img[y0:y1, x0:x1] = cv2.GaussianBlur(self.img[y0:y1, x0:x1], (0, 0), sigma)


def jaw_gap(servo):
    """Jaw centre distance from the robot's axis (mm): open 20 deg -> 50, closed on a cube 58 -> 30."""
    s = JAW_STUCK_DEG if CLAW_BROKEN else min(max(servo, 20.0), 58.0)
    return CUBE / 2 + JAW_W / 2 + (58.0 - s) / (58.0 - 20.0) * 20.0


def draw_robot(cv, c, h, servo):
    cv.solid(rect(c, h, -BODY_LEN / 2, BODY_LEN / 2, -BODY_WID / 2, BODY_WID / 2), 0, BODY_TOP, BODY_COL)
    cv.fill(cv.px(rect(c, h, -BODY_LEN / 2, BODY_LEN / 2, -BODY_WID / 2, BODY_WID / 2), BODY_TOP), TOP_COL)
    g = jaw_gap(servo)
    for sgn in (-1, 1):
        cv.solid(rect(c, h, JAW_FROM, JAW_TO, sgn * g - JAW_W / 2, sgn * g + JAW_W / 2), 0, JAW_H, JAW_COL)


def draw_cube(cv, p, yaw):
    base = cube_corners_true(p, yaw)
    cv.solid(base, 0, CUBE, (CUBE_SIDE_V,) * 3)     # sides: the whole silhouette first ...
    cv.fill(cv.px(base, CUBE), (238, 238, 238))    # ... then the top face over it
    for a, b, s in QR:
        cv.fill(cv.px(rect(p, yaw, a - s / 2, a + s / 2, b - s / 2, b + s / 2), CUBE), (35, 35, 35))


def render(c, h, servo, cp, cyaw):
    dx, dy = shift_now()
    key = (round(dx * 4), round(dy * 4))
    if _shifted["key"] != key:                     # the camera has slid: shift the whole background
        _shifted["key"], _shifted["img"] = key, cv2.warpAffine(bg, np.float32([[1, 0, dx], [0, 1, dy]]),
                                                                (bg.shape[1], bg.shape[0]), borderMode=cv2.BORDER_REPLICATE)
    img = _shifted["img"].copy()
    cv = Canvas(img, np.array([[1, 0, dx], [0, 1, dy], [0, 0, 1]], float) @ arena0.floor2img)
    cam_xy = arena0.cam[:2]
    if np.hypot(*(cp - cam_xy)) < np.hypot(*(c - cam_xy)):   # painter: the one further from the camera first
        draw_robot(cv, c, h, servo); draw_cube(cv, cp, cyaw)
    else:
        draw_cube(cv, cp, cyaw); draw_robot(cv, c, h, servo)
    f, l = fwd_left(h)
    cv.marker(c - TC[0] * f - TC[1] * l, h - OFFSET)        # the marker is the highest thing: always on top
    cv.soften()
    return img


class FakeCam:
    def __init__(self):
        self.frame, self.t, self.status, self.running = None, 0.0, "ok", True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            t0 = time.time()
            old = [p for p in list(hist) if p[0] <= t0 - LAG]
            st = old[-1] if old else (t0, robot["c"], robot["h"], robot["servo"], cube["p"], cube["yaw"])
            self.frame, self.t = render(*st[1:]), time.time()
            time.sleep(max(0, 1 / 30 - (time.time() - t0)))

    def latest(self):
        return self.frame, self.t


# ---------------------------------------------------------------- RENDER_CHECK=1: drawing vs. arena_drive's finders
if RENDER_CHECK:
    det = ad.make_detector()
    out_dir = env("E2E_RUNS_DIR") or SP
    s6 = spec.STATIONS["S6"]["dock"]
    cases = [("start", (sx, sy, sh), None, 30), ("S6 dock facing the alcove", (s6[0], s6[1], 0), None, 20),
             ("S6 dock, facing N", (s6[0], s6[1], 90), None, 30), ("S6 dock, facing S", (s6[0], s6[1], 270), None, 30),
             ("S6 dock, crept 60 mm", (s6[0] + 6, s6[1], 0), None, 20), ("S6, cube held", (s6[0], s6[1], 0), "held", 58),
             ("DZ point, cube held", (70, 50, 90), "held", 58), ("P3 point", (210, 35, 90), None, 30),
             ("J10,10 facing W", (10, 10, 180), None, 30)]
    print(f"camera at map {np.round(arena0.cam, 0)} mm; heading offset {calib['heading_offset_deg']} deg; TC {TC}")
    for name, (x, y, hd), held, servo in cases:
        c, h = np.array(ad.to_map(x, y), float), -math.radians(hd)
        f, l = fwd_left(h)
        cp, cyaw = (c + HOLD_MM * f, h) if held else (cube["start"], cube["yaw"])
        frame = render(c, h, servo, cp, cyaw)
        found = ad.find_robot(frame, det, arena0)
        msg = f"{name:28s} truth ({x:.1f},{y:.1f}) {hd:.0f} deg: "
        if not found:
            print(msg + "ROBOT NOT FOUND")
            continue
        hs = found[1] + arena0.heading_offset
        hs += ad.heading_trim(arena0, hs)
        pose = (ad.mark_to_centre(found[0], hs, arena0.turn_centre), hs)
        o = ad.to_official(*pose)
        side = max(np.ptp(found[2][:, 0]), np.ptp(found[2][:, 1]))
        msg += (f"seen ({o[0]:.1f},{o[1]:.1f}) {o[2]:.1f} deg  err {math.hypot(o[0] - x, o[1] - y) * 10:.1f} mm "
                f"{(o[2] - hd + 180) % 360 - 180:+.1f} deg (marker {side:.0f} px)")
        cube_seen = ad.find_cube(frame, arena0, "S6", pose)
        co = ad.to_official(cp)
        msg += f" | find_cube S6 (robot masked): " + (f"({cube_seen[0]:.1f},{cube_seen[1]:.1f}) vs true ({co[0]:.1f},{co[1]:.1f})"
                                                       if cube_seen else f"None (true ({co[0]:.1f},{co[1]:.1f}))")
        print(msg)
        mpx = cv2.perspectiveTransform(np.float32([[c]]), arena0.floor2img)[0, 0]
        X0, Y0 = int(max(mpx[0] - 160, 0)), int(max(mpx[1] - 200, 0))
        crop = frame[Y0:Y0 + 280, X0:X0 + 320].copy()
        cv2.polylines(crop, [np.int32(found[2] - [X0, Y0])], True, (0, 255, 255), 1)
        cv2.imwrite(os.path.join(out_dir, f"render_check_{name.split(',')[0].replace(' ', '_')}_{int(hd)}.png"),
                    cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    print(f"(pictures: {out_dir}{os.sep}render_check_*.png)")
    sys.exit(0)

# ---------------------------------------------------------------- the run
truth, truth_cube, truth_at, lat = [], [], [], {"drive": [], "drive_ko": 0, "other_ko": 0}
ALCOVES = [spec.alcove_rect(s_) for s_ in spec.STATIONS]


def cube_keepout(p_mm):
    """Keep-out depth (cm) of a point of the carried cube - 0 inside / around an alcove (that is where cubes are
    picked from: the claw reaches in there on purpose)."""
    x, y = ad.to_official(p_mm)
    if any(ax0 - 3 <= x <= ax1 + 3 and ay0 - 3 <= y <= ay1 + 3 for ax0, ay0, ax1, ay1 in ALCOVES):
        return 0.0
    return spec.keepout_depth(x, y)


def watch():                                       # true keep-out depth of the real robot body (+ the cube it holds)
    while not stop.is_set():
        c, h = robot["c"].copy(), robot["h"]
        d = ad.keepout_depth((c, h)); truth.append(d)
        pose_o = tuple(round(float(v), 1) for v in ad.to_official(c, h))
        if d > 0.5:
            truth_at.append((round(d, 1), t_rel(), pose_o))
        if cube["attached"]:
            dc = max(cube_keepout(p) for p in cube_corners_true(cube["p"], cube["yaw"]))
            truth_cube.append((round(dc, 1), t_rel(), pose_o))
        l, r = robot["cmd"]
        o = ad.to_official(c, h)
        driving = l * r > 0 and abs(l - r) < 80    # straight-ish driving (not a pivot)
        if driving:
            hh = o[2] % 180                        # lateral error from the lane it is driving along
            e = (o[1] - min(spec.LANE_Y, key=lambda v: abs(v - o[1]))) if (hh < 45 or hh > 135) else \
                (o[0] - min(spec.LANE_X, key=lambda v: abs(v - o[0])))
            if abs(e) < 8:                         # (not in a bay)
                lat["drive"].append(e)
        lat["drive_ko" if driving else "other_ko"] += d > 0.5
        time.sleep(0.05)


print(f"SIM: cube at ({CUBE_X}, {CUBE_Y}) cm yaw {CUBE_YAW} deg; claw {'BROKEN (never closes)' if CLAW_BROKEN else 'ok'};"
      f" servo {SERVO_DPS:.0f} deg/s; camera lag {LAG * 1000:.0f} ms; command delay {CMD_DELAY * 1000:.0f} ms;"
      f" marker id {ad.ROBOT_TAG_ID} at "
      f"{ad.TAG_HEIGHT_MM} mm, heading offset {calib['heading_offset_deg']} deg, TC {TC.tolist()}; arm_deg {ad.arm_angles(calib)}")
if sys.platform == "win32":                        # keep the PC awake while this runs (a laptop going to sleep froze a
    try:                                           # run for minutes, 2026-11-02). Ends with this program; no setting
        import ctypes                              # is changed. ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | 0x00000001 | 0x00000002)
    except Exception:
        pass
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
time.sleep(max(0.3, CMD_DELAY + 0.2))              # (let the fake ESP32 take the last commands: servo off, tele off)
cam.running = False; stop.set()
o = ad.to_official(robot["c"], robot["h"])
print(f"END: true turning point ({o[0]:.1f},{o[1]:.1f}) heading {o[2]:.0f}; packets {robot['pkts']} in {time.time()-T0:.0f} s;"
      f" worst true keep-out {max(truth):.1f} cm; frames > 0.5 cm: {sum(d > 0.5 for d in truth)}")
print(f"fake ESP32 on 127.0.0.1:{ad.CAR_PORT} heard from {len(robot['senders'])} sender(s)"
      + ("" if len(robot["senders"]) <= 1 else f" - WARNING: another program sent to this port {sorted(robot['senders'])}"))
if robot["frozen"]:
    print(f"WARNING: the whole simulation stood still {len(robot['frozen'])} time(s) (t s, for s): {robot['frozen']} - "
          "PC asleep or overloaded; timings (and --auto's 90 s step limit) are not meaningful for this run")
if truth_at:                                       # where the body was over a lane edge (time windows)
    spans, cur = [], [truth_at[0]]
    for a in truth_at[1:]:
        if a[1] - cur[-1][1] > 0.3:
            spans.append(cur); cur = []
        cur.append(a)
    spans.append(cur)
    print("  body keep-out spans: " + "; ".join(f"t {sp[0][1]}-{sp[-1][1]} s worst {max(sp)[0]} cm at {max(sp)[2]}"
                                                 for sp in spans))
e = np.abs(lat["drive"]) if lat["drive"] else np.zeros(1)
print(f"TRUE lateral error while driving straight: rms {np.sqrt(np.mean(e**2)):.2f} cm, 95% {np.percentile(e, 95):.2f}, "
      f"max {e.max():.2f} cm; true keep-out samples: driving {lat['drive_ko']}, turning/stopped {lat['other_ko']}")
if truth_cube:
    worst = max(truth_cube)
    bad = [d for d in truth_cube if d[0] > 0.5]
    print(f"TRUE keep-out of the CARRIED cube (alcoves excluded): worst {worst[0]:.1f} cm (t {worst[1]} s, robot at "
          f"{worst[2]}); samples > 0.5 cm: {len(bad)} of {len(truth_cube)}"
          + (f", from t {bad[0][1]} to {bad[-1][1]} s" if bad else ""))
print("would save:", {k: saved[k] for k in ("turn_centre_mm", "heading_trim_deg") if k in saved})
print("ARM COMMANDS (t s, servo deg sent):", robot["arm"])
print(f"SERVO at the end: {robot['servo']:.0f} deg, " + ("still DRIVEN" if robot["servo_on"] else "LIMP (servo off)"))
print("CLAW EVENTS: " + ("; ".join(cube["events"]) or "none") + f"; pushed {cube['pushed_mm']:.0f} mm in total")
co = ad.to_official(cube["p"])
bx0, by0, bx1, by1 = spec.BAYS["DZ"]["rect"]
corners = [ad.to_official(p) for p in cube_corners_true(cube["p"], cube["yaw"])]
centre_in = bx0 <= co[0] <= bx1 and by0 <= co[1] <= by1
whole_in = all(bx0 <= p[0] <= bx1 and by0 <= p[1] <= by1 for p in corners)
print(f"CUBE END: ({co[0]:.1f}, {co[1]:.1f}) cm, " + ("ATTACHED (held by the claw)" if cube["attached"] else "loose on the floor")
      + f" (started at ({CUBE_X}, {CUBE_Y})); inside DZ rect {spec.BAYS['DZ']['rect']}: centre "
      + ("YES" if centre_in else "NO") + ", whole cube " + ("YES" if whole_in else "NO"))
if DRIFT:
    print(f"camera drift at the end: {shift_now()[0]:.1f},{shift_now()[1]:.1f} px; saved corners moved by "
          + (str(np.round(np.array(saved['corners_px']) - np.array(calib0), 1).tolist()) if 'corners_px' in saved else "nothing (no re-map)"))
