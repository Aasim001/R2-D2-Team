#!/usr/bin/env python3
"""
Arena driver: a USB webcam on this computer (~2.5 m up) watches the whole arena, finds the robot
(a printed marker on its top plate) and drives it over Wi-Fi along the lanes of the official
competition floor (arena_spec.py) to any station or bay. Same UDP drive packets as follow_tape.py,
so the firmware (robot_main.ino) does not change.

One-time setup (the webcam must not move afterwards - if it moves, run --setup again):
  python arena_drive.py --make-tag          # robot_tag.png (print BIG, stick on the robot's top plate)
                                            # + floor_tag_10..13.png (lay them inside the blocks, at the
                                            #   positions it prints - they calibrate the webcam)
  python arena_drive.py --find              # shows each camera number - which one is the USB webcam?
  python arena_drive.py --setup --camera 1
      finds the 4 floor markers by itself: press Enter when all 4 are found.
      (no floor markers? press C and click 2 points on each side: SOUTH, EAST, NORTH, WEST)
      -> saves arena_calib.json. Check the last picture: the drawn blocks must sit on the tape.
  Measure the webcam height and put it in CAM_HEIGHT_MM (most accurate, esp. from overhead),
  and the floor -> marker height in TAG_HEIGHT_MM.

Run (put the robot in a lane, facing along it, at least 20 cm of free lane ahead):
  python arena_drive.py --car 192.168.1.50
      H     = heading test, do it once after setup: the robot drives forward a little, then turns
              right, so the script learns which way the marker faces and which way the robot turns
      1-6   = drive to station S1..S6 (stops on the docking square, pick side toward the alcove)
      D     = drive into the dispatch bay      7 / 8 / 9 = park in P1 / P2 / P3
              (arena_spec.CUTOUTS = True since 2026-10-08: drives in forward, reverses out. With
               CUTOUTS = False - solid blocks - it stops IN FRONT of the bay instead)
  Every start maps the arena from the red boundary first (checked against the black block tape), and
  during the run it re-maps by itself if the camera slides a little. --no-auto-setup skips the start.
      SPACE = pause / go on     R = roam (old "stay between the lines" mode)     Q = quit
  add --record to save the run (pictures, every decision, arrival errors) in laptop/runs/

For a different (non-competition) floor set OFFICIAL = False: then setup asks for the arena size,
you click its sides and the tape, and the robot roams between the lines.
"""
import argparse, json, math, os, socket, struct, sys, threading, time
# Windows Media Foundation: with its "hardware transforms" on, opening this webcam hangs for minutes;
# off, it opens in under a second and runs at 30 fps. Must be set before OpenCV is loaded.
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")
import cv2
import numpy as np
import arena_spec as spec                  # the official competition floor
import lane_follow                         # route following along the lanes

# ---------------- settings ----------------
CAMERA       = 0              # webcam number (see --find); setup remembers the one you used
CAM_W, CAM_H = 1920, 1080     # asked from the webcam (it gives the nearest it can); higher = marker
                              # seen from further away. 1280x720 is faster but only for a high webcam
CAM_HEIGHT_MM = None          # MEASURED height of the webcam lens above the floor (best, esp. overhead)
CAM_HFOV_DEG = 84             # or your webcam's horizontal field of view in degrees. 84 = the Lenovo FHD
                              # Webcam at 1920x1080, measured twice from the arena (at 1.3 m and 2.2 m high)
                              # Both None = work it out from the arena corners (not possible from overhead)
CAR_HOST     = "car1.local"   # or the IP the ESP32 prints
CAR_PORT     = 4210

# how the robot is found:
#   "color" = the yellow-orange TRIANGLE stuck on the robot's top (its point gives the direction;
#             the H test learns which way the point faces relative to the robot's front)
#   "aruco" = a printed ArUco marker (--make-tag)
ROBOT_MARK    = "color"
MARK_HSV      = ((9, 110, 100), (32, 255, 255))   # yellow-orange (the red boundary tape is hue 0-6)
MARK_MIN_AREA = 25            # px: smaller orange specks are ignored
MARK_AREA_CM2 = (15, 250)     # real size of the mark on the floor (the owner's triangle: ~74 cm2) - anything
MARK_CLOSE_PX = 11            # glare gaps up to about this wide (px) in the mark are bridged
MARK_MIN_FILL = 0.55          # else orange is not the robot; and it must look like a triangle (area / triangle)
TAG_DICT      = cv2.aruco.DICT_4X4_50
ROBOT_TAG_ID  = 10            # robot markers are ids 10-12 (R1-R3); 0-3 are the floor anchors (arena_spec)
ROBOT_TAG_MM  = 115           # side of a robot marker's BLACK square (sheet 13.5 cm) - measured 2026-10-10
TAG_HEIGHT_MM = 145           # floor -> top of the robot's mark (measured 2026-10-10: robot 14.5 cm, marker on top)
ROBOT_TOP_MM  = 150           # tallest part of the robot (so the robot itself is not mistaken for tape)
ROBOT_LEN_MM  = 130           # front to back
ROBOT_WID_MM  = 150           # side to side, wheels included

# top-down map
MM_PER_PX = 5                 # 1 map pixel = 5 mm of floor

# arena layout
OFFICIAL = True               # True = the official competition floor (arena_spec.py): 3 x 3 m, 4 blocks of
                              # 120 x 120 cm, 20 cm lanes, bays, stations, official coordinates. The map is
                              # exact, so the camera only has to find the robot (and the floor markers).
# Only used when OFFICIAL = False (a different floor): 4 blocks in a 2x2 grid, measured.
# None = no layout: tape is found by its colour only.
RING_MM = None                # middle of the outer tape -> outside edge of the block tape
GAP_MM  = None                # outside edge of one block's tape -> the next block's
USE_LIVE_TAPE = not OFFICIAL  # also add tape / dark objects the camera sees live (e.g. a ball on the floor).
                              # Off on the official floor: the map is exact, and on a practice floor without
                              # taped bays the block tape would wrongly close the bays.

# driving between the lines
BASE_SPEED   = 140            # PWM on straights (0-255); +/- keys change it live
MIN_PWM      = 95             # N20s stall below this
PIVOT_SPEED  = 140            # PWM when turning on the spot
KP           = 1.0            # steering strength (per radian of "the way to go" off the nose)
KD           = 0.05           # steering damping
PIVOT_DEG    = 55             # way to go more than this off the nose -> turn on the spot
NEED_MM      = ROBOT_WID_MM / 2 + 10   # robot centre must stay at least this far from any tape
LOOK_MM      = 300            # how far ahead each direction is checked
CLEAR_CAP_MM = 250            # more room than this counts the same (so it doesn't hunt in open space)
TURN_COST    = 20             # mm of room a 1 rad turn must win to be worth it (keeps it going straight)
CHANGE_COST  = 15             # same, for changing its mind between frames (keeps it steady at junctions)
LOST_S       = 0.3            # robot marker not seen this long -> stop
TEST_PWM     = 150            # speed used by the H heading test

HERE       = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(HERE, "arena_calib.json")
ROBOT_NUM  = None             # --robot N: robot N's own calibration file (arena_calib_carN.json), made by
                              # tools/bind_robots.py: its ArUco id, address, marker height and what it learnt
ROBOT_KEYS = ("tag_id", "tag_height_mm", "host", "robot", "pivot_pwm", "min_pwm", "cam_host",   # set by bind_robots.py,
              "claw")                                                                    # belong to the robot
MAP_FILE   = os.path.join(HERE, "arena_tape_map.png")
VIEW_H     = 480              # height of each half of the window


def wrap(a):
    """Angle(s) into -pi..pi."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def circ_mean(angles):
    return math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))


# ============================ CAMERA ============================
def open_camera(src):
    if isinstance(src, int):
        # Windows: Media Foundation first - DirectShow ignores the MJPG request on some webcams (the
        # Lenovo FHD here) and sends uncompressed 1080p, which USB only manages at ~5 fps (MSMF: 30 fps)
        apis = {"win32": [cv2.CAP_MSMF, cv2.CAP_DSHOW], "darwin": [cv2.CAP_AVFOUNDATION]}.get(sys.platform, [cv2.CAP_ANY])
        for api in apis:
            cap = cv2.VideoCapture(src, api)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # compressed = 30 fps over USB
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)                           # focus changes would shift the picture
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if cap.read()[0]:
                return cap
            cap.release()
        return None
    cap = cv2.VideoCapture(src)                                          # stream URL also works
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if cap.isOpened():
        return cap
    cap.release()
    return None


class Camera:                               # always holds the newest frame (no lag build-up)
    def __init__(self, src):
        self.src, self.frame, self.t, self.running, self.status = src, None, 0.0, True, "connecting"
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            cap = open_camera(self.src)
            if cap is None:
                self.status = f"cannot open camera {self.src} - retrying"
                time.sleep(1.0)
                continue
            self.status, fails = "ok", 0
            while self.running and fails < 30:              # ~1 s of failed reads -> reconnect
                ok, f = cap.read()
                if ok:
                    fails = 0
                    with self.lock:
                        self.frame, self.t = f, time.time()
                else:
                    fails += 1
                    time.sleep(0.03)
            cap.release()
            self.status = "camera dropped - reconnecting"

    def latest(self):
        with self.lock:
            return self.frame, self.t


def waiting_screen(text):
    img = np.zeros((360, 640, 3), np.uint8)
    for i, line in enumerate(["Waiting for camera...", text, "", "Q = quit"]):
        cv2.putText(img, line[:70], (15, 60 + 40 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
    return img


def label(img, lines, y0=22):
    for i, line in enumerate(lines):
        y = y0 + 24 * i
        cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
        cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)


# ============================ GEOMETRY ============================
def camera_pose(corners_px, W, H, size):
    """Where the webcam is, from the 4 clicked corners: (focal length px, [x, y, height] mm).
    Needed because the marker is ~14 cm above the floor: seen from the side it looks
    further away than it is, and this is used to pull it back to the right place."""
    w, h = size
    img = np.float32(corners_px)
    floor = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
    G = cv2.getPerspectiveTransform(floor, img)            # floor mm -> picture px
    f_rect = _focal_from_rectangle(G, w, h)                # None when seen (nearly) from straight above
    f = None
    if CAM_HEIGHT_MM is not None:
        # measured webcam height: the focal length that puts the camera at that height. Seen from the
        # side, height(f) rises and then falls again, so TWO focal lengths can fit - take the one
        # closest to what the rectangle itself says (from overhead there is only one).
        fs = w * np.exp(np.linspace(np.log(0.3), np.log(4), 800))
        d = np.array([_camera_at(G, fv, w, h)[2] - CAM_HEIGHT_MM for fv in fs])
        roots = [fs[i] - d[i] * (fs[i + 1] - fs[i]) / (d[i + 1] - d[i])
                 for i in range(len(fs) - 1) if d[i] == 0 or d[i] * d[i + 1] < 0]
        if not roots:
            f = float(fs[np.argmin(np.abs(d))])
            print(f"No focal length puts the webcam at CAM_HEIGHT_MM = {CAM_HEIGHT_MM} - is it right? Using the closest.")
        elif len(roots) == 1 or f_rect is None:
            f = float(roots[0])
        else:
            f = float(min(roots, key=lambda r: abs(r - f_rect)))
    elif CAM_HFOV_DEG is not None:
        f = w / (2 * math.tan(math.radians(CAM_HFOV_DEG / 2)))
    else:
        f = f_rect
        if f is None:
            print("Webcam field of view can't be measured from this view - assuming 70 deg. For an overhead"
                  " webcam set CAM_HEIGHT_MM (best) or CAM_HFOV_DEG at the top of the file.")
            f = w / (2 * math.tan(math.radians(70 / 2)))
    return f, _camera_at(G, f, w, h)


def _focal_from_rectangle(G, w, h):
    """Focal length from the floor rectangle's square corners, or None if it can't be measured.
    r1 . r2 = 0 and |r1| = |r2| each give a + b * f^2 = 0. Least squares over both, so a badly
    conditioned one (b ~ 0, e.g. a centred side view) gets ~no weight. From (nearly) straight above
    f can't be measured at all."""
    Gc = np.array([[1, 0, -w / 2], [0, 1, -h / 2], [0, 0, 1]]) @ G
    g1, g2 = Gc[:, 0], Gc[:, 1]
    a1, b1 = g1[0] * g2[0] + g1[1] * g2[1], g1[2] * g2[2]
    a2, b2 = g1[0] ** 2 + g1[1] ** 2 - g2[0] ** 2 - g2[1] ** 2, g1[2] ** 2 - g2[2] ** 2
    overhead = all(abs(g[2]) * 5 * w < math.hypot(g[0], g[1]) for g in (g1, g2))
    if not overhead and b1 * b1 + b2 * b2 > 0:
        c = -(a1 * b1 + a2 * b2) / (b1 * b1 + b2 * b2)
        if c > 0 and 0.3 * w < math.sqrt(c) < 4 * w:
            return math.sqrt(c)
    return None


def _camera_at(G, f, w, h):
    """Camera position [x, y, height] mm for a focal length f. G = K [r1 r2 t] (up to scale):
    take the camera's rotation and position straight out of it."""
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
    M = np.linalg.inv(K) @ G
    lam = 2 / (np.linalg.norm(M[:, 0]) + np.linalg.norm(M[:, 1]))
    if M[2, 2] < 0:                                        # the arena is in front of the camera
        lam = -lam
    r1, r2, t = M[:, 0] * lam, M[:, 1] * lam, M[:, 2] * lam
    U, _, Vt = np.linalg.svd(np.column_stack([r1, r2, np.cross(r1, r2)]))
    c = -(U @ Vt).T @ t
    return [float(c[0]), float(c[1]), float(abs(c[2]))]


# ---- official frame (cm, x east, y NORTH, heading counter-clockwise) <-> this program's map frame
# (mm, x east, y SOUTH = down the screen, so the map shows north up; heading clockwise on screen)
ARENA_MM = spec.SIZE * 10


def to_map(x_cm, y_cm):
    return x_cm * 10.0, (spec.SIZE - y_cm) * 10.0


def to_official(pos_mm, heading=None):
    """Map position (mm) [+ heading rad] -> official (x cm, y cm [, heading deg 0 = east, CCW])."""
    x, y = pos_mm[0] / 10.0, spec.SIZE - pos_mm[1] / 10.0
    return (x, y) if heading is None else (x, y, (-math.degrees(heading)) % 360)


def _rect_to_map(r):
    x0, y0, x1, y1 = r
    return (x0 * 10.0, (spec.SIZE - y1) * 10.0, x1 * 10.0, (spec.SIZE - y0) * 10.0)


def layout_rects(W, H):
    """(keep-out rectangles, drivable rectangles inside them) in map mm."""
    if OFFICIAL:
        return [_rect_to_map(r) for r in spec.keepout_rects()], [_rect_to_map(r) for r in spec.drivable_rects()]
    return layout_blocks(W, H), []


def layout_blocks(W, H):
    """The 4 blocks (x0, y0, x1, y1 in mm) from RING_MM / GAP_MM, or [] if not set."""
    if RING_MM is None or GAP_MM is None:
        return []
    bw, bh = (W - 2 * RING_MM - GAP_MM) / 2, (H - 2 * RING_MM - GAP_MM) / 2
    if bw <= 0 or bh <= 0:
        print("RING_MM / GAP_MM don't fit in the arena size - layout ignored")
        return []
    xs = [(RING_MM, RING_MM + bw), (W - RING_MM - bw, W - RING_MM)]
    ys = [(RING_MM, RING_MM + bh), (H - RING_MM - bh, H - RING_MM)]
    return [(x0, y0, x1, y1) for x0, x1 in xs for y0, y1 in ys]


def footprint(pos, heading, margin=0.0):
    """4 floor corners (mm) of the robot's outline."""
    l, w = ROBOT_LEN_MM / 2 + margin, ROBOT_WID_MM / 2 + margin
    fwd = np.array([math.cos(heading), math.sin(heading)])
    side = np.array([-fwd[1], fwd[0]])
    p = np.asarray(pos, float)
    return np.array([p + fwd * l + side * w, p + fwd * l - side * w, p - fwd * l - side * w, p - fwd * l + side * w])


def mark_to_centre(mark, heading, centre):
    """Marker position (map mm) -> the point the robot turns about (between its wheels). The marker
    is rarely stuck right over it; centre = (forward, left) mm from the marker to that point, so an
    on-the-spot turn doesn't look like a sideways jump. (Map y points down: 'left' = (sin, -cos).)"""
    c, s = math.cos(heading), math.sin(heading)
    return (mark[0] + centre[0] * c + centre[1] * s, mark[1] + centre[0] * s - centre[1] * c)


def fit_turn_centre(segments):
    """Where the robot turns about, relative to its marker, from turns seen by the camera.
    segments = one list per turn of (marker xy map mm, robot heading rad); each turn gets its own
    centre (the robot may slide a bit between turns). Least squares on marker = centre - f*fwd - l*left.
    Returns ((f, l) mm, median misfit mm), or None if no turn was big enough to tell."""
    segs = [s for s in segments if len(s) >= 5 and np.ptp(np.unwrap([p[1] for p in s])) > math.radians(25)]
    if not segs:
        return None
    n, rows, b = len(segs), [], []
    for k, seg in enumerate(segs):
        for (mx, my), h in seg:
            c, s = math.cos(h), math.sin(h)
            rx, ry = [0.0] * (2 * n + 2), [0.0] * (2 * n + 2)
            rx[2 * k], rx[-2], rx[-1] = 1.0, -c, -s
            ry[2 * k + 1], ry[-2], ry[-1] = 1.0, -s, c
            rows += [rx, ry]
            b += [mx, my]
    A, b = np.array(rows), np.array(b)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    r = (A @ sol - b).reshape(-1, 2)
    return (float(sol[-2]), float(sol[-1])), float(np.median(np.hypot(r[:, 0], r[:, 1])))


class Arena:
    """Everything that comes from setup: floor <-> picture mapping, tape colour, camera position."""
    def __init__(self, calib, load_map=True):
        self.W, self.H = calib["arena_w"], calib["arena_h"]
        self.frame_size = tuple(calib["frame_size"])                      # (w, h)
        self.size = (int(self.W / MM_PER_PX) + 1, int(self.H / MM_PER_PX) + 1)   # map (w, h) in px
        self.ranges = [(np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in calib["tape_ranges"]]
        off = calib.get("heading_offset_deg")
        self.heading_offset = None if off is None else math.radians(off)
        self.turn_sign = calib.get("turn_sign", 1)
        tc = calib.get("turn_centre_mm", [0.0, 0.0])          # (fwd, left) mm of the marker, per turn direction
        self.turn_centres = {d: np.array(tc[d] if isinstance(tc, dict) else tc, float) for d in ("left", "right")}
        self.turn_centre = mean_turn_centre(self.turn_centres)
        self.turn_fits = {}                                    # recent turning-point fits per direction (this run)
        self.turn_confirmed = False                            # a turn this run agreed with the stored turning point
        trim = calib.get("heading_trim_deg", {})               # facing E, N, W, S (official degrees)
        self.trims = [float(trim.get(d, 0.0)) for d in TRIM_DIRS]
        self.enc_rad_per_mm = float(calib.get("enc_rad_per_mm", ENC_RAD_PER_MM))
        self.enc_learnt = int(calib.get("enc_turns_learnt", 0))   # turns it has been learnt from
        self.new_mark = False                                      # SYNC found the mark turned / moved
        self.set_corners(calib["corners_px"], calib["camera_mm"])
        self.blocks, self.bays = layout_rects(self.W, self.H)
        if self.blocks:                                    # map drawn from the measurements / official plan
            saved = np.zeros((self.size[1], self.size[0]), np.uint8)
            for rects, val in ((self.blocks, 255), (self.bays, 0)):      # blocks solid, bays cut out of them
                for x0, y0, x1, y1 in rects:
                    cv2.rectangle(saved, (int(round(x0 / MM_PER_PX)), int(round(y0 / MM_PER_PX))),
                                  (int(round(x1 / MM_PER_PX)) - 1, int(round(y1 / MM_PER_PX)) - 1), val, -1)
        else:                                              # map = tape seen at setup
            saved = cv2.imread(MAP_FILE, cv2.IMREAD_GRAYSCALE) if load_map else None
            if saved is None or saved.shape != (self.size[1], self.size[0]):
                if load_map:
                    print("No tape map from setup - using only what the camera sees live")
                saved = np.zeros((self.size[1], self.size[0]), np.uint8)
        self.saved = saved

    def set_corners(self, corners, camera_mm):
        """Picture <-> floor mapping from the arena corners in the picture (also used to re-map live when
        the camera shifts a little during a run)."""
        self.corners = np.float32(corners)
        self.cam = np.array(camera_mm, float)
        floor = np.float32([[0, 0], [self.W, 0], [self.W, self.H], [0, self.H]])
        self.img2floor = cv2.getPerspectiveTransform(self.corners, floor)
        self.floor2img = cv2.getPerspectiveTransform(floor, self.corners)
        self.img2map = np.diag([1 / MM_PER_PX, 1 / MM_PER_PX, 1]) @ self.img2floor
        full = np.full((self.frame_size[1], self.frame_size[0]), 255, np.uint8)
        self.in_view = cv2.erode(cv2.warpPerspective(full, self.img2map, self.size, flags=cv2.INTER_NEAREST),
                                 np.ones((3, 3), np.uint8))               # map area the webcam can see
        # part of the picture where the marker can be: the arena, and the arena raised to robot height
        raised = cv2.perspectiveTransform(self.drop(floor, ROBOT_TOP_MM).reshape(1, 4, 2).astype(np.float32),
                                          self.floor2img)[0]
        pts = np.vstack([self.corners, raised])
        x0, y0 = np.maximum(pts.min(0) - 40, 0).astype(int)
        x1, y1 = np.minimum(pts.max(0) + 40, self.frame_size).astype(int)
        self.search_box = (x0, y0, x1, y1)
        x0, y0 = np.maximum(self.corners.min(0) - 4, 0).astype(int)
        x1, y1 = np.minimum(self.corners.max(0) + 4, self.frame_size).astype(int)
        self.floor_box = (x0, y0, x1, y1)
        self.crop2map = self.img2map @ np.array([[1, 0, x0], [0, 1, y0], [0, 0, 1]], float)

    # a point h mm above the floor appears (on the floor picture) further from the camera than it is
    def _k(self, h):
        return max(0.1, (self.cam[2] - h) / self.cam[2])

    def lift(self, pts, h):      # where it looks on the floor -> where it really is
        c = self.cam[:2]
        return c + (np.asarray(pts) - c) * self._k(h)

    def drop(self, pts, h):      # where it really is -> where it looks on the floor
        c = self.cam[:2]
        return c + (np.asarray(pts) - c) / self._k(h)

    def look(self, frame):
        """Top-down picture of the arena + where the tape is in it (live).
        The tape colour is checked in the full camera picture first: seen from the side, far-away
        tape is only 2-3 pixels thick, and stretching it to top-down first would wash it out."""
        top = cv2.warpPerspective(frame, self.img2map, self.size, flags=cv2.INTER_LINEAR)
        if not self.ranges or (self.blocks and not USE_LIVE_TAPE):
            return top, np.zeros((self.size[1], self.size[0]), np.uint8)
        x0, y0, x1, y1 = self.floor_box                    # only the part of the picture with the floor in it
        hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        mask = cv2.warpPerspective(mask, self.crop2map, self.size, flags=cv2.INTER_LINEAR)
        mask = np.where(mask > 60, 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # remove specks + thin grout lines
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # join tape broken by reflections
        return top, mask & self.in_view

    def shadow(self, pos, heading):
        """Map area covered by the robot's body as the webcam sees it (ignored when looking for tape)."""
        foot = footprint(pos, heading, margin=25)
        pts = np.vstack([foot, self.drop(foot, ROBOT_TOP_MM)]) / MM_PER_PX
        m = np.zeros((self.size[1], self.size[0]), np.uint8)
        cv2.fillConvexPoly(m, cv2.convexHull(pts.astype(np.int32)), 255)
        return m

    def clearance(self, tape):
        """For every map pixel: mm to the nearest tape (the arena edge counts as tape)."""
        free = np.where(tape > 0, 0, 255).astype(np.uint8)
        free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = 0
        return cv2.distanceTransform(free, cv2.DIST_L2, 5) * MM_PER_PX


# ============================ ROBOT MARKER ============================
def make_detector():
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.minMarkerPerimeterRate = 0.01                   # allow small (far away) markers
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(TAG_DICT), params)


def detect_tag(detector, gray):
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        for c, i in zip(corners, ids.ravel()):
            if i == ROBOT_TAG_ID:
                return c.reshape(4, 2)
    return None


def anchors_in(frame, detector):
    """Corners (px, 4 x 2) of the floor anchors (spec.FLOOR_MARKERS) found in the picture: {id: corners}."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found = {}
    for scale in (1, 2):                                   # small/far markers: try the picture enlarged too
        img = gray if scale == 1 else cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = detector.detectMarkers(img)
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if int(i) in spec.FLOOR_MARKERS and int(i) not in found:
                    found[int(i)] = (c.reshape(4, 2) + 0.5) / scale - 0.5
        if len(found) == len(spec.FLOOR_MARKERS):
            break
    return found


def floor_marker_centres(frame, detector):
    """Centres (px) of the floor markers (spec.FLOOR_MARKERS) found in the picture: {id: (x, y)}."""
    return {i: tuple(c.mean(0)) for i, c in anchors_in(frame, detector).items()}


def corners_from_anchors(anchors):
    """Arena corners (px, map order NW, NE, SE, SW) from all 4 floor anchors: (corners, fit rms mm, worst mm).
    The 4 centres give a first mapping; each detected corner is then matched to the nearest corner of that
    anchor's known square (so it doesn't matter which way an anchor is turned), and all 16 corners plus the
    centres give a least-squares mapping. The anchors lie flat on the floor, so there is no height parallax."""
    ids = sorted(spec.FLOOR_MARKERS)
    rough = cv2.getPerspectiveTransform(np.float32([anchors[i].mean(0) for i in ids]),
                                        np.float32([to_map(*spec.FLOOR_MARKERS[i]) for i in ids]))
    half = spec.FLOOR_MARKER_CM * 10 / 2
    src, dst = [], []
    for i in ids:
        cx, cy = to_map(*spec.FLOOR_MARKERS[i])
        known = np.array([[cx - half, cy - half], [cx + half, cy - half], [cx + half, cy + half], [cx - half, cy + half]])
        seen = cv2.perspectiveTransform(np.float32(anchors[i]).reshape(1, 4, 2), rough)[0]
        match = [int(np.argmin(np.linalg.norm(known - p, axis=1))) for p in seen]
        if len(set(match)) == 4:                           # (each detected corner on a different known corner)
            src += list(anchors[i])
            dst += [known[k] for k in match]
        src += [anchors[i].mean(0)] * 4                    # the centre counts as much as the 4 corners
        dst += [(cx, cy)] * 4
    src, dst = np.float32(src), np.float32(dst)
    H, _ = cv2.findHomography(src, dst, 0)                 # picture -> map mm, least squares
    err = np.linalg.norm(cv2.perspectiveTransform(src.reshape(1, -1, 2), H)[0] - dst, axis=1)
    floor = np.float32([[[0, 0], [ARENA_MM, 0], [ARENA_MM, ARENA_MM], [0, ARENA_MM]]])
    return (cv2.perspectiveTransform(floor, np.linalg.inv(H))[0].tolist(), float(np.sqrt(np.mean(err ** 2))),
            float(err.max()))


def red_boundary_corners(frame):
    """Arena corners (map order NW, NE, SE, SW) found from the red outer boundary tape, with no clicks.
    The side nearest the bottom of the picture is taken as SOUTH (fine while the floor has no
    cut-outs - it's symmetric). Returns (corners, report) or (None, why).
      1. rough corners = extreme ends of LONG straight red lines (small red things don't count)
      2. along each side: the reddest pixel across the line at 200 points (works where the tape is
         faint, far away, or partly hidden by people) -> robust (RANSAC) straight-line fit
      3. corners = where the 4 fitted lines meet"""
    I = frame.astype(np.int32)
    redness = cv2.GaussianBlur((I[:, :, 2] - np.maximum(I[:, :, 1], I[:, :, 0])).astype(np.float32), (3, 3), 0)
    segs = cv2.HoughLinesP((redness >= 25).astype(np.uint8) * 255, 1, np.pi / 360, threshold=60,
                           minLineLength=120, maxLineGap=25)
    if segs is None or len(segs) < 4:
        return None, "no long red lines found - is the red boundary in view?"
    P = np.vstack([segs.reshape(-1, 4)[:, :2], segs.reshape(-1, 4)[:, 2:]]).astype(np.float64)
    s, d = P[:, 0] + P[:, 1], P[:, 0] - P[:, 1]
    rough = {"NW": P[np.argmin(s)], "NE": P[np.argmax(d)], "SE": P[np.argmax(s)], "SW": P[np.argmin(d)]}
    h, w = frame.shape[:2]
    rng = np.random.default_rng(0)
    lines, report = {}, []
    for side, (ka, kb) in {"N": ("NW", "NE"), "E": ("NE", "SE"), "S": ("SE", "SW"), "W": ("SW", "NW")}.items():
        a, b = rough[ka], rough[kb]
        L = np.linalg.norm(b - a)
        if L < 100:
            return None, f"the {side} side looks too short - are all 4 corners of the red boundary in view?"
        u = (b - a) / L
        v = np.array([-u[1], u[0]])
        # (all as array operations, no Python loops: this also runs in a thread beside the driving loop,
        # and Python loops would hold up that loop)
        ts = np.linspace(0.05, 0.95, 200) * L              # 200 points along the side x 37 across it
        xy = (a + ts[:, None, None] * u + np.arange(-18, 19)[None, :, None] * v).round().astype(int)
        inside = (xy[..., 0] >= 0) & (xy[..., 0] < w) & (xy[..., 1] >= 0) & (xy[..., 1] < h)
        r = np.full(inside.shape, -np.inf, np.float32)
        r[inside] = redness[xy[..., 1][inside], xy[..., 0][inside]]
        k = np.argmax(r, axis=1)                           # the reddest pixel across the line at each point
        rows = np.arange(len(ts))
        pts = xy[rows, k][r[rows, k] >= 8].astype(np.float32)
        if len(pts) < 10:
            return None, f"too little red found along the {side} side"
        i, j = rng.integers(0, len(pts), (2, 400))         # RANSAC: ignore points that aren't the tape
        d = pts[j] - pts[i]
        n = np.linalg.norm(d, axis=1)
        ok = n >= 30
        if not ok.any():
            return None, f"the red along the {side} side is too short"
        nrm = np.column_stack([-d[ok, 1], d[ok, 0]]) / n[ok, None]   # normal of each candidate line
        dist = np.abs(pts @ nrm.T - (pts[i[ok]] * nrm).sum(axis=1))  # (points, candidates)
        best = dist[:, int(np.argmax((dist < 2.5).sum(axis=0)))] < 2.5
        vx, vy, x0, y0 = cv2.fitLine(pts[best], cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        lines[side] = (np.array([x0, y0], float), np.array([vx, vy], float))
        report.append(f"{side}: {int(best.sum())}/{len(pts)} points")

    def meet(s1, s2):
        (p1, d1), (p2, d2) = lines[s1], lines[s2]
        return p1 + np.linalg.solve(np.column_stack([d1, -d2]), p2 - p1)[0] * d1

    corners = [meet("W", "N"), meet("N", "E"), meet("E", "S"), meet("S", "W")]
    return [c.tolist() for c in corners], ", ".join(report)


TAPE_INSET_MM = 15            # the black block tape's centre line is this far inside the block edge (measured)
TAPE_FIT_MAX_MM = 12          # a map whose block edges are further than this (median) from the tape is wrong


def tape_fit_mm(frame, corners):
    """How well a map fits the floor: (median distance mm between where the map puts the block edges and
    where the black tape really is, share of edge points where tape was found). Measured 2026-10-08 on
    the empty arena: right map 5 mm; corners 3 px off 10 mm, 6-8 px off 15-20 mm, 12 px off 20-30 mm.
    Independent of the red boundary, so it can tell a real camera move from someone standing on the red
    tape. Official floor only (it needs the block plan)."""
    n = int(ARENA_MM // MM_PER_PX)
    H = cv2.getPerspectiveTransform(np.float32(corners), np.float32([[0, 0], [n, 0], [n, n], [0, n]]))
    gray = cv2.GaussianBlur(cv2.cvtColor(cv2.warpPerspective(frame, H, (n, n)), cv2.COLOR_BGR2GRAY),
                            (3, 3), 0).astype(np.float32)
    bays = [_rect_to_map(b) for b in spec.drivable_rects()]
    d = np.arange(-60, 61, 2.5)                            # mm across the edge, + = into the block
    xs, ys = [], []
    for x0, y0, x1, y1 in (_rect_to_map(r) for r in spec.keepout_rects()):
        for (ax, ay), (bx, by), (ix, iy) in (((x0, y0), (x1, y0), (0, 1)), ((x0, y1), (x1, y1), (0, -1)),
                                             ((x0, y0), (x0, y1), (1, 0)), ((x1, y0), (x1, y1), (-1, 0))):
            L = max(abs(bx - ax), abs(by - ay))
            for s in np.arange(60, L - 59, 50):            # every 5 cm, not at the corners
                px, py = ax + (bx - ax) * s / L, ay + (by - ay) * s / L
                if any(b[0] - 30 <= px <= b[2] + 30 and b[1] - 30 <= py <= b[3] + 30 for b in bays):
                    continue                               # (a bay's opening: no tape across it)
                xs.append((px + ix * d) / MM_PER_PX)
                ys.append((py + iy * d) / MM_PER_PX)
    prof = cv2.remap(gray, np.float32(xs), np.float32(ys), cv2.INTER_LINEAR)     # one row per edge point
    k = np.argmin(prof, axis=1)
    dark = prof[np.arange(len(k)), k] < 0.6 * np.median(prof, axis=1)
    if not dark.any():
        return 999.0, 0.0
    return float(np.median(np.abs(d[k[dark]] - TAPE_INSET_MM))), float(dark.mean())


TAPE_STEP_CM = 2.0            # measure_tape: one profile across the tape every this many cm along each side
TAPE_CONTRAST = 25            # ... the tape must be at least this much darker than the floor (green channel, 0-255)
TAPE_WIDTH_MM = (8, 35)       # ... and this wide (grout lines are ~4 mm; a robot or a foot is much wider)
TAPE_FIT_TOL_MM = 2.5         # ... points this close to a side's fitted line count; the rest (glare, robots) don't
TAPE_MIN_POINTS = 8


def measure_tape(frame, corners):
    """Where the lane-side edge of every tape line really is (arena_spec.SIDES: the 4 sides of each block and
    the outer boundary), from the camera picture, to a fraction of a pixel.
    Along each side, every TAPE_STEP_CM: the brightness profile across the tape is read straight from the full
    camera picture (green channel - black AND red tape are dark in it) at 0.5 mm steps on the floor, through the
    floor <-> picture mapping (tape lies flat on the floor: no height parallax). The edge is where the profile
    crosses halfway between the floor's and the tape's brightness, interpolated between samples. A straight line
    is fitted per side (RANSAC: glare gaps, robots, cubes and feet are left out).
    Returns {side: (offset mm at the side's middle, slope mm per cm, points used, rms mm)}; + = into the lane
    from the official edge. Sides that can't be measured are left out (arena_spec then uses the median)."""
    floor = np.float32([[0, 0], [ARENA_MM, 0], [ARENA_MM, ARENA_MM], [0, ARENA_MM]])
    f2i = cv2.getPerspectiveTransform(floor, np.float32(corners))         # map mm -> picture px
    green = frame[:, :, 1].astype(np.float32)
    d = np.arange(-45.0, 30.01, 0.5)                                     # mm across the edge, + = into the lane
    rng = np.random.default_rng(0)
    out = {}
    for name, s in spec.SIDES.items():
        (ax, ay), (bx, by), (nx, ny), L = s["a"], s["b"], s["n"], s["length"]
        ux, uy = (bx - ax) / L, (by - ay) / L
        ts = np.array([t for t in np.arange(6.0, L - 5.99, TAPE_STEP_CM)
                       if not any(lo <= t <= hi for lo, hi in s["skip"])])
        if len(ts) < TAPE_MIN_POINTS:
            continue
        px_off = ax + ux * ts[:, None] + nx * d[None, :] / 10                # official cm
        py_off = ay + uy * ts[:, None] + ny * d[None, :] / 10
        pts = np.stack([px_off * 10, (spec.SIZE - py_off) * 10], axis=-1).reshape(1, -1, 2).astype(np.float32)
        img = cv2.perspectiveTransform(pts, f2i)[0].reshape(len(ts), len(d), 2)
        prof = cv2.remap(green, img[..., 0], img[..., 1], cv2.INTER_LINEAR, borderValue=-1)
        found = []
        for k, t in enumerate(ts):
            p = prof[k]
            if (p < 0).any():                                            # (off the picture)
                continue
            lane_v = float(np.median(p[d >= 15]))
            near = d <= 5
            i = int(np.argmin(np.where(near, p, np.inf)))
            tape_v = float(p[i])
            if lane_v - tape_v < TAPE_CONTRAST:
                continue                                                 # no clearly dark tape here
            level = (lane_v + tape_v) / 2
            j = i                                                        # from the darkest spot toward the lane
            while j < len(d) - 1 and p[j] < level:
                j += 1
            k2 = i                                                       # ... and toward the block
            while k2 > 0 and p[k2] < level:
                k2 -= 1
            if j == len(d) - 1 or k2 == 0:
                continue
            e = d[j - 1] + (level - p[j - 1]) / (p[j] - p[j - 1]) * (d[j] - d[j - 1])   # crossing, interpolated
            e_far = d[k2] + (level - p[k2]) / (p[k2 + 1] - p[k2]) * (d[k2 + 1] - d[k2])
            if TAPE_WIDTH_MM[0] <= e - e_far <= TAPE_WIDTH_MM[1]:
                found.append((t, e))
        if len(found) < TAPE_MIN_POINTS:
            continue
        T, E = np.array(found).T
        best = None
        for _ in range(150):                                             # RANSAC line: edge = a + b (t - L/2)
            i1, i2 = rng.choice(len(T), 2, replace=False)
            if abs(T[i2] - T[i1]) < 10:
                continue
            b = (E[i2] - E[i1]) / (T[i2] - T[i1])
            a = E[i1] - b * (T[i1] - L / 2)
            inl = np.abs(E - (a + b * (T - L / 2))) <= TAPE_FIT_TOL_MM
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.sum() < max(TAPE_MIN_POINTS, 0.3 * len(T)):
            continue
        b, a = np.polyfit(T[best] - L / 2, E[best], 1)                   # least squares on the inliers
        rms = float(np.sqrt(np.mean((E[best] - (a + b * (T[best] - L / 2))) ** 2)))
        out[name] = (round(float(a), 2), round(float(b), 4), int(best.sum()), round(rms, 2))
    return out


def tape_report(edges):
    """One line per side + what it means for the lanes."""
    lines = [f"  {n:6s} edge {v[0]:+5.1f} mm  slope {v[1]:+.3f} mm/cm  ({v[2]} points, {v[3]:.1f} mm rms)"
             for n, v in sorted(edges.items())]
    missing = sorted(set(spec.SIDES) - set(edges))
    if missing:
        lines.append("  not measured (median used): " + ", ".join(missing))
    return "\n".join(lines)


def apply_tape(calib):
    """Make arena_spec follow the measured tape saved in the calibration (lane centres, nodes, keep-out)."""
    edges = calib.get("tape_edges") or {}
    spec.set_tape({k: (v[0], v[1]) for k, v in edges.items()})
    return edges


def draw_tape(img, corners, s=1.0):
    """Draw the measured tape edges (cyan) and the thick safety boundary (red) on a camera picture."""
    floor = np.float32([[0, 0], [ARENA_MM, 0], [ARENA_MM, ARENA_MM], [0, ARENA_MM]])
    G = cv2.getPerspectiveTransform(floor, np.float32(corners))
    def poly(pts, col, closed=True):
        m = np.float32([to_map(*p) for p in pts]).reshape(1, -1, 2)
        cv2.polylines(img, [np.int32(cv2.perspectiveTransform(m, G)[0] * s)], closed, col, 1)
    for safety, col in ((0, (255, 255, 0)), (spec.SAFETY_CM, (0, 0, 255))):
        g = spec._geometry(safety)
        poly(g["outer"], col)
        for a, b in g["segs"]:
            poly([a, b], col, False)


AUTO_MAP_LOOKS = 5            # auto-mapping: find the red boundary in this many frames ...
AUTO_MAP_AGREE_PX = 4         # ... and keep those within this of the median (a person in the way spoils one)


def map_boundary(cam):
    """The arena corners from the red boundary, seen in several frames: (corners, frame, report) or
    (None, None, why). One frame can be spoilt by someone standing on the tape; the median of
    AUTO_MAP_LOOKS, averaged over the frames that agree with it, can't."""
    t0 = time.time()
    while cam.latest()[0] is None and time.time() - t0 < 20:
        time.sleep(0.05)
    if cam.latest()[0] is None:
        return None, None, "no picture from the camera: " + cam.status
    time.sleep(1.5)                                        # let exposure / colour settle
    looks, why, last_t = [], "", 0.0
    while len(looks) < AUTO_MAP_LOOKS and time.time() - t0 < 30:
        frame, t = cam.latest()
        if t == last_t:
            time.sleep(0.03)
            continue
        last_t = t
        corners, report = red_boundary_corners(frame)
        if corners is None:
            why = report
            time.sleep(0.2)
            continue
        looks.append((np.array(corners), frame, report))
    if len(looks) < 3:
        return None, None, why or "the red boundary was found in too few frames"
    C = np.array([c for c, _, _ in looks])                 # (looks, 4 corners, xy)
    off = np.linalg.norm(C - np.median(C, axis=0), axis=2).max(axis=1)
    good = off <= AUTO_MAP_AGREE_PX
    if good.sum() < 3:
        return None, None, (f"the boundary moved between frames by up to {off.max():.0f} px - someone "
                            "walking on it, or the camera still moving?")
    frame, report = [(f, r) for (_, f, r), g in zip(looks, good) if g][-1]
    return C[good].mean(axis=0).tolist(), frame, f"{report}; {int(good.sum())}/{len(looks)} frames agree"


def map_anchors(cam):
    """The arena corners from the 4 floor anchors (spec.FLOOR_MARKERS), each corner the median over
    AUTO_MAP_LOOKS frames that show all 4: (corners, frame, report) or (None, None, why)."""
    detector, t0, last_t, looks, frame, seen = make_detector(), time.time(), 0.0, [], None, set()
    while cam.latest()[0] is None and time.time() - t0 < 20:
        time.sleep(0.05)
    while len(looks) < AUTO_MAP_LOOKS and time.time() - t0 < 25:
        f, t = cam.latest()
        if f is None or t == last_t:
            time.sleep(0.03)
            continue
        last_t = t
        a = anchors_in(f, detector)
        seen |= set(a)
        if len(a) == len(spec.FLOOR_MARKERS):
            looks.append(a)
            frame = f
    if len(looks) < 3:
        return None, None, (f"floor anchors {sorted(spec.FLOOR_MARKERS)}: only {sorted(seen) or 'none'} seen"
                            " (placed? in view? not covered?)")
    med = {i: np.median([lk[i] for lk in looks], axis=0) for i in spec.FLOOR_MARKERS}
    corners, rms, worst = corners_from_anchors(med)
    return corners, frame, f"{len(looks)} frames, anchor corners fit within {rms:.1f} mm rms ({worst:.1f} worst)"


def map_arena(cam):
    """The arena map at start: from the floor anchors and/or the red boundary, whichever lies better on the
    black block tape (the anchors also work at the venue, where the boundary is black). Prints how the two
    agree. Returns (corners, frame, report, calibrated_by) or (None, None, why, None)."""
    maps = []
    for name, fn in (("floor anchors (auto)", map_anchors), ("red boundary lines (auto)", map_boundary)):
        corners, frame, report = fn(cam)
        if corners is None:
            print(f"  {name}: {report}")
            continue
        fit, share = tape_fit_mm(frame, corners)
        maps.append((fit, corners, frame, f"{report}; black tape fit {fit:.0f} mm (seen at {share:.0%})", name))
        print(f"  {name}: {maps[-1][3]}")
    if not maps:
        return None, None, "neither the floor anchors nor the red boundary were found", None
    if len(maps) == 2:
        d = max(np.hypot(*(np.array(a) - b)) for a, b in zip(maps[0][1], maps[1][1]))
        print(f"  the two maps' arena corners differ by up to {d:.1f} px")
    fit, corners, frame, report, name = min(maps, key=lambda m: m[0])
    return corners, frame, report, name


def auto_setup(cam, src):
    """Map the arena from the floor anchors and/or the red boundary - no clicks. Done at every start (and by
    --auto-setup). Keeps what belongs to the robot (H test, turning point, heading trim).
    Returns the new calibration, or None."""
    corners, frame, report, how = map_arena(cam)
    if corners is None:
        print("Auto-mapping failed:", report)
        return None
    h, w = frame.shape[:2]
    fit, found = tape_fit_mm(frame, corners)               # does the map agree with the black block tape?
    if fit > TAPE_FIT_MAX_MM:
        print(f"Auto-mapping: the map is {fit:.0f} mm off the black block tape (tape seen at {found:.0%} of the"
              " edge points) - something on the boundary or the floor? Not used.")
        return None
    focal, cam_mm = camera_pose(corners, float(ARENA_MM), float(ARENA_MM), (w, h))
    old = json.load(open(CALIB_FILE)) if os.path.exists(CALIB_FILE) else {}
    calib = dict(camera=src, frame_size=[w, h], arena_w=float(ARENA_MM), arena_h=float(ARENA_MM),
                 corners_px=corners, tape_ranges=[], focal_px=round(focal, 1),
                 camera_mm=[round(c, 1) for c in cam_mm], official=True, calibrated_by=how,
                 heading_offset_deg=old.get("heading_offset_deg"), turn_sign=old.get("turn_sign", 1),
                 turn_centre_mm=old.get("turn_centre_mm", [0.0, 0.0]),
                 heading_trim_deg=old.get("heading_trim_deg", {}))
    for key in ("arm_deg", "arm_calibrated", "enc_rad_per_mm", "enc_turns_learnt") + ROBOT_KEYS:   # (the robot's)
        if key in old:
            calib[key] = old[key]
    moved = ""
    if old.get("corners_px"):                              # (the H test result is kept: it belongs to the
        moved = max(np.hypot(*(np.array(a) - b)) for a, b in zip(old["corners_px"], corners))   # robot, not
        moved = f", corners moved {moved:.1f} px since last time"                          # the camera)
    edges = measure_tape(frame, corners)                   # where the tape REALLY is (to a fraction of a px)
    if len(edges) >= len(spec.SIDES) // 2:
        calib["tape_edges"] = {k: list(v) for k, v in edges.items()}
    elif old.get("tape_edges"):
        calib["tape_edges"] = old["tape_edges"]          # (too little seen now: keep the last measurement)
    apply_tape(calib)
    save_calib(calib)
    print(f"Arena mapped from the {how.replace(' (auto)', '')} ({report}{moved}). Webcam about {cam_mm[2] / 1000:.2f} m up"
          + (" (CAM_HEIGHT_MM)" if CAM_HEIGHT_MM else "") + f". Fits the black tape within {fit:.0f} mm.")
    shift = max(math.hypot(v["xy"][0] - spec._OFFICIAL_XY[n][0], v["xy"][1] - spec._OFFICIAL_XY[n][1])
                for n, v in spec.NODES.items())
    print(f"  Tape measured on {len(edges)}/{len(spec.SIDES)} sides (lane-side edges up to "
          f"{max([abs(v[0]) for v in edges.values()] or [0]):.0f} mm from the plan); lane centres moved up to "
          f"{shift:.1f} cm. Every tape line counts {spec.SAFETY_CM:.1f} cm thicker.")
    out = frame.copy()
    draw_arena(out, corners, ARENA_MM, ARENA_MM)
    draw_tape(out, corners)
    cv2.imwrite(os.path.join(HERE, "auto_setup_check.jpg"), out)
    print("  (picture: auto_setup_check.jpg - the drawn blocks must sit on the black tape)")
    return calib


def corners_from_floor_markers(centres):
    """Picture positions of the 4 arena corners (map order: NW, NE, SE, SW) from the floor markers'
    centres - they are at known places on the official floor, so no clicking is needed."""
    ids = sorted(spec.FLOOR_MARKERS)
    src = np.float32([centres[i] for i in ids])
    dst = np.float32([to_map(*spec.FLOOR_MARKERS[i]) for i in ids])
    img2floor = cv2.getPerspectiveTransform(src, dst)
    floor = np.float32([[[0, 0], [ARENA_MM, 0], [ARENA_MM, ARENA_MM], [0, ARENA_MM]]])
    return cv2.perspectiveTransform(floor, np.linalg.inv(img2floor))[0].tolist()


def detect_in(detector, gray, box, scale):
    """Look for the marker inside box (x0, y0, x1, y1), enlarged by scale - a far-away marker is
    only a few pixels big, and enlarging it first makes it much easier to find."""
    x0, y0, x1, y1 = box
    crop = gray[max(0, y0):max(0, y1), max(0, x0):max(0, x1)]
    if crop.size == 0:
        return None
    if scale > 1:
        crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    c = detect_tag(detector, crop)
    # (c + 0.5) / scale - 0.5 = pixel-centre convention of cv2.resize
    return None if c is None else (c + 0.5) / scale - 0.5 + [max(0, x0), max(0, y0)]


def find_robot_color(frame, arena, hint=None, prev_heading=None):
    """The robot by its yellow-orange triangle: (centre mm, direction of the triangle's point rad,
    outline px) or None. The outline is un-warped to a top-down view at the mark's height first,
    so the triangle keeps its true shape even though the webcam sees it at a slant.
    prev_heading (rad, from the last frame): in the ~2 % of frames where the wrong corner looks
    like the apex, the corner that continues the previous direction is taken instead."""
    x0, y0, x1, y1 = arena.search_box
    crop = frame[y0:y1, x0:x1]
    mask = cv2.inRange(cv2.cvtColor(crop, cv2.COLOR_BGR2HSV), np.array(MARK_HSV[0]), np.array(MARK_HSV[1]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # the mark is glossy: a ceiling light's reflection can cut a white stripe through it (worst near the
    # camera) and split it in two - close such gaps so it stays one triangle
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MARK_CLOSE_PX,) * 2))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    best = None
    for c in contours:
        if cv2.contourArea(c) < MARK_MIN_AREA:
            continue
        px = c.reshape(-1, 2).astype(np.float32) + [x0, y0]
        floor = arena.lift(cv2.perspectiveTransform(px.reshape(1, -1, 2), arena.img2floor)[0], TAG_HEIGHT_MM)
        m = cv2.moments(floor.astype(np.float32))
        if m["m00"] <= 0:
            continue
        centre = np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])
        if not (-100 < centre[0] < arena.W + 100 and -100 < centre[1] < arena.H + 100):
            continue                                           # outside the arena (desks, people...)
        area_cm2 = m["m00"] / 100.0
        if not MARK_AREA_CM2[0] <= area_cm2 <= MARK_AREA_CM2[1]:
            continue                                           # wrong size for the mark (an arm, a speck...)
        tri_area = cv2.minEnclosingTriangle(floor.reshape(-1, 1, 2).astype(np.float32))[0]
        if tri_area <= 0 or m["m00"] / tri_area < MARK_MIN_FILL:
            continue                                           # not triangle-shaped
        # pick the patch nearest where the robot was last seen; without that, the biggest one
        score = (np.linalg.norm(px.mean(0) - hint.mean(0)) if hint is not None else 0) - 1e-3 * len(px)
        if best is None or score < best[0]:
            best = (score, centre, floor, px)
    if best is None or (hint is not None and best[0] > 200):   # (jumped too far: not the robot)
        return None
    _, centre, floor, px = best
    # direction, from the area moments of the outline (top-down, true shape) - every pixel counts, no
    # corner to pick (the old corner method chose the wrong tip in ~1 of 8 frames far from the camera,
    # and read 3x noisier): the 2nd moments give the two principal axes; along each, the 3rd moment (skew)
    # says which way the thin end (the tip) lies. The axis with the stronger skew gives the direction
    # (for this wide triangle: across its long base, toward the tip).
    m = cv2.moments(floor.reshape(-1, 1, 2).astype(np.float32))
    th = 0.5 * math.atan2(2 * m["mu11"], m["mu20"] - m["mu02"])
    l1 = (m["mu20"] + m["mu02"]) / 2 + math.hypot((m["mu20"] - m["mu02"]) / 2, m["mu11"])
    l2 = m["mu20"] + m["mu02"] - l1
    dirs, skews = [], []
    for ax, var in ((th, l1), (th + math.pi / 2, l2)):
        c, s = math.cos(ax), math.sin(ax)
        m3 = m["mu30"] * c ** 3 + 3 * m["mu21"] * c * c * s + 3 * m["mu12"] * c * s * s + m["mu03"] * s ** 3
        k = m3 / (m["m00"] * max(var / m["m00"], 1e-9) ** 1.5)
        dirs += [ax if k > 0 else ax + math.pi, ax if k <= 0 else ax + math.pi]   # (tip way, other way)
        skews.append(abs(k))
    a = shape = 0 if skews[0] > skews[1] else 2
    if prev_heading is not None and abs(wrap(dirs[a] - prev_heading)) > math.radians(60):
        # the robot can't have turned that far since the last frame: take the axis direction nearest it
        a = min(range(4), key=lambda i: abs(wrap(dirs[i] - prev_heading)))
    return centre, wrap(dirs[a]), px, wrap(dirs[shape])   # (last: what the shape alone says, for run()'s check)


def find_robot(frame, detector, arena, hint=None, prev_heading=None):
    """(centre mm, mark direction rad, mark outline px, direction from this frame alone) or None.
    ROBOT_MARK picks how.
    hint = the mark's outline/corners (px) from a recent frame, prev_heading = its direction then.
    hint = marker corners from a recent frame: look there first (small crop, enlarged to ~80 px),
    then in the whole arena as it is, then the whole arena enlarged 2x (a small marker can be
    missed at one size and found at another, so it tries more than one)."""
    if ROBOT_MARK == "color":
        return find_robot_color(frame, arena, hint, prev_heading)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tries = [(arena.search_box, 1), (arena.search_box, 2)]
    if hint is not None:
        (x0, y0), (x1, y1) = (hint.min(0) - 60).astype(int), (hint.max(0) + 60).astype(int)
        side = max(np.ptp(hint[:, 0]), np.ptp(hint[:, 1]), 1)
        tries.insert(0, ((x0, y0, x1, y1), float(np.clip(80 / side, 1, 3))))
    for box, scale in tries:
        c = detect_in(detector, gray, box, scale)
        if c is not None:
            break
    else:
        return None
    pts = cv2.perspectiveTransform(c.reshape(1, 4, 2).astype(np.float32), arena.img2floor)[0]
    pts = arena.lift(pts, TAG_HEIGHT_MM)
    up = (pts[0] + pts[1]) / 2 - (pts[2] + pts[3]) / 2       # marker's top edge direction
    return pts.mean(0), math.atan2(up[1], up[0]), c, math.atan2(up[1], up[0])


def make_tag():
    d = cv2.aruco.getPredefinedDictionary(TAG_DICT)
    img = cv2.aruco.generateImageMarker(d, ROBOT_TAG_ID, 600)
    b = 600 // 6                                           # white border = one marker cell (needed!)
    # same border on all 4 sides (label written inside it), so the middle of the sheet IS the middle
    # of the black square - centre the sheet and the marker is centred
    img = cv2.copyMakeBorder(img, b, b, b, b, cv2.BORDER_CONSTANT, value=255)
    cv2.putText(img, f"robot marker id {ROBOT_TAG_ID}", (b, 600 + b + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2)
    path = os.path.join(HERE, "robot_tag.png")
    cv2.imwrite(path, img)
    print(f"Saved {path}\nPrint it as BIG as the robot's top plate allows: the whole white square about"
          f" 130 mm\n(black square 115 mm, as on the robots now). Bigger = seen from further away - from a webcam ~2 m up a"
          f" small\nmarker can't be read at the far side of the arena. Cut along the OUTER edge: the white"
          f" border is needed.\nStick it flat and centred on the top plate (it may cover the 76 mm recess),"
          f" then measure\nfloor -> marker and put it in TAG_HEIGHT_MM (now {TAG_HEIGHT_MM}).")
    if OFFICIAL:                                           # 4 floor markers for automatic calibration
        for i, (x, y) in sorted(spec.FLOOR_MARKERS.items()):
            img = cv2.aruco.generateImageMarker(d, i, 600)
            img = cv2.copyMakeBorder(img, b, b, b, b, cv2.BORDER_CONSTANT, value=255)   # centred (see above)
            cv2.putText(img, f"floor marker {i}: centre at x={x} cm, y={y} cm", (b, 600 + b + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2)
            cv2.imwrite(os.path.join(HERE, f"floor_tag_{i}.png"), img)
        print(f"\nSaved floor_tag_{min(spec.FLOOR_MARKERS)}.png .. floor_tag_{max(spec.FLOOR_MARKERS)}.png. Print each"
              " with the black square 140 mm (sheet 190 mm) and lay"
              "\nit flat on the floor INSIDE a block (robots never drive there), the CENTRE OF THE BLACK SQUARE"
              "\n(= the centre of the sheet) as exactly as you can (each 3 mm off costs ~0.5 cm at the ring) at:")
        for i, (x, y) in sorted(spec.FLOOR_MARKERS.items()):
            print(f"   marker {i}: x = {x} cm, y = {y} cm   (from the bottom-left corner: x east, y north)")
        print("Which way each marker is turned does not matter. Robot + floor markers = 5 of the 10 allowed"
              " (7 with all 3 robots).")


# ============================ PLANNER ============================
ANGLES = np.radians(np.arange(-90, 91, 6))                 # directions tried, relative to the nose (+ = right)
STEPS  = np.arange(30, LOOK_MM + 1, 20, dtype=float)       # points checked along each direction (mm)


class Planner:
    """Each frame: try directions from 90 deg left to 90 deg right, pick the one with the most room
    between the lines (= the middle of the lane), steer toward it."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.prev_abs, self.smooth, self.prev, self.t, self.stuck_side = None, 0.0, 0.0, time.time(), 0

    def choose(self, dist, pos, heading):
        h, w = dist.shape

        def room(xs, ys):
            ix, iy = np.round(xs / MM_PER_PX).astype(int), np.round(ys / MM_PER_PX).astype(int)
            ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            r = np.zeros(xs.shape)
            r[ok] = dist[iy[ok], ix[ok]]
            return r

        here = float(room(np.array([pos[0]]), np.array([pos[1]]))[0])
        # already too close? then just don't get closer - but tape itself (< 20 mm) always blocks
        need = min(NEED_MM, max(here - 10, 20))
        dirs = heading + ANGLES
        clear = room(pos[0] + np.outer(np.cos(dirs), STEPS), pos[1] + np.outer(np.sin(dirs), STEPS))
        blocked = clear < need
        first = np.where(blocked.any(1), blocked.argmax(1), len(STEPS))   # points before the first blocked one
        valid = np.arange(len(STEPS))[None, :] < first[:, None]
        score = (np.minimum(clear, CLEAR_CAP_MM) * valid).mean(1) - TURN_COST * np.abs(ANGLES)
        if self.prev_abs is not None:
            score -= CHANGE_COST * np.abs(wrap(dirs - self.prev_abs))
        best = int(np.argmax(score))
        free = np.where(first > 0, STEPS[np.maximum(first - 1, 0)], 0.0)  # free mm along each direction
        stuck = bool(first[best] == 0)
        if stuck:
            if not self.stuck_side:                         # pick a side once, then keep turning that way
                right, left = clear[ANGLES > 0, :3].sum(), clear[ANGLES < 0, :3].sum()
                self.stuck_side = 1 if right >= left else -1
        else:
            self.stuck_side, self.prev_abs = 0, dirs[best]
        return dict(delta=float(ANGLES[best]), free=float(free[best]), stuck=stuck, clear_here=here,
                    dirs=dirs, free_each=free, best=best)

    def command(self, plan, base, turn_sign):
        now = time.time()
        dt, self.t = max(now - self.t, 1e-3), now
        if plan["stuck"]:                                   # tape all around the nose: turn on the spot
            s = self.stuck_side * PIVOT_SPEED * turn_sign
            return (s, -s), "STUCK"
        self.smooth = 0.5 * self.smooth + 0.5 * plan["delta"]
        d = self.smooth
        dd, self.prev = (d - self.prev) / dt, d
        if abs(d) > math.radians(PIVOT_DEG):                # way to go is far to the side: turn on the spot
            s = math.copysign(PIVOT_SPEED, d) * turn_sign
            return (s, -s), "PIVOT"
        turn = (KP * d + KD * dd) * base * turn_sign
        speed = base * (1 - 0.5 * min(1.0, abs(d) / math.radians(PIVOT_DEG)))   # slower in bends
        speed *= 0.5 + 0.5 * min(1.0, plan["free"] / LOOK_MM)                    # slower near tape ahead
        return (speed + turn, speed - turn), "DRIVE"


def to_pwm(v):
    if abs(v) < 20:
        return 0
    return int(np.sign(v) * min(255, max(MIN_PWM, abs(v))))


# ============================ HEADING TEST (H key) ============================
# still -> forward -> still -> BACK -> still -> turn right -> still -> turn LEFT -> still   (until s, (left, right))
# Coming back and turning back leaves the robot where it started (e.g. on a junction, where there's
# room to turn), lined up as before - it never turns out in a lane.
TEST_PHASES = [(0.3, (0, 0)), (0.9, (1, 1)), (1.4, (0, 0)), (2.0, (-1, -1)), (2.5, (0, 0)),
               (2.9, (1, -1)), (3.5, (0, 0)), (3.9, (-1, 1)), (4.4, (0, 0))]


def heading_test_step(test, now, found):
    """Returns (command, state, test or None when finished). Also runs the SYNC move (test["kind"])."""
    el = now - test["t0"]
    if found:
        test["log"].append((el, found[0], found[1]))
    sync = test.get("kind") == "sync"
    for until, (l, r) in (SYNC_PHASES if sync else TEST_PHASES):
        if el < until:
            pwm = SYNC_PWM * test.get("sign", 1) if sync else TEST_PWM
            return (l * pwm, r * pwm), ("SYNC" if sync else "HEADING TEST"), test
    return (0, 0), "STOPPED", None


# ============================ SYNC (every --auto start) ============================
# A short straight move (no turning: it never swings out of the lane): still -> forward -> still -> back
# -> still (or back first if there's no room ahead). The way it moved vs. what the triangle says = the
# heading offset, measured afresh at every start (the triangle may have been knocked since last time).
SYNC_PWM = 115                # gentle: ~6-8 cm each way
SYNC_PHASES = [(0.5, (0, 0)), (0.95, (1, 1)), (1.7, (0, 0)), (2.15, (-1, -1)), (2.9, (0, 0))]
SYNC_ROOM_MM = 110            # free lane needed along the heading for the move
SYNC_NEW_MARK_DEG = 20        # heading offset changed more than this = the mark was turned / moved on the robot


def room_to_turn(pose):
    """Could the robot turn on the spot here (+-50 deg, as the H test does) without its outline crossing
    into keep-out? (only at a junction, normally)"""
    allow = max(0.5, keepout_depth(pose) + 0.2)
    return all(keepout_depth((pose[0], pose[1] + math.radians(a))) <= allow for a in range(-50, 51, 10))


def room_along(pose, dist_mm):
    """Would the robot's outline stay out of keep-out if it drove dist_mm along its heading (- = back)?"""
    c, h = np.asarray(pose[0], float), pose[1]
    d = np.array([math.cos(h), math.sin(h)])
    allow = max(0.5, keepout_depth(pose) + 0.2)            # (cm; it may already be a little over now)
    return all(keepout_depth((c + d * dist_mm * f, h)) <= allow for f in np.linspace(0.25, 1, 4))


def sync_result(log, calib, arena, sign):
    """After the SYNC move: set the heading offset from how the robot really moved. Returns False if the
    robot didn't move properly (then --auto stops instead of driving blind)."""
    win = lambda a, b: [p for p in log if a <= p[0] < b]
    start, mid, end = win(0.15, 0.5), win(1.35, 1.7), win(2.55, 2.9)     # the still moments
    if not (start and mid and end):
        print("SYNC: the robot's mark wasn't seen the whole time - can't sync.")
        return False
    a, b, c = (np.mean([p[1] for p in w], 0) for w in (start, mid, end))
    d1, d2 = b - a, c - b                                  # first leg, second leg (opposite ways)
    fwd = sign * (d1 - d2)                                 # = 2 x the forward move
    if min(np.hypot(*d1), np.hypot(*d2)) < 25:
        print(f"SYNC: the robot hardly moved ({np.hypot(*d1):.0f} / {np.hypot(*d2):.0f} mm) - battery? Wi-Fi? "
              "Not driving.")
        return False
    agree = abs(wrap(math.atan2(d1[1], d1[0]) - math.atan2(-d2[1], -d2[0])))
    tag_dir = circ_mean([p[2] for p in start + mid + end])
    travel = h = math.atan2(fwd[1], fwd[0])
    for _ in range(5):                                     # (same as the H test: keep the trim out of it)
        h = travel - heading_trim(arena, h)
    offset = wrap(h - tag_dir)
    old = arena.heading_offset
    change = math.degrees(wrap(offset - old)) if old is not None else float("nan")
    if agree > math.radians(25):
        print(f"SYNC: the two legs disagree by {math.degrees(agree):.0f} deg (slipping? bumped?) - offset kept.")
        return True
    arena.heading_offset = offset
    calib["heading_offset_deg"] = round(math.degrees(offset), 1)
    if old is not None and abs(change) > SYNC_NEW_MARK_DEG:
        # the mark was turned / moved / replaced: the turning point and trims belonged to the old one
        arena.turn_centres = {d: np.zeros(2) for d in arena.turn_centres}
        arena.turn_centre, arena.trims = np.zeros(2), [0.0] * 4
        calib.update(learnt_calib(arena))
        arena.new_mark = True
        print(f"SYNC: the mark has been turned or moved ({change:+.0f} deg) - forgetting its old turning point "
              "and heading trims; they are learnt again.")
    save_calib(calib)
    print(f"SYNC OK: moved {np.hypot(*fwd) / 2:.0f} mm each way (legs agree within {math.degrees(agree):.0f} deg); "
          f"robot front = triangle {calib['heading_offset_deg']:+.1f} deg ({change:+.1f} deg vs last time).")
    return True


def heading_test_result(log, calib, arena):
    win = lambda a, b: [p for p in log if a <= p[0] < b]
    start, moved, turned = win(0, 0.3), win(1.1, 1.4), win(3.1, 3.5)   # the still moments
    if not (start and moved and turned):
        print("Heading test: robot marker not seen the whole time - is it visible and well lit?")
        return
    motion = np.mean([p[1] for p in moved], 0) - np.mean([p[1] for p in start], 0)
    if np.hypot(*motion) < 30:
        print("Heading test: the robot hardly moved. Battery switch on? Right --car address?")
        return
    tag_dir = circ_mean([p[2] for p in start + moved])
    # run() turns a triangle angle a into the robot's heading as h = a + offset, then h += heading_trim(h):
    # pick the offset that makes THAT equal the way it moved (else the trim would count twice)
    travel, h = math.atan2(motion[1], motion[0]), math.atan2(motion[1], motion[0])
    for _ in range(5):
        h = travel - heading_trim(arena, h)
    offset = wrap(h - tag_dir)
    robot_h = lambda a: a + offset + heading_trim(arena, a + offset)
    # follow the angle THROUGH the turn (unwrapped), so a fast turn of more than 180 deg still reads right
    seg = [p for p in log if 2.2 <= p[0] < 3.5]                    # still (back at the start) -> turned
    gaps = np.diff([p[0] for p in seg])
    a = np.unwrap([p[2] for p in seg])
    t = np.array([p[0] for p in seg])
    if not (t < 2.5).any() or not (t >= 3.1).any():
        print("Heading test: robot marker not seen around the turn - try again.")
        return
    spin = float(np.mean(a[t >= 3.1]) - np.mean(a[t < 2.5]))
    calib["heading_offset_deg"] = round(math.degrees(offset), 1)
    # where it turns about, relative to the marker: from the right turn and the left turn (windows open at
    # the turn commands, so the coasting from reversing isn't mistaken for part of the turn). One short
    # turn each way replaces nothing learnt yet, but only half of what many driving turns taught it.
    rate = 0.5 if any(np.any(c) for c in arena.turn_centres.values()) else 1.0
    for a0, a1 in ((2.5, 3.5), (3.5, 4.5)):
        learn_turn_centre([(p[1], robot_h(p[2])) for p in log if a0 <= p[0] < a1], arena, rate=rate)
    calib["turn_centre_mm"] = turn_centre_calib(arena)
    if len(gaps) and gaps.max() > 0.15:
        print("Heading test: the marker was lost during the turn (turned too fast?) - turn direction left"
              " unchanged. Lower TEST_PWM and press H again.")
    elif abs(spin) < math.radians(8):
        print("Heading test: the robot did not turn - turn direction left unchanged.")
    else:
        calib["turn_sign"] = 1 if spin > 0 else -1
    arena.heading_offset, arena.turn_sign = math.radians(calib["heading_offset_deg"]), calib["turn_sign"]
    save_calib(calib)
    print(f"Heading test OK: robot front = marker top {calib['heading_offset_deg']:+.0f} deg, "
          f"moved {np.hypot(*motion):.0f} mm, turned {math.degrees(spin):+.0f} deg -> turning is "
          f"{'normal' if calib['turn_sign'] == 1 else 'REVERSED (fixed in software)'}. Saved.")


# ============================ SETUP ============================
def corners_from_sides(clicks):
    """8 clicks = 2 points on each of the 4 sides, going round -> the 4 corners (where the sides
    meet - they may be outside the picture), clockwise on screen, corner 1 -> 2 along side 1."""
    p = np.float64(clicks).reshape(4, 2, 2)

    def meet(a, b):
        (p1, q1), (p2, q2) = p[a], p[b]
        d1, d2 = (q1 - p1) / np.linalg.norm(q1 - p1), (q2 - p2) / np.linalg.norm(q2 - p2)
        A = np.column_stack([d1, -d2])
        if abs(np.linalg.det(A)) < 0.05:                   # (nearly) parallel: no corner
            return None
        return p1 + np.linalg.solve(A, p2 - p1)[0] * d1

    c = [meet(3, 0), meet(0, 1), meet(1, 2), meet(2, 3)]
    if any(x is None for x in c):
        return None
    if not OFFICIAL:
        return order_corners(c)
    # official floor: sides clicked as SOUTH, EAST, NORTH, WEST -> corners SW, SE, NE, NW.
    # Map order is NW, NE, SE, SW (north up), which must go clockwise on screen - a webcam never shows
    # the floor mirrored, so anything else means the sides were clicked in the wrong order.
    nw_ne_se_sw = np.float32([c[3], c[2], c[1], c[0]])
    if not cv2.isContourConvex(nw_ne_se_sw.reshape(-1, 1, 2)):
        return None
    area = sum(nw_ne_se_sw[i, 0] * nw_ne_se_sw[(i + 1) % 4, 1] - nw_ne_se_sw[(i + 1) % 4, 0] * nw_ne_se_sw[i, 1]
               for i in range(4))
    if area <= 0:
        print("The sides seem to be in the wrong order (east and west swapped?): click SOUTH, EAST, NORTH, WEST.")
        return None
    return nw_ne_se_sw.tolist()


def draw_arena(img, corners, W, H, s=1.0):
    """Outline of the arena and (if measured) the 4 blocks, drawn on the camera picture.
    If the calibration is right, they sit exactly on the tape."""
    corners = np.float32(corners)
    cv2.polylines(img, [np.int32(corners * s)], True, (255, 160, 0), 2)
    floor = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
    G = cv2.getPerspectiveTransform(floor, corners)
    keep, bays = layout_rects(W, H)
    docks = [_rect_to_map((x - 10, y - 10, x + 10, y + 10)) for x, y in
             (s_["dock"] for s_ in spec.STATIONS.values())] if OFFICIAL else []
    for rects, col in ((keep, (255, 0, 255)), (bays, (0, 200, 0)), (docks, (0, 140, 255))):
        for x0, y0, x1, y1 in rects:
            pts = cv2.perspectiveTransform(np.float32([[[x0, y0], [x1, y0], [x1, y1], [x0, y1]]]), G)[0]
            cv2.polylines(img, [np.int32(pts * s)], True, col, 2 if col != (0, 140, 255) else 1)


def order_corners(pts):
    """Corners -> clockwise on screen, keeping corners 1-2 on the same (width) side."""
    p = np.float32(pts)
    if not cv2.isContourConvex(p.reshape(-1, 1, 2)):
        return None
    area = sum(p[i, 0] * p[(i + 1) % 4, 1] - p[(i + 1) % 4, 0] * p[i, 1] for i in range(4))
    if area < 0:
        p = p[[1, 0, 3, 2]]
    return p.tolist()


def tape_ranges(frame, pts, corners):
    """HSV colour range(s) of the tape from the clicked spots. Far-away tape is only a few pixels
    wide, so around each click only the pixels that look least like the floor are used.
    Clicks on black tape and on coloured tape (e.g. a red border) each get their own range."""
    img = frame
    inside = np.zeros(img.shape[:2], np.uint8)
    cv2.fillConvexPoly(inside, np.int32(corners), 255)
    floor = np.median(img[inside > 0], 0)                  # the floor is most of the arena
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    floor_v = float(np.median(hsv[:, :, 2][inside > 0]))
    dark, colour = [], []
    for x, y in pts:
        sl = np.s_[max(0, y - 4):y + 5, max(0, x - 4):x + 5]
        diff = np.linalg.norm(img[sl].reshape(-1, 3).astype(float) - floor, axis=1)
        p = hsv[sl].reshape(-1, 3)[diff >= np.percentile(diff, 70)].astype(int)
        # black tape: very dark (its saturation is random). Coloured tape: clearly saturated.
        (colour if np.median(p[:, 1]) >= 90 and np.median(p[:, 2]) >= 0.25 * floor_v else dark).append(p)
    ranges = []
    if dark:
        ranges += _dark_range(np.vstack(dark), floor_v)
    if colour:
        ranges += _colour_range(np.vstack(colour))
    return ranges


def _dark_range(v, floor_v):
    """Black tape: any hue/saturation, dark. Kept well below the floor's grout lines."""
    V = v[:, 2]
    return [[[0, 0, 0], [179, 255, int(np.clip(min(np.percentile(V, 95) + 25, 0.33 * floor_v), 20, 255))]]]


def _colour_range(v):
    H, S, V = v[:, 0], v[:, 1], v[:, 2]
    clip = lambda a: int(np.clip(a, 0, 255))
    pc = np.percentile
    s_lo = clip(max(0.4 * np.median(S), pc(S, 10) - 30))   # never down to the white floor
    v_lo = clip(max(0.4 * np.median(V), pc(V, 5) - 50))
    best = None
    for shift in (0, 90):                                  # red wraps around hue 0, so try it shifted too
        h = (H + shift) % 180
        lo, hi = pc(h, 2) - 8 - shift, pc(h, 98) + 8 - shift
        if best is None or hi - lo < best[1] - best[0]:
            best = (lo, hi)
    lo, hi = best
    if lo < 0:
        lo, hi = lo + 180, hi + 180
    lo, hi = int(lo), int(math.ceil(hi))
    hues = [(lo, hi)] if hi <= 179 else [(lo, 179), (0, hi - 180)]
    return [[[h0, s_lo, v_lo], [h1, 255, 255]] for h0, h1 in hues]


def save_calib(calib):
    with open(CALIB_FILE, "w") as f:
        json.dump(calib, f, indent=2)


def screen_size():
    """Usable screen size in pixels (Windows), so the setup window fits (this PC: 1366 x 768)."""
    try:
        import ctypes
        return ctypes.windll.user32.GetSystemMetrics(0), ctypes.windll.user32.GetSystemMetrics(1)
    except Exception:
        return 1280, 720


SIDE_NAMES = ["SOUTH (y = 0: the side with DZ and P3)", "EAST (x = 300)", "NORTH (y = 300: P1, P2, S5)",
              "WEST (x = 0: S6)"]


def setup(cam, src, arena_w, arena_h):
    ui = {"scale": 1.0, "clicks": []}

    def on_mouse(ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN:
            ui["clicks"].append((x / ui["scale"], y / ui["scale"]))

    cv2.namedWindow("setup")
    cv2.setMouseCallback("setup", on_mouse)
    sw, sh = screen_size()
    side_mm = [arena_w, arena_h, arena_w, arena_h]
    has_layout = bool(layout_rects(arena_w, arena_h)[0])
    if OFFICIAL:
        help1 = ["STEP 1: click 2 points on EACH side of the arena (8 clicks), in this order:",
                 "SOUTH side, EAST, NORTH, WEST. Far apart on each line; corners may be off-screen.",
                 "Backspace = undo, Enter = next"]
    else:
        help1 = ["STEP 1: click 2 points on EACH side of the arena rectangle (8 clicks), going round it.",
                 f"Side 1 = the {arena_w:.0f} mm side. Click far apart on each line; corners may be off-screen.",
                 "Backspace = undo, Enter = next"]
    layout_note = "Enter with no clicks = use only the measured layout" if has_layout else "Enter = save"
    help2 = ["STEP 2: robot OUT of the arena. Click on the tape in 3+ places: every colour, near AND far.",
             f"Backspace = undo.  {layout_note}"]
    help0 = ["STEP 1: looking for the 4 floor markers (ids " + ", ".join(map(str, sorted(spec.FLOOR_MARKERS))) + ")",
             "Enter = use them (when all 4 are found)    C = click the arena sides instead"]
    step, corners, frame, detector, seen, n_frames = (0 if OFFICIAL else 1), None, None, make_detector(), {}, 0
    while True:
        f, _ = cam.latest()
        if f is None:
            cv2.imshow("setup", waiting_screen(cam.status))
            if cv2.waitKey(100) & 0xFF in (27, ord('q')):
                return False
            continue
        frame = f
        ui["scale"] = min(1.0, (sw - 40) / frame.shape[1], (sh - 120) / frame.shape[0])
        view = frame.copy()
        if step == 0:                                      # automatic: floor markers
            found = floor_marker_centres(frame, detector)
            n_frames += 1
            for i, c in found.items():
                seen.setdefault(i, []).append(c)
                seen[i] = seen[i][-30:]
                cv2.circle(view, (int(c[0]), int(c[1])), 12, (0, 255, 0), 3)
                cv2.putText(view, f"{i} {spec.FLOOR_MARKERS[i]}", (int(c[0]) + 14, int(c[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if len(seen) == len(spec.FLOOR_MARKERS):
                draw_arena(view, corners_from_floor_markers({i: np.median(v, 0) for i, v in seen.items()}),
                           ARENA_MM, ARENA_MM)
        elif step == 1:
            pts = ui["clicks"]
            for i, p in enumerate(pts):
                cv2.circle(view, (int(p[0]), int(p[1])), 6, (255, 160, 0), -1)
                if i % 2 == 1:                             # side done: draw its whole line + its name/length
                    a, b = np.float64(pts[i - 1]), np.float64(p)
                    d = (b - a) / max(np.linalg.norm(b - a), 1e-6) * 5000
                    cv2.line(view, tuple(np.int32(a - d)), tuple(np.int32(a + d)), (255, 160, 0), 1)
                    mid = (a + b) / 2
                    name = SIDE_NAMES[i // 2].split(" ")[0] if OFFICIAL else f"side {i // 2 + 1}: {side_mm[i // 2]:.0f} mm"
                    cv2.putText(view, name, (int(mid[0]), int(mid[1]) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        else:
            draw_arena(view, corners, arena_w, arena_h)
            for p in ui["clicks"]:
                cv2.circle(view, (int(p[0]), int(p[1])), 7, (0, 255, 0), 2)
        view = cv2.resize(view, None, fx=ui["scale"], fy=ui["scale"])
        if step == 1 and OFFICIAL and len(ui["clicks"]) < 8:
            label(view, help1 + [f"next: {SIDE_NAMES[len(ui['clicks']) // 2]}"])
        else:
            label(view, help0 + [f"found: {sorted(seen)}"] if step == 0 else help1 if step == 1 else help2)
        cv2.imshow("setup", view)
        k = cv2.waitKey(30) & 0xFF
        if k in (27, ord('q')):
            return False
        if step == 0:
            if k in (ord('c'), ord('C')):
                step = 1
                ui["clicks"].clear()                       # clicks made so far (e.g. to focus the window) mean nothing
            elif k in (10, 13):
                if len(seen) == len(spec.FLOOR_MARKERS) and min(len(v) for v in seen.values()) >= 5:
                    corners = corners_from_floor_markers({i: np.median(v, 0) for i, v in seen.items()})
                    ui["clicks"].clear()
                    break
                print("Not all floor markers found yet (each must be seen in 5+ frames). Missing:",
                      sorted(set(spec.FLOOR_MARKERS) - set(seen)), " - or press C to click the sides.")
            continue
        if k == 8 and ui["clicks"]:
            ui["clicks"].pop()
        if step == 1 and len(ui["clicks"]) > 8:
            del ui["clicks"][8:]
        if k in (10, 13):
            if step == 1:
                if len(ui["clicks"]) != 8:
                    print("Click 8 points first: 2 on each side.")
                    continue
                corners = corners_from_sides(ui["clicks"])
                if corners is None:
                    print("Those lines don't make a proper 4-sided shape - click 2 points on each side, in order.")
                    ui["clicks"].clear()
                    continue
                ui["clicks"] = []
                if OFFICIAL and not USE_LIVE_TAPE:         # exact map: no tape colours needed
                    break
                step = 2
            elif ui["clicks"] or has_layout:
                break
            else:
                print("Click on the tape first (or set RING_MM / GAP_MM to use the measured layout).")

    h, w = frame.shape[:2]
    samples = [(int(x), int(y)) for x, y in ui["clicks"]]
    ranges = tape_ranges(frame, samples, corners) if samples else []
    focal, cam_mm = camera_pose(corners, arena_w, arena_h, (w, h))
    calib = dict(camera=src, frame_size=[w, h], arena_w=arena_w, arena_h=arena_h, corners_px=corners,
                 tape_ranges=ranges, focal_px=round(focal, 1), camera_mm=[round(c, 1) for c in cam_mm],
                 heading_offset_deg=None, turn_sign=1, official=OFFICIAL,
                 calibrated_by="floor markers" if step == 0 else "clicks")
    arena = Arena(calib, load_map=False)

    acc, n, last, t_start = np.zeros((arena.size[1], arena.size[0]), np.float32), 0, 0.0, time.time()
    while ranges and n < 15 and time.time() - t_start < 5:  # tape map = tape in most of 15 frames
        f, t = cam.latest()
        if t != last:
            last = t
            acc += arena.look(f)[1] > 0
            n += 1
        cv2.waitKey(10)
    tape = np.where(acc / max(n, 1) > 0.5, 255, 0).astype(np.uint8)
    cv2.imwrite(MAP_FILE, tape)
    save_calib(calib)

    hfov = math.degrees(2 * math.atan(w / (2 * focal)))
    dist = math.hypot(cam_mm[0] - arena_w / 2, cam_mm[1] - arena_h / 2)
    msg = [f"Saved {os.path.basename(CALIB_FILE)}" + (f" + {os.path.basename(MAP_FILE)}" if ranges else ""),
           f"Calibrated from the {calib['calibrated_by']}" + (" (official floor plan)" if OFFICIAL else ""),
           f"Webcam: about {cam_mm[2] / 1000:.2f} m above the floor, {dist / 1000:.2f} m from the arena centre,"
           f" field of view about {hfov:.0f} deg"
           + (" (from CAM_HEIGHT_MM)" if CAM_HEIGHT_MM else " (from CAM_HFOV_DEG)" if CAM_HFOV_DEG else ""),
           "  (if that is far off: set CAM_HEIGHT_MM to the measured webcam height and run --setup again)"
           + ("" if 50 <= hfov <= 100 or CAM_HFOV_DEG else
              "\n  <- UNUSUAL field of view (webcams are ~60-90 deg): were the sides clicked exactly on the"
              " lines" + (", and are --arena-w / --arena-h the right way round?" if not OFFICIAL else "?"))]
    if ranges:
        msg += [f"Tape colour HSV ranges: {ranges}",
                f"Tape found on {100 * np.count_nonzero(tape) / tape.size:.1f} % of the arena"
                + ("   <- TOO MUCH: the floor matches the tape colour. Run --setup again and click"
                   " right on the tape" if np.count_nonzero(tape) > 0.25 * tape.size else "")]
    print("\n".join(msg))
    top = arena.look(frame)[0]
    top[tape > 0] = (0, 0, 255)
    top[(arena.saved > 0) & (tape == 0)] //= 2                # measured blocks (if set): darker
    sc = VIEW_H / top.shape[0]
    top = cv2.resize(top, None, fx=sc, fy=sc, interpolation=cv2.INTER_NEAREST)
    label(top, ["Top-down map" + (". Red = tape seen" if ranges else "") +
                (", dark = keep-out blocks" if arena.blocks else ""),
                "Any key = done.", "Next: robot in a lane, run without --setup, press H."])
    s = VIEW_H / frame.shape[0]
    cam_view = cv2.resize(frame, (int(frame.shape[1] * s), VIEW_H))
    draw_arena(cam_view, corners, arena_w, arena_h, s)
    label(cam_view, ["The coloured lines must sit ON the tape" if arena.blocks else "Blue line = arena edge"])
    cv2.imshow("setup", np.hstack([cam_view, top]))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return True


# ============================ DRIVING ============================
# ---- wheel encoders (4 hall encoders, read by the firmware; streamed with "tele on", ~10 per second)
ENC_SIGN = (-1, 1, -1, 1)     # FL FR RL RR: the left wheels count backwards when driving forward (checked)
ENC_COUNTS_PER_MM = 13.2      # measured 2026-10-08 against the camera (13.15 forward, 13.46 back)
ENC_RAD_PER_MM = 1 / 170.0    # rotation per mm of (right - left) wheel travel; learnt from turns (skid steer:
                              # the wheels slip sideways in a turn, so this is less than 1 / wheel spacing)
ENC_GATE_DEG = 20             # a camera heading this far from what the wheels say it turned = a bad look
ENC_CHECK_DEG = 15            # ... asked only when the camera heading jumps more than this since the last good
                              # look (the wheels slip on the tiles, so they're a referee, not the main sensor)
STALL_S = 1.5                 # motors driven this long but the wheels don't turn -> stop (battery? stuck?)
CAM_STALL_S = 2.5             # ... or the camera sees it neither move 1 cm nor turn 4 deg in this long
ENC_DEAD_COUNTS = 1500        # same-side partner counted this much (~11 cm) and this wheel < 5 % of it = dead


class Car:                                  # 8-byte UDP drive packet: 'DR', left, right, sequence
    def __init__(self, host, port):
        self.addr = (socket.gethostbyname(host), port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.odo = []                       # (time, left mm, right mm, all 4 encoders ok, |speed| counts/s)
        self.servo, self.servo_cmd = None, None   # arm servo angle the robot reports / last one we asked for
        self.st = {}                        # the robot's latest status (telemetry), e.g. the claw's saved points
        self.replies, self.running = [], False
        self.enc_last, self.enc_abs, self.enc_dead = None, [0] * 4, [False] * 4   # dead-encoder check (_rx)

    def send(self, left, right):
        self.seq = (self.seq + 1) & 0xFFFF
        self.sock.sendto(b"DR" + struct.pack("<hhH", int(left), int(right), self.seq), self.addr)

    def text(self, cmd):
        self.sock.sendto(cmd.encode(), self.addr)

    def start_telemetry(self):
        """Wheel encoders: switch on the encoder stream. The firmware's straight-drive assist stays OFF:
        on this robot it made two diagonal wheels spin and slip (owner, 2026-10-08: FL/RR counted 4x less
        than RL/FR after a run with it on), so it drove worse, not straighter."""
        self.running = True
        threading.Thread(target=self._rx, daemon=True).start()
        for cmd in ("assist off", "tele on"):
            self.text(cmd)

    def _rx(self):
        self.sock.settimeout(0.3)
        while self.running:
            try:
                d = self.sock.recvfrom(2048)[0].decode(errors="replace")
            except (socket.timeout, ConnectionResetError, OSError):
                continue
            if not d.startswith("{"):
                self.replies.append(d)
                continue
            try:
                j = json.loads(d)
            except ValueError:
                continue
            mm = [ENC_SIGN[i] * j["cnt"][i] / ENC_COUNTS_PER_MM for i in range(4)]
            ok = [bool(v) for v in j.get("ok", [1, 1, 1, 1])]
            # a dead encoder (2026-10-10: car2's front-left never counts) reads 0 while its same-side partner
            # turns; the firmware only flags it with straight assist on, so check here and leave it out
            if self.enc_last is not None:
                for i in range(4):
                    self.enc_abs[i] += abs(j["cnt"][i] - self.enc_last[i])
                for i in range(4):
                    if (not self.enc_dead[i] and self.enc_abs[i ^ 2] > ENC_DEAD_COUNTS
                            and self.enc_abs[i] < 0.05 * self.enc_abs[i ^ 2]):
                        self.enc_dead[i] = True
                        print(f"Wheel encoder {('FL', 'FR', 'RL', 'RR')[i]} is not counting - left out "
                              f"(check its wiring)")
            self.enc_last = list(j["cnt"])
            ok = [o and not dead for o, dead in zip(ok, self.enc_dead)]
            side = lambda a, b: (mm[a] + mm[b]) / 2 if ok[a] and ok[b] else (mm[a] if ok[a] else mm[b])
            self.odo.append((time.time(), side(0, 2), side(1, 3), all(ok), max(abs(r) for r in j["rate"])))
            if "servo" in j:
                self.servo = j["servo"]
            self.st = j
            if len(self.odo) > 600:
                del self.odo[:100]

    def fresh(self, max_age=0.4):
        return bool(self.odo) and time.time() - self.odo[-1][0] < max_age

    def wheels_at(self, t):
        """(left mm, right mm) at time t, interpolated from the encoder stream (None if not covered)."""
        o = self.odo
        if len(o) < 2 or t < o[0][0] - 0.05:
            return None
        for a, b in zip(reversed(o[:-1]), reversed(o[1:])):
            if a[0] <= t:
                f = 0.0 if b[0] == a[0] else min(1.0, (t - a[0]) / (b[0] - a[0]))
                return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])
        return o[0][1], o[0][2]

    def turned(self, t0, t1):
        """How far the robot turned between t0 and t1 by its wheels: (right - left) travel in mm
        (x ENC_RAD_PER_MM = radians, counter-clockwise +), or None."""
        a, b = self.wheels_at(t0), self.wheels_at(t1)
        return None if a is None or b is None else (b[1] - b[0]) - (a[1] - a[0])

    def arm(self, deg):
        """Move the pickup-arm servo (firmware: 1 deg per 8 ms)."""
        self.servo_cmd = int(round(deg))
        self.text(f"servo {self.servo_cmd}")

    def claw(self, what, deg):
        """A claw move done by the firmware ('claw drop' / 'claw pick'); deg = the angle it will end at."""
        self.servo_cmd = int(round(deg))
        self.text("claw " + what)

    def close(self):
        self.running = False
        try:
            self.text("tele off")
        except OSError:
            pass


KEYS_HELP = (["1-6 go to S1-S6  D dispatch bay  7/8/9 park in P1/P2/P3" if spec.CUTOUTS else
              "1-6 go to S1-S6  D / 7/8/9 = stop in FRONT of DZ / P1-P3 (no cut-outs)",
              "SPACE pause/go  K pick  U drop  R roam  +/- speed  H heading test  Q quit"]
             if OFFICIAL else ["SPACE start/stop  +/- speed  H heading test  Q quit"])


def draw(frame, top, tape, arena, tag_px, pose, plan, state, base, cmd, fps, running, route=None, info=None,
         warn=None):
    s = VIEW_H / frame.shape[0]
    cam = cv2.resize(frame, (int(frame.shape[1] * s), VIEW_H))
    draw_arena(cam, arena.corners, arena.W, arena.H, s)
    if tag_px is not None:
        cv2.polylines(cam, [np.int32(tag_px * s)], True, (0, 255, 255), 2)
    if pose is not None:                                   # robot outline on the floor: should sit under the robot
        foot = cv2.perspectiveTransform(footprint(*pose).reshape(1, 4, 2).astype(np.float32), arena.floor2img)[0]
        cv2.polylines(cam, [np.int32(foot * s)], True, (0, 255, 0), 2)
    label(cam, [f"{state}  base {base}  L {cmd[0]} R {cmd[1]}  {fps:.0f} fps"] + (info or []) + KEYS_HELP)
    if warn:
        cv2.putText(cam, warn, (8, VIEW_H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    if not running:
        cv2.rectangle(cam, (0, 0), (cam.shape[1] - 1, cam.shape[0] - 1), (0, 0, 255), 3)

    m = (top * 0.45).astype(np.uint8)
    m[tape > 0] = (0, 0, 255)
    sc = VIEW_H / m.shape[0]
    m = cv2.resize(m, None, fx=sc, fy=sc, interpolation=cv2.INTER_NEAREST)
    k = sc / MM_PER_PX                                     # mm -> window pixels
    if route:                                              # planned route (official cm) on the map
        pts = np.int32([np.array(to_map(*p)) * k for p in route])
        cv2.polylines(m, [pts], False, (255, 120, 0), 2)
        cv2.circle(m, tuple(pts[-1]), 6, (255, 120, 0), -1)
    if pose is not None:
        pos, hd = pose
        p0 = (int(pos[0] * k), int(pos[1] * k))
        if plan is not None:
            for i, (a, d) in enumerate(zip(plan["dirs"], plan["free_each"])):
                end = (int((pos[0] + d * math.cos(a)) * k), int((pos[1] + d * math.sin(a)) * k))
                cv2.line(m, p0, end, (0, 255, 0) if i == plan["best"] else (90, 90, 90), 3 if i == plan["best"] else 1)
        cv2.polylines(m, [np.int32(footprint(pos, hd) * k)], True, (0, 255, 255), 2)
        tip = (int((pos[0] + 90 * math.cos(hd)) * k), int((pos[1] + 90 * math.sin(hd)) * k))
        cv2.arrowedLine(m, p0, tip, (0, 255, 255), 2, tipLength=0.3)
    label(m, ["top-down map" + ("" if arena.heading_offset is not None else "   - press H first -")])
    return np.hstack([cam, m])


class Recorder:
    """--record: saves what happened so a run can be reviewed afterwards.
    laptop/runs/arena-<date-time>/  log.csv (every frame), frames/*.jpg (3 per second + every
    state change), summary.txt"""
    def __init__(self):
        self.dir = os.path.join(HERE, "runs", "arena-" + time.strftime("%Y%m%d-%H%M%S")
                                + (f"-car{ROBOT_NUM}" if ROBOT_NUM else ""))
        os.makedirs(os.path.join(self.dir, "frames"))
        self.csv = open(os.path.join(self.dir, "log.csv"), "w")
        self.csv.write("t,fps,state,base,x_mm,y_mm,heading_deg,target_deg,free_mm,clear_mm,L,R,"
                       "x_cm,y_cm,heading_official_deg,goal,keepout,mark_x_mm,mark_y_mm,margin\n")
        self.t0, self.last_img, self.last_state = time.time(), -1.0, None
        self.rows, self.lost, self.stuck, self.fps, self.clears, self.targets = 0, 0, 0, [], [], []
        self.keepout_frames, self.arrivals, self.margin_frames = 0, [], 0
        print("Recording to", self.dir)

    def frame(self, view, state, base, fps, pose, plan, cmd, running, official=None, goal=None, keepout=False,
              mark=None, margin=False):
        t = time.time() - self.t0
        x = y = hd = tg = fr = cl = ox = oy = oh = mx = my = ""
        if mark is not None:                               # the marker itself (x/y = the robot's turning point)
            mx, my = f"{mark[0]:.0f}", f"{mark[1]:.0f}"
        if pose is not None:
            x, y, hd = f"{pose[0][0]:.0f}", f"{pose[0][1]:.0f}", f"{math.degrees(pose[1]):.1f}"
        if official is not None:
            ox, oy, oh = f"{official[0]:.1f}", f"{official[1]:.1f}", f"{official[2]:.1f}"
        if plan is not None:
            tg, fr, cl = f"{math.degrees(plan['delta']):.0f}", f"{plan['free']:.0f}", f"{plan['clear_here']:.0f}"
        self.csv.write(f"{t:.3f},{fps:.1f},{state},{base},{x},{y},{hd},{tg},{fr},{cl},{cmd[0]},{cmd[1]},"
                       f"{ox},{oy},{oh},{goal or ''},{int(keepout)},{mx},{my},{int(margin)}\n")
        if state == "ARRIVED" and self.last_state != "ARRIVED" and official is not None and goal in spec.NODES:
            gx, gy = spec.NODES[goal]["xy"]
            self.arrivals.append((goal, math.hypot(official[0] - gx, official[1] - gy)))
        if running:
            self.rows += 1
            if self.rows > 15:                             # skip the warm-up frames
                self.fps.append(fps)
            if pose is None:
                self.lost += 1
            self.keepout_frames += keepout
            self.margin_frames += margin
            if state == "STUCK" and self.last_state != "STUCK":
                self.stuck += 1
            if plan is not None:
                self.clears.append(plan["clear_here"])
                self.targets.append(abs(math.degrees(plan["delta"])))
        if state != self.last_state or t - self.last_img >= 0.33:
            cv2.imwrite(os.path.join(self.dir, "frames", f"{t:07.2f}_{state.replace(' ', '_')}.jpg"), view,
                        [cv2.IMWRITE_JPEG_QUALITY, 70])
            self.last_img = t
        self.last_state = state

    def close(self):
        avg = lambda v: sum(v) / len(v) if v else 0
        lines = [
            f"run folder:            {self.dir}",
            f"duration:              {time.time() - self.t0:.1f} s   (frames while running: {self.rows})",
            f"camera fps:            avg {avg(self.fps):.1f}, min {min(self.fps, default=0):.1f}",
            f"robot not seen:        {100 * self.lost / max(self.rows, 1):.1f} % of frames",
        ]
        if self.clears:                                    # roam mode was used
            lines += [
                f"stuck (turned on spot): {self.stuck} times",
                f"room to nearest tape:  avg {avg(self.clears):.0f} mm, min {min(self.clears, default=0):.0f} mm"
                f"   (robot half-width {ROBOT_WID_MM / 2:.0f} mm)",
                f"way to go off the nose: avg {avg(self.targets):.0f} deg   (roam mode)"]
        lines += [
            f"over the real tape:    {self.keepout_frames} frames ({100 * self.keepout_frames / max(self.rows, 1):.1f} %)",
            f"inside the margin:     {self.margin_frames} frames ({100 * self.margin_frames / max(self.rows, 1):.1f} %)"
            f"   (closer than {spec.SAFETY_CM:.1f} cm to the tape)",
            f"arrivals:              " + (", ".join(f"{g} {d:.1f} cm off" for g, d in self.arrivals) or "none"),
            f"route settings:        lane_follow BASE_PWM {lane_follow.BASE_PWM} K_HEAD {lane_follow.K_HEAD} "
            f"K_RATE {lane_follow.K_RATE} K_CROSS {lane_follow.K_CROSS} LATENCY_S {lane_follow.LATENCY_S}",
            f"roam settings:         BASE_SPEED {BASE_SPEED} KP {KP} KD {KD} PIVOT_DEG {PIVOT_DEG} "
            f"NEED_MM {NEED_MM:.0f} LOOK_MM {LOOK_MM} TURN_COST {TURN_COST}",
        ]
        self.csv.close()
        with open(os.path.join(self.dir, "summary.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print("\n".join(lines))


TURN_LEARN = 0.3              # each turn seen moves the robot's turning point this far toward what it showed
TURN_RELOCK_N = 3             # this many turns in a row agreeing (within 15 mm) but far from the known value = the
                              # known value is wrong (2026-10-10: a poor first fit locked in for a whole run)


def mean_turn_centre(centres):
    """The turning point used for the position: the average of the left- and right-turn values that are
    known (zero = not learnt yet, e.g. just after a new mark)."""
    known = [c for c in centres.values() if np.any(c)]
    return np.mean(known, axis=0) if known else np.zeros(2)


def turn_direction(samples):
    """'left' (counter-clockwise seen from above) or 'right'. (Map y points down: left = angle falls.)"""
    a = np.unwrap([p[1] for p in samples])
    return "left" if a[-1] < a[0] else "right"


def learn_turn_centre(samples, arena, rate=TURN_LEARN):
    """After an on-the-spot turn: refine where the robot turns about. samples = (marker xy map mm,
    robot heading rad) through the turn. Left and right turns are kept apart (one wheel pushing harder
    makes the robot slide a little while it turns, differently each way); the position uses their
    average, which is the point between the wheels. Returns True if it learnt something."""
    fit = fit_turn_centre([samples])
    if fit is None:                                        # (too small a turn to tell)
        return False
    (f, l), err = fit
    if err > 5 or math.hypot(f, l) > 80:
        print(f"  (turn: turning point unclear, fit {err:.1f} mm - ignored)")
        return False
    d = turn_direction(samples)
    if not hasattr(arena, "turn_fits"):                    # (test stand-ins of Arena)
        arena.turn_fits = {}
    recent = arena.turn_fits.setdefault(d, [])             # this direction's last few fits (this run)
    recent.append(np.array([f, l]))
    del recent[:-TURN_RELOCK_N]
    if not np.any(arena.turn_centres[d]):                  # nothing known yet (new mark): take it as it is
        rate = 1.0
    if rate < 1 and np.any(arena.turn_centres[d]) and np.hypot(*(np.array([f, l]) - arena.turn_centres[d])) > 15:
        # far from the known one: one bad look (far off the camera, e.g. the NW corner) - unless the last few
        # turns all agree with each other: then the KNOWN value was the bad one (e.g. a poor first fit) - take them
        if len(recent) == TURN_RELOCK_N and max(np.hypot(*(p - q)) for p in recent for q in recent) <= 15:
            arena.turn_centres[d] = np.median(recent, axis=0)
            arena.turn_centre = mean_turn_centre(arena.turn_centres)
            arena.turn_confirmed = True
            print(f"  ({d} turn: the last {TURN_RELOCK_N} turns agree on {arena.turn_centres[d][0]:+.0f} / "
                  f"{arena.turn_centres[d][1]:+.0f} mm, not the known value - re-learnt)")
            return True
        print(f"  ({d} turn: turning point {f:+.0f} / {l:+.0f} mm is far from the known one - a bad look, ignored)")
        return False
    arena.turn_centres[d] = (1 - rate) * arena.turn_centres[d] + rate * np.array([f, l])
    arena.turn_confirmed = True                            # (the turning point is right: slides can be told apart)
    arena.turn_centre = mean_turn_centre(arena.turn_centres)
    print(f"  {d} turn: it turned about {f:+.0f} mm fwd {l:+.0f} mm left of the marker (fit {err:.1f} mm)"
          f" -> using {arena.turn_centre[0]:+.0f} / {arena.turn_centre[1]:+.0f} mm")
    return True


def learn_enc_scale(times, headings, car, arena):
    """After a turn the camera saw: radians turned / (right - left) wheel travel -> the encoder rotation
    scale (skid steer slips, so it is learnt, not computed)."""
    if len(times) < 5 or car is None:
        return
    dth = float(np.unwrap(headings)[-1] - np.unwrap(headings)[0])   # map frame (counter-clockwise = -)
    tr = car.turned(times[0], times[-1])
    if tr is None or abs(dth) < math.radians(30) or abs(tr) < 20:
        return
    k = -dth / tr
    if not 1 / 400 < k < 1 / 60:
        print(f"  (encoders: turn ratio {1 / k if k else 0:.0f} mm/rad looks wrong - ignored)")
        return
    rate = 0.5 if arena.enc_learnt < 3 else 0.2
    arena.enc_rad_per_mm = (1 - rate) * arena.enc_rad_per_mm + rate * k
    arena.enc_learnt += 1
    print(f"  encoders: turned {math.degrees(abs(dth)):.0f} deg for {abs(tr):.0f} mm of wheel difference -> "
          f"{1 / arena.enc_rad_per_mm:.0f} mm per radian")


def turn_centre_calib(arena):
    return {d: [round(float(v), 1) for v in c] for d, c in arena.turn_centres.items()}


TRIM_DIRS = ("E", "N", "W", "S")   # heading trim: one per lane direction (official 0, 90, 180, 270 deg)
TRIM_LEARN = 0.5              # each straight stretch moves the trim this far toward what it showed
TRIM_STRETCH_MM = 60          # compare the way it moved with its heading over this much straight driving
                              # (short + fast: the triangle's angle error also changes with WHERE the robot is,
                              # by 5-10 deg - measured 2026-10-08 - so it must keep up as the robot drives)
TRIM_STRAIGHT_DEG = 8         # ... only while its heading changed less than this over the stretch


def heading_trim(arena, h):
    """Correction (rad, add to the map-frame heading h) for the triangle's angle error: seen at a slant
    and a bit blurred, its angle reads a few degrees off, by an amount that depends on which way it
    faces (measured: ~6 deg facing east, ~0 facing north). Learnt while driving, interpolated between
    the four lane directions."""
    o = (-math.degrees(h)) % 360                          # official heading (CCW from east)
    i, frac = int(o // 90) % 4, (o % 90) / 90
    return -math.radians((1 - frac) * arena.trims[i] + frac * arena.trims[(i + 1) % 4])


def learn_heading_trim(log, arena, rate=TRIM_LEARN):
    """log = [(robot xy map mm, heading map rad (already trimmed), reversing?)] while driving straight.
    A wheeled robot moves the way it points (or straight back), so once it has gone TRIM_STRETCH_MM the
    direction it moved is its true heading: nudge the trim toward it. Returns the log to carry on with."""
    (x0, y0), _, rev = log[0]
    (x1, y1), _, _ = log[-1]
    if any(p[2] != rev for p in log):                      # changed between forward and back: start again
        return log[-1:]
    if math.hypot(x1 - x0, y1 - y0) < TRIM_STRETCH_MM:
        return log
    hs = [p[1] for p in log]
    if np.ptp(np.unwrap(hs)) > math.radians(TRIM_STRAIGHT_DEG):   # it was turning, not driving straight
        return log[-1:]
    moved = math.atan2(y1 - y0, x1 - x0) + (math.pi if rev else 0)
    err = -math.degrees(wrap(moved - circ_mean(hs)))       # official degrees: + = really more counter-clockwise
    if abs(err) > 20:                                      # (a bad look, not a trim)
        return log[-1:]
    o = (-math.degrees(circ_mean(hs))) % 360
    i, frac = int(o // 90) % 4, (o % 90) / 90
    for k, w in ((i, 1 - frac), ((i + 1) % 4, frac)):
        arena.trims[k] += rate * w * err
    if abs(err) > 1.5:
        print(f"  heading trim: facing {o:.0f} deg it moved {err:+.1f} deg off its heading -> trim " +
              " ".join(f"{d} {v:+.1f}" for d, v in zip(TRIM_DIRS, arena.trims)))
    return log[-1:]


WRONG_LOCK_DEG = 50           # moving this far off the heading read from the mark = it took the wrong axis


def motion_heading_error(log):
    """(direction it moved - heading it says) in rad over the last TRIM_STRETCH_MM of straight driving,
    or None (not far enough yet / changed between forward and back). Same data as learn_heading_trim."""
    (x0, y0), _, rev = log[0]
    (x1, y1), _, _ = log[-1]
    if any(p[2] != rev for p in log) or math.hypot(x1 - x0, y1 - y0) < TRIM_STRETCH_MM:
        return None
    moved = math.atan2(y1 - y0, x1 - x0) + (math.pi if rev else 0)
    return wrap(moved - circ_mean([p[1] for p in log[-5:]]))


def learnt_calib(arena):
    """What the run loop learns about this robot, as it is saved in arena_calib.json."""
    return {"turn_centre_mm": turn_centre_calib(arena),
            "heading_trim_deg": {d: round(v, 1) for d, v in zip(TRIM_DIRS, arena.trims)},
            "enc_rad_per_mm": round(arena.enc_rad_per_mm, 6), "enc_turns_learnt": arena.enc_learnt}


class CameraCheck(threading.Thread):
    """Finds the red boundary (or else the 4 floor anchors) in one frame in the background (~0.15 s; the run loop must keep
    sending commands meanwhile, or the robot's 300 ms fail-safe stops it), and how well the current map
    and the new corners fit the black block tape in that frame. Results: .corners (or None), .fit_now,
    .fit_new (mm, see tape_fit_mm)."""
    def __init__(self, frame, corners_now):
        super().__init__(daemon=True)
        self.frame, self.map_corners = frame, np.array(corners_now, float)
        self.corners, self.fit_now, self.fit_new = None, 999.0, 999.0

    def run(self):
        self.corners, _ = red_boundary_corners(self.frame)
        if self.corners is None:                           # no red boundary (the venue's is black): the anchors
            anchors = anchors_in(self.frame, make_detector())
            if len(anchors) == len(spec.FLOOR_MARKERS):
                self.corners = corners_from_anchors(anchors)[0]
        if self.corners is not None:
            self.fit_now = tape_fit_mm(self.frame, self.map_corners)[0]
            self.fit_new = tape_fit_mm(self.frame, self.corners)[0]


JUMP_MM, JUMP_DEG = 30, 45    # a robot pose jump bigger than this (+ what it can move meanwhile) = bad look
MAX_SPEED_MMS, MAX_TURN_RS = 300, 3.0
JUMP_FRAMES = 5               # ... but after this many in a row, believe it (someone moved the robot)
ACQUIRE_FRAMES = 3            # finding the robot afresh: this many frames in a row must agree on its direction


# ============================ PICKUP ARM ============================
# cad/pickup_bot_v2.scad: a side scissor arm (lazy tongs, 4 cells) on ONE MG90S servo (signal D2, 5 V from the
# LM2596), with a one-way hinged flap at its tip, on the robot's PICK side (lane_follow.PICK_SIDE).
#   PICK : reach  -> the flap rides over the cube and drops behind it
#          carry  -> retracting, the flap hooks the cube and drags it against the side panel (held there)
#   DROP : release -> the arm opens ~15 mm, the flap lets go; the robot reverses out, the cube stays.
#          Once the robot is out of the bay the arm stows.
# The flap / side panel are centred along the robot (y = 65 of 130 mm = the wheel midpoint = its turning
# point), so stopping on a docking square lines the arm up with the alcove.
ARM_DEFAULT = {"stow": 20, "carry": 43, "release": 50, "reach": 74}
# ^ servo degrees, only a guess from the CAD (first link: stowed 77, carrying 54, full reach 23 deg = a 54 deg
#   swing) for a horn fitted so that 20 = stowed and a higher angle = further out. The arm won't move until
#   they are calibrated on the real robot: python tools/arm_calibrate.py  (sets arm_deg + arm_calibrated in
#   arena_calib.json). Never guess them: past full reach the scissor jams and the stalled servo can brown out
#   the ESP32.
ARM_DEG_PER_S = 125           # how fast the firmware moves the servo (1 deg / 8 ms)
PICK_STEPS = [("reach", 0.4), ("carry", 0.6)]   # (pose, then wait this long: flap drops / cube slides in)
DROP_STEPS = [("release", 0.4)]
CUBE_MM = 50                  # the package cube (handout: 4-5 cm); carried against the side panel


# ---- CLAW (2026-10-10): the owner replaced the scissor arm with a gripping claw - two geared arms on a TowerPro SG90
# (signal D2). calib "claw": "front" (car1: it sticks out of the robot's FRONT). The robot firmware does the moves:
#   "claw drop" opens it to its saved open angle, "claw pick" closes it to the saved touch point + a gentle squeeze
#   (both saved once on the robot's web page: Save open / Save touch; status fields open / touch / squeeze).
# At a station the robot arrives with its camera side (lane_follow.PICK_SIDE) toward the alcove for the QR look,
# then FACE (key F) turns it on the spot to face the alcove, PICK grabs the cube, and it is carried in FRONT
# (no sideways carry offset). DROP opens the claw in the bay; once out of the bay the claw closes to its touch
# angle again (folded in for driving - wide open it is wider than a lane).
CLAW_DEG_PER_S = 45           # the firmware's claw speed (22 ms / deg)


def claw_points(car):
    """(open, pick, touch) servo angles from the robot's status, or None if the claw isn't set up."""
    st = car.st or {}
    o, t, sq = st.get("open", -1), st.get("touch", -1), st.get("squeeze", 4)
    if o is None or t is None or o < 0 or t < 0 or o == t:
        return None
    return o, min(170, max(10, t + (1 if t > o else -1) * sq)), t


def arm_task(kind, calib, car=None):
    """kind 'pick' or 'drop' -> a task for arm_step(). Steps: (angle, wait after, claw command or None)."""
    if calib.get("claw") and car is not None:
        o, pick, _ = claw_points(car)
        steps = [(o, 0.3, "drop"), (pick, 0.8, "pick")] if kind == "pick" else [(o, 0.5, "drop")]
        return {"kind": kind, "steps": steps, "i": -1, "t": 0.0, "until": None, "speed": CLAW_DEG_PER_S}
    deg = calib.get("arm_deg", ARM_DEFAULT)
    steps = PICK_STEPS if kind == "pick" else DROP_STEPS
    return {"kind": kind, "steps": [(deg[name], wait, None) for name, wait in steps], "i": -1, "t": 0.0,
            "until": None, "speed": ARM_DEG_PER_S}


def arm_step(task, car, now):
    """Run an arm task: send each angle, wait until the robot reports it (or the time it needs + 0.5 s, if
    the encoder stream is off), then the step's extra wait. Returns True when done."""
    if task["i"] >= 0:
        target, wait = task["steps"][task["i"]][:2]
        if task["until"] is None:
            prev = task.get("prev", target)
            need = abs(target - prev) / task.get("speed", ARM_DEG_PER_S)
            there = car.fresh(1.0) and car.servo is not None and abs(car.servo - target) <= 1
            if there or now - task["t"] > need + 0.5:
                task["until"] = now + wait
            return False
        if now < task["until"]:
            return False
    task["i"] += 1
    if task["i"] >= len(task["steps"]):
        return True
    task["prev"] = car.servo if car.servo is not None else (car.servo_cmd or task["steps"][task["i"]][0])
    deg, _, claw_cmd = task["steps"][task["i"]]
    if claw_cmd:
        car.claw(claw_cmd, deg)
    else:
        car.arm(deg)
    task["t"], task["until"] = now, None
    return False


# ============================ QR (the robot's own ESP32-CAM) ============================
# Each robot carries an ESP32-CAM (CAD: on a mast, looking down toward the PICK side, i.e. at the alcove when the
# robot stands on a docking square). Its firmware serves http://<cam>/capture (one JPEG) and /stream.
# --auto step QR (key L): the robot stands still, a few frames are fetched and the cube's QR code decoded; every
# frame is shown in the "esp32-cam" window and the best one saved (runs/.../qr_<goal>_<cam>.jpg).
QR_FRAMES = 10                # frames tried per camera (if none reads alone, their average is tried: the robot
                              # stands still, so averaging takes the sensor noise out of a small, soft code)
QR_TIMEOUT_S = 5.0            # (a 1600x1200 JPEG over Wi-Fi takes ~0.5-1 s)
QR_URLS = ("/capture?size=uxga", "/capture?size=uxga&flash=1")   # firmware/esp32cam: full size, then with the LED
                              # (older camera firmware ignores the options and sends 320x240)


def decode_qr(img):
    """QR text in a (small, maybe soft) picture: OpenCV's QR detector(s) on the picture as it is, enlarged, and
    sharpened. Returns (text, corner points) or (None, None)."""
    try:                                                   # zxing-cpp (pip install zxing-cpp): best on tiny codes
        import zxingcpp
        for k in ((1, 2, 3) if img.shape[1] < 800 else (1,)):   # (enlarge only small pictures)
            pic = img if k == 1 else cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
            for r in zxingcpp.read_barcodes(cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)):
                if r.text:
                    p = r.position
                    return r.text, np.float32([[q.x, q.y] for q in (p.top_left, p.top_right, p.bottom_right,
                                                                     p.bottom_left)]) / k
    except ImportError:
        pass
    if hasattr(cv2, "wechat_qrcode_WeChatQRCode"):        # WeChat's detector (in this OpenCV build)
        texts, boxes = cv2.wechat_qrcode_WeChatQRCode().detectAndDecode(img)
        if texts and texts[0]:
            return texts[0], np.float32(boxes[0]).reshape(-1, 2)
    dets = [cv2.QRCodeDetector()]
    if hasattr(cv2, "QRCodeDetectorAruco"):
        dets.append(cv2.QRCodeDetectorAruco())
    big = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    sharp = cv2.addWeighted(big, 1.8, cv2.GaussianBlur(big, (0, 0), 3), -0.8, 0)
    for k, pic in (((1, img), (2, big), (2, sharp)) if img.shape[1] < 800 else ((1, img),)):
        gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
        for det in dets:
            try:
                text, pts, _ = det.detectAndDecode(gray)
            except cv2.error:
                continue
            if text:
                return text, (None if pts is None else pts.reshape(-1, 2) / k)
    return None, None


class QRLook(threading.Thread):
    """Fetch frames from the camera(s), decode the QR code. Results: .results = [(cam, text, frame, pts)]."""
    def __init__(self, cams):
        super().__init__(daemon=True)
        self.cams, self.results = cams, []

    def run(self):
        import urllib.request
        for cam in self.cams:
            best, frames = None, []
            for i in range(QR_FRAMES):
                url = QR_URLS[i % len(QR_URLS)]          # alternate: plain, flash, plain, ... (2026-10-10: car1's camera
                                                         # read only late - after 5 plain frames, ~70 s)
                try:
                    data = urllib.request.urlopen(f"http://{cam}{url}", timeout=QR_TIMEOUT_S).read()
                except Exception:
                    continue
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                frames.append(img)
                text, pts = decode_qr(img)
                best = (cam, text, img, pts) if (best is None or text) else best
                if text:
                    break
            frames = [f for f in frames if f.shape == frames[-1].shape]
            if best is not None and not best[1] and len(frames) >= 3:   # none read alone: try their average
                avg = np.mean(np.float32(frames), axis=0).astype(np.uint8)
                text, pts = decode_qr(avg)
                best = (cam, text, avg, pts)
            if best is not None:
                self.results.append(best)


def cube_corners(pos, heading):
    """Floor corners (mm) of a cube carried against the side panel (PICK side, centred along the robot)."""
    fwd = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([math.sin(heading), -math.cos(heading)])            # (map y points down)
    side = -left if lane_follow.PICK_SIDE == "right" else left
    p = np.asarray(pos, float)
    near, far, h = ROBOT_WID_MM / 2, ROBOT_WID_MM / 2 + CUBE_MM, CUBE_MM / 2
    return np.array([p + side * near + fwd * h, p + side * far + fwd * h, p + side * far - fwd * h,
                     p + side * near - fwd * h])


def keepout_depth(pose, carrying=False, safety=None):
    """How far (cm) the robot's outline (and a cube it carries) reaches over a tape line counted `safety` cm
    thick (default arena_spec.SAFETY_CM; 0 = the real tape edge). 0 = clear. Official floor only."""
    pts = list(footprint(*pose)) + (list(cube_corners(*pose)) if carrying else [])
    return max(spec.keepout_depth(*to_official(c), safety) for c in pts)


FOLLOW_HELP = {
    "OFF LANE": " - the robot is more than 6 cm off the lane line. Put it back on a lane and press a goal key.",
    "TURN FAILED": " - it could not finish a turn (kept overshooting or not turning). Check the battery/wheels,"
                   " try a lower lane_follow.PIVOT_PWM, then press a goal key again.",
    "TURN SLID": " - turning on the spot moved the robot more than 4 cm (a wheel not pushing, flat battery?)."
                 " Stopped before it slides over the lane edge. Check the wheels / battery.",
    "NO ROUTE": " - is the robot on a lane?",
}
GOAL_KEYS = {ord(str(i)): f"S{i}" for i in range(1, 7)}
GOAL_KEYS.update({ord('d'): "DZ", ord('D'): "DZ", ord('7'): "P1", ord('8'): "P2", ord('9'): "P3"})
# --auto: the same keys, pressed by a script (H = heading test, then goals), one after the other
AUTO_KEYS = {"H": ord('h'), "SYNC": ord('y'), "QR": ord('l'), "PICK": ord('k'), "DROP": ord('u'), "FACE": ord('f'),
             "OPEN": ord('o'),
             "DZ": ord('d'), "P1": ord('7'), "P2": ord('8'), "P3": ord('9'),
             **{f"S{i}": ord(str(i)) for i in range(1, 7)}}
AUTO_STEP_S = 90              # a scripted step taking longer than this = something is wrong: stop
CAMERA_CHECK_S = 3.0          # every this many s, check the red boundary is still where it was mapped (0 = off)
REMAP_PX = 4                  # boundary this far off the map in 3 readings in a row, and the black tape agrees
                              # the new corners fit better -> re-map live (the camera mount drifts: 12 px / 15 min)
CAMERA_MOVED_PX = 8           # moved more than this -> camera bumped: stop the robot (map is wrong now)
CAMERA_STOP_PX = 25           # this far off in even one reading: stop at once (then re-map or wait)
CAMERA_MOVED_N = 3            # ... seen this many times in a row, 0.4 s apart (one bad look = someone in the way)
CAMERA_WAIT_S = 20            # then wait this long for the view to come back before giving up (--auto)


def run(cam, calib, car_host, record, auto=None, qr_cams=()):
    """auto = list of steps for --auto, e.g. ["H", "S2", "DZ"]: pressed one after the other, each when
    the previous one has finished; stops at the first problem; quits when done."""
    arena = Arena(calib)
    if OFFICIAL:
        apply_tape(calib)                                  # lanes + keep-out follow the measured tape
    try:
        car = Car(car_host, CAR_PORT)
    except socket.gaierror:
        print(f"Can't find the robot at '{car_host}'. Use its IP:  --car 192.168.1.50")
        return
    print("Sending to", car.addr, " ", "  ".join(KEYS_HELP))
    car.start_telemetry()                                  # wheel encoders + the firmware's straight assist
    enc_said, stall_since, cam_stall = False, None, None
    arm, carrying, stow_pending = None, False, False       # pickup arm task / a cube on board / stow when out
    qr_job, qr_found = None, []                            # QR look (the robot's ESP32-CAM) / what was read
    claw_mode = bool(calib.get("claw"))                    # a gripping claw (see CLAW) instead of the scissor arm
    claw_folded = not claw_mode                            # (claw: closed to its touch angle once its status is in)
    arm_ok = bool(calib.get("arm_calibrated")) and not claw_mode
    if arm_ok:
        car.arm(calib["arm_deg"]["stow"])
    if arena.heading_offset is None:
        print("Put the robot in a lane with some room ahead and press H once (heading test).")
    detector, planner = make_detector(), Planner()
    follower = lane_follow.Follower(arena.turn_sign)
    mode, goal = ("route" if OFFICIAL else "roam"), None   # route = drive to a goal along the lanes
    rec = Recorder() if record else None
    running, base, test, hint, hint_heading, pose = False, BASE_SPEED, None, None, None, None
    cmd, state, last_t, last_send, last_seen, fps = (0, 0), "STOPPED", 0.0, 0.0, 0.0, 0.0
    no_shadow = np.zeros((arena.size[1], arena.size[0]), np.uint8)
    last_official, keepout_events, in_keepout = None, 0, False
    auto_t, auto_fail, auto_step = time.time() + 2.0, None, None   # (2 s for the camera to settle first)
    cam_check_t, cam_moved, cam_suspect = time.time() + 1.0, False, 0   # camera still where it was mapped?
    cam_job, cam_ok, cam_redo, cam_moved_t, cam_reads, remapped = None, 0, None, 0.0, [], False
    jumps, last_mark, acquire = 0, None, []
    turn_log, turn_spoilt, turn_t, drive_log, learnt_was = None, False, [], [], learnt_calib(arena)

    while True:
        frame, t = cam.latest()
        if frame is None:
            cv2.imshow("arena", waiting_screen(cam.status))
        new = frame is not None and t != last_t
        if new:
            if (frame.shape[1], frame.shape[0]) != arena.frame_size:
                print(f"Webcam gives {frame.shape[1]}x{frame.shape[0]} but setup was done at "
                      f"{arena.frame_size[0]}x{arena.frame_size[1]} - run --setup again.")
                break
            if last_t:
                fps = 0.9 * fps + 0.1 / max(t - last_t, 1e-3)
            last_t = t
            recent = t - last_seen < 1.0
            found = find_robot(frame, detector, arena, hint if recent else None,
                               hint_heading if t - last_seen < 0.5 else None)
            tracking = last_seen and t - last_seen < 0.5
            if found and not tracking:                     # (re)finding the robot: no heading to go on, and
                acquire = [p for p in acquire if t - p[0] < 1.0] + [(t, found[1])]   # ~1 in 8 far-off frames
                if len(acquire) < ACQUIRE_FRAMES or any(                             # takes the wrong corner
                        abs(wrap(p[1] - found[1])) > math.radians(30) for p in acquire[-ACQUIRE_FRAMES:]):
                    found = None                           # as the tip: wait for frames that agree
            elif found and jumps < JUMP_FRAMES:
                dt = t - last_seen                         # a jump the robot can't have made: a bad look
                if (np.hypot(*(found[0] - last_mark)) > JUMP_MM + MAX_SPEED_MMS * dt or      # (wrong corner
                        abs(wrap(found[1] - hint_heading)) > min(math.radians(JUMP_DEG) + MAX_TURN_RS * dt,
                                                                 math.radians(90))):   # (a corner swap is >= 108)
                    found, jumps = None, jumps + 1         # taken as the tip, a speck of colour...): skip it
                elif car.fresh() and abs(wrap(found[1] - hint_heading)) > math.radians(ENC_CHECK_DEG):
                    tr = car.turned(last_seen, t)          # a sudden heading change: did the wheels turn it?
                    if tr is not None:                     # if not, it's glare / a blurred far-off mark
                        turn = tr * arena.enc_rad_per_mm
                        allow = math.radians(ENC_GATE_DEG) + abs(turn) * (0.15 if arena.enc_learnt >= 2 else 0.4)
                        if abs(wrap(found[1] - (hint_heading - turn))) > allow:   # (map frame: CCW = angle falls)
                            found, jumps = None, jumps + 1
            # (a wrong lock is caught by how the robot MOVES - see motion_heading_error - not by the shape
            # alone: near the camera, glare makes the arrow mark pick the wrong axis in ~15 % of frames)
            if found:
                jumps, last_mark, acquire = 0, found[0], []
            pose, tag_px, plan = None, None, None
            if found:
                last_seen, tag_px, hint, hint_heading = t, found[2], found[2], found[1]
                if arena.heading_offset is not None:
                    h = found[1] + arena.heading_offset
                    h += heading_trim(arena, h)                # (the triangle reads a bit off facing some ways)
                    pose = (mark_to_centre(found[0], h, arena.turn_centre), h)
                else:
                    pose = (found[0], found[1])               # not calibrated yet: draw marker direction
            if cam_job is not None and not cam_job.is_alive():   # camera check (in the background) done:
                job, cam_job = cam_job, None
                corners_now = job.corners
                cam_check_t = time.time() + (0.4 if (cam_suspect or cam_reads) else CAMERA_CHECK_S)
                if corners_now is not None:
                    shift = max(np.hypot(*(np.array(a) - b)) for a, b in zip(arena.corners, corners_now))
                    cam_reads = (cam_reads[-2:] + [np.array(corners_now)]) if shift > REMAP_PX else []
                    just_remapped = False
                    agree = len(cam_reads) == 3 and max(np.linalg.norm(r - cam_reads[-1], axis=1).max()
                                                        for r in cam_reads) <= AUTO_MAP_AGREE_PX
                    if agree and job.fit_new <= TAPE_FIT_MAX_MM and job.fit_new < job.fit_now - 1:
                        moved_to = np.mean(cam_reads, axis=0)   # the camera really shifted (the black tape says
                        _, cam_mm = camera_pose(moved_to, arena.W, arena.H, arena.frame_size)   # so): re-map
                        arena.set_corners(moved_to, cam_mm)                                       # live
                        calib["corners_px"], calib["camera_mm"] = moved_to.tolist(), [round(c, 1) for c in cam_mm]
                        remapped, just_remapped, cam_reads, cam_suspect = True, True, [], 0
                        print(f"Camera shifted {shift:.0f} px - re-mapped the arena live (black tape fit "
                              f"{job.fit_now:.0f} -> {job.fit_new:.0f} mm).")
                    if shift <= CAMERA_MOVED_PX or just_remapped or job.fit_now <= TAPE_FIT_MAX_MM:
                        # (the map still fits the black tape: the camera hasn't moved - the red line was
                        # just partly hidden, e.g. by the robot itself on the top lane)
                        cam_suspect = 0
                        if cam_moved:        # someone was just standing on the tape (or it re-mapped): carry on
                            cam_ok += 1
                            cam_check_t = time.time() + 0.4
                            if cam_ok >= 2 or just_remapped:
                                cam_moved, cam_ok = False, 0
                                print("Camera view is right again - carrying on.")
                                if cam_redo and auto is not None:
                                    auto.insert(0, cam_redo)       # (do the interrupted step again)
                                cam_redo = None
                    else:
                        cam_ok = 0
                        if not cam_moved:    # someone standing on the boundary also gives a bad reading once:
                            cam_suspect += 1  # only believe it when it holds for CAMERA_MOVED_N looks in a row
                            cam_check_t = time.time() + 0.4
                            if cam_suspect >= CAMERA_MOVED_N or shift > CAMERA_STOP_PX:
                                cam_redo = auto_step if (running or test is not None) else None
                                cam_moved, running, test, cam_moved_t = True, False, None, time.time()
                                print(f"CAMERA MOVED? The boundary is {shift:.0f} px off the map - robot stopped. "
                                      f"Waiting up to {CAMERA_WAIT_S} s for it to settle (re-maps itself if the "
                                      "black tape confirms) or for whoever is on the tape to step off.")
            elif CAMERA_CHECK_S and OFFICIAL and cam_job is None and time.time() > cam_check_t:
                cam_job = CameraCheck(frame.copy(), arena.corners)   # camera bumped? (~0.15 s: in the background, so
                cam_job.start()                        # the robot keeps getting commands meanwhile)
            if cam_moved:
                running = False
            top, live = arena.look(frame)
            shadow = arena.shadow(*pose) if pose is not None else no_shadow
            tape = arena.saved | (live & ~shadow)
            official = to_official(*pose) if (pose is not None and OFFICIAL) else None
            if official is not None:
                last_official = official
                if stow_pending and not any(x0 - 2 < official[0] < x1 + 2 and y0 - 2 < official[1] < y1 + 2
                                            for x0, y0, x1, y1 in spec.drivable_rects()):
                    if claw_mode:                          # out of the bay: fold the arm / claw in
                        pts = claw_points(car)
                        if pts:
                            car.arm(pts[2])
                            print(f"Claw closed to {pts[2]} deg (folded in for driving).")
                    else:
                        car.arm(calib["arm_deg"]["stow"])
                    stow_pending = False
            if not claw_folded and car.st:                 # claw: start folded in (wide open it is wider than a lane)
                pts = claw_points(car)
                if pts and not carrying:
                    car.arm(pts[2])
                    print(f"Claw: open {pts[0]}, touch {pts[2]}, pick {pts[1]} deg - closed to {pts[2]} for driving.")
                elif not pts:
                    print("Claw: not set up yet (robot web page: Save open / Save touch) - PICK / DROP will refuse.")
                claw_folded = True
            follower.turn_sign = arena.turn_sign           # (the H test may have changed it)
            follower.carrying = carrying and not claw_mode   # (side carry: off-centre; the claw carries in front)
            # (a wrong turning point makes a clean turn LOOK like a slide: check only once a turn has confirmed it)
            follower.slide_limit = lane_follow.TURN_SLIDE_CM if arena.turn_confirmed else None
            if test is not None:
                cmd, state, test_next = heading_test_step(test, t, found)
                if test_next is None:
                    if test.get("kind") == "sync":
                        if not sync_result(test["log"], calib, arena, test["sign"]):
                            auto_fail = auto_fail or "SYNC failed"
                        elif (arena.new_mark or not np.any(arena.turn_centre)) and auto is not None:
                            arena.new_mark = False                 # (new mark, or turning point not known)
                            if pose is not None and room_to_turn(pose):
                                print("SYNC: room to turn here - running the heading test (H) to measure the "
                                      "new mark's turning point.")
                                auto.insert(0, "H")
                            else:
                                print("SYNC: no room to turn here - the turning point is learnt from the first "
                                      "turns while driving.")
                    else:
                        heading_test_result(test["log"], calib, arena)
                test = test_next
            elif arm is not None:                          # picking / dropping: stand still, run the arm
                cmd, state = (0, 0), {"pick": "PICKING", "open": "OPENING"}.get(arm["kind"], "DROPPING")
                if arm_step(arm, car, time.time()):
                    if arm["kind"] == "pick":
                        carrying = True
                        print("PICKED: claw closed on the cube." if claw_mode else
                              "PICKED: cube held against the side panel (arm at carry).")
                    elif arm["kind"] == "open":                # OPEN: just open the claw (near the cube)
                        print("OPEN: claw open.")
                    else:
                        carrying, stow_pending = False, True
                        print("DROPPED: " + ("claw open" if claw_mode else "arm open") +
                              " - it folds in once the robot is out of the bay.")
                    arm = None
            elif not running:
                cmd, state = (0, 0), "STOPPED"
            elif pose is None:
                if t - last_seen > LOST_S:                   # short dropouts: keep the last command
                    cmd, state = (0, 0), "NO ROBOT"
            elif mode == "route":
                l, r, state = follower.update(t, *official)
                cmd = (to_pwm(l), to_pwm(r))
                if state in lane_follow.STOPPED_STATES:
                    running = False
                    print(f"{goal}: {state}" + FOLLOW_HELP.get(state, ""))
                    if state != "ARRIVED":
                        auto_fail = auto_fail or f"{goal}: {state}"
            else:
                plan = planner.choose(arena.clearance(tape), *pose)
                (l, r), state = planner.command(plan, base, arena.turn_sign)
                cmd = (to_pwm(l), to_pwm(r))
            if running and mode == "route" and (state in ("TURN", "FINAL TURN") or
                                                (state == "SETTLE" and turn_log is not None)):
                turn_log = turn_log if turn_log is not None else []   # watch each turn (+ its coasting)
                if found:
                    turn_log.append((found[0], pose[1]))
                    turn_t.append(t)
                else:
                    turn_spoilt = True                     # (lost the robot during it: don't learn from it)
            elif turn_log is not None:
                if not turn_spoilt:
                    learn_turn_centre(turn_log, arena)
                    learn_enc_scale(turn_t, [p[1] for p in turn_log], car, arena)
                turn_log, turn_spoilt, turn_t = None, False, []
            if running and mode == "route" and state == "DRIVE" and cmd[0] * cmd[1] > 0:
                if pose is not None:                       # driving straight: the way it moves shows its
                    drive_log = drive_log[-90:] + [(pose[0], pose[1], cmd[0] < 0)]
                    err = motion_heading_error(drive_log)
                    if err is not None and abs(err) > math.radians(WRONG_LOCK_DEG) and hint_heading is not None:
                        snap = round(err / (math.pi / 2)) * (math.pi / 2)   # the mark's other axis / other end
                        hint_heading = wrap(hint_heading + snap)
                        print(f"  robot direction: it moves {math.degrees(err):+.0f} deg off the heading read from "
                              f"the mark - wrong axis taken; corrected by {math.degrees(snap):+.0f} deg")
                        drive_log = []
                    else:
                        drive_log = learn_heading_trim(drive_log, arena)
            else:                                          # true heading -> learn the triangle's error
                drive_log = []
            warn, info, close = None, [], 0.0
            if official is not None:
                side_cube = carrying and not claw_mode     # (a claw holds the cube in front: not modelled)
                depth = keepout_depth(pose, side_cube, 0)  # over the REAL tape
                close = keepout_depth(pose, side_cube)     # inside the safety margin (the tape counted thick)
                if depth > 0:
                    warn = f"OVER THE TAPE! {depth:.1f} cm"
                    keepout_events += not in_keepout
                elif close > 0:
                    warn = f"near the tape: {max(0.0, spec.SAFETY_CM - close):.1f} cm left"
                in_keepout = depth > 0
                info = [(f"car{ROBOT_NUM}  " if ROBOT_NUM else "") +
                        f"x {official[0]:5.1f}  y {official[1]:5.1f} cm  heading {official[2]:3.0f} deg"
                        f"   {'goal ' + goal if mode == 'route' and goal else 'ROAM' if mode == 'roam' else ''}"]
            view = draw(frame, top, tape, arena, tag_px, pose, plan, state, base, cmd, fps, running,
                        follower.route_points() if mode == "route" else None, info, warn)
            cv2.imshow("arena", view)
            if rec:
                rec.frame(view, state, base, fps, pose, plan, cmd, running, official, goal if mode == "route" else
                          mode, in_keepout if official is not None else False, found[0] if found else None,
                          margin=close > 0)
        if frame is not None and time.time() - t > 0.3:     # camera froze -> stop
            cmd, state = (0, 0), "NO CAMERA"
        if not enc_said and time.time() - auto_t > 0:      # (2 s after start) did the encoders answer?
            enc_said = True
            print("Wheel encoders: " + ("streaming (assist off)" if car.fresh() else
                                        "NO DATA (old firmware?) - driving on the camera alone"))
        if running and car.fresh() and max(abs(cmd[0]), abs(cmd[1])) >= MIN_PWM and car.odo[-1][4] < 30:
            stall_since = stall_since or time.time()       # motors driven, wheels not turning
            if time.time() - stall_since > STALL_S:
                running, cmd, state = False, (0, 0), "STALLED"
                print("WHEELS NOT TURNING although the motors are driven - battery flat? Robot stuck? Stopped.")
                auto_fail = auto_fail or "wheels not turning"
        else:
            stall_since = None
        # the same seen by the camera (works without the encoder stream): driven for CAM_STALL_S but the
        # robot hasn't moved or turned - e.g. a flat battery (2026-10-08: 20 s of "turn" with no movement)
        if running and pose is not None and max(abs(cmd[0]), abs(cmd[1])) >= MIN_PWM:
            if cam_stall is None:
                cam_stall = (time.time(), np.array(pose[0], float), pose[1])
            elif time.time() - cam_stall[0] > CAM_STALL_S:
                if (np.hypot(*(np.array(pose[0]) - cam_stall[1])) < 10 and
                        abs(wrap(pose[1] - cam_stall[2])) < math.radians(4)):
                    running, cmd, state = False, (0, 0), "STALLED"
                    print(f"ROBOT ISN'T MOVING: driven for {CAM_STALL_S:.0f} s but the camera sees no movement - "
                          "flat battery? motor power switch? stuck? Stopped.")
                    auto_fail = auto_fail or "robot not moving (battery?)"
                cam_stall = None
        elif max(abs(cmd[0]), abs(cmd[1])) < MIN_PWM:
            cam_stall = None
        if new or time.time() - last_send > 0.05:           # keep-alive at 20 Hz minimum
            car.send(*cmd)
            last_send = time.time()

        if qr_job is not None and not qr_job.is_alive():  # QR look done: show + save what the camera saw
            for cam_ip, text, img, pts in qr_job.results:
                show = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
                if pts is not None:
                    cv2.polylines(show, [np.int32(pts * 2)], True, (0, 255, 0), 2)
                label(show, [f"{cam_ip}  at {qr_goal}", f"QR: {text}" if text else "QR: not read"])
                cv2.imshow("esp32-cam", show)
                folder = rec.dir if rec else os.path.join(HERE, "runs")
                os.makedirs(folder, exist_ok=True)
                path = os.path.join(folder, f"qr_{qr_goal}_{cam_ip.split('.')[-1]}_{time.strftime('%H%M%S')}.jpg")
                cv2.imwrite(path, show)
                cv2.imwrite(path.replace(".jpg", "_raw.png"), img)   # (the camera's own pixels)
                print(f"QR at {qr_goal} via {cam_ip}: " + (repr(text) if text else "not read") + f"  (picture: {path})")
                if text:
                    qr_found.append((qr_goal, cam_ip, text))
            if not qr_job.results:
                print("QR: no picture from the camera(s)")
            qr_job = None
        k = cv2.waitKey(1) & 0xFF
        if auto is not None:                               # --auto: press the next key when idle
            busy = test is not None or running or cam_moved or arm is not None or qr_job is not None
            if cam_moved and time.time() - cam_moved_t > CAMERA_WAIT_S:
                auto_fail = auto_fail or "camera moved (restart: it maps the arena at every start)"
                busy = False
            if busy and auto_step and time.time() - auto_t > AUTO_STEP_S:
                auto_fail = auto_fail or f"{auto_step} took longer than {AUTO_STEP_S} s"
                running, test = False, None
            if auto_step == "H" and test is None and arena.heading_offset is None:
                auto_fail = auto_fail or "heading test failed"
            if k == 255 and not busy and time.time() - auto_t > 1.5:
                if auto_fail or not auto:
                    print("--auto: " + (f"STOPPED - {auto_fail}" if auto_fail else "all steps done"))
                    k = ord('q')
                else:
                    auto_step = auto.pop(0)
                    print(f"--auto: {auto_step}")
                    k, auto_t = AUTO_KEYS[auto_step], time.time()
        if k in GOAL_KEYS and OFFICIAL and test is None:   # drive to a station / bay
            if arena.heading_offset is None:
                print("Press H first (heading test) - the script doesn't know where the robot's front is yet.")
            elif last_official is None or time.time() - last_seen > LOST_S:
                print("Can't see the robot's marker - can't plan a route.")
                auto_fail = auto_fail or "robot not seen when a goal was due"   # (--auto: don't skip it)
            else:
                goal, mode = spec.goal_node(GOAL_KEYS[k]), "route"     # no cut-outs: a bay -> its mouth
                if follower.go(*last_official, goal):
                    running = True
                    print(f"Going to {goal}: " + " -> ".join(f"({p[0]:.0f},{p[1]:.0f})" for p in follower.route_points()))
                else:
                    print(f"No route to {goal} from here: {follower.state}" + FOLLOW_HELP.get(follower.state, ""))
                    auto_fail = auto_fail or f"{goal}: {follower.state}"
        elif k == ord(' '):
            if arena.heading_offset is None:
                print("Press H first (heading test) - the script doesn't know where the robot's front is yet.")
            elif test is None:
                running = not running
                planner.reset()
                if running and mode == "route":
                    if goal is None or last_official is None:
                        running = False
                        print("Press a goal key first (1-6, D, 7-9), or R to roam.")
                    elif time.time() - last_seen > LOST_S:        # don't re-plan from an old position
                        running = False
                        print("Can't see the robot's marker - can't re-plan from where it is now.")
                    elif not follower.go(*last_official, goal):   # re-plan from where it is now
                        running = False
                        print(f"Can't go on to {goal}: {follower.state}" + FOLLOW_HELP.get(follower.state, ""))
        elif k in (ord('k'), ord('K'), ord('u'), ord('U')) and not running and test is None and arm is None:
            kind = "pick" if k in (ord('k'), ord('K')) else "drop"
            here = goal if follower.state == "ARRIVED" else None
            if claw_mode and claw_points(car) is None:
                print("The claw isn't set up yet: on the robot's web page, Open -> Save open, then cube in, Close "
                      "until it touches -> Open 1 deg until it stops buzzing -> Save touch.")
                auto_fail = auto_fail or "claw not set up"
            elif not claw_mode and not arm_ok:
                print("The pickup arm isn't calibrated yet - run:  python tools/arm_calibrate.py  (won't guess angles"
                      " that could jam the scissor)")
                auto_fail = auto_fail or "arm not calibrated"
            elif kind == "pick" and carrying:
                print("PICK: already carrying a cube - drop it first (U).")
                auto_fail = auto_fail or "PICK while carrying"
            elif kind == "drop" and not carrying and auto is not None:
                print("DROP: no cube on board.")
                auto_fail = auto_fail or "DROP without a cube"
            else:
                if kind == "pick" and here not in spec.STATIONS:
                    print("PICK: note - the robot hasn't just arrived at a station.")
                if kind == "drop" and here not in spec.BAYS:
                    print("DROP: note - the robot hasn't just arrived in a bay.")
                if claw_mode:
                    print("PICK: open the claw, then close it on the cube ..." if kind == "pick" else
                          "DROP: opening the claw ...")
                else:
                    print("PICK: reach, then pull the cube in ..." if kind == "pick" else "DROP: opening the arm ...")
                arm = arm_task(kind, calib, car)
        elif k in (ord('o'), ord('O')) and not running and test is None and arm is None:   # OPEN the claw
            pts = claw_points(car) if claw_mode else None
            if not pts:
                print("OPEN: no claw set up on this robot (calib 'claw' + the robot page's Save open / Save touch).")
                auto_fail = auto_fail or "OPEN: claw not set up"
            else:
                print(f"OPEN: opening the claw to {pts[0]} deg ...")
                arm = {"kind": "open", "steps": [(pts[0], 0.3, "drop")], "i": -1, "t": 0.0, "until": None,
                       "speed": CLAW_DEG_PER_S}
        elif k in (ord('f'), ord('F')) and not running and test is None and arm is None:   # FACE the alcove
            if goal not in spec.STATIONS or follower.state != "ARRIVED" or last_official is None:
                print("FACE: drive to a station first (it turns there to face the alcove with the claw).")
                auto_fail = auto_fail or "FACE: not at a station"
            else:
                face = (spec.DIRS[spec.STATIONS[goal]["alcove"]] + (180 if calib.get("claw") == "back" else 0)) % 360
                follower.final_heading, follower.pieces, follower.i = math.radians(face), [], 0
                follower._reset_tries()
                follower.state, mode, running = "FINAL TURN", "route", True
                print(f"FACE: turning on {goal} to face its alcove ({face:.0f} deg) with the claw")
        elif k in (ord('r'), ord('R')) and test is None:
            mode, running = "roam", False
            print("Roam mode: SPACE = start/stop. Press a goal key to go back to driving routes.")
        elif k in (ord('l'), ord('L')) and not running and test is None and qr_job is None:   # QR look
            if not qr_cams:
                print("QR: no ESP32-CAM address - add --qr-cams IP[,IP] (or cam_host in the robot's calib)")
                auto_fail = auto_fail or "QR: no camera"
            else:
                print(f"QR: looking with {', '.join(qr_cams)} ...")
                qr_job, qr_goal = QRLook(qr_cams), goal
                qr_job.start()
        elif k in (ord('y'), ord('Y')) and not running and test is None:   # SYNC move
            if arena.heading_offset is None or pose is None or time.time() - last_seen > LOST_S:
                print("SYNC: can't see the robot (or no H test yet) - can't sync.")
                auto_fail = auto_fail or "SYNC: robot not seen"
            elif room_along(pose, SYNC_ROOM_MM) or room_along(pose, -SYNC_ROOM_MM):
                sign = 1 if room_along(pose, SYNC_ROOM_MM) else -1
                print("SYNC: short straight move " + ("forward and back" if sign > 0 else "back and forward")
                      + " to measure which way the robot points ...")
                test = {"t0": time.time(), "log": [], "kind": "sync", "sign": sign}
            else:
                print("SYNC: no room to move along the lane here - keeping the saved heading offset.")
        elif k in (ord('h'), ord('H')) and not running and test is None:
            print("Heading test: forward, stop, turn right, stop ...")
            test = {"t0": time.time(), "log": []}
        elif k in (ord('+'), ord('=')):
            base = min(255, base + 10)
        elif k == ord('-'):
            base = max(MIN_PWM, base - 10)
        elif k in (ord('q'), 27):
            break

    for _ in range(5):
        car.send(0, 0)
        time.sleep(0.02)
    car.close()
    if rec:
        rec.close()
    if cam_job is not None:
        cam_job.join()
    if learnt_calib(arena) != learnt_was or remapped:       # learnt while driving: keep it for next time
        calib.update(learnt_calib(arena))
        save_calib(calib)
        print(f"Saved what it learnt: turning point {arena.turn_centre[0]:+.0f} mm forward, "
              f"{arena.turn_centre[1]:+.0f} mm left of the marker; heading trim " +
              " ".join(f"{d} {v:+.1f}" for d, v in calib["heading_trim_deg"].items()) + " deg")


def find_cameras():
    print("Trying camera numbers 0-5. Each one that works is shown for 2 seconds.")
    for i in range(6):
        cap = open_camera(i)
        ok, f = (False, None)
        for _ in range(10):                                  # some cameras need a few frames
            if cap is None:
                break
            ok, f = cap.read()
            if ok:
                break
        if ok:
            print(f"  camera {i}: {f.shape[1]}x{f.shape[0]}")
            cv2.putText(f, f"camera {i}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
            cv2.imshow("find cameras", f)
            cv2.waitKey(2000)
        else:
            print(f"  camera {i}: nothing")
        if cap is not None:
            cap.release()
    cv2.destroyAllWindows()


def use_robot(n):
    """--robot N: this run is about robot N - its own calibration file (made by tools/bind_robots.py)."""
    global CALIB_FILE, ROBOT_NUM
    ROBOT_NUM = n
    CALIB_FILE = os.path.join(HERE, f"arena_calib_car{n}.json")


def apply_robot_calib(calib):
    """A robot calibration from bind_robots.py names the robot's ArUco marker and how high it sits."""
    global ROBOT_MARK, ROBOT_TAG_ID, TAG_HEIGHT_MM
    if calib and calib.get("tag_id") is not None:
        ROBOT_MARK, ROBOT_TAG_ID = "aruco", int(calib["tag_id"])
    if calib and calib.get("tag_height_mm"):
        TAG_HEIGHT_MM = float(calib["tag_height_mm"])
    for key, name in (("pivot_pwm", "PIVOT_PWM"), ("min_pwm", "MIN_PWM")):   # this robot's turning power
        if calib and calib.get(key):                    # (a robot with weak wheels needs more to turn)
            setattr(lane_follow, name, int(calib[key]))
            print(f"  {name} for this robot: {calib[key]}")


def main():
    ap = argparse.ArgumentParser(description="Webcam watches the arena, robot drives between the tape lines.")
    ap.add_argument("--robot", type=int, choices=(1, 2, 3), default=None,
                    help="drive robot N with its own calibration (arena_calib_carN.json from tools/bind_robots.py)")
    ap.add_argument("--find", action="store_true", help="show each camera number")
    ap.add_argument("--make-tag", action="store_true", help="save robot_tag.png to print")
    ap.add_argument("--setup", action="store_true", help="click the arena corners + tape (camera must not move after)")
    ap.add_argument("--auto-setup", action="store_true", help="only map the arena from the red boundary tape, then quit")
    ap.add_argument("--no-sync", action="store_true", help="--auto: skip the SYNC move before the first goal")
    ap.add_argument("--no-auto-setup", action="store_true",
                    help="don't re-map the arena at start (use the saved arena_calib.json)")
    ap.add_argument("--camera", default=None, help="webcam number (or stream URL)")
    ap.add_argument("--arena-w", type=float, default=None, help="setup: length of the 1st-2nd click side, mm")
    ap.add_argument("--arena-h", type=float, default=None, help="setup: length of the 2nd-3rd click side, mm")
    ap.add_argument("--car", default=CAR_HOST, help="robot address (car1.local or its IP)")
    ap.add_argument("--record", action="store_true", help="save pictures + decisions to laptop/runs/")
    ap.add_argument("--qr-cams", default=None, metavar="IPS",
                    help="ESP32-CAM address(es) for the QR step (default: cam_host in the robot's calib)")
    ap.add_argument("--orders", default=None, metavar="STATIONS",
                    help="fulfil orders, e.g. S1,S4: for each -> drive there, PICK, drive to DZ, DROP (after --auto steps)")
    ap.add_argument("--auto", default=None, metavar="STEPS",
                    help="press keys by script, e.g. H,S2,DZ (H = heading test; S1-S6, DZ, P1-P3 = goals)")
    a = ap.parse_args()
    if a.make_tag:
        make_tag()
        return
    if a.find:
        find_cameras()
        return

    if a.robot:
        use_robot(a.robot)
        if not os.path.exists(CALIB_FILE):
            print(f"No {os.path.basename(CALIB_FILE)} yet - run  python tools/bind_robots.py  first.")
            return
    calib = None
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE) as f:
            calib = json.load(f)
    apply_robot_calib(calib)
    if a.robot and a.car == CAR_HOST and calib.get("host"):
        a.car = calib["host"]                              # (the robot's address, as bind_robots.py found it)
    src = a.camera if a.camera is not None else (calib or {}).get("camera", CAMERA)
    if isinstance(src, str) and src.isdigit():
        src = int(src)

    if a.auto_setup:
        cam = Camera(src)
        auto_setup(cam, src)
        cam.running = False
        return
    if a.setup:
        if OFFICIAL:                                       # the official floor is always 3 x 3 m
            aw = ah = float(ARENA_MM)
        else:
            aw = a.arena_w or float(input("Arena side 1 (first two clicks), mm: "))
            ah = a.arena_h or float(input("Arena side 2, mm: "))
        cam = Camera(src)
        setup(cam, src, aw, ah)
        cam.running = False
        return
    steps = None
    if a.auto or a.orders:
        steps = [s.strip().upper() for s in (a.auto or "").split(",") if s.strip()]
        for st in [s.strip().upper() for s in (a.orders or "").split(",") if s.strip()]:
            if st not in spec.STATIONS:
                print(f"--orders: {st} is not a station (S1-S6)")
                return
            steps += ([st, "QR", "FACE", "PICK", "DZ", "DROP"] if (calib or {}).get("claw") in ("front", "back")
                      else [st, "PICK", "DZ", "DROP"])            # (a front / back claw: look, turn to the cube)
        bad = [s for s in steps if s not in AUTO_KEYS]
        if bad:
            print("--auto: unknown step(s)", bad, "- use SYNC, H, S1-S6, DZ, P1-P3, QR, OPEN, FACE, PICK, DROP")
            return
        if not a.no_sync and steps and steps[0] not in ("SYNC", "H"):
            steps.insert(0, "SYNC")                        # measure the heading afresh before driving
    cam = Camera(src)
    if OFFICIAL and not a.no_auto_setup:                   # map the arena afresh at every start: then a
        print("Mapping the arena from the red boundary ...")   # moved camera just works
        new = auto_setup(cam, src)
        if new is not None:
            calib = new
        elif calib is not None:
            print("Using the map saved last time - only right if the camera hasn't moved since.")
    if calib is not None and calib.get("official", False) != OFFICIAL:
        print("arena_calib.json was made for " + ("the official floor" if calib.get("official") else "another floor")
              + " but OFFICIAL is now " + str(OFFICIAL) + " - run --setup again.")
        cam.running = False
        return
    if calib is None:
        print("No arena map - is the whole red boundary in view? Or run with --setup (see the top of this file).")
        cam.running = False
        return
    qr_cams = [c.strip() for c in (a.qr_cams or calib.get("cam_host", "") or "").split(",") if c.strip()]
    run(cam, calib, a.car, a.record, steps, qr_cams)
    cam.running = False


if __name__ == "__main__":
    main()
