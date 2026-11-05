"""Where does a robot's claw hold the cube? Measured from the overhead camera, with a cube held in the claw, relative to
the robot's turning point (its pose): mm ahead and mm to the left. Saved per robot (arena_calib.json robots[marker]
claw_hold_mm / claw_side_mm), which arena_drive uses to line the CLAW up with a cube and to set the creep.

usage:  python tools/claw_measure.py ROBOT_IP MARKER_ID [--save]
  The robot stands still in the arena (its turning point already learnt - H test or a drive), the cube in its open
  claw as it would be held. The claw is closed to the robot's carry angle first.
"""
import os, sys, time, math, json, socket
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad                 # (first: it sets the webcam option OpenCV must see before loading)
import cv2, numpy as np

ip, tag = sys.argv[1], int(sys.argv[2])
save = "--save" in sys.argv
ad.ROBOT_TAG_ID = tag
calib = ad.load_calib()
deg = ad.arm_angles(calib)
if deg is None:
    sys.exit(f"marker {tag}: claw not calibrated (tools/arm_calibrate.py) - won't move it")
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(1.0)
s.sendto(f"servo {deg['carry']}".encode(), (ip, ad.CAR_PORT))
try:
    print("claw:", s.recvfrom(512)[0].decode())
except socket.timeout:
    sys.exit(f"no answer from the robot at {ip}")
cam = ad.Camera(calib.get("camera", 0))
corners, _, rep = ad.map_boundary(cam)
if corners is None:
    sys.exit("arena not mapped: " + rep)
calib["corners_px"] = corners
calib["focal_px"], calib["camera_mm"] = ad.camera_pose(corners, ad.ARENA_MM, ad.ARENA_MM, (1920, 1080))
arena = ad.Arena(calib)
det = ad.make_detector()
results = []
for _ in range(8):                                         # several frames: the median
    time.sleep(0.15)
    frame = cam.latest()[0]
    r = ad.find_robot(frame, det, arena)
    if r is None:
        continue
    h = r[1] + arena.heading_offset
    h += ad.heading_trim(arena, h)
    tp = np.array(ad.mark_to_centre(r[0], h, arena.turn_centre))
    fwd, left = np.array([math.cos(h), math.sin(h)]), np.array([math.sin(h), -math.cos(h)])   # map frame (y down)
    # top-down patch at the cube's TOP height, 1 px = 1 mm: rows = mm ahead (220 .. 20), columns = mm left (+130 .. -130)
    k = arena._k(ad.CUBE_MM)
    cxy = arena.cam[:2]
    seen = lambda p: cxy + (p - cxy) / k                   # true place at cube height -> where it shows on the floor
    quad = [tp + fwd * a + left * b for a, b in ((220, 130), (220, -130), (20, -130), (20, 130))]
    src = cv2.perspectiveTransform(np.float32([[seen(p) for p in quad]]), arena.floor2img)[0]
    W, H = 260, 200
    M = cv2.getPerspectiveTransform(src, np.float32([[0, 0], [W, 0], [W, H], [0, H]]))
    patch = cv2.warpPerspective(frame, M, (W, H))
    marker = np.zeros(frame.shape[:2], np.uint8)            # the marker sheet (white border = 1/6 of the square)
    c = r[2].mean(0)
    cv2.fillConvexPoly(marker, np.int32(c + (r[2] - c) * 1.45), 255)
    mask = cv2.warpPerspective(marker, M, (W, H)) > 0
    V = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[..., 2].astype(np.int16)
    floor_v = np.median(V[~mask])
    cand = ((V - floor_v > ad.CUBE_BRIGHT_V) & ~mask).astype(np.uint8)
    cand = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    cand = cv2.morphologyEx(cand, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cand[mask] = 0
    n, lab, stats, _ = cv2.connectedComponentsWithStats(cand, connectivity=8)
    big = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= ad.CUBE_MIN_PX]
    if not big:
        continue
    i = max(big, key=lambda i: stats[i, cv2.CC_STAT_AREA])
    rows, cols = np.nonzero(lab == i)
    ahead, lft = 220 - rows, 130 - cols                    # mm in the robot frame (at the cube's top)
    # the top face is at its true place; the side faces seen at a slant only add on the camera's side
    away = np.array([(tp - cxy) @ fwd, (tp - cxy) @ left]); away /= np.linalg.norm(away)
    s_al = ahead * away[0] + lft * away[1]
    t_ac = -ahead * away[1] + lft * away[0]
    far, mid = np.percentile(s_al, 98) - ad.CUBE_MM / 2, (np.percentile(t_ac, 2) + np.percentile(t_ac, 98)) / 2
    centre = far * away + mid * np.array([-away[1], away[0]])
    results.append(centre)
    if len(results) == 1:
        v = patch.copy(); v[mask] //= 3
        cv2.circle(v, (int(130 - centre[1]), int(220 - centre[0])), 4, (0, 0, 255), -1)
        cv2.imwrite(os.path.join(HERE, "claw_measure.jpg"), cv2.resize(v, None, fx=2, fy=2))
cam.running = False
if not results:
    sys.exit("no cube seen in front of the robot")
hold, side = np.median(results, axis=0)
spread = np.ptp(np.array(results), axis=0)
print(f"marker {tag}: the claw holds the cube {hold:.0f} mm ahead and {side:+.0f} mm left of the turning point "
      f"({len(results)} frames, spread {spread[0]:.0f} / {spread[1]:.0f} mm). Picture: tools/claw_measure.jpg")
if save:
    calib["claw_hold_mm"], calib["claw_side_mm"] = round(float(hold)), round(float(side))
    raw = ad.load_calib()                                  # (only this robot's two values change)
    raw["claw_hold_mm"], raw["claw_side_mm"] = calib["claw_hold_mm"], calib["claw_side_mm"]
    ad.save_calib(raw)
    print("saved for marker", tag)
