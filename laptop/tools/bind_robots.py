"""Which ArUco marker is on which robot? For each robot address it nudges that robot (about 3 cm forward and
back, then a small left/right wiggle) and watches which marker moves. From the same moves it also learns:
  - which way the robot's front is, relative to its marker      -> heading_offset_deg
  - whether its turning is reversed (motors swapped)             -> turn_sign
  - how high the marker sits, from how big it looks              -> tag_height_mm
and writes laptop/arena_calib_carN.json for each robot it found (the arena map comes from the red boundary,
like every start of arena_drive.py). Then drive a robot with:  python arena_drive.py --robot N --auto ...

usage:  python tools/bind_robots.py --camera 1 1=192.168.43.93 2=192.168.43.218
  --marker-mm = side of the BLACK square of the robot markers (default: arena_drive.ROBOT_TAG_MM, 115)
  --height-mm = floor -> top of the marker (default: arena_drive.TAG_HEIGHT_MM, 145, measured)
  The arena is mapped like every start of arena_drive.py: floor anchors (ids 0-3) and/or the red boundary.
Only small moves: each robot must stand where ~5 cm of travel each way is harmless (in a lane or a bay).
"""
import os, sys, json, math, time, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad          # (first: it sets the webcam option OpenCV must see before loading)
import cv2, numpy as np

PULSE_PWM, PULSE_S = 110, 0.3     # forward / back nudge (~3 cm)
TURN_PWM, TURN_S = 120, 0.2       # left / right wiggle (~10 deg)
SETTLE_S = 0.8                    # wait after each nudge before looking
LOOK_FRAMES = 8                   # frames averaged per look
MOVED_MM = 10                     # a marker that moved more than this during the nudges = this robot


def detect_all(frame, detector, box):
    """{id: corners px (4x2)} of every marker in the arena area, at 1x and (for small far ones) 2x."""
    x0, y0, x1, y1 = box
    gray = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    found = {}
    for scale in (1, 2):
        img = gray if scale == 1 else cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = detector.detectMarkers(img)
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if int(i) not in found:
                    found[int(i)] = (c.reshape(4, 2) + 0.5) / scale - 0.5 + [x0, y0]
    return found


def look(cam, detector, box, n=LOOK_FRAMES):
    """Median corners per marker id over n new frames."""
    seen, last, t0 = {}, 0.0, time.time()
    got = 0
    while got < n and time.time() - t0 < 10:
        frame, t = cam.latest()
        if frame is None or t == last:
            time.sleep(0.01)
            continue
        last, got = t, got + 1
        for i, c in detect_all(frame, detector, box).items():
            seen.setdefault(i, []).append(c)
    return {i: np.median(np.array(v), axis=0) for i, v in seen.items() if len(v) >= max(2, n // 2)}


def floor_pts(arena, c_px):
    return cv2.perspectiveTransform(np.float32(c_px).reshape(1, 4, 2), arena.img2floor)[0]


def side_mm(pts):
    return float(np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]))


def pose(arena, c_px, h):
    """Marker centre (map mm) and the direction of its top edge (map rad), at marker height h."""
    pts = arena.lift(floor_pts(arena, c_px), h)
    up = (pts[0] + pts[1]) / 2 - (pts[2] + pts[3]) / 2
    return pts.mean(0), math.atan2(up[1], up[0])


def drive(car, left, right, secs):
    """Hold a wheel command for secs (packets every 50 ms - the robot stops 300 ms after the last one)."""
    t = time.time()
    while time.time() - t < secs:
        car.send(left, right)
        time.sleep(0.05)
    for _ in range(3):
        car.send(0, 0)
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robots", nargs="+", help="N=IP, e.g. 1=192.168.43.93")
    ap.add_argument("--camera", default=None)
    ap.add_argument("--marker-mm", type=float, default=ad.ROBOT_TAG_MM, help="side of the robot markers' black square")
    ap.add_argument("--height-mm", type=float, default=ad.TAG_HEIGHT_MM, help="measured floor -> marker height")
    a = ap.parse_args()
    robots = []
    for r in a.robots:
        n, ip = r.split("=")
        robots.append((int(n), ip))
    base_file = os.path.join(ad.HERE, "arena_calib.json")
    old_base = json.load(open(base_file)) if os.path.exists(base_file) else {}
    src = a.camera if a.camera is not None else old_base.get("camera", ad.CAMERA)
    src = int(src) if str(src).isdigit() else src

    cam = ad.Camera(src)
    print("Mapping the arena (floor anchors / red boundary) ...")
    corners, frame, report, how = ad.map_arena(cam)
    if corners is None:
        sys.exit("Mapping failed: " + report)
    fit, _ = ad.tape_fit_mm(frame, corners)
    if fit > ad.TAPE_FIT_MAX_MM:
        sys.exit(f"The map is {fit:.0f} mm off the black tape - something on the boundary? Not used.")
    h_img, w_img = frame.shape[:2]
    focal, cam_mm = ad.camera_pose(corners, float(ad.ARENA_MM), float(ad.ARENA_MM), (w_img, h_img))
    camera = dict(camera=src, frame_size=[w_img, h_img], arena_w=float(ad.ARENA_MM), arena_h=float(ad.ARENA_MM),
                  corners_px=corners, tape_ranges=[], focal_px=round(focal, 1),
                  camera_mm=[round(c, 1) for c in cam_mm], official=True, calibrated_by=how)
    arena = ad.Arena(dict(camera, heading_offset_deg=None), load_map=False)
    print(f"  {report}; tape fit {fit:.0f} mm; webcam {cam_mm[2] / 1000:.2f} m up")
    detector = ad.make_detector()
    box = arena.search_box

    p0 = look(cam, detector, box, 15)
    inside = lambda c: all(-50 < v < ad.ARENA_MM + 50 for v in floor_pts(arena, c).mean(0))
    p0 = {i: c for i, c in p0.items() if inside(c)        # (a marker held up outside the arena doesn't count,
          and i not in ad.spec.FLOOR_MARKERS}               #  nor the floor anchors)
    print("Markers in the arena:", {i: tuple(np.round(c.mean(0)).astype(int)) for i, c in p0.items()} or "none")
    heights = {}
    for i, c in p0.items():                                # height from the marker's size: seen from the camera,
        s = side_mm(floor_pts(arena, c))                   # it looks bigger on the floor plane by H / (H - h)
        heights[i] = arena.cam[2] * (1 - a.marker_mm / s)
        print(f"  marker {i}: looks {s:.1f} mm on the floor plane -> about {heights[i]:.0f} mm above the floor"
              f" by its size (measured: {a.height_mm:.0f} mm - a big difference = corners found inside the edges)")

    results = {}
    for n, ip in robots:
        unbound = set(p0) - {r["tag"] for r in results.values()}
        if not unbound:                                    # never move a robot that can't be in the arena
            print(f"\nRobot {n} at {ip}: skipped - no unidentified marker in the arena (is it on the floor?)")
            continue
        print(f"\nRobot {n} at {ip}: nudging it ...")
        try:
            car = ad.Car(ip, ad.CAR_PORT)
        except OSError as e:
            print("  can't reach it:", e)
            continue
        for cmd in ("stop", "assist off"):
            car.text(cmd)
        time.sleep(0.3)
        looks = [look(cam, detector, box)]
        for l, r, s in ((PULSE_PWM, PULSE_PWM, PULSE_S), (-PULSE_PWM, -PULSE_PWM, PULSE_S),
                        (-TURN_PWM, TURN_PWM, TURN_S), (TURN_PWM, -TURN_PWM, TURN_S)):
            drive(car, l, r, s)
            time.sleep(SETTLE_S)
            looks.append(look(cam, detector, box))
        ids = [i for i in looks[0] if i in unbound and all(i in lk for lk in looks)]
        moved = {i: max(np.linalg.norm(floor_pts(arena, lk[i]).mean(0) - floor_pts(arena, looks[0][i]).mean(0))
                        for lk in looks[1:]) for i in ids}
        print("  movement per marker (mm):", {i: round(m) for i, m in moved.items()})
        mine = [i for i, m in moved.items() if m > MOVED_MM]
        if len(mine) != 1:
            print("  -> can't tell which marker is this robot's" + (" (none moved: battery? address?)" if not mine
                                                                   else f" (several moved: {mine})"))
            continue
        tag = mine[0]
        h = a.height_mm                                    # (measured; the size estimate above is only a check)
        (c0, a0), (c1, _), (c2, t1), (_, t2), _ = [pose(arena, lk[tag], h) for lk in looks]
        fwd, back = c1 - c0, c2 - c1
        if np.hypot(*fwd) < 15:
            print(f"  -> moved only {np.hypot(*fwd):.0f} mm forward - not enough to tell its front")
            continue
        offset = ad.wrap(math.atan2(fwd[1], fwd[0]) - a0)
        agree = abs(ad.wrap(math.atan2(fwd[1], fwd[0]) - math.atan2(-back[1], -back[0])))
        left_turn = ad.wrap(t2 - t1)                       # the left wiggle (from the pose after the back nudge)
        turn_sign = 1 if left_turn < 0 else -1             # (map y points down: counter-clockwise = angle falls)
        print(f"  -> marker {tag}; forward {np.hypot(*fwd):.0f} mm, back {np.hypot(*back):.0f} mm (agree within "
              f"{math.degrees(agree):.0f} deg); robot front = marker top {math.degrees(offset):+.1f} deg; "
              f"left wiggle turned {math.degrees(-left_turn):+.1f} deg -> turning "
              f"{'normal' if turn_sign == 1 else 'REVERSED'}; marker {h:.0f} mm up")
        if abs(left_turn) < math.radians(2):
            print("     (the wiggle hardly turned it - turn direction unsure, assuming normal; the H test checks it)")
            turn_sign = 1
        results[n] = dict(ip=ip, tag=tag, h=h, offset=offset, turn_sign=turn_sign)

    cam.running = False
    for n, r in results.items():
        path = os.path.join(ad.HERE, f"arena_calib_car{n}.json")
        old = json.load(open(path)) if os.path.exists(path) else {}
        same = (old.get("tag_id") == r["tag"] and old.get("heading_offset_deg") is not None and
                abs(math.degrees(ad.wrap(math.radians(old["heading_offset_deg"]) - r["offset"]))) < 20)
        calib = dict(camera, robot=n, host=r["ip"], tag_id=r["tag"], tag_height_mm=round(r["h"], 1),
                     heading_offset_deg=round(math.degrees(r["offset"]), 1), turn_sign=r["turn_sign"],
                     turn_centre_mm=old.get("turn_centre_mm", [0.0, 0.0]) if same else [0.0, 0.0],
                     heading_trim_deg=old.get("heading_trim_deg", {}) if same else {})
        for key in ("arm_deg", "arm_calibrated", "enc_rad_per_mm", "enc_turns_learnt", "pivot_pwm", "min_pwm", "cam_host"):
            if key in old:
                calib[key] = old[key]
        with open(path, "w") as f:
            json.dump(calib, f, indent=2)
        print(f"Saved {os.path.basename(path)}: car{n} = marker {r['tag']} at {r['ip']}"
              + ("" if same else " (new: turning point + heading trims are learnt again)"))
    if not results:
        print("No robot bound.")


if __name__ == "__main__":
    main()
