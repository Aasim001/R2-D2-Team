"""Check (and fix) which way a robot's marker says its front points: a ~3 cm nudge forward and back, watched by the
overhead webcam. The way the robot really moves = its front. Works anywhere (also in a bay, where arena_drive's
SYNC move finds no room). Use it after a marker was taken off / turned, before fleet_drive.py.

  python tools/heading_check.py --robot 1            # measure; fix arena_calib_car1.json if > 20 deg off
  python tools/heading_check.py --robot 1 --no-save  # only measure

2026-10-10: car1's marker had been turned round (180 deg) after its last SYNC; in the first fleet run it then
reversed out of the BACK of its bay. This catches that.
"""
import argparse, json, math, os, struct, sys, time
LAPTOP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, LAPTOP)
import arena_drive as ad                                  # (before cv2)
import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, required=True)
    ap.add_argument("--camera", type=int, default=1)
    ap.add_argument("--pwm", type=int, default=115)
    ap.add_argument("--secs", type=float, default=0.35, help="nudge length (~3 cm at 115 PWM)")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--tol", type=float, default=5.0, help="save the new offset if it differs more than this (deg). "
                    "2026-10-10: a ~13 deg error (marker knocked) passed the old 20 deg limit and car1 drove off the ring")
    a = ap.parse_args()
    path = os.path.join(LAPTOP, f"arena_calib_car{a.robot}.json")
    calib = json.load(open(path))
    arena = ad.Arena(calib, load_map=False)
    tag, tag_mm = int(calib["tag_id"]), float(calib.get("tag_height_mm") or ad.TAG_HEIGHT_MM)
    min_pwm = int(calib.get("min_pwm") or 0)
    pwm = max(a.pwm, min_pwm)
    car = ad.Car(calib["host"], ad.CAR_PORT)
    car.text("assist off")
    cam = ad.Camera(a.camera)
    det = ad.make_detector()
    t0 = time.time()
    while cam.latest()[0] is None and time.time() - t0 < 20:
        time.sleep(0.05)

    def look(n=6):
        """Average marker position (map mm) and raw angle (rad) over n frames."""
        ps, angs, last = [], [], None
        t1 = time.time()
        while len(ps) < n and time.time() - t1 < 5:
            f, ft = cam.latest()
            if f is None or ft == last:
                time.sleep(0.01)
                continue
            last = ft
            x0, y0, x1, y1 = arena.search_box
            corners, ids, _ = det.detectMarkers(cv2.cvtColor(f[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY))
            if ids is None or tag not in ids.ravel():
                continue
            c = corners[list(ids.ravel()).index(tag)].reshape(4, 2) + (x0, y0)
            pts = arena.lift(cv2.perspectiveTransform(c.reshape(1, 4, 2).astype(np.float32), arena.img2floor)[0], tag_mm)
            up = (pts[0] + pts[1]) / 2 - (pts[2] + pts[3]) / 2
            ps.append(pts.mean(0))
            angs.append(math.atan2(up[1], up[0]))
        if not ps:
            return None, None
        return np.mean(ps, axis=0), math.atan2(np.mean(np.sin(angs)), np.mean(np.cos(angs)))

    seq = [0]

    def drive(l, r, secs):
        t1 = time.time()
        while time.time() - t1 < secs:
            seq[0] = (seq[0] + 1) & 0xFFFF
            car.sock.sendto(b"DR" + struct.pack("<hhH", l, r, seq[0]), car.addr)
            time.sleep(0.05)
        for _ in range(4):
            seq[0] = (seq[0] + 1) & 0xFFFF
            car.sock.sendto(b"DR" + struct.pack("<hhH", 0, 0, seq[0]), car.addr)
            time.sleep(0.03)

    try:
        p0, a0 = look()
        if p0 is None:
            sys.exit(f"car{a.robot}'s marker (id {tag}) is not in view.")
        drive(pwm, pwm, a.secs)
        time.sleep(0.7)
        p1, _ = look()
        drive(-pwm, -pwm, a.secs)
        time.sleep(0.7)
        p2, _ = look()
    finally:
        car.text("stop")
        cam.running = False
    fwd = p1 - p0
    back = p2 - p1
    d_f, d_b = np.linalg.norm(fwd), np.linalg.norm(back)
    print(f"car{a.robot}: forward nudge moved {d_f:.0f} mm, back {d_b:.0f} mm")
    if d_f < 10 or d_b < 10:
        sys.exit("It hardly moved - battery? Wi-Fi? Not changed.")
    travel = math.atan2(fwd[1] - back[1], fwd[0] - back[0])     # (both legs: forward and minus backward)
    new = math.degrees((travel - a0 + math.pi) % (2 * math.pi) - math.pi)
    old = calib.get("heading_offset_deg")
    diff = None if old is None else (new - old + 180) % 360 - 180
    print(f"  robot front = marker {new:+.1f} deg (saved: {old}; difference {diff if diff is None else round(diff, 1)} deg)")
    if old is not None and abs(diff) <= a.tol:
        print(f"  OK - within {a.tol:.0f} deg of the saved heading offset.")
        return
    if a.no_save:
        print("  (--no-save: not changed)")
        return
    calib["heading_offset_deg"] = round(new, 1)
    if old is not None and abs(diff) > 60:                       # the marker was turned: the trims and turning
        calib["heading_trim_deg"] = {}                           # point (in marker coordinates) no longer fit
        calib["turn_centre_mm"] = [0.0, 0.0]
        print("  (the marker was turned: trims and turning point reset - they are learnt again while driving)")
    with open(path, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"  SAVED heading_offset_deg = {new:+.1f} in {os.path.basename(path)}")


if __name__ == "__main__":
    main()
