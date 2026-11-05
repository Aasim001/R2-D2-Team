"""Compare the triangle-tip direction from corners (minEnclosingTriangle, current) with the moments method
on live frames (no driving). usage: python tools/direction_probe.py [frames]"""
import os, sys, json, math
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad          # (first: it sets the webcam option OpenCV must see before loading)
import numpy as np, cv2

def moment_dir(floor):
    """Tip direction from area moments: principal axes from the 2nd moments; along each, the 3rd moment
    (skew: + = the thin end / tip lies that way). The axis with the stronger normalised skew wins."""
    m = cv2.moments(floor.reshape(-1, 1, 2).astype(np.float32))
    th = 0.5 * math.atan2(2 * m["mu11"], m["mu20"] - m["mu02"])
    l1 = (m["mu20"] + m["mu02"]) / 2 + math.hypot((m["mu20"] - m["mu02"]) / 2, m["mu11"]); l2 = m["mu20"] + m["mu02"] - l1
    best = None
    for ax, var in ((th, l1), (th + math.pi / 2, l2)):
        c, s = math.cos(ax), math.sin(ax)
        m3 = m["mu30"] * c**3 + 3 * m["mu21"] * c**2 * s + 3 * m["mu12"] * c * s**2 + m["mu03"] * s**3
        k = m3 / (m["m00"] * max(var / m["m00"], 1e-9) ** 1.5)
        if best is None or abs(k) > abs(best[1]):
            best = (ax if k > 0 else ax + math.pi, k)
    return best[0], l1 / max(l2, 1e-9)

if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 90
    calib = ad.load_calib(); arena = ad.Arena(calib)
    cap = ad.open_camera(calib.get("camera", 0)); old, new, el, pos = [], [], [], []
    for i in range(N + 10):
        ok, f = cap.read()
        if i < 10 or not ok: continue
        fd = ad.find_robot_color(f, arena)          # no prev: the raw per-frame choice (fd[3]: now = moments)
        if not fd: continue
        px = fd[2]
        floor = arena.lift(cv2.perspectiveTransform(px.reshape(1, -1, 2), arena.img2floor)[0], ad.TAG_HEIGHT_MM)
        d, e = moment_dir(floor); old.append(fd[3]); new.append(d); el.append(e); pos.append(fd[0])
    pass  # (release can hang on this webcam: let the process exit free it)
    def spread(a):
        a = np.array(a); med = math.atan2(np.median(np.sin(a)), np.median(np.cos(a)))
        dev = np.degrees(np.abs(ad.wrap(a - med))); return med, dev
    p = np.mean(pos, 0); print(f"robot mark at official ({p[0]/10:.0f}, {300 - p[1]/10:.0f}) cm, {len(old)} frames")
    for name, a in (("corners (old)", old), ("moments (new)", new)):
        med, dev = spread(a)
        print(f"{name}: median {math.degrees(med):6.1f} deg; frames > 30 deg off: {np.mean(dev > 30):.0%}; spread of the rest (90%) {np.percentile(dev[dev <= 30], 90):.1f} deg")
    print(f"elongation (long/short axis variance) median {np.median(el):.2f}; new - old (median) {math.degrees(ad.wrap(spread(new)[0] - spread(old)[0])):+.1f} deg")
