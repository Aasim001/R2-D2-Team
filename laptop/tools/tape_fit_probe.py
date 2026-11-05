"""How well does an arena map fit the black block tape? For each block edge, look across it in a top-down
view and find the dark tape line: its distance from where the map puts the edge. usage: python tools/tape_fit_probe.py"""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad          # (first: it sets the webcam option OpenCV must see before loading)
import numpy as np, cv2, arena_spec as spec

MM = 5                                                      # top-down: 1 px = 5 mm

def tape_offsets(frame, corners):
    """Signed distance (mm, + = into the block) of the darkest line across each block edge, sampled every
    5 cm, skipping the bay openings. Returns an array of offsets (nan where nothing dark was found)."""
    W = ad.ARENA_MM; n = W // MM
    H = cv2.getPerspectiveTransform(np.float32(corners), np.float32([[0, 0], [n, 0], [n, n], [0, n]]))
    gray = cv2.GaussianBlur(cv2.cvtColor(cv2.warpPerspective(frame, H, (n, n)), cv2.COLOR_BGR2GRAY), (3, 3), 0).astype(np.float32)
    bays = [ad._rect_to_map(b) for b in spec.drivable_rects()]
    out = []
    for x0, y0, x1, y1 in (ad._rect_to_map(r) for r in spec.keepout_rects()):
        for (ax, ay), (bx, by), (ix, iy) in (((x0, y0), (x1, y0), (0, 1)), ((x0, y1), (x1, y1), (0, -1)),
                                             ((x0, y0), (x0, y1), (1, 0)), ((x1, y0), (x1, y1), (-1, 0))):
            L = max(abs(bx - ax), abs(by - ay))
            for s in np.arange(60, L - 59, 50):
                px, py = ax + (bx - ax) * s / L, ay + (by - ay) * s / L
                if any(b[0] - 30 <= px <= b[2] + 30 and b[1] - 30 <= py <= b[3] + 30 for b in bays):
                    out.append(np.nan); continue
                d = np.arange(-60, 61, 2.5)                     # mm across the edge, + = into the block
                xs, ys = (px + ix * d) / MM, (py + iy * d) / MM
                prof = cv2.remap(gray, xs.astype(np.float32)[None], ys.astype(np.float32)[None], cv2.INTER_LINEAR)[0]
                k = int(np.argmin(prof))
                out.append(d[k] if prof[k] < 0.6 * np.median(prof) else np.nan)
    return np.array(out)

if __name__ == "__main__":
    calib = ad.load_calib(); ref = np.array(calib["corners_px"])
    f = cv2.imread(os.path.join(HERE, "empty_arena_2026-10-08.png"))
    for name, c in [("saved map", ref)] + [(f"shifted {dx:+d},{dy:+d} px", ref + [dx, dy]) for dx, dy in ((4, 0), (8, 0), (0, 8), (12, 0), (0, 12), (-12, -12))] + \
                   [("one corner +12 px", ref + [[12, 0], [0, 0], [0, 0], [0, 0]])]:
        o = tape_offsets(f, c); g = o[~np.isnan(o)]
        print(f"{name:20s} tape found at {len(g)}/{len(o)} points; offset median {np.median(g):5.1f} mm, "
              f"|offset - 7.5| median {np.median(np.abs(g - 7.5)):5.1f}  90% {np.percentile(np.abs(g - 7.5), 90):5.1f} mm")
