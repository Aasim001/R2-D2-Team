"""EXPERIMENT (unfinished): score how well an arena map fits the picture = share of points along the
official black block outlines that land on dark tape. Meant to confirm a live re-map before adopting
it. First try (2026-10-07) was inconclusive: people and fresh cut-out tape were in the arena, and even
good maps scored only 0.2-0.5 - needs tuning on an EMPTY arena (and the cut-out layout once CUTOUTS
is on). usage: python tools/tape_score_probe.py  (opens the camera, saves tools/frame_now.png)"""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))           # laptop/
import arena_drive as ad
import cv2

def tape_score(frame, corners, grow=1):
    W = H = ad.ARENA_MM
    G = cv2.getPerspectiveTransform(np.float32([[0, 0], [W, 0], [W, H], [0, H]]), np.float32(corners))
    blocks, _ = ad.layout_rects(W, H)
    pts = []
    for x0, y0, x1, y1 in blocks:
        for t in np.linspace(0.05, 0.95, 25):
            pts += [(x0 + t * (x1 - x0), y0), (x0 + t * (x1 - x0), y1), (x0, y0 + t * (y1 - y0)), (x1, y0 + t * (y1 - y0))]
    px = np.round(cv2.perspectiveTransform(np.float32(pts)[None], G)[0]).astype(int)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.erode(gray, np.ones((2 * grow + 1, 2 * grow + 1), np.uint8))   # darkest within +-grow px
    h, w = gray.shape
    ok = (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)
    v = gray[px[ok, 1], px[ok, 0]]
    floor = np.median(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)[px[ok, 1], px[ok, 0]])
    return float(np.mean(v < 0.55 * np.median(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))))

if __name__ == "__main__":
    calib = ad.load_calib(); ref = np.array(calib["corners_px"])
    cap = ad.open_camera(0)
    for _ in range(15): cap.read()
    ok, f = cap.read(); cap.release()
    cv2.imwrite(os.path.join(HERE, "frame_now.png"), f)
    fresh, _ = ad.red_boundary_corners(f)
    print("shift fresh vs saved map (px):", np.round(np.linalg.norm(np.array(fresh) - ref, axis=1), 1))
    for name, c in (("fresh map", fresh), ("saved map (12 px old)", ref)):
        print(f"{name:24s} tape score " + "  ".join(f"grow{g}: {tape_score(f, c, g):.2f}" for g in (0, 1, 2)))
    for dx, dy in ((4, 0), (8, 0), (0, 8), (16, 0), (0, 16)):
        c = np.array(fresh) + [dx, dy]
        print(f"fresh shifted ({dx:2d},{dy:2d}) px    tape score " + "  ".join(f"grow{g}: {tape_score(f, c, g):.2f}" for g in (0, 1, 2)))
