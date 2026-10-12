"""Save a zoomed crop of the robot's orange mark + its colour mask + the detected outline (tools/mark_crop.png),
and print the per-frame directions. usage: python tools/mark_crop.py"""
import os, sys, json, math
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad          # (first: it sets the webcam option OpenCV must see before loading)
import numpy as np, cv2
calib = json.load(open(ad.CALIB_FILE)); arena = ad.Arena(calib)
cap = ad.open_camera(calib.get("camera", 0)); hs, f = [], None
for i in range(40):
    ok, fr = cap.read()
    if not ok: continue
    f = fr
    if i >= 10:
        fd = ad.find_robot_color(f, arena)
        if fd: hs.append(math.degrees(fd[1]))
cap.release()
print("per-frame mark directions (deg, map):", " ".join(f"{a:.0f}" for a in hs))
fd = ad.find_robot_color(f, arena)
if not fd: print("robot not found"); sys.exit()
px = fd[2]; x, y, w, h = cv2.boundingRect(px.astype(np.int32)); pad = 40
x0, y0 = max(0, x - pad), max(0, y - pad)
crop = f[y0:y + h + pad, x0:x + w + pad].copy()
mask = cv2.inRange(cv2.cvtColor(crop, cv2.COLOR_BGR2HSV), np.array(ad.MARK_HSV[0]), np.array(ad.MARK_HSV[1]))
out = np.hstack([crop, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
cv2.polylines(out, [(px - [x0, y0]).astype(np.int32).reshape(-1, 1, 2)], True, (255, 0, 255), 1)
cv2.imwrite(os.path.join(HERE, "mark_crop.png"), cv2.resize(out, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
print("mark blob", w, "x", h, "px; mark at map", np.round(fd[0]).tolist())
