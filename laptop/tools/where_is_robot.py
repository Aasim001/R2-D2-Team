"""Quick look (no driving): where the camera sees the robot now, and has the camera moved since the
arena was mapped? Saves the frame as tools/now.jpg.   usage: python tools/where_is_robot.py"""
import sys, os, json, math, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))           # laptop/
import arena_drive as ad          # (sets the camera env var before cv2)
import cv2, numpy as np
calib = ad.load_calib(); arena = ad.Arena(calib)
cap = ad.open_camera(calib.get("camera", 0))
if cap is None: print("camera not available"); sys.exit(1)
poses, prev, shift = [], None, None
for i in range(40):
    ok, f = cap.read()
    if not ok: continue
    if i == 20:
        c, _ = ad.red_boundary_corners(f)
        if c is not None: shift = max(np.hypot(*(np.array(a) - b)) for a, b in zip(arena.corners, c))
    fd = ad.find_robot(f, None, arena, None, prev)
    if fd:
        prev = fd[1]; h = fd[1] + arena.heading_offset
        h += ad.heading_trim(arena, h)
        poses.append(ad.to_official(ad.mark_to_centre(fd[0], h, arena.turn_centre), h))
cap.release()
print(f"camera shift since mapping: {shift:.1f} px" if shift is not None else "boundary not found")
if poses:
    p = np.array(poses); print(f"robot seen in {len(p)}/40 frames: x {np.median(p[:,0]):.1f} y {np.median(p[:,1]):.1f} cm heading {np.median(p[:,2]):.0f} deg")
    import arena_spec as spec
    print("keep-out depth (cm):", round(ad.keepout_depth((ad.to_map(np.median(p[:,0]), np.median(p[:,1])), -math.radians(np.median(p[:,2])))), 1))
    print("nearest node:", spec.nearest_node(np.median(p[:,0]), np.median(p[:,1])))
else:
    print("robot not seen")
cv2.imwrite(os.path.join(HERE, "now.jpg"), f)
