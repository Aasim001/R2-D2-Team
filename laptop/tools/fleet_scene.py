"""Where the robots and the cubes are, from the shared camera: one JSON line for tools/fleet_run.py --plan.

usage:  python tools/fleet_scene.py --camera shm:arena_cam --tags 10,11
prints: {"robots": {"10": {"xy": [x, y], "heading": deg, "node": "P1"}, ...}, "cubes": {"S3": [x, y], ...}}
"""
import os, sys, json, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad                 # (first: it sets the webcam option OpenCV must see before loading)
import arena_spec as spec
import numpy as np

src = sys.argv[sys.argv.index("--camera") + 1] if "--camera" in sys.argv else "0"
src = int(src) if src.isdigit() else src
tags = [int(t) for t in sys.argv[sys.argv.index("--tags") + 1].split(",")]
cam = ad.Camera(src)
t0 = time.time()
while cam.latest()[0] is None and time.time() - t0 < 30:
    time.sleep(0.05)
frames, last = [], None
while len(frames) < 6 and time.time() - t0 < 40:
    f, t = cam.latest()
    if f is not None and t != last:
        frames.append(f)
        last = t
    time.sleep(0.03)
cam.running = False
det = ad.make_detector()
out, poses = {"robots": {}, "cubes": {}}, []
for tag in tags:
    ad.ROBOT_TAG_ID = tag
    arena = ad.Arena(ad.load_calib())
    seen = []
    for f in frames:
        r = ad.find_robot(f, det, arena)
        if r is None:
            continue
        h = r[1] + arena.heading_offset
        h += ad.heading_trim(arena, h)
        tp = np.array(ad.mark_to_centre(r[0], h, arena.turn_centre))
        seen.append((tp, h, ad.to_official(tp, h)))
    if not seen:
        continue
    x, y = float(np.median([s[2][0] for s in seen])), float(np.median([s[2][1] for s in seen]))
    hd = float(np.median([s[2][2] for s in seen]))
    bay = spec.bay_of(x, y)
    out["robots"][str(tag)] = dict(xy=[round(x, 1), round(y, 1)], heading=round(hd, 1),
                                   node=bay if bay else spec.nearest_node(x, y))
    poses.append((seen[-1][0], seen[-1][1]))
ad.OTHER_POSES[:] = poses                                   # (no robot counts as a cube)
arena = ad.Arena(ad.load_calib())
for st in spec.STATIONS:                                    # (every frame: in weak light / glare a cube is only
    found = [c for c in (ad.find_cube(f, arena, st) for f in frames) if c is not None]   # seen in some of them)
    if len(found) >= 2:
        out["cubes"][st] = [round(float(np.median([c[0] for c in found])), 1),
                            round(float(np.median([c[1] for c in found])), 1)]
print("SCENE " + json.dumps(out), flush=True)
