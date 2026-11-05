"""Camera relay: ONE process owns the webcam and shares every frame through shared memory, so several
arena_drive.py runs (one per robot) can watch the same arena:  python arena_drive.py --camera shm:arena_cam ...

usage:  python tools/cam_relay.py [CAMERA] [--name arena_cam]      (Ctrl+C or a file runs/coord/relay_stop ends it)

Memory layout (read by arena_drive.Camera._run_shm): uint64 seq (odd while a frame is being written),
float64 frame time, uint32 width, uint32 height, then height x width x 3 BGR bytes.
"""
import os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad                 # (first: it sets the webcam option OpenCV must see before loading)
import numpy as np
from multiprocessing import shared_memory

args = [a for a in sys.argv[1:] if not a.startswith("--")]
name = sys.argv[sys.argv.index("--name") + 1] if "--name" in sys.argv else "arena_cam"
if "--name" in sys.argv:
    args.remove(name)
src = int(args[0]) if args and args[0].isdigit() else (args[0] if args else (ad.load_calib() or {}).get("camera", 0))
stop_file = os.path.join(ad.COORD_DIR, "relay_stop")

shm, n, t0 = None, 0, time.time()
try:
    while not os.path.exists(stop_file):
        t_open = time.time()
        cap = ad.open_camera(src)
        if cap is None:
            print(f"cannot open camera {src} - retrying", flush=True)
            time.sleep(1.0)
            continue
        print(f"camera {src} open ({time.time() - t_open:.1f} s)", flush=True)
        fails = 0
        while fails < 30 and not os.path.exists(stop_file):
            ok, f = cap.read()
            if not ok:
                fails += 1
                time.sleep(0.03)
                continue
            fails, t = 0, time.time()
            h, w = f.shape[:2]
            if shm is None:
                shm = shared_memory.SharedMemory(name=name, create=True, size=32 + h * w * 3)
                seq = np.ndarray((1,), np.uint64, shm.buf, 0)
                ts = np.ndarray((1,), np.float64, shm.buf, 8)
                np.ndarray((2,), np.uint32, shm.buf, 16)[:] = (w, h)
                img = np.ndarray((h, w, 3), np.uint8, shm.buf, 32)
                seq[0] = 0
                print(f"sharing {w}x{h} frames as shm:{name}", flush=True)
            seq[0] += 1                                  # odd: being written
            img[:] = f
            ts[0] = t
            seq[0] += 1                                  # even: complete
            n += 1
            if n % 300 == 0:
                print(f"{n} frames, {n / (time.time() - t0):.1f} fps", flush=True)
        cap.release()
        print("camera dropped - reopening", flush=True)
except KeyboardInterrupt:
    pass
finally:
    if shm is not None:
        shm.close()
        shm.unlink()
    print("relay stopped", flush=True)
