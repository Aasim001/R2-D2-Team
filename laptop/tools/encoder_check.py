"""Encoder check (moves the robot ~5 cm forward and back, straight): wheel count signs, counts per mm
(camera vs encoders). usage: python tools/encoder_check.py [ROBOT_IP] [PWM] [SECONDS]"""
import os, sys, json, math, socket, struct, threading, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.dirname(HERE))
import arena_drive as ad
import numpy as np

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.43.93"
pwm = int(sys.argv[2]) if len(sys.argv) > 2 else 115
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 0.4
calib = json.load(open(ad.CALIB_FILE)); arena = ad.Arena(calib)
cam = ad.Camera(calib.get("camera", 0))
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sock.settimeout(0.2); addr = (ip, 4210)
tele, stop = [], threading.Event()
def rx():
    while not stop.is_set():
        try:
            d = sock.recvfrom(2048)[0].decode()
            if d.startswith("{"): tele.append((time.time(), json.loads(d)))
        except (socket.timeout, ValueError, ConnectionResetError): pass
sock.sendto(b"tele on", addr); threading.Thread(target=rx, daemon=True).start()
seq = [0]
def drive(l, r, dur):
    t = time.time()
    while time.time() - t < dur:
        seq[0] = (seq[0] + 1) & 0xFFFF; sock.sendto(b"DR" + struct.pack("<hhH", l, r, seq[0]), addr); time.sleep(0.05)
def mark(n=15):
    pts, last = [], 0
    while len(pts) < n:
        f, t = cam.latest()
        if f is None or t == last: time.sleep(0.01); continue
        last = t; fd = ad.find_robot(f, None, arena)
        if fd: pts.append(fd[0])
    return np.mean(pts, 0)
while cam.latest()[0] is None: time.sleep(0.1)
time.sleep(1.0)
c0 = tele[-1][1]["cnt"] if tele else None; p0 = mark()
drive(pwm, pwm, secs); drive(0, 0, 0.8)
c1 = tele[-1][1]["cnt"]; p1 = mark()
drive(-pwm, -pwm, secs); drive(0, 0, 0.8)
c2 = tele[-1][1]["cnt"]; p2 = mark()
sock.sendto(b"tele off", addr); stop.set(); cam.running = False
d1, d2 = np.array(c1) - np.array(c0), np.array(c2) - np.array(c1)
m1, m2 = np.hypot(*(p1 - p0)), np.hypot(*(p2 - p1))
print(f"telemetry packets: {len(tele)}")
print("forward counts per wheel FL FR RL RR:", d1.tolist(), f"  camera moved {m1:.0f} mm")
print("back    counts per wheel FL FR RL RR:", d2.tolist(), f"  camera moved {m2:.0f} mm")
good = [i for i in range(4) if abs(d1[i]) > 5]
if good and m1 > 10:
    print(f"counts per mm (mean of |fwd| wheels): {np.mean(np.abs(d1[good])) / m1:.2f};  back: {np.mean(np.abs(d2[good])) / max(m2, 1):.2f}")
print("signs OK (fwd +, back -):", [bool(d1[i] > 0 and d2[i] < 0) for i in range(4)])
