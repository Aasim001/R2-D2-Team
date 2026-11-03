"""Measure each N20 wheel on its own: the lowest PWM that starts it, and its steady speed (encoder counts/s) at
several PWMs, forward and backward. Prints a table and the per-wheel gain that would make all four match.

usage:  python tools/wheel_calibrate.py ROBOT_IP [--levels 80,120,160,200,255] [--save FILE.json]
        python tools/wheel_calibrate.py ROBOT_IP --moves [--levels 120,180,255]
          (--moves: all four wheels together - FORWARD, BACK, RIGHT, LEFT - with the straight assist off; shows
           each wheel's speed and the left/right balance per movement)

LIFT THE ROBOT so all four wheels spin freely in the air (one wheel driving on the floor drags the robot round),
battery switch on. Uses the firmware's single-wheel test ("wheel fl 150 1500"); the claw is not touched.
"""
import sys, json, time, socket, struct, argparse

NAMES = ["fl", "fr", "rl", "rr"]
ap = argparse.ArgumentParser()
ap.add_argument("ip")
ap.add_argument("--levels", default="80,120,160,200,255")
ap.add_argument("--save")
ap.add_argument("--moves", action="store_true")
a = ap.parse_args()
if a.moves and a.levels == "80,120,160,200,255":
    a.levels = "120,180,255"
LEVELS = [int(x) for x in a.levels.split(",")]
addr = (a.ip, 4210)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)


def ask(c):
    for _ in range(4):
        sock.sendto(c.encode(), addr)
        try:
            while True:
                d = sock.recvfrom(4096)[0].decode(errors="replace")
                if c != "status" or d.startswith("{"):
                    return d
        except (socket.timeout, ConnectionResetError):
            pass
    sys.exit(f"no answer from the robot at {a.ip} to '{c}'")


def counts():
    return json.loads(ask("status"))["cnt"]


enc_of = {}                       # motor index -> encoder index that actually counts for it (wiring check)


def run(i, pwm, ms=1500, t_a=0.6, t_b=1.3):
    """steady rate of motor i (counts/s, signed) between t_a and t_b after the start, read on whichever encoder
    moves - so a motor whose encoder is wired to another wheel's pins is still measured (and reported)"""
    r = ask(f"wheel {NAMES[i]} {pwm} {ms}")
    if not r.startswith("ok"):
        sys.exit(f"robot refused 'wheel {NAMES[i]} {pwm} {ms}': {r}")
    t0 = time.time()
    time.sleep(t_a)
    c0, ta = counts(), time.time()
    time.sleep(max(0.0, t_b - (time.time() - t0)))
    c1, tb = counts(), time.time()
    time.sleep(max(0.0, ms / 1000 - (time.time() - t0)) + 0.5)       # let it stop
    d = [c1[k] - c0[k] for k in range(4)]
    k = max(range(4), key=lambda k: abs(d[k]))
    if abs(d[k]) > 20:
        enc_of[i] = k
    return d[enc_of.get(i, i)] / (tb - ta)


def start_pwm(i, sign):
    for p in range(30, 161, 10):
        r = run(i, sign * p, ms=700, t_a=0.3, t_b=0.6)
        if abs(r) > 40:
            return p
    return None


def find_map():
    """motor -> encoder index, from a short run of each motor alone"""
    for i in range(4):
        run(i, 150, ms=700, t_a=0.3, t_b=0.6)
    return {i: enc_of.get(i, i) for i in range(4)}


def drive_rates(left, right, secs=1.6, t_a=0.7, t_b=1.4):
    """all four wheels (left side = left, right side = right); counts/s per encoder between t_a and t_b"""
    seq, t0, c0 = 0, time.time(), None
    while time.time() - t0 < secs:
        seq = (seq + 1) & 0xFFFF
        sock.sendto(b"DR" + struct.pack("<hhH", left, right, seq), addr)
        el = time.time() - t0
        if c0 is None and el >= t_a:
            c0, ta = counts(), time.time()
        elif c0 is not None and el >= t_b:
            c1, tb = counts(), time.time()
            break
        time.sleep(0.04)
    for _ in range(3):
        seq = (seq + 1) & 0xFFFF
        sock.sendto(b"DR" + struct.pack("<hhH", 0, 0, seq), addr)
        time.sleep(0.02)
    time.sleep(0.6)
    return [(c1[k] - c0[k]) / (tb - ta) for k in range(4)]


def moves():
    ask("assist off")
    m = find_map()
    cross = {NAMES[i].upper(): NAMES[k].upper() for i, k in m.items() if k != i}
    print("motor -> encoder:", cross or "all straight (as wired in docs/wiring.md)")
    out = {}
    for name, sl, sr in (("FORWARD", 1, 1), ("BACK", -1, -1), ("RIGHT", 1, -1), ("LEFT", -1, 1)):
        for p in LEVELS:
            r = drive_rates(sl * p, sr * p)
            spd = [abs(r[m[i]]) for i in range(4)]                  # per MOTOR
            want = [sl, sr, sl, sr]
            dirs_ok = all((r[m[i]] * want[i] * (-1 if i % 2 == 0 else 1)) > 0 for i in range(4))
            L, R = (spd[0] + spd[2]) / 2, (spd[1] + spd[3]) / 2
            avg = sum(spd) / 4 or 1
            spread = (max(spd) - min(spd)) / avg * 100
            out[f"{name} {p}"] = {"speed": spd, "L": L, "R": R}
            print(f"  {name:7s} {p:3d}: " + "  ".join(f"{NAMES[i].upper()} {spd[i]:5.0f}" for i in range(4))
                  + f"   left {L:5.0f} right {R:5.0f} (L/R {L / R if R else 0:.3f})   spread {spread:4.1f}%"
                  + ("" if dirs_ok else "   !! a wheel turned the wrong way"), flush=True)
    print("\nreading it: L/R = 1.000 means the two sides match (drives straight / turns on the spot);"
          " spread = fastest vs slowest wheel. Wheels in the air: on the floor the load changes this a little.")
    for name in ("FORWARD", "BACK", "RIGHT", "LEFT"):
        lr = [out[f"{name} {p}"]["L"] / out[f"{name} {p}"]["R"] for p in LEVELS if out[f"{name} {p}"]["R"]]
        print(f"  {name:7s}: mean L/R {sum(lr) / len(lr):.3f}")
    gains = {}
    for i in range(4):
        rel = [out[k]["speed"][i] / (sum(out[k]["speed"]) / 4) for k in out if sum(out[k]["speed"])]
        gains[NAMES[i]] = sum(rel) / len(rel)
    slow = min(gains.values())
    print("  per-motor speed vs the average over all movements: "
          + "  ".join(f"{n.upper()} {g:.3f}" for n, g in gains.items())
          + "\n  matching gain (scale the PWM by this): "
          + "  ".join(f"{n.upper()} {slow / g:.3f}" for n, g in gains.items()))
    return {"map": m, "moves": out, "gain": {n: slow / g for n, g in gains.items()}}


ask("stop")
if a.moves:
    try:
        r = moves()
    finally:
        ask("stop")
    if a.save:
        json.dump({"ip": a.ip, "levels": LEVELS, **r, "time": time.strftime("%F %T")}, open(a.save, "w"), indent=1)
        print("saved", a.save)
    sys.exit(0)
st = json.loads(ask("status"))
print(f"robot at {a.ip}: battery reading {st['batt']:.2f} V, encoders ok {st['ok']}")
res = {n: {"fwd": {}, "back": {}, "start_fwd": None, "start_back": None} for n in NAMES}
try:
    for i, n in enumerate(NAMES):
        for sign, d in ((1, "fwd"), (-1, "back")):
            res[n]["start_" + d] = start_pwm(i, sign)
            for p in LEVELS:
                res[n][d][p] = run(i, sign * p)
            print(f"  {n.upper()} {d:4s}: starts at {res[n]['start_' + d]}, "
                  + "  ".join(f"{p}:{res[n][d][p]:7.0f}" for p in LEVELS), flush=True)
finally:
    ask("stop")

print("\nsteady speed, counts/s (sign = encoder direction)")
for d in ("fwd", "back"):
    print(f"{d:5s} " + "".join(f"{p:>8d}" for p in LEVELS))
    for n in NAMES:
        print(f"  {n.upper():3s} " + "".join(f"{res[n][d][p]:8.0f}" for p in LEVELS))

print("\nper-wheel gain to match the slowest wheel (mean over the levels >= 120; 1.00 = slowest):")
gains = {}
for d in ("fwd", "back"):
    use = [p for p in LEVELS if p >= 120] or LEVELS
    spd = {n: sum(abs(res[n][d][p]) for p in use) / len(use) for n in NAMES}
    slow = min(spd.values()) or 1
    gains[d] = {n: slow / spd[n] if spd[n] else 0 for n in NAMES}
    print(f"  {d:5s} " + "  ".join(f"{n.upper()} {gains[d][n]:.2f} ({spd[n]:.0f}/s)" for n in NAMES))
crossed = {NAMES[i].upper(): NAMES[k].upper() for i, k in enc_of.items() if k != i}
if crossed:
    print("  WIRING: these motors are counted by another wheel's encoder pins (motor -> encoder):", crossed,
          "- fix the wiring (docs/wiring.md) before trusting the straight assist")
signs = {n: [1 if res[n][d][LEVELS[-1]] > 0 else -1 for d in ("fwd", "back")] for n in NAMES}
odd = [n.upper() for n in NAMES if signs[n] != [1, -1]]
if odd:
    print("  note: these wheels' encoders count backwards for forward PWM (only matters for display):", odd)
if a.save:
    json.dump({"ip": a.ip, "levels": LEVELS, "result": res, "gain": gains, "time": time.strftime("%F %T")},
              open(a.save, "w"), indent=1)
    print("saved", a.save)
