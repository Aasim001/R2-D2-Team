"""Multi-robot simulation of the fleet coordinator (fleet.py) on the official floor.

Each simulated robot has the same dynamics as lane_follow.simulate(): camera/Wi-Fi lag, the firmware's speed-up
ramp, the N20 wheel model, pose noise - plus per-robot speed differences, random stalls (a turn that needs retries,
a Wi-Fi hiccup) and random QR-look times. The REAL Follower and the REAL Coordinator run every frame.
Checked every frame: the robots' true outlines (car1's claw included) never overlap -> zero collisions;
every robot keeps making progress -> no deadlock; every job gets done.

  python fleet_sim.py                    # the standard 2-robot scenarios + 200 random 2-robot runs
  python fleet_sim.py --robots 3 --runs 200
  python fleet_sim.py --scenario swap -v # one scenario, with the coordinator's log
"""
import argparse, math, random, sys, time
import arena_spec as spec
import lane_follow
import fleet

DT, LAG, TRACK, RAMP = 1 / 30, 4, 12.0, 1200
BODY = (13.0, 14.5)                       # length, width (cm)
CLAW_CM = 10.0                            # car1: the claw sticks out this far in front
START = {"car1": ((50, 265), 270), "car2": ((250, 265), 270), "car3": ((210, 35), 90)}   # parked in P1 / P2 / P3
HOME = {"car1": "P1", "car2": "P2", "car3": "P3"}


def wheel(pwm):
    return 0.0 if abs(pwm) < 80 else math.copysign((abs(pwm) - 70) * 0.30, pwm)


def approach(cur, target, up):
    if target == 0:
        return 0.0
    if cur * target < 0:
        cur = 0.0
    if abs(target) < abs(cur):
        return target
    return cur + max(-up, min(up, target - cur))


class SimRobot:
    def __init__(self, name, rnd, claw=False):
        (x, y), h = START[name]
        h += 180 if rnd.random() < 0.5 else 0             # parked facing into OR out of its bay (both happen)
        x, y = x + rnd.uniform(-1.5, 1.5), y + rnd.uniform(-2.5, 2.5)   # ... and a little off the bay point
        self.name, self.x, self.y, self.h = name, float(x), float(y), math.radians(h + rnd.uniform(-5, 5))
        self.rnd, self.queue, self.fw = rnd, [(0, 0)] * LAG, [0.0, 0.0]
        self.speed = rnd.uniform(0.75, 1.15)               # this robot's motors
        self.stall_until, self.act_until = 0.0, None
        self.front = BODY[0] / 2 + (CLAW_CM if claw else 0.0)
        self.back = BODY[0] / 2

    def pose(self, noise=0.3):
        r = self.rnd
        return (self.x + r.gauss(0, noise), self.y + r.gauss(0, noise), math.degrees(self.h) + r.gauss(0, 0.5))

    def move(self, l, r, t):
        self.queue.append((l, r))
        l, r = self.queue.pop(0)
        if t < self.stall_until:                           # a stall: the wheels don't turn
            l = r = 0
        elif (l or r) and self.rnd.random() < 0.0011:      # ~ every 30 s of driving: a 1-4 s hiccup (a turn that
            self.stall_until = t + self.rnd.uniform(1, 4)  # needs a retry, a Wi-Fi drop-out, a weak wheel)
        self.fw = [approach(self.fw[0], l, RAMP * DT), approach(self.fw[1], r, RAMP * DT)]
        vl = wheel(self.fw[0]) * self.speed * self.rnd.gauss(1, 0.03)
        vr = wheel(self.fw[1]) * self.speed * self.rnd.gauss(1, 0.03)
        self.h += (vr - vl) / TRACK * DT
        self.x += (vl + vr) / 2 * DT * math.cos(self.h)
        self.y += (vl + vr) / 2 * DT * math.sin(self.h)

    def outline(self):
        c, s = math.cos(self.h), math.sin(self.h)
        w = BODY[1] / 2
        return [(self.x + a * c - b * s, self.y + a * s + b * c)
                for a, b in ((self.front, w), (self.front, -w), (-self.back, -w), (-self.back, w))]


def overlap(p, q):
    """Do two convex polygons overlap? (separating axis test)"""
    for poly in (p, q):
        for i in range(len(poly)):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
            nx, ny = y2 - y1, x1 - x2
            a = [nx * x + ny * y for x, y in p]
            b = [nx * x + ny * y for x, y in q]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


def gap(p, q):
    """Smallest corner-to-corner distance (cm), a cheap closeness measure."""
    return min(math.hypot(a[0] - b[0], a[1] - b[1]) for a in p for b in q)


def run(jobs, seed=1, verbose=False, limit_s=600):
    """jobs: {robot: [(goal, action), ...]}. Returns a result dict."""
    rnd = random.Random(seed)
    sims = {n: SimRobot(n, rnd, claw=(n == "car1")) for n in jobs}
    robots = [fleet.FleetRobot(n, j, park=HOME[n]) for n, j in jobs.items()]
    coord = fleet.Coordinator(robots, log=print if verbose else None)
    for n, s in sims.items():
        coord.place(n, s.x, s.y, 0.0)
    t, collisions, min_gap, last_progress = 0.0, 0, 1e9, 0.0
    progress = {n: (s.x, s.y, 0) for n, s in sims.items()}
    deadlock = None
    while t < limit_s:
        poses = {n: s.pose() for n, s in sims.items()}
        cmds = coord.step(t, poses)
        coord.check_invariants()
        for n, s in sims.items():
            l, r, st = cmds[n]
            s.move(l, r, t)
            fr = coord.robots[n]
            if fr.state == "ACT":                          # QR look / drop: timed here
                if s.act_until is None:
                    s.act_until = t + (rnd.uniform(10, 20) if fr.action == "qr" else rnd.uniform(2, 4))
                elif t >= s.act_until:
                    s.act_until = None
                    coord.act_done(n, t)
        outs = {n: s.outline() for n, s in sims.items()}
        names = list(outs)
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                g = gap(outs[names[i]], outs[names[j]])
                min_gap = min(min_gap, g)
                if overlap(outs[names[i]], outs[names[j]]):
                    collisions += 1
                    if verbose and collisions == 1:
                        print(f"  COLLISION at {t:.1f}s between {names[i]} and {names[j]}")
        for n, s in sims.items():                           # progress = moved 3 cm or finished a job
            px, py, nd = progress[n]
            k = len(coord.robots[n].done_jobs)
            if math.hypot(s.x - px, s.y - py) > 3 or k != nd:
                progress[n] = (s.x, s.y, k)
                last_progress = t
        if coord.all_done() or coord.failed():
            break
        if t - last_progress > 90:
            deadlock = {n: (r.state, r.goal, r.node) for n, r in coord.robots.items()}
            break
        t += DT
    return dict(time=round(t, 1), collisions=collisions, min_gap=round(min_gap, 1),
                done=coord.all_done(), failed=coord.failed(), deadlock=deadlock,
                jobs={n: r.done_jobs for n, r in coord.robots.items()},
                left={n: r.jobs for n, r in coord.robots.items()})


SCENARIOS = {   # (name: jobs) - 2 robots, starting parked in P1 (car1) and P2 (car2)
    "both_to_dz": {"car1": [("S6", "qr"), ("DZ", "drop")], "car2": [("S5", "qr"), ("DZ", "drop")]},
    "swap":       {"car1": [("S4", "qr"), ("DZ", "drop")], "car2": [("S6", "qr"), ("DZ", "drop")]},
    "same_lane":  {"car1": [("S1", "qr"), ("DZ", "drop")], "car2": [("S2", "qr"), ("DZ", "drop")]},
    "cross":      {"car1": [("S2", "qr"), ("DZ", "drop")], "car2": [("S3", "qr"), ("DZ", "drop")]},
    "near_dz":    {"car1": [("S3", "qr"), ("DZ", "drop")], "car2": [("S6", "qr"), ("DZ", "drop")]},
}


def random_jobs(rnd, names):
    st = list(spec.STATIONS)
    rnd.shuffle(st)
    return {n: [(st[i], "qr"), ("DZ", "drop")] for i, n in enumerate(names)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=2, choices=(2, 3))
    ap.add_argument("--runs", type=int, default=200, help="random runs (0 = scenarios only)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--scenario", help="run only this scenario: " + ", ".join(SCENARIOS))
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    names = ["car1", "car2", "car3"][:a.robots]
    bad = []
    if a.robots == 2 and a.runs >= 0:
        for sc, jobs in SCENARIOS.items():
            if a.scenario and sc != a.scenario:
                continue
            t0 = time.time()
            res = run(jobs, seed=a.seed, verbose=a.verbose)
            ok = res["done"] and not res["collisions"] and not res["deadlock"] and not res["failed"]
            print(f"{sc:11s}: {'OK  ' if ok else 'FAIL'} {res['time']:6.1f} s sim, collisions {res['collisions']}, "
                  f"closest {res['min_gap']} cm, deadlock {res['deadlock']}, failed {res['failed']}  "
                  f"({time.time() - t0:.1f} s real)")
            if not ok:
                bad.append((sc, res))
    if a.scenario:
        return
    rnd = random.Random(a.seed)
    stats = dict(ok=0, coll=0, dead=0, fail=0, notdone=0, times=[], gaps=[])
    t0 = time.time()
    for i in range(a.runs):
        jobs = random_jobs(rnd, names)
        res = run(jobs, seed=a.seed * 1000 + i)
        stats["times"].append(res["time"])
        stats["gaps"].append(res["min_gap"])
        problem = None
        if res["collisions"]:
            stats["coll"] += 1
            problem = "collision"
        elif res["deadlock"]:
            stats["dead"] += 1
            problem = "deadlock"
        elif res["failed"]:
            stats["fail"] += 1
            problem = "failed"
        elif not res["done"]:
            stats["notdone"] += 1
            problem = "not done"
        else:
            stats["ok"] += 1
        if problem:
            bad.append((f"random {i} {problem}", dict(res, given=jobs, seed=a.seed * 1000 + i)))
        if (i + 1) % 20 == 0:
            print(f"  {i + 1}/{a.runs} random runs: {stats['ok']} ok, {stats['coll']} collision, "
                  f"{stats['dead']} deadlock, {stats['fail']} failed, {stats['notdone']} unfinished "
                  f"({time.time() - t0:.0f} s)", flush=True)
    if a.runs:
        ts = sorted(stats["times"])
        print(f"RANDOM {a.robots} robots x {a.runs}: {stats['ok']} ok | collisions {stats['coll']} | deadlocks "
              f"{stats['dead']} | failed {stats['fail']} | unfinished {stats['notdone']} | time median "
              f"{ts[len(ts) // 2]:.0f} s, max {ts[-1]:.0f} s | closest robots {min(stats['gaps']):.1f} cm")
    for name, res in bad[:8]:
        print("PROBLEM", name, {k: res.get(k) for k in ("given", "seed", "time", "collisions", "deadlock", "failed", "left")})
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
