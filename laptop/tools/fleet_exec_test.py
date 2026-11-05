"""Test the plan EXECUTION code of arena_drive (plan_progress / plan_blocked: the action dependency graph) with two
simulated robots that move continuously along their planned cells at random speeds, with random extra pick / drop
time, stopping only when arena_drive says "plan: waiting for ...". Checks: everyone finishes (no deadlock) and how
close the robots ever get (centre to centre).

usage:  python tools/fleet_exec_test.py [RUNS]     (layout of 2026-11-04: robots in P1 / P3, cubes at S3 / S6)
"""
import os, sys, math, random, json, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import arena_drive as ad
import fleet_plan as fp

RUNS = int(sys.argv[1]) if len(sys.argv) > 1 else 200
robots = {10: dict(start="P1", home="P1", heading=math.radians(270)),
          11: dict(start="P3", home="P3", heading=math.radians(90))}
plan = fp.plan_fleet(robots, ["S3", "S6"])
print(fp.describe(plan))
tmp = tempfile.mkdtemp()
ad.COORD_DIR = tmp
for tag, v in plan["robots"].items():
    with open(os.path.join(tmp, f"plan_{tag}.json"), "w") as fh:
        json.dump({"version": 1, "legs": [dict(goal=L["goal"], action=L["action"], nodes=L["nodes"]) for L in v["legs"]],
                   "steps": [[*fp.CELLS[c], a, b] for c, a, b in v["steps"]],
                   "deps": {str(j): [[o, k] for o, k in d] for j, d in v["deps"].items()}}, fh)
plans = {}
for tag in robots:
    ad.ROBOT_TAG_ID = tag
    plans[tag] = ad.load_plan()


def path_of(P):
    pts = [tuple(s[:2]) for s in P["steps"]]
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    return pts, cum


def pos_at(pts, cum, s):
    for i in range(len(pts) - 1):
        if cum[i + 1] >= s:
            f = (s - cum[i]) / max(cum[i + 1] - cum[i], 1e-9)
            return (pts[i][0] + (pts[i + 1][0] - pts[i][0]) * f, pts[i][1] + (pts[i + 1][1] - pts[i][1]) * f)
    return pts[-1]


worst, fails, waits, times = 1e9, 0, [], []
for run in range(RUNS):
    rnd = random.Random(run)
    st = {}
    for tag, P in plans.items():
        pts, cum = path_of(P)
        dwell = {j: (s[3] - s[2]) for j, s in enumerate(P["steps"]) if s[3] - s[2] > 3.0 and j > 0}
        st[tag] = dict(P=P, pts=pts, cum=cum, s=0.0, k=0, v=fp.SPEED_CMS * rnd.uniform(0.6, 1.3),
                       dwell=dwell, done_dwell=set(), until=rnd.uniform(0, 4), waited=0.0)
    t, dt = 0.0, 0.05
    while t < 1500:
        others_files = {tag: dict(tag=tag, progress=r["k"], plan_version=1) for tag, r in st.items()}
        if all(r["k"] >= len(r["pts"]) - 1 and r["s"] >= r["cum"][-1] - 0.1 for r in st.values()):
            break
        for tag, r in st.items():
            if t < r["until"]:
                continue
            xy = pos_at(r["pts"], r["cum"], r["s"])
            r["k"] = ad.plan_progress(r["P"], r["k"], xy)
            j = r["k"]
            if j in r["dwell"] and j not in r["done_dwell"] and math.dist(xy, r["pts"][j]) < 1.0:
                r["done_dwell"].add(j)                     # pick / drop: planned time x random + a random extra
                r["until"] = t + r["dwell"][j] * rnd.uniform(0.7, 1.6) + rnd.uniform(0, 10)
                continue
            others = [v for o, v in others_files.items() if o != tag]
            if ad.plan_blocked(r["P"], r["k"], xy, others):
                r["waited"] += dt
                continue
            target = r["cum"][min(len(r["cum"]) - 1, j + 1)] if j in r["dwell"] and j not in r["done_dwell"] else None
            step = r["v"] * dt
            if j + 1 in r["dwell"] and (j + 1) not in r["done_dwell"]:
                step = min(step, max(0.0, r["cum"][j + 1] - r["s"]))   # (stop exactly on a pick / drop cell)
            r["s"] = min(r["cum"][-1], r["s"] + step)
        (a, b) = st.values()
        worst = min(worst, math.dist(pos_at(a["pts"], a["cum"], a["s"]), pos_at(b["pts"], b["cum"], b["s"])))
        t += dt
    else:
        fails += 1
    times.append(t)
    waits.append(sum(r["waited"] for r in st.values()))
print(f"{RUNS} runs: {fails} deadlocks / timeouts; closest the robots got {worst:.1f} cm (centres; they are 15 cm "
      f"wide); time {min(times):.0f}-{max(times):.0f} s (planned {plan['makespan']:.0f}); waiting {min(waits):.0f}-"
      f"{max(waits):.0f} s")
