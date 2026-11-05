"""Run several robots at once, each with its own arena_drive.py --auto script, sharing the one webcam.

usage:  python tools/fleet_run.py --plan --robot NAME TAG IP [--robot ...]
          THE FLEET PLANNER (fleet_plan.py): the camera finds the robots (each one's bay = its home) and the cubes at
          the stations; the planner picks who fetches what, the routes and who waits for whom; every robot follows
          its plan (arena_drive --plan) and goes back to its own bay.
        python tools/fleet_run.py --robot NAME TAG IP STEPS [--robot ...] [--extra "ARENA_DRIVE OPTIONS"]
  e.g.  --robot bot2 10 172.22.81.93 S6,PICK,DZ,DROP,POST:a_dropped,P3
        --robot bot1 11 172.22.81.218 S3,PICK,WAIT:a_dropped,DZ,DROP,P1

- tools/cam_relay.py owns the webcam; every run reads it as --camera shm:arena_cam.
- The arena is mapped once (arena_drive --auto-setup), then every run starts with --no-auto-setup.
- WAIT:x / POST:x steps make the robots take turns (files in runs/coord/). If one run stops early
  ("--auto: STOPPED"), runs/coord/abort is written and the others stop where they are.
- Everything printed goes to runs/fleet-<date-time>.log, each line tagged with the robot and the time.
"""
import os, sys, time, glob, json, math, threading, subprocess
HERE = os.path.dirname(os.path.abspath(__file__))
LAPTOP = os.path.dirname(HERE)
sys.path.insert(0, LAPTOP)
COORD = os.path.join(LAPTOP, "runs", "coord")
SHM = "arena_cam"
PY = sys.executable

PLAN = "--plan" in sys.argv
robots, extra, a = [], [], [x for x in sys.argv[1:] if x != "--plan"]
carrying, homes, cube_hints, robot_extra = set(), {}, {}, {}   # --plan, carrying on from a stop:
while a:
    if a[0] == "--extra":                      # more arena_drive options for every robot, e.g. "--carrying --no-sync"
        extra, a = a[1].split(), a[2:]
    elif a[0] == "--carrying":                 # --carrying TAG: that robot holds a cube (its first job: drop it in DZ)
        carrying.add(a[1])
        a = a[2:]
    elif a[0] == "--home":                     # --home TAG BAY: where it goes back to (default: the bay it stands in)
        homes[a[1]] = a[2]
        a = a[3:]
    elif a[0] == "--cube":                     # --cube ST:X,Y: a cube the camera can't see (under a robot)
        cube_hints[a[1].split(":")[0].upper()] = a[1]
        a = a[2:]
    elif a[0] == "--robot-extra":              # --robot-extra TAG "ARGS": arena_drive options for one robot
        robot_extra[a[1]] = a[2].split()
        a = a[3:]
    elif a[0] == "--robot" and PLAN:
        robots.append(dict(name=a[1], tag=a[2], ip=a[3], steps=None))
        a = a[4:]
    elif a[0] == "--robot":
        robots.append(dict(name=a[1], tag=a[2], ip=a[3], steps=a[4]))
        a = a[5:]
    else:
        sys.exit(f"unknown argument {a[0]}")
if not robots:
    sys.exit(__doc__)

os.makedirs(COORD, exist_ok=True)
for f in glob.glob(os.path.join(COORD, "*")):
    os.remove(f)
log = open(os.path.join(LAPTOP, "runs", "fleet-" + time.strftime("%Y%m%d-%H%M%S") + ".log"), "w")
T0 = time.time()
lock = threading.Lock()


def say(who, line):
    with lock:
        s = f"[{time.time() - T0:7.1f}] {who:6s} {line}"
        print(s, flush=True)
        log.write(s + "\n")
        log.flush()


def pump(proc, who, on_line=None):
    for raw in proc.stdout:
        line = raw.rstrip()
        say(who, line)
        if on_line:
            on_line(line)


env = dict(os.environ, PYTHONUNBUFFERED="1")
relay = subprocess.Popen([PY, os.path.join(HERE, "cam_relay.py"), "--name", SHM], cwd=LAPTOP, env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
ready = threading.Event()
threading.Thread(target=pump, args=(relay, "relay", lambda l: "sharing" in l and ready.set()), daemon=True).start()
if not ready.wait(90):
    say("fleet", "camera relay did not start - stopping")
    relay.kill()
    sys.exit(1)

for attempt in range(4):                       # (someone walking over the boundary spoils a look: try again)
    mapper = subprocess.run([PY, "arena_drive.py", "--auto-setup", "--camera", f"shm:{SHM}", "--tag", robots[0]["tag"]],
                            cwd=LAPTOP, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
    for line in mapper.stdout.splitlines():
        say("map", line)
    if "Arena mapped" in mapper.stdout:
        break
    say("fleet", f"mapping try {attempt + 1} failed - trying again in 3 s")
    time.sleep(3)
if "Arena mapped" not in mapper.stdout:
    say("fleet", "arena mapping failed - stopping")
    open(os.path.join(COORD, "relay_stop"), "w").close()
    relay.wait(10)
    sys.exit(1)

if PLAN:                                       # where is everything -> the plan
    import fleet_plan
    scene = subprocess.run([PY, os.path.join(HERE, "fleet_scene.py"), "--camera", f"shm:{SHM}",
                            "--tags", ",".join(r["tag"] for r in robots)],
                           cwd=LAPTOP, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=90)
    line = [l for l in scene.stdout.splitlines() if l.startswith("SCENE ")]
    if not line:
        say("fleet", "could not read the scene: " + scene.stdout[-300:])
        open(os.path.join(COORD, "relay_stop"), "w").close()
        sys.exit(1)
    sc = json.loads(line[-1][6:])
    for st_, h_ in cube_hints.items():
        sc["cubes"].setdefault(st_, [float(v) for v in h_.split(":")[1].split(",")])
    missing = [r["name"] for r in robots if r["tag"] not in sc["robots"]]
    if missing or not sc["cubes"]:
        say("fleet", f"not seen: robots {missing}, cubes {list(sc['cubes'])} - stopping")
        open(os.path.join(COORD, "relay_stop"), "w").close()
        sys.exit(1)
    fr = {}
    for r in robots:
        v = sc["robots"][r["tag"]]
        say("fleet", f"{r['name']} (marker {r['tag']}) at ({v['xy'][0]}, {v['xy'][1]}) heading {v['heading']} -> "
                     f"start / home {v['node']}")
        fr[int(r["tag"])] = dict(start=v["node"], home=homes.get(r["tag"], v["node"]), heading=math.radians(v["heading"]),
                                 todo=[("DZ", fleet_plan.DROP_S, "DROP")] if r["tag"] in carrying else [])
        if r["tag"] in carrying:
            robot_extra.setdefault(r["tag"], []).append("--carrying")
        robot_extra.setdefault(r["tag"], []).extend(x for h_ in cube_hints.values() for x in ("--cube-hint", h_))
    say("fleet", f"cubes at {list(sc['cubes'])}")
    t_plan = time.time()
    plan = fleet_plan.plan_fleet(fr, list(sc["cubes"]))
    if plan is None:
        say("fleet", "no plan found - stopping")
        open(os.path.join(COORD, "relay_stop"), "w").close()
        sys.exit(1)
    say("fleet", f"planned in {time.time() - t_plan:.2f} s:")
    for l_ in fleet_plan.describe(plan).splitlines():
        say("plan", l_)
    for r in robots:
        v = plan["robots"][int(r["tag"])]
        with open(os.path.join(COORD, f"plan_{r['tag']}.json"), "w") as fh:
            json.dump({"version": 1, "makespan": plan["makespan"],
                       "legs": [dict(goal=L["goal"], action=L["action"], nodes=L["nodes"]) for L in v["legs"]],
                       "steps": [[*fleet_plan.CELLS[c], round(a_, 2), round(b_, 2)] for c, a_, b_ in v["steps"]],
                       "deps": {str(j): [[o, k] for o, k in d] for j, d in v["deps"].items()}}, fh)
        r["steps"] = ",".join(x for L in v["legs"] for x in ([L["goal"]] + ([L["action"]] if L["action"] else [])))
    extra = extra + ["--plan"]

procs, result = [], {}


def watcher(r):
    def on_line(line):
        if "--auto: STOPPED" in line:
            result[r["name"]] = line
            open(os.path.join(COORD, "abort"), "w").close()
            say("fleet", f"{r['name']} stopped early -> abort the others")
        elif "--auto: all steps done" in line:
            result[r["name"]] = line
    return on_line


for i, r in enumerate(robots):                 # (the first --robot has the right of way where routes cross)
    cmd = [PY, "arena_drive.py", "--no-auto-setup", "--record", "--camera", f"shm:{SHM}", "--tag", r["tag"],
           "--car", r["ip"], "--auto", r["steps"], "--fleet", "--priority", str(i)] + extra + robot_extra.get(r["tag"], [])
    say("fleet", f"start {r['name']} (marker {r['tag']}, {r['ip']}, priority {i}): {r['steps']}")
    p = subprocess.Popen(cmd, cwd=LAPTOP, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    r["t_start"] = time.time()
    th = threading.Thread(target=pump, args=(p, r["name"], watcher(r)), daemon=True)
    th.start()
    procs.append((r, p, th))

for r, p, th in procs:
    p.wait()
    th.join(5)
    r["t_end"] = time.time()
    if r["name"] not in result:
        result[r["name"]] = f"exited (code {p.returncode}) without finishing"
        open(os.path.join(COORD, "abort"), "w").close()

open(os.path.join(COORD, "relay_stop"), "w").close()
try:
    relay.wait(15)
except subprocess.TimeoutExpired:
    relay.kill()
say("fleet", "---- summary ----")
for r, p, th in procs:
    say("fleet", f"{r['name']}: {r['t_end'] - r['t_start']:.1f} s - {result[r['name']]}")
say("fleet", f"all robots done after {max(r['t_end'] for r, _, _ in procs) - min(r['t_start'] for r, _, _ in procs):.1f} s")
log.close()
