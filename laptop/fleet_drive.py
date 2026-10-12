"""Two (or three) robots on the official floor AT THE SAME TIME, coordinated by fleet.py, with the overhead webcam.

  python fleet_drive.py --camera 1 --job 1=S6,DZ --job 2=S5,DZ --record
  python fleet_drive.py --job 1=S3,DZ --job 2=S4,P2            # destination can also be a parking bay

--job N=STATION,DEST : robot N (arena_calib_carN.json: its marker id, IP, ESP32-CAM, heading offset, turning point,
turning power) drives to STATION, stops on the docking square with its camera side to the alcove, reads the cube's
QR code (its ESP32-CAM, cam_host), then drives to DEST. After DZ it goes on to its parking bay (P1/P2/P3), so DZ is
free for the next robot. fleet.Coordinator decides who may drive where: whole-route reservations, routes around
standing robots, step-aside when blocked - no collisions, no deadlocks (fleet_sim.py: 0 in 600+ simulated runs).

Before the first fleet run (or after a marker was moved), run for each robot:  arena_drive.py --robot N --auto SYNC
(so its heading offset is right). Robots should start parked in their bays (or standing on a lane node).
Keys: SPACE = pause / go on (every robot stops), Q = quit. --record saves runs/fleet-<time>/ (log.csv, events.txt,
frames, the QR pictures).
"""
import argparse, json, math, os, sys, threading, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import arena_drive as ad                                   # (before cv2: the camera option it sets)
import cv2
import numpy as np
import arena_spec as spec
import lane_follow
import fleet

MAP_KEYS = ("camera", "frame_size", "arena_w", "arena_h", "corners_px", "tape_ranges", "focal_px", "camera_mm",
            "official", "calibrated_by", "tape_edges")      # the arena map (the same for every robot)
LOST_S = 0.5            # a robot not seen this long: its wheels stop (the coordinator holds its place)
JUMP_CM = 15.0          # a pose this far from the last good one (+ 40 cm/s x dt) is ignored for a few frames
TOO_CLOSE_CM = 20.0     # two robots' centres closer than this: EMERGENCY stop everything (tracking mix-up?)
COLOURS = [(0, 200, 255), (255, 120, 0), (0, 220, 0)]
# STALL WATCHDOG (2026-10-10: twice a robot reversing out of its bay got e.g. -127 / -95 - the minimum steering on a
# 1 cm offset puts the slow side at its stall floor - and did not move AT ALL; the steering integral then wound up to
# -160 / -95, the stalled motors drained the battery for minutes and the ESP32 browned out and dropped off the Wi-Fi).
STALL_S = 1.5           # wheels commanded this long without the robot moving STALL_CM / STALL_DEG = stuck
STALL_CM, STALL_DEG = 1.0, 4.0
KICK_S = 0.5            # stuck -> drive straight (both sides the same) this long, then let the follower go on
KICKS_REST = 4          # after this many kicks without progress: wheels off for REST_S (let the battery recover)
REST_S = 3.0
KICKS_FAIL = 8          # ... and after this many: FAILED (wheels off, its place stays held)


class Bot:
    """One real robot: its calibration, UDP link, pose tracking, QR look."""
    def __init__(self, n, jobs, calib, idx):
        self.n, self.name, self.calib, self.colour = n, f"car{n}", calib, COLOURS[idx % 3]
        self.arena = ad.Arena(calib, load_map=False)
        if self.arena.heading_offset is None:
            sys.exit(f"car{n} has no heading offset yet - run: python arena_drive.py --robot {n} --auto SYNC")
        self.tag, self.tag_mm = int(calib["tag_id"]), float(calib.get("tag_height_mm") or ad.TAG_HEIGHT_MM)
        self.car = ad.Car(calib["host"], ad.CAR_PORT)
        self.min_pwm = int(calib.get("min_pwm") or lane_follow.MIN_PWM)
        params = {k: int(calib[c]) for k, c in (("PIVOT_PWM", "pivot_pwm"), ("MIN_PWM", "min_pwm")) if calib.get(c)}
        self.fr = fleet.FleetRobot(self.name, jobs, park=f"P{n}" if n in (1, 2, 3) else None, params=params)
        self.fr.follower.turn_sign = self.arena.turn_sign
        self.pose, self.seen_t, self.jumps = None, 0.0, 0
        self.qr, self.qr_text, self.act_t = None, None, None
        self.cmd, self.offline, self.send_err = (0, 0), False, 0
        self.stall_t0, self.stall_p0, self.kicks, self.kick_until, self.kick_cmd, self.rest_until =             None, None, 0, 0.0, (0, 0), 0.0

    def watchdog(self, t, cmd):
        """Stall watchdog: returns the wheel command to send (cmd, or a straight kick, or 0 while resting)."""
        if t < self.rest_until:
            return (0, 0)
        if t < self.kick_until:
            return self.kick_cmd
        moving_cmd = cmd != (0, 0) and self.fr.state == "MOVE" and self.pose is not None
        if not moving_cmd:
            self.stall_t0 = None                            # (settling / waiting: not expected to move)
            return cmd
        if self.stall_t0 is None:
            self.stall_t0, self.stall_p0 = t, self.pose
            return cmd
        x0, y0, h0 = self.stall_p0
        x, y, h = self.pose
        if math.hypot(x - x0, y - y0) > STALL_CM or abs((h - h0 + 180) % 360 - 180) > STALL_DEG:
            self.stall_t0, self.stall_p0, self.kicks = t, self.pose, 0        # it moves: all good
            return cmd
        if t - self.stall_t0 < STALL_S:
            return cmd
        self.kicks += 1                                      # stuck
        f = self.fr.follower
        f.integ = 0.0                                        # (wound up while it couldn't move)
        f.imbalance = {False: 0.0, True: 0.0}
        self.stall_t0, self.stall_p0 = t, self.pose
        if self.kicks > KICKS_FAIL:
            self.fr.state = "FAILED"
            print(f"{self.name}: STUCK - {KICKS_FAIL} kicks did not move it (battery? wheel?) - FAILED, wheels off")
            return (0, 0)
        if self.kicks % KICKS_REST == 0:
            self.rest_until = t + REST_S
            print(f"{self.name}: still stuck after {self.kicks} kicks - wheels off {REST_S:.0f} s (battery)")
            return (0, 0)
        l, r = cmd
        m = min(255, max(abs(l), abs(r)) + 10 * self.kicks)
        if l * r >= 0:                                       # straight / curve: both sides the same
            sgn = 1 if (l + r) > 0 else -1
            self.kick_cmd = (sgn * m, sgn * m)
        else:                                                # turning on the spot: more power
            self.kick_cmd = (int(math.copysign(m, l)), int(math.copysign(m, r)))
        self.kick_until = t + KICK_S
        print(f"{self.name}: stuck {STALL_S:.1f} s at ({x:.1f},{y:.1f}) on {cmd} - kick {self.kicks}: {self.kick_cmd}")
        return self.kick_cmd

    def send(self, l, r):
        """Wheels; an unreachable robot (off the Wi-Fi) must not stop the loop."""
        try:
            self.car.send(l, r)
            if self.offline:
                print(f"{self.name}: reachable again")
            self.offline = False
        except OSError:
            self.send_err += 1
            if not self.offline:
                print(f"{self.name}: OFFLINE (can't send - off the Wi-Fi? battery?) - the others go on")
            self.offline = True

    def pwm(self, v):
        """Wheel output like arena_drive.to_pwm (floor ad.MIN_PWM 95). Not the robot's own min_pwm: with 110 the slow
        side could never go below 110, so only one side steered - car2 weaved across the lane and went OFF LANE
        (2026-10-10 run 5). The robot's min_pwm still sets its drive speed (lane_follow.MIN_PWM, via params)."""
        return ad.to_pwm(v)

    def from_marker(self, c):
        """Marker corners (px) -> robot turning point (official cm) + heading (deg), as arena_drive does."""
        a = self.arena
        pts = cv2.perspectiveTransform(c.reshape(1, 4, 2).astype(np.float32), a.img2floor)[0]
        pts = a.lift(pts, self.tag_mm)
        up = (pts[0] + pts[1]) / 2 - (pts[2] + pts[3]) / 2
        h = math.atan2(up[1], up[0]) + a.heading_offset
        h += ad.heading_trim(a, h)
        return ad.to_official(ad.mark_to_centre(pts.mean(0), h, a.turn_centre), h)


def find_markers(detector, frame, box):
    """{marker id: corners px} in the arena part of the picture (2x enlarged if a marker is missing)."""
    x0, y0, x1, y1 = box
    gray = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    out = {}
    for scale in (1, 2):
        img = gray if scale == 1 else cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = detector.detectMarkers(img)
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                out.setdefault(int(i), c.reshape(4, 2) / scale + (x0, y0))
        if scale == 1 and len(out) >= 6:
            break
    return out


def cm_to_px(arena, xy, h_mm=0):
    mm = np.float32([[xy[0] * 10.0, (spec.SIZE - xy[1]) * 10.0]])
    seen = arena.drop(mm, h_mm) if h_mm else mm
    return tuple(int(v) for v in cv2.perspectiveTransform(np.float32(seen).reshape(1, 1, 2), arena.floor2img)[0][0])


def draw(frame, bots, coord, paused, fps, msg):
    a = bots[0].arena
    for res, who in coord.owner.items():                   # held nodes: a dot in the robot's colour
        if isinstance(res, str):
            col = next(b.colour for b in bots if b.name == who)
            cv2.circle(frame, cm_to_px(a, spec.NODES[res]["xy"]), 14, col, 3)
    for b in bots:
        f = b.fr
        if f.state == "MOVE" and f.route:
            pts = [cm_to_px(a, spec.NODES[n]["xy"]) for n in f.route]
            cv2.polylines(frame, [np.int32(pts)], False, b.colour, 2)
        if b.pose:
            x, y, h = b.pose
            p = cm_to_px(a, (x, y))
            q = cm_to_px(a, (x + 12 * math.cos(math.radians(h)), y + 12 * math.sin(math.radians(h))))
            cv2.circle(frame, p, 9, b.colour, -1)
            cv2.line(frame, p, q, b.colour, 3)
            cv2.putText(frame, f"{b.name} {f.state} {f.goal or ''}", (p[0] + 12, p[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, b.colour, 2)
    lines = [f"{'PAUSED - SPACE to go on' if paused else 'running'}   {fps:.0f} fps   Q quit"] + \
            [f"{b.name}: {('OFFLINE ' if b.offline else '') + b.fr.state:6s} -> {b.fr.goal or '-'}   jobs left "
             f"{len(b.fr.jobs)}   QR {b.qr_text or '-'}" + (f"   kicks {b.kicks}" if b.kicks else "")
             for b in bots] + ([msg] if msg else [])
    for i, t in enumerate(lines):
        cv2.putText(frame, t, (12, 32 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(frame, t, (12, 32 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)


def main():
    ap = argparse.ArgumentParser(description="Several robots at once, coordinated (fleet.py).")
    ap.add_argument("--job", action="append", required=True, metavar="N=STATION,DEST",
                    help="robot N: drive to STATION, read its QR code, then drive to DEST (repeat per robot)")
    ap.add_argument("--camera", default=None, help="overhead webcam number")
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--no-auto-setup", action="store_true", help="use the saved arena map (don't map at start)")
    ap.add_argument("--dry", type=float, default=0, metavar="SECS",
                    help="check only: track + place + plan for SECS seconds, but the wheels get 0")
    a = ap.parse_args()

    jobs = {}
    for j in a.job:
        n, _, rest = j.partition("=")
        steps = [s.strip().upper() for s in rest.split(",") if s.strip()]
        if not n.isdigit() or len(steps) != 2 or steps[0] not in spec.STATIONS or steps[1] not in spec.NODES:
            sys.exit(f"--job {j}: use N=STATION,DEST, e.g. 1=S6,DZ")
        jobs[int(n)] = [(steps[0], "qr"), (steps[1], "drop" if steps[1] == "DZ" else "park")]
    nums = sorted(jobs)
    calibs = {}
    for n in nums:
        p = os.path.join(HERE, f"arena_calib_car{n}.json")
        if not os.path.exists(p):
            sys.exit(f"No {p} - run tools/bind_robots.py first")
        calibs[n] = json.load(open(p))
    src = a.camera if a.camera is not None else calibs[nums[0]].get("camera", 1)
    src = int(src) if isinstance(src, str) and src.isdigit() else src

    cam = ad.Camera(src)
    first = nums[0]
    ad.use_robot(first)                                    # map with robot 1's file (keeps its own values)
    ad.apply_robot_calib(calibs[first])
    if not a.no_auto_setup:
        print("Mapping the arena ...")
        new = ad.auto_setup(cam, src)
        if new is not None:
            calibs[first] = new
        else:
            print("Using the saved map - only right if the camera hasn't moved.")
    ad.apply_tape(calibs[first])
    for n in nums[1:]:                                      # the same map for every robot
        for k in MAP_KEYS:
            if k in calibs[first]:
                calibs[n][k] = calibs[first][k]
    bots = [Bot(n, jobs[n], calibs[n], i) for i, n in enumerate(nums)]
    coord = fleet.Coordinator([b.fr for b in bots])
    detector = ad.make_detector()
    box = bots[0].arena.search_box

    run_dir = None
    if a.record:
        run_dir = os.path.join(HERE, "runs", time.strftime("fleet-%Y%m%d-%H%M%S"))
        os.makedirs(os.path.join(run_dir, "frames"), exist_ok=True)
        logf = open(os.path.join(run_dir, "log.csv"), "w")
        logf.write("t," + ",".join(f"{b.name}_x,{b.name}_y,{b.name}_h,{b.name}_state,{b.name}_l,{b.name}_r"
                                   for b in bots) + "\n")
    import socket
    chk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    chk.settimeout(0.7)
    for b in bots:                                          # every robot must answer before anything moves
        ok = False
        for _ in range(4):
            try:
                chk.sendto(b"stop", b.car.addr)
                ok = chk.recvfrom(256)[0].startswith(b"ok")
                break
            except OSError:
                time.sleep(0.3)
        if not ok:
            sys.exit(f"{b.name} ({b.car.addr[0]}) does not answer - is it on and on the Wi-Fi? Not starting.")
        b.car.text("assist off")
    chk.close()
    print("Every robot answers. Waiting for every robot's marker ...")
    t_start, placed, paused, msg, said_failed = time.time(), False, False, "", []
    last_frame_t, fps, last_save = 0.0, 0.0, 0.0
    try:
        while True:
            frame, ft = cam.latest()
            if frame is None or ft == last_frame_t:
                time.sleep(0.003)
                continue
            fps = 0.9 * fps + 0.1 / max(1e-3, ft - last_frame_t) if last_frame_t else 0.0
            last_frame_t = ft
            t = ft - t_start
            marks = find_markers(detector, frame, box)
            for b in bots:                                  # poses
                if b.tag in marks:
                    p = b.from_marker(marks[b.tag])
                    if b.pose and math.hypot(p[0] - b.pose[0], p[1] - b.pose[1]) > JUMP_CM + 40 * (t - b.seen_t) \
                            and b.jumps < 5:
                        b.jumps += 1                        # a wild jump: ignore it (unless it persists)
                    else:
                        b.pose, b.seen_t, b.jumps = p, t, 0
            if not placed:
                if all(b.pose for b in bots):
                    for b in bots:
                        coord.place(b.name, b.pose[0], b.pose[1], t)
                    placed = True
                    print("All robots found - starting.")
                else:
                    missing = [b.name for b in bots if not b.pose]
                    msg = "waiting for " + ", ".join(missing)
            poses = {b.name: (b.pose if b.pose and t - b.seen_t < LOST_S else None) for b in bots}
            close = [(p, q) for i, p in enumerate(bots) for q in bots[i + 1:]
                     if p.pose and q.pose and math.hypot(p.pose[0] - q.pose[0], p.pose[1] - q.pose[1]) < TOO_CLOSE_CM]
            if close and not paused:
                paused = True
                msg = f"EMERGENCY STOP: {close[0][0].name} and {close[0][1].name} closer than {TOO_CLOSE_CM:.0f} cm"
                print(msg)
            if placed and not paused:
                cmds = coord.step(t, poses)
            else:
                cmds = {b.name: (0, 0, "PAUSED" if placed else "WAIT") for b in bots}
            for b in bots:                                  # wheels (every frame: keeps the 300 ms fail-safe fed)
                l, r, st = cmds[b.name]
                b.cmd = (0, 0) if a.dry else b.watchdog(t, (b.pwm(l), b.pwm(r)))
                b.send(*b.cmd)
                f = b.fr
                if f.state == "ACT":                        # QR look / drop: the robot stands still
                    if f.action == "qr":
                        host = b.calib.get("cam_host")
                        if b.qr is None and host:
                            b.qr = ad.QRLook([host])
                            b.qr.start()
                            print(f"{b.name}: reading the QR code at {f.done_jobs[-1][0]} ...")
                        if b.qr is None or not b.qr.is_alive():
                            for cam_ip, text, img, pts in (b.qr.results if b.qr else []):
                                b.qr_text = text or "not read"
                                print(f"{b.name}: QR at {f.done_jobs[-1][0]}: {b.qr_text!r}")
                                if run_dir and img is not None:
                                    cv2.imwrite(os.path.join(run_dir, f"qr_{b.name}_{f.done_jobs[-1][0]}.jpg"), img)
                            b.qr = None
                            coord.act_done(b.name, t)
                    else:                                   # "drop": no claw for now - stand 2 s
                        b.act_t = b.act_t or t
                        if t - b.act_t > 2.0:
                            b.act_t = None
                            coord.act_done(b.name, t)
            if run_dir:
                logf.write(f"{t:.3f}," + ",".join(
                    (f"{b.pose[0]:.1f},{b.pose[1]:.1f},{b.pose[2]:.1f}" if b.pose else ",,") +
                    f",{b.fr.state},{b.cmd[0]},{b.cmd[1]}" for b in bots) + "\n")
            view = frame.copy()
            draw(view, bots, coord, paused, fps, msg)
            if run_dir and t - last_save > 0.5:
                cv2.imwrite(os.path.join(run_dir, "frames", f"{t:07.1f}.jpg"), cv2.resize(view, None, fx=0.5, fy=0.5))
                last_save = t
            cv2.imshow("fleet", cv2.resize(view, None, fx=0.6, fy=0.6))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" "):
                paused = not paused
                msg = "" if not paused else "paused"
            if a.dry and t > a.dry:
                print("--dry: done.", {b.name: (b.pose and tuple(round(v, 1) for v in b.pose), b.fr.state, b.fr.route)
                                       for b in bots})
                break
            if placed and coord.all_done():
                print("All jobs done.")
                break
            if placed and coord.failed() and coord.failed() != said_failed:
                said_failed = coord.failed()
                print("FAILED:", said_failed, "- the others keep going; Q to quit")
                if len(said_failed) == len(bots):
                    print("Every robot has failed - stopping.")
                    break
    finally:
        for _ in range(5):
            for b in bots:
                b.send(0, 0)
            time.sleep(0.02)
        for b in bots:
            try:
                b.car.text("stop")
                b.car.close()
            except OSError:
                pass
        cam.running = False
        cv2.destroyAllWindows()
        for b in bots:
            print(f"{b.name}: done {b.fr.done_jobs}, left {b.fr.jobs}, QR {b.qr_text!r}")
        if run_dir:
            logf.close()
            with open(os.path.join(run_dir, "events.txt"), "w") as f:
                for b in bots:
                    for e in b.fr.events:
                        f.write(f"{e[0]:8.1f}s {b.name}: {e[1]}\n")
            print("Recorded to", run_dir)


if __name__ == "__main__":
    main()
