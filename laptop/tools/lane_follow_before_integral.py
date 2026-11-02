#!/usr/bin/env python3
"""
Route following for ONE robot along the lane centre lines of the official floor (arena_spec.py).

Works in the official frame: centimetres, x east, y north, heading in degrees, 0 = east,
counter-clockwise positive. No camera code in here: give it the robot's pose every frame and it
gives back the two wheel commands, so it can be tested on its own (python lane_follow.py).

How it drives:
  * the route is a list of straight pieces along lane centre lines (from arena_spec.route_from_point)
  * it only turns on the spot where there is room: at junctions and bay mouths. A 15 x 15 cm robot
    turning in the middle of a 20 cm lane would sweep its corners over the tape, so when it would
    otherwise have to turn round in a lane, it drives that stretch IN REVERSE instead:
      - at the start, if it is facing the wrong way
      - onto a docking square, if that is what puts its pick side toward the alcove
      - out of a bay (parking / dispatch: there is no room to turn in there anyway)
  * on a straight it steers to stay on the centre line (2.5 cm to spare each side)
  * after every stop and every turn it waits a moment, looks again, and nudges if it is off
"""
import math
import arena_spec as spec

# ---------------- tunables ----------------
BASE_PWM    = 130         # straight-line speed (PWM 0-255)
MIN_PWM     = 95          # N20s stall below this - also the speed for nudges
PIVOT_PWM   = 120         # turning on the spot, far from the target heading
ALIGN_DEG   = 5           # a turn is done within this
BEND_DEG    = 12          # a change of direction bigger than this at a node = a bend (stop and turn there)
REALIGN_DEG = 35          # heading this far off while driving -> stop and turn first
STOP_CM     = 1.0         # a stop point is reached within this (along the lane)
EDGE_SHORT_CM = 1.0       # stopping at a junction with the outer edge straight ahead: aim this much short,
                          # so an overshoot can't put the robot's turning circle over the edge
SLOW_CM     = 12          # start slowing down this far before a stop point
SETTLE_S    = 0.3         # wait this long after stopping before checking again
K_CROSS     = 0.10        # rad of heading correction per cm away from the centre line
K_HEAD      = 0.35        # turn strength per rad of heading error (as a fraction of the speed)
K_RATE      = 0.10        # damping: less turn per rad/s the robot is already turning
OFF_LANE_CM = 6           # further than this from the centre line -> stop (something is wrong)
LATENCY_S   = 0.15        # camera + Wi-Fi delay: stops and turns are ended this much early (a first
                          # guess - it is raised automatically whenever a turn overshoots)
PICK_SIDE   = "right"     # side of the robot the pick arm reaches out of (CAD: right side)
PIVOT_OK_DEG = 20         # a turn bigger than this away from a junction / bay mouth = "turning in a lane"
REVERSE_COST = 1.3        # when comparing plans, a cm driven backwards counts as this many cm
MAX_TRIES    = 8          # settle-and-check rounds per turn/stop before a looser tolerance is accepted
SMALL_DEG    = 25         # turns smaller than this are done in timed pulses (works whatever the delay)
OMEGA_GUESS  = 1.5        # first guess: rad/s the robot turns at MIN_PWM (corrected automatically)
SPEED_GUESS  = 8.0        # first guess: cm/s the robot drives at MIN_PWM (corrected automatically)
MAX_PULSE_S  = 0.6        # longest single pulse


STOPPED_STATES = ("IDLE", "ARRIVED", "NO ROUTE", "OFF LANE", "TURN FAILED")   # wheels stay at 0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Piece:
    """One straight piece of the route. reverse = drive it backwards."""
    def __init__(self, a, b):
        self.a, self.b = a, b
        self.length = math.hypot(b[0] - a[0], b[1] - a[1])
        self.dir = math.atan2(b[1] - a[1], b[0] - a[0])            # direction of travel (rad)
        self.reverse, self.stop = False, True
        # outer edge of the floor straight ahead, a half-lane past the end point?
        ax, ay = b[0] + 10 * math.cos(self.dir), b[1] + 10 * math.sin(self.dir)
        self.edge_ahead = not (0.5 < ax < spec.SIZE - 0.5 and 0.5 < ay < spec.SIZE - 0.5)

    @property
    def stop_at(self):
        """Length at which to stop: the end point, or a little short of it with the outer edge ahead."""
        return self.length - (EDGE_SHORT_CM if (self.stop and self.edge_ahead) else 0)

    @property
    def body(self):                                                # where the robot's nose points
        return wrap(self.dir + (math.pi if self.reverse else 0))


def make_pieces(points, start_heading, final_heading=None):
    """Straight pieces between the route points, each marked forward/reverse and stop/no-stop.
    start_heading / final_heading in rad (final = None: any)."""
    pieces = [Piece(p, q) for p, q in zip(points, points[1:]) if math.hypot(q[0] - p[0], q[1] - p[1]) >= 0.5]
    if not pieces:
        return pieces
    # runs = groups of pieces in a straight line (no bend between them)
    runs, cur = [], [pieces[0]]
    for prev, p in zip(pieces, pieces[1:]):
        if abs(wrap(p.dir - prev.dir)) < math.radians(BEND_DEG):
            cur.append(p)
        else:
            runs.append(cur)
            cur = [p]
    runs.append(cur)

    def set_run(run, rev):
        for p in run:
            p.reverse = rev

    def at_junction(pt):
        return any(v["kind"] == "junction" and math.hypot(pt[0] - v["xy"][0], pt[1] - v["xy"][1]) < 0.5
                   for v in spec.NODES.values())

    for run in runs:                                               # out of a bay: reverse (no room to turn
        bay = spec.bay_of(*run[0].a)                               # in there) - unless already facing out
        if bay is not None:
            mx, my = spec.BAYS[bay]["mouth"]
            outward = math.hypot(run[0].b[0] - mx, run[0].b[1] - my) < math.hypot(run[0].a[0] - mx, run[0].a[1] - my)
            facing_out = run is runs[0] and abs(wrap(run[0].dir - start_heading)) < math.pi / 2
            if outward and not facing_out:
                set_run(run, True)
    first, last = runs[0], runs[-1]
    starts_at_node = any(math.hypot(first[0].a[0] - v["xy"][0], first[0].a[1] - v["xy"][1]) < 3
                         and v["kind"] in ("junction", "mouth") for v in spec.NODES.values())
    if not first[0].reverse and not starts_at_node:                # facing the wrong way in a lane: back up,
        if abs(wrap(first[0].dir - start_heading)) > math.radians(100):   # but only to the first junction
            k = next((i for i, p in enumerate(first) if at_junction(p.b)), len(first) - 1)
            set_run(first[:k + 1], True)                           # (turns round there, where there's room)
    if final_heading is not None and not (len(runs) == 1 and spec.bay_of(*first[0].a)):
        if abs(wrap(last[-1].dir - final_heading)) > math.radians(90):     # arrive with the pick side toward
            k = max((i for i, p in enumerate(last[:-1]) if at_junction(p.b)), default=-1)   # the alcove:
            set_run(last[k + 1:], True)                            # reverse, but only from the last junction
    for i, p in enumerate(pieces):                                 # stop where the nose has to change
        nxt = pieces[i + 1] if i + 1 < len(pieces) else None
        p.stop = nxt is None or abs(wrap(nxt.body - p.body)) > math.radians(BEND_DEG) or nxt.reverse != p.reverse
    return pieces


def roomy(pt):
    """Is there room to turn on the spot here? Only at a junction or a bay mouth."""
    return any(v["kind"] in ("junction", "mouth") and math.hypot(pt[0] - v["xy"][0], pt[1] - v["xy"][1]) < 3
               for v in spec.NODES.values())


def grade(pieces, start, start_heading, final_heading):
    """(turns in a lane, cost) of a plan: fewest turns where there's no room first, then shortest."""
    bad, cur = 0, start_heading
    for p in pieces:
        if abs(wrap(p.body - cur)) > math.radians(PIVOT_OK_DEG) and not roomy(p.a):
            bad += 1
        cur = p.body
    end = pieces[-1].b if pieces else start
    if final_heading is not None and abs(wrap(final_heading - cur)) > math.radians(PIVOT_OK_DEG) and not roomy(end):
        bad += 1
    return bad, sum(p.length * (REVERSE_COST if p.reverse else 1) for p in pieces)


class Follower:
    def __init__(self, turn_sign=1):
        self.turn_sign = turn_sign          # -1 if the robot turns the other way (found by the H test)
        self.pieces, self.i, self.state, self.goal = [], 0, "IDLE", None
        self.final_heading = None           # rad, or None
        self.prev = None                    # (t, x, y, heading rad) for speed estimates
        self.v_est, self.w_est = 0.0, 0.0   # cm/s along the nose, rad/s counter-clockwise
        self.wait_until, self.after_wait = 0.0, None
        self.latency = LATENCY_S            # learned: raised when a fast turn overshoots
        self.stop_lat = LATENCY_S           # learned: from how far a straight-line stop over/undershoots
        self.pivot_min = MIN_PWM            # learned: raised if pulses at MIN_PWM don't turn the robot
        self.turn_gain, self.move_gain = 1.0, 1.0   # learned: pulse length corrections
        self.hist = []                      # recent poses, to tell when the robot is really still
        self._reset_tries()

    def _reset_tries(self):
        self.tries, self.turn_dir, self.last_err, self.pulse_until, self.weak = 0, 0, None, None, 0
        self.turn_started = None

    # ---------------------------------------------------------------- planning
    def go(self, x, y, heading_deg, goal):
        """Plan a route from the robot's pose to a goal node ('S1'..'S6', 'DZ', 'P1'..'P3', or any node).
        Without cut-outs (spec.CUTOUTS = False) a bay goal means the bay's mouth on the ring.
        Tries the direct route and routes via each junction, and keeps the one with the fewest turns
        where there's no room (mid-lane, on a docking square), then the shortest."""
        goal = spec.goal_node(goal)
        (_, _), off = spec.project_to_lane(x, y)
        if goal not in spec.NODES:
            self.state = "NO ROUTE"
            return False
        if off > OFF_LANE_CM:                                      # too far off: put it back by hand
            self.state = "OFF LANE"
            return False
        h0 = math.radians(heading_deg)
        self.final_heading = (math.radians(spec.heading_for_pick(goal, PICK_SIDE))
                              if goal in spec.STATIONS else None)
        best = None
        for via in [None] + [n for n, v in spec.NODES.items() if v["kind"] == "junction"]:
            pts = self._plan_points(x, y, goal, via)
            if pts is None:
                continue
            if off > 3:                                            # well off the centre line: only if the robot
                lane = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]) if len(pts) > 1 else None
                if lane is None or abs(abs(wrap(h0 - lane)) - math.pi / 2) < math.pi / 4:   # faces ACROSS the lane
                    pts = [(x, y)] + pts                           # go onto the line first (it must turn anyway)
            pieces = make_pieces(pts, h0, self.final_heading)
            g = grade(pieces, pts[0], h0, self.final_heading)
            if best is None or g < best[0]:
                best = (g, pieces)
        if best is None:
            self.state = "NO ROUTE"
            return False
        self.pieces, self.i, self.goal = best[1], 0, goal
        self._reset_tries()
        self.state = "TURN" if self.pieces else "FINAL TURN"
        return True

    @staticmethod
    def _plan_points(x, y, goal, via):
        if via is None:
            r = spec.route_from_point(x, y, goal)
            return None if r is None else r[0]
        r1, r2 = spec.route_from_point(x, y, via), spec.route(via, goal)
        if r1 is None or r2 is None:
            return None
        return r1[0] + [spec.NODES[n]["xy"] for n in r2[1:]]

    def stop(self):
        self.state, self.pieces = "IDLE", []

    def route_points(self):
        """Points of the route still to drive (for drawing)."""
        if self.i >= len(self.pieces):
            return []
        return [self.pieces[self.i].a] + [p.b for p in self.pieces[self.i:]]

    # ---------------------------------------------------------------- helpers
    def _wheels(self, left, right):
        """Physical wheel commands; swapped if this robot turns the other way."""
        return (right, left) if self.turn_sign < 0 else (left, right)

    def _pivot(self, err):
        """Turn on the spot toward a heading error (rad, + = counter-clockwise)."""
        s = math.copysign(max(PIVOT_PWM, self.pivot_min) if abs(err) > math.radians(SMALL_DEG + 10)
                          else self.pivot_min, err)
        return self._wheels(-s, s)

    def _wait(self, t, then):
        """Stop, and carry on with `then` once the robot is really still (at least SETTLE_S, at most 2 s:
        with a long camera/Wi-Fi delay it keeps moving for a while after the stop command)."""
        self.wait_until, self.wait_max, self.after_wait, self.state = t + SETTLE_S, t + 2.0, then, "SETTLE"
        return 0, 0, self.state

    def _still(self, t):
        """No movement over the last 0.25 s (compares poses, so camera noise doesn't fool it)."""
        old = [p for p in self.hist if p[0] <= t - 0.25]
        if not old:
            return False
        _, x0, y0, h0 = old[-1]
        _, x1, y1, h1 = self.hist[-1]
        return math.hypot(x1 - x0, y1 - y0) < 0.6 and abs(wrap(h1 - h0)) < math.radians(1.5)

    def _turn_to(self, t, target, then):
        """Turn on the spot to target (rad), then settle and check again. Big turns go fast until close.
        Small turns are timed pulses: how far a pulse turns doesn't depend on the delay (start and stop
        are delayed alike), so this converges whatever the delay. The pulse length learns: shorter after
        an overshoot, longer if the robot hardly moved (and more power if it doesn't move at all).
        After MAX_TRIES rounds a looser tolerance is accepted; after 2 x MAX_TRIES: TURN FAILED."""
        err = wrap(target - self._h)
        if self.pulse_until is not None:                           # a pulse is running
            if t < self.pulse_until:
                s = self.turn_dir * self.pivot_min
                return (*self._wheels(-s, s), self.state)
            self.pulse_until = None
            self.tries += 1
            return self._wait(t, self.state)
        tol = math.radians(ALIGN_DEG if self.tries < MAX_TRIES else 2 * ALIGN_DEG)
        if abs(err) <= tol:
            self._reset_tries()
            self.state = then
            return 0, 0, then
        if self.turn_started is None:
            self.turn_started = t
        if self.tries >= 2 * MAX_TRIES or t - self.turn_started > 20:   # (pulses tried / time limit)
            self.state = "TURN FAILED"
            return 0, 0, self.state
        if abs(err) > math.radians(SMALL_DEG + 10):                # big turn: fast, stop a bit early
            if self.turn_dir and math.copysign(1, err) != self.turn_dir:
                self.latency = min(0.8, self.latency * 1.3)        # overshot last time: stop earlier
            turning = abs(self.w_est) > 0.5                        # (rad/s - not just camera noise)
            if turning and abs(err) <= math.radians(SMALL_DEG) + abs(self.w_est) * self.latency:
                return self._wait(t, self.state)                   # it will coast most of the rest
            self.turn_dir = math.copysign(1, err)
            return (*self._pivot(err), self.state)
        if self.last_err is not None:                              # small turn: learn from the last pulse
            if math.copysign(1, err) != math.copysign(1, self.last_err):
                self.turn_gain = max(0.2, self.turn_gain * 0.6)    # overshot
                self.weak = 0
            elif abs(err) > 0.7 * abs(self.last_err):              # hardly moved
                self.turn_gain = min(4.0, self.turn_gain * 1.4)
                self.weak += 1
                if self.weak >= 3:                                 # pulses don't turn it: more power
                    self.pivot_min, self.weak = min(PIVOT_PWM + 40, self.pivot_min + 10), 0
            else:
                self.weak = 0
        self.last_err, self.turn_dir = err, math.copysign(1, err)
        self.pulse_until = t + min(MAX_PULSE_S, abs(err) / OMEGA_GUESS * self.turn_gain)
        return (*self._pivot(err), self.state)

    # ---------------------------------------------------------------- main
    def update(self, t, x, y, heading_deg):
        """Pose in -> (left PWM, right PWM, state). Call once per camera frame."""
        h = self._h = math.radians(heading_deg)
        if self.prev is not None and t > self.prev[0]:
            dt = t - self.prev[0]
            fwd = ((x - self.prev[1]) * math.cos(h) + (y - self.prev[2]) * math.sin(h)) / dt
            self.v_est = 0.6 * self.v_est + 0.4 * fwd
            self.w_est = 0.6 * self.w_est + 0.4 * wrap(h - self.prev[3]) / dt
        self.prev = (t, x, y, h)
        self.hist = [p for p in self.hist if p[0] > t - 1.0] + [(t, x, y, h)]

        if self.state in STOPPED_STATES:
            return 0, 0, self.state
        if self.state == "SETTLE":
            if t < self.wait_until or (t < self.wait_max and not self._still(t)):
                return 0, 0, self.state
            self.state = self.after_wait
        if self.state == "FINAL TURN":
            if self.final_heading is None:
                self.state = "ARRIVED"
                return 0, 0, self.state
            r = self._turn_to(t, self.final_heading, "ARRIVED")
            return r

        p = self.pieces[self.i]
        ux, uy = math.cos(p.dir), math.sin(p.dir)
        dx, dy = x - p.a[0], y - p.a[1]
        along = dx * ux + dy * uy                                  # cm travelled along this piece
        cross = ux * dy - uy * dx                                  # cm left (+) / right (-) of the centre line
        left_to_go = p.stop_at - along

        if self.state == "TURN":                                   # line up with the piece before driving it
            return self._turn_to(t, p.body, "DRIVE")

        if self.state == "NUDGE":                                  # settle-and-check at a stop point:
            if self.pulse_until is not None:                       # timed pulses, like small turns
                if t < self.pulse_until:
                    s = MIN_PWM * self.turn_dir * (-1 if p.reverse else 1)   # forward/back along the nose
                    return (*self._wheels(s, s), self.state)
                self.pulse_until = None
                self.tries += 1
                return self._wait(t, "NUDGE")
            if self.tries == 0 and self.last_err is None:          # first look after a DRIVE stop: learn
                if left_to_go < -STOP_CM:                          # the stopping delay from it
                    self.stop_lat = min(0.8, self.stop_lat * 1.3)  # (overshot: brake earlier next time)
                elif left_to_go > 3:
                    self.stop_lat = max(0.05, self.stop_lat * 0.8) # (stopped well short: brake later)
            tol = STOP_CM if self.tries < MAX_TRIES else 2 * STOP_CM
            if abs(left_to_go) <= tol or self.tries >= 2 * MAX_TRIES:   # (close enough: carry on)
                return self._next_piece(t)
            if self.last_err is not None:
                if math.copysign(1, left_to_go) != math.copysign(1, self.last_err):
                    self.move_gain = max(0.2, self.move_gain * 0.6)          # overshot
                elif abs(left_to_go) > 0.7 * abs(self.last_err):
                    self.move_gain = min(4.0, self.move_gain * 1.4)          # hardly moved
            self.last_err, self.turn_dir = left_to_go, math.copysign(1, left_to_go)
            self.pulse_until = t + min(MAX_PULSE_S, abs(left_to_go) / SPEED_GUESS * self.move_gain)
            s = MIN_PWM * self.turn_dir * (-1 if p.reverse else 1)
            return (*self._wheels(s, s), self.state)

        # DRIVE
        if abs(cross) > OFF_LANE_CM:
            self.state = "OFF LANE"
            return 0, 0, self.state
        if left_to_go <= STOP_CM + (abs(self.v_est) * self.stop_lat if p.stop else 0):
            if p.stop:
                return self._wait(t, "NUDGE")
            return self._next_piece(t)
        travel = wrap(h + (math.pi if p.reverse else 0))           # direction the robot is moving
        want = wrap(p.dir - math.atan(K_CROSS * cross))            # steer back toward the centre line
        err = wrap(want - travel)
        if abs(err) > math.radians(REALIGN_DEG):
            return self._wait(t, "TURN")
        v = BASE_PWM
        if p.stop:
            v = max(MIN_PWM, BASE_PWM * min(1.0, left_to_go / SLOW_CM))
        turn = (K_HEAD * err - K_RATE * self.w_est) * v            # + = counter-clockwise
        left, right = v - turn, v + turn
        if p.reverse:                                              # backwards: same turn, wheels mirrored
            left, right = -right, -left
        return (*self._wheels(left, right), self.state)

    def _next_piece(self, t):
        self._reset_tries()
        stopped = self.pieces[self.i].stop
        self.i += 1
        if self.i >= len(self.pieces):
            self.state = "FINAL TURN"
            return 0, 0, self.state
        if not stopped:
            self.state = "DRIVE"
            return 0, 0, self.state
        nxt = self.pieces[self.i]
        if abs(wrap(nxt.body - self._h)) <= math.radians(ALIGN_DEG):
            self.state = "DRIVE"
        else:
            self.state = "TURN"
        return 0, 0, self.state


# ---------------------------------------------------------------- self-test (no hardware)
def simulate(start, heading, goals, robot=(13.0, 14.5), seed=1, verbose=True, noise_cm=0.3, lag=4,
             ramp=1200, ramp_start=0):
    """Drive a simulated robot (lag, noise, wheel slip, the firmware's speed-up ramp) through goals.
    robot = (length, width) cm. ramp = firmware RAMP_PWM_PER_S (None = instant); ramp_start = PWM the
    ramp starts from (0 = from standstill, as robot_main.ino does now).
    Returns dict(worst keep-out depth cm, failed legs, worst stop error cm)."""
    import random
    rnd = random.Random(seed)
    TRACK, DT, LAG = 12.0, 1 / 30, lag                             # cm between wheels, frame time, frames of delay
    fw = [0.0, 0.0]                                                # firmware ramp state (left, right)

    def approach(cur, target, up):                                 # = robot_main.ino approach()
        if target == 0:
            return 0.0
        if cur * target < 0:
            cur = 0.0
        if abs(target) < abs(cur):
            return target
        if cur == 0:
            cur = math.copysign(min(abs(target), ramp_start), target)
        return cur + max(-up, min(up, target - cur))

    def wheel(pwm):                                                # ~55 cm/s at 255, stalls below ~80
        return 0.0 if abs(pwm) < 80 else math.copysign((abs(pwm) - 70) * 0.30, pwm)

    L, W = robot
    corners = [(L / 2, W / 2), (L / 2, -W / 2), (-L / 2, W / 2), (-L / 2, -W / 2)]
    x, y, h = start[0], start[1], math.radians(heading)
    f, t, queue, worst, worst_at = Follower(), 0.0, [(0, 0)] * LAG, 0.0, None
    fails, worst_stop = [], 0.0
    for g in goals:
        f.go(x, y, math.degrees(h), g)
        if verbose:
            print(f"  {g}: " + ", ".join(f"{'REV' if p.reverse else 'fwd'} {p.length:.0f}{'|' if p.stop else ''}" for p in f.pieces))
        t_leg = t
        while f.state not in STOPPED_STATES and t - t_leg < 120:
            l, r, st = f.update(t, x + rnd.gauss(0, noise_cm), y + rnd.gauss(0, noise_cm),
                                math.degrees(h) + rnd.gauss(0, noise_cm * 1.7))
            queue.append((l, r))
            l, r = queue.pop(0)
            if ramp:
                fw[0], fw[1] = approach(fw[0], l, ramp * DT), approach(fw[1], r, ramp * DT)
                l, r = fw
            vl, vr = wheel(l) * rnd.gauss(1, 0.03), wheel(r) * rnd.gauss(1, 0.03)
            h += (vr - vl) / TRACK * DT                            # counter-clockwise positive
            x += (vl + vr) / 2 * DT * math.cos(h)
            y += (vl + vr) / 2 * DT * math.sin(h)
            t += DT
            for cx, cy in corners:
                px, py = x + cx * math.cos(h) - cy * math.sin(h), y + cx * math.sin(h) + cy * math.cos(h)
                d = spec.keepout_depth(px, py)                     # how far past the lane edge
                if d > 0:
                    if d > worst:
                        worst, worst_at = d, (g, st, round(x, 1), round(y, 1), round(math.degrees(h) % 360))
        tx, ty = spec.NODES[spec.goal_node(g)]["xy"]
        herr = "" if f.final_heading is None else f", heading off by {abs(math.degrees(wrap(f.final_heading - h))):.1f} deg"
        if f.state != "ARRIVED":
            fails.append((g, f.state, round(x, 1), round(y, 1)))
        worst_stop = max(worst_stop, math.hypot(x - tx, y - ty))
        if verbose:
            print(f"     {f.state} after {t - t_leg:5.1f} s, {math.hypot(x - tx, y - ty):.1f} cm from the target{herr}")
    if verbose:
        print(f"  deepest robot corner past a lane edge: {worst:.1f} cm {worst_at or ''}")
    return dict(worst=worst, worst_at=worst_at, fails=fails, worst_stop=worst_stop)


if __name__ == "__main__":
    if spec.CUTOUTS:
        print("Robot parked in P1 (facing south) -> S2 -> DZ -> S5 -> P3 -> S6 -> S1 -> P2:")
        simulate((50, 265), 270, ["S2", "DZ", "S5", "P3", "S6", "S1", "P2"])
    else:
        print("No cut-outs (arena_spec.CUTOUTS = False): bay goals stop at the bay's mouth.")
        print("Robot on the top ring in front of P1 (facing east) -> S2 -> DZ -> S5 -> P3 -> S6 -> S1 -> P2:")
        simulate((50, 290), 0, ["S2", "DZ", "S5", "P3", "S6", "S1", "P2"])
