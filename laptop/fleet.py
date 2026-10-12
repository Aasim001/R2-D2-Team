"""Fleet coordination: several robots on the official lanes at the same time - no collisions, no deadlocks.

The floor graph (arena_spec.NODES / EDGES) is a set of EXCLUSIVE RESOURCES: every node (junction, docking
square, bay mouth, bay end) and every lane segment between two neighbouring nodes. Lanes are 20 cm wide and the
robots ~15 cm, so two robots can never share a segment, and a junction is full while a robot turns in it.

Rules (Coordinator.step(), once per camera frame):
  1. WHOLE-ROUTE RESERVATION. A waiting robot starts its next leg only when there is a route to its goal whose
     nodes and segments are all free; it takes ALL of them at once (no hold-and-wait). Whatever happens to its
     timing afterwards (slow turns, retries, Wi-Fi lag), nobody else can be on its way.
  2. ROUTE AROUND STANDING ROBOTS. The route is planned with every resource held by another robot blocked, so a
     robot scanning a QR code on a docking square (which blocks its lane) is driven around (the graph has loops).
  3. RELEASE BEHIND. A moving robot gives back each node - and the segment before it - once its centre is
     CLEAR_CM past that node.
  4. YIELD. A waiting robot that has had no route for YIELD_S because a STANDING robot is in its way makes that
     robot step aside to a free bay / docking square that clears the way (if both block each other, the one that
     has waited less steps aside). A MOVING robot in the way is simply waited for: it will release.
  5. DZ IS SHARED. A robot that has finished in DZ goes on to its parking bay, so DZ is free again.
Why it can't collide: two robots never hold the same node or segment, and a robot only drives on what it holds.
Why it can't deadlock: a moving robot holds its whole route, so it can always finish; a waiting robot holds only
the node it stands on, so a wait cycle can only be between standing robots, and rule 4 breaks it.

Jobs: each robot gets a list of (goal node, action): action "qr" (stand still and read the cube's QR code),
"drop" (stand still in DZ), "park" / None (just go there), "yield" (step aside - added by rule 4).
The run loop (fleet_sim.py or the real one) calls step() every frame with every robot's pose and sends the
returned wheel commands; actions are timed by the caller (act_started / act_done).
"""
import math
import arena_spec as spec
import lane_follow

CLEAR_CM = 30.0       # a node is given back once the robot's centre is this far past it (car1's claw sticks out
                      # ~10 cm in front: 6.5 cm body + 10 cm claw, plus the other robot's half-length and margin)
YIELD_S = 4.0         # no route for this long because a STANDING robot is in the way -> that robot steps aside
POCKET_KINDS = ("bay", "dock")   # where a robot may step aside to (junctions and mouths would block the ring)
FAIL_STATES = ("OFF LANE", "TURN FAILED", "TURN SLID", "NO ROUTE")
OFF_LANE_RECOVER_CM = 15.0   # an OFF LANE robot up to this far off gets ONE re-planned try (it steers back)
MAX_RETRIES = 3       # self-recovery: a failed turn / stop is retried this often (the robot still holds its whole
                      # route, so a retry is safe) before the robot is marked FAILED


def edge(a, b):
    return frozenset((a, b))


def node_at(pt, names):
    """The route node at point pt (cm), or None."""
    for n in names:
        x, y = spec.NODES[n]["xy"]
        if math.hypot(pt[0] - x, pt[1] - y) < 0.6:
            return n
    return None


def node_of_pose(x, y):
    """The node a standing robot is at: its bay if it's inside one, else the nearest lane node."""
    b = spec.bay_of(x, y)
    return b if b is not None else spec.nearest_node(x, y)


class FleetRobot:
    def __init__(self, name, jobs, park=None, params=None):
        self.name = name
        self.jobs = [(g, a) for g, a in jobs]     # still to do: (goal node, action)
        self.park = park                         # its home bay: where it goes after DZ
        self.params = params or {}               # per-robot lane_follow settings, e.g. {"PIVOT_PWM": 150}
        self.follower = lane_follow.Follower()
        self.state = "WAIT"                      # WAIT | MOVE | ACT | DONE | FAILED
        self.node = None                         # node it stands at (WAIT / ACT / DONE)
        self.route = []                          # nodes of the leg being driven (MOVE)
        self.held = set()                        # nodes (str) and segments (frozenset) it holds
        self.wait_since = 0.0                    # (fairness: the longest-waiting robot plans first)
        self.no_route_since = None
        self.retries = 0                         # self-recovery attempts on this leg
        self.off_lane_tries = 0                  # OFF LANE recoveries on this leg
        self.action = None                       # the action being done (ACT)
        self.done_jobs = []                      # (goal, action, time) done
        self.events = []                         # (time, text) - for the log

    @property
    def goal(self):
        return self.jobs[0][0] if self.jobs else None


class Coordinator:
    def __init__(self, robots, log=print):
        self.robots = {r.name: r for r in robots}
        self.owner = {}                          # resource -> robot name
        self.log = log

    # ---------------------------------------------------------------- resources
    def _hold(self, r, res):
        other = self.owner.get(res)
        assert other is None or other == r.name, f"{res} is held by {other}, not free for {r.name}"
        self.owner[res] = r.name
        r.held.add(res)

    def _release(self, r, res):
        if self.owner.get(res) == r.name:
            del self.owner[res]
        r.held.discard(res)

    def _blocked(self, r, ignore=()):
        """Nodes and segments held by other robots (except the robots in `ignore`)."""
        nodes, edges = set(), set()
        for res, who in self.owner.items():
            if who == r.name or who in ignore:
                continue
            (edges if isinstance(res, frozenset) else nodes).add(res)
        return nodes, edges

    def _route(self, r, goal, ignore=(), extra_nodes=()):
        """A route from r's node to goal over resources nobody else holds, or None."""
        nodes, edges = self._blocked(r, ignore)
        nodes |= set(extra_nodes)
        if goal in nodes and goal != r.node:
            return None                                          # the goal itself is taken
        if goal == r.node:
            return [goal]
        return spec.route(r.node, goal, nodes, edges)

    # ---------------------------------------------------------------- start / events
    def place(self, name, x, y, t=0.0):
        """Robot `name` stands at (x, y) cm: it holds that node."""
        r = self.robots[name]
        r.node = node_of_pose(x, y)
        self._hold(r, r.node)
        r.state, r.wait_since = "WAIT", t
        self._say(r, t, f"starts at {r.node}")

    def act_done(self, name, t):
        """The caller finished robot `name`'s action (QR look / drop)."""
        r = self.robots[name]
        if r.state == "ACT":
            r.state, r.action, r.wait_since = "WAIT", None, t
            self._say(r, t, "action done")

    def _say(self, r, t, text):
        r.events.append((round(t, 2), text))
        if self.log:
            self.log(f"[{t:7.1f}s] {r.name}: {text}")

    # ---------------------------------------------------------------- per frame
    def step(self, t, poses):
        """poses: {name: (x cm, y cm, heading deg) or None if not seen}. Returns {name: (left, right, state)}."""
        out = {}
        for r in self.robots.values():                           # moving robots first: they release resources
            if r.state == "MOVE":
                out[r.name] = self._drive(r, t, poses.get(r.name))
        waiting = sorted((r for r in self.robots.values() if r.state == "WAIT"), key=lambda r: (r.wait_since, r.name))
        for r in waiting:
            out[r.name] = self._plan(r, t, poses.get(r.name))
        for r in self.robots.values():
            if r.name not in out:
                out[r.name] = (0, 0, r.state)
        self._yield_check(t)
        return out

    def _drive(self, r, t, pose):
        if pose is None:                                         # not seen this frame: stand still, keep it all
            return 0, 0, "NOT SEEN"
        x, y, h = pose
        saved = {k: getattr(lane_follow, k) for k in r.params}  # this robot's own turning power etc.
        for k, v in r.params.items():
            setattr(lane_follow, k, v)
        try:
            left, right, st = r.follower.update(t, x, y, h)
        finally:
            for k, v in saved.items():
                setattr(lane_follow, k, v)
        self._release_behind(r, x, y)
        if st == "ARRIVED":
            goal, action = r.jobs.pop(0)
            r.node = goal
            for res in list(r.held):                             # keep only the node it stands at
                if res != goal:
                    self._release(r, res)
            r.done_jobs.append((goal, action, round(t, 1)))
            self._say(r, t, f"arrived at {goal}" + (f" -> {action}" if action else ""))
            if goal == "DZ" and r.park and not any(g == r.park for g, _ in r.jobs):
                r.jobs.append((r.park, "park"))                  # DZ is shared: move on afterwards
            if action in ("qr", "drop"):
                r.state, r.action = "ACT", action
            else:
                r.state, r.wait_since = ("WAIT" if r.jobs else "DONE"), t
            return 0, 0, r.state
        if st in FAIL_STATES:
            off = spec.project_to_lane(x, y)[1]
            if st == "OFF LANE" and (r.off_lane_tries >= 1 or off > OFF_LANE_RECOVER_CM):
                pass                                             # (too far, or already tried once: stop)
            elif r.retries < MAX_RETRIES:                        # self-recovery: drive the rest of the leg again
                r.retries += 1
                if st == "OFF LANE":                             # steer back from where it is (6-12 cm off)
                    r.off_lane_tries += 1
                    r.follower.off_lane_cm = OFF_LANE_RECOVER_CM
                ahead = node_at(r.follower.pieces[r.follower.i].b, r.route) if r.follower.i < len(r.follower.pieces)                     else r.route[-1]
                rest = r.route[r.route.index(ahead):] if ahead in r.route else r.route[-1:]
                if r.follower.go_route(x, y, h, rest):
                    self._say(r, t, f"{st} - retry {r.retries} of the rest of the leg: " + " -> ".join(rest))
                    return 0, 0, "RETRY"
            r.state = "FAILED"                                   # it keeps what it holds: the others go around
            self._say(r, t, f"FAILED on the way to {r.goal}: {st}")
            return 0, 0, "FAILED"
        return left, right, st

    def _release_behind(self, r, x, y):
        f = r.follower
        if not f.pieces or f.i >= len(f.pieces):
            return
        ahead = node_at(f.pieces[f.i].b, r.route)                # the node it is driving to now
        if ahead is None:
            return
        k = r.route.index(ahead)
        for j in range(k):                                       # nodes before it (and the segments to them)
            n = r.route[j]
            if n not in r.held:
                continue
            nx, ny = spec.NODES[n]["xy"]
            if j < k - 1 or math.hypot(x - nx, y - ny) > CLEAR_CM:
                self._release(r, n)
                if j > 0:
                    self._release(r, edge(r.route[j - 1], n))
                if j < k - 1:
                    self._release(r, edge(n, r.route[j + 1]))

    def _plan(self, r, t, pose):
        if not r.jobs:
            r.state = "DONE"
            return 0, 0, "DONE"
        goal, action = r.jobs[0]
        if goal == r.node:                                       # already there
            r.jobs.pop(0)
            r.done_jobs.append((goal, action, round(t, 1)))
            if action in ("qr", "drop"):
                r.state, r.action = "ACT", action
                self._say(r, t, f"at {goal} -> {action}")
            return 0, 0, r.state
        if pose is None:
            return 0, 0, "NOT SEEN"
        names = self._route(r, goal)
        if names is None:
            if r.no_route_since is None:
                r.no_route_since = t
                self._say(r, t, f"waits: no free route to {goal}")
            return 0, 0, "WAIT"
        for n in names:                                          # rule 1: take the whole route at once
            self._hold(r, n)
        for a, b in zip(names, names[1:]):
            self._hold(r, edge(a, b))
        x, y, h = pose
        off = spec.project_to_lane(x, y)[1]                      # starting a little off the lane (e.g. where it
        r.follower.off_lane_cm = OFF_LANE_RECOVER_CM if off > lane_follow.OFF_LANE_CM else lane_follow.OFF_LANE_CM
        if not r.follower.go_route(x, y, h, names):              # stopped last time): start, then steer back
            r.state = "FAILED"
            self._say(r, t, f"FAILED to start to {goal}: {r.follower.state}")
            return 0, 0, "FAILED"
        r.route, r.state, r.no_route_since, r.retries, r.off_lane_tries = names, "MOVE", None, 0, 0
        if off > lane_follow.OFF_LANE_CM:
            r.off_lane_tries = 1                                 # (this start already uses the wider tolerance)
        else:
            r.follower.off_lane_cm = lane_follow.OFF_LANE_CM
        self._say(r, t, f"goes to {goal}: " + " -> ".join(names))
        return 0, 0, "MOVE"

    def _yield_check(self, t):
        """Rule 4: a robot stuck behind a STANDING robot makes it step aside."""
        for a in self.robots.values():
            if a.state != "WAIT" or a.no_route_since is None or t - a.no_route_since < YIELD_S or not a.jobs:
                continue
            standing = [b for b in self.robots.values() if b is not a and b.state in ("WAIT", "DONE")]
            for b in standing:
                if b.jobs and b.jobs[0][1] == "yield":
                    continue                                     # already stepping aside
                path = self._route(a, a.goal, ignore=(b.name,))
                if path is None:
                    continue                                     # b isn't (the only one) in the way
                if b.state == "WAIT" and b.no_route_since is not None and \
                        (b.no_route_since, b.name) < (a.no_route_since, a.name):
                    continue                                     # both stuck: the one that waited less yields
                pocket = self._pocket(b, avoid=set(path))
                if pocket is None:
                    continue
                b.jobs.insert(0, (pocket, "yield"))
                b.state, b.wait_since = "WAIT", t
                self._say(b, t, f"steps aside to {pocket} for {a.name} (it needs {' -> '.join(path)})")
                a.no_route_since = t                             # (give it time to clear)
                return

    def _pocket(self, b, avoid):
        """Nearest free bay / docking square b can reach that isn't on `avoid` (the other robot's route).
        (b may cross that route on its way: the other robot is standing still, and b's own leg is reserved.)"""
        best = None
        for n, v in spec.NODES.items():
            if v["kind"] not in POCKET_KINDS or n in avoid or n in self.owner or n == "DZ":
                continue
            names = self._route(b, n)
            if names is None:
                continue
            d = spec.path_length(names)
            if best is None or d < best[0]:
                best = (d, n)
        return None if best is None else best[1]

    # ---------------------------------------------------------------- status
    def all_done(self):
        return all(r.state == "DONE" for r in self.robots.values())

    def failed(self):
        return [r.name for r in self.robots.values() if r.state == "FAILED"]

    def check_invariants(self):
        """Every resource has at most one owner, and each robot's `held` agrees with the owner map."""
        for r in self.robots.values():
            for res in r.held:
                assert self.owner.get(res) == r.name, (r.name, res, self.owner.get(res))
        for res, who in self.owner.items():
            assert res in self.robots[who].held, (res, who)
