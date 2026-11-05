"""Fleet planner: which robot fetches which cube, in which order, along which lanes, and who waits for whom.

The method warehouse robot fleets use (multi-agent path finding, MAPF), sized for 2-3 robots in 20 cm lanes:
1. The lanes are cut into CELLS (~CELL_CM long, a node at every junction / docking square / bay mouth / bay point).
2. Every robot's mission is planned in SPACE AND TIME: home bay -> station (pick) -> DZ (drop) -> home bay.
   A robot's plan = the cells it passes and when. Two robots must never be closer than SAFE_CM (cell centres) at
   overlapping times (+- MARGIN_S): that rules out collisions, head-on meetings in a lane, two robots in a dead end.
3. SEARCH: every assignment of cubes to robots, every order of each robot's cubes, every priority order of the
   robots (who plans first), and up to ROUTE_ALTS routes per leg (e.g. the centre cross instead of the ring), each
   leg leaving as early as it can without a conflict. The plan that finishes everything soonest wins (makespan, then
   the total). With 2-3 robots this is a few hundred cheap plans: in practice the optimum.
4. EXECUTION (robust to delays - a slow pick, a retry): an ACTION DEPENDENCY GRAPH (Hoenig et al. 2019, used for
   warehouse fleets). For every pair of robot steps whose cells are closer than SAFE_CM, the one planned earlier must
   be over before the other may start: "robot B may enter its step j once robot A has reached its step k". Every
   dependency points from an earlier planned time to a later one, so they can never wait for each other in a circle
   (no deadlock), and nobody enters a cell before the robot planned there first has left (no collision) - whatever
   the real speeds. A robot only waits when the plan says it must, and for no longer than it takes the other to pass.

python fleet_plan.py  -> plans the 2026-11-04 test (robots in P1 / P3, cubes at S3 / S6), prints it, and runs a
                         delay simulation (random speeds) checking the dependencies keep the robots apart.
"""
import itertools, math, random
import arena_spec as spec

CELL_CM = 20          # cell length along a lane (edges are cut into ceil(length / CELL_CM) pieces)
SAFE_CM = 25          # two robots' cell centres closer than this at the same time = conflict (15 cm robots; a pivot
                      # sweeps a 21 cm circle. 25 lets a robot wait in a bay 25 cm from a mouth another one passes)
MARGIN_S = 2.0        # ... "the same time" = occupancy intervals closer than this (s): timing slack in the plan
SPEED_CMS = 18.0      # driving speed along a lane (measured 2026-11-04: ~14-20 cm/s with BASE_PWM 160)
TURN_S = 2.5          # extra time for a bend at a node (stop, pivot, settle)
REVERSE_S = 1.0       # extra time to change between forward and reverse (backing out of a bay / a dock)
LEG_START_S = 2.0     # extra time at the start of a leg (the --auto step gap, planning, first turn)
PICK_S = 8.0          # standing at the docking square for a pick (align turn, open, creep, close, back out)
DROP_S = 3.5          # standing in DZ for a drop
ROUTE_ALTS = 4        # routes tried per leg (shortest first, none longer than shortest + ROUTE_EXTRA_CM)
ROUTE_EXTRA_CM = 220
DEPART_STEP_S = 0.5   # leaving later is tried in these steps ...
DEPART_MAX_S = 150.0  # ... up to this much later


# ------------------------------------------------------------------ cells
def _build_cells():
    cells, index, nbrs = [], {}, {}

    def add(key, xy):
        if key not in index:
            index[key] = len(cells)
            cells.append(xy)
            nbrs[index[key]] = set()
        return index[key]

    for n, v in spec.NODES.items():
        add(("n", n), v["xy"])
    edge_cells = {}
    for a, nb in spec.EDGES.items():
        for b, L in nb.items():
            if a > b:
                continue
            k = max(1, math.ceil(L / CELL_CM))
            (ax, ay), (bx, by) = spec.NODES[a]["xy"], spec.NODES[b]["xy"]
            ids = [index[("n", a)]] + [add(("e", a, b, i), (ax + (bx - ax) * i / k, ay + (by - ay) * i / k))
                                       for i in range(1, k)] + [index[("n", b)]]
            edge_cells[(a, b)], edge_cells[(b, a)] = ids, ids[::-1]
            for p, q in zip(ids, ids[1:]):
                nbrs[p].add(q)
                nbrs[q].add(p)
    return cells, index, nbrs, edge_cells


CELLS, CELL_OF, NBRS, EDGE_CELLS = _build_cells()
NEAR = [[j for j in range(len(CELLS)) if math.dist(CELLS[i], CELLS[j]) < SAFE_CM] for i in range(len(CELLS))]


def node_cell(name):
    return CELL_OF[("n", name)]


def nearest_cell(x, y):
    return min(range(len(CELLS)), key=lambda i: math.dist(CELLS[i], (x, y)))


# ------------------------------------------------------------------ routes between nodes
def routes(a, b, alts=ROUTE_ALTS, extra=ROUTE_EXTRA_CM):
    """Up to `alts` simple routes a -> b (node names), shortest first, none longer than the shortest + extra cm."""
    best = spec.route(a, b)
    if best is None:
        return []
    limit = spec.path_length(best) + extra
    out = []

    def dfs(n, path, d):
        if d > limit:
            return
        if n == b:
            out.append((d, list(path)))
            return
        for m, w in spec.EDGES[n].items():
            if m not in path:
                path.append(m)
                dfs(m, path, d + w)
                path.pop()

    dfs(a, [a], 0.0)
    out.sort(key=lambda r: r[0])
    return [p for _, p in out[:alts]]


def route_cells(nodes):
    """Cells along a node route (no repeats)."""
    out = [node_cell(nodes[0])]
    for a, b in zip(nodes, nodes[1:]):
        out += EDGE_CELLS[(a, b)][1:]
    return out


def _dir(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def leg_timeline(nodes, t0, heading=None, dwell=0.0):
    """[(cell, t_in, t_out)] for driving `nodes` leaving at t0, then standing dwell s at the end.
    heading = the robot's direction (rad) at the start, if known (a bend / reverse before the first move costs)."""
    cs = route_cells(nodes)
    t, out = t0 + LEG_START_S, []
    prev_dir = heading
    for i, c in enumerate(cs):
        if i + 1 < len(cs):
            d = _dir(CELLS[c], CELLS[cs[i + 1]])
            extra = 0.0
            if prev_dir is not None:
                turn = abs((d - prev_dir + math.pi) % (2 * math.pi) - math.pi)
                if turn > math.radians(150):
                    extra = REVERSE_S                      # (backing out: no pivot, just change direction)
                elif turn > math.radians(30):
                    extra = TURN_S
            t_out = t + extra + math.dist(CELLS[c], CELLS[cs[i + 1]]) / SPEED_CMS / 2
            out.append((c, t if i else t0, t_out))         # (the start cell is held from t0: standing there)
            t = t_out + math.dist(CELLS[c], CELLS[cs[i + 1]]) / SPEED_CMS / 2
            prev_dir = d
        else:
            out.append((c, t if i else t0, t + dwell))
    return out, prev_dir


# ------------------------------------------------------------------ conflicts
def conflicts(tl, reserved):
    """Does timeline tl [(cell, t_in, t_out)] come closer than SAFE_CM to any reserved occupancy at the same time?
    reserved: {cell: [(t_in, t_out), ...]} of the other robots."""
    for c, a, b in tl:
        for nc in NEAR[c]:
            for ra, rb in reserved.get(nc, ()):
                if a < rb + MARGIN_S and ra < b + MARGIN_S:
                    return True
    return False


def reserve(reserved, tl, hold_until=None):
    for c, a, b in tl:
        reserved.setdefault(c, []).append((a, b))
    if hold_until is not None and tl:                      # parked at the end: in the way for good
        c, a, b = tl[-1]
        reserved.setdefault(c, []).append((b, hold_until))


# ------------------------------------------------------------------ one robot's mission against the others
def plan_robot(start_node, legs, reserved, t_start=0.0, heading=None):
    """legs = [(goal_node, dwell_s, action)] in order. Each leg: the route alternative + departure time that arrives
    first without a conflict (waiting where the robot stands). Returns (steps, leg_info, finish_time) or None."""
    here, t, steps, info, h = start_node, t_start, [], [], heading
    for goal, dwell, action in legs:
        best = None
        for nodes in routes(here, goal):
            t0 = t
            while t0 <= t + DEPART_MAX_S:
                tl, h_end = leg_timeline(nodes, t0, h, dwell)
                wait = ([(tl[0][0], t, t0)] if t0 > t else [])   # standing where it is until it leaves
                if not conflicts(wait + tl, reserved):
                    if best is None or tl[-1][2] < best[0]:
                        best = (tl[-1][2], nodes, wait + tl, h_end)
                    break
                t0 += DEPART_STEP_S
        if best is None:
            return None
        end, nodes, tl, h = best
        info.append(dict(goal=goal, action=action, nodes=nodes, depart=tl[0][1], arrive=end - dwell, end=end))
        steps += tl
        here, t = goal, end
    return steps, info, t


def merge_steps(steps):
    """Consecutive steps in the same cell -> one step (the robot's path is a list of distinct consecutive cells)."""
    out = []
    for c, a, b in steps:
        if out and out[-1][0] == c:
            out[-1] = (c, out[-1][1], b)
        else:
            out.append((c, a, b))
    return out


def dependencies(plans):
    """Action dependency graph. plans: {robot: [(cell, t_in, t_out)]} (merged). Returns {robot: {j: [(other, k)]}}:
    robot may ENTER its step j once `other` has reached its step k (left the neighbourhood of j's cell for the last
    time it was planned to be there before)."""
    deps = {r: {} for r in plans}
    for r, st in plans.items():
        for o, so in plans.items():
            if o == r:
                continue
            for j, (c, a, b) in enumerate(st):
                near = set(NEAR[c])
                last = None
                for i, (oc, oa, ob) in enumerate(so):      # o's last visit near c that ends before r arrives
                    if oc in near and ob <= a + 1e-6:
                        last = i
                if last is None:
                    continue
                k = last + 1
                while k < len(so) and so[k][0] in near:
                    k += 1
                if k >= len(so):                           # o stays near c for good (parked): r can never go there
                    k = len(so) - 1
                deps[r].setdefault(j, []).append((o, k))
    return deps


# ------------------------------------------------------------------ the whole fleet
def plan_fleet(robots, cubes, t_start=0.0):
    """robots: {name: dict(start=node, home=node, heading=rad or None, todo=[...optional fixed legs first])}
    cubes: [station, ...] still to fetch. Tries every assignment / order / priority. Returns the best plan dict:
    makespan, total, per robot: legs (goal, action, nodes, times), steps (merged cells + times), deps."""
    names = list(robots)
    best = None
    for owner in itertools.product(names, repeat=len(cubes)):
        mine = {r: [c for c, o in zip(cubes, owner) if o == r] for r in names}
        for orders in itertools.product(*[list(itertools.permutations(mine[r])) for r in names]):
            for prio in itertools.permutations(names):
                reserved, out, ok = {}, {}, True
                for r in prio:
                    legs = list(robots[r].get("todo", []))
                    for st in orders[names.index(r)]:
                        legs += [(st, PICK_S, "PICK"), ("DZ", DROP_S, "DROP")]
                    legs.append((robots[r]["home"], 0.0, None))
                    p = plan_robot(robots[r]["start"], legs, reserved, t_start, robots[r].get("heading"))
                    if p is None:
                        ok = False
                        break
                    steps, info, fin = p
                    reserve(reserved, steps, hold_until=1e9)
                    out[r] = dict(steps=steps, legs=info, finish=fin)
                if not ok:
                    continue
                makespan = max(v["finish"] for v in out.values())
                total = sum(v["finish"] for v in out.values())
                if best is None or (makespan, total) < (best["makespan"], best["total"]):
                    best = dict(makespan=makespan, total=total, robots=out, prio=prio,
                                assign={r: list(orders[names.index(r)]) for r in names})
    if best is None:
        return None
    merged = {r: merge_steps(v["steps"]) for r, v in best["robots"].items()}
    deps = dependencies(merged)
    for r, v in best["robots"].items():
        v["steps"], v["deps"] = merged[r], deps[r]
    return best


def describe(plan):
    lines = [f"makespan {plan['makespan']:.0f} s (total {plan['total']:.0f} s), planning order {list(plan['prio'])}, "
             f"cubes {plan['assign']}"]
    for r, v in plan["robots"].items():
        lines.append(f"  {r}: done at {v['finish']:.0f} s")
        for L in v["legs"]:
            lines.append(f"    {L['goal']:3s} {L['action'] or '':4s} leave {L['depart']:5.1f} arrive {L['arrive']:5.1f}"
                         f"  via {' '.join(L['nodes'])}")
        n_dep = sum(len(x) for x in v["deps"].values())
        lines.append(f"    {len(v['steps'])} cells, {n_dep} waits-for-the-other rules")
    return "\n".join(lines)


# ------------------------------------------------------------------ execution simulator (delays)
def simulate(plan, speed_jitter=0.35, pick_extra=(0, 12), seed=0, dt=0.1):
    """Each robot walks its cells with a random speed factor and random extra pick time, obeying the dependencies.
    Returns (finished, closest distance between robots seen, sim time)."""
    rnd = random.Random(seed)
    names = list(plan["robots"])
    st = {r: plan["robots"][r]["steps"] for r in names}
    dep = {r: plan["robots"][r]["deps"] for r in names}
    k = {r: 0 for r in names}
    wait_until = {r: 0.0 for r in names}
    fac = {r: 1 + rnd.uniform(-speed_jitter, speed_jitter) for r in names}
    t, closest = 0.0, 1e9
    planned_dwell = {r: [max(0.0, b - a) for c, a, b in st[r]] for r in names}
    while t < 2000:
        if all(k[r] >= len(st[r]) - 1 for r in names):
            return True, closest, t
        for r in names:
            if k[r] >= len(st[r]) - 1 or t < wait_until[r]:
                continue
            nxt = k[r] + 1
            if all(k[o] >= kk for o, kk in dep[r].get(nxt, [])):
                k[r] = nxt
                extra = rnd.uniform(*pick_extra) if planned_dwell[r][nxt] >= PICK_S - 0.1 else 0.0
                wait_until[r] = t + planned_dwell[r][nxt] * fac[r] + extra
        for a, b in itertools.combinations(names, 2):
            closest = min(closest, math.dist(CELLS[st[a][k[a]][0]], CELLS[st[b][k[b]][0]]))
        t += dt
    return False, closest, t


if __name__ == "__main__":
    robots = {"bot2": dict(start="P1", home="P1"), "bot1": dict(start="P3", home="P3")}
    p = plan_fleet(robots, ["S3", "S6"])
    print(describe(p))
    worst, fails = 1e9, 0
    for s in range(300):
        ok, closest, t = simulate(p, seed=s)
        fails += not ok
        worst = min(worst, closest)
    print(f"delay simulation: 300 runs, {fails} deadlocks/timeouts, closest robots {worst:.1f} cm (cell centres; "
          f"SAFE_CM {SAFE_CM})")
