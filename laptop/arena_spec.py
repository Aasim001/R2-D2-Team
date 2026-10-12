#!/usr/bin/env python3
"""
The official competition floor, from the participant handout (problemstatementcontext.txt).
The arena is identical at every venue, so everything here is fixed.

Units: CENTIMETRES, official frame: origin = bottom-left corner of the taped floor,
x increases east, y increases north. Heading: degrees, 0 = east, counter-clockwise positive.

  python arena_spec.py            # prints the lane graph and a route example, saves arena_spec.png
"""
import heapq, math

SIZE = 300                    # 3 m x 3 m
LANE = 20                     # every lane is exactly 20 cm wide

# Are the bays (DZ, P1-P3) and alcoves actually cut out of the taped blocks on YOUR floor?
#   False = practice floor with solid blocks (no cut-outs yet): the bays count as keep-out and are
#           left out of the routes; a goal at a bay means its MOUTH (the spot on the ring in front of it).
#           Stations still work - their docking squares are in the lane.
#   True  = the official floor (bays drivable: drive in forward, reverse out).
CUTOUTS = True                # 2026-10-08: the owner taped P1-P3 + DZ; they match this plan within ~1 cm

# ---- keep-out blocks (x0, y0, x1, y1). Treat as solid shelving: a robot must never cross one.
# A = top-left, C = bottom-left, D = bottom-right are named in the handout; B (top-right) is the remaining one.
BLOCKS = {
    "A": (20, 160, 140, 280),
    "B": (160, 160, 280, 280),
    "C": (20, 20, 140, 140),
    "D": (160, 20, 280, 140),
}

# ---- lane centre lines: the perimeter ring and the cross through the middle
LANE_X = (10, 150, 290)       # north-south lanes run along these x
LANE_Y = (10, 150, 290)       # east-west lanes run along these y

# ---- bays: dead-end recesses cut into a block. Drive fully in (forward), reverse out.
#   rect = (x0, y0, x1, y1), mouth = where it meets the lane centre line, point = where the robot stops
BAYS = {
    "DZ": dict(rect=(60, 20, 80, 80), mouth=(70, 10), point=(70, 50)),       # dispatch, 20 x 60, off the bottom ring
    "P1": dict(rect=(40, 250, 60, 280), mouth=(50, 290), point=(50, 265)),   # parking, 20 x 30, off the top ring
    "P2": dict(rect=(240, 250, 260, 280), mouth=(250, 290), point=(250, 265)),
    "P3": dict(rect=(200, 20, 220, 50), mouth=(210, 10), point=(210, 35)),   # off the bottom ring
}

# ---- pick stations: docking square (20 x 20, in the lane) + alcove (20 wide x 10 deep, cut into a block).
#   alcove = direction from the docking square to the alcove ("N", "S", "E", "W").
#   All six checked against Figure 1 of the handout (arenaimg.jpg): the cube sits on this side of the
#   docking square. Block A = top-left, B = top-right, C = bottom-left, D = bottom-right.
STATIONS = {
    "S1": dict(dock=(150, 240), alcove="W"),     # vertical lane, cut into Block A (its east edge)
    "S2": dict(dock=(150, 90), alcove="E"),      # vertical lane, cut into Block D (its west edge)
    "S3": dict(dock=(50, 150), alcove="N"),      # horizontal lane, cut into Block A (its bottom edge)
    "S4": dict(dock=(240, 150), alcove="S"),     # horizontal lane, cut into Block D (its top edge)
    "S5": dict(dock=(190, 290), alcove="S"),     # top ring, cut into Block B (its top edge)
    "S6": dict(dock=(10, 200), alcove="E"),      # left ring, cut into Block A (its west edge)
}
ASSUMED = set()                                  # nothing assumed any more (Figure 1)

DIRS = {"E": 0.0, "N": 90.0, "W": 180.0, "S": 270.0}

# ---- floor anchors for automatic webcam calibration: ArUco (DICT_4X4_50) id -> centre (x, y) cm of the BLACK
# square, flat on a block (robots never drive there), clear of every bay and alcove. The team's plan
# (2026-10-10): ids 0-3, one per block near its outer corner, ~2.2 m x 1.9 m apart - the further apart, the less
# a few mm of placement error matters at the ring. Sheet 19 x 19 cm, black square 14 x 14 cm.
# Robot markers use ids 10-12 (R1-R3), so they can't be mistaken for an anchor.
FLOOR_MARKERS = {0: (40, 40), 1: (260, 40), 2: (260, 230), 3: (40, 230)}   # C, D, B, A
FLOOR_MARKER_CM = 14.0        # side of an anchor's black square


def alcove_rect(name):
    """The 20 x 10 cm recess for a station (part of the keep-out area for the robot's body)."""
    s = STATIONS[name]
    x, y = s["dock"]
    d = s["alcove"]
    if d == "W":
        return (x - 20, y - 10, x - 10, y + 10)
    if d == "E":
        return (x + 10, y - 10, x + 20, y + 10)
    if d == "N":
        return (x - 10, y + 10, x + 10, y + 20)
    return (x - 10, y - 20, x + 10, y - 10)


def keepout_rects():
    """Rectangles the robot's body must stay out of: the blocks, minus nothing - the bays are
    returned separately as drivable areas (see drivable_rects). Alcoves stay keep-out for the body."""
    return list(BLOCKS.values())


def drivable_rects():
    """Areas inside blocks that a robot may drive into: the bays (none if they aren't cut out yet)."""
    return [b["rect"] for b in BAYS.values()] if CUTOUTS else []


def goal_node(name):
    """Graph node to drive to for a goal name. Without cut-outs, a bay goal means the bay's mouth."""
    return name if (CUTOUTS or name not in BAYS) else name + "_mouth"


# ============================ THE REAL TAPE + A SAFETY MARGIN ============================
# A taped floor is never exactly the plan (the practice floor: up to ~2 cm off). arena_drive measures the
# lane-side edge of every tape line in the camera picture (measure_tape) and calls set_tape(): from then on the
# keep-out test, the lane centre lines and the graph nodes follow the REAL tape. And every tape line counts
# SAFETY_CM thicker into the lane, so the robot keeps a gap from it ("a little less room to wiggle, but it never
# crosses the line"). Without a measurement the tape edges are taken to be on the official block edges.
SAFETY_CM = 1.0               # every tape line counts this much thicker (into the lane)
ROBOT_W_CM, ROBOT_L_CM = 15.0, 13.0                  # robot outline (wheels included)
SWEEP_CM = 0.5 * math.hypot(ROBOT_W_CM, ROBOT_L_CM)  # radius its corners sweep when it turns on the spot


def _tape_sides():
    """Every straight tape line that bounds a lane: name -> dict(a, b = ends on the official edge (cm),
    n = unit normal pointing INTO the lane, length, skip = along-intervals (cm from a) with no straight tape:
    the openings of bays and alcoves)."""
    sides = {}
    for b, (x0, y0, x1, y1) in BLOCKS.items():
        sides[b + ".N"] = dict(a=(x0, y1), b=(x1, y1), n=(0.0, 1.0))
        sides[b + ".S"] = dict(a=(x0, y0), b=(x1, y0), n=(0.0, -1.0))
        sides[b + ".E"] = dict(a=(x1, y0), b=(x1, y1), n=(1.0, 0.0))
        sides[b + ".W"] = dict(a=(x0, y0), b=(x0, y1), n=(-1.0, 0.0))
    sides["OUT.N"] = dict(a=(0, SIZE), b=(SIZE, SIZE), n=(0.0, -1.0))
    sides["OUT.S"] = dict(a=(0, 0), b=(SIZE, 0), n=(0.0, 1.0))
    sides["OUT.E"] = dict(a=(SIZE, 0), b=(SIZE, SIZE), n=(-1.0, 0.0))
    sides["OUT.W"] = dict(a=(0, 0), b=(0, SIZE), n=(1.0, 0.0))
    openings = [b["rect"] for b in BAYS.values()] + [alcove_rect(n) for n in STATIONS]
    for name, s in sides.items():
        (ax, ay), (bx, by) = s["a"], s["b"]
        s["length"] = math.hypot(bx - ax, by - ay)
        s["skip"] = []
        if name.startswith("OUT"):
            continue
        for x0, y0, x1, y1 in openings:                    # an opening cut into this side of the block
            if ay == by and ay in (y0, y1) and x0 < bx and x1 > ax:
                s["skip"].append((x0 - ax - 3, x1 - ax + 3))
            if ax == bx and ax in (x0, x1) and y0 < by and y1 > ay:
                s["skip"].append((y0 - ay - 3, y1 - ay + 3))
    return sides


SIDES = _tape_sides()
TAPE_EDGES = {}               # side name -> (offset mm at the side's middle, slope mm per cm): where the lane-side
                              # edge of the tape really is, + = into the lane from the official edge (set_tape)
_DEFAULT_OFF = {"block": 0.0, "out": 0.0}
_GEOM_CACHE, _PIVOT_CACHE = {}, {}


def edge_offset_cm(name, t):
    """Measured lane-side tape edge of a side, cm into the lane from the official edge, t cm along it from a."""
    if name in TAPE_EDGES:
        off, slope = TAPE_EDGES[name]
    else:
        off, slope = _DEFAULT_OFF["out" if name.startswith("OUT") else "block"], 0.0
    return (off + slope * (t - SIDES[name]["length"] / 2)) / 10.0


def _side_line(name, safety):
    """The side's tape edge moved `safety` cm further into the lane: two points (cm)."""
    s = SIDES[name]
    (ax, ay), (bx, by), (nx, ny) = s["a"], s["b"], s["n"]
    da, db = edge_offset_cm(name, 0) + safety, edge_offset_cm(name, s["length"]) + safety
    return (ax + nx * da, ay + ny * da), (bx + nx * db, by + ny * db)


def _meet(l1, l2):
    """Where two (nearly perpendicular) lines meet."""
    (x1, y1), (x2, y2) = l1
    (x3, y3), (x4, y4) = l2
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def _geometry(safety):
    """Tape polygons with the margin: the 4 blocks grown by `safety`, and the area inside the outer tape shrunk
    by it (corners NW, NE, SE, SW). segs = the blocks' outlines as segments, with each bay's opening cut out and
    its walls (moved `safety` into the bay) added - the edge of the free floor inside the arena."""
    g = _GEOM_CACHE.get(safety)
    if g is None:
        quad = lambda p: [_meet(_side_line(p + ".W", safety), _side_line(p + ".N", safety)),
                          _meet(_side_line(p + ".N", safety), _side_line(p + ".E", safety)),
                          _meet(_side_line(p + ".E", safety), _side_line(p + ".S", safety)),
                          _meet(_side_line(p + ".S", safety), _side_line(p + ".W", safety))]
        blocks = {b: quad(b) for b in BLOCKS}
        segs = []
        for b, (nw, ne, se, sw) in blocks.items():
            north, south = [(nw, ne)], [(sw, se)]
            bx0, by0, bx1, by1 = BLOCKS[b]
            for bay in (BAYS.values() if CUTOUTS else []):
                x0, y0, x1, y1 = bay["rect"]
                if not (bx0 <= x0 and x1 <= bx1 and by0 <= y0 and y1 <= by1):
                    continue
                opens_north = bay["mouth"][1] > y1
                side = north if opens_north else south
                p, q = side.pop()                          # (one bay per block side on this floor)
                lo, hi = x0 + safety, x1 - safety
                ylo = p[1] + (q[1] - p[1]) * (lo - p[0]) / (q[0] - p[0])
                yhi = p[1] + (q[1] - p[1]) * (hi - p[0]) / (q[0] - p[0])
                side += [(p, (lo, ylo)), ((hi, yhi), q)]
                back = y0 + safety if opens_north else y1 - safety
                segs += [((lo, back), (lo, ylo)), ((hi, back), (hi, yhi)), ((lo, back), (hi, back))]
            segs += north + south + [(ne, se), (sw, nw)]
        g = _GEOM_CACHE[safety] = dict(blocks=blocks, outer=quad("OUT"), segs=segs)
    return g


def _inside(poly, x, y):
    """Inside a convex polygon whose corners go round clockwise (y up)."""
    return all((bx - ax) * (y - ay) - (by - ay) * (x - ax) <= 0
               for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]))


def _seg_dist(x, y, a, b):
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(ax + t * dx - x, ay + t * dy - y)


def _poly_dist(poly, x, y):
    return min(_seg_dist(x, y, a, b) for a, b in zip(poly, poly[1:] + poly[:1]))


def _bay_depth(x, y, safety):
    """(True, depth) if the point is in a bay or its mouth (its side and back walls count `safety` thicker)."""
    if not CUTOUTS:
        return False, 0.0
    for name, b in BAYS.items():
        x0, y0, x1, y1 = b["rect"]
        opens_north = b["mouth"][1] > y1
        side = next(k for k, (bx0, by0, bx1, by1) in BLOCKS.items()
                    if bx0 <= x0 and x1 <= bx1 and by0 <= y0 and y1 <= by1) + (".N" if opens_north else ".S")
        reach = max(0.0, edge_offset_cm(side, x - SIDES[side]["a"][0]))   # measured tape further into the lane
        ext = safety + 2.0 + reach                         # (the strip between the bay and its lane too)
        if x0 <= x <= x1 and (y0 if opens_north else y0 - ext) <= y <= (y1 + ext if opens_north else y1):
            back = y0 if opens_north else y1
            walls = [((x0, y0), (x0, y1)), ((x1, y0), (x1, y1)), ((x0, back), (x1, back))]   # tape segments
            return True, max(0.0, safety - min(_seg_dist(x, y, a, b) for a, b in walls))
    return False, 0.0


def keepout_depth(x, y, safety=None):
    """How far (cm) a floor point is over a tape line counted `safety` cm thick (default SAFETY_CM; 0 = the real
    tape edge). 0 = clear of it. Blocks (with their alcoves) and the outer boundary; bays are drivable."""
    safety = SAFETY_CM if safety is None else safety
    g = _geometry(safety)
    if not _inside(g["outer"], x, y):
        return _poly_dist(g["outer"], x, y)
    in_bay, depth = _bay_depth(x, y, safety)
    if in_bay:
        return depth
    if any(_inside(q, x, y) for q in g["blocks"].values()):
        return min(_seg_dist(x, y, a, b) for a, b in g["segs"])   # to the nearest free floor (lane or bay)
    return 0.0


def is_keepout(x, y):
    """True if the floor point (cm) is over a real tape line (no margin)."""
    return keepout_depth(x, y, safety=0) > 0


def room(x, y, safety=None):
    """Distance (cm) from a floor point to the nearest tape counted `safety` thick (negative = over it by that
    much). Bay openings count as free floor."""
    safety = SAFETY_CM if safety is None else safety
    g = _geometry(safety)
    if not _inside(g["outer"], x, y):
        return -_poly_dist(g["outer"], x, y)
    d = min(_poly_dist(g["outer"], x, y), min(_seg_dist(x, y, a, b) for a, b in g["segs"]))
    return -d if keepout_depth(x, y, safety) > 0 else d


def pivot_spot(xy, u, lateral_max):
    """Where to stop for an on-the-spot turn at the node xy, arriving along the unit direction u: (shortfall cm
    before the node, offset cm to the LEFT of the way it travels) that gives the circle its corners sweep
    (SWEEP_CM) the most room from the thick tape. Plus that room (cm, negative = the circle crosses the thick
    tape). Centred at a T-junction on the ring the circle all but touches the outer tape; ~2.5 cm short of it,
    it keeps ~1 cm from the thick tape. lateral_max: how far off-centre the robot can drive in its lane."""
    key = (round(xy[0], 2), round(xy[1], 2), round(u[0], 3), round(u[1], 3), round(lateral_max, 2))
    if key not in _PIVOT_CACHE:
        best = None
        for a in [i * 0.25 for i in range(-4, 17)]:                        # -1 .. 4 cm short
            for l in [i * 0.25 for i in range(-8, 9) if abs(i * 0.25) <= lateral_max + 1e-9]:
                cx, cy = xy[0] - u[0] * a - u[1] * l, xy[1] - u[1] * a + u[0] * l
                r = room(cx, cy) - SWEEP_CM
                score = r - 0.02 * (abs(a) + abs(l))                       # (no shift unless it helps)
                if best is None or score > best[0]:
                    best = (score, a, l, r)
        _PIVOT_CACHE[key] = best[1:]
    return _PIVOT_CACHE[key]


def heading_for_pick(station, pick_side="right"):
    """Robot heading (deg) on the docking square so its pick side faces the alcove.
    pick_side = which side of the robot the arm reaches out of ("right" or "left")."""
    to_alcove = DIRS[STATIONS[station]["alcove"]]
    return (to_alcove + (90 if pick_side == "right" else -90)) % 360   # right side = heading - 90


# ============================ LANE GRAPH ============================
def _build_graph():
    """Nodes: lane junctions, docking squares, bay mouths and bay stop points.
    Edges: straight runs along lane centre lines (and mouth <-> bay point)."""
    nodes = {}

    def add(name, xy, kind):
        nodes[name] = dict(xy=xy, kind=kind)

    for x in LANE_X:
        for y in LANE_Y:
            add(f"J{x},{y}", (x, y), "junction")
    for n, s in STATIONS.items():
        add(n, s["dock"], "dock")
    for n, b in BAYS.items():
        add(n + "_mouth", b["mouth"], "mouth")
        if CUTOUTS:                                        # no cut-out = no way into the bay
            add(n, b["point"], "bay")

    edges = {n: {} for n in nodes}

    def link(a, b):
        (ax, ay), (bx, by) = nodes[a]["xy"], nodes[b]["xy"]
        d = math.hypot(ax - bx, ay - by)
        edges[a][b] = edges[b][a] = d

    on_lane = [n for n, v in nodes.items() if v["kind"] != "bay"]
    for x in LANE_X:                                       # north-south lanes
        line = sorted((v["xy"][1], n) for n in on_lane for v in [nodes[n]] if v["xy"][0] == x)
        for (_, a), (_, b) in zip(line, line[1:]):
            link(a, b)
    for y in LANE_Y:                                       # east-west lanes
        line = sorted((v["xy"][0], n) for n in on_lane for v in [nodes[n]] if v["xy"][1] == y)
        for (_, a), (_, b) in zip(line, line[1:]):
            link(a, b)
    for n in BAYS:
        if n in nodes:
            link(n + "_mouth", n)
    for n, v in nodes.items():                             # every lane node must sit on a lane centre line
        x, y = v["xy"]
        assert v["kind"] == "bay" or x in LANE_X or y in LANE_Y, n
    return nodes, edges


NODES, EDGES = _build_graph()
_OFFICIAL_XY = {n: v["xy"] for n, v in NODES.items()}

# the two tape sides that bound each lane: N-S lanes (x) -> ((west, east) for the half north of the cross,
# then the half south of it); E-W lanes (y) -> ((south, north) west of the cross, then east of it)
V_SIDES = {10: (("OUT.W", "A.W"), ("OUT.W", "C.W")), 150: (("A.E", "B.W"), ("C.E", "D.W")),
           290: (("B.E", "OUT.E"), ("D.E", "OUT.E"))}
H_SIDES = {10: (("OUT.S", "C.S"), ("OUT.S", "D.S")), 150: (("C.N", "A.S"), ("D.N", "B.S")),
           290: (("A.N", "OUT.N"), ("B.N", "OUT.N"))}


def _edge_coord(name, at):
    """x (a N-S side) or y (an E-W side), cm, of a side's real lane-side tape edge, at the official y / x `at`."""
    s = SIDES[name]
    (ax, ay), (bx, by) = s["a"], s["b"]
    if ax == bx:                                           # N-S side
        return ax + s["n"][0] * edge_offset_cm(name, at - ay)
    return ay + s["n"][1] * edge_offset_cm(name, at - ax)


def _v_centre(X, y):
    halves = [h for h, ok in ((0, y >= 150), (1, y <= 150)) if ok]     # (a junction: both halves)
    return sum((_edge_coord(w, y) + _edge_coord(e, y)) / 2 for w, e in (V_SIDES[X][h] for h in halves)) / len(halves)


def _h_centre(Y, x):
    halves = [h for h, ok in ((0, x <= 150), (1, x >= 150)) if ok]
    return sum((_edge_coord(s, x) + _edge_coord(n, x)) / 2 for s, n in (H_SIDES[Y][h] for h in halves)) / len(halves)


def set_tape(edges):
    """Use measured tape edges {side: (offset mm, slope mm/cm)} (arena_drive.measure_tape): from now on the lane
    centre lines - and so every junction, docking square and bay mouth - sit in the middle of the REAL tape, and
    keepout_depth / room / pivot_spot use it. Sides not measured get the most lane-ward offset measured (cautious). {} = the official plan."""
    global TAPE_EDGES
    TAPE_EDGES = {k: (float(v[0]), float(v[1])) for k, v in (edges or {}).items() if k in SIDES}
    for kind, is_out in (("block", False), ("out", True)):
        offs = [v[0] for k, v in TAPE_EDGES.items() if k.startswith("OUT") == is_out]
        _DEFAULT_OFF[kind] = max(offs + [0.0])             # unmeasured side: assume the narrowest lane seen
    _GEOM_CACHE.clear()
    _PIVOT_CACHE.clear()
    for n, v in NODES.items():
        x, y = _OFFICIAL_XY[n]
        if v["kind"] != "bay":                             # (bay stop points stay on the bay's own axis)
            v["xy"] = (_v_centre(x, y) if x in LANE_X else x, _h_centre(y, x) if y in LANE_Y else y)
    for a in EDGES:
        for b in EDGES[a]:
            (ax, ay), (bx, by) = NODES[a]["xy"], NODES[b]["xy"]
            EDGES[a][b] = math.hypot(ax - bx, ay - by)


def route(start, goal, blocked=(), blocked_edges=()):
    """Shortest path (list of node names) between two nodes along the lanes (Dijkstra).
    blocked = nodes it must not pass (e.g. where another robot stands; start and goal are never skipped),
    blocked_edges = lane segments it must not use, as frozenset({a, b}) (fleet.py: held by another robot)."""
    dist, prev, todo = {start: 0.0}, {}, [(0.0, start)]
    while todo:
        d, n = heapq.heappop(todo)
        if n == goal:
            break
        if d > dist[n]:
            continue
        for m, w in EDGES[n].items():
            if (m in blocked and m != goal) or frozenset((n, m)) in blocked_edges:
                continue
            if d + w < dist.get(m, math.inf):
                dist[m], prev[m] = d + w, n
                heapq.heappush(todo, (d + w, m))
    if goal not in dist:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return path[::-1]


def nearest_node(x, y, lane_only=True):
    """Name of the closest graph node to a floor point (cm)."""
    best = None
    for n, v in NODES.items():
        if lane_only and v["kind"] == "bay":
            continue
        d = math.hypot(v["xy"][0] - x, v["xy"][1] - y)
        if best is None or d < best[0]:
            best = (d, n)
    return best[1]


def project_to_lane(x, y):
    """Closest point (cm) on any lane centre line or bay axis (the graph's edges: they follow the measured
    tape after set_tape), and the distance to it."""
    best = None
    segs = [(NODES[a]["xy"], NODES[b]["xy"]) for a, nb in EDGES.items() for b in nb if a < b]
    for (ax, ay), (bx, by) in segs:
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(px - x, py - y)
        if best is None or d < best[0]:
            best = (d, (px, py))
    return best[1], best[0]


def path_length(names):
    return sum(EDGES[a][b] for a, b in zip(names, names[1:]))


def route_from_point(x, y, goal):
    """Route from any point on a lane (or in a bay) to a goal node.
    Returns (points, names): points[0] = the robot's position projected onto the lane centre line,
    then the node positions along the shortest route. None if the goal can't be reached."""
    (px, py), _ = project_to_lane(x, y)
    best = None
    for a, nb in EDGES.items():
        for b in nb:
            if a > b:                                      # each edge once
                continue
            (ax, ay), (bx, by) = NODES[a]["xy"], NODES[b]["xy"]
            dx, dy = bx - ax, by - ay
            t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
            if not -1e-6 <= t <= 1 + 1e-6 or math.hypot(ax + t * dx - px, ay + t * dy - py) > 0.5:
                continue                                   # the robot is not on this edge
            for end in (a, b):
                r = route(end, goal)
                if r is None:
                    continue
                cost = math.hypot(NODES[end]["xy"][0] - px, NODES[end]["xy"][1] - py) + path_length(r)
                if best is None or cost < best[0]:
                    best = (cost, r)
    if best is None:
        return None
    names = best[1]
    pts = [(px, py)] + [NODES[n]["xy"] for n in names]
    if math.hypot(pts[1][0] - px, pts[1][1] - py) < 0.5:   # already standing on the first node
        pts = pts[1:]
    return pts, names


def bay_of(x, y, tol=0.5):
    """Name of the bay a point is inside (not counting its mouth on the lane), or None."""
    if not CUTOUTS:
        return None
    for n, b in BAYS.items():
        x0, y0, x1, y1 = b["rect"]
        if x0 - tol <= x <= x1 + tol and y0 - tol <= y <= y1 + tol:
            return n
    return None


def floor_image(px_per_cm=2):
    """Picture of the floor plan (north up), for checks and the live view. Needs OpenCV."""
    import cv2, numpy as np
    s = px_per_cm
    img = np.full((SIZE * s + 1, SIZE * s + 1, 3), 235, np.uint8)
    P = lambda x, y: (int(round(x * s)), int(round((SIZE - y) * s)))          # north up
    for x0, y0, x1, y1 in keepout_rects():
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (90, 90, 90), -1)
    for x0, y0, x1, y1 in drivable_rects():
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (200, 230, 200), -1)
    for n in STATIONS:
        x0, y0, x1, y1 = alcove_rect(n)
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (60, 160, 230), -1)
        x, y = STATIONS[n]["dock"]
        cv2.rectangle(img, P(x - 10, y + 10), P(x + 10, y - 10), (0, 140, 255), 1)
        cv2.putText(img, n + ("?" if n in ASSUMED else ""), P(x - 9, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35 * s, (0, 0, 0), 1)
    for n, b in BAYS.items():
        x, y = b["point"] if CUTOUTS else (b["mouth"][0], b["mouth"][1] + (-6 if b["mouth"][1] > 150 else 4))
        cv2.putText(img, n, P(x - 8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35 * s, (0, 90, 0), 1)
    for a, nb in EDGES.items():
        for b in nb:
            cv2.line(img, P(*NODES[a]["xy"]), P(*NODES[b]["xy"]), (255, 120, 0), 1)
    return img


if __name__ == "__main__":
    print(f"{len(NODES)} nodes, {sum(len(v) for v in EDGES.values()) // 2} edges")
    print(f"CUTOUTS = {CUTOUTS}" + ("" if CUTOUTS else "  (bays not cut out: bay goals stop at the bay's mouth)"))
    for a, b in (("P1", "S2"), ("P3", "S1"), ("S5", "DZ"), ("DZ", "P2")):
        a, b = goal_node(a), goal_node(b)
        r = route(a, b)
        length = sum(EDGES[p][q] for p, q in zip(r, r[1:]))
        print(f"{a} -> {b}: {length:.0f} cm  via {' > '.join(r)}")
    for n in STATIONS:
        print(f"{n}: dock {STATIONS[n]['dock']}, alcove {STATIONS[n]['alcove']}{' (ASSUMED)' if n in ASSUMED else ''},"
              f" robot heading for a right-side pick: {heading_for_pick(n):.0f} deg")
    try:
        import cv2
        cv2.imwrite("arena_spec.png", floor_image(3))
        print("saved arena_spec.png")
    except ImportError:
        pass
