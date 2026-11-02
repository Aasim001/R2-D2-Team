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

# ---- floor markers for automatic webcam calibration: ArUco id -> centre (x, y) cm of the BLACK square,
# flat on the floor inside a block (robots never drive there), clear of every bay and alcove.
# Near the OUTER corners of the blocks, as far apart as possible: the calibration is worked out from
# these 4 points only, so the further apart they are, the less a few mm of placement error matters
# at the ring (about 0.5 cm instead of 1.3-2.7 cm with markers bunched in the middle). Black square ~15 cm.
FLOOR_MARKERS = {10: (35, 35), 11: (265, 35), 12: (35, 230), 13: (265, 230)}


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


def is_keepout(x, y):
    """True if the floor point (cm) is somewhere a robot's body must not be."""
    if not (0 <= x <= SIZE and 0 <= y <= SIZE):
        return True
    for x0, y0, x1, y1 in drivable_rects():
        if x0 < x < x1 and y0 < y < y1:                    # inside a bay (its open end is on the block edge,
            return False                                   # which is not inside the block either)
    return any(x0 < x < x1 and y0 < y < y1 for x0, y0, x1, y1 in keepout_rects())


def keepout_depth(x, y):
    """How far (cm) a floor point is inside keep-out: its distance to the nearest lane strip or bay
    (0 = it is on a lane / in a bay)."""
    if not is_keepout(x, y):
        return 0.0
    free = ([(lx - LANE / 2, 0, lx + LANE / 2, SIZE) for lx in LANE_X] +
            [(0, ly - LANE / 2, SIZE, ly + LANE / 2) for ly in LANE_Y] + drivable_rects())
    return min(math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)) for x0, y0, x1, y1 in free)


def heading_for_pick(station, pick_side="right"):
    """Robot heading (deg) on the docking square so its pick side faces the alcove.
    pick_side = where the pick mechanism is: "right" or "left" side of the robot, or "front" (nose to the alcove)."""
    to_alcove = DIRS[STATIONS[station]["alcove"]]
    if pick_side == "front":
        return to_alcove
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


def route(start, goal):
    """Shortest path (list of node names) between two nodes along the lanes (Dijkstra)."""
    dist, prev, todo = {start: 0.0}, {}, [(0.0, start)]
    while todo:
        d, n = heapq.heappop(todo)
        if n == goal:
            break
        if d > dist[n]:
            continue
        for m, w in EDGES[n].items():
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
    """Closest point (cm) on any lane centre line or bay axis, and the distance to it."""
    best = None
    segs = [((lx, 10), (lx, 290)) for lx in LANE_X] + [((10, ly), (290, ly)) for ly in LANE_Y]
    if CUTOUTS:
        segs += [(b["mouth"], b["point"]) for b in BAYS.values()]
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
