"""Regression check (no robot, no camera): re-running the H test must NOT double-count the heading trim.
'H facing ...' lines = the real heading_test_result; run() errors should all stay under ~0.5 deg.
(Written by a review agent on 2026-10-07; the 'proposed fix' lines compare alternatives.)
usage: python tools/h_test_trim_check.py"""
import os, sys, math, types
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # laptop/
import arena_drive as ad

ad.save_calib = lambda c: None          # never touch the real calib file

OFF_TRUE_MAP = math.radians(-177.0)     # true triangle-tip -> robot-front offset (map rad)
E_ERR = [-6.5, 0.3, 0.0, 0.0]           # triangle reading error (official deg) facing E,N,W,S
TC_TRUE = (-33.0, 12.0)                 # true turning point (fwd, left) mm from the triangle


def interp(vals, o):
    o = o % 360
    i, frac = int(o // 90) % 4, (o % 90) / 90
    return (1 - frac) * vals[i] + frac * vals[(i + 1) % 4]


def raw_map(O):                         # triangle raw map angle when the robot truly faces official O
    return -math.radians(O + interp(E_ERR, O)) - OFF_TRUE_MAP


def mark_at(centre, O):                 # physical triangle position for turning point + true heading
    h = -math.radians(O)
    c, s = math.cos(h), math.sin(h)
    f, l = TC_TRUE
    return np.array([centre[0] - (f * c + l * s), centre[1] - (f * s - l * c)])


def make_log(O0, start=(1500.0, 1500.0)):
    log, fps = [], 17.0
    h0 = -math.radians(O0)
    fwd = np.array([math.cos(h0), math.sin(h0)])
    for k in range(int(4.4 * fps)):
        t = k / fps
        d, dO = 0.0, 0.0
        if 0.3 <= t < 0.9: d = 105 * (t - 0.3) / 0.6
        elif 0.9 <= t < 1.4: d = 105
        elif 1.4 <= t < 2.0: d = 105 * (1 - (t - 1.4) / 0.6)
        if 2.5 <= t < 2.9: dO = -38 * (t - 2.5) / 0.4          # right turn: official angle falls
        elif 2.9 <= t < 3.5: dO = -38
        elif 3.5 <= t < 3.9: dO = -38 * (1 - (t - 3.5) / 0.4)
        centre = np.array(start) + d * fwd
        O = O0 + dO
        log.append((t, mark_at(centre, O), raw_map(O)))
    return log


def new_arena(trims, offset_deg):
    a = types.SimpleNamespace()
    a.trims = list(trims)
    a.heading_offset = math.radians(offset_deg)
    a.turn_sign = 1
    a.turn_centres = {"left": np.array([-29.2, 9.4]), "right": np.array([-36.4, 15.6])}
    a.turn_centre = (a.turn_centres["left"] + a.turn_centres["right"]) / 2
    return a


def run_heading(arena, O):              # exactly what run() does with a triangle reading
    h = raw_map(O) + arena.heading_offset
    h += ad.heading_trim(arena, h)
    return (-math.degrees(h)) % 360


def errs(arena):
    return [round(((run_heading(arena, O) - O + 180) % 360) - 180, 2) for O in (0, 90, 180, 270)]


# baseline: offset as the old south H test gave (e(S) = 0), trims as in arena_calib.json
off_south = math.degrees(OFF_TRUE_MAP)  # offset = off_true + e(S) = off_true
base = new_arena([6.5, -0.3, 0.0, 0.0], off_south)
print("calib now (south H, trims E+6.5 N-0.3): run() error E,N,W,S =", errs(base))

for O0 in (350.0, 0.0, 265.0, 90.0):
    arena = new_arena([6.5, -0.3, 0.0, 0.0], off_south)
    calib = {"heading_offset_deg": round(off_south, 1), "turn_sign": 1}
    ad.heading_test_result(make_log(O0), calib, arena)
    print(f"  H facing {O0:5.1f}: offset {off_south:+.1f} -> {calib['heading_offset_deg']:+.1f};"
          f" run() error E,N,W,S = {errs(arena)};  turn centre {arena.turn_centre.round(1)}")

# proposed fix (re-implemented here, as a fixed-point iteration) - does it remove the error?
print("\nproposed fix (offset -= trim at the H direction):")
for O0 in (350.0, 0.0, 265.0, 90.0, 45.0):
    arena = new_arena([6.5, -0.3, 0.0, 0.0], off_south)
    log = make_log(O0)
    win = lambda a, b: [p for p in log if a <= p[0] < b]
    start, moved = win(0, 0.3), win(1.1, 1.4)
    motion = np.mean([p[1] for p in moved], 0) - np.mean([p[1] for p in start], 0)
    tag_dir = ad.circ_mean([p[2] for p in start + moved])
    travel = math.atan2(motion[1], motion[0])
    off0 = ad.wrap(travel - tag_dir)
    one = off0 - ad.heading_trim(arena, tag_dir + off0)                 # the reviewer's one-step fix
    off = off0
    for _ in range(5):                                                  # fixed point
        off = ad.wrap(travel - tag_dir - ad.heading_trim(arena, tag_dir + off))
    for name, o in (("one-step", one), ("fixed-point", off)):
        arena.heading_offset = o
        print(f"  H facing {O0:5.1f} {name:11s}: run() error E,N,W,S = {errs(arena)}")
    arena.heading_offset, arena.trims = off0, [0.0] * 4
    print(f"  H facing {O0:5.1f} reset-trims : run() error E,N,W,S = {errs(arena)}")

# how fast does the learning undo it? (trim learner: gain 0.3 per 15 cm stretch, residual = error)
arena = new_arena([6.5, -0.3, 0.0, 0.0], off_south)
calib = {"heading_offset_deg": round(off_south, 1), "turn_sign": 1}
ad.heading_test_result(make_log(350.0), calib, arena)
n = 0
while abs(errs(arena)[0]) > 1.0 and n < 50:
    n += 1
    e = errs(arena)[0]
    arena.trims[0] -= 0.3 * e
print(f"\nafter H facing 350: {n} straight 15 cm east stretches to get the east error under 1 deg")
