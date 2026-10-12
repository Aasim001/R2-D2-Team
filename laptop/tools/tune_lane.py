"""Sweep lane_follow steering settings in the fast simulator (scrub / imbalance / heading bias scenarios).
usage: python tools/tune_lane.py"""
import os, sys, itertools, multiprocessing as mp
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROUTE = ["S4", "S5", "P1", "S6", "S3", "S1", "P2", "DZ", "P3", "S2"]
SCEN = [dict(scrub=s, imbalance=i, head_bias_deg=b) for s in (4, 10) for i in (1.0, 1.06) for b in (0, 4)]
GRID = dict(K_INT=(0, 0.3, 0.8, 1.5), STEER_MIN=(0, 10, 20, 30), K_CROSS=(0.10, 0.15, 0.20), K_HEAD=(0.35, 0.6),
            BASE_PWM=(130, 110))

def run(params):
    import lane_follow as lf
    for k, v in params.items():
        setattr(lf, k, v)
    res = [lf.simulate((150, 150), 270, ROUTE, verbose=False, seed=1, **sc) for sc in SCEN]
    return params, dict(rms=sum(r["lat_rms"] for r in res) / len(res), ko=sum(r["ko_frac"] for r in res) / len(res),
                        worst=max(r["worst"] for r in res), fails=sum(len(r["fails"]) for r in res),
                        stop=max(r["worst_stop"] for r in res))

if __name__ == "__main__":
    combos = [dict(zip(GRID, v)) for v in itertools.product(*GRID.values())]
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        out = pool.map(run, combos)
    out.sort(key=lambda o: (o[1]["fails"], o[1]["ko"] + 0.02 * o[1]["rms"]))
    base = [o for o in out if o[0] == dict(K_INT=0, STEER_MIN=0, K_CROSS=0.10, K_HEAD=0.35, BASE_PWM=130)][0]
    fmt = lambda o: (f"{o[0]}  -> keep-out {100*o[1]['ko']:.2f} % of time, lateral rms {o[1]['rms']:.2f} cm, "
                     f"worst {o[1]['worst']:.1f} cm, fails {o[1]['fails']}, worst stop {o[1]['stop']:.1f} cm")
    print("OLD SETTINGS:", fmt(base))
    print("BEST 12:")
    for o in out[:12]:
        print("  ", fmt(o))
