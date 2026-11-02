"""Stress-test candidate steering settings: more noise, more lag, with/without scrub, several seeds."""
import os, sys, multiprocessing as mp
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROUTE = ["S4", "S5", "P1", "S6", "S3", "S1", "P2", "DZ", "P3", "S2"]
CANDS = {
    "old":            dict(K_INT=0,   STEER_MIN=0,  K_CROSS=0.10, K_HEAD=0.35, BASE_PWM=130),
    "best":           dict(K_INT=0.3, STEER_MIN=30, K_CROSS=0.20, K_HEAD=0.6,  BASE_PWM=110),
    "mid":            dict(K_INT=0.3, STEER_MIN=20, K_CROSS=0.15, K_HEAD=0.6,  BASE_PWM=120),
    "mid130":         dict(K_INT=0.3, STEER_MIN=20, K_CROSS=0.20, K_HEAD=0.6,  BASE_PWM=130),
    "bigger":         dict(K_INT=0.3, STEER_MIN=40, K_CROSS=0.25, K_HEAD=0.8,  BASE_PWM=110),
    "min20_c20_110":  dict(K_INT=0.3, STEER_MIN=20, K_CROSS=0.20, K_HEAD=0.6,  BASE_PWM=110),
}
SCEN = [dict(scrub=s, imbalance=i, head_bias_deg=b, noise_cm=n, lag=lg)
        for s in (0, 6, 12) for i in (0.95, 1.06) for b in (0, 5) for n, lg in ((0.3, 4), (0.7, 6))]

def run(item):
    name, params, seed = item
    import lane_follow as lf
    for k, v in params.items():
        setattr(lf, k, v)
    res = [lf.simulate((150, 150), 270, ROUTE, verbose=False, seed=seed, **sc) for sc in SCEN]
    return name, seed, res

if __name__ == "__main__":
    items = [(n, p, s) for n, p in CANDS.items() for s in (1, 2)]
    with mp.Pool(7) as pool:
        out = pool.map(run, items)
    agg = {}
    for name, seed, res in out:
        agg.setdefault(name, []).extend(res)
    for name, res in agg.items():
        n = len(res)
        print(f"{name:14s} keep-out {100*sum(r['ko_frac'] for r in res)/n:5.2f} % (worst scenario {100*max(r['ko_frac'] for r in res):5.2f} %), "
              f"lateral rms {sum(r['lat_rms'] for r in res)/n:.2f} cm, worst depth {max(r['worst'] for r in res):.1f} cm, "
              f"failed legs {sum(len(r['fails']) for r in res)}/{n * len(ROUTE)}, worst stop {max(r['worst_stop'] for r in res):.1f} cm")
