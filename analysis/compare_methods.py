#!/usr/bin/env python3
"""Compare two sets of EuRoC runs (e.g. M1 vs M2) with mean ± std and Welch's t-test p-values.

Reads per run: analysis/summary.json (eval_vio.py) and analysis/map_frame_ate.json (initial_alignment.py).
Usage (venv): python analysis/compare_methods.py <out.json> --a M1 runs/<..> runs/<..> --b M2 runs/<..> ...
Welch's t-test (unequal variances, two-sided), scipy.stats.ttest_ind(equal_var=False). With n = 5 per
group the test has low power; p-values are reported, not used as a sole decision rule.
"""
import argparse
import json
import os
import numpy as np
from scipy import stats


def metrics(run):
    s = json.load(open(os.path.join(run, "analysis", "summary.json")))
    m = {"ate_4dof_m": s["ate_posyaw_m"]["rmse"], "ate_se3_m": s["ate_se3_m"]["rmse"],
         "frontend_ms": s["runtime"]["frontend_ms"]["mean"],
         "latency_ms": s["runtime"]["backend_output_latency_ms"]["median"]}
    p = os.path.join(run, "analysis", "map_frame_ate.json")
    if os.path.exists(p):
        m["map_frame_ate_m"] = json.load(open(p))["map_frame_ate_m"]["rmse"]
    for seg, v in s["rpe"].items():
        m[f"rel_disp_{seg}"] = v["rel_disp_m"]["mean"]
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--a", nargs="+", required=True)
    ap.add_argument("--b", nargs="+", required=True)
    args = ap.parse_args()
    la, ra = args.a[0], args.a[1:]
    lb, rb = args.b[0], args.b[1:]
    A = [metrics(r) for r in ra]
    B = [metrics(r) for r in rb]
    res = {}
    print(f"{'metric':22s} {la:>18s} {lb:>18s}   Welch p")
    for k in A[0]:
        if not all(k in x for x in A + B):
            continue
        a = np.array([x[k] for x in A]); b = np.array([x[k] for x in B])
        t, p = stats.ttest_ind(a, b, equal_var=False)
        res[k] = {la: dict(mean=float(a.mean()), std=float(a.std(ddof=1)), n=len(a)),
                  lb: dict(mean=float(b.mean()), std=float(b.std(ddof=1)), n=len(b)), "welch_t": float(t), "welch_p": float(p)}
        print(f"{k:22s} {a.mean():8.3f} ± {a.std(ddof=1):6.3f} {b.mean():8.3f} ± {b.std(ddof=1):6.3f}   {p:.3g}")
    json.dump(dict(a=dict(label=la, runs=ra), b=dict(label=lb, runs=rb), metrics=res), open(args.out, "w"), indent=2)


if __name__ == "__main__":
    main()
