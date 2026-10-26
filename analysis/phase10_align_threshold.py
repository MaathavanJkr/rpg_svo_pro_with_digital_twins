#!/usr/bin/env python3
"""Phase 10 alignment robustness redesign: room-scale refine_T_WL threshold (rule in docs/PLAN.md, recorded before running).

Replays the original refinement criterion (svo/src/global_pose_handler.cpp, refineAlignment) on the delta = 0 Phase 9 logs of
V1_02 and V2_02 (M2 and M3, valid runs): pairs = registered poses received by svo_node after the D1 alignment
(registration_events.csv, branch to_handler; with refine_T_WL on these are the alignment pairs), VIO keyframe position at each
pair's stamp (analysis/est.tum = /svo/vio_backend_pose_imu at the stamp), C = rotation of T_w_vio (D1, d1_alignment.csv).
H = sum E^T E, E = [-I, [C p]x]; variance = |(H^-1)[5,5]|. For a threshold tau the criterion fires at the first pair whose
variance is below tau (time after the end of the D1 window; never = +inf). tau is chosen so that the median firing time over all
runs (pooled) is 20 s (bisection in log tau), then rounded to 2 significant digits; the firing-time distribution is reported per
sequence at the rounded value.
Usage (venv): python analysis/phase10_align_threshold.py  ->  runs/phase10_align/threshold.json
"""
import csv
import glob
import json
import os
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET_S = 20.0


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def variance_series(run):
    est = np.loadtxt(os.path.join(run, "analysis/est.tum"))
    d1 = list(csv.DictReader(open(os.path.join(run, "d1_alignment.csv"))))[-1]
    C = np.array([[float(d1[f"T_M_W_{i}{j}"]) for j in range(3)] for i in range(3)])
    t_end = int(d1["t_end_ns"]) * 1e-9
    st = np.array([int(r["stamp_ns"]) * 1e-9 for r in csv.DictReader(open(os.path.join(run, "registration_events.csv")))
                   if r["branch"] == "to_handler"])
    H = np.zeros((6, 6))
    t, var = [], []
    for s in st:
        k = np.argmin(np.abs(est[:, 0] - s))
        E = np.zeros((3, 6)); E[:, :3] = -np.eye(3); E[:, 3:] = skew(C @ est[k, 1:4])
        H += E.T @ E
        if len(t) >= 2:  # the code needs more than 2 pairs
            var.append(abs(np.linalg.inv(H)[5, 5]))
            t.append(s - t_end)
        else:
            t.append(s - t_end); var.append(np.inf)
    return np.array(t), np.array(var)


def fire_time(t, var, tau):
    k = np.flatnonzero(var < tau)
    return float(t[k[0]]) if len(k) else np.inf


def main():
    runs = []
    for seq in ("V1_02_medium", "V2_02_medium"):
        for m in ("M2", "M3"):
            for r in sorted(glob.glob(os.path.join(REPO, f"runs/2026-10-24_{seq}_{m}_p9_r[0-9][0-9]"))):
                if not json.load(open(os.path.join(r, "analysis/phase9_run.json")))["failed"]:
                    t, v = variance_series(r)
                    runs.append(dict(run=os.path.basename(r), seq=seq, method=m, t=t, var=v))
    med = lambda tau: float(np.median([fire_time(r["t"], r["var"], tau) for r in runs]))
    lo, hi = 1e-5, 1.0  # median fire time decreases with tau
    for _ in range(80):
        mid = np.sqrt(lo * hi)
        if med(mid) > TARGET_S:
            lo = mid
        else:
            hi = mid
    tau_exact = hi
    tau = float(f"{tau_exact:.2g}")
    out = dict(rule="median firing time 20 s after the D1 window, V1_02 + V2_02, M2 + M3 pooled, delta = 0 Phase 9 logs",
               original_threshold=3e-4, tau_exact=tau_exact, tau=tau, yaw_std_at_tau_deg=float(np.degrees(np.sqrt(tau))),
               n_runs=len(runs), median_fire_s_at_tau=med(tau))
    for seq in ("V1_02_medium", "V2_02_medium", "all"):
        ft = np.array([fire_time(r["t"], r["var"], tau) for r in runs if seq == "all" or r["seq"] == seq])
        fin = ft[np.isfinite(ft)]
        out[seq] = dict(n=len(ft), never=int(np.sum(~np.isfinite(ft))), median_s=float(np.median(ft)),
                        p10_s=float(np.percentile(fin, 10)) if len(fin) else None, p90_s=float(np.percentile(fin, 90)) if len(fin) else None,
                        min_s=float(fin.min()) if len(fin) else None, max_s=float(fin.max()) if len(fin) else None,
                        final_variance_median=float(np.median([r["var"][-1] for r in runs if seq == "all" or r["seq"] == seq])))
    # Also V1_01 (not part of the rule) for context
    os.makedirs(os.path.join(REPO, "runs/phase10_align"), exist_ok=True)
    json.dump(out, open(os.path.join(REPO, "runs/phase10_align/threshold.json"), "w"), indent=2)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
