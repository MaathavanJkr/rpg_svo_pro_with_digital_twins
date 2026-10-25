#!/usr/bin/env python3
"""Phase 10 throughput control: EXPLORATORY analyses (not pre-registered; no claims).

1. Run-to-run variability M2F vs M3 per sequence (valid runs only): sample std, range, and the
   Brown-Forsythe test (Levene with median centring, scipy.stats.levene(center="median")) on map-frame ATE
   and 4-DoF ATE.
2. Descriptive diagnosis of the worst M2F run on V2_02 (map-frame ATE 0.143 m, r05): map-frame position error
   over time (svo_node's map-frame output /svo/w_backend_pose_imu vs GT, evaluation only) against the other
   valid M2F and M3 runs, and per-KF registration quantities (accepted, correction size, fitness, inlier RMSE,
   registered-pose error vs GT) to see whether accepted registrations with large corrections or poor fitness
   coincide with the error growth. Nothing is changed.
Usage (venv + ROS): python analysis/phase10_tc_exploratory.py
Output: runs/phase10_tc/exploratory.json, docs/img/phase10_tc_v2_02_r05.png
"""
import csv
import glob
import json
import os
import numpy as np
import rosbag
from scipy import stats
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORST = "2026-10-25_V2_02_medium_M2F_p10tc_r05"
C_M2F, C_M3, C_BAD, MUTED = "#eb6834", "#1baf7a", "#2a78d6", "#52514e"


def runs(seq, m):
    out = []
    for f in sorted(glob.glob(os.path.join(REPO, f"runs/2026-10-25_{seq}_{m}_p10tc_r[0-9][0-9]/analysis/phase9_run.json"))):
        d = json.load(open(f))
        if not d["failed"]:
            out.append(d)
    return out


def gt(seq):
    g = np.loadtxt(os.path.join(REPO, f"dataset/derived/asl_meta/{seq}/mav0/state_groundtruth_estimate0/data.csv"),
                   delimiter=",", comments="#", usecols=(0, 1, 2, 3))
    g[:, 0] *= 1e-9
    return g


def map_error(run, g, t0_after):
    """Map-frame position error over time after the D1 window (t > t_end), from the recorded map-frame output."""
    tr = []
    with rosbag.Bag(os.path.join(REPO, "runs", run, "run.bag")) as b:
        for _, m, _ in b.read_messages(topics=["/svo/w_backend_pose_imu"]):
            p = m.pose.pose.position if hasattr(m.pose, "pose") else m.pose.position
            tr.append([m.header.stamp.to_sec(), p.x, p.y, p.z])
    tr = np.array(tr)
    tr = tr[(tr[:, 0] > t0_after) & (tr[:, 0] <= g[-1, 0])]
    gi = np.c_[[np.interp(tr[:, 0], g[:, 0], g[:, k]) for k in (1, 2, 3)]].T
    return tr[:, 0], np.linalg.norm(tr[:, 1:] - gi, axis=1)


def d1_end(run):
    rows = list(csv.DictReader(open(os.path.join(REPO, "runs", run, "d1_alignment.csv"))))
    return int(rows[-1]["t_end_ns"]) * 1e-9


def kf_table(run):
    rr = list(csv.DictReader(open(os.path.join(REPO, "runs", run, "registration.csv"))))
    ep = np.load(os.path.join(REPO, "runs", run, "analysis/events_poses.npz"))
    perr = {int(s): float(np.linalg.norm(T[:3, 3] - p)) for s, T, p in zip(ep["stamps_ns"], ep["T_result"], ep["p_gt"])}
    out = []
    for r in rr:
        ns = int(round(float(r["stamp"]) * 1e9 / 1e5) * 1e5)
        k = min(perr, key=lambda s: abs(s - ns)) if perr else None
        out.append(dict(t=float(r["stamp"]), valid=r["valid"] == "1", corr_t=float(r["corr_t_m"]), corr_r=float(r["corr_rot_deg"]),
                        fitness=float(r["fitness"]), rmse=float(r["inlier_rmse"]),
                        reg_err=perr[k] if k is not None and abs(k - ns) < 2e6 else np.nan))
    return out


def main():
    out = {"label": "EXPLORATORY, not pre-registered; no claims"}
    var = {}
    for seq in ("V1_02_medium", "V2_02_medium"):
        a, b = runs(seq, "M2F"), runs(seq, "M3")
        e = {}
        for key in ("map_frame_ate_m", "ate_4dof_m"):
            x, y = np.array([r[key] for r in a]), np.array([r[key] for r in b])
            bf = stats.levene(x, y, center="median")
            e[key] = dict(n=[len(x), len(y)], std_m2f=float(x.std(ddof=1)), std_m3=float(y.std(ddof=1)),
                          range_m2f=[float(x.min()), float(x.max())], range_m3=[float(y.min()), float(y.max())],
                          std_ratio=float(x.std(ddof=1) / y.std(ddof=1)),
                          brown_forsythe=dict(W=float(bf.statistic), p=float(bf.pvalue)))
        var[seq] = e
    out["variability"] = var

    seq = "V2_02_medium"
    g = gt(seq)
    curves = {}
    for m in ("M2F", "M3"):
        for r in runs(seq, m):
            t, err = map_error(r["run"], g, d1_end(r["run"]))
            curves[r["run"]] = (m, t, err)
    t_ref = curves[WORST][1]
    tt0 = t_ref[0]
    grid = np.arange(0, t_ref[-1] - tt0, 0.5)
    def on_grid(t, e):
        return np.interp(grid, t - t[0], e, left=np.nan, right=np.nan)
    others = np.array([on_grid(t, e) for k, (m, t, e) in curves.items() if m == "M2F" and k != WORST])
    m3s = np.array([on_grid(t, e) for k, (m, t, e) in curves.items() if m == "M3"])
    worst = on_grid(curves[WORST][1], curves[WORST][2])
    o_med, o_p90 = np.nanmedian(others, 0), np.nanpercentile(others, 90, axis=0)
    above = grid[worst > o_p90]
    kt = kf_table(WORST)
    t_kf = np.array([k["t"] for k in kt]) - tt0
    # Per-KF registration statistics of the worst run vs the other valid M2F runs (pooled).
    def summ(tab):
        acc = [k for k in tab if k["valid"]]
        return dict(n_kf=len(tab), accepted=len(acc), rejected=len(tab) - len(acc),
                    corr_t_median=float(np.median([k["corr_t"] for k in acc])), corr_t_p90=float(np.percentile([k["corr_t"] for k in acc], 90)),
                    corr_rot_median=float(np.median([k["corr_r"] for k in acc])), corr_rot_p90=float(np.percentile([k["corr_r"] for k in acc], 90)),
                    fitness_median=float(np.median([k["fitness"] for k in acc])), rmse_median=float(np.median([k["rmse"] for k in acc])),
                    reg_err_median=float(np.nanmedian([k["reg_err"] for k in acc])), reg_err_p90=float(np.nanpercentile([k["reg_err"] for k in acc], 90)))
    other_tabs = [kf_table(k) for k, (m, _, _) in curves.items() if m == "M2F" and k != WORST]
    pooled = [k for tab in other_tabs for k in tab]
    # Error growth: first time the worst run exceeds the others' p90 for >= 5 s continuously.
    exceed = worst > o_p90
    start = None
    run_len = 0
    for i, flag in enumerate(exceed):
        run_len = run_len + 1 if flag else 0
        if run_len >= 10 and start is None:
            start = float(grid[i - 9])
    win = [k for k in kt if start is not None and start - 5 <= k["t"] - tt0 <= start + 15]
    out["worst_run"] = dict(run=WORST, map_frame_ate=next(r["map_frame_ate_m"] for r in runs(seq, "M2F") if r["run"] == WORST),
                            error_max_m=float(np.nanmax(worst)), error_max_at_s=float(grid[np.nanargmax(worst)]),
                            first_sustained_exceed_p90_s=start, share_time_above_p90=float(np.mean(exceed[~np.isnan(worst)])),
                            registrations_worst=summ(kt), registrations_other_M2F_pooled=summ(pooled),
                            registrations_window_start_minus5_plus15s=summ(win) if len(win) > 3 else None,
                            time_axis="s after the end of the D1 window")
    json.dump(out, open(os.path.join(REPO, "runs/phase10_tc/exploratory.json"), "w"), indent=2)
    print(json.dumps(out, indent=1))

    fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    for row in others:
        ax[0].plot(grid, row, color=C_M2F, lw=0.6, alpha=0.35)
    for row in m3s:
        ax[0].plot(grid, row, color=C_M3, lw=0.6, alpha=0.35)
    ax[0].plot(grid, worst, color=C_BAD, lw=1.6, label=f"M2F r05 (map-frame ATE {out['worst_run']['map_frame_ate']:.3f} m)")
    ax[0].plot([], [], color=C_M2F, lw=1, label="other valid M2F runs (8)")
    ax[0].plot([], [], color=C_M3, lw=1, label="valid M3 runs (10)")
    ax[0].set_ylabel("map-frame position error [m]")
    ax[0].legend(frameon=False, fontsize=8, loc="upper left")
    if start is not None:
        for a in ax:
            a.axvline(start, color=MUTED, lw=0.8, ls="--")
    acc = np.array([k["valid"] for k in kt])
    for a, key, lab in ((ax[1], "corr_t", "correction |t| [m]"), (ax[2], "fitness", "fitness"), (ax[3], "reg_err", "registered pose error vs GT [m]")):
        v = np.array([k[key] for k in kt])
        a.scatter(t_kf[acc], v[acc], s=10, color=C_BAD, label="accepted")
        a.scatter(t_kf[~acc], v[~acc], s=14, marker="x", color=MUTED, label="rejected")
        a.set_ylabel(lab)
    ax[1].legend(frameon=False, fontsize=8)
    ax[3].set_xlabel("time after the end of the D1 window [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle("EXPLORATORY: V2_02, worst M2F run (r05) vs other runs; dashed: first sustained exceedance of the other M2F runs' p90", fontsize=10)
    fig.savefig(os.path.join(REPO, "docs/img/phase10_tc_v2_02_r05.png"), dpi=110, bbox_inches="tight")


if __name__ == "__main__":
    main()
