#!/usr/bin/env python3
"""Phase 7 checkpoint: point-to-plane (logged, M2) vs distance field (offline) on identical inputs.
Per-run metrics with Welch's t-test (n = runs) and a figure.

Usage (venv): python analysis/phase7_checkpoint.py runs/phase7/<tag>
Writes runs/phase7/<tag>/checkpoint.json and docs/img/phase7_checkpoint_<tag>.png
"""
import csv
import json
import os
import sys
from collections import defaultdict
import numpy as np
from scipy import stats
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
tagdir = sys.argv[1]
tag = os.path.basename(tagdir.rstrip("/"))
rows = list(csv.DictReader(open(os.path.join(tagdir, "per_kf.csv"))))
f = lambda r, k: float(r[k]) if r.get(k) not in (None, "", "nan") else np.nan
by_run = defaultdict(list)
for r in rows:
    by_run[r["run"]].append(r)

per_run = {}
for run, rs in by_run.items():
    rs = sorted(rs, key=lambda r: int(r["stamp_ns"]))
    m = {}
    for meth in ("p2p", "df"):
        acc = [r for r in rs if r[f"{meth}_valid"] == "True"]
        yaw = np.array([f(r, f"{meth}_corr_yaw_deg") for r in acc])
        dp = np.array([[f(r, f"{meth}_dpos_{a}") for a in "xyz"] for r in acc])
        need = np.array([[f(r, f"need_dpos_{a}") for a in "xyz"] for r in acc])
        ok = ~np.isnan(need).any(1)
        m[meth] = dict(accept_rate=len(acc) / len(rs),
                       pos_err_median=float(np.nanmedian([f(r, f"{meth}_err_m") for r in acc])),
                       yaw_diff_std=float(np.diff(yaw).std()), dpos_diff_std=float(np.linalg.norm(np.diff(dp, axis=0).std(0))),
                       r_dpos_needed=float(np.corrcoef(dp[ok].ravel(), need[ok].ravel())[0, 1]))
    m["vio_pos_err_median"] = float(np.nanmedian([f(r, "vio_err_m") for r in rs]))
    per_run[run] = m

res = dict(tag=tag, n_runs=len(per_run), metrics={})
for k in ("accept_rate", "pos_err_median", "yaw_diff_std", "dpos_diff_std", "r_dpos_needed"):
    a = np.array([v["p2p"][k] for v in per_run.values()]); b = np.array([v["df"][k] for v in per_run.values()])
    res["metrics"][k] = dict(p2p=[float(a.mean()), float(a.std(ddof=1))], df=[float(b.mean()), float(b.std(ddof=1))],
                             welch_p=float(stats.ttest_ind(a, b, equal_var=False)[1]))
# KF-level paired comparison of position error on KFs accepted by both
both = [r for r in rows if r["p2p_valid"] == "True" and r["df_valid"] == "True" and r.get("df_err_m") not in ("", "nan")]
ep = np.array([f(r, "p2p_err_m") for r in both]); ed = np.array([f(r, "df_err_m") for r in both])
res["kf_paired_pos_err"] = dict(n=len(both), median_diff_df_minus_p2p=float(np.median(ed - ep)),
                                wilcoxon_p=float(stats.wilcoxon(ed, ep)[1]), frac_df_better=float(np.mean(ed < ep)))
res["per_run"] = per_run
json.dump(res, open(os.path.join(tagdir, "checkpoint.json"), "w"), indent=2)
print(json.dumps({k: v for k, v in res.items() if k != "per_run"}, indent=2))

fig, axs = plt.subplots(1, 3, figsize=(19, 5.5))
ev = np.array([f(r, "vio_err_m") for r in both])
bins = np.linspace(0, 0.45, 46)
for x, lab in ((ev, "VIO before correction"), (ep, "point-to-plane (M2, logged)"), (ed, "distance field (offline)")):
    axs[0].hist(x, bins, histtype="step", lw=1.6, label=f"{lab}: median {np.median(x):.3f} m")
axs[0].set_xlabel("position error vs GT at keyframe [m]"); axs[0].set_ylabel("keyframes"); axs[0].legend(fontsize=8)
axs[0].set_title(f"{len(both)} KFs accepted by both, {len(per_run)} runs")
run0 = sorted(by_run)[0]
rs = sorted(by_run[run0], key=lambda r: int(r["stamp_ns"]))
t0 = int(rs[0]["stamp_ns"])
for meth, lab in (("p2p", "point-to-plane"), ("df", "distance field")):
    acc = [r for r in rs if r[f"{meth}_valid"] == "True"]
    axs[1].plot([(int(r["stamp_ns"]) - t0) * 1e-9 for r in acc], [f(r, f"{meth}_corr_yaw_deg") for r in acc], ".-", ms=3, lw=0.6, label=lab)
axs[1].set_xlabel("time [s]"); axs[1].set_ylabel("correction yaw [deg]"); axs[1].set_title(f"KF-to-KF correction yaw, {run0[-6:]}"); axs[1].legend()
need = np.array([[f(r, f"need_dpos_{a}") for a in "xyz"] for r in both]).ravel()
for meth, lab in (("p2p", "point-to-plane"), ("df", "distance field")):
    got = np.array([[f(r, f"{meth}_dpos_{a}") for a in "xyz"] for r in both]).ravel()
    axs[2].scatter(need, got, s=2, alpha=0.3, label=f"{lab} (r = {np.corrcoef(need, got)[0, 1]:.2f})")
lim = [-0.4, 0.4]
axs[2].plot(lim, lim, "k--", lw=0.8); axs[2].set_xlim(lim); axs[2].set_ylim(lim)
axs[2].set_xlabel("needed position change (GT - VIO) per axis [m]"); axs[2].set_ylabel("achieved position change [m]"); axs[2].legend(fontsize=8, markerscale=4)
fig.suptitle(f"Phase 7 checkpoint ({tag}): identical inputs, positions vs GT (GT orientation not used)")
fig.savefig(os.path.join(REPO, f"docs/img/phase7_checkpoint_{tag}.png"), dpi=115, bbox_inches="tight")
