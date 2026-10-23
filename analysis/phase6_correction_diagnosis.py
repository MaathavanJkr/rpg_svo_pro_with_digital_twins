#!/usr/bin/env python3
"""Phase 6 step 4: are the M2 ICP corrections mainly ICP noise or VIO drift? (offline, logs only)

For every ACCEPTED keyframe of the given runs (analysis/events.csv from phase6_events.py), compare
the error vs ground truth of
  (a) the VIO pose before correction (initial guess, map frame), and
  (b) the ICP output pose (T_icp * T_init),
for position (3-D and horizontal), yaw and full rotation. Rotation errors are given both vs the
GT estimate and vs raw Vicon orientation, because the GT orientation is unreliable by a few degrees
on V1 (docs/phase4_m1.md). Also tests whether the correction tracks the VIO error:
  yaw:      correction yaw  vs  -(VIO yaw error)      (ICP rotates about the map z-axis)
  position: (ICP pos - VIO pos)  vs  (GT pos - VIO pos)
If the corrections mainly remove VIO drift, (b) < (a) and the correction follows the VIO error.
If they are mainly ICP noise, (b) >= (a) and the correlation is weak.

Usage (venv): python analysis/phase6_correction_diagnosis.py runs/<id1> runs/<id2> ...
Writes docs/img/phase6_correction_diagnosis.png and runs/phase6_correction_diagnosis.json.
"""
import csv
import json
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(run):
    rows = list(csv.DictReader(open(os.path.join(run, "analysis", "events.csv"))))
    keep = [r for r in rows if r.get("accepted") == "True" and r.get("icp_err_t_m")]
    f = lambda k: np.array([float(r[k]) if r.get(k) not in (None, "") else np.nan for r in keep])
    return {k: f(k) for k in ["vio_err_t_m", "icp_err_t_m", "vio_err_xy_m", "icp_err_xy_m",
                              "vio_err_yaw_deg", "icp_err_yaw_deg", "vio_err_rot_deg", "icp_err_rot_deg",
                              "vio_err_yaw_vicon_deg", "icp_err_yaw_vicon_deg",
                              "vio_err_rot_vicon_deg", "icp_err_rot_vicon_deg", "corr_yaw_deg", "corr_rot_deg", "corr_t_m"]}


def med(x):
    return float(np.nanmedian(x))


def main(runs):
    data = [load(r) for r in runs]
    P = {k: np.concatenate([d[k] for d in data]) for k in data[0]}
    n = len(P["vio_err_t_m"])
    # Position: implied correction vs needed correction. Displacements are not stored per KF in the
    # csv, so use the error magnitudes plus per-run fraction-improved.
    res = dict(n_keyframes=n, runs=[os.path.basename(r.rstrip("/")) for r in runs])
    for name, a, b in [("pos_3d_m", "vio_err_t_m", "icp_err_t_m"), ("pos_xy_m", "vio_err_xy_m", "icp_err_xy_m"),
                       ("abs_yaw_gt_deg", "vio_err_yaw_deg", "icp_err_yaw_deg"),
                       ("abs_yaw_vicon_deg", "vio_err_yaw_vicon_deg", "icp_err_yaw_vicon_deg"),
                       ("rot_gt_deg", "vio_err_rot_deg", "icp_err_rot_deg"),
                       ("rot_vicon_deg", "vio_err_rot_vicon_deg", "icp_err_rot_vicon_deg")]:
        va, vb = np.abs(P[a]), np.abs(P[b])
        res[name] = dict(vio_median=med(va), icp_median=med(vb), vio_p90=float(np.nanpercentile(va, 90)),
                         icp_p90=float(np.nanpercentile(vb, 90)), frac_icp_better=float(np.nanmean(vb < va)))
    for src in ("gt", "vicon"):
        k = "vio_err_yaw_deg" if src == "gt" else "vio_err_yaw_vicon_deg"
        ok = ~np.isnan(P[k]) & ~np.isnan(P["corr_yaw_deg"])
        x, y = -P[k][ok], P["corr_yaw_deg"][ok]
        slope, icpt = np.polyfit(x, y, 1)
        res[f"corr_yaw_vs_needed_{src}"] = dict(pearson_r=float(np.corrcoef(x, y)[0, 1]), slope=float(slope),
                                               intercept_deg=float(icpt),
                                               residual_std_deg=float(np.std(y - (slope * x + icpt))),
                                               needed_yaw_std_deg=float(np.std(x)), corr_yaw_std_deg=float(np.std(y)))
    res["correction_median"] = dict(rot_deg=med(P["corr_rot_deg"]), yaw_deg=med(P["corr_yaw_deg"]), t_m=med(P["corr_t_m"]))
    json.dump(res, open(os.path.join(REPO, "runs/phase6_correction_diagnosis.json"), "w"), indent=2)
    print(json.dumps(res, indent=2))

    fig, axs = plt.subplots(2, 3, figsize=(17, 10))
    pairs = [("vio_err_xy_m", "icp_err_xy_m", "horizontal position error [m]", False),
             ("vio_err_t_m", "icp_err_t_m", "3-D position error [m]", False),
             ("vio_err_yaw_deg", "icp_err_yaw_deg", "yaw error vs GT estimate [deg]", True),
             ("vio_err_yaw_vicon_deg", "icp_err_yaw_vicon_deg", "yaw error vs raw Vicon [deg]", True),
             ("vio_err_rot_vicon_deg", "icp_err_rot_vicon_deg", "rotation error vs raw Vicon [deg]", False)]
    for ax, (a, b, lab, signed) in zip(axs.flat, pairs):
        va, vb = P[a][~np.isnan(P[a])], P[b][~np.isnan(P[b])]
        lo, hi = (min(va.min(), vb.min()), max(va.max(), vb.max()))
        bins = np.linspace(lo, hi, 40)
        ax.hist(va, bins, alpha=0.55, label=f"VIO before correction (median {np.median(va):.3f})")
        ax.hist(vb, bins, alpha=0.55, label=f"ICP output (median {np.median(vb):.3f})")
        if signed:
            ax.axvline(0, c="k", lw=0.8)
        ax.set_xlabel(lab); ax.set_ylabel("keyframes"); ax.legend(fontsize=8)
    ax = axs.flat[5]
    k = "vio_err_yaw_vicon_deg"
    ok = ~np.isnan(P[k])
    r = res["corr_yaw_vs_needed_vicon"]
    ax.scatter(-P[k][ok], P["corr_yaw_deg"][ok], s=6, alpha=0.5)
    xs = np.linspace(np.nanmin(-P[k]), np.nanmax(-P[k]), 2)
    ax.plot(xs, xs, "k--", lw=0.8, label="correction = needed")
    ax.plot(xs, r["slope"] * xs + r["intercept_deg"], "C3", lw=1, label=f"fit: r = {r['pearson_r']:.2f}, slope {r['slope']:.2f}")
    ax.set_xlabel("needed yaw correction = -(VIO yaw error vs raw Vicon) [deg]")
    ax.set_ylabel("ICP correction yaw [deg]"); ax.legend(fontsize=8)
    fig.suptitle(f"M2 V1_01 correction diagnosis: {len(runs)} runs, {n} accepted keyframes (errors vs ground truth, evaluation only)")
    fig.savefig(os.path.join(REPO, "docs/img/phase6_correction_diagnosis.png"), dpi=120, bbox_inches="tight")


if __name__ == "__main__":
    main(sys.argv[1:])
