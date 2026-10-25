#!/usr/bin/env python3
"""Phase 10 / D4 step 1 (OFFLINE): weighting realism check of the information matrices.

For every logged keyframe of the Phase 9 M2 runs on V1_01, V1_02, V2_02 (reg_events/kf_*.npz, cloud_filtered
float64 + T_init_world_body; identical inputs for both methods), run
  P2P: point-to-plane ICP as M2 (p2p_icp_persistent.py; verdict checked against the logged M2 verdict)
  DF : the C++ distance-field registration (fyp_df, v = 0.05 m untruncated, Huber 0.05, 100 iterations)
with each method's validity checks, and form both information conventions on the accepted KFs:
  heuristic: M2's formula (P2P: twin normals; DF: normalised grad D)  -> what M2 / M3 publish
  Hessian  : J^T W J / sigma^2 at the solution, reordered to [pos, rot] (D4 ablation option, as published)
The backend reads the published 6x6 matrix as INFORMATION of [body position, body rotation] (map frame),
so the implied covariance is inv(A): position block -> sigma_pos, rotation z -> sigma_yaw.
Empirical errors (GT for evaluation only):
  position: registered body position (T_c T_init) - GT position at the KF stamp (events_poses.npz), map frame;
            consistency = NEES e^T S_pos^-1 e (expected 3 for a consistent estimator); ratio = implied RMS std /
            empirical RMS error.
  yaw     : no reliable GT orientation (docs/phase4_m1.md); empirical per-KF yaw noise = KF-to-KF yaw jitter / sqrt 2
            (jitter = std of consecutive differences of the correction yaw, per run, Phase 7 definition; assumes
            independent per-KF errors and negligible VIO yaw change between consecutive KFs).
Diagnostic (not an arm): the Hessian re-expressed at the body (left perturbation of the correction about the map
origin -> body: dp = v - [p_body]x w, dtheta = w); the published convention keeps the map-origin parametrisation.
Sensitivity: the empirical errors contain any systematic offset (twin vs Vicon frame, GT reference point); the
mean error vector per sequence/method is reported, and the Hessian NEES is recomputed with it removed.
Calibration (proposal only): a single factor k per method fitted on V1_01 ONLY so that mean position NEES = 3
(A_cal = A_hessian / k^2), evaluated on V1_02 and V2_02.
Usage (venv): python analysis/phase10_d4_realism.py  ->  runs/phase10_d4/realism.json, realism_kf.csv, docs/img/phase10_d4_realism.png
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
import csv
import glob
import json
import re
import sys
import numpy as np
import open3d as o3d

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "analysis"))
sys.path.insert(0, os.path.join(REPO, "svo_registration/scripts"))
import df_register as dfr  # noqa: E402
import p2p_offline as p2p  # noqa: E402
import fyp_df  # noqa: E402
from p2p_icp_persistent import PersistentPointToPlaneICP  # noqa: E402

OUT = os.path.join(REPO, "runs/phase10_d4")
SEQS = ["V1_01_easy", "V1_02_medium", "V2_02_medium"]
CHI2_3_95 = 7.814727903251178


def skew(p):
    return np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])


def yaw_deg(R):
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def p2p_validate(r, cfg=dfr.CFG):
    T = r.transformation
    reasons = []
    if r.fitness < cfg["fitness_threshold"]: reasons.append("fitness")
    if r.inlier_rmse > cfg["inliers_rmse_thr"]: reasons.append("rmse")
    if not reasons:
        if np.linalg.norm(T[:3, 3]) > cfg["max_translation_distance"]: reasons.append("translation")
        ang = np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1)))
        if ang > cfg["max_rotation_distance"]: reasons.append("rotation")
    return not reasons


def implied(A):
    """sigma_pos (3,), sigma_yaw [deg], S_pos from an information matrix in [pos, rot]."""
    try:
        S = np.linalg.inv(A)
    except np.linalg.LinAlgError:
        return None
    if not np.all(np.isfinite(S)) or np.any(np.diag(S) <= 0):
        return None
    return np.sqrt(np.diag(S)[:3]), float(np.degrees(np.sqrt(S[5, 5]))), S[:3, :3]


def body_centred(A_pr, p_body):
    """Hessian information as published ([pos=v, rot=w] about the map origin) -> covariance at the body."""
    P = np.zeros((6, 6)); P[:3, 3:] = np.eye(3); P[3:, :3] = np.eye(3)
    S_wv = np.linalg.inv(P.T @ A_pr @ P)          # back to (w, v) order
    J = np.zeros((6, 6)); J[:3, :3] = -skew(p_body); J[:3, 3:] = np.eye(3); J[3:, :3] = np.eye(3)
    S = J @ S_wv @ J.T
    return np.sqrt(np.diag(S)[:3]), float(np.degrees(np.sqrt(S[5, 5]))), S[:3, :3]


def main():
    os.makedirs(OUT, exist_ok=True)
    twins = {}
    for room in ("V1", "V2"):
        pc = o3d.io.read_point_cloud(os.path.join(REPO, f"dataset/derived/euroc_twin/{room}_leica_v30mm.ply"))
        P, N = np.asarray(pc.points), np.asarray(pc.normals)
        md5 = {"V1": "8fc20009837a56090259b9c5da31724c", "V2": "d450781f0bf7894d71d3721a8002b121"}[room]
        twins[room] = dict(P=P, N=N, icp=PersistentPointToPlaneICP(P, N),
                           grid=fyp_df.DistanceGrid(os.path.join(REPO, f"dataset/derived/euroc_twin/df/{room}_leica_v30mm_udf_v50mm.bin"), md5))
    rows = []
    for seq in SEQS:
        tw = twins[seq[:2]]
        for run in sorted(glob.glob(os.path.join(REPO, f"runs/2026-10-24_{seq}_M2_p9_r[0-9][0-9]"))):
            ep = np.load(os.path.join(run, "analysis/events_poses.npz"))
            gt = {int(s): p for s, p in zip(ep["stamps_ns"], ep["p_gt"])}
            for f in sorted(glob.glob(os.path.join(run, "reg_events", "kf_*.npz"))):
                ns = int(re.search(r"kf_(\d+)", f).group(1))
                d = np.load(f)
                k = min(gt, key=lambda s: abs(s - ns)) if gt else None
                if k is None or abs(k - ns) > 2e6 or not np.all(np.isfinite(gt[k])):
                    continue
                pts, Ti, pg = d["cloud_filtered"].astype(np.float64), d["T_init_world_body"], gt[k]
                base = dict(seq=seq, run=os.path.basename(run), stamp_ns=ns)
                # P2P (= M2)
                r, _, _, _ = tw["icp"].register(pts, 0.5, 25)
                ok = p2p_validate(r)
                res = dict(correspondences=r.correspondence_set, inlier_rmse=r.inlier_rmse, T_c=r.transformation)
                rows.append(dict(base, method="P2P", valid=ok, valid_logged=bool(d["valid"]), T_c=r.transformation, Ti=Ti, pg=pg,
                                 A_h=p2p.information_heuristic(pts, res, tw["N"]) if ok else None,
                                 A_H=p2p.information_hessian(pts, res, tw["P"], tw["N"]) if ok else None))
                # DF (= M3)
                g = fyp_df.register(tw["grid"], pts, mu=None, delta=0.05, max_iter=100)
                v = dfr.validate(g)
                rows.append(dict(base, method="DF", valid=v["valid"], valid_logged=None, T_c=g["T_c"], Ti=Ti, pg=pg,
                                 A_h=dfr.information_heuristic(pts, g, v["inliers"]) if v["valid"] else None,
                                 A_H=dfr.information_hessian(g) if v["valid"] else None))
            print(seq, os.path.basename(run), len(rows), flush=True)

    # Per-KF quantities on accepted KFs.
    kf = []
    for r in rows:
        if not r["valid"]:
            continue
        T = r["T_c"] @ r["Ti"]
        e = T[:3, 3] - r["pg"]
        out = dict(seq=r["seq"], run=r["run"], method=r["method"], stamp_ns=r["stamp_ns"], err_m=float(np.linalg.norm(e)),
                   yaw_corr_deg=yaw_deg(r["T_c"][:3, :3]))
        for conv, A in (("heur", r["A_h"]), ("hess", r["A_H"])):
            im = implied(A) if A is not None and np.all(np.isfinite(A)) else None
            if im is None:
                out[f"{conv}_ok"] = False
                continue
            sp, sy, Sp = im
            out.update({f"{conv}_ok": True, f"{conv}_sig_pos_rms": float(np.sqrt(np.mean(sp ** 2))), f"{conv}_sig_yaw_deg": sy,
                        f"{conv}_nees": float(e @ np.linalg.solve(Sp, e))})
        if out.get("hess_ok"):
            sp, sy, Sp = body_centred(r["A_H"], T[:3, 3])
            out.update(hessbody_sig_pos_rms=float(np.sqrt(np.mean(sp ** 2))), hessbody_sig_yaw_deg=sy,
                       hessbody_nees=float(e @ np.linalg.solve(Sp, e)))
        out["_e"] = e
        out.update(ex=float(e[0]), ey=float(e[1]), ez=float(e[2]))
        if out.get("hess_ok"):
            out["_Sp_hess"] = np.linalg.inv(r["A_H"])[:3, :3]
        kf.append(out)
    with open(os.path.join(OUT, "realism_kf.csv"), "w", newline="") as fh:
        keys = [k for k in dict.fromkeys(k for r in kf for k in r) if not k.startswith("_")]
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore"); w.writeheader(); w.writerows(kf)

    p2p_agree = [r["valid"] == r["valid_logged"] for r in rows if r["method"] == "P2P"]
    summary = dict(inputs="Phase 9 M2 runs p9 r01-r10 (logged cloud_filtered), accepted KFs per method",
                   p2p_verdict_equals_logged_M2=f"{sum(p2p_agree)}/{len(p2p_agree)}")

    def yaw_noise(sel):
        """Empirical per-KF yaw noise [deg]: median over runs of std(diff(yaw_corr))/sqrt(2), consecutive accepted KFs."""
        vals = []
        for run in sorted({k["run"] for k in sel}):
            y = np.array([k["yaw_corr_deg"] for k in sorted(sel, key=lambda k: k["stamp_ns"]) if k["run"] == run])
            if len(y) > 10:
                vals.append(np.diff(y).std() / np.sqrt(2))
        return float(np.median(vals))

    res = {}
    calib = {}
    for m in ("P2P", "DF"):
        for seq in SEQS:
            sel = [k for k in kf if k["method"] == m and k["seq"] == seq]
            E = np.array([k["_e"] for k in sel])
            emp_pos = float(np.sqrt(np.mean(np.sum(E ** 2, axis=1)) / 3))  # empirical per-axis RMS error
            emp_yaw = yaw_noise(sel)
            bias = E.mean(axis=0)
            Ec = E - bias
            emp_pos_c = float(np.sqrt(np.mean(np.sum(Ec ** 2, axis=1)) / 3))
            okH = [(k, ec) for k, ec in zip(sel, Ec) if "_Sp_hess" in k]
            nees_c = np.array([ec @ np.linalg.solve(k["_Sp_hess"], ec) for k, ec in okH])
            e = dict(n_accepted=len(sel), empirical_pos_rms_per_axis_m=emp_pos,
                     mean_error_vector_m=bias.tolist(), mean_error_norm_m=float(np.linalg.norm(bias)),
                     sensitivity_bias_removed=dict(empirical_pos_rms_per_axis_m=emp_pos_c,
                                                   hess_nees_mean_over_3=float(nees_c.mean() / 3), hess_nees_median=float(np.median(nees_c))), empirical_err_median_m=float(np.median([k["err_m"] for k in sel])),
                     empirical_yaw_noise_deg=emp_yaw)
            for conv in ("heur", "hess", "hessbody"):
                ok = [k for k in sel if f"{conv}_nees" in k]
                if not ok:
                    continue
                nees = np.array([k[f"{conv}_nees"] for k in ok])
                sig = float(np.sqrt(np.mean(np.array([k[f"{conv}_sig_pos_rms"] for k in ok]) ** 2)))
                sy = float(np.median([k[f"{conv}_sig_yaw_deg"] for k in ok]))
                e[conv] = dict(n=len(ok), implied_pos_rms_per_axis_m=sig, ratio_pos_implied_over_empirical=sig / emp_pos,
                               nees_mean=float(nees.mean()), nees_mean_over_3=float(nees.mean() / 3), nees_median=float(np.median(nees)),
                               frac_nees_below_chi2_95=float(np.mean(nees < CHI2_3_95)),
                               implied_yaw_median_deg=sy, ratio_yaw_implied_over_empirical=sy / emp_yaw)
            res[f"{m}/{seq}"] = e
        # Calibrated Hessian (as published): k^2 = mean NEES / 3 on V1_01 only.
        k2 = res[f"{m}/V1_01_easy"]["hess"]["nees_mean_over_3"]
        cal = dict(k_squared_fitted_on_V1_01=k2, k=float(np.sqrt(k2)))
        for seq in SEQS:
            h = res[f"{m}/{seq}"]["hess"]
            cal[seq] = dict(nees_mean_over_3_after=h["nees_mean_over_3"] / k2,
                            ratio_pos_after=h["ratio_pos_implied_over_empirical"] * np.sqrt(k2),
                            ratio_yaw_after=h["ratio_yaw_implied_over_empirical"] * np.sqrt(k2),
                            frac_nees_below_chi2_95_after=float(np.mean(np.array([k["hess_nees"] for k in kf if k["method"] == m and k["seq"] == seq and "hess_nees" in k]) / k2 < CHI2_3_95)))
        calib[m] = cal
    summary["results"] = res
    summary["calibrated_hessian_proposal"] = calib
    json.dump(summary, open(os.path.join(OUT, "realism.json"), "w"), indent=2)
    print(json.dumps(summary, indent=1))

    plot()


def plot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res = json.load(open(os.path.join(OUT, "realism.json")))["results"]
    fig, ax = plt.subplots(1, 2, figsize=(14, 5.2))
    cols = {"heur": "#2a78d6", "hess": "#eb6834"}
    labs = {"heur": "heuristic (published by M2 / M3)", "hess": "Hessian (D4 option, as published)"}
    x = 0
    ticks = []
    for m in ("P2P", "DF"):
        for seq in SEQS:
            e = res[f"{m}/{seq}"]
            for j, conv in enumerate(("heur", "hess")):
                ax[0].bar(x + 0.38 * j, e[conv]["ratio_pos_implied_over_empirical"], 0.36, color=cols[conv], label=labs[conv] if x == 0 else None)
                ax[1].bar(x + 0.38 * j, e[conv]["ratio_yaw_implied_over_empirical"], 0.36, color=cols[conv])
            ticks.append((x + 0.19, f"{m}\n{seq[:5]}"))
            x += 1
    for a, t, yt in zip(ax, ("position: implied std / empirical RMS error", "yaw: implied std / empirical per-KF yaw noise"),
                        ([0.1, 0.3, 1, 3, 10], [0.3, 1, 3, 10, 30, 100, 300])):
        a.axhline(1, color="#0b0b0b", lw=1)
        a.set_yscale("log"); a.set_yticks(yt); a.set_yticklabels([f"{v:g}" for v in yt]); a.minorticks_off()
        a.set_title(t, loc="left"); a.grid(axis="y", alpha=0.3)
        a.set_xticks([p for p, _ in ticks]); a.set_xticklabels([l for _, l in ticks], fontsize=8)
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)
    fig.legend(frameon=False, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.04))
    fig.text(0.01, -0.04, "Ratio < 1: overconfident (implied uncertainty smaller than the observed error); 1 = consistent; > 1: underconfident. "
             "Offline, open-loop, inputs = Phase 9 M2 keyframes.", fontsize=8, color="#52514e")
    fig.savefig(os.path.join(REPO, "docs/img/phase10_d4_realism.png"), dpi=120, bbox_inches="tight")


if __name__ == "__main__":
    plot() if "--plot-only" in sys.argv else main()
