#!/usr/bin/env python3
"""Phase 7: offline comparison of point-to-plane ICP (M2, logged) and distance-field registration
(M3 prototype) on IDENTICAL inputs: the logged cloud_filtered and T_init_world_body per keyframe of
M2 runs, and the same twin file.

Steps per keyframe
  1. Sanity: re-run Open3D point-to-plane ICP exactly as the node does and compare with the logged T_icp.
  2. Distance-field registration (analysis/df_register.py) with the chosen grid / mu / Huber delta.
  3. Metrics: position error of the result vs GT position (interpolated; GT orientation NOT used),
     correction size, validity, iterations, lookups, runtime (Python: indicative only).
Per run (accepted KFs, in time order): KF-to-KF correction variability = std of corrections and std
of consecutive differences (yaw, translation), and correlation of the achieved position change
(R_c p + t_c - p at the body) with the needed one (p_gt - p_init), pooled over x/y/z.

Usage (venv): python analysis/phase7_offline.py --grid <udf npz> [--mu M] [--delta D] --tag NAME runs/<M2 run> ...
Output: runs/phase7/<tag>/{per_kf.csv, summary.json}
"""
import argparse
import csv
import glob
import json
import os
import re
import time
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as Rot

import df_grid
import df_register as dfr

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def p2p_replay(cloud, twin_pcd, max_corr=0.5):
    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cloud.astype(np.float64)))
    r = o3d.pipelines.registration.registration_icp(
        src, twin_pcd, max_corr, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25))
    return np.asarray(r.transformation)


def yaw_of(T):
    return float(np.degrees(Rot.from_matrix(T[:3, :3]).as_rotvec()[2]))


def run_variability(rows, key_prefix):
    acc = [r for r in rows if r[f"{key_prefix}_valid"]]
    if len(acc) < 3:
        return {}
    yaw = np.array([r[f"{key_prefix}_corr_yaw_deg"] for r in acc])
    tr = np.array([[r[f"{key_prefix}_dpos_{a}"] for a in "xyz"] for r in acc])
    need = np.array([[r[f"need_dpos_{a}"] for a in "xyz"] for r in acc])
    ok = ~np.isnan(need).any(1)
    return dict(n=len(acc), yaw_std_deg=float(yaw.std()), yaw_diff_std_deg=float(np.diff(yaw).std()),
                dpos_std_m=float(np.linalg.norm(tr.std(0))), dpos_diff_std_m=float(np.linalg.norm(np.diff(tr, axis=0).std(0))),
                r_dpos_vs_needed=float(np.corrcoef(tr[ok].ravel(), need[ok].ravel())[0, 1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--grid", required=True)
    ap.add_argument("--mu", type=float, default=None)
    ap.add_argument("--delta", type=float, default=0.05)
    ap.add_argument("--max-iter", type=int, default=25)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--no-replay", action="store_true")
    args = ap.parse_args()
    out = os.path.join(REPO, "runs/phase7", args.tag)
    os.makedirs(out, exist_ok=True)
    grid = df_grid.DistanceGrid(args.grid)
    twin = o3d.io.read_point_cloud(df_grid.TWIN)
    rows, per_run = [], {}
    for run in args.runs:
        ev = {int(r["stamp_ns"]): r for r in csv.DictReader(open(os.path.join(run, "analysis", "events.csv")))}
        poses = np.load(os.path.join(run, "analysis", "events_poses.npz"))
        p_gt = dict(zip(poses["stamps_ns"].tolist(), poses["p_gt"]))
        run_rows = []
        for f in sorted(glob.glob(os.path.join(run, "reg_events", "kf_*.npz"))):
            ns = int(re.search(r"kf_(\d+)", f).group(1))
            d = np.load(f)
            cloud = d["cloud_filtered"].astype(np.float64)
            T0 = d["T_init_world_body"]
            row = dict(run=os.path.basename(run.rstrip("/")), stamp_ns=ns, n_points=len(cloud))
            # 1. sanity replay
            if not args.no_replay:
                t_s = time.perf_counter()
                Tr = p2p_replay(cloud, twin)
                row["p2p_replay_s"] = time.perf_counter() - t_s
                row["p2p_replay_maxdiff"] = float(np.abs(Tr - d["T_icp"]).max())
            # 2. distance field
            res = dfr.register(cloud, grid, mu=args.mu, delta=args.delta, max_iter=args.max_iter)
            val = dfr.validate(res)
            Tdf = res["T_c"]
            row.update(df_valid=val["valid"], df_reason=val["reason"], df_fitness=val["fitness"],
                       df_rmse=val["inlier_rmse"], df_iterations=res["iterations"], df_lookups=res["lookups"],
                       df_runtime_s=res["runtime_s"], df_outside=res["n_outside"],
                       df_corr_t_m=val["corr_t_m"], df_corr_rot_deg=val["corr_rot_deg"], df_corr_yaw_deg=yaw_of(Tdf))
            Tp = d["T_icp"]
            ang = float(np.degrees(np.arccos(np.clip((np.trace(Tp[:3, :3]) - 1) / 2, -1, 1))))
            row.update(p2p_valid=bool(d["valid"]), p2p_reason=str(d["reject_reason"]), p2p_fitness=float(d["fitness"]),
                       p2p_rmse=float(d["inlier_rmse"]), p2p_corr_t_m=float(np.linalg.norm(Tp[:3, 3])),
                       p2p_corr_rot_deg=ang, p2p_corr_yaw_deg=yaw_of(Tp), p2p_runtime_s=float(d["t_icp_s"]))
            # 3. errors vs GT position (GT orientation not used)
            pb = T0[:3, 3]
            g = p_gt.get(ns)
            for name, Tc in (("df", Tdf), ("p2p", Tp)):
                pc = Tc[:3, :3] @ pb + Tc[:3, 3]
                for k, a in enumerate("xyz"):
                    row[f"{name}_dpos_{a}"] = float(pc[k] - pb[k])
                row[f"{name}_err_m"] = float(np.linalg.norm(pc - g)) if g is not None else np.nan
            row["vio_err_m"] = float(np.linalg.norm(pb - g)) if g is not None else np.nan
            for k, a in enumerate("xyz"):
                row[f"need_dpos_{a}"] = float(g[k] - pb[k]) if g is not None else np.nan
            run_rows.append(row)
        per_run[os.path.basename(run.rstrip("/"))] = dict(df=run_variability(run_rows, "df"), p2p=run_variability(run_rows, "p2p"))
        rows.extend(run_rows)

    keys = list(rows[0].keys())
    with open(os.path.join(out, "per_kf.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys); w.writeheader(); w.writerows(rows)
    A = {k: np.array([r.get(k, np.nan) for r in rows], dtype=float) for k in keys if k not in ("run", "df_reason", "p2p_reason")}
    dfv, ppv = A["df_valid"] > 0, A["p2p_valid"] > 0
    both = dfv & ppv
    med = lambda x: float(np.nanmedian(x))
    def agg(name):
        vals = [v[name] for v in per_run.values() if v[name]]
        return {k: [float(np.mean([v[k] for v in vals])), float(np.std([v[k] for v in vals], ddof=1)) if len(vals) > 1 else 0.0]
                for k in vals[0]} if vals else {}
    summary = dict(grid=args.grid, mu=args.mu, huber_delta=args.delta, max_iter=args.max_iter, n_keyframes=len(rows), runs=list(per_run),
                   sanity_p2p_replay_maxdiff=dict(max=float(np.nanmax(A.get("p2p_replay_maxdiff", [np.nan]))),
                                                  median=med(A.get("p2p_replay_maxdiff", [np.nan]))),
                   accept_rate=dict(df=float(dfv.mean()), p2p=float(ppv.mean())),
                   pos_err_median_m=dict(vio=med(A["vio_err_m"]), df_accepted=med(A["df_err_m"][dfv]), p2p_accepted=med(A["p2p_err_m"][ppv]),
                                         df_on_both=med(A["df_err_m"][both]), p2p_on_both=med(A["p2p_err_m"][both]), vio_on_both=med(A["vio_err_m"][both])),
                   pos_err_p90_m=dict(vio=float(np.nanpercentile(A["vio_err_m"], 90)), df_on_both=float(np.nanpercentile(A["df_err_m"][both], 90)),
                                      p2p_on_both=float(np.nanpercentile(A["p2p_err_m"][both], 90))),
                   frac_better_than_vio_on_both=dict(df=float(np.mean(A["df_err_m"][both] < A["vio_err_m"][both])),
                                                     p2p=float(np.mean(A["p2p_err_m"][both] < A["vio_err_m"][both]))),
                   correction_median=dict(df_yaw_deg=med(A["df_corr_yaw_deg"][dfv]), p2p_yaw_deg=med(A["p2p_corr_yaw_deg"][ppv]),
                                          df_t_m=med(A["df_corr_t_m"][dfv]), p2p_t_m=med(A["p2p_corr_t_m"][ppv])),
                   kf_to_kf_variability_mean_std_over_runs=dict(df=agg("df"), p2p=agg("p2p")),
                   df_iterations=dict(median=med(A["df_iterations"]), p90=float(np.nanpercentile(A["df_iterations"], 90)), max=float(np.nanmax(A["df_iterations"]))),
                   df_lookups_median=med(A["df_lookups"]), df_outside_median=med(A["df_outside"]),
                   runtime_ms_median_indicative=dict(df_python=1e3 * med(A["df_runtime_s"]), p2p_open3d_logged=1e3 * med(A["p2p_runtime_s"]),
                                                     p2p_open3d_replay=1e3 * med(A.get("p2p_replay_s", [np.nan]))),
                   df_reject_reasons={k: int(v) for k, v in zip(*np.unique([r["df_reason"] for r in rows if not r["df_valid"]], return_counts=True))} if (~dfv).any() else {},
                   per_run=per_run)
    json.dump(summary, open(os.path.join(out, "summary.json"), "w"), indent=2)
    print(json.dumps({k: v for k, v in summary.items() if k != "per_run"}, indent=2))


if __name__ == "__main__":
    main()
