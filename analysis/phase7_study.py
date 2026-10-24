#!/usr/bin/env python3
"""Phase 7 full offline study (OFFLINE, OPEN-LOOP, on inputs logged from M2 online runs).

Inputs: logged cloud_filtered + T_init_world_body of every processed keyframe of the given M2 runs,
GT positions (events_poses.npz, evaluation only; GT orientation is never used), the twin file and the
distance grids (analysis/df_grid.py).

Subcommands
  accuracy  every KF x every configuration (P2P d = 0.5 m; DF full factorial v x {untrunc, 2v, 5v, 10v};
            DF Huber sensitivity) -> runs/phase7/study/accuracy_<cfg>.csv
  basin     subset of KFs x perturbations (translation, rotation, combined, yaw-only) x methods
            -> runs/phase7/study/basin.csv
Both use the same iteration cap (100) and each method's own convergence criteria.

Usage (venv): python analysis/phase7_study.py accuracy|basin runs/<M2 run> ...
"""
import os
# One OpenMP thread per worker, set before Open3D is imported: Open3D's threading in worker processes
# deadlocked the first study run (fork after Open3D initialisation). Workers use the spawn context.
os.environ.setdefault("OMP_NUM_THREADS", "1")
import argparse
import csv
import glob
import json
import os
import re
import sys
import time
import multiprocessing as mp
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import df_grid
import df_register as dfr
import p2p_offline as p2p

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "runs/phase7/study")
GRIDS = {v: os.path.join(REPO, f"dataset/derived/euroc_twin/df/V1_leica_v30mm_udf_v{int(v * 1000)}mm.npz") for v in (0.05, 0.10, 0.20)}
MAX_ITER = 100
# Success thresholds for the convergence basin (fixed before running):
SAME_T_M, SAME_R_DEG = 0.05, 1.0   # final body pose vs the method's own unperturbed solution
GT_POS_M = 0.25                    # final body position vs GT position


def accuracy_configs():
    cfgs = {"p2p_d0.5": dict(method="p2p", d=0.5)}
    for v in (0.05, 0.10, 0.20):
        for name, mu in (("untrunc", None), ("mu2v", 2 * v), ("mu5v", 5 * v), ("mu10v", 10 * v)):
            cfgs[f"df_v{int(v * 1000)}_{name}_h0.05"] = dict(method="df", v=v, mu=mu, delta=0.05)
    for dlt in (0.02, 0.10):
        cfgs[f"df_v50_untrunc_h{dlt:.2f}"] = dict(method="df", v=0.05, mu=None, delta=dlt)
    return cfgs


BASIN_METHODS = {"p2p_d0.5": dict(method="p2p", d=0.5), "p2p_d1.0": dict(method="p2p", d=1.0),
                 "p2p_d2.0": dict(method="p2p", d=2.0),
                 "df_v50_untrunc": dict(method="df", v=0.05, mu=None, delta=0.05),
                 "df_v100_untrunc": dict(method="df", v=0.10, mu=None, delta=0.05),
                 "df_v200_untrunc": dict(method="df", v=0.20, mu=None, delta=0.05),
                 "df_v50_mu5v": dict(method="df", v=0.05, mu=0.25, delta=0.05)}


def basin_levels():
    L = [("translation", t, 0.0) for t in (0.1, 0.2, 0.3, 0.5, 0.75, 1.0)]
    L += [("rotation", 0.0, r) for r in (2, 5, 10, 15, 20)]
    L += [("combined", t, r) for t, r in ((0.1, 2), (0.3, 5), (0.5, 10), (0.75, 15), (1.0, 20))]
    L += [("yaw", 0.0, r) for r in (2, 5, 10, 15, 20)]
    return L


# ------------------------------------------------------------------ worker state
_W = {}


def _init():
    _W["grids"] = {v: df_grid.DistanceGrid(p) for v, p in GRIDS.items()}
    twin = o3d.io.read_point_cloud(df_grid.TWIN)
    _W["twin"] = twin
    _W["twin_pts"] = np.asarray(twin.points)
    _W["twin_nrm"] = np.asarray(twin.normals)


def solve(cfg, cloud):
    """Run one registration; returns dict with T_c, validity, iterations, lookups, runtime, information."""
    if cfg["method"] == "p2p":
        r = p2p.icp(cloud, _W["twin"], max_corr=cfg["d"], max_iter=MAX_ITER)
        val = p2p.validate(r)
        info_h = p2p.information_heuristic(cloud, r, _W["twin_nrm"]) if val["valid"] else np.full((6, 6), np.nan)
        info_H = p2p.information_hessian(cloud, r, _W["twin_pts"], _W["twin_nrm"])
        return dict(T_c=r["T_c"], valid=val["valid"], reason=val["reason"], iterations=r["iterations"],
                    lookups=r["lookups"], runtime_s=r["runtime_s"], info_h=info_h, info_H=info_H)
    g = _W["grids"][cfg["v"]]
    r = dfr.register(cloud, g, mu=cfg["mu"], delta=cfg["delta"], max_iter=MAX_ITER)
    val = dfr.validate(r)
    info_h = dfr.information_heuristic(cloud, r, val["inliers"]) if val["valid"] else np.full((6, 6), np.nan)
    info_H = dfr.information_hessian(r)
    return dict(T_c=r["T_c"], valid=val["valid"], reason=val["reason"], iterations=r["iterations"],
                lookups=r["lookups"], runtime_s=r["runtime_s"], info_h=info_h, info_H=info_H)


def info_summary(prefix, A):
    out = {}
    if np.isnan(A).any():
        return {f"{prefix}_{k}": np.nan for k in ("pos_diag", "rot_diag", "min_eig", "cond")}
    S = 0.5 * (A + A.T)
    ev = np.linalg.eigvalsh(S)
    out[f"{prefix}_pos_diag"] = float(np.mean(np.diag(S)[:3]))
    out[f"{prefix}_rot_diag"] = float(np.mean(np.diag(S)[3:]))
    out[f"{prefix}_min_eig"] = float(ev.min())
    out[f"{prefix}_cond"] = float(ev.max() / max(abs(ev.min()), 1e-300))
    return out


def _acc_task(args):
    cfg, kf = args
    s = solve(cfg, kf["cloud"])
    T, T0 = s["T_c"], kf["T_init"]
    pb = T0[:3, 3]
    pc = T[:3, :3] @ pb + T[:3, 3]
    row = dict(run=kf["run"], stamp_ns=kf["stamp_ns"], valid=s["valid"], reason=s["reason"],
               iterations=s["iterations"], lookups=s["lookups"], runtime_s=s["runtime_s"],
               corr_t_m=float(np.linalg.norm(T[:3, 3])),
               corr_rot_deg=float(np.degrees(Rot.from_matrix(T[:3, :3]).magnitude())),
               corr_yaw_deg=float(np.degrees(Rot.from_matrix(T[:3, :3]).as_rotvec()[2])),
               err_m=float(np.linalg.norm(pc - kf["p_gt"])), vio_err_m=float(np.linalg.norm(pb - kf["p_gt"])))
    for k, a in enumerate("xyz"):
        row[f"dpos_{a}"] = float(pc[k] - pb[k]); row[f"need_{a}"] = float(kf["p_gt"][k] - pb[k])
    row.update(info_summary("infoheur", s["info_h"])); row.update(info_summary("infohess", s["info_H"]))
    return row


def perturb(cloud, T_init, kind, t_m, r_deg, rng):
    """Rigid perturbation of the cloud (and the initial body pose) about the body position."""
    c = T_init[:3, 3]
    if kind == "yaw":
        axis = np.array([0.0, 0.0, rng.choice([-1.0, 1.0])])
    else:
        axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
    R = Rot.from_rotvec(np.radians(r_deg) * axis).as_matrix()
    d = rng.normal(size=3); d /= np.linalg.norm(d)
    t = t_m * d
    P = np.eye(4); P[:3, :3] = R; P[:3, 3] = c - R @ c + t
    return (cloud - c) @ R.T + c + t, P @ T_init


def _basin_task(args):
    mname, cfg, kf, kind, t_m, r_deg, seed, ref = args
    rng = np.random.default_rng(seed)
    cloud_p, T0p = perturb(kf["cloud"], kf["T_init"], kind, t_m, r_deg, rng)
    s = solve(cfg, cloud_p)
    Tfin = s["T_c"] @ T0p
    dt = float(np.linalg.norm(Tfin[:3, 3] - ref[:3, 3]))
    dr = float(np.degrees(Rot.from_matrix(Tfin[:3, :3] @ ref[:3, :3].T).magnitude()))
    gt = float(np.linalg.norm(Tfin[:3, 3] - kf["p_gt"]))
    return dict(method=mname, run=kf["run"], stamp_ns=kf["stamp_ns"], kind=kind, t_m=t_m, r_deg=r_deg,
                d_same_t_m=dt, d_same_r_deg=dr, gt_err_m=gt, accepted=s["valid"], iterations=s["iterations"],
                success=bool(dt <= SAME_T_M and dr <= SAME_R_DEG and gt <= GT_POS_M),
                same_solution=bool(dt <= SAME_T_M and dr <= SAME_R_DEG), gt_ok=bool(gt <= GT_POS_M))


def _ref_task(args):
    mname, cfg, kf = args
    s = solve(cfg, kf["cloud"])
    return mname, kf["stamp_ns"], s["T_c"] @ kf["T_init"]


def load_kfs(runs):
    kfs = []
    for run in runs:
        poses = np.load(os.path.join(run, "analysis", "events_poses.npz"))
        p_gt = dict(zip(poses["stamps_ns"].tolist(), poses["p_gt"]))
        for f in sorted(glob.glob(os.path.join(run, "reg_events", "kf_*.npz"))):
            ns = int(re.search(r"kf_(\d+)", f).group(1))
            if ns not in p_gt:
                continue
            d = np.load(f)
            kfs.append(dict(run=os.path.basename(run.rstrip("/")), stamp_ns=ns, cloud=d["cloud_filtered"].astype(np.float64),
                            T_init=d["T_init_world_body"], p_gt=p_gt[ns]))
    return kfs


def write_csv(path, rows):
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["accuracy", "basin"])
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--procs", type=int, default=10)
    ap.add_argument("--basin-stride", type=int, default=8)
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--max-kfs", type=int, default=None, help="testing: use only the first N keyframes")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    kfs = load_kfs(args.runs)
    if args.max_kfs:
        kfs = kfs[:args.max_kfs]
    print(f"{len(kfs)} keyframes from {len(args.runs)} runs", flush=True)
    with mp.get_context("spawn").Pool(args.procs, initializer=_init) as pool:
        if args.cmd == "accuracy":
            for name, cfg in accuracy_configs().items():
                if args.only and name not in args.only:
                    continue
                t0 = time.time()
                rows = pool.map(_acc_task, [(cfg, kf) for kf in kfs], chunksize=8)
                write_csv(os.path.join(OUT, f"accuracy_{name}.csv"), rows)
                print(f"{name}: {time.time() - t0:.0f} s", flush=True)
        else:
            sub = [kf for i, kf in enumerate(kfs) if i % args.basin_stride == 0]
            refs = {}
            for mname, ns, T in pool.map(_ref_task, [(m, c, kf) for m, c in BASIN_METHODS.items() for kf in sub], chunksize=4):
                refs[(mname, ns)] = T
            tasks = []
            for li, (kind, t_m, r_deg) in enumerate(basin_levels()):
                for ki, kf in enumerate(sub):
                    seed = 1000003 * li + ki  # same perturbation for every method
                    for mname, cfg in BASIN_METHODS.items():
                        tasks.append((mname, cfg, kf, kind, t_m, r_deg, seed, refs[(mname, kf["stamp_ns"])]))
            t0 = time.time()
            rows = pool.map(_basin_task, tasks, chunksize=8)
            write_csv(os.path.join(OUT, "basin.csv"), rows)
            json.dump(dict(n_keyframes=len(sub), stride=args.basin_stride, levels=basin_levels(), methods=BASIN_METHODS,
                           thresholds=dict(same_t_m=SAME_T_M, same_r_deg=SAME_R_DEG, gt_pos_m=GT_POS_M), seconds=time.time() - t0),
                      open(os.path.join(OUT, "basin_meta.json"), "w"), indent=2, default=str)
            print(f"basin: {len(rows)} registrations in {time.time() - t0:.0f} s", flush=True)


if __name__ == "__main__":
    main()
