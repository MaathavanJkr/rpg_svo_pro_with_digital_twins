#!/usr/bin/env python3
"""Phase 8: C++ distance-field registration (fyp_df) vs the Python prototype (analysis/df_register.py).

Inputs: the logged cloud_filtered of every processed keyframe of the five V1_01 M2 runs (922 KFs,
identical to Phase 7). Both implementations run on the same points with the Phase 8 configuration
(unsigned, untruncated, v = 0.05 m, Huber delta = 0.05, LM, 100-iteration cap); the TSDF path
(mu = 5v) is checked as well. Python grid: the .npz (float32 values); C++ grid: the .bin export
of the same array (loader verifies format, size, FNV-1a checksum and twin md5).
Validity and information post-processing are the Python functions in both cases (df_register.validate,
information_heuristic, information_hessian), so the comparison isolates the solver.

Tolerances (fixed before running): |dt| <= 1e-6 m, rotation difference <= 1e-6 deg, information
(heuristic and Hessian) max |A_cpp - A_py| / max |A_py| <= 1e-6, identical iteration count and verdict.
Also checks the grid query on random points (inside and outside) and that a corrupted payload is rejected.

Usage (venv): python analysis/phase8_equivalence.py   ->  runs/phase8/equivalence.json (+ per-KF csv)
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
import csv
import glob
import json
import shutil
import sys
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "svo_registration/scripts"))
import df_grid
import df_register as dfr
import fyp_df

NPZ = os.path.join(REPO, "dataset/derived/euroc_twin/df/V1_leica_v30mm_udf_v50mm.npz")
BIN = NPZ[:-4] + ".bin"
RUNS = sorted(glob.glob(os.path.join(REPO, "runs/2026-10-23_V1_01_easy_M2_p6_r0*")))
OUT = os.path.join(REPO, "runs/phase8")
TOL = dict(t_m=1e-6, r_deg=1e-6, info_rel=1e-6)
CONFIGS = {"udf_v50_untrunc": None, "tsdf_v50_mu5v": 0.25}


def rot_deg(Ra, Rb):
    # Rotation-vector norm of Ra^T Rb; arccos((tr - 1) / 2) cannot resolve angles below ~1e-6 deg
    # (one ulp of the trace near 1 is ~8.5e-7 deg).
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(Ra.T @ Rb).as_rotvec())))


def rel(a, b):
    if not (np.all(np.isfinite(a)) and np.all(np.isfinite(b))):
        return 0.0 if (np.array_equal(np.isnan(a), np.isnan(b)) and np.allclose(np.nan_to_num(a), np.nan_to_num(b))) else np.inf
    return float(np.max(np.abs(a - b)) / max(np.max(np.abs(b)), 1e-300))


def main():
    os.makedirs(OUT, exist_ok=True)
    gpy = df_grid.DistanceGrid(NPZ)
    gcpp = fyp_df.DistanceGrid(BIN, df_grid.TWIN_MD5)
    out = dict(grid=os.path.relpath(BIN, REPO), runs=[os.path.basename(r) for r in RUNS], tolerances=TOL)

    # Grid query on random points: inside, near the boundary and outside.
    rng = np.random.default_rng(0)
    lo = gpy.origin - 0.2
    hi = gpy.origin + gpy.v * (gpy.shape - 1) + 0.2
    P = rng.uniform(lo, hi, size=(200000, 3))
    for mu in (None, 0.25):
        D1, G1, v1 = gpy.query(P, mu)
        D2, G2, v2 = gcpp.query(P, mu)
        assert np.array_equal(v1, v2)
        out[f"query_mu_{mu}"] = dict(n=len(P), n_valid=int(v1.sum()), max_abs_D=float(np.max(np.abs(D1[v1] - D2[v1]))),
                                     max_abs_G=float(np.max(np.abs(G1[v1] - G2[v1]))))

    # Corrupted payload / header must be rejected.
    tmp = tempfile.mkdtemp()
    try:
        bad = os.path.join(tmp, "bad.bin")
        shutil.copy(BIN, bad); shutil.copy(BIN + ".json", bad + ".json")
        with open(bad, "r+b") as f:
            f.seek(12345); b = f.read(1); f.seek(12345); f.write(bytes([b[0] ^ 1]))
        try:
            fyp_df.DistanceGrid(bad, df_grid.TWIN_MD5); out["corrupt_payload_rejected"] = False
        except RuntimeError as e:
            out["corrupt_payload_rejected"] = str(e)
        with open(bad, "ab") as f:
            f.write(b"\0\0\0\0")
        try:
            fyp_df.DistanceGrid(bad, df_grid.TWIN_MD5); out["wrong_size_rejected"] = False
        except RuntimeError as e:
            out["wrong_size_rejected"] = str(e)
    finally:
        shutil.rmtree(tmp)

    files = [f for r in RUNS for f in sorted(glob.glob(os.path.join(r, "reg_events", "kf_*.npz")))]
    for name, mu in CONFIGS.items():
        rows = []
        for f in files:
            pts = np.load(f)["cloud_filtered"].astype(np.float64)
            a = dfr.register(pts, gpy, mu=mu, delta=0.05, max_iter=100)
            b = fyp_df.register(gcpp, pts, mu=mu, delta=0.05, max_iter=100)
            va, vb = dfr.validate(a), dfr.validate(b)
            ia = dfr.information_heuristic(pts, a, va["inliers"]) if va["valid"] else np.full((6, 6), np.nan)
            ib = dfr.information_heuristic(pts, b, vb["inliers"]) if vb["valid"] else np.full((6, 6), np.nan)
            rows.append(dict(
                kf=os.path.relpath(f, REPO), n_points=len(pts),
                dt_m=float(np.linalg.norm(a["T_c"][:3, 3] - b["T_c"][:3, 3])),
                dr_deg=rot_deg(a["T_c"][:3, :3], b["T_c"][:3, :3]),
                it_py=a["iterations"], it_cpp=b["iterations"], lookups_py=a["lookups"], lookups_cpp=b["lookups"],
                valid_py=va["valid"], valid_cpp=vb["valid"], reason_py=va["reason"], reason_cpp=vb["reason"],
                cost_rel=abs(a["cost"] - b["cost"]) / max(abs(a["cost"]), 1e-300),
                sigma2_rel=abs(a["sigma2"] - b["sigma2"]) / max(abs(a["sigma2"]), 1e-300),
                fitness_diff=abs(va["fitness"] - vb["fitness"]), rmse_diff=abs(va["inlier_rmse"] - vb["inlier_rmse"]),
                n_valid_eq=a["n_valid"] == b["n_valid"], n_outside_eq=a["n_outside"] == b["n_outside"],
                info_h_rel=rel(ib, ia), info_H_rel=rel(dfr.information_hessian(b), dfr.information_hessian(a)),
                info_H_cpp_fn_rel=rel(fyp_df.information_hessian(b["H_wv"], b["sigma2"]), dfr.information_hessian(b)),
                t_py_s=a["runtime_s"], t_cpp_s=b["runtime_s"]))
        with open(os.path.join(OUT, f"equivalence_{name}.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
        col = lambda k: np.array([r[k] for r in rows], dtype=float)
        s = dict(n_kf=len(rows), n_valid_py=int(col("valid_py").sum()), n_valid_cpp=int(col("valid_cpp").sum()))
        for k in ("dt_m", "dr_deg", "cost_rel", "sigma2_rel", "fitness_diff", "rmse_diff", "info_h_rel", "info_H_rel", "info_H_cpp_fn_rel"):
            s[k] = dict(max=float(col(k).max()), median=float(np.median(col(k))))
        s["iterations_equal"] = int(sum(r["it_py"] == r["it_cpp"] for r in rows))
        s["lookups_equal"] = int(sum(r["lookups_py"] == r["lookups_cpp"] for r in rows))
        s["verdict_equal"] = int(sum(r["valid_py"] == r["valid_cpp"] and r["reason_py"] == r["reason_cpp"] for r in rows))
        s["n_valid_equal"] = int(sum(r["n_valid_eq"] and r["n_outside_eq"] for r in rows))
        s["pass"] = bool(s["dt_m"]["max"] <= TOL["t_m"] and s["dr_deg"]["max"] <= TOL["r_deg"]
                         and s["info_h_rel"]["max"] <= TOL["info_rel"] and s["info_H_rel"]["max"] <= TOL["info_rel"]
                         and s["iterations_equal"] == len(rows) and s["verdict_equal"] == len(rows))
        s["runtime_in_loop_median_s"] = dict(py=float(np.median(col("t_py_s"))), cpp=float(np.median(col("t_cpp_s"))))
        out[name] = s
        print(name, json.dumps(s, indent=1), flush=True)
    json.dump(out, open(os.path.join(OUT, "equivalence.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
