# Phase 7: offline distance-field prototype (Python, ~/venvs/fyp)

## Setup (identical inputs to M2)
- **Inputs:** the logged `cloud_filtered` and `T_init_world_body` of every processed keyframe of the five V1_01 M2 runs (`runs/2026-10-23_V1_01_easy_M2_p6_r01..r05/reg_events/`, 922 KFs), and the same twin file `dataset/derived/euroc_twin/V1_leica_v30mm.ply` (md5 8fc20009…, verified by `analysis/df_grid.py`).
- **Caveat on inputs:** these clouds/poses come from M2 *online* runs, so the VIO had already been corrected by M2's own priors. Both methods see identical inputs, which is what this offline comparison requires.
- **Validity:** the M2 EuRoC checks (correspondence 0.5 m, inlier RMSE ≤ 0.15 m, fitness ≥ 0.75, correction ≤ 0.75 m / 5°). For the distance field, fitness and inlier RMSE use the untruncated grid distance (≈ nearest-twin-point distance, the analogue of Open3D's correspondence distance).
- **Code:** `analysis/df_grid.py` (grid), `analysis/df_register.py` (registration), `analysis/phase7_offline.py` (driver), `analysis/phase7_checkpoint.py` (per-run Welch tests + figure). Unit checks: analytic trilinear gradient = finite differences to 3e-11; a known 3.2° / 0.19 m correction on a synthetic scene is recovered to 0.05° / 1.2 mm.

## Distance grids (unsigned, exact; built once)
Exact Euclidean distance from each voxel centre to the nearest twin point (cKDTree), bounding box of the twin + 1 m margin. Truncation μ is applied at query time (one grid serves all μ). Figure: `docs/img/phase7_df_slices.png`.

| v | shape | voxels | memory (float32) | file | build |
|---|---|---|---|---|---|
| 0.05 m | 211 × 215 × 129 | 5 852 085 | 23.4 MB | 20.6 MB | 24.2 s |
| 0.10 m | 106 × 108 × 65 | 744 120 | 3.0 MB | 2.6 MB | 3.5 s |
| 0.20 m | 54 × 55 × 33 | 98 010 | 0.39 MB | 0.35 MB | 0.7 s |

## Registration (M3 prototype)
Correction T_c applied on the left in the map frame (same as M2), init = I. Objective Σ Huber_δ(D(R_c pᵢ + t_c)), D = trilinear interpolation of the grid; Levenberg–Marquardt with Jacobian ∇Dᵀ[−[p']ₓ | I]; points outside the grid discarded and counted. Information: M2's heuristic formula with n = ∇D/‖∇D‖ (D4 main), and JᵀWJ/σ² (D4 ablation).

## CHECKPOINT: first unsigned-distance results on V1_01 (v = 0.05 m, untruncated, Huber δ = 0.05 m)
Results `runs/phase7/udf_v50_untrunc/` (25-iteration budget, as M2's ICP) and `runs/phase7/udf_v50_untrunc_it100/` (100-iteration budget); figure `docs/img/phase7_checkpoint_udf_v50_untrunc.png`.

**1. Sanity: Open3D point-to-plane re-run offline on the logged inputs.** 920 of 922 KFs reproduce the logged T_icp to < 1e-6 (median |ΔT| 3.6e-9). 2 KFs differ (1e-4 and 0.046). Most likely cause: the event dump stored the clouds as **float32** while the node uses float64, so rounding can flip a borderline correspondence near the 0.5 m cut-off. Logging is now float64 (`online_registration_fyp.py`), effective for future runs.

**2. Solver convergence.** With a 25-iteration budget every KF hits the cap. With 100 iterations: median 26, p90 40, max 100 iterations, and all results identical to 4 decimals. The cap is not changing any conclusion. The slow tail (≈1e-3 in ξ between 25 and 50 iterations) comes from LM near the non-smooth minimum of the interpolated unsigned distance; noted for Phase 8. Point lookups per KF: median 18 801 (≈ 350 points × ~54 field evaluations including LM trial steps).

**3. Results (identical inputs; GT positions only, GT orientation not used).**

| Metric | point-to-plane (M2, logged) | **distance field** | test |
|---|---|---|---|
| accept rate (per run, mean ± std) | 0.975 ± 0.019 | **0.998 ± 0.003** | Welch p = 0.052 |
| median position error vs GT, per run | 0.080 ± 0.006 m | **0.075 ± 0.008 m** | Welch p = 0.33 |
| median position error, KFs accepted by both (n = 894) | 0.0788 m | **0.0726 m** (VIO 0.1204 m) | Wilcoxon paired p = 3e-25; DF better in 62.9 % of KFs; median paired difference −3.1 mm |
| p90 position error, both accepted | 0.196 m | 0.195 m (VIO 0.179 m) | |
| **KF-to-KF yaw correction variability** (std of consecutive differences) | 0.604 ± 0.014° | **0.484 ± 0.028°** | **Welch p = 0.0001** |
| KF-to-KF position-change variability (std of consecutive differences) | 0.054 ± 0.002 m | **0.049 ± 0.003 m** | Welch p = 0.021 |
| correlation achieved vs needed position change | 0.599 ± 0.026 | 0.627 ± 0.017 | Welch p = 0.081 |
| median correction yaw / translation | −2.51° / 0.103 m | −2.65° / 0.100 m | |
| DF rejects (all runs) | — | 2 (rotation) | |
| runtime per KF (indicative) | Open3D C++: 176 ms (logged), 243 ms (replay) | Python: 40 ms | not comparable; H2 decided in C++ (Phase 8) |

**Reading (offline, one sequence, inputs from M2 runs; no online claim):** on identical inputs, the distance-field registration (v = 0.05 m, untruncated) is accepted more often, gives slightly lower per-KF position error (paired: −3 mm median, better in 63 % of KFs), and has **~20 % less KF-to-KF yaw jitter** (p = 0.0001) than point-to-plane ICP. The p90 tail error is unchanged, and both methods remove the systematic ~−2.5° heading drift. Whether the lower jitter reduces the online loss of local consistency is for Phase 9.

**Not yet run (after approval):** variants (μ ∈ {2v, 5v, 10v}, v ∈ {0.10, 0.20}), information outputs (heuristic and Hessian) evaluation, convergence basin, memory/runtime table per variant.
