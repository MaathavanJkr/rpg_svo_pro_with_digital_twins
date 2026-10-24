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

**Superseded by the full study below** (the checkpoint used keyframe-level p-values; the full study uses run-level statistics and a block bootstrap, per decision 2026-10-24).

---

# FULL STUDY (2026-10-24)

> **All results in this section are OFFLINE and OPEN-LOOP, on inputs logged from M2 online runs (V1_01, 5 runs).** No online claim follows from them; the corrections are not fed back into the VIO here. Online effects are measured in Phase 9.

Code: `analysis/phase7_study.py` (accuracy + basin, parallel `spawn` workers), `analysis/p2p_offline.py` (Open3D point-to-plane with iteration count from Open3D's debug log, both information outputs), `analysis/df_register.py`, `analysis/phase7_report.py` (statistics and plots). Raw results: `runs/phase7/study/accuracy_*.csv`, `basin.csv`, `report.json`. **917 keyframes** (5 of the 922 logged have no ≤ 10 ms GT bracket).

## Settings (fixed before running)
- **Iteration cap 100 for both methods**, each with its own convergence criteria: Open3D ICP defaults (|Δfitness| < 1e-6 and |ΔRMSE| < 1e-6); DF LM: step < 1e-7 or relative cost change < 1e-9.
- **Huber δ = 0.05 m** (pre-set). Sensitivity δ ∈ {0.02, 0.10} reported, **not used to choose**.
- **Point-to-plane** with the EuRoC config (correspondence 0.5 m) for accuracy; 0.5 / 1.0 / 2.0 m in the basin.
- **Statistics:** keyframes are not independent, so everything is computed **per run (n = 5)**. DF vs P2P: **paired t-test at run level** (same inputs per run). For the two headline quantities, a **moving-block bootstrap** (blocks of 10 contiguous KFs, resampled within each run, B = 2000) gives a 95 % CI of DF − P2P. No keyframe-level p-values.

## Grids: memory and build time (truncation is applied at query time, so one grid per v serves all μ)
| v | voxels | memory (float32) | build | per variant μ ∈ {untrunc, 2v, 5v, 10v} |
|---|---|---|---|---|
| 0.05 m | 5 852 085 | 23.4 MB | 24.2 s | same grid (memory 23.4 MB, build 24.2 s) |
| 0.10 m | 744 120 | 3.0 MB | 3.5 s | same grid |
| 0.20 m | 98 010 | 0.39 MB | 0.7 s | same grid |
(A truncated grid could be stored in 8 bits over [0, μ], i.e. 5.9 / 0.74 / 0.10 MB; not implemented.)

## Accuracy, jitter, acceptance (run-level mean ± std, n = 5; p = paired t-test vs P2P)
Figure `docs/img/phase7_variants.png`.

| config | accept | median pos. err. vs GT [m] | p | KF-to-KF yaw jitter [°] | p | KF-to-KF pos. jitter [m] | r(achieved, needed) | iterations (median) | lookups / KF | runtime [ms]* |
|---|---|---|---|---|---|---|---|---|---|---|
| **P2P d = 0.5** | 0.975 ± 0.019 | 0.0796 ± 0.0062 | – | 0.604 ± 0.014 | – | 0.054 ± 0.002 | 0.60 ± 0.03 | 9 | 3 767 NN | 438 |
| DF v=0.05 untrunc | **0.998 ± 0.003** | 0.0748 ± 0.0082 | 0.082 | **0.485 ± 0.029** | **0.0008** | 0.049 ± 0.004 | 0.63 ± 0.02 | 26 | 20 880 | 61 |
| DF v=0.05 μ=2v | 0.986 ± 0.011 | 0.0729 ± 0.0068 | 0.011 | 0.599 ± 0.072 | 0.88 | 0.058 ± 0.009 | 0.63 ± 0.02 | 27 | 21 736 | 62 |
| DF v=0.05 μ=5v | 0.999 ± 0.002 | 0.0740 ± 0.0081 | 0.058 | 0.496 ± 0.048 | 0.006 | 0.048 ± 0.005 | 0.63 ± 0.03 | 26 | 20 660 | 60 |
| DF v=0.05 μ=10v | 0.998 ± 0.003 | 0.0747 ± 0.0080 | 0.068 | 0.485 ± 0.031 | 0.0009 | 0.049 ± 0.003 | 0.63 ± 0.02 | 26 | 20 689 | 60 |
| DF v=0.10 untrunc | 0.986 ± 0.019 | 0.0754 ± 0.0060 | 0.12 | 0.478 ± 0.024 | 0.0005 | 0.066 ± 0.015 | 0.56 ± 0.02 | 29 | 23 370 | 64 |
| DF v=0.10 μ=2v | 0.989 ± 0.019 | 0.0744 ± 0.0080 | 0.10 | 0.504 ± 0.046 | 0.011 | 0.069 ± 0.019 | 0.55 ± 0.03 | 30 | 23 744 | 67 |
| DF v=0.10 μ=5v | 0.986 ± 0.019 | 0.0752 ± 0.0065 | 0.12 | 0.475 ± 0.024 | 0.0005 | 0.066 ± 0.015 | 0.56 ± 0.02 | 29 | 23 362 | 66 |
| DF v=0.10 μ=10v | 0.986 ± 0.019 | 0.0754 ± 0.0060 | 0.12 | 0.478 ± 0.024 | 0.0005 | 0.066 ± 0.015 | 0.56 ± 0.02 | 29 | 23 373 | 65 |
| DF v=0.20 untrunc | 0.980 ± 0.014 | **0.0979 ± 0.0051** | 0.002 (worse) | 0.506 ± 0.028 | 0.0004 | 0.075 ± 0.017 | 0.59 ± 0.06 | 36 | 28 307 | 78 |
| DF v=0.20 μ=2v | 0.981 ± 0.013 | 0.0969 ± 0.0052 | 0.002 (worse) | 0.502 ± 0.037 | 0.001 | 0.075 ± 0.017 | 0.59 ± 0.06 | 37 | 29 142 | 83 |
| DF v=0.20 μ=5v / 10v | 0.980 ± 0.014 | 0.0979 ± 0.0051 | 0.002 (worse) | 0.506 ± 0.028 | 0.0004 | 0.075 ± 0.017 | 0.59 ± 0.06 | 36 | 28 307 | 79–80 |
\* Python DF and Open3D ICP times are **indicative only** (10 parallel workers; the Open3D time includes debug-log capture; online the logged ICP took 176 ms). P2P "lookups" = nearest-neighbour queries ((iterations + 1) × points); DF lookups = trilinear grid evaluations including LM trial steps. **H2 (speed) is decided in C++ in Phase 8.**

**Headline comparisons on KFs accepted by both (run-level paired test + block bootstrap 95 % CI of DF − P2P):**
| config | median pos. err. DF / P2P [m] | paired p | bootstrap CI Δ median pos. err. [m] | bootstrap CI Δ yaw jitter [°] |
|---|---|---|---|---|
| DF v=0.05 untrunc | 0.0735 / 0.0796 | 0.019 | [−0.0087, −0.0030] | [−0.171, −0.090] |
| DF v=0.05 μ=5v | 0.0731 / 0.0796 | 0.020 | [−0.0103, −0.0036] | [−0.164, −0.064] |
| DF v=0.05 μ=10v | 0.0734 / 0.0796 | 0.018 | [−0.0092, −0.0030] | [−0.168, −0.091] |
| DF v=0.10 untrunc | 0.0746 / 0.0796 | 0.047 | [−0.0084, −0.0007] | [−0.174, −0.096] |
| DF v=0.20 untrunc | 0.0973 / 0.0792 | 0.002 | [+0.014, +0.024] (worse) | [−0.148, −0.054] |

Reject reasons (all KFs): P2P 23 × rotation; DF v=0.05 untrunc 2 × rotation; DF v=0.05 μ=2v 11 × rmse + 2 × rotation; DF v=0.20 untrunc 16 × rotation + 2 × translation.

**Reading (offline, open-loop):**
- v = 0.05 and 0.10 m give ~5–6 mm lower median position error than point-to-plane on the same KFs (block-bootstrap CIs exclude 0) and **~0.12° (≈ 20 %) less KF-to-KF yaw jitter** (all CIs exclude 0). v = 0.20 m loses ~18 mm of position accuracy (grid discretisation) but keeps the lower jitter.
- **Truncation:** μ ≥ 5v is indistinguishable from untruncated, because few points sit farther than 5v from the surface once converged. μ = 2v (0.10 m at v = 0.05) rejects 11 KFs on inlier RMSE and loses the jitter advantage (0.599°, p = 0.88): too tight.
- v = 0.10 m has more position jitter than 0.05 m (0.066 vs 0.049 m) and a lower correlation with the needed correction (0.56 vs 0.63).
- DF needs ~3× more iterations than P2P (26–37 vs 9) but each iteration is a constant-time grid lookup instead of a nearest-neighbour search.

## Huber δ sensitivity (v = 0.05 m, untruncated; δ = 0.05 m stays the pre-set value)
Figure `docs/img/phase7_huber.png`.
| δ | accept | median pos. err. [m] | yaw jitter [°] | iterations |
|---|---|---|---|---|
| 0.02 | 0.998 ± 0.003 | 0.0739 ± 0.0072 | 0.472 ± 0.034 | 27 |
| **0.05 (pre-set)** | 0.998 ± 0.003 | 0.0748 ± 0.0082 | 0.485 ± 0.029 | 26 |
| 0.10 | 0.992 ± 0.012 | 0.0765 ± 0.0080 | 0.523 ± 0.015 | 25 |
Small, monotonic effect (smaller δ slightly better); the conclusion vs P2P is the same for all three. Not used to pick δ.

## Information outputs (both, per decision D4)
Medians over all KFs. Heuristic = M2's formula (for DF with n = ∇D/‖∇D‖); Hessian = JᵀWJ/σ², [pos, rot] order. Figure `docs/img/phase7_info_iterations.png`.
| config | heuristic: mean pos diag / rot diag / cond | Hessian: mean pos diag / rot diag / min eig / cond |
|---|---|---|
| P2P d = 0.5 | 47.3 / 2.34 / 336 | 3.47e4 / 1.89e5 / 3.63e3 / 111 |
| DF v=0.05 untrunc | 51.3 / 2.53 / 241 | 2.46e4 / 1.33e5 / 2.56e3 / 94 |
| DF v=0.10 untrunc | 47.8 / 2.36 / 286 | 1.46e4 / 9.03e4 / 1.33e3 / 129 |
| DF v=0.20 untrunc | 47.9 / 2.37 / 262 | 6.59e3 / 3.52e4 / 650 / 108 |
| DF v=0.05 μ=2v | 51.1 / 2.52 / 242 | 2.78e4 / 1.52e5 / 2.82e3 / 97 |
- The **heuristic** information has almost the same magnitude for both methods, because it is normalised by its largest entry and then scaled by the fixed factors (9², 2²). So M3 in the M2 convention would be weighted like M2 in the backend (as intended by D4), with a slightly better-conditioned matrix (cond 241 vs 336).
- The **Hessian-based** information is ~30 % lower for DF v = 0.05 than for P2P (Huber down-weighting, interpolation smoothing) and drops further with coarser grids (v = 0.20: ~5× lower min eigenvalue). In the D4 ablation, DF priors would therefore be weighted less than P2P priors, unlike in the heuristic convention.

## Convergence basin (H1)
115 keyframes (every 8th), the **same random perturbation per keyframe and level for every method**, applied rigidly about the body position to cloud and initial pose. **Success = final body pose within 0.05 m / 1.0° of the method's own unperturbed solution AND final body position within 0.25 m of GT** (thresholds fixed before running; GT orientation not used). The validity gates (e.g. the 5° rotation limit) are not part of success. Figure `docs/img/phase7_basin.png`.

Success rate (pooled):
| perturbation | P2P 0.5 | P2P 1.0 | P2P 2.0 | DF v=0.05 | DF v=0.10 | DF v=0.20 | DF v=0.05 μ=5v |
|---|---|---|---|---|---|---|---|
| translation 0.1 m | 0.84 | 0.84 | 0.84 | 0.90 | 0.95 | 0.93 | 0.93 |
| translation 0.5 m | 0.83 | 0.83 | 0.83 | 0.90 | 0.94 | 0.91 | 0.89 |
| translation 1.0 m | 0.68 | 0.80 | 0.81 | 0.85 | 0.88 | 0.87 | **0.48** |
| rotation 2° | 0.85 | 0.85 | 0.85 | 0.90 | 0.94 | 0.90 | 0.93 |
| rotation 20° | 0.81 | 0.83 | 0.82 | 0.90 | 0.94 | 0.91 | 0.88 |
| combined 0.5 m / 10° | 0.77 | 0.80 | 0.80 | 0.89 | 0.92 | 0.90 | 0.83 |
| combined 1.0 m / 20° | 0.62 | 0.69 | 0.69 | 0.73 | 0.75 | 0.75 | 0.49 |
| yaw-only 10° | 0.81 | 0.81 | 0.81 | 0.90 | 0.93 | 0.90 | 0.91 |
| yaw-only 20° | 0.63 | 0.71 | 0.71 | 0.86 | 0.90 | 0.88 | 0.63 |
(All levels in `runs/phase7/study/report.json`.)

Run-level (n = 5, paired t-test vs P2P d = 0.5):
| level | P2P 0.5 | P2P 2.0 | DF v=0.05 | DF v=0.10 |
|---|---|---|---|---|
| translation 1.0 m | 0.68 ± 0.09 | 0.81 ± 0.10 (p = 0.025) | 0.85 ± 0.04 (p = 0.006) | 0.88 ± 0.07 (p = 0.004) |
| combined 1.0 m / 20° | 0.62 ± 0.04 | 0.69 ± 0.11 (p = 0.14) | 0.73 ± 0.07 (p = 0.034) | 0.75 ± 0.06 (p = 0.014) |
| yaw-only 20° | 0.63 ± 0.10 | 0.71 ± 0.09 (p = 0.075) | 0.86 ± 0.06 (p = 0.001) | 0.90 ± 0.05 (p = 0.001) |
| translation 0.1 m (floor) | 0.84 ± 0.10 | 0.84 ± 0.10 | 0.90 ± 0.06 (p = 0.026) | 0.95 ± 0.04 (p = 0.041) |

**Floor-corrected view (added 2026-10-24).** Part of DF's unconditional advantage is a **higher floor** (success at the smallest perturbation: P2P 0.84–0.87, DF 0.90–0.95), i.e. more KFs whose solution is stable at all, not a larger basin. Therefore: **conditional success** = success at a level among the KFs that succeed at the smallest perturbation of that axis (0.1 m; 2°; 0.1 m/2°; yaw 2°), and **drop from floor** = floor − unconditional success at that level.

Conditional success (pooled; floor set per method) | drop from floor:
| level | P2P 0.5 | P2P 1.0 | P2P 2.0 | DF v=0.05 | DF v=0.10 | DF v=0.20 | DF μ=5v |
|---|---|---|---|---|---|---|---|
| translation 0.5 m | 0.98 \| +0.01 | 0.99 \| +0.01 | 0.99 \| +0.01 | 0.99 \| +0.01 | 0.99 \| +0.01 | 1.00 \| +0.00 | 0.95 \| +0.05 |
| translation 1.0 m | 0.79 \| +0.19 | 0.94 \| +0.05 | 0.95 \| +0.04 | 0.94 \| +0.06 | 0.94 \| +0.06 | 0.96 \| +0.04 | 0.52 \| +0.47 |
| rotation 20° | 0.94 \| +0.05 | 0.96 \| +0.03 | 0.95 \| +0.04 | 0.99 \| +0.01 | 0.99 \| +0.00 | 0.99 \| +0.00 | 0.94 \| +0.06 |
| combined 0.5 m / 10° | 0.91 \| +0.08 | 0.94 \| +0.05 | 0.94 \| +0.05 | 0.98 \| +0.02 | 0.98 \| +0.01 | 0.98 \| +0.02 | 0.90 \| +0.10 |
| combined 1.0 m / 20° | 0.72 \| +0.25 | 0.78 \| +0.18 | 0.79 \| +0.18 | 0.80 \| +0.19 | 0.80 \| +0.19 | 0.83 \| +0.16 | 0.53 \| +0.46 |
| yaw-only 10° | 0.93 \| +0.07 | 0.93 \| +0.07 | 0.93 \| +0.07 | 0.99 \| +0.01 | 1.00 \| −0.01 | 0.99 \| +0.01 | 0.98 \| +0.02 |
| yaw-only 20° | 0.73 \| +0.26 | 0.83 \| +0.17 | 0.83 \| +0.17 | 0.96 \| +0.04 | 0.97 \| +0.01 | 0.97 \| +0.03 | 0.67 \| +0.32 |
(All levels in `runs/phase7/study/report.json`, key `basin_conditional`.)

Run-level conditional success at the largest levels (n = 5, paired t-test):
| level | DF v=0.05 | P2P 0.5 | P2P 2.0 | p (DF vs P2P 0.5) | p (DF vs P2P 2.0) |
|---|---|---|---|---|---|
| translation 1.0 m | 0.94 ± 0.04 | 0.79 ± 0.09 | 0.95 ± 0.01 | 0.008 | 0.78 |
| combined 1.0 m / 20° | 0.80 ± 0.03 | 0.72 ± 0.08 | 0.79 ± 0.07 | 0.11 | 0.76 |
| rotation 20° | 0.99 ± 0.02 | 0.94 ± 0.05 | 0.95 ± 0.03 | 0.13 | 0.083 |
| yaw-only 20° | **0.96 ± 0.02** | 0.73 ± 0.04 | 0.83 ± 0.04 | **0.0003** | **0.006** |
(DF v = 0.10 vs P2P 2.0: translation 1.0 m p = 0.76, combined p = 0.76, rotation p = 0.16, yaw 20° p = 0.007.)

**Reading (offline, open-loop):**
- **There is a success floor even at the smallest perturbation** (P2P 0.84–0.87, DF 0.90–0.95): in 5–16 % of KFs the solution moves by > 5 cm or > 1° for a 0.1 m / 2° change of the start. This measures how well-determined each KF's registration is (weak geometry / few landmarks), not the basin. **DF has the higher floor, i.e. is more stable overall.**
- **Relative to the floor, translation and combined basins are comparable** once P2P uses a correspondence distance ≥ 1.0 m (translation 1.0 m: 0.94 vs 0.95, p = 0.78; combined 1.0 m / 20°: 0.80 vs 0.79, p = 0.76). DF beats P2P only against the EuRoC 0.5 m setting (translation 1.0 m: p = 0.008). So the unconditional translation/combined advantage was mostly the floor plus P2P's configuration.
- **Random-axis rotation:** DF slightly more stable (0.99 vs 0.94–0.95 at 20°), not significant at n = 5 (p = 0.08–0.16).
- **Yaw:** DF has a **clearly larger basin**: conditional success at 20° yaw 0.96–0.97 vs 0.73 (P2P 0.5) and 0.83 (P2P 1.0/2.0), p ≤ 0.007 against every P2P setting.
- **Truncation limits the basin:** μ = 5v (0.25 m) collapses at 1.0 m (conditional 0.52) and 20° yaw (0.67), as expected (no gradient beyond μ).

## Overall Phase 7 reading (offline, open-loop, V1_01 inputs from M2 runs)
| Hypothesis | Offline evidence | Status |
|---|---|---|
| H1 convergence basin | Floor-corrected: **larger basin for yaw** (20°: 0.96–0.97 vs 0.73–0.83, p ≤ 0.007 vs every P2P setting); translation and combined **comparable** relative to the floor when P2P uses d ≥ 1.0 m; random-axis rotation slightly better, not significant. DF is **more stable overall** (floor 0.90–0.95 vs 0.84–0.87). Truncation shrinks the basin. | **supported for yaw**; comparable for translation/combined; DF more stable overall (offline) |
| H2 speed | iterations 26–37 (DF) vs 9 (P2P); per iteration O(1) lookups vs NN search; Python vs C++ times not comparable | open (Phase 8, C++) |
| H3 accuracy | −5 to −6 mm median per-KF position error (v ≤ 0.10, CIs exclude 0), ~20 % less KF-to-KF yaw jitter; p90 tail unchanged | supported offline per KF; online effect open (Phase 9) |

**Note on the jitter result:** the per-KF yaw-jitter reduction (0.604 → 0.485°, ≈ 20 %) is **modest relative to the ~0.9° per-KF ICP noise** that cost M2 its local consistency online (docs/phase6_logging.md). Whether it is enough to change the online picture is open. **The D4 weighting ablation may matter as much as the solver**: with M2's heuristic weighting, both methods enter the backend with almost identical information (see Information outputs), so the remaining noise is weighted the same way.
| H4 memory | 0.39 / 3.0 / 23.4 MB for v = 0.20 / 0.10 / 0.05 m (room scale), scaling ≈ (1/v)³ (×7.6–7.9 per halving) | consistent (room scale is small) |

**Candidate for Phase 8 (for decision):** DF **v = 0.05 m, untruncated** (or μ = 10v, identical here), Huber 0.05 m, information in the M2 heuristic convention (D4 main) plus the Hessian variant for the ablation. v = 0.10 m is a close alternative (larger basin, 8× less memory, more position jitter).
