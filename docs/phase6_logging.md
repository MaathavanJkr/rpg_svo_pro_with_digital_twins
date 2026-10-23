# Phase 6: logging, M2 repeat runs, correction diagnosis

## Step 1 (prerequisite): Eigen `Matrix4d::inverse()` root cause
Cause: the upstream SVO Release flags (`-fno-signed-zeros`, also implied by `-ffast-math`; `svo_cmake/cmake/Modules/SvoSetup.cmake:33`) break Eigen 3.3.7's SSE double 4×4 inverse. All other Eigen ops used by SVO were verified identical to -O0; no estimation code uses a double 4×4 inverse. Flags kept (faithful baseline), so the build is unchanged and no M1 rerun is needed. Regression check: `analysis/tests/run_eigen_checks.sh`. Details: `docs/pipeline_notes.md`.

## Step 2: Phase 6 logging (commit e6848f3)
| Source | Content | Where |
|---|---|---|
| Registration node (`online_registration_fyp.py --event-dir`) | one `.npz` per **processed** KF: `cloud_raw` (as received, map frame), `cloud_filtered` (after the node's outlier + 0.1 m voxel filters), `T_init_world_body` (initial guess = VIO KF pose in map frame), `T_icp_init` (= I), `T_icp`, `T_result_world_body`, `fitness`, `inlier_rmse`, `n_correspondences`, `n_target`, `information_raw` (Σ HᵢᵀHᵢ before normalisation), `information_final` (as published), `valid`, `reject_reason`, `t_preproc_s`, `t_icp_s`, `t_info_s` | `runs/<id>/reg_events/kf_<stamp_ns>.npz` (written by a background thread after publishing; queue drained at shutdown) |
| svo_node (`log_registration_events`, default off, log only) | per registered pose received: wall time, stamp, branch (`initial_discard` / `not_initialized` / `to_handler` / `alignment`) | `runs/<id>/registration_events.csv` |
| Offline assembly (`analysis/phase6_events.py`) | per KF: published (bag) / processed / **dropped_by_queue** (queue_size=1 kept) / accepted / received + branch / **added** (each rosout "added" line matched to the latest earlier `to_handler` receipt) / poses / **GT pose interpolated** (≤ 10 ms bracket, linear/slerp) and raw-Vicon orientation / errors / timings | `runs/<id>/analysis/events.csv`, `events_poses.npz`, `events_summary.json` |
GT is attached offline only; no node reads GT beyond the D1 window. **Gate check:** all logs reload in Python (the assembly and diagnosis scripts read them for 5 runs).

## Step 3: M2 on V1_01, 5 runs (commit e6848f3, libsvo_ros c49078a025f0, logger on, rate 1.0, idle machine)
Runs `runs/2026-10-23_V1_01_easy_M2_p6_r01..r05`. Aggregate `runs/phase6_V1_01_M2_aggregate.json`. D1(B) online = offline in every run (t₀, all 157 window samples, Δyaw 1.3e-6°, ΔT 1.8e-8).

| Metric (mean ± std, n = 5) | M1 (VIO only) | **M2 (point-to-plane DT)** |
|---|---|---|
| Map-frame ATE, no alignment, t > t_end [m] | 0.132 ± 0.012 | **0.126 ± 0.012** |
| ATE 4-DoF aligned [m] | 0.056 ± 0.003 | **0.085 ± 0.012** |
| Rel. displacement error @ 5.9 m | 0.064 ± 0.007 | 0.095 ± 0.011 |
| @ 11.7 m | 0.062 ± 0.005 | 0.097 ± 0.015 |
| @ 17.6 m | 0.071 ± 0.005 | 0.113 ± 0.022 |
| @ 23.4 m | 0.075 ± 0.007 | 0.115 ± 0.020 |
| @ 29.3 m | 0.075 ± 0.006 | 0.118 ± 0.020 |
| Frontend [ms] / backend latency [ms] / svo_node CPU | 8.6 ± 2.5 / 51 ± 1 / 73 ± 2 % | 7.6 ± 0.1 / 52 ± 0 / 74 ± 1 % |

Per run (map-frame / 4-DoF ATE [m]): r01 0.117/0.076, r02 0.113/0.075, r03 0.139/0.093, r04 0.123/0.080, r05 0.137/0.102. ψ (D1(B)) 129.76–130.75°.

| Registration counts per run | r01 | r02 | r03 | r04 | r05 | mean ± std |
|---|---|---|---|---|---|---|
| KF clouds published | 186 | 186 | 183 | 183 | 185 | 184.6 ± 1.5 |
| processed | 185 | 186 | 183 | 183 | 185 | 184.4 ± 1.3 |
| **dropped by queue** (queue_size=1) | 1 | 0 | 0 | 0 | 0 | 0.2 ± 0.4 |
| accepted | 182 | 182 | 175 | 183 | 177 | 179.8 ± 3.6 |
| rejected (all: rotation > 5°) | 3 | 4 | 8 | 0 | 8 | 4.6 ± 3.4 |
| **added to backend** | 180 | 180 | 173 | 181 | 175 | 177.8 ± 3.6 |
Accepted − added = 2 per run, both explained: the first is discarded by the original `readInitialPose` logic; the last (the final keyframe) arrives after the last image, so no later frame consumes it (end-of-bag effect). Every rosout "added" line was matched (0 unmatched).

Registration runtime per KF (median of run medians): preprocessing 90.5 ± 0.8 ms, ICP 176.6 ± 1.2 ms, information matrix 33.0 ± 0.4 ms, **total 301 ± 2.5 ms** (KF interval ~0.8 s).

**Reading (no improvement claimed):** with 5 runs, M2's map-frame ATE (0.126 ± 0.012 m) is **not distinguishable** from M1's (0.132 ± 0.012 m). The single-run 0.088 m (M2 r02) was optimistic. M2 **degrades local consistency**: 4-DoF ATE +52 %, relative displacement error +48 to +57 %.

## Step 4: correction diagnosis (offline, from the 5 runs' logs; 894 accepted KFs)
Script `analysis/phase6_correction_diagnosis.py`; figure `docs/img/phase6_correction_diagnosis.png`; numbers `runs/phase6_correction_diagnosis.json`.

**Caveat on orientation:** after the position-based D1(B) alignment, the VIO body heading differs from the GT-estimate heading by ~7.6° and from raw-Vicon heading by ~5.4° (median). This matches the earlier finding that GT orientation on V1 is inconsistent with positions; absolute orientation errors vs GT are therefore not trustworthy here. Positions, and KF-to-KF variations of orientation, are.

| Error vs GT at accepted KFs | VIO before correction | ICP output | ICP better in |
|---|---|---|---|
| horizontal position (median / p90) [m] | 0.118 / 0.176 | **0.075** / 0.182 | 70 % |
| 3-D position (median / p90) [m] | 0.120 / 0.179 | **0.079** / 0.196 | 68 % |
| yaw vs GT estimate (median \|·\|) [°] | 7.58 | 4.96 | 99 % |
| yaw vs raw Vicon (median \|·\|) [°] | 5.36 | 2.77 | 99 % |

Corrections: median rotation 2.76°, median yaw −2.51°, median translation 0.103 m.

**Decomposition of the ~2.5° corrections:**
1. **Systematic part ≈ real VIO heading drift.** On M1 (no registration), the heading that best fits GT positions after the alignment window is −2.49° (mean; −1.85 … −3.09° per run) relative to the D1(B) window heading. The median M2 correction yaw is −2.51°. This comparison uses positions only, so it is unaffected by the GT-orientation issue. The corrections remove the heading drift accumulated after the 2 m alignment window, consistent with the yaw-vs-window-length trend (docs/frames.md).
2. **KF-to-KF part ≈ ICP noise** (the 0.9° figure is an *estimate*: the "needed correction" is derived from V1 orientation ground truth, GT estimate or raw Vicon, which is itself unreliable by a few degrees on V1; constant offsets cancel in the variability comparison, but time-varying GT-orientation errors would inflate the apparent needed-correction variation and lower the correlation). Correction yaw varies with std 0.93° between keyframes, while the needed correction varies with std only 0.49–0.51°. The correlation is weak (r = 0.32 vs raw Vicon, 0.39 vs GT estimate; slope 0.58–0.74; residual scatter 0.85–0.88°). Most of the per-KF variation is therefore noise of the ICP solution (≈ 350 landmarks per KF, rotation about the map origin), not tracking of VIO error.
3. Consequence, consistent with Step 3: absolute priors carrying ~0.9° of per-KF yaw noise (≈ 4 cm at 2.5 m lever arm) plus translation noise pull consecutive keyframes inconsistently. The median absolute position improves, but local consistency degrades, and the absolute map-frame ATE gain over the run is negligible.

**Conclusion (V1_01, M2 as published, 5 runs):** the ~2–3° corrections are mainly a systematic correction of real VIO heading drift, superimposed on ~0.9° per-keyframe ICP noise. The noise component explains the loss of local consistency; the systematic component is what a working registration should provide.

**Not changed:** M2's information weights (scale factors 9 / 2, power 1, exp(−0.9·rmse), max-normalisation) stay as the faithful baseline; no tuning. **The weighting question is covered by the D4 ablation** (Hessian-based covariance for both M2 and M3, 2×2 design, docs/PLAN.md).
