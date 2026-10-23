# Phase 4: VIO-only baseline (M1)

Launch `svo_ros/launch/fyp/euroc_m1.launch` (mono-inertial, cam0 + `/imu0`, `use_global_measurements=false`, `use_map_registration=false`), configs `svo_ros/param/fyp/euroc/` (unmodified copies of `param/Default/euroc_mono.yaml`, `vio_mono.yaml`; Oerlikon gp/gpose settings only so the node starts). Harness `analysis/run_euroc.sh`, evaluation `analysis/eval_vio.py`, D1 alignment `analysis/initial_alignment.py`. Bag played at rate 1.0, machine idle (load avg ≈ 1 on 12 cores).

## Prerequisite fix
VIO-only could not run in this repo (upstream bug, F3 in `docs/pipeline_notes.md`): the backend's global-measurement flags were never set false, so `use_global_measurements=false` segfaulted on the first frame. Fixed in `svo_ros/src/svo_interface.cpp` (approved option A, commit 06073b3).

## MH_01_easy sanity check (1 run + 1 logger-off reference)
| Run | ATE posyaw RMSE [m] | ATE SE3 [m] | RPE trans mean [m] @ 8.1 / 16.1 / 24.2 / 32.3 / 40.3 m | RPE rot mean [°] @ same | Frontend [ms] | Backend latency median [ms] | svo_node CPU |
|---|---|---|---|---|---|---|---|
| logger off | 0.139 | 0.114 | 0.131 / 0.180 / 0.196 / 0.212 / 0.234 | 0.77 / 1.14 / 1.39 / 1.59 / 1.78 | 8.8 | 47 | 77 % of 1 core |
| **logger on (official)** | **0.140** | 0.116 | 0.122 / 0.200 / 0.204 / 0.229 / 0.253 | 0.75 / 1.06 / 1.25 / 1.35 / 1.40 | 8.7 | 46 | 75 % of 1 core |

- All 3682 images processed in both runs; frontend real-time factor 0.17–0.18 (mean processing time / 50 ms).
- **Reference:** Cioffi & Scaramuzza 2020 (arXiv:2003.04159, Table I), VIO-only, monocular SVO + Ceres backend, posyaw alignment with rpg_trajectory_evaluation, median of 3 runs: **MH01 0.188 m**, V101 0.061 m. Our 0.140 m is the same order, not off. (Different parameter files/versions are possible; single run.)
- ATE window: whole estimate stream, including the first ~1 s before GT starts (unassociated poses are dropped by the 3 ms association).

## Frame-event logger overhead
| Measure | Value |
|---|---|
| Direct cost of `logFrameEvent()` (timed inside the call) | **5.4 µs mean, 148 µs max** per frame (3682 frames) |
| Frontend processing time, flag off → on | 8.8 → 8.7 ms (frontend timer excludes the logger; difference is run-to-run noise) |
| svo_node CPU, off → on | 77 % → 75 % of one core |
| ATE posyaw, off → on | 0.139 → 0.140 m |
The logger costs ~0.06 % of the frontend time per frame.

## D1 initial alignment on M1 (amended rule, approved 2026-10-23)
Rule (`docs/frames.md`): t₀ = first keyframe with backend and scale initialised **and GT available** (bracketing GT samples ≤ 10 ms apart); GT pose interpolated (linear / slerp). The unamended rule failed on MH_01: the first qualifying KF (bundle 17, 0.85 s after the first image) is 0.22 s before EuRoC GT starts.

| Run | t₀ | t₀ after first image | yaw ψ | Map-frame ATE (no alignment, window t₀→end) |
|---|---|---|---|---|
| MH_01 M1 r01 | 1403636581.113555 | 1.35 s | −15.10° | RMSE **0.245 m**, median 0.257 m, max 0.429 m, final 0.263 m (n=3633, 181.6 s) |

(The earlier `PREVIEW_*.json` in the run folder gave identical numbers and is superseded by `initial_alignment.json` / `map_frame_ate.json`.)

## V1_01_easy: 5 runs (M1, logger on, rate 1.0, idle machine)
Runs `runs/2026-10-23_V1_01_easy_M1_r01..r05` (commit 3688064, libsvo_ros md5 a8df1f97d777). Aggregate: `runs/phase4_V1_01_M1_aggregate.json` (`analysis/aggregate_euroc.py`, sample std).

| Run | ATE 4-DoF [m] | Map-frame ATE [m] | t₀ | ψ (yaw offset) | Frames | Frontend [ms] | Latency [ms] | CPU (1 core) |
|---|---|---|---|---|---|---|---|---|
| r01 ⚠ | 0.051 | 0.224 | 1403715279.062143 | 123.24° | **2887/2912** (25 dropped) | 13.0 | 49 | 75 % |
| r02 | 0.058 | 0.245 | 1403715279.062143 | 122.50° | 2912/2912 | 7.6 | 51 | 73 % |
| r03 | 0.058 | 0.237 | 1403715279.062143 | 122.55° | 2912/2912 | 7.4 | 51 | 72 % |
| r04 | 0.056 | 0.235 | 1403715279.062143 | 123.23° | 2912/2912 | 7.5 | 51 | 72 % |
| r05 | 0.057 | 0.247 | 1403715279.062143 | 122.52° | 2912/2912 | 7.4 | 51 | 71 % |
| **mean ± std** | **0.056 ± 0.003** | **0.237 ± 0.009** | identical | 122.81 ± 0.39° | | 8.6 ± 2.5 | 51 ± 1 | 73 ± 2 % |

- ATE SE(3)-aligned: 0.055 ± 0.003 m. t₀ is 5.80 s after the first image (4.76 s after GT starts), the same keyframe in every run.
- **Reference:** Cioffi & Scaramuzza 2020, VIO-only V101 **0.061 m** (posyaw, median of 3). Ours 0.056 ± 0.003 m: consistent, not off.
- ⚠ **r01 is flagged but kept in the statistics** (decision 2026-10-23): it processed 2887/2912 images and had a higher frontend time (13.0 ms). It was the first V1_01 run after the Oerlikon run; cause not determined (possibly cold disk cache for the bag). Its accuracy is in line with the others.

RPE (evo, all pairs, segments = 10–50 % of the 58.6 m GT path), mean ± std over runs:

| Segment | 5.9 m | 11.7 m | 17.6 m | 23.4 m | 29.3 m |
|---|---|---|---|---|---|
| translation [m] | 0.286 ± 0.005 | 0.239 ± 0.006 | 0.293 ± 0.005 | 0.342 ± 0.010 | 0.203 ± 0.013 |
| rotation [°] | 4.17 ± 0.05 | 3.67 ± 0.07 | 3.83 ± 0.07 | 3.55 ± 0.07 | 4.38 ± 0.08 |

**These RPE values are dominated by a ground-truth orientation discrepancy, see below; read them with that caveat.**

Orientation-free drift metric (decision 2026-10-23): relative displacement error after the 4-DoF alignment, e = ‖(p̂_j − p̂_i) − (p_j − p_i)‖ over the same GT path-length segments (`analysis/eval_vio.py`, `rel_disp_m`). It uses positions and one world-frame yaw only, so it is not affected by GT body orientation.

| Segment | 5.9 m | 11.7 m | 17.6 m | 23.4 m | 29.3 m |
|---|---|---|---|---|---|
| **rel. displacement error [m]** | **0.064 ± 0.007** | **0.062 ± 0.005** | **0.071 ± 0.005** | **0.075 ± 0.007** | **0.075 ± 0.006** |
| RPE translation [m] (for comparison) | 0.286 | 0.239 | 0.293 | 0.342 | 0.203 |

Check on MH_01 (GT orientation consistent there): rel. displacement 0.122 / 0.188 / 0.195 / 0.211 / 0.231 m vs RPE translation 0.122 / 0.200 / 0.204 / 0.229 / 0.253 m. The two agree, which confirms that the V1_01 RPE inflation comes from orientation.

## Finding: EuRoC GT orientation vs raw Vicon (affects orientation-based metrics)
On V1_01 the RPE rotation error is ~4° at every segment length (MH_01: 0.75 → 1.4°), and translation RPE is ~5× the ATE. Diagnosis (evaluation only):
- Roll/pitch (tilt) error of the estimate vs `state_groundtruth_estimate0` is ~2.4° for the whole run, identical in all 5 runs (MH_01: converges to 0.3–0.8°). Not a timestamp offset (minimum at Δt = 0…+20 ms).
- **The estimate agrees with the raw Vicon orientation** (`vicon0` mapped to the body frame with `vicon0/sensor.yaml` `T_BS`) to **0.50° tilt**, while **the GT estimate differs from raw Vicon by 2.26° tilt** (3.17° full rotation). The best Vicon time shift (+10 ms) does not remove it.
- Other sequences (GT estimate vs raw Vicon, median tilt / full rotation): V1_02 0.45° / 0.88°, V1_03 0.81° / 1.40°, V2_01 2.90° / 4.06°, V2_02 4.38° / 6.94°, V2_03 4.34° / 7.01°. V2 is confounded by a time offset beyond the ±100 ms searched; unresolved.
- **Interpretation (not a conclusion):** on V1_01 the orientation of `state_groundtruth_estimate0` appears inconsistent with the Vicon-measured body orientation by a few degrees. Which one is closer to the truth is not established (raw Vicon depends on the `T_BS` calibration; the GT estimate is the dataset's official reference and what the literature uses).
- **Impact:** (i) position-only metrics (4-DoF ATE, map-frame ATE positions) are unaffected by the body orientation itself; (ii) RPE (both translation, expressed in the start frame, and rotation) and any ATE_R are inflated; (iii) the D1 yaw ψ uses the GT orientation at t₀, so ψ can carry ~1–2° of this discrepancy, which shifts the map-frame ATE through the lever arm. It is the same t₀ and GT for M1/M2/M3, so method comparisons stay fair, but absolute map-frame numbers are affected. Decision requested at the Phase 4 gate.

### Decision (Phase 4 gate, 2026-10-23)
**Option (a):** keep `state_groundtruth_estimate0` for all metrics and for the D1 alignment. RPE is reported with the caveat above, plus the orientation-free relative displacement error.

**The D1 alignment is identical for M1, M2 and M3** (same t₀ event rule, same GT, same interpolation), so any yaw bias it inherits from the GT orientation is **common to all three methods**. Method differences in map-frame ATE are not caused by it; absolute map-frame values are.

### Sensitivity check (reported only; not a D1 change): t₀ yaw from raw Vicon
`analysis/initial_alignment.py --yaw-source vicon`: orientation at t₀ from `vicon0` (body orientation R_RS·R_BSᵀ, slerp on raw stamps), translation still from the GT estimate. Output files `*_sens_vicon_yaw.json`.

| Run | ψ (GT estimate, D1) | ψ (raw Vicon) | Δψ | Map-frame ATE, D1 [m] | Map-frame ATE, Vicon yaw [m] | Δ [m] |
|---|---|---|---|---|---|---|
| r01 | 123.24° | 125.46° | +2.22° | 0.224 | 0.127 | −0.097 |
| r02 | 122.50° | 124.72° | +2.22° | 0.245 | 0.149 | −0.096 |
| r03 | 122.55° | 124.77° | +2.22° | 0.237 | 0.138 | −0.099 |
| r04 | 123.23° | 125.45° | +2.22° | 0.235 | 0.139 | −0.096 |
| r05 | 122.52° | 124.74° | +2.22° | 0.247 | 0.149 | −0.098 |
| mean | | | **+2.22°** | **0.237 ± 0.009** | **0.140 ± 0.009** | **−0.097** |

**Reading:** the yaw taken from the official GT orientation at t₀ differs from the raw-Vicon yaw by 2.2°. That accounts for ~0.1 m (40 %) of M1's map-frame ATE on V1_01. The effect is a common offset for M1/M2/M3; it must be stated next to any absolute map-frame number for V1_01. (Raw Vicon was used on its own timestamps; the best-fit +10 ms shift found earlier was not applied.)
