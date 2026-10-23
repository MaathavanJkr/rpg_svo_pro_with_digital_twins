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
