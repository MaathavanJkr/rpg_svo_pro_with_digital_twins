# Coordinate frames and transforms

Notation: `T_A_B` maps coordinates from frame B to frame A (p_A = T_A_B · p_B). Every transform lists its source.
Started in Phase 2 (2026-10-23); extend it whenever a new transform is used.

## Frames

| Symbol | Meaning | ROS `frame_id` in code | Notes |
|---|---|---|---|
| **B** | Body = IMU frame | — | EuRoC: IMU (ADIS16448) frame. SVO's state `T_W_B` / `T_WS` is this frame. |
| **C0, C1** | cam0, cam1 optical frames | `cam0`, `cam1` | Mono pipeline uses C0 only. |
| **W** | VIO world: gravity-aligned, origin and yaw set at VIO initialisation | **`"vio"`** | `T_world_imuinit` = identity quaternion in both EuRoC and Oerlikon params. Yaw and position are arbitrary w.r.t. M. |
| **M** | Digital-twin (map) frame | **`"world"`** | Oerlikon: frame of `Oerlikon.ply`, and the `/gps` local coordinates (`coordinates: 'local'`). EuRoC: Leica scan frame R. |
| **V** | Vicon / EuRoC ground-truth world | — | EuRoC `state_groundtruth_estimate0` gives T_V_B. |
| **R** | Leica scan (pointcloud0) frame | — | EuRoC only. |

**[Naming hazard]** In the code, `"world"` / `w_` / `T_w_vio` refer to **M** (the map), and `"vio"` / `T_W_B` in backend states refer to **W**. So `T_w_vio` in the code is our **T_M_W**, and `state.get_T_W_B()` is our **T_W_B**.

## Transforms

| Transform | Value / how obtained | Source |
|---|---|---|
| T_B_C0 (EuRoC) | 4×4 from calibration, identical to dataset cam0 `T_BS` | `svo_ros/param/Default/euroc_mono.yaml` `T_B_C`; `mav0/cam0/sensor.yaml` |
| T_B_C (Oerlikon) | 4×4 | `svo_ros/param/Oerlikon/calibration.yaml` `T_B_C` |
| T_V_R (EuRoC, scan→Vicon) | **Identity** | `mav0/pointcloud0/sensor.yaml` `T_WR` (here "W" in the dataset = V). Visually consistent in the Phase 1 frame check (`docs/img/V*_framecheck_gt_overlay.png`). |
| M (EuRoC) | Defined as M := R = V | follows from T_V_R = I |
| T_V_B (GT) | per-timestamp pose; `T_BS` of the GT sensor = I | `mav0/state_groundtruth_estimate0/data.csv` (+ `sensor.yaml`) |
| T_B_vicon-marker | 4×4 (not used; we use the GT estimate) | `mav0/vicon0/sensor.yaml` `T_BS` |
| T_B_prism (MH Leica) | translation (0.0749, −0.0185, −0.1202) m (not used) | `mav0/leica0/sensor.yaml` |
| **T_M_W** (`T_w_vio`) | Oerlikon: 4-DoF (yaw + translation) GPS alignment, optionally refined once from registered poses. **EuRoC: decision D1(c)**, see below | `svo/src/global_positions_handler.cpp:391-440`, `svo/src/global_pose_handler.cpp:388-485` |
| T_M_B (registration output) | `T_icp · T_M_W · T_W_B` | `online_registration.py:211` |
| T_W_B measurement fed to the backend | `T_W_M · T_M_B`; the 6×6 information is rotated by `R_W_M` on both blocks | `svo/src/global_pose_handler.cpp:314-351` |
| Published estimates | `/svo/vio_backend_pose_imu` = T_W_B; `/svo/w_backend_pose_imu` = T_M_W · T_W_B | `svo_ceres_backend/src/ceres_backend_publisher.cpp:56-120` |
| Local registration cloud | landmarks p_W mapped to p_M = T_M_W · p_W | `ceres_backend_publisher.cpp:221-236` |

## Notes and pitfalls
- No TF is broadcast between `vio` (W) and `world` (M). RViz can only show one of the two groups correctly at a time, unless a TF is added.
- The ICP correction `T_icp` is a left-multiplied correction in M, with rotation about M's origin, not about the body.
- Evaluation: map-frame (unaligned) ATE compares T_M_B against T_V_B, using T_V_M = I (EuRoC). VIO-only (M1) has no T_M_W in the code, so the same documented initial alignment must be applied offline.

## EuRoC initial alignment: stated assumption (decision D1(B), amended 2026-10-23)
- **Assumption (state in the report): "initial alignment from GT positions over the first L m of flight", L = 2 m.** The VIO frame W is aligned to the map M once, by a 4-DoF (yaw + translation) least-squares fit of the VIO positions to the ground-truth positions over a short window at the start of the flight. This stands in for a known take-off pose/heading. No GT is used online after the window, and **GT orientation is not used at all**.
- **t₀** (window start) = the first keyframe with backend and scale both initialised AND ground truth available at its timestamp, observed with the FYP frame-event log in `svo_ros` (`log_frame_events`, off by default, log only). Per processed image: `is_kf` = `getLastFrames()->at(0)->isKeyframe()`; `backend_init` = backend exists and `lastOptimizedBundleId() >= 0`; `scale_init` = `isBackendScaleInitialised()`.
  - **GT available** at t: consecutive GT samples t_a ≤ t ≤ t_b (`state_groundtruth_estimate0`) with t_b − t_a ≤ **10 ms**; GT position at t is interpolated linearly between them.
- **Window:** the `/svo/vio_backend_pose_imu` samples from t₀ up to and including the first sample at which the GT path length reaches **L = 2 m**. The path length is the polyline through the GT positions interpolated at the kept sample stamps; samples without a 10 ms GT bracket are skipped. On V1_01 the window lasts 7.8 s (~157 samples).
- **Fit:** closed form. a_k = p̂_k − mean(p̂), b_k = p_k − mean(p); ψ = atan2(Σ(a_x b_y − a_y b_x), Σ(a_x b_x + a_y b_y)); R_M_W = R_z(ψ); t_M_W = mean(p) − R_z(ψ)·mean(p̂). Roll/pitch of W stay as estimated by VIO (gravity). M := V for EuRoC (scan frame = Vicon frame).
- **Map-frame ATE window:** estimate samples strictly after the alignment window ends (t > t_end). No metric is computed on the samples the alignment was fitted to; online methods only publish map-frame poses from t_end anyway.
- **Reference implementation:** `analysis/initial_alignment.py` (`window_alignment`, `gt_bracket`, `t0_from_kf_log`). **The online implementation (`svo_ros`, `d1_alignment`, `d1_window_length`) uses the identical rule**; numerical match is verified per run.
- **Why L = 2 m:** mirrors the original pipeline's `alignement_d_min: 2`. Offline check on the 5 M1 runs (V1_01), judged by yaw stability, not ATE: mean yaw 128.25° (L 0.5 m, std 0.70°), **130.37° (1 m, 0.38°), 130.07° (2 m, 0.25°)**, 129.17° (3 m), 127.74° (5 m), 128.46° (8 m). L = 1–2 m is a plateau (Δ 0.3°, lowest run-to-run spread at 2 m). Shorter windows have too little motion; longer windows drift as VIO yaw drift accumulates inside the window.
- **Why not GT orientation (legacy D1, superseded):** on V1 the orientation of `state_groundtruth_estimate0` is inconsistent with raw Vicon by ~2–3° tilt (Phase 4). The single-instant orientation-based yaw differed from the position-based yaw by **+7.3°**, from what the scan geometry asks for (ICP corrections +4.1° ± 0.5° in M2 r01) and from the best whole-trajectory fit (+4.75°). **Part of this offset may come from early VIO yaw/position error, not only from GT orientation**; the position fit absorbs both, consistently for all methods.
- `refine_T_WL: false` for M2 and M3 in the main runs (fixed alignment); refinement and injected yaw offsets are a Phase 10 experiment.
- M1: the identical computation is applied offline to the M1 trajectory.
