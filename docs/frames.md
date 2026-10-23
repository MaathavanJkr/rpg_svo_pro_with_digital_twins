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

## EuRoC initial alignment: stated assumption (decision D1(c), implemented in Phase 5)
- **Assumption (state in the report):** the VIO frame W is aligned to the map M **once**, using the ground-truth pose at a single timestamp. This mimics a known take-off pose. No other GT is used online.
- Timestamp t₀ = the **first keyframe with backend and scale both initialised AND ground truth available at its timestamp** (decision 2026-10-23; the GT condition was added after MH_01 showed the first such KF 0.22 s before EuRoC GT starts). It is observed with the FYP frame-event log in `svo_ros` (`log_frame_events`, off by default, log only). Each row is one processed image: `is_kf` = `getLastFrames()->at(0)->isKeyframe()`; `backend_init` = backend exists and `lastOptimizedBundleId() >= 0`; `scale_init` = `isBackendScaleInitialised()`.
  - **GT available** at t: there are consecutive GT samples t_a ≤ t ≤ t_b (`state_groundtruth_estimate0`) with t_b − t_a ≤ **10 ms**. KFs failing this are skipped; t₀ is the first KF passing all conditions.
  - **GT pose at t₀** is interpolated between that same pair: position linear, orientation slerp.
  - **The Phase 5 online implementation must use the identical rule (same event, same 10 ms bracket, same interpolation).** Reference implementation: `analysis/initial_alignment.py` (`gt_bracket`, `gt_at`, `t0_from_kf_log`).
- T_M_W is **4-DoF**: yaw ψ and translation only. **Exact definition** (reference implementation `analysis/initial_alignment.py`; Phase 5 online code must match it numerically):
  - T_W_B(t₀) = the VIO estimate sample with stamp exactly t₀; T_V_B(t₀) = GT interpolated at t₀ as above.
  - ΔR = R_V_B(t₀)·R_W_B(t₀)ᵀ, ψ = atan2(ΔR₁₀, ΔR₀₀), R_M_W = R_z(ψ), t_M_W = p_V_B(t₀) − R_z(ψ)·p_W_B(t₀).
  - With T_V_M = I, V and M coincide. Roll/pitch of W stay as estimated by VIO (gravity).
  - Map-frame ATE window: all estimates with stamp ≥ t₀, no further alignment.
- Implementation: flag-guarded addition in `svo_ros` (default off). Sets `T_w_vio` + `gp_initialized_`, then broadcasts a static TF `world`→`vio`. T_M_W, t₀ and ψ are logged to the run folder.
- `refine_T_WL: false` for M2 and M3 in the main runs (fixed alignment). Refinement is a separate ablation.
- M1: the identical computation is applied offline to the M1 trajectory, at the same definition of t₀.
