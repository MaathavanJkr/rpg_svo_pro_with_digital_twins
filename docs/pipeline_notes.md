# Pipeline notes (Phase 2: repository inspection)

Inspected on 2026-10-23 on branch `dev/volumetric-registration` (code identical to `master` @ `d641322`). The `voxel` branch was not read.
Everything below comes from reading the code; nothing was run. Items marked **[RISK]** affect later phases.

## 0. Build / runtime layout
- Catkin workspace: `~/Desktop/twinforge/catkin_ws`. `src/rpg_svo_pro_with_digital_twins` is a **symlink to this checkout**, so edits here are what gets built.
- `devel/lib/svo_ros/svo_node` was last built on **2026-10-22 17:46**. No C++ differs between master and anything built since (last C++ commit on master is 2024-12-02, and that build only changed Python/config/launch files). Every run records the binary's timestamp in `meta.txt`.
- `svo_ros` builds only one executable, `svo_node` ([svo_ros/CMakeLists.txt:77](../svo_ros/CMakeLists.txt#L77)).
- The registration stage is a **single Python node**: [svo_registration/scripts/online_registration.py](../svo_registration/scripts/online_registration.py). There is no C++ in `svo_registration`.
- **[RISK] Interpreter.** The shebang is `#!/usr/bin/env python`, but there is no `python` on this machine (only `python3`; `python-is-python3` is not installed). A run on 2026-10-22 did start this node (`~/.ros/log/latest/online_registration-2.log`); how `python` resolved then is unknown. Non-destructive fix for our launch files: `launch-prefix="python3"` on the node. Alternative: `sudo apt install python-is-python3` (needs your approval).
- Python dependencies of the node: `rospy`, `tf` (ROS); `open3d`, `transforms3d`, `numpy`, `yaml` (from `~/.local`). Run `roslaunch` **without** the venv active (see `docs/environment.md`).

## 1. Data flow (Oerlikon.launch, M2 as published)

```
svo_node (mono VIO + Ceres sliding window)
  │ on every KEYFRAME, only after T_w_vio is initialised and only if someone subscribes:
  │   /svo/w_backend_pose_points  (svo_msgs/PoseWithCloud, frame "world")
  │      .pose        = T_w_vio · T_vio_B          (KF body pose in map frame)
  │      .point_cloud = T_w_vio · {all non-fixed landmarks in the Ceres window}
  │      .header.stamp = KF timestamp
  ▼
online_registration.py
  │ crop the local cloud to a ±size_bounding_box box around the body → statistical outlier removal → 0.1 m voxel downsample
  │ crop the twin (already 1 m voxel-downsampled, z ≥ 0) to the same box
  │ Open3D point-to-plane ICP, init = I (correction expressed in the map frame), max 25 iterations
  │ validity: fitness ≥ 0.75, inlier_rmse ≤ inliers_rmse_thr, |t| ≤ max_translation_distance, angle ≤ max_rotation_distance
  │ reg_T_WB = T_icp · T_w_vio·T_vio_B
  ▼   /svo/w_registrated_pose (PoseWithCovarianceStamped, frame "world", stamp = KF stamp,
  │                            .covariance field holds an INFORMATION matrix, order [pos, rot])
svo_node globalposeCallback
  │ 1st message ever: discarded (readInitialPose)
  │ ignored until gp_initialized_ (set ONLY by GPS alignment)
  │ if refine_T_WL and not yet refined: used to re-estimate T_w_vio (4-DoF), NOT added as a factor
  │ else: converted to the VIO frame, T_vio_B = T_vio_w · reg_T_WB; information rotated by R_vio_w
  ▼
Ceres estimator: PoseError(T_vio_B_meas, information) on the KF whose stamp matches within 1 ms,
                 HuberLoss(50), at most 1 per KF
```

## 2. Registration node: [online_registration.py](../svo_registration/scripts/online_registration.py)

| Aspect | Implementation | Lines |
|---|---|---|
| Inputs | `--config-file` (key `registration_settings`), `--reference-map` (.ply) | [252-256](../svo_registration/scripts/online_registration.py#L252-L256) |
| Twin loading | `o3d.io.read_point_cloud`, drop points with **z < 0**, keep normals if present, then **`voxel_down_sample(1.0)` hard-coded** | [72-97](../svo_registration/scripts/online_registration.py#L72-L97) |
| Twin needs normals | `filter_city_twin` indexes `city_twin.normals` with the crop mask; point-to-plane needs target normals | [166-178](../svo_registration/scripts/online_registration.py#L166-L178) |
| Local cloud | from `PoseWithCloud.point_cloud` (already in map frame), box crop ±`size_bounding_box`, `remove_statistical_outlier(20, 0.5)`, `voxel_down_sample(0.1)` | [180-191](../svo_registration/scripts/online_registration.py#L180-L191) |
| ICP | `registration_icp(..., TransformationEstimationPointToPlane, init=I, max_iteration=25, max_correspondence_distance)` | [199-206](../svo_registration/scripts/online_registration.py#L199-L206) |
| Validation | fitness < 0.75 (**hard-coded**) or rmse > thr → reject; correction translation/rotation above limits → reject | [234-247](../svo_registration/scripts/online_registration.py#L234-L247) |
| Output pose | `icp.transformation @ T_world_body` | [211](../svo_registration/scripts/online_registration.py#L211) |
| "Covariance" | Σ over correspondences of HᵢᵀHᵢ with Hᵢ = [−(aᵢ×nᵢ)/‖aᵢ×nᵢ‖, −nᵢ], reordered to [pos, rot], raised to `information_matrix_power`, **normalised by its max entry**, × exp(−0.9·rmse), blocks scaled by `scale_factor_position²`, `scale_factor_rotation²`, cross terms by their product. Published in the `covariance` field, but the backend treats it as **information**. | [99-122](../svo_registration/scripts/online_registration.py#L99-L122), [218-226](../svo_registration/scripts/online_registration.py#L218-L226) |
| Topics | sub `/svo/w_backend_pose_points`; pub `/svo/w_registrated_pose`, `/svo/w_backend_points_registered`, `/city_twin` (latched); service `/visualize_city_twin` | [43-49](../svo_registration/scripts/online_registration.py#L43-L49) |

Remarks:
- **The information matrix is heuristic, not a Hessian-based covariance.** The rotational Jacobian row is normalised to unit length, which drops the lever-arm magnitude. `aᵢ` are absolute map-frame coordinates, so the rotation is about the map origin. The result is normalised by its largest entry, so its absolute scale is set by the two scale factors, not by the data. M3 must output information in the same convention (§6, open decision D4).
- `aᵢ × nᵢ = 0` (a point parallel to its normal) would divide by zero. This is unlikely, but not guarded.
- The rotation-angle check uses `arccos` without clipping. A trace slightly above 3 gives NaN, and NaN comparisons pass the check.
- Oerlikon config ([oerlikon.yaml](../svo_registration/config/oerlikon.yaml)): `max_correspondence_distance 8 m`, `inliers_rmse_thr 1.6`, `max_translation_distance 20`, `max_rotation_distance 15°`, `size_bounding_box 350 m`, `scale_factor_position 9`, `scale_factor_rotation 2`, `information_matrix_power 1`. These are city-scale values; **[RISK]** EuRoC needs its own config (new file).

### Does it work with mono landmarks?
**By design, yes.** The cloud is "all non-fixed `HomogeneousPointParameterBlock`s in the Ceres map" ([ceres_backend_publisher.cpp:176-188](../svo_ceres_backend/src/ceres_backend_publisher.cpp#L176-L188)). These exist identically in mono and stereo, and Oerlikon itself is mono (`pipeline_is_stereo: False`). Not yet verified at runtime. Phase 3 logs points per KF to confirm. Expected size: the window holds ≤ `max_n_kfs` KFs (10 for EuRoC defaults, 15 for Oerlikon) with ≤ `max_fts` (180/200) features each, so at most a few hundred to ~2 000 landmarks, before the node's 0.1 m voxel and outlier filters.

## 3. Global-measurement interface in svo_ros / svo_ceres_backend

| Piece | Where |
|---|---|
| Flags `use_global_measurements`, `use_map_registration` (both default **true**) | [svo_interface.cpp:98-146](../svo_ros/src/svo_interface.cpp#L98-L146) |
| Handlers are **always constructed**: `gp_settings_file` must contain `gp_settings`, `map_reg_settings_file` must contain `gpose_settings`, otherwise the node aborts (`LOG(FATAL)` / YAML exception) | [svo_factory.cpp:421-447](../svo_ros/src/svo_factory.cpp#L421-L447) |
| Subscribers: `gp_topic` (PoseStamped, default `/gps`), `map_alignement_T_w_b` (default `/mr`) | [svo_interface.cpp:693-719](../svo_ros/src/svo_interface.cpp#L693-L719) |
| GPS callback → W↔M alignment → `gp_initialized_` | [svo_interface.cpp:492-554](../svo_ros/src/svo_interface.cpp#L492-L554) |
| Registered-pose callback | [svo_interface.cpp:556-607](../svo_ros/src/svo_interface.cpp#L556-L607) |
| Pose conversion map→VIO, information rotation | [global_pose_handler.cpp:314-351](../svo/src/global_pose_handler.cpp#L314-L351) |
| Backend case dispatch | [ceres_backend_interface.cpp:658-760](../svo_ceres_backend/src/ceres_backend_interface.cpp#L658-L760) |
| Residual added | [estimator.cpp:1183-1225](../svo_ceres_backend/src/estimator.cpp#L1183-L1225) (GPS + registration, case D) and [estimator.cpp:1524-1566](../svo_ceres_backend/src/estimator.cpp#L1524-L1566) (registration only, case B); same code |
| Residual form: e = [p_meas − p ; 2·vec(q_meas ⊗ q⁻¹)], weighted by √information | [pose_error.cpp:105-123](../svo_ceres_backend/src/pose_error.cpp#L105-L123) |

**[RISK] Flag combinations.** (Before fix F3, §9, *any* `false` flag crashed the backend; the case analysis below holds after F3.) Registered poses reach the optimiser **only when both** `use_global_measurements` **and** `use_map_registration` are true (case 1). With only `use_map_registration=true`, the dispatcher falls into "case 3" (pure VIO), and `gp_initialized_` never becomes true. Within case 1, "case B" (registration, no GPS message at that frame) is supported, so GPS messages are not needed *per frame*, only for the initial alignment.

**[RISK] Timing.** A registration result is only used if its stamp matches a KF still in the sliding window (|Δt| < 1 ms, [estimator.cpp:1198](../svo_ceres_backend/src/estimator.cpp#L1198)). Slow registration therefore means lost measurements. This is a measurable effect for the M2/M3 runtime comparison. Log the number of results accepted vs published.

## 4. W→M initial alignment (as implemented)
1. **GPS alignment** ([global_positions_handler.cpp:391-440](../svo/src/global_positions_handler.cpp#L391-L440)). This starts once the backend scale is initialised and the VIO xy position is more than `alignement_d_min` (2 m) from the VIO origin. It pairs GPS positions with VIO positions extrapolated to the GPS stamp, and solves a **4-DoF yaw + translation** alignment (closed-form yaw, mean translation). It is accepted when the yaw variance from the alignment Hessian is below `alignement_threshold` (5e-4). On success: `T_w_vio` goes to the backend, publisher and pose handler, and `gp_initialized_ = true`.
   - With `estimate_world_vio_transform: false` the result is **T_w_vio = I** (the first GPS message still triggers it).
2. **Refinement from registration** (`refine_T_WL: true` in Oerlikon) ([global_pose_handler.cpp:388-485](../svo/src/global_pose_handler.cpp#L388-L485), [svo_interface.cpp:579-600](../svo_ros/src/svo_interface.cpp#L579-L600)). Registered KF positions (map) are paired with VIO KF positions, and a 4-DoF alignment is re-estimated (needs ≥ 3 pairs and yaw variance < 3e-4). Until this succeeds, registered poses are **only** used for alignment, not as factors.
3. Afterwards, `T_w_vio` is fixed (no re-alignment; noted as a TODO in the code).

**[RISK] Consequence for EuRoC (Phase 5).** There is no GPS, so as written, map registration never activates. Candidate approaches (**decision D1, needed by Phase 5**):
- (a) Publish a short synthetic `/gps` burst from GT only during the first seconds, enough for alignment, then stop. Uses GT for more than one pose, and the GPS residuals enter the optimiser while present.
- (b) `estimate_world_vio_transform: false` (T_w_vio = I) plus initialising the VIO frame to the first GT pose. SVO's `T_world_imuinit` only sets the initial rotation, not position, so this alone is insufficient. It still needs one `/gps` message.
- (c) A small, flag-guarded addition (e.g. a param `initial_T_w_vio` read at start-up that calls `setT_w_vio` and sets `gp_initialized_`), computed once from the first GT pose. Cleanest, but it touches `svo_ros` (not the registration stage), which CLAUDE.md asks to avoid. Needs your explicit approval.
- Whatever is chosen must be identical for M1/M2/M3, and `refine_T_WL` must be set the same way for M2 and M3.

## 5. Launch / config inventory

| File | Status |
|---|---|
| [svo_ros/launch/Oerlikon.launch](../svo_ros/launch/Oerlikon.launch) | Complete except the map path (default `$(find svo_registration)/maps/Oerlikon.ply` is missing; override `reference_map:=`). Registration node lacks a `python3` prefix. `/use_sim_time` false, while the bag clock starts at 136 s; check in Phase 3. |
| `svo_ros/param/Oerlikon/{calibration,mono_vio,gp_settings,gpose_settings}.yaml` | present |
| `svo_ros/launch/Default/euroc_vio_{mono,stereo,gp_mono,gp_stereo,mono_test}.launch` | **[RISK] broken paths**: reference `param/calib/euroc_mono.yaml`, `param/gp_settings.yaml`, `param/vio_mono.yaml`, `rviz_config_vio*.rviz`, none of which exist. The files actually live in `param/Default/`. No `map_reg_settings_file` is passed, so the node would abort (§3). |
| [svo_ros/param/Default/euroc_mono.yaml](../svo_ros/param/Default/euroc_mono.yaml) | EuRoC cam0 calibration + IMU noise. `T_B_C` equals EuRoC `cam0/sensor.yaml` `T_BS` exactly. |
| [svo_ros/param/Default/vio_mono.yaml](../svo_ros/param/Default/vio_mono.yaml) | EuRoC-style VIO params (`max_fts 180`, `max_n_kfs 10`, `map_scale 1.5`, `ceres_num_iterations 3`, `ceres_max_iteration_time 0.04`). It differs from Oerlikon's `mono_vio.yaml` in ~20 tuning values. |
| [svo_ros/RVIZ/rviz_config_digital_twins.rviz](../svo_ros/RVIZ/rviz_config_digital_twins.rviz) | Fixed Frame = `vio`; the twin and registered clouds are in `world`. **No TF between `vio` and `world` is broadcast anywhere** (no static publisher in any launch). `svo_node` broadcasts TF only in `vio` (`visualizer.cpp:272`), so the frame `world` does not exist in TF at all; with Fixed Frame `world`, RViz errors and draws nothing until a frame is provided (workaround in `docs/phase3_oerlikon.md`). |

**Plan for Phase 4 (M1):** a **new** launch file `svo_ros/launch/euroc/euroc_vio_mono_m1.launch` (originals untouched). It uses `param/Default/euroc_mono.yaml` and `param/Default/vio_mono.yaml`, sets `use_global_measurements=false` and `use_map_registration=false`, and points both settings files at valid YAMLs (new copies under `svo_ros/param/euroc/`).

## 6. Evaluation tooling
- `svo_benchmarking/scripts/benchmark.py` expects ASL data prepared under `svo_benchmarking/data/` and a `svo_benchmark` node. **That node is not built** (only `svo_node` exists). Not usable without extra work.
- `rpg_trajectory_evaluation` is in the workspace (`catkin_ws/src`). `evo` 1.31.1 is in `~/venvs/fyp`.
- **Plan:** record `/svo/vio_backend_pose_imu` (T_W B, frame `vio`) and `/svo/w_backend_pose_imu` (T_M B, frame `world`) with `rosbag record`, convert to TUM, and evaluate with evo against `state_groundtruth_estimate0` (B in V). These topics are only published when someone subscribes, so `rosbag record` must be running.
- M1 has no `world` output (T_w_vio is never initialised). M1's map-frame unaligned ATE must use the same documented initial alignment applied offline.

## 7. Other packages (not used by the DT pipeline)
- `gps_simulator`: synthesises GPS (multipath GMM, satellite-visibility GP) **from ground-truth poses** in ENU. Used for Oerlikon-style data. For EuRoC it would turn GT into a measurement and must not be used beyond the documented alignment (see D1).
- `svo_global_map`, `svo_online_loopclosing`, `svo_pgo`: not involved (`runlc: False`; loop closing hard-fails in this version: [svo_interface.cpp:158-160](../svo_ros/src/svo_interface.cpp#L158-L160)).
- `svo_registration/Dataset/*`, `Evaluation/*`: upstream data-preparation and plotting scripts for Oerlikon / Flightmare.

## 8. Decisions (resolved 2026-10-23)
- **D1 → option (c), implemented in Phase 5.** A flag-guarded addition in `svo_ros`, **off by default**, so Oerlikon and original behaviour are unchanged. No backend or residual changes. GT is used **once**: the first keyframe after backend initialisation, with GT and VIO poses at the same timestamp, gives a 4-DoF (yaw + translation) T_M_W. Roll/pitch come from VIO gravity. This sets `T_w_vio` and `gp_initialized_`. T_M_W, timestamp and yaw are logged to the run folder, and the identical procedure is applied offline for M1. After alignment a static TF `world→vio` is broadcast. Main EuRoC runs use `refine_T_WL: false` for both M2 and M3; refinement is a later ablation. See `docs/frames.md`.
- **D2 → `launch-prefix="python3"`** in a new launch copy (no sudo, originals untouched). Done in Phase 3: `svo_ros/launch/fyp/oerlikon_m2.launch`.
- **D3 → parameterised copy of the registration node for EuRoC (Phase 5).** Parameters: twin voxel size, z filter, fitness threshold, with defaults equal to the original hard-coded values (1.0 m, z ≥ 0, 0.75). The Leica scan is preprocessed **once** offline (crop outliers, downsample to 2–5 cm, estimate normals) into `dataset/derived/`, and M2 and M3 load the identical file. Bug fixes in the copy only, logged below.
- **D4 → main comparison:** M3 outputs information in the **same heuristic convention** as M2, using the normalised ∇D as the normal. **Ablation:** a Hessian-based covariance for both M2 and M3 (2×2 design). Recorded in `docs/PLAN.md`.

## 9. Bug fixes (F1/F2 in our copies only; F3 approved as an in-place svo_ros fix)
| # | Original location | Issue | Fix in copy | Phase |
|---|---|---|---|---|
| F1 | `online_registration.py:243` | `arccos` of trace term not clipped → NaN when trace > 3 by rounding; NaN passes the rotation check | clip the argument to [−1, 1] | 5 (planned) |
| F2 | `online_registration.py:109-110` | `cross/‖cross‖` divides by zero when a point is parallel to its normal | skip or guard zero-norm rows | 5 (planned) |
| F3 | `svo_ros/src/svo_interface.cpp` (constructor and `processImageBundle`) | `CeresBackendInterface::use_global_measurements_` / `use_map_registration_` default to `true` and were only ever set to `true`, so `use_global_measurements=false` segfaulted on the first frame (null `GlobalPositionsHandler`). VIO-only could not run. | Assign both configured flags to the backend explicitly (no change when both are true). **Applied 2026-10-23** (approved option A), in `svo_ros` itself, not in a copy. | 4 (done) |

### F3 in detail (applied 2026-10-23, commit 06073b3)
- **Symptom.** `euroc_m1.launch` (`use_global_measurements=false`, `use_map_registration=false`): `svo_node` segfaults (exit −11) on the first image. Stack: `SvoInterface::monoCallback → processImageBundle → FrameHandlerBase::addImageBundle → CeresBackendInterface::loadMapFromBundleAdjustment → addStatesInertialGpAndGposeMeasurementsToBackend → GlobalPositionsHandler::getMeasurementTillTime → pthread_mutex_lock` on address 0xa0 (null handler).
- **Root cause.** `CeresBackendInterface::use_global_measurements_` and `use_map_registration_` are initialised to `true` (`svo_ceres_backend/include/svo/ceres_backend_interface.hpp:286-287`). `svo_interface.cpp` only ever assigned `true` (inside `if (use_global_measurements_)` / `if (use_map_registration_)`), never `false`. With both params false, the backend still took "case 1" (GP + MR) while `setGpHandler`/`setGposeHandler` had not been called, so the handler pointers were null. The upstream `Default/euroc_vio_mono.launch` would hit the same crash once its paths are fixed.
- **Change.** Two assignments after the existing blocks in the constructor, and the same two in `processImageBundle` where the backend is re-created:
  `ceres_backend_interface_->use_global_measurements_ = use_global_measurements_;`
  `ceres_backend_interface_->use_map_registration_ = use_map_registration_;`
- **Effect.** With both flags true (Oerlikon, M2, M3) the assigned values equal the previous ones, so there is no behavioural change by construction (checked empirically by an Oerlikon run on this build, see `docs/phase3_oerlikon.md`). With both false the backend now takes "case 3" (pure VIO), as the code intended. Backend and residuals are untouched.
- **Verified** by the MH_01 M1 runs (3682/3682 frames, ATE posyaw 0.140 m).

### Eigen `Matrix4d::inverse()` under `-fno-signed-zeros`: ROOT CAUSE (2026-10-23)
- **Symptom:** in `libsvo_ros`, `T.inverse()` on an `Eigen::Matrix4d` returned wrong values (invalid rotation block, sign-flipped translation), found while verifying the D1 static TF.
- **Not the cause (checked):** mixed Eigen versions or headers. Every workspace package compiles against the system Eigen 3.3.7 (`-I/usr/include/eigen3`); no Linuxbrew or `eigen_catkin` Eigen headers are used. Compiler: `/usr/bin/c++`, GCC 9.4.0, `CMAKE_BUILD_TYPE=Release`, CPU i7-8700. `-march=native` / `-mno-avx` are irrelevant.
- **Cause:** the SVO Release flags in `svo_cmake/cmake/Modules/SvoSetup.cmake:33` (`-O3 … -fno-signed-zeros … -ffast-math -fno-finite-math-only`, upstream, used by all SVO packages). With **`-fno-signed-zeros`** (set explicitly and also implied by `-ffast-math`), Eigen 3.3.7's SSE kernel for the **double** fixed-size 4×4 inverse produces wrong results. Minimal test `analysis/tests/eigen_inverse_fastmath.cpp`: error 4.16 on a rigid transform, and A·A⁻¹ − I = 2 on a generic 4×4 matrix. It reproduces with plain `-O3 -fno-signed-zeros` and with `-O3 -ffast-math`, and is gone only when both are removed. `Matrix4f` and `Matrix3d` inverses are unaffected.
- **Scope (checked):** under the exact SVO flags, quaternion product / conjugate / inverse, vector rotation, 6×6 and 3×3 inverse, 4×4 multiplication and LDLT solve are identical to an `-O0` build to 12 decimals (`analysis/tests/eigen_ops_fastmath.cpp`). Source audit of `svo*`, `vikit`, `rpg_common`, `gps_simulator`: no fixed-size double 4×4 inverse and no projective `Eigen::Transform` inverse; minkindr's `QuatTransformation::inverse()` uses quaternion operations. Ceres and OpenGV are built without these flags.
- **Fix / decision:** the upstream build flags are **kept unchanged** (faithful baseline; changing `-ffast-math` would alter SVO's numerics everywhere). Our code never calls `Matrix4d::inverse()` (the D1 static TF uses an explicit rigid inverse). The build did not change, so no M1 rerun is needed. Regression check: `analysis/tests/run_eigen_checks.sh` (expected output: inverse WRONG under SVO flags, ~0 at -O0; other ops identical). **Rule for new C++ in this workspace: never use a double fixed-size 4×4 `.inverse()`; write rigid inverses explicitly.**
