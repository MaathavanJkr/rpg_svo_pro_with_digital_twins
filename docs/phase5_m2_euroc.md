# Phase 5: port M2 (original point-to-plane registration) to EuRoC V1

Decisions applied: D1 (amended t₀, 10 ms GT bracket, interpolation), D3 (preprocessed twin, parameterised node copy), `refine_T_WL: false`.

## 5a. Accepted-but-not-added registrations (Oerlikon): diagnosed
**A logging artifact, not a pipeline effect.** Category: other. `svo_node` stdout is block-buffered into `svo.log` and the tail is lost when the node is SIGTERMed at shutdown. `rosout.log` shows every registration published after refinement was added (r01 199/199, r02 202/202, r03 198/198, newbin 209/209, F3 205/205). Stamps equal keyframe stamps exactly; the window holds 15 KFs (~15 s), so window exit is implausible. No precision bug, so no fix proposed. Harness now copies `rosout.log`, and our launches line-buffer output. Details: `docs/phase3_oerlikon.md` note 5b.

## 5b. V1 digital twin (once, offline)
`analysis/preprocess_scan.py V1` → `dataset/derived/euroc_twin/V1_leica_v30mm.{ply,json}` (git-ignored; json records every parameter).
| Step | Value |
|---|---|
| Source | `pointcloud0/data.ply` (V1, md5 26a6853…), 3 199 068 points, read only |
| Crop | per axis [p0.1 − 0.2 m, p99.9 + 0.2 m] → box x [−4.58, 3.89], y [−3.68, 5.17], z [−0.23, 4.31] m; **380 points removed** (the far stray point and points beyond walls/openings) |
| Downsample | 3 cm voxels → **402 142 points** |
| Normals | Open3D hybrid KD-tree, r = 0.10 m, max 30 nn, unoriented; floor median \|n_z\| = 0.99993 |
| Output md5 | 8fc20009837a56090259b9c5da31724c (M2 and M3 must load this file) |
Figures: `docs/img/phase5_V1_twin_gt_overlay.png` (GT frame check only), `docs/img/phase5_V1_twin_normals.png`.

## 5c. Registration node copy + EuRoC config
- `svo_registration/scripts/online_registration_fyp.py`: parameters `twin_voxel_size`, `twin_min_z`, `fitness_threshold` (defaults = original 1.0 m, 0.0 m, 0.75); fixes F1 (arccos clip) and F2 (zero-norm guard); optional per-KF CSV log with counts, ICP stats, reject reason and timings (preprocessing / ICP / information matrix). **Equivalence check:** with Oerlikon config and defaults it matches the original on 115 recorded KFs (same verdict 115/115, |ΔT| ≤ 3e-14, |ΔInfo| ≤ 1.5e-12).
- `svo_registration/config/euroc_v1.yaml` (every value commented in the file): correspondence 0.5 m (M1 map-frame error max 0.44–0.46 m), inlier RMSE ≤ 0.15 m, fitness ≥ 0.75, correction ≤ 0.75 m / 5°, bounding box 10 m (no crop), no twin re-downsampling, no z filter, information weights as Oerlikon (9, 2, power 1).

## 5d. Online D1 alignment
`svo_ros` (flag `d1_alignment`, default off): the same frame-status event as the frame-event log, the same 10 ms GT bracket and linear/slerp interpolation, VIO pose = the `/svo/vio_backend_pose_imu` sample at t₀ (internal subscriber). Sets `T_w_vio` (backend, publisher, both handlers) and `gp_initialized_`, broadcasts static TF `world → vio`, writes `d1_alignment.csv`.
**Numerical check on V1_01 M2 r01 vs `analysis/initial_alignment.py` on the same run:** t₀ identical (1403715279062143000 ns), yaw 123.242279151° vs 123.242279189° (Δ 3.8e-8°), max |ΔT_M_W| = 5.5e-10 (TUM 9-decimal rounding of the offline input). **Match.**

## 5e. First M2 run on V1_01 (single run, not repeated)
Run `runs/2026-10-23_V1_01_easy_M2_r01` (commit db59773, libsvo_ros md5 fae2d64c5024), launch `euroc_m2.launch`, frame-event log on.

| | M1 (5 runs) | **M2 r01** |
|---|---|---|
| Map-frame ATE, no alignment (t₀ → end) | 0.237 ± 0.009 m | **0.216 m** (median 0.202, max 0.399) |
| ATE 4-DoF aligned | 0.056 ± 0.003 m | **0.119 m** |
| Rel. displacement error @ 5.9 / 11.7 / 17.6 / 23.4 / 29.3 m | 0.064 / 0.062 / 0.071 / 0.075 / 0.075 | **0.095 / 0.127 / 0.145 / 0.150 / 0.160** |
| Frames / frontend / latency / CPU | 2912/2912, 7.5 ms, 51 ms, 73 % | 2912/2912, 7.6 ms, 52 ms, 73 % |

Registration (190 KF clouds): **75 accepted, 115 rejected (114 rotation > 5°, 1 translation+rotation)**; 74 added to the backend (the first registration is discarded by the original `readInitialPose` logic; rosout and svo.log agree). Landmarks per KF median 478 raw / 348 after the node's filters. Fitness median 1.00, inlier RMSE 0.060 m, correction 0.168 m / **5.2°** (median). Runtime medians: preprocessing 89 ms, ICP 175 ms, information matrix 33 ms.

### Finding: the D1 yaw is ~4–5° off on V1_01 (GT orientation at t₀)
- The accepted ICP corrections are a **consistent yaw of +4.1° ± 0.5°** about the map z-axis (roll/pitch ~1°), in every 30 s window. The correction is applied about the map origin; with the body ~2.5 m from it, the 4° yaw also gives the ~0.17 m translation corrections.
- Independently, **GT positions** prefer the same direction: re-aligning the five M1 runs with ψ_D1 + δ gives map-frame ATE 0.237 (δ=0), 0.139 (+2.25°), 0.080 (+4°), minimum **0.072 m at δ = +4.75°** (evaluation-only diagnostic).
- Raw-Vicon orientation at t₀ gives δ = +2.22° (Phase 4 sensitivity).
- Reading: the scan frame and the GT positions agree with each other; the **GT-estimate orientation at the single instant t₀** carries a ~4–5° yaw error on V1_01 (consistent with the Phase 4 GT-orientation finding). D1 inherits it. It dominates M1's map-frame ATE (0.237 m vs ~0.07 m with the yaw removed), and in M2 the registration repeatedly asks for the missing yaw against a fixed T_w_vio: the 5° limit rejects 60 % of results, the accepted ones pull the VIO, and local consistency degrades (4-DoF ATE 0.056 → 0.119 m).
- **Visual check:** `docs/img/phase5_yaw_check.png` (`analysis/phase5_yaw_check.py`, visualisation only). Wall slice z 0.5–2.5 m of the twin with the M2 r01 map-frame landmarks: with D1 the south- and east-wall landmark bands are visibly rotated against the scan walls; rotated by +4.1° they run parallel. Median landmark-to-twin distance 11.2 cm (D1) → 8.2 cm (+4.1°). The west wall has few landmarks (rarely in view), so the zooms show the south and north walls. Caveat: the M2 r01 landmarks already include the effect of the accepted corrections.
- **Decision (2026-10-23): D1(B)**, an alignment from GT positions over the first 2 m of flight (docs/frames.md). Window length chosen on M1 runs by yaw stability (plateau L = 1–2 m), before looking at any M2/M3 result. M1 recomputed: map-frame ATE 0.132 ± 0.012 m (legacy 0.237 ± 0.009).

## RViz check (manual)
Terminal 1 (no venv):
```
source /opt/ros/noetic/setup.zsh && source ~/Desktop/twinforge/catkin_ws/devel/setup.zsh
roslaunch svo_ros euroc_m2.launch rviz:=true d1_gt_file:=$HOME/Desktop/twinforge/rpg_svo_pro_with_digital_twins/dataset/derived/asl_meta/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv
```
Terminal 2, after RViz is up: `rosbag play ~/Desktop/twinforge/rpg_svo_pro_with_digital_twins/dataset/vicon_room1/V1_01_easy/V1_01_easy.bag`

In RViz set **Fixed Frame = `world`**. The `world` frame exists only after the D1 alignment fires (~6 s into the bag; static TF `world→vio`); before that the Grid shows an error and nothing is drawn. This is expected. After ~6 s:
- `/city_twin` (grey): the room, with walls, floor and ceiling at ~4 m. A top-down view is easiest.
- `/svo/w_backend_points` (SVO landmarks in the map frame): should lie on or near the scan walls and floor. **What to look for:** landmarks on a wall forming a band rotated by a few degrees relative to the scan wall (larger offset at the room edges than near the centre); that would be the ~4° yaw seen in the logs.
- `/svo/w_backend_points_registered` (only accepted KFs, ~1 in 2.5): should lie on the scan surfaces.
- `/svo/w_backend_pose_imu_viz`: the drone pose inside the room, 1–2 m above the floor.
- `vio`-frame displays (`/svo/vio_backend_points`, `/svo/pose_imu`) now also render, through the static TF.

## Fixes after the user's RViz check (2026-10-23)
- **TF_REPEATED_DATA** (RViz, once, `vio (parent world)` at the D1 stamp; `svo_node` logged a single send): the static broadcaster was created at alignment time, so RViz connected to a brand-new latched publisher while the message was being sent and received it twice. The broadcaster is now created at start-up, and `sendTransform` is called exactly once. Verification: a `tf_echo world vio` listener running from before the alignment, in the next run.
- `euroc_m2.launch`: `d1_gt_file` now defaults to the V1_01_easy GT file (full path via the package); no argument is needed for V1_01.
