# Phase 3: Oerlikon reproduction (original pipeline, M2)

Run `runs/2026-10-23_oerlikon_M2_r01` (one run). Launch: `svo_ros/launch/fyp/oerlikon_m2.launch` (copy of `Oerlikon.launch` + `python3` prefix + optional RViz), `reference_map:=dataset/Oerlikon.ply`, `rviz:=false`.
Scripts: `analysis/run_oerlikon_m2.sh`, `analysis/phase3_oerlikon_eval.py`. evo from `~/venvs/fyp`.

## Clock choice
- The bag's header stamps start at 135.97 s (simulator clock). The launch sets `/use_sim_time=false`. **Kept as upstream**: the bag is played without `--clock`, at rate 1.
- Rationale (code check): wall-clock `Time::now()` appears only in visualisation stamps (`svo_ros/src/visualizer.cpp`, `backend_visualizer.cpp`, and the `/city_twin` cloud in `online_registration.py:39`). Everything that matters (IMU/image processing, keyframe stamps, registration message stamps, the backend's 1 ms KF match, published poses) uses message header stamps.
- Consequence: the real-time behaviour equals the original setup, since registration has wall-clock time to finish before its KF leaves the window.

## Results (single run; no repeat statistics yet)
| Quantity | Value |
|---|---|
| Map-frame position error vs `/groundtruth`, **no alignment** (evo APE) | RMSE **2.19 m**, median 1.36 m, max 6.22 m (over 349 s of map-frame output) |
| Same, SE(3)-aligned (for reference only) | RMSE 1.73 m |
| `/gps` in the bag vs GT | RMSE 39 m, growing from ~5 m to ~50 m. Property of the provided GPS; weight is low (variance 900 m²) |
| GPS alignment (T_w_vio) | at t₀+51 s |
| Registration refinement of T_w_vio (`refine_T_WL`) | at t₀+81 s; error drops from ~5 m to ~1 m (visible jump in the top view near (−5, −10)) |
| Keyframe clouds sent to registration | 344 |
| **Landmarks per KF (mono)** | raw median 840 (661–960); after node filters median 759 (563–916). **Mono landmarks do reach the registration.** |
| Registration accepted / rejected | 231 / 113. Reasons: rmse 60, rmse+rotation 14, rmse+translation+rotation 25, rotation 11, translation+rotation 3. **Fitness ≥ 0.97 always**, so the 0.75 fitness gate never binds |
| Dropped online (queue_size=1) | 0 (offline replay with the original node's methods reproduces the online accepted set exactly: 231/231, 0 mismatches) |
| Published registrations → use | 32 before refinement (1 discarded + alignment-only pairs); 199 after, of which **152 added** as backend residuals; **47 (24 %) not added** (most likely the KF left the window / failed the 1 ms stamp match; to be confirmed by Phase 5/6 logging) |
| Registration runtime (offline replay, same code, this PC) | median 0.19 s, p90 0.43 s per KF |

Figures: `docs/img/phase3_oerlikon_traj_top.png`, `docs/img/phase3_oerlikon_errors_points_fitness.png`.

## Caveats
- One run only. Runtime numbers are from the offline replay, not measured inside the live node.
- I have not compared these numbers with the paper's reported Oerlikon numbers; the figures/tables there should be checked before claiming "reproduced within X".
- Publication gaps at ~130–170 s, ~240–285 s and ~350–400 s coincide with rejections (rmse/rotation), not with point-count drops.
