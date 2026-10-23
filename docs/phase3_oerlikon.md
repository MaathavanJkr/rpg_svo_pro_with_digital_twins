# Phase 3: Oerlikon reproduction (original pipeline, M2)

Runs `runs/2026-10-23_oerlikon_M2_r01`, `_r02`, `_r03`. Launch: `svo_ros/launch/fyp/oerlikon_m2.launch` (copy of `Oerlikon.launch` + `python3` prefix + optional RViz), `reference_map:=dataset/Oerlikon.ply`, `rviz:=false`.
Scripts: `analysis/run_oerlikon_m2.sh`, `analysis/phase3_oerlikon_eval.py`. evo from `~/venvs/fyp`.

## Clock choice
- The bag's header stamps start at 135.97 s (simulator clock). The launch sets `/use_sim_time=false`. **Kept as upstream**: the bag is played without `--clock`, at rate 1.
- Rationale (code check): wall-clock `Time::now()` appears only in visualisation stamps (`svo_ros/src/visualizer.cpp`, `backend_visualizer.cpp`, and the `/city_twin` cloud in `online_registration.py:39`). Everything that matters (IMU/image processing, keyframe stamps, registration message stamps, the backend's 1 ms KF match, published poses) uses message header stamps.
- Consequence: the real-time behaviour equals the original setup, since registration has wall-clock time to finish before its KF leaves the window.

## Results: 3 runs (r01–r03), map frame, no alignment
Script: `analysis/phase3_aggregate.py` → `runs/phase3_oerlikon_aggregate.json`. ATE_P = translation RMSE, ATE_R = rotation-angle RMSE, on `/svo/w_backend_pose_imu` (T_M_B as published by the system) vs `/groundtruth` (frame `world`), nearest GT within 0.05 s. Cross-check: evo APE on r01 gives 2.19 m vs 2.21 m here.

| Run | Refinement at | ATE_P whole [m] | ATE_R whole [°] | ATE_P post-refinement [m] | ATE_R post-refinement [°] | Accepted / rejected | Backend residuals added |
|---|---|---|---|---|---|---|---|
| r01 | t₀+80.8 s | 2.21 | 3.98 | 1.80 | 3.89 | 231 / 113 | 152 |
| r02 | t₀+82.0 s | 1.95 | 3.83 | 1.38 | 3.69 | 233 / 108 | 200 |
| r03 | t₀+82.0 s | 1.99 | 3.86 | 1.43 | 3.79 | 230 / 113 | 152 |
| **mean ± std** | | **2.05 ± 0.14** | **3.89 ± 0.08** | **1.54 ± 0.23** | **3.79 ± 0.10** | | |

Windows: whole = 349 s (from GPS alignment at ~t₀+51 s to the end); post-refinement ≈ 318 s. Run-to-run spread (max − min): 0.26 m (whole), 0.42 m (post-refinement), 0.15° / 0.20° rotation.

Other quantities (r01 detail; r02/r03 similar, see each run's `analysis/summary.json`):
| Quantity | Value |
|---|---|
| Landmarks per KF (mono) | median 829–840 raw (648–1000); r01 after node filters median 759. **Mono landmarks reach the registration.** |
| Reject reasons (r01) | rmse 60, rmse+rotation 14, rmse+translation+rotation 25, rotation 11, translation+rotation 3. **Fitness ≥ 0.97 always**: the 0.75 gate never binds (see note 5a). |
| Dropped online (queue_size=1) | 0 in all runs: the offline replay of the original node's methods reproduces each online accepted set exactly |
| Published → backend (r01) | 32 before refinement (1 discarded + alignment-only); 199 after, 152 added, **47 (24 %) not added**. r02 added 200, r03 152. |
| Registration runtime (offline replay, r01) | median 0.19 s, p90 0.43 s per KF (preprocessing + ICP together) |
| `/gps` vs GT | RMSE 39 m; see GPS section |

Figures (r01): `docs/img/phase3_oerlikon_traj_top.png`, `docs/img/phase3_oerlikon_errors_points_fitness.png`.

### Equivalence check: new binary (frame-event logger built in, flag off)
`svo_ros` was rebuilt on 2026-10-23 13:15 with the log-only frame-event logger (`libsvo_ros.so` md5 `3a339b876666`). r01–r03 used the 2026-10-22 binary. One Oerlikon run on the new binary with `log_frame_events` off (`runs/2026-10-23_oerlikon_M2_newbin_r01`):

| | ATE_P whole [m] | ATE_P post [m] | ATE_R whole [°] | ATE_R post [°] | Accepted / rejected | Backend added |
|---|---|---|---|---|---|---|
| r01–r03 (old binary), range | 1.95–2.21 | 1.38–1.80 | 3.83–3.98 | 3.69–3.89 | 230–233 / 108–113 | 152–200 |
| new binary | 1.98 ✓ | 1.38 ✓ (at edge) | **3.65** ✗ | **3.49** ✗ | 239 / 100 | 200 |

**Result:** position within the old-binary spread. Rotation is 0.18–0.20° *lower* (better) than all three old-binary runs, so it is **not** strictly within the r01–r03 range. With the flag off the logger returns on its first line, so a behavioural change is not expected, but n=3 is too few to define the run-to-run spread and this check does not prove equivalence. It can be strengthened with more runs of both binaries if needed.

### Equivalence check 2: current build with F3 (flag fix), logger off
`runs/2026-10-23_oerlikon_M2_F3_r01` (commit 3688064, libsvo_ros md5 `a8df1f97d777`).

| | ATE_P whole [m] | ATE_P post [m] | ATE_R whole [°] | ATE_R post [°] | Accepted / rejected | Backend added |
|---|---|---|---|---|---|---|
| r01–r03 (old binary): mean ± std / range | 2.05 ± 0.14 / 1.95–2.21 | 1.54 ± 0.23 / 1.38–1.80 | 3.89 ± 0.08 / 3.83–3.98 | 3.79 ± 0.10 / 3.69–3.89 | 230–233 / 108–113 | 152, 200, 152 |
| logger build (newbin_r01) | 1.98 | 1.38 | 3.65 | 3.49 | 239 / 100 | 200 |
| **F3 build (F3_r01)** | **1.83** ✗ | **1.27** ✗ | **3.65** ✗ | **3.55** ✗ | 235 / 107 | 200 |

**Result: not within the r01–r03 spread.** The F3-build run is *better* on every metric (whole-window ATE_P 1.6σ below the old mean). Both new-build runs are better than all three old-binary runs in rotation. With both flags true, F3 assigns the same values as before and the logger returns immediately, so no code-path change is expected. The cause is not established. Observations: (a) error appears to track how many registrations reach the backend, which is timing-dependent (old: 152/200/152 with r02 = 200 the best old run; both new runs 200); (b) the old binary was overwritten by the failed link and cannot be rerun without rebuilding commit `d641322`. A proper A/B (≥ 5 runs per build, same session) would settle it; decision requested.

## Comparison with the paper (Merat et al., RA-L 2025, arXiv:2412.08496)
**Verdict: the pipeline runs end to end, and the results are of the same order as the paper but not strictly comparable.** Not called "reproduced".

Paper, real-world experiment (Table IV, SVO-Digital Twin): **ATE_P 1.09 m, ATE_R 3.40°**. Ours: 2.05 ± 0.14 m / 3.89 ± 0.08° (whole window), 1.54 ± 0.23 m / 3.79 ± 0.10° (post-refinement).

Why this is not like-for-like:
1. **Same flight: probable, not confirmed.** The paper never names "Oerlikon", and the download page has no README. For: the data-prep scripts in `svo_registration/Dataset/` reference `PaperData/RW/Dataset/Flight2→3.bag`; the bag is a real Elios 3 flight (`camera_2`, `/vio_module/imu_filtered`, GPX GPS, COLMAP ground truth) with loops of increasing altitude (3.6–16 m GT). Against: the paper says "circa 5 minutes" and ">500 m", while the bag has 399.5 s and 443.7 m of GT path, and the paper says altitude 0–17 m.
2. **The initial alignment is driven by GPS with hand-set offsets.** The `/gps` stream was built from the drone's GPX log with a hand-chosen linearisation altitude (`390+30-28` m; a commented-out alternative reads `425-80+470`), an x shift of −26 m, and a time offset marked "approximate guess" (`5_GPS_reading.py`). The GPS alignment at ~t₀+51 s, and therefore the first ~30 s of map-frame output (~5 m error), depends on these choices. Whether the paper's run used the same GPS processing is unknown.
3. **The evaluation window is unstated.** The paper defines ATE_P/ATE_R as RMSE in the world frame using the system's own T_WL (the same convention we use), but does not say whether the window starts at GPS alignment, at refinement, or elsewhere. Our two windows differ by 0.5 m, which is a large fraction of the paper-vs-ours gap.
4. Further differences: one paper number vs our 3-run mean; unknown hardware and real-time load (registration latency affects how many results reach the backend; we saw 152–200 added).

## GPS in the bag: intentionally degraded?
- **No statement anywhere that it is intentionally degraded.** The paper describes simulated GPS (ray tracing + GMM multipath, 5 Hz) for its simulation experiment only, and says nothing about the real-flight GPS. The download page has no description.
- **Source (from repo scripts):** real Elios 3 GPX log converted to ENU in `svo_registration/Dataset/5_GPS_reading.py`, with the hand-set offsets above. 1 Hz in the bag.
- **Observed error vs GT:** horizontal median 5.3 m (0–50 s) → 10–15 m mid-flight → 7.4 m at the end. Vertical median 0.5 m → 11 m → 31 m → **49 m**. The growing error is **mostly vertical**, consistent with a drifting GPS/barometric altitude or an altitude-reference choice in the conversion, not with an added synthetic degradation. Cause not confirmed.
- Its effect on the estimate is limited: `gp_settings.yaml` variance 900 m² (σ = 30 m).

## Notes for Phase 5/6 (from these runs)
- **(5a) Correspondence distance.** At `max_correspondence_distance: 8 m`, ICP fitness (inlier fraction within 8 m) is ≥ 0.97 for every KF and carries no information. The EuRoC config needs a room-scale distance (order 0.1–0.5 m, to be chosen in Phase 5), and fitness should then be re-checked as a gate.
- **(5b) Accepted-but-not-added diagnosis (24 % in r01; 152–200 added across runs).** Log per registration: KF stamp, the backend's window KF stamps at arrival, |Δt| to the nearest KF, and whether it was added. This separates (i) KF already marginalised (window exit), (ii) stamp mismatch > 1 ms (precision), and (iii) other causes (e.g. one-per-KF limit, `readInitialPose` discard, alignment-only phase).
- **(5c) Timing breakdown.** Log ICP solve time separately from preprocessing (crop, outlier removal, voxel downsampling, map crop) and from information-matrix computation, per KF.

## RViz visual check (manual)
RViz's Fixed Frame must exist in the TF tree. Nothing in the pipeline broadcasts a frame named `world`; `svo_node` only broadcasts TF in `vio`, and only while images are processed. So with Fixed Frame `world`, the Grid reports an error and nothing in `world` is drawn (observed by the user on 2026-10-23). Workaround for visualisation only:
```
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world fyp_world_anchor
```
This creates the `world` frame via a dummy child. It asserts **no** relation between `world` (M) and `vio` (W). Displays in `vio` still show "No transform" errors. The proper `world→vio` static TF comes with the D1 alignment in Phase 5 (EuRoC).
