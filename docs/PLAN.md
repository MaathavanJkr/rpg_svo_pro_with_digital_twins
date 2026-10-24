# Phase plan

Each phase ends at a GATE. Do not start the next phase until the user approves.

| Phase | Work | GATE |
|---|---|---|
| 0 — Housekeeping | Branch, .gitignore, runs/ folder and RUNLOG.md, docs/ folder. | Supervisor scope confirmed by the user. |
| 1 — Dataset verification | Formats (bag/ASL), ground truth, Leica pointcloud0, scan frame (sensor.yaml), Oerlikon files vs config paths; ASL→rosbag conversion if needed; docs/datasets.md + scan screenshots. | User approves the data table. |
| 2 — Repository inspection | Launch/configs, svo_registration (point-to-plane functions, twin loader, local cloud formation, covariance), global-measurement interface in svo_ros / svo_ceres_backend, W→M initial alignment, svo_benchmarking; docs/pipeline_notes.md + docs/frames.md. | User understands and approves the notes. |
| 3 — Reproduce Oerlikon | Run Oerlikon.launch; save trajectory and plots. | User confirms alignment in RViz. |
| 4 — VIO baseline M1 | SVO Pro EuRoC launch on MH_01_easy (sanity) and V1_01_easy, 5 runs, evaluate with evo (ATE aligned, RPE, drift plots). | Baseline accepted. |
| 5 — Port to EuRoC (M2) | euroc_dt.launch + config copied from Oerlikon, Leica scan preprocessing, documented initial alignment, ATE aligned AND map-frame unaligned. | User approves the alignment method and confirms frames in RViz. |
| 6 — Logging | Per registration event save the local cloud, initial guess, point-to-plane result + covariance, VIO pose, GT pose, runtime, point count. | Logs reload in Python. |
| 7 — Offline distance-field prototype (Python) | Voxel grids (unsigned/TSDF, v = 0.05/0.10/0.20 m), SE(3) GN/LM with trilinear interpolation and robust kernel, covariance from Hessian; accuracy, convergence-basin, runtime, and memory studies vs point-to-plane on identical inputs; docs/offline_results.md. | Go/no-go on C++. |
| 8 — C++ module | New file/node, same output interface, unit tests vs Python (solver: Eigen LM port of the prototype instead of Ceres, see decisions below). | Tests pass. |
| 9 — Online integration M3 | registration_method config flag, all six Vicon sequences × 5 runs. | Runs complete. |
| 10 — Evaluation and ablations | Main table M1/M2/M3, voxel/truncation ablations, reproducible figure scripts, H1–H4 status. | Results frozen. |
| 11 — Write-up | Report sections, poster content (drafts). | — |

## Design decisions recorded during execution
- **Main comparison (D4):** M3 publishes information in the same heuristic convention as M2 (normalised ∇D used as the normal). **Ablation:** Hessian-based covariance for both M2 and M3 → 2×2 design {M2, M3} × {heuristic, Hessian}.
- **Initial alignment (D1):** one-time 4-DoF GT alignment at the first keyframe after backend initialisation (see `docs/frames.md`). `refine_T_WL: false` in the main runs; refinement is an ablation.
- **Logging additions (Phases 5, 6, 9):** per keyframe, (i) registration result published vs accepted by the backend (the 1 ms KF stamp match), (ii) registration runtime, (iii) local-cloud point count before/after filtering, (iv) accept/reject with reason.
- **Phase 10 experiment: initial alignment robustness (supports H1).** Inject yaw offsets {0, 2, 5, 10}° into the D1(B) alignment (T_M_W ← R_z(δ)·T_M_W) and run M2 and M3 with `refine_T_WL` off and on (2 × 2 × 4 conditions). Report registration acceptance, ICP correction size, map-frame ATE and whether/when the yaw error is corrected.
- **D1 amended to D1(B) (2026-10-23):** alignment from GT positions over the first L = 2 m of flight; GT orientation not used (docs/frames.md).
- **Statistics (decision 2026-10-23):** all key M1/M2/M3 comparisons report mean ± std (sample std) and Welch's t-test p-values (`analysis/compare_methods.py`).
- **Phase 8 implementation (decided 2026-10-24):** C++ library + pybind11 module `fyp_df` (svo_registration/df_cpp), called from `online_registration_m3.py`, a copy of the M2 node that differs only in the registration step (`registration_method: point_to_plane | distance_field`). The solver is a line-by-line port of the Phase 7 prototype (Eigen LM with the prototype's damping and stopping criteria, Huber IRLS), **not Ceres** as first planned: Ceres' trust-region strategy would not reproduce the prototype's iterates, and the Phase 8 acceptance test is equality with the prototype. Built standalone with plain `-O3` (no `-ffast-math`, unlike SVO). Configuration: unsigned, untruncated, v = 0.05 m, Huber δ = 0.05, 100-iteration cap; information = M2 heuristic with normalised ∇D (main), Hessian (D4 ablation option).
- **H2 timing protocol (decided 2026-10-24, Phase 8):** offline, one process pinned to one core, `OMP_NUM_THREADS=1`, same inputs (the 922 logged V1_01 keyframes), same machine, methods interleaved per keyframe. (a) Core solve: Open3D `registration_icp` as configured in M2 (includes its internal KD-tree build of the target; the build is also timed alone) vs the C++ DF call. (b) End-to-end per-KF node callback for both node paths. Medians and p90; memory = RSS increase from loading the grid vs the twin + KD-tree. Online CPU in Phase 9 is reported separately (Open3D is multi-threaded there).
- **Future work (recorded 2026-10-24): map-based re-alignment after an SVO reset (option (c)).** After a full SVO reset (`resetBackend`) the VIO frame's heading is re-initialised and the one-shot D1 alignment is lost, so map-frame output stops (V1_03, docs/phase6_logging.md). A re-alignment from the registration itself (e.g. re-running the 4-DoF alignment on registered positions after a reset, similar to the original `refine_T_WL` path) would remove this limitation without using GT again. Not implemented; SVO settings are not changed (option (d) rejected).

## Sequences and reset rule (decided 2026-10-24, before Phase 9)
- **Main table (M1/M2/M3):** V1_01_easy, V1_02_medium, V2_02_medium, V2_03_difficult.
- **Excluded:** V1_03_difficult (SVO full resets at ~12.3 s in every run and mid/late-flight resets in 4/5; reported as a limitation, docs/sequence_survey.md and docs/phase6_logging.md).
- **Deferred:** V2_01_easy (early reset at 6.9 s; would need a re-alignment hook after resets within the first 15 s). **Optional, only if time allows after Phase 9.**
- **Pre-registered reset rule (applies to every run of the included sequences, all methods):** a run in which SVO performs a full reset (stage Relocalization → Paused, i.e. `resetBackend`) is a **failure for that method**. Failure counts are reported per method and sequence; failed runs are **excluded from ATE / RPE / drift statistics**; **no re-running until a run passes** (the planned number of runs per method and sequence is executed once; failures are not replaced). Detection: the frame-event log (`log_frame_events:=true`, always on in Phase 9), `analysis/reset_survey.py`.
