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
| 8 — C++ module | New file/node, same output interface, Ceres, unit tests vs Python. | Tests pass. |
| 9 — Online integration M3 | registration_method config flag, all six Vicon sequences × 5 runs. | Runs complete. |
| 10 — Evaluation and ablations | Main table M1/M2/M3, voxel/truncation ablations, reproducible figure scripts, H1–H4 status. | Results frozen. |
| 11 — Write-up | Report sections, poster content (drafts). | — |

## Design decisions recorded during execution
- **Main comparison (D4):** M3 publishes information in the same heuristic convention as M2 (normalised ∇D used as the normal). **Ablation:** Hessian-based covariance for both M2 and M3 → 2×2 design {M2, M3} × {heuristic, Hessian}.
- **Initial alignment (D1):** one-time 4-DoF GT alignment at the first keyframe after backend initialisation (see `docs/frames.md`). `refine_T_WL: false` in the main runs; refinement is an ablation.
- **Logging additions (Phases 5, 6, 9):** per keyframe, (i) registration result published vs accepted by the backend (the 1 ms KF stamp match), (ii) registration runtime, (iii) local-cloud point count before/after filtering, (iv) accept/reject with reason.
- **Phase 10 experiment: initial alignment robustness (supports H1).** Inject yaw offsets {0, 2, 5, 10}° into the D1(B) alignment (T_M_W ← R_z(δ)·T_M_W) and run M2 and M3 with `refine_T_WL` off and on (2 × 2 × 4 conditions). Report registration acceptance, ICP correction size, map-frame ATE and whether/when the yaw error is corrected.
- **D1 amended to D1(B) (2026-10-23):** alignment from GT positions over the first L = 2 m of flight; GT orientation not used (docs/frames.md).
