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
