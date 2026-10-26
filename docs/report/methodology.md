# Chapter 3: Methodology (draft, Phases 1–8)

*Draft generated from the project documents in `docs/` (2026-10-26). Results of the online experiments (Phases 9–10) belong in the results chapter and are referred to here only where they shaped a method decision. Source documents are cited in brackets for checking.*

## 3.1 Research design

The study compares three localisation methods on the same data, calibration, initial alignment and backend parameters:

- **M1, VIO only.** Monocular-inertial SVO Pro with its Ceres sliding-window backend, no map registration.
- **M2, point-to-plane digital-twin registration.** The original method of Merat et al. (2025): the sparse VIO landmark cloud of each keyframe is registered to the digital twin with point-to-plane ICP, and the result is fused into the backend as an absolute pose prior.
- **M3, distance-field digital-twin registration (this work).** The registration step of M2 is replaced by an alignment of the same cloud to a precomputed voxel distance field of the twin. Everything else is unchanged.

M2 and M3 differ only in the registration step; M1 shows whether digital-twin registration helps at all. The work was staged so that each step could be checked before the next one depended on it: reproduction of the original system on its own data (Phase 3), a VIO baseline (Phase 4), the port of the original registration to an indoor benchmark (Phase 5), instrumentation (Phase 6), an offline prototype of the distance-field registration on logged inputs (Phase 7), its C++ implementation and verification (Phase 8), and online experiments (Phases 9–10). Every phase ended at a review gate. [PLAN.md]

Three working rules shaped the method throughout. First, the original registration code was never modified; new behaviour was added in copies of files and selected by configuration. Second, ground truth was used only for evaluation and for one documented initial alignment, never to build the map or to tune a method. Third, every online experiment was pre-registered in writing (design, sample size, primary test, failure rule) before it was run, on a frozen and checksummed build; any later deviation was recorded with a timestamp before it was acted on (§3.12).

## 3.2 The system under study

SVO Pro runs a semi-direct visual front end and a Ceres-based sliding-window visual-inertial backend. In the digital-twin extension (Merat et al., 2025), every keyframe triggers the publication of the keyframe pose and all landmarks currently in the optimisation window, transformed into the map frame with the current VIO-to-map transform T_M_W. A separate Python node (`online_registration.py`) crops and filters this cloud, registers it to the twin, checks the result and publishes a registered pose with a 6×6 weighting matrix. The backend adds this pose as a `PoseError` prior on the keyframe with the matching timestamp (±1 ms), with a Huber loss (δ = 50). [pipeline_notes.md §1, §3]

Three properties of the original implementation, found by reading the code before any experiment, matter for the method:

1. **The weighting matrix is heuristic.** It is the sum over correspondences of HᵢᵀHᵢ with Hᵢ = [−(aᵢ×nᵢ)/‖aᵢ×nᵢ‖, −nᵢ], normalised by its largest entry, multiplied by exp(−0.9·RMSE) and scaled by fixed factors (9² for position, 2² for rotation). It is published in the message's `covariance` field but used by the backend as an *information* matrix. Its rotational rows are unit vectors about the map origin, so the lever arm is lost and the absolute scale comes from the two factors, not from the data. [pipeline_notes.md §2]
2. **Map registration is only active after an initial alignment.** In the original system T_M_W comes from a 4-DoF GPS alignment, optionally refined once from registered poses (`refine_T_WL`). Registered poses are ignored until this alignment exists. [pipeline_notes.md §3–4]
3. **The correction is expressed in the map frame.** ICP is initialised at the identity and estimates a correction T_c applied on the left in the map frame (rotation about the map origin), and the registered pose is T_c · T_M_W · T_W_B. [frames.md]

Coordinate frames are kept explicit throughout: body/IMU (B), camera (C0), VIO world (W), map/twin (M), Vicon ground truth (V) and Leica scan (R). The code's names differ from this notation (`world` is M, `vio` is W, `T_w_vio` is T_M_W); every transform and its source is listed in `docs/frames.md`.

## 3.3 Datasets and digital twin

**EuRoC MAV.** The Vicon-room sequences of the EuRoC MAV dataset (Burri et al., 2016) provide stereo images, IMU, ground truth and, uniquely, a Leica laser scan of each room. The official ROS bags were used directly (no conversion); monocular-inertial operation uses cam0 and `/imu0` (200 Hz). Ground truth is `state_groundtruth_estimate0`, which gives the body (IMU) frame in the Vicon frame; it starts about 1 s after the bag. V2_03 drops 17.7 % of cam0 frames (single-frame gaps), which is reported with its results. The Machine Hall sequences have no scan and were used only for a VIO sanity check. [datasets.md §1]

**Digital twin.** The scans (`pointcloud0`, Leica Nova MS50 from six stations, 3.20 M points in room 1 and 3.12 M in room 2, ~5 mm spacing, no normals) are given in the dataset's world frame with T_V_R = I, so the map frame equals the Vicon frame and map-frame errors can be computed directly against ground truth without any trajectory alignment (assumption A2). A headless frame check confirmed that all ground-truth positions lie inside the scanned room with no visible rotation or offset. The scan is an independent laser measurement; its registration to the Vicon frame is the dataset authors' calibration and is an error floor this study does not control. [datasets.md §2; report_notes.md A2]

Each scan was preprocessed once, offline, with a fixed recorded procedure (`analysis/preprocess_scan.py`): per-axis crop to [p0.1 − 0.2 m, p99.9 + 0.2 m] (removing stray points beyond walls and openings: 380 in room 1, 188 in room 2), 3 cm voxel downsampling (402 142 and 402 036 points) and normal estimation (radius 0.10 m, ≤ 30 neighbours). The output files are identified by their MD5 checksum, which every later component verifies, so M2 and M3 always use exactly the same map (assumption A3). The original files are never modified. [phase5_m2_euroc.md §5b]

**Sequences.** The main experiments use V1_01, V1_02, V2_02 and V2_03. A survey of SVO full resets (where SVO re-initialises and the VIO heading is lost) led to the exclusion of V1_03 (resets in every run) and to V2_01 being deferred; V2_03 was later also reported descriptively only because most of its runs reset (results chapter). [sequence_survey.md; PLAN.md]

**Oerlikon.** The original authors' urban data (a simulated flight over a city-scale twin of Zürich Oerlikon, 399 s) was used only to reproduce the original pipeline (§3.5).

## 3.4 Software environment and reproducibility

All work is on a separate branch of the author's fork; the upstream code is unchanged except for documented fixes (below). The catkin workspace is built with a script that removes a conflicting user-space toolchain from the path (`scripts/build.sh`). Analysis runs in a dedicated Python virtual environment (evo 1.31.1, Open3D 0.19.0); ROS nodes use the system Python. [environment.md]

Upstream defects found and fixed, each in a new or clearly marked code path and verified not to change the original behaviour:

- **F1/F2 (registration node copy):** the rotation check used `arccos` without clipping (NaN passed the check), and a degenerate point–normal pair could divide by zero. Fixed only in the parameterised copy `online_registration_fyp.py`; with the Oerlikon configuration the copy reproduces the original node's outputs on 115 keyframes. [phase5_m2_euroc.md §5c]
- **F3 (svo_ros):** the backend's global-measurement flags were never set to false, so VIO-only operation crashed on the first frame. Fixed in `svo_interface.cpp`. With both flags true (the original configuration) the fix assigns the same values as before. A single Oerlikon run on the fixed build was better than the three earlier runs on every metric (cause not established); the official five-run Oerlikon baseline on the final build was not distinguishable from the earlier runs (Welch p = 0.37). [phase4_m1.md; phase3_oerlikon.md]
- **Build flags:** SVO's release flags (`-ffast-math`, `-fno-signed-zeros`) break Eigen 3.3.7's SSE double 4×4 inverse. All other Eigen operations used by SVO were verified identical to an unoptimised build, the estimation code does not use the affected inverse, and the flags were kept to preserve the baseline; new C++ code avoids that inverse. [pipeline_notes.md; phase6_logging.md step 1]

Every online run is executed headless by a harness (`analysis/run_euroc.sh`) that checks the launch file before starting, aborts if a node does not come up, records the bag, CPU use (1 Hz) and node logs, copies the exact launch and configuration files into the run folder, and records the git commit and checksums of the binaries and scripts. Runs are played at rate 1.0 on an otherwise idle machine. Every run is listed in `runs/RUNLOG.md`. Runs in which an external process interfered are moved to a separate folder with the reason.

## 3.5 Reproduction of the original system (Phase 3)

Before porting, the original pipeline was run unchanged on its own Oerlikon data (launch file copied only to call `python3` and to make RViz optional), five times on the final build. The evaluation follows the original paper: translation and rotation error of the published map-frame pose against the simulator ground truth, without trajectory alignment, over the whole run and after the one-time refinement of T_M_W. The mono landmarks were confirmed to reach the registration (median ~830 per keyframe), and every registration published after refinement was confirmed to reach the backend (from the ROS log, after a stdout-buffering artefact had been identified and excluded). [phase3_oerlikon.md]

## 3.6 VIO baseline and evaluation metrics (Phase 4)

M1 uses SVO Pro's own EuRoC monocular parameter files, unmodified. The MH_01 run and five V1_01 runs (4-DoF ATE 0.056 ± 0.003 m against 0.061 m reported by Cioffi & Scaramuzza, 2020) confirmed that the baseline behaves as published. [phase4_m1.md]

Metrics (`analysis/eval_vio.py`, `analysis/initial_alignment.py`):

- **4-DoF ATE:** RMSE of position error after a closed-form yaw-plus-translation alignment of the whole trajectory (Zhang & Scaramuzza, 2018). Reported for comparability with the literature.
- **Map-frame ATE (primary for drift-reduction claims):** RMSE of position error with no trajectory alignment, using only the fixed initial alignment of §3.7, over the samples after the alignment window. Standard trajectory alignment would remove exactly the global correction the digital twin provides, so drift-reduction claims are made only on this metric.
- **Relative displacement error:** for segment lengths of 10–50 % of the path, the error in displacement between pose pairs after 4-DoF alignment. It depends only on positions. The usual RPE was also computed, but on V1 it is dominated by a ground-truth orientation inconsistency of several degrees (Phase 4), so orientation-dependent metrics are not used for conclusions.
- **Runtime:** frames processed, front-end time, backend output latency, CPU per process.

## 3.7 Initial alignment (assumption A1)

The VIO starts in a gravity-aligned frame with arbitrary yaw and origin; registration needs T_M_W. Indoors there is no GPS, so a one-time alignment stands in for a known take-off pose (decision D1(B)). From t₀, the first keyframe with an initialised backend and scale and with ground truth available (bracketing samples ≤ 10 ms apart), VIO positions are paired with ground-truth positions until the ground-truth path length reaches 2 m. T_M_W is then the closed-form 4-DoF least-squares fit (yaw about z plus translation); roll and pitch come from the VIO gravity estimate and ground-truth orientation is not used. The window length mirrors the original pipeline's 2 m GPS alignment distance and was confirmed on M1 runs by yaw stability (a plateau at 1–2 m, run-to-run spread 0.25°), before any registration result was examined. The rule is implemented identically online (in `svo_ros`, flag-guarded, setting T_M_W and the alignment flag in place of the GPS alignment) and offline (for M1); the two implementations agree to 1e-6° in every run. Map-frame metrics are computed only after the window. All methods use the same rule, and `refine_T_WL` is off in the main experiments. [frames.md]

An earlier variant that took the yaw from the ground-truth orientation at t₀ was abandoned when the logged registration corrections showed a consistent 4° yaw, which was traced to the ground-truth orientation (positions and the scan geometry agreed with each other). This is documented as a superseded decision. [phase5_m2_euroc.md]

## 3.8 Porting the original registration to EuRoC (Phase 5)

The registration node was copied and parameterised (`online_registration_fyp.py`), with defaults equal to the original. A room-scale configuration replaces the city-scale values; each value was derived from the M1 baseline, not from registration results: correspondence radius 0.5 m (the M1 map-frame error reaches ~0.45 m), inlier RMSE ≤ 0.15 m, fitness ≥ 0.75 (as original), correction limits 0.75 m and 5°, a 10 m crop box (whole room) and no further twin downsampling. The heuristic weighting factors were kept as in the original. The same values are used for room 2. [euroc_v1.yaml; phase5_m2_euroc.md §5c]

## 3.9 Instrumentation (Phase 6)

To compare registration methods on identical inputs and to diagnose behaviour, three logs were added, all off by default and log-only:

- the registration node writes one file per processed keyframe with the received cloud, the filtered cloud (float64), the initial guess, the correction, the result, fitness, RMSE, the raw and final weighting matrices, the verdict and timings;
- `svo_node` logs which branch each received registered pose takes (discarded, not initialised, added, used for alignment), and a frame-event log records the tracking stage of every frame (used to detect resets);
- an offline assembly matches, per keyframe, published clouds, processed clouds (to count clouds dropped by the node's queue of length 1), accepted registrations, backend receipt and "added" lines, and ground truth interpolated at the keyframe stamp.

Five M2 runs on V1_01 established the M2 baseline and the per-keyframe data for Phase 7. [phase6_logging.md]

## 3.10 Distance-field registration (Phases 7–8)

**Distance field.** For each preprocessed twin a dense grid covering its bounding box plus 1 m is built once, offline. Each voxel centre stores the exact Euclidean distance to the nearest twin point (KD-tree query). The field is unsigned, because a laser scan has no defined inside or outside. Truncation, when used, is applied at query time (D = min(d, μ), zero gradient beyond μ), so one grid serves all truncation values. Grids at v = 0.05, 0.10 and 0.20 m take 23.4, 3.0 and 0.39 MB. [phase7_offline.md]

**Registration.** As in M2, the unknown is a correction T_c = (R, t) applied on the left in the map frame, initialised at the identity. The objective is

  E(T_c) = Σᵢ ρ_δ( D(R pᵢ + t) ),

with D the trilinear interpolation of the grid, its analytic gradient ∇D, and ρ_δ the Huber function (δ = 0.05 m, fixed in advance; sensitivity at 0.02 and 0.10 m reported, not used for selection). It is minimised by Levenberg–Marquardt on SE(3) with the left perturbation (w, v) and Jacobian ∇Dᵀ[−[p′]× | I], iteratively reweighted for the Huber loss. Points outside the grid are discarded and counted. The iteration stops when the step is below 1e-7 or the relative cost decrease is below 1e-9, or after 100 iterations.

**Validity and weighting.** M3 uses M2's validity checks unchanged, with the ICP quantities replaced by their distance-field analogues on the untruncated field: fitness = fraction of points with D < 0.5 m, inlier RMSE = RMS of D over those points, followed by the same correction limits. For the main comparison M3 publishes its weighting in M2's heuristic convention, with the normalised gradient ∇D/‖∇D‖ in place of the surface normal (assumption A4, decision D4), so that the two methods differ in the registration and not in how strongly the backend trusts them. A Hessian-based alternative (JᵀWJ/σ²) is evaluated as an ablation (Phase 10).

**Offline study (Phase 7).** On the logged inputs of the five V1_01 M2 runs (917 keyframes with ground truth), both methods were run open-loop on identical clouds: a full factorial over voxel size, truncation and Huber δ for accuracy against ground-truth positions, and a convergence-basin study in which the same random perturbation (translation 0.1–1.0 m, rotation 2–20°, combined, and yaw-only) is applied to each keyframe for every method. Success means converging to the method's own unperturbed solution (within 0.05 m and 1°) and to within 0.25 m of ground truth; the thresholds were fixed before running and the validity gates are not part of success. Point-to-plane was run with correspondence radii of 0.5, 1.0 and 2.0 m. Because keyframes within a run are not independent, statistics were computed at run level (n = 5) and with a moving-block bootstrap (block length 10, 2000 resamples); keyframe-level p-values are not used. [phase7_offline.md]

**Implementation (Phase 8).** The registration was implemented as a C++/Eigen library with Python bindings, called from a copy of the M2 node that differs only in the registration step (`registration_method: point_to_plane | distance_field`). It is a line-by-line port of the Python prototype; Ceres was not used because equality with the prototype was the acceptance test and Ceres' trust-region strategy would not reproduce the prototype's iterates. The library is built with standard optimisation flags (no fast-math). The grid is exported once as a binary file with a header (format, dimensions, origin, voxel size, checksum, twin MD5); the loader rejects any mismatch. Verification, with tolerances fixed before running: on all 922 logged V1_01 keyframes the C++ results matched the prototype to ~1e-13 (pose, weighting matrices, iteration counts and verdicts identical), grid queries were bit-identical, and the node's point-to-plane path reproduced the logged online M2 outputs exactly. [phase8_cpp.md]

**Timing protocol (H2).** Speed was measured offline, in one process pinned to one CPU core with single-threaded libraries, on the same 922 inputs, with methods interleaved per keyframe: the core solve (Open3D ICP as configured in M2, which rebuilds a KD-tree of the twin on every call, timed together and with the tree build separately, against the C++ distance-field call), the per-iteration cost, the end-to-end node callback, and the memory of each map representation. The headline comparison is per iteration and per solve without the KD-tree rebuild; the speed-up including the rebuild is reported only as a property of the original implementation. [phase8_cpp.md §4, §7]

## 3.11 Online experiments: procedure (Phases 9–10)

Online comparisons use the frozen build of each experiment. Before every run the batch driver recomputes the checksums of the binaries, nodes, configurations, launch files, twins and grids and aborts on any difference. Methods are interleaved within each round (for example M1 → M2 → M3 per sequence), so that slow changes in machine state affect all methods equally. Ten runs per method and sequence were used in Phase 9; the per-experiment designs of Phase 10 are given in the results chapter. A technical smoke run checks each new configuration before a batch and is never part of the statistics. Per run the harness records the trajectory, the per-keyframe registration logs, CPU of each node, latencies and the failure status. [PLAN.md, Phase 9 pre-registration]

A second point-to-plane implementation, **M2-fast**, was added in Phase 10 as a throughput control: M2 with a KD-tree of the twin built once at start-up, re-implementing Open3D's ICP loop step by step. It reproduced every logged online M2 output to 1e-14 on 7 589 keyframes before use. [phase10_throughput.md]

## 3.12 Statistical methodology

- **Unit of analysis:** the run. Keyframes within a run are dependent and are never treated as independent samples.
- **Pre-registration:** every online experiment's design, sample size, primary test, interpretation rule and failure rule were written into `docs/PLAN.md` before the first run, on a frozen build. Deviations (one technical fix in the D4 ablation, an amended interpretation rule in the throughput control) were recorded with a timestamp before any affected result was analysed.
- **Failure rule:** a run in which SVO performs a full reset, the harness detects a node failure, no map-frame ATE can be computed or the online alignment does not fire is a failure for its method. Failures are counted and reported per method and sequence, excluded from accuracy statistics, and never repeated.
- **Tests:** Welch's two-sided t-test on run-level metrics, with the mean difference and its Welch 95 % confidence interval. Within a pre-registered family of primary tests the Holm correction is applied. A primary test that is significant after correction counts as supporting the stated difference; otherwise the confidence interval is reported and no claim is made. Secondary and exploratory analyses are labelled as such and not corrected.
- **Offline studies:** run-level statistics and a moving-block bootstrap (§3.10).

## 3.13 Assumptions and threats to validity (summary)

- A1: one-time alignment from ground-truth positions over the first 2 m (stands in for a known take-off pose).
- A2: map frame equals the Vicon frame by the dataset's calibration; its accuracy bounds the achievable map-frame error.
- A3/A4: identical map input and identical weighting convention for M2 and M3 in the main comparison.
- The acceptance thresholds were derived on room 1 and not retuned for room 2.
- The heuristic weighting is expressed about the map origin while the backend interprets it about the body (kept, as the faithful baseline).
- The validity checks do not reliably reject wrong registrations (fitness near 1 with a wrong solution was observed), and the backend's robust loss was set for the heuristic's weak weights.
- Short indoor sequences limit the drift that can be corrected; this motivated repeated runs and the run-level statistics above.

## References to verify before submission
Merat, Cioffi, Bauersfeld, Scaramuzza (RA-L 2025, arXiv:2412.08496); Forster et al. (SVO, SVO Pro); Burri et al. (IJRR 2016); Cioffi & Scaramuzza (2020, arXiv:2003.04159); Zhang & Scaramuzza (IROS 2018); Fitzgibbon (2003); Bylow et al. (2013); Newcombe et al. (2011); Curless & Levoy (1996); Oleynikova et al. (2017).
