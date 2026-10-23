# FYP PROJECT CONTEXT

## Volumetric Digital-Twin Registration for Drift Reduction in UAV Visual-Inertial SLAM

**Purpose of this document:** Give you (the assisting LLM) the complete project context so you can help with research, implementation, debugging, experiments, and documentation without the user re-explaining anything.

---

## 1. PROJECT OVERVIEW

**Title:** Volumetric Digital-Twin Registration for Drift Reduction in UAV Visual-Inertial SLAM

**Poster title:** Voxel-Based Digital Twin Registration for Drift Reduction in UAV Visual-Inertial SLAM
**Poster subtitle:** Geometric Map Alignment for GPS-Denied Drone Navigation

**One-sentence summary:** Replace the point-to-plane registration stage of an existing VIO + digital-twin system (SVO Pro with Digital Twins) with a voxel distance-field (TSDF/ESDF) registration, and measure whether it changes localization accuracy, drift, robustness, and computational cost.

**Application framing:** UAV localization in GPS-denied and GPS-degraded environments (indoor and urban). Experiments are indoor (EuRoC Vicon rooms); an urban reproduction of the original work is optional.

**Base repository (user's fork):** https://github.com/MaathavanJkr/rpg_svo_pro_with_digital_twins
**Upstream:** https://github.com/uzh-rpg/rpg_svo_pro_with_digital_twins
**Built on:** SVO Pro — https://github.com/uzh-rpg/rpg_svo_pro_open
**License:** GPLv3 (academic use is fine; keep the license and cite the authors).

---

## 2. PROBLEM STATEMENT

Visual-Inertial Odometry (VIO) estimates a UAV's 6-DoF pose incrementally from camera and IMU data. Small errors from tracking, IMU integration, calibration, and optimization accumulate into **drift**, mainly in position and yaw (the four unobservable degrees of freedom of VIO).

- GPS corrects drift outdoors but is unavailable indoors and unreliable in urban canyons.
- Loop closure only helps when places are revisited.

A **prior geometric map of the environment (digital twin)** can provide absolute pose constraints without visual feature matching: align the 3D structure the VIO observes with the known geometry.

---

## 3. THE EXISTING METHOD (BASELINE)

**Paper:** R. Merat, G. Cioffi, L. Bauersfeld, D. Scaramuzza, "Drift-free Visual SLAM using Digital Twins," IEEE Robotics and Automation Letters (RA-L), 2025. arXiv:2412.08496 — https://arxiv.org/abs/2412.08496

**How it works:**
1. SVO Pro runs sliding-window visual-inertial odometry with a Ceres optimization backend.
2. The sparse 3D point cloud produced by VIO landmarks is registered to the digital twin using **point-to-plane matching**. No image features are matched against the map.
3. The registration result is a **6-DoF global pose measurement**, fused tightly into the VIO optimization.
4. The original work targets **urban environments** with a **city-scale digital twin** (Oerlikon, Zurich). It was evaluated on a high-fidelity GPS simulator and real drone flights, and reported to outperform VIO-GPS fusion and to be more robust to viewpoint changes than visual-feature-based map localization.

**Repository top-level packages:**
`svo`, `svo_ros`, `svo_registration`, `svo_ceres_backend`, `svo_global_map`, `svo_pgo`, `svo_online_loopclosing`, `svo_benchmarking`, `svo_common`, `svo_direct`, `svo_img_align`, `svo_tracker`, `svo_vio_common`, `svo_msgs`, `rqt_svo`, `rpg_common`, `vikit`, `gps_simulator`, `scripts`, `doc`, `dependencies.yaml`.

**Documented usage (README):**
- Download the custom rosbag and city-twin files from https://download.ifi.uzh.ch/rpg/svo_with_digital_twins and place them as specified in the config file.
- Run `roslaunch svo_ros Oerlikon.launch`, and play the rosbag in another terminal.
- This launches `svo_ros` (VIO accepting global poses), `svo_registration` (registers the local point cloud to the twin), and RViz.
- For other data, topic names in the launch file must be adjusted.

**Tested platforms:** Ubuntu 18.04 + ROS Melodic, Ubuntu 20.04 + ROS Noetic. Built with catkin tools; dependencies imported with `vcs-import < dependencies.yaml`.

**Not yet known (must be inspected in the user's fork before any code advice):**
- Registration source files and function names.
- Digital-twin file format (mesh or point cloud) and how it is loaded.
- How the global measurement's covariance is set.
- How the VIO world frame is initially aligned with the digital-twin frame.
- Topic names and configuration parameter names.
- Whether any EuRoC launch/config files exist.

---

## 4. RESEARCH QUESTION, HYPOTHESES, CONTRIBUTION

**Research question:**
Does replacing point-to-plane registration with voxel distance-field registration change drift correction, registration robustness, and computational cost in a tightly-coupled VIO + digital-twin system?

**Hypotheses (to be tested, not assumed):**
- **H1 — Convergence basin:** Distance-field registration converges from larger initial pose errors than point-to-plane ICP.
- **H2 — Speed:** Per-iteration cost is lower because distance lookup is O(1) instead of a nearest-neighbour search.
- **H3 — Accuracy:** Map-frame trajectory error is equal or lower when used inside the full system.
- **H4 — Memory:** Memory cost is higher and grows with map volume and (1/voxel size)³; large maps need sparse storage.

**Contribution:**
1. A distance-field registration module that is a drop-in alternative to the existing registration stage (same inputs and outputs).
2. Porting the digital-twin pipeline to an indoor public benchmark (EuRoC Vicon rooms with their laser-scan reference).
3. A controlled comparison — VIO only vs point-to-plane digital twin vs distance-field digital twin — including convergence-basin, voxel-size, truncation, runtime, and memory studies.

**Novelty note:** Registering points against a distance field is an established technique (e.g., Fitzgibbon 2003; Bylow et al. 2013; KinectFusion-style SDF alignment). The contribution is the controlled evaluation of this technique inside a tightly-coupled VIO + digital-twin UAV localization system. A negative or mixed result is still a valid finding if the experiments are sound.

---

## 5. DIGITAL TWIN AND DATASETS

**Definition for this project:** A static geometric reference of the environment with a known coordinate frame, used only for localization. It is not a live IoT digital twin. Possible sources in general: CAD, BIM, laser scans, reconstructed 3D maps, synthetic environments.

**EuRoC MAV dataset** provides stereo images, IMU, calibration, and ground-truth trajectories. Typical sequence structure: `mav0/cam0`, `mav0/cam1`, `mav0/imu0`, ground-truth folder, and (for Vicon rooms) a reference point cloud folder.

**Reference maps available:**

| Source | Environment | Role |
|---|---|---|
| EuRoC Vicon Room 1 and Vicon Room 2 laser scans (Leica, expected at `mav0/pointcloud0/`) | Indoor, room scale | **Primary digital twin** for all registration experiments. Presence, file format (.ply), and frame relative to the Vicon ground-truth frame (see its `sensor.yaml`) must be verified after extraction. |
| Oerlikon city twin (upstream download) | Urban, city scale | Optional: reproduce the original paper to confirm the original pipeline works before porting it. |
| EuRoC Machine Hall | Indoor, large hall | **No reference scan** → usable for VIO-only baseline / sanity checks only. |

**Map independence:** The Leica scan is an independent laser measurement of the room, so it is a legitimate digital twin. EuRoC ground-truth poses must never be used to build the map.

**Initial alignment:** VIO starts in its own gravity-aligned frame with arbitrary yaw and position, so an initial VIO-to-twin alignment is required. Indoors this can be the first ground-truth pose (a documented assumption). Whatever is chosen must be identical for all compared methods and stated in the report.

**Sequences:**
- Sanity check: MH_01_easy (VIO only).
- Main experiments: V1_01_easy, V1_02_medium, V1_03_difficult, V2_01_easy, V2_02_medium, V2_03_difficult.
- Start with **V1_01_easy**. Do not run all sequences at once.

---

## 6. CURRENT STATUS

**Confirmed by the user:**
- Ubuntu 20.04 (ROS Noetic), dependencies installed.
- The fork builds and has previously been run locally.
- EuRoC ZIPs downloaded into the repo's `dataset/` folder:
  - Machine Hall (~12.1 GB): https://www.research-collection.ethz.ch/bitstreams/7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/download
  - Vicon Room 1 (~5.8 GB): https://www.research-collection.ethz.ch/bitstreams/02ecda9a-298f-498b-970c-b7c44334d880/download
  - Vicon Room 2 (~5.7 GB): https://www.research-collection.ethz.ch/bitstreams/ea12bc01-3677-4b4c-853d-87c7870b8c44/download
- The user has also worked with Open3D, GTSAM, ROS, and point-cloud / volumetric representations.

**Not yet verified:**
- Whether the ZIPs are extracted, and whether they contain ASL-format folders, rosbags, or both. SVO Pro consumes ROS topics, so ASL-only data must be converted to a rosbag.
- Whether `pointcloud0` exists in the V1/V2 sequences.
- Which launch file has been run, and on which data.
- Any EuRoC results.

**Housekeeping:** Datasets inside the repo should be in `.gitignore`.

---

## 7. FEASIBILITY

**Verdict:** Feasible as a staged comparative study — offline prototype first, online integration second. The main risks are integration effort and demonstrating a measurable effect on short indoor sequences, not the mathematics.

**Risks and mitigations:**

| Risk | Mitigation |
|---|---|
| No EuRoC + digital-twin launch/config; porting from urban to indoor (topics, calibration, twin loading, initial alignment) takes time | Reuse SVO Pro's EuRoC calibration/parameters; copy `Oerlikon.launch` into a new `euroc_dt.launch`; change one thing at a time. |
| Machine Hall has no reference map | Use V1/V2 for all digital-twin experiments. |
| Vicon sequences are short (~1–2.5 min, one room) → VIO drift is already small → improvements may be within noise | Repeat runs (e.g., 5 per sequence) and report mean ± std; include the harder sequences; run the convergence-basin study (independent of drift magnitude); optionally use Oerlikon for a long trajectory. |
| Sparse/noisy SVO landmarks weakly constrain registration | Accumulate points over the sliding window (check what the original does); log point counts and Hessian eigenvalues per registration. |
| Degenerate geometry (flat walls/floor) | Compute covariance from the Gauss-Newton Hessian; reject or down-weight degenerate solutions. |
| Truncated SDF has zero gradient beyond the truncation distance | Use an untruncated distance field, large truncation, or coarse-to-fine; truncation is also an experimental variable. |
| Raw laser-scan point cloud has no defined inside/outside | Use unsigned distance (squared distance is smooth), or estimate normals for signed distance. |
| C++ integration into the Ceres backend is time-consuming | The new module only needs to output the same 6-DoF pose + covariance as the original; keep the backend untouched. |
| Memory at large scale | A room at 5 cm voxels is a few million voxels (tens of MB) — fine. City scale would need sparse storage (OpenVDB / voxel hashing). |

**Minimum viable result (must achieve):**
1. VIO baseline on V1_01_easy, evaluated against ground truth.
2. Original point-to-plane digital-twin registration running on V1_01_easy with the Leica scan.
3. Offline distance-field registration on logged SVO point clouds from that run, compared to point-to-plane on identical inputs (accuracy vs ground truth, convergence basin, runtime).

**Target / stretch:**
- Online integration of the distance-field module; full three-method comparison on all six Vicon sequences.
- Voxel size and truncation ablation.
- Oerlikon reproduction and urban comparison.
- Runtime profiling on embedded UAV hardware (e.g., Jetson).

**Department scope (open issue):** The department expects an integrated-circuit/hardware component and a cybersecurity specialization, while this project is computer vision/robotics. This must be confirmed with the supervisor. Possible bridges (suggestions only): embedded real-time profiling or GPU/FPGA acceleration of distance-field lookup (hardware); robustness of map-based localization to tampered digital-twin data or to GPS spoofing (security).

---

## 8. SYSTEM ARCHITECTURE

```
EuRoC V1/V2 (images + IMU, as rosbag)
            |
            v
   SVO Pro VIO (frontend + Ceres sliding-window backend)
            |------------------> VIO pose (Method 1 output)
            |
            +--> sparse local point cloud (landmarks, VIO frame)
                        |
                        v
        Registration  [A] point-to-plane (original, Method 2)
                      [B] distance field TSDF/ESDF (new, Method 3)
                        ^
                        |
        Digital twin (Leica scan)
            [A] points + normals   [B] voxel distance grid (built offline)
                        |
                        v
        6-DoF global pose measurement + covariance
                        |
                        v
        Fused into the existing Ceres backend (interface unchanged)
                        |
                        v
        Trajectory -> evaluation against Vicon ground truth
```

**Design rule:** [B] produces exactly the same output as [A] and is selected by a launch/config flag. The original code is never modified destructively.

---

## 9. PROPOSED REGISTRATION METHOD

**Offline map construction (once per environment):**
1. Load the reference cloud; crop to the flight volume; downsample.
2. Create a voxel grid of size `v` covering the bounding box plus a margin.
3. For each voxel, store the distance `d(x)` to the nearest reference surface (Euclidean distance transform or KD-tree query). Optional: sign from estimated normals; optional: truncation at `μ`, i.e. `D(x) = clip(d(x)/μ, −1, 1)`.
4. Store gradients (precomputed or central differences at runtime).
5. Save to disk (binary grid or OpenVDB).

**Online registration (per keyframe / window):**

Transform: `p_ref = R · p_local + t`

Objective: `E(R, t) = Σ_i ρ( D(R p_i + t)² )`

- `D(·)`: trilinear interpolation of the distance grid.
- `ρ`: robust kernel (Huber or Cauchy); optionally also weight points by landmark depth/uncertainty.
- Optimize on SE(3) with Gauss-Newton or Levenberg-Marquardt (Ceres is already a dependency). Jacobian: `∂D/∂ξ = ∇D(p)ᵀ · [ I | −[p]× ]`.
- Initialization: current VIO estimate of the VIO-to-twin transform.
- Points outside the grid: discard and count.
- Convergence: maximum iterations, step-size and cost-change thresholds.
- Validity checks: inlier ratio, final RMS distance, Hessian condition number; reject failures.
- Covariance: `Σ ≈ σ² H⁻¹` from the final Hessian, passed to the backend.

**Variants to compare:**
- Untruncated distance vs TSDF with `μ ∈ {2v, 5v, 10v}`.
- Voxel size `v ∈ {0.05, 0.10, 0.20} m` (starting points, not final values).
- Single resolution vs coarse-to-fine.

**Difference from point-to-plane ICP:** ICP re-searches nearest neighbours each iteration and approximates the surface as a plane at the matched point. The distance field gives point-to-surface distance (up to grid resolution) with O(1) lookup and no correspondence step, at the cost of discretization error and memory. The experiments measure these trade-offs.

---

## 10. EXPERIMENTS AND EVALUATION

**Methods:**
- **M1** — VIO only (registration disabled).
- **M2** — Original point-to-plane digital-twin registration.
- **M3** — Distance-field digital-twin registration (variants from Section 9).

All methods use the same sequences, calibration, initial alignment, and backend parameters; M2 and M3 differ only in the registration module. The main comparison is M2 vs M3; M1 shows whether digital-twin registration helps at all.

**Offline registration study** (using point clouds logged from runs):
- Registration error vs ground truth (translation, rotation) per keyframe.
- Convergence basin: perturb the initial pose by increasing offsets (e.g., 0.1–1.0 m, 2–20°) and record success rates for point-to-plane vs distance field.
- Runtime per registration; map memory.

**Metrics:**
- **ATE RMSE**, reported two ways: (a) position-yaw aligned, for comparability with the literature; (b) **map frame with no alignment** (only the fixed known frame transform). Drift-reduction claims must come from (b), because standard alignment removes the global correction the digital twin provides.
- RPE at several segment lengths.
- Translation and rotation error over time (drift plots).
- Registration success/convergence rate and registration error vs ground truth.
- Registration runtime, total runtime / real-time factor, memory usage.

A method that reduces error but is too slow or memory-heavy for a UAV must be reported as such.

**Tools:** `svo_benchmarking` (in repo — inspect first), `rpg_trajectory_evaluation` or `evo`, Open3D (Python) for map preprocessing, prototyping, and visualization, RViz for online inspection.

---

## 11. IMPLEMENTATION PHASES

| Phase | Goal | Done when |
|---|---|---|
| 1 | Dataset verification | Extracted structure known; rosbag vs ASL known; `pointcloud0` confirmed for V1/V2 |
| 2 | Repository inspection | Launch files, registration source, twin loader, measurement interface, initial-alignment logic identified |
| 3 | (Recommended) Reproduce Oerlikon | Original pipeline confirmed working on its own data |
| 4 | VIO baseline (M1) | V1_01 evaluated vs ground truth; MH_01 sanity check done |
| 5 | Port digital-twin pipeline to EuRoC (M2) | `euroc_dt.launch` + config created; Leica scan loads in the correct frame (checked in RViz); M2 evaluated |
| 6 | Logging | SVO local clouds, VIO poses, and ground truth saved per registration event |
| 7 | Offline distance-field prototype (Python) | Voxel grid built and visualized; registration on logged clouds; offline study done |
| 8 | C++ module (new branch / separate node) | Same output interface as the original; tested on logged data |
| 9 | Online integration (M3) | Selectable by config flag; full runs complete |
| 10 | Evaluation and ablations | Tables and plots for all methods and variants |
| 11 | Write-up | Report, poster, demo |

Phases 1–7 form the minimum viable result; 8–9 are the target.

For every run, save: estimated trajectory, ground truth, logs, the exact config/launch files used, and runtime.

---

## 12. RULES AND CONSTRAINTS

- Do not rebuild the SLAM system; change only the registration stage.
- Never modify the original registration destructively — use a new branch, new file/node, and a config switch.
- Never invent launch files, source files, ROS topics, functions, parameters, or dataset paths. Ask the user for command output instead.
- Keep coordinate frames explicit: IMU/body (B), cameras (C0, C1), VIO world (W), digital twin (M), Vicon ground truth (V). Record every transform used and its source. Frame errors are the most likely cause of misleading results.
- Ground truth is used for evaluation (and, if needed, the documented one-time initial alignment) only — never for building the map.
- Do not claim the volumetric method improves anything until experiments show it. Report negative results honestly.
- Use "drift reduction", not "drift-free", except when citing the original paper's title.

---

## 13. KEY REFERENCES

- Merat, Cioffi, Bauersfeld, Scaramuzza — Drift-free Visual SLAM using Digital Twins, IEEE RA-L 2025 (arXiv:2412.08496).
- Forster et al. — SVO / SVO Pro (rpg_svo_pro_open).
- Burri et al. — The EuRoC micro aerial vehicle datasets, IJRR 2016.
- Curless & Levoy, 1996 — volumetric TSDF fusion.
- Newcombe et al., 2011 — KinectFusion.
- Fitzgibbon, 2003 — Robust registration of 2D and 3D point sets.
- Bylow et al., 2013 — Real-time camera tracking and 3D reconstruction using signed distance functions.
- Oleynikova et al., 2017 — Voxblox (TSDF/ESDF for MAVs).
- Zhang & Scaramuzza, 2018 — A tutorial on quantitative trajectory evaluation for visual(-inertial) odometry.

Verify bibliographic details before final citation.

---

## 14. IMMEDIATE NEXT TASK

**Goal:** Confirm dataset and repository structure, then get a VIO baseline on V1_01_easy (MH_01_easy as a sanity check).

If the user has not provided it yet, ask them to run these from the repo root and paste the output:

```bash
git status && git branch -a

find . -name "*.launch" | sort
find . -path ./dataset -prune -o \( -name "*.yaml" -o -name "*.yml" \) -print | sort
find . -path ./dataset -prune -o -iname "*registration*" -print
find . -iname "*euroc*" | head -50

ls -lh dataset/
find dataset -maxdepth 4 -type d | head -100
find dataset -name "*.bag" | head
find dataset -path "*pointcloud0*" | head

find . -name "package.xml" | sort
```

Use the output to determine:
1. Whether the data is extracted, and whether rosbags exist or must be created from ASL folders.
2. Whether V1/V2 contain the Leica reference scan.
3. Whether an EuRoC launch/config exists or must be created from `Oerlikon.launch`.
4. Where registration and digital-twin loading are implemented in `svo_registration` / `svo_ros`.

Then guide the user step by step, one command at a time, based on the real output.

---

## 15. HOW YOU SHOULD ASSIST

- Act as an FYP research and implementation assistant for this specific repository.
- The user prefers concise, sequential steps — give only the next necessary steps unless asked for more.
- Read terminal output carefully and respond with the next concrete command.
- Before suggesting code changes: identify the actual file, explain the change, preserve existing behaviour, and do not assume the fork matches upstream.
- Keep the existing method and the proposed contribution clearly separate, and keep hypotheses separate from findings.
- For datasets: verify structure, sensor configuration, ground-truth availability, and reference-map independence.
- Proactively flag coordinate-frame and evaluation-alignment issues.









Work only on branch dev/volumetric-registration, and never commit dataset/.
Append every run to runs/RUNLOG.md with the commit hash, config, sequence, and results.
Never use ground truth for anything except evaluation and the documented initial alignment.

---

### Working rules for Claude Code
- Work only on branch `dev/volumetric-registration`. Never commit to master. Never commit anything under dataset/ or any large data or twin files.
- Never modify the original point-to-plane registration destructively. New code goes in new files/nodes behind a config flag.
- Never invent file names, topics, parameters, or paths. Inspect the repo and the data first. If something is unknown, say so.
- Never modify or delete original dataset files. Converted or derived data goes in dataset/derived/.
- Ground truth is used only for evaluation and the documented one-time initial alignment. Never use it to build the map.
- Keep coordinate frames explicit (B, C0, C1, W, M, V). Record every transform and its source in docs/frames.md.
- Every run is appended to runs/RUNLOG.md with: date, git commit hash, sequence, method, launch/config files used, and results.
- I cannot assume you can see GUI windows (RViz). When a visual check is needed, produce headless plots/screenshots (Open3D/matplotlib) AND tell me exactly what to check in RViz myself.
- Ask before: any operation writing more than ~5 GB, installing system packages with sudo, or deleting anything.
- Work phase by phase from docs/PLAN.md. At the end of each phase, stop at its GATE, summarize results in under 15 lines, list what I must check, and wait for my approval.
- Never claim the volumetric method improves anything without experimental evidence. Report negative results honestly.
- Note: `doc/` is upstream documentation; project documents go in `docs/`.
