# Phase 8: C++ distance-field module (M3), offline verification and H2 timing (2026-10-24)

**Status:** module built, verified against the Python prototype, node copy verified offline. **No online runs** (Phase 9).

## 1. What was built
| Item | File | Notes |
|---|---|---|
| C++ library | `svo_registration/df_cpp/include/fyp_df/df_registration.h`, `src/df_registration.cpp` | `DistanceGrid` (loader + trilinear query with analytic gradient), `registerCloud` (LM on SE(3), Huber IRLS), `informationHessian`. Eigen only. |
| Python bindings | `svo_registration/df_cpp/src/bindings.cpp` → module `fyp_df` | `DistanceGrid(bin_path, expected_twin_md5)`, `.query(p, mu=None)`, `.memory_bytes()`; `register(grid, points, mu=None, delta=0.05, max_iter=100, tol_step=1e-7, tol_cost=1e-9)` returns the same keys as `analysis/df_register.register`; `information_hessian(H_wv, sigma2)`. The solve releases the GIL. |
| Build | `svo_registration/df_cpp/CMakeLists.txt`, `build.sh` | Standalone CMake, plain `-O3`, **no fast-math** (docs/environment.md). Output `svo_registration/scripts/fyp_df.cpython-38-x86_64-linux-gnu.so` (git-ignored; rebuild with `build.sh`). |
| Registration node | `svo_registration/scripts/online_registration_m3.py` | Copy of `online_registration_fyp.py`; differs only in the registration step (section 3). |
| Config | `svo_registration/config/euroc_v1_m3.yaml` | `euroc_v1.yaml` values unchanged + `registration_method: distance_field`, `df_twin_md5`, `df_huber_delta: 0.05`, `df_max_iter: 100`, `df_information: heuristic`. |
| Launch (not run yet) | `svo_ros/launch/fyp/euroc_m3.launch` | `euroc_m2.launch` with the M3 node, M3 config and arg `df_grid`. |
| Tests / timing | `analysis/phase8_equivalence.py`, `analysis/phase8_node_timing.py` | Results in `runs/phase8/`. |

### Solver = the Phase 7 prototype, ported line by line
State R = I, t = 0, λ = 1e-3. Per iteration: J_i = [q_i × g_i, g_i] (ξ = (ω, v), left perturbation in the map frame), Huber weights w = 1 if |r| ≤ δ else δ/|r|, H = Σ w J Jᵀ, b = Σ w r J; damped solve (H + λ·diag(diag H + 1e-12)) dx = −b (LU with partial pivoting, as `numpy.linalg.solve`); R ← Exp(ω) R, t ← Exp(ω) t + v; trial cost scaled by n_valid/n_valid_trial; accept if not larger, else λ ×= 10 (until λ ≥ 1e8); after acceptance λ = max(λ/10, 1e-9); stop when |dx| < 1e-7 or the relative cost decrease < 1e-9, or after 100 iterations. Afterwards the Hessian at the final estimate, the untruncated D/∇D at the corrected points, and σ² = Σ w r² / (n_valid − 6). PLAN.md had "Ceres" for Phase 8; the exact port was used instead because equality with the prototype is the acceptance test (decision recorded in docs/PLAN.md).

### Grid file and loader checks
The grid is built offline by `analysis/df_grid.py` (unchanged since Phase 7): `<stem>.bin` (float32, C order, x slowest / z fastest) + `<stem>.bin.json`. The loader refuses the grid (exception → node does not start) if: format ≠ `fyp_udf_v1`; dtype ≠ float32; dims × 4 ≠ header bytes ≠ file size; the FNV-1a 64 checksum of the payload differs; or the header's twin md5 ≠ `df_twin_md5`. The node additionally checks that the md5 of `--reference-map` (the twin it publishes for RViz) equals `df_twin_md5`, so grid and twin cannot silently diverge. Tested: corrupted payload byte → "checksum mismatch"; 4 extra bytes → "size mismatch"; wrong md5 → rejected.

## 2. Equivalence: C++ vs Python prototype (all 922 logged V1_01 keyframes)
Inputs: `cloud_filtered` of every processed KF of `runs/2026-10-23_V1_01_easy_M2_p6_r01..r05` (the Phase 7 inputs). Grids: Python `.npz`, C++ `.bin` of the same float32 array. Validity and information post-processing are the Python functions in both cases, so the test isolates the solver. **Tolerances fixed before running:** |Δt| ≤ 1e-6 m, rotation difference ≤ 1e-6°, information max |ΔA| / max |A| ≤ 1e-6, identical iteration count and verdict.

| Quantity (max / median over 922 KFs) | Untruncated v = 0.05 (Phase 8 configuration) | TSDF μ = 5v (path check) |
|---|---|---|
| Correction translation difference [m] | 4.9e-15 / 8.4e-16 | 7.5e-15 / 8.3e-16 |
| Correction rotation difference [°] | 1.1e-13 / 9.7e-15 | 1.0e-13 / 9.7e-15 |
| Final cost, σ² (relative) | 3.4e-14, 2.7e-14 / ~1e-15 | 1.7e-14, 1.6e-14 / ~1e-15 |
| Fitness / inlier RMSE difference | 0 / 1.2e-15 m | 0 / 8.7e-16 m |
| Information, heuristic (relative) | 5.1e-14 / 2.3e-15 | 4.1e-14 / 2.2e-15 |
| Information, Hessian (relative) | 6.9e-14 / 3.0e-15 | 3.6e-14 / 3.2e-15 |
| Same iterations / lookups / verdict + reason / n_valid, n_outside | 922 / 922 / 922 / 922 | 922 / 922 / 922 / 922 |
| Accepted (Python, C++) | 920, 920 | 921, 921 |
| **Pass** | **yes** | **yes** |

Grid query on 200 000 random points (inside, boundary and outside; untruncated and μ = 0.25): validity masks identical, D and ∇D **bit-identical** (max difference 0). The remaining ~1e-14 differences come from floating-point summation order (Eigen vs NumPy/LAPACK), not from the algorithm. Note on the metric: rotation differences are measured as the rotation-vector norm of R_aᵀR_b; the first version used arccos((tr − 1)/2), which cannot resolve angles below ~1e-6° (one ulp of the trace near 1 ≈ 8.5e-7°) and reported a spurious 5e-6°. The tolerance was not changed.

## 3. The node copy (`online_registration_m3.py`)
`diff online_registration_fyp.py online_registration_m3.py`: header comment; `hashlib` import; `DFRegistrationResult` (the four fields of Open3D's RegistrationResult the node uses); `__init__` takes `df_grid`; `load_config` reads the M3 keys (defaults select the M2 path) and loads/verifies the grid only for `distance_field`; in the callback the twin crop + ICP are in the `point_to_plane` branch and `register_distance_field` in the other; the `df_information: hessian` option inside the accepted branch; logging adds `method,iterations` (CSV) and `registration_method, iterations, df_information` (event `.npz`); `--df-grid` CLI argument. Unchanged: subscriptions/publications and message contents, local-cloud filtering, `validate_icp_result`, `calculate_information_matrix` + normalisation + scale factors, event writer, twin loading/publishing.

`register_distance_field` gives the downstream code the distance-field analogues of the ICP quantities (as in Phase 7): fitness = fraction of all points with untruncated D < `max_correspondence_distance` (0.5 m), inlier_rmse = RMS of D over them, correspondences = (inlier point, ∇D/|∇D| at its corrected position) for inliers with |∇D| > 1e-9; init = identity, correction applied on the left in the map frame (same as M2). The DF branch does not crop the twin (no target cloud), so `n_target` is logged as 0 and the twin crop is not part of its timing. Hessian option: JᵀWJ / σ², reordered to [pos, rot], published as is (no normalisation or scale factors), as in the Phase 7 ablation.

### Node-level check (offline, `analysis/phase8_node_timing.py`)
The node class is driven with messages rebuilt from the logged raw clouds (exact: the original message is float32) and logged VIO poses; publishers are stubs.
| Check (922 KFs) | Result |
|---|---|
| M2 path: verdict = logged online M2 verdict | 922 / 922 (accepted 899) |
| M2 path: published pose vs logged online M2 result (max / median) | 5.9e-15 / 4.6e-16 m; 1.1e-13 / 2.0e-14° |
| M2 path: published information vs logged (relative, max) | 3.2e-15 |
| Node's local filtering = logged `cloud_filtered` (at float32 precision¹) | 922 / 922 |
| M3 path: verdict = prototype on the node's filtered cloud | 922 / 922 (accepted 920; heuristic and Hessian option) |
| M3 path: published pose vs prototype (max) | 8.2e-15 m; 1.3e-13° |
| M3 path: information vs prototype (relative, max), heuristic / Hessian | 2.5e-14 / 7.6e-14 |

¹ The p6 runs logged `cloud_filtered` as float32 (before the Phase 7 float64 fix); the node computes it in float64 (differences ≤ 1.2e-7 m, same points and order). For the M3 check the prototype therefore runs on the node's own float64 cloud. The smoke test first compared against the logged float32 cloud and showed ~3e-8 m differences, all explained by this rounding.

## 4. H2 timing (offline, single core)
Protocol (decided here, docs/PLAN.md): one process pinned to CPU core 3 of an Intel i7-8700, `OMP_NUM_THREADS=1`, system Python 3.8 + Open3D 0.19.0, the node's own filtered clouds of the 922 KFs, methods interleaved per KF, first 5 KFs as warm-up; machine otherwise idle (desktop session only, no ROS).
**(a) Core solve, per keyframe** (917 KFs after warm-up; median local cloud 350 points)

| | median [ms] | p90 [ms] |
|---|---|---|
| Open3D `registration_icp`, as configured in M2 (cap 25, target = node's twin crop, 402 142 points) | 160.0 | 167.3 |
| … of which Open3D's KD-tree build of the target (timed alone on the same target) | 154.0 | 160.0 |
| … ICP iterations only (difference, indirect) | 5.2 | 12.1 |
| C++ DF `fyp_df.register` (Python call) | 1.19 | 1.73 |
| … library-internal time | 1.13 | 1.66 |

- **As M2 is built**, the core solve is **~135× faster** with DF (160 vs 1.19 ms median). This comes almost entirely from Open3D rebuilding the KD-tree of the whole twin on every call (M2 crops the twin per keyframe; at room scale the 10 m box keeps all of it). That is a property of the M2 implementation, not of point-to-plane ICP as such: with a persistent KD-tree, the ICP part would be ~5.2 ms (median, indirect estimate).
- **Per iteration (the H2 claim):** P2P 0.46 ms median (p90 1.08; 673 KFs that converged in < 25 iterations, iteration counts from Phase 7, so this is indirect) vs DF 0.043 ms (p90 0.052; includes rejected LM trial evaluations). Per point evaluation: ~1.30 µs (NN query + point-to-plane terms) vs ~0.054 µs (trilinear lookup). **DF needs more iterations** (median 26 vs 9), so the end-to-end solve advantage without the KD-tree rebuild is ~4–5× (5.2 vs 1.13 ms), not the ~10× per iteration.

**(b) End-to-end per-KF callback** (preprocessing + registration + validity + information + output message construction; ROS publish excluded)

| Node path | median [ms] | p90 [ms] | preprocessing | registration step | information |
|---|---|---|---|---|---|
| M2 (`point_to_plane`) | 277.9 | 288.2 | 84.4 (incl. twin crop of 402k points) | 159.8 | 32.1 |
| M3 (`distance_field`, heuristic information) | 35.7 | 40.7 | 2.0 | 1.45 | 30.9 |
| M3, Hessian information (D4 ablation) | 4.6 | 5.4 | 2.0 | 1.45 | 0.11 |

(Split columns are medians from the node's own CSV log.) In M3 the remaining cost is the M2 information heuristic, a per-point Python loop (31 ms), which the M3 node keeps unchanged by design. For comparison, Phase 6 measured the online M2 registration at 301 ± 2.5 ms/KF (multi-threaded Open3D, under the full system load); keyframe clouds arrive every 0.50–0.55 s (median; 10 % of intervals ≤ 0.25–0.30 s, minimum 0.15 s) in the logged V1_01 M2 runs, so M2 cannot keep up with the shortest intervals (queue_size 1 drops; 0.2 drops/run in Phase 6), whereas M3 at 36 ms (offline estimate) should not drop: to be confirmed online in Phase 9.

## 5. Memory
| Map representation (fresh process, RSS increase) | Memory |
|---|---|
| DF grid v = 0.05 m (211 × 215 × 129 float32) | 23.4 MB payload; RSS +23.1 MB; peak +23.1 MB |
| P2P: twin cloud (402 142 points + normals, float64) | RSS +19.4 MB |
| P2P: KD-tree of the twin (built on every call in M2) | RSS +16.7 MB |

The first timing run measured a peak of +47.8 MB for the grid load, because the loader read the payload into a temporary string and then copied it. The loader now reads directly into the grid buffer (re-verified: equivalence test unchanged, bit-identical queries; the solver is unchanged, so the timing still holds). **H4 at room scale:** the grid (23 MB) is **not** larger than point-to-plane's map data (twin + KD-tree ≈ 36 MB). The (1/v)³ growth (v = 0.10 → 3.0 MB, v = 0.20 → 0.39 MB, Phase 7) and city-scale extrapolation are Phase 10 material. Note that the M3 node still loads the twin, to publish it for RViz (+19 MB, display only).

## 6. What this does and does not show
- The C++ module reproduces the prototype to ~1e-13 on every logged V1_01 keyframe, so the Phase 7 offline accuracy, jitter and basin results carry over to the C++ module unchanged (for these inputs).
- The M3 node's M2 path is the M2 node (identical outputs to the logged online runs), and its DF path equals the prototype; M2 and M3 therefore differ only in the registration step.
- Timing is offline, single-threaded, and excludes ROS transport. Online CPU use (Open3D multi-threaded in M2) is measured in Phase 9. Nothing here says anything about closed-loop accuracy (Phase 9).

## 7. Hypothesis status after Phase 8 and report wording (approved 2026-10-24)
- **Solver (report wording):** "Levenberg–Marquardt on SE(3) in C++/Eigen, verified against the prototype to ~1e-13."
- **H2 (speed), headline:** per iteration **~10×** faster (DF 0.043 ms vs P2P 0.46 ms median; the P2P per-iteration figure is **indirect**: ICP time minus the separately timed KD-tree build, divided by Phase 7 iteration counts, on the 673 KFs that converged in < 25 iterations), and the overall solve **without the KD-tree rebuild ~4–5×** faster (1.13 vs ~5.2 ms median; DF needs ~3× more iterations). **The 135× figure (160 vs 1.19 ms) is a property of M2's implementation** (Open3D rebuilds the KD-tree of the whole twin on every call) and is reported only with that label, never as the algorithmic speed-up. Status: supported (offline, single core).
- **H4 (memory), reworded:** "At room scale the DF grid (23 MB, v = 0.05 m) is comparable to and slightly smaller than P2P's map + KD-tree (~36 MB). Memory grows with volume × (1/v)³; city-scale extrapolation and sparse storage are Phase 10."
