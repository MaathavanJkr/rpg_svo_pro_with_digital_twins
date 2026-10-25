# Phase 10 / D4 step 1: weighting realism check (offline, 2026-10-25)

**Question:** how realistic are the uncertainties implied by the two information conventions (heuristic, as M2/M3 publish; Hessian, the D4 option) for point-to-plane (P2P) and distance field (DF), compared with the observed registration errors?
**Inputs:** every logged keyframe of the Phase 9 M2 runs (p9 r01–r10) on V1_01, V1_02, V2_02 (`cloud_filtered` float64, `T_init_world_body`), identical for both methods. P2P = point-to-plane ICP as M2 (`p2p_icp_persistent.py`; verdicts equal to the logged M2 verdicts on 6658/6658 KFs); DF = C++ `fyp_df` (v = 0.05 m, untruncated, Huber 0.05, 100 iterations). Accepted KFs per method only. Script `analysis/phase10_d4_realism.py`; results `runs/phase10_d4/realism.json`, per-KF `realism_kf.csv`; figure `docs/img/phase10_d4_realism.png`. Offline, open-loop; GT used for evaluation only.

**Definitions.** The backend reads the published 6×6 matrix as the information of [body position, body rotation] (map frame), so the implied covariance is its inverse: σ_pos (per axis, RMS over axes and KFs) and σ_yaw (rotation about z, median over KFs). Empirical position error = registered body position − GT at the KF stamp; per-axis RMS. Consistency: position NEES = eᵀ Σ_pos⁻¹ e (3 for a consistent estimator; 95 % of values below 7.81). Empirical yaw noise per KF = KF-to-KF yaw jitter / √2 (Phase 7 definition, median over runs; there is no reliable GT orientation). Ratio = implied / empirical: < 1 overconfident, > 1 underconfident.

## Results
| Method / sequence | accepted KFs | empirical σ_pos per axis [m] | empirical yaw noise [°] | **heuristic**: implied σ_pos [m] (ratio) | heuristic: NEES/3 | heuristic: implied σ_yaw [°] (ratio) | **Hessian**: implied σ_pos [m] (ratio) | Hessian: NEES/3 (median NEES) | Hessian: implied σ_yaw [°] (ratio) |
|---|---|---|---|---|---|---|---|---|---|
| P2P V1_01 | 1806 | 0.065 | 0.44 | 0.40 (6.1) | 0.06 | 71 (162) | 0.014 (0.22) | 67 (82) | 0.22 (0.50) |
| P2P V1_02 | 2003 | 0.048 | 0.37 | 0.35 (7.3) | 0.04 | 59 (158) | 0.016 (0.33) | 26 (39) | 0.19 (0.50) |
| P2P V2_02 | 2706 | 0.049 | 0.50 | 0.33 (6.6) | 0.05 | 62 (123) | 0.015 (0.31) | 22 (30) | 0.24 (0.48) |
| DF V1_01 | 1838 | 0.062 | 0.35 | 0.33 (5.4) | 0.07 | 63 (182) | 0.016 (0.26) | 43 (49) | 0.25 (0.71) |
| DF V1_02 | 2003 | 0.042 | 0.26 | 0.31 (7.5) | 0.04 | 56 (215) | 0.017 (0.41) | 17.5 (22.5) | 0.20 (0.76) |
| DF V2_02 | 2721 | 0.041 | 0.31 | 0.31 (7.5) | 0.04 | 61 (199) | 0.016 (0.40) | 14 (16) | 0.26 (0.86) |

(NEES/3 for the heuristic: 0.04–0.07, i.e. the observed errors are far smaller than its claimed uncertainty. Precise values in `realism.json`.)

**Bias sensitivity.** The mean error vector is 0.041 m (P2P) / 0.038 m (DF) on V1_01, almost entirely in x (a systematic offset common to both methods, e.g. twin vs Vicon frame or GT reference point), and ≤ 0.012 m on V1_02 and V2_02. Removing it changes the Hessian NEES/3 only modestly (P2P V1_01 67 → 54, DF V1_01 43 → 36; V1_02 / V2_02 within 1). The conclusions below do not depend on it.

**Diagnostic: Hessian re-expressed at the body.** The published Hessian keeps the map-origin parametrisation of the correction (as M2's heuristic does). Re-expressing it at the body (δp = v − [p_body]× w) makes it *more* overconfident (position ratio 0.15–0.32), so the parametrisation is not the cause of the overconfidence.

## Reading
1. **The heuristic (what M2 and M3 publish) is strongly underconfident:** position σ 0.31–0.40 m vs observed 0.04–0.065 m (×5–7.5), and yaw σ ≈ 56–71° vs observed 0.26–0.50° (×120–215). In effect the registrations give the backend almost no direct yaw information; yaw can only be corrected indirectly, through the position residuals over successive keyframes. This is consistent with the slow absorption of the −3.4° heading offset in the worst throughput-control run (descriptive link only).
2. **The Hessian (D4 option) is overconfident:** position σ 0.014–0.017 m, i.e. ×2.4–4.5 too small (NEES/3 = 14–67), and yaw ×1.2–2.1 too small. DF's Hessian is less overconfident than P2P's on every sequence (position ratio 0.26–0.41 vs 0.22–0.33; yaw 0.71–0.86 vs 0.48–0.50).
3. Position overconfidence is larger on V1_01 than on V1_02 / V2_02 (larger errors on V1_01, part of it the 4 cm offset).

## Proposal: third arm "calibrated Hessian" (overconfidence is large, so the rule applies)
Information = Hessian / k², **one factor k per method, fitted on V1_01 ONLY** (mean position NEES = 3 on V1_01), applied unchanged to V1_02 and V2_02:
| | k (k²) fitted on V1_01 | V1_01 NEES/3 | V1_02 NEES/3 | V2_02 NEES/3 | position ratio after (V1_01 / V1_02 / V2_02) | yaw ratio after (V1_01 / V1_02 / V2_02) |
|---|---|---|---|---|---|---|
| P2P | 8.17 (66.7) | 1.00 | 0.39 | 0.33 | 1.8 / 2.7 / 2.5 | 4.1 / 4.1 / 3.9 |
| DF | 6.55 (42.9) | 1.00 | 0.41 | 0.33 | 1.7 / 2.7 / 2.6 | 4.7 / 5.0 / 5.6 |

Transfer to the held-out sequences is imperfect: the V1_01 factor makes the Hessian somewhat **under**confident on V1_02 / V2_02 (NEES/3 ≈ 0.33–0.41), and one factor cannot fit position and yaw at once (the Hessian overstates position more than yaw), so yaw becomes underconfident by ×4–5.6. The calibrated Hessian is still far closer to consistent than either published convention (heuristic: ×5–7.5 position, ×120–215 yaw; Hessian: ×0.22–0.41 position). Note on the fit statistic: mean NEES is tail-sensitive; a robust median-based alternative (k² = median NEES / 2.37, the χ²₃ median) would give about half the factor on V1_01 (P2P 82/2.37 ≈ 35 vs 67; DF 49/2.37 ≈ 21 vs 43), i.e. a less inflated, more confident calibrated Hessian. The mean-based fit is the one proposed, as it is the definition stated before computing; the choice is yours.

**Implementation needed for step 2 (not done yet):** the M2-fast node has no Hessian / calibrated option; the M3 node has `df_information: hessian` but no calibration factor. Both would get a config option (`information: heuristic | hessian | hessian_calibrated`, with the per-method k² in the config), verified offline against this script, in a new frozen build.

## Calibrated Hessian: definition (decided by the user and written here 2026-10-25T17:45:23+05:30, BEFORE any calibration factor was computed and before any online run of the D4 ablation)
- **information_cal = S · H · S**, H = the Hessian information as published ([pos, rot] order), **S = diag(1/k_p · I3, 1/k_r · I3)**; the position block scales by 1/k_p², the rotation block by 1/k_r², the cross terms by 1/(k_p · k_r). Implied covariance: Σ_cal = S⁻¹ Σ S⁻¹, i.e. position std × k_p, rotation std × k_r.
- **k_p:** chosen so that the mean position NEES on V1_01 equals 3 (k_p² = mean NEES / 3 of the uncalibrated Hessian on V1_01). Fit statistic: the mean, as pre-committed; median-based factors are reported as an offline sensitivity note only, never as an online arm.
- **k_r:** chosen so that the median implied yaw σ on V1_01 equals the empirical per-KF yaw noise on V1_01 (KF-to-KF yaw jitter / √2, Phase 7 definition, median over runs); k_r = empirical yaw noise / median implied yaw σ (uncalibrated).
- One pair (k_p, k_r) **per method** (P2P, DF), fitted on **V1_01 ONLY** (the logged Phase 9 M2 keyframes, as in step 1); applied unchanged to V1_02 and V2_02. V1_01 is in-sample.
- **Rejected alternative:** a single factor for position and rotation (step 1 proposal above). Rejected because it cannot fit yaw (step 1: with the position-fitted single factor, yaw becomes underconfident by ×3.9–5.6). No online result of any D4 arm existed when this choice was made.
