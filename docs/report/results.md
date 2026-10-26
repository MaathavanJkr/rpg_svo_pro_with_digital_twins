# Chapter 4: Results (draft, Phases 9–10)

*Draft generated from the project documents (2026-10-26). Every number is taken from the file named in brackets; tables there are the authoritative source. All online experiments were pre-registered (Chapter 3, §3.12); "significant" means significant after the pre-registered correction. Offline results (Phases 7–8) are summarised in Chapter 3 and only referred to here.*

## 4.1 Overview of the online experiments

| Experiment | Build | Sequences | Methods / arms | Runs | Failures (resets) | Source |
|---|---|---|---|---|---|---|
| Phase 9 main comparison | e7ebf29 | V1_01, V1_02, V2_02, V2_03 | M1, M2, M3 | 120 | 26 (22 on V2_03) | `runs/phase9/aggregate.json`, docs/phase9_online.md |
| Throughput control | 60eafa5 | V1_02, V2_02 | M2F, M3 | 40 | 2 | `runs/phase10_tc/aggregate.json`, docs/phase10_throughput.md |
| D4 weighting ablation | 0ed4c47 | V1_01, V1_02, V2_02 | {M2F, M3} × {heuristic, Hessian, calibrated Hessian} | 180 | 16 (12 in raw-Hessian arms) | `runs/phase10_d4/aggregate.json`, docs/phase10_d4_ablation.md |
| V2_02 replication | 0ed4c47 | V2_02 | M2F, M3 (heuristic) | 40 | 0 | `runs/phase10_rep/aggregate.json`, docs/phase10_replication.md |
| Initial-alignment robustness | 47b77cf | V1_02, V2_02 | — | stopped after 1 run | — | docs/PLAN.md (§4.6) |

M2F ("M2-fast") is the original point-to-plane registration with a KD-tree of the twin built once instead of per keyframe; offline it reproduces every logged online M2 output to 1e-14 (`runs/phase10_tc/m2fast_equivalence.json`). All runs were played at rate 1.0 on an otherwise idle machine; during the Phase 9 runs total CPU use was 6.8 % median of 12 cores (docs/phase9_online.md).

## 4.2 Main comparison (Phase 9)

**Failures.** SVO full resets occurred on V1_02 (M1 2/10, M3 1/10, at ~51.4 s), V2_02 (M3 1/10) and V2_03 (M1 9/10, M2 8/10, M3 5/10). On V2_03 the resets occurred at the same times (~7.5 s and ~64.8 s) for all three methods, i.e. they come from SVO's tracking on this sequence (17.7 % of its camera frames are missing), not from the registration. V2_03 is therefore reported descriptively only, as a limitation together with V1_03. (docs/phase9_online.md §1)

**Table 4.1** Map-frame and 4-DoF ATE, mean ± sample std over valid runs (docs/phase9_online.md §2).
| Sequence | Method | valid runs | map-frame ATE [m] | 4-DoF ATE [m] | relative displacement [m] |
|---|---|---|---|---|---|
| V1_01 | M1 | 10 | 0.127 ± 0.015 | 0.053 ± 0.009 | 0.067 ± 0.010 |
| | M2 | 10 | 0.110 ± 0.016 | 0.078 ± 0.012 | 0.098 ± 0.016 |
| | M3 | 10 | 0.106 ± 0.012 | 0.075 ± 0.008 | 0.095 ± 0.012 |
| V1_02 | M1 | 8 | 0.136 ± 0.018 | 0.074 ± 0.006 | 0.098 ± 0.008 |
| | M2 | 10 | 0.069 ± 0.012 | 0.056 ± 0.005 | 0.075 ± 0.006 |
| | M3 | 9 | 0.066 ± 0.017 | 0.058 ± 0.011 | 0.075 ± 0.014 |
| V2_02 | M1 | 10 | 0.169 ± 0.035 | 0.100 ± 0.009 | 0.120 ± 0.009 |
| | M2 | 10 | 0.095 ± 0.011 | 0.081 ± 0.008 | 0.100 ± 0.009 |
| | M3 | 9 | 0.081 ± 0.006 | 0.074 ± 0.005 | 0.091 ± 0.007 |

**Primary tests (M3 − M2, Welch, Holm over the four sequences).** Map-frame ATE: V1_01 −3.5 mm [−17.1, +10.1] (p_Holm 1); V1_02 −2.9 mm [−17.1, +11.4] (p_Holm 1); **V2_02 −14.1 mm [−22.9, −5.4] (p 0.0039, p_Holm 0.016)**; V2_03 not interpretable (2 vs 5 valid runs). No 4-DoF difference was significant. (docs/phase9_online.md §3)

**Digital twin versus VIO only (pre-registered secondary).** Both registrations lowered the map-frame ATE relative to M1 on all three sequences with valid comparisons: V1_01 −17 / −20 mm (M2 / M3; p 0.027 / 0.004), V1_02 −67 / −70 mm (p ≤ 1e-6), V2_02 −73 / −87 mm (p ≤ 6e-5). The V1_02 and V2_02 effects are below the Bonferroni threshold over all 34 secondary tests (0.0015). On V1_01, both registrations made local consistency worse: 4-DoF ATE and relative displacement increased by 22–31 mm (p ≤ 1e-4), as in Phase 6. (docs/phase9_online.md §4)

**Runtime online.** Callback per keyframe 290–300 ms (M2) vs 35–40 ms (M3); latency from cloud to backend 300–370 ms vs 37–40 ms; registration-node CPU 41–87 % vs 6–16 % of one core. On V1_02, V2_02 and V2_03, where a keyframe cloud arrives every ~0.20 s, M2 dropped 18–28 % of the clouds at its queue of length one, M3 none; this raised the pre-registered queue-drop flag. (docs/phase9_online.md §5)

## 4.3 Throughput control

To separate the algorithm from M2's implementation (the per-keyframe KD-tree rebuild that made M2 drop clouds), M2F was compared with M3 on V1_02 and V2_02 (10 runs each). M2F dropped no cloud (latency 83–86 ms vs 36–39 ms for M3). Map-frame ATE, M3 − M2F: V1_02 −3.8 mm [−17.1, +9.6] (p_Holm 0.56); V2_02 −19.9 mm [−38.2, −1.6] (p 0.036, p_Holm 0.073). Under the pre-registered three-way rule both outcomes are **inconclusive**: M3 was not significantly better after Holm, and the confidence interval did not exclude the Phase 9 effect size. With throughput equalised, point-to-plane's mean on V2_02 was unchanged (0.094 vs 0.095 m in Phase 9). Exploratory: on V2_02, M2F varied about five times more between runs than M3 (std 0.024 vs 0.0045 m, Brown–Forsythe p 0.024, not pre-registered); the worst M2F run started with a consistent ~3.4° heading offset that its registrations detected and the backend absorbed only gradually. (docs/phase10_throughput.md)

## 4.4 Confirmatory replication on V2_02

A pre-registered replication with a single primary test (20 runs per method, M2F vs M3, heuristic weighting, no failures, no drops) gave **M3 − M2F = −9.9 mm, 95 % CI [−19.7, −0.1] mm, Welch p = 0.047** (0.082 ± 0.014 vs 0.092 ± 0.016 m), about 11 % of point-to-plane's error. The 4-DoF difference was −4.2 mm [−9.8, +1.4] (p 0.13). The run-to-run spread was similar for both methods (0.014 vs 0.016 m), so the large spread difference of the throughput control did not recur. (docs/phase10_replication.md)

**Table 4.2** All batches on V2_02, map-frame ATE (reported together; not pooled for inference).
| Batch | Role | point-to-plane [m] | M3 [m] | M3 − P2P [mm] (95 % CI) | p |
|---|---|---|---|---|---|
| Phase 9 (M2) | primary | 0.095 ± 0.011 | 0.081 ± 0.006 | −14.1 [−22.9, −5.4] | p_Holm 0.016 |
| Throughput control (M2F) | primary | 0.094 ± 0.024 | 0.074 ± 0.004 | −19.9 [−38.2, −1.6] | p_Holm 0.073 |
| D4 heuristic arms (M2F) | secondary | 0.091 ± 0.010 | 0.079 ± 0.008 | −12.6 [−20.9, −4.2] | 0.006 (uncorrected) |
| Replication (M2F) | confirmatory | 0.092 ± 0.016 | 0.082 ± 0.014 | −9.9 [−19.7, −0.1] | 0.047 |

## 4.5 Information weighting (D4)

**Offline realism check.** On the 6 658 logged keyframes of the Phase 9 M2 runs, the heuristic weighting that M2 and M3 publish implied position uncertainties of 0.31–0.40 m per axis against observed registration errors of 0.04–0.065 m (5–7.5× too large) and yaw uncertainties of 56–71° against observed per-keyframe yaw noise of 0.26–0.50° (120–215× too large). The Hessian-based information was overconfident (position 0.22–0.41 of the observed error, NEES/3 = 14–67; yaw 0.48–0.86). A two-factor calibration (position and rotation) fitted on V1_01 only brought held-out yaw to 0.96–1.20 of the observed noise and held-out position NEES/3 to 0.33–0.46. (docs/phase10_d4_realism.md)

**Online ablation.** Replacing the heuristic with the calibrated Hessian made both methods markedly worse in all eight pre-registered tests (family A): map-frame and 4-DoF ATE increased by 85–156 mm (e.g. V2_02 map-frame M3 0.079 → 0.172 m, M2F 0.091 → 0.176 m; Holm p ≤ 2e-4). The raw Hessian diverged (map-frame ATE 0.6–7.5 m; 12 of the 16 resets). Under the calibrated Hessian, M3 and M2F did not differ (family B: V1_02 +22 mm [−17, +61], V2_02 −4 mm [−38, +31]; no claim). Accepted registrations beyond 3σ of their own published covariance: 0 % under the heuristic, 3–24 % under the calibrated Hessian, 95–99 % under the raw Hessian. (docs/phase10_d4_ablation.md §1–3)

**Exploratory reading.** With good inputs (open loop) only ~2 % of accepted registrations are more than 0.2 m off; in every calibrated run the first wrong accepted registration preceded the growth of the map-frame error by a median of 11–40 s, after which wrong registrations became more frequent (7–26 % before, 12–48 % after). This is consistent with a feedback loop in which strongly weighted wrong-but-accepted registrations move the state and degrade later initial guesses; a residual convention error is not excluded. The claim is scoped: open-loop calibration, no robust outlier handling, backend Huber loss unchanged. (docs/phase10_d4_ablation.md §3–4)

## 4.6 Initial-alignment robustness (not completed)

The pre-registered online test of H1 (injected heading offsets of 10° and 20°, recovery through the original one-time re-alignment `refine_T_WL`) was stopped after its technical smoke runs and the first batch run. The re-alignment criterion of the original code (yaw variance below 3e-4 rad², computed from the geometry of the keyframe positions only) cannot be met within a room-scale sequence: replaying it on real keyframe geometry gave final variances of 6.8e-4 to 1.8e-3 (V1_01, V1_02, V2_02). The mechanism was designed for city-scale trajectories. No result of this experiment is used. (docs/PLAN.md, "BATCH STOPPED") *[To be updated after the redesign decision.]*

## 4.7 Hypotheses: status

| Hypothesis | Evidence | Status |
|---|---|---|
| **H1** larger convergence basin | Offline, open-loop (Phase 7): larger basin for yaw (20°: success 0.96–0.97 vs 0.73–0.83, floor-corrected), comparable for translation and combined perturbations; DF more stable overall. Online test not completed (§4.6). | supported for yaw, **offline only** |
| **H2** faster | Offline, single core (Phase 8): ~10× per iteration (point-to-plane figure indirect), ~4–5× per solve without M2's per-call KD-tree rebuild (135× with it, a property of M2's implementation). Online: callback 35–40 vs 290–300 ms (M2) and 81–85 ms (M2F). | supported |
| **H3** equal or lower map-frame error | No difference on V1_01 and V1_02 (CIs about ±1.5 cm); a small, replicated improvement on V2_02 (−9.9 mm, 95 % CI [−19.7, −0.1], pre-registered confirmatory test; three earlier batches −13 to −20 mm). | supported ("equal or lower"); lower only on V2_02, small |
| **H4** memory higher, grows with volume × (1/v)³ | Room scale: DF grid 23 MB (v = 0.05 m) vs point-to-plane twin + KD-tree ~36 MB; grids 0.39 / 3.0 / 23.4 MB at v = 0.20 / 0.10 / 0.05 m. City-scale extrapolation pending (Phase 10). | not supported at room scale; scaling confirmed |

## 4.8 Points for the discussion chapter (not results)
- The digital twin reduces map-frame error relative to VIO only on every valid sequence, but on the easiest sequence (V1_01) it costs local consistency.
- The original heuristic weighting is far from calibrated, yet it acts as implicit outlier protection in this closed loop; better-calibrated weights need robust gating first.
- The validity checks accept some wrong registrations with fitness ≈ 1.
- Two components of the original system are scale-dependent in ways that matter indoors: the per-keyframe KD-tree rebuild (throughput) and the re-alignment criterion (never fires in a room).
