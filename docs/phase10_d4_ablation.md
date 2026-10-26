# Phase 10, experiment 2: D4 information-weighting ablation (online, 2026-10-25/26)

**Protocol:** pre-registered in docs/PLAN.md ("Phase 10 D4 ablation pre-registration"), with the dated deviation of 2026-10-25T20:49:46 (body-centred Hessian for both Hessian arms, factors refitted on V1_01; recorded before any change, together with the only online outcomes seen at that time: two smoke runs) and the smoke2 classification (M2F-cal reset at 51.4 s = known V1_02 reset), recorded before the batch. Frozen build `0ed4c47` (freeze record `64da2e2`). 6 arms {M2F, M3} × {heuristic, hessian, hessian_calibrated} × 3 sequences × 10 runs = 180 runs, interleaved per round, rate 1.0; batch 21:25–05:34, 180/180 completed by the harness. Factors (V1_01 only): P2P k_p 10.04, k_r 1.99; DF k_p 8.18, k_r 1.40.
Results: `runs/phase10_d4/aggregate.json`, `table.md`, `outliers.json`, `mechanism.json`; runs `runs/2026-10-25_<seq>_<M2F|M3>-<heur|hess|cal>_p10d4_rNN/`.

## 1. Main table (mean ± sample std over valid runs; failures = SVO full resets)
| Sequence | Arm | valid (failed) | map-frame ATE [m] | 4-DoF ATE [m] | accepted / published | rejected |
|---|---|---|---|---|---|---|
| V1_01 (in-sample, descriptive) | M2F-heur | 10 (0) | 0.114 ± 0.012 | 0.082 ± 0.009 | 181 / 184 | 3 |
| | M2F-hess | 9 (1) | 0.709 ± 0.325 | 0.561 ± 0.168 | 74 / 191 | 118 |
| | M2F-cal | 10 (0) | 0.281 ± 0.110 | 0.244 ± 0.084 | 162 / 188 | 26 |
| | M3-heur | 10 (0) | 0.112 ± 0.013 | 0.079 ± 0.009 | 184 / 184 | 0 |
| | M3-hess | 10 (0) | 0.829 ± 0.425 | 0.560 ± 0.294 | 58 / 194 | 136 |
| | M3-cal | 10 (0) | 0.260 ± 0.087 | 0.231 ± 0.075 | 166 / 186 | 20 |
| V1_02 | M2F-heur | 9 (1) | 0.063 ± 0.008 | 0.055 ± 0.005 | 278 / 279 | 1 |
| | M2F-hess | 10 (0) | 7.48 ± 20.5 | 6.47 ± 18.3 | 61 / 291 | 230 |
| | M2F-cal | 9 (1) | 0.203 ± 0.026 | 0.176 ± 0.018 | 240 / 283 | 43 |
| | M3-heur | 9 (1) | 0.070 ± 0.013 | 0.058 ± 0.011 | 278 / 279 | 2 |
| | M3-hess | 8 (2) | 7.31 ± 15.0 | 5.96 ± 13.6 | 33 / 291 | 258 |
| | M3-cal | 9 (1) | 0.225 ± 0.048 | 0.204 ± 0.043 | 220 / 281 | 60 |
| V2_02 | M2F-heur | 10 (0) | 0.091 ± 0.010 | 0.078 ± 0.007 | 327 / 345 | 17 |
| | M2F-hess | 6 (4) | 1.58 ± 1.71 | 0.96 ± 0.91 | 60 / 357 | 295 |
| | M2F-cal | 10 (0) | 0.176 ± 0.018 | 0.163 ± 0.017 | 304 / 346 | 42 |
| | M3-heur | 10 (0) | 0.079 ± 0.008 | 0.073 ± 0.004 | 332 / 344 | 12 |
| | M3-hess | 5 (5) | 0.624 ± 0.382 | 0.401 ± 0.153 | 128 / 359 | 230 |
| | M3-cal | 10 (0) | 0.172 ± 0.046 | 0.157 ± 0.042 | 292 / 346 | 54 |
Failures: 16/180, **12 of them in the raw Hessian arms** (V1_01 M2F-hess 1, V1_02 M3-hess 2, V2_02 M2F-hess 4, M3-hess 5); the other 4 are the known V1_02 reset at ~51 s (M2F-heur, M2F-cal, M3-heur, M3-cal, one each). No drops for any arm except M2F-hess (0.4 / 2.0 per run on V1_02 / V2_02).

## 2. Primary results
**Family A (Holm over 8): hessian_calibrated vs heuristic** — diff = cal − heur:
| Method, sequence, metric | diff [m] (95 % CI) | p (Holm) | outcome |
|---|---|---|---|
| M2F, V1_02, 4-DoF | +0.121 [+0.106, +0.135] | 7e-8 | supported: **calibrated worse** |
| M2F, V1_02, map-frame | +0.140 [+0.120, +0.161] | 2e-7 | supported: calibrated worse |
| M2F, V2_02, 4-DoF | +0.085 [+0.072, +0.098] | 3e-8 | supported: calibrated worse |
| M2F, V2_02, map-frame | +0.085 [+0.071, +0.099] | 3e-8 | supported: calibrated worse |
| M3, V1_02, 4-DoF | +0.146 [+0.112, +0.179] | 2e-5 | supported: calibrated worse |
| M3, V1_02, map-frame | +0.156 [+0.118, +0.193] | 2e-5 | supported: calibrated worse |
| M3, V2_02, 4-DoF | +0.084 [+0.054, +0.114] | 2e-4 | supported: calibrated worse |
| M3, V2_02, map-frame | +0.094 [+0.061, +0.127] | 2e-4 | supported: calibrated worse |

**Family B (Holm over 2): M3 vs M2F under hessian_calibrated, map-frame ATE** — diff = M3 − M2F: V1_02 +0.022 [−0.017, +0.061] m (p_Holm 0.50); V2_02 −0.004 [−0.038, +0.031] m (p_Holm 0.81). Not significant; no claim.

## 3. Secondary / descriptive
- **Raw Hessian arms** fail badly everywhere (map-frame 0.6–7.5 m, most registrations rejected, 12 of the 16 resets).
- **V1_01 (in-sample):** calibrated worse than heuristic as well (+0.148 / +0.167 m map-frame for M3 / M2F, p < 0.001, uncorrected).
- **Heuristic, M3 vs M2F (secondary, uncorrected):** V1_01 −0.002 m (p 0.74), V1_02 +0.007 m (p 0.21), **V2_02 −0.0126 m [−0.0209, −0.0042], p 0.006**. Third batch with M3 lower on V2_02 under the heuristic (Phase 9 −14 mm vs M2, throughput control −20 mm vs M2F); secondary here, not confirmatory.
- **Outlier counts (pre-registered descriptive; accepted registrations):**
| Sequence | heuristic (M2F / M3): > 3σ, \|e\| > 0.2 m | calibrated (M2F / M3): > 3σ, \|e\| > 0.2 m | raw Hessian (M2F / M3): > 3σ |
|---|---|---|---|
| V1_01 | 0 % / 0 %, 6.5 % / 6.9 % | 23 % / 24 %, 32 % / 25 % | 97 % / 98 % |
| V1_02 | 0 % / 0 %, 4.2 % / 3.6 % | 15 % / 20 %, 24 % / 20 % | 99 % / 99 % |
| V2_02 | 0 % / 0 %, 2.0 % / 0.9 % | 2.6 % / 6.2 %, 11 % / 13 % | 95 % / 98 % |
- **Cause or consequence (EXPLORATORY, `analysis/phase10_d4_mechanism.py`):** open loop (step 1, registrations initialised from a good VIO estimate) only 1.4–2.5 % of accepted registrations are > 0.2 m off on V1_02 / V2_02 (6–7 % on V1_01), close to the online heuristic arms. In every valid calibrated run the map-frame error eventually exceeds 0.2 m (median 18–38 s after the D1 window on V1_02 / V2_02); the first wrong accepted registration precedes it by a median of 11–40 s, the fraction of wrong accepted registrations is 7–26 % before and 12–48 % after that point. Reading (descriptive): consistent with a feedback loop. Wrong registrations occur at about the base rate under any weighting; under the calibrated weights they move the state, later registrations start from worse initial guesses, and errors compound. Under the heuristic's weak weights single wrong registrations barely move the estimate. The data do not separate a uniform convention error from this mechanism with certainty, but the 0 % > 3σ rate of the heuristic and the growth of wrong registrations over time in the calibrated runs point to the second. The validity checks accept these wrong solutions (limitation, docs/report_notes.md), and the backend Huber loss was not retuned (by design).
- **Calibration realism online vs offline:** the factors were fitted on open-loop errors (offline NEES/3 = 1 on V1_01, 0.33–0.46 held out), but online 3–24 % of accepted calibrated registrations lie beyond 3σ, far above the ~1 % expected: the online errors are larger than the open-loop errors the calibration saw.
- Speed: M3 callback 3 ms (Hessian arms: no heuristic loop) to 34–37 ms (heuristic); M2F 52–84 ms.

## 4. Report wording (approved 2026-10-26)
- **Family A:** "Replacing the heuristic with a Hessian-based information matrix calibrated on open-loop errors made both methods markedly worse (+85 to +156 mm, Holm p ≤ 2e-4); the raw Hessian diverged. In this closed loop, the heuristic's underconfidence acts as implicit outlier protection." **Scope:** the calibration was fitted on open-loop errors (logged keyframes, registrations initialised from a good VIO estimate); no robust outlier handling (no Mahalanobis gating or switchable constraints) was used; the backend's HuberLoss(50) was left unchanged (set for the heuristic's weak weights). The claim is limited to this configuration.
- **Family B:** CIs only, no claim (V1_02 +0.022 [−0.017, +0.061] m; V2_02 −0.004 [−0.038, +0.031] m).
- **Discussion (exploratory, labelled):** feedback-loop reading. Evidence: 0 % of accepted registrations beyond 3σ under the heuristic; in every valid calibrated run the first wrong accepted registration precedes the map-frame error growth (median 11–40 s), after which wrong registrations become more frequent (7–26 % before, 12–48 % after); the open-loop base rate of > 0.2 m registrations is ~2 % (1.4–2.5 % on V1_02 / V2_02). Alternative explanation not excluded: a residual convention error in how the backend uses the information. Online, 3–24 % of accepted calibrated registrations lie beyond 3σ, far above the calibration's ~1 %: closed-loop errors are larger than the open-loop errors the calibration was fitted on.
- **Future work:** robust gating of registrations (Mahalanobis gating, switchable constraints) and a backend robust loss matched to calibrated weights, before any re-test of Hessian-based weighting.
- The D4 decision (M3 publishes the heuristic convention for the main comparison) is supported by these results.
