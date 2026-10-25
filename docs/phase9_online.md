# Phase 9: online M1 / M2 / M3 on four Vicon sequences (2026-10-24/25)

**Protocol:** pre-registered in docs/PLAN.md ("Phase 9 pre-registration") before any run. Frozen build `e7ebf29` (`runs/phase9/build_freeze.txt`; checksums re-verified before each of the 120 runs, no difference). 10 runs per method per sequence, interleaved M1 → M2 → M3 per round, rate 1.0. Batch 19:20–00:42, all 120 runs completed by the harness (exit code 0). During runs the system was otherwise idle: total CPU busy 6.8 % median (max 13 %) of 12 cores, i.e. svo_node + registration node only. (The 1-min load average at run start, median 1.0, max 2.7, is the lagging average of the previous run's evaluation.)
Scripts: `analysis/run_phase9.sh`, `phase9_run_metrics.py`, `phase9_aggregate.py`, `phase9_figures.py`. Results: `runs/phase9/aggregate.json`, `table.md`, per-run `runs/2026-10-24_<seq>_<M>_p9_rNN/analysis/phase9_run.json`. Figure: `docs/img/phase9_ate.png`.

## 1. Failures (pre-registered reset rule)
| Sequence | M1 | M2 | M3 | Reset times (s from first image) |
|---|---|---|---|---|
| V1_01_easy | 0/10 | 0/10 | 0/10 | – |
| V1_02_medium | 2/10 | 0/10 | 1/10 | one reset at 51.4–51.5 s in each failed run |
| V2_02_medium | 0/10 | 0/10 | 1/10 | M3 r09: one reset at 39.8 s |
| V2_03_difficult | **9/10** | **8/10** | **5/10** | recurring at ~7.5 s and ~64.8 s, often followed by repeated resets until the end |

All failures are SVO full resets; no harness failure, no missing D1. The resets happen in all three methods at the same times, i.e. they come from the sequence and SVO's tracking (V2_03: 17.7 % of cam0 frames missing, docs/datasets.md), not from the registration. The single reset-free V2_03 run of the sequence survey (docs/sequence_survey.md, 2026-10-24) was not representative: it under-estimated V2_03's reset rate. Per the rule, failed runs are excluded and not repeated. Failed runs can have absurd ATEs (up to 10⁴ m: after a reset the VIO frame and scale are re-initialised); these are excluded and never averaged.

## 2. Main table (mean ± sample std over non-failed runs)
| Sequence | Method | valid runs | map-frame ATE [m] | 4-DoF ATE [m] | rel. disp. [m] | accepted / published | dropped by queue |
|---|---|---|---|---|---|---|---|
| V1_01_easy | M1 | 10 | 0.127 ± 0.015 | 0.053 ± 0.009 | 0.067 ± 0.010 | – | – |
| | M2 | 10 | 0.110 ± 0.016 | 0.078 ± 0.012 | 0.098 ± 0.016 | 181.4 / 185.7 | 0.4 |
| | M3 | 10 | 0.106 ± 0.012 | 0.075 ± 0.008 | 0.095 ± 0.012 | 185.7 / 186.1 | 0 |
| V1_02_medium | M1 | 8 | 0.136 ± 0.018 | 0.074 ± 0.006 | 0.098 ± 0.008 | – | – |
| | M2 | 10 | 0.069 ± 0.012 | 0.056 ± 0.005 | 0.075 ± 0.006 | 200.3 / 279.0 | 78.7 |
| | M3 | 9 | 0.066 ± 0.017 | 0.058 ± 0.011 | 0.075 ± 0.014 | 278.4 / 279.9 | 0 |
| V2_02_medium | M1 | 10 | 0.169 ± 0.035 | 0.100 ± 0.009 | 0.120 ± 0.009 | – | – |
| | M2 | 10 | 0.095 ± 0.011 | 0.081 ± 0.008 | 0.100 ± 0.009 | 270.6 / 344.9 | 63.8 |
| | M3 | 9 | 0.081 ± 0.006 | 0.074 ± 0.005 | 0.091 ± 0.007 | 331.2 / 344.2 | 0 |
| V2_03_difficult | M1 | 1 | 0.474 | 0.282 | 0.307 | – | – |
| | M2 | 2 | 0.822 ± 0.182 | 0.295 ± 0.080 | 0.302 ± 0.071 | **0 / 418.5** | 75.5 |
| | M3 | 5 | 0.383 ± 0.237 | 0.234 ± 0.096 | 0.262 ± 0.093 | 165.8 ± 136.9 / 417.2 | 0 |

M1 V1_01 on this build: map-frame 0.127 ± 0.015 m, 4-DoF 0.053 ± 0.009 m (historical Phase 4, n = 5: 0.132 ± 0.012 / 0.056 ± 0.003; consistent). Online D1 = offline D1 in all 80 M2/M3 runs (max |Δyaw| 2.9e-6°).

## 3. Primary test: M3 − M2, Welch, Holm across the 4 sequences
| Sequence | map-frame ATE diff [m] (95 % CI) | p | p Holm | 4-DoF ATE diff [m] (95 % CI) | p | p Holm |
|---|---|---|---|---|---|---|
| V1_01_easy | −0.0035 [−0.0171, +0.0101] | 0.60 | 1 | −0.0030 [−0.0128, +0.0067] | 0.52 | 1 |
| V1_02_medium | −0.0029 [−0.0171, +0.0114] | 0.67 | 1 | +0.0020 [−0.0068, +0.0109] | 0.62 | 1 |
| V2_02_medium | **−0.0141 [−0.0229, −0.0054]** | 0.0039 | **0.016** | −0.0061 [−0.0123, +0.0002] | 0.055 | 0.22 |
| V2_03_difficult | −0.439 [−1.033, +0.155] (n = 2 vs 5) | 0.094 | 0.28 | −0.061 [−0.334, +0.212] | 0.47 | 1 |

**Reading (report wording, approved 2026-10-25):** "M3 matched M2 within ±1.4 cm (95 % CI) on V1_01 and V1_02, and was 14 mm lower on V2_02 (Holm p = 0.016); attribution to the algorithm pending the throughput control." One of eight primary tests is significant after Holm. Note that on V1_02 M2 dropped even more clouds (28 %) than on V2_02 (18 %) yet matched M3, so dropped clouds alone do not determine the outcome. V2_03 is not interpretable (n = 2 vs 5, huge spread). No 4-DoF difference is significant. **Caveat for V2_02:** M2 dropped 18 % of the keyframe clouds there (queue_size 1, ~340 ms latency) while M3 processed all of them, so part of this difference may be implementation (throughput), not algorithm. This is exactly the case the queue-drop flag (§5) was pre-registered for.

## 4. Secondary (exploratory, uncorrected)
- **Digital twin vs VIO only (M1), pre-registered secondary:** map-frame ATE is lower with both M2 and M3 on V1_01 (−17 / −20 mm, p = 0.027 / 0.004), V1_02 (−67 / −70 mm, p ≤ 1e-6) and V2_02 (−73 / −87 mm, p ≤ 6e-5). The V1_02 and V2_02 effects (p ≤ 6e-5) are strong enough to survive any correction: Bonferroni over all 34 computed secondary tests gives 0.05/34 = 0.0015; the V1_01 effects are weaker (p = 0.027 would not survive). On V1_01, the 4-DoF ATE and relative displacement are **worse** with both registrations (+22–31 mm, p ≤ 1e-4), as in Phase 6 (the registration corrects the global frame but adds local jitter on an easy sequence where VIO drift is already small). On V1_02 and V2_02 all three metrics improve with both M2 and M3. V2_03: not computable (M1 has 1 valid run).
- **M3 vs M2, relative displacement:** equal on V1_01 and V1_02; −8.8 mm on V2_02 (p = 0.029).
- **Registration counts:** M3 accepts and adds more registrations on every sequence (V1_01 +4, V1_02 +78, V2_02 +61 per run, p ≤ 0.009), almost entirely because M2 drops clouds at the queue (M2 drops 0.4 / 78.7 / 63.8 per run; M3 0). Rejections are rare for both on V1/V2_02.
- **V2_03 (descriptive only):** in M2's 2 valid runs, **every** registration was rejected (0/342 and 0/344; rotation > 5° or translation + rotation, and RMSE > 0.15 m), so M2 was effectively VIO-only there. M3 accepted 0–374 of ~418 per valid run (median 129), also mostly rejected for rotation > 5°. The 5° / 0.75 m limits were set from V1 data and were not re-tuned (pre-registered); on V2_03 the VIO yaw error after the D1 window apparently exceeds them. Not a statistical comparison.

## 5. System metrics and the queue-drop flag
| Sequence / method | callback per KF [ms] (median) | latency cloud → svo [ms] (median) | registration load | CPU reg. node [% of one core] | CPU svo_node [%] |
|---|---|---|---|---|---|
| V1_01 M2 / M3 | 294 / 34.9 | 298 / 36.7 | 0.38 / 0.05 | 41 / 6 | 73 / 72 |
| V1_02 M2 / M3 | 298 / 38.1 | 369 / 39.9 | 0.68 / 0.12 | 75 / 16 | 76 / 73 |
| V2_02 M2 / M3 | 292 / 34.9 | 341 / 36.7 | 0.70 / 0.10 | 76 / 13 | 66 / 64 |
| V2_03 M2 / M3 | 265 / 8.3¹ | – / 31.8 | 0.79 / 0.05 | 87 / 8 | 48 / 44 |

¹ Rejected KFs skip the information computation, so the median callback is low when most are rejected. Registration load = Σ callback time / bag duration. Playback RTF 1.02–1.10 (rosbag start-up), frontend RTF 0.08–0.19, SVO frame drops ≤ 0.13 % for all methods.

**Queue-drop flag (pre-registered: M2 drop fraction > 2× M3's and ≥ 5 %): raised on V1_02 (28 %), V2_02 (18 %) and V2_03 (18 %); not on V1_01 (0.2 %).** V1_02 and V2_02 produce a keyframe cloud every 0.20 s (median; V1_01: 0.50 s), faster than M2's ~300 ms callback, so M2 drops about one cloud in four to five (queue_size 1); M3's ~35 ms callback keeps up. Per the pre-registration this **triggers the Phase 10 variant "M2 with a persistent KD-tree"** to separate implementation (Open3D rebuilding the KD-tree of the whole twin per call, Phase 8) from algorithm (point-to-plane vs distance field).

## 6. Hypotheses after Phase 9 (online, closed loop)
- **H3 (accuracy, map frame):** "M3 matched M2 within ±1.4 cm (95 % CI) on V1_01 and V1_02, and was 14 mm lower on V2_02 (Holm p = 0.016); attribution to the algorithm pending the throughput control." V2_03 not testable. Discussion: on V1_01 both registrations cost local consistency (4-DoF ATE and relative displacement worse than VIO only), as in Phase 6.
- **H2 (speed), online:** callback 35–40 ms vs 290–300 ms, latency 37–40 ms vs 300–370 ms, registration-node CPU 6–16 % vs 41–87 % of one core; consistent with Phase 8 (implementation-inclusive numbers).
- Both digital-twin methods reduce map-frame error relative to VIO only on V1_01, V1_02, V2_02 (exploratory), with a local-accuracy cost on V1_01.

## 7. What this does not show
- **V2_03 is a limitation alongside V1_03, reported descriptively only:** failure counts 9 / 8 / 5 of 10 (M1 / M2 / M3) with the same reset times for all methods; no claim is made from these counts (n = 10 is too small, and the resets come from SVO tracking on this sequence). The acceptance limits (5°, 0.75 m) were derived on V1 and not retuned, so M2 effectively ran VIO-only on V2_03 (0 accepted registrations in its 2 valid runs).
- The V2_02 M3 advantage cannot yet be attributed to the distance field rather than to M2's dropped clouds (Phase 10 variant).
- Thresholds and information weights were not tuned for V2 or for M3 (by design); a different 5° rotation limit would change V2_03 acceptance for both methods.
