# Phase 10, experiment 1: throughput control, M2-fast vs M3 (2026-10-25)

**Question:** is M3's lower map-frame ATE on V2_02 in Phase 9 (−14 mm, Holm p = 0.016) due to the distance field, or to M2 dropping 18 % of the keyframe clouds (Open3D rebuilds the twin's KD-tree on every call)?
**Protocol:** pre-registered in docs/PLAN.md ("Phase 10 throughput control pre-registration"), with the three-way interpretation rule amended at 2026-10-25T14:52:42+05:30, during the batch but before any of its results were read (the amendment is recorded there). Frozen build `60eafa5` (`runs/phase10_tc/build_freeze.txt`; all files shared with the Phase 9 build are byte-identical). 10 runs per method per sequence, per round M2F → M3, rate 1.0. Batch 13:27–15:02, 40/40 runs completed by the harness.
**M2-fast (M2F):** M2 with a KD-tree of the twin built once (`online_registration_m2fast.py`, `p2p_icp_persistent.py`). Offline it reproduces every logged online M2 output on V1_01, V1_02 and V2_02 to ≤ 1e-14 (`runs/phase10_tc/m2fast_equivalence.json`).
Results: `runs/phase10_tc/aggregate.json`, `table.md`; runs `runs/2026-10-25_<seq>_<M2F|M3>_p10tc_rNN/`.

## 1. Throughput: the control worked
| Sequence / method | runs (failed) | published | dropped by queue | accepted | callback [ms] (median) | latency cloud → svo [ms] | CPU reg. node [% of one core] |
|---|---|---|---|---|---|---|---|
| V1_02 M2F | 10 (0) | 279.6 | **0** | 277.6 | 84.5 | 86.4 | 31 |
| V1_02 M3 | 10 (1) | 278.8 | 0 | 277.8 | 37.1 | 38.9 | 15 |
| V2_02 M2F | 10 (1) | 343.1 | **0** | 327.6 | 80.9 | 83.0 | 27 |
| V2_02 M3 | 10 (0) | 343.7 | 0 | 330.0 | 34.5 | 36.4 | 13 |

(Phase 9, M2: dropped 78.7 / 63.8 per run on V1_02 / V2_02, latency 369 / 341 ms.) M2F dropped no cloud in any run, so both methods now process the same keyframes; accepted counts are equal within ~2. The crop fallback (`tree_rebuilt`) never triggered. Failures (reset rule): V1_02 M3 r06 (reset at 51.3 s, the same time as the Phase 9 V1_02 resets), V2_02 M2F r10 (reset at 4.3 s); excluded, not repeated.

## 2. Primary result (map-frame ATE; mean ± sample std over valid runs)
| Sequence | M2F [m] | M3 [m] | M3 − M2F [m] (95 % CI) | p (Welch) | p (Holm, 2 seq.) |
|---|---|---|---|---|---|
| V1_02_medium | 0.067 ± 0.012 (n = 10) | 0.064 ± 0.015 (n = 9) | −0.0038 [−0.0171, +0.0096] | 0.56 | 0.56 |
| V2_02_medium | 0.094 ± 0.024 (n = 9) | 0.074 ± 0.004 (n = 10) | −0.0199 [−0.0382, −0.0016] | 0.036 | 0.073 |

**Applying the pre-registered three-way rule** (d = M2F − M3; its CI is the negated CI above):
- **V2_02 (primary):** rule 1 (algorithm supported) needs M3 significantly better after Holm: p_Holm = 0.073, **not met**. Rule 2 (attributed to throughput) needs the upper bound of d's 95 % CI below 0.014 m: d CI = [+0.0016, +0.0382], upper bound 0.038, **not met**. **Outcome: inconclusive.**
- **V1_02 (reported the same way):** p_Holm = 0.56 (rule 1 not met); d CI = [−0.0096, +0.0171], upper bound 0.017 ≥ 0.014 (rule 2 not met). **Outcome: inconclusive.**

Secondary (4-DoF ATE, same test, labelled secondary): V1_02 −0.0026 [−0.0107, +0.0055] m (p_Holm 0.51); V2_02 −0.0112 [−0.0216, −0.0007] m (p 0.039, p_Holm 0.077).

## 3. Descriptive observations (no tests, not pre-registered)
- With the drops removed, M2F's V2_02 mean (0.094 m) is essentially the Phase 9 M2 mean (0.095 m), so removing the dropped clouds did not visibly improve the point-to-plane result; on V1_02 neither condition differed from M3 (Phase 9 M2 0.069 with 28 % drops, M2F 0.067 without). Cross-build comparisons are descriptive only (pre-registered).
- On V2_02, M2F's run-to-run spread is much larger than M3's (std 0.024 vs 0.004 m; M2F runs range 0.062–0.143 m, M3 0.067–0.079 m). M3 was similarly consistent in Phase 9 (0.081 ± 0.006). The point estimate still favours M3 by 20 mm, but with n = 9–10 and this unequal spread the difference does not survive Holm. A test of the variances was not pre-registered and is not claimed.
- Speed: M2F's callback (81–85 ms) is 3.5× shorter than M2's but still ~2.3× M3's (35–37 ms); the remaining M2F cost is the Python ICP loop plus M2's information heuristic (~31 ms, shared with M3).

## 4. What this means for H3
Phase 9 wording stays: "M3 matched M2 within ±1.4 cm (95 % CI) on V1_01 and V1_02, and was 14 mm lower on V2_02 (Holm p = 0.016); attribution to the algorithm pending the throughput control." **Throughput control outcome: inconclusive on V2_02** (and V1_02): with throughput equalised, M3 is still lower on average (−20 mm, unadjusted 95 % CI excludes 0) but not significantly after Holm, and the data cannot rule out an effect as large as Phase 9's. Neither attribution (algorithm or throughput) is supported by the pre-registered rule.
