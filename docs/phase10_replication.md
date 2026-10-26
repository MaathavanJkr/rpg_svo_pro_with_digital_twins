# Phase 10, experiment 3: confirmatory V2_02 replication, M2-fast vs M3, heuristic (2026-10-26)

**Protocol:** pre-registered in docs/PLAN.md ("Phase 10 V2_02 replication pre-registration"), written before any run, after three earlier batches had shown the same direction. Frozen build 0ed4c47 (the D4 build, unchanged; freeze record `runs/phase10_rep/build_freeze.txt` = the D4 list + the driver). V2_02_medium only; M2F-heur vs M3-heur; 20 runs each, interleaved per round; rate 1.0. Batch 13:15–15:02, 40/40 runs completed, **0 failures**. One primary test: map-frame ATE, Welch two-sided, α = 0.05, with the 95 % CI. Results: `runs/phase10_rep/aggregate.json`, `table.md` (per-run folders local, repository policy).

## Primary result
| | M2F (point-to-plane, persistent KD-tree) | M3 (distance field) |
|---|---|---|
| map-frame ATE [m], mean ± std (n) | 0.092 ± 0.016 (20) | 0.082 ± 0.014 (20) |

**M3 − M2F = −0.0099 m, 95 % CI [−0.0197, −0.0001] m, Welch p = 0.047 (df 37).** Under the pre-registered rule this is significant at α = 0.05: on V2_02, M3's map-frame ATE is lower than point-to-plane's (with throughput equalised; neither method dropped a cloud). The result is marginal: the CI's upper bound is 0.1 mm from zero and p is just below 0.05, so the size of the effect is uncertain (anything from ~0 to ~20 mm is compatible with the data).

## All batches on this question (reported alongside, as pre-registered)
| Batch | Comparison | Role | n per method | point-to-plane [m] | M3 [m] | M3 − P2P [m] (95 % CI) | p |
|---|---|---|---|---|---|---|---|
| Phase 9 (build e7ebf29) | M2 vs M3 | primary (Holm over 4 sequences) | 10 / 10 | 0.095 ± 0.011 | 0.081 ± 0.006 | −0.0141 [−0.0229, −0.0054] | 0.0039 (Holm 0.016) |
| Throughput control (60eafa5) | M2F vs M3 | primary (Holm over 2) | 9 / 10 | 0.094 ± 0.024 | 0.074 ± 0.004 | −0.0199 [−0.0382, −0.0016] | 0.036 (Holm 0.073) |
| D4 ablation, heuristic arms (0ed4c47) | M2F vs M3 | secondary | 10 / 10 | 0.091 ± 0.010 | 0.079 ± 0.008 | −0.0126 [−0.0209, −0.0042] | 0.006 (uncorrected) |
| **Replication (0ed4c47)** | **M2F vs M3** | **confirmatory, single test** | **20 / 20** | **0.092 ± 0.016** | **0.082 ± 0.014** | **−0.0099 [−0.0197, −0.0001]** | **0.047** |

All four batches point the same way; the confirmatory estimate (−10 mm) is the smallest of the four, as expected when earlier batches selected the question. The earlier batches are not pooled with the replication (they selected the hypothesis); a pooled estimate would be descriptive only.

## Descriptive
- 4-DoF ATE: 0.078 ± 0.009 vs 0.074 ± 0.008 m, −0.0042 [−0.0098, +0.0014] m (p 0.13). Relative displacement 0.095 vs 0.089 m.
- Run-to-run spread: std 0.016 (M2F) vs 0.014 m (M3). The large spread difference seen in the throughput control (0.024 vs 0.004 m, exploratory Brown-Forsythe p 0.02) did not recur here.
- Registrations: accepted 327 vs 330 of ~344 published, dropped 0 / 0; callback 81 vs 34 ms; latency 83 vs 36 ms (median, mean over runs).

## Report wording (proposal)
"In a pre-registered confirmatory replication on V2_02 (20 runs per method, throughput equalised), the distance-field registration gave a lower map-frame ATE than point-to-plane: −9.9 mm (95 % CI −19.7 to −0.1 mm, Welch p = 0.047). The effect is small and its lower end is compatible with no practical difference; three earlier batches showed the same direction (−13 to −20 mm). No difference was found on V1_01 and V1_02."
