# Phase 10, experiment 4: initial-alignment robustness (online, 2026-10-26/27)

**Protocol:** pre-registered in docs/PLAN.md ("Phase 10 alignment robustness pre-registration"), redesigned after the stopped first batch ("DEVIATION / REDESIGN", rules recorded before computing the threshold): room-scale refinement threshold τ = 0.0052 rad² (fixed by rule from the δ = 0 Phase 9 logs, `runs/phase10_align/threshold.json`), guard, stop rule. Frozen build 7a471a7 (freeze record f6854d5; libsvo_ros e5ab73806e38 with the flag-guarded offset). Heading offset δ injected after D1 about the body at the end of the D1 window; recovery via the original one-time re-alignment (`refine_T_WL`); gates 30° / 2.0 m for this experiment only; heuristic information. 180 runs (V1_02, V2_02), batch 20:02–03:35, all completed by the harness. Results: `runs/phase10_align/aggregate.json`, `table.md` (per-run folders local).

**Guard:** δ = 0 recovery (refined and |final heading error| < 2°), both methods and sequences pooled: **39/39 = 100 % ≥ 80 %** → the 2° outcome is informative.

**Failures (excluded, not repeated):** 13/180: nine at the known V1_02 reset (~51.4 s), three early resets at 4.2–5.7 s (before the end of the D1 window), one cascade from 43.6 s (V1_02 M3-d20 r10).

## Results (valid runs)
| Sequence | Arm | valid | recovered (Wilson 95 %) | |final heading error| [°] | refinement after D1 window [s] | map-frame ATE, online stream [m] |
|---|---|---|---|---|---|---|
| V1_02 | M2F05 δ0 | 10 | 10/10 [0.72, 1.00] | 0.77 ± 0.37 | 17.2 | 0.067 ± 0.013 |
| | M3 δ0 | 9 | 9/9 [0.70, 1.00] | 1.03 ± 0.43 | 17.2 | 0.060 ± 0.005 |
| | M2F05 δ10 | 9 | 8/9 [0.57, 0.98] | 1.29 ± 0.52 | 17.1 | 0.299 ± 0.010 |
| | M3 δ10 | 8 | 8/8 [0.68, 1.00] | 0.72 ± 0.25 | 17.2 | 0.303 ± 0.006 |
| | M2F20 δ10 (descr.) | 7 | 6/7 [0.49, 0.97] | 1.14 ± 0.56 | 17.2 | 0.305 ± 0.010 |
| | **M2F05 δ20** | 10 | **1/10 [0.02, 0.40]** | **5.16 ± 1.64** | 17.9 | 0.634 ± 0.020 |
| | **M3 δ20** | 8 | **7/8 [0.53, 0.98]** | **0.77 ± 0.87** | 28.9 | 0.668 ± 0.037 |
| | M2F20 δ20 (descr.) | 9 | 9/9 [0.70, 1.00] | 0.74 ± 0.59 | 18.8 | 0.605 ± 0.015 |
| | M2F05 / M3 refine-off δ10 | 5 / 4 | n/a (no refinement) | 10.00 / 10.00 (by construction) | – | 0.343 / 0.407 |
| V2_02 | M2F05 δ0 | 10 | 10/10 [0.72, 1.00] | 0.30 ± 0.24 | 20.5 | 0.105 ± 0.008 |
| | M3 δ0 | 10 | 10/10 [0.72, 1.00] | 0.34 ± 0.30 | 20.6 | 0.096 ± 0.012 |
| | M2F05 δ10 | 10 | 10/10 [0.72, 1.00] | 0.85 ± 0.69 | 21.3 | 0.325 ± 0.025 |
| | M3 δ10 | 10 | 10/10 [0.72, 1.00] | 0.53 ± 0.39 | 20.5 | 0.305 ± 0.013 |
| | M2F20 δ10 (descr.) | 10 | 10/10 [0.72, 1.00] | 0.52 ± 0.55 | 20.7 | 0.311 ± 0.017 |
| | M2F05 δ20 | 10 | 10/10 [0.72, 1.00] | 0.81 ± 0.43 | 24.7 | 0.590 ± 0.012 |
| | M3 δ20 | 9 | 9/9 [0.70, 1.00] | 0.22 ± 0.17 | 22.2 | 0.575 ± 0.009 |
| | M2F20 δ20 (descr.) | 9 | 9/9 [0.70, 1.00] | 0.31 ± 0.20 | 22.4 | 0.580 ± 0.014 |
| | M2F05 / M3 refine-off δ10 | 5 / 5 | n/a | 10.00 / 10.00 (by construction) | – | 0.437 / 0.431 |

## Primary (CMH stratified by sequence, Holm over 2)
- **δ = 20°: supported.** Recovery M3 16/17 vs M2F05 11/20 (V1_02 7/8 vs 1/10; V2_02 9/9 vs 10/10); χ² = 7.46, p = 0.0063, **p_Holm = 0.013**, Mantel–Haenszel odds ratio 63.
- **δ = 10°: not supported** (V1_02 8/8 vs 8/9, V2_02 10/10 vs 10/10; χ² = 0.00, p_Holm = 1); report rates, no claim.

## Secondary (uncorrected, labelled)
- Per-sequence Fisher: V1_02 δ20 p = 0.003 (0.88 vs 0.10); all other cells p = 1.
- |Final heading error|, M3 − M2F05 (Welch): V1_02 δ20 −4.39° [−5.68, −3.10] (p 4e-6); V2_02 δ20 −0.59° [−0.91, −0.26] (p 0.002); V1_02 δ10 −0.57° [−0.99, −0.15] (p 0.013); V2_02 δ10 −0.32° (p 0.23); δ0 not different (p 0.18, 0.77).
- **Descriptive M2F20 arm (point-to-plane with a 2.0 m correspondence radius):** recovered 9/9 at δ20 on V1_02 (0.74 ± 0.59°), like M3. **The V1_02 δ20 advantage of M3 is relative to point-to-plane with its faithful 0.5 m radius**; with a wider radius point-to-plane also recovers, consistent with the offline Phase 7 basin (P2P with d ≥ 1.0 m comparable for translation/combined; DF's yaw advantage largest against d = 0.5 m).
- In the failed V1_02 M2F05 δ20 runs refinement did happen (10/10) but converged to a heading 5.2 ± 1.6° off: point-to-plane registrations started 20° off with a 0.5 m radius partially converge, and the one-time refinement locks in their bias.
- δ = 0 measures refinement noise: |heading change| 0.3–1.0° relative to D1 (which has its own ~0.3° uncertainty).
- Refine-off block (descriptive): with refinement off, T_w_vio is never changed, so its heading error is 10° **by construction** (the pre-registered heading metric measures T_w_vio and is uninformative for this block). Any correction by the accepted registration priors acts on the VIO states and would show only in the map-frame error: 0.34 / 0.41 m (V1_02, M2F05 / M3) and 0.44 / 0.43 m (V2_02), vs 0.30–0.33 m for the refine-on arms at δ = 10°. No heading-correction claim is made for this block.
- Map-frame ATE (online stream) grows with δ for every arm (≈ 0.3 m at 10°, ≈ 0.6 m at 20°), because the error before the refinement at ~17–29 s dominates the RMSE; it is not a recovery measure.

## Reading
"With the original one-time re-alignment (threshold scaled to room size), both methods recovered from 10° heading errors; from 20°, the distance-field registration recovered in 16/17 runs vs 11/20 for point-to-plane with its 0.5 m correspondence radius (CMH, Holm p = 0.013), the difference coming from V1_02 (7/8 vs 1/10). Point-to-plane with a 2.0 m radius also recovered (descriptive). H1 is supported online against the faithful point-to-plane configuration; the advantage is a basin property relative to the correspondence radius, not an absolute one." The original 3e-4 threshold never fires at room scale (scale-dependence finding).
