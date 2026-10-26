# Paper outline (draft, 2026-10-26)

Status labels: **[have]** = result exists (source given); **[running]** = experiment in progress; **[planned]** = design proposal only (docs/PLAN.md, Publication track), not approved, no result.

## Candidate titles
1. "Point-to-Plane or Distance Field? A Controlled Comparison of Digital-Twin Registration in Visual-Inertial Odometry"
2. "Distance-Field Registration for Digital-Twin-Aided VIO: Throughput, Weighting and Scale Effects in a Controlled Study"

(Avoid "drift-free"; the evidence supports "drift reduction" and "controlled comparison".)

## Contributions (grounded; no overclaiming)
1. **A drop-in distance-field registration for an existing tightly coupled VIO + digital-twin system**, verified against its prototype to ~1e-13 and against the original system's outputs to 1e-14, with ~10× lower per-iteration cost and ~4–5× lower per-solve cost than point-to-plane (excluding the original's per-call KD-tree rebuild) and ~8× lower online registration latency. **[have]** (docs/phase8_cpp.md; docs/phase9_online.md)
2. **A pre-registered, controlled comparison** on the EuRoC Vicon rooms with an independent laser-scan twin: no difference on V1_01 and V1_02 (CIs about ±1.5 cm), and a small, replicated improvement on V2_02 (−9.9 mm, 95 % CI [−19.7, −0.1], confirmatory test; three earlier batches −13 to −20 mm). Both registrations reduce map-frame error relative to VIO only, at a local-consistency cost on the easiest sequence. **[have]** (docs/phase9_online.md, docs/phase10_replication.md)
3. **Measurement weighting in the closed loop:** the original heuristic information is strongly underconfident (yaw σ ≈ 60° vs 0.3–0.5° observed), yet replacing it with a Hessian-based information calibrated on open-loop errors made both methods markedly worse (+85 to +156 mm, Holm p ≤ 2e-4); the raw Hessian diverged. Scoped: no robust gating, unchanged backend loss. **[have]** (docs/phase10_d4_realism.md, docs/phase10_d4_ablation.md). A gated variant would turn this into a positive design rule. **[planned: P1]**
4. **Scale effects of the original design:** the per-keyframe KD-tree rebuild limits throughput at room scale (18–28 % of clouds dropped); the original re-alignment criterion cannot fire within a room-scale flight (yaw variance never below 3e-4 rad²). **[have]** (docs/phase9_online.md §5; docs/PLAN.md "BATCH STOPPED"). Online heading-recovery comparison with a room-scale threshold: **[running]**
5. **Robustness to imperfect and tampered digital twins** (DF vs P2P) **[planned: P2]**; city-scale memory/runtime and the original Oerlikon setting **[planned: P3]**.

Contributions 1–4 are the core of a paper with the existing results; 5 would strengthen it. The basin result of Phase 7 (larger yaw basin for DF, offline, open-loop) is supporting evidence, not a headline.

## Related-work positioning (from docs/related_work.md)
Point-to-SDF localisation (Huang et al., IROS 2019), cloud-to-map fusion in VIO (Zuo et al., RA-L 2019; Merat et al., RA-L 2025) and EuRoC prior-map localisation (GMMLoc, RA-L 2020) exist. The contribution is the controlled comparison inside one VIO + digital-twin system with an independent twin, and the systems findings (throughput, weighting, scale), not the distance-field registration itself.

## Figure list
1. System diagram: VIO → keyframe cloud → {point-to-plane | distance field} → registered pose + information → Ceres backend; offline grid build. **[have: text]**
2. Distance-field slices of the V1 twin with registered landmarks. **[have]** (docs/img/phase7_df_slices.png, phase9_smoke_V1_01_M3.png)
3. Map-frame and 4-DoF ATE per run, M1/M2/M3, four sequences. **[have]** (docs/img/phase9_ate.png)
4. V2_02 forest plot: M3 − point-to-plane difference with 95 % CI for all four batches, confirmatory replication highlighted. **[to draw from existing data]**
5. Runtime: per-iteration and per-solve cost, online latency and drop fraction vs keyframe interval. **[have data]** (runs/phase8/timing.json, runs/phase9/aggregate.json)
6. Weighting realism: implied vs observed uncertainty for heuristic and Hessian. **[have]** (docs/img/phase10_d4_realism.png)
7. D4 online: ATE by arm, plus fraction of accepted registrations beyond 3σ. **[have data]**
8. Heading recovery after injected offsets (rate and heading error over time), M3 vs point-to-plane. **[running]**
9. Twin imperfection/tampering degradation curves. **[planned: P2]**

## Limitations to state in the paper
Short indoor sequences; one environment type (two rooms); V1_03 and V2_03 excluded (SVO resets, same for all methods); acceptance thresholds derived on room 1; validity checks accept some wrong registrations; heuristic weighting kept as in the original; GT-position-based initial alignment (stands in for a known take-off pose).
