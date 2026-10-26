# Phase 10, experiment 3 (PROPOSAL, not pre-registered, no runs): initial-alignment robustness

**Question (H1 online):** after a wrong initial heading between the VIO frame and the map, does the system recover, and does the distance field (M3) recover more often than point-to-plane (M2F)? Phase 7 (offline, open-loop) found a larger yaw basin for DF (20°: 0.96–0.97 vs 0.73–0.83 floor-corrected).

## 1. The gate problem
With the main-experiment configuration, a yaw offset δ is not something the system can recover from, for two separate reasons, so the experiment would only measure gates:
1. **Validity gates.** The registration's correction is checked against 5° rotation and 0.75 m translation (`euroc_v{1,2}*.yaml`). The correction a correct registration must apply after an offset δ is a rotation of δ about the map frame, plus a translation of about δ · |p| for a body at distance |p| from the rotation centre (≈ 0.35–1.0 m at 10–20° in a room of ±4 m). For δ ≥ 5°, every correct registration is rejected, for both methods.
2. **No recovery path in the fixed-alignment configuration.** With `refine_T_WL: false`, T_w_vio stays at the D1 value; accepted registrations enter only as heuristic pose priors whose implied yaw σ is ≈ 60° (D4 step 1), so they barely correct heading.

## 2. Recovery mechanism: the original `refine_T_WL` (inspected in the code)
`svo_interface.cpp:866-889`, `global_pose_handler.cpp:388-485`, `ceres_backend_interface.cpp:203-210`. With `refine_T_WL: true`, once `gp_initialized_` (D1) is set, each accepted registered pose is **not** added as a factor; instead its position is paired with the VIO keyframe position at the same stamp and a 4-DoF (yaw + translation) alignment is re-estimated. As soon as ≥ 3 pairs give a yaw variance below `alignement_threshold` (3e-4 in `gpose_settings.yaml`), T_w_vio is replaced (backend, publisher, pose handler), `refined_T_WL = true`, and from then on registrations are added as factors. The refinement happens **once**; T_w_vio is fixed afterwards. This is the original system's own recovery mechanism (used in the Oerlikon reproduction), so it is the faithful choice. Registration-event logging already records the branch per message (`alignment` → `to_handler`), which gives the refinement time.

## 3. Proposed design
- **Offset injection (new code, flag-guarded, default off):** a `d1_yaw_offset_deg` parameter in svo_ros. After the D1(B) fit, T_M_W ← T(p_end) · Rz(δ) · T(−p_end) · T_M_W, i.e. a pure heading error rotating about the body position p_end at the end of the D1 window. Alternative for decision: rotation about the map origin (the PLAN.md text "T_M_W ← Rz(δ)·T_M_W"), which also shifts the position by (Rz(δ) − I)·p_end. I propose the body-centred version, because it isolates heading from position; it needs a decision. δ = 0 must reproduce D1 exactly (offline and online check). This changes libsvo_ros, so it needs a rebuild and a new frozen build.
- **Gates, for this experiment only, identical for both methods:** rotation limit 5° → **30°**, translation limit 0.75 m → **2.0 m** (covers δ = 20° at |p| up to ~5 m). Correspondence radius (0.5 m), fitness (0.75) and inlier RMSE (0.15 m) are unchanged; they are part of each method's definition, and the convergence basin is what is being tested. New configs `euroc_v{1,2}_align.yaml` / `euroc_v{1,2}_m3_align.yaml` (= the D4 heuristic configs with only the two limits changed). Risk, stated in advance: wider gates also admit more wrong solutions, for both methods.
- **Factors:** method {M2F, M3} (heuristic information) × offset δ ∈ {0, 5, 10, 20}° × sequence {V1_02, V2_02}, `refine_T_WL: true` throughout (`gpose_settings.yaml`). δ = 0 is the reference (it also measures what refinement alone does to an already good alignment).
- **Runs and duration:** 10 runs per cell → 2 × 4 × 2 × 10 = 160 runs, interleaved per round (sequence → δ → method). Per-run wall time ≈ 88–117 s of bag + ~45 s overhead → ≈ 7 h. A reduced option: δ ∈ {0, 10, 20}° → 120 runs, ≈ 5.3 h.
- Optional block (descriptive only, decide now or drop): `refine_T_WL: false` with the widened gates at δ = 10°, 5 runs per method and sequence (20 runs, ≈ 50 min), to show how slowly heuristic priors alone correct heading.

## 4. Outcomes, fixed in advance
GT is used for evaluation only. T_w_vio(t) is recovered offline from the recorded bag as T_w_B(t) · T_vio_B(t)⁻¹ (map-frame and VIO-frame backend outputs at the same stamp; both are recorded), so no new logging is needed. ψ_D1 = yaw of the logged D1 alignment without the offset (`d1_alignment.csv`; it has its own ~0.3° uncertainty, Phase 4).
- **Refinement** = the registration-event branch switches to `to_handler` (time t_ref after the D1 window).
- **Recovery (primary outcome per run):** refinement happened AND the final heading error |ψ(T_w_vio, end) − ψ_D1| < 1.0°. Rationale for 1°: a few times the D1 uncertainty, and well below the smallest offset (5°).
- **Secondary per run:** t_ref; heading error after refinement; map-frame ATE after the D1 window **from the online map-frame stream** (not the offline D1 evaluation, which is invalid when T_w_vio changes); 4-DoF ATE; registrations accepted before refinement; registration counts and rejections by reason.
- **Failure rule:** as Phase 9 (resets etc.); a run without refinement is NOT a failure, it is "not recovered".

## 5. Proposed analysis (to be fixed at pre-registration)
- **Primary:** recovery rate M3 vs M2F per offset δ ∈ {5, 10, 20}° and sequence (6 comparisons), Fisher's exact test, Holm over the 6; δ = 0 descriptive. Interpretation as before: significant after Holm = supported; otherwise report the rates with 95 % (Wilson) intervals, no claim. With 10 runs per cell only large differences are detectable (e.g. 10/10 vs 4/10, p ≈ 0.01 before correction); this is stated in advance.
- **Secondary:** heading error and map-frame ATE after refinement, M3 vs M2F (Welch), time to refinement.

## 6. Work before pre-registration (after approval of this design)
1. svo_ros: `d1_yaw_offset_deg` (flag-guarded), rebuild with `scripts/build.sh`, check δ = 0 identical to D1 (offline replay of the alignment + one online run), and δ = 10° gives the expected rotated T_M_W.
2. Configs (`*_align.yaml`), launch argument for the offset and for `map_reg_settings_file` (refine on), a driver, an analysis script for the outcomes above (tested on existing runs with δ = 0).
3. New frozen build, technical smoke runs (one per method at δ = 20° on V1_02), then the batch.

## Decisions needed
1. Offset about the body position at the end of the D1 window (proposed) or about the map origin.
2. Gate values (30°, 2.0 m) and keeping the correspondence radius at 0.5 m.
3. Offsets {0, 5, 10, 20}° with 160 runs (~7 h), or {0, 10, 20}° with 120 runs (~5.3 h).
4. The recovery definition (refined and |heading error| < 1° at the end) and Fisher + Holm over 6 as the primary analysis.
5. Whether to include the optional refine-off block.
