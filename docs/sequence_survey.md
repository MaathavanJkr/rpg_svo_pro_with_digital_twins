# Sequence survey: SVO full resets (2026-10-24)

Purpose: decide which Vicon sequences enter the main M1/M2/M3 table, given that a full SVO reset re-initialises the VIO heading and the one-shot D1 alignment is then lost (docs/phase6_logging.md, V1_03).
Method: M1 (VIO only, `euroc_m1.launch`, frame-event log on, same D1(B) rule), final build. A **full reset** = stage Relocalization → Paused (relocalisation failed → `resetAll` → `resetBackend`), followed by re-initialisation. Script `analysis/reset_survey.py`, output `runs/reset_survey.json`. Times are relative to the first image.

## Survey
| Sequence | Runs | Full resets (time [s]) | All resets within first ~15 s? | Tracking lost / relocalised | M1 4-DoF ATE [m] (ref.¹) | Rule |
|---|---|---|---|---|---|---|
| V1_01_easy | 5 (Phase 4) | none | – | 0–1 / 0–1 | 0.056 ± 0.003 (0.061) | (i) |
| V1_02_medium | 1 | none | – | 1 / 1 | 0.080 (0.103) | (i) |
| V1_03_difficult | 5 (Phase 6) | r01: 12.3, 101.7 · r02: 12.3, 101.7 · r03: 7.3, 12.4, 32.2, 101.8 · r04: 7.3, 12.4 · r05: 12.4, 101.8 | no in 4/5 runs | 12–21 / 8–19 | 0.294 ± 0.079 (0.179) | (iii) for 4 runs, (ii) for r04 |
| V2_01_easy | 1 | 6.9 | yes | 3 / 2 | 0.133 (0.065) | (ii), start → 7.6 s |
| V2_02_medium | 1 | none | – | 3 / 3 | 0.087 (0.103) | (i) |
| V2_03_difficult | 1 | none | – | 7 / 7 | 0.241 (0.232) | (i) |
¹ Cioffi & Scaramuzza 2020, VIO-only, mono SVO + Ceres, posyaw, median of 3. V2_01's 2× higher ATE is consistent with its early reset mixing two VIO frames in the 4-DoF alignment.

Observations:
- Resets are **run-dependent** (V1_03: 1–4 per run, at partly different times). A single survey run per sequence is indicative only; the rule must be applied **per run** in Phase 9.
- V1_03's late reset (~101.7 s) lies in the last ~3.3 s of a ~105 s flight; the rules as given treat it as mid-flight. The early resets (7.3 / 12.3 s) occur in every V1_03 run.
- No SVO settings were changed (option (d) rejected). Option (c), map-based re-alignment after a reset, is recorded as future work in docs/PLAN.md.

## Proposal per sequence (for decision)
| Sequence | Proposal | Note |
|---|---|---|
| V1_01_easy | **(i) include** | 5/5 runs reset-free |
| V1_02_medium | **(i) include** | confirm with its 5 runs (rule applied per run) |
| V2_01_easy | **(ii) include, "sequence start shifted by ≈7.6 s"** (final re-initialisation after the 6.9 s reset); D1(B) applied from there | needs the start-shift mechanism below |
| V2_02_medium | **(i) include** | |
| V2_03_difficult | **(i) include** | cam0 drops 17.7 % of frames (docs/datasets.md); 7 relocalisations but no full reset in the survey run |
| V1_03_difficult | **(iii) exclude from the main table; report as a limitation** (resets at ~12.3 s in all runs plus mid-flight/late resets in 4/5) | alternative for decision: evaluate the window between the final early re-initialisation (~12.6 s) and the late reset (~101.7 s), i.e. shift the start AND end. This goes beyond rules (i)–(iii), so it is not applied. |

## Mechanism needed for (ii) (not yet implemented)
- **Offline (M1):** `initial_alignment.py` gets a start time: t₀ = the first keyframe with backend+scale initialised and GT available **at or after the final re-initialisation**; the map-frame ATE window starts after the D1(B) window as usual.
- **Online (M2/M3):** the D1 alignment must fire after the final early re-initialisation. Proposed rule, identical online and offline: *if a full reset occurs within the first 15 s, the D1(B) alignment (window, fit, T_w_vio, static TF) is discarded and re-run from the first qualifying keyframe after re-initialisation; resets after 15 s are not handled (run classified (iii))*. This also needs `T_w_vio` to be re-applied to the new backend/publisher after the reset (the gap found on V1_03). This uses GT for the alignment again, but only inside the shifted "start" period, which is consistent with "sequence start shifted by X s".
- The D1(B) window must be re-validated per run; the shift X is then per run, reported in the run table.
- Requires a C++ change in `svo_ros` (re-alignment hook on reset). Implementation only after approval.

## Twin for Vicon Room 2
All V2 sequences need the V2 Leica scan preprocessed with the same rule as V1 (`analysis/preprocess_scan.py V2`, crop + 3 cm + normals) and its own registration config (same values as `euroc_v1.yaml` unless the room geometry requires otherwise). Not done yet.
