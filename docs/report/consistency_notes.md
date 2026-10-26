# Report consistency check (analysis/report_consistency_check.py)

Run 2026-10-26 on docs/report/methodology.md and docs/report/results.md. The script checks every number on a line that cites a
source for PRESENCE among the numeric values of the cited aggregate/JSON files (printed precision, m↔mm, fraction↔%). Output:
`runs/report_consistency.json`.

**Mismatches found and fixed (first run):**
- "Holm p ≤ 2e-4" (D4 family A) was wrong: the largest Holm p is 2.12e-4 (`runs/phase10_d4/aggregate.json`). Corrected to "≤ 2.2e-4" in results.md, phase10_d4_ablation.md (including the approved report wording), report_notes.md and paper/outline.md.
- Online runtime ranges "290–300 ms" / "300–370 ms" / "35–40 ms" were loose roundings; exact ranges from `runs/phase9/aggregate.json`: M2 callback 292–298 ms, latency 298–369 ms; M3 callback 35–38 ms (V1_01, V1_02, V2_02). Corrected in results.md and phase9_online.md.
- The −3.4° heading offset (worst throughput-control run) came from an inline analysis; it is now saved in `runs/phase10_tc/exploratory_heading.json` (−3.43°).
- §4.6 cited three single-run variances (6.8e-4, 1.8e-3); replaced by the 38-run medians in `runs/phase10_align/threshold.json`.
- Earlier (manual): the Phase 9 failure total is 26, not 27.

**Remaining unmatched numbers, verified manually (not in the cited JSON by design):** 917 (keyframes with GT, Phase 7, docs/phase7_offline.md text), 7 589 (= 2 775 + 2 003 + 2 811 keyframes in `m2fast_equivalence.json`), 51.4 s and 64.8 s (reset times, per-run `phase9_run.json`), 17.7 % (cam0 drops, docs/datasets.md), 6 658 (keyframe count, `realism.json` string field), 10 (run count).

**Limitation:** presence is checked, not identity; a number that matches some value in the cited files is not proven to be the intended field.
