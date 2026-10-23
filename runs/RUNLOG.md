# Run log

One entry per run. Append only; never edit past entries.

| Date | Commit | Sequence | Method | Launch / config files | Results | Notes |
|---|---|---|---|---|---|---|
| 2026-10-23 | 73cc782 (+ untracked launch/scripts, snapshot in run dir; committed afterwards) | Oerlikon (`dataset/oerlikon.bag`, 399 s) | M2 original (Phase 3 reproduction), run r01 | `svo_ros/launch/fyp/oerlikon_m2.launch` rviz:=false reference_map:=dataset/Oerlikon.ply; `svo_ros/param/Oerlikon/*`, `svo_registration/config/oerlikon.yaml` | Map-frame ATE (no alignment, evo) RMSE 2.19 m, median 1.36 m, max 6.22 m; SE3-aligned RMSE 1.73 m. GPS align at t+51 s, MR refinement at t+81 s. KFs 344, registrations accepted 231 / rejected 113, backend residuals added 152. Points/KF median 840 raw / 759 filtered. | `runs/2026-10-23_oerlikon_M2_r01/`; offline replay reproduces online accept set exactly; replay ICP median 0.19 s/KF |
| 2026-10-23 | b92a9f1 | Oerlikon | M2 original, run r02 | as r01 | Map-frame (no alignment) ATE_P whole 1.95 m / post-refinement 1.38 m; ATE_R 3.83° / 3.69°. KFs 341, accepted 233 / rejected 108, backend added 200. | `runs/2026-10-23_oerlikon_M2_r02/`; ATE from `analysis/phase3_aggregate.py` |
| 2026-10-23 | b92a9f1 (+ uncommitted svo_ros logger source and docs; **binary unchanged**, built 2026-10-22 17:46) | Oerlikon | M2 original, run r03 | as r01 | ATE_P whole 1.99 m / post 1.43 m; ATE_R 3.86° / 3.79°. KFs 343, accepted 230 / rejected 113, backend added 152. | `runs/2026-10-23_oerlikon_M2_r03/` |
| 2026-10-23 | — | Oerlikon | **M2 aggregate r01–r03** | — | ATE_P whole **2.05 ± 0.14 m**, post-refinement **1.54 ± 0.23 m**; ATE_R 3.89 ± 0.08° / 3.79 ± 0.10° (mean ± std, n=3). r01 recomputed with the aggregate script: 2.21 m (evo 2.19 m). | `runs/phase3_oerlikon_aggregate.json`; paper comparison in `docs/phase3_oerlikon.md` |
