# Run log

One entry per run. Append only; never edit past entries.

| Date | Commit | Sequence | Method | Launch / config files | Results | Notes |
|---|---|---|---|---|---|---|
| 2026-10-23 | 73cc782 (+ untracked launch/scripts, snapshot in run dir; committed afterwards) | Oerlikon (`dataset/oerlikon.bag`, 399 s) | M2 original (Phase 3 reproduction), run r01 | `svo_ros/launch/fyp/oerlikon_m2.launch` rviz:=false reference_map:=dataset/Oerlikon.ply; `svo_ros/param/Oerlikon/*`, `svo_registration/config/oerlikon.yaml` | Map-frame ATE (no alignment, evo) RMSE 2.19 m, median 1.36 m, max 6.22 m; SE3-aligned RMSE 1.73 m. GPS align at t+51 s, MR refinement at t+81 s. KFs 344, registrations accepted 231 / rejected 113, backend residuals added 152. Points/KF median 840 raw / 759 filtered. | `runs/2026-10-23_oerlikon_M2_r01/`; offline replay reproduces online accept set exactly; replay ICP median 0.19 s/KF |
