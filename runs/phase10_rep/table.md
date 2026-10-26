| Sequence | Method | runs (failed) | map-frame ATE [m] | 4-DoF ATE [m] | rel. disp. [m] | accepted / published | dropped | latency [ms] |
|---|---|---|---|---|---|---|---|---|
| V2_02_medium | M2F-heur | 20 (0) | 0.092 ± 0.016 | 0.078 ± 0.009 | 0.095 ± 0.012 | 327.400 ± 5.072 / 343.800 ± 3.365 | 0.000 ± 0.000 | 83.336 ± 1.369 |
| V2_02_medium | M3-heur | 20 (0) | 0.082 ± 0.014 | 0.074 ± 0.008 | 0.089 ± 0.012 | 329.600 ± 4.260 / 344.150 ± 2.758 | 0.000 ± 0.000 | 36.372 ± 0.348 |

| Primary: M3-heur − M2F-heur | metric | diff [m] (95 % CI) | p (Welch) | p (Holm, 1 seq.) |
|---|---|---|---|---|
| V2_02_medium | map_frame_ate_m | -0.0099 [-0.0197, -0.0001] | 0.0472 | 0.0472 |
| V2_02_medium | ate_4dof_m | -0.0042 [-0.0098, +0.0014] | 0.134 | 0.134 |
