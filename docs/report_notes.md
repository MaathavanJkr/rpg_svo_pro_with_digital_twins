# Report notes (running collection for the write-up)

## Stated assumptions
- **A1. One-time initial alignment (EuRoC).** The VIO frame is aligned to the digital-twin frame once, with a 4-DoF (yaw + translation) transform computed from the ground-truth pose at the first keyframe after the VIO backend has initialised. Roll and pitch come from the VIO gravity estimate. This stands in for a known take-off pose. Ground truth is not used online after this point. The same procedure is applied to all methods (M1 offline; M2/M3 online), and the transform is held fixed in the main experiments (`refine_T_WL: false`); online refinement is evaluated as an ablation. *(Decision D1, 2026-10-23; details in `docs/frames.md`.)*
- **A2. Map frame = Vicon frame (EuRoC).** The Leica scan is given in the dataset's world frame (`pointcloud0/sensor.yaml`, `T_WR = I`), so map-frame errors are computed directly against the Vicon ground truth without any trajectory alignment. The scan-to-Vicon registration accuracy of the dataset itself is an error floor we do not control.
- **A3. Identical map input.** M2 and M3 load the same preprocessed scan file (cropped, downsampled, with normals), produced once offline. *(Decision D3.)*
- **A4. Identical measurement weighting (main comparison).** M3 reports its information matrix in M2's heuristic convention (normalised ∇D in place of the surface normal). A Hessian-based covariance for both is an ablation (2×2). *(Decision D4.)*

## Facts worth citing
- The original pipeline only activates map registration after a GPS-based 4-DoF alignment. Registered poses enter the Ceres window as absolute pose priors (Huber δ = 50) on the keyframe with a matching stamp (±1 ms). *(docs/pipeline_notes.md)*
