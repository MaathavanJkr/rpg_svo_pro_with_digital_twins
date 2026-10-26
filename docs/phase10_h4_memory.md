# Phase 10, H4: city-scale memory extrapolation (analytic, 2026-10-26)

**Question (H4):** distance-field memory grows with map volume × (1/v)³; at what scale does it exceed point-to-plane's map + KD-tree, and what would sparse storage need? **Analytic estimate only** (no grid was built at city scale); inputs from the Oerlikon twin used by the original paper. Script: inline (recorded in the commit), output `runs/phase10_h4/memory.json`.

**Inputs.** `dataset/Oerlikon.ply`: 3 111 643 points (xyz + normals), bounding box + 1 m margin = 314 × 194 × 120 m. Surface area estimated from voxel occupancy at 0.5 m (776 539 occupied voxels → ≈ 1.9e5 m²); the twin's point spacing is too coarse to count surface voxels directly below ~0.25 m, so finer surface counts are extrapolated from this area. Room reference (measured, Phase 8): V1 grid 23.4 MB at v = 0.05 m; point-to-plane twin (402 k points, float64 + normals) 19.4 MB RSS + KD-tree 16.7 MB.

## Results
| v [m] | dense grid (float32, bbox + 1 m) | sparse truncated band, μ = 2v | sparse truncated band, μ = 5v |
|---|---|---|---|
| 0.05 | **235 GB** (6282 × 3877 × 2410) | 1.9 GB | 4.7 GB |
| 0.10 | 29.4 GB | 0.47 GB | 1.2 GB |
| 0.20 | 3.7 GB | 0.12 GB | 0.30 GB |
| 0.50 | 0.24 GB | 0.019 GB | 0.047 GB |
| 1.00 | 0.030 GB | 0.005 GB | 0.012 GB |

Sparse estimate: voxels within ±μ of the surface ≈ A · 2μ / v³, stored in 8³ blocks with an assumed 1.5× block-fill overhead and 32 B index per block (Voxblox/OpenVDB-style); an estimate, not a measurement.

Point-to-plane, for comparison: full twin 149 MB (float64 points + normals) + KD-tree ≈ 129 MB (41.5 B/point, room measurement) ≈ **0.28 GB**; the original pipeline's 1 m-downsampled twin (≈ 203 k points) ≈ 10 MB + 8 MB.

## Reading
- **Scaling confirmed:** dense memory follows volume × (1/v)³ (×8 per halving of v): 23 MB for a 10 m room at 5 cm, 235 GB for the 314 × 194 × 120 m Oerlikon volume at 5 cm.
- **H4 at city scale:** a dense, untruncated grid is larger than point-to-plane's map + KD-tree for v ≤ 0.5 m (0.24 GB vs 0.28 GB at 0.5 m, already comparable; 3.7–235 GB at 0.2–0.05 m). At the resolution the original pipeline actually uses for this twin (1 m), the dense grid (30 MB) is comparable to the downsampled twin + KD-tree (≈ 18 MB). **H4 ("memory higher") holds at city scale for fine voxels and not at room scale.**
- **Sparse storage** makes fine voxels feasible (0.1–4.7 GB at 0.05–0.2 m), but only with **truncation**, and Phase 7 found that truncation shrinks the convergence basin (μ = 5v: success at 1 m translation perturbation 0.48 vs 0.85 untruncated). At city scale the memory–basin trade-off is therefore a real design choice, not a free win; coarse-to-fine (untruncated coarse dense grid + truncated fine sparse band) would be the natural design (P3 / future work).
- Limitations: analytic; the area estimate is conservative (coarse twin); actual Voxblox/OpenVDB overheads differ; no runtime measured at city scale.
