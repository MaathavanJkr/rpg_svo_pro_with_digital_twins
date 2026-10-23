#!/usr/bin/env python3
"""Phase 1: Leica scan statistics and headless views.

Ground truth is loaded ONLY to overlay the trajectory as a visual frame check.
It is never used to build or modify the map.

Usage: python3 -I analysis/phase1_scan_check.py <asl_meta_dir> <out_img_dir>
"""
import sys
import os
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOMS = {
    "V1": ["V1_01_easy", "V1_02_medium", "V1_03_difficult"],
    "V2": ["V2_01_easy", "V2_02_medium", "V2_03_difficult"],
}


def load_gt(path):
    d = np.loadtxt(path, delimiter=",", comments="#")
    return d[:, 1:4]  # p_RS_R (body/IMU position in Vicon world)


def main(meta, out):
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(0)
    for room, seqs in ROOMS.items():
        ply = os.path.join(meta, seqs[0], "mav0/pointcloud0/data.ply")
        pcd = o3d.io.read_point_cloud(ply)
        P = np.asarray(pcd.points)
        lo, hi = P.min(0), P.max(0)
        # Density: nearest-neighbour spacing on a random subsample.
        tree = o3d.geometry.KDTreeFlann(pcd)
        idx = rng.choice(len(P), 5000, replace=False)
        nn = np.array([np.sqrt(tree.search_knn_vector_3d(P[i], 2)[2][1]) for i in idx])
        occ = len(np.unique(np.floor(P / 0.10).astype(np.int64), axis=0))
        print(f"[{room}] points={len(P)}")
        print(f"[{room}] bbox min={np.round(lo, 3)} max={np.round(hi, 3)} size={np.round(hi - lo, 3)}")
        print(f"[{room}] NN spacing: median={np.median(nn)*1000:.1f} mm, p90={np.percentile(nn, 90)*1000:.1f} mm")
        print(f"[{room}] occupied 10 cm voxels={occ}, mean pts/voxel={len(P)/occ:.1f}")

        gts = {}
        for s in seqs:
            G = load_gt(os.path.join(meta, s, "mav0/state_groundtruth_estimate0/data.csv"))
            gts[s] = G
            inside = np.all((G >= lo) & (G <= hi), axis=1).mean()
            dmin = np.array([np.sqrt(tree.search_knn_vector_3d(g, 1)[2][0]) for g in G[::50]])
            print(f"[{room}] {s}: GT inside scan bbox={inside*100:.1f}%, "
                  f"dist GT->nearest scan pt: min={dmin.min():.2f} m median={np.median(dmin):.2f} m, "
                  f"GT z range=[{G[:,2].min():.2f}, {G[:,2].max():.2f}]")

        S = P[rng.choice(len(P), min(300000, len(P)), replace=False)]
        floor_z = np.percentile(P[:, 2], 2)
        print(f"[{room}] scan z 2nd percentile (floor estimate)={floor_z:.2f} m")

        # Scan-only views.
        for name, (a, b), lab in [("top", (0, 1), ("x [m]", "y [m]")), ("side_xz", (0, 2), ("x [m]", "z [m]")),
                                  ("side_yz", (1, 2), ("y [m]", "z [m]"))]:
            fig, ax = plt.subplots(figsize=(8, 7))
            order = np.argsort(S[:, 2]) if name == "top" else np.arange(len(S))
            sc = ax.scatter(S[order, a], S[order, b], c=S[order, 2], s=0.05, cmap="viridis")
            fig.colorbar(sc, ax=ax, label="z [m]")
            # Clip axes to the room; a few dozen stray points lie tens of metres away.
            plo, phi = np.percentile(P, 0.1, axis=0), np.percentile(P, 99.9, axis=0)
            ax.set_xlim(plo[a] - 0.5, phi[a] + 0.5); ax.set_ylim(plo[b] - 0.5, phi[b] + 0.5)
            ax.set_aspect("equal"); ax.set_xlabel(lab[0]); ax.set_ylabel(lab[1])
            ax.set_title(f"{room} Leica scan (pointcloud0), {name} view, frame R (T_WR = I)")
            fig.savefig(os.path.join(out, f"{room}_scan_{name}.png"), dpi=130, bbox_inches="tight")
            plt.close(fig)

        # Frame check overlay (GT for visualisation only).
        # Top view: drop ceiling so the trajectory is visible.
        T = S[S[:, 2] < floor_z + 2.0]
        fig, axs = plt.subplots(1, 2, figsize=(16, 7))
        axs[0].scatter(T[:, 0], T[:, 1], c="0.6", s=0.05)
        axs[1].scatter(S[:, 0], S[:, 2], c="0.6", s=0.05)
        for s, G in gts.items():
            axs[0].plot(G[:, 0], G[:, 1], lw=1, label=s)
            axs[1].plot(G[:, 0], G[:, 2], lw=1, label=s)
        plo, phi = np.percentile(P, 0.1, axis=0), np.percentile(P, 99.9, axis=0)
        for ax, (xl, yl), (a, b) in zip(axs, [("x [m]", "y [m]"), ("x [m]", "z [m]")], [(0, 1), (0, 2)]):
            ax.set_xlim(plo[a] - 0.5, phi[a] + 0.5); ax.set_ylim(plo[b] - 0.5, phi[b] + 0.5)
            ax.set_aspect("equal"); ax.set_xlabel(xl); ax.set_ylabel(yl); ax.legend(markerscale=10)
        fig.suptitle(f"{room}: FRAME CHECK ONLY: GT trajectory (state_groundtruth_estimate0) over Leica scan; "
                     "GT is not used to build the map", fontsize=11)
        fig.savefig(os.path.join(out, f"{room}_framecheck_gt_overlay.png"), dpi=130, bbox_inches="tight")
        plt.close(fig)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
