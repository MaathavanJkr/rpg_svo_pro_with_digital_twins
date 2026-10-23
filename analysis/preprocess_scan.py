#!/usr/bin/env python3
"""Phase 5b (decision D3): preprocess a EuRoC Leica scan ONCE into the digital-twin file used by
both M2 and M3. The original pointcloud0/data.ply is only read, never modified.

Steps (all parameters recorded in the output .json):
  1. Crop to the room box: per axis [percentile(0.1) - margin, percentile(99.9) + margin] of the
     raw points (removes the few stray points tens of metres away; docs/datasets.md).
  2. Voxel downsample (default 3 cm).
  3. Normals: Open3D hybrid KD-tree search (radius 0.10 m, max 30 neighbours). Normal sign is
     irrelevant for point-to-plane ICP and for an unsigned distance field, so normals are not oriented.
Ground truth is used only for the headless overlay plot (frame check), never for the map.

Usage (venv): python analysis/preprocess_scan.py V1 [--voxel 0.03]
Output: dataset/derived/euroc_twin/<room>_leica_v<voxel_mm>mm.{ply,json} and docs/img/phase5_<room>_twin_*.png
"""
import argparse
import hashlib
import json
import os
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEQS = {"V1": ["V1_01_easy", "V1_02_medium", "V1_03_difficult"],
        "V2": ["V2_01_easy", "V2_02_medium", "V2_03_difficult"]}


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("room", choices=["V1", "V2"])
    ap.add_argument("--voxel", type=float, default=0.03)
    ap.add_argument("--margin", type=float, default=0.2)
    ap.add_argument("--normal-radius", type=float, default=0.10)
    ap.add_argument("--normal-max-nn", type=int, default=30)
    args = ap.parse_args()
    seqs = SEQS[args.room]
    src = os.path.join(REPO, "dataset/derived/asl_meta", seqs[0], "mav0/pointcloud0/data.ply")
    out_dir = os.path.join(REPO, "dataset/derived/euroc_twin")
    os.makedirs(out_dir, exist_ok=True)
    stem = f"{args.room}_leica_v{int(round(args.voxel * 1000))}mm"

    raw = o3d.io.read_point_cloud(src)
    P = np.asarray(raw.points)
    lo = np.percentile(P, 0.1, axis=0) - args.margin
    hi = np.percentile(P, 99.9, axis=0) + args.margin
    keep = np.all((P >= lo) & (P <= hi), axis=1)
    crop = raw.select_by_index(np.flatnonzero(keep))
    down = crop.voxel_down_sample(args.voxel)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.normal_radius, max_nn=args.normal_max_nn))
    down.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))  # drop all-zero RGB
    Q = np.asarray(down.points)
    N = np.asarray(down.normals)
    o3d.io.write_point_cloud(os.path.join(out_dir, stem + ".ply"), down, write_ascii=False)

    # Normal sanity: floor points (lowest 5 cm) should have |n_z| ~ 1.
    floor = Q[:, 2] < np.percentile(Q[:, 2], 2) + 0.05
    meta = dict(source=os.path.relpath(src, REPO), source_md5=md5(src), source_points=int(len(P)),
                crop=dict(rule=f"per-axis [p0.1 - {args.margin}, p99.9 + {args.margin}] of raw points",
                          box_min=lo.round(4).tolist(), box_max=hi.round(4).tolist(),
                          removed=int((~keep).sum()), kept=int(keep.sum())),
                voxel_m=args.voxel, points_after_downsample=int(len(Q)),
                normals=dict(method="Open3D KDTreeSearchParamHybrid", radius_m=args.normal_radius,
                             max_nn=args.normal_max_nn, oriented=False,
                             floor_abs_nz_median=float(np.median(np.abs(N[floor, 2])))),
                bbox_min=Q.min(0).round(4).tolist(), bbox_max=Q.max(0).round(4).tolist(),
                frame="R = V (pointcloud0/sensor.yaml T_WR = I)",
                output=os.path.relpath(os.path.join(out_dir, stem + ".ply"), REPO))
    meta["output_md5"] = md5(os.path.join(out_dir, stem + ".ply"))
    json.dump(meta, open(os.path.join(out_dir, stem + ".json"), "w"), indent=2)
    print(json.dumps(meta, indent=2))

    # Headless figures. GT is a frame check only.
    img = os.path.join(REPO, "docs/img")
    fig, axs = plt.subplots(1, 2, figsize=(16, 7))
    low = Q[Q[:, 2] < np.percentile(Q[:, 2], 2) + 2.0]
    axs[0].scatter(low[:, 0], low[:, 1], s=0.2, c="0.6")
    axs[1].scatter(Q[:, 0], Q[:, 2], s=0.2, c="0.6")
    for s in seqs:
        g = np.loadtxt(os.path.join(REPO, "dataset/derived/asl_meta", s, "mav0/state_groundtruth_estimate0/data.csv"),
                       delimiter=",", comments="#")
        axs[0].plot(g[:, 1], g[:, 2], lw=1, label=s)
        axs[1].plot(g[:, 1], g[:, 3], lw=1, label=s)
    for ax, (xl, yl) in zip(axs, [("x [m]", "y [m]"), ("x [m]", "z [m]")]):
        ax.set_aspect("equal"); ax.set_xlabel(xl); ax.set_ylabel(yl); ax.legend(markerscale=10)
    fig.suptitle(f"{args.room} digital twin ({stem}, {len(Q)} pts): FRAME CHECK ONLY, GT overlay; GT not used for the map")
    fig.savefig(os.path.join(img, f"phase5_{args.room}_twin_gt_overlay.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 7))
    sub = np.random.default_rng(0).choice(len(Q), min(60000, len(Q)), replace=False)
    sc = ax.scatter(Q[sub, 0], Q[sub, 1], c=np.abs(N[sub, 2]), s=0.3, cmap="viridis", vmin=0, vmax=1)
    fig.colorbar(sc, ax=ax, label="|n_z| (1 = horizontal surface)")
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"{args.room} twin normals check: |n_z| (floor/ceiling ~1, walls ~0)")
    fig.savefig(os.path.join(img, f"phase5_{args.room}_twin_normals.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
