#!/usr/bin/env python3
"""Phase 5 visual yaw check (visualisation only; changes no run or config).

Top-down view of a horizontal wall slice of the V1 twin (z in [0.5, 2.5] m) with the SVO landmarks
of one M2 run in the map frame (union of all /svo/w_backend_pose_points keyframe clouds, same z slice).
Panel 1: as published (D1 alignment). Panel 2: the same landmarks rotated by the median registration
yaw (+4.1 deg) about the map z-axis through the map origin, i.e. how ICP applies its correction.
Bottom row zooms on two opposite walls (south, north).

Usage (venv + ROS sourced): python analysis/phase5_yaw_check.py runs/<M2 run> [--yaw 4.1]
"""
import argparse
import os
import numpy as np
import open3d as o3d
import rosbag
from sensor_msgs import point_cloud2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--yaw", type=float, default=4.1)
    ap.add_argument("--twin", default=os.path.join(REPO, "dataset/derived/euroc_twin/V1_leica_v30mm.ply"))
    args = ap.parse_args()
    zlo, zhi = 0.5, 2.5

    T = np.asarray(o3d.io.read_point_cloud(args.twin).points)
    T = T[(T[:, 2] > zlo) & (T[:, 2] < zhi)]
    L = []
    for _, m, _ in rosbag.Bag(os.path.join(args.run, "run.bag")).read_messages(topics=["/svo/w_backend_pose_points"]):
        L.extend(point_cloud2.read_points(m.point_cloud, skip_nans=True, field_names=("x", "y", "z")))
    L = np.unique(np.round(np.array(L), 4), axis=0)
    L = L[(L[:, 2] > zlo) & (L[:, 2] < zhi)]
    c, s = np.cos(np.radians(args.yaw)), np.sin(np.radians(args.yaw))
    Lr = L @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T

    # Quantitative aid: median distance of landmarks inside the room box to the nearest twin point.
    tree = o3d.geometry.KDTreeFlann(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(T)))
    lo, hi = T.min(0) - 0.3, T.max(0) + 0.3
    def med_dist(P):
        P = P[np.all((P[:, :2] >= lo[:2]) & (P[:, :2] <= hi[:2]), axis=1)]
        return np.median([np.sqrt(tree.search_knn_vector_3d(p, 1)[2][0]) for p in P]), len(P)

    # Opposite walls with many landmarks: south and north (y-extremes of the twin slice).
    y_lo, y_hi = np.percentile(T[:, 1], 0.5), np.percentile(T[:, 1], 99.5)
    zooms = [("south wall", (-3.0, 3.8), (y_lo - 0.6, y_lo + 0.6)), ("north wall", (-1.5, 3.8), (y_hi - 0.6, y_hi + 0.6))]

    fig = plt.figure(figsize=(18, 14))
    gs = fig.add_gridspec(2, 4, height_ratios=[1.6, 0.5])
    for col, (P, title) in enumerate([(L, "D1 alignment (as published)"),
                                      (Lr, f"rotated by +{args.yaw:.1f} deg yaw about map origin")]):
        d, n = med_dist(P)
        ax = fig.add_subplot(gs[0, 2 * col:2 * col + 2])
        ax.scatter(T[:, 0], T[:, 1], s=0.2, c="0.5", label="twin wall slice (z 0.5-2.5 m)")
        ax.scatter(P[:, 0], P[:, 1], s=1, c="C3", alpha=0.25, label="SVO landmarks, map frame")
        ax.set_xlim(lo[0] - 0.5, hi[0] + 0.5); ax.set_ylim(lo[1] - 0.5, hi[1] + 0.5)
        ax.set_aspect("equal"); ax.set_xlabel("x [m] (map)"); ax.set_ylabel("y [m]")
        ax.set_title(f"{title}\nmedian landmark-to-twin distance {100 * d:.1f} cm (n={n})")
        ax.legend(loc="upper center", markerscale=8, fontsize=8)
        for k, (name, (xa, xb), (ya, yb)) in enumerate(zooms):
            ax.add_patch(plt.Rectangle((xa, ya), xb - xa, yb - ya, fill=False, ec="k", lw=0.8))
            z = fig.add_subplot(gs[1, 2 * col + k])
            z.scatter(T[:, 0], T[:, 1], s=0.6, c="0.5"); z.scatter(P[:, 0], P[:, 1], s=2, c="C3", alpha=0.35)
            z.set_xlim(xa, xb); z.set_ylim(ya, yb); z.set_aspect("equal"); z.set_title(f"{name} ({'D1' if col == 0 else '+%.1f deg' % args.yaw})", fontsize=9)
    fig.suptitle(f"Yaw check, {os.path.basename(args.run.rstrip('/'))}: visualisation only", fontsize=12)
    out = os.path.join(REPO, "docs/img/phase5_yaw_check.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print("saved", out, "landmarks", len(L))


if __name__ == "__main__":
    main()
