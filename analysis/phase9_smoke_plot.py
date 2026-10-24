#!/usr/bin/env python3
"""Phase 9 smoke run: headless check figure (what RViz would show), from the run's recorded bag.

Panels: (1) top view: twin (subsampled), registered keyframe clouds (/svo/w_backend_points_registered),
map-frame trajectory (/svo/w_backend_pose_imu) and GT; (2) side view x-z of the same; (3) map-frame
position error over time (GT used for evaluation only) and the per-KF correction size from registration.csv.
Usage (venv + ROS): python analysis/phase9_smoke_plot.py runs/<run_id> <sequence> <out.png>
"""
import csv
import os
import sys
import numpy as np
import open3d as o3d
import rosbag
from sensor_msgs import point_cloud2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main(run, seq, out):
    room = seq[:2]
    twin = np.asarray(o3d.io.read_point_cloud(os.path.join(REPO, f"dataset/derived/euroc_twin/{room}_leica_v30mm.ply")).points)
    twin = twin[np.random.default_rng(0).choice(len(twin), 60000, replace=False)]
    reg, traj = [], []
    with rosbag.Bag(os.path.join(run, "run.bag")) as b:
        for topic, m, _ in b.read_messages(topics=["/svo/w_backend_points_registered", "/svo/w_backend_pose_imu"]):
            if topic.endswith("registered"):
                reg.append(np.array(list(point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True))))
            else:
                p = m.pose.pose.position if hasattr(m.pose, "pose") else m.pose.position
                traj.append([m.header.stamp.to_sec(), p.x, p.y, p.z])
    reg = np.vstack(reg) if reg else np.zeros((0, 3))
    traj = np.array(traj)
    gt = np.loadtxt(os.path.join(REPO, f"dataset/derived/asl_meta/{seq}/mav0/state_groundtruth_estimate0/data.csv"),
                    delimiter=",", comments="#", usecols=(0, 1, 2, 3))
    gt[:, 0] *= 1e-9
    gi = np.c_[[np.interp(traj[:, 0], gt[:, 0], gt[:, k]) for k in (1, 2, 3)]].T
    inside = (traj[:, 0] >= gt[0, 0]) & (traj[:, 0] <= gt[-1, 0])
    err = np.linalg.norm(traj[:, 1:] - gi, axis=1)
    rows = list(csv.DictReader(open(os.path.join(run, "registration.csv"))))

    fig, ax = plt.subplots(1, 3, figsize=(20, 6.5))
    for a, (i, j, lab) in zip(ax[:2], ((0, 1, "x–y (top view)"), (0, 2, "x–z (side view)"))):
        a.scatter(twin[:, i], twin[:, j], s=0.2, c="0.75", label="twin (60k of 402k points)")
        a.scatter(reg[:, i], reg[:, j], s=1.0, c="C3", label=f"registered KF clouds ({len(reg)} pts)")
        a.plot(gt[:, i + 1], gt[:, j + 1], "k", lw=0.8, label="GT")
        a.plot(traj[:, i + 1], traj[:, j + 1], "C0", lw=0.8, label="map-frame estimate")
        a.set_aspect("equal"); a.set_title(lab); a.legend(loc="upper right", fontsize=8, markerscale=6)
    a = ax[2]
    t0 = traj[0, 0]
    a.plot(traj[inside, 0] - t0, err[inside], "C0", label="map-frame position error [m] (whole stream, incl. D1 window)")
    st = np.array([float(r["stamp"]) for r in rows]) - t0
    a.plot(st, [float(r["corr_t_m"]) for r in rows], "C3.", ms=3, label="registration correction |t| [m]")
    a.set_xlabel("time since first map-frame pose [s]"); a.legend(fontsize=8); a.grid(alpha=0.3)
    a.set_title(f"{os.path.basename(run)}: accepted {sum(r['valid'] == '1' for r in rows)}/{len(rows)}")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(out, len(reg), len(traj), float(np.sqrt(np.mean(err[inside] ** 2))))


if __name__ == "__main__":
    main(*sys.argv[1:4])
