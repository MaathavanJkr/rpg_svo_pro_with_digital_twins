#!/usr/bin/env python3
"""Evaluate one EuRoC run (any method) against state_groundtruth_estimate0.

  source /opt/ros/noetic/setup.zsh && source ~/venvs/fyp/bin/activate
  python analysis/eval_vio.py runs/<run_id> <sequence> [--topic /svo/vio_backend_pose_imu]

Outputs runs/<run_id>/analysis/{est.tum, gt.tum, summary.json, traj_top.png, ate_time.png}.

Metrics
- ATE (translation RMSE) after 4-DoF position+yaw alignment using all associated poses
  (closed form, as in Zhang & Scaramuzza 2018 / rpg_trajectory_evaluation 'posyaw').
- ATE after SE(3) alignment (evo Umeyama), for reference only.
- RPE (evo, all pairs) for segment lengths = 10/20/30/40/50 % of the GT path length
  (rpg_trajectory_evaluation convention): translation error [m] and rotation angle [deg].
- Runtime: frames processed vs images in the input bag, frontend processing time (/svo/info),
  backend output latency (receipt wall time - stamp, offset estimated from /imu0), CPU (cpu.csv).
Ground truth is used for evaluation only.
"""
import argparse
import copy
import json
import os
import numpy as np
import rosbag
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evo.core import sync, metrics
from evo.core.trajectory import PoseTrajectory3D
from evo.core.units import Unit

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def gt_trajectory(seq):
    path = os.path.join(REPO, "dataset/derived/asl_meta", seq, "mav0/state_groundtruth_estimate0/data.csv")
    d = np.loadtxt(path, delimiter=",", comments="#")
    # CSV quaternion order is w,x,y,z, which is evo's convention.
    return PoseTrajectory3D(positions_xyz=d[:, 1:4], orientations_quat_wxyz=d[:, 4:8], timestamps=d[:, 0] * 1e-9)


def bag_trajectory(bag, topic):
    t, p, q = [], [], []
    for _, m, _ in bag.read_messages(topics=[topic]):
        pose = m.pose.pose if hasattr(m.pose, "pose") else m.pose
        t.append(m.header.stamp.to_sec())
        p.append([pose.position.x, pose.position.y, pose.position.z])
        q.append([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    o = np.argsort(t)
    return PoseTrajectory3D(positions_xyz=np.array(p)[o], orientations_quat_wxyz=np.array(q)[o],
                            timestamps=np.array(t)[o])


def align_posyaw(p_est, p_gt):
    """4-DoF (yaw + translation) least-squares alignment: returns R (3x3, about z), t (3,)."""
    me, mg = p_est.mean(0), p_gt.mean(0)
    a, b = p_est - me, p_gt - mg
    yaw = np.arctan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R, mg - R @ me, yaw


def save_tum(path, traj):
    q = traj.orientations_quat_wxyz
    np.savetxt(path, np.c_[traj.timestamps, traj.positions_xyz, q[:, 1:4], q[:, 0]], fmt="%.9f",
               header="timestamp tx ty tz qx qy qz qw")


def stats(x):
    x = np.asarray(x, float)
    return dict(rmse=float(np.sqrt(np.mean(x ** 2))), mean=float(x.mean()), median=float(np.median(x)),
                max=float(x.max()), n=int(len(x)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("seq")
    ap.add_argument("--topic", default="/svo/vio_backend_pose_imu")
    args = ap.parse_args()
    out = os.path.join(args.run, "analysis")
    os.makedirs(out, exist_ok=True)

    bag = rosbag.Bag(os.path.join(args.run, "run.bag"))
    est_all = bag_trajectory(bag, args.topic)
    gt_all = gt_trajectory(args.seq)
    save_tum(os.path.join(out, "est.tum"), est_all)
    save_tum(os.path.join(out, "gt.tum"), gt_all)

    gt, est = sync.associate_trajectories(gt_all, est_all, max_diff=0.003)
    pe, pg = est.positions_xyz, gt.positions_xyz

    # ATE, 4-DoF position+yaw alignment
    R, t, yaw = align_posyaw(pe, pg)
    e4 = np.linalg.norm(pg - (pe @ R.T + t), axis=1)
    # ATE, SE(3) alignment (reference)
    est_se3 = copy.deepcopy(est); est_se3.align(gt, correct_scale=False)
    e6 = np.linalg.norm(pg - est_se3.positions_xyz, axis=1)

    # RPE, all pairs, rpg-style segment lengths
    L = float(np.sum(np.linalg.norm(np.diff(gt_all.positions_xyz, axis=0), axis=1)))
    rpe = {}
    for frac in (0.1, 0.2, 0.3, 0.4, 0.5):
        d = round(frac * L, 1)
        res = {}
        for name, rel in (("trans_m", metrics.PoseRelation.translation_part),
                          ("rot_deg", metrics.PoseRelation.rotation_angle_deg)):
            m = metrics.RPE(rel, delta=d, delta_unit=Unit.meters, all_pairs=True)
            m.process_data((gt, est))
            res[name] = dict(mean=float(np.mean(m.error)), rmse=float(np.sqrt(np.mean(m.error ** 2))),
                             n=int(len(m.error)))
        res["trans_pct_of_segment"] = 100.0 * res["trans_m"]["mean"] / d
        rpe[f"{d}m"] = res

    # Runtime
    n_img = sum(1 for _ in rosbag.Bag(os.path.join(REPO, "dataset", next(
        x for x in os.listdir(os.path.join(REPO, "dataset")) if os.path.isdir(os.path.join(REPO, "dataset", x, args.seq))),
        args.seq, args.seq + ".bag")).read_messages(topics=["/cam0/image_raw"]))
    proc = [m.processing_time for _, m, _ in bag.read_messages(topics=["/svo/info"])]
    n_info = len(proc)
    off = np.median([tt.to_sec() - m.header.stamp.to_sec() for _, m, tt in bag.read_messages(topics=["/imu0"])])
    lat = np.array([tt.to_sec() - m.header.stamp.to_sec() - off
                    for _, m, tt in bag.read_messages(topics=[args.topic])])
    runtime = dict(images_in_bag=n_img, frames_processed=n_info, frames_dropped_pct=100.0 * (1 - n_info / n_img),
                   frontend_ms=dict(mean=1e3 * float(np.mean(proc)), p95=1e3 * float(np.percentile(proc, 95))),
                   frontend_rtf=float(np.mean(proc) / 0.05),
                   backend_output_latency_ms=dict(median=1e3 * float(np.median(lat)),
                                                  p95=1e3 * float(np.percentile(lat, 95))))
    cpu_path = os.path.join(args.run, "cpu.csv")
    if os.path.exists(cpu_path):
        c = np.loadtxt(cpu_path, delimiter=",", skiprows=1, ndmin=2)
        busy = c[c[:, 2] > 5]  # samples while svo_node was active
        runtime["cpu"] = dict(svo_node_pct_one_core_mean=float(busy[:, 2].mean()),
                              system_busy_pct_mean=float(busy[:, 1].mean()), samples=int(len(busy)))
    bag.close()

    summary = dict(sequence=args.seq, topic=args.topic, gt_path_length_m=L,
                   est_duration_s=float(est_all.timestamps[-1] - est_all.timestamps[0]),
                   gt_duration_s=float(gt_all.timestamps[-1] - gt_all.timestamps[0]),
                   n_associated=int(len(pe)),
                   ate_posyaw_m=stats(e4), ate_posyaw_yaw_deg=float(np.degrees(yaw)),
                   ate_se3_m=stats(e6), rpe=rpe, runtime=runtime)
    json.dump(summary, open(os.path.join(out, "summary.json"), "w"), indent=2)
    print(json.dumps(summary, indent=2))

    pa = pe @ R.T + t
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(pg[:, 0], pg[:, 1], "k", lw=1.5, label="ground truth")
    ax.plot(pa[:, 0], pa[:, 1], "C0", lw=1, label="estimate (4-DoF aligned)")
    ax.set_aspect("equal"); ax.legend(); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"{args.seq}: {os.path.basename(args.run)}  ATE(posyaw) RMSE {summary['ate_posyaw_m']['rmse']:.3f} m")
    fig.savefig(os.path.join(out, "traj_top.png"), dpi=130, bbox_inches="tight"); plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 3.5))
    ax.plot(est.timestamps - est.timestamps[0], e4)
    ax.set_xlabel("time [s]"); ax.set_ylabel("position error [m]"); ax.set_title("ATE after 4-DoF alignment")
    fig.savefig(os.path.join(out, "ate_time.png"), dpi=130, bbox_inches="tight"); plt.close(fig)


if __name__ == "__main__":
    main()
