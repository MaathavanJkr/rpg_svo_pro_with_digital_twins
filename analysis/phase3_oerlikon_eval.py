#!/usr/bin/env python3
"""Phase 3: analyse one Oerlikon M2 run recorded by analysis/run_oerlikon_m2.sh.

1. Extracts trajectories to TUM files (for evo): w_backend (map frame), vio_backend (VIO frame),
   registered poses, GT, GPS.
2. Points per keyframe in /svo/w_backend_pose_points (raw), and after the node's own filters.
3. Offline replay of every keyframe cloud through the ORIGINAL node's methods
   (svo_registration/scripts/online_registration.py, imported, not copied) to classify each KF as
   accepted / rejected (with reason); KFs accepted offline but absent online = dropped online (queue_size=1).
4. Backend acceptance from svo.log ("registrated pose added to the optimization").
5. Headless plots.

Run with system python3 (needs ROS + workspace sourced; open3d/transforms3d from ~/.local):
  python3 analysis/phase3_oerlikon_eval.py runs/<run_id> dataset/Oerlikon.ply
"""
import sys
import os
import json
import time
import numpy as np
import rosbag
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "svo_registration", "scripts"))
import online_registration as orig  # noqa: E402  (original node, unmodified)
import open3d as o3d  # noqa: E402


def pose_row(stamp, p, q):
    return [stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def write_tum(path, rows):
    np.savetxt(path, np.array(rows), fmt="%.9f", header="timestamp tx ty tz qx qy qz qw")


def interp_pos(t_query, t, P):
    ok = (t_query >= t[0]) & (t_query <= t[-1])
    out = np.full((len(t_query), 3), np.nan)
    for k in range(3):
        out[ok, k] = np.interp(t_query[ok], t, P[:, k])
    return out


def main(run, ref_map):
    out = os.path.join(run, "analysis")
    os.makedirs(out, exist_ok=True)
    bag = rosbag.Bag(os.path.join(run, "run.bag"))

    traj = {k: [] for k in ["w_backend", "vio_backend", "registered", "groundtruth", "gps"]}
    kf_msgs = []
    for topic, m, _ in bag.read_messages():
        s = m.header.stamp.to_sec()
        if topic == "/svo/w_backend_pose_imu":
            traj["w_backend"].append(pose_row(s, m.pose.pose.position, m.pose.pose.orientation))
        elif topic == "/svo/vio_backend_pose_imu":
            traj["vio_backend"].append(pose_row(s, m.pose.pose.position, m.pose.pose.orientation))
        elif topic == "/svo/w_registrated_pose":
            traj["registered"].append(pose_row(s, m.pose.pose.position, m.pose.pose.orientation))
        elif topic == "/groundtruth":
            traj["groundtruth"].append(pose_row(s, m.pose.position, m.pose.orientation))
        elif topic == "/gps":
            traj["gps"].append(pose_row(s, m.pose.position, m.pose.orientation))
        elif topic == "/svo/w_backend_pose_points":
            kf_msgs.append(m)
    bag.close()
    for k, rows in traj.items():
        if rows:
            rows.sort()
            write_tum(os.path.join(out, f"{k}.tum"), rows)
    T = {k: np.array(v) for k, v in traj.items() if v}

    # ---- Offline replay through the original node's methods ----
    reg = orig.CityMapRegistrator.__new__(orig.CityMapRegistrator)
    reg.load_config(os.path.join(REPO, "svo_registration/config/oerlikon.yaml"), ref_map)
    online_stamps = set(np.round(T["registered"][:, 0], 6)) if "registered" in T else set()
    rows = []
    for m in kf_msgs:
        stamp = m.header.stamp.to_sec()
        n_raw = m.point_cloud.width * m.point_cloud.height
        Twb = reg.transformation_from_pose(m.pose)
        t0 = time.perf_counter()
        src = reg.filter_local_map(m, Twb)
        tgt = reg.filter_city_twin(Twb)
        icp = o3d.pipelines.registration.registration_icp(
            source=src, target=tgt, max_correspondence_distance=reg.max_correspondence_distance,
            init=np.identity(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25))
        dt = time.perf_counter() - t0
        valid = reg.validate_icp_result(icp)
        tn = float(np.linalg.norm(icp.transformation[:3, 3]))
        c = (np.trace(icp.transformation[:3, :3]) - 1) / 2
        ang = float(np.degrees(np.arccos(np.clip(c, -1, 1))))
        reasons = []
        if icp.fitness < 0.75: reasons.append("fitness")
        if icp.inlier_rmse > reg.inliers_rmse_thr: reasons.append("rmse")
        if tn > reg.max_translation_distance: reasons.append("translation")
        if ang > reg.max_rotation_distance: reasons.append("rotation")
        p = m.pose.pose.position
        reg_p = (icp.transformation @ Twb)[:3, 3]
        rows.append(dict(stamp=stamp, n_raw=n_raw, n_filtered=len(src.points), n_target=len(tgt.points),
                         fitness=icp.fitness, rmse=icp.inlier_rmse, corr_t=tn, corr_deg=ang,
                         valid_offline=bool(valid), reasons="+".join(reasons),
                         published_online=round(stamp, 6) in online_stamps, runtime_s=dt,
                         vio_px=p.x, vio_py=p.y, vio_pz=p.z,
                         reg_px=reg_p[0], reg_py=reg_p[1], reg_pz=reg_p[2]))
    import csv
    with open(os.path.join(out, "keyframes.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    # ---- Backend acceptance from the svo log ----
    log = open(os.path.join(run, "svo.log"), errors="replace").read()
    n_added = log.count("registrated pose added to the optimization")
    gps_align = log.count("GPS Alignment done")
    mr_align = log.count("World to local transform estimated")

    # ---- Errors vs GT (map frame, no alignment) ----
    gt = T["groundtruth"]
    def err(est):
        g = interp_pos(est[:, 0], gt[:, 0], gt[:, 1:4])
        e = np.linalg.norm(est[:, 1:4] - g, axis=1)
        return e[~np.isnan(e)], est[~np.isnan(e), 0]
    e_w, t_w = err(T["w_backend"])
    e_gps, t_gps = err(T["gps"])
    kf = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    g_kf = interp_pos(kf["stamp"], gt[:, 0], gt[:, 1:4])
    e_pre = np.linalg.norm(np.c_[kf["vio_px"], kf["vio_py"], kf["vio_pz"]] - g_kf, axis=1)
    e_reg = np.linalg.norm(np.c_[kf["reg_px"], kf["reg_py"], kf["reg_pz"]] - g_kf, axis=1)
    acc = kf["valid_offline"]

    summary = dict(
        n_keyframe_clouds=len(rows),
        points_raw=dict(min=int(kf["n_raw"].min()), median=float(np.median(kf["n_raw"])), max=int(kf["n_raw"].max())),
        points_after_node_filters=dict(min=int(kf["n_filtered"].min()), median=float(np.median(kf["n_filtered"])),
                                       max=int(kf["n_filtered"].max())),
        replay_accepted=int(acc.sum()), replay_rejected=int((~acc).sum()),
        reject_reasons={r: int(np.sum(kf["reasons"][~acc] == r)) for r in sorted(set(kf["reasons"][~acc]))},
        published_online=int(len(T.get("registered", []))),
        published_online_matched_to_kf=int(kf["published_online"].sum()),
        accepted_offline_not_published_online=int(np.sum(acc & ~kf["published_online"])),
        published_online_but_rejected_offline=int(np.sum(~acc & kf["published_online"])),
        backend_residuals_added=n_added, gps_alignment_done=gps_align, mr_refinement_done=mr_align,
        replay_runtime_s=dict(median=float(np.median(kf["runtime_s"])), p90=float(np.percentile(kf["runtime_s"], 90))),
        map_frame_pos_error_w_backend_m=dict(rmse=float(np.sqrt(np.mean(e_w**2))), median=float(np.median(e_w)),
                                             max=float(e_w.max()), final=float(e_w[-1])),
        gps_pos_error_m=dict(rmse=float(np.sqrt(np.mean(e_gps**2))), median=float(np.median(e_gps))),
        kf_error_before_registration_accepted_m=float(np.nanmedian(e_pre[acc])) if acc.any() else None,
        kf_error_after_registration_accepted_m=float(np.nanmedian(e_reg[acc])) if acc.any() else None,
        duration_s=dict(gt=float(gt[-1, 0] - gt[0, 0]), w_backend=float(T["w_backend"][-1, 0] - T["w_backend"][0, 0])),
    )
    json.dump(summary, open(os.path.join(out, "summary.json"), "w"), indent=2)
    print(json.dumps(summary, indent=2))

    # ---- Plots ----
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.plot(gt[:, 1], gt[:, 2], "k-", lw=2, label="ground truth (/groundtruth)")
    ax.plot(T["gps"][:, 1], T["gps"][:, 2], ".", ms=2, c="0.6", label="GPS (/gps)")
    ax.plot(T["w_backend"][:, 1], T["w_backend"][:, 2], "-", lw=1, c="C0", label="SVO + DT (/svo/w_backend_pose_imu)")
    if "registered" in T:
        ax.plot(T["registered"][:, 1], T["registered"][:, 2], "o", ms=3, mfc="none", c="C3",
                label=f"registered KF poses (n={len(T['registered'])})")
    ax.set_aspect("equal"); ax.set_xlabel("x [m] (map frame 'world')"); ax.set_ylabel("y [m]"); ax.legend()
    ax.set_title("Oerlikon M2 reproduction: top view, map frame, no alignment")
    fig.savefig(os.path.join(out, "traj_top.png"), dpi=130, bbox_inches="tight"); plt.close(fig)

    fig, axs = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    t0 = gt[0, 0]
    axs[0].plot(t_w - t0, e_w, label="SVO + DT vs GT")
    axs[0].plot(t_gps - t0, e_gps, ".", ms=3, c="0.6", label="GPS vs GT")
    axs[0].set_ylabel("position error [m]"); axs[0].legend(); axs[0].set_title("Map frame, no alignment")
    axs[1].plot(kf["stamp"] - t0, kf["n_raw"], label="raw landmarks per KF")
    axs[1].plot(kf["stamp"] - t0, kf["n_filtered"], label="after node filters (outlier + 0.1 m voxel)")
    axs[1].set_ylabel("points"); axs[1].legend()
    c = np.where(acc, "C2", "C3")
    axs[2].scatter(kf["stamp"] - t0, kf["fitness"], c=c, s=8)
    axs[2].axhline(0.75, c="k", lw=0.8, ls="--")
    axs[2].scatter(kf["stamp"][kf["published_online"]] - t0, np.full(kf["published_online"].sum(), 1.03),
                   marker="|", c="C0", s=30, label="published online")
    axs[2].set_ylabel("ICP fitness (green=valid, red=rejected)"); axs[2].set_xlabel("time since start [s]")
    axs[2].legend(loc="lower right")
    fig.savefig(os.path.join(out, "errors_points_fitness.png"), dpi=130, bbox_inches="tight"); plt.close(fig)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
