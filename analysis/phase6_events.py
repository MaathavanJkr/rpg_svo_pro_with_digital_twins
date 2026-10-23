#!/usr/bin/env python3
"""Phase 6: assemble the per-registration-event table of one M2/M3 run (offline).

Sources (all written by the run, nothing re-computed online):
  run.bag                  /svo/w_backend_pose_points: every keyframe cloud published by svo_node
  reg_events/kf_<ns>.npz   registration node: clouds (raw, filtered), initial guess, ICP result,
                           information matrices, verdict, timings (one file per PROCESSED keyframe)
  registration_events.csv  svo_node: each registered pose received, with the branch it took
  rosout.log               "... registrated pose added to the optimization!" lines (wall time)
  GT (evaluation only)     state_groundtruth_estimate0 interpolated at the keyframe stamp
                           (<= 10 ms bracket, linear / slerp); raw Vicon orientation too.

Per keyframe: published, processed, dropped_by_queue (published but not processed; queue_size=1),
accepted, received_by_svo + branch, added (each rosout "added" line is matched to the latest earlier
unmatched to_handler receipt), poses and errors vs GT.

Usage (venv + ROS): python analysis/phase6_events.py runs/<run_id> <sequence>
Output: runs/<run_id>/analysis/events.csv (+ events_poses.npz) and a printed summary.
"""
import csv
import glob
import json
import os
import re
import sys
import numpy as np
import rosbag
import yaml
from scipy.spatial.transform import Rotation as R, Slerp

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "analysis"))
import initial_alignment as ia  # noqa: E402


def pose_to_T(p):
    T = np.eye(4)
    T[:3, :3] = R.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def main(run, seq):
    out = os.path.join(run, "analysis")
    os.makedirs(out, exist_ok=True)
    pub = {}
    for _, m, _ in rosbag.Bag(os.path.join(run, "run.bag")).read_messages(topics=["/svo/w_backend_pose_points"]):
        pub[m.header.stamp.to_nsec()] = pose_to_T(m.pose.pose)
    ev = {int(re.search(r"kf_(\d+)\.npz", f).group(1)): f for f in glob.glob(os.path.join(run, "reg_events", "kf_*.npz"))}
    rec = {}
    with open(os.path.join(run, "registration_events.csv")) as f:
        for r in csv.DictReader(f):
            rec[int(r["stamp_ns"])] = (int(r["wall_ns"]), r["branch"])
    added_wall = [int(float(x) * 1e9) for x in re.findall(r"^(\d+\.\d+) INFO /svo .*registrated pose added",
                                                          open(os.path.join(run, "rosout.log"), errors="replace").read(), re.M)]
    # Match each "added" line to the latest earlier unmatched to_handler receipt.
    handler = sorted((w, s) for s, (w, b) in rec.items() if b == "to_handler")
    added = set()
    j = 0
    pending = []
    for w_add in sorted(added_wall):
        while j < len(handler) and handler[j][0] <= w_add:
            pending.append(handler[j][1]); j += 1
        if pending:
            added.add(pending.pop())  # latest receipt before this line
    unmatched_added = len(added_wall) - len(added)

    t_g, p_g, r_g = ia.load_gt(seq)
    vb = os.path.join(REPO, "dataset/derived/asl_meta", seq, "mav0/vicon0/")
    v = np.loadtxt(vb + "data.csv", delimiter=",", comments="#")
    T_BS = np.array(yaml.safe_load(open(vb + "sensor.yaml"))["T_BS"]["data"]).reshape(4, 4)
    vic = Slerp(v[:, 0] * 1e-9, R.from_quat(np.c_[v[:, 5:8], v[:, 4]]) * R.from_matrix(T_BS[:3, :3]).inv())

    rows, poses = [], {}
    for ns in sorted(set(pub) | set(ev)):
        t = ns * 1e-9
        row = dict(stamp_ns=ns, published=ns in pub, processed=ns in ev, dropped_by_queue=(ns in pub and ns not in ev))
        T_init = pub.get(ns)
        T_res = None
        if ns in ev:
            d = np.load(ev[ns])
            T_init = d["T_init_world_body"]
            T_res = d["T_result_world_body"]
            row.update(accepted=bool(d["valid"]), reject_reason=str(d["reject_reason"]),
                       n_raw=len(d["cloud_raw"]), n_filtered=len(d["cloud_filtered"]),
                       n_correspondences=int(d["n_correspondences"]), fitness=float(d["fitness"]),
                       inlier_rmse=float(d["inlier_rmse"]), t_preproc_s=float(d["t_preproc_s"]),
                       t_icp_s=float(d["t_icp_s"]), t_info_s=float(d["t_info_s"]))
            Tc = d["T_icp"]
            row.update(corr_t_m=float(np.linalg.norm(Tc[:3, 3])),
                       corr_rot_deg=float(np.degrees(R.from_matrix(Tc[:3, :3]).magnitude())),
                       corr_yaw_deg=float(np.degrees(R.from_matrix(Tc[:3, :3]).as_rotvec()[2])))
        w_b = rec.get(ns)
        row.update(received_by_svo=w_b is not None, svo_branch=w_b[1] if w_b else "", added=ns in added)
        i = ia.gt_bracket(t_g, t)
        if i is not None and T_init is not None:
            p_gt, r_gt = ia.gt_at(t_g, p_g, r_g, t)
            r_vic = vic([t])[0] if v[0, 0] * 1e-9 <= t <= v[-1, 0] * 1e-9 else None
            for name, T in (("vio", T_init), ("icp", T_res)):
                if T is None:
                    continue
                Rm = R.from_matrix(T[:3, :3])
                e = T[:3, 3] - p_gt
                row[f"{name}_err_t_m"] = float(np.linalg.norm(e))
                row[f"{name}_err_xy_m"] = float(np.linalg.norm(e[:2]))
                row[f"{name}_err_rot_deg"] = float(np.degrees((r_gt.inv() * Rm).magnitude()))
                row[f"{name}_err_yaw_deg"] = float(np.degrees((Rm * r_gt.inv()).as_rotvec()[2]))
                if r_vic is not None:
                    row[f"{name}_err_rot_vicon_deg"] = float(np.degrees((r_vic.inv() * Rm).magnitude()))
                    row[f"{name}_err_yaw_vicon_deg"] = float(np.degrees((Rm * r_vic.inv()).as_rotvec()[2]))
            poses[ns] = dict(T_init=T_init, T_result=T_res if T_res is not None else np.full((4, 4), np.nan),
                             p_gt=p_gt, q_gt_xyzw=r_gt.as_quat())
        rows.append(row)

    keys = sorted({k for r in rows for k in r}, key=lambda k: (k != "stamp_ns", k))
    with open(os.path.join(out, "events.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)
    np.savez_compressed(os.path.join(out, "events_poses.npz"), stamps_ns=np.array(list(poses)),
                        T_init=np.array([p["T_init"] for p in poses.values()]),
                        T_result=np.array([p["T_result"] for p in poses.values()]),
                        p_gt=np.array([p["p_gt"] for p in poses.values()]),
                        q_gt_xyzw=np.array([p["q_gt_xyzw"] for p in poses.values()]))
    proc = [r for r in rows if r["processed"]]
    summary = dict(published=sum(r["published"] for r in rows), processed=len(proc),
                   dropped_by_queue=sum(r["dropped_by_queue"] for r in rows),
                   accepted=sum(r.get("accepted", False) for r in rows),
                   rejected=sum(not r.get("accepted", True) for r in proc),
                   received_by_svo=sum(r["received_by_svo"] for r in rows),
                   branches={b: sum(r["svo_branch"] == b for r in rows) for b in sorted({r["svo_branch"] for r in rows} - {""})},
                   added=len(added), added_lines_in_rosout=len(added_wall), unmatched_added_lines=unmatched_added,
                   runtime_ms_median=dict(preproc=1e3 * np.median([r["t_preproc_s"] for r in proc]),
                                          icp=1e3 * np.median([r["t_icp_s"] for r in proc]),
                                          info=1e3 * np.median([r["t_info_s"] for r in proc if r["accepted"]]),
                                          total=1e3 * np.median([r["t_preproc_s"] + r["t_icp_s"] + r["t_info_s"] for r in proc])))
    json.dump(summary, open(os.path.join(out, "events_summary.json"), "w"), indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
