#!/usr/bin/env python3
"""Phase 8: node-level check and H2 timing of online_registration_m3.py (offline, no ROS master).

The node class is driven directly with the logged V1_01 M2 keyframes (922 KFs, runs ..._M2_p6_r01..r05):
each message is rebuilt from the logged raw cloud (float32 in the original message, logged as float64,
so the rebuild is exact) and the logged VIO keyframe pose T_init_world_body. Publishers are replaced by
stubs that keep the last message (publish() itself is not timed; message construction is).

Checks:
  M2 path (registration_method point_to_plane): verdict, published pose and information equal the
    logged online M2 outputs, i.e. the copy's M2 path is unchanged.
  M3 path (distance_field, heuristic and hessian): the node's local filtering reproduces the logged
    cloud_filtered (logged as float32 in these runs, so compared at float32 precision); verdict, published pose and information equal analysis/df_register.py on the node's own (float64) filtered cloud.
H2 timing (decided in Phase 8): one process pinned to one CPU core, OMP_NUM_THREADS=1, same inputs, same
machine; methods interleaved per keyframe, 5 warm-up KFs not counted.
  (a) core solve: Open3D registration_icp exactly as in M2 (target = the node's twin crop, max_iteration
      25; includes Open3D's internal KD-tree build of the target, also timed separately) vs the C++
      fyp_df.register call (wall time of the Python call and the library's internal time).
  (b) end-to-end per-KF callback (preprocessing + registration + validity + information + output
      message construction) for each node path, with the node's own split from its CSV log.
  Memory: RSS increase from loading the grid vs loading the twin + building its KD-tree (fresh processes).

Usage (system python3, catkin workspace sourced):
  python3 analysis/phase8_node_timing.py [--core 3]  ->  runs/phase8/node_check.json, timing.json, timing_per_kf.csv
"""
import os
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
import csv
import glob
import json
import subprocess
import sys
import tempfile
import time
from threading import Lock

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "svo_registration/scripts"))
sys.path.insert(0, os.path.join(REPO, "analysis"))
RUNS = sorted(glob.glob(os.path.join(REPO, "runs/2026-10-23_V1_01_easy_M2_p6_r0*")))
TWIN = os.path.join(REPO, "dataset/derived/euroc_twin/V1_leica_v30mm.ply")
BIN = os.path.join(REPO, "dataset/derived/euroc_twin/df/V1_leica_v30mm_udf_v50mm.bin")
CFG = os.path.join(REPO, "svo_registration/config/euroc_v1_m3.yaml")
OUT = os.path.join(REPO, "runs/phase8")
WARMUP = 5


def rss_kb(field="VmRSS"):
    for line in open("/proc/self/status"):
        if line.startswith(field):
            return int(line.split()[1])


def mem_probe(kind):
    """Run in a fresh process: RSS increase [kB] of loading one map representation."""
    import open3d as o3d
    import fyp_df
    r0, h0 = rss_kb(), rss_kb("VmHWM")
    if kind == "df":
        g = fyp_df.DistanceGrid(BIN, "8fc20009837a56090259b9c5da31724c")
        r1 = rss_kb()
        out = dict(grid_bytes=g.memory_bytes(), rss_delta_kB=r1 - r0, peak_delta_kB=rss_kb("VmHWM") - h0)
    else:
        pc = o3d.io.read_point_cloud(TWIN)
        r1 = rss_kb()
        kd = o3d.geometry.KDTreeFlann(pc)
        r2 = rss_kb()
        out = dict(points=len(pc.points), rss_delta_twin_kB=r1 - r0, rss_delta_kdtree_kB=r2 - r1,
                   peak_delta_kB=rss_kb("VmHWM") - h0)
        del kd
    print(json.dumps(out))


class Stub:
    def __init__(self):
        self.last = None

    def publish(self, m):
        self.last = m


def make_node(node, method, info, log_path, tmpdir):
    cfg = yaml.safe_load(open(CFG))
    cfg["registration_settings"].update(registration_method=method, df_information=info)
    path = os.path.join(tmpdir, f"{method}_{info}.yaml")
    yaml.safe_dump(cfg, open(path, "w"))
    obj = node.CityMapRegistrator.__new__(node.CityMapRegistrator)
    obj.lock = Lock()
    obj.df_grid_file = BIN
    obj.load_config(path, TWIN)
    obj.registration_T_w_b_pub = Stub()
    obj.registered_point_cloud_pub = Stub()
    obj.registration_estimate = np.eye(4)
    obj.event_dir = None
    obj.log = open(log_path, "w", buffering=1)
    obj.log.write("stamp,n_raw,n_filtered,n_target,fitness,inlier_rmse,corr_t_m,corr_rot_deg,"
                  "valid,reject_reason,t_preproc_s,t_icp_s,t_info_s,method,iterations\n")
    return obj


def make_msg(ev):
    import genpy
    import transforms3d as t3d
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header
    from svo_msgs.msg import PoseWithCloud
    raw = ev["cloud_raw"]
    assert np.array_equal(raw.astype(np.float32).astype(np.float64), raw)
    m = PoseWithCloud()
    ns = int(ev["stamp_ns"])
    m.header.stamp = genpy.Time(ns // 10 ** 9, ns % 10 ** 9)
    T = ev["T_init_world_body"]
    q = t3d.quaternions.mat2quat(T[:3, :3])
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = T[:3, 3]
    m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z = q
    m.point_cloud = point_cloud2.create_cloud_xyz32(Header(frame_id="world"), raw.astype(np.float32))
    return m


def published(obj):
    """(T 4x4, information 6x6) of the last published pose, or None."""
    import transforms3d as t3d
    m = obj.registration_T_w_b_pub.last
    if m is None:
        return None
    p, o = m.pose.pose.position, m.pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = t3d.quaternions.quat2mat([o.w, o.x, o.y, o.z])
    T[:3, 3] = [p.x, p.y, p.z]
    return T, np.array(m.pose.covariance).reshape(6, 6)


def rot_deg(Ra, Rb):
    from scipy.spatial.transform import Rotation
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(Ra.T @ Rb).as_rotvec())))


def rel(a, b):
    return float(np.max(np.abs(a - b)) / max(np.max(np.abs(b)), 1e-300))


def stats(x):
    x = np.asarray(x, dtype=float)
    return dict(n=len(x), median_ms=1e3 * float(np.median(x)), p90_ms=1e3 * float(np.percentile(x, 90)),
                mean_ms=1e3 * float(np.mean(x)), max_ms=1e3 * float(np.max(x)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--core", type=int, default=3)
    ap.add_argument("--mem-probe", choices=["df", "p2p"])
    ap.add_argument("--max-kfs", type=int, default=None, help="testing only")
    args = ap.parse_args()
    if args.mem_probe:
        return mem_probe(args.mem_probe)
    os.sched_setaffinity(0, {args.core})
    import open3d as o3d
    import fyp_df
    import df_register as dfr
    import online_registration_m3 as node
    out_dir = OUT if args.max_kfs is None else tempfile.mkdtemp()
    os.makedirs(out_dir, exist_ok=True)
    tmp = tempfile.mkdtemp()
    nodes = {k: make_node(node, m, i, os.path.join(tmp, f"log_{k}.csv"), tmp)
             for k, (m, i) in dict(p2p=("point_to_plane", "heuristic"), df=("distance_field", "heuristic"),
                                   df_hess=("distance_field", "hessian")).items()}
    grid = nodes["df"].df_grid
    files = [f for r in RUNS for f in sorted(glob.glob(os.path.join(r, "reg_events", "kf_*.npz")))][:args.max_kfs]
    chk = dict(p2p_verdict=0, p2p_dt=[], p2p_dr=[], p2p_info=[], filtered_equal=0, df_verdict=0, df_dt=[], df_dr=[],
               df_info_h=[], df_info_H=[], n_accept=dict(p2p=0, df=0, df_hess=0))
    rows = []
    for i, f in enumerate(files):
        ev = dict(np.load(f))
        msg = make_msg(ev)
        T_init = ev["T_init_world_body"]
        # The p6 runs logged cloud_filtered as float32 (before the Phase 7 float64 fix); the node computes it
        # in float64. Inputs for both methods and for the prototype are the node's own filtered cloud.
        pts = np.asarray(nodes["df"].filter_local_map(msg, T_init).points)
        chk["filtered_equal"] += int(pts.shape == ev["cloud_filtered"].shape
                                     and np.array_equal(pts.astype(np.float32), ev["cloud_filtered"].astype(np.float32)))
        row = dict(kf=os.path.relpath(f, REPO), n_points=len(pts))
        # (a) core solve, same points for both methods.
        target = nodes["p2p"].filter_city_twin(T_init)
        src = o3d.geometry.PointCloud()
        src.points = o3d.utility.Vector3dVector(pts)
        t0 = time.perf_counter()
        o3d.pipelines.registration.registration_icp(
            source=src, target=target, max_correspondence_distance=nodes["p2p"].max_correspondence_distance,
            init=np.identity(4), estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25))
        t1 = time.perf_counter()
        o3d.geometry.KDTreeFlann(target)
        t2 = time.perf_counter()
        r = fyp_df.register(grid, pts, mu=None, delta=0.05, max_iter=100)
        t3 = time.perf_counter()
        row.update(core_p2p_s=t1 - t0, kdtree_build_s=t2 - t1, core_df_s=t3 - t2, core_df_internal_s=r["runtime_s"],
                   df_iterations=r["iterations"])
        # (b) end-to-end callbacks.
        for k, obj in nodes.items():
            obj.registration_T_w_b_pub.last = None
            t0 = time.perf_counter()
            obj.svo_reg_callback(msg)
            row[f"e2e_{k}_s"] = time.perf_counter() - t0
            row[f"valid_{k}"] = bool(obj.valid)
            chk["n_accept"][k] += int(obj.valid)
        # Checks (not timed).
        out_p = published(nodes["p2p"])
        if bool(nodes["p2p"].valid) == bool(ev["valid"]):
            chk["p2p_verdict"] += 1
        if out_p is not None and bool(ev["valid"]):
            chk["p2p_dt"].append(float(np.linalg.norm(out_p[0][:3, 3] - ev["T_result_world_body"][:3, 3])))
            chk["p2p_dr"].append(rot_deg(out_p[0][:3, :3], ev["T_result_world_body"][:3, :3]))
            chk["p2p_info"].append(rel(out_p[1], ev["information_final"]))
        a = dfr.register(pts, dfr_grid(), mu=None, delta=0.05, max_iter=100)
        va = dfr.validate(a)
        if bool(nodes["df"].valid) == bool(va["valid"]) and bool(nodes["df_hess"].valid) == bool(va["valid"]):
            chk["df_verdict"] += 1
        if va["valid"]:
            Tr = a["T_c"] @ T_init
            out_d, out_h = published(nodes["df"]), published(nodes["df_hess"])
            chk["df_dt"].append(float(np.linalg.norm(out_d[0][:3, 3] - Tr[:3, 3])))
            chk["df_dr"].append(rot_deg(out_d[0][:3, :3], Tr[:3, :3]))
            chk["df_info_h"].append(rel(out_d[1], dfr.information_heuristic(pts, a, va["inliers"])))
            chk["df_info_H"].append(rel(out_h[1], dfr.information_hessian(a)))
        rows.append(row)
        if i % 100 == 0:
            print(i, len(files), flush=True)
    for obj in nodes.values():
        obj.log.close()
    # Node split from its own CSV log.
    for k in nodes:
        with open(os.path.join(tmp, f"log_{k}.csv")) as fh:
            for row, lr in zip(rows, csv.DictReader(fh)):
                row.update({f"{k}_t_preproc_s": float(lr["t_preproc_s"]), f"{k}_t_reg_s": float(lr["t_icp_s"]),
                            f"{k}_t_info_s": float(lr["t_info_s"])})
    with open(os.path.join(out_dir, "timing_per_kf.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)

    def summ(v):
        v = np.asarray(v, dtype=float)
        return dict(n=len(v), max=float(v.max()) if len(v) else None, median=float(np.median(v)) if len(v) else None)
    node_check = dict(n_kf=len(files), n_accept=chk["n_accept"],
                      p2p_verdict_equal_logged_M2=chk["p2p_verdict"], p2p_pose_dt_m=summ(chk["p2p_dt"]),
                      p2p_pose_dr_deg=summ(chk["p2p_dr"]), p2p_information_rel=summ(chk["p2p_info"]),
                      df_filtered_cloud_equal_logged_float32=chk["filtered_equal"], df_verdict_equal_prototype=chk["df_verdict"],
                      df_pose_dt_m=summ(chk["df_dt"]), df_pose_dr_deg=summ(chk["df_dr"]),
                      df_information_heuristic_rel=summ(chk["df_info_h"]), df_information_hessian_rel=summ(chk["df_info_H"]))
    json.dump(node_check, open(os.path.join(out_dir, "node_check.json"), "w"), indent=2)
    print(json.dumps(node_check, indent=1))

    T = rows[WARMUP:]
    col = lambda k: [r[k] for r in T]
    timing = dict(
        setup=dict(core=args.core, OMP_NUM_THREADS=os.environ["OMP_NUM_THREADS"], n_kf=len(T), warmup=WARMUP,
                   cpu=subprocess.run(["sh", "-c", "grep -m1 'model name' /proc/cpuinfo"], capture_output=True, text=True).stdout.strip(),
                   loadavg_end=open("/proc/loadavg").read().strip(), open3d=o3d.__version__,
                   fyp_df_so=fyp_df.__file__, twin_points=len(nodes["p2p"].city_twin.points),
                   df_iterations_median=float(np.median(col("df_iterations")))),
        core=dict(p2p_icp=stats(col("core_p2p_s")), p2p_kdtree_build_only=stats(col("kdtree_build_s")),
                  df_cpp_call=stats(col("core_df_s")), df_cpp_internal=stats(col("core_df_internal_s"))),
        end_to_end={k: stats(col(f"e2e_{k}_s")) for k in nodes},
        node_split={k: {p: stats(col(f"{k}_t_{p}_s")) for p in ("preproc", "reg", "info")} for k in nodes})
    timing["core"]["speedup_median_p2p_over_df"] = timing["core"]["p2p_icp"]["median_ms"] / timing["core"]["df_cpp_call"]["median_ms"]
    mem = {}
    for kind in ("df", "p2p"):
        p = subprocess.run([sys.executable, os.path.abspath(__file__), "--mem-probe", kind], capture_output=True, text=True)
        mem[kind] = json.loads(p.stdout.strip().splitlines()[-1])
    timing["memory"] = mem
    json.dump(timing, open(os.path.join(out_dir, "timing.json"), "w"), indent=2)
    print(json.dumps(timing, indent=1))


_G = {}


def dfr_grid():
    if "g" not in _G:
        import df_grid
        _G["g"] = df_grid.DistanceGrid(BIN[:-4] + ".npz")
    return _G["g"]


if __name__ == "__main__":
    main()
