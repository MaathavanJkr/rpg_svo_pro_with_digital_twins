#!/usr/bin/env python3
"""Phase 10 D4: offline check of the information options of the M2-fast and M3 nodes against the step 1 script.

For every logged keyframe of the Phase 9 M2 runs (p9 r01-r10) on one sequence, six node instances
({M2-fast, M3} x {heuristic, hessian, hessian_calibrated}, D4 configs euroc_<room>_d4.yaml / euroc_<room>_m3_d4.yaml)
are driven with the rebuilt message (as analysis/phase8_node_timing.py; publishers are stubs). Expected values are
computed with the step 1 functions (analysis/phase10_d4_realism.py) on the node's own filtered cloud:
  P2P: p2p_icp_persistent ICP, p2p_offline.information_heuristic / information_hessian;
  DF : fyp_df.register, df_register.validate / information_heuristic / information_hessian;
  calibrated: S H S with the factors in runs/phase10_d4/calibration.json.
Tolerances (fixed before running): verdict equal; published pose max |dT| <= 1e-12; published information
max |dA| / max |A| <= 1e-12.
Usage (system python3, catkin sourced): python3 analysis/phase10_d4_node_check.py <sequence>   (one process per sequence)
         python3 analysis/phase10_d4_node_check.py --summary
Output: runs/phase10_d4/node_check_<sequence>.json, node_check.json (summary)
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
import glob
import json
import sys
from threading import Lock
import numpy as np
import transforms3d as t3d

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "svo_registration/scripts"))
sys.path.insert(0, os.path.join(REPO, "analysis"))
OUT = os.path.join(REPO, "runs/phase10_d4")
TOL = dict(pose=1e-12, info_rel=1e-12)
OPTS = ("heuristic", "hessian", "hessian_calibrated")


def published(obj):
    m = obj.registration_T_w_b_pub.last
    if m is None:
        return None, None
    p, o = m.pose.pose.position, m.pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = t3d.quaternions.quat2mat([o.w, o.x, o.y, o.z])
    T[:3, 3] = [p.x, p.y, p.z]
    return T, np.array(m.pose.covariance).reshape(6, 6)


def run(seq):
    import online_registration_m2fast as n2
    import online_registration_m3 as n3
    import df_register as dfr
    import p2p_offline as p2p
    import fyp_df
    from phase8_node_timing import Stub, make_msg
    from p2p_icp_persistent import PersistentPointToPlaneICP
    cal = json.load(open(os.path.join(OUT, "calibration.json")))
    room = seq[:2]
    twin = os.path.join(REPO, f"dataset/derived/euroc_twin/{room}_leica_v30mm.ply")
    grid = os.path.join(REPO, f"dataset/derived/euroc_twin/df/{room}_leica_v30mm_udf_v50mm.bin")
    nodes = {}
    for opt in OPTS:
        for meth, mod, cfg in (("P2P", n2, f"euroc_{room.lower()}_d4.yaml"), ("DF", n3, f"euroc_{room.lower()}_m3_d4.yaml")):
            o = mod.CityMapRegistrator.__new__(mod.CityMapRegistrator)
            o.lock = Lock()
            if meth == "DF":
                o.df_grid_file = grid
            o.load_config(os.path.join(REPO, "svo_registration/config", cfg), twin)
            if meth == "P2P":
                o.information = opt
            else:
                o.df_information = opt
            o.registration_T_w_b_pub, o.registered_point_cloud_pub = Stub(), Stub()
            o.registration_estimate, o.event_dir, o.log = np.eye(4), None, None
            nodes[(meth, opt)] = o
    P2 = nodes[("P2P", "heuristic")].icp
    tw_P, tw_N = P2.points, P2.normals
    icp = PersistentPointToPlaneICP(tw_P, tw_N)
    g = nodes[("DF", "heuristic")].df_grid
    S = {m: np.r_[np.full(3, 1 / cal[m]["k_p"]), np.full(3, 1 / cal[m]["k_r"])] for m in ("P2P", "DF")}
    stats = {f"{m}/{o}": dict(n=0, verdict_equal=0, accepted=0, pose_max=0.0, info_rel_max=0.0, info_rel_median=[])
             for m in ("P2P", "DF") for o in OPTS}
    files = [f for r in sorted(glob.glob(os.path.join(REPO, f"runs/2026-10-24_{seq}_M2_p9_r[0-9][0-9]")))
             for f in sorted(glob.glob(os.path.join(r, "reg_events", "kf_*.npz")))]
    for i, f in enumerate(files):
        ev = dict(np.load(f))
        msg = make_msg(ev)
        Ti = ev["T_init_world_body"]
        pts = np.asarray(nodes[("P2P", "heuristic")].filter_local_map(msg, Ti).points)
        # expected
        r = icp.register(pts, 0.5, 25)[0]
        exp = {}
        cfg = dfr.CFG
        T = r.transformation
        ang = np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1)))
        ok = not (r.fitness < cfg["fitness_threshold"] or r.inlier_rmse > cfg["inliers_rmse_thr"]
                  or np.linalg.norm(T[:3, 3]) > cfg["max_translation_distance"] or ang > cfg["max_rotation_distance"])
        res = dict(correspondences=r.correspondence_set, inlier_rmse=r.inlier_rmse, T_c=T)
        H = p2p.information_hessian(pts, res, tw_P, tw_N) if ok else None
        exp["P2P"] = (ok, T @ Ti, {"heuristic": p2p.information_heuristic(pts, res, tw_N) if ok else None, "hessian": H,
                                   "hessian_calibrated": H * np.outer(S["P2P"], S["P2P"]) if ok else None})
        d = fyp_df.register(g, pts, mu=None, delta=0.05, max_iter=100)
        v = dfr.validate(d)
        Hd = dfr.information_hessian(d) if v["valid"] else None
        exp["DF"] = (v["valid"], d["T_c"] @ Ti, {"heuristic": dfr.information_heuristic(pts, d, v["inliers"]) if v["valid"] else None,
                                                 "hessian": Hd, "hessian_calibrated": Hd * np.outer(S["DF"], S["DF"]) if v["valid"] else None})
        for (m, o), node in nodes.items():
            node.registration_T_w_b_pub.last = None
            node.svo_reg_callback(msg)
            s = stats[f"{m}/{o}"]
            s["n"] += 1
            e_ok, e_T, e_A = exp[m]
            s["verdict_equal"] += int(bool(node.valid) == bool(e_ok))
            Tp, Ap = published(node)
            if e_ok and Tp is not None:
                s["accepted"] += 1
                s["pose_max"] = max(s["pose_max"], float(np.abs(Tp - e_T).max()))
                rel = float(np.abs(Ap - e_A[o]).max() / np.abs(e_A[o]).max())
                s["info_rel_max"] = max(s["info_rel_max"], rel)
                s["info_rel_median"].append(rel)
        if i % 500 == 0:
            print(seq, i, len(files), flush=True)
    for s in stats.values():
        s["info_rel_median"] = float(np.median(s["info_rel_median"])) if s["info_rel_median"] else None
        s["pass"] = bool(s["verdict_equal"] == s["n"] and s["pose_max"] <= TOL["pose"] and s["info_rel_max"] <= TOL["info_rel"])
    json.dump(dict(sequence=seq, n_kf=len(files), tolerances=TOL, results=stats), open(os.path.join(OUT, f"node_check_{seq}.json"), "w"), indent=2)


def summary():
    out = {}
    for f in sorted(glob.glob(os.path.join(OUT, "node_check_*_*.json"))):
        d = json.load(open(f))
        out[d["sequence"]] = d
    out["all_pass"] = all(s["pass"] for d in out.values() if isinstance(d, dict) for s in d["results"].values())
    json.dump(out, open(os.path.join(OUT, "node_check.json"), "w"), indent=2)
    for seq, d in out.items():
        if seq == "all_pass":
            continue
        for k, s in d["results"].items():
            print(f"{seq:16s} {k:24s} n {s['n']} verdict {s['verdict_equal']} acc {s['accepted']} pose {s['pose_max']:.1e} info max {s['info_rel_max']:.1e} pass {s['pass']}")
    print("all_pass", out["all_pass"])


if __name__ == "__main__":
    summary() if sys.argv[1] == "--summary" else run(sys.argv[1])
