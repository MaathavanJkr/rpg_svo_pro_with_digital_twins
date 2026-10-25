#!/usr/bin/env python3
"""Phase 10 throughput control: offline verification that M2-fast (online_registration_m2fast.py, persistent
KD-tree) reproduces M2's logged online outputs.

Inputs: every logged keyframe (reg_events/kf_*.npz) of the M2 runs on V1_01 (Phase 6 p6 + Phase 9 p9), V1_02,
V2_02 (p9) and, to exercise the crop fallback, V2_03 (p9; contains the only 2 KFs where M2's crop removed twin
points). The M2-fast node class is driven with messages rebuilt from the logged raw cloud and VIO pose
(as in analysis/phase8_node_timing.py; publishers are stubs). The ICP result is recorded by a thin wrapper
around the node's icp.register (test only), and compared with what the online M2 node logged.
Tolerances (fixed before running): verdict and reason, fitness, number of correspondences and n_target
equal; max |T_icp difference| <= 1e-12; |inlier_rmse difference| <= 1e-12; published pose
(T_result_world_body) max |difference| <= 1e-12; published information max |dA| / max |A| <= 1e-12.
Usage (system python3, catkin workspace sourced): python3 analysis/phase10_m2fast_equivalence.py
Output: runs/phase10_tc/m2fast_equivalence.json + m2fast_equivalence.csv
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
import csv
import glob
import json
import sys
import tempfile
import time
from threading import Lock
import numpy as np
import transforms3d as t3d

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "svo_registration/scripts"))
sys.path.insert(0, os.path.join(REPO, "analysis"))
import online_registration_m2fast as node  # noqa: E402
from phase8_node_timing import Stub, make_msg  # noqa: E402

OUT = os.path.join(REPO, "runs/phase10_tc")
TOL = dict(T=1e-12, rmse=1e-12, pose=1e-12, info_rel=1e-12)
GROUPS = {"V1_01_easy": ["runs/2026-10-23_V1_01_easy_M2_p6_r0*", "runs/2026-10-24_V1_01_easy_M2_p9_r*"],
          "V1_02_medium": ["runs/2026-10-24_V1_02_medium_M2_p9_r*"],
          "V2_02_medium": ["runs/2026-10-24_V2_02_medium_M2_p9_r*"],
          "V2_03_difficult": ["runs/2026-10-24_V2_03_difficult_M2_p9_r*"]}


def make_node(room, tmp):
    obj = node.CityMapRegistrator.__new__(node.CityMapRegistrator)
    obj.lock = Lock()
    obj.load_config(os.path.join(REPO, f"svo_registration/config/euroc_{room.lower()}.yaml"),
                    os.path.join(REPO, f"dataset/derived/euroc_twin/{room}_leica_v30mm.ply"))
    obj.registration_T_w_b_pub = Stub()
    obj.registered_point_cloud_pub = Stub()
    obj.registration_estimate = np.eye(4)
    obj.event_dir = None
    obj.log = open(os.path.join(tmp, f"log_{room}.csv"), "w")
    orig = obj.icp.register

    def recording_register(*a, **k):
        out = orig(*a, **k)
        obj._last = out
        return out
    obj.icp.register = recording_register
    return obj


def published(obj):
    m = obj.registration_T_w_b_pub.last
    if m is None:
        return None, None
    p, o = m.pose.pose.position, m.pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = t3d.quaternions.quat2mat([o.w, o.x, o.y, o.z])
    T[:3, 3] = [p.x, p.y, p.z]
    return T, np.array(m.pose.covariance).reshape(6, 6)


def main():
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp()
    nodes = {r: make_node(r, tmp) for r in ("V1", "V2")}
    rows = []
    for seq, pats in GROUPS.items():
        obj = nodes[seq[:2]]
        runs = sorted(r for p in pats for r in glob.glob(os.path.join(REPO, p)))
        files = [f for r in runs for f in sorted(glob.glob(os.path.join(r, "reg_events", "kf_*.npz")))]
        for f in files:
            ev = dict(np.load(f))
            msg = make_msg(ev)
            obj.registration_T_w_b_pub.last = None
            t0 = time.perf_counter()
            obj.svo_reg_callback(msg)
            dt = time.perf_counter() - t0
            res, tgt, _, rebuilt = obj._last
            v_ev = bool(ev["valid"])
            row = dict(kf=os.path.relpath(f, REPO), sequence=seq, valid=bool(obj.valid), valid_logged=v_ev,
                       tree_rebuilt=bool(rebuilt), n_target=len(tgt), n_target_logged=int(ev["n_target"]),
                       fitness_eq=res.fitness == float(ev["fitness"]),
                       ncorr_eq=len(res.correspondence_set) == int(ev["n_correspondences"]),
                       T_icp_maxabs=float(np.abs(res.transformation - ev["T_icp"]).max()),
                       rmse_abs=abs(res.inlier_rmse - float(ev["inlier_rmse"])), callback_s=dt)
            T, A = published(obj)
            if v_ev and T is not None:
                row["pose_maxabs"] = float(np.abs(T - ev["T_result_world_body"]).max())
                row["info_rel"] = float(np.abs(A - ev["information_final"]).max() / np.abs(ev["information_final"]).max())
            rows.append(row)
        print(seq, len(files), flush=True)
    for o in nodes.values():
        o.log.close()
    with open(os.path.join(OUT, "m2fast_equivalence.csv"), "w", newline="") as fh:
        keys = list(dict.fromkeys(k for r in rows for k in r))
        w = csv.DictWriter(fh, fieldnames=keys); w.writeheader(); w.writerows(rows)
    # Verdict reasons from the node's own CSV logs vs the logged events.
    reasons = {}
    for room in ("V1", "V2"):
        with open(os.path.join(tmp, f"log_{room}.csv")) as fh:
            reasons[room] = [line.rstrip("\n").split(",")[9] for line in fh if line.strip()]
    out = dict(tolerances=TOL, groups={k: [os.path.basename(p) for p in v] for k, v in GROUPS.items()})
    for seq in list(GROUPS) + ["all"]:
        R = [r for r in rows if seq == "all" or r["sequence"] == seq]
        acc = [r for r in R if "pose_maxabs" in r]
        mx = lambda k, rr: float(max((r[k] for r in rr), default=0.0))
        md = lambda k, rr: float(np.median([r[k] for r in rr])) if rr else None
        s = dict(n_kf=len(R), verdict_equal=sum(r["valid"] == r["valid_logged"] for r in R),
                 n_target_equal=sum(r["n_target"] == r["n_target_logged"] for r in R),
                 fitness_equal=sum(r["fitness_eq"] for r in R), ncorr_equal=sum(r["ncorr_eq"] for r in R),
                 tree_rebuilt=sum(r["tree_rebuilt"] for r in R), accepted=len(acc),
                 T_icp_maxabs=dict(max=mx("T_icp_maxabs", R), median=md("T_icp_maxabs", R)),
                 rmse_abs=dict(max=mx("rmse_abs", R), median=md("rmse_abs", R)),
                 pose_maxabs=dict(max=mx("pose_maxabs", acc), median=md("pose_maxabs", acc)),
                 info_rel=dict(max=mx("info_rel", acc), median=md("info_rel", acc)),
                 callback_ms_median_indicative=1e3 * md("callback_s", R))
        s["pass"] = bool(s["verdict_equal"] == s["n_kf"] == s["n_target_equal"] == s["fitness_equal"] == s["ncorr_equal"]
                         and s["T_icp_maxabs"]["max"] <= TOL["T"] and s["rmse_abs"]["max"] <= TOL["rmse"]
                         and s["pose_maxabs"]["max"] <= TOL["pose"] and s["info_rel"]["max"] <= TOL["info_rel"])
        out[seq] = s
    out["n_reason_rows"] = {k: len(v) for k, v in reasons.items()}
    json.dump(out, open(os.path.join(OUT, "m2fast_equivalence.json"), "w"), indent=2)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
