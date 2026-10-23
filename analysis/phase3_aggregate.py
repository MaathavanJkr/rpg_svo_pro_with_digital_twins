#!/usr/bin/env python3
"""Phase 3: aggregate Oerlikon M2 runs (map frame, no alignment) -> mean ± std.

Per run (after analysis/phase3_oerlikon_eval.py has produced analysis/*.tum):
  ATE_P = translation RMSE, ATE_R = rotation-angle RMSE (deg), both vs /groundtruth (frame 'world'),
  evaluated on /svo/w_backend_pose_imu with no alignment, nearest GT within 0.05 s, for
  (a) the whole map-frame output window and (b) from the refinement event onward
  (bag time of "World to local transform estimated", from svo.log + wall/bag offset).
Run in the venv: python analysis/phase3_aggregate.py [--out runs/<name>.json] runs/<id1> runs/<id2> ...
"""
import re
import sys
import json
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation


def load_tum(p):
    d = np.loadtxt(p, comments="#")
    return d[:, 0], d[:, 1:4], Rotation.from_quat(d[:, 4:8])


def ate(est, gt, t_from):
    te, pe, re_ = est
    tg, pg, rg = gt
    m = te >= t_from
    j = np.clip(np.searchsorted(tg, te[m]), 1, len(tg) - 1)
    j = np.where(np.abs(tg[j - 1] - te[m]) < np.abs(tg[j] - te[m]), j - 1, j)
    ok = np.abs(tg[j] - te[m]) < 0.05
    ep = np.linalg.norm(pe[m][ok] - pg[j[ok]], axis=1)
    er = np.degrees((rg[j[ok]].inv() * re_[m][ok]).magnitude())
    return dict(ate_p=float(np.sqrt(np.mean(ep ** 2))), ate_r=float(np.sqrt(np.mean(er ** 2))),
                n=int(ok.sum()), window_s=float(te[m][ok][-1] - te[m][ok][0]))


def refine_bag_time(run):
    log = open(f"{run}/svo.log", errors="replace").read()
    wall = float(re.search(r"\[(\d+\.\d+)\]: World to local transform estimated", log).group(1))
    b = rosbag.Bag(f"{run}/run.bag")
    off = np.median([t.to_sec() - m.header.stamp.to_sec() for _, m, t in b.read_messages(topics=["/groundtruth"])])
    return wall - off


args = sys.argv[1:]
out_path = "runs/phase3_oerlikon_aggregate.json"
if args and args[0] == "--out":
    out_path, args = args[1], args[2:]
rows = {}
for run in args:
    est = load_tum(f"{run}/analysis/w_backend.tum")
    gt = load_tum(f"{run}/analysis/groundtruth.tum")
    tr = refine_bag_time(run)
    s = json.load(open(f"{run}/analysis/summary.json"))
    rows[run] = dict(whole=ate(est, gt, -np.inf), post_refinement=ate(est, gt, tr),
                     refine_t_since_start=float(tr - gt[0][0]),
                     accepted=s["replay_accepted"], rejected=s["replay_rejected"],
                     published=s["published_online"], backend_added=s["backend_residuals_added"],
                     points_median=s["points_raw"]["median"])
agg = {}
for w in ("whole", "post_refinement"):
    for k in ("ate_p", "ate_r"):
        v = np.array([r[w][k] for r in rows.values()])
        agg[f"{w}.{k}"] = dict(mean=float(v.mean()), std=float(v.std(ddof=1)) if len(v) > 1 else 0.0,
                               min=float(v.min()), max=float(v.max()))
out = dict(runs=rows, aggregate=agg)
print(json.dumps(out, indent=2))
json.dump(out, open(out_path, "w"), indent=2)
