#!/usr/bin/env python3
"""Phase 10 D4: descriptive outlier analysis (secondary; defined in docs/PLAN.md, D4 deviation 2026-10-25, before the batch).

Per run (reg_events/kf_*.npz written by the node: information_final = the published matrix A, T_result_world_body,
valid) and GT (analysis/events_poses.npz, evaluation only), over the ACCEPTED registrations:
  n_beyond_3sigma : number with Mahalanobis distance sqrt(e^T S_pos^-1 e) > 3, S_pos = (A^-1)[:3, :3] (published
                    position covariance), e = registered body position - GT position at the KF stamp
  frac_err_gt_0p2 : fraction with |e| > 0.2 m
Also the per-arm aggregate (mean +- std over valid runs) for the D4 batch.
Usage (venv): python analysis/phase10_d4_outliers.py [run_dir ...]   (default: all *_p10d4_r[0-9][0-9] runs)
Output: runs/<run>/analysis/d4_outliers.json per run, runs/phase10_d4/outliers.json (aggregate)
"""
import glob
import json
import os
import re
import sys
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_outliers(run):
    ep = np.load(os.path.join(run, "analysis/events_poses.npz"))
    gt = {int(s): p for s, p in zip(ep["stamps_ns"], ep["p_gt"])}
    n_acc = n3 = n02 = n_skip = 0
    for f in sorted(glob.glob(os.path.join(run, "reg_events", "kf_*.npz"))):
        d = np.load(f)
        if not bool(d["valid"]):
            continue
        ns = int(re.search(r"kf_(\d+)", f).group(1))
        k = min(gt, key=lambda s: abs(s - ns)) if gt else None
        if k is None or abs(k - ns) > 2e6 or not np.all(np.isfinite(gt[k])):
            n_skip += 1
            continue
        A = d["information_final"]
        e = d["T_result_world_body"][:3, 3] - gt[k]
        S = np.linalg.inv(A)[:3, :3]
        n_acc += 1
        n3 += int(np.sqrt(e @ np.linalg.solve(S, e)) > 3)
        n02 += int(np.linalg.norm(e) > 0.2)
    out = dict(run=os.path.basename(run), accepted_with_gt=n_acc, accepted_without_gt=n_skip, n_beyond_3sigma=n3,
               frac_beyond_3sigma=n3 / n_acc if n_acc else None, frac_err_gt_0p2=n02 / n_acc if n_acc else None)
    json.dump(out, open(os.path.join(run, "analysis/d4_outliers.json"), "w"), indent=2)
    return out


def main():
    runs = sys.argv[1:] or sorted(glob.glob(os.path.join(REPO, "runs/*_p10d4_r[0-9][0-9]")))
    rows = []
    for r in runs:
        m = json.load(open(os.path.join(r, "analysis/phase9_run.json")))
        o = run_outliers(r)
        o.update(sequence=m["sequence"], arm=m["method"], failed=m["failed"])
        rows.append(o)
    agg = {}
    for key in sorted({(o["sequence"], o["arm"]) for o in rows}):
        v = [o for o in rows if (o["sequence"], o["arm"]) == key and not o["failed"]]
        def ms(k):
            x = np.array([o[k] for o in v if o[k] is not None], float)
            return dict(n=len(x), mean=float(x.mean()), std=float(x.std(ddof=1)) if len(x) > 1 else None) if len(x) else None
        agg[f"{key[0]}/{key[1]}"] = dict(n_beyond_3sigma=ms("n_beyond_3sigma"), frac_beyond_3sigma=ms("frac_beyond_3sigma"),
                                         frac_err_gt_0p2=ms("frac_err_gt_0p2"))
    if not sys.argv[1:]:
        json.dump(dict(label="secondary, descriptive", per_run=rows, per_arm=agg),
                  open(os.path.join(REPO, "runs/phase10_d4/outliers.json"), "w"), indent=2)
    for o in rows:
        print(f"{o['run'][11:]:40s} acc {o['accepted_with_gt']:4d} >3sigma {o['n_beyond_3sigma']:4d} ({o['frac_beyond_3sigma'] or 0:.2f}) |e|>0.2 {o['frac_err_gt_0p2'] or 0:.2f}")


if __name__ == "__main__":
    main()
