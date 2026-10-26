#!/usr/bin/env python3
"""Phase 10 D4: EXPLORATORY, descriptive (not pre-registered): are wrong accepted registrations a cause or a consequence of
the calibrated arm's degradation?

1. Open-loop base rate: fraction of accepted registrations with |e| > 0.2 m in the D4 step 1 offline check (logged Phase 9
   M2 inputs, i.e. registrations initialised from a good VIO estimate), per method and sequence (runs/phase10_d4/realism_kf.csv).
2. Timing in each valid hessian_calibrated and heuristic run: t_err = first time the map-frame position error (svo_node map-frame
   output vs GT, after the D1 window) exceeds 0.2 m for >= 2 s; t_reg = time of the first accepted registration with |e| > 0.2 m;
   t_3sig = first accepted registration beyond 3 sigma of its published covariance. Also the fraction of accepted registrations with
   |e| > 0.2 m before vs after t_err.
Usage (venv + ROS): python analysis/phase10_d4_mechanism.py  ->  runs/phase10_d4/mechanism.json
"""
import csv
import glob
import json
import os
import re
import sys
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "analysis"))
from phase10_tc_exploratory import gt, map_error, d1_end  # noqa: E402


def reg_events(run):
    ep = np.load(os.path.join(REPO, "runs", run, "analysis/events_poses.npz"))
    gtp = {int(s): p for s, p in zip(ep["stamps_ns"], ep["p_gt"])}
    out = []
    for f in sorted(glob.glob(os.path.join(REPO, "runs", run, "reg_events", "kf_*.npz"))):
        d = np.load(f)
        if not bool(d["valid"]):
            continue
        ns = int(re.search(r"kf_(\d+)", f).group(1))
        k = min(gtp, key=lambda s: abs(s - ns)) if gtp else None
        if k is None or abs(k - ns) > 2e6 or not np.all(np.isfinite(gtp[k])):
            continue
        e = d["T_result_world_body"][:3, 3] - gtp[k]
        S = np.linalg.inv(d["information_final"])[:3, :3]
        out.append((ns * 1e-9, float(np.linalg.norm(e)), float(np.sqrt(e @ np.linalg.solve(S, e)))))
    return out


def main():
    res = {"label": "EXPLORATORY, descriptive, not pre-registered"}
    kf = list(csv.DictReader(open(os.path.join(REPO, "runs/phase10_d4/realism_kf.csv"))))
    base = {}
    for m in ("P2P", "DF"):
        for seq in ("V1_01_easy", "V1_02_medium", "V2_02_medium"):
            e = np.array([float(k["err_m"]) for k in kf if k["method"] == m and k["seq"] == seq])
            base[f"{m}/{seq}"] = dict(n=len(e), frac_err_gt_0p2=float(np.mean(e > 0.2)))
    res["open_loop_base_rate"] = base
    timing = {}
    for f in sorted(glob.glob(os.path.join(REPO, "runs/*_p10d4_r[0-9][0-9]/analysis/phase9_run.json"))):
        d = json.load(open(f))
        if d["failed"] or not (d["method"].endswith("-cal") or d["method"].endswith("-heur")):
            continue
        run = d["run"]
        g = gt(d["sequence"])
        t, err = map_error(run, g, d1_end(run))
        t0 = t[0]
        above = err > 0.2
        t_err = None
        for i in range(len(t)):
            j = np.searchsorted(t, t[i] + 2.0)
            if above[i] and j <= len(t) and np.all(above[i:j]):
                t_err = float(t[i] - t0)
                break
        ev = [x for x in reg_events(run) if x[0] >= t0]
        t_reg = next((x[0] - t0 for x in ev if x[1] > 0.2), None)
        t_3s = next((x[0] - t0 for x in ev if x[2] > 3), None)
        pre = [x for x in ev if t_err is None or x[0] - t0 < t_err]
        post = [x for x in ev if t_err is not None and x[0] - t0 >= t_err]
        timing[run] = dict(sequence=d["sequence"], arm=d["method"], t_err_s=t_err, t_first_reg_gt_0p2_s=t_reg, t_first_beyond_3sigma_s=t_3s,
                           frac_reg_gt_0p2_before_t_err=float(np.mean([x[1] > 0.2 for x in pre])) if pre else None,
                           frac_reg_gt_0p2_after_t_err=float(np.mean([x[1] > 0.2 for x in post])) if post else None,
                           n_before=len(pre), n_after=len(post))
    res["timing_per_run"] = timing
    summ = {}
    for key in sorted({(v["sequence"], v["arm"]) for v in timing.values()}):
        v = [x for x in timing.values() if (x["sequence"], x["arm"]) == key]
        te = [x["t_err_s"] for x in v if x["t_err_s"] is not None]
        lead = [x["t_err_s"] - x["t_first_reg_gt_0p2_s"] for x in v if x["t_err_s"] is not None and x["t_first_reg_gt_0p2_s"] is not None]
        fb = [x["frac_reg_gt_0p2_before_t_err"] for x in v if x["frac_reg_gt_0p2_before_t_err"] is not None and x["t_err_s"] is not None]
        fa = [x["frac_reg_gt_0p2_after_t_err"] for x in v if x["frac_reg_gt_0p2_after_t_err"] is not None]
        summ[f"{key[0]}/{key[1]}"] = dict(n_runs=len(v), n_runs_error_exceeds_0p2=len(te), t_err_median_s=float(np.median(te)) if te else None,
                                          lead_first_wrong_reg_before_t_err_median_s=float(np.median(lead)) if lead else None,
                                          frac_wrong_before_median=float(np.median(fb)) if fb else None,
                                          frac_wrong_after_median=float(np.median(fa)) if fa else None)
    res["summary"] = summ
    json.dump(res, open(os.path.join(REPO, "runs/phase10_d4/mechanism.json"), "w"), indent=2)
    print(json.dumps(dict(base=base, summary=summ), indent=1))


if __name__ == "__main__":
    main()
