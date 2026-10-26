#!/usr/bin/env python3
"""Phase 10 initial-alignment robustness: per-run outcomes and the pre-registered analysis (docs/PLAN.md,
"Phase 10 alignment robustness pre-registration").

Per run (GT for evaluation only):
  psi_D1        yaw of the D1(B) fit WITHOUT the injected offset (d1_alignment.csv, yaw_deg)
  delta         injected offset (d1_alignment.csv, yaw_offset_deg; 0 if the column is absent)
  refine_on     refine_T_WL on (launch argument gpose_settings:=gpose_settings.yaml, from meta.txt)
  refined, t_ref  refine on: first registered pose taking the "to_handler" branch (registration_events.csv), time
                after the end of the D1 window; refine off: not applicable
  heading       yaw of T_w_vio(t) = T_wB(t) T_vioB(t)^-1 from the recorded map-frame (/svo/w_backend_pose_imu) and
                VIO-frame (/svo/vio_backend_pose_imu) backend outputs at identical stamps
  heading_err   wrap(heading - psi_D1) in deg; final = last matched sample; also the mean over the last 10 s
  recovered     refined AND |final heading_err| < 2.0 deg (refine-on arms only)
  map_ate       map-frame ATE (RMSE) of the ONLINE map-frame stream vs GT after the D1 window (valid when T_w_vio
                changes; the offline D1 evaluation is not)
Analysis (arms are method labels "<arm>-d<delta>", arm in M3, M2F05, M2F20, M3-roff, M2F05-roff):
  primary   M3 vs M2F05 recovery at delta = 10 and 20 deg, Cochran-Mantel-Haenszel (continuity-corrected chi^2, 1 df)
            stratified by sequence, Holm over the 2 tests
  secondary per-sequence Fisher exact tests; Wilson 95 % intervals of every rate; M2F20 arm; delta = 0; continuous
            outcomes (|final heading error|, map-frame ATE, t_ref) M3 vs M2F05, Welch; refine-off block (descriptive)
Usage (venv + ROS): python analysis/phase10_align_analysis.py run <run_dir> [...]   (per-run outcomes; prints)
                    python analysis/phase10_align_analysis.py aggregate [--pattern '*_p10al_r[0-9][0-9]']
Output: runs/<run>/analysis/align_outcome.json; runs/phase10_align/aggregate.json, table.md
"""
import csv
import glob
import json
import os
import re
import sys
import numpy as np
import rosbag
from scipy import stats

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "analysis"))
from phase9_aggregate import welch, holm  # noqa: E402
from phase10_tc_exploratory import gt, map_error  # noqa: E402

SEQS = ["V1_02_medium", "V2_02_medium"]
RECOVERY_DEG = 2.0


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def yaw_deg(qx, qy, qz, qw):
    # yaw of a rotation given as quaternion (x, y, z, w)
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw)], [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz)]])
    return np.degrees(np.arctan2(R[1, 0], R[0, 0]))


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def heading_series(run_dir):
    """T_w_vio yaw over time from the two recorded backend streams (identical header stamps)."""
    w, v = {}, {}
    with rosbag.Bag(os.path.join(run_dir, "run.bag")) as b:
        for topic, m, _ in b.read_messages(topics=["/svo/w_backend_pose_imu", "/svo/vio_backend_pose_imu"]):
            p = m.pose.pose if hasattr(m.pose, "pose") else m.pose
            q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            (w if topic.startswith("/svo/w_") else v)[m.header.stamp.to_nsec()] = quat_R(q)
    ts = sorted(set(w) & set(v))
    yaw = [np.degrees(np.arctan2(*(lambda R: (R[1, 0], R[0, 0]))(w[t] @ v[t].T))) for t in ts]
    return np.array(ts, dtype=np.int64), np.array(yaw)


def run_outcome(run_dir):
    run = os.path.basename(run_dir.rstrip("/"))
    meta = open(os.path.join(run_dir, "meta.txt")).read()
    d1 = list(csv.DictReader(open(os.path.join(run_dir, "d1_alignment.csv"))))[-1]
    psi = float(d1["yaw_deg"])
    delta = float(d1.get("yaw_offset_deg") or 0.0)
    t_end = int(d1["t_end_ns"])
    refine_on = "gpose_settings:=gpose_settings.yaml" in meta
    br = list(csv.DictReader(open(os.path.join(run_dir, "registration_events.csv"))))
    to_h = [int(r["stamp_ns"]) for r in br if r["branch"] == "to_handler"]
    refined = bool(to_h) if refine_on else None
    t_ref = (to_h[0] - t_end) * 1e-9 if (refine_on and to_h) else None
    ts, yaw = heading_series(run_dir)
    keep = ts >= t_end
    ts, yaw = ts[keep], yaw[keep]
    err = wrap(yaw - psi)
    last10 = err[ts >= ts[-1] - int(10e9)] if len(ts) else np.array([])
    seq = re.search(r"(V\d_\d\d_[a-z]+)", run).group(1)
    t, e = map_error(run, gt(seq), t_end * 1e-9)
    out = dict(run=run, sequence=seq, psi_D1_deg=psi, delta_deg=delta, refine_on=refine_on, refined=refined, t_ref_s=t_ref,
               heading_err_initial_deg=float(err[0]) if len(err) else None, heading_err_final_deg=float(err[-1]) if len(err) else None,
               heading_err_last10s_mean_deg=float(last10.mean()) if len(last10) else None,
               recovered=bool(refined and abs(err[-1]) < RECOVERY_DEG) if refine_on and len(err) else None,
               map_ate_online_m=float(np.sqrt(np.mean(e ** 2))) if len(e) else None,
               n_branch={k: sum(r["branch"] == k for r in br) for k in ("initial_discard", "not_initialized", "alignment", "to_handler")})
    os.makedirs(os.path.join(run_dir, "analysis"), exist_ok=True)
    json.dump(out, open(os.path.join(run_dir, "analysis", "align_outcome.json"), "w"), indent=2)
    return out


def wilson(k, n, z=1.959963984540054):
    if n == 0:
        return None
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [float(c - h), float(c + h)]


def cmh(tables):
    """Cochran-Mantel-Haenszel test for 2x2xK tables [[a, b], [c, d]] (a: arm A recovered), continuity corrected."""
    sa = se = sv = 0.0
    num = den = 0.0
    for (a, b), (c, d) in tables:
        n = a + b + c + d
        if n < 2:
            continue
        sa += a
        se += (a + b) * (a + c) / n
        sv += (a + b) * (c + d) * (a + c) * (b + d) / (n * n * (n - 1))
        num += a * d / n
        den += b * c / n
    if sv == 0:
        return dict(computable=False, reason="no variance (identical outcomes in every stratum)")
    chi2 = max(abs(sa - se) - 0.5, 0.0) ** 2 / sv  # continuity correction, clamped at 0 (as scipy)
    return dict(computable=True, chi2=float(chi2), p=float(stats.chi2.sf(chi2, 1)),
                mh_odds_ratio=float(num / den) if den > 0 else float("inf"))


def aggregate(pattern):
    rows = []
    for f in sorted(glob.glob(os.path.join(REPO, "runs", pattern, "analysis", "phase9_run.json"))):
        m = json.load(open(f))
        run_dir = os.path.dirname(os.path.dirname(f))
        o = run_outcome(run_dir)
        o.update(arm=m["method"], failed=m["failed"], failure_reason=m["failure_reason"], accepted=m.get("accepted"),
                 rejected=m.get("rejected"), published=m.get("published"), dropped=m.get("dropped_by_queue"))
        rows.append(o)
    ok = [r for r in rows if not r["failed"]]
    arms = sorted({r["arm"] for r in rows})
    per = {}
    for s in SEQS:
        for a in arms:
            g = [r for r in rows if r["sequence"] == s and r["arm"] == a]
            v = [r for r in g if not r["failed"]]
            rec = [r["recovered"] for r in v if r["recovered"] is not None]
            col = lambda k: [r[k] for r in v if r[k] is not None]
            e = dict(runs=len(g), failed=len(g) - len(v), failure_reasons=sorted({r["failure_reason"] for r in g if r["failed"]}),
                     recovered=int(sum(rec)), n_recovery=len(rec), recovery_rate=float(np.mean(rec)) if rec else None,
                     recovery_wilson95=wilson(int(sum(rec)), len(rec)) if rec else None,
                     refined=int(sum(bool(r["refined"]) for r in v if r["refined"] is not None)))
            for k in ("heading_err_final_deg", "t_ref_s", "map_ate_online_m", "accepted", "rejected"):
                x = np.array(col(k), dtype=float)
                if k == "heading_err_final_deg":
                    x = np.abs(x)
                    k = "abs_heading_err_final_deg"
                e[k] = dict(n=len(x), mean=float(x.mean()), std=float(x.std(ddof=1)) if len(x) > 1 else None,
                            median=float(np.median(x))) if len(x) else None
            per[f"{s}/{a}"] = e
    prim = []
    for d in (10, 20):
        tabs = []
        for s in SEQS:
            A = [r["recovered"] for r in ok if r["sequence"] == s and r["arm"] == f"M3-d{d}" and r["recovered"] is not None]
            B = [r["recovered"] for r in ok if r["sequence"] == s and r["arm"] == f"M2F05-d{d}" and r["recovered"] is not None]
            tabs.append(((sum(A), len(A) - sum(A)), (sum(B), len(B) - sum(B))))
        t = cmh(tabs)
        prim.append(dict(test=f"recovery M3 vs M2F05, delta {d} deg, CMH stratified by sequence", tables=tabs, **t))
    for t, p in zip(prim, holm([t.get("p") if t["computable"] else None for t in prim])):
        t["p_holm"] = p
        t["supported_holm_0.05"] = bool(p is not None and p < 0.05)
    sec = []
    for d in (0, 10, 20):
        for s in SEQS:
            A = [r for r in ok if r["sequence"] == s and r["arm"] == f"M3-d{d}"]
            B = [r for r in ok if r["sequence"] == s and r["arm"] == f"M2F05-d{d}"]
            ra = [r["recovered"] for r in A if r["recovered"] is not None]
            rb = [r["recovered"] for r in B if r["recovered"] is not None]
            if ra and rb:
                odds, p = stats.fisher_exact([[sum(ra), len(ra) - sum(ra)], [sum(rb), len(rb) - sum(rb)]])
                sec.append(dict(test=f"Fisher recovery M3 vs M2F05, {s}, delta {d}", p=float(p), rates=[float(np.mean(ra)), float(np.mean(rb))]))
            for k in ("heading_err_final_deg", "map_ate_online_m", "t_ref_s"):
                xa = [abs(r[k]) if k == "heading_err_final_deg" else r[k] for r in A if r[k] is not None]
                xb = [abs(r[k]) if k == "heading_err_final_deg" else r[k] for r in B if r[k] is not None]
                sec.append(dict(test=f"Welch {k} M3 vs M2F05, {s}, delta {d}", **welch(xb, xa)))
    out = dict(label="primary: CMH + Holm over 2; secondary uncorrected", recovery_threshold_deg=RECOVERY_DEG, n_runs=len(rows),
               per_sequence_arm=per, primary=prim, secondary=sec, runs=rows)
    od = os.path.join(REPO, "runs/phase10_align")
    os.makedirs(od, exist_ok=True)
    json.dump(out, open(os.path.join(od, "aggregate.json"), "w"), indent=2, default=str)
    f = lambda e: "–" if not e else (f"{e['mean']:.3f} ± {e['std']:.3f}" if e.get("std") is not None else f"{e['mean']:.3f}")
    L = ["| Sequence | Arm | runs (failed) | refined | recovered (Wilson 95 %) | |final heading err| [°] | t_ref [s] | map-frame ATE online [m] |",
         "|---|---|---|---|---|---|---|---|"]
    for s in SEQS:
        for a in arms:
            e = per[f"{s}/{a}"]
            rec = f"{e['recovered']}/{e['n_recovery']} [{e['recovery_wilson95'][0]:.2f}, {e['recovery_wilson95'][1]:.2f}]" if e["n_recovery"] else "n/a"
            L.append(f"| {s} | {a} | {e['runs']} ({e['failed']}) | {e['refined']} | {rec} | {f(e['abs_heading_err_final_deg'])} | {f(e['t_ref_s'])} | {f(e['map_ate_online_m'])} |")
    L += ["", "| Primary (Holm over 2) | χ² | p | p (Holm) | supported |", "|---|---|---|---|---|"]
    for t in prim:
        L.append(f"| {t['test']} | " + (f"{t['chi2']:.2f} | {t['p']:.3g} | {t['p_holm']:.3g} | {'yes' if t['supported_holm_0.05'] else 'no'} |" if t["computable"] else f"not computable ({t['reason']}) | – | – | – |"))
    open(os.path.join(od, "table.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    if sys.argv[1] == "run":
        for r in sys.argv[2:]:
            print(json.dumps(run_outcome(r)))
    else:
        aggregate(sys.argv[sys.argv.index("--pattern") + 1] if "--pattern" in sys.argv else "*_p10al_r[0-9][0-9]")
