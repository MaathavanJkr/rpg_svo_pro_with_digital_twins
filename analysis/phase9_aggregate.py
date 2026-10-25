#!/usr/bin/env python3
"""Phase 9: aggregate all runs (pre-registered analysis, docs/PLAN.md "Phase 9 pre-registration").

Input: runs/<date>_<seq>_<M1|M2|M3>_p9_rNN/analysis/phase9_run.json (analysis/phase9_run_metrics.py).
Per sequence and method: runs, failures (with reasons), then over NON-failed runs mean ± std (sample std)
of every metric.
Primary (confirmatory): M2 vs M3 per sequence, Welch's t-test (two-sided) on map-frame ATE and on 4-DoF
ATE; Holm correction across the 4 sequences, separately for each of the two metrics (two families of 4),
alpha = 0.05. Also the mean difference M3 - M2 with its Welch 95 % CI.
Secondary (exploratory, uncorrected, labelled as such): M1 vs M2 and M1 vs M3 (map-frame, 4-DoF ATE),
relative displacement error (all pairs), registration counts and timing (M2 vs M3).
Queue-drop flag (pre-registered): a sequence is flagged if M2's mean dropped_by_queue / published is
> 2x M3's AND >= 5 %. A flag triggers the Phase 10 "M2 with persistent KD-tree" variant.
A test needs >= 2 non-failed runs in both groups, otherwise it is reported as not computable.
Usage (venv): python analysis/phase9_aggregate.py [--pattern '*_p9_r[0-9][0-9]']
Output: runs/phase9/aggregate.json, runs/phase9/table.md
Phase 10 throughput control (same statistics, other groups):
  python analysis/phase9_aggregate.py --pattern '*_p10tc_r[0-9][0-9]' --seqs V1_02_medium V2_02_medium \
      --methods M2F M3 --primary M2F M3 --out runs/phase10_tc
  (primary = the given pair, Holm across the given sequences; the drop flag compares the same pair.)
"""
import argparse
import glob
import json
import os
import numpy as np
from scipy import stats

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEQS = ["V1_01_easy", "V1_02_medium", "V2_02_medium", "V2_03_difficult"]
METHODS = ["M1", "M2", "M3"]
PRIMARY = ["map_frame_ate_m", "ate_4dof_m"]
SECONDARY = ["rel_disp_m_mean"]
REG = ["published", "dropped_by_queue", "accepted", "rejected", "added"]
SYSTEM = ["cpu_svo_node_pct_mean", "cpu_reg_node_pct_mean", "frontend_rtf", "rtf_playback", "registration_load",
          "frames_dropped_pct", "backend_output_latency_ms_median"]


def ms(x):
    x = np.asarray([v for v in x if v is not None], dtype=float)
    if not len(x):
        return None
    return dict(n=int(len(x)), mean=float(x.mean()), std=float(x.std(ddof=1)) if len(x) > 1 else None)


def welch(a, b):
    """Welch test of b vs a; returns p, mean difference b - a and its 95 % CI (Welch-Satterthwaite df)."""
    a = np.asarray([v for v in a if v is not None], float); b = np.asarray([v for v in b if v is not None], float)
    if len(a) < 2 or len(b) < 2:
        return dict(computable=False, n_a=int(len(a)), n_b=int(len(b)))
    t, p = stats.ttest_ind(b, a, equal_var=False)
    va, vb = a.var(ddof=1) / len(a), b.var(ddof=1) / len(b)
    df = (va + vb) ** 2 / (va ** 2 / (len(a) - 1) + vb ** 2 / (len(b) - 1)) if va + vb > 0 else np.inf
    d = float(b.mean() - a.mean())
    h = float(stats.t.ppf(0.975, df) * np.sqrt(va + vb)) if np.isfinite(df) else 0.0
    return dict(computable=True, n_a=int(len(a)), n_b=int(len(b)), t=float(t), df=float(df), p=float(p),
                diff=d, ci95=[d - h, d + h])


def holm(ps):
    """Holm-adjusted p-values (None entries are skipped and returned as None)."""
    idx = [i for i, p in enumerate(ps) if p is not None]
    order = sorted(idx, key=lambda i: ps[i])
    m = len(order)
    adj = [None] * len(ps)
    run = 0.0
    for k, i in enumerate(order):
        run = max(run, min(1.0, (m - k) * ps[i]))
        adj[i] = run
    return adj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pattern", default="*_p9_r[0-9][0-9]")
    ap.add_argument("--out", default=os.path.join(REPO, "runs/phase9"))
    ap.add_argument("--seqs", nargs="+", default=SEQS)
    ap.add_argument("--methods", nargs="+", default=METHODS)
    ap.add_argument("--primary", nargs=2, default=["M2", "M3"])
    args = ap.parse_args()
    seqs, methods, (pa, pb) = args.seqs, args.methods, args.primary
    runs = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(REPO, "runs", args.pattern, "analysis/phase9_run.json")))]
    G = {(s, m): [r for r in runs if r["sequence"] == s and r["method"] == m] for s in seqs for m in methods}
    ok = {k: [r for r in v if not r["failed"]] for k, v in G.items()}
    col = lambda k, key: [r.get(key) for r in ok[k]]

    per = {}
    for (s, m), rs in G.items():
        e = dict(runs=len(rs), failed=sum(r["failed"] for r in rs),
                 failure_reasons=sorted({r["failure_reason"] for r in rs if r["failed"]}))
        for key in PRIMARY + SECONDARY + SYSTEM + (REG if m != "M1" else []):
            e[key] = ms(col((s, m), key))
        if m != "M1":
            e["latency_ms_median"] = ms([(r.get("latency_ms") or {}).get("median") for r in ok[(s, m)]])
            e["callback_ms_median"] = ms([(r.get("callback_ms") or {}).get("median") for r in ok[(s, m)]])
            pub = [r.get("published") for r in ok[(s, m)]]
            drp = [r.get("dropped_by_queue") for r in ok[(s, m)]]
            e["drop_fraction"] = ms([d / p for d, p in zip(drp, pub) if d is not None and p])
        per[f"{s}/{m}"] = e

    primary = {}
    for key in PRIMARY:
        tests = [welch(col((s, pa), key), col((s, pb), key)) for s in seqs]
        adj = holm([t.get("p") if t["computable"] else None for t in tests])
        for s, t, a in zip(seqs, tests, adj):
            t["p_holm"] = a
            t["significant_holm_0.05"] = bool(a is not None and a < 0.05)
        primary[key] = dict(zip(seqs, tests))

    secondary = {}
    for s in seqs:
        sec = {}
        if "M1" in methods:
            for key in PRIMARY + SECONDARY:
                sec[f"{key}: {pa} vs M1"] = welch(col((s, "M1"), key), col((s, pa), key))
                sec[f"{key}: {pb} vs M1"] = welch(col((s, "M1"), key), col((s, pb), key))
        sec[f"rel_disp_m_mean: {pb} vs {pa}"] = welch(col((s, pa), "rel_disp_m_mean"), col((s, pb), "rel_disp_m_mean"))
        for key in ("accepted", "added", "dropped_by_queue"):
            sec[f"{key}: {pb} vs {pa}"] = welch(col((s, pa), key), col((s, pb), key))
        secondary[s] = sec

    flags = {}
    for s in seqs:
        d2, d3 = per[f"{s}/{pa}"]["drop_fraction"], per[f"{s}/{pb}"]["drop_fraction"]
        f2, f3 = (d2 or {}).get("mean"), (d3 or {}).get("mean")
        flags[s] = {f"{pa.lower()}_drop_fraction": f2, f"{pb.lower()}_drop_fraction": f3,
                    "flag": bool(f2 is not None and f3 is not None and f2 >= 0.05 and f2 > 2 * f3)}

    out = dict(n_runs=len(runs), per_sequence_method=per, **{f"primary_{pa}_vs_{pb}": primary},
               secondary_exploratory_uncorrected=secondary, queue_drop_flags=flags)
    os.makedirs(args.out, exist_ok=True)
    json.dump(out, open(os.path.join(args.out, "aggregate.json"), "w"), indent=2)

    f = lambda e: "–" if not e else (f"{e['mean']:.3f} ± {e['std']:.3f}" if e["std"] is not None else f"{e['mean']:.3f}")
    L = ["| Sequence | Method | runs (failed) | map-frame ATE [m] | 4-DoF ATE [m] | rel. disp. [m] | accepted / published | dropped | latency [ms] |",
         "|---|---|---|---|---|---|---|---|---|"]
    for s in seqs:
        for m in methods:
            e = per[f"{s}/{m}"]
            reg = "–" if m == "M1" else f"{f(e['accepted'])} / {f(e['published'])}"
            L.append(f"| {s} | {m} | {e['runs']} ({e['failed']}) | {f(e['map_frame_ate_m'])} | {f(e['ate_4dof_m'])} | "
                     f"{f(e['rel_disp_m_mean'])} | {reg} | {'–' if m == 'M1' else f(e['dropped_by_queue'])} | "
                     f"{'–' if m == 'M1' else f(e['latency_ms_median'])} |")
    L += ["", f"| Primary: {pb} − {pa} | metric | diff [m] (95 % CI) | p (Welch) | p (Holm, 4 seq.) |", "|---|---|---|---|---|"]
    for key in PRIMARY:
        for s in seqs:
            t = primary[key][s]
            L.append(f"| {s} | {key} | " + (f"{t['diff']:+.4f} [{t['ci95'][0]:+.4f}, {t['ci95'][1]:+.4f}] | {t['p']:.3g} | {t['p_holm']:.3g} |"
                                            if t["computable"] else "not computable | – | – |"))
    open(os.path.join(args.out, "table.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))
    print(json.dumps(flags, indent=1))


if __name__ == "__main__":
    main()
