#!/usr/bin/env python3
"""Phase 10 D4 weighting ablation: aggregate (pre-registered analysis, docs/PLAN.md "Phase 10 D4 ablation pre-registration").

Input: runs/<date>_<seq>_<M2F|M3>-<heur|hess|cal>_p10d4_rNN/analysis/phase9_run.json (phase9_run_metrics.py, method label
"M2F-heur", ...). Failed runs (pre-registered reset rule) are counted and excluded.
Primary family A (Holm over 8 tests): hessian_calibrated vs heuristic, per method (M2F, M3), on V1_02 and V2_02, for
  4-DoF ATE and map-frame ATE; Welch two-sided; diff = cal - heur with 95 % CI.
Primary family B (Holm over 2 tests): M3 vs M2F under hessian_calibrated, map-frame ATE, on V1_02 and V2_02;
  diff = M3 - M2F with 95 % CI.
Interpretation: significant after Holm (alpha 0.05) = supported; otherwise report the CI, no claim.
Secondary / descriptive (no correction, labelled): every comparison involving the raw Hessian arm; everything on V1_01
  (in-sample for the calibration); M3 vs M2F under heuristic and hessian; relative displacement; registration counts,
  drops, latencies, callback times, CPU.
Usage (venv): python analysis/phase10_d4_aggregate.py  ->  runs/phase10_d4/aggregate.json, table.md
"""
import glob
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "analysis"))
from phase9_aggregate import ms, welch, holm  # noqa: E402

SEQS = ["V1_01_easy", "V1_02_medium", "V2_02_medium"]
HELD_OUT = ["V1_02_medium", "V2_02_medium"]
METHODS = ["M2F", "M3"]
INFOS = ["heur", "hess", "cal"]
METRICS = ["ate_4dof_m", "map_frame_ate_m"]
DESCR = ["rel_disp_m_mean", "published", "dropped_by_queue", "accepted", "rejected", "added", "cpu_svo_node_pct_mean",
         "cpu_reg_node_pct_mean", "registration_load", "frontend_rtf", "rtf_playback"]
OUT = os.path.join(REPO, "runs/phase10_d4")


def main():
    runs = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(REPO, "runs", "*_p10d4_r[0-9][0-9]", "analysis/phase9_run.json")))]
    arms = [f"{m}-{i}" for m in METHODS for i in INFOS]
    G = {(s, a): [r for r in runs if r["sequence"] == s and r["method"] == a] for s in SEQS for a in arms}
    ok = {k: [r for r in v if not r["failed"]] for k, v in G.items()}
    col = lambda k, key: [r.get(key) for r in ok[k]]

    per = {}
    for (s, a), rs in G.items():
        e = dict(runs=len(rs), failed=sum(r["failed"] for r in rs), failure_reasons=sorted({r["failure_reason"] for r in rs if r["failed"]}))
        for key in METRICS + DESCR:
            e[key] = ms(col((s, a), key))
        e["latency_ms_median"] = ms([(r.get("latency_ms") or {}).get("median") for r in ok[(s, a)]])
        e["callback_ms_median"] = ms([(r.get("callback_ms") or {}).get("median") for r in ok[(s, a)]])
        per[f"{s}/{a}"] = e

    fam_a = []
    for m in METHODS:
        for s in HELD_OUT:
            for key in METRICS:
                t = welch(col((s, f"{m}-heur"), key), col((s, f"{m}-cal"), key))
                fam_a.append(dict(test=f"{m}: cal vs heur, {s}, {key}", **t))
    for t, p in zip(fam_a, holm([t.get("p") if t["computable"] else None for t in fam_a])):
        t["p_holm"] = p
        t["supported_holm_0.05"] = bool(p is not None and p < 0.05)
    fam_b = []
    for s in HELD_OUT:
        t = welch(col((s, "M2F-cal"), "map_frame_ate_m"), col((s, "M3-cal"), "map_frame_ate_m"))
        fam_b.append(dict(test=f"cal: M3 vs M2F, {s}, map_frame_ate_m", **t))
    for t, p in zip(fam_b, holm([t.get("p") if t["computable"] else None for t in fam_b])):
        t["p_holm"] = p
        t["supported_holm_0.05"] = bool(p is not None and p < 0.05)

    sec = []
    for s in SEQS:
        for m in METHODS:
            for key in METRICS + ["rel_disp_m_mean"]:
                sec.append(dict(test=f"{m}: hess vs heur, {s}, {key}", **welch(col((s, f"{m}-heur"), key), col((s, f"{m}-hess"), key))))
                sec.append(dict(test=f"{m}: cal vs hess, {s}, {key}", **welch(col((s, f"{m}-hess"), key), col((s, f"{m}-cal"), key))))
                if s == "V1_01_easy" or key == "rel_disp_m_mean":
                    sec.append(dict(test=f"{m}: cal vs heur, {s}, {key}", **welch(col((s, f"{m}-heur"), key), col((s, f"{m}-cal"), key))))
        for i in INFOS:
            for key in METRICS:
                if i == "cal" and key == "map_frame_ate_m" and s in HELD_OUT:
                    continue  # primary family B
                sec.append(dict(test=f"{i}: M3 vs M2F, {s}, {key}", **welch(col((s, f"M2F-{i}"), key), col((s, f"M3-{i}"), key))))

    out = dict(n_runs=len(runs), per_sequence_arm=per, primary_family_A=fam_a, primary_family_B=fam_b,
               secondary_descriptive_uncorrected=sec)
    json.dump(out, open(os.path.join(OUT, "aggregate.json"), "w"), indent=2)

    f = lambda e: "–" if not e else (f"{e['mean']:.3f} ± {e['std']:.3f}" if e["std"] is not None else f"{e['mean']:.3f}")
    L = ["| Sequence | Arm | runs (failed) | map-frame ATE [m] | 4-DoF ATE [m] | rel. disp. [m] | accepted / published | dropped | latency [ms] |",
         "|---|---|---|---|---|---|---|---|---|"]
    for s in SEQS:
        for a in arms:
            e = per[f"{s}/{a}"]
            L.append(f"| {s} | {a} | {e['runs']} ({e['failed']}) | {f(e['map_frame_ate_m'])} | {f(e['ate_4dof_m'])} | {f(e['rel_disp_m_mean'])} | "
                     f"{f(e['accepted'])} / {f(e['published'])} | {f(e['dropped_by_queue'])} | {f(e['latency_ms_median'])} |")
    for name, fam, n in (("A", fam_a, 8), ("B", fam_b, 2)):
        L += ["", f"| Primary family {name} (Holm over {n}) | diff [m] (95 % CI) | p (Welch) | p (Holm) | supported |", "|---|---|---|---|---|"]
        for t in fam:
            L.append(f"| {t['test']} | " + (f"{t['diff']:+.4f} [{t['ci95'][0]:+.4f}, {t['ci95'][1]:+.4f}] | {t['p']:.3g} | {t['p_holm']:.3g} | {'yes' if t['supported_holm_0.05'] else 'no'} |"
                                            if t["computable"] else "not computable | – | – | – |"))
    open(os.path.join(OUT, "table.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
