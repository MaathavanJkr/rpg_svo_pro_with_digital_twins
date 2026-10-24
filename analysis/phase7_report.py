#!/usr/bin/env python3
"""Phase 7 report: run-level statistics, block bootstrap CIs and plots for the offline study
(OFFLINE, OPEN-LOOP, inputs logged from M2 online runs).

Statistics (keyframes are not independent): every accuracy / jitter quantity is computed PER RUN
(n = 5) and compared to point-to-plane with a paired t-test at run level (same inputs per run) and
Welch's t-test; plus a moving-block bootstrap (blocks of BLOCK contiguous keyframes, resampled within
each run, B = 2000) for the DF - P2P difference. No keyframe-level p-values.

Usage (venv): python analysis/phase7_report.py
Writes runs/phase7/study/report.json and docs/img/phase7_*.png
"""
import csv
import glob
import json
import os
from collections import defaultdict
import numpy as np
from scipy import stats
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = os.path.join(REPO, "runs/phase7/study")
IMG = os.path.join(REPO, "docs/img")
BLOCK, B = 10, 2000
RNG = np.random.default_rng(0)
LABEL = "OFFLINE, open-loop, inputs from M2 runs (V1_01, 5 runs)"


def load(name):
    rows = list(csv.DictReader(open(os.path.join(ST, f"accuracy_{name}.csv"))))
    for r in rows:
        r["valid"] = r["valid"] == "True"
        for k, v in list(r.items()):
            if k not in ("run", "reason", "valid"):
                try:
                    r[k] = float(v)
                except (TypeError, ValueError):
                    r[k] = np.nan
    by = defaultdict(list)
    for r in rows:
        by[r["run"]].append(r)
    return {k: sorted(v, key=lambda r: r["stamp_ns"]) for k, v in by.items()}


def run_metrics(rs, mask=None):
    acc = [r for i, r in enumerate(rs) if r["valid"] and (mask is None or mask[i])]
    yaw = np.array([r["corr_yaw_deg"] for r in acc])
    dp = np.array([[r[f"dpos_{a}"] for a in "xyz"] for r in acc])
    need = np.array([[r[f"need_{a}"] for a in "xyz"] for r in acc])
    return dict(accept=len([r for r in rs if r["valid"]]) / len(rs),
                pos_err_median=float(np.median([r["err_m"] for r in acc])),
                pos_err_p90=float(np.percentile([r["err_m"] for r in acc], 90)),
                yaw_jitter=float(np.diff(yaw).std()), pos_jitter=float(np.linalg.norm(np.diff(dp, axis=0).std(0))),
                r_needed=float(np.corrcoef(dp.ravel(), need.ravel())[0, 1]),
                iterations_median=float(np.nanmedian([r["iterations"] for r in rs])),
                lookups_median=float(np.nanmedian([r["lookups"] for r in rs])),
                runtime_ms_median=1e3 * float(np.nanmedian([r["runtime_s"] for r in rs])))


def both_mask(a, b):
    return [ra["valid"] and rb["valid"] for ra, rb in zip(a, b)]


def block_bootstrap(A, Bm):
    """CI of DF - P2P for median position error and yaw jitter, both-accepted KFs, blocks within runs."""
    d_err, d_jit = [], []
    runs = sorted(A)
    seqs = {}
    for run in runs:
        m = both_mask(A[run], Bm[run])
        seqs[run] = [(ra, rb) for ra, rb, k in zip(A[run], Bm[run], m) if k]
    for _ in range(B):
        ea, eb, ja, jb = [], [], [], []
        for run in runs:
            s = seqs[run]
            nb = max(1, len(s) // BLOCK)
            starts = RNG.integers(0, max(1, len(s) - BLOCK + 1), nb)
            for st in starts:
                blk = s[st:st + BLOCK]
                ea += [x[0]["err_m"] for x in blk]; eb += [x[1]["err_m"] for x in blk]
                ya = np.array([x[0]["corr_yaw_deg"] for x in blk]); yb = np.array([x[1]["corr_yaw_deg"] for x in blk])
                ja += list(np.diff(ya)); jb += list(np.diff(yb))
        d_err.append(np.median(ea) - np.median(eb)); d_jit.append(np.std(ja) - np.std(jb))
    q = lambda x: [float(np.percentile(x, 2.5)), float(np.percentile(x, 97.5))]
    return dict(pos_err_median_diff_ci95=q(d_err), yaw_jitter_diff_ci95=q(d_jit))


def compare(name, ref="p2p_d0.5"):
    A, Bm = load(name), load(ref)
    runs = sorted(A)
    out = {}
    per = {run: dict(df=run_metrics(A[run]), p2p=run_metrics(Bm[run]),
                     df_both=run_metrics(A[run], both_mask(A[run], Bm[run])),
                     p2p_both=run_metrics(Bm[run], both_mask(A[run], Bm[run]))) for run in runs}
    for k in ("accept", "pos_err_median", "pos_err_p90", "yaw_jitter", "pos_jitter", "r_needed",
              "iterations_median", "lookups_median", "runtime_ms_median"):
        a = np.array([per[r]["df"][k] for r in runs]); b = np.array([per[r]["p2p"][k] for r in runs])
        out[k] = dict(cfg=[float(a.mean()), float(a.std(ddof=1))], p2p=[float(b.mean()), float(b.std(ddof=1))])
        if name != ref and np.std(a - b) > 0:
            out[k]["paired_p"] = float(stats.ttest_rel(a, b)[1]); out[k]["welch_p"] = float(stats.ttest_ind(a, b, equal_var=False)[1])
    for k in ("pos_err_median", "yaw_jitter"):
        a = np.array([per[r]["df_both"][k] for r in runs]); b = np.array([per[r]["p2p_both"][k] for r in runs])
        out[k + "_both_accepted"] = dict(cfg=[float(a.mean()), float(a.std(ddof=1))], p2p=[float(b.mean()), float(b.std(ddof=1))])
        if name != ref and np.std(a - b) > 0:
            out[k + "_both_accepted"]["paired_p"] = float(stats.ttest_rel(a, b)[1])
    if name != ref:
        out["block_bootstrap"] = block_bootstrap(A, Bm)
    allrows = [r for run in runs for r in A[run]]
    for p in ("infoheur", "infohess"):
        for k in ("pos_diag", "rot_diag", "min_eig", "cond"):
            out[f"{p}_{k}_median"] = float(np.nanmedian([r[f"{p}_{k}"] for r in allrows]))
    reasons = defaultdict(int)
    for r in allrows:
        if not r["valid"]:
            reasons[r["reason"]] += 1
    out["reject_reasons"] = dict(reasons)
    return out, per


def main():
    names = sorted(os.path.basename(p)[9:-4] for p in glob.glob(os.path.join(ST, "accuracy_*.csv")))
    rep = {"label": LABEL, "configs": {}}
    for n in names:
        rep["configs"][n], _ = compare(n)
    grids = {}
    for j in glob.glob(os.path.join(REPO, "dataset/derived/euroc_twin/df/*.json")):
        m = json.load(open(j)); grids[m["voxel_m"]] = dict(shape=m["shape"], n_voxels=m["n_voxels"],
                                                          memory_MB_float32=m["memory_MB_float32"], build_s=m["build_s"])
    rep["grids"] = grids
    basin = os.path.join(ST, "basin.csv")
    if os.path.exists(basin):
        rows = list(csv.DictReader(open(basin)))
        agg = defaultdict(lambda: [0, 0, 0, 0, 0])
        for r in rows:
            k = (r["method"], r["kind"], float(r["t_m"]), float(r["r_deg"]))
            a = agg[k]; a[0] += 1; a[1] += r["success"] == "True"; a[2] += r["same_solution"] == "True"
            a[3] += r["gt_ok"] == "True"; a[4] += r["accepted"] == "True"
        rep["basin"] = [dict(method=k[0], kind=k[1], t_m=k[2], r_deg=k[3], n=v[0], success=v[1] / v[0],
                             same_solution=v[2] / v[0], gt_ok=v[3] / v[0], accepted=v[4] / v[0]) for k, v in sorted(agg.items())]
        rep["basin_meta"] = json.load(open(os.path.join(ST, "basin_meta.json")))
    json.dump(rep, open(os.path.join(ST, "report.json"), "w"), indent=2)
    plots(rep)
    print(json.dumps({k: v for k, v in rep.items() if k not in ("configs", "basin")}, indent=1)[:2000])


def plots(rep):
    C = rep["configs"]
    # 1. factorial: median position error and yaw jitter (run-level mean +- std)
    fig, axs = plt.subplots(1, 3, figsize=(20, 5.5))
    order = ["p2p_d0.5"] + [f"df_v{v}_{t}_h0.05" for v in (50, 100, 200) for t in ("untrunc", "mu2v", "mu5v", "mu10v")]
    order = [o for o in order if o in C]
    for ax, k, lab in zip(axs, ("pos_err_median", "yaw_jitter", "accept"),
                          ("median position error vs GT [m]", "KF-to-KF yaw jitter [deg]", "accept rate")):
        m = [C[o][k]["cfg"][0] for o in order]; s = [C[o][k]["cfg"][1] for o in order]
        ax.bar(range(len(order)), m, yerr=s, color=["C1"] + ["C0"] * (len(order) - 1), capsize=3)
        ax.set_xticks(range(len(order))); ax.set_xticklabels([o.replace("_h0.05", "") for o in order], rotation=60, ha="right", fontsize=8)
        ax.set_ylabel(lab); ax.set_title(f"{lab} (run-level mean ± std, n = 5)")
    fig.suptitle(f"Distance-field variants vs point-to-plane: {LABEL}")
    fig.savefig(os.path.join(IMG, "phase7_variants.png"), dpi=110, bbox_inches="tight"); plt.close(fig)
    # 2. Huber sensitivity
    hs = [h for h in ("df_v50_untrunc_h0.02", "df_v50_untrunc_h0.05", "df_v50_untrunc_h0.10") if h in C]
    if hs:
        fig, axs = plt.subplots(1, 3, figsize=(15, 4))
        for ax, k in zip(axs, ("pos_err_median", "yaw_jitter", "accept")):
            ax.errorbar([0.02, 0.05, 0.10][:len(hs)], [C[h][k]["cfg"][0] for h in hs], yerr=[C[h][k]["cfg"][1] for h in hs], marker="o", capsize=3)
            ax.axhline(C["p2p_d0.5"][k]["p2p"][0], color="C1", ls="--", label="point-to-plane")
            ax.set_xlabel("Huber delta [m]"); ax.set_ylabel(k); ax.legend()
        fig.suptitle(f"Huber sensitivity (v = 0.05 m, untruncated; 0.05 m is the pre-set value): {LABEL}")
        fig.savefig(os.path.join(IMG, "phase7_huber.png"), dpi=110, bbox_inches="tight"); plt.close(fig)
    # 3. basin
    if "basin" in rep:
        fig, axs = plt.subplots(1, 4, figsize=(22, 5))
        kinds = (("translation", "t_m", "translation perturbation [m]"), ("rotation", "r_deg", "rotation perturbation [deg]"),
                 ("combined", "r_deg", "combined (t, r) - plotted vs r [deg]"), ("yaw", "r_deg", "yaw-only perturbation [deg]"))
        methods = sorted({b["method"] for b in rep["basin"]})
        for ax, (kind, x, lab) in zip(axs, kinds):
            for i, m in enumerate(methods):
                pts = sorted([(b[x], b["success"]) for b in rep["basin"] if b["method"] == m and b["kind"] == kind])
                ax.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", ls="--" if m.startswith("p2p") else "-", label=m)
            ax.set_xlabel(lab); ax.set_ylabel("success rate"); ax.set_ylim(-0.02, 1.02); ax.set_title(kind)
        axs[0].legend(fontsize=8)
        fig.suptitle(f"Convergence basin (success = same solution within {rep['basin_meta']['thresholds']['same_t_m']} m / "
                     f"{rep['basin_meta']['thresholds']['same_r_deg']} deg AND position within {rep['basin_meta']['thresholds']['gt_pos_m']} m of GT): {LABEL}")
        fig.savefig(os.path.join(IMG, "phase7_basin.png"), dpi=110, bbox_inches="tight"); plt.close(fig)
    # 4. information + runtime
    fig, axs = plt.subplots(1, 3, figsize=(18, 4.8))
    for ax, k, lab in zip(axs[:2], ("infohess_min_eig_median", "infoheur_cond_median"),
                          ("Hessian-based information: median min eigenvalue", "heuristic information: median condition number")):
        ax.bar(range(len(order)), [C[o][k] for o in order], color=["C1"] + ["C0"] * (len(order) - 1))
        ax.set_xticks(range(len(order))); ax.set_xticklabels([o.replace("_h0.05", "") for o in order], rotation=60, ha="right", fontsize=8)
        ax.set_yscale("log"); ax.set_title(lab)
    ax = axs[2]
    ax.bar(range(len(order)), [C[o]["iterations_median"]["cfg"][0] for o in order], color=["C1"] + ["C0"] * (len(order) - 1))
    ax.set_xticks(range(len(order))); ax.set_xticklabels([o.replace("_h0.05", "") for o in order], rotation=60, ha="right", fontsize=8)
    ax.set_title("median iterations (cap 100)")
    fig.suptitle(f"Information outputs and iterations: {LABEL}")
    fig.savefig(os.path.join(IMG, "phase7_info_iterations.png"), dpi=110, bbox_inches="tight"); plt.close(fig)


if __name__ == "__main__":
    main()
