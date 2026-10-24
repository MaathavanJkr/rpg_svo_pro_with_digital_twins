#!/usr/bin/env python3
"""Phase 9 figure: per-run map-frame ATE and 4-DoF ATE (non-failed runs) for M1/M2/M3 on each sequence,
with the mean as a short bar and the number of valid runs under each group.
Usage (venv): python analysis/phase9_figures.py  ->  docs/img/phase9_ate.png
"""
import glob
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEQS = ["V1_01_easy", "V1_02_medium", "V2_02_medium", "V2_03_difficult"]
METHODS = [("M1", "#2a78d6", "o", "M1 VIO only"), ("M2", "#eb6834", "s", "M2 point-to-plane"),
           ("M3", "#1baf7a", "^", "M3 distance field")]
TEXT, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def main():
    runs = [json.load(open(f)) for f in glob.glob(os.path.join(REPO, "runs/*_p9_r[0-9][0-9]/analysis/phase9_run.json"))]
    fig, axs = plt.subplots(1, 2, figsize=(15, 5.6))
    fig.patch.set_facecolor("#fcfcfb")
    rng = np.random.default_rng(0)
    for ax, (key, title) in zip(axs, (("map_frame_ate_m", "Map-frame ATE (no alignment, after the D1 window)"),
                                      ("ate_4dof_m", "ATE after 4-DoF alignment"))):
        ax.set_facecolor("#fcfcfb")
        for si, s in enumerate(SEQS):
            for mi, (m, c, mk, lab) in enumerate(METHODS):
                g = [r for r in runs if r["sequence"] == s and r["method"] == m]
                v = np.array([r[key] for r in g if not r["failed"] and r.get(key) is not None])
                x = si + (mi - 1) * 0.26
                if len(v):
                    ax.scatter(x + rng.uniform(-0.06, 0.06, len(v)), v, s=36, marker=mk, color=c, edgecolor="#fcfcfb",
                               linewidth=1.0, zorder=3, label=lab if si == 0 else None)
                    ax.plot([x - 0.1, x + 0.1], [v.mean()] * 2, color=TEXT, lw=2, zorder=4)
                ax.text(x, -0.02, f"{len(v)}/{len(g)}", ha="center", va="top", fontsize=8, color=MUTED,
                        transform=ax.get_xaxis_transform())
        ax.set_xticks(range(len(SEQS)))
        ax.set_xticklabels(SEQS, color=TEXT)
        ax.tick_params(axis="x", pad=16, length=0)
        ax.set_ylabel("RMSE [m]", color=TEXT)
        ax.set_title(title, color=TEXT, loc="left")
        ax.grid(axis="y", color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color(GRID)
        ax.set_ylim(bottom=0)
    axs[0].legend(frameon=False, loc="upper left")
    fig.text(0.01, -0.03, "Dots: non-failed runs (pre-registered reset rule); bar: mean. Numbers: valid / total runs. "
             "Online runs, frozen build e7ebf29, playback rate 1.0.", fontsize=8, color=MUTED)
    out = os.path.join(REPO, "docs/img/phase9_ate.png")
    fig.savefig(out, dpi=120, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(out)


if __name__ == "__main__":
    main()
