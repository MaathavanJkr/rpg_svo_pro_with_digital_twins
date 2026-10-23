#!/usr/bin/env python3
"""Aggregate EuRoC runs of one method/sequence -> per-run table + mean ± std (sample std, ddof=1).

Needs, per run: analysis/summary.json (eval_vio.py) and, if present,
analysis/initial_alignment.json + analysis/map_frame_ate.json (initial_alignment.py).
Usage (venv): python analysis/aggregate_euroc.py <out.json> runs/<id1> runs/<id2> ...
"""
import json
import os
import sys
import numpy as np


def ms(v):
    v = np.asarray(v, float)
    return dict(mean=float(v.mean()), std=float(v.std(ddof=1)) if len(v) > 1 else 0.0,
                min=float(v.min()), max=float(v.max()), n=int(len(v)))


out_path, runs = sys.argv[1], sys.argv[2:]
rows = []
for r in runs:
    a = os.path.join(r, "analysis")
    s = json.load(open(os.path.join(a, "summary.json")))
    row = dict(run=os.path.basename(r.rstrip("/")),
               ate_posyaw=s["ate_posyaw_m"]["rmse"], ate_se3=s["ate_se3_m"]["rmse"],
               rpe_trans={k: v["trans_m"]["mean"] for k, v in s["rpe"].items()},
               rpe_rot={k: v["rot_deg"]["mean"] for k, v in s["rpe"].items()},
               rel_disp={k: v["rel_disp_m"]["mean"] for k, v in s["rpe"].items() if "rel_disp_m" in v},
               frames=f'{s["runtime"]["frames_processed"]}/{s["runtime"]["images_in_bag"]}',
               frontend_ms=s["runtime"]["frontend_ms"]["mean"],
               latency_ms=s["runtime"]["backend_output_latency_ms"]["median"],
               cpu_core_pct=s["runtime"].get("cpu", {}).get("svo_node_pct_one_core_mean"))
    if os.path.exists(os.path.join(a, "map_frame_ate.json")):
        ia = json.load(open(os.path.join(a, "initial_alignment.json")))
        mf = json.load(open(os.path.join(a, "map_frame_ate.json")))["map_frame_ate_m"]
        row.update(t0=ia["t0"], yaw_deg=ia["yaw_deg"], map_ate=mf["rmse"], map_ate_final=mf["final"])
    rows.append(row)

agg = dict(ate_posyaw=ms([r["ate_posyaw"] for r in rows]), ate_se3=ms([r["ate_se3"] for r in rows]),
           frontend_ms=ms([r["frontend_ms"] for r in rows]), latency_ms=ms([r["latency_ms"] for r in rows]))
if all("map_ate" in r for r in rows):
    agg["map_ate"] = ms([r["map_ate"] for r in rows])
    agg["yaw_deg"] = ms([r["yaw_deg"] for r in rows])
for key in ("rpe_trans", "rpe_rot", "rel_disp"):
    if all(r[key] for r in rows):
        agg[key] = {seg: ms([r[key][seg] for r in rows]) for seg in rows[0][key]}
if all(r["cpu_core_pct"] is not None for r in rows):
    agg["cpu_core_pct"] = ms([r["cpu_core_pct"] for r in rows])

json.dump(dict(runs=rows, aggregate=agg), open(out_path, "w"), indent=2)
print(f'{"run":36s} {"ATE4":>6s} {"mapATE":>7s} {"t0":>18s} {"yaw":>8s} {"fe ms":>6s} {"lat ms":>6s} {"cpu%":>5s} frames')
for r in rows:
    print(f'{r["run"]:36s} {r["ate_posyaw"]:6.3f} {r.get("map_ate", float("nan")):7.3f} {r.get("t0", float("nan")):18.6f} '
          f'{r.get("yaw_deg", float("nan")):8.2f} {r["frontend_ms"]:6.1f} {r["latency_ms"]:6.0f} {r["cpu_core_pct"] or 0:5.0f} {r["frames"]}')
f = lambda d: f'{d["mean"]:.3f} ± {d["std"]:.3f}'
print("ATE posyaw [m]:", f(agg["ate_posyaw"]), "| ATE SE3 [m]:", f(agg["ate_se3"]),
      "| map-frame ATE [m]:", f(agg["map_ate"]) if "map_ate" in agg else "n/a")
print("RPE trans mean [m]:", {k: f(v) for k, v in agg["rpe_trans"].items()})
print("RPE rot mean [deg]:", {k: f(v) for k, v in agg["rpe_rot"].items()})
if "rel_disp" in agg:
    print("Rel. displacement error mean [m] (orientation-free):", {k: f(v) for k, v in agg["rel_disp"].items()})
