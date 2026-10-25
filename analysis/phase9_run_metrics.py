#!/usr/bin/env python3
"""Phase 9: per-run status and system metrics (after eval_vio.py, initial_alignment.py and, for M2/M3,
phase6_events.py have run on the run folder). Definitions are pre-registered in docs/PLAN.md (Phase 9).

Failure (pre-registered reset rule + harness checks), first matching reason wins:
  harness   : meta.txt has a FAILED line or no "done" line (node died, external session, abort)
  reset     : any SVO full reset (stage Relocalization -> Paused) in frame_events.csv
  no_map    : no map-frame ATE could be computed (no D1 alignment / map_frame_ate.json missing)
  registration methods only (M2, M2F, M3), no_online_d1: the online D1 alignment did not fire (d1_alignment.csv missing or empty)
Failed runs are counted per method and sequence and excluded from ATE / RPE / drift statistics.

Metrics written to runs/<run>/analysis/phase9_run.json:
  ATE 4-DoF (eval_vio), map-frame ATE (initial_alignment, t > t_end), relative displacement error
  (mean over the RPE segment lengths), D1 online-vs-offline yaw difference (M2/M3).
  Registration (M2, M2F, M3): published / processed / dropped_by_queue / accepted / rejected / added
  (phase6_events), callback time per KF (registration.csv: preprocessing + registration + information;
  median, p90), latency per accepted KF = svo_node receipt wall time of the registered pose
  (registration_events.csv) - wall time at which rosbag record received the keyframe cloud (run.bag;
  same machine clock; median, p90).
  CPU (cpu.csv, samples while svo_node > 5 % of one core): svo_node and registration node, mean % of one core.
  Real-time factors: rtf_playback = play wall time / bag duration (rate 1.0, a check that playback was
  not slowed); frontend_rtf = mean frontend time / 50 ms (eval_vio); registration_load = sum of
  registration callback times / bag duration; frames_dropped_pct (eval_vio).
Usage (venv + ROS): python analysis/phase9_run_metrics.py runs/<run_id> <sequence> <M1|M2|M2F|M3>
"""
import csv
import json
import os
import re
import sys
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "analysis"))
import reset_survey  # noqa: E402


def load(path):
    return json.load(open(path)) if os.path.exists(path) else None


def q(x):
    x = np.asarray(x, dtype=float)
    return dict(n=int(len(x)), median=float(np.median(x)), p90=float(np.percentile(x, 90))) if len(x) else None


def main(run, seq, method):
    out = dict(run=os.path.basename(run.rstrip("/")), sequence=seq, method=method)
    meta = open(os.path.join(run, "meta.txt")).read()
    an = os.path.join(run, "analysis")
    summ, mfa, ia = load(os.path.join(an, "summary.json")), load(os.path.join(an, "map_frame_ate.json")), load(os.path.join(an, "initial_alignment.json"))
    rs = reset_survey.survey(run) if os.path.exists(os.path.join(run, "frame_events.csv")) else None
    out["resets"] = rs
    d1_online = None
    d1p = os.path.join(run, "d1_alignment.csv")
    if os.path.exists(d1p):
        rows = list(csv.DictReader(open(d1p)))
        d1_online = rows[-1] if rows else None
    reason = None
    if re.search(r"^FAILED", meta, re.M) or not re.search(r"^done$", meta, re.M):
        reason = "harness: " + "; ".join(re.findall(r"^(?:FAILED|ABORT).*$", meta, re.M) or ["no 'done' line"])
    elif rs is None or rs["n_resets"] > 0:
        reason = "reset" if rs else "no frame_events.csv"
    elif mfa is None or summ is None:
        reason = "no_map"
    elif method != "M1" and d1_online is None:
        reason = "no_online_d1"
    out["failed"] = reason is not None
    out["failure_reason"] = reason

    if summ:
        out["ate_4dof_m"] = summ["ate_posyaw_m"]["rmse"]
        out["rel_disp_m_mean"] = float(np.mean([v["rel_disp_m"]["mean"] if isinstance(v["rel_disp_m"], dict) else v["rel_disp_m"]
                                                for v in summ["rpe"].values()]))
        rt = summ["runtime"]
        out["frames_dropped_pct"] = rt["frames_dropped_pct"]
        out["frontend_ms_mean"] = rt["frontend_ms"]["mean"]
        out["frontend_rtf"] = rt["frontend_rtf"]
        out["backend_output_latency_ms_median"] = rt["backend_output_latency_ms"]["median"]
    if mfa:
        out["map_frame_ate_m"] = mfa["map_frame_ate_m"]["rmse"] if isinstance(mfa["map_frame_ate_m"], dict) else mfa["map_frame_ate_m"]
    if ia and d1_online:
        out["d1_yaw_offline_deg"] = ia["yaw_deg"]
        out["d1_yaw_online_minus_offline_deg"] = float(d1_online["yaw_deg"]) - ia["yaw_deg"]

    m = re.search(r"play_wall_seconds: ([0-9.]+)", meta)
    import rosbag
    seq_bag = next(os.path.join(REPO, "dataset", d, seq, seq + ".bag") for d in os.listdir(os.path.join(REPO, "dataset"))
                   if os.path.isdir(os.path.join(REPO, "dataset", d, seq)))
    with rosbag.Bag(seq_bag) as b:
        bag_s = b.get_end_time() - b.get_start_time()
    out["bag_duration_s"] = bag_s
    out["rtf_playback"] = float(m.group(1)) / bag_s if m else None

    cpu_p = os.path.join(run, "cpu.csv")
    if os.path.exists(cpu_p):
        rows = [r for r in csv.DictReader(open(cpu_p))]
        act = [r for r in rows if float(r["svo_node_pct_one_core"]) > 5]
        out["cpu_svo_node_pct_mean"] = float(np.mean([float(r["svo_node_pct_one_core"]) for r in act])) if act else None
        reg = [float(r["reg_node_pct_one_core"]) for r in act if r.get("reg_node_pct_one_core") not in (None, "")]
        out["cpu_reg_node_pct_mean"] = float(np.mean(reg)) if reg else None
        out["cpu_system_busy_pct_mean"] = float(np.mean([float(r["sys_busy_pct"]) for r in act])) if act else None

    if method != "M1":
        ev = load(os.path.join(an, "events_summary.json"))
        if ev:
            out.update({k: ev[k] for k in ("published", "processed", "dropped_by_queue", "accepted", "rejected", "added")})
        regp = os.path.join(run, "registration.csv")
        if os.path.exists(regp):
            rr = list(csv.DictReader(open(regp)))
            cb = [float(r["t_preproc_s"]) + float(r["t_icp_s"]) + float(r["t_info_s"]) for r in rr]
            out["callback_ms"] = {k: (v * 1e3 if k != "n" else v) for k, v in (q(cb) or {}).items()} or None
            out["registration_load"] = float(np.sum(cb)) / bag_s
            if rr and "iterations" in rr[0]:
                it = [float(r["iterations"]) for r in rr if r["iterations"] not in ("", None)]
                out["df_iterations_median"] = float(np.median(it)) if it else None
        # Latency: cloud received by rosbag record -> registered pose received by svo_node.
        rep = os.path.join(run, "registration_events.csv")
        if os.path.exists(rep):
            recv = {}
            for r in csv.DictReader(open(rep)):
                recv.setdefault(int(r["stamp_ns"]), int(r["wall_ns"]))
            pub = {}
            with rosbag.Bag(os.path.join(run, "run.bag")) as b:
                for _, msg, t in b.read_messages(topics=["/svo/w_backend_pose_points"]):
                    pub.setdefault(msg.header.stamp.to_nsec(), t.to_nsec())
            lat = [(recv[s] - pub[s]) * 1e-6 for s in recv if s in pub]
            out["latency_ms"] = q(lat)
    json.dump(out, open(os.path.join(an, "phase9_run.json"), "w"), indent=2)
    print(json.dumps({k: out[k] for k in out if k != "resets"}))


if __name__ == "__main__":
    main(*sys.argv[1:4])
