#!/usr/bin/env python3
"""Decision D1: one-time 4-DoF initial alignment T_M_W from ground truth (EuRoC), and the
map-frame (unaligned) ATE that follows from it.

CURRENT DEFINITION, D1(B), 2026-10-23 ("initial alignment from GT positions over the first L m of
flight"); must be matched numerically by the online implementation in svo_ros (d1_alignment):
  t0      = first keyframe with backend AND scale initialised AND GT available (unchanged, below).
  window  = estimate samples (/svo/vio_backend_pose_imu) from t0 up to and including the first
            sample at which the GT path length reaches L (default L = 2.0 m). The path length is
            the polyline through the GT positions interpolated at the kept sample stamps. Samples
            without a <= 10 ms GT bracket are skipped.
  T_M_W   = 4-DoF least-squares fit (yaw about z + 3-D translation, closed form) of the VIO
            positions to the GT positions over the window. GT orientation is NOT used.
  Map-frame ATE window: estimate samples after the window end (t > t_end), no further alignment.
  (GT orientation on V1 is unreliable by several degrees, docs/phase4_m1.md and phase5_m2_euroc.md.)

LEGACY DEFINITION (superseded; --legacy-orientation): orientation of GT at t0 only:
  t0      = stamp of the first keyframe with backend AND scale both initialised AND ground truth
            available at its timestamp (frame-event log from svo_node, log_frame_events:=true).
            "GT available" = t0 lies between two GT samples (state_groundtruth_estimate0) that are
            at most GT_MAX_GAP = 10 ms apart: t_a <= t0 <= t_b, t_b - t_a <= 0.010 s.
            (Amended 2026-10-23: on EuRoC the first such KF without the GT condition can precede GT.)
  T_W_B   = VIO estimate at exactly t0 (a sample of the estimate stream with that stamp).
  T_V_B   = ground truth at t0, interpolated between that same bracketing pair: position linear,
            orientation slerp.
  dR      = R_V_B(t0) @ R_W_B(t0)^T                  (rotation from W to V/M implied at t0)
  psi     = atan2(dR[1,0], dR[0,0])                    (yaw part only; roll/pitch of W kept from VIO)
  R_M_W   = Rz(psi)
  t_M_W   = p_V_B(t0) - R_M_W @ p_W_B(t0)
  M := V for EuRoC (scan frame = Vicon frame, docs/frames.md).
  Legacy map-frame ATE window: every estimate with stamp >= t0.

Usage (venv):
  python analysis/initial_alignment.py runs/<run_id> <sequence> --t0 <stamp_s>
  python analysis/initial_alignment.py runs/<run_id> <sequence> --kf-log runs/<run_id>/frame_events.csv
  (frame_events.csv is written by svo_node with log_frame_events:=true)
Writes runs/<run_id>/analysis/initial_alignment.json and map_frame_ate.json.

Sensitivity check only (NOT the D1 definition): --yaw-source vicon takes the orientation at t0
from raw Vicon (mav0/vicon0, body orientation R_RS * R_BS^T, slerp at t0 on raw stamps) instead of
state_groundtruth_estimate0; translation still from the GT estimate. Outputs get the suffix
_sens_vicon_yaw and never overwrite the D1 files.
"""
import argparse
import json
import os
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GT_MAX_GAP = 0.010  # s; max spacing of the GT samples bracketing t0


def load_gt(seq):
    d = np.loadtxt(os.path.join(REPO, "dataset/derived/asl_meta", seq,
                                "mav0/state_groundtruth_estimate0/data.csv"), delimiter=",", comments="#")
    t = d[:, 0] * 1e-9
    q_xyzw = np.c_[d[:, 5:8], d[:, 4]]
    return t, d[:, 1:4], Rotation.from_quat(q_xyzw)


def load_tum(path):
    d = np.loadtxt(path, comments="#")
    return d[:, 0], d[:, 1:4], Rotation.from_quat(d[:, 4:8])


def gt_bracket(t_gt, t):
    """Index i of the bracketing GT pair (i-1, i) with t_gt[i-1] <= t <= t_gt[i] and spacing
    <= GT_MAX_GAP, or None if GT is not available at t."""
    i = int(np.searchsorted(t_gt, t))
    if i < len(t_gt) and t_gt[i] == t:
        i = max(i, 1)
    if i == 0 or i >= len(t_gt) or t_gt[i] - t_gt[i - 1] > GT_MAX_GAP:
        return None
    return i


def gt_at(t_gt, p_gt, r_gt, t0):
    i = gt_bracket(t_gt, t0)
    if i is None:
        raise ValueError(f"ground truth not available at t0={t0:.9f}")
    a = (t0 - t_gt[i - 1]) / (t_gt[i] - t_gt[i - 1])
    p = (1 - a) * p_gt[i - 1] + a * p_gt[i]
    r = Slerp([t_gt[i - 1], t_gt[i]], r_gt[[i - 1, i]])([t0])[0]
    return p, r


def vicon_orientation_at(seq, t0):
    b = os.path.join(REPO, "dataset/derived/asl_meta", seq, "mav0/vicon0/")
    v = np.loadtxt(b + "data.csv", delimiter=",", comments="#")
    T_BS = np.array(yaml.safe_load(open(b + "sensor.yaml"))["T_BS"]["data"]).reshape(4, 4)
    R_RB = Rotation.from_quat(np.c_[v[:, 5:8], v[:, 4]]) * Rotation.from_matrix(T_BS[:3, :3]).inv()
    return Slerp(v[:, 0] * 1e-9, R_RB)([t0])[0]


def compute_T_M_W(p_WB, R_WB, p_VB, R_VB):
    dR = R_VB @ R_WB.T
    psi = np.arctan2(dR[1, 0], dR[0, 0])
    c, s = np.cos(psi), np.sin(psi)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p_VB - R @ p_WB
    return T, psi


def window_alignment(t_e, p_e, t_gt, p_gt, t0, L=None, T=None):
    """D1(B) candidate: 4-DoF least-squares fit of VIO positions to GT positions over a window that
    starts at t0 (an estimate sample) and ends at the first estimate sample where the GT path length
    (polyline through GT positions interpolated at the kept estimate stamps) reaches L metres
    (or, alternatively, where the elapsed time reaches T seconds). GT orientation is not used.
    Estimate samples without a <= 10 ms GT bracket are skipped. Returns (T_M_W, psi, info)."""
    k0 = int(np.argmin(np.abs(t_e - t0)))
    if abs(t_e[k0] - t0) > 1e-6:
        raise ValueError(f"no estimate sample at t0={t0:.9f}")
    idx, g, s = [], [], [0.0]
    for k in range(k0, len(t_e)):
        i = gt_bracket(t_gt, t_e[k])
        if i is None:
            continue
        a = (t_e[k] - t_gt[i - 1]) / (t_gt[i] - t_gt[i - 1])
        gk = (1 - a) * p_gt[i - 1] + a * p_gt[i]
        if g:
            s.append(s[-1] + float(np.linalg.norm(gk - g[-1])))
        idx.append(k); g.append(gk)
        if (L is not None and s[-1] >= L) or (T is not None and t_e[k] - t0 >= T):
            break
    else:
        raise ValueError("window end (L/T) not reached before the end of the estimate stream")
    pe, pg = p_e[idx], np.array(g)
    me, mg = pe.mean(0), pg.mean(0)
    ae, bg = pe - me, pg - mg
    psi = np.arctan2(np.sum(ae[:, 0] * bg[:, 1] - ae[:, 1] * bg[:, 0]),
                     np.sum(ae[:, 0] * bg[:, 0] + ae[:, 1] * bg[:, 1]))
    c, sn = np.cos(psi), np.sin(psi)
    R = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
    Tm = np.eye(4); Tm[:3, :3] = R; Tm[:3, 3] = mg - R @ me
    info = dict(n_samples=len(idx), t_end=float(t_e[idx[-1]]), duration_s=float(t_e[idx[-1]] - t0),
                gt_path_length_m=float(s[-1]))
    return Tm, psi, info


def t0_from_kf_log(path, t_gt):
    """t0 = first keyframe with backend and scale both initialised and GT available at its stamp
    (rows where the processed bundle is the current image, i.e. frame_stamp_ns == stamp_ns)."""
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=np.int64)
    ok = ((d["frame_stamp_ns"] == d["stamp_ns"]) & (d["is_kf"] > 0)
          & (d["backend_init"] > 0) & (d["scale_init"] > 0))
    for ns in d["frame_stamp_ns"][ok]:
        t = int(ns) * 1e-9
        if gt_bracket(t_gt, t) is not None:
            return t
    raise ValueError("no keyframe with backend+scale initialised and GT available in " + path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("seq")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--t0", type=float)
    g.add_argument("--kf-log")
    ap.add_argument("--est", default=None, help="TUM file of the VIO-frame estimate (default analysis/est.tum)")
    ap.add_argument("--window-length", type=float, default=2.0, help="D1(B) window L [m] (default 2.0)")
    ap.add_argument("--legacy-orientation", action="store_true",
                    help="superseded D1: GT orientation at t0 (outputs get suffix _legacy_orientation)")
    ap.add_argument("--yaw-source", choices=["gt", "vicon"], default="gt",
                    help="legacy mode only: gt, or vicon = sensitivity check")
    args = ap.parse_args()
    out = os.path.join(args.run, "analysis")
    t_g, p_g, r_g = load_gt(args.seq)
    t0 = args.t0 if args.t0 is not None else t0_from_kf_log(args.kf_log, t_g)

    t_e, p_e, r_e = load_tum(args.est or os.path.join(out, "est.tum"))
    k = int(np.argmin(np.abs(t_e - t0)))
    if abs(t_e[k] - t0) > 1e-6:
        raise ValueError(f"no estimate sample at t0={t0:.9f} (closest {t_e[k]:.9f})")
    i = gt_bracket(t_g, t0)
    if args.legacy_orientation:
        p_VB, r_VB = gt_at(t_g, p_g, r_g, t0)
        suffix = "_legacy_orientation"
        if args.yaw_source == "vicon":
            r_VB = vicon_orientation_at(args.seq, t0)
            suffix = "_sens_vicon_yaw"
        T, psi = compute_T_M_W(p_e[k], r_e[k].as_matrix(), p_VB, r_VB.as_matrix())
        t_eval = t0
        res = dict(sequence=args.seq, t0=t0, rule="LEGACY: GT orientation at t0 (superseded)",
                   gt_bracket=[float(t_g[i - 1]), float(t_g[i])], yaw_deg=float(np.degrees(psi)), T_M_W=T.tolist(),
                   p_W_B_t0=p_e[k].tolist(), p_V_B_t0=p_VB.tolist())
    else:
        T, psi, info = window_alignment(t_e, p_e, t_g, p_g, t0, L=args.window_length)
        suffix = ""
        t_eval = np.nextafter(info["t_end"], np.inf)  # strictly after the window end
        res = dict(sequence=args.seq, t0=t0, rule="D1(B): 4-DoF LSQ fit of VIO to GT positions from t0 over the first L m of GT path",
                   window_length_m=args.window_length, window=info, yaw_deg=float(np.degrees(psi)), T_M_W=T.tolist(),
                   t0_rule="first KF with backend+scale init and GT bracket <= 10 ms")
    res["yaw_source"] = args.yaw_source
    json.dump(res, open(os.path.join(out, f"initial_alignment{suffix}.json"), "w"), indent=2)

    # Map-frame ATE, no further alignment, window [t_eval, end].
    m = t_e >= t_eval
    p_M = p_e[m] @ T[:3, :3].T + T[:3, 3]
    j = np.clip(np.searchsorted(t_g, t_e[m]), 1, len(t_g) - 1)
    j = np.where(np.abs(t_g[j - 1] - t_e[m]) < np.abs(t_g[j] - t_e[m]), j - 1, j)
    ok = np.abs(t_g[j] - t_e[m]) < 0.003
    err = np.linalg.norm(p_M[ok] - p_g[j[ok]], axis=1)
    ate = dict(rmse=float(np.sqrt(np.mean(err ** 2))), mean=float(err.mean()), median=float(np.median(err)),
               max=float(err.max()), final=float(err[-1]), n=int(ok.sum()),
               window_start=float(t_e[m][ok][0]), window_s=float(t_e[m][ok][-1] - t_e[m][ok][0]))
    json.dump(dict(t0=t0, map_frame_ate_m=ate), open(os.path.join(out, f"map_frame_ate{suffix}.json"), "w"), indent=2)
    print(json.dumps(dict(alignment=res, map_frame_ate_m=ate), indent=2))


if __name__ == "__main__":
    main()
