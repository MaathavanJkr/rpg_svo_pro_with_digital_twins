#!/usr/bin/env python3
"""Decision D1: one-time 4-DoF initial alignment T_M_W from ground truth (EuRoC), and the
map-frame (unaligned) ATE that follows from it.

Definition (must be matched numerically by the Phase 5 online implementation):
  t0      = stamp of the first keyframe after the VIO backend has initialised (scale initialised).
  T_W_B   = VIO estimate at exactly t0 (a sample of the estimate stream with that stamp).
  T_V_B   = ground truth at t0: position linearly interpolated, orientation slerped between the
            two bracketing GT samples (state_groundtruth_estimate0, 200 Hz).
  dR      = R_V_B(t0) @ R_W_B(t0)^T                  (rotation from W to V/M implied at t0)
  psi     = atan2(dR[1,0], dR[0,0])                    (yaw part only; roll/pitch of W kept from VIO)
  R_M_W   = Rz(psi)
  t_M_W   = p_V_B(t0) - R_M_W @ p_W_B(t0)
  M := V for EuRoC (scan frame = Vicon frame, docs/frames.md).
Map-frame ATE: every estimate with stamp >= t0 is mapped by T_M_W and compared with GT
(nearest GT sample within 3 ms), with no further alignment.

Usage (venv):
  python analysis/initial_alignment.py runs/<run_id> <sequence> --t0 <stamp_s>
  python analysis/initial_alignment.py runs/<run_id> <sequence> --kf-log <csv with stamp,is_kf,scale_init>
Writes runs/<run_id>/analysis/initial_alignment.json and map_frame_ate.json.
"""
import argparse
import json
import os
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_gt(seq):
    d = np.loadtxt(os.path.join(REPO, "dataset/derived/asl_meta", seq,
                                "mav0/state_groundtruth_estimate0/data.csv"), delimiter=",", comments="#")
    t = d[:, 0] * 1e-9
    q_xyzw = np.c_[d[:, 5:8], d[:, 4]]
    return t, d[:, 1:4], Rotation.from_quat(q_xyzw)


def load_tum(path):
    d = np.loadtxt(path, comments="#")
    return d[:, 0], d[:, 1:4], Rotation.from_quat(d[:, 4:8])


def gt_at(t_gt, p_gt, r_gt, t0):
    i = np.searchsorted(t_gt, t0)
    if i == 0 or i >= len(t_gt):
        raise ValueError("t0 outside ground-truth range")
    a = (t0 - t_gt[i - 1]) / (t_gt[i] - t_gt[i - 1])
    p = (1 - a) * p_gt[i - 1] + a * p_gt[i]
    r = Slerp([t_gt[i - 1], t_gt[i]], r_gt[[i - 1, i]])([t0])[0]
    return p, r


def compute_T_M_W(p_WB, R_WB, p_VB, R_VB):
    dR = R_VB @ R_WB.T
    psi = np.arctan2(dR[1, 0], dR[0, 0])
    c, s = np.cos(psi), np.sin(psi)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p_VB - R @ p_WB
    return T, psi


def t0_from_kf_log(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    ok = (d["is_kf"] > 0) & (d["scale_init"] > 0)
    if not ok.any():
        raise ValueError("no keyframe after backend initialisation in " + path)
    return float(d["stamp"][ok][0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("seq")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--t0", type=float)
    g.add_argument("--kf-log")
    ap.add_argument("--est", default=None, help="TUM file of the VIO-frame estimate (default analysis/est.tum)")
    args = ap.parse_args()
    out = os.path.join(args.run, "analysis")
    t0 = args.t0 if args.t0 is not None else t0_from_kf_log(args.kf_log)

    t_e, p_e, r_e = load_tum(args.est or os.path.join(out, "est.tum"))
    k = int(np.argmin(np.abs(t_e - t0)))
    if abs(t_e[k] - t0) > 1e-6:
        raise ValueError(f"no estimate sample at t0={t0:.9f} (closest {t_e[k]:.9f})")
    t_g, p_g, r_g = load_gt(args.seq)
    p_VB, r_VB = gt_at(t_g, p_g, r_g, t0)
    T, psi = compute_T_M_W(p_e[k], r_e[k].as_matrix(), p_VB, r_VB.as_matrix())
    res = dict(sequence=args.seq, t0=t0, yaw_deg=float(np.degrees(psi)), T_M_W=T.tolist(),
               p_W_B_t0=p_e[k].tolist(), p_V_B_t0=p_VB.tolist(),
               note="4-DoF (yaw+translation) from GT at a single timestamp; see docs/frames.md (D1)")
    json.dump(res, open(os.path.join(out, "initial_alignment.json"), "w"), indent=2)

    # Map-frame ATE, no further alignment, window [t0, end].
    m = t_e >= t0
    p_M = p_e[m] @ T[:3, :3].T + T[:3, 3]
    j = np.clip(np.searchsorted(t_g, t_e[m]), 1, len(t_g) - 1)
    j = np.where(np.abs(t_g[j - 1] - t_e[m]) < np.abs(t_g[j] - t_e[m]), j - 1, j)
    ok = np.abs(t_g[j] - t_e[m]) < 0.003
    err = np.linalg.norm(p_M[ok] - p_g[j[ok]], axis=1)
    ate = dict(rmse=float(np.sqrt(np.mean(err ** 2))), mean=float(err.mean()), median=float(np.median(err)),
               max=float(err.max()), final=float(err[-1]), n=int(ok.sum()),
               window_s=float(t_e[m][ok][-1] - t0))
    json.dump(dict(t0=t0, map_frame_ate_m=ate), open(os.path.join(out, "map_frame_ate.json"), "w"), indent=2)
    print(json.dumps(dict(alignment=res, map_frame_ate_m=ate), indent=2))


if __name__ == "__main__":
    main()
