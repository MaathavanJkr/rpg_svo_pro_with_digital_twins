#!/usr/bin/env python3
"""Phase 7: point-to-plane ICP (M2's method) offline, with iteration counting and both information outputs.

icp(): Open3D registration_icp, TransformationEstimationPointToPlane, init = I, the given correspondence
distance and iteration cap, Open3D's own default convergence criteria (relative fitness / RMSE 1e-6).
The iteration count is read from Open3D's debug log ("ICP Iteration #k"), captured at the C file
descriptor level, so the registration is run exactly once.

information_heuristic(): M2's formula (online_registration.py calculate_information_matrix +
normalisation + scale factors) on the correspondences of the final transform.
information_hessian(): point-to-plane Gauss-Newton Hessian at the final transform, J_i = n_i^T [-[q_i]x | I],
divided by sigma^2 = sum r_i^2 / (n - 6), reordered to [pos, rot] (D4 ablation).
"""
import contextlib
import ctypes
import io
import os
import re
import tempfile
import time
import numpy as np
import open3d as o3d

import df_register as dfr


class _CaptureStdout:
    """Capture Open3D's debug log. Open3D's Python bindings print it through Python's sys.stdout
    (C-level fd capture does not see it); C stdio is captured too, in case a build prints there."""

    def __enter__(self):
        self.buf = io.StringIO()
        self.redir = contextlib.redirect_stdout(self.buf)
        self.redir.__enter__()
        ctypes.CDLL(None).fflush(None)
        self.tmp = tempfile.TemporaryFile(mode="w+b")
        self.fd = os.dup(1)
        os.dup2(self.tmp.fileno(), 1)
        return self

    def __exit__(self, *a):
        ctypes.CDLL(None).fflush(None)
        os.dup2(self.fd, 1)
        os.close(self.fd)
        self.tmp.seek(0)
        c_text = self.tmp.read().decode(errors="replace")
        self.tmp.close()
        self.redir.__exit__(*a)
        self.text = self.buf.getvalue() + c_text


def icp(points, twin_pcd, max_corr=0.5, max_iter=100, count_iterations=True):
    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
    est = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    iters = None
    if count_iterations:
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        with _CaptureStdout() as cap:
            t0 = time.perf_counter()
            r = o3d.pipelines.registration.registration_icp(src, twin_pcd, max_corr, np.identity(4), est, crit)
            dt = time.perf_counter() - t0
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Warning)
        iters = len(re.findall(r"ICP Iteration #\d+", cap.text))
    else:
        t0 = time.perf_counter()
        r = o3d.pipelines.registration.registration_icp(src, twin_pcd, max_corr, np.identity(4), est, crit)
        dt = time.perf_counter() - t0
    return dict(T_c=np.asarray(r.transformation), fitness=r.fitness, inlier_rmse=r.inlier_rmse,
                correspondences=np.asarray(r.correspondence_set), iterations=iters, runtime_s=dt,
                lookups=None if iters is None else (iters + 1) * len(points))


def validate(res, cfg=dfr.CFG):
    T = res["T_c"]
    tn = float(np.linalg.norm(T[:3, 3]))
    ang = float(np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1))))
    reasons = []
    if res["fitness"] < cfg["fitness_threshold"]: reasons.append("fitness")
    if res["inlier_rmse"] > cfg["inliers_rmse_thr"]: reasons.append("rmse")
    if not reasons:
        if tn > cfg["max_translation_distance"]: reasons.append("translation")
        if ang > cfg["max_rotation_distance"]: reasons.append("rotation")
    return dict(valid=not reasons, reason="+".join(reasons), corr_t_m=tn, corr_rot_deg=ang)


def information_heuristic(points, res, twin_normals, cfg=dfr.CFG):
    c = res["correspondences"]
    if len(c) == 0:
        return np.full((6, 6), np.nan)
    a, n = points[c[:, 0]], twin_normals[c[:, 1]]
    Apn = np.zeros((6, 6))
    for a_i, n_i in zip(a, n):
        cr = np.cross(a_i, n_i)
        cn = np.linalg.norm(cr)
        if cn < 1e-12:
            continue
        H_i = np.vstack([-(cr / cn).reshape(-1, 1), -n_i.reshape(-1, 1)]).T
        Apn += H_i.T @ H_i
    A = np.zeros((6, 6))
    A[:3, :3] = Apn[3:, 3:]; A[3:, 3:] = Apn[:3, :3]; A[:3, 3:] = Apn[3:, :3]; A[3:, :3] = Apn[:3, 3:]
    A = np.linalg.matrix_power(A, cfg["information_matrix_power"])
    A = (A / np.max(A)) * np.exp(-0.9 * res["inlier_rmse"])
    sp, sr = cfg["scale_factor_position"], cfg["scale_factor_rotation"]
    A[:3, :3] *= sp * sp; A[3:, 3:] *= sr * sr; A[:3, 3:] *= sr * sp; A[3:, :3] *= sr * sp
    return A


def information_hessian(points, res, twin_points, twin_normals):
    c = res["correspondences"]
    if len(c) < 7:
        return np.full((6, 6), np.nan)
    T = res["T_c"]
    q = points[c[:, 0]] @ T[:3, :3].T + T[:3, 3]
    n = twin_normals[c[:, 1]]
    r = np.einsum("ij,ij->i", n, q - twin_points[c[:, 1]])
    J = np.concatenate([np.einsum("ni,nij->nj", n, -dfr.skew(q)), n], axis=1)  # (w, v)
    H = J.T @ J
    s2 = float(np.sum(r ** 2) / max(len(r) - 6, 1))
    P = np.zeros((6, 6)); P[:3, 3:] = np.eye(3); P[3:, :3] = np.eye(3)
    return P @ (H / max(s2, 1e-12)) @ P.T
