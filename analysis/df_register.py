#!/usr/bin/env python3
"""Phase 7: distance-field registration (offline prototype, the M3 candidate).

Same problem and conventions as M2 (online_registration_fyp.py):
  input   : local cloud in the map frame (after the node's filters) and the VIO keyframe pose
  unknown : a correction T_c applied on the left in the map frame (p' = R_c p + t_c), init = I
  output  : T_result = T_c @ T_init, validity verdict, information matrix [pos, rot]

Objective: E(T_c) = sum_i rho_Huber(D(R_c p_i + t_c)), D = trilinear interpolation of the
unsigned distance grid (optionally truncated at mu). Levenberg-Marquardt on the retraction
T_c <- (Exp(w), v) o T_c, Jacobian dD/dxi = grad D(p')^T [ -[p']x | I ] with xi = (w, v).
Points outside the grid are discarded and counted; truncated points (D >= mu) have zero gradient.

Validity: the M2 EuRoC checks (svo_registration/config/euroc_v1.yaml), with the ICP quantities
replaced by their distance-field analogues computed on the UNTRUNCATED grid for every variant:
  fitness     = fraction of all points with D < max_correspondence_distance (0.5 m)
                (Open3D ICP: fraction of source points with a correspondence within that distance)
  inlier_rmse = RMS of D over those points (Open3D: RMS of nearest-point distances; D is the
                nearest-twin-point distance up to grid interpolation)
  then fitness >= 0.75, rmse <= 0.15 m, |t_c| <= 0.75 m, angle(R_c) <= 5 deg (arccos clipped).
Information:
  heuristic : M2's exact formula (calculate_information_matrix + normalisation + scale factors) with
              the target normal replaced by grad D / |grad D| at the corrected inlier point
              (decision D4, main comparison)
  hessian   : J^T W J of the final iteration / sigma^2, sigma^2 = sum w r^2 / (n - 6); reordered to
              [pos, rot] (D4 ablation)
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation

CFG = dict(max_correspondence_distance=0.5, inliers_rmse_thr=0.15, fitness_threshold=0.75,
           max_translation_distance=0.75, max_rotation_distance=5.0,
           scale_factor_position=9.0, scale_factor_rotation=2.0, information_matrix_power=1)


def skew(p):
    z = np.zeros(len(p))
    return np.stack([np.stack([z, -p[:, 2], p[:, 1]], 1),
                     np.stack([p[:, 2], z, -p[:, 0]], 1),
                     np.stack([-p[:, 1], p[:, 0], z], 1)], 1)


def huber_w(r, delta):
    a = np.abs(r)
    return np.where(a <= delta, 1.0, delta / np.maximum(a, 1e-12))


def huber_cost(r, delta):
    a = np.abs(r)
    return np.sum(np.where(a <= delta, 0.5 * a ** 2, delta * (a - 0.5 * delta)))


def register(points, grid, mu=None, delta=0.05, max_iter=25, tol_step=1e-7, tol_cost=1e-9):
    """Returns dict with T_c (4x4), iterations, lookups, final Hessian (xi order w, v), stats."""
    t0 = time.perf_counter()
    R = np.eye(3); t = np.zeros(3)
    lam = 1e-3
    lookups = 0

    def evaluate(R, t):
        q = points @ R.T + t
        D, G, valid = grid.query(q, mu)
        return q, D, G, valid

    q, D, G, valid = evaluate(R, t); lookups += len(points)
    cost = huber_cost(D[valid], delta)
    it = 0
    H = np.zeros((6, 6))
    for it in range(1, max_iter + 1):
        qv, r, g = q[valid], D[valid], G[valid]
        J = np.einsum("ni,nij->nj", g, np.concatenate([-skew(qv), np.broadcast_to(np.eye(3), (len(qv), 3, 3))], axis=2))
        w = huber_w(r, delta)
        H = J.T @ (J * w[:, None])
        b = J.T @ (w * r)
        accepted = False
        while lam < 1e8:
            dx = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), -b)
            Rn = Rotation.from_rotvec(dx[:3]).as_matrix() @ R
            tn = Rotation.from_rotvec(dx[:3]).as_matrix() @ t + dx[3:]
            qn, Dn, Gn, vn = evaluate(Rn, tn); lookups += len(points)
            cn = huber_cost(Dn[vn], delta) * (valid.sum() / max(vn.sum(), 1))  # comparable if a few points leave the grid
            if cn <= cost:
                accepted = True
                break
            lam *= 10
        if not accepted:
            break
        rel = (cost - cn) / max(cost, 1e-12)
        R, t, q, D, G, valid, cost = Rn, tn, qn, Dn, Gn, vn, cn
        lam = max(lam / 10, 1e-9)
        if np.linalg.norm(dx) < tol_step or rel < tol_cost:
            break
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    # Gauss-Newton Hessian at the final estimate (for the D4 Hessian-based information).
    qv, r, g = q[valid], D[valid], G[valid]
    J = np.einsum("ni,nij->nj", g, np.concatenate([-skew(qv), np.broadcast_to(np.eye(3), (len(qv), 3, 3))], axis=2))
    H = J.T @ (J * huber_w(r, delta)[:, None])
    # Final quantities for validity / information (untruncated grid).
    qf = points @ R.T + t
    Du, Gu, vu = grid.query(qf, None); lookups += len(points)
    w_final = huber_w(D[valid], delta)
    return dict(T_c=T, iterations=it, lookups=lookups, runtime_s=time.perf_counter() - t0, H_wv=H,
                n_points=len(points), n_valid=int(valid.sum()), n_outside=int((~vu).sum()),
                n_truncated=int(np.sum(G[valid].any(axis=1) == False)) if mu is not None else 0,
                cost=float(cost), D_untrunc=Du, G_untrunc=Gu, valid_untrunc=vu,
                sigma2=float(np.sum(w_final * D[valid] ** 2) / max(valid.sum() - 6, 1)))


def validate(res, cfg=CFG):
    D, v = res["D_untrunc"], res["valid_untrunc"]
    inl = v & (D < cfg["max_correspondence_distance"])
    fitness = inl.sum() / max(len(D), 1)
    rmse = float(np.sqrt(np.mean(D[inl] ** 2))) if inl.any() else np.inf
    T = res["T_c"]
    tn = float(np.linalg.norm(T[:3, 3]))
    ang = float(np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1))))
    reasons = []
    if fitness < cfg["fitness_threshold"]: reasons.append("fitness")
    if rmse > cfg["inliers_rmse_thr"]: reasons.append("rmse")
    if not reasons:
        if tn > cfg["max_translation_distance"]: reasons.append("translation")
        if ang > cfg["max_rotation_distance"]: reasons.append("rotation")
    return dict(valid=not reasons, reason="+".join(reasons), fitness=float(fitness), inlier_rmse=rmse,
                corr_t_m=tn, corr_rot_deg=ang, inliers=inl)


def information_heuristic(points, res, inl, cfg=CFG):
    """M2's formula with n_i = grad D / |grad D| at the corrected inlier point; a_i = uncorrected point."""
    g = res["G_untrunc"][inl]
    a = points[inl]
    gn = np.linalg.norm(g, axis=1)
    ok = gn > 1e-9
    n = g[ok] / gn[ok, None]
    a = a[ok]
    Apn = np.zeros((6, 6))
    for a_i, n_i in zip(a, n):
        c = np.cross(a_i, n_i)
        cn = np.linalg.norm(c)
        if cn < 1e-12:
            continue
        H_i = np.vstack([-(c / cn).reshape(-1, 1), -n_i.reshape(-1, 1)]).T
        Apn += H_i.T @ H_i
    A = np.zeros((6, 6))
    A[:3, :3] = Apn[3:, 3:]; A[3:, 3:] = Apn[:3, :3]; A[:3, 3:] = Apn[3:, :3]; A[3:, :3] = Apn[:3, 3:]
    A = np.linalg.matrix_power(A, cfg["information_matrix_power"])
    rmse = np.sqrt(np.mean(res["D_untrunc"][inl] ** 2))
    A = (A / np.max(A)) * np.exp(-0.9 * rmse)
    sp, sr = cfg["scale_factor_position"], cfg["scale_factor_rotation"]
    A[:3, :3] *= sp * sp; A[3:, 3:] *= sr * sr; A[:3, 3:] *= sr * sp; A[3:, :3] *= sr * sp
    return A


def information_hessian(res):
    """J^T W J / sigma^2, reordered from xi = (w, v) to [pos, rot]."""
    H = res["H_wv"] / max(res["sigma2"], 1e-12)
    P = np.zeros((6, 6)); P[:3, 3:] = np.eye(3); P[3:, :3] = np.eye(3)
    return P @ H @ P.T
