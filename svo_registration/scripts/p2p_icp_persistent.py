#!/usr/bin/env python3
# FYP Phase 10 (throughput control, "M2-fast"): point-to-plane ICP with a KD-tree of the twin that is
# built once and reused, instead of Open3D's registration_icp, which rebuilds the KD-tree of the
# target on every call (Phase 8: 154 of 160 ms per keyframe at room scale).
# The algorithm replicates Open3D 0.19 pipelines.registration.registration_icp with
# TransformationEstimationPointToPlane (L2 loss) and ICPConvergenceCriteria(relative_fitness=1e-6,
# relative_rmse=1e-6, max_iteration), step by step:
#   result = correspondences(source @ init)
#   for i in range(max_iteration):
#       update = point-to-plane step from result's correspondences (identity if none):
#                r_i = (p_i - q_i) . n_i, J_i = [p_i x n_i, n_i], x = solve(J^T J, -J^T r),
#                update = [Rz(x2) Ry(x1) Rx(x0) | x3..5]
#       transformation = update @ transformation; source <- update applied to the current points
#       backup = result; result = correspondences(source)
#       stop if |dfitness| < 1e-6 and |drmse| < 1e-6
#   correspondences: nearest twin point within max_correspondence_distance (exact NN, scipy cKDTree);
#   fitness = n_corr / n_source, inlier_rmse = sqrt(sum d^2 / n_corr) (0, 0 if none).
# Differences from Open3D are floating-point summation order only (verified against the logged online
# M2 outputs, analysis/phase10_m2fast_equivalence.py).
import numpy as np
from scipy.spatial import cKDTree


class RegistrationResult:
    """The fields of Open3D's RegistrationResult that the registration node uses."""

    def __init__(self, transformation, fitness, inlier_rmse, correspondence_set):
        self.transformation = transformation
        self.fitness = fitness
        self.inlier_rmse = inlier_rmse
        self.correspondence_set = correspondence_set


def _rot_zyx(x):
    """Open3D TransformVector6dToMatrix4d rotation: AngleAxis(x2, Z) * AngleAxis(x1, Y) * AngleAxis(x0, X)."""
    cx, sx = np.cos(x[0]), np.sin(x[0])
    cy, sy = np.cos(x[1]), np.sin(x[1])
    cz, sz = np.cos(x[2]), np.sin(x[2])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class PersistentPointToPlaneICP:
    def __init__(self, points, normals):
        self.points = np.ascontiguousarray(points, dtype=np.float64)
        self.normals = np.ascontiguousarray(normals, dtype=np.float64)
        self.tree = cKDTree(self.points)

    def _correspondences(self, tree, tgt, src, max_corr):
        d, j = tree.query(src, k=1, distance_upper_bound=max_corr)
        i = np.flatnonzero(np.isfinite(d))
        j = j[i]
        if len(i) == 0:
            return np.zeros((0, 2), dtype=np.int64), 0.0, 0.0
        diff = src[i] - tgt[j]
        err2 = np.sum(diff * diff, axis=1)
        corr = np.c_[i, j].astype(np.int64)
        return corr, len(i) / len(src), float(np.sqrt(np.sum(err2) / len(i)))

    def register(self, source, max_corr, max_iter=25, mask=None, rel_fitness=1e-6, rel_rmse=1e-6):
        """source: (N, 3) map-frame points; init = identity (as in M2). mask: optional boolean crop of the
        twin; None or all-True uses the persistent tree, otherwise a tree of the cropped twin is built for
        this call (correspondence indices then refer to the cropped arrays, as in M2).
        Returns (RegistrationResult, target_points, target_normals, tree_rebuilt)."""
        if mask is None or mask.all():
            tree, tgt, nrm, rebuilt = self.tree, self.points, self.normals, False
        else:
            tgt, nrm = self.points[mask], self.normals[mask]
            tree, rebuilt = cKDTree(tgt), True
        src = np.array(source, dtype=np.float64)
        T = np.eye(4)
        if len(tgt) == 0:
            return RegistrationResult(T, 0.0, 0.0, np.zeros((0, 2), dtype=np.int64)), tgt, nrm, rebuilt
        corr, fit, rmse = self._correspondences(tree, tgt, src, max_corr)
        for _ in range(max_iter):
            U = np.eye(4)
            if len(corr):
                p, q, n = src[corr[:, 0]], tgt[corr[:, 1]], nrm[corr[:, 1]]
                r = np.sum((p - q) * n, axis=1)
                J = np.c_[np.cross(p, n), n]
                x = np.linalg.solve(J.T @ J, -(J.T @ r))
                U[:3, :3] = _rot_zyx(x[:3])
                U[:3, 3] = x[3:]
            T = U @ T
            src = src @ U[:3, :3].T + U[:3, 3]
            fit0, rmse0 = fit, rmse
            corr, fit, rmse = self._correspondences(tree, tgt, src, max_corr)
            if abs(fit0 - fit) < rel_fitness and abs(rmse0 - rmse) < rel_rmse:
                break
        return RegistrationResult(T, fit, rmse, corr), tgt, nrm, rebuilt


def _skew_rows(p):
    z = np.zeros(len(p))
    return np.stack([np.stack([z, -p[:, 2], p[:, 1]], 1),
                     np.stack([p[:, 2], z, -p[:, 0]], 1),
                     np.stack([-p[:, 1], p[:, 0], z], 1)], 1)


def hessian_information(points, result, tgt_points, tgt_normals):
    """Phase 10 D4: point-to-plane Hessian information J^T J / sigma^2 at the solution, [pos, rot] order
    (same formula and operation order as analysis/p2p_offline.py:information_hessian). points: the uncorrected
    source cloud; J_i = n_i^T [-[q_i]x | I] with q_i the corrected point (left perturbation (w, v) about the
    map origin, as the heuristic); sigma^2 = sum r_i^2 / (n - 6). NaN if fewer than 7 correspondences."""
    c = np.asarray(result.correspondence_set)
    if len(c) < 7:
        return np.full((6, 6), np.nan)
    T = result.transformation
    q = points[c[:, 0]] @ T[:3, :3].T + T[:3, 3]
    n = tgt_normals[c[:, 1]]
    r = np.einsum("ij,ij->i", n, q - tgt_points[c[:, 1]])
    J = np.concatenate([np.einsum("ni,nij->nj", n, -_skew_rows(q)), n], axis=1)  # (w, v)
    H = J.T @ J
    s2 = float(np.sum(r ** 2) / max(len(r) - 6, 1))
    P = np.zeros((6, 6)); P[:3, 3:] = np.eye(3); P[3:, :3] = np.eye(3)
    return P @ (H / max(s2, 1e-12)) @ P.T


def body_information(A_pr, p_body):
    """Phase 10 D4 fix (docs/PLAN.md deviation 2026-10-25): Hessian information as computed ([pos = v, rot = w] of the
    left perturbation (w, v) of the correction about the map origin) re-expressed about the body pose, the backend's
    convention ([body position, body rotation]): A_body = Jinv^T H_wv Jinv with Jinv: [dp; dtheta] -> (w = dtheta,
    v = dp + [p_body]x dtheta), i.e. the inverse of dp = v - [p_body]x w, dtheta = w."""
    P = np.zeros((6, 6)); P[:3, 3:] = np.eye(3); P[3:, :3] = np.eye(3)
    H_wv = P.T @ A_pr @ P
    p = np.asarray(p_body, dtype=np.float64)
    Jinv = np.zeros((6, 6))
    Jinv[:3, 3:] = np.eye(3)
    Jinv[3:, :3] = np.eye(3)
    Jinv[3:, 3:] = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    return Jinv.T @ H_wv @ Jinv
