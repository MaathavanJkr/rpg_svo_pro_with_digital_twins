#!/usr/bin/env python3
"""Phase 7: voxel distance grid of the digital twin (offline, built once per voxel size).

Built from the SAME twin file as M2 (dataset/derived/euroc_twin/V1_leica_v30mm.ply; md5 checked).
Each voxel centre stores the exact Euclidean distance to the nearest twin point (KD-tree query),
i.e. an unsigned distance field (UDF); no sign, because the laser scan has no inside/outside.
Truncation (TSDF-style, D = min(d, mu)) is applied at query time, so one grid serves all mu.
The grid covers the twin's bounding box plus a margin; queries outside it are reported as invalid.

Usage (venv): python analysis/df_grid.py --voxel 0.05 [--margin 1.0] [--twin <preprocessed twin .ply>]
Output: dataset/derived/euroc_twin/df/<twin>_udf_v<mm>mm.npz (+ .json metadata) and slice figures, and
for the C++ module (Phase 8) <stem>.bin (raw float32, C order: x slowest, z fastest) + <stem>.bin.json
(format, dims, origin, voxel, dtype, twin, twin md5, FNV-1a 64-bit checksum of the payload).
"""
import argparse
import hashlib
import json
import os
import time
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TWIN = os.path.join(REPO, "dataset/derived/euroc_twin/V1_leica_v30mm.ply")
TWIN_MD5 = "8fc20009837a56090259b9c5da31724c"


class DistanceGrid:
    """Unsigned distance grid with trilinear interpolation and its analytic gradient."""

    def __init__(self, path):
        z = np.load(path)
        self.d = z["distance"].astype(np.float64)  # (nx, ny, nz), value at voxel centres
        self.origin = z["origin"]                  # centre of voxel (0, 0, 0)
        self.v = float(z["voxel"])
        self.shape = np.array(self.d.shape)

    def query(self, p, mu=None):
        """D(p) and dD/dp for points p (N, 3). Returns (D, grad, valid). Trilinear interpolation of
        the voxel-centre values; the gradient is the exact derivative of that interpolant.
        mu: truncation distance (None = untruncated). Beyond mu, D = mu and the gradient is 0."""
        g = (p - self.origin) / self.v
        i0 = np.floor(g).astype(np.int64)
        valid = np.all((i0 >= 0) & (i0 < self.shape - 1), axis=1)
        D = np.full(len(p), np.nan)
        G = np.zeros((len(p), 3))
        if not valid.any():
            return D, G, valid
        i0v = i0[valid]
        f = g[valid] - i0v
        x, y, zc = i0v[:, 0], i0v[:, 1], i0v[:, 2]
        c = np.empty((len(i0v), 2, 2, 2))
        for a in (0, 1):
            for b in (0, 1):
                for cc in (0, 1):
                    c[:, a, b, cc] = self.d[x + a, y + b, zc + cc]
        fx, fy, fz = f[:, 0], f[:, 1], f[:, 2]
        c00 = c[:, 0, 0, 0] * (1 - fx) + c[:, 1, 0, 0] * fx
        c01 = c[:, 0, 0, 1] * (1 - fx) + c[:, 1, 0, 1] * fx
        c10 = c[:, 0, 1, 0] * (1 - fx) + c[:, 1, 1, 0] * fx
        c11 = c[:, 0, 1, 1] * (1 - fx) + c[:, 1, 1, 1] * fx
        c0 = c00 * (1 - fy) + c10 * fy
        c1 = c01 * (1 - fy) + c11 * fy
        Dv = c0 * (1 - fz) + c1 * fz
        dx = ((c[:, 1, 0, 0] - c[:, 0, 0, 0]) * (1 - fy) + (c[:, 1, 1, 0] - c[:, 0, 1, 0]) * fy) * (1 - fz) \
            + ((c[:, 1, 0, 1] - c[:, 0, 0, 1]) * (1 - fy) + (c[:, 1, 1, 1] - c[:, 0, 1, 1]) * fy) * fz
        dy = ((c10 - c00) * (1 - fz) + (c11 - c01) * fz)
        dz = c1 - c0
        Gv = np.c_[dx, dy, dz] / self.v
        if mu is not None:
            trunc = Dv >= mu
            Dv = np.minimum(Dv, mu)
            Gv[trunc] = 0.0
        D[valid] = Dv
        G[valid] = Gv
        return D, G, valid


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def fnv1a64(data):
    """64-bit FNV-1a over bytes (same algorithm in the C++ loader)."""
    h = 0xcbf29ce484222325
    a = np.frombuffer(data, dtype=np.uint8)
    for chunk in np.array_split(a, max(1, len(a) // (1 << 20))):
        for b in chunk.tobytes():
            h ^= b
            h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def export_binary(npz_path, twin_path, twin_md5):
    z = np.load(npz_path)
    d = np.ascontiguousarray(z["distance"], dtype=np.float32)
    stem = npz_path[:-4]
    payload = d.tobytes(order="C")
    with open(stem + ".bin", "wb") as f:
        f.write(payload)
    hdr = dict(format="fyp_udf_v1", dims=list(d.shape), origin=[float(x) for x in z["origin"]], voxel=float(z["voxel"]),
               dtype="float32", order="C (x slowest, z fastest); value at voxel centre origin + voxel*(i,j,k)",
               bytes=len(payload), checksum_fnv1a64=format(fnv1a64(payload), "016x"),
               twin=os.path.relpath(twin_path, REPO), twin_md5=twin_md5)
    json.dump(hdr, open(stem + ".bin.json", "w"), indent=2)
    return stem + ".bin", hdr


def build(voxel, margin, out_dir, twin=TWIN):
    twin_md5 = md5(twin)
    expected = json.load(open(twin[:-4] + ".json"))["output_md5"]
    assert twin_md5 == expected, "twin file differs from its preprocessing record"
    if twin == TWIN:
        assert twin_md5 == TWIN_MD5, "twin file differs from the one used by M2"
    P = np.asarray(o3d.io.read_point_cloud(twin).points)
    lo = P.min(0) - margin
    hi = P.max(0) + margin
    n = np.ceil((hi - lo) / voxel).astype(int) + 1
    origin = lo
    t0 = time.time()
    tree = cKDTree(P)
    xs = [origin[k] + voxel * np.arange(n[k]) for k in range(3)]
    d = np.empty(tuple(n), dtype=np.float32)
    for i, x in enumerate(xs[0]):  # slab by slab to bound memory
        Y, Z = np.meshgrid(xs[1], xs[2], indexing="ij")
        q = np.c_[np.full(Y.size, x), Y.ravel(), Z.ravel()]
        d[i] = tree.query(q, workers=-1)[0].reshape(Y.shape).astype(np.float32)
    build_s = time.time() - t0
    stem = f"{os.path.basename(twin)[:-4]}_udf_v{int(round(voxel * 1000))}mm"
    path = os.path.join(out_dir, stem + ".npz")
    np.savez_compressed(path, distance=d, origin=origin, voxel=voxel)
    meta = dict(twin=os.path.relpath(twin, REPO), twin_md5=twin_md5, twin_points=int(len(P)), voxel_m=voxel,
                margin_m=margin, shape=n.tolist(), n_voxels=int(np.prod(n)), origin=origin.tolist(),
                memory_MB_float32=float(d.nbytes / 1e6), file_MB=float(os.path.getsize(path) / 1e6),
                build_s=build_s, method="exact Euclidean distance from voxel centres to nearest twin point (cKDTree)",
                d_max_m=float(d.max()))
    json.dump(meta, open(os.path.join(out_dir, stem + ".json"), "w"), indent=2)
    export_binary(path, twin, twin_md5)
    return path, meta


def slices_figure(paths, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(paths), 3, figsize=(17, 5.2 * len(paths)))
    axs = np.atleast_2d(axs)
    for row, p in enumerate(paths):
        g = DistanceGrid(p)
        for col, z in enumerate((0.5, 1.5)):
            k = int(round((z - g.origin[2]) / g.v))
            ax = axs[row, col]
            im = ax.imshow(g.d[:, :, k].T, origin="lower", cmap="viridis", vmax=1.0,
                           extent=[g.origin[0], g.origin[0] + g.v * g.shape[0], g.origin[1], g.origin[1] + g.v * g.shape[1]])
            ax.set_title(f"v = {g.v:.2f} m, horizontal slice z = {z} m"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
            fig.colorbar(im, ax=ax, label="unsigned distance [m] (clipped at 1 m for display)")
        j = int(round((0.5 - g.origin[1]) / g.v))
        ax = axs[row, 2]
        im = ax.imshow(g.d[:, j, :].T, origin="lower", cmap="viridis", vmax=1.0,
                       extent=[g.origin[0], g.origin[0] + g.v * g.shape[0], g.origin[2], g.origin[2] + g.v * g.shape[2]])
        ax.set_title(f"v = {g.v:.2f} m, vertical slice y = 0.5 m"); ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
        fig.colorbar(im, ax=ax, label="unsigned distance [m]")
    fig.savefig(out_png, dpi=110, bbox_inches="tight")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--voxel", type=float, nargs="+", default=[0.05, 0.10, 0.20])
    ap.add_argument("--margin", type=float, default=1.0)
    ap.add_argument("--twin", default=TWIN)
    ap.add_argument("--export-only", action="store_true", help="only write .bin/.bin.json for existing .npz grids")
    args = ap.parse_args()
    out_dir = os.path.join(REPO, "dataset/derived/euroc_twin/df")
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    stem0 = os.path.basename(args.twin)[:-4]
    for v in args.voxel:
        if args.export_only:
            p = os.path.join(out_dir, f"{stem0}_udf_v{int(round(v * 1000))}mm.npz")
            print(json.dumps(export_binary(p, args.twin, md5(args.twin))[1]))
            continue
        p, meta = build(v, args.margin, out_dir, args.twin)
        paths.append(p)
        print(json.dumps(meta))
    if paths:
        slices_figure(paths, os.path.join(REPO, f"docs/img/phase7_df_slices{'' if args.twin == TWIN else '_' + stem0}.png"))
