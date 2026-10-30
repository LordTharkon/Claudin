"""Automatic UV unwrapping and packing.

* Charts: connected triangles of one Geo whose normals stay within a cone of the chart's
  seed normal, projected onto the plane of that normal (low distortion, no flips). Every
  chart is checked for self-overlap and split further if needed.
* Packing: one texture per material. All charts in the model share a single texel density
  (the highest at which every material still fits), islands sit on whole pixels with at
  least PAD pixels between neighbours and from the texture border.
"""
import math

import numpy as np

from meshkit import Geo

PAD = 4          # px between islands (and from the border) at the texture size
CONE = 35.0      # degrees


def _tri_overlap(a, b):
    """True if 2D triangles a, b (3x2 arrays) overlap with positive area (SAT)."""
    for tri in (a, b):
        for i in range(3):
            e = tri[(i + 1) % 3] - tri[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() <= pb.min() + 1e-12 or pb.max() <= pa.min() + 1e-12:
                return False
    return True


def _self_overlaps(uv, tris):
    if len(tris) < 2:
        return False
    P = uv[tris]                       # (n,3,2)
    lo, hi = P.min(1), P.max(1)
    ext = (hi - lo).max(0)
    cell = max(ext.max() / max(math.sqrt(len(tris)), 1.0), 1e-6)
    grid = {}
    for t in range(len(tris)):
        i0, j0 = (lo[t] // cell).astype(int)
        i1, j1 = (hi[t] // cell).astype(int)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                grid.setdefault((i, j), []).append(t)
    seen = set()
    for bucket in grid.values():
        for x in range(len(bucket)):
            for y in range(x + 1, len(bucket)):
                s, t = bucket[x], bucket[y]
                if (s, t) in seen:
                    continue
                seen.add((s, t))
                if len(set(tris[s]) & set(tris[t])) >= 2:
                    continue  # neighbours across an edge
                if (lo[s] < hi[t]).all() and (lo[t] < hi[s]).all() and _tri_overlap(P[s], P[t]):
                    return True
    return False


def _basis(n):
    a = np.eye(3)[np.argmin(np.abs(n))]
    u = np.cross(n, a)
    u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def _min_rect(pts):
    """Rotate 2D points so their bounding box is (close to) minimal; returns rotated pts."""
    if len(pts) < 3:
        return pts
    c = pts - pts.mean(0)
    best, best_a = None, 0.0
    try:
        cov = np.cov(c.T)
        w, v = np.linalg.eigh(cov)
        cands = [math.atan2(v[1, 1], v[0, 1])]
    except Exception:
        cands = []
    cands += [0.0, math.pi / 4]
    for a in cands:
        R = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        q = c @ R.T
        area = np.prod(q.max(0) - q.min(0))
        if best is None or area < best:
            best, best_a = area, a
    R = np.array([[math.cos(best_a), math.sin(best_a)], [-math.sin(best_a), math.cos(best_a)]])
    return c @ R.T


def _charts_for_geo(g, cone):
    P = np.asarray(g.pos, dtype=float)
    T = np.asarray(g.idx, dtype=np.int64).reshape(-1, 3)
    if len(T) == 0:
        return []
    fn = np.cross(P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 0]])
    area = np.linalg.norm(fn, axis=1)
    nrm = fn / np.maximum(area, 1e-20)[:, None]
    # edge adjacency through welded positions
    q = np.round(P * 1e5).astype(np.int64)
    keys = {}
    vid = np.array([keys.setdefault(tuple(k), len(keys)) for k in q])
    edges = {}
    for t, (a, b, c) in enumerate(vid[T]):
        for u, v in ((a, b), (b, c), (c, a)):
            edges.setdefault((min(u, v), max(u, v)), []).append(t)
    nbr = [[] for _ in range(len(T))]
    for ts in edges.values():
        for x in ts:
            for y in ts:
                if x != y:
                    nbr[x].append(y)
    cosc = math.cos(math.radians(cone))
    owner = -np.ones(len(T), dtype=np.int64)
    out = []
    for seed in np.argsort(-area):
        if owner[seed] >= 0:
            continue
        cid = len(out)
        n0 = nrm[seed] if area[seed] > 1e-14 else np.array([0.0, 1.0, 0.0])
        stack, members = [seed], []
        owner[seed] = cid
        while stack:
            t = stack.pop()
            members.append(t)
            for s in nbr[t]:
                if owner[s] < 0 and (area[s] <= 1e-14 or nrm[s] @ n0 >= cosc):
                    owner[s] = cid
                    stack.append(s)
        out.append((n0, members))
    return P, T, out


def _project(P, T, members, n0):
    u, v = _basis(n0)
    tris = T[members]
    verts = np.unique(tris)
    local = {int(x): i for i, x in enumerate(verts)}
    uv = np.stack([P[verts] @ u, P[verts] @ v], 1)
    lt = np.vectorize(local.get)(tris) if len(tris) else tris
    return verts, lt.reshape(-1, 3), uv


def unwrap(parts, size=1024, cone=CONE):
    """parts: {node: [Geo]} -> ({node: [Geo with .uv]}, texel density px/m)."""
    charts = []   # dict(node, gi, mat, verts, tris(local), uv(local m), w, h)
    for node, geos in parts.items():
        for gi, g in enumerate(geos):
            res = _charts_for_geo(g, cone)
            if not res:
                continue
            P, T, groups = res
            for n0, members in groups:
                todo = [(n0, members, cone)]
                while todo:
                    nn, mem, cn = todo.pop()
                    verts, lt, uv = _project(P, T, mem, nn)
                    if len(mem) > 1 and _self_overlaps(uv, lt):
                        if cn > 10:
                            # re-chart this group with a tighter cone
                            sub = Geo(g.mat, g.pos, g.nrm, [tuple(t) for t in T[mem]], fix=False)
                            _, T2, groups2 = _charts_for_geo(sub, cn / 2)
                            todo += [(m0, [mem[k] for k in mm], cn / 2) for m0, mm in groups2]
                        else:
                            todo += [(nn, [t], cn) for t in mem]
                        continue
                    uv = _min_rect(uv) if len(verts) >= 3 else uv - uv.mean(0)
                    uv = uv - uv.min(0)
                    w, h = uv.max(0)
                    if h > w:  # keep islands landscape for shelf packing
                        uv = uv[:, ::-1].copy()
                        uv[:, 0] = uv[:, 0].max() - uv[:, 0]
                        w, h = h, w
                    charts.append(dict(node=node, gi=gi, mat=g.mat, verts=verts, tris=lt, uv=uv, w=w, h=h))

    by_mat = {}
    for c in charts:
        by_mat.setdefault(c['mat'], []).append(c)
    usable = size - PAD  # islands live in [PAD, size - PAD]; rects carry PAD/2 of margin each side

    def pack(cs, d, place=False):
        order = sorted(range(len(cs)), key=lambda i: -cs[i]['h'])
        x = y = PAD // 2
        shelf = 0
        for i in order:
            w = math.ceil(cs[i]['w'] * d) + 1 + PAD
            h = math.ceil(cs[i]['h'] * d) + 1 + PAD
            if w > usable - PAD // 2:
                return False
            if x + w > size - PAD // 2:
                x, y = PAD // 2, y + shelf
                shelf = 0
            if y + h > size - PAD // 2:
                return False
            if place:
                cs[i]['px'] = (x + PAD // 2, y + PAD // 2)
            x += w
            shelf = max(shelf, h)
        return True

    dens = []
    for mat, cs in by_mat.items():
        total = sum(c['w'] * c['h'] for c in cs) or 1e-6
        lo, hi = 0.0, size / math.sqrt(total)
        for _ in range(30):
            mid = (lo + hi) / 2
            if pack(cs, mid):
                lo = mid
            else:
                hi = mid
        dens.append(lo)
    d = min(dens)
    for cs in by_mat.values():
        assert pack(cs, d, place=True)

    # rebuild geometry with per-chart vertices and UVs
    out = {node: [] for node in parts}
    per_geo = {}
    for c in charts:
        per_geo.setdefault((c['node'], c['gi']), []).append(c)
    for node, geos in parts.items():
        for gi, g in enumerate(geos):
            pos, nrm, uvs, idx = [], [], [], []
            for c in per_geo.get((node, gi), []):
                o = len(pos)
                px, py = c['px']
                for k, v in enumerate(c['verts']):
                    pos.append(g.pos[v])
                    nrm.append(g.nrm[v])
                    uvs.append(((px + 0.5 + c['uv'][k, 0] * d) / size, (py + 0.5 + c['uv'][k, 1] * d) / size))
                idx += [tuple(int(i) + o for i in t) for t in c['tris']]
            if idx:
                ng = Geo(g.mat, pos, nrm, idx, fix=False)
                ng.uv = uvs
                out[node].append(ng)
    return out, d
