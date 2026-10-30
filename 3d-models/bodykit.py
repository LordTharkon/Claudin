"""Smooth-surface helpers for vehicle bodywork: lofted cross-sections,
rounded boxes, flush panels (glass, trim) and polygon fillets."""
import math

from meshkit import Geo, add, cross, dot, norm, sub, _signed_area, _triangulate


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def smin(a, b, k):
    """Smooth minimum: rounds the corner where two curves meet."""
    h = max(k - abs(a - b), 0.0) / k
    return min(a, b) - h * h * k / 4


def _bez(a, c, b, n):
    """n points of a quadratic bezier a->c->b, excluding a."""
    out = []
    for i in range(1, n + 1):
        t = i / n
        out.append(((1 - t) ** 2 * a[0] + 2 * (1 - t) * t * c[0] + t * t * b[0],
                    (1 - t) ** 2 * a[1] + 2 * (1 - t) * t * c[1] + t * t * b[1]))
    return out


def section(yb, top_fn, side_fn, rb, rs, nb=3, nc=5, ns=10, nt=14):
    """Closed, left/right-symmetric cross-section as (z, y) points.

    yb: bottom height; top_fn(z): roof/hood height at z; side_fn(y): half-width at y;
    rb / rs: bottom and shoulder corner radii. Point count depends only on n*.
    """
    yt0 = top_fn(0.0)
    rs = max(1e-4, min(rs, 0.45 * (yt0 - yb)))
    rb = max(1e-4, min(rb, 0.45 * (yt0 - yb)))
    zs = side_fn(yt0 - rs)
    yte = top_fn(max(zs - rs, 0.0))
    ys = yte - rs
    zs = side_fn(ys)
    zb = side_fn(yb + rb)
    half = [(zb - rb) * i / nb for i in range(nb + 1)]
    pts = [(z, yb) for z in half]
    pts += _bez((zb - rb, yb), (zb, yb), (zb, yb + rb), nc)
    for i in range(1, ns + 1):
        y = yb + rb + (ys - yb - rb) * i / ns
        pts.append((side_fn(y), y))
    pts += _bez((zs, ys), (zs, yte), (zs - rs, yte), nc)
    for i in range(1, nt + 1):
        z = (zs - rs) * (1 - i / nt)
        pts.append((z, top_fn(z)))
    return pts + [(-z, y) for z, y in reversed(pts[1:-1])]


def ring_at(x, sec):
    return [(x, y, z) for z, y in sec]


def loft(mat, rings, cap0=False, cap1=False):
    """Skin a list of equal-length closed rings with smooth normals."""
    n, m = len(rings), len(rings[0])
    pos = [p for r in rings for p in r]
    acc = [(0.0, 0.0, 0.0)] * len(pos)
    quads = []
    for i in range(n - 1):
        for j in range(m):
            a, b = i * m + j, i * m + (j + 1) % m
            c, d = b + m, a + m
            fn = cross(sub(pos[c], pos[a]), sub(pos[d], pos[b]))
            for k in (a, b, c, d):
                acc[k] = add(acc[k], fn)
            quads.append((a, b, c, d))
    # make normals point away from each ring's centre
    s = 0.0
    for i, r in enumerate(rings):
        cen = tuple(sum(p[k] for p in r) / m for k in range(3))
        for j, p in enumerate(r):
            s += dot(acc[i * m + j], sub(p, cen))
    sign = 1 if s >= 0 else -1
    nrm = [norm((v[0] * sign, v[1] * sign, v[2] * sign)) for v in acc]
    idx = []
    for a, b, c, d in quads:
        idx += [(a, b, c), (a, c, d)]
    out = [Geo(mat, pos, nrm, idx)]
    for flag, r, nb in ((cap0, rings[0], rings[1] if n > 1 else None), (cap1, rings[-1], rings[-2] if n > 1 else None)):
        if flag:
            cen = tuple(sum(p[k] for p in r) / m for k in range(3))
            ncen = tuple(sum(p[k] for p in nb) / m for k in range(3))
            nn = norm(sub(cen, ncen))
            p2 = [cen] + list(r)
            out.append(Geo(mat, p2, [nn] * len(p2), [(0, 1 + j, 1 + (j + 1) % m) for j in range(m)]))
    return out


def rbox(mat, sx, sy, sz, r, center=(0, 0, 0), n=4):
    """Box with all edges and corners rounded by radius r (long axis any)."""
    r = min(r, sx / 2 - 1e-4, sy / 2 - 1e-4, sz / 2 - 1e-4)
    cx, cy, cz = center
    rings = []
    angs = [math.pi / 2 * i / n for i in range(n + 1)]
    xs = [(-(sx / 2 - r) - r * math.cos(a), r * (1 - math.sin(a))) for a in angs]
    xs += [((sx / 2 - r) + r * math.sin(a), r * (1 - math.cos(a))) for a in angs]
    for x, d in xs:
        rr = max(r - d, 1e-4)
        hz, hy = sz / 2 - d, sy / 2 - d
        sec = section(-hy, lambda z, hy=hy: hy, lambda y, hz=hz: hz, rr, rr, nb=1, nc=n, ns=1, nt=1)
        rings.append([(cx + x, cy + y, cz + z) for z, y in sec])
    return loft(mat, rings, True, True)


def warp(geos, fn):
    """Bend geometry by moving each vertex with fn(p) -> p (normals kept)."""
    return [Geo(g.mat, [fn(p) for p in g.pos], g.nrm, g.idx, fix=False) for g in geos]


def fillet(poly, r, n=5):
    out = []
    L = len(poly)
    for i in range(L):
        p, v, q = poly[i - 1], poly[i], poly[(i + 1) % L]
        dp, dq = math.dist(p, v), math.dist(q, v)
        if dp < 1e-9 or dq < 1e-9:
            continue
        tp, tq = min(r / dp, 0.45), min(r / dq, 0.45)
        a = (v[0] + (p[0] - v[0]) * tp, v[1] + (p[1] - v[1]) * tp)
        b = (v[0] + (q[0] - v[0]) * tq, v[1] + (q[1] - v[1]) * tq)
        out.append(a)
        out += _bez(a, v, b, n)
    return out


def panel(mat, poly, fn, normal):
    """Flat-shaded panel: a 2D polygon triangulated then mapped to 3D by fn(u, v)."""
    pts = list(poly)
    if _signed_area(pts) < 0:
        pts.reverse()
    tris = _triangulate(pts)
    pos = [fn(u, v) for u, v in pts]
    n = norm(normal)
    return Geo(mat, pos, [n] * len(pos), [tuple(t) for t in tris])
