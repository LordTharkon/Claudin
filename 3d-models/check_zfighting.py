"""Find z-fighting in the generated models: pairs of faces from different pieces of
geometry that lie in the same plane, face the same way and overlap.

    python3 3d-models/check_zfighting.py            # all models
    python3 3d-models/check_zfighting.py pickup     # models whose name contains "pickup"
    python3 3d-models/check_zfighting.py --all      # also list same-material overlaps

Overlaps between two surfaces of the same material are not counted as failures: both
surfaces render identically, so whichever wins the depth test there is no flicker.
"""
import importlib
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from meshkit import cross, dot, norm, sub  # noqa: E402

BUILDERS = [
    ('build_crawler_dozer', None), ('build_wheel_dozer', None), ('build_backhoe_loader', None),
    ('build_mini_excavator', None), ('build_compact_track_loader', None),
    ('build_wheel_tractor_scraper', None), ('build_articulated_dump_truck', None),
    ('build_pickup_trucks', 'LIGHT'), ('build_pickup_trucks', 'DUALLY'),
]
PLANE_TOL = 0.0008   # faces closer than this (m) are treated as coplanar
MIN_AREA = 1e-5      # ignore overlaps smaller than 10 mm^2


def _clip(subject, a, b):
    """Clip polygon by the half-plane left of edge a->b (2D)."""
    out = []
    def inside(p):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= -1e-12
    def inter(p, q):
        x1, y1, x2, y2 = a[0], a[1], b[0], b[1]
        x3, y3, x4, y4 = p[0], p[1], q[0], q[1]
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(den) < 1e-18:
            return q
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    for i in range(len(subject)):
        p, q = subject[i - 1], subject[i]
        if inside(q):
            if not inside(p):
                out.append(inter(p, q))
            out.append(q)
        elif inside(p):
            out.append(inter(p, q))
    return out


def _area(poly):
    return 0.5 * abs(sum(poly[i - 1][0] * poly[i][1] - poly[i][0] * poly[i - 1][1] for i in range(len(poly))))


def _ccw(t):
    a, b, c = t
    if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) < 0:
        return [a, c, b]
    return list(t)


def overlap_area(t1, t2):
    poly = _ccw(t1)
    t2 = _ccw(t2)
    for i in range(3):
        poly = _clip(poly, t2[i], t2[(i + 1) % 3])
        if len(poly) < 3:
            return 0.0
    return _area(poly)


def check(model):
    model.finalize()   # check the exported rest pose, cylinders included
    tris = []  # (geo_id, part, mat, p0, p1, p2, n, d)
    gid = 0
    for part, geos in model.parts.items():
        for g in geos:
            for a, b, c in g.idx:
                p0, p1, p2 = g.pos[a], g.pos[b], g.pos[c]
                fn = cross(sub(p1, p0), sub(p2, p0))
                if math.sqrt(dot(fn, fn)) < 1e-9:
                    continue
                n = norm(fn)
                tris.append((gid, part, g.mat, p0, p1, p2, n, dot(n, p0)))
            gid += 1
    buckets = defaultdict(list)
    for i, t in enumerate(tris):
        n = t[6]
        key = (round(n[0], 2), round(n[1], 2), round(n[2], 2), round(t[7] / PLANE_TOL))
        buckets[key].append(i)
    found = defaultdict(float)
    seen = set()
    for key, ids in buckets.items():
        # also compare with the neighbouring distance bucket
        cand = list(ids) + buckets.get(key[:3] + (key[3] + 1,), [])
        if len(cand) < 2:
            continue
        boxes = {}
        for i in cand:
            p = tris[i][3:6]
            boxes[i] = [min(q[k] for q in p) for k in range(3)] + [max(q[k] for q in p) for k in range(3)]
        cand.sort(key=lambda i: boxes[i][0])
        for ai in range(len(cand)):
            i = cand[ai]
            ti, bi = tris[i], boxes[i]
            for aj in range(ai + 1, len(cand)):
                j = cand[aj]
                bj = boxes[j]
                if bj[0] > bi[3] + 1e-6:
                    break
                tj = tris[j]
                if ti[0] == tj[0] or (i, j) in seen:
                    continue
                if any(bj[k] > bi[k + 3] + 1e-6 or bi[k] > bj[k + 3] + 1e-6 for k in (1, 2)):
                    continue
                if dot(ti[6], tj[6]) < 0.9995 or abs(ti[7] - tj[7]) > PLANE_TOL:
                    continue
                seen.add((i, j))
                n = ti[6]
                ax = max(range(3), key=lambda k: abs(n[k]))
                u, v = [k for k in range(3) if k != ax]
                a2 = overlap_area([(p[u], p[v]) for p in ti[3:6]], [(p[u], p[v]) for p in tj[3:6]])
                if a2 > MIN_AREA:
                    c = ti[3]
                    k = tuple(sorted([(ti[1], ti[2]), (tj[1], tj[2])])) + (tuple(round(x, 2) for x in n),
                                                                          tuple(round(x, 2) for x in c))
                    found[k] += a2
    return found


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    show_all = '--all' in sys.argv
    filt = args[0] if args else ''
    total = 0
    for mod, cfg in BUILDERS:
        m = importlib.import_module(mod)
        model = m.build(getattr(m, cfg)) if cfg else m.build()
        if filt.lower() not in model.name.lower():
            continue
        found = check(model)
        # collapse to one line per (pair, normal), keeping one sample location
        summary = defaultdict(lambda: [0.0, None])
        same = 0
        for (a, b, n, c), area in found.items():
            if a[1] == b[1] and not show_all:
                same += 1
                continue
            s = summary[(a, b, n)]
            s[0] += area
            s[1] = s[1] or c
        print('%-24s %d z-fighting surface pair(s)%s' % (model.name, len(summary),
              '' if show_all else '  (%d harmless same-material overlaps hidden)' % same))
        for (a, b, n), (area, c) in sorted(summary.items(), key=lambda kv: -kv[1][0]):
            print('   %6.0f cm2  %s/%s  vs  %s/%s  normal %s  near %s' % (area * 1e4, a[0], a[1], b[0], b[1], n, c))
        total += len(summary)
    print('total:', total)
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main())
