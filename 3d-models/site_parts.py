"""Building blocks for construction-site props: corrugated sheet, rough chunks,
noisy mounds, screw jacks, small trailer wheels and mesh panels."""
import math
import random

from meshkit import Geo, add, box, box_span, cross, cyl_between, cylinder, dot, extrude, norm, sub
from bodykit import loft
from vehicle_parts import place_wheel, wheel


def corrugated(mat, length, height, pitch=0.13, depth=0.02, thick=0.002, crest=0.35):
    """Trapezoidal corrugated sheet: length along +X from 0, height along +Y from 0,
    ribs vertical, crests toward +Z (back face at z = 0, crests at z = depth)."""
    pts = []
    n = max(1, int(round(length / pitch)))
    p = length / n
    for i in range(n):
        x = i * p
        f, s = p * crest / 2, p * 0.15
        pts += [(x, 0.0), (x + p / 2 - f - s, 0.0), (x + p / 2 - f, depth), (x + p / 2 + f, depth), (x + p / 2 + f + s, 0.0)]
    pts.append((length, 0.0))
    outline = [(x, -z) for x, z in pts] + [(x, -(z + thick)) for x, z in reversed(pts)]
    return extrude(mat, outline, height).rotate('x', -90).move(y=height / 2)


def wall_sheets(mat, x0, x1, y0, y1, openings, **kw):
    """Corrugated wall from x0..x1, y0..y1 (crests toward +Z) with rectangular openings
    [(xa, xb, ya, yb)]; returns sheets in the z = 0 plane."""
    out = []
    edges = sorted({x0, x1} | {o[0] for o in openings} | {o[1] for o in openings})
    for a, b in zip(edges, edges[1:]):
        if b - a < 1e-6:
            continue
        cuts = sorted((o[2], o[3]) for o in openings if o[0] <= a + 1e-6 and o[1] >= b - 1e-6)
        y = y0
        for ya, yb in cuts + [(y1, y1)]:
            if ya - y > 1e-3:
                out.append(corrugated(mat, b - a, ya - y, **kw).move(x=a, y=y))
            y = max(y, yb)
    return out


def chunk(mat, center, size, rng, flat=0.6):
    """Rough broken lump (a jittered box), flat shaded."""
    sx, sy, sz = size
    c = []
    for i in (-1, 1):
        for j in (-1, 1):
            for k in (-1, 1):
                c.append((center[0] + i * sx / 2 * rng.uniform(0.55, 1.0),
                          center[1] + j * sy / 2 * rng.uniform(0.55, 1.0) * (flat if j > 0 else 1),
                          center[2] + k * sz / 2 * rng.uniform(0.55, 1.0)))
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    cen = tuple(sum(p[a] for p in c) / 8 for a in range(3))
    pos, nrm, idx = [], [], []
    for f in faces:
        for tri in ((f[0], f[1], f[2]), (f[0], f[2], f[3])):
            a, b, d = (c[t] for t in tri)
            n = norm(cross(sub(b, a), sub(d, a)))
            if dot(n, sub(a, cen)) < 0:
                a, d = d, a
                n = (-n[0], -n[1], -n[2])
            i = len(pos)
            pos += [a, b, d]
            nrm += [n, n, n]
            idx.append((i, i + 1, i + 2))
    return Geo(mat, pos, nrm, idx)


def mound(mat, rx, rz, h, rng, rough=0.12, rings=14, segs=40, power=1.6, center=(0, 0, 0)):
    """Irregular heap: concentric rings from the ground edge up to the crest."""
    phase = [rng.uniform(0, 2 * math.pi) for _ in range(4)]
    rs = []
    for i in range(rings + 1):
        t = i / rings                      # 0 at the rim, 1 at the crest
        ring = []
        for j in range(segs):
            a = 2 * math.pi * j / segs
            wob = 1 + rough * (math.sin(3 * a + phase[0]) * 0.6 + math.sin(5 * a + phase[1]) * 0.4)
            r = (1 - t) * wob
            y = h * (1 - (1 - t) ** power) if i else 0.0
            if 0 < i < rings:
                y *= 1 + rough * 0.8 * math.sin(4 * a + 7 * t + phase[2]) * math.sin(math.pi * t)
            ring.append((center[0] + rx * r * math.cos(a), center[1] + y, center[2] + rz * r * math.sin(a)))
        rs.append(ring)
    return loft(mat, rs)


def screw_jack(x, z, y0, y1, plate=0.22, tube=0.08, pad=True):
    """Blocking / levelling jack standing from y0 (ground) up to y1 (underside of frame)."""
    out = []
    if pad:
        out.append(box_span('timber', x - 0.20, x + 0.20, y0, y0 + 0.05, z - 0.15, z + 0.15))
        y0 += 0.05
    out.append(box_span('steel_dark', x - plate / 2, x + plate / 2, y0, y0 + 0.015, z - plate / 2, z + plate / 2))
    ym = y0 + (y1 - y0) * 0.6
    out.append(box_span('steel_dark', x - tube / 2, x + tube / 2, y0 + 0.015, ym, z - tube / 2, z + tube / 2))
    out.append(cyl_between('steel', (x, ym - 0.02, z), (x, y1 - 0.025, z), 0.018, 12))
    out.append(cylinder('steel_dark', 0.035, 0.02, 6).at((x, ym + 0.01, z)))   # adjusting nut
    out.append(box_span('steel_dark', x - 0.07, x + 0.07, y1 - 0.025, y1 - 0.002, z - 0.07, z + 0.07))
    return out


def trailer_wheel(r=0.30, w=0.165, bead=0.165):
    return wheel(r, w, bead, 'hwy', 'steel', 'steel', lugs=None)


def mesh_panel(mat, x0, x1, y0, y1, z, pitch=0.10, bar=0.006, frame=0.03, frame_mat=None, axis='x'):
    """Welded-mesh panel in a frame, in the plane z = const (axis='x') or x = const (axis='z')."""
    fm = frame_mat or mat
    out = []
    L = x1 - x0

    def B(a0, a1, b0, b1, t, m):
        if axis == 'x':
            return box_span(m, a0, a1, b0, b1, z - t / 2, z + t / 2)
        return box_span(m, z - t / 2, z + t / 2, b0, b1, a0, a1)
    out += [B(x0, x0 + frame, y0, y1, frame, fm), B(x1 - frame, x1, y0, y1, frame, fm),
            B(x0 + frame, x1 - frame, y0, y0 + frame, frame, fm), B(x0 + frame, x1 - frame, y1 - frame, y1, frame, fm)]
    n = max(1, int(L / pitch))
    for i in range(1, n):
        a = x0 + L * i / n
        out.append(B(a - bar / 2, a + bar / 2, y0 + frame, y1 - frame, bar * 0.8, mat))
    m_ = max(1, int((y1 - y0) / (pitch * 2)))
    for i in range(1, m_):
        b = y0 + (y1 - y0) * i / m_
        out.append(B(x0 + frame, x1 - frame, b - bar / 2, b + bar / 2, bar * 1.2, mat))
    return out


def lamp_box(center, size, facing=(1, 0, 0), lens='lamp_lens', body='plastic_black'):
    """Rectangular lamp facing +X: housing plus lens."""
    x, y, z = center
    sx, sy, sz = size
    return [box(body, sx, sy, sz, center), box(lens, 0.006, sy * 0.8, sz * 0.8, (x + sx / 2 + 0.003, y, z))]
