"""Assemblies shared by the Mining pack: mining cab, stairs, walkways with guardrails and
kickplates, ladders, hazard stripes, a scaled-up dozer blade, lamps and beacons.

Imports the shared helpers from `3d-models/` (one folder up) without changing them.
Importing this module also swaps in `hydraulic_parts_with_hoses` as the cylinder builder
used by `meshkit.Model.finalize`, so every hydraulic cylinder in a Mining model carries a
hose run on its barrel (the shared `meshkit.py` is untouched; the swap only happens in
processes that import this file).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import meshkit  # noqa: E402
from meshkit import (MATERIALS, Model, add, box, box_between, box_span, cross, cyl_between,  # noqa: E402,F401
                     cylinder, extrude, from_columns, lathe, mul, norm, polyline_tube, sphere, sub)
from vehicle_parts import hose  # noqa: E402

# ------------------------------------------------------------ materials ---
# Mining pack adds only `dirt` (safety berm); everything else is on the shared list.
MINING_MATERIALS = dict(MATERIALS)
MINING_MATERIALS['dirt'] = {'color': (0.24, 0.16, 0.09), 'metallic': 0.0, 'roughness': 1.0}


# ------------------------------------------------------- cylinder hoses ---
_base_hydraulic_parts = meshkit.hydraulic_parts


def hydraulic_parts_with_hoses(p0, p1, r_barrel, r_rod, extension=0.55,
                               barrel='paint_yellow', rod='chrome', pin='steel_dark'):
    """Shared cylinder geometry plus a hose run along the top of the barrel: a fitting
    at the base-end port and one near the rod-end gland, joined by a hose line."""
    b, r = _base_hydraulic_parts(p0, p1, r_barrel, r_rod, extension, barrel, rod, pin)
    d = sub(p1, p0)
    u = norm(d)
    side = norm(cross(u, (0, 1, 0))) if abs(u[1]) < 0.99 else (0, 0, 1)
    up = norm(cross(side, u))
    if up[1] < 0:
        up = mul(up, -1)
    hr = max(0.011, r_barrel * 0.16)
    a = add(p0, mul(u, r_barrel * 1.2 + 0.09))
    c = add(p0, mul(d, extension * 0.88))
    lift = r_barrel + hr * 2.2
    for q in (a, c):
        b.append(cyl_between('steel', add(q, mul(up, r_barrel * 0.85)), add(q, mul(up, r_barrel + hr * 1.4)), hr * 1.5, 8))
    b += hose([add(a, mul(up, r_barrel + hr)), add(a, mul(up, lift)), add(c, mul(up, lift)),
               add(c, mul(up, r_barrel + hr))], hr)
    return b, r


meshkit.hydraulic_parts = hydraulic_parts_with_hoses


# ----------------------------------------------------------------- blade ---

def blade(xb, H, wc, ww=0.0, wing_deg=0.0, curve=0.28, lean=0.12, thick=0.045,
          back=0.45, ribs=5, k=1.0, mat='paint_yellow'):
    """Dozer blade (copy of dozer_parts.blade with the fixed-size details scaled by `k`).

    xb: x of the cutting edge tip at ground level; H: blade height; wc: centre section
    width; ww / wing_deg: angled wings (U / semi-U blade)."""
    N = 20
    front = lambda t: (xb - curve * math.sin(math.pi * t) + lean * t, H * t)
    ts = [i / N for i in range(N + 1)]

    def band(t0, t1, fwd=0.0, aft=thick):
        seg = [t0 + (t1 - t0) * i / 10 for i in range(11)]
        a = [(front(t)[0] + fwd, front(t)[1]) for t in seg]
        b = [(front(t)[0] - aft, front(t)[1]) for t in reversed(seg)]
        return a + b

    def rib(t0, t1):
        seg = [t0 + (t1 - t0) * i / 10 for i in range(11)]
        a = [(front(t)[0] - thick + 0.002, front(t)[1]) for t in seg]
        return a + [(xb - back, front(t1)[1]), (xb - back, front(t0)[1])]

    moldboard = [(front(t)[0], front(t)[1]) for t in ts] + \
                [(front(t)[0] - thick, front(t)[1]) for t in reversed(ts)]
    edge_poly = band(0.0, 0.17, fwd=0.03 * k, aft=-0.001)
    endbit_poly = band(0.0, 0.16, fwd=0.04 * k, aft=0.0)
    beam_lo, beam_hi = rib(0.04, 0.30), rib(0.76, 0.95)
    lip_poly = band(0.955, 1.0, fwd=0.06 * k, aft=thick + 0.04 * k)

    def section(width):
        g = [extrude(mat, moldboard, width),
             extrude('wear_steel', edge_poly, width),
             extrude(mat, beam_lo, width),
             extrude(mat, beam_hi, width),
             extrude(mat, lip_poly, width)]
        n = max(1, int(width / (0.22 * k)))
        for i in range(n):
            z = -width / 2 + (i + 0.5) * width / n
            x, y = front(0.09)
            g.append(cylinder('steel_dark', 0.016 * k, 0.03 * k, 6).rotate('z', 90 + 18).at((x + 0.03 * k, y, z)))
        return [p.move(z=width / 2) for p in g]

    out = [p.move(z=-wc / 2) for p in section(wc)]
    for i in range(ribs):
        z = -wc / 2 + 0.06 * k + (wc - 0.12 * k) * i / (ribs - 1)
        out.append(extrude(mat, rib(0.03, 0.97), 0.035 * k).move(z=z))
    out.append(box_span(mat, xb - back - 0.02 * k, xb - back + 0.02 * k, H * 0.05, H * 0.85, -wc / 2 + 0.05, wc / 2 - 0.05))
    endplate = band(0.0, 1.0, fwd=0.10 * k, aft=thick)
    endbit = lambda w: extrude('wear_steel', endbit_poly, w)
    if ww > 0:
        kk = math.tan(math.radians(wing_deg))
        shear = ((1, 0, kk), (0, 1, 0), (0, 0, 1))
        wing = section(ww)
        wing.append(extrude(mat, rib(0.03, 0.97), 0.035 * k).move(z=ww - 0.04 * k))
        wing.append(extrude(mat, endplate, 0.03 * k).move(z=ww + 0.015 * k))
        wing.append(endbit(0.32 * k).move(z=ww - 0.165 * k))
        wing = [g.transform(shear).move(z=wc / 2) for g in wing]
        out += wing + [g.mirror_z() for g in wing]
    else:
        tip = [extrude(mat, endplate, 0.03 * k).move(z=wc / 2 + 0.015 * k),
               endbit(0.32 * k).move(z=wc / 2 - 0.165 * k)]
        out += tip + [g.mirror_z() for g in tip]
    return out


# ----------------------------------------------------------- lamps etc ---

def lamp(center, facing=1, size=1.0, axis='x'):
    """LED work lamp: housing plus emissive lens facing +axis (facing=1) or -axis."""
    x, y, z = center
    s = size
    if axis == 'x':
        return [box('chassis_black', 0.14 * s, 0.16 * s, 0.22 * s, center),
                box('lamp_lens', 0.01, 0.12 * s, 0.18 * s, (x + facing * 0.074 * s, y, z)),
                box('chassis_black', 0.04, 0.08 * s, 0.04, (x - facing * 0.08 * s, y - 0.07 * s, z))]
    return [box('chassis_black', 0.22 * s, 0.16 * s, 0.14 * s, center),
            box('lamp_lens', 0.18 * s, 0.12 * s, 0.01, (x, y, z + facing * 0.074 * s)),
            box('chassis_black', 0.04, 0.08 * s, 0.04, (x, y - 0.07 * s, z - facing * 0.08 * s))]


def beacon(base, r=0.07):
    x, y, z = base
    return [cylinder('chassis_black', r * 1.25, 0.05, 16).at((x, y + 0.025, z)),
            cylinder('lamp_beacon', r, r * 1.9, 16, r_top=r * 0.75).at((x, y + 0.05 + r * 0.95, z)),
            cylinder('chassis_black', r * 0.8, 0.02, 12).at((x, y + 0.06 + r * 1.9, z))]


def reflector(center, normal_axis='x', facing=1, mat='lamp_red', w=0.08, h=0.08):
    x, y, z = center
    if normal_axis == 'x':
        return box(mat, 0.012, h, w, (x + facing * 0.006, y, z))
    return box(mat, w, h, 0.012, (x, y, z + facing * 0.006))


# ----------------------------------------------------------- hazard stripes ---

def _clip(poly, f):
    """Keep the part of a 2D polygon where f(p) >= 0 (f linear)."""
    out = []
    for i in range(len(poly)):
        a, b = poly[i - 1], poly[i]
        fa, fb = f(a), f(b)
        if fa >= 0:
            out.append(a)
        if (fa >= 0) != (fb >= 0):
            t = fa / (fa - fb)
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    # rotate so the order matches the input winding (start point irrelevant)
    return out


def hazard_stripes(origin, u, v, w, h, pitch=0.15, thick=0.006, slope=1.0, mats=('paint_red', 'paint_white')):
    """Diagonal reflective red/white stripes on a w x h rectangle. `origin` is one corner,
    `u` / `v` unit vectors along the width / height; the stripes stand `thick` proud along
    u x v (so that cross product must point out of the surface)."""
    n = norm(cross(u, v))
    M = from_columns(norm(u), norm(v), n)
    rect = [(0, 0), (w, 0), (w, h), (0, h)]
    out = []
    k0 = int(math.floor(-slope * h / pitch)) - 1
    k1 = int(math.ceil(w / pitch)) + 1
    for k in range(k0, k1):
        lo, hi = k * pitch, (k + 1) * pitch
        poly = _clip(rect, lambda p: (p[0] + slope * p[1]) - lo)
        if len(poly) < 3:
            continue
        poly = _clip(poly, lambda p: hi - (p[0] + slope * p[1]))
        if len(poly) < 3:
            continue
        area = 0.5 * abs(sum(poly[i][0] * poly[i - 1][1] - poly[i - 1][0] * poly[i][1] for i in range(len(poly))))
        if area < 1e-4:
            continue
        g = extrude(mats[k % 2], poly, thick).move(z=thick / 2 + 0.001)
        out.append(g.transform(M, origin))
    return out


# ------------------------------------------------------- access systems ---

def stairs(xa, ya, xb, yb, zc, width=0.75, rise=0.24, rails=(1, -1), mat='paint_yellow', tread='steel_dark'):
    """Inclined access stair running in x from (xa, ya) (bottom tread) up to (xb, yb)
    (landing level), centred on z = zc. Stringers, grating treads and handrails on the
    sides listed in `rails` (+1 = +z side, -1 = -z side)."""
    n = max(2, round((yb - ya) / rise))
    hw = width / 2
    sx = 1 if xb > xa else -1
    run = abs(xb - xa) / n
    out = []
    for i in range(n):
        y = ya + (yb - ya) * i / n
        x = xa + (xb - xa) * i / n
        out.append(box_span(tread, x - run * 0.5 - 0.02, x + run * 0.5 + 0.02, y - 0.03, y, zc - hw + 0.03, zc + hw - 0.03))
        out.append(box_span(mat, x - run * 0.5 - 0.025, x - run * 0.5 + 0.005, y - 0.07, y - 0.004, zc - hw + 0.035, zc + hw - 0.035) if sx > 0 else
                   box_span(mat, x + run * 0.5 - 0.005, x + run * 0.5 + 0.025, y - 0.07, y - 0.004, zc - hw + 0.035, zc + hw - 0.035))
    for s in (1, -1):
        z = zc + s * (hw - 0.015)
        out.append(box_between(mat, (xa - sx * 0.15, ya - 0.12, z), (xb, yb - 0.10, z), 0.03, 0.20))
    for s in rails:
        z = zc + s * (hw + 0.02)
        top = [(xa, ya + 1.0, z), (xb, yb + 1.0, z), (xb + sx * 0.35, yb + 1.0, z)]
        out += polyline_tube(mat, top, 0.022, 8)
        out.append(cyl_between(mat, (xa, ya - 0.05, z), (xa, ya + 1.0, z), 0.022, 8))
        out.append(cyl_between(mat, ((xa + xb) / 2, (ya + yb) / 2 - 0.05, z), ((xa + xb) / 2, (ya + yb) / 2 + 1.0, z), 0.022, 8))
        out.append(cyl_between(mat, (xa, ya + 0.5, z), (xb, yb + 0.5, z), 0.016, 8))
    return out


def guardrail(pts, deck_y, h=1.07, kick=0.12, mat='paint_yellow', post_gap=1.6, posts_at_ends=True, side_out=None):
    """Handrail along the deck edge through (x, z) points: top rail, mid rail, posts and a
    kickplate. `side_out` = (dx, dz) pushes the kickplate a little to the outside."""
    out = []
    p3 = [(x, deck_y + h, z) for x, z in pts]
    out += polyline_tube(mat, p3, 0.022, 8)
    for (xa, za), (xb, zb) in zip(pts, pts[1:]):
        out.append(cyl_between(mat, (xa, deck_y + h * 0.52, za), (xb, deck_y + h * 0.52, zb), 0.016, 8))
        L = math.hypot(xb - xa, zb - za)
        n = max(1, math.ceil(L / post_gap))
        for i in range(n + 1):
            if not posts_at_ends and (i == 0 and (xa, za) == pts[0] or i == n and (xb, zb) == pts[-1]):
                continue
            if i == 0 and (xa, za) != pts[0]:
                continue  # shared with previous segment
            t = i / n
            x, z = xa + (xb - xa) * t, za + (zb - za) * t
            out.append(cyl_between(mat, (x, deck_y, z), (x, deck_y + h, z), 0.022, 8))
        # kickplate
        dx, dz = (xb - xa) / L, (zb - za) / L
        ox, oz = side_out if side_out else (0.0, 0.0)
        out.append(box_between(mat, (xa + ox, deck_y + kick / 2, za + oz), (xb + ox, deck_y + kick / 2, zb + oz), 0.008, kick))
    return out


def ladder(x, y0, y1, zc, width=0.55, axis='z', mat='paint_yellow', rung='steel_dark', lean=0.0):
    """Vertical (or slightly leaning) ladder; rails 'width' apart along `axis`."""
    out = []
    hw = width / 2
    n = max(2, round((y1 - y0) / 0.3))
    for s in (1, -1):
        if axis == 'z':
            a, b = (x, y0, zc + s * hw), (x + lean, y1, zc + s * hw)
        else:
            a, b = (x + s * hw, y0, zc), (x + s * hw, y1, zc + lean)
        out.append(box_between(mat, a, b, 0.04, 0.07, (1, 0, 0) if axis == 'z' else (0, 0, 1)))
    for i in range(n):
        t = (i + 0.5) / n
        y = y0 + (y1 - y0) * t
        if axis == 'z':
            xx = x + lean * t
            out.append(cyl_between(rung, (xx, y, zc - hw), (xx, y, zc + hw), 0.016, 8))
        else:
            zz = zc + lean * t
            out.append(cyl_between(rung, (x - hw, y, zz), (x + hw, y, zz), 0.016, 8))
    return out


def deck(x0, x1, z0, z1, y, t=0.05, mat='paint_yellow', grating='steel_dark'):
    """Walkway deck: structural plate with a non-slip grating top."""
    return [box_span(mat, x0, x1, y - t, y - 0.006, z0, z1),
            box_span(grating, x0 + 0.02, x1 - 0.02, y - 0.006, y, z0 + 0.02, z1 - 0.02)]


# ------------------------------------------------------------------- cab ---

def cab(x0, x1, y0, y1, hw, sill=0.45, mat='paint_yellow', roof_mat='paint_white', door_side=1,
        post=0.12, interior=True, seat_x=0.42, wheel=False):
    """ROPS/FOPS operator cab for large machines. x0 rear, x1 front, floor y0, roof
    underside y1, half-width hw. Adapted from dozer_parts.cab: heavier posts, separate
    roof material, roof-mounted pressuriser / A-C unit, LED lamps, beacon, aerial."""
    out = []
    p = post
    out.append(box_span('chassis_black', x0 + 0.03, x1 - 0.03, y0 + 0.004, y0 + 0.06, -hw + 0.03, hw - 0.03))
    ys = y0 + sill
    out.append(box_span(mat, x0, x0 + 0.03, y0, ys, -hw, hw))
    out.append(box_span(mat, x1 - 0.03, x1, y0, ys, -hw, hw))
    for s in (1, -1):
        out.append(box_span(mat, x0 + 0.03, x1 - 0.03, y0, ys, s * (hw - 0.03), s * hw))
    for px in (x0 + p / 2, x1 - p / 2):
        for s in (1, -1):
            out.append(box(mat, p, y1 - ys, p, (px, (ys + y1) / 2, s * (hw - p / 2))))
    for s in (1, -1):
        out.append(box_span(mat, x0 + p, x1 - p, y1 - 0.10, y1, s * (hw - p), s * hw))
        out.append(box(mat, 0.07, y1 - ys - 0.09, 0.07, ((x0 + x1) / 2 - 0.1, (ys + y1 - 0.1) / 2, s * (hw - 0.035))))
    out.append(box_span(mat, x0 + p, x1 - p, y1 - 0.10, y1, -hw + p, -hw + p + 0.06))
    out.append(box_span(mat, x0, x0 + p, y1 - 0.10, y1, -hw + p, hw - p))
    out.append(box_span(mat, x1 - p, x1, y1 - 0.10, y1, -hw + p, hw - p))
    # roof: ROPS canopy in the roof colour with a drip rail, sun visor and A-C / pressuriser
    out.append(box_span(roof_mat, x0 - 0.10, x1 + 0.14, y1, y1 + 0.14, -hw - 0.09, hw + 0.09))
    out.append(box_span('chassis_black', x0 - 0.10, x1 + 0.14, y1 - 0.03, y1 - 0.003, -hw - 0.09, hw + 0.09))
    out.append(box_span(roof_mat, x0 + 0.25, x0 + 1.05, y1 + 0.14, y1 + 0.42, -0.45, 0.45))
    out.append(box_span('steel_dark', x0 + 0.30, x0 + 1.00, y1 + 0.20, y1 + 0.36, 0.45, 0.452))
    for i in range(5):
        out.append(box_span('chassis_black', x0 + 0.34 + i * 0.13, x0 + 0.42 + i * 0.13, y1 + 0.22, y1 + 0.34, 0.452, 0.458))
    # glazing
    g0, g1 = ys, y1 - 0.10
    out.append(box_span('glass', x1 - 0.02, x1 - 0.01, g0, g1, -hw + p, hw - p))
    out.append(box_span('glass', x0 + 0.01, x0 + 0.02, g0, g1, -hw + p, hw - p))
    for s in (1, -1):
        out.append(box_span('glass', x0 + p, x1 - p, g0, g1, s * (hw - 0.02), s * (hw - 0.01)))
    # door seal outline, handle and grab handles
    dx0, dx1 = (x0 + x1) / 2 - 0.065, x1 - p
    zd = door_side * (hw + 0.004)
    for a in (dx0, dx1):
        out.append(box_span('rubber_trim', a - 0.012, a + 0.012, y0 + 0.05, g1, zd - 0.006, zd + 0.006))
    out.append(box_span('steel_dark', dx0 + 0.05, dx0 + 0.18, ys - 0.14, ys - 0.10, zd, zd + door_side * 0.03))
    zz = door_side * (hw + 0.07)
    for px in (x1 + 0.03, dx0 - 0.09):
        out += polyline_tube('chrome', [(px, y0 + 0.3, door_side * hw), (px, y0 + 0.3, zz),
                                        (px, y1 - 0.3, zz), (px, y1 - 0.3, door_side * hw)], 0.018, 8)
    # wipers
    out.append(box('rubber_trim', 0.015, 0.75, 0.025, (x1 + 0.002, g0 + 0.40, -0.25)).rotate('x', 25, (x1, g0 + 0.05, -0.25)))
    out.append(box('rubber_trim', 0.015, 0.75, 0.025, (x1 + 0.002, g0 + 0.40, 0.35)).rotate('x', 25, (x1, g0 + 0.05, 0.35)))
    out.append(box_span('rubber_trim', x0 + 0.001, x0 + 0.008, g0 + 0.05, g0 + 0.07, -0.3, 0.3))
    # mirrors
    for s in (1, -1):
        a = (x1 - 0.05, y1 - 0.18, s * hw)
        b = (x1 + 0.12, y1 - 0.12, s * (hw + 0.36))
        out.append(cyl_between('steel_dark', a, b, 0.014, 8))
        out.append(box('chassis_black', 0.05, 0.34, 0.22, (b[0], b[1] - 0.15, b[2])))
        out.append(box('glass', 0.01, 0.30, 0.18, (b[0] - 0.027, b[1] - 0.15, b[2])))
    # roof lights, beacon and aerial
    for s in (1, -1):
        out += lamp((x1 + 0.10, y1 + 0.22, s * (hw - 0.18)), 1)
        out += lamp((x0 - 0.08, y1 + 0.22, s * (hw - 0.18)), -1)
    out += beacon((x0 + 0.35, y1 + 0.14, -hw + 0.30))
    out += beacon((x0 + 0.35, y1 + 0.14, hw - 0.30))
    out.append(cyl_between('steel_dark', (x1 - 0.3, y1 + 0.14, hw - 0.45), (x1 - 0.3, y1 + 0.60, hw - 0.45), 0.007, 6))
    if not interior:
        return out
    sx = x0 + (x1 - x0) * seat_x
    out.append(box_span('chassis_black', sx - 0.25, sx + 0.25, y0 + 0.06, y0 + 0.40, -0.22, 0.22))
    out.append(box_span('seat_fabric', sx - 0.27, sx + 0.25, y0 + 0.40, y0 + 0.53, -0.27, 0.27))
    out.append(box('seat_fabric', 0.11, 0.66, 0.52, (sx - 0.28, y0 + 0.86, 0)).rotate('z', 10, (sx - 0.28, y0 + 0.53, 0)))
    out.append(box('seat_fabric', 0.09, 0.17, 0.32, (sx - 0.35, y0 + 1.28, 0)))
    for s in (1, -1):
        zc = min(0.48, hw - 0.08)
        out.append(box_span('chassis_black', sx - 0.32, sx + 0.36, y0 + 0.06, y0 + 0.64, s * 0.31, s * zc))
        out.append(box_span('seat_fabric', sx - 0.2, sx + 0.25, y0 + 0.64, y0 + 0.69, s * 0.33, s * (zc - 0.02)))
        top = (sx + 0.22, y0 + 0.69, s * 0.40)
        out.append(cyl_between('steel_dark', top, (top[0] + 0.02, top[1] + 0.15, top[2]), 0.013, 8))
        out.append(sphere('plastic_black', 0.038, 10, 6).at((top[0] + 0.02, top[1] + 0.18, top[2])))
    out.append(box_span('chassis_black', x1 - 0.32, x1 - 0.03, y0 + 0.06, ys + 0.06, -0.35, 0.35))
    out.append(box('plastic_black', 0.02, 0.18, 0.30, (x1 - 0.34, ys + 0.08, 0)).rotate('z', 25, (x1 - 0.34, ys, 0)))
    out.append(box('plastic_black', 0.02, 0.16, 0.22, (x1 - 0.04, y1 - 0.25, hw - 0.35)).rotate('z', -20, (x1 - 0.04, y1 - 0.25, 0)))
    for s in (1, -1):
        out.append(box('chassis_black', 0.07, 0.20, 0.09, (x1 - 0.45, y0 + 0.16, s * 0.13)).rotate('z', -30, (x1 - 0.45, y0 + 0.06, 0)))
    if wheel:
        from vehicle_parts import steering_wheel
        out += steering_wheel((x1 - 0.48, ys + 0.30, 0), 0.20, 35)
    return out


# ------------------------------------------------------------ misc parts ---

def exhaust(x, y0, y1, z, r=0.10):
    return [cylinder('chassis_black', r * 1.6, 0.06, 16).at((x, y0 + 0.03, z)),
            cyl_between('steel_dark', (x, y0 + 0.01, z), (x, y1, z), r, 18),
            cylinder('steel_dark', r * 1.18, 0.16, 18).at((x, y1 - 0.25, z)),
            cyl_between('steel_dark', (x, y1, z), (x - 0.06, y1 + 0.09, z), r, 18)]


def precleaner(x, y0, z, s=1.0):
    h = 0.30 * s
    return [cyl_between('chassis_black', (x, y0, z), (x, y0 + h, z), 0.06 * s, 14),
            cylinder('plastic_black', 0.17 * s, 0.28 * s, 20).at((x, y0 + h + 0.14 * s, z)),
            cylinder('glass', 0.165 * s, 0.18 * s, 20).at((x, y0 + h + 0.095 * s, z)),
            cylinder('chassis_black', 0.18 * s, 0.06 * s, 20, r_top=0.10 * s).at((x, y0 + h + 0.31 * s, z))]


def louvres(x0, x1, y0, y1, z, side=1, pitch=0.07, mat='steel_dark'):
    """Rows of angled slats standing proud of a side panel at `z`."""
    out = [box_span('chassis_black', x0 - 0.02, x1 + 0.02, y0 - 0.02, y1 + 0.02,
                    z - side * 0.002, z + side * 0.004)]
    y = y0 + pitch / 2
    while y < y1:
        out.append(box(mat, x1 - x0, 0.014, 0.045).rotate('x', side * -35).at(((x0 + x1) / 2, y, z + side * 0.02)))
        y += pitch
    return out


def tank_cap(x, y, z, r=0.09):
    return [cylinder('chassis_black', r, 0.05, 16).at((x, y + 0.025, z)),
            cylinder('steel_dark', r * 0.5, 0.03, 8).at((x, y + 0.065, z))]


def sight_gauge(x, y0, y1, z, side=1):
    return [box_span('steel_dark', x - 0.04, x + 0.04, y0, y1, z, z + side * 0.025),
            box_span('glass', x - 0.02, x + 0.02, y0 + 0.03, y1 - 0.03, z + side * 0.025, z + side * 0.03)]


def save(model, slug):
    """Write Mining/<slug>/<slug_with_underscores>.glb."""
    meshkit.save(model, os.path.join('Mining', slug), slug.replace('-', '_'))
