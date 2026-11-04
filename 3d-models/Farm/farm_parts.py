"""Shared assemblies for the Farm pack: R1 tractor tyres, agricultural wheels, the
tractor cab, three-point hitch, small fittings and the farm material list.

The generic helpers live one folder up (meshkit, uvkit, vehicle_parts, ...); this
module puts that folder on sys.path so the Farm build scripts can import them.
Nothing in the shared files is modified.
"""
import math
import os
import sys

FARM = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(FARM)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from meshkit import (MATERIALS, Geo, add, box, box_between, box_span, cross, cyl_between,  # noqa: E402
                     cylinder, dot, extrude, lathe, mul, norm, polyline_tube, save, sphere, sub)
from vehicle_parts import rim  # noqa: E402

# House colour scheme for every machine: paint_yellow bodywork, chassis_black frames,
# paint_gray_dark wheels and rims. Three materials are added for farm surfaces that
# nothing on the standard list fits.
FARM_MATERIALS = dict(MATERIALS)
FARM_MATERIALS.update({
    'hay':              {'color': (0.62, 0.48, 0.20), 'metallic': 0.0, 'roughness': 0.95},
    'steel_galvanized': {'color': (0.68, 0.70, 0.72), 'metallic': 0.6, 'roughness': 0.45},
    'grain':            {'color': (0.60, 0.42, 0.14), 'metallic': 0.0, 'roughness': 0.9},
})
NEW_MATERIALS = ('hay', 'steel_galvanized', 'grain')

BODY, FRAME, WHEEL = 'paint_yellow', 'chassis_black', 'paint_gray_dark'


def save_farm(model, slug, budget):
    """Write Farm/<slug>/<slug_>.glb and check the triangle budget."""
    tris = model.stats()[1] if model.done else None
    save(model, 'Farm/' + slug, slug.replace('-', '_'))
    tris = model.stats()[1]
    assert tris < budget, '%s: %d triangles is over the %d budget' % (model.name, tris, budget)
    return tris


# ------------------------------------------------------------------ tyres ---

def _carcass_profile(r, w, bead_r, d):
    base = r - d
    hw = w / 2
    s = base - bead_r
    side = [(bead_r, -hw * 0.80), (bead_r + 0.15 * s, -hw * 0.93), (bead_r + 0.45 * s, -hw),
            (bead_r + 0.75 * s, -hw * 0.98), (base - 0.10 * s, -hw * 0.93), (base - 0.025 * s, -hw * 0.82),
            (base, -hw * 0.66)]
    return side, base, hw


def _radius_at(side, y):
    """Outer carcass radius at axial position y (crown is flat between the shoulders)."""
    ay = -abs(y)
    pts = side[3:]
    if ay >= pts[-1][1]:
        return pts[-1][0]
    for (r0, y0), (r1, y1) in zip(pts, pts[1:]):
        if y0 <= ay <= y1:
            t = (ay - y0) / (y1 - y0) if y1 != y0 else 0
            return r0 + (r1 - r0) * t
    return pts[0][0]


def _sweep_lug(mat, samples, b, d, side_prof):
    """One tread bar following the carcass. samples: [(phi, y)] along the bar centreline,
    b: bar width, d: bar height. Tyre axis is +Y; a point is (R sin phi, y, R cos phi)."""
    P = lambda R, phi, y: (R * math.sin(phi), y, R * math.cos(phi))
    rings = []
    rc = _radius_at(side_prof, 0.0)
    for i, (phi, y) in enumerate(samples):
        a = samples[max(i - 1, 0)]
        c = samples[min(i + 1, len(samples) - 1)]
        ds, dy = (c[0] - a[0]) * rc, c[1] - a[1]
        l = math.hypot(ds, dy) or 1.0
        ps, py = -dy / l * b / 2, ds / l * b / 2   # perpendicular in the unrolled tread plane
        R0 = _radius_at(side_prof, y)
        ring = []
        for sg in (-1, 1):
            ph, yy = phi + sg * ps / rc, y + sg * py
            Rb = _radius_at(side_prof, yy)
            ring.append((P(Rb - 0.006, ph, yy), P(Rb + d, ph, yy)))
        rings.append((ring, P(R0 + d / 2, phi, y)))
    pos, nrm, idx = [], [], []

    def quad(a, b_, c, e, ref):
        n = norm(cross(sub(b_, a), sub(e, a)))
        cen = mul(add(add(a, b_), add(c, e)), 0.25)
        if dot(n, sub(cen, ref)) < 0:
            n = mul(n, -1)
        k = len(pos)
        pos.extend([a, b_, c, e])
        nrm.extend([n] * 4)
        idx.extend([(k, k + 1, k + 2), (k, k + 2, k + 3)])

    for (r0, c0), (r1, c1) in zip(rings, rings[1:]):
        ref = mul(add(c0, c1), 0.5)
        (l0b, l0t), (h0b, h0t) = r0
        (l1b, l1t), (h1b, h1t) = r1
        quad(l0t, h0t, h1t, l1t, ref)      # top
        quad(l0b, l0t, l1t, l1b, ref)      # side
        quad(h0b, h0t, h1t, h1b, ref)      # other side
    for (ring, c), (ring2, c2) in ((rings[0], rings[1]), (rings[-1], rings[-2])):
        (lb, lt), (hb, ht) = ring
        quad(lb, lt, ht, hb, c2)        # end caps face away from the bar
    return Geo(mat, pos, nrm, idx)


def r1_tyre(r, w, bead_r, lugs=None, depth=None, angle=48, segs=44, hand=1):
    """Agricultural R1 drive tyre: open-centre chevron bars that overlap the centre line
    and wrap onto the shoulder. Built around +Y (outer face +Y) like vehicle_parts.tyre.
    `hand` flips the chevron so the bar tips lead in forward rotation."""
    d = depth if depth is not None else 0.028 + 0.025 * r
    side, base, hw = _carcass_profile(r, w, bead_r, d)
    prof = side + [(rr, -y) for rr, y in reversed(side)]
    prof.append(prof[0])
    out = [lathe('tyre_rubber', prof, segs)]
    n = lugs or int(round(2 * math.pi * r / 0.24))
    b = 0.030 + 0.035 * r
    k_ang = math.tan(math.radians(angle))
    rc = base
    for k in range(n):
        for sgn, off in ((1, 0.0), (-1, 0.5)):
            phi0 = 2 * math.pi * (k + off) / n
            pts = []
            for i in range(4):
                u = i / 3
                ya = -0.05 * w + 0.545 * w * u          # from just across the centre to the shoulder
                s = k_ang * 0.5 * w * (u - 0.22 * u * u)  # bar flattens toward the shoulder
                pts.append((phi0 + hand * s / rc, sgn * ya))
            out.append(_sweep_lug('tyre_rubber', pts, b, d, side))
    return out


def ag_wheel(r, w, bead_r, lugs=None, hand=1, rim_mat=WHEEL, rim_style='steel', segs=44):
    """R1 tyre on a dished steel rim (house colour), axis +Y, outer face +Y."""
    return r1_tyre(r, w, bead_r, lugs, hand=hand, segs=segs) + rim(bead_r, w, rim_style, rim_mat, nuts=8)


def implement_tyre(r, w, bead_r, ribs=6, segs=40):
    """Flotation / implement tyre with straight circumferential ribs (balers, trailers)."""
    d = 0.018
    side, base, hw = _carcass_profile(r, w, bead_r, d)
    crown = []
    ys = [-hw * 0.66 + i * hw * 1.32 / (2 * ribs) for i in range(2 * ribs + 1)]
    for i, y in enumerate(ys):
        crown.append((r if i % 2 else base, y))
    prof = side + crown[1:-1] + [(rr, -y) for rr, y in reversed(side)]
    prof.append(prof[0])
    return [lathe('tyre_rubber', prof, segs)]


def place(geos, center, outward=1):
    """Turn a +Y-facing wheel so its outer face points to +Z (outward=1) or -Z,
    keeping the tread pattern's direction of travel (mirror rather than rotate)."""
    out = []
    for g in geos:
        g = g.rotate('x', 90)
        if outward < 0:
            g = g.mirror_z()
        out.append(g.at(center))
    return out


# ---------------------------------------------------------------- fittings ---

def lamp(center, facing=1, size=1.0, lens='lamp_lens'):
    """Work lamp facing +x (facing=1) or -x."""
    x, y, z = center
    s = size
    return [box(FRAME, 0.10 * s, 0.10 * s, 0.14 * s, center),
            box(lens, 0.008, 0.075 * s, 0.11 * s, (x + facing * 0.054 * s, y, z)),
            box(FRAME, 0.03, 0.05 * s, 0.03, (x - facing * 0.06 * s, y - 0.06 * s, z))]


def flat_lamp(center, size, facing=1, lens='lamp_red', axis='x'):
    """Flush rectangular lamp on a surface facing +/-x (axis='x') or +/-z (axis='z')."""
    x, y, z = center
    a, h = size
    if axis == 'x':
        return [box(lens, 0.012, h, a, (x + facing * 0.006, y, z))]
    return [box(lens, a, h, 0.012, (x, y, z + facing * 0.006))]


def hose(points, r=0.012):
    out = [cyl_between('hose_rubber', a, b, r, 8, caps=False) for a, b in zip(points, points[1:])]
    out += [sphere('hose_rubber', r, 8, 4).at(p) for p in points[1:-1]]
    out += [cylinder('steel', r * 1.4, 0.03, 8).at(p) for p in (points[0], points[-1])]
    return out


def handrail(points, r=0.016, mat=FRAME):
    return polyline_tube(mat, points, r, 10)


def step(x0, x1, y, z0, z1, mat='steel_dark'):
    """Open tread step: frame with three slats."""
    out = [box_span(mat, x0, x1, y - 0.02, y, z0, z0 + math.copysign(0.025, z1 - z0)),
           box_span(mat, x0, x1, y - 0.02, y, z1 - math.copysign(0.025, z1 - z0), z1)]
    for k in range(3):
        zz = z0 + (z1 - z0) * (k + 0.5) / 3
        out.append(box_span(mat, x0 + 0.01, x1 - 0.01, y - 0.012, y, zz - 0.015, zz + 0.015))
    return out


def ladder(p0, p1, width, rungs, side_mat=FRAME, rung_mat='steel_dark', r=0.018, axis=(0, 0, 1)):
    """Straight ladder from p0 (bottom centre) to p1 (top centre); width measured along `axis`."""
    h = mul(norm(axis), width / 2)
    out = [cyl_between(side_mat, sub(p0, h), sub(p1, h), r, 10),
           cyl_between(side_mat, add(p0, h), add(p1, h), r, 10)]
    for k in range(rungs):
        t = (k + 0.5) / rungs
        c = add(p0, mul(sub(p1, p0), t))
        out.append(cyl_between(rung_mat, sub(c, h), add(c, h), r * 0.8, 8))
    return out


def mirror(base, tip, facing=-1):
    """Arm from `base` to a rear-view mirror head at `tip` (glass facing -x by default)."""
    x, y, z = tip
    return [cyl_between('steel_dark', base, tip, 0.012, 8),
            box(FRAME, 0.04, 0.24, 0.16, (x, y - 0.10, z)),
            box('glass', 0.008, 0.21, 0.13, (x + facing * 0.023, y - 0.10, z))]


def beacon(center):
    x, y, z = center
    return [cylinder(FRAME, 0.07, 0.04, 16).at((x, y + 0.02, z)),
            cylinder('lamp_beacon', 0.06, 0.12, 16, r_top=0.045).at((x, y + 0.10, z))]


# --------------------------------------------------------------------- cab ---

def tractor_cab(x0, x1, yf, yr, hw, ys, x_fender, hw_low, door_side=-1, mat=BODY, roof_mat=BODY,
                lower_front=None, seat_x=None, sill_front=None, wheel_tilt=40, loader_stick=True, door_x0=None):
    """Four-post tractor cab with glazing all round.

    x0/x1: rear/front of the cab, yf floor, yr underside of the roof, hw half width of
    the glazed upper part; below `ys` (top of the rear fenders) the cab narrows to
    `hw_low` behind `x_fender` so the rear tyres fit, and stays full width ahead of it.
    Returns a list of Geo."""
    out = []
    P = 0.07
    sx = seat_x if seat_x is not None else x1 - 0.85
    sill_front = sill_front if sill_front is not None else yf + 0.30
    # floor and lower body
    out.append(box_span(FRAME, x0, x1, yf - 0.05, yf, -hw_low, hw_low))
    out.append(box_span(FRAME, x_fender, x1, yf - 0.05, yf, -hw, hw))
    out.append(box_span('rubber_trim', x0 + 0.04, x1 - 0.06, yf, yf + 0.01, -min(hw_low, hw - P) + 0.03, min(hw_low, hw - P) - 0.03))
    for s in (1, -1):
        if x_fender > x0 + 0.05:
            out.append(box_span(mat, x0, x_fender, yf, ys, s * (hw_low - 0.025), s * hw_low))     # tunnel wall
            out.append(box_span(mat, x0 + 0.03, x_fender, ys - 0.025, ys, s * hw_low, s * (hw - 0.02)))  # sill shelf
    out.append(box_span(mat, x0, x0 + 0.03, yf, ys, -min(hw_low, hw - P), min(hw_low, hw - P)))
    # corner posts and roof frame
    for px in (x0 + P / 2, x1 - P / 2):
        for s in (1, -1):
            yb = yf if px > x_fender else ys - 0.03
            out.append(box(FRAME, P, yr - yb, P, (px, (yr + yb) / 2, s * (hw - P / 2))))
    for s in (1, -1):
        out.append(box_span(FRAME, x0, x1, yr - 0.06, yr, s * (hw - P), s * hw))
    out.append(box_span(FRAME, x0, x0 + P, yr - 0.06, yr, -hw + P, hw - P))
    out.append(box_span(FRAME, x1 - P, x1, yr - 0.06, yr, -hw + P, hw - P))
    # roof: rounded shell with a flat liner underneath and a vent on top
    from bodykit import rbox
    out.append(rbox(roof_mat, x1 - x0 + 0.22, 0.13, 2 * hw + 0.12, 0.05, ((x0 + x1) / 2 + 0.02, yr + 0.065, 0)))
    out.append(box_span(FRAME, x0 + 0.30, x0 + 0.62, yr + 0.13, yr + 0.16, -0.25, 0.25))
    out.append(box_span('plastic_black', x0 + P, x1 - P, yr - 0.03, yr - 0.004, -hw + P, hw - P))
    # glazing: windscreen down to the dash, rear window, side glass above the fenders
    out.append(box_span('glass', x1 - 0.022, x1 - 0.010, sill_front, yr - 0.06, -hw + P, hw - P))
    out.append(box_span('glass', x0 + 0.010, x0 + 0.022, ys + 0.04, yr - 0.06, -hw + P, hw - P))
    for s in (1, -1):
        out.append(box_span('glass', x0 + P, x1 - P, ys, yr - 0.06, s * (hw - 0.018), s * (hw - 0.006)))
        # lower glass of the door / side panel ahead of the fender
        xl = max(x_fender, x0 + P)
        out.append(box_span('glass', xl + 0.03, x1 - P, yf + 0.25, ys, s * (hw - 0.018), s * (hw - 0.006)))
        out.append(box_span(mat, xl, x1 - P, yf, yf + 0.25, s * (hw - 0.025), s * hw))
        out.append(box_span(FRAME, xl, xl + 0.03, yf + 0.25, ys, s * (hw - 0.03), s * (hw - 0.004)))
    # front panel below the windscreen
    lf = lower_front if lower_front is not None else 0.0
    if sill_front > yf:
        out.append(box_span(mat, x1 - 0.03, x1, yf, sill_front, -hw + P, hw - P))
    # door outline, hinges, handle (door side)
    zd = door_side * (hw + 0.002)
    xd = door_x0 if door_x0 is not None else x_fender
    for a in (xd - 0.02, x1 - P - 0.01):
        out.append(box_span('rubber_trim', a - 0.012, a + 0.012, yf + 0.02, yr - 0.07, zd - 0.006, zd + 0.006))
    out.append(box_span('rubber_trim', xd - 0.02, x1 - P, yr - 0.08, yr - 0.07, zd - 0.006, zd + 0.006))
    out.append(box_span('steel_dark', xd + 0.03, xd + 0.14, ys - 0.12, ys - 0.09, zd, zd + door_side * 0.03))
    for y in (yf + 0.4, yr - 0.4):
        out.append(cyl_between('steel_dark', (x1 - P / 2, y - 0.05, zd), (x1 - P / 2, y + 0.05, zd), 0.016, 8))
    # wipers, mirrors, lights, beacon, antenna
    out.append(box('rubber_trim', 0.012, 0.55, 0.022, (x1 + 0.002, sill_front + 0.33, 0.1))
               .rotate('x', 20, (x1, sill_front + 0.06, 0.1)))
    out.append(box('rubber_trim', 0.012, 0.45, 0.022, (x0 - 0.002, ys + 0.30, 0)).rotate('x', -20, (x0, ys + 0.08, 0)))
    for s in (1, -1):
        out += mirror((x1 - 0.03, yr - 0.12, s * hw), (x1 + 0.18, yr - 0.05, s * (hw + 0.36)))
        out += lamp((x1 + 0.10, yr + 0.07, s * (hw - 0.12)), 1, 0.9)
        out += lamp((x0 - 0.06, yr + 0.07, s * (hw - 0.12)), -1, 0.9)
        out.append(box('lamp_amber', 0.05, 0.05, 0.05, (x1 + 0.10, yr + 0.07, s * (hw - 0.32))))
    out += beacon((x0 + 0.22, yr + 0.13, -hw + 0.22))
    out.append(cyl_between('steel_dark', (x0 + 0.22, yr + 0.13, hw - 0.25), (x0 + 0.22, yr + 0.45, hw - 0.25), 0.006, 6))
    # interior: seat on a suspension base, dash pod with steering wheel, right console
    out.append(box_span(FRAME, sx - 0.18, sx + 0.18, yf + 0.012, yf + 0.30, -0.18, 0.18))
    out.append(box_span('seat_fabric', sx - 0.24, sx + 0.24, yf + 0.30, yf + 0.42, -0.25, 0.25))
    out.append(box('seat_fabric', 0.10, 0.60, 0.48, (sx - 0.26, yf + 0.72, 0)).rotate('z', 12, (sx - 0.26, yf + 0.42, 0)))
    out.append(box('seat_fabric', 0.08, 0.15, 0.28, (sx - 0.36, yf + 1.10, 0)))
    out.append(box_span('seat_fabric', sx - 0.20, sx + 0.20, yf + 0.55, yf + 0.60, 0.26, 0.34))   # armrest
    dx = x1 - 0.06
    out.append(box_span(FRAME, dx - 0.22, dx, yf + 0.012, yf + 0.70, -0.17, 0.17))
    out.append(box('plastic_black', 0.18, 0.06, 0.32, (dx - 0.12, yf + 0.74, 0)).rotate('z', -25, (dx - 0.12, yf + 0.70, 0)))
    from vehicle_parts import steering_wheel
    out += steering_wheel((dx - 0.40, yf + 0.92, 0), 0.19, wheel_tilt)
    for z, h in ((-0.20, 0.10), (0.14, 0.10), (0.22, 0.10), (0.30, 0.07)):   # clutch, brakes, throttle
        out.append(box('plastic_black', 0.05, h, 0.07, (dx - 0.32, yf + 0.12, z)).rotate('z', -35, (dx - 0.32, yf + 0.05, z)))
        out.append(cyl_between('steel_dark', (dx - 0.30, yf + 0.14, z), (dx - 0.22, yf + 0.30, z), 0.01, 6))
    zc0, zc1 = 0.30, min(hw_low - 0.05, 0.48)
    out.append(box_span(FRAME, sx - 0.30, sx + 0.30, yf + 0.012, yf + 0.48, zc0, zc1))
    out.append(box_span('plastic_black', sx - 0.25, sx + 0.25, yf + 0.48, yf + 0.50, zc0 + 0.02, zc1 - 0.02))
    for i, (lx, h) in enumerate(((sx - 0.12, 0.18), (sx - 0.02, 0.22), (sx + 0.08, 0.20), (sx + 0.18, 0.16))):
        zz = zc0 + 0.06 + 0.04 * (i % 2)
        top = (lx, yf + 0.50 + h, zz)
        out.append(cyl_between('steel_dark', (lx, yf + 0.50, zz), top, 0.008, 6))
        out.append(sphere('plastic_black' if i else 'lamp_red', 0.022, 8, 4).at(top))
    if loader_stick:
        top = (sx + 0.22, yf + 0.66, zc1 - 0.06)
        out.append(cyl_between('steel_dark', (sx + 0.22, yf + 0.50, zc1 - 0.06), top, 0.012, 8))
        out.append(box('plastic_black', 0.06, 0.09, 0.05, add(top, (0, 0.04, 0))))
    return out


# ----------------------------------------------------------- 3-point hitch ---

def three_point_hitch(m, parent, xl, yl, zl, link_len, yb, zb, rock, lift_len, top0, top1,
                      prefix='hitch', mat=FRAME, ball_r=0.045, assist=None):
    """Category II/III three-point hitch.

    Lower links pivot at (xl, yl, +/-zl) and run back to ball ends at
    (xl - link_len, yb, +/-zb): node `<prefix>_tilt`. Lift arms on the rockshaft at
    `rock` = (x, y, z) running back by `lift_len`: node `<prefix>_lift_arm_tilt`. Lift
    rods are rigid links between them. Top link from `top0` to `top1`:
    `<prefix>_top_link_tilt`. `assist`: optional external lift cylinders
    ((x, y, z) barrel pin on `parent`, fraction along the lift arm)."""
    h = m.node(prefix + '_tilt', parent, (xl, yl, 0))
    xb = xl - link_len
    for s in (1, -1):
        a, b = (xl, yl, s * zl), (xb, yb, s * zb)
        m.add(h, box_between(mat, a, b, 0.035, 0.11, (0, 1, 0)))
        m.add(h, cyl_between('steel_dark', (xl, yl, s * zl - 0.04), (xl, yl, s * zl + 0.04), 0.035, 12))
        m.add(h, sphere('steel', ball_r, 12, 6).at(b))
        m.add(h, cyl_between('steel', add(b, (0, 0, -s * 0.02)), add(b, (0, 0, s * 0.08)), 0.014, 8))
        m.add(h, cyl_between('steel', add(b, (0.02, 0.05, s * 0.08)), add(b, (0.02, -0.08, s * 0.08)), 0.004, 6))  # lynch pin
    xr, yr_, zr = rock
    la = m.node(prefix + '_lift_arm_tilt', parent, (xr, yr_, 0))
    m.add(la, cyl_between('steel_dark', (xr, yr_, -zr - 0.06), (xr, yr_, zr + 0.06), 0.05, 16))
    for s in (1, -1):
        e = (xr - lift_len, yr_ + 0.02, s * zr)
        m.add(la, box_between(mat, (xr, yr_, s * zr), e, 0.05, 0.10, (0, 1, 0)))
        frac = 0.62
        lp = (xl - link_len * frac, yl + (yb - yl) * frac + 0.04, s * (zl + (zb - zl) * frac) + s * 0.04)
        side = 'right' if s > 0 else 'left'

        def rod(p0, p1, s=s):
            return [cyl_between('steel', p0, p1, 0.016, 10),
                    cyl_between('steel_dark', add(p0, mul(sub(p1, p0), 0.35)), add(p0, mul(sub(p1, p0), 0.6)), 0.024, 10),
                    cyl_between('steel_dark', add(p0, (0, 0, -0.03)), add(p0, (0, 0, 0.03)), 0.022, 8),
                    cyl_between('steel_dark', add(p1, (0, 0, -0.03)), add(p1, (0, 0, 0.03)), 0.022, 8)]
        m.link('%s_lift_rod_%s_rotate' % (prefix, side), la, h, add(e, (0.04, -0.02, s * 0.045)), lp, rod)
        if assist:
            base, f = assist
            pin = (xr - lift_len * f, yr_ + 0.05, s * zr)
            m.add(la, box_span(mat, pin[0] - 0.04, pin[0] + 0.04, yr_, pin[1] + 0.03, s * (zr - 0.04), s * (zr + 0.04)))
            m.cylinder('%s_assist_cylinder_%s' % (prefix, side), parent, la,
                       (base[0], base[1], s * base[2]), (pin[0], pin[1], s * (zr + 0.07)), 0.05, 0.03, 0.55, barrel=BODY)
    t = m.node(prefix + '_top_link_tilt', parent, top0)
    m.add(t, cyl_between('steel_dark', top0, top1, 0.022, 12))
    mid0, mid1 = add(top0, mul(sub(top1, top0), 0.3)), add(top0, mul(sub(top1, top0), 0.7))
    m.add(t, cyl_between(mat, mid0, mid1, 0.034, 12))
    m.add(t, cyl_between('steel', add(mid0, mul(sub(mid1, mid0), 0.45)), add(add(mid0, mul(sub(mid1, mid0), 0.45)), (0, 0.12, 0)), 0.01, 6))
    for p in (top0, top1):
        m.add(t, cyl_between('steel', add(p, (0, 0, -0.05)), add(p, (0, 0, 0.05)), 0.03, 10))
    return h


def tractor_hood(x0, x1, yb, yt0, yt1, hw, mat=BODY, vents=None):
    """Engine hood from the cab (x0) to the nose (x1): side profile with a rounded
    nose, raised crown, front grille with bars, headlights and indicators, side vents.
    yt0/yt1: hood top at the cab / nose end."""
    out = []
    n = 0.10
    prof = [(x0, yb), (x1 - n, yb), (x1, yb + n), (x1, yt1 - n), (x1 - n, yt1), (x0, yt0)]
    out.append(extrude(mat, prof, 2 * hw))
    out.append(extrude(mat, [(x0, yt0 - 0.02), (x1 - n - 0.02, yt1 - 0.02), (x1 - n - 0.02, yt1 + 0.025), (x0, yt0 + 0.035)],
                       2 * hw - 0.14))
    gy0, gy1 = yb + n, yt1 - n
    gz = hw - 0.08
    out.append(box_span('steel_dark', x1 - 0.005, x1 + 0.005, gy0, gy1 - 0.10, -gz, gz))
    y = gy0 + 0.02
    while y < gy1 - 0.12:
        out.append(box_span(FRAME, x1 + 0.002, x1 + 0.015, y, y + 0.022, -gz + 0.01, gz - 0.01))
        y += 0.05
    for s in (1, -1):
        out.append(box_span(FRAME, x1 - 0.05, x1 + 0.02, gy1 - 0.09, gy1, s * 0.06, s * (hw - 0.04)))
        out.append(box_span('lamp_lens', x1 + 0.02, x1 + 0.026, gy1 - 0.08, gy1 - 0.01, s * 0.08, s * (hw - 0.06)))
        out.append(box_span('lamp_amber', x1 - n - 0.04, x1 - n + 0.02, yt1 - 0.01, yt1 + 0.02, s * (hw - 0.10), s * (hw - 0.03)))
        vx0, vx1 = vents or (x1 - 0.75, x1 - 0.20)
        out.append(box_span('steel_dark', vx0, vx1, yb + 0.08, gy1 - 0.05, s * hw, s * (hw + 0.004)))
        yy = yb + 0.11
        while yy < gy1 - 0.08:
            out.append(box_span(FRAME, vx0 + 0.02, vx1 - 0.02, yy, yy + 0.02, s * (hw + 0.004), s * (hw + 0.012)))
            yy += 0.045
        for xs in (x0 + (x1 - x0) * 0.35, ):
            out.append(box_span(FRAME, xs, xs + 0.005, yb + 0.01, yt0 - 0.02, s * (hw - 0.001), s * (hw + 0.003)))
    return out


def auger_flight(mat, p0, p1, r_in, r_out, pitch, hand=1, steps_per_turn=16, thick=0.006, phase=0.0):
    """Helical auger flighting from p0 to p1 (two-sided strip of `thick`)."""
    from meshkit import _basis_from, from_columns, length
    L = length(sub(p1, p0))
    turns = L / pitch
    n = max(2, int(turns * steps_per_turn))
    pos, nrm, idx = [], [], []
    for side in (1, -1):
        base = len(pos)
        for i in range(n + 1):
            t = i / n
            a = phase + hand * 2 * math.pi * turns * t
            y = L * t + side * thick / 2
            for r in (r_in, r_out):
                pos.append((r * math.cos(a), y, r * math.sin(a)))
            # surface normal of the helicoid at mid radius
            rm = (r_in + r_out) / 2
            tang = (-math.sin(a) * hand * 2 * math.pi * rm, pitch, math.cos(a) * hand * 2 * math.pi * rm)
            radial = (math.cos(a), 0, math.sin(a))
            nn = norm(cross(radial, tang))
            if nn[1] * side < 0:
                nn = mul(nn, -1)
            nrm += [nn, nn]
        for i in range(n):
            k = base + 2 * i
            idx += [(k, k + 2, k + 3), (k, k + 3, k + 1)]
    g = Geo(mat, pos, nrm, idx)
    d, y, z = _basis_from(sub(p1, p0), (0, 0, 1))
    m = from_columns(z, d, y)
    if m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) \
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]) < 0:
        m = from_columns(y, d, z)
    return g.transform(m, p0)
