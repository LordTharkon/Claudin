"""Wheel dozer (rubber-tyred, articulated bulldozer), ~30 t class.

Key real-world dimensions used (metres):
  wheelbase 3.70, tread 2.40, 29.5R25 tyres (1.87 dia x 0.75 wide)
  width over tyres ~3.15, straight blade 4.00 wide x 1.55 high
  height to top of cab ~3.9, overall length with blade ~7.8
"""
import math

from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, cylinder,
                     extrude, hydraulic_cylinder, lathe, sphere)
from dozer_parts import blade, cab, exhaust, handrail, lamp, louvres, precleaner

WB = 3.70
AX_F, AX_R = WB / 2, -WB / 2
TREAD = 2.40
ZW = TREAD / 2
TYRE_R, TYRE_W = 0.935, 0.75
WHEEL_Y = 0.915          # slightly loaded radius
BLADE_X = 3.78


def wheel():
    """Tyre + rim along +Y (outer face toward +Y), centred on the origin."""
    g = []
    hw = TYRE_W / 2
    base = TYRE_R - 0.06
    prof = [(0.33, -0.30), (0.40, -0.35), (0.55, -hw), (0.70, -hw + 0.008), (0.80, -0.34),
            (base - 0.02, -0.29), (base, -0.22), (base, 0.22), (base - 0.02, 0.29), (0.80, 0.34),
            (0.70, hw - 0.008), (0.55, hw), (0.40, 0.35), (0.33, 0.30), (0.33, -0.30)]
    g.append(lathe('rubber', prof, 64))
    # L-3 style chevron lugs, staggered left/right
    n = 22
    lug = box('rubber', 0.12, 0.34, 0.06)
    for k in range(n):
        for s, off in ((1, 0.0), (-1, 0.5)):
            phi = 360 * (k + off) / n
            # local: x circumferential, y axial, z radial
            l = lug.rotate('z', s * 24).move(y=s * 0.19, z=base + 0.03)
            g.append(l.rotate('y', phi))
            sh = box('rubber', 0.10, 0.08, 0.05).move(y=s * 0.345, z=base - 0.03)
            g.append(sh.rotate('x', -s * 30, (0, s * 0.345, base - 0.03)).rotate('z', s * 24).rotate('y', phi))
    # rim: barrel, flanges, dished disc, hub, wheel nuts
    rim = [(0.29, -0.31), (0.36, -0.33), (0.37, -0.31), (0.33, -0.29), (0.33, 0.29), (0.37, 0.31),
           (0.36, 0.33), (0.29, 0.31), (0.29, -0.31)]
    g.append(lathe('paint', rim, 48))
    g.append(lathe('paint', [(0.0, 0.02), (0.30, 0.02), (0.30, 0.06), (0.0, 0.06)], 40))
    g.append(lathe('paint_dark', [(0, 0.06), (0.21, 0.06), (0.21, 0.16), (0.16, 0.22), (0.0, 0.22)], 32))
    for k in range(18):
        a = 2 * math.pi * k / 18
        g.append(cylinder('steel', 0.017, 0.05, 6).at((0.18 * math.cos(a), 0.085, 0.18 * math.sin(a))))
    for k in range(12):
        a = 2 * math.pi * (k + 0.5) / 12
        g.append(cylinder('grille', 0.04, 0.002, 12).at((0.245 * math.cos(a), 0.061, 0.245 * math.sin(a))))
    return g


def running_gear(m):
    w = wheel()
    for ax, tag in ((AX_F, 'front'), (AX_R, 'rear')):
        for s in (1, -1):
            # rotate so local +Y (outer face) points outward along +-Z
            geo = [g.rotate('x', 90 if s > 0 else -90).at((ax, WHEEL_Y, s * ZW)) for g in w]
            m.add('wheel_%s_%s' % (tag, 'right' if s > 0 else 'left'), geo)
        p = 'axles'
        m.add(p, cyl_between('paint_dark', (ax, WHEEL_Y, -ZW + 0.3), (ax, WHEEL_Y, ZW - 0.3), 0.17, 24))
        m.add(p, sphere('paint_dark', 0.36, 24, 12).scale(1.0, 0.85, 0.9).at((ax, WHEEL_Y, 0)))
        m.add(p, cyl_between('paint_dark', (ax - 0.2, WHEEL_Y, 0), (ax - 0.55 if ax > 0 else ax + 0.55, WHEEL_Y, 0), 0.14, 20))
        for s in (1, -1):  # planetary final drives
            m.add(p, lathe('paint_dark', [(0, 0), (0.30, 0), (0.30, 0.18), (0.24, 0.24), (0, 0.24)], 32)
                  .rotate('x', -90 if s > 0 else 90).at((ax, WHEEL_Y, s * (ZW - 0.42))))


def front_frame(m):
    p = 'front_frame'
    m.add(p, box_span('paint', 0.30, 2.62, 0.78, 1.48, -0.50, 0.50))
    m.add(p, box_span('paint', 2.45, 2.95, 0.55, 1.62, -0.55, 0.55))       # nose
    m.add(p, box_span('steel_dark', 2.93, 3.00, 0.70, 0.90, -0.10, 0.10))  # tow block
    # articulation hitch plates (front half)
    for y in (0.80, 1.36):
        m.add(p, box_span('paint', -0.25, 0.32, y, y + 0.10, -0.38, 0.38))
    for y0, y1 in ((0.76, 0.94), (1.32, 1.50)):
        m.add(p, cylinder('steel', 0.10, y1 - y0, 20).at((0.0, (y0 + y1) / 2, 0)))
    # front fenders (arc around the tyre) with mud flaps
    arc_o, arc_i, n = TYRE_R + 0.12, TYRE_R + 0.08, 16
    pts = [(AX_F + arc_o * math.cos(math.radians(a)), WHEEL_Y + arc_o * math.sin(math.radians(a)))
           for a in (20 + 140 * i / n for i in range(n + 1))]
    pts += [(AX_F + arc_i * math.cos(math.radians(a)), WHEEL_Y + arc_i * math.sin(math.radians(a)))
            for a in (160 - 140 * i / n for i in range(n + 1))]
    for s in (1, -1):
        m.add(p, extrude('paint', pts, 0.86).move(z=s * ZW))
        m.add(p, box_span('paint', AX_F - 0.4, AX_F + 0.4, 1.46, 1.60, s * 0.50, s * (ZW - 0.42)))  # fender bracket
        m.add(p, lamp((AX_F + 0.65, WHEEL_Y + arc_o * 0.92, s * (ZW - 0.2)), 1))
    # steering cylinders across the hitch
    for s in (1, -1):
        m.add(p, hydraulic_cylinder((-0.75, 1.10, s * 0.62), (0.55, 1.10, s * 0.48), 0.07, 0.04, extension=0.55))
        m.add(p, box_span('paint', 0.45, 0.65, 1.0, 1.2, s * 0.44, s * 0.52))


def rear_frame(m):
    p = 'rear_frame'
    m.add(p, box_span('paint', -3.55, -0.32, 0.78, 1.40, -0.56, 0.56))
    for y in (0.92, 1.22):  # hitch clevis (rear half)
        m.add(p, box_span('paint', -0.70, -0.08, y, y + 0.10, -0.42, 0.42))
    m.add(p, box_span('paint', -0.95, -0.70, 0.95, 1.20, -0.66, 0.66))   # steering cylinder anchors
    # under-cab structure
    m.add(p, box_span('paint', -1.62, -0.32, 1.40, 1.96, -0.80, 0.80))
    # walkways / rear fenders over the tyres (non-slip tread plate)
    for s in (1, -1):
        m.add(p, box_span('paint', -2.95, -0.32, 1.92, 1.97, s * 0.78, s * 1.62))
        m.add(p, box_span('grille', -2.90, -0.40, 1.97, 1.975, s * 0.85, s * 1.58))
        m.add(p, box_span('paint', -2.95, -0.32, 1.80, 1.97, s * 1.60, s * 1.62))
        m.add(p, box('paint', 0.60, 0.04, 0.84, (-3.15, 1.78, s * 1.20)).rotate('z', 35, (-2.95, 1.92, 0)))
    # engine enclosure (rear-engine)
    hood = [(-0.74, 1.40), (0.74, 1.40), (0.74, 2.32), (0.60, 2.46), (-0.60, 2.46), (-0.74, 2.32)]
    m.add(p, extrude('paint', hood, 1.90).rotate('y', 90).move(x=-2.55))
    m.add(p, box_span('paint_dark', -3.40, -1.70, 2.455, 2.47, -0.35, 0.35))
    for s in (1, -1):
        m.add(p, louvres(-3.30, -2.00, 1.55, 2.25, s * 0.74, s))
        m.add(p, box_span('paint_dark', -1.92, -1.905, 1.45, 2.30, s * 0.739, s * 0.745))
        m.add(p, box_span('steel_dark', -2.10, -2.04, 1.85, 1.95, s * 0.74, s * 0.76))
    m.add(p, box_span('paint', -1.70, -1.60, 1.40, 2.46, -0.74, 0.74))  # bulkhead behind cab
    # rear radiator grille
    m.add(p, box_span('paint', -3.62, -3.50, 1.40, 2.46, -0.74, 0.74))
    m.add(p, box_span('grille', -3.63, -3.60, 1.50, 2.36, -0.64, 0.64))
    y = 1.55
    while y < 2.34:
        m.add(p, box('paint_dark', 0.05, 0.025, 1.28, (-3.64, y, 0)).rotate('z', 20, (-3.64, y, 0)))
        y += 0.065
    # counterweight / rear bumper with lights and hitch
    cw = [(-3.50, 0.62), (-3.50, 1.42), (-3.82, 1.42), (-3.95, 1.25), (-3.95, 0.80), (-3.80, 0.62)]
    m.add(p, extrude('paint', cw, 2.10))
    m.add(p, box_span('steel_dark', -3.98, -3.90, 0.82, 1.02, -0.18, 0.18))
    m.add(p, cyl_between('steel', (-3.94, 0.80, 0), (-3.94, 1.04, 0), 0.04, 12))
    for s in (1, -1):
        m.add(p, box('lens_red', 0.02, 0.10, 0.20, (-3.955, 1.22, s * 0.78)))
        m.add(p, lamp((-3.70, 2.52, s * 0.55), -1))
    m.add(p, exhaust(-2.20, 2.46, 3.05, -0.35, r=0.075))
    m.add(p, precleaner(-2.75, 2.46, 0.30))
    # access ladder (left side, ahead of rear tyre) and deck handrails
    for z in (-1.20, -1.55):
        m.add(p, box_between('paint', (-0.62, 0.42, z), (-0.45, 1.92, z), 0.04, 0.06, (1, 0, 0)))
    for i in range(5):
        t = (i + 0.5) / 5
        x, y = -0.62 + 0.17 * t, 0.42 + 1.5 * t
        m.add(p, box_span('grille', x - 0.06, x + 0.06, y - 0.015, y + 0.015, -1.54, -1.21))
    for s in (1, -1):
        m.add(p, handrail([(-0.40, 2.95, s * 1.58), (-2.85, 2.95, s * 1.58)], post_to=1.97))
        m.add(p, cyl_between('paint', (-1.60, 2.55, s * 1.58), (-1.60, 1.97, s * 1.58), 0.019, 10))
    m.add(p, handrail([(-0.45, 1.97, -1.58), (-0.45, 2.95, -1.58)]))

    # operator cab
    m.add('cab', cab(-1.60, -0.36, 1.97, 3.72, 0.80, door_side=-1))


def blade_assembly(m):
    xb, H = BLADE_X, 1.55
    xbk = xb - 0.49
    m.add('blade', blade(xb, H, 4.00, curve=0.32, lean=0.14, back=0.47, ribs=7))
    for s in (1, -1):
        z = s * 0.60
        # push arms from the front frame sides to the lower blade
        m.add('push_arms', box_between('paint', (2.15, 0.82, s * 0.62), (xbk, 0.34, s * 0.95), 0.16, 0.24))
        m.add('push_arms', cyl_between('steel_dark', (2.15, 0.82, s * 0.46), (2.15, 0.82, s * 0.74), 0.08, 16))
        for dz in (-0.11, 0.11):
            m.add('blade', box_span('paint', xbk - 0.14, xbk + 0.02, 0.18, 0.52, s * 0.95 + dz - 0.02, s * 0.95 + dz + 0.02))
        # lift cylinders from the nose top to the upper blade
        m.add('lift_cylinders', hydraulic_cylinder((2.70, 1.70, z * 0.7), (xbk + 0.02, 1.18, z * 1.15), 0.09, 0.052,
                                                   extension=0.55))
        m.add('front_frame', box_span('paint', 2.55, 2.85, 1.60, 1.78, s * 0.36, s * 0.48))
        for dz in (-0.1, 0.1):
            m.add('blade', box_span('paint', xbk - 0.12, xbk + 0.02, 1.04, 1.30, z * 1.15 + dz - 0.02, z * 1.15 + dz + 0.02))
    # tilt cylinder (diagonal, right) and fixed brace (left)
    m.add('push_arms', hydraulic_cylinder((2.55, 1.05, 0.52), (xbk + 0.02, 1.38, 1.55), 0.07, 0.042, extension=0.6))
    m.add('push_arms', box_between('paint', (2.55, 1.05, -0.52), (xbk + 0.02, 1.38, -1.55), 0.09, 0.09))
    for s in (1, -1):
        m.add('blade', box_span('paint', xbk - 0.14, xbk + 0.02, 1.28, 1.48, s * 1.50, s * 1.60))
    m.add('front_frame', lamp((2.98, 1.50, 0.35), 1), lamp((2.98, 1.50, -0.35), 1))


def build():
    m = Model('Wheel Dozer', MATERIALS)
    running_gear(m)
    front_frame(m)
    rear_frame(m)
    blade_assembly(m)
    return m


if __name__ == '__main__':
    import os
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'wheel-dozer')
    os.makedirs(out, exist_ok=True)
    model = build()
    model.write_glb(os.path.join(out, 'wheel_dozer.glb'))
    model.write_obj(os.path.join(out, 'wheel_dozer.obj'), 'wheel_dozer.mtl')
    lo, hi = model.bounds()
    print('wheel dozer: L %.2f  W %.2f  H %.2f m  | %d verts, %d tris' %
          (hi[0] - lo[0], hi[2] - lo[2], hi[1] - lo[1], *model.stats()))
