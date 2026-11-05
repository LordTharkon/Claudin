"""Mine processing props: five separate top-level nodes in one .glb.

  mobile_jaw_crusher       tracked jaw-crushing plant, ~15 x 3.0 x 3.8 m (static tracks)
  conveyor_section         20 m troughed belt conveyor on trestles, 900 mm belt, ~7 deg incline
  stockpile_cone           crushed-rock cone, 9.2 m base, 3.2 m high (~35 deg repose)
  site_office              transportable crib room / site office, 12 x 3 m on skids
  underground_portal       concrete headwall with wing walls, D-shaped 5.0 x 6.0 m opening,
                           steel sets, earth backfill, tunnel stub and ventilation duct with fan

Items are laid out in two rows with clear space (>= 0.5 m) between their bounding boxes.
"""
import math
import random

from mining_parts import (MINING_MATERIALS, Model, beacon, box, box_between, box_span, cyl_between,
                          cylinder, deck, extrude, guardrail, hazard_stripes, hose, ladder, lamp, louvres,
                          polyline_tube, save)
from meshkit import belt_path, resample_loop
from site_parts import mound
from vehicle_parts import arch_polygon

CRUSHER = (13.6, 5.0)        # item centres (x, z)
CONVEYOR = (-4.0, -7.0)
PILE = (11.6, -7.0)
OFFICE = (-0.8, 5.0)
PORTAL = (-13.2, 5.0)


def along_x(mat, poly, x0, x1):
    """Extrude a (z, y) polygon along x from x0 to x1 (poly z maps to world -z)."""
    return extrude(mat, poly, x1 - x0).rotate('y', 90).move(x=(x0 + x1) / 2)


# ------------------------------------------------------------ jaw crusher ---

def jaw_crusher(m):
    n = m.node('mobile_jaw_crusher', None, (CRUSHER[0], 0, CRUSHER[1]))
    cx, cz = CRUSHER
    g = []
    # static tracks: shoes round the idler / sprocket loop, rollers, frames
    loop = belt_path([(-2.60, 0.38, 0.36), (2.10, 0.38, 0.36)], 360)
    pts, _ = resample_loop(loop, 44)
    for s in (1, -1):
        z = s * 1.18
        for (px, py), tx, ty in pts:
            ang = math.degrees(math.atan2(ty, tx))
            g.append(box('track_steel', 0.30, 0.05, 0.50).rotate('z', ang).at((px - ty * 0.02, py + tx * 0.02, z)))
            g.append(box('track_steel', 0.04, 0.035, 0.50).rotate('z', ang).at((px - ty * 0.06, py + tx * 0.06, z)))
        for x in (-2.60, 2.10):
            g.append(cyl_between('track_steel', (x, 0.38, z - 0.20), (x, 0.38, z + 0.20), 0.33, 18))
        for x in (-1.6, -0.6, 0.4, 1.3):
            g.append(cyl_between('track_steel', (x, 0.17, z - 0.18), (x, 0.17, z + 0.18), 0.12, 12))
        g.append(box_span('chassis_black', -2.55, 2.05, 0.26, 0.62, z - 0.16, z + 0.16))
        g.append(cyl_between('chassis_black', (2.10, 0.38, z - s * 0.26), (2.10, 0.38, z - s * 0.40), 0.24, 16))
        g.append(box_span('chassis_black', -1.0, 0.5, 0.62, 0.95, s * 0.70, s * 1.10))
    # main chassis
    g.append(box_span('chassis_black', -7.20, 5.40, 0.95, 1.40, -0.70, 0.70))
    # feed hopper with vibrating grizzly feeder
    g.append(box_span('paint_yellow', -7.20, -3.40, 1.40, 2.05, -0.62, 0.62))
    for x in (-6.90, -4.0):
        g.append(box_span('steel_dark', x - 0.10, x + 0.10, 1.41, 1.70, -0.75, 0.75))
    for s in (1, -1):
        side = [(-7.45, 2.05), (-3.40, 2.05), (-3.20, 3.75), (-7.45, 3.75)]
        flare = ((1, 0, 0), (0, 1, 0), (0, s * 0.53, 1))       # sides lean out with height
        g.append(extrude('paint_yellow', side, 0.05).transform(flare, (0, 0, s * (0.62 - 0.53 * 2.05))))
        g.append(box_span('paint_yellow', -7.45, -3.20, 3.65, 3.78, s * 1.45, s * 1.55))
    rear = [(-0.62, 2.05), (0.62, 2.05), (1.52, 3.75), (-1.52, 3.75)]
    g.append(along_x('paint_yellow', rear, -7.50, -7.45))
    for k in range(10):
        z = -0.50 + k * (1.0 / 9)
        g.append(box_span('wear_steel', -6.80, -4.00, 2.05, 2.10, z - 0.03, z + 0.03))
    # jaw crusher chamber, flywheels, motor and belt guard
    g.append(box_span('paint_yellow', -3.40, -1.20, 1.40, 3.25, -0.80, 0.80))
    g.append(box_span('wear_steel', -3.30, -3.00, 3.25, 3.30, -0.65, 0.65))
    for s in (1, -1):
        g.append(cyl_between('steel_dark', (-1.85, 2.70, s * 0.80), (-1.85, 2.70, s * 0.98), 0.08, 12))
        g.append(cyl_between('steel_dark', (-1.85, 2.70, s * 0.98), (-1.85, 2.70, s * 1.18), 0.70, 28))
        g.append(cyl_between('chassis_black', (-1.85, 2.70, s * 1.18), (-1.85, 2.70, s * 1.20), 0.20, 14))
    g.append(box_span('steel', -1.00, 0.30, 2.30, 3.05, 0.95, 1.25))                 # belt guard
    g.append(cyl_between('paint_gray_dark', (-0.30, 1.55, 0.30), (-0.30, 1.55, 1.05), 0.30, 18))   # electric motor
    # power unit
    g.append(box_span('paint_yellow', 0.20, 3.40, 1.40, 3.10, -1.20, 1.20))
    for s in (1, -1):
        g.append(louvres(0.50, 2.20, 1.80, 2.80, s * 1.20, s, pitch=0.07))
        for x in (1.30, 2.30):
            g.append(box_span('chassis_black', x, x + 0.012, 1.50, 3.00, s * 1.199, s * 1.206))
    g.append(cyl_between('steel_dark', (2.90, 3.10, -0.60), (2.90, 3.70, -0.60), 0.08, 12))
    g.append(box_span('paint_gray_dark', 3.40, 3.70, 1.60, 2.60, 0.40, 1.10))        # control cabinet
    g.append(box_span('chassis_black', 3.70, 3.71, 1.70, 2.50, 0.45, 1.05))
    # walkway round the crusher chamber with guardrail and ladder
    g.append(deck(-3.60, 0.20, 0.82, 1.50, 2.30))
    g.append(guardrail([(-3.55, 1.47), (0.15, 1.47)], 2.30, h=1.0))
    g.append(ladder(-1.80, 0.20, 2.29, 1.60, width=0.45))
    for k in range(3):
        g.append(box_span('paint_yellow', -3.40 + k * 1.70, -3.30 + k * 1.70, 1.40, 2.25, 0.80, 0.85))
    # product conveyor, inclined forward from under the jaw to the discharge
    a0, a1 = (-2.20, 1.10), (7.30, 3.40)
    ang = math.degrees(math.atan2(a1[1] - a0[1], a1[0] - a0[0]))
    L = math.dist(a0, a1)
    conv = [box_span('steel_dark', 0, L, -0.10, 0.10, s * 0.48, s * 0.55) for s in (1, -1)]
    conv.append(box_span('rubber_trim', 0.10, L - 0.10, 0.12, 0.14, -0.42, 0.42))
    for k in range(int(L / 0.9)):
        x = 0.45 + k * 0.9
        conv.append(cyl_between('steel', (x, 0.07, -0.45), (x, 0.07, 0.45), 0.05, 8))
    conv.append(cyl_between('steel_dark', (L, 0.03, -0.46), (L, 0.03, 0.46), 0.15, 16))
    conv.append(cyl_between('steel_dark', (L - 0.40, 0.25, 0.55), (L - 0.40, 0.25, 0.80), 0.16, 14))    # drum motor
    for c in conv:
        g.append(c.rotate('z', ang).move(*a0))
    for xs in (3.40, 5.80):
        ys = a0[1] + (xs - a0[0]) * math.tan(math.radians(ang)) - 0.12
        for s in (1, -1):
            g.append(cyl_between('steel_dark', (xs, ys, s * 0.50), (4.70 if xs < 4 else 5.30, 1.30, s * 0.60), 0.05, 8))
    # lamps, beacon and hazard stripes at both ends
    g.append(lamp((3.45, 3.20, 0.0), 1, 0.7))
    g.append(beacon((2.90, 3.10, 0.60), 0.06))
    g.append(box_span('paint_yellow', 5.40, 5.60, 0.95, 1.40, -0.70, 0.70))
    g.append(hazard_stripes((5.60, 0.97, 0.68), (0, 0, -1), (0, 1, 0), 1.36, 0.41, 0.12))
    g.append(box_span('paint_yellow', -7.40, -7.20, 0.95, 1.40, -0.70, 0.70))
    g.append(hazard_stripes((-7.40, 0.97, -0.68), (0, 0, 1), (0, 1, 0), 1.36, 0.41, 0.12))
    m.add(n, [p.move(cx, 0, cz) for p in _flat(g)])


def _flat(items):
    out = []
    for it in items:
        if isinstance(it, (list, tuple)):
            out += _flat(it)
        else:
            out.append(it)
    return out


# ------------------------------------------------------------- conveyor ---

def conveyor(m):
    n = m.node('conveyor_section', None, (CONVEYOR[0], 0, CONVEYOR[1]))
    cx, cz = CONVEYOR
    L, y0, rise = 20.0, 1.20, 2.40
    ang = math.degrees(math.atan2(rise, L))
    run = L / math.cos(math.radians(ang))
    c = []   # authored horizontal along +x from the tail (x=0, y=0), then tilted
    for s in (1, -1):
        c.append(box_span('paint_gray', 0, run, -0.20, 0.0, s * 0.62, s * 0.68))           # channel stringers
        c.append(box_span('paint_gray', 0, run, -0.20, -0.18, s * 0.55, s * 0.62))
        c.append(box_span('paint_gray', 0, run, -0.02, 0.0, s * 0.55, s * 0.62))
    k = 0
    x = 0.6
    while x < run - 0.4:
        # troughing idler set: centre roll and two 35 deg wing rolls on a frame
        c.append(box_span('paint_gray', x - 0.04, x + 0.04, 0.0, 0.05, -0.62, 0.62))
        c.append(cyl_between('steel', (x, 0.14, -0.19), (x, 0.14, 0.19), 0.055, 10))
        for s in (1, -1):
            a = (x, 0.14, s * 0.22)
            b = (x, 0.14 + 0.36 * math.sin(math.radians(35)), s * (0.22 + 0.36 * math.cos(math.radians(35))))
            c.append(cyl_between('steel', a, b, 0.055, 10))
            c.append(box_between('paint_gray', (x, 0.05, s * 0.55), (x, b[1] + 0.02, b[2] + s * 0.04), 0.06, 0.04))
        if k % 3 == 1:   # return idler under the stringers
            c.append(cyl_between('steel', (x, -0.32, -0.50), (x, -0.32, 0.50), 0.05, 10))
            for s in (1, -1):
                c.append(box_span('paint_gray', x - 0.03, x + 0.03, -0.32, -0.20, s * 0.55, s * 0.58))
        x += 1.2
        k += 1
    # belt: trough on the carry side, flat on the return, round the head and tail pulleys
    t = 0.012
    yb = 0.14 + 0.055
    c.append(box_span('rubber_trim', 0.0, run, yb, yb + t, -0.20, 0.20))
    for s in (1, -1):
        L2 = 0.26
        wing = box_span('rubber_trim', 0.0, run, 0, t, 0, L2)
        wing = wing.rotate('x', -35) if s > 0 else wing.mirror_z().rotate('x', 35)
        c.append(wing.move(0, yb, s * 0.20))
    c.append(box_span('rubber_trim', 0.0, run, -0.39, -0.39 + t, -0.45, 0.45))
    for xp, r in ((0.0, 0.29), (run, 0.29)):
        c.append(cyl_between('steel_dark', (xp, -0.10, -0.48), (xp, -0.10, 0.48), r - 0.012, 18))
        c.append(cyl_between('steel', (xp, -0.10, -0.58), (xp, -0.10, 0.58), 0.05, 10))
    # head end: drive (motor + gearbox), chute and discharge hood
    c.append(box_span('paint_gray_dark', run - 0.25, run + 0.15, -0.30, 0.05, 0.72, 1.05))
    c.append(cyl_between('paint_gray', (run - 0.05, -0.12, 1.05), (run - 0.05, -0.12, 1.55), 0.18, 16))
    c.append(box_span('paint_gray', run - 0.10, run + 0.55, 0.15, 0.65, -0.60, 0.60))
    # walkway with handrail on the +z side
    c.append(deck(0.3, run - 0.3, 0.70, 1.40, -0.30, t=0.04, mat='paint_gray'))
    c += guardrail([(0.3, 1.38), (run - 0.3, 1.38)], -0.30, h=1.0, post_gap=2.5)
    for xx in (2.0, 6.0, 10.0, 14.0, 18.0):
        c.append(box_span('paint_gray', xx - 0.04, xx + 0.04, -0.38, -0.34, 0.62, 1.40))
    g = [p.rotate('z', ang).move(-L / 2, y0, 0) for p in _flat(c)]
    # trestles: twin legs with braces on concrete pads, under the stringers
    for xt in (-8.0, -3.0, 2.0, 7.0):
        top = y0 + (xt + L / 2) * math.tan(math.radians(ang)) - 0.22
        for s in (1, -1):
            g.append(cyl_between('paint_gray', (xt, 0.20, s * 1.00), (xt, top, s * 0.62), 0.06, 10))
            g.append(box_span('concrete', xt - 0.35, xt + 0.35, 0.0, 0.20, s * 0.70, s * 1.30))
            g.append(box_span('steel_dark', xt - 0.12, xt + 0.12, 0.20, 0.22, s * 0.88, s * 1.12))
        g.append(box_between('paint_gray', (xt, 0.55, -0.92), (xt, 0.55, 0.92), 0.06, 0.06, (1, 0, 0)))
        if top > 1.6:
            g.append(box_between('paint_gray', (xt, 0.55, -0.92), (xt, top - 0.05, 0.62), 0.05, 0.05, (1, 0, 0)))
        g.append(box_between('paint_gray', (xt, top, -0.62), (xt, top, 0.62), 0.08, 0.08, (1, 0, 0)))
    m.add(n, [p.move(cx, 0, cz) for p in g])


# ------------------------------------------------------------- stockpile ---

def stockpile(m):
    n = m.node('stockpile_cone', None, (PILE[0], 0, PILE[1]))
    m.add(n, mound('gravel', 4.6, 4.6, 3.2, random.Random(7), rough=0.05, rings=12, segs=36, power=1.25,
                   center=(PILE[0], -0.02, PILE[1])))


# ---------------------------------------------------------------- office ---

def office(m):
    n = m.node('site_office', None, (OFFICE[0], 0, OFFICE[1]))
    cx, cz = OFFICE
    L, W, y0, H = 12.0, 3.0, 0.35, 2.75
    g = []
    for s in (1, -1):   # skids with fork pockets and lifting lugs
        g.append(box_span('steel_dark', -L / 2, L / 2, 0.0, 0.20, s * 1.05, s * 1.25))
        for x in (-1.0, 1.0):
            g.append(box_span('chassis_black', x - 0.18, x + 0.18, 0.04, 0.16, s * 1.25, s * 1.27))
    for x in (-5.5, -2.0, 2.0, 5.5):
        g.append(box_span('steel_dark', x - 0.08, x + 0.08, 0.20, y0, -1.40, 1.40))
    g.append(box_span('paint_white', -L / 2, L / 2, y0, y0 + H, -W / 2, W / 2))
    # roof with a slight fall and gutter
    roof = [(-W / 2 - 0.08, y0 + H), (W / 2 + 0.08, y0 + H), (W / 2 + 0.08, y0 + H + 0.12), (-W / 2 - 0.08, y0 + H + 0.22)]
    g.append(along_x('paint_white', roof, -L / 2 - 0.08, L / 2 + 0.08))
    g.append(box_span('steel', -L / 2 - 0.08, L / 2 + 0.08, y0 + H - 0.05, y0 + H + 0.08, W / 2 + 0.08, W / 2 + 0.16))
    # panel seams on the long walls
    for s in (1, -1):
        for i in range(1, 10):
            x = -L / 2 + i * L / 10
            g.append(box_span('paint_gray', x - 0.012, x + 0.012, y0 + 0.02, y0 + H - 0.02, s * W / 2, s * (W / 2 + 0.008)))
        g.append(box_span('paint_gray', -L / 2, L / 2, y0, y0 + 0.10, s * W / 2, s * (W / 2 + 0.01)))
    # windows (front +z), with frames; door with steps, landing and handrail; lamp
    for x in (-4.6, -2.4, 2.6, 4.8):
        g.append(box_span('aluminum', x - 0.62, x + 0.62, y0 + 1.00, y0 + 2.10, W / 2 + 0.008, W / 2 + 0.03))
        g.append(box_span('glass', x - 0.56, x + 0.56, y0 + 1.06, y0 + 2.04, W / 2 + 0.03, W / 2 + 0.036))
        g.append(box_span('aluminum', x - 0.012, x + 0.012, y0 + 1.06, y0 + 2.04, W / 2 + 0.036, W / 2 + 0.045))
    for x in (-3.5, 3.5):   # rear windows
        g.append(box_span('aluminum', x - 0.62, x + 0.62, y0 + 1.00, y0 + 2.10, -W / 2 - 0.03, -W / 2 - 0.008))
        g.append(box_span('glass', x - 0.56, x + 0.56, y0 + 1.06, y0 + 2.04, -W / 2 - 0.036, -W / 2 - 0.03))
    dx = 0.2
    g.append(box_span('paint_gray_dark', dx - 0.47, dx + 0.47, y0 + 0.10, y0 + 2.15, W / 2 + 0.008, W / 2 + 0.05))
    g.append(box_span('glass', dx - 0.25, dx + 0.25, y0 + 1.35, y0 + 1.95, W / 2 + 0.05, W / 2 + 0.056))
    g.append(box_span('steel', dx + 0.30, dx + 0.38, y0 + 1.00, y0 + 1.04, W / 2 + 0.05, W / 2 + 0.10))
    g.append(deck(dx - 0.90, dx + 0.90, W / 2 + 0.06, W / 2 + 1.30, y0 + 0.08, t=0.05))
    for k, (zs, ys) in enumerate(((W / 2 + 1.30, y0 - 0.12), (W / 2 + 1.58, y0 - 0.30))):
        g.append(box_span('steel_dark', dx - 0.55, dx + 0.55, ys - 0.04, ys, zs, zs + 0.28))
    for s in (1, -1):
        g.append(box_span('steel_dark', dx + s * 0.88 - 0.04, dx + s * 0.88 + 0.04, 0.0, y0 + 0.03, W / 2 + 1.20, W / 2 + 1.28))
    g.append(guardrail([(dx - 0.88, W / 2 + 0.10), (dx - 0.88, W / 2 + 1.25)], y0 + 0.08, h=1.0, kick=0))
    g.append(guardrail([(dx + 0.88, W / 2 + 0.10), (dx + 0.88, W / 2 + 1.25)], y0 + 0.08, h=1.0, kick=0))
    g.append(lamp((dx, y0 + 2.40, W / 2 + 0.10), 1, 0.6, axis='z'))
    # wall-mounted AC units and a meter box
    for x in (-3.5, 3.6):
        g.append(box_span('paint_white', x - 0.45, x + 0.45, y0 + 0.40, y0 + 1.00, -W / 2 - 0.40, -W / 2 - 0.012))
        g.append(box_span('steel_dark', x - 0.38, x + 0.38, y0 + 0.47, y0 + 0.93, -W / 2 - 0.41, -W / 2 - 0.40))
    g.append(box_span('paint_gray', L / 2, L / 2 + 0.15, y0 + 0.80, y0 + 1.50, -0.50, -0.10))
    m.add(n, [p.move(cx, 0, cz) for p in _flat(g)])


# ---------------------------------------------------------------- portal ---

def d_shape(w, hs, n=16, inset=0.0):
    """Opening outline in (z, y): straight walls to hs, semicircular crown."""
    r = w / 2 - inset
    pts = [(r, 0.0), (r, hs)]
    pts += [(r * math.cos(math.pi * i / n), hs + r * math.sin(math.pi * i / n)) for i in range(1, n)]
    pts += [(-r, hs), (-r, 0.0)]
    return pts


def portal(m):
    n = m.node('underground_portal', None, (PORTAL[0], 0, PORTAL[1]))
    cx, cz = PORTAL
    W, H, T = 10.0, 8.0, 0.80
    ow, ohs = 5.0, 3.5
    g = []
    hole = d_shape(ow, ohs)
    u = [(W / 2, 0.0), (W / 2, H), (-W / 2, H), (-W / 2, 0.0)] + list(reversed(hole))
    u = [(-z, y) for z, y in u][::-1]
    g.append(along_x('concrete', u, -T, 0.0))
    g.append(box_span('concrete', 0.0, 0.25, H - 0.40, H, -W / 2, W / 2))              # cope beam
    # wing walls, sloping down away from the face
    for s in (1, -1):
        wing = [(0.0, 0.0), (3.5, 0.0), (3.5, 1.2), (0.0, H - 0.4)]
        g.append(extrude('concrete', wing, 0.50).move(z=s * (W / 2 - 0.25)))
    # earth backfill behind the headwall, with the tunnel bored through it
    eh = 9.0
    e = [(W / 2 + 1.5, 0.0), (W / 2 + 1.5, 4.0), (W / 2 - 1.0, eh), (-W / 2 + 1.0, eh), (-W / 2 - 1.5, 4.0), (-W / 2 - 1.5, 0.0)] \
        + list(reversed(d_shape(ow + 0.6, ohs)))
    e = [(-z, y) for z, y in e][::-1]
    g.append(along_x('dirt', e, -6.0, -T))
    # tunnel lining stub and floor, dark end plug
    g.append(along_x('concrete', [(-z, y) for z, y in arch_polygon(0.0, ohs, ow / 2, ow / 2 + 0.30, 0, 180, 16)], -6.0, -T + 0.01))
    for s in (1, -1):
        g.append(box_span('concrete', -6.0, -T + 0.01, 0.0, ohs, s * ow / 2, s * (ow / 2 + 0.30)))
    g.append(box_span('gravel', -6.0, 3.5, 0.002, 0.08, -ow / 2 + 0.01, ow / 2 - 0.01))
    plug = [(-z, y) for z, y in d_shape(ow, ohs)][::-1]
    g.append(along_x('chassis_black', plug, -5.92, -5.90))
    # steel sets: I-section ribs following the opening, one at the face and two inside
    rib = d_shape(ow, ohs, n=12, inset=0.12)
    for xs in (0.12, -2.0, -4.0):
        for (za, ya), (zb, yb) in zip(rib, rib[1:]):
            g.append(box_between('steel_dark', (xs, ya, za), (xs, yb, zb), 0.20, 0.20, (1, 0, 0)))
        for s in (1, -1):
            g.append(box_span('steel_dark', xs - 0.20, xs + 0.20, 0.08, 0.11, s * (ow / 2 - 0.32), s * (ow / 2 - 0.02)))
    for zz in (-1.5, 0.0, 1.5):   # collar ties between the sets
        yy = ohs + math.sqrt(max(0.0, (ow / 2 - 0.22) ** 2 - zz ** 2)) - 0.12
        g.append(cyl_between('steel', (0.02, yy, zz), (-4.0, yy, zz), 0.03, 6))
    # hazard stripes on the portal jambs, portal lights
    for s in (1, -1):
        g.append(lamp((0.10, ohs + ow / 2 + 0.50, s * 1.50), 1, 0.8))
    # ventilation duct hung at the crown, out of the portal and down to a surface fan
    yd = ohs + ow / 2 - 0.75
    zd = -0.9
    duct = [(-5.8, yd, zd), (0.6, yd, zd), (1.6, yd - 0.4, zd - 0.6), (2.6, 1.6, -3.4), (3.4, 1.0, -3.4)]
    g += polyline_tube('paint_yellow', duct, 0.50, 16)
    for xb in (-5.0, -3.0, -1.0, 0.4):   # hanger straps and wire
        g.append(cyl_between('steel', (xb, yd + 0.50, zd), (xb, yd + 0.80, zd), 0.012, 6))
    g.append(cyl_between('steel', (-5.8, yd + 0.80, zd), (0.4, yd + 0.80, zd), 0.01, 6))
    g.append(cyl_between('paint_gray_dark', (3.40, 1.0, -3.4), (5.20, 1.0, -3.4), 0.55, 24))
    g.append(cyl_between('steel', (5.20, 1.0, -3.4), (5.70, 1.0, -3.4), 0.62, 24, r_top=0.70))
    g.append(cyl_between('steel_dark', (5.70, 1.0, -3.4), (5.72, 1.0, -3.4), 0.66, 24))
    for s in (1, -1):
        g.append(box_span('steel_dark', 3.60, 5.10, 0.0, 0.48, -3.4 + s * 0.30 - 0.05, -3.4 + s * 0.30 + 0.05))
    g.append(box_span('paint_gray_dark', 3.90, 4.40, 0.0, 1.10, -2.40, -2.00))   # starter box
    m.add(n, [p.move(cx, 0, cz) for p in _flat(g)])


def build():
    m = Model('Mine Processing Props', MINING_MATERIALS)
    jaw_crusher(m)
    conveyor(m)
    stockpile(m)
    office(m)
    portal(m)
    return m


def gaps(m):
    """Bounding boxes of the top-level items and the smallest clearance between any two."""
    bb = {}
    for name in m.order:
        if m.nodes[name]['parent'] is None:
            pts = [p for g in m.parts[name] for p in g.pos]
            bb[name] = ([min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)])
    out = []
    names = list(bb)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            (la, ha), (lb, hb) = bb[a], bb[b]
            dx = max(lb[0] - ha[0], la[0] - hb[0])
            dz = max(lb[2] - ha[2], la[2] - hb[2])
            out.append((max(dx, dz), a, b))
    return bb, sorted(out)


if __name__ == '__main__':
    m = build()
    bb, gp = gaps(m)
    for k, v in bb.items():
        print('%-20s x %6.2f..%6.2f  y %5.2f..%5.2f  z %6.2f..%6.2f' % (k, v[0][0], v[1][0], v[0][1], v[1][1], v[0][2], v[1][2]))
    print('closest pair: %.2f m (%s / %s)' % gp[0])
    save(m, 'mine-processing-props')
    print('tris', m.stats()[1])
