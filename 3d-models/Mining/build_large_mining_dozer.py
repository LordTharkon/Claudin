"""Large mining dozer (tracked bulldozer), ~100 t operating-weight class.

Key real-world dimensions used (metres):
  track gauge 2.80, single-grouser extreme-service shoes 0.71 wide, width over tracks 3.51
  317 mm chain pitch, 8 bottom rollers, track on ground ~4.6
  U-blade 6.30 wide x 2.65 high on push arms, dual tilt cylinders
  height to top of ROPS cab ~4.7 (roof unit and beacons), overall length with blade and
  single-shank ripper ~10
"""
import math

from mining_parts import (MINING_MATERIALS, Model, beacon, blade, box, box_between, box_span, cab,
                          cyl_between, cylinder, deck, exhaust, extrude, from_columns, guardrail,
                          hazard_stripes, hose, lamp, lathe, louvres, precleaner, save, sight_gauge,
                          sphere, stairs, tank_cap)
from meshkit import belt_path, resample_loop

# ---- undercarriage
GAUGE = 2.80
ZT = GAUGE / 2
SHOE_W = 0.71
PITCH = 0.317
H_LINK, T_PLATE, H_GROUSER = 0.20, 0.05, 0.09
Y_PATH = T_PLATE + H_GROUSER
IDLER = (2.30, Y_PATH + 0.62, 0.62)
SPROCKET = (-2.30, Y_PATH + 0.74, 0.74)
ROLLER_R = 0.19
ROLLER_Y = Y_PATH + H_LINK + ROLLER_R
ROLLER_X = [-1.78 + i * (3.56 / 7) for i in range(8)]
CARRIER_X = [-0.65, 1.20]
CARRIER_R = 0.15
LINK_Z = 0.205          # chain link centre offset from the track centre line

DECK_Y = 1.85           # fender / walkway deck height
BLADE_X = 4.62          # cutting edge
BLADE_H = 2.65
XBK = BLADE_X - 0.88    # back of the blade (push-arm and cylinder clevises)
ARM_Z = 2.00            # push arms run outside the tracks


def top_run_y(x):
    (x0, y0, r0), (x1, y1, r1) = SPROCKET, IDLER
    t = (x - x0) / (x1 - x0)
    return (y0 + r0) + ((y1 + r1) - (y0 + r0)) * t


def track_shoe():
    """One shoe in local (u=along chain, n=outward, z=across) space."""
    g = [box('track_steel', PITCH * 0.96, T_PLATE, SHOE_W, (0, T_PLATE / 2, 0)),
         box('track_steel', 0.06, H_GROUSER + 0.005, SHOE_W, (PITCH * 0.18, T_PLATE + H_GROUSER / 2 - 0.0025, 0)),
         cylinder('track_steel', 0.055, 0.50, 8).rotate('x', 90).at((PITCH / 2, -H_LINK * 0.5, 0))]
    for s in (1, -1):
        g.append(box('track_steel', PITCH * 0.98, H_LINK * 0.85, 0.09, (0, -H_LINK * 0.45, s * LINK_Z)))
        for bx in (-0.08, 0.08):
            g.append(box('track_steel', 0.04, 0.02, 0.04, (bx - PITCH * 0.1, T_PLATE + 0.01, s * LINK_Z)))
    return g


def track_side(m):
    part = 'track_right'
    u = 'undercarriage_right'
    m.node(u, 'body')
    m.node(part, u)
    loop = belt_path([IDLER, SPROCKET])
    n_guess = round(sum(math.dist(loop[i], loop[i - 1]) for i in range(len(loop))) / PITCH)
    samples, _ = resample_loop(loop, n_guess)
    shoe = track_shoe()
    for (px, py), tx, ty in samples:
        M = from_columns((tx, ty, 0), (ty, -tx, 0), (0, 0, 1))
        m.add(part, [g.transform(M, (px, py, ZT)) for g in shoe])

    # track frame and roller guards
    frame = [(-1.95, 0.70), (1.95, 0.70), (2.05, 0.80), (2.05, 1.02), (1.90, 1.12), (-1.60, 1.12),
             (-1.95, 1.00)]
    m.add(u, extrude('chassis_black', frame, 0.44).move(z=ZT))
    m.add(u, box_span('chassis_black', -2.0, 2.0, 0.40, 0.70, ZT + 0.24, ZT + 0.27))
    for x in ROLLER_X[::2]:
        m.add(u, box_span('chassis_black', x + 0.20, x + 0.36, 0.40, 1.08, ZT + 0.22, ZT + 0.27))
    m.add(u, cyl_between('chassis_black', (0.35, 0.95, ZT), (1.75, 0.95, ZT), 0.16, 18))  # recoil housing
    for s in (1, -1):
        m.add(u, box_span('chassis_black', 1.85, 2.42, 0.62, 0.90, ZT + s * 0.25, ZT + s * 0.30))  # idler yoke
    # bottom rollers (double flange)
    k = 1.7
    prof = [(0, -0.2), (0.12, -0.2), (0.135, -0.185), (0.135, -0.16), (0.11, -0.145), (0.11, -0.085),
            (0.13, -0.07), (0.13, 0.07), (0.11, 0.085), (0.11, 0.145), (0.135, 0.16), (0.135, 0.185),
            (0.12, 0.2), (0, 0.2)]
    roller = lathe('track_steel', [(r * k, y * k) for r, y in prof], 20).rotate('x', 90)
    for i, x in enumerate(ROLLER_X):
        n = m.node('track_roller_right_%d_spin' % (i + 1), u, (x, ROLLER_Y, ZT))
        m.add(n, roller.at((x, ROLLER_Y, ZT)))
        for s in (1, -1):
            m.add(u, box_span('chassis_black', x - 0.11, x + 0.11, ROLLER_Y, 0.72, ZT + s * 0.345, ZT + s * 0.39))
            m.add(u, cylinder('steel_dark', 0.065, 0.04, 10).rotate('x', 90).at((x, ROLLER_Y, ZT + s * 0.41)))
    # carrier rollers
    for i, x in enumerate(CARRIER_X):
        cy = top_run_y(x) - H_LINK - CARRIER_R
        cp = [(0, -0.27), (0.12, -0.27), (0.145, -0.245), (0.145, -0.15), (0.17, -0.12), (0.17, 0.12),
              (0.145, 0.15), (0.145, 0.245), (0.12, 0.27), (0, 0.27)]
        car = lathe('track_steel', [(r * CARRIER_R / 0.17, y) for r, y in cp], 18)
        n = m.node('carrier_roller_right_%d_spin' % (i + 1), u, (x, cy, ZT))
        m.add(n, car.rotate('x', 90).at((x, cy, ZT)))
        m.add(u, box_span('chassis_black', x - 0.08, x + 0.08, 1.10, cy, ZT - 0.40, ZT - 0.30))
        m.add(u, cyl_between('steel_dark', (x, cy, ZT - 0.42), (x, cy, ZT - 0.22), 0.05, 10))
    # front idler
    ix, iy, ir = IDLER
    ri = ir - H_LINK * 0.95
    idl = lathe('track_steel', [(0, -0.26), (ri - 0.06, -0.26), (ri, -0.24), (ri, -0.155), (ri + 0.06, -0.12),
                               (ri + 0.06, 0.12), (ri, 0.155), (ri, 0.24), (ri - 0.06, 0.26), (0, 0.26)], 40)
    n = m.node('idler_right_spin', u, (ix, iy, ZT))
    m.add(n, idl.rotate('x', 90).at((ix, iy, ZT)))
    for s in (1, -1):
        m.add(n, cylinder('steel_dark', 0.17, 0.08, 20).rotate('x', 90).at((ix, iy, ZT + s * 0.30)))
        for kk in range(6):
            a = math.radians(kk * 60 + 30)
            m.add(n, cylinder('steel_dark', 0.08, 0.006, 12).rotate('x', 90)
                  .at((ix + 0.30 * math.cos(a), iy + 0.30 * math.sin(a), ZT + s * 0.263)))
    # drive sprocket: bolted rim segments, hub, final drive inboard
    sx, sy, sr = SPROCKET
    sp = [cylinder('track_steel', sr - 0.17, 0.10, 40).rotate('x', 90)]
    n_teeth = 14
    for kk in range(n_teeth):
        a = 360 * (kk + 0.5) / n_teeth
        tooth = extrude('track_steel', [(-0.07, sr - 0.20), (0.07, sr - 0.20), (0.045, sr - 0.02), (0.0, sr + 0.0), (-0.045, sr - 0.02)], 0.12)
        sp.append(tooth.rotate('z', a))
    for kk in range(14):
        a = 2 * math.pi * kk / 14
        sp.append(cylinder('steel_dark', 0.026, 0.14, 6).rotate('x', 90).at((0.44 * math.cos(a), 0.44 * math.sin(a), 0)))
    for kk in range(7):  # segment joints
        a = 2 * math.pi * (kk + 0.25) / 7
        sp.append(box('steel_dark', 0.03, 0.12, 0.13).rotate('z', math.degrees(a) - 90).at(((sr - 0.26) * math.cos(a), (sr - 0.26) * math.sin(a), 0)))
    sp.append(cylinder('chassis_black', 0.32, 0.16, 28).rotate('x', 90).at((0, 0, 0.09)))
    n = m.node('sprocket_right_spin', u, (sx, sy, ZT))
    m.add(n, [g.at((sx, sy, ZT)) for g in sp])
    fd = lathe('chassis_black', [(0, 0.0), (0.46, 0.0), (0.52, 0.06), (0.52, 0.30), (0.40, 0.42), (0.30, 0.50), (0, 0.50)], 32)
    m.add(u, fd.rotate('x', -90).at((sx, sy, ZT - 0.17)))


def body(m):
    p = 'body'
    # main frame, belly guard, equaliser bar, pivot shaft
    m.add(p, box_span('chassis_black', -2.95, 2.80, 0.55, 1.40, -0.90, 0.90))
    m.add(p, box_span('steel_dark', -0.90, 2.70, 0.48, 0.55, -0.82, 0.82))
    for i in range(6):
        m.add(p, cylinder('steel_dark', 0.07, 0.006, 16).at((-0.50 + i * 0.5, 0.477, 0)))
    m.add(p, box_span('chassis_black', 0.60, 0.86, 1.02, 1.20, -ZT - 0.08, ZT + 0.08))
    for s in (1, -1):
        m.add(p, cyl_between('steel', (0.73, 1.11, s * (ZT - 0.27)), (0.73, 1.11, s * (ZT + 0.27)), 0.075, 14))
    m.add(p, cyl_between('steel_dark', (-1.85, 0.85, -ZT + 0.30), (-1.85, 0.85, ZT - 0.30), 0.22, 24))

    # engine hood with chamfered top, side doors and louvres
    hood = [(-0.95, 1.40), (0.95, 1.40), (0.95, 2.90), (0.80, 3.05), (-0.80, 3.05), (-0.95, 2.90)]
    m.add(p, extrude('paint_yellow', hood, 3.30).rotate('y', 90).move(x=1.40))
    m.add(p, box_span('chassis_black', -0.10, 2.90, 3.045, 3.06, -0.45, 0.45))
    for s in (1, -1):
        m.add(p, louvres(0.25, 2.35, DECK_Y + 0.25, 2.75, s * 0.95, s))
        for x in (0.20, 1.30, 2.45):
            m.add(p, box_span('chassis_black', x, x + 0.015, DECK_Y + 0.05, 2.88, s * 0.949, s * 0.956))
        m.add(p, box_span('steel_dark', 2.30, 2.38, 2.30, 2.42, s * 0.95, s * 0.975))
    for x in (0.3, 2.5):
        m.add(p, box('paint_yellow', 0.16, 0.12, 0.04, (x, 3.11, 0)))
    m.add(p, tank_cap(2.55, 3.05, 0.35, 0.08))

    # radiator guard (front) with grille and cylinder brackets
    gx0, gx1 = 3.00, 3.30
    m.add(p, box_span('paint_yellow', gx0, gx1, 1.40, 3.05, -1.05, -0.88))
    m.add(p, box_span('paint_yellow', gx0, gx1, 1.40, 3.05, 0.88, 1.05))
    m.add(p, box_span('paint_yellow', gx0, gx1, 2.90, 3.05, -0.88, 0.88))
    m.add(p, box_span('paint_yellow', gx0, gx1, 1.40, 1.55, -0.88, 0.88))
    m.add(p, box_span('steel_dark', gx0 + 0.01, gx0 + 0.06, 1.55, 2.90, -0.88, 0.88))
    y = 1.62
    while y < 2.86:
        m.add(p, box('chassis_black', 0.09, 0.03, 1.74, (gx1 - 0.08, y, 0)).rotate('z', -20, (gx1 - 0.08, y, 0)))
        y += 0.085
    m.add(p, box_span('paint_yellow', 2.81, 3.45, 0.56, 1.40, -0.95, 0.95))  # nose casting
    m.add(p, box_span('steel_dark', 3.43, 3.53, 0.75, 1.05, -0.14, 0.14))   # tow hook block
    m.add(p, cyl_between('steel', (3.48, 0.78, 0), (3.48, 1.02, 0), 0.05, 10))
    for s in (1, -1):
        for dz in (0.0, 0.26):
            z = s * (1.07 + dz)
            m.add(p, extrude('paint_yellow', [(2.55, 2.05), (3.20, 2.05), (3.20, 2.45), (2.95, 2.70), (2.70, 2.70)], 0.06).move(z=z))
        m.add(p, box_span('paint_yellow', 2.60, 3.20, 2.05, 2.15, s * 0.95, s * 1.40))
    m.add(p, lamp((3.20, 3.17, 0.70), 1), lamp((3.20, 3.17, -0.70), 1))
    m.add(p, exhaust(1.55, 3.05, 3.95, -0.45, r=0.11))
    m.add(p, precleaner(0.55, 3.05, 0.45, 1.2))
    m.add(p, precleaner(0.55, 3.05, -0.20, 1.2))

    # fender decks over the tracks, front slopes, walkways with guardrails
    for s in (1, -1):
        m.add(p, deck(-3.20, 2.60, s * 0.92, s * 1.90, DECK_Y))
        m.add(p, box_span('paint_yellow', -3.20, 2.60, DECK_Y - 0.30, DECK_Y - 0.05, s * 1.86, s * 1.90))  # fender skirt
        m.add(p, box('paint_yellow', 0.75, 0.05, 0.98, (2.93, DECK_Y - 0.16, s * 1.41)).rotate('z', -26, (2.60, DECK_Y - 0.03, 0)))
        m.add(p, box_span('paint_yellow', -3.20, -3.15, DECK_Y - 0.30, DECK_Y - 0.05, s * 0.92, s * 1.90))
        m.add(p, box_span('chassis_black', -2.32, -2.12, 1.30, DECK_Y - 0.05, s * 0.88, s * 1.10))  # deck supports
        m.add(p, box_span('chassis_black', 1.5, 1.7, 1.30, DECK_Y - 0.05, s * 0.88, s * 1.10))
        m.add(p, lamp((2.62, DECK_Y + 0.12, s * 1.70), 1))
    # left side: open at the rear for the stair; right side full length
    m.add(p, guardrail([(-3.15, 1.86), (2.55, 1.86)], DECK_Y))
    m.add(p, guardrail([(-2.70, -1.86), (2.55, -1.86)], DECK_Y))
    m.add(p, guardrail([(2.55, -1.86), (2.55, -1.00)], DECK_Y))
    m.add(p, guardrail([(2.55, 1.86), (2.55, 1.00)], DECK_Y))
    m.add(p, guardrail([(-3.15, 1.86), (-3.15, 1.15)], DECK_Y))

    # rear stair (left), up onto the fender deck behind the cab
    m.add(p, stairs(-3.90, 0.55, -3.22, DECK_Y, -1.52, width=0.66, rise=0.26, rails=(1, -1)))
    m.add(p, box_span('paint_yellow', -3.30, -3.15, 0.90, DECK_Y - 0.05, -1.88, -1.82))
    m.add(p, cyl_between('paint_yellow', (-3.90, 0.45, -1.52), (-3.32, 1.1, -1.52), 0.04, 8))

    # fuel tank behind the cab, with fill cap, sight gauge, lights and hazard stripes
    m.add(p, box_span('paint_yellow', -3.05, -2.40, 1.40, 2.95, -1.12, 1.12))
    m.add(p, box_span('paint_yellow', -3.08, -2.37, 2.95, 3.00, -1.15, 1.15))
    m.add(p, tank_cap(-2.70, 3.00, 0.75))
    for s in (1, -1):
        m.add(p, box('lamp_red', 0.02, 0.12, 0.22, (-3.06, 2.55, s * 0.95)))
        m.add(p, box('lamp_amber', 0.02, 0.08, 0.16, (-3.06, 2.35, s * 0.95)))
        m.add(p, lamp((-3.12, 2.78, s * 0.50), -1))
        m.add(p, hazard_stripes((-3.20, DECK_Y - 0.295, 0.95 if s > 0 else -1.87), (0, 0, 1), (0, 1, 0), 0.92, 0.24, 0.13))
    # hydraulic tank on the right fender beside the cab
    m.add(p, box_span('paint_yellow', -2.25, -1.05, DECK_Y, 2.85, 1.10, 1.78))
    m.add(p, tank_cap(-1.30, 2.85, 1.45, 0.08))
    m.add(p, sight_gauge(-1.65, 2.05, 2.70, 1.78, 1))
    m.add(p, cyl_between('steel', (-1.05, 2.0, 1.35), (-0.60, 2.0, 1.35), 0.04, 10))
    # hydraulic lines along the hood to the lift cylinders
    for s in (1, -1):
        m.add(p, hose([(0.2, 3.02, s * 0.75), (2.45, 3.02, s * 0.75), (2.70, 2.75, s * 1.0)], 0.024))
        m.add(p, hose([(0.2, 3.08, s * 0.68), (2.40, 3.08, s * 0.68), (2.68, 2.80, s * 0.95)], 0.024))

    # cab
    m.add('cab', box_span('paint_yellow', -2.35, -0.25, 1.40, DECK_Y, -0.90, 0.90))
    m.add('cab', cab(-2.35, -0.35, DECK_Y, 4.10, 1.00, sill=0.50, door_side=-1, post=0.13))
    m.add('cab', box_span('paint_yellow', -0.35, -0.25, DECK_Y, 3.05, -0.95, 0.95))  # bulkhead to hood


def blade_assembly(m):
    m.node('push_frame_tilt', 'body', (0.30, 0.90, 0))
    m.node('blade_tilt', 'push_frame_tilt', (XBK, 0.80, 0))
    m.add('blade_tilt', blade(BLADE_X, BLADE_H, 3.60, ww=1.35, wing_deg=21, curve=0.52, lean=0.24,
                              thick=0.08, back=0.88, ribs=7, k=1.8))
    for s in (1, -1):
        z = s * ARM_Z
        # push arm: tapered box from the trunnion to the lower blade, trunnion ball
        m.add('push_frame_tilt', box_between('paint_yellow', (0.30, 0.90, z), (XBK - 0.05, 0.62, z), 0.24, 0.40))
        m.add('push_frame_tilt', box_between('paint_yellow', (0.80, 1.06, z), (XBK - 0.25, 0.86, z), 0.20, 0.10))
        m.add('push_frame_tilt', sphere('steel_dark', 0.17, 16, 8).at((0.30, 0.90, z)))
        m.add('body', cyl_between('chassis_black', (0.30, 0.90, s * (ZT + 0.22)), (0.30, 0.90, s * (ARM_Z - 0.12)), 0.17, 20))
        for dz in (-0.17, 0.17):
            m.add('blade_tilt', box_span('paint_yellow', XBK - 0.22, XBK + 0.02, 0.35, 0.95, z + dz - 0.04, z + dz + 0.04))
        # tilt cylinders (dual tilt) from the push arms to the top of the blade
        m.cylinder('blade_tilt_cylinder_%s' % ('right' if s > 0 else 'left'), 'push_frame_tilt', 'blade_tilt',
                   (2.00, 1.20, z), (XBK + 0.02, 2.15, s * 1.85), 0.11, 0.065, extension=0.58)
        m.add('push_frame_tilt', box_span('paint_yellow', 1.86, 2.14, 0.84, 1.28, z - 0.13, z + 0.13))
        m.add('blade_tilt', box_span('paint_yellow', XBK - 0.24, XBK + 0.02, 2.02, 2.30, s * 1.85 - 0.14, s * 1.85 + 0.14))
        # lift cylinders: radiator-guard brackets to the upper blade
        zl = s * 1.20
        m.cylinder('blade_lift_cylinder_%s' % ('right' if s > 0 else 'left'), 'body', 'blade_tilt',
                   (2.95, 2.55, zl), (XBK + 0.02, 1.72, s * 1.30), 0.13, 0.08, extension=0.60)
        for dz in (-0.15, 0.15):
            m.add('blade_tilt', box_span('paint_yellow', XBK - 0.22, XBK + 0.02, 1.55, 1.90, s * 1.30 + dz - 0.035, s * 1.30 + dz + 0.035))
    # blade-top lights and centre brace ball
    m.add('blade_tilt', box_span('paint_yellow', XBK - 0.08, XBK + 0.02, 0.50, 2.20, -0.40, 0.40))


def ripper(m):
    p = 'body'
    m.add(p, box_span('chassis_black', -3.45, -3.06, 0.55, 2.05, -0.85, 0.85))  # ripper mount
    for s in (1, -1):
        m.add(p, box_span('paint_yellow', -3.40, -3.05, 1.40, 1.75, s * 0.85, s * 0.98))
    r = m.node('ripper_tilt', 'body', (-3.40, 0.78, 0))
    xb = -4.45  # shank pocket / beam
    for z in (-0.62, -0.48, 0.48, 0.62):
        zz = z * 0.55
        m.add(r, box_between('paint_yellow', (-3.40, 0.78, z), (xb + 0.12, 0.95, zz), 0.07, 0.28))
    m.add(r, cyl_between('steel_dark', (-3.40, 0.78, -0.70), (-3.40, 0.78, 0.70), 0.07, 14))
    # beam / shank holder
    m.add(r, box_span('paint_yellow', xb - 0.24, xb + 0.24, 0.80, 2.05, -0.36, -0.08))
    m.add(r, box_span('paint_yellow', xb - 0.24, xb + 0.24, 0.80, 2.05, 0.08, 0.36))
    m.add(r, box_span('paint_yellow', xb - 0.26, xb + 0.26, 1.98, 2.10, -0.40, 0.40))
    m.add(r, box_span('paint_yellow', xb + 0.10, xb + 0.24, 0.80, 1.98, -0.08, 0.08))
    for s in (1, -1):
        m.add(r, box_span('paint_yellow', xb - 0.15, xb + 0.22, 1.10, 1.45, s * 0.36, s * 0.85))  # lift-cylinder ears
    # single shank with tip, shank protector and pin puller
    shank = [(0.08, 2.55), (-0.18, 2.55), (-0.18, 0.95), (-0.08, 0.55), (0.10, 0.25), (0.32, 0.20),
             (0.20, 0.45), (0.08, 0.80)]
    m.add(r, extrude('wear_steel', shank, 0.15).move(x=xb))
    m.add(r, extrude('wear_steel', [(0.10, 0.22), (0.44, 0.12), (0.50, 0.17), (0.30, 0.32)], 0.15).move(x=xb))
    m.add(r, extrude('wear_steel', [(0.14, 0.32), (0.31, 0.22), (0.20, 0.50), (0.13, 0.55)], 0.145).move(x=xb))
    for y in (1.30, 1.75):
        m.add(r, cyl_between('steel_dark', (xb, y, -0.46), (xb, y, 0.46), 0.05, 10))
    m.add(r, cyl_between('steel_dark', (xb + 0.05, 2.10, 0.0), (xb + 0.05, 2.35, 0.0), 0.035, 8))
    m.add(r, lamp((xb + 0.32, 1.98, 0.30), 1, 0.9))
    for s in (1, -1):
        m.cylinder('ripper_lift_cylinder_%s' % ('right' if s > 0 else 'left'), p, r,
                   (-3.05, 1.62, s * 0.92), (xb + 0.05, 1.30, s * 0.92), 0.12, 0.075, extension=0.55)
        m.cylinder('ripper_pitch_cylinder_%s' % ('right' if s > 0 else 'left'), p, r,
                   (-3.42, 1.95, s * 0.30), (xb + 0.20, 1.85, s * 0.30), 0.10, 0.06, extension=0.55)
        m.add(r, box_span('paint_yellow', xb + 0.10, xb + 0.30, 1.72, 1.98, s * 0.20, s * 0.40))
    m.add(r, hazard_stripes((xb - 0.24, 1.55, 0.08), (0, 0, 1), (0, 1, 0), 0.28, 0.40, 0.12))
    m.add(r, hazard_stripes((xb - 0.24, 1.55, -0.36), (0, 0, 1), (0, 1, 0), 0.28, 0.40, 0.12))


def build():
    m = Model('Large Mining Dozer', MINING_MATERIALS)
    m.node('body')
    m.node('cab', 'body')
    track_side(m)
    m.mirror('right', 'left')
    body(m)
    blade_assembly(m)
    ripper(m)
    return m


if __name__ == '__main__':
    save(build(), 'large-mining-dozer')
