"""Crawler dozer (tracked bulldozer), medium class, ~20 t operating weight.

Key real-world dimensions used (metres):
  track gauge 1.98, shoe width 0.56, width over tracks 2.54
  track on ground ~2.70, 203 mm chain pitch
  semi-U blade 3.40 wide x 1.30 high
  height to top of cab ~3.25, overall length with blade and ripper ~5.9
"""
import math

from meshkit import (MATERIALS, Model, belt_path, box, box_between, box_span, cyl_between,
                     cylinder, extrude, from_columns, lathe, resample_loop, save, sphere)
from dozer_parts import blade, cab, exhaust, handrail, lamp, louvres, precleaner

# ---- undercarriage geometry
GAUGE = 1.98
ZT = GAUGE / 2
SHOE_W = 0.56
PITCH = 0.203
H_LINK, T_PLATE, H_GROUSER = 0.10, 0.025, 0.055
Y_PATH = T_PLATE + H_GROUSER            # pitch line height on the ground run
IDLER = (1.38, Y_PATH + 0.48, 0.48)     # (x, y, pitch radius)
SPROCKET = (-1.32, Y_PATH + 0.50, 0.50)
ROLLER_R = 0.11
ROLLER_Y = Y_PATH + H_LINK + ROLLER_R
ROLLER_X = [-0.92 + i * (1.94 / 6) for i in range(7)]
CARRIER_X = [-0.38, 0.62]
CARRIER_R = 0.085

BLADE_X = 2.78  # cutting edge


def top_run_y(x):
    (x0, y0, r0), (x1, y1, r1) = SPROCKET, IDLER
    t = (x - x0) / (x1 - x0)
    return (y0 + r0) + ((y1 + r1) - (y0 + r0)) * t


def track_shoe():
    """One shoe in local (u=along chain, n=outward, z=across) space."""
    g = [box('track_steel', PITCH * 0.97, T_PLATE, SHOE_W, (0, T_PLATE / 2, 0)),
         box('track_steel', 0.032, H_GROUSER + 0.005, SHOE_W, (PITCH * 0.22, T_PLATE + H_GROUSER / 2 - 0.0025, 0)),
         box('track_steel', 0.02, H_GROUSER * 0.5 + 0.005, SHOE_W, (-PITCH * 0.3, T_PLATE + H_GROUSER * 0.25 - 0.0025, 0))]
    for s in (1, -1):
        # chain links (rails the rollers run on) and pin bosses
        g.append(box('track_steel', PITCH * 0.98, H_LINK * 0.85, 0.055, (0, -H_LINK * 0.45, s * 0.115)))
        g.append(cylinder('track_steel', 0.032, 0.03, 10).rotate('x', 90).at((PITCH / 2, -H_LINK * 0.5, s * 0.157)))
        for bx in (-0.05, 0.05):  # shoe bolts
            g.append(cylinder('track_steel', 0.013, 0.012, 6).at((bx, T_PLATE + 0.004, s * 0.115)))
    g.append(cylinder('track_steel', 0.036, 0.175, 14).rotate('x', 90).at((PITCH / 2, -H_LINK * 0.5, 0)))
    return g


def track_side(m):
    """Right-hand track assembly (centred on z = +ZT)."""
    part = 'track_right'
    m.node('undercarriage_right')
    loop = belt_path([IDLER, SPROCKET])
    n_guess = round(sum(math.dist(loop[i], loop[i - 1]) for i in range(len(loop))) / PITCH)
    samples, total = resample_loop(loop, n_guess)
    shoe = track_shoe()
    for (px, py), tx, ty in samples:
        u = (tx, ty, 0)
        nrm = (ty, -tx, 0)      # outward for a CCW loop
        M = from_columns(u, nrm, (0, 0, 1))
        m.add(part, [g.transform(M, (px, py, ZT)) for g in shoe])

    # track frame (side profile extruded across)
    frame = [(-0.98, 0.42), (1.02, 0.42), (1.16, 0.52), (1.16, 0.66), (1.02, 0.74),
             (-0.80, 0.74), (-0.98, 0.62)]
    m.add('undercarriage_right', extrude('chassis_black', frame, 0.30).move(z=ZT))
    # roller guards on the outside
    m.add('undercarriage_right', box_span('chassis_black', -1.0, 1.08, 0.24, 0.44, ZT + 0.165, ZT + 0.185))
    for x in ROLLER_X[1:-1:2]:
        m.add('undercarriage_right', box_span('chassis_black', x - 0.05, x + 0.05, 0.24, 0.72, ZT + 0.15, ZT + 0.19))
    # recoil housing ahead of the frame and idler yoke
    m.add('undercarriage_right', cyl_between('chassis_black', (0.25, 0.60, ZT), (0.95, 0.60, ZT), 0.10, 18))
    m.add('undercarriage_right', box_span('chassis_black', 1.05, 1.42, 0.50, 0.64, ZT - 0.19, ZT - 0.16))
    m.add('undercarriage_right', box_span('chassis_black', 1.05, 1.42, 0.50, 0.64, ZT + 0.16, ZT + 0.19))

    # bottom rollers (double flange)
    prof = [(0, -0.2), (0.12, -0.2), (0.135, -0.185), (0.135, -0.16), (0.11, -0.145), (0.11, -0.085),
            (0.13, -0.07), (0.13, 0.07), (0.11, 0.085), (0.11, 0.145), (0.135, 0.16), (0.135, 0.185),
            (0.12, 0.2), (0, 0.2)]
    roller = lathe('track_steel', prof, 28).rotate('x', 90)
    for i, x in enumerate(ROLLER_X):
        n = m.node('track_roller_right_%d_spin' % (i + 1), 'undercarriage_right', (x, ROLLER_Y, ZT))
        m.add(n, roller.at((x, ROLLER_Y, ZT)))
        for s in (1, -1):
            m.add('undercarriage_right', box_span('chassis_black', x - 0.07, x + 0.07, ROLLER_Y, 0.44,
                                                  ZT + s * 0.20, ZT + s * 0.235))
            m.add('undercarriage_right', cylinder('steel_dark', 0.04, 0.03, 10).rotate('x', 90)
                  .at((x, ROLLER_Y, ZT + s * 0.235)))

    # carrier rollers on stub brackets
    for i, x in enumerate(CARRIER_X):
        cy = top_run_y(x) - H_LINK - CARRIER_R
        car = lathe('track_steel', [(0, -0.16), (0.07, -0.16), (0.085, -0.145), (0.085, -0.085), (0.1, -0.07),
                                   (0.1, 0.07), (0.085, 0.085), (0.085, 0.145), (0.07, 0.16), (0, 0.16)], 24)
        n = m.node('carrier_roller_right_%d_spin' % (i + 1), 'undercarriage_right', (x, cy, ZT))
        m.add(n, car.rotate('x', 90).at((x, cy, ZT)))
        m.add('undercarriage_right', box_span('chassis_black', x - 0.05, x + 0.05, 0.74, cy, ZT - 0.24, ZT - 0.18))
        m.add('undercarriage_right', cyl_between('steel_dark', (x, cy, ZT - 0.235), (x, cy, ZT - 0.16), 0.03, 10))

    # front idler with centre flange and spoked web
    ix, iy, ir = IDLER
    idl = lathe('track_steel', [(0, -0.16), (0.34, -0.16), (0.38, -0.145), (0.38, -0.085), (0.42, -0.065),
                               (0.42, 0.065), (0.38, 0.085), (0.38, 0.145), (0.34, 0.16), (0, 0.16)], 48)
    n = m.node('idler_right_spin', 'undercarriage_right', (ix, iy, ZT))
    m.add(n, idl.rotate('x', 90).at((ix, iy, ZT)))
    for s in (1, -1):
        m.add(n, cylinder('steel_dark', 0.12, 0.06, 20).rotate('x', 90).at((ix, iy, ZT + s * 0.196)))
        for k in range(6):
            a = math.radians(k * 60 + 30)
            m.add(n, cylinder('steel_dark', 0.055, 0.005, 16).rotate('x', 90)
                  .at((ix + 0.24 * math.cos(a), iy + 0.24 * math.sin(a), ZT + s * 0.161)))

    # drive sprocket: rim with teeth, hub and final drive housing inboard
    sx, sy, sr = SPROCKET
    sp = []
    sp.append(cylinder('track_steel', 0.36, 0.07, 40).rotate('x', 90))
    n_teeth = 11
    for k in range(n_teeth):
        a = 360 * (k + 0.5) / n_teeth
        tooth = extrude('track_steel', [(-0.045, 0.33), (0.045, 0.33), (0.03, 0.46), (0.0, 0.475), (-0.03, 0.46)], 0.08)
        sp.append(tooth.rotate('z', a))
    for k in range(10):
        a = 2 * math.pi * k / 10
        sp.append(cylinder('steel_dark', 0.018, 0.10, 6).rotate('x', 90).at((0.26 * math.cos(a), 0.26 * math.sin(a), 0)))
    sp.append(cylinder('chassis_black', 0.22, 0.12, 30).rotate('x', 90).at((0, 0, 0.06)))
    n = m.node('sprocket_right_spin', 'undercarriage_right', (sx, sy, ZT))
    m.add(n, [g.at((sx, sy, ZT)) for g in sp])
    fd = lathe('chassis_black', [(0, 0.0), (0.30, 0.0), (0.34, 0.04), (0.34, 0.22), (0.26, 0.30), (0.20, 0.36), (0, 0.36)], 36)
    m.add('undercarriage_right', fd.rotate('x', -90).at((sx, sy, ZT - 0.05)))


def body(m):
    p = 'body'
    # main frame between the tracks
    m.add(p, box_span('chassis_black', -1.70, 1.62, 0.40, 1.06, -0.60, 0.60))
    # belly guard with perforations
    m.add(p, box_span('steel_dark', -0.40, 1.55, 0.36, 0.40, -0.55, 0.55))
    for i in range(6):
        m.add(p, cylinder('steel_dark', 0.045, 0.005, 16).at((-0.15 + i * 0.3, 0.358, 0)))
    # equaliser bar across the front of the track frames
    m.add(p, box_span('chassis_black', 0.48, 0.66, 0.66, 0.78, -ZT - 0.05, ZT + 0.05))
    for s in (1, -1):
        m.add(p, cyl_between('steel', (0.57, 0.72, s * (ZT - 0.18)), (0.57, 0.72, s * (ZT + 0.18)), 0.05, 14))
    # pivot shaft at the rear (track frames pivot about it)
    m.add(p, cyl_between('steel_dark', (-1.32, 0.58, -ZT + 0.2), (-1.32, 0.58, ZT - 0.2), 0.16, 24))

    # engine hood, extruded along x with chamfered top
    hood = [(-0.58, 1.06), (0.58, 1.06), (0.58, 1.86), (0.47, 1.98), (-0.47, 1.98), (-0.58, 1.86)]
    hood_g = extrude('paint_yellow', hood, 2.0).rotate('y', 90).move(x=0.75)
    m.add(p, hood_g)
    m.add(p, box_span('chassis_black', -0.23, 1.74, 1.975, 1.99, -0.30, 0.30))  # top access panel seam
    for s in (1, -1):
        m.add(p, louvres(0.05, 1.25, 1.22, 1.78, s * 0.58, s))
        m.add(p, box_span('chassis_black', 0.62, 0.635, 1.10, 1.84, s * 0.579, s * 0.585))  # door seam
        m.add(p, box_span('steel_dark', 1.30, 1.36, 1.45, 1.55, s * 0.58, s * 0.60))     # latch
    # lifting eyes and radiator cap
    for x in (0.0, 1.5):
        m.add(p, box('paint_yellow', 0.12, 0.10, 0.03, (x, 2.03, 0)))
    m.add(p, cylinder('steel', 0.06, 0.04, 16).at((1.58, 2.0, 0.2)))

    # front radiator guard / grille
    m.add(p, box_span('paint_yellow', 1.75, 1.92, 1.06, 1.98, -0.60, -0.48))
    m.add(p, box_span('paint_yellow', 1.75, 1.92, 1.06, 1.98, 0.48, 0.60))
    m.add(p, box_span('paint_yellow', 1.75, 1.92, 1.86, 1.98, -0.60, 0.60))
    m.add(p, box_span('paint_yellow', 1.75, 1.92, 1.06, 1.16, -0.60, 0.60))
    m.add(p, box_span('steel_dark', 1.76, 1.80, 1.16, 1.86, -0.48, 0.48))
    y = 1.20
    while y < 1.85:
        m.add(p, box('chassis_black', 0.06, 0.025, 0.96, (1.86, y, 0)).rotate('z', -20, (1.86, y, 0)))
        y += 0.065
    # heavy nose casting and lift cylinder brackets
    m.add(p, box_span('paint_yellow', 1.62, 2.02, 0.42, 1.06, -0.62, 0.62))
    m.add(p, box_span('steel_dark', 1.98, 2.06, 0.55, 0.75, -0.08, 0.08))  # tow hook block
    for s in (1, -1):
        for dz in (0.0, 0.16):
            z = s * (0.64 + dz)
            m.add(p, extrude('paint_yellow', [(1.45, 1.30), (1.85, 1.30), (1.85, 1.56), (1.70, 1.72), (1.55, 1.72)], 0.04).move(z=z))
    m.add(p, lamp((1.84, 2.06, 0.42), 1), lamp((1.84, 2.06, -0.42), 1))

    # exhaust and pre-cleaner
    m.add(p, exhaust(1.18, 1.98, 2.62, -0.25))
    m.add(p, precleaner(0.55, 1.98, 0.25))

    # fenders / cab platform over the tracks
    for s in (1, -1):
        m.add(p, box_span('paint_yellow', -1.95, -0.10, 1.12, 1.17, s * 0.58, s * 1.33))
        m.add(p, box_span('paint_yellow', -1.95, -0.10, 1.03, 1.17, s * 1.31, s * 1.33))
        m.add(p, box('paint_yellow', 0.45, 0.04, 0.75, (0.08, 1.04, s * 0.955)).rotate('z', -22, (-0.10, 1.12, 0)))
        # step plate and handrail at the cab
        m.add(p, box_span('steel_dark', -0.85, -0.45, 1.172, 1.18, s * 0.92, s * 1.28))
    m.add(p, handrail([(-0.15, 1.17, 1.28), (-0.15, 1.62, 1.28), (0.35, 1.62, 1.0), (0.35, 1.50, 0.6)], post_to=None))

    # fuel tank behind the cab
    m.add(p, box_span('paint_yellow', -2.02, -1.62, 1.00, 1.62, -0.66, 0.66))
    m.add(p, box_span('paint_yellow', -2.02, -1.62, 1.17, 1.62, -1.30, 1.30))
    m.add(p, cylinder('chassis_black', 0.07, 0.07, 18).at((-1.82, 1.66, 0.85)))
    for s in (1, -1):
        m.add(p, box('lamp_red', 0.02, 0.10, 0.18, (-2.03, 1.45, s * 1.12)))
        m.add(p, lamp((-2.06, 1.45, s * 0.82), -1))
    m.add(p, handrail([(-2.05, 1.30, -0.4), (-2.10, 1.30, -0.4), (-2.10, 1.30, 0.4), (-2.05, 1.30, 0.4)]))

    # cab (ROPS)
    m.add('cab', cab(-1.62, -0.22, 1.17, 3.08, 0.84, door_side=1))
    m.add('cab', box_span('paint_yellow', -0.22, -0.16, 1.17, 1.98, -0.70, 0.70))  # bulkhead to hood


def blade_assembly(m):
    xb, H = BLADE_X, 1.30
    xbk = xb - 0.47
    m.node('push_frame_tilt', None, (0.12, 0.56, 0))          # C-frame pivots on the trunnions
    m.node('blade_tilt', 'push_frame_tilt', (xbk, 0.40, 0))   # side tilt about the blade centre
    m.add('blade_tilt', blade(xb, H, 2.50, ww=0.45, wing_deg=25, curve=0.28, lean=0.12, back=0.45))
    # push arms (C-frame members outside the tracks)
    for s in (1, -1):
        z = s * 1.37
        tr = (0.12, 0.56, z)
        m.add('push_frame_tilt', box_between('paint_yellow', tr, (xbk, 0.32, z), 0.14, 0.22))
        m.add('body', cyl_between('chassis_black', (0.12, 0.56, s * (ZT + 0.15)), (0.12, 0.56, s * 1.45), 0.11, 20))
        m.add('push_frame_tilt', sphere('steel_dark', 0.09).at((0.12, 0.56, s * 1.45)))
        for dz in (-0.1, 0.1):  # blade clevis
            m.add('blade_tilt', box_span('paint_yellow', xbk - 0.12, xbk + 0.02, 0.18, 0.48, z + dz - 0.02, z + dz + 0.02))
    # tilt cylinder (right) and brace (left)
    m.cylinder('blade_tilt_cylinder', 'push_frame_tilt', 'blade_tilt', (0.95, 0.80, 1.37), (xbk + 0.02, 1.18, 1.37),
               0.075, 0.045, extension=0.6)
    m.add('push_frame_tilt', box_between('paint_yellow', (0.95, 0.80, -1.37), (xbk + 0.02, 1.18, -1.37), 0.09, 0.09))
    for s in (1, -1):
        m.add('push_frame_tilt', box_span('paint_yellow', 0.85, 1.05, 0.62, 0.84, s * 1.32, s * 1.42))
        m.add('blade_tilt', box_span('paint_yellow', xbk - 0.14, xbk + 0.02, 1.08, 1.26, s * 1.33, s * 1.41))
    # lift cylinders: radiator guard -> top of blade
    for s in (1, -1):
        z = s * 0.72
        m.cylinder('blade_lift_cylinder_%s' % ('right' if s > 0 else 'left'), 'body', 'blade_tilt',
                   (1.70, 1.58, z), (xbk + 0.02, 0.98, z), 0.085, 0.05, extension=0.62)
        for dz in (-0.09, 0.09):
            m.add('blade_tilt', box_span('paint_yellow', xbk - 0.12, xbk + 0.02, 0.86, 1.10, z + dz - 0.02, z + dz + 0.02))
        # hydraulic lines along the hood
        m.add('body', cyl_between('hose_rubber', (0.4, 1.95, s * 0.5), (1.65, 1.95, s * 0.5), 0.016, 8))
        m.add('body', cyl_between('hose_rubber', (1.65, 1.95, s * 0.5), (1.72, 1.66, z), 0.016, 8))


def ripper(m):
    # mount bracket bolted to the rear of the main frame / final drives
    m.add('ripper_mount', box_span('chassis_black', -2.08, -1.70, 0.36, 0.98, -0.60, 0.60))
    p = m.node('ripper_frame_tilt', None, (-2.02, 0.48, 0))
    xr = -2.62
    for s in (1, -1):
        for z in (s * 0.42, s * 0.52):
            m.add(p, box_between('paint_yellow', (-2.02, 0.90, z), (xr + 0.06, 0.92, z), 0.04, 0.14))  # upper link
            m.add(p, box_between('paint_yellow', (-2.02, 0.48, z), (xr + 0.06, 0.44, z), 0.04, 0.16))  # lower link
        m.cylinder('ripper_cylinder_%s' % ('right' if s > 0 else 'left'), 'ripper_mount', 'ripper_frame_tilt',
                   (-1.98, 1.22, s * 0.62), (xr + 0.08, 1.02, s * 0.62), 0.07, 0.042, extension=0.55)
        m.add('ripper_mount', box_span('paint_yellow', -2.06, -1.92, 0.98, 1.30, s * 0.57, s * 0.67))
    # beam
    m.add(p, box_span('paint_yellow', xr - 0.14, xr + 0.12, 0.40, 1.06, -1.05, 1.05))
    m.add(p, box_span('paint_yellow', xr - 0.17, xr + 0.15, 0.98, 1.06, -1.08, 1.08))
    # shanks with replaceable tips and protectors
    shank = [(0.10, 1.10), (-0.10, 1.10), (-0.10, 0.40), (-0.06, 0.18), (0.05, 0.02), (0.14, 0.05),
             (0.06, 0.22), (0.05, 0.40)]
    for z in (-0.90, 0.0, 0.90):
        m.add(p, extrude('wear_steel', shank, 0.08).move(x=xr - 0.12, z=z))
        m.add(p, extrude('wear_steel', [(0.04, 0.02), (0.18, -0.01), (0.22, 0.02), (0.13, 0.10)], 0.09).move(x=xr - 0.12, z=z))
        m.add(p, extrude('wear_steel', [(0.05, 0.11), (0.14, 0.05), (0.07, 0.23), (0.04, 0.25)], 0.085).move(x=xr - 0.12, z=z))
        m.add(p, cyl_between('steel_dark', (xr - 0.12, 0.80, z - 0.08), (xr - 0.12, 0.80, z + 0.08), 0.03, 10))


def build():
    m = Model('Crawler Dozer', MATERIALS)
    track_side(m)
    m.mirror('right', 'left')
    body(m)
    blade_assembly(m)
    ripper(m)
    return m


if __name__ == '__main__':
    save(build(), 'Construction/crawler-dozer', 'crawler_dozer')
