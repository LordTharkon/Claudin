"""Compact track loader (tracked skid steer), modern vertical-lift ~4.5 t class.

Key real-world dimensions used (metres):
  400 mm rubber tracks, width over tracks 1.96, 1.98 m bucket
  ground contact ~1.45, height to top of cab ~2.1, length with bucket on ground ~3.7
"""
from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, cylinder,
                     extrude, hydraulic_cylinder, save)
from dozer_parts import cab, lamp, louvres
from vehicle_parts import loader_bucket, rubber_track, sprocket, track_wheel

ZT = 0.78
TW = 0.40
TH, LUG = 0.05, 0.035
YP = TH + LUG
SPROCKET = (-0.70, YP + 0.50, 0.24)
IDLER_F = (0.72, YP + 0.24, 0.24)
IDLER_R = (-0.60, YP + 0.18, 0.18)
ROLLERS = [-0.33, -0.05, 0.22, 0.48]
ZA = 0.70  # loader arm plane


def undercarriage(m):
    p = 'undercarriage_right'
    m.add('track_right', [g.move(z=ZT) for g in rubber_track([SPROCKET, IDLER_F, IDLER_R], TW, TH, LUG, 0.08, 'block')])
    m.add(p, extrude('paint_dark', [(-0.62, 0.24), (0.72, 0.24), (0.80, 0.33), (0.70, 0.46), (-0.40, 0.56),
                                    (-0.66, 0.46)], 0.24).move(z=ZT))
    m.add(p, track_wheel(0.22, 0.34).at((IDLER_F[0], IDLER_F[1], ZT)))
    m.add(p, track_wheel(0.16, 0.34).at((IDLER_R[0], IDLER_R[1], ZT)))
    for x in ROLLERS:
        m.add(p, track_wheel(0.085, 0.34).at((x, YP + 0.085, ZT)))
    m.add(p, [g.at((SPROCKET[0], SPROCKET[1], ZT)) for g in sprocket(0.23, 0.07, 14)])
    m.add(p, cylinder('paint_dark', 0.19, 0.20, 24).rotate('x', -90).at((SPROCKET[0], SPROCKET[1], ZT - 0.17)))
    m.add(p, box_span('paint_dark', -0.10, 0.10, 0.42, 0.62, ZT - 0.22, ZT - 0.10))  # torsion axle mount
    m.mirrored_copy('track_right', 'track_left')
    m.mirrored_copy(p, 'undercarriage_left')


def body(m):
    p = 'body'
    m.add(p, box_span('paint_dark', -1.38, 0.82, 0.32, 0.95, -0.58, 0.58))
    m.add(p, box_span('counterweight', 0.78, 0.86, 0.32, 0.70, -0.55, 0.55))  # front bumper
    for s in (1, -1):
        # fenders over the tracks
        m.add(p, box_span('paint', -1.42, 0.86, 0.93, 0.97, s * 0.56, s * 0.99))
        m.add(p, box_span('paint', 0.80, 0.86, 0.70, 0.97, s * 0.56, s * 0.99))
        # loader towers
        m.add(p, extrude('paint', [(-1.42, 0.95), (-0.40, 0.95), (-0.55, 1.35), (-1.00, 1.84), (-1.30, 1.84),
                                   (-1.42, 1.60)], 0.12).move(z=s * 0.57))
        m.add(p, louvres(-1.30, -0.75, 1.05, 1.30, s * 0.63, s, 0.045))
        m.add(p, box('lens_red', 0.02, 0.10, 0.07, (-1.43, 1.50, s * 0.57)))
        m.add(p, lamp((-1.48, 1.68, s * 0.57), -1, 0.7))
    # engine compartment and rear door
    m.add(p, box_span('paint', -1.40, -0.42, 0.95, 1.38, -0.51, 0.51))
    m.add(p, box_span('paint', -1.44, -1.40, 0.36, 1.36, -0.50, 0.50))
    m.add(p, box_span('grille', -1.45, -1.43, 0.95, 1.30, -0.40, 0.40))
    y = 1.0
    while y < 1.28:
        m.add(p, box_span('paint_dark', -1.46, -1.44, y, y + 0.018, -0.40, 0.40))
        y += 0.04
    m.add(p, box_span('steel_dark', -1.47, -1.44, 0.75, 0.80, -0.12, 0.12))  # door handle
    m.add(p, box_span('steel_dark', -1.46, -1.40, 0.36, 0.48, -0.20, 0.20))  # tie-down / hitch
    m.add(p, box_span('grille', -1.20, -0.60, 1.38, 1.385, -0.35, 0.35))
    m.add(p, cyl_between('grille', (-1.05, 1.38, 0.32), (-1.05, 1.55, 0.32), 0.035, 12))
    # cab
    m.add('cab', cab(-0.48, 0.58, 0.95, 1.95, 0.46, sill=0.22, door_side=1))
    for z0, z1, y0, y1 in ((-0.40, -0.37, 1.0, 1.85), (0.37, 0.40, 1.0, 1.85), (-0.40, 0.40, 1.0, 1.03),
                           (-0.40, 0.40, 1.82, 1.85), (-0.40, 0.40, 1.38, 1.40)):  # front door frame
        m.add('cab', box_span('steel_dark', 0.585, 0.60, y0, y1, z0, z1))
    m.add('cab', box_span('steel_dark', 0.60, 0.62, 1.25, 1.45, 0.30, 0.34))   # door handle


def loader(m):
    p = 'loader'
    P = (-1.15, 1.72)
    pts = [P, (-0.30, 1.44), (0.62, 1.00), (1.06, 0.76), (1.16, 0.34)]
    for s in (1, -1):
        z = s * ZA
        for a, b in zip(pts, pts[1:]):
            m.add(p, box_between('paint', (a[0], a[1], z), (b[0], b[1], z), 0.10, 0.22))
        for q in pts[1:-1]:
            m.add(p, cyl_between('paint', (q[0], q[1], z - 0.05), (q[0], q[1], z + 0.05), 0.11, 18))
        m.add(p, cyl_between('steel_dark', (P[0], P[1], z - 0.12), (P[0], P[1], z + 0.08), 0.06, 14))
        # vertical-lift link and lift cylinder
        m.add(p, box_between('paint', (-0.80, 1.02, s * 0.66), (-0.55, 1.55, s * 0.66), 0.05, 0.10))
        m.add(p, hydraulic_cylinder((-1.30, 0.98, s * 0.64), (-0.16, 1.40, s * 0.64), 0.06, 0.035, extension=0.55))
        # tilt cylinders to the quick-attach plate
        m.add(p, hydraulic_cylinder((0.70, 1.02, s * 0.42), (1.13, 0.64, s * 0.42), 0.05, 0.03, extension=0.5))
        m.add(p, lamp((0.30, 1.30, s * (ZA + 0.07)), 1, 0.6))
        # grab rail / step on the arm
        m.add(p, box_span('grille', 0.85, 1.05, 0.88, 0.90, s * 0.60, s * 0.66))
    m.add(p, box_between('paint', (0.72, 0.98, -ZA), (0.72, 0.98, ZA), 0.12, 0.12, (1, 0, 0)))
    m.add(p, box_span('paint', 1.10, 1.17, 0.10, 0.66, -0.74, 0.74))  # quick-attach plate
    m.add(p, cyl_between('steel', (1.13, 0.60, -0.55), (1.13, 0.60, -0.40), 0.015, 8))
    m.add(p, cyl_between('steel', (1.13, 0.60, 0.40), (1.13, 0.60, 0.55), 0.015, 8))
    m.add('bucket', [g.at((1.16, 0.02, 0)) for g in loader_bucket(1.98, 0.70, 0.58)])


def build():
    m = Model('Compact Track Loader', MATERIALS)
    undercarriage(m)
    body(m)
    loader(m)
    return m


if __name__ == '__main__':
    save(build(), 'compact-track-loader', 'compact_track_loader')
