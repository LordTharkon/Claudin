"""Compact (mini) excavator, modern zero-tail-swing ~3.5 t class, with cab and dozer blade.

Key real-world dimensions used (metres):
  width over 300 mm rubber tracks 1.75, track length 2.10, tail swing radius 0.87
  boom ~2.0, arm ~1.5, 450 mm bucket, height to top of cab ~2.5
"""
import math

from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, cylinder, extrude, save)
from dozer_parts import blade, cab, lamp, louvres
from vehicle_parts import dig_bucket, hose, rubber_track, sprocket, track_wheel

ZT = 0.725
TRACK_W = 0.30
T_THICK, LUG = 0.045, 0.035
YP = T_THICK + LUG
IDLER = (0.78, YP + 0.26, 0.26)
SPROCKET = (-0.78, YP + 0.27, 0.27)
BOOM_Z = 0.28


def plan(mat, pts, y0, y1):
    """Extrude a top-view (x, z) outline vertically between y0 and y1."""
    return extrude(mat, [(x, -z) for x, z in pts], y1 - y0).rotate('x', -90).move(y=(y0 + y1) / 2)


def undercarriage(m):
    p = m.node('undercarriage_right', 'undercarriage')
    m.node('track_right', 'undercarriage')
    m.add('track_right', [g.move(z=ZT) for g in rubber_track([IDLER, SPROCKET], TRACK_W, T_THICK, LUG, 0.075, 'zigzag')])
    m.add(p, extrude('chassis_black', [(-0.62, 0.22), (0.62, 0.22), (0.70, 0.36), (0.60, 0.50), (-0.60, 0.50),
                                    (-0.68, 0.36)], 0.22).move(z=ZT))
    m.add(m.node('idler_right_spin', p, (IDLER[0], IDLER[1], ZT)), track_wheel(0.24, 0.26).at((IDLER[0], IDLER[1], ZT)))
    m.add(m.node('sprocket_right_spin', p, (SPROCKET[0], SPROCKET[1], ZT)),
          [g.at((SPROCKET[0], SPROCKET[1], ZT)) for g in sprocket(0.25, 0.06, 13)])
    m.add(p, cylinder('chassis_black', 0.20, 0.22, 24).rotate('x', -90).at((SPROCKET[0], SPROCKET[1], ZT - 0.17)))  # travel motor
    for i, x in enumerate((-0.42, -0.14, 0.14, 0.42)):
        m.add(m.node('track_roller_right_%d_spin' % (i + 1), p, (x, YP + 0.07, ZT)), track_wheel(0.07, 0.24).at((x, YP + 0.07, ZT)))
    top = (0.0, IDLER[1] + 0.26 - 0.05, ZT)
    m.add(m.node('carrier_roller_right_spin', p, top), track_wheel(0.05, 0.12, split=False).at(top))
    m.mirror('right', 'left')
    q = 'undercarriage'
    m.add(q, box_span('chassis_black', -0.45, 0.45, 0.26, 0.48, -0.62, 0.62))
    m.add(q, cylinder('chassis_black', 0.45, 0.08, 40).at((0, 0.52, 0)))
    # dozer blade
    m.node('blade_tilt', q, (0.40, 0.38, 0))
    m.add('blade_tilt', blade(1.30, 0.36, 1.69, curve=0.06, lean=0.03, thick=0.02, back=0.13, ribs=4))
    for s in (1, -1):
        m.add('blade_tilt', box_between('paint_yellow', (0.40, 0.38, s * 0.42), (1.18, 0.20, s * 0.52), 0.10, 0.12))
    m.cylinder('blade_cylinder', q, 'blade_tilt', (0.42, 0.44, 0), (1.15, 0.30, 0), 0.05, 0.03, extension=0.55)


def upper(m):
    p = 'upper_rotate'
    yb = 0.56
    arc = [(0.87 * math.cos(math.radians(a)), 0.87 * math.sin(math.radians(a))) for a in range(90, 271, 6)]
    outline = [(0.55, 0.87)] + arc + [(0.55, -0.87), (0.80, -0.62), (0.80, 0.62)]
    m.add(p, plan('paint_yellow', outline, yb + 0.004, 0.95))
    m.add(p, plan('chassis_black', outline, yb, yb + 0.06).scale(1.01, 1, 1.01))
    # counterweight band around the tail
    cw = [(0.89 * math.cos(math.radians(a)), 0.89 * math.sin(math.radians(a))) for a in range(118, 243, 4)]
    cw += [(0.80 * math.cos(math.radians(a)), 0.80 * math.sin(math.radians(a))) for a in range(242, 117, -4)]
    m.add(p, plan('paint_gray_dark', cw, yb + 0.06, 0.92))
    # engine hood over the tail, right-side cover ahead of it
    hood = [(0.86 * math.cos(math.radians(a)), 0.86 * math.sin(math.radians(a))) for a in range(114, 247, 6)]
    hood = [(-0.36, 0.78)] + hood + [(-0.36, -0.78)]
    m.add(p, plan('paint_yellow', hood, 0.95, 1.28))
    m.add(p, plan('chassis_black', [(x * 0.9, z * 0.9) for x, z in hood], 1.28, 1.30))
    m.add(p, box_span('paint_yellow', -0.36, 0.55, 0.95, 1.24, 0.14, 0.82))
    m.add(p, box_span('chassis_black', 0.10, 0.11, 0.97, 1.22, 0.82, 0.825))
    m.add(p, louvres(-0.30, 0.0, 1.0, 1.2, 0.82, 1, 0.04))
    m.add(p, louvres(-0.32, -0.05, 1.0, 1.2, -0.80, -1, 0.04))
    m.add(p, cyl_between('steel_dark', (-0.55, 1.29, 0.45), (-0.55, 1.45, 0.45), 0.03, 12))  # exhaust
    m.add(p, cylinder('steel', 0.05, 0.03, 16).at((0.30, 1.25, 0.60)))                   # fuel cap
    m.add(p, box('lamp_red', 0.02, 0.06, 0.10, (-0.85, 0.85, 0.22)))
    m.add(p, box('lamp_red', 0.02, 0.06, 0.10, (-0.85, 0.85, -0.22)))
    # cab on the left
    m.add('cab', [g.move(z=-0.36) for g in cab(-0.36, 0.62, 0.95, 2.30, 0.47, sill=0.25, door_side=-1)])
    # swing post on the front of the upper
    m.add(p, box_span('paint_yellow', 0.70, 0.92, yb + 0.004, 1.02, BOOM_Z - 0.20, BOOM_Z + 0.20))
    sp = m.node('swing_post_rotate', p, (0.94, 0.80, BOOM_Z))
    m.add(sp, box_span('paint_yellow', 0.90, 1.12, 0.60, 1.00, BOOM_Z - 0.16, BOOM_Z + 0.16))
    for y in (0.645, 0.955):
        m.add(sp, cyl_between('steel', (0.94, y - 0.04, BOOM_Z), (0.94, y + 0.04, BOOM_Z), 0.07, 14))
    m.cylinder('swing_cylinder', p, sp, (0.10, 0.80, 0.62), (0.92, 0.80, BOOM_Z + 0.22), 0.05, 0.03, extension=0.55)


def front_end(m):
    z = BOOM_Z
    S, K, T = (1.04, 0.86), (1.52, 1.96), (2.30, 2.12)
    tail, D = (2.22, 2.42), (2.58, 0.66)
    p = m.node('boom_tilt', 'swing_post_rotate', (*S, z))
    am = m.node('arm_tilt', p, (*T, z))
    bt = m.node('bucket_tilt', am, (*D, z))
    m.add(p, box_between('paint_yellow', (*S, z), (*K, z), 0.16, 0.24, (1, 0, 0)))
    m.add(p, box_between('paint_yellow', (*K, z), (*T, z), 0.16, 0.22))
    m.add(p, cyl_between('paint_yellow', (*K, z - 0.08), (*K, z + 0.08), 0.12, 18))
    for q in (S, T):
        m.add(p, cyl_between('steel_dark', (*q, z - 0.10), (*q, z + 0.10), 0.05, 12))
    m.add(am, box_between('paint_yellow', (*tail, z), (*D, z), 0.13, 0.20, (1, 0, 0)))
    m.cylinder('boom_cylinder', 'swing_post_rotate', p, (1.16, 0.70, z), (1.47, 1.50, z), 0.055, 0.035, extension=0.55)
    m.add(p, box_span('paint_yellow', 1.36, 1.48, 1.40, 1.56, z - 0.08, z + 0.08))
    m.cylinder('arm_cylinder', p, am, (1.40, 2.14, z), (2.20, 2.47, z), 0.05, 0.032, extension=0.6)
    m.add(p, box_span('paint_yellow', 1.32, 1.50, 2.04, 2.20, z - 0.08, z + 0.08))
    # bucket, teeth toward the machine (-x)
    ang, s = -12, 0.62
    bk = [g.rotate('z', ang).rotate('y', 180).at((*D, z)) for g in dig_bucket(0.45, s, 4, ears=0.22)]
    m.add(bt, bk)
    a = math.radians(ang)
    lx, ly = -0.13 * s, 0.08 * s
    link = (D[0] - (lx * math.cos(a) - ly * math.sin(a)), D[1] + lx * math.sin(a) + ly * math.cos(a))
    arm_pt = (2.62, 0.95)
    m.link('bucket_link_rotate', am, bt, (*arm_pt, z), (*link, z),
           lambda a, b: [box_between('steel_dark', (a[0], a[1], z + sg * 0.08), (b[0], b[1], z + sg * 0.08), 0.025, 0.06, (1, 0, 0))
                         for sg in (1, -1)])
    m.cylinder('bucket_cylinder', am, bt, (2.42, 2.12, z), (link[0], link[1] + 0.02, z), 0.045, 0.028, extension=0.55)
    m.add(am, box_span('paint_yellow', 2.38, 2.52, 2.04, 2.20, z - 0.07, z + 0.07))
    for sg in (1, -1):
        m.add(p, hose([(1.10, 1.05, z + sg * 0.09), (1.40, 2.0, z + sg * 0.09), (1.70, 2.22, z + sg * 0.09),
                       (2.20, 2.25, z + sg * 0.085)], 0.011))
        m.add(am, hose([(2.30, 2.28, z + sg * 0.08), (2.45, 1.95, z + sg * 0.075)], 0.011))
    m.add(p, lamp((1.42, 1.80, z - 0.13), 1, 0.7))
    # travel pose: arm curled in, bucket curled
    m.pose(p, 'z', 4)
    m.pose(am, 'z', -32)
    m.pose(bt, 'z', -45)


def build():
    m = Model('Mini Excavator', MATERIALS)
    m.node('undercarriage')
    m.node('upper_rotate', 'undercarriage', (0.0, 0.52, 0.0))   # slewing ring
    m.node('cab', 'upper_rotate')
    undercarriage(m)
    upper(m)
    front_end(m)
    return m


if __name__ == '__main__':
    save(build(), 'Construction/mini-excavator', 'mini_excavator')
