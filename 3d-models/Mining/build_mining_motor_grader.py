"""Mining motor grader (articulated, 24 ft / 7.3 m moldboard).

Key real-world dimensions used (metres):
  front axle to tandem centre 8.70, tandem axle spacing 1.90, tread 3.24
  29.5R29-class tyres (2.00 dia x 0.75 wide)
  moldboard 7.30 x 1.07, circle 1.9 dia; rest pose: moldboard angled 50 deg for travel,
  cutting edge ~0.1 m off the ground
  height to top of cab ~4.5, overall length ~13 (scarifier to rear bumper)
"""
import math

from mining_parts import (MINING_MATERIALS, Model, beacon, blade, box, box_between, box_span, cab,
                          cyl_between, cylinder, deck, exhaust, extrude, guardrail, hazard_stripes,
                          hose, ladder, lamp, lathe, louvres, precleaner, save, sight_gauge, sphere,
                          tank_cap)
from vehicle_parts import arch_polygon, place_wheel, wheel

AX_F = 5.70
TC = -3.00                  # tandem centre (walking-beam pivot)
AX_T = (TC + 0.95, TC - 0.95)
TR, TW, BEAD = 1.00, 0.75, 0.37
WY = 0.98
ZW = 1.62
HINGE_X = 0.45              # frame articulation
CIRCLE = (1.20, 1.05)       # circle centre (x, y)
BALL = (5.00, 1.18)         # drawbar ball joint
EDGE_X = CIRCLE[0] + 0.65   # moldboard cutting edge (straight-ahead design pose)
MB_W, MB_H, MB_Y = 7.30, 1.07, 0.10
ANGLE = 50                  # moldboard angle in the rest pose
DECK_Y = 2.10


def running_gear(m):
    w = wheel(TR, TW, BEAD, 'chevron', 'otr', lugs=20, depth=0.065)
    for s, side in ((1, 'right'), (-1, 'left')):
        st = m.node('wheel_front_%s_steer' % side, 'front_axle_rotate', (AX_F, WY, s * (ZW - 0.42)))
        n = m.node('wheel_front_%s_spin' % side, st, (AX_F, WY, s * ZW))
        m.add(n, place_wheel(w, (AX_F, WY, s * ZW), s))
        m.add(st, cyl_between('chassis_black', (AX_F, WY, s * (ZW - 0.50)), (AX_F, WY, s * (ZW - 0.26)), 0.20, 20))
        m.add(st, box_span('chassis_black', AX_F - 0.12, AX_F + 0.12, WY - 0.15, 1.42, s * (ZW - 0.56), s * (ZW - 0.44)))
        m.add(st, box_span('chassis_black', AX_F - 0.55, AX_F - 0.05, 1.18, 1.30, s * (ZW - 0.56), s * (ZW - 0.46)))
        m.cylinder('steering_cylinder_%s' % side, 'front_axle_rotate', st, (AX_F - 0.50, 1.24, s * 0.30),
                   (AX_F - 0.50, 1.24, s * (ZW - 0.58)), 0.06, 0.035, extension=0.55)
        # tandem walking beam with its two wheels
        t = m.node('tandem_%s_tilt' % side, 'rear_frame', (TC, 1.05, s * 1.08))
        case = arch_polygon(AX_T[0], WY, 0.0, 0.42, -90, 90, 10)[:11] + arch_polygon(AX_T[1], WY, 0.0, 0.42, 90, 270, 10)[:11]
        m.add(t, extrude('paint_yellow', case, 0.26).move(z=s * 1.08))
        m.add(t, cyl_between('steel_dark', (TC, 1.05, s * 0.85), (TC, 1.05, s * 1.25), 0.20, 18))
        for ax, tag in ((AX_T[0], 'front'), (AX_T[1], 'rear')):
            n = m.node('wheel_tandem_%s_%s_spin' % (side, tag), t, (ax, WY, s * ZW))
            m.add(n, place_wheel(w, (ax, WY, s * ZW), s))
            m.add(t, cyl_between('chassis_black', (ax, WY, s * 1.20), (ax, WY, s * (ZW - 0.26)), 0.20, 18))
    # front axle (oscillating) and bolster
    a = 'front_axle_rotate'
    m.add(a, box_span('paint_yellow', AX_F - 0.18, AX_F + 0.18, 1.20, 1.40, -(ZW - 0.58), ZW - 0.58))
    m.add(a, cyl_between('steel_dark', (AX_F - 0.35, 1.35, 0), (AX_F + 0.35, 1.35, 0), 0.10, 14))


def front_frame(m):
    p = 'front_frame_rotate'
    side = [(HINGE_X - 0.15, 1.30), (0.95, 1.30), (1.75, 2.05), (4.70, 2.05), (5.35, 1.45), (6.85, 1.45),
            (6.85, 2.00), (5.40, 2.05), (4.95, 2.68), (1.55, 2.68), (HINGE_X - 0.15, 1.92)]
    m.add(p, extrude('paint_yellow', side, 0.56))
    for y in (1.30, 1.80):   # articulation tongues
        m.add(p, box_span('paint_yellow', HINGE_X - 0.40, HINGE_X, y, y + 0.12, -0.24, 0.24))
    m.add(p, cylinder('steel', 0.09, 0.80, 18).at((HINGE_X, 1.66, 0)))
    m.add(p, box_span('paint_yellow', AX_F - 0.30, AX_F + 0.30, 1.42, 1.48, -0.40, 0.40))
    # push plate with hazard stripes, lights
    m.add(p, box_span('paint_yellow', 6.85, 7.00, 0.95, 2.00, -0.95, 0.95))
    m.add(p, hazard_stripes((7.00, 1.02, 0.90), (0, 0, -1), (0, 1, 0), 1.80, 0.30, 0.14))
    for s in (1, -1):
        m.add(p, lamp((5.10, 2.80, s * 0.20), 1))
        m.add(p, lamp((6.98, 2.08, s * 0.75), 1))
        m.add(p, box('lamp_amber', 0.02, 0.08, 0.14, (7.005, 1.80, s * 0.70)))
        # lift-cylinder saddle brackets
        m.add(p, box_span('paint_yellow', 1.80, 2.20, 2.20, 2.70, s * 0.28, s * 0.40))
        m.add(p, hose([(1.0, 2.70, s * 0.18), (1.90, 2.72, s * 0.18), (2.0, 2.60, s * 0.36)], 0.025))
    # scarifier brackets
    for s in (1, -1):
        m.add(p, box_span('paint_yellow', 6.75, 7.05, 1.10, 2.25, s * 0.40, s * 0.52))


def scarifier(m):
    p = m.node('scarifier_tilt', 'front_frame_rotate', (7.00, 1.20, 0))
    for s in (1, -1):
        m.add(p, box_between('paint_yellow', (7.00, 1.20, s * 0.46), (7.60, 0.95, s * 0.46), 0.10, 0.18))
    m.add(p, cyl_between('steel_dark', (7.00, 1.20, -0.58), (7.00, 1.20, 0.58), 0.05, 12))
    m.add(p, box_span('paint_yellow', 7.50, 7.78, 0.80, 1.12, -0.95, 0.95))
    shank = [(0.06, 0.40), (-0.06, 0.40), (-0.06, -0.18), (0.00, -0.42), (0.10, -0.50), (0.06, -0.20)]
    for i in range(7):
        z = -0.84 + i * 0.28
        m.add(p, extrude('wear_steel', shank, 0.06).move(x=7.64, y=0.85, z=z))
        m.add(p, extrude('steel_dark', [(-0.01, -0.40), (0.10, -0.52), (0.15, -0.48), (0.04, -0.35)], 0.07).move(x=7.64, y=0.85, z=z))
    m.add(p, box_span('paint_yellow', 7.45, 7.60, 1.12, 1.45, -0.10, 0.10))
    m.cylinder('scarifier_lift_cylinder', 'front_frame_rotate', p, (6.95, 2.15, 0), (7.52, 1.35, 0), 0.07, 0.04, extension=0.55)


def drawbar_circle_moldboard(m):
    cx, cy = CIRCLE
    d = m.node('drawbar_tilt', 'front_frame_rotate', (BALL[0], BALL[1], 0))
    m.add(d, sphere('steel_dark', 0.14, 14, 8).at((BALL[0], BALL[1], 0)))
    m.add('front_frame_rotate', box_span('paint_yellow', BALL[0] - 0.15, BALL[0] + 0.15, BALL[1] + 0.05, 1.46, -0.15, 0.15))
    for s in (1, -1):
        m.add(d, box_between('paint_yellow', (BALL[0] - 0.05, BALL[1], s * 0.08), (cx + 0.35, cy + 0.14, s * 0.78), 0.14, 0.22))
        m.add(d, box_between('paint_yellow', (cx + 0.35, cy + 0.14, s * 0.78), (cx - 0.70, cy + 0.14, s * 0.70), 0.14, 0.22))
        m.cylinder('lift_cylinder_%s' % ('right' if s > 0 else 'left'), 'front_frame_rotate', d,
                   (2.00, 2.50, s * 0.48), (1.85, cy + 0.30, s * 0.92), 0.09, 0.055, extension=0.55)
        m.add(d, box_span('paint_yellow', 1.72, 1.98, cy + 0.10, cy + 0.38, s * 0.78, s * 0.98))
    m.add(d, box_span('paint_yellow', cx - 0.70, cx + 0.35, cy + 0.20, cy + 0.30, -0.78, 0.78))  # drawbar top plate
    m.add(d, box_span('chassis_black', cx - 0.95, cx - 0.55, cy + 0.30, cy + 0.65, -0.20, 0.20))  # circle drive
    m.add(d, cylinder('chassis_black', 0.10, 0.25, 14).at((cx - 0.75, cy + 0.78, 0)))
    # circle: ring with internal teeth, drop arms to the moldboard carrier
    c = m.node('circle_rotate', d, (cx, cy, 0))
    m.add(c, lathe('steel_dark', [(0.82, cy - 0.10), (0.97, cy - 0.10), (0.97, cy + 0.08), (0.82, cy + 0.08), (0.82, cy - 0.10)], 48)
          .move(x=cx))
    for k in range(36):
        a = 360 * k / 36
        m.add(c, box('steel_dark', 0.06, 0.12, 0.05, (0.79, cy, 0)).rotate('y', a).move(x=cx))
    pv = (EDGE_X - 0.62, 0.62)   # moldboard pitch pivot (design pose)
    for s in (1, -1):
        arm = [(cx + 0.15, cy + 0.10), (cx + 0.55, cy + 0.10), (pv[0] + 0.18, pv[1] - 0.10), (pv[0] - 0.12, pv[1] - 0.10)]
        m.add(c, extrude('paint_yellow', arm, 0.08).move(z=s * 0.78))
        m.add(c, box_span('paint_yellow', cx - 0.20, cx + 0.15, cy + 0.08, cy + 0.42, s * 0.70, s * 0.86))  # tip-cylinder lug
    m.add(c, cyl_between('steel_dark', (pv[0], pv[1], -0.86), (pv[0], pv[1], 0.86), 0.05, 12))
    # moldboard carrier (pitch) and the moldboard itself (side shift on the rails)
    t = m.node('moldboard_tilt', c, (pv[0], pv[1], 0))
    for s in (1, -1):
        m.add(t, box_span('paint_yellow', EDGE_X - 0.62, EDGE_X - 0.32, 0.40, 1.15, s * 0.58, s * 0.74))
        m.add(t, box_span('paint_yellow', EDGE_X - 0.70, EDGE_X - 0.30, 1.12, 1.24, s * 0.58, s * 0.74))
        m.cylinder('moldboard_tip_cylinder_%s' % ('right' if s > 0 else 'left'), c, t,
                   (cx - 0.05, cy + 0.30, s * 0.92), (EDGE_X - 0.62, 1.20, s * 0.92), 0.06, 0.035, extension=0.55)
        m.add(t, box_span('paint_yellow', EDGE_X - 0.66, EDGE_X - 0.58, 1.10, 1.30, s * 0.74, s * 0.94))
    sl = m.node('moldboard_slide', t, (EDGE_X - 0.40, 0.75, 0))
    mb = blade(EDGE_X, MB_H, MB_W, curve=0.20, lean=-0.06, thick=0.04, back=0.28, ribs=9, k=0.9)
    m.add(sl, [g.move(y=MB_Y) for g in mb])
    for y in (0.62, 1.00):   # slide rails
        m.add(sl, cyl_between('chrome', (EDGE_X - 0.45, y, -MB_W / 2 + 0.25), (EDGE_X - 0.45, y, MB_W / 2 - 0.25), 0.055, 16))
        for z in (-MB_W / 2 + 0.25, MB_W / 2 - 0.25):
            m.add(sl, box_span('paint_yellow', EDGE_X - 0.52, EDGE_X - 0.28, y - 0.08, y + 0.08, z - 0.06, z + 0.06))
    for s in (1, -1):
        m.add(sl, box_span('paint_yellow', EDGE_X - 0.40, EDGE_X - 0.26, 0.70, 0.92, s * 0.35, s * 0.45))
    m.add(sl, box_span('paint_yellow', EDGE_X - 0.40, EDGE_X - 0.26, 0.70, 0.92, 2.55, 2.65))
    m.cylinder('moldboard_side_shift_cylinder', t, sl, (EDGE_X - 0.52, 0.80, -0.40), (EDGE_X - 0.52, 0.80, 2.60),
               0.065, 0.04, extension=0.5)
    m.pose('circle_rotate', 'y', ANGLE)


def rear_frame(m):
    p = 'rear_frame'
    for s in (1, -1):
        m.add(p, box_span('chassis_black', -5.40, HINGE_X + 0.10, 1.12, 1.70, s * 0.36, s * 0.56))
    for y in (1.14, 1.95):    # articulation clevis
        m.add(p, box_span('chassis_black', HINGE_X - 0.05, HINGE_X + 0.30, y, y + 0.12, -0.30, 0.30))
    for x in (-5.0, -3.0, -0.6):
        m.add(p, box_span('chassis_black', x - 0.10, x + 0.10, 1.20, 1.60, -0.36, 0.36))
    # tandem pivot housings
    for s in (1, -1):
        m.add(p, box_span('chassis_black', TC - 0.30, TC + 0.30, 0.90, 1.40, s * 0.56, s * 0.85))
    # engine enclosure, louvres, exhaust, pre-cleaner, rear grille
    hood = [(-0.88, 1.70), (0.88, 1.70), (0.88, 2.95), (0.72, 3.10), (-0.72, 3.10), (-0.88, 2.95)]
    m.add(p, extrude('paint_yellow', hood, 3.40).rotate('y', 90).move(x=-3.55))
    m.add(p, box_span('paint_yellow', -1.85, -1.68, 1.70, 2.10, -0.86, 0.86))
    for s in (1, -1):
        m.add(p, louvres(-4.90, -2.40, 2.20, 2.85, s * 0.88, s))
        m.add(p, box_span('chassis_black', -2.25, -2.235, 1.75, 2.95, s * 0.879, s * 0.885))
    m.add(p, exhaust(-2.70, 3.10, 3.70, -0.30, r=0.08))
    m.add(p, precleaner(-3.40, 3.10, 0.30, 1.0))
    m.add(p, box_span('paint_yellow', -5.40, -5.25, 1.70, 3.10, -0.88, 0.88))
    m.add(p, box_span('steel_dark', -5.42, -5.40, 1.80, 3.00, -0.78, 0.78))
    y = 1.86
    while y < 2.96:
        m.add(p, box('chassis_black', 0.05, 0.025, 1.54, (-5.44, y, 0)).rotate('z', 20, (-5.44, y, 0)))
        y += 0.07
    # rear bumper / counterweight with lights and stripes, fuel tank between the rails
    m.add(p, box_span('paint_yellow', -5.65, -5.40, 0.85, 1.70, -1.05, 1.05))
    m.add(p, box_span('steel_dark', -5.70, -5.65, 1.05, 1.30, -0.15, 0.15))
    for s in (1, -1):
        m.add(p, hazard_stripes((-5.65, 0.90, 0.25 if s > 0 else -1.00), (0, 0, 1), (0, 1, 0), 0.75, 0.32, 0.14))
        m.add(p, box('lamp_red', 0.02, 0.10, 0.18, (-5.655, 1.48, s * 0.80)))
        m.add(p, box('lamp_amber', 0.02, 0.08, 0.14, (-5.655, 1.32, s * 0.80)))
        m.add(p, lamp((-5.45, 3.18, s * 0.60), -1))
    m.add(p, box_span('paint_yellow', -4.85, -4.10, 0.72, 1.60, -0.34, 0.34))
    m.add(p, tank_cap(-4.30, 1.60, 0.0))
    # tandem fenders / walkways with guardrails, ladders, hydraulic tank
    for s in (1, -1):
        m.add(p, deck(-4.45, -0.85, s * 0.88, s * 2.02, DECK_Y))
        m.add(p, box_span('paint_yellow', -4.45, -0.85, DECK_Y - 0.22, DECK_Y - 0.05, s * 1.98, s * 2.02))
        for x in (-4.2, -1.1):
            m.add(p, box_span('chassis_black', x - 0.08, x + 0.08, 1.60, DECK_Y - 0.05, s * 0.56, s * 0.90))
        m.add(p, guardrail([(-0.90, s * 1.99), (-4.40, s * 1.99), (-4.40, s * 0.95)], DECK_Y, h=1.0))
        m.add(p, ladder(-0.70, 0.45, DECK_Y - 0.012, s * 1.62, 0.50, axis='z'))
        m.add(p, deck(-0.85, -0.55, s * 1.30, s * 1.95, DECK_Y))
    m.add(p, cylinder('paint_yellow', 0.30, 0.85, 20).at((-3.80, DECK_Y + 0.425, 1.45)))
    m.add(p, tank_cap(-3.80, DECK_Y + 0.85, 1.45, 0.07))
    m.add(p, hose([(-3.80, DECK_Y + 0.15, 1.15), (-3.0, DECK_Y + 0.05, 0.92), (-2.3, 1.80, 0.70)], 0.03))
    # cab
    m.add('cab', box_span('paint_yellow', -1.68, 0.30, 1.70, DECK_Y, -0.86, 0.86))
    m.add('cab', cab(-1.68, 0.30, DECK_Y, 4.00, 0.95, sill=0.40, door_side=-1, post=0.11, wheel=True))


def build():
    m = Model('Mining Motor Grader', MINING_MATERIALS)
    m.node('rear_frame')
    m.node('cab', 'rear_frame')
    m.node('front_frame_rotate', 'rear_frame', (HINGE_X, 1.66, 0))      # articulation (frame_rotate)
    m.node('front_axle_rotate', 'front_frame_rotate', (AX_F, 1.35, 0))   # front axle oscillation
    running_gear(m)
    front_frame(m)
    scarifier(m)
    drawbar_circle_moldboard(m)
    rear_frame(m)
    for s in (1, -1):
        m.cylinder('articulation_cylinder_%s' % ('right' if s > 0 else 'left'), 'rear_frame', 'front_frame_rotate',
                   (-0.70, 1.45, s * 0.66), (1.30, 1.45, s * 0.36), 0.07, 0.04, extension=0.55)
    return m


if __name__ == '__main__':
    save(build(), 'mining-motor-grader')
