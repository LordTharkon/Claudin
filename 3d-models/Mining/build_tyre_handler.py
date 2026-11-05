"""Tyre handler: articulated wheel-loader carrier with a tyre manipulator on the lift arms.

Key real-world dimensions used (metres):
  wheelbase 3.40 (hitch centred), tread 2.30, 23.5R25-class tyres (1.60 dia x 0.60 wide)
  arm pins 2.30 high, manipulator pitch pin at the arm tips, roll bearing ahead of it
  clamp pads 1.30 x 0.60 with rubber facings and turning rollers, open to 2.8 between
  the rollers (3.5 over the pads); cab floor 1.75, roof 3.45, ~4.0 over the roof unit
  overall ~9.2 x 3.5 x 4.0
"""
import math

from mining_parts import (MINING_MATERIALS, Model, box, box_span, cab, cyl_between, cylinder, deck,
                          exhaust, extrude, guardrail, hazard_stripes, hose, ladder, lamp, louvres,
                          precleaner, save, sight_gauge, sphere, tank_cap)
from vehicle_parts import arch_polygon, place_wheel, wheel

AX_F, AX_R = 1.70, -1.70
TR, TW, BEAD = 0.80, 0.60, 0.32
WY = TR
ZW = 1.15
HITCH_Y = 0.95
BP = (1.30, 2.30)        # arm pins on the loader tower
AT = (3.90, 1.55)        # manipulator pitch pin at the arm tips
ARM_Z = 0.70
ROLL_X = 4.30            # roll bearing face
GRIP_Y = 1.70            # roll axis height
PAD_Z = 1.62             # pad carriage centre (open)
DECK_Y = 1.75


def arm_y(x):
    return BP[1] + (AT[1] - BP[1]) * (x - BP[0]) / (AT[0] - BP[0])


def running_gear(m):
    w = wheel(TR, TW, BEAD, 'chevron', 'otr', lugs=20, depth=0.06)
    for ax, tag, par in ((AX_F, 'front', 'hitch_rotate'), (AX_R, 'rear', 'rear_axle_rotate')):
        for s, side in ((1, 'right'), (-1, 'left')):
            n = m.node('wheel_%s_%s_spin' % (tag, side), par, (ax, WY, s * ZW))
            m.add(n, place_wheel(w, (ax, WY, s * ZW), s))
            m.add(par, cyl_between('chassis_black', (ax, WY, s * (ZW - 0.52)), (ax, WY, s * (ZW - 0.34)), 0.26, 24))
        m.add(par, cyl_between('chassis_black', (ax, WY, -ZW + 0.50), (ax, WY, ZW - 0.50), 0.14, 18))
        m.add(par, sphere('chassis_black', 0.30, 18, 9).scale(1, 0.85, 0.9).at((ax, WY, 0)))
    m.add('rear_axle_rotate', box_span('chassis_black', AX_R - 0.30, AX_R + 0.30, WY + 0.12, WY + 0.24, -0.30, 0.30))


def front_frame(m):
    p = 'hitch_rotate'
    m.add(p, box_span('chassis_black', 0.40, 2.60, 0.75, 1.55, -0.45, 0.45))
    for y in (0.80, 1.28):
        m.add(p, box_span('chassis_black', -0.20, 0.41, y, y + 0.10, -0.30, 0.30))
    for y0, y1 in ((0.74, 0.96), (1.22, 1.44)):
        m.add(p, cylinder('steel', 0.09, y1 - y0, 18).at((0.0, (y0 + y1) / 2, 0)))
    for s in (1, -1):
        tower = [(0.55, 1.55), (1.95, 1.55), (1.70, 2.20), (1.50, 2.55), (1.10, 2.55), (0.75, 2.10)]
        m.add(p, extrude('paint_yellow', tower, 0.12).move(z=s * 0.56))
        m.add(p, cyl_between('steel_dark', (BP[0], BP[1], s * 0.48), (BP[0], BP[1], s * (ARM_Z + 0.13)), 0.08, 14))
        m.add(p, box_span('paint_yellow', 0.85, 1.15, 0.85, 1.20, s * 0.45, s * 0.60))       # lift-cylinder lugs
        m.add(p, extrude('paint_yellow', arch_polygon(AX_F, WY, TR + 0.07, TR + 0.11, 25, 160, 16), TW + 0.10).move(z=s * ZW))
        m.add(p, lamp((AX_F + 0.75, 1.82, s * (ZW + 0.05)), 1, 0.8))
        m.add(p, box('lamp_amber', 0.02, 0.06, 0.12, (2.76, 1.38, s * 0.30)))
    m.add(p, box_span('paint_yellow', 2.60, 2.75, 0.85, 1.45, -0.45, 0.45))
    for s in (1, -1):
        m.add(p, hazard_stripes((2.75, 0.87, 0.43 if s > 0 else -0.03), (0, 0, -1), (0, 1, 0), 0.40, 0.56, 0.10))
    for s in (1, -1):
        m.add(p, hose([(0.50, 1.60, s * 0.30), (1.10, 1.70, s * 0.30), (1.40, 2.10, s * 0.42)], 0.025))


def rear_frame(m):
    p = 'rear_frame'
    m.add(p, box_span('chassis_black', -3.75, -0.40, 0.80, 1.55, -0.50, 0.50))
    for y in (0.62, 1.06, 1.50):
        m.add(p, box_span('chassis_black', -0.55, 0.20, y, y + 0.10, -0.40, 0.40))
    # engine enclosure
    hood = [(-0.78, 1.55), (0.78, 1.55), (0.78, 2.45), (0.62, 2.62), (-0.62, 2.62), (-0.78, 2.45)]
    m.add(p, extrude('paint_yellow', hood, 2.70).rotate('y', 90).move(x=-2.40))
    for s in (1, -1):
        m.add(p, louvres(-3.55, -1.70, 1.95, 2.40, s * 0.78, s, pitch=0.06))
    m.add(p, exhaust(-2.20, 2.62, 3.25, 0.30, r=0.08))
    m.add(p, precleaner(-1.40, 2.62, -0.35, 0.9))
    m.add(p, tank_cap(-3.30, 2.62, 0.0, 0.07))
    # radiator grille and counterweight
    m.add(p, box_span('steel_dark', -3.77, -3.75, 1.65, 2.50, -0.70, 0.70))
    y = 1.70
    while y < 2.46:
        m.add(p, box('chassis_black', 0.04, 0.02, 1.36, (-3.79, y, 0)).rotate('z', 20, (-3.79, y, 0)))
        y += 0.065
    cw = [(-3.75, 0.55), (-3.75, 1.62), (-4.05, 1.62), (-4.22, 1.45), (-4.22, 0.75), (-4.02, 0.55)]
    m.add(p, extrude('paint_yellow', cw, 2.20))
    m.add(p, cyl_between('steel', (-4.24, 0.70, 0), (-4.24, 0.95, 0), 0.05, 10))
    for s in (1, -1):
        m.add(p, hazard_stripes((-4.22, 0.78, 0.25 if s > 0 else -1.05), (0, 0, 1), (0, 1, 0), 0.80, 0.42, 0.12))
        m.add(p, box('lamp_red', 0.02, 0.10, 0.18, (-4.23, 1.36, s * 0.85)))
        m.add(p, box('lamp_amber', 0.02, 0.08, 0.14, (-4.23, 1.36, s * 0.62)))
        m.add(p, lamp((-3.85, 2.62, s * 0.62), -1, 0.8))
    # rear fenders, side decks, tanks, access ladder (left) and guardrails
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', arch_polygon(AX_R, WY, TR + 0.07, TR + 0.11, 20, 160, 16), TW + 0.10).move(z=s * ZW))
    m.add(p, deck(-1.20, 0.30, -1.50, -0.80, DECK_Y))
    m.add(p, deck(-1.20, 0.30, 0.80, 1.50, DECK_Y))
    m.add(p, box_span('chassis_black', -0.60, -0.40, 1.55, DECK_Y - 0.05, -0.80, 0.80))
    m.add(p, box_span('paint_yellow', -1.15, -0.65, 1.00, DECK_Y - 0.05, 0.60, 1.20))        # hydraulic tank
    m.add(p, sight_gauge(-0.90, 1.15, 1.55, 1.20, 1))
    m.add(p, box_span('paint_yellow', -1.15, -0.65, 1.00, DECK_Y - 0.05, -1.20, -0.60))      # fuel tank
    m.add(p, tank_cap(-0.90, DECK_Y, 1.30, 0.06))
    m.add(p, ladder(-0.40, 0.45, DECK_Y - 0.01, -1.45, width=0.45, axis='x'))
    m.add(p, guardrail([(-1.15, 1.47), (0.27, 1.47), (0.27, 0.83)], DECK_Y))
    m.add(p, guardrail([(-1.15, -1.47), (-0.62, -1.47)], DECK_Y))
    m.add(p, guardrail([(0.27, -1.47), (0.27, -0.83)], DECK_Y))
    m.add('cab', cab(-1.55, 0.15, DECK_Y, 3.45, 0.90, sill=0.50, door_side=-1, post=0.10, wheel=True))


def arms(m):
    a = 'arm_tilt'
    dx, dy = AT[0] - BP[0], AT[1] - BP[1]
    L = math.hypot(dx, dy)
    nx, ny = -dy / L, dx / L
    for s in (1, -1):
        z = s * ARM_Z
        prof = [(BP[0] - 0.20 * dx / L + 0.24 * nx, BP[1] - 0.20 * dy / L + 0.24 * ny),
                (AT[0] + 0.18 * dx / L + 0.18 * nx, AT[1] + 0.18 * dy / L + 0.18 * ny),
                (AT[0] + 0.18 * dx / L - 0.18 * nx, AT[1] + 0.18 * dy / L - 0.18 * ny),
                (2.60 - 0.30 * nx, arm_y(2.60) - 0.30 * ny),
                (BP[0] - 0.20 * dx / L - 0.24 * nx, BP[1] - 0.20 * dy / L - 0.24 * ny)]
        m.add(a, extrude('paint_yellow', prof, 0.14).move(z=z))
        for c, r in ((BP, 0.22), (AT, 0.18)):
            m.add(a, cyl_between('paint_yellow', (c[0], c[1], z - 0.10), (c[0], c[1], z + 0.10), r, 20))
            m.add(a, cyl_between('steel_dark', (c[0], c[1], z - 0.15), (c[0], c[1], z + 0.15), r * 0.45, 12))
        m.add(a, box_span('paint_yellow', 2.45, 2.75, arm_y(2.6) - 0.50, arm_y(2.6) - 0.20, s * 0.56, s * 0.63))
        m.cylinder('arm_lift_cylinder_%s' % ('right' if s > 0 else 'left'), 'hitch_rotate', a,
                   (1.00, 1.02, s * 0.53), (2.60, arm_y(2.6) - 0.40, s * 0.53), 0.13, 0.08, extension=0.6)
    xc = 2.70
    m.add(a, cyl_between('paint_yellow', (xc, arm_y(xc), -ARM_Z), (xc, arm_y(xc), ARM_Z), 0.15, 20))
    for dz in (-0.12, 0.12):
        m.add(a, extrude('paint_yellow', [(xc - 0.20, arm_y(xc)), (xc + 0.15, arm_y(xc)), (xc + 0.05, arm_y(xc) + 0.45),
                                          (xc - 0.15, arm_y(xc) + 0.45)], 0.05).move(z=dz))
    m.add(a, lamp((xc, arm_y(xc) + 0.30, 0.40), 1, 0.6), lamp((xc, arm_y(xc) + 0.30, -0.40), 1, 0.6))
    for s in (1, -1):
        m.add(a, hose([(BP[0] + 0.2, BP[1] + 0.20, s * (ARM_Z - 0.10)), (AT[0] - 0.3, AT[1] + 0.22, s * (ARM_Z - 0.10)),
                       (AT[0] + 0.05, AT[1] + 0.40, s * 0.30)], 0.025))


def manipulator(m):
    t, r = 'gripper_tilt', 'gripper_rotate'
    # pitch carrier: housing on the arm-tip pins carrying the roll bearing
    for s in (1, -1):
        m.add(t, extrude('paint_yellow', [(AT[0] - 0.20, AT[1] - 0.15), (AT[0] + 0.25, AT[1] - 0.30), (ROLL_X - 0.02, GRIP_Y - 0.45),
                                          (ROLL_X - 0.02, GRIP_Y + 0.55), (AT[0] + 0.05, AT[1] + 0.75), (AT[0] - 0.15, AT[1] + 0.20)], 0.06)
              .move(z=s * (ARM_Z - 0.13)))
        m.add(t, extrude('paint_yellow', [(AT[0] - 0.15, AT[1] - 0.10), (AT[0] + 0.15, AT[1] - 0.20), (AT[0] + 0.15, AT[1] + 0.20),
                                          (AT[0] - 0.12, AT[1] + 0.12)], 0.05).move(z=s * (ARM_Z + 0.13)))
    m.add(t, cyl_between('steel_dark', (AT[0], AT[1], -ARM_Z - 0.17), (AT[0], AT[1], ARM_Z + 0.17), 0.07, 12))
    m.add(t, box_span('paint_yellow', AT[0] + 0.15, ROLL_X - 0.02, GRIP_Y - 0.45, GRIP_Y + 0.55, -ARM_Z + 0.16, ARM_Z - 0.16))
    m.add(t, cyl_between('steel_dark', (ROLL_X - 0.02, GRIP_Y, 0), (ROLL_X, GRIP_Y, 0), 0.50, 32))
    m.add(t, cyl_between('chassis_black', (AT[0] + 0.40, GRIP_Y + 0.55, 0.20), (AT[0] + 0.40, GRIP_Y + 0.75, 0.20), 0.10, 14))   # roll motor
    for dz in (-0.10, 0.10):   # pitch-cylinder ear
        m.add(t, extrude('paint_yellow', [(AT[0] + 0.00, AT[1] + 0.60), (AT[0] + 0.20, AT[1] + 0.55), (AT[0] + 0.20, AT[1] + 0.95),
                                          (AT[0] + 0.05, AT[1] + 0.95)], 0.04).move(z=dz))
    m.cylinder('gripper_tilt_cylinder', 'arm_tilt', t, (2.72, arm_y(2.72) + 0.40, 0), (AT[0] + 0.12, AT[1] + 0.85, 0), 0.10, 0.065, extension=0.6)
    # roll: rotating plate and the clamp beam with guides
    m.add(r, cyl_between('paint_yellow', (ROLL_X, GRIP_Y, 0), (ROLL_X + 0.10, GRIP_Y, 0), 0.50, 32))
    m.add(r, box_span('paint_yellow', ROLL_X + 0.10, ROLL_X + 0.36, GRIP_Y - 0.18, GRIP_Y + 0.14, -1.72, 1.72))
    for s in (1, -1):
        m.add(r, box_span('chrome', ROLL_X + 0.14, ROLL_X + 0.32, GRIP_Y - 0.24, GRIP_Y - 0.18, s * 0.55, s * 1.70))
    m.add(r, box_span('paint_yellow', ROLL_X + 0.10, ROLL_X + 0.30, GRIP_Y + 0.14, GRIP_Y + 0.22, -0.25, 0.25))
    for s in (1, -1):
        m.add(r, hose([(ROLL_X + 0.20, GRIP_Y + 0.22, s * 0.15), (ROLL_X + 0.20, GRIP_Y + 0.40, s * 0.30),
                       (ROLL_X + 0.20, GRIP_Y + 0.34, s * 0.60)], 0.02))
    # clamp pads on carriages
    for s, side in ((1, 'right'), (-1, 'left')):
        zc = s * PAD_Z
        n = m.node('pad_%s_slide' % side, r, (ROLL_X + 0.23, GRIP_Y, zc))
        m.add(n, box_span('paint_yellow', ROLL_X + 0.06, ROLL_X + 0.40, GRIP_Y - 0.30, GRIP_Y + 0.26, zc - 0.12, zc + 0.12))
        m.add(n, box_span('paint_yellow', ROLL_X + 0.40, ROLL_X + 0.70, GRIP_Y - 0.12, GRIP_Y + 0.12, zc - 0.10, zc + 0.10))
        m.add(n, box_span('paint_yellow', ROLL_X + 0.25, ROLL_X + 0.85, GRIP_Y - 0.65, GRIP_Y + 0.65, zc - 0.10, zc + 0.10))   # pad plate
        # rubber facing on the inner side and two turning rollers
        zi = zc - s * 0.10
        m.add(n, box_span('rubber_trim', ROLL_X + 0.28, ROLL_X + 0.82, GRIP_Y - 0.60, GRIP_Y + 0.60,
                          min(zi, zi - s * 0.04), max(zi, zi - s * 0.04)))
        for dx in (0.40, 0.70):
            m.add(n, cyl_between('steel_dark', (ROLL_X + dx, GRIP_Y - 0.55, zi - s * 0.07), (ROLL_X + dx, GRIP_Y + 0.55, zi - s * 0.07), 0.06, 14))
            for yy in (GRIP_Y - 0.58, GRIP_Y + 0.58):
                m.add(n, box_span('steel_dark', ROLL_X + dx - 0.06, ROLL_X + dx + 0.06, yy - 0.03, yy + 0.03,
                                  min(zi - s * 0.04, zi - s * 0.13), max(zi - s * 0.04, zi - s * 0.13)))
        m.cylinder('pad_%s_cylinder' % side, r, n, (ROLL_X + 0.23, GRIP_Y + 0.30, s * 0.20), (ROLL_X + 0.23, GRIP_Y + 0.30, s * (PAD_Z - 0.05)),
                   0.08, 0.05, extension=0.55)
        m.add(n, box_span('paint_yellow', ROLL_X + 0.15, ROLL_X + 0.32, GRIP_Y + 0.26, GRIP_Y + 0.40, s * (PAD_Z - 0.12), s * (PAD_Z + 0.02)))
        m.add(n, lamp((ROLL_X + 0.70, GRIP_Y + 0.20, zc + s * 0.06), 1, 0.5))
    m.add(r, box_span('paint_yellow', ROLL_X + 0.15, ROLL_X + 0.32, GRIP_Y + 0.14, GRIP_Y + 0.40, -0.26, 0.26))


def build():
    m = Model('Tyre Handler', MINING_MATERIALS)
    m.node('rear_frame')
    m.node('cab', 'rear_frame')
    m.node('rear_axle_rotate', 'rear_frame', (AX_R, WY + 0.18, 0))
    m.node('hitch_rotate', 'rear_frame', (0.0, HITCH_Y, 0.0))
    m.node('arm_tilt', 'hitch_rotate', (BP[0], BP[1], 0))
    m.node('gripper_tilt', 'arm_tilt', (AT[0], AT[1], 0))
    m.node('gripper_rotate', 'gripper_tilt', (ROLL_X, GRIP_Y, 0))
    running_gear(m)
    front_frame(m)
    rear_frame(m)
    arms(m)
    manipulator(m)
    for s in (1, -1):
        m.cylinder('steering_cylinder_%s' % ('right' if s > 0 else 'left'), 'rear_frame', 'hitch_rotate',
                   (-0.95, 1.20, s * 0.62), (0.72, 1.20, s * 0.55), 0.10, 0.06, extension=0.55)
        m.add('rear_frame', box_span('chassis_black', -1.10, -0.85, 1.05, 1.35, s * 0.50, s * 0.72))
        m.add('hitch_rotate', box_span('chassis_black', 0.62, 0.82, 1.05, 1.35, s * 0.45, s * 0.62))
    return m


if __name__ == '__main__':
    m = build()
    save(m, 'tyre-handler')
    print('tris', m.stats()[1])
