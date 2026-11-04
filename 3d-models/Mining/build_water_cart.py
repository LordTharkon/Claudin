"""Mine water cart: rigid two-axle mining haul-truck chassis carrying a ~90 kL water tank.

Key real-world dimensions used (metres):
  wheelbase 5.90, 40.00R57-class tyres (3.56 dia x 1.05 wide), duals at the rear
  front track 4.84, rear dual centres 3.56 / 5.84, upper deck 7.0 wide at 4.15
  tank 8.0 long x 5.6 wide, stepped between the rear tyres, ~89 m3 gross
  rear spray bar with 4 heads, front water cannon on a turret (cannon_rotate / cannon_tilt)
  height to the top of the cab ~6.4 (tank-top handrails ~6.3), overall length ~12.5
"""
import math

from mining_parts import (MINING_MATERIALS, Model, beacon, box, box_between, box_span, cab,
                          cyl_between, cylinder, deck, exhaust, extrude, guardrail, hazard_stripes,
                          hose, ladder, lamp, louvres, precleaner, save, sight_gauge, sphere, stairs,
                          tank_cap)
from vehicle_parts import arch_polygon, place_wheel, wheel

AX_F, AX_R = 3.30, -2.60
TR, TW, BEAD = 1.78, 1.05, 0.74
WY = 1.72
ZF = 2.42                  # front wheel centre
ZR_IN, ZR_OUT = 1.78, 2.92  # rear duals
DECK_Y = 4.15
TANK_X0, TANK_X1 = -5.60, 2.40
TANK_TOP = 5.25
CANNON = (5.35, 2.45)      # turret position (x, z) on the upper deck


def tank_poly(e=0.0):
    """Tank cross-section (z, y), grown by e: narrow between the rear tyres, full width above."""
    return [(-1.15 - e, 2.75 - e), (1.15 + e, 2.75 - e), (1.15 + e, 3.62 - e), (2.60 + e, 3.62 - e),
            (2.80 + e, 3.82), (2.80 + e, 5.00), (2.55 + e * 0.5, TANK_TOP + e), (-2.55 - e * 0.5, TANK_TOP + e),
            (-2.80 - e, 5.00), (-2.80 - e, 3.82), (-2.60 - e, 3.62 - e), (-1.15 - e, 3.62 - e)]


def along_x(poly, x0, x1, mat='paint_yellow'):
    """Extrude a (z, y) cross-section along x from x0 to x1."""
    return extrude(mat, poly, x1 - x0).rotate('y', 90).move(x=(x0 + x1) / 2)


def running_gear(m):
    w = wheel(TR, TW, BEAD, 'chevron', 'otr', lugs=28, depth=0.11)
    for s, side in ((1, 'right'), (-1, 'left')):
        kp = (AX_F, WY, s * (ZF - 0.75))
        st = m.node('wheel_front_%s_steer' % side, 'chassis', kp)
        n = m.node('wheel_front_%s_spin' % side, st, (AX_F, WY, s * ZF))
        m.add(n, place_wheel(w, (AX_F, WY, s * ZF), s))
        # spindle, upright and steering arm (steer with the wheel)
        m.add(st, cyl_between('chassis_black', (AX_F, WY, s * (ZF - 0.95)), (AX_F, WY, s * (ZF - 0.42)), 0.36, 24))
        m.add(st, box_span('chassis_black', AX_F - 0.25, AX_F + 0.25, WY - 0.35, WY + 0.95, s * (ZF - 1.05), s * (ZF - 0.85)))
        m.add(st, box_span('chassis_black', AX_F + 0.20, AX_F + 0.80, 1.90, 2.08, s * (ZF - 1.02), s * (ZF - 0.88)))
        m.cylinder('steering_cylinder_%s' % side, 'chassis', st, (AX_F + 0.75, 1.99, s * 0.85), (AX_F + 0.72, 1.99, s * (ZF - 0.95)),
                   0.10, 0.06, extension=0.55)
        # front suspension strut (oil-pneumatic)
        m.add('chassis', cyl_between('paint_yellow', (AX_F, WY + 1.20, s * (ZF - 0.95)), (AX_F, WY + 2.15, s * (ZF - 0.95)), 0.22, 20))
        m.add(st, cyl_between('chrome', (AX_F, WY + 0.90, s * (ZF - 0.95)), (AX_F, WY + 1.30, s * (ZF - 0.95)), 0.15, 18))
        # rear duals on one hub
        n = m.node('wheel_rear_%s_spin' % side, 'chassis', (AX_R, WY, s * (ZR_IN + ZR_OUT) / 2))
        m.add(n, place_wheel(w, (AX_R, WY, s * ZR_OUT), s))
        m.add(n, place_wheel(w, (AX_R, WY, s * ZR_IN), -s))
        m.add(n, cyl_between('chassis_black', (AX_R, WY, s * (ZR_IN + 0.30)), (AX_R, WY, s * (ZR_OUT - 0.30)), 0.55, 28))
    # rear axle housing, differential, final drives, struts and A-frame
    c = 'chassis'
    m.add(c, cyl_between('chassis_black', (AX_R, WY, -ZR_IN + 0.40), (AX_R, WY, ZR_IN - 0.40), 0.45, 28))
    m.add(c, sphere('chassis_black', 0.85, 24, 12).scale(1.0, 0.85, 0.95).at((AX_R, WY, 0)))
    for s in (1, -1):
        m.add(c, cyl_between('paint_yellow', (AX_R - 0.15, WY + 0.30, s * 1.00), (AX_R + 0.75, 2.75, s * 0.95), 0.20, 18))
        m.add(c, box_between('chassis_black', (AX_R + 0.3, WY, s * 0.6), (1.4, 1.75, s * 0.25), 0.22, 0.35))
    m.add(c, sphere('steel_dark', 0.25, 16, 8).at((1.45, 1.75, 0)))


def chassis(m):
    c = 'chassis'
    for s in (1, -1):
        m.add(c, box_span('chassis_black', -4.80, 6.05, 1.65, 2.70, s * 0.55, s * 0.85))
    for x in (-4.6, -1.2, 1.6, 4.4, 5.8):
        m.add(c, box_span('chassis_black', x - 0.12, x + 0.12, 1.85, 2.55, -0.55, 0.55))
    # engine / radiator housing under the upper deck
    m.add(c, box_span('paint_yellow', 2.65, 5.90, 2.55, DECK_Y - 0.07, -1.30, 1.30))
    for s in (1, -1):
        m.add(c, louvres(3.10, 5.40, 2.85, 3.78, s * 1.30, s))
    m.add(c, box_span('paint_yellow', 5.90, 6.08, 2.55, DECK_Y - 0.07, -1.45, 1.45))
    m.add(c, box_span('steel_dark', 6.08, 6.10, 2.70, DECK_Y - 0.20, -1.30, 1.30))
    y = 2.78
    while y < DECK_Y - 0.25:
        m.add(c, box('chassis_black', 0.08, 0.03, 2.56, (6.13, y, 0)).rotate('z', -20, (6.13, y, 0)))
        y += 0.09
    # upper deck on columns, front fenders below it
    m.add(c, deck(2.65, 6.05, -3.45, 3.45, DECK_Y, t=0.10))
    m.add(c, box_span('paint_yellow', 2.65, 6.05, DECK_Y - 0.30, DECK_Y - 0.10, 3.40, 3.45))
    m.add(c, box_span('paint_yellow', 2.65, 6.05, DECK_Y - 0.30, DECK_Y - 0.10, -3.45, -3.40))
    for s in (1, -1):
        m.add(c, extrude('paint_yellow', arch_polygon(AX_F, WY, TR + 0.10, TR + 0.15, 15, 165, 18), TW + 0.20).move(z=s * ZF))
        m.add(c, box_span('chassis_black', 2.75, 2.95, 2.55, DECK_Y - 0.10, s * 1.30, s * 1.60))
        m.add(c, box_span('chassis_black', 5.70, 5.90, 2.55, DECK_Y - 0.10, s * 1.45, s * 1.75))
    # front bumper deck with ladders, hazard stripes, lights
    m.add(c, box_span('paint_yellow', 6.05, 6.70, 1.80, 2.48, -2.70, 2.70))
    m.add(c, deck(6.08, 6.70, -2.70, 2.70, 2.55, t=0.07))
    m.add(c, box_span('steel_dark', 6.70, 6.80, 1.95, 2.25, -0.25, 0.25))
    for s in (1, -1):
        m.add(c, hazard_stripes((6.70, 1.86, 2.62 if s > 0 else -0.35), (0, 0, -1), (0, 1, 0), 2.27, 0.30, 0.16))
        m.add(c, ladder(6.62, 0.55, 2.50, s * 2.35, 0.55))
        m.add(c, lamp((6.75, 2.36, s * 1.55), 1, 1.3))
        m.add(c, box('lamp_amber', 0.02, 0.10, 0.20, (6.705, 2.36, s * 2.00)))
    m.add(c, guardrail([(6.66, -2.06), (6.66, 2.06)], 2.55, h=1.0))
    for s in (1, -1):
        m.add(c, guardrail([(6.66, s * 2.66), (6.10, s * 2.66)], 2.55, h=1.0))
    # diagonal stairway across the grille up to the upper deck (built along x, turned onto z)
    st = stairs(-1.90, 2.80, 0.95, DECK_Y, 6.38, width=0.66, rise=0.25)
    m.add(c, [g.rotate('y', 90) for g in st])
    m.add(c, deck(6.05, 6.72, -1.95, -0.62, DECK_Y))
    # upper deck guardrails (front edge open at the stair landing)
    m.add(c, guardrail([(6.68, -0.62), (6.68, -1.92), (6.05, -1.92), (6.05, -3.40), (2.70, -3.40)], DECK_Y))
    m.add(c, guardrail([(6.00, -0.40), (6.00, 3.40), (2.70, 3.40), (2.70, 1.45)], DECK_Y))
    # deck kit: air cleaners, exhaust, lights, beacon, mirrors
    m.add(c, precleaner(3.30, DECK_Y + 0.75, 1.70, 1.2))
    m.add(c, precleaner(3.30, DECK_Y + 0.75, 0.95, 1.2))
    m.add(c, box_span('paint_yellow', 2.90, 4.10, DECK_Y, DECK_Y + 0.75, 0.55, 2.10))
    m.add(c, exhaust(4.40, DECK_Y, DECK_Y + 1.15, 0.80, r=0.13))
    for s in (1, -1):
        m.add(c, lamp((6.05, DECK_Y + 0.35, s * 3.10), 1, 1.3))
        a = (5.80, DECK_Y + 0.9, s * 3.40)
        b = (6.20, DECK_Y + 1.2, s * 3.58)
        m.add(c, cyl_between('steel_dark', (5.80, DECK_Y - 0.05, s * 3.40), a, 0.03, 8), cyl_between('steel_dark', a, b, 0.025, 8))
        m.add(c, box('chassis_black', 0.06, 0.55, 0.35, (b[0], b[1] - 0.25, b[2])), box('glass', 0.01, 0.50, 0.30, (b[0] + 0.035, b[1] - 0.25, b[2])))
    # fuel tank (left) and hydraulic tank (right) between the axles
    m.add(c, box_span('paint_yellow', -0.60, 1.30, 1.30, 2.65, -2.30, -0.90))
    m.add(c, tank_cap(0.9, 2.65, -1.80))
    m.add(c, sight_gauge(0.0, 1.55, 2.45, -2.30, -1))
    m.add(c, box_span('paint_yellow', -0.60, 1.30, 1.30, 2.65, 0.90, 2.30))
    m.add(c, tank_cap(0.9, 2.65, 1.80, 0.08))
    m.add(c, sight_gauge(0.0, 1.65, 2.45, 2.30, 1))

    # cab on the left of the upper deck
    m.add('cab', [g.move(z=-2.35) for g in cab(2.75, 4.85, DECK_Y, 5.92, 1.00, sill=0.55, door_side=-1, wheel=True)])


def tank(m):
    c = 'chassis'
    for s in (1, -1):  # subframe
        m.add(c, box_span('chassis_black', TANK_X0 + 0.05, TANK_X1 - 0.05, 2.70, 2.75, s * 0.50, s * 0.90))
    m.add(c, along_x(tank_poly(), TANK_X0, TANK_X1))
    x = TANK_X0 + 0.6
    while x < TANK_X1 - 0.3:   # stiffening bands
        m.add(c, along_x(tank_poly(0.04), x - 0.06, x + 0.06))
        x += 1.15
    # tank-top walkway with guardrails, manholes, vent and beacons
    m.add(c, deck(TANK_X0 + 0.25, TANK_X1 - 0.15, -0.55, 0.55, TANK_TOP + 0.10, t=0.10))
    for s in (1, -1):
        m.add(c, guardrail([(TANK_X0 + 0.30, s * 0.53), (TANK_X1 - 0.20, s * 0.53)], TANK_TOP + 0.10, h=1.0, post_gap=2.0))
    for x in (-1.6, -4.2):
        m.add(c, cylinder('paint_yellow', 0.34, 0.12, 24).at((x, TANK_TOP + 0.06, 1.25)))
        m.add(c, cylinder('steel_dark', 0.29, 0.04, 20).at((x, TANK_TOP + 0.14, 1.25)))
        m.add(c, box_span('steel_dark', x - 0.05, x + 0.05, TANK_TOP + 0.16, TANK_TOP + 0.20, 1.0, 1.5))
    m.add(c, cyl_between('steel', (0.6, TANK_TOP, -1.40), (0.6, TANK_TOP + 0.55, -1.40), 0.06, 12))
    m.add(c, cyl_between('steel', (0.6, TANK_TOP + 0.55, -1.40), (0.75, TANK_TOP + 0.62, -1.40), 0.06, 12))
    for s in (1, -1):
        m.add(c, beacon((TANK_X0 + 0.25, TANK_TOP, s * 2.35)))
        m.add(c, lamp((TANK_X0 - 0.08, TANK_TOP - 0.20, s * 2.20), -1, 1.2))
    # level indicator and side fill point
    m.add(c, box_span('steel_dark', 1.4, 1.5, 3.90, 5.10, -2.83, -2.80))
    m.add(c, box_span('lamp_amber', 1.42, 1.48, 4.6, 4.7, -2.84, -2.83))
    m.add(c, cyl_between('steel', (1.9, 3.20, -1.15), (1.9, 3.20, -1.75), 0.12, 16))
    m.add(c, cyl_between('steel_dark', (1.9, 3.20, -1.75), (1.9, 3.20, -1.85), 0.18, 16))
    # access ladders: deck -> tank top at the front, ground -> tank top at the rear
    m.add(c, ladder(TANK_X1 + 0.10, DECK_Y, TANK_TOP + 0.10, 1.10, 0.55))
    for s in (1, -1):
        m.add(c, cyl_between('paint_yellow', (TANK_X1 + 0.10, TANK_TOP + 0.05, 1.10 + s * 0.275), (TANK_X1 + 0.10, TANK_TOP + 1.05, 1.10 + s * 0.275), 0.022, 8))
    m.add(c, ladder(TANK_X0 - 0.12, 0.70, TANK_TOP + 0.10, -1.70, 0.55))
    for s in (1, -1):
        m.add(c, cyl_between('paint_yellow', (TANK_X0 - 0.12, TANK_TOP + 0.05, -1.70 + s * 0.275), (TANK_X0 - 0.12, TANK_TOP + 1.05, -1.70 + s * 0.275), 0.022, 8))
        m.add(c, cyl_between('paint_yellow', (TANK_X0 - 0.12, 3.0, -1.70 + s * 0.275), (TANK_X0, 3.0, -1.70 + s * 0.275), 0.02, 6))
    # rear: tail lights, hazard stripes, pump and spray bar with 4 heads
    for s in (1, -1):
        m.add(c, box('lamp_red', 0.02, 0.16, 0.30, (TANK_X0 - 0.01, 3.30, s * 0.80)))
        m.add(c, box('lamp_amber', 0.02, 0.12, 0.24, (TANK_X0 - 0.01, 3.05, s * 0.80)))
        m.add(c, hazard_stripes((TANK_X0, 3.66, 1.30 if s > 0 else -2.75), (0, 0, 1), (0, 1, 0), 1.45, 0.35, 0.16))
    m.add(c, box_span('paint_gray_dark', -5.35, -4.78, 1.80, 2.50, -0.45, 0.45))     # pump
    m.add(c, cyl_between('steel', (-5.05, 2.49, 0), (-5.05, 2.75, 0), 0.14, 16))
    m.add(c, hose([(-5.35, 2.20, 0), (-5.80, 2.00, 0), (-5.85, 1.62, 0)], 0.09))
    m.add(c, cyl_between('steel', (-5.90, 1.55, -2.72), (-5.90, 1.55, 2.72), 0.08, 16))
    for s in (1, -1):
        m.add(c, box_between('chassis_black', (-4.80, 2.20, s * 0.70), (-5.90, 1.62, s * 1.40), 0.10, 0.14))
    for z in (-2.40, -0.80, 0.80, 2.40):
        m.add(c, cyl_between('steel_dark', (-5.90, 1.55, z - 0.16), (-5.90, 1.55, z + 0.16), 0.11, 14))   # valve
        m.add(c, cylinder('paint_gray_dark', 0.07, 0.18, 12).at((-5.90, 1.73, z)))                      # actuator
        m.add(c, cyl_between('steel', (-5.90, 1.50, z), (-6.02, 1.38, z), 0.05, 12))
        fan = box('steel', 0.02, 0.24, 0.42, (0, 0, 0)).rotate('z', -35)
        m.add(c, fan.at((-6.08, 1.30, z)))


def cannon(m):
    x, z = CANNON
    c = 'chassis'
    m.add(c, cylinder('paint_yellow', 0.32, 0.10, 24).at((x, DECK_Y + 0.05, z)))
    m.add(c, cyl_between('steel', (x, DECK_Y + 0.10, z), (x, DECK_Y + 0.55, z), 0.16, 20))
    m.add(c, hose([(2.40, 3.10, 1.0), (2.60, 3.10, 1.6), (2.75, 3.60, 2.2), (x - 0.6, DECK_Y - 0.15, z), (x, DECK_Y - 0.15, z)], 0.10))
    r = m.node('cannon_rotate', c, (x, DECK_Y + 0.55, z))
    m.add(r, cylinder('chassis_black', 0.26, 0.14, 24).at((x, DECK_Y + 0.62, z)))
    for s in (1, -1):
        m.add(r, box_span('paint_yellow', x - 0.18, x + 0.18, DECK_Y + 0.69, DECK_Y + 1.05, z + s * 0.15, z + s * 0.21))
    m.add(r, box_span('paint_yellow', x - 0.40, x - 0.15, DECK_Y + 0.69, DECK_Y + 0.80, z - 0.12, z + 0.12))
    t = m.node('cannon_tilt', r, (x, DECK_Y + 0.92, z))
    m.add(t, cyl_between('steel', (x, DECK_Y + 0.92, z - 0.15), (x, DECK_Y + 0.92, z + 0.15), 0.14, 18))
    m.add(t, cyl_between('steel', (x - 0.25, DECK_Y + 0.92, z), (x + 0.85, DECK_Y + 0.92, z), 0.10, 18))
    m.add(t, cyl_between('steel', (x + 0.85, DECK_Y + 0.92, z), (x + 1.45, DECK_Y + 0.92, z), 0.08, 18, r_top=0.05))
    m.add(t, cyl_between('steel_dark', (x + 1.45, DECK_Y + 0.92, z), (x + 1.55, DECK_Y + 0.92, z), 0.06, 14))
    m.add(t, cyl_between('steel_dark', (x + 0.30, DECK_Y + 0.92, z), (x + 0.38, DECK_Y + 0.92, z), 0.14, 18))
    m.add(t, box_span('steel_dark', x + 0.20, x + 0.40, DECK_Y + 0.72, DECK_Y + 0.84, z - 0.05, z + 0.05))
    m.cylinder('cannon_tilt_cylinder', r, t, (x - 0.30, DECK_Y + 0.75, z), (x + 0.30, DECK_Y + 0.79, z), 0.045, 0.028, extension=0.55)


def build():
    m = Model('Water Cart', MINING_MATERIALS)
    m.node('chassis')
    m.node('cab', 'chassis')
    running_gear(m)
    chassis(m)
    tank(m)
    cannon(m)
    return m


if __name__ == '__main__':
    save(build(), 'water-cart')
