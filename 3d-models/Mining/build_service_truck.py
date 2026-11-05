"""Mine service / lube truck: rigid 6x4 on-highway-class chassis with a service body.

Key real-world dimensions used (metres):
  front axle 3.90, tandem at -2.20 / -3.55 (1.35 spread), 315/80R22.5 tyres (1.08 dia),
  front track 2.06, rear duals at 0.34 centres
  conventional bonneted day cab, roof at 2.85; body 2.5 wide, 6.5 long, top deck at 2.60
  three bays of roll-up doors each side, four hose reels in the open rear bay
  three fluid tanks across the body front (engine oil, hydraulic oil, coolant)
  knuckle crane on the rear deck, stowed folded forward; overall ~10.0 x 2.6 x 3.6
"""
import math

from mining_parts import (MINING_MATERIALS, Model, beacon, box, box_span, cyl_between,  # isort: skip
                          cylinder, deck, extrude, guardrail, hazard_stripes, hose, ladder, lamp, save,
                          sphere, tank_cap)
from bodykit import rbox  # noqa: E402  (on sys.path once mining_parts is imported)
from vehicle_parts import arch_polygon, place_wheel, rim, steering_wheel, tyre, wheel

AX_F, AX_R1, AX_R2 = 3.90, -2.20, -3.55
TR, TW, BEAD = 0.54, 0.31, 0.29
WY = TR
ZF = 1.03
ZD_IN, ZD_OUT = 0.735, 1.075
BODY_X0, BODY_X1 = -4.95, 1.60
BODY_Y0, DECK_Y = 1.12, 2.60
HW = 1.25
CAB_X0, CAB_X1, CAB_Y0, CAB_Y1 = 2.00, 3.55, 1.28, 2.85
CRANE = (-4.30, 0.72)          # slew centre (x, z) on the rear deck
BOOM_P = (-4.30, 3.00)         # boom hinge
JIB_P = (-1.72, 3.00)          # knuckle (jib hinge) at the boom tip
TANKS = (-0.55, 0.32, 1.19)    # tank centres (x)


def running_gear(m):
    w = wheel(TR, TW, BEAD, 'hwy', 'steel', 'steel', segs=32, lugs=32)
    # inner duals are hidden behind the outer ones: plain grooved tyre and the rim barrel only
    w_in = tyre(TR, TW, BEAD, 'hwy', segs=32, lugs=32)[:1] + rim(BEAD, TW, 'steel', 'steel')[:2]
    for s, side in ((1, 'right'), (-1, 'left')):
        st = m.node('wheel_front_%s_steer' % side, 'chassis', (AX_F, WY, s * (ZF - 0.22)))
        n = m.node('wheel_front_%s_spin' % side, st, (AX_F, WY, s * ZF))
        m.add(n, place_wheel(w, (AX_F, WY, s * ZF), s))
        m.add(st, cyl_between('chassis_black', (AX_F, WY, s * (ZF - 0.25)), (AX_F, WY, s * (ZF - 0.12)), 0.12, 16))
        for ax, tag in ((AX_R1, 'rear_1'), (AX_R2, 'rear_2')):
            n = m.node('wheel_%s_%s_spin' % (tag, side), 'chassis', (ax, WY, s * (ZD_IN + ZD_OUT) / 2))
            m.add(n, place_wheel(w, (ax, WY, s * ZD_OUT), s))
            m.add(n, place_wheel(w_in, (ax, WY, s * ZD_IN), -s))
    m.add('chassis', cyl_between('chassis_black', (AX_F, WY, -ZF + 0.25), (AX_F, WY, ZF - 0.25), 0.07, 12))
    for ax in (AX_R1, AX_R2):
        m.add('chassis', cyl_between('chassis_black', (ax, WY, -ZD_IN + 0.15), (ax, WY, ZD_IN - 0.15), 0.08, 14))
        m.add('chassis', sphere('chassis_black', 0.22, 16, 8).scale(1, 0.9, 1.0).at((ax, WY, 0)))
    for s in (1, -1):   # leaf springs and walking beam
        m.add('chassis', box_span('steel_dark', AX_F - 0.70, AX_F + 0.70, WY + 0.08, WY + 0.18, s * 0.38, s * 0.48))
        m.add('chassis', box_span('steel_dark', AX_R2 - 0.20, AX_R1 + 0.20, WY + 0.06, WY + 0.18, s * 0.38, s * 0.48))
    m.add('chassis', cyl_between('chassis_black', (AX_R1 + 0.6, WY + 0.15, 0), (AX_F - 0.9, 0.80, 0), 0.05, 10))


def chassis(m):
    p = 'chassis'
    for s in (1, -1):
        m.add(p, box_span('chassis_black', BODY_X0, 4.80, 0.78, 1.06, s * 0.38, s * 0.47))
    for x in (-4.6, -2.9, -1.0, 1.0, 3.0):
        m.add(p, box_span('chassis_black', x - 0.04, x + 0.04, 0.82, 1.02, -0.38, 0.38))
    # front bumper with lamps
    m.add(p, box_span('steel', 4.80, 5.00, 0.62, 1.02, -1.22, 1.22))
    for s in (1, -1):
        m.add(p, box('lamp_lens', 0.012, 0.12, 0.22, (5.006, 0.86, s * 0.90)))
        m.add(p, box('lamp_amber', 0.012, 0.08, 0.14, (5.006, 0.86, s * 0.62)))
        m.add(p, box_span('steel_dark', 4.95, 5.06, 0.64, 0.70, s * 0.30, s * 0.40))
    # fuel tank (left), battery / air tanks (right), exhaust stack and intake behind the cab
    m.add(p, cyl_between('aluminum', (1.75, 0.80, -0.82), (0.45, 0.80, -0.82), 0.30, 24))
    m.add(p, tank_cap(1.30, 1.10, -0.82, 0.05))
    m.add(p, box_span('chassis_black', 0.45, 1.75, 0.55, 1.05, 0.50, 1.10))
    m.add(p, box_span('steel_dark', 0.50, 1.70, 1.05, 1.07, 0.50, 1.10))
    m.add(p, cyl_between('chrome', (1.80, 1.10, 0.95), (1.80, 3.30, 0.95), 0.07, 14))
    m.add(p, cyl_between('chrome', (1.80, 3.30, 0.95), (1.72, 3.40, 0.95), 0.07, 14))
    m.add(p, cyl_between('steel_dark', (1.80, 1.40, -0.95), (1.80, 2.95, -0.95), 0.14, 18))
    m.add(p, cylinder('chassis_black', 0.16, 0.10, 18).at((1.80, 3.00, -0.95)))
    # mudflaps behind the tandem
    for s in (1, -1):
        m.add(p, box_span('rubber_trim', AX_R2 - 0.68, AX_R2 - 0.66, 0.25, 1.05, s * 0.60, s * 1.20))


def cab(m):
    p = 'cab'
    # bonnet, grille, front fenders
    m.add(p, rbox('paint_white', 1.32, 0.86, 1.70, 0.10, (4.18, 1.62, 0)))
    m.add(p, box_span('steel_dark', 4.83, 4.85, 1.18, 1.95, -0.62, 0.62))
    y = 1.24
    while y < 1.92:
        m.add(p, box_span('chrome', 4.85, 4.87, y, y + 0.03, -0.60, 0.60))
        y += 0.08
    for s in (1, -1):
        m.add(p, extrude('paint_white', arch_polygon(AX_F, WY, TR + 0.06, TR + 0.10, 0, 180, 16), 0.42).move(z=s * ZF))
        m.add(p, box_span('paint_white', AX_F + 0.62, 4.85, 1.10, 1.30, s * 0.82, s * 1.24))
        m.add(p, box('lamp_lens', 0.012, 0.14, 0.26, (4.856, 1.20, s * 1.02)))
        m.add(p, box('lamp_amber', 0.012, 0.06, 0.14, (4.856, 1.20, s * 0.76)))
    # cab shell, glazing, doors
    cx = (CAB_X0 + CAB_X1) / 2
    m.add(p, rbox('paint_white', CAB_X1 - CAB_X0, CAB_Y1 - CAB_Y0, 2.36, 0.12, (cx, (CAB_Y0 + CAB_Y1) / 2, 0)))
    gy0, gy1 = 2.08, 2.70
    m.add(p, box_span('glass', CAB_X1 - 0.002, CAB_X1 + 0.006, gy0, gy1, -1.02, 1.02))
    m.add(p, box_span('chassis_black', CAB_X1 - 0.004, CAB_X1 + 0.002, gy0 - 0.04, gy1 + 0.04, -1.06, 1.06))
    m.add(p, box_span('glass', CAB_X0 - 0.006, CAB_X0 + 0.002, 2.15, 2.65, -0.70, 0.70))
    for s in (1, -1):
        zs = s * 1.18
        m.add(p, box_span('glass', CAB_X0 + 0.55, CAB_X1 - 0.12, gy0, gy1, zs - 0.004, zs + 0.004))
        for a in (CAB_X0 + 0.45, CAB_X1 - 0.05):
            m.add(p, box_span('rubber_trim', a - 0.01, a + 0.01, CAB_Y0 + 0.05, gy1 + 0.05, s * 1.177, s * 1.186))
        m.add(p, box_span('steel_dark', CAB_X1 - 0.40, CAB_X1 - 0.25, 1.85, 1.89, s * 1.18, s * 1.21))
        # steps and grab handles
        for k, yy in enumerate((0.62, 0.98)):
            m.add(p, box_span('steel', CAB_X0 + 0.55, CAB_X1 - 0.20, yy - 0.03, yy, s * (1.00 + 0.06 * k), s * 1.22))
            m.add(p, box_span('chassis_black', CAB_X0 + 0.53, CAB_X0 + 0.55, yy - 0.12, yy, s * 1.00, s * 1.22))
        m.add(p, box_span('chassis_black', CAB_X0 + 0.53, CAB_X0 + 0.55, 0.50, 1.28, s * 0.98, s * 1.18))
        m.add(p, cyl_between('chrome', (CAB_X0 + 0.40, 1.40, s * 1.21), (CAB_X0 + 0.40, 2.40, s * 1.21), 0.016, 8))
        # mirrors
        a, b = (CAB_X1 - 0.05, 2.30, s * 1.18), (CAB_X1 + 0.15, 2.30, s * 1.42)
        m.add(p, cyl_between('steel_dark', a, b, 0.014, 8))
        m.add(p, box('chassis_black', 0.06, 0.36, 0.20, (b[0], b[1] - 0.05, b[2])))
        m.add(p, box('glass', 0.01, 0.32, 0.16, (b[0] - 0.033, b[1] - 0.05, b[2])))
    # roof: light bar, beacons, aerial, sun visor
    m.add(p, box_span('paint_white', CAB_X1 - 0.05, CAB_X1 + 0.12, 2.74, 2.78, -1.05, 1.05))
    m.add(p, box_span('chassis_black', CAB_X1 - 0.60, CAB_X1 - 0.40, 2.85, 2.90, -0.90, 0.90))
    for s in (1, -1):
        m.add(p, lamp((CAB_X1 - 0.50, 2.98, s * 0.55), 1, 0.6))
    m.add(p, beacon((CAB_X0 + 0.25, 2.85, -0.85), 0.06), beacon((CAB_X0 + 0.25, 2.85, 0.85), 0.06))
    m.add(p, cyl_between('steel_dark', (CAB_X0 + 0.40, 2.85, 0.0), (CAB_X0 + 0.40, 3.40, 0.0), 0.006, 6))
    # interior: seats, wheel, dash
    m.add(p, box_span('plastic_black', CAB_X1 - 0.40, CAB_X1 - 0.10, 1.40, 1.95, -1.05, 1.05))
    for z in (-0.55, 0.55):
        m.add(p, box_span('seat_fabric', CAB_X0 + 0.45, CAB_X0 + 0.95, 1.65, 1.78, z - 0.25, z + 0.25))
        m.add(p, box('seat_fabric', 0.10, 0.70, 0.48, (CAB_X0 + 0.42, 2.10, z)).rotate('z', 10, (CAB_X0 + 0.42, 1.78, z)))
        m.add(p, box_span('chassis_black', CAB_X0 + 0.55, CAB_X0 + 0.85, 1.32, 1.65, z - 0.18, z + 0.18))
    m.add(p, steering_wheel((CAB_X1 - 0.55, 2.10, -0.55), 0.22, 30))


def body(m):
    p = 'body'
    x0, x1 = BODY_X0, BODY_X1
    m.add(p, box_span('chassis_black', x0, x1, 1.06, BODY_Y0, -0.60, 0.60))        # subframe
    m.add(p, box_span('paint_yellow', x0 + 1.30, x1, BODY_Y0, DECK_Y, -HW, HW))
    # rear reel bay: floor, roof, side walls, centre partition
    m.add(p, box_span('paint_yellow', x0, x0 + 1.30, BODY_Y0, BODY_Y0 + 0.08, -HW, HW))
    m.add(p, box_span('paint_yellow', x0, x0 + 1.30, DECK_Y - 0.10, DECK_Y, -HW, HW))
    for s in (1, -1):
        m.add(p, box_span('paint_yellow', x0, x0 + 1.30, BODY_Y0 + 0.08, DECK_Y - 0.10, s * (HW - 0.05), s * HW))
    m.add(p, box_span('steel_dark', x0 + 1.28, x0 + 1.29, BODY_Y0 + 0.30, DECK_Y - 0.30, -HW + 0.10, HW - 0.10))
    # roll-up doors: three bays a side, each with a head box; doors slide up into it
    bays = [(x0 + 1.40, x0 + 3.05), (x0 + 3.15, x0 + 4.80), (x0 + 4.90, x1 - 0.10)]
    for s, side in ((1, 'right'), (-1, 'left')):
        zs = s * HW
        for i, (a, b) in enumerate(bays):
            m.add(p, box_span('paint_yellow', a - 0.04, b + 0.04, DECK_Y - 0.22, DECK_Y - 0.02, zs, zs + s * 0.06))
            for xx in (a - 0.04, b):
                m.add(p, box_span('aluminum', xx, xx + 0.04, BODY_Y0 + 0.10, DECK_Y - 0.22, zs, zs + s * 0.03))
            d = m.node('door_%s_%d_slide' % (side, i + 1), p, ((a + b) / 2, BODY_Y0 + 0.12, zs))
            yy = BODY_Y0 + 0.12
            k = 0
            while yy < DECK_Y - 0.25:
                m.add(d, box_span('aluminum', a, b, yy, min(yy + 0.072, DECK_Y - 0.24), zs + s * 0.004, zs + s * (0.020 if k % 2 else 0.016)))
                yy += 0.075
                k += 1
            m.add(d, box_span('steel', a + 0.005, b - 0.005, BODY_Y0 + 0.10, BODY_Y0 + 0.17, zs + s * 0.006, zs + s * 0.03))
            m.add(d, box_span('steel_dark', (a + b) / 2 - 0.12, (a + b) / 2 + 0.12, BODY_Y0 + 0.19, BODY_Y0 + 0.23, zs + s * 0.03, zs + s * 0.05))
        m.add(p, box_span('chassis_black', x0, x1, BODY_Y0 - 0.06, BODY_Y0, s * (HW - 0.02), s * (HW + 0.06)))
        for x in (x0 + 0.30, x1 - 0.30, (x0 + x1) / 2):
            m.add(p, box('lamp_amber', 0.08, 0.05, 0.012, (x, BODY_Y0 - 0.03, s * (HW + 0.066))))
    # front headboard with light bar and beacons
    m.add(p, box_span('paint_yellow', x1, x1 + 0.08, BODY_Y0, 3.30, -HW, HW))
    for s in (1, -1):
        m.add(p, lamp((x1 + 0.05, 3.40, s * 0.95), -1, 0.7))
        m.add(p, box_span('paint_yellow', x1 - 0.06, x1, DECK_Y, 3.30, s * (HW - 0.10), s * HW))
    m.add(p, beacon((x1 + 0.04, 3.30, 0.0), 0.06))
    # fluid tanks across the body front in saddles, colour-banded
    bands = ('paint_orange', 'paint_blue', 'paint_green')
    for x, band in zip(TANKS, bands):
        yc = DECK_Y + 0.44
        m.add(p, cyl_between('steel', (x, yc, -1.10), (x, yc, 1.10), 0.40, 28))
        for s in (1, -1):
            m.add(p, cyl_between('steel', (x, yc, s * 1.10), (x, yc, s * 1.14), 0.36, 28))
            m.add(p, cyl_between(band, (x, yc, s * 0.82), (x, yc, s * 0.92), 0.405, 28))
            m.add(p, extrude('paint_yellow', [(x - 0.40, DECK_Y), (x + 0.40, DECK_Y), (x + 0.30, yc - 0.20), (x - 0.30, yc - 0.20)], 0.08).move(z=s * 0.60))
        m.add(p, tank_cap(x, yc + 0.39, 0.30, 0.07))
        m.add(p, cyl_between('steel_dark', (x, yc + 0.38, -0.40), (x, yc + 0.52, -0.40), 0.02, 8))
    # rear deck: grating, guardrail, ladder at the rear
    m.add(p, deck(x0 + 0.02, TANKS[0] - 0.50, -HW + 0.02, HW - 0.02, DECK_Y + 0.04, t=0.04))
    m.add(p, guardrail([(x0 + 0.05, -HW + 0.05), (TANKS[0] - 0.50, -HW + 0.05)], DECK_Y + 0.04, h=0.75))
    m.add(p, guardrail([(x0 + 0.05, -0.40), (x0 + 0.05, -HW + 0.05)], DECK_Y + 0.04, h=0.75))
    m.add(p, ladder(x0 - 0.10, 0.45, DECK_Y + 0.04, 0.0, width=0.46))
    for s in (1, -1):
        m.add(p, box_span('paint_yellow', x0 - 0.12, x0, DECK_Y - 0.20, DECK_Y - 0.10, s * 0.20, s * 0.26))
    # hose reels in the rear bay, nozzles hanging in holsters
    for i, zr in enumerate((-0.84, -0.28, 0.28, 0.84)):
        rc = (x0 + 0.62, 1.78, zr)
        m.add(p, box_span('paint_yellow', rc[0] - 0.30, rc[0] + 0.30, BODY_Y0 + 0.08, BODY_Y0 + 0.14, zr - 0.24, zr + 0.24))
        for dz in (-0.235, 0.235):
            m.add(p, box_span('paint_yellow', rc[0] - 0.05, rc[0] + 0.05, BODY_Y0 + 0.14, rc[1], zr + dz - 0.01, zr + dz + 0.01))
        n = m.node('hose_reel_%d_spin' % (i + 1), p, rc)
        m.add(n, cyl_between('steel_dark', (rc[0], rc[1], zr - 0.24), (rc[0], rc[1], zr + 0.24), 0.04, 10))
        for dz in (-0.20, 0.20):
            m.add(n, cyl_between('paint_red' if i % 2 else 'paint_yellow', (rc[0], rc[1], zr + dz - 0.01), (rc[0], rc[1], zr + dz + 0.01), 0.36, 28))
        m.add(n, cyl_between('hose_rubber', (rc[0], rc[1], zr - 0.19), (rc[0], rc[1], zr + 0.19), 0.27, 28))
        m.add(p, hose([(rc[0] - 0.20, rc[1] - 0.18, zr), (x0 - 0.02, 1.45, zr), (x0 - 0.05, 1.30, zr)], 0.018))
        m.add(p, cyl_between('steel', (x0 - 0.05, 1.30, zr), (x0 - 0.05, 1.12, zr), 0.022, 8))
        m.add(p, box_span('chassis_black', x0 - 0.10, x0 - 0.01, 1.05, 1.25, zr - 0.05, zr + 0.05))
    # rear bumper, lights and hazard stripes
    m.add(p, box_span('paint_yellow', x0 - 0.18, x0, 0.55, 0.85, -HW, HW))
    for s in (1, -1):
        m.add(p, hazard_stripes((x0 - 0.18, 0.57, 0.30 if s > 0 else -HW + 0.02), (0, 0, 1), (0, 1, 0), 0.93, 0.26, 0.10))
        m.add(p, box('lamp_red', 0.012, 0.12, 0.20, (x0 - 0.006, 1.00, s * 1.10)))
        m.add(p, box('lamp_amber', 0.012, 0.10, 0.14, (x0 - 0.006, 1.00, s * 0.88)))
        m.add(p, lamp((x0 - 0.08, DECK_Y - 0.05, s * 1.05), -1, 0.7))


def crane(m):
    cx, cz = CRANE
    rot, boom, jib = 'crane_rotate', 'crane_boom_tilt', 'crane_jib_tilt'
    m.add('body', cylinder('paint_yellow', 0.28, 0.12, 24).at((cx, DECK_Y + 0.10, cz)))
    m.add(rot, cylinder('steel_dark', 0.25, 0.04, 24).at((cx, DECK_Y + 0.18, cz)))
    m.add(rot, box_span('paint_yellow', cx - 0.18, cx + 0.18, DECK_Y + 0.20, BOOM_P[1] + 0.10, cz - 0.16, cz + 0.16))
    m.add(rot, box_span('paint_yellow', cx + 0.15, cx + 0.40, DECK_Y + 0.20, DECK_Y + 0.40, cz - 0.10, cz + 0.10))
    m.add(rot, cyl_between('steel_dark', (BOOM_P[0], BOOM_P[1], cz - 0.20), (BOOM_P[0], BOOM_P[1], cz + 0.20), 0.05, 12))
    # control station beside the column
    m.add(rot, box_span('chassis_black', cx - 0.10, cx + 0.05, DECK_Y + 0.55, DECK_Y + 0.80, cz + 0.19, cz + 0.31))
    for k in range(4):
        m.add(rot, cyl_between('chassis_black', (cx - 0.05 + k * 0.035, DECK_Y + 0.80, cz + 0.22), (cx - 0.05 + k * 0.035, DECK_Y + 0.92, cz + 0.22), 0.008, 6))
    # boom (stowed forward, horizontal)
    m.add(boom, box_span('paint_yellow', BOOM_P[0] - 0.12, JIB_P[0] + 0.10, BOOM_P[1] - 0.12, BOOM_P[1] + 0.10, cz - 0.10, cz + 0.10))
    m.add(boom, box_span('paint_yellow', -3.70, -3.40, BOOM_P[1] - 0.24, BOOM_P[1] - 0.12, cz - 0.07, cz + 0.07))
    m.add(boom, box_span('paint_yellow', -3.15, -2.90, BOOM_P[1] - 0.02, BOOM_P[1] + 0.08, cz + 0.10, cz + 0.22))
    m.add(boom, cyl_between('steel_dark', (JIB_P[0], JIB_P[1], cz - 0.15), (JIB_P[0], JIB_P[1], cz + 0.15), 0.05, 12))
    m.add(boom, hose([(BOOM_P[0] + 0.10, BOOM_P[1] + 0.12, cz - 0.05), (JIB_P[0] - 0.20, BOOM_P[1] + 0.12, cz - 0.05)], 0.016))
    # jib folded back on top of the boom, extension and hook
    m.add(jib, box_span('paint_yellow', -3.95, JIB_P[0] + 0.12, JIB_P[1] + 0.13, JIB_P[1] + 0.29, cz - 0.08, cz + 0.08))
    m.add(jib, box_span('paint_yellow', JIB_P[0] - 0.08, JIB_P[0] + 0.12, JIB_P[1] + 0.06, JIB_P[1] + 0.40, cz - 0.09, cz + 0.09))
    m.add(jib, box_span('steel', -4.15, -3.95, JIB_P[1] + 0.16, JIB_P[1] + 0.26, cz - 0.05, cz + 0.05))
    m.add(jib, cyl_between('steel_dark', (-4.12, JIB_P[1] + 0.16, cz), (-4.12, JIB_P[1] - 0.05, cz), 0.012, 6))
    m.add(jib, extrude('steel_dark', [(-4.16, JIB_P[1] - 0.05), (-4.08, JIB_P[1] - 0.05), (-4.06, JIB_P[1] - 0.18),
                                      (-4.14, JIB_P[1] - 0.22), (-4.18, JIB_P[1] - 0.16), (-4.14, JIB_P[1] - 0.14)], 0.03).move(z=cz))
    m.cylinder('crane_boom_cylinder', rot, boom, (cx + 0.32, DECK_Y + 0.30, cz), (-3.55, BOOM_P[1] - 0.20, cz), 0.07, 0.045, extension=0.55)
    m.cylinder('crane_jib_cylinder', boom, jib, (-3.02, BOOM_P[1] + 0.03, cz + 0.16), (JIB_P[0] + 0.02, JIB_P[1] + 0.32, cz + 0.16),
               0.06, 0.035, extension=0.55)


def build():
    m = Model('Service Truck', MINING_MATERIALS)
    m.node('chassis')
    m.node('cab', 'chassis')
    m.node('body', 'chassis')
    m.node('crane_rotate', 'body', (CRANE[0], DECK_Y + 0.16, CRANE[1]))
    m.node('crane_boom_tilt', 'crane_rotate', (BOOM_P[0], BOOM_P[1], CRANE[1]))
    m.node('crane_jib_tilt', 'crane_boom_tilt', (JIB_P[0], JIB_P[1], CRANE[1]))
    running_gear(m)
    chassis(m)
    cab(m)
    body(m)
    crane(m)
    return m


if __name__ == '__main__':
    m = build()
    save(m, 'service-truck')
    print('tris', m.stats()[1])
