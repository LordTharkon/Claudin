"""Round baler, towed, fixed chamber (1.25 m x 1.20 m bales), roller chamber, net wrap.

Key real-world dimensions used (metres):
  bale chamber 1.25 dia x 1.22 wide with 18 steel rollers, pickup 2.25 wide
  tyres 500/50-17 flotation, track 2.10, hitch eye height 0.48
  overall ~4.3 long, 2.6 wide, 2.3 high
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, WHEEL, auger_flight, hose, implement_tyre, place, save_farm)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from vehicle_parts import rim

C = (-0.10, 1.30)        # chamber centre
RW = 0.92                # side-wall outer radius
RC = 0.74                # roller centres
ZW = 0.70                # side walls
RT, WT, BT, ZT = 0.465, 0.40, 0.216, 1.05
PK = (0.80, 0.62)        # pickup pivot
PR = (1.08, 0.33)        # pickup reel centre
PW = 1.12                # pickup half width


def arc(r, a0, a1, step=10, c=C):
    n = max(1, int(abs(a1 - a0) / step))
    return [(c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def band(r0, r1, a0, a1, step=8, c=C):
    return arc(r1, a0, a1, step, c) + arc(r0, a1, a0, step, c)


def roller(a, node, m):
    x, y = C[0] + RC * math.cos(math.radians(a)), C[1] + RC * math.sin(math.radians(a))
    m.add(node, cyl_between('steel_dark', (x, y, -ZW + 0.04), (x, y, ZW - 0.04), 0.10, 20))
    for s in (1, -1):
        m.add(node, cylinder('steel', 0.055, 0.04, 12).rotate('x', 90).at((x, y, s * (ZW + 0.02))))


def body(m):
    p = 'chassis'
    # front body side walls, roof and front covers
    front = arc(RW, 90, -30) + [(0.98, 0.78), (0.98, 0.42), (C[0], 0.42)]
    for s in (1, -1):
        m.add(p, extrude(BODY, front, 0.025).move(z=s * ZW))
    m.add(p, extrude(BODY, band(RW - 0.03, RW, 25, 90), 2 * ZW - 0.03))
    m.add(p, box_span(BODY, 0.58, 0.98, 1.40, 2.05, -ZW + 0.015, ZW - 0.015))           # net-wrap housing
    m.add(p, box_span(BODY, 0.60, 1.02, 2.05, 2.09, -ZW - 0.02, ZW + 0.02))             # lid
    m.add(p, box_span('steel_dark', 0.99, 1.00, 1.52, 1.62, -0.25, 0.25))               # lid handle plate
    m.add(p, cyl_between('steel', (1.01, 1.57, -0.20), (1.01, 1.57, 0.20), 0.012, 8))
    m.add(p, box_span(FRAME, 0.60, 1.00, 0.42, 0.95, -ZW + 0.015, ZW - 0.015))          # rotor feeder housing
    # left side drive shield with louvres, right side lube/info panel
    m.add(p, extrude(BODY, [(0.05, 0.50), (0.98, 0.50), (0.98, 2.00), (0.40, 2.12), (0.05, 1.90)], 0.04).move(z=-(ZW + 0.06)))
    for k in range(6):
        y = 1.0 + 0.12 * k
        m.add(p, box_span(FRAME, 0.40, 0.85, y, y + 0.03, -(ZW + 0.085), -(ZW + 0.075)))
    m.add(p, box_span(FRAME, 0.30, 0.60, 0.80, 1.00, ZW + 0.012, ZW + 0.02))
    # chassis rails, axle stubs, wheels
    for s in (1, -1):
        m.add(p, box_span(FRAME, -0.70, 1.00, 0.30, 0.45, s * (ZW - 0.04), s * (ZW + 0.06)))
        m.add(p, cyl_between(FRAME, (C[0] - 0.05, RT, s * (ZW + 0.06)), (C[0] - 0.05, RT, s * (ZT - 0.05)), 0.06, 14))
        wnode = m.node('wheel_%s_spin' % ('right' if s > 0 else 'left'), p, (C[0] - 0.05, RT, s * ZT))
        m.add(wnode, place(implement_tyre(RT, WT, BT, ribs=5) + rim(BT, WT, 'steel', WHEEL, nuts=6),
                           (C[0] - 0.05, RT, s * ZT), s))
    for a in range(-15, 276, 17):
        if not 90 < a < 270:
            roller(a, p, m)
    # gearbox, drawbar, PTO driveline, jack, hoses
    m.add(p, box_span('steel_dark', 0.98, 1.20, 0.95, 1.25, -0.15, 0.15))
    for s in (1, -1):
        m.add(p, box_between(FRAME, (0.98, 0.80, s * 0.45), (1.75, 1.00, s * 0.05), 0.10, 0.14))
    m.add(p, box_between(FRAME, (1.70, 1.00, 0), (2.45, 0.52, 0), 0.12, 0.14))
    m.add(p, box_between(FRAME, (2.40, 0.52, 0), (2.62, 0.48, 0), 0.08, 0.06))
    m.add(p, lathe('steel', [(0.035, -0.03), (0.07, -0.03), (0.07, 0.03), (0.035, 0.03), (0.035, -0.03)], 16)
          .at((2.66, 0.48, 0)))
    m.add(p, cyl_between('steel', (1.20, 1.10, 0), (2.40, 0.66, 0), 0.025, 10))
    m.add(p, cyl_between('plastic_black', (1.30, 1.06, 0), (2.25, 0.72, 0), 0.055, 14))
    m.add(p, cyl_between('plastic_black', (2.25, 0.72, 0), (2.42, 0.66, 0), 0.075, 14, r_top=0.055))
    m.add(p, cyl_between('steel_dark', (2.15, 0.02, 0.16), (2.15, 0.82, 0.16), 0.035, 12))     # parking jack
    m.add(p, cyl_between('steel_dark', (2.15, 0.82, 0.16), (2.15, 0.90, 0.16), 0.05, 12))
    m.add(p, cyl_between('steel', (2.15, 0.90, 0.16), (2.27, 0.92, 0.16), 0.012, 8))
    m.add(p, box_span('steel_dark', 2.05, 2.25, 0.0, 0.02, 0.06, 0.26))
    m.add(p, box_span(FRAME, 2.10, 2.20, 0.55, 0.70, 0.06, 0.12))
    for z in (-0.10, -0.15):
        m.add(p, hose([(0.98, 1.20, z - 0.30), (1.70, 1.10, z), (2.40, 0.70, z + 0.03), (2.55, 0.80, z + 0.02)], 0.011))
    m.add(p, hose([(0.98, 1.70, 0.40), (1.70, 1.15, 0.12), (2.40, 0.75, 0.12), (2.55, 0.85, 0.15)], 0.007))
    # bale ramp behind the tailgate
    for s in (1, -1):
        m.add(p, box_between(FRAME, (-0.70, 0.38, s * 0.55), (-1.45, 0.14, s * 0.55), 0.05, 0.06))
    for x, y in ((-0.95, 0.30), (-1.20, 0.22), (-1.42, 0.15)):
        m.add(p, cyl_between('steel_dark', (x, y + 0.03, -0.60), (x, y + 0.03, 0.60), 0.03, 10))


def tailgate(m):
    t = m.node('tailgate_tilt', 'chassis', (C[0], C[1] + RW - 0.02, 0))
    rear = arc(RW, 90, 270)
    for s in (1, -1):
        m.add(t, extrude(BODY, rear, 0.025).move(z=s * ZW))
        m.add(t, box_span(FRAME, C[0] - 0.035, C[0] - 0.008, C[1] - RW + 0.05, C[1] + RW - 0.05, s * (ZW - 0.012), s * (ZW + 0.03)))
    m.add(t, extrude(BODY, band(RW - 0.03, RW, 90, 150), 2 * ZW - 0.03))
    m.add(t, extrude(FRAME, band(RW - 0.03, RW, 205, 262), 2 * ZW - 0.03))
    for a in range(-15, 276, 17):
        if 90 < a < 270:
            roller(a, t, m)
    m.add(t, cyl_between('steel_dark', (C[0], C[1] + RW - 0.02, -ZW - 0.05), (C[0], C[1] + RW - 0.02, ZW + 0.05), 0.04, 12))
    # lights and reflectors on the rear, lock hooks at the bottom
    for s in (1, -1):
        lx, ly = C[0] - RW - 0.02, C[1] - 0.45
        m.add(t, box_span(FRAME, lx - 0.04, lx + 0.10, ly - 0.08, ly + 0.08, s * (ZW - 0.22), s * (ZW + 0.02)))
        m.add(t, box_span('lamp_red', lx - 0.05, lx - 0.04, ly - 0.06, ly + 0.02, s * (ZW - 0.20), s * (ZW - 0.06)))
        m.add(t, box_span('lamp_amber', lx - 0.05, lx - 0.04, ly + 0.02, ly + 0.07, s * (ZW - 0.20), s * (ZW - 0.06)))
        m.add(t, box_span('lamp_red', lx - 0.05, lx - 0.04, ly - 0.06, ly + 0.07, s * (ZW - 0.04), s * ZW))
        m.add(t, box_span('steel_dark', C[0] - 0.12, C[0] + 0.02, C[1] - RW + 0.06, C[1] - RW + 0.18, s * (ZW + 0.025), s * (ZW + 0.05)))
        side = 'right' if s > 0 else 'left'
        m.add(t, box_span(BODY, -0.70, -0.55, 1.85, 2.00, s * (ZW + 0.025), s * (ZW + 0.08)))
        m.add('chassis', box_span(BODY, 0.18, 0.34, 1.45, 1.60, s * (ZW + 0.025), s * (ZW + 0.08)))
        m.cylinder('tailgate_cylinder_' + side, 'chassis', t, (0.26, 1.52, s * (ZW + 0.11)), (-0.62, 1.92, s * (ZW + 0.11)),
                   0.045, 0.028, 0.55)
        m.add('chassis', hose([(0.30, 1.40, s * (ZW + 0.09)), (0.55, 1.20, s * (ZW + 0.09)), (0.90, 1.18, s * 0.60)], 0.01))


def pickup(m):
    k = m.node('pickup_tilt', 'chassis', (*PK, 0))
    # arms from the pivot, end plates, reel, tine bars and stripper bands
    for s in (1, -1):
        m.add(k, box_between(FRAME, (PK[0], PK[1], s * 0.82), (PR[0], PR[1] + 0.05, s * 0.82), 0.05, 0.10))
        m.add(k, extrude(BODY, [(PR[0] - 0.32, 0.12), (PR[0] + 0.18, 0.12), (PR[0] + 0.30, 0.30), (PR[0] + 0.16, 0.58),
                                (PR[0] - 0.35, 0.62)], 0.02).move(z=s * (PW + 0.01)))
        m.add(k, cyl_between('steel_dark', (PK[0], PK[1], s * 0.76), (PK[0], PK[1], s * 0.88), 0.035, 10))
    m.add(k, cyl_between('steel_dark', (*PR, -PW), (*PR, PW), 0.07, 16))
    for j in range(4):
        a = math.radians(45 + 90 * j)
        bx, by = PR[0] + 0.17 * math.cos(a), PR[1] + 0.17 * math.sin(a)
        m.add(k, cyl_between('steel', (bx, by, -PW + 0.02), (bx, by, PW - 0.02), 0.012, 6))
        n = 32
        for i in range(n):
            z = -PW + 0.06 + (2 * PW - 0.12) * i / (n - 1)
            ta = a - math.radians(25)
            m.add(k, box_between('steel', (bx, by, z), (bx + 0.13 * math.cos(ta), by + 0.13 * math.sin(ta), z), 0.006, 0.006))
    for i in range(17):
        z = -PW + 0.06 + (2 * PW - 0.12) * (i + 0.5) / 17
        m.add(k, extrude('steel_dark', band(0.205, 0.215, -80, 160, 12, PR), 0.04).move(z=z))
    m.add(k, extrude(BODY, band(0.24, 0.26, 95, 175, 12, PR) + [], 2 * PW))                     # top cover
    m.add(k, box_between(BODY, (PR[0] - 0.28, 0.58, -PW), (PR[0] - 0.28, 0.58, PW), 0.06, 0.05, (1, 0, 0)))
    # wind guard / crop roller on rods
    for s in (1, -1):
        m.add(k, cyl_between('steel', (PR[0] - 0.20, 0.60, s * 0.95), (PR[0] + 0.30, 0.62, s * 0.95), 0.015, 8))
    m.add(k, cyl_between('steel_dark', (PR[0] + 0.32, 0.55, -1.00), (PR[0] + 0.32, 0.55, 1.00), 0.06, 16))
    for i in range(12):
        z = -0.95 + 1.9 * i / 11
        m.add(k, cyl_between('steel', (PR[0] + 0.20, 0.60, z), (PR[0] + 0.42, 0.40, z), 0.006, 6))
    # stub augers feeding the edges toward the chamber
    for s in (1, -1):
        m.add(k, cyl_between('steel_dark', (PR[0] - 0.20, 0.45, s * 0.60), (PR[0] - 0.20, 0.45, s * (PW - 0.01)), 0.05, 12))
        m.add(k, auger_flight(BODY, (PR[0] - 0.20, 0.45, s * PW), (PR[0] - 0.20, 0.45, s * 0.62), 0.05, 0.15, 0.25,
                              hand=s, steps_per_turn=12))
    # gauge wheels on the pickup ends
    for s, side in ((1, 'right'), (-1, 'left')):
        c = (PR[0] - 0.10, 0.17, s * (PW + 0.16))
        m.add(k, box_between(FRAME, (PR[0] - 0.40, 0.45, s * (PW + 0.02)), (c[0], c[1] + 0.03, s * (PW + 0.02)), 0.03, 0.05))
        m.add(k, cyl_between(FRAME, (c[0], c[1], s * (PW + 0.02)), (c[0], c[1], s * (PW + 0.10)), 0.025, 10))
        g = m.node('pickup_gauge_wheel_%s_spin' % side, k, c)
        m.add(g, place(implement_tyre(0.17, 0.12, 0.10, ribs=2, segs=24) + rim(0.10, 0.12, 'steel', WHEEL, nuts=4), c, s))
    # pickup lift cylinder (left)
    m.add('chassis', box_span(FRAME, 0.78, 0.92, 1.05, 1.18, -(ZW + 0.08), -(ZW + 0.16)))
    m.add(k, box_span(FRAME, PR[0] - 0.10, PR[0] + 0.02, 0.58, 0.68, -0.86, -0.96))
    m.cylinder('pickup_lift_cylinder', 'chassis', k, (0.85, 1.10, -(ZW + 0.20)), (PR[0] - 0.04, 0.70, -(ZW + 0.20)), 0.035, 0.02, 0.55)


def build():
    m = Model('Round Baler', FARM_MATERIALS)
    m.node('chassis')
    body(m)
    tailgate(m)
    pickup(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'round-baler', 60000)
