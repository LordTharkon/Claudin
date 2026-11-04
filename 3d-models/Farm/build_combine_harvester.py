"""Combine harvester, class 7, on wheels, with a 9.1 m (30 ft) rigid grain header.

Key real-world dimensions used (metres):
  wheelbase 3.75, drive tyres 800/70R32 (1.95 dia), steer tyres 600/65R28 (1.49 dia)
  front track 2.84, grain tank ~10.5 m3, height to cab roof ~4.0
  header cut width 9.10, reel diameter 1.10, unloading auger ~6.5 m (stowed)
  length with header ~10.3, without ~7.8
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, ag_wheel, auger_flight, beacon, handrail, hose, lamp, ladder,
                        place, save_farm, step, tractor_cab)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from dozer_parts import exhaust, louvres

X_F, X_R = 0.0, -3.75
RF, WF, BF, ZF = 0.975, 0.80, 0.406, 1.42
RR, WR, BR, ZR = 0.745, 0.60, 0.356, 1.30
KP = 0.92
BHW = 0.85                      # body half width
CAB = dict(x0=1.00, x1=2.55, yf=2.35, yr=3.85, hw=0.82, ys=2.70, x_fender=1.00, hw_low=0.82)
FH_PIVOT = (1.00, 1.75)         # feederhouse pivot on the body
HX = 3.25                       # header back wall
HW = 4.55                       # header half width (9.10 m cut)


def wheels(m):
    p = 'chassis'
    front = ag_wheel(RF, WF, BF, lugs=26)
    rear = ag_wheel(RR, WR, BR, lugs=22)
    for s, side in ((1, 'right'), (-1, 'left')):
        sp = m.node('wheel_front_%s_spin' % side, p, (X_F, RF, s * ZF))
        m.add(sp, place(front, (X_F, RF, s * ZF), s))
        m.add(p, cyl_between(FRAME, (X_F, RF, s * 0.85), (X_F, RF, s * (ZF - 0.05)), 0.30, 22, r_top=0.26))  # final drive
        st = m.node('wheel_rear_%s_steer' % side, p, (X_R, RR, s * KP))
        m.add(st, cyl_between(FRAME, (X_R, RR - 0.15, s * KP), (X_R, RR + 0.15, s * KP), 0.08, 14))
        m.add(st, cyl_between(FRAME, (X_R, RR, s * KP), (X_R, RR, s * (ZR + 0.02)), 0.12, 18))
        m.add(st, cyl_between(FRAME, (X_R + 0.05, RR + 0.12, s * KP), (X_R + 0.32, RR + 0.10, s * (KP - 0.06)), 0.03, 8))
        rs = m.node('wheel_rear_%s_spin' % side, st, (X_R, RR, s * ZR))
        m.add(rs, place(rear, (X_R, RR, s * ZR), s))
    # drive axle housing and rear steering axle
    m.add(p, box_span(FRAME, -0.28, 0.28, 0.72, 1.25, -0.86, 0.86))
    m.add(p, box_span(FRAME, X_R - 0.12, X_R + 0.12, RR - 0.10, RR + 0.10, -KP + 0.08, KP - 0.08))
    m.add(p, box_span(FRAME, X_R - 0.25, X_R + 0.25, RR + 0.10, RR + 0.40, -0.22, 0.22))      # axle pivot
    m.add(p, cyl_between(BODY, (X_R + 0.32, RR + 0.10, -0.60), (X_R + 0.32, RR + 0.10, 0.30), 0.045, 12))
    m.add(p, cyl_between('chrome', (X_R + 0.32, RR + 0.10, 0.30), (X_R + 0.32, RR + 0.10, 0.78), 0.022, 10))
    for s in (1, -1):
        m.add(p, box_span(FRAME, X_R - 0.10, X_R + 0.10, RR - 0.16, RR + 0.16, s * (KP - 0.10), s * (KP - 0.02)))


def body(m):
    p = 'chassis'
    # main frame and separator body
    m.add(p, box_span(FRAME, -4.40, 0.90, 0.55, 0.95, -BHW + 0.05, BHW - 0.05))
    m.add(p, box_span(BODY, -4.40, 0.90, 0.95, 2.95, -BHW, BHW))
    m.add(p, box_span(BODY, 0.90, 1.10, 1.30, 2.25, -0.80, 0.80))        # feederhouse face plate
    for s in (1, -1):
        zs = s * (BHW + 0.002)
        # panel seams, door outlines and handles
        for x in (-3.05, -1.80, -0.55):
            m.add(p, box_span(FRAME, x, x + 0.012, 0.96, 2.94, zs - 0.003, zs + 0.003))
        for x0, x1 in ((-1.75, -0.65), (-2.95, -1.95)):
            for a, b in (((x0, 1.05), (x1, 1.07)), ((x0, 2.10), (x1, 2.12)), ((x0, 1.05), (x0 + 0.02, 2.12)), ((x1 - 0.02, 1.05), (x1, 2.12))):
                m.add(p, box_span('rubber_trim', a[0], b[0], a[1], b[1], zs - 0.004, zs + 0.004))
            m.add(p, box_span('steel_dark', x1 - 0.20, x1 - 0.08, 1.55, 1.59, zs, zs + s * 0.03))
        m.add(p, louvres(-0.45, 0.70, 1.15, 1.65, s * BHW, s, 0.06))
        # lower sieve / cleaning shoe side covers and rear lights
        m.add(p, box_span(FRAME, -4.40, -3.20, 0.60, 0.95, s * (BHW - 0.05), s * BHW))
        m.add(p, box_span(FRAME, -4.48, -4.40, 2.30, 2.52, s * 0.58, s * 0.80))
        m.add(p, box_span('lamp_red', -4.49, -4.48, 2.33, 2.42, s * 0.60, s * 0.70))
        m.add(p, box_span('lamp_amber', -4.49, -4.48, 2.42, 2.50, s * 0.60, s * 0.70))
        m.add(p, box_span('lamp_lens', -4.49, -4.48, 2.33, 2.49, s * 0.71, s * 0.78))
    # engine deck, rotary radiator screen (right), exhaust and intake
    m.add(p, box_span(BODY, -4.10, -1.90, 2.95, 3.45, -BHW, BHW))
    m.add(p, box_span(FRAME, -4.10, -1.90, 3.45, 3.47, -BHW + 0.05, BHW - 0.05))
    m.add(p, lathe('steel_dark', [(0, 0.0), (0.62, 0.0), (0.62, 0.06), (0.55, 0.07), (0, 0.07)], 32)
          .rotate('x', 90).at((-3.00, 2.30, BHW)))
    for k in range(8):
        a = math.radians(22.5 * k)
        m.add(p, box(FRAME, 1.12, 0.03, 0.02, (-3.00, 2.30, BHW + 0.085)).rotate('z', 22.5 * k, (-3.00, 2.30, 0)))
    m.add(p, cylinder(FRAME, 0.12, 0.05, 20).rotate('x', 90).at((-3.00, 2.30, BHW + 0.105)))
    m.add(p, exhaust(-2.30, 3.45, 4.05, 0.45, 0.075))
    m.add(p, cylinder(FRAME, 0.22, 0.30, 20).at((-3.60, 3.60, -0.40)))
    m.add(p, cylinder(FRAME, 0.24, 0.04, 20).at((-3.60, 3.77, -0.40)))
    for s in (1, -1):
        m.add(p, handrail([(-4.05, 3.47, s * 0.80), (-4.05, 4.00, s * 0.80), (-2.00, 4.00, s * 0.80), (-2.00, 3.47, s * 0.80)]))
        m.add(p, box_span(FRAME, -3.10, -3.07, 3.47, 4.00, s * 0.785, s * 0.815))
    # straw chopper hood and spreader at the rear
    m.add(p, extrude(BODY, [(-4.40, 1.00), (-4.85, 1.10), (-5.00, 1.55), (-4.70, 2.20), (-4.40, 2.30)], 1.30))
    m.add(p, box_span(FRAME, -4.95, -4.50, 0.85, 1.02, -0.62, 0.62))
    for s in (1, -1):
        m.add(p, cylinder('steel_dark', 0.32, 0.03, 24).at((-4.75, 0.80, s * 0.32)))
        for k in range(4):
            m.add(p, box(FRAME, 0.28, 0.05, 0.02, (-4.75 + 0.14 * math.cos(k * math.pi / 2), 0.84,
                                                      s * 0.32 + 0.14 * math.sin(k * math.pi / 2))).rotate('y', -90 * k, (-4.75, 0.84, s * 0.32)))
        m.add(p, box_span(FRAME, -4.80, -4.70, 0.82, 1.02, s * 0.30, s * 0.34))
    # rear ladder to the engine deck (right)
    m.add(p, ladder((-4.48, 1.10, 0.55), (-4.48, 3.45, 0.55), 0.38, 7))


def grain_tank(m):
    p = 'chassis'
    t = 'grain_tank'
    m.node(t, p)
    x0, x1, yb, yt, hw = -1.85, 0.92, 2.95, 3.75, 1.38
    # open-topped tank: side/bottom shell, end walls and a grain fill inside
    wt = 0.04
    outer = [(-BHW, yb - 0.30), (BHW, yb - 0.30), (hw, yb), (hw, yt), (-hw, yt), (-hw, yb)]
    shell = [(-BHW, yb - 0.30), (BHW, yb - 0.30), (hw, yb), (hw, yt), (hw - wt, yt), (hw - wt, yb + 0.015),
             (BHW - wt, yb - 0.26), (-BHW + wt, yb - 0.26), (-hw + wt, yb + 0.015), (-hw + wt, yt), (-hw, yt), (-hw, yb)]
    along = lambda g, xc: g.rotate('y', -90).move(x=xc)
    m.add(t, along(extrude(BODY, shell, x1 - x0 - 2 * wt), (x0 + x1) / 2))
    for xc in (x0 + wt / 2, x1 - wt / 2):
        m.add(t, along(extrude(BODY, outer, wt), xc))
    yg = yt - 0.32
    m.add(t, along(extrude('grain', [(-BHW + wt, yb - 0.26), (BHW - wt, yb - 0.26), (hw - wt, yb + 0.015), (hw - wt, yg),
                                    (-hw + wt, yg), (-hw + wt, yb + 0.015)], x1 - x0 - 2 * wt - 0.004), (x0 + x1) / 2))
    m.add(t, along(lathe('grain', [(0, -0.01), (0.95, -0.01), (0.95, 0.0), (0, 0.16)], 24).scale(1.0, 1.0, 1.3), (x0 + x1) / 2 - 0.1)
          .move(y=yg))
    for s in (1, -1):
        for x in (x0 + 0.70, x0 + 1.40, x0 + 2.10):
            m.add(t, box_span(BODY, x - 0.03, x + 0.03, yb - 0.02, yt - 0.02, s * hw, s * (hw + 0.03)))   # stiffening ribs
        m.add(t, box_span(FRAME, x0 + 0.03, x1 - 0.03, yt - 0.02, yt + 0.01, s * (hw - 0.05), s * (hw + 0.02)))  # rim
    m.add(t, box('glass', 0.012, 0.24, 0.50, (x1 + 0.006, yt - 0.25, 0)))      # sight window toward the cab
    # tank lids: fold-up covers hinged on the outer top edges
    for s, side in ((1, 'right'), (-1, 'left')):
        lid = m.node('grain_tank_lid_%s_tilt' % side, t, ((x0 + x1) / 2, yt + 0.02, s * (hw + 0.01)))
        m.add(lid, box_span(BODY, x0 + 0.05, x1 - 0.05, yt + 0.012, yt + 0.035, s * 0.01, s * (hw + 0.02)))
        for x in (x0 + 0.5, (x0 + x1) / 2, x1 - 0.5):
            m.add(lid, box_span(BODY, x - 0.02, x + 0.02, yt + 0.035, yt + 0.07, s * 0.04, s * (hw - 0.02)))
        for x in (x0 + 0.25, x1 - 0.25):
            m.add(lid, cyl_between('steel_dark', (x - 0.08, yt + 0.02, s * (hw + 0.01)), (x + 0.08, yt + 0.02, s * (hw + 0.01)), 0.018, 8))
    # unloading auger: vertical riser on the left, swivel, tube stowed forward along the side
    rx, rz = -1.55, -(hw + 0.16)
    m.add(t, cyl_between(BODY, (rx, 1.90, rz), (rx, 3.30, rz), 0.17, 20))
    m.add(t, box_between(BODY, (rx, 2.20, rz), (rx, 2.20, -BHW), 0.30, 0.35))
    m.add(t, cyl_between(FRAME, (rx, 3.28, rz), (rx, 3.36, rz), 0.21, 20))
    ua = m.node('unload_auger_rotate', t, (rx, 3.32, rz))
    elbow = (rx, 3.62, rz)
    tip = (3.15, 3.95, rz - 0.20)
    m.add(ua, cyl_between(BODY, (rx, 3.36, rz), elbow, 0.17, 20))
    m.add(ua, sphere(BODY, 0.17, 20, 10).at(elbow))
    m.add(ua, cyl_between(BODY, elbow, tip, 0.17, 20))
    for f in (0.25, 0.5, 0.75):
        c = tuple(elbow[i] + (tip[i] - elbow[i]) * f for i in range(3))
        m.add(ua, cyl_between(FRAME, (c[0] - 0.04, c[1], c[2]), (c[0] + 0.04, c[1], c[2]), 0.19, 20))
    m.add(ua, sphere(BODY, 0.17, 20, 10).at(tip))
    m.add(ua, cyl_between(BODY, tip, (tip[0] + 0.12, tip[1] - 0.45, tip[2]), 0.17, 20, r_top=0.14))
    m.add(ua, cyl_between('rubber_trim', (tip[0] + 0.12, tip[1] - 0.45, tip[2]), (tip[0] + 0.14, tip[1] - 0.60, tip[2]), 0.14, 18, r_top=0.12))
    m.add(ua, lamp((tip[0] + 0.05, tip[1] + 0.22, tip[2]), 1, 0.8))
    # auger support rail over the tube and the transport cradle on the cab platform
    m.add(ua, box_between(FRAME, (elbow[0] + 0.5, elbow[1] + 0.26, rz), (tip[0] - 0.4, tip[1] + 0.22, tip[2]), 0.05, 0.05))
    m.add(ua, box_between(FRAME, (elbow[0] + 0.5, elbow[1] + 0.03, rz), (elbow[0] + 0.5, elbow[1] + 0.26, rz), 0.04, 0.04, (1, 0, 0)))
    m.add(ua, box_between(FRAME, (tip[0] - 0.4, tip[1] - 0.05, tip[2]), (tip[0] - 0.4, tip[1] + 0.22, tip[2]), 0.04, 0.04, (1, 0, 0)))
    cx = 2.45
    cy = elbow[1] + (tip[1] - elbow[1]) * (cx - elbow[0]) / (tip[0] - elbow[0])
    cz = elbow[2] + (tip[2] - elbow[2]) * (cx - elbow[0]) / (tip[0] - elbow[0])
    m.add(p, box_span(FRAME, cx - 0.04, cx + 0.04, 2.35, cy - 0.20, cz - 0.04, cz + 0.04))
    m.add(p, extrude(FRAME, [(-0.22, 0.0), (0.22, 0.0), (0.22, 0.18), (0.15, 0.04), (-0.15, 0.04), (-0.22, 0.18)], 0.12)
          .rotate('y', 90).at((cx, cy - 0.22, cz)))


def cab_platform(m):
    c = m.node('cab', 'chassis')
    m.add(c, tractor_cab(**CAB, door_side=-1, sill_front=CAB['yf'] + 0.06, seat_x=1.75, wheel_tilt=30, door_x0=1.75))
    # platform, support and access ladder on the left
    m.add(c, box_span('steel_dark', 0.90, 2.75, 2.27, 2.33, -1.80, 1.02))
    for s in (1, -1):
        m.add(c, box_between(FRAME, (1.05, 1.60, s * 0.70), (2.40, 2.26, s * 0.75), 0.08, 0.10))
    m.add(c, handrail([(2.70, 2.33, -1.76), (2.70, 3.30, -1.76), (2.10, 3.30, -1.76)]))
    m.add(c, handrail([(1.05, 3.30, -1.76), (0.95, 3.30, -1.76), (0.95, 2.33, -1.76)]))
    m.add(c, handrail([(2.70, 2.33, 0.98), (2.70, 3.25, 0.98), (2.10, 3.25, 0.98), (2.10, 2.33, 0.98)]))
    m.add(c, handrail([(2.70, 3.30, -1.76), (2.70, 3.30, -0.95)]))
    m.add(c, ladder((1.92, 0.30, -1.86), (1.58, 2.27, -1.86), 0.45, 7, axis=(1, 0, 0)))
    m.add(c, handrail([(1.62, 1.10, -1.86), (1.32, 2.33, -1.86), (1.32, 3.30, -1.86), (1.32, 3.30, -1.76)]))
    m.add(c, handrail([(2.12, 1.10, -1.86), (1.85, 2.33, -1.86), (1.85, 3.30, -1.86), (1.85, 3.30, -1.76)]))
    for z in (-0.20, 0.20):
        m.add(c, lamp((2.67, 3.93, z), 1, 0.9))


def feederhouse_and_header(m):
    p = 'chassis'
    h = m.node('header_tilt', p, (*FH_PIVOT, 0))
    # feederhouse: inclined duct from the body to the header back wall
    a, b = FH_PIVOT, (HX, 0.82)
    m.add(h, box_between(BODY, (a[0] + 0.05, a[1] - 0.05, 0), (b[0], b[1], 0), 1.40, 0.62))
    m.add(h, cyl_between('steel_dark', (a[0], a[1], -0.76), (a[0], a[1], 0.76), 0.06, 14))
    for s in (1, -1):
        for dy in (-0.18, 0.18):   # side stiffeners along the duct
            m.add(h, box_between(BODY, (a[0] + 0.25, a[1] - 0.05 + dy, s * 0.715), (b[0] - 0.10, b[1] + dy, s * 0.715), 0.03, 0.05))
        m.add(h, box_between('steel_dark', (a[0] + 0.9, a[1] - 0.40, s * 0.705), (a[0] + 1.3, a[1] - 0.56, s * 0.705), 0.012, 0.30))  # inspection cover
        m.add(h, box_span(FRAME, 1.95, 2.10, 1.00, 1.18, s * 0.50, s * 0.60))      # lift cylinder lugs
    # header: back wall, top beam, floor pan, cutterbar, end sheets with crop dividers
    m.add(h, box_span(BODY, HX - 0.04, HX + 0.06, 0.20, 1.40, -HW, HW))
    m.add(h, box_span(BODY, HX - 0.20, HX + 0.06, 1.30, 1.48, -HW, HW))
    m.add(h, box_span(FRAME, HX - 0.20, HX - 0.04, 0.30, 1.30, -0.75, 0.75))     # adapter frame
    m.add(h, extrude(BODY, [(HX, 0.20), (HX + 0.30, 0.10), (4.85, 0.08), (4.85, 0.11), (HX + 0.30, 0.14), (HX + 0.04, 0.26)], 2 * HW))
    m.add(h, box_span('wear_steel', 4.82, 4.98, 0.07, 0.10, -HW + 0.02, HW - 0.02))
    n = int(2 * HW / 0.152)
    guard = [(0.0, -0.012), (0.20, 0.0), (0.0, 0.016)]
    for k in range(n):
        z = -HW + 2 * HW * (k + 0.5) / n
        m.add(h, extrude('steel', guard, 0.035).at((4.86, 0.085, z)))
    for s in (1, -1):
        end = [(HX - 0.20, 0.06), (4.95, 0.06), (5.40, 0.24), (4.70, 0.85), (HX + 0.40, 1.48), (HX - 0.20, 1.48)]
        m.add(h, extrude(BODY, end, 0.03).move(z=s * (HW + 0.015)))
        m.add(h, extrude(BODY, [(4.40, 0.10), (5.40, 0.24), (4.70, 0.85)], 0.10).move(z=s * (HW + 0.07)))
        m.add(h, box_span(FRAME, 4.10, 4.40, 0.02, 0.06, s * (HW - 0.05), s * (HW + 0.10)))     # skid shoe
        m.add(h, box_span(FRAME, HX + 0.10, HX + 0.18, 1.20, 1.30, s * (HW + 0.03), s * (HW + 0.10)))
        m.add(h, box_span('lamp_amber', HX + 0.18, HX + 0.19, 1.21, 1.29, s * (HW + 0.04), s * (HW + 0.09)))
    for z in (-3.0, -1.5, 1.5, 3.0):
        m.add(h, box_span(FRAME, 4.10, 4.40, 0.02, 0.07, z - 0.08, z + 0.08))
    # cross auger with left/right flighting and centre fingers
    ax, ay = HX + 0.45, 0.48
    m.add(h, cyl_between('steel_dark', (ax, ay, -HW + 0.05), (ax, ay, HW - 0.05), 0.19, 24))
    for s in (1, -1):
        m.add(h, auger_flight(BODY, (ax, ay, s * 0.75), (ax, ay, s * (HW - 0.10)), 0.18, 0.30, 0.60, hand=s, steps_per_turn=14))
        m.add(h, cylinder(FRAME, 0.22, 0.04, 20).rotate('x', 90).at((ax, ay, s * (HW - 0.03))))
    for k in range(6):
        a = math.radians(60 * k + 15)
        z = -0.55 + 0.22 * k
        m.add(h, cyl_between('steel', (ax, ay, z), (ax + 0.30 * math.cos(a), ay + 0.30 * math.sin(a), z), 0.012, 6))
    # hoses from the feederhouse to the header
    for z in (-0.55, -0.45):
        m.add(h, hose([(2.10, 1.62, z), (2.80, 1.38, z), (HX - 0.10, 1.38, z - 0.30), (HX - 0.10, 1.38, -HW + 0.4)], 0.014))
    # reel on lift arms
    arm = m.node('reel_arm_tilt', h, (HX - 0.05, 1.45, 0))
    rc = (4.55, 1.62)
    for z in (-HW + 0.08, 0.0, HW - 0.08):
        m.add(arm, box_between(BODY, (HX - 0.05, 1.45, z), (rc[0] + 0.10, rc[1], z), 0.08, 0.12))
        m.add(arm, cyl_between('steel_dark', (HX - 0.05, 1.45, z - 0.07), (HX - 0.05, 1.45, z + 0.07), 0.03, 10))
    for s, side in ((1, 'right'), (-1, 'left')):
        z = s * (HW - 0.08)
        m.cylinder('reel_lift_cylinder_' + side, h, arm, (HX + 0.10, 0.80, z), (4.00, 1.50, z), 0.04, 0.025, 0.55)
        m.add(h, box_span(BODY, HX + 0.06, HX + 0.14, 0.74, 0.86, z - 0.04, z + 0.04))
        m.add(arm, hose([(HX + 0.0, 1.50, z - s * 0.06), (3.85, 1.56, z - s * 0.06), (4.40, 1.66, z - s * 0.06)], 0.01))
    reel = m.node('reel_spin', arm, (*rc, 0))
    R = 0.55
    m.add(reel, cyl_between('steel_dark', (rc[0], rc[1], -HW + 0.10), (rc[0], rc[1], HW - 0.10), 0.08, 16))
    spiders = (-HW + 0.18, -2.25, -0.14, 0.14, 2.25, HW - 0.18)
    for z in spiders:
        m.add(reel, cylinder(FRAME, 0.14, 0.06, 12).rotate('x', 90).at((rc[0], rc[1], z)))
        for k in range(6):
            a = math.radians(60 * k)
            m.add(reel, box_between(BODY, (rc[0], rc[1], z), (rc[0] + (R - 0.02) * math.cos(a), rc[1] + (R - 0.02) * math.sin(a), z), 0.025, 0.05))
    for k in range(6):
        a = math.radians(60 * k)
        bx, by = rc[0] + R * math.cos(a), rc[1] + R * math.sin(a)
        m.add(reel, cyl_between('steel', (bx, by, -HW + 0.10), (bx, by, HW - 0.10), 0.022, 8))
        # tines hang roughly downward, pitched back a little
        tdir = (math.cos(a - math.radians(80)), math.sin(a - math.radians(80)))
        nt = int((2 * HW - 0.3) / 0.20)
        for j in range(nt):
            z = -HW + 0.25 + (2 * HW - 0.5) * j / (nt - 1)
            m.add(reel, box_between('plastic_black', (bx, by, z), (bx + 0.24 * tdir[0], by + 0.24 * tdir[1], z), 0.008, 0.012))
    # feederhouse lift cylinders from the drive axle
    for s, side in ((1, 'right'), (-1, 'left')):
        m.cylinder('feederhouse_lift_cylinder_' + side, p, h, (0.42, 1.05, s * 0.55), (2.02, 1.05, s * 0.55), 0.07, 0.045, 0.55)
        m.add(p, box_span(FRAME, 0.28, 0.48, 0.98, 1.12, s * 0.48, s * 0.62))
        m.add(p, hose([(0.40, 1.40, s * 0.40), (0.40, 1.20, s * 0.50), (0.48, 1.12, s * 0.63)], 0.014))


def build():
    m = Model('Combine Harvester', FARM_MATERIALS)
    m.node('chassis')
    wheels(m)
    body(m)
    grain_tank(m)
    cab_platform(m)
    feederhouse_and_header(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'combine-harvester', 60000)
