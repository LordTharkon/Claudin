"""Row-crop tractor, ~250 hp class, MFWD with cab, dual rear wheels and front weights.

Key real-world dimensions used (metres):
  wheelbase 3.05, front tyres 420/85R34 (1.55 dia), rear 480/80R50 duals (2.05 dia)
  rear inner track 1.90, dual spacing 0.80, height to cab roof ~3.25
  overall length with front weights ~6.6, width over the duals ~4.0
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, ag_wheel, beacon, handrail, hose, place, save_farm, step,
                        three_point_hitch, tractor_cab, tractor_hood)
from meshkit import Model, box_between, box_span, cyl_between, cylinder, extrude, sphere
from dozer_parts import exhaust, precleaner
from vehicle_parts import arch_polygon

AX_F, AX_R = 1.55, -1.50
RF, WF, BF, ZF = 0.775, 0.42, 0.432, 0.98
RR, WR, BR, ZR, ZD = 1.025, 0.48, 0.635, 0.95, 1.75
KP = 0.76
CAB = dict(x0=-1.95, x1=0.05, yf=1.45, yr=3.00, hw=0.82, ys=2.17, x_fender=-0.40, hw_low=0.62)


def chassis(m):
    p = 'chassis'
    front = ag_wheel(RF, WF, BF, lugs=22)
    rear = ag_wheel(RR, WR, BR, lugs=26)
    for s, side in ((1, 'right'), (-1, 'left')):
        st = m.node('wheel_front_%s_steer' % side, p, (AX_F, RF, s * KP))
        m.add(st, cyl_between(FRAME, (AX_F, RF - 0.17, s * KP), (AX_F, RF + 0.17, s * KP), 0.09, 16))
        m.add(st, cyl_between(FRAME, (AX_F, RF, s * KP), (AX_F, RF, s * (ZF + 0.05)), 0.15, 20))
        m.add(st, cyl_between(FRAME, (AX_F, RF, s * (ZF - 0.05)), (AX_F, RF, s * (ZF + 0.09)), 0.21, 20))  # planetary hub
        m.add(st, extrude(BODY, arch_polygon(AX_F, RF, RF + 0.06, RF + 0.09, 20, 150, 16), 0.52).move(z=s * ZF))
        m.add(st, box_between(FRAME, (AX_F - 0.05, RF + 0.15, s * KP), (AX_F - 0.05, RF + 0.80, s * (ZF - 0.18)), 0.06, 0.04))
        m.add(st, box_span(FRAME, AX_F - 0.12, AX_F + 0.02, RF + 0.76, RF + 0.85, s * (ZF - 0.28), s * (ZF - 0.16)))
        m.add(st, lamp_pair(AX_F - 0.30, RF + 0.84, s * (ZF + 0.10)))
        sp = m.node('wheel_front_%s_spin' % side, st, (AX_F, RF, s * ZF))
        m.add(sp, place(front, (AX_F, RF, s * ZF), s))
        # rear: inner wheel + outer dual on a bar axle, one spin node per side
        rs = m.node('wheel_rear_%s_spin' % side, p, (AX_R, RR, s * ZR))
        m.add(rs, place(rear, (AX_R, RR, s * ZR), s))
        m.add(rs, place(rear, (AX_R, RR, s * ZD), s))
        m.add(rs, cyl_between('steel', (AX_R, RR, s * (ZR + 0.06)), (AX_R, RR, s * (ZD + 0.12)), 0.06, 18))
        m.add(rs, cyl_between('steel_dark', (AX_R, RR, s * (ZD + 0.02)), (AX_R, RR, s * (ZD + 0.14)), 0.13, 18))
        m.add(rs, cyl_between('steel_dark', (AX_R, RR, s * (ZR + 0.02)), (AX_R, RR, s * (ZR + 0.14)), 0.13, 18))
        m.add(p, cyl_between(FRAME, (AX_R, RR, s * 0.36), (AX_R, RR, s * 0.80), 0.15, 20, r_top=0.12))
        m.add(p, cyl_between(FRAME, (AX_R, RR, s * 0.80), (AX_R, RR, s * (ZR + 0.03)), 0.17, 20))
    # front axle with steering cylinders and drive shaft
    m.add(p, cyl_between(FRAME, (AX_F, RF, -KP + 0.08), (AX_F, RF, KP - 0.08), 0.095, 18))
    m.add(p, sphere(FRAME, 0.22, 18, 9).scale(1, 0.9, 1).at((AX_F, RF, 0.10)))
    m.add(p, box_span(FRAME, AX_F - 0.30, AX_F + 0.30, RF + 0.08, RF + 0.30, -0.25, 0.25))   # oscillation pivot
    for s in (1, -1):
        m.add(p, box_span(FRAME, AX_F - 0.12, AX_F + 0.12, RF - 0.16, RF + 0.16, s * (KP - 0.09), s * (KP - 0.02)))
        m.add(p, cyl_between(BODY, (AX_F + 0.30, RF + 0.05, s * 0.25), (AX_F + 0.22, RF + 0.08, s * 0.55), 0.04, 12))
        m.add(p, cyl_between('chrome', (AX_F + 0.22, RF + 0.08, s * 0.55), (AX_F + 0.18, RF + 0.10, s * 0.70), 0.02, 10))
    m.add(p, cyl_between('steel_dark', (0.10, 0.70, 0.10), (AX_F - 0.15, RF - 0.05, 0.10), 0.045, 12))
    # engine frame, hood, exhaust and air intake
    m.add(p, box_span(FRAME, 0.07, 2.80, 0.68, 1.12, -0.36, 0.36))
    m.add(p, box_span(FRAME, 2.70, 2.95, 0.70, 1.10, -0.40, 0.40))                   # weight bracket
    m.add(p, tractor_hood(0.05, 2.80, 1.10, 1.92, 1.80, 0.42, vents=(1.90, 2.55)))
    m.add(p, exhaust(0.28, 1.92, 3.30, 0.26, 0.065))
    m.add(p, cyl_between('steel_dark', (0.28, 2.05, 0.26), (0.28, 2.70, 0.26), 0.11, 18))
    m.add(p, precleaner(0.28, 1.92, -0.26, 0.45))
    # front suitcase weights
    n = 10
    for k in range(n):
        z = -0.42 + 0.84 * (k + 0.5) / n
        m.add(p, extrude('paint_gray_dark', [(2.95, 0.66), (3.30, 0.66), (3.34, 0.72), (3.34, 1.10), (3.24, 1.18),
                                              (3.10, 1.18), (3.04, 1.14), (2.95, 1.14)], 0.074).move(z=z))
        m.add(p, box_span('paint_gray_dark', 3.10, 3.24, 1.18, 1.25, z - 0.02, z + 0.02))
    m.add(p, cyl_between('steel', (3.17, 1.215, -0.46), (3.17, 1.215, 0.46), 0.022, 10))
    for s in (1, -1):
        m.add(p, box_span(FRAME, 2.95, 3.30, 0.60, 0.66, s * 0.34, s * 0.44))
    # transmission / rear housing, fuel tank, battery box
    m.add(p, box_span(FRAME, -2.00, 0.07, 0.55, 1.42, -0.38, 0.38))
    m.add(p, box_span(FRAME, -1.80, -1.20, 0.75, 1.30, -0.50, 0.50))
    m.add(p, extrude(BODY, [(-0.38, 0.70), (0.30, 0.70), (0.40, 0.80), (0.40, 1.35), (-0.38, 1.35)], 0.30).move(z=0.53))
    m.add(p, cyl_between('steel_dark', (0.10, 1.35, 0.55), (0.10, 1.42, 0.55), 0.05, 12))
    m.add(p, box_span(FRAME, 0.10, 0.40, 0.75, 1.05, -0.68, -0.40))
    # rear fenders (over the inner wheels) with lights
    for s in (1, -1):
        m.add(p, extrude(BODY, arch_polygon(AX_R, RR, RR + 0.08, RR + 0.12, 15, 168, 24), 0.62).move(z=s * ZR))
        arc = [(AX_R + 1.12 * math.cos(math.radians(a)), RR + 1.12 * math.sin(math.radians(a))) for a in range(20, 165, 18)]
        m.add(p, extrude(BODY, arc, 0.015).move(z=s * (ZR - 0.31 - 0.0076)))
        x, y = AX_R - 1.13 * math.cos(math.radians(25)), RR + 1.13 * math.sin(math.radians(25))
        m.add(p, box_span(FRAME, x - 0.07, x, y - 0.12, y + 0.12, s * (ZR + 0.05), s * (ZR + 0.27)))
        m.add(p, box_span('lamp_red', x - 0.085, x - 0.07, y - 0.10, y, s * (ZR + 0.07), s * (ZR + 0.17)))
        m.add(p, box_span('lamp_amber', x - 0.085, x - 0.07, y, y + 0.10, s * (ZR + 0.07), s * (ZR + 0.17)))
        m.add(p, box_span('lamp_lens', x - 0.085, x - 0.07, y - 0.10, y + 0.02, s * (ZR + 0.18), s * (ZR + 0.25)))
    # rear: PTO, drawbar, hydraulic remotes, hitch with assist cylinders
    m.add(p, cyl_between('steel', (-2.01, 0.80, 0), (-2.20, 0.80, 0), 0.022, 12))
    m.add(p, cyl_between('steel_dark', (-2.00, 0.80, 0), (-2.05, 0.80, 0), 0.07, 14))
    m.add(p, box_span(BODY, -2.24, -2.00, 0.92, 0.94, -0.20, 0.20))
    for s in (1, -1):
        m.add(p, box_span(BODY, -2.24, -2.00, 0.76, 0.94, s * 0.18, s * 0.20))
    m.add(p, box_span(FRAME, -2.55, -1.20, 0.48, 0.54, -0.07, 0.07))
    m.add(p, cyl_between('steel', (-2.45, 0.40, 0), (-2.45, 0.60, 0), 0.025, 10))
    m.add(p, box_span(FRAME, -2.25, -2.00, 1.20, 1.38, -0.44, 0.44))       # rockshaft housing
    m.add(p, box_span(FRAME, -2.10, -2.00, 0.95, 1.20, -0.08, 0.08))       # top-link bracket
    m.add(p, box_span(FRAME, -2.12, -2.00, 1.38, 1.48, -0.42, 0.42))       # remote valve block
    for k in range(4):
        z = -0.30 + 0.20 * k
        m.add(p, cyl_between('steel', (-2.12, 1.43, z), (-2.22, 1.43, z), 0.022, 10))
        m.add(p, cylinder('lamp_red' if k % 2 else 'lamp_amber', 0.026, 0.02, 10).rotate('z', 90).at((-2.23, 1.43, z)))
    for s in (1, -1):
        m.add(p, box_span(FRAME, -2.15, -1.90, 0.52, 0.70, s * 0.46, s * 0.56))   # assist cylinder anchors
    three_point_hitch(m, p, -1.85, 0.70, 0.40, 1.00, 0.62, 0.52, (-2.10, 1.30, 0.40), 0.60,
                      (-2.05, 1.08, 0), (-2.85, 1.18, 0), assist=((-2.02, 0.62, 0.51), 0.62), ball_r=0.06)


def lamp_pair(x, y, z):
    """Small front-fender marker light."""
    return [box_span(FRAME, x - 0.04, x + 0.04, y - 0.03, y + 0.03, z - 0.04, z + 0.04),
            box_span('lamp_amber', x + 0.04, x + 0.05, y - 0.02, y + 0.02, z - 0.03, z + 0.03)]


def cab(m):
    c = m.node('cab', 'chassis')
    m.add(c, tractor_cab(**CAB, door_side=-1, wheel_tilt=35))
    # entry ladder on the left: three steps, rails both sides
    xs0, xs1 = -0.36, -0.02
    for k, y in enumerate((0.42, 0.78, 1.12)):
        m.add(c, step(xs0, xs1, y, -0.62, -0.92))
    for x in (xs0 - 0.02, xs1 + 0.02):
        m.add(c, box_span(FRAME, x - 0.012, x + 0.012, 0.40, 1.42, -0.93, -0.89))
    m.add(c, handrail([(0.09, 1.60, -0.83), (0.12, 1.60, -0.90), (0.12, 2.60, -0.90), (0.09, 2.60, -0.83)]))
    m.add(c, handrail([(xs0 - 0.04, 0.90, -0.92), (xs0 - 0.04, 1.75, -0.92), (xs0 + 0.01, 1.80, -0.85)]))
    m.add(c, handrail([(0.09, 1.60, 0.83), (0.12, 1.60, 0.90), (0.12, 2.60, 0.90), (0.09, 2.60, 0.83)]))
    # extra roof work lights
    for z in (-0.25, 0.25):
        from farm_parts import lamp
        m.add(c, lamp((0.17, 3.07, z), 1, 0.9))


def build():
    m = Model('Row-Crop Tractor', FARM_MATERIALS)
    m.node('chassis')
    chassis(m)
    cab(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'row-crop-tractor', 60000)
