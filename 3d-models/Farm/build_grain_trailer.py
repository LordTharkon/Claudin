"""Tipping grain trailer, two axles (turntable front axle), ~14 t payload, rear tipping.

Key real-world dimensions used (metres):
  body 6.00 x 2.40 inside ~2.30, sides 1.00 + 0.50 extensions (~21 m3)
  wheelbase 3.40, track 1.84, tyres 445/45R19.5 flotation, front-mounted hoist ram
  overall ~7.6 long, 2.55 wide, 2.85 high
"""
import math

from farm_parts import BODY, FARM_MATERIALS, FRAME, WHEEL, hose, implement_tyre, ladder, place, save_farm
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from vehicle_parts import rim

X0, X1 = -3.00, 3.00          # body rear / front
YF = 1.30                     # body floor
HB = 1.00                     # side height
HE = 0.50                     # extension height
HWB = 1.20                    # body half width
AX_R, AX_F = -1.75, 1.65
RT, WT, BT, ZT = 0.50, 0.42, 0.248, 0.92


def wheel_geo():
    return implement_tyre(RT, WT, BT, ribs=6, segs=40) + rim(BT, WT, 'steel', WHEEL, nuts=8)


def chassis(m):
    p = 'chassis'
    for s in (1, -1):
        m.add(p, box_span(FRAME, -3.05, 3.20, 1.00, 1.24, s * 0.40, s * 0.50))
    for x in (-2.9, -1.75, -0.6, 0.5, 1.65, 2.9):
        m.add(p, box_span(FRAME, x - 0.05, x + 0.05, 1.05, 1.22, -0.40, 0.40))
    # rear axle on leaf springs, mudguards
    m.add(p, cyl_between(FRAME, (AX_R, RT, -ZT + 0.06), (AX_R, RT, ZT - 0.06), 0.065, 16))
    for s, side in ((1, 'right'), (-1, 'left')):
        for k in range(4):
            m.add(p, box_span('steel_dark', AX_R - 0.55 + 0.06 * k, AX_R + 0.55 - 0.06 * k, RT + 0.07 + 0.025 * k, RT + 0.095 + 0.025 * k,
                              s * 0.40, s * 0.50))
        m.add(p, box_span(FRAME, AX_R - 0.60, AX_R - 0.50, RT + 0.065, 1.00, s * 0.39, s * 0.51))
        m.add(p, box_span(FRAME, AX_R + 0.50, AX_R + 0.60, RT + 0.12, 1.00, s * 0.39, s * 0.51))
        w = m.node('wheel_rear_%s_spin' % side, p, (AX_R, RT, s * ZT))
        m.add(w, place(wheel_geo(), (AX_R, RT, s * ZT), s))
        m.add(p, mudguard(AX_R, s))
        m.add(p, box_between(FRAME, (AX_R, 1.02, s * 0.50), (AX_R, RT + 0.62, s * (ZT - 0.05)), 0.05, 0.05))
    # turntable ring
    m.add(p, cylinder(FRAME, 0.62, 0.06, 32).at((AX_F, 0.97, 0)))
    # rear: light bar, underrun guard, hinge brackets
    m.add(p, box_span(FRAME, -3.10, -3.02, 0.62, 0.74, -1.10, 1.10))
    for s in (1, -1):
        m.add(p, box_span(FRAME, -3.06, -2.96, 0.74, 1.05, s * 0.42, s * 0.50))
        m.add(p, box_span(FRAME, -3.13, -3.10, 0.60, 0.76, s * 0.85, s * 1.15))
        m.add(p, box_span('lamp_red', -3.14, -3.13, 0.63, 0.73, s * 0.86, s * 0.98))
        m.add(p, box_span('lamp_amber', -3.14, -3.13, 0.63, 0.73, s * 0.99, s * 1.07))
        m.add(p, box_span('lamp_lens', -3.14, -3.13, 0.63, 0.73, s * 1.08, s * 1.14))
        m.add(p, box_span(FRAME, -3.10, -2.98, 1.12, 1.30, s * 0.30, s * 0.40))
        m.add(p, cyl_between('steel_dark', (-3.00, 1.27, s * 0.26), (-3.00, 1.27, s * 0.55), 0.035, 12))
    # ram base cradle at the front of the chassis
    m.add(p, box_span(FRAME, 3.05, 3.40, 0.95, 1.10, -0.25, 0.25))
    for s in (1, -1):
        m.add(p, box_span(FRAME, 3.15, 3.30, 1.10, 1.25, s * 0.14, s * 0.22))
    m.add(p, cyl_between('steel_dark', (3.22, 1.17, -0.24), (3.22, 1.17, 0.24), 0.03, 12))


def mudguard(x, s):
    out = [extrude('plastic_black', [(x - 0.55, RT + 0.05), (x - 0.45, RT + 0.45), (x - 0.25, RT + 0.58), (x + 0.25, RT + 0.58),
                                      (x + 0.45, RT + 0.45), (x + 0.55, RT + 0.05), (x + 0.55, RT + 0.10), (x + 0.47, RT + 0.48),
                                      (x + 0.27, RT + 0.62), (x - 0.27, RT + 0.62), (x - 0.47, RT + 0.48), (x - 0.55, RT + 0.10)],
                   WT + 0.12).move(z=s * ZT)]
    out.append(box_span('lamp_amber', x - 0.04, x + 0.04, RT + 0.52, RT + 0.58, s * (ZT + WT / 2 + 0.06), s * (ZT + WT / 2 + 0.065)))
    return out


def front_axle(m):
    fa = m.node('front_axle_steer', 'chassis', (AX_F, 0.97, 0))
    m.add(fa, cylinder(FRAME, 0.60, 0.05, 32).at((AX_F, 0.915, 0)))
    m.add(fa, box_span(FRAME, AX_F - 0.50, AX_F + 0.50, 0.70, 0.89, -0.50, 0.50))
    m.add(fa, cyl_between(FRAME, (AX_F, RT, -ZT + 0.06), (AX_F, RT, ZT - 0.06), 0.065, 16))
    for s, side in ((1, 'right'), (-1, 'left')):
        for k in range(4):
            m.add(fa, box_span('steel_dark', AX_F - 0.45 + 0.06 * k, AX_F + 0.45 - 0.06 * k, RT + 0.07 + 0.025 * k, RT + 0.095 + 0.025 * k,
                               s * 0.40, s * 0.50))
        m.add(fa, box_span(FRAME, AX_F - 0.50, AX_F - 0.40, RT + 0.065, 0.70, s * 0.39, s * 0.51))
        m.add(fa, box_span(FRAME, AX_F + 0.40, AX_F + 0.50, RT + 0.12, 0.70, s * 0.39, s * 0.51))
        w = m.node('wheel_front_%s_spin' % side, fa, (AX_F, RT, s * ZT))
        m.add(w, place(wheel_geo(), (AX_F, RT, s * ZT), s))
        m.add(fa, mudguard(AX_F, s))
        m.add(fa, box_between(FRAME, (AX_F, 0.85, s * 0.50), (AX_F, RT + 0.62, s * (ZT - 0.05)), 0.05, 0.05))
        m.add(fa, box_span(FRAME, AX_F + 0.50, AX_F + 0.70, 0.62, 0.80, s * 0.30, s * 0.40))
    # A-frame drawbar hinged on the axle frame, supported on its stand
    pv = (AX_F + 0.62, 0.72)
    d = m.node('drawbar_tilt', fa, (*pv, 0))
    eye = (4.40, 0.52)
    for s in (1, -1):
        m.add(d, box_between(FRAME, (pv[0], pv[1], s * 0.35), (eye[0] - 0.55, eye[1] + 0.02, s * 0.04), 0.08, 0.10))
        m.add(d, cyl_between('steel_dark', (pv[0], pv[1], s * 0.28), (pv[0], pv[1], s * 0.42), 0.04, 12))
    m.add(d, box_between(FRAME, (eye[0] - 0.60, eye[1] + 0.02, 0), (eye[0] - 0.06, eye[1], 0), 0.08, 0.08))
    m.add(d, lathe('steel', [(0.03, -0.025), (0.065, -0.025), (0.065, 0.025), (0.03, 0.025), (0.03, -0.025)], 16).at((*eye, 0)))
    sx = 3.30
    sy = pv[1] + (eye[1] - pv[1]) * (sx - pv[0]) / (eye[0] - pv[0])
    m.add(d, box_between('steel_dark', (sx, sy - 0.05, 0.0), (sx + 0.05, 0.02, 0.0), 0.06, 0.06, (1, 0, 0)))
    m.add(d, box_span('steel_dark', sx - 0.10, sx + 0.20, 0.0, 0.02, -0.10, 0.10))
    # brake / hydraulic lines with coupling heads along the drawbar
    for z, head in ((0.10, 'paint_red'), (0.16, 'paint_yellow'), (-0.12, 'steel'), (-0.18, 'steel')):
        m.add(d, hose([(pv[0] + 0.10, pv[1] + 0.06, z * 2), (3.40, 0.68, z), (eye[0] - 0.40, 0.62, z), (eye[0] - 0.25, 0.70, z)], 0.010))
        m.add(d, cylinder(head, 0.025, 0.05, 10).rotate('z', 90).at((eye[0] - 0.22, 0.70, z)))


def body(m):
    b = m.node('body_tilt', 'chassis', (X0, 1.27, 0))
    t = 0.02
    # floor, sub-frame runners, sides, front bulkhead
    m.add(b, box_span(BODY, X0, X1, YF, YF + 0.05, -HWB, HWB))
    for s in (1, -1):
        m.add(b, box_span(FRAME, X0, X1, YF - 0.12, YF, s * 0.38, s * 0.52))
    for x in (-2.4, -1.2, 0.0, 1.2, 2.4):
        m.add(b, box_span(FRAME, x - 0.04, x + 0.04, YF - 0.10, YF, -HWB, HWB))
    ytop = YF + HB + HE
    for s in (1, -1):
        m.add(b, box_span(BODY, X0, X1, YF + 0.05, YF + HB, s * (HWB - t), s * HWB))
        m.add(b, box_span(BODY, X0, X1, YF + HB + 0.02, ytop, s * (HWB - t), s * HWB))
        m.add(b, box_span(BODY, X0, X1, YF + HB - 0.05, YF + HB + 0.05, s * HWB, s * (HWB + 0.05)))     # mid rail
        m.add(b, box_span(BODY, X0, X1, ytop - 0.05, ytop, s * HWB, s * (HWB + 0.05)))                  # top rail
        for k in range(7):
            x = X0 + 0.05 + (X1 - X0 - 0.10) * k / 6
            m.add(b, box_span(BODY, x - 0.04, x + 0.04, YF + 0.05, YF + HB - 0.05, s * HWB, s * (HWB + 0.04)))
            m.add(b, box_span(BODY, x - 0.03, x + 0.03, YF + HB + 0.05, ytop - 0.05, s * HWB, s * (HWB + 0.035)))
        m.add(b, box_span(FRAME, X0 + 0.08, X1 - 0.08, YF - 0.04, YF + 0.04, s * (HWB - 0.01), s * (HWB + 0.03)))
        m.add(b, box_span('lamp_amber', 0.0 - 0.04, 0.0 + 0.04, YF, YF + 0.05, s * (HWB + 0.03), s * (HWB + 0.035)))
    m.add(b, box_span(BODY, X1 - t, X1, YF + 0.05, ytop + 0.25, -HWB, HWB))
    m.add(b, box_span(BODY, X1, X1 + 0.05, ytop - 0.05, ytop + 0.25, -HWB, HWB))
    for z in (-0.8, -0.4, 0.4, 0.8):
        m.add(b, box_span(BODY, X1, X1 + 0.04, YF + 0.05, ytop, z - 0.04, z + 0.04))
    m.add(b, box_span('glass', X1 - 0.03, X1 + 0.06, YF + HB + 0.10, YF + HB + 0.40, -0.25, 0.25))   # sight window
    m.add(b, ladder((X1 + 0.12, YF + 0.10, -0.95), (X1 + 0.12, ytop + 0.20, -0.95), 0.40, 6))
    # ram top bracket on the bulkhead
    m.add(b, box_span(FRAME, X1, X1 + 0.18, YF + 0.95, YF + 1.15, -0.20, 0.20))
    m.add(b, cyl_between('steel_dark', (X1 + 0.12, YF + 1.05, -0.22), (X1 + 0.12, YF + 1.05, 0.22), 0.03, 12))
    # rear hinge lugs
    for s in (1, -1):
        m.add(b, box_span(FRAME, X0 - 0.06, X0 + 0.10, 1.22, YF, s * 0.30, s * 0.40))
    # tailgate hinged at the top, with a grain chute hatch that slides up
    tg = m.node('tailgate_tilt', b, (X0 - 0.06, ytop - 0.02, 0))
    m.add(tg, box_span(BODY, X0 - 0.05, X0 - 0.02, YF + 0.05, ytop - 0.04, -HWB + 0.03, HWB - 0.03))
    for z in (-0.85, -0.3, 0.3, 0.85):
        m.add(tg, box_span(BODY, X0 - 0.09, X0 - 0.05, YF + 0.05, ytop - 0.06, z - 0.04, z + 0.04))
    for y in (YF + 0.08, YF + HB, ytop - 0.08):
        m.add(tg, box_span(BODY, X0 - 0.09, X0 - 0.05, y - 0.04, y + 0.04, -HWB + 0.03, HWB - 0.03))
    for s in (1, -1):
        m.add(tg, cyl_between('steel_dark', (X0 - 0.06, ytop - 0.02, s * (HWB - 0.12)), (X0 - 0.06, ytop - 0.02, s * (HWB - 0.02)), 0.025, 10))
        m.add(tg, box_span(FRAME, X0 - 0.10, X0 - 0.04, YF + 0.10, YF + 0.24, s * (HWB - 0.15), s * (HWB - 0.045)))   # locks
    chute = m.node('grain_chute_slide', tg, (X0 - 0.11, YF + 0.30, 0))
    m.add(tg, box_span(FRAME, X0 - 0.11, X0 - 0.09, YF + 0.12, YF + 0.62, -0.22, -0.18))
    m.add(tg, box_span(FRAME, X0 - 0.11, X0 - 0.09, YF + 0.12, YF + 0.62, 0.18, 0.22))
    m.add(chute, box_span(BODY, X0 - 0.125, X0 - 0.105, YF + 0.14, YF + 0.46, -0.18, 0.18))
    m.add(chute, cyl_between('steel', (X0 - 0.13, YF + 0.42, -0.08), (X0 - 0.17, YF + 0.42, -0.08), 0.01, 6))
    m.add(chute, cyl_between('steel', (X0 - 0.13, YF + 0.42, 0.08), (X0 - 0.17, YF + 0.42, 0.08), 0.01, 6))
    m.add(chute, cyl_between('steel', (X0 - 0.17, YF + 0.42, -0.09), (X0 - 0.17, YF + 0.42, 0.09), 0.012, 6))
    # hoist: ram at the front, pinned to the chassis and the bulkhead bracket
    m.cylinder('hoist_cylinder', 'chassis', b, (3.22, 1.17, 0), (X1 + 0.12, YF + 1.05, 0), 0.10, 0.07, 0.45, barrel=FRAME)
    m.add('chassis', hose([(3.30, 1.10, 0.15), (3.45, 0.90, 0.30), (3.60, 0.75, 0.25)], 0.012))


def build():
    m = Model('Tipping Grain Trailer', FARM_MATERIALS)
    m.node('chassis')
    chassis(m)
    front_axle(m)
    body(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'grain-trailer', 60000)
