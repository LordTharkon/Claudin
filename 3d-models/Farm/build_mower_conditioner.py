"""Trailed centre-pivot disc mower conditioner, 3.5 m cut.

Key real-world dimensions used (metres):
  3.5 m cutterbar with 8 oval discs (2 knives each), rubber chevron conditioner rolls
  hitch eye 0.51 above the ground (the row-crop tractor's drawbar height)
  10.0/75-15.3 wheels, track 4.14; overall ~4.9 long, 4.4 wide, 2.0 high

Rest pose: transport. Drawbar straight ahead on `drawbar_rotate` (centre pivot, about
y; swung by `drawbar_swing_cylinder`), header lifted 13 deg on `header_tilt` (pivot on
the frame legs, about z) by `header_lift_cylinder_*`. Side guards open on
`side_guard_{left,right}_tilt` (hinged along their top edge).
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, WHEEL, coupling, hazard_board, hitch_eye, hose, implement_tyre,
                        parking_jack, place, rear_light, reflector, save_farm, smv_panel)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude
from vehicle_parts import rim

XF = -3.30                      # frame cross beam
YF = 1.55
ZL = 1.90                       # frame legs
PV = (-3.05, 1.10)              # header pivot (x, y)
XC = -1.85                      # cutterbar front
HZ = 1.80                       # header half width
RT, WT, BT = 0.40, 0.26, 0.19
XWH = -3.55
EX = 0.90                       # hitch eye ahead of the design origin


def frame(m):
    p = 'frame'
    m.add(p, box_span(BODY, XF - 0.10, XF + 0.10, YF - 0.20, YF, -ZL - 0.06, ZL + 0.06))
    for s in (1, -1):
        m.add(p, box_between(BODY, (XF, YF - 0.10, s * ZL), (XWH, RT + 0.12, s * ZL), 0.14, 0.16, (0, 0, 1)))
        m.add(p, box_span(FRAME, XWH - 0.10, XWH + 0.10, RT - 0.06, RT + 0.14, s * (ZL - 0.09), s * (ZL + 0.09)))
        side = 'right' if s > 0 else 'left'
        c = (XWH, RT, s * (ZL + 0.17))
        m.add(p, cyl_between(FRAME, (XWH, RT, s * (ZL + 0.09)), (XWH, RT, s * (ZL + 0.12)), 0.05, 12))
        w = m.node('wheel_%s_spin' % side, p, c)
        m.add(w, place(implement_tyre(RT, WT, BT, ribs=4, segs=28) + rim(BT, WT, 'steel', WHEEL, nuts=6), c, s))
        # header pivot brackets, suspension springs, lift cylinder anchors
        m.add(p, box_span(FRAME, PV[0] - 0.08, PV[0] + 0.08, PV[1] - 0.08, YF - 0.20, s * (HZ - 0.10), s * (HZ - 0.02)))
        m.add(p, box_span(FRAME, XF - 0.06, XF + 0.06, YF - 0.30, YF - 0.20, s * 1.20, s * 1.30))
        m.add(p, box_span(FRAME, XF + 0.10, XF + 0.26, YF - 0.12, YF - 0.02, s * 0.85, s * 0.95))
        for k in range(8):
            y = YF - 0.32 - 0.04 * k
            m.add(p, cyl_between('steel', (XF + 0.02, y, s * 1.21), (XF + 0.02, y - 0.02, s * 1.29), 0.006, 6))
        # rear lamps on the legs, warning boards, reflectors
        lx = XF - 0.12
        m.add(p, box_span(FRAME, lx - 0.04, lx, YF - 0.60, YF - 0.20, s * (ZL - 0.035), s * (ZL + 0.035)))
        m.add(p, rear_light((lx - 0.04, YF - 0.30, s * ZL), -1))
        m.add(p, hazard_board((lx - 0.025, YF - 0.62, s * ZL), 0.10, 0.40, facing=-1, stripes=3))
        m.add(p, hazard_board((XF + 0.115, YF - 0.10, s * 1.55), 0.26, 0.16, facing=1, stripes=3))
        m.add(p, reflector((XF, YF - 0.10, s * (ZL + 0.06)), s, 'z', 'lamp_amber', 0.06))
    m.add(p, box_span(FRAME, XF - 0.14, XF - 0.10, YF - 0.05, YF + 0.35, -0.05, 0.05))
    m.add(p, smv_panel((XF - 0.145, YF + 0.20, 0.0), 0.34))
    # centre-pivot tower and swivel gearbox
    m.add(p, box_span(FRAME, XF + 0.10, XF + 0.50, YF - 0.20, YF + 0.05, -0.25, 0.25))
    m.add(p, cylinder('steel_dark', 0.16, 0.08, 20).at((XF + 0.35, YF + 0.09, 0)))


def drawbar(m):
    px = XF + 0.35
    d = m.node('drawbar_rotate', 'frame', (px, YF + 0.13, 0))
    eye = (EX, 0.51, 0.0)
    m.add(d, cylinder(FRAME, 0.15, 0.10, 20).at((px, YF + 0.18, 0)))
    m.add(d, box_between(BODY, (px, YF + 0.18, 0), (-0.90, 1.25, 0), 0.16, 0.18))
    m.add(d, box_between(BODY, (-0.92, 1.26, 0), (0.65, 0.58, 0), 0.14, 0.16))
    m.add(d, box_between(BODY, (0.60, 0.58, 0), (0.85, 0.53, 0), 0.10, 0.10))
    m.add(d, hitch_eye(eye))
    # front gearbox and PTO driveline in a plastic guard
    m.add(d, box_span('steel_dark', 0.20, 0.45, 0.76, 0.98, -0.12, 0.12))
    m.add(d, cyl_between('steel', (0.45, 0.86, 0.0), (0.62, 0.80, 0.0), 0.022, 10))
    m.add(d, cyl_between('plastic_black', (0.47, 0.85, 0.0), (0.60, 0.80, 0.0), 0.06, 14))
    m.add(d, cyl_between('steel', (0.20, 0.86, 0.0), (px - 0.30, YF + 0.10, 0.0), 0.025, 10))
    m.add(d, cyl_between('plastic_black', (0.10, 0.90, 0.0), (px - 0.40, YF + 0.10, 0.0), 0.055, 14))
    # jack, hose support, hydraulic and lighting lines
    m.add(d, box_span(FRAME, -0.05, 0.10, 0.60, 0.86, 0.08, 0.20))
    m.add(d, parking_jack(0.02, 0.22, 0.0, 0.92, crank=1))
    m.add(d, box_span(FRAME, -0.30, -0.20, 1.02, 1.30, -0.15, -0.08))
    for k, z in enumerate((-0.14, -0.20, -0.26)):
        m.add(d, hose([(px, YF + 0.22, -0.10), (-0.25, 1.32, z), (0.50, 0.95, z), (0.70, 0.85, z)], 0.010 if k < 2 else 0.006))
        if k < 2:
            m.add(d, coupling((0.74, 0.85, z), 'lamp_red' if k == 0 else 'lamp_amber'))
    m.add(d, reflector((0.60, 0.66, 0.0), 1, 'x', 'lamp_lens', 0.07))
    # swing cylinder from the frame to an arm on the drawbar
    m.add(d, box_span(FRAME, -2.55, -2.43, YF + 0.05, YF + 0.20, 0.08, 0.30))
    m.add('frame', box_span(FRAME, XF + 0.30, XF + 0.50, YF - 0.10, YF + 0.05, 1.20, 1.32))
    m.cylinder('drawbar_swing_cylinder', 'frame', d, (XF + 0.40, YF + 0.10, 1.26), (-2.49, YF + 0.13, 0.27), 0.05, 0.03, 0.55,
               barrel=BODY)
    m.add('frame', hose([(XF + 0.42, YF + 0.16, 1.20), (XF + 0.50, YF + 0.24, 0.60), (XF + 0.38, YF + 0.20, 0.12)], 0.009))


def header(m):
    h = m.node('header_tilt', 'frame', (PV[0], PV[1], 0))
    # pivot tube and two lift arms
    m.add(h, cyl_between('steel_dark', (PV[0], PV[1], -HZ + 0.03), (PV[0], PV[1], HZ - 0.03), 0.05, 12))
    for s in (1, -1):
        m.add(h, box_between(FRAME, (PV[0], PV[1], s * (HZ - 0.20)), (XC - 0.20, 0.30, s * (HZ - 0.20)), 0.06, 0.10, (0, 0, 1)))
    # cutterbar with 8 oval discs and knives, skid shoes
    m.add(h, box_span(FRAME, XC - 0.40, XC + 0.02, 0.04, 0.12, -HZ + 0.06, HZ - 0.06))
    m.add(h, box_span('wear_steel', XC - 0.10, XC + 0.04, 0.02, 0.06, -HZ + 0.08, HZ - 0.08))
    n = 8
    for i in range(n):
        z = -HZ + 0.25 + (2 * HZ - 0.50) * i / (n - 1)
        a = 25 * (1 if i % 2 else -1)
        disc = cylinder('steel', 0.24, 0.035, 18).scale(1.0, 1.0, 0.62).rotate('y', a)
        m.add(h, disc.at((XC - 0.16, 0.14, z)))
        m.add(h, cylinder('steel_dark', 0.07, 0.10, 12).at((XC - 0.16, 0.19, z)))
        for sg in (1, -1):
            ang = math.radians(a)
            kx, kz = sg * 0.25 * math.cos(ang), -sg * 0.25 * math.sin(ang)
            m.add(h, box('wear_steel', 0.10, 0.006, 0.045, (XC - 0.16 + kx, 0.13, z + kz)).rotate('y', a, (XC - 0.16 + kx, 0.13, z + kz)))
        m.add(h, box_span('steel_dark', XC - 0.30, XC - 0.02, 0.0, 0.04, z - 0.05, z + 0.05) if i % 2 == 0 else [])
    # canopy: profile extruded along z, front curtain, side shields
    prof = [(XC + 0.02, 0.50), (XC - 0.10, 0.72), (XC - 0.50, 0.92), (PV[0] + 0.20, 1.02), (PV[0] - 0.05, 0.95),
            (PV[0] - 0.05, 0.92), (PV[0] + 0.18, 0.98), (XC - 0.50, 0.88), (XC - 0.12, 0.69), (XC - 0.01, 0.48)]
    m.add(h, extrude(BODY, prof, 2 * HZ))
    m.add(h, box_span('rubber_trim', XC + 0.005, XC + 0.02, 0.20, 0.50, -HZ + 0.10, HZ - 0.10))         # front curtain
    m.add(h, box_span(FRAME, XC - 0.02, XC + 0.005, 0.46, 0.52, -HZ + 0.06, HZ - 0.06))
    # conditioner rolls (intermeshing rubber chevrons) and swath boards at the rear
    for y, r in ((0.32, 0.12), (0.58, 0.12)):
        m.add(h, cyl_between('rubber_trim', (XC - 0.85, y, -1.40), (XC - 0.85, y, 1.40), r, 16))
        for k in range(8):
            a = 2 * math.pi * k / 8
            yy, xx = y + (r + 0.01) * math.sin(a), XC - 0.85 + (r + 0.01) * math.cos(a)
            for sg in (1, -1):
                m.add(h, box_between('rubber_trim', (xx, yy, 0.0), (xx, yy, sg * 1.38), 0.02, 0.02, (math.cos(a), math.sin(a), 0)))
    for s in (1, -1):
        m.add(h, box_span(FRAME, XC - 0.95, XC - 0.75, 0.20, 0.72, s * 1.40, s * 1.45))
        m.add(h, box_between(BODY, (PV[0] - 0.02, 0.62, s * 1.30), (PV[0] - 0.60, 0.55, s * 0.70), 0.012, 0.30))
        m.add(h, box_between(FRAME, (PV[0] - 0.02, 0.80, s * 1.30), (PV[0] - 0.30, 0.80, s * 1.00), 0.02, 0.03))
    m.add(h, box_span(BODY, PV[0] - 0.20, PV[0] - 0.05, 0.40, 0.92, -1.40, 1.40))                         # rear deflector hood
    # gearbox drive on the left, lift cylinders, side guards
    m.add(h, box_span(FRAME, XC - 0.60, XC - 0.30, 0.55, 0.85, -HZ - 0.12, -HZ + 0.02))
    m.add(h, cyl_between('plastic_black', (XC - 0.45, 0.85, -HZ - 0.05), (PV[0] + 0.30, 1.30, -0.30), 0.05, 12))
    for s in (1, -1):
        side = 'right' if s > 0 else 'left'
        m.add(h, box_span(FRAME, XC - 0.75, XC - 0.60, 0.90, 1.00, s * 0.85, s * 0.95))
        m.cylinder('header_lift_cylinder_' + side, 'frame', h, (XF + 0.18, YF - 0.07, s * 0.90), (XC - 0.68, 1.03, s * 0.90),
                   0.04, 0.025, 0.55, barrel=BODY)
        m.add('frame', hose([(XF + 0.20, YF - 0.02, s * 0.90), (XF + 0.30, YF + 0.05, s * 0.60), (XF + 0.35, YF + 0.06, s * 0.20)], 0.008))
        g = m.node('side_guard_%s_tilt' % side, h, (XC - 0.45, 0.94, s * (HZ + 0.02)))
        m.add(g, box_span(BODY, XC - 1.00, XC + 0.05, 0.18, 0.94, s * (HZ + 0.005), s * (HZ + 0.025)))
        m.add(g, box_span(FRAME, XC - 0.95, XC, 0.15, 0.20, s * (HZ + 0.008), s * (HZ + 0.03)))
        m.add(g, cyl_between('steel_dark', (XC - 0.80, 0.94, s * (HZ + 0.02)), (XC - 0.10, 0.94, s * (HZ + 0.02)), 0.015, 8))
        m.add(g, reflector((XC - 0.48, 0.60, s * (HZ + 0.025)), s, 'z', 'lamp_amber', 0.07))
        m.add(h, box_span(FRAME, XC - 0.42, XC - 0.10, 0.04, 0.10, s * (HZ - 0.06), s * (HZ + 0.02)))      # skid
    m.pose(h, 'z', 13)


def build():
    m = Model('Mower Conditioner', FARM_MATERIALS)
    m.node('frame')
    frame(m)
    drawbar(m)
    header(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'mower-conditioner', 40000)
