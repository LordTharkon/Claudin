"""Trailed pneumatic disc seed drill, 6.0 m, front hopper, folding coulter bar.

Key real-world dimensions used (metres):
  hitch eye 0.51 above the ground (the row-crop tractor's drawbar height)
  ~3500 L hopper, fan and distributor head; 36 single-disc coulters at 0.167 m row spacing
  in two staggered ranks, 360 mm discs, 340 mm press wheels, following harrow
  centre bar 1.5 wide, wings 2.25 each (6.0 working width), 400/60-15.5 transport wheels
  overall ~6.0 long, 3.5 wide, 3.3 high folded

Rest pose: transport. Coulter-bar wings folded up 90 deg on `wing_{left,right}_tilt`
(right +90 / left -90 deg about x unfold them), markers stowed back along the wing frames
on `marker_{left,right}_tilt` (+165 deg about x deploys the right one, -165 the left),
hopper lid closed on `hopper_lid_tilt` (front hinge, about z).
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, WHEEL, align_y, coupling, dished_disc, hazard_board, hitch_eye,
                        hose, implement_tyre, ladder, parking_jack, place, rear_light, reflector, save_farm,
                        smv_panel)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe
from vehicle_parts import rim

XB, YB = -4.30, 0.90            # coulter bar centre x and top height
ZH, ZW = 0.75, 3.00             # wing hinge, wing tip
SPC = 6.0 / 36
RT, WT, BT = 0.42, 0.40, 0.20   # transport tyres
XW = -2.70                      # transport axle


def press_wheel(r, w, segs=16):
    hw = w / 2
    prof = [(r * 0.55, -hw), (r * 0.9, -hw), (r, -hw * 0.3), (r, hw * 0.3), (r * 0.9, hw), (r * 0.55, hw), (r * 0.55, -hw)]
    return [lathe('tyre_rubber', prof, segs), cylinder('steel_dark', r * 0.55, w * 0.7, segs)]


DISC = dished_disc(0.18, 0.012, 0.005, 14, 'wear_steel', rings=2)
PRESS = press_wheel(0.17, 0.065, 12)


def coulter(z, rank):
    """One coulter unit hanging behind the bar: arm, angled disc, seed boot, press wheel."""
    x0 = XB - 0.08 - 0.32 * rank
    out = []
    out.append(box_span(FRAME, XB - 0.08 - 0.06, XB - 0.08, YB - 0.24, YB - 0.10, z - 0.03, z + 0.03) if rank == 0 else
               box_span(FRAME, XB - 0.10, XB + 0.10, YB - 0.30, YB - 0.24, z - 0.025, z + 0.025))
    pivot = (XB - 0.10, YB - 0.27, z)
    disc_c = (x0 - 0.30, 0.18, z)
    out.append(box_between(FRAME, pivot, (disc_c[0] + 0.05, disc_c[1] + 0.10, z), 0.04, 0.06, (0, 0, 1)))
    a = math.radians(7)
    out.append(align_y(DISC, (math.sin(a), 0, math.cos(a)), disc_c))
    out.append(cylinder('steel_dark', 0.045, 0.05, 8).rotate('x', 90).at((disc_c[0], disc_c[1], z - 0.03)))
    out.append(box_between('steel_dark', (disc_c[0] - 0.05, 0.30, z - 0.03), (disc_c[0] - 0.14, 0.06, z - 0.03), 0.02, 0.05, (0, 0, 1)))
    pw = (disc_c[0] - 0.42, 0.17, z)
    out.append(box_between(FRAME, (disc_c[0] - 0.02, disc_c[1] + 0.10, z + 0.035), (pw[0], pw[1] + 0.02, z + 0.035), 0.02, 0.04, (0, 0, 1)))
    out += align_y(PRESS, (0, 0, 1), pw)
    out.append(cyl_between('steel', (pivot[0] - 0.03, pivot[1] + 0.02, z + 0.04), (disc_c[0] + 0.12, disc_c[1] + 0.30, z + 0.04), 0.008, 6))
    # seed hose from the bar to the boot
    out.append(cyl_between('hose_rubber', (XB, YB + 0.02, z), (XB - 0.12, YB - 0.05, z - 0.03), 0.012, 6))
    out.append(cyl_between('hose_rubber', (XB - 0.12, YB - 0.05, z - 0.03), (disc_c[0] - 0.06, 0.34, z - 0.03), 0.012, 6))
    return out


def section(m, node, z0, z1):
    """Coulter bar, coulters (two staggered ranks) and following harrow between z0 and z1 (|z| up)."""
    lo, hi = min(z0, z1), max(z0, z1)
    m.add(node, box_span(BODY, XB - 0.09, XB + 0.09, YB - 0.18, YB, lo, hi))
    n = int(round((hi - lo) / SPC))
    for i in range(n):
        z = lo + SPC * (i + 0.5)
        m.add(node, coulter(z, i % 2))
    # following harrow: bar on two arms, spring tines
    xh = XB - 1.30
    m.add(node, box_span(FRAME, xh - 0.03, xh + 0.03, 0.45, 0.51, lo + 0.02, hi - 0.02))
    for f in (0.2, 0.8):
        z = lo + (hi - lo) * f
        m.add(node, box_between(FRAME, (XB - 0.09, YB - 0.10, z), (xh, 0.51, z), 0.04, 0.06, (0, 0, 1)))
    for i in range(n):
        z = lo + SPC * (i + 0.5)
        m.add(node, cyl_between('steel', (xh, 0.48, z), (xh - 0.10, 0.45, z), 0.008, 6))
        m.add(node, cyl_between('steel', (xh - 0.10, 0.45, z), (xh - 0.30, 0.06, z), 0.006, 6))


def chassis(m):
    p = 'frame'
    eye = (0.0, 0.51, 0.0)
    m.add(p, hitch_eye(eye))
    m.add(p, box_between(BODY, (-0.08, 0.51, 0), (-1.10, 0.66, 0), 0.10, 0.10))
    for s in (1, -1):
        m.add(p, box_between(BODY, (-1.05, 0.66, s * 0.05), (-1.80, 0.80, s * 0.90), 0.10, 0.14))
        m.add(p, box_span(BODY, -4.30, -1.70, 0.68, 0.86, s * 0.86, s * 0.98))                 # side members
    m.add(p, box_span(BODY, -1.85, -1.70, 0.68, 0.86, -0.86, 0.86))
    m.add(p, box_span(BODY, -3.40, -3.25, 0.68, 0.86, -0.86, 0.86))
    m.add(p, parking_jack(-0.80, 0.20, 0.0, 0.95, crank=1))
    m.add(p, box_span(FRAME, -0.86, -0.74, 0.62, 0.70, 0.06, 0.26))
    # transport axle and wheels
    m.add(p, box_span(FRAME, XW - 0.08, XW + 0.08, RT - 0.08, RT + 0.08, -1.12, 1.12))
    for s in (1, -1):
        m.add(p, box_span(FRAME, XW - 0.10, XW + 0.10, RT + 0.08, 0.68, s * 0.84, s * 0.98))
        side = 'right' if s > 0 else 'left'
        c = (XW, RT, s * 1.40)
        w = m.node('wheel_%s_spin' % side, p, c)
        m.add(w, place(implement_tyre(RT, WT, BT, ribs=5, segs=30) + rim(BT, WT, 'steel', WHEEL, nuts=6), c, s))
        m.add(p, cyl_between(FRAME, (XW, RT, s * 1.12), (XW, RT, s * 1.22), 0.06, 12))
        # mudguard
        arc = [(XW + (RT + 0.08) * math.cos(math.radians(a)), RT + (RT + 0.08) * math.sin(math.radians(a))) for a in range(10, 175, 15)]
        arc2 = [(XW + (RT + 0.10) * math.cos(math.radians(a)), RT + (RT + 0.10) * math.sin(math.radians(a))) for a in range(170, 5, -15)]
        from meshkit import extrude as ex
        m.add(p, ex(BODY, arc + arc2, 0.46).move(z=s * 1.40))
    # hopper: tapered bottom, upright body, rim; distributor tower and fan
    hp = [(-2.15, 0.90), (-2.95, 0.90), (-3.30, 1.40), (-3.30, 2.40), (-1.75, 2.40), (-1.75, 1.40)]
    m.add(p, extrude(BODY, hp, 2.50))
    m.add(p, box_span(FRAME, -2.95, -2.15, 0.86, 0.90, -0.40, 0.40))                  # metering unit
    m.add(p, box_span('steel_dark', -2.90, -2.70, 0.70, 0.86, -0.15, 0.15))
    m.add(p, box_span(BODY, -3.32, -1.73, 2.38, 2.42, -1.27, 1.27))
    m.add(p, box_span('glass', -1.745, -1.735, 1.60, 2.20, 0.80, 0.95))                 # level window
    m.add(p, cyl_between(FRAME, (-1.95, 0.95, -1.00), (-1.95, 0.95, -0.70), 0.26, 22))    # fan
    m.add(p, cyl_between('steel_dark', (-1.95, 0.95, -1.03), (-1.95, 0.95, -1.00), 0.22, 22))
    m.add(p, cyl_between(FRAME, (-2.20, 0.80, -0.70), (-2.80, 0.78, -0.30), 0.07, 12))
    m.add(p, cyl_between(FRAME, (-2.80, 0.78, -0.10), (-3.60, 0.82, 0.0), 0.07, 12))
    m.add(p, cyl_between(FRAME, (-3.60, 0.82, 0.0), (-3.70, 1.65, 0.0), 0.07, 12))       # riser to the head
    m.add(p, cyl_between(BODY, (-3.70, 1.62, 0.0), (-3.70, 1.82, 0.0), 0.25, 24))       # distributor head
    m.add(p, cyl_between(BODY, (-3.70, 1.82, 0.0), (-3.70, 1.86, 0.0), 0.20, 24))
    # platform and ladder behind the hopper
    m.add(p, box_span('steel_dark', -3.85, -3.32, 1.55, 1.58, 0.35, 1.20))
    m.add(p, ladder((-3.95, 0.30, 0.95), (-3.60, 1.55, 0.95), 0.40, 4, axis=(0, 0, 1)))
    m.add(p, cyl_between(FRAME, (-3.85, 1.58, 1.20), (-3.85, 2.55, 1.20), 0.018, 8))
    m.add(p, cyl_between(FRAME, (-3.85, 2.55, 1.20), (-3.85, 2.55, 0.40), 0.018, 8))
    m.add(p, cyl_between(FRAME, (-3.85, 1.58, 0.40), (-3.85, 2.55, 0.40), 0.018, 8))
    m.add(p, box_span(FRAME, -3.85, -3.45, 0.84, 1.55, 1.12, 1.18))
    # coulter bar carrier, centre section
    for s in (1, -1):
        m.add(p, box_span(BODY, XB + 0.09, -3.40, 0.70, 0.84, s * 0.74, s * 0.86))
    section(m, p, -ZH, ZH)
    for z in (-0.50, 0.50):
        m.add(p, cyl_between('steel_dark', (-3.70, 1.70, z * 0.3), (XB, YB + 0.02, z * 1.7), 0.03, 8))
    # hydraulics to the tractor
    for k, z in enumerate((-0.18, -0.12, 0.12, 0.18)):
        m.add(p, hose([(-1.80, 0.88, z * 4), (-1.10, 0.74, z), (-0.30, 0.62, z), (-0.20, 0.72, z)], 0.010))
        m.add(p, coupling((-0.16, 0.72, z), 'lamp_red' if k < 2 else 'lamp_amber'))
    m.add(p, box_span(FRAME, -0.30, -0.18, 0.56, 0.68, -0.24, 0.24))
    # rear lights, SMV, warning boards
    xr = XB - 0.12
    for s in (1, -1):
        m.add(p, box_span(FRAME, xr - 0.05, xr, YB, YB + 0.80, s * 0.84, s * 0.90))
        m.add(p, rear_light((xr - 0.04, YB + 0.72, s * 0.75), -1))
        m.add(p, hazard_board((xr - 0.02, YB + 0.42, s * 0.75), 0.24, 0.40, facing=-1, stripes=3))
        m.add(p, reflector((-1.70, 0.77, s * 0.98), s, 'z', 'lamp_amber', 0.06))
    m.add(p, box_span(FRAME, xr - 0.05, xr, YB, YB + 0.50, -0.04, 0.04))
    m.add(p, smv_panel((xr - 0.05, YB + 0.52, 0.0), 0.36))
    m.add(p, reflector((-1.86, 0.77, 0.30), 1, 'x', 'lamp_lens', 0.07))


def wing(m, s):
    side = 'right' if s > 0 else 'left'
    hp = (XB, YB + 0.10, s * ZH)
    w = m.node('wing_%s_tilt' % side, 'frame', hp)
    section(m, w, s * (ZH + 0.01), s * ZW)
    for d in (-1, 1):
        m.add('frame', box_span(FRAME, XB + d * 0.05 - 0.03, XB + d * 0.05 + 0.03, YB, YB + 0.16, s * (ZH - 0.12), s * (ZH - 0.01)))
        m.add(w, box_span(FRAME, XB + d * 0.13 - 0.03, XB + d * 0.13 + 0.03, YB, YB + 0.16, s * (ZH + 0.01), s * (ZH + 0.12)))
    m.add(w, cyl_between('steel', (XB - 0.18, YB + 0.10, s * ZH), (XB + 0.18, YB + 0.10, s * ZH), 0.025, 10))
    # fold lug and cylinder from a post on the centre bar
    m.add(w, box_span(FRAME, XB + 0.10, XB + 0.16, YB, YB + 0.42, s * (ZH + 0.15), s * (ZH + 0.25)))
    m.add('frame', box_span(FRAME, XB + 0.10, XB + 0.16, YB, YB + 0.62, s * 0.30, s * 0.40))
    m.cylinder('wing_%s_fold_cylinder' % side, 'frame', w, (XB + 0.13, YB + 0.55, s * 0.36), (XB + 0.13, YB + 0.38, s * (ZH + 0.20)),
               0.045, 0.028, 0.55, barrel=BODY)
    m.add('frame', hose([(XB + 0.13, YB + 0.60, s * 0.36), (XB + 0.20, YB + 0.70, s * 0.20), (-3.45, 0.86, s * 0.20)], 0.009))
    # tip reflectors and a marker arm stowed back along the wing
    m.add(w, reflector((XB, YB - 0.09, s * ZW), s, 'z', 'lamp_amber', 0.06))
    hy, hz = YB + 0.20, s * (ZW - 0.05)
    m.add(w, box_span(FRAME, XB - 0.06, XB + 0.06, YB, hy + 0.06, s * (ZW - 0.14), s * (ZW + 0.04)))
    mk = m.node('marker_%s_tilt' % side, w, (XB, hy, hz))
    L = 2.30
    m.add(mk, cyl_between('steel_dark', (XB - 0.07, hy, hz), (XB + 0.07, hy, hz), 0.03, 10))
    m.add(mk, box_between(BODY, (XB, hy, hz), (XB, hy, hz + s * L), 0.06, 0.06))
    m.add(mk, box_between(FRAME, (XB, hy, hz + s * L), (XB, hy - 0.40, hz + s * L), 0.04, 0.04, (0, 0, 1)))
    disc = dished_disc(0.18, 0.025, 0.006, 16, 'wear_steel', rings=2)
    m.add(mk, align_y(disc, (math.sin(math.radians(20)), 0, s * math.cos(math.radians(20))), (XB, hy - 0.40, hz + s * L)))
    m.add(mk, box_span(FRAME, XB - 0.04, XB + 0.04, hy, hy + 0.20, hz + s * 0.15, hz + s * 0.25))
    m.add(w, box_span(FRAME, XB - 0.04, XB + 0.04, YB, hy + 0.10, s * (ZW - 0.80), s * (ZW - 0.70)))
    m.cylinder('marker_%s_cylinder' % side, w, mk, (XB, hy + 0.08, s * (ZW - 0.75)), (XB, hy + 0.17, hz + s * 0.20),
               0.03, 0.02, 0.6, barrel=BODY)
    m.pose(mk, 'x', -165 * s)
    m.pose(w, 'x', -90 * s)


def lid(m):
    h = m.node('hopper_lid_tilt', 'frame', (-1.72, 2.44, 0))
    m.add(h, box_span(BODY, -3.34, -1.71, 2.42, 2.47, -1.29, 1.29))
    m.add(h, box_span(BODY, -3.20, -1.85, 2.47, 2.52, -1.15, 1.15))
    for z in (-0.9, 0.0, 0.9):
        m.add(h, cyl_between('steel_dark', (-1.72, 2.44, z - 0.08), (-1.72, 2.44, z + 0.08), 0.018, 8))
    m.add(h, box_span('steel_dark', -3.38, -3.33, 2.36, 2.46, -0.12, 0.12))


def build():
    m = Model('Seed Drill', FARM_MATERIALS)
    m.node('frame')
    chassis(m)
    wing(m, 1)
    wing(m, -1)
    lid(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'seed-drill', 40000)
