"""Trailed tandem disc harrow, 6.0 m working width, hydraulic folding wings.

Key real-world dimensions used (metres):
  560 mm (22 in) notched front / plain rear discs at 230 mm spacing, gangs at 18 deg
  centre section 2.24 wide, wings 1.88 each, working width 6.0
  hitch eye 0.51 above the ground (the row-crop tractor's drawbar height)
  four 11L-15 transport wheels on a lifting axle; overall ~5.6 long, 3.7 wide, 3.1 high folded

Rest pose: transport. Wheels down with the discs clear of the ground, wings folded up
90 deg about their hinges. `wing_right_tilt` +90 deg / `wing_left_tilt` -90 deg about x
unfold them to the working width.
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, WHEEL, align_y, coupling, dished_disc, hazard_board, hitch_eye,
                        hose, implement_tyre, parking_jack, place, rear_light, reflector, save_farm, smv_panel)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder
from vehicle_parts import rim

YF = 1.05                   # top of the main frame tubes
YG = 0.62                   # gang axle height (transport)
XF, XR = -2.00, -3.45       # front / rear cross tubes
XGF, XGR = -2.35, -3.80     # front / rear gang centres
ZH, ZW = 1.12, 3.00         # wing hinge, wing tip
DR, PITCH, ANG = 0.28, 0.23, 18.0
RT, WT, BT = 0.39, 0.28, 0.19


def gang(m, node, z0, z1, xc, front, side):
    """Disc gang between z0 and z1 (|z| increasing), angled ANG deg in plan.
    Front gangs throw outward, rear gangs throw back inward."""
    s = 1 if z1 > 0 else -1
    a = math.radians(ANG) * (1 if front else -1) * s
    d = (math.sin(a), 0.0, math.cos(a))           # gang axis, roughly along +z
    zm = (z0 + z1) / 2
    L = abs(z1 - z0)
    c = (xc, YG, zm)
    n = int(L / PITCH)
    p0 = (c[0] - d[0] * L / 2, YG, c[2] - d[2] * L / 2)
    p1 = (c[0] + d[0] * L / 2, YG, c[2] + d[2] * L / 2)
    m.add(node, cyl_between('steel_dark', p0, p1, 0.022, 8))
    concave = 1 if front else -1                   # front discs concave outward (+s), rear inward
    blade = dished_disc(DR, 0.045, 0.006, 18, 'wear_steel' if front else 'steel')
    for i in range(n):
        t = -L / 2 + PITCH * (i + 0.5) + (L - n * PITCH) / 2
        pc = (c[0] + d[0] * t, YG, c[2] + d[2] * t)
        m.add(node, align_y(blade, (d[0] * s * concave, 0, d[2] * s * concave), pc))
        if i < n - 1:
            q = (pc[0] + d[0] * PITCH * 0.5, YG, pc[2] + d[2] * PITCH * 0.5)
            m.add(node, cyl_between(FRAME, (q[0] - d[0] * 0.08, YG, q[2] - d[2] * 0.08),
                                    (q[0] + d[0] * 0.08, YG, q[2] + d[2] * 0.08), 0.055, 10))
    # gang bar under the frame following the gang angle, bearing hangers (C-bars) down to the axle
    m.add(node, box_between(BODY, (p0[0], YF - 0.21, p0[2]), (p1[0], YF - 0.21, p1[2]), 0.10, 0.10))
    for f in (0.22, 0.78):
        t = -L / 2 + L * f
        pc = (c[0] + d[0] * t, YG, c[2] + d[2] * t)
        m.add(node, cyl_between('steel_dark', (pc[0] - d[0] * 0.06, YG, pc[2] - d[2] * 0.06),
                                (pc[0] + d[0] * 0.06, YG, pc[2] + d[2] * 0.06), 0.075, 12))
        m.add(node, box_span(FRAME, pc[0] - 0.03, pc[0] + 0.03, YG + 0.06, YF - 0.24, pc[2] - 0.05, pc[2] + 0.05))
    m.add(node, box_between('steel_dark', (p0[0] - 0.32, YG + 0.18, p0[2]), (p1[0] - 0.32, YG + 0.18, p1[2]), 0.04, 0.04))
    for i in range(0, n, 2):
        t = -L / 2 + PITCH * (i + 0.5) + (L - n * PITCH) / 2
        pc = (c[0] + d[0] * t - 0.32, YG + 0.18, c[2] + d[2] * t)
        m.add(node, box_between('steel', pc, (pc[0] + 0.08, YG - 0.02, pc[2]), 0.04, 0.008, (0, 0, 1)))


def centre(m):
    p = 'frame'
    # main frame: two side rails, front and rear cross tubes, diagonal braces
    for s in (1, -1):
        m.add(p, box_span(BODY, -4.10, -1.75, YF - 0.12, YF, s * (ZH - 0.14), s * (ZH - 0.02)))
        m.add(p, box_span(BODY, -4.10, -1.75, YF - 0.12, YF, s * 0.30, s * 0.42))
        m.add(p, box_between(BODY, (XF, YF - 0.06, s * 0.42), (XR, YF - 0.06, s * (ZH - 0.14)), 0.08, 0.08))
    for x in (XF, XR):
        m.add(p, box_span(BODY, x - 0.075, x + 0.075, YF - 0.15, YF, -ZH + 0.02, ZH - 0.02))
    m.add(p, box_span(BODY, -4.12, -3.98, YF - 0.12, YF, -ZH + 0.02, ZH - 0.02))
    # wing hinge knuckles on the frame
    for s in (1, -1):
        for x in (XF, XR):
            m.add(p, box_span(FRAME, x - 0.10, x + 0.10, YF - 0.05, YF + 0.06, s * (ZH - 0.12), s * (ZH - 0.005)))
        m.add(p, box_span(FRAME, XF + 0.08, XF + 0.16, YF - 0.30, YF - 0.12, s * 0.30, s * 0.42))      # fold cylinder anchor
    # gangs on the centre section (front pair throws out, rear pair throws in)
    for s in (1, -1):
        gang(m, p, s * 0.06, s * (ZH - 0.06), XGF, True, s)
        gang(m, p, s * 0.06, s * (ZH - 0.06), XGR, False, s)
    # A-frame drawbar with the hitch eye at drawbar height, jack and hoses
    eye = (0.0, 0.51, 0.0)
    for s in (1, -1):
        m.add(p, box_between(BODY, (XF + 0.05, YF - 0.07, s * 0.80), (-0.70, 0.62, s * 0.06), 0.10, 0.12))
    m.add(p, box_between(BODY, (-0.75, 0.62, 0), (-0.10, 0.52, 0), 0.10, 0.10))
    m.add(p, box_span(FRAME, -0.16, -0.02, 0.48, 0.555, -0.056, 0.056))
    m.add(p, hitch_eye(eye))
    m.add(p, box_between(FRAME, (-0.95, 0.66, -0.06), (-1.25, 0.74, -0.06), 0.03, 0.30, (0, 1, 0)))  # jack bracket
    m.add(p, parking_jack(-1.10, -0.20, 0.0, 0.95, crank=-1))
    m.add(p, box_span(FRAME, -1.16, -1.04, 0.70, 0.78, -0.20, -0.06))
    for k, z in enumerate((-0.12, -0.06, 0.06, 0.12)):
        m.add(p, hose([(XF, YF + 0.04, z * 3), (-1.50, 0.80, z), (-0.40, 0.66, z), (-0.20, 0.75, z)], 0.010))
        m.add(p, coupling((-0.16, 0.75, z), 'lamp_red' if k < 2 else 'lamp_amber'))
    m.add(p, hose([(XF, YF + 0.05, 0.18), (-1.0, 0.82, 0.10), (-0.30, 0.70, 0.18), (-0.20, 0.82, 0.18)], 0.006))  # lighting lead
    m.add(p, box_span(FRAME, -0.30, -0.18, 0.66, 0.86, 0.12, 0.24))                                         # hose holder
    # transport axle: arms hinged on the frame, four wheels, lift cylinders
    xa, xw = -2.62, -2.95
    ax = m.node('transport_axle_tilt', p, (xa, YF - 0.20, 0))
    for s in (1, -1):
        m.add(p, box_span(FRAME, xa - 0.06, xa + 0.06, YF - 0.26, YF - 0.12, s * 0.12, s * 0.24))
        m.add(ax, box_between(FRAME, (xa, YF - 0.20, s * 0.18), (xw, RT, s * 0.18), 0.08, 0.12, (0, 0, 1)))
    m.add(ax, cyl_between('steel_dark', (xa, YF - 0.20, -0.30), (xa, YF - 0.20, 0.30), 0.04, 12))
    m.add(ax, box_span(FRAME, xw - 0.07, xw + 0.07, RT - 0.07, RT + 0.07, -0.98, 0.98))
    for s, side in ((1, 'right'), (-1, 'left')):
        for zi, nm in ((0.42, 'inner'), (0.80, 'outer')):
            c = (xw, RT, s * zi)
            w = m.node('wheel_%s_%s_spin' % (side, nm), ax, c)
            m.add(w, place(implement_tyre(RT, WT, BT, ribs=5, segs=30) + rim(BT, WT, 'steel', WHEEL, nuts=6), c, s))
        m.add(ax, box_span(FRAME, xw + 0.05, xw + 0.15, RT + 0.02, RT + 0.42, s * 0.20, s * 0.28))
        m.add(p, box_span(FRAME, -2.15, -2.05, YF, YF + 0.16, s * 0.20, s * 0.28))
        m.cylinder('axle_lift_cylinder_' + side, p, ax, (-2.10, YF + 0.10, s * 0.24), (xw + 0.10, RT + 0.38, s * 0.24),
                   0.05, 0.03, 0.55, barrel=BODY)
        m.add(p, hose([(-2.10, YF + 0.16, s * 0.24), (-2.0, YF + 0.20, s * 0.10), (XF, YF + 0.04, s * 0.08)], 0.009))
    # rear lights, SMV, warning boards, reflectors
    for s in (1, -1):
        m.add(p, box_span(FRAME, -4.20, -4.12, YF - 0.10, YF + 0.55, s * 0.92, s * 1.00))
        m.add(p, rear_light((-4.24, YF + 0.45, s * 0.86), -1))
        m.add(p, hazard_board((-4.205, YF + 0.10, s * 0.70), 0.42, 0.28, facing=-1))
        m.add(p, reflector((-1.80, YF - 0.06, s * (ZH - 0.08) + s * 0.06), s, 'z', 'lamp_amber', 0.06))
    m.add(p, box_span(FRAME, -4.20, -4.12, YF - 0.05, YF + 0.70, -0.04, 0.04))
    m.add(p, smv_panel((-4.215, YF + 0.50, 0.0), 0.36))
    m.add(p, reflector((-0.80, 0.68, 0.0), 1, 'x', 'lamp_lens', 0.07))


def wing(m, s):
    side = 'right' if s > 0 else 'left'
    w = m.node('wing_%s_tilt' % side, 'frame', (XF, YF, s * ZH))
    # wing frame authored unfolded, out to the tip
    for x in (XF, XR):
        m.add(w, box_span(BODY, x - 0.075, x + 0.075, YF - 0.15, YF, s * (ZH + 0.02), s * ZW))
        m.add(w, box_span(FRAME, x - 0.10, x + 0.10, YF - 0.05, YF + 0.06, s * (ZH + 0.005), s * (ZH + 0.12)))
        m.add(w, cyl_between('steel', (x - 0.12, YF, s * ZH), (x + 0.12, YF, s * ZH), 0.03, 10))
    m.add(w, box_span(BODY, -4.10, -1.80, YF - 0.12, YF, s * (ZW - 0.12), s * ZW))
    m.add(w, box_span(BODY, -4.10, -3.53, YF - 0.12, YF, s * (ZH + 0.12), s * (ZH + 0.24)))
    m.add(w, box_between(BODY, (XF - 0.08, YF - 0.06, s * (ZH + 0.30)), (XR + 0.08, YF - 0.06, s * (ZW - 0.14)), 0.08, 0.08))
    m.add(w, box_span(BODY, -4.12, -3.98, YF - 0.12, YF, s * (ZH + 0.12), s * ZW))
    gang(m, w, s * (ZH + 0.10), s * (ZW - 0.04), XGF, True, s)
    gang(m, w, s * (ZH + 0.10), s * (ZW - 0.04), XGR, False, s)
    # fold lug hanging below the hinge, transport lock hook, tip reflectors and a hazard board
    m.add(w, box_span(FRAME, XF + 0.08, XF + 0.16, YF - 0.42, YF - 0.12, s * (ZH - 0.01), s * (ZH + 0.12)))
    m.add(w, box_span(FRAME, XR - 0.05, XR + 0.05, YF, YF + 0.22, s * (ZW - 0.40), s * (ZW - 0.34)))
    m.add(w, reflector((-4.12, YF - 0.06, s * (ZW - 0.30)), -1, 'x', 'lamp_red', 0.07))
    m.add(w, reflector((-1.80, YF - 0.06, s * (ZW - 0.30)), 1, 'x', 'lamp_lens', 0.07))
    m.add(w, hazard_board((-1.79, YF + 0.10, s * (ZW - 0.40)), 0.40, 0.22, facing=1))
    m.add(w, box_span(FRAME, -1.84, -1.80, YF - 0.02, YF + 0.05, s * (ZW - 0.60), s * (ZW - 0.20)))
    m.cylinder('wing_%s_fold_cylinder' % side, 'frame', w, (XF + 0.12, YF - 0.26, s * 0.36), (XF + 0.12, YF - 0.38, s * (ZH + 0.06)),
               0.045, 0.028, 0.55, barrel=BODY)
    m.add('frame', hose([(XF + 0.12, YF - 0.22, s * 0.36), (XF + 0.20, YF - 0.05, s * 0.30), (XF + 0.12, YF + 0.03, s * 0.12)], 0.009))
    m.pose(w, 'x', -90 * s)


def build():
    m = Model('Disc Harrow', FARM_MATERIALS)
    m.node('frame')
    centre(m)
    wing(m, 1)
    wing(m, -1)
    return m


if __name__ == '__main__':
    save_farm(build(), 'disc-harrow', 40000)
