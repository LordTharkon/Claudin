"""Self-propelled sprayer, high clearance, ~6000 L tank and a 36 m boom folded for transport.

Key real-world dimensions used (metres):
  wheelbase 4.00, track 3.00, tyres 380/90R46 (1.85 dia), crop clearance ~1.35
  boom 36.0 wide: 3.1 centre section + 2 x (8.0 inner + 7.0 outer + 1.45 breakaway tip)
  nozzle spacing 0.50, transport: L ~9.1, W ~4.1, H ~3.7
The boom is authored unfolded and posed into transport: each inner wing swings
90 deg forward about its vertical pivot, the outer section folds 180 deg back
alongside it.
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, ag_wheel, beacon, handrail, hose, lamp, ladder, place,
                        save_farm, step, tractor_cab)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from bodykit import rbox
from dozer_parts import exhaust, louvres

X_F, X_R = 0.0, -4.0
RW, WW, BW, ZW = 0.926, 0.38, 0.584, 1.50
LEG = 1.22                 # wheel leg / kingpin offset
CAB = dict(x0=0.90, x1=2.50, yf=2.05, yr=3.50, hw=0.85, ys=2.40, x_fender=0.90, hw_low=0.85)
XB = -5.60                 # boom plane (truss centre line) behind the machine
YB = 1.95                  # boom bottom chord height in transport
ZP = 1.55                  # wing pivots (ends of the centre section)
L_IN, L_OUT, L_TIP = 8.0, 7.0, 1.45
HOFF = 0.18                # outer-fold hinge offset behind the truss


def truss(x, z0, z1, y0, h0, h1, depth0, depth1, panel=0.9, mat=BODY, nozzles=True):
    """Triangular boom truss along z from z0 to z1 at x: two bottom chords and a top chord,
    braced in both side planes, with a spray line and nozzles every 0.5 m underneath."""
    out = []
    sg = 1 if z1 > z0 else -1
    L = abs(z1 - z0)

    def at(t):
        z = z0 + (z1 - z0) * t
        h = h0 + (h1 - h0) * t
        d = depth0 + (depth1 - depth0) * t
        return z, h, d
    ends = [at(0.0), at(1.0)]
    (za, ha, da), (zb, hb, db) = ends
    for sx in (-1, 1):
        out.append(cyl_between(mat, (x + sx * da / 2, y0, za), (x + sx * db / 2, y0, zb), 0.035 if da > 0.3 else 0.028, 8))
    out.append(cyl_between(mat, (x, y0 + ha, za), (x, y0 + hb, zb), 0.03, 8))
    n = max(1, int(round(L / panel)))
    for i in range(n + 1):
        z, h, d = at(i / n)
        out.append(box_between(mat, (x - d / 2, y0, z), (x + d / 2, y0, z), 0.03, 0.03))   # bottom rung
        if 0 < i <= n:
            zp, hp, dp = at((i - 1) / n)
            for sx in (-1, 1):
                a = (x + sx * dp / 2, y0, zp) if i % 2 else (x, y0 + hp, zp)
                b = (x, y0 + h, z) if i % 2 else (x + sx * d / 2, y0, z)
                out.append(cyl_between(mat, a, b, 0.018, 6, caps=False))
        for sx in (-1, 1):
            out.append(cyl_between(mat, (x + sx * d / 2, y0, z), (x, y0 + h, z), 0.018, 6, caps=False))
    if nozzles:
        out.append(cyl_between('steel', (x, y0 - 0.07, za), (x, y0 - 0.07, zb), 0.014, 6))
        k = 0.25
        while k < L:
            z = z0 + sg * k
            out.append(cylinder('plastic_black', 0.018, 0.08, 6).at((x, y0 - 0.11, z)))
            out.append(cylinder('paint_red', 0.012, 0.025, 6).at((x, y0 - 0.16, z)))
            k += 0.5
    return out


def chassis(m):
    p = 'chassis'
    wh = ag_wheel(RW, WW, BW, lugs=24)
    for s, side in ((1, 'right'), (-1, 'left')):
        for x, tag in ((X_F, 'front'), (X_R, 'rear')):
            par = p
            if tag == 'front':
                par = m.node('wheel_front_%s_steer' % side, p, (x, 1.60, s * LEG))
            m.add(par, box_span(FRAME, x - 0.11, x + 0.11, RW - 0.05, 1.62, s * (LEG - 0.11), s * (LEG + 0.11)))
            m.add(par, cyl_between(FRAME, (x, RW, s * LEG), (x, RW, s * (ZW - 0.04)), 0.17, 18))     # hub motor
            m.add(par, extrude(BODY, [(x - 0.75, RW + 0.38), (x - 0.55, RW + 0.70), (x + 0.55, RW + 0.70),
                                      (x + 0.75, RW + 0.38), (x + 0.75, RW + 0.42), (x + 0.57, RW + 0.74),
                                      (x - 0.57, RW + 0.74), (x - 0.75, RW + 0.42)], 0.44).move(z=s * ZW))  # mudguard
            m.add(par, box_span(FRAME, x - 0.06, x + 0.06, RW + 0.12, RW + 0.72, s * (LEG + 0.02), s * (LEG + 0.10)))
            sp = m.node('wheel_%s_%s_spin' % (tag, side), par, (x, RW, s * ZW))
            m.add(sp, place(wh, (x, RW, s * ZW), s))
            # axle beam out to the leg
            m.add(p, box_span(FRAME, x - 0.14, x + 0.14, 1.62, 1.82, s * 0.40, s * (LEG + 0.12)))
            if tag == 'front':
                m.add(p, cylinder('steel_dark', 0.10, 0.06, 16).at((x, 1.61, s * LEG)))
        m.add(p, cyl_between(BODY, (X_F + 0.35, 1.58, s * 0.30), (X_F + 0.20, 1.58, s * (LEG - 0.10)), 0.04, 12))   # steering
        m.add(p, box_span(FRAME, X_F + 0.12, X_F + 0.24, 1.40, 1.62, s * (LEG - 0.16), s * (LEG - 0.08)))
    # main frame
    for s in (1, -1):
        m.add(p, box_span(FRAME, -5.35, 2.55, 1.45, 1.75, s * 0.30, s * 0.46))
    for x in (-5.0, -2.0, 1.2, 2.4):
        m.add(p, box_span(FRAME, x - 0.06, x + 0.06, 1.50, 1.70, -0.30, 0.30))
    m.add(p, box_span(FRAME, 0.90, 2.50, 1.75, 2.00, -0.80, 0.80))           # cab mount
    m.add(p, box_span(FRAME, 2.50, 2.62, 1.55, 1.95, -0.60, 0.60))           # front bumper
    for s in (1, -1):
        m.add(p, box_span(FRAME, 2.62, 2.66, 1.62, 1.72, s * 0.35, s * 0.55))
        m.add(p, box_span('lamp_lens', 2.66, 2.665, 1.63, 1.71, s * 0.37, s * 0.53))
        m.add(p, box_span('lamp_amber', 2.62, 2.635, 1.75, 1.82, s * 0.45, s * 0.58))
    # engine compartment at the rear
    m.add(p, rbox(BODY, 2.10, 1.15, 1.60, 0.10, (-4.10, 2.33, 0)))
    for s in (1, -1):
        m.add(p, louvres(-4.90, -3.40, 1.95, 2.60, s * 0.80, s, 0.06))
    m.add(p, box_span('steel_dark', -5.155, -5.15, 1.95, 2.70, -0.60, 0.60))
    for y in (2.00, 2.10, 2.20, 2.30, 2.40, 2.50, 2.60):
        m.add(p, box_span(FRAME, -5.165, -5.155, y, y + 0.03, -0.58, 0.58))
    m.add(p, exhaust(-3.40, 2.90, 3.55, -0.45, 0.06))
    for s in (1, -1):
        m.add(p, box_span(FRAME, -5.16, -5.12, 2.75, 2.85, s * 0.55, s * 0.75))
        m.add(p, box_span('lamp_red', -5.17, -5.16, 2.77, 2.83, s * 0.57, s * 0.65))
        m.add(p, box_span('lamp_amber', -5.17, -5.16, 2.77, 2.83, s * 0.66, s * 0.73))
    # main tank (poly) with shrouds, lid, rinse tank and chemical inductor
    m.add(p, rbox('paint_white', 3.40, 1.25, 2.00, 0.30, (-1.15, 2.42, 0)))
    m.add(p, lathe(BODY, [(0, 3.03), (0.24, 3.03), (0.24, 3.11), (0.20, 3.15), (0, 3.15)], 24).move(x=-1.2))
    m.add(p, cylinder(FRAME, 0.27, 0.03, 24).at((-1.2, 3.035, 0)))
    m.add(p, rbox('paint_gray', 0.90, 0.35, 1.30, 0.08, (0.05, 3.18, 0)))
    for s in (1, -1):
        m.add(p, box_span(BODY, -2.80, 0.50, 1.76, 2.15, s * 1.0, s * 1.03))
        m.add(p, box_span(FRAME, -2.80, 0.50, 2.15, 2.18, s * 0.98, s * 1.04))
    m.add(p, cylinder(BODY, 0.24, 0.42, 20, r_top=0.30).at((-0.40, 1.60, -1.12)))      # inductor hopper
    m.add(p, cylinder(FRAME, 0.31, 0.03, 20).at((-0.40, 1.825, -1.12)))
    m.add(p, box_span(FRAME, -0.48, -0.32, 1.40, 1.70, -1.04, -0.90))
    m.add(p, hose([(-0.40, 1.39, -1.12), (-0.45, 1.30, -0.80), (-0.60, 1.35, -0.40)], 0.025))
    # pump and plumbing under the tank
    m.add(p, cylinder('steel_dark', 0.14, 0.25, 16).rotate('x', 90).at((-2.40, 1.35, 0.0)))
    m.add(p, hose([(-2.40, 1.35, 0.15), (-3.00, 1.42, 0.25), (-4.60, 1.48, 0.25), (-5.30, 1.80, 0.25)], 0.025))
    m.add(p, hose([(-2.40, 1.35, -0.15), (-1.80, 1.40, -0.20), (-1.30, 1.75, -0.25)], 0.025))
    # boom transport cradles beside the cab
    for s in (1, -1):
        m.add(p, box_span(FRAME, 1.95, 2.10, 1.62, 1.75, s * 0.46, s * 2.10))
        m.add(p, box_span(FRAME, 1.95, 2.10, 1.75, 2.00, s * 1.24, s * 1.31))
        m.add(p, box_span(FRAME, 1.95, 2.10, 1.75, 2.00, s * 2.10, s * 2.17))
        m.add(p, box_span(FRAME, 1.95, 2.10, 1.75, 1.87, s * 1.31, s * 2.10))
        m.add(p, box_span('rubber_trim', 1.96, 2.09, 1.87, 1.90, s * 1.31, s * 2.10))

def cab(m):
    c = m.node('cab', 'chassis')
    m.add(c, tractor_cab(**CAB, door_side=-1, sill_front=CAB['yf'] + 0.08, seat_x=1.65, door_x0=1.65, wheel_tilt=35))
    # platform and ladder on the left, ahead of the front wheel
    m.add(c, box_span('steel_dark', 0.95, 1.70, 1.97, 2.03, -1.25, -0.85))
    m.add(c, ladder((1.05, 0.30, -1.20), (1.35, 1.97, -1.20), 0.40, 6, axis=(1, 0, 0)))
    m.add(c, handrail([(0.95, 2.03, -1.25), (0.95, 3.05, -1.25), (1.62, 3.05, -1.25)]))
    for z in (-0.5, 0.5):
        m.add(c, lamp((2.62, 3.57, z * 0.6), 1, 0.9))


def boom(m):
    p = 'chassis'
    # lift masts on the rear frame; the boom carriage slides on them
    for s in (1, -1):
        m.add(p, box_span(FRAME, -5.40, -5.30, 1.35, 2.95, s * 0.48, s * 0.58))
        m.add(p, box_span(FRAME, -5.40, -5.16, 2.90, 2.98, s * 0.42, s * 0.64))
    m.add(p, box_span(FRAME, -5.40, -5.16, 1.40, 1.48, -0.64, 0.64))
    lift = m.node('boom_lift', p, (-5.45, YB, 0))
    m.add(lift, box_span(BODY, -5.52, -5.42, YB - 0.10, YB + 0.62, -0.70, 0.70))
    for s in (1, -1):
        m.add(lift, box_span(BODY, -5.46, -5.24, YB - 0.10, YB + 0.62, s * 0.60, s * 0.70))
        for y in (YB, YB + 0.50):
            m.add(lift, cyl_between('plastic_black', (-5.29, y, s * 0.47), (-5.29, y, s * 0.60), 0.04, 10))
    m.cylinder('boom_lift_cylinder', p, lift, (-5.25, 1.50, 0.0), (-5.25, YB + 0.45, 0.0), 0.055, 0.035, 0.55)
    m.add(lift, box_span(BODY, -5.30, -5.20, YB + 0.38, YB + 0.52, -0.06, 0.06))
    # centre section with the wing pivots and fold-cylinder anchors
    m.add(lift, truss(XB, -ZP, ZP, YB, 0.55, 0.55, 0.42, 0.42, nozzles=True))
    for s in (1, -1):
        m.add(lift, box_span(BODY, XB - 0.25, XB + 0.25, YB - 0.04, YB + 0.62, s * (ZP - 0.10), s * ZP))
        m.add(lift, cyl_between('steel_dark', (XB, YB - 0.06, s * ZP), (XB, YB + 0.64, s * ZP), 0.035, 10))
        m.add(lift, box_span(BODY, XB + 0.40, XB + 0.56, YB + 0.20, YB + 0.36, s * 0.52, s * 0.66))
        m.add(lift, box_between(BODY, (XB + 0.21, YB + 0.28, s * 0.60), (XB + 0.48, YB + 0.28, s * 0.60), 0.08, 0.08))
    for s, side in ((1, 'right'), (-1, 'left')):
        inner = m.node('boom_%s_rotate' % side, lift, (XB, YB, s * ZP))
        z0, z1 = s * ZP, s * (ZP + L_IN)
        m.add(inner, truss(XB, z0, z1, YB, 0.55, 0.32, 0.42, 0.30))
        m.add(inner, box_span(BODY, XB - 0.24, XB + 0.40, YB - 0.04, YB + 0.60, z0, z0 + s * 0.12))
        m.add(inner, box_span(BODY, XB - 0.25, XB + 0.10, YB - 0.03, YB + 0.36, z1 - s * 0.10, z1))
        hinge = (XB - HOFF, YB, z1)
        m.add(inner, cyl_between('steel_dark', (hinge[0], YB - 0.05, z1), (hinge[0], YB + 0.38, z1), 0.03, 10))
        m.add(inner, box_span(BODY, XB - 0.05, XB + 0.05, YB + 0.20, YB + 0.34, z1 - s * 0.95, z1 - s * 0.80))
        m.cylinder('boom_%s_fold_cylinder' % side, lift, inner, (XB + 0.48, YB + 0.28, s * 0.60), (XB + 0.35, YB + 0.28, s * (ZP + 0.20)),
                   0.045, 0.028, 0.55)
        m.add(inner, box_span(BODY, XB + 0.20, XB + 0.40, YB + 0.20, YB + 0.36, s * (ZP + 0.12), s * (ZP + 0.26)))
        m.add(inner, hose([(XB, YB - 0.07, z0 + s * 0.2), (XB - 0.10, YB - 0.10, z0 + s * 0.4), (XB - 0.05, YB - 0.07, z1 - s * 0.3)], 0.016))
        outer = m.node('boom_%s_outer_rotate' % side, inner, hinge)
        z2 = z1 + s * L_OUT
        m.add(outer, truss(XB, z1 + s * 0.02, z2, YB, 0.32, 0.20, 0.30, 0.22))
        m.add(outer, box_span(BODY, XB - 0.20, XB + 0.10, YB - 0.03, YB + 0.34, z1 + s * 0.02, z1 + s * 0.12))
        m.add(outer, box_span(BODY, XB - 0.05, XB + 0.05, YB + 0.18, YB + 0.30, z1 + s * 0.35, z1 + s * 0.50))
        m.cylinder('boom_%s_outer_fold_cylinder' % side, inner, outer, (XB, YB + 0.27, z1 - s * 0.87), (XB, YB + 0.24, z1 + s * 0.42),
                   0.035, 0.022, 0.55)
        tip = m.node('boom_%s_tip_rotate' % side, outer, (XB, YB, z2))
        m.add(tip, truss(XB, z2 + s * 0.02, z2 + s * L_TIP, YB, 0.20, 0.10, 0.22, 0.10, panel=0.5))
        m.add(tip, cyl_between('steel_dark', (XB, YB - 0.05, z2), (XB, YB + 0.22, z2), 0.022, 8))
        m.add(tip, box_span('lamp_amber', XB - 0.03, XB + 0.03, YB + 0.10, YB + 0.18, z2 + s * (L_TIP - 0.06), z2 + s * L_TIP))
        # transport pose: wing forward, outer section folded back alongside
        m.pose(inner, 'y', 90 * s)
        m.pose(outer, 'y', 180)


def build():
    m = Model('Self-Propelled Sprayer', FARM_MATERIALS)
    m.node('chassis')
    chassis(m)
    cab(m)
    boom(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'self-propelled-sprayer', 60000)
