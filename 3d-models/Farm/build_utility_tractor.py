"""Utility tractor, ~75 hp class, 4WD with ROPS cab and a front loader.

Key real-world dimensions used (metres):
  wheelbase 2.20, front tyres 12.4R24 (1.17 dia), rear 16.9R30 (1.52 dia)
  rear track 1.60, height to cab roof ~2.65, loader bucket 1.90 wide
  overall length with loader ~4.9, without ~4.1
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, ag_wheel, handrail, hose, lamp, place, save_farm,
                        step, three_point_hitch, tractor_cab)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, sphere
from dozer_parts import exhaust, precleaner
from vehicle_parts import arch_polygon, loader_bucket

AX_F, AX_R = 1.10, -1.10
RF, WF, BF, ZF = 0.585, 0.315, 0.305, 0.78
RR, WR, BR, ZR = 0.76, 0.43, 0.381, 0.80
KP = 0.60            # kingpin offset
CAB = dict(x0=-1.55, x1=0.0, yf=1.10, yr=2.50, hw=0.68, ys=1.62, x_fender=-0.28, hw_low=0.52)


def chassis(m):
    p = 'chassis'
    # wheels: front on steering knuckles, rear on the axle trumpets
    front = ag_wheel(RF, WF, BF, lugs=20)
    rear = ag_wheel(RR, WR, BR, lugs=22)
    for s, side in ((1, 'right'), (-1, 'left')):
        st = m.node('wheel_front_%s_steer' % side, p, (AX_F, RF, s * KP))
        m.add(st, cyl_between(FRAME, (AX_F, RF - 0.13, s * KP), (AX_F, RF + 0.13, s * KP), 0.07, 16))
        m.add(st, cyl_between(FRAME, (AX_F, RF, s * KP), (AX_F, RF, s * (ZF + 0.02)), 0.10, 18))
        m.add(st, cyl_between(FRAME, (AX_F - 0.05, RF + 0.13, s * KP), (AX_F - 0.30, RF + 0.10, s * (KP - 0.05)), 0.025, 8))  # steering arm
        # front fender on a bracket from the knuckle
        m.add(st, extrude(BODY, arch_polygon(AX_F, RF, RF + 0.05, RF + 0.075, 25, 155, 16), 0.34).move(z=s * ZF))
        m.add(st, box_between(FRAME, (AX_F, RF + 0.12, s * KP), (AX_F, RF + 0.60, s * (ZF - 0.12)), 0.05, 0.03))
        m.add(st, box_span(FRAME, AX_F - 0.06, AX_F + 0.06, RF + 0.56, RF + 0.64, s * (ZF - 0.20), s * (ZF - 0.10)))
        sp = m.node('wheel_front_%s_spin' % side, st, (AX_F, RF, s * ZF))
        m.add(sp, place(front, (AX_F, RF, s * ZF), s))
        m.add(m.node('wheel_rear_%s_spin' % side, p, (AX_R, RR, s * ZR)), place(rear, (AX_R, RR, s * ZR), s))
    # steering cylinder across the front axle
    m.add(p, cyl_between(BODY, (AX_F - 0.30, RF + 0.08, -0.45), (AX_F - 0.30, RF + 0.08, 0.25), 0.035, 12))
    m.add(p, cyl_between('chrome', (AX_F - 0.30, RF + 0.08, 0.25), (AX_F - 0.30, RF + 0.09, 0.52), 0.017, 10))
    # front axle (4WD) with diff and drive shaft
    m.add(p, cyl_between(FRAME, (AX_F, RF, -KP + 0.06), (AX_F, RF, KP - 0.06), 0.075, 18))
    m.add(p, sphere(FRAME, 0.17, 18, 9).scale(1, 0.9, 1).at((AX_F, RF, 0.08)))
    for s in (1, -1):
        m.add(p, box_span(FRAME, AX_F - 0.09, AX_F + 0.09, RF - 0.12, RF + 0.12, s * (KP - 0.07), s * (KP - 0.02)))
    m.add(p, cyl_between('steel_dark', (0.0, 0.50, 0.08), (AX_F - 0.12, RF - 0.03, 0.08), 0.035, 12))
    # engine and front frame
    m.add(p, box_span(FRAME, 0.02, 1.85, 0.52, 0.95, -0.28, 0.28))
    m.add(p, box_span(FRAME, 1.85, 2.02, 0.50, 0.90, -0.30, 0.30))                       # front bumper / weight frame
    m.add(p, cyl_between('steel', (1.97, 0.70, -0.32), (1.97, 0.70, 0.32), 0.025, 10))     # tow pin
    m.add(p, box_span('steel_dark', 1.90, 2.03, 0.62, 0.78, -0.04, 0.04))
    # transmission / rear axle housing
    m.add(p, box_span(FRAME, -1.52, 0.0, 0.40, 1.05, -0.30, 0.30))
    m.add(p, box_span(FRAME, -1.45, -0.70, 0.55, 0.95, -0.40, 0.40))
    for s in (1, -1):
        m.add(p, cyl_between(FRAME, (AX_R, RR, s * 0.30), (AX_R, RR, s * 0.66), 0.11, 18, r_top=0.09))
        m.add(p, cyl_between(FRAME, (AX_R, RR, s * 0.66), (AX_R, RR, s * 0.84), 0.13, 18))
    # hood with crown, grille, headlights and vents
    hood = [(0.0, 0.92), (1.82, 0.92), (1.91, 1.00), (1.91, 1.34), (1.83, 1.43), (0.0, 1.49)]
    m.add(p, extrude(BODY, hood, 0.66))
    m.add(p, extrude(BODY, [(0.0, 1.47), (1.80, 1.41), (1.80, 1.445), (0.0, 1.505)], 0.52))
    m.add(p, box_span('steel_dark', 1.905, 1.915, 0.98, 1.32, -0.25, 0.25))
    y = 1.00
    while y < 1.30:
        m.add(p, box_span(FRAME, 1.912, 1.925, y, y + 0.022, -0.24, 0.24))
        y += 0.05
    for s in (1, -1):
        m.add(p, box_span(FRAME, 1.86, 1.93, 1.27, 1.35, s * 0.17, s * 0.30))
        m.add(p, box_span('lamp_lens', 1.93, 1.935, 1.285, 1.335, s * 0.18, s * 0.29))
        m.add(p, box_span('lamp_amber', 1.90, 1.915, 1.36, 1.39, s * 0.24, s * 0.30))
        m.add(p, box_span('steel_dark', 1.25, 1.75, 1.00, 1.30, s * 0.330, s * 0.334))
        yy = 1.03
        while yy < 1.28:
            m.add(p, box_span(FRAME, 1.27, 1.73, yy, yy + 0.02, s * 0.334, s * 0.342))
            yy += 0.045
        m.add(p, box_span(FRAME, 0.60, 0.605, 0.93, 1.47, s * 0.329, s * 0.333))       # panel seam
    m.add(p, exhaust(0.20, 1.48, 2.62, 0.20, 0.045))
    m.add(p, cyl_between('steel_dark', (0.20, 1.62, 0.20), (0.20, 2.05, 0.20), 0.075, 16))   # muffler
    m.add(p, precleaner(0.20, 1.48, -0.20, 0.30))
    # fuel tank (right) and battery box under the cab front
    m.add(p, box_span(BODY, -0.26, -0.02, 0.58, 1.02, 0.32, 0.57))
    m.add(p, cyl_between('steel_dark', (-0.14, 1.02, 0.54), (-0.14, 1.07, 0.54), 0.04, 12))
    m.add(p, box_span(FRAME, -0.24, -0.04, 0.60, 0.85, -0.50, -0.30))
    # rear fenders with lights and grab handles
    for s in (1, -1):
        m.add(p, extrude(BODY, arch_polygon(AX_R, RR, RR + 0.06, RR + 0.10, 8, 172, 24), 0.50).move(z=s * 0.82))
        arc = [(AX_R + 0.84 * math.cos(math.radians(a)), RR + 0.84 * math.sin(math.radians(a))) for a in range(10, 171, 16)]
        m.add(p, extrude(BODY, arc, 0.015).move(z=s * 0.565))
        m.add(p, box_span(FRAME, -1.86, -1.80, 1.10, 1.30, s * 0.86, s * 1.04))
        m.add(p, box_span('lamp_red', -1.875, -1.86, 1.14, 1.22, s * 0.88, s * 0.96))
        m.add(p, box_span('lamp_amber', -1.875, -1.86, 1.22, 1.28, s * 0.88, s * 0.96))
        m.add(p, box_span('lamp_lens', -1.875, -1.86, 1.14, 1.20, s * 0.97, s * 1.02))
    # PTO stub and shield, drawbar
    m.add(p, cyl_between('steel', (-1.53, 0.62, 0), (-1.66, 0.62, 0), 0.0175, 12))
    m.add(p, cyl_between('steel_dark', (-1.52, 0.62, 0), (-1.56, 0.62, 0), 0.05, 12))
    m.add(p, box_span(BODY, -1.68, -1.52, 0.72, 0.735, -0.16, 0.16))
    for s in (1, -1):
        m.add(p, box_span(BODY, -1.68, -1.52, 0.60, 0.735, s * 0.145, s * 0.16))
    m.add(p, box_span(FRAME, -1.90, -0.80, 0.36, 0.40, -0.05, 0.05))
    m.add(p, cyl_between('steel', (-1.82, 0.30, 0), (-1.82, 0.46, 0), 0.018, 10))
    m.add(p, box_span(FRAME, -1.75, -1.50, 0.92, 1.06, -0.34, 0.34))     # rockshaft housing
    m.add(p, box_span(FRAME, -1.58, -1.50, 0.78, 0.92, -0.06, 0.06))     # top-link bracket
    three_point_hitch(m, p, -1.35, 0.45, 0.30, 0.80, 0.42, 0.42, (-1.65, 1.00, 0.30), 0.45,
                      (-1.55, 0.85, 0), (-2.15, 0.90, 0))


def cab(m):
    c = m.node('cab', 'chassis')
    m.add(c, tractor_cab(**CAB, door_side=-1))
    # entry steps and grab handles on the left, right-hand step too
    for s in (1, -1):
        for y in (0.48, 0.80):
            m.add(c, step(-0.25, -0.05, y, s * 0.58, s * 0.80))
        for x in (-0.27, -0.03):
            m.add(c, box_span(FRAME, x - 0.01, x + 0.01, 0.47, 1.05, s * 0.59, s * 0.63))
        m.add(c, handrail([(0.02, 1.25, s * 0.69), (0.05, 1.25, s * 0.74), (0.05, 2.15, s * 0.74), (0.02, 2.15, s * 0.69)]))


def loader(m):
    """Front loader on its own subtree: mounts (fixed), arms, bucket, cylinders."""
    L = m.node('front_loader', 'chassis')
    A, E, D = (0.40, 1.68), (1.75, 1.50), (2.30, 0.45)
    ZA = 0.47
    arm = m.node('loader_arm_tilt', L, (*A, 0))
    bk = m.node('bucket_tilt', arm, (*D, 0))
    for s, side in ((1, 'right'), (-1, 'left')):
        z = s * ZA
        # mount tower bolted to the frame side plus subframe rail
        tower = [(0.22, 0.62), (0.70, 0.62), (0.62, 1.10), (0.52, 1.80), (0.30, 1.80), (0.24, 1.40)]
        m.add(L, extrude(BODY, tower, 0.04).move(z=z))
        m.add(L, box_span(BODY, 0.22, 1.60, 0.60, 0.72, s * 0.29, s * 0.45))
        m.add(L, box_span(BODY, 0.50, 0.70, 0.76, 0.90, s * 0.49, s * 0.60))
        for x, y in ((0.30, 0.68), (0.60, 0.68), (1.30, 0.66), (0.40, 1.60)):
            m.add(L, cyl_between('steel', (x, y, s * 0.44), (x, y, s * 0.51), 0.02, 6))
        # arm: upper beam over the hood side, elbow gusset, lower leg down to the bucket
        m.add(arm, box_between(BODY, (*A, z), (*E, z), 0.075, 0.17))
        m.add(arm, box_between(BODY, (*E, z), (*D, z), 0.075, 0.17))
        m.add(arm, extrude(BODY, [(E[0] - 0.30, E[1] + 0.03), (E[0] + 0.06, E[1] + 0.07), (E[0] + 0.02, E[1] - 0.30),
                                  (E[0] - 0.20, E[1] - 0.06)], 0.08).move(z=z))
        for q in (A, D):
            m.add(arm, cyl_between('steel_dark', (q[0], q[1], z - s * 0.06), (q[0], q[1], z + s * 0.06), 0.035, 12))
        m.add(arm, box_span(BODY, 1.18, 1.32, 1.58, 1.74, s * 0.37, s * 0.44))          # bucket cylinder ear
        m.add(arm, box_span(BODY, 1.08, 1.22, 1.50, 1.60, s * 0.50, s * 0.58))          # lift cylinder ear
        m.cylinder('loader_lift_cylinder_' + side, L, arm, (0.62, 0.84, s * 0.555), (1.15, 1.52, s * 0.555), 0.045, 0.028, 0.55)
        m.cylinder('loader_bucket_cylinder_' + side, arm, bk, (1.25, 1.66, s * 0.39), (2.20, 0.82, s * 0.39), 0.040, 0.025, 0.55)
        # hoses: from the valve under the cab to the tower, then along the arm
        m.add(L, hose([(0.0, 1.00, s * 0.33), (0.25, 0.95, s * 0.46), (0.55, 0.90, s * 0.52), (0.62, 0.90, s * 0.53)]))
        m.add(arm, hose([(0.45, 1.75, s * 0.42), (0.90, 1.69, s * 0.42), (1.22, 1.70, s * 0.40)]))
        m.add(arm, hose([(0.45, 1.72, s * 0.42), (1.00, 1.64, s * 0.425), (1.55, 1.56, s * 0.425), (1.75, 1.36, s * 0.42),
                         (1.85, 1.13, s * 0.405)]))
    # cross tube ahead of the grille
    m.add(arm, cyl_between(BODY, (2.03, 0.98, -ZA), (2.03, 0.98, ZA), 0.055, 16))
    # bucket with mounting brackets
    m.add(bk, [g.at((D[0] - 0.10, 0.03, 0)) for g in loader_bucket(1.90, 0.72, 0.62)])
    for z in (ZA - 0.06, ZA + 0.06, 0.39 - 0.045, 0.39 + 0.045):
        for s in (1, -1):
            m.add(bk, extrude(BODY, [(D[0] - 0.10, 0.10), (D[0] + 0.03, 0.10), (D[0] + 0.07, 0.45), (D[0] - 0.04, 0.88),
                                     (D[0] - 0.15, 0.88), (D[0] - 0.13, 0.55)], 0.024).move(z=s * z))
    for z in (-0.39, 0.39):
        m.add(bk, cyl_between('steel_dark', (2.20, 0.82, z - 0.04), (2.20, 0.82, z + 0.04), 0.022, 10))


def build():
    m = Model('Utility Tractor', FARM_MATERIALS)
    m.node('chassis')
    chassis(m)
    cab(m)
    loader(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'utility-tractor', 60000)
