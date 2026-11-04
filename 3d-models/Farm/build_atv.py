"""Farm-spec ATV / quad bike, ~500 cc 4x4, with front and rear steel racks and a tow hitch.

Key real-world dimensions used (metres):
  wheelbase 1.28, track ~0.95 front / 0.92 rear, 25x8-12 front and 25x10-12 rear tyres
  seat height ~0.88, ground clearance ~0.26, 50 mm receiver hitch at the rear
  overall ~2.15 long, 1.20 wide, 1.25 high (to the mirrors / rack tops)

Rest pose: wheels straight. Rig: `wheel_front_{left,right}_steer` (about y at the kingpins)
-> `_spin`; `wheel_rear_{left,right}_spin`; `handlebar_rotate` (bars, grips, levers and the
pod, about the steering column, which leans back 20 deg).
"""
import math

from farm_parts import FARM_MATERIALS, place, r1_tyre, save_farm
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from bodykit import rbox
from vehicle_parts import arch_polygon, rim

PL = 'paint_green'            # body plastics
TUBE = 'chassis_black'        # racks, bumpers, frame
AXF, AXR = 0.64, -0.64
RF, WF = 0.317, 0.20
RR, WR = 0.317, 0.25
ZF, ZR = 0.40, 0.38           # wheel centre offsets (inner face to centre handled by width)
KP = 0.28
COL = (0.40, 0.70, 0.0)       # steering column base
LEAN = 20.0


def wheel(r, w, hand):
    return r1_tyre(r, w, 0.152, lugs=14, depth=0.022, angle=40, segs=28, hand=hand) + rim(0.152, w, 'steel', 'paint_gray_dark', nuts=4)


def tube(pts, r=0.014, mat=TUBE):
    out = [cyl_between(mat, a, b, r, 8) for a, b in zip(pts, pts[1:])]
    out += [sphere(mat, r, 8, 4).at(p) for p in pts[1:-1]]
    return out


def rack(x0, x1, y, hz, rails=4):
    """Flat tube rack: perimeter, cross rails and four legs down to the body."""
    out = tube([(x0, y, -hz), (x1, y, -hz), (x1, y, hz), (x0, y, hz), (x0, y, -hz)], 0.014)
    for k in range(1, rails):
        x = x0 + (x1 - x0) * k / rails
        out.append(cyl_between(TUBE, (x, y, -hz), (x, y, hz), 0.010, 6))
    for z in (-hz * 0.5, 0.0, hz * 0.5):
        out.append(cyl_between(TUBE, (x0, y, z), (x1, y, z), 0.008, 6))
    return out


def body(m):
    p = 'chassis'
    # frame rails and skid plate
    for s in (1, -1):
        m.add(p, tube([(0.85, 0.32, s * 0.16), (0.55, 0.28, s * 0.16), (-0.55, 0.28, s * 0.16), (-0.85, 0.40, s * 0.16)], 0.018))
        m.add(p, tube([(0.70, 0.62, s * 0.14), (0.30, 0.70, s * 0.14), (-0.40, 0.70, s * 0.14), (-0.75, 0.62, s * 0.14)], 0.016))
    m.add(p, box_span('steel_dark', -0.50, 0.53, 0.255, 0.27, -0.18, 0.18))
    # engine, CVT cover, exhaust and muffler (right side)
    m.add(p, box_span('steel_dark', -0.25, 0.20, 0.32, 0.62, -0.13, 0.13))
    m.add(p, rbox('plastic_black', 0.42, 0.26, 0.10, 0.03, (0.0, 0.46, -0.19)))
    m.add(p, tube([(0.10, 0.50, 0.12), (0.05, 0.55, 0.22), (-0.40, 0.62, 0.22), (-0.62, 0.66, 0.24)], 0.022, 'steel_dark'))
    m.add(p, cyl_between('steel_dark', (-0.55, 0.66, 0.24), (-0.85, 0.68, 0.24), 0.055, 14))
    # front and rear fenders with the centre body, foot wells
    for ax, r, hw, z in ((AXF, RF, 0.16, ZF), (AXR, RR, 0.18, ZR)):
        for s in (1, -1):
            m.add(p, extrude(PL, arch_polygon(ax, r, r + 0.06, r + 0.085, 15, 165, 12), 2 * hw).move(z=s * (z + 0.01)))
    m.add(p, rbox(PL, 0.62, 0.20, 0.70, 0.06, (AXF + 0.08, 0.76, 0.0)))          # front hood
    m.add(p, rbox(PL, 0.62, 0.20, 0.66, 0.06, (AXR - 0.06, 0.76, 0.0)))          # rear body
    for s in (1, -1):
        m.add(p, box_span('plastic_black', -0.32, 0.30, 0.42, 0.45, s * 0.18, s * 0.52))    # floorboards
        for k in range(4):
            x = -0.26 + 0.15 * k
            m.add(p, box_span('steel_dark', x, x + 0.02, 0.45, 0.47, s * 0.22, s * 0.50))
        m.add(p, extrude(PL, [(-0.34, 0.45), (0.33, 0.45), (0.42, 0.72), (-0.36, 0.72)], 0.03).move(z=s * 0.53))
    # tank / centre console and seat
    m.add(p, rbox(PL, 0.40, 0.18, 0.30, 0.05, (0.18, 0.84, 0.0)))
    m.add(p, cylinder('plastic_black', 0.04, 0.03, 12).at((0.24, 0.94, 0.0)))
    m.add(p, rbox('seat_fabric', 0.72, 0.12, 0.34, 0.05, (-0.30, 0.88, 0.0)))
    m.add(p, box_span(PL, -0.66, 0.02, 0.80, 0.83, -0.16, 0.16))
    # front fascia: headlights, grille, brush guard
    m.add(p, box_span('plastic_black', 0.92, 0.95, 0.52, 0.72, -0.24, 0.24))
    for s in (1, -1):
        m.add(p, rbox(PL, 0.10, 0.10, 0.18, 0.03, (0.94, 0.78, s * 0.20)))
        m.add(p, box_span('lamp_lens', 0.985, 0.992, 0.75, 0.81, s * 0.13, s * 0.27))
        m.add(p, box_span('lamp_amber', 0.97, 0.98, 0.70, 0.73, s * 0.20, s * 0.27))
    m.add(p, tube([(0.85, 0.30, -0.22), (1.04, 0.36, -0.22), (1.05, 0.70, -0.20), (1.05, 0.70, 0.20), (1.04, 0.36, 0.22), (0.85, 0.30, 0.22)], 0.018))
    m.add(p, cyl_between(TUBE, (1.045, 0.50, -0.21), (1.045, 0.50, 0.21), 0.014, 8))
    # front rack
    m.add(p, rack(0.58, 1.02, 0.92, 0.42))
    for s in (1, -1):
        m.add(p, cyl_between(TUBE, (0.62, 0.92, s * 0.36), (0.62, 0.80, s * 0.32), 0.012, 8))
        m.add(p, cyl_between(TUBE, (0.98, 0.92, s * 0.36), (0.96, 0.80, s * 0.30), 0.012, 8))
    # rear rack, tail light, reflectors
    m.add(p, rack(-1.06, -0.48, 0.92, 0.44, 5))
    for s in (1, -1):
        m.add(p, cyl_between(TUBE, (-0.52, 0.92, s * 0.38), (-0.52, 0.80, s * 0.30), 0.012, 8))
        m.add(p, cyl_between(TUBE, (-1.02, 0.92, s * 0.38), (-0.98, 0.80, s * 0.30), 0.012, 8))
        m.add(p, box_span('lamp_red', -1.02, -1.006, 0.70, 0.74, s * 0.24, s * 0.30))
    m.add(p, box_span('plastic_black', -1.045, -1.005, 0.68, 0.78, -0.12, 0.12))
    m.add(p, box_span('lamp_red', -1.052, -1.045, 0.70, 0.76, -0.09, 0.09))
    # rear bumper and 2 in receiver hitch with ball mount and pin
    m.add(p, tube([(-0.85, 0.40, -0.22), (-1.02, 0.42, -0.22), (-1.02, 0.42, 0.22), (-0.85, 0.40, 0.22)], 0.018))
    m.add(p, box_span(TUBE, -1.12, -0.85, 0.34, 0.40, -0.03, 0.03))
    m.add(p, box_span('steel', -1.22, -1.08, 0.35, 0.39, -0.022, 0.022))
    m.add(p, box_span('steel', -1.26, -1.18, 0.39, 0.41, -0.04, 0.04))
    m.add(p, sphere('steel', 0.025, 10, 5).at((-1.22, 0.43, 0.0)))
    m.add(p, cyl_between('steel', (-1.22, 0.405, 0.0), (-1.22, 0.42, 0.0), 0.012, 8))
    m.add(p, cyl_between('steel', (-1.10, 0.37, -0.05), (-1.10, 0.37, 0.05), 0.006, 6))
    # front A-arms and coil-over shocks, rear swing arm / axle tube and shocks
    for s in (1, -1):
        for y in (0.30, 0.40):
            m.add(p, box_between(TUBE, (AXF - 0.12, y, s * 0.14), (AXF, y + (RF - y) * 0.6, s * KP), 0.02, 0.02))
            m.add(p, box_between(TUBE, (AXF + 0.12, y, s * 0.14), (AXF, y + (RF - y) * 0.6, s * KP), 0.02, 0.02))
        m.add(p, cyl_between('steel_dark', (AXF - 0.04, 0.33, s * 0.24), (AXF - 0.10, 0.66, s * 0.16), 0.022, 8))
        m.add(p, cyl_between('paint_red', (AXF - 0.05, 0.40, s * 0.225), (AXF - 0.09, 0.60, s * 0.17), 0.032, 10))
        m.add(p, cyl_between('steel_dark', (AXR + 0.02, RR, s * 0.20), (-0.20, 0.62, s * 0.12), 0.022, 8))
        m.add(p, cyl_between('paint_red', (AXR - 0.01, 0.40, s * 0.19), (AXR + 0.06, 0.58, s * 0.15), 0.034, 10))
    m.add(p, cyl_between('steel_dark', (AXR, RR, -ZR + 0.08), (AXR, RR, ZR - 0.08), 0.03, 10))
    m.add(p, box_between('steel_dark', (AXR, RR, 0.0), (-0.10, 0.30, 0.0), 0.10, 0.06))
    m.add(p, box_span('steel_dark', AXR - 0.08, AXR + 0.08, RR - 0.08, RR + 0.08, -0.10, 0.10))
    # wheels
    for s, side in ((1, 'right'), (-1, 'left')):
        st = m.node('wheel_front_%s_steer' % side, p, (AXF, RF, s * KP))
        m.add(st, cyl_between('steel_dark', (AXF, RF - 0.07, s * KP), (AXF, RF + 0.07, s * KP), 0.025, 10))
        m.add(st, cyl_between('steel_dark', (AXF, RF, s * KP), (AXF, RF, s * (ZF - 0.06)), 0.035, 10))
        m.add(st, box_between('steel_dark', (AXF, RF, s * KP), (AXF - 0.10, RF + 0.02, s * KP), 0.02, 0.03))
        sp = m.node('wheel_front_%s_spin' % side, st, (AXF, RF, s * ZF))
        m.add(sp, place(wheel(RF, WF, 1), (AXF, RF, s * ZF), s))
        rs = m.node('wheel_rear_%s_spin' % side, p, (AXR, RR, s * ZR))
        m.add(rs, place(wheel(RR, WR, 1), (AXR, RR, s * ZR), s))
    # tie rods from the column base to the steering arms
    for s in (1, -1):
        m.add(p, cyl_between('steel', (AXF - 0.12, RF + 0.02, s * 0.03), (AXF - 0.10, RF + 0.02, s * (KP - 0.02)), 0.008, 6))


def handlebars(m):
    c = m.node('handlebar_rotate', 'chassis', COL)
    a = math.radians(LEAN)
    top = (COL[0] - 0.36 * math.sin(a), COL[1] + 0.36 * math.cos(a), 0.0)
    m.add(c, cyl_between('steel_dark', COL, top, 0.016, 10))
    m.add(c, rbox(PL, 0.16, 0.08, 0.32, 0.03, (top[0] + 0.02, top[1] - 0.01, 0.0)))     # headlight pod
    m.add(c, box_span('lamp_lens', top[0] + 0.098, top[0] + 0.104, top[1] - 0.04, top[1] + 0.01, -0.08, 0.08))
    m.add(c, box_span('plastic_black', top[0] - 0.08, top[0] - 0.02, top[1] + 0.03, top[1] + 0.05, -0.06, 0.06))
    bar = [(top[0] - 0.02, top[1] + 0.05, -0.38), (top[0] - 0.05, top[1] + 0.06, -0.22), (top[0], top[1] + 0.04, -0.06),
           (top[0], top[1] + 0.04, 0.06), (top[0] - 0.05, top[1] + 0.06, 0.22), (top[0] - 0.02, top[1] + 0.05, 0.38)]
    m.add(c, tube(bar, 0.011, 'steel_dark'))
    for s in (1, -1):
        g = (top[0] - 0.02, top[1] + 0.05, s * 0.38)
        m.add(c, cyl_between('rubber_trim', (g[0], g[1], s * 0.28), (g[0], g[1], s * 0.41), 0.017, 10))
        m.add(c, box_span('plastic_black', g[0] - 0.03, g[0] + 0.03, g[1] - 0.03, g[1] + 0.03, s * 0.22, s * 0.27))
        m.add(c, box_between('steel_dark', (g[0] + 0.02, g[1], s * 0.24), (g[0] + 0.07, g[1] - 0.01, s * 0.36), 0.01, 0.015))
        m.add(c, cyl_between('steel_dark', (g[0], g[1], s * 0.22), (g[0] - 0.04, g[1] + 0.20, s * 0.30), 0.006, 6))
        m.add(c, box('plastic_black', 0.02, 0.06, 0.10, (g[0] - 0.04, g[1] + 0.23, s * 0.30)))
        m.add(c, box('glass', 0.004, 0.05, 0.085, (g[0] - 0.052, g[1] + 0.23, s * 0.30)))
    m.add(c, box_span('plastic_black', top[0] - 0.02, top[0] + 0.03, top[1] + 0.03, top[1] + 0.07, 0.20, 0.24))   # thumb throttle


def build():
    m = Model('ATV', FARM_MATERIALS)
    m.node('chassis')
    body(m)
    handlebars(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'atv', 25000)
