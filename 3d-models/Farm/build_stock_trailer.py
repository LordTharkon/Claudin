"""Bumper-pull aluminium stock (livestock) trailer, 16 ft box, tandem axle.

Key real-world dimensions used (metres):
  box 4.88 long x 2.03 wide outside (1.93 inside), 1.98 inside height, floor 0.42 above the
  ground (the livestock-props loading ramp meets this height)
  tandem 3500 lb axles on leaf springs, 1.22 m spread, 225/75R15 tyres
  A-frame tongue, ball coupler 0.50 high; overall ~6.8 long, 2.23 wide, 2.50 high

Rest pose: level on the jack, all gates closed. Rig: `rear_gate_rotate` (full-width rear
swing gate, hinged on the left rear post, about y) carrying `rear_gate_slider_slide`
(sliding half-gate, local -z); `center_gate_rotate` (interior divider gate hinged on the
right wall, about y); `jack_leg_lift` (tongue-jack inner leg, vertical); wheels `_spin`.
"""
import math

from farm_parts import (FARM_MATERIALS, FRAME, implement_tyre, place, reflector, save_farm)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from vehicle_parts import arch_polygon, rim

AL = 'aluminum'
X0, X1 = -4.88, 0.0           # box rear / front
YF = 0.42                     # floor top
YR = YF + 1.98                # roof underside
HZ = 1.015                    # half width outside
T = 0.04                      # wall post depth
RW, WW, BW = 0.356, 0.225, 0.19
AXLES = (-2.10, -3.32)
ZWH = 0.84


def wall(m, n, s):
    """Side wall at z = s*HZ: posts, solid lower skin, open slats above, top rail."""
    z0, z1 = s * (HZ - T), s * HZ
    xs = [X0 + 0.05 + (X1 - X0 - 0.10) * k / 8 for k in range(9)]
    for x in xs:
        m.add(n, box_span(AL, x - 0.03, x + 0.03, YF - 0.12, YR, z0, z1))
    # lower skin up to 0.80 above the floor
    m.add(n, box_span(AL, X0 + 0.08, X1 - 0.08, YF - 0.12, YF + 0.80, s * (HZ - T + 0.012), s * (HZ - T + 0.016)))
    # slats with gaps above
    y = YF + 0.86
    while y + 0.12 < YR - 0.08:
        m.add(n, box_span(AL, X0 + 0.08, X1 - 0.08, y, y + 0.12, s * (HZ - T / 2 - 0.006), s * (HZ - T / 2 + 0.006)))
        y += 0.22
    m.add(n, box_span(AL, X0, X1, YR - 0.08, YR, s * (HZ - T - 0.01), s * HZ))                # top rail
    m.add(n, box_span(AL, X0, X1, YF - 0.16, YF - 0.08, s * (HZ - 0.10), s * (HZ + 0.005)))    # bottom rail
    # fenders over the tandem wheels with diamond-plate tops
    for ax in AXLES:
        m.add(n, extrude(AL, arch_polygon(ax, RW, RW + 0.06, RW + 0.075, 15, 165, 10), 0.14).move(z=s * (HZ + 0.03)))
    m.add(n, box_span(AL, AXLES[1] - 0.30, AXLES[0] + 0.30, RW + 0.06, RW + 0.075, s * (HZ - 0.02), s * (HZ + 0.10)))
    # clearance lamps and side reflectors
    for x, lens in ((X1 - 0.12, 'lamp_amber'), (X0 + 0.12, 'lamp_red')):
        m.add(n, box_span(lens, x - 0.04, x + 0.04, YR - 0.06, YR - 0.02, s * HZ, s * (HZ + 0.012)))
    m.add(n, reflector((X0 + 0.30, YF - 0.12, s * (HZ + 0.005)), s, 'z', 'lamp_red', 0.05))
    m.add(n, reflector((X1 - 0.30, YF - 0.12, s * (HZ + 0.005)), s, 'z', 'lamp_amber', 0.05))


def body(m):
    n = 'chassis'
    # main rails, cross members, floor and mats
    for s in (1, -1):
        m.add(n, box_span(FRAME, X0, X1 + 0.20, YF - 0.30, YF - 0.16, s * 0.80, s * 0.88))
    for k in range(9):
        x = X0 + 0.15 + (X1 - X0 - 0.30) * k / 8
        m.add(n, box_span(AL, x - 0.03, x + 0.03, YF - 0.16, YF - 0.06, -HZ + 0.06, HZ - 0.06))
    m.add(n, box_span(AL, X0, X1, YF - 0.06, YF - 0.01, -HZ + 0.03, HZ - 0.03))
    m.add(n, box_span('rubber_trim', X0 + 0.05, X1 - 0.05, YF - 0.01, YF, -HZ + T + 0.01, HZ - T - 0.01))
    wall(m, n, 1)
    wall(m, n, -1)
    # inside wheel wells
    for s in (1, -1):
        m.add(n, box_span(AL, AXLES[1] - 0.45, AXLES[0] + 0.45, YF, RW + 0.42, s * (HZ - T - 0.28), s * (HZ - T - 0.005)))
    # front: slanted nose with a vent window, front posts
    nose = [(X1, YF - 0.12), (X1 + 0.10, YF + 0.30), (X1 + 0.10, YR - 0.30), (X1, YR)]
    m.add(n, extrude(AL, nose + [(X1 - 0.02, YR), (X1 - 0.02, YF - 0.12)], 2 * HZ - 0.002))
    m.add(n, box_span('steel_dark', X1 + 0.10, X1 + 0.104, YR - 0.85, YR - 0.45, -0.60, 0.60))
    for k in range(6):
        y = YR - 0.82 + 0.065 * k
        m.add(n, box_span(AL, X1 + 0.104, X1 + 0.11, y, y + 0.025, -0.58, 0.58))
    # roof: sheet over bows with a slight crown
    for k in range(9):
        x = X0 + 0.05 + (X1 - X0 - 0.10) * k / 8
        m.add(n, box_span(AL, x - 0.025, x + 0.025, YR, YR + 0.05, -HZ + 0.01, HZ - 0.01))
    roof = [(-HZ - 0.02, YR + 0.05), (HZ + 0.02, YR + 0.05), (HZ + 0.02, YR + 0.06), (0.0, YR + 0.10), (-HZ - 0.02, YR + 0.06)]
    m.add(n, extrude(AL, roof, X1 - X0 + 0.04).rotate('y', 90).move(x=(X0 + X1) / 2))
    # rear frame: corner posts, header, sill, latch keepers, lamps
    for s in (1, -1):
        m.add(n, box_span(AL, X0 - 0.08, X0, YF - 0.16, YR, s * (HZ - 0.08), s * HZ))
        m.add(n, box_span(FRAME, X0 - 0.10, X0 - 0.08, YF - 0.05, YF + 0.25, s * (HZ - 0.07), s * (HZ - 0.01)))
        m.add(n, box_span('lamp_red', X0 - 0.112, X0 - 0.10, YF + 0.03, YF + 0.17, s * (HZ - 0.06), s * (HZ - 0.02)))
        m.add(n, box_span('lamp_amber', X0 - 0.112, X0 - 0.10, YF + 0.18, YF + 0.23, s * (HZ - 0.06), s * (HZ - 0.02)))
    m.add(n, box_span(AL, X0 - 0.08, X0, YR - 0.10, YR + 0.05, -HZ + 0.08, HZ - 0.08))
    m.add(n, box_span(AL, X0 - 0.12, X0, YF - 0.16, YF, -HZ + 0.08, HZ - 0.08))
    for z in (-0.12, 0.0, 0.12):
        m.add(n, box_span('lamp_red', X0 - 0.09, X0 - 0.08, YR - 0.06, YR - 0.02, z - 0.03, z + 0.03))
    m.add(n, box_span(FRAME, X0 - 0.13, X0 - 0.12, YF - 0.12, YF - 0.02, -0.25, 0.05))                # blank plate bracket
    for s in (1, -1):
        m.add(n, box_span('steel', X0 - 0.12, X0 - 0.08, YF + 0.95, YF + 1.05, s * (HZ - 0.05), s * (HZ + 0.01)))


def running_gear(m):
    n = 'chassis'
    for i, ax in enumerate(AXLES):
        m.add(n, cyl_between('steel_dark', (ax, RW, -ZWH - 0.03), (ax, RW, ZWH + 0.03), 0.04, 12))
        for s in (1, -1):
            # leaf spring and hangers
            m.add(n, box_span('steel_dark', ax - 0.36, ax + 0.36, RW + 0.02, RW + 0.07, s * 0.80, s * 0.86))
            for dx in (-0.38, 0.38):
                m.add(n, box_span(FRAME, ax + dx - 0.03, ax + dx + 0.03, RW + 0.04, YF - 0.30, s * 0.79, s * 0.87))
            side = 'right' if s > 0 else 'left'
            c = (ax, RW, s * (HZ - 0.06))
            w = m.node('wheel_%s_%s_spin' % ('front' if i == 0 else 'rear', side), n, c)
            m.add(w, place(implement_tyre(RW, WW, BW, ribs=4, segs=28) + rim(BW, WW, 'steel', 'steel', nuts=6), c, s))
            m.add(n, cyl_between('steel', (ax, RW, s * ZWH), (ax, RW, s * (HZ - 0.15)), 0.06, 12))
    m.add(n, box_span(FRAME, AXLES[1] + 0.30, AXLES[0] - 0.30, RW + 0.04, YF - 0.30, -0.865, -0.785))   # equaliser hangers
    m.add(n, box_span(FRAME, AXLES[1] + 0.30, AXLES[0] - 0.30, RW + 0.04, YF - 0.30, 0.785, 0.865))


def tongue(m):
    n = 'chassis'
    cp = (1.55, 0.50, 0.0)
    for s in (1, -1):
        m.add(n, box_between(FRAME, (X1 + 0.20, YF - 0.23, s * 0.84), (cp[0] - 0.25, 0.48, s * 0.05), 0.07, 0.12))
    m.add(n, box_span(FRAME, cp[0] - 0.40, cp[0] - 0.10, 0.42, 0.54, -0.08, 0.08))
    # ball coupler with latch and safety chains
    m.add(n, box_span('steel', cp[0] - 0.15, cp[0] + 0.12, 0.47, 0.56, -0.07, 0.07))
    m.add(n, sphere('steel', 0.065, 14, 7).scale(1, 0.6, 1).at((cp[0] + 0.10, 0.52, 0.0)))
    m.add(n, box_span('steel_dark', cp[0] - 0.10, cp[0] + 0.05, 0.56, 0.58, -0.02, 0.02))
    for s in (1, -1):
        pts = [(cp[0] - 0.25, 0.45, s * 0.07)]
        for k in range(1, 9):
            t = k / 8
            pts.append((cp[0] - 0.25 + 0.40 * t, 0.45 - 0.18 * math.sin(math.pi * t), s * (0.07 + 0.03 * t)))
        for a, b in zip(pts, pts[1:]):
            m.add(n, cyl_between('steel', a, b, 0.008, 6))
    # breakaway box, cable, lighting plug
    m.add(n, box_span(FRAME, 0.60, 0.80, 0.55, 0.70, 0.30, 0.38))
    m.add(n, cyl_between('cable_rubber', (0.70, 0.62, 0.30), (cp[0] - 0.20, 0.50, 0.06), 0.006, 6))
    m.add(n, cyl_between('cable_rubber', (X1 + 0.20, 0.30, -0.70), (cp[0] - 0.30, 0.48, -0.10), 0.008, 6))
    m.add(n, cylinder(FRAME, 0.025, 0.06, 10).rotate('z', 90).at((cp[0] - 0.28, 0.49, -0.10)))
    # tongue jack: outer tube on the left A-frame member, inner leg and foot plate on jack_leg_lift
    jx, jz = 0.95, -0.42
    m.add(n, cyl_between(FRAME, (jx, 0.30, jz), (jx, 0.85, jz), 0.04, 12))
    m.add(n, box_span(FRAME, jx - 0.05, jx + 0.05, 0.40, 0.48, jz + 0.04, jz + 0.10))
    m.add(n, cylinder(FRAME, 0.05, 0.08, 12).at((jx, 0.87, jz)))
    m.add(n, cyl_between('steel', (jx, 0.90, jz), (jx, 0.90, jz - 0.15), 0.01, 6))
    m.add(n, cyl_between('steel', (jx, 0.90, jz - 0.15), (jx + 0.12, 0.90, jz - 0.15), 0.01, 6))
    m.add(n, cylinder('plastic_black', 0.016, 0.07, 8).rotate('z', 90).at((jx + 0.15, 0.90, jz - 0.15)))
    leg = m.node('jack_leg_lift', n, (jx, 0.02, jz))
    m.add(leg, cyl_between('steel', (jx, 0.03, jz), (jx, 0.40, jz), 0.03, 12))
    m.add(leg, cylinder('steel_dark', 0.10, 0.03, 16).at((jx, 0.015, jz)))


def gates(m):
    n = 'chassis'
    # rear swing gate: full width, hinged on the left rear post (z = -HZ + 0.08)
    hz = -HZ + 0.06
    g = m.node('rear_gate_rotate', n, (X0 - 0.10, 0, hz))
    gw = 2 * HZ - 0.14
    gx = X0 - 0.11
    m.add(g, box_span(AL, gx - 0.03, gx + 0.03, YF - 0.10, YR - 0.12, hz + 0.02, hz + 0.08))
    m.add(g, box_span(AL, gx - 0.03, gx + 0.03, YF - 0.10, YR - 0.12, hz + gw - 0.06, hz + gw))
    for y in (YF - 0.10, YF + 0.80, YR - 0.18):
        m.add(g, box_span(AL, gx - 0.03, gx + 0.03, y, y + 0.06, hz + 0.08, hz + gw - 0.06))
    m.add(g, box_span(AL, gx - 0.012, gx + 0.012, YF - 0.04, YF + 0.80, hz + 0.08, hz + gw - 0.06))
    y = YF + 0.92
    while y + 0.12 < YR - 0.20:
        m.add(g, box_span(AL, gx - 0.012, gx + 0.012, y, y + 0.10, hz + 0.08, hz + gw * 0.5))
        y += 0.20
    for yh in (YF + 0.10, YR - 0.30):
        m.add(g, cyl_between('steel', (X0 - 0.10, yh - 0.06, hz), (X0 - 0.10, yh + 0.06, hz), 0.02, 8))
        m.add(g, box_span('steel', gx - 0.035, gx + 0.035, yh - 0.03, yh + 0.03, hz, hz + 0.25))
    # slam latch bar and handle on the free edge
    m.add(g, cyl_between('steel', (gx - 0.05, YF + 1.0, hz + gw - 0.30), (gx - 0.05, YF + 1.0, hz + gw + 0.06), 0.012, 8))
    m.add(g, box_span('steel', gx - 0.09, gx - 0.04, YF + 0.92, YF + 1.08, hz + gw - 0.34, hz + gw - 0.30))
    m.add(g, reflector((gx - 0.03, YF + 0.60, hz + 0.30), -1, 'x', 'lamp_red', 0.06))
    m.add(g, reflector((gx - 0.03, YF + 0.60, hz + gw - 0.30), -1, 'x', 'lamp_red', 0.06))
    # slider half-gate in the upper right of the swing gate (slides toward -z)
    sl = m.node('rear_gate_slider_slide', g, (gx, YF + 1.30, hz + gw * 0.75))
    for y in (YF + 0.88, YR - 0.24):
        m.add(g, box_span('steel', gx - 0.05, gx - 0.035, y, y + 0.04, hz + gw * 0.5, hz + gw - 0.06))
    m.add(sl, box_span(AL, gx - 0.032, gx - 0.016, YF + 0.92, YR - 0.24, hz + gw * 0.5 + 0.02, hz + gw - 0.08))
    for y in (YF + 1.05, YF + 1.30, YF + 1.55, YF + 1.80):
        m.add(sl, box_span(AL, gx - 0.036, gx - 0.032, y, y + 0.03, hz + gw * 0.5 + 0.04, hz + gw - 0.10))
    m.add(sl, box_span('steel', gx - 0.07, gx - 0.036, YF + 1.30, YF + 1.40, hz + gw * 0.5 + 0.06, hz + gw * 0.5 + 0.09))
    # centre (divider) gate hinged on the right wall at mid length, closed across the box
    cx = (X0 + X1) / 2
    zh = HZ - T - 0.02
    c = m.node('center_gate_rotate', n, (cx, 0, zh))
    zf = -HZ + T + 0.03
    for z in (zh - 0.04, zf + 0.03):
        m.add(c, box_span(AL, cx - 0.03, cx + 0.03, YF + 0.03, YR - 0.10, z - 0.03, z + 0.03))
    for y in (YF + 0.05, YF + 0.50, YF + 0.95, YF + 1.40, YR - 0.16):
        m.add(c, box_span(AL, cx - 0.025, cx + 0.025, y, y + 0.06, zf + 0.06, zh - 0.07))
    for y in (YF + 0.40, YR - 0.40):
        m.add(c, cyl_between('steel', (cx, y - 0.06, zh), (cx, y + 0.06, zh), 0.018, 8))
    m.add(c, cyl_between('steel', (cx, YF + 1.10, zf), (cx, YF + 1.10, zf - 0.025), 0.012, 8))
    m.add(n, box_span('steel', cx - 0.06, cx + 0.06, YF + 1.06, YF + 1.14, -HZ + T, -HZ + T + 0.006))


def build():
    m = Model('Stock Trailer', FARM_MATERIALS)
    m.node('chassis')
    body(m)
    running_gear(m)
    tongue(m)
    gates(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'stock-trailer', 25000)
