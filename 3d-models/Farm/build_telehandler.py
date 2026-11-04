"""Farm telehandler (agri spec), ~6 m lift height, four-wheel steer, grain bucket.

Key real-world dimensions used (metres):
  wheelbase 2.80, track 1.88, 460/70R24 tyres (1.26 dia), ground clearance ~0.42
  two-stage boom pivoting at the rear (1.95 high): 4.1 m retracted, +1.9 m telescope
  -> ~6.0 m lift height at ~62 deg; side cab on the left, engine on the right
  overall ~4.6 long (to the bucket lip, boom lowered), 2.31 wide, 2.50 high

Rest pose: travel. Boom lowered (-11 deg) and retracted, carriage levelled, wheels straight.
Rig: `boom_tilt` (about z at the rear pivot) <- `boom_lift_cylinder`; `boom_stage_slide`
(telescope along the boom, local +x); `carriage_tilt` (about z at the boom nose) <-
`carriage_tilt_cylinder` and carrying the grain bucket on a quick-hitch carriage; all four
wheels `wheel_{front,rear}_{left,right}_steer` -> `_spin` (four-wheel / crab steer).
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, WHEEL, ag_wheel, beacon, handrail, hose, lamp, mirror, place,
                        rear_light, reflector, save_farm, step)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude
from dozer_parts import exhaust
from vehicle_parts import arch_polygon, loader_bucket, steering_wheel

AX_F, AX_R = 1.25, -1.55
R, W, B, ZW = 0.63, 0.46, 0.305, 0.94
KP = 0.62
P = (-1.85, 1.95, 0.20)       # boom pivot
BL = 4.10                     # retracted boom length to the nose pin
BW, BH = 0.34, 0.42           # outer boom section
DOWN = -11.0
CAB = dict(x0=-0.95, x1=0.62, y0=0.95, y1=2.40, z0=-1.10, z1=-0.28)


def chassis(m):
    p = 'chassis'
    # central box frame with axle mounts
    m.add(p, box_span(FRAME, -2.05, 1.65, 0.48, 0.95, -0.36, 0.36))
    m.add(p, box_span(FRAME, -2.20, -1.85, 0.40, 1.30, -0.80, 0.80))                    # rear counterweight
    m.add(p, box_span(BODY, -2.24, -2.20, 0.55, 1.20, -0.70, 0.70))
    m.add(p, box_span(FRAME, 1.55, 1.80, 0.45, 0.85, -0.55, 0.55))                      # front tow / bumper
    # rear pick-up hitch and drawbar pin, lights and reflectors on the counterweight
    m.add(p, box_span('steel_dark', -2.40, -2.23, 0.52, 0.70, -0.15, 0.15))
    m.add(p, cyl_between('steel', (-2.33, 0.50, 0), (-2.33, 0.78, 0), 0.025, 10))
    for s in (1, -1):
        m.add(p, rear_light((-2.265, 1.10, s * 0.55), -1))
        m.add(p, reflector((-2.24, 0.75, s * 0.55), -1, 'x', 'lamp_red', 0.07))
    # boom pivot tower
    for s in (1, -1):
        m.add(p, box_span(BODY, P[0] - 0.25, P[0] + 0.35, 0.95, P[1] + 0.12, P[2] + s * (BW / 2 + 0.02), P[2] + s * (BW / 2 + 0.07)))
    m.add(p, cyl_between('steel', (P[0], P[1], P[2] - BW / 2 - 0.10), (P[0], P[1], P[2] + BW / 2 + 0.10), 0.05, 14))
    # engine bay on the right, below and beside the boom
    hz0, hz1 = 0.42, 1.12
    m.add(p, box_span(BODY, -1.85, 0.90, 0.80, 1.55, hz0, hz1))
    m.add(p, box_span(BODY, -1.85, 0.90, 1.55, 1.58, hz0 + 0.02, hz1 - 0.02))
    for k in range(7):
        x = -1.55 + 0.12 * k
        m.add(p, box_span(FRAME, x, x + 0.06, 0.95, 1.45, hz1, hz1 + 0.006))
    m.add(p, box_span('steel_dark', 0.88, 0.904, 0.90, 1.45, hz0 + 0.06, hz1 - 0.06))     # front grille
    m.add(p, exhaust(-0.60, 1.58, 2.05, 0.95, 0.045))
    m.add(p, box_span('steel_dark', 0.40, 0.70, 1.58, 1.61, 0.55, 0.80))                 # air intake cap
    # left side: fuel and hydraulic tanks, steps, cab mounts
    m.add(p, box_span(BODY, -1.85, -1.00, 0.60, 1.05, -1.12, -0.36))
    m.add(p, cyl_between('steel_dark', (-1.40, 1.05, -0.95), (-1.40, 1.10, -0.95), 0.05, 12))
    m.add(p, step(-0.45, -0.05, 0.62, -1.15, -0.85))
    m.add(p, box_span(FRAME, -0.48, -0.45, 0.55, 0.95, -1.15, -0.85))
    m.add(p, box_span(FRAME, -0.05, -0.02, 0.55, 0.95, -1.15, -0.85))
    # axles, steering rams, wheels (four-wheel steer)
    front = ag_wheel(R, W, B, lugs=18, segs=40)
    for ax, nm in ((AX_F, 'front'), (AX_R, 'rear')):
        m.add(p, cyl_between(FRAME, (ax, R, -KP + 0.06), (ax, R, KP - 0.06), 0.10, 16))
        m.add(p, box_span(FRAME, ax - 0.20, ax + 0.20, R - 0.18, R + 0.18, -0.24, 0.24))   # differential
        m.add(p, box_span(FRAME, ax - 0.12, ax + 0.12, R + 0.10, 0.50, -0.30, 0.30))
        m.add(p, cyl_between(BODY, (ax + 0.18, R - 0.05, -0.45), (ax + 0.18, R - 0.05, 0.25), 0.04, 12))     # steering ram
        m.add(p, cyl_between('chrome', (ax + 0.18, R - 0.05, 0.25), (ax + 0.18, R - 0.05, 0.50), 0.022, 10))
        for s, side in ((1, 'right'), (-1, 'left')):
            st = m.node('wheel_%s_%s_steer' % (nm, side), p, (ax, R, s * KP))
            m.add(st, cyl_between(FRAME, (ax, R - 0.14, s * KP), (ax, R + 0.14, s * KP), 0.09, 14))
            m.add(st, cyl_between(FRAME, (ax, R, s * KP), (ax, R, s * (ZW - 0.12)), 0.14, 18))
            m.add(st, cyl_between(FRAME, (ax, R, s * (ZW - 0.16)), (ax, R, s * (ZW - 0.04)), 0.20, 18))
            # mudguard turning with the wheel
            m.add(st, extrude(BODY, arch_polygon(ax, R, R + 0.07, R + 0.10, 25, 155, 14), 0.50).move(z=s * ZW))
            m.add(st, box_between(FRAME, (ax, R + 0.14, s * KP), (ax, R + 0.66, s * (ZW - 0.24)), 0.05, 0.04))
            sp = m.node('wheel_%s_%s_spin' % (nm, side), st, (ax, R, s * ZW))
            m.add(sp, place(front, (ax, R, s * ZW), s))
    # front lights on the guards
    m.add(p, reflector((1.80, 0.70, 0.40), 1, 'x', 'lamp_lens', 0.07))


def cab(m):
    c = m.node('cab', 'chassis')
    x0, x1, y0, y1, z0, z1 = CAB['x0'], CAB['x1'], CAB['y0'], CAB['y1'], CAB['z0'], CAB['z1']
    zc = (z0 + z1) / 2
    p = 0.06
    m.add(c, box_span(FRAME, x0, x1, y0 - 0.05, y0, z0, z1))
    m.add(c, box_span(BODY, x0 + p, x1 - p, y0, y0 + 0.30, z1 - 0.03, z1))                       # boom-side lower panel
    m.add(c, box_span(BODY, x0, x0 + 0.03, y0, y0 + 0.55, z0 + p, z1 - p))
    for px in (x0 + p / 2, x1 - p / 2):
        for pz in (z0 + p / 2, z1 - p / 2):
            m.add(c, box(FRAME, p, y1 - y0, p, (px, (y0 + y1) / 2, pz)))
    m.add(c, box_span(FRAME, x0, x1, y1 - 0.06, y1, z0, z0 + p))
    m.add(c, box_span(FRAME, x0, x1, y1 - 0.06, y1, z1 - p, z1))
    m.add(c, box_span(BODY, x0 - 0.08, x1 + 0.10, y1, y1 + 0.08, z0 - 0.04, z1 + 0.03))      # roof
    m.add(c, box_span('glass_tinted', x0 + 0.10, x1 - 0.10, y1 + 0.08, y1 + 0.09, z0 + 0.10, z1 - 0.10))
    # glazing: windscreen, roof window above, rear, full-height door glass, boom-side glass
    m.add(c, box_span('glass', x1 - 0.02, x1 - 0.008, y0 + 0.35, y1 - 0.06, z0 + p, z1 - p))
    m.add(c, box_span('glass', x0 + 0.008, x0 + 0.02, y0 + 0.55, y1 - 0.06, z0 + p, z1 - p))
    m.add(c, box_span('glass', x0 + p, x1 - p, y0 + 0.30, y1 - 0.06, z1 - 0.018, z1 - 0.006))
    m.add(c, box_span('glass', x0 + p, x1 - p, y0 + 0.05, y1 - 0.06, z0 + 0.006, z0 + 0.018))
    m.add(c, box_span(BODY, x1 - 0.03, x1, y0, y0 + 0.35, z0 + p, z1 - p))
    # door frame, handle and hinges on the left
    for a in (x0 + 0.40, x1 - 0.08):
        m.add(c, box_span('rubber_trim', a - 0.012, a + 0.012, y0 + 0.04, y1 - 0.08, z0 - 0.006, z0))
    m.add(c, box_span('steel_dark', x0 + 0.45, x0 + 0.58, y0 + 0.95, y0 + 0.98, z0 - 0.03, z0 - 0.006))
    m.add(c, handrail([(x1 + 0.03, y0 + 0.25, z0 + 0.10), (x1 + 0.03, y0 + 0.25, z0 - 0.05), (x1 + 0.03, y1 - 0.40, z0 - 0.05),
                       (x1 + 0.03, y1 - 0.40, z0 + 0.10)], 0.014, 'chrome'))
    # wipers, mirrors, lights, beacon
    m.add(c, box('rubber_trim', 0.012, 0.55, 0.022, (x1 + 0.002, y0 + 0.65, zc)).rotate('x', 20, (x1, y0 + 0.40, zc)))
    m.add(c, mirror((x1 - 0.03, y1 - 0.20, z0), (x1 + 0.15, y1 - 0.15, z0 - 0.12)))
    for z in (z0 + 0.12, z1 - 0.12):
        m.add(c, lamp((x1 + 0.12, y1 + 0.13, z), 1, 0.8))
        m.add(c, lamp((x0 - 0.10, y1 + 0.13, z), -1, 0.8))
    m.add(c, beacon((x0 + 0.02, y1 + 0.08, z0 + 0.04)))
    # interior: seat, steering column, joystick console
    sx = x0 + 0.62
    m.add(c, box_span(FRAME, sx - 0.18, sx + 0.18, y0 + 0.01, y0 + 0.32, zc - 0.18, zc + 0.18))
    m.add(c, box_span('seat_fabric', sx - 0.24, sx + 0.24, y0 + 0.32, y0 + 0.44, zc - 0.24, zc + 0.24))
    m.add(c, box('seat_fabric', 0.10, 0.60, 0.46, (sx - 0.27, y0 + 0.74, zc)).rotate('z', 10, (sx - 0.27, y0 + 0.44, zc)))
    m.add(c, box_span(FRAME, x1 - 0.30, x1 - 0.03, y0 + 0.01, y0 + 0.75, zc - 0.18, zc + 0.18))
    m.add(c, steering_wheel((x1 - 0.42, y0 + 0.95, zc), 0.18, 35))
    m.add(c, box_span(FRAME, sx - 0.25, sx + 0.25, y0 + 0.01, y0 + 0.55, z1 - 0.22, z1 - 0.07))
    m.add(c, cyl_between('steel_dark', (sx + 0.12, y0 + 0.55, z1 - 0.15), (sx + 0.14, y0 + 0.70, z1 - 0.15), 0.012, 8))
    m.add(c, box('plastic_black', 0.07, 0.10, 0.06, (sx + 0.14, y0 + 0.75, z1 - 0.15)))


def boom(m):
    b = m.node('boom_tilt', 'chassis', P)
    px, py, pz = P
    xt = px + BL
    # outer section: tapered heel around the pivot, box section to the nose
    m.add(b, extrude(BODY, [(px - 0.25, py - 0.10), (px + 0.10, py - 0.24), (xt - 1.95, py - BH / 2), (xt - 1.95, py + BH / 2),
                            (px + 0.10, py + 0.22), (px - 0.25, py + 0.14)], BW).move(z=pz))
    m.add(b, box_span(BODY, xt - 1.95, xt - 0.35, py - BH / 2, py + BH / 2, pz - BW / 2, pz + BW / 2))
    m.add(b, box_span(FRAME, xt - 0.40, xt - 0.33, py - BH / 2 - 0.01, py + BH / 2 + 0.01, pz - BW / 2 - 0.01, pz + BW / 2 + 0.01))
    m.add(b, cyl_between('steel_dark', (px, py, pz - BW / 2 - 0.02), (px, py, pz + BW / 2 + 0.02), 0.09, 16))
    # lift cylinder lug under the outer section
    lug = (px + 2.15, py - BH / 2 - 0.10)
    m.add(b, box_span(BODY, lug[0] - 0.12, lug[0] + 0.12, lug[1] - 0.04, py - BH / 2 + 0.02, pz - 0.12, pz + 0.12))
    # boom angle indicator, hose carrier along the side
    m.add(b, box_span('paint_white', px + 0.30, px + 0.60, py - 0.06, py + 0.06, pz - BW / 2 - 0.006, pz - BW / 2))
    # lift cylinder from the chassis
    m.add('chassis', box_span(FRAME, -0.90, -0.60, 0.85, 1.00, pz - 0.14, pz + 0.14))
    m.cylinder('boom_lift_cylinder', 'chassis', b, (-0.75, 0.95, pz), (lug[0], lug[1], pz), 0.085, 0.055, 0.6, barrel=BODY)
    # telescopic inner stage (slides along the boom's local +x)
    st = m.node('boom_stage_slide', b, (xt - 0.35, py, pz))
    iw, ih = BW - 0.06, BH - 0.06
    m.add(st, box_span(BODY, xt - 2.00, xt - 0.05, py - ih / 2, py + ih / 2, pz - iw / 2, pz + iw / 2))
    m.add(st, box_span(FRAME, xt - 0.20, xt + 0.08, py - ih / 2 - 0.06, py + ih / 2 + 0.01, pz - iw / 2 - 0.02, pz + iw / 2 + 0.02))
    m.add(st, box_span('wear_steel', xt - 0.31, xt - 0.25, py + ih / 2, py + ih / 2 + 0.008, pz - iw / 2 + 0.03, pz + iw / 2 - 0.03))
    # carriage tilt cylinder lug on top of the stage
    m.add(st, box_span(FRAME, xt - 0.95, xt - 0.75, py + ih / 2, py + ih / 2 + 0.12, pz - 0.06, pz + 0.06))
    # hoses from the chassis along the boom to the nose
    for k, dz in enumerate((-0.05, 0.05)):
        m.add(b, hose([(px + 0.15, py + BH / 2 + 0.03, pz + dz), (xt - 1.00, py + BH / 2 + 0.03, pz + dz)], 0.012))
        m.add(st, hose([(xt - 1.02, py + ih / 2 + 0.06, pz + dz), (xt - 0.20, py + ih / 2 + 0.05, pz + dz),
                        (xt + 0.02, py + 0.10, pz + dz + (0.12 if k else -0.12))], 0.012))
        m.add('chassis', hose([(-1.20, 1.00, pz + dz), (px + 0.40, py - 0.30, pz + dz), (px + 0.15, py + BH / 2 + 0.03, pz + dz)], 0.012))
    m.add(st, lamp((xt - 0.10, py + ih / 2 + 0.10, pz + iw / 2 - 0.05), 1, 0.7))
    # carriage on the nose pin: quick-hitch frame and a high-volume grain bucket
    nose = (xt + 0.02, py - 0.10, pz)
    c = m.node('carriage_tilt', st, nose)
    m.add(c, cyl_between('steel_dark', (nose[0], nose[1], pz - iw / 2 - 0.05), (nose[0], nose[1], pz + iw / 2 + 0.05), 0.045, 12))
    cx = nose[0] + 0.12
    for s in (1, -1):
        m.add(c, box_span(FRAME, nose[0] - 0.06, cx + 0.04, nose[1] - 0.05, nose[1] + 0.35, pz + s * (iw / 2 + 0.06), pz + s * (iw / 2 + 0.10)))
    m.add(c, box_span(FRAME, cx, cx + 0.08, nose[1] - 0.70, nose[1] + 0.40, -0.60, 0.60))
    m.add(c, box_span(FRAME, cx - 0.08, cx, nose[1] + 0.30, nose[1] + 0.40, pz - 0.10, pz + 0.10))
    m.add(c, cyl_between('steel', (cx + 0.04, nose[1] - 0.66, -0.70), (cx + 0.04, nose[1] - 0.66, 0.70), 0.025, 10))
    m.add(c, [g.move(x=cx + 0.08, y=nose[1] - 0.95, z=0.0) for g in loader_bucket(2.30, 1.05, 1.15, BODY)])
    m.cylinder('carriage_tilt_cylinder', st, c, (xt - 0.85, py + ih / 2 + 0.07, pz), (nose[0] + 0.02, nose[1] + 0.35, pz),
               0.06, 0.04, 0.55, barrel=BODY)
    m.pose(b, 'z', DOWN)
    m.pose(c, 'z', -DOWN)


def build():
    m = Model('Telehandler', FARM_MATERIALS)
    m.node('chassis')
    chassis(m)
    cab(m)
    boom(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'telehandler', 60000)
