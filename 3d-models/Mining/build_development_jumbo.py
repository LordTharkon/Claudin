"""Twin-boom development jumbo (face drilling rig), articulated diesel-electric carrier.

Key real-world dimensions used (metres):
  front axle to hitch 1.50, hitch to rear axle 2.40, tread 1.90, 12.00-24-class tyres
  (1.24 dia x 0.34 wide)
  two booms on swing brackets at the front of the carrier, 4.5 m feeds (14 ft rods)
  lying along the booms for tramming, rock drill at the back of each feed
  centre FOPS canopy behind the booms, roof at 2.85; rear cable reel ~1.9 m diameter
  overall ~14.0 x 2.5 x 3.0
"""
import math

from mining_parts import (MINING_MATERIALS, Model, box, box_span, cyl_between, cylinder, extrude,
                          hazard_stripes, hose, lamp, lathe, louvres, precleaner, save, sight_gauge,
                          sphere, tank_cap, underground_cab)
from vehicle_parts import arch_polygon, place_wheel, wheel

AX_F, AX_R = 1.50, -2.40
TR, TW, BEAD = 0.62, 0.34, 0.30
WY = TR
ZW = 0.95
HX, HY = 0.0, 0.70
BOOM_Z = 0.62
SWING = (3.20, 1.30)          # boom swing pivot (vertical axis)
LIFT = (3.42, 1.575)          # boom lift hinge
TIP_X = 6.30                  # feed cradle at the boom tip
FEED_Y = 1.90                 # feed beam centre line
FEED_X0, FEED_X1 = 2.62, 7.10
REEL = (-5.90, 1.46)
CANOPY = (0.55, 2.47, -1.20, 0.22)


def along_x(poly, x0, x1, mat='paint_yellow'):
    """Extrude a (z, y) cross-section along x (poly x maps to -z, so pass symmetric shapes)."""
    return extrude(mat, poly, x1 - x0).rotate('y', 90).move(x=(x0 + x1) / 2)


def running_gear(m):
    w = wheel(TR, TW, BEAD, 'chevron', 'otr', lugs=18, depth=0.04)
    for ax, tag, par in ((AX_F, 'front', 'front_frame'), (AX_R, 'rear', 'hitch_rotate')):
        for s, side in ((1, 'right'), (-1, 'left')):
            n = m.node('wheel_%s_%s_spin' % (tag, side), par, (ax, WY, s * ZW))
            m.add(n, place_wheel(w, (ax, WY, s * ZW), s))
            m.add(par, cyl_between('chassis_black', (ax, WY, s * (ZW - 0.30)), (ax, WY, s * (ZW - 0.18)), 0.19, 20))
        m.add(par, cyl_between('chassis_black', (ax, WY, -ZW + 0.28), (ax, WY, ZW - 0.28), 0.10, 16))
        m.add(par, sphere('chassis_black', 0.21, 16, 8).scale(1, 0.85, 0.9).at((ax, WY, 0)))
        for s in (1, -1):
            m.add(par, extrude('paint_yellow', arch_polygon(ax, WY, TR + 0.05, TR + 0.08, 20, 160, 14), TW + 0.08).move(z=s * ZW))


def front_frame(m):
    p = 'front_frame'
    m.add(p, box_span('chassis_black', 0.30, 3.29, 0.33, 1.01, -0.45, 0.45))
    for y in (0.42, 0.90):   # hitch tongues
        m.add(p, box_span('chassis_black', HX - 0.20, 0.31, y, y + 0.10, -0.30, 0.30))
    for y0, y1 in ((0.36, 0.58), (0.84, 1.06)):
        m.add(p, cylinder('steel', 0.08, y1 - y0, 16).at((HX, (y0 + y1) / 2, 0)))
    # deck under the canopy (over the front tyres) and boom mounting block
    m.add(p, box_span('paint_yellow', 0.40, 2.60, 1.33, 1.48, -1.20, 1.20))
    m.add(p, box_span('paint_yellow', 0.40, 2.60, 1.01, 1.33, -0.70, 0.70))
    m.add(p, box_span('paint_yellow', 2.60, 3.30, 1.02, 1.22, -0.90, 0.90))
    m.add(p, box_span('paint_yellow', 2.85, 3.30, 0.60, 1.02, -0.90, 0.90))
    # right side of the canopy: hydraulic tank and control cabinet
    m.add(p, box_span('paint_yellow', 0.60, 1.70, 1.48, 2.30, 0.40, 1.15))
    m.add(p, tank_cap(1.00, 2.30, 0.80, 0.06))
    m.add(p, sight_gauge(1.30, 1.65, 2.15, 1.15, 1))
    m.add(p, box_span('paint_gray_dark', 1.80, 2.40, 1.48, 2.25, 0.50, 1.10))
    m.add(p, box_span('chassis_black', 1.85, 2.35, 1.58, 2.15, 1.10, 1.115))
    m.add(p, box_span('steel', 2.25, 2.29, 1.80, 1.95, 1.115, 1.14))
    # front nose: lamps, indicators, hazard stripes facing forward
    m.add(p, box_span('paint_yellow', 3.30, 3.45, 0.32, 0.98, -0.90, 0.90))
    for s in (1, -1):
        zt = (0.88 if s > 0 else -0.30)
        m.add(p, hazard_stripes((3.45, 0.36, zt), (0, 0, -1), (0, 1, 0), 0.58, 0.58, 0.10))
        m.add(p, box('lamp_amber', 0.02, 0.06, 0.12, (3.46, 0.88, s * 0.12)))
        m.add(p, lamp((3.52, 0.62, s * 0.18), 1, 0.6))
    # stabiliser jack housings
    for s in (1, -1):
        z = s * 1.02
        m.add(p, box_span('paint_yellow', 2.88, 3.22, 0.70, 1.22, s * 0.90, s * 1.20))
        m.add(p, box_span('paint_yellow', 2.95, 3.15, 0.50, 1.45, z - 0.10, z + 0.10))
        m.add(p, cyl_between('paint_yellow', (3.05, 1.45, z), (3.05, 1.80, z), 0.06, 12))
        m.add(p, cylinder('steel_dark', 0.07, 0.03, 12).at((3.05, 1.815, z)))
        lift = m.node('stabiliser_%s_lift' % ('right' if s > 0 else 'left'), p, (3.05, 0.50, z))
        m.add(lift, box_span('steel', 2.98, 3.12, 0.22, 0.50, z - 0.07, z + 0.07))
        m.add(lift, box_span('steel_dark', 2.88, 3.22, 0.16, 0.22, z - 0.15, z + 0.15))
    for s in (1, -1):
        m.add(p, hose([(2.20, 1.50, s * 0.30), (2.75, 1.36, s * 0.30), (3.00, 1.40, s * 0.45)], 0.03))
        m.add(p, hose([(2.20, 1.50, s * 0.18), (2.80, 1.45, s * 0.20), (3.10, 1.60, s * 0.40)], 0.03))
    m.add('canopy', underground_cab(*CANOPY, floor_y=1.48, roof_y=2.85, tub_y=1.40, enclosed=False))


def boom(m, s):
    side = 'right' if s > 0 else 'left'
    z = s * BOOM_Z
    rot = m.node('boom_%s_rotate' % side, 'front_frame', (SWING[0], SWING[1], z))
    tilt = m.node('boom_%s_tilt' % side, rot, (LIFT[0], LIFT[1], z))
    feed = m.node('feed_%s_slide' % side, tilt, (TIP_X, FEED_Y, z))
    drill = m.node('rock_drill_%s_slide' % side, feed, (FEED_X0 + 0.40, FEED_Y + 0.18, z))
    # swing bracket: turntable on the mounting block and a clevis for the lift hinge
    m.add(rot, cylinder('steel_dark', 0.22, 0.06, 20).at((SWING[0], 1.25, z)))
    m.add(rot, box_span('paint_yellow', 3.05, 3.60, 1.28, 1.48, z - 0.17, z + 0.17))
    for dz in (-0.14, 0.14):
        m.add(rot, box_span('paint_yellow', 3.25, 3.60, 1.28, 1.80, z + dz - 0.03, z + dz + 0.03))
    m.add(rot, box_span('paint_yellow', 3.45, 3.70, 0.95, 1.28, z - 0.10, z + 0.10))      # lift-cylinder lug
    m.add(rot, cyl_between('steel_dark', (LIFT[0], LIFT[1], z - 0.20), (LIFT[0], LIFT[1], z + 0.20), 0.05, 12))
    m.cylinder('boom_%s_swing_cylinder' % side, 'front_frame', rot, (2.62, 1.36, s * 0.24), (3.48, 1.36, s * 0.40),
               0.06, 0.035, extension=0.55)
    # boom: square telescopic tube, inner section, tip bracket and feed cradle
    m.add(tilt, cyl_between('paint_yellow', (LIFT[0], LIFT[1], z - 0.10), (LIFT[0], LIFT[1], z + 0.10), 0.11, 16))
    m.add(tilt, box_span('paint_yellow', LIFT[0], 5.30, 1.46, 1.69, z - 0.10, z + 0.10))
    m.add(tilt, box_span('paint_yellow', 5.30, 5.42, 1.44, 1.71, z - 0.12, z + 0.12))
    m.add(tilt, box_span('steel', 5.42, TIP_X - 0.12, 1.48, 1.67, z - 0.08, z + 0.08))
    m.add(tilt, box_span('paint_yellow', TIP_X - 0.12, TIP_X + 0.12, 1.42, 1.80, z - 0.14, z + 0.14))
    m.add(tilt, cyl_between('steel_dark', (TIP_X, 1.55, z - 0.17), (TIP_X, 1.55, z + 0.17), 0.07, 12))
    m.add(tilt, box_span('paint_yellow', 4.55, 4.80, 1.30, 1.46, z - 0.08, z + 0.08))    # lift-cylinder lug
    m.add(tilt, hose([(3.80, 1.72, z + 0.06), (5.20, 1.72, z + 0.06), (5.60, 1.71, z + 0.04), (TIP_X - 0.15, 1.74, z + 0.06)], 0.022))
    m.add(tilt, hose([(3.80, 1.72, z - 0.06), (5.20, 1.72, z - 0.06), (5.60, 1.71, z - 0.04), (TIP_X - 0.15, 1.74, z - 0.06)], 0.022))
    m.add(tilt, lamp((5.36, 1.82, z), 1, 0.55))
    m.cylinder('boom_%s_lift_cylinder' % side, rot, tilt, (3.58, 1.05, z), (4.68, 1.36, z), 0.075, 0.045, extension=0.6)
    # feed: aluminium beam with rails, front centraliser, rear hose guide, drill rod
    m.add(feed, box_span('aluminum', FEED_X0 + 0.005, FEED_X1 - 0.01, FEED_Y - 0.075, FEED_Y + 0.07, z - 0.085, z + 0.085))
    for dz in (-0.075, 0.075):
        m.add(feed, box_span('steel', FEED_X0 + 0.05, FEED_X1 - 0.30, FEED_Y + 0.07, FEED_Y + 0.09, z + dz - 0.012, z + dz + 0.012))
    m.add(feed, box_span('paint_yellow', TIP_X - 0.30, TIP_X + 0.30, FEED_Y - 0.12, FEED_Y - 0.08, z - 0.14, z + 0.14))
    m.add(feed, box_span('paint_yellow', FEED_X1 - 0.25, FEED_X1, FEED_Y - 0.10, FEED_Y + 0.26, z - 0.12, z + 0.12))
    m.add(feed, box_span('rubber_trim', FEED_X1, FEED_X1 + 0.06, FEED_Y + 0.08, FEED_Y + 0.24, z - 0.07, z + 0.07))
    m.add(feed, box_span('steel_dark', FEED_X1 - 0.65, FEED_X1 - 0.45, FEED_Y + 0.09, FEED_Y + 0.27, z - 0.09, z + 0.09))
    m.add(feed, cyl_between('steel', (FEED_X0 + 0.70, FEED_Y + 0.18, z), (FEED_X1 - 0.05, FEED_Y + 0.18, z), 0.019, 8))
    m.add(feed, box_span('steel_dark', FEED_X0, FEED_X0 + 0.08, FEED_Y - 0.08, FEED_Y + 0.35, z - 0.09, z + 0.09))
    # rock drill on its carriage
    m.add(drill, box_span('steel_dark', FEED_X0 + 0.08, FEED_X0 + 0.72, FEED_Y + 0.09, FEED_Y + 0.13, z - 0.10, z + 0.10))
    m.add(drill, box_span('paint_red', FEED_X0 + 0.14, FEED_X0 + 0.62, FEED_Y + 0.13, FEED_Y + 0.31, z - 0.09, z + 0.09))
    m.add(drill, cyl_between('steel_dark', (FEED_X0 + 0.62, FEED_Y + 0.18, z), (FEED_X0 + 0.72, FEED_Y + 0.18, z), 0.05, 12))
    m.add(drill, box_span('chassis_black', FEED_X0 + 0.24, FEED_X0 + 0.40, FEED_Y + 0.31, FEED_Y + 0.36, z - 0.05, z + 0.05))
    m.add(drill, hose([(FEED_X0 + 0.20, FEED_Y + 0.31, z + 0.04), (FEED_X0 + 0.02, FEED_Y + 0.45, z + 0.04),
                       (FEED_X0 - 0.10, FEED_Y + 0.40, z + 0.04)], 0.022))
    m.add(drill, hose([(FEED_X0 + 0.30, FEED_Y + 0.31, z - 0.04), (FEED_X0 + 0.05, FEED_Y + 0.48, z - 0.04),
                       (FEED_X0 - 0.10, FEED_Y + 0.42, z - 0.04)], 0.022))


def rear_unit(m):
    p = 'hitch_rotate'
    m.add(p, box_span('chassis_black', -6.60, -0.30, 0.32, 1.02, -0.45, 0.45))
    for y in (0.28, 0.66, 1.10):   # hitch clevis plates
        m.add(p, box_span('chassis_black', -0.31, HX + 0.20, y, y + 0.10, -0.36, 0.36))
    # power pack (hydraulic pumps, compressor) between the hitch and the rear axle
    m.add(p, box_span('paint_yellow', -1.70, -0.40, 1.02, 1.90, -1.00, 1.00))
    for s in (1, -1):
        m.add(p, louvres(-1.55, -0.60, 1.25, 1.75, s * 1.00, s, pitch=0.06))
    m.add(p, box_span('paint_gray_dark', -1.55, -0.70, 1.90, 2.30, -0.80, 0.10))   # electrical cabinet
    m.add(p, box_span('chassis_black', -1.50, -0.75, 2.30, 2.32, -0.75, 0.05))
    m.add(p, box_span('steel', -0.96, -0.88, 2.02, 2.16, 0.10, 0.12))
    # diesel engine for tramming, behind the rear axle
    hood = [(-1.05, 0.95), (1.05, 0.95), (1.05, 1.80), (0.90, 1.95), (-0.90, 1.95), (-1.05, 1.80)]
    m.add(p, along_x(hood, -4.80, -3.15))
    for s in (1, -1):
        m.add(p, louvres(-4.60, -3.40, 1.15, 1.70, s * 1.05, s, pitch=0.06))
    m.add(p, box_span('chassis_black', -4.65, -3.30, 1.945, 1.955, -0.30, 0.30))
    m.add(p, box_span('steel', -4.50, -3.70, 1.95, 2.20, 0.35, 0.85))           # exhaust scrubber box
    m.add(p, cyl_between('steel_dark', (-4.50, 2.08, 0.60), (-4.95, 2.08, 0.60), 0.05, 12))
    m.add(p, precleaner(-3.45, 1.95, -0.60, 0.7))
    m.add(p, tank_cap(-4.30, 1.95, -0.55, 0.06))
    # cable reel: side frames, guide, reel itself on cable_reel_spin
    for s in (1, -1):
        frame = [(-5.05, 1.02), (-6.75, 1.02), (-6.55, 1.70), (-5.90, 1.98), (-5.25, 1.70)]
        m.add(p, extrude('paint_yellow', frame, 0.05).move(z=s * 0.86))
        m.add(p, cyl_between('steel_dark', (REEL[0], REEL[1], s * 0.80), (REEL[0], REEL[1], s * 0.92), 0.10, 14))
    m.add(p, box_span('paint_yellow', -6.80, -6.55, 2.20, 2.32, -0.40, 0.40))
    for s in (1, -1):
        m.add(p, box_span('paint_yellow', -6.80, -6.70, 1.02, 2.20, s * 0.30, s * 0.40))
    for y in (2.26,):
        m.add(p, cyl_between('steel', (-6.68, y, -0.30), (-6.68, y, 0.30), 0.04, 10))
    reel = m.node('cable_reel_spin', p, (REEL[0], REEL[1], 0))
    m.add(reel, cyl_between('steel_dark', (REEL[0], REEL[1], -0.80), (REEL[0], REEL[1], 0.80), 0.06, 12))
    for s in (1, -1):
        m.add(reel, cyl_between('paint_yellow', (REEL[0], REEL[1], s * 0.70), (REEL[0], REEL[1], s * 0.76), 0.92, 40))
    m.add(reel, cyl_between('cable_rubber', (REEL[0], REEL[1], -0.70), (REEL[0], REEL[1], 0.70), 0.72, 40))
    for k in range(-3, 4):
        zk = k * 0.19
        m.add(reel, lathe('cable_rubber', [(0.72, -0.045), (0.75, -0.03), (0.755, 0.0), (0.75, 0.03), (0.72, 0.045)], 40)
              .rotate('x', 90).move(REEL[0], REEL[1], zk))
    for k in range(6):
        a = math.radians(60 * k)
        for s in (1, -1):
            m.add(reel, box('paint_yellow', 0.06, 0.06, 0.06).at((REEL[0] + 0.55 * math.cos(a), REEL[1] + 0.55 * math.sin(a), s * 0.79)))
    m.add(p, [g.recolor('cable_rubber') for g in hose([(-6.68, 2.20, 0.0), (-6.90, 1.95, 0.0), (-6.92, 1.40, 0.0)], 0.035)])
    # rear bumper and light posts with hazard stripes
    m.add(p, box_span('paint_yellow', -6.95, -6.60, 0.25, 0.50, -1.15, 1.15))
    m.add(p, box_span('steel_dark', -7.02, -6.95, 0.30, 0.45, -0.15, 0.15))
    for s in (1, -1):
        z = s * 1.05
        m.add(p, box_span('paint_yellow', -6.95, -6.83, 0.50, 1.75, z - 0.08, z + 0.08))
        m.add(p, box_span('paint_yellow', -6.83, -6.60, 0.50, 0.70, z - 0.06, z + 0.06))
        m.add(p, hazard_stripes((-6.95, 0.55, z - 0.06), (0, 0, 1), (0, 1, 0), 0.12, 0.85, 0.10))
        m.add(p, box('lamp_red', 0.02, 0.10, 0.10, (-6.96, 1.50, z)))
        m.add(p, box('lamp_amber', 0.02, 0.08, 0.10, (-6.96, 1.62, z)))
        m.add(p, lamp((-6.92, 1.86, z), -1, 0.6))
    for s in (1, -1):
        m.add(p, hose([(-0.40, 0.95, s * 0.30), (-0.10, 0.92, s * 0.22), (0.40, 0.90, s * 0.18)], 0.03))


def build():
    m = Model('Development Jumbo', MINING_MATERIALS)
    m.node('front_frame')
    m.node('canopy', 'front_frame')
    m.node('hitch_rotate', 'front_frame', (HX, HY, 0.0))          # articulation hitch (rear unit)
    running_gear(m)
    front_frame(m)
    for s in (1, -1):
        boom(m, s)
    rear_unit(m)
    for s in (1, -1):
        m.cylinder('steering_cylinder_%s' % ('right' if s > 0 else 'left'), 'front_frame', 'hitch_rotate',
                   (0.95, 0.80, s * 0.52), (-0.75, 0.80, s * 0.55), 0.07, 0.04, extension=0.55)
        m.add('front_frame', box_span('chassis_black', 0.85, 1.05, 0.68, 0.92, s * 0.45, s * 0.60))
        m.add('hitch_rotate', box_span('chassis_black', -0.85, -0.65, 0.68, 0.92, s * 0.45, s * 0.62))
    return m


if __name__ == '__main__':
    m = build()
    save(m, 'development-jumbo')
    print('tris', m.stats()[1])
