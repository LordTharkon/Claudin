"""Underground haul truck (articulated, ~50 t payload), low profile.

Key real-world dimensions used (metres):
  front axle to hitch 2.35, hitch to rear axle 4.05; tread 2.24; 18.00R33-class tyres
  (1.78 dia x 0.52 wide), single tyres on both axles
  ~25 m3 tray 3.0 wide, floor 1.88, sides to 2.80, tipping about pins at the rear of the frame
  low enclosed cab on the left beside the front engine bay, roof at 2.68
  overall ~10.5 x 3.0 x 2.8
"""
from mining_parts import (MINING_MATERIALS, Model, box, box_span, cyl_between, cylinder, extrude,
                          hazard_stripes, hose, lamp, louvres, precleaner, save, sight_gauge, sphere,
                          tank_cap, underground_cab)
from vehicle_parts import arch_polygon, place_wheel, wheel

AX_F, AX_R = 3.70, -2.70
TR, TW, BEAD = 0.89, 0.52, 0.42
WY = TR
ZW = 1.12
HX, HY = 1.35, 0.90
HOOD_Y = 2.20
TRAY_X0, TRAY_X1 = -5.25, 1.10
FLOOR = 1.88
TRAY_HW = 1.42
DUMP = (-4.70, 1.42)          # tray hinge pins
CAB = (1.70, 2.95, -1.46, -0.52)


def along_x(poly, x0, x1, mat='paint_yellow'):
    return extrude(mat, poly, x1 - x0).rotate('y', 90).move(x=(x0 + x1) / 2)


def running_gear(m):
    w = wheel(TR, TW, BEAD, 'chevron', 'otr', lugs=22, depth=0.06)
    for ax, tag, par in ((AX_F, 'front', 'front_frame'), (AX_R, 'rear', 'hitch_rotate')):
        for s, side in ((1, 'right'), (-1, 'left')):
            n = m.node('wheel_%s_%s_spin' % (tag, side), par, (ax, WY, s * ZW))
            m.add(n, place_wheel(w, (ax, WY, s * ZW), s))
            m.add(par, cyl_between('chassis_black', (ax, WY, s * (ZW - 0.44)), (ax, WY, s * (ZW - 0.27)), 0.27, 24))
        m.add(par, cyl_between('chassis_black', (ax, WY, -ZW + 0.42), (ax, WY, ZW - 0.42), 0.14, 20))
        m.add(par, sphere('chassis_black', 0.30, 18, 9).scale(1, 0.85, 0.9).at((ax, WY, 0)))


def front_frame(m):
    p = 'front_frame'
    m.add(p, box_span('chassis_black', 1.75, 4.95, 0.50, 1.30, -0.48, 0.48))
    for y in (0.62, 1.10):   # hitch tongues
        m.add(p, box_span('chassis_black', HX - 0.20, 1.76, y, y + 0.10, -0.32, 0.32))
    for y0, y1 in ((0.56, 0.78), (1.04, 1.26)):
        m.add(p, cylinder('steel', 0.09, y1 - y0, 18).at((HX, (y0 + y1) / 2, 0)))
    # engine bay: right of the cab, full width ahead of it, chamfered top
    hood = [(-z, y) for z, y in [(-0.45, 1.30), (1.38, 1.30), (1.38, HOOD_Y - 0.15), (1.23, HOOD_Y), (-0.30, HOOD_Y), (-0.45, HOOD_Y - 0.15)]]
    hood.reverse()   # along_x maps poly x to -z
    m.add(p, along_x(hood, 1.90, 3.05))
    full = [(-1.38, 1.85), (1.38, 1.85), (1.38, HOOD_Y - 0.15), (1.23, HOOD_Y), (-1.23, HOOD_Y), (-1.38, HOOD_Y - 0.15)]
    m.add(p, along_x(full, 3.05, 4.95))
    m.add(p, box_span('paint_yellow', 3.05, 4.95, 1.30, 1.85, -0.50, 0.50))
    m.add(p, box_span('chassis_black', 2.10, 4.70, HOOD_Y - 0.004, HOOD_Y + 0.006, -0.10, 0.10))
    m.add(p, louvres(2.05, 2.90, 1.45, 2.00, 1.38, 1))
    for s in (1, -1):
        m.add(p, louvres(3.20, 4.70, 1.92, 2.00, s * 1.38, s, pitch=0.05))
        m.add(p, extrude('paint_yellow', arch_polygon(AX_F, WY, TR + 0.05, TR + 0.09, 15, 165, 16), TW + 0.10).move(z=s * ZW))
    for x in (2.40, 4.00):
        m.add(p, box_span('chassis_black', x, x + 0.012, 1.95, HOOD_Y - 0.17, 1.379, 1.386))
    # exhaust scrubber and pre-cleaner on the hood, fuel tank under the cab side
    m.add(p, box_span('steel', 3.30, 4.30, HOOD_Y, HOOD_Y + 0.30, 0.35, 1.05))
    for x in (3.35, 4.25):
        m.add(p, box_span('steel_dark', x - 0.02, x + 0.02, HOOD_Y + 0.30, HOOD_Y + 0.32, 0.35, 1.05))
    m.add(p, cyl_between('steel_dark', (3.30, HOOD_Y + 0.15, 0.70), (2.60, HOOD_Y + 0.15, 0.70), 0.07, 14))
    m.add(p, cyl_between('steel_dark', (2.60, HOOD_Y + 0.15, 0.70), (2.45, HOOD_Y + 0.40, 0.70), 0.07, 14))
    m.add(p, precleaner(4.55, HOOD_Y, -0.70, 0.8))
    m.add(p, tank_cap(2.40, HOOD_Y, 0.20, 0.07))
    m.add(p, box_span('paint_yellow', 1.80, 2.85, 0.50, 1.10, 0.52, 1.30))      # fuel tank
    m.add(p, tank_cap(2.60, 1.10, 1.05, 0.07))
    m.add(p, sight_gauge(2.10, 0.65, 1.00, 1.30, 1))
    # front radiator grille, bumper, lamps, hazard stripes
    m.add(p, box_span('steel_dark', 4.95, 4.97, 1.40, HOOD_Y - 0.10, -1.10, 1.10))
    y = 1.46
    while y < HOOD_Y - 0.14:
        m.add(p, box('chassis_black', 0.04, 0.02, 2.16, (4.99, y, 0)).rotate('z', -20, (4.99, y, 0)))
        y += 0.07
    m.add(p, box_span('paint_yellow', 4.95, 5.25, 0.45, 1.38, -1.25, 1.25))
    m.add(p, box_span('steel_dark', 5.25, 5.33, 0.60, 0.82, -0.18, 0.18))
    for s in (1, -1):
        m.add(p, hazard_stripes((5.25, 0.48, -0.25 if s > 0 else 1.22), (0, 0, -1), (0, 1, 0), 0.97, 0.50, 0.12))
        m.add(p, lamp((5.32, 1.20, s * 0.55), 1, 0.85))
        m.add(p, box('lamp_amber', 0.02, 0.07, 0.14, (5.26, 1.20, s * 1.05)))
        m.add(p, lamp((5.02, HOOD_Y + 0.10, s * 0.95), 1, 0.8))
    # cab platform steps and hydraulic lines toward the hitch
    m.add(p, box_span('chassis_black', 1.80, 2.85, 0.50, 0.62, -1.40, -0.48))
    for s in (1, -1):
        m.add(p, hose([(1.80, 0.70, s * 0.40), (1.40, 0.72, s * 0.30), (1.05, 0.75, s * 0.22)], 0.03))
    m.add('cab', underground_cab(*CAB, floor_y=0.92, roof_y=2.58, tub_y=0.62))


def rear_frame(m):
    p = 'hitch_rotate'
    m.add(p, box_span('chassis_black', -4.85, 0.70, 0.62, 1.32, -0.50, 0.50))
    for y in (0.48, 0.86, 1.30):   # hitch clevis plates
        m.add(p, box_span('chassis_black', 0.69, HX + 0.24, y, y + 0.10, -0.40, 0.40))
    m.add(p, box_span('chassis_black', 0.40, 0.70, 0.55, 1.40, -0.75, 0.75))
    # hoist-cylinder base lugs and tray rests
    for s in (1, -1):
        m.add(p, box_span('chassis_black', 0.20, 0.55, 0.70, 1.20, s * 0.50, s * 0.85))
        m.add(p, box_span('rubber_trim', -0.20, 0.20, 1.32, 1.40, s * 0.30, s * 0.45))
        m.add(p, box_span('chassis_black', DUMP[0] - 0.22, DUMP[0] + 0.22, 1.10, DUMP[1] + 0.10, s * 0.50, s * 0.60))
    m.add(p, cyl_between('steel_dark', (DUMP[0], DUMP[1], -0.82), (DUMP[0], DUMP[1], 0.82), 0.06, 12))
    # rear fender plates, tail lights and hazard stripes on the frame end
    for s in (1, -1):
        m.add(p, extrude('chassis_black', arch_polygon(AX_R, WY, TR + 0.05, TR + 0.08, 95, 175, 10), TW + 0.06).move(z=s * ZW))
        m.add(p, box_span('rubber_trim', AX_R - 0.95, AX_R - 0.92, 0.30, 0.98, s * (ZW - 0.26), s * (ZW + 0.26)))
    m.add(p, box_span('paint_yellow', -5.10, -4.85, 0.40, 1.20, -1.20, 1.20))
    for s in (1, -1):
        m.add(p, hazard_stripes((-5.10, 0.42, 0.22 if s > 0 else -1.18), (0, 0, 1), (0, 1, 0), 0.96, 0.40, 0.12))
        m.add(p, box('lamp_red', 0.02, 0.10, 0.18, (-5.11, 1.02, s * 0.95)))
        m.add(p, box('lamp_amber', 0.02, 0.08, 0.14, (-5.11, 1.02, s * 0.70)))
        m.add(p, lamp((-5.17, 0.96, s * 0.38), -1, 0.7))


def tray(m):
    p = 'dump_body_tilt'
    x0, x1 = TRAY_X0, TRAY_X1
    top_f, top_r = 2.80, 2.62
    m.add(p, box_span('paint_yellow', -4.80, x1 - 0.12, FLOOR - 0.04, FLOOR, -TRAY_HW + 0.05, TRAY_HW - 0.05))
    m.add(p, box_span('wear_steel', -4.75, x1 - 0.20, FLOOR, FLOOR + 0.015, -TRAY_HW + 0.10, TRAY_HW - 0.10))
    # tail chute rising to the lip
    tail = [(-4.80, FLOOR - 0.04), (-4.80, FLOOR), (x0, FLOOR + 0.42), (x0 - 0.02, FLOOR + 0.36)]
    m.add(p, extrude('paint_yellow', tail, 2 * TRAY_HW - 0.10))
    # front wall with a short spill lip toward the hitch
    m.add(p, box_span('paint_yellow', x1 - 0.12, x1, FLOOR - 0.30, top_f, -TRAY_HW, TRAY_HW))
    m.add(p, box_span('paint_yellow', x1, x1 + 0.30, top_f - 0.06, top_f, -TRAY_HW, TRAY_HW))
    for z in (-0.90, 0.0, 0.90):
        m.add(p, box_span('paint_yellow', x1, x1 + 0.10, FLOOR - 0.25, top_f - 0.06, z - 0.05, z + 0.05))
    # sides: plate, top rail, vertical stiffeners
    side = [(x0, FLOOR + 0.36), (-4.80, FLOOR - 0.04), (x1 - 0.12, FLOOR - 0.30), (x1 - 0.12, top_f), (x0, top_r)]
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', side, 0.05).move(z=s * (TRAY_HW - 0.025)))
        m.add(p, box_between_rail(x0, top_r, x1 - 0.12, top_f, s))
        for x in (-4.20, -3.00, -1.80, -0.60, 0.50):
            yt = top_r + (top_f - top_r) * (x - x0) / (x1 - 0.12 - x0)
            m.add(p, box_span('paint_yellow', x - 0.05, x + 0.05, FLOOR + 0.05, yt - 0.10, s * TRAY_HW, s * (TRAY_HW + 0.06)))
    for s in (1, -1):
        m.add(p, box_span('paint_yellow', x0 - 0.04, x0, FLOOR + 0.40, top_r - 0.05, s * (TRAY_HW - 0.05), s * (TRAY_HW - 0.25)))
        m.add(p, hazard_stripes((x0 - 0.04, FLOOR + 0.45, -TRAY_HW + 0.05 if s < 0 else TRAY_HW - 0.25),
                                (0, 0, 1), (0, 1, 0), 0.20, top_r - FLOOR - 0.55, 0.10))
        m.add(p, box('lamp_red', 0.012, 0.08, 0.08, (x0 - 0.046, top_r - 0.12, s * (TRAY_HW - 0.15))))
    # longitudinal rails under the floor, down to the hinge ears and hoist lugs
    for s in (1, -1):
        rail = [(-4.80, FLOOR - 0.04), (x1 - 0.12, FLOOR - 0.04), (x1 - 0.12, FLOOR - 0.30), (-4.30, FLOOR - 0.30),
                (DUMP[0] + 0.15, DUMP[1] - 0.12), (DUMP[0] - 0.15, DUMP[1] - 0.12), (DUMP[0] - 0.15, DUMP[1] + 0.20)]
        m.add(p, extrude('paint_yellow', rail, 0.10).move(z=s * 0.70))
        m.add(p, box_span('paint_yellow', -1.45, -1.10, FLOOR - 0.48, FLOOR - 0.04, s * 0.84, s * 1.06))
    m.add(p, cyl_between('paint_yellow', (DUMP[0], DUMP[1], -0.78), (DUMP[0], DUMP[1], 0.78), 0.11, 16))
    for x in (-3.60, -1.90, -0.40):
        m.add(p, box_span('paint_yellow', x - 0.06, x + 0.06, FLOOR - 0.22, FLOOR - 0.04, -0.65, 0.65))


def box_between_rail(xa, ya, xb, yb, s):
    from mining_parts import box_between
    z = s * (TRAY_HW + 0.03)
    return box_between('paint_yellow', (xa, ya - 0.06, z), (xb, yb - 0.06, z), 0.10, 0.14)


def build():
    m = Model('Underground Haul Truck', MINING_MATERIALS)
    m.node('front_frame')
    m.node('cab', 'front_frame')
    m.node('hitch_rotate', 'front_frame', (HX, HY, 0.0))           # articulation hitch (rear unit)
    m.node('dump_body_tilt', 'hitch_rotate', (DUMP[0], DUMP[1], 0))
    running_gear(m)
    front_frame(m)
    rear_frame(m)
    tray(m)
    for s in (1, -1):
        side = 'right' if s > 0 else 'left'
        m.cylinder('steering_cylinder_%s' % side, 'front_frame', 'hitch_rotate',
                   (2.30, 1.00, s * 0.55), (0.55, 1.00, s * 0.68), 0.085, 0.05, extension=0.55)
        m.add('front_frame', box_span('chassis_black', 2.15, 2.45, 0.88, 1.12, s * 0.48, s * 0.62))
        m.cylinder('hoist_cylinder_%s' % side, 'hitch_rotate', 'dump_body_tilt',
                   (0.38, 0.95, s * 0.95), (-1.28, FLOOR - 0.40, s * 0.95), 0.13, 0.09, extension=0.6)
    return m


if __name__ == '__main__':
    m = build()
    save(m, 'underground-haul-truck')
    print('tris', m.stats()[1])
