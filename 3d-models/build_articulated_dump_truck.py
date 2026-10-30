"""Articulated dump truck (off-road hauler), modern 6x6 ~40 t class.

Key real-world dimensions used (metres):
  front axle to bogie centre 5.50, bogie axle spacing 1.70, articulation 1.75 behind front axle
  875/65R29 tyres (1.90 dia x 0.80 wide), tread 2.55
  body width ~3.5, height to top of cab ~3.7, overall length ~10.8
"""
from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, extrude, save, sphere)
from dozer_parts import cab, handrail, lamp, louvres
from vehicle_parts import arch_polygon, place_wheel, wheel

AX_F, AX_B1, AX_B2 = 3.70, -1.95, -3.65
TR, TW = 0.95, 0.80
WY = 0.94
ZW = 1.275
HITCH_X = 1.95
BODY_F, BODY_R, TAIL_END = 1.55, -3.80, -5.20


def running_gear(m):
    w = wheel(TR, TW, 0.37, 'chevron', 'otr', lugs=22)
    for ax, tag in ((AX_F, 'front'), (AX_B1, 'bogie_front'), (AX_B2, 'bogie_rear')):
        for s, side in ((1, 'right'), (-1, 'left')):
            par = 'tractor' if tag == 'front' else 'bogie_tilt'
            m.add(m.node('wheel_%s_%s_spin' % (tag, side), par, (ax, WY, s * ZW)), place_wheel(w, (ax, WY, s * ZW), s))
        axle = 'tractor' if tag == 'front' else 'bogie_tilt'
        m.add(axle, cyl_between('chassis_black', (ax, WY, -ZW + 0.3), (ax, WY, ZW - 0.3), 0.16, 24))
        m.add(axle, sphere('chassis_black', 0.33, 22, 11).scale(1, 0.85, 0.9).at((ax, WY, 0)))
        for s in (1, -1):
            m.add(axle, cyl_between('chassis_black', (ax, WY, s * (ZW - 0.50)), (ax, WY, s * (ZW - 0.34)), 0.30, 26))
    # bogie beams with centre pivots
    mid = (AX_B1 + AX_B2) / 2
    for s in (1, -1):
        m.add('bogie_tilt', box_between('chassis_black', (AX_B1 + 0.25, WY + 0.05, s * 0.68), (AX_B2 - 0.25, WY + 0.05, s * 0.68), 0.16, 0.28))
        m.add('trailer_rotate', cyl_between('steel_dark', (mid, WY + 0.10, s * 0.45), (mid, WY + 0.10, s * 0.80), 0.13, 18))


def tractor(m):
    p = 'tractor'
    m.add(p, box_span('chassis_black', HITCH_X, 5.30, 0.78, 1.38, -0.50, 0.50))
    # sloping hood and crown
    m.add(p, extrude('paint_yellow', [(3.35, 1.30), (5.25, 1.30), (5.42, 1.45), (5.40, 1.86), (5.22, 2.06), (3.35, 2.30)], 1.50))
    m.add(p, extrude('paint_yellow', [(3.35, 2.28), (5.22, 2.04), (5.22, 2.08), (3.35, 2.33)], 1.20))
    for s in (1, -1):
        m.add(p, louvres(3.70, 4.70, 1.45, 1.90, s * 0.75, s))
        m.add(p, box_span('chassis_black', 4.95, 4.96, 1.32, 2.05, s * 0.749, s * 0.755))
    m.add(p, box_span('steel_dark', 5.40, 5.43, 1.48, 1.84, -0.62, 0.62))
    y = 1.52
    while y < 1.80:
        m.add(p, box_span('chassis_black', 5.42, 5.44, y, y + 0.02, -0.61, 0.61))
        y += 0.05
    # bumper with steps, tow pin and lights
    m.add(p, extrude('paint_yellow', [(5.20, 0.72), (5.50, 0.72), (5.58, 0.82), (5.58, 1.22), (5.20, 1.30)], 2.10))
    for s in (1, -1):
        m.add(p, lamp((5.58, 1.12, s * 0.85), 1))
        m.add(p, box('lamp_amber', 0.02, 0.06, 0.12, (5.585, 0.95, s * 0.88)))
        m.add(p, box_span('steel_dark', 5.40, 5.62, 0.62, 0.64, s * 0.30, s * 0.60))
    m.add(p, box_span('steel_dark', 5.57, 5.64, 0.82, 1.02, -0.12, 0.12))
    # front fenders with lights
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', arch_polygon(AX_F, WY, TR + 0.07, TR + 0.11, 15, 172), TW + 0.10).move(z=s * ZW))
        m.add(p, lamp((AX_F + 0.75, WY + 0.98, s * (ZW - 0.25)), 1, 0.9))
    # cab, side deck, ladder
    m.add('cab', box_span('paint_yellow', HITCH_X + 0.05, 3.35, 1.38, 1.96, -0.82, 0.82))
    m.add('cab', cab(HITCH_X + 0.05, 3.35, 1.96, 3.55, 0.85, door_side=-1))
    m.add('cab', box_span('paint_yellow', HITCH_X + 0.05, 3.10, 1.93, 1.98, -1.25, -0.85))
    m.add('cab', box_span('steel_dark', HITCH_X + 0.10, 3.05, 1.98, 1.985, -1.20, -0.88))
    m.add('cab', handrail([(HITCH_X + 0.10, 2.95, -1.23), (3.05, 2.95, -1.23)], post_to=1.98))
    for z in (-0.95, -1.22):
        m.add('cab', box_between('paint_yellow', (2.25, 0.40, z), (2.40, 1.93, z), 0.04, 0.06, (1, 0, 0)))
    for i in range(5):
        y = 0.52 + i * 0.29
        x = 2.25 + 0.15 * (y - 0.40) / 1.53
        m.add('cab', box_span('steel_dark', x - 0.07, x + 0.07, y, y + 0.03, -1.21, -0.96))
    m.add(p, cyl_between('steel_dark', (HITCH_X - 0.07, 1.38, 0.70), (HITCH_X - 0.07, 3.10, 0.70), 0.06, 14))  # exhaust stack
    # articulation / oscillation joint and steering cylinders
    m.add('tractor', cyl_between('steel_dark', (HITCH_X - 0.30, 1.10, 0), (HITCH_X + 0.10, 1.10, 0), 0.34, 28))
    for y in (0.85, 1.35):
        m.add('trailer_rotate', cyl_between('steel', (HITCH_X - 0.45, y - 0.06, 0), (HITCH_X - 0.45, y + 0.06, 0), 0.12, 18))
        m.add('trailer_rotate', box_span('chassis_black', HITCH_X - 0.65, HITCH_X - 0.25, y - 0.05, y + 0.05, -0.40, 0.40))
    for s in (1, -1):
        m.cylinder('steering_cylinder_' + ('right' if s > 0 else 'left'), 'tractor', 'trailer_rotate',
                   (HITCH_X + 0.70, 1.05, s * 0.50), (HITCH_X - 0.80, 1.05, s * 0.45), 0.07, 0.04, extension=0.55)


def trailer(m):
    p = 'trailer_rotate'
    for s in (1, -1):
        m.add(p, box_span('chassis_black', -4.35, HITCH_X - 0.45, 0.95, 1.45, s * 0.32, s * 0.52))
        m.add(p, box('lamp_red', 0.02, 0.10, 0.18, (-4.36, 1.30, s * 0.42)))
        m.add(p, box_span('rubber_trim', -4.70, -4.67, 0.35, 1.70, s * 0.85, s * 1.69))  # mud flaps
        m.add(p, box_span('chassis_black', -4.72, -4.30, 1.66, 1.72, s * 0.50, s * 1.70))
    for x in (1.0, -0.6, -2.8, -4.25):
        m.add(p, box_span('chassis_black', x - 0.08, x + 0.08, 1.05, 1.40, -0.40, 0.40))
    for s in (1, -1):  # hoist cylinders
        m.cylinder('hoist_cylinder_' + ('right' if s > 0 else 'left'), p, 'dump_body_tilt',
                   (0.85, 1.20, s * 0.62), (-0.35, 1.98, s * 0.62), 0.11, 0.07, extension=0.55)
        m.add(p, box_span('chassis_black', 0.72, 0.98, 1.05, 1.35, s * 0.52, s * 0.72))
    m.add(p, box_span('chassis_black', -4.20, -3.70, 1.40, 1.75, -0.60, 0.60))  # body pivot brackets
    m.add(p, cyl_between('steel', (-3.95, 1.62, -0.75), (-3.95, 1.62, 0.75), 0.07, 14))


def body(m):
    p = 'dump_body_tilt'
    outer = [(-1.74, 3.45), (-1.74, 2.45), (-1.02, 2.01), (1.02, 2.01), (1.74, 2.45), (1.74, 3.45)]
    inner = [(1.70, 3.45), (1.70, 2.47), (1.00, 2.06), (-1.00, 2.06), (-1.70, 2.47), (-1.70, 3.45)]
    L = BODY_F - BODY_R
    m.add(p, extrude('paint_yellow', outer + inner, L).rotate('y', 90).move(x=(BODY_F + BODY_R) / 2))
    m.add(p, extrude('paint_yellow', outer, 0.06).rotate('y', 90).move(x=BODY_F - 0.03))  # front wall
    m.add(p, box_span('paint_yellow', BODY_F - 0.06, BODY_F, 3.45, 3.68, -1.74, 1.74))     # raised front lip
    m.add(p, box_span('paint_yellow', BODY_F, HITCH_X + 0.02, 3.58, 3.64, -1.60, 1.60))   # spill canopy
    # subframe rails under the floor
    for s in (1, -1):
        m.add(p, box_span('paint_yellow', BODY_R - 0.30, BODY_F - 0.10, 1.75, 2.01, s * 0.45, s * 0.62))
    # tail chute: lower section sheared upward toward the rear
    Lt = BODY_R - TAIL_END
    rise = 0.45
    k = rise / Lt
    low = [(-1.74, 2.58), (-1.74, 2.45), (-1.02, 2.01), (1.02, 2.01), (1.74, 2.45), (1.74, 2.58),
           (1.70, 2.58), (1.70, 2.47), (1.00, 2.06), (-1.00, 2.06), (-1.70, 2.47), (-1.70, 2.58)]
    tail = extrude('paint_yellow', low, Lt).rotate('y', 90)
    tail = tail.transform(((1, 0, 0), (-k, 1, 0), (0, 0, 1)), ((BODY_R + TAIL_END) / 2, rise / 2, 0))
    m.add(p, tail)
    for s in (1, -1):
        z = s * 1.72
        m.add(p, extrude('paint_yellow', [(BODY_R + 0.02, 2.50), (TAIL_END, 2.50 + rise), (TAIL_END, 3.08), (BODY_R + 0.02, 3.45)], 0.04).move(z=z))
        m.add(p, box_between('paint_yellow', (BODY_R, 3.47, z), (TAIL_END, 3.10, z), 0.10, 0.08))
        m.add(p, box_span('paint_yellow', BODY_R, BODY_F, 3.42, 3.52, z - 0.05, z + 0.05))  # top rail
        # outside ribs
        x = BODY_F - 0.55
        while x > BODY_R:
            m.add(p, box_span('paint_yellow', x - 0.05, x + 0.05, 2.45, 3.42, z, z + s * 0.08))
            m.add(p, box_between('paint_yellow', (x, 2.02, s * 1.02), (x, 2.46, s * 1.75), 0.10, 0.08, (s, 0, 0)))
            x -= 0.90


def build():
    m = Model('Articulated Dump Truck', MATERIALS)
    m.node('tractor')
    m.node('cab', 'tractor')
    m.node('trailer_rotate', 'tractor', (HITCH_X - 0.45, 1.10, 0))          # articulation pins
    m.node('bogie_tilt', 'trailer_rotate', ((AX_B1 + AX_B2) / 2, WY + 0.10, 0))  # tandem walking beams
    m.node('dump_body_tilt', 'trailer_rotate', (-3.95, 1.62, 0))             # body hinge
    running_gear(m)
    tractor(m)
    trailer(m)
    body(m)
    return m


if __name__ == '__main__':
    save(build(), 'Construction/articulated-dump-truck', 'articulated_dump_truck')
