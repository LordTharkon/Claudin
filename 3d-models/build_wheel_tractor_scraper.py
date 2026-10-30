"""Wheel tractor scraper, modern single-engine open-bowl ~16 m3 class.

Key real-world dimensions used (metres):
  wheelbase 8.10, 33.25-29 tyres (2.06 dia x 0.85 wide), tread 2.60
  cutting width ~3.0, overall length ~13.1, width ~3.5, height to top of cab ~3.9
"""
from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, extrude, save, sphere)
from dozer_parts import cab, exhaust, handrail, lamp, louvres, precleaner
from vehicle_parts import _shell, arch_polygon, hose, place_wheel, wheel

AX_T, AX_R = 3.60, -4.50
TR, TW = 1.03, 0.85
WY = 1.00
ZW = 1.30
BW = 1.45   # bowl side-wall plane
ZD = 1.68   # draft arm plane


def wheels(m):
    w = wheel(TR, TW, 0.37, 'chevron', 'otr', lugs=24)
    for ax, tag in ((AX_T, 'tractor'), (AX_R, 'rear')):
        for s, side in ((1, 'right'), (-1, 'left')):
            par = 'tractor' if ax > 0 else 'bowl_tilt'
            m.add(m.node('wheel_%s_%s_spin' % (tag, side), par, (ax, WY, s * ZW)), place_wheel(w, (ax, WY, s * ZW), s))
        axle = 'tractor' if ax > 0 else 'bowl_tilt'
        m.add(axle, cyl_between('chassis_black', (ax, WY, -ZW + 0.35), (ax, WY, ZW - 0.35), 0.19, 24))
        m.add(axle, sphere('chassis_black', 0.40, 24, 12).scale(1, 0.85, 0.9).at((ax, WY, 0)))
        for s in (1, -1):
            m.add(axle, cyl_between('chassis_black', (ax, WY, s * (ZW - 0.55)), (ax, WY, s * (ZW - 0.38)), 0.34, 28))


def tractor(m):
    p = 'tractor'
    m.add(p, box_span('paint_yellow', 2.30, 6.30, 0.85, 1.45, -0.55, 0.55))
    hood = [(-0.75, 1.30), (0.75, 1.30), (0.75, 2.25), (0.62, 2.40), (-0.62, 2.40), (-0.75, 2.25)]
    m.add(p, extrude('paint_yellow', hood, 2.30).rotate('y', 90).move(x=5.15))
    m.add(p, box_span('chassis_black', 4.05, 6.25, 2.395, 2.41, -0.35, 0.35))
    for s in (1, -1):
        m.add(p, louvres(4.40, 5.90, 1.50, 2.15, s * 0.75, s))
        m.add(p, box_span('chassis_black', 4.30, 4.31, 1.35, 2.30, s * 0.749, s * 0.755))
    # radiator grille and front bumper
    m.add(p, box_span('paint_yellow', 6.30, 6.42, 1.30, 2.40, -0.75, 0.75))
    m.add(p, box_span('steel_dark', 6.42, 6.44, 1.40, 2.30, -0.62, 0.62))
    y = 1.45
    while y < 2.28:
        m.add(p, box('chassis_black', 0.04, 0.025, 1.22, (6.45, y, 0)).rotate('z', -20, (6.45, y, 0)))
        y += 0.06
    m.add(p, extrude('paint_yellow', [(6.20, 0.70), (6.65, 0.70), (6.78, 0.85), (6.78, 1.25), (6.65, 1.32), (6.20, 1.32)], 2.10))
    for s in (1, -1):
        m.add(p, lamp((6.75, 1.40, s * 0.85), 1))
        m.add(p, lamp((6.38, 2.50, s * 0.55), 1))
    m.add(p, box_span('steel_dark', 6.76, 6.82, 0.85, 1.05, -0.12, 0.12))
    m.add(p, exhaust(4.50, 2.40, 3.20, -0.40, 0.09))
    m.add(p, precleaner(5.10, 2.40, 0.35))
    # fenders and decks
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', arch_polygon(AX_T, WY, TR + 0.08, TR + 0.13, 15, 172), TW + 0.12).move(z=s * ZW))
        m.add(p, box_span('paint_yellow', 2.45, 4.10, 2.12, 2.17, s * 0.70, s * 1.75))
        m.add(p, box_span('steel_dark', 2.50, 4.05, 2.17, 2.175, s * 0.78, s * 1.70))
        m.add(p, handrail([(2.50, 3.10, s * 1.72), (4.05, 3.10, s * 1.72)], post_to=2.17))
    # ladder on the left, behind the tractor tyre
    for z in (-1.30, -1.62):
        m.add(p, box_between('paint_yellow', (2.48, 0.45, z), (2.55, 2.12, z), 0.04, 0.06, (1, 0, 0)))
    for i in range(6):
        y = 0.55 + i * 0.27
        m.add(p, box_span('steel_dark', 2.44, 2.60, y, y + 0.03, -1.60, -1.32))
    # cab
    m.add('cab', box_span('paint_yellow', 2.40, 4.00, 1.45, 2.12, -0.70, 0.70))
    m.add('cab', cab(2.55, 3.95, 2.17, 3.70, 0.70, door_side=-1))
    # hitch
    m.add(p, cyl_between('steel', (2.25, 1.04, 0), (2.25, 1.76, 0), 0.16, 20))
    m.add(p, box_span('paint_yellow', 2.05, 2.45, 1.05, 1.15, -0.45, 0.45))
    m.add(p, box_span('paint_yellow', 2.05, 2.45, 1.65, 1.75, -0.45, 0.45))
    for s in (1, -1):  # steering cylinders
        m.cylinder('steering_cylinder_' + ('right' if s > 0 else 'left'), 'tractor', 'scraper_hitch_rotate',
                   (2.95, 1.30, s * 0.55), (1.95, 1.40, s * 0.42), 0.07, 0.04, extension=0.55)


def gooseneck(m):
    p = 'scraper_hitch_rotate'
    path = [(2.05, 1.40), (1.80, 2.40), (0.95, 2.85), (-0.05, 2.78)]
    for a, b in zip(path, path[1:]):
        m.add(p, box_between('paint_yellow', (*a, 0), (*b, 0), 0.62, 0.55))
    for q in path[1:-1]:
        m.add(p, cyl_between('paint_yellow', (*q, -0.31), (*q, 0.31), 0.275, 24))
    m.add(p, box_between('paint_yellow', (-0.05, 2.70, -ZD - 0.08), (-0.05, 2.70, ZD + 0.08), 0.45, 0.45, (1, 0, 0)))
    for s in (1, -1):
        m.add(p, box_between('paint_yellow', (-0.05, 2.62, s * ZD), (-2.25, 1.08, s * ZD), 0.16, 0.36))
        m.add(p, cyl_between('steel_dark', (-2.25, 1.08, s * (BW + 0.02)), (-2.25, 1.08, s * (ZD + 0.11)), 0.10, 16))
        # bowl lift cylinders
        m.cylinder('bowl_lift_cylinder_' + ('right' if s > 0 else 'left'), p, 'bowl_tilt',
                   (-0.10, 2.45, s * 1.55), (-0.85, 1.62, s * 1.55), 0.09, 0.055, extension=0.55)
        m.add(p, hose([(1.85, 2.70, s * 0.33), (0.95, 3.14, s * 0.33), (0.0, 3.0, s * 0.45), (-0.10, 2.65, s * 1.40)], 0.02))


def bowl(m):
    p = 'bowl_tilt'
    side = [(-0.55, 2.35), (-0.55, 0.55), (-0.80, 0.12), (-2.90, 0.50), (-3.35, 0.72), (-3.35, 2.35)]
    for s in (1, -1):
        z = s * BW
        m.add(p, extrude('paint_yellow', side, 0.05).move(z=z))
        m.add(p, box_span('paint_yellow', -3.40, -0.50, 2.30, 2.45, z - 0.06, z + 0.06))  # top rail
        for x in (-1.45, -2.25, -2.95):
            m.add(p, box_span('paint_yellow', x - 0.06, x + 0.06, 0.55, 2.30, z, z + s * 0.10))
        m.add(p, box_span('paint_yellow', -2.55, -1.95, 0.80, 1.35, z, z + s * 0.12))  # draft arm boss
        m.add(p, box_span('paint_yellow', -0.95, -0.75, 1.50, 1.75, z, z + s * 0.14))   # lift cyl bracket
    m.add(p, box_between('paint_yellow', (-0.80, 0.12, 0), (-2.90, 0.50, 0), 2 * BW, 0.05))
    m.add(p, box_between('paint_yellow', (-2.90, 0.50, 0), (-3.35, 0.72, 0), 2 * BW, 0.05))
    # cutting edge with centre stinger
    m.add(p, box_between('wear_steel', (-0.62, 0.05, 0), (-0.86, 0.13, 0), 2 * BW - 0.01, 0.06))
    m.add(p, box_between('wear_steel', (-0.50, 0.03, 0), (-0.66, 0.08, 0), 1.0, 0.05))
    for s in (1, -1):
        m.add(p, extrude('wear_steel', [(-0.545, 0.20), (-0.545, 0.62), (-0.80, 0.10), (-0.66, 0.06)], 0.08).move(z=s * (BW + 0.04)))
    # apron and its arms/cylinders
    apron = [(-0.62, 0.30), (-0.36, 0.62), (-0.20, 1.10), (-0.20, 1.72), (-0.36, 2.28)]
    m.add('apron_tilt', extrude('paint_yellow', _shell(apron, -0.04), 2 * BW - 0.06))
    for s in (1, -1):
        m.add('apron_tilt', extrude('paint_yellow', [(-0.62, 0.30), (-0.36, 0.62), (-0.20, 1.10), (-0.20, 1.72), (-0.36, 2.28),
                                         (-0.55, 2.28), (-0.55, 0.55)], 0.04).move(z=s * (BW - 0.05)))
        m.add('apron_tilt', box_between('paint_yellow', (-1.25, 2.28, s * (BW + 0.10)), (-0.30, 1.55, s * (BW + 0.10)), 0.08, 0.18))
        m.cylinder('apron_cylinder_' + ('right' if s > 0 else 'left'), p, 'apron_tilt',
                   (-1.75, 2.48, s * (BW + 0.10)), (-0.80, 2.15, s * (BW + 0.10)), 0.06, 0.035, extension=0.5)
        m.add(p, cyl_between('steel_dark', (-1.25, 2.28, s * BW), (-1.25, 2.28, s * (BW + 0.16)), 0.06, 12))
    # ejector
    m.add('ejector_slide', box_span('paint_yellow', -3.05, -2.97, 0.55, 2.28, -BW + 0.04, BW - 0.04))
    for z in (-0.9, 0.0, 0.9):
        m.add('ejector_slide', box_span('paint_yellow', -3.30, -3.05, 0.65, 2.20, z - 0.06, z + 0.06))
    m.cylinder('ejector_cylinder', p, 'ejector_slide', (-4.30, 1.60, 0), (-3.10, 1.60, 0), 0.10, 0.06, extension=0.45)


def rear(m):
    p = 'bowl_tilt'
    m.add(p, box_span('paint_yellow', -6.02, -3.30, 1.10, 2.05, -0.72, 0.72))
    m.add(p, extrude('paint_yellow', [(-3.35, 2.35), (-3.35, 0.72), (-4.30, 1.10), (-4.30, 2.05)], 1.44))  # bowl-to-frame gusset
    m.add(p, box_span('paint_yellow', -3.40, -3.30, 2.05, 2.45, -BW, BW))
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', arch_polygon(AX_R, WY, TR + 0.08, TR + 0.13, 8, 172), TW + 0.12).move(z=s * ZW))
        m.add(p, box_span('paint_yellow', -5.60, -3.40, 2.12, 2.17, s * 0.72, s * 1.75))
        m.add(p, box('lamp_red', 0.02, 0.12, 0.22, (-6.03, 1.85, s * 0.50)))
        m.add(p, lamp((-5.90, 2.25, s * 0.55), -1))
    # push block
    m.add(p, box_span('steel_dark', -6.35, -6.02, 0.95, 1.75, -0.85, 0.85))
    m.add(p, box_span('chassis_black', -6.40, -6.35, 1.00, 1.70, -0.80, 0.80))


def build():
    m = Model('Wheel Tractor Scraper', MATERIALS)
    m.node('tractor')
    m.node('cab', 'tractor')
    m.node('scraper_hitch_rotate', 'tractor', (2.25, 1.40, 0))   # articulation, vertical pin
    m.node('bowl_tilt', 'scraper_hitch_rotate', (-2.25, 1.08, 0))  # draft-arm pivots on the bowl
    m.node('apron_tilt', 'bowl_tilt', (-1.25, 2.28, 0))
    m.node('ejector_slide', 'bowl_tilt', (-3.01, 1.40, 0))
    wheels(m)
    tractor(m)
    gooseneck(m)
    bowl(m)
    rear(m)
    return m


if __name__ == '__main__':
    save(build(), 'wheel-tractor-scraper', 'wheel_tractor_scraper')
