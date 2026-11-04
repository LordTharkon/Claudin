"""Mine-spec light vehicle: crew-cab 4x4 ute on a flat tray, set up for site use.

Key real-world dimensions used (metres):
  wheelbase 3.18, track 1.55, 265/75R16 all-terrain tyres (0.80 dia) on black steel rims
  length 5.3 (bull bar to tray), width 1.9 over tray and flares, height ~2.0 to the beacon
  internal ROPS, alloy bull bar, side steps, roof LED light bar and amber beacon, UHF aerial,
  snorkel, flat alloy tray with drop sides, headboard, tray toolbox and underbody toolboxes,
  ~4 m buggy-whip flag on its own node (`buggy_whip_flag`), blank mine-number panels
"""
import math

from mining_parts import (MINING_MATERIALS, Model, beacon, box, box_between, box_span, cyl_between,
                          cylinder, extrude, hazard_stripes, hose, polyline_tube, save, sphere)
from bodykit import panel
from vehicle_parts import arch_polygon, place_wheel, rim, steering_wheel, tyre

AX_F, AX_R = 1.45, -1.73
TR, TW, BEAD = 0.40, 0.265, 0.2032
WY = 0.39
ZW = 0.775
HW = 0.88            # body half-width
SILL = 0.52
BELT = 1.21
ROOF = 1.94
CAB_X = -0.98        # back of cab
TRAY = (-2.70, -1.04)
DECK = 1.00          # tray deck height
WHITE = 'paint_white'


def wheels(m):
    w = tyre(TR, TW, BEAD, 'at', segs=40) + rim(BEAD, TW, 'steel', 'chassis_black', nuts=6)
    for s, side in ((1, 'right'), (-1, 'left')):
        st = m.node('wheel_front_%s_steer' % side, 'body', (AX_F, WY, s * (ZW - 0.13)))
        n = m.node('wheel_front_%s_spin' % side, st, (AX_F, WY, s * ZW))
        m.add(n, place_wheel(w, (AX_F, WY, s * ZW), s))
        n = m.node('wheel_rear_%s_spin' % side, 'body', (AX_R, WY, s * ZW))
        m.add(n, place_wheel(w, (AX_R, WY, s * ZW), s))
    b = 'body'
    for ax in (AX_F, AX_R):
        m.add(b, cyl_between('chassis_black', (ax, WY, -ZW + 0.14), (ax, WY, ZW - 0.14), 0.045, 12))
        m.add(b, sphere('chassis_black', 0.14, 14, 7).scale(1, 0.9, 1).at((ax, WY, 0.1)))
    m.add(b, cyl_between('chassis_black', (AX_F - 0.15, WY, 0.1), (AX_R + 0.15, WY + 0.02, 0.1), 0.035, 10))  # tail shaft
    for s in (1, -1):  # leaf springs at the rear, rails
        m.add(b, box_between('steel_dark', (AX_R + 0.65, 0.50, s * 0.48), (AX_R - 0.60, 0.50, s * 0.48), 0.07, 0.06))
        m.add(b, box_span('chassis_black', -2.62, 2.25, 0.44, 0.60, s * 0.38, s * 0.48))


def lower_body(m):
    b = 'body'
    a0 = math.degrees(math.asin((SILL - WY) / 0.50))
    arch = [(AX_F + 0.50 * math.cos(math.radians(a)), WY + 0.50 * math.sin(math.radians(a)))
            for a in [180 - a0 - (180 - 2 * a0) * i / 14 for i in range(15)]]
    prof = [(AX_F + 0.483, SILL), (2.36, SILL), (2.40, 0.64), (2.40, 1.06), (2.30, 1.17), (0.88, BELT),
            (CAB_X, BELT), (CAB_X, SILL)] + arch
    m.add(b, extrude(WHITE, prof, 2 * HW))
    # greenhouse
    gh = [(0.86, BELT), (0.45, 1.90), (0.38, ROOF), (-0.90, ROOF), (-0.96, 1.90), (-0.96, BELT)]
    m.add(b, extrude(WHITE, gh, 1.64))
    m.add(b, box_span('rubber_trim', 0.84, 0.89, BELT - 0.01, BELT + 0.012, -0.83, 0.83))   # scuttle seal
    # windscreen, side and rear glass
    d = (0.45 - 0.86, 1.90 - BELT)
    L = math.hypot(*d)
    nx, ny = d[1] / L, -d[0] / L
    p0 = (0.86 - 0.07 * d[0] / L + 0.006 * nx, BELT + 0.07 * d[1] / L + 0.006 * ny)
    p1 = (0.45 + 0.05 * d[0] / L * -1 + 0.006 * nx, 1.90 - 0.05 * d[1] / L + 0.006 * ny)
    m.add(b, box_between('glass', (p0[0], p0[1], 0), (p1[0], p1[1], 0), 1.48, 0.008))
    m.add(b, box('rubber_trim', 0.015, 0.55, 0.02, (0.70, 1.48, 0.25)).rotate('z', 30, (0.70, 1.48, 0)))
    for s in (1, -1):
        z = s * 0.823
        fn = lambda u, v, z=z: (u, v, z)
        m.add(b, panel('glass', [(0.06, 1.28), (0.75, 1.28), (0.40, 1.86), (0.06, 1.86)], fn, (0, 0, s)))
        m.add(b, panel('glass_tinted', [(-0.84, 1.28), (-0.03, 1.28), (-0.03, 1.86), (-0.84, 1.86)], fn, (0, 0, s)))
    m.add(b, box_span('glass_tinted', -0.966, -0.962, 1.30, 1.84, -0.62, 0.62))
    # door shut lines, handles, blank mine-number panels
    for s in (1, -1):
        z = s * (HW + 0.002)
        for x in (0.86, 0.02, -0.92):
            m.add(b, box_span('rubber_trim', x - 0.006, x + 0.006, SILL + 0.06, BELT - 0.01, z - 0.003, z + 0.003))
        for x in (0.62, -0.22):
            m.add(b, box_span('plastic_black', x - 0.08, x + 0.08, 1.06, 1.10, z, z + s * 0.02))
        m.add(b, box_span('plastic_black', 0.18, 0.76, 0.70, 1.00, z - 0.002, z + s * 0.004))
        m.add(b, box_span(WHITE, 0.20, 0.74, 0.72, 0.98, z + s * 0.004, z + s * 0.008))
    # flares, grille, lights, snorkel, mirrors
    for s in (1, -1):
        m.add(b, extrude('plastic_black', arch_polygon(AX_F, WY, 0.50, 0.57, 12, 168, 12), 0.10).move(z=s * (HW + 0.03)))
    m.add(b, box_span('plastic_black', 2.40, 2.42, 0.74, 1.04, -0.52, 0.52))
    for y in (0.80, 0.88, 0.96):
        m.add(b, box_span('chrome', 2.42, 2.43, y - 0.012, y + 0.012, -0.50, 0.50))
    for s in (1, -1):
        m.add(b, cylinder('chassis_black', 0.10, 0.03, 20).rotate('z', 90).at((2.415, 0.90, s * 0.68)))
        m.add(b, cylinder('lamp_lens', 0.085, 0.02, 20).rotate('z', 90).at((2.43, 0.90, s * 0.68)))
        m.add(b, box('lamp_amber', 0.02, 0.05, 0.10, (2.41, 0.765, s * 0.68)))
    m.add(b, polyline_tube('plastic_black', [(1.05, 1.00, HW + 0.05), (1.05, 1.18, HW + 0.05), (0.80, 1.60, HW + 0.05),
                                             (0.55, 1.98, HW + 0.05)], 0.045, 10))
    m.add(b, box_span('plastic_black', 0.48, 0.70, 1.96, 2.06, HW - 0.02, HW + 0.12))
    for s in (1, -1):
        a = (0.80, 1.25, s * HW)
        c = (0.88, 1.32, s * (HW + 0.16))
        m.add(b, box_between('plastic_black', a, c, 0.03, 0.04))
        m.add(b, box('plastic_black', 0.05, 0.22, 0.17, (0.88, 1.40, s * (HW + 0.19))))
        m.add(b, box('glass', 0.01, 0.19, 0.14, (0.851, 1.40, s * (HW + 0.19))))
    # roof: LED light bar, amber beacon, UHF aerial
    for s in (1, -1):
        m.add(b, box_span('chassis_black', 0.18, 0.26, ROOF, ROOF + 0.05, s * 0.55, s * 0.60))
    m.add(b, box_span('plastic_black', 0.15, 0.30, ROOF + 0.05, ROOF + 0.13, -0.68, 0.68))
    m.add(b, box_span('lamp_lens', 0.30, 0.305, ROOF + 0.06, ROOF + 0.12, -0.64, 0.64))
    m.add(b, box_span('chassis_black', -0.72, -0.48, ROOF, ROOF + 0.02, -0.12, 0.12))
    m.add(b, beacon((-0.60, ROOF + 0.02, 0.0), 0.065))
    m.add(b, cylinder('chassis_black', 0.025, 0.05, 8).at((2.50, 1.27, -0.75)))
    m.add(b, cyl_between('plastic_black', (2.50, 1.29, -0.75), (2.50, 2.40, -0.75), 0.006, 6))


def cabin(m):
    """Interior: seats, dash, steering wheel and the internal ROPS cage."""
    b = 'body'
    m.add(b, box_span('plastic_black', 0.52, 0.84, 0.95, BELT - 0.01, -0.80, 0.80))
    m.add(b, steering_wheel((0.38, 1.12, 0.40), 0.19, 25))
    for z in (0.40, -0.40):
        m.add(b, box_span('seat_fabric', -0.05, 0.40, 0.62, 0.78, z - 0.25, z + 0.25))
        m.add(b, box('seat_fabric', 0.12, 0.62, 0.48, (-0.08, 1.08, z)).rotate('z', 12, (-0.08, 0.78, z)))
        m.add(b, box('seat_fabric', 0.09, 0.15, 0.26, (-0.16, 1.48, z)))
    m.add(b, box_span('plastic_black', -0.02, 0.38, 0.60, 0.92, -0.10, 0.10))       # console
    m.add(b, box_span('seat_fabric', -0.90, -0.45, 0.62, 0.78, -0.72, 0.72))
    m.add(b, box('seat_fabric', 0.12, 0.60, 1.44, (-0.86, 1.08, 0)).rotate('z', 10, (-0.86, 0.78, 0)))
    m.add(b, box_span('chassis_black', 0.84, 0.86, 0.55, 0.96, -0.80, 0.80))        # firewall
    m.add(b, box_span('chassis_black', -0.94, 0.84, 0.55, 0.60, -0.80, 0.80))       # floor
    # internal ROPS (6-point cage)
    r = 0.028
    for s in (1, -1):
        z = s * 0.72
        m.add(b, polyline_tube('paint_gray_dark', [(0.66, 0.62, z), (0.70, BELT, z), (0.40, 1.86, z), (-0.84, 1.86, z),
                                                   (-0.84, 0.62, z)], r, 8))
        m.add(b, cyl_between('paint_gray_dark', (0.01, 0.62, z), (0.01, 1.86, z), r, 8))
    for x in (0.40, 0.01, -0.84):
        m.add(b, cyl_between('paint_gray_dark', (x, 1.86, -0.72), (x, 1.86, 0.72), r, 8))
    m.add(b, cyl_between('paint_gray_dark', (-0.84, 1.86, 0.72), (-0.84, 0.70, -0.72), r, 8))


def bull_bar_and_steps(m):
    b = 'body'
    m.add(b, box_span('aluminum', 2.40, 2.62, 0.40, 0.70, -0.92, 0.92))
    m.add(b, box_span('aluminum', 2.62, 2.64, 0.46, 0.64, -0.36, 0.36))
    m.add(b, box_span('plastic_black', 2.64, 2.645, 0.47, 0.63, -0.35, 0.35))
    m.add(b, box_span(WHITE, 2.645, 2.65, 0.49, 0.61, -0.33, 0.33))                   # blank number panel
    r = 0.030
    for s in (1, -1):
        m.add(b, polyline_tube('aluminum', [(2.56, 0.70, s * 0.30), (2.58, 1.12, s * 0.30), (2.52, 1.24, s * 0.20),
                                            (2.52, 1.24, s * 0.02)], r, 8))
        m.add(b, polyline_tube('aluminum', [(2.56, 0.70, s * 0.90), (2.54, 1.02, s * 0.88), (2.50, 1.10, s * 0.78),
                                            (2.52, 1.12, s * 0.42), (2.58, 1.10, s * 0.31)], r, 8))
        m.add(b, box('lamp_amber', 0.02, 0.05, 0.12, (2.625, 0.60, s * 0.70)))
        m.add(b, cylinder('chassis_black', 0.07, 0.10, 14).rotate('z', 90).at((2.60, 0.80, s * 0.62)))
        m.add(b, cylinder('lamp_lens', 0.06, 0.01, 14).rotate('z', 90).at((2.655, 0.80, s * 0.62)))
    m.add(b, cyl_between('aluminum', (2.52, 1.24, -0.02), (2.52, 1.24, 0.02), r, 8))
    for s in (1, -1):  # side steps
        m.add(b, box_span('aluminum', CAB_X + 0.05, 0.90, 0.40, 0.46, s * 0.70, s * 0.95))
        m.add(b, box_span('steel_dark', CAB_X + 0.10, 0.85, 0.46, 0.465, s * 0.74, s * 0.92))
        for x in (-0.6, 0.5):
            m.add(b, box_span('chassis_black', x - 0.04, x + 0.04, 0.44, 0.55, s * 0.48, s * 0.70))


def tray(m):
    b = 'body'
    x0, x1 = TRAY
    m.add(b, box_span('aluminum', x0, x1, DECK - 0.04, DECK, -0.95, 0.95))
    for x in (x0 + 0.1, -2.10, -1.55, x1 - 0.1):
        m.add(b, box_span('chassis_black', x - 0.04, x + 0.04, 0.60, DECK - 0.04, -0.70, 0.70))
    for s in (1, -1):
        m.add(b, box_span('chassis_black', x0, x1, 0.60, 0.68, s * 0.30, s * 0.45))
    # drop sides and tailgate with hinges, latches
    for s in (1, -1):
        m.add(b, box_span('aluminum', x0, x1, DECK, DECK + 0.27, s * 0.93, s * 0.95))
        for x in (-2.35, -1.55):
            m.add(b, box_span('steel', x - 0.03, x + 0.03, DECK + 0.20, DECK + 0.25, s * 0.95, s * 0.965))
    m.add(b, box_span('aluminum', x0, x0 + 0.02, DECK, DECK + 0.27, -0.93, 0.93))
    for z in (-0.6, 0.6):
        m.add(b, box_span('steel', x0 - 0.015, x0, DECK + 0.20, DECK + 0.25, z - 0.03, z + 0.03))
    m.add(b, box_span('plastic_black', x0 - 0.004, x0, DECK + 0.03, DECK + 0.24, -0.30, 0.30))
    m.add(b, box_span(WHITE, x0 - 0.008, x0 - 0.004, DECK + 0.05, DECK + 0.22, -0.28, 0.28))   # blank number panel
    # headboard with protector bars
    hb = [(x1, DECK, 0.94), (x1, 1.92, 0.94), (x1, 1.92, -0.94), (x1, DECK, -0.94)]
    m.add(b, polyline_tube('aluminum', hb, 0.03, 8))
    for z in (-0.6, -0.3, 0.0, 0.3, 0.6):
        m.add(b, cyl_between('aluminum', (x1, DECK + 0.6, z), (x1, 1.92, z), 0.014, 6))
    m.add(b, box_span('aluminum', x1 - 0.02, x1 + 0.02, DECK, DECK + 0.6, -0.94, 0.94))
    # tray toolbox behind the headboard
    tb = (x1 - 0.52, x1 - 0.04)
    m.add(b, box_span('aluminum', tb[0], tb[1], DECK, DECK + 0.55, -0.90, 0.90))
    m.add(b, box_span('rubber_trim', tb[0] - 0.004, tb[0], DECK + 0.40, DECK + 0.41, -0.88, 0.88))
    for z in (-0.5, 0.5):
        m.add(b, box_span('steel', tb[0] - 0.02, tb[0], DECK + 0.30, DECK + 0.38, z - 0.05, z + 0.05))
    # underbody toolboxes behind the rear wheels, mudguards, rear bar, lights, stripes
    for s in (1, -1):
        m.add(b, box_span('aluminum', x0 + 0.05, x0 + 0.48, 0.55, DECK - 0.05, s * 0.50, s * 0.92))
        m.add(b, box_span('steel', x0 + 0.20, x0 + 0.33, 0.85, 0.90, s * 0.92, s * 0.935))
        m.add(b, extrude('plastic_black', arch_polygon(AX_R, WY, 0.48, 0.52, 0, 180, 12), 0.30).move(z=s * ZW))
        m.add(b, box_span('rubber_trim', AX_R - 0.47, AX_R - 0.44, 0.25, 0.60, s * 0.64, s * 0.92))
        m.add(b, box('lamp_red', 0.02, 0.12, 0.16, (x0 - 0.01, DECK - 0.12, s * 0.80)))
        m.add(b, box('lamp_amber', 0.02, 0.06, 0.16, (x0 - 0.01, DECK - 0.24, s * 0.80)))
    m.add(b, box_span('steel_dark', x0 - 0.06, x0 + 0.05, 0.46, 0.62, -0.82, 0.82))
    m.add(b, hazard_stripes((x0 - 0.06, 0.465, -0.80), (0, 0, 1), (0, 1, 0), 1.60, 0.15, 0.10))
    m.add(b, box_span('steel', x0 - 0.18, x0 - 0.06, 0.48, 0.56, -0.05, 0.05))      # tow hitch
    m.add(b, sphere('chrome', 0.03, 10, 5).at((x0 - 0.15, 0.59, 0)))


def buggy_whip(m):
    """~4 m fibreglass whip with pennant and tip lamp, sprung base on the tray corner."""
    base = (TRAY[0] + 0.06, DECK + 0.27, 0.90)
    f = m.node('buggy_whip_flag', 'body', base)
    x, y, z = base
    m.add(f, box_span('steel_dark', x - 0.05, x + 0.05, y, y + 0.04, z - 0.05, z + 0.05))
    m.add(f, cylinder('steel', 0.022, 0.16, 10).at((x, y + 0.12, z)))
    m.add(f, cyl_between('paint_orange', (x, y + 0.20, z), (x, y + 4.00, z), 0.010, 8, r_top=0.005))
    m.add(f, cylinder('lamp_red', 0.012, 0.05, 8).at((x, y + 4.025, z)))
    pennant = [(0.0, 0.0), (0.0, 0.32), (0.48, 0.16)]
    m.add(f, extrude('paint_red', pennant, 0.004).move(x=x + 0.01, y=y + 3.58, z=z))


def build():
    m = Model('Mine Light Vehicle', MINING_MATERIALS)
    m.node('body')
    wheels(m)
    lower_body(m)
    cabin(m)
    bull_bar_and_steps(m)
    tray(m)
    buggy_whip(m)
    return m


if __name__ == '__main__':
    save(build(), 'mine-light-vehicle')
