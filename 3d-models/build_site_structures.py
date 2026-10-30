"""Site structures: hoarding wall with gate, survey tripod with total station and stake,
modular stair tower."""
import math

from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe,
                     polyline_tube, save, sphere)
from bodykit import rbox


def hoarding():
    """Modular 2.4 m plywood hoarding section on timber posts, plus a gate section."""
    m = Model('Site Hoarding', MATERIALS)
    H = 2.40
    zf = 0.0       # face plane: plywood on the +Z side of the posts
    m.node('hoarding_section')
    m.node('gate_section')
    p = 'hoarding_section'

    def post(x, node):
        m.add(node, box_span('timber', x - 0.05, x + 0.05, 0.0, H + 0.05, zf - 0.10, zf))
        m.add(node, box_span('timber', x - 0.04, x + 0.04, 0.0, 0.06, zf - 1.30, zf - 1.12))   # kicker foot stake
        m.add(node, box_between('timber', (x, 1.70, zf - 0.08), (x, 0.03, zf - 1.20), 0.05, 0.10, (1, 0, 0)))
    for x in (-2.40, 0.0):
        post(x, p)
    for y in (0.30, 1.20, 2.10):      # rails behind the plywood
        m.add(p, box_span('timber', -2.35, -0.05, y - 0.05, y + 0.05, zf - 0.06, zf - 0.01))
    for x0 in (-2.40, -1.20):         # two 1.2 x 2.4 sheets, 18 mm
        m.add(p, box_span('plywood', x0 + 0.002, x0 + 1.198, 0.02, H, zf + 0.001, zf + 0.019))
    m.add(p, box_span('plywood', -2.40, 0.0, H - 0.0, H + 0.03, zf - 0.095, zf + 0.04))        # capping board
    # gate section: gate post at the far end, leaf hinged on the shared post at x = 0
    g = 'gate_section'
    post(2.40, g)
    gt = m.node('gate_rotate', g, (0.06, 1.20, zf + 0.03))
    x0, x1 = 0.08, 2.32
    y0, y1 = 0.05, 2.35
    for a, b in (((x0, y0), (x1, y0)), ((x0, y1), (x1, y1)), ((x0, 1.20), (x1, 1.20))):
        m.add(gt, box_between('timber', (a[0], a[1] + 0.05, zf - 0.02), (b[0], b[1] + 0.05 if a[1] < 1 else b[1] - 0.05, zf - 0.02), 0.04, 0.10))
    for x in (x0 + 0.05, x1 - 0.05):
        m.add(gt, box_span('timber', x - 0.05, x + 0.05, y0, y1, zf - 0.04, zf))
    m.add(gt, box_between('timber', (x0 + 0.10, y0 + 0.15, zf - 0.03), (x1 - 0.10, 1.12, zf - 0.03), 0.03, 0.10))   # brace
    m.add(gt, box_span('plywood', x0, x1, y0 + 0.02, y1, zf + 0.001, zf + 0.019))
    for y in (0.40, 2.00):            # strap hinges and pins
        m.add(gt, box_span('steel_dark', x0, x0 + 0.45, y - 0.03, y + 0.03, zf + 0.019, zf + 0.025))
        m.add(g, cyl_between('steel_dark', (0.06, y - 0.06, zf + 0.03), (0.06, y + 0.06, zf + 0.03), 0.012, 8))
    m.add(gt, box_span('steel_dark', x1 - 0.20, x1 - 0.02, 1.10, 1.20, zf + 0.019, zf + 0.025))   # hasp
    m.add(gt, rbox('steel', 0.05, 0.07, 0.025, 0.008, (x1 - 0.10, 1.10, zf + 0.04)))              # padlock
    m.add(gt, cyl_between('steel', (x1 - 0.30, 0.02, zf + 0.04), (x1 - 0.30, 0.45, zf + 0.04), 0.010, 8))  # drop bolt
    return m


def survey():
    """Survey tripod with a total station (1.6 m tall) and a survey stake with a pink flag."""
    m = Model('Survey Tripod and Stake', MATERIALS)
    m.node('tripod')
    hy = 1.30
    t = 'tripod'
    m.add(t, cylinder('aluminum', 0.09, 0.03, 32).at((0, hy - 0.015, 0)))           # head plate
    for k in range(3):
        a = math.radians(90 + 120 * k)
        foot = (0.55 * math.cos(a), 0.003, 0.55 * math.sin(a))
        top = (0.08 * math.cos(a), hy - 0.04, 0.08 * math.sin(a))
        mid = tuple(top[i] + (foot[i] - top[i]) * 0.55 for i in range(3))
        side = (-math.sin(a) * 0.022, 0, math.cos(a) * 0.022)
        for sg in (1, -1):    # twin upper legs (timber-look yellow) and single lower leg
            m.add(t, cyl_between('paint_yellow', tuple(top[i] + sg * side[i] for i in range(3)),
                                 tuple(mid[i] + sg * side[i] for i in range(3)), 0.014, 10))
        m.add(t, cyl_between('aluminum', tuple(mid[i] + (top[i] - mid[i]) * 0.25 for i in range(3)),
                             tuple(foot[i] + (top[i] - foot[i]) * 0.06 for i in range(3)), 0.012, 10))
        m.add(t, box('plastic_black', 0.05, 0.05, 0.07, mid))                       # leg clamp
        m.add(t, cyl_between('steel', tuple(foot[i] + (top[i] - foot[i]) * 0.06 for i in range(3)), foot, 0.012, 10, r_top=0.002))
        m.add(t, box_between('plastic_black', tuple(top[i] for i in range(3)), (0, hy - 0.02, 0), 0.04, 0.03))
    # tribrach with levelling screws
    m.add(t, cylinder('plastic_black', 0.075, 0.04, 24).at((0, hy + 0.02, 0)))
    for k in range(3):
        a = math.radians(30 + 120 * k)
        m.add(t, cylinder('plastic_black', 0.018, 0.035, 12).at((0.06 * math.cos(a), hy + 0.018, 0.06 * math.sin(a))))
    # total station: rotating alidade with standards, telescope tilting between them
    by = hy + 0.04
    ts = m.node('total_station_rotate', t, (0, by, 0))
    m.add(ts, rbox('paint_white', 0.16, 0.06, 0.16, 0.02, (0, by + 0.03, 0)))
    for s in (1, -1):
        m.add(ts, rbox('paint_white', 0.06, 0.16, 0.035, 0.012, (0, by + 0.14, s * 0.07)))
        m.add(ts, box_span('plastic_black', -0.075, -0.0805, by + 0.01, by + 0.055, -0.06, 0.06) if s > 0 else
              box_span('plastic_black', 0.0805, 0.075 + 0.0105, by + 0.01, by + 0.055, -0.06, 0.06))   # keypads
    m.add(ts, box_span('glass', -0.0815, -0.0805, by + 0.03, by + 0.05, -0.04, 0.0))
    m.add(ts, box_span('plastic_black', -0.03, 0.03, by + 0.23, by + 0.25, -0.08, 0.08))       # carrying handle
    for s in (1, -1):
        m.add(ts, box_span('plastic_black', -0.012, 0.012, by + 0.215, by + 0.23, s * 0.07 - 0.01, s * 0.07 + 0.01))
    ty = by + 0.15
    tel = m.node('telescope_tilt', ts, (0, ty, 0))
    m.add(tel, cyl_between('plastic_black', (-0.09, ty, 0), (0.09, ty, 0), 0.032, 20))
    m.add(tel, cylinder('glass', 0.026, 0.004, 20).rotate('z', 90).at((0.092, ty, 0)))
    m.add(tel, cyl_between('paint_white', (0, ty, -0.052), (0, ty, 0.052), 0.04, 20))
    m.add(tel, cyl_between('plastic_black', (0.0, ty + 0.045, 0), (0.06, ty + 0.045, 0), 0.008, 8))  # sight
    # survey stake with flagging tape
    st = m.node('survey_stake')
    sx = 1.40
    m.add(st, box_span('timber', sx - 0.022, sx + 0.022, 0.0, 0.62, -0.022, 0.022))
    m.add(st, box_span('fabric_pink', sx - 0.024, sx + 0.024, 0.50, 0.53, -0.024, 0.024))       # wrap
    for k, (dz, ang, ln) in enumerate(((0.012, -12, 0.32), (-0.012, -20, 0.27))):
        tape = box('fabric_pink', 0.03, ln, 0.002, (0, -ln / 2, 0))
        m.add(st, tape.rotate('z', ang).at((sx + 0.026, 0.52, dz)))
    m.add(st, box_span('fabric_pink', sx - 0.024, sx + 0.024, 0.60, 0.625, -0.024, 0.024))      # painted top
    return m


def stair_tower():
    """Modular steel stair tower: 2.6 x 1.3 m frame, two flights of 1.5 m with landings."""
    m = Model('Stair Tower', MATERIALS)
    m.node('stair_tower_frame')
    m.node('stairs', 'stair_tower_frame')
    f = 'stair_tower_frame'
    hx, hz = 1.30, 0.65
    yj = 0.12                    # base jack height
    L1, L2 = yj + 1.50, yj + 3.00
    r = 0.024
    for x in (-hx, 0.0, hx):
        for z in (-hz, hz):
            m.add(f, box_span('steel_dark', x - 0.08, x + 0.08, 0.0, 0.008, z - 0.08, z + 0.08))   # base plate
            m.add(f, cyl_between('steel', (x, 0.008, z), (x, yj, z), 0.016, 10))                   # screw jack
            m.add(f, cylinder('steel_dark', 0.035, 0.025, 6).at((x, yj - 0.03, z)))
            m.add(f, cyl_between('steel', (x, yj, z), (x, L2 + 1.0, z), r, 12))                    # standard
    for y in (yj + 0.20, L1, L2, L2 + 0.5, L2 + 1.0):
        for z in (-hz, hz):
            m.add(f, cyl_between('steel', (-hx, y, z), (hx, y, z), 0.022, 10))
        for x in (-hx, 0.0, hx):
            m.add(f, cyl_between('steel', (x, y, -hz), (x, y, hz), 0.022, 10))
    for z in (-hz, hz):           # face bracing
        for x0, x1, ya, yb_ in ((-hx, 0.0, yj + 0.2, L1), (0.0, hx, L1, yj + 0.2), (-hx, 0.0, L1, L2), (0.0, hx, L2, L1)):
            m.add(f, cyl_between('steel', (x0, ya, z + (0.04 if z < 0 else -0.04)), (x1, yb_, z + (0.04 if z < 0 else -0.04)), 0.020, 10))
    s = 'stairs'
    run0, run1 = -0.98, 0.98

    def flight(xa, xb, ya, yb_, z0, z1):
        n = 8
        for zs in (z0, z1):
            m.add(s, box_between('steel', (xa, ya, zs), (xb, yb_, zs), 0.03, 0.18))
        for k in range(n):
            t = (k + 0.5) / n
            x = xa + (xb - xa) * t
            y = ya + (yb_ - ya) * (k + 1) / (n + 1) + 0.03
            m.add(s, box_span('aluminum', x - 0.12, x + 0.12, y - 0.025, y, z0 + 0.016, z1 - 0.016))
        for zs in (z0 - 0.02, z1 + 0.02):    # handrails with posts
            pts = [(xa + 0.05, ya + 1.0, zs), (xb - 0.05, yb_ + 1.0, zs)]
            m.add(s, polyline_tube('steel', pts, 0.02, 10))
            for t in (0.05, 0.5, 0.95):
                x = xa + (xb - xa) * t
                y = ya + (yb_ - ya) * t
                m.add(s, cyl_between('steel', (x, y + 0.08, zs), (x, y + 1.0, zs), 0.018, 8))
            m.add(s, polyline_tube('steel', [(xa + 0.05, ya + 0.55, zs), (xb - 0.05, yb_ + 0.55, zs)], 0.016, 8))
    flight(run0, run1, yj, L1, -hz + 0.04, -0.02)
    flight(run1, run0, L1, L2, 0.02, hz - 0.04)
    # landings (steel decks with toe boards)
    for x0, x1, y, z0, z1 in ((run1, hx - 0.02, L1, -hz + 0.03, hz - 0.03), (-hx + 0.02, run0, L2, -hz + 0.03, hz - 0.03),
                              (run0, hx - 0.02, L2, -hz + 0.03, -0.02)):
        m.add(s, box_span('steel', x0, x1, y - 0.04, y, z0, z1))
    for x0, x1 in ((-hx + 0.03, hx - 0.03),):
        for z in (-hz + 0.03, hz - 0.03):
            m.add(s, box_span('timber', x0, x1, L2, L2 + 0.15, z - 0.012, z + 0.012))
    m.add(s, box_span('timber', run1 + 0.01, hx - 0.03, L1, L1 + 0.15, -hz + 0.018, -hz + 0.042))
    m.add(s, box_span('timber', run1 + 0.01, hx - 0.03, L1, L1 + 0.15, hz - 0.042, hz - 0.018))
    m.add(s, box_span('timber', hx - 0.06, hx - 0.03, L1, L1 + 0.15, -hz + 0.06, hz - 0.06))
    m.add(s, box_span('timber', -hx + 0.03, -hx + 0.06, L2, L2 + 0.15, -hz + 0.06, hz - 0.06))
    return m


MODELS = [(hoarding, 'Construction/site-hoarding', 'site_hoarding'),
          (survey, 'Construction/survey-tripod-and-stake', 'survey_tripod_and_stake'),
          (stair_tower, 'Construction/stair-tower', 'stair_tower')]

if __name__ == '__main__':
    for fn, folder, name in MODELS:
        save(fn(), folder, name)
