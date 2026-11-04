"""Livestock props in one file, each a separate named top-level node, laid out in two rows
along X: bale feeder ring, water trough, frost-free waterer, cattle panel fence pieces,
squeeze chute and alley, feed bunk, creep feeder, mineral feeder and lick tub, calf hutch
with wire run, and a portable loading ramp.

Key real-world dimensions used (metres):
  bale feeder ring 2.4 dia, 1.2 high; water trough 3.0 x 1.2 oval, 0.6 deep
  cattle panel 3.6 x 1.65 (6 oval rails), corner post 0.10 sq, gate panel 3.6 with a 1.2 man gate
  squeeze chute 2.6 long, 0.75 inside, 2.0 high; alley 6.0 long, 0.75 wide, 1.65 high
  feed bunk 6.0 x 1.0 concrete U; creep feeder 2.4 x 1.8 on skids; calf hutch 2.2 x 1.3 dome
  loading ramp deck 0.42 at the top (the stock trailer's floor height), 0.80 wide
Fence pieces have their local origin on the ground at their start post so they tile along x.
"""
import math

from farm_parts import FARM_MATERIALS, GALV, FRAME, save_farm
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from site_parts import mesh_panel
from bodykit import rbox

TRAILER_FLOOR = 0.42


def ring_pts(c, r, y, n=24, a0=0.0, a1=360.0):
    return [(c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), y,
             c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def tubes(pts, r, mat=GALV, segs=6):
    return [cyl_between(mat, a, b, r, segs, caps=False) for a, b in zip(pts, pts[1:])]


# ---------------------------------------------------------------- feeders ---

def bale_feeder_ring(m, xc):
    n = m.node('bale_feeder_ring')
    c = (xc, 0.0)
    R = 1.20
    for y, r in ((0.08, 0.030), (0.62, 0.026), (1.18, 0.030)):
        m.add(n, tubes(ring_pts(c, R, y, 32), r))
    # sheeted skirt between the bottom ring and the mid ring
    prof = [(R - 0.006, 0.16), (R + 0.006, 0.16), (R + 0.006, 0.58), (R - 0.006, 0.58), (R - 0.006, 0.16)]
    m.add(n, lathe(GALV, prof, 32).move(x=xc))
    # tombstone head openings: U bars standing on the mid ring
    k = 14
    for i in range(k):
        a0 = 360.0 * i / k
        aw = 360.0 / k * 0.36
        pts = [ring_pts(c, R, 0.62, 1, a0 - aw, a0 - aw)[0]]
        for j in range(7):
            t = math.pi * j / 6
            ang = a0 - aw * math.cos(t)
            p = ring_pts(c, R, 0.98 + 0.12 * math.sin(t), 1, ang, ang)[0]
            pts.append(p)
        pts.append(ring_pts(c, R, 0.62, 1, a0 + aw, a0 + aw)[0])
        m.add(n, tubes(pts, 0.016))
    # uprights at the section joints (two halves bolted together), feet
    for i in range(6):
        a = 360.0 * (i + 0.5) / 6
        p0 = ring_pts(c, R, 0.0, 1, a, a)[0]
        m.add(n, cyl_between(GALV, p0, (p0[0], 1.20, p0[2]), 0.024, 8))
    for a in (90.0 / 14, 180 + 90.0 / 14):
        p = ring_pts(c, R + 0.03, 0.62, 1, a, a)[0]
        m.add(n, box(GALV, 0.06, 0.08, 0.06, p))
    return xc + R + 1.0


def water_trough(m, x0):
    n = m.node('water_trough')
    L, Wd, H = 3.0, 1.2, 0.60
    xc = x0 + L / 2
    sx = L / Wd
    r = Wd / 2
    prof = [(0.0, 0.0), (r - 0.04, 0.0), (r, 0.04), (r, H - 0.03), (r + 0.02, H - 0.02), (r + 0.02, H + 0.01),
            (r - 0.015, H + 0.01), (r - 0.015, 0.03), (0.0, 0.03)]
    # corrugated wall reads through a few raised bands
    m.add(n, lathe(GALV, prof, 40).scale(sx, 1.0, 1.0).move(x=xc))
    for y in (0.15, 0.30, 0.45):
        m.add(n, lathe(GALV, [(r, y - 0.02), (r + 0.012, y - 0.01), (r + 0.012, y + 0.01), (r, y + 0.02)], 40)
              .scale(sx, 1.0, 1.0).move(x=xc))
    m.add(n, cylinder('glass', r - 0.016, 0.01, 40).scale(sx, 1.0, 1.0).at((xc, H - 0.09, 0.0)))   # water surface
    # drain plug, float-valve cover box at one end with its supply pipe
    m.add(n, cyl_between('steel', (xc + L / 2 - 0.10, 0.05, 0.30), (xc + L / 2 - 0.10, 0.05, 0.40), 0.02, 8))
    fx = x0 + 0.38
    m.add(n, box_span(GALV, fx - 0.25, fx + 0.10, H - 0.10, H + 0.06, -0.22, 0.22))
    m.add(n, box_span(GALV, fx - 0.27, fx + 0.12, H + 0.06, H + 0.08, -0.24, 0.24))
    m.add(n, cyl_between('steel', (fx - 0.27, H + 0.08, -0.20), (fx - 0.27, H + 0.08, 0.20), 0.008, 6))
    m.add(n, cyl_between('plastic_black', (fx - 0.30, 0.02, 0.0), (x0 - 0.40, 0.02, 0.0), 0.02, 8))
    m.add(n, cyl_between('plastic_black', (fx - 0.30, 0.02, 0.0), (fx - 0.30, H - 0.05, 0.0), 0.02, 8))
    return x0 + L + 1.0


def frost_free_waterer(m, x0):
    n = m.node('frost_free_waterer')
    xc = x0 + 0.6
    m.add(n, box_span('concrete', xc - 0.75, xc + 0.75, 0.0, 0.12, -0.60, 0.60))
    m.add(n, rbox('paint_gray', 0.95, 0.62, 0.62, 0.06, (xc, 0.43, 0.0)))
    m.add(n, rbox('paint_gray', 1.00, 0.06, 0.66, 0.025, (xc, 0.76, 0.0)))
    for s in (1, -1):
        bx = xc + s * 0.30
        m.add(n, box_span('paint_gray_dark', bx - 0.17, bx + 0.17, 0.79, 0.82, -0.18, 0.18))
        m.add(n, sphere('plastic_black', 0.15, 16, 8).scale(1.0, 0.45, 1.0).at((bx, 0.86, 0.0)))
    for z in (-0.24, 0.24):
        m.add(n, cyl_between('steel', (xc - 0.48, 0.30, z), (xc - 0.52, 0.30, z), 0.012, 6))
    return xc + 0.75 + 1.0


# ------------------------------------------------------------------ fence ---

def cattle_panel(m, x0, name='cattle_panel'):
    """Continuous 3.6 m panel: six oval rails, end uprights with connector pins."""
    n = m.node(name, None, (x0, 0, 0))
    L = 3.6
    for x in (x0 + 0.02, x0 + L - 0.02):
        m.add(n, cyl_between(GALV, (x, 0.05, 0), (x, 1.65, 0), 0.024, 8))
    for k, y in enumerate((0.25, 0.52, 0.78, 1.04, 1.32, 1.60)):
        m.add(n, cyl_between(GALV, (x0 + 0.04, y, 0), (x0 + L - 0.04, y, 0), 0.025, 6).scale(1.0, 1.0, 0.5).move(z=0))
    for x in (x0 + 1.2, x0 + 2.4):
        m.add(n, cyl_between(GALV, (x, 0.22, 0), (x, 1.63, 0), 0.014, 6))
    for y in (0.40, 1.30):
        m.add(n, cyl_between('steel', (x0 + L - 0.02, y, 0), (x0 + L + 0.02, y, 0), 0.012, 6))
        m.add(n, cyl_between('steel', (x0 + 0.02, y - 0.06, 0), (x0 + 0.02, y + 0.10, 0), 0.016, 6))
    return x0 + L


def corner_post(m, x0):
    n = m.node('fence_corner_post', None, (x0, 0, 0))
    m.add(n, box_span(GALV, x0 - 0.05, x0 + 0.05, 0.05, 1.85, -0.05, 0.05))
    m.add(n, box_span(GALV, x0 - 0.06, x0 + 0.06, 1.85, 1.87, -0.06, 0.06))
    for y in (0.40, 1.30):
        m.add(n, box_span(GALV, x0 + 0.05, x0 + 0.11, y - 0.04, y + 0.04, -0.02, 0.02))     # panel lug, +x
        m.add(n, box_span(GALV, -0.02 + x0, x0 + 0.02, y - 0.04, y + 0.04, 0.05, 0.11))      # panel lug, +z
    m.add(n, box_span('concrete', x0 - 0.20, x0 + 0.20, 0.0, 0.05, -0.20, 0.20))
    return x0 + 0.6


def gate_panel(m, x0):
    """3.6 m gate panel: frame with a 1.2 m man gate hung on the left upright."""
    n = m.node('fence_gate_panel', None, (x0, 0, 0))
    L = 3.6
    for x in (x0 + 0.02, x0 + 1.32, x0 + L - 0.02):
        m.add(n, cyl_between(GALV, (x, 0.05, 0), (x, 1.80 if x < x0 + 1.4 else 1.65, 0), 0.026, 8))
    m.add(n, cyl_between(GALV, (x0 + 0.02, 1.78, 0), (x0 + 1.32, 1.78, 0), 0.026, 8))           # header over the gate
    for y in (0.25, 0.52, 0.78, 1.04, 1.32, 1.60):
        m.add(n, cyl_between(GALV, (x0 + 1.34, y, 0), (x0 + L - 0.04, y, 0), 0.025, 6).scale(1.0, 1.0, 0.5))
    g = m.node('fence_gate_rotate', n, (x0 + 0.06, 0, 0))
    a, b = x0 + 0.08, x0 + 1.26
    for x in (a, b):
        m.add(g, cyl_between(GALV, (x, 0.12, 0), (x, 1.62, 0), 0.020, 8))
    for y in (0.12, 0.50, 0.88, 1.25, 1.62):
        m.add(g, cyl_between(GALV, (a, y, 0), (b, y, 0), 0.018, 6))
    m.add(g, cyl_between(GALV, (a, 0.14, 0), (b, 1.60, 0), 0.014, 6))
    for y in (0.40, 1.40):
        m.add(g, cyl_between('steel', (x0 + 0.06, y - 0.05, 0), (x0 + 0.06, y + 0.05, 0), 0.018, 8))
        m.add(g, cyl_between('steel', (x0 + 0.06, y, 0), (a, y, 0), 0.012, 6))
    m.add(g, cyl_between('steel', (b - 0.10, 1.10, 0), (b + 0.07, 1.10, 0), 0.010, 6))
    m.add(n, box_span('steel', x0 + 1.29, x0 + 1.35, 1.06, 1.14, -0.03, 0.03))
    return x0 + L


# ------------------------------------------------------------ chute, alley ---

def squeeze_chute(m, x0):
    n = m.node('squeeze_chute')
    L, hw, H = 2.60, 0.38, 2.00
    xa, xb = x0, x0 + L
    # base frame and floor with cleats
    for s in (1, -1):
        m.add(n, box_span(FRAME, xa, xb, 0.0, 0.10, s * (hw + 0.05), s * (hw + 0.15)))
    m.add(n, box_span('steel_dark', xa, xb, 0.10, 0.13, -hw - 0.04, hw + 0.04))
    for k in range(9):
        x = xa + 0.15 + 0.28 * k
        m.add(n, box_span('steel', x, x + 0.03, 0.13, 0.15, -hw + 0.02, hw - 0.02))
    # four corner posts, top frame
    for x in (xa + 0.05, xb - 0.05):
        for s in (1, -1):
            m.add(n, box_span(GALV, x - 0.05, x + 0.05, 0.10, H, s * (hw + 0.05), s * (hw + 0.15)))
    for s in (1, -1):
        m.add(n, box_span(GALV, xa, xb, H - 0.08, H, s * (hw + 0.05), s * (hw + 0.15)))
    for x in (xa + 0.05, xb - 0.05):
        m.add(n, box_span(GALV, x - 0.05, x + 0.05, H - 0.08, H, -hw - 0.05, hw + 0.05))
    # squeeze sides: lower sheet panel, upper bars, levers
    for s in (1, -1):
        m.add(n, box_span(GALV, xa + 0.12, xb - 0.12, 0.15, 0.95, s * (hw + 0.02), s * (hw + 0.03)))
        for y in (1.10, 1.30, 1.50, 1.70):
            m.add(n, box_span(GALV, xa + 0.12, xb - 0.12, y, y + 0.05, s * (hw + 0.02), s * (hw + 0.05)))
        m.add(n, cyl_between(GALV, ((xa + xb) / 2, 1.0, s * (hw + 0.16)), ((xa + xb) / 2 - 0.3, 2.3, s * (hw + 0.30)), 0.02, 8))
        m.add(n, cyl_between('plastic_black', ((xa + xb) / 2 - 0.27, 2.17, s * (hw + 0.29)), ((xa + xb) / 2 - 0.3, 2.3, s * (hw + 0.30)), 0.025, 8))
    # head gate at the front end: two leaves sliding apart along z
    hx = xb + 0.05
    m.add(n, box_span(GALV, hx - 0.05, hx + 0.05, 0.10, H + 0.15, -hw - 0.75, -hw - 0.65))
    m.add(n, box_span(GALV, hx - 0.05, hx + 0.05, 0.10, H + 0.15, hw + 0.65, hw + 0.75))
    m.add(n, box_span(GALV, hx - 0.05, hx + 0.05, H + 0.07, H + 0.15, -hw - 0.75, hw + 0.75))
    m.add(n, box_span(GALV, hx - 0.05, hx + 0.05, 0.10, 0.18, -hw - 0.75, hw + 0.75))
    for s, side in ((-1, 'left'), (1, 'right')):
        g = m.node('head_gate_%s_slide' % side, n, (hx, 0.0, s * 0.10))
        z0, z1 = s * 0.10, s * (hw + 0.50)
        lo, hi = min(z0, z1), max(z0, z1)
        m.add(g, box_span(GALV, hx - 0.03, hx + 0.03, 0.20, H + 0.05, lo, hi))
    m.add(n, cyl_between(GALV, (hx + 0.08, H + 0.11, -0.6), (hx + 0.40, 2.40, -0.9), 0.02, 8))      # head gate lever
    # rear tailgate: guillotine gate sliding up in two guides
    tx = xa - 0.03
    for s in (1, -1):
        m.add(n, box_span(GALV, tx - 0.04, tx + 0.04, 0.02, H + 0.60, s * (hw + 0.02), s * (hw + 0.10)))
    m.add(n, box_span(GALV, tx - 0.04, tx + 0.04, H + 0.52, H + 0.60, -hw - 0.10, hw + 0.10))
    m.add(n, cylinder('steel', 0.05, 0.04, 12).rotate('z', 90).at((tx, H + 0.64, 0)))
    t = m.node('tailgate_slide', n, (tx, 0.18, 0))
    m.add(t, box_span(GALV, tx - 0.015, tx + 0.015, 0.18, 1.70, -hw + 0.0, hw - 0.0))
    for y in (0.40, 0.80, 1.20, 1.60):
        m.add(t, box_span(GALV, tx - 0.03, tx - 0.015, y, y + 0.04, -hw + 0.02, hw - 0.02))
    m.add(t, cyl_between('steel', (tx, 1.70, 0), (tx, H + 0.64, 0), 0.004, 4))
    return xb + 1.6


def alley(m, x0):
    n = m.node('alley_section', None, (x0, 0, 0))
    L, hw, H = 6.0, 0.40, 1.65
    for k in range(4):
        x = x0 + 2.0 * k
        for s in (1, -1):
            m.add(n, box_span(GALV, x - 0.04, x + 0.04, 0.0, H + 0.10, s * (hw + 0.01), s * (hw + 0.09)))
    for s in (1, -1):
        m.add(n, box_span(GALV, x0, x0 + L, 0.20, 1.10, s * (hw + 0.002), s * (hw + 0.008)))       # sheeted lower sides
        for y in (1.28, 1.48, H):
            m.add(n, cyl_between(GALV, (x0, y, s * (hw + 0.03)), (x0 + L, y, s * (hw + 0.03)), 0.025, 6))
    for k in (1, 2):
        x = x0 + 2.0 * k
        m.add(n, cyl_between(GALV, (x, H + 0.08, -hw - 0.05), (x, H + 0.08, hw + 0.05), 0.022, 6))
    return x0 + L + 1.2


# --------------------------------------------------------------- bunk, creep ---

def feed_bunk(m, x0):
    n = m.node('feed_bunk')
    L = 6.0
    prof = [(-0.50, 0.0), (0.50, 0.0), (0.50, 0.70), (0.38, 0.70), (0.34, 0.28), (0.25, 0.20), (-0.25, 0.20),
            (-0.34, 0.28), (-0.38, 0.70), (-0.50, 0.70)]
    m.add(n, extrude('concrete', prof, L).rotate('y', 90).move(x=x0 + L / 2))
    m.add(n, box_span('hay', x0 + 0.40, x0 + L - 0.40, 0.20, 0.30, -0.24, 0.24))
    # neck rail on posts along one side
    for k in range(4):
        x = x0 + 0.20 + (L - 0.40) * k / 3
        m.add(n, cyl_between(GALV, (x, 0.70, 0.44), (x, 1.30, 0.44), 0.03, 8))
    m.add(n, cyl_between(GALV, (x0 + 0.20, 1.25, 0.44), (x0 + L - 0.20, 1.25, 0.44), 0.03, 8))
    return x0 + L + 1.0


def creep_feeder(m, x0):
    n = m.node('creep_feeder')
    Lx, Lz = 2.40, 1.80
    xc = x0 + Lx / 2
    # skids and drawbar loop
    for s in (1, -1):
        m.add(n, box_span(FRAME, x0 - 0.20, x0 + Lx + 0.20, 0.0, 0.12, s * 0.70, s * 0.80))
        m.add(n, box_between(FRAME, (x0 + Lx + 0.20, 0.06, s * 0.75), (x0 + Lx + 0.35, 0.20, s * 0.40), 0.05, 0.05))
    m.add(n, cyl_between(FRAME, (x0 + Lx + 0.35, 0.20, -0.40), (x0 + Lx + 0.35, 0.20, 0.40), 0.03, 8))
    # feed troughs both sides, hopper above
    for s in (1, -1):
        m.add(n, box_span(GALV, x0 + 0.05, x0 + Lx - 0.05, 0.12, 0.42, s * 0.20, s * 0.52))
        m.add(n, box_span('grain', x0 + 0.10, x0 + Lx - 0.10, 0.14, 0.36, s * 0.22, s * 0.48))
    hop = [(0.20, 0.45), (0.55, 0.85), (0.55, 1.45), (-0.55, 1.45), (-0.55, 0.85), (-0.20, 0.45)]
    m.add(n, extrude(GALV, [(z, y) for z, y in hop], Lx - 0.10).rotate('y', 90).move(x=xc))
    # creep panels (bars spaced for calves) around the sides
    for s in (1, -1):
        for k in range(9):
            x = x0 + 0.10 + (Lx - 0.20) * k / 8
            m.add(n, cyl_between(GALV, (x, 0.12, s * (Lz / 2 - 0.02)), (x, 1.10, s * (Lz / 2 - 0.02)), 0.018, 6))
        m.add(n, cyl_between(GALV, (x0, 1.10, s * (Lz / 2 - 0.02)), (x0 + Lx, 1.10, s * (Lz / 2 - 0.02)), 0.022, 6))
        m.add(n, box_between(GALV, (x0 + 0.05, 1.10, s * (Lz / 2 - 0.02)), (x0 + 0.05, 1.45, s * 0.55), 0.04, 0.04))
        m.add(n, box_between(GALV, (x0 + Lx - 0.05, 1.10, s * (Lz / 2 - 0.02)), (x0 + Lx - 0.05, 1.45, s * 0.55), 0.04, 0.04))
    # hinged lid along the ridge
    lid = m.node('creep_feeder_lid_tilt', n, (xc, 1.50, -0.58))
    m.add(lid, box_span(GALV, x0 - 0.02, x0 + Lx + 0.02, 1.47, 1.51, -0.60, 0.62))
    m.add(lid, box_span(GALV, x0 - 0.02, x0 + Lx + 0.02, 1.40, 1.47, 0.58, 0.62))
    for x in (x0 + 0.40, x0 + Lx - 0.40):
        m.add(lid, cyl_between('steel', (x - 0.06, 1.50, -0.58), (x + 0.06, 1.50, -0.58), 0.015, 8))
    m.add(lid, cyl_between('steel', (xc - 0.12, 1.53, 0.55), (xc + 0.12, 1.53, 0.55), 0.010, 6))
    return x0 + Lx + 1.2


def mineral_feeders(m, x0):
    # covered mineral feeder: tub under a pivoting hood on a post
    n = m.node('mineral_feeder')
    xc = x0 + 0.6
    m.add(n, box_span('timber', xc - 0.06, xc + 0.06, 0.0, 0.85, -0.06, 0.06))
    m.add(n, lathe('plastic_black', [(0.0, 0.40), (0.45, 0.40), (0.50, 0.62), (0.47, 0.62), (0.42, 0.44), (0.0, 0.44)], 24).move(x=xc))
    m.add(n, cylinder('woven_polypropylene', 0.43, 0.06, 24).at((xc, 0.47, 0.0)))
    m.add(n, lathe('paint_gray', [(0.0, 1.10), (0.62, 0.78), (0.62, 0.74), (0.0, 1.06)], 24, 40, 320).move(x=xc))
    m.add(n, cyl_between('steel', (xc, 0.85, 0), (xc, 1.10, 0), 0.03, 8))
    # lick tub on the ground
    t = m.node('lick_tub')
    tx = x0 + 1.9
    m.add(t, lathe('plastic_black', [(0.0, 0.0), (0.30, 0.0), (0.34, 0.50), (0.31, 0.50), (0.27, 0.03), (0.0, 0.03)], 24).move(x=tx))
    m.add(t, cylinder('grain', 0.315, 0.02, 24).at((tx, 0.40, 0.0)))
    return x0 + 2.8


def calf_hutch(m, x0):
    n = m.node('calf_hutch')
    L, hw, H = 2.20, 0.65, 1.30
    xb = x0
    # poly dome: arched shell (half-oval section) closed at the back, open front with a lip
    sec = [(hw * math.cos(math.radians(a)), H * math.sin(math.radians(a))) for a in range(0, 181, 15)]
    inner = [(z * 0.96, y * 0.97) for z, y in reversed(sec)]
    shell = [(z, y) for z, y in sec] + inner
    m.add(n, extrude('paint_white', shell, L).rotate('y', 90).move(x=xb + L / 2))
    m.add(n, extrude('paint_white', sec, 0.03).rotate('y', 90).move(x=xb + 0.015))
    m.add(n, box_span('paint_white', xb, xb + L, 0.0, 0.03, -hw - 0.06, -hw + 0.02))
    m.add(n, box_span('paint_white', xb, xb + L, 0.0, 0.03, hw - 0.02, hw + 0.06))
    m.add(n, box_span('paint_white', xb + 0.70, xb + 0.90, H * 0.94, H * 0.94 + 0.06, -0.08, 0.08))     # roof vent
    m.add(n, box_span('hay', xb + 0.10, xb + L - 0.10, 0.0, 0.05, -hw + 0.08, hw - 0.08))
    # wire run in front: three welded-mesh panels and a bucket holder
    rx0, rx1 = xb + L, xb + L + 1.80
    for z in (-hw, hw):
        m.add(n, mesh_panel(GALV, rx0, rx1, 0.0, 0.95, z, 0.10, 0.005, 0.025))
    m.add(n, mesh_panel(GALV, -hw, hw, 0.0, 0.95, rx1, 0.10, 0.005, 0.025, axis='z'))
    m.add(n, box_span(GALV, rx1 - 0.01, rx1 + 0.01, 0.70, 0.73, -0.40, -0.10))
    m.add(n, lathe('paint_red', [(0.0, 0.52), (0.12, 0.52), (0.15, 0.78), (0.13, 0.78), (0.10, 0.55), (0.0, 0.55)], 16)
          .move(x=rx1 + 0.17, z=-0.25))
    return rx1 + 1.2


def loading_ramp(m, x0):
    """Portable loading ramp: top deck at the stock trailer's floor height, ramp down to the
    ground, sheeted side rails. The high end is toward +x (back it up to the trailer)."""
    n = m.node('loading_ramp')
    Y = TRAILER_FLOOR
    hw = 0.40
    xg, xt, xe = x0, x0 + 2.40, x0 + 3.00           # ground end, top of slope, deck end
    deck = [(xg, 0.0), (xt, Y - 0.04), (xe, Y - 0.04), (xe, Y), (xt, Y), (xg, 0.04)]
    m.add(n, extrude('steel_dark', deck, 2 * hw))
    for k in range(10):
        t = (k + 0.5) / 10
        x = xg + (xt - xg) * t
        y = 0.04 + (Y - 0.04) * t
        m.add(n, box_span('timber', x - 0.025, x + 0.025, y - 0.005, y + 0.03, -hw + 0.03, hw - 0.03))
    # frame, legs and skids
    for s in (1, -1):
        m.add(n, box_between(FRAME, (xg, 0.02, s * (hw + 0.03)), (xe, Y - 0.02, s * (hw + 0.03)), 0.06, 0.08, (0, 0, 1)))
        for x in (xt, xe - 0.05):
            m.add(n, box_span(FRAME, x - 0.04, x + 0.04, 0.02, Y - 0.04, s * (hw - 0.04), s * (hw + 0.04)))
        m.add(n, box_span('steel_dark', xt - 0.10, xe + 0.05, 0.0, 0.02, s * (hw - 0.08), s * (hw + 0.08)))
    # side rails: sheeted lower panel and a top rail on posts
    for s in (1, -1):
        z = s * (hw + 0.07)
        for k in range(4):
            x = xg + 0.10 + (xe - xg - 0.15) * k / 3
            ybase = min(Y, (x - xg) / (xt - xg) * Y)
            m.add(n, box_span(GALV, x - 0.03, x + 0.03, ybase, ybase + 1.60, z - 0.03, z + 0.03))
        top = [(xg + 0.10, 1.55), (xt, Y + 1.55), (xe - 0.05, Y + 1.55)]
        ytop = lambda x: min(Y, (x - xg) / (xt - xg) * Y)
        m.add(n, box_between(GALV, (xg + 0.10, ytop(xg + 0.10) + 1.55, z), (xt, Y + 1.55, z), 0.04, 0.05, (0, 0, 1)))
        m.add(n, box_between(GALV, (xt, Y + 1.55, z), (xe - 0.05, Y + 1.55, z), 0.04, 0.05, (0, 0, 1)))
        panel = [(xg + 0.10, ytop(xg + 0.10) + 0.10), (xt, Y + 0.10), (xe - 0.05, Y + 0.10), (xe - 0.05, Y + 1.00),
                 (xt, Y + 1.00), (xg + 0.10, ytop(xg + 0.10) + 1.00)]
        m.add(n, extrude(GALV, panel, 0.006).move(z=z - s * 0.034))
    return xe + 1.0


def shift(m, names, dx, dz):
    """Move whole items (and their child nodes) to their place in the layout."""
    for n in m.order:
        top = n
        while m.nodes[top]['parent']:
            top = m.nodes[top]['parent']
        if top in names:
            m.parts[n] = [g.move(dx, 0.0, dz) for g in m.parts[n]]
            px, py, pz = m.nodes[n]['pivot']
            m.nodes[n]['pivot'] = (px + dx, py, pz + dz)


def build():
    m = Model('Livestock Props', FARM_MATERIALS)
    # back row
    x = bale_feeder_ring(m, -14.0)
    x = water_trough(m, x)
    x = frost_free_waterer(m, x)
    x = cattle_panel(m, x)
    x = corner_post(m, x)
    x = gate_panel(m, x)
    x = mineral_feeders(m, x + 0.6)
    loading_ramp(m, x)
    back = set(m.order)
    shift(m, back, 0.0, -2.0)
    # front row
    x = squeeze_chute(m, -14.0)
    x = alley(m, x)
    x = feed_bunk(m, x)
    x = creep_feeder(m, x)
    calf_hutch(m, x)
    shift(m, set(m.order) - back, 0.0, 2.4)
    return m


if __name__ == '__main__':
    save_farm(build(), 'livestock-props', 25000)
