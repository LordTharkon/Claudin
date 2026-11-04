"""Farmyard props in one file, each a separate named top-level node: open-front pole barn
with a closed bay and sliding doors, diesel tank on a stand, bunded chemical IBC, pallets of
seed and fertiliser bags, loose feed sacks, wrapped silage bales (black, white, green), a
silage clamp wall section, a stack of weighting tyres, wheelbarrow, pitchfork, shovel, poly
water tank, jerry can, a farm entrance with a blank sign and a cattle grid.

Key real-world dimensions used (metres):
  pole barn 18.0 x 9.0 (four 4.5 m bays), 4.30 eave on the open front, ridge 5.55; the
  row-crop tractor (6.25 x 4.01 x 3.45) fits each 4.3 m clear bay under the eave
  diesel tank ~1000 US gal (3.8 m3): 1.30 dia x 2.90 on a 1.2 m stand; IBC 1.2 x 1.0 x 1.16
  bags 0.60 x 0.40 x 0.14 on 1.2 x 1.0 pallets; wrapped bales 1.25 dia x 1.20
  poly water tank 5000 L (1.90 dia x 2.0); cattle grid 3.6 x 2.4
"""
import math

from farm_parts import FARM_MATERIALS, FRAME, GALV, corrugated_sheet, hose, save_farm
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from bodykit import rbox

TIMBER = 'timber'


# --------------------------------------------------------------- pole barn ---

BL, BD = 18.0, 9.0           # length (x), depth (z)
YE, YR = 4.30, 5.55          # eave, ridge
BAY = 4.5


def roof_y(z):
    """Underside of the roof sheets at depth z (open front at z = +BD/2)."""
    return YE + (YR - YE) * (1 - abs(z) / (BD / 2))


def pole_barn(m, x0):
    n = m.node('pole_barn')
    zf, zb = BD / 2, -BD / 2
    xs = [x0 + BAY * k for k in range(5)]
    # concrete floor slab and pole footings
    m.add(n, box_span('concrete', x0 - 0.2, x0 + BL + 0.2, 0.0, 0.12, zb - 0.2, zf + 0.2))
    # posts (front and back) and trusses at each frame
    for x in xs:
        for z in (zf - 0.10, zb + 0.10):
            m.add(n, box_span(TIMBER, x - 0.10, x + 0.10, 0.12, YE - 0.02, z - 0.10, z + 0.10))
        a, b, c = (x, roof_y(zf - 0.10) - 0.11, zf - 0.10), (x, YR - 0.11, 0.0), (x, roof_y(zb + 0.10) - 0.11, zb + 0.10)
        m.add(n, box_between(TIMBER, (x, YE - 0.10, zf), (x, YE - 0.10, zb), 0.08, 0.20, (0, 1, 0)))     # bottom chord
        for p0, p1 in ((a, b), (c, b)):
            m.add(n, box_between(TIMBER, p0, p1, 0.08, 0.18, (0, 1, 0)))   # rafters below the purlins
        for f in (0.33, 0.66):
            for z in (f * zf, f * zb):
                yt = roof_y(z) - 0.12
                m.add(n, box_between(TIMBER, (x, YE - 0.05, z), (x, yt - 0.10, z), 0.06, 0.10, (1, 0, 0)))
        m.add(n, box_between(TIMBER, (x, YE - 0.05, 0.0), (x, YR - 0.25, 0.0), 0.06, 0.12, (1, 0, 0)))
    # purlins along x on both slopes, eave beams, wall girts
    for k in range(5):
        for zs in (1, -1):
            z = zs * (BD / 2 - 0.20 - 1.05 * k)
            m.add(n, box_span(TIMBER, x0 - 0.10, x0 + BL + 0.10, roof_y(z) - 0.12, roof_y(z) - 0.025, z - 0.04, z + 0.04))
    for z in (zf - 0.10, zb + 0.10):
        m.add(n, box_span(TIMBER, x0 - 0.10, x0 + BL + 0.10, YE - 0.30, YE - 0.02, z - 0.11, z + 0.11))
    for y in (1.2, 2.6):
        m.add(n, box_span(TIMBER, x0, x0 + BL, y, y + 0.15, zb + 0.20, zb + 0.25))
        for x in (x0 + 0.10, x0 + BL - 0.10):
            m.add(n, box_span(TIMBER, x - 0.025, x + 0.025, y, y + 0.15, zb + 0.20, zf - 0.20))
    # roof: corrugated sheets on both slopes with a ridge cap
    for zs in (1, -1):
        z_out = zs * (BD / 2 + 0.30)
        y_out = roof_y(BD / 2) - (YR - YE) / (BD / 2) * 0.30
        org = (x0 - 0.20 if zs > 0 else x0 + BL + 0.20, y_out, z_out)
        v = (0.0, YR - y_out, -z_out)
        ln = math.hypot(v[1], v[2])
        m.add(n, corrugated_sheet(GALV, org, (zs, 0, 0), (0, v[1] / ln, v[2] / ln), BL + 0.40, ln,
                                  0.20, 0.025))      # crests up on both slopes
    m.add(n, box_between(GALV, (x0 - 0.20, YR + 0.05, 0.0), (x0 + BL + 0.20, YR + 0.05, 0.0), 0.50, 0.03, (0, 1, 0)))
    # back wall and gable ends clad in vertical corrugated sheet
    m.add(n, corrugated_sheet(GALV, (x0 + BL + 0.10, 0.12, zb - 0.02), (-1, 0, 0), (0, 1, 0), BL + 0.20, YE - 0.12, 0.20, 0.025))
    for xe, ux in ((x0 - 0.02, (0, 0, 1)), (x0 + BL + 0.02, (0, 0, -1))):
        zs0 = zb - 0.02 if ux[2] > 0 else zf + 0.02
        m.add(n, corrugated_sheet(GALV, (xe, 0.12, zs0), ux, (0, 1, 0), BD + 0.04, YE - 0.12, 0.20, 0.025))
        tri = [(zb, YE), (zf, YE), (0.0, YR - 0.02)]
        m.add(n, extrude(GALV, tri, 0.006).rotate('y', 90).move(x=xe + (-0.004 if ux[2] > 0 else 0.004)))
    # closed bay at the +x end: partition wall and a door track across the front
    xp = x0 + BL - BAY
    m.add(n, corrugated_sheet(GALV, (xp + 0.12, 0.12, zf - 0.20), (0, 0, -1), (0, 1, 0), BD - 0.43, YE - 0.42, 0.20, 0.025))
    m.add(n, box_span('steel_dark', xp - 0.10, x0 + BL + 0.10, YE - 0.42, YE - 0.30, zf + 0.02, zf + 0.10))
    for k, side in ((0, 'left'), (1, 'right')):
        dx0 = xp + 0.05 + k * (BAY / 2)
        d = m.node('barn_door_%s_slide' % side, n, (dx0, YE - 0.36, zf + 0.20))
        w = BAY / 2 - 0.05
        m.add(d, corrugated_sheet(GALV, (dx0, 0.08, zf + 0.15), (1, 0, 0), (0, 1, 0), w, YE - 0.55, 0.20, 0.025))
        for y in (0.10, 1.80, YE - 0.55):
            m.add(d, box_span(TIMBER, dx0, dx0 + w, y, y + 0.12, zf + 0.18, zf + 0.23))
        for x in (dx0, dx0 + w - 0.10):
            m.add(d, box_span(TIMBER, x, x + 0.10, 0.10, YE - 0.43, zf + 0.18, zf + 0.23))
        m.add(d, box_between(TIMBER, (dx0 + 0.10, 0.22, zf + 0.205), (dx0 + w - 0.10, 1.80, zf + 0.205), 0.05, 0.10, (0, 0, 1)))
        for x in (dx0 + 0.30, dx0 + w - 0.30):
            m.add(d, box_span('steel_dark', x - 0.03, x + 0.03, YE - 0.48, YE - 0.30, zf + 0.10, zf + 0.15))
            m.add(d, cylinder('steel_dark', 0.05, 0.03, 12).rotate('x', 90).at((x, YE - 0.32, zf + 0.06)))
        m.add(d, box_span('steel', dx0 + (w - 0.25 if k == 0 else 0.15), dx0 + (w - 0.15 if k == 0 else 0.25), 1.00, 1.30, zf + 0.23, zf + 0.26))
    return x0 + BL


# ----------------------------------------------------------- fuel, chemical ---

def diesel_tank(m, x0, z0):
    n = m.node('diesel_tank')
    R, L, Y = 0.65, 2.90, 1.20 + 0.65
    xc = x0 + L / 2
    m.add(n, cyl_between(FRAME, (x0, Y, z0), (x0 + L, Y, z0), R, 28))
    for x in (x0, x0 + L):
        m.add(n, cylinder(FRAME, R - 0.02, 0.04, 28).rotate('z', 90).scale(0.6, 1, 1).at((x + (0.012 if x > xc else -0.012), Y, z0)))
    # stand: four legs, braces, cradles, ladder
    for x in (x0 + 0.40, x0 + L - 0.40):
        m.add(n, box_span(GALV, x - 0.06, x + 0.06, Y - R - 0.10, Y - R + 0.15, z0 - R, z0 + R))
        for s in (1, -1):
            m.add(n, box_span(GALV, x - 0.04, x + 0.04, 0.03, Y - R - 0.10, z0 + s * (R - 0.10) - 0.04, z0 + s * (R - 0.10) + 0.04))
            m.add(n, box_span('concrete', x - 0.15, x + 0.15, 0.0, 0.03, z0 + s * (R - 0.10) - 0.15, z0 + s * (R - 0.10) + 0.15))
        m.add(n, box_between(GALV, (x, 0.10, z0 - R + 0.10), (x, Y - R - 0.12, z0 + R - 0.10), 0.03, 0.05, (1, 0, 0)))
    for s in (1, -1):
        m.add(n, box_between(GALV, (x0 + 0.40, 0.20, z0 + s * (R - 0.10)), (x0 + L - 0.40, 0.20, z0 + s * (R - 0.10)), 0.04, 0.05))
    # fill cap, vent, gauge, outlet valve, hose and nozzle hung on a hook
    m.add(n, cylinder('steel', 0.08, 0.10, 14).at((xc - 0.6, Y + R + 0.04, z0)))
    m.add(n, cyl_between('steel', (xc + 0.6, Y + R - 0.02, z0), (xc + 0.6, Y + R + 0.35, z0), 0.02, 8))
    m.add(n, cylinder('steel', 0.04, 0.05, 10).at((xc + 0.6, Y + R + 0.37, z0)))
    m.add(n, cylinder('plastic_black', 0.07, 0.04, 14).at((xc, Y + R + 0.01, z0)))
    m.add(n, box_span('steel_dark', x0 + L + 0.04, x0 + L + 0.15, Y - R + 0.10, Y - R + 0.22, z0 - 0.06, z0 + 0.06))
    m.add(n, hose([(x0 + L + 0.15, Y - R + 0.16, z0), (x0 + L + 0.35, Y - R - 0.30, z0 + 0.10), (x0 + L + 0.30, 0.80, z0 + 0.25),
                   (x0 + L - 0.05, 0.65, z0 + R + 0.05), (x0 + L - 0.40, 1.00, z0 + R + 0.06)], 0.016))
    m.add(n, box_span(GALV, x0 + L - 0.44, x0 + L - 0.36, 0.95, 1.12, z0 + R - 0.08, z0 + R + 0.02))
    m.add(n, box_between('steel_dark', (x0 + L - 0.40, 1.00, z0 + R + 0.06), (x0 + L - 0.40, 0.80, z0 + R + 0.10), 0.05, 0.08))
    m.add(n, cyl_between('steel', (x0 + L - 0.40, 0.80, z0 + R + 0.10), (x0 + L - 0.40, 0.62, z0 + R + 0.18), 0.012, 8))
    return x0 + L + 1.2


def chemical_ibc(m, x0, z0):
    n = m.node('chemical_ibc')
    xc = x0 + 0.75
    # bund (spill pallet) tray with grating
    m.add(n, box_span('paint_gray_dark', xc - 0.75, xc + 0.75, 0.0, 0.40, z0 - 0.70, z0 + 0.70))
    m.add(n, box_span('steel', xc - 0.70, xc + 0.70, 0.40, 0.43, z0 - 0.65, z0 + 0.65))
    # pallet base, white poly bottle, steel cage, valve and cap
    y0 = 0.43
    m.add(n, box_span(FRAME, xc - 0.60, xc + 0.60, y0, y0 + 0.15, z0 - 0.50, z0 + 0.50))
    m.add(n, rbox('paint_white', 1.10, 0.95, 0.92, 0.08, (xc, y0 + 0.65, z0)))
    m.add(n, cylinder('plastic_black', 0.08, 0.04, 14).at((xc, y0 + 1.14, z0)))
    for x in (xc - 0.58, xc + 0.58):
        for z in (z0 - 0.48, z0 + 0.48):
            m.add(n, cyl_between('steel', (x, y0 + 0.15, z), (x, y0 + 1.16, z), 0.012, 6))
    for k in range(4):
        y = y0 + 0.20 + 0.32 * k
        pts = [(xc - 0.58, y, z0 - 0.48), (xc + 0.58, y, z0 - 0.48), (xc + 0.58, y, z0 + 0.48), (xc - 0.58, y, z0 + 0.48), (xc - 0.58, y, z0 - 0.48)]
        for a, b in zip(pts, pts[1:]):
            m.add(n, cyl_between('steel', a, b, 0.010, 6, caps=False))
    for k in range(5):
        x = xc - 0.58 + 1.16 * k / 4
        m.add(n, cyl_between('steel', (x, y0 + 1.16, z0 - 0.48), (x, y0 + 1.16, z0 + 0.48), 0.010, 6))
    m.add(n, cyl_between('plastic_black', (xc + 0.58, y0 + 0.30, z0), (xc + 0.70, y0 + 0.30, z0), 0.04, 12))
    m.add(n, box_span('paint_orange', xc + 0.66, xc + 0.70, y0 + 0.36, y0 + 0.40, z0 - 0.02, z0 + 0.02))
    return xc + 0.75 + 1.0


# ---------------------------------------------------------------- bagged ---

def pallet(x, z):
    out = [box_span(TIMBER, x - 0.60, x + 0.60, 0.122, 0.144, z - 0.50, z + 0.50)]
    for dz in (-0.45, 0.0, 0.45):
        out.append(box_span(TIMBER, x - 0.58, x + 0.58, 0.022, 0.122, z + dz - 0.05, z + dz + 0.05))
    out.append(box_span(TIMBER, x - 0.60, x + 0.60, 0.0, 0.022, z - 0.50, z + 0.50))
    return out


def bag(c, along, mat):
    sx, sz = (0.60, 0.40) if along == 'x' else (0.40, 0.60)
    return rbox(mat, sx - 0.01, 0.14, sz - 0.01, 0.05, c, n=1)


def bag_pallet(m, name, x, z, mat, layers=5):
    n = m.node(name)
    m.add(n, pallet(x, z))
    for L in range(layers):
        y = 0.144 + 0.07 + 0.14 * L
        flip = L % 2
        if not flip:
            spots = [(x - 0.30 + 0.60 * i, z - 0.30 + 0.40 * j, 'x') for i in range(2) for j in range(2)] + [(x, z + 0.30, 'x')]
        else:
            spots = [(x - 0.40 + 0.40 * i, z - 0.20, 'z') for i in range(3)] + [(x - 0.30 + 0.60 * i, z + 0.30, 'x') for i in range(2)]
        for bx, bz, al in spots:
            m.add(n, bag((bx, y, bz), al, mat))
    return x + 1.6


def feed_sacks(m, x, z):
    n = m.node('feed_sacks')
    for i, (dx, dz, a) in enumerate(((0.0, 0.0, 0), (0.42, 0.10, 15), (0.20, 0.55, -20))):
        m.add(n, rbox('woven_polypropylene', 0.40, 0.18, 0.75, 0.07, (0, 0.09, 0), n=1)[0].rotate('y', a).at((x + dx, 0, z + dz)))
    m.add(n, rbox('woven_polypropylene', 0.40, 0.18, 0.75, 0.07, (0, 0.09, 0), n=1)[0].rotate('y', 5).at((x + 0.20, 0.18, z + 0.08)))
    m.add(n, rbox('woven_polypropylene', 0.38, 0.70, 0.28, 0.08, (0, 0.35, 0), n=1)[0].rotate('y', 10).at((x + 0.95, 0, z + 0.20)))
    return x + 1.6


# ---------------------------------------------------------------- silage ---

def wrapped_bale(c, mat, axis='z'):
    r, h = 0.625, 0.60
    prof = [(0.0, -h), (r - 0.10, -h), (r - 0.02, -h + 0.03), (r, -h + 0.12), (r, h - 0.12), (r - 0.02, h - 0.03), (r - 0.10, h), (0.0, h)]
    g = lathe(mat, prof, 24)
    g = g.rotate('x', 90) if axis == 'z' else g
    return [g.at(c)]


def silage(m, x0, z0):
    for k, (name, mat) in enumerate((('wrapped_bale_black', 'plastic_black'), ('wrapped_bale_white', 'paint_white'),
                                     ('wrapped_bale_green', 'paint_green'))):
        n = m.node(name)
        x = x0 + 0.7 + 1.56 * k
        m.add(n, wrapped_bale((x, 0.62, z0), mat))
        if k == 0:     # one stood on its end on top of the black bale stack
            m.add(n, wrapped_bale((x, 0.62, z0 + 1.32), mat))
    # clamp wall: precast L-section, 3 m panel
    w = m.node('silage_clamp_wall')
    xw = x0 + 5.0
    prof = [(0.0, 0.0), (1.40, 0.0), (1.40, 0.25), (0.30, 0.25), (0.15, 2.50), (0.0, 2.50)]
    m.add(w, extrude('concrete', [(z, y) for z, y in prof], 3.0).rotate('y', 90).move(x=xw + 1.5, z=z0 - 0.7))
    for x in (xw + 0.4, xw + 2.6):
        m.add(w, cyl_between('steel', (x, 2.50, z0 - 0.62), (x, 2.58, z0 - 0.62), 0.03, 8))
    # tyre stack (old car / tractor tyres used to weight silage sheets)
    t = m.node('tyre_stack')
    tx = xw + 4.2
    prof = [(0.20, -0.09), (0.30, -0.10), (0.36, -0.07), (0.37, 0.0), (0.36, 0.07), (0.30, 0.10), (0.20, 0.09), (0.19, 0.0)]
    prof.append(prof[0])
    tyre = lathe('tyre_rubber', prof, 18)
    for k in range(5):
        m.add(t, tyre.at((tx + 0.02 * (k % 2), 0.10 + 0.20 * k, z0 + 0.03 * ((k + 1) % 3))))
    for k, (dx, dz) in enumerate(((0.85, 0.30), (0.80, -0.50))):
        m.add(t, tyre.at((tx + dx, 0.10, z0 + dz)))
    return tx + 1.6


# ----------------------------------------------------------------- small items ---

def wheelbarrow(m, x, z):
    n = m.node('wheelbarrow')
    # pressed tray (open top, rolled rim), tube frame with handles and grips, legs; wheel toward +x
    yb, hw = 0.40, 0.31
    side = [(x - 0.20, yb + 0.30), (x, yb), (x + 0.55, yb), (x + 0.85, yb + 0.30)]
    for sg in (1, -1):
        m.add(n, extrude(GALV, side, 0.004).move(z=z + sg * hw))
    m.add(n, box_span(GALV, x, x + 0.55, yb, yb + 0.004, z - hw, z + hw))
    m.add(n, box_between(GALV, (x + 0.55, yb + 0.002, z), (x + 0.85, yb + 0.30, z), 2 * hw, 0.004, (0, 0, 1)))
    m.add(n, box_between(GALV, (x, yb + 0.002, z), (x - 0.20, yb + 0.30, z), 2 * hw, 0.004, (0, 0, 1)))
    rim = [(x - 0.20, yb + 0.30, z - hw), (x + 0.85, yb + 0.30, z - hw), (x + 0.85, yb + 0.30, z + hw),
           (x - 0.20, yb + 0.30, z + hw), (x - 0.20, yb + 0.30, z - hw)]
    for a_, b_ in zip(rim, rim[1:]):
        m.add(n, cyl_between(GALV, a_, b_, 0.010, 6))
    for sg in (1, -1):
        A, B = (x + 0.75, 0.30, z + sg * 0.06), (x - 0.85, 0.62, z + sg * 0.26)
        P = lambda t, A=A, B=B: tuple(A[i] + (B[i] - A[i]) * t for i in range(3))
        m.add(n, cyl_between('paint_green', A, P(0.91), 0.016, 8))
        m.add(n, cyl_between('rubber_trim', P(0.90), P(1.0), 0.020, 8))
        m.add(n, cyl_between('paint_green', (x - 0.30, 0.48, z + sg * 0.17), (x - 0.34, 0.0, z + sg * 0.20), 0.013, 6))
    m.add(n, lathe('tyre_rubber', [(0.10, -0.04), (0.17, -0.05), (0.19, 0.0), (0.17, 0.05), (0.10, 0.04), (0.10, -0.04)], 18)
          .rotate('x', 90).at((x + 0.80, 0.19, z)))
    m.add(n, cyl_between('steel', (x + 0.80, 0.19, z - 0.07), (x + 0.80, 0.19, z + 0.07), 0.012, 8))
    return x + 1.4


def hand_tools(m, x, z):
    n = m.node('pitchfork')
    # leaning against nothing: laid flat on the ground along x
    m.add(n, cyl_between(TIMBER, (x, 0.02, z), (x + 1.20, 0.02, z), 0.016, 8))
    m.add(n, box_span('steel', x + 1.20, x + 1.26, 0.01, 0.03, z - 0.10, z + 0.10))
    for k in range(4):
        zz = z - 0.09 + 0.06 * k
        m.add(n, cyl_between('steel', (x + 1.24, 0.015, zz), (x + 1.55, 0.04, zz), 0.005, 5))
    s = m.node('shovel')
    zs = z + 0.55
    m.add(s, cyl_between(TIMBER, (x, 0.02, zs), (x + 1.10, 0.02, zs), 0.016, 8))
    m.add(s, box_span('plastic_black', x - 0.06, x + 0.0, 0.01, 0.04, zs - 0.07, zs + 0.07))
    m.add(s, cyl_between('steel', (x + 1.08, 0.02, zs), (x + 1.20, 0.02, zs), 0.022, 8))
    m.add(s, extrude('steel', [(0.0, -0.13), (0.30, -0.12), (0.38, 0.0), (0.30, 0.12), (0.0, 0.13)], 0.004)
          .rotate('x', 90).at((x + 1.19, 0.012, zs)))
    return x + 1.9


def water_tank(m, x, z):
    n = m.node('poly_water_tank')
    R, H = 0.95, 1.95
    prof = [(0.0, 0.0), (R - 0.05, 0.0), (R, 0.05), (R, H - 0.10), (R - 0.10, H), (0.30, H + 0.08), (0.0, H + 0.08)]
    m.add(n, lathe('paint_green', prof, 28).move(x=x, z=z))
    for k in range(5):
        y = 0.30 + 0.35 * k
        m.add(n, lathe('paint_green', [(R, y - 0.04), (R + 0.015, y - 0.02), (R + 0.015, y + 0.02), (R, y + 0.04)], 28).move(x=x, z=z))
    m.add(n, cylinder('plastic_black', 0.22, 0.05, 20).at((x, H + 0.10, z)))
    m.add(n, cyl_between('steel', (x + R - 0.02, 0.12, z), (x + R + 0.12, 0.12, z), 0.025, 10))
    m.add(n, cyl_between('paint_orange', (x + R + 0.06, 0.12, z), (x + R + 0.06, 0.20, z), 0.012, 6))
    return x + 2.2


def jerry_can(m, x, z):
    n = m.node('jerry_can')
    m.add(n, rbox('paint_red', 0.17, 0.46, 0.35, 0.02, (x, 0.23, z), n=1))
    for zz in (-0.08, 0.0, 0.08):
        m.add(n, box_span('paint_red', x - 0.03, x + 0.03, 0.46, 0.50, z + zz - 0.012, z + zz + 0.012))
    m.add(n, box_span('paint_red', x - 0.03, x + 0.03, 0.49, 0.51, z - 0.10, z + 0.10))
    m.add(n, cylinder('paint_red', 0.03, 0.04, 10).at((x, 0.48, z + 0.14)))
    return x + 0.6


# --------------------------------------------------------------- entrance ---

def farm_entrance(m, x0, z0):
    n = m.node('farm_entrance')
    W = 4.6
    for z in (z0 - W / 2, z0 + W / 2):
        m.add(n, cylinder(TIMBER, 0.14, 4.0, 14).at((x0, 2.0, z)))
        m.add(n, cylinder(TIMBER, 0.15, 0.03, 14).at((x0, 4.015, z)))
    m.add(n, box_span(TIMBER, x0 - 0.10, x0 + 0.10, 3.55, 3.80, z0 - W / 2 - 0.40, z0 + W / 2 + 0.40))     # crossbar
    for z in (z0 - 0.9, z0 + 0.9):
        for y in (3.55,):
            m.add(n, cyl_between('steel', (x0 + 0.06, y, z), (x0 + 0.06, y - 0.25, z), 0.008, 6))
    m.add(n, box_span(TIMBER, x0 + 0.03, x0 + 0.07, 2.65, 3.30, z0 - 1.20, z0 + 1.20))                      # blank sign
    m.add(n, box_span(TIMBER, x0 + 0.02, x0 + 0.08, 2.62, 2.66, z0 - 1.22, z0 + 1.22))
    m.add(n, box_span(TIMBER, x0 + 0.02, x0 + 0.08, 3.29, 3.33, z0 - 1.22, z0 + 1.22))
    for s in (1, -1):
        m.add(n, box_between(TIMBER, (x0, 3.20, z0 + s * (W / 2 - 0.10)), (x0, 3.55, z0 + s * (W / 2 - 0.55)), 0.10, 0.10, (1, 0, 0)))
    # cattle grid between the posts: concrete pit walls and steel rails
    g = m.node('cattle_grid')
    L, D = 2.4, 3.6
    xa = x0 - L / 2
    m.add(g, box_span('gravel', xa, xa + L, 0.0, 0.002, z0 - D / 2 + 0.15, z0 + D / 2 - 0.15))
    for k in range(13):
        x = xa + 0.10 + (L - 0.20) * k / 12
        m.add(g, box_span(FRAME, x - 0.04, x + 0.04, 0.002, 0.10, z0 - D / 2 + 0.10, z0 + D / 2 - 0.10))
    for s in (1, -1):
        m.add(g, box_span('concrete', xa - 0.15, xa + L + 0.15, 0.0, 0.25, z0 + s * (D / 2 + 0.10) - 0.10, z0 + s * (D / 2 + 0.10) + 0.10))
    return x0 + 2.0


def build():
    m = Model('Farmyard Props', FARM_MATERIALS)
    pole_barn(m, -20.0)
    # yard items in two rows beside the barn
    zA, zB = -2.6, 2.4
    x = diesel_tank(m, 0.0, zA)
    x = chemical_ibc(m, x, zA)
    x = bag_pallet(m, 'seed_bag_pallet', x + 0.6, zA, 'paint_green')
    x = bag_pallet(m, 'fertiliser_bag_pallet', x, zA, 'paint_white')
    x = feed_sacks(m, x, zA - 0.3)
    x = water_tank(m, x + 1.0, zA)
    jerry_can(m, x, zA)
    x = silage(m, 0.0, zB)
    x = wheelbarrow(m, x + 1.0, zB)
    x = hand_tools(m, x + 0.1, zB - 0.3)
    farm_entrance(m, x + 1.6, zB)
    return m


if __name__ == '__main__':
    save_farm(build(), 'farmyard-props', 30000)
