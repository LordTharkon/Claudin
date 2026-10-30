"""Site materials: rebar bundle, lumber stacks, concrete pipes, sandbags, rubble and gravel
piles, skip bin and bulk bag."""
import math
import random

from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe,
                     polyline_tube, save)
from bodykit import rbox
from site_parts import chunk, mound


def rebar_bundle():
    """30 x 25 mm bars, 12 m long, strapped with 3 steel ties, on timber dunnage."""
    m = Model('Rebar Bundle', MATERIALS)
    rng = random.Random(7)
    m.node('dunnage')
    m.node('rebar_bars', 'dunnage')
    r, L = 0.0125, 12.0
    yb = 0.10
    pts = []
    rows = [6, 7, 6, 5, 4, 2]          # hex-packed, flattened bundle as it sits on the dunnage
    for i, nrow in enumerate(rows):
        y = yb + r + i * r * 1.75
        for j in range(nrow):
            z = (j - (nrow - 1) / 2) * 2 * r * 1.02
            pts.append((y, z))
    for y, z in pts:
        dx0, dx1 = rng.uniform(-0.06, 0.06), rng.uniform(-0.06, 0.06)
        m.add('rebar_bars', cyl_between('steel_dark', (-L / 2 + dx0, y, z), (L / 2 + dx1, y, z), r, 8))
    top = max(y for y, _ in pts) + r
    zw = max(abs(z) for _, z in pts) + r
    for x in (-4.5, 0.0, 4.5):            # strap ties: band round the bundle with a buckle
        band = [(-zw - 0.004, yb - 0.002), (zw + 0.004, yb - 0.002), (zw + 0.004, top * 0.55 + yb * 0.45),
                (zw * 0.45, top + 0.004), (-zw * 0.45, top + 0.004), (-zw - 0.004, top * 0.55 + yb * 0.45)]
        outer = [(z + (0.003 if z > 0 else -0.003), y + (0.003 if y > (top + yb) / 2 else -0.003)) for z, y in band]
        m.add('rebar_bars', _band(band, outer, x))
        m.add('rebar_bars', box_span('steel', x - 0.03, x + 0.03, top - 0.002, top + 0.018, -0.03, 0.03))
    for x in (-3.5, 1.0, 5.0):
        m.add('dunnage', box_span('timber', x - 0.05, x + 0.05, 0.0, yb, -0.50, 0.50))
    return m


def _band(inner, outer, x, width=0.032):
    """Closed strap loop: inner and outer outlines (z, y) swept 'width' along X at x."""
    from bodykit import loft
    n = len(inner)
    geos = []
    for k in range(n):
        a_i, b_i = inner[k], inner[(k + 1) % n]
        a_o, b_o = outer[k], outer[(k + 1) % n]
        sec0 = [(x - width / 2, a_i[1], a_i[0]), (x + width / 2, a_i[1], a_i[0]), (x + width / 2, a_o[1], a_o[0]), (x - width / 2, a_o[1], a_o[0])]
        sec1 = [(x - width / 2, b_i[1], b_i[0]), (x + width / 2, b_i[1], b_i[0]), (x + width / 2, b_o[1], b_o[0]), (x - width / 2, b_o[1], b_o[0])]
        geos += loft('steel', [sec0, sec1])
    return geos


def lumber_stack():
    """Three separate stacks: a 2x4 bundle, a plywood sheet stack and timber beams, each on bearers."""
    m = Model('Lumber Stack', MATERIALS)
    rng = random.Random(3)
    # 2x4 bundle: 8 wide x 10 high, 3.66 m long, two steel bands
    b = m.node('lumber_2x4_bundle')
    zc, w, t, L = -1.55, 0.089, 0.038, 3.66
    yb = 0.10
    for x in (-1.4, 0.0, 1.4):
        m.add(b, box_span('timber', x - 0.05, x + 0.05, 0.0, yb, zc - 0.45, zc + 0.45))
    for i in range(10):
        for j in range(8):
            dx = rng.uniform(-0.015, 0.015)
            y0 = yb + i * (t + 0.001)
            z0 = zc + (j - 3.5) * (w + 0.001)
            m.add(b, box_span('timber', -L / 2 + dx, L / 2 + dx, y0, y0 + t, z0 - w / 2, z0 + w / 2))
    top = yb + 10 * (t + 0.001)
    hz = 4 * (w + 0.001)
    for x in (-1.0, 1.0):
        m.add(b, box_span('steel', x - 0.016, x + 0.016, top, top + 0.002, zc - hz - 0.002, zc + hz + 0.002))
        for s in (1, -1):
            m.add(b, box_span('steel', x - 0.016, x + 0.016, yb - 0.002, top + 0.002, zc + s * (hz + 0.002) - (0.002 if s > 0 else 0),
                              zc + s * (hz + 0.002) + (0 if s > 0 else 0.002)))
    # plywood stack: 30 sheets of 18 mm, 2.44 x 1.22 m
    p = m.node('plywood_stack')
    zc = 0.0
    for x in (-1.0, 0.0, 1.0):
        m.add(p, box_span('timber', x - 0.05, x + 0.05, 0.0, yb, zc - 0.65, zc + 0.65))
    for i in range(30):
        dx, dz = rng.uniform(-0.008, 0.008), rng.uniform(-0.008, 0.008)
        y0 = yb + i * 0.0182
        m.add(p, box_span('plywood', -1.22 + dx, 1.22 + dx, y0, y0 + 0.018, zc - 0.61 + dz, zc + 0.61 + dz))
    # timber beams: 2 layers of 3 x (150 x 300) beams, 6 m, with stickers between layers
    tb = m.node('timber_beams')
    zc = 1.75
    for x in (-2.4, -0.8, 0.8, 2.4):
        m.add(tb, box_span('timber', x - 0.05, x + 0.05, 0.0, yb, zc - 0.55, zc + 0.55))
    for layer in range(2):
        y0 = yb + layer * (0.15 + 0.025)
        for j in range(3):
            z0 = zc + (j - 1) * 0.32
            dx = rng.uniform(-0.03, 0.03)
            m.add(tb, box_span('timber', -3.0 + dx, 3.0 + dx, y0, y0 + 0.15, z0 - 0.15, z0 + 0.15))
        if layer == 0:
            for x in (-2.4, -0.8, 0.8, 2.4):
                m.add(tb, box_span('timber', x - 0.025, x + 0.025, y0 + 0.15, y0 + 0.175, zc - 0.50, zc + 0.50))
    return m


def concrete_pipes():
    """Three 1.2 m (internal) socketed concrete pipes on bearers, chocked with timber wedges."""
    m = Model('Concrete Pipes', MATERIALS)
    m.node('bearers')
    ri, ro, rb, L = 0.60, 0.72, 0.80, 2.44
    yb = 0.10
    cy = yb + ro
    prof = [(ri, -L / 2), (ro, -L / 2), (ro, L / 2 - 0.26), (rb, L / 2 - 0.22), (rb, L / 2), (ro - 0.04, L / 2),
            (ro - 0.04, L / 2 - 0.10), (ri, L / 2 - 0.13), (ri, -L / 2)]
    for s in (1, -1):
        m.add('bearers', box_span('timber', -2.65, 2.65, 0.0, yb, s * 0.75 - 0.075, s * 0.75 + 0.075))
    dx = math.sqrt(ro ** 2 - (ro - 0.06) ** 2)     # where the pipe wall is 60 mm above the bearer
    for i, x in enumerate((-1.75, 0.0, 1.75)):
        n = m.node('concrete_pipe_%d' % (i + 1), 'bearers')
        m.add(n, lathe('concrete', prof, 48).rotate('x', 90 if i % 2 else -90).at((x, cy, 0)))
        for side in (1, -1):          # wedges either side of the pipe on each bearer
            x0 = x + side * dx
            wedge = [(x0, yb), (x0 + side * 0.26, yb), (x0, yb + 0.12)]
            for s in (1, -1):
                m.add(n, extrude('timber', wedge, 0.12).move(z=s * 0.75))
    return m


def sandbags():
    """Sandbag wall three courses high (two rows deep) plus two loose bags."""
    m = Model('Sandbag Stack', MATERIALS)
    rng = random.Random(11)
    m.node('sandbag_wall')
    m.node('loose_sandbags')
    L, W, H = 0.60, 0.33, 0.12

    def bag(cx, cy, cz, yaw):
        g = rbox('woven_polypropylene', L, H, W, 0.05, (0, 0, 0)) + \
            rbox('woven_polypropylene', 0.08, 0.05, 0.12, 0.02, (L / 2 + 0.03, 0.0, 0))
        return [q.rotate('y', yaw).at((cx, cy, cz)) for q in g]
    for course, nbag in enumerate((6, 5, 4)):
        y = H / 2 + course * (H - 0.012)
        for row in (-1, 1) if course < 2 else (0,):
            for k in range(nbag):
                x = (k - (nbag - 1) / 2) * (L + 0.02)
                m.add('sandbag_wall', bag(x + rng.uniform(-0.02, 0.02), y, row * (W / 2 + 0.01) + rng.uniform(-0.015, 0.015),
                                           rng.uniform(-3, 3)))
    m.add('loose_sandbags', bag(2.35, H / 2, 0.25, 25))
    m.add('loose_sandbags', bag(2.20, H / 2, -0.60, -40))
    return m


def rubble_and_gravel():
    """Rubble heap with broken concrete and rebar ends, and a smooth gravel cone (separate nodes)."""
    m = Model('Rubble and Gravel Piles', MATERIALS)
    rng = random.Random(5)
    r = m.node('rubble_heap')
    cx = -1.9
    m.add(r, mound('gravel', 1.6, 1.25, 0.85, rng, rough=0.18, center=(cx, 0, 0)))
    for _ in range(45):
        a = rng.uniform(0, 2 * math.pi)
        d = math.sqrt(rng.uniform(0, 1)) * 0.95
        x, z = cx + 1.6 * d * math.cos(a), 1.25 * d * math.sin(a)
        y = 0.85 * (1 - d ** 1.6) * rng.uniform(0.75, 0.95)
        s = rng.uniform(0.15, 0.45)
        sy = s * rng.uniform(0.4, 0.7)
        m.add(r, chunk('concrete', (x, max(y, sy / 2), z), (s, sy, s * rng.uniform(0.6, 1.0)), rng))   # never below ground
    for _ in range(9):              # bent rebar ends poking out
        a = rng.uniform(0, 2 * math.pi)
        d = rng.uniform(0.2, 0.75)
        x, z = cx + 1.6 * d * math.cos(a), 1.25 * d * math.sin(a)
        y = 0.85 * (1 - d ** 1.6) * 0.8
        p0 = (x, y - 0.1, z)
        p1 = (x + rng.uniform(-0.1, 0.1), y + rng.uniform(0.25, 0.45), z + rng.uniform(-0.1, 0.1))
        p2 = (p1[0] + rng.uniform(-0.25, 0.25), p1[1] + rng.uniform(-0.05, 0.10), p1[2] + rng.uniform(-0.25, 0.25))
        m.add(r, polyline_tube('steel_dark', [p0, p1, p2], 0.008, 8))
    g = m.node('gravel_pile')
    gx, R, H = 1.9, 1.35, 0.85
    prof = [(R * (1 - t), H * (1 - (1 - t) ** 1.15) * (1 - 0.25 * math.sin(math.pi * t) ** 2) + 0.0) for t in
            [i / 16 for i in range(17)]]
    prof = [(R, 0.0)] + prof[1:-1] + [(0.0, H)]
    prof = [(0.0, 0.0)] + prof
    m.add(g, lathe('gravel', prof, 48).at((gx, 0, 0)))
    for _ in range(30):             # stray stones at the toe
        a = rng.uniform(0, 2 * math.pi)
        d = R * rng.uniform(1.0, 1.15)
        s = rng.uniform(0.03, 0.06)
        m.add(g, chunk('gravel', (gx + d * math.cos(a), s * 0.35, d * math.sin(a)), (s, s * 0.7, s), rng))
    return m


def skip_and_bulk_bag():
    """6 cu yd skip bin and a bulk bag of sand on a pallet (separate nodes)."""
    m = Model('Skip Bin and Bulk Bag', MATERIALS)
    k = m.node('skip_bin')
    P = 'paint_green'
    yb, H, hb, ht, hw = 0.10, 1.25, 1.00, 1.60, 0.85     # floor height, top, half-length bottom/top, half-width
    t = 0.006
    side = [(-hb, yb), (hb, yb), (ht, H), (-ht, H)]
    for s in (1, -1):
        m.add(k, extrude(P, side, t).move(z=s * (hw - t / 2)))
        for x in (-1.0, -0.33, 0.33, 1.0):        # top-hat stiffeners
            xb = x * hb / ht if abs(x) < hb else x * 0.86
            m.add(k, box_between(P, (x * (hb / 1.0) * 0.98, yb + 0.02, s * (hw + 0.02)), (x * (ht / 1.0) * 0.98 * (1.0 if abs(x) < 1 else 0.98), H - 0.04, s * (hw + 0.02)), 0.04, 0.08, (0, 0, 1)))
        for x in (-1.25, 1.25):                    # lifting trunnions
            m.add(k, cyl_between('steel_dark', (x, 0.95, s * hw), (x, 0.95, s * (hw + 0.10)), 0.045, 14))
            m.add(k, cyl_between('steel_dark', (x, 0.95, s * (hw + 0.10)), (x, 0.95, s * (hw + 0.12)), 0.07, 14))
    m.add(k, box_span(P, -hb, hb, yb, yb + t, -hw, hw))
    for sx in (1, -1):
        m.add(k, box_between(P, (sx * hb, yb + t / 2, 0), (sx * ht, H - t / 2, 0), 2 * hw - 2 * t, t, (0, 1, 0)))
    for sx in (1, -1):        # top rim
        m.add(k, box_span(P, sx * ht - (0.06 if sx > 0 else 0), sx * ht + (0 if sx > 0 else 0.06), H - 0.06, H + 0.01, -hw - 0.03, hw + 0.03))
    for s in (1, -1):
        m.add(k, box_span(P, -ht + 0.06, ht - 0.06, H - 0.06, H + 0.01, s * (hw + 0.03) - (0.06 if s > 0 else 0), s * (hw + 0.03) + (0 if s > 0 else 0.06)))
    for s in (1, -1):         # skids
        m.add(k, box_span('chassis_black', -hb + 0.05, hb - 0.05, 0.0, yb, s * 0.55 - 0.05, s * 0.55 + 0.05))
    # bulk bag of sand on a pallet
    bb = m.node('bulk_bag')
    bx, bz = 2.85, 0.0
    for z in (-0.45, 0.0, 0.45):          # pallet bearers and deck boards
        m.add(bb, box_span('timber', bx - 0.60, bx + 0.60, 0.0, 0.10, bz + z - 0.05, bz + z + 0.05))
    for i in range(7):
        x = bx - 0.54 + i * 0.18
        m.add(bb, box_span('timber', x - 0.05, x + 0.05, 0.10, 0.122, bz - 0.50, bz + 0.50))
    m.add(bb, rbox('woven_polypropylene', 0.95, 0.92, 0.95, 0.08, (bx, 0.122 + 0.46, bz)))
    m.add(bb, cylinder('woven_polypropylene', 0.18, 0.10, 24).at((bx, 1.09, bz)))      # top spout
    m.add(bb, cylinder('sand', 0.16, 0.012, 24).at((bx, 1.143, bz)))
    for sx in (1, -1):
        for sz in (1, -1):
            loop = lathe('woven_polypropylene', [(0.10 + 0.015 * math.cos(a), 0.008 * math.sin(a)) for a in
                                                 (2 * math.pi * j / 8 for j in range(9))], 16, 0, 180)
            m.add(bb, loop.rotate('x', -90).rotate('y', 45 * (1 if sx * sz > 0 else -1)).at((bx + sx * 0.40, 1.02, bz + sz * 0.40)))
    return m


MODELS = [(rebar_bundle, 'Construction/rebar-bundle', 'rebar_bundle'),
          (lumber_stack, 'Construction/lumber-stack', 'lumber_stack'),
          (concrete_pipes, 'Construction/concrete-pipes', 'concrete_pipes'),
          (sandbags, 'Construction/sandbag-stack', 'sandbag_stack'),
          (rubble_and_gravel, 'Construction/rubble-and-gravel-piles', 'rubble_and_gravel_piles'),
          (skip_and_bulk_bag, 'Construction/skip-bin-and-bulk-bag', 'skip_bin_and_bulk_bag')]

if __name__ == '__main__':
    for fn, folder, name in MODELS:
        save(fn(), folder, name)
