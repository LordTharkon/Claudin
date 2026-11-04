"""Mine site props, one .glb with a separate named node per prop (laid out in a row along x):

* `lighting_plant`: towable mine-spec lighting plant, 4 LED floods on a 3-stage telescoping
  mast (`mast_lift` -> `mast_stage_2_lift` -> `mast_stage_3_lift`, ~2.0 m each, ~8.5 m
  raised), `lamp_head_rotate`, four outriggers `outrigger_*_slide`, wheels `_spin`
* `safety_berm_section`: modular 6 m safety berm / windrow, 2.0 m high, flat ends so
  sections butt together, a few embedded rocks
* `haul_road_sign`: 2.5 x 1.4 m sign on two posts with a blank face, concrete footings
* `fuel_bowser`: self-bunded skid-mounted fuel tank (~25 kL) with pump cabinet, meter,
  `fuel_hose_reel_spin`, nozzle holster, side ladder, top guardrail, vents and manhole
"""
import math
import random

from mining_parts import (MINING_MATERIALS, Model, box, box_between, box_span, cyl_between,
                          cylinder, extrude, guardrail, hazard_stripes, ladder, lathe, louvres,
                          polyline_tube, save, sphere, tank_cap)
from bodykit import panel, rbox
from meshkit import Geo, add, cross, dot, mul, norm, sub
from site_parts import chunk

LP_X = -7.40     # lighting plant (axle)
BERM_X = 1.00
SIGN_X = 6.20
FUEL_X = 11.20


def simple_wheel(r, w, segs=24):
    """Light trailer wheel (tyre and steel disc) along +Y, outer face toward +Y."""
    hw = w / 2
    b = r * 0.58
    prof = [(b, -hw * 0.8), (r * 0.80, -hw), (r * 0.97, -hw * 0.9), (r, -hw * 0.55), (r, hw * 0.55),
            (r * 0.97, hw * 0.9), (r * 0.80, hw), (b, hw * 0.8), (b, -hw * 0.8)]
    out = [lathe('tyre_rubber', prof, segs)]
    out.append(lathe('steel', [(0, hw * 0.25), (b + 0.005, hw * 0.25), (b + 0.005, hw * 0.75), (b - 0.01, hw * 0.75),
                               (b - 0.01, hw * 0.35), (0, hw * 0.35)], segs))
    out.append(cylinder('chassis_black', b * 0.35, 0.05, 12).at((0, hw * 0.45, 0)))
    return out


def lighting_plant(m):
    root = m.node('lighting_plant')
    x = LP_X
    ax_y = 0.33
    # chassis, drawbar, hitch, jockey wheel
    for s in (1, -1):
        m.add(root, box_span('chassis_black', x - 1.95, x + 1.30, 0.45, 0.58, s * 0.45, s * 0.55))
        m.add(root, box_between('chassis_black', (x + 1.30, 0.515, s * 0.50), (x + 2.55, 0.515, s * 0.05), 0.10, 0.12))
    for dx in (-1.85, -0.30, 1.20):
        m.add(root, box_span('chassis_black', x + dx - 0.05, x + dx + 0.05, 0.47, 0.56, -0.45, 0.45))
    m.add(root, box_span('steel_dark', x + 2.50, x + 2.80, 0.46, 0.57, -0.07, 0.07))
    m.add(root, sphere('steel', 0.05, 10, 5).at((x + 2.74, 0.60, 0)))
    m.add(root, cyl_between('steel_dark', (x + 2.10, 0.20, 0.13), (x + 2.10, 0.95, 0.13), 0.035, 10))
    m.add(root, cyl_between('plastic_black', (x + 2.10, 0.95, 0.13), (x + 2.10, 1.05, 0.13), 0.03, 8))
    jw = m.node('lighting_plant_jockey_wheel_spin', root, (x + 2.10, 0.10, 0.13))
    m.add(jw, [g.rotate('x', 90).at((x + 2.10, 0.10, 0.13)) for g in simple_wheel(0.10, 0.06, 12)])
    # axle, wheels, mudguards
    m.add(root, cyl_between('chassis_black', (x - 0.30, ax_y, -0.78), (x - 0.30, ax_y, 0.78), 0.035, 10))
    for s, side in ((1, 'right'), (-1, 'left')):
        n = m.node('lighting_plant_wheel_%s_spin' % side, root, (x - 0.30, ax_y, s * 0.80))
        m.add(n, [g.rotate('x', 90 if s > 0 else -90).at((x - 0.30, ax_y, s * 0.80)) for g in simple_wheel(0.33, 0.185)])
        arc = [(x - 0.30 + 0.43 * math.cos(math.radians(a)), ax_y + 0.43 * math.sin(math.radians(a))) for a in range(15, 166, 15)]
        arc += [(px, py - 0.03) for px, py in reversed(arc)]
        m.add(root, extrude('chassis_black', arc, 0.26).move(z=s * 0.80))
        m.add(root, box('lamp_red', 0.02, 0.06, 0.10, (x - 1.96, 0.55, s * 0.42)))
        m.add(root, box('lamp_amber', 0.02, 0.05, 0.05, (x + 1.25, 0.55, s * 0.58)))
    # generator canopy with louvres, door seams, control panel, exhaust, fuel cap
    m.add(root, box_span('paint_yellow', x - 1.45, x + 0.95, 0.58, 1.68, -0.62, 0.62))
    m.add(root, box_span('paint_yellow', x - 1.48, x + 0.98, 1.68, 1.73, -0.65, 0.65))
    for s in (1, -1):
        m.add(root, louvres(x + 0.25, x + 0.80, 0.85, 1.45, s * 0.62, s, pitch=0.06))
        for dx in (-1.10, -0.20):
            m.add(root, box_span('chassis_black', x + dx, x + dx + 0.012, 0.62, 1.64, s * 0.619, s * 0.625))
        m.add(root, box_span('steel_dark', x - 0.40, x - 0.32, 1.20, 1.26, s * 0.62, s * 0.64))
    m.add(root, box_span('plastic_black', x + 0.95, x + 0.96, 0.85, 1.45, -0.40, 0.40))
    m.add(root, box_span('glass', x + 0.96, x + 0.965, 1.10, 1.38, -0.30, 0.30))
    m.add(root, cyl_between('steel_dark', (x - 0.70, 1.73, -0.35), (x - 0.70, 1.95, -0.35), 0.04, 10))
    m.add(root, tank_cap(x + 0.50, 1.73, 0.40, 0.05))
    # mast: base section behind the canopy, 3 nested stages, lamp head with 4 floods
    mb = (x - 1.72, 0.0)
    m.add(root, box_span('paint_yellow', mb[0] - 0.11, mb[0] + 0.11, 0.58, 2.62, -0.11, 0.11))
    m.add(root, box_span('chassis_black', mb[0] + 0.11, x - 1.45, 1.40, 1.50, -0.08, 0.08))
    m.add(root, box_span('chassis_black', mb[0] - 0.24, mb[0] - 0.11, 1.10, 1.32, -0.08, 0.08))   # winch
    m.add(root, cyl_between('steel', (mb[0] - 0.17, 1.21, -0.10), (mb[0] - 0.17, 1.21, 0.10), 0.05, 10))
    top = 2.62
    parent = root
    for i, (name, hs) in enumerate((('mast_lift', 0.085), ('mast_stage_2_lift', 0.065), ('mast_stage_3_lift', 0.047))):
        y0, y1 = 0.66 + i * 0.04, top + 0.06 * (i + 1)
        n = m.node(name, parent, (mb[0], y1, 0))
        m.add(n, box_span('paint_yellow' if i % 2 else 'aluminum', mb[0] - hs, mb[0] + hs, y0, y1, -hs, hs))
        m.add(n, box_span('chassis_black', mb[0] - hs - 0.008, mb[0] + hs + 0.008, y1 - 0.05, y1 - 0.006, -hs - 0.008, hs + 0.008))
        parent = n
    head = m.node('lamp_head_rotate', parent, (mb[0], top + 0.20, 0))
    hy = top + 0.30
    m.add(head, box_span('steel_dark', mb[0] - 0.04, mb[0] + 0.04, top + 0.18, hy - 0.04, -0.04, 0.04))
    m.add(head, box_span('steel_dark', mb[0] - 0.04, mb[0] + 0.04, hy - 0.04, hy + 0.02, -0.62, 0.62))
    for z in (-0.46, -0.16, 0.16, 0.46):
        m.add(head, box_span('plastic_black', mb[0] + 0.02, mb[0] + 0.12, hy - 0.16, hy + 0.20, z - 0.13, z + 0.13))
        m.add(head, box_span('lamp_lens', mb[0] + 0.12, mb[0] + 0.125, hy - 0.13, hy + 0.17, z - 0.11, z + 0.11))
    # outriggers: fixed housings across the frame, sliding beams with jack legs
    for xo, tag in ((x + 1.05, 'front'), (x - 1.62, 'rear')):
        m.add(root, box_span('chassis_black', xo - 0.05, xo + 0.05, 0.40, 0.48, -0.56, 0.56))
        for s, side in ((1, 'right'), (-1, 'left')):
            z1 = s * 0.66
            n = m.node('outrigger_%s_%s_slide' % (tag, side), root, (xo, 0.44, z1))
            m.add(n, box_span('steel_dark', xo - 0.035, xo + 0.035, 0.405, 0.475, s * 0.10, z1))
            m.add(n, box_span('steel_dark', xo - 0.045, xo + 0.045, 0.16, 0.60, z1 - 0.045, z1 + 0.045))
            m.add(n, cyl_between('steel', (xo, 0.60, z1), (xo, 0.70, z1), 0.012, 6))
            m.add(n, box_span('steel_dark', xo - 0.10, xo + 0.10, 0.145, 0.16, z1 - 0.10, z1 + 0.10))


def safety_berm(m):
    n = m.node('safety_berm_section')
    rng = random.Random(7)
    prof = [(2.85, 0.0), (2.20, 0.55), (1.50, 1.13), (0.90, 1.62), (0.40, 1.93), (0.0, 2.0),
            (-0.40, 1.93), (-0.90, 1.62), (-1.50, 1.13), (-2.20, 0.55), (-2.85, 0.0)]
    N = 10
    grid = []
    for i in range(N + 1):
        x = BERM_X - 3.0 + 6.0 * i / N
        end = i in (0, N)
        row = []
        for k, (z, y) in enumerate(prof):
            dz = dy = 0.0
            if not end and 0 < k < len(prof) - 1:   # flat ends so sections butt together
                dy = rng.uniform(-0.07, 0.07)
                dz = rng.uniform(-0.08, 0.08)
            row.append((x, y + dy, z + dz))
        grid.append(row)
    # surface with smooth normals, every face oriented away from the berm's spine
    pos = [p for row in grid for p in row]
    K = len(prof)
    acc = [(0.0, 0.0, 0.0)] * len(pos)
    tris = []
    for i in range(N):
        for k in range(K - 1):
            a, b, c, d = i * K + k, (i + 1) * K + k, (i + 1) * K + k + 1, i * K + k + 1
            for t in ((a, b, c), (a, c, d)):
                p0, p1, p2 = (pos[j] for j in t)
                fn = cross(sub(p1, p0), sub(p2, p0))
                cen = ((p0[0] + p1[0] + p2[0]) / 3, -1.0, 0.0)
                if dot(fn, sub(p0, cen)) < 0:
                    t = (t[0], t[2], t[1])
                    fn = mul(fn, -1)
                tris.append(t)
                for j in t:
                    acc[j] = add(acc[j], fn)
    m.add(n, Geo('dirt', pos, [norm(v) for v in acc], tris))
    for i, sgn in ((0, -1), (N, 1)):
        x = grid[i][0][0]
        m.add(n, panel('dirt', [(z, y) for _, y, z in grid[i]], lambda u, v, x=x: (x, v, u), (sgn, 0, 0)))
    for k in range(9):
        side = 1 if k % 2 else -1
        t = rng.uniform(0.15, 0.85)
        x = BERM_X - 3.0 + 6.0 * (k + 0.5) / 9
        z = side * (2.85 - 2.0 * t)
        y = 2.0 * t * 0.92 - 0.05
        sz = rng.uniform(0.45, 0.80)
        m.add(n, chunk('gravel', (x, max(y, sz * 0.3), z), (sz, sz * 0.8, sz * 0.9), rng))


def haul_road_sign(m):
    n = m.node('haul_road_sign')
    x = SIGN_X
    for z in (-0.80, 0.80):
        m.add(n, cylinder('concrete', 0.18, 0.10, 16).at((x - 0.15, 0.05, z)))
        m.add(n, cyl_between('steel', (x - 0.15, 0.09, z), (x - 0.15, 3.05, z), 0.05, 12))
        m.add(n, cylinder('steel', 0.055, 0.02, 12).at((x - 0.15, 3.06, z)))
    for y in (1.85, 2.75):
        m.add(n, box_span('steel', x - 0.10, x - 0.06, y - 0.04, y + 0.04, -1.10, 1.10))
    m.add(n, box_span('aluminum', x - 0.06, x - 0.02, 1.62, 3.02, -1.25, 1.25))
    m.add(n, box_span('plastic_black', x - 0.02, x - 0.012, 1.62, 3.02, -1.25, 1.25))
    m.add(n, box_span('paint_white', x - 0.012, x - 0.006, 1.68, 2.96, -1.19, 1.19))   # blank face
    for z in (-0.80, 0.80):
        for y in (1.85, 2.75):
            m.add(n, cylinder('steel_dark', 0.016, 0.01, 6).rotate('z', 90).at((x - 0.001, y, z)))


def fuel_bowser(m):
    n = m.node('fuel_bowser')
    x = FUEL_X
    # skids with chamfered ends, cross members, fork pockets
    for s in (1, -1):
        skid = [(-3.25, 0.06), (-3.10, 0.0), (3.35, 0.0), (3.50, 0.06), (3.50, 0.24), (-3.25, 0.24)]
        m.add(n, extrude('chassis_black', [(px + x, py) for px, py in skid], 0.20).move(z=s * 0.95))
        for dx in (-0.6, 0.6):
            m.add(n, box_span('steel_dark', x + dx - 0.16, x + dx + 0.16, 0.05, 0.20, s * 0.845, s * 1.055))
    for dx in (-3.0, -1.5, 0.0, 1.5, 3.0):
        m.add(n, box_span('chassis_black', x + dx - 0.06, x + dx + 0.06, 0.10, 0.24, -0.85, 0.85))
    # self-bunded tank, lifting lugs, manhole, vents, level gauge, fill point
    m.add(n, rbox('paint_white', 5.20, 2.30, 2.40, 0.10, (x - 0.55, 1.39, 0), 3))
    for dx in (-3.0, 1.9):
        for s in (1, -1):
            m.add(n, box_span('steel_dark', x + dx - 0.06, x + dx + 0.06, 2.50, 2.66, s * 1.02, s * 1.04))
    m.add(n, cylinder('paint_white', 0.32, 0.10, 20).at((x - 1.8, 2.59, 0)))
    m.add(n, cylinder('steel_dark', 0.27, 0.04, 20).at((x - 1.8, 2.66, 0)))
    for dz in (-0.6, 0.6):
        m.add(n, cyl_between('steel', (x - 0.4, 2.54, dz), (x - 0.4, 2.85, dz), 0.04, 10))
        m.add(n, cylinder('steel_dark', 0.08, 0.06, 12).at((x - 0.4, 2.88, dz)))
    m.add(n, box_span('steel_dark', x - 2.6, x - 2.4, 0.80, 2.20, 1.20, 1.23))
    m.add(n, box_span('lamp_amber', x - 2.56, x - 2.44, 1.70, 1.76, 1.23, 1.235))
    m.add(n, cyl_between('steel', (x - 3.15, 0.70, 0.5), (x - 3.40, 0.70, 0.5), 0.06, 12))
    m.add(n, cyl_between('steel_dark', (x - 3.40, 0.70, 0.5), (x - 3.46, 0.70, 0.5), 0.09, 12))
    # side ladder and top guardrail
    m.add(n, ladder(x - 2.80, 0.25, 2.58, -1.22, 0.50, axis='x'))
    m.add(n, guardrail([(x - 3.00, -0.98), (x - 3.00, 0.98), (x - 1.0, 0.98), (x - 1.0, -0.98), (x - 2.55, -0.98)], 2.54, h=1.0, kick=0.0, post_gap=2.1))
    # pump cabinet with meter, roller door, hose reel and nozzle holster
    c0, c1 = x + 2.05, x + 3.30
    m.add(n, box_span('paint_white', c0, c1, 0.24, 2.10, -1.10, 1.10))
    m.add(n, box_span('paint_white', c0 - 0.02, c1 + 0.04, 2.10, 2.16, -1.13, 1.13))
    for s in (1, -1):
        m.add(n, box_span('steel_dark', c0 + 0.10, c1 - 0.10, 0.35, 1.95, s * 1.10, s * 1.104))
        for k in range(10):
            y = 0.45 + k * 0.15
            m.add(n, box_span('steel', c0 + 0.12, c1 - 0.12, y, y + 0.012, s * 1.104, s * 1.108))
    m.add(n, box_span('plastic_black', c1, c1 + 0.08, 1.45, 1.85, -0.35, 0.05))
    m.add(n, box_span('glass', c1 + 0.08, c1 + 0.085, 1.55, 1.78, -0.30, 0.0))
    rx, ry = c1 + 0.30, 1.00
    for s in (1, -1):
        m.add(n, box_span('steel_dark', c1, rx + 0.08, 0.45, 0.55, s * 0.40, s * 0.44))
        m.add(n, box_between('steel_dark', (rx, 0.50, s * 0.42), (rx, ry, s * 0.42), 0.04, 0.06, (1, 0, 0)))
    reel = m.node('fuel_hose_reel_spin', n, (rx, ry, 0))
    m.add(reel, cylinder('paint_red', 0.40, 0.02, 24).rotate('x', 90).at((rx, ry, 0.36)))
    m.add(reel, cylinder('paint_red', 0.40, 0.02, 24).rotate('x', 90).at((rx, ry, -0.36)))
    m.add(reel, cylinder('hose_rubber', 0.30, 0.70, 24).rotate('x', 90).at((rx, ry, 0)))
    m.add(reel, cyl_between('steel', (rx, ry, -0.42), (rx, ry, 0.42), 0.04, 10))
    m.add(n, polyline_tube('hose_rubber', [(rx + 0.25, ry - 0.20, 0.25), (rx + 0.20, 0.60, 0.70), (c1 - 0.10, 0.80, 1.06),
                                           (c1 - 0.40, 1.10, 1.14)], 0.022, 8))
    m.add(n, box_span('steel_dark', c1 - 0.55, c1 - 0.30, 1.07, 1.32, 1.10, 1.16))
    m.add(n, box_between('plastic_black', (c1 - 0.42, 1.12, 1.17), (c1 - 0.42, 1.45, 1.17), 0.05, 0.06, (1, 0, 0)))
    m.add(n, cyl_between('aluminum', (c1 - 0.42, 1.30, 1.17), (c1 - 0.22, 1.18, 1.17), 0.018, 8))
    # hazard stripes on the skid ends
    for s in (1, -1):
        m.add(n, hazard_stripes((x + 3.50, 0.07, s * 1.05 if s > 0 else -0.85), (0, 0, -1), (0, 1, 0), 0.20, 0.15, 0.08))


def build():
    m = Model('Mine Site Props', MINING_MATERIALS)
    lighting_plant(m)
    safety_berm(m)
    haul_road_sign(m)
    fuel_bowser(m)
    return m


if __name__ == '__main__':
    save(build(), 'mine-site-props')
