"""Assemblies shared by both dozers: blades, cabs, lamps, louvres, rails."""
import math

from meshkit import (box, box_span, cyl_between, cylinder, extrude, polyline_tube,
                     sphere, Geo)


def blade(xb, H, wc, ww=0.0, wing_deg=0.0, curve=0.28, lean=0.12, thick=0.045,
          back=0.45, ribs=5, mat='paint'):
    """Dozer blade with a curved moldboard, cutting edge, end bits and box-section back.

    xb: x of the cutting edge tip at ground level; H: blade height;
    wc: width of the straight centre section; ww/wing_deg: optional angled
    wings (semi-U blade). Returns a list of Geo.
    """
    N = 20
    front = lambda t: (xb - curve * math.sin(math.pi * t) + lean * t, H * t)
    ts = [i / N for i in range(N + 1)]

    def band(t0, t1, fwd=0.0, aft=thick):
        """Polygon between the face shifted forward by `fwd` and back by `aft`."""
        seg = [t0 + (t1 - t0) * i / 10 for i in range(11)]
        a = [(front(t)[0] + fwd, front(t)[1]) for t in seg]
        b = [(front(t)[0] - aft, front(t)[1]) for t in reversed(seg)]
        return a + b

    def rib(t0, t1):
        seg = [t0 + (t1 - t0) * i / 10 for i in range(11)]
        a = [(front(t)[0] - thick + 0.002, front(t)[1]) for t in seg]
        return a + [(xb - back, front(t1)[1]), (xb - back, front(t0)[1])]

    moldboard = [(front(t)[0], front(t)[1]) for t in ts] + \
                [(front(t)[0] - thick, front(t)[1]) for t in reversed(ts)]
    edge_poly = band(0.0, 0.19, fwd=0.03, aft=0.0)
    endbit_poly = band(0.0, 0.16, fwd=0.04, aft=0.0)
    beam_lo, beam_hi = rib(0.04, 0.30), rib(0.76, 0.95)
    lip_poly = band(0.955, 1.0, fwd=0.06, aft=thick + 0.04)

    def section(width):
        """Moldboard section of `width`, spanning local z in [0, width]."""
        g = [extrude(mat, moldboard, width),
             extrude('edge', edge_poly, width),
             extrude(mat, beam_lo, width),
             extrude(mat, beam_hi, width),
             extrude(mat, lip_poly, width)]
        # cutting-edge bolts
        n = max(1, int(width / 0.22))
        for i in range(n):
            z = -width / 2 + (i + 0.5) * width / n
            x, y = front(0.1)
            g.append(cylinder('steel_dark', 0.016, 0.03, 6).rotate('z', 90 + 18).at((x + 0.03, y, z)))
        return [p.move(z=width / 2) for p in g]

    out = []
    # centre section and its ribs
    out += [p.move(z=-wc / 2) for p in section(wc)]
    for i in range(ribs):
        z = -wc / 2 + 0.06 + (wc - 0.12) * i / (ribs - 1)
        out.append(extrude(mat, rib(0.03, 0.97), 0.035).move(z=z))
    # push-plate behind the lower centre (where push arms and braces land)
    out.append(box_span(mat, xb - back - 0.02, xb - back + 0.02, H * 0.05, H * 0.85, -wc / 2 + 0.05, wc / 2 - 0.05))

    endplate = band(0.0, 1.0, fwd=0.10, aft=thick)
    endbit = lambda w: extrude('edge', endbit_poly, w)
    if ww > 0:
        k = math.tan(math.radians(wing_deg))
        shear = ((1, 0, k), (0, 1, 0), (0, 0, 1))
        wing = section(ww)
        wing.append(extrude(mat, rib(0.03, 0.97), 0.035).move(z=ww - 0.04))
        wing.append(extrude(mat, endplate, 0.03).move(z=ww + 0.015))
        wing.append(endbit(0.32).move(z=ww - 0.16))
        wing = [g.transform(shear).move(z=wc / 2) for g in wing]
        out += wing + [g.mirror_z() for g in wing]
    else:
        tip = [extrude(mat, endplate, 0.03).move(z=wc / 2 + 0.015),
               endbit(0.32).move(z=wc / 2 - 0.16)]
        out += tip + [g.mirror_z() for g in tip]
    return out


def lamp(center, facing=1, size=1.0):
    """Work lamp: housing plus emissive lens facing +x (facing=1) or -x."""
    x, y, z = center
    s = size
    return [box('paint_dark', 0.12 * s, 0.12 * s, 0.16 * s, center),
            box('lens', 0.01, 0.09 * s, 0.13 * s, (x + facing * 0.062 * s, y, z)),
            box('paint_dark', 0.03, 0.06 * s, 0.03, (x - facing * 0.07 * s, y - 0.05 * s, z))]


def louvres(x0, x1, y0, y1, z, side=1, pitch=0.055, mat='grille'):
    """Rows of angled slats standing proud of a side panel at `z`."""
    out = [box_span('paint_dark', x0 - 0.02, x1 + 0.02, y0 - 0.02, y1 + 0.02,
                    z - side * 0.002, z + side * 0.004)]
    y = y0 + pitch / 2
    while y < y1:
        g = box(mat, x1 - x0, 0.012, 0.035).rotate('x', side * -35).at((
            (x0 + x1) / 2, y, z + side * 0.016))
        out.append(g)
        y += pitch
    return out


def cab(x0, x1, y0, y1, hw, sill=0.40, mat='paint', door_side=1):
    """ROPS operator cab. x0 rear, x1 front, floor y0, roof underside y1, half-width hw."""
    out = []
    p = 0.09  # ROPS post size
    out.append(box_span('paint_dark', x0, x1, y0, y0 + 0.06, -hw, hw))  # floor plate
    ys = y0 + sill
    # lower body panels
    out.append(box_span(mat, x0, x0 + 0.03, y0, ys, -hw, hw))
    out.append(box_span(mat, x1 - 0.03, x1, y0, ys, -hw, hw))
    for s in (1, -1):
        out.append(box_span(mat, x0, x1, y0, ys, s * (hw - 0.03), s * hw))
    # posts and roof frame
    for px in (x0 + p / 2, x1 - p / 2):
        for s in (1, -1):
            out.append(box(mat, p, y1 - y0, p, (px, (y0 + y1) / 2, s * (hw - p / 2))))
    for s in (1, -1):
        out.append(box_span(mat, x0, x1, y1 - 0.08, y1, s * (hw - p), s * hw))
        # mid door pillar
        out.append(box(mat, 0.06, y1 - ys, 0.06, ((x0 + x1) / 2 - 0.1, (ys + y1) / 2, s * (hw - 0.03))))
    out.append(box_span(mat, x0, x0 + p, y1 - 0.08, y1, -hw, hw))
    out.append(box_span(mat, x1 - p, x1, y1 - 0.08, y1, -hw, hw))
    # roof with overhang and drip rail
    out.append(box_span(mat, x0 - 0.08, x1 + 0.10, y1, y1 + 0.10, -hw - 0.07, hw + 0.07))
    out.append(box_span('paint_dark', x0 - 0.05, x1 + 0.05, y1 + 0.10, y1 + 0.16, -hw + 0.05, hw - 0.05))
    # glazing
    g0, g1 = ys, y1 - 0.08
    out.append(box_span('glass', x1 - 0.02, x1 - 0.01, g0, g1, -hw + p, hw - p))
    out.append(box_span('glass', x0 + 0.01, x0 + 0.02, g0, g1, -hw + p, hw - p))
    for s in (1, -1):
        out.append(box_span('glass', x0 + p, x1 - p, g0, g1, s * (hw - 0.02), s * (hw - 0.01)))
    # door seal outline and handle on door side
    dx0, dx1 = (x0 + x1) / 2 - 0.07, x1 - p
    zd = door_side * (hw + 0.004)
    for a, b in (((dx0, y0 + 0.05), (dx0, g1)), ((dx1, y0 + 0.05), (dx1, g1))):
        out.append(box_span('rubber', a[0] - 0.012, a[0] + 0.012, a[1], b[1], zd - 0.006, zd + 0.006))
    out.append(box_span('steel_dark', dx0 + 0.04, dx0 + 0.16, ys - 0.12, ys - 0.09, zd, zd + door_side * 0.03))
    # wipers
    out.append(box_between_wiper(x1, g0, hw))
    out.append(box_span('rubber', x0 - 0.002, x0 + 0.02, g0 + 0.05, g0 + 0.07, -0.25, 0.25))
    # grab handles beside door
    zz = door_side * (hw + 0.06)
    for px in (x1 + 0.02, dx0 - 0.08):
        out += polyline_tube('chrome', [(px, y0 + 0.25, door_side * hw), (px, y0 + 0.25, zz),
                                        (px, y1 - 0.25, zz), (px, y1 - 0.25, door_side * hw)], 0.016)
    # mirrors
    for s in (1, -1):
        a = (x1 - 0.05, y1 - 0.15, s * hw)
        b = (x1 + 0.10, y1 - 0.10, s * (hw + 0.28))
        out.append(cyl_between('steel_dark', a, b, 0.012, 8))
        out.append(box('paint_dark', 0.04, 0.26, 0.17, (b[0], b[1] - 0.12, b[2])))
        out.append(box('glass', 0.01, 0.23, 0.14, (b[0] - 0.022, b[1] - 0.12, b[2])))
    # lights and beacon
    for s in (1, -1):
        out += lamp((x1 + 0.08, y1 + 0.2, s * (hw - 0.15)), 1)
        out += lamp((x0 - 0.06, y1 + 0.2, s * (hw - 0.15)), -1)
    out.append(cylinder('paint_dark', 0.07, 0.04, 16).at((x0 + 0.25, y1 + 0.18, -hw + 0.25)))
    out.append(cylinder('beacon', 0.06, 0.12, 16, r_top=0.045).at((x0 + 0.25, y1 + 0.26, -hw + 0.25)))
    out.append(cyl_between('steel_dark', (x0 + 0.25, y1 + 0.16, hw - 0.3), (x0 + 0.25, y1 + 0.55, hw - 0.3), 0.006, 6))
    # interior: seat, consoles, controls
    sx = x0 + (x1 - x0) * 0.42
    out.append(box_span('paint_dark', sx - 0.25, sx + 0.25, y0 + 0.06, y0 + 0.38, -0.22, 0.22))
    out.append(box_span('seat', sx - 0.25, sx + 0.25, y0 + 0.38, y0 + 0.50, -0.26, 0.26))
    out.append(box('seat', 0.10, 0.62, 0.50, (sx - 0.26, y0 + 0.82, 0)).rotate('z', 10, (sx - 0.26, y0 + 0.5, 0)))
    out.append(box('seat', 0.08, 0.16, 0.30, (sx - 0.32, y0 + 1.22, 0)))
    for s in (1, -1):
        out.append(box_span('paint_dark', sx - 0.30, sx + 0.35, y0 + 0.06, y0 + 0.62, s * 0.30, s * 0.46))
        out.append(box_span('seat', sx - 0.2, sx + 0.25, y0 + 0.62, y0 + 0.67, s * 0.32, s * 0.44))
        top = (sx + 0.22, y0 + 0.67, s * 0.38)
        out.append(cyl_between('steel_dark', top, (top[0] + 0.02, top[1] + 0.14, top[2]), 0.012, 8))
        out.append(sphere('seat', 0.035, 10, 6).at((top[0] + 0.02, top[1] + 0.17, top[2])))
    out.append(box_span('paint_dark', x1 - 0.30, x1 - 0.03, y0 + 0.06, ys + 0.05, -0.3, 0.3))  # dash
    out.append(box('grille', 0.02, 0.16, 0.24, (x1 - 0.32, ys + 0.06, 0)).rotate('z', 25, (x1 - 0.32, ys, 0)))
    for s in (1, -1):  # foot pedals
        out.append(box('paint_dark', 0.06, 0.18, 0.08, (x1 - 0.42, y0 + 0.15, s * 0.12)).rotate('z', -30, (x1 - 0.42, y0 + 0.06, 0)))
    return out


def box_between_wiper(x1, g0, hw):
    return box('rubber', 0.015, 0.6, 0.025, (x1 + 0.002, g0 + 0.32, 0)).rotate('x', 25, (x1, g0 + 0.05, 0))


def handrail(points, r=0.019, post_to=None, mat='paint'):
    """Rail through `points`; optionally drop a post from each point down to y=post_to."""
    out = polyline_tube(mat, points, r)
    if post_to is not None:
        for x, y, z in points:
            out.append(cyl_between(mat, (x, post_to, z), (x, y, z), r, 10))
    return out


def exhaust(x, y0, y1, z, r=0.065):
    return [cylinder('paint_dark', r * 1.6, 0.05, 16).at((x, y0 + 0.025, z)),
            cyl_between('grille', (x, y0, z), (x, y1, z), r, 18),
            cylinder('steel_dark', r * 1.15, 0.12, 18).at((x, y1 - 0.15, z)),
            # angled rain-cap cut
            cyl_between('grille', (x, y1, z), (x - 0.04, y1 + 0.06, z), r, 18)]


def precleaner(x, y0, z, h=0.26):
    return [cyl_between('paint_dark', (x, y0, z), (x, y0 + h, z), 0.045, 14),
            cylinder('rubber', 0.13, 0.22, 20).at((x, y0 + h + 0.11, z)),
            cylinder('glass', 0.125, 0.14, 20).at((x, y0 + h + 0.07, z)),
            cylinder('paint_dark', 0.14, 0.05, 20, r_top=0.08).at((x, y0 + h + 0.245, z))]
