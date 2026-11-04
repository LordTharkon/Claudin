"""Farm props in one file, each a separate named node laid out in a row along X:
round hay bale stack, small square bale stack, single round bale, corrugated grain
bin (6 m dia) with ladders and a roof hatch, and a field gate on timber posts.

Key real-world dimensions used (metres):
  round bale 1.25 dia x 1.20 wide; small square bale 0.90 x 0.46 x 0.36
  grain bin 6.0 dia, 6.0 to the eave, 30 deg roof, ~170 m3
  field gate 3.6 x 1.2 galvanised tube, hung on 0.2 m timber posts
"""
import math

from farm_parts import FARM_MATERIALS, FRAME, ladder, save_farm
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, extrude, lathe, sphere
from bodykit import rbox

GALV = 'steel_galvanized'
BALE_R, BALE_W = 0.625, 1.20


def round_bale(center, axis='z'):
    """Net-wrapped round bale lying on its side, axis along z (or x)."""
    r, h = BALE_R, BALE_W / 2
    prof = [(0, -h + 0.02), (0.18, -h + 0.01), (0.34, -h + 0.015), (0.50, -h), (r - 0.05, -h),
            (r - 0.01, -h + 0.04), (r, -h + 0.10), (r, h - 0.10), (r - 0.01, h - 0.04), (r - 0.05, h),
            (0.50, h), (0.34, h - 0.015), (0.18, h - 0.01), (0, h - 0.02)]
    g = lathe('hay', prof, 28).scale(1.0, 1.0, 1.0)
    g = g.rotate('x', 90) if axis == 'z' else g.rotate('z', 90)
    return [g.scale(1.0, 0.97, 1.0).at(center)]


def round_bale_stack(m, x0):
    n = m.node('round_bale_stack')
    d = 2 * BALE_R
    for i in range(3):
        m.add(n, round_bale((x0 + d * i, BALE_R * 0.97, 0)))
    for i in range(2):
        m.add(n, round_bale((x0 + d * (i + 0.5), BALE_R * 0.97 + d * math.sin(math.radians(60)) * 0.97, 0)))
    return x0 + 2 * d


def square_bale(c, along='x'):
    sx, sz = (0.90, 0.46) if along == 'x' else (0.46, 0.90)
    out = [rbox('hay', sx, 0.36, sz, 0.03, c, n=1)]
    for f in (-0.25, 0.25):
        if along == 'x':
            out.append(box('paint_orange', 0.012, 0.366, 0.466, (c[0] + f * sx, c[1], c[2])))
        else:
            out.append(box('paint_orange', 0.466, 0.366, 0.012, (c[0], c[1], c[2] + f * sz)))
    return out


def square_bale_stack(m, x0):
    n = m.node('small_square_bale_stack')
    for layer in range(4):
        y = 0.18 + 0.36 * layer
        if layer % 2 == 0:
            for i in range(2):
                for j in range(4):
                    m.add(n, square_bale((x0 + 0.45 + 0.90 * i, y, -0.69 + 0.46 * j), 'x'))
        else:
            for i in range(4):
                for j in range(2):
                    m.add(n, square_bale((x0 + 0.23 + 0.46 * i, y, -0.45 + 0.90 * j), 'z'))
    # a few loose bales on top
    for i in range(3):
        m.add(n, square_bale((x0 + 0.45 + 0.46 * i, 0.18 + 0.36 * 4, 0.10), 'z'))
    return x0 + 1.84


def grain_bin(m, xc):
    n = m.node('grain_bin')
    R, YE, Y0 = 3.0, 6.0, 0.25
    slope = math.tan(math.radians(30))
    # concrete pad, corrugated wall, stiffeners, eave
    m.add(n, cylinder('concrete', R + 0.35, Y0, 32).at((xc, Y0 / 2, 0)))
    prof = []
    waves = 23
    for i in range(waves):
        y = Y0 + (YE - Y0) * i / waves
        dy = (YE - Y0) / waves
        prof += [(R, y), (R + 0.025, y + dy * 0.5)]
    prof.append((R, YE))
    m.add(n, lathe(GALV, prof, 32).move(x=xc))
    for k in range(16):
        a = 2 * math.pi * (k + 0.5) / 16
        c, s = math.cos(a), math.sin(a)
        m.add(n, box(GALV, 0.04, YE - Y0, 0.09, (0, (YE + Y0) / 2, 0)).rotate('y', -math.degrees(a)).move(x=xc + (R + 0.045) * c, z=(R + 0.045) * s))
    # roof cone with ribs, peak collar and vent
    ye2 = YE + (R + 0.15 - 0.40) * slope
    m.add(n, lathe(GALV, [(0.40, ye2 - 0.02), (R + 0.15, YE - 0.05), (R + 0.15, YE + 0.01), (0.40, ye2 + 0.03)], 32).move(x=xc))
    roof = lambda r: YE + 0.01 + (R + 0.15 - r) * slope        # top of the roof sheet at radius r
    for k in range(16):
        a = 2 * math.pi * k / 16
        c, s = math.cos(a), math.sin(a)
        p0 = (xc + (R + 0.10) * c, roof(R + 0.10) + 0.025, (R + 0.10) * s)
        p1 = (xc + 0.45 * c, roof(0.45) + 0.025, 0.45 * s)
        m.add(n, box_between(GALV, p0, p1, 0.05, 0.05))
    m.add(n, cylinder(GALV, 0.42, 0.30, 24).at((xc, ye2 + 0.15, 0)))
    m.add(n, lathe(GALV, [(0, ye2 + 0.30), (0.55, ye2 + 0.30), (0.55, ye2 + 0.33), (0, ye2 + 0.55)], 24).move(x=xc))
    m.add(n, cylinder(GALV, 0.30, 0.12, 24).at((xc, ye2 + 0.36, 0)))
    # outside wall ladder (with a few cage hoops) and roof ladder to the peak
    phi = math.radians(72)
    c, s = math.cos(phi), math.sin(phi)
    t = (-s, 0, c)
    lr = R + 0.30
    m.add(n, ladder((xc + lr * c, 0.40, lr * s), (xc + lr * c, YE + 0.90, lr * s), 0.46, 20, side_mat=GALV, rung_mat=GALV, axis=t))
    for y in (0.6, YE - 0.4):
        for sg in (-1, 1):
            b = (xc + lr * c + sg * 0.23 * t[0], y, lr * s + sg * 0.23 * t[2])
            m.add(n, cyl_between(GALV, b, (xc + (R + 0.04) * c + sg * 0.23 * t[0], y, (R + 0.04) * s + sg * 0.23 * t[2]), 0.015, 8))
    for k in range(5):
        y = 2.4 + k * 0.85
        pts = []
        for j in range(9):
            a = math.pi * j / 8
            off = 0.36 * math.cos(a)
            out = 0.40 * math.sin(a)
            pts.append((xc + (lr + out) * c + off * t[0], y, (lr + out) * s + off * t[2]))
        for a_, b_ in zip(pts, pts[1:]):
            m.add(n, cyl_between(GALV, a_, b_, 0.014, 6, caps=False))
    for sg in (-1, 1):
        a_ = (xc + (R + 0.05) * c + sg * 0.23 * t[0], roof(R + 0.05) + 0.11, (R + 0.05) * s + sg * 0.23 * t[2])
        b_ = (xc + 0.55 * c + sg * 0.23 * t[0], roof(0.55) + 0.11, 0.55 * s + sg * 0.23 * t[2])
        m.add(n, cyl_between(GALV, a_, b_, 0.018, 8))
        m.add(n, cyl_between(GALV, (a_[0], YE + 0.90, a_[2]), a_, 0.018, 8))
        m.add(n, cyl_between(GALV, (xc + lr * c + sg * 0.23 * t[0], YE + 0.90, lr * s + sg * 0.23 * t[2]),
                             (a_[0], YE + 0.90, a_[2]), 0.018, 8))
        m.add(n, cyl_between(GALV, (b_[0], b_[1] - 0.11, b_[2]), b_, 0.016, 8))
    for k in range(8):
        f = (k + 0.5) / 8
        rr = (R + 0.05) + (0.55 - (R + 0.05)) * f
        yy = roof(rr) + 0.11
        m.add(n, cyl_between(GALV, (xc + rr * c - 0.23 * t[0], yy, rr * s - 0.23 * t[2]),
                             (xc + rr * c + 0.23 * t[0], yy, rr * s + 0.23 * t[2]), 0.014, 8))
    # roof hatch (eave manway) facing +z, hinged on its upper edge
    rh0, rh1 = 2.05, 2.65
    yr = lambda r: YE + 0.03 + (R + 0.15 - r) * slope
    for zz in (rh0, rh1):
        m.add(n, box_between(GALV, (xc - 0.34, yr(zz) + 0.04, zz), (xc + 0.34, yr(zz) + 0.04, zz), 0.04, 0.08))
    for sx in (-0.32, 0.32):
        m.add(n, box_between(GALV, (xc + sx, yr(rh0) + 0.04, rh0), (xc + sx, yr(rh1) + 0.04, rh1), 0.04, 0.08, (0, 1, 0)))
    h = m.node('hatch_rotate', n, (xc, yr(rh0) + 0.10, rh0 - 0.02))
    lid = box(GALV, 0.74, 0.03, (rh1 - rh0) / math.cos(math.radians(30)) + 0.06, (0, 0, 0)).rotate('x', 30)
    m.add(h, lid.at((xc, (yr(rh0) + yr(rh1)) / 2 + 0.10, (rh0 + rh1) / 2)))
    m.add(h, cyl_between('steel_dark', (xc - 0.30, yr(rh0) + 0.10, rh0 - 0.02), (xc + 0.30, yr(rh0) + 0.10, rh0 - 0.02), 0.018, 8))
    m.add(h, box_between('steel_dark', (xc - 0.05, yr(rh1) + 0.13, rh1), (xc + 0.05, yr(rh1) + 0.13, rh1), 0.03, 0.03))
    # side door and aeration fan at the base
    for ang, fn in ((35, 'door'), (-40, 'fan')):
        a = math.radians(ang)
        c2, s2 = math.cos(a), math.sin(a)
        rot = lambda g: g.rotate('y', 90 - ang).move(x=xc + (R + 0.04) * c2, z=(R + 0.04) * s2)
        if fn == 'door':
            m.add(n, rot(box_span(GALV, -0.48, 0.48, Y0 + 0.30, Y0 + 1.45, -0.03, 0.03)))
            m.add(n, rot(box_span('steel_dark', -0.40, 0.40, Y0 + 0.36, Y0 + 1.39, 0.03, 0.045)))
            m.add(n, rot(box_span('steel', 0.24, 0.34, Y0 + 0.85, Y0 + 0.90, 0.045, 0.08)))
        else:
            m.add(n, rot(box_span(GALV, -0.45, 0.45, Y0, Y0 + 0.90, -0.02, 0.35)))
            m.add(n, rot(cyl_between(GALV, (0, Y0 + 0.45, 0.35), (0, Y0 + 0.45, 0.95), 0.40, 24)))
            m.add(n, rot(cyl_between('paint_gray_dark', (0, Y0 + 0.45, 0.95), (0, Y0 + 0.45, 1.25), 0.36, 24)))
            m.add(n, rot(box_span('chassis_black', -0.12, 0.12, Y0 + 0.86, Y0 + 1.10, 1.00, 1.20)))
    return xc + R + 0.4


def field_gate(m, x0):
    n = m.node('field_gate')
    L, H, y0 = 3.60, 1.20, 0.10
    xh, xs = x0 + 0.10, x0 + 0.10 + L + 0.30
    for x in (xh, xs):
        m.add(n, box_span('timber', x - 0.10, x + 0.10, 0.0, 1.45, -0.10, 0.10))
        m.add(n, extrude('timber', [(x - 0.10, 1.45), (x + 0.10, 1.45), (x, 1.52)], 0.20))
    for y in (y0 + 0.20, y0 + 0.95):
        m.add(n, cyl_between('steel', (xh + 0.10, y, 0), (xh + 0.17, y, 0), 0.012, 8))
        m.add(n, cyl_between('steel', (xh + 0.17, y - 0.06, 0), (xh + 0.17, y + 0.06, 0), 0.012, 8))
    m.add(n, box_span(GALV, xs - 0.14, xs - 0.10, y0 + 0.95, y0 + 1.05, -0.05, 0.05))      # latch catch
    m.add(n, box_span(GALV, xs - 0.18, xs - 0.10, y0 + 0.95, y0 + 0.98, -0.05, 0.05))
    g = m.node('gate_rotate', n, (xh + 0.17, y0, 0))
    a, b = xh + 0.20, xh + 0.20 + L
    for y, r in ((y0 + H, 0.024), (y0 + 0.02, 0.022), (y0 + 0.30, 0.016), (y0 + 0.55, 0.016), (y0 + 0.82, 0.016)):
        m.add(g, cyl_between(GALV, (a, y, 0), (b, y, 0), r, 10))
    for x in (a, b):
        m.add(g, cyl_between(GALV, (x, y0, 0), (x, y0 + H + 0.02, 0), 0.026, 10))
    for x in (a + L / 3, a + 2 * L / 3):
        m.add(g, cyl_between(GALV, (x, y0 + 0.02, 0), (x, y0 + H, 0), 0.014, 8))
    m.add(g, cyl_between(GALV, (a, y0 + 0.05, 0), (a + 2 * L / 3, y0 + H - 0.02, 0), 0.016, 8))
    for y in (y0 + 0.20, y0 + 0.95):
        m.add(g, cyl_between('steel', (xh + 0.17, y + 0.06, 0), (a, y + 0.06, 0), 0.016, 8))
        m.add(g, cylinder('steel', 0.03, 0.04, 10).at((xh + 0.17, y + 0.08, 0)))
    m.add(g, box_span(GALV, b - 0.02, b + 0.10, y0 + 0.97, y0 + 1.03, -0.03, 0.03))         # spring latch
    m.add(g, cyl_between('steel', (b - 0.15, y0 + 1.00, 0.0), (b + 0.12, y0 + 1.00, 0.0), 0.010, 6))
    m.add(g, box_span('paint_red', b - 0.22, b - 0.14, y0 + 0.97, y0 + 1.06, -0.03, 0.03))
    return xs + 0.10


def build():
    m = Model('Farm Props', FARM_MATERIALS)
    x = round_bale_stack(m, -12.6)
    gb = grain_bin(m, x + 4.6)
    m.add(m.node('round_bale'), round_bale((gb + 1.4, BALE_R * 0.97, 0.6)))
    x = square_bale_stack(m, gb + 2.8)
    field_gate(m, x + 1.0)
    return m


if __name__ == '__main__':
    save_farm(build(), 'farm-props', 20000)
