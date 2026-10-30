"""Modern full-size pickups from one parametric, smooth-surfaced body.

* Pickup Truck: half-ton crew cab, 5.5 ft bed, 4x4, 275/65R18 all-terrain tyres
  (wheelbase 3.68, length 5.89, width 2.03, height ~1.96)
* Dually Pickup Truck: heavy-duty one-ton crew cab, 8 ft bed, dual rear wheels,
  LT235/80R17 tyres (wheelbase 4.29, length 6.71, width 2.44 over rear fenders, height ~2.03)

The body is lofted from rounded cross-sections (rounded shoulders, cab sides that lean in
toward the roof, tapered nose, crowned hood with power dome, sculpted lower sides and
fender bulges); glass and trim are flush panels on those surfaces.
"""
import math

from meshkit import (MATERIALS, Model, box_between, box_span, cyl_between, cylinder,
                     extrude, lathe, save, sphere)
from bodykit import fillet, loft, panel, rbox, ring_at, section, smin, smoothstep, warp
from vehicle_parts import place_wheel, rim, tyre

LIGHT = dict(name='Pickup Truck', folder='pickup-truck', file='pickup_truck',
             wb=3.68, foh=0.86, roh=1.08, width=2.03, bed=1.68,
             tyre_r=0.41, tyre_w=0.275, bead=0.2286, track=1.73,
             rocker=0.50, belt=1.30, hood=1.20, roof=1.94, hood_len=1.62, rake=0.85,
             paint='car_blue', dually=False)
DUALLY = dict(name='Dually Pickup Truck', folder='dually-pickup-truck', file='dually_pickup_truck',
              wb=4.29, foh=0.92, roh=1.22, width=2.03, rear_width=2.44, bed=2.44,
              tyre_r=0.395, tyre_w=0.245, bead=0.216, track=1.75, dual_c=0.87, dual_gap=0.30,
              rocker=0.56, belt=1.40, hood=1.33, roof=2.02, hood_len=1.70, rake=0.80,
              paint='car_white', dually=True)


def bump(t):
    return math.cos(math.pi * t / 2) ** 2 if abs(t) < 1 else 0.0


class Body:
    """Shared dimensions and surface functions for one truck."""

    def __init__(self, c):
        self.c = c
        self.hw = c['width'] / 2
        self.xF, self.xR = c['wb'] / 2, -c['wb'] / 2
        self.xf, self.xr = self.xF + c['foh'], self.xR - c['roh']
        self.xbf = self.xr + c['bed']
        self.xcb = self.xbf - 0.06
        self.xws = self.xf - c['hood_len']
        self.xwt = self.xws - c['rake']
        self.rk, self.belt, self.hf, self.roof = c['rocker'], c['belt'], c['hood'], c['roof']
        self.wy = c['tyre_r'] - 0.01
        self.Ra = c['tyre_r'] + 0.075
        self.dxA = math.sqrt(max(self.Ra ** 2 - (self.rk - self.wy) ** 2, 0))
        # greenhouse
        self.g_yb = self.belt - 0.04
        self.g_zb = self.hw - 0.06
        self.g_slope = 0.22

    # ---- lower body
    def flare(self, x):
        return 0.022 * (bump((x - self.xF) / 0.8) + bump((x - self.xR) / 0.8))

    def nose_inset(self, x):
        x0 = self.xf - 0.09
        if x <= x0:
            return 0.0
        t = min((x - x0) / 0.09, 1.0)
        return 0.09 * (1 - math.sqrt(max(0.0, 1 - t * t)))

    def half_width(self, x):
        taper = 0.075 * ((x - (self.xf - 0.5)) / 0.5) ** 2 if x > self.xf - 0.5 else 0.0
        return self.hw + self.flare(x) - taper - self.nose_inset(x)

    def bottom(self, x, axle):
        if abs(x - axle) < self.dxA:
            return self.wy + math.sqrt(self.Ra ** 2 - (x - axle) ** 2)
        return self.rk

    def side_fn(self, x, extra=0.0, pod=0.0):
        hwx = self.half_width(x) + extra
        rk, belt = self.rk, self.belt

        def f(y):
            z = hwx
            z -= 0.045 * max(0.0, (rk + 0.30 - y) / 0.30) ** 2              # rocker tuck-under
            z -= 0.014 * max(0.0, min(1.0, (y - (belt - 0.13)) / 0.13))     # shoulder roll-in
            if pod:
                z += pod * (1 - smoothstep((y - (belt - 0.30)) / 0.22))
            return z
        return f

    def lower_section(self, x):
        d = self.nose_inset(x)
        yb = self.bottom(x, self.xF)
        if x > self.xF + self.dxA:
            yb = self.rk + 0.06 * (x - self.xF - self.dxA) / (self.xf - self.xF - self.dxA)
        yb += 0.6 * d
        if x >= self.xws:
            yt = self.hf + (self.belt + 0.015 - self.hf) * (self.xf - x) / (self.xf - self.xws)
            crown = 0.025
            dome = 0.032 * smoothstep((x - self.xws - 0.12) / 0.12) * smoothstep((self.xf - 0.32 - x) / 0.12)
            rs = 0.07
        else:
            yt, crown, dome, rs = self.belt, 0.0, 0.0, 0.035
        yt -= 0.8 * d
        hwx = self.half_width(x)

        def top(z):
            return yt + crown * (1 - (z / hwx) ** 2) + dome * smoothstep((0.50 - abs(z)) / 0.08)
        return section(yb, top, self.side_fn(x), 0.025, rs + d, nt=18)

    # ---- greenhouse
    def g_top(self, x):
        L = self.belt + (self.roof - 0.02 - self.belt) * (self.xws - x) / (self.xws - self.xwt)
        t = smin(L, self.roof, 0.10)
        if x < self.xcb + 0.08:
            t -= 0.035 * ((self.xcb + 0.08 - x) / 0.08) ** 2
        return t

    def g_rs(self, x):
        return min(0.10, 0.45 * (self.g_top(x) - self.g_yb))

    def g_side(self, y):
        return self.g_zb - self.g_slope * (y - self.g_yb)

    def g_crown(self, x):
        return 0.022 * smoothstep((self.xwt - 0.04 - x) / 0.25)

    def g_section(self, x):
        yt, cr = self.g_top(x), self.g_crown(x)
        ze = self.g_side(yt)
        return section(self.g_yb, lambda z: yt + cr * (1 - (z / ze) ** 2), self.g_side, 0.0, self.g_rs(x), nb=2, nc=6, ns=8, nt=12)


def build(c):
    m = Model(c['name'], MATERIALS)
    b = Body(c)
    P = c['paint']
    hw, xF, xR, xf, xr = b.hw, b.xF, b.xR, b.xf, b.xr
    xws, xwt, xcb, xbf = b.xws, b.xwt, b.xcb, b.xbf
    rk, belt, hf, roof, wy, Ra = b.rk, b.belt, b.hf, b.roof, b.wy, b.Ra
    chrome = c['dually']
    trim = 'chrome' if chrome else 'trim_black'

    # ------------------------------------------------------------ lower body (cab + front clip)
    xs = set()
    x = xcb
    while x < xf - 0.09:
        xs.add(round(x, 4))
        x += 0.04
    a0 = math.asin((rk - wy) / Ra)
    for i in range(29):
        a = a0 + (math.pi - 2 * a0) * i / 28
        xs.add(round(xF + Ra * math.cos(a), 5))
    for i in range(11):
        xs.add(round(xf - 0.09 + 0.09 * math.sin(math.pi / 2 * i / 10), 5))
    xs.add(round(xws, 4))
    xs = sorted(v for v in xs if xcb <= v <= xf)
    m.add('body', loft(P, [ring_at(x, b.lower_section(x)) for x in xs], cap0=True, cap1=True))

    # ------------------------------------------------------------ greenhouse
    gx = []
    x = xcb
    while x < xws:
        gx.append(x)
        x += 0.025 if x > xwt - 0.2 else 0.05
    gx.append(xws)
    m.add('body', loft(P, [ring_at(x, b.g_section(x)) for x in gx], cap0=True))

    # windshield
    ws = []
    n = 24
    for i in range(n + 1):
        x = xws - 0.03 + (xwt + 0.05 - (xws - 0.03)) * i / n
        yt = b.g_top(x)
        ws.append((x, b.g_side(yt - b.g_rs(x)) - b.g_rs(x) - 0.035))
    poly = fillet(ws + [(x, -z) for x, z in reversed(ws)], 0.05)
    slope = (b.roof - belt) / (xws - xwt)
    m.add('glass', panel('glass_tint', poly, lambda u, v: (u, b.g_top(u) + 0.006, v), (slope, 1, 0)))

    # side glass split by a black B-pillar
    yw_b = belt + 0.06
    ywt = lambda x: min(b.g_top(x) - b.g_rs(x) - 0.035, roof - 0.13)
    x_front = xws
    while ywt(x_front) - yw_b < 0.06:
        x_front -= 0.005
    xb = (xws + xcb) / 2 + 0.06
    x_end = xcb + 0.16

    def window(x0, x1):  # x0 rear, x1 front
        top = [(x1 + (x0 - x1) * i / 16, ywt(x1 + (x0 - x1) * i / 16)) for i in range(17)]
        return fillet([(x0, yw_b), (x1, yw_b)] + top, 0.05)

    for s in (1, -1):
        on_side = lambda u, v, s=s, o=0.004: (u, v, s * (b.g_side(v) + o))
        nrm = (0, b.g_slope, s)
        m.add('glass', panel('glass_tint', window(xb + 0.05, x_front), on_side, nrm))
        m.add('glass', panel('glass_tint', window(x_end, xb - 0.05), on_side, nrm))
        m.add('body', panel('trim_black', [(xb - 0.05, yw_b - 0.015), (xb + 0.05, yw_b - 0.015),
                                           (xb + 0.05, ywt(xb) + 0.02), (xb - 0.05, ywt(xb) + 0.02)],
                            lambda u, v, s=s: (u, v, s * (b.g_side(v) + 0.003)), nrm))
        # window-line trim
        m.add('body', panel(trim, [(x_end - 0.02, yw_b - 0.025), (x_front + 0.03, yw_b - 0.025),
                                   (x_front + 0.03, yw_b - 0.008), (x_end - 0.02, yw_b - 0.008)],
                            lambda u, v, s=s: (u, v, s * (b.g_side(v) + 0.003)), nrm))
    # rear glass and third brake light
    zr = lambda y: b.g_side(y) - 0.15
    rg = fillet([(-zr(belt + 0.08), belt + 0.08), (zr(belt + 0.08), belt + 0.08),
                 (zr(roof - 0.13), roof - 0.13), (-zr(roof - 0.13), roof - 0.13)], 0.06)
    m.add('glass', panel('glass_tint', rg, lambda u, v: (xcb - 0.004, v, u), (-1, 0, 0)))
    m.add('body', rbox('lens_red', 0.03, 0.035, 0.30, 0.012, (xcb - 0.01, roof - 0.085, 0)))

    # ------------------------------------------------------------ bed walls (lofted U-section)
    rw = c.get('rear_width', c['width']) / 2
    pod_amt = rw - hw if c['dually'] else 0.0
    floor_y = c['tyre_r'] * 2 + 0.03
    z_in = hw - 0.065

    def pod(x):
        if not pod_amt:
            return 0.0
        t = abs(x - xR)
        return pod_amt * (1.0 if t < 0.62 else bump((t - 0.62) / 0.40))

    def bed_ring(x):
        yb = b.bottom(x, xR)
        side = b.side_fn(x, pod=pod(x))
        rr = 0.03
        zo_top = side(belt - rr)
        # inner wall, rounded rail cap, outer side down to the sill / wheel arch
        pts = [(z_in, yb), (z_in, belt - rr)] + [(z_in + rr * (1 - math.cos(a)), belt - rr + rr * math.sin(a))
                                                  for a in (math.pi / 2 * i / 4 for i in range(1, 5))]
        pts += [(zo_top - rr * (1 - math.cos(a)), belt - rr + rr * math.sin(a)) for a in (math.pi / 2 * (4 - i) / 4 for i in range(5))]
        for i in range(1, 15):
            y = belt - rr - (belt - rr - yb - 0.025) * i / 14
            pts.append((side(y), y))
        zb = side(yb + 0.025)
        pts += [(zb - 0.02, yb), ((zb + z_in) / 2, yb)]
        return [(x, y, z) for z, y in pts]

    bxs = set()
    x = xr
    while x < xbf:
        bxs.add(round(x, 4))
        x += 0.04
    for i in range(29):
        a = a0 + (math.pi - 2 * a0) * i / 28
        bxs.add(round(xR + Ra * math.cos(a), 5))
    if pod_amt:
        for t in range(41):
            bxs.add(round(xR - 1.02 + 2.04 * t / 40, 5))
    bxs.add(round(xbf, 4))
    bxs = sorted(v for v in bxs if xr <= v <= xbf)
    right_wall = loft(P, [bed_ring(x) for x in bxs], True, True)
    m.add('bed', right_wall, [g.mirror_z() for g in right_wall])

    # bed floor, front wall, wheelhouses, tailgate
    m.add('bed', box_span('trim_black', xr + 0.06, xbf, floor_y - 0.03, floor_y, -z_in, z_in))
    x = xr + 0.15
    while x < xbf - 0.05:
        m.add('bed', box_span('trim_black', x - 0.03, x + 0.03, floor_y, floor_y + 0.012, -z_in + 0.01, z_in - 0.01))
        x += 0.24
    m.add('bed', rbox(P, 0.06, belt - rk - 0.14, 2 * z_in + 0.02, 0.02, (xbf - 0.03, (belt + rk + 0.14) / 2, 0)))
    r_wh = Ra + 0.03
    top = [(xR + r_wh * math.cos(a), wy + r_wh * math.sin(a))
           for a in (math.radians(t) for t in range(0, 181, 6)) if wy + r_wh * math.sin(a) >= floor_y - 0.02]
    for s in (1, -1):
        m.add('bed', extrude('trim_black', top, 0.25).move(z=s * (z_in - 0.125)))
    m.add('bed', rbox(P, 0.065, belt - rk - 0.13, 2 * z_in - 0.01, 0.02, (xr + 0.033, (belt + rk + 0.13) / 2 - 0.003, 0)))
    m.add('bed', rbox('trim_black', 0.02, 0.05, 0.24, 0.01, (xr - 0.002, belt - 0.08, 0)))
    m.add('bed', panel('trim_black', fillet([(-0.62, rk + 0.24), (0.62, rk + 0.24), (0.62, belt - 0.18), (-0.62, belt - 0.18)], 0.04),
                       lambda u, v: (xr - 0.0015, v, u), (-1, 0, 0)))

    # ------------------------------------------------------------ wheel-arch liners and lips
    for axle, width in ((xF, hw), (xR, rw)):
        arc_o = [(axle + Ra * math.cos(a), wy + Ra * math.sin(a)) for a in (a0 + (math.pi - 2 * a0) * i / 24 for i in range(25))]
        arc_i = [(axle + (Ra + 0.025) * math.cos(a), wy + (Ra + 0.025) * math.sin(a)) for a in (math.pi - a0 - (math.pi - 2 * a0) * i / 24 for i in range(25))]
        m.add('body', extrude('trim_black', arc_o + arc_i, 2 * width - 0.04))
        for s in (1, -1):
            zl = b.half_width(axle) + (pod(axle) if axle == xR else 0) - 0.02
            lip = lathe(P, [(Ra + 0.012 + 0.024 * math.cos(t), 0.024 * math.sin(t))
                                                                    for t in (2 * math.pi * k / 10 for k in range(11))],
                        30, math.degrees(a0), 180 - math.degrees(a0))
            m.add('body', lip.rotate('x', -90).at((axle, wy, s * zl)))

    # ------------------------------------------------------------ door seams, handles, steps, mirrors
    side_lo = b.side_fn
    for s in (1, -1):
        for xs_ in (xws + 0.03, xb, xcb + 0.05):
            f = side_lo(xs_)
            pts = [(xs_ - 0.003, rk + 0.06), (xs_ + 0.003, rk + 0.06), (xs_ + 0.003, belt - 0.02), (xs_ - 0.003, belt - 0.02)]
            m.add('body', panel('trim_black', pts, lambda u, v, f=f, s=s: (u, v, s * (f(v) + 0.0015)), (0, 0, s)))
        for xh in (xb - 0.10, xcb + 0.22):
            zh = side_lo(xh)(belt - 0.10)
            m.add('body', rbox(trim if chrome else P, 0.20, 0.04, 0.04, 0.018, (xh - 0.04, belt - 0.10, s * (zh + 0.012))))
            m.add('body', panel('trim_black', fillet([(xh - 0.15, belt - 0.125), (xh + 0.07, belt - 0.125), (xh + 0.07, belt - 0.075), (xh - 0.15, belt - 0.075)], 0.02),
                                lambda u, v, s=s, zh=zh: (u, v, s * (zh + 0.0015)), (0, 0, s)))
        # running boards
        m.add('body', rbox('trim_black', xws - xcb - 0.10, 0.05, 0.17, 0.02, ((xws + xcb) / 2, rk - 0.08, s * (hw + 0.02))))
        for xm in (xws + 0.3, xcb - 0.3):
            m.add('body', box_span('steel_dark', xm - 0.03, xm + 0.03, rk - 0.08, rk + 0.02, s * (hw - 0.12), s * (hw - 0.05)))
        # mirrors
        if c['dually']:
            arm = [(xws - 0.10, belt + 0.06), (xws - 0.10, belt + 0.32)]
            for ax_, ay_ in arm:
                m.add('body', cyl_between('trim_black', (ax_, ay_, s * (hw - 0.05)), (ax_ - 0.03, ay_ + 0.04, s * (hw + 0.30)), 0.015, 10))
            m.add('body', rbox('trim_black', 0.11, 0.42, 0.25, 0.04, (xws - 0.13, belt + 0.24, s * (hw + 0.36))))
            m.add('body', rbox('glass', 0.012, 0.37, 0.21, 0.004, (xws - 0.19, belt + 0.24, s * (hw + 0.36))))
            m.add('body', rbox('lens_amber', 0.07, 0.015, 0.12, 0.006, (xws - 0.13, belt + 0.455, s * (hw + 0.36))))
        else:
            m.add('body', rbox(P, 0.10, 0.07, 0.10, 0.03, (xws - 0.12, belt + 0.09, s * (hw - 0.01))))
            m.add('body', rbox(P, 0.12, 0.19, 0.25, 0.05, (xws - 0.15, belt + 0.18, s * (hw + 0.12))))
            m.add('body', rbox('glass', 0.012, 0.16, 0.21, 0.006, (xws - 0.212, belt + 0.18, s * (hw + 0.12))))
            m.add('body', rbox('lens_amber', 0.05, 0.012, 0.12, 0.005, (xws - 0.12, belt + 0.085, s * (hw + 0.13))))

    # roof details
    if c['dually']:
        for k in range(5):
            x = xwt - 0.05
            m.add('body', rbox('lens_amber', 0.05, 0.03, 0.09, 0.012, (x, b.g_top(x) + b.g_crown(x) + 0.01, -0.30 + 0.15 * k)))
    else:
        x = xcb + 0.30
        m.add('body', extrude('trim_black', fillet([(x + 0.08, 0), (x - 0.10, 0), (x - 0.06, 0.06), (x - 0.02, 0.065)], 0.01), 0.035)
              .move(y=b.g_top(x) + b.g_crown(x) - 0.002))

    # ------------------------------------------------------------ front fascia
    fz = b.half_width(xf) + b.nose_inset(xf)  # half-width of the nose just before the round-over
    gy0, gy1 = rk + 0.17, hf - 0.11
    if chrome:
        m.add('front', rbox('chrome', 0.07, gy1 - gy0 + 0.06, 1.34, 0.035, (xf - 0.005, (gy0 + gy1) / 2, 0)))
        m.add('front', rbox('grille', 0.04, gy1 - gy0 - 0.03, 1.20, 0.02, (xf + 0.015, (gy0 + gy1) / 2, 0)))
        y = gy0 + 0.04
        while y < gy1 - 0.03:
            m.add('front', rbox('chrome', 0.03, 0.045, 1.20, 0.015, (xf + 0.03, y, 0)))
            y += 0.095
        m.add('front', rbox('chrome', 0.03, gy1 - gy0 - 0.03, 0.05, 0.015, (xf + 0.032, (gy0 + gy1) / 2, 0)))
    else:
        m.add('front', rbox('trim_black', 0.07, gy1 - gy0 + 0.05, 1.30, 0.04, (xf - 0.005, (gy0 + gy1) / 2, 0)))
        m.add('front', rbox('grille', 0.04, gy1 - gy0 - 0.03, 1.18, 0.025, (xf + 0.012, (gy0 + gy1) / 2, 0)))
        rows = int((gy1 - gy0 - 0.04) / 0.055)
        for i in range(rows):
            y = gy0 + 0.04 + i * 0.055
            for k in range(-5, 6):
                zz = k * 0.105 + (0.05 if i % 2 else 0)
                if abs(zz) < 0.56:
                    m.add('front', rbox('trim_black', 0.02, 0.03, 0.07, 0.01, (xf + 0.032, y, zz)))
        m.add('front', rbox(P, 0.03, 0.035, 1.18, 0.015, (xf + 0.03, (gy0 + gy1) / 2 + 0.03, 0)))  # body-colour bar
    # headlamps wrapping round the corners
    hy = hf - 0.16
    for s in (1, -1):
        m.add('front', rbox('trim_black', 0.06, 0.14, 0.30, 0.03, (xf - 0.015, hy, s * 0.82)))
        corner = rbox('trim_black', 0.20, 0.13, 0.06, 0.03, (xf - 0.10, hy, s * (fz - 0.03)))
        m.add('front', corner)
        m.add('front', rbox('lens', 0.015, 0.018, 0.28, 0.008, (xf + 0.017, hy + 0.045, s * 0.82)))
        m.add('front', rbox('lens', 0.18, 0.016, 0.015, 0.006, (xf - 0.10, hy + 0.045, s * (fz + 0.002))))
        for zc in (0.74, 0.86):
            m.add('front', cylinder('lens', 0.035, 0.012, 20).rotate('z', 90).at((xf + 0.017, hy - 0.015, s * zc)))
        m.add('front', rbox('lens_amber', 0.012, 0.015, 0.12, 0.006, (xf + 0.017, hy - 0.055, s * 0.90)))
    # front bumper (wrap-around), valance, skid plate, fog lamps, tow hooks
    bmat = 'chrome' if chrome else P
    bump_geo = rbox(bmat, 0.22, 0.24, 2 * hw - 0.02, 0.06, (xf - 0.01, rk + 0.02, 0))
    m.add('front', warp(bump_geo, lambda p: (p[0] - 0.13 * (p[2] / hw) ** 4, p[1], p[2])))
    m.add('front', warp(rbox('trim_black', 0.12, 0.10, 1.50, 0.04, (xf + 0.01, rk - 0.11, 0)),
                        lambda p: (p[0] - 0.10 * (p[2] / 0.75) ** 4, p[1], p[2])))
    m.add('front', rbox('steel_dark', 0.35, 0.02, 0.80, 0.01, (xf - 0.15, rk - 0.16, 0)))
    for s in (1, -1):
        m.add('front', rbox('trim_black', 0.03, 0.08, 0.16, 0.03, (xf + 0.095, rk - 0.01, s * 0.70)))
        m.add('front', cylinder('lens', 0.03, 0.01, 18).rotate('z', 90).at((xf + 0.11, rk - 0.01, s * 0.70)))
        m.add('front', rbox('lens_red' if not chrome else 'steel_dark', 0.08, 0.05, 0.04, 0.015, (xf + 0.08, rk - 0.10, s * 0.42)))

    # ------------------------------------------------------------ rear
    for s in (1, -1):
        zt_ = b.side_fn(xr + 0.01)(belt - 0.2)
        m.add('rear', rbox('lens_red', 0.035, 0.40, 0.11, 0.015, (xr + 0.002, belt - 0.24, s * (zt_ - 0.055))))
        m.add('rear', rbox('lens', 0.037, 0.06, 0.08, 0.01, (xr + 0.001, belt - 0.30, s * (zt_ - 0.055))))
    rb_mat = 'chrome' if chrome else 'trim_black'
    m.add('rear', warp(rbox(rb_mat, 0.20, 0.20, 2 * hw - 0.04, 0.05, (xr - 0.05, rk, 0)),
                       lambda p: (p[0] + 0.05 * (p[2] / hw) ** 4, p[1], p[2])))
    m.add('rear', rbox('trim_black', 0.06, 0.04, 0.80, 0.015, (xr - 0.13, rk + 0.085, 0)))  # step pad
    m.add('rear', rbox('trim_black', 0.02, 0.15, 0.34, 0.01, (xr - 0.15, rk - 0.01, 0)))
    m.add('rear', box_span('car_white', xr - 0.165, xr - 0.16, rk - 0.07, rk + 0.05, -0.15, 0.15))
    m.add('rear', box_span('steel_dark', xr - 0.22, xr + 0.10, rk - 0.20, rk - 0.13, -0.035, 0.035))

    # dually pod marker lights
    if c['dually']:
        for s in (1, -1):
            for dx, mat in ((0.60, 'lens_amber'), (-0.60, 'lens_red')):
                x = xR + dx
                z = b.side_fn(x, pod=pod(x))(rk + 0.40)
                m.add('bed', rbox(mat, 0.06, 0.03, 0.02, 0.008, (x, rk + 0.40, s * (z + 0.005))))

    # ------------------------------------------------------------ chassis
    ch = 'chassis'
    for s in (1, -1):
        m.add(ch, box_span('trim_black', xr + 0.15, xf - 0.15, rk - 0.16, rk + 0.04, s * 0.40, s * 0.50))
        m.add(ch, box_between('steel_dark', (xR + 0.70, rk - 0.02, s * 0.48), (xR - 0.70, rk - 0.02, s * 0.48), 0.07, 0.06))
        m.add(ch, box_between('steel_dark', (xR + 0.40, rk - 0.10, s * 0.48), (xR - 0.40, rk - 0.10, s * 0.48), 0.07, 0.05))
        m.add(ch, cyl_between('steel_dark', (xR + 0.15, wy, s * 0.55), (xR + 0.25, rk, s * 0.45), 0.03, 10))
        m.add(ch, cyl_between('steel_dark', (xF - 0.10, wy, s * 0.60), (xF - 0.05, rk + 0.15, s * 0.55), 0.04, 10))
    for x in (xf - 0.25, xws, xcb, xR, xr + 0.25):
        m.add(ch, box_span('trim_black', x - 0.04, x + 0.04, rk - 0.14, rk - 0.02, -0.40, 0.40))
    zt = c['track'] / 2
    m.add(ch, cyl_between('steel_dark', (xR, wy, -zt + 0.05), (xR, wy, zt - 0.05), 0.05, 16))
    m.add(ch, sphere('steel_dark', 0.15, 18, 9).at((xR, wy, 0)))
    if c['dually']:
        m.add(ch, cyl_between('steel_dark', (xF, wy, -zt + 0.10), (xF, wy, zt - 0.10), 0.05, 16))
        m.add(ch, sphere('steel_dark', 0.14, 18, 9).at((xF, wy, 0.15)))
    else:
        m.add(ch, box_span('steel_dark', xF - 0.15, xF + 0.10, wy - 0.08, wy + 0.10, -0.15, 0.20))
        for s in (1, -1):
            m.add(ch, cyl_between('steel_dark', (xF, wy, s * 0.15), (xF, wy, s * (zt - 0.08)), 0.025, 10))
            m.add(ch, box_between('steel_dark', (xF - 0.15, wy - 0.12, s * 0.40), (xF, wy - 0.10, s * (zt - 0.10)), 0.06, 0.03))
    m.add(ch, box_span('steel_dark', xws - 0.30, xws + 0.70, rk - 0.18, rk + 0.15, -0.25, 0.25))
    m.add(ch, cyl_between('steel_dark', (xws - 0.30, rk - 0.05, 0), (xR + 0.15, wy, 0), 0.04, 12))
    m.add(ch, box_span('steel_dark', xcb - 0.30, xcb + 0.60, rk - 0.20, rk + 0.05, -0.38, -0.05))
    ex = [(xws, rk - 0.12, 0.30), (xcb, rk - 0.14, 0.30), (xR + 0.60, rk - 0.14, 0.30), (xR + 0.40, wy + 0.15, 0.30),
          (xR - 0.40, wy + 0.15, 0.30), (xR - 0.60, rk - 0.10, 0.30), (xr + 0.05, rk - 0.12, 0.62)]
    for p0, p1 in zip(ex, ex[1:]):
        m.add(ch, cyl_between('steel', p0, p1, 0.035, 10))
    m.add(ch, cyl_between('chrome', ex[-1], (xr - 0.02, rk - 0.125, 0.64), 0.045, 14))

    # ------------------------------------------------------------ wheels
    r, w, bd = c['tyre_r'], c['tyre_w'], c['bead']
    if c['dually']:
        tr = tyre(r, w, bd, 'hwy')
        front = tr + rim(bd, w, 'dually_front', 'chrome', nuts=8)
        outer = tr + rim(bd, w, 'deep', 'chrome', nuts=8)
        inner = tr + rim(bd, w, 'steel', 'steel', nuts=8)
        zc, gap = c['dual_c'], c['dual_gap'] / 2
        for s, sd in ((1, 'right'), (-1, 'left')):
            m.add('wheel_front_' + sd, place_wheel(front, (xF, wy, s * zt), s))
            m.add('wheel_rear_outer_' + sd, place_wheel(outer, (xR, wy, s * (zc + gap)), s))
            m.add('wheel_rear_inner_' + sd, place_wheel(inner, (xR, wy, s * (zc - gap)), s))
            m.add('wheel_rear_outer_' + sd, cyl_between('steel_dark', (xR, wy, s * (zc - gap)), (xR, wy, s * (zc + gap - 0.08)), 0.11, 20))
        spare = tr + rim(bd, w, 'steel', 'steel', nuts=8)
    else:
        whl = tyre(r, w, bd, 'at') + rim(bd, w, 'alloy', 'alloy', nuts=6)
        for s, sd in ((1, 'right'), (-1, 'left')):
            m.add('wheel_front_' + sd, place_wheel(whl, (xF, wy, s * zt), s))
            m.add('wheel_rear_' + sd, place_wheel(whl, (xR, wy, s * zt), s))
        spare = tyre(r, w, bd, 'at') + rim(bd, w, 'steel', 'steel_dark', nuts=6)
    m.add('spare_wheel', [g.at((xr + 0.60, rk - 0.16, 0)) for g in spare])
    return m


if __name__ == '__main__':
    for cfg in (LIGHT, DUALLY):
        save(build(cfg), cfg['folder'], cfg['file'])
