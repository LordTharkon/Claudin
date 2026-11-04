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

from meshkit import (MATERIALS, Geo, Model, box_between, norm, box_span, cyl_between, cylinder,
                     extrude, lathe, save, sphere)
from bodykit import fillet, loft, panel, ring_at, section, smin, smoothstep, warp
from bodykit import rbox as _rbox
from vehicle_parts import place_wheel, rim, tyre

LIGHT = dict(name='Pickup Truck', folder='Construction/pickup-truck', file='pickup_truck',
             wb=3.68, foh=0.86, roh=1.08, width=2.03, bed=1.68,
             tyre_r=0.41, tyre_w=0.275, bead=0.2286, track=1.73,
             rocker=0.50, belt=1.30, hood=1.20, roof=1.94, hood_len=1.57, rake=0.85,
             paint='paint_blue', dually=False)
DUALLY = dict(name='Dually Pickup Truck', folder='Construction/dually-pickup-truck', file='dually_pickup_truck',
              wb=4.29, foh=0.92, roh=1.22, width=2.03, rear_width=2.44, bed=2.44,
              tyre_r=0.395, tyre_w=0.245, bead=0.216, track=1.75, dual_c=0.87, dual_gap=0.30,
              rocker=0.56, belt=1.40, hood=1.33, roof=2.02, hood_len=1.65, rake=0.80,
              paint='paint_white', dually=True)


TYRE_SEGS, RIM_SEGS = 36, 32   # 1.5 mm chord error at these sizes; smooth normals hide the facets
HWY_LUGS = 40                   # shoulder blocks round the highway tyre (6 cm pitch)
SPARE_SEGS = 32                 # the under-bed spare is only seen from low angles


def rbox(mat, sx, sy, sz, r, center=(0, 0, 0), n=None):
    """bodykit.rbox with as many corner segments as its radius needs (1 mm chord error,
    1 to 4 segments per quarter round): small trim and lamp boxes get 1 or 2."""
    if n is None:
        n = max(1, min(4, round(math.pi / 4 / math.acos(max(-1.0, 1 - 0.001 / max(r, 1e-4))))))
    return _rbox(mat, sx, sy, sz, r, center, n)


def in_front_of(geos, x_cut):
    """Drop the triangles lying wholly behind the plane x = x_cut (buried in a panel)."""
    out = []
    for g in geos:
        idx = [t for t in g.idx if max(g.pos[i][0] for i in t) > x_cut + 1e-6]
        out.append(Geo(g.mat, g.pos, g.nrm, idx, fix=False))
    return out


def stations(spans, keys=(), skip=None):
    """Loft stations along x. `spans` is a list of (x0, x1, step): stations about `step` apart
    from x0 to x1. Key stations are added; regular stations strictly inside `skip` = (a, b)
    (a span the keys already cover) or within 1 cm of a key are left out, so no sliver rings."""
    lo, hi = min(sp[0] for sp in spans), max(sp[1] for sp in spans)
    xs = set()
    for x0, x1, step in spans:
        n = max(1, math.ceil((x1 - x0) / step - 1e-9))
        xs |= {round(x0 + (x1 - x0) * i / n, 5) for i in range(n + 1)}
    if skip:
        xs = {x for x in xs if not skip[0] < x < skip[1]}
    ks = {round(k, 5) for k in keys if lo <= k <= hi}
    xs = {x for x in xs if all(abs(x - k) >= 0.01 for k in ks)}
    return sorted(xs | ks)


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
        self.xcb = self.xbf + 0.05
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
        return section(self.g_yb, lambda z: yt + cr * (1 - (z / ze) ** 2), self.g_side, 0.0, self.g_rs(x), nb=2, nc=6, ns=2, nt=8)   # straight sides


def build(c):
    m = Model(c['name'], MATERIALS)
    m.node('frame')
    m.node('body', 'frame')
    m.node('cab_glass', 'body')
    m.node('bed', 'frame')
    m.node('front_fascia', 'body')
    m.node('rear_bumper', 'frame')
    m.node('spare_wheel', 'frame')
    b = Body(c)
    P = c['paint']
    hw, xF, xR, xf, xr = b.hw, b.xF, b.xR, b.xf, b.xr
    xws, xwt, xcb, xbf = b.xws, b.xwt, b.xcb, b.xbf
    rk, belt, hf, roof, wy, Ra = b.rk, b.belt, b.hf, b.roof, b.wy, b.Ra
    chrome = c['dually']
    trim = 'chrome' if chrome else 'plastic_black'

    # ------------------------------------------------------------ lower body (cab + front clip)
    # 10 cm apart on the straight runs, with stations at every kink (windshield base, hood
    # dome edges, nose taper), round the wheel arch and through the nose round-over
    a0 = math.asin((rk - wy) / Ra)
    keys = [xws, xws + 0.12, xws + 0.24, xf - 0.44, xf - 0.32, xf - 0.5, xF - b.dxA, xF + b.dxA]
    keys += [xF + Ra * math.cos(a0 + (math.pi - 2 * a0) * i / 18) for i in range(19)]
    keys += [xf - 0.09 + 0.09 * math.sin(math.pi / 2 * i / 6) for i in range(7)]
    # the cab section is constant back from the front fender flare, so only the front clip
    # needs the 10 cm spacing
    x_flare = xF - 0.8
    xs = stations([(xcb, x_flare, 0.5), (x_flare, xf, 0.10)], keys + [xcb, xf], (xF - b.dxA, xF + b.dxA))
    m.add('body', loft(P, [ring_at(x, b.lower_section(x)) for x in xs], cap0=True, cap1=True))

    # ------------------------------------------------------------ greenhouse
    # dense only where the roof rounds into the windshield and where it drops at the back;
    # the roof is a constant section between the two
    gx = stations([(xcb + 0.003, xcb + 0.08, 0.02), (xcb + 0.08, xwt - 0.30, 0.5),
                   (xwt - 0.30, xwt + 0.15, 0.025), (xwt + 0.15, xws, 0.10)])
    m.add('body', loft(P, [ring_at(x, b.g_section(x)) for x in gx], cap0=True))

    # windshield
    # a strip that follows the windshield's curve into the roof, with rounded corners
    xa, xz, n, rc = xws - 0.05, xwt + 0.05, 40, 0.05
    wpos, wnrm, widx = [], [], []
    for i in range(n + 1):
        x = xa + (xz - xa) * i / n
        e = min(x - xz, xa - x)
        inset = rc - math.sqrt(max(0.0, rc * rc - (rc - e) ** 2)) if e < rc else 0.0
        zl = b.g_side(b.g_top(x) - b.g_rs(x)) - b.g_rs(x) - 0.045 - inset
        dydx = (b.g_top(x + 0.002) - b.g_top(x - 0.002)) / 0.004
        nn = (-dydx, 1.0, 0.0)
        wpos += [(x, b.g_top(x) + 0.008, -zl), (x, b.g_top(x) + 0.008, zl)]
        wnrm += [nn, nn]
        if i:
            k = 2 * i
            widx += [(k - 2, k - 1, k + 1), (k - 2, k + 1, k)]
    m.add('cab_glass', Geo('glass_tinted', wpos, [norm(v) for v in wnrm], widx))

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
        m.add('cab_glass', panel('glass_tinted', window(xb + 0.05, x_front), on_side, nrm))
        m.add('cab_glass', panel('glass_tinted', window(x_end, xb - 0.05), on_side, nrm))
        m.add('body', panel('plastic_black', [(xb - 0.05, yw_b - 0.015), (xb + 0.05, yw_b - 0.015),
                                           (xb + 0.05, ywt(xb) + 0.02), (xb - 0.05, ywt(xb) + 0.02)],
                            lambda u, v, s=s: (u, v, s * (b.g_side(v) + 0.0045)), nrm))
        # window-line trim
        m.add('body', panel(trim, [(x_end - 0.02, yw_b - 0.025), (x_front + 0.03, yw_b - 0.025),
                                   (x_front + 0.03, yw_b - 0.008), (x_end - 0.02, yw_b - 0.008)],
                            lambda u, v, s=s: (u, v, s * (b.g_side(v) + 0.003)), nrm))
    # rear glass and third brake light
    zr = lambda y: b.g_side(y) - 0.15
    rg = fillet([(-zr(belt + 0.08), belt + 0.08), (zr(belt + 0.08), belt + 0.08),
                 (zr(roof - 0.13), roof - 0.13), (-zr(roof - 0.13), roof - 0.13)], 0.06)
    m.add('cab_glass', panel('glass_tinted', rg, lambda u, v: (xcb - 0.002, v, u), (-1, 0, 0)))
    m.add('body', rbox('lamp_red', 0.03, 0.035, 0.30, 0.012, (xcb - 0.01, roof - 0.085, 0)))

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
        zo_top = max(side(belt - rr), z_in + 2 * rr + 0.002)   # keep the rail top from folding over
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

    # 12 cm apart through the rear fender flare, round the arch and through the dually pod
    # transitions; the bed is a constant section beyond them
    keys = [xR + Ra * math.cos(a0 + (math.pi - 2 * a0) * i / 18) for i in range(19)] + [xR - b.dxA, xR + b.dxA]
    if pod_amt:
        keys += [xR + s * (0.62 + 0.40 * t / 6) for s in (1, -1) for t in range(7)]
    x0, x1 = max(xr, xR - (1.02 if pod_amt else 0.8)), min(xbf, xR + (1.02 if pod_amt else 0.8))
    bxs = stations([(xr, x0, 0.5), (x0, x1, 0.12), (x1, xbf, 0.5)], keys + [xr, xbf], (xR - b.dxA, xR + b.dxA))
    right_wall = loft(P, [bed_ring(x) for x in bxs], True, True)
    m.add('bed', right_wall, [g.mirror_z() for g in right_wall])

    # bed floor, front wall, wheelhouses, tailgate
    m.add('bed', box_span('plastic_black', xr + 0.06, xbf - 0.03, floor_y - 0.03, floor_y, -z_in, z_in))
    x = xr + 0.15
    while x < xbf - 0.05:
        m.add('bed', box_span('plastic_black', x - 0.03, x + 0.03, floor_y, floor_y + 0.012, -z_in + 0.01, z_in - 0.01))
        x += 0.24
    m.add('bed', rbox(P, 0.06, belt - rk - 0.14, 2 * z_in + 0.02, 0.02, (xbf - 0.03, (belt + rk + 0.14) / 2, 0)))
    r_wh = Ra + 0.03
    top = [(xR + r_wh * math.cos(a), wy + r_wh * math.sin(a))
           for a in (math.radians(t) for t in range(0, 181, 6)) if wy + r_wh * math.sin(a) >= floor_y - 0.02]
    for s in (1, -1):
        m.add('bed', extrude('plastic_black', top, 0.25).move(z=s * (z_in - 0.125)))
    tg = m.node('tailgate_tilt', 'bed', (xr + 0.033, rk + 0.13, 0))   # bottom hinge
    m.add(tg, rbox(P, 0.065, belt - rk - 0.13, 2 * z_in - 0.01, 0.02, (xr + 0.033, (belt + rk + 0.13) / 2 - 0.003, 0)))
    m.add(tg, rbox('plastic_black', 0.02, 0.05, 0.24, 0.01, (xr - 0.002, belt - 0.08, 0)))
    m.add(tg, panel('plastic_black', fillet([(-0.62, rk + 0.24), (0.62, rk + 0.24), (0.62, belt - 0.18), (-0.62, belt - 0.18)], 0.04),
                       lambda u, v: (xr - 0.0015, v, u), (-1, 0, 0)))

    # ------------------------------------------------------------ wheel-arch liners and lips
    for axle, width in ((xF, hw), (xR, rw)):
        e = a0 + 0.03  # stop just short of the sill so the liner ends are not flush with it
        arc_o = [(axle + (Ra - 0.01) * math.cos(a), wy + (Ra - 0.01) * math.sin(a)) for a in (e + (math.pi - 2 * e) * i / 24 for i in range(25))]
        arc_i = [(axle + (Ra + 0.025) * math.cos(a), wy + (Ra + 0.025) * math.sin(a)) for a in (math.pi - e - (math.pi - 2 * e) * i / 24 for i in range(25))]
        m.add('body', extrude('plastic_black', arc_o + arc_i, 2 * width - 0.04))
        for s in (1, -1):
            zl = b.half_width(axle) + (pod(axle) if axle == xR else 0) - 0.02
            lip = lathe(P, [(Ra + 0.012 + 0.024 * math.cos(t), 0.024 * math.sin(t))
                                                                    for t in (2 * math.pi * k / 8 for k in range(9))],
                        20, math.degrees(a0), 180 - math.degrees(a0))
            m.add('body', lip.rotate('x', -90).at((axle, wy, s * zl)))

    # ------------------------------------------------------------ door seams, handles, steps, mirrors
    side_lo = b.side_fn
    for s in (1, -1):
        for xs_ in (xws + 0.03, xb, xcb + 0.05):
            f = side_lo(xs_)
            pts = [(xs_ - 0.003, rk + 0.06), (xs_ + 0.003, rk + 0.06), (xs_ + 0.003, belt - 0.02), (xs_ - 0.003, belt - 0.02)]
            m.add('body', panel('plastic_black', pts, lambda u, v, f=f, s=s: (u, v, s * (f(v) + 0.0015)), (0, 0, s)))
        for xh in (xb - 0.10, xcb + 0.22):
            zh = side_lo(xh)(belt - 0.10)
            m.add('body', rbox(trim if chrome else P, 0.20, 0.04, 0.04, 0.018, (xh - 0.04, belt - 0.10, s * (zh + 0.012))))
            m.add('body', panel('plastic_black', fillet([(xh - 0.15, belt - 0.125), (xh + 0.07, belt - 0.125), (xh + 0.07, belt - 0.075), (xh - 0.15, belt - 0.075)], 0.02),
                                lambda u, v, s=s, zh=zh: (u, v, s * (zh + 0.0015)), (0, 0, s)))
        # running boards
        m.add('body', rbox('plastic_black', xws - xcb - 0.10, 0.05, 0.17, 0.02, ((xws + xcb) / 2, rk - 0.08, s * (hw + 0.02))))
        for xm in (xws + 0.3, xcb - 0.3):
            m.add('body', box_span('steel_dark', xm - 0.03, xm + 0.03, rk - 0.08, rk + 0.02, s * (hw - 0.12), s * (hw - 0.05)))
        # mirrors
        if c['dually']:
            arm = [(xws - 0.10, belt + 0.06), (xws - 0.10, belt + 0.32)]
            for ax_, ay_ in arm:
                m.add('body', cyl_between('plastic_black', (ax_, ay_, s * (hw - 0.05)), (ax_ - 0.03, ay_ + 0.04, s * (hw + 0.30)), 0.015, 10))
            m.add('body', rbox('plastic_black', 0.11, 0.42, 0.25, 0.04, (xws - 0.13, belt + 0.24, s * (hw + 0.36))))
            m.add('body', rbox('glass', 0.012, 0.37, 0.21, 0.004, (xws - 0.19, belt + 0.24, s * (hw + 0.36))))
            m.add('body', rbox('lamp_amber', 0.07, 0.015, 0.12, 0.006, (xws - 0.13, belt + 0.455, s * (hw + 0.36))))
        else:
            m.add('body', rbox(P, 0.10, 0.07, 0.10, 0.03, (xws - 0.12, belt + 0.09, s * (hw - 0.01))))
            m.add('body', rbox(P, 0.12, 0.19, 0.25, 0.05, (xws - 0.15, belt + 0.18, s * (hw + 0.12))))
            m.add('body', rbox('glass', 0.012, 0.16, 0.21, 0.006, (xws - 0.212, belt + 0.18, s * (hw + 0.12))))
            m.add('body', rbox('lamp_amber', 0.05, 0.012, 0.12, 0.005, (xws - 0.12, belt + 0.085, s * (hw + 0.13))))

    # roof details
    if c['dually']:
        for k in range(5):
            x = xwt - 0.05
            m.add('body', rbox('lamp_amber', 0.05, 0.03, 0.09, 0.012, (x, b.g_top(x) + b.g_crown(x) + 0.01, -0.30 + 0.15 * k)))
    else:
        x = xcb + 0.30
        m.add('body', extrude('plastic_black', fillet([(x + 0.08, 0), (x - 0.10, 0), (x - 0.06, 0.06), (x - 0.02, 0.065)], 0.01), 0.035)
              .move(y=b.g_top(x) + b.g_crown(x) - 0.002))

    # ------------------------------------------------------------ front fascia
    fz = b.half_width(xf) + b.nose_inset(xf)  # half-width of the nose just before the round-over
    gy0, gy1 = rk + 0.17, hf - 0.11
    if chrome:
        m.add('front_fascia', rbox('chrome', 0.07, gy1 - gy0 + 0.06, 1.34, 0.035, (xf - 0.005, (gy0 + gy1) / 2, 0)))
        m.add('front_fascia', rbox('plastic_black', 0.04, gy1 - gy0 - 0.03, 1.20, 0.02, (xf + 0.015, (gy0 + gy1) / 2, 0)))
        y = gy0 + 0.04
        while y < gy1 - 0.03:
            m.add('front_fascia', rbox('chrome', 0.03, 0.045, 1.20, 0.015, (xf + 0.03, y, 0)))
            y += 0.095
        m.add('front_fascia', rbox('chrome', 0.03, gy1 - gy0 - 0.03, 0.05, 0.015, (xf + 0.032, (gy0 + gy1) / 2, 0)))
    else:
        m.add('front_fascia', rbox('plastic_black', 0.07, gy1 - gy0 + 0.05, 1.30, 0.04, (xf - 0.005, (gy0 + gy1) / 2, 0)))
        m.add('front_fascia', rbox('plastic_black', 0.04, gy1 - gy0 - 0.03, 1.18, 0.025, (xf + 0.012, (gy0 + gy1) / 2, 0)))
        rows = int((gy1 - gy0 - 0.04) / 0.055)
        for i in range(rows):
            y = gy0 + 0.04 + i * 0.055
            for k in range(-5, 6):
                zz = k * 0.105 + (0.05 if i % 2 else 0)
                if abs(zz) < 0.56:
                    # half-buried in the grille panel: keep only the part in front of it
                    m.add('front_fascia', in_front_of(rbox('plastic_black', 0.02, 0.03, 0.07, 0.01, (xf + 0.032, y, zz)),
                                                      xf + 0.032))
        m.add('front_fascia', rbox(P, 0.03, 0.035, 1.18, 0.015, (xf + 0.03, (gy0 + gy1) / 2 + 0.03, 0)))  # body-colour bar
    # headlamps wrapping round the corners
    hy = hf - 0.16
    for s in (1, -1):
        m.add('front_fascia', rbox('plastic_black', 0.06, 0.14, 0.30, 0.03, (xf - 0.015, hy, s * 0.82)))
        corner = rbox('plastic_black', 0.20, 0.13, 0.06, 0.03, (xf - 0.10, hy, s * (fz - 0.03)))
        m.add('front_fascia', corner)
        m.add('front_fascia', rbox('lamp_lens', 0.015, 0.018, 0.28, 0.008, (xf + 0.017, hy + 0.045, s * 0.82)))
        m.add('front_fascia', rbox('lamp_lens', 0.18, 0.016, 0.015, 0.006, (xf - 0.10, hy + 0.045, s * (fz + 0.002))))
        for zc in (0.74, 0.86):
            m.add('front_fascia', cylinder('lamp_lens', 0.035, 0.012, 20).rotate('z', 90).at((xf + 0.017, hy - 0.015, s * zc)))
        m.add('front_fascia', rbox('lamp_amber', 0.012, 0.015, 0.12, 0.006, (xf + 0.017, hy - 0.055, s * 0.90)))
    # front bumper (wrap-around), valance, skid plate, fog lamps, tow hooks
    bmat = 'chrome' if chrome else P
    bump_geo = rbox(bmat, 0.22, 0.24, 2 * hw - 0.02, 0.06, (xf - 0.01, rk + 0.02, 0))
    m.add('front_fascia', warp(bump_geo, lambda p: (p[0] - 0.13 * (p[2] / hw) ** 4, p[1], p[2])))
    m.add('front_fascia', warp(rbox('plastic_black', 0.12, 0.10, 1.50, 0.04, (xf + 0.01, rk - 0.11, 0)),
                        lambda p: (p[0] - 0.10 * (p[2] / 0.75) ** 4, p[1], p[2])))
    m.add('front_fascia', rbox('steel_dark', 0.35, 0.02, 0.80, 0.01, (xf - 0.15, rk - 0.16, 0)))
    for s in (1, -1):
        m.add('front_fascia', rbox('plastic_black', 0.03, 0.08, 0.16, 0.03, (xf + 0.095, rk - 0.01, s * 0.70)))
        m.add('front_fascia', cylinder('lamp_lens', 0.03, 0.01, 18).rotate('z', 90).at((xf + 0.11, rk - 0.01, s * 0.70)))
        m.add('front_fascia', rbox('lamp_red' if not chrome else 'steel_dark', 0.08, 0.05, 0.04, 0.015, (xf + 0.08, rk - 0.10, s * 0.42)))

    # ------------------------------------------------------------ rear
    for s in (1, -1):
        zt_ = b.side_fn(xr + 0.01)(belt - 0.2)
        m.add('rear_bumper', rbox('lamp_red', 0.035, 0.40, 0.11, 0.015, (xr + 0.002, belt - 0.24, s * (zt_ - 0.058))))
        m.add('rear_bumper', rbox('lamp_lens', 0.037, 0.06, 0.08, 0.01, (xr - 0.001, belt - 0.30, s * (zt_ - 0.058))))
    rb_mat = 'chrome' if chrome else 'plastic_black'
    m.add('rear_bumper', warp(rbox(rb_mat, 0.20, 0.20, 2 * hw - 0.04, 0.05, (xr - 0.05, rk, 0)),
                       lambda p: (p[0] + 0.05 * (p[2] / hw) ** 4, p[1], p[2])))
    m.add('rear_bumper', rbox('plastic_black', 0.06, 0.04, 0.80, 0.015, (xr - 0.13, rk + 0.085, 0)))  # step pad
    m.add('rear_bumper', rbox('plastic_black', 0.02, 0.15, 0.34, 0.01, (xr - 0.15, rk - 0.01, 0)))
    m.add('rear_bumper', box_span('paint_white', xr - 0.165, xr - 0.16, rk - 0.07, rk + 0.05, -0.15, 0.15))
    m.add('rear_bumper', box_span('steel_dark', xr - 0.22, xr + 0.10, rk - 0.20, rk - 0.13, -0.035, 0.035))

    # dually pod marker lights
    if c['dually']:
        for s in (1, -1):
            for dx, mat in ((0.60, 'lamp_amber'), (-0.60, 'lamp_red')):
                x = xR + dx
                z = b.side_fn(x, pod=pod(x))(rk + 0.40)
                m.add('bed', rbox(mat, 0.06, 0.03, 0.02, 0.008, (x, rk + 0.40, s * (z + 0.005))))

    # ------------------------------------------------------------ chassis
    ch = 'frame'
    for s in (1, -1):
        m.add(ch, box_span('plastic_black', xr + 0.15, xf - 0.15, rk - 0.16, rk + 0.04, s * 0.40, s * 0.50))
        m.add(ch, box_between('steel_dark', (xR + 0.70, rk - 0.02, s * 0.48), (xR - 0.70, rk - 0.02, s * 0.48), 0.07, 0.06))
        m.add(ch, box_between('steel_dark', (xR + 0.40, rk - 0.10, s * 0.48), (xR - 0.40, rk - 0.10, s * 0.48), 0.07, 0.05))
        m.add(ch, cyl_between('steel_dark', (xR + 0.15, wy, s * 0.55), (xR + 0.25, rk, s * 0.45), 0.03, 10))
        m.add(ch, cyl_between('steel_dark', (xF - 0.10, wy, s * 0.60), (xF - 0.05, rk + 0.15, s * 0.55), 0.04, 10))
    for x in (xf - 0.25, xws, xcb, xR, xr + 0.25):
        m.add(ch, box_span('plastic_black', x - 0.04, x + 0.04, rk - 0.14, rk - 0.02, -0.40, 0.40))
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
        tr = tyre(r, w, bd, 'hwy', segs=TYRE_SEGS, lugs=HWY_LUGS, lean_crown=True)
        front = tr + rim(bd, w, 'dually_front', 'chrome', nuts=8, segs=RIM_SEGS)
        outer = tr + rim(bd, w, 'deep', 'chrome', nuts=8, segs=RIM_SEGS)
        # the inner dual's face is clamped against the outer wheel: barrel and disc only
        inner = tr + rim(bd, w, 'steel', 'steel', nuts=0, segs=RIM_SEGS)[:2]
        zc, gap = c['dual_c'], c['dual_gap'] / 2
        for s, sd in ((1, 'right'), (-1, 'left')):
            st = m.node('wheel_front_%s_steer' % sd, 'frame', (xF, wy, s * (zt - 0.10)))
            m.add(m.node('wheel_front_%s_spin' % sd, st, (xF, wy, s * zt)), place_wheel(front, (xF, wy, s * zt), s))
            rr = m.node('wheel_rear_%s_spin' % sd, 'frame', (xR, wy, s * zc))   # dual pair turns together
            m.add(rr, place_wheel(outer, (xR, wy, s * (zc + gap)), s))
            m.add(rr, place_wheel(inner, (xR, wy, s * (zc - gap)), s))
            m.add(rr, cyl_between('steel_dark', (xR, wy, s * (zc - gap)), (xR, wy, s * (zc + gap - 0.08)), 0.11, 20))
        spare = tyre(r, w, bd, 'hwy', segs=SPARE_SEGS, lugs=HWY_LUGS, lean_crown=True) + rim(bd, w, 'steel', 'steel', nuts=8, segs=SPARE_SEGS)
    else:
        whl = tyre(r, w, bd, 'at', segs=TYRE_SEGS) + rim(bd, w, 'aluminum', 'aluminum', nuts=6, segs=RIM_SEGS)
        for s, sd in ((1, 'right'), (-1, 'left')):
            st = m.node('wheel_front_%s_steer' % sd, 'frame', (xF, wy, s * (zt - 0.08)))
            m.add(m.node('wheel_front_%s_spin' % sd, st, (xF, wy, s * zt)), place_wheel(whl, (xF, wy, s * zt), s))
            m.add(m.node('wheel_rear_%s_spin' % sd, 'frame', (xR, wy, s * zt)), place_wheel(whl, (xR, wy, s * zt), s))
        spare = tyre(r, w, bd, 'at', segs=SPARE_SEGS) + rim(bd, w, 'steel', 'steel_dark', nuts=6, segs=SPARE_SEGS)
    m.add('spare_wheel', [g.at((xr + 0.60, rk - 0.16, 0)) for g in spare])
    return m


if __name__ == '__main__':
    for cfg in (LIGHT, DUALLY):
        save(build(cfg), cfg['folder'], cfg['file'])
