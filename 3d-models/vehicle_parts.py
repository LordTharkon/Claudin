"""Assemblies shared by the wheeled and tracked machines: tyres, rims, rubber
tracks, buckets and small fittings. Wheels are built around +Y with the outer
face toward +Y; `place_wheel` turns them to face +Z or -Z."""
import math

from meshkit import (belt_path, box, cyl_between, cylinder, extrude, from_columns, lathe,
                     norm, resample_loop, sphere)


# ------------------------------------------------------------------ tyres ---

def tyre(r, w, bead_r, tread='chevron', depth=None, segs=64, lugs=None):
    """Tyre of outside radius r and width w on a rim of bead radius bead_r.

    tread: 'chevron' (earthmover L-3/L-4 bar lugs), 'r4' (industrial lugs),
    'at' (all-terrain blocks), 'hwy' (ribbed light-truck highway tread).
    """
    d = depth if depth is not None else {'chevron': 0.06, 'r4': 0.045, 'at': 0.016, 'hwy': 0.012}[tread] * (r / 0.9 if tread in ('chevron', 'r4') else 1)
    base = r - d
    hw = w / 2
    s = base - bead_r
    side = [(bead_r, -hw * 0.80), (bead_r + 0.15 * s, -hw * 0.93), (bead_r + 0.45 * s, -hw),
            (bead_r + 0.75 * s, -hw * 0.98), (base - 0.12 * s, -hw * 0.92), (base - 0.03 * s, -hw * 0.80),
            (base, -hw * 0.64)]
    prof = side + [(rr, -y) for rr, y in reversed(side)]
    if tread == 'hwy':
        # carve circumferential grooves into the crown
        crown = []
        for k, y in enumerate([-hw * 0.64 + i * (hw * 1.28) / 10 for i in range(11)]):
            crown.append((r - (d if k in (3, 7) else 0), y))
        prof = side[:-1] + [(base, -hw * 0.64)] + crown + [(base, hw * 0.64)] + [(rr, -y) for rr, y in reversed(side[:-1])]
    prof.append(prof[0])
    out = [lathe('tyre_rubber', prof, segs)]

    if tread in ('chevron', 'r4'):
        n = lugs or (22 if tread == 'chevron' else 20)
        ang = 24 if tread == 'chevron' else 38
        lug = box('tyre_rubber', w * 0.15, w * 0.45, d)
        for k in range(n):
            for sgn, off in ((1, 0.0), (-1, 0.5)):
                phi = 360 * (k + off) / n
                l = lug.rotate('z', sgn * ang).move(y=sgn * w * 0.25, z=base + d / 2)
                out.append(l.rotate('y', phi))
                sh = box('tyre_rubber', w * 0.13, w * 0.1, d * 0.8).move(y=sgn * w * 0.46, z=base - d * 0.5)
                out.append(sh.rotate('x', -sgn * 30, (0, sgn * w * 0.46, base - d * 0.5)).rotate('z', sgn * ang).rotate('y', phi))
    elif tread == 'at':
        n = lugs or int(2 * math.pi * r / 0.075)
        for k in range(n):
            phi = 360 * k / n
            st = 1 if k % 2 else -1
            blocks = [(0.05 * w * st, 0.30 * w, 0.62),  # (axial centre, axial size, circ fraction)
                      (-0.27 * w, 0.20 * w, 0.7), (0.27 * w, 0.20 * w, 0.7)]
            for y, a, f in blocks:
                g = box('tyre_rubber', 2 * math.pi * r / n * f, a, d).move(y=y + st * 0.02 * w, z=base + d / 2)
                out.append(g.rotate('y', phi))
            for sgn in (1, -1):  # shoulder blocks wrapping the sidewall
                g = box('tyre_rubber', 2 * math.pi * r / n * 0.55, 0.12 * w, d).move(y=sgn * 0.47 * w, z=base - d * 0.6)
                out.append(g.rotate('x', -sgn * 40, (0, sgn * 0.47 * w, base - d * 0.6)).rotate('y', phi + (8 if k % 2 else 0)))
    elif tread == 'hwy':
        n = lugs or int(2 * math.pi * r / 0.05)
        for k in range(n):
            for sgn in (1, -1):
                g = box('tyre_rubber', 2 * math.pi * r / n * 0.6, 0.09 * w, d).move(y=sgn * 0.46 * w, z=base - d * 0.4)
                out.append(g.rotate('x', -sgn * 35, (0, sgn * 0.46 * w, base - d * 0.4)).rotate('y', 360 * k / n))
    return out


# ------------------------------------------------------------------- rims ---

def rim(bead_r, w, style='otr', mat='paint_yellow', nuts=10, spokes=6):
    """Rim of bead radius bead_r for a tyre of width w (rim width ~0.8 w).

    style: 'otr' (earthmover), 'steel' (truck/tractor disc), 'aluminum' (car spokes),
    'dually_front' (offset disc with hub pilot), 'deep' (deep-dish rear dual).
    """
    hr = w * 0.40
    fl = max(0.02, bead_r * 0.11)
    t = max(0.012, bead_r * 0.04)
    out = [lathe(mat, [(bead_r - t, -hr - 0.01), (bead_r + fl, -hr - 0.01), (bead_r + fl, -hr + t),
                       (bead_r, -hr + 2 * t), (bead_r, hr - 2 * t), (bead_r + fl, hr - t),
                       (bead_r + fl, hr + 0.01), (bead_r - t, hr + 0.01), (bead_r - t, -hr - 0.01)], 48)]
    hub_r = bead_r * 0.42
    nut_r = bead_r * 0.30
    if style == 'aluminum':
        face = hr * 0.55
        out.append(lathe('steel_dark', [(0, face - 0.06), (bead_r - t, face - 0.06), (bead_r - t, face - 0.05), (0, face - 0.05)], 40))
        out.append(lathe(mat, [(0, face - 0.03), (hub_r, face - 0.03), (hub_r, face + 0.01), (hub_r * 0.6, face + 0.03), (0, face + 0.03)], 32))
        for k in range(spokes):
            a = 360 * k / spokes
            for da in (-7, 7):
                sp = extrude(mat, [(hub_r * 0.8, -0.022), (bead_r - t, -0.032), (bead_r - t, 0.032), (hub_r * 0.8, 0.022)], 0.035)
                out.append(sp.rotate('x', 90).rotate('y', -(a + da)).move(y=face - 0.005))
        out.append(lathe('chrome', [(0, face + 0.03), (hub_r * 0.45, face + 0.03), (hub_r * 0.4, face + 0.045), (0, face + 0.05)], 24))
        nut_y = face + 0.02
    else:
        face = {'otr': 0.05 * w, 'steel': 0.12 * w, 'dually_front': hr + 0.04, 'deep': -hr + 0.02}[style]
        out.append(lathe(mat, [(0, face - t), (bead_r - t, face - t), (bead_r - t, face), (0, face)], 40))
        if style == 'deep':
            # cone from the disc out to the outer bead seat
            out.append(lathe(mat, [(hub_r * 1.5, face), (bead_r - t, hr - 2 * t), (bead_r - t * 2, hr - 2 * t), (hub_r * 1.5 - t, face)], 40))
        if style == 'dually_front':
            out.append(lathe(mat, [(bead_r - t, hr - 2 * t), (hub_r * 1.6, face - t), (hub_r * 1.6 - t, face - t), (bead_r - 2 * t, hr - 2 * t)], 40))
        holes = 8 if style in ('steel', 'deep', 'dually_front') else 12
        for k in range(holes):
            a = 2 * math.pi * (k + 0.5) / holes
            rr = (hub_r + bead_r) / 2
            out.append(cylinder('steel_dark', bead_r * 0.11, 0.002, 12).at((rr * math.cos(a), face + 0.001, rr * math.sin(a))))
        cap = 'chrome' if style in ('dually_front', 'deep') else 'chassis_black'
        hub_h = 0.10 if style == 'dually_front' else 0.4 * hr
        out.append(lathe(cap, [(0, face), (hub_r * 0.85, face), (hub_r * 0.8, face + hub_h * 0.7), (hub_r * 0.5, face + hub_h), (0, face + hub_h)], 32))
        nut_y = face + 0.012
    for k in range(nuts):
        a = 2 * math.pi * k / nuts
        out.append(cylinder('chrome' if style in ('aluminum', 'dually_front', 'deep') else 'steel', bead_r * 0.05, 0.03, 6)
                   .at((nut_r * math.cos(a), nut_y, nut_r * math.sin(a))))
    return out


def wheel(r, w, bead_r, tread='chevron', rim_style='otr', rim_mat='paint_yellow', **kw):
    return tyre(r, w, bead_r, tread, **kw) + rim(bead_r, w, rim_style, rim_mat)


def place_wheel(geos, center, outward=1, steer=0.0):
    """Turn a +Y-facing wheel so its outer face points to +Z (outward=1) or -Z."""
    out = []
    for g in geos:
        g = g.rotate('x', 90 if outward > 0 else -90)
        if steer:
            g = g.rotate('y', steer)
        out.append(g.at(center))
    return out


# --------------------------------------------------------- rubber tracks ---

def rubber_track_shoe(p, width, thick=0.045, lug_h=0.03, pattern='bar'):
    """One segment of a rubber track belt, `p` long, with its tread lugs and steel guide lug,
    in chain space (u = x along the belt, n = y outward, z across; pitch line at y = 0, the
    wheel side of the belt). The tread repeats every segment so all segments can share a
    mesh: 'bar' full-width bars, 'zigzag' a chevron of two angled lugs, 'block' two
    staggered blocks."""
    g = [box('tyre_rubber', p * 1.04, thick, width).move(y=thick / 2),
         box('track_steel', p * 0.45, 0.045, 0.06).move(y=-0.02)]
    y = thick + lug_h / 2
    if pattern == 'bar':
        g.append(box('tyre_rubber', p * 0.45, lug_h, width * 0.96).move(y=y))
    elif pattern == 'zigzag':
        for s in (1, -1):
            g.append(box('tyre_rubber', p * 0.45, lug_h, width * 0.44).rotate('y', s * 18).move(y=y, z=s * width * 0.24))
    elif pattern == 'block':
        for s in (1, -1):
            g.append(box('tyre_rubber', p * 0.47, lug_h, width * 0.42).move(x=-s * p * 0.22, y=y, z=s * width * 0.26))
    return g


def track_wheel(r, w, mat='track_steel', split=True):
    """Idler / roller for a rubber track: two rims either side of the guide lugs."""
    if split:
        prof = [(0, -w / 2), (r * 0.9, -w / 2), (r, -w / 2 + 0.01), (r, -0.04), (r * 0.75, -0.035),
                (r * 0.75, 0.035), (r, 0.04), (r, w / 2 - 0.01), (r * 0.9, w / 2), (0, w / 2)]
    else:
        prof = [(0, -w / 2), (r, -w / 2), (r, w / 2), (0, w / 2)]
    return lathe(mat, prof, 36).rotate('x', 90)


def sprocket(r, w=0.06, teeth=12, mat='track_steel'):
    out = [cylinder(mat, r * 0.82, w, 36).rotate('x', 90)]
    for k in range(teeth):
        t = extrude(mat, [(-r * 0.12, r * 0.75), (r * 0.12, r * 0.75), (r * 0.06, r * 1.02), (-r * 0.06, r * 1.02)], w)
        out.append(t.rotate('z', 360 * k / teeth))
    out.append(cylinder('chassis_black', r * 0.5, w + 0.08, 24).rotate('x', 90))
    for k in range(8):
        a = 2 * math.pi * k / 8
        out.append(cylinder('steel', r * 0.05, w + 0.1, 6).rotate('x', 90).at((r * 0.38 * math.cos(a), r * 0.38 * math.sin(a), 0)))
    return out


# --------------------------------------------------------------- buckets ---

def _shell(pts, t):
    """Thicken an open polyline into a closed polygon (offset to the left)."""
    off = []
    for i, p in enumerate(pts):
        a = pts[max(i - 1, 0)]
        b = pts[min(i + 1, len(pts) - 1)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        l = math.hypot(dx, dy) or 1
        off.append((p[0] - dy / l * t, p[1] + dx / l * t))
    return list(pts) + off[::-1]


def dig_bucket(width, size=1.0, teeth=5, mat='paint_yellow', ears=0.2):
    """Excavator / backhoe bucket. Local frame: pin at the origin, bucket hanging
    toward -y with the teeth pointing toward +x. `size` scales a ~0.65 m deep bucket."""
    s = size
    curve = [(0.40 * s, -0.60 * s), (0.22 * s, -0.68 * s), (0.02 * s, -0.66 * s), (-0.13 * s, -0.55 * s),
             (-0.19 * s, -0.36 * s), (-0.15 * s, -0.16 * s), (-0.05 * s, -0.02 * s), (0.08 * s, 0.03 * s)]
    t = 0.02 * max(s, 0.6)
    out = [extrude(mat, _shell(curve, -t), width)]
    side = curve + [(0.30 * s, -0.12 * s)]
    for sg in (1, -1):
        out.append(extrude(mat, side, t * 1.2).move(z=sg * (width / 2 - t * 0.6)))
    # cutting lip, wear strips and teeth
    out.append(extrude('wear_steel', _shell([(0.24 * s, -0.66 * s), (0.44 * s, -0.59 * s)], -t * 1.6), width - 0.006))
    for k in range(3):
        z = (k - 1) * width * 0.3
        out.append(extrude('wear_steel', _shell(curve[1:6], t * 0.8), 0.04 * s).move(z=z))
    for k in range(teeth):
        z = -width / 2 + width * (k + 0.5) / teeth
        tooth = extrude('wear_steel', [(0, -0.035 * s), (0.10 * s, -0.012 * s), (0.17 * s, 0.0), (0.10 * s, 0.03 * s), (0, 0.04 * s)], 0.06 * s)
        ang = math.degrees(math.atan2(0.07, 0.20))
        out.append(tooth.rotate('z', ang).at((0.40 * s, -0.61 * s, z)))
    # mounting ears: main pin at the origin, link pin above-forward
    for sg in (1, -1):
        z = sg * ears * width
        out.append(extrude(mat, [(-0.12 * s, -0.05 * s), (0.30 * s, -0.10 * s), (0.30 * s, 0.10 * s), (0.18 * s, 0.14 * s), (-0.08 * s, 0.07 * s)], 0.03).move(z=z))
    out.append(cyl_between('steel_dark', (0, 0, -ears * width - 0.04), (0, 0, ears * width + 0.04), 0.035 * s + 0.01, 14))
    out.append(cyl_between('steel_dark', (0.22 * s, 0.05 * s, -ears * width - 0.04), (0.22 * s, 0.05 * s, ears * width + 0.04), 0.03 * s + 0.01, 14))
    return out


def loader_bucket(width, depth=0.75, height=0.65, mat='paint_yellow', teeth=0, edge='wear_steel'):
    """General-purpose loader bucket. Local frame: x forward, back of bucket at x=0,
    cutting edge at (depth, 0)."""
    t = 0.012 + 0.008 * width / 2
    curve = [(depth, 0.0), (depth * 0.25, 0.0), (0.02, height * 0.15), (0.0, height * 0.55),
             (0.05, height * 0.9), (depth * 0.32, height)]
    out = [extrude(mat, _shell(curve, -t), width)]
    side = curve + [(depth * 0.55, height * 0.75)]
    for sg in (1, -1):
        out.append(extrude(mat, side, 0.02).move(z=sg * (width / 2 - 0.01)))
    out.append(extrude(edge, [(depth * 0.55, -0.004), (depth + 0.03, -0.004), (depth + 0.03, 0.02), (depth * 0.55, 0.03)], width - 0.006))
    out.append(box(mat, 0.06, 0.05, width, (depth * 0.32, height + 0.01, 0)))  # spill guard lip
    for k in range(teeth):
        z = -width / 2 + width * (k + 0.5) / teeth
        out.append(extrude('wear_steel', [(depth, 0.0), (depth + 0.13, 0.008), (depth + 0.13, 0.025), (depth, 0.045)], 0.05).move(z=z))
    # back ribs
    for sg in (1, -1):
        out.append(extrude(mat, [(0.0, height * 0.1), (-0.05, height * 0.2), (-0.05, height * 0.85), (0.04, height * 0.95)], 0.03).move(z=sg * width * 0.3))
    return out


# ------------------------------------------------------------------ misc ---

def steering_wheel(center, r=0.19, tilt=30, axis='x'):
    """Steering wheel facing back along -x, tilted `tilt` degrees from vertical."""
    ring = lathe('plastic_black', [(r + 0.017 * math.cos(a), 0.017 * math.sin(a))
                          for a in (2 * math.pi * i / 12 for i in range(13))], 32)
    out = [ring, cylinder('plastic_black', r * 0.3, 0.05, 16).move(y=0.02)]
    for k in range(3):
        a = math.radians(90 + 120 * k)
        out.append(cyl_between('plastic_black', (0, 0.02, 0), (r * math.cos(a), 0, r * math.sin(a)), 0.012, 8))
    out.append(cyl_between('chassis_black', (0, 0, 0), (0, -0.35, 0), 0.03, 10))
    # wheel plane is x-z, column along -y; stand it up facing -x then tilt
    return [g.rotate('z', 90 - tilt).at(center) for g in out]


def arch_polygon(cx, cy, r_in, r_out, a0=0.0, a1=180.0, n=20):
    """Ring segment (e.g. a fender) as an (x, y) polygon."""
    o = [(cx + r_out * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
          cy + r_out * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]
    i_ = [(cx + r_in * math.cos(math.radians(a1 - (a1 - a0) * i / n)),
           cy + r_in * math.sin(math.radians(a1 - (a1 - a0) * i / n))) for i in range(n + 1)]
    return o + i_


def hose(points, r=0.014):
    out = []
    for a, b in zip(points, points[1:]):
        out.append(cyl_between('hose_rubber', a, b, r, 8))
    out += [sphere('hose_rubber', r, 8, 4).at(p) for p in points[1:-1]]
    return out
