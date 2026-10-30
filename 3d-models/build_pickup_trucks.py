"""Modern full-size pickups from one parametric body:

* Pickup Truck: half-ton crew cab, 5.5 ft bed, 4x4, 275/65R18 all-terrain tyres
  (wheelbase 3.68, length 5.89, width 2.03, height ~1.96)
* Dually Pickup Truck: heavy-duty one-ton crew cab, 8 ft bed, dual rear wheels,
  LT235/80R17 tyres (wheelbase 4.29, length 6.71, width 2.44 over rear fenders, height ~2.03)
"""
import math

from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, cylinder,
                     extrude, save, sphere)
from vehicle_parts import place_wheel, rim, tyre

LIGHT = dict(name='Pickup Truck', folder='pickup-truck', file='pickup_truck',
             wb=3.68, foh=0.85, roh=1.06, width=2.03, bed=1.68,
             tyre_r=0.41, tyre_w=0.275, bead=0.2286, track=1.73,
             rocker=0.50, belt=1.33, hood=1.22, roof=1.93,
             paint='car_blue', trim='trim_black', dually=False)
DUALLY = dict(name='Dually Pickup Truck', folder='dually-pickup-truck', file='dually_pickup_truck',
              wb=4.29, foh=0.90, roh=1.20, width=2.03, rear_width=2.44, bed=2.44,
              tyre_r=0.395, tyre_w=0.245, bead=0.216, track=1.75, dual_c=0.87, dual_gap=0.30,
              rocker=0.55, belt=1.40, hood=1.32, roof=2.00,
              paint='car_white', trim='chrome', dually=True)


def arch_pts(cx, cy, r, y_cut, forward):
    """Arc over a wheel from y_cut back down to y_cut; forward=True runs rear->front."""
    a0 = math.asin(max(-1.0, min(1.0, (y_cut - cy) / r)))
    n = 18
    angs = [math.pi - a0 - (math.pi - 2 * a0) * i / n for i in range(n + 1)]
    if not forward:
        angs.reverse()
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in angs]


def build(c):
    m = Model(c['name'], MATERIALS)
    P, T = c['paint'], c['trim']
    W, hw = c['width'], c['width'] / 2
    xF, xR = c['wb'] / 2, -c['wb'] / 2
    xf, xr = xF + c['foh'], xR - c['roh']
    xbf = xr + c['bed']
    xcb = xbf - 0.05
    xws = xf - 1.62
    rk, belt, hf, roof = c['rocker'], c['belt'], c['hood'], c['roof']
    wy = c['tyre_r'] - 0.01
    Ra = c['tyre_r'] + 0.07
    hw_g = hw - 0.08  # greenhouse half-width (tumblehome)

    # ---------------------------------------------------------- body shell
    lower = [(xf, rk + 0.10), (xf, hf - 0.06), (xf - 0.07, hf), (xws, belt), (xcb, belt), (xcb, rk)]
    lower += arch_pts(xF, wy, Ra, rk, True)
    m.add('body', extrude(P, lower, W))
    # hood power dome
    hood_y = lambda x: hf + (belt - hf) * (xf - 0.07 - x) / (xf - 0.07 - xws)
    x1, x2 = xf - 0.30, xws + 0.08
    m.add('body', extrude(P, [(x1, hood_y(x1) - 0.01), (x2, hood_y(x2) - 0.01), (x2, hood_y(x2) + 0.03),
                              (x1 + 0.08, hood_y(x1 + 0.08) + 0.03)], 0.90))
    # greenhouse
    xwt = xws - 0.82
    green = [(xws, belt), (xwt, roof - 0.03), (xwt - 0.08, roof), (xcb + 0.10, roof), (xcb + 0.03, roof - 0.05), (xcb, belt)]
    m.add('body', extrude(P, green, 2 * hw_g))
    # windshield, back glass, side glass and pillars
    d = (xwt - xws, roof - 0.03 - belt)
    l = math.hypot(*d)
    n = (d[1] / l * 0.006, -d[0] / l * 0.006)
    a = (xws - 0.04 + n[0], belt + 0.05 + n[1])
    b = (xwt + 0.02 + n[0], roof - 0.07 + n[1])
    m.add('glass', box_between('glass_tint', (*a, 0), (*b, 0), 2 * hw_g - 0.16, 0.01))
    m.add('glass', box_span('glass_tint', xcb - 0.006, xcb + 0.01, belt + 0.06, roof - 0.08, -hw_g + 0.22, hw_g - 0.22))
    xb = (xws + xcb) / 2 + 0.10  # B-pillar
    side = [(xws - 0.12, belt + 0.04), (xwt + 0.07, roof - 0.07), (xcb + 0.14, roof - 0.07), (xcb + 0.10, belt + 0.04)]
    for s in (1, -1):
        m.add('glass', extrude('glass_tint', side, 0.01).move(z=s * (hw_g + 0.004)))
        m.add('body', box_span('trim_black', xb - 0.05, xb + 0.05, belt + 0.02, roof - 0.05, s * (hw_g + 0.004), s * (hw_g + 0.012)))
        m.add('body', box_span(T if T == 'chrome' else 'trim_black', xws - 0.10, xcb + 0.08, belt - 0.012, belt + 0.004,
                               s * hw, s * (hw + 0.006)))  # belt moulding
        # door seams and handles
        for x in (xws + 0.02, xb, xcb + 0.06):
            m.add('body', box_span('trim_black', x - 0.004, x + 0.004, rk + 0.04, belt - 0.01, s * hw, s * (hw + 0.002)))
        for x in (xb + 0.12, xcb + 0.20):
            m.add('body', box_span(T if T == 'chrome' else P, x - 0.13, x, belt - 0.11, belt - 0.07, s * hw, s * (hw + 0.02)))
        # running boards
        m.add('body', box_span('trim_black', xws + 0.05, xcb - 0.05, rk - 0.10, rk - 0.06, s * (hw - 0.05), s * (hw + 0.12)))
        for x in (xws + 0.25, xcb - 0.25):
            m.add('body', box_span('trim_black', x - 0.03, x + 0.03, rk - 0.08, rk + 0.02, s * (hw - 0.10), s * (hw - 0.04)))
        # mirrors (towing mirrors on the heavy duty)
        if c['dually']:
            arm0, arm1 = (xws - 0.10, belt + 0.06, s * hw), (xws - 0.12, belt + 0.10, s * (hw + 0.30))
            m.add('body', cyl_between('trim_black', arm0, arm1, 0.014, 8))
            m.add('body', cyl_between('trim_black', (xws - 0.10, belt + 0.30, s * hw), (xws - 0.12, belt + 0.34, s * (hw + 0.30)), 0.014, 8))
            m.add('body', box('trim_black', 0.10, 0.40, 0.24, (xws - 0.12, belt + 0.22, s * (hw + 0.36))))
            m.add('body', box('glass', 0.01, 0.36, 0.20, (xws - 0.175, belt + 0.22, s * (hw + 0.36))))
            m.add('body', box('lens_amber', 0.06, 0.02, 0.10, (xws - 0.12, belt + 0.43, s * (hw + 0.36))))
        else:
            m.add('body', box_between('trim_black', (xws - 0.10, belt + 0.06, s * hw), (xws - 0.14, belt + 0.10, s * (hw + 0.06)), 0.06, 0.06))
            m.add('body', box('trim_black', 0.10, 0.20, 0.24, (xws - 0.14, belt + 0.16, s * (hw + 0.12))))
            m.add('body', box('glass', 0.01, 0.17, 0.20, (xws - 0.195, belt + 0.16, s * (hw + 0.12))))
        # fuel door (left) / wheel-arch liners
        m.add('body', extrude('trim_black', [(x, y) for x, y in arch_pts(xF, wy, Ra, rk, True)] +
                              [(x, y) for x, y in arch_pts(xF, wy, Ra + 0.02, rk, False)], W - 0.04))
    # roof details
    if c['dually']:
        for k in range(5):
            m.add('body', box('lens_amber', 0.05, 0.03, 0.08, (xwt - 0.02, roof + 0.012, -0.30 + 0.15 * k)))
    else:
        m.add('body', extrude('trim_black', [(xcb + 0.45, roof), (xcb + 0.30, roof), (xcb + 0.34, roof + 0.06)], 0.04))

    # ---------------------------------------------------------- front end
    gf = 'chrome' if c['dually'] else 'trim_black'
    m.add('front', box_span(gf, xf - 0.02, xf + 0.035, rk + 0.12, hf - 0.07, -0.66, 0.66))
    m.add('front', box_span('grille', xf + 0.03, xf + 0.04, rk + 0.17, hf - 0.12, -0.60, 0.60))
    y = rk + 0.20
    while y < hf - 0.13:
        m.add('front', box_span(gf, xf + 0.035, xf + 0.05, y, y + 0.03 if c['dually'] else y + 0.015, -0.60, 0.60))
        y += 0.10 if c['dually'] else 0.06
    if c['dually']:
        m.add('front', box_span(gf, xf + 0.035, xf + 0.05, rk + 0.17, hf - 0.12, -0.02, 0.02))
    for s in (1, -1):
        m.add('front', box_span('trim_black', xf - 0.03, xf + 0.02, hf - 0.24, hf - 0.06, s * 0.66, s * (hw - 0.04)))
        m.add('front', box_span('lens', xf + 0.02, xf + 0.025, hf - 0.10, hf - 0.08, s * 0.68, s * (hw - 0.07)))  # LED DRL
        for zc in (0.78, 0.90):
            m.add('front', cylinder('lens', 0.045, 0.01, 20).rotate('z', 90).at((xf + 0.022, hf - 0.165, s * zc)))
        m.add('front', box_span('lens_amber', xf + 0.015, xf + 0.02, hf - 0.22, hf - 0.20, s * 0.70, s * (hw - 0.08)))
    # bumper, skid plate, fog lamps, tow hooks
    bm = 'chrome' if c['dually'] else P
    m.add('front', extrude(bm, [(xf - 0.12, rk - 0.10), (xf + 0.10, rk - 0.08), (xf + 0.13, rk + 0.02), (xf + 0.10, rk + 0.14),
                                (xf - 0.05, rk + 0.14)], W - 0.04))
    m.add('front', box_span('trim_black', xf - 0.05, xf + 0.115, rk - 0.06, rk + 0.04, -0.55, 0.55))
    m.add('front', box_span('trim_black', xf - 0.30, xf - 0.05, rk - 0.22, rk - 0.10, -0.55, 0.55))
    for s in (1, -1):
        m.add('front', cylinder('lens', 0.04, 0.01, 18).rotate('z', 90).at((xf + 0.12, rk - 0.01, s * 0.72)))
        m.add('front', box_span('lens_red' if not c['dually'] else 'chrome', xf + 0.06, xf + 0.15, rk - 0.03, rk + 0.02, s * 0.40, s * 0.44))

    # ---------------------------------------------------------- bed
    bed_side = [(xbf, belt), (xbf, rk)] + arch_pts(xR, wy, Ra, rk, False) + [(xr, rk + 0.08), (xr, belt)]
    floor_y = c['tyre_r'] * 2 + 0.03
    for s in (1, -1):
        m.add('bed', extrude(P, bed_side, 0.05).move(z=s * (hw - 0.025)))
        m.add('bed', box_span('trim_black', xr + 0.02, xbf, belt, belt + 0.02, s * (hw - 0.06), s * hw))  # rail cap
        m.add('bed', box_span('lens_red', xr - 0.006, xr + 0.02, belt - 0.42, belt - 0.03, s * (hw - 0.07), s * (hw - 0.005)))
        m.add('bed', box_span('lens', xr - 0.008, xr + 0.02, belt - 0.30, belt - 0.25, s * (hw - 0.065), s * (hw - 0.01)))
        # inner wheelhouse
        r_wh = Ra + 0.02
        top = [(xR + r_wh * math.cos(a), wy + r_wh * math.sin(a))
               for a in (math.radians(t) for t in range(0, 181, 10)) if wy + r_wh * math.sin(a) >= floor_y]
        m.add('bed', extrude('trim_black', top, 0.26).move(z=s * (hw - 0.18)))
        m.add('bed', extrude('trim_black', arch_pts(xR, wy, Ra, rk, True) + arch_pts(xR, wy, Ra + 0.02, rk, False), 0.06)
              .move(z=s * (hw - 0.03)))
        # stake pockets
        for x in (xbf - 0.08, xr + 0.10):
            m.add('bed', box_span('grille', x - 0.03, x + 0.03, belt + 0.021, belt + 0.023, s * (hw - 0.05), s * (hw - 0.02)))
    m.add('bed', box_span(P, xbf, xbf + 0.05, rk + 0.15, belt, -hw, hw))
    m.add('bed', box_span('trim_black', xr + 0.06, xbf + 0.05, floor_y - 0.03, floor_y, -hw + 0.05, hw - 0.05))
    x = xr + 0.15
    while x < xbf:
        m.add('bed', box_span('trim_black', x - 0.03, x + 0.03, floor_y, floor_y + 0.015, -hw + 0.06, hw - 0.06))
        x += 0.25
    # tailgate
    m.add('bed', box_span(P, xr, xr + 0.06, rk + 0.12, belt, -hw + 0.06, hw - 0.06))
    m.add('bed', box_span('trim_black', xr - 0.015, xr, belt - 0.10, belt - 0.05, -0.12, 0.12))
    m.add('bed', box_span('trim_black', xr - 0.004, xr, rk + 0.20, belt - 0.18, -0.65, 0.65))
    m.add('bed', box_span('trim_black', xr + 0.02, xr + 0.06, belt, belt + 0.02, -hw + 0.06, hw - 0.06))
    # rear step bumper and hitch receiver
    m.add('rear', box_span('chrome' if c['dually'] else 'trim_black', xr - 0.16, xr + 0.04, rk - 0.10, rk + 0.10, -hw + 0.02, hw - 0.02))
    m.add('rear', box_span('trim_black', xr - 0.165, xr - 0.10, rk + 0.03, rk + 0.10, -0.55, 0.55))
    m.add('rear', box_span('trim_black', xr - 0.17, xr - 0.16, rk - 0.05, rk + 0.08, -0.17, 0.17))  # plate recess
    m.add('rear', box_span('car_white', xr - 0.175, xr - 0.17, rk - 0.04, rk + 0.07, -0.15, 0.15))
    m.add('rear', box_span('steel_dark', xr - 0.20, xr + 0.10, rk - 0.22, rk - 0.15, -0.035, 0.035))

    # dually rear fender pods
    if c['dually']:
        rw = c['rear_width'] / 2
        pod = arch_pts(xR, wy, belt - 0.10 - wy, rk, False) + arch_pts(xR, wy, Ra, rk, True)
        for s in (1, -1):
            g = extrude(P, pod, rw - hw + 0.02)
            m.add('bed', g.move(z=s * (hw - 0.01 + (rw - hw + 0.02) / 2)))
            m.add('bed', box('lens_amber', 0.05, 0.03, 0.03, (xR + 0.62, rk + 0.42, s * (rw - 0.03))))
            m.add('bed', box('lens_red', 0.05, 0.03, 0.03, (xR - 0.62, rk + 0.42, s * (rw - 0.03))))
            m.add('bed', extrude('trim_black', arch_pts(xR, wy, Ra, rk, True) + arch_pts(xR, wy, Ra + 0.02, rk, False), rw - hw)
                  .move(z=s * (hw + (rw - hw) / 2)))

    # ---------------------------------------------------------- chassis
    ch = 'chassis'
    for s in (1, -1):
        m.add(ch, box_span('trim_black', xr + 0.15, xf - 0.15, rk - 0.16, rk + 0.04, s * 0.40, s * 0.50))
        # leaf springs and shocks
        m.add(ch, box_between('steel_dark', (xR + 0.70, rk - 0.02, s * 0.48), (xR - 0.70, rk - 0.02, s * 0.48), 0.07, 0.06))
        m.add(ch, box_between('steel_dark', (xR + 0.40, rk - 0.10, s * 0.48), (xR - 0.40, rk - 0.10, s * 0.48), 0.07, 0.05))
        m.add(ch, cyl_between('steel_dark', (xR + 0.15, wy, s * 0.55), (xR + 0.25, rk, s * 0.45), 0.03, 10))
        m.add(ch, cyl_between('steel_dark', (xF - 0.10, wy, s * 0.60), (xF - 0.05, rk + 0.15, s * 0.55), 0.04, 10))
    for x in (xf - 0.25, xws, xcb, xR, xr + 0.25):
        m.add(ch, box_span('trim_black', x - 0.04, x + 0.04, rk - 0.14, rk - 0.02, -0.40, 0.40))
    zt = c['track'] / 2
    m.add(ch, cyl_between('steel_dark', (xR, wy, -zt + 0.05), (xR, wy, zt - 0.05), 0.05, 16))
    m.add(ch, sphere('steel_dark', 0.15, 18, 9).at((xR, wy, 0)))
    if c['dually']:  # solid front axle
        m.add(ch, cyl_between('steel_dark', (xF, wy, -zt + 0.10), (xF, wy, zt - 0.10), 0.05, 16))
        m.add(ch, sphere('steel_dark', 0.14, 18, 9).at((xF, wy, 0.15)))
    else:  # independent front suspension
        m.add(ch, box_span('steel_dark', xF - 0.15, xF + 0.10, wy - 0.08, wy + 0.10, -0.15, 0.20))
        for s in (1, -1):
            m.add(ch, cyl_between('steel_dark', (xF, wy, s * 0.15), (xF, wy, s * (zt - 0.08)), 0.025, 10))
            m.add(ch, box_between('steel_dark', (xF - 0.15, wy - 0.12, s * 0.40), (xF, wy - 0.10, s * (zt - 0.10)), 0.06, 0.03))
    m.add(ch, box_span('steel_dark', xws - 0.30, xws + 0.70, rk - 0.18, rk + 0.15, -0.25, 0.25))  # transmission
    m.add(ch, cyl_between('steel_dark', (xws - 0.30, rk - 0.05, 0), (xR + 0.15, wy, 0), 0.04, 12))  # driveshaft
    m.add(ch, box_span('steel_dark', xcb - 0.30, xcb + 0.60, rk - 0.20, rk + 0.05, -0.38, -0.05))   # fuel tank
    ex = [(xws, rk - 0.12, 0.30), (xcb, rk - 0.14, 0.30), (xR + 0.60, rk - 0.14, 0.30), (xR + 0.40, wy + 0.15, 0.30),
          (xR - 0.40, wy + 0.15, 0.30), (xR - 0.60, rk - 0.10, 0.30), (xr + 0.05, rk - 0.12, 0.62)]
    for p0, p1 in zip(ex, ex[1:]):
        m.add(ch, cyl_between('steel', p0, p1, 0.035, 10))
    m.add(ch, cyl_between('chrome', ex[-1], (xr - 0.02, rk - 0.125, 0.64), 0.045, 14))

    # ---------------------------------------------------------- wheels
    r, w, bd = c['tyre_r'], c['tyre_w'], c['bead']
    if c['dually']:
        tr = tyre(r, w, bd, 'hwy')
        front = tr + rim(bd, w, 'dually_front', 'chrome', nuts=8)
        outer = tr + rim(bd, w, 'deep', 'chrome', nuts=8)
        inner = tr + rim(bd, w, 'steel', 'steel', nuts=8)
        zc, gap = c['dual_c'], c['dual_gap'] / 2
        for s, side in ((1, 'right'), (-1, 'left')):
            m.add('wheel_front_' + side, place_wheel(front, (xF, wy, s * zt), s))
            m.add('wheel_rear_outer_' + side, place_wheel(outer, (xR, wy, s * (zc + gap)), s))
            m.add('wheel_rear_inner_' + side, place_wheel(inner, (xR, wy, s * (zc - gap)), s))
            m.add('wheel_rear_outer_' + side, cyl_between('steel_dark', (xR, wy, s * (zc - gap)), (xR, wy, s * (zc + gap - 0.08)), 0.11, 20))
        spare = tr + rim(bd, w, 'steel', 'steel', nuts=8)
    else:
        whl = tyre(r, w, bd, 'at') + rim(bd, w, 'alloy', 'alloy', nuts=6)
        for s, side in ((1, 'right'), (-1, 'left')):
            m.add('wheel_front_' + side, place_wheel(whl, (xF, wy, s * zt), s))
            m.add('wheel_rear_' + side, place_wheel(whl, (xR, wy, s * zt), s))
        spare = tyre(r, w, bd, 'at') + rim(bd, w, 'steel', 'steel_dark', nuts=6)
    m.add('spare_wheel', [g.at((xr + 0.60, rk - 0.16, 0)) for g in spare])
    return m


if __name__ == '__main__':
    for cfg in (LIGHT, DUALLY):
        save(build(cfg), cfg['folder'], cfg['file'])
