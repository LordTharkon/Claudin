"""Site cabins: a site office trailer and a 20 ft shipping container."""
import math

from meshkit import MATERIALS, Model, box, box_between, box_span, cyl_between, cylinder, extrude, save
from bodykit import rbox
from dozer_parts import handrail
from site_parts import corrugated, mesh_panel, screw_jack, wall_sheets


def site_office():
    """6.0 x 2.4 m steel-clad site office, 2.6 m to the roof, standing on blocking jacks."""
    m = Model('Site Office Trailer', MATERIALS)
    L, W = 6.0, 2.4
    hl, hw = L / 2, W / 2
    yb, yf, yt, yr = 0.30, 0.45, 2.45, 2.60      # base frame bottom, floor, wall top, roof peak
    m.node('base_frame')
    m.node('cabin', 'base_frame')
    m.node('steps', 'base_frame')
    m.node('jacks')

    # steel base frame with fork pockets
    p = 'base_frame'
    for s in (1, -1):
        m.add(p, box_span('chassis_black', -hl, hl, yb, yf, s * (hw - 0.10), s * hw))
        for x in (-1.0, 1.0):
            m.add(p, box_span('steel_dark', x - 0.18, x + 0.18, yb + 0.03, yf - 0.03, s * hw, s * (hw + 0.004)))
    for x in (-hl, hl):
        m.add(p, box_span('chassis_black', x - 0.10 if x > 0 else x, x if x > 0 else x + 0.10, yb, yf, -hw + 0.10, hw - 0.10))
    for x in (-1.8, -0.6, 0.6, 1.8):
        m.add(p, box_span('chassis_black', x - 0.05, x + 0.05, yb + 0.02, yf - 0.01, -hw + 0.10, hw - 0.10))

    c = 'cabin'
    # corner posts and top frame
    for x in (-hl + 0.06, hl - 0.06):
        for s in (1, -1):
            m.add(c, box('paint_gray', 0.12, yt + 0.13 - yf, 0.12, (x, (yf + yt + 0.13) / 2, s * (hw - 0.06))))
    for s in (1, -1):
        m.add(c, box_span('paint_gray', -hl + 0.12, hl - 0.12, yt, yt + 0.13, s * (hw - 0.10), s * hw))
    for x in (-hl, hl):
        m.add(c, box_span('paint_gray', x if x < 0 else x - 0.10, x + 0.10 if x < 0 else x, yt, yt + 0.13, -hw + 0.12, hw - 0.12))
    # cambered roof sheet
    m.add(c, extrude('paint_white', [(-hw + 0.10, yt + 0.06), (hw - 0.10, yt + 0.06), (hw - 0.10, yt + 0.11),
                                     (0.0, yr), (-hw + 0.10, yt + 0.11)], L - 0.20).rotate('y', 90))
    # walls: corrugated steel cladding with openings for 2 windows and the door (front, +Z)
    win1, win2 = (-2.25, -1.05, 1.20, 2.10), (1.55, 2.55, 1.20, 2.10)
    door = (0.20, 1.10, yf + 0.01, 2.41)
    front = wall_sheets('paint_white', -hl + 0.12, hl - 0.12, yf, yt, [win1, win2, door], pitch=0.10, depth=0.018)
    m.add(c, [g.move(z=hw - 0.035) for g in front])
    back = wall_sheets('paint_white', -hl + 0.12, hl - 0.12, yf, yt, [], pitch=0.10, depth=0.018)
    m.add(c, [g.mirror_z().move(z=-hw + 0.035) for g in back])
    for x in (-hl, hl):
        end = wall_sheets('paint_white', -hw + 0.12, hw - 0.12, yf, yt, [], pitch=0.10, depth=0.018)
        sgn = 1 if x > 0 else -1
        m.add(c, [g.rotate('y', 90 * sgn).move(x=x - sgn * 0.035) for g in end])
    # inner lining behind the cladding so the openings are not see-through
    m.add(c, box_span('paint_white', -hl + 0.12, hl - 0.12, yf, yt, -hw + 0.06, -hw + 0.07))
    for x0, x1, y0, y1 in (win1, win2):
        # aluminium window frame, glass, mullion and sill
        zf = hw - 0.035
        # frame members straddle the cut edges of the cladding (no shared faces)
        m.add(c, box_span('aluminum', x0 - 0.05, x1 + 0.05, y0 - 0.05, y0 + 0.012, zf - 0.02, zf + 0.04))
        m.add(c, box_span('aluminum', x0 - 0.05, x1 + 0.05, y1 - 0.012, y1 + 0.05, zf - 0.02, zf + 0.04))
        m.add(c, box_span('aluminum', x0 - 0.05, x0 + 0.012, y0 + 0.012, y1 - 0.012, zf - 0.02, zf + 0.04))
        m.add(c, box_span('aluminum', x1 - 0.012, x1 + 0.05, y0 + 0.012, y1 - 0.012, zf - 0.02, zf + 0.04))
        m.add(c, box_span('aluminum', (x0 + x1) / 2 - 0.02, (x0 + x1) / 2 + 0.02, y0 + 0.006, y1 - 0.006, zf - 0.01, zf + 0.025))
        m.add(c, box_span('glass', x0, x1, y0, y1, zf - 0.005, zf + 0.001))
        m.add(c, box_span('paint_gray', x0 - 0.08, x1 + 0.08, y0 - 0.09, y0 - 0.05, zf - 0.02, zf + 0.07))
        for k in range(1, 6):   # security bars
            xb = x0 + (x1 - x0) * k / 6
            m.add(c, cyl_between('steel_dark', (xb, y0 + 0.01, zf + 0.06), (xb, y1 - 0.01, zf + 0.06), 0.008, 8))
        m.add(c, box_span('steel_dark', x0 - 0.04, x1 + 0.04, y0 + 0.01, y0 + 0.03, zf + 0.04, zf + 0.075))
        m.add(c, box_span('steel_dark', x0 - 0.04, x1 + 0.04, y1 - 0.03, y1 - 0.01, zf + 0.04, zf + 0.075))
    # door frame (fixed) and the door leaf, hinged on its left edge
    x0, x1, y0, y1 = door
    zf = hw - 0.035
    m.add(c, box_span('paint_gray', x0 - 0.06, x0 + 0.004, y0 + 0.002, y1 + 0.06, zf - 0.02, zf + 0.05))
    m.add(c, box_span('paint_gray', x1 - 0.004, x1 + 0.06, y0 + 0.002, y1 + 0.06, zf - 0.02, zf + 0.05))
    m.add(c, box_span('paint_gray', x0 + 0.004, x1 - 0.004, y1 - 0.004, y1 + 0.06, zf - 0.02, zf + 0.05))
    d = m.node('door_rotate', c, (x0, (y0 + y1) / 2, zf + 0.03))
    m.add(d, box_span('paint_gray', x0 + 0.005, x1 - 0.005, y0 + 0.005, y1 - 0.005, zf - 0.01, zf + 0.03))
    m.add(d, box_span('glass', x0 + 0.30, x1 - 0.30, 1.55, 2.05, zf + 0.03, zf + 0.034))
    m.add(d, rbox('steel', 0.14, 0.03, 0.03, 0.012, (x1 - 0.14, 1.45, zf + 0.06)))
    m.add(d, box_span('steel', x1 - 0.12, x1 - 0.06, 1.38, 1.52, zf + 0.03, zf + 0.04))
    for y in (0.75, 1.45, 2.15):  # hinge knuckles
        m.add(d, cyl_between('steel', (x0, y - 0.05, zf + 0.035), (x0, y + 0.05, zf + 0.035), 0.014, 10))
    # light over the door, power inlet box
    m.add(c, box_span('plastic_black', 0.50, 0.80, 2.47, 2.55, zf + 0.06, zf + 0.20))
    m.add(c, box_span('lamp_lens', 0.52, 0.78, 2.465, 2.47, zf + 0.08, zf + 0.18))
    m.add(c, box_span('paint_gray', hl - 0.03, hl + 0.10, 0.80, 1.10, -0.60, -0.35))
    # air-conditioning unit on the back wall
    ax, ay = 2.0, 1.55
    m.add(c, rbox('paint_white', 0.85, 0.58, 0.32, 0.03, (ax, ay, -hw - 0.20)))
    m.add(c, cylinder('steel_dark', 0.21, 0.01, 32).rotate('x', 90).at((ax - 0.10, ay, -hw - 0.36)))
    for k in range(-3, 4):
        m.add(c, box_span('steel_dark', ax - 0.31 + 0.035 * 0 + k * 0.0, ax + 0.11, ay + k * 0.05 - 0.004, ay + k * 0.05 + 0.004,
                          -hw - 0.372, -hw - 0.366))
    m.add(c, box_span('steel_dark', ax + 0.20, ax + 0.38, ay - 0.22, ay + 0.22, -hw - 0.364, -hw - 0.360))
    for x in (ax - 0.32, ax + 0.32):
        m.add(c, box_span('steel_dark', x - 0.02, x + 0.02, ay - 0.32, ay - 0.28, -hw - 0.36, -hw - 0.03))
    m.add(c, box_span('paint_white', ax + 0.30, ax + 0.38, ay - 0.10, ay + 0.70, -hw - 0.09, -hw - 0.03))  # pipe cover

    # steps: landing + two treads with handrails
    s_ = 'steps'
    sx0, sx1 = 0.05, 1.25
    z0 = hw + 0.03
    m.add(s_, box_span('steel', sx0, sx1, yf - 0.06, yf - 0.03, z0, z0 + 0.80))
    for k, (y, za) in enumerate(((0.30, z0 + 0.80), (0.15, z0 + 1.08))):
        m.add(s_, box_span('steel', sx0 + 0.05, sx1 - 0.05, y - 0.03, y, za, za + 0.28))
    for x in (sx0, sx1 - 0.04):
        m.add(s_, box_between('steel_dark', (x + 0.02, yf - 0.03, z0 + 0.80), (x + 0.02, 0.07, z0 + 1.36), 0.04, 0.12, (1, 0, 0)))
        for zz in (z0 + 0.05, z0 + 0.75):
            m.add(s_, box_span('steel_dark', x, x + 0.04, 0.0, yf - 0.06, zz - 0.02, zz + 0.02))
        m.add(s_, handrail([(x + 0.02, yf - 0.03, z0 + 0.10), (x + 0.02, yf + 0.95, z0 + 0.10),
                            (x + 0.02, yf + 0.95, z0 + 0.78), (x + 0.02, 0.95, z0 + 1.36), (x + 0.02, 0.019, z0 + 1.36)],
                           mat='steel'))

    # blocking jacks under the frame
    for x in (-2.6, -0.45, 2.6):   # middle pair kept clear of the steps
        for s in (1, -1):
            m.add('jacks', screw_jack(x, s * (hw - 0.05), 0.0, yb))
    return m


def shipping_container():
    """ISO 20 ft dry container, 6.058 x 2.438 x 2.591 m, doors at the rear (-X)."""
    m = Model('Shipping Container 20ft', MATERIALS)
    L, W, H = 6.058, 2.438, 2.591
    hl, hw = L / 2, W / 2
    P = 'paint_red'
    m.node('container')
    c = 'container'
    cx, cy, cz = 0.178, 0.118, 0.162     # corner casting size
    # corner castings with apertures
    for x in (-hl + cx / 2, hl - cx / 2):
        for y in (cy / 2, H - cy / 2):
            for s in (1, -1):
                ctr = (x, y, s * (hw - cz / 2))
                m.add(c, box(P, cx, cy, cz, ctr))
                m.add(c, rbox('steel_dark', 0.12, 0.06, 0.006, 0.02, (x, y, s * (hw + 0.002))))
                m.add(c, rbox('steel_dark', 0.006, 0.06, 0.06, 0.02, (x + (0.091 if x > 0 else -0.091), y, ctr[2])))
                if y > 1:   # aperture in the top face (bottom castings sit on the ground)
                    m.add(c, rbox('steel_dark', 0.08, 0.006, 0.06, 0.02, (x, y + 0.061, ctr[2])))
    # corner posts, rails, headers
    for x in (-hl + 0.075, hl - 0.075):
        for s in (1, -1):
            m.add(c, box(P, 0.15, H - 2 * cy, 0.15, (x, H / 2, s * (hw - 0.075))))
    for s in (1, -1):
        m.add(c, box_span(P, -hl + cx, hl - cx, 0.0, 0.17, s * (hw - 0.10), s * hw))       # bottom side rail
        m.add(c, box_span(P, -hl + cx, hl - cx, H - 0.10, H - 0.004, s * (hw - 0.08), s * hw))  # top side rail
        for x in (-1.03, 1.03):  # forklift pockets
            m.add(c, box_span('steel_dark', x - 0.18, x + 0.18, 0.02, 0.13, s * hw, s * (hw + 0.004)))
        for x in (-2.3, 2.3):     # vents
            m.add(c, box_span(P, x - 0.08, x + 0.08, H - 0.24, H - 0.16, s * (hw - 0.03), s * (hw - 0.004)))
    for y0, y1 in ((0.0, 0.17), (H - 0.12, H - 0.004)):
        m.add(c, box_span(P, hl - 0.15, hl - 0.004, y0, y1, -hw + cz, hw - cz))            # front sill / header
        m.add(c, box_span(P, -hl + 0.004, -hl + 0.20, y0, y1, -hw + cz, hw - cz))          # rear sill / header
    # corrugated side walls, front wall and roof
    for s in (1, -1):
        sheet = corrugated(P, L - 2 * cx, H - 0.27, pitch=0.278, depth=0.036, crest=0.38)
        sheet = sheet.move(x=-hl + cx, y=0.17, z=hw - 0.05)
        m.add(c, sheet if s > 0 else sheet.mirror_z())
    fw = corrugated(P, W - 2 * cz, H - 0.29, pitch=0.278, depth=0.036, crest=0.38)
    m.add(c, fw.rotate('y', 90).move(x=hl - 0.08, y=0.17, z=hw - cz))
    roof = corrugated(P, L - 2 * cx, W - 0.20, pitch=0.30, depth=0.012)
    m.add(c, roof.rotate('x', -90).move(x=-hl + cx, y=H - 0.03, z=hw - 0.10))
    m.add(c, box_span('plywood', -hl + 0.20, hl - 0.15, 0.13, 0.158, -hw + 0.10, hw - 0.10))   # floor
    for x in [-hl + 0.4 + 0.6 * i for i in range(10)]:
        m.add(c, box_span(P, x - 0.025, x + 0.025, 0.02, 0.13, -hw + 0.10, hw - 0.10))       # cross members
    # rear doors, hinged at the outer edges (left door = -Z when facing the doors from behind)
    dh0, dh1 = 0.18, H - 0.13
    dw = (W - 0.30) / 2 - 0.003
    xd = -hl + 0.11   # door skin recessed so hardware stays inside the ISO envelope
    for name, s in (('door_L_rotate', -1), ('door_R_rotate', 1)):
        zo = s * (hw - 0.15)                      # outer (hinge) edge
        zi = zo - s * dw                          # inner edge at the centre line
        d = m.node(name, c, (xd, (dh0 + dh1) / 2, zo))
        za, zb = min(zo, zi), max(zo, zi)
        m.add(d, box_span(P, xd - 0.03, xd + 0.03, dh0, dh0 + 0.06, za, zb))
        m.add(d, box_span(P, xd - 0.03, xd + 0.03, dh1 - 0.06, dh1, za, zb))
        m.add(d, box_span(P, xd - 0.03, xd + 0.03, dh0 + 0.06, dh1 - 0.06, za, za + 0.06))
        m.add(d, box_span(P, xd - 0.03, xd + 0.03, dh0 + 0.06, dh1 - 0.06, zb - 0.06, zb))
        panel = corrugated(P, dw - 0.12, dh1 - dh0 - 0.12, pitch=0.25, depth=0.025, crest=0.45)
        m.add(d, panel.rotate('y', -90).move(x=xd - 0.0, y=dh0 + 0.06, z=za + 0.06))
        # locking bars with handles and cams (keepers stay on the frame)
        for zb_ in (za + dw * 0.28, za + dw * 0.72):
            m.add(d, cyl_between('steel', (xd - 0.06, dh0 + 0.08, zb_), (xd - 0.06, dh1 - 0.08, zb_), 0.014, 10))
            for y in (dh0 + 0.30, dh1 - 0.30):
                m.add(d, box_span('steel', xd - 0.05, xd - 0.03, y - 0.03, y + 0.03, zb_ - 0.03, zb_ + 0.03))
            m.add(d, box_between('steel', (xd - 0.08, 1.15, zb_), (xd - 0.08, 1.12, zb_ - s * 0.30), 0.03, 0.02, (1, 0, 0)))
            for y in (dh0 + 0.02, dh1 - 0.02):
                m.add(c, box_span('steel_dark', xd - 0.10, xd - 0.04, y - 0.025, y + 0.025, zb_ - 0.035, zb_ + 0.035))
        for y in (0.45, 1.05, 1.65, 2.25):   # hinge blades on the door, pins on the corner post
            m.add(d, box_span('steel', xd - 0.04, xd - 0.03, y - 0.03, y + 0.03, za if s < 0 else zb - 0.18,
                              za + 0.18 if s < 0 else zb))
            m.add(c, cyl_between('steel_dark', (xd - 0.045, y - 0.045, zo + s * 0.02), (xd - 0.045, y + 0.045, zo + s * 0.02), 0.016, 10))
        m.add(d, box_span('steel', xd - 0.032, xd - 0.03, 1.55, 1.75, zi - s * 0.45 - 0.12, zi - s * 0.45 + 0.12)) if s > 0 else None
    return m


MODELS = [(site_office, 'Construction/site-office-trailer', 'site_office_trailer'),
          (shipping_container, 'Construction/shipping-container-20ft', 'shipping_container_20ft')]

if __name__ == '__main__':
    for fn, folder, name in MODELS:
        save(fn(), folder, name)
