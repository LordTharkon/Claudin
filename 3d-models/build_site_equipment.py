"""Site equipment: towable light tower, towable generator, bunded fuel tank, material hoist."""
import math

from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, cylinder, extrude,
                     lathe, polyline_tube, save, sphere)
from bodykit import rbox
from dozer_parts import exhaust, louvres
from site_parts import mesh_panel, trailer_wheel
from vehicle_parts import arch_polygon, hose, place_wheel


def _trailer(m, x_rear, x_frame_front, x_coupler, half, axle_x, wheel_r, wheel_z, y_frame=0.48, frame='chassis_black'):
    """Single-axle site trailer: frame, A-frame drawbar, coupler, jockey wheel, wheels, fenders, lights."""
    p = 'trailer_frame'
    m.node(p)
    for s in (1, -1):
        m.add(p, box_span(frame, x_rear, x_frame_front, y_frame - 0.10, y_frame, s * (half - 0.06), s * half))
        m.add(p, box_between(frame, (x_frame_front, y_frame - 0.05, s * (half - 0.03)), (x_coupler - 0.25, y_frame - 0.05, s * 0.05),
                             0.06, 0.10))
        m.add(p, box('lamp_red', 0.02, 0.08, 0.14, (x_rear - 0.012, y_frame - 0.05, s * (half - 0.15))))
    for x in (x_rear + 0.03, (x_rear + x_frame_front) / 2, x_frame_front - 0.03):
        m.add(p, box_span(frame, x - 0.03, x + 0.03, y_frame - 0.09, y_frame - 0.01, -half + 0.06, half - 0.06))
    # coupler head and handle
    m.add(p, box_span('steel_dark', x_coupler - 0.30, x_coupler - 0.05, y_frame - 0.11, y_frame - 0.005, -0.06, 0.06))
    m.add(p, rbox('steel', 0.16, 0.12, 0.135, 0.04, (x_coupler - 0.06, y_frame - 0.055, 0)))
    m.add(p, cyl_between('steel', (x_coupler - 0.20, y_frame, 0), (x_coupler - 0.12, y_frame + 0.08, 0), 0.012, 8))
    # jockey wheel post and wheel
    jx = x_coupler - 0.55
    m.add(p, cyl_between('steel_dark', (jx, 0.20, 0.10), (jx, y_frame + 0.25, 0.10), 0.03, 14))
    m.add(p, cyl_between('steel', (jx, y_frame + 0.25, 0.10), (jx + 0.10, y_frame + 0.30, 0.10), 0.01, 8))
    jw = m.node('jockey_wheel_spin', p, (jx, 0.10, 0.10))
    m.add(jw, place_wheel(trailer_wheel(0.10, 0.06, 0.05), (jx, 0.10, 0.10), 1))
    m.add(p, box_span('steel_dark', jx - 0.02, jx + 0.02, 0.10, 0.22, 0.10 - 0.045, 0.10 - 0.035))
    # axle, springs, wheels, fenders
    m.add(p, cyl_between('steel_dark', (axle_x, wheel_r, -wheel_z + 0.06), (axle_x, wheel_r, wheel_z - 0.06), 0.035, 14))
    w = trailer_wheel(wheel_r, 0.165, 0.165)
    for s, side in ((1, 'right'), (-1, 'left')):
        n = m.node('wheel_%s_spin' % side, p, (axle_x, wheel_r, s * wheel_z))
        m.add(n, place_wheel(w, (axle_x, wheel_r, s * wheel_z), s))
        m.add(p, box_between('steel_dark', (axle_x - 0.35, y_frame - 0.12, s * (half - 0.03)), (axle_x + 0.35, y_frame - 0.12, s * (half - 0.03)), 0.05, 0.04))
        m.add(p, extrude('paint_gray', arch_polygon(axle_x, wheel_r, wheel_r + 0.05, wheel_r + 0.07, 10, 170), 0.22).move(z=s * wheel_z))
    return p


# ---------------------------------------------------------------------------- light tower

def light_tower():
    """Towable LED light tower: mast telescopes from 2.5 m to 8 m (four 1.375 m stages)."""
    m = Model('Light Tower', MATERIALS)
    half, axle_x, wr, wz = 0.42, -0.10, 0.28, 0.66
    tf = _trailer(m, -1.05, 0.90, 2.05, half, axle_x, wr, wz, y_frame=0.50)
    # engine / battery canopy
    c = m.node('canopy', tf)
    m.add(c, rbox('paint_yellow', 1.60, 0.76, 1.08, 0.03, (-0.05, 0.89, 0)))
    for s in (1, -1):
        m.add(c, louvres(-0.65, -0.15, 0.75, 1.10, s * 0.54, s, 0.045))
        m.add(c, box_span('chassis_black', 0.05, 0.056, 0.56, 1.22, s * 0.539, s * 0.545))
        m.add(c, box_span('steel_dark', 0.60, 0.66, 0.95, 1.00, s * 0.54, s * 0.555))
    m.add(c, box_span('plastic_black', 0.75, 0.756, 0.70, 1.05, -0.30, 0.30))   # control panel door
    m.add(c, box_span('glass', 0.756, 0.76, 0.85, 1.00, -0.20, 0.05))
    m.add(c, box_span('lamp_red', 0.756, 0.775, 0.78, 0.82, 0.12, 0.16))
    m.add(c, exhaust(0.45, 1.27, 1.42, 0.30, 0.03))
    m.add(c, cylinder('steel_dark', 0.04, 0.03, 16).at((0.30, 1.285, -0.30)))     # fuel cap
    # outriggers: tubes across the frame, beams slide out sideways, screw jacks at the ends
    for x, tag in ((0.75, 'front'), (-0.95, 'rear')):
        m.add(tf, box_span('steel_dark', x - 0.05, x + 0.05, 0.31, 0.39, -half - 0.02, half + 0.02))
        for s, side in ((1, 'right'), (-1, 'left')):
            zo = s * (half + 0.10)
            o = m.node('outrigger_%s_%s_slide' % (tag, side), tf, (x, 0.35, zo))
            m.add(o, box_span('paint_yellow', x - 0.035, x + 0.035, 0.32, 0.38, s * 0.08, zo))
            m.add(o, box_span('steel_dark', x - 0.045, x + 0.045, 0.15, 0.62, zo - 0.045, zo + 0.045))
            m.add(o, cyl_between('steel', (x, 0.07, zo), (x, 0.15, zo), 0.02, 10))
            m.add(o, box_span('steel_dark', x - 0.09, x + 0.09, 0.05, 0.07, zo - 0.09, zo + 0.09))
            m.add(o, cyl_between('steel', (x, 0.62, zo), (x, 0.66, zo), 0.012, 8))
            m.add(o, cyl_between('steel', (x, 0.66, zo), (x + 0.12, 0.66, zo), 0.01, 8))
    # telescopic mast: fixed base tube + four nested lifting stages
    mx = -0.62
    sizes = [0.18, 0.15, 0.12, 0.095, 0.07]
    parent = tf
    names = ['mast_base', 'mast_lift', 'mast_stage_2_lift', 'mast_stage_3_lift', 'mast_stage_4_lift']
    for k, (sz, nm) in enumerate(zip(sizes, names)):
        y0, y1 = 0.52 + 0.06 * k, 2.25 + 0.04 * k
        n = m.node(nm, parent, (mx, y1, 0)) if k else m.node(nm, parent)
        m.add(n, box_span('aluminum', mx - sz / 2, mx + sz / 2, y0, y1, -sz / 2, sz / 2))
        m.add(n, box_span('steel_dark', mx - sz / 2 - 0.006, mx + sz / 2 + 0.006, y1 - 0.05, y1 + 0.004, -sz / 2 - 0.006, sz / 2 + 0.006))
        parent = n
    m.add('mast_base', box_span('steel_dark', mx - 0.13, mx + 0.13, 1.20, 1.40, -0.13, 0.13))   # mast bracket on canopy
    m.add('mast_base', rbox('steel_dark', 0.16, 0.22, 0.16, 0.02, (mx + 0.10, 1.65, 0.16)))      # winch
    m.add('mast_base', cyl_between('steel', (mx + 0.10, 1.65, 0.24), (mx + 0.10, 1.65, 0.34), 0.012, 8))
    m.add('mast_base', cyl_between('steel', (mx + 0.10, 1.65, 0.34), (mx + 0.25, 1.70, 0.34), 0.01, 8))
    # lamp head (turns about Y on top of the last stage)
    hy = 2.50
    h = m.node('lamp_head_rotate', parent, (mx, 2.41, 0))
    m.add(h, cyl_between('steel_dark', (mx, 2.41, 0), (mx, hy, 0), 0.035, 14))
    m.add(h, box_span('steel_dark', mx - 0.03, mx + 0.03, hy - 0.03, hy + 0.03, -0.70, 0.70))
    for z in (-0.51, -0.17, 0.17, 0.51):
        fx = mx + 0.09
        lamp = rbox('aluminum', 0.08, 0.36, 0.30, 0.015, (0, 0, 0)) + [box('lamp_lens', 0.006, 0.30, 0.25, (0.043, 0, 0))]
        lamp += [box('aluminum', 0.03, 0.34, 0.012, (-0.055, 0, dz)) for dz in (-0.10, -0.05, 0, 0.05, 0.10)]
        m.add(h, [g.rotate('z', -15).at((fx, hy, z)) for g in lamp])
        m.add(h, box_span('steel_dark', mx + 0.03, fx - 0.03, hy - 0.02, hy + 0.02, z - 0.02, z + 0.02))
    return m


# ---------------------------------------------------------------------------- generator

def generator():
    """Towable diesel generator, 3.2 x 1.5 x 1.8 m overall, enclosed canopy on a single-axle trailer."""
    m = Model('Diesel Generator', MATERIALS)
    half, axle_x, wr, wz = 0.45, -0.35, 0.30, 0.66
    tf = _trailer(m, -1.55, 0.75, 1.60, half, axle_x, wr, wz, y_frame=0.60)
    c = m.node('canopy', tf)
    x0, x1, y0, y1, hz = -1.55, 0.75, 0.62, 1.72, 0.55
    m.add(c, rbox('paint_green', x1 - x0, y1 - y0, 2 * hz, 0.035, ((x0 + x1) / 2, (y0 + y1) / 2, 0)))
    m.add(c, box_span('chassis_black', x0 + 0.02, x1 - 0.02, y0 - 0.02, y0 + 0.04, -hz + 0.02, hz - 0.02))   # base skid
    # side access door (+Z), hinged on its front edge
    dx0, dx1, dy0, dy1 = -0.95, 0.05, 0.76, 1.56
    zf = hz
    d = m.node('panel_rotate', c, (dx1, (dy0 + dy1) / 2, zf + 0.018))
    m.add(d, rbox('paint_green', dx1 - dx0, dy1 - dy0, 0.018, 0.006, ((dx0 + dx1) / 2, (dy0 + dy1) / 2, zf + 0.010)))
    m.add(d, louvres(dx0 + 0.10, dx1 - 0.25, dy0 + 0.12, dy1 - 0.12, zf + 0.019, 1, 0.05))
    m.add(d, rbox('steel', 0.10, 0.03, 0.03, 0.01, (dx0 + 0.08, (dy0 + dy1) / 2, zf + 0.035)))
    for y in (dy0 + 0.10, dy1 - 0.10):
        m.add(d, cyl_between('steel', (dx1 - 0.01, y - 0.05, zf + 0.022), (dx1 - 0.01, y + 0.05, zf + 0.022), 0.01, 8))
    # fixed louvres on the other side, radiator outlet at the rear, control panel at the front
    m.add(c, louvres(-1.30, -0.10, 0.85, 1.50, -hz, -1, 0.05))
    for k in range(9):
        y = 0.85 + k * 0.075
        m.add(c, box('steel_dark', 0.035, 0.012, 0.80, (x0 - 0.010, y, 0)).rotate('z', 35, (x0 - 0.010, y, 0)))
    m.add(c, box_span('chassis_black', x0 - 0.004, x0 + 0.01, 0.80, 1.54, -0.45, 0.45))
    m.add(c, box_span('plastic_black', x1 - 0.005, x1 + 0.006, 1.05, 1.50, -0.35, 0.35))
    m.add(c, box_span('glass', x1 + 0.006, x1 + 0.010, 1.13, 1.44, -0.30, 0.10))
    m.add(c, cylinder('lamp_red', 0.03, 0.03, 16).rotate('z', 90).at((x1 + 0.02, 1.30, 0.22)))
    m.add(c, box_span('steel_dark', x1 - 0.004, x1 + 0.012, 0.75, 0.98, -0.40, 0.40))   # output sockets panel
    for k in range(4):
        m.add(c, cylinder('plastic_black', 0.035, 0.03, 16).rotate('z', 90).at((x1 + 0.02, 0.86, -0.27 + 0.18 * k)))
    # roof: exhaust with rain cap, lifting frame, fuel filler on the side
    # short exhaust outlet with a hinged rain flap, flush lifting point (keeps the 1.8 m height)
    m.add(c, cyl_between('steel_dark', (-1.10, y1 - 0.01, -0.25), (-1.10, 1.77, -0.25), 0.045, 18))
    m.add(c, box('steel_dark', 0.11, 0.008, 0.11, (-1.10, 1.778, -0.25)).rotate('z', 12, (-1.155, 1.77, -0.25)))
    m.add(c, box_span('steel_dark', -0.52, -0.28, y1 - 0.005, y1 + 0.03, -0.12, 0.12))
    m.add(c, box_span('steel', -0.44, -0.36, y1 + 0.03, y1 + 0.07, -0.015, 0.015))
    m.add(c, cylinder('steel_dark', 0.05, 0.03, 16).rotate('x', 90).at((-1.40, 0.95, -hz - 0.012)))
    return m


# ---------------------------------------------------------------------------- fuel tank

def fuel_tank():
    """1000 L double-walled (bunded) fuel tank on skids, with pump cabinet, hose reel and nozzle."""
    m = Model('Bunded Fuel Tank', MATERIALS)
    m.node('skids')
    m.node('tank', 'skids')
    m.node('pump_cabinet', 'skids')
    for s in (1, -1):
        m.add('skids', box_span('steel_dark', -0.95, 1.42, 0.0, 0.12, s * 0.32, s * 0.44))
        for x in (-0.45, 0.45):
            m.add('skids', box_span('steel_dark', x - 0.12, x + 0.12, 0.025, 0.095, s * 0.44, s * 0.444))   # fork pocket
    for x in (-0.80, 0.0, 0.80, 1.30):
        m.add('skids', box_span('steel_dark', x - 0.04, x + 0.04, 0.08, 0.12, -0.32, 0.32))
    t = 'tank'
    m.add(t, rbox('paint_green', 1.70, 1.00, 1.05, 0.05, (0.0, 0.62, 0)))
    m.add(t, box_span('paint_green', -0.86, 0.86, 0.615, 0.635, -0.535, 0.535))          # outer-wall seam band
    m.add(t, rbox('paint_green', 1.40, 0.04, 0.80, 0.01, (0.0, 1.13, 0)))                 # manway plate
    m.add(t, cylinder('steel', 0.08, 0.06, 24).at((-0.45, 1.175, 0.20)))                 # fill cap
    m.add(t, cyl_between('steel', (-0.45, 1.12, -0.25), (-0.45, 1.40, -0.25), 0.025, 12))  # vent
    m.add(t, cylinder('steel', 0.045, 0.06, 16, r_top=0.02).at((-0.45, 1.43, -0.25)))
    m.add(t, cylinder('steel_dark', 0.06, 0.03, 20).at((0.25, 1.165, -0.25)))             # contents gauge
    m.add(t, cylinder('glass', 0.05, 0.004, 20).at((0.25, 1.182, -0.25)))
    m.add(t, cylinder('lamp_red', 0.02, 0.05, 12).at((0.45, 1.165, 0.30)))                # leak indicator
    for x in (-0.80, 0.80):   # lifting lugs
        for s in (1, -1):
            m.add(t, extrude('steel', [(-0.05, 0), (0.05, 0), (0.035, 0.08), (-0.035, 0.08)], 0.02).move(x=x, y=1.13, z=s * 0.42))
    # pump cabinet with hose reel and nozzle
    c = 'pump_cabinet'
    cx0, cx1 = 0.88, 1.40
    for x in (cx0 + 0.02, cx1 - 0.02):
        for s in (1, -1):
            m.add(c, box_span('paint_green', x - 0.02, x + 0.02, 0.12, 1.15, s * 0.48 - 0.02, s * 0.48 + 0.02))
    m.add(c, box_span('paint_green', cx0, cx1, 1.15, 1.18, -0.52, 0.52))                  # roof
    m.add(c, box_span('paint_green', cx1 - 0.02, cx1, 0.12, 1.15, -0.46, 0.46))           # end panel
    m.add(c, box_span('paint_green', cx0 + 0.04, cx1 - 0.02, 0.12, 0.15, -0.46, 0.46))    # floor
    m.add(c, box_span('paint_green', cx0 + 0.04, cx1 - 0.02, 0.12, 1.15, -0.48, -0.46))   # back panel
    m.add(c, rbox('chassis_black', 0.30, 0.20, 0.22, 0.02, (1.13, 0.28, -0.25)))           # pump
    m.add(c, cyl_between('chassis_black', (1.13, 0.28, -0.14), (1.13, 0.28, -0.04), 0.07, 16))
    m.add(c, box_span('steel_dark', 1.02, 1.24, 0.42, 0.62, -0.12, -0.04))                # meter
    m.add(c, cylinder('glass', 0.05, 0.004, 16).rotate('x', 90).at((1.13, 0.52, -0.038)))
    # hose reel on the cabinet sides
    rc = (1.13, 0.80, 0.10)
    m.add(c, cyl_between('steel', (1.13, 0.80, -0.46), (1.13, 0.80, 0.30), 0.02, 10))
    reel = m.node('hose_reel_spin', c, rc)
    m.add(reel, cyl_between('steel_dark', (1.13, 0.80, -0.06), (1.13, 0.80, 0.26), 0.10, 24))
    for z in (-0.07, 0.27):
        m.add(reel, cyl_between('steel_dark', (1.13, 0.80, z - 0.005), (1.13, 0.80, z + 0.005), 0.24, 32))
    for i in range(6):
        z = -0.04 + i * 0.054
        for r in (0.13, 0.17):
            m.add(reel, lathe('hose_rubber', [(r + 0.02 * math.cos(a), 0.02 * math.sin(a)) for a in
                                              (2 * math.pi * k / 8 for k in range(9))], 28).rotate('x', 90).at((1.13, 0.80, z)))
    # nozzle in its holster on the outside of the cabinet, hose running to it
    m.add(c, box_span('steel_dark', cx1, cx1 + 0.08, 0.55, 0.70, 0.18, 0.30))
    nz = [rbox('plastic_black', 0.05, 0.14, 0.05, 0.02, (cx1 + 0.05, 0.66, 0.24)),
          cyl_between('steel', (cx1 + 0.05, 0.60, 0.24), (cx1 + 0.12, 0.48, 0.24), 0.012, 10),
          box_span('steel_dark', cx1 + 0.03, cx1 + 0.07, 0.60, 0.62, 0.225, 0.255)]
    m.add(c, nz)
    m.add(c, hose([(1.13, 0.60, 0.15), (1.25, 0.40, 0.25), (cx1 + 0.06, 0.45, 0.33), (cx1 + 0.06, 0.76, 0.33),
                   (cx1 + 0.05, 0.75, 0.24)], 0.016))
    return m


# ---------------------------------------------------------------------------- material hoist

def material_hoist():
    """Rack-and-pinion material/passenger hoist: six 1.508 m mast sections and a cage."""
    m = Model('Material Hoist', MATERIALS)
    m.node('base_frame')
    m.node('base_enclosure', 'base_frame')
    m.node('mast', 'base_frame')
    a = 0.225            # half mast width
    sec, n = 1.508, 6
    y0 = 0.20
    top = y0 + sec * n
    b = 'base_frame'
    m.add(b, box_span('steel_dark', -0.60, 3.35, 0.0, 0.12, -1.05, 1.05))
    m.add(b, box_span('steel_dark', -0.45, 0.45, 0.12, y0, -0.45, 0.45))
    for x in (1.25, 2.45):    # cage buffers (springs)
        for s in (1, -1):
            m.add(b, cyl_between('steel', (x, 0.12, s * 0.45), (x, 0.36, s * 0.45), 0.07, 16))
            m.add(b, cylinder('chassis_black', 0.08, 0.03, 16).at((x, 0.375, s * 0.45)))
    # base enclosure: mesh panels around three sides of the cage, gate opening at the far end
    e = 'base_enclosure'
    for s in (1, -1):
        m.add(e, mesh_panel('steel_dark', 0.35, 3.30, 0.12, 2.12, s * 1.02, pitch=0.12, frame_mat='paint_orange'))
    m.add(e, mesh_panel('steel_dark', -1.0, -0.62, 0.12, 2.12, 3.30, pitch=0.12, frame_mat='paint_orange', axis='z'))
    m.add(e, mesh_panel('steel_dark', 0.62, 1.0, 0.12, 2.12, 3.30, pitch=0.12, frame_mat='paint_orange', axis='z'))
    m.add(e, box_span('paint_orange', 3.28, 3.32, 2.06, 2.12, -0.62, 0.62))
    # mast: corner tubes, bracing and rack, section by section
    mm = 'mast'
    for i in range(n):
        ya = y0 + i * sec
        for sx in (1, -1):
            for sz in (1, -1):
                m.add(mm, cyl_between('steel', (sx * a, ya, sz * a), (sx * a, ya + sec - 0.004, sz * a), 0.038, 12))
        for yy in (ya + 0.04, ya + sec * 0.5):
            for sz in (1, -1):
                m.add(mm, cyl_between('steel', (-a, yy, sz * a), (a, yy, sz * a), 0.016, 8))
            m.add(mm, cyl_between('steel', (-a, yy, -a), (-a, yy, a), 0.016, 8))
        for sz in (1, -1):
            m.add(mm, cyl_between('steel', (-a, ya + 0.04, sz * a), (a, ya + sec * 0.5, sz * a), 0.014, 8))
            m.add(mm, cyl_between('steel', (a, ya + sec * 0.5, sz * a), (-a, ya + sec - 0.02, sz * a), 0.014, 8))
        m.add(mm, cyl_between('steel', (-a, ya + 0.04, -a), (-a, ya + sec * 0.5, a), 0.014, 8))
        m.add(mm, box_span('steel_dark', -a - 0.05, a + 0.05, ya + 0.003, ya + 0.015, -a - 0.05, a + 0.05))   # section flange
    teeth = [(0.0, 0.0)]
    pitch = 0.0419
    k = 0
    y = 0.0
    while y + pitch <= sec * n:
        teeth += [(y + pitch * 0.2, 0.0), (y + pitch * 0.35, 0.025), (y + pitch * 0.65, 0.025), (y + pitch * 0.8, 0.0)]
        y += pitch
    teeth += [(sec * n, 0.0), (sec * n, -0.03), (0.0, -0.03)]
    rack = extrude('wear_steel', [(t, v) for v, t in teeth], 0.05)
    m.add(mm, rack.move(x=a + 0.07, y=y0, z=0.0))   # polygon is (depth, height): teeth face +X
    m.add(mm, box_span('steel_dark', a + 0.03, a + 0.065, y0 + 0.004, top - 0.004, -0.04, 0.04))
    m.add(mm, box_span('paint_orange', -0.30, 0.30, top, top + 0.06, -0.30, 0.30))       # mast cap / top limit
    # cage (rests on the buffers in the rest pose) with drive unit
    cy0 = 0.39
    cg = m.node('cage_lift', 'mast', (0.55, cy0, 0))
    x0, x1, hz, hh = 0.55, 3.05, 0.75, 2.20
    m.add(cg, box_span('steel', x0, x1, cy0, cy0 + 0.05, -hz, hz))
    for x in (x0, x1 - 0.06):
        for s in (1, -1):
            m.add(cg, box_span('paint_orange', x, x + 0.06, cy0 + 0.05, cy0 + hh, s * hz - (0.06 if s > 0 else 0), s * hz + (0 if s > 0 else 0.06)))
    for s in (1, -1):
        m.add(cg, box_span('paint_orange', x0, x1, cy0 + hh - 0.06, cy0 + hh, s * hz - (0.05 if s > 0 else 0), s * hz + (0 if s > 0 else 0.05)))
        m.add(cg, box_span('paint_orange', x0, x1, cy0 + 1.05, cy0 + 1.10, s * hz - (0.05 if s > 0 else 0), s * hz + (0 if s > 0 else 0.05)))
        m.add(cg, mesh_panel('steel_dark', x0 + 0.06, x1 - 0.06, cy0 + 0.05, cy0 + hh - 0.06, s * (hz - 0.02), pitch=0.10, frame=0.02))
    m.add(cg, box_span('paint_orange', x0, x1, cy0 + hh, cy0 + hh + 0.04, -hz, hz))       # roof
    m.add(cg, box_span('steel', x0 + 0.8, x0 + 1.4, cy0 + hh + 0.04, cy0 + hh + 0.06, -0.40, 0.40))  # roof hatch
    m.add(cg, mesh_panel('steel_dark', -hz + 0.06, hz - 0.06, cy0 + 0.05, cy0 + hh - 0.06, x1 - 0.03, pitch=0.10, frame=0.03,
                         frame_mat='paint_orange', axis='z'))   # landing gate
    m.add(cg, box_span('steel', x1 - 0.01, x1 + 0.05, cy0 + 1.0, cy0 + 1.12, hz - 0.30, hz - 0.12))   # gate latch
    m.add(cg, box_span('paint_orange', x0 - 0.02, x0 + 0.04, cy0 + 0.05, cy0 + hh - 0.06, -hz + 0.06, hz - 0.06))  # mast-side wall
    # drive unit: two motors with gearboxes driving pinions on the rack, guide rollers on the mast
    m.add(cg, box_span('paint_orange', a + 0.13, x0 - 0.02, cy0 + 0.55, cy0 + 1.85, -0.30, -0.26))   # drive frame
    m.add(cg, box_span('paint_orange', a + 0.13, x0 - 0.02, cy0 + 0.55, cy0 + 1.85, 0.46, 0.50))
    for k, y in enumerate((cy0 + 1.55, cy0 + 0.85)):
        m.add(cg, rbox('paint_orange', 0.12, 0.26, 0.20, 0.02, (a + 0.25, y, -0.16)))            # gearbox
        m.add(cg, cyl_between('chassis_black', (a + 0.25, y, 0.10), (a + 0.25, y, 0.44), 0.07, 20))   # motor
        m.add(cg, cyl_between('steel', (a + 0.16, y, -0.08), (a + 0.16, y, 0.04), 0.02, 10))
        m.add(cg, cylinder('wear_steel', 0.06, 0.05, 18).rotate('x', 90).at((a + 0.155, y, 0.0)))   # pinion on the rack
    for y in (cy0 + 0.30, cy0 + 2.00):
        for sz in (1, -1):
            m.add(cg, cylinder('chassis_black', 0.045, 0.04, 16).rotate('x', 90).at((a + 0.045, y, sz * (a + 0.07))))
            m.add(cg, box_span('paint_orange', a + 0.07, x0 - 0.02, y - 0.03, y + 0.03, sz * (a + 0.09) - 0.02, sz * (a + 0.09) + 0.02))
    m.add(cg, box_span('lamp_amber', x1 - 0.20, x1 - 0.10, cy0 + hh + 0.04, cy0 + hh + 0.12, hz - 0.15, hz - 0.05))
    return m


MODELS = [(light_tower, 'Construction/light-tower', 'light_tower'),
          (generator, 'Construction/diesel-generator', 'diesel_generator'),
          (fuel_tank, 'Construction/fuel-tank-1000l', 'fuel_tank_1000l'),
          (material_hoist, 'Construction/material-hoist', 'material_hoist')]

if __name__ == '__main__':
    for fn, folder, name in MODELS:
        save(fn(), folder, name)
