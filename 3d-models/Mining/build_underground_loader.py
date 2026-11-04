"""Underground loader (LHD, load-haul-dump), ~14 t payload class, very low profile.

Key real-world dimensions used (metres):
  wheelbase 3.90 (hitch 0.45 ahead of mid-wheelbase), tread 2.10, 18.00R25-class tyres
  (1.60 dia x 0.50 wide), 6 m3 bucket 2.60 wide carried just off the floor
  side-mounted operator compartment on the left of the rear frame, ROPS/FOPS canopy at 2.40
  rear engine with an exhaust scrubber box on the hood, overall ~10.9 x 2.7 x 2.4
"""
from mining_parts import (MINING_MATERIALS, Model, beacon, box, box_span, cyl_between, cylinder,
                          extrude, hazard_stripes, hose, lamp, louvres, precleaner, save, sight_gauge,
                          tank_cap)
from vehicle_parts import _shell, arch_polygon, place_wheel, wheel

AX_F, AX_R = 1.95, -1.95
TR, TW, BEAD = 0.80, 0.50, 0.32
WY = TR
ZW = 1.08
HX, HY = 0.45, 0.90           # articulation hitch
BP = (1.05, 1.62)             # boom pins on the front frame tower
BKP = (3.55, 0.56)            # bucket pins at the boom tips
ARM_Z = 0.62
BUCKET_W = 2.60
HOOD_Y = 1.86
CAB = (-1.15, 0.38, -1.34, -0.42)   # x0, x1, z0, z1 of the operator compartment
FLOOR_Y, ROOF_Y = 0.72, 2.28


def arm_y(x):
    return BP[1] + (BKP[1] - BP[1]) * (x - BP[0]) / (BKP[0] - BP[0])


def along_x(poly, x0, x1, mat='paint_yellow'):
    """Extrude a (z, y) cross-section along x from x0 to x1."""
    return extrude(mat, poly, x1 - x0).rotate('y', 90).move(x=(x0 + x1) / 2)


def running_gear(m):
    w = wheel(TR, TW, BEAD, 'chevron', 'otr', lugs=20, depth=0.05)
    for ax, tag, par in ((AX_F, 'front', 'hitch_rotate'), (AX_R, 'rear', 'rear_axle_rotate')):
        for s, side in ((1, 'right'), (-1, 'left')):
            n = m.node('wheel_%s_%s_spin' % (tag, side), par, (ax, WY, s * ZW))
            m.add(n, place_wheel(w, (ax, WY, s * ZW), s))
        m.add(par, cyl_between('chassis_black', (ax, WY, -ZW + 0.40), (ax, WY, ZW - 0.40), 0.13, 20))
        m.add(par, sphere_diff(ax))
        for s in (1, -1):  # planetary hubs
            m.add(par, cyl_between('chassis_black', (ax, WY, s * (ZW - 0.42)), (ax, WY, s * (ZW - 0.26)), 0.24, 24))
    m.add('rear_axle_rotate', box_span('chassis_black', AX_R - 0.30, AX_R + 0.30, WY + 0.10, WY + 0.22, -0.30, 0.30))


def sphere_diff(ax):
    from mining_parts import sphere
    return sphere('chassis_black', 0.27, 18, 9).scale(1, 0.85, 0.9).at((ax, WY, 0))


def front_frame(m):
    p = 'hitch_rotate'
    m.add(p, box_span('chassis_black', 0.96, 2.94, 0.42, 1.17, -0.47, 0.47))
    m.add(p, box_span('chassis_black', 2.95, 3.10, 0.50, 0.95, -0.40, 0.40))      # bucket stop / bumper
    for y in (0.62, 1.08):   # hitch tongues
        m.add(p, box_span('paint_yellow', HX - 0.20, 1.00, y, y + 0.10, -0.32, 0.32))
    for y0, y1 in ((0.56, 0.78), (1.02, 1.24)):
        m.add(p, cylinder('steel', 0.09, y1 - y0, 18).at((HX, (y0 + y1) / 2, 0)))
    # boom tower plates up to the boom pins
    for s in (1, -1):
        tower = [(0.62, 1.18), (1.70, 1.18), (1.40, 1.70), (1.20, 1.86), (0.90, 1.86), (0.62, 1.55)]
        m.add(p, extrude('paint_yellow', tower, 0.10).move(z=s * 0.46))
        m.add(p, cyl_between('steel_dark', (BP[0], BP[1], s * 0.40), (BP[0], BP[1], s * (ARM_Z + 0.12)), 0.075, 14))
        # lift-cylinder base lugs low on the frame
        m.add(p, box_span('paint_yellow', 1.30, 1.62, 0.55, 0.92, s * 0.48, s * 0.56))
    # front fenders with lamps and hazard stripes on the leading faces
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', arch_polygon(AX_F, WY, TR + 0.06, TR + 0.10, 20, 160, 16), TW + 0.10).move(z=s * ZW))
        m.add(p, box_span('paint_yellow', AX_F - 0.25, AX_F + 0.25, 1.18, 1.30, s * 0.48, s * (ZW - 0.30)))
        m.add(p, box_span('paint_yellow', AX_F + 0.72, AX_F + 0.86, 1.05, 1.50, s * (ZW - 0.24), s * (ZW + 0.18)))
        zt = max(s * (ZW - 0.24), s * (ZW + 0.18)) - 0.02
        m.add(p, hazard_stripes((AX_F + 0.86, 1.07, zt), (0, 0, -1), (0, 1, 0), 0.38, 0.41, 0.10))
        m.add(p, lamp((AX_F + 0.80, 1.62, s * (ZW - 0.05)), 1, 0.85))
        m.add(p, box('lamp_amber', 0.02, 0.06, 0.12, (AX_F + 0.87, 1.40, s * (ZW - 0.05))))
    # hydraulic lines up the tower
    for s in (1, -1):
        m.add(p, hose([(0.60, 1.20, s * 0.30), (1.25, 1.25, s * 0.30), (1.60, 1.05, s * 0.40)], 0.025))
        m.add(p, hose([(0.60, 1.20, s * 0.18), (1.05, 1.80, s * 0.18), (1.95, 1.40, s * 0.12)], 0.025))


def rear_frame(m):
    p = 'rear_frame'
    m.add(p, box_span('chassis_black', -5.54, -0.21, 0.42, 1.17, -0.41, 0.41))
    for y in (0.48, 0.86, 1.30):   # hitch clevis plates
        m.add(p, box_span('paint_yellow', -0.55, HX + 0.22, y, y + 0.10, -0.38, 0.38))
    m.add(p, box_span('paint_yellow', -0.70, -0.55, 0.45, 1.42, -0.55, 0.55))
    # engine enclosure (chamfered), louvres, doors
    hood = [(-1.18, 0.95), (1.18, 0.95), (1.18, HOOD_Y - 0.15), (1.02, HOOD_Y), (-1.02, HOOD_Y), (-1.18, HOOD_Y - 0.15)]
    m.add(p, along_x(hood, -5.25, -2.85))
    m.add(p, box_span('chassis_black', -5.10, -3.00, HOOD_Y - 0.005, HOOD_Y + 0.006, -0.40, 0.40))
    for s in (1, -1):
        m.add(p, louvres(-4.90, -3.20, 1.10, 1.62, s * 1.18, s))
        for x in (-5.05, -4.05, -3.05):
            m.add(p, box_span('chassis_black', x, x + 0.012, 1.00, HOOD_Y - 0.17, s * 1.179, s * 1.186))
        m.add(p, box_span('steel_dark', -3.18, -3.10, 1.40, 1.48, s * 1.18, s * 1.20))
    # rear fenders between engine and hitch
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', arch_polygon(AX_R, WY, TR + 0.06, TR + 0.10, 20, 160, 16), TW + 0.10).move(z=s * ZW))
    m.add(p, box_span('paint_yellow', -2.85, -1.20, 1.68, 1.76, 0.42, 1.30))     # right fender deck
    # exhaust scrubber box on the hood: inlet from the turbo, outlet pipe aft
    m.add(p, box_span('steel', -4.55, -3.35, HOOD_Y, HOOD_Y + 0.38, 0.25, 0.95))
    for x in (-4.50, -3.40):
        m.add(p, box_span('steel_dark', x - 0.02, x + 0.02, HOOD_Y + 0.38, HOOD_Y + 0.40, 0.25, 0.95))
    for x in (-4.30, -3.90, -3.60):
        m.add(p, cylinder('steel_dark', 0.02, 0.03, 6).at((x, HOOD_Y + 0.405, 0.30)))
    m.add(p, cyl_between('steel_dark', (-3.25, HOOD_Y + 0.19, 0.60), (-3.35, HOOD_Y + 0.19, 0.60), 0.09, 16))
    m.add(p, cyl_between('steel_dark', (-3.25, HOOD_Y + 0.01, 0.60), (-3.25, HOOD_Y + 0.19, 0.60), 0.09, 16))
    m.add(p, cyl_between('steel_dark', (-4.55, HOOD_Y + 0.19, 0.60), (-5.10, HOOD_Y + 0.19, 0.60), 0.07, 16))
    m.add(p, cyl_between('steel_dark', (-5.10, HOOD_Y + 0.19, 0.60), (-5.30, HOOD_Y + 0.05, 0.60), 0.07, 16))
    m.add(p, box_span('steel_dark', -4.10, -3.80, HOOD_Y + 0.10, HOOD_Y + 0.28, 0.95, 0.97))  # drain / sample port
    m.add(p, precleaner(-3.10, HOOD_Y, -0.55, 0.8))
    m.add(p, tank_cap(-4.80, HOOD_Y, -0.55, 0.07))
    # rear radiator grille
    m.add(p, box_span('paint_yellow', -5.55, -5.25, 0.95, HOOD_Y - 0.05, -1.10, 1.10))
    m.add(p, box_span('steel_dark', -5.56, -5.545, 1.05, HOOD_Y - 0.15, -0.95, 0.95))
    y = 1.12
    while y < HOOD_Y - 0.18:
        m.add(p, box('chassis_black', 0.05, 0.02, 1.86, (-5.58, y, 0)).rotate('z', 20, (-5.58, y, 0)))
        y += 0.07
    # rear bumper / counterweight with lights, tow hitch and hazard stripes
    m.add(p, box_span('paint_yellow', -5.88, -5.55, 0.40, 0.95, -1.20, 1.20))
    m.add(p, box_span('steel_dark', -5.98, -5.88, 0.55, 0.78, -0.18, 0.18))
    m.add(p, cyl_between('steel', (-5.94, 0.50, 0), (-5.94, 0.84, 0), 0.04, 12))
    for s in (1, -1):
        m.add(p, hazard_stripes((-5.88, 0.43, 0.25 if s > 0 else -1.15), (0, 0, 1), (0, 1, 0), 0.90, 0.48, 0.12))
        m.add(p, box('lamp_red', 0.02, 0.10, 0.18, (-5.565, 1.30, s * 1.02)))
        m.add(p, box('lamp_amber', 0.02, 0.08, 0.14, (-5.565, 1.16, s * 1.02)))
        m.add(p, lamp((-5.62, HOOD_Y + 0.10, s * 0.90), -1, 0.85))
    m.add(p, box('lamp_red', 0.02, 0.06, 0.40, (-5.565, HOOD_Y - 0.10, 0)))
    # hydraulic tank (right, beside the operator) and fuel tank (right, behind)
    m.add(p, box_span('paint_yellow', -1.10, 0.10, 0.45, 1.52, 0.48, 1.25))
    m.add(p, tank_cap(-0.40, 1.52, 0.90, 0.07))
    m.add(p, sight_gauge(-0.80, 0.70, 1.30, 1.25, 1))
    m.add(p, box_span('chassis_black', -2.85, -2.75, 0.45, 1.55, -1.25, 1.25))
    for s in (1, -1):
        m.add(p, hose([(0.0, 0.70, s * 0.45), (0.25, 0.75, s * 0.30), (0.62, 0.80, s * 0.22)], 0.03))
    # fire-suppression bottle and isolator box behind the cab
    m.add(p, cyl_between('paint_red', (-1.75, 1.75, -1.05), (-1.75, 2.15, -1.05), 0.09, 14))
    m.add(p, cylinder('steel_dark', 0.04, 0.06, 10).at((-1.75, 2.18, -1.05)))
    m.add(p, box_span('paint_yellow', -2.80, -1.20, 1.68, 1.75, -1.30, -0.42))
    m.add(p, box_span('chassis_black', -2.30, -2.00, 1.75, 1.98, -0.70, -0.45))
    m.add(p, box('paint_red', 0.04, 0.05, 0.05, (-2.31, 1.88, -0.58)))


def operator_compartment(m):
    p = 'cab'
    x0, x1, z0, z1 = CAB
    zc = (z0 + z1) / 2
    m.add(p, box_span('paint_yellow', x0, x1, 0.42, FLOOR_Y, z0, z1))                      # tub
    m.add(p, box_span('chassis_black', x0 + 0.04, x1 - 0.04, FLOOR_Y + 0.003, FLOOR_Y + 0.02, z0 + 0.04, z1 - 0.04))
    # low side walls (inner side full height to the dash)
    m.add(p, box_span('paint_yellow', x0, x1, FLOOR_Y, FLOOR_Y + 0.55, z1 - 0.05, z1))
    m.add(p, box_span('paint_yellow', x0, x0 + 0.05, FLOOR_Y, FLOOR_Y + 0.45, z0, z1 - 0.05))
    m.add(p, box_span('paint_yellow', x1 - 0.05, x1, FLOOR_Y, FLOOR_Y + 0.45, z0, z1 - 0.05))
    # ROPS/FOPS canopy: four posts, roof plate with FOPS bars
    for x in (x0 + 0.06, x1 - 0.06):
        for z in (z0 + 0.06, z1 - 0.06):
            m.add(p, box('paint_yellow', 0.10, ROOF_Y - FLOOR_Y, 0.10, (x, (FLOOR_Y + ROOF_Y) / 2, z)))
    m.add(p, box_span('paint_yellow', x0 - 0.04, x1 + 0.04, ROOF_Y, ROOF_Y + 0.07, z0 - 0.04, z1 + 0.04))
    for i in range(4):
        x = x0 + 0.20 + i * (x1 - x0 - 0.40) / 3
        m.add(p, box_span('paint_yellow', x - 0.03, x + 0.03, ROOF_Y + 0.07, ROOF_Y + 0.11, z0, z1))
    m.add(p, box_span('chassis_black', x0 - 0.04, x1 + 0.04, ROOF_Y - 0.02, ROOF_Y - 0.002, z0 - 0.04, z1 + 0.04))
    # glazing at the ends and inner side, mesh guard window over the outer side
    gy0, gy1 = FLOOR_Y + 0.47, ROOF_Y - 0.04
    m.add(p, box_span('glass', x1 - 0.035, x1 - 0.025, gy0, gy1, z0 + 0.11, z1 - 0.11))
    m.add(p, box_span('glass', x0 + 0.025, x0 + 0.035, gy0, gy1, z0 + 0.11, z1 - 0.11))
    m.add(p, box_span('glass', x0 + 0.11, x1 - 0.11, FLOOR_Y + 0.57, gy1, z1 - 0.035, z1 - 0.025))
    for i in range(5):
        y = FLOOR_Y + 0.95 + i * 0.12
        m.add(p, box_span('steel_dark', x0 + 0.11, x1 - 0.11, y - 0.012, y + 0.012, z0 + 0.02, z0 + 0.04))
    for i in range(1, 6):
        x = x0 + 0.11 + i * (x1 - x0 - 0.22) / 6
        m.add(p, box_span('steel_dark', x - 0.012, x + 0.012, FLOOR_Y + 0.88, ROOF_Y - 0.02, z0 + 0.04, z0 + 0.06))
    # transverse seat facing +z, joystick consoles fore and aft, dash
    sx = (x0 + x1) / 2
    m.add(p, box_span('chassis_black', sx - 0.20, sx + 0.20, FLOOR_Y + 0.02, FLOOR_Y + 0.30, z0 + 0.18, z0 + 0.52))
    m.add(p, box_span('seat_fabric', sx - 0.25, sx + 0.25, FLOOR_Y + 0.30, FLOOR_Y + 0.40, z0 + 0.14, z0 + 0.60))
    m.add(p, box('seat_fabric', 0.48, 0.58, 0.10, (sx, FLOOR_Y + 0.72, z0 + 0.13)).rotate('x', -8, (sx, FLOOR_Y + 0.40, z0 + 0.13)))
    m.add(p, box('seat_fabric', 0.28, 0.14, 0.08, (sx, FLOOR_Y + 1.10, z0 + 0.10)))
    for s in (1, -1):
        cx = sx + s * 0.40
        m.add(p, box_span('chassis_black', cx - 0.08, cx + 0.08, FLOOR_Y + 0.02, FLOOR_Y + 0.52, z0 + 0.20, z0 + 0.55))
        m.add(p, cyl_between('steel_dark', (cx, FLOOR_Y + 0.52, z0 + 0.48), (cx, FLOOR_Y + 0.64, z0 + 0.48), 0.012, 8))
        from mining_parts import sphere
        m.add(p, sphere('plastic_black', 0.032, 10, 6).at((cx, FLOOR_Y + 0.67, z0 + 0.48)))
    m.add(p, box_span('plastic_black', x0 + 0.20, x1 - 0.20, FLOOR_Y + 0.55, FLOOR_Y + 0.60, z1 - 0.20, z1 - 0.05))
    m.add(p, box('plastic_black', 0.30, 0.18, 0.02, (sx, FLOOR_Y + 0.70, z1 - 0.10)).rotate('x', 20, (sx, FLOOR_Y + 0.60, z1 - 0.10)))
    for s in (1, -1):   # pedals
        m.add(p, box('chassis_black', 0.08, 0.18, 0.06, (sx + s * 0.12, FLOOR_Y + 0.12, z1 - 0.20)).rotate('x', 30, (sx, FLOOR_Y, z1 - 0.20)))
    # step and grab handles on the outer side
    m.add(p, box_span('steel_dark', x0 + 0.30, x1 - 0.30, 0.38, 0.42, z0 - 0.22, z0 - 0.005))
    for x in (x0 + 0.30, x1 - 0.30):
        m.add(p, box_span('paint_yellow', x - 0.02, x + 0.02, 0.36, 0.45, z0 - 0.21, z0))
    for x in (x0 + 0.15, x1 - 0.15):
        m.add(p, cyl_between('chrome', (x, FLOOR_Y + 0.40, z0 - 0.02), (x, FLOOR_Y + 0.40, z0 - 0.09), 0.016, 8))
        m.add(p, cyl_between('chrome', (x, FLOOR_Y + 0.40, z0 - 0.09), (x, FLOOR_Y + 1.20, z0 - 0.09), 0.016, 8))
        m.add(p, cyl_between('chrome', (x, FLOOR_Y + 1.20, z0 - 0.09), (x, FLOOR_Y + 1.20, z0 - 0.02), 0.016, 8))
    # canopy lights (front and rear) and beacon
    m.add(p, lamp((x1 + 0.08, ROOF_Y - 0.06, zc), 1, 0.7))
    m.add(p, lamp((x0 - 0.08, ROOF_Y - 0.06, zc), -1, 0.7))
    m.add(p, beacon((x0 + 0.39, ROOF_Y + 0.07, zc), 0.05))
    m.add(p, box('glass', 0.01, 0.12, 0.18, (x1 + 0.10, ROOF_Y - 0.25, z0 - 0.06)))
    m.add(p, cyl_between('steel_dark', (x1 - 0.05, ROOF_Y - 0.20, z0), (x1 + 0.09, ROOF_Y - 0.25, z0 - 0.06), 0.01, 6))


def boom(m):
    a = 'boom_tilt'
    for s in (1, -1):
        z = s * ARM_Z
        prof = [(BP[0] - 0.20, BP[1] + 0.12), (2.40, arm_y(2.40) + 0.22), (BKP[0] + 0.15, BKP[1] + 0.16),
                (BKP[0] + 0.15, BKP[1] - 0.16), (2.40, arm_y(2.40) - 0.24), (BP[0] - 0.20, BP[1] - 0.16)]
        m.add(a, extrude('paint_yellow', prof, 0.12).move(z=z))
        for c, r in ((BP, 0.17), (BKP, 0.15)):
            m.add(a, cyl_between('paint_yellow', (c[0], c[1], z - 0.09), (c[0], c[1], z + 0.09), r, 20))
            m.add(a, cyl_between('steel_dark', (c[0], c[1], z - 0.135), (c[0], c[1], z + 0.135), r * 0.45, 12))
        # lift-cylinder lugs under the arm
        m.add(a, box_span('paint_yellow', 2.38, 2.62, arm_y(2.5) - 0.38, arm_y(2.5) - 0.12, s * 0.50, s * 0.56))
        m.cylinder('boom_lift_cylinder_%s' % ('right' if s > 0 else 'left'), 'hitch_rotate', a,
                   (1.46, 0.74, s * 0.52), (2.50, arm_y(2.5) - 0.28, s * 0.52), 0.11, 0.07, extension=0.6)
    # cross tube with the tilt-cylinder bracket
    m.add(a, cyl_between('paint_yellow', (2.30, arm_y(2.30), -ARM_Z), (2.30, arm_y(2.30), ARM_Z), 0.14, 20))
    for dz in (-0.12, 0.12):
        m.add(a, extrude('paint_yellow', [(2.10, arm_y(2.1) + 0.05), (2.50, arm_y(2.5)), (2.40, 1.38), (2.22, 1.40)], 0.05).move(z=dz))
    m.cylinder('bucket_tilt_cylinder', a, 'bucket_tilt', (2.32, 1.30, 0), (BKP[0] + 0.12, BKP[1] + 0.78, 0), 0.12, 0.08, extension=0.6)
    m.add(a, lamp((2.30, 1.20, 0.36), 1, 0.6), lamp((2.30, 1.20, -0.36), 1, 0.6))


def bucket(m):
    p = 'bucket_tilt'
    x0, y0 = BKP
    curve = [(1.40, -0.48), (0.55, -0.50), (0.30, -0.40), (0.18, -0.10), (0.16, 0.55), (0.24, 0.85),
             (0.48, 0.98), (0.70, 1.00)]
    loc = lambda pts: [(x + x0, y + y0) for x, y in pts]
    m.add(p, extrude('paint_yellow', loc(_shell(curve, -0.04)), BUCKET_W - 0.06))
    side = curve + [(1.05, 0.65)]
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', loc(side), 0.04).move(z=s * (BUCKET_W / 2 - 0.02)))
        m.add(p, extrude('wear_steel', loc([(1.40, -0.48), (0.80, -0.49), (0.82, -0.28), (1.28, -0.28)]), 0.04)
              .move(z=s * (BUCKET_W / 2 + 0.02)))
        for z in (s * 0.30, s * 0.95):
            m.add(p, extrude('paint_yellow', loc([(0.20, -0.36), (0.05, -0.25), (0.05, 0.75), (0.25, 0.90), (0.15, 0.30)]), 0.05).move(z=z))
        for dz in (-0.10, 0.10):   # boom pin ears
            m.add(p, extrude('paint_yellow', loc([(-0.14, -0.18), (0.18, -0.30), (0.18, 0.25), (-0.06, 0.18)]), 0.04)
                  .move(z=s * ARM_Z + dz))
    m.add(p, cyl_between('steel_dark', (x0, y0, -ARM_Z - 0.13), (x0, y0, ARM_Z + 0.13), 0.06, 12))
    for dz in (-0.09, 0.09):     # tilt-cylinder ear
        m.add(p, extrude('paint_yellow', loc([(0.02, 0.55), (0.20, 0.50), (0.24, 0.95), (0.06, 0.92)]), 0.04).move(z=dz))
    m.add(p, cyl_between('steel_dark', (x0 + 0.12, y0 + 0.78, -0.13), (x0 + 0.12, y0 + 0.78, 0.13), 0.045, 10))
    # bolt-on cutting edge (LHDs run a flat edge, no teeth) and heel wear strips
    m.add(p, extrude('wear_steel', loc([(1.05, -0.535), (1.48, -0.535), (1.48, -0.48), (1.05, -0.48)]), BUCKET_W - 0.02))
    for i in range(8):
        z = -BUCKET_W / 2 + 0.20 + (BUCKET_W - 0.40) * i / 7
        m.add(p, cyl_between('steel', (x0 + 1.25, y0 - 0.475, z), (x0 + 1.25, y0 - 0.465, z), 0.025, 6))
    for k in range(4):
        z = (k - 1.5) * 0.70
        m.add(p, extrude('wear_steel', loc([(0.40, -0.545), (1.00, -0.545), (1.00, -0.515), (0.40, -0.515)]), 0.14).move(z=z))

def build():
    m = Model('Underground Loader', MINING_MATERIALS)
    m.node('rear_frame')
    m.node('cab', 'rear_frame')
    m.node('rear_axle_rotate', 'rear_frame', (AX_R, WY + 0.15, 0))     # oscillating rear axle
    m.node('hitch_rotate', 'rear_frame', (HX, HY, 0.0))                # articulation hitch (front frame)
    m.node('boom_tilt', 'hitch_rotate', (BP[0], BP[1], 0))
    m.node('bucket_tilt', 'boom_tilt', (BKP[0], BKP[1], 0))
    running_gear(m)
    front_frame(m)
    rear_frame(m)
    operator_compartment(m)
    boom(m)
    bucket(m)
    for s in (1, -1):
        m.cylinder('steering_cylinder_%s' % ('right' if s > 0 else 'left'), 'rear_frame', 'hitch_rotate',
                   (-0.50, 1.00, s * 0.48), (1.20, 1.00, s * 0.42), 0.085, 0.05, extension=0.55)
        m.add('hitch_rotate', box_span('paint_yellow', 1.10, 1.30, 0.90, 1.18, s * 0.34, s * 0.48))
    return m


if __name__ == '__main__':
    m = build()
    save(m, 'underground-loader')
    print('tris', m.stats()[1])
