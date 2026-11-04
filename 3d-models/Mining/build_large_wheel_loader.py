"""Large mining wheel loader (articulated, Z-bar linkage), ~13 m3 rock bucket.

Key real-world dimensions used (metres):
  wheelbase 5.20 (hitch centred), tread 3.44, 45/65R45-class tyres (2.68 dia x 1.15 wide)
  13 m3 rock bucket 5.0 wide with 9 teeth, carried racked back ~0.4 m off the ground
  boom pin 3.55 high, cab floor 3.60, height to top of cab ~6.5
  overall length ~16 (bucket teeth to rear stair), width over bucket / decks ~5.0
"""
import math

from mining_parts import (MINING_MATERIALS, Model, beacon, box, box_between, box_span, cab,
                          cyl_between, cylinder, deck, exhaust, extrude, guardrail, hazard_stripes,
                          hose, lamp, louvres, precleaner, save, sight_gauge, sphere, stairs, tank_cap)
from vehicle_parts import _shell, arch_polygon, place_wheel, wheel

AX_F, AX_R = 2.60, -2.60
TR, TW, BEAD = 1.34, 1.15, 0.57
WY = 1.30
ZW = 1.72
HITCH_Y = 1.62
BP = (3.15, 3.55)        # boom (lift arm) pins on the loader tower
BKP = (6.70, 1.05)       # bucket pins at the arm tips
ARM_Z = 0.88
BCP = (4.85, 2.55)       # bell-crank pivot on the arm cross tube
BC_T = (4.45, 3.75)      # bell-crank top (tilt-cylinder rod eye)
BC_B = (5.10, 1.78)      # bell-crank bottom (tilt link)
BE = (BKP[0] + 0.15, BKP[1] + 0.95)  # bucket link ear
BUCKET_W = 5.00
DECK_Y = 3.60


def arm_y(x):
    return BP[1] + (BKP[1] - BP[1]) * (x - BP[0]) / (BKP[0] - BP[0])


def running_gear(m):
    w = wheel(TR, TW, BEAD, 'chevron', 'otr', lugs=24, depth=0.10)
    for ax, tag, par in ((AX_F, 'front', 'hitch_rotate'), (AX_R, 'rear', 'rear_axle_rotate')):
        for s, side in ((1, 'right'), (-1, 'left')):
            n = m.node('wheel_%s_%s_spin' % (tag, side), par, (ax, WY, s * ZW))
            m.add(n, place_wheel(w, (ax, WY, s * ZW), s))
        m.add(par, cyl_between('chassis_black', (ax, WY, -ZW + 0.55), (ax, WY, ZW - 0.55), 0.24, 24))
        m.add(par, sphere('chassis_black', 0.50, 22, 11).scale(1, 0.85, 0.9).at((ax, WY, 0)))
        for s in (1, -1):  # planetary final drives
            m.add(par, cyl_between('chassis_black', (ax, WY, s * (ZW - 0.80)), (ax, WY, s * (ZW - 0.52)), 0.42, 28))
            m.add(par, cyl_between('steel_dark', (ax, WY, s * (ZW - 0.95)), (ax, WY, s * (ZW - 0.80)), 0.30, 20))
    # rear axle trunnion (oscillation) mounts
    m.add('rear_axle_rotate', box_span('chassis_black', AX_R - 0.45, AX_R + 0.45, WY + 0.2, WY + 0.4, -0.45, 0.45))
    m.add('rear_frame', cyl_between('steel_dark', (AX_R - 0.70, WY + 0.30, 0), (AX_R + 0.70, WY + 0.30, 0), 0.14, 16))


def front_frame(m):
    p = 'hitch_rotate'
    m.add(p, box_span('paint_yellow', 0.55, 4.20, 0.95, 2.25, -0.55, 0.55))
    m.add(p, box_span('paint_yellow', 3.70, 4.30, 0.80, 2.40, -0.70, 0.70))      # nose
    m.add(p, box_span('steel_dark', 4.30, 4.40, 1.15, 1.45, -0.18, 0.18))        # tow block
    for y in (1.30, 1.86):   # hitch tongues
        m.add(p, box_span('paint_yellow', -0.25, 0.60, y, y + 0.14, -0.42, 0.42))
    for y0, y1 in ((1.24, 1.50), (1.80, 2.06)):
        m.add(p, cylinder('steel', 0.13, y1 - y0, 20).at((0.0, (y0 + y1) / 2, 0)))
    # loader tower: side plates up to the boom pins and a cross beam for the tilt cylinder
    for s in (1, -1):
        tower = [(1.85, 2.25), (3.65, 2.25), (3.55, 3.30), (3.35, 3.85), (2.95, 3.85), (2.20, 3.20)]
        m.add(p, extrude('paint_yellow', tower, 0.16).move(z=s * 0.66))
        m.add(p, cyl_between('steel_dark', (BP[0], BP[1], s * 0.55), (BP[0], BP[1], s * (ARM_Z + 0.19)), 0.13, 18))
        # lift-cylinder base lugs
        m.add(p, box_span('paint_yellow', 1.90, 2.30, 1.70, 2.25, s * 0.30, s * 0.56))
    m.add(p, box_span('paint_yellow', 2.30, 2.85, 2.85, 3.25, -0.58, 0.58))
    m.add(p, box_span('paint_yellow', 2.40, 2.75, 2.85, 3.25, -0.20, 0.20).move(x=-0.01))
    # front fenders with lamps
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', arch_polygon(AX_F, WY, TR + 0.10, TR + 0.15, 25, 165, 18), TW + 0.16).move(z=s * ZW))
        m.add(p, box_span('paint_yellow', AX_F - 0.35, AX_F + 0.35, 2.25, 2.70, s * 0.55, s * (ZW - 0.60)))
        m.add(p, lamp((AX_F + 1.05, WY + TR + 0.05, s * (ZW + 0.30)), 1))
        m.add(p, box('lamp_amber', 0.02, 0.08, 0.16, (4.405, 2.10, s * 0.55)))
    m.add(p, lamp((3.38, 4.0, 0.42), 1), lamp((3.38, 4.0, -0.42), 1))
    # hydraulic lines from the frame to the cylinders
    for s in (1, -1):
        m.add(p, hose([(0.7, 2.30, s * 0.40), (1.95, 2.40, s * 0.40), (2.35, 3.00, s * 0.30)], 0.035))
        m.add(p, hose([(0.7, 2.30, s * 0.26), (1.85, 2.30, s * 0.26), (2.05, 2.05, s * 0.36)], 0.035))


def rear_frame(m):
    p = 'rear_frame'
    m.add(p, box_span('paint_yellow', -6.30, -0.60, 1.35, 2.45, -0.70, 0.70))
    for y in (1.14, 1.58, 2.06):   # hitch clevis plates
        m.add(p, box_span('paint_yellow', -0.95, -0.08, y, y + 0.12, -0.55, 0.55))
    m.add(p, box_span('paint_yellow', -1.25, -0.95, 1.35, 2.30, -0.82, 0.82))   # steering-cylinder anchors
    m.add(p, box_span('paint_yellow', -2.30, -0.25, 2.45, DECK_Y - 0.06, -1.10, 1.10))  # under-cab structure
    # engine enclosure with chamfered top, doors and louvres
    hood = [(-1.25, 2.45), (1.25, 2.45), (1.25, 4.55), (1.05, 4.75), (-1.05, 4.75), (-1.25, 4.55)]
    m.add(p, extrude('paint_yellow', hood, 3.80).rotate('y', 90).move(x=-4.20))
    m.add(p, box_span('chassis_black', -5.90, -2.50, 4.745, 4.76, -0.55, 0.55))
    for s in (1, -1):
        m.add(p, louvres(-5.70, -3.00, DECK_Y + 0.15, 4.40, s * 1.25, s))
        for x in (-5.85, -4.35, -2.85):
            m.add(p, box_span('chassis_black', x, x + 0.015, DECK_Y + 0.05, 4.55, s * 1.249, s * 1.256))
        m.add(p, box_span('steel_dark', -3.05, -2.95, 3.90, 4.02, s * 1.25, s * 1.275))
    m.add(p, exhaust(-4.70, 4.75, 5.55, -0.40, r=0.13))
    m.add(p, exhaust(-4.70, 4.75, 5.55, 0.40, r=0.13))
    m.add(p, precleaner(-3.10, 4.75, 0.55, 1.3))
    m.add(p, precleaner(-3.10, 4.75, -0.55, 1.3))
    m.add(p, tank_cap(-5.70, 4.75, 0.0, 0.09))
    # rear radiator grille
    m.add(p, box_span('paint_yellow', -6.38, -6.10, 2.45, 4.75, -1.25, 1.25))
    m.add(p, box_span('steel_dark', -6.39, -6.36, 2.60, 4.60, -1.10, 1.10))
    y = 2.68
    while y < 4.56:
        m.add(p, box('chassis_black', 0.07, 0.03, 2.18, (-6.42, y, 0)).rotate('z', 20, (-6.42, y, 0)))
        y += 0.085
    # counterweight with lights, tow pin and hazard stripes
    cw = [(-6.30, 0.75), (-6.30, 2.95), (-6.80, 2.95), (-6.98, 2.70), (-6.98, 1.05), (-6.70, 0.75)]
    m.add(p, extrude('paint_yellow', cw, 3.10))
    m.add(p, box_span('steel_dark', -7.02, -6.96, 1.25, 1.55, -0.22, 0.22))
    m.add(p, cyl_between('steel', (-7.0, 1.20, 0), (-7.0, 1.60, 0), 0.06, 12))
    for s in (1, -1):
        m.add(p, box('lamp_red', 0.02, 0.14, 0.26, (-6.99, 2.40, s * 1.20)))
        m.add(p, box('lamp_amber', 0.02, 0.10, 0.20, (-6.99, 2.18, s * 1.20)))
        m.add(p, lamp((-6.92, 3.05, s * 1.25), -1))
        m.add(p, hazard_stripes((-6.98, 1.10, 0.30 if s > 0 else -1.50), (0, 0, 1), (0, 1, 0), 1.20, 0.45, 0.15))
    # rear fenders under the side decks
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', arch_polygon(AX_R, WY, TR + 0.12, TR + 0.17, 20, 160, 18), TW + 0.20).move(z=s * ZW))
        m.add(p, box_span('chassis_black', AX_R - 0.25, AX_R + 0.25, 2.30, DECK_Y - 0.06, s * 0.70, s * 1.10))
    # fuel (left) and hydraulic (right) tanks between the wheels
    m.add(p, box_span('paint_yellow', -1.20, -0.30, 1.30, 3.30, -2.05, -0.75))
    m.add(p, tank_cap(-0.75, 3.30, -1.75))
    m.add(p, sight_gauge(-0.55, 1.70, 3.00, -2.05, -1))
    m.add(p, box_span('paint_yellow', -1.20, -0.30, 1.30, 3.30, 0.75, 2.05))
    m.add(p, tank_cap(-0.75, 3.30, 1.75, 0.08))
    m.add(p, sight_gauge(-0.55, 1.90, 2.90, 2.05, 1))
    for s in (1, -1):
        m.add(p, hose([(-0.40, 1.45, s * 0.95), (-0.40, 1.45, s * 0.50), (0.20, 1.50, s * 0.30)], 0.04))

    # side decks: full length on the right, ahead of the stair on the left
    m.add(p, deck(-6.10, -0.25, 1.15, 2.40, DECK_Y))
    m.add(p, deck(-4.35, -0.25, -2.40, -1.15, DECK_Y))
    m.add(p, deck(-6.10, -4.35, -1.55, -1.15, DECK_Y))
    m.add(p, box_span('paint_yellow', -6.10, -0.25, DECK_Y - 0.28, DECK_Y - 0.05, 2.36, 2.40))
    m.add(p, box_span('paint_yellow', -4.35, -0.25, DECK_Y - 0.28, DECK_Y - 0.05, -2.40, -2.36))
    m.add(p, deck(-0.25, 0.25, -2.40, -1.15, DECK_Y))
    m.add(p, deck(-0.25, 0.25, 1.15, 2.40, DECK_Y))
    for s in (1, -1):
        for z in (1.18, 2.22):
            m.add(p, box_span('chassis_black', -0.18, -0.04, 2.45, DECK_Y - 0.06, s * z, s * (z + 0.14)))
        m.add(p, box_span('chassis_black', -5.60, -5.40, 2.45, DECK_Y - 0.06, s * 1.25, s * 1.60))
    m.add(p, guardrail([(-6.05, 2.37), (0.22, 2.37), (0.22, 1.18)], DECK_Y))
    m.add(p, guardrail([(-6.05, 2.37), (-6.05, 1.30)], DECK_Y))
    m.add(p, guardrail([(-4.40, -2.37), (0.22, -2.37), (0.22, -1.18)], DECK_Y))
    m.add(p, guardrail([(-6.05, -1.52), (-6.05, -1.30)], DECK_Y))
    # rear stair (left side), from behind the rear tyre up to the deck
    m.add(p, stairs(-6.95, 0.62, -4.42, DECK_Y, -1.98, width=0.72, rise=0.25))
    m.add(p, box_span('paint_yellow', -6.20, -6.05, 0.85, 1.40, -2.30, -1.66))
    m.add(p, cyl_between('paint_yellow', (-6.15, 1.0, -1.98), (-6.15, 1.0, -1.40), 0.05, 10))
    for s in (1, -1):
        m.add(p, lamp((-0.20, DECK_Y + 1.25, s * 2.37), 1))

    # operator cab
    m.add('cab', cab(-2.15, -0.25, DECK_Y, 6.00, 1.10, sill=0.55, door_side=-1, post=0.13, wheel=True))


def lift_arms(m):
    a = 'lift_arm_tilt'
    for s in (1, -1):
        z = s * ARM_Z
        dx, dy = BKP[0] - BP[0], BKP[1] - BP[1]
        L = math.hypot(dx, dy)
        nx, ny = -dy / L, dx / L
        prof = [(BP[0] - 0.30 * dx / L + 0.32 * nx, BP[1] - 0.30 * dy / L + 0.32 * ny),
                (BCP[0] + 0.40 * nx, BCP[1] + 0.40 * ny + 0.10),
                (BKP[0] + 0.25 * dx / L + 0.24 * nx, BKP[1] + 0.25 * dy / L + 0.24 * ny),
                (BKP[0] + 0.25 * dx / L - 0.24 * nx, BKP[1] + 0.25 * dy / L - 0.24 * ny),
                (BCP[0] - 0.36 * nx, BCP[1] - 0.36 * ny - 0.18),
                (BP[0] - 0.30 * dx / L - 0.32 * nx, BP[1] - 0.30 * dy / L - 0.32 * ny)]
        m.add(a, extrude('paint_yellow', prof, 0.22).move(z=z))
        for c, r in ((BP, 0.30), (BKP, 0.24)):
            m.add(a, cyl_between('paint_yellow', (c[0], c[1], z - 0.16), (c[0], c[1], z + 0.16), r, 24))
            m.add(a, cyl_between('steel_dark', (c[0], c[1], z - 0.23), (c[0], c[1], z + 0.23), r * 0.45, 12))
        # lift-cylinder lugs under the arm
        m.add(a, box_span('paint_yellow', 4.20, 4.60, arm_y(4.4) - 0.48, arm_y(4.4) - 0.10, s * 0.38, s * 0.78))
        m.cylinder('lift_cylinder_%s' % ('right' if s > 0 else 'left'), 'hitch_rotate', a,
                   (2.10, 1.95, s * 0.43), (4.40, arm_y(4.4) - 0.36, s * 0.43), 0.19, 0.12, extension=0.6)
    # cross tube carrying the bell crank
    m.add(a, cyl_between('paint_yellow', (BCP[0], BCP[1], -ARM_Z), (BCP[0], BCP[1], ARM_Z), 0.26, 24))
    for s in (1, -1):
        m.add(a, box_span('paint_yellow', BCP[0] - 0.30, BCP[0] + 0.30, BCP[1] - 0.05, BCP[1] + 0.40, s * 0.18, s * 0.40))
    # bell crank: two plates on a pivot through the cross tube
    b = 'bell_crank_tilt'
    for s in (1, -1):
        z = s * 0.13
        lever = [(BC_T[0] - 0.20, BC_T[1] + 0.05), (BC_T[0] + 0.18, BC_T[1] + 0.12), (BCP[0] + 0.30, BCP[1] + 0.10),
                 (BC_B[0] + 0.20, BC_B[1] - 0.05), (BC_B[0] - 0.12, BC_B[1] - 0.15), (BCP[0] - 0.30, BCP[1] - 0.05)]
        m.add(b, extrude('paint_yellow', lever, 0.08).move(z=z))
    for c in (BCP, BC_T, BC_B):
        m.add(b, cyl_between('steel_dark', (c[0], c[1], -0.22), (c[0], c[1], 0.22), 0.09, 12))
    m.cylinder('tilt_cylinder', 'hitch_rotate', b, (2.62, 3.05, 0), BC_T + (0,), 0.20, 0.13, extension=0.58)
    m.link('tilt_link_rotate', b, 'bucket_tilt', BC_B + (0,), BE + (0,),
           lambda p0, p1: [box_between('paint_yellow', p0, p1, 0.16, 0.30),
                           cyl_between('paint_yellow', (p1[0], p1[1], -0.10), (p1[0], p1[1], 0.10), 0.16, 16),
                           cyl_between('paint_yellow', (p0[0], p0[1], -0.10), (p0[0], p0[1], 0.10), 0.16, 16)])
    m.add(a, lamp((BCP[0] + 0.1, BCP[1] + 0.55, 0.55), 1, 0.8), lamp((BCP[0] + 0.1, BCP[1] + 0.55, -0.55), 1, 0.8))


def bucket(m):
    p = 'bucket_tilt'
    x0, y0 = BKP
    # inside profile from the cutting edge round to the top spill guard (local, pin at origin)
    curve = [(2.25, -0.70), (0.95, -0.72), (0.45, -0.60), (0.27, -0.25), (0.25, 0.55), (0.33, 1.25),
             (0.62, 1.52), (0.92, 1.58)]
    loc = lambda pts: [(x + x0, y + y0) for x, y in pts]
    m.add(p, extrude('paint_yellow', loc(_shell(curve, -0.06)), BUCKET_W - 0.10))
    side = curve + [(1.50, 1.10)]
    for s in (1, -1):
        m.add(p, extrude('paint_yellow', loc(side), 0.06).move(z=s * (BUCKET_W / 2 - 0.03)))
        m.add(p, extrude('wear_steel', loc([(2.25, -0.70), (1.20, -0.71), (1.25, -0.40), (2.05, -0.40)]), 0.07)
              .move(z=s * (BUCKET_W / 2 + 0.03)))    # side cutter
    m.add(p, extrude('wear_steel', loc([(1.55, -0.775), (2.32, -0.775), (2.32, -0.70), (1.55, -0.70)]), BUCKET_W - 0.04))
    # wear strips under the floor and back ribs
    for k in range(5):
        z = (k - 2) * 1.05
        m.add(p, extrude('wear_steel', loc([(0.55, -0.79), (1.50, -0.79), (1.50, -0.765), (0.55, -0.765)]), 0.18).move(z=z))
    for s in (1, -1):
        for z in (s * 0.45, s * 1.45):
            m.add(p, extrude('paint_yellow', loc([(0.30, -0.55), (0.10, -0.40), (0.10, 1.10), (0.40, 1.35), (0.28, 0.40)]), 0.08).move(z=z))
        # arm pin ears and link ear
        ear = [(-0.25, -0.30), (0.30, -0.45), (0.30, 0.40), (-0.10, 0.30)]
        for dz in (-0.17, 0.17):
            m.add(p, extrude('paint_yellow', loc(ear), 0.06).move(z=s * ARM_Z + dz))
    m.add(p, cyl_between('steel_dark', (x0, y0, -ARM_Z - 0.22), (x0, y0, ARM_Z + 0.22), 0.10, 14))
    lear = [(-0.05, 0.60), (0.30, 0.55), (0.30, 1.25), (0.10, 1.20)]
    for dz in (-0.14, 0.14):
        m.add(p, extrude('paint_yellow', loc(lear), 0.06).move(z=dz))
    m.add(p, cyl_between('steel_dark', (BE[0], BE[1], -0.20), (BE[0], BE[1], 0.20), 0.08, 12))
    # 9 teeth: adapters and tips, edge segments between
    n = 9
    for i in range(n):
        z = -BUCKET_W / 2 + 0.25 + (BUCKET_W - 0.5) * i / (n - 1)
        m.add(p, extrude('steel_dark', loc([(2.00, -0.79), (2.45, -0.75), (2.48, -0.68), (2.00, -0.62)]), 0.16).move(z=z))
        m.add(p, extrude('wear_steel', loc([(2.45, -0.76), (2.72, -0.735), (2.75, -0.715), (2.45, -0.665)]), 0.14).move(z=z))
        m.add(p, cyl_between('steel', (x0 + 2.30, y0 - 0.70, z - 0.085), (x0 + 2.30, y0 - 0.70, z + 0.085), 0.03, 6))
    for i in range(n - 1):
        z = -BUCKET_W / 2 + 0.25 + (BUCKET_W - 0.5) * (i + 0.5) / (n - 1)
        m.add(p, extrude('wear_steel', loc([(2.25, -0.765), (2.42, -0.755), (2.42, -0.715), (2.25, -0.70)]), 0.30).move(z=z))


def build():
    m = Model('Large Wheel Loader', MINING_MATERIALS)
    m.node('rear_frame')
    m.node('cab', 'rear_frame')
    m.node('rear_axle_rotate', 'rear_frame', (AX_R, WY + 0.30, 0))     # oscillating rear axle
    m.node('hitch_rotate', 'rear_frame', (0.0, HITCH_Y, 0.0))           # articulation hitch (front frame)
    m.node('lift_arm_tilt', 'hitch_rotate', (BP[0], BP[1], 0))
    m.node('bell_crank_tilt', 'lift_arm_tilt', (BCP[0], BCP[1], 0))
    m.node('bucket_tilt', 'lift_arm_tilt', (BKP[0], BKP[1], 0))
    running_gear(m)
    front_frame(m)
    rear_frame(m)
    lift_arms(m)
    bucket(m)
    for s in (1, -1):
        m.cylinder('steering_cylinder_%s' % ('right' if s > 0 else 'left'), 'rear_frame', 'hitch_rotate',
                   (-1.10, 1.80, s * 0.90), (1.05, 1.80, s * 0.62), 0.13, 0.08, extension=0.55)
        m.add('hitch_rotate', box_span('paint_yellow', 0.92, 1.18, 1.62, 1.98, s * 0.55, s * 0.70))
    m.pose('bucket_tilt', 'z', 6)    # racked back in the carry position
    return m


if __name__ == '__main__':
    save(build(), 'large-wheel-loader')
