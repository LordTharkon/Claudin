"""Reversible mounted plough, 5 furrow, for the row-crop tractor's Cat III three-point hitch.

Key real-world dimensions used (metres):
  lower hitch pins 1.04 apart at 0.62 above the ground, top-link pin 0.56 above them
  (the row-crop tractor's ball ends and top-link end in its rest pose)
  furrow width 0.40 (16 in), body spacing along the beam 0.95, under-beam clearance 0.80
  turnover shaft 0.86 above the ground; overall ~6.6 long, ~2.4 wide, ~1.9 high

Rest pose: working position, standing on its shares and parking stand with the lower
bodies throwing right (+z). `beam_rotate` turns the beam and both body sets 180 deg about
the turnover shaft (x axis) to plough the other way.
"""
import math

from farm_parts import (BODY, FARM_MATERIALS, FRAME, WHEEL, coupling, hazard_board, hose, implement_tyre, place,
                        rear_light, reflector, save_farm, sheet)
from meshkit import Model, box, box_between, box_span, cyl_between, cylinder, sphere
from vehicle_parts import rim

YP, ZP, YT = 0.62, 0.52, 1.18       # lower pins height / half spacing, top-link pin height
SH = (-0.42, 0.86)                  # turnover shaft: headstock rear face x, height
YB = 0.86                           # beam centreline height
NB, DX, FW = 5, 0.95, 0.40          # bodies, spacing along x, furrow width
X0, Z0 = -1.55, -0.20               # first leg on the beam


def leg_pos(i):
    return (X0 - DX * i, Z0 + FW * i)


def body_parts(skim=True):
    """One plough body in local coordinates: origin on the beam centreline above the leg,
    ground at y = -YB, soil thrown to +z."""
    out = []
    g = -YB
    # leg (standard) from the beam clamp down to the frog, shear-bolt plates at the beam
    out.append(box_span(FRAME, -0.11, 0.11, -0.12, 0.12, -0.13, 0.13))
    out.append(box_span(FRAME, -0.07, 0.07, -0.12, 0.13, -0.14, -0.13))
    for x in (-0.07, 0.07):
        out.append(cyl_between('steel', (x, -0.10, 0.12), (x, -0.10, 0.155), 0.015, 8))
        out.append(cyl_between('steel', (x, 0.10, 0.12), (x, 0.10, 0.155), 0.015, 8))
    out.append(box_between(FRAME, (0.0, -0.10, 0.0), (-0.06, g + 0.30, 0.0), 0.05, 0.16, (1, 0, 0)))
    out.append(box_span('steel_dark', -0.40, 0.18, g + 0.10, g + 0.32, -0.04, 0.02))    # frog
    # landside bar along the furrow wall
    out.append(box_span('steel', -0.62, 0.20, g + 0.015, g + 0.14, -0.065, -0.04))
    out.append(box_span('wear_steel', -0.70, -0.60, g + 0.005, g + 0.12, -0.07, -0.036))  # heel
    # mouldboard: rows bottom -> top, columns shin -> tail
    rows = []
    for a in range(7):
        v = a / 6
        row = []
        for b in range(9):
            u = b / 8
            yb, yt = g + 0.07 + 0.10 * u * u, g + 0.36 + 0.22 * u      # lower edge rises to the wing, top to the tail
            y = yb + (yt - yb) * v
            x = 0.30 - 1.10 * u - 0.16 * v - 0.12 * v * u               # top edge sweeps back
            z = -0.035 + 0.38 * u ** 0.8 + 0.30 * (u * v) ** 1.3 + 0.02 * v
            z -= 0.05 * math.sin(math.pi * u) * math.sin(math.pi * v * 0.8)   # concave face
            row.append((x, y, z))
        rows.append(row)
    out.append(sheet('wear_steel', rows, 0.012))
    # share: wedge in front of the bottom of the mouldboard, point at the landside
    share = []
    for a in range(2):
        y = g + 0.01 + 0.075 * a
        row = []
        for b in range(5):
            u = b / 4
            x = 0.42 - 0.08 * a + (-0.30 - 0.05 * a - 0.42) * u
            z = -0.045 + (0.37 + 0.02 * a) * u
            row.append((x, y, z + 0.02 * a))
        share.append(row)
    out.append(sheet('steel', share, 0.014))
    out.append(box_between('wear_steel', (0.40, g + 0.03, -0.05), (0.23, g + 0.33, -0.03), 0.03, 0.04, (0, 0, 1)))  # shin
    # mouldboard stays from the frog to the back of the board
    out.append(cyl_between(FRAME, (-0.30, g + 0.28, 0.0), (-0.75, g + 0.45, 0.38), 0.018, 8))
    out.append(cyl_between(FRAME, (-0.20, g + 0.20, 0.0), (-0.55, g + 0.22, 0.30), 0.016, 8))
    if skim:
        # skim coulter on its own shank ahead of the leg
        sxp = 0.62
        out.append(box_span(FRAME, sxp - 0.06, sxp + 0.06, -0.12, 0.12, 0.13, 0.17))
        out.append(box_between(FRAME, (sxp, -0.10, 0.15), (sxp + 0.05, g + 0.45, 0.10), 0.04, 0.05, (1, 0, 0)))
        sk = []
        for a in range(3):
            v = a / 2
            y = g + 0.22 + 0.16 * v
            row = []
            for b in range(4):
                u = b / 3
                row.append((sxp + 0.20 - 0.10 * v - 0.36 * u, y + 0.02 * u * v, 0.08 + 0.18 * u * (0.6 + v)))
            sk.append(row)
        out.append(sheet('steel', sk, 0.008))
        out.append(box_between('wear_steel', (sxp + 0.22, g + 0.20, 0.06), (sxp - 0.20, g + 0.20, 0.30), 0.04, 0.012))
    return out


def lower_body(i):
    x, z = leg_pos(i)
    return [g.at((x, YB, z)) for g in body_parts()]


def upper_body(i):
    x, z = leg_pos(i)
    return [g.rotate('x', 180).at((x, YB, z)) for g in body_parts()]


def headstock(m):
    h = m.node('headstock')
    # side plates carrying the lower hitch pins and the top-link clevis
    for s in (1, -1):
        m.add(h, box_span(BODY, -0.20, 0.06, 0.48, 1.30, s * 0.30, s * 0.34))
        m.add(h, cyl_between('steel', (0.0, YP, s * 0.31), (0.0, YP, s * (ZP + 0.11)), 0.019, 12))      # Cat III pin 37 mm
        m.add(h, cyl_between('steel_dark', (0.0, YP, s * 0.34), (0.0, YP, s * 0.38), 0.05, 12))
        m.add(h, cyl_between('steel', (0.0, YP - 0.04, s * (ZP + 0.09)), (0.0, YP + 0.05, s * (ZP + 0.09)), 0.005, 6))
        m.add(h, box_span(FRAME, 0.02, 0.10, YT - 0.09, YT + 0.09, s * 0.05, s * 0.07))                 # top-link clevis
        m.add(h, box_between(FRAME, (-0.10, YP - 0.10, s * 0.32), (-0.10, YT + 0.05, s * 0.06), 0.05, 0.10, (1, 0, 0)))
    m.add(h, cyl_between('steel', (0.06, YT, -0.09), (0.06, YT, 0.09), 0.016, 10))
    m.add(h, box_span(BODY, -0.20, 0.02, YT + 0.06, YT + 0.14, -0.34, 0.34))
    m.add(h, box_span(FRAME, -0.20, 0.0, 0.48, 0.58, -0.30, 0.30))
    # turnover housing on the headstock rear face
    m.add(h, box_span(BODY, -0.30, -0.20, 0.55, 1.20, -0.26, 0.26))
    m.add(h, cyl_between(FRAME, (-0.30, YB, 0), (SH[0], YB, 0), 0.17, 24))
    m.add(h, cyl_between('steel_dark', (SH[0] + 0.02, YB, 0), (SH[0] - 0.02, YB, 0), 0.19, 24))
    # turnover cylinder anchor and hydraulic lines forward to the couplings
    m.add(h, box_span(FRAME, -0.30, -0.20, 1.20, 1.40, -0.40, -0.26))
    for k, (z, col) in enumerate(((-0.12, 'lamp_red'), (-0.04, 'lamp_amber'))):
        m.add(h, hose([(-0.30, 1.32, -0.33 + 0.02 * k), (-0.10, 1.38, z), (0.10, 1.32, z - 0.1), (0.30, 1.12, z - 0.15)], 0.010))
        m.add(h, coupling((0.33, 1.12, z - 0.15), col))
    # parking stand under the headstock
    m.add(h, box_span(FRAME, -0.26, -0.18, 0.08, 0.50, 0.18, 0.24))
    m.add(h, box_span('steel_dark', -0.32, -0.12, 0.0, 0.02, 0.11, 0.31))
    m.add(h, cyl_between('steel', (-0.22, 0.42, 0.14), (-0.22, 0.42, 0.28), 0.012, 8))
    # forward-facing warning boards on the hitch frame
    for s in (1, -1):
        m.add(h, hazard_board((0.075, 1.02, s * 0.22), 0.16, 0.30, facing=1, axis='x', stripes=3))


def beam(m):
    b = m.node('beam_rotate', 'headstock', (SH[0], YB, 0))
    # turnover head: shaft, flange and a short cross beam to the main beam
    m.add(b, cyl_between('steel_dark', (SH[0] - 0.02, YB, 0), (SH[0] - 0.12, YB, 0), 0.14, 20))
    m.add(b, box_span(BODY, SH[0] - 0.32, SH[0] - 0.12, YB - 0.22, YB + 0.22, -0.22, 0.22))
    m.add(b, box_between(BODY, (SH[0] - 0.25, YB, 0), (X0 + 0.40, YB, Z0), 0.20, 0.20))
    # crank arm for the turnover cylinder
    m.add(b, box_between(FRAME, (SH[0] - 0.22, YB, 0), (SH[0] - 0.22, YB + 0.34, 0.24), 0.06, 0.10, (1, 0, 0)))
    m.cylinder('turnover_cylinder', 'headstock', b, (-0.25, 1.30, -0.36), (SH[0] - 0.22, YB + 0.33, 0.24), 0.05, 0.03, 0.55,
               barrel=BODY)
    m.add('headstock', hose([(-0.30, 1.32, -0.33), (-0.32, 1.42, -0.30), (-0.28, 1.36, -0.22)], 0.009))
    # main beam: one diagonal 200 x 200 tube carrying the legs
    xe, ze = leg_pos(NB - 1)
    p0, p1 = (X0 + 0.45, YB, Z0 - FW * 0.45 / DX), (xe - 0.45, YB, ze + FW * 0.45 / DX)
    m.add(b, box_between(BODY, p0, p1, 0.20, 0.20))
    for i in range(NB):
        m.add(b, lower_body(i))
        m.add(b, upper_body(i))
    # depth / transport wheel on a swing arm behind the last body, on the landside
    wx, wz, wr = xe - 1.15, ze - 0.10, 0.36
    m.add(b, box_span(FRAME, xe - 0.60, xe - 0.40, YB - 0.14, YB + 0.14, ze + 0.06, ze + 0.26))
    arm = m.node('depth_wheel_arm_tilt', b, (xe - 0.50, YB, ze + 0.16))
    m.add(arm, cyl_between('steel_dark', (xe - 0.50, YB, ze + 0.02), (xe - 0.50, YB, ze + 0.30), 0.05, 12))
    m.add(arm, box_between(FRAME, (xe - 0.50, YB, ze + 0.16), (wx, wr, wz + 0.25), 0.08, 0.12, (0, 0, 1)))
    m.add(arm, cyl_between(FRAME, (wx, wr, wz + 0.25), (wx, wr, wz + 0.16), 0.05, 12))
    m.add(arm, box_between(FRAME, (xe - 0.50, YB + 0.10, ze + 0.16), (xe - 0.85, YB + 0.42, ze + 0.16), 0.05, 0.06, (0, 0, 1)))
    # depth adjuster: screw spindle with handle between the beam and the arm
    m.add(b, cyl_between('steel', (xe - 0.42, YB + 0.12, ze + 0.16), (xe - 0.82, YB + 0.42, ze + 0.16), 0.016, 8))
    m.add(b, cyl_between('steel_dark', (xe - 0.47, YB + 0.16, ze + 0.16), (xe - 0.62, YB + 0.27, ze + 0.16), 0.03, 10))
    m.add(b, cyl_between('steel', (xe - 0.42, YB + 0.12, ze + 0.16), (xe - 0.42, YB + 0.12, ze + 0.32), 0.01, 6))
    w = m.node('depth_wheel_spin', arm, (wx, wr, wz))
    m.add(w, place(implement_tyre(wr, 0.26, 0.20, ribs=4, segs=28) + rim(0.20, 0.26, 'steel', WHEEL, nuts=6), (wx, wr, wz), 1))
    # rear light board on the beam end, reflectors along the beam
    lx = xe - 0.62
    m.add(b, box_between(FRAME, (xe - 0.40, YB + 0.10, ze - 0.05), (lx, YB + 0.62, ze - 0.05), 0.05, 0.05, (0, 0, 1)))
    m.add(b, box_span(FRAME, lx - 0.04, lx, YB + 0.55, YB + 0.75, ze - 0.45, ze + 0.30))
    m.add(b, rear_light((lx - 0.06, YB + 0.65, ze - 0.28), -1))
    m.add(b, rear_light((lx - 0.06, YB + 0.65, ze + 0.15), -1))
    for i in (1, 3):
        x, z = leg_pos(i)
        xm, zm = x + DX / 2, z - FW / 2
        m.add(b, box_span(FRAME, xm - 0.05, xm + 0.05, YB - 0.05, YB + 0.05, zm + 0.10, zm + 0.13))
        m.add(b, reflector((xm, YB, zm + 0.13), 1, 'z', 'lamp_amber', 0.06))


def build():
    m = Model('Reversible Plough', FARM_MATERIALS)
    headstock(m)
    beam(m)
    return m


if __name__ == '__main__':
    save_farm(build(), 'reversible-plough', 40000)
