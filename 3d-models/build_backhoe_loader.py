"""Backhoe loader, modern 4WD class (~8 t), loader bucket carried, backhoe ready to dig.

Key real-world dimensions used (metres):
  wheelbase 2.20, front tyres 12.5/80-18 (0.96 dia), rear 16.9-28 (1.36 dia)
  loader bucket 2.30 wide, width over stabilisers ~2.5 (pads down)
  height to top of cab ~2.95, overall length ~7.7
"""
import math

from meshkit import (MATERIALS, Model, box, box_between, box_span, cyl_between, extrude,
                     hydraulic_cylinder, save, sphere)
from dozer_parts import cab, exhaust, lamp, precleaner
from vehicle_parts import (arch_polygon, dig_bucket, hose, loader_bucket, place_wheel,
                           steering_wheel, wheel)

AX_F, AX_R = 1.10, -1.10
RF, RR = 0.48, 0.68
ZF, ZR = 0.94, 0.96


def rot_pt(p, deg, origin):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (origin[0] + p[0] * c - p[1] * s, origin[1] + p[0] * s + p[1] * c)


def chassis(m):
    p = 'chassis'
    front = wheel(RF, 0.32, 0.229, 'r4', 'steel', lugs=18)
    rear = wheel(RR, 0.43, 0.356, 'r4', 'steel', lugs=22)
    for s, side in ((1, 'right'), (-1, 'left')):
        m.add('wheel_front_' + side, place_wheel(front, (AX_F, RF, s * ZF), s))
        m.add('wheel_rear_' + side, place_wheel(rear, (AX_R, RR, s * ZR), s))
    m.add(p, box_span('paint_dark', -1.80, 1.95, 0.46, 0.90, -0.42, 0.42))
    for ax, y, z in ((AX_F, RF, ZF), (AX_R, RR, ZR)):
        m.add(p, cyl_between('paint_dark', (ax, y, -z + 0.2), (ax, y, z - 0.2), 0.10 if ax > 0 else 0.13, 20))
        m.add(p, sphere('paint_dark', 0.24 if ax > 0 else 0.28, 20, 10).scale(1, 0.85, 1).at((ax, y, 0)))
    for s in (1, -1):  # steering knuckles / final drives
        m.add(p, cyl_between('paint_dark', (AX_F, RF, s * 0.62), (AX_F, RF, s * 0.80), 0.15, 20))
        m.add(p, cyl_between('paint_dark', (AX_R, RR, s * 0.60), (AX_R, RR, s * 0.76), 0.20, 20))
    # fenders
    for s in (1, -1):
        m.add(p, extrude('paint', arch_polygon(AX_F, RF, RF + 0.05, RF + 0.08, 15, 165), 0.38).move(z=s * ZF))
        m.add(p, extrude('paint', arch_polygon(AX_R, RR, RR + 0.05, RR + 0.09, 5, 175), 0.50).move(z=s * ZR))
        m.add(p, box_span('paint_dark', AX_F - 0.06, AX_F + 0.06, RF + 0.05, RF + 0.20, s * 0.70, s * 0.78))
        m.add(p, lamp((AX_R + 0.35, RR + 0.86, s * (ZR + 0.12)), 1, 0.8))

    # sloped engine hood with front grille
    m.add(p, extrude('paint', [(0.45, 0.85), (2.05, 0.85), (2.13, 0.95), (2.11, 1.38), (1.95, 1.48), (0.45, 1.72)], 0.80))
    m.add(p, extrude('paint', [(0.45, 1.70), (1.95, 1.46), (1.95, 1.50), (0.45, 1.74)], 0.62))  # crown
    m.add(p, box_span('grille', 2.11, 2.14, 0.98, 1.34, -0.32, 0.32))
    y = 1.02
    while y < 1.33:
        m.add(p, box_span('paint_dark', 2.13, 2.15, y, y + 0.02, -0.32, 0.32))
        y += 0.05
    for s in (1, -1):
        m.add(p, box_span('paint_dark', 1.0, 1.01, 0.9, 1.55, s * 0.399, s * 0.405))  # side panel seam
        m.add(p, box_span('grille', 1.15, 1.85, 1.0, 1.25, s * 0.40, s * 0.405))      # side vents
        m.add(p, lamp((2.02, 1.50, s * 0.30), 1, 0.9))
    m.add(p, box_span('counterweight', 1.95, 2.25, 0.46, 0.85, -0.45, 0.45))  # front weight / bumper
    m.add(p, exhaust(0.70, 1.66, 2.45, 0.26, 0.05))
    m.add(p, precleaner(0.70, 1.66, -0.24, 0.14))

    # cab
    m.add('cab', box_span('paint_dark', -1.45, 0.40, 0.85, 1.0, -0.70, 0.70))
    m.add('cab', cab(-1.45, 0.40, 1.00, 2.78, 0.70, door_side=-1))
    m.add('cab', steering_wheel((0.08, 1.78, 0), 0.18, 35))
    for s in (1, -1):  # entry steps
        for k, y in enumerate((0.45, 0.72)):
            m.add('cab', box_span('grille', -0.35, 0.05, y, y + 0.025, s * 0.70, s * (0.88 - 0.05 * k)))


def loader(m):
    p = 'loader'
    A, B, C, D = (0.52, 1.25), (1.35, 1.68), (2.45, 1.18), (2.80, 0.52)
    for s in (1, -1):
        z = s * 0.60
        m.add(p, box_span('paint', 0.42, 0.64, 0.85, 1.36, s * 0.44, s * 0.66))  # tower
        for a, b in ((A, B), (B, C), (C, D)):
            m.add(p, box_between('paint', (a[0], a[1], z), (b[0], b[1], z), 0.10, 0.20, (0, 0, 1) if False else (0, 1, 0)))
        for q in (A, B, C, D):
            m.add(p, cyl_between('paint' if q in (B, C) else 'steel_dark', (q[0], q[1], z - 0.06), (q[0], q[1], z + 0.06), 0.10 if q in (B, C) else 0.06, 18))
        m.add(p, hydraulic_cylinder((1.45, 0.82, s * 0.50), (1.75, 1.50, s * 0.53), 0.065, 0.04, extension=0.55))
        m.add(p, hydraulic_cylinder((2.05, 1.50, s * 0.30), (2.80, 0.98, s * 0.30), 0.06, 0.035, extension=0.55))
        m.add(p, box_span('paint', 1.95, 2.12, 1.38, 1.56, s * 0.26, s * 0.34))
    m.add(p, box_between('paint', (2.15, 1.38, -0.60), (2.15, 1.38, 0.60), 0.14, 0.16, (1, 0, 0)))
    bk = loader_bucket(2.30, 0.80, 0.72)
    m.add('loader_bucket', [g.at((D[0] - 0.05, D[1] - 0.16, 0)) for g in bk])
    for z in (-0.6, -0.3, 0.3, 0.6):
        m.add('loader_bucket', box_span('paint', D[0] - 0.12, D[0] + 0.02, D[1] - 0.10, D[1] + 0.52, z - 0.03, z + 0.03))


def backhoe(m):
    p = 'backhoe'
    # rear main frame and stabilisers (down)
    m.add(p, box_span('paint', -2.10, -1.75, 0.55, 1.32, -0.95, 0.95))
    for s in (1, -1):
        piv = (-1.98, 1.18, s * 0.92)
        pad = (-2.08, 0.07, s * 1.22)
        m.add(p, box_between('paint', piv, pad, 0.16, 0.20, (1, 0, 0)))
        m.add(p, hydraulic_cylinder((-1.92, 1.48, s * 0.70), (-2.04, 0.55, s * 1.10), 0.06, 0.035, extension=0.5))
        m.add(p, box_span('paint', -2.0, -1.84, 1.32, 1.56, s * 0.64, s * 0.76))
        m.add(p, box_span('steel_dark', -2.24, -1.92, 0.0, 0.05, s * 1.08, s * 1.36))
        for dx in (-0.1, 0.0, 0.1):
            m.add(p, box_span('steel_dark', -2.08 + dx - 0.01, -2.08 + dx + 0.01, -0.03, 0.0, s * 1.08, s * 1.36))
        m.add(p, box('lens_red', 0.02, 0.10, 0.16, (-2.11, 1.20, s * 0.55)))
    # swing frame (king post) and swing cylinders
    m.add(p, box_span('paint', -2.52, -2.12, 0.62, 1.32, -0.22, 0.22))
    for y in (0.70, 1.25):
        m.add(p, cyl_between('steel', (-2.20, y - 0.06, 0), (-2.20, y + 0.06, 0), 0.08, 16))
    for s in (1, -1):
        m.add(p, hydraulic_cylinder((-1.90, 0.95, s * 0.62), (-2.30, 0.95, s * 0.24), 0.06, 0.035, extension=0.5))

    S, K, T = (-2.42, 1.15), (-2.82, 2.78), (-3.65, 2.98)
    tail, D = (-3.55, 3.34), (-3.95, 0.95)
    m.add(p, box_between('paint', (*S, 0), (*K, 0), 0.22, 0.34, (1, 0, 0)))
    m.add(p, box_between('paint', (*K, 0), (*T, 0), 0.22, 0.30))
    m.add(p, cyl_between('paint', (*K, -0.11), (*K, 0.11), 0.17, 20))
    m.add(p, cyl_between('steel_dark', (*S, -0.14), (*S, 0.14), 0.07, 14))
    m.add(p, cyl_between('steel_dark', (*T, -0.14), (*T, 0.14), 0.07, 14))
    # dipper
    m.add(p, box_between('paint', (*tail, 0), (*D, 0), 0.17, 0.26, (1, 0, 0)))
    # boom cylinder (front side of boom) and crowd cylinder (top of boom)
    m.add(p, hydraulic_cylinder((-2.15, 0.82, 0), (-2.52, 2.36, 0), 0.075, 0.045, extension=0.55))
    m.add(p, box_span('paint', -2.66, -2.45, 2.28, 2.46, -0.12, 0.12))
    m.add(p, hydraulic_cylinder((-2.62, 3.05, 0), (-3.52, 3.40, 0), 0.07, 0.042, extension=0.6))
    m.add(p, box_span('paint', -2.80, -2.56, 2.88, 3.10, -0.12, 0.12))
    # bucket with linkage
    ang = -15
    link_pin = rot_pt((-0.13, 0.08), ang, D)
    m.add('backhoe_bucket', [g.rotate('z', ang).at((*D, 0)) for g in dig_bucket(0.61, 1.0)])
    dip_pt = (-3.99, 1.32)
    for s in (1, -1):
        m.add(p, box_between('steel_dark', (*dip_pt, s * 0.11), (*link_pin, s * 0.11), 0.03, 0.08, (1, 0, 0)))
    m.add(p, hydraulic_cylinder((-3.78, 2.62, 0), (link_pin[0], link_pin[1] + 0.02, 0), 0.06, 0.038, extension=0.55))
    m.add(p, box_span('paint', -3.86, -3.66, 2.55, 2.75, -0.10, 0.10))
    # hydraulic hoses along boom and dipper
    for s in (1, -1):
        m.add(p, hose([(-2.15, 1.10, s * 0.13), (-2.55, 2.60, s * 0.13), (-2.95, 3.10, s * 0.13), (-3.55, 3.18, s * 0.11),
                       (-3.75, 2.70, s * 0.10)]))
    m.add(p, lamp((-2.75, 2.70, 0.15), -1, 0.8))


def build():
    m = Model('Backhoe Loader', MATERIALS)
    chassis(m)
    loader(m)
    backhoe(m)
    return m


if __name__ == '__main__':
    save(build(), 'backhoe-loader', 'backhoe_loader')
