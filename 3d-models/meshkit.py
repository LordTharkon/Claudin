"""Tiny dependency-free mesh toolkit used to build the 3D models in this folder.

Conventions (match glTF 2.0):
  * units are metres, so every model is built at true 1:1 scale
  * +Y is up, +X is the machine's forward direction, Z is lateral
Geometry is kept as `Geo` chunks (one material each), grouped into named parts
inside a `Model`, which can be written out as .glb and .obj/.mtl.
"""
import json
import math
import struct

# ---------------------------------------------------------------- vectors ---

def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def length(a): return math.sqrt(dot(a, a))
def norm(a):
    l = length(a)
    return (0.0, 1.0, 0.0) if l < 1e-12 else (a[0] / l, a[1] / l, a[2] / l)
def lerp(a, b, t): return add(a, mul(sub(b, a), t))

# --------------------------------------------------------------- matrices ---
# 3x3 matrices are tuples of rows; p' = M p.

IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))

def mat_vec(m, v):
    return (dot(m[0], v), dot(m[1], v), dot(m[2], v))

def mat_mul(a, b):
    bt = tuple(zip(*b))
    return tuple(tuple(dot(r, c) for c in bt) for r in a)

def mat_det(m):
    return dot(m[0], cross(m[1], m[2]))

def mat_inv_t(m):
    """Inverse-transpose, used to carry normals through a transform."""
    d = mat_det(m)
    c0, c1, c2 = cross(m[1], m[2]), cross(m[2], m[0]), cross(m[0], m[1])
    return (mul(c0, 1 / d), mul(c1, 1 / d), mul(c2, 1 / d))

def from_columns(x, y, z):
    return ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))

def rot(axis, deg):
    if isinstance(axis, str):
        axis = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}[axis]
    x, y, z = norm(axis)
    a = math.radians(deg)
    c, s, t = math.cos(a), math.sin(a), 1 - math.cos(a)
    return ((t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c))

# -------------------------------------------------------------------- geo ---

class Geo:
    """Triangle soup with per-vertex normals and a single material name."""
    __slots__ = ('mat', 'pos', 'nrm', 'idx')

    def __init__(self, mat, pos, nrm, idx, fix=True):
        self.mat, self.pos, self.nrm, self.idx = mat, pos, nrm, idx
        if fix:
            self._orient()

    def _orient(self):
        # Wind every triangle so its face normal agrees with its vertex normals.
        p, n, out = self.pos, self.nrm, []
        for a, b, c in self.idx:
            fn = cross(sub(p[b], p[a]), sub(p[c], p[a]))
            if dot(fn, add(add(n[a], n[b]), n[c])) < 0:
                out.append((a, c, b))
            else:
                out.append((a, b, c))
        self.idx = out

    def transform(self, m=IDENTITY, t=(0, 0, 0)):
        nm = mat_inv_t(m)
        pos = [add(mat_vec(m, p), t) for p in self.pos]
        nrm = [norm(mat_vec(nm, v)) for v in self.nrm]
        idx = self.idx if mat_det(m) > 0 else [(a, c, b) for a, b, c in self.idx]
        return Geo(self.mat, pos, nrm, idx, fix=False)

    def move(self, x=0.0, y=0.0, z=0.0):
        return self.transform(IDENTITY, (x, y, z))

    def at(self, p):
        return self.transform(IDENTITY, p)

    def rotate(self, axis, deg, pivot=(0, 0, 0)):
        m = rot(axis, deg)
        return self.transform(m, sub(pivot, mat_vec(m, pivot)))

    def scale(self, sx, sy=None, sz=None):
        sy = sx if sy is None else sy
        sz = sx if sz is None else sz
        return self.transform(((sx, 0, 0), (0, sy, 0), (0, 0, sz)))

    def mirror_z(self):
        return self.transform(((1, 0, 0), (0, 1, 0), (0, 0, -1)))

    def recolor(self, mat):
        return Geo(mat, self.pos, self.nrm, self.idx, fix=False)


def _quad(pos, nrm, idx, c, u, v):
    n = norm(cross(u, v))
    i = len(pos)
    for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        pos.append(add(c, add(mul(u, su), mul(v, sv))))
        nrm.append(n)
    idx += [(i, i + 1, i + 2), (i, i + 2, i + 3)]

# ------------------------------------------------------------- primitives ---

def box(mat, sx, sy, sz, center=(0, 0, 0)):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    pos, nrm, idx = [], [], []
    for axis, u, v, h in (((1, 0, 0), (0, hy, 0), (0, 0, hz), hx),
                          ((0, 1, 0), (0, 0, hz), (hx, 0, 0), hy),
                          ((0, 0, 1), (hx, 0, 0), (0, hy, 0), hz)):
        for s in (1, -1):
            _quad(pos, nrm, idx, add(center, mul(axis, h * s)), mul(u, s), v)
    return Geo(mat, pos, nrm, idx)


def box_span(mat, x0, x1, y0, y1, z0, z1):
    """Axis-aligned box from min/max extents."""
    return box(mat, abs(x1 - x0), abs(y1 - y0), abs(z1 - z0),
               ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def lathe(mat, profile, segs=32, a0=0.0, a1=360.0):
    """Revolve a (radius, height) profile around +Y.

    The profile must run counter-clockwise in the (r, y) plane, i.e. the
    outside surface lies to the right of the direction of travel.
    """
    full = abs(a1 - a0) >= 360
    ang = [math.radians(a0 + (a1 - a0) * i / segs) for i in range(segs + 1)]
    pos, nrm, idx = [], [], []
    for j in range(len(profile) - 1):
        (r0, y0), (r1, y1) = profile[j], profile[j + 1]
        dr, dy = r1 - r0, y1 - y0
        nr, ny = dy, -dr
        l = math.hypot(nr, ny) or 1.0
        nr, ny = nr / l, ny / l
        base = len(pos)
        for a in ang:
            c, s = math.cos(a), math.sin(a)
            pos += [(r0 * c, y0, r0 * s), (r1 * c, y1, r1 * s)]
            nrm += [(nr * c, ny, nr * s)] * 2
        for i in range(segs):
            a, b = base + 2 * i, base + 2 * i + 2
            idx += [(a, b, b + 1), (a, b + 1, a + 1)]
    if not full:
        # close the open sides of a partial revolution
        for a, s in ((ang[0], -1), (ang[-1], 1)):
            pts = [(r * math.cos(a), y, r * math.sin(a)) for r, y in profile]
            n = norm((-math.sin(a) * s, 0, math.cos(a) * s))
            base = len(pos)
            pos += pts
            nrm += [n] * len(pts)
            for k in range(1, len(pts) - 1):
                idx.append((base, base + k, base + k + 1))
    return Geo(mat, pos, nrm, idx)


def cylinder(mat, r, h, segs=24, r_top=None, caps=True):
    """Cylinder (or cone frustum) along +Y centred on the origin."""
    rt = r if r_top is None else r_top
    prof = [(r, -h / 2), (rt, h / 2)]
    if caps:
        prof = [(0, -h / 2)] + prof + [(0, h / 2)]
    return lathe(mat, prof, segs)


def sphere(mat, r, segs=20, rings=10):
    prof = [(r * math.cos(t), r * math.sin(t))
            for t in (math.pi * (i / rings - 0.5) for i in range(rings + 1))]
    g = lathe(mat, prof, segs)
    # smooth shading for spheres: normals point straight out
    return Geo(mat, g.pos, [norm(p) for p in g.pos], g.idx, fix=False)


def _signed_area(poly):
    return 0.5 * sum(poly[i][0] * poly[i - 1][1] - poly[i - 1][0] * poly[i][1]
                     for i in range(len(poly))) * -1


def _triangulate(poly):
    """Ear clipping for a simple counter-clockwise polygon."""
    def cr(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def inside(p, a, b, c):
        return cr(a, b, p) >= -1e-12 and cr(b, c, p) >= -1e-12 and cr(c, a, p) >= -1e-12

    rem, tris = list(range(len(poly))), []
    while len(rem) > 3:
        for k in range(len(rem)):
            i0, i1, i2 = rem[k - 1], rem[k], rem[(k + 1) % len(rem)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if cr(a, b, c) <= 1e-12:
                continue
            if any(inside(poly[j], a, b, c) for j in rem if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            rem.pop(k)
            break
        else:
            tris.append((rem[-1], rem[0], rem[1]))
            rem.pop(0)
    tris.append(tuple(rem))
    return tris


def extrude(mat, poly, depth, caps=True):
    """Extrude an (x, y) polygon along Z, centred on z = 0."""
    pts = []
    for p in poly:
        if not pts or math.dist(p, pts[-1]) > 1e-9:
            pts.append(tuple(p))
    if math.dist(pts[0], pts[-1]) < 1e-9:
        pts.pop()
    if _signed_area(pts) < 0:  # make counter-clockwise
        pts.reverse()
    hz = depth / 2
    pos, nrm, idx = [], [], []
    n = len(pts)
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        e = norm((y1 - y0, -(x1 - x0), 0))
        b = len(pos)
        pos += [(x0, y0, -hz), (x1, y1, -hz), (x1, y1, hz), (x0, y0, hz)]
        nrm += [e] * 4
        idx += [(b, b + 1, b + 2), (b, b + 2, b + 3)]
    if caps:
        tris = _triangulate(pts)
        for z, nz in ((hz, 1), (-hz, -1)):
            b = len(pos)
            pos += [(x, y, z) for x, y in pts]
            nrm += [(0, 0, nz)] * n
            idx += [(b + i, b + j, b + k) for i, j, k in tris]
    return Geo(mat, pos, nrm, idx)


def _basis_from(d, up=(0, 1, 0)):
    d = norm(d)
    if abs(dot(d, norm(up))) > 0.999:
        up = (1, 0, 0) if abs(d[0]) < 0.9 else (0, 0, 1)
    y = norm(sub(up, mul(d, dot(up, d))))
    return d, y, cross(d, y)


def cyl_between(mat, p0, p1, r, segs=20, r_top=None, caps=True):
    d = sub(p1, p0)
    x, y, z = _basis_from(d, (0, 0, 1))
    # local cylinder axis is +Y -> map it onto d
    m = from_columns(z, x, y) if mat_det(from_columns(z, x, y)) > 0 else from_columns(y, x, z)
    g = cylinder(mat, r, length(d), segs, r_top, caps)
    return g.transform(m, lerp(p0, p1, 0.5))


def box_between(mat, p0, p1, w, h, up=(0, 1, 0)):
    """Box whose long axis runs from p0 to p1; `h` measured along `up`."""
    d = sub(p1, p0)
    x, y, z = _basis_from(d, up)
    return box(mat, length(d), h, w).transform(from_columns(x, y, z), lerp(p0, p1, 0.5))


def polyline_tube(mat, pts, r, segs=12):
    """Bent rod (handrails, pipes): cylinders joined by spheres."""
    out = [cyl_between(mat, pts[i], pts[i + 1], r, segs, caps=False) for i in range(len(pts) - 1)]
    out += [sphere(mat, r, segs, max(4, segs // 2)).at(p) for p in pts]
    return out


def hydraulic_cylinder(p0, p1, r_barrel, r_rod, extension=0.55,
                       barrel='paint', rod='chrome', pin='steel_dark'):
    """Barrel at p0, chromed rod at p1, with clevis eyes at both ends."""
    d = sub(p1, p0)
    L = length(d)
    u = norm(d)
    e0 = add(p0, mul(u, r_barrel * 1.2))
    split = add(p0, mul(d, extension))
    e1 = sub(p1, mul(u, r_rod * 2.0))
    parts = [
        cyl_between(barrel, e0, split, r_barrel, 24),
        cyl_between(barrel, sub(split, mul(u, 0.04)), add(split, mul(u, 0.02)), r_barrel * 1.12, 24),
        cyl_between(barrel, sub(e0, mul(u, 0.03)), add(e0, mul(u, 0.05)), r_barrel * 1.1, 24),
        cyl_between(rod, split, e1, r_rod, 20),
        cyl_between(barrel, e1, p1, r_rod * 1.3, 16),
    ]
    # eyes (pins perpendicular to the cylinder, horizontal where possible)
    side = norm(cross(u, (0, 1, 0))) if abs(u[1]) < 0.99 else (0, 0, 1)
    for p, rr in ((p0, r_barrel * 0.9), (p1, r_rod * 1.5)):
        parts.append(cyl_between(pin, sub(p, mul(side, rr * 0.9)), add(p, mul(side, rr * 0.9)), rr, 16))
    return parts


def convex_hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cr(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cr(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]  # counter-clockwise


def belt_path(circles, samples=720):
    """Closed CCW loop wrapping a set of (cx, cy, r) circles, e.g. a track chain."""
    pts = []
    for cx, cy, r in circles:
        for i in range(samples):
            a = 2 * math.pi * i / samples
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return convex_hull(pts)


def resample_loop(loop, count):
    """Return `count` (point, tangent) pairs evenly spaced along a closed loop."""
    segs, total = [], 0.0
    for i in range(len(loop)):
        a, b = loop[i], loop[(i + 1) % len(loop)]
        l = math.dist(a, b)
        segs.append((total, l, a, b))
        total += l
    out, step, k = [], total / count, 0
    for i in range(count):
        s = i * step
        while segs[k][0] + segs[k][1] < s:
            k += 1
        s0, l, a, b = segs[k]
        t = (s - s0) / l if l else 0
        p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        out.append((p, (b[0] - a[0]) / l, (b[1] - a[1]) / l))
    return out, total

# ------------------------------------------------------------------ model ---

class Model:
    def __init__(self, name, materials):
        self.name = name
        self.materials = materials  # name -> dict(color, metallic, roughness, ...)
        self.parts = {}

    def add(self, part, *geos):
        bucket = self.parts.setdefault(part, [])
        for g in geos:
            if isinstance(g, (list, tuple)):
                self.add(part, *g)
            else:
                bucket.append(g)

    def mirrored_copy(self, src, dst):
        self.parts[dst] = [g.mirror_z() for g in self.parts[src]]

    def _merged(self):
        """part -> material -> (pos, nrm, idx) with geos of a material merged."""
        out = {}
        for part, geos in self.parts.items():
            mats = out.setdefault(part, {})
            for g in geos:
                pos, nrm, idx = mats.setdefault(g.mat, ([], [], []))
                o = len(pos)
                pos += g.pos
                nrm += g.nrm
                idx += [(a + o, b + o, c + o) for a, b, c in g.idx]
        return out

    def bounds(self):
        lo, hi = [1e9] * 3, [-1e9] * 3
        for geos in self.parts.values():
            for g in geos:
                for p in g.pos:
                    for i in range(3):
                        lo[i] = min(lo[i], p[i])
                        hi[i] = max(hi[i], p[i])
        return lo, hi

    def stats(self):
        tris = sum(len(g.idx) for gs in self.parts.values() for g in gs)
        verts = sum(len(g.pos) for gs in self.parts.values() for g in gs)
        return verts, tris

    # ---- writers

    def write_glb(self, path):
        merged = self._merged()
        mat_names = sorted({m for mats in merged.values() for m in mats})
        materials = []
        for name in mat_names:
            spec = self.materials[name]
            r, g, b = spec['color']
            a = spec.get('alpha', 1.0)
            m = {'name': name,
                 'pbrMetallicRoughness': {'baseColorFactor': [r, g, b, a],
                                          'metallicFactor': spec.get('metallic', 0.0),
                                          'roughnessFactor': spec.get('roughness', 0.6)}}
            if a < 1:
                m['alphaMode'] = 'BLEND'
                m['doubleSided'] = True
            if 'emissive' in spec:
                m['emissiveFactor'] = list(spec['emissive'])
            materials.append(m)

        blob = bytearray()
        views, accessors, meshes, nodes = [], [], [], []

        def push(data, target):
            while len(blob) % 4:
                blob.append(0)
            views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': len(data), 'target': target})
            blob.extend(data)
            return len(views) - 1

        for part, mats in merged.items():
            prims = []
            for name, (pos, nrm, idx) in mats.items():
                flat_p = [c for p in pos for c in p]
                flat_n = [c for n in nrm for c in n]
                flat_i = [i for t in idx for i in t]
                pv = push(struct.pack('<%df' % len(flat_p), *flat_p), 34962)
                nv = push(struct.pack('<%df' % len(flat_n), *flat_n), 34962)
                iv = push(struct.pack('<%dI' % len(flat_i), *flat_i), 34963)
                lo = [min(p[i] for p in pos) for i in range(3)]
                hi = [max(p[i] for p in pos) for i in range(3)]
                accessors += [
                    {'bufferView': pv, 'componentType': 5126, 'count': len(pos), 'type': 'VEC3', 'min': lo, 'max': hi},
                    {'bufferView': nv, 'componentType': 5126, 'count': len(nrm), 'type': 'VEC3'},
                    {'bufferView': iv, 'componentType': 5125, 'count': len(flat_i), 'type': 'SCALAR'},
                ]
                a = len(accessors)
                prims.append({'attributes': {'POSITION': a - 3, 'NORMAL': a - 2}, 'indices': a - 1,
                              'material': mat_names.index(name)})
            meshes.append({'name': part, 'primitives': prims})
            nodes.append({'name': part, 'mesh': len(meshes) - 1})

        nodes.append({'name': self.name, 'children': list(range(len(nodes)))})
        doc = {
            'asset': {'version': '2.0', 'generator': 'Claudin meshkit.py',
                      'extras': {'units': 'metres', 'scale': '1:1'}},
            'scene': 0,
            'scenes': [{'name': self.name, 'nodes': [len(nodes) - 1]}],
            'nodes': nodes, 'meshes': meshes, 'materials': materials,
            'accessors': accessors, 'bufferViews': views,
            'buffers': [{'byteLength': len(blob)}],
        }
        js = json.dumps(doc, separators=(',', ':')).encode()
        js += b' ' * (-len(js) % 4)
        blob.extend(b'\0' * (-len(blob) % 4))
        total = 12 + 8 + len(js) + 8 + len(blob)
        with open(path, 'wb') as f:
            f.write(struct.pack('<4sII', b'glTF', 2, total))
            f.write(struct.pack('<I4s', len(js), b'JSON') + js)
            f.write(struct.pack('<I4s', len(blob), b'BIN\0') + bytes(blob))

    def write_obj(self, obj_path, mtl_name):
        merged = self._merged()
        lines = ['# %s - 1:1 scale, units: metres, +Y up, +X forward' % self.name,
                 'mtllib %s' % mtl_name]
        off = 1
        used = set()
        for part, mats in merged.items():
            lines.append('o %s' % part)
            for name, (pos, nrm, idx) in mats.items():
                used.add(name)
                lines.append('usemtl %s' % name)
                lines += ['v %.5f %.5f %.5f' % p for p in pos]
                lines += ['vn %.4f %.4f %.4f' % n for n in nrm]
                lines += ['f %d//%d %d//%d %d//%d' % (a + off, a + off, b + off, b + off, c + off, c + off)
                          for a, b, c in idx]
                off += len(pos)
        with open(obj_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        mtl = []
        for name in sorted(used):
            s = self.materials[name]
            mtl += ['newmtl %s' % name, 'Kd %.4f %.4f %.4f' % tuple(s['color']),
                    'Ka 0 0 0', 'Ks %.3f %.3f %.3f' % ((0.9 - s.get('roughness', 0.6) * 0.8,) * 3),
                    'Ns %d' % int(10 + (1 - s.get('roughness', 0.6)) * 400),
                    'd %.3f' % s.get('alpha', 1.0)]
            if 'emissive' in s:
                mtl.append('Ke %.3f %.3f %.3f' % tuple(s['emissive']))
            mtl.append('')
        with open(obj_path.rsplit('/', 1)[0] + '/' + mtl_name, 'w') as f:
            f.write('\n'.join(mtl))


# Shared material library (linear-space PBR values).
MATERIALS = {
    'paint':       {'color': (0.93, 0.62, 0.03), 'metallic': 0.1, 'roughness': 0.45},
    'paint_dark':  {'color': (0.06, 0.06, 0.06), 'metallic': 0.1, 'roughness': 0.55},
    'steel':       {'color': (0.42, 0.42, 0.43), 'metallic': 0.85, 'roughness': 0.45},
    'steel_dark':  {'color': (0.10, 0.10, 0.10), 'metallic': 0.7, 'roughness': 0.7},
    'steel_worn':  {'color': (0.30, 0.29, 0.28), 'metallic': 0.8, 'roughness': 0.55},
    'edge':        {'color': (0.55, 0.55, 0.56), 'metallic': 0.9, 'roughness': 0.35},
    'chrome':      {'color': (0.92, 0.92, 0.94), 'metallic': 0.7, 'roughness': 0.18},
    'rubber':      {'color': (0.025, 0.025, 0.025), 'metallic': 0.0, 'roughness': 0.9},
    'grille':      {'color': (0.03, 0.03, 0.03), 'metallic': 0.5, 'roughness': 0.6},
    'glass':       {'color': (0.25, 0.35, 0.40), 'metallic': 0.0, 'roughness': 0.05, 'alpha': 0.35},
    'lens':        {'color': (1.0, 0.98, 0.9), 'metallic': 0.0, 'roughness': 0.1, 'emissive': (0.9, 0.9, 0.8)},
    'lens_red':    {'color': (0.8, 0.02, 0.02), 'metallic': 0.0, 'roughness': 0.2, 'emissive': (0.5, 0.0, 0.0)},
    'beacon':      {'color': (1.0, 0.45, 0.0), 'metallic': 0.0, 'roughness': 0.2, 'emissive': (0.6, 0.25, 0.0), 'alpha': 0.85},
    'seat':        {'color': (0.04, 0.04, 0.045), 'metallic': 0.0, 'roughness': 0.85},
    'lens_amber':  {'color': (1.0, 0.55, 0.0), 'metallic': 0.0, 'roughness': 0.2, 'emissive': (0.5, 0.25, 0.0)},
    'glass_tint':  {'color': (0.03, 0.04, 0.05), 'metallic': 0.2, 'roughness': 0.05, 'alpha': 0.92},
    'trim_black':  {'color': (0.035, 0.035, 0.038), 'metallic': 0.0, 'roughness': 0.7},
    'alloy':       {'color': (0.70, 0.71, 0.73), 'metallic': 0.6, 'roughness': 0.3},
    'car_blue':    {'color': (0.02, 0.07, 0.20), 'metallic': 0.6, 'roughness': 0.25},
    'car_white':   {'color': (0.86, 0.87, 0.88), 'metallic': 0.1, 'roughness': 0.3},
    'counterweight': {'color': (0.12, 0.12, 0.13), 'metallic': 0.3, 'roughness': 0.6},
}


def save(model, folder, name, obj=False):
    """Write <folder>/<name>.glb (and optionally .obj/.mtl) next to this file."""
    import os
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), folder)
    os.makedirs(out, exist_ok=True)
    model.write_glb(os.path.join(out, name + '.glb'))
    if obj:
        model.write_obj(os.path.join(out, name + '.obj'), name + '.mtl')
    lo, hi = model.bounds()
    print('%s: L %.2f  W %.2f  H %.2f m  | %d verts, %d tris' %
          (model.name, hi[0] - lo[0], hi[2] - lo[2], hi[1] - lo[1], *model.stats()))
