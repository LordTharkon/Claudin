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
    __slots__ = ('mat', 'pos', 'nrm', 'idx', 'uv')

    def __init__(self, mat, pos, nrm, idx, fix=True):
        self.mat, self.pos, self.nrm, self.idx = mat, pos, nrm, idx
        self.uv = None
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


def hydraulic_parts(p0, p1, r_barrel, r_rod, extension=0.55,
                    barrel='paint_yellow', rod='chrome', pin='steel_dark'):
    """(barrel geos, rod geos) for a cylinder with its barrel pin at p0 and rod eye at p1."""
    d = sub(p1, p0)
    u = norm(d)
    e0 = add(p0, mul(u, r_barrel * 1.2))
    split = add(p0, mul(d, extension))
    e1 = sub(p1, mul(u, r_rod * 2.0))
    side = norm(cross(u, (0, 1, 0))) if abs(u[1]) < 0.99 else (0, 0, 1)
    pin_geo = lambda p, rr: cyl_between(pin, sub(p, mul(side, rr * 0.9)), add(p, mul(side, rr * 0.9)), rr, 16)
    b = [cyl_between(barrel, e0, split, r_barrel, 24),
         cyl_between(barrel, sub(split, mul(u, 0.04)), add(split, mul(u, 0.02)), r_barrel * 1.12, 24),
         cyl_between(barrel, sub(e0, mul(u, 0.03)), add(e0, mul(u, 0.05)), r_barrel * 1.1, 24),
         pin_geo(p0, r_barrel * 0.9)]
    r = [cyl_between(rod, sub(split, mul(u, 0.01)), e1, r_rod, 20),
         cyl_between(barrel, e1, p1, r_rod * 1.3, 16),
         pin_geo(p1, r_rod * 1.5)]
    return b, r


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
    """A rigged vehicle: named nodes with real pivots, each holding Geo chunks.

    Geometry is authored in world coordinates in a convenient design pose. Nodes may
    carry a rest-pose rotation about their pivot (see `pose`) which is baked in by
    `finalize`; hydraulic cylinders are generated after posing so they stay attached
    to both of their mounts. On export every node's origin is its pivot.
    """

    def __init__(self, name, materials):
        self.name = name
        self.materials = materials
        self.nodes = {}     # name -> dict(parent, pivot, rot)
        self.order = []
        self.parts = {}     # name -> [Geo]
        self.cyls = []
        self.done = False

    def node(self, name, parent=None, pivot=(0, 0, 0)):
        assert name not in self.nodes, name
        assert parent is None or parent in self.nodes, parent
        self.nodes[name] = dict(parent=parent, pivot=tuple(pivot), rot=None)
        self.order.append(name)
        self.parts.setdefault(name, [])
        return name

    def add(self, part, *geos):
        if part not in self.nodes:
            self.node(part)
        bucket = self.parts[part]
        for g in geos:
            if isinstance(g, (list, tuple)):
                self.add(part, *g)
            else:
                bucket.append(g)

    def pose(self, name, axis, deg):
        """Rotate a node (and everything under it) about its pivot for the rest pose."""
        self.nodes[name]['rot'] = (axis, deg)

    def cylinder(self, name, parent, attach, p0, p1, r_barrel, r_rod, extension=0.55, barrel='paint_yellow'):
        """Hydraulic cylinder: barrel pinned to `parent` at p0, rod eye pinned to `attach` at p1.
        Exported as <name>_rotate (barrel, origin at p0) with child <name>_rod_slide (origin at p1)."""
        self.cyls.append(dict(name=name, parent=parent, attach=attach, p0=tuple(p0), p1=tuple(p1),
                              rb=r_barrel, rr=r_rod, ext=extension, mat=barrel))

    def link(self, name, parent, attach, p0, p1, builder):
        """Rigid link pinned to `parent` at p0 whose other pin follows `attach` at p1.
        `builder(p0, p1)` returns its geometry once both pins are posed. Node origin is p0."""
        self.cyls.append(dict(name=name, parent=parent, attach=attach, p0=tuple(p0), p1=tuple(p1), builder=builder))

    def mirror(self, src='right', dst='left'):
        """Duplicate every node / cylinder whose name contains `src`, mirrored across z = 0."""
        sw = lambda n: n.replace(src, dst) if n and src in n else n
        for name in list(self.order):
            if src in name:
                nd = self.nodes[name]
                px, py, pz = nd['pivot']
                self.node(sw(name), sw(nd['parent']), (px, py, -pz))
                self.parts[sw(name)] = [g.mirror_z() for g in self.parts[name]]
        for c in list(self.cyls):
            if src in c['name']:
                m = dict(c)
                m['name'], m['parent'], m['attach'] = sw(c['name']), sw(c['parent']), sw(c['attach'])
                m['p0'] = (c['p0'][0], c['p0'][1], -c['p0'][2])
                m['p1'] = (c['p1'][0], c['p1'][1], -c['p1'][2])
                self.cyls.append(m)

    # ---- rest pose, cylinders, centring

    def finalize(self):
        if self.done:
            return
        xf = {}

        def tf(name):
            if name is None:
                return (IDENTITY, (0.0, 0.0, 0.0))
            if name in xf:
                return xf[name]
            nd = self.nodes[name]
            M0, t0 = tf(nd['parent'])
            if nd['rot']:
                R = rot(*nd['rot'])
                q = nd['pivot']
                L = (R, sub(q, mat_vec(R, q)))
                xf[name] = (mat_mul(M0, L[0]), add(mat_vec(M0, L[1]), t0))
            else:
                xf[name] = (M0, t0)
            return xf[name]

        app = lambda T, p: add(mat_vec(T[0], p), T[1])
        pivots = {}
        for name in self.order:
            nd = self.nodes[name]
            T = tf(name)
            pivots[name] = app(tf(nd['parent']), nd['pivot'])
            self.parts[name] = [g.transform(*T) if T[0] is not IDENTITY or any(T[1]) else g for g in self.parts[name]]
        for c in self.cyls:
            p0 = app(tf(c['parent']), c['p0'])
            p1 = app(tf(c['attach']), c['p1'])
            if 'builder' in c:
                self.node(c['name'], c['parent'], p0)
                pivots[c['name']] = p0
                self.parts[c['name']] = list(c['builder'](p0, p1))
                continue
            barrel, rod = hydraulic_parts(p0, p1, c['rb'], c['rr'], c['ext'], barrel=c['mat'])
            n0, n1 = c['name'] + '_rotate', c['name'] + '_rod_slide'
            self.node(n0, c['parent'], p0)
            self.node(n1, n0, p1)
            pivots[n0], pivots[n1] = p0, p1
            self.parts[n0], self.parts[n1] = barrel, rod
        # origin on the ground, centred under the vehicle
        lo, hi = self.bounds()
        off = (-(lo[0] + hi[0]) / 2, -lo[1], -(lo[2] + hi[2]) / 2)
        for name in self.order:
            self.parts[name] = [g.move(*off) for g in self.parts[name]]
            self.nodes[name]['pivot'] = add(pivots[name], off)
            self.nodes[name]['rot'] = None
        self.done = True

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

    # ---- writer

    def write_glb(self, path, tex_size=1024):
        import uvkit
        self.finalize()
        uv_parts, density = uvkit.unwrap(self.parts, tex_size)
        self.texel_density = density
        used = sorted({g.mat for gs in uv_parts.values() for g in gs})
        blob = bytearray()
        views, accessors, meshes, nodes = [], [], [], []
        images, textures, materials = [], [], []

        def push(data, target=None):
            while len(blob) % 4:
                blob.append(0)
            v = {'buffer': 0, 'byteOffset': len(blob), 'byteLength': len(data)}
            if target:
                v['target'] = target
            views.append(v)
            blob.extend(data)
            return len(views) - 1

        for i, name in enumerate(used):
            spec = self.materials[name]
            images.append({'name': name, 'mimeType': 'image/png',
                           'bufferView': push(solid_png(spec['color'], tex_size))})
            textures.append({'name': name, 'sampler': 0, 'source': i})
            a = spec.get('alpha', 1.0)
            m = {'name': name,
                 'pbrMetallicRoughness': {'baseColorTexture': {'index': i},
                                          'baseColorFactor': [1.0, 1.0, 1.0, a],
                                          'metallicFactor': spec.get('metallic', 0.0),
                                          'roughnessFactor': spec.get('roughness', 0.6)}}
            if a < 1:
                m['alphaMode'] = 'BLEND'
            if 'emissive' in spec:
                m['emissiveFactor'] = list(spec['emissive'])
            materials.append(m)

        index = {}
        for name in self.order:
            nd = self.nodes[name]
            pp = self.nodes[nd['parent']]['pivot'] if nd['parent'] else (0.0, 0.0, 0.0)
            node = {'name': name}
            t = sub(nd['pivot'], pp)
            if any(abs(c) > 1e-9 for c in t):
                node['translation'] = [round(c, 6) for c in t]
            geos = uv_parts.get(name, [])
            if geos:
                piv = nd['pivot']
                prims = []
                by_mat = {}
                for g in geos:
                    by_mat.setdefault(g.mat, []).append(g)
                for mname in sorted(by_mat):
                    pos, nrm, uvs, idx = [], [], [], []
                    for g in by_mat[mname]:
                        o = len(pos)
                        pos += [sub(p, piv) for p in g.pos]
                        nrm += g.nrm
                        uvs += g.uv
                        idx += [i + o for tri in g.idx for i in tri]
                    fp = [c for p in pos for c in p]
                    pv = push(struct.pack('<%df' % len(fp), *fp), 34962)
                    fn = [c for n in nrm for c in n]
                    nv = push(struct.pack('<%df' % len(fn), *fn), 34962)
                    fu = [c for u in uvs for c in u]
                    uvv = push(struct.pack('<%df' % len(fu), *fu), 34962)
                    iv = push(struct.pack('<%dI' % len(idx), *idx), 34963)
                    lo = [min(p[k] for p in pos) for k in range(3)]
                    hi = [max(p[k] for p in pos) for k in range(3)]
                    accessors += [
                        {'bufferView': pv, 'componentType': 5126, 'count': len(pos), 'type': 'VEC3', 'min': lo, 'max': hi},
                        {'bufferView': nv, 'componentType': 5126, 'count': len(nrm), 'type': 'VEC3'},
                        {'bufferView': uvv, 'componentType': 5126, 'count': len(uvs), 'type': 'VEC2'},
                        {'bufferView': iv, 'componentType': 5125, 'count': len(idx), 'type': 'SCALAR'},
                    ]
                    a = len(accessors)
                    prims.append({'attributes': {'POSITION': a - 4, 'NORMAL': a - 3, 'TEXCOORD_0': a - 2},
                                  'indices': a - 1, 'material': used.index(mname)})
                meshes.append({'name': name, 'primitives': prims})
                node['mesh'] = len(meshes) - 1
            index[name] = len(nodes)
            nodes.append(node)
        for name in self.order:
            par = self.nodes[name]['parent']
            if par:
                nodes[index[par]].setdefault('children', []).append(index[name])
        root_name = self.name.lower().replace(' ', '_')
        nodes.append({'name': root_name,
                      'children': [index[n] for n in self.order if self.nodes[n]['parent'] is None]})
        doc = {
            'asset': {'version': '2.0', 'generator': 'Claudin meshkit.py',
                      'extras': {'units': 'metres', 'up': '+Y', 'forward': '+X', 'scale': '1:1'}},
            'scene': 0,
            'scenes': [{'name': root_name, 'nodes': [len(nodes) - 1]}],
            'nodes': nodes, 'meshes': meshes, 'materials': materials,
            'textures': textures, 'images': images,
            'samplers': [{'magFilter': 9729, 'minFilter': 9987, 'wrapS': 33071, 'wrapT': 33071}],
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


def _srgb(c):
    c = max(0.0, min(1.0, c))
    return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055


def solid_png(color, size):
    """Single-colour RGB PNG (linear colour converted to sRGB)."""
    import zlib
    px = bytes(int(round(_srgb(c) * 255)) for c in color)
    row = b'\x00' + px * size
    raw = zlib.compress(row * size, 9)

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', size, size, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', raw) + chunk(b'IEND', b''))


# Shared material library (linear-space PBR values). Names follow the model-pack list.
MATERIALS = {
    'paint_yellow':    {'color': (0.93, 0.62, 0.03), 'metallic': 0.1, 'roughness': 0.45},
    'paint_blue':      {'color': (0.02, 0.07, 0.20), 'metallic': 0.6, 'roughness': 0.25},
    'paint_white':     {'color': (0.86, 0.87, 0.88), 'metallic': 0.1, 'roughness': 0.3},
    'paint_gray_dark': {'color': (0.12, 0.12, 0.13), 'metallic': 0.3, 'roughness': 0.6},
    'chassis_black':   {'color': (0.06, 0.06, 0.06), 'metallic': 0.1, 'roughness': 0.55},
    'plastic_black':   {'color': (0.035, 0.035, 0.038), 'metallic': 0.0, 'roughness': 0.7},
    'steel':           {'color': (0.42, 0.42, 0.43), 'metallic': 0.85, 'roughness': 0.45},
    'steel_dark':      {'color': (0.10, 0.10, 0.10), 'metallic': 0.7, 'roughness': 0.7},
    'track_steel':     {'color': (0.30, 0.29, 0.28), 'metallic': 0.8, 'roughness': 0.55},
    'wear_steel':      {'color': (0.55, 0.55, 0.56), 'metallic': 0.9, 'roughness': 0.35},
    'chrome':          {'color': (0.92, 0.92, 0.94), 'metallic': 0.7, 'roughness': 0.18},
    'aluminum':        {'color': (0.70, 0.71, 0.73), 'metallic': 0.6, 'roughness': 0.3},
    'tyre_rubber':     {'color': (0.025, 0.025, 0.025), 'metallic': 0.0, 'roughness': 0.9},
    'hose_rubber':     {'color': (0.03, 0.03, 0.03), 'metallic': 0.0, 'roughness': 0.75},
    'rubber_trim':     {'color': (0.02, 0.02, 0.02), 'metallic': 0.0, 'roughness': 0.8},
    'seat_fabric':     {'color': (0.04, 0.04, 0.045), 'metallic': 0.0, 'roughness': 0.85},
    'glass':           {'color': (0.25, 0.35, 0.40), 'metallic': 0.0, 'roughness': 0.05, 'alpha': 0.35},
    'glass_tinted':    {'color': (0.03, 0.04, 0.05), 'metallic': 0.2, 'roughness': 0.05, 'alpha': 0.92},
    'lamp_lens':       {'color': (1.0, 0.98, 0.9), 'metallic': 0.0, 'roughness': 0.1, 'emissive': (0.9, 0.9, 0.8)},
    'lamp_amber':      {'color': (1.0, 0.55, 0.0), 'metallic': 0.0, 'roughness': 0.2, 'emissive': (0.5, 0.25, 0.0)},
    'lamp_red':        {'color': (0.8, 0.02, 0.02), 'metallic': 0.0, 'roughness': 0.2, 'emissive': (0.5, 0.0, 0.0)},
    'lamp_beacon':     {'color': (1.0, 0.45, 0.0), 'metallic': 0.0, 'roughness': 0.2, 'emissive': (0.6, 0.25, 0.0)},
}


def save(model, folder, name):
    """Write <folder>/<name>.glb next to this file."""
    import os
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), folder)
    os.makedirs(out, exist_ok=True)
    model.write_glb(os.path.join(out, name + '.glb'))
    lo, hi = model.bounds()
    print('%s: L %.2f  W %.2f  H %.2f m  | %d verts, %d tris | %.0f px/m' %
          (model.name, hi[0] - lo[0], hi[2] - lo[2], hi[1] - lo[1], *model.stats(), model.texel_density))
