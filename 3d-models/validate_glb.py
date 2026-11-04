"""Check every .glb in this folder against the model-pack requirements.

    python3 3d-models/validate_glb.py            # all models
    python3 3d-models/validate_glb.py pickup     # only paths containing "pickup"

Exits non-zero if any model fails. Checks: glTF 2.0 / no Draco / allowed extensions;
Y-up metres with the origin on the ground, centred; rest pose (node translations only, except
track shoes `track_<side>_shoe_NN`, which carry the rotation that follows the track loop);
TEXCOORD_0 on every primitive, UVs inside 0-1, no overlapping UVs per material texture,
>= 4 px island padding at 1024 px and texel-density spread; material names from the
list, one PNG baseColorTexture each named after the material, white baseColorFactor
(alpha only on glass), metal-roughness only, no vertex colours, no duplicates; named
nodes and meshes, no "Instanced" nodes, rig suffixes.
"""
import glob
import json
import os
import re
import struct
import sys

import numpy as np

ALLOWED = {'paint_yellow', 'paint_yellow_dusty', 'paint_white', 'paint_red', 'paint_orange', 'paint_blue',
           'paint_green', 'paint_gray', 'paint_gray_dark', 'chassis_black', 'plastic_black', 'steel',
           'steel_dark', 'aluminum', 'chrome', 'wear_steel', 'track_steel', 'tyre_rubber', 'hose_rubber',
           'cable_rubber', 'rubber_trim', 'wire_rope', 'seat_fabric', 'glass', 'glass_tinted', 'lamp_lens',
           'lamp_amber', 'lamp_red', 'lamp_beacon'}
# additions for site props, used only where nothing on the standard list fits
ALLOWED |= {'timber', 'plywood', 'concrete', 'gravel', 'sand', 'woven_polypropylene', 'fabric_pink'}
EXT_OK = {'KHR_texture_transform', 'KHR_materials_emissive_strength'}
SUFFIXES = ('_steer', '_spin', '_tilt', '_rotate', '_lift', '_slide')
TEX = 1024
PAD = 4
SHOE = re.compile(r'^track_(left|right)_shoe_\d{2,}$')


def quat_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load(path):
    data = open(path, 'rb').read()
    magic, ver, total = struct.unpack('<4sII', data[:12])
    assert magic == b'glTF' and ver == 2 and total == len(data)
    jlen, jtype = struct.unpack('<I4s', data[12:20])
    doc = json.loads(data[20:20 + jlen])
    off = 20 + jlen
    blen, btype = struct.unpack('<I4s', data[off:off + 8])
    return doc, data[off + 8:off + 8 + blen]


def accessor(doc, binary, i):
    a = doc['accessors'][i]
    v = doc['bufferViews'][a['bufferView']]
    n = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
    dt = {5126: np.float32, 5125: np.uint32, 5123: np.uint16}[a['componentType']]
    arr = np.frombuffer(binary, dt, a['count'] * n, v.get('byteOffset', 0) + a.get('byteOffset', 0))
    return arr.reshape(-1, n) if n > 1 else arr


def raster_overlaps(tris_px):
    """Count pixels covered by more than one triangle (pixel-centre sampling, open edges)."""
    cov = np.zeros((TEX, TEX), np.int16)
    for t in tris_px:
        x0, y0 = np.floor(t.min(0)).astype(int)
        x1, y1 = np.ceil(t.max(0)).astype(int)
        x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, TEX - 1), min(y1, TEX - 1)
        if x1 < x0 or y1 < y0:
            continue
        xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        (ax, ay), (bx, by), (cx, cy) = t
        d = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        if abs(d) < 1e-12:
            continue
        s = 1 if d > 0 else -1
        e0 = s * ((bx - ax) * (ys - ay) - (by - ay) * (xs - ax))
        e1 = s * ((cx - bx) * (ys - by) - (cy - by) * (xs - bx))
        e2 = s * ((ax - cx) * (ys - cy) - (ay - cy) * (xs - cx))
        inside = (e0 > 1e-9) & (e1 > 1e-9) & (e2 > 1e-9)
        cov[y0:y1 + 1, x0:x1 + 1] += inside
    return int((cov > 1).sum())


def islands(uv_px, tris):
    """Union triangles sharing a UV point; returns list of (xmin, ymin, xmax, ymax)."""
    key = np.round(uv_px * 64).astype(np.int64)
    ids = {}
    vid = np.array([ids.setdefault((a, b), len(ids)) for a, b in key])
    parent = list(range(len(ids)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b, c in vid[tris]:
        ra, rb, rc = find(a), find(b), find(c)
        parent[rb] = ra
        parent[find(rc)] = ra
    boxes = {}
    for v in range(len(ids)):
        boxes.setdefault(find(v), []).append(v)
    pts = np.zeros((len(ids), 2))
    for (a, b), i in ids.items():
        pts[i] = (a / 64, b / 64)
    return [(*pts[vs].min(0), *pts[vs].max(0)) for vs in boxes.values()]


def min_gap(boxes):
    """Smallest gap between any two island boxes (px), negative if they overlap."""
    b = sorted(boxes)
    best = 1e9
    for i in range(len(b)):
        for j in range(i + 1, len(b)):
            if b[j][0] - b[i][2] >= best:
                break
            gx = max(b[j][0] - b[i][2], b[i][0] - b[j][2])
            gy = max(b[j][1] - b[i][3], b[i][1] - b[j][3])
            best = min(best, max(gx, gy))
    return best


def validate(path):
    doc, binary = load(path)
    errs, info = [], []
    E = errs.append
    if doc['asset'].get('version') != '2.0':
        E('asset.version is not 2.0')
    used = set(doc.get('extensionsUsed', [])) | set(doc.get('extensionsRequired', []))
    if used - EXT_OK:
        E('disallowed extensions: %s' % sorted(used - EXT_OK))
    if len(glob.glob(os.path.join(os.path.dirname(path), '*.glb'))) != 1:
        E('folder does not contain exactly one .glb')

    # ---- nodes
    names = [n.get('name') for n in doc['nodes']]
    for i, n in enumerate(doc['nodes']):
        if not n.get('name'):
            E('node %d has no name' % i)
        elif 'instanc' in n['name'].lower():
            E('node "%s" looks like an instance name' % n['name'])
        for k in ('rotation', 'scale', 'matrix'):
            if k == 'rotation' and k in n and n.get('name') and SHOE.match(n['name']):
                if abs(np.linalg.norm(n[k]) - 1) > 1e-5:
                    E('node "%s" rotation is not a unit quaternion' % n['name'])
                continue
            if k in n:
                E('node "%s" has %s (rest pose should only translate)' % (n.get('name'), k))
    if len(set(names)) != len(names):
        E('duplicate node names')
    for i, me in enumerate(doc['meshes']):
        if not me.get('name'):
            E('mesh %d has no name' % i)
    rigged = [n for n in names if n and n.endswith(SUFFIXES)]
    bad_suffix = [n for n in names if n and re.search(r'_(steer|spin|tilt|rotate|lift|slide)(_|$)', n) and not n.endswith(SUFFIXES)]
    if bad_suffix:
        E('rig suffix not at end of name: %s' % bad_suffix)
    info.append('%d nodes, %d rigged (%s)' % (len(names), len(rigged),
                ', '.join('%s %d' % (s, sum(n.endswith(s) for n in rigged)) for s in SUFFIXES if any(n.endswith(s) for n in rigged))))

    # ---- world bounds (translation + rotation hierarchy)
    world = {}
    root = doc['scenes'][doc.get('scene', 0)]['nodes']

    def walk(i, R, t):
        n = doc['nodes'][i]
        t = t + R @ np.array(n.get('translation', [0, 0, 0]))
        if 'rotation' in n:
            R = R @ quat_matrix(n['rotation'])
        world[i] = (R, t)
        for c in n.get('children', []):
            walk(c, R, t)
    for r in root:
        walk(r, np.eye(3), np.zeros(3))
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    drawn = 0
    for i, n in enumerate(doc['nodes']):
        if 'mesh' in n:
            R, t = world[i]
            for p in doc['meshes'][n['mesh']]['primitives']:
                P = accessor(doc, binary, p['attributes']['POSITION']).astype(np.float64) @ R.T + t
                lo = np.minimum(lo, P.min(0))
                hi = np.maximum(hi, P.max(0))
                drawn += doc['accessors'][p['indices']]['count'] // 3
    size = hi - lo
    if abs(lo[1]) > 1e-3:
        E('lowest point is at y=%.4f, not on the ground' % lo[1])
    if abs((lo[0] + hi[0]) / 2) > 1e-3 or abs((lo[2] + hi[2]) / 2) > 1e-3:
        E('origin not centred under the vehicle')
    if not (size[0] > size[2] * 0.9):
        E('model does not appear to face +X (length %.2f < width %.2f)' % (size[0], size[2]))
    info.append('size L %.2f x W %.2f x H %.2f m' % (size[0], size[2], size[1]))
    unique = sum(doc['accessors'][p['indices']]['count'] // 3 for me in doc['meshes'] for p in me['primitives'])
    info.append('%d tris drawn, %d unique (%d meshes)' % (drawn, unique, len(doc['meshes'])))

    # ---- materials / textures
    mats = doc.get('materials', [])
    seen = {}
    for m in mats:
        nm = m.get('name')
        if not nm:
            E('unnamed material')
            continue
        base = nm[:-7] if nm.endswith('_2sided') else nm
        if base not in ALLOWED and not base.startswith('decal_'):
            E('material "%s" not in the allowed list' % nm)
        if m.get('doubleSided') and not nm.endswith('_2sided'):
            E('material "%s" is double-sided but not named *_2sided' % nm)
        if m.get('extensions'):
            E('material "%s" uses extensions' % nm)
        pbr = m.get('pbrMetallicRoughness')
        if not pbr or 'baseColorTexture' not in pbr:
            E('material "%s" has no baseColorTexture' % nm)
            continue
        f = pbr.get('baseColorFactor', [1, 1, 1, 1])
        if f[:3] != [1.0, 1.0, 1.0]:
            E('material "%s" baseColorFactor is not white' % nm)
        if f[3] < 1 and not base.startswith('glass'):
            E('material "%s" has alpha but is not glass' % nm)
        img = doc['images'][doc['textures'][pbr['baseColorTexture']['index']]['source']]
        if img.get('mimeType') != 'image/png' or img.get('name') != nm:
            E('material "%s" texture is not a PNG named after it' % nm)
        bv = doc['bufferViews'][img['bufferView']]
        png = binary[bv.get('byteOffset', 0):bv.get('byteOffset', 0) + 24]
        if png[:8] != b'\x89PNG\r\n\x1a\n':
            E('material "%s" image is not PNG data' % nm)
        import hashlib
        img_bytes = binary[bv.get('byteOffset', 0):bv.get('byteOffset', 0) + bv['byteLength']]
        sig = json.dumps([pbr.get('metallicFactor'), pbr.get('roughnessFactor'), f, m.get('emissiveFactor'),
                          m.get('alphaMode'), m.get('doubleSided', False)]) + hashlib.sha1(img_bytes).hexdigest()
        if sig in seen:
            E('materials "%s" and "%s" are duplicates' % (seen[sig], nm))
        seen[sig] = nm

    # ---- primitives and UVs
    by_mat = {}
    dens = []
    for me in doc['meshes']:
        for p in me['primitives']:
            at = p['attributes']
            if 'TEXCOORD_0' not in at:
                E('mesh "%s" has a primitive without TEXCOORD_0' % me['name'])
                continue
            if any(k.startswith('COLOR_') for k in at):
                E('mesh "%s" has vertex colours' % me['name'])
            if p.get('extensions'):
                E('mesh "%s" primitive uses extensions (Draco?)' % me['name'])
            uv = accessor(doc, binary, at['TEXCOORD_0']).astype(np.float64)
            if uv.min() < 0 or uv.max() > 1:
                E('mesh "%s" has UVs outside 0-1' % me['name'])
            pos = accessor(doc, binary, at['POSITION']).astype(np.float64)
            idx = accessor(doc, binary, p['indices']).reshape(-1, 3).astype(np.int64)
            by_mat.setdefault(p['material'], []).append((uv * TEX, idx))
            P = pos[idx]
            A3 = np.linalg.norm(np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), axis=1) / 2
            U = uv[idx] * TEX
            A2 = np.abs((U[:, 1, 0] - U[:, 0, 0]) * (U[:, 2, 1] - U[:, 0, 1]) - (U[:, 1, 1] - U[:, 0, 1]) * (U[:, 2, 0] - U[:, 0, 0])) / 2
            ok = A3 > 1e-4
            dens.append(np.sqrt(A2[ok] / A3[ok]))
    worst_gap = 1e9
    for mi, items in by_mat.items():
        uv = np.concatenate([u for u, _ in items])
        offs = np.cumsum([0] + [len(u) for u, _ in items[:-1]])
        tris = np.concatenate([t + o for (_, t), o in zip(items, offs)])
        ov = raster_overlaps(uv[tris])
        if ov:
            E('material "%s": %d px of overlapping UVs' % (mats[mi]['name'], ov))
        boxes = islands(uv, tris)
        g = min_gap(boxes)
        border = min(min(b[0], b[1], TEX - b[2], TEX - b[3]) for b in boxes)
        worst_gap = min(worst_gap, g, 2 * border)
        if g < PAD - 1e-3:
            E('material "%s": islands only %.2f px apart (need %d)' % (mats[mi]['name'], g, PAD))
        if border < PAD - 1e-3:
            E('material "%s": island %.2f px from the texture edge' % (mats[mi]['name'], border))
    d = np.concatenate(dens)
    info.append('texel density %.0f px/m (5-95%%: %.0f-%.0f), min island gap %.1f px' %
                (np.median(d), np.percentile(d, 5), np.percentile(d, 95), worst_gap))
    info.append('materials: ' + ', '.join(m['name'] for m in mats))
    return errs, info


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    filt = sys.argv[1] if len(sys.argv) > 1 else ''
    bad = 0
    for path in sorted(glob.glob(os.path.join(here, '**', '*.glb'), recursive=True)):
        if filt not in path:
            continue
        errs, info = validate(path)
        print('%s  %s' % ('FAIL' if errs else 'PASS', os.path.relpath(path, here)))
        for line in info:
            print('      ' + line)
        for e in errs:
            print('    ! ' + e)
        bad += bool(errs)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
