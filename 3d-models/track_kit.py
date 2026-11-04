"""Tracks built from individual shoe nodes that share one mesh, so a viewer can run them.

Layout written for every track group (e.g. `track_right`):

* the group node has no mesh; its origin is on the track centre line (x = y = 0, z = the
  track's z) and its children are the shoes, in order round the loop:
  `<group>_shoe_00`, `<group>_shoe_01`, ...
* every shoe node's origin sits on the chain pitch line (the belt path round the idler and
  sprocket pitch circles) and its rotation follows the loop: local +X along the tangent (the
  travel direction), local +Y inward (toward the rollers), local +Z = world +Z;
* the shoes are evenly spaced (loop length / shoe count, i.e. the pitch rounded to a whole
  number of shoes), counter-clockwise seen from +Z. Moving shoe N to shoe N+1's rest
  transform runs the track one shoe; looping that runs it continuously;
* every shoe of a group reuses one glTF mesh, `<group>_shoe`.

Usage in a build script:

    track_loop_nodes(m, 'track_right', parent, belt_path(circles), pitch, z, shoe_geos)
    m.mirror('right', 'left')        # mirrors the shoe nodes like every other part
    mirror_track_shoes(m)            # ... and registers the mirrored shoes
    save_with_instanced_tracks(m, 'Construction/<slug>', '<slug_with_underscores>')

This is the approach first used for the Mining dozer (`Mining/mining_parts.py`), generalised
so each track group gets its own shared mesh (rubber tracks with a handed tread are not
symmetric left to right) and so any pack folder can use it.
"""
import json
import math
import os
import struct

from meshkit import belt_path, from_columns, resample_loop, save

HERE = os.path.dirname(os.path.abspath(__file__))


def track_shoe_frame(tx, ty):
    """Rotation of a shoe at a loop point with unit tangent (tx, ty) in the x-y plane.
    Columns: local +X = travel tangent, local +Y = inward, local +Z = +Z (det +1)."""
    return from_columns((tx, ty, 0), (-ty, tx, 0), (0, 0, 1))


def loop_length(loop):
    return sum(math.dist(loop[i], loop[i - 1]) for i in range(len(loop)))


def track_loop_nodes(m, group, parent, loop, pitch, z, shoe_geos, count=None):
    """Add `group` (under `parent`) with one child node per shoe round the closed CCW `loop`.

    `shoe_geos` is one shoe authored in chain space: u (= x) along the chain, n (= y) outward
    from the loop, z across the track; its pitch-line point is the origin. The shoe count is
    `count`, or the loop length / `pitch` rounded. Returns the actual spacing."""
    n = count or round(loop_length(loop) / pitch)
    samples, total = resample_loop(loop, n)
    m.node(group, parent, (0.0, 0.0, z))
    if not hasattr(m, 'track_shoes'):
        m.track_shoes = {}
    for i, ((px, py), tx, ty) in enumerate(samples):
        name = '%s_shoe_%02d' % (group, i)
        node = m.node(name, group, (px, py, z))
        M = from_columns((tx, ty, 0), (ty, -tx, 0), (0, 0, 1))   # authoring frame (outward +Y)
        m.add(node, [g.transform(M, (px, py, z)) for g in shoe_geos])
        m.track_shoes[name] = (group, track_shoe_frame(tx, ty))
    return total / n


def rubber_track_nodes(m, group, parent, circles, z, width, thick, lug_h, pitch, pattern='bar'):
    """Rubber track wrapped round (x, y, r) circles (the wheel side of the belt), as a group
    of belt-segment shoes about `pitch` long (see `vehicle_parts.rubber_track_shoe`)."""
    from vehicle_parts import rubber_track_shoe
    loop = belt_path(circles)
    n = max(8, round(loop_length(loop) / pitch))
    p = loop_length(loop) / n
    return track_loop_nodes(m, group, parent, loop, p, z, rubber_track_shoe(p, width, thick, lug_h, pattern), n)


def mirror_track_shoes(m, src='right', dst='left'):
    """Register the shoes `Model.mirror` created. A z-mirror keeps every tangent in the x-y
    plane, so the mirrored shoe has the same rotation (and the same order round the loop)."""
    for name, (group, R) in list(m.track_shoes.items()):
        if src in name:
            m.track_shoes[name.replace(src, dst)] = (group.replace(src, dst), R)


def quat(R):
    """(x, y, z, w) quaternion of a 3x3 rotation matrix given as rows."""
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, 0.25 * s)
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        q = (0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s, (R[2][1] - R[1][2]) / s)
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        q = ((R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s, (R[0][2] - R[2][0]) / s)
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
        q = ((R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s, (R[1][0] - R[0][1]) / s)
    q = [c / math.sqrt(sum(k * k for k in q)) for c in q]
    if q[3] < 0:
        q = [-c for c in q]
    return [round(c, 7) for c in q]


def instance_track_shoes(path, shoes, templates):
    """Rewrite a written .glb so every shoe node reuses its group's template mesh.

    `shoes` maps shoe name -> (group, rotation); `templates` maps group -> template shoe name.
    Each template mesh is moved into its shoe's local frame and renamed `<group>_shoe`; every
    shoe node of the group gets that mesh and its rotation."""
    data = open(path, 'rb').read()
    jlen = struct.unpack('<I', data[12:16])[0]
    doc = json.loads(data[20:20 + jlen])
    blen = struct.unpack('<I', data[20 + jlen:24 + jlen])[0]
    blob = bytearray(data[28 + jlen:28 + jlen + blen])
    idx = {nd['name']: i for i, nd in enumerate(doc['nodes'])}
    mesh_of = {}
    for group, tname in templates.items():
        tnode = doc['nodes'][idx[tname]]
        mesh = doc['meshes'][tnode['mesh']]
        mesh['name'] = group + '_shoe'
        Rt = tuple(zip(*shoes[tname][1]))   # inverse = transpose
        for prim in mesh['primitives']:
            for key in ('POSITION', 'NORMAL'):
                acc = doc['accessors'][prim['attributes'][key]]
                off, cnt = doc['bufferViews'][acc['bufferView']]['byteOffset'], acc['count']
                vals = struct.unpack_from('<%df' % (cnt * 3), blob, off)
                out = []
                for i in range(cnt):
                    v = vals[3 * i:3 * i + 3]
                    out += [sum(Rt[r][c] * v[c] for c in range(3)) for r in range(3)]
                struct.pack_into('<%df' % (cnt * 3), blob, off, *out)
                if key == 'POSITION':
                    acc['min'] = [min(out[k::3]) for k in range(3)]
                    acc['max'] = [max(out[k::3]) for k in range(3)]
        mesh_of[group] = tnode['mesh']
    for name, (group, R) in shoes.items():
        nd = doc['nodes'][idx[name]]
        assert name == templates[group] or 'mesh' not in nd, name
        nd['mesh'] = mesh_of[group]
        nd['rotation'] = quat(R)
    js = json.dumps(doc, separators=(',', ':')).encode()
    js += b' ' * (-len(js) % 4)
    total = 12 + 8 + len(js) + 8 + len(blob)
    with open(path, 'wb') as f:
        f.write(struct.pack('<4sII', b'glTF', 2, total))
        f.write(struct.pack('<I4s', len(js), b'JSON') + js)
        f.write(struct.pack('<I4s', len(blob), b'BIN\0') + bytes(blob))


def save_with_instanced_tracks(model, folder, name):
    """Like `meshkit.save`, but every shoe of a track group shares one mesh (the group's
    first shoe). All shoes stay in the model until it is finalised, so bounds and centring
    see them; the others are emptied just before writing."""
    model.finalize()
    templates = {}
    for shoe, (group, _) in model.track_shoes.items():
        if group in templates:
            model.parts[shoe] = []
        else:
            templates[group] = shoe
    save(model, folder, name)
    instance_track_shoes(os.path.join(HERE, folder, name + '.glb'), model.track_shoes, templates)
