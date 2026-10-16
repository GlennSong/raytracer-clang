#!/usr/bin/env python3
"""Write a sculpted tower MASS as a glTF binary for the toolkit's mesh mass (mesh_mass.h, issue #105).

    python3 tools/make_mass.py assets/masses/sculpted_01.glb

The form: a lathe of a profile r(y) -- a waist at the foot (0.77 of the belly), the belly at half the height,
closing to 0.6 at the top -- with a soft three-lobed section that turns 70 degrees over the height. 20 units tall,
about 12 across at the belly; the engine fits it to the lot's box and the storeys' height (mesh_mass.h), and
widens any storey too narrow for the core, so only the shape matters. Y up, as glTF and the engine
read it. One mesh, one primitive, positions + normals, no material (the grammar dresses it in glass).
"""
import json, math, struct, sys

def build(H=20.0, around=48, rings=60):
    verts, norms, tris = [], [], []
    rows = []
    for j in range(rings + 1):
        t = j / rings
        y = H * t
        r_prof = 3.6 + 1.8 * math.sin(math.pi * (0.1 + 0.8 * t)) - 0.9 * max(0.0, t - 0.8) * 5.0
        twist = math.radians(70.0) * t
        row = []
        for i in range(around):
            a = 2.0 * math.pi * i / around
            lobe = 1.0 + 0.16 * math.cos(3.0 * (a - twist))
            r = r_prof * lobe
            row.append(len(verts))
            verts.append((math.cos(a) * r, y, math.sin(a) * r))
        rows.append(row)
    for j in range(rings):
        for i in range(around):
            a, b = rows[j][i], rows[j][(i + 1) % around]
            c, d = rows[j + 1][(i + 1) % around], rows[j + 1][i]
            tris.append((a, b, c)); tris.append((a, c, d))
    # caps: a centre vertex each
    bot = len(verts); verts.append((0.0, 0.0, 0.0))
    top = len(verts); verts.append((0.0, H, 0.0))
    for i in range(around):
        tris.append((bot, rows[0][(i + 1) % around], rows[0][i]))
        tris.append((top, rows[rings][i], rows[rings][(i + 1) % around]))
    # vertex normals from the faces
    acc = [[0.0, 0.0, 0.0] for _ in verts]
    for (a, b, c) in tris:
        pa, pb, pc = verts[a], verts[b], verts[c]
        u = [pb[k] - pa[k] for k in range(3)]; v = [pc[k] - pa[k] for k in range(3)]
        n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        for idx in (a, b, c):
            for k in range(3): acc[idx][k] += n[k]
    for n in acc:
        L = math.sqrt(sum(x * x for x in n)) or 1.0
        norms.append(tuple(x / L for x in n))
    return verts, norms, tris

def write_glb(path, verts, norms, tris):
    pos = b"".join(struct.pack("<3f", *v) for v in verts)
    nrm = b"".join(struct.pack("<3f", *n) for n in norms)
    idx = b"".join(struct.pack("<3I", *t) for t in tris)
    def pad4(b): return b + b"\0" * ((4 - len(b) % 4) % 4)
    pos, nrm, idx = pad4(pos), pad4(nrm), pad4(idx)
    bin_chunk = pos + nrm + idx
    mins = [min(v[k] for v in verts) for k in range(3)]
    maxs = [max(v[k] for v in verts) for k in range(3)]
    gltf = {
        "asset": {"version": "2.0", "generator": "raytracer-clang tools/make_mass.py"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0, "name": "sculpted_mass"}],
        "meshes": [{"name": "sculpted_mass", "primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1}, "indices": 2, "mode": 4}]}],
        "buffers": [{"byteLength": len(bin_chunk)}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(pos), "target": 34962},
                        {"buffer": 0, "byteOffset": len(pos), "byteLength": len(nrm), "target": 34962},
                        {"buffer": 0, "byteOffset": len(pos) + len(nrm), "byteLength": len(idx), "target": 34963}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": len(verts), "type": "VEC3", "min": mins, "max": maxs},
                      {"bufferView": 1, "componentType": 5126, "count": len(norms), "type": "VEC3"},
                      {"bufferView": 2, "componentType": 5125, "count": len(tris) * 3, "type": "SCALAR"}],
    }
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js = js + b" " * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(bin_chunk)
    with open(path, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total))
        f.write(struct.pack("<I4s", len(js), b"JSON")); f.write(js)
        f.write(struct.pack("<I4s", len(bin_chunk), b"BIN\0")); f.write(bin_chunk)
    return total

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "assets/masses/sculpted_01.glb"
    v, n, t = build()
    size = write_glb(out, v, n, t)
    print(f"{out}: {len(v)} vertices, {len(t)} triangles, {size} bytes")
