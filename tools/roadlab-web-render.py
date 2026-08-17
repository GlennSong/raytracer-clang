#!/usr/bin/env python3
"""Draw an exported roadlab scene on a GPU, headlessly, and time it.

The last unanswered question about the marking evaluator was the only one that
needed hardware: not "does it translate" (roadlab-wgsl-run.py) and not "can a
fragment reach its boundaries" (roadlab-sampler-run.py), but "what does it cost
per fragment in a real frame". A software rasteriser cannot answer that and an
argument cannot either.

So this runs the actual pipeline — the exported mesh, one draw call, the shared
rl_paint_view.wgsl — into an offscreen texture, twice: once with the evaluator
live and once with it branched out. The difference between the two timings IS the
per-fragment cost of the markings, measured rather than estimated, and the PNG is
there so a number that looks good cannot come from a frame that is empty.

  roadlab --demo showcase --web out/showcase
  tools/roadlab-web-render.py --prefix out/showcase --out out/showcase.png

Exit codes: 0 rendered, 2 could not run (no wgpu, no adapter).
"""

import argparse
import json
import math
import os
import struct
import sys
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RL_DIR = os.path.join(ROOT, "proto", "roadlab")
SHADERS = ["rl_paint.wgsl", "rl_paint_sampler.wgsl", "rl_paint_view.wgsl"]

VERTEX_FLOATS = 12          # webexport.h kWebVertexFloats
VERTEX_BYTES = VERTEX_FLOATS * 4


def write_png(path, rgba, width, height):
    """A PNG in thirty lines, because Pillow is not worth a dependency here."""
    raw = bytearray()
    stride = width * 4
    for y in range(height):
        raw.append(0)                                   # filter: none
        raw += rgba[y * stride:(y + 1) * stride]

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    head = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", head))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 6)))
        f.write(chunk(b"IEND", b""))


# --- the camera, by hand (no numpy linear algebra worth importing for four rows)


def look_at_proj(eye, target, up, fov_y, aspect, near, far):
    def sub(a, b):
        return [a[i] - b[i] for i in range(3)]

    def norm(a):
        m = math.sqrt(sum(c * c for c in a)) or 1.0
        return [c / m for c in a]

    def cross(a, b):
        return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]]

    def dot(a, b):
        return sum(a[i] * b[i] for i in range(3))

    f = norm(sub(target, eye))
    s = norm(cross(f, up))
    u = cross(s, f)
    # WebGPU clip space: y up, z in [0, 1].
    g = 1.0 / math.tan(fov_y * 0.5)
    a = g / aspect
    b = far / (near - far)
    c = near * far / (near - far)
    # Column-major, the order WGSL's mat4x4 expects in a uniform buffer: each
    # line below is one COLUMN, not one row. w ends up as the distance along the
    # view direction, which must come out POSITIVE in front of the camera —
    # negate it and the whole scene lands behind the near plane and the frame is
    # pure clear colour, which times beautifully and shows nothing.
    return [
        a * s[0], u[0] * g, -f[0] * b, f[0],
        a * s[1], u[1] * g, -f[1] * b, f[1],
        a * s[2], u[2] * g, -f[2] * b, f[2],
        -a * dot(s, eye), -g * dot(u, eye), dot(f, eye) * b + c, -dot(f, eye),
    ]


def load(prefix, np):
    with open(prefix + ".json") as f:
        man = json.load(f)
    with open(prefix + ".bin", "rb") as f:
        blob = f.read()

    verts = blob[man["vertexOffset"]:man["vertexOffset"] + man["vertexCount"] * VERTEX_BYTES]
    idx = blob[man["indexOffset"]:man["indexOffset"] + man["indexCount"] * 4]

    def texture(offset, width, rows):
        a = np.frombuffer(blob, dtype=np.float32, count=width * rows * 4,
                          offset=offset).reshape(rows, width, 4)
        return np.ascontiguousarray(a)

    prof = texture(man["profileOffset"], man["profileTexWidth"], man["profileTexHeight"])
    style = texture(man["styleOffset"], man["styleWidth"], man["styleRows"])
    return man, verts, idx, prof, style


def upload_tex(device, wgpu, data):
    rows, width, _ = data.shape
    tex = device.create_texture(
        size=(width, rows, 1), format=wgpu.TextureFormat.rgba32float,
        usage=wgpu.TextureUsage.TEXTURE_BINDING | wgpu.TextureUsage.COPY_DST)
    device.queue.write_texture(
        {"texture": tex, "mip_level": 0, "origin": (0, 0, 0)}, data.tobytes(),
        {"offset": 0, "bytes_per_row": width * 16, "rows_per_image": rows},
        (width, rows, 1))
    return tex


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True, help="the prefix passed to `roadlab --web`")
    ap.add_argument("--out", default=None, help="PNG to write (default <prefix>.png)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--frames", type=int, default=60, help="timed frames per mode")
    ap.add_argument("--pitch", type=float, default=22.0, help="camera pitch, degrees")
    ap.add_argument("--yaw", type=float, default=35.0, help="camera yaw, degrees")
    ap.add_argument("--zoom", type=float, default=0.55,
                    help="eye distance as a fraction of the scene diagonal")
    args = ap.parse_args()
    out_png = args.out or (args.prefix + ".png")

    try:
        import numpy as np
        import wgpu
    except ImportError as e:
        print("cannot render: %s (pip install wgpu numpy)" % e)
        return 2
    try:
        adapter = wgpu.gpu.request_adapter_sync(power_preference="high-performance")
        device = adapter.request_device_sync()
    except Exception as e:                                   # noqa: BLE001
        print("cannot render: no WebGPU adapter (%s)" % e)
        return 2

    man, verts, idx, prof, style = load(args.prefix, np)
    source = ""
    for name in SHADERS:
        with open(os.path.join(RL_DIR, name)) as f:
            source += f.read()
    module = device.create_shader_module(code=source)

    U = wgpu.BufferUsage
    vbuf = device.create_buffer_with_data(data=verts, usage=U.VERTEX)
    ibuf = device.create_buffer_with_data(data=idx, usage=U.INDEX)
    ubuf = device.create_buffer(size=96, usage=U.UNIFORM | U.COPY_DST)

    V, F = wgpu.ShaderStage.VERTEX, wgpu.ShaderStage.FRAGMENT
    g0_layout = device.create_bind_group_layout(entries=[
        {"binding": 0, "visibility": V | F,
         "buffer": {"type": wgpu.BufferBindingType.uniform}}])
    g1_layout = device.create_bind_group_layout(entries=[
        {"binding": 0, "visibility": F,
         "texture": {"sample_type": wgpu.TextureSampleType.unfilterable_float}},
        {"binding": 1, "visibility": F,
         "texture": {"sample_type": wgpu.TextureSampleType.unfilterable_float}}])
    g0 = device.create_bind_group(layout=g0_layout, entries=[
        {"binding": 0, "resource": {"buffer": ubuf, "offset": 0, "size": 96}}])
    g1 = device.create_bind_group(layout=g1_layout, entries=[
        {"binding": 0, "resource": upload_tex(device, wgpu, prof).create_view()},
        {"binding": 1, "resource": upload_tex(device, wgpu, style).create_view()}])

    pipeline = device.create_render_pipeline(
        layout=device.create_pipeline_layout(bind_group_layouts=[g0_layout, g1_layout]),
        vertex={
            "module": module, "entry_point": "rlViewVertex",
            "buffers": [{
                "array_stride": VERTEX_BYTES,
                "step_mode": wgpu.VertexStepMode.vertex,
                "attributes": [
                    {"format": wgpu.VertexFormat.float32x3, "offset": 0, "shader_location": 0},
                    {"format": wgpu.VertexFormat.float32x3, "offset": 12, "shader_location": 1},
                    {"format": wgpu.VertexFormat.float32x3, "offset": 24, "shader_location": 2},
                    {"format": wgpu.VertexFormat.float32x3, "offset": 36, "shader_location": 3},
                ]}]},
        primitive={"topology": wgpu.PrimitiveTopology.triangle_list,
                   "cull_mode": wgpu.CullMode.none},
        depth_stencil={"format": wgpu.TextureFormat.depth24plus,
                       "depth_write_enabled": True,
                       "depth_compare": wgpu.CompareFunction.less},
        fragment={"module": module, "entry_point": "rlViewFragment",
                  "targets": [{"format": wgpu.TextureFormat.rgba8unorm}]})

    colour = device.create_texture(
        size=(args.width, args.height, 1), format=wgpu.TextureFormat.rgba8unorm,
        usage=wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.COPY_SRC)
    depth = device.create_texture(
        size=(args.width, args.height, 1), format=wgpu.TextureFormat.depth24plus,
        usage=wgpu.TextureUsage.RENDER_ATTACHMENT)

    lo, hi = man["bounds"]["lo"], man["bounds"]["hi"]
    centre = [(lo[i] + hi[i]) * 0.5 for i in range(3)]
    diag = math.sqrt(sum((hi[i] - lo[i]) ** 2 for i in range(3)))
    dist = diag * args.zoom
    pitch, yaw = math.radians(args.pitch), math.radians(args.yaw)
    eye = [centre[0] + dist * math.cos(pitch) * math.cos(yaw),
           centre[1] + dist * math.sin(pitch),
           centre[2] + dist * math.cos(pitch) * math.sin(yaw)]
    vp = look_at_proj(eye, centre, [0, 1, 0], math.radians(45.0),
                      args.width / args.height, max(diag * 0.001, 0.5), diag * 4.0)

    def draw(paint_on):
        u = struct.pack("<16f", *vp) + struct.pack("<4f", -0.4, 0.82, 0.4, 0.0) + \
            struct.pack("<4f", 1.0 if paint_on else 0.0, 0.25, 0.0, 0.0)
        device.queue.write_buffer(ubuf, 0, u)
        enc = device.create_command_encoder()
        rp = enc.begin_render_pass(
            color_attachments=[{
                "view": colour.create_view(), "load_op": wgpu.LoadOp.clear,
                "store_op": wgpu.StoreOp.store, "clear_value": (0.44, 0.55, 0.68, 1.0)}],
            depth_stencil_attachment={
                "view": depth.create_view(), "depth_load_op": wgpu.LoadOp.clear,
                "depth_store_op": wgpu.StoreOp.store, "depth_clear_value": 1.0})
        rp.set_pipeline(pipeline)
        rp.set_bind_group(0, g0)
        rp.set_bind_group(1, g1)
        rp.set_vertex_buffer(0, vbuf)
        rp.set_index_buffer(ibuf, wgpu.IndexFormat.uint32)
        rp.draw_indexed(man["indexCount"], 1, 0, 0, 0)
        rp.end()
        device.queue.submit([enc.finish()])

    def time_frames(paint_on, n):
        draw(paint_on)                       # warm the pipeline, then measure
        device._poll_wait()
        t0 = time.perf_counter()
        for _ in range(n):
            draw(paint_on)
        device._poll_wait()
        return (time.perf_counter() - t0) * 1000.0 / n

    on = time_frames(True, args.frames)
    off = time_frames(False, args.frames)

    draw(True)
    data = device.queue.read_texture(
        {"texture": colour, "mip_level": 0, "origin": (0, 0, 0)},
        {"offset": 0, "bytes_per_row": args.width * 4, "rows_per_image": args.height},
        (args.width, args.height, 1))
    write_png(out_png, bytes(data), args.width, args.height)

    px = np.frombuffer(bytes(data), dtype=np.uint8).reshape(-1, 4)
    # The frame has to contain the scene, not just the clear colour. A timing
    # from an empty frame is the most flattering wrong answer available.
    sky = np.array([112, 140, 173], dtype=np.int16)
    covered = float(np.count_nonzero(np.abs(px[:, :3].astype(np.int16) - sky).sum(1) > 12))
    covered /= px.shape[0]

    print("%s on %s (%s)" % (os.path.basename(args.prefix), adapter.info["device"],
                             adapter.info["backend_type"]))
    print("  %d verts, %d triangles, %dx%d" %
          (man["vertexCount"], man["indexCount"] // 3, args.width, args.height))
    print("  markings ON  : %8.2f ms/frame" % on)
    print("  markings OFF : %8.2f ms/frame" % off)
    print("  evaluator    : %8.2f ms/frame  (%+.1f%%)" % (on - off, 100.0 * (on / off - 1.0)))
    print("  scene covers %.1f%% of the frame" % (100.0 * covered))
    print("  wrote %s" % out_png)
    if adapter.info["adapter_type"] == "CPU":
        # Say it rather than let a reader take 20 ms for a GPU number. A software
        # rasteriser's absolute milliseconds mean nothing; what survives the move
        # to hardware is the RATIO, because both modes rasterise the same
        # triangles and differ only in the fragment branch.
        print("  NOTE: software rasteriser. The absolute times are meaningless;")
        print("        the percentage is the part that transfers to hardware.")
    if covered < 0.02:
        print("  the frame is almost entirely sky — the timings above mean nothing")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
