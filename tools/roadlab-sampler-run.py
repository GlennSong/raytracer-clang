#!/usr/bin/env python3
"""Run rlFetchBoundaries on a GPU and compare it to PaintAtlas::sample.

roadlab-wgsl-run.py already proves the marking EVALUATOR agrees CPU-to-GPU. But
it hands the shader its boundaries as a storage buffer, so the other half of the
contract has never executed: the fetch that reads those boundaries out of two
textures, blending between rings on one axis, snapping to an index on the other,
and skipping padding slots that must not be mistaken for unpainted ones. That
half lives in rl_paint_sampler.wgsl, which until now was only ever parsed.

This runs it. `roadlab --web <prefix>` writes the atlas beside a fixture of CPU
answers at deliberately fractional rows — between rings, where the blend
actually does something and where a swapped axis hides — plus rows straddling
every style-row seam. This replays them through a compute pass that calls
rlFetchBoundaries and nothing else.

  roadlab --demo showcase --web /tmp/showcase
  tools/roadlab-sampler-run.py --prefix /tmp/showcase

Exit codes: 0 agreed, 1 disagreed, 2 could not run (no wgpu, no adapter).
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RL_DIR = os.path.join(ROOT, "proto", "roadlab")

RL_MAX_BOUNDS = 12
BOUNDARY_FLOATS = 8

# Thin on purpose: one dispatch per case, calling the shared fetch and dumping
# exactly what it produced. Anything this harness computes itself is something
# the comparison stops proving.
HARNESS = """
@group(0) @binding(0) var<storage, read> rlRows : array<f32>;
@group(0) @binding(1) var<storage, read_write> rlCounts : array<i32>;
@group(0) @binding(2) var<storage, read_write> rlOut : array<RlBoundary>;

@compute @workgroup_size(64)
fn rlSampleMain(@builtin(global_invocation_id) gid : vec3<u32>) {
  let i = gid.x;
  if (i >= arrayLength(&rlRows)) { return; }

  var bounds : array<RlBoundary, RL_MAX_BOUNDS>;
  let count = rlFetchBoundaries(rlRows[i], &bounds);
  rlCounts[i] = count;
  for (var k : i32 = 0; k < RL_MAX_BOUNDS; k = k + 1) {
    rlOut[i * u32(RL_MAX_BOUNDS) + u32(k)] = bounds[k];
  }
}
"""

FIELDS = ["t", "style", "width", "gap", "dashOn", "dashOff", "wear", "color"]
# Only `t` is blended. Everything else is a nearest fetch out of the style table,
# so "close" is not a category it has: those must come back bit-identical or the
# fetch read the wrong texel, and a tolerance would hide exactly that.
EXACT = set(FIELDS) - {"t"}


def load(prefix, np):
    with open(prefix + ".json") as f:
        man = json.load(f)
    with open(prefix + ".samples.json") as f:
        cases = json.load(f)["cases"]
    with open(prefix + ".bin", "rb") as f:
        blob = f.read()

    def texture(offset, width, rows):
        n = width * rows * 4
        arr = np.frombuffer(blob, dtype=np.float32, count=n,
                            offset=offset).reshape(rows, width, 4)
        return np.ascontiguousarray(arr)

    prof = texture(man["profileOffset"], man["profileTexWidth"], man["profileTexHeight"])
    style = texture(man["styleOffset"], man["styleWidth"], man["styleRows"])
    if man["maxBounds"] != RL_MAX_BOUNDS:
        raise SystemExit("manifest says RL_MAX_BOUNDS=%d, this harness assumes %d"
                         % (man["maxBounds"], RL_MAX_BOUNDS))
    return man, cases, prof, style


def upload(device, wgpu, data):
    rows, width, _ = data.shape
    tex = device.create_texture(
        size=(width, rows, 1), format=wgpu.TextureFormat.rgba32float,
        usage=wgpu.TextureUsage.TEXTURE_BINDING | wgpu.TextureUsage.COPY_DST)
    device.queue.write_texture(
        {"texture": tex, "mip_level": 0, "origin": (0, 0, 0)},
        data.tobytes(),
        {"offset": 0, "bytes_per_row": width * 16, "rows_per_image": rows},
        (width, rows, 1))
    return tex


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True,
                    help="the prefix passed to `roadlab --web`")
    ap.add_argument("--tolerance", type=float, default=1e-5,
                    help="max allowed difference on a metre-valued field")
    args = ap.parse_args()

    try:
        import numpy as np
        import wgpu
    except ImportError as e:
        print("cannot run the sampler: %s (pip install wgpu numpy)" % e)
        return 2

    try:
        adapter = wgpu.gpu.request_adapter_sync(power_preference="high-performance")
    except Exception as e:                                   # noqa: BLE001
        print("cannot run the sampler: no WebGPU adapter (%s)" % e)
        return 2

    # No required features on purpose. The fetch reads the offset texture with
    # textureLoad and blends by hand, so RGBA32F never has to be filterable —
    # which is the whole point, since float32-filterable is optional in WebGPU
    # and a browser without it would otherwise have no way to run this at all.
    device = adapter.request_device_sync()

    man, cases, prof, style = load(args.prefix, np)
    with open(os.path.join(RL_DIR, "rl_paint.wgsl")) as f:
        source = f.read()
    with open(os.path.join(RL_DIR, "rl_paint_sampler.wgsl")) as f:
        source += f.read()
    module = device.create_shader_module(code=source + HARNESS)

    count = len(cases)
    rows = np.array([c["row"] for c in cases], dtype=np.float32)

    S = wgpu.BufferUsage.STORAGE
    row_buf = device.create_buffer_with_data(data=rows.tobytes(), usage=S)
    cnt_buf = device.create_buffer(size=count * 4, usage=S | wgpu.BufferUsage.COPY_SRC)
    out_buf = device.create_buffer(size=count * RL_MAX_BOUNDS * BOUNDARY_FLOATS * 4,
                                   usage=S | wgpu.BufferUsage.COPY_SRC)

    C = wgpu.ShaderStage.COMPUTE
    g0_layout = device.create_bind_group_layout(entries=[
        {"binding": 0, "visibility": C,
         "buffer": {"type": wgpu.BufferBindingType.read_only_storage}},
        {"binding": 1, "visibility": C,
         "buffer": {"type": wgpu.BufferBindingType.storage}},
        {"binding": 2, "visibility": C,
         "buffer": {"type": wgpu.BufferBindingType.storage}},
    ])
    g1_layout = device.create_bind_group_layout(entries=[
        {"binding": 0, "visibility": C,
         "texture": {"sample_type": wgpu.TextureSampleType.unfilterable_float}},
        {"binding": 1, "visibility": C,
         "texture": {"sample_type": wgpu.TextureSampleType.unfilterable_float}},
    ])

    prof_tex = upload(device, wgpu, prof)
    style_tex = upload(device, wgpu, style)
    g0 = device.create_bind_group(layout=g0_layout, entries=[
        {"binding": 0, "resource": {"buffer": row_buf, "offset": 0, "size": row_buf.size}},
        {"binding": 1, "resource": {"buffer": cnt_buf, "offset": 0, "size": cnt_buf.size}},
        {"binding": 2, "resource": {"buffer": out_buf, "offset": 0, "size": out_buf.size}},
    ])
    g1 = device.create_bind_group(layout=g1_layout, entries=[
        {"binding": 0, "resource": prof_tex.create_view()},
        {"binding": 1, "resource": style_tex.create_view()},
    ])

    pipeline = device.create_compute_pipeline(
        layout=device.create_pipeline_layout(bind_group_layouts=[g0_layout, g1_layout]),
        compute={"module": module, "entry_point": "rlSampleMain"})

    encoder = device.create_command_encoder()
    cpass = encoder.begin_compute_pass()
    cpass.set_pipeline(pipeline)
    cpass.set_bind_group(0, g0)
    cpass.set_bind_group(1, g1)
    cpass.dispatch_workgroups((count + 63) // 64)
    cpass.end()
    device.queue.submit([encoder.finish()])

    got_n = np.frombuffer(device.queue.read_buffer(cnt_buf), dtype=np.int32)
    got_b = np.frombuffer(device.queue.read_buffer(out_buf),
                          dtype=np.float32).reshape(count, RL_MAX_BOUNDS, BOUNDARY_FLOATS)

    bad_count = []
    worst = {f: (0.0, -1, -1) for f in FIELDS}
    exact = {f: 0 for f in FIELDS}
    compared = 0
    for i, case in enumerate(cases):
        if int(got_n[i]) != case["n"]:
            bad_count.append((i, case["row"], case["n"], int(got_n[i])))
            continue
        for bi, want in enumerate(case["b"]):
            compared += 1
            for fi, name in enumerate(FIELDS):
                # The fixture holds f32s printed with %.9g, so they round-trip
                # exactly — but only back into an f32. Parsing them as Python
                # doubles and comparing there manufactures a 3e-10 "difference"
                # that is entirely the decimal, not the GPU.
                d = abs(float(got_b[i, bi, fi]) - float(np.float32(want[fi])))
                if d > worst[name][0]:
                    worst[name] = (d, i, bi)
                if d == 0.0:
                    exact[name] += 1

    print("%d fetch cases from %s on %s (%s)" %
          (count, os.path.basename(args.prefix), adapter.info["device"],
           adapter.info["backend_type"]))
    print("  atlas: %d profile rows, tiled %d wide into a %dx%d texture; "
          "%d style rows x %d texels" %
          (man["profileRows"], man["rowsPerTile"], man["profileTexWidth"],
           man["profileTexHeight"], man["styleRows"], man["styleWidth"]))
    print("  boundary count mismatches : %d" % len(bad_count))
    print("  boundaries compared       : %d" % compared)
    ok = not bad_count
    # Per field, not per boundary. `t` comes off a lerp and may differ in the last
    # place; everything else comes off a nearest fetch and has no excuse — a single inexact style is a bug, so it gets its own column
    # rather than being averaged into one reassuring percentage.
    for name in FIELDS:
        d, i, bi = worst[name]
        limit = 0.0 if name in EXACT else args.tolerance
        how = "nearest, must be exact" if name in EXACT else "blended between rings"
        flag = "" if d <= limit else "   <-- OVER"
        print("    %-8s worst %.3e   exact %d/%d   (%s)%s" %
              (name, d, exact[name], compared, how, flag))
        if d > limit:
            ok = False
            print("      case %d row %.4f slot %d: cpu %s gpu %s" %
                  (i, cases[i]["row"], bi, cases[i]["b"][bi],
                   [float(x) for x in got_b[i, bi]]))
    for i, row, want_n, got in bad_count[:5]:
        print("    case %d row %.4f: cpu found %d boundaries, gpu found %d"
              % (i, row, want_n, got))

    print("rlFetchBoundaries and PaintAtlas::sample %s" % ("agree" if ok else "DISAGREE"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
