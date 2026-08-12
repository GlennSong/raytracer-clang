#!/usr/bin/env python3
"""Run the generated WGSL marking evaluator and compare it to the C++ one.

Everything else about the port is checked structurally: rl_paint.wgsl is
generated from rl_paint.h, a digest catches drift, naga parses it and lowers it
to SPIR-V. None of that says the two compute the same numbers — a translation can
be flawless WGSL and still paint a different road.

So this executes the shader. `roadlab --paint-fixture` dumps a corpus of inputs
with the CPU's answers; this replays them through a compute pipeline and diffs.
It needs no hardware: wgpu falls back to Vulkan on llvmpipe, so it runs in CI on
a machine with no GPU at all.

  make roadlab-wgsl-conformance          # build the fixture and run it
  tools/roadlab-wgsl-run.py --fixture f  # replay an existing one

Exit codes: 0 agreed, 1 disagreed, 2 could not run (no wgpu, no adapter).
"""

import argparse
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WGSL = os.path.join(ROOT, "proto", "roadlab", "rl_paint.wgsl")

RL_MAX_BOUNDS = 12
CASE_FLOATS = 6      # s t fw roadWear wearMask wheel
CASE_INTS = 2        # count, pad
CASE_BYTES = (CASE_FLOATS + CASE_INTS) * 4
BOUNDARY_BYTES = 8 * 4

# The harness around the generated evaluator. Deliberately thin: it copies the
# case's boundaries into a function-scope array and calls the shared function, so
# what runs on the GPU is the generated code and nothing else.
HARNESS = """
struct RlCase {
  s : f32,
  t : f32,
  fw : f32,
  roadWear : f32,
  wearMask : f32,
  wheel : f32,
  count : i32,
  pad : i32,
};

@group(0) @binding(0) var<storage, read> rlCases : array<RlCase>;
@group(0) @binding(1) var<storage, read> rlBounds : array<RlBoundary>;
@group(0) @binding(2) var<storage, read_write> rlOut : array<vec4<f32>>;

@compute @workgroup_size(64)
fn rlMain(@builtin(global_invocation_id) gid : vec3<u32>) {
  let i = gid.x;
  if (i >= arrayLength(&rlCases)) { return; }
  let c = rlCases[i];

  var local : array<RlBoundary, RL_MAX_BOUNDS>;
  for (var k : i32 = 0; k < RL_MAX_BOUNDS; k = k + 1) {
    local[k] = rlBounds[i * u32(RL_MAX_BOUNDS) + u32(k)];
  }

  let p = rlEvaluateMarkings(&local, c.count, c.s, c.t, c.fw,
                             c.roadWear, c.wearMask, c.wheel);
  rlOut[i] = vec4<f32>(p.coverage, p.r, p.g, p.b);
}
"""


def read_fixture(path):
    with open(path, "rb") as f:
        blob = f.read()
    magic, count = struct.unpack_from("<ii", blob, 0)
    if magic != 0x524C5058:
        raise SystemExit("%s is not a roadlab paint fixture" % path)
    off = 8
    cases = blob[off:off + count * CASE_BYTES]
    off += count * CASE_BYTES
    bounds = blob[off:off + count * RL_MAX_BOUNDS * BOUNDARY_BYTES]
    off += count * RL_MAX_BOUNDS * BOUNDARY_BYTES
    expected = blob[off:off + count * 4 * 4]
    off += count * 4 * 4
    if off != len(blob):
        raise SystemExit("%s is %d bytes, expected %d" % (path, len(blob), off))
    return count, cases, bounds, expected


FMA_PROBE = """
@group(0) @binding(0) var<storage, read_write> o : array<f32>;
@compute @workgroup_size(1)
fn main() {
  // 270.8067 is a dash phase 386 m along a road; 1.4 is the hatch period. The
  // product 1.4 * 193 needs more mantissa than an f32 has, so a real fma and a
  // multiply-then-subtract differ in the last place OF THE PRODUCT — which is
  // an ulp at 270, landing whole in an answer of magnitude 0.6.
  o[0] = fma(-1.4, 193.0, 270.8067016601562);
}
"""

# The correctly-rounded value of that fma, computed exactly.
FMA_EXPECTED = 0.6067062616348267


def device_honours_fma(device, wgpu, np):
    """Does this device compute fma() as a single rounding, or lower it?

    Not pedantry — it decides what this comparison can prove. rl_paint.h uses an
    explicit fma in rlWrap because the periodic styles need the product computed
    at full width; a backend that lowers it to a multiply and a subtract is
    evaluating a different expression, and will differ from the CPU by an ulp of
    the product no matter how correct the translation is.

    llvmpipe, which is what this runs on with no hardware, does lower it. Rather
    than loosen the tolerance for everyone, detect it and say so."""
    module = device.create_shader_module(code=FMA_PROBE)
    buf = device.create_buffer(size=4, usage=wgpu.BufferUsage.STORAGE |
                               wgpu.BufferUsage.COPY_SRC)
    layout = device.create_bind_group_layout(entries=[
        {"binding": 0, "visibility": wgpu.ShaderStage.COMPUTE,
         "buffer": {"type": wgpu.BufferBindingType.storage}}])
    bind = device.create_bind_group(layout=layout, entries=[
        {"binding": 0, "resource": {"buffer": buf, "offset": 0, "size": 4}}])
    pipeline = device.create_compute_pipeline(
        layout=device.create_pipeline_layout(bind_group_layouts=[layout]),
        compute={"module": module, "entry_point": "main"})
    enc = device.create_command_encoder()
    cp = enc.begin_compute_pass()
    cp.set_pipeline(pipeline)
    cp.set_bind_group(0, bind)
    cp.dispatch_workgroups(1)
    cp.end()
    device.queue.submit([enc.finish()])
    got = float(np.frombuffer(device.queue.read_buffer(buf), dtype=np.float32)[0])
    return got == np.float32(FMA_EXPECTED)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fixture", required=True)
    ap.add_argument("--tolerance", type=float, default=None,
                    help="max allowed coverage difference; default depends on "
                         "whether the device honours fma")
    args = ap.parse_args()

    try:
        import wgpu
        import numpy as np
    except ImportError as e:
        print("cannot run the WGSL: %s (pip install wgpu numpy)" % e)
        return 2

    try:
        adapter = wgpu.gpu.request_adapter_sync(power_preference="high-performance")
        device = adapter.request_device_sync()
    except Exception as e:                                   # noqa: BLE001
        print("cannot run the WGSL: no WebGPU adapter (%s)" % e)
        return 2

    fused = device_honours_fma(device, wgpu, np)
    # A device that fuses evaluates the same expression the C++ reference does,
    # so anything past a couple of ulp is a translation bug. One that lowers fma
    # cannot, and the gap it opens scales with the station — measured at 2.7e-4
    # on a hatch 386 m along. Loosening the bound there is the honest reading;
    # loosening it everywhere would hide a real defect on real hardware.
    tolerance = args.tolerance if args.tolerance is not None else (2e-6 if fused else 1e-3)

    count, cases, bounds, expected = read_fixture(args.fixture)
    with open(WGSL) as f:
        source = f.read() + HARNESS
    module = device.create_shader_module(code=source)

    def storage(data, usage):
        return device.create_buffer_with_data(data=data, usage=usage)

    S = wgpu.BufferUsage.STORAGE
    case_buf = storage(cases, S)
    bound_buf = storage(bounds, S)
    out_buf = device.create_buffer(size=count * 16,
                                   usage=S | wgpu.BufferUsage.COPY_SRC)

    layout = device.create_bind_group_layout(entries=[
        {"binding": 0, "visibility": wgpu.ShaderStage.COMPUTE,
         "buffer": {"type": wgpu.BufferBindingType.read_only_storage}},
        {"binding": 1, "visibility": wgpu.ShaderStage.COMPUTE,
         "buffer": {"type": wgpu.BufferBindingType.read_only_storage}},
        {"binding": 2, "visibility": wgpu.ShaderStage.COMPUTE,
         "buffer": {"type": wgpu.BufferBindingType.storage}},
    ])
    bind = device.create_bind_group(layout=layout, entries=[
        {"binding": 0, "resource": {"buffer": case_buf, "offset": 0, "size": case_buf.size}},
        {"binding": 1, "resource": {"buffer": bound_buf, "offset": 0, "size": bound_buf.size}},
        {"binding": 2, "resource": {"buffer": out_buf, "offset": 0, "size": out_buf.size}},
    ])
    pipeline = device.create_compute_pipeline(
        layout=device.create_pipeline_layout(bind_group_layouts=[layout]),
        compute={"module": module, "entry_point": "rlMain"})

    encoder = device.create_command_encoder()
    cpass = encoder.begin_compute_pass()
    cpass.set_pipeline(pipeline)
    cpass.set_bind_group(0, bind)
    cpass.dispatch_workgroups((count + 63) // 64)
    cpass.end()
    device.queue.submit([encoder.finish()])

    got = np.frombuffer(device.queue.read_buffer(out_buf), dtype=np.float32).reshape(-1, 4)
    want = np.frombuffer(expected, dtype=np.float32).reshape(-1, 4)

    dcov = np.abs(got[:, 0] - want[:, 0])
    worst = float(dcov.max())
    # Colour is a palette lookup, so it is either the same entry or a different
    # one — no tolerance applies. The branch that picks it compares against half
    # the accumulated coverage, so an exact tie can legitimately fall either way;
    # count those separately from a real disagreement.
    colour_diff = np.any(np.abs(got[:, 1:] - want[:, 1:]) > 1e-6, axis=1)
    painted = want[:, 0] > 0.01
    colour_bad = int(np.count_nonzero(colour_diff & painted))

    exact = int(np.count_nonzero(dcov == 0.0))
    print("%d cases on %s (%s)" % (count, adapter.info["device"], adapter.info["backend_type"]))
    print("  fma is %s by this device" % ("fused" if fused else "LOWERED to mul+add"))
    print("  worst coverage difference : %.3e  (tolerance %.0e)" % (worst, tolerance))
    print("  mean coverage difference  : %.3e" % float(dcov.mean()))
    print("  bit-exact                 : %d / %d  (%.2f%%)" %
          (exact, count, 100.0 * exact / count))
    print("  colour mismatches on painted fragments : %d" % colour_bad)

    # The bit-exact fraction is the guard the tolerance cannot be: a translation
    # bug that shifts every fragment slightly would sit under any tolerance
    # loose enough for float noise, but it cannot leave most of the corpus
    # matching to the last bit.
    exact_ok = exact >= 0.9 * count
    ok = worst <= tolerance and colour_bad == 0 and exact_ok
    if not exact_ok:
        print("  only %.1f%% bit-exact — expected >=90%%; that is a systematic "
              "difference, not rounding" % (100.0 * exact / count))
    if not ok:
        bad = int(np.argmax(dcov))
        s, t, fw, rw, wm, wh, cnt, _ = struct.unpack_from("<6f2i", cases, bad * CASE_BYTES)
        print("\nworst case #%d: s=%g t=%g fw=%g wear=%g mask=%g wheel=%g count=%d"
              % (bad, s, t, fw, rw, wm, wh, cnt))
        print("  cpu %s\n  gpu %s" % (want[bad], got[bad]))
    print("WGSL and C++ %s" % ("agree" if ok else "DISAGREE"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
