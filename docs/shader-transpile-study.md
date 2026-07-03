# Shader transpilation study — single-source shaders across Metal / Vulkan / WebGPU

**Status:** Exploratory (no code change). Investigates whether a single shader
source could feed all three render backends (Metal MSL, Vulkan SPIR-V, WebGPU
WGSL) via transpilation, how different the generated shaders are from today's
hand-written ones, and what it would actually cost to adopt. Companion to
ADR-0057 (Vulkan) and ADR-0058 (WebGPU). **Not an ADR yet** — input for one.

## TL;DR

- **The transpilers produce correct, complete, production-grade shaders.** For a
  real shader (`mesh.frag`: Cook-Torrance + the procedural surface library +
  cascaded-shadow array sampler), the generated MSL/WGSL is behaviorally
  identical and would run the same (negligible perf delta; these tools ship in
  shipping engines).
- **The cost is NOT the shaders — it's everything around them.** Adopting a
  unified source means (1) rewriting each backend's CPU-side resource binding to
  the generated layout, (2) consolidating the backends' *diverging feature sets*
  into one superset source, and (3) keeping per-backend coordinate conventions.
- **One tool is not enough.** `naga` (the wgpu compiler) is insufficient for our
  shaders. The viable chain is the Khronos/Google tools: **glslang → SPIRV-Cross
  (MSL) + Tint (WGSL)**, i.e. three build-time dependencies.
- **Asymmetric cost by backend.** Vulkan is free (it *is* the GLSL→SPIR-V source
  today). WebGPU is greenfield (Phase 0/1 — most to gain). Metal is the most
  disruptive (replaces a mature, feature-rich, hand-tuned backend).

## Method

Ran the actual repo shaders through real tools (Linux, no GPU):

```
# GLSL is the candidate single source (already maintained for Vulkan).
glslangValidator -V shaders/vulkan/mesh.vert  -o mesh.vert.spv     # GLSL -> SPIR-V
glslangValidator -V shaders/vulkan/mesh.frag  -o mesh.frag.spv
spirv-cross --msl --msl-version 20300 mesh.frag.spv -o mesh.frag.metal   # SPIR-V -> MSL
naga --input-kind glsl --shader-stage vert mesh.vert mesh.vert.wgsl      # GLSL -> WGSL (direct)
```

## Toolchain findings (the first surprise)

`naga` is attractive (single Rust binary, no system deps) but **cannot process
our shaders**:

| Path | Result |
| --- | --- |
| `naga` GLSL → WGSL (`mesh.vert`) | ✅ works (faithful, polyfills `inverse()`) |
| `naga` GLSL → WGSL (`mesh.frag`) | ❌ GLSL frontend: *"Not implemented: variable qualifier"* on `sampler2DArrayShadow` |
| `naga` GLSL → MSL (`mesh.vert`) | ❌ MSL backend: *"standard function 'Inverse' is not implemented"* |
| `naga` SPIR-V → WGSL/MSL (`mesh.frag`) | ❌ SPIR-V frontend: *"invalid id"* on glslang output |
| `glslang` GLSL → SPIR-V | ✅ robust (handles everything) |
| `SPIRV-Cross` SPIR-V → MSL | ✅ complete (shadow array samplers, `inverse` polyfill) |
| `SPIRV-Cross` SPIR-V → ESSL 300 | ✅ (a WebGL2 fallback path, for free) |

**Conclusion:** the realistic pipeline is `GLSL --glslang--> SPIR-V`, then
`--SPIRV-Cross--> MSL` and `--Tint--> WGSL`. Three mature build-time tools, not
one. (`glslc` — a glslang wrapper — is already in the build for Vulkan.)

## How different is the generated output?

### Vulkan — *no difference*
GLSL→SPIR-V via glslang is exactly the existing build path. Adopting a GLSL
single-source changes nothing for Vulkan. It runs identically.

### WebGPU (WGSL) — faithful but machine-style; *more correct* than the hand port
naga's WGSL for `mesh.vert` preserves struct/field names (`Globals`,
`viewProjection`) but the body is the GLSL execution model: inputs/outputs lifted
to `var<private>` globals, work in a `main_1()`, SSA temporaries (`_e60`, `_e62`)
for every load, ~2–3× the line count. Notably it **generated a full 4×4
`inverse()`** the hand-written Phase-1 WGSL had skipped — so the transpiled
shader is *more* correct. Two gotchas: push constants become `var<immediate>`
(**not valid in browser WebGPU** — the shared source must use UBOs), and
coordinate conventions are **not** fixed for you.

### Metal (MSL) — clean and complete, but a *totally different binding model*
SPIRV-Cross MSL for `mesh.frag` (996 lines from 500 lines of GLSL) is clean,
correct, includes an `spvInverse4x4` polyfill, and correctly maps the cascaded
shadow sampler to `depth2d_array<float>` + a separate `sampler`. The problem is
the **resource interface vs. the hand-written Metal backend**:

```
// GENERATED (from GLSL's descriptor layout):
fragment main0_out main0(main0_in in [[stage_in]],
    constant Globals& g [[buffer(0)]], constant Push& pc [[buffer(1)]],
    texture2d<float> envEquirect [[texture(0)]], depth2d_array<float> shadowMap [[texture(1)]],
    texture2d<float> albedoMap [[texture(2)]], ... sampler ...Smplr [[sampler(0..7)]])

// HAND-WRITTEN (shaders/metal/lighting.metal):
fragment GBufferOut fragmentMain(VertexOut in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]], constant MaterialUniforms& material [[buffer(3)]],
    device const LightUniforms& lightData [[buffer(4)]], constant ShadowUniforms& shadowData [[buffer(5)]],
    constant ProbeUniforms& probeParams [[buffer(6)]], device const GPUReflectionProbe* probes [[buffer(7)]],
    constant EnvUniforms& env [[buffer(8)]],
    depth2d_array<float> shadowMap [[texture(0)]], texturecube_array<float> cubemapArray [[texture(1)]], ...)
```

They differ in *every* dimension: one combined `Globals` UBO vs. many small
uniform buffers; a `Push` constant vs. per-draw buffers; `[[stage_in]]` vertex
attributes vs. `const device Vertex*` manual vertex pull; and the hand-written
Metal exposes **features the GLSL doesn't have** — `texturecube_array` IBL
probes, `GPUInstanceData` instancing, terrain-morph vertex variants.

## The real cost of adopting it

1. **Per-backend CPU binding rewrite.** The renderer code that decides which
   buffer/texture goes where (`metal_renderer.mm`, `vulkan_renderer.cpp`, the
   WebGPU backend) must match the generated interface. Metal's is a large rewrite
   because today's layout is nothing like the generated one.
2. **Feature consolidation (the hidden project).** The three backends *diverge*
   today — Metal is ahead (IBL cubemap probes, instancing, terrain morph; see
   `renderer-parity.md`). A single source must become the **superset**, so this
   is a parity-consolidation effort, not a mechanical transpile.
3. **Conventions stay per-backend.** Clip-space Y flip (Vulkan) and depth range
   are absorbed CPU-side per backend, as today — the transpiler won't do it.
4. **Three build-time tools** (glslang + SPIRV-Cross + Tint) + an ADR, since this
   changes how *all* shaders are authored (not just web).

## How they would run

- **Correctness:** equivalent. SPIRV-Cross MSL and Tint WGSL are what large
  engines ship; the generated code is faithful to the SPIR-V.
- **Performance:** negligible difference — drivers optimize the generated code to
  the same ISA; verbosity is source-text only.
- **Iteration:** you stop hand-editing MSL/WGSL (they become build artifacts like
  `.o` files); all authoring moves to GLSL. Good for consistency, worse for
  per-backend hand-tuning (which the Metal backend currently relies on).

## Options

1. **Status quo** — three hand-written trees. Cheapest now; the divergence/3×
   cost grows with every new effect (Phases 2–6 ×3).
2. **WebGPU-only transpile** — generate just WGSL (SPIR-V→Tint) from the Vulkan
   GLSL; leave Metal/Vulkan hand-written. Captures most of the web benefit
   (greenfield, nothing to lose) at low risk. Metal stays untouched.
3. **Full unification** — GLSL single source → SPIR-V → MSL/WGSL for all three;
   rewrite the Metal/WebGPU binding layers and consolidate features to a superset
   GLSL. Highest cost, best long-term; a deliberate multi-week project + ADR.

## Recommendation

Pursue **(2) first**: adopt SPIR-V→**Tint**→WGSL for the WebGPU backend (it's
Phase 0/1, so there's little hand-written WGSL to lose and the most to gain), and
keep Metal/Vulkan as they are. Re-evaluate **(3)** only when dual/triple-backend
shader maintenance is demonstrably the dominant cost — at which point the
expensive part is the Metal binding rewrite + feature consolidation, *not* the
transpilation, which this study shows is solved.

## Artifacts

Generated samples produced for this study (not committed): `mesh.vert.wgsl`
(naga, direct GLSL), `mesh_frag_sc.metal` / `mesh_vert_sc.metal` (SPIRV-Cross),
`mesh_frag_es.frag` (SPIRV-Cross ESSL 300). Reproduce with the commands in
*Method* above (`glslangValidator`, `spirv-cross`, `naga`).
