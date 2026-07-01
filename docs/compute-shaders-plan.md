# Compute Shaders — Opportunity Analysis

Status: **analysis / not a committed decision.** Captures where GPU compute would
and wouldn't pay off for this engine, and what porting each API costs. Promote
the parts we act on to ADRs in `docs/decisions.md`.

## What a compute shader is

A GPU program that runs *outside* the graphics pipeline — no vertices, no
rasterizer, no fragments. You dispatch a 3D grid of **workgroups**
(Vulkan/WebGPU) / **threadgroups** (Metal); each holds a block of
threads/invocations (declared in the shader: `local_size_*` in GLSL,
`[[threads_per_threadgroup]]` in MSL, `@workgroup_size` in WGSL). Threads in a
workgroup cooperate via fast **shared/threadgroup memory** + barriers; across
workgroups only via global buffers + atomics. The execution model is identical
across all three APIs — only the shading language and host plumbing differ.

## Cross-API cost (we ship Vulkan GLSL + Metal MSL today; no WebGPU)

| | Language → compiled | Entry point | Sync between passes |
|---|---|---|---|
| **Vulkan** | GLSL/HLSL → SPIR-V (offline `glslc`) | `layout(local_size_x=…) in;` | **Manual** pipeline/memory barriers — #1 bug source |
| **Metal** | MSL → `.metallib` (we compile at runtime, ADR path) | `kernel void … [[thread_position_in_grid]]` | Mostly **automatic** |
| **WebGPU** | WGSL (new language, we have none) | `@compute @workgroup_size(…) fn` | **Automatic** |

Per-backend gotchas when porting one to another:
- **Dispatch granularity:** Vulkan/WebGPU dispatch counts *workgroups* (you
  compute `ceil(N/local_size)` and guard `if (id >= N) return;`); Metal's
  `dispatchThreads` counts *threads* and divides for you.
- **Barriers:** manual on Vulkan, automatic on Metal/WebGPU. Code written
  against Metal/WebGPU is silently wrong on Vulkan if a barrier is missing.
- **Binding models diverge:** descriptor sets vs. argument buffers vs. bind
  groups — the same abstraction problem we already solve for vert/frag shaders.
- **Capability limits vary:** workgroup size, shared-mem size, atomics (WebGPU
  most restrictive). Query, don't hardcode.
- **Readback is explicit everywhere** and async on WebGPU.

There is **no** compute source shareable verbatim across all three; the algorithm
ports mechanically but is re-authored per dialect. WebGPU is a *whole new backend*
(renderer impl + WGSL translations of every shader), not just a compute add-on.

## Where it pays off (ranked by ROI)

| Workload | Today's baseline | Realistic speedup | Verdict |
|---|---|---|---|
| **Path tracing** | CPU, multithreaded (`path_tracer.cpp`, per-scanline `JobSystem`) | **10–50× (100×+ with HW ray tracing)** | **The prize.** But a rewrite, not a port (see below). |
| **Sim — particle/grid** (thermal erosion; future fluids/cloth/boids) | CPU, serial-ish | **10–50×** | Yes *if* the algorithm is grid/particle-shaped. |
| **Sim — droplet erosion** (`erosion.cpp`) | CPU, sequential | ~10–30× w/ atomics; big if re-formulated | Maybe — offline bake; full win needs a pipe-model reformulation. |
| **Sim — rigid-body physics** (Jolt) | CPU, cache-optimized | often ≤1× (a loss) | **No.** Constraint solvers resist the GPU; keep Jolt. |
| **SSAO** | **Already a GPU fragment shader** (`ssao.frag`, 16 spp, half-res) | **1.2–2×** | No — only when building GTAO/temporal anyway. |

### Path tracing — the caveats that erode the headline number
- **Rewrite, not port.** `Scene::tracePath` is recursive C++ with `std::` types
  and CPU RNG. GPU wants: an **iterative** loop (no recursion), a per-pixel
  counter-based RNG (PCG — can't call `randomDouble()`), and a **flattened BVH**
  in a buffer.
- **`double` kills throughput.** The tracer uses `double`/`Real` throughout; GPUs
  run `double` at 1/32–1/64 rate. Must go single-precision.
- **Divergence tax.** The chromatic-aberration branch (3 rays/channel) and
  per-material branching cause warp divergence (naive megakernel loses 2–5×;
  wavefront path tracing recovers it at high complexity cost).
- We already have the algorithm *and* a Metal backend, so this is the most
  tractable big win in the repo — likely **20–50×** on a decent GPU, moving the
  offline render toward interactive.

### SSAO — why it's near the bottom
Already embarrassingly parallel on the fragment stage. Compute only adds
threadgroup-memory depth-tile caching (10–40% on the sampling loop) and fusing
the `ssao_blur.frag` round-trip. Net frame impact is a rounding error.

### The two traps
The workloads people *assume* are easy wins — SSAO and physics — are the worst
ROI here (one's already fast, one resists the GPU). And every estimate above is
contingent on the GPU-shaped rework (no recursion, `float`, GPU RNG, coherent
memory). The naive port gets a fraction of the headline number.

## If we act: renderer-seam shape

Fits the existing `Renderer` RHI seam (ADR-0001):
- Add a compute concept: `ComputePipelineHandle` + `dispatch(handle, gx, gy, gz)`
  + a resource-barrier primitive.
- Implement per backend: `vulkan_renderer.cpp` (explicit barriers) and
  `metal_renderer.mm` (`MTLComputePipelineState` / compute encoder — the Metal
  post stack already runs a compute encoder for SSAO/SSR/bloom/DOF), no-op in
  `null_renderer.cpp`.
- Add `shaders/vulkan/*.comp` (GLSL→SPIR-V) + `shaders/metal/*.metal` `kernel`
  per pass — matches the existing dual-authoring pattern.

**Dependency note (ADR-worthy):** CLAUDE.md mandates standard-library-only. A
cross-API "write once, translate" story needs a shader cross-compiler
(glslang/SPIRV-Cross, or Tint for WGSL) — a new build-time dep to weigh before
committing.
