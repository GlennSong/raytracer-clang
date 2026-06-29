# `src/renderer/webgpu/` — Agent Guide

The WebGPU backend (browser / WebAssembly), an implementation of the `Renderer`
seam (`../AGENTS.md`) compiled under Emscripten. **Status: Phases 0–1 landed —
compiles + links against emsdk 6.0.1** (the `viewer_web` target builds clean).
In-browser behaviour is **unverified** (no GPU in CI — needs a real browser run).
Decision: ADR-0058. Plan: `docs/webgpu-renderer-plan.md`. Parity reference:
`../metal/AGENTS.md`.

### What exists after Phase 1
- `webgpu_renderer.cpp` — `WebGpuRenderer : Renderer` (file-local class). Defines
  `Renderer::create()`. instance (`wgpuCreateInstance`) → async adapter+device
  (`wgpuInstanceRequestAdapter` / `wgpuAdapterRequestDevice`, awaited via ASYNCIFY
  — see below) → queue → surface (from the `#canvas` selector) → surface
  configure → depth target → forward pipeline + uniform buffers/bind group.
- **Rendering:** `uploadMesh` packs the (double) engine `Vertex` to a float
  `GpuVertex` and uploads vertex/index buffers (`wgpuQueueWriteBuffer`).
  `setCamera`/`setLights` fill a `GpuGlobals` UBO (view-projection — **no Y-flip**;
  WebGPU NDC is Y-up with [0,1] depth like Metal — camera pos, one directional
  light, flat ambient). `drawMesh` queues a draw; `endFrame` writes the per-draw
  **dynamic** uniform buffer (one 256-byte slot per draw — WebGPU has no push
  constants), acquires the surface texture, and records the render pass.
- **Shaders:** WGSL embedded as a string literal (`kMeshWgsl`), compiled at
  runtime — matches Metal's MSL-string approach (no offline SPIR-V step like
  Vulkan). Real Cook-Torrance (GGX + height-correlated Smith) for the sun + flat
  ambient; scene-linear with a manual `pow(1/2.2)` sRGB encode (the swapchain is
  non-sRGB `BGRA8Unorm`).
- **Stubs until later phases:** `uploadTexture` returns a valid handle, no GPU
  texture yet (Phase 2). No shadows/IBL/sky/post/instancing/terrain. Back-face
  culling **off** until winding is confirmed on device (Phase 2 turns it on).
  Standard [0,1] depth, `LessEqual` (no reverse-Z).
- **Seam plumbing:** no new `Window`/`Renderer` methods — the surface comes from
  the canvas selector, and `nativeWindowHandle()` (null on the web) is ignored.
  CMake's `if(EMSCRIPTEN)` block (root `CMakeLists.txt`) selects this backend and
  builds the `viewer_web` target; `src/web_main.cpp` is the entry point;
  `web/index.html` is the shell.

### WebGPU binding: emdawnwebgpu (NOT the legacy `-sUSE_WEBGPU`)
Emscripten 6.x removed the old `-sUSE_WEBGPU` binding (and
`emscripten_webgpu_get_device`). This backend targets the **emdawnwebgpu** port —
Dawn's implementation of the standardized `<webgpu/webgpu.h>` — pulled in by
`--use-port=emdawnwebgpu` (compile + link). Consequences for the code:
- **All string fields are `WGPUStringView`** (`{data, length}`), not `const char*`
  — use the `sv()` helper. Shader source is `WGPUShaderSourceWGSL`; the canvas
  surface source is `WGPUEmscriptenSurfaceSourceCanvasHTMLSelector`.
- **Device acquisition is async-only.** `RequestAdapter`/`RequestDevice` take
  callbacks and the browser can't block, so `initialize()` awaits them with
  `emscripten_sleep` under **`-sASYNCIFY`** (set in the CMake link options). If you
  drop ASYNCIFY, the device handshake will hang — restructure to a callback-driven
  startup instead.
- Field gotchas vs. older samples: `WGPURenderPassColorAttachment.depthSlice` must
  be `WGPU_DEPTH_SLICE_UNDEFINED`; `WGPUDepthStencilState.depthWriteEnabled` is a
  `WGPUOptionalBool` (not a bool); `WGPUVertexAttribute` leads with `nextInChain`
  (set fields by name, don't aggregate-init positionally).
- The port is a small (~130 KB) header+JS-glue package; the actual Dawn
  implementation runs in the browser's `navigator.gpu`. For offline builds, fetch
  the `emdawnwebgpu_pkg-*.zip` from the Dawn releases and point
  `-DRT_EMDAWN_PORT=<path>/emdawnwebgpu.port.py`.

### Conventions
- Single-threaded: the web build links **without `-pthread`**. Do not introduce
  `std::thread` use on this path — `Application` forces `JobSystem` synchronous
  mode under `__EMSCRIPTEN__`. Adding real threads means opting into pthreads +
  `SharedArrayBuffer` + COOP/COEP headers (ADR-0058 revisit trigger).
- GPU is `f32` only; pack the engine's `double` math to float on upload (as the
  Vulkan backend does). `packMat4` transposes row-major engine `Mat4` to the
  column-major GPU layout.
- Keep WGSL in lockstep with the Metal MSL / Vulkan GLSL when porting later
  phases — the three-tree divergence is the main tech-debt risk (ADR-0058).
