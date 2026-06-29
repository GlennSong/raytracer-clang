# `src/renderer/webgpu/` — Agent Guide

The WebGPU backend (browser / WebAssembly), an implementation of the `Renderer`
seam (`../AGENTS.md`) compiled under Emscripten. **Status: Phases 0–1 landed** —
device/surface bring-up, a cleared swapchain with depth, and forward lit
single-directional-light Cook-Torrance draws (`webgpu_renderer.cpp`, WGSL
embedded in that file). **Unverified on device** (no emsdk/GPU in CI — needs a
real browser run). Decision: ADR-0058. Plan: `docs/webgpu-renderer-plan.md`.
Parity reference: `../metal/AGENTS.md`.

### What exists after Phase 1
- `webgpu_renderer.cpp` — `WebGpuRenderer : Renderer` (file-local class). Defines
  `Renderer::create()`. instance (`wgpuCreateInstance`) → device
  (`emscripten_webgpu_get_device`, preinitialized in JS) → queue → surface (from
  the `#canvas` selector) → surface configure → depth target → forward pipeline +
  uniform buffers/bind group.
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

### webgpu.h API-churn notes (read this first if it won't compile)
Written against the webgpu.h C API as Emscripten ships it around **emsdk 3.1.x**.
Newer emsdk/Dawn revisions renamed things; adjust if the build breaks:
- Shader source: `WGPUShaderModuleWGSLDescriptor` + `.code` (`const char*`) →
  newer `WGPUShaderSourceWGSL` and string fields as `WGPUStringView`
  (`{ .data, .length }`) instead of NUL-terminated `const char*`.
- Swapchain: this backend uses the **surface-based** API
  (`wgpuSurfaceConfigure` / `wgpuSurfaceGetCurrentTexture`). Much older emsdk used
  `WGPUSwapChain` (`wgpuDeviceCreateSwapChain` / `wgpuSwapChainGetCurrentTextureView`).
- `WGPURenderPassColorAttachment::depthSlice` exists on newer headers
  (zero-init = slice 0); harmless on older ones.
- `entryPoint` / `label` are `const char*` here; newer headers use `WGPUStringView`.

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
