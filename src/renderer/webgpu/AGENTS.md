# `src/renderer/webgpu/` — Agent Guide

The WebGPU backend (browser / WebAssembly), an implementation of the `Renderer`
seam (`../AGENTS.md`) compiled under Emscripten. **Status: feature parity with the
Vulkan backend** across the whole render graph. Decision: ADR-0058. Plan:
`docs/webgpu-renderer-plan.md`. Parity reference: `../metal/AGENTS.md`. Build
guide: `docs/web-build.md`. Concepts overview: `docs/rendering.md`.

**Verification is structural**, not visual: headless SwiftShader doesn't composite
the WebGPU canvas (screenshots come back blank even when rendering succeeds), so
correctness is checked via no WGSL/device validation errors (an uncaptured-error
callback logs them), frames pumping, and `rt_web_*` readbacks — plus the WGSL is a
faithful port of the proven Metal/Vulkan trees. Visual confirmation is on-device.

### What the backend does (all landed)
- `webgpu_renderer.cpp` — `WebGpuRenderer : Renderer` (file-local class). Defines
  `Renderer::create()`. instance → async adapter+device (awaited via ASYNCIFY,
  below) → queue → surface (from the `#canvas` selector) → configured swapchain →
  all pipelines + bind-group layouts.
- **Geometry:** `uploadMesh` packs the (double) engine `Vertex` to a float
  `GpuVertex`; `drawMesh` / `drawMeshInstanced` (real hardware instancing) /
  `drawTerrain` (CDLOD morph). Per-draw model+material rides a **dynamic** uniform
  buffer (256-byte slots — WebGPU has no push constants).
- **Materials:** `uploadTexture` (RGBA8 + blit-chain **mipmaps**),
  `uploadTextureHDR` (RGBA16Float equirect), a per-material group-2 bind group
  (albedo/normal/MR/emissive/AO + sampler), TBN normal mapping, alpha-test.
- **Lighting:** multi-light Cook-Torrance (GGX + height-correlated Smith),
  **cascaded shadow maps** with hardware PCF, a procedural **sky** or a bound HDR
  **equirect environment** (`setEnvironmentMap`), IBL with a **baked split-sum
  BRDF LUT**.
- **Render graph** (`endFrame`): shadow passes → main pass (MRT: HDR color +
  G-buffer normal/roughness) → SSAO → SSR → bloom → composite (tone-map + grade +
  AO/SSR/bloom) → swapchain. The scene renders into an offscreen `RGBA16Float`
  HDR target; only the composite writes the swapchain. SSAO/SSR render at
  `postEffectScale` (half-res default) and are linear-upscaled in the composite.
- **Shaders:** WGSL embedded as string literals (`kMeshWgsl`, `kCompositeWgsl`,
  `kBloomWgsl`, `kSsaoWgsl`, `kSsrWgsl`, `kBlitWgsl`, `kBrdfWgsl`), compiled at
  runtime — matches Metal's MSL-string approach (no offline SPIR-V step).
- **Seam plumbing:** no new `Window`/`Renderer` methods — the surface comes from
  the canvas selector, and `nativeWindowHandle()` (null on the web) is ignored.
  CMake's `if(EMSCRIPTEN)` block selects this backend and builds `viewer_web`;
  `src/web_main.cpp` is the entry point; `web/viewer.html` is the shell,
  `web/index.html` the scene gallery.

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
  `emscripten_sleep` under **`-sASYNCIFY`** (set in the CMake link options). Drop
  ASYNCIFY and the device handshake hangs — restructure to callback-driven startup.
- **Newer struct/field names.** Texture copies use `WGPUTexelCopyTextureInfo` /
  `WGPUTexelCopyBufferLayout`. `WGPURenderPassColorAttachment.depthSlice` must be
  `WGPU_DEPTH_SLICE_UNDEFINED`; `WGPUDepthStencilState.depthWriteEnabled` is a
  `WGPUOptionalBool`; `WGPUVertexAttribute` leads with `nextInChain` (set fields
  by name, don't aggregate-init positionally). Verify against the port's
  `webgpu.h` — the Dawn/Emscripten-specific parts are explicitly not API-stable.
- For offline builds, fetch the `emdawnwebgpu_pkg-*.zip` from Dawn's releases and
  point `-DRT_EMDAWN_PORT=<path>/emdawnwebgpu.port.py`.

### Conventions / gotchas
- **Single-threaded:** the web build links **without `-pthread`**. Don't introduce
  `std::thread` — `Application` forces `JobSystem` synchronous mode under
  `__EMSCRIPTEN__`. Real threads mean opting into pthreads + `SharedArrayBuffer` +
  COOP/COEP headers (ADR-0058 revisit trigger).
- **Coordinate system:** WebGPU NDC is Y-up, depth `[0,1]` — like Metal, unlike
  Vulkan. **No Y-flip**; the cascade fit uses standard-Z.
- **GPU is `f32` only:** pack the engine's `double` math to float on upload;
  `packMat4` transposes row-major engine `Mat4` to the column-major GPU layout.
  `size_t` is 32-bit on wasm32 — cast 64-bit hashes explicitly.
- **WGSL has no forward declarations** and **no push constants**; `textureSample`
  needs uniform control flow (use `textureSampleLevel`/`textureLoad` in branches).
- **Keep WGSL in lockstep with the Metal MSL / Vulkan GLSL** when editing shaders —
  the three-tree divergence is the main tech-debt risk (ADR-0058). See
  `docs/rendering.md` and `docs/shader-transpile-study.md`.
- **Live tuning hooks:** the debug panel drives exported `rt_web_*` functions
  (`src/web_main.cpp`) that flip the same `Renderer`/`Settings` fields the desktop
  ImGui overlay uses. Add a hook there, export it in `CMakeLists.txt`.
