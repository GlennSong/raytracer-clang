# WebGPU renderer plan (ADR-0058)

The web target's implementation of the `Renderer` seam, compiled to WebAssembly
under Emscripten and driving the browser's `navigator.gpu`. Phased like the
Vulkan backend (`vulkan-renderer-plan.md`): each stage is independently
verifiable, so the work lands in reviewable slices. Parity reference is the Metal
backend (`src/renderer/metal/AGENTS.md`) and `docs/renderer-parity.md`.

**Status: Phases 0+1 landed (code-complete, unverified on device).** No emsdk /
GPU in CI — the backend is written against the webgpu.h C API and needs a real
browser run to verify.

## Build & run

```bash
# Needs an active emsdk on PATH (https://emscripten.org). Not installed in CI.
emcmake cmake -S . -B build-web
cmake --build build-web --target viewer_web
# Serve over HTTP (WebGPU needs a secure context; localhost qualifies) and open:
python3 -m http.server --directory build-web
# → http://localhost:8000/index.html
```

Outputs `viewer_web.{js,wasm,data}` plus a copy of `web/index.html`. The shell
creates the WebGPU device (async) before instantiating the MODULARIZE'd module
and hands it over via `Module.preinitializedWebGPUDevice`; the backend reads it
through `emscripten_webgpu_get_device()`, keeping `Renderer::initialize()`
synchronous.

## Architecture (what's reused vs. new)

- **Window:** Emscripten's built-in GLFW3 shim (`-sUSE_GLFW=3`), so
  `src/renderer/window.cpp` is reused unchanged. No web-specific `Window`.
- **Surface:** created from the `#canvas` selector, not the native handle (which
  is null on the web). No new `Window` seam method (contrast Vulkan).
- **Main loop:** `src/web_main.cpp` runs `Application::begin()` once then hands
  `runFrame()` to `emscripten_set_main_loop` (requestAnimationFrame-driven).
- **Threading:** single-threaded — `Application` forces `JobSystem` synchronous
  mode under `__EMSCRIPTEN__`, so no `-pthread` / `SharedArrayBuffer` / COOP-COEP.
- **Shaders:** WGSL embedded as a source string in the backend, compiled at
  runtime (matches Metal's MSL-string approach; no offline step like Vulkan).

## Phases

### Phase 0 — bring-up ✅
Instance → device (preinit) → queue → surface (canvas) → surface configure →
depth target → cleared swapchain. Renders an empty frame in the page's sky color.

### Phase 1 — forward lit mesh ✅
`uploadMesh` packs the double `Vertex` to a float `GpuVertex` and uploads
vertex/index buffers. `setCamera`/`setLights` fill a `GpuGlobals` UBO
(view-projection — no Y-flip; WebGPU NDC is Y-up with [0,1] depth like Metal —
camera pos, one directional light, flat ambient). Per-draw model + material ride
a single **dynamic** uniform buffer (one 256-byte slot per draw; WebGPU has no
push constants). `drawMesh` queues draws; `endFrame` records the whole render
pass. WGSL `mesh` shader is real Cook-Torrance (GGX + height-correlated Smith)
for the sun + a flat ambient stand-in, scene-linear with a manual sRGB encode
(the swapchain is non-sRGB BGRA8). Back-face culling **off** until winding is
confirmed on device (matches Vulkan Phase 1).

### Phase 2 — textures + material maps (todo)
Real `uploadTexture` (GPU textures + sampler bind group), albedo/MR/normal/AO/
emissive maps, alpha-cut foliage (`FLAG_ALPHA_TEST`), the procedural surface
library (port `applySurface` from `mesh.frag`/`common.metal` to WGSL). Turn on
back-face culling.

### Phase 3 — shadows (todo)
Cascaded shadow maps: a depth-only shadow pass into a texture array, the cascade
fit (shared engine-side), PCF sampling. Port `shadows.metal` / `mesh.frag`'s
`computeShadow`.

### Phase 4 — IBL + sky (todo)
Procedural sky pass, reflection-probe / cubemap bake, split-sum IBL + BRDF LUT,
HDR equirect environment. Port `environment.metal`.

### Phase 5 — post stack (todo)
SSAO, SSR, bloom, tone mapping (ACES/AgX) + color grade, lens effects, DoF,
debug views. Port `post.metal`. Needs an offscreen HDR target + composite pass.

### Phase 6 — instancing + terrain (todo)
`drawMeshInstanced` (instance buffer), CDLOD `drawTerrain` morph, wind sway
(`FLAG_WIND`).

## Known risks / unknowns (unverified on device)
- **webgpu.h API churn.** The swapchain → surface transition and the
  `WGPUShaderModuleWGSLDescriptor` → `WGPUShaderSourceWGSL` / `WGPUStringView`
  renames mean the backend may need small edits to match the installed emsdk. See
  `src/renderer/webgpu/AGENTS.md`.
- **Swapchain format.** Hardcoded `BGRA8Unorm` (the universal
  `getPreferredCanvasFormat()`); Phase 2+ can query surface capabilities and use
  an `*-srgb` view to drop the manual gamma.
- **Asset FS.** `assets/` is baked into the wasm via `--preload-file`; no
  `settings.json` is preloaded (none exists at repo root), so settings fall back
  to defaults.
