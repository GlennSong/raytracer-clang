# WebGPU renderer plan (ADR-0058)

The web target's implementation of the `Renderer` seam, compiled to WebAssembly
under Emscripten and driving the browser's `navigator.gpu`. Phased like the
Vulkan backend (`vulkan-renderer-plan.md`): each stage is independently
verifiable, so the work lands in reviewable slices. Parity reference is the Metal
backend (`src/renderer/metal/AGENTS.md`) and `docs/renderer-parity.md`.

**Status: Phases 0+1 landed — builds on emsdk 6.0.1 and runs in a browser.** The
`viewer_web` target builds clean (WebGPU backend + Jolt + engine_core all to
wasm). Verified in headless Chromium (SwiftShader): device/surface/pipeline come
up, the frame loop pumps, `endFrame` records the scene's draws against a
successfully-acquired surface texture, and there are no WebGPU validation errors.
The one unconfirmed thing is visible output — headless SwiftShader won't
composite a WebGPU canvas for screenshot/readback, so eyeballing pixels needs a
real-GPU browser run.

### Known issues / web polish
- **Pointer Lock needs a user gesture.** Play mode disables the cursor on start;
  browsers reject pointer-lock without a click, so it throws a (non-fatal) console
  error. Proper fix is app-level: capture the pointer on canvas click. Orbit
  (drag) camera works regardless.
- **Missing assets 404.** `arena.json` references `models/DamagedHelmet.glb` and an
  HDR env map that aren't in the repo's `assets/`; non-fatal (the level still
  loads). Pre-existing, not web-specific.

## Bundle size (verified, emsdk 6.0.1, with physics)

| Build | wasm | js | data | gzipped total (over the wire) |
| --- | --- | --- | --- | --- |
| Release (`-O2`)     | 4.3 MB / 1.5 MB gz | 169 KB / 43 KB gz | 110 KB / 29 KB gz | **~1.6 MB** |
| MinSizeRel (`-Oz`)  | 3.6 MB / 1.2 MB gz | 169 KB / 43 KB gz | 110 KB / 29 KB gz | **~1.3 MB** |

Jolt physics is a large share of the wasm; a no-physics build (if the engine's
`PhysicsWorld` dependency in `terrain_lod_system` were made optional) would be
materially smaller. The `.data` is the preloaded `assets/` (levels + scripts).

## Build & run

```bash
# Needs an active emsdk on PATH (https://emscripten.org). Not in CI.
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release
cmake --build build-web --target viewer_web
# Serve over HTTP (WebGPU needs a secure context; localhost qualifies) and open:
python3 -m http.server --directory build-web
# → http://localhost:8000/index.html
```

WebGPU comes from the **emdawnwebgpu** port (`--use-port=emdawnwebgpu`, the
default), auto-downloaded by emcc. For an offline build, fetch the
`emdawnwebgpu_pkg-*.zip` from Dawn's GitHub releases, extract it, and pass
`-DRT_EMDAWN_PORT=<path>/emdawnwebgpu.port.py`.

Outputs `viewer_web.{js,wasm,data}` plus a copy of `web/index.html`. The backend
acquires the WebGPU device itself (`RequestAdapter`/`RequestDevice`, awaited via
`-sASYNCIFY`), so the MODULARIZE'd module just needs the canvas — no JS-side
device handoff.

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

## Known risks / unknowns (unverified in-browser)
- **In-browser behaviour unverified.** It compiles + links on emsdk 6.0.1, but no
  GPU in CI means the actual rendering, device handshake, and resize paths need a
  real browser run to confirm.
- **emdawnwebgpu is a moving target.** The Dawn/Emscripten-specific parts of
  `webgpu.h` (and `webgpu_cpp.h`) are explicitly **not** API-stable; a newer
  emsdk/port may rename things again. The core webgpu.h (webgpu-native headers) is
  stable. See `src/renderer/webgpu/AGENTS.md` for the current field gotchas.
- **Swapchain format.** Hardcoded `BGRA8Unorm` (the universal
  `getPreferredCanvasFormat()`); Phase 2+ can query surface capabilities and use
  an `*-srgb` view to drop the manual gamma.
- **Asset FS.** `assets/` is baked into the wasm via `--preload-file`; no
  `settings.json` is preloaded (none exists at repo root), so settings fall back
  to defaults.
