# WebGPU renderer plan (ADR-0058)

The web target's implementation of the `Renderer` seam, compiled to WebAssembly
under Emscripten and driving the browser's `navigator.gpu`. Phased like the
Vulkan backend (`vulkan-renderer-plan.md`): each stage is independently
verifiable, so the work lands in reviewable slices. Parity reference is the Metal
backend (`src/renderer/metal/AGENTS.md`) and `docs/renderer-parity.md`.

**Status: Phases 0–6 landed + gameplay — parity with the Vulkan backend.** The
`viewer_web` target builds clean (WebGPU backend + Jolt + engine_core to wasm).
The backend now has: the analytic surface library + **texture/material maps**
(albedo/normal/MR/emissive/AO, TBN normal mapping, alpha-test, **mipmaps**),
**cascaded** shadow maps with PCF, a procedural **sky** (gradient + sun disc)
**and a bound HDR equirect environment**, an offscreen **HDR pipeline** with
ACES/AgX **tone-map + grade**, **bloom**, a material **G-buffer**, **SSAO** +
**SSR**, **IBL** (procedural-sky or HDR equirect irradiance/prefilter + a **baked
split-sum BRDF LUT**), real GPU **instancing** (verified 8× draw-call reduction),
**CDLOD terrain morph**, and **wind** sway. Gameplay runs too — play mode (Jolt
physics + the Lua gun), day/night controls, and desktop+touch input — all wired
through exported `rt_web_*` hooks and a `web/viewer.html` debug panel (with
quality presets — Mobile/Balanced/High); `web/index.html` is a scene-gallery
landing page that deep-links each level into the viewer. The IBL
matches Vulkan's approach (per-fragment equirect sampling + baked BRDF LUT); the
only thing beyond it is Metal's offline **cubemap prefilter / irradiance
convolution / reflection probes**, which the Vulkan backend also omits.

**Verification caveat:** headless SwiftShader does **not** composite the WebGPU
canvas (both `page.screenshot` and `getImageData` return the blank canvas), so
pixels can't be eyeballed in CI. Everything here is verified *structurally* — no
WGSL/device validation errors, the frame pumps, draws/instances/uniforms plumb,
readback values (camera, sun direction, render stats) — plus the shaders are
byte-for-byte ports of the proven Metal/Vulkan trees. Visual confirmation is
on-device (a real GPU browser).

### Still missing vs. Metal (todo)
- **Metal-only IBL luxuries: cubemap prefilter, irradiance convolution, and
  parallax-corrected reflection probes.** The Vulkan backend does *not* have
  these either — it (like the web backend now) samples the equirect per-fragment
  and weights specular by a baked BRDF LUT. Adding them would be a strict Metal
  match, but there is no HDR content in the shipping scenes to benefit, so this
  stays the lowest-priority item.
- **Lens distortion / DoF** and **per-material two-sided/back-face culling** —
  minor polish.

### Known issues / web polish
- ~~Pointer Lock needs a user gesture.~~ Fixed: Play (a click) captures the
  pointer for FPS look; a canvas click re-captures after Esc; the editor keeps
  drag-to-look. The viewer defaults to AgX tone mapping at 130% exposure, and
  the simulation pauses while the debug panel is open or no play session is
  active (rendering + free-look keep running).
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

Outputs `viewer_web.{js,wasm,data}` plus `web/{index,viewer}.html`, `scenes.json`,
and the baked `thumbs/`. `index.html` is the gallery; `viewer.html` loads the
module (`?level=<id>` selects the scene). Thumbnails are path-traced offline by
`./raytracer --level <f> --out thumbs/<id>.png`. The backend
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

### Phase 2 — textures + material maps ✅
✅ The procedural surface library (`applySurface`), real `uploadTexture` (RGBA8
GPU textures) with **mipmaps** (manual blit-chain generation), a per-material
group-2 bind group (albedo/normal/MR/emissive/AO + sampler, cached by map set,
1×1 defaults for missing maps), tangent-space normal mapping (TBN), and alpha-cut
foliage (`FLAG_ALPHA_TEST`). ❌ Back-face culling stays off — the alpha-test
foliage cards are single-sided, so global culling would drop leaves; it needs a
per-material two-sided flag (matches Vulkan, which also ships cull-none).

### Phase 3 — shadows ✅
Cascaded shadow maps: a depth-only pass per cascade into a depth-2d array, the
Vulkan cascade fit (frustum-slice split + texel-snapped ortho box, adjusted for
WebGPU standard-Z), hardware-PCF sampling with view-depth cascade selection. The
per-cascade index rides a dynamic-uniform (WebGPU has no push constants).

### Phase 4 — IBL + sky ✅
✅ Procedural sky background pass (gradient + sun disc, world-ray from
`invViewProjection`), a **bound HDR equirect environment** (`uploadTextureHDR` →
RGBA16Float, `setEnvironmentMap` → envMode on `skySunColor.w`; the sky + IBL
sample the equirect when set, else the procedural sky), and a **baked split-sum
BRDF LUT** (RG16Float, GGX importance sampling, a port of
`shaders/vulkan/brdf_lut.frag`). This matches the Vulkan backend. ❌ Metal's
offline cubemap prefilter / irradiance convolution / reflection probes — todo
(Vulkan omits them too; no HDR content in-scene to benefit).

### Phase 5 — post stack ✅
✅ Offscreen HDR target + composite, ACES/AgX tone map + grade + exposure, bloom,
SSAO (hemisphere kernel on **G-buffer normals**), a material G-buffer (world
normal + roughness via MRT), and **SSR** (screen-space raymarch off the
G-buffer, fresnel/roughness/edge-weighted). Debug views incl. AO-only + cascades.
❌ Lens distortion + DoF — todo (minor).

### Phase 6 — instancing + terrain ✅
✅ `drawMeshInstanced` — real hardware instancing (per-instance model in a second
vertex buffer; main + shadow passes), verified ~8× fewer draw calls in the city.
✅ Wind sway (`FLAG_WIND`) — height-weighted vertex displacement on both draw
paths (wall-clock driven, frame-rate independent). ✅ CDLOD `drawTerrain` morph —
`vs_terrain`/`vs_shadow_terrain` blend toward the baked coarser target over the
morphStart..morphEnd camera-distance band (verified with the `cdlod` level).

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
