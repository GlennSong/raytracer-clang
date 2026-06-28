# Vulkan Renderer — Plan

A second GPU backend so the realtime viewer runs on PC (Windows) and Linux at
graphical parity with the macOS/Metal backend. Vulkan plugs into the existing
`Renderer` RHI seam (ADR-0001) as a third implementation, selected by CMake on
non-Apple platforms in place of `NullRenderer`. **One Vulkan backend covers both
targets** (Linux and Windows both run Vulkan).

This is a planning document; each phase lands with its own work and updates
`docs/decisions.md` where it makes a real architectural choice. The umbrella
decision is **ADR-0057**. Roadmap cross-reference: Tier 5 "Second rendering
backend (Vulkan)".

**Status:** Phases 0–3 implemented — device/swapchain bring-up, the offline
SPIR-V toolchain, forward lit draws, the full multi-light Cook-Torrance model +
procedural-surface library + texture maps, and **cascaded shadow maps**
(`src/renderer/vulkan/vulkan_renderer.{h,cpp}`,
`shaders/vulkan/mesh.{vert,frag}`, `mesh_shadow.vert`). Written against the
Vulkan 1.0 spec but **unverified on device** (no GPU/SDK in CI; needs a
Linux/Windows run with the validation layers). ADR-0057 accepted in principle
(Pending).

Phase 2 textures landed: `uploadTexture` creates real RGBA8 `VkImage`s (staging
upload + layout transitions); a per-frame transient descriptor pool binds a
material set (set 1: albedo/MR/AO/emissive sampled, gated by textureFlags; a 1x1
white default stands in for absent maps). **Normal mapping is deferred** (needs a
TBN basis), as are **mipmaps**. Instanced and terrain geometry already render via
the base `Renderer` defaults (which call `drawMesh`); the dedicated
`drawMeshInstanced` batching and `drawTerrain` CDLOD **morph** are owed
refinements. Then Phases 3+ (shadows, IBL/environment, post stack).

---

## Scope & parity target

Match the *viewer's* feature set, not the offline path tracer (the two are
separate paths and stay separate). The viewer pipeline is ordinary forward
shading + screen-space post; nothing exotic:

- Forward lit pass: PBR materials (albedo/metallic/roughness/emission + the
  texture maps), directional/point/spot lights (ADR-0017 units), the analytic
  procedural-surface library (brick/concrete/asphalt/…), per-vertex tint.
- Cascaded shadow maps (sun) with PCF, artistic tint/strength.
- Environment: procedural analytic sky + day/night, FBM clouds, **or** a baked
  cubemap from an equirectangular HDR; IBL (irradiance + prefiltered specular +
  BRDF LUT); reflection probes with parallax correction.
- Screen-space post: SSAO (temporal), SSR, bloom, tonemap (ACES/AgX) + grade,
  lens effects (distortion/CA/vignette) + DOF.
- Instancing (vegetation/scatter, wind sway), CDLOD terrain morph, foliage
  alpha-test with depth prepass, wireframe + debug views.

Explicitly **out of scope:** unifying viewer ↔ offline tracer; compute shaders;
hardware ray tracing. The descriptor model and SPIR-V toolchain built here are
the foundation if those are wanted later (ADR-0057 revisit trigger).

---

## Decisions locked in ADR-0057

1. **Targets:** Linux + Windows from one backend.
2. **Surface creation behind the seam.** `Window` gains a pimpl'd
   `createVulkanSurface(VkInstance) -> VkSurfaceKHR` (forward-declared Vulkan
   handles, no GLFW types leaked), so the backend never reaches through GLFW
   (ADR-0001). `GLFW_NO_API` is already hinted at `window.cpp:264`.
3. **Shaders → SPIR-V offline.** Port the six MSL files to GLSL, compile with
   `glslc`/`glslangValidator` at build time via a CMake custom command, ship
   `.spv`. Keeps the runtime dependency-free (no `libshaderc`); cost is no
   hot-reload on this backend.
4. **Vulkan conventions absorbed in the backend.** Y-flipped clip space and
   [0,1] depth handled at projection-upload / viewport setup so engine math and
   the shared `shaders/metal/shader_types.h` GPU structs are unchanged.

---

## Phased bring-up

Each phase is independently verifiable and lands in a reviewable slice. "Verify"
means on real Linux/Windows hardware with the Vulkan validation layers enabled
(no GPU in CI — same constraint Metal already has).

### Phase 0 — Build wiring & device bring-up
- CMake: `find_package(Vulkan REQUIRED)` on the non-Apple branch, add
  `src/renderer/vulkan/vulkan_renderer.cpp`, link `Vulkan::Vulkan` (replaces the
  `NullRenderer` line). Keep `NullRenderer` as the fallback when Vulkan is absent
  so headless CI still links.
- Instance (+ validation layers in debug), physical-device selection, logical
  device + queues (graphics/present), the `Window::createVulkanSurface` seam,
  swapchain + image views, command pool/buffers, per-frame sync
  (image-available / render-finished semaphores, in-flight fences), and `resize`
  (swapchain recreate).
- **Verify:** `./build/viewer` opens a window and clears to a color; clean
  validation-layer log; resize works.

### Phase 1 — First lit mesh
- Vertex buffer / index buffer upload (`uploadMesh`), the `Vertex` layout, a UBO
  for camera/transform, a descriptor set, one graphics pipeline, depth buffer.
- Port `common.metal` (vertex stage, BRDF helpers) → GLSL; wire the offline
  SPIR-V build.
- Implement `setCamera`, a single-light `setLights`, `drawMesh`, `beginFrame`/
  `endFrame`.
- **Verify:** a lit mesh matches Metal for the same scene/camera; correct depth;
  Y-orientation correct (clip-space flip absorbed).

### Phase 2 — Full forward pass
- Port `lighting.metal`: all light types, PBR + texture maps (`uploadTexture`,
  samplers, descriptor arrays/bindless-lite), the procedural-surface library,
  per-vertex tint, fog.
- `drawMeshInstanced` (instance buffer) and `drawTerrain` (morph band in the
  vertex shader, ADR-0036).
- **Verify:** a full level renders at parity; instancing + terrain morph correct;
  `RenderStats` populated.

### Phase 3 — Shadows  *(code landed; verify on device)*
- CSM: depth array (a layer per cascade), depth-only shadow pipeline
  (`mesh_shadow.vert`), the cascade fit ported from Metal's `setLights`
  (forward-Z corner reconstruction; the shadow VP carries no clip Y-flip so the
  depth write and the PCF read use the same NDC→uv mapping), PCF via
  `sampler2DArrayShadow`, normal-bias + dynamic depth-bias. Shadow strength is
  wired; artistic **tint** and `ambientStrength` are deferred.
- **Verify:** cascades line up; no acne/peter-panning; flip depth-bias sign or
  tune constants if needed.

### Phase 4 — Environment & IBL
- Port `environment.metal`: skybox (procedural sky + clouds), equirect→cubemap
  bake (`uploadTextureHDR`, `setEnvironmentMap`), irradiance + prefiltered spec
  + BRDF LUT precompute, reflection probes (`setReflectionProbes`) with parallax.
- **Verify:** HDR and procedural-sky modes both match; probe reflections correct.

### Phase 5 — Post-processing stack
- Offscreen HDR render targets + the post chain from `post.metal`, one effect at
  a time, each verified before the next: SSAO (+ temporal history) → SSR → bloom
  → tonemap + grade → lens effects + DOF. Plus the debug views and wireframe.
- **Verify:** each effect matches Metal; the `Renderer`'s live toggles/params
  (`ssaoEnabled`, `ssrParams`, `bloomParams`, `tonemapOperator`, …) all drive it.

### Phase 6 — Parity sweep & polish
- Side-by-side a set of representative levels (forest, city_arena, an HDR-lit
  scene) Metal vs Vulkan; chase remaining differences.
- Optional: Dear ImGui Vulkan backend (`imgui_impl_vulkan`) behind
  `RT_ENABLE_IMGUI` to match the Metal overlay (ADR-0011), wired the same way the
  Metal backend hooks `initDebugUi`/`shutdownDebugUi`.
- Gamepad on Linux/Windows uses GLFW's joystick path (the GCController `.mm` is
  macOS-only); confirm `gamecontrollerdb.txt` loads.

---

## Risks & watch-items

- **Boilerplate volume.** The backend will be materially larger than
  `metal_renderer.mm` (~2000 lines). Mitigated by phasing and by keeping memory
  allocation simple first (one allocation per resource is fine for bring-up;
  fold in a sub-allocator / VMA-style pooling only if it bites).
- **Two shader trees.** `shaders/vulkan/*.glsl` parallels `shaders/metal/*.metal`
  until/unless unified. Reuse the shared `shader_types.h` GPU-struct header
  across both to keep CPU↔GPU layouts in lockstep; watch for drift on every
  shader change.
- **Convention bugs.** Y-flip, [0,1] depth, descriptor binding model, and
  push-constants-vs-UBO are the usual sources of "looks subtly wrong"; nail them
  in Phases 1–2 with the debug views.
- **No CI GPU.** Validation layers + on-device checks are the safety net; keep
  the engine/unit-test layer CPU-only and untouched.

---

## Device-verified findings (Windows + NVIDIA RTX 3060, 2026-06-27)

First time the backend was compiled and run on a real GPU (Phases 0–3 had only
ever been written, never built — no SDK in CI). It now builds with clang and
renders `assets/levels/arena.json` (1280×720, forward + multi-light + CSM). The
first compile surfaced bugs fixed in `f7a0908` (shader `patch` keyword),
`204e4c5` (shader `g.lightCount` → `g.counts.x`), `8e25546` (43 printf-style
`LOG_*` calls vs the stream logger). Open items found on device:

- **ImGui overlay does not render on Vulkan.** With `-DRT_ENABLE_IMGUI=ON` the
  tilde toggle and GLFW input work, but nothing draws: `vulkan_renderer.cpp`
  references ImGui nowhere and CMake compiles no `imgui_impl_vulkan` (only
  `imgui_impl_glfw` + Apple's `imgui_impl_metal`). Metal renders the overlay via
  `metal_renderer`'s draw-data submission. **To do:** add `imgui_impl_vulkan`
  (init with instance/device/queue/render pass + a descriptor pool) and submit
  `ImGui::GetDrawData()` in the frame, mirroring the Metal path.
- **Camera left/right is inverted vs Metal** (yaw feels backwards; pitch is fine).
  Traced and *not* reproduced statically: mouse input is platform-identical
  (`window.cpp:164`, plain GLFW delta), camera yaw is shared engine code
  (`camera_system.cpp:108`, `-mouseDeltaX`), and the Vulkan matrices only apply a
  *vertical* Y-flip (`packMat4`, transpose verified — no X mirror). No code cause
  for a Vulkan-only horizontal inversion was found; needs an on-device A/B vs a
  Metal screenshot. **The fix is NOT the shared `-mouseDeltaX` sign** — Metal uses
  the same line happily, so flipping it would just break Metal.
- **Validation warnings (benign):** `vkCreateGraphicsPipelines` reports vertex
  attributes 1–4 "not consumed by vertex shader" — the shadow pipeline's vertex
  input declares the full layout but its vertex shader uses only position.
  *Fixed:* the shadow pipeline now declares a position-only vertex attribute
  (binding stride unchanged; same interleaved buffer, offset 0).
- **Qt editor viewport is unrendered on non-Apple.** `editor_app` builds wherever
  Qt6 is installed and its Qt shell/UX runs on Windows/Linux, but the embedded 3D
  viewport links `null_renderer.cpp` there (`CMakeLists.txt` editor block: Metal on
  APPLE, else NullRenderer) — the standalone-viewer Vulkan path does **not** apply.
  The editor embeds the renderer through `HostedWindow`/`EngineViewport`, which
  binds to a native view (NSView on macOS, `editor_main.cpp:109`). **To do:** drive
  a `VkSurfaceKHR` from the Qt widget's `HWND`/`xcb` window through `HostedWindow`
  so the editor viewport renders on Vulkan, mirroring the Metal `CAMetalLayer`
  embedding. Distinct from the viewer's surface path (viewer owns a GLFW window).

## File map (new)

```
src/renderer/vulkan/
  vulkan_renderer.h          # Renderer subclass declaration
  vulkan_renderer.cpp        # backend implementation (phased)
shaders/vulkan/
  common.glsl / *.vert/.frag # ports of shaders/metal/*.metal
  (shared) shaders/metal/shader_types.h reused for GPU structs
```

Touched: `CMakeLists.txt` (Vulkan branch + SPIR-V build step),
`src/renderer/window.{h,cpp}` (`createVulkanSurface` seam),
`docs/decisions.md` (ADR-0057), `docs/ROADMAP.md` (Tier 5 status).
