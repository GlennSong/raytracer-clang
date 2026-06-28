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

**Status:** Phases 0–2 implemented (2 partial) — device/swapchain bring-up, the
offline SPIR-V toolchain, forward lit draws, and the full multi-light
Cook-Torrance model + the procedural-surface library
(`src/renderer/vulkan/vulkan_renderer.{h,cpp}`, `shaders/vulkan/mesh.{vert,frag}`).
Written against the Vulkan 1.0 spec but **unverified on device** (no GPU/SDK in
CI; needs a Linux/Windows run with the validation layers). ADR-0057 accepted in
principle (Pending).

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

### Phase 3 — Shadows
- CSM: shadow-map array, the shadow render pass (`shadows.metal`), cascade fit
  (already engine-side), PCF, bias/normal-bias, artistic tint/strength.
- **Verify:** cascades line up with Metal; no acne/peter-panning beyond Metal's.

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
