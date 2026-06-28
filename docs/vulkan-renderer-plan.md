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
`shaders/vulkan/mesh.{vert,frag}`, `mesh_shadow.vert`). Phase 4a adds the procedural-sky skybox + analytic IBL; Phase 5a adds the
offscreen-HDR + tonemap composite (`composite.{vert,frag}`; the scene renders to
an HDR target, composite tonemaps to a UNORM swapchain). Written against the
Vulkan 1.0 spec; Phases 0–3 are device-verified on Windows (renders
`arena.json`), Phases 4–5 are **unverified on device**. Note: Phase 5a
restructured the frame loop (offscreen target + composite pass + swapchain
format), so the whole render path needs re-verification. ADR-0057 accepted in
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
- **Phase 4a (code landed; verify on device):** procedural day/night sky + FBM
  cloud **skybox** (`sky.vert`/`sky.frag`, fullscreen triangle, drawn first with
  depth off; ray reconstructed via the inverse of the *flipped* clip transform so
  it matches geometry), and **analytic procedural-sky IBL** in `mesh.frag`
  (irradiance ≈ sky in the normal dir; specular ≈ sky in the reflection dir blurred
  by roughness, with roughness-aware Fresnel) replacing the flat-ambient stand-in.
  Sky params ride the globals UBO from `SceneLighting::sky`.
- **Phase 4b — HDR equirect IBL (code landed; verify on device):**
  `uploadTextureHDR` creates an RGBA16F equirectangular map (float→half on CPU);
  `setEnvironmentMap` binds it at set 0 binding 2 (a 1x1 default until set) and
  flips `envMode`. `mesh.frag`/`sky.frag` sample the equirect (lat-long) for the
  skybox + IBL when `envMode==1` (irradiance from the N sample, specular from the
  R sample blended by roughness), else fall back to the procedural sky.
  `environmentMapEnabled` is the live toggle.
- **Phase 4b (still owed):** a real GGX-prefiltered cubemap split-sum (cubemap
  bake + prefilter/irradiance/BRDF-LUT **compute**, replacing the single-sample
  equirect approximation + the missing roughness mip chain) and reflection probes
  (`setReflectionProbes`) with parallax.
- **Verify:** sky orientation matches; ambient/reflections read right; sun disc
  clips to white until the Phase 5 tonemap lands (expected).

### Phase 5 — Post-processing stack
- **Phase 5a (code landed; verify on device):** the offscreen-HDR + composite
  architecture. The scene (sky + geometry) now renders to a linear `RGBA16F` HDR
  target; a composite pass tonemaps it (ACES/AgX) + grade + exposure into the
  swapchain (`composite.{vert,frag}`, ported from `post.metal`). The swapchain
  switched to **UNORM** (the tonemap folds in the sRGB encode). This fixes the
  clipping sun and matches Metal's view transform. Driven by
  `tonemapOperator`/`gradeParams`/`SceneLighting::exposure`.
- **Phase 5b — bloom (code landed; verify on device):** bright-pass (soft-knee)
  + separable Gaussian (H,V) at half-res (`bloom.frag`, two ping-pong RGBA16F
  targets), added back in the composite before tonemap. Driven by `bloomEnabled`
  + `bloomParams` (threshold/knee/intensity). Simpler than Metal's 5-mip pyramid
  — a single blurred level; the pyramid is a later refinement.
- **Phase 5b — G-buffer + SSAO (code landed; verify on device):** the scene pass
  is now MRT — color 0 = HDR, color 1 = a world-normal G-buffer (RGBA8); depth is
  stored + sampleable. `ssao.frag` reconstructs world position from depth via
  `invViewProjection` (same basis as the verified skybox), samples a
  cosine-weighted hemisphere kernel against the depth buffer, and writes a
  half-res AO target the composite multiplies in (clamped to `aoFloor`). Driven
  by `ssaoEnabled` + `ssaoParams`. World-space; an approximation of Metal's GTAO.
- **Phase 5b — SSAO blur (code landed; verify on device):** a 4×4 box blur
  (`ssao_blur.frag`) over the half-res raw AO target denoises the
  hemisphere-kernel sampling before the composite reads it. Runs as a second
  fullscreen pass (raw `aoImage` → `aoBlurImage`) reusing the SSAO render pass;
  the composite now samples the blurred target. No temporal reprojection yet.
- **Phase 5b — SSR (code landed; verify on device):** world-space ray march of
  the reflection ray against the depth buffer (`ssr.frag`), sampling the HDR
  scene on a hit; faded by screen edge + Fresnel + roughness (packed in the
  normal G-buffer's `.a`). Half-res; composite mixes it in by confidence. Driven
  by `ssrEnabled` + `ssrParams` (maxRayDist/thickness/maxRoughness/blendStrength).
  Fixed-step march (no binary refine) — an approximation of Metal's `ssrRayMarch`.
- **Phase 5b — lens effects (code landed; verify on device):** Brown radial
  distortion + lateral chromatic aberration + vignette, **folded into the
  composite** (ported from `post.metal` fragmentLensWarp) so there's no extra
  pass — distortion warps the sample UV, CA splits the HDR channels, vignette
  darkens corners; exact passthrough at neutral params. Driven by the active
  camera's `LensParams` (`lensEffectsEnabled` gates it).
- **Phase 5b (still owed):** DOF (off by default — needs a CoC gather); SSAO
  temporal reprojection (+ SSR binary-search refine); wireframe debug view.
- **Known debt:** the HDR + depth scene targets are single (shared across frames
  in flight), matching the existing single-depth simplification — a cross-frame
  aliasing hazard. Make them per-frame when it bites.
- **Verify:** colors match Metal; `tonemapOperator` (ACES↔AgX) + grade sliders
  drive it; sun no longer clips.

### Phase 6 — Editor viewport (code landed; verify on device)
The Qt editor now renders its viewport through the Vulkan backend on PC/Linux
(it was NullRenderer off-Apple). Design: `HostedWindow` implements the Vulkan
surface seam (`createVulkanSurface` / `requiredVulkanInstanceExtensions`) by
delegating to a host-installed provider (`setVulkanSurfaceProvider`) — the
hosted window stays toolkit-free and headless-testable (new
`test_hosted_window` case). The Qt shell (`editor_main.cpp`) resolves the
platform-native handles (HWND on Windows; X11 `Display*` + window XID via Qt's
`QX11Application` native interface) and hands them to
`src/editor_app/vulkan_viewport.cpp`, which owns the Vulkan + Win32/Xlib surface
headers — deliberately a separate TU because Xlib's macros (`None`, `Status`,
`Bool`) collide with Qt. Win32 + Xlib are implemented; Wayland falls back with a
clear log (run under `QT_QPA_PLATFORM=xcb`, i.e. Xwayland). CMake: a shared
`rt_build_vulkan_shaders()` function builds one `vulkan_shaders` target for both
the viewer and the editor; `editor_app` links the Vulkan backend +
`vulkan_viewport.cpp` on non-Apple when `find_package(Vulkan)` succeeds, else
`null_renderer.cpp`. **Verify on device:** the viewport renders at parity with
the standalone viewer; resize tracks the Qt widget; play/edit input feel matches.

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
`LOG_*` calls vs the stream logger).

**Update 2026-06-28:** Phase 4a (sky + IBL) and Phase 5a (HDR target + tonemap
composite) compile (clang, new `sky.*`/`composite.*` shaders) and run clean on
device — backend reports "Phase 5a", scene loads, **no validation errors** — and
the shadow-attribute warning below was fixed (`98e1b8a`).

**Update 2026-06-28 (Phase 5b):** bloom + SSAO (world-normal G-buffer; new
`bloom.frag`/`ssao.frag`) compile and run — backend reports "Phase 5b", scene
loads — but with **one new validation error** (first item below).

**Update 2026-06-28 (Phase 5b post stack):** SSR + lens effects + debug views
build and run **validation-clean** on device (default config) after two fixes
pushed this session: a build-breaking stray `}` in `vulkan_renderer.cpp` (closed
`namespace engine` early, so `Renderer::create` didn't compile — broke both
configs), and a null `compositeSampler` in the SSR descriptor write
(`updateSsrDescriptors` runs before `createCompositeResources`; the sampler is
now created lazily on first use, guarded in both places). The independentBlend
fix is confirmed clean on device. `-DRT_ENABLE_IMGUI=ON` still fails to compile
(ImGui-version item below). Open items:

- **Phase 5b G-buffer pipeline violates VUID-…-pAttachments-00605.**
  `vkCreateGraphicsPipelines` for the mesh/G-buffer pass:
  `pColorBlendState->pAttachments[1]` differs from `pAttachments[0]` while the
  `independentBlend` device feature is not enabled — the spec requires all blend
  attachments identical unless `independentBlend` is on. Phase 5b added a second
  color attachment (world-normal, `mesh.frag` `outNormal` at location 1) with a
  different blend state than the HDR color attachment. Found on device 2026-06-28;
  the cloud agent has no GPU to catch it. *Fixed:* the sky pipeline is what
  differs (it masks the normal attachment), so `independentBlend` is now enabled
  in `createLogicalDevice` (the mesh pipeline already used identical attachments).
- **ImGui crashes on Vulkan with `-DRT_ENABLE_IMGUI=ON`** (verified 2026-06-28 on
  Phase 5a). Not just "doesn't draw" — it asserts `GImGui != 0 ... No current
  context` at the first `ImGui::NewFrame`, because `ImGui::CreateContext()` is
  never called on the non-Apple path (that wiring lives in the Metal renderer).
  `vulkan_renderer.cpp` references ImGui nowhere, and CMake compiles no
  `imgui_impl_vulkan` (only `imgui_impl_glfw` + Apple's `imgui_impl_metal`).
  Default build (ImGui OFF) is unaffected. *Addressed (code landed; verify on
  device):* `VulkanRenderer::initDebugUi` now creates the context +
  `ImGui_ImplVulkan_Init` (into the composite render pass, with its own
  descriptor pool); `beginFrame` does the backend new-frame + `ImGui::NewFrame`,
  `endFrame` calls `ImGui::Render`, and the composite pass records
  `ImGui_ImplVulkan_RenderDrawData`. CMake compiles `imgui_impl_vulkan.cpp` on
  the non-Apple Vulkan path. The `ImGui_ImplVulkan_Init` signature is guarded by
  `IMGUI_VERSION_NUM` across a three-tier ladder: `<1.90` (RenderPass as the 2nd
  Init arg + command-buffer font upload), `1.90–1.91` (RenderPass/MSAASamples
  flat in `InitInfo` + `CreateFontsTexture()`), and **`≥1.92` (19200)** where
  `MSAASamples`/`RenderPass` moved into `init.PipelineInfoMain.*` and the explicit
  `ImGui_ImplVulkan_CreateFontsTexture()` was removed (fonts built lazily). The
  pinned submodule is ImGui 1.92.8, so the `≥19200` branch is the live path.
  *Addressed (code landed; verify `-DRT_ENABLE_IMGUI=ON` builds + runs on
  device — ImGui 1.92's `ImGui_ImplVulkan_InitInfo` couldn't be compiled in CI).*
  This is Vulkan-only: the Metal backend uses `imgui_impl_metal` whose API is
  unchanged across the bump. Default build (ImGui OFF) is unaffected.
  **Device test 2026-06-28 (`3c69731`):** `-DRT_ENABLE_IMGUI=ON` now *compiles*
  and no longer crashes — the context is created ("[vulkan] ImGui backend
  initialized") — but the overlay is **still non-functional**, two issues left:
  (1) `ImGui::NewFrame` asserts `Invalid DisplaySize value` (imgui.cpp:10943)
  *every frame* — `io.DisplaySize` is never set on this path (the platform
  new-frame that should feed the window size isn't running before
  `ImGui::NewFrame`); (2) a descriptor-pool **WARN** — ImGui 1.92's
  `imgui_impl_vulkan` allocates a `VK_DESCRIPTOR_TYPE_SAMPLER` descriptor, but
  `initDebugUi`'s pool only declares `COMBINED_IMAGE_SAMPLER`; add a `SAMPLER`
  pool size. Default build (ImGui OFF) verified clean (no asserts/validation).
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
  *Addressed (`ac099af`): code landed.* **Untested on this Windows machine — Qt6
  is not installed, so `editor_app` is skipped (`find_package(Qt6)` not found).**
  Needs a Qt6 install (e.g. vcpkg `qtbase`) to build + device-verify the editor's
  Vulkan viewport here; the viewer path is verified.

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
