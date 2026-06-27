# `src/renderer/vulkan/` — Agent Guide

The Vulkan backend (Linux + Windows), the second implementation of the
`Renderer` seam (`../AGENTS.md`). **Status: not yet built** — this guide is the
spec to build against. Decision: ADR-0057. Plan + phase breakdown:
`docs/vulkan-renderer-plan.md`. Reference implementation for feature parity:
`../metal/AGENTS.md` (match its pass graph and conventions).

> When code lands here, keep this guide in sync — it exists so future work
> doesn't re-derive the structure. Update it at the end of each phase.

## Target shape

| Concern | Choice (ADR-0057) |
| --- | --- |
| Platforms | Linux + Windows, one backend (both run Vulkan) |
| Selection | CMake non-Apple branch links this in place of `null_renderer.cpp` |
| Surface | `Window::createVulkanSurface(VkInstance)` (pimpl'd in `window.cpp`, no GLFW types leaked) — **not** by reaching through GLFW |
| Shaders | GLSL in `shaders/vulkan/`, compiled to SPIR-V **offline** by `glslc` in CMake; ship `.spv`. No runtime shader compiler (stays dep-free). Trade-off: no hot-reload (Metal has it). |
| GPU structs | reuse `shaders/metal/shader_types.h` for CPU↔GPU layout (std140/std430-aware); keep in lockstep |

Files (planned): `vulkan_renderer.h` (declares the `Renderer` subclass),
`vulkan_renderer.cpp` (implementation, built up over the phases).

## Conventions — same as Metal except where Vulkan forces a difference

Match `../metal/AGENTS.md` for the pass graph, the live-tuning fields, the
material texture-flag bitmask, and resource handles (`SlotMap<…,Tag>` per the
seam). Differences to absorb **inside this backend** so engine math stays shared:

- **Clip space / depth.** Vulkan is Y-down in NDC and depth is [0,1] (vs Metal's
  Y-up, and the Metal backend's reverse-Z [1,0]). Absorb this at projection
  upload + viewport (negative-height viewport for the Y-flip, or flip in the
  projection). Keep **reverse-Z** for parity with Metal's depth precision
  (clear to 0, `VK_COMPARE_OP_GREATER`, `depthClampEnable` as needed). Engine
  `Mat4::perspective`/`lookAt` are unchanged — the fix-up is local.
- **Winding.** Front faces are clockwise engine-wide. Set
  `frontFace = VK_FRONT_FACE_CLOCKWISE`, `cullMode = BACK`. (Watch: the Y-flip
  interacts with winding — verify on a known mesh in Phase 1, the same way the
  Metal probe bake flips to counterclockwise under an X-mirror.)
- **Coordinate/UV origin.** Texture coordinate origin and cubemap face
  conventions differ from Metal; verify the equirect→cubemap bake and IBL faces
  against the Metal output in Phase 4.

## Explicit work Metal hides (the bulk of the new code)

Metal's backend is ~2000 lines because the driver hides a lot. Budget for these
Vulkan-only responsibilities (none change the *render logic*, only the plumbing):

- Instance + validation layers (debug), physical-device select, logical device +
  graphics/present queues.
- Swapchain + image views + recreation on `resize`; per-frame sync (image-
  available / render-finished semaphores, in-flight fences); command pool/buffers.
- **Descriptor sets / layouts** for every uniform + texture binding (Metal just
  `setBytes`/`setTexture`s by index). Plan a small set of descriptor-set layouts
  up front (per-frame, per-material, per-pass).
- Explicit **render passes / framebuffers** (or dynamic rendering) for each
  target: shadow array, the MRT main pass (HDR color + view-normals + depth), and
  each post target.
- **Memory allocation** + barriers/layout transitions. Bring-up can do one
  allocation per resource; fold in pooling only if it bites (don't prematurely
  add VMA — it's a dependency, needs an ADR).
- **Push constants vs UBOs:** prefer push constants for tiny per-draw data
  (model matrix index), UBOs/SSBOs for the big `LightUniforms`/instance arrays
  (mirrors Metal's `lightBuffer` choice).

## Build wiring (CMake, Phase 0)

In the viewer's non-Apple branch (`CMakeLists.txt`, currently the `else()` that
adds `null_renderer.cpp`):

```cmake
find_package(Vulkan REQUIRED)            # headers, loader, and glslc
list(APPEND VIEWER_SOURCES src/renderer/vulkan/vulkan_renderer.cpp)
set(PLATFORM_LIBS Vulkan::Vulkan)
# + a custom command compiling shaders/vulkan/*.{vert,frag,comp} -> *.spv
```

Keep `NullRenderer` as the fallback when `Vulkan` isn't found, so headless/CI
still links. `Renderer::create()` returns the Vulkan backend under the build
guard.

## Phased bring-up (see `docs/vulkan-renderer-plan.md` for detail)

Each phase is independently verifiable on real Linux/Windows hardware with the
validation layers on (no GPU in CI — same constraint Metal has).

0. **Device + swapchain** → clear screen, clean validation log, resize works.
1. **First lit mesh** → vertex/index upload, one UBO + descriptor set, depth,
   `common.metal`→GLSL. Verify Y-orientation + depth vs Metal.
2. **Full forward pass** → all lights, PBR + texture maps, the `applySurface`
   library, instancing, `drawTerrain` CDLOD morph.
3. **Cascaded shadows.**
4. **Environment + IBL** → procedural sky, equirect→cubemap bake, irradiance +
   prefilter + BRDF LUT, reflection probes.
5. **Post stack** → SSAO (temporal) → SSR → bloom → tonemap+grade → lens+DOF,
   one effect at a time, each verified before the next. Wire the `Renderer`
   live-tuning fields.
6. **Parity sweep** → side-by-side levels (forest, city_arena, an HDR scene) vs
   Metal; optional `imgui_impl_vulkan` behind `RT_ENABLE_IMGUI`; gamepad via
   GLFW joystick (the GCController path is macOS-only).

## Parity checklist (what "1:1 with Metal" means here)

Forward PBR (albedo/metallic/roughness/emission + 5 texture maps + per-vertex
tint + analytic surfaces) · directional/point/spot lights (physical units) ·
cascaded shadows + PCF + artistic tint · procedural sky + day/night + FBM clouds
· HDR cubemap + IBL (irradiance/prefilter/BRDF LUT) · reflection probes
(parallax) · SSAO (temporal) · SSR · bloom · ACES/AgX tonemap + grade · lens
(distortion/CA/vignette) + DOF · fog · instancing + wind · CDLOD terrain morph ·
alpha-tested foliage + depth prepass · debug views + wireframe.
