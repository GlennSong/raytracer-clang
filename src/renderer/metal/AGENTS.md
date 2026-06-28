# `src/renderer/metal/` — Agent Guide

The Metal backend (macOS), the reference implementation of the `Renderer` seam
(`../AGENTS.md`). Forward rendering + cascaded shadows + IBL + reflection probes
+ a screen-space post stack. This guide is the map so you don't re-read
~3000 lines of `.mm` + `.metal` to find one thing. Symbol names are greppable;
line numbers drift, so grep the name.

Files: `metal_renderer.h` (thin), `metal_renderer.mm` (everything, pimpl'd as
`MetalRenderer::Impl`), and the shaders in `shaders/metal/`.

## How shaders are built (important, differs from Vulkan)

At `initialize`, the six `.metal` files are **read from disk and concatenated**
in dependency order — `shader_types.h` → `common.metal` → `environment.metal` →
`shadows.metal` → `lighting.metal` → `post.metal` — with `#line N "file"`
directives between them, then compiled in one `newLibraryWithSource:` call. So:
- **Shaders compile at runtime from source** (no offline step). Edit a `.metal`
  file, rerun — no rebuild. Errors report the right file via the `#line` pragmas.
- Paths are **relative to the working directory** → run the viewer from the repo
  root (or the shader reads fail).
- Pipelines are `MTLRenderPipelineState`/`MTLComputePipelineState` built from
  `newFunctionWithName:` lookups. Adding a shader entry point = add the function
  in the `.metal` file *and* build its pipeline in `initialize`.

(Vulkan instead compiles GLSL→SPIR-V offline at build time — see
`../vulkan/AGENTS.md`. The shaders are ported per-backend; only `shader_types.h`
layout is shared in spirit.)

## Per-frame pass graph (`endFrame`)

Everything is queued during the frame (`drawMesh`/`drawTerrain` push to
`opaqueDrawCalls` / `transparentDrawCalls` / `terrainDrawCalls`); `endFrame` runs
the whole graph in one command buffer:

1. **Reflection-probe bake** (`bakeProbes`) — once, when draw calls first exist.
   6 faces/probe → `probeCubemapArray`. *Forward-Z, counterclockwise* (the
   X-mirrored cube-face projection flips winding) — the one place that differs
   from the main pass.
2. **Shadow pass** — per cascade (`activeCascadeCount`, ≤ `RT_MAX_CASCADES`),
   depth-only into `shadowMap` (Depth32Float 2D array, 2048²×4). Conservative
   sphere cull per cascade; `shadowInstancedPipeline` + `terrainShadowPipeline`.
3. **Main color pass** — single encoder, MRT into `sceneColorTexture`
   (RGBA16Float HDR) + `viewNormalTexture` (RGBA8Unorm, for SSR/SSAO), depth
   `depthTexture` (**reverse-Z**, clear 0, test Greater). Sub-stages in order:
   skybox → opaque (sorted, then mesh-batched/instanced) → terrain (CDLOD morph)
   → foliage (depth prepass then early-Z lit, if `depthPrepassEnabled`) →
   transparent (back-to-front) → wireframe overlay.
4. **Post compute** — one compute encoder: SSAO (GTAO → bilateral blur →
   temporal reproject) → SSR (ray-march → bilateral blur) → bloom (downsample
   pyramid → upsample) → DOF. All half-res RGBA16Float/R16Float targets.
5. **Composite** (`fragmentComposite`) → drawable (BGRA8Unorm): tone map
   (ACES/AgX) + grade, apply AO/SSR/bloom, draw sky for far-plane pixels, debug
   views.
6. **Lens warp** (`fragmentLensWarp`) → drawable, if lens effects active: Brown
   distortion + chromatic aberration + vignette over `postLDRTexture`.
7. **ImGui** render (if `RT_ENABLE_IMGUI`), then `presentDrawable` + `commit`.

## Pipelines (built in `initialize`)

Geometry (render): `opaquePipeline`, `terrainPipeline`, `transparentPipeline`,
`opaqueInstancedPipeline`, `transparentInstancedPipeline`, `foliageDepthPipeline`
(no color), `foliageLitPipeline` (early-Z), `skyboxPipeline`,
`equirectBakePipeline`, `compositePipeline`, `lensWarpPipeline`, `shadowPipeline`,
`shadowInstancedPipeline`, `terrainShadowPipeline`.
Compute: `irradiancePipeline`, `brdfLUTPipeline`, `prefilterPipeline`,
`ssrPipeline`+`ssrBlurH/V`, `aoPipeline`+`aoBlurH/V`+`aoTemporalPipeline`,
`bloomDownsamplePipeline`+`bloomUpsamplePipeline`, `dofPipeline`.

## Resources & handles

- `meshes` is a `SlotMap<GPUMesh, MeshTag>` (vertex+index `MTLBuffer`, indexCount,
  materialIndex, bounds). `textures` is a `SlotMap<id<MTLTexture>, TextureTag>`.
  `MeshHandle`/`TextureHandle` index these (stale-handle safe).
- `uploadMesh` packs `Vertex` → `GPUVertex`; `uploadTexture` (RGBA8Unorm+mips),
  `uploadTextureHDR` (RGBA16Float `__fp16`+mips).
- **Environment (ADR-0016):** `setEnvironmentMap` → `bakeEnvironmentCubemap`
  (equirect → 1024² cube) → `bakeEnvironmentIBL` (prefiltered 128²×5 mips +
  irradiance 32²). `brdfLUT` (256² RG16Float) is computed once at init.
- **Instancing** uses 3-deep ring buffers (`instanceBuffers`,
  `shadowInstanceBuffers`, `foliageInstanceBuffers`) indexed by `frameIndex`.
  Caps: 4096 opaque / 16384 shadow / 8192 foliage; overflow falls back to
  single-draw.

## Uniform binding map (the GPU data flow)

GPU structs live in `shaders/metal/shader_types.h` (shared C++/MSL). Bound via
`setVertex/FragmentBytes` except `LightUniforms` (a `lightBuffer`, exceeds the
4 KB `setBytes` limit). Fragment buffer/texture slots are **shared between the
shadow setup and the skybox** — the skybox clobbers texture 0 / sampler 0 /
fragment buffer 5, so they are **restored after the skybox** or the lit pass
reads garbage (a 0 `shadowMapSize` → NaN). This is the #1 footgun in the file.

## Conventions you must not break

- **Reverse-Z** main pass: projection via `Mat4::reverseZ()`, depth clears to 0,
  test Greater. Linear-depth reconstruction (SSR/SSAO/temporal/DOF) assumes it.
  The probe bake is the exception (forward-Z, self-contained).
- **Winding:** clockwise front faces, back-cull — except the probe bake
  (counterclockwise, see above).
- **Render-target formats:** scene HDR RGBA16Float; view-normals RGBA8Unorm;
  depth Depth32Float; LDR/drawable BGRA8Unorm; shadow Depth32Float array.
- **Material texture flags** (bitmask): albedo / metallic-roughness / normal / AO
  / emissive present-bits drive which maps the fragment samples; absent = default
  white.
- Run from the repo root (runtime shader-file reads). `RT_FRAME_DUMP` captures
  frame ~90 to PNG (headless verification); `RT_DUMP_ENV` dumps baked env faces.

## Shader files

| File | Role / key entry points |
| --- | --- |
| `shader_types.h` | All GPU structs (Camera/Light/Material/Instance/Shadow/Probe/Env/SSAO/SSR/Bloom/Composite/Lens/DOF/Terrain uniforms) |
| `common.metal` | `Vertex`/`VertexOut`/`GBufferOut`, Fresnel/BRDF helpers, the analytic `applySurface` library (brick…road markings) |
| `environment.metal` | `vertexSkybox`/`fragmentSkybox`, `fragmentEquirectBake`, compute `integrateBRDF`/`prefilterEnvMap`/`convolveIrradiance` |
| `shadows.metal` | `vertexShadow`(+`Instanced`), `terrainVertexShadow`, `computeShadow` (cascade select + PCF + cross-fade) |
| `lighting.metal` | `vertexMain`/`terrainVertexMain`/`vertexMainInstanced`; `fragmentMain`(+`Instanced`) Cook-Torrance GGX + IBL + shadows + probes + fog; foliage depth/lit variants |
| `post.metal` | `ssrRayMarch`+blurs, `gtaoCompute`+blurs+`aoTemporal`, `bloomDownsample`/`Upsample`, `vertexComposite`/`fragmentComposite`, `dofGather`, `fragmentLensWarp` |

## When you change something

- **New post effect / knob:** add the field to `Renderer` (`../renderer.h`),
  read it into the matching `*Uniforms` in `shader_types.h`, build its
  pipeline in `initialize`, slot it into the `endFrame` graph, and mirror it in
  the Vulkan backend (parity, ADR-0057). **Record it in
  `docs/renderer-parity.md`** — the Metal↔Vulkan ledger — so the other backend's
  gap is tracked until it catches up and is device-verified.
- **New geometry feature:** decide which of the 7 geometry pipelines it rides (or
  add one), and whether it needs a shadow-caster variant.
- **Touching shadows or skybox:** re-check the shared-slot restore after skybox.
