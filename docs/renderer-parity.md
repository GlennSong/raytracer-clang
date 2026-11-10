# Renderer Parity Matrix — Vulkan ↔ Metal ↔ WebGPU

The project ships three render backends behind the `Renderer` seam (ADR-0001):
**Vulkan** (Linux + Windows, ADR-0057), **Metal** (macOS / visionOS) and
**WebGPU** (the browser build, ADR-0058, `docs/webgpu-renderer-plan.md`). The
stated goal is graphical parity — the viewport should look the same on any
platform. This document is the running ledger of that parity, plus a
verification log, because development happens on one machine at a time and a
feature added (or checked) on one backend can silently lag on the others.

**Vulkan is now the reference backend.** Since August 2026 development happens
on Linux (RTX 3080), and most rendering features since then landed in Vulkan
only. Metal was the reference before that; the rows written then still read
that way. The owner's position (2026-11-07): Metal and WebGPU not having parity
is fine for now — this ledger is the list of what they lack.

## How to maintain this (the process)

**Whenever you add or change a renderer feature, update this file in the same
change.** Concretely:

1. Add or edit the feature's row. Set the status for the backend you touched.
2. If you implemented a feature on only one backend, mark the others ❌/⚠️ so
   the gap is visible — don't leave it blank.
3. Status is two things: *implemented?* and *verified on real hardware?* Code
   written blind (no device to run it on — e.g. Metal edits made on Linux, or
   any WebGPU change) is 🟡 until someone runs it on the device and confirms it
   matches. Flip 🟡 → ✅ only after a device check, and add a dated line to the
   Verification Log below.
4. If a backend approximates another (e.g. analytic IBL vs a baked cube), use
   ⚠️ and say how it differs in Notes.
5. When you close a gap, also strike it from the Parity backlog below the
   matrix.

This is deliberately tech-debt accounting: ⚠️/❌/🟡 cells are the backlog.

## Legend

| Symbol | Meaning |
| --- | --- |
| ✅ | Implemented **and** verified on device |
| 🟡 | Implemented, **not yet** verified on device (written blind / awaiting a run) |
| ⚠️ | Partial or approximated — renders, but not 1:1 with the reference backend |
| ❌ | Not implemented |
| — | Not applicable |

How to read the columns:

- **Vulkan** reflects `src/renderer/vulkan/` + `shaders/vulkan/`. Rows added in
  the 2026-11-07 re-survey are ✅ where the feature was developed and shot on the
  RTX 3080 (per its ADR/commit); older rows keep their logged status.
- **Metal** reflects `src/renderer/metal/` + `shaders/metal/`. Features from
  before August 2026 are ✅ (developed on the author's Mac). Metal shader edits
  made since then were written on Linux with no Mac in the loop, so they are 🟡.
- **WebGPU** reflects `src/renderer/webgpu/webgpu_renderer.cpp` +
  `shaders/webgpu/`. It compiles and links (emsdk 6.0.1) but **has never been
  checked in a real GPU browser** (headless SwiftShader does not composite the
  canvas), so nothing on it is ✅. Only WGSL modules the renderer embeds
  (`kMeshWgsl`, `kSsaoWgsl`, `kSsrWgsl`, `kBloomWgsl`, `kCompositeWgsl`,
  `kBrdfWgsl`, `kBlitWgsl`, `kAtmosphereWgsl`) run at all; `clouds.wgsl`,
  `water.wgsl` and `star.wgsl` exist but are not wired.

## Matrix

### Forward shading & materials
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| PBR (albedo/metallic/roughness/emission) | ✅ | ✅ | 🟡 | Cook-Torrance GGX; verified rendering `arena.json` (Vulkan). WebGPU `mesh.wgsl` is a port. |
| Texture maps: albedo / MR / AO / emissive | ✅ | ✅ | 🟡 | glTF MR convention. |
| Normal mapping (TBN) | ✅ | ✅ | 🟡 | Vulkan `mesh.vert` passes world tangent; `mesh.frag` builds the TBN and perturbs N on texFlags bit 2 (ported from `lighting.metal`). WebGPU: only with a map and a real tangent. |
| Per-vertex tint | ✅ | ✅ | 🟡 | |
| Analytic procedural surface library | ⚠️ | ✅ | ⚠️ | 2026-08-23: the Vulkan port had drifted — `surfTerrain` (13), `surfWater` (12) and the whole `applySurfaceRelief` layer (road/terrain/water normal+roughness) were missing from `mesh.frag`, and the road tilt still used the pre-c5fe504 0.6. All ported to current Metal; water albedo+waves device-verified (RTX 3080), terrain grain/relief and the 0.35 road tilt await a Metal-side A/B (same-pose `shot` captures both backends now). Originally ported byte-for-byte from the Metal library, which since 2026-07-29 is one module per material: `surfaces_facade.metal` (the ten tiling patterns) + `surface_{road,water,terrain}.metal` (each holding that material's albedo **and** its normal/roughness relief) + `surfaces.metal` (the `applySurface` / `applySurfaceRelief` dispatchers). The primitives they build on (`hash21`/`vnoise2`/`fbm2`/`tile1`/`surfUV`) stay in `common.metal`. **WebGPU:** ids 1–11 only (facade patterns + road markings) and the road relief; no `surfTerrain`/`surfWater` and no terrain/water relief. |
| Directional / point / spot lights | ✅ | ✅ | 🟡 | ADR-0017 physical units. Metal and WebGPU loop 32 lights a frame; Vulkan lights from clusters (see Clustered lights). |
| Fog | ✅ | ✅ | ⚠️ | Aerial-perspective fog in `mesh.frag` (1-exp(-density·dist)), params via the globals UBO `fog` field. Vulkan fades toward the **sky** the surface occludes (d9003dd7; Metal's scattering sky does the same). WebGPU still fades toward the fixed `fog.rgb` — the old Vulkan behaviour that turned night distances white. |
| Underwater (#58): absorption, caustics, Snell's window | ✅ | ❌ | ❌ | 2026-09-28, Vulkan only (`composite.frag` `underwaterScene`, driven by `SceneLighting::underwater` from UnderwaterSystem): per-channel absorption along each ray's underwater path, iterated-warp caustics on up-facing bed, the surface from below (sky disc inside ~48.6 deg, the depths mirrored outside). Device-verified on the RTX 3080 in river_valley's sea. Metal and WebGPU ignore the params (surface-level scenes unchanged). |
| Transparency / alpha blending | 🟡 | ✅ | ❌ | Material `opacity < 1` routes to a blended pipeline (Vulkan: premultiplied, no depth write, normal G-buffer masked), drawn back-to-front after opaque. Vulkan has been in daily use through the glass work (Sept–Oct) but no formal device check is logged. **WebGPU has no transparent pass**: `opacity` is ignored, so glass, water panes and sprites draw opaque. |
| Alpha-tested foliage + depth prepass | ✅ | ✅ | 🟡 | `mesh.frag` discards under the albedo alpha mask (FLAG_ALPHA_TEST) — the visual cutout. Metal also has the foliage depth prepass (`fragmentFoliageDepthInstanced`); on Vulkan/WebGPU that perf step is still a follow-up. |

### Shadows
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Cascaded shadow maps + PCF | ✅ | ✅ | 🟡 | Device-verified on Vulkan (Phase 3). WebGPU uses a comparison sampler (hardware 2x2). |
| Shadow strength | ✅ | ✅ | ❌ | WebGPU's shadow term is on/off; no strength uniform. |
| Shadow artistic tint / ambientStrength | 🟡 | ✅ | ❌ | Occluded direct + ambient lerp toward `shadowTint` (globals); `ambientStrength` darkens IBL separately. Ported from `lighting.metal`. Not in `mesh.wgsl`. |

### Environment & IBL
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Procedural sky + day/night + FBM clouds | ✅ | ✅ | ⚠️ | `sky.frag`; part of the verified arena render. WebGPU's `fs_sky` draws the gradient, sun, moon, stars and city glow but **no 2D cloud layer**. |
| Terrain horizon shadows (mountains) | ✅ | ❌ | ❌ | 2026-08-26: `terrainShadow` in mesh.frag — an R8 horizon map (set 0 binding 4; `setTerrainHorizon`) of sin(horizon elevation) toward light 0, marched on the CPU from a 256² raster of the analytic terrain (`engine/procgen/terrain_horizon.*`, job-pool rows) whenever the light moves > 2°; multiplies the cascade visibility. The camera-side disc occlusion is backend-agnostic (CPU). Metal/WebGPU: the `setTerrainHorizon` seam is a no-op — bind the texture and port the 12-line lookup. |
| Stars + Milky Way (celestial sphere) | ✅ | 🟡 | 🟡 | 2026-08-26: `starField` — procedural field hashed on a seamless 3-D LATTICE over the EQUATORIAL frame (`skyCelX/Y/Z` globals, rotated by sidereal time on the CPU; one candidate per voxel, 27-neighbour lookup), power-law brightness, blue-white..warm tint, Milky Way band on the real galactic pole; gated by `skyStars.x` (day / moon wash), horizon extinction, under the clouds. Vulkan device-run on the metro; `environment.metal` (both paths) and `mesh.wgsl` mirror it, uncompiled here. |
| Moon disc with phases (the month) | ✅ | 🟡 | 🟡 | 2026-08-26: `moonDisc` — a sphere disc lit from the TRUE sun (`skyMoonDir`/`skyMoonSun` globals), crescent/quarter/full by construction, earthshine floor, glow ∝ lit fraction. Vulkan device-run on the metro (reference); `environment.metal` (both the procedural and scattering paths) and `mesh.wgsl` mirror it, uncompiled here. |
| HDR equirectangular environment | 🟡 | ✅ | 🟡 | Vulkan and WebGPU sample the equirect directly (`uploadTextureHDR`); unverified. |
| IBL: irradiance + GGX-prefilter + BRDF LUT | ⚠️ | ✅ | ⚠️ | **BRDF LUT done** on Vulkan (🟡, baked split-sum at set 0 binding 3, used by `mesh.frag`) and WebGPU (`brdf_lut.wgsl`). Irradiance + GGX-prefilter still use the analytic sky/equirect approximation on both (WebGPU blurs by a crude `mix` toward irradiance) — the baked prefiltered + irradiance cubes are Metal-only. |
| Reflection probes (parallax) | ✅ | ✅ | ❌ | 2026-11 (ADR-0372..0375, 0380): Vulkan's own probes, not Metal's -- ProbeSystem places them along the streets and indoors, the renderer captures a few faces a frame (G-buffer cube array), relights them each frame (`probe_relight.comp`), prefilters their mips (`probe_prefilter.comp`) and `mesh.frag` reads them by a plan grid with a parallax box; near the camera the reflection rays replace them (ADR-0376). Read only under the physical sky (`RT_SKY=physical`). WebGPU: `setReflectionProbes` is a no-op. |
| Physically-based scattering sky (transmittance + sky-view LUTs) | ✅ | ✅ | ❌ | Vulkan since 2026-11 (ADR-0366..0371): the CPU's transmittance and multiple-scattering tables (`engine/sky/sky_atmosphere.h`), the sky view each frame (`sky_view.comp`), the aerial-perspective volume, the sky cube with GGX mips and the sky's light as harmonics; the clouds' sun tinted by the air above them (ADR-0369). Opt-in by `RT_SKY=physical`; the default frame still draws the analytic day model. Metal: Piedmont P5 (50b92739, 2026-07-25), `environment.metal`, opt-in per level -- a different model from Vulkan's, not a port. |
| Sky horizon by direction (ADR-0339) | ✅ | ❌ | ❌ | 2026-11-05 (0a429b55): `skyHorizonToward` in `sky_common.glsl`, shared by `sky.frag` and the ambient in `mesh.frag` — at a low sun the horizon is warm only toward the sun, so walls facing away are no longer filled warm. Metal/WebGPU use one horizon colour all round. |
| Light pollution (city sky glow, stars fade) | ✅ | 🟡 | 🟡 | dc05c3cb: a warm horizon dome toward the city (`skyCity`), the faint stars and the Milky Way fading with pollution. Ported to `environment.metal` and `mesh.wgsl`, uncompiled/unrun there. |
| Ambient strength scales sky light, not its reflection | ✅ | ✅ | 🟡 | 5b269793 (2026-10-16): Vulkan and WebGPU brought in line with Metal — mirrors (glass towers) show the sky at its own brightness. |
| Planetary atmosphere (single scattering) | ❌ | 🟡 | 🟡 | Fullscreen Rayleigh+Mie raymarch (procedural-planet-plan P3), a port of the tested CPU reference `engine/procgen/atmosphere.cpp`. Shaders authored for Metal and WebGPU (the Vulkan GLSL copy, never wired, was deleted in #117 wave 0; it is in git history): `shaders/metal/atmosphere.metal`, `shaders/webgpu/atmosphere.wgsl` (**naga-validated WGSL**; Metal compile-unverified). **Pipeline wiring done on Metal AND WebGPU** as an ADDITIVE glow (loadOp Load, blend One+One): the raymarch runs after the scene / before post so the limb halo blooms, driven by `setAtmosphere(AtmosphereRenderParams)` from the level's top-level `atmosphere` block; the WGSL variant drops the scene fetch / view-transmittance term (the blend composites). Metal: `metal_renderer.mm endFrame`. WebGPU: `webgpu_renderer.cpp recordAtmosphere` (couldn't compile on-device — the emdawnwebgpu port host is blocked by egress policy). **Vulkan: none** (status corrected from 🟡 to ❌ in the 2026-11-07 survey: no shader, no `setAtmosphere`). No device run yet on any backend. Multiple scattering (Hillaire) is the follow-up. |
| Volumetric clouds (slab + shell) | 🟡 | ✅ | ❌ | 2026-08-23: VULKAN WIRED — half-res march + depth-guided bilateral composite onto HDR before post, the shared cloud_noise.h Perlin-Worley bake uploaded as two 3D textures, CloudUniforms in a per-frame UBO ring; clouds.frag rewritten to the CURRENT clouds.metal (weather map, stratified 2x2 dither + IGN, two step sizes, zero-run stretch + entry refinement, analytic in-scatter, aerial fade). Device-run on the metro: deck renders over the city, 2D overlay retired while active. Slice-banding on far undersides + top stipple need a Metal same-pose A/B; the scattering-sky sun-transmittance tint is skipped (no scattering sky on Vulkan yet). Since then Vulkan has moved well past Metal — see the cloud rows below. **WebGPU:** `clouds.wgsl` (an inline hash-fbm port) exists but the renderer never embeds or runs it. |
| Cloud weather map (formations, gaps) | ✅ | ✅ | ❌ | 2c87bb4a (Metal, 2026-08-20), carried into the Vulkan rewrite. |
| Cloud history: reprojected temporal resolve | ✅ | ❌ | ❌ | ADR-0328 / 0334 / 0335 (#187, 2026-11-05): `clouds_resolve.frag` blends the half-res march with last frame's clouds, reprojected by each pixel's cloud depth, camera-relative in double; clamped to the 3x3 neighbourhood. `RT_CLOUD_HISTORY` / `RT_CLOUD_DEPTH` A/B. Metal has no cloud history. |
| Cloud march: per-pixel offsets, sun cone, haze on cloud only | ✅ | ❌ | ❌ | ADR-0332 / 0333 (#187): each pixel's own march offset inside every step (the island's far-cloud blocks), rays from 100 m out, the sun's light march in a cone, haze applied to cloud only. `RT_CLOUD_JITTER=coherent` A/B. |
| Cloud types per weather | ✅ | ❌ | ❌ | ADR-0336 (91ef1d97): the weather map's G channel is a cloud-type field; each weather its own clouds. The shared `cloud_noise.h` bakes the map, but only `clouds.frag` reads the types. `RT_CLOUD_TYPES=0` A/B. |
| Water surface (glint / Fresnel / depth-tint) | ❌ | 🟡 | 🟡 | A separate water SURFACE shader (procedural-planet-plan P2): animated ripple normals, GGX sun glint, Schlick-Fresnel sky reflection, depth-tinted transmission + shoreline foam. Reads the existing `Surface::Water` UV convention (uv.x depth, uv.y shore). `shaders/metal/water.metal`, `shaders/webgpu/water.wgsl` (**naga-validated WGSL**). **Pipeline wiring not done** on either. No Vulkan shader (its unwired GLSL copy was deleted in #117 wave 0; status corrected from 🟡 to ❌) — Vulkan shades water in `mesh.frag` instead (see the Water rows under Materials). |
| Star / sun disc (blackbody + limb darkening) | ❌ | 🟡 | 🟡 | Fullscreen star pass (procedural-planet-plan P6): a bright HDR disc with limb darkening + a Planckian blackbody colour from temperature, plus a bloom-friendly glow, gated on the reverse-Z far plane. `shaders/metal/star.metal`, `shaders/webgpu/star.wgsl` (**naga-validated WGSL**). No Vulkan shader (status corrected from 🟡 to ❌). **Pipeline wiring not done** anywhere. No device run. |

### Lighting (new since August)
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Clustered lights (default on Vulkan; `RT_CLUSTERED=0` is the forward A/B) | ✅ | ❌ | ❌ | ADR-0361 (723b3d77) built it; ADR-0363 made it the default. A froxel grid binned on the GPU (`cluster_bin.comp`, ADR-0364; `RT_GPU_CLUSTERS=0` bins on the CPU with `light_clusters.cpp`), three SSBOs at set 0 bindings 8–10; `mesh.frag` loops only its froxel's lights, the sun from the 32. `Renderer::lightCapacity()` tells LightSystem (ADR-0362) how many it may hand over: 4,096 here, 31 on Metal/WebGPU, where the picked set is trimmed by importance. |
| Shadow caster culling per cascade | ✅ | ✅ | 🟡 | Vulkan fcbdb2bf (each cascade drew the whole frame; `RT_NO_SHADOW_CULL` A/B). Metal has its own per-cascade test (`RT_OLD_SHADOW_CULL` A/B). WebGPU carries per-cascade cull data, unrun. |

### Materials & surfaces (new since August)
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Material features: triplanar, per-instance variation, top layer | ✅ | ❌ | ❌ | ADR-0098 (9d2249a9): world-space triplanar albedo/normal, brightness/hue hashed per instance, moss/snow/dust on up-facing faces. `renderer.h` says "Metal owed". Rocks and other kit meshes look flat/untextured without it. |
| UV blend (bark crossfade, FLAG_UV_BLEND) | ✅ | ❌ | ❌ | ADR-0183: the tangent slot carries a second UV + weight; albedo and normal are sampled at both and mixed. Metal/WebGPU read the first UV and treat the data as a tangent, so tree normal maps are wrong where it is set. |
| Terrain layers (textured ground: grass/dirt/sand/rock, hex tiling, rock sub-layers) | ✅ | ⚠️ | ❌ | 40768737, ADR-0209 / 0210 (`terrainLayers`, `texHex`, `layerRock` in `mesh.frag`). Metal shades the ground with the analytic `surfTerrain` only; WebGPU with the vertex colour. |
| Terrain normal map + cover / macro maps (FLAG_TERRAIN) | ✅ | ❌ | ❌ | ADR-0205 / 0208 (a1aace05, 8c5e07e2): far ground lit by a baked normal map, cover weights and macro shading (gullies darker, ridges paler) per texel; set 0 bindings 5–7. `setTerrainNormalMap` / `setTerrainSurfaceMaps` are no-ops on Metal and WebGPU. The CDLOD coarse-normal morph (ADR-0204) is on all three. |
| Ground surface relief, crags, stains | ✅ | ❌ | ❌ | ADR-0207 (layer heights + hummock field), ADR-0120 (rock crags, snow ripple), ADR-0213 (streak stains under ledges). Vulkan `mesh.frag` only. |
| Rivers and lakes: one water mesh with a flow map (Surface::River, id 20) | ✅ | ❌ | ❌ | ADR-0099 (d0818ed8): flow direction in the tangent, speed/fade in colour; two-phase advected noise (Vlachos). Metal/WebGPU have no id 20 and draw the base colour. |
| Water composite: depth opacity, Fresnel sky, sparkle, shore foam | ✅ | ⚠️ | ❌ | ADR-0099 (3fdcce6a): Beer-Lambert depth over the bed, Fresnel sky, per-cell sun sparkle, shore foam, SSR mirror. Metal has `surfWater` albedo + wave relief (id 12) but not this composite. WebGPU has neither. |
| Surf: breakers on a signed waterline distance | ✅ | ❌ | ❌ | ADR-0330 (92bc4f73). |
| SSAO fades with camera distance | ✅ | ❌ | ❌ | ADR-0128: `ssao.frag` fades AO out over 120–350 m so the far kernel stops darkening whole hillsides. Metal's GTAO and `ssao.wgsl` have no distance fade. |

### Foliage & impostors (new since August)
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Grass field: fade, continuous thinning, no shadow (FLAG_GRASS) | ✅ | ⚠️ | ⚠️ | 7ee76ef8, 56c0ce9a: clumps shrink away over a fade band by their rank (the transform's bottom row). Metal draws the field whole, unfaded, with shadows. WebGPU zeroes the bottom row (83127571) and draws it whole. |
| Far grass: camera-facing cards, slope shear, flowers | ✅ | ❌ | ❌ | ADR-0097 (5f06d691) far field, reworked by ADR-0329 (247d99e6): one card turned to the camera in `mesh.vert`, sheared to the ground's slope. Elsewhere the cards stand still. |
| Grass transmission + ground-normal shading | ✅ | ❌ | ❌ | cef1b397 (sun through the blades, halved in 46d8d4a3), ADR-0202 (grass shades with its ground's slope, `RT_GRASS_GROUND=0` A/B). |
| Ground clutter (FLAG_CLUTTER) | ✅ | ⚠️ | ⚠️ | ADR-0212: stones and twigs planted like grass, lit as solids. Elsewhere drawn as plain meshes without the grass fade. |
| Foliage card lighting (FLAG_FOLIAGE, banded crowns) | ✅ | ❌ | ❌ | ac6efb65: the card normal faces the viewer and bends part way to the sky, so a crown has no dark card backs. Metal/WebGPU light cards by their flat normal. |
| Leaf transmission | ✅ | ❌ | ❌ | ADR-0129 / 0130: sun through the leaves at the crown's lit rim, tinted by the leaf colour. |
| Tree cards fade edge-on; alpha coverage kept down the mips | ✅ | ❌ | ❌ | ADR-0190 (81a81ef9) + the Golus coverage fix for grass and banded tree cards. Elsewhere edge-on cards show as slivers and far cards thin away. |
| LOD band: dithered per-pixel crossfade (FLAG_LOD_BAND) | ✅ | ❌ | ❌ | ADR-0129, 331d136f (complementary Bayer edges). `supportsLodBand()` is false on Metal/WebGPU, but the forest still flags its near models and impostors with the band, so on those backends **both draw at once** inside the band. |
| City tier crossfade | ✅ | ❌ | ❌ | ADR-0226 (6419faea): a city cell crossfades between its LOD tiers through FLAG_LOD_BAND. `render_system.cpp` checks `supportsLodBand()`, so Metal/WebGPU fall back to a hard swap (graceful). |
| Many-view impostors (FLAG_IMPOSTOR_VIEWS) | ✅ | ❌ | ❌ | ADR-0193 (c8708759): one card per far tree, turned to the camera in `mesh.vert`; `mesh.frag` blends the 4 nearest of 8x3 baked views. On by default (`RT_IMPOSTOR_VIEWS=0` turns it off). **Likely broken on Metal/WebGPU:** the card is not turned, the whole view atlas is sampled as one texture, and the FLAG_SHADOW_ONLY crossed shadow cards are drawn visibly. Unverified — no device run. |
| Impostor self-shadow | ✅ | ❌ | ❌ | ADR-0202 (9966d6f3): a far crown shadows itself from its sun-side views' depth. `RT_IMPOSTOR_SELF_SHADOW=0` A/B. |
| Alpha-cut shadow casters | ✅ | ❌ | ❌ | ADR-0129 (0759b53f): `mesh_shadow_alpha.vert/.frag` discards under the leaf mask in the shadow pass. Metal's and WebGPU's shadow pipelines are depth-only (no fragment stage), so leaf cards cast solid quad shadows. |
| Wind from the weather, gust fronts | ✅ | ⚠️ | ⚠️ | ADR-0337 (34143f25): one weather wind for everything; travelling gust bands and a mean downwind lean in `mesh.vert`. Metal/WebGPU have the basic FLAG_WIND sway only. |

### City: windows, glass, lamps (new since August)
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Lit panes: per-window tint + glass colour packed in the vertex colour | ✅ | 🟡 | 🟡 | be24391d, 77121514 (FLAG_EMISSIVE_VERTEX_TINT + the packed glass/lit-index decode). Mirrored in `lighting_surface.metal` (b0ecc01c fixed its vertex colour plumbing) and `mesh.wgsl`. |
| Interior-mapped rooms behind lit panes (FLAG_INTERIOR_MAP) | ✅ | 🟡 | ❌ | 71ae65b1, d17af92a: a virtual 3 m x storey x 4 m room per pane, 16 rooms in an atlas, blinds on 3 in 10. Metal has the port (d17af92a "the Metal pane shader"); `renderer.h`'s "Metal owed" comment is stale. WebGPU decodes the pane colour but shows no room. |
| Front-only facade panes (FLAG_FRONT_ONLY) | ✅ | 🟡 | ❌ | 6b39d4b9: the exterior pane is gone from inside, so the interior's own pane shows the city. |
| Facade glass reflects like coated glass (`RT_GLASS_REFLECT`, 3x) | ✅ | ❌ | ❌ | b8ec6b32: F0 of metallic-0.9/roughness-0.08 glass scaled by `ambient.w`. WebGPU uses `ambient.w` for wind time. |
| Glass as a filter (`RT_GLASS`) | ✅ | ❌ | ❌ | 22b4dfad: clear glass passes the view (absorbing 0.4 of its opacity) and adds its reflection, instead of laying a grey veil. Metal uses the old alpha blend; WebGPU has no transparent pass. |
| The viewer's own glass: room reflection, ceiling panels, double glazing | ✅ | ❌ | ❌ | 22b4dfad, 6d98e22f, e5faa0ed: inside `SceneLighting::viewerBox`, windows reflect the lit room (`roomRadiance`) and the office ceiling panels (`ceilingPanels`) at ~15 %. |
| Emission follows paint (FLAG_EMISSION_FOLLOWS_PAINT) | ✅ | ❌ | 🟡 | 2a3f4499: a room's faint self-light scales with its paint (a dark bar stays dark). `renderer.h` says "Metal owed". |
| Traffic signal state (FLAG_SIGNAL_STATE) | ✅ | ❌ | ❌ | fe9726d4: one static signal model; the state in the instance's bottom row lights the green/amber/red lens in `mesh.vert`. Elsewhere all three lenses stay dark. |
| Vehicle lamps lit by a mask (FLAG_LAMP_STATE) | ✅ | 🟡 | ❌ | ADR-0342 (ecc43987, 2026-11-06): lenses tagged by role in their u; head/brake/indicator/marker mask in the bottom row. Metal port moves the mask to `albedo.w` (`lighting_entry.metal`), unrun. WebGPU zeroes the bottom row, so no lamps light. |
| Sprites: particles, beacon glows (FLAG_ALPHA_FROM_MAP) | ✅ | 🟡 | ❌ | 6ded9757, ADR-0239: the albedo map's alpha shapes a billboard in the transparent pass; sprites leave the glass path. Metal honours the flag in `lighting_surface.metal`. WebGPU has no transparent pass. |
| Sprites cast no shadow | ✅ | ⚠️ | ❌ | ADR-0328 addendum (ae23e3b2): the dust's square shadows. Metal casts from opaque draws only, which probably excludes sprites — unclear from code. |

### Screen-space post
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Bloom | ✅ | ✅ | 🟡 | Device-verified on Vulkan. WebGPU: bright pass + separable blur. |
| SSAO | ✅ | ✅ | 🟡 | Base verified; the Vulkan box-blur denoise (`ssao_blur.frag`) + IGN rotation are 🟡 (recent). Metal is GTAO; Vulkan/WebGPU a hemisphere kernel. |
| SSAO temporal reprojection | ❌ | ✅ | ❌ | Vulkan is spatial-only (blur). Needs motion vectors + history. Quality refinement, not a hard gap. |
| SSR | 🟡 | ✅ | 🟡 | Base verified; the binary-search refinement is recent/unverified. |
| Tonemap (ACES / AgX) + grade | ✅ | ✅ | 🟡 | Device-verified on Vulkan. |
| Lens (distortion / CA / vignette) | ✅ | ✅ | ❌ | Folded into composite; device-verified on Vulkan. Not in `composite.wgsl`. |
| Depth of field | 🟡 | ✅ | ❌ | `dof.frag` golden-angle gather; written blind, unverified. No WebGPU pass. |

### Geometry paths
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Instanced rendering (correctness) | ✅ | ✅ | 🟡 | Vulkan: real GPU instancing since ADR-0097 (3d20f028, the per-instance model at vertex binding 1; `RT_NO_INSTANCING=1` A/B) — the old "CPU `drawMesh` fallback" note is obsolete. Metal batches into instanced pipelines. WebGPU: hardware instancing, ~8x fewer draw calls (structural check, no browser run). |
| Per-draw material sets at scale | ✅ | — | — | 2026-08-23: the per-frame material descriptor pool was CAPPED at 2048 sets and silently SKIPPED every draw past it — at metro scale (~6k visible draws) whole buildings, the beach, the ocean and terrain nodes vanished view-dependently as the frustum-visible set crossed the cap. Now a grow-on-demand pool chain (2048-set links, appended on exhaustion, whole chain reset per frame); a draw drops only on true OOM, logged. Metal has no equivalent quota (encoder-bound textures), which is why the failure mode was invisible on the reference. WebGPU uses a 256-byte dynamic-uniform slot per draw. Device-verified: the same metro pose went from a brown void to beach+ocean+full city; zero pool warnings over a full run. |
| Vegetation wind sway | ✅ | ✅ | 🟡 | `mesh.vert` applies the FLAG_WIND sway (height-weighted, phase-offset; same constants as Metal) from the model base, so the per-instance fallback sways. Vulkan's gusts: see Foliage. |
| CDLOD terrain morph | ✅ | ✅ | 🟡 | `drawTerrain` override + `terrain.vert` morph each vertex toward its coarser-LOD position (tangent slot) over the [start,end] camera-distance band. Device-verified on Vulkan (stable after the reverse-Z fix). Since ADR-0204 all three also morph the normal toward the coarser level's. |
| Mipmaps | ✅ | ✅ | 🟡 | `createImageRGBA8` generates the full chain via a blit downsample; sampler already mip-aware (LINEAR, unclamped LOD). WebGPU: a blit-shader downsample (`blit.wgsl`). |
| Back-face culling + FLAG_TWO_SIDED | ✅ | ✅ | ❌ | 2026-08-23: Vulkan rendered EVERYTHING two-sided (the "Phase 1: cull nothing" deferral) — harmless until the flat-facade middle LOD (823986c) shipped coplanar front/back geometry that self-z-fights when both faces rasterize: buildings/vehicles dissolved view-dependently at distance on metro-scale levels (the "disappearing geometry" report). Metal never saw it (culls Back per batch, FLAG_TWO_SIDED opts out). Vulkan now mirrors that via a back-culled opaque pipeline variant (API 1.0, so a variant rather than dynamic cull state); terrain culls Back unconditionally like Metal's encoder default. Device-verified A/B at the reporter's exact camera pose. **WebGPU still culls nothing** (`cullMode None` on every pipeline), so it has the same flat-facade z-fight Vulkan had. |
| Reverse-Z depth | ✅ | ✅ | ❌ | Vulkan switched 2026-06-28 (the distant terrain flicker). WebGPU uses Depth24Plus with forward `LessEqual`, so the far-terrain flicker is expected there. |

### Geometry & performance (new since August)
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| GPU memory: one allocator (VMA), one upload queue, deferred destruction | ✅ | — | — | ADR-0094 (979a1f16). Backend-internal; Metal and WebGPU manage memory their own way. Part of the 9.6 → 3.6 GB metro peak. |
| 32-byte packed vertex | ✅ | ❌ | ❌ | ADR-0096 (04d4d5fb): octahedral snorm16 normal/tangent + 8-bit colour, the full layout only where data needs it (`RT_VERTEX_FULL=1` A/B). Metal and WebGPU upload the full float vertex (WebGPU's `GpuVertex` is 56 bytes). Memory, not looks. |
| Two-phase mesh upload (pack on a worker) | ✅ | ❌ | ❌ | 8749dba4: `prepareMesh` on a residency worker, `uploadPrepared` on the render thread. Others use the default raw path (hitches, not looks). |
| Occlusion culling against a recent frame's depth | ✅ | ❌ | ❌ | ADR-0100 (a5e18463): `occlusion_reduce.comp` reduces depth, the CPU tests boxes; hidden objects still cast shadows via FLAG_SHADOW_ONLY. `occlusionDepth()` is null elsewhere, so they simply draw everything (graceful). |
| FLAG_SHADOW_ONLY honoured | ✅ | ❌ | ❌ | Shadow-map-only draws. Set by occlusion (Vulkan only) **and unconditionally by the forest's impostor shadow cards**, which Metal/WebGPU then draw visibly. |
| GPU crowd (far travellers placed by compute) | ✅ | ❌ | ❌ | 92308af3 (`crowd.comp`): far cars and people placed from their route and clock on the GPU (100k–515k agents). `supportsGpuCrowd()` false elsewhere, so citysim draws its CPU band (graceful, slower). |
| GPU erosion bake (compute) | ✅ | ✅ | ❌ | ADR-0122 / 0123: `erosion.comp` + `erosion_water.comp`, the Linux twin of the Metal bake. A bake, not a frame pass. |
| Present sync off for benchmarks (`RT_VSYNC=0`) | ✅ | ✅ | — | `setPresentSync`. The browser owns WebGPU's present. |

### Debug & tooling
| Feature | Vulkan | Metal | WebGPU | Notes |
| --- | --- | --- | --- | --- |
| Debug views (AO/SSR/depth/normals/shadow/albedo/facing/cascades) | 🟡 | ✅ | ⚠️ | Full set implemented on Vulkan; albedo/depth fixes recent, unverified (2026-08-23: albedo view showed near-camera CDLOD terrain black on device — suspect, see log). WebGPU writes depth/normals/shadow/albedo/facing/cascades in `mesh.wgsl` and AO in the composite; an SSR view is unclear from code. |
| Headless frame capture (`shot` / RT_FRAME_DUMP) | ✅ | ✅ | ❌ | Vulkan `requestFrameDump` lands 2026-08-23: swapchain built with TRANSFER_SRC, armed frame copies the composited image to a host buffer, fence + `stbi_write_png` — same two arms and semantics as Metal. Device-verified (RTX 3080). This was the missing parity *instrument*: without it only Metal could be screenshot-compared. WebGPU has no capture (and SwiftShader can't composite the canvas). |
| Wireframe (modes 1 & 2) | ✅ | ✅ | ❌ | LINE-mode pipeline; device-verified on Vulkan. |
| Dear ImGui overlay | ✅ | ✅ | — | Vulkan backend (1.92 API) device-verified. The web build uses an HTML debug panel (`web/viewer.html`) instead. |
| Editor viewport (hosted window) | ✅ | ✅ | — | Vulkan surface from the Qt viewport; device-verified on Windows. |
| Gamepad | ✅ | ✅ | ⚠️ | Standalone viewer uses GLFW's joystick path off-Apple; the hosted editor's GCController poll is macOS-only. The web page feeds a virtual pad through `rt_web_axis` / `rt_web_button`. |
| GPU pass timing (`RT_GPU_TIMES`) | ✅ | ⚠️ | ❌ | fcbdb2bf: per-pass timestamp queries. Metal logs whole-command-buffer time only (`RT_GPU_TIME`, `lastGpuFrameMs`). |
| GPU memory report (`mem?`) | ✅ | ❌ | ❌ | 533c3213: `memoryReport()`, what is on the GPU by kind. |
| Game UI layer (map tool, UI text, wrist watch) | ✅ | ❌ | ❌ | ADR-0088 (4f3a599b), ADR-0248..0251: `submitUi` + `ui.vert/.frag`. A no-op on Metal/WebGPU, so the map tool and in-game text don't draw. |
| XR stereo rendering (visionOS) | — | ✅ | — | Aug 2026 (57a43664 and on): per-eye rendering through the compositor's drawables, hand skeletons, gaze-pinch teleport. Metal-only by design. |

## Parity backlog

The gaps, by backend, roughly most visible first. Strike an item when its row
goes ✅/🟡.

### Metal
1. Many-view forest impostors: cards not turned, atlas sampled whole, shadow-only crossed cards drawn visibly (FLAG_IMPOSTOR_VIEWS + FLAG_SHADOW_ONLY).
2. LOD band crossfade: forest near models and impostors both draw inside the band (FLAG_LOD_BAND).
3. Terrain layers, terrain normal/cover/macro maps, surface relief, crags, stains — the island ground falls back to `surfTerrain`.
4. Rivers/lakes flow-map water (id 20) and the water composite (depth opacity, Fresnel, sparkle, foam), surf.
5. Grass: fade/thinning, far camera-facing cards, transmission, ground-normal shading.
6. Foliage card lighting, leaf transmission, edge-on card fade, alpha-cut shadow casters.
7. Traffic signal state (lenses stay dark).
8. Glass: filter model, 3x facade reflectance, the viewer's own glass room/ceiling panels.
9. Clouds: history reprojection, per-pixel march offsets/sun cone, cloud types.
10. Terrain horizon shadows (mountains at sunset).
11. Sky horizon by direction (ADR-0339).
12. Underwater.
13. Material features (triplanar, variation, top layer) and UV blend.
14. Game UI layer (map tool, UI text, watch).
15. Emission follows paint; SSAO distance fade; gust wind.
16. Run the blind Metal ports once on a Mac: lamp mask, interior map, front-only panes, lit-pane decode, sprites, stars/moon/light pollution.
17. Perf only: occlusion culling, GPU crowd, packed vertex, two-phase upload, clustered lights, per-pass GPU timing, `mem?`.

### WebGPU
1. Never run in a real GPU browser — do that first; everything below is from code.
2. No transparent pass: glass, sprites, water panes draw opaque.
3. No back-face culling: the flat-facade LOD z-fights (the Aug "disappearing geometry" bug).
4. Forward-Z Depth24: far terrain flicker (reverse-Z missing).
5. Many-view impostors and FLAG_SHADOW_ONLY cards: same breakage as Metal; LOD band draws both.
6. Ground: no terrain surfaces at all (no `surfTerrain`, layers, maps, relief) — vertex colour only.
7. Water: no `surfWater`, no flow map, no composite; `water.wgsl` unwired.
8. No volumetric clouds (`clouds.wgsl` unwired) and no 2D sky cloud layer.
9. Fog fades to `fog.rgb` (white night distances).
10. Instance bottom row zeroed: no signal state, lamp mask or grass rank.
11. Grass/foliage: fade, far cards, transmission, foliage lighting, edge-on fade, alpha-cut shadows.
12. City glass: interior-mapped rooms, front-only panes, glass filter, viewer's glass, facade reflectance.
13. Terrain horizon shadows; sky horizon by direction; underwater.
14. Shadow strength and tint; lens; DOF; SSAO temporal.
15. Reflection probes; frame capture; wireframe; game UI layer; GPU timing.

## Verification Log

Dated record of what was confirmed on real hardware, so 🟡→✅ flips are auditable.

- **2026-06-27/28 (Windows / RTX 3060):** Phases 0–3 render `arena.json`
  (forward PBR, textures, lights, CSM, procedural sky). Default post stack runs
  validation-clean: bloom, SSAO, SSR, lens, ACES/AgX tonemap + grade
  (`independentBlend` fix confirmed needed). ImGui-on-Vulkan overlay works
  (1.92 `PipelineInfoMain` API). Editor viewport renders through the Vulkan
  backend via the Qt surface.
- **2026-06-28 (Windows / RTX 3060, 2nd pass):** confirmed working — mipmaps
  (smooth distant textures), normal mapping (surface detail), fog, vegetation
  wind + alpha-cut foliage, wireframe, SSAO (blur). Diagnosed + fixed: distant
  terrain flicker was forward-Z precision → switched to **reverse-Z** (confirmed:
  terrain no longer flickers; CDLOD renders stable).
  Clarified (not bugs): brick/checkerboard "speed lines" are procedural-surface
  aliasing (shared with Metal); DOF needs the camera panel + a low f-stop
  (f/8 gives sub-pixel CoC).
- **2026-08-23 (Linux / RTX 3080, first Linux device pass):** Vulkan frame
  capture implemented + verified (`shot` via the control channel; arena and
  metro captures). Scene-pass `deps[0]` gained explicit `FRAGMENT_SHADER` +
  `LATE_FRAGMENT_TESTS` in `srcStageMask` — matching the post passes' explicit
  style, but on scrutiny this is documentation, not a fix: a `srcStageMask`
  scope includes all logically EARLIER stages, so `COLOR_ATTACHMENT_OUTPUT`
  already covered the prior frame's fragment-shader reads. Machine-checked on
  device: sync validation (`validate_sync` + `syncval_submit_time_validation`,
  VVL 1.4.341) reports ZERO hazards on arena, both pre- and post-change — the
  frames-in-flight sharing of the scene targets is correctly ordered, and the
  "shared depth target" debt note is softer than feared. The view-dependent
  vanishing-geometry report is therefore STILL UNDIAGNOSED — it is not an
  attachment race. Surface library gaps ported (see the
  matrix row): water albedo + wave relief confirmed on device; terrain grain
  unconfirmed — inside the city the visible ground is citysim lot fill, not
  CDLOD terrain, so judge terrain against Metal same-pose captures. Albedo
  debug view showed near CDLOD terrain black at close range — untrusted as an
  instrument until re-checked.
- **2026-08-23 (Linux / RTX 3080, 2nd pass — the vanishing-geometry hunt):**
  Reproduced live via the new capture seam at the reporter's held camera pose.
  The facing debug view showed far flat-facade-LOD towers striped front/back —
  coplanar self-z-fighting — and geometry losing depth entirely (sky ground
  colour showing through). Root cause: Vulkan drew everything two-sided while
  Metal culls back faces; the flat-facade middle LOD (823986c, Metal-authored)
  made the divergence fatal. Fixed with the back-culled pipeline variant +
  FLAG_TWO_SIDED routing (see the matrix row); before/after verified at the
  same pose. The earlier sync-race theory for this symptom is retracted (see
  the 1st-pass entry — sync-val clean). SECOND mechanism found the same day
  behind the same symptom: the 2048-set material pool cap silently skipping
  draws (see the new Geometry-paths row) — the reporter's "faces/buildings/
  terrain still disappear by angle" after the culling fix was this, not
  winding. Both fixes verified at the reporter's exact camera pose.
- **Pending device check (Vulkan):** transparency; shadow tint; HDR equirect IBL (4b) + BRDF
  LUT; debug views (albedo/depth); depth of field (with a low f-stop); SSR
  binary-search refinement.
- **2026-11-07 (code survey, not a device check):** the ledger re-surveyed
  from code — `git log` of `shaders/` + `src/renderer/` since 2026-08-01, the
  rendering ADRs, and the Vulkan shaders diffed against `shaders/metal/` and
  `shaders/webgpu/`. Added the WebGPU column and the rows for features since
  August, with Vulkan as the reference. Vulkan ✅ on the new rows means the
  feature was developed and shot on the RTX 3080 per its ADR/commit, not a fresh
  pass. Corrections from code: Vulkan instancing is real GPU instancing
  (ADR-0097), and Vulkan has no atmosphere, water-surface or star-disc shader
  (🟡 → ❌). Nothing was flipped to ✅ on Metal or WebGPU.

## Known non-goals (all backends)
Per ADR-0057: unifying the viewer with the offline path tracer and hardware ray
tracing are out of scope and not tracked here. (ADR-0057 also ruled out compute
shaders; Vulkan now uses them — `crowd.comp`, `occlusion_reduce.comp`, the
erosion bake — so those are tracked as rows above.)
