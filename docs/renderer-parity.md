# Renderer Parity Matrix — Metal ↔ Vulkan

The project ships two render backends behind the `Renderer` seam (ADR-0001):
**Metal** (macOS, the reference) and **Vulkan** (Linux + Windows, ADR-0057). The
stated goal is graphical parity — the viewport should look the same on either
platform. This document is the running ledger of that parity, plus a verification
log, because development happens on one machine at a time and a feature added (or
checked) on one backend can silently lag on the other.

## How to maintain this (the process)

**Whenever you add or change a renderer feature, update this file in the same
change.** Concretely:

1. Add or edit the feature's row. Set the status for the backend you touched.
2. If you implemented a feature on only one backend, mark the other ❌/⚠️ so the
   gap is visible — don't leave it blank.
3. Status is two things: *implemented?* and *verified on real hardware?* Code
   written blind (the cloud env has no GPU) is 🟡 until someone runs it on the
   device and confirms it matches. Flip 🟡 → ✅ only after a device check, and
   add a dated line to the Verification Log below.
4. If a backend approximates the other (e.g. analytic IBL vs a baked cube), use
   ⚠️ and say how it differs in Notes.

This is deliberately tech-debt accounting: ⚠️/❌/🟡 cells are the backlog.

## Legend

| Symbol | Meaning |
| --- | --- |
| ✅ | Implemented **and** verified on device |
| 🟡 | Implemented, **not yet** verified on device (written blind / awaiting a run) |
| ⚠️ | Partial or approximated — renders, but not 1:1 with the other backend |
| ❌ | Not implemented |
| — | Not applicable |

Metal is the reference backend (developed and run on the author's Mac), so its
column is ✅ for shipping features unless noted. Vulkan status reflects the code
in `src/renderer/vulkan/` and what has been confirmed on the Windows/RTX 3060.

## Matrix

### Forward shading & materials
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| PBR (albedo/metallic/roughness/emission) | ✅ | ✅ | Cook-Torrance GGX; verified rendering `arena.json`. |
| Texture maps: albedo / MR / AO / emissive | ✅ | ✅ | glTF MR convention. |
| Normal mapping (TBN) | ✅ | ✅ | Vulkan `mesh.vert` passes world tangent; `mesh.frag` builds the TBN and perturbs N on texFlags bit 2 (ported from `lighting.metal`). |
| Per-vertex tint | ✅ | ✅ | |
| Analytic procedural surface library | ✅ | ⚠️ | 2026-08-23: the Vulkan port had drifted — `surfTerrain` (13), `surfWater` (12) and the whole `applySurfaceRelief` layer (road/terrain/water normal+roughness) were missing from `mesh.frag`, and the road tilt still used the pre-c5fe504 0.6. All ported to current Metal; water albedo+waves device-verified (RTX 3080), terrain grain/relief and the 0.35 road tilt await a Metal-side A/B (same-pose `shot` captures both backends now). Originally ported byte-for-byte from the Metal library, which since 2026-07-29 is one module per material: `surfaces_facade.metal` (the ten tiling patterns) + `surface_{road,water,terrain}.metal` (each holding that material's albedo **and** its normal/roughness relief) + `surfaces.metal` (the `applySurface` / `applySurfaceRelief` dispatchers). The primitives they build on (`hash21`/`vnoise2`/`fbm2`/`tile1`/`surfUV`) stay in `common.metal`. |
| Directional / point / spot lights | ✅ | ✅ | ADR-0017 physical units. |
| Fog | ✅ | ✅ | Aerial-perspective fog in `mesh.frag` (1-exp(-density·dist) toward fog color), params via the globals UBO `fog` field. Ported from `lighting.metal`. |
| Transparency / alpha blending | ✅ | 🟡 | Material `opacity < 1` routes to a blended pipeline (src-alpha/one-minus, no depth write, normal G-buffer masked), drawn back-to-front after opaque. Unverified on device. |
| Alpha-tested foliage + depth prepass | ✅ | ✅ | `mesh.frag` discards under the albedo alpha mask (FLAG_ALPHA_TEST) — the visual cutout. The depth-prepass overdraw optimization (perf, not visual) is still a follow-up. |

### Shadows
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| Cascaded shadow maps + PCF | ✅ | ✅ | Device-verified (Phase 3). |
| Shadow strength | ✅ | ✅ | |
| Shadow artistic tint / ambientStrength | ✅ | 🟡 | Occluded direct + ambient lerp toward `shadowTint` (globals); `ambientStrength` darkens IBL separately. Ported from `lighting.metal`. |

### Environment & IBL
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| Procedural sky + day/night + FBM clouds | ✅ | ✅ | `sky.frag`; part of the verified arena render. |
| HDR equirectangular environment | ✅ | 🟡 | Vulkan samples the equirect directly (`uploadTextureHDR`); unverified. |
| IBL: irradiance + GGX-prefilter + BRDF LUT | ✅ | ⚠️ | **BRDF LUT done** (🟡, baked split-sum at set 0 binding 3, used by `mesh.frag`). Irradiance + GGX-prefilter still use the analytic sky/equirect approximation — the baked prefiltered + irradiance cubes are the next IBL stage. |
| Reflection probes (parallax) | ✅ | ❌ | `setReflectionProbes` is a no-op in Vulkan. **Parity gap.** |
| Planetary atmosphere (single scattering) | 🟡 | 🟡 | Fullscreen Rayleigh+Mie raymarch (procedural-planet-plan P3), a port of the tested CPU reference `engine/procgen/atmosphere.cpp`. Shaders authored for all three backends: `shaders/vulkan/atmosphere.frag` (**SPIR-V compile-verified**), `shaders/metal/atmosphere.metal`, `shaders/webgpu/atmosphere.wgsl` (**naga-validated WGSL**; Metal compile-unverified). **Pipeline wiring done on Metal AND WebGPU** as an ADDITIVE glow (loadOp Load, blend One+One): the raymarch runs after the scene / before post so the limb halo blooms, driven by `setAtmosphere(AtmosphereRenderParams)` from the level's top-level `atmosphere` block; the WGSL variant drops the scene fetch / view-transmittance term (the blend composites). Metal: `metal_renderer.mm endFrame`. WebGPU: `webgpu_renderer.cpp recordAtmosphere` (couldn't compile on-device — the emdawnwebgpu port host is blocked by egress policy). **Vulkan still shader-only** (full-composite variant, no wiring). No device run yet on any backend. Multiple scattering (Hillaire) is the follow-up. |
| Volumetric clouds (slab + shell) | 🟡 | 🟡 | Fullscreen raymarched clouds (procedural-planet-plan P4) with a SHARED density/lighting/march core over two domains (`layer.w`): **slab** for a ground-scene cloudscape (the procgen city/terrain sky, depth-occluded) and **shell** for a planet's cloud deck. Supersedes the flat 2D FBM `sky.frag` clouds for both. `shaders/vulkan/clouds.frag` (**SPIR-V compile-verified**), `shaders/metal/clouds.metal`, `shaders/webgpu/clouds.wgsl` (**naga-validated WGSL**). Inline hash-fbm density (a 3D Perlin-Worley texture set is the on-device upgrade). **Pipeline wiring not done on any backend** (CloudUniforms + scene color/depth bound, pass after the scene). No device run yet. |
| Water surface (glint / Fresnel / depth-tint) | 🟡 | 🟡 | A water SURFACE shader (procedural-planet-plan P2): animated ripple normals, GGX sun glint, Schlick-Fresnel sky reflection, depth-tinted transmission + shoreline foam. Reads the existing `Surface::Water` UV convention (uv.x depth, uv.y shore), so it serves a **planet ocean AND city/coast water**. Writes the same MRT as `mesh.frag`. `shaders/vulkan/water.{vert,frag}` (**SPIR-V compile-verified**), `shaders/metal/water.metal`, `shaders/webgpu/water.wgsl` (**naga-validated WGSL**). **Pipeline wiring not done** (route Surface::Water draws to it; a `Draw` UBO with deep-colour + roughness). Supersedes/augments the analytic `surfWater` today. |
| Star / sun disc (blackbody + limb darkening) | 🟡 | 🟡 | Fullscreen star pass (procedural-planet-plan P6): a bright HDR disc with limb darkening + a Planckian blackbody colour from temperature, plus a bloom-friendly glow, gated on the reverse-Z far plane. `shaders/vulkan/star.frag` (**SPIR-V compile-verified**), `shaders/metal/star.metal`, `shaders/webgpu/star.wgsl` (**naga-validated WGSL**). **Pipeline wiring not done** (StarUniforms + scene color/depth bound). No device run. |

### Screen-space post
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| Bloom | ✅ | ✅ | Device-verified. |
| SSAO | ✅ | ✅ | Base verified; the box-blur denoise (`ssao_blur.frag`) + IGN rotation are 🟡 (recent). |
| SSAO temporal reprojection | ✅ | ❌ | Vulkan is spatial-only (blur). Needs motion vectors + history. Quality refinement, not a hard gap. |
| SSR | ✅ | 🟡 | Base verified; the binary-search refinement is recent/unverified. |
| Tonemap (ACES / AgX) + grade | ✅ | ✅ | Device-verified. |
| Lens (distortion / CA / vignette) | ✅ | ✅ | Folded into composite; device-verified. |
| Depth of field | ✅ | 🟡 | `dof.frag` golden-angle gather; written blind, unverified. |

### Geometry paths
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| Instanced rendering (correctness) | ✅ | ⚠️ | Vulkan renders via the CPU `drawMesh` fallback — visually correct but no GPU batching (perf, not a visual gap). |
| Vegetation wind sway | ✅ | ✅ | `mesh.vert` applies the FLAG_WIND sway (height-weighted, phase-offset; same constants as Metal) from the model base, so the per-instance fallback sways. |
| CDLOD terrain morph | ✅ | ✅ | `drawTerrain` override + `terrain.vert` morph each vertex toward its coarser-LOD position (tangent slot) over the [start,end] camera-distance band. Device-verified (stable after the reverse-Z fix). |
| Mipmaps | ✅ | ✅ | `createImageRGBA8` generates the full chain via a blit downsample; sampler already mip-aware (LINEAR, unclamped LOD). |
| Back-face culling + FLAG_TWO_SIDED | ✅ | ✅ | 2026-08-23: Vulkan rendered EVERYTHING two-sided (the "Phase 1: cull nothing" deferral) — harmless until the flat-facade middle LOD (823986c) shipped coplanar front/back geometry that self-z-fights when both faces rasterize: buildings/vehicles dissolved view-dependently at distance on metro-scale levels (the "disappearing geometry" report). Metal never saw it (culls Back per batch, FLAG_TWO_SIDED opts out). Vulkan now mirrors that via a back-culled opaque pipeline variant (API 1.0, so a variant rather than dynamic cull state); terrain culls Back unconditionally like Metal's encoder default. Device-verified A/B at the reporter's exact camera pose. |

### Debug & tooling
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| Debug views (AO/SSR/depth/normals/shadow/albedo/facing/cascades) | ✅ | 🟡 | Full set implemented; albedo/depth fixes recent, unverified (2026-08-23: albedo view showed near-camera CDLOD terrain black on device — suspect, see log). |
| Headless frame capture (`shot` / RT_FRAME_DUMP) | ✅ | ✅ | Vulkan `requestFrameDump` lands 2026-08-23: swapchain built with TRANSFER_SRC, armed frame copies the composited image to a host buffer, fence + `stbi_write_png` — same two arms and semantics as Metal. Device-verified (RTX 3080). This was the missing parity *instrument*: without it only Metal could be screenshot-compared. |
| Wireframe (modes 1 & 2) | ✅ | ✅ | LINE-mode pipeline; device-verified. |
| Dear ImGui overlay | ✅ | ✅ | Vulkan backend (1.92 API) device-verified. |
| Editor viewport (hosted window) | ✅ | ✅ | Vulkan surface from the Qt viewport; device-verified on Windows. |
| Gamepad | ✅ | ✅ | Standalone viewer uses GLFW's joystick path off-Apple; the hosted editor's GCController poll is macOS-only. |

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
  the 1st-pass entry — sync-val clean).
- **Pending device check (Vulkan):** transparency; shadow tint; HDR equirect IBL (4b) + BRDF
  LUT; debug views (albedo/depth); depth of field (with a low f-stop); SSR
  binary-search refinement.

## Known non-goals (both backends)
Per ADR-0057: unifying the viewer with the offline path tracer, compute shaders,
and hardware ray tracing are out of scope and not tracked here.
