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
| Normal mapping (TBN) | ✅ | 🟡 | Vulkan `mesh.vert` passes world tangent; `mesh.frag` builds the TBN and perturbs N on texFlags bit 2 (ported from `lighting.metal`). Unverified on device. |
| Per-vertex tint | ✅ | ✅ | |
| Analytic procedural surface library | ✅ | ✅ | Ported byte-for-byte from `common.metal`. |
| Directional / point / spot lights | ✅ | ✅ | ADR-0017 physical units. |
| Fog | ✅ | 🟡 | Aerial-perspective fog in `mesh.frag` (1-exp(-density·dist) toward fog color), params via the globals UBO `fog` field. Ported from `lighting.metal`. Unverified on device. |
| Transparency / alpha blending | ✅ | 🟡 | Material `opacity < 1` routes to a blended pipeline (src-alpha/one-minus, no depth write, normal G-buffer masked), drawn back-to-front after opaque. Unverified on device. |
| Alpha-tested foliage + depth prepass | ✅ | 🟡 | `mesh.frag` discards under the albedo alpha mask (FLAG_ALPHA_TEST) — the visual cutout. The depth-prepass overdraw optimization (perf, not visual) is still a follow-up. Unverified on device. |

### Shadows
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| Cascaded shadow maps + PCF | ✅ | ✅ | Device-verified (Phase 3). |
| Shadow strength | ✅ | ✅ | |
| Shadow artistic tint / ambientStrength | ✅ | 🟡 | Occluded direct + ambient lerp toward `shadowTint` (globals); `ambientStrength` darkens IBL separately. Ported from `lighting.metal`. Unverified on device. |

### Environment & IBL
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| Procedural sky + day/night + FBM clouds | ✅ | ✅ | `sky.frag`; part of the verified arena render. |
| HDR equirectangular environment | ✅ | 🟡 | Vulkan samples the equirect directly (`uploadTextureHDR`); unverified. |
| IBL: irradiance + GGX-prefilter + BRDF LUT | ✅ | ⚠️ | Vulkan uses an **analytic procedural-sky approximation**, not a baked prefiltered cube + LUT. **Parity gap** for accurate HDR-lit ambient/specular. |
| Reflection probes (parallax) | ✅ | ❌ | `setReflectionProbes` is a no-op in Vulkan. **Parity gap.** |

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
| Vegetation wind sway | ✅ | 🟡 | `mesh.vert` applies the FLAG_WIND sway (height-weighted, phase-offset; same constants as Metal) from the model base, so the per-instance fallback sways. Unverified on device. |
| CDLOD terrain morph | ✅ | 🟡 | `drawTerrain` override + `terrain.vert` morph each vertex toward its coarser-LOD position (tangent slot) over the [start,end] camera-distance band. Unverified on device. |
| Mipmaps | ✅ | 🟡 | `createImageRGBA8` generates the full chain via a blit downsample; sampler already mip-aware (LINEAR, unclamped LOD). Unverified on device. |

### Debug & tooling
| Feature | Metal | Vulkan | Notes |
| --- | --- | --- | --- |
| Debug views (AO/SSR/depth/normals/shadow/albedo/facing/cascades) | ✅ | 🟡 | Full set implemented; albedo/depth fixes recent, unverified. |
| Wireframe (modes 1 & 2) | ✅ | 🟡 | LINE-mode pipeline; written blind, unverified. |
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
- **Pending device check (Vulkan):** HDR equirect IBL (4b); debug views
  (albedo/depth fixes); wireframe; depth of field; SSAO box-blur + IGN rotation;
  SSR binary-search refinement; normal mapping (TBN); aerial-perspective fog;
  mipmaps; transparency / alpha blending; **shadow tint; vegetation wind;
  terrain morph**.

## Known non-goals (both backends)
Per ADR-0057: unifying the viewer with the offline path tracer, compute shaders,
and hardware ray tracing are out of scope and not tracked here.
