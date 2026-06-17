# Open-world foundations — a bounded, curated world: depth, partitioning, LOD & streaming

**Status:** Design plan (plan-first; no rendering code yet). **Target chosen:** a
**bounded, artist-curated "place"** — *not* an infinite Minecraft generator. A large
chunk of terrain ~**16 km across** with forests, rivers, a city, and walk-to-able
distant mountains, with room to get crazier later. The model is **GTA V / Horizon
Forbidden West**: finite, streamed, curated; single precision. Pairs with ADR-0027 /
`world-system-plan.md` (the *content* model — fields + recipes, authored as data,
per-tile overrides). This doc owns the *coordinate, rendering, and LOD/streaming*
foundation. Decisions: ADR-0034.

---

## 0. Why now — the symptoms that forced this

Distant-terrain work surfaced failures that are all the same assumption — *the
engine assumes a small world centred on the origin*:

- **Terrain composited as sky.** The composite/SSR/SSAO classify a pixel as
  background with a fixed `depth >= 0.999`. With `near=0.1` that NDC value maps to
  only ~99 m, so all terrain past ~99 m is painted over with the skybox (the
  "circular cut" — the 99 m ground disc). Hyperbolic depth + tiny near plane.
- **Frustum culling misjudges terrain.** The LOD terrain is one giant origin-centred
  mesh (plus concentric rings); its bounding sphere is a poor proxy, so the
  point-sphere test rejects/keeps it wrongly as the camera moves.
- **LOD ring seams crack** (already in TECH_DEBT) — concentric rings aren't a real
  streaming/LOD scheme.

These are not independent bugs; they're the small-world assumption leaking. The fix
is a coherent foundation, sized to the *actual* target.

## 1. Precision: a bounded world fits single precision (so precision is *not* the problem)

A 32-bit float carries ~7 significant digits, so absolute precision ≈ `distance /
2^23`. For a world **centred on the origin**:

| World size (centred) | float precision at the edge |
|---|---|
| 16 km across (±8 km) | ~1–2 mm |
| ~130 km across (±64 km) | ~1.5 cm |
| ~500 km across | ~6 cm |

At our chosen **~16 km, precision is ~1–2 mm everywhere** — better than we need for
rendering or physics. This is the crucial consequence of picking a *bounded* target:
**the whole infinite-world precision apparatus is unnecessary.** We do **not** need
camera-relative rendering, floating-origin rebasing, or double coordinates on the
GPU. (Our CPU side is already `Real=double`, so the authoritative simulation has even
more headroom; we only ever narrow to float at upload, on small numbers.)

The real problems are therefore **depth precision/range**, **spatial partitioning**,
and **LOD** — not coordinate precision.

> **Rule: never hardcode the world extent.** Size lives in one constant (far plane +
> streaming-grid extent). Growing 16 → 64–130 km later (still single precision,
> ~cm) must be a one-line change, not a refactor.

## 2. What GTA V / Horizon actually do (the model we're copying)

- **Finite, fits single precision.** GTA V is ~8 km across; Horizon a few tens of km.
  No floating origin, no camera-relative — the map is just small enough.
- **Content is authored, not regenerated.** Artists place the city, sculpt valleys,
  route rivers; procgen *assists* (terrain base, scatter, vegetation). The result is
  a baked, curated *place* — matches ADR-0027 (fields + recipes + per-tile overrides,
  authored as data).
- **Streaming by sectors/cells.** A grid of streaming sectors; assets page in/out by
  distance. Nothing regenerates — it loads.
- **Aggressive LOD + HLOD + impostors** (§5) and **occlusion culling** (§3, later) —
  what keeps draw calls (and the Mac) sane.

We are not building Minecraft. Procgen is a *tool to build the bounded place*, not an
endless generator.

## 3. Reverse-Z depth + robust sky, and spatial partitioning (the near-term core)

### 3.1 Reverse-Z + robust background (Phase 0)
The current break and the latent far-field z-fighting are one fix:
- **Reverse-Z.** Map near→1, far→0 on the `Depth32Float` buffer we already use.
  Float concentrates precision near 0; reverse-Z puts the *far* field there, giving
  near-uniform precision from 0.1 m out to a **16–20 km far plane** — without the
  collapse that caused the 99 m cut. Touches: `Mat4::perspective` (reverse mapping),
  depth clear (0 not 1), depth compare (`Greater`/`GreaterEqual`), skybox z, and the
  depth-reconstruction math in SSR/SSAO.
- **Robust sky test.** Replace every `depth >= 0.999` with a test against the
  *cleared* far value: under reverse-Z, background = `depth <= 0`, exact and
  scene-independent. Stop-gap until reverse-Z lands: a *linearized* test
  (`linearDepth >= 0.999*far`), correct for any near/far ratio.

### 3.2 Spatial partitioning (Phase 1)
Replace the single origin-centred bounding sphere with a **sector grid** (and/or
BVH/octree) over the world. It drives three things: correct **frustum culling** (each
chunk/object tested by its own tight bounds), **streaming** decisions (which sectors
are near the camera), and later **occlusion culling** (don't draw the blocks behind a
building). **Re-enable per-object frustum culling** here, once chunk bounds are tight
(it's currently bypassed by a reverted diagnostic).

## 4. Terrain: chunked, tight bounds, geometric LOD (Phase 1)

Replace the single origin-centred tile + concentric rings with a **grid/quadtree of
terrain chunks**, each:
- generated deterministically from `(tileCoord, worldSeed)` (ADR-0027 §5), authored-
  over by per-sector overrides, cached, freed when far;
- a *small* mesh with its **own tight AABB** (so culling is correct — kills the
  origin-sphere misjudgement);
- carrying its own **geometric LOD**; neighbours differ by ≤1 level, stitched with
  skirts/edge-stitching (retires the "rings crack at seams" debt).

Candidate schemes (decide in a later ADR): **geometry clipmaps** (camera-centred
concentric grids, GPU-friendly) or **CDLOD / chunked quadtree** (per-chunk LOD with
morphing). **A walkable distant mountain is just coarse terrain LOD** here — terrain
is a continuous surface you stand on, refined as you approach. It is *not* an
impostor (those are for discrete props, §5). At 16 km, a mountain 8–15 km away already
reads as a hazy background silhouette and is walk-to-able within the map.

## 5. Object LOD: discrete meshes → impostors → HLOD (Phases 2–3)

The ladder that keeps draw times low as the world fills with trees and buildings:

- **Discrete mesh LOD (Phase 2).** Each prop has a few LOD meshes selected by
  distance, crossfaded/dithered to hide pops. Cheapest first win; pairs with the
  existing instanced scatter path.
- **Impostors / billboards (Phase 2, the high-value foliage step).** A flat quad
  painted with a *pre-rendered image* of the object, facing the camera — 2 triangles
  + 1 texture instead of thousands. *Simple billboard*: one texture, for radially-
  symmetric things (distant trees, bushes). *Octahedral/multi-view impostor*: bake the
  object from many angles, sample/blend at runtime → looks 3D from any direction.
  **Forests want this**: near = instanced meshes, mid = low-poly LOD, far = impostor
  cards (often a whole tree cluster baked into one card) → a forest to the horizon
  without melting the GPU.
- **HLOD — Hierarchical LOD (Phase 3, for the city).** LOD for *groups*: bake a city
  block (50 buildings + props) offline into **one merged, simplified mesh + atlas**;
  draw that single proxy far away (one draw call), swap back to individual buildings
  (with their own LODs) up close. Nests: district → block → building → full detail;
  the coarsest HLOD level can itself be impostor cards.

**Impostor vs HLOD:** an impostor is a flat card with a baked picture; HLOD is a real
(simplified, merged) 3D mesh for a cluster. Cities use both. Both need an **offline
bake step** (render-to-card / merge+decimate) — a real piece of the content pipeline.

## 6. Sector streaming (Phase 3)

Page authored/generated content in and out by distance over the §3.2 partition grid —
the ADR-0027 §7 streaming manager, but over a **finite** sector set, not an unbounded
tile map. Terrain chunks, instance groups, and (later) HLOD proxies stream as the
camera moves; sectors carry their per-sector overrides (ADR-0027).

## 7. Skybox & atmosphere

Keep the skybox — it *is* the sky. The "let it render as-is" instinct was really three
needs it masked, each handled above: **robust sky/geometry classification** (§3.1),
**terrain that reaches the horizon** (§4), and **atmospheric blend** of distant
terrain into the sky (the aerial-perspective fog already added).

## 8. The "walk around the world" dream (future — out of scope)

Walking around the planet and returning is a **topology change** (the world *wraps* —
a torus or, for a real planet, a sphere), orthogonal to everything above. It needs a
sphere/torus coordinate scheme, cube-sphere terrain, real atmospheric scattering, and
— because a planet is huge — the *infinite-world* precision apparatus this plan
deliberately skips (camera-relative rendering + floating-origin rebasing, possibly
64-bit coords). Recorded as a future ADR; the bounded-world work is a clean
foundation for it either way.

## 9. Phasing

| Phase | Scope | Unblocks | Risk |
|---|---|---|---|
| **0** | Reverse-Z + robust sky test; re-enable culling; (diagnostics already reverted) — *implemented; projection math unit-tested; shader half pending macOS-viewer verification* | The current broken render; far-field z-fighting | Small, Metal-only (user-verified) |
| **1** | Spatial partition (sector grid/BVH) + chunked terrain w/ tight bounds + terrain LOD | Correct culling; distant terrain/mountains; retires ring/seam debt | Medium–large |
| **2** | Object mesh LOD + **foliage impostors** | A forest to the horizon at frame rate | Medium; needs an impostor bake |
| **3** | Sector streaming + **HLOD/building impostors** | The curated city; the full 16 km place | Large; content-pipeline work |
| later | Occlusion culling | Dense city perf | Medium |

Phase 0 is the smallest change that makes `range.json` render again and is worth doing
regardless. Phases 1–3 build the bounded place. The infinite/planetary apparatus
(camera-relative, floating origin) is **not** on this path — it's §8's future.

## 10. Open questions (resolve as phases start)

- **Clipmaps vs CDLOD** for terrain LOD (Phase 1 ADR).
- **Impostor scheme** — simple billboard vs octahedral; per-tree vs per-cluster bake;
  where the bake step lives in the pipeline (Phase 2 ADR).
- **Reverse-Z blast radius** — enumerate every depth consumer (SSR, SSAO, temporal AO,
  debug views) before flipping; they must change together (Phase 0).
- **World-size constant** — confirm 16 km holds the intended content; the rule in §1
  keeps it cheap to grow.
- **Offline tracer parity** — the tracer renders in absolute world space with no far
  clip, so it's the correctness oracle; reverse-Z is realtime-only and validated
  against it.

## 11. Decisions to record

- **ADR-0034** (this plan): *bounded ~16 km single-precision world; reverse-Z; spatial
  partitioning; chunked terrain + object LOD (impostors/HLOD); sector streaming.*
  Status **Pending** until Phase 0 lands.
- A later ADR fixes the terrain LOD scheme (clipmaps vs CDLOD) and the impostor bake.
