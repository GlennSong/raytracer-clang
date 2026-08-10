# Piedmont at 2 fps — diagnosis and plan

Status: investigation (headless, July 2026). Numbers below are measured by
running the real generators outside the viewer (`/tmp/probe_pm`, built from the
Lot Lab's link set); frame-time attribution needs the on-device tools that just
landed on main (frame ledger, pass-cost bisect).

## What is actually there to draw

Measured from the shipping pipeline, `piedmont_mini.json`:

| Content | Meshes | Triangles | Culling today |
|---|---|---|---|
| Roads (`buildRoadNetMesh`) | **1 per road entity** | 22 K (mini) | **none that works** — one AABB spans the whole city, so the frustum test never rejects it and every street draws every frame |
| Buildings (`growLotBuildingsOnNets`) | 19 part-meshes → chunked at 250 m | **697 K (mini, 132 lots)** | AABB frustum + `drawDistance` 900 m, HLOD mass boxes past that |
| Full `piedmont.json` | — | generation alone runs **minutes**, headless | — |

Two structural facts fall out:

1. **Buildings dominate the triangle budget and the detail radius is enormous.**
   `detailDistance: 900` means full-geometry facades — every window reveal,
   frame, muntin and cornice — for everything within 900 m. Mini's 132 lots are
   697 K triangles; Piedmont proper is thousands of lots. The dial is at 900
   because the only thing past it is a flat-shaded mass box, and mass boxes up
   close look terrible. **The missing middle LOD is why the detail radius is
   inflated, and the inflated radius is why the GPU drowns.**
2. **The road network is a monolith.** One `Renderable`, one mesh, one draw,
   whose bounds are the whole city. It can never be frustum-culled, never
   distance-culled, and has no LOD. Same pattern for `road_walls` and `water`.

Also relevant:

* **The haze is off in practice.** Piedmont ships fog density 0.0004 — 30%
  haze at 900 m, 50% at 1.7 km. The levels where fog *hides* the far field use
  0.0025–0.005 (89–99% at 900 m). Nothing atmospheric is covering the LOD swap,
  which is why the swap has to be pushed out to 900 m to be unobjectionable.
* **The post stack is under active investigation on main** (bloom
  double-added bug fixed; frame ledger + pass-cost bisect landed). That work
  attributes the *other* half of the frame; this doc is about the geometry half.
* **The citysim step is a third, separate cost** — out of scope here.

## Why the city feels like one object

The building chunks, HLOD boxes, lamp/signal instances and road meshes are all
anonymous runtime entities — no `Name`, no hierarchy, no `SourceSpec` (by
design, ADR-0022 runtime tier). The editor shows one `streets` entity and an
opaque cloud. There is no *City → block → lot* structure to select, inspect,
hide, or cull against, even though the generator knows all of it
(`NetLotResult.plan` carries every block and lot polygon already).

## The plan

Ordered by measured leverage; each item is independently landable.

### R1 — chunk the road mesh (small, headless, do first)

`buildRoadNetMesh` output goes through the same `chunkMeshByCell` the buildings
already use: one Renderable per 250 m cell, each with a real AABB. Frustum
culling starts working on roads; distance policy becomes possible. No new
tech — reusing the existing chunker on one more producer. Same treatment for
`road_walls`.

### R2 — a real middle LOD for buildings (the big win)

Three tiers instead of two, so the detail radius can drop:

| Tier | Range | What it is |
|---|---|---|
| LOD0 full grammar | 0–~300 m | today's facades |
| **LOD1 flat facade** | ~300–900 m | walls as single quads per plan edge, windows painted into a per-building **texture** (or the window-grid shader the surface library already supports), roof slab + parapet only. No reveals, frames, muntins, trim, balconies. ~50–100 triangles per building instead of ~5 000 |
| LOD2 mass box | 900 m+ | today's `appendLotMassBox` |

The blueprint work makes LOD1 nearly free to generate: the opening list *is*
the data a facade texture needs — spans along the wall × sill/head heights —
without touching 3-D geometry. LOD1 is the first consumer of the plan-owns-
openings model, and the reason it can look *good* rather than approximate.

With LOD1 in place, `detailDistance` drops to ~300 and the LOD0 triangle load
falls roughly with the square of the radius — order of magnitude.

### R3 — turn the haze back on, value-matched

Raise fog toward the 0.002–0.003 band on city levels and tint the LOD2 mass
boxes toward the fog colour at distance (the loader already value-matches their
albedo; extend that to the fog term). Haze is the cheapest LOD blender there
is, and it is currently doing nothing.

### R4 — bake cache: generate once, load fast

Everything is regenerated on every load; full Piedmont takes minutes before
the GPU sees a byte. The fix is the ADR-0022 "baked static asset" tier applied
to the whole generated city:

* Key = hash(level JSON + generator code version + seed).
* Value = the loader's post-generation products, serialized: chunked meshes,
  colliders, instance tables, HLOD boxes, nav graph, flatten regions.
* First load generates and writes the cache (background thread); subsequent
  loads stream it straight to GPU upload. Editing a road in the editor
  invalidates only that entity's entries.
* This also becomes the natural home for **offline LOD1 baking** (facade
  textures want to be baked, not built per frame) and later the impostor
  atlas (ADR-0038 §6 designed it; still unbuilt).

Generation itself should also go wide — the per-lot grammar work is
embarrassingly parallel and single-threaded today (`JobSystem` exists).

### R5 — break the monolith into a real hierarchy

Promote the city's structure to named, nested entities: `City → Block[n] →
Lot[m]` + `Roads`, each block owning its chunks' entities as children. This is
the ADR-0022 "editable procedural instance" tier the register already defers —
it makes the editor navigable, gives culling a natural spatial grouping, lets
a block be hidden/soloed for debugging, and is the seam the Lot Lab's Qt panel
(plan §17.9) plugs into. Costs entity count; the chunk granularity already
exists, so this is naming and parenting, not re-meshing.

## How to verify on device

1. `RT_DUMP_DRAWS=1` (render_system.cpp) — the one-shot draw audit: confirms
   what is actually submitted per frame before/after each step.
2. The new frame ledger + pass-cost bisect from main — attributes post-stack
   vs geometry, and confirms R1/R2 moved the needle.

## Sequencing against the lot plan

R1 and R4 touch nothing the lot plan changes — land any time. R2's LOD1
consumes the blueprint/opening model, which argues for pulling that slice of
P2 forward. R5 is the same entity-promotion the Qt lab needs. The two tracks
converge rather than compete.
