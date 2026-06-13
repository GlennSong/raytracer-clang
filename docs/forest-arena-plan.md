# "The Forest" Arena — Plan (Tier 4 Phase B milestone)

A concrete visual target that proves the procgen substrate (ADR-0021) end to
end. One scene, built entirely by generators:

- a **procedural heightfield terrain** (noise/FBM/domain-warp),
- **slope/altitude-driven material** on it (grass on flats, rock on cliffs),
- **trees + foliage** from an **L-system** and **rocks** from noise-displaced
  primitives, generated into the **asset manager** as a few reusable "species"
  meshes,
- **scattered by the thousands** with sensible density (no trees on cliffs or
  above a treeline; clumping via a noise mask), drawn with **instanced
  rendering**,
- under an **HDR sky** (reuse the existing IBL + day/night).

It exercises every value type at once — `Mesh`, `Field`, `Material`, `Frame` —
which is exactly the cross-paradigm test ADR-0021 wants before any procgen
"language". It is also a believable game/tech-demo arena, not a contrived
benchmark.

## What it forces us to build

| Capability | New? | Maps to |
|---|---|---|
| Own/share/free generated meshes | **3.1** | AssetManager (`docs/asset-system-plan.md`) |
| Procgen-grade mesh assembly | **3.3** | MeshBuilder (merge/transform/normals/UV) |
| Noise (FBM, domain warp) | **3.7** | noise library |
| Engine-side PBR material + slope/height blend | **3.2** | material system |
| Heightfield terrain generator | Phase B.2 | noise → heightfield → `Mesh` |
| L-system trees / foliage | Phase B.1 | grammar → `Mesh` |
| **Scatter / distribution** | **new — Phase B.4** | the `Frame` generator |
| **Instanced rendering** | **new — pulled fwd** | `InstanceGroup` + `drawMeshInstanced` |
| HDR sky / lighting | exists | ADR-0016/0017 + day-night |
| Walkable terrain collision | optional first | Jolt `HeightFieldShape` (defer: fly-cam first) |

The two genuinely new systems are **Scatter** and **instanced rendering** — the
rest is the already-planned substrate.

## The key design decision: how instances live in the ECS

Two ways to represent 5,000 scattered trees:

- **A — 5,000 entities**, each `Transform` + `Renderable` sharing one
  `MeshHandle`; the render system buckets by handle into one instanced draw.
  Uniform ECS, each tree individually selectable/physical — but 5,000 entities'
  worth of per-frame iteration and interpolation for static scenery.
- **B — one `InstanceGroup` component** = `{ MeshHandle, std::vector<Mat4>
  instances, optional per-instance data }`, drawn as a single instanced call.
  Scales to 100k cheaply; the natural home for the scatter result. Instances
  aren't individual entities (no per-instance physics/picking without extra
  work).

**Recommendation: B (`InstanceGroup`) for static scatter** — it's what "a forest
of thousands" actually wants, and it's the direct consumer of the scatter
generator's transform set. Keep **A (entities + handle-bucketing)** available for
*dynamic, individually-simulated* fleets (the "1,000 spaceships" case). Both sit
on the same `drawMeshInstanced` renderer path; they differ only in where the
instance transforms come from (a component vs. a per-frame bucket of entities).
**This is the one thing worth confirming before building.**

## Instanced render path

1. **Renderer seam:** `drawMeshInstanced(MeshHandle, span<InstanceData>)`, where
   `InstanceData` is at least a model matrix (+ later a tint/material-override
   for per-instance variation). The stats already reserve `instancedDrawCalls`
   / `totalInstances`.
2. **RenderSystem:** for each `InstanceGroup` (and/or each `MeshHandle` bucket of
   instanced entities), build the per-instance buffer — culled CPU-side (group
   bounds first; per-instance / chunked culling is the Tier 5 scaling follow-up)
   — and issue one instanced draw.
3. **Metal backend:** a per-instance structured buffer the vertex shader indexes
   by `instance_id`; `drawIndexedPrimitives:…instanceCount:N`.

## Scatter / distribution (Phase B.4)

The `Frame` generator: `scatter(surface, rules, seed) -> std::vector<Transform>`.

- Samples placements over the terrain (jittered grid or Poisson-disk) and reads
  the terrain at each: **height, slope, and a density `Field`** (a noise mask) to
  decide keep/drop and which species. Random yaw + scale jitter per instance.
- Rules: max slope (no trees on cliffs), altitude band (treeline / waterline),
  density falloff, min-spacing. "In ways that make sense" = these rules over the
  terrain field.
- Seeded (ADR-0021/0002) so the forest is reproducible.
- Output feeds an `InstanceGroup` per species. This is where terrain (Field),
  asset meshes (Mesh), and instancing meet — the integration knot.

## Build order (critical path to first Forest)

1. **3.1 AssetManager** (keystone; also fixes the mesh leak).
2. **3.3 MeshBuilder** (procgen-grade) + **3.7 Noise**.
3. **Instanced rendering** — `InstanceGroup` + `drawMeshInstanced` + RenderSystem
   batching.
4. **B.2 Terrain** — noise heightfield → `Mesh`; **3.2 material** with a
   slope/height blend (full procedural *textures* can come later).
5. **B.1 L-system** trees/foliage + **noise-displaced rock** meshes → asset
   manager. (SDF rocks (A.1) are a nicer later upgrade — not on the first path.)
6. **B.4 Scatter** → `InstanceGroup`s.
7. **Assemble "The Forest"** arena (a level/scene that wires the above) + HDR sky.

Terrain physics (walkable ground via Jolt `HeightFieldShape`) is deferred behind
a fly-camera first look — add it when "walk the forest" matters.

## Linux / macOS boundary

Most of this is **headless, CI-testable** — the whole *generation* pipeline is
pure data:
- noise, terrain mesh, L-system mesh, rock mesh, scatter transforms, AssetManager
  (stub renderer), and the RenderSystem instance-buffer **batching** logic.

**macOS-only:** the actual `drawMeshInstanced` GPU path + instanced shader, the
slope/height material shader, and — of course — looking at the result.

So we can build and unit-test the entire forest *generator* on Linux and hand the
Mac just the render verification.

## Persistence — store the recipe, not the result

Generation is deterministic `(params, seed) → content` (ADR-0021/0002), so the
level JSON stores the **seed + parameters**, and the engine reconstructs terrain,
meshes, and scatter at load. We do **not** serialize tens of thousands of
transforms or megabytes of mesh — just the recipe:

```json
"procedural": {
  "seed": 1337,
  "terrain": { "size": 4096, "octaves": 6, "warp": 0.3 },
  "species": [ { "kind": "lsystem", "rules": "...", "lods": 3 } ],
  "scatter": { "maxSlope": 30, "treeline": 120, "density": "noise:..." }
}
```

Tiny files, vast worlds. It sits next to the hand-authored `entities` array (the
LevelWriter already preserves sections it doesn't own). Hand-editing a generated
instance later means a **"bake to entities"** or per-instance overrides (an editor
concern, deferred). For huge worlds, cooked meshes can be cached to a binary
asset cache (asset-system 3.1c) to regenerate less; first cut regenerates at
load. *This is ADR-worthy when built* ("procedural content persists as a recipe,
regenerated deterministically at load").

## Loading progress — reuse the job + progress pattern

World construction runs as `JobSystem` tasks (terrain, per-species mesh, scatter)
behind a **loading state** with a progress bar driven by an atomic the jobs
update — the same "progress on its own thread" pattern built for the offline
tracer. Parallel and reportable, so construction never feels like a frozen hang.

## Grass — a separate, specialized system (not the general scatter)

Trees/rocks are *thousands* of instances (CPU scatter → `InstanceGroup` is fine);
grass is *millions* of blades, which breaks that model. Grass is **GPU-placed,
camera-relative** (blade positions generated per near-camera tile, wind animated
in the vertex shader, distance-faded) — a dedicated grass system that reuses the
scatter density-`Field` for *placement rules* but renders its own way. Out of
scope for the first Forest; a follow-up.

## Scaling layers (additive) — and the seams that keep them additive

The first Forest is bounded; growing it to a *vast* forest is a sequence of
independent Tier-5 systems, none a rewrite **if** these seams exist from the
start:

| Seam (designed in the first cut) | Unlocks later, no rewrite |
|---|---|
| Asset = **LOD chain** (N `MeshHandle`s per logical asset) | mesh LOD, billboard impostors |
| Scatter → **per-tile `InstanceGroup`s** | uniform-grid frustum/distance culling, chunking, streaming |
| Level = **recipe block** (seed + params) | huge worlds in tiny files |
| Generation = **`JobSystem` tasks + progress** | loading bar, async, cooking cache |

Performance order of attack (the ~20fps tech-debt item applies even now):
instancing (drawcalls) → tile culling (offscreen work) → LOD/impostors/density
fade (distant work). All additive.

## Scope: first cut vs. later

- **First cut:** single terrain tile, a few species, thousands of instances,
  group-bounds culling, slope/height material blend, HDR sky, fly camera.
- **Later (Tier 5):** per-tile grid culling + LOD + impostors for huge counts,
  terrain chunking/streaming, hydraulic/thermal erosion, full procedural
  *texture* synthesis, SDF rocks, terrain collision (Jolt `HeightFieldShape`),
  the GPU grass system, wind/animation on foliage.

## Open questions

1. **Instance representation:** `InstanceGroup` component (recommended) vs. N
   entities — confirm before building the render path.
2. **Per-instance variation:** tint/scale only (cheap) vs. per-instance material
   override (needs 3.2 + a richer instance buffer). Start with transform + tint.
3. **Culling granularity for the first cut:** whole-group bounds (simplest) vs.
   tiling the scatter for chunk culling (better, but pulls in chunking early).
