# World system — fields, recipes, streaming (design plan)

How a world (terrain + biomes + scatter + water + later cities) is represented,
authored, and generated. Captures the direction agreed in the 2026-06-15 design
discussion. The firm, hard-to-reverse decisions are in **ADR-0027**; this doc
holds the detail and the exploratory parts.

Builds on: ADR-0021 (procgen substrate: `(params,seed)->content` over value
types), ADR-0022 (realness spectrum), ADR-0025 (Lua is the authoring path),
ADR-0026 (procedural objects are multi-output assets). Current code: heightfield
terrain (`procgen/terrain.cpp`), noise/slope scatter (`procgen/scatter.cpp`),
vegetation as instance groups (`level_loader.cpp`), `flora.lua` recipes.

## 1. The model: fields + recipes

**A world is a stack of *fields* plus a set of *recipes* that read them.**

- **Fields** — 2D rasters aligned to the terrain: `height`, `slope`/`aspect`
  (derived), `moisture`, `temperature`, `biome id`, and N **scatter-density
  masks** (forest, grass, rocks, …). Each field is either *procedural* (params)
  or a *painted raster asset*; the two mix freely.
- **Recipes** (Lua) — `(region, fields, seed) -> content`. A **forest** is a
  scatter recipe over a region, placing tree/underbrush/grass/rock species
  *weighted by the masks*. A **river** carves a channel + lays a water surface.
  A **city** suppresses natural scatter and runs building placement.

The world file = a terrain definition + a list of **field layers** + a list of
**recipe instances** (region shape + recipe ref + seed + param overrides) +
sparse **per-tile overrides** for human edits.

## 2. Fields

| Field | Source | Notes |
|---|---|---|
| height | procedural (noise) or painted | the terrain itself |
| slope / aspect | derived from height | drives rockiness, snow, buildability |
| moisture, temperature | procedural | inputs to biome classification |
| biome id | derived (height+slope+moisture) or painted | selects material + which recipe runs |
| scatter masks (forest, grass, rock, …) | procedural seed, then **brush-painted** | density weight per scatter type |

**Masks are raster layers, not vertex colors** — resolution-independent, sampled
by both the scatter system and terrain-material blending (splat). Procedurally
**seeded** (e.g. `forest = f(altitude, slope, moisture, noise)`) then
**paintable** with editor brushes. The mask is the shared artifact between the
generator and the artist.

## 3. Recipes (Lua)

All recipes share the ADR-0021 signature and produce value-type content. Kinds:

- **terrain** — `(tileCoord, seed) -> heightfield (+ material splat)`.
- **scatter** — `(region, fields, seed) -> placements` of species sub-recipes
  (each species is itself a recipe, e.g. `recipes/flora/oak.lua` per ADR-0026).
- **water** — `(spline|polygon, fields, seed) -> water surface (+ carve edits)`.
- **carve** — `(volume, seed) -> SDF edit` subtracted from terrain (see §6).
- **city / building** — future; a *split/shape grammar*, not a plant L-system
  (see §8).

```lua
-- recipes/scatter/temperate_forest.lua
return {
  name = "temperate_forest",
  params = {
    density = { type="float", min=0, max=1, default=0.6 },
    species = { type="list", default={"oak","birch","fern","rock_small"} },
  },
  scatter = function(region, fields, seed)
    -- weight = forestMask * (1 - steepness) * noise; emit species instances
    -- by sampling fields at candidate points within region.
  end,
}
```

## 4. Authoring is data, not code (editor + Lua)

The **editor authors data**; it never writes code. It edits:
- **regions** (footprint shapes / splines),
- **painted mask layers** (brushes → raster assets),
- **carve edits** (placed SDF volumes),
- **recipe parameters** and **seeds**.

All of that is **stowed as assets**; **Lua recipes are themselves assets**; the
world file references them. "Build by script" (edit the Lua recipe) and "build in
the editor" (paint masks / drop regions / tweak params) are the same path with
two front-ends — consistent with ADR-0025.

**What is an entity in the browser:**
- The **terrain** (one entity, though tiled underneath).
- Each **scatter region / biome volume** — one entity: footprint + recipe ref +
  its mask layers. Selectable, editable; expands to instance groups at runtime.
- **Water bodies** (spline/polygon) — entities.
- **Carve edits** — entities (a placed volume with an SDF op).
- **Painted masks / fields** — toggleable overlays you brush, not entities.
- **Individual scattered props** (trees, grass) — *render data, not entities*
  (instance groups), per ADR-0022.

## 5. Terrain (heightfield now)

Baseline stays the current **heightfield**: cheap, streams well, trivial LOD.
Tiled (see §7) but presented continuous. Material is a **splat** blended by the
biome/slope fields (grass low → rock on steep/high → snow on peaks).

## 6. Carving (later): SDF edits over the heightfield

A heightfield (`y=h(x,z)`) cannot represent caves/overhangs/undercuts — one
height per column. The path to carving reuses our existing SDF →
Surface-Nets polygonizer (`polygonizeSdf`):

- Terrain-as-SDF + **subtract** edit volumes (`max(a,-b)`): a tube swept along a
  spline = a river channel or tunnel; a box/sphere = a cave mouth or arch.
- Only chunks **containing an edit** convert their local heightfield to an SDF,
  apply the booleans, and re-mesh; all other chunks stay cheap heightfield.
- The carved chunk's generated mesh becomes its **collider** (mesh collider).

**SDF-carve vs voxels:** SDF-carve keeps terrain analytic + a small list of
*designed* edits, meshed on demand — ideal for authored rivers/caves. **Voxels**
store an editable 3D grid for *runtime carve-anywhere* (heavier: memory, LOD,
meshing). Order: **heightfield now → SDF-carve when designed carving is needed →
voxels only if runtime digging becomes a gameplay feature.**

## 7. Open-world streaming

Target is an open, streamed world. Reconciled with determinism (ADR-0002):

- **Deterministic per-tile generation** — each tile generated reproducibly from
  `(tileCoord, worldSeed)`, so a streamed-out tile regenerates identically.
- **Human edits as sparse per-tile override assets** — painted masks, carves,
  placed buildings layer on top of the procedural base; only edited tiles store
  data.
- **Streaming by distance with LOD rings** — full detail near camera; reduced
  density / imposters far; scatter recipes run per-tile at stream-in, freed at
  stream-out.
- **Build phasing:** design for streaming from day one (deterministic tile
  generators + per-tile overrides), but **implement against a small fixed tile
  set first**, then add the streaming manager.

### 7.1 Streaming-readiness — what to build toward now (not the manager yet)

The streaming *manager* (async load/unload, LOD rings, imposters) is **not**
built now. But to avoid a painful retrofit, everything built before it should
obey these orienting constraints so a tile is "just a bounded region" the day the
manager arrives:

- **Generators are tile-local and seed-deterministic** — no global mutable state;
  content addressed by world-space coordinate, reproducible from
  `(coord, worldSeed)` (ADR-0002).
- **Recipes take a bounded region as input** (already the §3 signature) — so a
  tile is a region with no special-casing.
- **Fields are sampled by world-space coordinate**, not array index — tiling
  becomes transparent (a tile samples its sub-rectangle of the same function).
- **Scatter output is per-region instance groups that can be freed** — keep
  everything resident today, but structure lifetime so a region's instances can
  be dropped/regenerated without touching neighbors.
- **Human edits are sparse overrides keyed by tile coord from day one** — even
  with one tile, store edits in the per-tile override layout (§9).
- **Pin a world-origin convention + a chunk-size constant now**, before a
  streamer exists, so coordinates and tile boundaries are stable.

Deferred until the manager: async load/unload scheduling, LOD rings, imposters,
and tile prioritization by camera distance.

## 8. City (future, generated)

Generated, no simulation. Buildings are best generated by a **split/shape
grammar** (subdivide a mass → floors → facade → panels; cf. CityEngine CGA),
*not* a turtle/plant L-system. Per **ADR-0028** this is an **L1 grammar sibling**
of the L-system: a C++ interpreter exposed to Lua, *driven by* free Lua recipes
(loops/functions), with props attached to facade **attach points** (§ADR-0028.2)
— the same composition mechanism as blossoms-on-tree. Interiors + exterior LOD +
interior streaming are far future. **Open decision:** the split-grammar as a
Lua-exposed engine helper (recommended, ADR-0028) vs a dedicated node graph —
deferred until the plant/scatter recipes prove the substrate.

## 9. World file & override sketch

```jsonc
{
  "world": {
    "seed": 1234,
    "tileSize": 256,
    "terrain": { "recipe": "recipes/terrain/alpine.lua", "params": { } },
    "fields": [
      { "name": "moisture",   "source": "procedural", "params": { } },
      { "name": "forestMask", "source": "asset", "raster": "masks/forest.r16",
        "paintable": true }
    ],
    "regions": [
      { "name": "north woods", "footprint": "<polygon|spline>",
        "recipe": "recipes/scatter/temperate_forest.lua",
        "seed": 7, "masks": ["forestMask"], "params": { "density": 0.7 } }
    ],
    "water":  [ { "kind": "river", "spline": "<spline>",
                  "recipe": "recipes/water/river.lua" } ],
    "overrides": "world_overrides/"   // sparse per-tile edited data
  }
}
```

## 10. Open questions

- Field storage at scale: per-tile raster chunks vs one big sparse raster? (ties
  to streaming).
- Mask resolution / channels: how many scatter masks before a splat-style packed
  texture is needed?
- Water realism bar: stylized flow-map surface vs any physics interaction.
- City: split-grammar-as-Lua vs node graph (see §8).
- LOD strategy for scatter (imposters? density falloff?) and for carved SDF
  chunks (no trivial heightfield LOD).

## 11. Phasing (when build starts)

1. Fields + scatter **recipes** over the existing heightfield (no streaming):
   procedural masks → instance groups; one **region entity**.
2. **Brush painting** masks in the editor (raster layers) + overlay rendering.
3. **Biome/material splat** driven by fields.
4. **Tiling + streaming** manager (deterministic tiles + per-tile overrides).
5. **Water** (still bodies → rivers).
6. **SDF carving** edits.
7. **City / building** grammar.
