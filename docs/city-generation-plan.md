# City Generation Plan

A design for procedural cities: roads → blocks → parcels → buildings, plus the
"shape language" that grows a building, and where impostors fit. This is the
detail behind **ROADMAP Tier 4 Phase D** ("City / road layout"),
**`world-system-plan.md` §8 / §11.7** (City), and **ADR-0028 §3** (a split/shape
grammar as an L1 sibling of the L-system). The decisions here are ratified by
**ADR-0038** (Pending). Nothing is built yet; this is the plan behind that ADR.

> **The headline:** the city paradigm is already half-decided by our ADRs, and
> it is *not* the plant L-system that grows the trees. A building wants a
> **split/shape grammar** (subdivide a mass → floors → facade → panels;
> CityEngine CGA), and a city is a **region recipe** that lays roads, carves
> blocks, subdivides parcels, and runs the building grammar per lot — masking out
> nature (ADR-0027 §1). Impostors are a *rendering-at-scale* concern (ADR-0034
> §5), not part of generation — they come last.

---

## 1. The three questions, mapped to our architecture

The brief asked three things. Each already has a home in the codebase or the
ADRs:

| Question | Answer | Where it lives |
| --- | --- | --- |
| "A shape language to build a city" | A **split/shape grammar** (CGA-style): a *scope* (oriented box) rewritten by `split`/`repeat`/`comp`/`inset`/`extrude`/`roof` ops into mesh parts. | New L1 interpreter, sibling of `ParametricLSystem` (ADR-0028 §1/§3). |
| "Roads → city blocks → place buildings" | A **road graph** over a density field → **block faces** (planar-graph cycles) → **parcels** (recursive OBB subdivision) → **per-lot building** via the grammar. | New `procgen/city/` modules over the existing substrate. |
| "Imposters?" | Yes — but as the **object-LOD ladder** for a big city (discrete LOD → HLOD/building impostors), *after* generation produces the content. | ADR-0034 §5; depends on spatial partition (ADR-0035/0036). |

The key reframe: **generation and scaling are two separate workstreams.** The
grammar + road pipeline *produce* a city; impostors/HLOD let us *draw* a big one.
Build the producer first — it gives us something to look at, test headless, and
LOD later. A single hero building proves the grammar the way one tree proved
`growTree` (the Forest Arena pattern).

---

## 2. The substrate is already here

City generation invents very little. It composes existing value types (ADR-0021)
and tools:

- **Road centerlines** → `Spline<Vec3>` / `Path3` (curve lib, ADR-0031). Roads
  are arc-length-parameterized splines with rotation-minimizing frames — already
  what the tree branch sweep uses.
- **Building masses & booleans** → `sdf.*` + `polygonizeSdf` for welded/organic
  forms; `MeshBuilder::append/appendTransformed/merged` for kit-bashed parts
  (the common case for rectilinear buildings).
- **Grammar precedent** → `ParametricLSystem` (`procgen/lsystem.h`) is the L1
  pattern to mirror: declarative rules fed from Lua, hot rewriting in C++.
- **Placement / street furniture** → `scatterOnTerrain` (Poisson-disk, density
  masks, clustering) for lamps, trees, props along road edges.
- **Ground height** → `terrainHeight(params, noise, x, z)` to sit roads and
  foundations on terrain; `MeshCollider` for road/sidewalk collision.
- **Repeated assets** → `InstanceGroup` + `Renderer::drawMeshInstanced` (one draw
  per repeated building type / prop / streetlight).
- **Multi-material output** → `ScriptMeshPart` / `runProcgenModel` (a building is
  wall + glass + roof + trim parts, each its own material — ADR-0032).
- **Lua authoring** → `procgen_bindings.cpp` already registers `sdf` / `noise` /
  `mesh` / `lsystem` / `terrain` / `scatter` globals. The city adds `roads`,
  `parcels`, and a `building` (shape-grammar) global alongside them.
- **Zoning** → a `district` field, exactly analogous to the biome-id field of
  ADR-0027 §1 (height/density/use sampled per location).

What's genuinely new: the **shape grammar interpreter**, the **road-graph →
block-face** extraction, **parcel subdivision**, and the **city region recipe**
that wires them.

---

## 3. The pipeline (roads → blocks → parcels → buildings)

A directed pipeline, each stage a pure `(input, seed) → output` over value types
(ADR-0021), headless-testable end to end except the final render.

### 3.1 Road network — the skeleton
A graph of **nodes** (intersections) and **edges** (road segments, each a
`Spline<Vec3>` polyline with a width + class: arterial / collector / local).

- **Input:** a *population/density field* (FBM noise, or painted mask per
  ADR-0027 §3) + optional *seed roads* (highways, a river/coast the city hugs).
- **Method (phased):**
  1. **Bootstrap — deformed grid:** lay a grid (or radial+ring template) over the
     region, jitter vertices by noise, drop edges where density is low. Trivial,
     deterministic, good enough to unblock blocks/parcels/buildings downstream.
  2. **Target — agent/extended-L-system growth** (Parish & Müller 2001): grow
     arterials toward density peaks, branch collectors, fill with local streets;
     snap near-intersections, enforce min angle/length. Optionally **tensor-field
     guided** (Chen et al. 2008) for coherent street directions that bend around
     terrain/coast.
- **Output:** a planar road graph, snapped to `terrainHeight`. Edges become road
  surface meshes (swept ribbons along the spline) + `MeshCollider`.
- **Reuse:** `Spline<Vec3>`, `terrainHeight`, `MeshBuilder` sweep.

### 3.2 City blocks — the faces of the graph
**This is the heart of "roads → city blocks": a city block is literally an
enclosed face of the road network.** Roads are the *lines*; the blocks are the
*holes between the lines*. Once the road graph is planar, extracting blocks is a
classic computational-geometry operation, no extra authoring.

The road graph must first be **planar** (a 2D graph in the XZ plane — terrain
height is sampled *after*, so an overpass is not a true crossing). Planarize:
wherever two road segments intersect, insert a node and split both edges, so the
only adjacencies are at shared endpoints. Snap near-coincident nodes. Now:

- **Block = minimal cycle (face) of the planar graph.** Build a **half-edge**
  (doubly-connected edge list): each undirected road becomes two opposite
  directed half-edges. At every node, sort incident half-edges by angle. Trace a
  face by repeatedly taking, at each node you arrive at, the **next half-edge
  clockwise** from the one you came in on (the `next = twin → rotate CW`
  traversal). Each closed traversal is one face; mark half-edges used so each is
  visited once. Discard the single **outer face** (the unbounded one, identified
  by negative signed area / largest perimeter). Every remaining face is a city
  block. O(E log E) for the angular sorts; robust and deterministic.
- **Block footprint = inset face polygon.** Inset each block polygon inward by
  the bordering road's half-width + sidewalk width (a per-edge straight-skeleton
  offset, or a simple per-edge push for convex-ish blocks). That gap *is* the
  street + sidewalk; the inset interior is the **buildable footprint** the
  parcel subdivision (§3.3) then carves.
- **Tag each block** with a `district` sampled at its centroid (downtown /
  residential / commercial / industrial / park) — drives zoning, height, and
  whether it builds at all.
- **Edge cases:** dead-end roads (a half-edge with no enclosing cycle — they
  contribute to a face boundary but enclose nothing, handled naturally by the
  traversal); degenerate slivers (drop blocks below a min area); non-planar
  overpasses (defer — keep the graph planar first, ADR-0027-style); concave or
  ring-shaped blocks (the inset + parcel step must tolerate non-convex polygons).

So the dependency is strict and one-directional: **road graph → planarize →
faces → blocks → (inset) footprints → parcels → buildings.** Get the road graph
right and the blocks fall out for free; this is why §3.1's road network is the
real lever, and why even the deformed-grid bootstrap already yields usable blocks.

### 3.3 Parcels / lots — subdivide a block
Split each block footprint into building **lots**, each fronting a street.

- **Method (phased):**
  1. **Recursive OBB split:** compute the polygon's oriented bounding box, split
     along its longest axis at (jittered) midpoints, recurse until lots fall in a
     target area band. Simple, robust, gives believable rows. *Start here.*
  2. **Street-access refinement:** keep splits perpendicular to the frontage so
     every lot touches a road; corner lots are special-cased. (Straight-skeleton
     subdivision is the high-end version — defer.)
- **Output:** per lot: footprint polygon, the frontage edge (street-facing
  direction → building orientation), area, district tag.

### 3.4 Buildings — place and grow
Per lot, decide *whether* and *what*, then grow it with the shape grammar (§4):

1. **Occupancy & use:** sample the density/zoning field. Some lots → park,
   parking, plaza, or empty. Others → a building *type* drawn from the
   district's height/footprint distribution (a 60-story tower downtown, a 2-story
   house in a suburb).
2. **Mass:** inset the lot by a setback → footprint; pick a height from the type;
   the **scope** is the oriented box `(frontage frame, footprint, height)`.
3. **Grow:** run the split/shape grammar on the scope → multi-part mesh
   (`ScriptMeshPart[]`: walls, glass, roof, trim) + **attach points** for props
   (ADR-0028 §2: AC units, signage, balconies).
4. **Foundation:** sit the base on `terrainHeight` (step/terrace on slopes).

### 3.5 Decoration
Street furniture (lamps, trees, hydrants, benches) scattered along road edges via
a `scatterOnTerrain`-style pass with a road-distance mask; facade props populated
onto building attach points (the blossom-on-branch mechanism of ADR-0028 §2).

---

## 4. The shape language — a split/shape grammar (the centerpiece)

This is what the brief called "a shape language to build a city," and ADR-0028 §3
already chose it: **a split/shape grammar (CityEngine CGA), an L1 engine
interpreter exposed to Lua, a sibling of the L-system — not a plant L-system.**

### 4.1 Why not the L-system we already have
The turtle L-system grows *branches* — it is additive, organic, frames moving
through space. A building is the opposite: a **solid mass recursively
subdivided** into floors, bays, panels. Trying to express "split this facade into
4 equal window bays with 0.3 m mullions" as turtle rewrites is the wrong tool.
CGA's `split`/`repeat`/`comp` are the right primitives. They share the *substrate*
(seeded RNG, parameters, mesh output) but not the interpreter — exactly the L1
"grammar sibling" relationship ADR-0028 names.

### 4.2 The model: scopes and rules
- **Scope:** an oriented bounding box with a local frame — position, three axes,
  size `(sx, sy, sz)`. Every rule operates on the *current scope* and emits child
  scopes or geometry. (CGA's "scope" exactly.)
- **A rule** maps a shape symbol to a successor of operations, e.g.
  `Building → split(Y){ groundFloor | repeat: floor | roof }`.
- **Stochastic & parametric** like our L-systems: multiple weighted productions
  per symbol; successor sizes are expressions over scope dims and parameters.

### 4.3 The operation vocabulary (the "language")
A small, composable op set covers most architecture:

- `split(axis){ a | b | repeat(c) | d }` — subdivide the scope along X/Y/Z into
  sized parts; `repeat` tiles a part to fill (floors up a tower, bays across a
  facade). Sizes: absolute, relative, or `~` floating (CGA semantics).
- `comp(faces){ front: …, side: …, top: … }` — split a solid into its faces
  (apply a facade rule to walls, a different rule to the roof).
- `inset(d)` / `offset(d)` — shrink/grow the scope (setbacks, window reveals).
- `extrude(h)` — turn a footprint polygon into a prism (lot → mass).
- `roof(type, pitch)` — flat / hip / gable / mansard cap on the current scope.
- `taper` / `setback(atHeight)` — ziggurat/Art-Deco towers.
- `hollow(wallThickness)` — emit the scope as walls + floor + ceiling around a
  void instead of a solid mass: the **walkable-shell** op. A hollowed ground
  floor / lobby / atrium is an enterable open volume (no rooms — that's Tier C).
- `opening(door|arch|garage)` on a wall part — punch a real hole (a passable
  doorway, sized to human clearance) rather than paint a door on a solid. The
  player rig fits through it.
- `instance(assetId)` / `prim(box|cylinder|...)` — emit geometry (a window
  asset, a door leaf, a wall quad with a material).
- `attach(tag)` — flag an attach point for prop population (ADR-0028 §2), **and
  expose each floor as a `plate` scope** so a future interior-layout generator
  can fill it (the Tier-C seam, §4.6).
- `material(name)` — set the emitted part's material (wall/glass/concrete/trim).

A facade rule then reads like CGA:
```
Facade  → split(X){ ~1: Wall | repeat(3m): Bay | ~1: Wall }
Bay     → split(Y){ 1m: Sill | 1.5m: Window | 0.5m: Lintel }
Window  → inset(0.1) extrude(-0.1) material("glass") prim(quad)
```

### 4.4 Implementation shape
- C++ interpreter in `procgen/city/shape_grammar.{h,cpp}` (or
  `procgen/building.*`). Rule storage + parametric successors mirror
  `ParametricLSystem`; output is `MeshBuilder` parts, not a turtle mesh.
- **Lua surface** in `procgen_bindings.cpp`: a `building` global —
  `building.grammar()` → object, `:rule(sym, successor)`, `:run(scope, seed)` →
  `ScriptMeshPart[]`. Rules are declarative data fed from Lua (ADR-0028 §1); the
  rewrite/emit loop stays hot in C++. Authored recipes live in
  `assets/scripts/city.lua` (cf. `flora.lua`).
- **Headless-tested** like `test_flora.cpp` / `test_script_vm.cpp`: a known
  recipe → deterministic vertex/part counts; a script-vs-C++ equivalence check.

### 4.6 Scope decision (June 2026): walkable shells, not interiors
Decided with the user: the building language targets **Tier A+B — facades plus
walkable shells**, not Tier C interiors. The grammar emits detailed exteriors
*and* can `hollow` a mass + `opening` real doorways so the ground floor / lobby /
atrium is an **enterable open volume** at human scale. It does **not** generate
rooms, hallways, connected doors, or reachable layouts — that is a separate
*floor-plan synthesis* generator (graph + reachability constraints; e.g. Merrell
et al.), a different paradigm from the facade grammar.

The seam is designed now so Tier C drops in later without a redesign: the grammar
exposes each floor as a **`plate` scope** + attach points (ADR-0028 §2). A future
interior generator is a recipe that *fills* a plate scope — the same
recipe-calls-recipe composition as blossoms-on-a-branch. **Human scale is a
first-class constraint**: split sizes are real meters (floor-to-floor ~3–4 m,
door clearance ~2.1 m, sill ~0.9 m, railing ~1.1 m), and part of this work is
pinning those + the player eye/clearance as named constants the grammar reads
(today the convention is loose). A door the grammar punches is a door the player
rig fits through.

### 4.5 Geometry strategy
Rectilinear buildings are **kit-bashed** (`MeshBuilder::append` of wall/window
quads + box masses) — cheap, clean UVs, instanceable windows. Reserve **SDF +
`polygonizeSdf`** for organic/blended forms (domes, parametric blobs, eroded
brutalism) where welds matter — same split as trees (cylinders vs. SDF skin,
ADR-0029).

---

## 5. Where impostors fit (the "imposters?" question)

Impostors are **not** a generation feature — they are how we *draw a big city at
frame rate*, and they're already specified by **ADR-0034 §5** as the object-LOD
ladder:

> discrete mesh LOD → foliage impostors → **HLOD (merged simplified proxies) +
> building impostors** for the distant city.

A building impostor is a **billboard / octahedral-card** baked from the building
mesh (render it from N directions to an atlas; draw a camera-facing quad far
away). **HLOD** merges a whole block/district into one decimated proxy mesh.
None of this exists yet — it's owed tech debt (`TECH_DEBT.md`, `NEXT_STEPS.md`
list "distant-tree impostors still owed").

Two consequences for *this* plan:

1. **Impostors come after generation.** You can't bake an impostor of a building
   you haven't generated. Generation produces the content; impostors/HLOD scale
   the draw. So they sit in Phase 4, gated on the spatial partition (ADR-0035 AABB
   culling landed; sector grid / streaming is the rest of ADR-0034).
2. **But design for them now (cheap).** The building grammar should emit, besides
   the full mesh: (a) a **coarse proxy** (the mass boxes with a baked facade
   texture) for HLOD, and (b) a clean single-material silhouette for impostor
   baking. Emitting these is nearly free at generation time and makes the LOD
   bake trivial later. Repeated building *types* also collapse into
   `InstanceGroup`s, which already carry a `drawDistance` cull — the first, free
   LOD lever.

Walkable ground (streets, plazas, the terrain the city sits on) is **terrain
LOD**, never an impostor (ADR-0034 §4).

---

## 6. Integration with the world system (ADR-0027)

A city is **one region entity** (ADR-0027 §4), not thousands of building
entities. Its recipe (`recipes/city/downtown.lua`) is
`(region, fields, seed) → content` and:

- **Masks out nature:** the city footprint suppresses the forest/grass scatter
  recipes underneath it (ADR-0027 §1, world-system-plan §3 "a city suppresses
  natural scatter").
- **Reads fields:** a `district`/`density` field (procedural + brush-paintable,
  ADR-0027 §3) drives zoning, height, and road density.
- **Expands to instance groups + per-building meshes** at load (or per-tile when
  streaming lands), individual props staying render data not entities (ADR-0022 /
  ADR-0027 §4).
- **Determinism & tiling (ADR-0027 §5):** every stage is seed-reproducible. The
  hard tiling problem is **roads crossing tile boundaries** — the road graph is
  city-global, not tile-local. First cut: generate a **bounded city region whole**
  (like the Forest Arena), defer cross-tile road stitching to the streaming step
  (generate the graph globally, then clip geometry per tile).

Authoring is data, two front-ends (ADR-0025/0027 §2): the same recipe runs from
Lua or from the editor (paint a district mask, drop a city region, tune params).

---

## 7. Phasing — vertical slices, "shape language" first

Each phase is independently shippable and headless-testable except the final
render (ADR design principle 4). Ordered so the most reusable, most exciting
piece (the shape grammar) lands first and each phase has something to look at.

**Status (June 2026): Phases 0–3 implemented** under `src/engine/procgen/city/`
(`polygon`, `shape_grammar`, `parcel`, `road_network`, `city`), deterministic and
tested (`tests/test_city.cpp`), with a Lua `building.*` authoring surface
(`tests/test_script_vm.cpp`, `assets/scripts/city.lua`) and an offline render
(`shape:"city"`, `assets/levels/city.json`). HLOD proxy emitted. Phase 4
(impostor bake, streaming, cross-tile roads) deferred — needs a GPU / the spatial
partition.

- **Phase 0 — The building shape grammar (standalone). ✅ Done.** The interpreter
  + the `split`/`repeat`/`comp`/`inset`/`extrude`/`roof`/`hollow`/`opening` ops
  (`shape_grammar.{h,cpp}`), Lua-exposed (`building.grow`/`building.height`). Hero:
  a **mid-rise mixed-use block** with a **walkable ground shell** (hollowed, real
  entrance opening), windowed floors, parapet roof; a skyscraper is the same with
  big `floors` + setbacks. Multi-part mesh (wall/glass/trim/roof/door) + attach
  points + a coarse proxy. `assets/scripts/city.lua` is the recipe.
- **Phase 1 — Lot → building. ✅ Done.** Recursive-OBB parcel subdivision
  (`parcel.{h,cpp}`) with frontage; occupancy/type/setback per district.
- **Phase 2 — Roads → blocks. ✅ Done.** A road graph + deformed-grid generator,
  planarize (split crossings), and half-edge DCEL block-face extraction
  (`road_network.{h,cpp}`). (Agent/L-system/tensor-field road *growth* is the
  open upgrade past the grid bootstrap — §9.)
- **Phase 3 — The City. ✅ Done (headless + offline render).** `city.{h,cpp}` ties
  it together with district zoning (downtown towers → midtown → residential +
  parks); `shape:"city"` renders it offline (`assets/levels/city.json`).
  *Remaining for a "City Arena" parity with The Forest:* sit it on terrain, mask
  natural scatter under the footprint, scatter street furniture, run it as a
  proper ADR-0027 world *region recipe* — and the viewer (Metal) render.
- **Phase 4 — Scale. ⏳ Deferred (needs a GPU / spatial partition).** The
  generator already emits the inputs — a coarse per-building proxy and a merged
  `CityModel::hlodProxy` (headless-tested). Owed: the **impostor-card bake**
  (render-to-atlas) + **HLOD swap/crossfade** (Metal, ADR-0034 §5), and **sector
  streaming + cross-tile road stitching** (ADR-0027 §5 — the city generates whole
  today). Only worth it when the city is large enough to need it.

**ADR-0038** ("City generation: a split/shape grammar over a road→block→parcel
pipeline, as a world recipe") records the decisions in §3–§6 against the
alternatives below; it is Pending until Phase 0 build starts.

---

## 8. Alternatives considered (for the eventual ADR)

- **Buildings via the plant L-system** — rejected (ADR-0027/0028, §4.1 here):
  paradigm mismatch; additive turtle growth can't cleanly express recursive mass
  subdivision. The split grammar shares the substrate, not the interpreter.
- **A node graph for the grammar instead of Lua** — deferred (ADR-0025): Lua is
  the one procgen authoring path; a visual editor, if ever, emits Lua. The CGA
  grammar is declarative data fed from Lua, consistent with the L-system.
- **Pure-grid roads forever** — rejected as the *target* (fine as the Phase 1
  bootstrap): real cities want density-driven arterials and terrain-aware bends;
  the deformed grid unblocks downstream work but the agent/tensor-field growth is
  the goal.
- **Voronoi/Poisson parcels** — viable but rejected as the default: recursive OBB
  split gives street-aligned rows that read as city blocks; Voronoi reads organic
  (good for old-town districts as a *variant*, not the base).
- **Thousands of building entities** — rejected (ADR-0022/0027 §4): a city is one
  region entity over instance groups + generated meshes.
- **Impostors as part of generation** — rejected: impostors are a render-scale
  LOD concern (ADR-0034 §5), baked *from* generated meshes, gated on the spatial
  partition; generation comes first.
- **Full traffic/agent simulation** — out of scope (world-system-plan §8:
  "Generated, no simulation"). The temporal-generator track (ADR-0021) is where
  any future agent sim would live, not the static-geometry pipeline.

## 9. Open questions

- **Road growth algorithm** for Phase 2: extended-L-system (Parish & Müller) vs.
  tensor fields (Chen et al.) vs. agent-based. Bootstrap with a deformed grid;
  decide the target when Phase 2 starts.
- **Parcel quality:** is recursive OBB enough, or do we need straight-skeleton
  subdivision for believable corner/irregular lots?
- **Cross-tile roads:** generate the graph globally then clip per tile, vs. a
  tile-local graph with boundary-matching constraints (ADR-0027 §5). Defer to
  Phase 4.
- **Window/facade detail vs. instancing:** per-bay geometry (rich, heavy) vs. a
  tiled facade texture/atlas (cheap, the real shipping answer) vs. both via LOD.
- **District field authoring:** procedural-only first, or brush-paintable
  districts from the start (ADR-0027 §3)?
- **Interiors:** explicitly far future (world-system-plan §8); exterior-only now.
