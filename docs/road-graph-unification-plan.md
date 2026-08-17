# One road graph — unification proposal

**Status:** APPROVED, stowed. Steps 1–2 landed (`ae4e396`); steps 3–6 are a
single uninterrupted session's work — see "Measured cost" before starting.

## Why

There are two road types and following the code means holding both in your head:

- `RoadGraph` (`road_network.h`) — the geometric working form. Attributes live on
  `RoadNode` / `RoadEdge` structs.
- `RoadNet` (`road_net.h`) — the authored/persisted form. Topology lives in
  **eight parallel arrays** (`edges`, `tangents`, `nodeElev`, `nodeKinds`,
  `edgeWidths`, `edgeSpecs`, `edgeBaked`, `edgeLayers`, `edgeClasses`), any of
  which may be short or absent.

The conversion runs one way almost exclusively: **graph → net once**, in
`applyGenerateRecipe`; **net → graph at ~29 call sites** (`netGraph`,
`navRoadGraph`, `constrainedNetGraph`). So every generator already produces the
good shape, and we immediately transpose it into the awkward one.

Measured cost of the parallel arrays: **54 defensive `.size()` guards** across
`road_net.cpp` (24), `corridor_bake.cpp` (13), `corridor_plan.cpp` (13),
`city_planner.cpp` (2), `city_lots.cpp` (2) — plus `roadNetEdgeWidth(net, ei)`, an
accessor that exists only because `edgeWidths` might not be there.

## The idea: split by LIFETIME, not by representation

The earlier verdict was "keep both", on the grounds that `RoadNet` holds things a
geometric graph should not — the look, and a `std::function` terrain sampler. That
reasoning was right about the fields and wrong about the fix. Those fields do not
need a second *topology*; they need to stop living on the topology at all.

Three things with three different lifetimes are currently welded into one struct:

| What | Lifetime | Belongs |
| --- | --- | --- |
| nodes, edges, widths, classes, layers, elevations | the road itself | the graph |
| width default, sidewalk, curb, markings, colour | how it's drawn | a look struct |
| `cityHubs`, `freewayPlans`, `siteFootprints` | what the generator knew | a plan struct |
| `heightAt` | one load; not serializable | **a function parameter** |

## Proposed types

```cpp
// ---- geometry: the ONE graph -------------------------------------------
struct RoadNode {
    Vec2  pos;
    Vec2  tangent{0, 0};        // spline shape; zero = auto (Catmull-Rom)
    double elev = 0;
    bool  elevAbsolute = false; // true = authored deck height, not a drape
    JunctionKind kind = JunctionKind::Auto;
};

struct RoadEdge {
    int    a = 0, b = 0;
    double width = 0;           // ALWAYS resolved; 0 never reaches a consumer
    RoadClass klass = RoadClass::Local;
    int    layer = 0;           // grade separation tier
    int    spec  = -1;          // index into RoadGraph::specs
    bool   baked = false;       // came from a corridor solve
    RoadProvenance provenance = RoadProvenance::Street;
    bool   oneWay = false, walkable = true;
    uint8_t access = road_access::kAllStreet;
    double parkOffset = 0, parkWidth = 0;
};

struct RoadGraph {
    std::vector<RoadNode> nodes;
    std::vector<RoadEdge> edges;
    std::vector<RoadSpec> specs;      // band table; RoadEdge::spec indexes it
    int  addNode(const Vec2& p, Real tol = 0.5);
    void addEdge(int a, int b, Real width = 8,
                 RoadClass klass = RoadClass::Local);
};

// ---- presentation: how it is drawn, not what it is ---------------------
struct RoadLook {
    double defaultWidth = 10.0;   // seeds RoadEdge::width where unspecified
    double sidewalk = 3.5, curb = 0.16, cornerRadius = 3.0, lift = 0.08;
    bool   markings = true, crosswalks = true, autoRoundabout = true;
    Vec3   color{0.09, 0.09, 0.10};
};

// ---- provenance: what the generator knew, for downstream passes --------
struct RoadPlan {
    std::vector<CityHub> cityHubs;                  // polycentric zoning
    std::vector<std::vector<Vec2>> freewayPlans;    // corridor anchors
    std::vector<Footprint> siteFootprints;          // P8 footprint-first
};

// ---- the entity/component that replaces RoadNet ------------------------
struct RoadEntity {
    RoadGraph graph;
    RoadLook  look;
    RoadPlan  plan;
};
```

`heightAt` is **gone from the struct**. It is already a parameter to
`buildRoadNetLattice(g, heightAt, …)`; the remaining readers take it the same way.
That deletes the "set on load, not serialized" special case and the lifetime
question that comes with storing a closure.

## Four rules that do the actual work

1. **`RoadEdge::width` is always resolved.** Today `roadNetEdgeWidth(net, ei)`
   falls back to `net.width` because `edgeWidths` may be short. Instead the
   generator and the deserializer resolve it **once at construction** from
   `RoadLook::defaultWidth`. The accessor and its 54 sibling guards disappear
   because there is nothing left to guard.
2. **The JSON wire format does not change.** `roadNetFromJson`/`roadNetToJson`
   become the *only* place that knows about parallel arrays — they transpose on
   the way in and out. No saved level, no editor document, no test fixture moves.
3. **Generators return `RoadGraph` directly.** `buildDistrict` and `buildMetro`
   already do. `applyGenerateRecipe` stops transposing and becomes an assignment;
   `netGraph` / `constrainedNetGraph` collapse to a constraints pass over a graph
   we already have.
4. **Node and edge ORDER is preserved exactly.** Generators seed rng off indices,
   so a reordering silently regenerates every city. This is the migration's one
   real correctness risk and wants a determinism test pinned before and after.

## What this buys the editing and generation tools

- **Editor.** Drag is `graph.nodes[i].pos`; widen is `graph.edges[i].width`. Today
  both mean "index into the right parallel array and hope it is long enough."
- **Generation (piedmont).** `buildMetro` produces hubs + freeway plans + a graph;
  those land in `RoadPlan` and `RoadGraph` with no transpose. The metro path is
  the one that carries the most side-band data, so it gains the most.
- **Lua.** `city.layout` already hands Lua a `{nodes, edges}` table — unchanged.
  The bindings build a `RoadGraph` instead of a net, which is what the lattice
  wanted anyway.
- **Levels.** `city.json`, `city_arena.json` and every metro level are `shape:"road"`
  and read the same JSON. `twin_cities.lua` goes through the same bindings. None
  of their content changes; they are shaped around the new graph automatically
  because they never touched the C++ type.

## Measured cost — read this before starting

The "~29 call sites" figure elsewhere counts CONVERSIONS (`netGraph`,
`navRoadGraph`, `constrainedNetGraph`). The migration's real surface is FIELD
touches, and it is an order of magnitude bigger:

| field | touches on `net` alone |
| --- | --- |
| `net.edges` | 78 |
| `net.nodes` | 72 |
| `net.edgeWidths` | 41 |
| `net.edgeClasses` | 35 |
| `net.edgeSpecs` | 27 |
| `net.edgeLayers` | 27 |
| `net.tangents` | 26 |
| `net.nodeElev` | 23 |
| `net.edgeBaked` | 21 |
| `net.nodeKinds` | 12 |

Plus other variable names, across `road_net.cpp` (311 field references),
`corridor_plan.cpp` (76), `corridor_bake.cpp` (68), `level_loader.cpp` (52), and
further sites in the editor, citysim, `city_lots.cpp` and `city_planner.cpp`.
Realistically 600+ individual edits.

### Why a find-and-replace will corrupt this

The parallel arrays carry **fallback semantics, not just data**:

```cpp
double roadNetEdgeWidth(const RoadNet& net, int ei) {
    if (ei >= 0 && ei < (int)net.edgeWidths.size() && net.edgeWidths[ei] > 0.0)
        return net.edgeWidths[ei];
    return net.width;                 // short OR zero => the net default
}
```

A short or zero entry *means something*. So the migration has to resolve those
fallbacks at every **construction** site, not merely rewrite readers. Blindly
rewriting `net.edgeWidths[i]` → `net.graph.edges[i].width` changes behaviour
wherever an array was short — and changes it **silently**, because the order gate
below hashes ordering, not widths. That per-site judgement is the actual work.

There is no safe midpoint: the tree compiles against the arrays or against the
graph, nothing in between. Budget one uninterrupted run.

## Migration order

1. ~~Add `RoadNode::tangent` and `RoadEdge::baked` to `RoadGraph`.~~ **DONE**
   (`ae4e396`). Both APPENDED — inserting `baked` after `width` broke three
   positional brace-inits and landed a `RoadClass` in a `bool`.
2. ~~Pin determinism.~~ **DONE** — `tests/test_road_graph_order.cpp`, including a
   permutation test so the gate cannot pass by ignoring order. Expected hashes:
   district `11690313017498955230`, metro `7979088897799340174`. If these move
   during steps 4–6, the ordering broke, not the geometry.
3. Introduce `RoadLook` / `RoadPlan` / `RoadEntity` **as part of the same step
   that moves call sites** — do NOT land them unused ahead of time. Unused types
   waiting for a consumer is the aspirational-infrastructure pattern already
   flagged against `SurfaceField` in `docs/TECH_DEBT.md`.
4. Move the ~29 `net → graph` call sites to read `entity.graph` directly. The
   compiler enumerates them — the same method that worked for retiring the second
   city pipeline.
5. Delete `RoadNet`, `roadNetEdgeWidth`, `netGraph`, `constrainedNetGraph`, and the
   54 guards. Keep `navRoadGraph` — it does real work (spline sampling), not a
   conversion.
6. Confine parallel arrays to `roadNetFromJson` / `roadNetToJson`.

## Risks, honestly

- **Breadth.** ~29 call sites across the loader, the editor, corridors, citysim
  and the lot pipeline. Mechanical, but wide.
- **Determinism.** Step 4 is where a city silently regenerates. Step 2 exists for
  this reason; do not skip it.
- **`specs` ownership.** `RoadSpec` tables are populated only on the `"metro"`
  path today; district levels are specless. The merged type must keep `spec = -1`
  meaning "legacy, synthesize from width/look", or district roads lose their
  sidewalks.
- **Editor round-trip.** `spawnDocumentEntity` serialises the recipe, not the
  graph. Worth confirming a hand-edited road still round-trips through Play.
