# One road module, two builders

**Trigger.** Glenn, 2026-09-20: "I'd like to take what's in lanelab and make it a new road building
module and allow us to switch between using the old one and the new one ... I'd like to be able to
call one or the other up via lua or some common C++ interface. That is, I think we would get rid of
lanelab altogether and everything would be integrated. Metro_test_v2 would just use the new road
system."

## Where we are

ADR-0083 built lanelab behind a link boundary; ADR-0085 kept both generators and chose per entity by
`shape` — `shape:"road"` sweeps the lattice, `shape:"lanelab"` runs the lane-atomic generator. That
ADR also wrote down the only thing the rest of the engine needs from a road generator, and it has
held: **a unified `RoadGraph`, a `RoadDeckField`, terrain flattens, and blocks.** Everything
downstream (nav graph, traffic, furniture, street signs, the city map, lots) reads those four.

What is NOT shared today is the input. The lattice takes a `generate` **recipe** and plans its own
graph; lanelab takes a **resolved lane graph** (`RoadLabGraph`) from a file or an inline spec. That
difference is why ADR-0085 rejected one entity schema — and it is the thing to fix, because a level
like metro is a recipe.

The bridge already exists: `procgen/lanelab/level_import.h` runs the engine's *own* recipe
(`applyGenerateRecipe`) and turns the resulting chains into a lane graph (freeway loop, diamonds,
pockets as options). That is how the `assets/lanelab/` metro was made.

## The shape to build

Split **planning** from **building**, and make the builder swappable:

```
recipe (level "generate" block) ──applyGenerateRecipe──► planned RoadGraph
                                                              │
                                        ┌─────────────────────┴───────────────────┐
                                        ▼                                         ▼
                            LatticeRoadBuilder (road_net)              LaneRoadBuilder (lanelab)
                                        └─────────────────────┬───────────────────┘
                                                              ▼
                                                        RoadProducts
                    meshes by cell + materials · RoadDeckField · terrain flattens
                    unified RoadGraph (nav) · right-of-way keep-out · blocks
```

```cpp
// src/engine/procgen/city/roads/road_builder.h  (as built)
struct RoadBuildInput {
    const RoadEntity* road = nullptr;      // planned: the recipe has run
    RoadGroundFn ground;                   // null = flat
    nlohmann::json options;                // the level's road block
};
struct RoadProducts {
    RenderMesh mesh;                       // carriageway, kerbs, sidewalks, markings
    RoadDeckField deck;                    // the surface things stand on
    CurbBandAudit bands;                   // sidewalk loops (the city map reads them)
};
class RoadBuilder {
public:
    virtual ~RoadBuilder() = default;
    virtual const char* name() const = 0;              // "lattice" | "lanes"
    virtual RoadProducts build(const RoadBuildInput&) const = 0;
    virtual std::vector<TerrainFlatten> flattens(const RoadBuildInput&) const = 0;
    virtual RoadGraph navGraph(const RoadBuildInput&) const = 0;
};
RoadBuilder* roadBuilder(std::string_view name);   // registry; unknown name -> nullptr + warning
```

**Selection is data.** A road entity says `"builder": "lattice"` (default) or `"builder": "lanes"`
beside its existing `generate` block. `shape:"lanelab"` keeps working as an alias for
`shape:"road"` + `builder:"lanes"` with an authored graph, so lab levels do not move. Lua reaches
the same registry by name, so a recipe script can pick a builder.

## Phases

1. ~~**Interface, no behaviour change.**~~ **Done** (ADR-0089). `procgen/city/roads/` holds the
   module: `road_entity.{h,cpp}` (the shared model that was road_net's first half), `road_builder`
   (interface + registry) and `lattice_builder.cpp`. `procgen/deprecated/roads/` holds the old
   builder: `road_net_mesh.{h,cpp}` (mesher, carve, walls) and `road_lattice.{h,cpp}`. The loader
   and the terrain pre-pass both resolve `roads::roadBuilderFor(roadBlock)`. Gate: run_tests
   1376/1378 (the two known pre-existing failures) and the level census byte-identical across the
   move.
2. ~~**Fold lanelab in.**~~ **Done** (ADR-0089). `procgen/lanelab/*` → `procgen/city/roads/lanes/*`,
   namespace `engine::lanelab` → `engine::roads::lanes`, guards and identifiers renamed with it
   (`lanelab.{h,cpp}` → `lanes.{h,cpp}`, `g_lanelab` → `g_lanes`, `kLanelabBuildTag` →
   `kLanesBuildTag` — the tag's VALUE is untouched, it is a cache key). The static library is gone:
   the sources build inside `engine_core`, with Clipper2 and CDT as ordinary (private) engine deps.
   `RT_ENABLE_LANELAB`/`RT_BUILD_LANELAB` became one switch, `RT_ROADS_LANES`, which asks "is the
   lanes builder in this build?" — a capability, not a lab. `lanelab_tool`/`lanelab_tests` are
   `lanes_tool`/`lanes_tests`. What did NOT change: the data. `shape:"lanelab"` and
   `assets/lanelab/` still name the entity and its levels, because levels are content and they
   migrate in phase 3.

3. **metro on lanes.** metro_v2_test keeps its recipe and sets `builder:"lanes"`; the lanes builder
   plans with `applyGenerateRecipe` and converts with the `level_import` logic. Keep `lattice`
   selectable for A/B. Then re-run every metro gate — playable, citylots, junctions, pedestrians,
   buses, parking, street signs, map — and fix the fallout.
4. **Retire the duplication** that phase 3 exposes (the bundle producer's identity key must include
   the recipe; the lab's own level list folds into the level scan).

## Risks worth naming now

- **The nav graph changes shape.** On lanes, the unified graph comes from `roadTwin()`, not from the
  recipe's chains: different node spacing, different junction nodes. Everything measured this week
  (stop lines, box radii, sign corners, bus stops) keys off that graph, so metro's numbers will
  move. The gates are written in terms of behaviour, not counts, which is what makes this checkable.
- **Build time and caching.** The lanes path is heavier and caches through the bundle producer, whose
  identity is the graph bytes. A recipe-planned graph must fold the recipe (and terrain) into that
  key or a stale city will load.
- **Two meshers, one look.** Materials, markings and kerb heights are authored per generator today;
  metro switching builders will show any place where they disagree.
- **ADR-0085's hazard still stands:** code reached only under the lanes path must prove it is
  looking at lanes content, not assume it from a build flag.

## What phase 1 deliberately did not move

* **The nav graph call sites.** The loader still calls `navRoadGraph` where it derives the level
  road graph (it is the lattice's `navGraph`, the same function). On lanes the graph comes from
  `roadTwin()`, so those sites must resolve a builder too — but they work from components, not from
  the level JSON, so they need the chosen builder recorded beside the entity. Phase 3's first job.
* **`road_network`, `road_mesh`, `road_spec` and the planning files** (metro, district, constraints,
  rules, semantics). They are shared road model, used by both builders; they stay in
  `procgen/city` until the lanes builder lands, then move into `city/roads` as a rename.
* **`shape:"lanelab"`.** Still the lab's own entry point until the lanes builder can take a recipe.

## What phase 3 has to decide before it starts: the terrain contract

Reading the two paths side by side (`loadRoadEntity` vs `loadLanesEntity`) turns up a fork the
ADR-0085 product list papers over. **The lattice CARVES the level's terrain**: it returns
`TerrainFlatten` regions, the loader folds them into the heightfield before anything is built, and
one ground serves roads, lots and CDLOD. **The lanes builder REPLACES the ground**: its pipeline
conforms its own `HeightGrid`, publishes it as `g_lanes.ground`, and the loader skips drawing the
lanes' own terrain mesh when the level has a `terrain` block. `terrain_conform.h` says as much —
"the lab nudges a heightfield grid; integration emits flatten regions" is listed as an export path
that was never built.

**Decided (Glenn, 2026-09-20): the builder hands back the ground.** "I think it should hand back the
ground which is what it currently does. I think handing back patches is the old way of doing it."
So `flattens()` became `ground()`, returning a `GroundPlan` with exactly one side filled — `carve`
(the lattice's patches, pressed into the level's terrain) or `replace` (a baked `GroundGrid` of
absolute heights with the cut and fill already in it). Making lanes emit patches would have
re-described, more coarsely, a grid it already computes exactly — and lost precision at junctions,
which is where it is most careful. The path is also already proven: the lanes grid rides into the
terrain through `TerrainParams::erodedBase` (the slot a baked erosion uses), so CDLOD renders it with
LOD, morphing and material blending, falling back to the level's own terrain outside the grid over a
60 m blend. What blocked it in September — lot pads graded after the terrain, 706 of 1284 lots buried
— was fixed by publishing the lanes' blocks and ground BEFORE the terrain pre-pass.

The second product list is also wider than `RoadProducts` today: lanes build **many** meshes (one
per render cell × material, with paint/collidable flags), not one. `RoadProducts::mesh` becomes a
list of named cell meshes; the lattice returns a list of one, and the loader spawns per mesh the way
`loadLanesEntity` already does.

And the third thing phase 3 owes: the loader still calls `navRoadGraph` directly where it derives
the level road graph, so a lanes level's nav graph would silently come from the lattice's sampler
rather than `roadTwin()`. Those sites work from components, not from the level JSON, so the chosen
builder has to be recorded beside the road entity first.

## Priority, after looking at the old freeway in the viewer (2026-09-20)

Glenn, on the legacy corridor freeway: *"we know it doesn't work because it doesn't correctly merge
properly with other roads ... that's all legacy ... the new road system should be given priority."*

So the order changes. The lane builder must not consume the legacy corridor's **output** — that is
the old freeway, and it is going away. What survives of that pipeline is its **input**: the route
planner's anchor polyline, "a freeway should run from here to here", plus an interchange spacing.
That part is cheap, geometry-free and honest.

1. **Lanes builds the freeway from a route.** `level_import` stops tracing the city's outer face;
   it takes a route as data (the recipe's `plan.freewayPlans`, or a level's own `freewayPlans`
   block) and the lanes builder constructs the carriageways, ramps, elevation and interchanges
   itself, the way it builds everything else. The traced ring becomes a bypass a level ASKS for.
2. **Then the shapes Glenn wants**: a freeway cutting through the city elevated with ramps; a
   cloverleaf where two perpendicular freeways meet, then a route around the city. `freeway_cross`
   in the corpus is the first geometry of the second one.
3. **Then the corridor pipeline retires** — `corridor_plan`, `corridor_mesh`, the bake, and the
   `corridor_freeways` recipe key — once the levels that ship with it (freeway_lab,
   freeway_variants, hillcity, metropolis, metropolis_roads, metropolis_sky) can be built by lanes.

metro_v2_test is untouched by all of this and stays the city it has always been.

### How the route goes in (the seam, read 2026-09-20)

`level_import.cpp` builds its freeway from one variable: `loop`, a polyline, plus `isLoop` (which
source chains the freeway replaces) and `loopS` (stations along it). Everything after that —
landing selection, diamonds, band ramps, trimming streets short of the carriageway, the frontage
ring — works by station along that polyline. Today `loop` comes from `outerFace(chains)`: the city's
traced boundary.

So the change is to make `loop` come from **data**, and the ring one case of it:

```
ImportOptions.routes : [ {points: [[x,y]...], closed: bool} ]   // authored, or the recipe's plan
     empty + freewayLoop  -> today's traced ring (a bypass a level ASKS for)
     one open route       -> a freeway that cuts through, ends at the map edge
     several routes       -> they cross; the crossing is an interchange (the cloverleaf)
```

Reading the rest of the block changes the shape of the job. The ring is not just a polyline: the
code around it is built on *inside*. `pointInRing(loop, p)` decides which streets are in the city,
which side a ramp lands on, and where the frontage ring goes; `offsetLoop` wraps; `designLoop` is
morphological closing-and-opening, which only means anything for a closed curve. Almost all of that
machinery exists to cope with the fact that the ring was TRACED from a jagged outline.

An authored route needs none of it. It arrives already designed, it has no inside, and its
alignment is not ours to redraw. So the route is a **separate, simpler path** rather than a retrofit
of the ring:

* two carriageways offset either side of the polyline (the ring already does this — reusable)
* every street chain it crosses is either **trimmed short** of the carriageway or promoted to a
  **landing**; "inside the ring" becomes "left or right of the route", which is well defined
* no frontage ring, no morphological redraw, no ring tests
* its two ends are open, so the profile must come down to the ground there — the defect
  `steep_climb` already pins

The ring path stays where it is, for the bypass case, until it is retired.

The level's `freewayPlans` block is the natural authoring surface — the loader already reads one at
top level for the rules lab, so the same data serves both road systems while the old one lives.

### Diamonds on a route: three things that had to be right (2026-09-20)

A route's diamond is simpler than the ring's — each carriageway drops a ramp into the band beside it
and meets the street the freeway crosses, with no frontage road to invent. Getting it to hold took
three corrections, each of which the invariants named:

1. **Size the ramp against the ground it LANDS on, not the crossing.** The first cut measured the
   climb at the crossing point and arrived at the street too high; the profile solver then dragged
   the STREET up to meet it — `c27 26.97% > 8%` — and the junction solve went divergent
   (338 → 474 cm).
2. **Finish ALONG the street, not into its centreline.** A ramp driven in at right angles is a
   sliver in the pavement union, not a junction: 193 non-manifold edges and 309 cracks. Approaching
   from 40 m back and running tangent brought that to 51 and 80.
3. **Anchor the free end's HEIGHT to the street.** The graph format already has it: a bare edge id
   as `from`/`to` is a height-only anchor (`readAnchor`, road_graph_spec.cpp:35). With
   `"to": "c214"` the ramp arrives at the street's own level instead of the two fighting —
   **0 non-manifold, 24 cracks, and the junction solve converges to 0.4 cm.**

The ramps must also be emitted AFTER the streets they name: an anchor may not reference a later
edge. Route ramps are collected and appended once the street edges exist, and any whose street did
not survive is dropped with a note.
