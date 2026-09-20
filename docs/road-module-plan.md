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
// src/engine/procgen/roads/road_builder.h
struct RoadBuildInput {
    RoadGraph planned;                     // from the recipe, or authored
    RoadLook look;                         // widths, sidewalk, markings, lift
    std::function<Real(Real, Real)> ground;
    nlohmann::json options;                // builder-specific knobs (the level's block)
    Real renderCell = 250.0;
};
struct RoadProducts {
    std::vector<CellMesh> meshes;          // world-space, split by render cell
    RoadDeckField deck;
    std::vector<TerrainFlatten> flattens;
    RoadGraph nav;                         // what buildNavGraph consumes
    RoadGraph rowKeepOut;                  // freeway/ramp right of way, for the lot pass
    std::vector<Poly2> blocks;
    CurbBandAudit bands;                   // sidewalk loops (the city map reads them)
};
class RoadBuilder {
public:
    virtual ~RoadBuilder() = default;
    virtual const char* name() const = 0;          // "lattice" | "lanes"
    virtual RoadProducts build(const RoadBuildInput&) = 0;
};
RoadBuilder* roadBuilder(std::string_view name);   // registry; unknown name -> nullptr + warning
```

**Selection is data.** A road entity says `"builder": "lattice"` (default) or `"builder": "lanes"`
beside its existing `generate` block. `shape:"lanelab"` keeps working as an alias for
`shape:"road"` + `builder:"lanes"` with an authored graph, so lab levels do not move. Lua reaches
the same registry by name, so a recipe script can pick a builder.

## Phases

1. **Interface, no behaviour change.** Add `road_builder.h`, wrap the lattice and lanelab paths
   behind it, and make the loader instantiate `RoadProducts` uniformly. The gate is the census:
   every shipped level's `[citylots]`, `[furniture]`, entity counts and playable checks identical to
   this branch's parent.
2. **Fold lanelab in.** `procgen/lanelab/*` → `procgen/roads/lanes/*`, namespace `engine::lanelab` →
   `engine::roads`, the `lanelab` static library folded into `engine_core` (Clipper2 and CDT become
   ordinary engine deps), `RT_ENABLE_LANELAB` retired as a policy switch, lab levels moved into the
   normal tree. No "lab" left — it is the lanes builder.
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
