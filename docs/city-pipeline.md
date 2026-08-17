# The city build pipeline (v2)

Canonical ordering for procedural city generation, decided on-device 2026-07-13.
This **supersedes** the ad-hoc ordering scattered through metropolis-scale-plan
§10.6 / freeway-rules §12: those built streets first and bolted the freeway on
afterward, which is the root cause of the freeway crossing bad junctions, ramps
hunting for landings, and cut streets orphaning blocks. v2 inverts that.

## Two load-bearing principles

1. **Each stage's output is the next stage's fixed input. Never reach backward.**
   Terrain constrains hubs; hubs place the freeway; the freeway's ramp landings
   seed the streets; the streets define the blocks; districts + blocks define
   the lots; lots get realized; streets get decorated. The freeway is the
   *first cause the city organizes around*, not a reactive intruder.

2. **The pipeline is scale-independent.** It runs identically at 2.6 km or
   8 km; map size is a knob turned LAST, after the machine is proven at the
   current scale. A "small town downstream" is just this same build seeded at a
   far hotspot with a small budget.

Two design invariants that hold the whole thing together:
- **Districts are intent declared early and refined late** (coarse before
  streets so they steer street *style*; snapped to real streets after, so they
  steer building *type*).
- **Frontage is the law of parceling** — every lot faces a street; leftover
  block interior becomes a reachable court (plaza/park + alley), never a
  landlocked building.

## One city pipeline (as of 2026-08-16)

There used to be two, and it was the most confusing fact about this code. A
second generator — `shape:"city"` / `city.cpp:generateCity` — built `city.json`
and `city_arena.json` with its own zoning (`paramsForDistrict`), its own massing
(`growBuilding` on box scopes), per-building box colliders, and no architect, no
style book, no landmarks, no plazas or rowhouses. It shared only `buildDistrict`
and `extractBlocks` with the real one.

Both levels were migrated onto `shape:"road"` + `citysim.buildLots` and the
generator deleted, along with its two host bake paths (`level_loader.cpp` and the
offline tracer's `level_scene.cpp`) and its `CityModel`->`RoadNet` nav bridge.

**The live pipeline is `city_lots.cpp:growLotBuildingsOnNets`** — everything below
describes it.

One duplication remains: `level_scene.cpp` re-implements `growCityLots` for the
offline tracer. It is narrower than it looks — the JSON half is shared through
`readLotGrowParams`, and only the ~12 road-net-derived lines (hub list, coreness
anchor) fork. The fix is to hoist those, not to rewrite.

### Two "district"s that mean different things

- **`district.h` / `buildDistrict` / `DistrictNet`** — a *road generator*. Its
  "districts" are geometric sectors carved by arterials, whose only purpose is
  street density. Nothing to do with land use.
- **`architect.h` / `DistrictMap::tagAt` / `DistrictTag`** — the *zoning* map
  (Financial / Commercial / Residential / OldTown / Industrial). This is what
  decides building type.

They share a word and nothing else.

### `RoadGraph` and `RoadEntity` — ONE topology (unified 2026-08-17)

The old two-type split (`RoadGraph` + a parallel-array `RoadNet`) is gone —
`docs/road-graph-unification-plan.md` steps 3–6 landed. What ships now:

- **`RoadGraph`** (`road_network.h`) is the ONE topology: `RoadNode` /
  `RoadEdge` structs with every attribute inline and ALWAYS RESOLVED —
  `RoadEdge::width` is never 0/"use the default", classes/layers/specs/baked
  are always present — plus the roads-v2 `specs` band table. Everything reads
  and writes this: the generators, `planarize`, `applyConstraints`,
  `extractBlocks`, the lattice mesher, the nav graph, the corridor bake, and
  the editor's node/tangent/widen ops.
- **`RoadEntity`** (`road_net.h`) is the editable component:
  `{ RoadGraph graph; RoadLook look; RoadPlan plan; }`. `RoadLook` is how the
  road is drawn (default width, sidewalk, curb, markings, colour, junction
  policy); `RoadPlan` is what the generator knew (`cityHubs`, `freewayPlans`,
  `siteFootprints`). The `heightAt` terrain closure is NOT stored anywhere —
  it is one load's state, so every reader takes it as a parameter (pass what
  the loader or the editor's conform pass knows).

The JSON wire format did NOT change: `roadNetFromJson` / `roadNetToJson` are
the only code that still knows the parallel-array shape — they resolve every
fallback on the way in (a missing/zero edge width becomes the look's default,
a short class array becomes Local, a null `node_elev` becomes at-grade) and
reconstruct the sparse form on the way out. No saved level moved.

What the unification deleted: the eight parallel arrays and their fallback
semantics, the 54 defensive `.size()` guards they cost across five files, the
`roadNetEdgeWidth` accessor, and the `netGraph`/`constrainedNetGraph`
transposition layer. `navRoadGraph(road, ground)` survives because it does
real work — spline sampling + constraints + semantic classification — not a
representation change. `applyGenerateRecipe` is now an assignment: the
generator's graph BECOMES `entity.graph`, same node/edge order (the
determinism gate `tests/test_road_graph_order.cpp` pins exactly this).

## The order

**1. Terrain.** Heightfield + erosion + coastline/mountains. Derive the
buildability mask (slope / water / beach margin). Nothing builds where it says
no. The one stage with no upstream dependency.

**2. Regional hubs.** A handful of major anchor points on buildable land — the
tentpoles of the metropolis. They exist only to organize the freeway and the
districts. Few, deliberate, spread out. (At large map scale, distant hubs
become *satellite towns* the freeway connects to.)

**3. Freeway placement.** Route ONE spine near (not through) the regional hubs.
Off the beach, straight and smooth, network rules honored (no self-cross, bridge
where corridors must meet). Output: an *alignment* — centerline + vertical
profile, elevated through the dense middle, grounding at the map edges.
Confirmed policy: **one freeway is enough** for a city this size (R1.4 top-1).

**4. Corridor synthesis.** The alignment becomes real freeway geometry: deck,
median, structure (girders/bents/piers), elevated sections. Mainline only — no
ramps yet.

**5. Exits and entrances.** Stamp interchanges (feasibility-driven: only where
reachable). Each exit + on-ramp peels off the deck, clothoids down, ends at a
**ramp landing point** on the ground. These landings are the crucial output —
the fixed doors between freeway and streets. Everything after grows *toward*
them.

**6. City sizing + local hotspots.** Decide how much city to build, then scatter
a finer layer of neighborhood-seed hotspots near the corridor and hubs. Where
step 2 was tentpoles, these are individual neighborhoods.

**6b. Coarse zoning.** Give each local hotspot a district kind (downtown,
financial, residential, suburb, office park, industrial, main-street, beach
front) and paint soft regions. Happens BEFORE streets, because the district
decides how its streets look.

**7. Street growth — TWO-TIER (skeleton + template fill).** The single largest
piece of the rebuild, and the fix for malformed blocks at the root. Colonization
today is the *primary* generator, which is why blocks are garbage: it makes
irregular self-intersecting faces that can't parcel. Demote it:
- **Skeleton (colonization, sparse):** very few seeds, high influence,
  aggressive merge — lays a handful of smooth non-axis-aligned *arterials*
  giving the city its organic character line, anchored to the ramp landings
  (each landing's first road is a deliberate arterial spine, not colonization
  noise). This is what colonization is genuinely good at.
- **Fabric (template fill, NOT colonization):** arterials + freeway + coast
  bound a set of regions; each region is *filled* with a district-appropriate
  pattern template — rotated grid (downtown), loops + cul-de-sacs (suburb),
  sparse big-parcel loops (industrial), shoreline strip (beach front), radials
  (old core). Templates produce clean regular blocks *by construction*.

  Grids/radials/cul-de-sacs stop being hoped-for outcomes of colonization and
  become the fill primitives themselves.

**8. Block demarcation.** Read the finished street net as areas:
- **Enclosed blocks** — faces fully ringed by streets (what we extract today).
- **Ribbon frontage** — strips along OPEN roads not fully enclosed (main-street
  strip, edge-of-town). New: today only closed faces are found.
Inset from the road edges for sidewalk + setback.

**9. District refinement.** Snap the coarse 6b zones to real street boundaries
(a district ends at an avenue, not mid-block). This refined map drives building
selection. (Coarse @ 6b → street style; refined @ 9 → building type.)

**10. Lot subdivision (frontage-first).** Cut blocks into lots working INWARD
from the street frontage, not by area. Split the perimeter into frontage runs,
pull each lot to a district-typical depth. Every lot touches a street. Leftover
interior becomes a shared **court** (plaza / pocket park) with an alley or
walking path connecting out. Kills the current "crammed, inaccessible interior
lots" defect. Acceptance rig: the plan-only demarcation view — every lot must
front a street, or the interior is an explicitly reachable court.

**11. Lot realization (the site).** Treat each lot as a designed site chosen by
district: ground surface (concrete / exposed terrain / lawn), building envelope
+ placement, parking, driveway, yard, walkways, landscaping. Unifies today's
scattered specials (yards, plazas, park planting) into "how a lot of type X is
composed." Suburb = house set back + yard + drive; office park = low slab +
parking apron + hedges; downtown = tower + plinth to the sidewalk line. Building
geometry (shape grammar) plugs in here; skippable for the demarcation view.

**12. Street decoration.** Last, because it needs settled geometry: signals at
junctions, lamp posts along sidewalks, and finer sidewalk furniture — bike
racks, newspaper boxes, planted street trees, shrubs, painted curbs, benches.

## As built — where each stage actually lives

Traced from source 2026-08-15 on the `living_city.json` path (Pipeline B). The
designed order above is the *intent*; this table is the *implementation*.

| # | Stage | As built | Status |
| --- | --- | --- | --- |
| 1 | Terrain | `terrain.{h,cpp}`; buildability via `buildability.cpp:classifyLand` | ✅ — but the buildability gate is **metro-only**; the district kind never calls it |
| 2 | Regional hubs | `metro.cpp` hotspots → `CityHub` | ✅ metro only |
| 3–5 | Freeway, corridor, ramp landings | `alignment.cpp`, `corridor_plan.cpp`, `corridor_mesh.cpp`, `corridor_bake.cpp` | ✅ built — on the `metro` and `shape:"corridor"` paths (inactive for `district`) |
| 6 / 6b | City sizing, coarse zoning | `metro.cpp` sites + `CityHub::kind` | ✅ metro path |
| 7 | Street growth (two-tier) | `metro.cpp:buildMetro` (colonization) + `patch_fabric.cpp` (fill) — *or* `district.cpp:buildDistrict` (arterial cuts + recursive OBB bisection) | ⚠️ Partial. The district kind is template-only, not two-tier. `tensorRoads`/`radialRoads`/`gridRoads` are built and reachable from Lua via `city.layout` (`twin_cities.lua` ships both radial and tensor) but are **not wired to any `generate.kind`** |
| 8 | Block demarcation | `road_network.cpp:extractBlocks` (half-edge DCEL) + `city_lots.cpp:edgeBlocks` (ribbon/rim blocks) | ✅ both clauses built |
| 9 | **District refinement** | — | ❌ **Not built.** `DistrictMap::tagAt` zones off radial rings and hub distance, so a zone boundary can land mid-block instead of at an avenue |
| 10 | Lot subdivision (frontage-first) | `parcel.cpp:subdivideBlock` — boundary walk, mitred corners, court remainder; OBB bisection survives only as the zero-lot fallback. **Alleys (2026-08-17)**: blocks whose parcelled lots fail the 14 m frontage reach get one service lane cut into the lot pipeline's clearance graph (`RoadClass::Alley` + a draped Path strip), so orphaned rows (rim-block outer rows, measured) gain legal frontage — `citysim.alleys`/`alleyWidth` | ✅ built, both halves. Gated by `ring_parceler_fronts_every_lot_on_any_polygon`, `every_built_lot_touches_a_road`, `alleys_recover_frontage_orphaned_rim_rows` |
| 11 | **Lot realization (the site)** | `sculptYard`, `sculptPark`, `sculptPlaza`, `sculptUnderPad` | ⚠️ **The main gap.** Yards/parks/plazas exist; paved forecourts, driveways, parking aprons and per-district ground surfaces do not — so most lots read as raw terrain between sidewalk and wall |
| 12 | Street decoration | `street_furniture.cpp:planStreetFurniture` off the nav graph; protos in `street_kit.cpp` | ✅ signals, lamps |

Stages not in the original list but load-bearing in practice:

| Stage | As built |
| --- | --- |
| Road inset before parcelling | `inset(block, roadMargin)` where `roadMargin = 4.0 + sidewalk`, **then** `pushPolyClearOfRoads` (per-edge, width-aware). See b167c1d — the scalar alone under-insets wide arterials, and faces are walked over control *chords* while asphalt is meshed from the *warped spline* |
| Grading cascade | terrain → road (`roadNetConformRegions`) → block (`block_grade.cpp:gradeBlocks`, plane-fit) → pad (`makeFlattenPad`). Blocks are dilated 4.5 m first so the road conform and block grade overlap |
| Recipe selection | `architect.cpp:architectPick` — one rng roll against a weighted per-district table; seed decorrelated first so neighbouring lots don't skew. **Lua-authorable (2026-08-17)**: `archetype_book.lua` replaces a district's weights (`makeArchetypeBook`, resolved once at load against `kRecipeRegistry` — same recipe names as the style book; all-or-nothing hard-error on any invalid entry; identity weights reproduce the city bit-for-bit; landmark lots ignore it). The 56 recipe BODIES and `tagAt` zoning stay C++ deliberately |
| Landmark placement | `architect.cpp:planLandmarks` — own pass, quotas **per hub cluster**, deterministic best-lot scoring. Gated by `landmarks_are_planned_not_rolled` |
| Massing → geometry | Eight `Massing` values dispatched by inline `if` chains inside the ~1200-line `growLotBuildings`; geometry from `shape_grammar.cpp:growPlanBuilding` / `growBuilding` |
| Into the scene | Parts merged per `PartId` district-wide → `chunkMeshByCell` (250 m) → one `Renderable` per cell × part; **one** `MeshCollider` of extruded plan prisms for the whole district; HLOD via `appendLotMassBox` |

### Blocks are NOT under-built — the city is mostly not blocks

> **Correction (2026-08-16).** An earlier revision of this section, written from a
> top-down capture, said blocks were "~20% covered" and blamed the parcel grain.
> That was eyeballing, and it was wrong. `growLotBuildings` now reports coverage
> directly, and on `living_city` buildings cover **60.5% of the buildable block
> interior** (14,683 m² of 24,252 m²) — a normal, even dense, city block.
>
> What the aerial actually shows is that the blocks total 24,252 m² inside a city
> footprint of roughly 90,000 m². **~73% of the city is roads, road margins and
> leftover terrain, not blocks at all.** So the lever is the road network and the
> margins — not the parceller, and not the lot grain.
>
> Two rounds of tuning were spent on the wrong target before this was measured
> (commit d5db9a7 for the parcel theories, and the block-size knob below). The
> per-block probe is `RT_PARCEL_DEBUG=1`.

### Older analysis: why block *interiors* lose area

Not a bug list — how the current parameters compose. `growLotBuildings` prints a
full `[citylots]` diagnostic every load. **Read that line first**, because it
identifies the cause directly and the intuitive answer is wrong.

Measured on `living_city.json` (2026-08-15, main @ 258369d):

```
[citylots] 26 blocks -> 76 lots, 66 built, 7 green, 0 courts
           | rej: chance 3, sliver 1, aspect 0, fill 2, plan 0, clear 0, box 0,
             frontage 1 | 0 blocks all carriageway
```

**The bottleneck is parcelling, not rejection.** 66 of 76 lots build — an 87 %
success rate, with only 7 rejections across the whole city. The city looks empty
because the blocks only ever yielded **76 lots from 26 blocks (~2.9 per block)**.
Chasing the rejection counters would be chasing 7 lots; the missing hundreds were
never parcelled in the first place.

Why so few lots per block:

- **The district grain is far coarser than the blocks.** Financial/Industrial
  `frontWidth = 42 m`, `lotDepth = 48–60 m`, against a `block_size` of 82 m that
  insets to ~66 m of buildable interior. A 66 m edge yields ~2 slots — so a
  four-sided block produces a handful of very large lots instead of a street wall.
- **Edges shorter than `0.55 × frontWidth` are skipped outright** — under ~23 m for
  Financial/Industrial. Short edges of a block contribute nothing, which is the
  direct mechanism for "buildings clump along one side".
- **The depth floor** `0.55 × lotDepth` (26–33 m for Financial) rejects any edge
  whose inward ray cast comes back shallower, after only three 0.75× retries.
- **`0 courts`** — note the court theory does *not* apply here. The court is
  `inset(interior, lotDepth)`, and at 27–60 m depths against a 66 m interior it
  collapses to nothing on every block. Courts only eat block cores on much larger
  blocks, which is why this needs measuring per level rather than assuming.

Secondary, and real but small at this scale:

- **Silent drops emit no green at all** (lots under `minArea`, courts under 40 m²,
  interiors under 135 m²) — those render as bare terrain rather than a planted pad,
  and are invisible to every counter.
- **`rejClear` → `rejBox` compound**: a lot failing road clearance is pushed to the
  box path, which then demands 0.72 fill. Zero occurrences on `living_city`, but it
  bites on curvier nets.

## Build sequence (each stage verified in the plan-only sandbox first)

1. **Pipeline inversion** — corridor first; ramp landings as growth seeds with
   arterial spines. Street growth stubbed to **arterials only** so the skeleton
   is visible before the fabric exists. ← STAGE 1, in progress.
2. **Frontage parceling + courts/alleys** (step 10) — immediately visible in the
   demarcation view; fixes crammed lots.
3. **District-driven street templates** (step 7 fabric) — grids/loops/radials
   per zone.
4. **Ribbon lots along open roads** (step 8 second clause).
5. **LotPlan site design** (step 11), then **street decoration** (step 12).

## Map scale (deferred — the LAST knob)

Current: ~2.6 km coastal square. Target vision: ~8×4 km coastal strip with the
metropolis at one end and satellite towns strung along the freeway (gives the
freeway a *reason*: it connects places). Deferred until the inverted pipeline is
clean at current scale. Costs to remember when we do scale:
- **Terrain resolution must scale with size** or the ground goes blobby
  (~2.9 m/sample at 2.6 km / res 900; holding that at 8 km needs ~res 2700 →
  erosion cost grows ~with res²). Map size is mostly a *terrain* budget.
- **Rectangular domain support** — terrain gen + CDLOD assume a square. A
  coastal strip wants the long shoreline; either add rect-domain support (right
  answer) or use an 8 km square with the coast offset (cheap, wasteful).
