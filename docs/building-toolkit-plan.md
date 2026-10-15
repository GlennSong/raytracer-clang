# The architectural toolkit (issue #105, ADR-0174)

The plan and the technical brief, as agreed on issue #105 on 2026-10-14 and 15, with the state of each step. The
issue holds the discussion; this file is the record. No schedule: steps land when they are proven.

## Status

| step | what | state (2026-10-15) |
| --- | --- | --- |
| 0 | ADR-0174, this document, ledger rows, the `toolkit_lab` level | done: the lab is one lot, one `podium_tower`, walkable lobby to roof (the roof exit and the tiered collider landed with it) |
| 1a | `SpaceGraph` built FROM today's growers (`spaceGraphOf` over `storeyPlans`, `storeyRooms`, `CorePlan`, the facade's doors); `storeyView` equals `storeyPlans` | done: `storey_view_equals_storey_plans_every_recipe`, 372 buildings, 0 mismatches |
| 1b | `reachable` / `unreachableCells`, the gate over every recipe, the blueprint sheet, `building.space_graph` in Lua | done: `graph_gate_every_recipe` (347 of 372 passed on day one; 372 after 1c), `level_blueprints_and_graph_gate` (`RT_BLUEPRINTS=<dir>`), `procgen_script_reads_a_space_graph` |
| 1c | what 1b found | done: apartment end units (`layoutApartment`); the straight stair through a setback (`stairRuns`, transfer storeys open floors; `character_climbs_the_straight_stair_through_a_setback`); the ring on an L's narrow arm (`ringPlan` depth into the plan). 372 of 372 pass; the fix list is empty. OWED: transfer floors furnished as lounges (open floors today) |
| 2a | `Section` with lines only; `fromLot`; `Face::Flat` wraps `FaceRect` | done: `section.{h,cpp}` (`rect`, `ngon`, `fromLot`, `offset`, `withHole`, `tessellate`), `face.{h,cpp}` (`Face::ofEdge` behind `planEdgeRect`), `growSectionBuilding`, `building.section{}`; the every-recipe census identical (`test_section.cpp`) |
| 2b | arcs, splines, holes, Clipper2 ops; cylindrical faces | not started |
| 3a | `corePreset` reproduces `corePlan`; shafts as cells | not started (the adapter already makes shafts cells) |
| 3b | placement rules, zones, sky lobbies, scissor stairs | not started |
| 4a | `Mass`; `Stack` with identity transforms reproduces `massStack` | not started |
| 4b | transforms, `Support`, `emitShell` / `emitSoffit` / `emitCap`; drum and pagoda deleted | not started |
| 5 | `Complex`; `growPlanBuilding` as a one-cell complex; strip and mall as cells | not started |
| 6 | `SweepMass`, SDF words, `MeshMass`, `Envelope`, `rakeCells` | not started |
| 7 | books, variety census, neighbour rule, lot-driven sections, use bands | not started |

What the adapter reads today (`space_graph.cpp`): every storey's hall (its outline, holed by its rooms and the core),
every walled room, each core stair and hoistway as one shaft cell up the building with a door on every storey, the
roof deck where the core reaches it; doors from the room planner, passages where a room has no wall along an edge
(a lounge or a desk row open to the corridor, a flat's living room off its hall), the facade's street doors and open
vehicle bays, the straight stair hall to hall, a mall's escalators and glass lift. A hall its rooms cover whole (its
free floor inside the walls under 2 m2) is not a place to stand and is not gated. A garage is not modelled yet.

## The plan (issue #105)

## Plan: the architectural toolkit on two foundations (2026-10-15, rewritten)

**Goal.** A toolkit of grammar words expressive enough for boxy towers, art deco, Sears, a Westfield mall, curved and twisted towers, stadiums, stacked-module housing, spiral ramps and wavy implicit forms, every one of them functioning inside and out and fully traversable by stairs, escalators, lifts and ramps. Not recreations of named buildings. Today's buildings stay: every existing recipe becomes a preset over the words and comes out byte-identical. There is no schedule; the order below is dependency, not time.

### The words, plainly

- **Cell**: one space inside a building, a volume with a floor, a ceiling and a use. A room is a cell; so is an atrium, a stairwell, a lift shaft, a seating bowl, a ramp.
- **Portal**: the passage between two cells, or a cell and the outside: a door, an open passage, a stair, an escalator, a lift, a ramp, a vomitory, or a **void** (a railing edge onto an atrium: you see across, you cannot pass).
- **Floor surface**: level, sloped, stepped or helical. Level is the common case, not the only one.
- **Section**: one floor's outline. **Mass**: anything that can give its outline at a height (a **slice**) and its skin as a mesh. **Envelope**: the skin when it is not the same thing as the floors.
- **Prismatic / non-prismatic, punched / curtain / diagrid, shell, soffit**: as before.

### Four decisions

1. **A twisted or curved mass is always glass** (curtain or diagrid). Brick and stone stay on straight walls and base floors.
2. **A style ships only as "a real building, fully accessible, inside and out, base to roof."** Door, lobby, lifts and stairs to every floor, every floor planned and furnished, a stair to a real door onto a walkable roof.
3. **Every new shape is a word in the grammar; no one-off growers.** `growCylinder` and `growPagoda` (`shape_grammar.cpp:1843`, `:1916`) get re-expressed and deleted. The mall's atria and escalators, the garage's decks and ramps, the auditorium's raked rows, the core's stairs and hoistways are the other one-offs this absorbs.
4. **The traversability gate is a graph check per building plus a physics check per portal word.** Reachability from the entrance over passable portals to every habitable cell and the roof, for every building in a city; one walker test per emitter (stair, escalator, ramp, lift, vomitory) proves every instance; the full physical walk stays a lineup test.

### The diagnosis

Every limit traces to the data model: a building is **one straight-edged polygon, in one frame, copied level by level, tiers nested, one core in the middle, rooms as rectangles on one storey**. The L/U/T carve (`city_lots.cpp:5678`) is a back notch nobody sees. The round tower already runs through the plan grammar as a 16-gon (`city_lots.cpp:5940`), which is the best evidence that outlines can change without the rest breaking.

### Foundation A: the space graph (interior)

A building is a graph of **cells** joined by **portals**. A cell has a plan outline, a floor surface (level, sloped, stepped, helical), a ceiling height and a use; it may span storeys. A storey plan becomes a **derived view**, the union of the floors of the cells at that height, which is what facade bands, the LOD twin, the lot cache and the census read, so they keep working. Today's rooms (`room_plan.h` `Room`, `RoomWall`, `WallOpening`) are cells with a level floor and one storey of height; the planners keep working and the graph grows round them. Circulation elements are cells with movement surfaces. Streaming moves from a window of storeys to a neighbourhood of cells, so an atrium streams as one piece.

This absorbs: multi-storey atria and courts, double-height lobbies, sky lobbies, mezzanines, duplexes, stacked modules (Habitat 67, 56 Leonard) with roofs as terraces, seating bowls, the Guggenheim's helix, garage ramps.

### Foundation B: the exterior model

- **Section**: an outline whose edges are straight, arc or spline, with holes; generators (rect, ngon, ellipse, superellipse, rounded, lobed, star, from-lot) and ops (offset, chamfer, notch, union of frame-aligned rects, and true union / difference / intersection through Clipper2, which is already in the tree and used by three city files). Tessellates to today's `Poly2`.
- **Face**: flat, cylindrical or ruled, with the facade laid out in face coordinates; every facade mode, plus diagrid, works on all three.
- **Mass sources behind one interface, `slice(y)` and `shell()`**: **Stack** (sections with storey ranges, keyed transform curves for rotate / scale / slide / chamfer, relations nest / flush / support, `shell` between differing outlines, `cap`), **Sweep** (a section along a spline path with a rotation-minimising frame: leaning, bent, snaking, looping, ring, ribbon), **Implicit** (the engine's existing SDF kit, sphere / box / capsule / union / intersect / subtract / smooth union + polygonizer, plus new words: `extrude(section)`, `revolve`, `sweep`, warps twist / bend / taper / wave / noise), **Mesh** (any closed mesh, the artist hatch for Blender NURBS forms), **Complex** (cells in their own frames with an adjacency list of touching faces and openings, plus bridges; no booleans needed).
- **The slice bridge**: every mass is sliced at storey heights into level outlines with holes; those feed Foundation A. A slice too small to hold a room or reach a shaft is skin only (a fin, a canopy). The far LOD of any mass is the loft of its slices; the near hero is the polygonized or swept surface. Glass bands on a strange skin come from slicing at spandrel heights too.
- **Envelope**: the skin of a mass that is not the floor stack: double skins, twisted skins on straight floors, diagrid wrappers, ring roofs over bowls, canopies.

### The words over the foundations

Stack transforms and relations; cells in frames with adjacency and bridges; circulation words (shaft, bank, lobby, stair, zone, sky lobby, escalator, ramp, vomitory) placed by rules with today's core as a preset; rakes as stepped-floor cells; use bands (podium shops, offices, sky lobby, hotel, flats, plant, deck) with their own floor heights and lift zones; attachments (balconies, canopies, fire escapes, signage) on faces; caps (parapet, spire, dome, vault, sloped, aperture, flared).

### Two directions, one graph

- **Outside in**: mass first (stack, sweep, implicit, mesh, complex) → slices → declared tall and sloped cells punch through → circulation cells placed → use bands → planners cut the rest into rooms. Towers.
- **Inside out**: program first (concourse and courts, bowl and concourse, wards on wings) → the massing fits → the skin wraps. Malls, stadiums, hospitals, campuses.

### How the examples compose

| building | words |
| --- | --- |
| boxy skyscraper | rect section, nest stack, curtain faces, parapet cap, core preset. Today's preset. |
| art deco tower | rect section, tiered nest stack, punched faces with piers and courses, spire cap. |
| Sears Tower | nine rect cells on one frame, each a stack ending at its own floor, zoned banks with a sky lobby. |
| Westfield mall | a spine cell with a bend, anchor cells, pavilion cells with an arc edge, garage cells, bridges, courts as tall cells, escalator cells at the courts. |
| curved building | arc or spline edges, cylindrical faces, a dome or vault cap. |
| twisted tower | section + `rot(t)`; keyed curve for a varying rate; `slide(t)` for an off-centre pivot; skin-only twist = an envelope; spiralling wings = union section + `rot(t)`. |
| stadium | ring section with a hole, rake cells, concourse cells, vomitory portals, ring-cantilever envelope. |
| Guggenheim | one helical cell, a lift cell, a dome cap. |
| Habitat 67 | boxes as 3D cells with support relations, roofs as terraces, elevated street cells, cores. |
| wavy implicit tower | `sdf.extrude(section)` with wave and twist warps, a subtracted void, sliced into floors, banded glass on the skin. |

### Order by dependency

| step | adds | gate |
| --- | --- | --- |
| 0 | the plan in `docs/`, ADR, ledger rows, a `toolkit_lab` level with `watch_scripts` | docs exist; the lineup loads |
| 1 | **Space graph** + `reachable` + the graph gate over the lineup; today's rooms and core mapped onto it | the gate is red where today's towers fail (roof door, 16-gon interiors), then green after fixes |
| 2 | **Section + Face** with straight edges first | mesh-hash census + `rt_bake --sections` identical; then arcs, splines, holes, booleans for new recipes |
| 3 | **Circulation words**, core as preset, real roof door, zones, sky lobby | every existing recipe passes the gate; a zoned tower walks |
| 4 | **Mass interface + slice bridge + Stack** transforms / relations / shell / cap; drum and pagoda deleted | twisted ellipse, Gherkin, overhang stack walked |
| 5 | **Complex** cells in frames, adjacency, bridges | Sears, a spine mall, rowhouses unchanged, all walked |
| 6 | **Sweep, Implicit warps, Mesh, Envelope, rakes** | Guggenheim, a bowl, a wavy tower, a double skin walked |
| 7 | books, variety census, neighbour rule, lot-driven section choice, use bands | census targets held by tests |

Each step re-bakes (`kParamsVersion`, `kLotsBuildTag`). The recipe studio (#104) comes online after step 5. Nothing is excluded: NURBS arrive as spline curves and the mesh hatch, booleans as Clipper2 section ops and SDF ops, sweeps as a mass source. No mesh-CSG library is needed because a 3D boolean's floors are 2D booleans of its slices and its skin is the SDF's.

### Proof of decision 3

`growCylinder` → `section(ngon 24)` + curtain `shell` + `cap`; `growPagoda` → scaled tiers + a flared cap per tier + a finial; both deleted. The mesh-hash census (`test_building_lod.cpp:229`) and the bake-section diff show the cities unchanged except where a recipe asks for a new word.

The technical brief is the next comment.


## The technical brief (issue #105)


Rules obeyed: C++17, one header + one `.cpp` per module, no new dependencies (Clipper2 and the SDF kit are already in the tree; an ADR records their entry into the building grammar), every word in C++ and Lua together (AGENTS.md, ADR-0042), a ledger row before each module, `editor_app` as the acceptance gate, and byte-identity for every existing recipe at every step, proven by the mesh-hash census (`tests/test_building_lod.cpp:229`) and `rt_bake --sections` diffs on `tower_lineup` and `metro_v2_test`.

### 1. Module map (`src/engine/procgen/city/` unless noted)

| module | holds | replaces or wraps |
| --- | --- | --- |
| `space_graph.h/.cpp` | `FloorSurface`, `Cell`, `Portal`, `SpaceGraph`, `storeyView`, `reachable` | `Room` / `RoomWall` / `WallOpening` / `DoorSpec`; the mall's atria holes; the garage's decks and ramps |
| `section.h/.cpp` | `Edge` (straight / arc / spline), `Section`, generators, ops, booleans via Clipper2, `tessellate` | `Poly2` upstream of the storey view; `offsetPlan`, `tierBox`, the lot-pass carve, the mall's hand-built union |
| `face.h/.cpp` | `Face` (Flat, Cylindrical, Ruled), `facadeLayout` over a `Face` | `FaceRect` / `planEdgeRect` (`shape_grammar.cpp:2421`) |
| `mass.h/.cpp` | the `Mass` interface (`slice`, `shell`, `bounds`), `MeshMass` (slicing a closed mesh), `loftSlices` | new |
| `mass_stack.h/.cpp` | `Curve`, `Transform`, `Relation`, `StackEntry`, `Stack` as a `Mass`; `emitShell`, `emitSoffit`, `emitCap` | `massStack` / `storeyPlans` (`shape_grammar.cpp:3430`, `:3578`), `tierInsetOk` / `tierNestedIn`, `growCylinder` / `growPagoda` |
| `sweep.h/.cpp` | `SweepMass`: section along a spline path, rotation-minimising frame | new |
| `procgen/sdf.h/.cpp` (existing) | `sdfExtrude(section, h)`, `sdfRevolve`, `sdfSweep`, warps `twist` / `bend` / `taper` / `wave` / `displace`, `sliceSdf(y)` (marching squares), `ImplicitMass` | extends the existing kit |
| `cells.h/.cpp` | `CellFrame`, `Adjacency`, `Bridge`, `Complex` as a `Mass` of masses | `partyWalls` / `partyN` / `partyAt`, `exposedRuns` generalised |
| `circulation.h/.cpp` | `Shaft`, `Bank`, `Zone`, `CirculationPlan`, `corePreset`, placement rules | `core_plan.h` (the core stays as the preset) |
| `envelope.h/.cpp` | `Envelope { Mass skin; Kind }` | the drum's tubes; double skin, ring roof, canopy new |
| `rake.h/.cpp` | `rakeCells(section ring, rows, rise, run, aisles, vomitories)` | the auditorium's raked rows (`room_plan.cpp:1386`) |

`shape_grammar.h` stays the facade. `growPlanBuilding(plan, params, baseY, detail)` keeps its signature as a one-cell preset; `growComplex(const Complex&, …)` is the general entry; `growBuilding(scope, params)` becomes `growCell` on a rect section.

### 2. Types

```cpp
// space_graph.h
struct FloorSurface { enum class Kind { Level, Sloped, Stepped, Helical } kind;
                      Real y0;                 // at the cell's origin
                      Vec2 grade;              // Sloped: dy per metre in plan (Helical: along the arc)
                      Real rise = 0, run = 0;  // Stepped: one row
                      Real yAt(const Vec2& p) const; };
struct Cell   { Poly2 plan; std::vector<Poly2> holes; FloorSurface floor; Real height;
                RoomKind use; int k0, k1;      // the storeys it spans (derived)
                int mass = 0; };               // which mass / complex cell it lives in
struct Portal { int a, b;                      // cells; b < 0 = outside
                enum class Kind { Door, Passage, Void, Stair, Escalator, Lift, Ramp, Vomitory } kind;
                Vec2 p0, p1; Real sill, head; };
struct SpaceGraph { std::vector<Cell> cells; std::vector<Portal> portals; int entrance; int roof; };
std::vector<StoreyPlan> storeyView(const SpaceGraph&, const BuildingParams&);   // the derived view
bool reachable(const SpaceGraph&, int from, int to, WalkerCaps caps);          // BFS over passable portals
std::vector<int> unreachableCells(const SpaceGraph&, WalkerCaps);
```

```cpp
// section.h
struct Edge    { enum class Kind { Line, Arc, Spline } kind; Vec2 a, b; Real bulge = 0; std::vector<Vec2> ctrl; };
struct Section { std::vector<Edge> outer; std::vector<std::vector<Edge>> holes;
                 Poly2 tessellate(Real tol = 0.25) const; std::vector<Poly2> holesTessellated(Real tol) const; };
Section rect(Real w, Real d); Section ngon(int n, Real r); Section ellipse(Real a, Real b);
Section superellipse(Real a, Real b, Real n); Section rounded(const Section&, Real r);
Section lobed(int n, Real r, Real lobe); Section star(int n, Real r1, Real r2); Section fromLot(const Poly2&);
Section offset(const Section&, Real d); Section chamfer(const Section&, int corner, Real c);
Section notch(const Section&, int edge, Real w, Real d); Section withHole(Section, const Section&);
Section unionRects(const SiteFrame&, const std::vector<Rect2>&);               // rectilinear, no Clipper
Section unite(const Section&, const Section&); Section subtract(const Section&, const Section&);
Section intersect(const Section&, const Section&);                             // Clipper2; chord runs re-tagged as arcs
```

```cpp
// face.h
struct Face { enum class Kind { Flat, Cylindrical, Ruled } kind;
              Vec3 at(Real u, Real v) const; Vec3 n(Real u, Real v) const; Real width, height;
              static Face flat(const Vec3& bl, const Vec3& h, Real w, Real hgt);
              static Face cylindrical(const Edge&, Real y0, Real h);
              static Face ruled(const Edge& lower, const Edge& upper, Real y0, Real h); };
FacadeLayout facadeLayout(const Face&, FacadeMode, const BuildingParams&);   // bays in (u, v); unchanged rules
```

```cpp
// mass.h
struct Slice { std::vector<Section> pieces; };                                 // disjoint outlines with holes
struct Mass  { virtual Slice slice(Real y) const = 0;
               virtual RenderMesh shell(int lod) const = 0;                    // lod 0 = loft of slices, 1 = native
               virtual Aabb bounds() const = 0; };
std::vector<StoreyPlan> sliceStoreys(const Mass&, const BuildingParams&);     // the bridge into Foundation A
```

```cpp
// mass_stack.h
struct Curve      { std::vector<std::pair<Real, Real>> keys; Real at(Real t) const; };   // or a spline
struct Transform  { Curve rot, scaleU, scaleV, slideU, slideV, chamfer; Vec2 pivot; };
enum class Relation { Nest, Flush, Support };
struct StackEntry { Section section; int k0, k1; Relation rel; Transform xf; Real minOverlap = 0.6, maxOverhang = 6.0; };
struct Cap        { enum class Kind { Parapet, Flat, Spire, Dome, Vault, Sloped, Aperture, Flared } kind; /* numbers */ };
struct Stack : Mass { std::vector<StackEntry> entries; Cap cap; /* slice = transform the entry's section */ };
```

```cpp
// cells.h
struct CellFrame { SiteFrame frame; std::shared_ptr<Mass> mass; BuildingParams params; std::string role; };
struct Opening   { Real u0, u1; int k0, k1; };
struct Adjacency { int a, faceA, b, faceB; std::vector<Opening> openings; };
struct Bridge    { int a, b; int storey; Real width; };
struct Complex : Mass { std::vector<CellFrame> cells; std::vector<Adjacency> adj; std::vector<Bridge> bridges; };
```

```cpp
// circulation.h
struct Shaft { enum class Kind { Hoistway, Stair, Service, Scenic, Escalator, Ramp, Vomitory } kind;
               SiteFrame frame; Real w, d; int k0, k1; Real doorU, doorW; int cell; };
struct Bank  { std::vector<int> shafts; int lobbyFace; };
struct Zone  { int bank; int k0, k1; };
struct CirculationPlan { std::vector<Shaft> shafts; std::vector<Bank> banks; std::vector<Zone> zones; int skyLobby = -1;
                         void addTo(SpaceGraph&) const; };                     // shafts become cells, doors become portals
CirculationPlan corePreset(const Poly2&, const std::vector<MassTier>&, const BuildingParams&, std::size_t entranceEdge);
CirculationPlan place(const Mass&, const std::vector<StoreyPlan>&, const CirculationSpec&);   // the rules
```

`BuildingParams` carries **descriptors**, never geometry: `SectionSpec`, `MassSpec` (stack / sweep / implicit / mesh / complex), `CirculationSpec`, `EnvelopeSpec`, `UseBands`, `DeclaredCells` (tall and sloped cells the recipe names). The lot cache serialises them (`kParamsVersion` 13 → 14, `lot_cache.cpp:20`); the Lua reader (`readBuildingParamsOnto`, `procgen_bindings.cpp:1030`) reads them as tables. Geometry is derived at grow time, so the cache key stays the params.

### 3. The pipeline

**Outside in** (`growCell` / `growComplex`):

1. The `Mass` from its spec, fitted in the cell frame's rectangle (a stack's base section the way `tierBox` fits the OBB today).
2. `sliceStoreys(mass)` → level outlines with holes per storey, with the edge kinds kept where the mass is a stack (so faces know which edges are arcs).
3. The `SpaceGraph` starts as one cell per slice piece per storey; declared tall and sloped cells replace the cells they span (an atrium through five storeys is one cell; the slices get its hole).
4. `place(circulation)` adds shaft cells and their portals; `reachable` is checked now, before any mesh.
5. Use bands assign `RoomKind` per layer; the planners partition each remaining cell into rooms given the obstacles (shafts, tall cells) and the outline, arcs included (the perimeter planner for non-rectangles).
6. Exterior: per storey, per edge of the storey view, a `Face`: equal outlines above and below → `Flat` (today's rectangle, bit for bit); arc edge → `Cylindrical`; differing outlines → `Ruled` via `emitShell`. Relations give slab + parapet or `emitSoffit`; `emitCap` closes the top; the envelope, when present, is drawn from its own mass and the storey faces become inner glass.
7. Interior meshes per cell: floor surface (level slab, sloped plane, stepped rows, helical strip), walls on cell boundaries, portals cut; the collider is the floor surfaces plus walls directly, so a ramp's collider is its surface. Shaft walls for every storey ride in the exterior mesh as now (`emitCoreShaftWalls`).
8. Proxy: the base slice's prism; the far LOD: `shell(0)`, the loft of slices.

**Inside out**: a program (`ProgramSpec`: cells with sizes, uses and required portals) → a placement that builds the `SpaceGraph` first (concourse and courts, bowl and concourse, wards on wings) → the `Complex` massing fitted round it → step 6 onward.

**Consumers**: `growInterior` takes the graph, not a plan and a core; `BuildingInteriorSystem` streams a neighbourhood of cells (by portal distance from the player's cell) instead of a storey window (`building_interior_system.h:66`); `ElevatorSystem` reads banks and zones, several per building; `ladders.cpp` reads the storey view; `appendBuildingPrism` (`level_loader.cpp:5973`) runs per complex cell, with the loft shell replacing the prism where slices differ; `SkylineCensus` counts the site rectangle and gains the variety fields; `lot_cache` stores the specs.

### 4. Migration, by dependency, each step proven

| step | change | proof |
| --- | --- | --- |
| 0 | `docs/building-toolkit-plan.md`, ADR-0174, ledger rows, `toolkit_lab` level | docs; the lineup loads in `editor_app` |
| 1a | `SpaceGraph` built FROM today's `RoomPlan` + `CorePlan` + `DoorSpec` (an adapter, no grower changes); `storeyView` equals `storeyPlans` | identical meshes; a test asserts the view equals the old storey plans for every recipe |
| 1b | `reachable` + `unreachableCells` + the graph gate test over every recipe in the lineup and `metro_v2_test` | red where today's towers fail; the list is the fix list |
| 1c | the roof door, the 16-gon interiors, whatever 1b found | the gate goes green |
| 2a | `Section` with lines only; `fromLot` wraps the plan; `Face::Flat` wraps `FaceRect` | mesh-hash census + `rt_bake --sections`: identical |
| 2b | arcs, splines, holes, Clipper2 ops; `Cylindrical` faces; new lineup entries only | identical for old recipes; audit + gate for the new |
| 3a | `corePreset` reproduces `corePlan`; shafts as cells via `addTo` | identical; `test_core_plan` passes against the preset |
| 3b | placement rules, zones, sky lobby, scissor stairs | a zoned tower passes the gate; lift panel lists served floors |
| 4a | `Mass` interface; `Stack` with identity transforms reproduces `massStack` | identical |
| 4b | transforms, `Support`, `emitShell`, `emitSoffit`, `emitCap`; drum and pagoda deleted | identical except the two, compared by hand once |
| 5 | `Complex`; `growPlanBuilding` = a one-cell complex; rowhouse strip and mall as cells | identical for both; Sears and the spine mall new |
| 6 | `SweepMass`, SDF words + `sliceSdf` + `ImplicitMass`, `MeshMass`, `Envelope`, `rakeCells` | new recipes; the Guggenheim, a bowl, a wavy tower, a double skin pass the gate |
| 7 | books, variety census, neighbour rule, lot-driven sections, use bands | census targets as tests |

Each step bumps `kLotsBuildTag` (`lots_producer.cpp:22`) and re-bakes; steps with new params bump `kParamsVersion`. The first act of every refactor step is to capture the baseline: run `building_mesh_hash_census` and `rt_bake --sections tower_lineup metro_v2_test` into a file, then diff after.

### 5. Tests

Kept: `mesh_audit_every_building_recipe`, `building_mesh_hash_census`, `grown_buildings_stand_on_rectilinear_footprints` (counting the site), `test_core_plan` (against the preset), `test_room_plan`, `test_elevator`, the physics walks (fire-escape climb, garage drive-to-roof, cab ride), `RT_LEVELS=metro_v2_test level_tests` with `RT_NOCACHE=1`.

New: `storey_view_equals_storey_plans_every_recipe` (the step-1 guard); `graph_gate_every_recipe` (decision 4); one walker test per portal emitter (`walk_stair_cell`, `walk_escalator_cell`, `walk_ramp_cell`, `ride_lift_cell`, `walk_vomitory`); `section_tessellation_keeps_corner_correspondence`; `section_boolean_retags_arcs`; `face_layout_same_bays_flat_and_arc`; `shell_is_flat_face_when_outlines_equal` (the byte-identity guard, forever); `stack_support_rule`; `slice_bridge_matches_stack_storeys`; `implicit_slice_is_closed`; `complex_shared_faces_not_drawn`; `rake_rows_are_level_rings`; `walk_lineup_base_to_roof`; `skyline_variety_targets`.

### 6. Lua, ledger, docs

Words bound as they land, each wrapping the C++ builder: `building.section{}`, `building.mass{}` (stack / sweep / implicit / mesh / complex), `building.cells{}`, `building.circulation{}`, `building.envelope{}`, `building.program{}`, and the recipe table that composes them (the #104 surface); SDF words `sdf.extrude`, `sdf.revolve`, `sdf.sweep`, `sdf.warp`, `sdf.slice`. Ledger rows before each module: SpaceGraph vs `RoomPlan` / `CorePlan` / mall atria / garage ramps; Section vs `Poly2` / `offsetPlan` / `tierBox` / the carve / Clipper2 in `road_offset.h`; Face vs `FaceRect`; Mass vs `massStack`; Implicit vs `sdf.h` / `polygonizeSdf`; Cells vs `partyWalls` / the mall union; Circulation vs `CorePlan`; Envelope vs `growCylinder`; Rake vs `RoomKind::Auditorium`. Docs: ADR-0174 (the two foundations; SDF and Clipper2 enter the grammar), `docs/building-toolkit-plan.md`, `docs/capabilities.md` rows, a `toolkit_lab` level with `watch_scripts`.

### 7. Performance and risks

Lua at the recipe tier only; slicing, lofting and polygonizing in C++ at grow time; an SDF slice is one marching-squares pass per storey; the polygonized skin is the near hero only, the far LOD is the loft of slices; a 50-storey, 24-edge twister is 1 200 shell quads; a complex costs the sum of its cells. Risks, by size: the planners assume one core rectangle on one storey (`room_plan.cpp`, 2 501 lines), so the obstacle-and-volume partition is the long pole; streaming by cell neighbourhood changes the interior system's residency model; `Poly2` runs through 16 000 lines, so `Section` stays upstream of the storey view and everything downstream keeps the tessellated `Poly2`; Clipper drops arc tags, so the re-tag pass must land with the booleans; the footprint census must count sites before arcs reach a city.

