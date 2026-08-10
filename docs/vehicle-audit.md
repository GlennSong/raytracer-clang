# Vehicle unification — the audit

## Status (updated as work lands)

**Done since the audit was written:**

- **The promoted car is the real car.** It draws the level's Lua body (minus its
  baked wheels) and takes its Jolt wheels and lamps from the recipe's own
  layout and markers.
- **Doubled taillights** — fixed. A promoted car carried no lamp markers, so
  `VehicleSystem` lit four lenses at guessed chassis corners. Those coincided
  with the retired box car's baked boxes and so were invisible; against a real
  body they showed as a second pair beyond the tail.
- **Ride height / top-heavy** — fixed. `VehicleWheel::position` is the
  suspension ATTACHMENT point; feeding it a drawn wheel centre parked every
  commandeered car **0.375 m** in the air (measured). The resting drop is
  `clamp(max - 0.075, min, max)` and is mass-independent, so it is corrected
  exactly. `comOffsetY` is now derived per body instead of a flat -0.4.
- **Catalogue unification (audit item 1 / plan step 1)** — landed.
  `vehicle_classes.lua` is the size; the hardcoded per-slot box is gone from the
  authoring path and `CitySim::setFleet` adopts each recipe's own dimensions.
  **This fixed the squished wheels** (the fit was non-uniform: pickup 10% oval,
  hatchback 8%). Wheels now measure exactly `2r` on both axes.
- **The Makefile builds again.** Neither `make` nor `make test` linked at HEAD —
  the source lists had rotted (`make test` missing the P8 modules
  `city_footprint` / `arterial_skeleton` / `patch_fabric` / `city_planner`;
  `make` missing `script_assets`, `script_modules`, `nav_graph`, `pathfind`).
- **`make test` carries Lua** (audit blocker B). It now compiles with
  `RT_ENABLE_SCRIPTING`, links Lua, and runs the scripting tests — 921 → 998
  cases, reading the real shipped `assets/scripts/*.lua`. The C++ box fallback is
  no longer what this build exercises, which is the precondition for deleting
  it. Also fixed a trap in that target: it compiles its whole source list in one
  command with no `.o` files, so ADDING a test could not make the binary out of
  date — newly listed tests silently never ran. It now depends on the Makefile.

- **Fleet LENGTH comes from the asset.** `vehicle.fleet`'s own length decides how
  many variants a city has; a thirteenth slot is no longer ignored, and a shorter
  fleet no longer fills the gap with boxes.
- **THE BOX PATH IS GONE.** `fleetCarMesh`, `buildCarMesh`, `styleForType`,
  `kCarColors`, `carVariantCount`, `fleetBodySize` and `defaultLampMarkers` are
  deleted — 139 lines out of `city_meshes`. A level with no vehicle catalogue now
  draws NO cars, which is what Glenn's ruling asks for: absence is a data
  decision, and box cars never were legitimate. `freeway_lab`, `freeway_variants`
  and `rules_lab` gained `"vehicles": "vehicles.lua"`; `coast_city` already said
  `cars: 0` and meant it.

**Still open, in order:** the framerate, still unmeasured (`tools/piedmont_perf.sh`
plus the downtown-budget plan in `vehicle-unification-plan.md`); LOD by
decimation, which needs a decision because no mesh simplifier exists; and the
question this round surfaced but did not answer — whether
`-DRT_ENABLE_SCRIPTING=OFF` should still be a supported configuration, since
citysim can no longer draw a car in it. It compiles and the sim runs; it just has
no traffic.

---


The audit `docs/vehicle-unification-plan.md` requires before anything is
committed. Three questions, answered against the tree at `a502bd9`. Every claim
below is grounded in a file and line, not in what the plan assumed.

**Headline:** the deletion list is short and mostly clean, but it is BLOCKED by
three things the plan did not know about — one of which is a live gameplay bug
(the car the player commandeers is still a box), and one of which changes the
shape of step 3 (there is no mesh reduction in this engine at all, and the
fleet's "mid" LOD is currently a no-op).

---

## 1. Every reference to the box path

### Deletable, verified — nothing else reaches them

| Symbol | Defined | Only reached from |
|---|---|---|
| `buildCarMesh(int style, Vec3 color, Vec3 size, bool)` | [city_meshes.cpp:100](src/apps/citysim/city_meshes.cpp:100), decl [:29](src/apps/citysim/city_meshes.h:29) | `fleetCarMesh` ([city_meshes.cpp:189](src/apps/citysim/city_meshes.cpp:189)) |
| `styleForType(VehicleType)` | [city_meshes.cpp:167](src/apps/citysim/city_meshes.cpp:167), decl [:33](src/apps/citysim/city_meshes.h:33) | `fleetCarMesh` only |
| `kCarColors` / `kNumCarVariants` | [city_meshes.cpp:73](src/apps/citysim/city_meshes.cpp:73) | `fleetCarMesh` (paint) — but see `carVariantCount` below |
| `defaultLampMarkers(Vec3)` | [city_render.cpp:67](src/apps/citysim/city_render.cpp:67) | the `lights.empty()` fallback, [city_render.cpp:414](src/apps/citysim/city_render.cpp:414) |
| `addColoredBox` + the `parts` branch of the reader | [vehicle_body.cpp:47](src/apps/citysim/scripting/vehicle_body.cpp:47) and ~:115–150 | no shipping asset — see the correction below |

`buildCarMesh`/`styleForType` are private to the box path. Note the name
collides with `engine::buildCarMesh` in [car_mesh.cpp:136](src/engine/procgen/vehicle/car_mesh.cpp:136)
— the *real* generator; different namespace, don't grep them together.

### Blocked — a live call site with no Lua alternative

`fleetCarMesh` ([city_meshes.cpp:185](src/apps/citysim/city_meshes.cpp:185)) has
three callers, and only one of them is the fallback the plan describes:

1. [city_render.cpp:392](src/apps/citysim/city_render.cpp:392) — `if (!scripted)`,
   the documented fallback. Fine.
2. **[city_vehicles.cpp:120](src/apps/citysim/city_vehicles.cpp:120) — the
   PROMOTED car. Unconditional. No `#ifdef`, no scripted branch, no fallback
   semantics.**
3. [test_city_render.cpp:211](tests/test_city_render.cpp:211) — pins the
   with/without-wheels contract.

**(2) is a bug, not just an obstacle.** When the player commandeers an ambient
car (ADR-0062), `CityVehicleSystem` builds the entity's `Renderable` from
`fleetCarMesh` and `VehicleSystem::createVehicles`
([vehicle_system.cpp:97](src/engine/systems/vehicle_system.cpp:97)) only adds the
Jolt body and wheel entities — it never touches the `Renderable`. So the box
mesh survives to the screen. The city around the player is `mesh.car`; the car
in the player's hands is the retired box. Same site, [city_vehicles.cpp:31](src/apps/citysim/city_vehicles.cpp:31):
`configFromBody` derives the Jolt chassis, mass, torque and wheels from `kFleet`
rather than the class package, so the promoted car handles by a second set of
numbers too. This is the most visible instance of exactly what the plan is
trying to fix, and it should be fixed first.

### Keep — misfiled as box geometry

`fleetBodySize` ([city_meshes.cpp:178](src/apps/citysim/city_meshes.cpp:178)) is
**not** box geometry; it is the dimension lookup. It feeds
`carGroupHalfExtents` ([city_render.cpp:1045](src/apps/citysim/city_render.cpp:1045)),
which feeds the K-tier kinematic proxies ([city_physics.cpp:125](src/apps/citysim/city_physics.cpp:125))
and parked-car placement ([city_render.cpp:1267](src/apps/citysim/city_render.cpp:1267)).
Re-source it from the catalogue; do not delete it.

### Correction to the plan

The plan lists "the box `parts` wheels still in `vehicles.lua`". **They are
already gone.** `fleet_car` returns `{ body = body, lights = lights }` and
nothing else ([vehicles.lua:245](assets/scripts/vehicles.lua:245)); the wheels
are `mesh.cylinder` rolled onto the lateral axis and merged into `body`
([vehicles.lua:215–224](assets/scripts/vehicles.lua:215)). What is stale is the
*comment* at [vehicles.lua:162–164](assets/scripts/vehicles.lua:162), which still
says "`parts` carries only the WHEELS now". The reader's `parts` branch is
therefore dead for every shipping asset, exercised only by the malformed-recipe
test ([test_vehicle_body.cpp:103](tests/test_vehicle_body.cpp:103)) — deletable,
but the reader's "`parts` required when there is no `body`" contract and that
test have to change in the same commit.

### Blockers on `kFleet` itself (plan step 1)

`kFleet` ([city_sim.cpp:36](src/apps/citysim/city_sim.cpp:36)) is read by
`city_sim.cpp:444` (SimVehicle dimensions), `city_meshes.cpp:179/188`,
`city_vehicles.cpp:108`, and four tests. Deleting it needs two things the tree
does not have yet:

- **`make test` compiles citysim with no Lua.** `SCRIPT_FLAGS` is scoped to
  `$(TARGET)` ([Makefile:331](Makefile:331)); the `test` target
  ([Makefile:350](Makefile:350)) gets only `DEBUG_FLAGS`, and
  `TEST_ENGINE_SRCS` includes `city_sim.cpp`, `city_meshes.cpp` and
  `city_render.cpp`. So the C++ fallback is what `make test` actually
  exercises. Per Glenn's ruling ("scripting is engine infrastructure, not a
  build option") the fix is to give the test target Lua, not to keep the
  fallback — but that is a prerequisite commit, not a side effect.
- **Fleet length is decided in C++, not the catalogue.** `city_render` loops
  `v < carVariantCount()` = 12, from `kCarColors`
  ([city_render.cpp:368](src/apps/citysim/city_render.cpp:368)). A 13th slot in
  `vehicles.lua` is never built; a shorter Lua fleet makes `loadFleetCarBody`
  return "slot out of range" and each missing slot silently falls back to a box.
  Until the count comes from the Lua fleet, "a new vehicle is a catalogue entry,
  not a code change" is not true.

### The distortion, quantified

The plan says the fit-scaling hack "DISTORTS the mesh to match a hardcoded box".
It does, and non-uniformly. Class dimensions from `vehicle_classes.dims`
([vehicle_classes.lua:273](assets/scripts/vehicle_classes.lua:273)) against the
slot box passed to `fleet_car`:

| class | class L/W/H | slot box L/W/H | scale L/W/H |
|---|---|---|---|
| sedan | 4.59 / 1.82 / 1.45 | 4.20 / 1.80 / 1.30 | 0.914 / 0.989 / **0.896** |
| hatchback | 3.95 / 1.78 / 1.48 | 4.20 / 1.82 / 1.45 | 1.064 / 1.022 / 0.978 |
| jeep | 4.40 / 1.86 / 1.75 | 4.60 / 1.95 / 1.70 | 1.045 / 1.048 / 0.973 |
| pickup | 5.70 / 1.96 / 1.95 | 5.20 / 1.95 / 1.60 | 0.912 / 0.995 / **0.821** |
| van | 5.20 / 1.94 / 1.90 | 5.40 / 2.00 / 2.10 | 1.039 / 1.031 / **1.107** |
| box truck | 6.60 / 2.16 / 2.90 | 6.60 / 2.40 / 2.80 | 1.000 / 1.111 / 0.966 |

The pickup is squashed 18% in height, the van stretched 11%. (The code fits
against the merged shell's actual `half_extent`, so the shipped factors differ
slightly from these — but the mismatch and its sign are real.) The numbers are
authored three times: `kFleet` in C++, `kCarColors` in C++, and the literal
`W, H, L` arguments at [vehicles.lua:249](assets/scripts/vehicles.lua:249).

---

## 2. What mesh reduction exists

**None.** This is a new component, and per the plan Glenn should hear it now.

- `MeshBuilder` ([mesh_builder.h](src/engine/mesh_builder.h)) offers
  `append` / `appendTransformed` / `transform` / `merged` / `recomputeNormals` /
  `emitTri` / `emitQuad` / `gridIndices` / `emitLattice` /
  `generatePlanarUVs` / `bakeHeightColor`. No decimation, no simplification, no
  vertex welding.
- The only simplification in the tree is **polyline** Douglas–Peucker
  ([terrain_field.cpp:402](src/engine/procgen/terrain_field.cpp:402),
  `simplifyClosed` at [city_footprint.cpp:322](src/engine/procgen/city/city_footprint.cpp:322)).
  That is 2D curve decimation for roads and footprints; nothing there reduces a
  triangle mesh.
- `AssetManager` documents the LOD-chain seam as unbuilt:
  "`std::vector<MeshHandle> lods` lands later without touching consumers"
  ([asset_manager.h:54](src/engine/asset_manager.h:54)). There is no runtime LOD
  selection for meshes either.

**And the LOD the fleet already asks for does nothing.** `CarLod` has four
values ([car_mesh.h:40](src/engine/procgen/vehicle/car_mesh.h:40)) but is read at
exactly one place in the generator:

```
const bool cutApertures = (p.lod != CarLod::Low && p.lod != CarLod::Proxy);
```
— [car_mesh.cpp:290](src/engine/procgen/vehicle/car_mesh.cpp:290)

`High` and `Mid` both take the same branch, and `p.lod` is used nowhere else. So
`FLEET_LOD = "mid"` ([vehicles.lua:178](assets/scripts/vehicles.lua:178)) builds
byte-identical geometry to the hero car — the comment there ("the fleet runs a
cheaper LOD than the hero car") is not true today. That is part of why ambient
traffic costs ~1900 tris a car.

So step 3 of the plan ("LOD by decimation") needs, in order: a mesh simplifier
(quadric error metrics or edge-collapse over `RenderMesh`), an LOD chain in
`AssetManager`, and distance selection in the render path. `CarLod` is a
re-authoring switch and should be retired rather than extended — it is what
Glenn's "LODs must be DECIMATIONS, never a second re-authoring" rules out.

---

## 3. What procgen is hardcoded in C++ that should be a recipe

The standing criticism is correct and larger than the plan states: **the entire
P8 pipeline is unreachable from Lua.** It is driven only by level JSON.

- The Lua `city` table is: `layout`, `lots`, `road_mesh`, `stroke`, `union`,
  `weld`, `solid`, `resolve`, `deck`, `roadbed`, `lane_markings`
  ([procgen_bindings.cpp:2982](src/engine/scripting/procgen_bindings.cpp:2982)).
- `city.layout` reaches only `grid` / `radial` / `tensor` — the pre-metro
  generators (`buildLayoutGraph`, [procgen_bindings.cpp:1221](src/engine/scripting/procgen_bindings.cpp:1221)).
  Nothing in Lua can call `buildMetro`.
- The P8 pipeline is configured by **56 `g.value(...)` JSON keys** into a
  49-field `MetroParams` ([road_net.cpp:2046–2200](src/engine/procgen/city/road_net.cpp:2046),
  [metro.h](src/engine/procgen/city/metro.h)). That is the recipe surface, and it
  is JSON-only.
- Modules with **zero** mention in the bindings: `arterial_skeleton`,
  `city_footprint`, `patch_fabric`, `metro`, `architect`, `district`,
  `road_constraints`, `road_lattice`, `road_offset`, `road_rules`,
  `road_semantics`, `road_spec`, `street_furniture`, `structure_set`,
  `surface_field`, `block_grade`, `buildability`, `alignment`,
  `corridor_{plan,mesh,bake}`, `water_mesh`.

### The string-dispatched enums, located

| Choice | Site |
|---|---|
| `fabric: "mix" / "chords" / "bisect" / "court"` | [metro.cpp:1236](src/engine/procgen/city/metro.cpp:1236), :1243, :1252, :1254, :1284 |
| `skeleton: "footprint"` | [metro.cpp:322](src/engine/procgen/city/metro.cpp:322), :470, :990, :1006, :1014 |
| `stop_after: "footprint" / "skeleton"` | [metro.cpp:353](src/engine/procgen/city/metro.cpp:353), :1006 |
| `kind: "metro"` | [road_net.cpp:2065](src/engine/procgen/city/road_net.cpp:2065), :2195, :2366 |
| planner layer `"footprint"` | [city_planner.cpp:232](src/engine/city_planner.cpp:232) |

### Primitives that need bindings to make those recipes real

Each already exists in C++; the work is a binding plus a value type crossing the
boundary, not new geometry.

| Proposed binding | Backed by |
|---|---|
| `city.footprint{}` | `city_footprint.h` — buildable flood fill, morphological opening, contour walk, DP simplify, Chaikin smooth, inscribed-disc fallback |
| `city.skeleton{}` | `arterial_skeleton.h` — rim + spine + recursive bisection, global junction registry |
| `city.fabric{}` (chords / bisect / court) | `patch_fabric.h` — `fabricCoonsPatch`, chord straightening, ring court, `insetRingArcs` |
| polygon bisection | `arterial_skeleton.cpp` recursive bisection; `parcel.cpp` recursive-OBB |
| line displacement | `arterial_skeleton.cpp` midpoint-displacement cuts |
| chain stationing | `patch_fabric.h` station chains |
| junction consolidation | `road_constraints.h` — `consolidateJunctionSpans`, `dissolveAcuteArms` |
| the architect's building tables | `architect.cpp` district tables |

Partially exposed already: `polygon.h` and `road_mesh.h` have some surface;
`city_lots` and `parcel` have two mentions each.

### Note on drift

`city.grow`, `city.curves` and `city.paths` — recorded in the project memory as
working — **do not exist in the bindings on this branch**. Whatever branch they
landed on, they are not reachable here. Worth reconciling before anyone plans
against them.

---

## Recommended order (revises the plan's step 0)

1. **Fix the promoted car** ([city_vehicles.cpp:120](src/apps/citysim/city_vehicles.cpp:120)) —
   it is a shipping bug, it is independent of everything else, and it is the one
   Glenn will see the moment he presses G.
2. **Give `make test` Lua** — the prerequisite for deleting any fallback, and
   the concrete form of "scripting is not a build option".
3. **Move fleet length + dimensions to the catalogue** (plan step 1), which then
   unblocks `kFleet` and `kCarColors`.
4. **Tell Glenn about the simplifier** before starting plan step 3 — it is a new
   component, and `CarLod` "mid" is a no-op today.
5. Deletions last, as the plan says.
