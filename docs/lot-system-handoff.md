# Lot System — handoff to a local session

Everything below was built in a **remote container with no GPU**. The pipeline
is tested headlessly and reviewed on paper (SVG sheets), but no frame of it has
ever been rendered. The first run in the editor **segfaulted**, and finding that
crash is job one.

Branch: `claude/procgen-city-building-kpuxzd`.

---

## 1. What this is

A replacement for the city's building pipeline, built as ten layered modules in
`src/engine/procgen/city/`. Read `docs/lot-system-plan.md` for the design; this
document is about the state of the code and how to keep working on it.

The chain, bottom to top:

| module | what it owns |
| --- | --- |
| `shape2` | the 2-D kernel: regions with holes and **true circular arcs** (DXF bulge) |
| `shape_ops` | booleans / offset / fillet, exact loop algebra with a sampled-field fallback |
| `plan_grammar` | floor plans as scripts over ops; 23 named templates |
| `mass_stack` | plans stacked into levels; profiles, lofts, the support rule |
| `facade_plan` | bay grids, openings (`fenestrate`), the element registry |
| `lot_program` | what a piece of land is FOR: minimums, targets, setbacks, quotas |
| `site_plan` | zones on a lot (building / frontage / circulation / open / parking / service) |
| `material_set` | palettes per tier |
| `building_recipe` | 37 recipes as data → `BuiltBuilding` |
| `lot_fixtures` | instances, interactables, lights |
| `parcel_block` | blocks → lots, by buildability inversion |
| **`lot_mesh`** | **`BuiltBuilding` → triangles** |
| **`lot_city`** | **blocks → the whole chain → what the engine consumes** |

### The one design decision that matters most

`lot_mesh` and `lot_city` emit into the types the city path **already speaks** —
`BuildingMesh` parts keyed by `PartId`, `LotBuilding` records, the LOD1 twin, the
HLOD proxy. The loader, renderer, surface bake, colliders, chunker and editor
round-trip all consume those today, so **nothing downstream was changed**, and
`LotParams::lotSystem` switches a level between the two pipelines.

Keep it that way. If you find yourself adding a branch outside
`lot_city.cpp`/`lot_mesh.cpp` for "the new path", stop and ask whether the output
can be made to match instead.

---

## 2. How to build, test and look at it

```bash
make test                       # 1197 cases, the whole engine, Jolt-free
cmake -S . -B build && cmake --build build
./build/viewer assets/levels/lot_lab.json     # the lab level (lotSystem: true)
```

`assets/levels/lot_lab.json` is piedmont_mini's streets with
`citysim.lotSystem: true`. **Flip that flag to `false` to A/B the old pipeline on
identical streets** — that is the fastest way to tell "the new pass is broken"
from "this was already broken".

Two headless harnesses, both worth knowing:

```bash
sh tools/lot_sheets.sh /tmp/sheets   # builds + runs all three SVG sheet tools
```

* `lotsystem_{plans,sites,kernel}.svg` — every plan template, site layouts,
  kernel proofs
* `block_{site,parcels,plans}.svg` — a whole block scene, end to end, and the
  template histogram on stdout
* `lotmesh_{buildings,lod}.svg` — **every meshed building, software-rendered**
  (painter's sort, backface cull). This is what caught the inverted winding and
  the missing roofs. Once you have a GPU you can drop it, but it is much faster
  to iterate on than a level load.

`tools/lot_system_build.sh` links any subset of tests fast:

```bash
TESTS="tests/test_lot_mesh.cpp" OUT=/tmp/t sh tools/lot_system_build.sh && /tmp/t
```

---

## 3. THE CRASH — what is known

```
[citysim] fleet: 12/12 scripted car bodies (vehicles.lua), 1898 tris/car avg
[WARN] SimClock dropped backlog (stall #2): fixed steps capped at 8 this frame
zsh: segmentation fault  ./build/editor_app
```

Generation finished and at least one frame ran, so this is **not** a crash
inside the generator. It is something a consumer does with what the generator
handed it, or something in the renderer.

### Ruled out already (verified headlessly, see §5)

* **Building plan polygons are well formed.** 80 plans checked: no out-of-range
  triangulation indices, no empty triangulations, no clockwise rings, no
  duplicate points. The prism-collider path at `level_loader.cpp:3151` is fed
  valid input.
* **Unknown place types do not crash** — `city_render.cpp:247` warns and skips.
  (They *were* silently dropping hotels/schools/hospitals from the sim, which is
  now mapped in `lot_city.cpp::placeTypeFor`.)
* **Mesh integrity**: no NaNs, unit normals, in-range indices, winding matches
  the engine convention — all pinned by `tests/test_lot_mesh.cpp`.

### First moves

```bash
lldb -- ./build/editor_app assets/levels/lot_lab.json
run
bt all
```

A backtrace ends this in minutes and everything below is guesswork by
comparison. If it does not reproduce under lldb, try ASan
(`-fsanitize=address`) — the shape of "ran a few frames, then died" fits a
buffer overrun whose damage surfaces later.

### Prioritised suspects, if you need them

1. **Green lots.** `lot_city.cpp` emits `type == "green"` records with `pad` set
   but `padMesh`, `width`, `depth` and `height` all **zero/empty**. The old pass
   always filled `padMesh`. `level_loader.cpp:3240` handles the combination, but
   the tree scatter below it multiplies by `lb.width`/`lb.depth` (so every tree
   lands on the centroid) and `level_scene.cpp:667` has its own version of the
   same branch. Cheap experiment: stop emitting greens and see if the crash
   goes away.
2. **The chunker / HLOD.** `appendLotMassBox` uses `width`/`depth`/`yaw`, which
   are zero for greens — a degenerate box in a spatial structure is a classic
   way to get a NaN into a comparator and blow up a sort later.
3. **Triangle volume.** ~20 k triangles per block is fine, but the lab's road
   recipe leaves large faces (see §4) and the parts are merged into ONE
   `RenderMesh` per `PartId` across the whole city. If any part exceeds what the
   renderer's index type or a chunker assumes, it will die at draw time, not at
   build time. Check `parts[i].vertices.size()` against 65 k / 4 M limits.
4. **`baseY` and terrain.** `lot_city.cpp` samples `params.ground` at the
   envelope centroid and bakes it into vertex positions. The old pass also
   grades a pad and applies a plinth. If the loader additionally offsets by
   `groundY`, buildings are double-shifted — visually wrong rather than a crash,
   but worth checking early because it is easy to see.

---

## 4. Known-imperfect, not crashes

* **The lab level's streets are thin.** Its metro recipe yields ~13 real blocks;
  larger faces trip the new guard and log
  `[lotcity] face N is … left unbuilt`. That guard is correct (it stops the pass
  OOM-killing itself on a kilometre-wide face) but the road params want tuning
  so more of the map fills. Start from `piedmont_mini.json`'s generate block and
  reintroduce freeways/corridors, which is what the fabric fill keys off.
* **Site furnishing is meshed, fixtures are not.** `meshSiteInto` draws zones and
  boundary runs. `LotFixtures` (instance batches, lights, interactables) is
  built but never consumed — that is the next real feature.
* **No instancing.** Props go through `CityInstanceGroup` in the old path;
  `lot_city` does not fill it yet.
* **`lb.yaw` is always 0** and `width`/`depth` come from an axis-aligned box, so
  the box collider is looser than the old pass's oriented one. The *prism*
  collider from `lb.plan` is exact, which is what actually matters.
* **Multi-mass recipes** (`office_park`, `strip_mall`, `church`, `market_hall`)
  mesh every mass, but `LotBuilding::plan` records only `levels[0].plans[0]`, so
  the collider covers the first mass only.

---

## 5. Invariants worth not breaking

These are all tests; each exists because the opposite happened.

* **The engine's winding convention is inverted from the obvious one.**
  `MeshBuilder::emitQuad` orders indices so `cross(b-a, c-a)` points **against**
  the shading normal. Hand-wound geometry must match, or it is backfacing
  against the whole world. `lotmesh_winding_matches_the_engine_convention`.
* **Every field result must be refit as arcs.** A marching-squares contour is a
  tessellation whose source is gone. A 24-storey tower whose taper ran through
  the field once carried 8 684 plan edges and meshed to **13.8 M triangles**.
  `lotmesh_a_tower_has_a_sane_triangle_budget`.
* **`fitArcs` must stay idempotent.** It fits over the *tessellated* loop and a
  span covering exactly one input edge keeps that edge verbatim. Fit twice and
  the second pass has only the first's output to measure against — the field
  takes a `minEdge` argument so callers fit **once**.
* **A face is not always a block.** Oversized faces are logged and skipped.
* **Lot size follows the PROGRAM, not the block.** Programs declare both a
  minimum (eligibility) and a target (grain); the cutter stations cuts on whole
  target-width lots. Without it, lot size is a power-of-two artefact and a
  bigger block gives *smaller* lots.
* **The scale fit must PEAK, not decay.** A one-sided penalty changes magnitudes
  but never rank, so the largest program in a mix becomes unreachable.
* **Two `engine::X` structs with different layouts is an ODR violation** that
  costs nothing until both are linked into one binary, then corrupts the stack.
  `make test` links every test file together and is the only thing that catches
  it — so it must never be left broken.

---

## 6. Where to go next, in order

1. **Fix the crash.** Nothing else matters until the level boots.
2. **Look at it and tune.** The SVG sheets say the geometry is sane; only a
   screen says whether it is *good*. Expect the first pass to be wrong about
   window density, wall colour and how the ground reads.
3. **Fill the lab's streets** (§4) so there is a city rather than islands.
4. **Fixtures**: `LotFixtures` → instances, lights, interactables.
5. **Frame ledger before/after** against the old pass on identical streets
   (`RT_FRAME_STATS=<csv>`, `tools/frame-report.py`). The R1/R2 work got
   Piedmont to 60 fps; the new pass must not give that back. Both LOD tiers
   exist (22 546 / 756 / 10 triangles for a glass tower) but have never been
   measured in a frame.
6. Only then consider switching Piedmont over.

---

## 7. Things I would tell you in person

* **Draw it before you believe it.** Three separate defects were found by
  rendering to SVG and looking, that no test caught: a parceller stranding green
  space, plans that were all boxes, and the inverted backface cull. Cheap sheets
  beat expensive certainty.
* **Measure distributions, not samples.** The RNG seed-correlation bug (every
  building on a street picking the same palette) was invisible in every
  single-output test.
* **When a number surprises you, chase it.** "13.8 million triangles" and
  "a bigger block gives smaller lots" both looked like tuning and were both
  structural bugs.
