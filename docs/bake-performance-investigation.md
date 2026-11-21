# Making level bakes faster: an investigation

2026-11-21. Measured on island_8_nature (branch claude/island8-batch, the j396 road file), with the
`build-viewer/rt_bake` binary of 2026-11-20 copied aside. No engine code was changed for this report.

## The answer in one paragraph

The island's bake takes about 42 minutes, and **one CPU core does almost all of it**. Most of that time is
not spent on work that is big or global. It is spent repeating one small question: "where along this road is
this point?" (`project()` in `polyline_ops.cpp:21`). Each call reads the road's whole centre line from start
to end, and the mesher asks it once per vertex, for millions of vertices. That question, and a few others
like it, take about **28 of the 42 minutes**. Speeding them up needs no new design. It needs a lookup grid
for the question, plus the shared thread pool for loops whose items are already independent. That alone
should bring the city stage from ~38 min to a few minutes (an estimate). Glenn's direction, "plan
globally, build per tile", is the right second step. It brings the other two wins: the peak memory is set by
a tile, not the world (today the lots stage needs ~47 GiB of address space), and editing one junction
rebakes a few tiles instead of the island. But tiling first would mean parallelising slow code. Fix the slow
questions first, then tile.

### Where the time goes (island_8_nature, one full bake, measured)

Wall time comes from stack samples of the running bake taken every ~15 s with `eu-stack` (171 samples). "Threads
running" means how many of the process's threads were actually on a CPU in those samples. The whole run took
42:15 wall time and 46:45 CPU time (110% of one core). Its peak memory was 31.8 GB resident, and it then
failed (see "Memory" below).

| Producer / stage | What it does | Wall time | Threads running | Notes |
|---|---|---|---|---|
| **city: meshes → layer meshes** | triangulates the sidewalk, shoulder and median layers and gives each vertex its height | **17.9 min** | 1 | per vertex, `laneHeightAt` → `project()` scans a whole road's centre line; per triangle, `contains()` rebuilds a whole polygon (`toPath`) |
| city: footprints | each lane's outline, merged into one pavement shape for the whole island, with gaps closed | 6.8 min | 1 | polygon boolean operations (union, intersect) over the whole island |
| **city: meshes → deck vertices** | gives each road-surface vertex its lane's frame | **4.6 min** | 1 | `deck_mesh.cpp:1012`: `project()` onto the owner lane's whole line, per vertex |
| city: meshes → deck heights, parapets, road model, junction ranks | barrier lines, the road model, crossing order | 4.2 min | 1 | `DeckHeight::deck` → `laneHeightAt` → `project()` again |
| city: adjacency | which lanes run side by side | 1.5 min | 1 | every pair of 9,331 lanes (`lane_expand.cpp:236`), each pair checked by `project()` |
| city: surfaces | triangulates the road surface (5 M triangles) | 0.8 min | **13** | the only stage that uses the thread pool |
| city: everything else (design, walks, holes, deck, drape, pack, write) | | ~4 min | 1–2 | |
| **city subtotal** | | **~40.8 min** | ≈1.1 | written at 2,446 s; peak 10.7 GB resident (city-only run) |
| lots: plan + grow every lot | parcels blocks, picks landmarks, grows every building | ~1.6 min | 1 | measured in a lots-only run (below) |
| lots: state the walkways | where every lot path's ends land | ~0.9 min | 1 | `stateLotPathsOn` compares every path with every other: 36,224² |
| lots: write | splits the parts into 250 m cells and writes them | ~1.5 min | 1 | |
| **lots subtotal** | | **3.7 min** | ≈1.0 | lots-only run: 221 s, peak 35.3 GB resident / 47 GiB address space |

Not in `rt_bake`, but part of "baking" a level the first time it loads (from earlier notes, not re-measured
today; label: **from notes**):

| Built at load, cached | Wall time | Threads | Keyed by |
|---|---|---|---|
| the weather bake (weathered ground) | ~16 min cold, 5.1 GB | GPU + pool | terrain settings |
| the terrain pyramid (baked ground) | minutes cold (not measured) | pool (`sharedJobs`) | the whole ground with every flatten |
| the population | seconds to minutes (not measured) | its own job pool | every place and the whole street map |
| the road model | 13–15 min, 4.6 GB, when not cached (ADR-0480 notes) | 1 | road file + tags |

## Glossary

- **Bake**: computing a level's slow parts (roads, lots, buildings, meshes) once and saving them to
  `cache/levels/<key>/`, so later loads read them (docs/bakes.md).
- **Producer**: one stage of the bake that writes named sections into the bundle. The island has two:
  `city` (roads) and `lots` (lots and buildings). They run one after the other (`bake.cpp:173`).
- **Key**: a hash of everything a producer reads. If the key matches a saved bundle, that producer is copied
  instead of rebuilt. The lots key includes the city key (`lots_producer.cpp:83`), so new roads always mean new
  lots.
- **Shared blocks / clone**: a new bundle shares disk space with the old one for every section that comes out
  byte-identical (`FICLONERANGE`, a Linux call that makes two files share the same disk blocks). It saves disk,
  not time: everything is still computed.
- **Render cell**: the 250 m square the meshes are cut into for drawing (`cityRenderCell`). Sections are already
  stored per cell: `city/e0/cell/<cx>_<cz>/...`, `lots/cell/<cx>_<cz>/...`.
- **Tile**: in this report, a larger square (say 1 km) used as the unit of parallel work and of rebaking.
  Not the same as a render cell; a tile holds many.
- **Project / projection**: finding the point on a line nearest a given point, and how far along the line it
  is (its "station").
- **Footprint, layer**: a lane's outline on the ground; the sidewalk, shoulder and median strips around it.
- **Pavement holes / blocks**: the gaps the merged pavement leaves; each gap is a city block for the lot pass.
- **Thread pool / `sharedJobs`**: the engine's one set of worker threads (`job_system.h:83`); `parallelRows`
  runs a loop's items across it (`job_system.h:88`).
- **Address space**: memory a process has reserved, used or not. `ulimit -v` limits this, not resident memory.
- **Halo**: extra margin read around a tile so work near its edge sees its neighbours.

## 1. Where the time goes, in detail

### How it was measured
- `rt_bake island_8_nature --out <scratch>` under `/usr/bin/time -v`, with a 50 GB address-space limit.
- Every 15 s the profiler recorded the process's memory, thread count and CPU time. It also took one
  `eu-stack` sample of every thread (`out/bakeperf/profile.sh`). `out/bakeperf/agg.py` turns the samples into
  the table. Inlined call sites were resolved with `addr2line` (deck_mesh.cpp:968, 1012, 1035, 1310).
- The lots numbers come from the lots-only runs made during the memory investigation the same day.
  `rt_bake --producer city` was run once first, so the lots stage could rerun alone in ~4 min.
- The Nov 11 bundle's own manifest records the same picture: `e0.meshes` 1,457 s, `footprints` 402 s,
  `terrain` 144 s, `profiles` 40 s, `surfaces` 45 s, `lots grow` 153 s.

### What the slow parts have in common
1. **A whole-line scan per point.** `project(xy, s, p)` (`polyline_ops.cpp:21`) checks every segment of a
   polyline. Roads on the island are kilometres long, with points every few metres. The scan is called:
   - per layer vertex, through `DeckHeight::layerHeight` → `deck()` → `laneHeightAt` (`lane_expand.cpp:228`),
     several times per vertex, once per candidate lane;
   - per deck vertex (`deck_mesh.cpp:1012`);
   - per barrier point (`deck_mesh.cpp:1310`);
   - per lane pair, 25 times (`adjacency`, `lane_expand.cpp:236`).

   The answers never change during the bake, so a segment grid per line, built once, answers them in near-constant
   time. This is "the slow question", about 28 min of the 42.
2. **A polygon rebuilt per point test.** `triangulateConstrained` tests every triangle's centre with
   `contains()` (`pavement.cpp:100`). That converts the whole ring to a Clipper path each time
   (`geom2d.cpp:107`, `toPath`). The sidewalk layer is one polygon the size of a city. A prepared, y-binned
   ring already exists (`BinnedRing`, `geom2d.cpp:124`, used by `PreparedSet`) but is not used here. This
   is about 5 of the 17.9 min of layer meshes.
3. **Everything is serial.** Only `buildSurfaces` uses the pool. The layer vertices, deck vertices,
   polygons, lane pairs and lots are independent items in loops that run on one core.
4. **Everything is whole-world.** The pavement is one union over every lane on the island. The lot pass
   holds every building's parts in 70 island-wide meshes (19.6 GiB of vertices).

### Memory
- City stage: ~4–6 GB for most of the run, 10.7 GB peak (draping and packing at the end).
- Lots stage: 47 GiB of address space at the end of growing, 39 GiB while writing.
  - The parts are 203 M vertices of 104 bytes, held as 70 island-wide meshes.
  - Each mesh grows by doubling, which reserves space it never uses.
  - The party-wall pass and the cell writer each make a second copy of the biggest mesh (6.6 GiB).

  Details are in the j396 memory report: `out/j396/oom/measure.log`.
- The full bake with a 50 GB cap **failed** in the lots stage (`std::bad_alloc` at 2,534 s). The lots-only run
  passed under the same cap only because the city's leftovers were not in the process.

## 2. What must be global, and what can be local

Reach is how far one item's result depends on things around it.

### City producer (roads), `lanes::build` and `cityProductsFromResult`

| Stage | Reads | Writes | Class | Reach and evidence |
|---|---|---|---|---|
| weathered ground, hydrology | terrain settings | weather cache | **GLOBAL** | rivers drain the island; cached separately |
| road layout (`deriveLayout`) | road file | junctions, streets | GLOBAL (cheap) | `lanes.cpp:186` |
| design (`designRoads`) | road file, terrain | every edge's profile and heights; crossings | **GLOBAL** | a road's grade is fitted along its whole length; crossings are found between every pair of edges (`road_design.cpp:50,59`) |
| lanes (`expand`) | edges | lanes | per road | `lanes.cpp:206` |
| footprints | lanes | one outline per lane; closing fillets; layers | LOCAL in effect, GLOBAL in code | `pavement.cpp:229–287`. A closing fillet is a few r² (`rules.closing`) and touches only the lanes around it, but `unionAll` merges the whole island first |
| surfaces (CDT) | footprints, heights | 5 M road triangles | LOCAL | already parallel (`pavement.cpp:411–418`) |
| piers | decks, other roads | piers | LOCAL (pier spacing) | `lanes.cpp:250` checks every lane per pier: local in effect |
| adjacency | lanes | side-by-side pairs | LOCAL (two lane widths) | `lane_expand.cpp:236`, all pairs |
| ground conform | the 20 m road ground grid, decks | the conformed grid | LOCAL (the conform band) | `lanes.cpp:301` |
| walks | pavement, junctions | sidewalk lanes, corners | LOCAL, per junction | `lanes.cpp:318` |
| pavement holes → **blocks** | the whole pavement union | block outlines | **GLOBAL shape, LOCAL content** | a hole is bounded by its own streets; finding holes needs the union |
| layer and deck meshes, paint | the above | per-cell meshes | LOCAL | written per 250 m cell already (`city_producer.cpp:341`) |
| road model | road file, lanes | the model | GLOBAL (routing) | built at load or bake |

### Lots producer, `growLotBuildings` (`src/engine/procgen/city/city_lots.cpp`)

| Coupling | What depends on what | Class | Reach | Where |
|---|---|---|---|---|
| block seed | a block's dice = `mix(block INDEX, level seed)` | **GLOBAL by accident** | the index is the block's position in the hole list, so adding one block anywhere reshuffles every block after it | `:3937` |
| district tags, coreness, suburb gradient | distance to the town centre and hubs | PER PLACE | the whole town | `:3382`, `:3947` |
| nightlife strips | distance to a night centre per town | PER PLACE | 90 m+ around each | `:3378` |
| shop mix per street | district + nightlife + paseo | PER PLACE (by position) | none between streets | `:5595–5601` |
| big-box stores | 700 m apart, **in block order**; at most 16 | **PER PLACE, order-dependent** | 700 m | `:3869`, `:4208–4288` |
| campus, then sports field within 500 m, dorms within 350 m, mall 400 m from the campus | **the first block that qualifies** | **PER PLACE, order-dependent** | the town | `:4364–4621` |
| landmark quotas (courthouse, school, church) | the best lot in each town's cluster | **PER PLACE** | the town | `planLandmarks`, `:5005` |
| whole-block landmarks | block shape, relief | LOCAL | the block | `:4115–4200` |
| block grading | each block's own plane or terraces | LOCAL | block + 4.5 m + blend | `block_grade.cpp:147`, `:4795–4830` |
| paseo grades | on top of the block grades | LOCAL | the paseo | `:4930–4957` |
| the ground every lot reads | base ground + **all** grades in one sampler | LOCAL in effect | a point reads only grades that overlap it | `:4826` |
| party walls | a wall is blank as high as the neighbour against it | LOCAL, per block | the block | `:6979–7101` (`byBlock`) |
| alleys | cut through a block | LOCAL | the block | `:4887` |
| door walks | walk out to the nearest street | LOCAL | 30 m (`kMaxWalk`) | `:2270–2309` |
| walkway ends | a path's end meets another path, a street within 14 m, or nothing | LOCAL in effect, GLOBAL in code | 0.8–14 m; but compares every path with every other | `stateLotPathsOn`, `:2135` |
| door frontage | the nearest street a building may front | LOCAL in effect | nearest street (unbounded search) | `LotRoads::nearestFrontable` |
| road-locked lots | a lot on ground that is street again | LOCAL | the lot | `:7175` |
| per-lot dice | `mix(block seed, lot index in block)` | LOCAL | the block | `:5047` |

### Built at load

| Thing | Class | Why | Where |
|---|---|---|---|
| weather / erosion | GLOBAL | water flows across the island | `terrain_weather.cpp` |
| terrain pyramid | LOCAL content, GLOBAL key | built in tiles already, but keyed by the whole ground with every flatten, so one moved lot rebuilds it all | `level_loader.cpp:2070–2100` |
| population | **GLOBAL** | who lives and works where, across towns; keyed by every place and the whole street map | `city_sim.cpp:964–995` |
| forest | LOCAL | keeps off lots, beaches, roads near it | `level_loader.cpp:5556` |
| road model | GLOBAL | routing needs every road | `road_model_cache` |

**In short:** the truly global work is small, either data or cheap if indexed:
- the weathered ground;
- road design (grades, crossings);
- the pavement's holes (which blocks exist);
- per-town choices (districts, quotas, campus, big boxes);
- the population and the road model.

Everything heavy is local: meshing, lot growing, grading, party walls, paths, writing.

## 3. A design: plan globally, build per tile

### The plan (one pass, whole world, small data)
1. **Road plan.** Run the road design, lanes, footprint outlines and the pavement holes, with the slow questions
   indexed (step 1 below). Output: centre lines with heights, lane outlines, block outlines.
2. **Lot plan.** Pass A parcels every block. Pass B places landmarks. The order-dependent choices are all
   decided here: big boxes, campus, sports field, dorms, mall. Output per block:
   - its outline, district and seed;
   - its lots, with the role chosen for each;
   - its grade planes.

   These are small (`lots/plan` is already a section).
3. **Tile list.** Squares of ~1 km. **Each block belongs to the tile holding its centroid, whole.** Party walls,
   alleys, paseos, grades and door walks stay inside their block, so no lot-pass coupling crosses a tile edge.
   Roads are cut the same way: each road triangle belongs to the tile holding its centre. This is the rule
   `chunkByCell` already uses for render cells.

### The per-tile jobs (in parallel on `sharedJobs`)
- **Roads:** layer and deck meshes, barriers and paint for the triangles in the tile. They read the road plan, a
  shared read-only input.
- **Lots:** grow every block in the tile from its plan record, then chunk it into render cells and pack it.
- Each job writes **its own** list of (render cell, material) pieces. Nothing is appended to a shared mesh.

### Where a reconcile step cannot be avoided
- **Render cells shared by two tiles.** A block may reach into a render cell owned by another tile. The cell's
  section is then assembled from both tiles' pieces in a fixed order (by global block order), when written.
- **The pavement union and its holes.** The holes have to be found for the whole world. Closing fillets and
  layer unions can be done per tile with a halo of the closing radius plus a margin. A polygon that crosses
  a tile edge is cut there, which changes the triangulation along the cut. That is a visible change and not
  byte-identical, so keep the union global (it is ~7 min today, mostly unindexed lane loops) until it is measured
  after step 1.
- **The walkway ends** (`stateLotPathsOn`). Paths meet within 0.8–14 m. Do it per tile with a 14 m halo, keeping
  "the first such path in the lots' order" by global path order.
- **Ground seams.** The terrain pyramid is keyed per tile by the flattens that overlap it. Neighbours meet
  on shared edge samples, which the pyramid already does internally.

### Does the output stay byte-identical?
- **Steps 1–4 below: yes, if done carefully.** An indexed projection must break ties like the linear scan
  (lowest segment on equal distance). Parallel loops must write only their own item, and the combined
  results must keep their old order.
- **Tiled lots (step 5): yes, with care.** Today a render cell's chunk is the triangles of the island-wide mesh in
  append order. Assembled from per-block runs in global block order, it is the same bytes. The party-wall
  regrowths (appended at the end today) must also be placed last, in their old order.
- **Stable block ids (step 6): no.** Seeding a block from its index (`:3937`) is what makes one edit reshuffle the
  world, so partial bakes need a seed from the block's own outline. That changes every lot once, needs a lots
  tag bump, and is a design decision.

### The job system to use
- **`sharedJobs()` and `parallelRows`** (`src/job_system.h:83–96`): the process's one pool (ADR-0288). It
  already runs the surfaces CDT (`pavement.cpp:418`), the pyramid (`level_loader.cpp:2099`), the weather and the
  forest.
- `jolt_job_adapter.cpp` puts Jolt physics jobs onto the same pool.
- The population uses its own `JobSystem decide` (`city_sim.cpp:1046`).
- Nothing new is needed.
- **Thread-safety checks needed before lot growing runs in parallel:**
  - the Lua state (style books, recipes, `require`);
  - the model cache (`Parsed model: assets/masses/sculpted_01.glb` is logged during growing);
  - `LotPlanDebug` counters;
  - direct appends into `outParts`.

## 4. Partial bakes

### Today: what editing junction j396 rebakes
- The road file's bytes key the city (`keyedRoadFileText`, `city_producer.cpp:106`), so the **whole city stage
  reruns: ~41 min**.
- The lots key includes the city key, so the **whole lots stage reruns: ~4 min**, with ~47 GiB of address space.
- At load, the moved flattens give a new terrain pyramid key, so the **whole pyramid** is rebuilt. New places
  give a new population key, so the **whole population** is rebuilt. If the lane level fails its check, the road
  model is rebuilt on every load (13–15 min; ADR-0480).
- Shared blocks then find most sections byte-identical and clone them. That saves disk, but every
  section was still computed. Also, block seeds come from block indices: if j396 adds or removes a block, or
  moves one in the hole order, every later block regrows differently.

### Under the design
- The plan reruns. It is the only global part, minutes after step 1, an estimate.
- Each tile has a key: the hash of its slice of the plan (the road triangles, block records and grade planes
  it holds), plus the tags.
  - A tile whose key matches the previous bundle's is copied (cloned) without being built.
  - Only the tiles that changed are built.
- j396 changed two roads' geometry around one junction ("both roads bent down into the five-way lights";
  ADR-0480). Two things can make it reach further:
  - designRoads refits a road's heights along its whole length, so the road could reach further;
  - blocks touching those streets regrow.
- Estimate: **2–6 render cells of road mesh and the 2–8 blocks beside them, in 1–2 tiles**. That is
  seconds of tile work after the plan, instead of 45 minutes.
- Per-tile keys fit today's bundles unchanged. Sections are already per render cell, and the bundle writer
  already clones byte-identical sections. A tile key only lets it skip *computing* them.
- The pyramid and population need their own steps:
  - per-tile pyramid keys (tiles already exist);
  - the population stays global, but it is a cache of seconds to minutes (not measured).

## 5. Payoff and order

All payoffs are **estimates** from the measured shares above. Each step can ship and be measured alone with
`rt_bake --out <scratch>` and the manifest's `timings`. The first four keep the bundles byte-identical
(`rt_bake --sections` diff).

| # | Step | Saves (estimate) | Byte-identical? | Risk |
|---|---|---|---|---|
| 1 | **Index the slow questions.** A segment grid per polyline for `project()` and `laneHeightAt` (or a station hint where the caller walks along the line). The prepared ring in `triangulateConstrained`. A box grid for `adjacency` and the pier check. A grid of path ends in `stateLotPathsOn`. | city 41 → ~10 min; lots −1 min | yes, with tie-breaking kept | low: pure functions; compare against the scan in a test |
| 2 | **Use the pool in the independent loops.** Layer polygons and their vertices, deck vertices, barrier runs, packing per material and cell, the lots' cell writer. | city ~10 → ~4 min (24 threads, minus serial parts) | yes | low–medium: per-item outputs, combined in order |
| 3 | **Stream the lots write.** Chunk one cell at a time and free each material once written. Grow big part meshes by less than doubling. | lots peak 47 → ~30 GiB address space | yes | low |
| 4 | **Global footprints, measured again after 1–2.** If still minutes, index the fillet loop's per-piece lane test and run the per-lane outlines on the pool. | footprints 6.8 → ~1–2 min | yes | medium: Clipper results must not change |
| 5 | **Plan / tile split for lots.** Plan pass (A + B + quotas) once. Grow, chunk and pack per tile on the pool. Assemble cells in block order. | lots 3.7 → ~0.5 min; lots peak → a few GB | yes, if assembly order is kept | medium–high: Lua and model-cache thread safety, debug counters, party-wall regrowth order |
| 6 | **Stable block ids and per-tile keys** → partial bakes | a junction edit: ~45 min → plan + 1–2 tiles | **no**: block seeds change once; lots tag bump | design decision: every lot changes once; level pins re-taken |
| 7 | **City tiles** (meshes per tile; per-tile pyramid keys) | memory bounded by a tile; partial city rebakes | mostly; seams need care | high: union and triangulation at tile edges |

Expected after steps 1–3: the island bakes in **~6–8 min instead of ~45**, and the lots stage fits the 30 GB
test limit. After 5: **~5 min**, with peak memory set by the city stage (~5–10 GB). After 6: editing a junction
costs **the plan plus a few tiles: ~1–3 min**. These are all estimates from the shares above, not measured.

### Risks to watch
- **Determinism.** The engine promises the scheduler never changes results (ADR-0002). Every parallel loop
  must write only its own item, and every combine must keep the old order. Diff sections against a saved
  bundle after each step.
- **Lua and caches in parallel growing.** Recipes and style books are Lua. A model is parsed during growing.
  Both need a per-thread state or a read-only preload before step 5.
- **The order-dependent choices.** Big boxes, campus, dorms, sports field, mall and the debug caps
  (`dbg->wholeBlocks < 24` also hides log lines) must all move into the plan, or tiles will disagree.
- **"Every seed works"** and "kinds are data": the plan's records must hold choices, not new code paths.
- **Memory with many tiles in flight.** Cap the number of tiles growing at once, so 24 tiles' parts aren't
  all held together.
- **Partial-bake staleness.** A tile key that leaves out an input is the docs' "NOT caught" row
  (docs/bakes.md). Each tile key must hash everything its jobs read.

## Files
- Profiler and aggregator: `out/bakeperf/profile.sh`, `out/bakeperf/agg.py`.
- Samples and log of the 42-minute run: `out/bakeperf/full.samples`, `out/bakeperf/full.log`.
- Memory measurements: `out/j396/oom/measure.log`, `out/j396/oom/t1.timeline`.
