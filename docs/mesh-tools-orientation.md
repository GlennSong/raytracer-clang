# Mesh tools and models: orientation (#191)

For whoever picks this work up, person or helper. Read this first, then the two files it points at. Written
2026-11-19.

## Where the work is

- **Folder:** `.claude/worktrees/tmp-meshtools`, branch `tmp/mesh-tools`, cut from `claude/island8-batch` at 31c6e538.
- **Never pushed, not merged.** Glenn's call: keep working here until the driving session on `claude/island8-batch`
  is done, then merge into that branch.
- **Updates from the parent come in by merge** (`git merge claude/island8-batch` here, Glenn's call 2026-11-19), never
  the other way, and never a rebase: this page names this branch's commits by hash.
- **Stay inside this folder.** Another session is working in `.claude/worktrees/buildings`. Do not edit, build or
  kill anything there. It also runs `run_tests`: stop only your own process, by its id, after checking
  `/proc/<id>/cwd`.
- **Build:** `cmake --build build-viewer --target <target>` here. The unit suite is `./build-viewer/run_tests`
  (about 25 minutes; `RT_TEST_FILTER=<part of a name>` runs a few cases). Do not rebuild `run_tests` or edit source
  while it runs.

## What to read

1. **`docs/model-record-design.md`**: the agreed design for models and prefabs, the order of building, and Glenn's
   calls. This is the plan.
2. **Issue #191 on GitHub** and its comments: the seven steps and every decision. Follow-ons: #308 buildings, #309
   materials and decals, #310 motion as data, #311 flexible parts, #312 breaking.
3. **AGENTS.md**: the engine's rules. "Procgen Authoring", "Engineering Ethos", "Working & Communicating" and
   "Model file names" bear on this work most.

## What is built (all on this branch)

| Commit | What |
|---|---|
| b2425aaf | `tools/rt_model_sheet.cpp`: one command draws a Lua recipe from four sides and writes its size, counts and mesh audit |
| 7c5771ea | Lua words over the toolkit (`mesh.weld`, `simplify`, `remesh`, `fill_holes`, `hull`, `tri`, ...), the word list test, and booleans (`MeshBuilder::boolean` in `engine/mesh_boolean.cpp`, over `third_party/manifold`) |
| bdff2422 | `mesh.sheet`, `mesh.ruled`, `mesh.patch` |
| 4301e016, 7fb30148 | the design note and the model file name conventions |
| 664eb630 | model step 1, nodes: `ProcNode` and `ProcFrame` in `procgen/proc_model.h`, Lua `m:node` and the node words, read by the bake, the loader, `m:flatten` and the model sheet; `tests/test_model_nodes.cpp` |
| dc9bda04 | model step 2, markers and named materials: `ProcMarker`, the model's material table (`ProcPart::material` is an index), Lua `m:marker`, `m:material`, `m:add(mesh, "paint")`; `tests/test_model_markers.cpp` |
| b8d4fb81 | model step 3, uses: `ProcUse` replaces the instance group; the one walk in `procgen/proc_model.cpp` (`placedUses`, `drawnPieces`, `flattenModel`); Lua `n:use`, `m:use`; `tests/test_model_uses.cpp` |
| 9e36e21c | the mirrored twin: `mirroredModel` in `procgen/proc_model.cpp`, Lua `m:mirrored()`, left to right only |
| b74d5d99 | model step 4, motion and rigs: `ProcMotion`, `ProcPose`, `posedModel`; Lua `n:motion`, `m:posed`, `m:joints`; the rig tables in `assets/scripts/rigs.lua`; the sheet's `--set`; `tests/test_model_motion.cpp` |
| 5c403683 | the rig's joints renamed to the names the car kit and the game use; `assets/scripts/car_rig_lab.lua`, any kit car as a model on the car rig (model sheet only); the sheet lists nodes in tree order |
| a4d6aa98 | model step 5, levels of detail: `levelFrom`, a level on parts and uses, `addSimplifiedLevel`, `MeshBuilder::simplifiedSurface`; Lua `m:level`, `{ level = n }`; the sheet's `--at`; `tests/test_model_levels.cpp` |
| 25780c6d | merge of `claude/island8-batch` into this branch, up to 74cdd74d (the walking road work, ADR-0464 to 0467). Glenn's call: its updates into this one, nothing the other way |
| 268ba601 | model step 6, first piece, collision shapes on nodes: `ProcCollider` gains a name, a node, a rotation and the hull kind; `shapeMesh`; Lua `m:shape` / `n:shape`; the sheet's `--shapes`; `tests/test_model_shapes.cpp` |
| b0e9dcb7 | second merge of `claude/island8-batch`, up to 23a20eeb (ADR-0468, where a place joins the sidewalks) |
| d6c798a6 | model step 6, second piece, a model to a glTF file and back: `procgen/model_file.{h,cpp}`, `writeGlbTree`, `ModelImporter::loadCpuTree`; Lua `model.from_file`, `model.from_files`; the sheet's `--glb` and a `.glb` as input; AGENTS.md "Model file names" brought up to what is built; `tests/test_model_file.cpp` |
| 6fc7cf5a | model step 6, last piece, the model store: `procgen/model_store.{h,cpp}`, `scripting/model_recipes.{h,cpp}` (`obtainModel`), the list `assets/scripts/models.lua`, Lua `model.get`, `ProcModel::frozen`; the sheet takes a model's name; `tests/test_model_store.cpp`. THE MODEL HALF OF THE DESIGN NOTE IS DONE |
| ec4c66ba | the car lab's middle level is built by the kit, not simplified (Glenn's call after looking at the file); `kit.build` takes `autosmooth`; `n:use(piece, nil, opts)` keeps its options |
| f411f2ca | the kit's hood and trunk lid take both shoulders (Glenn saw the body differ left to right). Changes the game's kit cars on the right shoulder |
| 8712173e | the sheet measures triangle shape and left against right: `measureShape`, `checkMirror` in `engine/mesh_audit.h`; `tests/test_mesh_shape.cpp` |
| 949357c5 | the kit creases the same band on both flanks (the second cause of left differing from right). Changes every subdivided kit car |
| d534a3c5 | a door opening's wall stands at the opening only: `solidify` takes `against`; the sedan's 532 edges with three or more faces are gone. Changes kit cars with panels |
| 55cf9eba | the sheet's shape lines name their part; the survey's findings below |
| 7a63568e | third merge of `claude/island8-batch`, up to bec6d732 (ADR-0469 to 0473, sidewalks at junctions and road file fixes). Roads only; the car's fingerprint is unchanged |
| 77ce4a76 | PREFAB STEP 7, prefab data and stamping one entity: `procgen/prefab.h`, `scripting/prefab_list.{h,cpp}`, `engine/prefab_stamp.{h,cpp}`, `engine/model_spawn.{h,cpp}` (the loader's material and collider code, now shared), `modelDraws`, `ModelInstance` / `ModelPiece`, `Renderable::lodAnchor`; `assets/scripts/prefabs.lua`, `crane.lua`, `assets/levels/prefab_lab.json`; `tests/test_prefab.cpp`, `tests/test_prefab_physics.cpp` |

The unit suite passed 1592 of 1592 on 77ce4a76. `physics_tests` (built here since 2026-11-20:
`cmake --build build-viewer --target physics_tests`) passes all but one driving case, see "What is next". The
level suite has not been run.

## How to check your work

- **Look at it:** `./build-viewer/rt_model_sheet RECIPE.lua --out build-viewer/sheets/x.png`, then open the PNG. The
  `.txt` beside it has the audit. `--set NODE=VALUE` (repeatable) draws a pose: `--set door_front_l=50 --set hood=40`. `--at METRES` draws the
  level of detail seen from that far. `--shapes` draws the collision shapes in place of the meshes. `--glb FILE` also writes the model as a glTF file to
  open in Blender, and a `.glb` can be given in place of the recipe.
  `assets/scripts/car_rig_lab.lua` (any kit car on the car rig) is the worked example; `build-viewer/sheets/uses.lua`
  (a box car) and `nodes.lua` (a crane) are smaller ones, not tracked. A model of `assets/scripts/models.lua` is
  drawn by its name: `./build-viewer/rt_model_sheet sedan --out build-viewer/sheets/sedan.png`. Scratch recipes live in `build-viewer/sheets/` (not tracked).
- **Has the shape changed?** The `.txt` has a `fingerprint` line: one number for all the drawn triangles and what
  each is made of, whatever their order. The same number before and after a change means the same triangles. Take it for every recipe before
  you start (a loop over `assets/scripts/*.lua` with `--size 64 --spp 1` takes about 20 seconds; the scripts that are
  data tables and return no model just report that).
- **The picture draws both sides of a face,** so facing is judged by the audit's numbers and by tests, not by eye.
- **A new function in `mesh_builder.h` must go on the list** in `tests/test_mesh_lua_words.cpp` with its Lua word or
  the reason it is C++ only. The test fails otherwise.

## Things learned the hard way

- The engine's round primitives close their seam with a vertex about 5e-8 m from the first. The boolean treats
  vertices within 1e-5 m as one point; without that the library refuses them as not closed.
- The boolean library does not turn the normals of a cut cavity's walls. `mesh_boolean.cpp` does.
- `mesh.cut` clamps to a plane; it does not open a hole.
- A recipe that walks a Lua table with `pairs` (the car kit does) adds its parts in a different order every run.
  Compare shapes by the sheet's fingerprint, never by the order of parts or by a hash that depends on it.
  `bake_lab.lua` goes further: its baked normal maps come out different each run for the same reason, so its
  fingerprint never repeats. Run it many times and look for the old number among the results.
- A Lua word that takes optional arguments must not read a stack slot it was not given: slot 0 is not "nothing",
  it crashes. Test every word with its shortest call.
- `mesh.simplify` makes a poor level of anything a recipe can build lower. It weighs only how far the surface moves:
  flat panels come out as long thin triangles in no order, and far away it eats small separate shells first and
  crumples thin solids. Build each level with the recipe (the car kit builds low; `car_rig_lab.lua` shows three
  levels made that way) and keep `simplify` for things with no recipe.
- Measure a mesh, do not judge it by eye. The sheet's `.txt` has "triangle shape" (thin and sharp triangles, edge
  lengths, how regular the points are, open and over-shared edges, per part) and "left against right" (points with
  no mirror across x = 0). They are `measureShape` and `checkMirror` in `engine/mesh_audit.h`. "Thin" alone misleads:
  a long narrow quad is thin and in perfect order. "Regular" is the number that follows what the eye calls order.
- The kit car's thinnest triangles (edges of 0.02 mm on a hood) are the cage's design, not a bug to patch: every
  section has the same number of points, so ahead of the windscreen the cabin's corners are folded into a ribbon a
  few millimetres wide along the hood's edge, and the middle of the hood is one quad across. Changing it changes
  every car's look: Glenn's call.
- A survey of every recipe (2026-11-20, `--size 64 --spp 1` over `assets/scripts/*.lua`): the building labs carry
  most of the edges with three or more faces (circulation_lab 178,084, hotel_lab 56,098, store_lab 36,406,
  floor_lab 28,533, tower_lineup 27,493). Those are the buildings session's to look at, not this branch's.
- `n:use(piece, nil, { level = n })` once dropped its options, and the use went to the full level without a word.
  Fixed; the sheet's "copies of used pieces" per level is where it showed.
- A test that reads a recipe's model must stop when the recipe fails, or it reads parts that are not there and the
  whole test binary dies with no message (its output is buffered when piped: `stdbuf -o0 ./build-viewer/run_tests`
  shows the script error). When running several `RT_TEST_FILTER`s in a loop, check each one printed its own "cases
  passed" line: a crashed one prints nothing, and a missing line is easy to read past.
- Leave the source alone while the unit suite runs. Some cases read source files at run time (the word list test
  reads `mesh_builder.h`), so an edit made mid-run fails a case that has nothing wrong with it.
- Edit source with the edit tools, not shell appends (AGENTS.md).

## What is next

Glenn's order (2026-11-20): the systems first, the fancy crane last. **Prefabs, then test them with many procgen
models, then modelling tools.**

1. **Step 8, motion on the entity and the script.** A `ModelInstance`'s pose moves its pieces and its bodies; a
   script sets a node's target. The design note's step 7 section lists what step 7 left for it (mesh bodies are
   merged per friction, not per node; nothing is kinematic).
2. **More models through prefabs:** furniture pieces, street props, a tree, a building part. Each is a line in
   `models.lua` and one in `prefabs.lua`; what breaks is the finding.
3. **Modelling tools,** in the order listed in the design note ("Modelling tools still to build"): edge bevel
   first.

The design note's "The crane Glenn asked for" maps each of his wishes for the crane to the system it waits on.

Things found on the way that are someone's to decide:

- The older glTF writer (`writeGlb`) writes triangles the engine's way round, so its files read back inside out;
  the plain importer (`loadCpu`) does not read vertex colours. Both are written up in the design note.
- **`near_tier_in_the_crowd_follows_the_player` fails in `physics_tests`** on this branch after the third merge
  (seven checks, the pinned state hash among them). It fails the same way, with the same hash, with the car kit as
  it was before this branch's kit fixes, so it is not the kit: it came in with the parent's road commits, or was
  failing there already. The driving session's to look at.

Not started: edge bevel (deferred until the crane shows whether it is missed), baked maps on instanced objects
(step 4), the ports, the crane. ADRs are written at merge, when numbers are free.
