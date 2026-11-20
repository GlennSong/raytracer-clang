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
| (this commit) | model step 6, second piece, a model to a glTF file and back: `procgen/model_file.{h,cpp}`, `writeGlbTree`, `ModelImporter::loadCpuTree`; Lua `model.from_file`, `model.from_files`; the sheet's `--glb` and a `.glb` as input; AGENTS.md "Model file names" brought up to what is built; `tests/test_model_file.cpp` |

The unit suite passed 1525 of 1525 on bdff2422. The level suite has not been run; nothing here changes baked output.

## How to check your work

- **Look at it:** `./build-viewer/rt_model_sheet RECIPE.lua --out build-viewer/sheets/x.png`, then open the PNG. The
  `.txt` beside it has the audit. `--set NODE=VALUE` (repeatable) draws a pose: `--set door_front_l=50 --set hood=40`. `--at METRES` draws the
  level of detail seen from that far. `--shapes` draws the collision shapes in place of the meshes. `--glb FILE` also writes the model as a glTF file to
  open in Blender, and a `.glb` can be given in place of the recipe.
  `assets/scripts/car_rig_lab.lua` (any kit car on the car rig) is the worked example; `build-viewer/sheets/uses.lua`
  (a box car) and `nodes.lua` (a crane) are smaller ones, not tracked. Scratch recipes live in `build-viewer/sheets/` (not tracked).
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
- `mesh.simplify` makes a good middle level and a bad far one: it eats small separate shells first and crumples thin
  solids. A far level wants its own low-poly build (the car kit has one) and plain stand-ins for small pieces.
- A test that reads a recipe's model must stop when the recipe fails, or it reads parts that are not there and the
  whole test binary dies with no message (its output is buffered when piped: `stdbuf -o0 ./build-viewer/run_tests`
  shows the script error). When running several `RT_TEST_FILTER`s in a loop, check each one printed its own "cases
  passed" line: a crashed one prints nothing, and a missing line is easy to read past.
- Leave the source alone while the unit suite runs. Some cases read source files at run time (the word list test
  reads `mesh_builder.h`), so an edit made mid-run fails a case that has nothing wrong with it.
- Edit source with the edit tools, not shell appends (AGENTS.md).

## What is next

**The model store, the last piece of model step 6.** Built models kept under a name so many entities share one; a
handle into it; a cache key (the recipe, its arguments, the libraries it requires, the toolkit's version, and for a
loaded model its files). Look first at what already caches built things by key (`procgen/furniture_kit.h`, the bundle
and lot caches, `AssetManager`), so this is one of those and not a new kind of cache. `ProcModel::name` is there for
it.

Then the prefab half of the design note, from step 7: prefab data and stamping one entity (the model instance, its
drawing by level of detail, physics from the model's shapes).

Two things found on the way that are someone's to decide, both written up in the design note's last section: the
older glTF writer (`writeGlb`) writes triangles the engine's way round, so its files read back inside out; and the
plain importer (`loadCpu`) does not read vertex colours.

The design note's last section, "What is built", lists what each step left alone and which later step owns it.

Not started: edge bevel (deferred until the crane shows whether it is missed), baked maps on instanced objects
(step 4), the ports, the crane. ADRs are written at merge, when numbers are free.
