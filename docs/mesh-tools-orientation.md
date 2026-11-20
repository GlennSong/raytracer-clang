# Mesh tools and models: orientation (#191)

For whoever picks this work up, person or helper. Read this first, then the two files it points at. Written
2026-11-19.

## Where the work is

- **Folder:** `.claude/worktrees/tmp-meshtools`, branch `tmp/mesh-tools`, cut from `claude/island8-batch` at 31c6e538.
- **Never pushed, not merged.** Glenn's call: keep working here until the driving session on `claude/island8-batch`
  is done, then merge into that branch.
- **Stay inside this folder.** Another session is working in `.claude/worktrees/buildings`. Do not edit, build or
  kill anything there. It also runs `run_tests`: stop only your own process, by its id, after checking
  `/proc/<id>/cwd`.
- **Build:** `cmake --build build-viewer --target <target>` here. The unit suite is `./build-viewer/run_tests`
  (about 25 minutes; `RT_TEST_FILTER=<part of a name>` runs a few cases). Do not rebuild `run_tests` while it runs.

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
| (this commit) | the mirrored twin: `mirroredModel` in `procgen/proc_model.cpp`, Lua `m:mirrored()`, left to right only |

The unit suite passed 1525 of 1525 on bdff2422. The level suite has not been run; nothing here changes baked output.

## How to check your work

- **Look at it:** `./build-viewer/rt_model_sheet RECIPE.lua --out build-viewer/sheets/x.png`, then open the PNG. The
  `.txt` beside it has the audit. Scratch recipes live in `build-viewer/sheets/` (not tracked).
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
- Edit source with the edit tools, not shell appends (AGENTS.md).

## What is next

**Model step 4 in the design note: motion limits and poses.** Turn and slide on nodes, with limits; the model sheet
takes `--set cabin=90` to draw a pose. Before building, read the design note's "Rules for a rig" section: it holds
what Glenn's car example settled (motion sits above any flip, the same value means the same action on both sides)
and his call that a kind of thing has ONE rig table in Lua which each recipe applies.

The design note's last section, "What is built", lists what each step left alone and which later step owns it.

Not started: edge bevel (deferred until the crane shows whether it is missed), baked maps on instanced objects
(step 4), the ports, the crane. ADRs are written at merge, when numbers are free.
