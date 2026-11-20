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

The unit suite passed 1525 of 1525 on bdff2422. The level suite has not been run; nothing here changes baked output.

## How to check your work

- **Look at it:** `./build-viewer/rt_model_sheet RECIPE.lua --out build-viewer/sheets/x.png`, then open the PNG. The
  `.txt` beside it has the audit. Scratch recipes live in `build-viewer/sheets/` (not tracked).
- **The picture draws both sides of a face,** so facing is judged by the audit's numbers and by tests, not by eye.
- **A new function in `mesh_builder.h` must go on the list** in `tests/test_mesh_lua_words.cpp` with its Lua word or
  the reason it is C++ only. The test fails otherwise.

## Things learned the hard way

- The engine's round primitives close their seam with a vertex about 5e-8 m from the first. The boolean treats
  vertices within 1e-5 m as one point; without that the library refuses them as not closed.
- The boolean library does not turn the normals of a cut cavity's walls. `mesh_boolean.cpp` does.
- `mesh.cut` clamps to a plane; it does not open a hole.
- Edit source with the edit tools, not shell appends (AGENTS.md).

## What is next

Model step 1 in the design note: **nodes** in `ProcModel` (`procgen/proc_model.h`) and in Lua's `model` table
(`scripting/procgen_bindings.cpp`, the `l_model_*` functions), read by `bakeProcModel` (`level_scene.cpp`), the level
loader's script path (`level_loader.cpp`, near "The realtime twin of level_scene's bakeProcModel") and the model
sheet. A model with one node must stay exactly today's flat model: prove it by the same triangles from every
existing recipe.

Not started: edge bevel (deferred until the crane shows whether it is missed), baked maps on instanced objects
(step 4), the ports, the crane. ADRs are written at merge, when numbers are free.
