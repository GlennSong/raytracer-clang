# Road Network v2 — seamless junctions, junction rules, live tuning

Successor to `road-network-execution-plan.md`. The canonical road system (real spline
graph, `shape:"road"` + `generate` recipe → `buildDistrict` → editable `RoadNet` →
`buildRoadNetMesh`) is committed and works. This plan covers the three things that are
*not* yet right: junction seams, the roundabout/degree rules, and road scale + live tuning.

## Locked decisions

1. **Seamless junctions → deterministic shared-boundary stitch + a global vertex-weld
   safety net.** Local-SDF "hero" junctions are optional and come later. Rationale: the
   seams are mostly *emitted-separately* cracks (the pad and the ribbon meeting it compute
   the same boundary position but emit independent vertices), so sharing the vertices *by
   construction* can't crack — more robust than a hybrid-SDF stitch, which is itself the
   fragile join we're worried about.
2. **Tuning UI lives in the in-viewport ImGui panel of `viewer` (`-DRT_ENABLE_IMGUI=ON`).**
   Matches "in a scene, regenerate and watch it build." (`editor_app` is the Qt path, heavier.)
3. **Strategic roundabouts are deliberate:** explicit sites in the recipe
   (`roundabouts:[{x,z,r}]`) plus an `auto_roundabouts:N` option (promote the N most-central
   nodes). Reuses the existing ring builder; approaches already conform (spokes retargeted).
   Automatic busy/acute promotion stays available for hand-authored nets and the mesh path
   (`RoadRules.autoRoundabout`, default on) but is **off** in the city generator.

## Findings that shaped this (code refs)

- Junction seams are **independent-vertex cracks**: pad boundary `mouthR/mouthL`
  (`road_mesh.cpp:434`) vs ribbon edge `L[0]/R[0]` (`road_mesh.cpp:686`) compute the same
  point but emit separate vertices → float drift → hairline gaps.
- The **ugly roundabout seams are sidewalk flaps**: each ring attach-node skirts its
  sidewalk radially and they overlap/z-fight (`road_net.cpp:251`). Killing *auto* roundabouts
  removes them at the source.
- The **editor bakes the recipe on any edit** (`editor_system.cpp:180`,
  `spec.recipe = roadNetToJson(net)`) — the `grown.json` baking bug, and it blocks live tuning.
- `DistrictParams` has a **single `blockSize`** (`district.h`) — no min/max.
- Roundabouts are **real nodes+edges, no flag**: `applyConstraints` (`road_constraints.cpp:92`)
  promotes busy (`> maxDegree`) or acute (`< minArmAngle`) nodes to rings. Degree is *checked*
  but never *capped*. `applyConstraints`/`nodeNeedsRoundabout` are also used by the mesh path
  (`road_net.cpp:125,243`) and 5 tests — so the generator opts out rather than changing defaults.

## Phase 1 — Junction rules & block scale (graph-level, headless-testable)

- **T1.1 — `autoRoundabout` flag, generator opts out.** Add `bool autoRoundabout = true` to
  `RoadRules`; gate the busy/acute promotion in `applyConstraints` (and `nodeNeedsRoundabout`)
  on it. The city generator (`level_loader` district path) passes `autoRoundabout = false`.
  *Accept:* `grown.json` generates with zero auto-roundabouts; existing roundabout tests +
  `road_roundabout.json` unchanged (default still on).
- **T1.2 — Degree-cap pass (`capDegree`, ≤ `maxDegree`).** Split each over-busy node into two
  nodes a short distance apart, joined by a short link, arms partitioned by bearing at the
  widest gap; repeat until all ≤ cap. Runs in the generator before `applyConstraints`/`planarize`.
  *Accept:* a test asserts max degree ≤4 over a busy hub and that no spoke is dropped.
- **T1.3 — Strategic roundabouts.** Recipe `roundabouts:[{x,z,r}]` + `auto_roundabouts:N`;
  drive the existing ring builder by explicit selection; re-planarize. *Accept:* a placed ring
  shows retargeted approaches in the graph and round-trips losslessly.
- **T1.4 — Block min/max.** Replace `blockSize` with `blockSizeMin`/`blockSizeMax` (keep
  `blockSize` as an alias setting both); `subdivideStreets` stops when a block fits `[min,max]`
  with an aspect target so blocks are lot-sized. *Accept:* generated block bbox dims ∈ `[min,max]`.

## Phase 2 — Live tuning loop (ImGui)

- **T2.1 — Fix recipe preservation (prereq).** Split regenerate: `regenerateRoadGeometry`
  (hand edits → bake, current) vs `regenerateRoadFromRecipe` (generated roads → re-run
  `buildDistrict`, keep the `generate` recipe). Stop clobbering at `editor_system.cpp:180`.
  *Accept:* editing a generated road in `--edit` keeps `grown.json` a ~17-line recipe.
- **T2.2 — Generation tuning panel.** ImGui sliders for all `DistrictParams` + the `RoadRules`
  knobs (min arm angle, max degree, roundabout toggle, block min/max, widths, arterials,
  jitter, irregular, seed). Pattern: `FieldMeta` + `ImGuiPropertyVisitor` (`properties.cpp:94`).
- **T2.3 — Regenerate button + live rebuild.** Re-run the gen chain (`level_loader.cpp:285-323`)
  → repopulate `RoadNet` → `buildRoadNetMesh` → upload, writing params back into the `generate`
  recipe (not baked). *Accept:* drag a slider → Regenerate → network rebuilds in-viewport (<~50ms
  for `grown.json`).
- **T2.4 (optional) — "Watch it build" stepping.** Reveal arterials → subdivision → planarize →
  constraints in sequence.

## Phase 3 — Seamless junctions (mesh; Phase 1 removed the worst cases)

- **T3.1 — Global vertex-weld safety net (quick win).** Index the mesh and collapse coincident
  vertices (tol ~1e-3) at the end of `buildRoadMesh` (`road_mesh.cpp:723`) / `buildRoadNetMesh`
  (`road_net.cpp:290`). Mesh is non-indexed today, so this also yields an indexed mesh.
  *Accept:* no hairline cracks under zoom on `grown.json`; vertex count drops.
- **T3.2 — Deterministic shared-boundary stitch (the real fix).** Each ribbon's end
  cross-section *uses* the pad's mouth vertices (`road_mesh.cpp:434`) instead of recomputing
  them (`road_mesh.cpp:686`); same for sidewalk rails. Junction and ribbon share an edge by
  construction. *Accept:* zero pad↔ribbon seam, sidewalks continuous through corners.
- **T3.3 (optional) — Local-SDF hero junctions.** Confine `unionRoadbed` to a per-junction bbox
  (the owed perf fix, `road_net.cpp:256`), behind a per-road flag, stitched to T3.2's shared
  vertices. *Accept:* the SDF look at interactive cost, no boundary seam.

## Critical path

`T1.1 → T1.2 → T2.1 → T2.2/T2.3` (now it's visible/tunable) `→ T3.1 → T3.2`.
`T1.3, T1.4, T2.4, T3.3` are parallelizable / optional.

## Status

- [x] Decisions locked
- [x] **T1.1 — `autoRoundabout` flag (default on) + generator opt-out.** `road_constraints.{h,cpp}`,
  `level_loader.cpp`. Mesh path + hand-authored rings + 5 existing tests unchanged; `grown.json`
  generates with zero auto-roundabouts (verified top-down + oblique).
- [x] **T1.2 — `capDegree` ≤4 pass.** Over-busy nodes split into short staggered junctions;
  3 new tests green. The 3-arterial centre (degree-6) now resolves to ≤4-arm junctions.
- [x] **T1.4 — block min/max sizing.** `DistrictParams.blockSizeMin/Max`; recipe keys
  `block_size` / `block_size_min` / `block_size_max`; `subdivideStreets` brackets block edges.
  Unit test + visual (denser grid) confirm. *Note:* at extreme density + `curved`, `capDegree`
  node clusters curve-fit into ring-like artifacts — a `capDegree` placement refinement (the
  canonical `grown.json` is clean; straight roads at the same density are clean).
- [ ] T1.3 — strategic roundabouts (explicit sites + `auto_roundabouts:N`)
- [x] **T2.1 — recipe preservation + shared generate.** `applyGenerateRecipe` (the loader and the
  editor share one path) and `roadRecipeForSave` (keeps the `generate` block, never bakes the nodes)
  in `road_net.{h,cpp}`; `regenerateRoad` is recipe-aware. 2 unit tests (capped net + recipe survives
  an edit) — the grown.json "save changed" bug is fixed at the source.
- [x] **T2.2 / T2.3 — generation tuning panel + live regenerate.** ImGui sliders (radius, arterials,
  block min/max, irregular, jitter, artery/street width, seed + Reseed) at the top of the road
  inspector; any change rewrites the `generate` recipe and calls `regenerateRoadFromRecipe` to
  rebuild live. Compiles under `-DRT_ENABLE_IMGUI=ON`; panel rendering verified headlessly.
- [ ] T2.4 (optional) — "watch it build" phase stepping (live Reseed/sliders already let you watch)
- [ ] Phase 3 — seamless junctions
