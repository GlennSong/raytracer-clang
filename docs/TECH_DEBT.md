# Tech Debt / Known Issues

Small-but-real problems parked deliberately while bigger systems land. Add
items with enough context that future-us can pick one up cold; delete items
when fixed (git history is the archive).

## Rendering / performance

- **Framerate dips to ~20fps in the arena viewer.** Workable but trending
  down. Suspects, in rough order: the post stack accumulated passes (SSAO,
  SSR, bloom, shadow maps, and now DOF/lens-warp wiring) running at full
  resolution; no instancing for the repeated primitive meshes; per-frame
  mesh-bounds queries per entity in RenderSystem. Needs a frame capture on
  the Mac (Xcode GPU capture) before optimizing blind. Quick levers to try:
  half-res SSAO/SSR, FPS cap comparison, toggling passes in the Debug panel
  to bisect the cost.
- **Realtime depth of field doesn't visibly work** (Metal `dofGather` pass,
  written blind on Linux, default-off). The lens-warp pass (distortion/CA/
  vignette) reportedly works; DOF needs on-device debugging — check the CoC
  scale (sensor-meters -> pixels), that `dofTexture` actually replaces
  `sceneColorTexture` at composite, and the depth fetch. The offline tracer
  is the reference: same LensParams produce correct thin-lens DOF there.
- **Camera gizmos render in reflections/shadows** (they are plain
  Renderables). Fine until a debug-draw layer exists.
- **Repeated edit/play cycles re-upload level meshes** without freeing the
  previous set — slow GPU-memory leak across mode switches. Fixed by the
  `AssetManager` (ROADMAP 3.1, `docs/asset-system-plan.md`): refcounted, deduped
  mesh ownership with `release` on overwrite and `clear()` on world teardown.

## Offline tracer

- **glTF ("mesh") entities are skipped** by the level importer — offline
  renders omit imported models. Tessellating glTF into tracer triangles via
  ModelImporter is the fix (it already produces vertex/index arrays).
- **No HDR importance sampling** — bright skies converge slowly (the
  extracted-sun NEE covers the worst case).
- **Emissive geometry has no NEE** — the Cornell light is found by chance,
  as it always was; fine at 128+ spp with the bilateral filter.

## Viewer / UX

- **`settings.json` cameraStorePath leaks across runs** if a session ends in
  a state that never rewrites it; harmless (stale path just reloads the same
  sidecar) but worth folding into the editor's document handling.
- **Metal lens passes still unverified beyond the warp** — see DOF above;
  also eyeball CA/vignette strength constants against taste.

## Procgen / editor

- **Procedural objects aren't shown or editable in the editor** (terrain,
  scattered vegetation). By design for now (ADR-0022): they carry no
  `SourceSpec`, so they're regenerated runtime objects, not document entities.
  Revisit when the node graph (ADR-0021 Phase C) gives generators editable
  instances; until then decide per content type which realness tier it gets.
- **Gamepad doesn't work in the editor.** `HostedWindow` returns an empty
  gamepad set — the Qt host never polls. The GLFW viewer polls gamepads
  (`window.cpp`); to enable it in the editor, poll via the GCController backend
  (`gamepad_gc.mm`, host-agnostic on macOS) and inject a `GamepadSet` into
  `HostedWindow` each frame. Mac-only to verify.
- **Kit-bashed L-system trees are disjoint, self-intersecting cylinders** —
  not welded into one surface. The clean fix is SDF/implicit skinning (branches
  as smooth-min'd capsules, meshed) — a reason to prioritize the SDF mesher
  (ROADMAP Phase A.1). Generalized-cylinder sweeps are an alternative for the
  trunk but don't solve branch joints. *Done (Phase A.1): trees/rocks skin via
  SDF + Surface Nets.*
- **Surface Nets leaves occasional gaps on thin/pinched SDF features.** It
  places one vertex per cell, so a cell where the isosurface has two near-
  touching sheets (twisty branches) can't triangulate cleanly — an isolated
  hole or sliver. Mitigated by the shorter-diagonal quad split, higher
  resolution, and more smooth-union blend (fewer multi-sheet cells). The
  principled fix is **marching cubes** (per-cell topology via the 256-case
  table) or manifold dual contouring — bigger, and best added when the result
  is on-device-verifiable. Tracked for ROADMAP Phase A.1 follow-up.

## Scripting / Lua bindings (ADR-0023/0024)

Added fast across several sessions; the logic is headless-tested but the code
took shortcuts worth paying down before the binding surface grows much more.

- **Binding helpers are copy-pasted across surfaces.** `checkVec3`, `pushVec3`,
  `optField` are defined independently in both `procgen_bindings.cpp` and
  `gameplay_bindings.cpp` (anon namespaces). Lift them into one module-internal
  `lua_helpers.h` so there's a single definition.
- **Entity pack/unpack is duplicated and must stay in sync by hand.** The
  `(generation<<32)|index` encoding lives in `script_system.cpp` (`packEntity`)
  and the decode in `gameplay_bindings.cpp` (`toEntity`). If one drifts, entity
  ids corrupt silently. Move both to one shared `packEntity/unpackEntity`.
- **Every binding is hand-written C-API stack juggling** (manual push/pop
  balance, magic stack indices) — verbose and easy to get subtly wrong. ADR-0023
  deferred a binding lib (sol2); revisit when the surface grows, since the
  per-function boilerplate is the recurring cost.
- **`noise.*` rebuilds a `Noise` per call.** `noise.fbm2(seed, x, y)` constructs
  `Noise(seed)` (a 512-entry permutation build) on every call — wasteful in a
  loop. Should be a `noise.create(seed)` userdata with methods (like the LSystem
  object).
- **Lua behaviour instance refs are never released.** `ScriptSystem` `luaL_ref`s
  each behaviour table but never `luaL_unref`s on entity/component destroy → the
  registry grows until the VM closes. Bounded leak; needs a removal hook. (Also
  in the ADR-0024 register.)
- **`ScriptBehaviour` mixes data with runtime state.** `source` (data) sits next
  to `instanceRef`/`started`/`failed` (runtime), with `-1` as a magic "unloaded"
  ref. A side table keyed by entity, or a clear data/loaded split, is cleaner.
- **The gameplay VM also opens the procgen builders.** Convenient (the gun
  generates its own mesh) but a per-frame `update()` can now call `polygonize`
  etc. with no guardrail — an easy perf footgun.
- **Script failures are log-only.** Behaviour load/`start`/`update` errors
  `LOG_ERROR` + set `failed`; species-script errors are skipped. Nothing surfaces
  to the user/editor — content just goes missing.
- **Cwd-relative asset paths.** `arena_state` reads `assets/scripts/gun.lua` and
  the loader reads `assets/scripts/flora.lua` relative to the process cwd (works
  only when launched from the repo root). Needs real asset-root resolution.
- **`gun.lua` embeds the follow behaviour as a Lua string literal.** No syntax
  check until runtime; awkward to edit. Fine as a demo, not as the pattern.

## Lua flora / forest assembly

- **`loadVegetation` is a 213-line god-function.** It handles tree/rock/builtin/
  graph/script kinds, inline-vs-path detection, per-variant seeds, two scatter
  passes, and entity creation. The mesh-source branching wants a small "species
  mesh provider" seam (builtin | graph | script -> mesh).
- **`script` species inline-vs-path is a `.lua`-suffix heuristic.** `{"script":
  "..."}` is a file path iff it ends in `.lua`, else inline Lua — ambiguous.
  Prefer explicit fields (e.g. `scriptFile` vs `script`).
- **Each scatter pass builds its own `ScriptVM` and re-parses `flora.lua`.** The
  `vegetation` and `foliage` passes don't share a VM, and `flora.lua` is re-read
  every level load (no cache). Share one VM / cache the prelude.
- **The `seed` global is an implicit loader↔script contract.** A species script
  that forgets to read `seed` yields identical variants silently. Undocumented.
- **Instancing for scattered plants — landed engine-side; two follow-ups left.**
  `loadVegetation` now collapses placements into one `InstanceGroup` per species
  (mesh + baked world matrices); `RenderSystem` coarse-culls per group and issues
  `drawMeshInstanced`, whose default loops `drawMesh` so the Metal auto-batcher
  coalesces them into instanced draws (no backend change; headless-tested via the
  scatter-bucket + collapse tests). Remaining: (a) `MAX_INSTANCES = 4096` is a
  per-pass cap — batches beyond it fall back to single draws (degrade, not
  corrupt), so grow the buffer / chunk when forests scale; a *direct* Metal
  `drawMeshInstanced` (skipping the per-call sort) is a minor optimization on top.
  (b) Group bounds are coarse — one group spans the whole region, so the cull
  rarely fires; per-instance / chunk culling is the Tier 5 scaling follow-up.
- **Mesh ops deep-copy the whole buffer.** Every `mesh.translate/scale/orient/
  merge/recompute_normals/bake_height_color` copies vertices+indices (functional
  style). Kit-bashing a leaf canopy allocates a lot. Fine at gen time; revisit if
  generation latency bites.
- **`mesh.bake_height_color(m, c, c)` is used as a flat-color hack.** Relies on
  the mesh having Y extent; a perfectly flat mesh (minY==maxY) would divide-by-
  zero → NaN colors. Add a real `mesh.set_color(m, c)`.
- **`flora.lua` uses Lua `math.random` + global `math.randomseed`** for rock/
  grass/flower — a second RNG alongside the engine `Noise`, reseeded per call via
  global state. Deterministic per call, but not coordinated with the engine's
  seeded streams. Prefer driving variation off `noise.*`/an engine RNG binding.
- **`TurtleParams.taper` is a blunt per-F multiplier** — thins trunk and twigs
  uniformly by path depth, not botanically. Good enough; revisit for nicer trees.
- **Leaf cards orient `+Y -> heading` with no variation,** chosen blind. Likely
  needs jitter/scale variation and visual tuning on-device.

## Verification gap (the meta-debt)

- **Most scripting/flora *integration* is macOS-render-unverified.** Headless
  tests cover generation and logic (counts, taper, determinism, spawn, follow
  math), but the in-game wiring only builds/runs on macOS: `arena_state` (gun +
  script attach), the gun viewmodel, the forest look, the foliage pass. Treat all
  of it as "compiles + logic-tested, not seen." First Mac session: confirm the
  gun renders/fires, trees read as trees (leaf size/density/taper), and check
  forest+foliage framerate.
- **Several tests assert via vertex-count inequalities** ("different species ->
  different vertex counts", "leaves add vertices"). Brittle proxies that can
  break on tuning — smoke tests, not exact specs.
