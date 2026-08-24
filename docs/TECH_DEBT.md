# Tech Debt / Known Issues

Small-but-real problems parked deliberately while bigger systems land. Add
items with enough context that future-us can pick one up cold; delete items
when fixed (git history is the archive).

## City procgen — dead-code audit (2026-08-15)

Commissioned before deleting anything, on the theory that ~60 files in
`src/engine/procgen/city/` held a lot of abandoned code. **The theory was mostly
wrong, and the way it was wrong is the useful part.**

An audit that traces only the *engine* path for one level concludes that most of
the city module is dead. That conclusion is an artifact of the method. Two things
keep code alive that a single-level engine trace cannot see:

1. **The Lua binding surface (ADR-0042).** `procgen_bindings.cpp` exposes the
   road/mesh vocabulary to scripts. Code with no C++ caller is often reachable —
   and *required to be reachable* — from Lua. Deleting it breaks the engine rule.
2. **The other generation kinds.** `living_city` uses `"district"`, but that is
   the minority path (2 levels). **`"metro"` is the dominant one — 10 levels,
   including `piedmont` and `metropolis`.** Anything "dead on the district path"
   may be load-bearing for the flagship scenes.

Level census, for reference:

| path | levels |
| --- | --- |
| `road:metro` | 10 — coast_city, freeway_lab, hillcity, metro_hills, metropolis, metropolis_roads, metropolis_sky, piedmont, piedmont_mini, piedmont_roads |
| `road:district` | 2 — grown, living_city |
| ~~`shape:city`~~ | *retired 2026-08-16; city + city_arena migrated to `road`* |
| `shape:corridor` | 2 — freeway_lab, freeway_variants |
| `citysim.buildLots` | 7 — coast_city, hillcity, living_city, metro_hills, metropolis, metropolis_sky, piedmont |

### Verdicts

**Not dead — alive via Lua. Do not delete.**

| Code | Reached by |
| --- | --- |
| `tensorRoads`, `radialRoads`, `gridRoads`, `pruneSteepEdges`, `connectComponents` | `city.layout` (`procgen_bindings.cpp:1145`). **`twin_cities.lua` ships both `pattern="radial"` and `pattern="tensor"`**, so the tensor field is live content, not theory. `city.layout` is the most-used verb in the script corpus (7 call sites). |
| `road_mesh.cpp` weld/union family — `unionRibbons`, `weldRibbons`, `unionRoadbed`, `bridgeDeck`, `deckBarriers`, `deckMarkings`, `laneMarkings`, `buildRoadMesh` | All bound (`city.union`, `city.weld`, `city.roadbed`, `city.deck`, `city.lane_markings`, `city.road_mesh`). |
| `road_crossings.cpp:resolveCrossings` | `city.resolve` — used twice in shipped scripts. |
| `metro.cpp`, `patch_fabric.cpp`, `city_footprint.cpp`, `arterial_skeleton.cpp`, `buildability.cpp` | The `"metro"` kind — the majority path. |
| `corridor_mesh/plan/bake.cpp`, `alignment.cpp` | `shape:"corridor"` levels + metro freeway plans. |

*Caveat worth tracking separately:* several of those Lua verbs are **bound but
unused by any shipped script** (`city.union`, `city.weld`, `city.stroke`,
`city.deck`, `city.lane_markings`). That is untested surface area, not dead code —
the honest fix is a script that exercises them, not deletion.

**Genuinely dead — safe to delete, nothing to salvage.**

| Code | Notes |
| --- | --- |
| `BuildingParams::wallThickness` | Declared at `shape_grammar.h:162`, defaulted `0.3`, **never read anywhere**. The only true zero-value item in the audit. |

**Dead, but carrying knowledge worth keeping.**

| Code | What it knows | Recommendation |
| --- | --- | --- |
| `surface_field.h` — `SurfaceField` (no production caller; tests only) | The **one-ground-oracle** design from ADR-0075. It unifies the three historically interchangeable height closures (`HeightSampler`, `HeightField`, `RoadNet::heightAt`), owns the cut/fill edit stack with a spatial index, and encodes a genuinely good invariant: *a stale index is a perf bug, never a correctness one* (queries fall back to the linear fold). Today `level_loader` does this by hand — accumulating `TerrainFlatten` vectors, calling `rebuildFlattenIndex` manually, passing bare closures. | **Wire it, don't cut it.** This is unbuilt-but-wanted infrastructure. Adopting it would delete hand-rolled folding from the loader and give the grading cascade one owner. |
| `DistrictNet::blocks` (computed by `buildDistrict`, discarded by `applyGenerateRecipe`; only `city.cpp` consumes it) | The generator knows its own sector faces *before* warp/prune move every node. Discarding is currently correct — the post-warp graph is the truth. | **Keep, revisit under stage-9 work.** Zoning-follows-streets (task #5) may want the generator's own sector identity, which `extractBlocks` cannot recover. |

**Duplication — real, but each needs a decision, not a delete.**

| Item | Reality |
| --- | --- |
| ~~**Two city pipelines**~~ | **RESOLVED 2026-08-16.** `city.json`/`city_arena.json` migrated onto `road` + `citysim.buildLots`; `city.{h,cpp}`, both host bake paths and the `CityModel`→`RoadNet` nav bridge deleted. The building-winding gate was ported to `growLotBuildings` first, so retiring the generator did not take the only test for inside-out facades with it. |
| `level_scene.cpp` re-implements `growCityLots` | **Narrower than first reported.** The JSON half is genuinely shared (`readLotGrowParams`); only the road-net-derived half — hub list + coreness anchor — is duplicated, with a comment explaining that the editor otherwise grows a *different* city (no districts, coreness 0, no towers). Still a fork risk; the fix is to hoist those ~12 lines, not to rewrite. |
| `emitCornice` (box path) vs `sweptCornice` (plan path) | Two lambdas in different functions carrying the same tier table `{0.10,0.16},{0.24,0.18},{0.34,0.08}`. One emits boxes, one emits plan slabs — so they cannot merge outright, but the table should be one shared constant. |

**Not dead code — a real gap.**

- **`VentGrille`, `UtilityPanel`, `FanTop` have no case in `shaders/metal/surfaces.metal`** (13 cases, ids 1–13; these are 14–16). They work only through the CPU bake, so a material carrying one of those ids *without* bound maps renders untextured. Either add the analytic cases or document the three as bake-only.
- Relatedly: `applySurface` only runs when no albedo map is bound
  (`lighting_surface.metal:88`), and the lot pipeline always binds baked maps — so
  **city buildings never execute the analytic shader at all**. The `Surface` id
  survives as provenance. Worth deciding whether the analytic path is still wanted.

### Method note

The first pass of this audit called ~9 modules dead. Checking each against the Lua
bindings and the non-`district` generation kinds cut that to **one** field. Before
deleting anything in `procgen/city/`, grep `procgen_bindings.cpp` and check the
level census above — a C++-caller search alone will lie to you.

## Rendering / performance

**Open follow-ups (rough priority — detail in the notes below):**
0. **CDLOD terrain — viewer-unverified + interim gaps (ADR-0036, Phase 1c).** The
   CPU core (selection + morph) is unit-tested, but the Metal terrain pipeline /
   morph vertex shader, the engine `TerrainLodSystem`, and the level wiring are
   **Metal/viewer-only and unverified on Linux** — needs a viewer pass on
   `assets/levels/cdlod.json` (morphing should be seamless, no popping/cracks at LOD
   boundaries; the far field should coarsen). Closed since the first cut (each
   viewer-verified): terrain **shadow casting** (morphing caster pipeline) + the
   empty-cascade/depth-range artifact; **seamless normals** across LOD (world-
   consistent eps); **camera-centered cascades** (shadows stable when turning); and
   **near-node colliders** — `TerrainLodSystem` owns a moving window of static
   triangle-mesh bodies around the player (addMesh/removeBody), so you walk on the
   surface. Remaining interim gaps: (a) **normals are not morphed** (height only), so
   lighting can shift subtly mid-morph; (b) render node meshes are cached and never
   evicted (distance/budget eviction is Phase 3 streaming); (c) colliders use the
   un-morphed leaf geometry — exact under the player (morph ≈ 0 there), a slight
   mismatch only out at the window edge.
1. **`lightBuffer` frame-in-flight ring** — last known CPU/GPU race; same fix as
   the instance buffer (ring `MAX_FRAMES_IN_FLIGHT` deep, index per `beginFrame`).
   Subtle light tearing, not geometry pops. ~10 min, low risk.
2. ~~**Reversed-Z depth**~~ — *Implemented (ADR-0034 Phase 0, reverse-Z branch).*
   GPU projection is `reverseZ() * perspective` (near→1/far→0); the screen pass
   clears depth to 0 and tests Greater; every post-shader sky test became
   `depth <= 0.0` and the linear-depth reconstruction flipped to
   `linearizeReverseZ`; SSR's NDC hit test moved to linear eye space; the skybox
   clip-z and composite sky-ray were flipped. The self-contained reflection-probe
   bake stays forward-Z (own depth buffer). Projection math is unit-tested
   (`test_math`). **Verified on-device:** sky classification, depth debug view,
   sun shadows (needed a CPU-side CSM cascade-fit fix — near/far corners were
   unprojected with the wrong reverse-Z NDC z), and SSR (needed a confidence fix —
   the dielectric-F0 fresnel crushed head-on hits to ~4%; replaced with a
   high-floor grazing term). A color-coded SSR debug view (view 2) was added to
   localize this. The bilateral edge-stop follow-up is DONE (stale here until
   2026-08-05): both blurs now use `bilateralDepthWeight` in
   `post_common.metal`, which linearizes reverse-Z via the camera uniform and
   weights by *relative* depth difference — no forward-Z-era constants remain.
3. **Ambient-only AO (gather/respond split, ADR-0017 Phase 4)** — stop the
   composite multiplying AO into direct sun + emissive so residual AO wobble stops
   being amplified. Needs the ambient term carried separately into the composite.
4. **Local-light shadows** (point/spot) — a shadow atlas with a per-frame budget;
   required for believable interiors. None exist today (sun-only).
5. **Generalized-cylinder tree branches** (see Procgen) — replace SDF/Surface-Nets
   skinning with swept tubes: clean topology (culling back on), real bark UVs,
   exact thin twigs, and a natural home for vertex-shader wind.
6. **World-scale foundation (later, large):** the bounded ~16 km curated world —
   spatial partitioning + chunked terrain LOD + sector streaming + object LOD
   (impostors/HLOD), atmospheric/aerial perspective, horizon-map terrain shadows.
   This is the actual "open world" lift (ADR-0034 / `open-world-foundations-plan.md`);
   reverse-Z (#2) is the first phase and a prerequisite. (Camera-relative rendering
   + origin rebasing are *not* needed at this scale — single precision is exact to
   ~mm; they're the documented upgrade path for an endless/planetary world.)

Minor: shadow-map size fixed at 2048 (expose a 4096 option as a slider).

- **SSAO had no temporal stabilization and reconstructed normals from depth.**
  The depth-reconstructed normal sprayed garbage over thin/edgy geometry
  (foliage), so AO crawled/blocked under motion — the forest's worst artifact.
  *Fixed:* `gtaoCompute` now reads the real view-normal G-buffer (same encoding
  as SSR), and the default direction count went 4→6. **Unverified — Metal/shader,
  macOS-only.**
- **SSAO now has temporal accumulation** (the residual foliage flicker after the
  normal-G-buffer + denser-sampling fixes). Sub-pixel foliage with no AA flips
  each pixel between leaf and background as the camera moves, swinging AO frame to
  frame; *both* values are individually valid, so only accumulating over time
  stabilizes it. *Implemented:* `aoTemporal` kernel reprojects this frame's world
  position into last frame via the previous view-projection, samples a history AO
  texture, clamps it to the current 3×3 neighborhood (TAA-style, suppresses
  ghosting/disocclusion without a history-depth buffer), and blends. Resolved AO
  is kept in `aoHistory` (blit) for next frame; the blend weight is the `Temporal`
  slider (`ssaoParams.temporal`, default 0.9; 0 = off). History is invalidated on
  resize and on the first frame. **Unverified — Metal/shader, macOS-only.**
  Residual risk: at high `temporal` values, fast motion may smear (neighborhood
  clamp bounds it but there's no velocity rejection); dial the slider if so.
- **SSAO was spatially under-sampled (blocky patches that shift under motion).**
  The world radius (1.5m) projects to hundreds of px up close but the screen
  radius was clamped to 64px while only 4 steps sampled it → ~16px-spaced samples
  → coarse AO that reads "stable" as flat grey but shows as shifting blocks once
  multiplied into the lit scene. *Mitigated:* screen-radius cap 64→32px, default
  steps 4→8, directions→6; Steps/Directions sliders widened (16/12) so it's
  tunable live. **Unverified (Metal/shader).** Deeper: SSAO multiplies the WHOLE
  composited HDR (`hdr.rgb *= max(ao,floor)` in fragmentComposite), incl. direct
  sun + emissive, not just ambient — so AO contrast at foliage silhouettes is
  amplified and, with no AA/TAA, can still shimmer. Correct fix needs the
  gather/respond split (ADR-0017 Phase 4) so AO only attenuates indirect light.
- **SSR ignored material roughness — every surface reflected like a mirror.**
  The ray-march took only depth/normal/scene-color (no roughness), and the
  G-buffer normal's `.w` was a wasted constant `1.0`, so rough ground mirrored the
  forest (strong at grazing angles via the Fresnel term) and rough treetops added
  shimmering reflections over the foliage. *Fixed:* the G-buffer now packs
  perceptual roughness into normal `.w`; `ssrRayMarch` fades reflection to 0 by
  `maxRoughness` (default 0.6, `Max Roughness` slider) and early-outs for rough
  pixels (also skips the march for the common case). **Unverified — Metal/shader,
  macOS-only.** Not yet gated by *metallic* (no free G-buffer channel left); fine
  while reflective surfaces are low-roughness dielectrics/water.

- **Shadows: single camera-following ortho map → cascaded shadow maps.** The old
  one 60 m box centered on the camera cut a hard "slice" across the level when
  high up (geometry left the box → forced lit) and gave coarse ~3 cm texels that
  shimmered under motion. *Fixed (Part A):* 3-cascade CSM (`shaders/metal/shadows.metal`
  + `metal_renderer.mm setLights`/shadow pass): the view frustum is split out to
  `shadowParams.distance` (slider), each cascade fit to its sub-frustum's bounding
  sphere, texel-snapped, rendered into a `depth2d_array` slice; the lit pass picks
  the cascade by view depth and cross-fades the seam. Debug "Cascades" view + a
  conservative per-cascade caster cull. **Unverified — Metal, macOS-only.**
  Follow-ups: **reversed-Z** depth (Part B — large-world far-plane precision;
  touches every depth-reading shader's sky-test/linearize), local-light shadows,
  and (much later) virtual shadow maps. Shadow-map size still fixed at 2048.

- **Instance buffer was a single shared buffer with no frame-in-flight guard.**
  The main frame doesn't `waitUntilCompleted` (only the frame-dump path does), so
  ~3 frames are in flight, yet `issuePass` wrote per-frame instance transforms
  into one shared `instanceBuffer` via `[contents]` — the CPU stomped data the GPU
  was still reading, so forest trees popped to a neighbor's (often closer)
  transform for a frame. *Fixed:* ring-buffered `instanceBuffers[MAX_FRAMES_IN_FLIGHT=3]`
  indexed by a per-`beginFrame` counter; `nextDrawable` caps the CPU at ~3 frames
  ahead so a 3-deep ring is always free when reused (no fence). **Unverified —
  Metal, macOS-only.** `lightBuffer` (LightUniforms, written every `setLights`,
  read on the async main pass) has the *same* latent race — light data tears
  rather than geometry so it's subtle/unreported; ring it the same way next.

- **Framerate dips to ~20fps in the arena viewer.** Workable but trending
  down. Suspects, in rough order: the post stack accumulated passes (SSAO,
  SSR, bloom, shadow maps, and now DOF/lens-warp wiring) running at full
  resolution; no instancing for the repeated primitive meshes; per-frame
  mesh-bounds queries per entity in RenderSystem. Needs a frame capture on
  the Mac (Xcode GPU capture) before optimizing blind. Quick levers to try:
  half-res SSAO/SSR, FPS cap comparison, toggling passes in the Debug panel
  to bisect the cost. *The CPU side of that capture now exists (ADR-0077):
  run the arena with `RT_FRAME_STATS=arena.csv`, read it with
  `tools/frame-report.py` — if `render` is fat while draw calls are modest,
  the frame is GPU-bound and the Xcode GPU capture is the next step; if
  update/fixed are fat, attach Tracy and skip the GPU capture.*
- **Realtime depth of field doesn't visibly work** (Metal `dofGather` pass,
  written blind on Linux, default-off). The lens-warp pass (distortion/CA/
  vignette) reportedly works; DOF needs on-device debugging. One of the three
  original suspects is RULED OUT by code inspection (2026-08-05):
  `dofTexture` DOES replace `sceneColorTexture` at composite when active
  (`metal_renderer.mm` binds `dofActive ? dofTexture : ...`), so the remaining
  suspects are the CoC scale (sensor-meters -> pixels) and the depth fetch.
  The offline tracer is the reference: same LensParams produce correct
  thin-lens DOF there.
- **Camera gizmos render in reflections/shadows** (they are plain
  Renderables). Fine until a debug-draw layer exists.
- **Repeated edit/play cycles re-upload level meshes** without freeing the
  previous set — slow GPU-memory leak across mode switches. Fixed by the
  `AssetManager` (ROADMAP 3.1, `docs/asset-system-plan.md`): refcounted, deduped
  mesh ownership with `release` on overwrite and `clear()` on world teardown.

**ADR-0037 follow-ups (perf + tone/grade pass — all Metal-only, viewer-verified, not on CI):**
- **AgX display encode unverified.** `tonemapAgX` (`shaders/metal/post_composite.metal`)
  bakes its own ~2.2 display encode (the minimal-fit convention) and was *not*
  bit-compared to ACES on-device. If AgX reads noticeably darker/brighter than
  ACES at neutral grade, it's a one-line gamma-convention fix. Eyeball on a bright
  outdoor scene.
- **HDR display output is unbuilt.** The composite grade runs in scene-linear
  before the tone map specifically so it's display-agnostic (ADR-0037), but output
  is still SDR (`saturate` + `pow(1/2.2)`). True HDR = extended-range `CAMetalLayer`
  (`wantsExtendedDynamicRangeContent` + RGBA16F + EDR colorspace) and swap the
  encode (gamma→PQ/EDR); the grade carries over untouched.
- **Sky ground tint double-serves lighting + skybox.** `env.skyGround` is both the
  below-horizon sky color *and* (in procedural mode) the ambient ground-bounce
  (`envIrradiance` point-samples `sampleEnvironment(normal)`). The ADR-0037 haze
  blend pulled downward-facing normals bluer. To decouple: keep haze for the
  skybox view ray, use the real earthy tint for the irradiance/normal path.
- **SSAO floor in full-coverage views.** Half-res SSAO (ADR-0037) still costs
  ~16 ms when the screen is all terrain/foliage (no sky early-out) — that's the
  two blurs + temporal resolve, not GTAO samples. Lever: fold/​shrink the blur, or
  a coarser temporal. Sky-heavy views are already 60.
- **Foliage prepass instance cap.** `FOLIAGE_MAX_INSTANCES = 8192`
  (`metal_renderer.mm`); foliage beyond it is silently dropped from the prepass+lit
  pass for the frame. Generous for a frustum, but a very dense close forest could
  hit it — grow the buffer or fall back to the single-pass path on overflow.
- **Live-tuned look values not all baked.** Fog density/color and
  `vegetationDrawDistance` are *level-owned* (live-tunable, **not** persisted to
  settings.json) — bake chosen values into the level JSON. Tonemap op + grade +
  bloom + SSAO params *are* persisted to settings.json. `cdlod.json` ships fog
  `density 0.005`; `chunked.json` `0.0025` — both first-pass guesses.
- **Distant-tree LOD/impostors still owed.** Fog + draw distance hide the far
  forest but we still pay full cost for what's drawn; impostors/HLOD (ADR-0034) are
  the structural scaling lever as density grows. The foliage prepass and SSAO work
  bought headroom but didn't address geometry/vertex throughput.

## Offline tracer

- **Scattered vegetation is not imported.** The `vegetation`/`foliage` blocks
  are handled only by the engine loader (`loadVegetation`), so offline forest
  renders omit the scatter — hero `shape:"tree"` entities and rocks DO import.
  (A previous entry here claimed glTF `"mesh"` entities were skipped; that was
  stale — `addGltfModel` has imported them since ADR-0039, re-verified
  2026-08-05 by rendering arena.json's DamagedHelmet offline, 70k triangles,
  textures intact.)
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

- **SDF-skinned trees glitch in "massive blocks" under lateral camera motion near
  dense overlapping canopy.** Under investigation. *Ruled out so far:* depth,
  view-normal, albedo, and shadow G-buffers all look stable in their debug views;
  L-system generation is deterministic; winding is globally consistent; frustum
  cull is conservative/normalized. A `FLAG_DOUBLE_SIDED` material flag (cull→None)
  was tried and **reverted** — the artifact isn't back-face popping (normals are
  stable). Leading suspect now: depth-tie/z-fight between overlapping tree
  *instances* at near-equal depth (invisible in the depth view, but the lit result
  swaps surfaces). Added debug views (facing green/red, wireframe) to localize it.
  Separately, the **AO map still flickers** at foliage silhouettes even with
  temporal accumulation — SSAO's horizon search is inherently unstable where
  sub-pixel overlapping geometry shifts which surface each sample lands on.
- **Procedural objects aren't shown or editable in the editor** (terrain,
  scattered vegetation). By design for now (ADR-0022): they carry no
  `SourceSpec`, so they're regenerated runtime objects, not document entities.
  Revisit when generators get editable instances (e.g. authored as Lua scripts
  with inspectable params, ADR-0023); until then decide per content type which
  realness tier it gets. (The node graph that was once slated for this was
  removed — ADR-0025; Lua is the single authoring path.)
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

- **Rock generation is duplicated C++ ↔ Lua.** `procgen/rock.{h,cpp}`
  (`generateRock` / `generateRockSdf`, `test_rock.cpp`) is now only reached by the
  level loader's `kind == "rock"` branch, used by exactly one level
  (`forest.json`); `flora.lua` already has `flora.rock`, an explicit Lua port of
  the same SDF generator. With Lua as the single procgen authoring path
  (ADR-0023/0025, node graph already removed), the C++ rock is redundant. Kept for
  now (it works + is tested). To retire: migrate forest.json's rock species to
  `kind:"script"` → `flora.rock(...)`, drop the loader branch + `rock.{h,cpp}` +
  `test_rock.cpp` + their CMake/Make entries. Caveat: visual equivalence of the
  Lua rock is author-verify-on-device, and `test_rock`'s coverage would need to
  move to a `flora.rock` script-VM test.

## Lua flora / forest assembly

- **`loadVegetation` is a 493-line god-function** (213 when this entry was
  written — it has more than doubled). It handles tree/rock/builtin/
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
  `loadVegetation` collapses placements into one `InstanceGroup` per species (mesh
  + baked world matrices); `RenderSystem` does a coarse group reject, then a
  PER-INSTANCE frustum cull (`frustumCullInstances`) so only on-screen plants
  draw, then one `drawMeshInstanced` of the visible subset (default loops
  `drawMesh`; the Metal auto-batcher coalesces). Headless-tested (bucket/collapse/
  per-instance-cull). Remaining: (a) `MAX_INSTANCES = 4096` is a per-pass cap —
  batches beyond it fall back to single draws (degrade, not corrupt), so grow the
  buffer / chunk when forests scale; a *direct* Metal `drawMeshInstanced` is a
  minor opt on top. (b) Per-instance cull rebuilds a `std::vector<Mat4>` per group
  per frame (allocations) and is O(instances); a reusable scratch buffer + spatial
  chunking is the Tier 5 scaling follow-up. The coarse group bounds rarely reject
  (one group spans the region), so the per-instance pass does the real work.
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

## Code health — scanned, judged, parked (2026-08-05)

First full `make health` run (ADR-0077). The same-day fixes: the copied Lua
binding helpers (now `scripting/lua_helpers.h`) and the level-JSON parse
clones between the two loaders (now `engine/level_params.{h,cpp}` — which
also healed a real drift: the offline city parse had silently dropped the
district-road fields). What follows is what the scan found that was judged
REAL but deliberately NOT fixed, with the reason and the intended fix. Line
refs are from the 2026-08-05 scan — re-run `make health` before picking one
up, they drift.

- **Vulkan pipeline boilerplate is the dominant duplication family.** One
  ~22-line pipeline-creation block appears at TEN sites in
  `vulkan_renderer.cpp` (1327, 1598, 1677, 1924, 2177, 2327, 2644, 2983,
  3022, 3546), a 35-line variant at five (1207, 1467, 1822, 2075, 2582),
  plus several smaller echoes (12×4, 10×9, 8×4…) — a few hundred redundant
  lines total. Fix: a `makePipeline`-style helper taking a small
  pass-description struct, absorbing the repeated create-info dance.
  **Deliberately not done in this pass: no Vulkan SDK in the agent
  environment, and editing 3.5k lines of backend blind is the
  43-broken-logging-calls failure mode.** Do it in a session where the
  backend compiles (Linux CI validates real shaders; a Vulkan-capable
  machine verifies) — never blind.
- **WebGPU repeats the same shape at smaller scale.** `createPipeline`
  (`webgpu_renderer.cpp:1616`) is 663 lines with its own internal 12×2 dup
  (1103/1176). When the Vulkan helper lands, port the pattern.
- **Cross-backend near-clones to judge, not auto-fix.**
  `vulkan_renderer.cpp:60-76` ≈ `webgpu_renderer.cpp:71-87` (11 lines) and
  `metal_renderer.h:20-30` ≈ `vulkan_renderer.h:23-33` (10). Backends repeat
  structure by design (parity docs, renderer AGENTS.md); lift a shared
  helper only where it doesn't couple the seams.
- **God-function ranking (seam candidates — each is its own reviewed
  refactor, not a drive-by).** `LevelLoader::load` 1661 lines
  (`level_loader.cpp:2053`; the level_params extraction was the first slice —
  the natural next one is per-shape entity loaders, the giant shape-dispatch
  if/else wants a table). `MetalRenderer::endFrame` 1153 + `initialize` 708
  (`metal_renderer.mm`; a pass-graph split — Metal-only, needs on-device
  verification). `growLotBuildings` 1004 (`city_lots.cpp:897`). The 775-line
  unnamed lambda in `road_net.cpp:469` (naming it is step one).
  `CityRenderSystem::build` 636, `buildCarMesh` 618, `buildMetro` 577,
  `LevelScene::load` 490 — and `loadVegetation` 493, which has its own entry
  above and has DOUBLED since that entry was written.
- **Small intra-file clone pairs — fix on touch, not worth standalone PRs:**
  `alignment.cpp` 18×2 (128/164), `road_constraints.cpp` 12×2 (197/340),
  `city.cpp:164` ↔ `street_kit.cpp:12` 14×2, `city_lots.cpp` 11×2
  (952/984), `city_render.cpp` 10×2 (901/929), `level_scene.cpp` 10×2
  (383/434), `procgen_bindings.cpp` 10×2 (1814/2154).
- **Parses judged intentionally PARALLEL between the two loaders — do not
  force-share.** The lighting/environment blocks (the tracer's `SceneLight`
  list vs the viewer's `SceneLighting` differ in substance: `castsShadow` and
  shadow tuning are viewer-only, the default-noon-sun fallback and HDR sun
  extraction are tracer-only) and the primitive material parse (the loader
  goes through the editor's property layer; the tracer builds its `Material`
  directly). For these, the JSON *field names* are the shared contract — when
  adding a lighting or material field, grep BOTH loaders. Everything that was
  genuinely one parse in two places is now in `level_params`
  (terrain/tree/city/erosion/lots/water params, `parseVec3`/`parseOrientation`,
  `propagateWaterSeaLevel`).
- **Shader scan (2026-08-05, after adding .frag/.vert/.wgsl to the scanner —
  the first scan covered .metal only).** Judged, parked:
  (a) **Every Vulkan shader repeats the same ~36-line uniform-block header**
  (10 files, `dof/mesh/sky/ssao/ssr/terrain/water.*`) — GLSL has no includes
  by default, but glslc supports `#include`; one `common.glsl` would collapse
  it. This block is exactly where the first real Vulkan compile found the
  wrong-uniform-field bug — the highest-value shader dedup, Vulkan-verify
  needed. (b) **The Metal AO and SSR blurs are H/V copy-pairs**
  (`post_ao.metal` 17×2, `post_ssr.metal` 20×2) — a fix applied to one
  direction and not the other would be a subtle axis-dependent artifact;
  fold each into one templated/param'd kernel on next touch. (c)
  `lighting_entry.metal` repeats a 33-line vertex-transform block across its
  three entry variants. (d) Cross-backend clones (metal↔vulkan surfaces/
  common/brdf blocks) are the parity-by-design family — governed by
  `docs/renderer-parity.md`, not for blind dedup.
- **Scanner blind spot (tool debt).** `code-health.py` matches VERBATIM
  (whitespace-insensitive) lines, so a clone with renamed identifiers evades
  it — the two `parseTerrainParams` copies (`tp` vs `p`) were only caught by
  hand-diffing. Upgrade: normalize identifiers to placeholder tokens before
  hashing. Until then, treat the duplicate list as a floor, not a census.
- **Baseline for the trend** (compare future `--json` snapshots against
  this): 367 files / 87,292 lines; top-20 duplication ≈ 670 redundant line
  copies (was 731 before the same-day fixes); 12 debt markers; the fan-in
  heavyweights are `rt_math.h` (57 includers × 629 lines), `renderer.h`
  (44 × 666), `components.h` (35 × 472).

## Hitching: diagnosed suspects (2026-08-06, device-reported)

A real macOS arena capture reads: median 16.70 ms (a clean 60 fps) but p99
116.57 and max 755.13 — **hitching, not steady-state cost**. CPU work is
0.70 ms/frame total; 94% of the frame is `acquire`. Glenn reports the hitches
fire **when the physics gun is used and intermittently while walking**.

Read `acquire` correctly first: it is `nextDrawable` blocking until a
swapchain image frees, which happens when the GPU finishes an earlier frame.
So it is where a stall SURFACES, not where it is CAUSED — anything that makes
the GPU or its queue fall behind shows up as acquire on the FOLLOWING frames.
The ledger now records per-frame `mesh_uploads` / `texture_uploads`
(monotonic counters in `RenderStats`, diffed per frame) so this class stops
being guesswork: a slow frame that uploaded something names its own cause.

**Second capture (arena2, with the upload counters) settled several of these
— including against me:**

- **Mid-play uploads are NOT the hitch. Falsified.** 62 uploads across the
  run, 38 of them on frame 1 (level load); of the 34 slowest frames exactly
  ONE uploaded anything, and that one was frame 1. Suspects 1 and 2 below are
  therefore *not* the cause of the in-play hitches (the pose meshes do upload
  during play — ~24 of them — they just never land on a slow frame). The
  uploadTexture stall and the lazy pose meshes remain worth fixing on
  principle; they are not this bug.
- **`gpu_ms` is contaminated and must not be read as GPU cost.** It reported
  58.4 ms typical against a 33.35 ms typical frame — impossible, since a frame
  cannot finish faster than its own GPU work. The timed command buffer also
  carries `presentDrawable`, so its GPU window includes waiting on the display
  for a free drawable. **Fix:** time a command buffer that does not present —
  submit the render work and the present as two buffers on the same queue
  (ordering is preserved), or move the present to a scheduled handler. Both
  restructure frame submission, and visionOS's `CompositorSurface` brackets
  the frame differently, so this needs a device to verify. Until then the
  report labels the number an upper bound rather than a cost.
- **The biggest hitch was in code no bracket covered.** Frame 3324 took
  307 ms while its phases summed to 30 ms — 90% unattributed. *Fixed:*
  `Poll` (window/OS event pump) and `Dispatch` (event-bus drain + end-of-frame
  state swap) are now bracketed, and the report charts a derived
  **unattributed** band so a gap is visible instead of invisible. A re-capture
  will name that 277 ms.
- **This run's real problem is steady state, not hitching.** Typical `acquire`
  is already 31.80 ms and the median frame is 33.35 ms — vsync-locked at 30 fps
  on *every* frame; spikes add only +16.87 ms on top. Note the previous arena
  capture had a 16.70 ms median (a clean 60 fps) on the same scene, so
  something environmental differs between the two runs (window size, display,
  or power state) and is worth pinning down before optimising anything.

Suspects, in confidence order — none yet confirmed on device:

1. **`MetalRenderer::uploadTexture` stalls the pipeline.** It ends with
   `[cmdBuf waitUntilCompleted]` purely to build mipmaps, so ANY texture
   created during play blocks the CPU until the GPU drains. Command buffers on
   one queue already execute in submission order, so a later render would see
   the mipmaps without the wait — a one-line removal, but Metal-unverifiable
   here, so it is recorded rather than changed blind. Confirm first with the
   new `texture_uploads` column; if slow frames carry a texture upload, this
   is it.
2. **Lazy per-pose player-body meshes.** `CityPlayerBodySystem::poseMesh`
   builds and uploads a mesh the first time each of `kWalkPoses` poses is
   used — i.e. *while you walk*, one hitch per new pose until the set is warm.
   Matches "every so often while walking" exactly. Fix: pre-warm all poses at
   level load (they are a fixed, small set), don't build them mid-play.
3. **The gun's bullets are cache hits** (`acquirePrimitive("box", …)` by key),
   so the mesh is not the cost — but each shot creates a Jolt dynamic body and
   `gun.lua` caps at MAX=500 live bullets, so repeated firing grows the
   physics set. That cost lands in `fixed`, which the capture shows at
   0.20 ms average — check its spike-anatomy row before pursuing.
4. **`spawn.model` uploads without a dedup key** (`script_system.cpp`
   `acquireMesh(*c.mesh)`), as does `spawnVehicle` (commented "unique upload").
   Correct for genuinely unique procgen meshes; a hitch source if a recipe
   spawns repeatedly.

**Third capture (arena3, with poll/dispatch bracketed): the hitches now have
names, and they are three unrelated problems.** `unattributed` fell to 0.00 —
the frame is fully accounted for. The worst frames split into:

- **`dispatch`/state-swap stalls** — frame 2614 spent **241 ms** in the
  event-drain + state-swap bracket (they were one bracket then; now split into
  `dispatch` and `state_swap` so the next capture says which). One occurrence
  in 2644 frames. Prime suspect: pushing `DebugOverlayState` runs ImGui's
  first-time font-atlas build, which goes through ImGui's own Metal upload
  (invisible to our upload counters).
- **`poll` stalls** — 157 ms on frame 1 (boot) and **99 ms on frame 864**,
  mid-session, inside the window/OS event pump. Not engine code; a window
  server or system hiccup. Worth confirming it recurs before chasing.
- **`acquire` spikes** — frames 10/12/29/1141/1374/543 at 50–114 ms, purely
  GPU-side, the same class as the steady-state cost.

**Steady state is unchanged and is the bigger problem:** typical `acquire`
32.37 ms, median frame 33.31 ms — vsync-locked at 30 fps on *every* frame,
with only 0.64 ms of CPU work in the whole frame. Still unexplained: the FIRST
split capture had a 16.70 ms median (60 fps) on the same scene; captures 2 and
3 are both 30 fps. Pin down what differs (window size, display, power state)
before optimising, or every before/after comparison is unreliable.

**Fourth capture (base.csv, 2056x1302 = 2.68M px, all passes on) + two bisect
runs.** Steady state unchanged: median 33.40 ms, typical `acquire` 32.32 ms,
0.8 ms of CPU work in the whole frame — GPU-bound at 30 fps.

New from this one:

- **A 1-second stall.** Frame 1329: 1029.87 ms total, **1014 ms in
  `acquire`** — the GPU (or the display pipeline) went away for a full second.
  It sits in a burst with frames 1363–1381 (84/82/152/310/114 ms, all
  acquire-dominated), so something happened *around* that moment rather than
  steady cost. Worth reproducing before chasing: window focus change, display
  mode switch, or another process taking the GPU.
- **A 296 ms state swap.** Frame 1691 spent **296.60 ms in `state_swap`** —
  the split from `dispatch` paid off immediately: it is a state's `onEnter`,
  not the event bus. Prime suspect remains the debug overlay's first push
  (ImGui font-atlas build, which uploads through ImGui's own Metal path and is
  therefore invisible to our upload counters).
- **`poll` at 240 ms on frame 1** — boot-time OS event pump, expected.
- **Uploads confirmed irrelevant a third time**: 38 of 62 on frame 1, and only
  that one slow frame carries any.

**The pass bisect was broken, and the numbers it produced were nonsense:**
`17.11 / 33.27 / 17.81 / 33.20` (SSAO and bloom "costing" −16 ms) and, at
quarter size, `16.72 / 16.67 / 16.66 / 16.66` (everything "costing" 0.05 ms).
Both are vsync artefacts — the first a frame flipping across the refresh
boundary, the second every configuration finishing early and waiting. *Fixed:*
`Renderer::setPresentSync` (Metal: `CAMetalLayer.displaySyncEnabled`) lets the
bisect unlock presentation for the measurement and restore it after, and the
bisect now refuses to report a ranking when a pass appears to cost negative
time, when the medians look quantised to 60/90/120 Hz, or when the backend
cannot disable sync. **FIRST REAL PASS RANKING (2026-08-06, half-width window, 2.68M px scene,
sync NOT disabled so these are LOWER bounds):**

| pass | cost | share of a 32.44 ms frame |
|---|---|---|
| **bloom** | **7.01 ms** | 22% |
| SSAO | 3.93 ms | 12% |
| SSR | 1.59 ms | 5% |
| (the three together) | 12.53 ms | 39% |

**Bloom is the most expensive post pass — more than SSAO and SSR combined**,
which contradicts the standing assumption in the notes above that SSAO
dominates (the "SSAO floor ~16 ms" entry was never measured, it was inferred).
Bloom is a mip chain: its cost is resolution-driven and the usual fix is fewer
mips / a cheaper downsample, not tuning thresholds. Verify at full size before
acting, and note the third run had `setPresentSync` return false on a Mac
where `CAMetalLayer.displaySyncEnabled` should have worked — worth a look, as
the true costs are higher than these lower bounds.

**Bisect bug that cost a user their bloom (found 2026-08-06, fixed):** the
bisect was driven from inside the Performance panel's `CollapsingHeader`
block, so collapsing the header (or hiding the panel) mid-run froze it with a
pass still disabled — and `onStop` then persisted that measurement state into
`settings.json` as if it were the user's choice, so bloom silently stayed off
on the next launch. *Fixed:* the bisect is a member of `DebugOverlaySystem`
(not a function-local static), `update()` runs unconditionally at the top of
`render()` so it always reaches its end and restores, and `onStop` cancels it
BEFORE saving settings. **If a session was quit mid-bisect before this fix,
`settings.json` may hold `bloom.enabled=false` / `ssao.enabled=false` /
`ssr.enabled=false`** — the Debug panel's Reset Defaults + Save Settings, or
deleting those keys, restores it.

A tool lesson worth keeping: the bisect originally REFUSED this result merely
because the sync flag was off, discarding a sound measurement. Failing to
disable sync understates savings; it does not invalidate them. Only genuinely
artefactual numbers (a negative cost, or medians quantised to a refresh rate)
are refused now.

## Verification gap (the meta-debt)

- **Linux CI is red on arrival: 2/1083 test cases fail only on Linux.**
  The first-ever ubuntu run of the suite (Build & Test workflow, run
  30992735285) passed everything except `drive_freeway_mainline_is_clear` and
  `zoo_acute_four_way` — both road-network drive probes that pass on macOS.
  Likely floating-point precision in freeway-weld/acute-junction geometry (the
  "3 welds is a metric artifact" family). Until fixed or per-platform gated,
  the Linux job's signal reads "expected red", which is CI rot — fix soon.
- **No CI existed until 2026-08-05, and the render backends were never
  compiled in it.** Stages 0+1 of the plan below now run on every push
  (`.github/workflows/build.yml`): Linux compiles the Vulkan backend with
  glslc-validated shaders (asserting the backend was actually selected), macOS
  builds the Metal viewer, runs ctest, and offline-compiles the Metal shader
  library via `tools/check-metal-shaders.sh` (which extracts the runtime
  concatenation list from `metal_renderer.mm`). Stage 2 (headless offscreen
  render + golden-image parity) remains open. Historical context: This is the
  root of the gap below. The Vulkan backend shipped Phases 0–3 *never compiled*
  (no SDK in the loop), and Metal shaders are runtime-compiled, so neither
  `vulkan_renderer.cpp`/`shaders/vulkan/*` nor `shaders/metal/*` is validated until
  someone runs the app on the right machine — the first real Vulkan compile (a
  Windows session) found a reserved-keyword shader bug, a wrong uniform field, and
  43 broken logging calls. **Plan:** `docs/ci-plan.md` — staged (headless
  build+tests → backend compile + offline shader validation on both Vulkan/Metal →
  headless offscreen render + cross-backend golden-image parity via lavapipe/Metal).
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

- **The visionOS branch was verified Metal-only; Vulkan/WebGPU are
  compile-risk-unverified against its shared-code changes.** The branch's
  renderer work is all Metal (`post_composite.metal` sky pass-through,
  skybox cull fix, DoF/bloom NaN guards, visionOS depth-carry clamp), but it
  also touched shared engine code every backend compiles:
  `Application::settingsFilePath()`, the `DebugOverlaySystem::load/saveSettings`
  split (which added bloom/AO/SSR *enable* toggles to settings — Vulkan/web will
  now honor those keys at boot), and a `DayNightSystem` boot log.
  Verified 2026-08-05 pre-merge: the Vulkan backend compiles the branch clean
  on Linux CI (real shaders via glslc); the Emscripten build compiles clean
  from two fresh configures and boots the arena (level load + frame-0 draws
  confirmed). Full web *visual* verification needs a real browser — the
  agent-embedded browser pane starves requestAnimationFrame (the production
  gh-pages site freezes on one frame in-pane too), so in-pane screenshots only
  show each build's boot frame. Two build-tooling traps documented the hard
  way: an incremental emscripten rebuild across a git checkout produces
  silently-broken artifacts (always fresh-configure per commit when bisecting
  web behavior), and the boot frame can render before the canvas size reaches
  the engine (benign under live rAF; it IS the visible frame when starved).
  Two lessons worth porting deliberately, not urgently:
  (a) **one sky** — Metal's composite used to re-derive sky from
  `invViewProjection` (a June workaround outliving its bug) and it broke under
  XR's asymmetric infinite-far projections; Vulkan/WebGPU already pass the
  skybox through (written post-lesson), so they need only a *visual* confirm
  next time they run; (b) **infinite-far robustness** — any shader that
  linearizes reverse-Z depth or unprojects clip z=0 NaNs under an infinite far
  plane (Metal's DoF CoC and bloom did; grep the other backends' post stacks
  when they next get attention). Metal now applies the
  sRGB transfer function exactly once, chosen per target: `CompositeUniforms
  .targetEncodesSRGB` (set from the presentation surface's pixel format) tells
  `encodeForTarget` in `shaders/metal/post_composite.metal` whether the shader or
  the hardware owns it. `shaders/vulkan/composite.frag` and
  `shaders/webgpu/composite.wgsl` still fold the ~2.2 encode into the tone
  mappers and assume a linear-storage target — Vulkan goes as far as *selecting*
  `VK_FORMAT_B8G8R8A8_UNORM` for that reason (`vulkan_renderer.cpp`, "Prefer a
  UNORM swapchain"). That assumption is fine on both today, so this is a
  divergence rather than a bug, but the three backends no longer share a
  contract. Port the flag when either backend next needs an sRGB target — or
  sooner, to keep the ledger honest. **No GPU here for either, so any port is
  compile-verified only.**

- **AgX's display encode is inverted approximately, not lifted.** ACES separated
  exactly: its encode was a trailing `pow(c, 1/2.2)` that moved out unchanged, so
  macOS output is provably identical. AgX's encode lives *inside* the polynomial
  sigmoid fit (`agxContrastApprox`), so `tonemapAgX` recovers linear with a
  closing `pow(c, 2.2)`. Against the shader's own encode that round-trips
  cleanly, but against a hardware sRGB target it differs slightly near black,
  where true sRGB has a linear toe that pure 2.2 does not. A linear-output AgX
  fit would remove the approximation. Only visible with `tonemapOperator = 1` on
  visionOS.

## Terrain placement: the map vs the territory (2026-08-23)

Every "floating object" report in this project's history — buried buildings,
floating park paths, floating alley pavements, walkways with one edge on the
ground — is ONE bug class with three faces. The engine keeps two independent
representations of the ground:

1. **The analytic function** `terrainHeight(params, noise, x, z)` — noise +
   erosion + an ever-growing flatten list (road carves, pads, block grades,
   walkway ramps). Every PLACEMENT samples this.
2. **The rendered/collided mesh** — CDLOD tiles that sample that function at
   grid corners (spacing grows with LOD) and interpolate linearly between.
   Everything STANDS on this.

They agree only where someone forces them to. The three faces:

- **Ordering**: the function keeps changing during the bake (each pass appends
  flattens). Anything placed against an earlier version is stale. The
  buried-buildings fix, the deferred parks, and the deferred alleys are all
  this face; each sculptor has re-discovered it independently because nothing
  enforces "the ground is final now".
- **Sampling**: a tile only touches the function at its corners; a 2 m band
  is invisible to an 8 m cell unless its flatten is stamped (and dilated for
  coarse queries). The walkway band-stamp ramps close this face.
- **Plane conflicts**: flattens blend by falloff and priority; where a band
  runs along a TERRACE LIP the tile's low-side corners legitimately adopt the
  lower plane and the cell's surface dives under the band's high edge.
  Measured by `walkways_adhere_to_the_terrain_a_cdlod_tile_actually_renders`
  at ~5.8 m worst on a 22% fixture slope. NO flatten arrangement fixes this —
  both planes are "correct".

The durable fix is to stop placing against the map: either (a) a hard
FREEZE POINT after which the ground oracle is immutable and furnishing
queries the FINAL function (kills the ordering face by construction — an
assert, not a convention), plus (b) MESH-CONFORMING ribbons: walkways emit
with vertices at every terrain-grid crossing, heights read from the tile's
own interpolation (kills sampling + plane conflicts — the ribbon IS the
rendered surface, offset a few centimetres), or (c) full place-on-mesh:
build terrain first, raycast placements down onto the built mesh. The tile
test above carries the target (< 0.55 m fine-cell) as its documented goal
and pins today's ceiling so the class cannot silently deepen.

### Addendum (same day): what the tile gate localized

With walkways conforming to the tile's own dilated bilinear sampler
(LotParams::groundMeshCell + the dilate-aware groundWith hook,
terrain_lod's step*1.45), the residual floaters localize to paths CROSSING
TERRACE LIPS: adjacent block-grade planes differ by metres inside one
cell, the rendered mesh is a cliff there, and no draping can lay a flat
band on a cliff. The fix is ROUTING, not sampling: the walkway generators
must treat grade steps as obstacles — route spokes/lanes around lips, or
emit stair runs where crossing is intended (sculptPlaza's podium stairs
are the in-codebase pattern). Until then the tile gate pins the ceiling
and prints the worst offender's source and position.

### Addendum (2026-08-24): the instrument measures, the metro is gated

Two moves this round. (1) `RT_GROUND_PROBES=1` is now SELF-MEASURING: every
post scores the analytic height against the finest tile's own bilinear
interpolation, wears the verdict (green flush / orange <=1 m / red beyond),
and the loader logs the histogram + worst offender. Metro at adoption:
9409 probes, 95.5% flush, worst 10.5 m at (-948.75, -57.5) — a thin seam
along the freeway bench cuts, dilation smearing a cut edge across one cell.
`metro_ground_probes_adhere_between_the_seams` (level_tests) asserts that
histogram through the real loader, so the flush floor cannot silently sink.
(2) Walkway strips are tessellated to the RENDERED CELL (segment length
0.75x groundMeshCell, park spokes / plaza walks / alleys): a strip can only
follow a piecewise-bilinear tile if it has a joint inside every cell it
crosses. Fine-cell worst in the tile gate fell 6.5 -> 2.28 m; the coarse
16 m figure grew to 11.74 m by the same move (hugging the real ground means
diverging from a distant tile that smears cliff notches) — far-LOD only,
bounded by the gate, shrinks when lip-aware routing lands.
