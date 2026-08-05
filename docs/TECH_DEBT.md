# Tech Debt / Known Issues

Small-but-real problems parked deliberately while bigger systems land. Add
items with enough context that future-us can pick one up cold; delete items
when fixed (git history is the archive).

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
