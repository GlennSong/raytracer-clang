# Session Handoff — Next Steps

Where development stands after the property-layer follow-on session (June
2026), and what to do next. Written as a starting brief for a fresh working
session.

## Current state (all on main)

- **City generation (latest, ADR-0038, `docs/city-generation-plan.md`)** — the
  Tier-4 Phase D city, headless and deterministic, under
  `src/engine/procgen/city/`: a **split/shape grammar** (`shape_grammar`) grows a
  walkable, windowed mid-rise from scope ops (split/repeat/comp/inset/extrude/
  roof/hollow/opening); the **road→block→parcel pipeline** (`road_network` =
  deformed-grid + planarize + half-edge DCEL block faces; `parcel` = recursive-OBB
  lots; `polygon` = 2D geometry); and the **city region recipe** (`city`) with
  district zoning (downtown towers → midtown → residential + parks). Lua
  `building.*` authoring (`assets/scripts/city.lua`), offline render via
  `shape:"city"`, and an HLOD proxy. The **City Arena**
  (`assets/levels/city_arena.json` → `./raytracer --level ... --camera Arena`)
  drapes the city on rolling terrain. The terrain is now **cut/filled to the city**
  (not the reverse): `generateCity` emits `TerrainFlatten` footprints (a ramp per
  road segment, a pad per block) that the loaders feed into `TerrainParams::flatten`,
  so `terrainHeight` grades the ground flat under the roads/blocks — mesh, collider
  and CDLOD field alike — and roads carry a slab thickness so they stand proud of
  the ground. The player walks it on a **Jolt `CharacterVirtual` controller** with
  step-up (curbs/sidewalks/stoops/low boxes are climbable). Both the viewer and the
  offline path spawn the city. Covered by `tests/test_city.cpp`, `test_terrain.cpp`
  (flatten), `tests/test_physics.cpp` (character), `test_script_vm`.
  **Owed:** procedural facade textures (brick/concrete via a shader + world UVs,
  Metal-side); suppressing an external forest scatter under the footprint; Phase 4
  (impostor bake + HLOD swap, sector streaming + cross-tile roads — GPU /
  spatial-partition gated). See the city rows in the ADR register.

- **Procedural city on CDLOD terrain + tensor roads + site selection (latest
  session, ADR-0044)** — a Lua **city recipe now drapes on and grades the engine's
  CDLOD terrain**, not just its own baked ground: a `shape:"script"` entity flagged
  `onTerrain:true` is pre-run with the level's natural `terrainHeight` injected as
  the Lua global `ground`; it samples + `terrain.conform`s it and hands cut/fill
  footprints back via `m:conform(regions)` (`ProcModel` gained a `flatten` channel),
  which both loaders fold into `TerrainParams.flatten` before meshing — the script
  sibling of the C++ city's `onTerrain`. A **tensor-field road generator**
  (`tensorRoads`, `city.layout{pattern="tensor"}`) traces streets along a blended
  radial+grid orientation field so one city morphs from rings/avenues in the core to
  a grid at the rim. **Site selection** (`findFlatSites` / `terrain.flat_sites`)
  floodfills the heightfield for buildable flat valleys (slope+height bitmap →
  connected regions → largest inscribed disc), so cities plant on flat ground, not
  peaks. A **simple L-system tree** (`lsystem_tree.lua`) is instanced as a forest.
  The demo `twin_cities.json` puts a radial Étoile + a tensor city on found sites,
  joined by a low-routed highway, with grounded varied buildings (relief gate + lot
  pad/slab + foundation + district archetypes + coverage/forecourt fit). Covered by
  `test_city.cpp` (tensor), `test_terrain_field.cpp` (flat_sites), `test_script_vm`.
  **Owed:** see "Procedural city — phase 2" under Next steps — this shipped as a
  base to iterate on, not finished.

- **Open-world Phase 1 + interactive render budget (latest session)** — chunked
  terrain → CDLOD heightfield (ADR-0035/0036), vegetation scatter, and a perf +
  look pass (**ADR-0037**), all viewer-verified with the user: foliage depth
  prepass (alpha-cut overdraw), half-res SSAO + temporal jitter (the arena went
  20→60; dense terrain tunable to 60), a display-agnostic grade + selectable
  ACES/AgX tone map, aerial fog with live controls + below-horizon haze, a
  vegetation draw-distance slider, and HDR-env hygiene (clear on level load +
  runtime toggle). Caveats live in `docs/TECH_DEBT.md` (AgX encode unverified,
  HDR output unbuilt, ground-tint coupling, distant-tree impostors still owed).
- **Virtual cameras** (docs/virtual-camera-plan.md, ROADMAP 3.4): placeable
  `SceneCamera` entities, viewport switching (V/B/X, C places, F detaches in
  game), physical `LensParams`, camera sidecar persistence, offline renders
  of placed cameras (`./raytracer --level <json> --camera <name>`).
- **Offline tracer = realtime light model**: GGX PBR, NEE sun/point/spot
  shadows, HDR IBL + dominant-sun extraction, ACES. Cornell unchanged.
- **Edit mode** (docs/edit-mode-plan.md, 3.5): document-based edit/play loop
  (level JSON is the document; Play saves then loads), SourceSpec +
  LevelWriter, bounding-sphere picking, ImGuizmo gizmos (1/2/3), placeable
  PlayerSpawn, scene-view controls.
- **Editor application** (docs/editor-app-plan.md, 3.6): Qt 6 shell hosting
  the engine via `engine_core` + `HostedWindow`; hierarchy/inspector/assets
  docks through `EditorBridge`; Save/Play/Stop toolbar; Import Asset with
  validation; property layer generating the Unity-style inspector.
- **Property-layer follow-ons (this session)** — all five from the previous
  handoff, Linux-verified:
  - *Add/Remove Component*: `ComponentRegistry::Entry` grew `addTo`/
    `removeFrom` thunks (`allowAddRemove<T>`; policy: Camera + Player Spawn
    only). Qt has an Add Component menu button + per-section Remove; the
    ImGui inspector has the same via a popup.
  - *ImGui visitor*: `ImGuiPropertyVisitor` (src/engine/imgui_properties.*)
    renders any described component; EditorSystem's inspector and the camera
    panel's name/lens section are generated now — the hand-written field
    code is deleted.
  - *Serialization through the property layer*: `FieldMeta::id()` carries
    the document key ("focalLength", "albedo", ...); the JSON visitors key
    by it. LevelWriter's material block, level-loader material parsing
    (including glTF per-key overrides), CameraStore's lens block, and the
    offline tracer's material import all share that single description.
    Format note: the checkerboard flag is now `"checkerboard": true` in
    written files; all loaders still read the legacy `"flags":
    ["checkerboard"]` array.
  - *Post-edit hooks*: `Entry::onEdited(world, entity, label)` — inspectors
    dispatch after a write (label = field, null = bulk/undo restore).
    EditorSystem installs Size -> MeshBuilder rebuild at onStart, making
    SourceSpec.size editable from BOTH inspectors (was read-only in Qt).
  - *Undo/redo*: `UndoStack` (src/engine/undo_stack.*) on EditorSystem,
    shared with the Qt shell via the bridge (Edit menu, Ctrl+Z / Shift+Ctrl+Z
    both sides). Commands: component-JSON field edits (captured at the
    writeField / ImGui-visitor chokepoints, drag-granular via ImGui
    activation tracking, consecutive same-field edits coalesce), Transform
    edits (gizmo drag = one entry), create/delete with full component
    snapshots (delete-undo recreates under a fresh handle and remaps the
    rest of the log), component add/remove. Capped at 256 entries.
- **Play-in-editor + shell polish (second pass, same session arc)**:
  - *Playable Play*: the Qt viewport now implements pointer capture —
    `HostedWindow` grew a relative-mouse mode (`injectMouseDelta` +
    cursor-mode callback); `EngineViewport` hides/grabs/warps the Qt cursor
    while the engine asks for `CursorMode::Disabled`. Play/Stop/open also
    return keyboard focus to the viewport (clicking the toolbar used to
    strand WASD in the Qt shell — the "can't move in play mode" bug).
    `EditorState` now explicitly requests a visible cursor (it inherited
    play-mode capture from PlayingState; harmless before only because
    HostedWindow ignored cursor modes).
  - *In-viewport editor UI is shell-aware*: when a bridge is connected the
    ImGui Editor window (Add buttons + inspector) no longer draws — the Qt
    panels own those duties (hosted mode never bridged ImGui keyboard input
    anyway). Gizmo, selection ring, and the new ground grid
    (ImGuizmo::DrawGrid, `editorGrid` setting) still draw. The standalone
    GLFW viewer keeps the full ImGui editor.
  - *Qt additions*: toolbar "Add" dropdown (primitives + camera; creation is
    queued through the bridge and spawned by the editor at the live view),
    Edit > Duplicate (Ctrl+D) / Delete, dirty-document title asterisk +
    save prompt on close (UndoStack revision vs revision-at-last-save).
- **Game-clean presentation + transport controls (third pass)**:
  - Plain play looks like the shipped game: the always-registered debug
    panels (Debug > Cameras) draw only while the backtick overlay is up
    (`FrameContext::debugOverlayActive`, set by Application). The framing
    overlay (thirds/letterbox) stays — it's opted into, not debug chrome.
  - The viewer boots straight into play now (`--edit` starts in the editor;
    `--play` accepted for compatibility). Esc still swaps play <-> editor.
  - Pause moved onto SimClock itself (`setPaused`, orthogonal to timeScale,
    plus `requestStep` for single fixed-step advance), so Space in-game and
    the Qt shell's new Pause/Step toolbar buttons drive one switch
    (`Application::simClock()` is the shell hook). Play always starts
    unpaused.
  - Day/night + clouds run on SIMULATION time now (fixedUpdate, was
    frameDelta): sim pause freezes the sun and cloud drift, Step advances
    them one tick, slow-mo scales them. Their panel (which used to float in
    ImGui's implicit fallback window over plain play) joined the gated
    Debug window; edits there still apply instantly while paused, because
    state is pushed into the view every frame. That's the debug-panel rule
    going forward: knobs apply in update/render (real time), integration
    happens in fixedUpdate (sim time).
- **Play From Here + observe-while-playing (fourth pass)**:
  - *Play From Here* (Qt toolbar + in-viewport button): saves, sets a
    one-shot `playFromHere` settings flag, and ArenaState spawns the player
    at the editor camera (position from the flyEye* settings the camera
    already persists on state exit; facing carries over the same way).
  - *Observer mode*: the bridge now rides into play — ArenaState attaches
    it via `attachObserver` (read-only: `editable()` false, document writes
    refused, no command log, selection held bridge-side). The Qt hierarchy
    stays navigable during a playtest — including a "Player (live)" row —
    and the inspector keeps syncing live values with its widgets grayed.
    Pairs with Pause/Step: freeze a moment, click around, read state.
- **Pick fix + shell reactivity (fifth pass)**: the editor pick ray had its
  near/far unprojection swapped (origin sat ON the far plane looking back,
  so overlapping objects selected furthest-first) — fixed, convention
  pinned by a regression test. Shift-drag gizmo snapping (editor.snap*
  settings). Engine->shell notifications: EditorNotice queue on the bridge
  + `logging::setSink` -> Console dock + EDITING/PLAYING/PAUSED indicator.
- **Editor feel pass (sixth round, from on-device feedback)**:
  - *Precision picking*: BoundingSphere now carries the model-space AABB
    (computed at upload, all backends); the editor picks sphere-broad-phase
    then a local-space slab test — flat things (planes) are only clickable
    where they actually are, and overlap ordering is by true hit distance.
  - *Entity names*: SourceSpec.name ("name" in level JSON, described as the
    Shape section's first field, editable in both inspectors, undo-aware);
    the hierarchy shows it in place of "shape #id".
  - *Environment as document state*: Level > Environment... menu swaps or
    removes the HDR (bridge edits ONLY environment.hdr in the level JSON,
    preserving skyColor etc., then the shell reloads to re-cook IBL/sun).
    Removing it hands lighting back to the procedural sky + day/night.
  - *Shell polish*: toolbar reorganized into document | transport | tools
    groups with standard media icons (video-player order: Play, Play Here,
    Pause, Step, Stop, Restart); inspector dock starts at ~330px with an
    aligned label column, flat group boxes, tighter spacing.
- **Hierarchy / grouping (seventh round)**: stable document ids
  (SourceSpec.id/parentId), `worldMatrix` composition through the parent
  chain (render/pick/gizmo/frame), null-object groups, PLAY-time flattening,
  and a drag-to-reparent `QTreeWidget` with cycle-guarded undo. See the
  dedicated section below.
- **Multi-select (eighth round)**: EditorSystem holds a selection set with a
  primary (gizmo anchor + inspector); shift-click in the viewport toggles,
  the Qt tree is `ExtendedSelection` (Ctrl-click toggles, Shift-click
  ranges). The gizmo moves the whole set rigidly with the primary (one
  compound `TransformEditMulti` undo entry); selection rings draw on all
  (primary brighter). Delete acts on the whole set. The tree-selection
  reconcile is signal-blocked (also smooths first-paint). Drag-reparent
  moves the WHOLE selection and accepts the drop as IgnoreAction so Qt never
  removes/deletes the source rows (an InternalMove MoveAction did, leaving
  dangling QTreeWidgetItem pointers — a crash dragging a multi-selection
  into a group).
- **World-preserving reparent**: reparenting keeps a child's world position
  (Unity's worldPositionStays) — its local transform is rewritten relative
  to the new parent, so dropping objects into a group doesn't teleport them;
  moving the group then carries them by their preserved offsets. The
  Reparent undo command restores both the link and the rewritten transform.
- **Tests**: 189 engine cases (`make test` / ctest) + physics + the Qt
  interaction test (undo via bridge, Add/Remove Component, dirty tracking;
  hosted-window tests cover the relative-mouse capture mode; clock tests
  cover pause/step). GLFW + Qt6 dev packages install in the remote env, so
  everything except Metal compiles and smoke-runs on Linux.

Build on a Mac: `brew install qt`, `git submodule update --init --recursive`
(JoltPhysics, imgui, ImGuizmo), then
`cmake -B build -DRT_ENABLE_IMGUI=ON && cmake --build build`.
Targets: `editor_app` (the editor), `viewer` (the game; boots into play,
`--edit` starts in the editor), `raytracer` (offline), `run_tests`.

## Next steps, in rough priority

0a. **Procedural city — phase 2 (the next big block, from playtest feedback)** —
   the CDLOD-terrain city + tensor roads + site selection shipped to main as a
   *base*, and a walkthrough surfaced real issues. Treat this as a focused phase,
   not a tweak. Known-rough areas, roughly by payoff:
   - **Conform fidelity at the seams.** Buildings are grounded now (foundations +
     lot slabs), but the road↔block↔terrain joins still need a proper pass: curbs
     and sidewalks meeting the graded falloff, the ramp grade (road node heights)
     vs the block pad grade (block-centre height) disagreeing where a block fronts
     a road, and the falloff width vs slab thickness. Re-evaluate how a road
     corridor and an adjacent block pad are reconciled into ONE consistent surface
     (today they are independent `TerrainFlatten` footprints blended by nearest).
   - **Robust lot subdivision for curved/concave blocks.** The tensor generator's
     curved roads make 20-50-vertex block faces that the parcel `inset`/subdivide
     chokes on; `twin_cities.lua` works around it with a Lua collinear-vertex
     simplifier. The real fix belongs in C++: a robust polygon inset + an OBB
     subdivision (or a Voronoi/straight-skeleton parcelling, see the algorithm
     backlog) that handles many-vertex, gently-concave faces directly.
   - **Tensor city rim density.** Its core reads well; the rim blocks are large,
     irregular cells that yield few lots. Wants finer streamline spacing at the rim
     and/or block-splitting so the outskirts populate.
   - **Recipe duplication.** `twin_cities.lua` re-ports `city.lua`'s placement
     (relief gate, pads, slab, foundation, variety) because the Lua sandbox has no
     `require`. Decide on an engine-level recipe-composition mechanism (a shared
     prelude, a registered module table, or promoting the placement into C++
     bindings) so the city pipeline has one home.
   - **Site selection, fuller.** `terrain.flat_sites` works (floodfill → inscribed
     disc) but the recipe biases it near fixed targets. A fuller version: place N
     cities purely from the ranked sites, then route the connecting roads on the
     buildable grid (least-cost path over the slope-weighted bitmap — already the
     top algorithm in the backlog below) so highways follow valleys around the
     mountains instead of a 3-point dog-leg.
   - **Walk-test the feel.** Foundations/curbs/tree capsules and the character
     step-up over sidewalks are unverified on a real viewer (the offline tracer has
     no physics); needs a Mac pass.
   New, reusable substrate this session already added and tested:
   `tensorRoads`/`pattern="tensor"`, `findFlatSites`/`terrain.flat_sites`, the
   `onTerrain` script→CDLOD-terrain conform path (`ground` global + `m:conform`),
   and `lsystem_tree.lua`. See ADR-0044.

0. **Editor recipe studio (a big block, planned)** — the payoff of the Lua
   procgen authoring (ADR-0042/0043). A whole editor system to *interpret and
   view Lua output in semi-realtime* so a human (or an agent over MCP) can
   iterate on recipes: a **recipe preview** panel (pick a `.lua`, run the right
   runner — `runProcgenTexture` → image, `runProcgenMesh`/`runProcgenModelValue`
   → geometry, a `HeightField` → heightmap image + 3D mesh — and display it),
   **hot-reload** on file change, **parameter controls** (a recipe declares its
   tunable params; the editor renders sliders/color pickers), and eventually a
   **visual node graph** (text Lua and the graph are two front-ends to the same
   DAG). Covers materials, heightmaps, and roadways. The headless runners +
   bake paths it needs already exist and are tested (the `--bake-texture` CLI is
   a one-shot version); this block is the desktop/Qt UI on top, so it is
   macOS/desktop-gated and saved as one focused effort.

### Candidate procgen algorithms (backlog — captured, not yet built)

Each is a *generator/primitive* that feeds substrate we already own, so adding
one is mostly the algorithm, not new pipeline. Pull these off the backlog as the
relevant system needs them; do NOT add them before the current systems are
consolidated (a unified `city.lua`, viewer script-scene parity, CDLOD-from-field).

- **Least-cost path (A\*/Dijkstra) over a slope-weighted grid** — the right tool
  for *terrain-aware roads* (follow valleys, switchback up grades) and for routing
  an artist spine intelligently to a target. Sample the heightfield → cost grid →
  path → a `RoadGraph` edge. Pairs with the eroded-valley terrain (low-cost
  corridors). Highest-value next algorithm for roads.
- **Space colonization** — scatter attractor points; grow a network toward them,
  consuming attractors within a kill distance. Two uses for us, both reusing
  existing substrate: (a) *natural trees* — it emits a branch skeleton that
  `skinTree` already turns into bark+leaves, a volume-filling counterpart to the
  L-system; (b) *organic demand-driven roads* — emits a `RoadGraph` into
  planarize→blocks→`road_mesh`. Crown/served-area shape = the attractor cloud.
- **Worley / cellular (Voronoi) noise** — drops straight into `texture_field`
  (stone, scales, cracks, cells) and into terrain (biome cells, mesas). A missing
  noise primitive.
- **Voronoi / Delaunay partition** — organic city districts and lot subdivision
  (an alternative to the current OBB parcels); cracked-surface meshes.
- **Straight skeleton** — proper hip/gable **roofs** from an arbitrary footprint
  polygon; a real building-realism win for the shape grammar.
- **Poisson-disk (blue-noise) sampling** — even-but-random scatter, and the
  attractor distribution that seeds space colonization.
- **Wave Function Collapse** — modular **interiors**/decoration/tilemaps (NOT
  roads — we own the road graph). For building interiors when we get there.
- **Tensor fields** — the other organic-roads approach (orientation field roads
  follow); alternative/complement to space colonization for street networks.

1. **Mac verification session** (nothing else depends on more code first):
   - `editor_app` viewport: NSView + CAMetalLayer path, retina scale, input
     feel, ImGuizmo matrix conventions (transpose bridge in
     editor_system.cpp), pick accuracy.
   - New this round: play-in-editor end to end — pointer capture feel on
     macOS (QCursor::setPos/grabMouse behavior), WASD focus after Play,
     Esc back to edit; the regenerated ImGui inspector in the standalone
     viewer (slider/drag feel, ColorEdit3 popup commit for undo); the
     ground grid; Qt shortcut routing (Ctrl+Z/D, Del) vs viewport keys.
   - Tech debt with on-device pointers in docs/TECH_DEBT.md: realtime DOF
     (dofGather), the ~20fps dip (bisect via Debug-panel pass toggles +
     Xcode GPU capture).
   - ADR-0037 follow-ups to eyeball: AgX vs ACES brightness at neutral grade
     (gamma-convention check); fog/grade values to bake into level JSON.
2. **Render scaling + look (ADR-0037 follow-ups, docs/TECH_DEBT.md)**: distant-
   tree impostors/HLOD (the geometry/vertex lever the prepass+SSAO work didn't
   touch); HDR display output (grade is already HDR-ready — add the extended-range
   layer + PQ/EDR encode); optional decoupled ground-bounce ambient; AgX look
   presets. Lower SSAO blur/temporal floor if dense-terrain views still dip.
4. **Editor quality of life, remaining**: multi-select, camera frustum
   gizmos, axes indicator (grid landed; duplicate shortcut and dirty-save
   prompts landed in the Qt shell).
5. **Asset pipeline (A4 continuation / ROADMAP 3.1)**: cook HDR -> prebaked
   cubemap + extracted sun at import (also cuts load time); asset manifest.
6. **Phase 6 — iPhone virtual camera**: needs a research/planning pass first
   (ARKit pose streaming options vs existing VCam protocols), then the
   PoseSource seam + CameraPuppetSystem + mock replay source
   (docs/virtual-camera-plan.md sketches it).
7. **Vulkan backend** (Tier 5): the seam + [0,1] depth are ready; the real
   decision is shader strategy (dual-source vs SPIRV-Cross/slang
   single-source) — decide before writing.

## Grouping / transform hierarchy — DONE (seventh round)

The transform-hierarchy feature landed:
- **Stable document ids**: `SourceSpec.id` / `parentId` ("id" / "parent" in
  level JSON). Minted on editor create, assigned to id-less entities on load
  and at save (`maxDocumentId` / `nextDocumentId` /
  `assignMissingDocumentIds`); `findByDocumentId` resolves an id to its
  runtime Entity. They survive save/load — the foundation for parenting and
  (next) per-component apply-back + undo-across-reload.
- **World composition**: Transform stays LOCAL; `worldMatrix(world, e)`
  composes up the parentId chain (depth-capped against cycles). Render,
  picking (now a world-space box test), the gizmo (manipulates world, writes
  back through `parentWorldMatrix().inverse()`), selection ring, and F-frame
  all use it. `transformFromMatrix` (shared decompose) reads manipulated
  matrices back and flattens.
- **Null objects / groups**: `SourceSpec.isGroup()` (empty shape, no
  Renderable) — a named transform to parent under. Add via toolbar
  ("empty group") or the in-viewport panel; pick from the hierarchy.
- **PLAY flattens**: the loader bakes each parented entity's composed world
  transform into its Transform and clears parentId in play mode, so the
  runtime (render, physics) never walks a hierarchy and bodies are world-
  space. Editor mode keeps the hierarchy live.
- **Tree hierarchy**: the Qt panel is a `QTreeWidget` built from the
  parentId graph; drag an item onto another to reparent (onto empty = root),
  applied through `EditorBridge::reparent` with a cycle guard and recorded
  as an undo `Reparent` command. `[grp]`/`[cam]` tag prefixes.

Remaining threads off this: multi-mesh glTF parenting under one root (the
importer still moves only the first sub-mesh); camera parenting (SceneCamera
entities don't carry SourceSpec, so they don't parent yet); preserving tree
expansion state across rebuilds.

## Editor UI polish (tenth round, from feedback)

- Inspector: color swatch + OS picker for color fields; object name/kind
  header; default-name placeholder; trash-icon delete in the header
  (replacing the bottom button); section separator lines; clearer
  Add Component button (icon + tooltip).
- Toolbar: icon-only transport cluster with tooltips; Play is a split button
  (click plays, press-and-hold -> Play / Play From Here); Move/Rotate/Scale
  gizmo-mode buttons (exclusive, mirror the engine's 1/2/3 via
  `EditorBridge::setGizmoMode/gizmoMode`); everything tooltipped.
- Hierarchy tree: indentation, alternating row colours, taller rows, and
  type icons (folder / camera / file) instead of `[grp]`/`[cam]` text tags.

## Editor viewport gizmos (ninth round)

- **Group markers**: null objects draw a small cyan wireframe box at their
  world origin (brighter when selected) and are now viewport-pickable via a
  fixed box at that origin (GROUP_MARKER_HALF), not only hierarchy-clickable.
- **Camera frustums**: a selected placed camera draws its framing pyramid
  (apex -> rectangle) from the lens FOV + a focus-clamped draw distance, so
  you can see what it frames. Shared `projectToScreen` / `drawWorldLine`
  background-draw-list helpers back both.

## Editor <-> engine connections — QoL backlog

The seams that exist today: EditorBridge (selection, document, undo,
creation requests), `FrameContext::debugOverlayActive`,
`Application::simClock()` (the pause/step pattern), registry post-edit
hooks, and the property layer's component-JSON snapshots. Ideas that build
on them, roughly by payoff-per-effort:

1. ~~**Play From Here**~~ — DONE (toolbar + in-viewport buttons).
2. ~~**Inspect during play**~~ — DONE: read-only observer attach with live
   hierarchy/inspector + "Player (live)" row; runtime registry rows
   (Velocity, Rigid Body, Controlled By — display-only, never authorable);
   and "Bake" (toolbar, confirm dialog) writes the LIVE play world over the
   document — physics as a level-design tool. Whole-world by design:
   per-component apply needs stable entity ids first.
3. ~~**Engine -> shell notifications**~~ — DONE: EditorNotice queue on the
   bridge (ModeChanged / SelectionChanged / DocumentSaved) drained per
   frame for instant chrome+panel refresh; `logging::setSink` feeds a Qt
   Console dock (tabbed with Assets, thread-safe, bounded); status bar
   shows EDITING / PLAYING / PAUSED.
4. **Gizmo snap settings**: Shift-drag snap landed (editor.snap* settings)
   and the Grid toolbar toggle; remaining: snap-increment fields in the
   shell.
5. **Selection feedback in-viewport**: hover highlight; per-type gizmos for
   the selected entity (camera frustum, spawn capsule, collider bounds).
6. **Restart playtest** button (re-enter the play state without bouncing
   through the editor), and a slow-mo dropdown next to Pause (the clock's
   timeScale knob is already shell-reachable).
7. **Camera bookmarks**: number-keyed editor viewpoints, saved per level.
8. **Eject/possess surfaced in the shell**: the F-detach freecam exists;
   give it a toolbar toggle during play, plus "select what I'm aiming at"
   to feed the play-inspector above.
9. **Runtime stats in the shell**: renderer stats (draws, culled) on the Qt
   status bar via the bridge, replacing the ImGui HUD inside the editor.
10. **Asset hot-reload**: bridge.reimport(path) -> engine swaps the
    mesh/texture in place; belongs with the A4 cook step.

## Known sharp edges

- Metal lens-warp/DOF passes and the whole editor viewport are written but
  unverified on device (everything else is CI-verified).
- Undo scope: the camera-workflow tools that also run during play — the `C`
  place-camera hotkey (CameraSystem) and the Debug > Cameras panel — do NOT
  record onto the editor's command log (they have no editor session to log
  into). Editing through the editor inspectors/toolbar is fully covered.
- Undo of a glTF add/delete covers only the document entity (the first
  sub-mesh) — same multi-mesh limitation as everywhere else (no transform
  hierarchy yet; offline tracer skips glTF).
- Mesh re-uploads leak across edit/play cycles and size edits (needs a mesh
  cache or removeMesh on clear) — size-edit undo/redo makes this easier to
  trigger; same fix covers it.
- settings.json carries some cross-mode camera state; harmless but worth
  folding into the document model eventually.
- The Debug > Cameras panel now lives behind the backtick overlay; the
  tools it carries that the shell doesn't replicate yet (look-through
  preview, offline render button) are candidates for Qt-side homes.
