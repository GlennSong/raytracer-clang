# Session Handoff — Next Steps

Where development stands after the property-layer follow-on session (June
2026), and what to do next. Written as a starting brief for a fresh working
session.

## Current state (all on main)

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
- **Tests**: 170 engine cases (`make test` / ctest) + physics + the Qt
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
2. **Editor quality of life, remaining**: multi-select, camera frustum
   gizmos, axes indicator (grid landed; duplicate shortcut and dirty-save
   prompts landed in the Qt shell).
3. **Asset pipeline (A4 continuation / ROADMAP 3.1)**: cook HDR -> prebaked
   cubemap + extracted sun at import (also cuts load time); asset manifest.
4. **Phase 6 — iPhone virtual camera**: needs a research/planning pass first
   (ARKit pose streaming options vs existing VCam protocols), then the
   PoseSource seam + CameraPuppetSystem + mock replay source
   (docs/virtual-camera-plan.md sketches it).
5. **Vulkan backend** (Tier 5): the seam + [0,1] depth are ready; the real
   decision is shader strategy (dual-source vs SPIRV-Cross/slang
   single-source) — decide before writing.

## Editor <-> engine connections — QoL backlog

The seams that exist today: EditorBridge (selection, document, undo,
creation requests), `FrameContext::debugOverlayActive`,
`Application::simClock()` (the pause/step pattern), registry post-edit
hooks, and the property layer's component-JSON snapshots. Ideas that build
on them, roughly by payoff-per-effort:

1. **Play From Here**: spawn the player at the editor camera instead of the
   PlayerSpawn entity (toolbar button next to Play; pass a spawn override
   into ArenaState). The single biggest iteration-loop win.
2. **Inspect during play**: keep the bridge attached in play as READ-ONLY —
   hierarchy + inspector show live values (transform, physics motion).
   Pairs naturally with Pause/Step: freeze a moment, click around, read
   state. Later: "apply this component back to the document" — the property
   layer's JSON snapshots make that diff nearly free.
3. **Engine -> shell notifications**: the panels poll at 150ms; a small
   event queue on the bridge (selectionChanged, modeChanged, logLine) gives
   instant refresh, a Qt console dock for the engine log, and a status bar
   mode indicator (EDITING / PLAYING / PAUSED).
4. **Gizmo snap settings**: ImGuizmo supports translate/rotate snap
   natively; expose snap increments + grid toggle as Qt toolbar fields.
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
