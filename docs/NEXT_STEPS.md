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
- **Tests**: 167 engine cases (`make test` / ctest) + physics + the Qt
  interaction test (now also covers undo via bridge and the Add/Remove
  Component flow). GLFW + Qt6 dev packages install in the remote env, so
  everything except Metal compiles and smoke-runs on Linux.

Build on a Mac: `brew install qt`, `git submodule update --init --recursive`
(JoltPhysics, imgui, ImGuizmo), then
`cmake -B build -DRT_ENABLE_IMGUI=ON && cmake --build build`.
Targets: `editor_app` (the editor), `viewer` (runtime; `--play` skips the
editor state), `raytracer` (offline), `run_tests`.

## Next steps, in rough priority

1. **Mac verification session** (nothing else depends on more code first):
   - `editor_app` viewport: NSView + CAMetalLayer path, retina scale, input
     feel, ImGuizmo matrix conventions (transpose bridge in
     editor_system.cpp), pick accuracy.
   - New this round: the regenerated ImGui inspector (slider/drag feel,
     ColorEdit3 popup commit-on-close behavior for undo), undo chords inside
     the viewport vs Qt shortcut routing, drag-undo granularity.
   - Tech debt with on-device pointers in docs/TECH_DEBT.md: realtime DOF
     (dofGather), the ~20fps dip (bisect via Debug-panel pass toggles +
     Xcode GPU capture).
2. **Editor quality of life**: editor grid/axes, multi-select, duplicate
   shortcut, save prompts on dirty document (the undo stack's depth at last
   save is a ready-made dirty signal), camera frustum gizmos.
3. **Asset pipeline (A4 continuation / ROADMAP 3.1)**: cook HDR -> prebaked
   cubemap + extracted sun at import (also cuts load time); asset manifest.
4. **Phase 6 — iPhone virtual camera**: needs a research/planning pass first
   (ARKit pose streaming options vs existing VCam protocols), then the
   PoseSource seam + CameraPuppetSystem + mock replay source
   (docs/virtual-camera-plan.md sketches it).
5. **Vulkan backend** (Tier 5): the seam + [0,1] depth are ready; the real
   decision is shader strategy (dual-source vs SPIRV-Cross/slang
   single-source) — decide before writing.

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
