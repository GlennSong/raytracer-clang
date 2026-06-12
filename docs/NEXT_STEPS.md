# Session Handoff — Next Steps

Where development stands after the virtual-camera / edit-mode / editor-app
arc (merged to main at `f07bf3e`, June 2026), and what to do next. Written as
a starting brief for a fresh working session.

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
  PlayerSpawn (green capsule -> "player" block), scene-view controls
  (right-drag look, WASD/QE, scroll dolly, F frames selection).
- **Editor application** (docs/editor-app-plan.md, 3.6): Qt 6 shell hosting
  the engine via `engine_core` + `HostedWindow`; hierarchy/inspector/assets
  docks through `EditorBridge`; Save/Play/Stop toolbar; Import Asset with
  validation; **property layer** (`describeProperties` + `PropertyVisitor` +
  `ComponentRegistry` + JSON visitors) generating the Unity-style inspector.
  Headless Qt interaction tests (`editor_qt_tests`).
- **Tests**: 157 engine cases (`make test` / ctest) + the Qt interaction
  test. GLFW + Qt6 dev packages install in the remote env, so everything
  except Metal compiles and smoke-runs on Linux.

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
   - Tech debt with on-device pointers in docs/TECH_DEBT.md: realtime DOF
     (dofGather), the ~20fps dip (bisect via Debug-panel pass toggles +
     Xcode GPU capture).
2. **Property layer follow-ons** (designs agreed, all Linux-verifiable):
   - Add Component menu: registry add/remove thunks + UI.
   - ImGui visitor so in-viewport panels regenerate from describeProperties
     (delete the hand-written panel code).
   - Migrate LevelWriter's material block (and CameraStore lens fields) onto
     the JSON visitors — one source of truth for serialization.
   - Post-edit hooks in the registry (SourceSpec.size -> mesh rebuild) so
     size becomes editable natively.
   - **Undo/redo**: command log at PropertyInspector::writeField + gizmo
     drags + add/delete. The chokepoints exist; this is now mostly bookwork.
3. **Editor quality of life**: editor grid/axes, multi-select, duplicate
   shortcut, save prompts on dirty document, camera frustum gizmos.
4. **Asset pipeline (A4 continuation / ROADMAP 3.1)**: cook HDR -> prebaked
   cubemap + extracted sun at import (also cuts load time); asset manifest.
5. **Phase 6 — iPhone virtual camera**: needs a research/planning pass first
   (ARKit pose streaming options vs existing VCam protocols), then the
   PoseSource seam + CameraPuppetSystem + mock replay source
   (docs/virtual-camera-plan.md sketches it).
6. **Vulkan backend** (Tier 5, now firmer since cross-platform is decided):
   the seam + [0,1] depth are ready; the real decision is shader strategy
   (dual-source vs SPIRV-Cross/slang single-source) — decide before writing.

## Known sharp edges

- Metal lens-warp/DOF passes and the whole editor viewport are written but
  unverified on device (everything else is CI-verified).
- glTF models: offline tracer skips them; editor moves only the first
  sub-mesh of multi-mesh models (no transform hierarchy yet).
- Mesh re-uploads leak across edit/play cycles and size edits (needs a mesh
  cache or removeMesh on clear).
- settings.json carries some cross-mode camera state; harmless but worth
  folding into the document model eventually.
