# Editor Application — Plan

A standalone authoring application in the Unity/Blender mold: the engine
renders the viewport, native desktop UI surrounds it — hierarchy panel, asset
browser, inspector, menu bar — with a Play button that "compiles" (saves the
document + cooks assets) and runs the game either in the viewport or as a
separate process. The in-engine ImGui tooling stays for what it's good at
(debug overlays, quick in-viewport panels); the application is the
user-facing authoring surface.

**The non-negotiable:** editor viewport and runtime behave 1:1 because they
ARE the same code — one engine library, two hosts. Nothing is emulated.

Successor to `docs/edit-mode-plan.md` (whose document model, SourceSpec/
LevelWriter, picking, and gizmos all carry forward — they become the engine
half of this application). Roadmap cross-reference: Tier 3.6.

**Status:** A1 done, A2 done, A3 substantially done — **and as of 2026-08-23…27
the editor renders, on Linux, on real hardware.** Shell: **Qt 6**
(cross-platform is a requirement; the Vulkan backend is what actually ships it).
This section's long-standing caveat — "written here, verified there", the Mac as
the only machine that could see a pixel — no longer applies: the viewport is
`VulkanViewport`, device-run on an RTX 3080 (Fedora, Qt 6.11).

Landed: the `engine_core` static library (all hosts link it); the abstract
`Window` seam with the GLFW implementation behind `createPlatformWindow()`;
`HostedWindow` (event injection, headless-tested) with a minimal ImGui platform
layer; `Application::begin/runFrame/end` so a host event loop can drive frames; a
`NullRenderer` for backend-less platforms; the Qt shell (`editor_app`: viewport
widget forwarding Qt input, QTimer-driven frames, Play/Esc in-viewport); and the
two native docks that make it an editor rather than a window — `PropertyInspector`
(a Unity-style inspector generated entirely from `describeProperties` +
`ComponentRegistry`, so it knows widgets and never fields; build/sync/write
visitor passes over one field ordering; widgets hold no references into the ECS,
so component-storage moves and entity replacement are safe by construction) and
`CityPlannerPanel` (camera presets, the Bones recipe knobs, graph-only Regenerate
+ explicit Bake, layer toggles, stats — so road tuning is *measured* in the editor
instead of regenerate-and-screenshot loops). Both moc-free. Gated by
`tests/test_property_inspector_qt.cpp` and `tests/test_city_planner_qt.cpp`
(ctest target `editor_qt_tests`).

**Three platform decisions the Linux round forced**, none of which were visible
from the Mac:

- **Qt picks Wayland; the Vulkan viewport needs X11.** The Wayland platform
  plugin hands the app no `Display*`, so the viewport came up blank behind "no
  Xlib display for the Vulkan surface". `preferXcbForVulkanViewport()` defaults to
  the xcb plugin when one is reachable (Xwayland backs the same session); an
  explicit `QT_QPA_PLATFORM` or `-platform` still wins, and a Wayland session with
  no X display gets a warning rather than a blank pane.
- **Every host must install an OS clipboard and name it.** The debug UI's
  copy/paste was a per-host detail that the Qt editor simply did not have — so
  Teleport poses could be produced and not pasted. Each host now installs one
  (`wl-copy`/`wl-paste` under Wayland) and reports which.
- **The editor opens the control socket too.** ADR-0078's channel was a viewer
  feature; an editor that cannot be driven by an agent is an editor that cannot be
  measured by one.

**Still owed:** Mac verification of the NSView + CAMetalLayer + retina path (the
Metal backend accepts an NSView handle — Qt's `winId` — as well as an NSWindow,
but nobody has run it); A4 (asset browser) and A5 (undo/redo, dirty state,
multiple documents) untouched. **A build-system trap worth one line:** the
Makefile owns `build/` (`BUILD_DIR = build`), so `cmake -S . -B build` — which
CLAUDE.md still recommends — collides with the offline tracer's object directory
and fails with "not a CMake build directory". Configure into `build-viewer/`.

A3 landed too: `EditorBridge` (engine-side, headless-tested) attaches while
an EditorState is active and detaches during Play; the Qt shell now has a
Hierarchy dock (two-way selection sync with the viewport ring/gizmo), a
native Inspector (position/scale spin boxes, delete), an Assets dock
(QFileSystemModel over assets/, double-click a .json opens that level), and
a toolbar with Save / Play / Stop driving the document loop through
Application::requestState. Panels poll the bridge (150ms) and gray out while
playing. A4's first slice (Import Asset with glTF/HDR validation) is in.

Arena-building pass: the player is now a placeable **PlayerSpawn** entity in
the editor (green capsule gizmo, pickable, in the hierarchy); LevelWriter
syncs its Transform into the level's "player" block, which the game loader
consumes unchanged — move the capsule, hit Play, start there. Editor camera
is scene-view style (right-drag looks, WASD/QE fly, scroll dollies, F frames
the selection; buttonless free-look stays a game-freecam behavior; the F
detach toggle is disabled in editor states via cameraDetachEnabled).

The property layer is in: describeProperties(component, PropertyVisitor&)
is the single source of truth (FieldMeta carries ranges/log/units/choices/
read-only — semantics, never widgets); JSON read/write visitors round-trip
any described component; a ComponentRegistry (one line per type) enumerates
an entity's components at runtime. The Qt inspector is now fully generated
from it — Unity-style group-box sections per component (Transform, Shape,
Material, Camera lens, Player Spawn), built/synced/written via three visitor
passes that re-resolve through the registry every time (no stored references
into sparse-set storage). Adding a component = struct + describe + one
registration line; it appears everywhere. The follow-on thread landed too
(June 2026): registry add/remove thunks drive Add/Remove Component in both
inspectors; `ImGuiPropertyVisitor` regenerates the in-viewport panels (the
hand-written field code in EditorSystem and CameraPanelSystem's lens section
is gone); the LevelWriter material block, the loaders' material parsing, and
CameraStore's lens block run through the JSON visitors — FieldMeta ids ARE
the file format (legacy "flags" arrays still load); registry post-edit hooks
make SourceSpec.size editable from every inspector (the editor wires
Size -> mesh rebuild at onStart, where it has the renderer); and undo/redo
is in — an `UndoStack` command log on EditorSystem fed from the
writeField/ImGui-visitor chokepoints, gizmo drag ends, add/duplicate/delete,
and component add/remove, applied back through the property layer.
docs/NEXT_STEPS.md has the details and remaining gaps.

---

## Architecture

```
+--------------------------------------------------------------+
|  Editor app (native shell)                                    |
|  menu bar | toolbar (Play / Save / gizmo modes)               |
|  +-----------+ +---------------------------+ +-------------+  |
|  | Hierarchy | |  VIEWPORT                 | | Inspector   |  |
|  | (entity   | |  = the engine, rendering  | | (transform, |  |
|  |  outline) | |  the same World/systems   | |  material,  |  |
|  |           | |  as the runtime; gizmos   | |  lens, ...) |  |
|  |           | |  draw in-viewport         | |             |  |
|  +-----------+ +---------------------------+ +-------------+  |
|  | Asset browser (assets/ tree; import -> cooked outputs)   | |
+--------------------------------------------------------------+
        |  links                                  spawns / swaps
        v                                                v
   libengine  <----- identical code -----v  game runtime (viewer --play
   (ECS, systems, renderer, document)       or play-in-viewport state)
```

Three structural pieces make this work, and two of them are refactors of
things we already have:

### 1. Engine as a library (`engine_core`)

Today CMake compiles overlapping file lists per target. Restructure into a
static library — ECS, systems, camera/lens, document (loader/writer/stores),
renderer seam + Metal backend — linked by four hosts: the game runtime
(today's viewer), the editor app, `run_tests`, and the offline tracer's
shared parts. Pure build hygiene with no behavior change, verifiable on
Linux, and it pays for itself immediately (compile times, one definition of
truth).

### 2. An embedded-window seam (the key enabler)

ADR-0001 already isolates GLFW behind `Window` and hands the renderer an
*opaque native handle*. The editor needs the inverse arrangement: the **host
app** owns the real window/view and the engine renders into a view it is
*given*. Concretely:

- A second `Window` implementation (`HostedWindow`): no GLFW; the shell
  injects events (mouse, keys, resize) translated to the engine's
  backend-neutral `Event`/`InputState`, and provides the native view handle.
  The translation layer is plain code — unit-testable headlessly, which is
  rare for windowing and a direct payoff of the seam.
- `Renderer::initialize` accepts an NSView*/CAMetalLayer* rather than only an
  NSWindow* (small Metal-backend change; it already treats the handle as
  opaque).

After this, "the application uses the engine as a window" is literally true:
`Application` runs unchanged inside whatever view the shell provides.

### 3. The shell itself — the one real decision

The shell needs docking/split panes, tree views (hierarchy), a file-ish
browser (assets), and property panels. Options:

**DECIDED: Qt 6.** The engine is cross-platform by intent — a Vulkan render
backend for PC/Linux is on the roadmap (Tier 5, now firmer) — so the editor
shell must be too. Qt is the industry answer for exactly this (Maya, Houdini,
countless engine tools): native-enough widgets everywhere, QDockWidget/
QTreeView/QFileSystemModel are the docking/hierarchy/asset-browser, and the
viewport embeds via a native window handle per platform (CAMetalLayer on
macOS today, a Vulkan surface later) — the same opaque-handle seam the
renderer already uses. LGPL via dynamic linking; installed via
Homebrew/distro packages and found with find_package(Qt6), NOT vendored (it
is the one dependency too big for the submodule pattern).

Rejected alternatives, for the record: AppKit/Obj-C++ (clean on macOS, but a
dead end against the cross-platform requirement — it would mean a second
shell later); growing the in-engine UI kit into an IDE (docking, tree views,
text editing, file dialogs are years of toolkit work Qt gives away free — a
custom *game* UI kit for HUD/menus remains a separate, worthwhile runtime
item).

### Cross-platform implications beyond the shell

- **Render backends:** the `Renderer` seam was built for this (ADR-0001); a
  `vulkan_renderer.cpp` slot is even stubbed in CMakeLists. Helpfully,
  ADR-0009 chose [0,1] depth — *Vulkan's* convention — so the projection
  math and frustum tests port unchanged.
- **Shaders are the real cross-platform debt:** everything is MSL today. The
  eventual choice is dual-source (MSL + GLSL/HLSL, drift risk) vs a single
  source cross-compiled (HLSL/GLSL -> SPIR-V -> MSL via SPIRV-Cross, or
  slang). Decide when the Vulkan backend starts; until then keep shader
  *logic* well-factored (the common/lighting/post split already helps).
- **Already portable:** GLFW (runtime windowing), Jolt, the entire
  engine_core, the offline tracer, and the test suite (Linux CI is the proof).
  The macOS-only pieces are exactly the Metal backend and gamepad_gc.mm
  (which has a GLFW fallback by design, ADR-0013).

### Selection and edits flow both ways

The shell and the engine share the `World`. A thin `EditorBridge` (C++ class,
engine-side, headless-testable) is the single conduit the shell talks to:
entity list + names (for the hierarchy), selection (viewport click updates the
panel; panel click updates the viewport ring/gizmo — reuses EditorSystem's
selection), component accessors for the inspector, add/delete/duplicate, save/
play requests, and a dirty/changed notification so panels refresh. The ImGui
in-viewport panels keep working through the same systems — they become the
"quick tools" layer, exactly as Unity keeps scene-view overlays.

### Play = compile + run (both flavors)

"Compile the assets" is the document step that already exists: LevelWriter +
camera sidecar (+ future asset cooking, below). Two run modes, both cheap:

- **Play-in-viewport:** the existing `StateTransition` (editor state -> game
  state) inside the embedded engine — today's behavior, kept.
- **Play standalone:** save, then spawn the runtime as a child process
  (`viewer --play <level>`); the editor stays open. This is the "engine as a
  separate window" mode and is ~20 lines once the document is saved.

### Asset import & cooking (grows into ROADMAP 3.1)

v1 asset browser: a read view of `assets/` (levels, models, env), double-click
to open a level, drag/choose a glTF to place it (the editor Add path).
"Import" v1 = copy into `assets/` + validate (glTF parses, HDR decodes) +
register in a simple manifest. Cooking grows behind the same action when
there's something to cook: HDR -> prebaked cubemap + extracted sun (the
runtime already computes these at load — moving them to import time is the
first real "compile" win, and directly attacks load time). Stable asset IDs/
GUIDs, binary mesh formats, dependency tracking: later, driven by need, as
the ROADMAP 3.1 asset system.

---

## Phases

### A1 — Engine as a library + embedded-window seam (start here; no Mac needed)
`engine_core` CMake target; `HostedWindow` (event injection, no GLFW) with
unit tests; `Renderer::initialize` accepts a view handle. Today's viewer
re-targets onto the library unchanged. Framework-agnostic — this phase is
identical whether the shell ends up AppKit or Qt.

### A2 — Shell skeleton (Qt; buildable wherever Qt6 is installed) — **DONE**
Qt app: QMainWindow, menu bar, dock layout with an engine viewport widget
(native window handle handed to the renderer — CAMetalLayer on macOS;
HostedWindow event forwarding from Qt events), toolbar with Save /
Play-in-viewport / Play-standalone. Exit criteria: the full existing editor
(picking, gizmos, ImGui quick panels) runs inside the app's viewport pane and
Play works both ways. Rendering verification still needs the Mac (Metal is
the only backend), but the Qt code itself compiles on Linux.

### A3 — Native panels via EditorBridge — **substantially done** (inspector + city planner landed and gated; hierarchy outline and the Add menu still open)
Hierarchy outline (entities + cameras, rename, two-way selection sync with
the viewport), native inspector (transform, material, primitive size,
physics, camera lens), Add menu in the menu bar. The ImGui Editor panel
shrinks to in-viewport quick tools.

### A4 — Asset browser + import v1
assets/ tree view, open-level, place-model, import-with-validation +
manifest. First cooking step: HDR -> cubemap + sun at import.

### A5 — Editor-grade polish (orders by pain)
Undo/redo (now mandatory — native apps set expectations), document
dirty-state + save prompts, multiple levels open, keyboard map editor,
project settings panel.

## Risks / honesty

- **~~Shell compilation~~ — RESOLVED (2026-08-23).** The bet was that Qt6 dev
  packages might be installable where the macOS SDK is not, making the shell
  Linux-compilable while only the Metal *rendering* needed a Mac. It paid off
  better than written: Qt6 installs *and* the Vulkan backend renders, so the
  whole application — shell, viewport, panels — is now developed and verified on
  Linux, with macOS as the parity target rather than the only eye. The
  "written here, verified there" caveat that shaped this document's first three
  months is retired.
- **The remaining risk inverted.** Metal is now the unverified half: the NSView +
  CAMetalLayer + retina path has code but no run. Every editor feature landed
  from here carries a Metal parity debt in the same sense
  `docs/renderer-parity.md` already tracks for the renderer.
- **Two windowing stacks** (GLFW for the standalone runtime, hosted for the
  editor) share the `Window` seam; input parity bugs are possible — the
  headless event-translation tests are the mitigation.
- **Undo** stops being optional once this looks like a real app (A5, but
  design component edits to route through the bridge early so a command log
  has one chokepoint).
- **Performance:** the 20fps tech-debt item gets more visible inside an
  editor; the frame-capture session should come before or with A2.
