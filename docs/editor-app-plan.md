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

- **Native macOS, Objective-C++ + AppKit (recommended).** The engine is
  already macOS-only at the render layer (Metal), so going native adds no new
  platform constraint. Obj-C++ (.mm) calls the C++ engine directly — no
  bridge layer, no bindings to maintain. NSOutlineView/NSSplitView/
  NSCollectionView are exactly hierarchy/docking/asset-browser, free. The
  precedent already exists in-repo: `metal_renderer.mm`, `gamepad_gc.mm`.
  Cost: this code can only be compiled/iterated on the Mac (like the Metal
  work — written here, verified there).
- **Qt.** The cross-platform answer, and the industry one (Maya, Houdini).
  Right call *if* a second platform backend (Vulkan/D3D, ROADMAP Tier 5) is
  ever serious. Heavy dependency, its own build system gravity, LGPL care.
- **Grow the in-engine custom UI kit into an IDE.** Rejected: docking, tree
  views, text editing, and file dialogs are years of toolkit work that AppKit
  and Qt give away free. (A custom *game* UI kit for HUD/menus remains a
  separate, worthwhile item — that's runtime UI, not tooling.)

**Recommendation: AppKit/Obj-C++.** Decide before Phase A2; Phase A1 is
identical under any choice.

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

### A2 — Shell skeleton (Mac iterations begin)
AppKit app: window, menu bar, NSSplitView with an engine viewport view
(CAMetalLayer hosted, HostedWindow event forwarding), toolbar with Save /
Play-in-viewport / Play-standalone. Exit criteria: the full existing editor
(picking, gizmos, ImGui quick panels) runs inside the app's viewport pane and
Play works both ways.

### A3 — Native panels via EditorBridge
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

- **The shell can't be compiled in the remote environment** (AppKit needs the
  macOS SDK). Pattern as with the Metal passes: written carefully here,
  verified in your Mac sessions — expect a real iteration loop on A2,
  budget for it. Everything in A1 and the EditorBridge is Linux-verifiable.
- **Two windowing stacks** (GLFW for the standalone runtime, hosted for the
  editor) share the `Window` seam; input parity bugs are possible — the
  headless event-translation tests are the mitigation.
- **Undo** stops being optional once this looks like a real app (A5, but
  design component edits to route through the bridge early so a command log
  has one chokepoint).
- **Performance:** the 20fps tech-debt item gets more visible inside an
  editor; the frame-capture session should come before or with A2.
