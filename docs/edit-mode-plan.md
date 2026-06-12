# Edit Mode — Plan

A Blender-style editor mode for the viewer: simulation fully stopped, a free
editor viewpoint, click-to-select, transform gizmos (move/rotate/scale), an
Add menu for primitives / glTF models / cameras, an inspector for the selected
entity — and a **Play** button that drops straight into the running game.
Companion to `docs/virtual-camera-plan.md` (the camera workflow becomes one
citizen of this editor). Roadmap cross-reference: Tier 3.5.

**Status:** E1-E5 implemented in one pass: EditorState + StateTransition
(Play saves the document and swaps to the game; Esc swaps back),
SourceSpec + LevelWriter (entities-array patch, round-trip tested),
bounding-sphere picking with a screen-space selection ring, the inspector
(transform/material/size/physics, duplicate/delete), the Add menu
(primitives, glTF by path, cameras), and ImGuizmo gizmos (vendored submodule,
1/2/3 = move/rotate/scale, gracefully absent if the submodule isn't fetched).
**Needs on-device verification on macOS** — especially ImGuizmo's matrix
conventions and retina mouse-pick coordinates. E6 (undo, grid, multi-select)
not started.

---

## The core architectural decision: the level file is the document

Two ways to "pause and edit":

1. **Freeze the live world** (timeScale 0, mutate entities in place, unfreeze).
   Tempting because the clock already supports it (ADR-0002), but editing a
   *running* world is quicksand: physics bodies must be teleported under
   Jolt's feet (PhysicsSystem owns those Transforms, ADR-0012), bullets and
   other runtime spawns linger, and whatever the simulation already knocked
   over is now "the level".
2. **Document-based, Unity-style**: the level JSON (+ camera sidecar) is the
   document. Edit mode operates on a clean world loaded from the document
   with **no physics/gameplay systems running at all**. Play **saves, then
   loads** the document into the game state; Stop returns to the editor,
   discarding runtime changes. Every Play starts from exactly what you built.

This plan chooses (2). It matches the infrastructure we already have (level
loading, sidecar persistence, a state stack with deferred transitions), makes
the edit/play boundary trivially correct, and gives reproducible playtests.
Consequence: the missing piece is a level **writer** — see E2.

## What already exists (the head start)

- **Mode mechanism:** `StateStack` with deferred push/pop (`applyPending`) —
  Edit ↔ Play is a state swap, not new machinery.
- **Editor viewpoint:** fly/orbit controllers + detached freecam with
  UI-aware mouse look. In `EditorState` the camera is simply never pinned.
- **Pause semantics:** not even needed — `EditorState` just doesn't add
  `PhysicsSystem`/`MotionSystem`/`PlayerSystem`/`ShootingSystem`. On Play,
  rigid bodies are created fresh from `Transform` + `Collider`, which is
  already exactly how level load works today.
- **Geometry:** `MeshBuilder` primitives (box/sphere/plane/cylinder/cone/
  wedge/torus/capsule) and `ModelImporter` (glTF) — the Add menu is wiring,
  not new tech.
- **Inspector pattern:** the Cameras panel already live-edits Transforms and
  parameters with PrevTransform sync; generalizing to any entity is mostly
  moving code.
- **Math:** `Mat4::inverse` (pick-ray unprojection), `getMeshBounds` +
  transform (the culling path) for ray-vs-bounding-sphere picking.
- **Persistence:** level JSON loader, camera sidecar save/load, nlohmann
  read+write.

## What is genuinely new

1. **A level writer + authoring metadata (the structural piece).** Loaded
   entities currently forget where they came from (shape, size, model file).
   Add a `SourceSpec` component (shape/size/model path/physics block) filled
   by `LevelLoader` and by the Add menu, so `LevelWriter::save(world, path)`
   can serialize Transform + material + spec back to the same JSON the loader
   reads. Fully headless-testable (round-trip tests like the camera store's).
2. **Mouse picking.** Build a ray from the mouse through the camera
   (inverse view-projection), test against transformed mesh bounding spheres,
   nearest hit wins. Coarse but fine for an arena of primitives; a precise
   triangle test can reuse the offline tracer's KD-tree later if needed.
3. **Transform gizmos.** Recommend vendoring **ImGuizmo** (MIT, single
   header+source, built exactly for this: translate/rotate/scale handles
   drawn through ImGui drawlists). It follows the ADR-0011 pattern — core
   ImGui usage inside a system's `render()`, no backend code — and the
   vendored-dependency precedent (imgui, Jolt, nlohmann, tinygltf). Needs an
   ADR + on-device verification of our column-major/[0,1]-depth conventions
   against its GL-style expectations. Hand-rolling gizmos is the fallback
   (axis-constrained dragging is well-trodden but fiddly) — try ImGuizmo
   first.
4. **The Edit ↔ Play transition.** A small request mechanism (e.g. a
   `StateRequest` on FrameContext or an app-level callback) so the editor's
   Play button can say "save document, swap to ArenaState(level)" and the
   game's Stop can swap back. `StateStack::applyPending` already handles the
   deferred swap safely.

## Phases

### E1 — Mode skeleton (small)
`EditorState`: editor camera (fly/orbit, never pinned), `RenderSystem`,
`DayNightSystem` frozen at a fixed time, level loaded with physics *data* but
no simulation systems. Toolbar window with **Play**; in game, **Esc/Stop**
returns to the editor. Transition request mechanism. Exit criteria: bounce
between editing (everything still) and playing (everything live) on the same
level file.

### E2 — The document (medium, the keystone)
`SourceSpec` component; `LevelLoader` fills it; `LevelWriter` saves world →
level JSON (entities, materials, physics blocks, lighting/environment passed
through); cameras keep their sidecar. Play = save + load. Round-trip unit
tests. After this, *everything the editor does is just component edits.*

### E3 — Selection + inspector (medium)
Click-to-pick (bounding spheres), selected-entity highlight (emission tint
first), inspector section in the Debug window: Transform fields, material
(albedo/metallic/roughness/emission), shape params from `SourceSpec`, Delete /
Duplicate. The camera inspector becomes the `SceneCamera` section of the same
inspector.

### E4 — Gizmos (medium + Mac verification)
Vendor ImGuizmo (submodule + ADR), wire view/proj/model matrices, decompose
manipulation results back to position/orientation/scale, grid snapping
(modifier key), W/E/R (or Blender G/R/S) mode keys. Works on cameras too —
this replaces "nudge numbers in the panel" as the primary placement tool.

### E5 — Add menu (small-medium)
Add → primitive (MeshBuilder defaults), camera (existing path), glTF by path
(ModelImporter; the asset browser can wait). Spawn at a ground-plane raycast
through the view center, selected with the move gizmo active.

### E6 — Polish (later, pick by pain)
Undo/redo (command log over component edits), editor grid + world axes,
duplicate-with-offset, multi-select, precise picking, play-in-place toggle.

## Risks / open questions

- **ImGuizmo conventions** (column-major, depth range, handedness) need one
  on-device session to verify; budget for it in E4.
- **Undo** is deliberately deferred — the document model means a stray edit
  is recoverable by reloading the last save, which makes v1 livable.
- **Runtime-spawned entities** (bullets, future procgen) must not leak into
  the saved document: the writer serializes only entities with `SourceSpec`
  (+ cameras), which handles this by construction.
- **Two states sharing one `World`**: today FrameContext owns a single world
  reference via Application. Loading "fresh" on transition means clearing and
  repopulating that world (a `World::clear()` or recreate seam) — small but
  must be deliberate about handle invalidation (generation bumps already make
  stale handles safe to detect).
