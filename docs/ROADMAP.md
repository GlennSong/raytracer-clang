# Engine Roadmap

A living planning document for the realtime 3D engine. The long-term vision is
a **procedural generation engine** — L-systems, cities, roads, terrain, and
open-world simulation — with procgen driving textures, geometry, materials, and
animation. Physics via **Jolt** (the library behind Horizon Forbidden West).

This roadmap is organized in tiers. Earlier tiers unblock later ones. Within a
tier, items are roughly priority-ordered, but can be tackled in parallel where
noted.

Architecture decisions behind completed work live in `docs/decisions.md`.
Standing code rules live in `AGENTS.md`.

---

## Tier 1 — Foundation

Items that make every subsequent feature faster to build and safer to change.

### 1.1 ImGui integration
**Status:** Implemented (ADR-0011); pending macOS build verification.
**Why first:** Debug UI, parameter tweaking, entity inspection, and an in-game
console all fall out of a single integration. Every subsequent tier benefits
from runtime-tunable controls. ImGui already ships GLFW + Metal backends —
exactly our stack.

**Scope:**
- ✅ Seam designed (ADR-0011): ImGui *core* callable by systems in `render()`;
  ImGui's GLFW/Metal *backends* stay behind `Window`/`Renderer`. No-op hooks
  added (`initDebugUi`/`shutdownDebugUi`/`newDebugUiFrame`), wired in
  `Application`, with a `DebugOverlaySystem` (FPS/entities/camera) that is inert
  until enabled.
- ✅ Build flag `RT_ENABLE_IMGUI` (CMake `option`, OFF) + `third_party/imgui`
  submodule (v1.92.8) — Linux/offline/tests stay green; macOS lights it up.
- ✅ Backend glue written: `ImGui_ImplGlfw_*` in `window.cpp`, `ImGui_ImplMetal_*`
  in `metal_renderer.mm` (drawable/descriptor moved to `beginFrame` so the UI
  frame brackets the scene). API checked against the vendored headers.
- ⏳ **Verify on a Mac** (`cmake -DRT_ENABLE_IMGUI=ON`); the debug overlay should
  appear over the scene.
- ⏳ In-game console wired to a command registry (after the base overlay).

**Delivers for free:**
- **System font rendering** — ImGui bundles a bitmap font and loads TTFs via
  stb_truetype internally. No separate font system needed for debug text.
- **In-game console** — ImGui's demo includes a console widget. Wire it up to a
  command registry and we get a runtime command line with autocomplete.

**Depends on:** Nothing.

### 1.2 Projection math engine-side
**Status:** Done — `Mat4::perspective` / `Mat4::orthographic` in `math.h`,
covered by `tests/test_math.cpp`.
**Why:** Projection matrix construction currently lives in `metal_renderer.mm`.
Moving it to `Mat4` makes the math backend-neutral, Linux-compilable, and
testable. Small, mechanical change — removes tech debt before more features
pile on.

**Scope:**
- ✅ Added `Mat4::perspective(fovYRadians, aspect, near, far)` and
  `Mat4::orthographic(height, aspect, near, far)` targeting Metal's [0,1] depth.
- ✅ The Metal backend now calls these (via the existing `toSimd`) instead of
  its own inline matrix builders, which were deleted.
- ✅ Fixed the pre-existing perspective matrix depth convention (was OpenGL
  [-1,1] on a Metal [0,1] renderer — see ADR-0009); a regression test pins
  near→0 / far→1.

**Depends on:** Nothing. Can parallel with 1.1.

### 1.3 Automated tests
**Status:** Done — `tests/` target with 33 cases; `make test` (and CTest via
the `run_tests` CMake target). Runs on Linux/CI; no GPU deps.
**Why:** `SlotMap`, `SparseSet`, math, and ECS queries have no test coverage.
A `tests/` target is the safety net before bigger refactors.

**Scope:**
- ✅ Minimal hand-rolled runner (`tests/test_framework.h`) — standard library
  only, per AGENTS.md's no-external-deps rule (Catch2 was the alternative).
- ✅ Covers: `Handle`/`SlotMap` lifecycle + recycling/stale detection,
  `SparseSet` add/remove/iterate (swap-and-pop), `World` create/destroy and
  `each` queries (single + intersection), `Vec3`/`Mat4` operations including
  the projection depth mapping (1.2), and `SimClock` stepping / `timeScale` /
  spiral-of-death guard.

**Depends on:** Nothing. Can parallel with 1.1 and 1.2.

---

## Tier 2 — 3D Interaction Infrastructure

The systems needed to move through and interact with a 3D world.

### 2.1 Input-action mapping
**Status:** Done — `InputMap` in `src/engine/input/`, covered by
`tests/test_input_map.cpp`; `DevControlSystem` migrated off hardcoded keys.
**Why:** Keybindings are hardcoded in `DevControlSystem`. An action layer
("move_forward", "toggle_wireframe" → configurable keys) is necessary before
building a real 3D camera or any interactive simulation controls.

**Scope:**
- ✅ `InputMap` abstraction: named button actions and axes bound to
  keys/mouse, driven by the backend-neutral `Event`/`KeyCode` types so it is
  unit-testable without a window.
- ✅ Data-driven binding table via key *names* (`bindButtonByName`,
  `keyCodeFromName`); `DevControlSystem` reads `bind.<action>` overrides from
  `Settings` and falls back to defaults.
- ✅ Continuous queries (`held`, `axis` → summed/clamped to [-1, 1]) and
  per-frame edges (`pressed`/`released`). Wired into `Application`/`FrameContext`
  (`ctx.actions`), updated from the event stream each frame; edges consumed in
  `update()` so a press toggles exactly once.

**Remaining for a follow-up:** `CameraSystem`'s `P` toggle and the orbit
camera's WASD still read keys/`InputState` directly; migrate them to actions
when the fly camera (2.2) lands (its natural consumer).

**Depends on:** Nothing directly, but benefits from ImGui (1.1) for a binding
editor.

### 2.2 Free-fly / FPS camera
**Status:** Done (engine-side) — `FlyCameraController` + `CameraController` seam;
orbit migrated onto it and off direct `InputState`. Feel/sensitivity to be tuned
on a macOS viewer build.
**Why:** The orbit camera can't navigate generated cities or terrain. A
free-fly camera (WASD + mouse look) is essential for exploring procedurally
generated worlds.

**Scope:**
- ✅ A `CameraController` seam (`engine/camera/`) with `CameraInput`; `FlyCamera`
  and `OrbitCamera` controllers both implement it. Pure/window-free, unit-tested.
- ✅ Input routed through the action layer (2.1): movement/look/boost/toggles as
  actions/axes; **gamepad-drivable for free** (sticks/triggers), and the orbit
  camera no longer reads `InputState` directly. Mouse-look/scroll come from the
  pointer (unbounded, so not normalized actions).
- ✅ Runtime toggle between orbit and fly (`Tab` / gamepad Back); per-mode pose
  and projection persisted.
- ⏳ Collision-aware movement deferred until physics lands (2.3).

**Depends on:** Input-action mapping (2.1). Enhanced by ImGui (1.1) for camera
settings.

### 2.3 Jolt Physics integration
**Status:** Foundation done (Steps A + B): math (Quat/TRS/lookAt), Jolt submodule
+ build wiring + a sealed `PhysicsWorld`, verified by headless physics tests.
Next: the ECS `PhysicsSystem` (Step C). See ADR-0012.
**Why:** Rigid bodies, collision detection, and world interaction. Essential for
simulation (objects falling, stacking, vehicles on roads) and for the player
moving through generated environments without clipping through geometry.

**Why Jolt:** Modern C++17 library, active development, proven at scale
(Horizon Forbidden West), clean API, permissive license. Preferred over Bullet
(older API), PhysX (heavier integration), or rolling our own (not worth it).
Cross-platform pure C++ — unlike the window/render backends, it **builds and
runs headless in CI/Linux**, so physics is unit-testable here. Pin to a release
tag (latest stable v5.5.0); GitHub reachable from this environment.

**Scope:**
- ✅ **Math foundation (Step B):** a `Quat` class (compose, `rotate`, `slerp`,
  axis-angle/Euler conversion), `Mat4::trs` and `Mat4::lookAt` (view matrix off
  the backend), Vec3 helpers. `Transform` now stores a quaternion `orientation`
  instead of Euler angles; `MotionSystem`/scene/interpolation migrated. Retires
  the Euler-wobble debt ADR-0006 flagged. Unit-tested.
- ✅ **ADR-0012** for the integration (layers, single-threaded job system, the
  single-precision bridge, the Jolt-free wrapper seam).
- ✅ **Jolt submodule (v5.5.0) + build wiring**; a `PhysicsWorld` pimpl sealing
  Jolt (broadphase/object layers, temp allocator, job system, refcounted global
  init). Viewer CMake target made optional so the project configures headless.
- ✅ **Headless physics tests** (`physics_tests` CMake target): sphere falls,
  rests at radius height, determinism, initial velocity, static body, safe
  invalid handle — all green in Linux/CI.
- ✅ **`PhysicsSystem` (Step C):** `RigidBody` / `Collider` components; creates
  bodies from Transform+Collider; steps in `fixedUpdate` (deterministic, fits
  ADR-0002); captures `PrevTransform` and writes simulated transforms back. Its
  core (`createBodies`/`step`) takes a `World` directly, so it is unit-tested
  headless. `MotionSystem` is **repositioned** as the kinematic mover (cheap,
  collision-free scripted motion) and yields any entity that also has a
  `RigidBody`. Viewer demo: a sphere falls onto the floor while the box keeps
  spinning on MotionSystem.
- ⏳ More shapes (capsule/mesh), materials (friction/restitution), contact
  events, Jolt kinematic bodies (script-driven motion that pushes dynamics).
- ⏳ Debug visualization of colliders — needs a line/debug-draw primitive
  (macOS/Metal); deferred / minimal.

**Depends on:** ECS (done), quaternion math (done). Benefits from ImGui (1.1)
for physics debug viz.

### 2.4 Gamepad & local-player input
**Status:** Engine-side done (gamepad bindings, per-player input, `ControlledBy`);
GLFW polling in `window.cpp` pending macOS verification with a real controller.
See ADR-0010 for the engine/game boundary.
**Why:** Multiple controllers (e.g. four Xbox pads) and the foundation for local
"couch" multiplayer. Built on the action layer (2.1).

**Scope:**
- ✅ Backend-neutral gamepad types (`renderer/gamepad.h`): `GamepadButton` /
  `GamepadAxis` / `GamepadState`, up to `MAX_GAMEPADS` (16).
- ✅ `InputMap` gains device-relative gamepad button/axis bindings, polled
  button-edge diffing, and a stick deadzone.
- ✅ Per-player layer (`PlayerInput` / `PlayerInputs`): device assignment,
  gamepad auto-join/disconnect, event routing (KBM → owning slot, gamepad
  state → owning slot by device id). Exposed as `ctx.players`; system/menu
  controls stay on the global `ctx.actions`.
- ✅ Generic `ControlledBy{ playerIndex }` component — the only engine bridge
  from a player slot to an entity (game adds its own components).
- ⏳ `window.cpp` polls GLFW gamepads and emits connect/disconnect events —
  written, but macOS-only, so unverified in the Linux sandbox.

**Deferred (ADR-0010):** networked multiplayer (transport, authority,
replication, prediction/rollback) — premature; the deterministic sim (ADR-0002)
and ECS keep the door open.

**Next (game layer, not engine):** a `ControlledBy`-driven movement system in a
demo, to exercise split-screen / multi-pad control end to end.

**Depends on:** Input-action mapping (2.1).

---

## Tier 3 — Content Pipeline

The systems that manage, create, and describe renderable content. These must be
designed for **runtime procedural generation from the start** — not as "load
from disk" pipelines with procgen bolted on later. Every asset type should be
constructible programmatically as a first-class path.

### 3.1 Asset / resource system
**Status:** Not started (see tech-debt register: legacy uint32_t handles)
**Why:** The engine needs a unified way to create, own, and reference meshes,
textures, and materials — whether loaded from disk or generated at runtime.

**Scope:**
- Migrate `MeshHandle` / `BufferHandle` from `uint32_t` to `Handle`/`SlotMap`
  (ADR-0007).
- An `AssetManager` that owns GPU resources, supports dynamic creation and
  destruction, and provides typed handles.
- Async loading support (load from disk on a background thread, procgen on any
  thread, upload to GPU on the render thread).

**Depends on:** Handle/SlotMap (done).

### 3.2 Material system
**Status:** Not started
**Why:** PBR materials exist in the Metal renderer but aren't a proper
engine-side abstraction. Procgen needs to create and vary materials
programmatically — noise-driven roughness, weathering, biome-based color.

**Scope:**
- An engine-side `Material` type with PBR properties (albedo, roughness,
  metallic, normal, emissive).
- Material instances (shared base + per-instance parameter overrides).
- Texture slots that accept both loaded textures and procedurally generated
  ones.
- Material component in the ECS.

**Depends on:** Asset system (3.1).

### 3.3 Mesh generation API
**Status:** Not started
**Why:** The bread and butter of procedural geometry — L-system branches,
building facades, road surfaces, terrain patches. The engine needs a clean API
for constructing vertex/index buffers programmatically.

**Scope:**
- A `MeshBuilder` that accumulates vertices, normals, UVs, indices.
- Primitive generators: box, sphere, cylinder, plane, disc.
- Operations: merge meshes, transform vertices, compute normals, generate UVs.
- Produces handles via the asset system (3.1) — no special path for procgen vs
  loaded meshes.

**Depends on:** Asset system (3.1).

---

## Tier 4 — Procedural Generation

The creative core. Each subsystem generates content using the Tier 3 pipeline.

### 4.1 L-systems / grammar framework
A parametric L-system engine: axiom + production rules → symbol strings →
interpreted as geometry (turtle graphics in 3D). Start with simple branching
structures (trees, bushes, coral), then extend to stochastic and context-
sensitive grammars.

**Depends on:** Mesh generation API (3.3), material system (3.2).

### 4.2 Terrain generation
Noise-based heightfield generation (Perlin, Simplex, domain-warped FBM) with
hydraulic/thermal erosion. Chunked LOD for large worlds. Biome assignment
driving material selection.

**Depends on:** Mesh generation API (3.3), material system (3.2). Enhanced by
physics (2.3) for erosion simulation.

### 4.3 Procedural textures and materials
Noise-driven texture synthesis, weathering, aging, and material variation.
Operates on the material system (3.2) to create runtime texture data without
authored assets.

**Depends on:** Material system (3.2), asset system (3.1).

### 4.4 City / road layout generation
Road networks (L-systems, tensor fields, or agent-based), lot subdivision,
building placement and facade generation. This is where L-systems (4.1),
terrain (4.2), mesh generation (3.3), and physics (2.3) converge.

**Depends on:** Terrain (4.2), L-systems (4.1), mesh generation (3.3),
physics (2.3).

---

## Tier 5 — Scale & Polish (future)

Items that become relevant as the world grows large.

- **Scene graph / spatial indexing** — BVH or octree for frustum culling and
  efficient queries over large generated worlds.
- **LOD system** — distance-based level of detail for procgen meshes and
  terrain chunks.
- **Instanced rendering** — draw thousands of generated trees/buildings
  efficiently.
- **Mixed precision / large world coordinates** — revisit ADR-0005 when float
  positions lose precision at world scale.
- **Second rendering backend (Vulkan)** — validate the platform abstraction
  (ADR-0001).
- **Multithreaded systems** — parallel system execution, job system for
  procgen workloads.
- **Custom allocators** — revisit ADR-0008 when allocation churn is measured.

---

## Design Principles

These apply across all tiers:

1. **Procgen is a first-class citizen.** Every asset type (mesh, texture,
   material) must be constructible at runtime — never assume content comes from
   disk.
2. **Debug visualization matters more than game UI.** This is a creative /
   technical tool. Invest in ImGui inspectors, console commands, and debug
   draw over polished player-facing UI.
3. **Measure before optimizing.** Don't add allocators, LOD, or spatial
   indexing until profiling shows they're needed (ADR-0008).
4. **Small, reversible steps.** Each feature is a PR-sized unit with its own
   ADR if the decision is significant. Avoid multi-week branches.
5. **Platform abstraction holds.** New backends are added by implementing a
   seam, not by threading `#ifdef`s (ADR-0001).

---

*Update this document as tiers are completed and new priorities emerge. Add
ADRs to `docs/decisions.md` for significant decisions made during
implementation.*
