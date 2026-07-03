# Architecture Decision Records

A living log of the significant engineering decisions behind the realtime
engine, kept so we can review them as the engine grows. Each record states the
context, the decision, the alternatives we weighed, the consequences (including
any tech debt incurred), and an explicit **revisit trigger** — the condition
that should make us reopen the decision.

Status values: **Accepted** (in effect), **Provisional** (in effect but
expected to change), **Deferred** (consciously postponed), **Pending** (chosen
in principle, not yet built).

See also `AGENTS.md` for the standing engineering rules these decisions inform.

---

## ADR-0001 — Platform specifics live behind the Window/Renderer seams
**Status:** Accepted · **Date:** 2026-06-03

**Context.** The engine must run on multiple platforms eventually, but only a
macOS/Metal backend exists today. We needed a rule that prevents windowing/OS
and graphics-backend details from leaking into engine and game code.

**Decision.** All platform specifics sit behind two seams — `Window`
(windowing + input) and `Renderer` (RHI). GLFW is sealed inside `window.cpp`
(pimpl'd so `window.h` carries no GLFW types); the native window handle crosses
to the renderer as an opaque `void*`; events use our own `Event`/`KeyCode`
types, never GLFW codes. New platforms are added by implementing a seam, not by
threading `#ifdef`s through shared code. Codified as a rule in `AGENTS.md`.

**Alternatives considered.**
- Drop GLFW and hand-write per-OS windowing — rejected: huge, low value; GLFW
  is cross-platform and standard.
- Let the renderer reach through GLFW for the native handle (the original
  code) — rejected: couples the backend to the windowing lib.

**Consequences / tech debt.**
- Only one backend (Metal) implemented; the Vulkan path in `CMakeLists.txt` is
  still commented out, so the abstraction is unproven by a second backend.
- `window.cpp` is the single file still allowed to use GLFW; it can't be
  compiled in the Linux CI sandbox (needs GLFW/Metal), so it is verified on
  macOS only.

**Revisit trigger.** Shipping on a non-desktop target (web/mobile/console), or
adding a second backend (Vulkan) — at which point validate the seam holds.

---

## ADR-0002 — Fixed timestep; simulation speed via `timeScale`, not step size
**Status:** Accepted · **Date:** 2026-06-03

**Context.** The sim must be stable and deterministic while the frame rate
varies, and the user wants to speed up / slow down / pause.

**Decision.** `SimClock` accumulates real time and runs a fixed-size step,
exposing an interpolation alpha for smooth rendering. Speed is controlled by a
`timeScale` multiplier on accumulated time — the **step size stays constant**,
so integration behaviour is unaffected when speeding up, slowing down, or
pausing (`timeScale = 0`). A spiral-of-death guard caps catch-up.

**Alternatives considered.**
- Shrink/grow the fixed step to change speed — rejected: changes integration
  accuracy and per-second cost. `setFixedStep` exists only as an
  accuracy/perf knob, not a speed control.
- Variable timestep — rejected: non-deterministic, unstable for physics.

**Consequences / tech debt.** None significant. The interpolation path assumes
each entity stores previous + current transform (see ADR-0006).

**Revisit trigger.** Adding networked/rollback netcode (may need
re-simulation hooks), or deterministic lockstep requirements.

---

## ADR-0003 — Typed, backend-neutral event system (hybrid with polled input)
**Status:** Accepted · **Date:** 2026-06-03

**Context.** Input was polled snapshots only, forcing hand-rolled key-edge
detection and giving no access to OS signals (resize, focus, close).

**Decision.** `Window` produces a per-frame queue of typed `Event`s (key,
mouse, resize, focus, iconify, close) with our own enums. We keep the polled
`InputState` for *continuous* input (camera movement, exposure ramp) and use
events for *discrete* reactions. This hybrid — held-state snapshot **and**
event stream — is the standard engine split.

**Alternatives considered.**
- Replace polling entirely with events — rejected: continuous input genuinely
  wants "is held this frame".
- A pub/sub dispatcher with listener registration — deferred: the per-frame
  queue is enough until multiple independent consumers exist.

**Consequences / tech debt.**
- No event *dispatcher* yet; systems receive events via `System::onEvent`.
- macOS live-resize still stalls inside `glfwPollEvents`; mitigated by the
  window draw callback, but a full fix (refresh-driven redraw) is partial.

**Revisit trigger.** Multiple decoupled event consumers needing
subscribe/filter, or an input-action mapping layer (see register below).

---

## ADR-0004 — `Application` + `System` scheduler is the engine spine
**Status:** Accepted · **Date:** 2026-06-03

**Context.** `main()` *was* the engine — a monolithic loop. Every new
capability (physics, audio, AI) would have meant editing that loop.

**Decision.** An `Application` owns the window, renderer, world, clock, and an
ordered list of `System`s, and drives the frame loop
(`events → update → N fixed steps → render`). Behaviour lives in composable
systems with six lifecycle hooks (`onStart/onEvent/update/fixedUpdate/render/
onStop`). `FrameContext` is the single seam passed to every hook and is
deliberately independent of the entity-storage model, so it survives the move
to an ECS. `main()` is now configuration.

**Alternatives considered.**
- Keep the loop in `main()` and add features inline — rejected: doesn't scale.
- Bake dev controls (pause/quit/time-scale) into `Application` — rejected: kept
  `Application` as pure mechanism; controls are a `DevControlSystem`.

**Consequences / tech debt.** See `RenderView` and `MotionSystem` in the
register — both are interim seams introduced here.

**Revisit trigger.** Needing system ordering/dependencies beyond registration
order, parallel system execution, or fixed/variable systems in separate
schedules.

---

## ADR-0005 — Scalar precision: a global `Real` typedef; float/double deferred
**Status:** Provisional (`Real = double`) · **Date:** 2026-06-03

**Context.** Math is `double` on the CPU while the GPU path is `float` (many
`double→float` casts in `metal_renderer.mm`). A physics/sim engine eventually
needs a deliberate precision choice. The user chose to defer it.

**Decision.** Introduce a single `using Real = double;` in `math.h` and route
`Vec3`/`Mat4`/`Ray` and math functions through `Real`. This makes the
float/double switch a one-line change later, at near-zero cost now (no-op while
`Real == double`). The float-vs-double choice itself is **deferred**.

**Alternatives considered.**
- Hardcode `float` or `double` now — rejected: premature; deferral is cheap.
- Templated `Vec3<T>` / mixed precision (double world coords + float GPU math),
  as in **Unreal Engine 5 Large World Coordinates**, **glm**, **Eigen** —
  deferred: meaningfully more machinery than a light engine needs yet. A global
  `Real` matches **Godot** (`real_t`), **Ogre** (`Ogre::Real`), **Bullet**
  (`btScalar`), and **PhysX** (`PxReal`).

**Consequences / tech debt.**
- A single global `Real` **cannot express mixed precision** — it forces one
  choice for the whole program. If we ever need large-world double coordinates
  *and* float GPU math simultaneously, this typedef must evolve into a
  templated or two-type (`Vec3f`/`Vec3d`) scheme. **This is the most likely
  precision-related tech debt.**
- Literals stay untyped (e.g. `0.5`); only declarations use `Real`, so a future
  flip to `float` incurs harmless double→float conversions, not a literal sweep.

**Revisit trigger.** (a) Large/open worlds where float positions lose
precision; (b) physics needing double-precision integration; (c) GPU-bound
perf where CPU-side float would help. Any of these → reconsider global `Real`
vs templated/mixed precision.

---

## ADR-0006 — Entity model: a lightweight sparse-set ECS
**Status:** Accepted · **Date:** 2026-06-03

**Context.** The world was a `std::vector<Entity>` of fixed structs
(`Transform` + render fields + velocity). For 2D/3D/physics/simulation
generality we want game type to be a *configuration* of components, not a fork.

**Decision.** Adopted a pragmatic sparse-set ECS. Entities are
`Handle<EntityTag>` backed by a `SlotMap` (generation-checked); components
(`Transform`, `PrevTransform`, `Velocity`, `Renderable`) live in per-type
`SparseSet`s; systems query via `World::each<Ts...>(fn)` receiving
`(Entity, Ts&...)`. "Simulated" = has `Velocity`; every renderable carries
`PrevTransform` so render interpolation uses one uniform query. Sparse sets, not
archetypes — flexibility without heavyweight migration machinery.

**Alternatives considered.**
- Lightweight optional-components on the old `World` — viable, lower ceiling.
- Scheduler-only with fixed entity structs — rejected: component set baked in,
  poor fit for 2D vs 3D vs sim.
- Archetype ECS (EnTT groups, etc.) — rejected: more machinery than a light
  engine needs now.

**Consequences / tech debt.** The fixed `Entity` struct and `World::step` are
gone; `MotionSystem`/`RenderSystem` query components. The `System`/
`FrameContext` seam (ADR-0004) survived unchanged, as designed. Known limits:
`each` forbids structural mutation of the iterated component types mid-callback
(documented contract); it iterates the first listed pool (no "smallest pool"
optimization); `each` always passes `Entity` (no component-only overload).

**Revisit trigger.** Entity counts/iteration showing up in a profile (→ pick
smallest pool, or groups); needing safe structural edits during iteration
(→ deferred command buffer).

---

## ADR-0007 — Core primitives: `Handle` + `SlotMap`, logging, assert
**Status:** Accepted · **Date:** 2026-06-03

**Context.** The ECS (ADR-0006) and a future asset manager both need stable,
recyclable identities with stale-handle detection; the codebase logs via
scattered `std::cerr` and has no assert convention.

**Decision.** Added at the core layer (`src/` root, alongside `math.h`):
`Handle<Tag>` (index + generation), `SlotMap<T>` (recycling storage with
generation-checked access), leveled logging (`LOG_INFO/WARN/ERROR`), and
`ASSERT`/`CHECK` macros. Purely additive — existing `MeshHandle`/`World` are
**not** migrated yet (that touches the macOS-only backend; deferred to
Steps 3/5).

**Alternatives considered.** Raw `uint32_t` handles (current `MeshHandle`) —
rejected for entities/assets: no stale detection, no recycling safety.

**Consequences / tech debt.** ~~Two handle styles coexist until migration: the
new `Handle` and the legacy `uint32_t` `MeshHandle`/`BufferHandle` in
`renderer.h`.~~ **Resolved.** `MeshHandle`/`BufferHandle` are now
`Handle<MeshTag>` / `Handle<BufferTag>`, and the Metal backend stores meshes in
a `SlotMap<GPUMesh, MeshTag>` (handing out generation-checked handles, dropping
the old `uint32_t` counter + `unordered_map`). `Renderable.mesh` defaults to a
null handle instead of `0`. Engine-side consumers (`components.*`,
`render_system.cpp`, `viewer_main.cpp`) are headless syntax-verified; the Metal
storage swap is macOS-only, so unverified in CI (the standing backend
constraint). `SlotMap` gained a `clear()`. This is the first piece of the asset
system (ROADMAP 3.1) landing ahead of the manager itself.

**Revisit trigger.** Building the asset manager — it owns these handles and adds
async creation/destruction on top of the now-migrated `Handle`/`SlotMap` base.

---

## ADR-0008 — Defer a custom memory-management system
**Status:** Deferred · **Date:** 2026-06-03

**Context.** Question raised whether to build a memory-management system —
pools, custom allocators (arena/stack), and per-subsystem tracking/leak
detection — to manage and track objects in memory.

**Decision.** Defer it. Pooled object storage is already provided by `SlotMap`
(recycling slots, stable handles, generation-checked access); the ECS
(ADR-0006) builds its component pools on it. Otherwise rely on RAII /
smart pointers (per `AGENTS.md`) and the system allocator. A frame
(linear/bump) arena is the first allocator we'd add — but only when transient
per-frame allocation churn is actually measured. Broader allocators and memory
tracking wait for profiling data or a hard constraint.

**Alternatives considered.**
- Build a frame arena now — rejected: no transient churn to put in it yet;
  speculative.
- Build a full allocator hierarchy + tracking now — rejected: premature without
  profiling data; adds complexity and hides use-after-free/leaks from
  AddressSanitizer/Valgrind, trading away memory-safety tooling for unproven
  speed. Workloads are tiny, single-threaded, desktop (good system malloc).

**Consequences / tech debt.**
- No custom allocators and no per-subsystem memory budgets/leak tracking beyond
  what sanitizers provide. If perf or budgets later bite, this is retrofit work.
- `SlotMap` is the de facto pooling mechanism until then.

**Revisit trigger.** Any of: (a) allocation cost shows up in a profile;
(b) multithreading introduces allocator contention; (c) targeting a console or
a fixed memory budget; (d) measurable transient per-frame churn — add a frame
arena first, then targeted allocators with tracking as data dictates.

---

## ADR-0009 — Camera projection: perspective + orthographic via CameraState
**Status:** Accepted · **Date:** 2026-06-03

**Context.** Only perspective rendering existed. 2D (and CAD-style views) needs
orthographic projection, and the engine should let cameras choose.

**Decision.** `CameraState` carries a `CameraProjection` mode (`Perspective` |
`Orthographic`) plus `orthoHeight`; the renderer builds the matching projection
matrix. `OrbitCamera` exposes an `orthographic` flag and derives `orthoHeight`
from its orbit distance (`2·distance·tan(fov/2)`) so toggling preserves framing
and zoom keeps working. Runtime toggle on `P`, persisted in settings.
Orthographic is the basis for future 2D rendering.

**Alternatives considered.**
- Compute view/projection matrices engine-side (in `Mat4`) and pass them to a
  "dumb" renderer — cleaner and Linux-verifiable, but would disturb the working
  perspective path (depth-range/handedness conventions live in the backend) and
  is a larger interface change. Deferred.
- A separate 2D camera type — deferred; reusing `OrbitCamera` made the feature
  testable on the existing scene without a 2D demo.

**Consequences / tech debt.**
- ~~**Projection-matrix construction still lives in the backend**~~ **Resolved
  (ROADMAP 1.2).** Construction moved engine-side to `Mat4::perspective` /
  `Mat4::orthographic`; the Metal backend builds a `Mat4` and uploads it via the
  existing `toSimd`, so the math is now backend-neutral and Linux-testable. A
  second backend reuses these rather than re-implementing. The **view matrix**
  (`Mat4::lookAt`) was likewise moved engine-side during the quaternion-math
  work, so the backend now holds no matrix math at all.
- No 2D camera controller yet; `OrbitCamera` in ortho mode is the stand-in.
- ~~The pre-existing **perspective** matrix uses the OpenGL [-1,1] depth
  convention~~ **Resolved (ROADMAP 1.2).** Perspective now targets Metal's [0,1]
  depth directly (near→0, far→1), matching the ortho path; a regression test in
  `tests/test_math.cpp` pins the mapping, so close geometry is no longer at risk
  of being wrongly clipped.

**Revisit trigger.** Adding a second backend (de-duplicate projection math, or
move it engine-side), or building real 2D (a dedicated pan/zoom camera).

---

## ADR-0010 — Input/player split: engine owns devices + per-player input, not "players"
**Status:** Accepted · **Date:** 2026-06-04

**Context.** We want multiple gamepads (e.g. four Xbox controllers) and,
eventually, local "couch" multiplayer. The open question was where the line
sits: should the engine define a `Player` object (with its own state) so
multiplayer is "built in", or is that game-specific? Conflating the two would
bake gameplay policy into the engine, against ADR-0006 (game type = a
*configuration of components*, not an engine-defined type).

**Decision.** Split input into three layers by a single litmus test — *"a human
with input devices and their own bindings"* is **engine**; *"a player with
health/score/team/an avatar"* is **game**:

1. **Device layer (engine).** A backend-neutral `GamepadState` (connected,
   buttons, axes) per device, for up to `MAX_GAMEPADS` (16, GLFW's limit). The
   `Window` seam polls the backend and fills these; our own `GamepadButton` /
   `GamepadAxis` enums cross the seam, never GLFW constants (ADR-0001).
2. **Per-player input layer (engine).** A `PlayerInput` slot = a device
   assignment **+ its own `InputMap`** (so per-player rebinding is free).
   `PlayerInputs` manages slots and auto-joins a player when an unassigned pad
   appears. It carries **no gameplay state**. `InputMap` gains device-relative
   gamepad sources (bind to "the A button" / "left stick X"; the player layer
   routes hardware device N to the owning slot), plus a stick **deadzone**.
3. **Game layer (not engine).** The engine provides a generic
   `ControlledBy{ playerIndex }` component as the only bridge: the *game* tags
   whatever entity it likes and adds its own components. The engine never
   decides what a player entity *is*.

System/menu controls (quit, pause) stay on a **global** `InputMap`
(`ctx.actions`); gameplay reads **per-player** input (`ctx.players`). One shared
`World` for all players (see below).

**Alternatives considered.**
- An engine `Player` type owning per-player state — **rejected**: gameplay
  policy in the engine; violates ADR-0006. `ControlledBy` + game components give
  the same capability without the engine prescribing semantics.
- **Per-player `World`/registry** ("each player owns their own ECS state") —
  **rejected**: couch co-op players share one world (they collide, see, and
  interact); separate Worlds fragment the entity space and break every
  cross-player query, physics, and render pass. Players are an *input-routing
  layer over one World*, not a partition of it.
- Bindings that name a specific device index (`gamepad 2's A`) — rejected:
  device-relative bindings + slot routing keep `InputMap` reusable across slots.
- **Networked multiplayer now** — **deferred** (see below).

**Consequences / tech debt.**
- `FrameContext` grows a `PlayerInputs& players` alongside the global
  `actions`. Single-player is just the one-slot case.
- The actual gamepad **polling lives in `window.cpp`** (GLFW) and
  `gamepad_gc.mm` (GCController), both macOS-only and not Linux-compilable —
  same constraint as the rest of the window seam. The GCController layer was
  necessary because macOS 13+ DriverKit intercepts Xbox/PS controllers; see
  ADR-0013. The engine-side parts (`InputMap` gamepad logic, `PlayerInputs`,
  deadzone) are unit-tested without a window.
- No "drive the avatar from a player's axis" system is provided — that first
  touch of gameplay belongs to a game/demo layer, not the engine.
- **Networked multiplayer is explicitly out of scope**: it is a cross-cutting
  commitment (transport, authority, replication/serialization, prediction +
  reconciliation or deterministic rollback) that would distort an engine still
  lacking a second render backend, assets, and physics. Deferring costs little:
  the deterministic fixed-step sim (ADR-0002) and the ECS (ADR-0006) are exactly
  the substrate netcode is built on, so the door stays open.

**Revisit trigger.** Building real local multiplayer gameplay (add the
`ControlledBy`-driven movement system in the game layer); or starting netcode —
at which point reopen ADR-0002 (rollback/lockstep) and design replication over
the single authoritative `World`.

---

## ADR-0011 — Dear ImGui for debug UI, behind the platform seams
**Status:** Accepted (implemented; pending macOS build verification) · **Date:** 2026-06-04

**Context.** The engine needs debug overlays, live parameter tuning, entity
inspection, and an in-game console (ROADMAP 1.1). Dear ImGui is the standard
immediate-mode choice and ships GLFW + Metal backends — exactly our stack. The
question is how to integrate it without violating the platform-abstraction rule
(ADR-0001: no windowing/graphics-backend symbols in engine/game code), and
without breaking the Linux-buildable offline tracer + tests.

**Decision.** Split ImGui by the same seam rule we use everywhere:
- **ImGui *core* (`ImGui::Begin/Text/SliderFloat/...`) is portable** and may be
  called by engine/game systems in their `render()` hook. This is the whole
  point of immediate-mode UI; wrapping it in a neutral abstraction would defeat
  it. It is *not* a graphics/windowing backend, so it does not breach ADR-0001.
- **ImGui's *platform backends* stay behind the seams.** `ImGui_ImplGlfw_*`
  lives only in `window.cpp` (it needs the `GLFWwindow*`); `ImGui_ImplMetal_*`
  lives only in `metal_renderer.mm` (it needs the device/command buffer).
  Neither leaks into engine code.

Frame flow (no extra bracketing system needed): `Window` runs the GLFW
new-frame in `pollEvents`; the Metal backend brackets ImGui *inside*
`beginFrame()`/`endFrame()` (Metal new-frame + `ImGui::NewFrame()` after begin;
`ImGui::Render()` + draw-data submit before present). Systems therefore just
emit ImGui calls in `render()` between the engine's existing begin/end.

Seam surface (no-ops unless built with ImGui):
- `Renderer::initDebugUi(void* nativeWindow)` / `shutdownDebugUi()` — Metal
  side; new-frame/submit are internal to begin/endFrame.
- `Window::initDebugUi()` / `newDebugUiFrame()` / `shutdownDebugUi()` — GLFW side.
- `DebugOverlaySystem` (engine) — the agreed home for debug UI; a no-op `System`
  unless `RT_ENABLE_IMGUI` is defined, then it draws the base overlay
  (FPS, entity count, camera). Other systems may also emit ImGui directly.

**Build.** A CMake `option(RT_ENABLE_IMGUI ... OFF)`. Off (default): the seam
methods are no-ops, no dependency, nothing changes — Linux/offline/tests stay
green. On: add ImGui as a git submodule, compile its core + the glfw/metal
backends, and define `RT_ENABLE_IMGUI`.

**Vendoring.** Git submodule (`third_party/imgui`) pinned to a release tag —
keeps our tree clean and updates explicit. (Vendoring a copy was the
alternative; submodule preferred for a single well-known dep.)

**Alternatives considered.**
- A neutral `DebugUi` wrapper interface over ImGui — rejected: reinvents
  immediate-mode for no portability gain (we have one UI lib).
- A dedicated bracketing `ImGuiSystem` registered first/last — rejected:
  bracketing inside the renderer's begin/endFrame is simpler and order-proof.
- Building it now in this environment — impossible: ImGui's backends need
  GLFW + Metal, which don't compile in the Linux sandbox.

**Implementation note.** The Metal backend defers its whole pass to `endFrame`,
but ImGui widgets are emitted by systems' `render()` (before `endFrame`), and
`ImGui::NewFrame()` must precede them. So `beginFrame` now acquires the drawable
and builds the pass descriptor (needed by `ImGui_ImplMetal_NewFrame`) and starts
the ImGui frame; `endFrame` reuses that descriptor and submits the ImGui draw
data after the scene. The GLFW new-frame runs in `Window::pollEvents`.

**Consequences / tech debt.**
- The backend glue (`ImGui_ImplGlfw_*` in `window.cpp`, `ImGui_ImplMetal_*` in
  `metal_renderer.mm`) is written but **macOS-only, so unverified in CI/Linux**
  — it needs a Mac build (`-DRT_ENABLE_IMGUI=ON`). API signatures were checked
  against the vendored ImGui headers; the engine-side seam, build option, and
  `DebugOverlaySystem` are Linux-verified.
- Modal-resize repaint calls `renderFrame` without a preceding `pollEvents`, so
  the GLFW new-frame is skipped for those repaints (input frozen mid-drag, as it
  already is); ImGui keeps its last display size, no assert.

**Revisit trigger.** Implementing the macOS backend glue (flip the option, add
the submodule, fill the TODOs); or adding a second render backend (give it the
same `initDebugUi`/`shutdownDebugUi` treatment).

---

## ADR-0012 — Jolt physics, sealed behind a Jolt-free `PhysicsWorld`
**Status:** Accepted (Steps A + C done; step now runs on the shared pool, ADR-0014) · **Date:** 2026-06-04

**Context.** The engine needs rigid bodies and collision (ROADMAP 2.3); Jolt was
the chosen library. The questions for *how* to integrate: how to keep Jolt types
out of engine/game code, how to avoid Jolt's compile-define consistency footgun,
the object-layer scheme, the job system, and the precision bridge.

**Decision.**
- **Seal Jolt behind `PhysicsWorld`** (`engine/physics/`), a pimpl whose header
  carries no `JPH::` type — exactly how `Window` seals GLFW (ADR-0001). Engine,
  game, and tests speak only our `Vec3`/`Quat` and an opaque `PhysicsBodyId`.
  The ECS `PhysicsSystem` (Step C) will drive bodies through this wrapper.
- **Git submodule pinned to a release tag** (`third_party/JoltPhysics` @ v5.5.0).
  **Link the `Jolt` CMake target**, not hand-rolled include paths: all the
  `JPH_*` config defines are `PUBLIC` on that target, so they propagate to us
  automatically — which is what defuses the #1 Jolt footgun (mismatched defines
  between the lib and its consumers).
- **Two object layers** (`NON_MOVING`/`MOVING`) with a 1:1 broadphase mapping —
  the standard minimal scheme so the static tree never rebuilds.
- ~~**Single-threaded job system** (`JobSystemSingleThreaded`).~~ **Updated.**
  Jolt's step now runs on the engine's shared `JobSystem` (ADR-0014) via a
  `JoltJobAdapter` (`engine/physics/jolt_job_adapter.*`) — a `JPH::JobSystem`
  that forwards each ready Jolt job to `JobSystem::run` instead of spawning a
  second pool. Jolt still owns the job *graph* (deps/barriers, via the
  `JobSystemWithBarrier` base); we own the *threads*. `PhysicsWorld::initialize`
  takes an optional `JobSystem*`; null keeps the deterministic
  `JobSystemSingleThreaded` path (used by unit tests). Determinism holds —
  same machine + Jolt, multi-threaded simulation is repeatable — and a test
  pins the threaded result equal to single-threaded.
- **Single precision** (`JPH_DOUBLE_PRECISION` off); the wrapper bridges our
  `double` to Jolt `float`. Consistent with the deferred precision choice
  (ADR-0005); revisit for large-world coordinates.
- Jolt's process-global allocator/factory/type registration is **reference-
  counted** across `PhysicsWorld` instances (so multiple worlds / sequential
  tests are safe).

**Verification angle (notable).** Jolt is cross-platform C++ with no GPU/OS
dependency, so — unlike the window/render backends — **it builds and runs
headless here**. A `physics_tests` CMake target links Jolt and asserts real
behaviour (a sphere falls, rests at radius height on the floor, determinism,
initial velocity). To make this configurable on headless/CI machines, the
**viewer CMake target is now optional** (built only when GLFW is found); the
offline tracer, unit tests, and physics tests always configure.

**Alternatives considered.**
- Expose `JPH::` types directly in engine code — rejected: couples the engine to
  Jolt; the wrapper costs little and keeps the door open to swapping later.
- Vendored copy instead of submodule — rejected: submodule keeps our tree clean
  and the version explicit.
- ~~Multi-threaded `JobSystemThreadPool` now — deferred.~~ When we did thread the
  step, the **adapter over our own pool** was chosen over Jolt's stock
  `JobSystemThreadPool` precisely to avoid two competing pools on one machine —
  one pool, with Jolt as a guest that brings its schedule but rents our threads.

**Consequences / tech debt.**
- Adds a real third-party dependency (engine/viewer scope; the offline tracer
  stays std-lib-only). Building it requires `git submodule update --init`.
- **Step C done:** `RigidBody`/`Collider` + `PhysicsSystem` step/write-back;
  `MotionSystem` repositioned (kinematic mover, yields to physics) rather than
  retired. Core logic is headless-tested.
- **Collider debug-draw** needs a line/debug primitive (macOS/Metal) — deferred.
- **`JoltJobAdapter` must be compiled `-fno-rtti`** (set per-source in CMake):
  Jolt is built without RTTI, so a derived class with an out-of-line key function
  would emit typeinfo referencing a base typeinfo symbol Jolt never produced
  (link error). Also: inside a `JPH::JobSystem`-derived class the unqualified name
  `JobSystem` resolves to the Jolt base, so our pool is spelled `::JobSystem`.
- The wrapper currently exposes box/sphere shapes and basic body ops; capsule,
  mesh colliders, materials (friction/restitution), and contact events are
  follow-ups.

**Revisit trigger.** Multithreading the sim (job pool); large-world precision
(double); or a second physics need that the wrapper's surface doesn't cover.

---

## ADR-0013 — GCController gamepad backend for macOS
**Status:** Accepted · **Date:** 2026-06-04

**Context.** GLFW 3.4's IOKit-based joystick backend cannot read Xbox or
PlayStation controllers on macOS 13+. Apple's DriverKit extension
(`com.apple.gamecontroller.driver.XboxGamepad`) claims these devices at the USB
level and re-presents them with a vendor-specific HID descriptor (usage page
`0xFF00`). IOKit sees the device as present but reports 0 axes, 0 buttons, 0
hats — regardless of permissions, gamepad mapping databases, or GLFW version.
Steam works because it has its own HID driver layer bypassing IOKit entirely.

The **only** reliable path on modern macOS is Apple's Game Controller framework
(`GCController`), which speaks the vendor protocol natively.

**Decision.** Add a GCController polling layer (`renderer/gamepad_gc.mm`) that
runs alongside GLFW's existing IOKit path. Architecture:

- **Callback-based.** A `valueChangedHandler` on each controller's
  `GCExtendedGamepad` caches the latest input snapshot in a static
  `GCCachedPad` array. Polling alone does not work — GCController delivers
  input updates asynchronously through the Cocoa run loop, which GLFW's
  `glfwPollEvents()` does not fully service.
- **Run loop pump.** `gcPollGamepads()` explicitly services `NSRunLoop` each
  frame (`runMode:beforeDate:`) so GCController's internal dispatch delivers
  pending input events and connect/disconnect notifications.
- **Notification-based connect/disconnect.** Listens for
  `GCControllerDidConnectNotification` / `GCControllerDidDisconnectNotification`
  rather than re-enumerating `[GCController controllers]` each frame. On
  connect, a slot is assigned and the `valueChangedHandler` installed; on
  disconnect, the slot is released.
- **Overlay, not replacement.** `gcPollGamepads()` runs after GLFW's gamepad
  loop in `Window::pollEvents()` and overwrites any slot where GCController has
  data. Controllers that GLFW's IOKit path handles natively (non-Apple-claimed
  devices, or future GLFW versions with GCController support) continue to work
  unchanged. On non-Apple platforms, `gcPollGamepads` is an inline no-op.
- **Y-axis negated** to match GLFW convention (stick-up = negative), so existing
  camera bindings work without modification.
- **`gamecontrollerdb.txt`** (SDL_GameControllerDB) loaded at init as a fallback
  for GLFW's IOKit mapping path. Harmless and provides coverage for any
  controller that IOKit *can* still see.

Adheres to ADR-0001: `gamepad_gc.mm` is a platform-specific file behind the
Window seam; the engine sees only `GamepadState`. The header (`gamepad_gc.h`)
compiles to an inline no-op on non-Apple platforms.

**Alternatives considered.**
- Building GLFW from source with GCController support — rejected: GLFW 3.4
  (including `main` branch) has no GCController backend; it is IOKit-only on
  macOS.
- Writing our own IOKit HID layer to parse the vendor-specific descriptor —
  rejected: reverse-engineering Apple's proprietary protocol is fragile and
  duplicates what GCController already does.
- Using SDL2 for gamepad input — rejected: heavy dependency for a single
  feature; GCController is ~100 lines of Obj-C++ and is the official Apple API.
- Polling `gp.leftThumbstick.xAxis.value` directly without callbacks — tried
  first; values always read 0 because the run loop was not servicing
  GCController's internal dispatch. The callback approach solved this.

**Consequences / tech debt.**
- `gamepad_gc.mm` is Objective-C++ and macOS-only. It links
  `GameController.framework`, added to `CMakeLists.txt` under the Apple
  platform block.
- The `GCCachedPad` cache is written from GCController's dispatch queue and
  read from the main thread. `GamepadState` is small and reads happen between
  frames, so a torn read produces at most one frame of mixed old/new data —
  acceptable for input.
- Controllers already plugged in at launch require a run loop pump during
  `gcInit()` to deliver the pending connect notifications before enumeration.
  If a controller is still not detected at launch, a physical reconnect
  triggers the notification reliably.

**Revisit trigger.** GLFW adding native GCController support (eliminating the
need for this layer); targeting non-Apple platforms that need a similar
workaround (→ consider SDL2 at that point); or Apple changing the DriverKit
protocol (unlikely to break GCController, since it's their own framework).

---

## ADR-0014 — A minimal shared-queue `JobSystem` for parallelism
**Status:** Accepted · **Date:** 2026-06-05

**Context.** Parallelism was ad-hoc: the offline tracer hand-rolled
`std::thread`-per-tile in `main.cpp`, and Jolt runs on its own single-threaded
job system (ADR-0012). There was no shared place that owns threads. Every future
parallel workload — parallel ECS systems (ADR-0004's revisit trigger), async
asset loading, procgen fan-out (Tier 4) — would otherwise re-invent thread
management. We wanted the foundational threading primitive *before* the asset
system, since async loading is built on it. (`docs/ROADMAP.md` lists a job system
under Tier 5; it was pulled forward as low-level foundation by user direction.)

**Decision.** A `JobSystem` (core layer, `src/job_system.{h,cpp}`, std-lib only
per AGENTS.md) owning a fixed pool of worker threads that drain **one shared,
mutex-guarded queue**. Public surface kept deliberately small:
- `parallelFor(begin, end, body, grainSize)` — splits a range into grain-sized
  chunks, runs them across the pool, and blocks until done. The calling thread
  helps drain while it waits, so no worker idles behind the caller.
- `run(task, counter)` / `wait(counter)` — the lower-level async primitive
  (counter-based completion) that async asset loads will sit on later.
- `AUTO` worker count = `hardware_concurrency() - 1` (leave a core for the
  caller); **0 workers = synchronous mode** (tasks run inline, no threads),
  which keeps tests deterministic and single-core machines correct.

The internals are intentionally simple: **no work stealing**, one queue. The API
hides the queue so the internals can become per-worker deques later without
touching callers — but only when a profile asks for it (ADR-0008: measure
before optimizing).

**Determinism (defends ADR-0002).** The scheduler must never change *results*.
`parallelFor` is contracted as safe only over **independent** work (each index
writes its own data; no order-dependent reductions). Fixed-step ECS systems stay
serial unless a system explicitly opts into `parallelFor` over independent
entities. The contract is documented, not enforced.

**First consumer.** The offline tracer's per-tile `std::thread` loop was migrated
onto `run()` + a completion counter (rows are tasks; the main thread reports
progress, then `wait()`s). This deleted the hand-rolled threading and is the
real, **headless/Linux-verifiable** proving ground — a full Cornell-box render
matches the old output, with `user`≈`3×real` confirming the parallelism.

**Alternatives considered.**
- **Work-stealing deques now** — rejected: premature (ADR-0008); no measured
  contention, and our workloads (image rows, future asset loads) are coarse
  enough that one queue is fine. The API leaves the door open.
- **Reuse Jolt's job system** — rejected: it's sealed behind `PhysicsWorld`
  (ADR-0012) and tuned for physics jobs; coupling general engine parallelism to
  the physics dependency is backwards. (Later we may *give* Jolt an adapter over
  this pool — see revisit.)
- **`std::async` / per-task `std::thread`** — rejected: no pool (thread churn),
  no batching/`wait`, and exactly the ad-hoc style we're replacing.

**Consequences / tech debt.**
- Threads now link into the offline tracer + tests (`-pthread` / CMake
  `Threads::Threads`). The std-lib-only rule holds — no new third-party dep.
- **Does not make `SlotMap`/`SparseSet`/`World` thread-safe**, and does **not**
  wire the pool into Jolt or into ECS system execution. Those are explicit
  follow-ups; `parallelFor` bodies must not structurally mutate the ECS.
- Single shared queue can become a contention point at high task rates — fine
  for current coarse workloads, revisited by profile.

**First external consumer.** Jolt's physics step now runs on this pool via a
`JPH::JobSystem` adapter (ADR-0012) — `QueueJob` forwards each ready job to
`run`. This validated the `run`/`wait` surface against a real, dependency-rich
external scheduler.

**Revisit trigger.** A profile showing queue contention or scheduling overhead
(→ per-worker work-stealing deques); or parallelizing ECS systems (→ define core
container thread-safety / a deferred command buffer, ADR-0006).

---

## ADR-0015 — Engine code lives in `namespace engine`
**Status:** Accepted (migration complete) · **Date:** 2026-06-05

**Context.** All of our types sat in the global namespace — `Vec3`, `Mat4`,
`Handle`, `World`, `Entity`, `Renderer`, `JobSystem`, `Material`, `Scene`, … As
third-party libraries arrived (Jolt, ImGui; asset/glTF loaders to come) this
started to bite: wiring our pool into Jolt (ADR-0012) hit a real collision —
inside a class deriving from `JPH::JobSystem`, the bare name `JobSystem` resolved
to Jolt's, forcing a `::JobSystem` workaround. Global names only get more
crowded as the surface grows. Engines namespace themselves for exactly this
reason (Jolt `JPH::`, Godot `godot::`, Bullet `bt`).

**Decision.** Put all our code in `namespace engine`. Macros stay global
(`LOG_*`, `CHECK`, `ASSERT`) — they ignore namespaces and are already
`UPPER_SNAKE`.

Migrate **layer by layer, bottom-up (core first)** rather than in one sweep, to
keep each step reviewable and to limit blast radius on the macOS-only backends
that can't be compiled in CI. Between stages, each migrated header re-exports its
public names at global scope with transitional `using engine::Name;`
declarations, so un-migrated consumers keep compiling **unchanged**. The aliases
are deleted in the final stage. (For the broad engine layer, enumerating ~50
types across ~20 headers would be error-prone, so the equivalent shim there is a
single `using namespace engine;` in each leaf consumer — test/`viewer_main`
`.cpp` files only, never a header — rather than per-type aliases.) Order: **core math + identity (rt_math, handle,
slot_map) → job_system + logging → engine (world/ECS/systems/cameras/physics) →
renderer → offline tracer → drop the shims.**

**Name.** `engine` (descriptive, unambiguous about intent). Considered `rt` (ties
to the `rt_` file prefix; shorter) and `rtx`/`rte`; `engine` was chosen for
clarity at call sites now that this is an engine, not just a tracer.

**Alternatives considered.**
- **One big-bang sweep** — rejected for now: a ~60-file mechanical diff that's
  hard to review and would touch the unverifiable Metal/GLFW files all at once.
- **A short prefix instead of a namespace** (`btScalar`-style) — rejected:
  namespaces compose with `using`, support ADL, and are the modern idiom.
- **Re-export via `using namespace engine;` in headers** — rejected: pollutes
  every includer and, being a using-*directive*, doesn't satisfy qualified
  (`::Name`) lookup, so it would break existing `::JobSystem`-style references.
  Explicit `using engine::Name;` declarations do, so those are used.

**Consequences / tech debt.**
- A window of **transitional global aliases** exists until the migration
  completes (tracked in the register). Each is a one-line `using` to delete.
- **Forward-declared types** can't be shimmed transparently: a global
  `using engine::JobSystem;` conflicts with a global `class JobSystem;` forward
  declaration. `JobSystem` is forward-declared in three physics files, so it
  migrates *with* the physics layer (its decls flip to `namespace engine { … }`),
  not in the first core step.
- **Operators need no alias** — found by ADL through their engine-typed operands.
- Inside a `JPH::`-derived class, our names still need qualifying
  (`engine::JobSystem`), since the base scope is searched first.

**Outcome.** Migration completed in five staged commits (core → job_system/
logging → engine → renderer → tracer + shim removal). All of `src/` now lives in
`namespace engine`; every transitional alias is gone. The only global-scope
namespace references left are a `using namespace engine;` in the leaf
consumers (`main.cpp`, `viewer_main.cpp`, and the test `.cpp`s) and `int main()`
itself. The macOS-only backends (`metal_renderer.mm`, `window.cpp`,
`gamepad_gc.mm`) were wrapped with the namespace placed outside any ObjC
construct and after all (incl. conditional) includes; **confirmed building and
running on macOS** (viewer), closing the usual can't-compile-in-CI gap for this
change.

**Revisit trigger.** If `engine` proves too generic against a future
embedded/3rd-party `engine` symbol, revisit the name.

---

## ADR-0016 — An environment-provider seam; HDR via vendored `stb_image`
**Status:** Accepted (provider seam + HDR path); procedural day/night **Implemented** + clouds **first pass** (macOS verification pending) · **Date:** 2026-06-09

**Context.** The scene's environment is hardcoded: `sampleEnvironment(dir)` in
`shaders/metal/phong.metal` is a fixed daytime-sky function (sun disc, horizon
blend, ground tint). Everything downstream samples *that one function* — the
skybox pass, the per-pixel ambient/diffuse term, and the reflection-probe bake
(ADR for probes lives in the post-processing work). There is no way to feed in a
captured environment, and no axis for time-of-day. We want three things, not one:
a **captured HDR** environment for realism and image-based lighting; a **richer
procedural sky** with a real sun direction / day–night cycle; and, later,
**clouds**. These should coexist and be swappable, not fork the renderer.

A decode question rode along: an HDR equirect map is Radiance RGBE, which needs a
decoder. The project's standing rule (AGENTS.md) is "no external dependencies."

**Decision — two parts.**

1. **Environment-provider seam.** Model the environment as *one question*: given
   a world-space direction, what radiance arrives? Both consumers that exist
   today (skybox fragment shader, probe bake) already funnel through
   `sampleEnvironment(dir)`, so the seam lives at that function plus a small
   uniform selecting the active provider. Planned providers:
   - **HDR** — sample a vendored equirect float texture (`dir → spherical UV`).
   - **Procedural** — the analytic sky, to be promoted from today's fixed tint to
     a sun-direction / turbidity model (day–night) driven by ImGui controls.
   - **Composited** — procedural atmosphere with a clouds layer on top.

   The provider is a *render-side* concept selected by a uniform/flag, not a new
   class hierarchy in the shader; the C++ side owns which provider is bound and
   its parameters. Because the probe bake renders the skybox into cubemap faces,
   IBL tracks whichever provider is active **for free** — bake the HDR (or the
   static procedural sky) and reflections/ambient follow. Animated clouds are a
   sky-dome *visual* layer and are **not** baked into probes (a stale snapshot at
   most); a captured HDR already contains its own clouds, so the clouds layer is
   procedural-only.

2. **HDR decode reuses vendored `stb_image`.** `third_party/tinygltf/stb_image.h`
   is already in the tree and already compiled into the build via the glTF
   importer (`src/engine/model_importer.cpp`). `stbi_loadf()` decodes `.hdr`
   (Radiance RGBE) to linear `float` RGB in one call — and PNG/JPG/TGA besides.
   Use it rather than hand-rolling an RGBE decoder.

**Dependency-rule clarification.** AGENTS.md's "no external dependencies" is
reinterpreted, not broken: it now means **no new third-party dependencies**.
Single-header libraries already vendored and built — `stb_image`/`stb_image_write`
(via tinygltf), tinygltf itself, Dear ImGui, Jolt — are accepted; using
`stb_image` for HDR adds no new dependency, no new submodule, no new build edge,
and matches how glTF textures are already decoded. The from-scratch RGBE decoder
(~60 lines, a stable format) was a defensible alternative but only adds
maintained surface area for strictly less coverage than the decoder we already
ship. AGENTS.md is updated to state the refined rule.

**Alternatives considered.**
- **HDR replaces the procedural sky** — rejected: loses day–night authoring and
  the clouds path; the provider seam keeps both as first-class.
- **Hand-written RGBE decoder** — rejected (see above): small but redundant
  against vendored `stb_image`, which also covers LDR formats we'll want anyway.
- **A new third-party HDR/IBL library** (e.g. a dedicated loader/baker) —
  rejected: heavier than the need; `stbi_loadf` + the existing probe bake suffice.
- **Provider as a shader subclass / function-pointer table** — rejected: a
  uniform-selected branch is simpler and the consumer count is small (skybox +
  bake); revisit if provider count or per-provider cost grows.

**Consequences / tech debt.**
- A float-texture upload path is required; today's `uploadTexture` is RGBA8-only.
  Added as an HDR-specific entry point (`RGBA16Float` equirect 2D) rather than
  overloading the 8-bit path.
- **Equirect → cubemap bake is implemented.** The equirect HDR is baked once at
  load (`setEnvironmentMap` → `Impl::bakeEnvironmentCubemap`) into a mipmapped
  `MTLTextureTypeCube`: six faces rendered with the probe bake's per-face cameras
  (the verified cube convention) by a small `fragmentEquirectBake` pass that emits
  raw radiance, blitted into the cube slices, then mip-generated. The skybox,
  composite, and probe-bake skybox now do a cheap `envCube.sample(dir)` instead of
  per-sample `atan2`/`acos`; the equirect 2D texture is kept only as the bake
  source. A 1×1 `defaultCubemap` keeps a valid binding in procedural mode.
  *macOS/Metal verification pending.* Follow-up: GGX prefilter of the cube mips
  (shared with the probe path, which still box-filters) for correct rough IBL.
- Procedural **day–night is implemented** (step 2): a pure, unit-tested engine
  helper `DayNightCycle` (`src/engine/day_night_cycle.*`) maps a normalized
  time-of-day to a sun arc and a graded sky/light palette; a `DayNightSystem`
  advances it each frame and drives **both** the procedural sky and the scene's
  directional sun, so shading and shadows track the sky. The sky parameters ride
  in `LightUniforms` (already bound everywhere `sampleEnvironment` is sampled),
  so no new shader buffer was threaded; `sampleEnvironment(dir, env)` now reads
  sun direction/colors from that uniform instead of hardcoded constants. Defaults
  in `ProceduralSky` reproduce the original fixed daytime sky. *macOS/Metal
  verification pending* (Linux can't compile the backend); the time-of-day curve
  is covered by `tests/test_day_night.cpp`. A full analytic atmosphere
  (Preetham/Hosek–Wilkie) remains a possible refinement.
- Procedural **clouds — first pass** (step 3): an FBM noise layer painted on the
  sky dome (`applyClouds`/`cloudFbm` in `phong.metal`), drifted over time and
  shaded against the active sun (sunlit tops, dark night silhouettes, warm at
  dusk). It is a **screen/SSR visual only and is never baked into reflection
  probes** — as the ADR requires: the probe bake reuses `fragmentSkybox` with a
  new `EnvUniforms.cloudsEnabled = 0` gate, while the main pass and the composite
  sky path overlay clouds. Cloud parameters (coverage/density/scale/time) ride in
  `LightUniforms`; `DayNightSystem` drifts the phase and exposes ImGui controls.
  Clouds attach to the procedural sky only (a captured HDR carries its own).
  *macOS/Metal verification pending.* Follow-up: richer cloud lighting
  (multi-layer, silver lining).
- The **composite sky path now honors the active provider.** Direct-view sky
  pixels (`depth >= 0.999`) are re-derived in `fragmentComposite`; previously
  this always ran the procedural sky, so a bound HDR map showed in the skybox/IBL
  but not where the sky was seen directly. The composite pass now takes the
  equirect map (texture 6) + sampler and an `envMode` flag in `CompositeParams`,
  and samples HDR vs. procedural+clouds to match `fragmentSkybox`.
- Volumetric (raymarched) clouds are explicitly **out of scope** here — a
  Tier-4/5-sized effort, not a slot-in to this seam.

**Revisit trigger.** Equirect sampling showing up hot in a GPU capture (→ bake to
cubemap); adding a second renderer backend (the provider uniform/shader split
must hold behind the RHI, ADR-0001); or provider count/per-provider state growing
enough that the uniform-selected branch wants to become real polymorphism.

---

## ADR-0017 — Unified physically based lighting with an artistic-control and shading-model seam
**Status:** Accepted; Phases 0–3 **implemented** (consolidation; BRDF unification; shadow artistic controls + camera-following shadow volume + HDR sun extraction — shadows visually verified; environment unification: cube-bake orientation proven by `tests/test_cube_faces.cpp`, GGX-prefiltered + irradiance cubes, one shader env path, composite equirect workaround removed). Phase 3 note: the procedural sky stays analytic behind the same provider interface — baking it per-frame as day/night animates is a deferred optimization. Phase 4 (gather/respond shading-model seam) is next. · **Date:** 2026-06-10

**Context.** The lit path is two mismatched halves. Indirect lighting (HDR IBL,
reflection probes) is proper GGX split-sum PBR (BRDF LUT, prefiltered mips), but
direct lighting in `evaluateLighting` is still Blinn-Phong: a `shininess =
mix(16, 512, 1-roughness)` remap with `(s+8)/8π` normalization, a diffuse term
missing the 1/π Lambert factor (≈3× too hot relative to IBL), and the old
LearnOpenGL `1/(1 + 0.09d + 0.032d²)` point/spot falloff instead of inverse
square. Roughness therefore means different things to the sun and the sky, and
direct vs. indirect light are not in the same energy units — which is why
balancing a sun against an HDR environment never converges.

That mismatch also explains the headline visual complaint: **shadows are faint
under an HDR environment**. The shadow factor multiplies only each light's
direct term; the ambient/IBL terms (`ambientDiffuse`, `envSpecular`) are never
shadowed, and under an HDR they carry most of the energy — a fully shadowed
pixel keeps ~80% of its brightness. There are no artistic controls to push back
with: no shadow strength, no shadow tint, just the single `ambientMultiplier`
scalar.

Accumulated cruft compounds it: `fragmentMain` and `fragmentMainInstanced` are
~150-line near-duplicates; the three-way environment branch (HDR / probes /
procedural) is duplicated in both and partially again in
`sampleReflectionProbes`, each copy with slightly different Fresnel/energy
treatment; everything lives in one 1,650-line `phong.metal`; uniform structs are
hand-mirrored between `metal_renderer.mm` and MSL with manual padding;
`ShadowConfig.bias`/`normalBias`/`pcfRadius` flow from level JSON to the GPU and
are never read (the real bias is hard-coded on the encoder); the shadow ortho
volume is a fixed 30-unit box anchored at the world origin; the skybox pass
pre-multiplies exposure that composite multiplies again (sky seen through SSR is
double-exposed); and composite re-derives sky pixels per-fragment to work around
a mis-oriented equirect→cube bake. Separately, the engine should eventually
support **NPR (toon) shading** and per-feature toggles without forking the
renderer.

**Decision — a phased refactor with two commitments.**

1. **One BRDF, one set of units.** Direct lighting moves to Cook-Torrance GGX
   (GGX NDF, height-correlated Smith visibility, Schlick Fresnel) with
   Lambert/π diffuse, sharing roughness semantics with the existing
   split-sum/prefilter path. Point/spot lights get inverse-square falloff with a
   windowed radius. After this, sun-vs-environment balance is a single exposure
   decision, and every artistic control sits on predictable ground.

2. **Shading is split into *gather* and *respond*, with artistic controls in the
   gather.** Gather produces, per surface point: each light's direction,
   radiance, and visibility-as-color (shadow), plus indirect irradiance and
   prefiltered specular. Respond is the BRDF that turns those into outgoing
   radiance. Artistic shadow controls live in gather as a small struct on
   `SceneLighting`:

   ```cpp
   struct ShadowArtistic {
       float strength = 1.0f;        // 0 = shadows off, 1 = full direct occlusion
       Vec3  tint{0, 0, 0};          // shadowed regions lerp toward this color
       float ambientStrength = 0.5f; // how much shadow also darkens IBL/ambient
   };
   ```

   `tint` gives "deep blue, not black" shadows; `ambientStrength` is what makes
   shadows *dark under an HDR* (it occludes the environment terms, which today
   ignore shadowing entirely). Because visibility is a color computed in gather,
   every response model — PBR or toon — inherits the same shadows, lights, and
   probes. NPR becomes a per-material `ShadingModel` (Standard, Toon, Unlit)
   selected via Metal **function constants** compiled as cached pipeline
   variants; feature toggles (shadows, IBL, clouds) use the same mechanism
   instead of runtime branches.

**Phases** (each shippable and verifiable on its own):

- **Phase 0 — Consolidate (no intended visual change).** Shared
  `shader_types.h` included by both C++ and MSL (simd types) replaces the
  hand-mirrored uniform structs; `phong.metal` splits into focused modules
  concatenated at load (runtime `newLibraryWithSource` has no include paths);
  the two fragment shaders merge into one `shadeSurface()`; the dead shadow
  uniforms get wired (bias → encoder depth bias, normalBias → world-space
  offset, pcfRadius → PCF kernel); exposure is applied exactly once (composite).
- **Phase 1 — Unify the BRDF** (commitment 1).
- **Phase 2 — Shadows: reach, response, artistic control.** `ShadowArtistic`
  (above) in shader + level JSON + ImGui; shadow ortho volume fit to the camera
  frustum ∩ scene bounds instead of the origin-anchored box; **HDR sun
  extraction** — `EnvironmentLoader` finds the dominant bright direction/color
  and drives the shadow-casting directional sun, so a skybox-lit scene gets
  crisp shadows that match its sun; SSAO moves to darken indirect light only.
- **Phase 3 — One environment path.** Both providers (HDR and procedural sky)
  bake into the same cubemap + prefiltered mips + irradiance; the three-way
  fragment branch collapses to irradiance(N) / prefiltered(R, rough) / BRDF LUT
  with probes blended on top; the mis-oriented equirect→cube bake is fixed and
  the composite per-pixel sky workaround removed; radiance clamps and divergent
  Fresnel paths disappear. Ambient tint / ground-bounce grading lands here.
- **Phase 4 — The shading-model seam** (commitment 2): gather/respond split,
  function-constant pipeline variants, Toon model, `SceneLighting` split into
  `Lights`/`Shadows`/`Environment`/`Grading` sub-structs with JSON + ImGui.
- **Phase 5 — Later polish:** CSM, spot/point shadow maps, PCSS, auto-exposure
  adaptation.

**Alternatives considered.**
- **Keep Blinn-Phong and only add shadow controls** — rejected: tint/strength
  knobs on top of mismatched units fight the exposure knob; every HDR swap
  re-breaks the balance. The BRDF unification is what makes the controls stick.
- **Full deferred shading** — rejected for now: the forward pass with a small
  G-buffer (color + view normals) already feeds SSR/SSAO; deferred is a much
  bigger migration and not required for any planned feature. Revisit with
  many-light scenes (Tier 4 procedural cities).
- **Uber-shader runtime branches for NPR/toggles** — rejected: the lit shader
  is already branch-heavy; function constants give dead-code elimination per
  variant and a natural pipeline-cache key.
- **Shader subclass/polymorphism for shading models** — rejected: same
  reasoning as ADR-0016's provider seam; variant count is small and enumerable.

**Consequences / tech debt.**
- Visual output will shift at Phase 1 (energy renormalization) — levels'
  `exposure`/`ambientMultiplier`/light intensities need a one-time retune.
  Acceptable: today's values encode the wrong units.
- Pipeline variants (Phase 4) multiply PSO count; mitigated by lazy creation +
  cache keyed on the function-constant set.
- The SSAO indirect-only fix wants ambient available at AO-apply time; the
  honest route is a Z-prepass (also useful later), the interim route keeps the
  composite apply with a tunable floor. Decided per-measurement in Phase 2.
- The concatenating shader loader is itself interim — if/when shaders are
  precompiled to a `.metallib`, real `#include` replaces it.
- **Root cause of the historic "mis-oriented cube bake" (found in Phase 3):**
  not face orientation at all. `vertexSkybox` normalized the reconstructed
  view ray *at the vertices*; far-plane offsets are affine in NDC (their
  interpolation is exact), but interpolating *normalized* rays warps interior
  pixels — up to ~25° with the oversized fullscreen triangle. Every consumer
  (env bake, probe bakes, main-pass skybox) was warped; the composite looked
  right only because it reconstructs per pixel. The CPU unit test passed
  because the math was right — the GPU link wasn't. Found via load-time GPU
  validation, which stays in as a canary: an orientation readback check, a
  direction-reconstruction probe, and a prefilter NaN/firefly scan, plus
  headless visual tools (`RT_FRAME_DUMP=<png>` one-frame capture,
  `RT_DUMP_ENV=<dir>` cube-face/equirect dumps). Lesson recorded: math-level
  unit tests validate intent; only GPU readback validates the chain.

**Revisit trigger.** A second renderer backend (the gather/respond split and
function-constant variants must re-prove themselves behind the RHI seam,
ADR-0001); many-light scenes making forward lighting the frame's hot spot (→
deferred or clustered); NPR needs outgrowing per-material models (e.g.
full-screen stylization passes).

---

## ADR-0018 — The property layer: components describe their editable fields once
**Status:** Accepted · **Date:** 2026-06-13

**Context.** The editor needs a Unity-style inspector, JSON load/save, the
in-viewport ImGui panels, and undo — all over the same component fields.
Hand-writing each per component is five places to edit per field and five
places to drift (the original inspector and `LevelWriter` already disagreed on
the material block).

**Decision.** Each component implements one
`describeProperties(T&, PropertyVisitor&)` that walks its editable fields once —
label + live reference + semantics (`FieldMeta`: range, log scale, unit, color,
choices, read-only, serialization id). Every consumer is a *visitor* over that
walk: the Qt inspector (build/sync/write passes), the in-viewport panels
(`ImGuiPropertyVisitor`), JSON read/write (`JsonReadVisitor`/`JsonWriteVisitor`,
where the `FieldMeta` id is the file-format key), and undo (component-state JSON
snapshots). A `ComponentRegistry` (one line per type) enumerates which
components an entity carries and walks each, so UI code never names a concrete
component; entries also carry add/remove thunks and post-edit hooks. Adding a
component = struct + describe + one registration line; it appears in every
inspector and serializes automatically.

**Alternatives considered.**
- Hand-written inspectors/serializers per component — rejected: N×5 drift
  surface, already observed.
- A reflection library / codegen — rejected: heavyweight external dep; the
  visitor pattern gives the same single-source guarantee in plain C++17.
- Qt's property system (`Q_PROPERTY`/moc) — rejected: pulls moc into engine
  code and couples field descriptions to Qt; the layer is engine-side and
  UI-toolkit-free by construction.

**Consequences / tech debt.**
- `FieldMeta` ids ARE the level/camera JSON format now; renaming one is a format
  change. Legacy spellings (e.g. the checkerboard `flags` array) are still read
  on load for compatibility.
- Read-only fields are display-only and the JSON read visitor skips them by
  contract (size was read-only until a registry post-edit hook — Size → mesh
  rebuild — made it editable everywhere).
- Orientation is deliberately *not* a described field (the gizmo owns rotation),
  so rotation undo rides a dedicated `Transform` command, not the visitors.

**Revisit trigger.** A described field whose type the `PropertyVisitor`
interface can't express (nested structs, arrays, asset references) — extend the
visitor rather than fork the inspector.

---

## ADR-0019 — The editor is the engine hosted in a native shell, behind one bridge
**Status:** Accepted · **Date:** 2026-06-13

**Context.** We want a real level editor whose viewport behaves 1:1 with the
game, without a second renderer or an emulated runtime. (Design detail lives in
`docs/edit-mode-plan.md` and `docs/editor-app-plan.md`; this records the
decision and its trade-offs.)

**Decision.** The editor *is* the same engine, hosted. The level JSON is the
document: an `EditorState` loads it, edits mutate the live `World`, and Play
"compiles" (saves) then swaps to the game state. The Qt shell hosts the engine
through the `HostedWindow` seam — ADR-0001 inverted: the host owns the OS
window and the engine renders into a handed-in view, with input injected. A
single `EditorBridge` is the only conduit between shell and engine (selection,
document actions, creation requests, undo, component add/remove, gizmo mode,
plus an `EditorNotice` queue the shell drains for instant refresh). The bridge
attaches in two modes — *editable* (an `EditorState` is active) and *observer*
(read-only during Play: live hierarchy/inspector, no writes). Undo is a command
log (`UndoStack`) fed from the bridge's chokepoints. In-viewport ImGui tooling
that pre-dates the shell stays as "quick tools," suppressed when the native
panels own the duty.

**Alternatives considered.**
- A separate editor renderer / scene representation — rejected: two paths that
  drift; the viewport would not match the game.
- Direct shell→`World` access (no bridge) — rejected: every panel would couple
  to ECS internals and to whether gameplay or the editor owns the world;
  observer mode would be scattered checks instead of one flag.
- An immediate-mode (ImGui) editor as the authoring surface — rejected: docking,
  tree views, native file dialogs are years of toolkit work Qt gives free; ImGui
  stays for in-viewport overlays only.

**Consequences / tech debt.**
- The shell is Qt; only the Metal *rendering inside the viewport* needs macOS —
  the bridge, document model, and panels are Linux-CI-verified (headless Qt
  interaction test).
- First-person Play pointer capture is poll-based in the hosted viewport
  (Qt `grabMouse`/move events are unreliable for warp-style lock on macOS).
- Some cross-mode camera state still rides settings.json rather than the
  document.

**Revisit trigger.** A second shell (web/another toolkit) — the bridge is the
contract to re-implement against; or the in-viewport ImGui tools fully
superseded by native panels (then delete them).

---

## ADR-0020 — Transform hierarchy is a document concept, flattened for the runtime
**Status:** Accepted · **Date:** 2026-06-13

**Context.** Authors want to group objects under a null/parent and move them
together. The ECS (ADR-0006) deliberately stores flat world transforms — no
parent links — and physics/render assume world space. We needed grouping
without giving the runtime a scene graph to walk every frame.

**Decision.** Parenting lives in the *document*, keyed by **stable document
ids** (`SourceSpec.id`/`parentId`, serialized; minted on create, assigned to
id-less entities on load/save) rather than the runtime `Handle` (reminted each
load). In the editor, `worldMatrix(world, e)` composes a child's local
`Transform` up the parent chain (cycle-guarded); render, picking, the gizmo, and
framing use it, and the gizmo writes a manipulated world matrix back through the
parent's inverse. Reparenting preserves world position (the child's local
transform is rewritten). **Entering Play flattens the hierarchy:** each parented
entity's composed world transform is baked into its `Transform` and the parent
link cleared, so the runtime (physics body creation, render interpolation) never
walks a hierarchy and stays in world space — exactly as before. Groups are null
objects (`SourceSpec` with empty shape, no `Renderable`).

**Alternatives considered.**
- A runtime scene graph (a parent component the simulation walks) — rejected for
  now: physics bodies and render interpolation would need hierarchical world
  resolution every frame; a large change for an authoring convenience.
- Parent by runtime `Handle` — rejected: handles are reminted per load, so
  parenting wouldn't survive save/load; stable ids are the prerequisite (and
  also unlock future per-component apply-back and undo-across-reload).
- Baking groups away at *save* time — rejected: the hierarchy wouldn't round-trip
  in the editor; it must persist in the document and flatten only at the play
  boundary.

**Consequences / tech debt.**
- `Transform` now has two meanings: *local* in the editor (composed via
  parentId), *world* at runtime (flattened). The flatten point (`level_loader`,
  non-editor mode) is the single bridge between them.
- A child with physics under a *moving* parent only composes statically in the
  editor; animated parents driving physics children would need the runtime graph
  above.
- glTF multi-mesh and camera parenting are not yet wired (cameras carry no
  `SourceSpec`); the id/parent machinery is ready for them.

**Revisit trigger.** Runtime-animated hierarchies (skeletal motion, moving
platforms carrying dynamic children) or glTF node hierarchies needing per-node
transforms — promote parenting into a runtime component the simulation resolves.

---

## ADR-0021 — Procgen is a generator library over shared value types, not a language first
**Status:** Accepted · **Date:** 2026-06-13

**Context.** The long-term vision (ROADMAP) is a procedural-generation engine:
plants, terrain, cities, planets, clouds, textures — and the open question of a
single "procgen language" spanning meshes, buildings, roads, particles,
shaders, and bullet patterns. The temptation is to design that unifying language
up front. We need to fix the *shape* of the procgen effort before committing
Tier 3/4 work to it.

**Decision.** Every generator is `(parameters, seed) -> content`, and `content`
is one of a small set of **value types**: `Mesh`, `Field` (Rⁿ→scalar — SDF /
noise), `Material`/`Texture`, and `Frame` (transform sets / instancing). Those
value types *are* Tier 3 — the asset manager (3.1) owns them, the mesh builder
(3.3) is the `Mesh` type, the material system (3.2) is the `Material` type — so
the content pipeline is the **procgen substrate**, not a precursor to it.

From that, four commitments:
1. **Build bottom-up as a C++ library of composable generators** over the value
   types, with a deterministic evaluation context (seeded RNG streams +
   parameter binding; fits the determinism stance of ADR-0002). Prove it across
   three paradigms on the *same* types — a grammar (L-system), a field (noise
   heightfield), an implicit shape (SDF CSG).
2. **The "procgen language" (node graph and/or text DSL, Blender-geometry-nodes
   style) is a presentation layer distilled from the working library, AFTER
   three generators exist** — never designed first.
3. **Geometric modeling (Plasticity-style) is pursued via SDF / implicit
   modeling**, not a B-rep kernel: booleans/blends are min / max / smooth-min,
   meshed by marching cubes → dual contouring.
4. **Temporal generators (particles, bullet patterns) share the seeded-RNG +
   parameter substrate but live on a separate simulation track**, not the
   static-geometry pipeline.

**Alternatives considered.**
- *Language/graph first* — rejected: designing the unifying DSL with zero
  working generators means guessing the abstraction (a Turing-tarpit /
  lowest-common-denominator trap). Houdini (VEX/SOPs) and Blender (geometry
  nodes) grew the graph *over* an operation library, not the reverse.
- *One representation for everything* — rejected: grammars, dataflow, fields,
  and B-rep are genuinely different paradigms; the only real shared layer is the
  evaluation substrate + value types, not the pipeline.
- *B-rep / Parasolid for modeling* (as Plasticity uses) — rejected: a licensed
  commercial kernel, multi-year to roll in-house, and against both the
  no-new-dependencies rule (AGENTS.md) and the engine's procgen-robustness
  priorities. SDF is the achievable, composable analog.

**Consequences / tech debt.**
- Tier 3 (asset manager / material / mesh builder) is reframed as the procgen
  *value-type layer* and prioritized as foundation; a noise library joins it.
- No procgen syntax is committed now; the language is deferred until three
  generators expose what it must express.
- SDF modeling forgoes exact-CAD precision (acceptable for a generation engine,
  not a CAD tool).
- All generators must be seed-deterministic, to match ADR-0002 and keep
  generated worlds reproducible.

**Revisit trigger.** After three generators exist on the substrate, revisit
whether a node graph and/or DSL is worth building and what it must express —
driven by what the generators actually share. Revisit B-rep only if exact-CAD
*authoring* (not generation) becomes a goal.

---

## ADR-0022 — Procedural objects: generators as data, and the realness spectrum
**Status:** Accepted · **Date:** 2026-06-13

**Context.** The first generators (terrain, L-system trees, rocks, scatter) are
hand-written C++ (`rock.cpp`, `lsystem.cpp`, ...). The intent (ADR-0021) is that
these become *data* — node graphs / scripts an author composes from the mesh +
noise + SDF tools — so "a rock is a generator graph, not a rock.cpp file", and
the same generator can fill a level at runtime or be baked to a static asset.
Two things need pinning down: how generators are structured so they can evolve
into data, and how "real" a procedural object is (today they don't appear in the
editor and can't be selected/edited).

**Decision.**
1. **A generator is a uniform `(params, seed) -> Mesh (+ Material)` producer**
   over the tool library (mesh builder, noise, and SDF when it lands). The C++
   generators are the *bootstrap*: they define the vocabulary of operations a
   later node-graph evaluator (ADR-0021 Phase C) will expose as data. The same
   substrate evaluates a hand-written C++ generator and a graph asset, so code
   today migrates to data later without changing consumers.
2. **Generators live under `src/engine/procgen/`** (terrain, lsystem, rock,
   scatter, noise as the field primitive), separate from general tools the rest
   of the engine uses (mesh builder stays in `engine/`).
3. **Realness spectrum** — a procedural thing takes one of three forms, chosen
   per use, all driven by the *same* generator:
   - **Runtime-procedural**: regenerated from a recipe every load; not a
     document entity; not editable; cheap space-filling (today's terrain +
     scattered vegetation).
   - **Baked static asset**: the generator runs offline → a mesh + material on
     disk → placed/edited/collided like any authored asset; fixed geometry,
     reusable.
   - **Editable procedural instance** (future): a generator instance in the
     document with tunable params, pickable and re-runnable in the editor —
     needs the node graph + editor authoring UI.
4. **A coherent level is a mix**: authored entities + runtime-procedural recipes
   (fill/scatter) + baked assets. The recipe-vs-bake choice is per-object,
   driven by whether it must be edited, collided precisely, or reused exactly.

**Alternatives considered.**
- *Keep generators as bespoke C++ indefinitely* — rejected: every new content
  type would be a code change; the author can never compose new things without a
  programmer. The node-graph-as-data path is the whole point (ADR-0021).
- *Make procedural objects full editable document entities now* — rejected:
  premature; needs the node graph and stable per-instance identity first. They
  stay runtime-regenerated until then (tech-debt item below).

**Consequences / tech debt.**
- Procedural objects (terrain, vegetation) are intentionally **not shown or
  editable in the editor** — they carry no `SourceSpec`, so they're regenerated
  runtime objects, not document entities. Tracked in the register; the "which
  realness tier" decision per content type is deferred.
- No baking pipeline or node graph yet; the C++ generators are the stand-in.

**Revisit trigger.** When the node-graph evaluator (ADR-0021 Phase C) lands,
generators become authorable data — revisit editor authoring of generator
instances and the bake-to-static-asset pipeline then.

---

## ADR-0023 — Lua as the embedded scripting language: one VM, procgen-first, separate binding surfaces
**Status:** Accepted — Step 1 **implemented** (Lua 5.4.7 vendored + built; sandboxed `ScriptVM` seal; the procgen binding surface — `sdf.*`/`noise.*`/`polygonize` → Field/Mesh — headless-tested in `tests/test_script_vm.cpp`, incl. a script-vs-C++-substrate equivalence check). Gameplay surface, hot-reload, and graph↔script interop are follow-ups. · **Date:** 2026-06-14

**Context.** The engine needs a scripting layer for two purposes that have so far
been served by C++ only. (1) **Procgen authoring.** ADR-0021 fixed procgen as a
C++ library of composable generators with the *language* — "node graph **and/or
text DSL**, distilled from the working library" — deferred until generators
exist. They now do (terrain, L-systems, SDF rocks, scatter), and the node-graph
evaluator (ADR-0021 Phase C, `docs/node-graph-plan.md`) has shipped Phases 1–3.
The node graph is the **visual** presentation layer; the "text DSL sugar on top"
that ADR-0021/ROADMAP Phase C foreshadowed is still unbuilt. (2) **Gameplay.**
There is no way to express behaviour (triggers, spawns, level logic, tuning)
without recompiling; the engine is a `System`/ECS C++ binary end to end. Both
wants point at the same missing piece — an embedded interpreter — so the question
is which language, and how it relates to the node graph and the existing seams.

A hard constraint rides along: AGENTS.md forbids new third-party dependencies
without an ADR (ADR-0016 refined the rule to *"no **new** dependencies"*; vendored
libs already built — Jolt, ImGui, tinygltf, stb — are accepted). A scripting VM
is unambiguously a new dependency. This ADR is that authorization.

**Decision.**

1. **Embed Lua.** Adopt Lua (5.4.x) as the engine's scripting language. It is a
   tiny, self-contained ANSI-C interpreter built for embedding, and is the
   gamedev de-facto standard for exactly this role.

2. **One VM, separate binding surfaces.** A single interpreter implementation
   backs both uses; what differs is *which API is exposed*. Two binding surfaces:
   - **Procgen surface (pure/deterministic).** Binds the generator substrate
     (mesh builder, noise, SDF ops, L-system, scatter, polygonize) and nothing
     with ambient state. A procgen script is `(params, seed) -> content`, the
     same contract as a C++ generator (ADR-0021/0022) and a node graph. No
     wall-clock, no ambient RNG, no `io`/`os` — randomness only through seeded
     streams. This makes procgen scripts reproducible (ADR-0002) and safe to run
     on worker threads.
   - **Gameplay surface (effectful).** Binds ECS/world access, input, events,
     spawning, etc. — and **may call the procgen surface** (gameplay is the
     effectful layer that orchestrates the pure one), never the reverse.

3. **Procgen-first.** Build the procgen binding surface first. It is the
   smaller, purer, already-specified surface (the node graph names the exact
   vocabulary), it is fully **headless/Linux-testable** (like Jolt — pure C/C++,
   no GPU/OS), and doing Lua once procgen-first avoids throwaway gameplay
   plumbing. The Lua text script and the node graph are **two front-ends over the
   same C++ substrate** (ADR-0021/0022): the graph for visual authoring, Lua for
   text authoring and anything imperative the graph is awkward for. Neither
   replaces the other; both lower to the same generator functions.

4. **Seal Lua behind a `ScriptVM`** — a pimpl whose header carries no `lua_*`
   type, exactly as `Window` seals GLFW (ADR-0001) and `PhysicsWorld` seals Jolt
   (ADR-0012). Engine/game code speaks our types and an opaque VM handle; the
   `lua_State` and C API live only in the `.cpp`. One `lua_State` **per thread**
   (Lua has no global lock / GIL — see below), so the procgen surface is
   naturally parallel on the shared pool (ADR-0014): a `parallelFor` over species
   can each drive their own VM with no contention.

5. **Vendor as a pinned git submodule** under `third_party/lua`, built as a small
   static library by CMake — matching how Jolt and ImGui are vendored (clean
   tree, explicit version). Cross-platform pure C, so it builds and is tested in
   CI/Linux; the std-lib-only **offline tracer (`make`) is untouched** —
   scripting is an engine/CMake concern, like physics.

6. **Hand-written bindings over the raw C API first; defer a binding library.**
   The procgen surface is a few dozen functions; bind them by hand behind
   `ScriptVM` rather than pulling in a C++ binding lib (sol2/LuaBridge) — which
   would be a *second* new dependency. Revisit sol2 only if binding boilerplate
   measurably dominates (the ADR-0008 "measure first" ethos).

**Alternatives considered.**
- **Python (CPython / pybind11)** — rejected for the *runtime*. The GIL
  serializes script execution across threads, which is backwards for procgen
  fan-out on the JobSystem (ADR-0014); the runtime + stdlib is multi-MB to
  deploy vs. Lua's kilobytes; embedding ergonomics and per-call cost are heavier;
  and there's a minor determinism caveat (hash-seed/iteration order). Python's
  real strength is familiarity/ecosystem, which belongs to **offline tooling**
  (asset cooking, build/editor automation), not the engine runtime — left open as
  a separate, non-embedded option.
- **A hand-rolled DSL / interpreter** — rejected: re-implements lexer/parser/VM/
  GC/error handling that Lua already provides, tuned and battle-tested, for less
  capability and more maintenance. The node graph already covers the *visual*
  DSL; a bespoke *text* VM is the wheel Lua is.
- **Node graph only, no text language** — rejected: ADR-0021 explicitly named
  "graph **and/or** DSL"; graphs are clumsy for imperative logic (loops,
  conditionals, gameplay rules), and a text surface is the right tool there. They
  coexist over one substrate.
- **A parser-library-based DSL** (ANTLR, etc.) — rejected: still a new dependency,
  and yields only a parser — we'd still hand-build the runtime Lua already is.

**Consequences / tech debt.**
- Adds the first **new** third-party dependency since the ADR-0016 rule refinement
  (engine/CMake scope; the offline tracer stays std-lib-only). Requires
  `git submodule update --init`; CLAUDE.md/AGENTS.md updated to list it.
- **Two procgen front-ends now coexist** (node graph + Lua) over one C++
  substrate. Graph↔script interop (a "Script" node; or a graph callable from
  Lua) is a deliberate **open question**, not built here — kept open by the fact
  both lower to the same generator functions.
- The procgen surface must be a **deterministic sandbox** (no `io`/`os`/ambient
  time, seeded RNG only); enforced by *what we bind*, not by trusting scripts.
- Per-thread `lua_State`s mean procgen scripts must not share mutable VM state
  across threads — fits the "independent work only" contract of `parallelFor`
  (ADR-0014).
- No gameplay surface, hot-reload, or debugger yet — explicit follow-ups; the
  gameplay bindings touch the ECS and are the larger, later surface.

**Revisit trigger.** Binding boilerplate dominating (→ adopt sol2, a second
dependency, with its own ADR); per-frame gameplay script cost showing in a
profile (→ JIT via LuaJIT, or move hot paths to C++); needing offline
tooling/automation (→ reconsider Python *there*, non-embedded); or graph↔script
interop becoming a real authoring need (→ design the bridge then).

---

## ADR-0024 — Gameplay scripting: a MonoBehaviour-style `ScriptBehaviour` over the ECS
**Status:** Accepted — **implemented** (effectful gameplay surface — entity/input/camera/spawn; `ScriptBehaviour` component + `ScriptSystem` with a headless `tick` + a deferred spawn command buffer; per-entity instance tables; start/update lifecycle). Proven by porting the C++ `ShootingSystem` to `assets/scripts/gun.lua` — a physics-block gun attached to the player, covered headlessly by `tests/test_script_system.cpp` and `tests/test_gun_script.cpp` (runs the real asset). Wired into `ArenaState` in place of `ShootingSystem` (macOS/viewer-gated, so CI-unverified per the standing backend constraint). Destroy-from-script, hot-reload, and ref cleanup remain follow-ups. · **Date:** 2026-06-14

**Context.** ADR-0023 chose Lua, built the *procgen* (pure/sandboxed) surface
first, and deferred the *gameplay* (effectful) surface. The open question it left:
how does a script become behaviour on an entity? The reference the user named is
Unity's **MonoBehaviour** — a script with `Start()`/`Update()` attached to a
GameObject. Our engine already has that exact lifecycle at the C++ level: a
`System` has `onStart`/`update`/`fixedUpdate`/`onEvent`/`render`/`onStop`
(ADR-0004). So the question is really: expose that lifecycle to Lua, per entity,
without coupling the core ECS to the scripting subsystem or breaking determinism.

**Decision.**
1. **A `ScriptBehaviour` component = a Lua script on an entity.** Its `source` is
   a chunk that **`return`s a table** of hooks; that returned table *is* the
   per-entity instance, so its fields are the behaviour's state — exactly
   MonoBehaviour members. Hooks mirror the engine lifecycle: `start(self, e)`
   once, `update(self, e, dt)` each frame (room to add `on_event`, `fixed_update`
   later). The entity arrives as an opaque packed-Handle integer.
2. **A `ScriptSystem` drives them.** It owns **one gameplay `ScriptVM`** (effectful
   bindings, *not* the procgen sandbox), lazily loads each behaviour into its own
   registry-held instance table, runs `start` once, then `update(e, dt)` every
   frame. Its core is a headless `tick(World&, double)` (the `PhysicsSystem`
   pattern), so the whole layer is unit-tested without a window/renderer.
3. **Effectful surface, the other half of the ADR-0023 split.** `openGameplayLibrary`
   binds `log`, `entity.*` (position/orientation/`look_along`/`snap_prev`),
   `input.*` (bound actions), `camera.*` (the active view = aim source), and
   `spawn.block`/`spawn.model` (deferred). Unlike procgen, these read/**mutate the
   World**; the per-tick `GameplayContext` (world/input/camera/spawn-buffer) is
   bound for a tick and cleared after, so nothing stale is reachable between
   ticks. The ScriptSystem VM **also opens the procgen builders** (they are pure),
   so a gameplay behaviour can *generate* geometry — e.g. `gun.lua` kit-bashes its
   own viewmodel and spawns it via `spawn.model`.
4. **Runs in variable-rate `update()`, never `fixedUpdate()`.** Gameplay scripts
   are deliberately outside the deterministic stance — physics stays the
   fixed-step authority (ADR-0002/0012). Frame logic is where MonoBehaviour-style
   scripts belong.
5. **`ScriptBehaviour` lives in the scripting module, not core `components.h`.**
   The ECS stays scripting-agnostic and the entire layer gates on
   `RT_ENABLE_SCRIPTING`. The component carries only plain ints for its runtime
   refs — no `lua_*` type leaks into a header (the ADR-0023 seal rule).

**Alternatives considered.**
- **A Lua *System* (one script ticking a query) instead of a per-entity
  component** — rejected as the *primary* model: per-entity behaviour is the
  Unity-familiar, ECS-natural unit and what was asked for. A system-level script
  surface can be added later as a different binding without disturbing this one.
- **One `lua_State` per behaviour (or coroutine per entity)** — rejected: one VM
  per thread is the Lua idiom; per-entity *instance tables* already give state
  isolation (a test pins it) without N interpreters.
- **Full ECS reflection from Lua (read/write any component)** — deferred: start
  with Transform + lifecycle; grow the surface (via the property layer, ADR-0018)
  as real behaviours demand it, rather than exposing everything speculatively.
- **Put `ScriptBehaviour` in core `components.h`** — rejected: couples the ECS to
  the optional scripting dep; keeping it in the module preserves the gate.

**Consequences / tech debt.**
- **No spawn/destroy from scripts yet.** Scripts only mutate `Transform`, so the
  `World::each` no-structural-mutation contract (ADR-0006) holds. Entity
  creation/destruction from Lua needs a deferred command buffer first.
- **Instance-ref cleanup is missing.** A destroyed entity's `ScriptBehaviour`
  registry ref is not `luaL_unref`'d, so refs accumulate until the VM closes — a
  bounded leak for now (tracked below); fix when behaviours churn.
- **Not hot-reloadable**, and **not yet registered in a running state** — the slice
  is exercised through `tick()` directly. Wiring `ScriptSystem` into PlayingState
  (and an editor "attach script" affordance) is the next step.
- **Single-threaded** gameplay VM (main thread) — fine; procgen keeps the
  per-thread-VM parallelism story (ADR-0023), gameplay does not need it.

**Revisit trigger.** Scripts needing to spawn/despawn entities (→ deferred command
buffer, ADR-0006); behaviour churn making the ref leak matter (→ unref on
component removal / entity destroy); wanting hot-reload; behaviours needing
fixed-step determinism (→ a `fixed_update` hook on the fixed schedule); or script
cost in a profile (→ LuaJIT, or move hot logic to C++).

---

## ADR-0025 — Remove the node graph: Lua is the one procgen authoring path

**Status:** Accepted. Supersedes ADR-0021 Phase C and the node-graph half of
ADR-0023's "two front-ends" framing.

**Context.** ADR-0021 ended with a "procgen language (node graph and/or text
DSL)" to be distilled from the C++ generator library, and `node-graph-plan.md`
shipped Phases 1–3: a headless graph engine (`node_graph.{h,cpp}`), JSON
generators (`*.graph.json`), and a list-style ImGui editor (`NodeEditorSystem`).
ADR-0023 then chose Lua and explicitly kept the graph as the *visual* front-end
alongside Lua's *text* front-end — "neither replaces the other."

In practice that balance never materialized. Lua became the authoring path we
actually use (flora library, gun viewmodel, whole-generator binding surface,
`test_script_vm`), while the graph stayed a stub: one consumer
(`rock.graph.json`), a list editor rather than a real canvas, and **no
graph↔Lua bridge** (the interop ADR-0023 left open was never built). Carrying two
parallel front-ends over the same substrate is maintenance with no payoff.

**Decision.** Remove the node graph entirely. Lua (ADR-0023/0024) is the single
procgen + gameplay authoring surface over the C++ generator library
(ADR-0021/0022). Deleted: `procgen/node_graph.{h,cpp}`,
`systems/node_editor_system.{h,cpp}`, `tests/test_node_graph.cpp`,
`assets/generators/rock.graph.json`, `docs/node-graph-plan.md`, and the
`"graph"` species branch in `level_loader.cpp` (the level loader now resolves a
species mesh from a built-in generator or a Lua `script`). The forest's rock
species falls back to the built-in SDF/displaced rock generator.

**Consequences.**
- One authoring story, one set of bindings to maintain; the C++ generators and
  the Lua surface are unaffected.
- Lose visual/dataflow authoring. If a node canvas is ever wanted, it should be
  re-derived as a thin front-end that *emits Lua* (or calls the same generator
  functions), not a second evaluator — so it can't drift from the text path.
- ADR-0021/0023 stay as historical record; this ADR is the live word on Phase C.

**Revisit trigger.** A concrete need for visual authoring by non-programmers, or
a generator graph too awkward to express as a Lua script.

---

## ADR-0026 — Procedural objects are multi-output assets: mesh + rig + collision, Lua recipe, data instance
**Status:** Pending · **Date:** 2026-06-15

**Context.** ADR-0021/0022 set the procgen substrate (generators are
`(params, seed) -> content` over shared value types) and the realness spectrum
(runtime-procedural / baked static asset / editable procedural instance);
ADR-0025 made Lua the one authoring path. The first parametric tree
(`src/engine/procgen/tree.{h,cpp}`, `growTree`) exposed two gaps the earlier
ADRs left implicit:

1. A real game object is **not one mesh**. A tree is a small prefab — a
   *skinned* render mesh (bark + leaves), a **skeleton/rig**, a **collision**
   representation, and **material/texture** bindings — that a maintainer expects
   to sway in wind and animate. `growTree` already builds the branch node tree
   (which *is* a bone skeleton) and currently discards it.
2. The tree was wired in as a `shape:"tree"` JSON block that **inlines a recipe
   into the level**, bypassing Lua. That is a second authoring path, against
   ADR-0025.

We need to pin how a procedural object is structured, authored, instanced, and
edited before building more of it.

**Decision.**
1. **A generator emits a multi-output asset, not a bare `Mesh`.** A tree
   generator returns a `TreeAsset`: skinned bark + leaf meshes, a **skeleton**
   (bones = branch segments, derived from the L-system node tree), per-vertex
   **skin weights** binding rings to bones, a **collision** representation, and
   material refs. The skeleton is reused as the wind/animation rig — the rig is
   a near-free byproduct of generation, not a separate authoring step.
2. **Collision is its own representation, separate from the render mesh and from
   sway.** Gameplay collision (bump/shoot) is a cheap static shape — a capsule
   chain over the major limbs is preferred to a full triangle soup. Wind sway is
   a *separate* concern layered on top, chosen per project from a cost ladder:
   (a) vertex-shader wind (no rig/physics; the shipping default), (b) rig +
   procedural wind animation, (c) rig + Jolt joints (a body+spring per bone — the
   literal "physics object", expensive, rarely per-branch). The static collider
   is kept regardless of which sway approach is used.
3. **Four authoring layers, named and not collapsed:**
   - **Operations** (C++): `growTree`, `lsystem`, `sdf`, mesh ops — the systems.
   - **Recipe / generator asset** (Lua file under `assets/`): a composition plus
     a **declared parameter schema** (names, types, ranges, defaults) = "a
     species". Authored outside the engine.
   - **Instance** (level/editor data): an asset ref + transform + seed +
     param overrides. Pure data; not a grammar.
   - **Baked asset** (optional): a frozen generator output (mesh/rig) on disk.
4. **Lua authors recipes; data places instances** (refining ADR-0025's "Lua is
   the authoring path"): the *recipe* is the Lua asset; the *instance* is data
   that references it. The `shape:"tree"` JSON shortcut is removed in favor of an
   entity that references a recipe asset.
5. **The editor is the iteration surface.** A procedural instance becomes a real
   document entity via a `ProcgenSource` component (recipe ref + seed + params);
   the recipe's declared param schema drives the inspector, re-running the
   generator on change. "Build by script" (author the Lua recipe) and "build in
   the editor" (edit params/seed) are the same path with two front-ends. This
   realizes ADR-0022's deferred *editable procedural instance* tier.

**Alternatives considered.**
- *Keep generators emitting a single `Mesh`* — rejected: forces rig/collision/
  material to be re-derived or hand-authored per object; throws away the
  skeleton the L-system already builds.
- *Bind `growTree`'s current output to Lua now, defer the asset shape* — rejected
  as premature: the binding would be reworked the moment rig + skin weights +
  capsule collision + a param schema land; design the asset first.
- *Per-branch Jolt joints as the default sway* — rejected as the default:
  expensive and rarely how trees sway in shipping games; kept as an opt-in rung
  on the cost ladder, not the baseline.
- *Editor-only authoring (no text recipes)* — rejected: contradicts ADR-0025 and
  blocks headless/scripted generation; the editor edits the same Lua/params.

**Consequences / tech debt.**
- `growTree` must be reshaped from `TreeMesh` to a `TreeAsset` (skeleton + skin
  weights + collision + material); current consumers (the `shape:"tree"` entity)
  change with it, and that JSON shortcut is removed.
- Needs new engine pieces: a skinned-mesh + `Skeleton`/animator path, a
  `ProcgenSource` component, a recipe-asset loader with a param schema, and Lua
  bindings over the multi-output generator. Each is its own step.
- Wind/skinning are not built; only the static collider exists today. The
  shipped C++ generator, pipe-model taper, textures, and gameplay collider are
  retained — this ADR adds layers around them.
- A param-schema format must be chosen (likely declared in the Lua recipe), used
  by both the editor inspector and the baker.

**Revisit trigger.** When skinned meshes + a skeleton/animator exist, validate
the `TreeAsset` shape against a second procedural asset (e.g. a building or a
creature) before generalizing it; revisit the wind cost-ladder choice when a
target platform's perf budget is known.

---

## ADR-0027 — The world is fields + recipes, streamed in deterministic tiles, authored as data
**Status:** Pending · **Date:** 2026-06-15

**Context.** The world (terrain, biomes, scatter vegetation, water, later
cities) is bigger than the current pieces — a single heightfield
(`procgen/terrain.cpp`) with noise/slope scatter (`procgen/scatter.cpp`) baked
into a level. We need a representation that an artist authors *and* that
generates an open, streamed world, on the procgen substrate (ADR-0021) with Lua
authoring (ADR-0025). Detail in `docs/world-system-plan.md`.

**Decision.**
1. **A world is a stack of *fields* + a set of *recipes* that read them.** Fields
   are 2D rasters aligned to terrain (`height`, derived `slope`/`aspect`,
   `moisture`, `temperature`, `biome id`, and N scatter-density masks). Recipes
   (Lua) are `(region, fields, seed) -> content`: a forest = a scatter recipe
   over a region weighted by masks; a river = a water recipe (+ carve); a city =
   a building recipe that masks out nature.
2. **Authoring is data, not code.** The editor edits regions, painted mask
   layers (brushes -> raster assets), carve volumes, recipe params, and seeds —
   all stowed as assets; Lua recipes are themselves assets; the world file
   references them. "Build by script" and "build in editor" are one path, two
   front-ends (extends ADR-0025).
3. **Scatter/biome masks are procedurally seeded, then brush-paintable** raster
   layers (not vertex colors), sampled by scatter *and* terrain-material splat.
4. **A scatter region / biome is one entity** (footprint + recipe + masks),
   expanding to instance groups at runtime; individual scattered props stay
   render data, not entities (extends ADR-0022). Terrain, water bodies, and
   carve edits are entities; fields/masks are paintable overlays.
5. **Open-world via deterministic per-tile generation + sparse per-tile override
   assets.** Each tile regenerates identically from `(tileCoord, worldSeed)`
   (ADR-0002); human edits layer on top per tile. Streamed by distance with LOD.
   Design for streaming now; implement against a fixed tile set first.
6. **Terrain is a heightfield now; carving comes later via SDF edits.** A
   heightfield cannot do caves/overhangs. When designed carving is needed,
   subtract SDF volumes from terrain and re-mesh only affected chunks with the
   existing `polygonizeSdf`. Full voxels only if runtime carve-anywhere becomes a
   gameplay feature. Order: heightfield -> SDF-carve -> voxels.

**Alternatives considered.**
- *Author the world by hand-editing JSON / placing every object* — rejected: no
  scale, no procedural reuse; the fields+recipes model is the whole point.
- *Vertex-painted scatter weights* — rejected: tessellation-bound; raster masks
  are resolution-independent and shared with material splat.
- *Voxel terrain from the start* — rejected for now: heavy (memory, LOD, meshing)
  and unneeded until runtime carving is a requirement; SDF-carve covers designed
  carves first.
- *Plant/turtle L-system for buildings* — rejected as the building paradigm:
  buildings want a split/shape grammar (subdivide mass -> floors -> facade), a
  cousin but distinct; deferred (world-system-plan §8).
- *Thousands of tree entities for a forest* — rejected: a forest is one region
  entity over instance groups (ADR-0022).

**Consequences / tech debt.**
- New subsystems implied (each its own step): a field/mask layer system, editor
  brushes + overlay rendering, terrain tiling + a streaming manager with
  per-tile overrides, a biome/material splat path, water, and (later) an
  SDF-carve terrain path and a building grammar.
- The current single-heightfield + baked-scatter level path is superseded by the
  world file; it stays until the world system lands.
- Streaming + determinism constrain every generator to be tile-local and
  seed-reproducible.
- Param-schema reuse with ADR-0026 (recipes declare params for the editor).

**Revisit trigger.** Revisit voxels when runtime carving is a gameplay need;
revisit the building generator paradigm once the scatter/plant recipes prove the
Lua substrate; validate the tile/streaming design against the first real
open-world scene.

---

## ADR-0028 — Generators are a layered Lua vocabulary; recipes compose via attach points
**Status:** Pending · **Date:** 2026-06-15

**Context.** A grammar (L-system) has its own declarative rules + a turtle. Open
question: how much can general-purpose Lua (loops, functions, recursion,
conditionals, noise) extend procgen *beyond* a grammar's own rules — e.g. a
sakura = a branch grammar, a separate blossom variant populated onto the tree,
the tree grouped into a grove; and buildings via a split grammar. Current state
(ADR-0023): the L-system is already a C++ engine object exposed to Lua
(`lsystem.create/rule/expand`, `turtle_mesh*`, `leaves`); `assets/scripts/flora.lua`
already drives it with plain Lua — loops build the rule set, place leaf cards,
and merge meshes. This ADR ratifies that model and extends it to composition and
to a building grammar. Detail: `docs/lsystem-botany-plan.md`, `docs/world-system-plan.md` §8.

**Decision.**
1. **Three layers (already the shape of the code).**
   - **L0 — primitives** (C++ exposed to Lua): `sdf.*`, `noise.*`, `mesh.*`, the
     turtle, `polygonize`, `terrain`, `scatter`.
   - **L1 — grammar interpreters** as engine objects exposed to Lua: the
     L-system now; a **split/shape-grammar sibling later for buildings**. Rules
     are declarative *data fed from Lua* (`sys:rule(...)`); hot expansion stays
     in C++.
   - **L2 — free Lua recipes**: full general-purpose code orchestrating L0/L1 and
     *each other*. **A grammar is a tool called from Lua, never a wall** —
     anything it can't express cleanly (field-weighted thinning, collision-aware
     growth, spline bending) is written in Lua around it.
2. **Recipes compose by calling recipes, connected through named attach points
   (sockets)** — frames a parent asset exposes (terminal branch nodes, facade
   panel anchors) that a child recipe populates. Attach points are flagged nodes
   on the asset skeleton (extends ADR-0026). Blossom-on-branch, fruit-on-tree,
   tree-in-grove, and prop-on-building are the same mechanism at different
   scales. Child seeds derive from `parentSeed + attachIndex` (ADR-0002).
3. **Building generation is a split/shape grammar implemented as an L1 sibling
   helper** exposed to Lua (not a plant L-system, not a separate engine);
   deferred, but slots into this model (ADR-0027; world-system-plan §8).
4. **Organic branch geometry is generated at L0/L2, independent of the grammar:**
   curved internodes (Catmull-Rom + rotation-minimizing frame, botany §3.5) plus
   per-ring surface noise (bark bumps / fork swell) so branches read organic, not
   straight tubes.

**Alternatives considered.**
- *Pure declarative grammar (everything as rewrite rules)* — rejected: blossom
  placement, field-weighted thinning, composition, and collision-aware growth
  want general code; a grammar alone can't.
- *Grammar as a sealed engine that returns a finished asset (Lua only sets
  params)* — rejected: forecloses composition and post-processing; the explicit
  goal is Lua loops/functions in the loop.
- *Buildings via a plant L-system* — rejected: paradigm mismatch (ADR-0027).
- *Reimplement grammars in pure Lua* — rejected: expansion is hot; keep it C++,
  expose as objects (ADR-0023).

**Consequences / tech debt.**
- New shared primitive: **attach points / sockets** on assets (ties to the
  ADR-0026 skeleton) — needs a small API (`asset:attach_points(tag)`, a populate
  helper) and a recipe-calls-recipe convention in the Lua layer.
- The L1 grammar surface grows (parametric L-system in; split-grammar later) but
  the L0/L2 contract stays stable — recipes don't change shape as grammars land.
- Organic branches add a curvature + surface-noise pass to the cylinder skinner
  (botany §3.5/§4.1).

**Revisit trigger.** Revisit if composition needs a dependency/graph model beyond
direct calls, or if the split-grammar wants a different substrate than it shares
with the L-system.

---

## ADR-0029 — Branch skinning is generalized cylinders; SDF retained for fusion
**Status:** Accepted (retroactive — describes shipped code) · **Date:** 2026-06-15

**Context.** ADR-0021 made SDF + Surface Nets the geometric-modeling path. For
tree *branches* that is the wrong default: `buildTurtleMeshSdf` floors capsule
radius at ~1.5 grid cells (thin twigs vanish or balloon; cost O(resolution³)),
Surface Nets emits no UVs/tangents (no bark texture — the clay look), and
smooth-min rounds away forks. Phase 1 of `docs/lsystem-botany-plan.md` (§4.1)
shipped a different path in `src/engine/procgen/tree.cpp` (`growTree`); this ADR
ratifies it.

**Decision.**
1. **Branches are skinned as generalized cylinders** — sweep a ring of vertices
   along each branch internode (`addTube` per node→child edge), ring radius from
   the pipe model, **UVs flowing length × circumference** so bark textures and
   tangents (normal maps) work. Cost O(branches × ring-verts); twigs are free; no
   volumetric grid.
2. **This sits *alongside* the SDF path, not replacing it.** It is a Mesh
   generator in the ADR-0021 sense (produces the `Mesh` value type). SDF +
   Surface Nets remains the path for **CSG / organic fusion** (rocks today; the
   planned lower-trunk / root-flare / burl hybrid).
3. **Radii by the pipe model** (da Vinci / Murray's law): a **bottom-up pass**
   over the node tree (`assignRadii`), `r_parent^n = Σ r_child^n`, `n ≈ 2.3`,
   with a tip-radius clamp. The forward `taper` heuristic stays as a fallback.
4. **The bark mesh doubles as the static collider** (collision triangle soup =
   branch vertices/indices; leaves excluded — you bounce off wood, not foliage).

**Boundary (SDF vs cylinder).** Cylinders for all branches now; SDF smooth-union
is reserved for the lower trunk / root flare / burls where fusion genuinely
matters. That **hybrid is not yet implemented** — defining the seam is part of
this decision, building it is owed (botany §4.1).

**Refinement — implemented (2026-06-15).** Branches are now skinned as
*continuous curved* limbs (ADR-0031): the skeleton is decomposed into chains
(each node follows its straightest child as the apical leader; other children
fork off, anchored at the joint), a Catmull-Rom curve is fit through each chain,
sampled at even arc length, and swept with a **rotation-minimizing frame** —
twist-free UVs and real curvature, replacing the per-segment `frameFor`/`addTube`
path. Branch junctions still simply overlap (hidden under bark/foliage); the
branch-collar flare + proper stitching remain later polish.

**Alternatives considered.** Pure SDF (rejected — twigs, missing UVs, O(res³));
metaballs (same UV/cost problems); kit-bashed disjoint cylinders
(`buildTurtleMesh`) (rejected — self-intersecting joints, no continuity, poor
collider).

**Consequences / tech debt.** Two skinning paths to maintain. The trunk-flare
SDF hybrid and the RMF/curvature pass are owed (botany §3.5, §4.1).

**Revisit trigger.** Revisit the SDF/cylinder boundary when the trunk-flare
hybrid is built, or if junction artifacts show through under foliage.

---

## ADR-0030 — Parametric L-system with expression-valued successor parameters
**Status:** Accepted (retroactive — describes shipped code) · **Date:** 2026-06-15

**Context.** The plain `char` `LSystem` cannot carry magnitudes — length/width/
angle live in one global `TurtleParams`, so a self-similar grammar cannot taper
per recursion. ABoP §1.10 parametric modules fix this. Botany plan §3.1 flagged
an open decision: a **restricted subset** (parameters passed through, no
expressions) vs **full arithmetic**. Phase 3a shipped `ParametricLSystem` in
`src/engine/procgen/lsystem.{h,cpp}`; this ADR ratifies what was built.

**Decision.**
1. **Modules carry numeric parameters** (`Module` = symbol + `float` params;
   `ModuleString` replaces the char string for this path). Both coexist: the
   `char` `LSystem` stays for simple grammars, `ParametricLSystem` for trees.
2. **Successor parameters are arithmetic *expressions* over the predecessor's
   formal parameters** — we chose the **fuller** option, not the plan's
   recommended restricted subset. Supported: `+ - * /`, parentheses, unary minus,
   numeric literals, formal names; each compiles to a
   `ParamExpr = std::function<float(params)>` (mirrors the `Sdf = std::function`
   precedent, ADR-0021). E.g. `A(l,w)` →
   `F(l,w)[+(30)A(l*0.7,w*0.6)][-(30)A(l*0.7,w*0.6)]`.
3. **Productions match by symbol *and* parameter arity**; unmatched modules are
   copied verbatim. Repeating a predecessor adds **weighted stochastic
   alternatives** (seeded RNG), as in `LSystem` — deterministic per
   `(params, seed)` (ADR-0002).

**Why fuller than planned.** Tapering by depth (`l*0.7`, `w*0.6`) is the entire
point and needs at least multiply; once an expression evaluator exists, full
`+ - * /` is marginal extra effort and keeps length/width/angle math in the
grammar instead of a second pass. The restricted subset would have forced taper
back into the global `TurtleParams` — the very thing this replaces.

**Alternatives considered.** Restricted pass-through parameters (rejected — can't
express taper); a full embedded scripting expression language / Lua-per-rule
(rejected — overkill on a hot path; four operators cover ABoP grammars); keep
taper in `TurtleParams` only (rejected — global, not per-branch).

**Consequences / tech debt.** A small expression parser/evaluator to maintain;
published ABoP grammars become near copy-paste. The parametric system is now
**exposed to Lua** (`lsystem.parametric` + `tree.skin`, ADR-0032) and supports
**guarded productions** — `A(l):l<=t -> …` — added 2026-06-16: a production fires
only when its condition holds. Guards are full boolean expressions —
`A(l):l<=clear && l>term` — (`||` over `&&` over the six comparisons), so a single
symbol can express a phase *range*. This makes **N-phase trees pure grammar**:
`flora.phased_tree` (Lua) is a majestic three-phase tree — one symbol `A` with
trunk / crown / terminal-cap rules — with **no engine code** beyond the generic
guard support (the proof the language is robust to 4-, 6-, N-phase). Both
`buildGrammar` (C++) and `flora.param_tree`/`phased_tree` (Lua) use guarded
productions. Context-sensitivity (`a<b>c`) remains the one classic L-system
feature still future (botany §3.6).

**Revisit trigger.** Revisit if grammars need context-sensitivity (neighbor
matching), or a second numeric type (e.g. vector parameters) in modules.

---

## ADR-0031 — Curves are a templated cubic Hermite kernel; consumers layer on top
**Status:** Accepted (design ratified; implementation pending) · **Date:** 2026-06-15

**Context.** Curves are wanted in at least three places: procgen geometry (branch
centerlines for the §3.5 organic-branch work / ADR-0029), F-curves for animation
(value-over-time channels), and 2D vector / SVG import. The branch-curvature work
needs one *now*, so the representation is on the critical path rather than
hypothetical. The failure mode is a single mega `Curve` class trying to serve all
three: they disagree on fundamentals — parameter is **arc length** (procgen,
even ring spacing) vs **time** (animation) vs **natural t** (vector); dimension is
Vec3 vs scalar vs Vec2; animation needs tangent *modes* (stepped/auto/broken)
the others don't.

**Decision.** Split the **math kernel** from the **consumers**.

1. **Kernel:** a piecewise-cubic `Spline<T>` templated on the value type
   `T ∈ {float, Vec2, Vec3}` (it needs only `+` and `scalar *`, which the math
   types already provide — same spirit as `Sdf`/`ParamExpr = std::function`).
   Store each knot in **Hermite form (value + in/out tangent)** — the canonical
   representation everything lowers to. The kernel exposes `eval(t)` and
   `tangent(t)`; it lives in **core math** (dependency-free, near `rt_math`).
2. **Authoring front-ends all lower to Hermite knots:**
   - **Catmull-Rom** (pass-through points): tangent = `(p[i+1] − p[i−1]) / 2`
     — for procgen skeletons; trivial authoring.
   - **Cubic Bezier** (SVG, DCC tools): handles convert to Hermite tangents.
   - **Keyframe** (animation): already *is* value + in/out tangent.
3. **Per-consumer services layer on the kernel, each in its own module:**
   - `Path3` (procgen): arc-length LUT (cumulative length → t) for even ring
     spacing + the rotation-minimizing-frame helper. ← the ADR-0029 §3.5 dependency.
   - `AnimCurve` (animation): time lookup + tangent modes — lives in animation,
     **not** the kernel.
   - `Path2` + an **SVG path parser**: lives in asset loading, not the kernel.

**Phasing (YAGNI guard).** Phase 1 — the kernel + Catmull-Rom + arc-length + RMF
helper — is built **now**, justified by the branch-curvature work, and is
headless-testable (`make test`). `AnimCurve` and SVG import come **when their
domains need them**; they are *proof the abstraction is right*, not work to do up
front. Build the kernel and the procgen consumer; stub nothing else.

**Alternatives considered.** A single mega `Curve` class (rejected — the
parameterization semantics conflict, as above). Bespoke per-domain curves with no
shared kernel (rejected — duplicates the cubic math three times; reuse is the
whole point). NURBS / rational curves (rejected — overkill; cubics cover trees,
animation, and SVG, whose elliptic arcs can be approximated by cubic Beziers).

**Consequences / tech debt.** A small templated header kernel to maintain.
Unifies every curve consumer on one evaluator. Ties directly into ADR-0029 §3.5:
the continuous branch sweep is a `Path3` (Catmull-Rom through skeleton nodes) with
an RMF. SVG arcs are cubic *approximations*, not exact, until/unless rational
curves are added.

**Revisit trigger.** Revisit if a consumer needs rational curves (exact conics /
perspective-correct SVG arcs) or degree > 3.

---

## ADR-0032 — Procedural models are N material-parts; placement & scatter consume any N
**Status:** Accepted · **Date:** 2026-06-15

**Context.** The good tree is *two* meshes — opaque bark + alpha-cut leaves — but
the vegetation scatter assumed **one mesh + one material per species**, so it
couldn't place the new tree. Special-casing "two" is wrong: a tree might gain
fruit/moss, a building has walls/glass/roof. The glTF importer already models
this (`ImportedModel = vector<ImportedMesh{mesh, material}>`); procgen just
didn't use it.

**Decision.** A procedural asset is an **ordered list of parts**, each a
`{geometry, material-intent}`; every placement path consumes any N.
1. **Lua return shape:** a flora script returns a single `Mesh` (back-compat,
   one part with the caller's default material) **or a list of parts** —
   `{ mesh=, texture=("bark"|"leaf"), alpha_test=, albedo=, roughness=, metallic= }`.
   `runProcgenModel` decodes both into `std::vector<ScriptMeshPart>`.
2. **Material intent stays GPU-agnostic (ADR-0021/0026):** a part names a
   *built-in procedural texture* and flags (`alpha_test`); the renderer-aware
   loader generates + uploads it (`barkTexture`/`leafTexture`) and builds the
   `RenderMaterial`. Procgen never touches the GPU.
3. **Scatter emits one `InstanceGroup` per (variant, part)**, the parts sharing
   the variant's per-instance transforms — N parts → N instanced draws, no new
   renderer work (instancing already coalesces by mesh handle).
4. **Footprint spacing:** `ScatterParams.minSpacing` (dart-throwing Poisson disk)
   rejects candidates within a distance; the loader defaults it to the canopy
   radius × maxScale × `spacingFactor`, so big meshes don't jumble.

**Alternatives considered.** Special-case two submeshes (rejected — not general;
the next asset breaks it). Merge bark+leaves into one mesh (rejected — can't mix
an opaque and an alpha-tested material in one draw). A full material-asset system
with shared material handles (deferred — overkill now; built-in texture names
cover the cases, ADR-0021 "distill, don't design up front").

**Consequences / tech debt.** `growTree`'s C++ `shape:"tree"` path still spawns
its two entities by hand (could fold onto the same N-part path later). Static
placement of arbitrary N-part Lua models isn't wired (only scatter is). Built-in
texture *names* are a stopgap until a real texture/material binding. A scattered
species may opt into a **per-trunk static capsule collider** (`collide: true`,
auto-measured from the mesh or `colliderRadius`/`colliderHeight`), one body per
instance — done. LOD for distant instances remains owed.

**Revisit trigger.** Revisit when a part needs an authored (non-built-in)
texture or a shared material across assets, or when N-part static placement is
needed.

---

## ADR-0033 — `Skeleton` is a shared L-system value type; consumers attach meaning
**Status:** Accepted · **Date:** 2026-06-17

**Context.** Interpreting an L-system module string with a turtle yields a
branching node structure. It lived privately in `tree.cpp` (`walkSkeleton`/
`Node`), but the same structure is wanted by many generators — trees, mountain
ridge networks, vegetation, rivers, roads, lightning. Branching is one general
pattern, not a tree feature (ADR-0021).

**Decision.** Extract a **domain-agnostic `Skeleton`** value type
(`engine/procgen/skeleton.{h,cpp}`): `SkeletonNode` = `pos, heading, parent,
depth (branch order), radius, distFromRoot, isTip`; `Skeleton` adds
`childLists()`. `buildSkeleton(modules, angleJitter, rng|seed)` runs the 3D
turtle. The skeleton carries **no domain meaning** — *consumers* attach it:
- **Tree:** pipe-model radii + droop + generalized-cylinder skinning + leaves.
- **Mountain range:** lay the turtle's (x,y) plane onto the ground (x,z), height
  per node by `depth`, distance-to-nearest-segment → terrain uplift, then erosion
  (`buildRangeRidges`).
- Future: vegetation, rivers, roads — same skeleton, different reader.

Distilled *from* two real consumers (tree refactor = byte-identical; branching
ranges = new), per ADR-0021's "don't design the universal type up front."

**Consequences.** A new shared value type alongside `Mesh`/`Field`/`Frame`. The
tree's turtle is no longer private. **Owed:** expose `Skeleton` to Lua (a userdata
from `lsystem`), so recipes build skeletons that any consumer (`tree.skin`,
`terrain` ridges, scatter-along) reads — the "L-system makes skeletons for many
things" goal.

**Revisit trigger.** When a consumer needs non-tree graph topology (loops) or
per-node attributes beyond the current set.

---

## ADR-0034 — A bounded, curated world (~16 km, single precision): reverse-Z, spatial partitioning, terrain + object LOD (impostors/HLOD), and sector streaming
**Status:** Pending · **Date:** 2026-06-17

**Context.** Distant-terrain work surfaced a cluster of failures — terrain past
~99 m composited as sky (a fixed `depth >= 0.999` test that maps to ~99 m under a
0.1 m near plane), frustum culling misjudging the one origin-centred terrain mesh,
and LOD ring seams. The target was clarified: **not** an infinite Minecraft world,
but a **bounded, artist-curated "place"** in the GTA V / Horizon Forbidden West
mould — a large chunk of terrain (~16 km across) with forests, rivers, a city, and
walk-to-able distant mountains, with room to grow. That target reshapes the
solution: at ~16 km centred on the origin, **single-precision floats are exact to
~1–2 mm**, so the heavy infinite-world machinery (camera-relative rendering,
floating-origin rebasing, double GPU coords) is *not* needed; the real needs are
correct depth, spatial partitioning, and aggressive LOD. ADR-0027 /
`world-system-plan.md` cover the *content* model (fields + recipes, authored as
data, per-tile overrides); this ADR owns the *coordinate, rendering, and
LOD/streaming* foundation. Detail in `docs/open-world-foundations-plan.md`.

**Decision.**
1. **Bounded, single-precision, origin-centred world; budget ~16 km across**
   (±8 km → ~1–2 mm float precision everywhere). No floating origin, no
   camera-relative rendering, no double GPU coords. **No code may hardcode the
   extent** — the world size is one constant (far plane + streaming-grid extent), so
   growing to ~64–130 km later (still single precision, ~cm precision) is a
   one-line change.
2. **Reverse-Z depth + robust background classification.** Map near→1/far→0 on the
   `Depth32Float` buffer for near-uniform precision across a wide (~16–20 km) far
   plane; background becomes `depth <= 0` (the clear value), retiring the
   `depth >= 0.999` magic constant everywhere (composite/SSR/SSAO/debug). Stop-gap
   before reverse-Z lands: a linearized `linearDepth >= 0.999*far` test.
3. **Spatial partitioning.** A sector grid (and/or BVH/octree) over the world for
   correct frustum culling, streaming decisions, and later occlusion culling —
   replacing the single origin-centred bounding sphere that misjudges large meshes.
4. **Terrain is chunked with tight bounds + geometric LOD** (clipmaps or CDLOD,
   chosen in a later ADR), replacing the origin-centred tile + concentric rings, so
   culling is correct and seams stitch. **A walkable distant mountain is coarse
   terrain LOD, not an impostor** (terrain is a continuous walkable surface).
5. **Object LOD ladder: discrete mesh LOD → impostors/billboards → HLOD.** Discrete
   per-object LOD meshes + terrain LOD first; then **foliage impostors** (billboard/
   octahedral cards) for distant trees so a forest reaches the horizon cheaply; then
   **HLOD** (merged simplified proxy meshes) + building impostors for the distant
   city. Discrete props get impostors; walkable terrain does not.
6. **Sector streaming of a bounded set.** Page authored/generated content in and out
   by distance from the camera over the partition grid — the ADR-0027 §7 streaming
   manager, but over a *finite* sector set, not an unbounded tile map.

Phased, each independently shippable: **0** reverse-Z + sky test + re-enable cull;
**1** spatial partition + chunked terrain w/ tight bounds + terrain LOD; **2** object
mesh LOD + foliage impostors; **3** sector streaming + HLOD/building impostors;
later, occlusion culling.

**Alternatives considered.**
- *Design for infinite/Minecraft from the start (camera-relative + floating origin +
  double/int world coords)* — rejected for this target: over-built for a bounded
  ~16 km world where single precision is already exact to ~mm; kept as the documented
  **upgrade path** if we later go endless or planetary.
- *64-bit integer universe coords (Star Citizen)* — rejected: planetary/space tier.
- *Logarithmic depth* instead of reverse-Z — viable (space sims use it), but
  reverse-Z is simpler with our existing float depth buffer and the cheaper win.
- *Keep one big terrain mesh, just fix bounds/threshold* — rejected: patches symptoms,
  doesn't partition/stream/LOD; won't host a city + forests at frame rate.
- *Turn off the skybox for "true" rendering* — rejected: a skybox is the sky; the real
  needs are robust sky classification (§2), terrain-to-horizon (§4), and atmospheric
  blend (existing fog).
- *Impostors for the distant mountain landform* — rejected: terrain is walkable, so it
  is geometric LOD; impostors are for the discrete props on it and the distant city.

**Consequences / tech debt.**
- Reverse-Z touches every depth consumer (projection, clear, compare, skybox,
  SSR/SSAO/temporal-AO/debug views) — they must flip together; Metal-only, so
  user/viewer-verified, with the offline tracer (absolute world space, no far clip)
  as the precision oracle.
- Impostor/HLOD need an offline bake step (render-to-card / merge+decimate) and a
  LOD-selection + crossfade path — a real chunk of the content pipeline.
- Supersedes the concentric LOD rings (and their seam debt) once Phase 1 lands.
- The temporary diagnostics on the current branch (camera near/far log; bypassed
  per-object frustum cull) are reverted as Phase 0 begins.

**Revisit trigger.** If the world wants to become **endless** (Minecraft) or
**planetary/wrap-around** (walk around the world and return), revisit: that needs
camera-relative rendering + floating-origin rebasing (endless) or a sphere/torus
topology + cube-sphere terrain + atmospheric scattering (planetary) — extending this
ADR, recorded as its own decision. Also revisit the world-size constant if ~16 km
proves too small, and the depth scheme if a second backend lands.

---

## ADR-0035 — Chunked terrain (uniform grid, analytic normals) + AABB frustum culling (open-world Phase 1)
**Status:** Accepted · **Date:** 2026-06-17

**Context.** ADR-0034 Phase 1 calls for spatial partitioning + chunked terrain with
tight bounds + geometric LOD, replacing the single origin-centred terrain tile and
the concentric `generateTerrainLOD` rings (whose bounding sphere misjudged culling
and whose ring boundaries T-junction-cracked). This ADR records the concrete first
cut; the heavier geometric-LOD scheme is deliberately split out.

**Decision.**
1. **AABB frustum culling.** Add `Frustum::containsAABB` (p-vertex plane test) and
   `transformedAABB` (model-space box → world box via the 8 corners). The per-entity
   render path culls against the world AABB using the `boxMin/boxMax` already carried
   in `BoundingSphere` — tight for large/flat meshes where a sphere swallows the sky.
   Instance groups keep the sphere coarse-reject (small props). CPU frustum stays
   forward-Z (the view volume is identical to reverse-Z).
2. **Chunked terrain.** `generateTerrainChunks` tiles a bounded square world into a
   `chunksPerSide²` grid of independently-meshed chunks (each its own `RenderMesh`,
   world-space, identity transform, tight AABB), from the shared `terrainHeight`
   field. Opt-in per level via `"chunks"`; without it the legacy single tile + rings
   path is unchanged. Near chunks (centre within `colliderRadius`) carry a collider.
   Wired into both the viewer ECS loader and the offline `addTerrain`.
3. **Seamless borders without skirts.** Chunks use **uniform** resolution, so
   matching grids share exact edge vertices (no T-junction cracks), and normals are
   taken **analytically** from the height field (`terrainNormal`, central
   differences at a shared eps) so they are continuous across borders (no lighting
   seam). This retires the ring-seam tech debt outright.
4. **Geometric LOD deferred.** Distance-/camera-driven per-chunk resolution with
   crack handling (skirts or CDLOD vertex morphing) is **not** in this phase — it
   rides with the morphing work (Phase 1c, its own ADR). Uniform chunks are the
   foundation both clipmaps and CDLOD could later build on or replace.

**Chosen scheme: chunked quadtree family, not clipmaps.** A bounded curated world
with discrete chunks that also host per-chunk scatter (Phase 2/3) maps naturally to
a chunk grid/quadtree with per-chunk AABB culling and streaming; clipmaps
(camera-centred concentric grids) fit endless GPU-driven terrain less well here.

**Alternatives considered.**
- *Vertical skirts now (to allow mixed-resolution chunks)* — deferred: skirt winding
  is Metal-only and unverifiable on Linux; uniform-res + analytic normals is
  crack-free with no winding risk, and LOD lands with the morph work anyway.
- *Keep the rings, just fix bounds* — rejected: doesn't give per-chunk culling or a
  streaming substrate, and leaves the T-junction debt.
- *Global analytic normals replacing topological everywhere* — out of scope; only
  chunks switch (a genuine improvement: seamless), the legacy mesh path is untouched.

**Consequences / tech debt.**
- Uniform-resolution chunks make a large extent expensive (no LOD yet) — keep demo
  worlds modest until Phase 1c. The world extent is a level parameter, not hardcoded
  (ADR-0034 rule).
- Erosion (`erode`) still applies only on the legacy single-mesh path; per-chunk
  erosion (bake a per-chunk heightmap) is a follow-up.
- Verified on Linux via the offline tracer (the parity oracle: chunk surface ==
  height field, borders seamless — unit-tested + an offline render of
  `assets/levels/chunked.json`); viewer culling is unit-tested (`test_frustum`).

**Revisit trigger.** Phase 1c (geometric LOD: per-chunk resolution + CDLOD morphing
or skirts) extends this; streaming (Phase 3) pages these chunks by distance.

---

## ADR-0036 — CDLOD heightfield terrain (geometric LOD via vertex morphing) (open-world Phase 1c)
**Status:** Accepted · **Date:** 2026-06-17

**Context.** ADR-0035 left terrain as a uniform-resolution chunk grid — correct and
seamless, but every chunk meshes at the same density regardless of distance, so the
bounded ~16 km world (ADR-0034) with walkable distant mountains is unaffordable. The
far field needs to coarsen. ADR-0034's open question (clipmaps vs CDLOD) is resolved
here in favour of **CDLOD on a heightfield**, chosen with the user over geometry
clipmaps (which discard the chunk/AABB/streaming substrate and want GPU height
sampling) and over discrete-LOD-plus-skirts (visible popping). Overhangs/caves are
explicitly out of scope: a heightfield is `y = f(x,z)`, so it gives dramatic canyons
and cliffs but not undercuts — those are a future *volumetric feature layer* (SDF +
the in-tree surface nets), which the representation-agnostic AABB cull already admits.

**Decision.**
1. **Quadtree node selection (CPU, pure).** `selectLodNodes` cuts a restricted
   quadtree over the square world: the root covers the world at the coarsest level,
   leaves are level 0 (finest). A node subdivides when the camera is within the
   finer level's visibility range, else it is emitted whole. Distance is to the
   **nearest point** of the node's XZ box — this *guarantees* adjacent emitted nodes
   differ by ≤1 LOD level (a shared edge has one distance, so a 2-level jump is a
   contradiction), which is exactly the invariant the morph relies on to stay
   crack-free. Coverage is exact (each recursion emits one node or four that
   partition it) — no gaps/overlaps. Ranges double per level by default.
2. **Vertex morphing (the CDLOD core).** Each node is one fixed-resolution grid
   (even `gridRes`, so the next-coarser grid aligns on even indices). Per vertex we
   bake the **morph target** — the height it collapses to on the coarser grid (the
   average of its even-indexed neighbours; even/even vertices target themselves) —
   into `Vertex::tangent` (terrain has no normal map, so the channel is free; no
   global vertex-layout change). The terrain vertex shader lerps
   `worldPos = mix(position, tangent, morphK)` where `morphK` ramps 0→1 over the
   node level's morph band `[ranges[L-1], ranges[L]]` by the per-vertex camera
   distance. So as a node nears the distance where its parent takes over, its
   geometry has already morphed to match — no pop, no T-junction crack against a
   coarser neighbour.
3. **Heightfield reuse.** Node meshes come from the shared `terrainHeight` /
   `terrainNormal` / `terrainColor` (ADR-0035), so visuals match the existing terrain
   exactly and the offline tracer stays the parity oracle. No heightmap *texture* /
   GPU vertex-texture-fetch — height stays CPU-analytic; "heightmap" here means the
   2.5D representation, not a sampled texture. (A GPU-VTF clipmap remains the
   documented upgrade path if node mesh churn ever dominates.)
4. **Normals not morphed (interim).** Height morphs; the fine normal is kept across
   the morph band. The lighting error mid-morph is subtle on terrain; morphing
   normals (or a normal map) is a follow-up if it reads.

**Alternatives considered.**
- *Geometry clipmaps* — rejected for this codebase: discards the chunk/AABB/stream
  substrate and wants GPU height sampling; better for endless GPU-driven terrain.
- *Discrete per-chunk LOD + skirts* — rejected: visible popping; CDLOD is this plus
  the morph, and the morph removes both the pop and the cracks skirts would hide.
- *Changing the global `Vertex` for a morph attribute* — avoided: reusing the unused
  `tangent` keeps every other pipeline and the offline tracer untouched.

**Consequences / tech debt.**
- The selection + morph-target math is pure and **unit-tested on Linux**
  (`test_terrain_lod`): coverage tiles the world, ≤1 neighbour-level invariant, finer
  near camera, morph targets collapse to coarse heights, flat terrain ⇒ identity
  morph. The **vertex-morph shader + per-node draw path are Metal-only and
  unverified** — they need a viewer pass (the standing macOS-only verification gap).
- Node meshes are generated lazily and cached; eviction by distance/budget is
  **Phase 3 streaming**, not here.
- Colliders: only near nodes need them (as today); collider LOD is unchanged.

**Revisit trigger.** Sector streaming (Phase 3) pages these nodes by distance;
overhangs/caves add a volumetric feature layer beside the heightfield; a GPU-VTF
clipmap replaces CPU node meshing if churn dominates a profile.

---

## ADR-0037 — Interactive render budget + display-agnostic tone/grade pipeline
**Status:** Accepted · **Date:** 2026-06-19

**Context.** Standing in the CDLOD forest fell to 12–15 fps and the image read
washed out; from a high peak the finite world's terrain edge showed a hard band
below the horizon. This ADR records the perf + look pass that followed — all
**viewer-verified with the user** on `cdlod.json`/`arena.json` (arena reached a
solid 60; the dense terrain view is tunable to 60), separate from the terrain ADRs.

**Decision.**
1. **Foliage depth prepass (alpha-cut overdraw).** Alpha-cut leaves shade via
   `discard_fragment()`, which forces *late* depth testing, so close-up every
   overlapping leaf card ran the full lit shader. Split foliage into a depth-only
   prepass (writes the nearest leaf depth; alpha cut only) + a lit pass with
   `[[early_fragment_tests]]` and an **Equal / no-write** depth state, so each
   pixel shades once. Both stages reuse `vertexMainInstanced` from one instance
   buffer, so depths match bit-for-bit (incl. wind sway) and the Equal test is
   exact. Scoped to **foliage only** (not all opaque) to avoid the "vanish on
   depth mismatch" cliff; solids and the shadow pass are untouched. Runtime-gated
   by `depthPrepassEnabled`.
2. **SSAO at half resolution + temporal rotation jitter.** Profiling found SSAO
   the dominant frame cost (~33 ms full-res; toggling it took the arena 20→60).
   Run all four AO passes (GTAO + 2 blurs + temporal) at **half res** — the
   composite already upsamples via normalized sampling, as SSR has all along;
   the kernels map their half-res coords to the full-res G-buffer
   (`aoCoordToGBuffer`). GTAO default dropped 48→20 samples; a per-frame
   golden-angle **rotation** that the temporal resolve averages keeps low
   direction counts banding-free. Directions/Steps stay live sliders.
3. **Display-agnostic tone/grade pipeline (HDR-ready).** The composite is
   `grade → view transform → encode`. The **grade** (contrast in log2 around
   middle grey + saturation around luma) runs in **scene-linear, before the tone
   map**, so it is independent of the output encode and carries over unchanged to
   a future HDR output path. The **view transform** is selectable: ACES (existing)
   or **AgX** (minimal fit — gentler highlight rolloff, far less hue skew). A raw
   **gamma slider was deliberately rejected** — gamma is the *encode* step (sRGB
   2.2 today, PQ/EDR under HDR), not an artistic control. Grade defaults neutral.
4. **Atmosphere wiring.** Aerial fog (already in the lit shader, just configured
   too weakly) got live overlay controls + a **"Match Sky"** button; the
   procedural sky now **holds the horizon haze for a band below the horizon**
   (fading to the ground tint only at steep angles), so a finite world's far
   terrain dissolves into sky instead of meeting a hard ground band. A
   `vegetationDrawDistance` override lets draw distance be balanced against fog
   live (pull trees in to where fog hides them).
5. **HDR environment hygiene.** Levels with no `"hdr"` key now **clear** any
   previously-bound environment map on load (the renderer is reused across loads,
   so a prior level's HDR sky used to persist); a runtime `environmentMapEnabled`
   toggle forces the procedural sky for A/B.

**Alternatives considered.**
- *Depth prepass for all opaque (not just foliage)* — rejected: Equal-depth needs
  bit-identical transforms for **every** opaque draw or geometry vanishes;
  foliage-only confines the blast radius and is where the overdraw actually is.
- *Contrast/gamma on the final SDR pixels* — rejected: breaks under HDR; the grade
  belongs in linear before the tone map (the Blender/OCIO model).
- *Tree impostors/LOD for the forest* — deferred; the next scaling lever as density
  grows (ADR-0034 impostors/HLOD).

**Consequences / tech debt.**
- All of the above is **Metal-only, verified in the viewer this session, not on
  Linux/CI** (the standing macOS verification gap). AgX bakes its own display
  encode and was **not** bit-checked against ACES on-device — flagged in TECH_DEBT.
- The grade is HDR-ready but the **output path is still SDR** (clamp + 2.2); true
  HDR display (extended-range layer + PQ/EDR) is unbuilt.
- The sky's below-horizon **ground tint double-serves** as ambient ground-bounce
  (procedural IBL samples it by surface normal) *and* skybox color; the haze change
  pulled undersides slightly bluer. Decoupling is a follow-up.
- The **offline path tracer keeps its own ACES** and gains no grade/AgX — viewer
  and offline tone now differ.
- SSAO in a **full-coverage view** (dense terrain, no sky) still costs ~16 ms at
  half res — a blur/temporal floor, not sample count.

**Revisit trigger.** HDR display output (the grade slots in unchanged);
distant-tree impostors/HLOD when forest density grows; AgX look presets; decoupled
ground-bounce ambient.

---

## ADR-0038 — City generation: a split/shape grammar over a road→block→parcel pipeline, as a world recipe
**Status:** Accepted — Phases 0–3 **implemented**, including the **City Arena**
(the city draped on terrain with street/park trees, `assets/levels/city_arena.json`):
the headless generation pipeline under `src/engine/procgen/city/`, Lua `building.*`
authoring, offline-tracer `shape:"city"` render, and the HLOD proxy; covered by
`tests/test_city.cpp` + `tests/test_script_vm.cpp`. The Metal viewer render,
impostor-card bake, sector streaming, and cross-tile roads (Phase 4) deferred —
they need a GPU and the spatial partition. · **Date:** 2026-06-19

**Context.** The ROADMAP's Tier 4 Phase D names "City / road layout" as a
capstone application, and ADR-0027 §8 / ADR-0028 §3 already sketched the paradigm
(a split/shape grammar, not a plant L-system; a city as a region recipe that
masks out nature). With the plant/scatter recipes now proving the Lua substrate
(ADR-0028/0032), it is time to fix the *shape* of city generation before building
it: how buildings are grown, how roads produce blocks, how it sits in the world
system, and — the question that most changes the design — how far into building
interiors the language goes. Full design and phasing in
`docs/city-generation-plan.md`.

**Decision.**
1. **A city is one region recipe over the world system (ADR-0027), not thousands
   of entities.** `(region, fields, seed) -> content`: it reads a `district`/
   density field (procedural + brush-paintable), **suppresses the natural scatter
   recipes** under its footprint, snaps to `terrainHeight`, and expands to
   instance groups + generated meshes at load (per-tile when streaming lands).
   Individual buildings/props stay render data, not document entities (ADR-0022).
2. **Buildings are grown by a split/shape grammar (CityEngine CGA), an L1 engine
   interpreter exposed to Lua — a sibling of the L-system, ratifying ADR-0028
   §3.** It rewrites a **scope** (an oriented box + frame) by ops
   `split`/`repeat`/`comp`/`inset`/`extrude`/`roof`/`taper`/`setback`/`hollow`/
   `opening`/`prim`/`instance`/`attach`/`material`, emitting multi-part meshes
   (`ScriptMeshPart[]`, ADR-0032). Rules are declarative data fed from Lua; the
   rewrite/emit loop stays hot in C++ (ADR-0023/0028 §1). This is a *different
   interpreter* from the turtle L-system — additive branch growth cannot express
   recursive mass subdivision — sharing only the substrate (seeded RNG, params,
   mesh output).
3. **Roads → blocks → parcels → buildings is a strict one-directional pipeline,
   and a city block is an enclosed face of the planar road graph.** Roads are a
   graph of nodes + `Spline<Vec3>` edges (curve lib, ADR-0031); planarize at
   crossings; extract minimal-cycle faces via a **half-edge (DCEL) next-CW
   traversal**, discarding the outer face; inset each face by road half-width to
   the buildable footprint; subdivide into lots by **recursive OBB split**; grow
   a building per lot via the grammar. Get the road graph right and blocks/lots
   fall out mechanically — so the road network is the design lever.
4. **Scope is Tier A+B — facades plus walkable shells, not interiors** (decided
   with the user). The grammar emits detailed exteriors *and* can `hollow` a mass
   + `opening` real doorways so a ground floor / lobby / atrium is an enterable
   open volume at human scale. It does **not** generate rooms, hallways, connected
   doors, or reachable layouts — that is a separate *floor-plan synthesis*
   generator (graph + reachability constraints), a different paradigm. The seam is
   designed now: the grammar exposes each floor as a **`plate` scope** + attach
   points (ADR-0028 §2), so a future Tier-C interior generator *fills* a plate
   without redesign.
5. **Human scale is a first-class, metric constraint.** The world is broadly
   metric (Jolt gravity; 1 unit ≈ 1 m); grammar split sizes are real meters
   (floor-to-floor ~3–4 m, door clearance ~2.1 m, sill ~0.9 m, railing ~1.1 m),
   pinned as named constants the grammar and the player rig share. A door the
   grammar punches is a door the player fits through.
6. **Impostors/HLOD are render-scale LOD (ADR-0034 §5), after generation, designed
   for now.** Generation produces the meshes; the distant-city draw is a discrete
   LOD → HLOD (merged proxy) → building-impostor (octahedral card baked from the
   mesh) ladder, gated on the spatial partition (ADR-0035/0036). The grammar emits
   a coarse proxy + a clean silhouette alongside the full mesh so the LOD bake is
   cheap later; repeated types collapse into `InstanceGroup`s (the free first LOD
   via `drawDistance`).
7. **Phased grammar-first** (`city-generation-plan.md` §7): **0** the building
   grammar standalone (hero: a **mid-rise mixed-use block**; skyscraper as the
   `repeat`/setback scale test); **1** lot→building (parcels, occupancy); **2**
   roads→blocks (deformed-grid bootstrap → agent/L-system growth); **3** the "City
   Arena" integration target (mirrors the Forest Arena); **4** LOD/impostors +
   per-tile streaming with cross-tile road stitching.

**Alternatives considered.**
- *Buildings via the plant L-system* — rejected (ADR-0027/0028): paradigm
  mismatch; additive turtle growth can't express recursive mass subdivision.
- *A node graph for the grammar instead of Lua* — deferred (ADR-0025): Lua is the
  one procgen authoring path; a visual editor, if ever, emits Lua.
- *Design full interiors (Tier C) now* — rejected with the user: floor-plan
  synthesis is a distinct, research-grade generator (connectivity + reachability);
  walkable shells deliver enterable space at a fraction of the cost, and the
  plate-scope seam keeps Tier C open.
- *Pure-grid roads forever* — rejected as the *target* (fine as the Phase-2
  bootstrap): real cities want density-driven arterials and terrain-aware bends.
- *Voronoi/Poisson parcels* — rejected as default: recursive OBB gives
  street-aligned rows that read as blocks; Voronoi is an organic *variant*.
- *Generate blocks directly (not from roads)* — rejected: blocks are a *derived*
  consequence of the road graph; deriving them keeps roads and blocks consistent
  by construction.
- *Thousands of building entities* — rejected (ADR-0022/0027 §4): one region
  entity over instance groups + generated meshes.
- *Impostors as part of generation* — rejected: a render-scale LOD concern baked
  *from* generated meshes (ADR-0034 §5), gated on the spatial partition.
- *Traffic/agent simulation* — out of scope (world-system-plan §8: "generated, no
  simulation"); any future agent sim lives on the temporal-generator track
  (ADR-0021), not the static-geometry pipeline.

**Consequences / tech debt.**
- New subsystems (**built**): the **shape-grammar interpreter**
  (`procgen/city/shape_grammar.*`), **road-graph generation + planarize +
  half-edge face extraction** (`road_network.*`), **parcel subdivision**
  (`parcel.*`), 2D geometry (`polygon.*`), the **city region recipe** (`city.*`),
  and a Lua `building.*` global in `procgen_bindings.cpp`. All headless,
  deterministic, and tested; the offline tracer renders `shape:"city"`. The
  detailed full-size city is heavy (~3 M triangles at 800 m) — which is exactly
  why the HLOD/impostor ladder below exists.
- **A `roads`/`parcels` Lua surface is not yet exposed** — only `building.*` is
  (the centerpiece); the full road→block→parcel pipeline is C++/level-JSON-driven.
  A Lua surface for authoring road graphs + a city as a *world region recipe*
  (ADR-0027) is the next authoring step.
- **Phase 4 (deferred, needs a GPU / spatial partition):** building **impostor-card
  bake** (render-to-octahedral-atlas) and **HLOD swap/crossfade** — the generator
  emits the inputs (a coarse per-building proxy and a merged `CityModel::hlodProxy`,
  headless-tested) but the bake and LOD selection are Metal-side (ADR-0034 §5).
  **Sector streaming + cross-tile road stitching** (ADR-0027 §5) — the city
  generates whole today (Forest-Arena style); per-tile generation with a global
  graph clipped per tile is owed.
- The **offline tracer bakes the detailed city** (no LOD — it is the quality
  oracle); per-vertex colour carries hue on white materials (the tree convention),
  so glass reads reflective via metallic/roughness only.
- **Cross-tile roads** are the hard streaming problem (the graph is city-global,
  not tile-local): first cut generates a bounded city region *whole* (Forest-Arena
  style); per-tile clipping + boundary stitching is deferred to Phase 4 (ADR-0027
  §5).
- **Human-scale constants are loose today** (the player eye-height offset is 0.7);
  pinning the metric reference is part of this work.
- **Impostor/HLOD bake** is a real chunk of the content pipeline (render-to-card /
  merge+decimate + LOD selection/crossfade), owed when the city is large enough to
  need it — same lever as the distant-tree impostors already owed (ADR-0034/0037).
- Interiors (Tier C), curved/non-rectilinear masses beyond `taper`/`setback`, and
  interior streaming are explicitly **not** in this decision.

**Revisit trigger.** When enterable **interiors with sensible layouts** (Tier C)
become a goal — design the floor-plan generator that fills plate scopes; when the
road network needs to graduate past the deformed-grid bootstrap (agent/L-system vs.
tensor fields, `city-generation-plan.md` §9); when **cross-tile road stitching** is
needed for streaming (ADR-0027 §5); or if the split grammar wants a substrate it
does not share with the L-system (ADR-0028 revisit).

---

## ADR-0039 — Materials as named assets; procedural surfaces baked to shared tiling textures
**Status:** Accepted — **Phase A** (named material library) and **Phase B**
(bake procedural surfaces to a PBR texture set, offline sampling) **implemented**;
the Metal-viewer bind of the baked maps and runtime `Renderable → MaterialHandle`
indirection deferred (viewer is macOS-gated; the indirection is a wide call-site
change). Covered by `tests/test_surface_maps.cpp`, `tests/test_level_writer.cpp`
(named table round-trip). · **Date:** 2026-06-20

**Context.** Material variation rode entirely on per-vertex colour, then on a
world-space *analytic* surface library (brick/concrete/…) evaluated per pixel in
the lighting shader, selected by an id packed into material flag bits. Two
problems a production engine wouldn't have: (1) materials weren't *assets* —
every entity inlined its own block, none were shared/named; (2) a per-pixel
pattern `switch` in the lighting megashader doesn't scale, gives no
mip-filtering (distant facades alias), and produces only colour — no relief,
roughness, or AO, so a "brick" wall looked like a flat sticker.

**Decision.**
- **Materials are named assets (Phase A).** The level gains a top-level
  `"materials"` table; an entity's `"material"` is a string reference or an
  inline object. Both loaders resolve and **dedup** (referencing entities share
  one material); `SourceSpec.materialName` carries the reference so `LevelWriter`
  round-trips it (definition emitted once into the table). No runtime handle
  indirection yet — the resolved value still lands on the existing
  `RenderMaterial`/`Scene` material, so there is no renderer change.
- **Procedural surfaces are baked to shared tiling textures (Phase B).** The
  analytic functions become CPU bakers (`surfaceMaps`, mirroring `barkMaps`):
  one run produces a seamless **albedo + normal + metallic-roughness + AO +
  height** set per surface; every material that wants "brick" samples the one
  shared set. This is how modern engines use procedural materials — bake once,
  share, sample with mip-filtering — instead of per-pixel pattern math. A brick
  wall now carries real **relief (normal)**, matte mortar (**roughness**) and
  crevice **AO**. The offline tracer gained albedo/normal/MR/AO sampling (it had
  none) with a world-planar tiling frame, so it previews the baked materials.

**Consequences / deferred.** The Metal viewer already samples
albedo/normal/MR/AO, so binding the baked maps there is mostly loader work, but
it needs a world-planar (or world-scaled-UV) tiling decision and is macOS-gated,
so it's unverified and deferred. True runtime `Renderable → MaterialHandle`
indirection (so editing a material asset propagates to all users) is a wide
change — material is embedded by value across ~dozens of call sites and edited
through a `World`-only registry hook — and is deferred behind the verifiable
data layer. The analytic `applySurface` remains as the city's in-code path until
the city's materials are converted to baked-texture assets too.

---

## ADR-0040 — Building realism: height drives structure & cladding; tripartite composition
**Status:** Accepted — **Pass A** (street-level fixes + the height→material rule)
**implemented**; **Pass B** (material palettes, base/shaft/crown differentiation,
curtain-wall mullions/spandrels, a crown kit) planned. · **Date:** 2026-06-20

**Context.** The generated buildings read as a uniform mid-rise field. Three
reasons, all of which violate how real buildings work:
1. **Material is a per-district dice roll independent of height** — a 40-storey
   brick tower is possible. In reality material follows the *structural system*,
   which follows *height*: load-bearing masonry doesn't scale (the Monadnock
   Building's 6-foot brick base is why), so around ~12 storeys the wall stops
   holding the building up and becomes a lightweight **curtain wall** hung off a
   frame — which wants to be glass + metal. Short ⇒ brick/stone with punched
   windows; tall ⇒ glass-and-metal skin on a frame.
2. **No base/shaft/crown reading** — the base looks like the shaft, so buildings
   are extruded boxes. A real building has a heavier base (lobby/retail, bigger
   openings, a capping cornice), a repetitive shaft, and a distinct crown.
3. **Street-level defects** — the graded **pad sits too high** and the
   **base-course plinth runs across the entrance**, clipping the doorway ("the
   foundation eats the base"); **pilasters run full height arbitrarily** instead
   of being a deliberate base element or a vertical-expression style.

**Decision.** Couple material and form to height, the way real construction does:
- **Height → structure → cladding bands** (the master rule). Cladding is chosen
  by storey count, not independently: low (1–4) masonry brick/stone/stucco with
  *punched windows*; mid (5–11) framed masonry/precast (brick/concrete); high
  (12–24) and super-tall (25+) a **glass curtain wall** (glass + metal/precast),
  never brick. Corrugated metal stays an **industrial** cladding only.
- **Material-family coherence.** A building draws its wall/trim from one coherent
  family; the only legitimate second treatment is a different *base* (Pass B),
  never brick mixed into a glass tower.
- **Tripartite composition.** Base ≠ shaft ≠ crown. Pass A keeps the taller
  glassy ground floor + capping string-course; Pass B differentiates base
  material/treatment and adds a crown kit (parapet for low; mechanical
  penthouse + rooftop water tank + cap for tall).
- **Pilasters are compositional, not default.** Restricted to the **base** (piers
  framing the storefront, capped by the base cornice) — full-height piers become
  an explicit "vertical expression" style in Pass B, never sprinkled.
- **Street-level corrections.** Lower the block pad so the building meets the
  sidewalk; the base course **steps around the entrance bay** and is sized to the
  building rather than a flat tall plinth.

**Consequences.** Variety now comes from *district → height → structural system →
cladding*, with controlled randomness *within* a coherent family — diversity that
obeys the same constraints real buildings do, instead of random mixing. Pass A is
mostly rule-coupling + bug-fix (no new geometry kinds). Pass B adds the palette
system, base/shaft/crown geometry, curtain-wall mullion/spandrel detail, and the
crown kit — the items ADR-0038's facade work deferred.

---

## ADR-0041 — Instancing as engine-wide infrastructure: a path-tracer BLAS/TLAS to match the realtime instance groups
**Status:** Accepted — **Phase 1** (path-tracer BLAS/TLAS + a backend-neutral
instance payload) **in progress**; **Phase 2** (city emits props as instances;
both backends consume them) planned. · **Date:** 2026-06-20

**Context.** The procedural city merge-bakes every repeated street object — ~600
street trees, plus lamp posts and traffic signals — into one giant vertex-coloured
mesh per material class. That is why the city trees are deliberately crippled
(`makeCityTree` disables real leaves and fakes the canopy with sphere blobs): a
real L-system tree with alpha-cut leaf cards, multiplied by ~600 baked copies,
explodes the triangle budget, and the single prop mesh has no slot for a separate
alpha-tested leaf material. The fix the user asked for ("actual trees, like the
CDLOD demo") is blocked on *how the city is split into meshes*, not on the
renderers — both the path tracer and the realtime viewer can already draw a full
leaf-card tree (the hero `shape:"tree"` / forest scatter).

Crucially, the realtime half of instancing **already exists**: `InstanceGroup`
(components.h / renderer.h) carries a shared `MeshHandle` + one `RenderMaterial`
+ a list of world matrices, and `Renderer::drawMeshInstanced` coalesces them into
one instanced draw (`vertexMainInstanced`); `loadVegetation` scatters the forest
this way. The gaps are therefore narrow and specific:
1. **The path tracer has no instancing at all.** `Scene` is a flat triangle soup
   under one kd-tree; scatter instances are skipped offline (tech-debt register:
   "Expand scatter instances to triangles or share the generator"). 600 trees ×
   thousands of triangles each, expanded to triangles, is both slow to build and
   memory-heavy.
2. **The city doesn't use instance groups** for its props — it bakes them.

**Decision.** Make instancing a first-class, backend-neutral concept the
procgen layer emits and *both* renderers consume, and give the path tracer the
two-level acceleration structure that makes it pay off — the standard solution
(a BLAS per unique mesh, a TLAS over placements; ray is transformed into instance
space, not the geometry into the world).

- **Path tracer (Phase 1, this slice).** A `MeshProto` is a prototype mesh
  (triangles in local space + its own kd-tree **BLAS** + local bounds), built
  once. A `SceneInstance` references a proto by index and carries `worldFromLocal`
  + the cached `localFromWorld` inverse + a normal matrix + a world AABB. `Scene`
  gains `protos` + `instances` + an **`InstanceBVH` (TLAS)** over the instance
  AABBs. On a query the ray walks the TLAS; for each candidate the ray is mapped
  into the proto's local frame (origin and **un-normalised** direction, so the
  hit `t` is shared between frames), intersected against the BLAS, and the local
  hit (point/normal/tangent) is mapped back to world. Alpha-cut visibility,
  vertex colour, UVs and per-proto materials all flow through unchanged. The
  legacy flat-triangle path is untouched, so existing scenes (Cornell box,
  textured arena) render identically; instances are purely additive.
- **Backend-neutral payload (Phase 1).** Generators hand back instance groups as
  *(prototype mesh, material descriptor, world matrices)* — the same shape as the
  realtime `InstanceGroup`, but renderer-agnostic. The viewer loader turns each
  into an `InstanceGroup` entity (existing path); the offline loader turns each
  into a `MeshProto` + `SceneInstance`s (new path). One generator output, two
  thin adapters — paying down the duplicated-bake seam that ADR-0038's two
  loaders otherwise force.
- **City (Phase 2).** `makeCityTree` grows a *real* tree (bark + alpha-cut leaf
  cards, thicker trunk/limbs), kept as a handful of variant protos. Street trees,
  lamp posts and traffic signals become instance groups (one proto, many
  matrices) instead of baked geometry — so the triangle budget is one tree, not
  600. Trees get curated, evenly-spaced rows on the verge with tree pits, the
  remaining items from the user's tree feedback.

**Consequences.** The path tracer stops paying memory/build cost linear in
instance count (one BLAS shared by all placements), and the offline render
finally shows the scattered foliage it currently drops. The city's "objects on
the street" and, later, repeated building archetypes become cheap to multiply —
the foundation for growing city complexity. The same instancing substrate serves
every procgen project (forests, rock fields, cities), not just this one. Cost:
the TLAS adds a second acceleration structure and a per-instance ray transform
(a few matrix-vector products on the candidates the TLAS returns); negligible
beside the BLAS traversal it gates. Phase 1 is offline-testable headless; the
realtime instanced draw path already exists and stays as-is.

---

## ADR-0042 — The city as data-driven Lua recipes over a bound C++ vocabulary
**Status:** Accepted — **Phase 1** (street-furniture recipes) **in progress**;
Phases 2–4 (op-vocabulary, block/district recipes, full-city composition)
planned. · **Date:** 2026-06-20

**Context.** The city's generative pieces are written in C++: `street_kit.cpp`
*is* what a lamp post / traffic signal looks like, `shape_grammar.cpp` is what a
building looks like, and `city.cpp` hard-codes how the parts assemble. But the
engine's intended architecture — already real for flora (`flora.lua`), the gun
viewmodel (`gun.lua`), and *per-building* authoring (`city.lua` calling
`building.grow`) — is **a C++ vocabulary of fast primitives with the recipes
living in Lua** (ADR-0023 ScriptVM, ADR-0028 the L1 grammar, ADR-0030 the
parametric L-system + `tree.skin`, ADR-0032 flora). The Lua binding surface
(`procgen_bindings.cpp`: `mesh.*`, `sdf.*`, `noise.*`, `lsystem.*`, `tree.*`,
`building.grow`, `scatter.*`) simply *stops at the building*: the street kit, the
prop placement, and the whole road→block→parcel→assembly are C++-only and have no
Lua surface. So a street lamp is C++ not because that is its right home, but
because the bridge for that vocabulary was never built — the city was grown
C++-first to get topology and the BLAS/TLAS correct under test.

The goal: be able to construct the city **in parts via Lua** — an individual
prop, a building, a block, a district, or the whole city, or anything in between
— and tune every piece from script without recompiling.

**Decision.** Progressively expose the city's generative pieces as Lua-bound
primitives + recipe assets, behind one composable contract, while the
performance-critical substrate stays in C++.

- **The boundary (what stays C++ vs moves to Lua).**
  - *C++ substrate (stays):* mesh ops, the split/shape-grammar **interpreter**
    (the rewrite/emit loop), instancing/BLAS/TLAS, noise/L-systems, and the
    road→block→parcel **solver**. These are hot, stable, reused by every procgen
    project. A solver is an algorithm, not a recipe.
  - *Lua recipes (move):* *what* a part looks like (a lamp is a 4.6 m pole + a
    glowing head), placement rules ("lamps every 30 m, alternating"), district
    cladding rules, and every tunable number — hot-reloadable, art-directed.
  - Single source of truth: a Lua binding calls the *same* C++ builder the city
    uses (e.g. `streetfurniture.lamp{…}` → the C++ lamp builder), so exposing a
    part never forks its geometry.
- **One composable contract — the `model` value.** Every recipe (a lamp, a
  building, a block, a city) returns the same shape: named part meshes + instance
  groups + colliders + attach points. Models nest — a block model embeds building
  and prop models; a city model embeds block models — so the *same* recipe call
  is valid at every granularity ("individual parts or collected together … or
  anything in between"). This generalises the mesh value `building.grow` already
  returns.
- **Phased migration (convert piece by piece; each phase ships green).**
  1. **Street furniture** (this slice): parametrise the lamp/signal builders in
     `street_kit`, bind `streetfurniture.lamp{…}` / `.traffic_signal{…}`, ship
     `streetfurniture.lua`. Proves the bind → recipe → tune loop on the smallest
     self-contained part — exactly the "a street light should be a generative
     asset" case.
  2. **Op-vocabulary:** bind the scope/`emit*` ops (`emitBox`, `emitParapet`,
     `emitShell`, `splitScope`, instancing groups, part/material ids) so arbitrary
     props and building details are authorable in Lua.
  3. **Block & district recipes:** expose the solver's outputs (block faces,
     verge points, intersection corners, lots) so a Lua recipe decorates a block
     / zones a district by rule.
  4. **Full-city composition:** a `city.lua` composes the above into a whole-city
     model; the C++ `generateCity` becomes either a thin host that runs the city
     recipe or a fast default kept in parallel.

**Colliders follow the geometry.** A `ProcModel` carries a `colliders` channel
alongside its render parts and instances, because a procgen scene is not solid
until it has physics. Since procgen scenery is *static world geometry*, collision
follows the actual generated mesh rather than a bounding approximation: the Lua
surface is `m:add_solid(mesh)` (render + collide the same triangles — building
shells, foundations), `m:collide(mesh)` (a standalone static trimesh — terrain,
roads), `m:add_collider{shape=...}` (a primitive only where the shape truly is
one — a round tower is exactly a capsule), and an `add_instances(..., {collide=})`
spec so a verge of street lamps is solid (a thin capsule each). The viewer's
loader turns each into the same `Collider`/`MeshCollider` + `RigidBody` the
hand-authored loaders build; the offline path tracer (no physics) ignores the
channel — the one legitimate renderer divergence. Colliders live in the recipe,
so they regenerate with the geometry on every load (no extra round-trip state).

**Consequences.** Authoring iteration moves from recompile-to-tune to
edit-the-script (hot reload), and the same generative vocabulary serves every
procgen project, not just the city. Geometry never forks because the bindings
wrap the existing builders. Costs: the binding surface grows, and during the
migration a part may exist as both a C++ default and a Lua recipe until the C++
caller is switched to source it from script; the deterministic, headless,
test-covered C++ path is the safety net so each phase lands without regressing
the city. The road/parcel solver deliberately stays C++ — Lua orchestrates and
decorates its outputs, it does not re-implement the algorithm.

---

## ADR-0043 — Procedural textures composed from primitives (a 2D field vocabulary)
**Status:** Accepted — the texture-field vocabulary + bake + Lua binding **done**;
applying baked textures to model parts (material maps on script geometry) is the
next slice. · **Date:** 2026-06-21

**Context.** Procedural textures existed only as *presets*: `surface_maps.h` bakes
a full PBR set for a named `Surface` (Brick, Concrete, …), and a recipe could only
*ask for* one. That violates the ADR-0042 engine rule the same way pre-baked street
furniture did — "you should be able to *build* a brick texture from primitives, not
just request the existing one." Materials are the next domain to make Lua-authorable
piece by piece.

**Decision.** Add a compositional 2D-texture substrate that mirrors the SDF field
exactly — *a texture is a function*, so primitives are closures and combinators
wrap them (`texture_field.h`: `Field2 = std::function<double(double,double)>`).
- **Primitives:** `constant`, `noise`, `fbm`, `checker`, `brick` (a running-bond
  lattice with mortar gaps + per-brick variation), `gradient_y`.
- **Combinators:** `add`, `mul`, `mix`, `scale_bias`, `clamp` — immutable
  composition (each returns a new field).
- **Bake:** `bakeFieldGray` (roughness/height/AO) and `bakeFieldColor` (lerp two
  colours by a mask → albedo) → `TextureData`.
- **Lua surface (ADR-0042):** a `texture.*` library over the same C++ functions —
  `texture.brick{…}:mul(texture.fbm{…}:scale_bias(…))` then `texture.bake_color(…)`
  → an `Image`. A brick wall is *built*, not requested (`brick.lua`). A
  `--bake-texture <recipe.lua>` mode previews any recipe to PNG.

**Consequences.** The same field substrate serves every map type and tiles by
construction; a brick (or any) texture is now authored and tuned in script and
hot-reloadable. The C++ `surface_maps` presets remain as fast defaults, but they
are no longer the only way to get a texture.

**Update — material bundle + bake-onto-part (the graph→maps→mesh pipeline).**
The authoring DAG (`Field2`) is the right abstraction independent of where it
runs; a material is a *bundle of map fields* (albedo/normal/roughness/…), baked
once to a texture set, then applied as an ordinary `Material`. Decisions:
- **Bake on CPU, sample at render.** Compositing is baked once per material, not
  done in realtime, and the bake is **renderer-agnostic** — both the path tracer
  and the viewer sample the *identical* maps. This keeps the engine rule "the
  renderer is the only divergence." A GPU-shader bake would tie material authoring
  to one graphics backend (Metal-only) and is rejected as the default; it stays a
  possible future optimisation of the *same* DAG.
- **One über-shader + baked maps, not per-material fragment shaders.** The graph
  lowers to *textures*, never to a unique shader permutation, so a procedural
  material and an artist's textures are indistinguishable at render time (the
  glTF↔procedural unification rule). Many named material recipes (red_brick,
  weathered_brick, …) are authored and assigned per part — materials are
  first-class reusable values like the street furniture.
- **No authored UVs required.** Parts are textured by the world-planar tiling
  frame (`surfFrame`, `meshUV=false`) at a per-material `tile` size, so any
  procedural mesh wears a material without per-vertex UV authoring; authored UVs
  (glTF) are honoured where present.
- **Carrier:** a `ProcModel` part becomes `{mesh, ProcMaterial}` (`ProcMaterial`
  = baked albedo/normal maps + roughness/metallic + tile). `material.new{…}` (Lua)
  bundles baked `Image`s; `model:add(mesh, material)` attaches it; `bakeProcModel`
  binds the maps into the path tracer's `Material` slots.

**Update — the field substrate generalises to terrain.** The same closure-graph
idea extends to a world-space heightfield (`terrain_field.h`: `HeightField =
function<double(double,double)>`): primitives (`noise`/`fbm`/`ridged`/`warp`/
`terrace`) + combinators (`add`/`mul`/`scale`/`max`/`min`/`mix`/`clamp`) +
`bakeHeightMesh` (tessellate to a grid mesh with gradient normals). Lua: `terrain`
is a *callable table* — `terrain(params, seed)` still runs the C++ preset
(`generateTerrain`, back-compat) while `terrain.fbm{…}:max(terrain.ridged{…})` and
`terrain.mesh(field, …)` compose and tessellate. So terrain is now *built from
primitives*, not only requested — the same move as textures, one domain over.

---

## ADR-0044 — Road generation: pluggable generators + terrain-aware routing (planned)
**Status:** Proposed — captures the design for the next slice. · **Date:** 2026-06-21

**Context.** Roads today are a single generator: `gridRoads` (a deformed grid)
→ `planarize` → `extractBlocks` → grade (Laplacian-smoothed node heights) →
`TerrainFlatten` cut/fill under roads + block aprons. That pipeline is solid, but
(1) only a Manhattan-ish grid exists — no radial or organic patterns; (2) roads
are laid *regardless* of terrain and the terrain is then flattened to fit, rather
than routed to *follow* it; (3) a sloped block is one flat apron pad, never
terraced — so foundations can't stairstep on a hill.

**Decision (planned).** Keep the solver in C++ (it's an algorithm), make the
*generators* pluggable and the *pattern/params* Lua-authored:
- **Pluggable generators**, each returning a `RoadGraph`, so `planarize`/blocks/
  grade/flatten stay shared: `gridRoads` (have), `radialRoads` (ring + spoke),
  and an agent/tensor-field **grower** for organic + terrain-following. Graphs can
  be unioned (a radial core into a grid fringe is two generators → one planarize).
- **Terrain-aware routing** lives in the grower: weight growth by slope so roads
  seek saddles and follow contours (tensor-field / agent road growth), leaving the
  existing `TerrainFlatten` system less to cut.
- **Lua surface:** `city.layout` gains `pattern = "grid"|"radial"|"organic"` +
  density / slope-tolerance, so *which* network and *how it answers the terrain*
  is a recipe; the graph math stays C++.
- **Stairstepped lots** extend `TerrainFlatten`: split a sloped block into stepped
  terrace bands (flat pads at stepped heights + retaining walls) and seat building
  foundations on their band — several stepped pads instead of one.

Sequenced after the terrain heightfield (done) so roads have richer terrain to
respond to.

**Landed so far.** The `TerrainFlatten` cut/fill is now exposed to Lua as a
HeightField combinator: `terrain.conform(field, layout, {margin, falloff})`
samples the field at each road node, builds a constant-grade ramp corridor per
edge (`makeFlattenRamp`), and returns a conformed field that levels the ground to
each road (flat across, a single incline along) and eases back to the natural
slope across `falloff`. Recipes build the terrain mesh *and* the road ribbon on
the conformed field, so urban roads sit flat on hilly terrain instead of
rippling. `applyFlatten` (the pointwise blend) was promoted from a `terrain.cpp`
static to a public function so the combinator shares the exact cut/fill math the
noise terrain uses.

Sidewalks also landed: `buildRoadMesh` grows a raised kerb skirt along the
carriageway edges and around the junction corners (`sidewalk`/`curb` params) — a
curb lip facing the street, a concrete slab top a `curb` height above the road,
and an outer face dropping back to the ground — emitted through the winding-aware
`MeshBuilder::emitQuad`, vertex-coloured concrete in the same mesh.

Lane markings + multi-lane: `buildRoadMesh` paints a double-yellow centreline,
dashed white lane dividers, and solid edge lines as thin raised stripes on the
carriageway (ribbons only, so junctions stay plain). Lanes mirror each side of
the centre and their count grows with the road width, so arterials read as
multi-lane for free.

Block pads + parks: `terrain.conform` now also takes `pads` ({y, poly}) and
flattens each to a level plot (`makeFlattenPad`) alongside the road ramps. The
recipe (`city.lua` `plan_blocks`) classifies each road-graph face from the
*natural* terrain — faces flat enough (relief under a threshold) become level
plots seated at the local height; too-hilly faces are left as natural green
hillside — and a deterministic slice of the developed plots are parks (a flat
lawn) rather than buildings. So a block is now a container that the terrain is
conformed to.

Lot subdivision: a block is now partitioned into building parcels rather than
taking one oversized building centred on the whole face. `city.lots(block, ...)`
exposes the C++ `subdivideBlock` (recursive OBB bisection) to Lua, returning each
lot as an oriented box { cx, cz, w, d, yaw, area }; `mesh.place` seats a part at a
position + yaw. The lot is `inset` off the road centrelines first (so parcels
clear the carriageway + sidewalk) and squared to its own box — the yaw is the lot
edge facing the street (chosen by the parcel's outward frontage), so a grid
building fronts the road *perpendicular* instead of skewing toward the corner the
raw radial frontage pointed at. `city.lua` `plan_blocks` subdivides each
developable face and seats a building *sized to fit* each lot (skipping lots too
small — a left gap), turned to the parcel. The lot is dressed as a unit (a "lot
recipe"): a level ground slab (paved forecourt downtown -> lawn outward), the
building set back behind a forecourt at a district-dependent coverage, an
open-front hedge framing garden lots, and a small `diagonal_chance` that turns
(and shrinks) the occasional building as *deliberate* variety. So buildings sit
inside their plots at real urban density, fronting the street, with gardens —
not a field of oversized boxes.

Curved roads as a first-class primitive (sampled, not analytic): a `RoadArc`
(centre/radius/sweep) is the *source* geometry of a curved road; `sampleArc`
samples it into the planar graph as a fine polyline with a bounded chord error
(≤0.15 m), so the existing segment-based pipeline (planarize/blocks/parcels) is
untouched while the road reads as a true smooth curve and lots line the arc. The
deliberately-deferred part of "full curved topology" is *analytic* curve–curve
intersection + curved-boundary block faces — sampled instead because the result
is visually identical for a fraction of the cost/risk. Straight roads (the grid)
are just edges; the radial generator builds its rings from arcs.

Radial = Place de l'Étoile: `radialRoads` was a wagon wheel (coarse polygon
rings, all spokes converging to one centre node = a spike). Rebuilt: each ring is
a true circular arc (sampled fine), the avenues (fewer now, default 8) radiate
from the inner **roundabout** (ring 1) outward and never touch a centre point, so
there is no spike and the disc inside ring 1 is an island.
`city.lua` fills that island, when `center_plaza` is set, with a round paved plaza
+ a monument (an Arc/obelisk stand-in) instead of subdividing it — the avenues
radiate around it. Sidewalks follow the chorded rings, so they curve too. The grid
generator is untouched.

Hub plaza + building variety + parameterized recipes: a many-armed junction
(`plaza`/`plaza_min_arms`) trims every arm back to the plaza radius so the pad
fills a clean circular plaza instead of a cramped fan — the radial hub reads
cleanly. The building grammar's variety (massing `shape` = box/cylinder/pagoda,
facade `style` = brick/concrete/stucco/glass/metal, setbacks, bay width) is now
exposed to Lua, and `city.lua` picks an archetype per plot by district + a hash,
so the skyline is varied (glass/metal towers downtown, some round; concrete/brick
mid-rises; brick low-rises). Levels parameterize a recipe through an `opts` block,
marshalled into a Lua global `args` (`setRecipeArgs`): the same `city.lua` drives
`city_lua.json` (grid) and `city_radial.json` (radial + plaza) from the level
alone. Stairstepped terrace lots and the terrain-following grower remain as above.

Tensor-field generator — the dovetail of radial and grid (after Chen et al.,
SIGGRAPH 2008): rather than stamping one pattern, `tensorRoads` builds a
continuous field of road *orientations* and traces streets along it, so the
layout morphs smoothly from radial in the core to a grid at the rim with no seam.
The field is a symmetric traceless tensor `[[a,b],[b,-a]]`; its two eigenvectors
are everywhere perpendicular and have no head/tail (a road has no direction),
which is exactly why a *tensor* field blends without the cowlick singularities a
vector field gives. We sum a radial singularity at the centre (weight decaying as
`exp(-(r/decay)^2)` — its eigenvectors give avenues + rings) and a constant grid
basis (the rim lattice); roads are evenly-spaced streamlines (Jobard & Lefebvre)
of both eigenvector families, integrated RK2. The separation rule is kept *tight*
(`dStop ≈ 0.4·spacing`): same-family streamlines are parallel and only crowd when
the radial spokes converge, so a small stop curbs near-duplicate slivers without
truncating a line at the cross-family intersections that *form* the blocks — that
balance is what lets the usual `planarize`/`extractBlocks` pass recover clean
faces (sparse truncation starved it; none let parallel lines bunch into slivers).
Exposed as `city.layout{ pattern="tensor", spacing, step, grid_angle,
radial_strength/decay, grid_strength }`. *Why not space colonization:* SCA grows
organic branches toward attractors — great for medieval sprawl or knitting two
districts across an irregular gap, but it can't make a crisp grid or true rings,
so it's a connective layer, not a district generator.

Lua recipes draped on the CDLOD terrain (the script sibling of the C++ city's
`onTerrain`): a `shape:"script"` entity flagged `onTerrain:true` is *pre-run*
before the terrain is meshed, with the level's natural `terrainHeight` injected as
the Lua global `ground` (`setGlobalHeightField`). The recipe samples it to seat
its city and calls `terrain.conform(ground, layout, …)` — which now returns the
conformed field *and* an opaque regions handle — then hands the regions to
`m:conform(regions)`, recording cut/fill footprints on the model (`ProcModel`
gained a `flatten` channel). Both loaders (viewer `level_loader`, offline
`level_scene`) fold those footprints into `TerrainParams.flatten` and grade the
CDLOD ground flat under the roads/blocks, then spawn/bake from the cached model so
the recipe runs once and the walkable terrain meets the carriageways. This is what
lets `twin_cities.lua` place a radial and a tensor city (plus a graded highway and
an instanced L-system forest, `lsystem_tree.lua`) on one walkable CDLOD terrain.

---

## ADR-0045 — Grade-limited road routing (a road follows the terrain at a walkable grade)
**Status:** Accepted · **Date:** 2026-06-21

**Context.** ADR-0044 conforms the terrain *to* the road: each edge becomes a
straight ramp plane between its two endpoints, and `applyFlatten` cuts/fills the
ground to it. That is right for an urban street on a chosen flat site, but it is
wrong for a road crossing open terrain. The highway in `twin_cities.lua` was three
nodes (start, a hand-picked "lowest" mid-waypoint, end) → two long ramps; cross a
hill and one straight plane either carves a canyon through it or rides up and over
at whatever grade the endpoints imply — with no bound on steepness. City layouts
are likewise generated in pure 2D and conformed afterward, so a road can run
straight up a slope the character could never walk. The symptom (playtest): roads
draping over mountain tops and terrain poking through the carriageway, because a
single long ramp only matches the ground at its two ends.

The missing idea: **a road can't climb what the player can't walk.** Grade must be
a *constraint on the path*, not something the conform discovers after the fact.

**Decision.** Add a grade-limited router that lays a road as a path which *follows*
the ground while holding a maximum grade — so it bends around a peak, or
switchbacks up it, instead of going straight over. `routeRoad(field, from, to,
RouteParams)` (in `terrain_field`, on the `HeightField` vocabulary) runs **A\*** over
a `cell`-spaced grid of the heightfield, 8-connected: a step is *forbidden* when its
grade `|Δh|/dist > maxGrade`, the cost is horizontal distance (plus a `turnPenalty`
that straightens the line and an optional `climbCost` that biases onto flatter
ground), and the heuristic is straight-line horizontal distance (admissible). To
gain height on steep ground the path must traverse along the slope and double back —
switchbacks fall out of the constraint, not a special case. The staircase is
collapsed to its corners (Douglas–Peucker) so the result is a handful of segments,
seated on the ground (`y = field(x,z)`). No grade-legal route in the search box →
empty, and the caller falls back to a straight road (never worse than before).

Exposed to Lua as `terrain.route(field, { from, to, max_grade, cell, width,
turn_penalty, climb_cost, ... })`, which returns a **chain layout**
(`{nodes, edges, blocks={}}`) — the same shape `city.layout` returns — so it drops
straight into the existing `terrain.conform` + `city.road_mesh` pipeline with no new
plumbing. `twin_cities.lua`'s highway now routes A→B at `max_grade = 0.12`: the
many short, shallow segments mean the conform only smooths a road that already
follows gentle ground, so the cut/fill is small and the carriageway stops poking
through. Because the routed nodes are dense, the verge-forest loop was generalised
to walk an arbitrary polyline.

**Why A\* over the tensor/agent grower ADR-0044 sketched.** The grower is the right
tool for *generating a whole network* that answers terrain; point-to-point routing
of a single arterial/highway is a shortest-path problem, and A\* with a hard grade
gate is the smallest, most predictable thing that makes "carve around / switchback,
never exceed the grade" true and unit-testable. It also composes: a recipe can route
any two points, and the primitive is available when city generators become
terrain-aware. Covered by `tests/test_terrain_field.cpp` (straight on flat ground,
grade held + bent around a steep cone, empty when walled in).

**Deferred.** Intra-city *arterials* that bound blocks aren't rerouted here: the
faces are extracted from the 2D graph before conform, so bending a block-boundary
edge would desync the lots. The cities sit on flat discs (`terrain.flat_sites`), so
their internal grades are bounded *by placement*; making the C++ road generators
themselves terrain-aware (so blocks follow the routed arterials) stays the ADR-0044
grower work. Literal hairpin switchbacks on a uniform steep plane are limited by the
8-neighbour grid's 45° minimum step angle — fine for real (varied) terrain, where
the router winds through gentler ground; a 16-neighbour grid would sharpen them.

---

## ADR-0046 — Terrain-aware city generation (the network answers the slope, blocks follow)
**Status:** Accepted · **Date:** 2026-06-22

**Context.** ADR-0045 routes a *single* road (the highway) at a walkable grade, but
`city.layout` still generated whole networks in pure 2D and conformed the terrain to
them afterward. On a hilly site that means streets run straight up the fall line and
the ground is cut/filled to match — the same "road on the mountain top" symptom,
multiplied across a network. The reason it couldn't just reuse the ADR-0045 router:
a city's *arterials bound its blocks*, and the blocks (→ lots → buildings → pads)
are extracted from the road graph; bending a block-boundary edge after extraction
would desync the lots from the road.

**The lever.** `city.layout` runs **generator → `planarize` → `extractBlocks`**, in
that order. So *any* change to the road graph made **before** extraction flows into
the blocks for free — no lot/parcel rework. That is what makes terrain-aware
generation tractable as an edit rather than a rewrite: do it upstream of the faces.

**Decision.** Two mechanisms, both upstream of `extractBlocks`, exposed by passing an
optional `terrain` HeightField (plus `max_grade`, `slope_align`) to `city.layout`:

1. **Contour coupling in the tensor field** (the flagship; after Chen et al. §6).
   `fieldAt` blends the tensor toward the **contour** orientation (perpendicular to
   the terrain gradient) where the ground is steep, weighted by `slopeAlign` and
   saturating near `maxGrade`, and scaled by the base field magnitude so it dominates
   on steep ground yet vanishes on flats. The traced avenues then bend to follow the
   hillside instead of marching across it. (The *perpendicular* family then tends
   toward the fall line — which is exactly what mechanism 2 removes.)

2. **Connectivity-preserving pruning, shared by every generator.** `pruneSteepEdges`
   runs on the *planar* graph (each short segment judged on its own grade) and drops
   streets steeper than `maxGrade` so the steep flanks fall into the larger natural-
   hillside blocks. The first cut dropped edges purely by grade with a no-orphan
   guard — and **shattered the network into hundreds of fragments**, because a steep
   edge is often the *only* bridge between two parts (a degree guard stops degree-0
   nodes but says nothing about connectivity). The fix is a max-spanning-forest
   completion: keep every gentle edge and Arterial (they define the base
   connectivity), then add back only the **gentlest** steep edges needed to reconnect
   whatever that leaves split. A steep street survives exactly where there's no
   gentler way around; a redundant one (gentle detour exists) is dropped. The result
   has the *same* connected components as the input.

3. **A stitch backstop — one connected network, always.** Strong contour coupling can
   leave streamlines that never cross (parallel on a steep flank), so the *raw* graph
   is already fragmented before pruning, and a component-preserving prune faithfully
   keeps those gaps. `connectComponents` closes them: while more than one component
   remains, add the single shortest connector between the closest nodes of different
   components (then re-`planarize` so connectors split at anything they cross). So
   regardless of where a split came from, `city.layout` returns one coherent network.

On the `hill_roads` study scene (a road network on smooth rolling hills, no city) the
three take the tensor layout from ~40% walkable streets and ~0.19 mean grade
(terrain-blind) to ~68% and ~0.13 — while staying a **single connected component**
(the grade-blind prune alone left ~400). Covered by `tests/test_city.cpp` (prune
keeps the network connected / keeps arterials + never orphans; `connectComponents`
heals a split graph into one; the tensor field's streets run gentler with coupling
than without). Contour coupling reads cleanest on *smooth* relief — sharp ridged
terrain gives wiggly contours the streamlines chase into a tangle — so the study
scene uses a low-octave fbm massif.

The study scene generates the network *in place* (the terrain and the layout share
an origin), which is the precondition for correctness: the generator must sample the
terrain at the street's true world position. A recipe like `twin_cities.lua` that
builds a city at the origin and *translates* it to its site can't just pass its world
heightfield — it would read the slope at the wrong place. So terrain-awareness is
wired into the standalone road, not the translated cities (which sit on flat discs
anyway, where it's a no-op); giving `city.layout` a `center` so a city generates at
its world location is the small follow-up that lets a *placed* city use it.

**Why pruning over rerouting the steep streets.** Routing each steep local street
(ADR-0045) into a switchback would cross its neighbours in a dense block and is far
more code; on a hillside a city simply *doesn't build* the unwalkable street — the
block grows and stays green. Pruning is the smaller, more faithful move, and the
ADR-0045 router stays the tool for the long point-to-point arterials/highways.

**Deferred.** Grid/radial generators get terrain-awareness only via prune + stitch
(their *pattern* isn't bent — only the tensor field follows contours); a steep grid
thins and gets stitched rather than curving. The stitch connectors are picked by
straight-line distance, not grade, so a healed bridge can itself be steep (it's the
unavoidable access, but routing it via ADR-0045 would be gentler); and node-
relaxation toward gentler ground is still owed. The contour coupling is also only as
clean as the terrain is smooth — high-frequency relief gives wiggly contours the
streamlines chase; a multi-scale gradient would steady it on rough ground.

---

## ADR-0047 — Earthwork-weighted routing: one dial from "hug the terrain" to "cut through it"
**Status:** Accepted · **Date:** 2026-06-22

**Context.** The system had the two *ends* of the road-vs-terrain spectrum as separate,
unpriced tools: `routeRoad` (ADR-0045) only **hugs** — it drapes on the ground and a
hard grade gate forbids steep steps, so it returns *empty* when no walkable hug
exists (e.g. a tall peak with no gentle way round); `terrain.conform` (ADR-0044) only
**modifies** — it cuts/fills the ground to whatever road it's handed. Real roads live
on the continuum between, and *which* point you want is an economic choice: a mountain
trail with cheap labour and dear earthmoving switchbacks (hugs); a motorway with cheap
earthmoving and a need for speed cuts straight through (modifies). Nothing in the
system let you express that choice, or even produce a road across ground too steep to
hug. This is exactly the problem the highway-alignment-optimization literature (civil
engineering) and Galin et al. 2010 ("Procedural Generation of Roads", graphics) solve:
a single weighted shortest path whose cost prices length, grade, curvature, **and**
earthwork together.

**Decision.** Give `routeRoad` an optional **earthwork mode** (a `cutFill` weight) that
turns the hug/modify choice into one continuous dial. The change is in the search
*state*: instead of routing over `(x, z)` with the road glued to the ground, route
over `(x, z, road-elevation)` — the road carries its own profile. Then:
- the grade limit becomes a **hard constraint on the ROAD profile** (each step may
  change road height by at most `maxGrade·dist`), so the result is *always* walkable —
  even across ground too steep to hug, which the hug router can't do at all;
- cut/fill, `|road − terrain|`, is a **soft cost** weighted by `cutFill`, added to the
  per-step length (and turn penalty);
- `cutFill` is the dial. **High** → deviating is dear → the road hugs and switchbacks
  to keep earthwork near zero. **Low** → reshaping is cheap → the road runs straight
  and flat, notching hilltops and filling hollows. `cutFill < 0` (default) keeps the
  exact ADR-0045 hug behaviour, so existing callers are untouched.

The routed profile flows through to the world: `terrain.route` emits each node's road
height `y`, and `terrain.conform` now grades the ground *to* a node's `y` when present
(instead of re-sampling the field), so the cut notch and fill embankment are real
geometry. One correctness subtlety: the hug path's XZ Douglas-Peucker simplifier would
merge across a vertical bend and fake an illegal grade, so earthwork mode uses a
*lossless* 3D-collinear collapse that preserves the grade-limited profile exactly
(verified: road grade stays ≤ `maxGrade` across the whole sweep).

On a road crossing a ~0.9-grade peak (`road_earthwork` demo, straight-line 440 m), the
dial sweeps cleanly and every road stays at 0.12 grade:

| `cutFill` | length | earthwork (proxy) | max cut/fill |
|---|---|---|---|
| 3.0 (hug) | ×2.30 | 564 | 3 m |
| 0.5 | ×1.76 | 949 | 5 m |
| 0.1 | ×1.46 | 1868 | 11 m |
| 0.02 (cut) | ×1.01 | 8226 | 47 m |

Covered by `tests/test_terrain_field.cpp` (over a cone peak, high `cutFill` detours
around the base moving far less earth; low `cutFill` cuts straight through; both keep
the profile walkable).

**Deferred.** Two real simplifications. (1) The cut/fill cost is a flat `|road −
terrain|·length` proxy for cross-sectional volume — it doesn't price *haul* (moving
the cut spoil to a fill), which the civil mass-haul diagram balances, nor distinguish a
cheap fill from an expensive deep cut. (2) Horizontal and vertical alignment are
solved *jointly* on one grid; the mature decomposition routes the centerline first,
then fits a vertical profile (piecewise grades + parabolic crest/sag curves) by DP for
sight-distance and balanced earthwork. Both are natural next increments; the current
form already makes the hug↔cut decision a single, testable knob.

---

## ADR-0048 — One road model: a curve (spline) as the source, the graph as a derived view
**Status:** Proposed — captures the target architecture; a smoothing prototype landed. · **Date:** 2026-06-22

**Context.** Roads are carried by *three* representations that don't share a type:
radial rings are true circular **arcs** (`RoadArc`), tensor streets are **RK2
polylines**, and routed roads (ADR-0045/47) are **8-connected grid A\* polylines**.
All three are immediately flattened into the `RoadGraph` of straight segments, and the
mesh (`road_mesh`) draws a straight ribbon per edge — so any curve information is lost
at sampling, smoothness is faked by segment density, and there is no curvature
continuity. The grid router is the worst: it can only travel in 45° steps, so its
switchbacks are literal staircases — the "unnatural curves" symptom. Three reps also
means three places to fix anything (smoothing, LOD, min-radius).

The observation that drives this ADR: a *radial pattern is splines, a grid is splines,
they'd just be straight splines* — so there should be **one road type, a curve**, and a
pattern is just *which curves* (a ring is a closed curve; a grid is straight curves;
others intersect them). That's correct as the geometry/authoring model — it is exactly
how OpenDRIVE and CityEngine model roads.

**Decision (proposed).** Unify on a **`RoadCurve`** (a spline — polyline / arc /
clothoid pieces, open or closed, with width + class) as the **source of truth**, and
keep the planar graph as a **derived topology view**, not a parallel representation:
- **Generators emit `RoadCurve`s**, not edges. Radial = N closed curves (rings) + M
  straight curves (spokes); grid = two straight-curve families; tensor = streamline
  curves; routed = the A\* path as a curve. The pattern is the *set of curves*.
- **The graph is generated by sampling + planarize**, but each sampled node/edge keeps
  a **back-reference to its parent curve + parameter `t`**. Topology (blocks,
  connectivity, prune, stitch) stays on the robust segment graph; the *mesh and
  measurements read the true curve*, not the chord. Smoothness becomes intrinsic for
  every pattern at once.
- **Intersections stay on the sampled polylines** (a 2-line solve, robust) rather
  than analytic curve–curve roots (a 2-line solve, robust). "A circle and a spoke
  cross → split both → the wedge blocks fall out" via the existing planarize/
  `extractBlocks`. Analytic
  curved *faces* are deferred (see below).
- **Curve type:** centripetal Catmull-Rom for organic/routed roads (no overshoot on
  irregular spacing), arcs for rings, clothoids later for G2 "road feel"; with a
  **minimum radius by road class** (arterial vs. trail — the AASHTO design-speed link
  from ADR-0044/0047) clamped during the fit.

This is an *evolution* of what exists (`RoadArc`→`sampleArc` already does
curve→sampled-edges with chord error); planarize/blocks/mesh change only at their seams.

**Prototype landed (this slice).** `smoothCentripetalCatmullRom(centerline, closed,
chordTol, decimateTol)` (in `terrain_field`): decimate the polyline to corners (so a
spline doesn't just wiggle down the staircase), fit a **centripetal** Catmull-Rom
through them, and re-sample by arc length. Wired as `terrain.route{ smooth=true }`
(carries the road's `y`, so a cut/fill profile survives). A/B on the ADR-0047
switchback route: the sharpest turn between consecutive segments drops from **90° to
~8°** — the hairpins read as real mountain-road curves instead of a staircase
(`road_earthwork.lua` with `smooth`). Covered by `tests/test_terrain_field.cpp`
(a staircase's 90° corners round below 40% of the raw turn; endpoints stay exact).

One subtlety the prototype surfaced: centripetal Catmull-Rom loads its curvature *at
the control points* (span ends), so a midpoint-flatness sampler misses the bend
entirely — arc-length stepping is what actually samples the corner. The general
`RoadCurve` sampler should keep that lesson.

**Minimum radius / the offset-curve problem (this slice).** A road ribbon is the
centerline *offset* by its half-width (+ sidewalk); an offset curve **self-intersects
on the inside of any bend tighter than the offset distance** (`κ·d = 1` is the cusp;
`r_inner = r_center − d` must stay positive). So a Catmull-Rom hairpin whose radius
falls below the half-width *folds the ribbon over itself* — the overlapping geometry
seen at the apexes. Catmull-Rom can't bound its radius, so the fix is the road-design
primitive: `roundRoadCorners(centerline, minRadius, …)` replaces each corner with a
**tangent–arc–tangent fillet** at a *guaranteed* `minRadius`, capped to half the
shorter adjacent leg so neighbouring fillets can't overlap. Set `minRadius ≥
halfWidth + sidewalk + margin` (per road class) and the offset can't fold. Wired as
`terrain.route{ smooth=true, min_radius=R }`; A/B on the switchback (min_radius 14 vs.
half-width 7 + sidewalk 2.2) shows the ribbon holding uniform width through the bends
instead of pinching. Test: a 90° corner filleted to R=20 leaves no point sharper than
that radius. Where legs are too short for `minRadius` (a true 180° hairpin) the cap
bites — so `roundRoadCorners` leaves that apex a **sharp vertex** instead of a
too-tight arc, and `road_mesh` builds a **turning head** there: a degree-2 bend whose
deflection exceeds `hairpinDeflection` pulls both legs back and fills a **disc**
centred on the apex. (First attempt reused the ≥3-way junction pad — but that fan-
triangulates a ring of arms *assumed to spread around the node*, and a hairpin's two
near-parallel arms make the fan wrap a ~340° triangle across the back and fold over
itself; a disc centred on the apex can't fold.) A/B on the switchback shows an
isolated hairpin become a clean turning head. Covered by `tests/test_city.cpp` (a
sharp degree-2 reversal yields a distinct mesh — ribbons set back, disc filled) and
`tests/test_terrain_field.cpp` (the radius guarantee).

Two things stay deferred. (1) A *stack* of switchbacks crammed within a few metres
(very steep, high-`cutFill` terrain) still tangles — overlapping pads — because that's
really the *router* producing too-dense reversals; the fix is router-side (a
switchback-spacing / turn penalty), not the mesh. (2) The fully-general "allow tight
curves then offset robustly and *trim* the self-intersection" path (medial-axis /
straight-skeleton / polyline-buffer joins — the SVG/CNC stroking problem) stays behind
the simpler "constrain the radius, pad what's left" approach.

**Literature.** The two bullseyes: **ASAM OpenDRIVE** — a road is a reference line of
*line / arc / spiral (clothoid) / cubic* pieces with lanes offset from it (the exact
"one path type, made of spline pieces" model, shipped for driving sim); and **CGAL 2D
Arrangements** — the rigorous "set of curves → vertices/edges/faces by their
intersections" framework (segments, arcs, conics, Bézier with robust predicates), the
formal version of curved-face extraction if we ever go analytic. Lineage:
**Parish & Müller 2001** (city roads as a self-sensitive graph), **CityEngine** (graph
topology vs. edge *shapes*, separated — the dual we're adopting), **Chen et al. 2008**
(tensor streamlines), **Galin et al. 2010/2011** (roads as shortest-path curves +
networks), **McCrae & Singh 2009** ("Sketching Piecewise Clothoid Curves" — fitting
clothoids to a rough polyline, i.e. the natural upgrade from Catmull-Rom).

**Path stroking — back to basics (this slice).** Before unifying the generators, the
fundamental primitive had to be provably right: turning a centerline into a filled
ribbon for *any* curve at *any* width without folding. `strokeRibbon(centerline,
halfW, y, color, closed)` (in `road_mesh`) does it the 2D-graphics way — a trapezoid
per segment (variable half-width per point) plus a **round join** that fans the OUTER
wedge at each vertex (+ a one-triangle inner bevel, so the curvature-flip sliver and
the convex-vertex cusp can't appear). Roundness comes from centerline *density* — a
coarse 3-point hairpin strokes to facets, a finely-sampled arc strokes round — so
smoothing stays a separate centerline step. (A full disc per vertex — the exact
Minkowski sum — was tried and rejected: it scallops the edge and piles up coplanar
overdraw.) The inside of a bend has trapezoids overlap, but that's a
coplanar fill, not a fold; and because the join sweep equals the signed turn angle, a
**180° hairpin gets a semicircular turning cap for free** — the general stroke handles
a switchback by construction, no special case. A path that curves tighter than its
half-width simply fills its own centre (the offset self-intersection, benign when
flat). Exposed as `city.stroke{ points, width|widths, closed }`; a flat terrain-free
demo (`stroke_test.lua`) strokes a circle, S-curves, a hairpin, a variable-width
taper and a spiral. A test suite pins the invariants (`tests/test_city.cpp`): straight
line is an exact rectangle; variable width is the exact trapezoid; a closed circle is
an annulus with an uncovered centre; nothing strays outside the half-width on a
hairpin; every triangle faces up; the hairpin has a round cap; and a planarized
X-crossing builds a clean junction (centre filled, nothing past the arms).

**Non-overlapping network union (this slice).** `unionRibbons(spines, cell, …)` merges
a set of centerline spines into ONE non-overlapping surface — every crossing becomes a
shared junction, not stacked ribbons. Rather than an exact polygon arrangement
(Clipper/CGAL, dependency + degeneracy minefield), it's a **coverage union**, which
turns out to fit the problem exactly: the signed distance to a polyline *is* the
Minkowski sum (so round joins/caps are free), `min` over the spines is the union, and
the region {sdf < 0} is meshed by **marching-squares filled cells** on a `cell`-spaced
grid. Triangles are therefore **non-overlapping by construction** (grid cells are
disjoint), holes (a ring's centre, a roundabout) appear for free, self-crossings (a
figure-8) merge like any other crossing, and oblique/multi-way meets need no special
case. Exposed as `city.union{ spines, cell }`; `stroke_union.lua` renders the six
crossing cases merged. Tests (`tests/test_city.cpp`): two perpendicular strips union to
*less* area than the two unioned separately, by exactly the ~100 m² crossing overlap
(method-consistent, so it proves the merge); a ring keeps its hole. Cost: it's
grid-dense (≈40k triangles for the six-case panel) and crisp only to ~`cell` (corners
round at cell scale) — so the trade vs. an exact arrangement is *robustness + zero
degeneracy cases* now, against triangle count and sub-cell sharpness. Decimation (or a
later exact arrangement for hero assets) is the optimisation; the union itself is
provably correct.

The SDF makes the full **roadbed** fall out for free: `unionRoadbed` reads the same
distance field as *bands* — carriageway `{sdf<0}`, raised sidewalk `{0<=sdf<width}`,
the curb step between (a vertical face up the `sdf=0` contour) — so sidewalks wrap the
whole merged network with zero special-casing (junctions, the roundabout's centre
hole, all just work). Each cell is clipped to its bands (Sutherland-Hodgman on the
scalar field) and every vertex is seated on a terrain `heightAt`, so the dense union
grid that looked wasteful when flat is exactly what lets it **drape smoothly over 3D
ground**. Per-band vertex colours carry a grain, and the mesh takes a procedural
asphalt/concrete material (`add_solid(mesh, material)`, world-planar) on top. Exposed
as `city.roadbed{ spines, cell, sidewalk, curb, height }`; `roadbed_demo.lua` drapes a
small network on hills. And it's wired to the actual generators: a `RoadGraph`
overload (`graphToSpines` → one spine per edge, half-width from the edge width) and a
Lua `layout=` argument let `city.layout`/`gridRoads`/`radialRoads`/`tensorRoads`
output feed straight in — so the engine converts its generated splines to a merged,
draped roadbed in one call (`roadbed_network.lua` roadbeds a terrain-aware tensor
network on its own terrain). Test: a straight road grows a sidewalk band (covered just
outside the carriageway, nothing past it) and its vertices follow the terrain ramp.

**Lane markings (landed).** The union has no centerline parameter, so markings are a
*thin marking stroke composited on top*: `traceChains` walks the graph into its maximal
degree-2 polylines between junctions (the unit a stripe follows), each chain is trimmed
back from its junctions (`trimEnds` — markings break at intersections), stroked at the
paint width via the same `strokeRibbon` primitive, optionally chopped into a dash/gap
pattern, and every vertex draped a hair over the asphalt on the road `heightAt`. Exposed
as `city.lane_markings{ layout, height, mark_width, trim, dash, gap, lift }`. Wired into
two shipping cities: the **radial** city (Lua `city.lua` `roadbed` path — a draped
roadbed + sidewalks + a yellow centreline down every spoke and ring, merged cleanly
through the central plaza) and the **city-arena** (C++ `generateCity`, ADR-0027/0038 §6).
The arena's old per-edge `emitFlatRoad`/`emitLaneLine` — which stacked and z-fought where
streets crossed — is replaced by `unionRoadbed(graph)` (carriageway only; the block
aprons stay the sidewalks) plus `laneMarkings(graph)` for the yellow centreline, both
draped on a smooth *street-grade* field (project a query point onto its nearest edge,
lerp the Laplacian-smoothed node grades) so the merged surface sits level with the aprons
at the curb. The crosswalks/stop bars now ride the slab top, and the terrain is still cut
to grade under each road. So the same SDF/stroke pipeline now draws every road in both
city styles.

**Density + crosswalks (revision).** The uniform SDF roadbed spends polygons *everywhere*
(a marching-squares grid at a fixed cell), and the separately-composited `laneMarkings`
trimmed by a fixed arc length still let the stripe creep into a tight junction. The fix is
to make the **junction-aware** `buildRoadMesh` the city mesher instead: it already trims
each ribbon back to the curb corner, fills only the intersection with one pad, and draws
the markings on the *trimmed* span — so the markings stop at the junction *structurally*,
and the polygons concentrate where two curves meet. Two refinements close the loop: (1)
the ribbon (and its sidewalk) now subdivides along its length **only where the terrain
sags from the straight chord** by more than `conformTol` — flat ground stays a single
stable quad, so polygons appear where the surface bends, not on flat runs; (2) a **zebra
crosswalk** is laid across each junction arm at the trim point (the intersection mouth),
since the mesher knows exactly where every intersection is. The cities switch off
`unionRoadbed`/`laneMarkings` onto this: the arena (`buildRoadMesh` + crosswalks, draped
on the street grade, stop bars draped so they don't bury) and the radial (`city.lua`
`roadbed` → the dressed `road_mesh`). Every other `city.road_mesh` scene (twin_cities,
lua_city_terrain, roads_terrain, hill_roads, road_earthwork) inherits the adaptive
tessellation for free; `unionRoadbed` stays for the standalone `roadbed_demo` /
`roadbed_network` SDF studies. So: fewer, more stable triangles, markings that stop at the
intersection, and crosswalks back where the streets cross.

**Deferred / risks.** (1) The graph back-reference (`curve, t` per node) isn't built
yet — the prototype smooths the *output* polyline, it doesn't yet make the graph a view
of curves; that's the core of the migration. (2) Smoothing moves the centerline OFF the
routed grid path, so a tight switchback can creep back over `maxGrade` or clip terrain
it was routed around — the fit must stay in a corridor of the path or re-check grade
(the hug router's whole point). (3) Analytic curved block faces (CGAL-style) stay
deferred — sampling is visually identical for a fraction of the cost/risk (ADR-0044).
(4) Catmull-Rom only approximates a circle; rings should stay true arcs in `RoadCurve`,
not CR through sampled points.

---

## ADR-0049 — The editable road: a RoadNet entity the editor can widen and drag

**Context.** The roads above are *generated* (procedural city, Lua recipes). The next
step is *authoring*: select a road in the editor, widen it, drag its curve nodes, and
see it regenerate live. The two systems that make that hard already exist — a fast pure
graph→mesh function (`buildRoadMesh`, no SDF grid after ADR-0048, so it rebuilds in
milliseconds) and a live-regen-on-edit hook (`ComponentRegistry::onEdited`, used today
to rebuild a primitive's mesh when its Size changes). So this is mostly *promotion +
wiring*, not new modelling.

**Decision.** A `RoadNet` — control **nodes + edges + look** (width, sidewalk, curb,
corner radius, markings) — is the editable counterpart to `city.road_mesh`, promoted to
a first-class entity. `buildRoadNetMesh` feeds its graph to `buildRoadMesh`; the edit
ops the editor drives are `roadNetSetWidth` (the inspector widen) and `roadNetMoveNode`
(the viewport node drag), with JSON I/O for a `shape:"road"` level entity. It loads in
both the viewer (`level_loader`) and the offline tracer (`level_scene`), draped on the
level terrain, carrying a `SourceSpec` recipe so it round-trips through save/reload.
Registered as the **"Road"** inspector component; `editor_system` wires `onEdited` to
regenerate the mesh and re-sync the saved recipe, so widening (or undo) is live. The
viewport gets `roadNodeHandles` (a world point per node) + `moveRoadNode` (drop a node,
regen) — the engine owns the data + regen; the shell owns only the handle draw + mouse
picking. `assets/levels/road_edit.json` is a sample editable road (a curved T).

**Why nodes-as-graph first, splines later.** This slice keeps roads as a polyline graph
(the thing `buildRoadMesh` already eats), so the full edit→regen→re-upload loop is
proven with zero new geometry risk. The genuine next piece — control points with **in/out
tangents** (a `RoadCurve` sampled to the graph, the "graph as a view of curves" of
ADR-0048) — layers on top: it only changes how the polyline is *produced*, not how it's
meshed or wired. Width is per-net for now; per-edge width is a small extension (the edges
already carry a `width`).

**Splines (landed).** `curved` makes each edge a **Hermite cubic** through its
endpoints' tangents instead of a straight chord: `tangents` is one through-direction
per node; a zero/missing tangent is auto — **Catmull-Rom** on a degree-2 through-road,
a straight chord into a junction/dead-end (so junctions stay sharp and the pad fills,
chains stay smooth). `roadNetSetTangent` (the tangent-handle drag) and `roadNetTangentAt`
(seed the handle from the stored override or the auto) drive it; the editor gets
`roadTangentHandles` + `moveRoadTangent` alongside the node handles. The sampler keeps the
original node indices (junction degree preserved) and appends curve samples, then hands
the fine graph to `buildRoadMesh` — so the continuous-chain stroking smooths the result
for free. `assets/levels/road_spline.json` is five **collinear** control nodes bent into a
smooth S purely by their tangents. Tests: a tangent bows the spline off the chord, auto
smooths a chain, and `curved`/`tangents` round-trip.

**Topology editing (landed).** The road is now authorable, not just adjustable:
`roadNetSplitEdge` (click a road to insert a point — the edge becomes two), `roadNetExtend`
(grow a node off an end), `roadNetAddNode`/`roadNetAddEdge`, and `roadNetDeleteNode` (drop a
node + its incident edges, reindexing the rest and keeping `tangents` parallel).
`roadNetNearestEdge` picks which segment a world point is over so the shell can route a
click to a split. The editor wraps these as `splitRoadEdge`/`extendRoad`/`deleteRoadNode`/
`nearestRoadEdge`, each regenerating + syncing the recipe. Tests cover split (a point
inserts, one edge → two), extend (grows + rejects a bad end), and delete (a junction drops
all three edges; a leaf drops one and the survivors keep their indices).

**Deferred / risks.** (1) Per-edge width (one global width for now; the edges already carry
a `width`, so it's a small step). (1b) A single tangent per node (the through-direction) is
oriented per edge by sign, so a deliberate cusp/reversal needs separate in/out handles — a
small extension when wanted.
(3) Live terrain *conform* on drag (currently drapes only; re-conforming is deferred to
drag-release). (4) The road `Transform` is assumed identity (nodes are world-space); a
moved Transform would offset the baked mesh but not the node handles — fine for v1, but a
whole-road move should go through the nodes or compose the Transform into the handles.

---

## ADR-0050 — A reusable curve-path toolkit (roads + animation), foundation first

**Context.** Editing a road's nodes/tangents in the viewport is one instance of a more
general need: *manipulating a control-point path in 3D*. The same widget should drive a
**road network** AND an **animation path** — a curve painted in the world, associated with
an object, that "plays" by driving the object along it — and let you move tangents in
full 3D or lock to a plane. So rather than a road-only gizmo, build an independent toolkit
that hooks onto various features.

**Decision — four decoupled layers, foundation (the testable ones) first.**
1. **`EditableCurve` (core, headless).** A 3D **cubic-Bezier** path. Knots carry handle
   *points* (the chosen representation — Bezier control points stored as offsets so they
   ride along when a knot moves) with a per-knot mode: **Auto** (handles derived from
   neighbours via Catmull-Rom, resolved on the fly — no bake), **Aligned** (in/out
   collinear; dragging one mirrors the other), **Broken** (independent corner). Provides
   eval, **arc-length reparameterization** (constant-speed traversal), sampling, and edits.
2. **`curve_edit` manipulator math (headless).** `nearestHandle` (pick the dot under a
   ray) and `projectDrag` (land a dragged handle on a constraint plane through its current
   position): **Ground/XZ, XY, YZ, Screen**. Pure geometry — reusable by any handle gizmo.
3. **Bindings** (`HandleSource`-style adapter, to come with the tool). Roads stay a
   branching `RoadNet` graph and route edits to the ops already built; animation paths use
   `EditableCurve` directly. Lets the *same* tool drive different data.
4. **`AnimationPath` + `animationPose` (data + pure evaluator).** Curve + duration + loop
   (Once/Loop/PingPong) + face-along-tangent. The evaluator walks the curve at constant
   speed and orients +Z along travel — the "play / isolated-sim" side; editing (when not
   simulating) uses the same handles.

Layers 1, 2, and 4's evaluator are pure and unit-tested here (curve eval/arc-length,
handle pick + plane-drag, constant-speed + facing). **Why Bezier handle points:** they're
what an editor drags directly and match DCC tools; converted to the Hermite the
evaluation needs internally.

**Landed since.** The `HandleSource` binding (Layer 2) with `CurveHandleSource` /
`RoadHandleSource` adapters, and the `PathEditTool` state machine (Layer 3) — both
headless and unit-tested (the pick → plane-drag → apply → regen loop runs against a curve
or a road with synthetic rays). `EditorSystem` now drives a selected road's handles
through the tool: `updatePathEdit` runs the click/drag each frame (handles get first
refusal on the click, so a drag never re-selects), `drawPathHandles` renders the
centreline + node/tangent dots. The drag-loop logic is verified in engine_core; only the
ImGui draw + live mouse are exercised in the editor.

The tool also tracks a **hovered handle** (white ring on the dot under the cursor) and an
**active node** (set on grab, kept after release) for emphasis — both unit-tested. And
the ADR-0049 topology ops are wired to viewport gestures: **Shift+click a node** deletes
it, **Ctrl+click the road** subdivides that edge, **Ctrl+click the ground** extends a new
connected segment from the active node (intersections form by extending into existing
geometry). A cheat-sheet overlay lists the gestures.

**Deferred (the shell-coupled parts).** (1) `AnimationPath` as an ECS component + a
play/scrub system that writes the target `Transform` during sim (animation playback —
roads are the current focus). (2) Plane-constraint hotkeys (the tool's `setPlaneOverride`
is in place and tested; binding G/X/Y/Z to it is editor input plumbing). (3) Road
sub-object undo bracketing through `onGrab`/`onEdit` — topology edits (delete/split/
extend) and node drags are not yet on the command log.

---

## ADR-0051 — The road graph goes 2.5D: per-node elevation/layer, per-edge class + carriageway mode, level-aware crossings

**Context.** The road graph (`RoadGraph`, and the editable `RoadNet`) is a planar 2D graph
draped on terrain: every place two edges cross in XY is forced to be an at-grade
intersection. Grade separation — an overpass, an underpass, a freeway threading under an
arterial, a bridge over water — is not merely unbuilt; it is *unrepresentable*. It is the
single missing primitive behind freeways, ramps, interchanges, and water crossings.

**Decision.** Promote the graph to **2.5D / layered**:
1. **Per-node elevation + layer tag.** Two edges crossing in XY share a junction node *only*
   if their vertical profiles meet within clearance there; otherwise they are grade-separated
   and keep distinct nodes (no shared vertex).
2. **Per-edge vertical profile, first-class.** Generalize `roadProfile` from "drape on
   terrain" to a profile that may rise onto a deck or dip into a cut, bounded by class
   max-grade and a minimum vertical clearance (~5 m) at any separation.
3. **Per-edge class + carriageway mode.** Extend `RoadClass` with `Freeway` and `Ramp`.
   Carriageway mode ∈ {single ribbon, dual carriageway (two ribbons + median), ramp taper}.
   **Class is the master knob** — it sets min radius, max grade, lane count, and *access
   policy* (who may cross at-grade vs. who must be grade-separated).

**Why.** The category error behind the spoke bug and the freeway gap is the same: roads were
treated as a flat point-graph. Adding a layer dimension and a class-driven access policy is
what lets one machine express surface intersections *and* grade separations. See
`docs/road-constraint-plan.md` Phase 0.

**Landed since.** `RoadEdge.layer` (0 = ground) and `planarizeLayered` — a crossing splits
into a shared node only when the two edges share a layer; a cross-layer crossing stays
intact as an overpass — plus `gradeSeparationCount`. Identical to `planarize` when every
edge is on layer 0. Per-node elevation / carriageway mode aren't in yet; `clearanceProfile`
(ADR-0054) supplies the deck height a layered edge rides at. Tests: `test_road_layers`.

---

## ADR-0052 — Generate-and-constrain: a single named local-constraints pass + rule registry

**Context.** Legality fixups are scattered — `planarize`, `connectComponents`,
`pruneSteepEdges`, the `fairHermite` fold cap, the junction trim clamp — each a one-off doing
a piece of "make this network legal." There is no shared, reusable notion of *the rules a road
network must obey*, so the editable road obeys almost none of them and a hand-built hub
self-destructs.

**Decision.** Adopt the Parish & Müller (CityEngine, SIGGRAPH 2001) **generate-and-constrain**
split and make the constraint half a first-class, reusable phase:
`applyConstraints(RoadGraph, RuleSet) -> RoadGraph`. The *global-goals* half already exists
(the tensor field, `tensorRoads`). The pass runs a **rule registry incrementally and locally**
— each rule adjusts/snaps/rejects a candidate against its immediate neighbourhood, never via a
global solve. Both generators (after streamline tracing) and the **editor** (on a dragged
node's neighbourhood, so illegal configs snap or refuse live) call it.

Rule catalog by scope: edge (min radius / max grade by class, min segment, max deflection),
node (**min arm angle** → caps degree to `2π/θ_min`; max-degree → roundabout promotion;
no-acute-merge → tangential fuse), network (planarity with level test, connectivity / dead-end
policy, block-size feedback, class-access grammar).

**Why.** The natural global look *emerges* from local legality + the goal field; nothing ever
constrains all nodes at once (the part that does not scale). The **min-arm-angle rule** alone
makes the over-packed hub unrepresentable, fixing the spoke bug at the source rather than in
the mesher. Build it first as the proof of the approach. See plan Phase 1.

**Landed since.** `applyConstraints(RoadGraph, RoadRules)` with the first two rules —
min-arm-angle and max-degree, both resolved by **roundabout promotion** (the super-node is
replaced by a ring of attach nodes joined by sampled arcs, so every survivor is degree <=
3) — and `nodeNeedsRoundabout` for the classifier. Wired into `buildRoadNetMesh`, and shared
with terrain-conform via `constrainedNetGraph` so the ground grades to the ring, not the raw
spokes. A 3-way T and 4-way crossing stay flat patches. Tests: `test_road_constraints`. Still
to do: the no-acute-merge rule, and a live editor warn/preview.

**Landed since (the rules registry).** `road_rules.h` — `DesignRules`, the single class-keyed
table of road parameters (per `RoadClass`: min curve radius, max grade, lane width/count,
divided-median; network-wide: min arm angle, max arms before a roundabout, bridge clearance,
deck thickness, ramp grade) with `defaultDesign()`. `RoadClass` gained `Freeway` and `Ramp`.
`buildLayeredRoadNetMesh` now reads clearance/deck/ramp-grade from it instead of inline magic
numbers; the constraint pass's `RoadRules` mirrors the junction policy (to be folded in).
Tests: `test_road_layers` (classes rank sensibly — freeway curves/climbs gently, is divided;
ramp is one lane). This answers "where the rules live": one place, class-driven.

**Meshing a roundabout — the SDF roadbed, not the analytic pad.** The analytic per-junction
pad (`buildRoadMesh`) cannot weld a ring: the N degree-3 attach junctions each skirt their
sidewalk radially outward, so the skirts overlap (z-fighting flaps) and the ring reads as a
faceted blob. So a net that promotes a roundabout is meshed through the **SDF union roadbed**
(ADR-0048) instead — one distance field gives a continuous welded sidewalk, merges the
junctions, and opens the island as a hole for free; centreline markings are overlaid via
`laneMarkings` (the shader-UV markings need the analytic ribbon, so they're skipped on this
path). Simple roads with no roundabout stay on the crisp analytic path. The auto ring radius
is also floored at `widestArmHalfWidth * ringWidthFactor` so the ring reads as a drivable
annulus around a visible island, not just a merge hole.

---

## ADR-0053 — Every crossing resolves to a template chosen by (classes, level, degree, angle)

**Context.** A 4-way grid crossing, a many-armed roundabout, a freeway/arterial diamond, a
freeway/freeway cloverleaf, a freeway T trumpet, and a tangential merge are today either
unbuilt or special-cased in the mesher. They are, in fact, the *same kind of thing*: a rule
for what a crossing of two roads becomes.

**Decision.** A single **crossing resolver** classifies each crossing from
`(classA, classB, levelDiff, degree, angle)` and rewrites it via a **parametric template
macro** (an L-system production: crossing = non-terminal, template = production, classifier =
which fires). Members: junction patch (deg ≤ 4 at-grade), roundabout (many arms / large
required radius — one super-node → a ring of degree-3 nodes), diamond (freeway × surface),
trumpet (freeway T), cloverleaf / stack (freeway × freeway), merge/fork (sub-acute). Each
template emits nodes + edges + ramps + vertical profiles, then feeds the result **back through
the ADR-0052 pass** so its own ramps obey radius/grade/clearance.

**Why.** Unifies all intersection/interchange types under one classifier and one extension
point (add a template), instead of a growing pile of mesher special cases. The roundabout
member is also the principled fix for the spoke bug: it gives a busy node *extent*. Build
order: patch → roundabout → diamond → trumpet → cloverleaf/stack. See plan Phase 2.

**Landed since (first components).** The hand-authored grade-separated cloverleaf
(`net_interchange.lua`) + the **merge/diverge gore** primitive: `bridgeDeck`/`deckBarriers`
gained a per-point half-width overload, so a ramp can TAPER to a nose where it meets the
mainline (exposed as `city.deck{ widths = {...} }`). The loops now taper into the highways
instead of ending blunt on top of them — the connecting half of a real on/off ramp. Tests:
`test_road_layers` (deck tapers with per-point width). Still to do: the classifier that
detects a freeway crossing and emits the template automatically, edge-aligned gores (nose at
the mainline's right edge, not its centreline), and the diamond/trumpet/stack recipes.

---

## ADR-0054 — Vertical profiles, a clearance solver, and bridges/underpasses (water crossings reuse the same path)

**Context.** ADR-0051 gives edges a vertical profile and lets crossings be grade-separated,
but nothing yet *makes the levels clear each other* or *builds the structure* between road and
ground/water.

**Decision.** Extend `roadProfile` into a real vertical engine: (1) a **clearance solver**
that, at each grade-separated crossing, pushes one road's profile up (bridge) or down
(cut/underpass) until the two decks clear, within approach-grade limits; (2) a **bridge
structure emitter** (deck, abutments, piers to terrain; approaches via the existing
`roadConformRegions` cut/fill); (3) **underpass** as a profile dip + deep terrain cut with a
deck overhead. **A road × water crossing is the same template** — water is the lower deck, the
road bridges it — so waterways reuse this engine outright rather than adding a water-specific
path.

**Why.** Grade separation needs a vertical solver; bridges and underpasses are its two signs;
and folding water crossings into the same template is the first dividend of the
"features are fields + templates" extensibility thesis (ADR-0055). See plan Phase 3.

**Landed since.** `clearanceProfile(s, minHeight, maxGrade)` — the vertical solver: the
minimal road profile that dominates a per-sample minimum height (ground, or lower-deck +
clearance at a crossing) within the grade limit, as the slope-limited upper envelope of the
constraints (two linear sweeps). A bridge approach falls out: hug ground, ramp up at grade
to clear, ease down; ends ride high honestly when the approach is too short. Tests in
`test_road_layers`. Still to do: the deck/pier mesh emitter and the per-crossing wiring that
feeds the lower deck height + clearance in (and the water-as-lower-deck reuse).

**Landed since (overpass end to end).** `bridgeDeck` (a deck slab riding a per-point height
with a fascia edge), `RoadNet.edgeLayers` + JSON `edge_layers`, layer propagation through
`netGraph`, and `buildLayeredRoadNetMesh`: ground roads (layer 0) build flat; each higher-layer
chain is densified, its crossings with ground roads detected, its deck height solved by
`clearanceProfile` (ground + clearance + slab at each crossing, ramped at 8%), then meshed with
`bridgeDeck`. A net with any `layer > 0` routes here. Demo: `road_overpass.json`. Tests:
`test_road_layers` (edge-layer round-trip, deck lifts above grade, same-layer stays flat).
Piers/abutments and the water crossing remain open.

---

## ADR-0055 — Multi-scale world layout as a field stack; closed networks (no map-edge dead ends); features extend as fields/rules/templates

**Context.** Phases 0–3 make a road network legal and buildable but say nothing about *what to
build and where*: where metropolitan areas, towns, beach towns, and freeway arteries sit; when
a road is a city grid vs. an exit onto a highway to the next town; how blocks are decided and
whether they are dense/radial/mixed; and — since the map is finite — how to avoid highways that
dangle off the edge.

**Decision — a top-down stack of layers, each reading fields from above and writing
fields/seeds for below:**
- **A. Suitability/cost fields** from terrain + water (buildability, water-proximity,
  coastalness, attractors).
- **B. Settlement placement** — weighted Poisson-disk on suitability, each settlement carrying
  class (metropolis/city/town/village/beach), radius, and **character** (grid/radial/organic/
  coastal). This is the "important waypoints" decision.
- **C. Inter-settlement highway skeleton** — Delaunay-pruned / gravity connections (big pairs →
  `Freeway`), routed on the cost field (avoid water/steep or pay to bridge). **Closure enforced
  here:** min-degree ≥ 2, no node on the map boundary; lone settlements get a ring road; the
  boundary is a redirect field that arcs roads back inward. Highways loop and arc toward places,
  never end in space.
- **D. Intra-settlement streets** — seed the tensor field from the settlement's character; the
  Layer-C highway enters the boundary and degrades via the class grammar (highway → interchange/
  exit → arterial → collector → local grid). The settlement boundary (Layer-B radius) is *where a
  road becomes city streets vs. an exit*.
- **E. Blocks & parcels** — planar faces (`extractBlocks`) sized by a density field peaking at
  the centre (dense downtown, sparse suburb), feeding the block-size feedback rule.

**Extensibility invariant.** A new world system contributes **fields, rules, and/or crossing
templates — never a new pipeline stage.** Worked example (waterways): a barrier/cost field
(routing), a coastal attractor (beach-town placement), a bridge template (crossing, ADR-0054),
and a land-use mask (blocks) — four existing hooks, zero pipeline. The same shape covers rail (a
class + level-crossing template), parks (a land-use field), districts (a character field),
landmarks (placement seeds), and trade routes (attractor fields). See plan Phase 4 +
"Extensibility."

---

## ADR-0056 — One unified road system: spline graph → in-house polygon join engine → solid → texture markings

**Context.** Roads had grown into a disjointed pile of systems: an analytic
junction pad (sharp mitered corners, fragile), an SDF roadbed (welds by
rounding, no UV), and a bridge deck — each a separate mesher, none agreeing.
Highways *overlapped* the city rather than connecting; "clovers" were one-sided
floating planes; markings were extra geometry. The mandate: one system that is
light, conforms to terrain but irons out bumps, welds every join (curbs, curves,
grid intersections, roundabouts) by **one** means, has real volume, treats
markings as **textures**, supports multilane, and is sourced from a spline graph.

**Decision.** Build one pipeline (`docs/unified-road-plan.md`):

1. **Join engine** (`road_offset.{h,cpp}`) — in-house 2-D polygon offsetting:
   `ribbonOutline` strokes a centerline to a closed ribbon; `polygonUnion`
   booleans the ribbons into welded boundary loops (exterior CCW + holes CW);
   `bridgeHoles` (Eberly) punches block interiors; `ringRibbon` offsets a
   **closed** centerline to an annulus (roundabout = band with an open island).
   One corner-fillet pass (`roundPolygonCorners`) rounds every junction notch —
   replaces both the analytic miter and the SDF round.
2. **Volume** (`weldSolid`) — extrude the welded outline to a closed slab: deck,
   mirrored underside, vertical side walls (outer skirt + hole shafts). The deck
   rides a smoothed, grade-limited vertical profile (`roadProfile`): terrain is
   sampled per spine and ironed flat, so the road follows hills, not bumps.
3. **Markings as texture** — the `RoadMarkings` surface paints from road-local UV
   (u = lateral in [1,3], v = arc-length) at shade time: double-yellow centre,
   dashed white lane dividers (multilane), solid white edges. To stay crisp, the
   paint rides its own road-aligned **strip** per spine (the ear-clipped union
   deck is too skew to interpolate clean UV), floating just over a plain welded
   deck; strips over another road's corridor are skipped so junctions stay plain.
4. **Crossing resolver** (`road_crossings.{h,cpp}`) — splices every centerline
   crossing/T into a shared node so loose generator output + a highway become one
   connected graph; same-grade crossings weld, cross-grade are flagged for ramps.

Lua surface: `city.solid`, `city.resolve`, `material.new{ surface="roadmarkings" }`.
`assets/levels/showcase.json` is the single demo (welded multilane grid, open
blocks, roundabout, a highway connected to the grid by ramps, on smoothed
terrain). The analytic-pad / SDF-roadbed / bridge-deck prototype scenes
(net_grid, net_radial, net_city, net_interchange, roadbed_network, hill_roads,
net_weld, road_earthwork) were removed.

**Owed.** Full grade-separated ramp *synthesis* from the resolver's
grade-separated nodes (today ramps are authored spines); intersection markings
(crosswalks/stop bars) — junctions render plain; the welded deck still uses the
ear-clip triangulator (now with a force-progress fallback so many bridged holes
can't leave a gap) rather than a constrained one.

---

## ADR-0057 — Vulkan is the second render backend (Linux + Windows), behind the existing seam
**Status:** Pending · **Date:** 2026-06-27

**Context.** The engine has shipped one GPU backend (Metal, macOS) behind the
`Renderer` RHI seam (ADR-0001), whose revisit trigger explicitly names this
moment: "adding a second backend (Vulkan) — at which point validate the seam
holds." We want the realtime viewer to run on PC and Linux at graphical parity
with Metal. The `Renderer` interface is lean and backend-neutral (resource
upload + draw dispatch; projection/view math is engine-side), so a second
backend is an implementation of that interface, not an engine change. The
feature set to match is ordinary forward shading + screen-space post (CSM
shadows, SSAO, SSR, bloom, tonemap, lens effects, IBL from a baked cubemap,
instancing, CDLOD terrain morph) — no compute, tessellation, or ray-tracing
extensions today.

**Decision.** Add a Vulkan backend (`src/renderer/vulkan/vulkan_renderer.cpp`)
as a third `Renderer` implementation, selected by CMake on non-Apple platforms
in place of `NullRenderer`. **One backend covers both targets** — Vulkan runs on
Linux and Windows, the two platforms asked for. Specific choices:

- **Surface creation stays behind the seam.** The backend must not reach through
  GLFW (ADR-0001), but `glfwCreateWindowSurface` needs the `GLFWwindow*`. So
  `Window` gains a pimpl'd `createVulkanSurface(VkInstance) -> VkSurfaceKHR`
  (declared with forward-declared Vulkan handle types, no GLFW types leaked),
  keeping GLFW sealed in `window.cpp`. `GLFW_NO_API` is already hinted
  (`window.cpp:264`), so the window is created context-free and Vulkan-ready.
- **Shaders compile offline to SPIR-V.** Metal compiles MSL from source strings
  at runtime; Vulkan consumes SPIR-V. We port the six MSL files to GLSL and
  compile them with `glslc`/`glslangValidator` at build time via a CMake custom
  command, shipping `.spv` artifacts. This keeps the runtime **dependency-free**
  (no `libshaderc` link), consistent with the no-external-deps rule, at the cost
  of no shader hot-reload on this backend (acceptable; revisit if iteration hurts).
- **Convention fixes are localized to the backend**, not the engine: Vulkan's
  Y-flipped clip space and [0,1] depth range are absorbed in the projection
  upload / viewport setup so engine math (`Mat4::perspective`, `lookAt`) is
  unchanged and shared with Metal and the offline tracer.

**Alternatives considered.**
- **WebGPU (Dawn / wgpu-native).** One backend for macOS+Linux+Windows (could
  eventually retire Metal) with nicer WGSL shaders — rejected for now: pulls in
  a heavy native dependency (Dawn's GN/C++ build; wgpu is Rust), a hard clash
  with the standard-library-only rule. Revisit only if dual-backend maintenance
  becomes the dominant cost.
- **bgfx / sokol_gfx.** Mature cross-platform abstractions — rejected: each
  imposes its own shader pipeline and would mean rewriting the RHI *to their
  API* instead of behind our own seam, plus the same dependency objection.
- **OpenGL.** Least code, most direct MSL→GLSL port — rejected: deprecated on
  macOS (we have Metal there), a dead-end API, and it forecloses the compute /
  `VK_KHR_ray_tracing` path that a raytracing project will plausibly want.

**Consequences / tech debt.**
- Validates ADR-0001's seam with a real second backend (its revisit trigger).
- Largest new cost is Vulkan boilerplate: the backend will be materially larger
  than `metal_renderer.mm` (~2000 lines) — instance/device/swapchain, descriptor
  sets/layouts, explicit render passes, pipelines, memory allocation, and manual
  barriers/sync are all explicit. The render *logic* is the same forward+post
  pipeline.
- New build step (GLSL→SPIR-V) and a parallel shader tree to keep in lockstep
  with `shaders/metal/*` until/unless the two are unified; a divergence risk to
  watch. The shared `shaders/metal/shader_types.h` GPU-struct header can be
  reused across both to keep CPU/GPU layouts in sync.
- No GPU in CI: like Metal today, the backend is validated by hand on Linux/
  Windows hardware with the Vulkan validation layers; unit tests stay CPU-only.
- Built incrementally (clear screen → lit mesh → full forward → shadows → post
  stack → instancing/terrain), each stage independently verifiable, so the work
  lands in reviewable slices rather than one drop.

**Revisit trigger.** Dual-backend (Metal + Vulkan) shader/maintenance cost
outgrowing the no-deps benefit (reconsider WebGPU and retiring Metal); needing a
mobile/console/web target; or adopting hardware ray tracing or a compute-driven
pipeline (Vulkan compute / `VK_KHR_ray_tracing`), at which point the SPIR-V
toolchain and descriptor model here are the foundation.

---

## ADR-0058 — WebGPU is the web render backend (Emscripten/WASM), behind the existing seam
**Status:** Pending · **Date:** 2026-06-29

**Context.** ADR-0057's revisit trigger named "needing a … web target" as the
moment to reconsider WebGPU. We want the realtime viewer to run in a browser.
The same property that made Vulkan a backend-not-an-engine change holds here: the
`Renderer` RHI seam (ADR-0001) is backend-neutral, and the frame loop is already
decomposed into `begin()`/`runFrame()` (application.h, the editor host path), so
the browser's requestAnimationFrame-driven loop is a drop-in for the blocking
`while (running())` loop. ADR-0057 rejected WebGPU only as a *desktop* backend,
because Dawn/wgpu are heavy native dependencies that clash with the
standard-library-only rule. On the **web** that objection disappears: the
toolchain (Emscripten) *is* the build target and ships WebGPU bindings — there is
no third-party native library to vendor.

**Decision.** Add a WebGPU backend (`src/renderer/webgpu/webgpu_renderer.cpp`) as
the web implementation of `Renderer`, selected by CMake under the Emscripten
toolchain (`if(EMSCRIPTEN)`) and providing `Renderer::create()`. Specific
choices:

- **Reuse the GLFW window seam via Emscripten's GLFW3 shim** (`-sUSE_GLFW=3`), so
  `window.cpp` is reused unchanged (it already hints `GLFW_NO_API`, so no GL
  context is created and the canvas is WebGPU-ready). No web-specific `Window`.
- **The surface is the canvas, not the native handle.** `nativeWindowHandle()` is
  null on the web (window.cpp's non-Apple path); the backend creates its surface
  from the `#canvas` selector (`WGPUSurfaceDescriptorFromCanvasHTMLSelector`), so
  no new seam method is needed (contrast Vulkan's `createVulkanSurface`).
- **WebGPU via the emdawnwebgpu port, device acquired with ASYNCIFY.** Emscripten
  6.x removed the legacy `-sUSE_WEBGPU` binding (and `emscripten_webgpu_get_device`)
  in favour of the **emdawnwebgpu** port (Dawn's standardized `<webgpu/webgpu.h>`,
  `--use-port=emdawnwebgpu`). Adapter/device acquisition is async-only there, but
  `Renderer::initialize()` is synchronous, so the backend awaits
  `RequestAdapter`/`RequestDevice` with `emscripten_sleep` under `-sASYNCIFY` —
  keeping the async dance contained in the backend (no JS device handoff; the
  MODULARIZE'd module just needs the canvas).
- **WGSL shaders embedded as source strings**, compiled at runtime — matching the
  Metal backend (which compiles MSL strings) rather than the Vulkan offline
  SPIR-V path, since the web build has no build-time shader compiler step and
  file I/O is awkward in the browser FS.
- **Single-threaded.** `Application` forces `JobSystem` synchronous mode under
  `__EMSCRIPTEN__`, so the build links without `-pthread` and needs no
  `SharedArrayBuffer` / cross-origin-isolation (COOP/COEP) headers to be served.
  Revisit if a profile shows the main thread is the bottleneck (then opt into
  pthreads + the isolation headers, or move sim work to a worker).

**Alternatives considered.**
- **WebGL2 (via Emscripten).** Widest browser reach and Emscripten's most mature
  path — rejected as the primary path: it is a global-state-machine API that maps
  poorly onto the command-encoder-style `Renderer` seam (we'd fight our own
  abstraction), it forecloses compute, and it is the legacy direction. Kept in
  reserve as a separate fallback project if supporting pre-WebGPU browsers / very
  old devices becomes a hard requirement.
- **Native build + pixel streaming** (run Vulkan/Metal server-side, stream
  frames) — rejected: not "compiling to the web"; heavy infra, latency, and cost.
- **Dawn/wgpu compiled to wasm ourselves** — rejected: redundant. Emscripten
  already provides the WebGPU C API mapped onto the browser's `navigator.gpu`.

**Consequences / tech debt.**
- A **third shader tree** (MSL + GLSL + WGSL) to keep in lockstep — the dominant
  long-term cost ADR-0057 flagged. A future shader-transpile step (Tint/Naga, or
  generating WGSL from the GLSL) could retire it; out of scope for the foundation.
- **Compiles + links on emsdk 6.0.1** (`viewer_web`: WebGPU backend + Jolt +
  engine_core, all to wasm) and **runs in a real browser** (headless Chromium /
  SwiftShader): the device/surface/pipeline come up, the frame loop pumps,
  `endFrame` records the scene's draws against a successfully-acquired surface
  texture, and there are **no WebGPU validation errors** (a device uncaptured-error
  callback is wired to the log). Visible pixel capture is the one thing still
  unconfirmed — headless SwiftShader doesn't composite a WebGPU canvas for
  screenshot/readback, so a real-GPU browser run is needed to eyeball output. The
  emdawnwebgpu Dawn-specific API is explicitly not stable, so a newer port may
  need small edits — `src/renderer/webgpu/AGENTS.md` lists the current gotchas.
- **Bundle size** (verified, with physics): ~1.6 MB gzipped over the wire at `-O2`
  (4.6 MB raw), ~1.3 MB gzipped at `-Oz` (wasm dominates; Jolt is a large share).
  Comparable to a medium SPA — not a blocker for a desktop-browser target.
- Built incrementally like Vulkan (clear screen → lit mesh → full forward →
  shadows → post → instancing/terrain), each stage independently verifiable.
  **This ADR lands Phases 0+1** (bring-up + forward single-light Cook-Torrance);
  later phases are tracked in `docs/webgpu-renderer-plan.md`.
- The `Real = double` engine math (ADR-0005) stays valid CPU-side; the GPU side
  is `f32` only (matrices/vertices are packed to float on upload, as on Vulkan).

**Revisit trigger.** Needing pre-WebGPU browser support (reconsider a WebGL2
fallback); the three-way shader divergence becoming painful (adopt a transpile
step); or the single-threaded main loop becoming the bottleneck (opt into
pthreads + COOP/COEP).

---

## ADR-0059 — A living-world agent layer: runtime NavGraph + A* + deterministic agent sim, then physics vehicles in Lua

**Context.** The procgen pipeline builds beautiful but *empty* worlds — spline
road graphs, welded junctions, scattered forests — with no motion in them. The
engine already has the three things a traffic/crowd system needs and most
engines lack at this stage: a **deterministic seeded sim** (ADR-0002), **Jolt
physics** (ADR-0012) — and Jolt v5.5.0 ships the full vehicle API
(`VehicleConstraint`/`WheeledVehicleController`) — and a **real spline road
graph** (ADR-0049/0056). The wishlist (June 2026): cars and pedestrians that
*use* the generated roads; a car the player can jump into and drive with a
forgiving, video-game feel; vehicle bodies authored in **Lua** like `flora` and
`gun.lua`; and agents with a *reason* to move — a daily schedule, home→work→home.

The blocker: the road network is consumed at mesh-build time and discarded. The
`RoadNet`/`RoadGraph` exist only long enough to stroke ribbons; nothing
navigable survives to runtime. (`netGraph(net)` already samples every edge into a
fine straight-segment polyline that follows the rendered centerline exactly — the
ideal source for a nav graph, computed and thrown away.)

**Decision.** Build the system in phases over the existing value types and seams,
deterministic from a seed, headless/CI-testable before any GPU/physics work
(ADR-0002, "small reversible steps"). Confirmed scope (June 2026): **Jolt wheeled
vehicle tuned arcade-forgiving** for the player car; **hybrid simulation** —
AI agents kinematic on lane splines (scales to hundreds), the player's car (and
optionally a few near it) on full Jolt physics; vehicle bodies authored in Lua.

*Phase 1 (this ADR — the navigation foundation, headless):* a new `engine/ai/`
module, pure data + pure functions, no ECS/render/physics:
1. **`NavGraph`** (`nav_graph.{h,cpp}`) — the persistent, *directed* lane graph
   derived from a (sampled) `RoadGraph`. Each undirected `RoadEdge` becomes two
   `NavLink`s (one per direction; one-way for `Ramp`); lane count per direction
   is derived from carriageway width. Geometry helpers place a point in a lane
   (`laneCenter`, right-hand traffic) or on the sidewalk verge (`sidewalkPoint`),
   plus `nearestNode`/`nearestLink` to snap trip ends. Built from the *same*
   `netGraph(net)` the mesher uses, so lanes line up with the visible asphalt.
2. **Pathfinding** (`pathfind.{h,cpp}`) — A* over the directed lane graph. Cost is
   **travel time** (`length / classSpeed`), so routes prefer arterials over local
   streets like real driving; the heuristic is straight-line time at the fastest
   class speed (admissible → optimal). Deterministic tie-break by link index.
3. **Agent sim** (`agent_sim.{h,cpp}`) — a deterministic `AgentSim` owning cars
   and pedestrians, each with a `home` and `work` node and a daily schedule
   (depart-for-work / depart-for-home, per-agent jittered). A wrapping in-world
   clock drives state transitions (AtHome→Commuting→AtWork→Returning); agents A*
   to their destination and advance along the route by arc length at a
   class-appropriate speed (cars) or walking speed (pedestrians). Same seed + dt
   sequence → identical trajectories.

*Phase 2 (landed):* an ECS `TrafficSystem` (`systems/traffic_system.{h,cpp}`)
that builds the NavGraph from the level's `RoadNet` entities (via the new public
`navRoadGraph` accessor — the same sampled+constrained graph the mesher uses),
runs the `AgentSim`, and bakes agent poses into two `InstanceGroup`s (cars,
pedestrians) rebuilt each fixed step, so RenderSystem draws the whole crowd as
two instanced batches. Built lazily on the first `fixedUpdate` after load and
registered in `ArenaState`. `build`/`step` take a `World` directly so the spawn +
pose-sync logic is unit-tested headless (the GPU mesh upload is the only
AssetManager-gated part). **Render path is backend-agnostic:** the crowd draws
through the generic `Renderer` seam (`drawMeshInstanced`, whose default loops
`drawMesh`), so it renders on **both** Metal and the new Vulkan backend (ADR-0057)
with no traffic-specific wiring — i.e. verifiable on a Linux/Vulkan viewer build,
not Metal-only. (Coalesced GPU instancing on Vulkan — Metal already batches by
mesh handle — is an open perf optimization, noted in the tech-debt register; the
default per-instance loop is correct meanwhile.) The nav stack is additionally
proven headless over the real grid/radial/tensor generators
(`tests/test_traffic_city.cpp`): connectivity, routing, a simulated commute day,
and determinism over a generated city.

*Chase camera (landed, verified):* `FollowCameraController` — a pure, window-free
third-person rig (orbit behind a tracked target, look-to-orbit, zoom-dolly),
unit-tested headless like the fly/orbit controllers. The camera half of "jump in
and drive", landed ahead of the physics car so it ships verified.

*Phases 3-4 (landed, UNVERIFIED — submodule-gated):* the drivable physics car and
its Lua authoring. This sandbox cannot fetch the Jolt/Lua submodules (the git
proxy only allows the repo, so submodule clones 403), so this code is written
against the documented Jolt v5.5.0 vehicle API and the existing flora/gun Lua
patterns but has NOT been compiled here — it must be built/tuned on a submodule
machine. What was added:
- **`PhysicsWorld` vehicle seam** — Jolt-free `VehicleConfig`/`VehicleWheel`, an
  opaque `VehicleId`, and `addVehicle`/`setVehicleInput`/`vehiclePosition`/
  `wheelTransform`/`removeVehicle`, wrapping Jolt's `WheeledVehicleController` +
  `VehicleConstraint` + a raycast collision tester in the .cpp (no JPH:: in the
  header, per ADR-0012).
- **`Vehicle`/`InVehicle` components + `VehicleSystem`** — creates the Jolt
  vehicle from the config + Transform, drives it from the seated driver's input,
  writes chassis + wheel transforms back, and owns the enter/exit interaction
  (board a nearby car with G, suppress on-foot movement via `InVehicle`, switch
  the view to the chase camera; `PlayerSystem` yields while seated).
- **Lua bodies** — `assets/scripts/vehicles.lua` (`vehicle.sedan/hatchback`,
  authored over the procgen `mesh.*` builders like `flora`), a `VehicleSpec`
  reader (`scripting/vehicle_spec.cpp`) that runs the recipe and reads the body
  mesh + handling params, and `spawnVehicle` to build the entity.
- **Data-driven placement** — the level loader reads a top-level `"vehicles"`
  array (`loadVehicles`): each entry is a `recipe` name (or a full `script`) plus
  a `position`, `yaw`, and `seed`, spawned in play mode via the reader above.
  `assets/levels/arena.json` ships two (a sedan + a hatchback) near the player.
The non-submodule-bound glue (VehicleSystem, the camera switch, the loader hook,
components) was nonetheless syntax-checked (`clang -fsyntax-only`) against the
real engine headers; only the Jolt body code and the Lua reader are uncompiled.

*Later phases (planned):* the kinematic-far / physics-near handoff (promote AI
traffic cars near the player to full Jolt vehicles, demote distant ones).

**Why this shape.** It is a *convergence* of shipped systems, not a new pillar:
the nav graph falls out of the road graph; A* rides the existing curve/arc-length
math; the sim obeys the deterministic-clock discipline already used for physics.
Phase 1 is entirely pure and unit-tested on Linux/CI (`make test`), matching the
engine's culture and de-risking the GPU/physics phases that follow.

**Owed.** A compile/tune pass on the submodule-gated vehicle code (Phases 3-4
above) on a Jolt/Lua build — likely small Jolt member-name/ctor touch-ups and
handling/feel tuning. The kinematic-far/physics-near handoff and a level-JSON
`vehicles` block (above). `NavGraph` is rebuilt from the `RoadNet` on demand
rather than cached on an entity; turn-restrictions are not modelled (all turns
except U-turns allowed). Same-lane **car-following** keeps agents queued behind
each other (`carFollowingCap`), and cars **ease to a crawl approaching
intersections** (`NavGraph::isJunction` + a deadlock-free slowdown, never a stop)
— but full **right-of-way / traffic-signal arbitration** is still future work, so
cross-traffic doesn't yet truly yield, and the player's physics car isn't avoided
(no inter-agent collision). All headless-tested. Destroyed
`Vehicle` entities don't `removeVehicle` (no destroy hook, same bounded-leak class
as the ScriptBehaviour note below).

---

## ADR-0060 — Agent-based city simulation as an application (perception, signals, rules)

**Context.** The traffic/crowd sim (ADR-0057/0058) had grown realistic enough to
expose what "realistic" really needs: agents that perceive, obey signals, yield to
pedestrians, turn with a radius, and drive their cars *as agents* — and which can
make faults. That is a simulation *application*, not core engine: its policy
(traffic rules, schedules, signal timing) is game-specific, while the substrate
(NavGraph, A*, perception, wheeled-vehicle physics) is reusable.

**Decision.** Build it under `src/apps/citysim/`, a deterministic agent sim over
the core navigation substrate. Full plan + phases + test cases:
`docs/agent-sim-plan.md`. Key choices:
- **Agent = brain (data); a car is an inert `Vehicle` until an agent possesses it.**
  A pedestrian is a brain-only agent. The *player is just an agent whose brain is
  input* — same possession model, unifying player and crowd.
- **Two layers:** a pure, deterministic decision core (perception → rules →
  kinematic motion), headless/CI-tested; and a device-verified physics/render skin
  (Jolt wheeled cars near/driven by the player; instanced models for
  cars/peds/signs/lights).
- **Core vs app split:** `NavGraph`/A*/`perception` (vision cone) and the
  `PhysicsWorld` vehicle stay in `engine/`; traffic signals, rules, agent brains,
  schedules, and the `CitySim` driver live in the app. Anything that proves
  generally useful graduates back into `engine/` (perception already did).

**Phase 1 (landed, headless):** core `engine/ai/perception.h` (a forward
`VisionCone` + `sees`/`forwardDistance`); app `traffic_signal.{h,cpp}` (a
deterministic `SignalController` over NavGraph junctions — opposing arms share a
phase, phases cycle green→yellow→red, perpendicular arms never both green); app
`traffic_rules.{h,cpp}` (pure `approachStop` / `signalSpeedCap` /
`nearestObstacleAhead`). 12 tests; suite 566/566.

**Phases 2-5 (landed, headless).** `CitySim` (`src/apps/citysim/city_sim.{h,cpp}`)
is the agent framework: agents possess cars (two-way link), commute on a daily
schedule, follow lanes/gaps, obey signals (hard stop line at red) and crosswalks,
brake for what they SEE in a vision cone, make deterministic perception faults
below reliability 1.0, and steer through junctions on a bounded-turn-radius arc
(rate-limited heading) instead of snapping. Tests: `test_city_sim`,
`test_city_signals`, `test_city_perception`, `test_city_steering`.

**Phase 6 (render landed, headless-tested; hybrid physics device-side).** App
`CityRenderSystem` (`src/apps/citysim/city_render.{h,cpp}`) builds the NavGraph
from the level's RoadNets, runs the `CitySim`, and bakes poses into
InstanceGroups — one for cars, one for pedestrians, one per signal state with an
emissive lens that lights up for the active phase. `arena_state` now registers it
in place of the old AgentSim-backed `TrafficSystem`. App sources compile into
`engine_core` (like `traffic_system.cpp`) so viewer/editor link them; the
core→app dependency direction is preserved (the bridge depends on core, not vice
versa). Tests: `test_city_render`. Remaining device-side: running near/driven AI
cars on Jolt wheeled physics (the hybrid model — far cars stay kinematic); the
player car already uses Jolt via `VehicleSystem`.

---

## ADR-0061 — Reactive, perception-driven agents (state machines + local steering) and one composable vehicle

**Context.** The city sim (ADR-0060) moves agents by GLOBAL plans: an A* route is
computed up front and each agent snaps along lanes leg by leg. That produces
discontinuities (lane-side snaps at nodes, arrival teleports, one-step merge
overlaps), agents that can drive/​walk *through* each other (no real reaction),
and NPC cars that share nothing with the player's car (separate mesh + separate
kinematic mover). The desired feel is more emergent: an agent should act on what
it can SEE in front of it and decide locally — closer to boids than to a solved
path — and every vehicle should be the same kind of object the player drives.

**Decision (direction; phased).**

1. **An agent is a reactive brain with a cone of vision and a state machine.**
   Each step an agent perceives only what falls in its `VisionCone` (neighbours,
   the car/person ahead, a signal, the player), runs a small finite state machine
   (e.g. pedestrian: `Resting → Walking → Avoiding → WaitingToCross → Crossing`;
   driver: `Cruising → Following → Yielding → Stopping → Turning`), and outputs a
   **steering intent**, not a teleport. Decisions use only visible/local
   information — an agent can be surprised (it does not know what is behind it or
   what is coming next). Optional agent MEMORY (recently-seen obstacles) is a
   later pass.
2. **Movement is continuous local steering (boids-style), integrated over time.**
   Behaviours combine as weighted steering: SEEK the current goal/waypoint,
   SEPARATE from visible neighbours (evade — but only ones in the cone), align/
   follow to form lanes, and avoid obstacles. This replaces the leg-by-leg lane
   snap, so motion is continuous and agents part around each other instead of
   overlapping. The NavGraph still supplies the road/sidewalk geometry and a goal
   to seek; it stops being a rigid rail. A* (if kept) only picks a coarse goal
   sequence — the moment-to-moment behaviour is reactive.
3. **One composable vehicle for player and AI.** A vehicle is an entity composed
   of components — Body (Lua-authored mesh + dimensions + type: car/truck/tractor-
   trailer), Chassis (Jolt physics + drivetrain), Handling (input→forces), Seats
   (occupancy: an agent or the player), Lights (toggle), and a **Controller** that
   is the sole source of `{throttle, steer, brake}`. `PlayerController` reads
   input; `DriverAgent` is the reactive brain above producing the same three
   numbers. The vehicle does not know who drives it — so NPC cars look and move
   like the player's, just with an AI at the wheel, and discontinuities vanish
   because physics integrates position.
4. **Debug widgets.** Each agent can render a footprint (bounding circle/rect) and
   a forward trajectory vector on the ground, to make the perception/steering
   legible while tuning.

**Why this over the A* rail.** It is *less* optimal for routing but far more
dynamic and robust: reactive agents don't teleport, resolve their own conflicts
by steering, and degrade gracefully. It also unifies the two vehicle code paths
onto the player's.

**Phasing (each shippable).** (1) Pedestrians go reactive first — perception-
gated local avoidance + a walking FSM (headless-testable). (2) Debug widgets.
(3) Drivers go reactive — FSM + steering that follows the lane and reacts to what
it sees. (4) Unify the vehicle body (NPC cars from `vehicles.lua`). (5) The
`DriverAgent` controller drives a real Jolt vehicle near the camera (device). See
`docs/vehicle-agent-plan.md`.

**Status.** Phases 1–3 have landed (headless-tested). (1) Pedestrians steer
continuously around what they see (`city_sim.cpp` reactive pass) + a hard body
floor. (2) Per-agent debug widgets — footprint ring coloured by state + forward
arrow — draw on top via `FLAG_OVERLAY` on both backends. (3) The driver reactive
layer was already in `advance()` (bounded-turn-radius steering, cone braking for
people, car-following, signal stops); Phase 3 adds the **explicit driver FSM** on
top of it: each step a car labels itself Cruising / Following / Yielding / Turning
/ Waiting from what governs it, surfaced through the debug widgets and covered by
`tests/test_city_driver_fsm.cpp`. The `Agent::State` enum now spans both the
pedestrian and driver states (`Count` sizes the render arrays).

Phase 4 (composable vehicle body) has landed in its headless-testable form: a
shared **fleet table** (`city_sim.cpp` `kFleet`) gives every vehicle a Body — a
`VehicleType` + real dimensions (sedan / hatchback / SUV / pickup / van / box
truck). The sim's `SimVehicle` takes its slot's body; the renderer mirrors the
SAME slot (mesh style + size from the fleet, `city_render.cpp`); and the collider
sizes each group's boxes to match (`carGroupHalfExtents`). Car-following is now
**length-aware** (`pairMinGap`): a sedan pair reproduces the historical 5.0 m gap
exactly, and a longer body keeps (and is kept at) a proportionally larger bumper
gap, so adding trucks never lets traffic pack tighter — covered by
`tests/test_city_fleet.cpp`. What is NOT yet done in Phase 4: NPC bodies are still
procedural box meshes that MIRROR the player's `vehicles.lua car_body` by hand
rather than being built from the actual Lua recipe (true 1:1 unification needs the
Lua/viewer build). Phase 5 (`DriverAgent` controller on a real Jolt vehicle near
the camera) remains — it needs a device build (Jolt/Lua/viewer) to verify.

---

## ADR-0062 — One car system: every car is a physics Vehicle driven by a Controller

**Context.** The engine had TWO car systems that shared nothing (ADR-0060/0060).
The player drove a real Jolt wheeled `Vehicle` (`VehicleSystem`), read from host
input. NPC city cars were KINEMATIC `SimVehicle` pose-holders moved along the
NavGraph by `CitySim` and mirrored into kinematic collider boxes — they could not
be driven, could not be entered, did not obey physics, and did not handle like the
player's car. This split is exactly what an "agent" is supposed to dissolve: if an
agent's car isn't the same object the player drives, the agent idea is hollow. The
user's standing requirement: **one car system** — the player can walk up to ANY
car and drive it, ejecting whatever agent was inside, and every car drives the
same because every car IS the same object.

**Decision.** There is one kind of car — an engine `Vehicle` (Jolt) — and one
control seam. Whoever drives it, the player or an AI, speaks only in a
`DriverInput {throttle, steer, brake, handBrake}`, fed to the one physics path
(`PhysicsWorld::setVehicleInput`).

1. **The controller seam (`engine/ai/driver_agent.h`).** `computeDriverInput` is a
   pure function: given what the car IS (`DriverState`: forward heading + forward
   speed) and what the brain WANTS (`DriverCommand`: desired heading + speed), it
   returns the pedal/wheel positions — steering aims the heading, throttle/brake
   close the speed gap. Unit-tested headless (`test_driver_agent.cpp`).
2. **`AgentDriver` component.** Marks a `Vehicle` as AI-driven: it carries the
   brain's current `DriverCommand` + tuning + an `agentId` back to the brain.
   `VehicleSystem::driveVehicles` is now one path: a seated player writes input
   from host axes; an `AgentDriver` car computes input from its command via the
   shared controller; an empty car holds the brake. Same input → same physics for
   every car.
3. **Enter any car / eject the agent.** `handleEnterExit` lets the player take the
   nearest car whether or not an AI drives it; entering removes the car's
   `AgentDriver` (the agent is kicked out) and seats the player — one car, one
   driver.
4. **The city bridge (`apps/citysim/city_vehicles.cpp`, viewer/device).**
   `CityVehicleSystem` spawns each CitySim driver a real `Vehicle` from its fleet
   body (a `VehicleConfig` sized from the body + the same fleet mesh), tagged
   `AgentDriver{agentId}`. The `CitySim` keeps running as the **planner**: each
   step its ghost agent produces a heading + speed, which the bridge writes into
   the car's command; the real car chases its ghost (heading blended with a pull
   back toward the planned lane). The render bridge cedes car ownership
   (`setCarsExternallyOwned`) so cars aren't drawn twice, and the kinematic car
   colliders are gone — `CityPhysicsSystem` now only does peds + poles. When the
   player commandeers a car, the bridge sees the `AgentDriver` removed and calls
   `CitySim::releaseDriver` so the planner stops fighting the physical car.

**Refinement: the loop is CLOSED (same ADR, follow-up).** The first cut chased
the ghost open-loop (heading blended toward the ghost's position, speed copied
verbatim), which lags, cuts corners, and strands a car that falls behind its
plan. Now:

- **Heading — pure pursuit** (`engine/ai/lane_follow.h`): `LaneFollower` tracks
  the car's real progress along the ghost's lane polyline (monotonic, windowed —
  a path that doubles back can't snap progress onto the wrong leg) and aims at a
  speed-proportional lookahead point. `CitySim::lanePath()` exports the route as
  that polyline; the bridge rebuilds it per trip (`Agent::trips`).
- **Speed — station control**: the bridge commands `planned + gain*(gapToGhost -
  standoff)`, capped — a lagging car catches up, an overshooting car eases off,
  a ghost held at a red parks the car at the line. `pursuitCommand` additionally
  caps speed by the REMAINING path, so a car can never sail past its path end
  and orbit the last point (the harness caught exactly that failure).
- **Tether — the plan waits for the physics**: `CitySim::setAgentTether` holds a
  ghost that leads its physical car by more than a leash, so a collision, hill,
  or slow start can never permanently separate car from plan.
- **Controller — slow into turns**: `computeDriverInput` sheds target speed as
  the desired heading swings away from the car's (`DriverTuning::turnSlowdown`),
  so a car brakes into a sharp turn and accelerates out instead of carrying
  cruise speed through it.
- **A closed-loop harness** (`tests/test_lane_follow.cpp`): a kinematic
  bicycle-model car driven by the real controller stack — lane convergence from
  an offset, a 90° corner, a hairpin with monotonic progress, parking at the
  path end, determinism. Driving quality is now measured headless; the on-device
  work is tuning, not debugging the control law.

**Refinement 2: real-pose perception, recovery, personality.**

- **Eyes on real poses** (`engine/ai/traffic_sense.h`): each physical car senses
  the other PHYSICAL road users from a forward cone — NPC cars where physics
  actually put them, the player's car, the on-foot player, pedestrians — and
  `followSpeed` caps its commanded speed off the nearest one ahead in its lane
  (length-aware ramp to zero at the bumper, floored at a moving leader's pace so
  convoys flow). Moving cross/oncoming traffic is deliberately not a leader (the
  planner's anti-deadlock lesson); anything STOPPED in the lane is an obstacle
  whichever way it faces. Closed-loop tested: a two-car bicycle-model convoy
  never touches and still keeps up (`tests/test_traffic_sense.cpp`).
- **Stuck/flip recovery**: a `StuckDetector` (wants speed, gets none, for 4 s)
  or a >2 s roll triggers `PhysicsWorld::resetVehicleUpright` — a beached,
  wedged, or flipped NPC car self-recovers instead of blocking a lane forever.
- **Personality**: each agent carries a `speedFactor` in [0.85, 1.15] (derived
  from its brain seed's bits — no rng draw, so seeded scenarios are unchanged)
  applied to its nominal driving/walking pace, and each car's controller gains +
  following buffer vary deterministically per agent — the fleet stops moving in
  lockstep. Junction/signal caps stay shared (road rules, not temperament).

**Refinement 3: first device feedback (parked roadblocks, corridor sensing,
honest widgets).** Device testing surfaced two failures. (a) *Cars piled up at
junctions*: arrived/idle driver ghosts rested at the END of their lane — harmless
kinematically, but the PHYSICAL car that follows the pose became a roadblock
parked at the junction mouth until its next trip, hours later. Drivers now park
OFF the carriageway (verge beside the link, a per-agent setback so arrivals don't
stack; idle poses likewise), and the off-trip bridge branch creeps the car to
that kerb spot. (b) *Debug rings with nothing in them*: the widgets drew at the
planner-ghost pose while the physical car was elsewhere. The bridge now reports
each car's REAL pose (`setExternalCarPoses`); a driver's ring circles the actual
car, and unreported drivers (released to the player) draw none. Alongside: the
straight sensing cone is replaced on-trip by a **corridor along the pursuit
path** (`senseAlongPath`) — it bends with the road, so a stopped queue in the
oncoming lane of a curve can no longer read as "ahead of me" and freeze both
directions — and an **anti-gridlock valve** noses a car slowly past a STOPPED
cross car (never a pedestrian/player) after a few blocked seconds, so mutual
junction blocks self-resolve.

**Refinement 4: physical walkers, think cadence, honest debug HUD.**

- **Pedestrians get bodies** (`apps/citysim/city_walkers.cpp`): each ped agent
  spawns an entity with a kinematic character capsule (the player's own
  `CharacterController` + `moveCharacter` mechanism), driven each step toward
  its planner ghost (walker-simple station control) — walkers now collide with
  parked cars, poles, kerbs, the player; the ghost tether applies to them too.
  Knockdown-and-recover: a fast vehicle passing through a walker's body floors
  it (`KnockdownTimer`, a proximity+speed trigger) — it lies flat, gets up,
  walks on. The render bridge cedes ped ownership (`setPedsExternallyOwned`).
  The v1 down pose tilts the render box; the capsule stays upright (ragdoll is
  future work).
- **Think cadence** (`Agent::thinkTimer`/`leanTarget`, `setThinkPeriod`): agents
  DECIDE on a slow, per-agent-staggered clock (default 0.35 s) and COMMIT — the
  reactive scan runs only on think ticks; between them the committed decision
  holds while every tick still integrates smoothly toward it. Per-tick
  re-deciding was the walker oscillation ("wigging out"); a held decision reads
  as intent. Controllers stay per-tick (stability); decisions go slow
  (commitment).
- **Near-field cross braking** (`senseAlongPath` `crossNearRange`): far-field
  moving cross traffic is still ignored (flow control belongs to the signals),
  but a crosser within ~8 m ON MY PATH is an imminent T-bone — brake. Two
  mutual stoppers resolve via the creep valve. This is what physical junction
  bodies needed that ghost junctions never did.
- **Debug HUD that answers "what is it thinking?"**: the footprint is now a
  bright emissive WIRE HOOP at body height (never a flat "shadow ring"),
  coloured by the FSM state with traffic-light semantics — green = going, RED =
  held, amber = braking/avoiding, violet = turning, teal = following — and the
  arrow points from the agent to its CURRENT GOAL (the pursuit lookahead /
  planned spot), reported by the bridges alongside the real poses.

**Refinement 5: junction throughput, walker awareness, ground-projected HUD
(second device round).** All-red rings + junction pile-ups traced to the box
never draining and spawn placement. (a) **Don't block the box**: a car whose
plan is crossing (`nearJunction` + ghost moving) never idles inside a junction —
it commits at a minimum pace, spooked by nothing except a person at its bumper;
the creep valve quickened. (b) **Spawns never land in the box or mid-lane**: a
mid-trip ghost's car spawns on the verge beside it, staggered, backed out of any
junction, and merges via pursuit. (c) **Walkers got 360° spatial awareness**: a
separation steer from every nearby person (beyond the planner's forward cone), a
BLOCKED state (wants to walk, body pinned → stand still a beat, re-think — shown
as a red ring via the widget `stateOverride`), and the player gets a wide hard
berth in the plan itself (`kPlayerClearance`) so near-misses read as
step-arounds. (d) **The HUD projects on the GROUND** like painted road markings
(regular depth — always visible the way lane paint is), rims proud of the body,
wide intent arrows; the waist-height overlay hoop was invisible inside body
geometry on device, which also suggests the Metal instanced FLAG_OVERLAY state
isn't taking effect — recorded below. (e) **The agent is VISIBLE in its car**:
the driver capsule now shows for `AgentDriver` cars, not just a seated player.

**RETHINK (supersedes the ambient-dynamic default): one motion authority per
regime.** Three device rounds of jammed junctions forced the honest diagnosis.
The all-dynamic design put TWO clocks over every ambient car — a kinematic
planner ghost marching on rails, and an untuned, never-yet-driven Jolt plant
trying to chase it — and every fix (tether, station control, box-drain, creep
valves) was another reconciliation loop between them. That is the signature of a
structural error, not a tuning problem. The counter-evidence was in the same
builds: pedestrians work great, because their plant (a kinematic character) is
trivially commandable — the plan proposes, the body executes, one authority.
And the pre-bridge kinematic traffic DROVE (its bugs were flow bugs, which got
fixed), because it too had one authority.

**The answer to "how should an agent be constructed to drive a vehicle":**

1. **Strategic** — A* route over the NavGraph → a lane-accurate path. Data.
2. **Tactical (the agent)** — the FSM + perception deciding, at think cadence,
   a target speed along that path (stop at the line, follow the leader, slow
   for the turn) and when to re-plan. This layer IS the agent; it is
   plant-agnostic and headless-testable, and it already exists.
3. **Motor — exactly ONE owner per regime.**
   - **Ambient regime (the default for all traffic)**: the planner integrates
     the car along its path directly (position = path(s), s += v*dt) — the
     CitySim as it always worked — drawn instanced, collided via KINEMATIC
     proxies. No ghost, no tether, no reconciliation, no way to "get stuck"
     except where the brain says stop. This is how ambient traffic works in
     shipped open-world games, for this reason.
   - **Interactive regime (promotion)**: full Jolt dynamics is an INTERACTION
     response, not a gait. When the player commandeers an ambient car
     (`CityVehicleSystem`, now the PROMOTION system), the sim releases the
     agent (instanced car + widget vanish), a real `Vehicle` with the same
     fleet body spawns in its place, and VehicleSystem's enter flow seats the
     player the same frame. Later, the same mechanism can make a shot/rammed
     car briefly dynamic — the AgentDriver + pursuit/sensing stack stays in
     engine/ai, harness-tested, ready to drive promoted cars.

"One car system" survives where it matters — one component model, one fleet
body, one enter flow, one physics API — while motion is owned per regime by the
machinery that is good at it.

**Also this round:** the lofted car shell REGRESSED on device (inverted faces
from a winding heuristic, plus double wheels/lamps — VehicleSystem already adds
physics wheels + lens entities to every Vehicle) — the fleet and `vehicles.lua`
are back on the boxy bodies; the shell stays in-tree as experimental (register
row below). `fleetCarMesh` gained a wheel-less variant for promoted cars;
commandeered agents disappear from the instanced bake + widgets.

**What's verified vs owed.** The planner (signals, following, junctions, think
cadence, personality), the walkers, promotion bookkeeping (release → instance
drop), and the engine/ai controller stack are headless-tested. The promotion
spawn and the player's own car remain Jolt-gated and device-UNVERIFIED. Owed:
verify the Jolt vehicle drives well ONCE (the player's car is the test bed —
tune `configFromBody` there before any AI ever drives dynamics again), ragdoll
knockdown, a GPU line-primitive debug path, the Metal instanced `FLAG_OVERLAY`
depth state (device evidence says it doesn't apply), and the shell rework
(correct winding, crisper colour islands, no baked trim) before it returns.

---

## ADR-0063 — The cognition loop: sense → remember → predict → decide → act

**Context.** ADR-0060/0060 gave agents a vision cone and reactive rules, and
ADR-0062 settled who owns motion. What agents still lacked was an inner life
between frames: perception was a momentary snapshot (a body leaving the cone
ceased to exist), there was no anticipation (a car reacted to a crosser only
once it was already in the lane), and decisions were values re-derived per
tick — the mechanism behind every "wigging out" oscillation we fixed piecemeal.
The user's ask: go a step beyond ambient smoke-and-mirrors — an agent with
goals, senses, short-term memory, trajectory prediction, and decisions that
hold — built from math and structure, and a lab level to watch one think.

**Decision.** A five-stage loop, each stage a small pure engine/ai primitive,
composed in the CitySim and reusable by any future agent:

1. **Sense** — `SensorVolume` (`engine/ai/perception.h`): the planar vision
   wedge plus a HEIGHT BAND (2.5D). Ground agents decide in plan view, but the
   city is grade-separated: a car crossing the overpass 5.8 m up must not be a
   phantom to brake for. A full camera-style frustum was considered and
   rejected — for agents that neither aim nor fly it buys nothing over
   wedge+band, and it costs a matrix test per candidate versus two dot
   products. Occlusion (walls, crests) is a later line-of-sight ray on the few
   nearest candidates, not a reason to start 3D.
2. **Remember** — `AgentMemory` (`engine/ai/agent_memory.h`): sightings become
   TRACKS. Each carries a smoothed velocity estimate (successive sightings
   differenced + EMA — a poor man's Kalman, which is all a game needs), decays
   on a confidence over `memorySeconds` (4 s), and while unseen EXTRAPOLATES
   along its velocity: object permanence. Re-sighting re-anchors; a track cap
   (32) evicts the weakest. ~1.5 KB per agent.
3. **Predict** — `closestApproach` / `timeToCollision` (same header): constant-
   velocity CPA and disc-collision time from the tracked velocities. Dot
   products, not ML.
4. **Decide** — `Commitment` (`engine/ai/commitment.h`): utility choice with
   hysteresis. The incumbent option keeps its seat unless a challenger beats it
   by a clear margin AND a minimum hold has elapsed; an emergency (imminent
   TTC) preempts instantly; re-adopting the incumbent never resets its clock.
   Near-tied scores can flip forever without the behaviour ever flapping — the
   principled form of the think-cadence commitment that fixed the walkers.
5. **Act** — the existing motor layers (ADR-0062): planner integration for
   ambient agents, `AgentDriver` → Jolt for promoted ones.

**Wired into the CitySim** (`city_sim.{h,cpp}`): every `Agent` carries an
`AgentMemory`; the sim keeps a monotonic `simSeconds_` clock; the per-step
snapshot is now `SensedGhost{pos, elevation, id}` (stable ids: agent index,
-(1+k) for host-injected obstacles) so tracks persist across steps and the
sensor can height-gate. Drivers sense per tick (the reliability roll and fault
accounting are unchanged — a blind agent never observes, so the perception
tests hold), then act on MEMORY: yield to any confident track whose predicted
position sits in the corridor ahead, and brake when `timeToCollision` against a
track dips under 2 s — anticipation, so a car eases off BEFORE the crosser is
in the lane, and keeps yielding ~3 s after losing sight (pinned by
`cars_remember_a_person_after_losing_sight`: the car creeps to the remembered
spot, walls short of it, and drives on only once the track fades). Walkers
sense at think cadence — successive sightings a think apart are exactly what
feeds the velocity estimator — and lean away from where each remembered
neighbour WILL be (`kPedAnticipation` 0.4 s), so two approaching walkers part
early instead of flinching late. Poles stay instantaneous (static and eternal —
memory would add nothing).

**The agent lab** (`assets/levels/agent_lab.json`): a theta circuit — a 120×80
ring with a middle bar meeting at two signalled 3-way junctions — running ONE
driver and ONE walker with the debug HUD on from boot. Population comes from a
new top-level `"citysim"` level block, parsed by `level_loader` into a
`CitySimConfig` entity (`components.h`) that `CityRenderSystem::build` reads
and applies over its constructor params (`{cars, pedestrians, seed,
hoursPerSecond, perceptionReliability, debugWidgets}`) — so the lab isolates
one thinking agent while `grown.json` keeps its bustle, from data, no code
fork. Watch the ring: green cruising, amber easing for a remembered crosser
BEFORE it reaches the lane, red walled at the ghost of someone who just left.

**Scaling (asked: "how do we process many agents — a compute shader?").** The
loop is O(nearby) per agent and runs at think cadence (0.35 s, staggered), so
the per-frame cost is `agents / (thinkPeriod / dt)` scans — with spatial
hashing for candidate pruning and read-only snapshots the `JobSystem` can fan
out deterministically, this architecture is comfortable at hundreds of agents
on CPU, and AI-LOD (full cognition near the camera, schedule-only far away)
buys the next order of magnitude. A compute shader was REJECTED at this scale:
GPU readback latency lands decisions a frame late, float nondeterminism breaks
ADR-0002's replayable sims, and debuggability dies — it earns its keep only
past ~10k homogeneous agents, which is not this game. Grounding: Reynolds
steering, ORCA/RVO, IDM/MOBIL, pure-pursuit/Stanley (already in
`lane_follow.h`), Halo 2's perception+memory model, utility AI with hysteresis;
the ML on-ramps (imitation-learn the classical controller into a tiny MLP; RL
for the dynamic regime) stay open — the CitySim is already a deterministic gym.

**Tests.** `tests/test_agent_memory.cpp` pins the pure core (velocity
convergence, extrapolate-then-forget, re-anchoring, eviction, CPA/TTC, flip-
flop suppression, sustained-winner switching, emergency preemption, height
gating); `test_city_perception.cpp` gains sim-level object permanence;
`test_city_render.cpp` pins the level-config override and the lab circuit
(navigable, exactly two junctions, both agents come alive). Suite 661/661.

**Device round 3 (rings + moving traffic confirmed working).** Four fixes from
the first on-device look at the cognition build:

- **The turn "blink".** A car turning at a junction blinked out and back: the
  pose sampled only the CURRENT leg's lane line, and the lane offset direction
  rotates with the leg — so crossing a node teleported the car sideways by up
  to ~4.6 m in one frame (worst on wide arterials). `refreshPose` now traces a
  quadratic Bezier from the incoming lane line to the outgoing one across every
  interior node (blend span = road half-width + 1 m, clamped to 45% of each
  leg), continuous at the leg change by construction — the position finally
  drives the arc the rate-limited heading was already pretending to. Verified
  numerically: worst per-tick displacement through a 90° arterial bend dropped
  from a multi-metre sideways snap to ~1.3x speed·dt (the arc is slightly
  longer than the leg parameterisation — continuous, chained positions).
- **Reds must READ as obeyed.** The signal stop line was `length - 0.5` — the
  node itself — so a car legally holding at a red stood in the middle of the
  intersection on top of the painted crosswalk, looking exactly like a
  red-light runner. The car stop line is now set back: junction box radius (the
  mouth) + the zebra band (painted 0.5..3.6 m past the mouth) + margin + half
  the car's own body, so the BUMPER holds short of the crosswalk. Walkers still
  hold at the corner. The braking cap eases to the LINE, not the node.
- **The lab must keep moving.** With the daily schedule, the lab's one car
  commuted once and parked for (real) minutes — "I would expect the car to be
  running around the track." New WANDER mode (`CitySim::setWander`, level
  `"citysim": {"wander": true}`): agents at rest immediately start a fresh trip
  to a random reachable node. The theta test now pins ≥3 consecutive trips.
- **Parked cars looking misplaced + white rings.** Cars idle at home/work parked
  on the verge by DESIGN (ADR-0062: an idle body on the carriageway is a
  roadblock), but the initial idle pose kept the default (1,0) heading — parked
  cars sat at random angles to their road, reading as "failed to be placed".
  Idle bodies now face along their road from build. Widget ring emission tuned
  4x → 1.7x: at 4x the bloom saturated every hue toward white on device.

---

## Interim seams & tech-debt register

Deliberate shortcuts taken to keep steps small and low-risk. Each is expected
to be replaced; listed here so they stay visible.

**Cross-platform parity discipline.** The realtime viewer targets two backends —
Metal (macOS) and Vulkan (Linux/Windows) — and they must stay at feature parity
(ADR-0044/0057). A render feature is not "done" when it works on one backend: it
is done when it is wired on **both**, or when the gap is recorded here as a
parity debt with the file + the backend that still owes it. Land a feature on
one backend and forget the other and the two drift apart fast. So: when you add
anything to one renderer (`metal_renderer.mm`) mirror it in the other
(`vulkan_renderer.cpp`) in the same change if you can, and if you can't (e.g. one
side needs a device you can't build here), add a register row naming the owed
backend and mark it UNVERIFIED so a device pass closes it.

| Item | Where | Why interim | Replace with |
|---|---|---|---|
| `RenderView` shared resource | `engine/system.h` | Minimal stand-in for engine resources | A real ECS resource/blackboard (ADR-0006) |
| ~~`MotionSystem` is a physics placeholder~~ | ~~`engine/systems/motion_system.cpp`~~ | *Repositioned (ROADMAP 2.3 Step C): not a placeholder — it is the kinematic mover for cheap, collision-free scripted motion. Dynamics live in `PhysicsSystem` (ADR-0012); MotionSystem yields any entity with a `RigidBody`.* | — |
| ~~Hardcoded keybindings~~ | ~~`engine/systems/dev_control_system.cpp`~~ | *Resolved (ROADMAP 2.1, 2.2): named-action layer `InputMap` (`engine/input/`); `DevControlSystem` and `CameraSystem` (orbit + fly) drive entirely off actions, gamepad included.* | — |
| Incremental logging adoption | various | Avoided a sweep | Migrate remaining `std::cerr` sites |
| ~~No automated tests~~ | ~~repo-wide~~ | *Resolved (ROADMAP 1.3): `tests/` target, `make test` / CTest, 33 cases over `SlotMap`/`SparseSet`/`World`/math/`SimClock`.* | — |
| ~~Legacy `uint32_t` handles~~ | ~~`renderer.h` (`MeshHandle`)~~ | *Resolved (ADR-0007): `MeshHandle`/`BufferHandle` are now `Handle<Tag>`; the Metal backend stores meshes in a `SlotMap<GPUMesh>`. First brick of the asset system.* | — |
| macOS-only verification | `window.cpp`, `metal_renderer.mm`, `gamepad_gc.mm` | Only backend; not Linux-compilable | Second backend + CI that can build it |
| Partial live-resize fix | `Window` draw callback | Repaints, but sim is frozen mid-drag | Refresh-driven redraw / resize-aware loop |
| No custom allocators / memory tracking | repo-wide | Deferred (ADR-0008); `SlotMap` covers pooling | Frame arena when churn measured; allocators + tracking on data |
| Core containers not thread-safe; `JobSystem` is single-queue | `slot_map.h`, `sparse_set.h`, `world.*`, `job_system.*` | `JobSystem` (ADR-0014) parallelizes independent work only; ECS/containers stay single-threaded | Container thread-safety / deferred command buffer when ECS systems parallelize; work-stealing when a profile demands it |
| ~~Transitional namespace shims~~ | ~~core headers + leaf consumers~~ | *Resolved (ADR-0015): the staged `namespace engine` migration is complete; all `using engine::…` aliases removed. Leaf consumers keep a `using namespace engine;` by design.* | — |
| ~~Projection matrices built in backend~~ | ~~`metal_renderer.mm`~~ | *Resolved (ROADMAP 1.2): `Mat4::perspective`/`orthographic` now build the matrices engine-side; backend calls them via `toSimd`. Perspective depth convention fixed to Metal [0,1]; regression-tested.* | — |
| No dedicated 2D camera | `renderer/orbit_camera.*` | Ortho via OrbitCamera as stand-in | A pan/zoom 2D camera when 2D is built |
| Editor mesh re-uploads leak | `engine/systems/editor_system.cpp`, `level_loader.cpp` | Edit/play cycles and size edits upload new meshes without freeing the old (ADR-0019/0020 editor) | The `AssetManager` (ROADMAP 3.1, `docs/asset-system-plan.md`): refcounted, deduped mesh ownership; `release` on overwrite, `clear()` on world teardown |
| Lights & render settings outside the document model | `renderer.h` (`SceneLighting`), level JSON `lighting` | Authored by hand-editing JSON; not entities, not inspectable/undoable | `Light` component + a LightSystem; art-direction render settings via the property layer (ADR-0018), perf/quality stay in settings.json |
| ~~`ScriptSystem` not wired into a running state~~ | ~~`engine/scripting/script_system.*`~~ | *Resolved (ADR-0024): registered in `ArenaState` in place of `ShootingSystem`; the player gets the `gun.lua` ScriptBehaviour on level load. macOS/viewer-gated, so CI-unverified.* | An editor "attach script" affordance (author scripts in the editor) |
| Lua behaviour instance refs aren't released | `engine/scripting/script_system.cpp`, `script_behaviour.h` | Slice (ADR-0024): a destroyed entity's registry ref isn't `luaL_unref`'d — bounded leak until the VM closes | `luaL_unref` on `ScriptBehaviour` removal / entity destroy (needs a removal hook) |
| Vehicle physics + Lua code is UNVERIFIED | `engine/physics/physics_world.cpp` (vehicle), `engine/scripting/vehicle_spec.cpp`, `assets/scripts/vehicles.lua` (ADR-0059) | The Jolt/Lua submodules can't be fetched in this env (proxy 403s submodule clones), so the Jolt `WheeledVehicleController` wrapper and the Lua spec reader were written against the documented API but never compiled. The surrounding glue (VehicleSystem, camera switch, arena hook) was `clang -fsyntax-only`-checked | A compile + drive/tune pass on a Jolt/Lua build; expect minor Jolt member-name/ctor fixes and handling tuning |
| Destroyed `Vehicle` entities leak the Jolt vehicle | `engine/systems/vehicle_system.cpp` (ADR-0059) | No entity-destroy hook to call `PhysicsWorld::removeVehicle` (same class as the ScriptBehaviour leak above) | `removeVehicle` on `Vehicle` removal / entity destroy when a removal hook lands |
| Vulkan has no coalesced instanced draw | `renderer/vulkan/vulkan_renderer.cpp` (ADR-0057/0058) | The Vulkan backend doesn't override `drawMeshInstanced`, so traffic/foliage `InstanceGroup`s render via the base per-instance `drawMesh` loop — correct but CPU-bound for big crowds; Metal already batches by mesh handle | A Vulkan instanced path (instance-matrix vertex buffer or SSBO + `vkCmdDrawIndexed` instanceCount) on a device build |
| AI-car Jolt bridge is UNVERIFIED on device | `apps/citysim/city_vehicles.cpp`, `engine/ai/{driver_agent,lane_follow,traffic_sense}.h`, `engine/systems/vehicle_system.cpp` (ADR-0062) | RESOLVED IN DESIGN, owed a device pass: NPC cars are real engine `Vehicle`s (Jolt) driven by an `AgentDriver` through the shared controller — one car system, player can enter/eject any of them. The control loop is CLOSED over the car's real pose (pure-pursuit heading, station-control speed, ghost tether), each car SENSES the other physical road users and follows them length-aware (`traffic_sense.h`), stalls/flips self-recover via `resetVehicleUpright`, and per-agent personality varies pace/gains/gaps. All of that is exercised headless against a bicycle-model plant (`test_lane_follow.cpp`, `test_traffic_sense.cpp`), so the on-device work is gain tuning against the real Jolt plant, not control-law debugging | Device tune (steerGain / lookahead / configFromBody / sense cone); then lane changes + junction yielding on the sensed-neighbour model |
| City pedestrians render as boxes | `apps/citysim/city_render.cpp`, `apps/citysim/city_walkers.cpp` (ADR-0060/0061) | Cars are now curved lofted shells (`buildCarShell` — see the resolved row below); pedestrians are still `MeshBuilder::box` bodies (per-walker clothing tints). A walker needs a simple articulated body (capsule torso, head, leg swing) to stop reading as a crate. | A procedural walker body + simple walk-cycle animation (a content pass) |
| Curved car shell is EXPERIMENTAL (regressed on device) | `engine/procgen/vehicle_mesh.{h,cpp}`, `mesh.car_shell` binding (ADR-0062) | The lofted shell shipped with inverted faces (the outward-winding heuristic fails on parts of the loft — see-through panels), smeared vertex-colour "textures", and it baked wheels/lamps that DOUBLED VehicleSystem's physics wheels + lens entities. The fleet and `vehicles.lua` are back on the boxy bodies; the generator + `mesh.car_shell` + tests stay in-tree, unwired. | Rework before return: robust winding (build quads with known orientation instead of a fix-up pass), per-face colour islands (crisp glass edges), no baked trim, more segments; then a viewer look pass |
| Wide vehicle bodies may cross the lane centreline | `apps/citysim/city_sim.cpp` (`laneCenter`/`laneSpacing`), `apps/citysim/city_render.cpp` | Following is length-aware, but lane placement still centres a car at a width-relative lane offset regardless of the body's WIDTH. A wide van/box-truck (2.0–2.4 m) on a narrow lane could visually overhang the centreline / clip an oncoming wide body. Unverified (no viewer here). | Lateral lane-fit (inset wide bodies, or widen the effective lane) + an oncoming-width check; verify on device |
| Crosswalk road-texture paint is UNVERIFIED on device | `engine/procgen/city/road_mesh.cpp` (`weldSolid`), `shaders/metal/common.metal`, `shaders/vulkan/mesh.frag`, `shaders/webgpu/mesh.wgsl`, `src/scene.cpp` (`surfRoadMarkings`) (ADR-0062) | RESOLVED (design): crosswalks are painted into the ROAD TEXTURE, not overlaid geometry. The default weld mesher bakes the carriageway UV `mv` as metres past the junction MOUTH (`min(arc, len-arc) - halfWidth`), and the RoadMarkings shader stripes a set-back zebra band there — part of the road surface, on the approach, never a floating decal over the centreline. Gated on `mu > 1.05` so it never lands on the raised curb (which shares the surface with a 0..1 UV). The CityRenderSystem decal group is gone (kept only its centres). Mesher baking is headless-tested (`test_road_net.cpp`); the shader paint is UNVERIFIED (no Metal/Vulkan/WebGPU build here). The zebra band was mirrored into the WebGPU backend's WGSL when main's web renderer merged in (parity discipline), equally unverified. Owed elsewhere: the SDF (`unionRoadbed`) + analytic meshers don't bake the crosswalk `mv` (opt-in paths). | Viewer check on each backend; bake the crosswalk `mv` in the SDF/analytic meshers too if either becomes default |
| AI city cars use kinematic collider proxies (UNVERIFIED) | `apps/citysim/city_physics.cpp`, `engine/physics/physics_world.cpp` (`moveKinematic`) (ADR-0060) | `CityPhysicsSystem` (viewer/editor only — needs Jolt) gives each AI car a KINEMATIC Jolt box that tracks the drawn car pose via `PhysicsWorld::moveKinematic` (Jolt `MoveKinematic`), so the player + physics gun collide with cars and a moving car pushes what it hits. The cars are NOT dynamically simulated (the sim owns their motion) — no suspension/wheel response, and a car can shove a body through a wall since its motion is scripted. The Jolt `moveKinematic` impl is written against the documented API but UNVERIFIED here (Jolt can't build in this env), like the vehicle code. | Full hybrid: near the camera promote AI cars to real Jolt wheeled vehicles fed by the agent brain; verify on a device build |
| Destroyed AI-car colliders leak on count change | `apps/citysim/city_physics.cpp` | `onStop` releases the kinematic bodies, but there is no per-car removal hook mid-session (the pool is rebuilt wholesale if the car count changes) — fine for a fixed car set, same class as the vehicle-destroy leak | Per-car body lifecycle if cars ever spawn/despawn at runtime |
| AI cars can still briefly overlap at merges/turns | `apps/citysim/city_sim.cpp` (ADR-0060) | Car-following now spans NODES (a car keeps its gap to the leader on its continuing route across a junction/bend, chaining over short links), which removed the constant same-direction pile-through. Signals separate perpendicular traffic. What remains is a one-step touch when two cars from different approaches merge/turn onto the same point the same tick (all cars advance from the start-of-step snapshot) — genuine body overlap in only ~2-3% of steps on a single-junction stress test, far less on a real grid. A single-owner junction RESERVATION was tried and REMOVED: it clustered cars at the stop line and made overlap worse. | Sequential (index-ordered) advance or sub-stepping so a follower sees the leader's move within the tick; or a proper turn/merge conflict matrix with yielding |
| AI cars don't react to being shot | `apps/citysim/city_vehicles.cpp` (ADR-0060/0061) | Superseded by ADR-0062: NPC cars are now real dynamic Jolt `Vehicle`s, so a bullet impulse SHOULD shove/rock them like the player's car (low CoM resists flipping). Unverified until the device build runs — confirm the response reads right and doesn't fling cars. | Verify on device; add a small settle/brace so a hit car recovers its lane instead of drifting off |
| Pedestrian avoidance misses knock-down + queueing | `apps/citysim/city_sim.cpp` (ADR-0060/0060) | Peds push apart each step (they step around, not through, each other + the player), steer around their vision-cone neighbours, AND now avoid signal POLES: the render bridge feeds pole foot positions to the sim as static obstacles, which peds lean around and are radially pushed out of so they never stand inside one (`setStaticObstacles`, `kPoleClearance`; covered by `test_city_perception.cpp`). Still missing: longitudinal following (peds queue close at a shared destination) and a knock-down-then-recover state when struck. | Simple ped following, and a ragdoll/knock-down physics state |
| Script entity **destroy** (and component edits) not exposed | `engine/scripting/gameplay_bindings.*` | Spawn is done (deferred command buffer, ADR-0024); destroy/structural edits still need command-buffer ops | Extend the command buffer with destroy + add/remove-component; bullets also need a lifetime/despawn rule |
| `shape:"tree"` inlines a recipe in level JSON | `engine/level_loader.cpp` (`loadTreeEntity`) | Slice to ship a collidable parametric tree; a second authoring path against ADR-0025 | An entity that references a Lua **recipe asset** (ADR-0026); remove the inline `tree` block |
| Tree skeleton discarded after skinning | `engine/procgen/tree.cpp` (`growTree`) | The branch node tree (a natural bone rig) is dropped; output is a static mesh + triangle collider only | A `TreeAsset` with skeleton + skin weights + capsule collision; wind/animation rig (ADR-0026) |
| ~~Forest uses the old SDF tree, not `growTree`~~ | ~~`assets/levels/forest.json`~~ | *Resolved (ADR-0032): `flora.param_tree` grows the real curved tree from a parametric grammar and returns a bark+leaf model; `loadVegetation` scatters N parts as N instance groups with footprint spacing + opt-in per-trunk capsule colliders; `forest.json` uses three collidable param-tree species. Owed: LOD for distant instances.* | Instanced LOD for distant plants |
| Offline path tracer skips scatter + glTF | `src/level_scene.cpp` | The offline tracer now renders procgen terrain and the hero `shape:"tree"` (per-vertex normal/uv/color + alpha-cut leaf cards, for realtime↔offline parity), but not the `vegetation`/`foliage` scatter or glTF `mesh` entities — so `forest.json` renders terrain only offline | Expand scatter instances to triangles (or share the generator) + tessellate glTF offline |
| ~~`ParametricLSystem` not exposed to Lua~~ | ~~`engine/scripting/procgen_bindings.cpp`~~ | *Resolved (ADR-0030): `lsystem.parametric()` (rule/expand with expression successors) + `tree.skin(modules, params, seed) -> bark, leaves` are bound; `growTree` was split into a grammar half and a reusable `skinTree`, so Lua authors the grammar and skins the real curved-cylinder tree. Covered by `procgen_script_skins_a_parametric_tree`.* | — |
| ~~Cosmetic gun model dropped in the Lua port~~ | ~~`src/game/arena_state.cpp`~~ | *Resolved (ADR-0024): `gun.lua` now **generates** the viewmodel with the procgen builders (open in the gameplay VM) and spawns it via `spawn.model` as its own camera-following ScriptBehaviour entity. Covered by `tests/test_gun_script.cpp`.* | — |
| Distant-terrain LOD rings crack at seams | `engine/procgen/terrain.cpp` (`generateTerrainRing`/`generateTerrainLOD`) | Concentric coarsening rings extend terrain to the horizon cheaply (mountains/hills), but adjacent rings differ in resolution, so T-junctions leave hairline cracks at ring boundaries | Vertical skirts at ring edges, or stitch the boundary rows to the finer ring |
| Debug-gizmo overlay draw is BROKEN-ON-DEVICE (Metal), unverified (Vulkan) | `renderer/renderer.h` (`RenderMaterial::FLAG_OVERLAY`), `renderer/metal/metal_renderer.mm` (`depthStateOverlay`), `renderer/vulkan/vulkan_renderer.cpp` (`overlayPipeline`) (ADR-0061/0061) | `FLAG_OVERLAY` should draw marked materials on top with depth test/write OFF (Metal per-batch `depthStateOverlay`; Vulkan `overlayPipeline`). Device evidence says the Metal INSTANCED path doesn't apply it: waist-height overlay hoops were hidden inside body geometry ("visible only peering through geometry"). The debug widgets no longer depend on it (they ground-project with regular depth, like road paint); nothing else uses the flag today. | Debug the Metal instanced depth-state selection (encoder state ordering / pass), verify Vulkan, or replace both with a real line-primitive debug pass |
| Wind sway is height-weighted + instanced-only | `shaders/metal/lighting.metal` (`vertexMainInstanced`), `metal_renderer.mm`, `RenderMaterial::FLAG_WIND` | Cosmetic foliage sway: a vertex displacement weighted by height above the instance origin, self-timed off the wall clock. Only the **instanced** draw path sways (scattered grass + forest trees), so the non-instanced hero `shape:"tree"` leaves don't; it's a uniform field sway, not a per-branch tree rig (ADR-0026). Metal-only — **unverified on Linux/CI**; needs a macOS viewer check. | A real per-branch wind rig for trees; wind on the single-mesh path; expose/author wind params |
| Procedural bark relief is normal-map only | `engine/procgen/tree.cpp` (`barkMaps`), level loaders | Per-species bark (oak furrows / birch lenticels / pine plates) generates an albedo value pattern + a tangent-space **normal map** (no true displacement — silhouette stays smooth). The relief look is **Metal-only, unverified offline** (the path tracer doesn't normal-map). | Parallax-occlusion mapping or tessellated displacement for silhouette; verify in viewer |
| Parking is an informal verge, homes are abstract nodes | `apps/citysim/city_sim.cpp` (`idlePose`, arrival parking) (ADR-0062/0062) | An idle car parks on the grass beside its home/work NODE (off the carriageway so it can't roadblock — correct, but it reads as "failed to be placed" until you know it's an agent at home). Homes/works are raw graph nodes, not the buildings the generator already produces. | Buildings as stops (assign home/work to a parcel; park at its frontage), a painted parking-lane band on residential links (the mesher already does surface bands — same technique as the crosswalk paint), and/or despawn-at-home with the schedule persisting (hybrid: keep parked bodies near the player, cull distant ones) |
| Living-world wasm build is browser-UNVERIFIED | `CMakeLists.txt` (WEB_SOURCES), `src/game/arena_state.cpp` (ADR-0058/0063) | The web target now links the full living-world stack — CitySim + cognition, VehicleSystem, the citysim physics/promotion/walker bridges, Jolt, Lua — and `viewer_web` compiles + links clean under Emscripten 6.0.1 (also fixed: `onelua.c` was in the lua_static glob, latent on lazy desktop linkers, duplicate-symbol fatal under wasm-ld). Never yet RUN in a browser: WebGPU frame, wasm perf with 40 agents + Jolt, and input are unproven. | Load grown/agent_lab in a WebGPU browser; profile; then decide AI-LOD needs for web |
| City generates whole, not streamed | `engine/procgen/city/city.cpp` (`generateCity`) | Phase 3 (ADR-0038) builds a bounded city in one pass (Forest-Arena style); the road graph is city-global, not tile-local | Per-tile generation with the global graph clipped per tile + boundary stitching (ADR-0027 §5 streaming) |
| City impostor/HLOD bake is unbuilt | `engine/procgen/city/`, renderer | The generator emits a coarse per-building proxy + a merged `CityModel::hlodProxy` (headless-tested), but the **impostor-card bake** (render-to-octahedral-atlas) and **HLOD swap/crossfade** are GPU/Metal work, gated on the spatial partition | Impostor bake + LOD-selection/crossfade (ADR-0034 §5); same lever as the owed distant-tree impostors |
| City Lua surface is buildings-only | `engine/scripting/procgen_bindings.cpp` (`building.*`) | Only the split/shape grammar is Lua-exposed; the road→block→parcel pipeline + the city are C++/level-JSON-driven, not yet a Lua **world region recipe** (ADR-0027) | A `roads`/`parcels`/`city` Lua surface so a city is an authorable region recipe like flora |
| City viewer render + natural-scatter masking owed | `engine/procgen/city/city.cpp`, `src/level_scene.cpp` | *The City Arena landed: the city drapes on terrain (foundations at the min ground height under each footprint; roads follow it) with street/park trees, rendered offline (`assets/levels/city_arena.json`).* Still owed: a **Metal viewer** render path, and **suppressing a surrounding forest scatter under the city footprint** (today the city carries its own trees; it doesn't yet mask an external scatter recipe, ADR-0027 §1) | Viewer render path; a city-footprint mask the terrain scatter reads |

---

*Add a new ADR when a decision is hard to reverse, affects multiple modules, or
trades off something a future maintainer would question. Keep the register
current as interim seams are paid down.*
