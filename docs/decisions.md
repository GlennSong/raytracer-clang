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
- The actual gamepad **polling lives in `window.cpp`** (GLFW), which is
  macOS-only and not Linux-compilable — same constraint as the rest of the
  window seam. The engine-side parts (`InputMap` gamepad logic, `PlayerInputs`,
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
**Status:** Accepted (Steps A + C done; step now runs on the shared pool, ADR-0013) · **Date:** 2026-06-04

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
  Jolt's step now runs on the engine's shared `JobSystem` (ADR-0013) via a
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

## ADR-0013 — A minimal shared-queue `JobSystem` for parallelism
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

## Interim seams & tech-debt register

Deliberate shortcuts taken to keep steps small and low-risk. Each is expected
to be replaced; listed here so they stay visible.

| Item | Where | Why interim | Replace with |
|---|---|---|---|
| `RenderView` shared resource | `engine/system.h` | Minimal stand-in for engine resources | A real ECS resource/blackboard (ADR-0006) |
| ~~`MotionSystem` is a physics placeholder~~ | ~~`engine/systems/motion_system.cpp`~~ | *Repositioned (ROADMAP 2.3 Step C): not a placeholder — it is the kinematic mover for cheap, collision-free scripted motion. Dynamics live in `PhysicsSystem` (ADR-0012); MotionSystem yields any entity with a `RigidBody`.* | — |
| ~~Hardcoded keybindings~~ | ~~`engine/systems/dev_control_system.cpp`~~ | *Resolved (ROADMAP 2.1, 2.2): named-action layer `InputMap` (`engine/input/`); `DevControlSystem` and `CameraSystem` (orbit + fly) drive entirely off actions, gamepad included.* | — |
| Incremental logging adoption | various | Avoided a sweep | Migrate remaining `std::cerr` sites |
| ~~No automated tests~~ | ~~repo-wide~~ | *Resolved (ROADMAP 1.3): `tests/` target, `make test` / CTest, 33 cases over `SlotMap`/`SparseSet`/`World`/math/`SimClock`.* | — |
| ~~Legacy `uint32_t` handles~~ | ~~`renderer.h` (`MeshHandle`)~~ | *Resolved (ADR-0007): `MeshHandle`/`BufferHandle` are now `Handle<Tag>`; the Metal backend stores meshes in a `SlotMap<GPUMesh>`. First brick of the asset system.* | — |
| macOS-only verification | `window.cpp`, `metal_renderer.mm` | Only backend; not Linux-compilable | Second backend + CI that can build it |
| Partial live-resize fix | `Window` draw callback | Repaints, but sim is frozen mid-drag | Refresh-driven redraw / resize-aware loop |
| No custom allocators / memory tracking | repo-wide | Deferred (ADR-0008); `SlotMap` covers pooling | Frame arena when churn measured; allocators + tracking on data |
| Core containers not thread-safe; `JobSystem` is single-queue | `slot_map.h`, `sparse_set.h`, `world.*`, `job_system.*` | `JobSystem` (ADR-0013) parallelizes independent work only; ECS/containers stay single-threaded | Container thread-safety / deferred command buffer when ECS systems parallelize; work-stealing when a profile demands it |
| ~~Projection matrices built in backend~~ | ~~`metal_renderer.mm`~~ | *Resolved (ROADMAP 1.2): `Mat4::perspective`/`orthographic` now build the matrices engine-side; backend calls them via `toSimd`. Perspective depth convention fixed to Metal [0,1]; regression-tested.* | — |
| No dedicated 2D camera | `renderer/orbit_camera.*` | Ortho via OrbitCamera as stand-in | A pan/zoom 2D camera when 2D is built |

---

*Add a new ADR when a decision is hard to reverse, affects multiple modules, or
trades off something a future maintainer would question. Keep the register
current as interim seams are paid down.*
