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

**Consequences / tech debt.** Two handle styles coexist until migration: the
new `Handle` and the legacy `uint32_t` `MeshHandle`/`BufferHandle` in
`renderer.h`.

**Revisit trigger.** Building the ECS and asset manager — migrate existing
handle usages then.

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
- **Projection-matrix construction still lives in the backend**
  (`metal_renderer.mm`): perspective and now orthographic. Each future backend
  re-implements it, and the math is not Linux-compile-verifiable. Moving it
  engine-side (into `Mat4`) is the eventual cleanup.
- No 2D camera controller yet; `OrbitCamera` in ortho mode is the stand-in.
- The pre-existing **perspective** matrix uses the OpenGL [-1,1] depth
  convention on a Metal [0,1] renderer; it survives only because the scene sits
  far enough that the perspective divide lands NDC z in [0,1]. Geometry very
  close to the camera would be wrongly clipped. The ortho matrix targets [0,1]
  directly; perspective should be made consistent in the eventual cleanup.

**Revisit trigger.** Adding a second backend (de-duplicate projection math, or
move it engine-side), or building real 2D (a dedicated pan/zoom camera).

---

## Interim seams & tech-debt register

Deliberate shortcuts taken to keep steps small and low-risk. Each is expected
to be replaced; listed here so they stay visible.

| Item | Where | Why interim | Replace with |
|---|---|---|---|
| `RenderView` shared resource | `engine/system.h` | Minimal stand-in for engine resources | A real ECS resource/blackboard (ADR-0006) |
| `MotionSystem` does simple Euler integration | `engine/systems/motion_system.cpp` | Placeholder kinematics, not real physics | A physics/collision system (Step 6) |
| Hardcoded keybindings | `engine/systems/dev_control_system.cpp` | No input-mapping layer yet | An input-action mapping abstraction |
| Incremental logging adoption | various | Avoided a sweep | Migrate remaining `std::cerr` sites |
| No automated tests | repo-wide | No harness yet | A `tests/` target (esp. for `SlotMap`, math) |
| Legacy `uint32_t` handles | `renderer.h` (`MeshHandle`) | Pre-`Handle` primitive | `Handle`/`SlotMap` (ADR-0007) once assets land |
| macOS-only verification | `window.cpp`, `metal_renderer.mm` | Only backend; not Linux-compilable | Second backend + CI that can build it |
| Partial live-resize fix | `Window` draw callback | Repaints, but sim is frozen mid-drag | Refresh-driven redraw / resize-aware loop |
| No custom allocators / memory tracking | repo-wide | Deferred (ADR-0008); `SlotMap` covers pooling | Frame arena when churn measured; allocators + tracking on data |
| Projection matrices built in backend | `metal_renderer.mm` | Working perspective path lives there (ADR-0009) | Move projection math engine-side (`Mat4`) |
| No dedicated 2D camera | `renderer/orbit_camera.*` | Ortho via OrbitCamera as stand-in | A pan/zoom 2D camera when 2D is built |

---

*Add a new ADR when a decision is hard to reverse, affects multiple modules, or
trades off something a future maintainer would question. Keep the register
current as interim seams are paid down.*
