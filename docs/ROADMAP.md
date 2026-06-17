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
**Status:** Done — engine input layer, GLFW polling, and GCController backend
all verified on macOS 15.5 with an Xbox Series controller over USB.
See ADR-0010 for the engine/game boundary, ADR-0013 for the GCController macOS
gamepad backend.
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
- ✅ `window.cpp` polls GLFW gamepads (IOKit path) and emits connect/disconnect
  events; `gamepad_gc.mm` provides a GCController overlay for controllers that
  macOS claims via DriverKit (Xbox/PS on macOS 13+). See ADR-0013.
- ✅ `gamecontrollerdb.txt` (SDL_GameControllerDB) loaded at init for GLFW's
  IOKit fallback path.
- ✅ Verified on macOS 15.5 with Xbox Series controller (USB): left stick moves,
  right stick looks, triggers up/down, bumper boosts, Back/Start toggle modes.

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

**This tier is the procgen substrate (ADR-0021).** Every generator is
`(parameters, seed) → content`, where `content` is one of a few value types:
`Mesh` (3.3), `Material`/`Texture` (3.2), `Field` (Rⁿ→scalar — SDF/noise, Tier
4), all owned by the asset manager (3.1). So 3.1/3.2/3.3 plus a noise library
(3.7) are not a precursor to procgen — they are the value types it emits. Build
them first; Tier 4 stands on them.

### 3.1 Asset / resource system
**Status:** In design — handle migration done (`MeshHandle`/`BufferHandle` →
`Handle`/`SlotMap`, ADR-0007); the `AssetManager` design is in
`docs/asset-system-plan.md` (owns GPU resource lifetime, dedup, refcounting,
async seam). It is the keystone of the procgen substrate (ADR-0021) and the fix
for the editor mesh-leak tech debt. **Next to build.**
**Why:** The engine needs a unified way to create, own, and reference meshes,
textures, and materials — whether loaded from disk or generated at runtime.

**Scope:**
- ✅ Migrate `MeshHandle` / `BufferHandle` from `uint32_t` to `Handle`/`SlotMap`
  (ADR-0007). Done: distinct-tag `Handle` types; the Metal backend stores meshes
  in a `SlotMap<GPUMesh, MeshTag>` with generation-checked handles. Engine side
  headless-verified; backend storage swap macOS-only.
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

### 3.4 Virtual camera system
**Status:** Planned — see `docs/virtual-camera-plan.md` for the full phased plan.
**Why:** Placeable camera entities (ECS) with switchable viewports and a
physical lens model (focal length, aperture/DOF, distortion, chromatic
aberration) turn the engine into a virtual-filming tool: frame a shot with the
fly camera, place a camera there, look through it live, and render it offline
for ground truth. Late-stage milestone: drive a camera's pose from an iPhone
(ARKit) through an external pose seam.

**Scope (summary):**
- `SceneCamera` + `LensParams` components; pose from the entity's `Transform`.
- `CameraSystem` view-source selection: editor controllers vs. placed cameras,
  with cycling, place-at-current-view, and ImGui camera panel.
- Thin-lens DOF/distortion/CA in the offline tracer; post-process
  approximations in the Metal viewer.
- Level JSON persistence for placed cameras.

**Depends on:** Nothing hard; ImGui (1.1) for the panel, level format (2.3-era
loader) for persistence.

### 3.5 Edit mode (in-engine level editor)
**Status:** Planned — see `docs/edit-mode-plan.md` for the full phased plan.
**Why:** A Blender-style editing loop — simulation stopped, free editor view,
click-to-select, transform gizmos, an Add menu (primitives / glTF / cameras),
then a Play button straight into the running game. The level JSON becomes the
document: Play saves-then-loads it, so every playtest starts from exactly what
was built. Most infrastructure exists (state stack, editor cameras, MeshBuilder,
glTF import, inspector pattern, JSON persistence); the genuinely new pieces are
a level writer + authoring metadata, mouse picking, and gizmos (vendored
ImGuizmo behind the ADR-0011 pattern).

**Depends on:** ImGui (1.1), level format. Unblocks comfortable authoring for
procgen tiers (placing generators, tuning scenes).

### 3.6 Editor application (native shell around the engine)
**Status:** Planned — see `docs/editor-app-plan.md`.
**Why:** ImGui is right for in-engine tooling but wrong as the authoring
surface. A Unity/Blender-style application — native hierarchy/inspector/asset
panels around an engine-rendered viewport, Play running the game in-viewport
or as a separate process — makes level building comfortable. 1:1 fidelity by
construction: one `engine_core` library, two hosts. Phase A1 (engine-as-
library + an embedded-window seam) is framework-agnostic and Linux-testable;
the shell is **Qt 6** — the engine is cross-platform by intent (Vulkan
backend for PC/Linux planned, Tier 5), so the editor must be too. A1's
engine_core library is in place.

**Depends on:** Edit mode (3.5) — its document model, picking, and gizmos are
the engine half of this application. Feeds the asset system (3.1): import +
cooking live in the editor's asset browser.

---

### 3.7 Noise library
**Status:** Not started
**Why:** Perlin / Simplex / value noise + FBM + domain warp. Foundational to
terrain, procedural textures, and clouds; tiny, pure math, no deps,
Linux/CI-testable. Slotted into Tier 3 (not 4) because it is a value-type
building block, not a generator. Seedable (ADR-0021/0002).

**Depends on:** Nothing. Pairs with the mesh builder (3.3).

### 3.8 Curve library
**Status:** Phase 1 done — `Spline<T>` Hermite kernel (Catmull-Rom, eval/
derivative, arc-length, rotation-minimizing frames) in `src/curve.{h,cpp}`,
covered by `tests/test_curve.cpp`; first consumer is the curved tree branch
sweep (`growTree`). `AnimCurve` (F-curves) and SVG import remain (ADR-0031).
**Why:** One curve primitive serves procgen (branch centerlines for the organic
branches of ADR-0029 §3.5), animation (F-curves), and 2D vector / SVG import.
Slotted into Tier 3 as a value-type building block alongside noise (3.7), not a
generator. Pure math, no deps, Linux/CI-testable.

**Decision (ADR-0031):** split a **math kernel** from its consumers. Kernel is a
piecewise-cubic `Spline<T>` for `T ∈ {float, Vec2, Vec3}`, stored as Hermite
knots (value + in/out tangent) — the canonical form Catmull-Rom, cubic Bezier,
and keyframes all lower to; exposes `eval`/`tangent`. Per-consumer services layer
on top in their own modules: `Path3` (arc-length LUT + rotation-minimizing frame,
procgen), `AnimCurve` (time + tangent modes, animation), `Path2` + SVG parser
(asset loading). **Phase 1 (now):** kernel + Catmull-Rom + arc-length + RMF,
which immediately gives `growTree` continuous curved branches (ADR-0029 §3.5).
AnimCurve and SVG follow when their domains need them.

**Depends on:** Nothing. First consumer is the tree branch sweep (Tier 4 / B.1).

---

## Tier 4 — Procedural Generation

The creative core. **Strategy is fixed by ADR-0021:** build a C++ *library of
composable generators* over the Tier 3 value types (`Mesh`/`Field`/`Material`/
`Frame`), deterministic from a seed; distill a node graph / DSL only after
several generators expose what it must express; pursue geometric modeling via
SDF/implicit functions, not a B-rep kernel. The phases below sequence the
substrate before the language before the big applications.

### Phase A — Generator substrate
A deterministic evaluation context (seeded RNG streams, parameter binding) and
the `Field` value type alongside the Tier 3 `Mesh`/`Material`. A `Field` is
Rⁿ→scalar — the home of noise (3.7) and SDFs — sampleable, maskable, and
meshable.

- **A.1 SDF / implicit modeling + mesher.** CSG via min/max, smooth blends via
  smooth-min; mesh a field to a `Mesh` (marching cubes first, dual contouring
  later for sharp features). The in-house, robust analog to a Plasticity/
  Parasolid B-rep kernel (ADR-0021) — and the basis for organic shapes,
  terrain, and clouds. **Prioritized (June 2026 feedback):** it is the clean fix
  for *welded* organic geometry — kit-bashed L-system cylinders are disjoint and
  self-intersecting; skinning branches as smooth-min'd capsules and meshing the
  field yields one continuous surface. It is also the heart of "a rock is a
  generator graph": a rock becomes a few SDF ops with tunable params, not
  rock.cpp.

### Phase B — Three generators, one substrate
Implement one of each paradigm on the shared value types, to surface what they
truly share before any language:
- **B.1 L-systems / grammar** — parametric, stochastic, context-sensitive;
  turtle-interpreted to a `Mesh`. Trees/bushes/coral first; the path to
  split/shape grammars for buildings (CityEngine-style) later.
- **B.2 Noise heightfield terrain** — FBM/domain-warp (3.7) → heightfield →
  `Mesh`; biome assignment drives `Material` selection. Erosion (hydraulic/
  thermal) and chunked LOD as it scales. Physics (2.3) can drive erosion.
- **B.3 SDF CSG shape** — a small modeling example over Phase A.1.
- **B.4 Scatter / distribution** — the `Frame` generator: place instances over a
  surface or volume with seeded density rules (slope, altitude, a noise mask),
  emitting a set of transforms. The bridge between terrain (Field/Mesh), the
  asset meshes, and instanced rendering. This is where the value types compose.

**Instanced rendering (pulled forward from Tier 5) — done.** The substrate's
whole payoff is "thousands of the same mesh" (forests, fields, fleets), so
instancing is a Phase B prerequisite, not a late optimization. Implemented: an
`InstanceGroup` component (shared `MeshHandle` + baked world matrices) and a
`Renderer::drawMeshInstanced` seam (default loops `drawMesh`; the Metal backend
coalesces by mesh handle into instanced draws). `loadVegetation` emits one group
per species. Coarse group-cull only for now; per-instance/chunk culling is Tier 5.
See `docs/forest-arena-plan.md`.

### Phase B milestone — "The Forest" arena
The integration target that proves Phases A–B end to end and exercises every
value type at once: a procedural heightfield terrain, slope/altitude-based
material, L-system trees + noise-displaced rocks generated into the asset
manager, scattered by the thousands with sensible density and drawn instanced,
under an HDR sky. Full plan: `docs/forest-arena-plan.md`. Most of the pipeline
is headless/CI-testable; only the final render needs macOS.

**Flora is now authorable in Lua** (ADR-0023/0024): `assets/scripts/flora.lua`
provides `flora.tree/rock/grass/flower` over the procgen builders — stochastic,
upward-tapering trees with **real leaf cards** (not SDF blobs), three species,
plus rocks/grass/flowers. The level loader accepts a `{ "kind":"script" }`
vegetation species (inline Lua or a `.lua` path), so Lua-generated flora scatters
in alongside the C++ tree/rock species; `assets/levels/forest.json` uses it. A
second `foliage` block runs an independent, denser scatter pass for ground cover
(grass/flowers) with a low `maxSlopeDeg`, so it lands on the gentle, green ground
(`terrainColor` reads steep slopes as rock). Generation is headless-tested
(`tests/test_flora.cpp`); the look needs macOS.
Supporting bindings: `lsystem.segments/leaves`, `mesh.orient`, `sdf.smooth_union_all`,
and a `TurtleParams.taper` for continuous trunk thinning.

### Phase C — The procgen language (distilled, not designed up front)
Only after Phase B. A text authoring layer over the value types
(`Mesh`/`Field`/`Frame`/attributes), distilled *from* what the Phase B
generators shared, per ADR-0021.

The language is **Lua** (ADR-0023): one embedded VM with separate binding
surfaces, built **procgen-first** (the pure/deterministic substrate, callable
later by the effectful gameplay surface), sealed behind a Jolt-style `ScriptVM`
(no `lua_*` types in headers). This also lands the engine's general scripting
layer for gameplay (ADR-0024).

A node-graph evaluator was prototyped (Phases 1–3) as a parallel *visual*
front-end but **removed** (ADR-0025): Lua became the path actually used and the
graph never grew a canvas or a graph↔Lua bridge. Lua is now the single procgen
authoring path; any future visual editor should emit Lua rather than be a second
evaluator.

### Phase D — Applications (by appetite)
Each composes Phases A–C:
- **Procedural textures/materials** — noise-driven synthesis, weathering,
  variation feeding the material system (3.2).
- **Voxel terrain** — volumetric representation (caves/overhangs/destruction);
  meshed via Phase A.1; commits to chunking + LOD (Tier 5).
- **Procedural clouds** — volumetric FBM, raymarched (renderer-side).
- **City / road layout** — split-grammar buildings, road networks (L-systems /
  tensor fields / agent-based), lot subdivision. Converges B.1 + B.2 + meshing.
- **Procedural planet** — cube-sphere quadtree LOD + spherical terrain +
  atmosphere. The capstone.

### Separate track — temporal generators
Particles and bullet patterns share the seeded-RNG + parameter substrate but
are **simulation, not static geometry** (ADR-0021); they live in their own
subsystem rather than the mesh/field pipeline.

---

## Tier 5 — Scale & Polish (future)

Items that become relevant as the world grows large.

- **Open-world foundations** — a **bounded, curated** ~16 km world (GTA V / Horizon
  model, single precision — *not* infinite Minecraft), where the small-world
  assumption breaks distant terrain today. Phased: (0) reverse-Z depth + robust
  sky/background classification + re-enable culling; (1) spatial partitioning
  (sector grid/BVH) + chunked terrain with tight bounds + terrain LOD; (2) object
  mesh LOD + foliage impostors; (3) sector streaming + HLOD/building impostors;
  later, occlusion culling. The infinite/planetary apparatus (camera-relative
  rendering, floating-origin rebasing) is the documented upgrade path, not on this
  path. See `docs/open-world-foundations-plan.md` and ADR-0034; pairs with the
  ADR-0027 content model (fields + recipes + per-tile overrides).
- **Scene graph / spatial indexing** — BVH or octree for frustum culling and
  efficient queries over large generated worlds.
- **LOD system** — distance-based level of detail for procgen meshes and
  terrain chunks.
- **Instanced rendering** — *done (Tier 4 Phase B):* an `InstanceGroup` component
  (mesh + baked world matrices) + a `Renderer::drawMeshInstanced` seam; scattered
  vegetation collapses into one group per species instead of an entity per plant.
  The default `drawMeshInstanced` loops `drawMesh`, which the Metal backend
  already coalesces into instanced draws. *Per-instance LOD and chunk culling for
  huge worlds remain here* (group bounds are coarse today), plus growing the
  `MAX_INSTANCES` buffer / a direct Metal instanced path.
- **Mixed precision / large world coordinates** — revisit ADR-0005 when float
  positions lose precision at world scale.
- **Second rendering backend (Vulkan)** — validate the platform abstraction
  (ADR-0001).
- **Multithreaded systems** — parallel system execution, job system for
  procgen workloads. *Foundation done: a minimal shared-queue `JobSystem`
  (`src/job_system.*`, ADR-0014) — `parallelFor` + counter-based `run`/`wait`,
  synchronous mode for tests; the offline tracer renders on it. Pulled forward
  from Tier 5 as low-level foundation. **Jolt physics now steps on this pool**
  via a `JoltJobAdapter` (ADR-0012). Remaining: parallel ECS system execution
  (needs container thread-safety).*
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
