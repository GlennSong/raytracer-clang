# Raytracer Project — Agent Guidelines

## Project Overview

A portable C++ raytracer built from scratch using only standard libraries.
Compiled with clang++ targeting C++17. **No *new* third-party dependencies** —
single-header/vendored libraries already in `third_party/` and built (Jolt, Dear
ImGui, tinygltf, the `stb_image`/`stb_image_write` headers it bundles, and Lua —
the scripting VM, ADR-0023) are accepted; prefer them over hand-rolling
equivalents (ADR-0016). Do not add new submodules or external libraries without
an ADR.

Significant architectural decisions — with their alternatives, trade-offs, and
revisit triggers — are recorded in `docs/decisions.md`. Add an ADR there when a
decision is hard to reverse or spans multiple modules.

## Architecture

```
src/
  rt_math.h / rt_math.cpp   — Vec3, Mat4, Quat, Ray, all linear algebra (Real scalar)
  handle.h                   — Handle<Tag>: type-safe recyclable IDs
  slot_map.h                 — SlotMap<T>: recycling storage, stale-handle detection
  log.h / log.cpp            — leveled logging (LOG_INFO / LOG_WARN / LOG_ERROR)
  check.h                    — ASSERT (debug-only) / CHECK (always-on) macros
  image.h / image.cpp        — Image buffer and PPM output
  camera.h / camera.cpp      — Camera model and ray generation
  geometry.h / geometry.cpp  — Shapes (Sphere, Triangle), hit records
  material.h / material.cpp  — Materials and BRDFs
  scene.h / scene.cpp        — Scene graph, object list
  kdtree.h / kdtree.cpp      — KD-tree acceleration structure
  main.cpp                   — Entry point, render loop
```

The interactive viewer adds a renderer and engine layer on top of the shared
math/core modules:

```
src/
  renderer/
    renderer.h               — RHI: backend-agnostic rendering interface
    event.h                   — Backend-neutral window/input events (incl. gamepad)
    gamepad.h                 — Backend-neutral gamepad types (buttons/axes/state)
    gamepad_gc.h / .mm        — GCController backend (macOS); no-op elsewhere (ADR-0013)
    window.h / window.cpp     — Windowing + input (GLFW), the platform boundary
    settings.h / settings.cpp — Persisted viewer settings
    metal/metal_renderer.mm   — Metal backend implementation (macOS)
  engine/
    clock.h / clock.cpp       — Fixed-timestep simulation clock
    world.h / world.cpp       — Entity / Transform world model (sparse-set ECS)
    components.h / .cpp       — ECS components (Transform, Renderable, Velocity, etc.)
    application.h / .cpp      — Application spine + system scheduler
    system.h                  — System base class + FrameContext
    input/
      input_map.h / .cpp      — Named-action input mapping layer
      player_input.h / .cpp   — Per-player input routing + device assignment
    camera/
      camera_controller.h     — CameraController seam (orbit/fly)
      orbit_camera_controller.h / .cpp
      fly_camera_controller.h / .cpp
    physics/
      physics_world.h / .cpp  — Jolt wrapper (pimpl, Jolt-free header)
    systems/
      camera_system.h / .cpp  — Camera input bindings + controller switching
      dev_control_system.h / .cpp — Pause/quit/time-scale/exposure
      motion_system.h / .cpp  — Kinematic mover (Velocity-driven)
      physics_system.h / .cpp — Jolt-driven rigid body simulation
      render_system.h / .cpp  — ECS → RenderView bridge
      debug_overlay_system.h / .cpp — ImGui overlays (inert without RT_ENABLE_IMGUI)
  viewer_main.cpp             — Interactive viewer entry point
```

Each module is one header + one implementation file. Keep includes minimal —
a module should only include what it directly uses.

## Platform Abstraction (Engine Rule)

The engine defines coherent, backend-neutral interfaces; OS-, windowing-, and
graphics-backend specifics are implemented *behind* those interfaces — one
implementation per platform, selected at build time. This rule is load-bearing:
it is what lets new platforms be added by writing an implementation rather than
by threading conditionals through shared code.

- Engine and application code MUST NOT reference a windowing library, OS, or
  graphics-backend symbol directly. No `GLFW_*`, no `NSWindow`/`HWND`, no
  Metal/Vulkan types outside the file that implements that seam.
- The seams are `Renderer` (rendering backend) and `Window` (windowing + input).
  They expose only backend-neutral types.
- Anything crossing a seam must be backend-neutral: events use our own
  `Event` / `KeyCode` / `MouseButton` (never GLFW codes); native window handles
  cross as opaque `void*`; etc.
- Platform-specific code is confined to its own file or a single `#ifdef` branch
  inside the seam's implementation — e.g. `metal/metal_renderer.mm`, or the
  native-handle branch in `window.cpp`. Adding a backend means adding an
  implementation of the seam, not editing call sites.

## Procgen Authoring (Engine Rule)

Everything the engine can generate — meshes, materials, terrain, props,
buildings, whole scenes — MUST eventually be authorable from Lua over a bound
C++ vocabulary (ADR-0042). The script holds the *recipe* and the tunable
parameters; C++ holds the hot *substrate*. This rule is load-bearing: it is what
lets entire procedural scenes be built, tuned, and hot-reloaded as data without
recompiling, and lets one generative vocabulary serve every procgen project.

- **The boundary.** C++ owns the performance-critical, stable substrate — mesh
  ops, grammar/L-system interpreters, solvers (road/parcel, etc.), noise,
  instancing/BLAS/TLAS. Lua owns the recipes: *what* a thing looks like,
  placement/zoning rules, and every tunable number. A solver is an algorithm, not
  a recipe; a "what a street lamp looks like" is a recipe, not C++.
- **Single source of truth.** A Lua binding MUST wrap the *same* C++ builder the
  engine uses, never a fork of its geometry. Exposing a part parametrises the
  existing builder; it does not duplicate it.
- **One pipeline, one divergence.** Generated data flows through the *same*
  engine pipeline regardless of output. The only legitimate divergence between
  the offline path tracer and the realtime renderer is the renderer itself —
  both consume the same meshes, materials, and instances.
- **Composability.** Recipes return a common model value (parts + instances +
  colliders + attach points) that nests, so the same call is valid for an
  individual part, a collection, or a whole scene — or anything in between.

## Engineering Ethos (Engine Rule)

These are tools we are building for the long term, not a throwaway project.
Invest in making them powerful and expressive.

- **Prototype, then build it right.** A prototype exists to prove an idea. Once
  proven, take the time to make it proper — parametric, single-source-of-truth,
  tested, documented (an ADR when it is a real decision) — before moving on. Do
  not leave prototype-quality code in place as if it were finished.
- **Design for extension and reuse.** Prefer interfaces and vocabularies that
  other parts of the engine (and future procgen projects) can build on, over a
  one-off that solves only today's case.

## Coding Standards

### Naming Conventions
- **Classes / Structs / Enums**: `PascalCase` — `Vec3`, `Ray`, `Material`, `KdTree`
- **Functions / Methods**: `camelCase` — `normalize()`, `intersect()`, `loadObj()`
- **Variables / Parameters**: `camelCase` — `hitPoint`, `lightDir`, `maxDepth`
- **Member variables**: `camelCase`, no prefix — `origin`, `direction`, not `m_origin`
- **Constants / Enum values**: `UPPER_SNAKE_CASE` — `MAX_BOUNCES`, `IMAGE_WIDTH`
- **File names**: `lowercase` — `math.h`, `camera.cpp`

### No Hungarian Notation
Do not prefix variables with type indicators (`fValue`, `pNode`, `iCount`).
Name variables by what they represent, not what type they are.

### Memory Management
- Use `std::unique_ptr` for sole ownership (scene objects, KD-tree nodes).
- Use `std::shared_ptr` only when ownership is genuinely shared.
- Raw `new` / `delete` only inside custom allocators or pool structures.
- Prefer stack allocation for small, short-lived objects (Vec3, Ray, etc.).
- RAII everywhere — no manual cleanup in destructors if smart pointers suffice.

### Code Organization
- Keep modules self-contained: one header + one .cpp per logical unit.
- Minimize header includes; forward-declare where possible.
- All math types and operations live in `math.h` / `math.cpp`.
- Use `Real` (from `math.h`) for engine math scalars rather than raw `double`
  or `float`; the precision choice is centralized there (see ADR-0005).
- No `using namespace std;` in headers. Acceptable in .cpp files.

### Style
- Braces on same line for functions and control flow.
- 4-space indentation, no tabs.
- Keep functions short and focused.
- No comments that restate the code. Comment only when the *why* is non-obvious.
- Const-correct: mark parameters and methods `const` where appropriate.

### Build
- Makefile with clang++, C++17 standard.
- Warnings: `-Wall -Wextra -Wpedantic`
- Debug build: `-g -O0`
- Release build: `-O2`
- Output binary: `raytracer`

## Render Pipeline (Target)

1. Parse scene (OBJ files, camera, lights)
2. Build acceleration structure (KD-tree)
3. For each pixel: generate camera ray → trace → shade → accumulate
4. Path tracing with configurable bounce depth and samples per pixel
5. Write output as square PPM image
