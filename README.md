# Raytracer / Realtime Engine

A from-scratch C++17 renderer and game engine, built with only the standard
library and a handful of vendored single-header/submodule dependencies. It ships
two programs from a shared core:

- **`raytracer`** — an offline CPU path tracer. Renders a scene to a PNG/PPM
  image. Unbiased Monte Carlo with next-event estimation, physical lens model,
  and HDR image-based lighting.
- **`viewer`** — a realtime, playable 3D engine. ECS world, physics (Jolt), Lua
  scripting, procedural generation (terrain, L-system flora, cities, roads), and
  a GPU renderer behind a platform-neutral seam (Metal on macOS; **Vulkan for
  Linux/Windows is in progress** — see [ADR-0057](docs/decisions.md) and
  [docs/vulkan-renderer-plan.md](docs/vulkan-renderer-plan.md)).

The two paths share scene/level loading, materials, lights, and the entire
procedural-generation substrate. They differ only in the renderer — the offline
path traces, the viewer rasterizes — by design (see `AGENTS.md`, "Procgen
Authoring").

---

## Features

### Offline path tracer (`raytracer`)
- Unbiased Monte Carlo path tracing with next-event estimation (explicit light
  sampling) on a job-system thread pool.
- KD-tree acceleration; instancing.
- Materials: diffuse, metal, glass (Fresnel), emissive.
- Lights: directional (sun), point, spot — in physical units (ADR-0017),
  matched to the realtime path.
- HDR environment lighting from equirectangular `.hdr` maps, with dominant-sun
  extraction; ACES display transform.
- Physical lens model (ADR / `docs/virtual-camera-plan.md`): focal length,
  f-stop, focus distance → thin-lens depth of field, barrel/pincushion
  distortion, chromatic aberration.
- Renders procedural scenes (terrain + hero parametric trees) through the same
  generators the viewer uses.
- Output: PNG (via bundled `stb_image_write`), or P3 PPM with `--out scene.ppm`.

### Realtime engine (`viewer`)
**Rendering** (current Metal backend; Vulkan targets 1:1 parity):
- Forward PBR shading — albedo / metallic / roughness / emission, plus albedo,
  normal, metallic-roughness, emissive, and AO texture maps; per-vertex tint.
- An analytic procedural-surface library (brick, concrete, stucco, roof
  tile/shingle, corrugated metal, asphalt, pavement, cobblestone, wood siding,
  road lane markings) selected per-material with no texture maps.
- Lights: directional / point / spot in physical units; cascaded shadow maps
  with PCF and artistic tint/strength.
- Environment: procedural analytic sky with day/night cycle and FBM clouds,
  **or** a baked cubemap from an equirectangular HDR; image-based lighting
  (irradiance + prefiltered specular + BRDF LUT); reflection probes with
  parallax correction.
- Screen-space post: SSAO (temporal), SSR, bloom, tone mapping (ACES / AgX) +
  color grade, lens effects (distortion / chromatic aberration / vignette) and
  depth of field; aerial-perspective fog.
- Instanced rendering (vegetation/scatter) with height-weighted wind sway;
  CDLOD terrain with vertex morphing; alpha-tested foliage with a depth prepass.
- Debug views (AO/SSR/depth/normals/shadow/albedo/facing), wireframe overlay,
  stats HUD.

**Engine**:
- Sparse-set ECS world; fixed-timestep simulation with interpolation (ADR-0002).
- Jolt-backed rigid-body physics (cross-platform, headless-testable).
- Lua scripting layer (`ScriptVM`, ADR-0023) — procedural recipes authored as
  data and hot-reloadable.
- Procedural generation: noise/terrain, L-system botany, SDF tools, parametric
  trees, a road network (spline graph → polygon join engine), and city
  generation.
- Input: keyboard/mouse + gamepad (Apple GCController on macOS, GLFW joystick
  elsewhere); named-action input mapping.
- Optional Dear ImGui debug overlay; optional Qt6 editor app.

---

## Build

### Prerequisites
- **C++17 compiler** — `clang++` is the project standard (`g++` and MSVC also
  work). 
- **CMake ≥ 3.16** and a build tool (`make` or Ninja) for the viewer.
- **Git submodules** (Jolt, Dear ImGui, Lua):
  ```bash
  git submodule update --init --recursive
  ```
- **Per platform, for the `viewer`:**
  - **macOS:** Xcode command-line tools (Metal/MetalKit/Cocoa are part of the
    SDK). GLFW (`brew install glfw`).
  - **Linux:** GLFW + the Vulkan SDK and X11/Wayland dev headers, e.g. on
    Debian/Ubuntu:
    `sudo apt install cmake clang libglfw3-dev libvulkan-dev vulkan-tools glslang-tools spirv-tools`
  - **Windows:** the [Vulkan SDK](https://vulkan.lunarg.com/) (headers, loader,
    `glslc`) and GLFW via [vcpkg](https://vcpkg.io) (`vcpkg install glfw3`). Build
    with **clang** (CMake defaults to MSVC, which rejects the project's `-Wextra`).
    The Qt **editor** additionally needs `vcpkg install qtbase`. Full recipe —
    toolchain, configure, running the viewer + editor — in
    **[docs/windows-build.md](docs/windows-build.md)**.

> **Note:** the Vulkan backend is under construction (ADR-0057). On non-Apple
> platforms today the `viewer` links a `NullRenderer` (window + UI run, nothing
> is drawn). The build commands below are the same once Vulkan lands; what you
> install now (Vulkan SDK + GLFW) is what it will use.

### Offline path tracer (no GPU needed — builds anywhere)
The `Makefile` builds the tracer and the unit tests with just `clang++`:
```bash
make            # debug build  -> ./raytracer
make release    # optimized build
make test       # Jolt-free unit tests (math, ECS, input, camera)
make clean
./raytracer                 # renders output.png
./raytracer --out scene.ppm # P3 PPM instead
```
(The tracer also builds via CMake as the `raytracer` target.)

### Full build — viewer, physics, tests (CMake)
The **same commands on every platform**; CMake picks the GPU backend per OS
(Metal on Apple, Vulkan elsewhere once it lands, `NullRenderer` as fallback):
```bash
git submodule update --init --recursive
cmake -S . -B build
cmake --build build
ctest --test-dir build          # unit + physics tests
./build/viewer                  # boots into play; --edit for edit mode
./build/viewer --play level.json
```

### Build options
| Option | Default | Effect |
| --- | --- | --- |
| `RT_ENABLE_PHYSICS` | `ON` | Jolt physics + its headless tests (cross-platform). |
| `RT_ENABLE_SCRIPTING` | `ON` | Lua scripting layer (pure C, cross-platform). |
| `RT_ENABLE_IMGUI` | `OFF` | Dear ImGui debug overlay. |

The `viewer` target builds only where GLFW is found. A Qt6 install additionally
enables the `editor_app` target.

---

## Project layout

```
src/
  rt_math, handle, slot_map, log, image, camera, geometry, material, scene,
  kdtree, path_tracer …   — shared core + offline tracer
  renderer/               — RHI seam (renderer.h), window/input, GPU backends
    metal/                — Metal backend (macOS)         [see metal/AGENTS.md]
    vulkan/               — Vulkan backend (Linux/Windows) [see vulkan/AGENTS.md]
  engine/                 — ECS, systems, physics, scripting, procgen
shaders/
  metal/                  — Metal Shading Language sources
  vulkan/                 — GLSL → SPIR-V (planned)
docs/
  decisions.md            — Architecture Decision Records (ADRs)
  ROADMAP.md              — multi-tier development plan
  *-plan.md               — per-feature design docs
```

## Documentation
- **`AGENTS.md`** — engineering standards, conventions, and the load-bearing
  engine rules (platform abstraction, procgen authoring, playable scenes).
- **`docs/decisions.md`** — every significant architectural decision, with
  alternatives and revisit triggers.
- **`docs/ROADMAP.md`** — where the project is going.
- Per-area `AGENTS.md` files (e.g. `src/renderer/AGENTS.md`,
  `src/renderer/metal/AGENTS.md`) — focused guides to a subsystem's internals.

## Conventions (short version)
C++17, clang++, standard library only — no *new* external deps without an ADR.
PascalCase types, camelCase functions/variables, UPPER_SNAKE constants, no
Hungarian notation. Smart pointers for heap ownership. One header + one `.cpp`
per module. Front faces wind **clockwise**. See `AGENTS.md` for the full set.
