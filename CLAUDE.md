# Raytracer — Claude Code Context

## Build & Run

```bash
make            # debug build (offline path tracer)
make release    # optimized build
make clean      # remove build artifacts
./raytracer     # renders to output.png (PNG via stb_image_write; --out x.ppm for P3)
make test       # Jolt-free unit tests (math, ECS, input, camera)
```

The interactive viewer and physics build via CMake. Jolt, Dear ImGui, Lua,
Tracy, and miniaudio are git submodules — fetch them first (re-run after pulling commits that
add a new submodule; CMake also tries to auto-init them):

```bash
git submodule update --init --recursive    # fetch third_party/{JoltPhysics,imgui,lua,tracy,miniaudio}
cmake -S . -B build-viewer && cmake --build build-viewer
ctest --test-dir build-viewer               # runs unit + physics tests
./build-viewer/viewer                       # the game (boots into play; --edit for edit mode)
./build-viewer/editor_app                   # the Qt editor (builds wherever Qt6 is found)
```

**Do not configure CMake into `build/`.** The Makefile above owns it
(`BUILD_DIR = build`), so on any box where `make` has run, `cmake --build build`
fails with `Error: not a CMake build directory (missing CMakeCache.txt)`. Any
`build-*/` name is gitignored; `build-viewer/` is the convention.

The viewer target builds only where GLFW is found (e.g. macOS); physics
(`-DRT_ENABLE_PHYSICS=ON`, default) is cross-platform and builds/tests headless.
The Dear ImGui debug overlay (`-DRT_ENABLE_IMGUI=ON`, default) is toggled
in-app with the tilde/grave key. The Lua
scripting layer (`-DRT_ENABLE_SCRIPTING=ON`, default) is pure C — cross-platform,
builds/tests headless — sealed behind `ScriptVM` (ADR-0023); the procgen binding
surface is covered by `tests/test_script_vm.cpp`. Audio
(`-DRT_ENABLE_AUDIO=ON`, default) is miniaudio (submodule) sealed behind
`AudioEngine` (ADR-0069) — device-optional, tested headless via a pumped mix
(`tests/test_audio.cpp`). `-DRT_ENABLE_PROFILER=ON` compiles the Tracy client
(ADR-0068); the `RT_PROFILE_*` macros in `src/profile.h` are no-ops without it.
Every build also carries the always-on frame ledger (ADR-0077):
`RT_FRAME_STATS=<csv>` captures per-frame timings on any host,
`tools/frame-report.py` renders the capture, `make health` scans for
duplicated/patchy code — workflow in `docs/profiling.md`.

Gamepad support uses Apple's GCController framework on macOS (ADR-0013) for
Xbox/PS controllers, with GLFW's IOKit path as fallback. A
`gamecontrollerdb.txt` (SDL_GameControllerDB) is loaded at init for the IOKit
path. See `src/renderer/gamepad_gc.mm`.

Other targets: `docs/visionos-build.md` (Apple Vision Pro — simulator +
device, same MetalRenderer behind the PresentationSurface seam;
`src/visionos_app/AGENTS.md` has the display contract) and
`docs/web-build.md` (WebGPU/WASM via Emscripten).

## Planning

See `docs/ROADMAP.md` for the multi-tier development plan (foundation → 3D
infrastructure → content pipeline → procedural generation). Architecture
decisions live in `docs/decisions.md`.

`docs/knowledge-retention-plan.md` is queued work on the harness itself: turning
the engine rules and decision records into gates that fail, because several of
them were written down, read, and violated anyway. Worth reading before adding a
subsystem that overlaps an existing one.

## Key Conventions

- C++17, clang++, standard library only — no external deps
- PascalCase types, camelCase functions/variables, UPPER_SNAKE constants
- No hungarian notation
- Smart pointers for heap ownership, stack alloc for math types
- One header + one .cpp per module, minimal includes
- See AGENTS.md for full coding standards
