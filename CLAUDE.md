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
cmake -S . -B build && cmake --build build
ctest --test-dir build                      # runs unit + physics tests
./build/viewer                              # the game (boots into play; --edit for edit mode)
```

The viewer target builds only where GLFW is found (e.g. macOS); physics
(`-DRT_ENABLE_PHYSICS=ON`, default) is cross-platform and builds/tests headless.
Add `-DRT_ENABLE_IMGUI=ON` to enable the Dear ImGui debug overlay. The Lua
scripting layer (`-DRT_ENABLE_SCRIPTING=ON`, default) is pure C — cross-platform,
builds/tests headless — sealed behind `ScriptVM` (ADR-0023); the procgen binding
surface is covered by `tests/test_script_vm.cpp`. Audio
(`-DRT_ENABLE_AUDIO=ON`, default) is miniaudio (submodule) sealed behind
`AudioEngine` (ADR-0069) — device-optional, tested headless via a pumped mix
(`tests/test_audio.cpp`). `-DRT_ENABLE_PROFILER=ON` compiles the Tracy client
(ADR-0068); the `RT_PROFILE_*` macros in `src/profile.h` are no-ops without it.

Gamepad support uses Apple's GCController framework on macOS (ADR-0013) for
Xbox/PS controllers, with GLFW's IOKit path as fallback. A
`gamecontrollerdb.txt` (SDL_GameControllerDB) is loaded at init for the IOKit
path. See `src/renderer/gamepad_gc.mm`.

## Planning

See `docs/ROADMAP.md` for the multi-tier development plan (foundation → 3D
infrastructure → content pipeline → procedural generation). Architecture
decisions live in `docs/decisions.md`.

## Key Conventions

- C++17, clang++, standard library only — no external deps
- PascalCase types, camelCase functions/variables, UPPER_SNAKE constants
- No hungarian notation
- Smart pointers for heap ownership, stack alloc for math types
- One header + one .cpp per module, minimal includes
- See AGENTS.md for full coding standards
