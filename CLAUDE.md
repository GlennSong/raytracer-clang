# Raytracer — Claude Code Context

## Build & Run

```bash
make            # debug build (offline path tracer)
make release    # optimized build
make clean      # remove build artifacts
./raytracer     # renders to output.ppm
make test       # Jolt-free unit tests (math, ECS, input, camera)
```

The interactive viewer and physics build via CMake. Jolt and Dear ImGui are git
submodules — fetch them first (re-run after pulling commits that add a new
submodule; CMake also tries to auto-init them):

```bash
git submodule update --init --recursive    # fetch third_party/{JoltPhysics,imgui}
cmake -S . -B build && cmake --build build
ctest --test-dir build                      # runs unit + physics tests
./build/viewer                              # interactive viewer (where GLFW exists)
```

The viewer target builds only where GLFW is found (e.g. macOS); physics
(`-DRT_ENABLE_PHYSICS=ON`, default) is cross-platform and builds/tests headless.
Add `-DRT_ENABLE_IMGUI=ON` to enable the Dear ImGui debug overlay.

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
