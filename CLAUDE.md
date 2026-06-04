# Raytracer — Claude Code Context

## Build & Run

```bash
make            # debug build (offline path tracer)
make release    # optimized build
make clean      # remove build artifacts
./raytracer     # renders to output.ppm
make test       # Jolt-free unit tests (math, ECS, input, camera)
```

The interactive viewer and physics build via CMake. Jolt is a git submodule, so
fetch it first (CMake will also try to auto-init it):

```bash
git submodule update --init --recursive   # fetch third_party/JoltPhysics
cmake -S . -B build && cmake --build build
ctest --test-dir build                     # runs unit + physics tests
```

The viewer target builds only where GLFW is found (e.g. macOS); physics
(`-DRT_ENABLE_PHYSICS=ON`, default) is cross-platform and builds/tests headless.

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
