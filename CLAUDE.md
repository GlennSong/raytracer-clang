# Raytracer — Claude Code Context

## Build & Run

```bash
make            # debug build
make release    # optimized build
make clean      # remove build artifacts
./raytracer     # renders to output.ppm
```

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
