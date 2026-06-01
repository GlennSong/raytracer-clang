# Raytracer Project — Agent Guidelines

## Project Overview

A portable C++ raytracer built from scratch using only standard libraries.
Compiled with clang++ targeting C++17. No external dependencies.

## Architecture

```
src/
  math.h / math.cpp       — Vec3, Mat4, Ray, all linear algebra
  image.h / image.cpp      — Image buffer and PPM output
  camera.h / camera.cpp    — Camera model and ray generation
  geometry.h / geometry.cpp — Shapes (Sphere, Triangle), hit records
  material.h / material.cpp — Materials and BRDFs
  scene.h / scene.cpp       — Scene graph, object list
  objloader.h / objloader.cpp — Wavefront OBJ parser
  kdtree.h / kdtree.cpp     — KD-tree acceleration structure
  main.cpp                   — Entry point, render loop
```

Each module is one header + one implementation file. Keep includes minimal —
a module should only include what it directly uses.

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
