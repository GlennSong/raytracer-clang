#ifndef RAYTRACER_ENGINE_SCRIPTING_PROCGEN_BINDINGS_H
#define RAYTRACER_ENGINE_SCRIPTING_PROCGEN_BINDINGS_H

#include <memory>
#include <string>

namespace engine {

class ScriptVM;
struct RenderMesh;

// The procgen binding surface (ADR-0023): the pure generator vocabulary exposed
// to Lua. These mirror the node-graph value types and nodes (ADR-0021,
// docs/node-graph-plan.md) — a script is the *text* front-end to the same C++
// substrate the visual graph wraps; both lower to the same functions.
//
// Registers into a (sandboxed) ScriptVM:
//   sdf.sphere(center, radius)            -> Field
//   sdf.box(center, halfExtent)           -> Field
//   sdf.capsule(a, b, radius)             -> Field
//   sdf.union/intersect/subtract(a, b)    -> Field
//   sdf.smooth_union(a, b, k)             -> Field
//   noise.value2(seed, x, y)             -> Scalar
//   noise.fbm2(seed, x, y[, octaves])    -> Scalar
//   noise.fbm3(seed, x, y, z[, octaves]) -> Scalar
//   polygonize(field, {min=, max=}, res)  -> Mesh
// Vectors are 3-element Lua arrays, e.g. {0, 1, 0}. Fields and Meshes are opaque
// userdata (their C++ objects are released by __gc).
void openProcgenLibrary(ScriptVM& vm);

// Run a procgen script that `return`s a mesh (typically from `polygonize`) and
// hand back the result. Returns false (with `error` filled, if non-null) when
// the script errors or does not return a Mesh. `openProcgenLibrary` must have
// been called on `vm` first.
bool runProcgenMesh(ScriptVM& vm, const std::string& code,
                    std::shared_ptr<RenderMesh>& out, std::string* error = nullptr);

}  // namespace engine

#endif
