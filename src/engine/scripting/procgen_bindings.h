#ifndef RAYTRACER_ENGINE_SCRIPTING_PROCGEN_BINDINGS_H
#define RAYTRACER_ENGINE_SCRIPTING_PROCGEN_BINDINGS_H

#include <memory>
#include <string>
#include <vector>

#include "../../rt_math.h"   // Vec3

namespace engine {

class ScriptVM;
struct RenderMesh;

// One material-part of a procedural model (ADR-0032): geometry plus the intent
// to build a RenderMaterial. `hasMaterial == false` means "use the caller's
// default material" (a bare single-mesh script return — back-compat). `texture`
// names a built-in procedural texture ("bark" | "leaf"; "" = none); a leaf
// texture implies alpha-cut.
struct ScriptMeshPart {
    std::shared_ptr<RenderMesh> mesh;
    bool hasMaterial = false;
    Vec3 albedo{1, 1, 1};
    float roughness = 1.0f;
    float metallic = 0.0f;
    bool alphaTest = false;
    std::string texture;
};

// The procgen binding surface (ADR-0023): the pure generator vocabulary exposed
// to Lua. These mirror the node-graph value types and nodes (ADR-0021,
// docs/node-graph-plan.md) — a script is the *text* front-end to the same C++
// substrate the visual graph wraps; both lower to the same functions.
//
// Registers into a (sandboxed) ScriptVM the full generator vocabulary across the
// value types (Field / Mesh / Frame):
//   Fields (SDF):
//     sdf.sphere(center, radius) / box(center, halfExtent) / capsule(a, b, r)
//     sdf.union/intersect/subtract(a, b) / smooth_union(a, b, k)        -> Field
//   Noise:
//     noise.value2/fbm2/fbm3(seed, x[, y[, z]][, octaves])             -> Scalar
//   Mesh primitives + assembly:
//     mesh.box/sphere/cylinder/cone/plane/torus/capsule(...)            -> Mesh
//     mesh.merge({m1, m2, ...}) / translate / scale / rotate_y          -> Mesh
//     mesh.recompute_normals / bake_height_color(m, low, high)          -> Mesh
//   Grammar (L-system):
//     local sys = lsystem.create(); sys:rule("F","FF"[,w]); sys:expand(axiom,n[,seed])
//     lsystem.turtle_mesh(symbols, params) / turtle_mesh_sdf(symbols, params, k, res)
//   Generators:
//     polygonize(field, {min=, max=}, res)                              -> Mesh
//     terrain(params, seed)                                             -> Mesh
//     scatter(scatterParams, terrainParams, terrainSeed)               -> Frames
// Vectors are 3-element Lua arrays, e.g. {0, 1, 0}; params/bounds are tables with
// named fields. Fields/Meshes/LSystems are opaque userdata (released by __gc);
// Frames are plain Lua arrays of {position, yaw, scale}. The heavy lifting stays
// in C++ — a script orchestrates (ADR-0023).
void openProcgenLibrary(ScriptVM& vm);

// Run a procgen script that `return`s a mesh (typically from `polygonize`) and
// hand back the result. Returns false (with `error` filled, if non-null) when
// the script errors or does not return a Mesh. `openProcgenLibrary` must have
// been called on `vm` first.
bool runProcgenMesh(ScriptVM& vm, const std::string& code,
                    std::shared_ptr<RenderMesh>& out, std::string* error = nullptr);

// Run a procgen script that returns a *model* — one mesh, or a list of parts —
// and hand back the parts (ADR-0032). Accepts: a single Mesh (one part,
// hasMaterial=false), a list of Meshes, or a list of part tables
// `{ mesh=, texture=, alpha_test=, albedo=, roughness=, metallic= }`. Returns
// false (with `error`) on a script error or a non-model return.
bool runProcgenModel(ScriptVM& vm, const std::string& code,
                     std::vector<ScriptMeshPart>& out, std::string* error = nullptr);

}  // namespace engine

#endif
