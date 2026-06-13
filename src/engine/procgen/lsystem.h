#ifndef RAYTRACER_ENGINE_LSYSTEM_H
#define RAYTRACER_ENGINE_LSYSTEM_H

#include "../../renderer/renderer.h"   // RenderMesh
#include <string>
#include <unordered_map>
#include <vector>

namespace engine {

// A deterministic L-system (ROADMAP 4 Phase B.1): rewrite an axiom by the
// production rules `iterations` times. Stochastic/parametric variants come
// later; this is the grammar half of the procgen substrate (ADR-0021).
struct LSystem {
    std::unordered_map<char, std::string> rules;
    std::string expand(const std::string& axiom, int iterations) const;
};

// 3D turtle interpretation parameters. The turtle grows along its local +Y;
// branches taper as they nest. Standard symbols, interpreted by buildTurtleMesh:
//   F      draw a branch segment forward (a cylinder), advance
//   + -    yaw   (rotate about local Z by +/- angle)
//   & ^    pitch (rotate about local X)
//   / \    roll  (rotate about local Y)
//   [ ]    push / pop turtle state (a branch); push tapers the radius
struct TurtleParams {
    float length = 1.0f;        // segment length per F
    float radius = 0.12f;       // branch radius at the trunk
    float radiusTaper = 0.8f;   // radius *= taper on each push (thinner branches)
    float angleDeg = 25.0f;     // turn angle for the yaw/pitch/roll symbols
    int   segmentSlices = 6;    // cylinder resolution
};

// Interpret an already-expanded L-system string into a branch mesh of
// kit-bashed cylinders (fast, but disjoint and self-intersecting at joints).
RenderMesh buildTurtleMesh(const std::string& symbols, const TurtleParams& params);

// Convenience: expand `axiom` by `system` for `iterations`, then interpret.
RenderMesh generateTree(const LSystem& system, const std::string& axiom,
                        int iterations, const TurtleParams& params);

// A branch as a capsule segment (for SDF skinning).
struct BranchSegment {
    Vec3 a, b;
    float radius;
};

// The turtle's branches as capsule segments (no meshing) — the input to SDF
// skinning, or to physics/analysis.
std::vector<BranchSegment> turtleSegments(const std::string& symbols,
                                          const TurtleParams& params);

// Interpret the string as a single *welded* surface: each branch is a capsule,
// smooth-union'd (smoothness = blend radius) and polygonized at `resolution`
// (grid cells per axis). One continuous, fillet-jointed mesh — no disjoint or
// self-intersecting segments. Heavier than buildTurtleMesh; generate once per
// species. (ROADMAP Phase A.1 / B.1.)
RenderMesh buildTurtleMeshSdf(const std::string& symbols, const TurtleParams& params,
                              double smoothness, int resolution);
RenderMesh generateTreeSdf(const LSystem& system, const std::string& axiom, int iterations,
                           const TurtleParams& params, double smoothness, int resolution);

}  // namespace engine

#endif
