#ifndef RAYTRACER_ENGINE_LSYSTEM_H
#define RAYTRACER_ENGINE_LSYSTEM_H

#include "../../renderer/renderer.h"   // RenderMesh
#include <string>
#include <unordered_map>
#include <vector>

namespace engine {

// A parametric L-system (ROADMAP 4 Phase B.1): rewrite an axiom by the
// production rules `iterations` times. Each symbol may have several weighted
// productions — with one it is deterministic, with several expand() picks per
// application from a seeded RNG, so different seeds grow different trees.
struct LSystem {
    struct Production {
        std::string text;
        double weight = 1.0;
    };
    std::unordered_map<char, std::vector<Production>> rules;

    // Add a production for `symbol`; call repeatedly to make it stochastic.
    void rule(char symbol, const std::string& replacement, double weight = 1.0);

    std::string expand(const std::string& axiom, int iterations,
                       uint32_t seed = 0) const;
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

// Convenience: expand `axiom` by `system` for `iterations` (seed selects the
// stochastic variant), then interpret.
RenderMesh generateTree(const LSystem& system, const std::string& axiom,
                        int iterations, const TurtleParams& params,
                        uint32_t seed = 0);

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
                           const TurtleParams& params, double smoothness, int resolution,
                           uint32_t seed = 0);

}  // namespace engine

#endif
