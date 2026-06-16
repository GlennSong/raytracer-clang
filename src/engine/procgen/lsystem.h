#ifndef RAYTRACER_ENGINE_LSYSTEM_H
#define RAYTRACER_ENGINE_LSYSTEM_H

#include "../../renderer/renderer.h"   // RenderMesh
#include <functional>
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

// ---------------------------------------------------------------------------
// Parametric L-system (ABoP §1.10). Modules carry numeric parameters, so a
// production can shrink length / width / angle with recursion depth — the basis
// of self-similar-but-tapering trees that the plain char grammar above cannot
// express. Productions match by symbol *and* parameter arity; each successor
// parameter is an arithmetic expression over the predecessor's parameters.
// Several productions for one predecessor make it stochastic (weighted),
// exactly like LSystem.

// A module: a symbol plus numeric parameters, e.g. F(1.0, 0.3). The bracket
// modules '[' and ']' (branch push / pop) and bare turn symbols carry none.
struct Module {
    char symbol = 0;
    std::vector<float> params;
};
using ModuleString = std::vector<Module>;

// A compiled successor-parameter expression, evaluated against the bound
// predecessor parameters (mirrors the `Sdf = std::function` precedent).
using ParamExpr = std::function<float(const std::vector<float>&)>;

// A compiled production guard: a boolean condition over the bound predecessor
// parameters (e.g. "l < 0.5"). Empty = always applies (ABoP guarded productions).
using ParamPredicate = std::function<bool(const std::vector<float>&)>;

class ParametricLSystem {
public:
    // Add a production. `predecessor` is a symbol with formal parameter names —
    // "A(l,w)" (or just "A"), with an optional guard "A(l,w):l<0.5" so a rule
    // fires only when the condition holds (ABoP guarded productions — used for
    // terminal sprays once an internode gets short). `successor` is a module
    // template whose parameters are expressions over those names, e.g.
    //   "F(l,w)[+(30)A(l*0.7,w*0.6)][-(30)A(l*0.7,w*0.6)]"
    // Supported expression syntax: + - * /, parentheses, unary minus, numeric
    // literals, and the formal names. Repeat the same predecessor to add
    // weighted stochastic alternatives.
    void rule(const std::string& predecessor, const std::string& successor,
              double weight = 1.0);

    // Rewrite `iterations` times; `seed` selects the stochastic variant. Modules
    // with no matching production (by symbol + arity) are copied verbatim.
    ModuleString expand(const ModuleString& axiom, int iterations,
                        uint32_t seed = 0) const;
    ModuleString expand(const std::string& axiom, int iterations,
                        uint32_t seed = 0) const;   // parses a literal axiom

private:
    struct ModuleTemplate {
        char symbol = 0;
        std::vector<ParamExpr> params;
    };
    struct Production {
        std::vector<std::string> formals;
        std::vector<ModuleTemplate> successor;
        ParamPredicate condition;   // empty = unconditional; else a guard ("l<0.5")
        double weight = 1.0;
    };
    std::unordered_map<char, std::vector<Production>> rules_;
};

// Parse a module string of numeric *literals* (e.g. an axiom "A(1,0.2)") into
// modules. Parameter expressions are not allowed here — only an axiom.
ModuleString parseModuleLiterals(const std::string& text);
// Serialize modules back to compact text ("F(1,0.5)[+(30)A]") — tests/debug.
std::string moduleString(const ModuleString& s);

// 3D turtle interpretation parameters. The turtle grows along its local +Y;
// branches taper as they nest. Standard symbols, interpreted by buildTurtleMesh:
//   F      draw a branch segment forward (a cylinder), advance
//   L      drop a leaf blob (a sphere of leafRadius) at the current point
//   + -    yaw   (rotate about local Z by +/- angle)
//   & ^    pitch (rotate about local X)
//   / \    roll  (rotate about local Y)
//   [ ]    push / pop turtle state (a branch); push tapers the radius
struct TurtleParams {
    float length = 1.0f;        // segment length per F
    float radius = 0.12f;       // branch radius at the trunk
    float radiusTaper = 0.8f;   // radius *= taper on each push (thinner branches)
    float taper = 1.0f;         // radius *= taper per F (continuous thinning up
                                //   the trunk/branches; 1.0 = no taper)
    float angleDeg = 25.0f;     // turn angle for the yaw/pitch/roll symbols
    int   segmentSlices = 6;    // cylinder resolution
    float leafRadius = 0.0f;    // leaf-blob radius for L (0 = no leaves)
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

// A leaf attachment point: where an `L` symbol lands and the turtle heading
// there (so a caller can place an oriented leaf card instead of a blob).
struct LeafPlacement {
    Vec3 position;
    Vec3 direction;   // turtle heading (local +Y) at the leaf
};

// Every leaf attachment point in the string (independent of leafRadius, so a
// caller that draws real leaves needn't enable the blob path).
std::vector<LeafPlacement> turtleLeaves(const std::string& symbols,
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
