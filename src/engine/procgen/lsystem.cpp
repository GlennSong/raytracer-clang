#include "lsystem.h"
#include "../mesh_builder.h"
#include "sdf.h"
#include "../../rt_math.h"

#include <algorithm>
#include <vector>

namespace engine {

std::string LSystem::expand(const std::string& axiom, int iterations) const {
    std::string current = axiom;
    for (int it = 0; it < iterations; it++) {
        std::string next;
        next.reserve(current.size() * 2);
        for (char c : current) {
            auto rule = rules.find(c);
            if (rule != rules.end()) next += rule->second;
            else next += c;
        }
        current.swap(next);
    }
    return current;
}

namespace {

// The turtle state machine, shared by every interpretation. `onForward` is
// called for each F with the segment's start position, the turtle orientation
// (local +Y is the heading), and the current branch radius; the caller turns
// that into geometry (a cylinder, a capsule SDF, a collected segment).
template <typename Fn>
void walkTurtle(const std::string& symbols, const TurtleParams& params, Fn onForward) {
    struct State {
        Vec3 position;
        Mat4 orientation;   // rotates local +Y onto the heading
        float radius;
    };
    State st{Vec3(0, 0, 0), Mat4::identity(), params.radius};
    std::vector<State> stack;
    const double angle = params.angleDeg * PI / 180.0;

    for (char c : symbols) {
        switch (c) {
            case 'F': {
                onForward(st.position, st.orientation, st.radius);
                Vec3 heading = st.orientation.transformDirection(Vec3(0, 1, 0));
                st.position = st.position + heading * params.length;
                break;
            }
            case '+': st.orientation = st.orientation * Mat4::rotateZ(angle);  break;
            case '-': st.orientation = st.orientation * Mat4::rotateZ(-angle); break;
            case '&': st.orientation = st.orientation * Mat4::rotateX(angle);  break;
            case '^': st.orientation = st.orientation * Mat4::rotateX(-angle); break;
            case '/': st.orientation = st.orientation * Mat4::rotateY(angle);  break;
            case '\\': st.orientation = st.orientation * Mat4::rotateY(-angle); break;
            case '[': stack.push_back(st); st.radius *= params.radiusTaper;    break;
            case ']': if (!stack.empty()) { st = stack.back(); stack.pop_back(); } break;
            default: break;   // letters with no turtle meaning (e.g. rule symbols)
        }
    }
}

}  // namespace

RenderMesh buildTurtleMesh(const std::string& symbols, const TurtleParams& params) {
    RenderMesh mesh;
    walkTurtle(symbols, params, [&](const Vec3& pos, const Mat4& orient, float radius) {
        // A cylinder is centered on its local Y over [-h/2, h/2]; place its
        // center half a length along the heading so the segment runs from pos to
        // pos + heading*length.
        RenderMesh seg = MeshBuilder::cylinder(radius, params.length, params.segmentSlices);
        Vec3 heading = orient.transformDirection(Vec3(0, 1, 0));
        Vec3 center = pos + heading * (params.length * 0.5);
        Mat4 xform = Mat4::translate(center.x, center.y, center.z) * orient;
        MeshBuilder::appendTransformed(mesh, seg, xform);
    });
    return mesh;
}

std::vector<BranchSegment> turtleSegments(const std::string& symbols,
                                          const TurtleParams& params) {
    std::vector<BranchSegment> segments;
    walkTurtle(symbols, params, [&](const Vec3& pos, const Mat4& orient, float radius) {
        Vec3 heading = orient.transformDirection(Vec3(0, 1, 0));
        segments.push_back({pos, pos + heading * params.length, radius});
    });
    return segments;
}

RenderMesh buildTurtleMeshSdf(const std::string& symbols, const TurtleParams& params,
                              double smoothness, int resolution) {
    std::vector<BranchSegment> segments = turtleSegments(symbols, params);
    if (segments.empty()) return RenderMesh{};

    // Bounds + sizing first, so we can keep branches above a minimum radius.
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    double maxRadius = 0.0;
    auto expand = [&](const Vec3& p) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    };
    for (const BranchSegment& s : segments) {
        expand(s.a);
        expand(s.b);
        maxRadius = std::max(maxRadius, static_cast<double>(s.radius));
    }

    double margin = maxRadius + smoothness;
    Vec3 size = (hi - lo) + Vec3(2 * margin, 2 * margin, 2 * margin);
    double cell = std::max({size.x, size.y, size.z}) / std::max(1, resolution);
    // A capsule thinner than ~1.5 cells can't be captured by Surface Nets (it
    // falls between samples), producing the "weird geometry" on thin branches.
    // Clamp each branch radius up to that floor so every branch stays solid.
    double minRadius = 1.5 * cell;

    std::vector<Sdf> capsules;
    capsules.reserve(segments.size());
    for (const BranchSegment& s : segments)
        capsules.push_back(sdfCapsule(s.a, s.b, std::max(static_cast<double>(s.radius), minRadius)));

    // Pad the sampling box so the full radius + blend fits with a cell to spare.
    Vec3 pad(margin + cell, margin + cell, margin + cell);
    SdfBounds bounds{lo - pad, hi + pad};

    Sdf field = sdfSmoothUnion(capsules, smoothness);
    return polygonizeSdf(field, bounds, resolution);
}

RenderMesh generateTree(const LSystem& system, const std::string& axiom,
                        int iterations, const TurtleParams& params) {
    return buildTurtleMesh(system.expand(axiom, iterations), params);
}

RenderMesh generateTreeSdf(const LSystem& system, const std::string& axiom, int iterations,
                           const TurtleParams& params, double smoothness, int resolution) {
    return buildTurtleMeshSdf(system.expand(axiom, iterations), params, smoothness, resolution);
}

}  // namespace engine
