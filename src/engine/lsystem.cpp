#include "lsystem.h"
#include "mesh_builder.h"
#include "../rt_math.h"

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

RenderMesh buildTurtleMesh(const std::string& symbols, const TurtleParams& params) {
    RenderMesh mesh;

    struct State {
        Vec3 position;
        Mat4 orientation;   // rotates local +Y onto the current heading
        float radius;
    };
    State st{Vec3(0, 0, 0), Mat4::identity(), params.radius};
    std::vector<State> stack;

    const double angle = params.angleDeg * PI / 180.0;

    for (char c : symbols) {
        switch (c) {
            case 'F': {
                Vec3 heading = st.orientation.transformDirection(Vec3(0, 1, 0));
                // A cylinder is centered on its local Y over [-h/2, h/2]; place
                // its center half a length along the heading so the segment runs
                // from the current position to position + heading*length.
                RenderMesh seg = MeshBuilder::cylinder(st.radius, params.length,
                                                       params.segmentSlices);
                Vec3 center = st.position + heading * (params.length * 0.5);
                Mat4 xform = Mat4::translate(center.x, center.y, center.z) * st.orientation;
                MeshBuilder::appendTransformed(mesh, seg, xform);
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
    return mesh;
}

RenderMesh generateTree(const LSystem& system, const std::string& axiom,
                        int iterations, const TurtleParams& params) {
    return buildTurtleMesh(system.expand(axiom, iterations), params);
}

}  // namespace engine
