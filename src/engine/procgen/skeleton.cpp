#include "skeleton.h"

namespace engine {

std::vector<std::vector<int>> Skeleton::childLists() const {
    std::vector<std::vector<int>> children(nodes.size());
    for (size_t i = 0; i < nodes.size(); i++)
        if (nodes[i].parent >= 0)
            children[nodes[i].parent].push_back(static_cast<int>(i));
    return children;
}

Skeleton buildSkeleton(const ModuleString& s, float angleJitter, std::mt19937& rng) {
    Skeleton skel;
    std::vector<SkeletonNode>& nodes = skel.nodes;
    nodes.push_back(SkeletonNode{});   // root at origin, heading +Y, depth 0

    struct State { int node; Mat4 orient; int depth; };
    State st{0, Mat4::identity(), 0};
    std::vector<State> stack;

    std::uniform_real_distribution<float> jit(-angleJitter, angleJitter);
    auto turn = [&](char axis, float deg) {
        double a = (deg + jit(rng)) * PI / 180.0;
        switch (axis) {
            case 'Z': st.orient = st.orient * Mat4::rotateZ(a); break;
            case 'X': st.orient = st.orient * Mat4::rotateX(a); break;
            case 'Y': st.orient = st.orient * Mat4::rotateY(a); break;
        }
    };

    for (const Module& m : s) {
        float arg = m.params.empty() ? 0.0f : m.params[0];
        switch (m.symbol) {
            case 'F': {
                Vec3 heading = st.orient.transformDirection(Vec3(0, 1, 0));
                SkeletonNode n;
                n.parent = st.node;
                n.pos = nodes[st.node].pos + heading * arg;
                n.heading = heading;
                n.depth = st.depth;
                n.distFromRoot = nodes[st.node].distFromRoot + arg;
                nodes[st.node].childCount++;
                st.node = static_cast<int>(nodes.size());
                nodes.push_back(n);
                break;
            }
            case 'A':   // an unexpanded apex => a tip (leaf attach / ridge end)
            case 'L':
                nodes[st.node].isTip = true;
                break;
            case '+': turn('Z',  arg); break;
            case '-': turn('Z', -arg); break;
            case '&': turn('X',  arg); break;
            case '^': turn('X', -arg); break;
            case '/': turn('Y',  arg); break;
            case '\\': turn('Y', -arg); break;
            case '[': stack.push_back(st); st.depth++; break;  // a branch is deeper
            case ']': if (!stack.empty()) { st = stack.back(); stack.pop_back(); } break;
            default: break;
        }
    }
    return skel;
}

Skeleton buildSkeleton(const ModuleString& modules, float angleJitter, uint32_t seed) {
    std::mt19937 rng(seed);
    return buildSkeleton(modules, angleJitter, rng);
}

}  // namespace engine
