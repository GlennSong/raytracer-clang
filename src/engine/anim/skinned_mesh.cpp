#include "skinned_mesh.h"

namespace engine {
namespace anim {

RenderMesh skinMesh(const SkinnedMesh& mesh,
                    const std::vector<Mat4>& skinningMatrices) {
    RenderMesh out = mesh.bind;
    const std::size_t count =
        std::min(out.vertices.size(), mesh.influences.size());
    for (std::size_t i = 0; i < count; i++) {
        const VertexWeights& vw = mesh.influences[i];
        const Vertex& bind = mesh.bind.vertices[i];
        Vec3 position, normal, tangent;
        Real totalWeight = 0;
        for (int k = 0; k < 4; k++) {
            if (vw.weights[k] <= 0.0f) continue;
            if (vw.joints[k] >= skinningMatrices.size()) continue;
            const Mat4& m = skinningMatrices[vw.joints[k]];
            Real w = vw.weights[k];
            totalWeight += w;
            position += m.transformPoint(bind.position) * w;
            normal += m.transformDirection(bind.normal) * w;
            tangent += m.transformDirection(bind.tangent) * w;
        }
        if (totalWeight <= 1e-6) continue;   // unweighted: stays at bind
        position = position * (1.0 / totalWeight);
        Vertex& v = out.vertices[i];
        v.position = position;
        if (normal.lengthSquared() > 1e-12) v.normal = normalize(normal);
        if (tangent.lengthSquared() > 1e-12) v.tangent = normalize(tangent);
    }
    return out;
}

}  // namespace anim
}  // namespace engine
