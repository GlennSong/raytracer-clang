#ifndef RAYTRACER_ENGINE_ANIM_SKINNED_MESH_H
#define RAYTRACER_ENGINE_ANIM_SKINNED_MESH_H

#include "skeleton.h"
#include "../../renderer/renderer.h"

#include <cstdint>
#include <vector>

namespace engine {
namespace anim {

// A vertex's skinning influences: up to four joints and their blend weights
// (the glTF JOINTS_0/WEIGHTS_0 model). Joint indices are anim::Skeleton
// indices (the importer remaps from glTF's skin-local numbering).
struct VertexWeights {
    uint16_t joints[4] = {0, 0, 0, 0};
    float weights[4] = {0, 0, 0, 0};
};

// A mesh that deforms with a Skeleton (character-posing-plan P2): bind-pose
// geometry plus per-vertex influences, parallel arrays. `bind` is ordinary
// RenderMesh, so everything that eats meshes (upload, tracer tessellation)
// works on the SKINNED result unchanged.
struct SkinnedMesh {
    RenderMesh bind;
    std::vector<VertexWeights> influences;   // one per bind vertex
    Vec3 baseColor{1, 1, 1};                 // glTF baseColorFactor (P2 scope)
};

// CPU linear-blend skinning: deform the bind mesh by the pose's skinning
// matrices (anim::skinningMatrices — world * inverseBind). Positions blend as
// points, normals/tangents as directions (renormalized). CPU on purpose
// (ADR-0073): one implementation feeds the realtime upload path AND the
// offline tracer; GPU skinning is a later perf move behind the same data.
RenderMesh skinMesh(const SkinnedMesh& mesh,
                    const std::vector<Mat4>& skinningMatrices);


}  // namespace anim
}  // namespace engine

#endif
