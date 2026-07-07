#ifndef RAYTRACER_ENGINE_ANIM_MANNEQUIN_H
#define RAYTRACER_ENGINE_ANIM_MANNEQUIN_H

#include "skeleton.h"
#include "../../renderer/renderer.h"

#include <vector>

namespace engine {
namespace anim {

// The posable mannequin (ADR-0072, character-posing-plan P0): a procedurally
// built humanoid — a real Skeleton plus one rigid mesh part per bone — so the
// posing/timeline/render workflow is provable end to end before any character
// import exists. Parts are authored in their joint's LOCAL frame; posing a
// part is just its joint's world matrix (the P0 stand-in for skinning).

struct MannequinPart {
    RenderMesh mesh;     // in the joint's local frame
    int joint = -1;      // skeleton joint the part rides
    Vec3 color;          // baked into the mesh's vertex colors too
};

struct Mannequin {
    Skeleton skeleton;   // finalized (inverse binds ready)
    std::vector<MannequinPart> parts;
};

// A T-pose humanoid of the given height (metres), feet at y=0, facing +Z.
// Joint names follow the pelvis/spine/chest/neck/head core with .L/.R
// mirrored limbs: shoulder/elbow/wrist and hip/knee/ankle.
Mannequin buildMannequin(Real height = 1.8);

// Bake every part into ONE world-space mesh for a pose: part vertices
// transformed by model * jointWorld. What both renderers consume — the
// realtime path uploads it, the offline tracer tessellates it — so a posed
// mannequin renders everywhere the engine renders.
RenderMesh bakeMannequinMesh(const Mannequin& mannequin, const Pose& pose,
                             const Mat4& model = Mat4());


}  // namespace anim
}  // namespace engine

#endif
