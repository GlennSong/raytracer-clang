#ifndef RAYTRACER_ENGINE_ANIM_SKELETON_H
#define RAYTRACER_ENGINE_ANIM_SKELETON_H

#include "../components.h"   // Transform + lerp(Transform) — one source of truth
#include "../../rt_math.h"

#include <string>
#include <vector>

namespace engine {

class DebugDraw;

namespace anim {

// The character animation substrate (ADR-0072, character-posing-plan P0): a
// joint hierarchy + full-body poses, source-agnostic by design. Lives in the
// `anim` namespace because `engine::Skeleton` is already taken — the procgen
// L-system branch skeleton (procgen/skeleton.h), a different concept that may
// one day FEED this one (a tree's branch graph is a natural rig) — the
// procedural mannequin fills it today, the glTF skin importer fills it in P2,
// and a future character generator fills the same type. Everything downstream
// (posing UI, timeline, skinning, the comic renderer) consumes only this.
//
// Conventions: joints are stored parent-before-child (addJoint enforces it),
// so world matrices compose in one forward pass. Transforms are the engine's
// TRS `Transform`; world composition is parentWorld * local, matching the
// document hierarchy (components.cpp worldMatrix).

struct Joint {
    std::string name;
    int parent = -1;        // index of the parent joint; -1 = root
    Transform bindLocal;    // local TRS in the bind pose (the mannequin's T-pose)
};

class Skeleton {
public:
    // Add a joint; `parent` must already exist (or -1). Returns the joint's
    // index, or -1 if the parent ordering is violated.
    int addJoint(const std::string& name, int parent, const Transform& bindLocal);

    int jointCount() const { return static_cast<int>(joints.size()); }
    const Joint& joint(int index) const { return joints[index]; }
    int find(const std::string& name) const;   // -1 when absent

    // World matrix per joint in the bind pose.
    std::vector<Mat4> bindWorld() const;

    // Inverse bind matrices (world -> joint space at bind): what skinning
    // multiplies against. Computed once by finalize(); empty before that.
    void finalize();
    const std::vector<Mat4>& inverseBind() const { return inverseBinds; }

    // Adopt authored inverse binds (the glTF skin accessor, P2) instead of
    // computing them — the file's matrices are authoritative for imported
    // rigs. Must be one per joint, in this skeleton's joint order.
    void setInverseBinds(std::vector<Mat4> matrices) {
        inverseBinds = std::move(matrices);
    }

private:
    std::vector<Joint> joints;
    std::vector<Mat4> inverseBinds;
};

// A full set of joint-local transforms — one keyframe of the whole body.
// Poses are what the pose library stores (P1) and what timeline keys hold
// (P4): a static pose is just a one-key clip by construction.
struct Pose {
    std::vector<Transform> locals;   // one per joint, skeleton order

    static Pose bind(const Skeleton& skeleton);
};

// Interpolate two poses (position/scale lerp, orientation slerp per joint —
// ADR-0006). The seed of timeline playback.
Pose lerpPose(const Pose& a, const Pose& b, Real t);

// World matrix per joint for a pose (one forward pass over the hierarchy).
std::vector<Mat4> worldJointMatrices(const Skeleton& skeleton, const Pose& pose);

// world * inverseBind per joint — identity at bind pose. What a skinned
// vertex is deformed by (P2); rigid per-bone parts use world directly.
std::vector<Mat4> skinningMatrices(const Skeleton& skeleton,
                                   const std::vector<Mat4>& world);

// Bone lines (parent origin -> joint origin) into the debug overlay
// (ADR-0067) — the editor's skeleton visualization.
void debugDrawSkeleton(DebugDraw& debug, const Skeleton& skeleton,
                       const std::vector<Mat4>& world, const Vec3& color,
                       Real seconds = 0);


}  // namespace anim
}  // namespace engine

#endif
