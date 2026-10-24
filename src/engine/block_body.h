#ifndef RAYTRACER_ENGINE_BLOCK_BODY_H
#define RAYTRACER_ENGINE_BLOCK_BODY_H

// THE PERSON AS SIX BLOCKS, posed part by part -- the same blocks buildStandingPerson (procgen/person.cpp) welds
// into one walker mesh: two legs, the torso, two arms, the head. Posed, they are the player's body in third person
// doing what it does (climbing, swimming, flying, hanging on a rope); as physics bodies joined at the hips,
// shoulders and neck they are the ragdoll (PhysicsWorld::addRagdoll).
//
// Body frame: the capsule centre at the origin, +y up, +z the way the body faces -- so +x is the body's LEFT (y up,
// facing +z, right-handed) and the L parts sit at +x.

#include "../rt_math.h"
#include "physics/physics_world.h"

#include <array>
#include <vector>

namespace engine {

struct BlockBody {
    enum Part { Torso, Head, ArmL, ArmR, LegL, LegR, kParts };
    struct PartSpec {
        Vec3 size;      // the block, unscaled (the 1.8 m person)
        Vec3 joint;     // where it hangs from (body frame, unscaled): the hip, shoulder, neck; the torso's pelvis
        Vec3 rest;      // the way from the joint to the block's centre at rest (unscaled)
        int parent;     // -1 for the torso
    };
    static const std::array<PartSpec, kParts>& parts();
    static constexpr Real kHalfHeight = 0.9;   // the person spans y in [-0.9, 0.9] unscaled
};

// A POSE: where the body is and which way each limb points. Directions are unit vectors in the BODY frame
// (after `orientation`); a limb's rest direction is straight down (arms, legs) or up (head).
struct BodyPose {
    Vec3 position;                 // the capsule centre (world)
    Quat orientation;              // the body's facing and lean
    Real scale = 1;                // (capsule half height + radius) / 0.9
    Vec3 armL{0, -1, 0}, armR{0, -1, 0}, legL{0, -1, 0}, legR{0, -1, 0};
};

struct PartTransform {
    Vec3 position;     // the block's centre (world)
    Quat orientation;
    Vec3 size;         // scaled
};

// Where each block goes for a pose.
std::array<PartTransform, BlockBody::kParts> poseBlocks(const BodyPose& pose);
// The rotation taking local -y to `dir` (unit) with as little twist as it can: a limb hung along a direction.
Quat hangAlong(const Vec3& dir);
// A body-frame direction for a limb reaching from its joint to a world point.
Vec3 reachToward(const BodyPose& pose, BlockBody::Part limb, const Vec3& world);
// THE RAGDOLL for a pose: the six blocks as bodies where the pose puts them (67 kg, the torso most of it), joined
// at the neck, shoulders and hips with a person's limits (arms 85° each way, legs 60°, the head 35°).
std::vector<PhysicsWorld::RagdollPart> blockRagdoll(const BodyPose& pose);
// Where a part's joint is now, from its block's transform (the hip of a leg, the shoulder of an arm): what holds
// a ragdoll together, for tests.
Vec3 jointOf(BlockBody::Part part, const PartTransform& block, Real scale);

}  // namespace engine

#endif
