#ifndef RAYTRACER_ENGINE_BLOCK_BODY_H
#define RAYTRACER_ENGINE_BLOCK_BODY_H

// THE PERSON AS TEN BLOCKS, posed part by part: the torso, the head, and each limb in two -- upper arm and forearm,
// thigh and shin (Glenn, 2026-10-25: "should we have elbow and knee joints in the player and agent model so there's
// some knee bend and slightly more detailed ragdolling?"). Straight, a limb's two blocks are exactly the one block
// buildStandingPerson (procgen/person.cpp) welds for the far crowd. Posed, they are the player's body in third
// person doing what it does (climbing, swimming, flying, hanging on a rope) and the near walkers' stride
// (buildPosedPerson); as physics bodies joined at the hips, shoulders and neck and HINGED at the knees and elbows
// they are the ragdoll (PhysicsWorld::addRagdoll).
//
// Body frame: the capsule centre at the origin, +y up, +z the way the body faces -- so +x is the body's LEFT (y up,
// facing +z, right-handed) and the L parts sit at +x.

#include "../rt_math.h"
#include "physics/physics_world.h"
#include "../renderer/renderer.h"   // RenderMesh

#include <array>
#include <vector>

namespace engine {

struct BlockBody {
    // ArmL..LegR are the UPPER halves (upper arm, thigh) and keep their old numbers; a limb's lower half is +4
    enum Part { Torso, Head, ArmL, ArmR, LegL, LegR, ForearmL, ForearmR, ShinL, ShinR, kParts };
    struct PartSpec {
        Vec3 size;      // the block, unscaled (the 1.8 m person)
        Vec3 joint;     // where it hangs from AT REST (body frame, unscaled): the hip, shoulder, neck, knee, elbow;
                        // the torso's pelvis
        Vec3 rest;      // the way from the joint to the block's centre at rest (unscaled)
        int parent;     // -1 for the torso
    };
    static const std::array<PartSpec, kParts>& parts();
    static constexpr Real kHalfHeight = 0.9;   // the person spans y in [-0.9, 0.9] unscaled
    static constexpr int kLimbs = 4;           // ArmL, ArmR, LegL, LegR: limb k is upper part 2 + k, lower part 6 + k
    static constexpr Part upper(int limb) { return static_cast<Part>(ArmL + limb); }
    static constexpr Part lower(int limb) { return static_cast<Part>(ForearmL + limb); }
    static constexpr bool isLeg(int part) { return part == LegL || part == LegR || part == ShinL || part == ShinR; }
    static constexpr bool isLower(int part) { return part >= ForearmL; }
};

// A POSE: where the body is, which way each limb points and how far it bends. A limb's direction is the way from
// its shoulder or hip to its hand or foot, a unit vector in the BODY frame (after `orientation`); its rest is
// straight down. Its bend is the elbow's or knee's, in radians (0 straight): the elbow folds the forearm forward
// with the point of the elbow back, the knee folds the shin back with the kneecap forward. A bent limb is shorter
// along its direction -- the hand or foot comes in -- so a pose that only sets directions is the straight body.
struct BodyPose {
    Vec3 position;                 // the capsule centre (world)
    Quat orientation;              // the body's facing and lean
    Real scale = 1;                // (capsule half height + radius) / 0.9
    Vec3 armL{0, -1, 0}, armR{0, -1, 0}, legL{0, -1, 0}, legR{0, -1, 0};
    Real elbowL = 0, elbowR = 0, kneeL = 0, kneeR = 0;
    Quat neck = Quat::identity();  // the head turned on the neck (body frame): a nod, a snap in a crash
    Vec3& dir(int limb) { return limb == 0 ? armL : limb == 1 ? armR : limb == 2 ? legL : legR; }
    const Vec3& dir(int limb) const { return limb == 0 ? armL : limb == 1 ? armR : limb == 2 ? legL : legR; }
    Real& bend(int limb) { return limb == 0 ? elbowL : limb == 1 ? elbowR : limb == 2 ? kneeL : kneeR; }
    Real bend(int limb) const { return limb == 0 ? elbowL : limb == 1 ? elbowR : limb == 2 ? kneeL : kneeR; }
};

// A LIMB AS POSED (two-bone): its root joint, the elbow or knee, the hand or foot, in the world; the upper and
// lower halves' directions; and the axis the joint bends about (right-handed: turning the lower half about it
// bends the limb further).
struct LimbSolve {
    Vec3 root, mid, end;
    Vec3 upperDir, lowerDir;
    Vec3 hingeAxis;
};
LimbSolve solveLimb(const BodyPose& pose, int limb);
// The most an elbow and a knee fold (radians).
constexpr Real kElbowMax = 2.5, kKneeMax = 2.6;
// What it takes to turn a limp elbow and knee (N m): enough that a body down doesn't fold every joint shut.
constexpr Real kElbowFriction = 1.5, kKneeFriction = 5.0;

struct PartTransform {
    Vec3 position;     // the block's centre (world)
    Quat orientation;
    Vec3 size;         // scaled
};

// Where each block goes for a pose. A limb's upper half is turned so its x is the hinge's axis (and its -y runs down
// it); the lower half is that turned about x by the fold.
std::array<PartTransform, BlockBody::kParts> poseBlocks(const BodyPose& pose);
// The rotation with these as its x and y (orthonormal; z = x cross y).
Quat frameOf(const Vec3& x, const Vec3& y);
// The fold of a limb from its two halves' orientations (poseBlocks', or a ragdoll's: its hinge is the upper's x).
Real foldOf(const Quat& upper, const Quat& lower);
// THE SKINNED LIMB, folded `fold` radians: one continuous tube -- upper half, the bend, lower half -- in the upper
// half's frame from poseBlocks with its ROOT JOINT at the origin (the shoulder or hip), unscaled. Placed at the
// root with the upper's orientation and the pose's scale it covers both halves' blocks.
RenderMesh skinnedLimb(int limb, Real fold, const Vec3& colour);
// The rotation taking local -y to `dir` (unit) with as little twist as it can: a limb hung along a direction.
Quat hangAlong(const Vec3& dir);
// A body-frame direction for a limb reaching from its joint to a world point.
Vec3 reachToward(const BodyPose& pose, BlockBody::Part limb, const Vec3& world);
// TWO-BONE IK: point the limb at a world point AND bend it so the hand or foot lands on it -- straight (and short of
// it) when it is out of reach. `limb` is the upper part (ArmL..LegR).
void reachTo(BodyPose& pose, BlockBody::Part limb, const Vec3& world);
// THE RAGDOLL for a pose: the ten blocks as bodies where the pose puts them (67 kg, the torso most of it), joined
// at the neck, shoulders and hips with a person's limits (arms 85° each way, legs 60°, the head 35°) and hinged at
// the elbows and knees, which fold one way only (straight to kElbowMax / kKneeMax).
std::vector<PhysicsWorld::RagdollPart> blockRagdoll(const BodyPose& pose);
// Where a part's joint is now, from its block's transform (the hip of a thigh, the shoulder of an upper arm, the
// knee of a shin): what holds a ragdoll together, for tests.
Vec3 jointOf(BlockBody::Part part, const PartTransform& block, Real scale);
// The far end of a limb part (the elbow of an upper arm, the foot of a shin).
Vec3 endOf(BlockBody::Part part, const PartTransform& block, Real scale);

}  // namespace engine

#endif
