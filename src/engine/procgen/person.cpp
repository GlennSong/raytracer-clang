#include "person.h"

#include "../block_body.h"
#include "../mesh_builder.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// Deterministic outfits: shirt / pants / skin.
const PersonOutfit kOutfits[] = {
    {{0.62, 0.35, 0.25}, {0.22, 0.24, 0.30}, {0.87, 0.68, 0.55}},
    {{0.30, 0.42, 0.58}, {0.16, 0.16, 0.18}, {0.55, 0.38, 0.28}},
    {{0.38, 0.50, 0.32}, {0.30, 0.26, 0.22}, {0.76, 0.56, 0.42}},
    {{0.55, 0.48, 0.30}, {0.24, 0.28, 0.36}, {0.42, 0.29, 0.22}},
    {{0.45, 0.32, 0.48}, {0.20, 0.20, 0.24}, {0.87, 0.68, 0.55}},
    {{0.60, 0.58, 0.55}, {0.28, 0.22, 0.20}, {0.64, 0.46, 0.34}},
    {{0.72, 0.62, 0.28}, {0.18, 0.22, 0.28}, {0.55, 0.38, 0.28}},
    {{0.35, 0.55, 0.55}, {0.26, 0.24, 0.22}, {0.76, 0.56, 0.42}},
    {{0.66, 0.30, 0.34}, {0.22, 0.26, 0.24}, {0.87, 0.68, 0.55}},
    {{0.42, 0.38, 0.60}, {0.20, 0.18, 0.16}, {0.64, 0.46, 0.34}},
    {{0.52, 0.56, 0.62}, {0.30, 0.30, 0.34}, {0.42, 0.29, 0.22}},
    {{0.58, 0.44, 0.36}, {0.24, 0.20, 0.26}, {0.76, 0.56, 0.42}},
    // LAST SLOT: the PLAYER's outfit -- a vivid red jacket over near-black trousers, deliberately absent from the muted
    // walker palette above so the player reads instantly in a crowd (citysim's personOutfitCount() excludes it)
    {{0.88, 0.12, 0.10}, {0.10, 0.10, 0.12}, {0.87, 0.68, 0.55}},
};
constexpr int kNumOutfits = static_cast<int>(sizeof(kOutfits) / sizeof(kOutfits[0]));

// A coloured box centred at `c`, PITCHED about the X-axis line y = pivotY (a limb about its hip or shoulder)
void addBoxPitched(RenderMesh& out, const Vec3& size, const Vec3& c, const Vec3& color, Real pivotY, Real angle) {
    MeshBuilder::emitBox(out, MeshFrame::centred(c, size).rotatedAbout(Vec3(0, pivotY, 0), Vec3(1, 0, 0), angle), color);
}

}  // namespace

int personOutfitTotal() { return kNumOutfits; }

const PersonOutfit& personOutfit(int outfit) { return kOutfits[((outfit % kNumOutfits) + kNumOutfits) % kNumOutfits]; }

RenderMesh buildStandingPerson(Real swing, int outfit) {
    const PersonOutfit& o = personOutfit(outfit);
    RenderMesh m;
    // hips at y = -0.05, shoulders at y = 0.50; legs swing opposed, arms counter-swing at ~70%
    const Real hipY = -0.05, shoulderY = 0.50;
    addBoxPitched(m, Vec3(0.17, 0.85, 0.22), Vec3(-0.115, -0.475, 0), o.pants, hipY, swing);         // left leg
    addBoxPitched(m, Vec3(0.17, 0.85, 0.22), Vec3(0.115, -0.475, 0), o.pants, hipY, -swing);         // right leg
    MeshBuilder::emitBox(m, MeshFrame::centred(Vec3(0, 0.25, 0), Vec3(0.44, 0.60, 0.26)), o.shirt);  // torso
    addBoxPitched(m, Vec3(0.12, 0.62, 0.16), Vec3(-0.29, 0.19, 0), o.shirt, shoulderY, -swing * 0.7);  // left arm
    addBoxPitched(m, Vec3(0.12, 0.62, 0.16), Vec3(0.29, 0.19, 0), o.shirt, shoulderY, swing * 0.7);    // right arm
    MeshBuilder::emitBox(m, MeshFrame::centred(Vec3(0, 0.70, 0), Vec3(0.26, 0.26, 0.26)), o.skin);   // head
    return m;
}

RenderMesh buildPosedPerson(const BodyPose& pose, int outfit) { return buildPosedPerson(pose, personOutfit(outfit)); }

RenderMesh buildPosedPerson(const BodyPose& pose, const PersonOutfit& o) {
    RenderMesh m;
    const std::array<PartTransform, BlockBody::kParts> blocks = poseBlocks(pose);
    for (int i : {BlockBody::Torso, BlockBody::Head}) {
        const PartTransform& b = blocks[static_cast<std::size_t>(i)];
        MeshBuilder::emitBox(m, MeshFrame::centred(b.position, b.size, b.orientation.rotate(Vec3(1, 0, 0)),
                                                   b.orientation.rotate(Vec3(0, 1, 0))),
                             i == BlockBody::Head ? o.skin : o.shirt);
    }
    // each limb one skinned tube at its exact fold (block_body.h skinnedLimb), set at its shoulder or hip
    for (int k = 0; k < BlockBody::kLimbs; ++k) {
        const BlockBody::Part up = BlockBody::upper(k);
        const PartTransform& u = blocks[static_cast<std::size_t>(up)];
        const PartTransform& l = blocks[static_cast<std::size_t>(BlockBody::lower(k))];
        const RenderMesh tube = skinnedLimb(k, foldOf(u.orientation, l.orientation), BlockBody::isLeg(up) ? o.pants : o.shirt);
        const Vec3 root = jointOf(up, u, pose.scale);
        const uint32_t base = static_cast<uint32_t>(m.vertices.size());
        for (Vertex v : tube.vertices) {
            v.position = root + u.orientation.rotate(v.position * pose.scale);
            v.normal = u.orientation.rotate(v.normal);
            v.tangent = u.orientation.rotate(v.tangent);
            m.vertices.push_back(v);
        }
        for (uint32_t i : tube.indices) m.indices.push_back(base + i);   // (a rigid move: the winding holds)
    }
    return m;
}

BodyPose walkBodyPose(Real swing) {
    BodyPose pose;
    // buildStandingPerson pitches the -x leg back by +swing and the +x (L) leg forward; the arms the other way at 70%
    auto pitched = [](Real a) { return Vec3(0, -std::cos(a), -std::sin(a)); };   // + back
    const Real legR = swing, legL = -swing, armR = -0.7 * swing, armL = 0.7 * swing;
    pose.legL = pitched(legL);
    pose.legR = pitched(legR);
    pose.armL = pitched(armL);
    pose.armR = pitched(armR);
    // the knee: soft under the body, folding as the leg trails (the push-off and the swing through)
    auto knee = [&](Real a) { return 0.06 + 0.7 * std::clamp(a / kMaxSwing, Real(0), Real(1)); };
    auto elbow = [&](Real a) { return 0.18 + 0.4 * std::clamp(-a / (0.7 * kMaxSwing), Real(0), Real(1)); };
    pose.kneeL = knee(legL);
    pose.kneeR = knee(legR);
    pose.elbowL = elbow(armL);
    pose.elbowR = elbow(armR);
    return pose;
}

bool advanceStride(Real& stridePhase, Real speed, Real dt) {
    if (speed <= kWalkAnimMinSpeed) return false;
    stridePhase += speed * dt / kStrideLength;
    stridePhase -= std::floor(stridePhase);
    return true;
}

Real strideSwing(Real stridePhase) { return kMaxSwing * std::sin(2.0 * 3.14159265358979 * stridePhase); }

Real walkPoseSwing(int pose) { return strideSwing(static_cast<Real>(pose) / kWalkPoses); }

int walkPoseIndex(Real& stridePhase, Real speed, Real dt) {
    if (!advanceStride(stridePhase, speed, dt)) return 0;
    return static_cast<int>(stridePhase * kWalkPoses) % kWalkPoses;
}

}  // namespace engine
