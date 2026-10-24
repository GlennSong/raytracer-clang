#include "block_body.h"

#include <algorithm>
#include <cmath>

namespace engine {

const std::array<BlockBody::PartSpec, BlockBody::kParts>& BlockBody::parts() {
    // procgen/person.cpp's blocks: legs from the hips (y -0.05), arms from the shoulders (y 0.50), the head on the
    // neck (y 0.57), the torso up from the pelvis
    static const std::array<PartSpec, kParts> p = {{
        {Vec3(0.44, 0.60, 0.26), Vec3(0, -0.05, 0), Vec3(0, 0.30, 0), -1},        // torso
        {Vec3(0.26, 0.26, 0.26), Vec3(0, 0.57, 0), Vec3(0, 0.13, 0), Torso},      // head
        {Vec3(0.12, 0.62, 0.16), Vec3(0.29, 0.50, 0), Vec3(0, -0.31, 0), Torso},  // left arm (+x)
        {Vec3(0.12, 0.62, 0.16), Vec3(-0.29, 0.50, 0), Vec3(0, -0.31, 0), Torso}, // right arm
        {Vec3(0.17, 0.85, 0.22), Vec3(0.115, -0.05, 0), Vec3(0, -0.425, 0), Torso},   // left leg
        {Vec3(0.17, 0.85, 0.22), Vec3(-0.115, -0.05, 0), Vec3(0, -0.425, 0), Torso},  // right leg
    }};
    return p;
}

Quat hangAlong(const Vec3& dir) {
    // the shortest rotation from -y to dir
    const Vec3 from(0, -1, 0);
    const Vec3 to = normalize(dir);
    const Real c = dot(from, to);
    if (c > 0.99999) return Quat::identity();
    if (c < -0.99999) return Quat::fromAxisAngle(Vec3(1, 0, 0), 3.14159265358979);
    const Vec3 axis = cross(from, to);
    return Quat::fromAxisAngle(axis, std::acos(std::clamp(c, Real(-1), Real(1))));
}

std::array<PartTransform, BlockBody::kParts> poseBlocks(const BodyPose& pose) {
    std::array<PartTransform, BlockBody::kParts> out;
    const auto& parts = BlockBody::parts();
    const Real s = pose.scale;
    const Vec3 dirs[BlockBody::kParts] = {Vec3(0, 1, 0), Vec3(0, 1, 0), pose.armL, pose.armR, pose.legL, pose.legR};
    for (int i = 0; i < BlockBody::kParts; ++i) {
        const BlockBody::PartSpec& p = parts[static_cast<std::size_t>(i)];
        Quat local;
        Vec3 offset;
        if (i == BlockBody::Torso || i == BlockBody::Head) {
            local = Quat::identity();
            offset = p.rest;
        } else {
            const Vec3 d = dirs[i].lengthSquared() > 1e-10 ? normalize(dirs[i]) : Vec3(0, -1, 0);
            local = hangAlong(d);
            offset = d * p.rest.length();
        }
        PartTransform& t = out[static_cast<std::size_t>(i)];
        t.position = pose.position + pose.orientation.rotate((p.joint + offset) * s);
        t.orientation = pose.orientation * local;
        t.size = p.size * s;
    }
    return out;
}

std::vector<PhysicsWorld::RagdollPart> blockRagdoll(const BodyPose& pose) {
    const std::array<PartTransform, BlockBody::kParts> blocks = poseBlocks(pose);
    const auto& parts = BlockBody::parts();
    static const Real kMass[BlockBody::kParts] = {30, 5, 4, 4, 12, 12};
    static const Real kSwing[BlockBody::kParts] = {0, 35, 85, 85, 60, 60};
    static const Real kTwist[BlockBody::kParts] = {0, 30, 40, 40, 20, 20};
    std::vector<PhysicsWorld::RagdollPart> out(BlockBody::kParts);
    for (int i = 0; i < BlockBody::kParts; ++i) {
        const PartTransform& b = blocks[static_cast<std::size_t>(i)];
        PhysicsWorld::RagdollPart& r = out[static_cast<std::size_t>(i)];
        r.halfExtent = b.size * 0.5;
        r.position = b.position;
        r.orientation = b.orientation;
        r.mass = kMass[i];
        r.parent = parts[static_cast<std::size_t>(i)].parent;
        r.joint = jointOf(static_cast<BlockBody::Part>(i), b, pose.scale);
        r.twistAxis = normalize(b.position - r.joint);
        r.swingDeg = kSwing[i];
        r.twistDeg = kTwist[i];
    }
    return out;
}

Vec3 jointOf(BlockBody::Part part, const PartTransform& block, Real scale) {
    const BlockBody::PartSpec& p = BlockBody::parts()[static_cast<std::size_t>(part)];
    // the block hangs from its joint along its own -y (limbs) or stands on it along +y (torso, head)
    const Vec3 along = part == BlockBody::Torso || part == BlockBody::Head ? Vec3(0, 1, 0) : Vec3(0, -1, 0);
    return block.position - block.orientation.rotate(along) * (p.rest.length() * scale);
}

Vec3 reachToward(const BodyPose& pose, BlockBody::Part limb, const Vec3& world) {
    const BlockBody::PartSpec& p = BlockBody::parts()[static_cast<std::size_t>(limb)];
    const Vec3 joint = pose.position + pose.orientation.rotate(p.joint * pose.scale);
    const Vec3 d = pose.orientation.conjugate().rotate(world - joint);
    return d.lengthSquared() > 1e-10 ? normalize(d) : Vec3(0, -1, 0);
}

}  // namespace engine
