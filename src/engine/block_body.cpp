#include "block_body.h"

#include "mesh_builder.h"

#include <algorithm>
#include <cmath>

namespace engine {

const std::array<BlockBody::PartSpec, BlockBody::kParts>& BlockBody::parts() {
    // procgen/person.cpp's blocks: legs from the hips (y -0.05), arms from the shoulders (y 0.50), the head on the
    // neck (y 0.57), the torso up from the pelvis. Each limb is its old block cut in two: the arm's 0.62 m at the
    // elbow (0.31 + 0.31), the leg's 0.85 m at the knee (0.43 thigh + 0.42 shin).
    static const std::array<PartSpec, kParts> p = {{
        {Vec3(0.44, 0.60, 0.26), Vec3(0, -0.05, 0), Vec3(0, 0.30, 0), -1},             // torso
        {Vec3(0.26, 0.26, 0.26), Vec3(0, 0.57, 0), Vec3(0, 0.13, 0), Torso},           // head
        {Vec3(0.12, 0.31, 0.16), Vec3(0.29, 0.50, 0), Vec3(0, -0.155, 0), Torso},      // left upper arm (+x)
        {Vec3(0.12, 0.31, 0.16), Vec3(-0.29, 0.50, 0), Vec3(0, -0.155, 0), Torso},     // right upper arm
        {Vec3(0.17, 0.43, 0.22), Vec3(0.115, -0.05, 0), Vec3(0, -0.215, 0), Torso},    // left thigh
        {Vec3(0.17, 0.43, 0.22), Vec3(-0.115, -0.05, 0), Vec3(0, -0.215, 0), Torso},   // right thigh
        {Vec3(0.12, 0.31, 0.16), Vec3(0.29, 0.19, 0), Vec3(0, -0.155, 0), ArmL},       // left forearm, from the elbow
        {Vec3(0.12, 0.31, 0.16), Vec3(-0.29, 0.19, 0), Vec3(0, -0.155, 0), ArmR},      // right forearm
        {Vec3(0.17, 0.42, 0.22), Vec3(0.115, -0.48, 0), Vec3(0, -0.21, 0), LegL},      // left shin, from the knee
        {Vec3(0.17, 0.42, 0.22), Vec3(-0.115, -0.48, 0), Vec3(0, -0.21, 0), LegR},     // right shin
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

namespace {

Real partLength(BlockBody::Part part) { return 2.0 * BlockBody::parts()[static_cast<std::size_t>(part)].rest.length(); }

// Which way a limb's middle joint points, in the body frame: the kneecap forward, the point of the elbow back (and a
// little out and down). With a share of a second way in it, so a limb pointing straight along the first still has
// a side to bend to: a thigh out in front bends its knee up, an arm reaching back drops its elbow.
Vec3 poleOf(int limb) {
    if (limb >= 2) return Vec3(0, 0.35, 1);
    const Real side = limb == 0 ? 1.0 : -1.0;   // (the L side is +x)
    return Vec3(0.25 * side, -0.35, -1);
}

}  // namespace

LimbSolve solveLimb(const BodyPose& pose, int limb) {
    const BlockBody::Part up = BlockBody::upper(limb), lo = BlockBody::lower(limb);
    const Real s = pose.scale;
    const Real a = partLength(up) * s, b = partLength(lo) * s;
    const Real maxBend = limb >= 2 ? kKneeMax : kElbowMax;
    const Real th = std::clamp(pose.bend(limb), Real(0), maxBend);
    const Vec3 d = pose.dir(limb).lengthSquared() > 1e-10 ? normalize(pose.dir(limb)) : Vec3(0, -1, 0);
    // the hand or foot: the two halves with the joint folded th from straight (the interior angle is pi - th)
    const Real c = std::sqrt(std::max(Real(1e-8), a * a + b * b + 2 * a * b * std::cos(th)));
    // the pole off the limb's line
    const Vec3 pole = poleOf(limb);
    Vec3 side = pole - d * dot(pole, d);
    side = side.lengthSquared() > 1e-8 ? normalize(side) : Vec3(0, 0, limb >= 2 ? 1 : -1);
    // the upper half leans off the line toward the pole by alpha (law of cosines at the root)
    const Real ca = std::clamp((a * a + c * c - b * b) / (2 * a * c), Real(-1), Real(1));
    const Real sa = std::sqrt(std::max(Real(0), 1 - ca * ca));
    const Vec3 u = d * ca + side * sa;
    LimbSolve out;
    const Vec3 rootBody = BlockBody::parts()[static_cast<std::size_t>(up)].joint * s;
    out.root = pose.position + pose.orientation.rotate(rootBody);
    out.mid = pose.position + pose.orientation.rotate(rootBody + u * a);
    out.end = pose.position + pose.orientation.rotate(rootBody + d * c);
    out.upperDir = pose.orientation.rotate(u);
    const Vec3 l = out.end - out.mid;
    out.lowerDir = l.lengthSquared() > 1e-12 ? normalize(l) : out.upperDir;
    // turning the lower half about (side x d) swings it away from the pole: the joint folds further
    out.hingeAxis = normalize(pose.orientation.rotate(cross(side, d)));
    return out;
}

Quat frameOf(const Vec3& x, const Vec3& y) {
    const Vec3 X = normalize(x), Y = normalize(y), Z = cross(X, Y);
    Mat4 m;   // (column-vector: the columns are where x, y, z go)
    m.m[0][0] = X.x; m.m[1][0] = X.y; m.m[2][0] = X.z;
    m.m[0][1] = Y.x; m.m[1][1] = Y.y; m.m[2][1] = Y.z;
    m.m[0][2] = Z.x; m.m[1][2] = Z.y; m.m[2][2] = Z.z;
    return Quat::fromRotationMatrix(m);
}

Real foldOf(const Quat& upper, const Quat& lower) {
    const Quat r = upper.conjugate() * lower;   // the lower half in the upper's frame: a turn about its x
    Real a = 2.0 * std::atan2(r.x, r.w);
    if (a > 3.14159265358979) a -= 6.283185307179586;
    if (a < -3.14159265358979) a += 6.283185307179586;
    return a;
}

std::array<PartTransform, BlockBody::kParts> poseBlocks(const BodyPose& pose) {
    std::array<PartTransform, BlockBody::kParts> out;
    const auto& parts = BlockBody::parts();
    const Real s = pose.scale;
    for (int i : {BlockBody::Torso, BlockBody::Head}) {
        const BlockBody::PartSpec& p = parts[static_cast<std::size_t>(i)];
        PartTransform& t = out[static_cast<std::size_t>(i)];
        const Quat turn = i == BlockBody::Head ? pose.neck : Quat::identity();   // (the head about its neck)
        t.position = pose.position + pose.orientation.rotate(p.joint * s + turn.rotate(p.rest * s));
        t.orientation = pose.orientation * turn;
        t.size = p.size * s;
    }
    for (int k = 0; k < BlockBody::kLimbs; ++k) {
        const LimbSolve l = solveLimb(pose, k);
        const BlockBody::Part up = BlockBody::upper(k), lo = BlockBody::lower(k);
        // THE LIMB'S FRAME: x the hinge's axis, -y down the upper half, so the lower half is the upper turned about
        // its own x by the fold -- what the skin bends by (skinnedLimb) and what a ragdoll's hinge reads back (foldOf)
        const Real fold = std::acos(std::clamp(dot(l.upperDir, l.lowerDir), Real(-1), Real(1)));
        PartTransform& tu = out[static_cast<std::size_t>(up)];
        tu.position = (l.root + l.mid) * 0.5;
        tu.orientation = frameOf(l.hingeAxis, l.upperDir * -1.0);
        tu.size = parts[static_cast<std::size_t>(up)].size * s;
        PartTransform& tl = out[static_cast<std::size_t>(lo)];
        tl.position = l.mid + l.lowerDir * (partLength(lo) * s * 0.5);
        tl.orientation = tu.orientation * Quat::fromAxisAngle(Vec3(1, 0, 0), fold);
        tl.size = parts[static_cast<std::size_t>(lo)].size * s;
    }
    return out;
}

std::vector<PhysicsWorld::RagdollPart> blockRagdoll(const BodyPose& pose) {
    const std::array<PartTransform, BlockBody::kParts> blocks = poseBlocks(pose);
    const auto& parts = BlockBody::parts();
    // 67 kg: the torso 30, the head 5, an upper arm 2.2 and a forearm 1.8, a thigh 7.5 and a shin 4.5
    static const Real kMass[BlockBody::kParts] = {30, 5, 2.2, 2.2, 7.5, 7.5, 1.8, 1.8, 4.5, 4.5};
    static const Real kSwing[BlockBody::kParts] = {0, 35, 85, 85, 60, 60, 0, 0, 0, 0};
    static const Real kTwist[BlockBody::kParts] = {0, 30, 40, 40, 20, 20, 0, 0, 0, 0};
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
    // the elbows and knees: hinges about the axis they fold on, from straight to their most, the pose's fold now
    for (int k = 0; k < BlockBody::kLimbs; ++k) {
        const LimbSolve l = solveLimb(pose, k);
        PhysicsWorld::RagdollPart& r = out[static_cast<std::size_t>(BlockBody::lower(k))];
        const Real bend = std::acos(std::clamp(dot(l.upperDir, l.lowerDir), Real(-1), Real(1)));
        r.hinge = true;
        r.hingeAxis = l.hingeAxis;
        r.hingeMinDeg = -bend * 57.29577951308232 - 2.0;   // (a hair past straight)
        r.hingeMaxDeg = ((k >= 2 ? kKneeMax : kElbowMax) - bend) * 57.29577951308232;
        r.hingeFriction = (k >= 2 ? kKneeFriction : kElbowFriction) * pose.scale;
    }
    return out;
}

Vec3 jointOf(BlockBody::Part part, const PartTransform& block, Real scale) {
    const BlockBody::PartSpec& p = BlockBody::parts()[static_cast<std::size_t>(part)];
    // the block hangs from its joint along its own -y (limbs) or stands on it along +y (torso, head)
    const Vec3 along = part == BlockBody::Torso || part == BlockBody::Head ? Vec3(0, 1, 0) : Vec3(0, -1, 0);
    return block.position - block.orientation.rotate(along) * (p.rest.length() * scale);
}

Vec3 endOf(BlockBody::Part part, const PartTransform& block, Real scale) {
    const BlockBody::PartSpec& p = BlockBody::parts()[static_cast<std::size_t>(part)];
    return block.position + block.orientation.rotate(Vec3(0, -1, 0)) * (p.rest.length() * scale);
}

Vec3 reachToward(const BodyPose& pose, BlockBody::Part limb, const Vec3& world) {
    const BlockBody::PartSpec& p = BlockBody::parts()[static_cast<std::size_t>(limb)];
    const Vec3 joint = pose.position + pose.orientation.rotate(p.joint * pose.scale);
    const Vec3 d = pose.orientation.conjugate().rotate(world - joint);
    return d.lengthSquared() > 1e-10 ? normalize(d) : Vec3(0, -1, 0);
}

void reachTo(BodyPose& pose, BlockBody::Part limb, const Vec3& world) {
    const int k = limb - BlockBody::ArmL;
    if (k < 0 || k >= BlockBody::kLimbs) return;
    const BlockBody::PartSpec& p = BlockBody::parts()[static_cast<std::size_t>(limb)];
    const Vec3 joint = pose.position + pose.orientation.rotate(p.joint * pose.scale);
    pose.dir(k) = reachToward(pose, limb, world);
    const Real a = partLength(limb) * pose.scale, b = partLength(BlockBody::lower(k)) * pose.scale;
    const Real c = std::clamp((world - joint).length(), std::fabs(a - b) + 1e-4, a + b);
    // the fold that makes the halves span c: c^2 = a^2 + b^2 + 2ab cos(bend)
    pose.bend(k) = std::acos(std::clamp((c * c - a * a - b * b) / (2 * a * b), Real(-1), Real(1)));
}

// THE SKIN: one tube from the root joint to the hand or foot, its cross-section the blocks' (eased from the upper's
// to the lower's across the joint), its rings close round the joint. Straight it runs down -y; bent, each ring turns
// about the hinge (x, through the joint) by its share of the fold -- none above the bend, all of it below, eased
// across 2r round it. For a hinge this is what dual-quaternion skinning does: the rings rotate, they don't blend
// their corners (linear blending pinches a bent joint thin, the "candy wrapper"), so the knee keeps its thickness.
RenderMesh skinnedLimb(int limb, Real fold, const Vec3& colour, Real girth) {
    const BlockBody::Part up = BlockBody::upper(limb), lo = BlockBody::lower(limb);
    const Vec3 thick(girth, 1, girth);   // (the cross-section only: the bones keep their lengths)
    const Vec3 su = BlockBody::parts()[static_cast<std::size_t>(up)].size * thick;
    const Vec3 sl = BlockBody::parts()[static_cast<std::size_t>(lo)].size * thick;
    const Real a = partLength(up), b = partLength(lo);
    const Real r = 0.75 * std::max(su.x, su.z) * 0.5;   // the bend's half width
    const Real th = std::clamp(fold, Real(0), limb >= 2 ? kKneeMax : kElbowMax);
    // the rings' places down the straight limb: the ends, and nine across the bend
    std::vector<Real> at = {0};
    constexpr int kBendRings = 9;
    for (int i = 0; i < kBendRings; ++i) at.push_back(a - r + 2 * r * i / (kBendRings - 1));
    at.push_back(a + b);
    auto weight = [&](Real d) {   // the share of the fold a ring at d takes
        const Real t = std::clamp((d - (a - r)) / (2 * r), Real(0), Real(1));
        return t * t * (3 - 2 * t);
    };
    auto place = [&](Real d, Real x, Real z) {
        const Real w = weight(d);
        const Real y = -d + a, ang = w * th;   // (about the joint at y = -a)
        const Real c = std::cos(ang), sn = std::sin(ang);
        return Vec3(x, y * c - z * sn - a, y * sn + z * c);
    };
    auto half = [&](Real d) {   // the cross-section's half size at d
        const Real w = weight(d);
        return Vec3((su.x + (sl.x - su.x) * w) * 0.5, 0, (su.z + (sl.z - su.z) * w) * 0.5);
    };
    auto centre = [&](Real d) { return place(d, 0, 0); };
    RenderMesh m;
    // the four sides, ring to ring: corners (+x+z, -x+z, -x-z, +x-z) round the tube
    const Real cx[4] = {1, -1, -1, 1}, cz[4] = {1, 1, -1, -1};
    for (std::size_t i = 0; i + 1 < at.size(); ++i) {
        const Real d0 = at[i], d1 = at[i + 1];
        const Vec3 h0 = half(d0), h1 = half(d1);
        for (int k = 0; k < 4; ++k) {
            const int n = (k + 1) % 4;
            const Vec3 p0 = place(d0, cx[k] * h0.x, cz[k] * h0.z), p1 = place(d0, cx[n] * h0.x, cz[n] * h0.z);
            const Vec3 p2 = place(d1, cx[n] * h1.x, cz[n] * h1.z), p3 = place(d1, cx[k] * h1.x, cz[k] * h1.z);
            Vec3 nrm = cross(p2 - p0, p3 - p1);
            const Vec3 out = (p0 + p1 + p2 + p3) * 0.25 - (centre(d0) + centre(d1)) * 0.5;
            if (dot(nrm, out) < 0) nrm = nrm * -1.0;
            // (a face pinched shut on the inside of a deep fold faces out from the tube: every fold the same faces,
            // so a walk cycle's poses share their topology)
            if (nrm.lengthSquared() < 1e-16) nrm = out.lengthSquared() > 1e-16 ? out : Vec3(0, 0, 1);
            MeshBuilder::emitQuad(m, p0, p1, p2, p3, normalize(nrm), colour);
        }
    }
    // the caps: the root (up, inside the shoulder or hip) and the hand or foot
    for (int e = 0; e < 2; ++e) {
        const Real d = e ? at.back() : 0.0;
        const Vec3 h = half(d);
        Vec3 q[4];
        for (int k = 0; k < 4; ++k) q[k] = place(d, cx[k] * h.x, cz[k] * h.z);
        const Vec3 nrm = e ? normalize(place(d, 0, 0) - place(d - 0.01, 0, 0)) : Vec3(0, 1, 0);
        MeshBuilder::emitQuad(m, q[0], q[1], q[2], q[3], nrm, colour);
    }
    return m;
}

}  // namespace engine
