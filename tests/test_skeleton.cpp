#include "test_framework.h"
#include "../src/engine/anim/mannequin.h"
#include "../src/engine/anim/skeleton.h"
#include "../src/engine/debug_draw.h"

using namespace engine;
namespace anim = engine::anim;   // Skeleton/Pose live here (procgen owns engine::Skeleton)

namespace {

Vec3 origin(const Mat4& m) { return m.transformPoint(Vec3(0, 0, 0)); }

// A 3-joint arm: root at the origin, elbow +1x, wrist +1x again.
anim::Skeleton makeArm() {
    anim::Skeleton sk;
    Transform t;
    int root = sk.addJoint("root", -1, t);
    t.position = Vec3(1, 0, 0);
    int elbow = sk.addJoint("elbow", root, t);
    sk.addJoint("wrist", elbow, t);
    sk.finalize();
    return sk;
}

}  // namespace

TEST_CASE(skeleton_rejects_forward_parent_references) {
    anim::Skeleton sk;
    CHECK(sk.addJoint("a", -1, {}) == 0);
    CHECK(sk.addJoint("b", 5, {}) == -1);    // parent doesn't exist yet
    CHECK(sk.addJoint("c", -2, {}) == -1);   // nonsense parent
    CHECK(sk.addJoint("d", 0, {}) == 1);
    CHECK(sk.jointCount() == 2);
    CHECK(sk.find("d") == 1);
    CHECK(sk.find("nope") == -1);
}

TEST_CASE(skeleton_world_matrices_compose_parent_chains) {
    anim::Skeleton sk = makeArm();
    anim::Pose bind = anim::Pose::bind(sk);
    auto world = anim::worldJointMatrices(sk, bind);
    CHECK_APPROX(origin(world[0]).x, 0.0, 1e-9);
    CHECK_APPROX(origin(world[1]).x, 1.0, 1e-9);
    CHECK_APPROX(origin(world[2]).x, 2.0, 1e-9);
}

TEST_CASE(skeleton_rotating_a_parent_moves_the_children) {
    anim::Skeleton sk = makeArm();
    anim::Pose pose = anim::Pose::bind(sk);
    // Bend the elbow 90 degrees about +Z: the wrist swings from +x to +y.
    pose.locals[1].orientation = Quat::fromAxisAngle(Vec3(0, 0, 1), M_PI / 2.0);
    auto world = anim::worldJointMatrices(sk, pose);
    Vec3 wrist = origin(world[2]);
    CHECK_APPROX(wrist.x, 1.0, 1e-9);   // elbow stays put
    CHECK_APPROX(wrist.y, 1.0, 1e-9);   // forearm now points up
    CHECK_APPROX(origin(world[1]).x, 1.0, 1e-9);
}

TEST_CASE(skeleton_skinning_matrices_are_identity_at_bind) {
    anim::Skeleton sk = makeArm();
    anim::Pose bind = anim::Pose::bind(sk);
    auto skin = anim::skinningMatrices(sk, anim::worldJointMatrices(sk, bind));
    // A bind-pose vertex must not move: world * inverseBind == identity.
    Vec3 probe(0.7, 0.3, -0.2);
    for (const Mat4& m : skin) {
        Vec3 out = m.transformPoint(probe);
        CHECK_APPROX((out - probe).length(), 0.0, 1e-9);
    }
}

TEST_CASE(pose_lerp_interpolates_and_hits_endpoints) {
    anim::Skeleton sk = makeArm();
    anim::Pose a = anim::Pose::bind(sk);
    anim::Pose b = anim::Pose::bind(sk);
    b.locals[1].position = Vec3(3, 0, 0);
    b.locals[1].orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), M_PI / 2.0);

    anim::Pose start = anim::lerpPose(a, b, 0.0);
    anim::Pose mid = anim::lerpPose(a, b, 0.5);
    anim::Pose end = anim::lerpPose(a, b, 1.0);
    CHECK_APPROX(start.locals[1].position.x, 1.0, 1e-9);
    CHECK_APPROX(mid.locals[1].position.x, 2.0, 1e-9);
    CHECK_APPROX(end.locals[1].position.x, 3.0, 1e-9);
    CHECK_APPROX(mid.locals[1].orientation.length(), 1.0, 1e-9);   // slerp stays unit
}

TEST_CASE(skeleton_debug_draw_emits_one_line_per_bone) {
    anim::Skeleton sk = makeArm();
    DebugDraw dd;
    anim::debugDrawSkeleton(dd, sk, anim::worldJointMatrices(sk, anim::Pose::bind(sk)),
                      Vec3(1, 1, 0));
    CHECK(dd.lineCount() == 2);   // root has no bone; elbow + wrist do
}

TEST_CASE(mannequin_builds_a_plausible_upright_biped) {
    anim::Mannequin m = anim::buildMannequin(1.8);
    CHECK(m.skeleton.jointCount() == 17);
    CHECK(!m.parts.empty());
    for (const auto& part : m.parts) {
        CHECK(part.joint >= 0);
        CHECK(part.joint < m.skeleton.jointCount());
        CHECK(!part.mesh.vertices.empty());
    }

    auto world = m.skeleton.bindWorld();
    int head = m.skeleton.find("head");
    int pelvis = m.skeleton.find("pelvis");
    int ankleL = m.skeleton.find("ankle.L");
    int wristL = m.skeleton.find("wrist.L");
    int wristR = m.skeleton.find("wrist.R");
    CHECK(head >= 0);
    CHECK(origin(world[head]).y > origin(world[pelvis]).y);      // head above hips
    CHECK(origin(world[ankleL]).y < 0.15);                        // feet near floor
    CHECK(origin(world[ankleL]).y > 0.0);
    // T-pose symmetry: wrists mirrored about x=0 at the same height.
    CHECK_APPROX(origin(world[wristL]).x, -origin(world[wristR]).x, 1e-9);
    CHECK_APPROX(origin(world[wristL]).y, origin(world[wristR]).y, 1e-9);

    // Height scales the whole rig.
    anim::Mannequin tall = anim::buildMannequin(2.7);
    auto tallWorld = tall.skeleton.bindWorld();
    CHECK_APPROX(origin(tallWorld[head]).y, origin(world[head]).y * 1.5, 1e-6);
}

TEST_CASE(mannequin_pose_moves_the_right_parts_only) {
    anim::Mannequin m = anim::buildMannequin();
    anim::Pose bind = anim::Pose::bind(m.skeleton);
    RenderMesh bindMesh = anim::bakeMannequinMesh(m, bind);
    CHECK(!bindMesh.vertices.empty());
    CHECK(bindMesh.indices.size() % 3 == 0);

    // Raise the LEFT arm: rotate shoulder.L 90 degrees about +Z (T-pose arm
    // +x swings to +y).
    anim::Pose wave = bind;
    int shoulderL = m.skeleton.find("shoulder.L");
    wave.locals[shoulderL].orientation =
        Quat::fromAxisAngle(Vec3(0, 0, 1), M_PI / 2.0);
    auto bindWorld = anim::worldJointMatrices(m.skeleton, bind);
    auto waveWorld = anim::worldJointMatrices(m.skeleton, wave);

    int wristL = m.skeleton.find("wrist.L");
    int wristR = m.skeleton.find("wrist.R");
    // The left wrist rose well above its T-pose height...
    CHECK(origin(waveWorld[wristL]).y > origin(bindWorld[wristL]).y + 0.3);
    // ...and the right wrist didn't move at all.
    CHECK_APPROX((origin(waveWorld[wristR]) - origin(bindWorld[wristR])).length(),
                 0.0, 1e-9);

    // The baked mesh changed with the pose (same size, different vertices).
    RenderMesh waveMesh = anim::bakeMannequinMesh(m, wave);
    CHECK(waveMesh.vertices.size() == bindMesh.vertices.size());
    double moved = 0;
    for (std::size_t i = 0; i < waveMesh.vertices.size(); i++)
        moved += (waveMesh.vertices[i].position - bindMesh.vertices[i].position)
                     .length();
    CHECK(moved > 1.0);
}
