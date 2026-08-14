#include "test_framework.h"

#include "../src/engine/xr/xr_grasp.h"

#include <cmath>

using namespace engine;

// Grasp core (engine/xr/xr_grasp.h) — grip memory, hand memory, placement
// assist. All synthetic-stream tests: these thresholds define how grabbing
// feels on device, and every tuning iteration must be checkable on the host.

namespace {
bool nearVec(const Vec3& a, const Vec3& b, Real tol) {
    return (a - b).length() <= tol;
}
Real quatAngle(const Quat& a, const Quat& b) {
    const Real d = std::min(Real(1), std::fabs(a.normalized().dot(b.normalized())));
    return 2 * std::acos(d);
}
}  // namespace

// --- Grip memory ----------------------------------------------------------

TEST_CASE(grasp_opposed_needs_contacts_on_each_side) {
    // Thumb pressing +X, index pressing -X: a real grip.
    XrGripMemory grip;
    grip.capture({{1, Vec3(-0.05, 0, 0), Vec3(1, 0, 0)},
                  {2, Vec3(0.05, 0, 0), Vec3(-1, 0, 0)}},
                 Vec3(0, 0, 0), Quat::identity());
    CHECK(grip.contactCount() == 2);
    CHECK(grip.opposed());

    // Two fingers on the SAME face (a palm slap): never a grab.
    grip.capture({{1, Vec3(-0.05, 0.01, 0), Vec3(1, 0, 0)},
                  {2, Vec3(-0.05, -0.01, 0), Vec3(1, 0, 0)}},
                 Vec3(0, 0, 0), Quat::identity());
    CHECK(!grip.opposed());

    // One contact alone can't oppose anything.
    grip.capture({{1, Vec3(-0.05, 0, 0), Vec3(1, 0, 0)}}, Vec3(0, 0, 0),
                 Quat::identity());
    CHECK(!grip.opposed());

    // Perpendicular normals (finger on top + finger on side) don't qualify —
    // dot 0 is above the -0.3 threshold.
    grip.capture({{1, Vec3(0, 0.05, 0), Vec3(0, -1, 0)},
                  {2, Vec3(-0.05, 0, 0), Vec3(1, 0, 0)}},
                 Vec3(0, 0, 0), Quat::identity());
    CHECK(!grip.opposed());

    // A three-finger wrap where only one pair opposes still counts.
    grip.capture({{1, Vec3(0, 0.05, 0), Vec3(0, -1, 0)},
                  {2, Vec3(-0.05, 0, 0), Vec3(1, 0, 0)},
                  {3, Vec3(0.05, 0, 0), Vec3(-1, 0, 0)}},
                 Vec3(0, 0, 0), Quat::identity());
    CHECK(grip.opposed());
}

TEST_CASE(grasp_anchors_ride_the_object) {
    // Capture on an object at a pose, then move AND rotate the object; the
    // world anchors must follow it rigidly.
    XrGripMemory grip;
    const Vec3 p0(1, 1, 0);
    const Quat q0 = Quat::fromAxisAngle(Vec3(0, 1, 0), 0.4);
    const Vec3 tipWorld = p0 + q0.rotate(Vec3(0.05, 0, 0));
    grip.capture({{7, tipWorld, q0.rotate(Vec3(-1, 0, 0))}}, p0, q0);

    // Unmoved: anchor reproduces the capture point.
    CHECK(nearVec(grip.worldAnchor(0, p0, q0), tipWorld, 1e-9));

    // Object carried elsewhere and turned: anchor lands at the same
    // object-local spot in the new frame.
    const Vec3 p1(0, 2, -1);
    const Quat q1 = Quat::fromAxisAngle(Vec3(1, 0, 0), 1.1);
    CHECK(nearVec(grip.worldAnchor(0, p1, q1),
                  p1 + q1.rotate(Vec3(0.05, 0, 0)), 1e-9));
}

TEST_CASE(grasp_opening_measures_tip_drift_not_object_motion) {
    XrGripMemory grip;
    const Vec3 p0(0, 1, 0);
    const Quat q0 = Quat::identity();
    grip.capture({{1, Vec3(-0.05, 1, 0), Vec3(1, 0, 0)},
                  {2, Vec3(0.05, 1, 0), Vec3(-1, 0, 0)}},
                 p0, q0);

    // Tips still on their anchors: closed.
    CHECK(grip.opening({{1, Vec3(-0.05, 1, 0)}, {2, Vec3(0.05, 1, 0)}}, p0,
                       q0) < 1e-9);

    // Object moved WITH the hand, tips riding along: still closed. (This is
    // the carry case — object motion alone must never read as release.)
    const Vec3 p1(0.3, 1.2, 0.1);
    CHECK(grip.opening({{1, p1 + Vec3(-0.05, 0, 0)}, {2, p1 + Vec3(0.05, 0, 0)}},
                       p1, q0) < 1e-9);

    // Fingers spread 3cm off each anchor: opening reports the mean drift.
    const Real open = grip.opening(
        {{1, Vec3(-0.08, 1, 0)}, {2, Vec3(0.08, 1, 0)}}, p0, q0);
    CHECK(std::fabs(open - 0.03) < 1e-9);

    // Tracking dropout (no matching ids): must read 0, not "opened" — a
    // dropout mid-hold must not drop the object.
    CHECK(grip.opening({{9, Vec3(5, 5, 5)}}, p0, q0) == 0.0);
    CHECK(grip.opening({}, p0, q0) == 0.0);
}

// --- Hand memory ----------------------------------------------------------

TEST_CASE(grasp_hand_memory_coasts_through_dropout) {
    // Hand moving at a steady 1 m/s along X, tracked for 100ms at 90Hz,
    // then a 60ms dropout (within budget): the pose must CONTINUE the arc,
    // not freeze at the last tracked position.
    XrHandMemory mem;
    const Real dt = 1.0 / 90.0;
    Real t = 0;
    Vec3 p(0, 1, 0);
    for (int i = 0; i < 10; ++i) {
        mem.feed(true, p, Quat::identity(), t);
        t += dt;
        p.x += dt;  // 1 m/s
    }
    CHECK(mem.available());
    CHECK(!mem.coasting());
    const Real lastTrackedX = mem.position().x;

    for (int i = 0; i < 5; ++i) {  // ~56ms untracked
        mem.feed(false, Vec3(0, 0, 0), Quat::identity(), t);
        t += dt;
    }
    CHECK(mem.coasting());
    // Coasted forward, decayed below full extrapolation.
    CHECK(mem.position().x > lastTrackedX + 0.02);
    CHECK(mem.position().x < lastTrackedX + 0.06);
    CHECK(std::fabs(mem.position().y - 1.0) < 1e-6);
}

TEST_CASE(grasp_hand_memory_freezes_after_budget) {
    XrHandMemory mem;
    const Real dt = 1.0 / 90.0;
    Real t = 0;
    Vec3 p(0, 0, 0);
    for (int i = 0; i < 10; ++i) {
        mem.feed(true, p, Quat::identity(), t);
        t += dt;
        p.x += dt;
    }
    // Long dropout: run well past the 150ms budget.
    for (int i = 0; i < 40; ++i) {
        mem.feed(false, Vec3(0, 0, 0), Quat::identity(), t);
        t += dt;
    }
    const Vec3 frozen = mem.position();
    mem.feed(false, Vec3(0, 0, 0), Quat::identity(), t);
    t += dt;
    CHECK(nearVec(mem.position(), frozen, 1e-12));  // dead stop, no creep
    // And the freeze point is bounded by the decayed budget, not a full
    // 40-frame extrapolation.
    CHECK(frozen.x < 0.35);
}

TEST_CASE(grasp_hand_memory_blends_on_reacquire) {
    XrHandMemory mem;
    const Real dt = 1.0 / 90.0;
    Real t = 0;
    for (int i = 0; i < 10; ++i) {
        mem.feed(true, Vec3(0, 1, 0), Quat::identity(), t);
        t += dt;
    }
    // Dropout while stationary, then tracking reappears 10cm away (the
    // classic reacquire jump).
    for (int i = 0; i < 5; ++i) {
        mem.feed(false, Vec3(0, 0, 0), Quat::identity(), t);
        t += dt;
    }
    const Vec3 target(0.1, 1, 0);
    mem.feed(true, target, Quat::identity(), t);
    // First reacquired frame: nowhere near the target yet (no teleport).
    CHECK(mem.position().x < 0.02);
    CHECK(!mem.coasting());
    // After the blend window it has converged.
    for (int i = 0; i < 10; ++i) {
        t += dt;
        mem.feed(true, target, Quat::identity(), t);
    }
    CHECK(nearVec(mem.position(), target, 1e-6));
}

TEST_CASE(grasp_hand_memory_unavailable_until_first_track) {
    XrHandMemory mem;
    CHECK(!mem.available());
    mem.feed(false, Vec3(0, 0, 0), Quat::identity(), 0.0);
    CHECK(!mem.available());
    CHECK(!mem.coasting());
    mem.feed(true, Vec3(1, 2, 3), Quat::identity(), 0.011);
    CHECK(mem.available());
    CHECK(nearVec(mem.position(), Vec3(1, 2, 3), 1e-12));
}

// --- Placement assist -----------------------------------------------------

TEST_CASE(grasp_nearest_axis_aligned_snaps_and_stays_right_handed) {
    // Identity and small perturbations snap to identity.
    CHECK(quatAngle(xrNearestAxisAligned(Quat::identity()), Quat::identity()) <
          1e-6);
    const Quat wobble = Quat::fromAxisAngle(Vec3(0, 1, 0), 0.1) *
                        Quat::fromAxisAngle(Vec3(1, 0, 0), -0.08);
    CHECK(quatAngle(xrNearestAxisAligned(wobble), Quat::identity()) < 1e-6);

    // A near-90-degree yaw snaps to the exact 90-degree yaw.
    const Quat yawish = Quat::fromAxisAngle(Vec3(0, 1, 0), PI / 2 - 0.12);
    const Quat yaw90 = Quat::fromAxisAngle(Vec3(0, 1, 0), PI / 2);
    CHECK(quatAngle(xrNearestAxisAligned(yawish), yaw90) < 1e-6);

    // Any input yields a PROPER rotation (det +1): check via the basis of a
    // messy composite.
    const Quat messy = (Quat::fromAxisAngle(Vec3(1, 2, 3), 0.9) *
                        Quat::fromAxisAngle(Vec3(0, 1, 0), 2.2)).normalized();
    const Quat snapped = xrNearestAxisAligned(messy);
    const Mat4 m = snapped.toMat4();
    const Vec3 c0(m.m[0][0], m.m[1][0], m.m[2][0]);
    const Vec3 c1(m.m[0][1], m.m[1][1], m.m[2][1]);
    const Vec3 c2(m.m[0][2], m.m[1][2], m.m[2][2]);
    CHECK(std::fabs(dot(cross(c0, c1), c2) - 1.0) < 1e-9);
}

TEST_CASE(grasp_align_snaps_a_gentle_setdown) {
    // A Jenga block held nearly straight, hovering just above the support:
    // the assist proposes flush, axis-aligned placement.
    const Vec3 he(0.075, 0.015, 0.025);
    const Quat held = Quat::fromAxisAngle(Vec3(0, 1, 0), 0.08);
    const Vec3 pos(0.30, 1.032, 0.20);  // ~2mm above flush (topY + he.y)
    const XrAlignedPose a = xrAlignToSupport(
        he, pos, held, 1.015, Vec3(0, 0, 0), Vec3(1, 0, 1));
    CHECK(a.valid);
    CHECK(std::fabs(a.position.y - (1.015 + 0.015)) < 1e-9);
    CHECK(std::fabs(a.position.x - 0.30) < 1e-9);
    CHECK(quatAngle(a.orientation, Quat::identity()) < 1e-6);
}

TEST_CASE(grasp_align_respects_orientation_of_the_snap) {
    // The block held on its SIDE (rolled 90 about Z): flush height must use
    // the extent that now points up (he.x), not he.y.
    const Vec3 he(0.075, 0.015, 0.025);
    const Quat rolled = Quat::fromAxisAngle(Vec3(0, 0, 1), PI / 2 + 0.05);
    const XrAlignedPose a = xrAlignToSupport(
        he, Vec3(0.5, 0.09, 0.5), rolled, 0.0, Vec3(0, 0, 0), Vec3(1, 0, 1));
    CHECK(a.valid);
    CHECK(std::fabs(a.position.y - 0.075) < 1e-9);
}

TEST_CASE(grasp_align_rejects_skew_and_distance) {
    const Vec3 he(0.05, 0.05, 0.05);
    // 25 degrees off any alignment: not what the user meant — no snap.
    const Quat skewed = Quat::fromAxisAngle(Vec3(0, 0, 1), 0.44);
    CHECK(!xrAlignToSupport(he, Vec3(0.5, 0.052, 0.5), skewed, 0.0,
                            Vec3(0, 0, 0), Vec3(1, 0, 1)).valid);
    // Aligned but 8cm above the support: too far to be a set-down.
    CHECK(!xrAlignToSupport(he, Vec3(0.5, 0.13, 0.5), Quat::identity(), 0.0,
                            Vec3(0, 0, 0), Vec3(1, 0, 1)).valid);
    // Aligned, close, but centred far OUTSIDE the support rect: the clamp
    // would have to drag it more than the snap distance — no snap.
    CHECK(!xrAlignToSupport(he, Vec3(1.4, 0.052, 0.5), Quat::identity(), 0.0,
                            Vec3(0, 0, 0), Vec3(1, 0, 1)).valid);
    // Just inside the edge: clamped so the footprint stays on the support.
    const XrAlignedPose edge = xrAlignToSupport(
        he, Vec3(0.98, 0.052, 0.5), Quat::identity(), 0.0, Vec3(0, 0, 0),
        Vec3(1, 0, 1));
    CHECK(edge.valid);
    CHECK(std::fabs(edge.position.x - 0.95) < 1e-9);
}
