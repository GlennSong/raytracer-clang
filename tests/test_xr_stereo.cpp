#include "test_framework.h"

#include "../src/engine/xr/xr_view_math.h"

using namespace engine;

// Stereo, proven by arithmetic instead of by looking.
//
// The two-view render path only ever executes on a real headset: the visionOS
// simulator reports a single view, so "run it and see" is not available off
// device, and on device the classic failures — eyes swapped, an eye offset
// dropped, separation not tracking world scale — are either invisible or read
// as vague discomfort. Worse, they are invisible BY CONSTRUCTION to anyone who
// does not fuse stereo. So the invariants are pinned here as numbers.
//
// Scope: this covers the eye POSE composition (engine/xr/xr_view_math.h), which
// is ours. The per-eye PROJECTIONS are the compositor's — queried from
// `cp_drawable_compute_projection` and passed through untouched — so the
// projection used below is a plain symmetric perspective standing in for one,
// chosen only to turn eye poses into screen-space disparity we can assert on.

namespace {

// A representative adult interpupillary distance, in metres. Nothing depends
// on the exact value — every assertion below is relative to it.
constexpr Real IPD = 0.063;

// The runtime's head->eye offsets: view 0 left, view 1 right, half an IPD to
// either side along the head's own X axis.
Mat4 deviceFromEye(int view) {
    const Real sign = (view == 0) ? -1.0 : 1.0;
    return Mat4::translate(sign * IPD * 0.5, 0, 0);
}

// A 90-degree-vertical-FOV square projection: yScale and xScale both land on
// exactly 1, so a point at distance d projects to ndc.x = x_eye / d and the
// disparity assertions below read as plain ratios rather than magic numbers.
Mat4 unitProjection() {
    return Mat4::perspective(M_PI * 0.5, 1.0, 0.01, 1000.0);
}

// Where a world point lands in this eye's normalized device coordinates.
Vec3 projectIntoEye(const Mat4& worldFromEye, const Vec3& worldPoint) {
    return (unitProjection() * worldFromEye.inverse()).transformPoint(worldPoint);
}

}  // namespace

// --- Eye separation ------------------------------------------------------

TEST_CASE(eye_separation_is_the_ipd_at_unit_world_scale) {
    const Mat4 head = Mat4::identity();
    const Mat4 left  = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(0), 1.0);
    const Mat4 right = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(1), 1.0);

    CHECK_APPROX(xrEyeSeparation(left, right), IPD, 1e-9);
    // ...and along X, with the right eye on the +X side. A composition that
    // dropped one offset would still separate the eyes by SOMETHING; the axis
    // and the ordering are what say it did so correctly.
    CHECK_APPROX(xrTranslationOf(right).x - xrTranslationOf(left).x, IPD, 1e-9);
    CHECK(xrTranslationOf(right).x > xrTranslationOf(left).x);
}

TEST_CASE(eye_separation_scales_with_world_scale) {
    const Mat4 head = Mat4::identity();
    // Giant mode: at 25 world units per metre the eyes sit 25 IPDs apart, and
    // that hyper-stereo is the whole reason a life-size world reads as a model
    // on a table. If separation ever stops tracking scale, the tabletop effect
    // silently degrades to "a small world seen from far away".
    for (Real scale : {1.0, 2.0, 10.0, 25.0}) {
        const Mat4 left  = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(0), scale);
        const Mat4 right = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(1), scale);
        CHECK_APPROX(xrEyeSeparation(left, right), IPD * scale, 1e-9);
    }
}

TEST_CASE(head_yaw_carries_the_eye_separation_axis) {
    // Yaw 90 degrees: the head's right axis now points down world -Z, so the
    // eyes must separate along Z and no longer along X. This is the assertion
    // that catches an offset applied in the wrong space — composing
    // deviceFromEye on the outside instead of the inside leaves the eyes
    // stubbornly separated along world X no matter where the user looks.
    const Mat4 head = Mat4::rotateY(M_PI * 0.5);
    const Mat4 left  = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(0), 1.0);
    const Mat4 right = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(1), 1.0);

    const Vec3 separation = xrTranslationOf(right) - xrTranslationOf(left);
    CHECK_APPROX(separation.x, 0.0, 1e-9);
    CHECK_APPROX(separation.y, 0.0, 1e-9);
    CHECK_APPROX(separation.z, -IPD, 1e-9);
    CHECK_APPROX(xrEyeSeparation(left, right), IPD, 1e-9);
}

TEST_CASE(head_translation_moves_both_eyes_together) {
    const Mat4 head = Mat4::translate(3, 1.7, -4);
    const Mat4 left  = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(0), 1.0);
    const Mat4 right = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(1), 1.0);

    CHECK_APPROX(xrEyeSeparation(left, right), IPD, 1e-9);
    // Midpoint of the eyes is the head.
    const Vec3 mid = (xrTranslationOf(left) + xrTranslationOf(right)) * 0.5;
    CHECK_APPROX(mid.x, 3.0, 1e-9);
    CHECK_APPROX(mid.y, 1.7, 1e-9);
    CHECK_APPROX(mid.z, -4.0, 1e-9);
}

TEST_CASE(world_scale_moves_the_head_as_far_as_it_spreads_the_eyes) {
    // Both distances scale together, or the user's proportions come apart:
    // eyes 25 IPDs wide on a head that only travelled one metre would put the
    // world at a scale the body contradicts.
    const Mat4 head = Mat4::translate(0, 1.6, 0);
    const Mat4 left  = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(0), 25.0);
    const Mat4 right = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(1), 25.0);

    const Vec3 mid = (xrTranslationOf(left) + xrTranslationOf(right)) * 0.5;
    CHECK_APPROX(mid.y, 1.6 * 25.0, 1e-9);
    CHECK_APPROX(xrEyeSeparation(left, right), IPD * 25.0, 1e-9);
}

TEST_CASE(the_locomotion_base_offsets_both_eyes_into_the_world) {
    const Mat4 head = Mat4::identity();
    const Vec3 base(100, 5, -20);
    const Mat4 left  = xrWorldFromEye(base, head, deviceFromEye(0), 1.0);
    const Mat4 right = xrWorldFromEye(base, head, deviceFromEye(1), 1.0);

    CHECK_APPROX(xrTranslationOf(left).x,  100.0 - IPD * 0.5, 1e-9);
    CHECK_APPROX(xrTranslationOf(right).x, 100.0 + IPD * 0.5, 1e-9);
    CHECK_APPROX(xrTranslationOf(left).y, 5.0, 1e-9);
    CHECK_APPROX(xrTranslationOf(left).z, -20.0, 1e-9);
    // The base is a world offset, NOT a real-metres one: it must not be
    // multiplied by world scale a second time.
    const Mat4 scaled = xrWorldFromEye(base, head, deviceFromEye(0), 25.0);
    CHECK_APPROX(xrTranslationOf(scaled).y, 5.0, 1e-9);
}

// --- Screen-space disparity ----------------------------------------------

TEST_CASE(a_point_ahead_projects_right_of_center_in_the_left_eye) {
    // THE swapped-eye test. A point straight ahead sits to the RIGHT of the
    // left eye's centre and to the LEFT of the right eye's, because each eye
    // is displaced towards the other side. Swap the two views and every sign
    // here inverts, which on a headset produces depth that reads inside out —
    // uncomfortable, hard to name, and impossible to spot without fusing.
    const Mat4 head = Mat4::identity();
    const Mat4 left  = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(0), 1.0);
    const Mat4 right = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(1), 1.0);

    const Real distance = 2.0;
    const Vec3 ahead(0, 0, -distance);
    const Real leftX  = projectIntoEye(left, ahead).x;
    const Real rightX = projectIntoEye(right, ahead).x;

    CHECK(leftX > 0.0);
    CHECK(rightX < 0.0);
    CHECK(leftX > rightX);
    // With the unit projection the disparity is exactly IPD / distance.
    CHECK_APPROX(leftX - rightX, IPD / distance, 1e-9);
}

TEST_CASE(disparity_falls_off_with_distance) {
    const Mat4 head = Mat4::identity();
    const Mat4 left  = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(0), 1.0);
    const Mat4 right = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(1), 1.0);

    auto disparityAt = [&](Real distance) {
        const Vec3 p(0, 0, -distance);
        return projectIntoEye(left, p).x - projectIntoEye(right, p).x;
    };

    const Real near = disparityAt(2.0);
    const Real far = disparityAt(20.0);
    CHECK(near > far);
    CHECK_APPROX(near / far, 10.0, 1e-6);      // strictly inverse in distance
    // Effectively at infinity the eyes agree: no disparity left to fuse.
    CHECK(disparityAt(10000.0) < 1e-4);
}

TEST_CASE(world_scale_deepens_disparity) {
    // Same physical room, scale 25: the widened eyes see far more disparity,
    // which is the measurable form of "the world looks like a model".
    const Mat4 head = Mat4::identity();
    auto disparityAtScale = [&](Real scale) {
        const Mat4 left  = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(0), scale);
        const Mat4 right = xrWorldFromEye(Vec3(0, 0, 0), head, deviceFromEye(1), scale);
        const Vec3 p(0, 0, -50.0);
        return projectIntoEye(left, p).x - projectIntoEye(right, p).x;
    };
    CHECK_APPROX(disparityAtScale(25.0) / disparityAtScale(1.0), 25.0, 1e-6);
}

// --- Locomotion base -----------------------------------------------------

TEST_CASE(locomotion_base_seeds_a_floor_below_the_gameplay_camera) {
    XrLocomotionBase locomotion;
    CHECK(!locomotion.valid);

    const Vec3 seeded = locomotion.update(Vec3(10, 12.6, -3));
    CHECK(locomotion.valid);
    // The game camera is the player's eye; the tracking origin is the floor
    // under it. Seeding at the camera itself would put a standing user's head
    // an extra 1.6 m up — on an elevated city deck, well inside the ceiling.
    CHECK_APPROX(seeded.x, 10.0, 1e-9);
    CHECK_APPROX(seeded.y, 12.6 - XR_SEED_EYE_HEIGHT, 1e-9);
    CHECK_APPROX(seeded.z, -3.0, 1e-9);
}

TEST_CASE(locomotion_base_follows_gameplay_by_deltas) {
    XrLocomotionBase locomotion;
    const Vec3 seeded = locomotion.update(Vec3(0, 1.6, 0));
    CHECK_APPROX(seeded.y, 0.0, 1e-9);

    // A teleport moves the gameplay camera 50 m; the user must go with it.
    const Vec3 after = locomotion.update(Vec3(50, 1.6, 25));
    CHECK_APPROX(after.x, 50.0, 1e-9);
    CHECK_APPROX(after.z, 25.0, 1e-9);
    CHECK_APPROX(after.y, 0.0, 1e-9);
}

TEST_CASE(an_unchanged_hint_never_walks_the_base) {
    // The feedback guard. XrCameraSystem overwrites the gameplay camera from
    // the head pose every frame, so on a frame where no gameplay system writes
    // the camera, the hint that comes back is head-derived. Following it by
    // ABSOLUTE position would add the user's own head motion to the base and
    // the world would slide away as they leaned. Following by delta makes a
    // repeated hint a no-op, which is what keeps that impossible.
    XrLocomotionBase locomotion;
    const Vec3 seeded = locomotion.update(Vec3(4, 1.6, 4));
    for (int frame = 0; frame < 120; ++frame) {
        const Vec3 held = locomotion.update(Vec3(4, 1.6, 4));
        CHECK_APPROX(held.x, seeded.x, 1e-12);
        CHECK_APPROX(held.y, seeded.y, 1e-12);
        CHECK_APPROX(held.z, seeded.z, 1e-12);
    }
}

TEST_CASE(a_second_writer_of_the_base_survives_an_unchanged_hint) {
    // This is the ONLY behaviour that distinguishes following-by-delta from
    // just tracking the latest hint: with a single writer the two are
    // algebraically identical (the deltas telescope). They diverge the moment
    // anything else moves the base — a recentre, a direct teleport of the
    // tracking origin — because absolute following would silently snap it back
    // on the very next frame. Worth pinning now: the equivalence makes it easy
    // to "simplify" this into the absolute form without noticing what breaks.
    XrLocomotionBase locomotion;
    locomotion.update(Vec3(0, 1.6, 0));
    locomotion.base = Vec3(0, 0, -40);

    const Vec3 after = locomotion.update(Vec3(0, 1.6, 0));   // hint unchanged
    CHECK_APPROX(after.z, -40.0, 1e-9);
}

TEST_CASE(delta_following_does_not_accumulate_drift) {
    // Accumulation is the standing cost of the delta form: a base built by
    // summing thousands of increments can wander where a recomputed one
    // cannot. Walk a long jittery path back to where it started and require
    // the base to come back with it.
    XrLocomotionBase locomotion;
    const Vec3 start(0, 1.6, 0);
    const Vec3 seeded = locomotion.update(start);

    Vec3 hint = start;
    for (int frame = 0; frame < 10000; ++frame) {
        const Real step = ((frame % 7) - 3) * 0.013;   // sums to a wander, not a walk
        hint += Vec3(step, 0, -step);
        locomotion.update(hint);
    }
    const Vec3 returned = locomotion.update(start);
    CHECK_APPROX(returned.x, seeded.x, 1e-9);
    CHECK_APPROX(returned.y, seeded.y, 1e-9);
    CHECK_APPROX(returned.z, seeded.z, 1e-9);
}

TEST_CASE(locomotion_base_reset_reseeds_from_the_next_hint) {
    XrLocomotionBase locomotion;
    locomotion.update(Vec3(0, 1.6, 0));
    locomotion.update(Vec3(30, 1.6, 0));
    locomotion.reset();

    const Vec3 reseeded = locomotion.update(Vec3(-7, 10.6, 2));
    CHECK_APPROX(reseeded.x, -7.0, 1e-9);
    CHECK_APPROX(reseeded.y, 10.6 - XR_SEED_EYE_HEIGHT, 1e-9);
    CHECK_APPROX(reseeded.z, 2.0, 1e-9);
}

TEST_CASE(gameplay_motion_and_head_motion_compose_without_double_counting) {
    // The end-to-end shape of the bug this design exists to prevent: the user
    // leans forward half a metre (head pose changes, hint does not) and is
    // then teleported 10 m. The eye must end up at teleport + lean, once each.
    XrLocomotionBase locomotion;
    const Vec3 base0 = locomotion.update(Vec3(0, 1.6, 0));

    const Mat4 leaned = Mat4::translate(0, 1.6, -0.5);
    const Mat4 eye0 = xrWorldFromEye(base0, leaned, Mat4::identity(), 1.0);
    CHECK_APPROX(xrTranslationOf(eye0).z, -0.5, 1e-9);

    const Vec3 base1 = locomotion.update(Vec3(0, 1.6, -10));
    const Mat4 eye1 = xrWorldFromEye(base1, leaned, Mat4::identity(), 1.0);
    CHECK_APPROX(xrTranslationOf(eye1).z, -10.5, 1e-9);
    CHECK_APPROX(xrTranslationOf(eye1).y, 1.6, 1e-9);
}
