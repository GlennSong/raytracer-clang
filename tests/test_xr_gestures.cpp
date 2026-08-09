#include "test_framework.h"

#include "../src/engine/xr/xr_gestures.h"

#include <cmath>

using namespace engine;

// Hand-gesture recognition (engine/xr/xr_gestures.h), driven with synthetic
// joint streams. Real hand tracking jitters at the millimetre level and drops
// out when hands occlude or leave view; these tests encode how the trackers
// must absorb that — hysteresis against chatter, freezes against dropouts —
// and that the throw estimators recover the velocities a hand actually had.

namespace {

// A hand with every joint tracked and placed at the wrist, except the joints
// a test positions explicitly.
XrHand handAt(const Vec3& wrist) {
    XrHand hand;
    hand.tracked = true;
    for (int j = 0; j < XR_HAND_JOINT_COUNT; j++) {
        hand.joints[j].tracked = true;
        hand.joints[j].originFromJoint =
            Mat4::translate(wrist.x, wrist.y, wrist.z);
    }
    return hand;
}

void placeJoint(XrHand& hand, XrHandJointId joint, const Vec3& p) {
    hand.joints[joint].originFromJoint = Mat4::translate(p.x, p.y, p.z);
}

// Thumb and index tips `gap` metres apart, centred at `centre`.
XrHand pinchHand(const Vec3& centre, Real gap) {
    XrHand hand = handAt(centre);
    placeJoint(hand, XR_JOINT_THUMB_TIP, centre + Vec3(-gap / 2, 0, 0));
    placeJoint(hand, XR_JOINT_INDEX_TIP, centre + Vec3(gap / 2, 0, 0));
    return hand;
}

bool nearVec(const Vec3& a, const Vec3& b, Real tol) {
    return (a - b).length() <= tol;
}

}  // namespace

TEST_CASE(pinch_needs_confirm_frames_before_grabbing) {
    XrPinchTracker tracker;   // defaults: close 2.5cm, open 4cm, confirm 2
    const Vec3 c(0, 1.2, -0.3);

    tracker.feed(pinchHand(c, 0.10));
    CHECK(!tracker.pinched());

    // One closed frame is not a grab (a single jittered sample must not be).
    tracker.feed(pinchHand(c, 0.01));
    CHECK(!tracker.pinched());
    // The second confirms.
    tracker.feed(pinchHand(c, 0.01));
    CHECK(tracker.pinched());
    CHECK(tracker.began());
    // Edge lasts exactly one frame.
    tracker.feed(pinchHand(c, 0.01));
    CHECK(tracker.pinched());
    CHECK(!tracker.began());
}

TEST_CASE(pinch_hysteresis_does_not_chatter_between_thresholds) {
    XrPinchTracker tracker;
    const Vec3 c(0, 1.2, -0.3);
    tracker.feed(pinchHand(c, 0.01));
    tracker.feed(pinchHand(c, 0.01));
    CHECK(tracker.pinched());

    // Jitter in the dead band (between close 2.5cm and open 4cm): held.
    for (int i = 0; i < 20; i++) {
        tracker.feed(pinchHand(c, 0.028 + 0.008 * ((i % 3) - 1)));
        CHECK(tracker.pinched());
        CHECK(!tracker.released());
    }

    // A real open releases (after the confirm frames).
    tracker.feed(pinchHand(c, 0.06));
    tracker.feed(pinchHand(c, 0.06));
    CHECK(!tracker.pinched());
    CHECK(tracker.released());
}

TEST_CASE(pinch_single_frame_spike_neither_grabs_nor_releases) {
    XrPinchTracker tracker;
    const Vec3 c(0, 1.2, -0.3);

    // Open hand with one closed spike: no grab.
    tracker.feed(pinchHand(c, 0.10));
    tracker.feed(pinchHand(c, 0.01));
    tracker.feed(pinchHand(c, 0.10));
    tracker.feed(pinchHand(c, 0.10));
    CHECK(!tracker.pinched());

    // Pinched hand with one open spike: no release.
    tracker.feed(pinchHand(c, 0.01));
    tracker.feed(pinchHand(c, 0.01));
    CHECK(tracker.pinched());
    tracker.feed(pinchHand(c, 0.06));
    tracker.feed(pinchHand(c, 0.01));
    tracker.feed(pinchHand(c, 0.01));
    CHECK(tracker.pinched());
    CHECK(!tracker.released());
}

TEST_CASE(pinch_tracking_loss_freezes_instead_of_releasing) {
    XrPinchTracker tracker;
    const Vec3 c(0, 1.2, -0.3);
    tracker.feed(pinchHand(c, 0.01));
    tracker.feed(pinchHand(c, 0.01));
    CHECK(tracker.pinched());
    const Vec3 heldPoint = tracker.pinchPoint();

    // The hand leaves the camera's view mid-carry. The object must NOT drop:
    // state frozen, no release edge, last pinch point held.
    XrHand lost;
    lost.tracked = false;
    for (int i = 0; i < 30; i++) {
        tracker.feed(lost);
        CHECK(tracker.pinched());
        CHECK(!tracker.released());
        CHECK(!tracker.tracking());
    }
    CHECK(nearVec(tracker.pinchPoint(), heldPoint, 1e-9));

    // Tracking returns with the hand still closed: still carried, no edges.
    tracker.feed(pinchHand(c, 0.01));
    CHECK(tracker.pinched());
    CHECK(!tracker.began());
}

TEST_CASE(palm_pose_frames_are_mirrored_between_hands) {
    // Both hands flat, palm up: wrist at the origin-ish, index/little
    // knuckles forward, the LEFT hand's little finger on the left.
    XrHand right = handAt(Vec3(0.2, 1.0, -0.3));
    placeJoint(right, XR_JOINT_WRIST, Vec3(0.2, 1.0, -0.3));
    placeJoint(right, XR_JOINT_INDEX_KNUCKLE, Vec3(0.17, 1.0, -0.38));
    placeJoint(right, XR_JOINT_LITTLE_KNUCKLE, Vec3(0.25, 1.0, -0.38));

    XrHand left = handAt(Vec3(-0.2, 1.0, -0.3));
    placeJoint(left, XR_JOINT_WRIST, Vec3(-0.2, 1.0, -0.3));
    placeJoint(left, XR_JOINT_INDEX_KNUCKLE, Vec3(-0.17, 1.0, -0.38));
    placeJoint(left, XR_JOINT_LITTLE_KNUCKLE, Vec3(-0.25, 1.0, -0.38));

    const XrPalmPose rp = xrPalmPose(right, /*leftHand=*/false);
    const XrPalmPose lp = xrPalmPose(left, /*leftHand=*/true);
    CHECK(rp.valid);
    CHECK(lp.valid);
    // Both palms face up (the mirrored winding is exactly what the leftHand
    // flag corrects), fingers point forward (-Z-ish) on both.
    CHECK(rp.normal.y > 0.9);
    CHECK(lp.normal.y > 0.9);
    CHECK(rp.fingers.z < -0.8);
    CHECK(lp.fingers.z < -0.8);
    CHECK(xrPalmUp(rp));
    CHECK(xrPalmUp(lp));
}

TEST_CASE(palm_down_is_not_palm_up) {
    // Right hand palm DOWN: swap index/little so the winding flips.
    XrHand hand = handAt(Vec3(0.2, 1.0, -0.3));
    placeJoint(hand, XR_JOINT_WRIST, Vec3(0.2, 1.0, -0.3));
    placeJoint(hand, XR_JOINT_INDEX_KNUCKLE, Vec3(0.25, 1.0, -0.38));
    placeJoint(hand, XR_JOINT_LITTLE_KNUCKLE, Vec3(0.17, 1.0, -0.38));
    const XrPalmPose p = xrPalmPose(hand, /*leftHand=*/false);
    CHECK(p.valid);
    CHECK(p.normal.y < -0.9);
    CHECK(!xrPalmUp(p));
}

TEST_CASE(palm_pose_invalid_on_untracked_joints) {
    XrHand hand = handAt(Vec3(0, 1, 0));
    hand.joints[XR_JOINT_LITTLE_KNUCKLE].tracked = false;
    CHECK(!xrPalmPose(hand, false).valid);
    XrHand lost;
    lost.tracked = false;
    CHECK(!xrPalmPose(lost, true).valid);
}

TEST_CASE(throw_linear_velocity_recovered_from_noisy_track) {
    // A hand sweeping at (1.5, 2.0, -0.5) m/s, sampled at 90 Hz with ±3mm
    // deterministic "jitter". The least-squares fit has to see through it.
    XrPoseHistory history;
    const Vec3 v(1.5, 2.0, -0.5);
    for (int i = 0; i < 18; i++) {
        const Real t = i / 90.0;
        const Real n = 0.003 * std::sin(i * 2.399);   // bounded pseudo-noise
        history.push(t, v * t + Vec3(n, -n, n), Quat::identity());
    }
    const Vec3 est = history.linearVelocity(0.15);
    CHECK(nearVec(est, v, 0.25));   // within 0.25 m/s on a 2.6 m/s throw
}

TEST_CASE(throw_two_sample_diff_would_fail_where_the_fit_succeeds) {
    // Same track, but the LAST sample carries a 1cm spike — the classic
    // release-frame glitch. A finite difference over the last two samples
    // (1cm over 11ms ≈ 0.9 m/s of phantom velocity per axis) is far off;
    // the windowed fit stays close. This is WHY it is a fit.
    XrPoseHistory history;
    const Vec3 v(1.0, 0, 0);
    int samples = 14;
    for (int i = 0; i < samples; i++) {
        const Real t = i / 90.0;
        Vec3 p = v * t;
        if (i == samples - 1) p += Vec3(0.01, 0.01, 0.01);
        history.push(t, p, Quat::identity());
    }
    const Vec3 est = history.linearVelocity(0.15);
    CHECK(nearVec(est, v, 0.35));

    const Real lastDt = 1.0 / 90.0;
    const Real spikeVel = 0.01 / lastDt;   // what naive diff adds per axis
    CHECK(spikeVel > 0.8);                  // the failure the fit avoids
}

TEST_CASE(throw_velocity_empty_and_single_sample_are_zero) {
    XrPoseHistory history;
    CHECK(nearVec(history.linearVelocity(), Vec3(0, 0, 0), 1e-12));
    history.push(0.0, Vec3(1, 2, 3), Quat::identity());
    CHECK(nearVec(history.linearVelocity(), Vec3(0, 0, 0), 1e-12));
    CHECK(nearVec(history.angularVelocity(), Vec3(0, 0, 0), 1e-12));
}

TEST_CASE(throw_angular_velocity_recovers_wrist_spin) {
    // Wrist flicking around +Y at 6 rad/s, sampled at 90 Hz.
    XrPoseHistory history;
    const Vec3 axis(0, 1, 0);
    const Real rate = 6.0;
    for (int i = 0; i < 14; i++) {
        const Real t = i / 90.0;
        history.push(t, Vec3(0, 0, 0), Quat::fromAxisAngle(axis, rate * t));
    }
    const Vec3 w = history.angularVelocity(0.15);
    CHECK(std::abs(w.y - rate) < 0.3);
    CHECK(std::abs(w.x) < 0.1);
    CHECK(std::abs(w.z) < 0.1);
}

TEST_CASE(throw_history_ring_survives_wraparound) {
    // Push far past capacity; the trailing window still fits recent motion.
    XrPoseHistory history;
    const Vec3 v(0, 0, -2.0);
    for (int i = 0; i < 200; i++) {
        const Real t = i / 90.0;
        history.push(t, v * t, Quat::identity());
    }
    CHECK(nearVec(history.linearVelocity(0.15), v, 0.05));
    history.clear();
    CHECK(history.size() == 0);
    CHECK(nearVec(history.linearVelocity(), Vec3(0, 0, 0), 1e-12));
}
