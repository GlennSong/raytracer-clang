// Hand-gesture recognition over the engine's neutral hand skeletons — the
// layer that turns XrState::hands (27 joints, filled by whichever headset
// backend is running) into the events interaction systems consume: pinch
// grab/release edges, palm poses for palm-anchored UI, and release-time
// linear + angular velocity so a thrown object flies like the hand threw it.
//
// Everything here is pure math over xr_state.h types — no platform, renderer
// or physics includes — so the thresholds, hysteresis and estimators are
// host-tested with synthetic joint streams (tests/test_xr_gestures.cpp).
// Real hand tracking jitters and drops out; the tests encode exactly how
// those rough edges must be absorbed.

#ifndef ENGINE_XR_GESTURES_H
#define ENGINE_XR_GESTURES_H

#include <cstddef>

#include "../../rt_math.h"
#include "xr_state.h"

namespace engine {

// Translation column of a joint's pose — where the joint is, ORIGIN space.
inline Vec3 xrJointPosition(const XrHand& hand, XrHandJointId joint) {
    const Mat4& m = hand.joints[joint].originFromJoint;
    return Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
}

// --- Pinch tracking -------------------------------------------------------
//
// Thumb-tip / index-tip distance with HYSTERESIS: close under closeDistance
// to pinch, open past openDistance to release, and each transition needs
// confirmFrames consecutive frames on the far side — a single jittered
// sample must never grab or drop an object. Tracking loss FREEZES the
// tracker (state held, no edges): a hand leaving the camera's view while
// carrying something must not read as "released", or the object falls the
// moment the user glances away.
struct XrPinchConfig {
    Real closeDistance = 0.025;   // metres, enter Pinched below this
    Real openDistance = 0.040;    // metres, leave Pinched above this
    int confirmFrames = 2;        // consecutive frames to confirm an edge
};

class XrPinchTracker {
public:
    explicit XrPinchTracker(XrPinchConfig config = {}) : config_(config) {}

    // Feed this frame's skeleton (the hand this tracker owns). Edges are
    // valid until the next feed.
    void feed(const XrHand& hand);

    bool pinched() const { return pinched_; }
    bool began() const { return began_; }      // edge: closed this frame
    bool released() const { return released_; } // edge: opened this frame
    bool tracking() const { return tracking_; } // fed a tracked hand last frame
    // Midpoint of thumb and index tips — where a grab happens. Last-tracked
    // value is held through dropouts, matching the frozen state.
    Vec3 pinchPoint() const { return pinchPoint_; }
    Real pinchDistance() const { return distance_; }

private:
    XrPinchConfig config_;
    bool pinched_ = false;
    bool began_ = false;
    bool released_ = false;
    bool tracking_ = false;
    int framesPast_ = 0;   // consecutive frames past the pending threshold
    Vec3 pinchPoint_{0, 0, 0};
    Real distance_ = 1.0;
};

// --- Palm pose ------------------------------------------------------------
//
// A stable hand frame for palm-anchored UI, derived from the wrist and the
// index/little knuckles (the most reliably tracked joints): position is
// their centroid, fingers points wrist->knuckles, normal points OUT of the
// palm — which is the opposite cross-product winding per hand, hence the
// explicit leftHand flag.
struct XrPalmPose {
    bool valid = false;
    Vec3 position;   // palm centre, ORIGIN space
    Vec3 normal;     // unit, out of the palm
    Vec3 fingers;    // unit, wrist toward the knuckles
    Vec3 across;     // unit, completes the frame (normal × fingers)
};

XrPalmPose xrPalmPose(const XrHand& hand, bool leftHand);

// Palm facing up (within maxAngle of +Y) — the palette summon gesture.
bool xrPalmUp(const XrPalmPose& palm, Real maxAngleRadians = 0.7);

// --- Throw estimation -----------------------------------------------------
//
// Ring of timestamped grip poses while an object is carried. On release, the
// trailing window yields the velocities the object leaves with: linear from
// a least-squares line fit (robust to per-frame jitter in a way last-two-
// samples never is), angular from averaged quaternion deltas — so a wrist
// flick spins the object and an arc tosses it, i.e. hands work like hands.
class XrPoseHistory {
public:
    void push(Real timeSeconds, const Vec3& position, const Quat& orientation);
    void clear();
    size_t size() const { return count_; }

    // Least-squares linear velocity over the trailing `window` seconds.
    // Zero until two usable samples exist.
    Vec3 linearVelocity(Real window = 0.15) const;

    // Average angular velocity (axis * rad/s) over the trailing window.
    Vec3 angularVelocity(Real window = 0.15) const;

private:
    static constexpr size_t kCapacity = 32;
    struct Sample {
        Real t;
        Vec3 p;
        Quat q;
    };
    Sample samples_[kCapacity];
    size_t head_ = 0;    // next write slot
    size_t count_ = 0;

    // Oldest-to-newest visit of the samples inside the window.
    template <typename Fn>
    void eachInWindow(Real window, Fn&& fn) const {
        if (count_ == 0) return;
        const Real newest = samples_[(head_ + kCapacity - 1) % kCapacity].t;
        for (size_t i = 0; i < count_; i++) {
            const Sample& s =
                samples_[(head_ + kCapacity - count_ + i) % kCapacity];
            if (newest - s.t <= window) fn(s);
        }
    }
};

}  // namespace engine

#endif  // ENGINE_XR_GESTURES_H
