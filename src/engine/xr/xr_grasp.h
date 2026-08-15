// The grasp stack's pure brain: grip memory (where the fingers took hold,
// in the object's own space), hand memory (coasting through tracking
// dropouts), and the placement-assist alignment solver. No physics or
// platform includes — everything here is host-tested with synthetic data
// (tests/test_xr_grasp.cpp), because these thresholds define how grabbing
// FEELS and must not need a headset per tuning iteration.

#ifndef ENGINE_XR_GRASP_H
#define ENGINE_XR_GRASP_H

#include <cstddef>
#include <utility>
#include <vector>

#include "../../rt_math.h"

namespace engine {

// --- Grip memory ----------------------------------------------------------
//
// "Remember where the finger positions are": at hook time the touching
// fingertips' points and press directions are captured in OBJECT-LOCAL
// space, so the memory rides the object however it moves in the hand.
// It answers two questions: was this a legitimate grab (opposed() — the
// "held on each side" condition), and has the hand let go (opening() —
// live fingertips drifting off their remembered anchors).
struct XrGripPoint {
    int id = 0;        // caller's fingertip identifier (joint id)
    Vec3 point;        // world space at capture/query time
    Vec3 normal;       // unit, pressing INTO the object (away from finger)
};

class XrGripMemory {
public:
    void capture(const std::vector<XrGripPoint>& worldContacts,
                 const Vec3& objectPosition, const Quat& objectOrientation);
    void clear();

    size_t contactCount() const { return anchors_.size(); }

    // The "each side" test: at least two contacts whose press directions
    // oppose (min pairwise normal dot below the threshold). A palm slap or
    // two same-side touches never satisfy it.
    bool opposed(Real maxPairDot = -0.3) const;

    // Mean OUTWARD drift of live fingertips off their remembered anchors,
    // given the object's CURRENT pose: only motion along the remembered
    // press direction (away from the object) counts, clamped at zero per
    // tip. Direction matters — while held, hand-object collision is off,
    // so fingers SINK INTO the shape; raw anchor distance read that as
    // "opened" and the hold oscillated (device log: unhook v=0.02 / hook
    // closure, once a second). Sinking and surface sliding are not letting
    // go. Matched by id; unmatched (untracked) tips are skipped. No
    // matches -> 0 (a dropout mid-hold must not drop the object).
    Real opening(const std::vector<std::pair<int, Vec3>>& liveWorldTips,
                 const Vec3& objectPosition,
                 const Quat& objectOrientation) const;

    // Remembered anchor i in world space at the object's current pose (the
    // contact-dot draw).
    Vec3 worldAnchor(size_t i, const Vec3& objectPosition,
                     const Quat& objectOrientation) const;

    // Release when opening() exceeds this (with the system's own dwell).
    static constexpr Real kReleaseOpening = 0.035;

private:
    struct Anchor {
        int id;
        Vec3 localPoint;
        Vec3 localNormal;
    };
    std::vector<Anchor> anchors_;
};

// --- Hand memory ----------------------------------------------------------
//
// Per-frame snapshots of a hand pose so brief tracking dropouts (mid-throw,
// hand behind hand) COAST instead of freezing or snapping: extrapolate the
// last fitted velocity with decay for up to coastBudget, then hold; on
// reacquire, blend back over blendTime instead of teleporting.
struct XrHandMemoryConfig {
    Real coastBudget = 0.15;   // seconds of extrapolation on dropout
    Real coastDecay = 0.10;    // velocity halves roughly every ~70ms
    Real blendTime = 0.08;     // reacquire blend
    Real fitWindow = 0.12;     // velocity fit window over the snapshot ring
};

class XrHandMemory {
public:
    explicit XrHandMemory(XrHandMemoryConfig config = {}) : config_(config) {}

    // Feed once per frame. tracked=false advances the coast.
    void feed(bool tracked, const Vec3& position, const Quat& orientation,
              Real now);

    bool available() const { return primed_; }
    bool coasting() const { return coasting_; }
    const Vec3& position() const { return position_; }
    const Quat& orientation() const { return orientation_; }

    // The fastest short-span velocity in the snapshot ring within `window`
    // of the newest TRACKED sample (two-step spans, so one noisy frame
    // can't fake a spike). This is the throw-restore signal: hand-tracked
    // releases always fire LATE — finger extension lags real fingers under
    // fast motion, and fast swings drop tracking outright — so by release
    // time the arm has decelerated and the spring-held object with it. The
    // peak is what the throw actually was. Coasting adds no samples, so
    // after a mid-throw dropout this still reports the motion at loss.
    Vec3 peakVelocity(Real window) const;

private:
    XrHandMemoryConfig config_;
    // Snapshot ring for the velocity fit (least-squares, same rationale as
    // XrPoseHistory: one noisy frame must not fling the coast).
    static constexpr size_t kRing = 16;
    struct Snap {
        Real t;
        Vec3 p;
    };
    Snap ring_[kRing];
    size_t head_ = 0, count_ = 0;

    Vec3 position_{0, 0, 0};
    Quat orientation_;
    Vec3 coastVelocity_{0, 0, 0};
    Real lostAt_ = 0, lastNow_ = 0, reacquiredAt_ = -1;
    Vec3 blendFromP_;
    Quat blendFromQ_;
    bool primed_ = false, coasting_ = false;

    Vec3 fitVelocity() const;
};

// --- Placement assist -----------------------------------------------------
//
// The Jenga helper: given a held box's pose and a support's axis-aligned
// top rectangle, the SNAPPED pose — nearest axis-aligned orientation,
// face-flush on the support — or invalid when the hand is too far off for
// the snap to be what the user meant. Applied by the system ONLY at gentle
// release; while held it only draws a ghost, so it can never fight the
// natural grip.
struct XrAlignedPose {
    bool valid = false;
    Vec3 position;
    Quat orientation;
};

XrAlignedPose xrAlignToSupport(const Vec3& halfExtent, const Vec3& position,
                               const Quat& orientation, Real supportTopY,
                               const Vec3& supportMinXZ,
                               const Vec3& supportMaxXZ,
                               Real maxAngleRadians = 0.26,   // ~15 deg
                               Real maxSnapDistance = 0.04);

// The nearest of the 24 axis-aligned orientations to `q` (helper for the
// solver, exposed for tests).
Quat xrNearestAxisAligned(const Quat& q);

// --- Closure probe --------------------------------------------------------
//
// Which fingertips are ON the shape (within `reach` of its oriented
// surface), as grip points whose normals press toward the surface. This is
// the HOOK detector — deliberately predictive, not contact-driven: the
// finger capsules are kinematic (infinitely strong), so by the time two of
// them have PENETRATED far enough to raise opposed contact events, the
// squeeze has already ejected the object ("it shoots away"). Proximity
// fires while the fingers merely surround it. A tip inside the shape counts
// (distance 0) and presses toward the centre.
std::vector<XrGripPoint> xrProbeGrip(
    const std::vector<std::pair<int, Vec3>>& tips, const Vec3& halfExtent,
    bool sphere, const Vec3& position, const Quat& orientation, Real reach);

}  // namespace engine

#endif  // ENGINE_XR_GRASP_H
