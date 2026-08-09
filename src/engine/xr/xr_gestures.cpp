#include "xr_gestures.h"

namespace engine {

void XrPinchTracker::feed(const XrHand& hand) {
    began_ = false;
    released_ = false;

    const XrHandJoint& thumb = hand.joints[XR_JOINT_THUMB_TIP];
    const XrHandJoint& index = hand.joints[XR_JOINT_INDEX_TIP];
    tracking_ = hand.tracked && thumb.tracked && index.tracked;
    if (!tracking_) {
        // Freeze: no edges, no threshold progress. A dropout mid-carry must
        // not release; a dropout mid-open must not grab.
        framesPast_ = 0;
        return;
    }

    const Vec3 a = xrJointPosition(hand, XR_JOINT_THUMB_TIP);
    const Vec3 b = xrJointPosition(hand, XR_JOINT_INDEX_TIP);
    distance_ = (a - b).length();
    pinchPoint_ = (a + b) * 0.5;

    if (!pinched_) {
        if (distance_ < config_.closeDistance) {
            if (++framesPast_ >= config_.confirmFrames) {
                pinched_ = true;
                began_ = true;
                framesPast_ = 0;
            }
        } else {
            framesPast_ = 0;
        }
    } else {
        if (distance_ > config_.openDistance) {
            if (++framesPast_ >= config_.confirmFrames) {
                pinched_ = false;
                released_ = true;
                framesPast_ = 0;
            }
        } else {
            framesPast_ = 0;
        }
    }
}

XrPalmPose xrPalmPose(const XrHand& hand, bool leftHand) {
    XrPalmPose palm;
    const XrHandJoint& wrist = hand.joints[XR_JOINT_WRIST];
    const XrHandJoint& index = hand.joints[XR_JOINT_INDEX_KNUCKLE];
    const XrHandJoint& little = hand.joints[XR_JOINT_LITTLE_KNUCKLE];
    if (!hand.tracked || !wrist.tracked || !index.tracked || !little.tracked)
        return palm;

    const Vec3 w = xrJointPosition(hand, XR_JOINT_WRIST);
    const Vec3 i = xrJointPosition(hand, XR_JOINT_INDEX_KNUCKLE);
    const Vec3 l = xrJointPosition(hand, XR_JOINT_LITTLE_KNUCKLE);

    const Vec3 toIndex = i - w;
    const Vec3 toLittle = l - w;
    Vec3 across = cross(toIndex, toLittle);
    const Real len = across.length();
    if (len < 1e-9) return palm;   // degenerate (joints collinear)

    // cross(wrist->index, wrist->little) points out of the LEFT palm (worked
    // through in test_xr_gestures: palm-up hands, fingers forward); the right
    // hand mirrors, so flip. Device-verified via the palette summon — if
    // palettes appear on the BACK of the hand, this sign is the bug.
    Vec3 normal = across / len;
    if (!leftHand) normal = normal * -1.0;

    palm.valid = true;
    palm.position = (w + i + l) / 3.0;
    palm.normal = normal;
    palm.fingers = normalize((i + l) * 0.5 - w);
    palm.across = normalize(cross(palm.normal, palm.fingers));
    return palm;
}

bool xrPalmUp(const XrPalmPose& palm, Real maxAngleRadians) {
    if (!palm.valid) return false;
    return dot(palm.normal, Vec3(0, 1, 0)) > std::cos(maxAngleRadians);
}

void XrPoseHistory::push(Real timeSeconds, const Vec3& position,
                         const Quat& orientation) {
    samples_[head_] = {timeSeconds, position, orientation};
    head_ = (head_ + 1) % kCapacity;
    if (count_ < kCapacity) count_++;
}

void XrPoseHistory::clear() {
    head_ = 0;
    count_ = 0;
}

Vec3 XrPoseHistory::linearVelocity(Real window) const {
    // Least-squares slope of position over time: v = Σ(dt·dp) / Σ(dt²)
    // around the window means. One noisy endpoint sample barely moves the
    // fit; a two-sample finite difference would inherit it entirely.
    Real sumT = 0;
    Vec3 sumP(0, 0, 0);
    int n = 0;
    eachInWindow(window, [&](const Sample& s) {
        sumT += s.t;
        sumP += s.p;
        n++;
    });
    if (n < 2) return Vec3(0, 0, 0);
    const Real meanT = sumT / n;
    const Vec3 meanP = sumP / static_cast<Real>(n);

    Real denom = 0;
    Vec3 numer(0, 0, 0);
    eachInWindow(window, [&](const Sample& s) {
        const Real dt = s.t - meanT;
        denom += dt * dt;
        numer += (s.p - meanP) * dt;
    });
    if (denom < 1e-12) return Vec3(0, 0, 0);
    return numer / denom;
}

Vec3 XrPoseHistory::angularVelocity(Real window) const {
    // Average the per-step rotation vectors: for consecutive samples,
    // dq = q2 * q1⁻¹ is the rotation across dt; its axis*angle/dt is an
    // angular-velocity sample. Unit quaternions: inverse == conjugate.
    Vec3 sum(0, 0, 0);
    Real totalDt = 0;
    bool havePrev = false;
    Sample prev{};
    eachInWindow(window, [&](const Sample& s) {
        if (havePrev) {
            const Real dt = s.t - prev.t;
            if (dt > 1e-6) {
                Quat dq = s.q * prev.q.conjugate();
                Vec3 axis;
                Real angle = 0;
                dq.toAxisAngle(axis, angle);
                // toAxisAngle returns angle in [0, 2π]; the long way round
                // is the same rotation the short way, negated.
                if (angle > PI) {
                    angle = 2 * PI - angle;
                    axis = axis * -1.0;
                }
                sum += axis * angle;   // weight by dt via the angle itself
                totalDt += dt;
            }
        }
        prev = s;
        havePrev = true;
    });
    if (totalDt < 1e-9) return Vec3(0, 0, 0);
    return sum / totalDt;
}

}  // namespace engine
