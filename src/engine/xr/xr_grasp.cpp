#include "xr_grasp.h"

#include <algorithm>
#include <cmath>

#include "xr_touch.h"

namespace engine {

// --- XrGripMemory ---------------------------------------------------------

void XrGripMemory::capture(const std::vector<XrGripPoint>& worldContacts,
                           const Vec3& objectPosition,
                           const Quat& objectOrientation) {
    anchors_.clear();
    anchors_.reserve(worldContacts.size());
    const Quat inv = objectOrientation.conjugate();
    for (const XrGripPoint& c : worldContacts) {
        Anchor a;
        a.id = c.id;
        a.localPoint = inv.rotate(c.point - objectPosition);
        a.localNormal = inv.rotate(c.normal);
        anchors_.push_back(a);
    }
}

void XrGripMemory::clear() { anchors_.clear(); }

bool XrGripMemory::opposed(Real maxPairDot) const {
    // Normals are stored object-local; relative angles between them are the
    // same in any frame, so the pairwise test needs no object pose.
    for (size_t i = 0; i + 1 < anchors_.size(); ++i) {
        for (size_t j = i + 1; j < anchors_.size(); ++j) {
            if (dot(anchors_[i].localNormal, anchors_[j].localNormal) <
                maxPairDot) {
                return true;
            }
        }
    }
    return false;
}

Real XrGripMemory::opening(
    const std::vector<std::pair<int, Vec3>>& liveWorldTips,
    const Vec3& objectPosition, const Quat& objectOrientation) const {
    Real sum = 0;
    int matched = 0;
    for (const Anchor& a : anchors_) {
        for (const auto& tip : liveWorldTips) {
            if (tip.first != a.id) continue;
            const Vec3 anchorWorld =
                objectPosition + objectOrientation.rotate(a.localPoint);
            sum += (tip.second - anchorWorld).length();
            matched++;
            break;
        }
    }
    if (matched == 0) return 0;  // no data must not read as "hand opened"
    return sum / matched;
}

Vec3 XrGripMemory::worldAnchor(size_t i, const Vec3& objectPosition,
                               const Quat& objectOrientation) const {
    return objectPosition + objectOrientation.rotate(anchors_[i].localPoint);
}

// --- XrHandMemory ---------------------------------------------------------

void XrHandMemory::feed(bool tracked, const Vec3& position,
                        const Quat& orientation, Real now) {
    const Real dt = primed_ ? std::max(Real(0), now - lastNow_) : Real(0);
    lastNow_ = now;

    if (tracked) {
        if (coasting_) {
            // Reacquired: blend from wherever the coast left the pose so the
            // hand glides back instead of teleporting.
            coasting_ = false;
            reacquiredAt_ = now;
            blendFromP_ = position_;
            blendFromQ_ = orientation_;
        }

        // Ring push for the velocity fit.
        ring_[head_] = Snap{now, position};
        head_ = (head_ + 1) % kRing;
        if (count_ < kRing) count_++;

        if (reacquiredAt_ >= 0 && now - reacquiredAt_ < config_.blendTime &&
            config_.blendTime > 0) {
            const Real alpha = (now - reacquiredAt_) / config_.blendTime;
            position_ = blendFromP_ + (position - blendFromP_) * alpha;
            orientation_ = Quat::slerp(blendFromQ_, orientation, alpha);
        } else {
            reacquiredAt_ = -1;
            position_ = position;
            orientation_ = orientation;
        }
        primed_ = true;
        return;
    }

    // Untracked. Before the first tracked frame there is nothing to coast.
    if (!primed_) return;

    if (!coasting_) {
        coasting_ = true;
        lostAt_ = now;
        coastVelocity_ = fitVelocity();
        reacquiredAt_ = -1;
    }

    if (now - lostAt_ <= config_.coastBudget) {
        // Continue the arc with decaying velocity; orientation holds (angular
        // extrapolation amplifies noise for no perceptual gain over 150ms).
        position_ += coastVelocity_ * dt;
        if (config_.coastDecay > 0) {
            coastVelocity_ *= std::exp(-dt / config_.coastDecay);
        }
    }
    // Past the budget: freeze in place until reacquired.
}

Vec3 XrHandMemory::peakVelocity(Real window) const {
    if (count_ < 3) return Vec3(0, 0, 0);
    const Snap& newest = ring_[(head_ + kRing - 1) % kRing];
    Vec3 best(0, 0, 0);
    Real bestSq = 0;
    for (size_t k = 2; k < count_; ++k) {
        const Snap& older = ring_[(head_ + kRing - 1 - k) % kRing];
        const Snap& span = ring_[(head_ + kRing - 1 - (k - 2)) % kRing];
        if (newest.t - older.t > window) break;
        const Real dt = span.t - older.t;
        if (dt < 1e-4) continue;
        const Vec3 v = (span.p - older.p) / dt;
        const Real m = dot(v, v);
        if (m > bestSq) {
            bestSq = m;
            best = v;
        }
    }
    return best;
}

Vec3 XrHandMemory::fitVelocity() const {
    // Least-squares slope of the ring's recent positions over time (the
    // XrPoseHistory rationale: one noisy endpoint frame must not fling the
    // coast the way a two-sample finite difference would).
    if (count_ < 2) return Vec3(0, 0, 0);
    const Snap& newest = ring_[(head_ + kRing - 1) % kRing];

    Real sumT = 0;
    Vec3 sumP(0, 0, 0);
    int n = 0;
    for (size_t k = 0; k < count_; ++k) {
        const Snap& s = ring_[(head_ + kRing - 1 - k) % kRing];
        if (newest.t - s.t > config_.fitWindow) break;
        sumT += s.t;
        sumP += s.p;
        n++;
    }
    if (n < 2) return Vec3(0, 0, 0);
    const Real meanT = sumT / n;
    const Vec3 meanP = sumP / static_cast<Real>(n);

    Real denom = 0;
    Vec3 numer(0, 0, 0);
    for (size_t k = 0; k < count_; ++k) {
        const Snap& s = ring_[(head_ + kRing - 1 - k) % kRing];
        if (newest.t - s.t > config_.fitWindow) break;
        const Real dtk = s.t - meanT;
        denom += dtk * dtk;
        numer += (s.p - meanP) * dtk;
    }
    if (denom < 1e-12) return Vec3(0, 0, 0);
    return numer / denom;
}

// --- Placement assist -----------------------------------------------------

Quat xrNearestAxisAligned(const Quat& q) {
    // Snap the rotated basis greedily: the image of local X grabs its
    // dominant signed world axis, local Y grabs its dominant among the
    // remaining two, and Z = cross(X, Y) keeps the frame right-handed
    // (det +1), so the result is always one of the 24 box orientations.
    const Mat4 m = q.normalized().toMat4();

    Vec3 cols[3];
    for (int c = 0; c < 3; ++c) {
        cols[c] = Vec3(m.m[0][c], m.m[1][c], m.m[2][c]);
    }

    auto axisComponent = [](const Vec3& v, int axis) {
        return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
    };
    auto makeAxis = [](int axis, Real sign) {
        Vec3 v(0, 0, 0);
        if (axis == 0) v.x = sign;
        else if (axis == 1) v.y = sign;
        else v.z = sign;
        return v;
    };

    bool taken[3] = {false, false, false};
    Vec3 snapped[3];
    for (int c = 0; c < 2; ++c) {  // Z comes from the cross product
        int best = -1;
        Real bestAbs = -1;
        for (int axis = 0; axis < 3; ++axis) {
            if (taken[axis]) continue;
            const Real a = std::fabs(axisComponent(cols[c], axis));
            if (a > bestAbs) {
                bestAbs = a;
                best = axis;
            }
        }
        taken[best] = true;
        snapped[c] = makeAxis(best, axisComponent(cols[c], best) < 0 ? -1 : 1);
    }
    snapped[2] = cross(snapped[0], snapped[1]);

    Mat4 out;  // identity; fill the 3x3 with the snapped columns
    for (int c = 0; c < 3; ++c) {
        out.m[0][c] = snapped[c].x;
        out.m[1][c] = snapped[c].y;
        out.m[2][c] = snapped[c].z;
    }
    return Quat::fromRotationMatrix(out);
}

XrAlignedPose xrAlignToSupport(const Vec3& halfExtent, const Vec3& position,
                               const Quat& orientation, Real supportTopY,
                               const Vec3& supportMinXZ,
                               const Vec3& supportMaxXZ, Real maxAngleRadians,
                               Real maxSnapDistance) {
    XrAlignedPose out;
    const Quat snapped = xrNearestAxisAligned(orientation);

    // Angle between the held orientation and its snap target.
    const Real d = std::min(
        Real(1), std::fabs(orientation.normalized().dot(snapped)));
    const Real angle = 2 * std::acos(d);
    if (angle > maxAngleRadians) return out;

    // World-space half extents of the snapped box: each local axis lands
    // exactly on a world axis, so project the half extent through |basis|.
    const Mat4 m = snapped.toMat4();
    auto extentAlong = [&](int worldRow) {
        return std::fabs(m.m[worldRow][0]) * halfExtent.x +
               std::fabs(m.m[worldRow][1]) * halfExtent.y +
               std::fabs(m.m[worldRow][2]) * halfExtent.z;
    };
    const Real extX = extentAlong(0);
    const Real extY = extentAlong(1);
    const Real extZ = extentAlong(2);

    // Face-flush on the support; keep the footprint on the rectangle where
    // the rectangle is big enough, else center on it.
    Vec3 target = position;
    target.y = supportTopY + extY;
    const auto clampCenter = [](Real v, Real lo, Real hi, Real ext) {
        if (hi - lo <= 2 * ext) return (lo + hi) * Real(0.5);
        return std::clamp(v, lo + ext, hi - ext);
    };
    target.x = clampCenter(position.x, supportMinXZ.x, supportMaxXZ.x, extX);
    target.z = clampCenter(position.z, supportMinXZ.z, supportMaxXZ.z, extZ);

    if ((target - position).length() > maxSnapDistance) return out;

    out.valid = true;
    out.position = target;
    out.orientation = snapped;
    return out;
}

std::vector<XrGripPoint> xrProbeGrip(
    const std::vector<std::pair<int, Vec3>>& tips, const Vec3& halfExtent,
    bool sphere, const Vec3& position, const Quat& orientation, Real reach) {
    std::vector<XrGripPoint> out;
    for (const auto& tip : tips) {
        const Vec3 onSurface =
            sphere ? xrNearestPointOnSphere(halfExtent.x, position, tip.second)
                   : xrNearestPointOnBox(halfExtent, position, orientation,
                                         tip.second);
        if ((onSurface - tip.second).length() > reach) continue;
        // Press direction: toward the surface point, or toward the centre
        // for a tip already inside (nearest point == the tip itself).
        Vec3 press = onSurface - tip.second;
        if (press.length() < 1e-6) press = position - tip.second;
        if (press.length() < 1e-6) continue;   // tip AT the centre — no info
        out.push_back(
            {tip.first, onSurface, press * (1.0 / press.length())});
    }
    return out;
}

}  // namespace engine
