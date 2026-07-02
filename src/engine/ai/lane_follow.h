#ifndef RAYTRACER_ENGINE_AI_LANE_FOLLOW_H
#define RAYTRACER_ENGINE_AI_LANE_FOLLOW_H

#include "driver_agent.h"   // DriverCommand (and Vec2/Real via polygon.h)

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace engine {

// Closed-loop lane following (ADR-0061): PURE PURSUIT over a lane-centre
// polyline. The AI driver's heading is derived from the car's REAL position —
// track progress along the path, aim at a point a speed-proportional distance
// ahead — instead of chasing a planner ghost open-loop. That is what makes an
// agent-driven car recover its lane after a collision, hug a corner instead of
// cutting it, and stay stable at speed (the lookahead grows with speed, which
// damps the steering the way a human looks further down the road when fast).
//
// Pure and engine-core: no physics, no ECS — headless-testable by closing the
// loop against any vehicle model (tests use a kinematic bicycle model; the
// viewer closes it against the real Jolt vehicle).
class LaneFollower {
public:
    // Set the path (consecutive duplicate points are dropped). Resets progress.
    void setPath(std::vector<Vec2> pts) {
        pts_.clear();
        cum_.clear();
        seg_ = 0;
        station_ = 0;
        for (const Vec2& p : pts)
            if (pts_.empty() || (p - pts_.back()).length() > 1e-6) pts_.push_back(p);
        if (pts_.size() < 2) { pts_.clear(); return; }
        cum_.assign(pts_.size(), 0.0);
        for (std::size_t i = 1; i < pts_.size(); ++i)
            cum_[i] = cum_[i - 1] + (pts_[i] - pts_[i - 1]).length();
    }

    bool valid() const { return pts_.size() >= 2; }
    Real length() const { return cum_.empty() ? 0.0 : cum_.back(); }
    Real station() const { return station_; }               // arc-length progress
    Real remaining() const { return length() - station_; }
    // Path internals for corridor queries (traffic_sense.h senseAlongPath).
    const std::vector<Vec2>& points() const { return pts_; }
    const std::vector<Real>& arcs() const { return cum_; }
    std::size_t segmentHint() const { return seg_; }

    // Advance progress from the car's REAL position and return the lateral
    // distance to the path. Progress is MONOTONIC and the closest-point search
    // runs a bounded window forward of the last segment, so a path that doubles
    // back near itself (a hairpin, a block loop) can't snap progress onto the
    // wrong leg.
    Real update(const Vec2& pos) {
        if (!valid()) return 0.0;
        std::size_t bestSeg = seg_;
        Real bestT = 0.0, bestD2 = std::numeric_limits<Real>::max();
        std::size_t end = std::min(pts_.size() - 1, seg_ + kSearchWindow);
        for (std::size_t i = seg_; i < end; ++i) {
            Vec2 a = pts_[i], ab = pts_[i + 1] - a;
            Real L2 = ab.x * ab.x + ab.y * ab.y;
            Real t = L2 < 1e-12 ? 0.0 : ((pos.x - a.x) * ab.x + (pos.y - a.y) * ab.y) / L2;
            t = std::max(Real(0), std::min(Real(1), t));
            Vec2 q = a + ab * t;
            Real dx = pos.x - q.x, dy = pos.y - q.y;
            Real d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; bestSeg = i; bestT = t; }
        }
        seg_ = bestSeg;
        Real s = cum_[bestSeg] + (pts_[bestSeg + 1] - pts_[bestSeg]).length() * bestT;
        if (s > station_) station_ = s;                     // never backtracks
        return std::sqrt(bestD2);
    }

    // The path point `ahead` metres past the current station (clamped to the end).
    Vec2 lookahead(Real ahead) const {
        if (!valid()) return Vec2();
        Real s = station_ + std::max(Real(0), ahead);
        if (s >= length()) return pts_.back();
        std::size_t i = seg_;
        while (i + 2 < pts_.size() && cum_[i + 1] < s) ++i;
        Real segLen = cum_[i + 1] - cum_[i];
        Real t = segLen > 1e-9 ? (s - cum_[i]) / segLen : 0.0;
        return pts_[i] + (pts_[i + 1] - pts_[i]) * t;
    }

private:
    static constexpr std::size_t kSearchWindow = 12;   // segments scanned per update
    std::vector<Vec2> pts_;
    std::vector<Real> cum_;    // cumulative arc length per point
    std::size_t seg_ = 0;      // monotonic segment hint
    Real station_ = 0;
};

// Speed-proportional pursuit lookahead: short when crawling (tight tracking),
// long at speed (stability). The classic pure-pursuit stability lever.
inline Real pursuitLookahead(Real speed, Real minLook = 4.0, Real gain = 0.7) {
    return minLook + gain * std::max(Real(0), speed);
}

// The pursuit command: aim the desired heading at the lookahead point from the
// car's real position. desiredSpeed is the caller's plan (signals / following /
// catch-up live above this), except for one built-in guard: it is capped by the
// REMAINING path (endBrakeGain * metres left), so the car eases to a stop at the
// path end. Without the cap, a car commanded to cruise past its last point
// orbits it at full lock forever — the harness caught exactly that.
inline DriverCommand pursuitCommand(const LaneFollower& lf, const Vec2& pos,
                                    Real desiredSpeed, Real lookaheadDist,
                                    Real endBrakeGain = 0.5) {
    DriverCommand c;
    c.desiredSpeed = desiredSpeed;
    if (lf.valid()) {
        c.desiredSpeed = std::min(c.desiredSpeed, endBrakeGain * lf.remaining());
        Vec2 d = lf.lookahead(lookaheadDist) - pos;
        Real l = std::sqrt(d.x * d.x + d.y * d.y);
        if (l > 1e-6) c.desiredHeading = d * (1.0 / l);
    }
    return c;
}

}  // namespace engine

#endif
