#ifndef RAYTRACER_ENGINE_AI_TRAFFIC_SENSE_H
#define RAYTRACER_ENGINE_AI_TRAFFIC_SENSE_H

#include "lane_follow.h"   // LaneFollower (corridor sensing along the pursuit path)
#include "perception.h"    // VisionCone, sees(), forwardDistance()

#include <cmath>
#include <cstddef>
#include <vector>

namespace engine {

// Real-pose traffic perception (ADR-0062). The planner's car-following runs on
// its own ghosts; these helpers give a PHYSICAL car eyes on the other PHYSICAL
// road users — the car ahead that physics has actually put there, the player's
// car, a pedestrian in the lane — so two real cars can never rely on their plans
// alone to stay apart. Pure + headless (exercised by the bicycle harness).

// What one driver knows about another road user, sensed at its real pose.
struct SensedBody {
    Vec2 pos;
    Vec2 heading{1, 0};   // unit travel direction (meaningless when speed ~ 0)
    Real speed = 0;       // forward speed magnitude (m/s)
    Real halfLength = 2.1;
    // A body cars must ALWAYS yield to (a pedestrian, the on-foot player): the
    // gridlock creep valve never noses through one of these.
    bool yieldAlways = false;
};

// The nearest body ahead that the driver must respect.
struct LeaderSense {
    bool found = false;
    Real gap = 0;         // distance to it (centre-to-centre / along-path, m)
    Real speed = 0;       // leader speed ALONG my travel direction (>= 0)
    Real halfLength = 0;
    Real align = 1;       // cos(angle between our headings); < ~0.5 = cross body
    bool yieldAlways = false;
};

// Scan `bodies` from a forward cone and return the nearest relevant one:
//  - ahead, inside the cone, and laterally within `lateralMax` of my forward
//    axis (my lane — not the oncoming or adjacent one);
//  - travelling my way, OR (nearly) stationary. Moving cross/oncoming traffic
//    is deliberately NOT a leader — braking for it deadlocks junctions (the
//    same lesson as the planner's cone); a stopped anything in my lane is an
//    obstacle no matter which way it points.
// `skip` excludes the sensing car's own entry.
inline LeaderSense senseLeader(const VisionCone& cone, const std::vector<SensedBody>& bodies,
                               Real lateralMax = 2.2, Real stationarySpeed = 1.0,
                               int skip = -1) {
    LeaderSense out;
    Real best = 1e30;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (static_cast<int>(i) == skip) continue;
        const SensedBody& b = bodies[i];
        if (!sees(cone, b.pos)) continue;
        if (forwardDistance(cone, b.pos) <= 0.1) continue;
        Vec2 d = b.pos - cone.origin;
        Real lat = std::fabs(cone.forward.x * d.y - cone.forward.y * d.x);
        if (lat > lateralMax) continue;
        Real align = dot(cone.forward, b.heading);
        if (b.speed > stationarySpeed && align < 0.5) continue;
        Real centreDist = d.length();
        if (centreDist < best) {
            best = centreDist;
            out.found = true;
            out.gap = centreDist;
            out.speed = align > 0 ? b.speed * align : 0.0;
            out.halfLength = b.halfLength;
            out.align = align;
            out.yieldAlways = b.yieldAlways;
        }
    }
    return out;
}

// Corridor sensing ALONG THE PURSUIT PATH (the sharper tool when a path exists):
// the nearest body within `corridorHalf` of the lane polyline over the next
// `range` metres of path. Unlike the straight cone this bends with the road — a
// stopped queue in the ONCOMING lane of a curve never reads as "ahead of me"
// (with a straight cone it does, and two such queues freeze each other), and a
// body only blocks me if it actually sits on MY lane's path. `gap` comes back as
// along-path distance.
//
// Cross-traffic rule, refined for PHYSICAL cars: far-field moving cross traffic
// is still ignored (braking 20 m early for every crosser is the junction
// deadlock), but within `crossNearRange` a moving crosser ON MY PATH is a
// collision about to happen — brake for it. Two crossers who both stop resolve
// via the caller's creep valve. Evaluated against the path tangent where the
// body sits.
inline LeaderSense senseAlongPath(const LaneFollower& lf, Real range, Real corridorHalf,
                                  const std::vector<SensedBody>& bodies,
                                  int skip = -1, Real stationarySpeed = 1.0,
                                  Real crossNearRange = 8.0) {
    LeaderSense out;
    if (!lf.valid()) return out;
    const std::vector<Vec2>& P = lf.points();
    const std::vector<Real>& S = lf.arcs();
    const Real s0 = lf.station(), sEnd = s0 + range;
    Real bestGap = 1e30;
    for (std::size_t bi = 0; bi < bodies.size(); ++bi) {
        if (static_cast<int>(bi) == skip) continue;
        const SensedBody& b = bodies[bi];
        // This body's closest in-corridor projection onto the path window.
        Real gap = 1e30, align = 1;
        for (std::size_t i = lf.segmentHint(); i + 1 < P.size() && S[i] <= sEnd; ++i) {
            Vec2 a = P[i], ab = P[i + 1] - a;
            Real L2 = ab.x * ab.x + ab.y * ab.y;
            if (L2 < 1e-12) continue;
            Real t = ((b.pos.x - a.x) * ab.x + (b.pos.y - a.y) * ab.y) / L2;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            Vec2 q = a + ab * t;
            Real dx = b.pos.x - q.x, dy = b.pos.y - q.y;
            if (dx * dx + dy * dy > corridorHalf * corridorHalf) continue;
            Real L = std::sqrt(L2);
            Real along = S[i] + L * t - s0;
            if (along <= 0.3 || along > range || along >= gap) continue;
            gap = along;
            align = dot(Vec2(ab.x / L, ab.y / L), b.heading);
        }
        if (gap > range) continue;
        // Moving cross traffic: ignore far-field (flow control belongs to the
        // signals), brake near-field (imminent collision on my path).
        if (b.speed > stationarySpeed && align < 0.5 && gap > crossNearRange) continue;
        if (gap < bestGap) {
            bestGap = gap;
            out.found = true;
            out.gap = gap;
            out.speed = align > 0 ? b.speed * align : 0.0;
            out.halfLength = b.halfLength;
            out.align = align;
            out.yieldAlways = b.yieldAlways;
        }
    }
    return out;
}

// Speed permitted behind that leader: full speed beyond the slow zone, a linear
// ramp down to ZERO at bumper contact + buffer (length-aware, like the planner's
// followCap), but never below a moving leader's own pace — a convoy flows at the
// leader's speed instead of compressing to a crawl.
inline Real followSpeed(Real desired, const LeaderSense& l, Real ownHalfLength,
                        Real bumperGap = 0.8, Real slowZone = 12.0) {
    if (!l.found || desired <= 0) return desired;
    Real minGap = ownHalfLength + l.halfLength + bumperGap;
    if (l.gap <= minGap) return 0;
    if (l.gap >= minGap + slowZone) return desired;
    Real cap = desired * (l.gap - minGap) / slowZone;
    if (l.speed > cap) cap = l.speed;      // match a moving leader
    return cap < desired ? cap : desired;
}

// Knockdown-and-recover for walkers (ADR-0062): a hit — a car shoving through, a
// blast — puts a walker DOWN for a spell; it then gets up and walks on. Pure
// timer logic so the trigger policy (contact, proximity, impulse) stays with the
// caller and the behaviour is headless-testable.
struct KnockdownTimer {
    Real downSeconds = 2.5;   // how long a hit keeps the walker down
    Real timer = 0;           // > 0 -> currently down

    bool down() const { return timer > 0; }
    void knock() { timer = downSeconds; }        // (re-)floor the walker
    bool update(Real dt) {                        // advance; true while still down
        if (timer > 0) timer -= dt;
        if (timer < 0) timer = 0;
        return down();
    }
};

// Stall watchdog: the brain wants motion but the car isn't moving (wedged on a
// pole, beached, flipped). Fires once after `stallSeconds` of continuous stall,
// then re-arms — the caller runs its recovery (e.g. resetVehicleUpright).
struct StuckDetector {
    Real stallSeconds = 4.0;   // this long stalled -> recover
    Real minCommand = 1.0;     // only count while the brain asks for real speed
    Real minMotion = 0.3;      // actual speed under this counts as stalled
    Real timer = 0;

    bool update(Real commandedSpeed, Real actualSpeed, Real dt) {
        if (commandedSpeed > minCommand && actualSpeed < minMotion) timer += dt;
        else timer = 0;
        if (timer >= stallSeconds) { timer = 0; return true; }
        return false;
    }
};

}  // namespace engine

#endif
