#ifndef RAYTRACER_ENGINE_AI_TRAFFIC_SENSE_H
#define RAYTRACER_ENGINE_AI_TRAFFIC_SENSE_H

#include "perception.h"   // VisionCone, sees(), forwardDistance()

#include <cmath>
#include <cstddef>
#include <vector>

namespace engine {

// Real-pose traffic perception (ADR-0061). The planner's car-following runs on
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
};

// The nearest body ahead that the driver must respect.
struct LeaderSense {
    bool found = false;
    Real gap = 0;         // centre-to-centre distance (m)
    Real speed = 0;       // leader speed ALONG my heading (>= 0)
    Real halfLength = 0;
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
