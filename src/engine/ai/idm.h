#ifndef RAYTRACER_ENGINE_AI_IDM_H
#define RAYTRACER_ENGINE_AI_IDM_H

#include "../procgen/city/polygon.h"   // Real
#include <algorithm>
#include <cmath>

namespace engine {

// The Intelligent Driver Model (roads-v2 plan §3.1) — the standard smooth
// car-follower and the sim's anti-pile-up backbone. One law covers free
// acceleration toward the desired speed AND braking for a leader: with sane
// parameters the model is collision-free (the interaction term grows without
// bound as the gap closes), so followers ease off long before contact instead
// of riding a linear speed ramp into the bumper ahead. Pure and headless.
struct IdmParams {
    Real accelMax = 2.5;    // comfortable acceleration a (m/s^2)
    Real decelComf = 3.5;   // comfortable braking b (m/s^2) — the model may
                            // exceed it in emergencies; the caller clamps
    Real headway = 1.1;     // desired time headway T (s)
    Real minGap = 2.0;      // standstill jam gap s0 (m, bumper to bumper)
    Real delta = 4.0;       // free-acceleration exponent
};

// Acceleration for a car at speed `v` seeking `v0`, with net (bumper) gap `s`
// to its leader and closing speed `dv` = v - vLeader. Pass s = +infinity (or
// any huge value) when there is no leader — the interaction term vanishes and
// the law reduces to free acceleration toward v0.
inline Real idmAccel(Real v, Real v0, Real s, Real dv, const IdmParams& p = {}) {
    v = std::max(Real(0), v);
    v0 = std::max(Real(0.1), v0);
    const Real free = 1.0 - std::pow(v / v0, p.delta);
    if (!(s < Real(1e8))) return p.accelMax * free;      // no leader (also NaN-safe)
    s = std::max(Real(0.05), s);
    const Real sStar = p.minGap + std::max(Real(0), v * p.headway +
                       v * dv / (2.0 * std::sqrt(p.accelMax * p.decelComf)));
    const Real ratio = sStar / s;
    return p.accelMax * (free - ratio * ratio);
}

// One integration step of the law, clamped so a single tick can neither
// reverse the car nor brake harder than `hardDecel` (the physical limit the
// vehicle/tuning allows; IDM's raw output is unbounded in emergencies).
inline Real idmStep(Real v, Real v0, Real s, Real dv, Real dt,
                    Real hardDecel, const IdmParams& p = {}) {
    Real a = idmAccel(v, v0, s, dv, p);
    a = std::max(a, -hardDecel);
    return std::max(Real(0), v + a * dt);
}

}  // namespace engine

#endif
