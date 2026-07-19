#ifndef RAYTRACER_APPS_CITYSIM_DRIVER_CONTROL_H
#define RAYTRACER_APPS_CITYSIM_DRIVER_CONTROL_H

// The PHYSICAL driver (roads-v2.1 R5, plan §3.1 layer 3): turns the behaviour
// stack's plan — a lane frame to follow and a speed to hold (IDM's output) —
// into pedal and wheel inputs for a Jolt wheeled vehicle. Pure math, no Jolt,
// no sim types: testable headless in the driving lab and reusable by any
// actuation tier.
//
//   LATERAL  — Stanley steering: heading error + arctan(k * crossTrack / v).
//     Tracks the lane centreline through straights and junction arcs alike;
//     the same law retires the kinematic tier's junction-mouth overlap (a
//     physical car OCCUPIES its swept path instead of teleporting a ribbon).
//   LONGITUDINAL — the desired speed comes from the SAME brains the kinematic
//     tier uses (IDM follower + signals + junction rules), so behaviour is
//     tier-invariant; only actuation differs. Pedals track the speed error.

#include "../../engine/procgen/city/polygon.h"   // Vec2, Real
#include <algorithm>
#include <cmath>

namespace citysim {

struct DriverInput {
    engine::Real throttle = 0;   // [0,1]
    engine::Real brake = 0;      // [0,1]
    engine::Real steer = 0;      // [-1,1], + = right
};

struct DriverGains {
    engine::Real stanleyK = 1.6;       // cross-track gain (m/s per metre)
    engine::Real maxSteerRad = 0.61;   // wheel lock the [-1,1] input maps to
    engine::Real accelPerThrottle = 3.2;   // m/s^2 at full throttle (approx)
    engine::Real decelPerBrake = 7.0;      // m/s^2 at full brake (approx)
    engine::Real speedGain = 0.9;      // pedal response per m/s of speed error
};

// One control step. `pos`/`heading` are the chassis pose in the plane,
// `speed` its forward speed (m/s, signed). The lane frame is the segment
// a->b the driver should ride; `targetSpeed` is the brains' plan (IDM +
// signals + rules already folded in).
inline DriverInput driveTowards(const engine::Vec2& pos,
                                const engine::Vec2& heading,
                                engine::Real speed, engine::Real targetSpeed,
                                const engine::Vec2& laneA,
                                const engine::Vec2& laneB,
                                const DriverGains& g = {}) {
    using engine::Real;
    using engine::Vec2;
    DriverInput in;

    // --- Stanley lateral: heading error + cross-track correction.
    Vec2 lane = laneB - laneA;
    const Real laneLen = lane.length();
    if (laneLen > 1e-6) {
        lane = lane * (1.0 / laneLen);
        // Signed cross-track: + when the car sits LEFT of the lane (steer
        // right to return).
        const Vec2 rel = pos - laneA;
        const Real cross = lane.x * rel.y - lane.y * rel.x;
        const Real headingErr =
            std::atan2(heading.x * lane.y - heading.y * lane.x,
                       heading.x * lane.x + heading.y * lane.y);
        // steer angle in lane frame: correct heading first, then pull the
        // cross-track to zero, softened at speed (the 1.2 floor keeps the
        // arctan usable from standstill).
        // Sign convention MEASURED in the lab (the steer-sign probe):
        // +steer turns toward -x in the (x,z) plan. In that frame the
        // heading-error term enters POSITIVE and the cross-track term
        // NEGATIVE — derived twice wrong before measuring once.
        const Real steerRad =
            headingErr - std::atan2(g.stanleyK * cross,
                                    std::max(Real(1.2), std::fabs(speed)));
        in.steer = std::clamp(steerRad / g.maxSteerRad, Real(-1), Real(1));
    }

    // --- Longitudinal: pedals track the speed error.
    const Real err = targetSpeed - speed;
    if (err >= 0) {
        in.throttle = std::clamp(err * g.speedGain, Real(0), Real(1));
    } else {
        in.brake = std::clamp(-err * g.speedGain, Real(0), Real(1));
        // Full stop wanted: hold the brake so the car cannot idle-creep.
        if (targetSpeed < 0.05) in.brake = std::max(in.brake, Real(0.6));
    }
    return in;
}

}  // namespace citysim

#endif
