#ifndef RAYTRACER_ENGINE_VEHICLE_STEERING_H
#define RAYTRACER_ENGINE_VEHICLE_STEERING_H

// STEERING ASSIST for a keyboard-driven car (Glenn, 2026-09-14: "it fishtails
// and rolls easily"). A key is a square wave — full lock the instant it is
// pressed, at any speed — and full lock at 90 km/h asks a 2.7 m wheelbase
// for a 5 m radius: the tyres saturate at once, the rear steps out, and
// the driver's instant counter-lock swings it back the other way (the tank
// slapper). Two pure rules stand between the input and the Jolt wheel:
//
//   SPEED-SENSITIVE LOCK  the lock fraction falls with forward speed,
//       1 at and under `fullLockBelow`, 1/2 at `halfLockAt`, then on a
//       1/(1+x²) tail — full lock to park, a few degrees on the freeway,
//       the way every arcade driver (and a real rack's leverage at speed)
//       behaves.
//   RATE LIMIT  the wheel turns at `rateIn` full-scales per second toward a
//       larger angle and `rateOut` back toward centre, so a tap is a nudge
//       and a hold is a smooth wind-on; the return is faster so releasing
//       the key straightens the car without a lag the driver must chase.
//
// Pure and header-only like vehicle_lamps.h: VehicleSystem runs it for the
// seated player each fixed step; tests/test_vehicle_handling.cpp drives the
// Jolt car through it headlessly. AI drivers (citysim's Stanley controller)
// compute an angle, not a key, and bypass it.

#include "../rt_math.h"
#include <algorithm>
#include <cmath>

namespace engine {

// Tuned twice on 2026-09-14: the first cut (full lock to 5 m/s, half at
// 14, 3.5/s on) made Glenn's turns feel long — "the turn arc is very long"
// — though the steady radius measured the same as the raw key's; it was
// the RESPONSE. The yaw assist now carries the spin, so the wheel can be
// quick again: full lock to 8 m/s, half at 22 (about 12 degrees at 90
// km/h), wound on in a sixth of a second.
struct SteerAssist {
    Real fullLockBelow = 8.0;    // m/s: full lock available up to here
    Real halfLockAt = 22.0;      // m/s: half lock here (~80 km/h)
    Real rateIn = 6.0;           // full-scales per second, winding on
    Real rateOut = 8.0;          // full-scales per second, returning to centre
};

// The lock fraction [0,1] available at a forward speed (m/s, sign ignored).
inline Real steerLockAt(Real forwardSpeed, const SteerAssist& a = {}) {
    const Real over = std::max(Real(0), std::fabs(forwardSpeed) - a.fullLockBelow);
    const Real k = std::max(Real(0.01), a.halfLockAt - a.fullLockBelow);
    const Real x = over / k;
    return 1.0 / (1.0 + x * x);
}

// One fixed step of the assist: `prev` is last step's applied steer,
// `input` the raw driver value in [-1,1]; returns the value to hand Jolt.
inline Real shapeSteer(Real prev, Real input, Real forwardSpeed, Real dt,
                       const SteerAssist& a = {}) {
    const Real target = std::clamp(input, Real(-1), Real(1)) * steerLockAt(forwardSpeed, a);
    const bool windingOn = std::fabs(target) > std::fabs(prev) && target * prev >= 0;
    const Real rate = windingOn ? a.rateIn : a.rateOut;
    const Real step = std::clamp(target - prev, -rate * dt, rate * dt);
    return prev + step;
}

struct SteerShaper {
    Real value = 0;
    Real step(Real input, Real forwardSpeed, Real dt, const SteerAssist& a = {}) {
        value = shapeSteer(value, input, forwardSpeed, dt, a);
        return value;
    }
};

}  // namespace engine

#endif
