#ifndef RAYTRACER_ENGINE_VEHICLE_FUEL_H
#define RAYTRACER_ENGINE_VEHICLE_FUEL_H

// FUEL (ADR-0357; Glenn, 2026-11-07: "fuel and gas stations ... when empty: the engine dies, coast"). A car's tank
// burns while its engine runs -- a trickle at idle, more the harder the throttle is pressed -- and at empty the
// engine dies: the throttle does nothing, the starter turns it over and it will not catch, and the car coasts on
// what speed it has. A pump fills it again. Pure, so the tests can hold it; the tuning is data (vehicles.lua
// KIT_FUEL, read per car into its spec as `fuel`).

#include "../rt_math.h"
#include <algorithm>
#include <cmath>

namespace engine {

struct FuelRule {
    Real tank = 45.0;     // litres
    Real idle = 1.2;      // litres an hour, ticking over
    Real full = 30.0;     // ...and at full throttle
    Real pace = 12.0;     // game time: the burn runs this many times the real one (a full tank is minutes, not hours)
    Real start = 1.0;     // the share of the tank a car starts with
    Real fill = 9.0;      // litres a second a pump pours
    Real low = 0.15;      // the share the readout warns below
};

// Litres a running engine burns in `dt` seconds at `throttle` (-1..1: reverse burns like forward).
inline Real fuelBurn(const FuelRule& r, Real throttle, Real dt) {
    const Real t = std::min(Real(1), std::fabs(throttle));
    return (r.idle + (r.full - r.idle) * t) * r.pace * dt / 3600.0;
}

// The tank after a pump has poured for `dt` seconds.
inline Real fuelPour(const FuelRule& r, Real litres, Real dt) { return std::min(r.tank, litres + r.fill * dt); }

}  // namespace engine

#endif
