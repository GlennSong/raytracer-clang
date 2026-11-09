// FUEL (ADR-0357): what a running engine burns, and what a pump pours.

#include "test_framework.h"
#include "../src/engine/vehicle_fuel.h"

using namespace engine;

TEST_CASE(an_engine_burns_more_the_harder_it_is_pressed) {
    const FuelRule r;
    const Real idle = fuelBurn(r, 0.0, 1.0), half = fuelBurn(r, 0.5, 1.0), full = fuelBurn(r, 1.0, 1.0);
    CHECK(idle > 0);                         // ticking over still burns
    CHECK(half > idle && full > half);
    CHECK(fuelBurn(r, -1.0, 1.0) == full);   // reverse burns like forward
    CHECK(fuelBurn(r, 3.0, 1.0) == full);    // a pedal past the floor is the floor
    // a full tank at full throttle lasts minutes of play, not an afternoon -- nor seconds
    const Real minutes = r.tank / full / 60.0;
    std::printf("    [fuel] a %.0f L tank lasts %.1f min flat out, %.0f min idling\n", r.tank, minutes, r.tank / idle / 60.0);
    CHECK(minutes > 4.0 && minutes < 20.0);
}

TEST_CASE(a_pump_fills_the_tank_and_stops_at_full) {
    const FuelRule r;
    Real litres = 0;
    int steps = 0;
    while (litres < r.tank && steps < 100000) { litres = fuelPour(r, litres, 1.0 / 60); ++steps; }
    CHECK(litres == r.tank);
    const Real seconds = steps / 60.0;
    CHECK(seconds > 2.0 && seconds < 12.0);   // a few seconds holding R, not a wait
    CHECK(fuelPour(r, r.tank, 1.0) == r.tank);
}
