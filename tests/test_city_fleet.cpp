#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

#include <algorithm>
#include <cmath>
#include <set>

using namespace engine;
using namespace citysim;

// The composable vehicle fleet (ADR-0061 Phase 4): the cars are built from a shared body table (dimensions + type), so
// they come in real sizes -- sedans, hatchbacks, SUVs, pickups, a van, a box truck. The households' cars of the driving
// take their bodies from it (CitySim::ambientSlotFor).

TEST_CASE(fleet_offers_varied_bodies_including_large_ones) {
    CHECK(vehicleFleetSize() >= 6);
    std::set<int> types;
    Real sedanLen = 0, maxLen = 0;
    bool hasBoxTruck = false;
    for (int s = 0; s < vehicleFleetSize(); ++s) {
        const VehicleBody& b = vehicleFleetBody(s);
        types.insert(static_cast<int>(b.type));
        if (b.type == VehicleType::Sedan) sedanLen = b.length;
        if (b.type == VehicleType::BoxTruck) hasBoxTruck = true;
        maxLen = std::max(maxLen, b.length);
        CHECK(b.length > 0 && b.width > 0 && b.height > 0);
    }
    CHECK(types.size() >= 4);         // a genuinely mixed fleet
    CHECK(hasBoxTruck);               // includes a big vehicle
    CHECK(maxLen > sedanLen);         // ...that is longer than the player's sedan
}

TEST_CASE(fleet_slot_wraps_and_is_stable) {
    // A slot beyond the table wraps back into it (cars index by their number).
    const VehicleBody& a = vehicleFleetBody(0);
    const VehicleBody& b = vehicleFleetBody(vehicleFleetSize());
    CHECK(a.length == b.length);
    CHECK(a.type == b.type);
    // Negative wraps too (defensive).
    const VehicleBody& c = vehicleFleetBody(-vehicleFleetSize());
    CHECK(c.length == a.length);
}

// A SERVICE BODY (a fire engine, a bus, a cab) is never dealt to an ordinary car: ambientSlotFor skips them. (Moved
// from test_city_bus.cpp with ADR-0460; the stationed fire engine it also checked went with ADR-0461/0462.)
TEST_CASE(service_bodies_are_never_dealt_to_ordinary_cars) {
    CitySim sim;
    std::vector<VehicleBody> fleet = {{4.6, 1.8, 1.45, VehicleType::Sedan}, {4.2, 1.75, 1.5, VehicleType::Hatchback},
                                      {9.2, 2.52, 3.21, VehicleType::FireEngine}, {11.3, 2.55, 3.2, VehicleType::Bus}};
    sim.setFleet(fleet);
    for (int i = 0; i < 400; ++i) {
        const int s = sim.ambientSlotFor(i);
        CHECK(s == 0 || s == 1);   // only the cars
    }
}
