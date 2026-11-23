// THE PLAYER ON A BUS (ADR-0486): the bus's own rules, in its body frame -- where you get on, where you may stand,
// where you get off. (Being carried in its frame is engine/riding.h's, the trains' too.) A cabin laid out as the kit bus
// lays its saloon: seat pairs either side of a 0.56 m aisle, two doors on the kerb side (+X), the driver ahead.
#include "test_framework.h"

#include <cmath>
#include "../src/apps/citysim/city_bus_ride.h"
#include "../src/apps/citysim/city_car.h"
#include "../src/apps/citysim/city_driving.h"

using namespace engine;
using namespace citysim;

namespace {
BusCabin kitBus() {
    BusCabin c;
    c.halfWidth = 1.275;
    c.halfLength = 5.9;
    c.driverSeat = Vec3(-0.7, -0.75, 4.6);
    for (Real z = 3.2; z > -5.0; z -= 0.8)
        for (Real x : {-0.96, -0.52, 0.52, 0.96}) {
            if (x > 0 && z < 0.9 && z > -0.6) continue;   // (no seats in front of the middle door)
            c.seats.push_back(Vec3(x, -0.80, z));
            c.facing.push_back(Vec3(0, 0, 1));
        }
    c.doors.push_back(Vec3(0.83, -1.24, 4.3));   // the front door, beside the driver's partition
    c.doors.push_back(Vec3(0.83, -1.24, 0.2));   // the middle door
    return c;
}
}  // namespace

TEST_CASE(bus_ride_the_aisle_and_the_door_wells_are_walkable_the_seats_are_not) {
    const BusCabin c = kitBus();
    const Real floor = busFloor(c);
    CHECK(floor == Real(-1.24));
    CHECK(busWalkable(c, Vec3(0, floor, 0)));        // the aisle, mid-bus
    CHECK(busWalkable(c, Vec3(0, floor, -4.5)));     // ...to the back
    CHECK(busWalkable(c, Vec3(0, floor, 3.9)));      // ...and up to the partition
    CHECK(!busWalkable(c, Vec3(0, floor, 5.5)));     // not into the driver's cab
    CHECK(!busWalkable(c, Vec3(-0.9, floor, 2.0)));  // not through a seat
    CHECK(!busWalkable(c, Vec3(0.9, floor, 2.0)));
    CHECK(busWalkable(c, Vec3(0.8, floor, 0.2)));    // the middle door well, out to the door
    CHECK(busWalkable(c, Vec3(0.8, floor, 4.3)));    // the front door's
    CHECK(busDoorWell(c, Vec3(0.8, floor, 0.2)) == 1);
    CHECK(busDoorWell(c, Vec3(0.0, floor, 2.0)) == -1);
    CHECK(!busWalkable(c, Vec3(-1.1, floor, 0.2)));  // the far wall at the door: no door there
}

TEST_CASE(bus_ride_you_get_on_from_a_doorway_and_off_through_one) {
    const BusCabin c = kitBus();
    const Real floor = busFloor(c);
    // ON: feet on the pavement in a doorway's mouth (the kerb a little lower than the floor)
    CHECK(busDoorMouth(c, Vec3(1.6, floor - 0.2, 0.2)) == 1);
    CHECK(busDoorMouth(c, Vec3(1.6, floor - 0.2, 4.3)) == 0);
    CHECK(busDoorMouth(c, Vec3(1.6, floor - 0.2, 2.0)) == -1);   // beside the bus, not at a door
    CHECK(busDoorMouth(c, Vec3(-1.6, floor - 0.2, 0.2)) == -1);  // the road side: no door
    CHECK(busDoorMouth(c, Vec3(3.5, floor - 0.2, 0.2)) == -1);   // too far out on the pavement
    CHECK(busDoorMouth(c, Vec3(1.6, floor + 2.5, 0.2)) == -1);   // not at the floor's height (on a roof)
    // ...inside the door you came in by, on the floor, walkable
    const Vec3 in = busInside(c, 1);
    CHECK(busWalkable(c, in) && in.y == floor && busStepsOff(c, in) < 0);
    // OFF: past the wall in a door well; where you land is outside it, on the pavement side
    CHECK(busStepsOff(c, Vec3(1.2, floor, 0.2)) == 1);
    CHECK(busStepsOff(c, Vec3(2.0, floor, 4.3)) == 0);
    CHECK(busStepsOff(c, Vec3(0.8, floor, 0.2)) < 0);
    CHECK(busStepsOff(c, Vec3(0.0, floor, 2.0)) < 0);
    CHECK(busStepsOff(c, Vec3(1.5, floor, 2.0)) < 0);   // (through the wall beside the door: no way out)
    const Vec3 out = busOutside(c, 1);
    CHECK(out.x > c.halfWidth && out.z == Real(0.2));
}

TEST_CASE(bus_ride_a_standing_room_bus_has_a_wide_aisle_between_its_wall_benches) {
    BusCabin c = kitBus();
    c.seats.clear();
    c.facing.clear();
    for (Real z = 3.2; z > -5.0; z -= 0.48)
        for (Real side : {-1.0, 1.0}) {
            c.seats.push_back(Vec3(side * 1.05, -0.80, z));
            c.facing.push_back(Vec3(-side, 0, 0));   // facing the aisle
        }
    const Real floor = busFloor(c);
    CHECK(busWalkable(c, Vec3(0.5, floor, 1.5)));    // well off the middle: the floor between the benches
    CHECK(busWalkable(c, Vec3(-0.5, floor, -2.0)));
    CHECK(!busWalkable(c, Vec3(-1.0, floor, -2.0)));  // the bench itself
}

// THE DOORS (ADR-0487): shut while it drives; as it stands at its stop they slide open over `door_seconds`, and are shut
// again `doors_shut_before` before it pulls away; fully open is when anyone gets on or off (bus_service.json).
TEST_CASE(bus_doors_open_as_it_stands_and_are_shut_before_it_pulls_away) {
    const BusService& s = busService();
    CHECK(s.usable && s.doorSeconds > 0);
    CityBusRuns r;
    r.standing = false;
    r.dwellLeft = 0;
    CHECK(busDoorsOpenness(r) == 0 && !busDoorsOpen(r));                                  // driving: shut
    r.standing = true;
    r.dwellLeft = s.dwellSeconds;
    CHECK(busDoorsOpenness(r) == 0);                                                        // just stopped: still shut
    r.dwellLeft = s.dwellSeconds - s.doorSeconds * 0.5;
    CHECK(std::fabs(busDoorsOpenness(r) - 0.5) < 1e-9 && !busDoorsOpen(r));               // opening: nobody through yet
    r.dwellLeft = s.dwellSeconds - s.doorSeconds;
    CHECK(busDoorsOpen(r));                                                                 // open
    r.dwellLeft = s.dwellSeconds * 0.5;
    CHECK(busDoorsOpen(r));
    r.dwellLeft = s.doorsShutBefore + s.doorSeconds * 0.5;
    CHECK(std::fabs(busDoorsOpenness(r) - 0.5) < 1e-9 && !busDoorsOpen(r));               // closing
    r.dwellLeft = s.doorsShutBefore;
    CHECK(busDoorsOpenness(r) == 0);                                                        // shut before it goes
    r.dwellLeft = 0;
    CHECK(busDoorsOpenness(r) == 0);
}
