// THE BUS's RULES for a rider (ADR-0486): where you get on, stand and get off, in the bus's body frame. Plain functions
// (in libcitysim, for the tests); the system that carries the player is city_bus_ride.cpp.
#include "city_bus_ride.h"

#include "city_car.h"   // CityBusRuns
#include "../../engine/camera/fly_camera_controller.h"
#include "../../engine/input/input_map.h"
#include "../../engine/interact_broker.h"
#include "../../engine/riding.h"
#include "../../engine/systems/physics_system.h"
#include "../../engine/world.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>

namespace citysim {

using engine::Entity;
using engine::Mat4;
using engine::Real;
using engine::Transform;
using engine::Vec3;

namespace {
constexpr Real kDoorHalf = 0.55;    // m: half a doorway, along the bus
constexpr Real kSeatClear = 0.24;   // m: from a seat's hip point to the aisle's edge beside it (half a seat's width)
constexpr Real kMouthOut = 1.3;     // m: how far out of a doorway, on the pavement, walking in still gets you on
constexpr Real kOffPast = 0.30;     // m: past the door's floor point, toward the kerb: you are out of the door

// The aisle's half width: from the middle to the nearest seat's edge (a bus of seat pairs: the gap between the pairs;
// a standing-room bus: the floor between its wall benches), never under a body's width.
Real aisleHalf(const BusCabin& c) {
    Real near = c.halfWidth;
    for (const Vec3& s : c.seats) near = std::min(near, std::fabs(s.x));
    return std::clamp(near - kSeatClear, Real(0.25), std::max(Real(0.25), c.halfWidth - Real(0.5)));
}
// The saloon's length: from behind the driver's partition to the rearmost seat
void saloonZ(const BusCabin& c, Real& rear, Real& front) {
    Real lo = c.halfLength;
    for (const Vec3& s : c.seats) lo = std::min(lo, s.z);
    rear = std::max(-c.halfLength + Real(0.3), lo - Real(0.4));
    front = c.driverSeat.z - 0.6;
    for (const Vec3& d : c.doors) front = std::max(front, std::min(d.z, c.halfLength - Real(0.4)));
}
}  // namespace

Real busFloor(const BusCabin& c) { return c.doors.empty() ? Real(0) : c.doors.front().y; }

int busDoorWell(const BusCabin& c, const Vec3& l) {
    for (std::size_t i = 0; i < c.doors.size(); ++i) {
        const Vec3& d = c.doors[i];
        const Real side = d.x >= 0 ? Real(1) : Real(-1);
        if (std::fabs(l.z - d.z) < kDoorHalf && l.x * side >= -aisleHalf(c) && l.x * side <= std::fabs(d.x) + kOffPast + 0.05)
            return static_cast<int>(i);
    }
    return -1;
}

bool busWalkable(const BusCabin& c, const Vec3& l) {
    if (busDoorWell(c, l) >= 0) return true;
    Real rear = 0, front = 0;
    saloonZ(c, rear, front);
    return std::fabs(l.x) <= aisleHalf(c) && l.z >= rear && l.z <= front;
}

int busDoorMouth(const BusCabin& c, const Vec3& f) {
    for (std::size_t i = 0; i < c.doors.size(); ++i) {
        const Vec3& d = c.doors[i];
        const Real side = d.x >= 0 ? Real(1) : Real(-1), out = f.x * side;
        if (std::fabs(f.z - d.z) < kDoorHalf - 0.1 && out > std::fabs(d.x) - 0.1 && out < std::fabs(d.x) + kMouthOut &&
            std::fabs(f.y - d.y) < 1.0)
            return static_cast<int>(i);
    }
    return -1;
}

int busStepsOff(const BusCabin& c, const Vec3& l) {
    for (std::size_t i = 0; i < c.doors.size(); ++i) {
        const Vec3& d = c.doors[i];
        if (std::fabs(l.z - d.z) < kDoorHalf && l.x * (d.x >= 0 ? 1 : -1) > std::fabs(d.x) + kOffPast) return static_cast<int>(i);
    }
    return -1;
}

Vec3 busInside(const BusCabin& c, int door) {
    const Vec3& d = c.doors[static_cast<std::size_t>(door)];
    return Vec3(d.x * Real(0.6), d.y, d.z);
}

Vec3 busOutside(const BusCabin& c, int door) {
    const Vec3& d = c.doors[static_cast<std::size_t>(door)];
    return Vec3(d.x + (d.x >= 0 ? Real(1) : Real(-1)) * Real(1.0), d.y, d.z);
}

}  // namespace citysim
