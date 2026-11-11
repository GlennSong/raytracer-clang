#ifndef RAYTRACER_APPS_DRIVING_TRAFFIC_STATE_H
#define RAYTRACER_APPS_DRIVING_TRAFFIC_STATE_H

// WHERE EVERY CAR IS, AND WHO IS ON EACH PATH (driving-model-design.md 2 and 7.3; issue #249).
//
// A car's whole truth is a small record: the path its front bumper is on, how far along it, how far to the side of the
// centre line, its speed and acceleration, whether anyone is driving it, and the earlier paths its body still lies on
// (its TRAIL). Where it is in the world and which way it points are READ OFF the path's curve (carPose) and never
// stored, so they can never disagree with it.
//
// From the records, one function (rebuildPathLists) builds, for every path, the cars with any part of their body on
// it, in order of distance along. The questions other code asks -- the car ahead, the car behind, who is on a stretch --
// are pure functions of the records and the lists. Nothing here moves a car.
//
// Same result every run: the lists are sorted whole (position, then car id), so they never depend on the order the
// records are stored in, and no answer does either.
//
// Units: metres, seconds.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/rule_set.h"
#include "apps/driving/vehicle_class.h"
#include "rt_math.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

// WHICH CAR: the host's number for it (the city sim's vehicle index). The library hands it back on events and answers,
// and the lists and the hash are in its order.
struct CarId {
    static constexpr uint32_t kNone = 0xFFFFFFFFu;
    uint32_t index = kNone;

    CarId() = default;
    explicit CarId(uint32_t i) : index(i) {}
    bool valid() const { return index != kNone; }
    bool operator==(const CarId& o) const { return index == o.index; }
    bool operator!=(const CarId& o) const { return index != o.index; }
    bool operator<(const CarId& o) const { return index < o.index; }
};

// A PLACE ON THE ROAD: a path of the lane graph (a lane or a connector) and a distance along it, in metres from its
// start. A trip's destination, a hold and a car's front are each one of these.
struct PathPoint {
    engine::PathId path;
    Real along = 0;

    bool operator==(const PathPoint& o) const { return path == o.path && along == o.along; }
    bool operator!=(const PathPoint& o) const { return !(*this == o); }
};

// HOW MANY EARLIER PATHS a car's body may still lie on behind the path its front is on. A 12 m bus across a junction
// cut into short pieces covers the lane it came from, a connector or two and the lane it entered; six covers that with
// room.
// A car that would need more is refused when placed.
constexpr std::size_t kMaxTrail = 6;

// ONE CAR ON THE ROAD, moving or parked (a parked car is a car at rest that nobody is driving).
struct Car {
    CarId id;                         // the host's number for it
    uint32_t vehicleClass = 0;        // index into the VehicleClasses table
    engine::PathId path;              // the path its front bumper is on
    // The second path it is ALSO listed on while changing lanes (the lane it is moving into or out of); none otherwise.
    engine::PathId besidePath;
    Real along = 0;                   // where its front bumper is: metres along `path`
    // How far its middle is to the side of the path's centre line: + toward the kerb side, - toward the offside. Zero
    // except during a lane change.
    Real offset = 0;
    Real speed = 0;                   // m/s along the path
    Real acceleration = 0;            // m/s2 along the path
    // The earlier paths its body still lies on, nearest first: trail[0] leads into `path`, trail[1] into trail[0], ...
    std::array<engine::PathId, kMaxTrail> trail;
    uint8_t trailCount = 0;           // how many of `trail` are in use
    bool driven = false;              // someone is driving it (it is on a trip)
};

// WHY A CAR IS LISTED ON A PATH.
enum class OnPath : uint8_t {
    Front,    // its front bumper is here
    Tail,     // only the back of its body is here: its front has moved on to a later path
    Beside,   // it is part-way through a lane change into or out of this lane: it is level with this stretch of it
};
constexpr uint8_t kOnPathCount = 3;

// ONE CAR'S PLACE ON ONE PATH: the stretch of the path its body covers.
struct PathEntry {
    CarId car;
    OnPath why = OnPath::Front;
    Real front = 0;   // metres along this path of its front-most point here
    Real rear = 0;    // ... and of its rear-most point here
};
bool operator==(const PathEntry& a, const PathEntry& b);

// A PATH'S CARS, a view into PathLists: entries in order of `front`, then car id.
struct PathList {
    const PathEntry* first = nullptr;
    std::size_t count = 0;
    const PathEntry* begin() const { return first; }
    const PathEntry* end() const { return first + count; }
    std::size_t size() const { return count; }
    bool empty() const { return count == 0; }
    const PathEntry& operator[](std::size_t i) const { return first[i]; }
};

// EVERY PATH'S CARS, kept as one array: path by path, each path's entries in order of `front`, then `rear`, then car
// id, then why. Built whole by rebuildPathLists.
struct PathLists {
    std::vector<PathEntry> entries;
    std::vector<uint32_t> firstOfPath;   // one per path and one more: path i's entries are [firstOfPath[i], [i + 1])
    // Where each car's record is in the records the lists were built from, in car-id order, so an answer can read a
    // car by its id. It follows the order the records were stored in, so it is not part of what the lists say.
    std::vector<uint32_t> recordOfCar;

    // The cars on a path; empty for an unknown one.
    PathList on(engine::PathId path) const;
};
// The same lists (entries and their division into paths; recordOfCar is not compared, see above).
bool operator==(const PathLists& a, const PathLists& b);
inline bool operator!=(const PathLists& a, const PathLists& b) { return !(a == b); }

// ONE CAR'S BODY, laid out along the paths it covers, front path first: on each, the stretch it covers.
struct BodyPiece {
    engine::PathId path;
    Real front = 0;
    Real rear = 0;
};
struct Body {
    std::array<BodyPiece, kMaxTrail + 1> pieces;
    std::size_t count = 0;
    // How much of its length found no path to lie on (its trail ran out first); 0 when it fits.
    Real unplaced = 0;
};
// Lays `length` of body back from the car's front: along its front path, then along its trail in order. A car on an
// unknown path has no pieces and all its length unplaced.
Body layBody(const engine::LaneGraph& roads, const Car& car, Real length);

// BUILDS EVERY PATH'S LIST from the car records, whatever order they are in. Each car is listed on its front path
// (Front), on each trail path its body reaches (Tail), and on its besidePath (Beside) over the same stretch as on its
// front path, clamped to the lane: the lanes of a section run side by side from one start to one end, so a distance
// along one is level with the same distance along the other. A car with an unknown class or path is not listed.
void rebuildPathLists(const engine::LaneGraph& roads, const VehicleClasses& classes, const std::vector<Car>& cars,
                      PathLists& out);

// A car's record by its id, through the lists' recordOfCar (built from the same records); null when not there.
const Car* findCar(const std::vector<Car>& cars, const PathLists& lists, CarId id);

// "SOMEONE NEAR": a car, how far away, and how fast it is going. No car: `car` is not valid.
struct Neighbour {
    CarId car;
    Real gap = 0;
    Real speed = 0;
    bool found() const { return car.valid(); }
};

// THE CAR AHEAD of car `me`: the nearest car any part of whose body lies ahead of my front bumper, along my path and
// then along `pathsAhead` (the paths after mine, in order; the caller's route), no further than `lookAhead` metres.
// `gap` runs from my front to ITS REAR -- which may be on another path than its front, so a car whose front has moved
// on but whose tail is still on my path or one before its front is found by its tail. A car listed Beside in my lane
// counts. I never find myself. A car overlapping my front has a gap below zero. A tie goes to the lower car id.
Neighbour carAhead(const engine::LaneGraph& roads, const std::vector<Car>& cars, const PathLists& lists, CarId me,
                   const std::vector<engine::PathId>& pathsAhead, Real lookAhead);

// THE CAR BEHIND a point of a lane (for a lane change): the nearest car whose front bumper is listed on `lane` (Front
// or Beside) at or behind `along`; `gap` runs from its front to the point. `except` (one's own car) is passed over.
// Only this lane is searched. A tie goes to the lower car id.
Neighbour carBehind(const std::vector<Car>& cars, const PathLists& lists, engine::PathId lane, Real along,
                    CarId except = CarId());

// WHO IS ON A STRETCH of a path: every entry with any part of its body between `from` and `to` (ends included), in
// list order.
std::vector<PathEntry> carsOnStretch(const PathLists& lists, engine::PathId path, Real from, Real to);

// WHICH WAY "TOWARD THE KERB" LIES, as the sign to put on perp() of the direction of travel (perp turns a plan
// direction (x, z) a quarter turn clockwise seen from above, height up): +1 when the kerb is that way, -1 when it is the
// other way. The one place the side traffic keeps is turned into geometry.
Real kerbwardPerpSign(DrivesOn drivesOn);

// WHERE A CAR IS AND WHICH WAY IT POINTS, read off the curve: x, height, z.
struct CarPose {
    engine::Vec3 front;    // the middle of its front bumper, moved sideways by its offset
    engine::Vec3 rear;     // the middle of its rear bumper, likewise (along its trail where its body reaches back)
    engine::Vec3 travel;   // unit: which way its front is going (the curve's direction at its front, with the climb)
    engine::Vec3 body;     // unit: from its rear to its front (through a bend, the chord of its body); `travel` when
                           // the two are in one place
};
CarPose carPose(const engine::LaneGraph& roads, const VehicleClasses& classes, const RuleSet& rules, const Car& car);

}  // namespace driving

#endif
