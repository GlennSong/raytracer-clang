#ifndef RAYTRACER_APPS_DRIVING_DRIVING_H
#define RAYTRACER_APPS_DRIVING_DRIVING_H

// THE DRIVING MODEL'S FRONT DOOR (driving-model-design.md 3 and 3.1, issue #234): the one class a host (the city sim,
// the lab, a test) talks to. It is given the road (a lane graph it reads and never owns), steps the traffic one tick at
// a time, takes trips in ("this person drives this car to that place"), lets the host steer a trip (a new destination,
// a hold, an early end) and hands events back after each tick.
//
// For now nothing drives: cars are placed on the road and removed (traffic_state.h holds a car's record and the
// per-path lists), a trip is recorded for a placed car, can be asked about, steered and ended, and the tick only
// advances its clock. The three steps of a tick (design 7.1: plan, settle claims, move) are laid out as empty private functions,
// so the tasks after this one fill them in place.
//
// The boundary (design 3): this library links engine_core only and includes nothing from the city sim, the renderer,
// the entity world, the physics wrapper or Lua. The arrow goes one way: citysim may include this, never the reverse.
// tests/test_driving.cpp checks the includes of every file in src/apps/driving.
//
// Same result every run: nothing here iterates an unordered container, events come out sorted by car, and
// stateHash() folds everything held in a fixed order.
//
// Units: metres, seconds.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/route.h"
#include "apps/driving/traffic_state.h"   // CarId, PathPoint, Car, the per-path lists
#include "apps/driving/vehicle_class.h"
#include "slot_map.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace driving {

using engine::Real;

// (PersonId, who is at the wheel, is in rule_set.h, beside personalValue which reads a person's habits from it.)

// The host's name for a hold (a bus stop's number, a fare's): an opaque number, unique within one trip.
using HoldName = uint64_t;

// "STOP HERE UNTIL RELEASED", placed on a trip by the host (design glossary: a bus waiting for passengers, a cab
// picking up). Nothing acts on it yet.
struct Hold {
    HoldName name = 0;
    PathPoint at;
};

// ONE TRIP: this person is driving this car to that place (design 3.1), and the way there (route.h). The mode and
// remembered facts join it in later tasks.
struct Trip {
    CarId car;
    PersonId person = 0;
    PathPoint destination;
    std::vector<Hold> holds;     // sorted by name
    uint64_t beganAtTick = 0;    // the tick count when the host began it
    Route route;                 // from where the car was when it was last routed, to the destination
};

// THE HOST'S HANDLE TO A TRIP. Trips are begun and ended, and their slots reused, so the handle carries a generation:
// a handle to an ended trip is refused, never mistaken for whichever trip took its slot.
struct TripTag {};
using TripId = engine::Handle<TripTag>;

// WHAT THE LIBRARY TELLS THE HOST (design 3.1's table), the fixed vocabulary of the events list.
enum class EventKind : uint8_t {
    Parked,        // the car is parked at its destination: the person gets out
    NoRoute,       // there is no way from where the car is to the destination
    CannotPark,    // the car reached its destination and found nowhere to leave it
    MissedTurn,    // the car missed a turn and was routed again (for the driver card; the host need not act)
    Crashed,       // the car hit something
    EndedEarly,    // the host ended the trip before it arrived (endTrip)
};
constexpr uint8_t kEventKindCount = 6;

struct Event {
    EventKind kind = EventKind::Parked;
    CarId car;
    TripId trip;
    PersonId person = 0;
    uint64_t tick = 0;   // the tick count when it happened
};

// The order events come out in: by car, then kind, then trip, then tick -- whole, so the list never depends on the
// order things were stored or done in.
bool eventBefore(const Event& a, const Event& b);

class Driving {
public:
    // Given the road it drives on: a lane graph that must outlive this object (it is read, never owned). Null is an
    // empty world: it ticks, and every trip is refused, since there is nowhere to go.
    // (The compiled rule set is given here too, by the task that brings it: design 3. The vehicle classes come in
    // through setVehicleClasses.)
    explicit Driving(const engine::LaneGraph* roads = nullptr);

    const engine::LaneGraph* roads() const { return roads_; }

    // THE KINDS OF VEHICLE on the road (vehicle_class.h, #248), copied in by the host from its vehicle data. A car is
    // placed as one of them, and its length says how far back its body reaches. Setting them rebuilds the per-path
    // lists (a car whose class index is no longer in the table drops out of them).
    void setVehicleClasses(VehicleClasses classes);
    const VehicleClasses& vehicleClasses() const { return vehicleClasses_; }

    // PLACE A CAR on the road (moving or parked; nobody drives it until a trip begins): `car` of class `vehicleClass`
    // (an index into vehicleClasses()), its front bumper at `front`, and `trail` the earlier paths its body reaches
    // back over, nearest first (each leading into the one before it; paths the body does not reach are dropped).
    // Refused (false, and the reason in `why` when given) when the car is already placed, the class or the path is
    // unknown, the distance is off the path, a trail path is unknown or does not lead on, the trail is longer than
    // kMaxTrail, or the body does not fit on the path and the trail.
    bool placeCar(CarId car, uint32_t vehicleClass, const PathPoint& front,
                  const std::vector<engine::PathId>& trail = {}, std::string* why = nullptr);
    // REMOVE A CAR from the road. Refused (false, with the reason) when it is not placed or is on a trip (end the trip
    // first).
    bool removeCar(CarId car, std::string* why = nullptr);
    // The car's record, or null when it is not placed.
    const Car* car(CarId id) const;
    std::size_t carCount() const { return cars_.size(); }
    // Every car's record, in car-id order, and every path's cars (rebuilt whenever a car is placed or removed), for
    // the questions in traffic_state.h.
    const std::vector<Car>& cars() const { return cars_; }
    const PathLists& pathLists() const { return pathLists_; }

    // ONE STEP of `dt` seconds: plan, settle claims, move (design 7.1). For now it only advances the clock.
    void tick(Real dt);
    uint64_t tickCount() const { return tickCount_; }
    double clockSeconds() const { return clockSeconds_; }

    // BEGIN A TRIP: `person` drives `car` to `destination`, by the route found from where the car's front is now.
    // Refused (a null handle) when the car has not been placed, is already on a trip, or the destination is not a
    // place on the road (an unknown path, or a distance off its ends). Refused too when the destination is on the road
    // but there is no route there for the car's class: then a NoRoute event is queued for the car (with a null trip).
    TripId beginTrip(CarId car, PersonId person, const PathPoint& destination);
    // The trip, or null when the handle is null or stale (its trip has ended).
    const Trip* trip(TripId id) const;
    std::size_t tripCount() const { return trips_.size(); }
    // The trip's route (read-only), or null for a null or stale handle.
    const Route* route(TripId id) const;
    // What binding the vehicle classes to the road had to say (a class tag the road does not know), one line each.
    const std::vector<std::string>& routeNotes() const { return routeNotes_; }

    // STEER A TRIP (design 3.1: the host steers trips, never cars). Each is refused (false) for a stale handle or a
    // place not on the road.
    // A new destination (a cab given a new fare, a bus's next stop), routed from where the car is now. When there is
    // no route there, the trip keeps its old destination and route, a NoRoute event is queued, and the answer is false.
    bool changeDestination(TripId id, const PathPoint& destination);
    // ROUTE AGAIN from where the car is now to the same destination (after a missed turn). When there is no route, the
    // old one is kept, a NoRoute event is queued, and the answer is false.
    bool reroute(TripId id);
    // Place hold `name` at `at`; a hold of that name already on the trip moves there.
    bool placeHold(TripId id, HoldName name, const PathPoint& at);
    // Release hold `name`; false when the trip has no such hold.
    bool releaseHold(TripId id, HoldName name);
    // End the trip before it arrives. The handle goes stale, and an EndedEarly event is queued.
    bool endTrip(TripId id);

    // THE EVENTS since the last drain, sorted (eventBefore). The list is empty afterwards.
    std::vector<Event> drainEvents();

    // ONE NUMBER OVER EVERYTHING HELD (the clock, every car's record in car-id order, every trip, the queued events),
    // folded in a fixed order: the same start and the same calls give the same number at every tick.
    uint64_t stateHash() const;

private:
    // The three steps of a tick, in order (design 7.1). Empty until the tasks that fill them.
    void plan();            // each driver works out what it wants; reads last tick's state, writes nothing shared
    void settleClaims();    // two cars that wanted the same thing: granted in a fixed order
    void move();            // each car moves; events are queued for the host

    bool onRoad(const PathPoint& p) const;
    Car* findPlaced(CarId id);
    // The route for a placed car from its front to `to`, for its class.
    Route routeFor(const Car& c, const PathPoint& to) const;
    void queueEvent(EventKind kind, CarId car, TripId trip, PersonId person);

    const engine::LaneGraph* roads_ = nullptr;
    VehicleClasses vehicleClasses_;
    RouteBounds routeBounds_;                 // studyRoads(*roads_), measured once
    std::vector<RouteClass> routeClasses_;    // each vehicle class bound to the road, by class index
    std::vector<std::string> routeNotes_;
    std::vector<Car> cars_;     // every placed car, in car-id order
    PathLists pathLists_;       // rebuilt from cars_ whenever a car is placed or removed
    uint64_t tickCount_ = 0;
    double clockSeconds_ = 0;
    engine::SlotMap<Trip, TripTag> trips_;
    std::map<CarId, TripId> tripOfCar_;   // each car's trip, in car order (the hash walks the trips by it)
    std::vector<Event> events_;           // queued since the last drain, in the order they happened
};

}  // namespace driving

#endif
