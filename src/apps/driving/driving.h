#ifndef RAYTRACER_APPS_DRIVING_DRIVING_H
#define RAYTRACER_APPS_DRIVING_DRIVING_H

// THE DRIVING MODEL'S FRONT DOOR (driving-model-design.md 3 and 3.1, issue #234): the one class a host (the city sim,
// the lab, a test) talks to. It is given the road (a lane graph it reads and never owns), steps the traffic one tick at
// a time, takes trips in ("this person drives this car to that place"), lets the host steer a trip (a new destination,
// a hold, an early end) and hands events back after each tick.
//
// For now nothing drives: a trip is recorded, can be asked about, steered and ended, and the tick only advances its
// clock. The three steps of a tick (design 7.1: plan, settle claims, move) are laid out as empty private functions,
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
#include "slot_map.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace driving {

using engine::Real;

// WHO IS AT THE WHEEL: the host's stable number for a person (the city sim's AgentId). The library never reads meaning
// into it; it only hands it back on events (and, later, works out the person's driving habits from a hash of it).
using PersonId = uint64_t;

// WHICH CAR: the host's number for it (the city sim's vehicle index). The library holds no record of the car yet --
// where a car is on the road arrives with the tick (task C6) -- so for now a car is just this number, which is also
// the order events come out in.
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
// start. A trip's destination and a hold are both one of these.
struct PathPoint {
    engine::PathId path;
    Real along = 0;

    bool operator==(const PathPoint& o) const { return path == o.path && along == o.along; }
    bool operator!=(const PathPoint& o) const { return !(*this == o); }
};

// The host's name for a hold (a bus stop's number, a fare's): an opaque number, unique within one trip.
using HoldName = uint64_t;

// "STOP HERE UNTIL RELEASED", placed on a trip by the host (design glossary: a bus waiting for passengers, a cab
// picking up). Nothing acts on it yet.
struct Hold {
    HoldName name = 0;
    PathPoint at;
};

// ONE TRIP: this person is driving this car to that place (design 3.1). The route, mode and remembered facts join it
// in later tasks.
struct Trip {
    CarId car;
    PersonId person = 0;
    PathPoint destination;
    std::vector<Hold> holds;     // sorted by name
    uint64_t beganAtTick = 0;    // the tick count when the host began it
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
    // (The compiled rule set and the vehicle classes are given here too, by the tasks that bring them: design 3.)
    explicit Driving(const engine::LaneGraph* roads = nullptr);

    const engine::LaneGraph* roads() const { return roads_; }

    // ONE STEP of `dt` seconds: plan, settle claims, move (design 7.1). For now it only advances the clock.
    void tick(Real dt);
    uint64_t tickCount() const { return tickCount_; }
    double clockSeconds() const { return clockSeconds_; }

    // BEGIN A TRIP: `person` drives `car` to `destination`. Refused (a null handle) when the car is already on a trip,
    // or the destination is not a place on the road (an unknown path, or a distance off its ends).
    TripId beginTrip(CarId car, PersonId person, const PathPoint& destination);
    // The trip, or null when the handle is null or stale (its trip has ended).
    const Trip* trip(TripId id) const;
    std::size_t tripCount() const { return trips_.size(); }

    // STEER A TRIP (design 3.1: the host steers trips, never cars). Each is refused (false) for a stale handle or a
    // place not on the road.
    // A new destination (a cab given a new fare, a bus's next stop).
    bool changeDestination(TripId id, const PathPoint& destination);
    // Place hold `name` at `at`; a hold of that name already on the trip moves there.
    bool placeHold(TripId id, HoldName name, const PathPoint& at);
    // Release hold `name`; false when the trip has no such hold.
    bool releaseHold(TripId id, HoldName name);
    // End the trip before it arrives. The handle goes stale, and an EndedEarly event is queued.
    bool endTrip(TripId id);

    // THE EVENTS since the last drain, sorted (eventBefore). The list is empty afterwards.
    std::vector<Event> drainEvents();

    // ONE NUMBER OVER EVERYTHING HELD (the clock, every trip, the queued events), folded in a fixed order: the same
    // start and the same calls give the same number at every tick.
    uint64_t stateHash() const;

private:
    // The three steps of a tick, in order (design 7.1). Empty until the tasks that fill them.
    void plan();            // each driver works out what it wants; reads last tick's state, writes nothing shared
    void settleClaims();    // two cars that wanted the same thing: granted in a fixed order
    void move();            // each car moves; events are queued for the host

    bool onRoad(const PathPoint& p) const;

    const engine::LaneGraph* roads_ = nullptr;
    uint64_t tickCount_ = 0;
    double clockSeconds_ = 0;
    engine::SlotMap<Trip, TripTag> trips_;
    std::map<CarId, TripId> tripOfCar_;   // each car's trip, in car order (the hash walks the trips by it)
    std::vector<Event> events_;           // queued since the last drain, in the order they happened
};

}  // namespace driving

#endif
