#ifndef RAYTRACER_APPS_DRIVING_DRIVING_H
#define RAYTRACER_APPS_DRIVING_DRIVING_H

// THE DRIVING MODEL'S FRONT DOOR (driving-model-design.md 3 and 3.1, issue #234): the one class a host (the city sim,
// the lab, a test) talks to. It is given the road (a lane graph it reads and never owns), steps the traffic one tick at
// a time, takes trips in ("this person drives this car to that place"), lets the host steer a trip (a new destination,
// a hold, an early end) and hands events back after each tick.
//
// Cars are placed on the road and removed (traffic_state.h holds a car's record and the per-path lists); a trip is
// recorded for a placed car, can be asked about, steered and ended. Once the host has handed over the rules (useRules),
// each tick runs the three steps of design 7.1 over the driven cars (issue #253):
//   1. PLAN: each driver whose turn it is notices its facts along its way ahead, decides by the rules, and HOLDS the
//      answers (a place to stop by, as an odometer reading; a top speed; the way ahead). A driver whose `arrived` fact
//      is yes ends its trip instead: a Parked event, and the car stays where it is, placed and undriven.
//   2. SETTLE CLAIMS: nothing yet (junctions and lane changes bring claims).
//   3. MOVE: every driven car works out its speed from its held answers and a fresh look at the car ahead (act.h),
//      moves along its held way, carrying over onto the next paths as many times as it must, and records what held it
//      back. Every car reads the state as it was at the start of the step and writes a second copy; the copies then
//      swap and every path's list is rebuilt.
// Without the rules the tick only advances its clock.
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
#include "apps/driving/decide.h"
#include "apps/driving/notice.h"
#include "apps/driving/road_rules.h"
#include "apps/driving/route.h"
#include "apps/driving/rule_set.h"
#include "apps/driving/trace.h"           // each driven car's recent decisions, and the driver card
#include "apps/driving/traffic_state.h"   // CarId, PathPoint, Car, Hold, the per-path lists
#include "apps/driving/vehicle_class.h"
#include "apps/driving/wait_graph.h"      // who is waiting for whom
#include "slot_map.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace driving {

using engine::Real;

// (PersonId, who is at the wheel, is in rule_set.h, beside personalValue which reads a person's habits from it. A
// Hold and its HoldName are in traffic_state.h, so the facts can see them.)

// ONE TRIP: this person is driving this car to that place (design 3.1), and the way there (route.h). The mode and
// remembered facts join it in later tasks.
struct Trip {
    CarId car;
    PersonId person = 0;
    PathPoint destination;
    std::vector<Hold> holds;     // sorted by name
    uint64_t beganAtTick = 0;    // the tick count when the host began it
    Route route;                 // from where the car was when it was last routed, to the destination
    // How far along the route the car has got: the first leg its front can be on (route.h's `fromLeg`). Moved on at
    // each decision; back to 0 whenever the trip is routed again.
    uint32_t legReached = 0;
    // THE WAY AHEAD as of the driver's last decision (route.h wayAhead): the paths the car moves along until it decides
    // again, its path at that moment first. The move step never takes the car past the end of the last of them.
    WayAhead way;
    uint32_t wayAt = 0;          // which of `way.paths` the car's front is on now
    // The way stops short of where the driver looked: its last path leads nowhere on the route (a lane that ends, or
    // the destination's). Then its end is a place to come to rest at; otherwise it is only where the driver stopped
    // looking, a limit it never comes near.
    bool wayEnds = false;
    // WHAT THE DRIVER DECIDED RECENTLY (trace.h), and the reasons behind what it holds now. Written only while
    // DrivingSettings::trace is on; never read by the tick, and left out of stateHash (traceHash folds it).
    CarTrace trace;
};

// HOW THE TICK RUNS, set by the host (or a test). These numbers move to the rule set's `detail` table (design 7.2) in
// a later task; until then they live here, each a named default.
struct DrivingSettings {
    // How often a driver decides, in ticks: car `id` decides on the ticks where tick count mod N equals id mod N, so
    // the drivers are spread evenly over the ticks (and every driver decides on its first tick of a trip, whatever its
    // turn). At 60 ticks a second the default is the design's "near" rate, 10 decisions a second. Zero is taken as 1.
    static constexpr uint32_t kDefaultDecideEveryTicks = 6;
    uint32_t decideEveryTicks = kDefaultDecideEveryTicks;
    // Keep each driven car's trace (trace.h). It only records: switching it off changes nothing a car does, and the
    // state hash is the same either way.
    bool trace = true;
};

// WHEN A FIXED CAP CUT SOMETHING SHORT, counted so it is never silent (both are covered by the state hash). Each
// count is per car per time it happened: a tick for a tail, a decision for a look-ahead.
struct CapCounts {
    // A moving car's body lay on more paths behind its front than kMaxTrail: the oldest were dropped from its trail,
    // and its listing on the oldest path kept reaches back past that path's start by the rest of its body, so
    // followers keep at least the true gap (traffic_state.h). The count, and the last car it happened to.
    uint64_t tailsOffTrail = 0;
    CarId lastTailOffTrail;
    // A driver's way ahead stopped at kMaxFollowedPaths paths, short of how far it looks, so it searched no further
    // for the car ahead or anything else. The end of that way is still a wall the car never passes.
    uint64_t lookAheadsCut = 0;
    CarId lastLookAheadCut;
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
    // empty world: it ticks, and every trip is refused, since there is nowhere to go. (The vehicle classes come in
    // through setVehicleClasses, the rules through useRules.)
    explicit Driving(const engine::LaneGraph* roads = nullptr);

    const engine::LaneGraph* roads() const { return roads_; }

    // THE RULES, already loaded and checked by the host (the Lua readers in scripting/, then buildRoadRules with a
    // registry that has the built-in facts and the move step's needs: registerBuiltInFacts, registerMoveNeeds). Copied
    // in; this library loads nothing. Refused (false, the reason in `why`) when the registry has no `arrived` fact or
    // the rule set lacks a number or setting the move step reads (buildRoadRules names those first). Until rules are
    // given, a tick moves nothing.
    bool useRules(RuleSet ruleSet, FactRegistry facts, AnswerVocabulary vocabulary, CompiledRules rules,
                  RoadRules roadRules, std::string* why = nullptr);
    bool hasRules() const { return hasRules_; }
    const RuleSet& ruleSet() const { return ruleSet_; }
    const FactRegistry& facts() const { return facts_; }
    const CompiledRules& rules() const { return rules_; }

    // How often a fixed cap cut something short since this object was made (CapCounts).
    const CapCounts& capCounts() const { return capCounts_; }

    // HOW THE TICK RUNS (how often drivers decide). Takes effect from the next tick.
    void setSettings(const DrivingSettings& settings) { settings_ = settings; }
    const DrivingSettings& settings() const { return settings_; }

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

    // ONE STEP of `dt` seconds: plan, settle claims, move (design 7.1), then the clock moves on.
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
    // End the trip before it arrives. The handle goes stale, and an EndedEarly event is queued. The car stays placed
    // where it is, undriven and at rest (the host has taken it over).
    bool endTrip(TripId id);

    // THE EVENTS since the last drain, sorted (eventBefore). The list is empty afterwards.
    std::vector<Event> drainEvents();

    // ONE NUMBER OVER EVERYTHING HELD (the clock, every car's record in car-id order, every trip, the queued events),
    // folded in a fixed order: the same start and the same calls give the same number at every tick.
    uint64_t stateHash() const;

    // ---- Seeing why (trace.h, wait_graph.h; issue #256). All read-only: none of it changes what a car does. ----
    // The car's trip, or a null handle when it is on none.
    TripId tripOf(CarId car) const;
    // The car's trace, or null when it is on no trip (the ring goes with the trip).
    const CarTrace* trace(CarId car) const;
    // ONE NUMBER OVER EVERY TRACE, in car order (kept apart from stateHash, which leaves the traces out).
    uint64_t traceHash() const;
    // THE DRIVER CARD of a car, as plain text (trace.h describeCar).
    std::string describeCar(CarId car) const { return driving::describeCar(*this, car); }
    // WHO IS WAITING FOR WHOM, from the cars as they stand now (wait_graph.h), and the same as plain text with the
    // rules named.
    WaitReport waitReport() const;
    std::string describeWaiting(const WaitReport& report) const;

private:
    // The three steps of a tick, in order (design 7.1).
    void plan();            // each driver due works out what it wants; reads last tick's state, writes nothing shared
    void settleClaims();    // two cars that wanted the same thing: granted in a fixed order (nothing to settle yet)
    void move(Real dt);     // each driven car moves; then the copies swap and the lists are rebuilt

    bool onRoad(const PathPoint& p) const;
    Car* findPlaced(CarId id);
    // The route for a placed car from its front to `to`, for its class.
    Route routeFor(const Car& c, const PathPoint& to) const;
    void queueEvent(EventKind kind, CarId car, TripId trip, PersonId person);
    // Each record's trip (a null handle for a car not on one), by its place in cars_: walks cars_ and tripOfCar_
    // side by side, both in car-id order.
    void findTripsOfRecords();
    // A car whose trip has ended (arrived, or ended by the host): undriven, at rest where it is, nothing held.
    static void leaveParked(Car& c);
    // What the driver of car record `record` on trip `t` decided this tick, before it is applied.
    struct Planned {
        std::size_t record = 0;
        TripId trip;
        bool arrived = false;     // its `arrived` fact was yes: the trip ends here
        HeldAnswers held;
        WayAhead way;
        bool wayEnds = false;
        uint32_t legReached = 0;
        TracedReason stopWhy;       // the reasons behind `held`, kept only while tracing
        TracedReason topSpeedWhy;
    };

    const engine::LaneGraph* roads_ = nullptr;
    VehicleClasses vehicleClasses_;
    RouteBounds routeBounds_;                 // studyRoads(*roads_), measured once
    std::vector<RouteClass> routeClasses_;    // each vehicle class bound to the road, by class index
    std::vector<std::string> routeNotes_;
    std::vector<Car> cars_;     // every placed car, in car-id order
    PathLists pathLists_;       // rebuilt from cars_ whenever a car is placed or removed, and after every move
    uint64_t tickCount_ = 0;
    double clockSeconds_ = 0;
    engine::SlotMap<Trip, TripTag> trips_;
    std::map<CarId, TripId> tripOfCar_;   // each car's trip, in car order (the hash walks the trips by it)
    std::vector<Event> events_;           // queued since the last drain, in the order they happened
    DrivingSettings settings_;
    CapCounts capCounts_;

    // The rules (useRules).
    bool hasRules_ = false;
    RuleSet ruleSet_;
    FactRegistry facts_;
    AnswerVocabulary vocabulary_;
    CompiledRules rules_;
    RoadRules roadRules_;
    std::vector<FactSlot> wanted_;        // the facts worked out for each driver: the rules' own, and `arrived`
    FactSlot arrivedSlot_ = kNoFact;
    PersonalSetting gapKept_;             // the setting gap_kept, and its name's hash (personalValue)
    uint32_t gapKeptHash_ = 0;
    Real standstillGap_ = 0;

    // Working memory reused from tick to tick (never part of the state).
    std::vector<Car> carsNext_;           // the second copy of the cars the move step writes
    std::vector<TripId> tripOfRecord_;    // findTripsOfRecords
    std::vector<Planned> planned_;
    std::vector<NoticedFact> noticed_;
    NoticeScratch noticeScratch_;
    Decision decision_;
    std::vector<engine::PathId> pathsAfter_;
};

}  // namespace driving

#endif
