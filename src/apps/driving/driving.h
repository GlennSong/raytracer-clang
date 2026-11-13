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
//   1. PLAN: first, when the host registered the junction step (give_way.h registerJunctionFacts), every driven car
//      notes what its own state says about the lines ahead (came to rest, committed, arrived) and ANNOUNCES itself on
//      the board for the crossing points of the junctions within its look-ahead time (announce.h), from the state at
//      the start of the tick; the board is then sorted. Then each driver whose turn it is notices its facts along its
//      way ahead, decides by the rules, and HOLDS the answers (a place to stop by, as an odometer reading; a top speed;
//      the way ahead). Before noticing, it brings its plans for the junctions on its way ahead up to date
//      (junction_plan.h) and judges, from this tick's board, whether it must give way at them (give_way.h). A driver
//      whose `arrived` fact is yes ends its trip instead: a Parked event, and the car stays where it is, placed and
//      undriven. Every decision reads the cars as they stood at the start of the tick and this tick's board; the
//      answers are applied only once all are made, so the order cars are visited in changes nothing.
//   2. SETTLE CLAIMS: with the room step (room.h registerRoomFacts), the room beyond each junction that drivers asked
//      for this tick is granted or refused, one car at a time in one fixed order (room.h RoomLedger), and grants held
//      are kept or let go. Lane changes will bring claims of their own (task E1).
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
#include "apps/driving/announce.h"        // the board: who will be on which crossing point, and when
#include "apps/driving/conflict.h"        // who goes first where two cars' paths meet: the one resolver
#include "apps/driving/decide.h"
#include "apps/driving/give_way.h"        // judging the gaps a driver gives way by
#include "apps/driving/junction_plan.h"   // each driver's plan for the junctions ahead
#include "apps/driving/notice.h"
#include "apps/driving/right_of_way.h"    // who gives way to whom, worked out at load
#include "apps/driving/room.h"            // room beyond a junction, granted one car at a time
#include "apps/driving/signal_programme.h"   // the traffic lights' clock: a pure function of the time
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

class Measurements;   // measure.h: the measurements table, which watches the tick

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
    // ANOTHER WAY (issue #284): the tick its driver last asked for another way, refused room at a junction for longer
    // than the rule set's room_refused_another_way (0: never). It asks again no sooner than that long after.
    uint64_t askedAnotherWayTick = 0;
    // THE WAY AHEAD as of the driver's last decision (route.h wayAhead): the paths the car moves along until it decides
    // again, its path at that moment first. The move step never takes the car past the end of the last of them.
    WayAhead way;
    uint32_t wayAt = 0;          // which of `way.paths` the car's front is on now
    // The way stops short of where the driver looked: its last path leads nowhere on the route (a lane that ends, or
    // the destination's). Then its end is a place to come to rest at; otherwise it is only where the driver stopped
    // looking, a limit it never comes near.
    bool wayEnds = false;
    // THE JUNCTIONS AHEAD, as the driver plans for them (junction_plan.h): made when a junction's connector comes onto
    // the way ahead, kept until the car is through, dropped at once when the trip is routed again. They hold what the
    // car remembers of each line and what its driver decided there (issue #262), which what it does reads, so stateHash
    // folds them; planHash folds them too, with the plan counts.
    JunctionPlans junctions;
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
    // Routes may change lanes (RouteClass::laneChanges). The tick cannot change lanes yet (task E1), so a car given a
    // route that needs one stops at the end of its lane for good; a host whose cars must finish their trips turns this
    // off until lane changes exist. Takes effect for routes found from then on.
    bool laneChanges = true;
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
    // A driver had more junctions in view than kMaxJunctionPlans: the farthest wait to be planned for until a nearer one
    // is cleared. Counted per decision. (Like the plans themselves, these two are not in stateHash; planHash folds them.)
    uint64_t junctionPlansCut = 0;
    CarId lastJunctionPlanCut;
    // A new plan's connector had more places to wait than kMaxWaitingPoints: the plan keeps the first ones, so its car
    // commits later than it might. Counted per plan made.
    uint64_t waitingPointsCut = 0;
    CarId lastWaitingPointCut;
};

// HOW THE DRIVERS' JUNCTION PLANS WENT since this object was made (junction_plan.h).
struct JunctionPlanCounts {
    uint64_t made = 0;
    uint64_t cleared = 0;        // dropped: the car was through
    uint64_t wayChanged = 0;     // dropped: its way no longer went through the connector
    uint64_t routeChanged = 0;   // dropped: the trip was routed again
    uint32_t mostCrossings = 0;  // the most crossing points along one plan's connector
    engine::PathId mostCrossingsConnector;
    uint32_t mostAtOnce = 0;     // the most junctions one driver had planned for at once
    // WHAT DRIVERS DECIDED at junctions (issue #262), counted per change of decision on a plan.
    uint64_t decidedGo = 0;          // decided to go (from undecided, or after giving way)
    uint64_t decidedGiveWay = 0;     // decided to give way
    uint64_t wentAfterGivingWay = 0; // ... of the go decisions, after giving way on the same approach (a gap taken)
    uint64_t changedMindTwice = 0;   // plans whose driver switched between go and give way more than once
    // ... not counting waits INSIDE the junction (issue #263): a driver past its line, or too fast to stop at it, that
    // stopped inside for a car the resolver put first. Counted here, each stop once.
    uint64_t waitedInside = 0;
    // ... nor switches made while held at the line for room beyond (issue #264): it cannot go whatever it judges, and
    // announces that it is not going.
    uint64_t switchedWhileHeldForRoom = 0;
    // Decisions to stop at a crossing point on a LANE, giving way to a car the resolver put first there (issue #284).
    uint64_t gaveWayOnALane = 0;
    // ANOTHER WAY (issue #284): drivers refused room at a junction for longer than room_refused_another_way that asked
    // the router for a way avoiding that turn, and of them, those given one (the rest keep their way and wait on).
    uint64_t askedAnotherWay = 0;
    uint64_t tookAnotherWay = 0;
    // The gaps taken after giving way: the smallest and largest gap had, and the smallest gap needed (seconds).
    Real smallestGapTaken = 0, largestGapTaken = 0, smallestGapNeeded = 0;
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
    // given, a tick moves nothing. Taking them, it works out who gives way to whom at every junction (rightOfWay()), and
    // every driver plans its junctions afresh at its next decision.
    bool useRules(RuleSet ruleSet, FactRegistry facts, AnswerVocabulary vocabulary, CompiledRules rules,
                  RoadRules roadRules, std::string* why = nullptr);
    bool hasRules() const { return hasRules_; }
    const RuleSet& ruleSet() const { return ruleSet_; }
    const FactRegistry& facts() const { return facts_; }
    const CompiledRules& rules() const { return rules_; }

    // How often a fixed cap cut something short since this object was made (CapCounts).
    const CapCounts& capCounts() const { return capCounts_; }

    // ---- Junctions (issue #260) ----
    // WHO GIVES WAY TO WHOM at every junction, worked out by useRules from the road, the rule set and the binding.
    const RightOfWay& rightOfWay() const { return rightOfWay_; }
    // One junction's table as text (right_of_way.h describeJunctionRules).
    std::string describeJunction(engine::JunctionId junction) const;
    // The car's plans for the junctions ahead, or null when it is on no trip.
    const JunctionPlans* junctionPlans(CarId car) const;
    const JunctionPlanCounts& junctionPlanCounts() const { return planCounts_; }
    // THE TRAFFIC LIGHTS (issue #261): whether they run a programme (the host registered the light clock,
    // signal_programme.h), its numbers, and what a connector's lamp shows now -- the same pure function of the clock the
    // drivers read, for anything that draws or reports the lights (Dark where none runs).
    bool lightsRun() const { return lightsRun_; }
    const LightTiming& lightTiming() const { return lightTiming_; }
    // THE SAFETY CHECK ON THE PROGRAMMES, made by useRules once they are timed (signal_programme.h
    // checkSignalProgrammes; issue #284): each stage where two movements whose paths meet would let traffic in together
    // without the rule set allowing it, in plain words. Empty when sound, or when no light runs. Also among the right-of-way
    // table's notes (describeRightOfWay prints them), so a host that reports the table reports these.
    const std::vector<std::string>& signalProblems() const { return signalProblems_; }
    Lamp lamp(engine::PathId connector) const {
        return lightsRun_ && roads_ ? lampAt(*roads_, rightOfWay_, connector, clockSeconds_) : Lamp::Dark;
    }
    // THE JUNCTION STEP (issue #262): on when the rules came with a registry that has it (registerJunctionFacts). This
    // tick's board of announcements (announce.h), as it was when the drivers decided.
    bool junctionStep() const { return junctionsOn_; }
    const JunctionNumbers& junctionNumbers() const { return junctionNumbers_; }
    const AnnouncementBoard& board() const { return board_; }
    // WHO GOES FIRST (issue #263): this tick's conflicts, settled by the one conflict resolver (conflict.h) from the
    // board, and its counts summed over every tick so far.
    const ConflictResolver& resolver() const { return resolver_; }
    const ConflictCounts& conflictCounts() const { return conflictTotals_; }
    // ROOM BEYOND (issue #264): on when the rules came with a registry that has the room step (room.h registerRoomFacts,
    // after the junction step); its numbers; how the room went (asks, grants, refusals); the grants outstanding after the
    // last tick; and the road-data report: stretches some vehicle classes cannot stop in, and where they can never pass
    // (worked out when asked, from the road, the rules and the vehicle classes).
    bool roomStep() const { return roomOn_; }
    const RoomNumbers& roomNumbers() const { return roomNumbers_; }
    const RoomCounts& roomCounts() const { return room_.counts(); }
    const std::vector<RoomGrant>& roomGrants() const { return room_.grants(); }
    RoomRoadReport roomRoadReport() const;
    // ONE NUMBER OVER EVERY DRIVER'S JUNCTION PLANS (in car order) and the plan counts and caps, kept apart from
    // stateHash, which leaves them out while nothing a car does reads them.
    uint64_t planHash() const;

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
    // THE CLASS AS ROUTES FOR IT ARE FOUND (route.h): bound to the road, lane changes as the settings say, and with the
    // room step the turns it could never (or too seldom) be granted room through closed to it (room.h
    // turnsClosedForRoom). A host that
    // finds places to send cars uses the same, so it never picks a place only a closed turn reaches.
    RouteClass routeClass(uint32_t vehicleClass) const;
    // THE CLASS'S VEHICLE TAGS on the road (route.h RouteClass::tags): which crossing points concern its cars
    // (CrossingPoint::concerns, ADR-0412). Every record concerns a class the road has not bound.
    engine::TagMask tagsOf(uint32_t vehicleClass) const {
        return vehicleClass < routeClasses_.size() ? routeClasses_[vehicleClass].tags : engine::kEveryVehicle;
    }

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
    // AT THE END OF A ROAD THAT JUST ENDS (a lane with no path after it, such as the open edge of a map): the car is
    // on a trip, at rest, with its front no further than kArrivedWithin from the end of a path that leads nowhere.
    // Nothing in the model will ever move it on; what happens next is the host's choice. False for a car not placed.
    bool atRoadEnd(CarId car) const;

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
    // WHETHER THE REASON A WAITING CAR NAMES STILL APPLIES to it now (issue #284: before, a car at rest always named
    // something, so "waiting for no reason" could hardly be found): the car it is behind is on the road and on its way
    // ahead; the car it gives way to is announced on a crossing point ahead of it; the car whose tail bounds its room is
    // the one its refused claim names; the place it stops by is still held and not behind it; a plain stop at a line
    // whose lights run is not one whose lamp has shown green since before the driver's last decision; the end of its
    // way is the end. False (with what failed, when `why` is given) is waiting for no reason: waitReport lists the car so.
    bool waitReasonHolds(const Car& c, std::string* why = nullptr) const;
    // A WAIT THAT CAN NEVER END (issue #284): the car stands refused room beyond the nearest junction it waits at for a
    // reason that cannot change -- its body fits nowhere within room_through_junctions, everyone stops first at the next
    // junction, or the turns its claim passes through are a way on the lights never let it through (route.h
    // ClosedChains::never), or a turn closed to its class whole. The router avoids all of these, so a car found here was
    // put down or routed past them some other way (a host's placement, a stale route); the measurements report it.
    bool waitCanNeverEnd(const Car& c, std::string* why = nullptr) const;
    std::string describeWaiting(const WaitReport& report) const;
    // The graph's regulation kinds bound to the rule set's devices (useRules), for code that reads facts the way a
    // driver does (measure.h).
    const RoadRules& roadRules() const { return roadRules_; }

    // ---- The measurements table (measure.h; issue #257). It only watches: nothing it holds is read by the tick, and
    // the state hash is the same with it attached or not. ----
    // Attach `m` (it must outlive the attachment; null detaches). After each tick it is shown the world read-only. When
    // it has not yet begun, it first remembers where every car is now.
    void attachMeasurements(Measurements* m);
    Measurements* measurements() const { return measurements_; }

private:
    // The three steps of a tick, in order (design 7.1).
    void plan();            // each driver due works out what it wants; reads last tick's state, writes nothing shared
    // The start of the plan step: every driven car notes the lines ahead and announces itself (announce.h).
    void announce();
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
        JunctionPlans junctions;    // the plans for the junctions on `way`, updated before the driver noticed
        PlanUpdate planUpdate;      // what updating them did
        GiveWayVerdict verdict;     // what it judged about giving way at them (the junction step only)
        std::array<Real, kMaxJunctionPlans> lineIn{};   // ... and when the board had it reaching each one's line
        // ROOM BEYOND (the room step only): the plan it asks room at (kMaxJunctionPlans: none), whether to wait inside,
        // and where it waits if refused (what its facts read).
        std::size_t roomAsk = kMaxJunctionPlans;
        bool roomAskInside = false;
        RoomStop roomStop;
    };
    // Counts and traces what one update of a car's plans did.
    void notePlanUpdate(const PlanUpdate& u, CarId car, Trip& t, const JunctionPlans& plans);
    // Records on the trip's plans what the driver decided (give_way.h), counting and tracing each change, and fixes the
    // arrival at each line it judged for the first time (`lineIn`, by plan).
    void applyVerdict(const GiveWayVerdict& v, const std::array<Real, kMaxJunctionPlans>& lineIn, const Car& car,
                      Trip& t);

    // ONE CAR'S NEAREST LINE, as the announcing step works it out (working memory, rebuilt every tick).
    struct LineWork {
        bool has = false;
        uint8_t plan = 0;             // which of its plans
        double toLine = 0;            // metres from its front
        Real ownIn = 0;               // seconds to reach it, by its own estimate
        Real in = 0;                  // ... and no earlier than the car ahead queued for the same line
        bool notGoing = false;        // stopping short of the line for another reason, or queued behind such a car
        bool stopsShort = false;      // it holds a stop at or short of the line (not a full stop first there)
        bool neverClear = false;      // queued for the line behind a car that stops short of it (or behind such a car)
        double frontLimit = kNever;   // the odometer reading its front cannot pass (a stopped car beyond the line)
        engine::PathId fromLane;      // the lane the connector leaves: cars queued for one line share it
        Real through = 0;             // the speed it will hold through that junction
    };
    // When the board has a car reaching the line of each of its plans, by connector (kNever: not announced).
    using LineTimes = std::array<std::pair<engine::PathId, Real>, kMaxJunctionPlans>;
    // The trip was routed again: its plans go at once (each noted).
    void dropPlansForNewRoute(Trip& t);
    // ANOTHER WAY (issue #284; design 9.6 point 2): a driver refused room at the nearest junction it waits at, having asked
    // there for longer than room_refused_another_way (and not asked for another way that long), asks the router for a
    // way from where it is that does not take that turn. Found, the trip takes it (its plans dropped, as any new route);
    // not found, it keeps its way and waits on. Never a push through: the car only ever changes where it means to go.
    void askAnotherWay(Car& c, Trip& t);
    // Closes to each class's routes the turns it could never (or too seldom) be granted room through (room.h
    // turnsClosedForRoom; none without the room step). Run when the rules or the vehicle classes are handed over.
    void closeTurnsForRoom();

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
    Measurements* measurements_ = nullptr;   // watches after each tick; never read by the tick, not in the hash

    // The rules (useRules).
    bool hasRules_ = false;
    RuleSet ruleSet_;
    FactRegistry facts_;
    AnswerVocabulary vocabulary_;
    CompiledRules rules_;
    RoadRules roadRules_;
    RightOfWay rightOfWay_;               // built from the road and the rules by useRules
    JunctionPlanCounts planCounts_;
    std::vector<FactSlot> wanted_;       // the facts worked out for each driver: the rules' own, and `arrived`
    FactSlot arrivedSlot_ = kNoFact;
    PersonalSetting gapKept_;             // the setting gap_kept, and its name's hash (personalValue)
    uint32_t gapKeptHash_ = 0;
    Real standstillGap_ = 0;
    bool junctionsOn_ = false;            // the junction step (give_way.h hasJunctionStep)
    bool lightsRun_ = false;              // the lights run a programme (road_rules.h RoadRules::lightsRun)
    LightTiming lightTiming_;             // ... by these numbers (signal_programme.h)
    std::vector<std::string> signalProblems_;   // ... and what checkSignalProgrammes found at load
    Real step_ = 0;                       // the step of the tick under way, seconds (the decide interval is a number of them)
    JunctionNumbers junctionNumbers_;
    AnnouncementBoard board_;             // rewritten at the start of every plan step
    ConflictResolver resolver_;           // ... and its conflicts settled from it, once a tick
    std::vector<std::pair<uint64_t, uint64_t>> behind_;   // ... with each car held behind the car ahead (working memory)
    ConflictCounts conflictTotals_;       // the resolver's counts over every tick (not read by the tick)
    bool roomOn_ = false;                 // the room step (room.h hasRoomStep)
    RoomNumbers roomNumbers_;             // ... its numbers
    RoomRoads roomRoads_;                 // ... the road as it reads it (built by useRules)
    RoomLedger room_;                     // ... the grants, settled once a tick
    std::vector<RoomSeat> roomSeats_;     // ... every driven car as the settle step sees it (working memory)

    // Working memory reused from tick to tick (never part of the state).
    std::vector<Car> carsNext_;           // the second copy of the cars the move step writes
    std::vector<TripId> tripOfRecord_;    // findTripsOfRecords
    std::vector<Planned> planned_;
    std::vector<NoticedFact> noticed_;
    NoticeScratch noticeScratch_;
    Decision decision_;
    std::vector<engine::PathId> pathsAfter_;
    std::vector<LineWork> lines_;         // the announcing step's, per record
    std::vector<uint32_t> lineOrder_;     // ... the records with a line, nearest the line first
    std::vector<LineTimes> lineTimes_;    // ... per record
};

}  // namespace driving

#endif
