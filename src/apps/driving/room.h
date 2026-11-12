#ifndef RAYTRACER_APPS_DRIVING_ROOM_H
#define RAYTRACER_APPS_DRIVING_ROOM_H

// ROOM BEYOND THE JUNCTION: NOBODY ENTERS A JUNCTION IT CANNOT LEAVE (driving-model-design.md 9.5, 9.6 and 7.1's settle
// step; issue #264, docs/driving-briefs/D5-room-beyond.md; ADR-0407).
//
// Words: the BOX is a junction's inside (its connectors); a car's ROOM BEYOND is the stretch of road after the box where
// it can come to rest clear of it; a STRETCH is the lanes from where one junction's box ends to the next place a car
// might have to stop (the next junction's line, or where the road ends); a GRANT is room given to one car, remembered on
// its junction plan (junction_plan.h RoomClaim); a CHAIN is one claim that passes through junctions too close together for
// the car to stop between.
//
// WHY A CLAIM, NOT A LOOK. Two cars from different approaches can both see the same one car's worth of room beyond. So
// room is ASKED FOR (in the plan step, by the driver) and GRANTED (in the settle step, one car at a time, each grant
// reducing the room before the next is looked at). A car not granted room stops at the line, through the rule set (the
// fact `no_room_beyond` and the place `room_line`, which waits for the car whose tail bounds the room: StopFor::NoRoom),
// and announces that it is not going (announce.h), so cross traffic uses the time.
//
// HOW MUCH ROOM THERE IS, on the lane a connector joins (the brief's decisions):
//   - the FAR END is where the back of the queue would come to rest: each car on the stretch, from the front one back,
//     is supposed to brake comfortably to a stop from its speed now (its own class's braking), no further than the car
//     ahead of it less a standstill gap, nor past the stretch's end; the far end is the last one's tail, less a gap. With
//     no car, it is the stretch's end (the next line, the lane's end) or a hold of the asker's own on it, if nearer.
//     (The brief says "the last car brakes": that alone would let a moving last car "rest" beyond a stopped car ahead of
//     it, so each car is held behind the one ahead.)
//   - the NEAR END is where the box ends: the lane's start, or past a crosswalk on it filed under the junction, by the
//     rule set's room_clear_of_crosswalk (a car must not come to rest on the far crosswalk).
//   - the room is far end - near end - the length and gap of every car IN TRANSIT to it: each car whose front is on a
//     connector leading into the lane (granted or not: a car placed there by a level-of-detail change counts), and each
//     car granted room on the lane whose front has not reached it. Derived from where cars are, each tick. And of every
//     car FIRST IN THE QUEUE for it: refused it for want of room before this car first asked, still waiting at its line
//     (through a red too) -- so "who asked first goes first" holds across ticks, not only among cars asking in one.
// HOW MUCH A CAR NEEDS: its length (and trailer) and a standstill gap (the rule set's standstill_gap).
//
// WHEN A DRIVER ASKS (roomToAsk): at the decision where it decides to go at a junction it holds no grant for (its
// judgement, give_way.h, decides "go" only within its decision horizon: one decision before the point where it could no
// longer stop comfortably at the line), or to pull forward and wait inside (D4) when the rule set's room_to_wait_inside
// says waiting inside needs room; and, at rest at its line, when it judged nothing to give way to there but that did not
// count as a decision (released by a red: it is about to move off). Not before. It asks again at each decision while
// refused; there is no timer. Its way ahead is made long enough to measure the claim (roomReach: a car at rest looks no
// further than the junctions it planned, and a claim may pass through the next ones).
//
// GRANTING (RoomLedger::settle), once a tick in the settle step, in one fixed order: cars that first asked earlier
// first; among those that first asked in the same tick, the conflict resolver's order (conflict.h order: who goes first
// where the two meet, the same answer the judgement gives way by), then the lower car id. A car whose lane has a car
// ahead of it short of the same line that holds no grant there is refused (that car is asked first: a car behind it
// holding room it cannot reach would starve it). Each grant is counted against the room before the next is looked at.
// REFUSED, a car waits at the line; one that had rolled no more than at_line_within over it as the refusal came stops
// where it is.
//
// JUNCTIONS CLOSE TOGETHER. When the stretch beyond is shorter than the car needs, it cannot stop there: the claim goes
// on through the next junction and is granted only if (a) there is room beyond THAT junction, (b) nothing rests on the
// way between (each car on the way is held behind the one ahead, so a car resting there leaves no room further on), (c)
// the next junction does not make everyone stop first (a stop sign, an all-way stop), and (d) where it has lights, its
// light for the car's next turn will let it in -- green, or yellow, which a car committed through does not stop for --
// from when the car reckons to get there until room_arrival_slack later, in case it is slower than it reckoned (the lamp
// is a pure function of the clock: signal_programme.h). Up to room_through_junctions junctions (three, the brief); beyond
// that the road has a stretch no vehicle of this length can ever stop in, and the claim is refused as a road-data
// problem. The junctions passed through hold their part of the grant too (RoomClaim::chained), and once the car is
// committed at the first it is committed at them (commitThrough). Waiting inside a junction needs room to stop just
// beyond it: a chain is never granted to wait inside. A turn into a stretch a class can never be granted room through,
// or whose two lights let it through less often than once in room_cycles_through of the longer cycle, is closed to that
// class's routes at load (turnsClosedForRoom) and named in the road-data report (reportRoomRoads).
//
// A GRANT HOLDS until the car's front reaches the lane (it is then counted where it is), or the driver gives it back:
// short of the line, it now holds a stop at or before the line for something else (it decided to give way at the line, a
// red stops it, a hold) -- or the claim at the junction before was given back. A grant lives on the junction plan, so a
// plan dropped (through it, the way changed, the trip routed again or ended) takes its grant with it: grants cannot leak.
//
// POLICY AND MECHANISM: every number (the gap, how many junctions, the crosswalk clearance, whether waiting inside needs
// room, how long a light passed through must let a car in, how seldom is too seldom) is the rule set's, read as a reader
// of it (registerRoomFacts), and the stop is a rule. This file names no kind of junction control, sign or vehicle.
//
// Same result every run: seats come in car-id order, claims are sorted, grants are kept sorted by (lane, car), and the
// cars on a stretch are taken in a fixed order; no unordered container, no pointer order. The steady state allocates
// nothing: every list is working memory kept from tick to tick.
//
// Units: metres, seconds.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/announce.h"        // Mover: how a car moves, for when it reaches the next junction
#include "apps/driving/conflict.h"        // the resolver's order among cars asking in one tick
#include "apps/driving/junction_plan.h"   // JunctionPlans, RoomClaim
#include "apps/driving/notice.h"          // FactRegistry, WayAhead
#include "apps/driving/right_of_way.h"
#include "apps/driving/road_rules.h"
#include "apps/driving/rule_set.h"
#include "apps/driving/traffic_state.h"
#include "apps/driving/vehicle_class.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

struct GiveWayVerdict;   // give_way.h

// ---- What the room step reads from the rule set -------------------------------------------------------------------

constexpr const char* kNumberRoomThroughJunctions = "room_through_junctions";   // ratio: junctions one claim may cover
constexpr const char* kNumberRoomClearOfCrosswalk = "room_clear_of_crosswalk";  // m: room begins this far past a crosswalk
constexpr const char* kNumberRoomToWaitInside = "room_to_wait_inside";          // ratio: 1, waiting inside needs room
constexpr const char* kNumberRoomArrivalSlack = "room_arrival_slack";   // s: a light passed through lets it in this long
constexpr const char* kNumberRoomCyclesThrough = "room_cycles_through";   // ratio: turns lined up less often close
// The name the room step reads the rule set under, in the problems buildRoadRules reports.
constexpr const char* kRoomReaderName = "the room step (room beyond a junction, asked for and granted one car at a time)";

// The facts. Present only for a car refused room at a junction whose line its front has not passed: yes, and where
// that line is (a place to stop by: StopFor::NoRoom, about the car whose tail bounds the room, when one does). A rule set
// says { name = "no room beyond", when = "no_room_beyond", give = { stop_at = "room_line" } }.
constexpr const char* kFactNoRoomBeyond = "no_room_beyond";   // yes/no
constexpr const char* kFactRoomLine = "room_line";            // place

// THE MOST JUNCTIONS ONE CLAIM MAY PASS THROUGH (the rule set's room_through_junctions is held to it): the fixed size of
// the working lists for one claim.
constexpr std::size_t kMaxRoomJunctions = 4;

// THE NUMBERS, read once when the rules are handed over.
struct RoomNumbers {
    Real standstillGap = 0;          // standstill_gap: the gap a car needs beyond its length
    Real atLineWithin = 0;           // at_line_within: "at the line"
    uint32_t throughJunctions = 1;   // room_through_junctions, 1 to kMaxRoomJunctions
    Real clearOfCrosswalk = 0;       // room_clear_of_crosswalk
    bool toWaitInside = true;        // room_to_wait_inside (not zero)
    Real arrivalSlack = 0;           // room_arrival_slack
    Real cyclesThrough = 0;          // room_cycles_through
};

// Adds the facts above and the room step as a reader of its numbers. A host registers it after the junction step
// (give_way.h registerJunctionFacts), which it needs; a host that does not gets no room step (cars enter as before).
// False (with the reasons) when one was registered already.
bool registerRoomFacts(FactRegistry& registry, std::vector<std::string>* problems = nullptr);
// Whether `registry` has the room step.
bool hasRoomStep(const FactRegistry& registry);
// Reads the numbers from `rules`. False (the reason in `why`) when one is missing.
bool readRoomNumbers(const RuleSet& rules, RoomNumbers& out, std::string* why = nullptr);

// A VEHICLE'S BODY, nose to tail with any trailer it pulls.
Real bodyLength(const VehicleClass& cls);
// THE ROOM IT NEEDS beyond a junction: its body and a standstill gap.
Real roomNeeded(const VehicleClass& cls, Real standstillGap);

// ---- The road, read once --------------------------------------------------------------------------------------------

// For every lane: the connectors leading into it, and where its room begins (0, or past a crosswalk on it filed under a
// junction, by room_clear_of_crosswalk). Built by Driving::useRules.
struct RoomRoads {
    std::vector<uint32_t> firstInto;      // per path, and one more: where its connectors begin in `into`
    std::vector<engine::PathId> into;     // the connectors leading into each path, in id order
    std::vector<Real> roomFrom;           // per path

    // The connectors leading into `lane` (empty for an unknown path).
    std::pair<const engine::PathId*, const engine::PathId*> connectorsInto(engine::PathId lane) const;
};
RoomRoads studyRoomRoads(const engine::LaneGraph& roads, Real clearOfCrosswalk);

// THE ROAD-DATA REPORT (the brief's "report the place as a road-data problem"), worked out at load for a person to
// read: every stretch from one junction's box to the next junction's line shorter than some vehicle class needs (those
// vehicles can only pass through the next junction without stopping), whether that junction makes everyone stop first
// (then they can never use it), and, where both junctions have lights, each pair of turns through the two whose lights
// let a car arriving from rest through less often than once in room_cycles_through of the longer cycle (then such a car
// waits at the first line that long, or for ever: the two lights want one programme). One line each.
struct RoomRoadReport {
    uint32_t shortStretches = 0;         // stretches shorter than the longest vehicle class needs
    uint32_t blockedByAStop = 0;         // ... ending at a junction where everyone stops first: some class never fits
    uint32_t turnPairsTooSeldom = 0;     // pairs of turns at two lit junctions that let it through too seldom
    std::vector<std::string> lines;
};
RoomRoadReport reportRoomRoads(const engine::LaneGraph& roads, const RightOfWay& row, const RuleSet& rules,
                               const RoadRules& roadRules, const VehicleClasses& classes, const RoomRoads& layout,
                               const RoomNumbers& numbers, bool lightsRun, Real comfortableSideAccel);

// THE TURNS CLOSED TO A CLASS for want of room, sorted: a connector into a stretch the class cannot stop in where every
// way on through the next junction is barred to it -- the road ends there, everyone stops first there, or (both
// junctions with lights) the two lights let the class through, arriving from rest at the first line on its green, less
// often than once in room_cycles_through of the longer cycle, or never: the two programmes are not coordinated for it.
// A car routed through one would wait at its line that long or for ever,
// so Driving closes them to the class's routes (route.h RouteClass::closed), as a turn too tight for it is closed; the
// report above names them.
std::vector<engine::PathId> turnsClosedForRoom(const engine::LaneGraph& roads, const RightOfWay& row,
                                               const RuleSet& rules, const RoadRules& roadRules, const RoomRoads& layout,
                                               const RoomNumbers& numbers, bool lightsRun, Real comfortableSideAccel,
                                               const VehicleClass& cls);

// ---- Asking (the plan step) -----------------------------------------------------------------------------------------

// WHICH PLAN THE DRIVER OF `me` ASKS ROOM AT this decision, from what it judged (give_way.h): the nearest plan whose line
// its front has not passed (by more than at_line_within), that it holds no grant for and is not committed at -- when
// the driver decided to go there, or to pull forward and wait inside (when `numbers.toWaitInside`). kMaxJunctionPlans
// for none; `toWaitInside` says which it was.
std::size_t roomToAsk(const JunctionPlans& plans, const Car& me, const GiveWayVerdict& verdict,
                      const RoomNumbers& numbers, bool* toWaitInside);
// COMMITTED THROUGH: a car whose body will not fit between two junctions has no place to wait between them, so its last
// waiting point before the second is its last before the first. Granted room through both (the second's grant is
// chained), once it is committed at the first it is committed at the second too, by the same tick: the resolver puts it
// before the second junction's cars that have not committed (first in goes first), and a yellow there, which it could
// only stop for with its tail in the box behind, does not stop it (give_way.h). Driving::announce calls it after noting
// what each car's state says about its lines.
void commitThrough(JunctionPlans& plans);
// HOW FAR THE DRIVER'S WAY MUST REACH (metres from its front) for a claim at the nearest junction it holds no room at to
// be measured: past the stretch beyond it, into the next junction's connector, wherever that stretch is too short for
// `need` (the claim passes through that junction, and only the way says which turn it takes there, and only a plan made
// for that junction can hold its part of the grant), up to the rule set's room_through_junctions. Zero when the way
// `way` reaches far enough already, or there is nothing to ask. A car at rest looks no further than the junctions it
// has planned, so without this a car whose body will not fit beyond its junction could never be granted room
// (Driving::plan lengthens its look-ahead by it).
Real roomReach(const engine::LaneGraph& roads, const RoomRoads& layout, const JunctionPlans& plans, const Car& me,
               const WayAhead& way, Real need, const RoomNumbers& numbers);
// NOTES AN ASK on plan `plan` (the apply phase, after the decision): it is Asked (a plan already granted stays so), and
// the tick of its first ask there is kept.
void askForRoom(JunctionPlans& plans, std::size_t plan, bool toWaitInside, uint64_t tick);

// WHERE A REFUSED DRIVER WAITS: the nearest plan whose room was refused at its last ask, whose line its front has not
// passed (by more than at_line_within) and where it is not committed. What the facts read.
struct RoomStop {
    bool waits = false;
    Real toLine = 0;      // metres from its front to that line
    CarId by;             // the car whose tail bounds the room (none: the lights, a stop, or the road itself)
};
RoomStop roomStopFor(const JunctionPlans& plans, const Car& me, const RoomNumbers& numbers);

// ---- Granting (the settle step) -------------------------------------------------------------------------------------

// ONE DRIVEN CAR as the settle step sees it.
struct RoomSeat {
    const Car* car = nullptr;
    JunctionPlans* plans = nullptr;           // its plans: grants are written here
    const WayAhead* way = nullptr;            // its way ahead, its front's path first
    const std::vector<Hold>* holds = nullptr;
    const VehicleClass* cls = nullptr;
};

// ONE GRANT outstanding: counted against `lane` until the car's front reaches it.
struct RoomGrant {
    engine::PathId lane;
    uint32_t car = 0;
    double laneAt = 0;     // the odometer reading where its front reaches the lane
    double odometer = 0;   // its odometer now
    Real need = 0;
};

// WHAT THE SETTLE STEP READS.
struct RoomWorld {
    const engine::LaneGraph& roads;
    const RightOfWay& row;
    const RuleSet& rules;
    const RoadRules& roadRules;
    const VehicleClasses& classes;
    const std::vector<Car>& cars;
    const PathLists& lists;
    const RoomRoads& layout;
    const RoomNumbers& numbers;
    const ConflictResolver& resolver;   // this tick's order: who goes first where two cars meet
    bool lightsRun = false;
    Real comfortableSideAccel = 0;      // the speed a car holds through a junction (announce.h speedThrough)
    uint64_t tick = 0;
    double now = 0;
};

// HOW THE ROOM WENT, since the ledger was made.
struct RoomCounts {
    uint64_t asked = 0;                 // asks settled
    uint64_t granted = 0;               // ... granted
    uint64_t grantedThrough = 0;        // ... of them through one junction or more (a chain)
    uint64_t grantedToWaitInside = 0;   // ... of them to wait inside
    std::array<uint64_t, kRoomRefusalCount> refused{};   // ... refused, by why
    uint64_t usedOnLane = 0;            // grants held until the car's front reached the lane
    uint64_t givenBack = 0;             // grants given back before the car entered (gave way at the line, a red, ...)
    uint64_t orderedByResolver = 0;     // pairs of asks in one tick put in order by the conflict resolver
    uint32_t mostHeld = 0;              // the most grants outstanding at once
    std::vector<std::pair<uint32_t, uint32_t>> roadDataPlaces;   // (junction, lane) refused TooShort or MustStopAhead
};

// HOW MUCH ROOM ONE CLAIM FINDS (measureRoom).
struct RoomAnswer {
    bool granted = false;
    RoomRefusal refusal = RoomRefusal::None;
    CarId by;                            // the car whose tail bounds the room, or a car in transit that took it
    engine::JunctionId at;               // the junction a refusal is about (lights, a stop, too short)
    Real need = 0;
    Real room = 0;                       // the room found where the car would rest (below `need` when refused)
    uint8_t junctions = 0;               // how many junctions the claim covers (1: its own)
    std::array<engine::PathId, kMaxRoomJunctions> lanes{};   // the lane beyond each junction covered
    std::array<double, kMaxRoomJunctions> laneAt{};          // ... the odometer reading where the front reaches it
    std::array<uint8_t, kMaxRoomJunctions> plans{};          // ... the plan of each junction covered
};

// THE LEDGER: grants every tick's asks, in order, and keeps the outstanding grants.
class RoomLedger {
public:
    // The settle step. `seats` are every driven car on a trip, in car-id order. First the grants held are kept or let
    // go (front on the lane, given back); then every ask of this tick is granted or refused, in the order above.
    void settle(const RoomWorld& world, std::vector<RoomSeat>& seats);
    // HOW MUCH ROOM car `seat` would find for plan `plan` now, with `grants` outstanding (sorted by lane, then car).
    // Pure: changes nothing. (Public so the tests can ask it.)
    RoomAnswer measure(const RoomWorld& world, const RoomSeat& seat, std::size_t plan,
                       const std::vector<RoomGrant>& grants);

    const RoomCounts& counts() const { return counts_; }
    // The grants outstanding after the last settle, sorted by (lane, car).
    const std::vector<RoomGrant>& grants() const { return grants_; }

private:
    struct Claim {
        uint32_t seat = 0;
        uint8_t plan = 0;
        uint64_t askedTick = 0;
        uint32_t car = 0;
    };
    struct OnWay {          // one car with part of its body on the way a claim measures
        uint32_t car = 0;
        Real rear = 0;      // its rear-most point on the way, metres from the first lane's start
        Real front = 0;     // its front, when it is on the way
        bool frontOn = false;
    };
    struct Stretch {        // one stretch of the way, metres from the first lane's start
        Real start = 0, end = 0;
        Real roomFrom = 0;              // where room begins on it, from its start
        engine::PathId firstLane;
        engine::PathId intoNext;        // the connector it leads into on the car's way (none: the way stops)
    };
    struct Seg {            // one path of the way, and where it begins
        engine::PathId path;
        Real from = 0;
    };
    struct Waiting {        // a car refused room on `lane` for want of it, waiting at its line: first in the queue for it
        engine::PathId lane;
        uint32_t car = 0;
        uint64_t askedTick = 0;
        Real need = 0;
    };

    void keepOrLetGo(const RoomWorld& world, RoomSeat& seat);
    void orderSameTick(const RoomWorld& world, std::size_t first, std::size_t end);
    // The car ahead of the claimant in its lane, short of the plan's line, that holds no grant there (none: nobody).
    CarId aheadWithoutRoom(const RoomWorld& world, const std::vector<RoomSeat>& seats, const RoomSeat& seat,
                           const JunctionPlan& plan);
    void addGrant(const RoomGrant& g);

    RoomCounts counts_;
    std::vector<RoomGrant> grants_;
    std::vector<Waiting> waiting_;   // sorted by (lane, asked tick, car)
    std::vector<Claim> claims_;
    std::vector<Claim> ordered_;
    std::vector<uint8_t> placed_;
    std::vector<Seg> segs_;
    std::vector<Stretch> stretches_;
    std::vector<OnWay> onWay_;
    std::vector<uint32_t> transit_;
    std::vector<engine::PathId> pathsAfter_;
};

}  // namespace driving

#endif
