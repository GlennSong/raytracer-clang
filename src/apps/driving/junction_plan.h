#ifndef RAYTRACER_APPS_DRIVING_JUNCTION_PLAN_H
#define RAYTRACER_APPS_DRIVING_JUNCTION_PLAN_H

// A DRIVER'S PLAN FOR THE JUNCTIONS AHEAD (driving-model-design.md 9.1-9.4; issue #260, task D1).
//
// When a junction comes within a driver's look-ahead (its way ahead runs through one of the junction's connectors), the
// driver makes a PLAN for it and keeps it until it is through: which junction, which connector it will take, the kind
// of turn (as the connector states it), where along the road the connector begins (as an odometer reading, so it stays
// put while the car moves), and the places on the connector where its body can wait. The plan is not rebuilt each tick
// or each decision: at each decision the driver only checks that each plan's connector is still on its way (or under its
// body), drops the ones that are not, and makes plans for junctions newly in view. The crossing points along the
// connector, and who gives way at each, are not copied: they are read from the lane graph (Path::crossings, in order)
// and the right-of-way table (right_of_way.h) when someone asks.
//
// WAITING POINTS (design 9.4): the places along a connector where a car's front may stop without any part of its body
// standing in another path's crossing stretch. The first is always the line (the connector's start). After it, for
// each clear gap between two crossing stretches (RightOfWay's gaps) at least as long as the car, the end of that gap:
// the front stopped short of the next stretch with the tail clear of the one before. The last of them is the car's LAST
// WAITING POINT. The car is COMMITTED (task D4 orders committed cars by when) once it can no longer wait there or short
// of the crossing stretch that follows it: its front is on that stretch (JunctionPlan::commitAt), or it cannot stop short
// of it braking as hard as it can (issue #284: before, a car was committed a centimetre past the line, since the line is
// the last waiting point of most connectors, and a car that rolled a few ticks over its line outranked the main road).
// A long vehicle has fewer waiting points than a short one at the same junction, which is the point: it cannot wait
// where it does not fit. At lights (issue #261) a far turn may also wait at its connector's place to wait inside
// (waitInsideAt below), where its body stands across the cross street's paths, which have a red: that is a waiting point
// only while its own light is green or yellow. Once its light is red, a car past its line is committed whatever its
// waiting points say: it entered lawfully and the next green waits for it to clear (Driving::announce, noteAtTheLine).
//
// A plan is DROPPED when the car is through (its body has left the connector: CLEARED), when the way ahead no longer
// runs through its connector (WAY CHANGED: the driver chose another lane or turn), or at once when the trip is routed
// again (ROUTE CHANGED: changeDestination, reroute). The trace keeps a note of each plan made and dropped.
//
// Cost: everything here is fixed arrays; updating allocates nothing. A driver plans for at most kMaxJunctionPlans
// junctions at once and keeps at most kMaxWaitingPoints waiting points per plan; more is counted, never silent.
//
// Same result every run: plans follow the way ahead in order, and nothing here walks an unordered container.
//
// Units: metres, seconds.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/notice.h"          // WayAhead
#include "apps/driving/conflict.h"            // Question
#include "apps/driving/right_of_way.h"
#include "apps/driving/traffic_state.h"   // Car

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace driving {

using engine::Real;

// How many junctions ahead a driver plans for at once. A driver looks ahead its junction look-ahead time (10 s in the
// lab): at 14 m/s that is 140 m, and where the lab's lights stand 25 m apart (junctions 14, 11, 110 and 8: connectors of
// 20 m and links of 6 m) five junctions were in view with the one under the car, which cut 35 plans in five minutes once
// the lights' green wave let cars run that fast (issue #264). Six leaves room. A junction past the last is planned for
// once a nearer one is cleared (counted: CapCounts::junctionPlansCut).
constexpr std::size_t kMaxJunctionPlans = 6;
// How many waiting points a plan keeps: the line and up to three places inside the junction. A connector with more gaps
// a car fits in keeps the first ones (counted: CapCounts::waitingPointsCut); the last kept is then its last waiting
// point, which errs toward committing late.
constexpr std::size_t kMaxWaitingPoints = 4;

// WHAT A DRIVER HAS DECIDED about one junction (issue #262, give_way.h): nothing yet (it is not yet near enough, or it
// has not decided since the plan was made), to GO through, or to GIVE WAY at the line.
enum class GiveWayDecision : uint8_t { Undecided, Go, GiveWay };
constexpr uint8_t kGiveWayDecisionCount = 3;
const char* giveWayDecisionWords(GiveWayDecision d);   // "undecided", "go", "give way"

// WHY A DRIVER GIVES WAY at a junction's line, or what it judged when it decided to go.
enum class GiveWayCause : uint8_t {
    None,           // nothing to give way to: no meeting it must judge has anyone announced on it
    Gap,            // a gap judged against a car's announcement (the car, the crossing, the gap needed and had)
    NextJunction,   // it gives way at the next junction, and there is no room for its body between the two
    // (A car from its own lane standing inside the junction on a split's stretch is not given way to here: since #283 it
    // is the car ahead, followed, traffic_state.h carAhead. Issue #284 took out the special case that did it here.)
};
constexpr uint8_t kGiveWayCauseCount = 3;

// THE JUDGEMENT BEHIND A DECISION: to whom, where, and the gaps. For Go, the tightest gap the driver accepted (none
// when nobody was there to judge); for GiveWay, the first meeting where the gap fell short.
struct GiveWayJudgement {
    GiveWayCause cause = GiveWayCause::None;
    CarId other;                       // Gap: the car judged
    engine::CrossingId crossing;       // Gap: at which crossing point
    engine::PathId otherPath;          // Gap: the path that car is on there
    Real need = 0;                     // Gap: the gap needed, seconds (shrunk by patience, or the hard one once going)
    Real have = 0;                     // Gap: the gap there is, seconds (below `need` when giving way)
    Question by = Question::None;      // Gap: why that car goes first (the conflict resolver's question, conflict.h)
    // GiveWay: where along the connector the car stops, metres: 0 is the line; more is inside the junction, short of the
    // crossing point (task D4: a car past its line, or that can no longer stop at it, waits inside).
    Real stopAlong = 0;
};
bool operator==(const GiveWayJudgement& a, const GiveWayJudgement& b);

// ROOM BEYOND A JUNCTION (issue #264, task D5; room.h does the work): whether the driver has asked for, been granted or
// been refused room on the lane its connector joins.
enum class RoomState : uint8_t {
    None,      // not asked (or given back, or used: its front reached the lane)
    Asked,     // asked at this tick's decision; settled in this tick's settle step
    Granted,   // granted: it may enter; counted against the lane's room until its front is on the lane
    Refused,   // refused at its last ask: it waits at the line and asks again at its next decision
};
constexpr uint8_t kRoomStateCount = 4;
const char* roomStateWords(RoomState s);   // "not asked", "asked", "granted", "refused"

// WHY ROOM WAS REFUSED.
enum class RoomRefusal : uint8_t {
    None,
    NoRoom,          // the room beyond is shorter than the car needs (behind a car's tail, or taken by cars on the way in)
    CarAheadFirst,   // a car ahead of it in its lane, short of the same line, has not been granted room yet
    LightsAhead,     // its body will not fit before the next junction, whose light for its turn will not let it in (green
                     // or yellow) from when it gets there until the rule set's room_arrival_slack later
    MustStopAhead,   // its body will not fit before the next junction, where it must stop (a stop sign, an all-way stop)
    TooShort,        // its body will not fit before any of the next room_through_junctions junctions' lines: the road
                     // has a stretch no vehicle of this length can ever stop in (a road-data problem)
    NotPlanned,      // its way or its plans do not reach far enough to say where it would stop
    NoStopToWait,    // it asked to wait inside, but beyond the junction there is no stretch it could stop in (it would
                     // have to pass through the next junction too, which waiting inside cannot promise)
    GivesWayAhead,   // its body will not fit before the next junction, where its turn gives way to (or asks while
                     // driving with) a car it cannot see will be clear of it as it gets there (issue #284): it could only
                     // wait for that car with its tail in the box behind
};
constexpr uint8_t kRoomRefusalCount = 9;
const char* roomRefusalWords(RoomRefusal r);   // in words, for the driver card and the lab's printout

// WHAT A DRIVER REMEMBERS OF ITS CLAIM ON THE ROOM BEYOND ONE JUNCTION (room.h). A grant is a remembered fact: it holds
// until the car's front reaches the lane (then the car is counted where it is), or the driver gives it back (it decides
// to give way at the line, or its approach starts afresh at a red), or the plan goes.
struct RoomClaim {
    RoomState state = RoomState::None;
    bool asked = false;              // it has asked here at least once ...
    uint64_t askedTick = 0;          // ... first at this tick (cars that asked earlier are granted first)
    bool toWaitInside = false;       // the last ask was to pull forward and wait inside (D4), not to go through
    bool chained = false;            // granted as part of the claim made at the junction before: the car's body would not
                                     // fit between the two, so it passes through this one without stopping
    uint8_t through = 0;             // granted or refused at the head of a claim: how many junctions it covers (1: this)
    engine::PathId lane;             // granted: the lane its connector joins, which the grant counts against ...
    double laneAt = 0;               // ... until its front's odometer reaches this (the lane's start)
    Real need = 0;                   // metres: its length (and trailer) and a standstill gap
    Real had = 0;                    // metres of room there were at its last ask
    RoomRefusal refusal = RoomRefusal::None;   // refused: why ...
    CarId by;                        // ... behind whose tail (NoRoom, CarAheadFirst), when a car is to blame
    engine::JunctionId at;           // ... and at which junction (LightsAhead, MustStopAhead, TooShort)
};
bool operator==(const RoomClaim& a, const RoomClaim& b);

// ONE JUNCTION AHEAD, as a driver plans for it.
struct JunctionPlan {
    engine::JunctionId junction;
    engine::PathId connector;             // the connector the way ahead takes through it
    engine::Turn turn = engine::Turn::None;   // the connector's own stated turn
    double startOdometer = 0;             // the car's odometer reading when its front reaches the connector's start
    uint64_t madeAtTick = 0;              // the tick count when the plan was made
    uint32_t crossingCount = 0;           // how many crossing points lie along the connector (Path::crossings)
    uint8_t waitingCount = 0;             // how many of waitingAt are in use (at least 1: the line)
    std::array<Real, kMaxWaitingPoints> waitingAt{};   // along the connector, in order; the first is 0 (the line)
    // WHERE THIS CAR MAY WAIT INSIDE FOR A GAP on its green (issue #261): its connector's checked place to wait inside
    // (right_of_way.h ConnectorRules::waitInsideAt) when its whole body fits past the line; it is one of waitingAt.
    // kNoWaitInside: nowhere.
    Real waitInsideAt = engine::kNoWaitInside;
    // WHERE THE CAR COMMITS, metres along the connector (issue #284): the start of the first crossing stretch at or beyond
    // its last waiting point (splits left out, as for the waiting points), or the connector's length when none lies there.
    // Its front reaching it, or being unable to stop short of it at its hardest braking, commits the car (noteAtTheLine).
    Real commitAt = 0;

    // WHAT THE CAR REMEMBERS OF THE LINE (issue #262). Its ARRIVAL: when it reaches its line, fixed once, the first
    // time its driver judged this junction -- as the tick its own estimate then had it reaching the line -- or, where it
    // comes to a full stop first, the tick it came to rest there, or, if it never judged, the tick it could no longer
    // stop before the line. "Who reached their line first" settles the pairs asked while driving, by these (give_way.h
    // says why it is fixed when first judged rather than read off the line as each car gets there). CAME TO REST at the
    // line: "I came to rest at this line", which a full stop needs before the car may go (noteAtTheLine, every tick).
    // COMMITTED: it can no longer wait at its last waiting point or short of the stretch after it (design 9.4; commitAt
    // above); task D4 orders committed cars by when.
    bool arrivalFixed = false;
    uint64_t arrivalTick = 0;
    double arrivalAt = 0;
    bool cameToRest = false;
    double restAt = 0;
    bool committed = false;
    uint64_t committedTick = 0;
    // WHAT ITS DRIVER DECIDED at its last decision (give_way.h), how many times it switched between going and giving
    // way on this approach (the brief allows one), and the judgement behind it.
    GiveWayDecision decision = GiveWayDecision::Undecided;
    uint8_t mindChanges = 0;
    GiveWayJudgement judged;
    // ROOM BEYOND (issue #264, room.h): its claim on the room on the lane its connector joins.
    RoomClaim room;

    // The last place along the connector where its front may wait (commitAt is where waiting there ends).
    Real lastWaitingPoint() const { return waitingCount > 0 ? waitingAt[waitingCount - 1] : Real(0); }
};
bool operator==(const JunctionPlan& a, const JunctionPlan& b);
inline bool operator!=(const JunctionPlan& a, const JunctionPlan& b) { return !(a == b); }

// A DRIVER'S PLANS, nearest junction first.
struct JunctionPlans {
    std::array<JunctionPlan, kMaxJunctionPlans> plans{};
    uint8_t count = 0;

    // The plan for `connector`, or null.
    const JunctionPlan* find(engine::PathId connector) const;
};

// WHY A PLAN WAS MADE OR DROPPED.
enum class PlanChange : uint8_t {
    Made,           // its junction came within the look-ahead
    Cleared,        // the car is through: its body has left the connector
    WayChanged,     // the way ahead no longer runs through the connector
    RouteChanged,   // the trip was routed again
};
constexpr uint8_t kPlanChangeCount = 4;
const char* planChangeWords(PlanChange c);   // "made", "dropped: through it", ...

// ONE NOTE FOR THE TRACE: a plan made or dropped.
struct PlanNote {
    uint64_t tick = 0;
    double seconds = 0;
    PlanChange change = PlanChange::Made;
    engine::JunctionId junction;
    engine::PathId connector;
};
bool operator==(const PlanNote& a, const PlanNote& b);

// WHAT ONE UPDATE DID: the notes (made and dropped, in that update's order) and what the caps cut.
struct PlanUpdate {
    std::array<PlanNote, 2 * kMaxJunctionPlans> notes{};
    uint8_t noteCount = 0;
    uint32_t made = 0;
    uint32_t dropped = 0;
    bool plansCut = false;            // more junctions were in view than kMaxJunctionPlans
    bool waitingPointsCut = false;    // a new plan had more waiting points than kMaxWaitingPoints
};

// THE WAITING POINTS of a car `carLength` long on `connector` (see the top of this file), into `plan`'s waitingAt, and
// where it commits (the first crossing stretch beyond the last of them that concerns a car carrying `tags`, ADR-0412).
// True when they all fitted, false when kMaxWaitingPoints cut them short.
bool findWaitingPoints(const engine::LaneGraph& roads, const RightOfWay& row, engine::PathId connector, Real carLength,
                       JunctionPlan& plan, engine::TagMask tags = engine::kEveryVehicle);

// UPDATES A DRIVER'S PLANS at a decision. `car` is the car as it stands (front, odometer, trail); `way` its new way
// ahead, starting with the path its front is on (a way that does not is read as that path alone); `carLength` its
// vehicle's length; `tick` and `seconds` stamp the notes. Plans whose connector is still on the way, or still under the
// car's body, are kept as they are; the rest are dropped (Cleared when the car's front is past the connector's start,
// else WayChanged); connectors on the way with no plan get one, in way order, up to kMaxJunctionPlans. `tags`: its
// vehicle class's tags on these roads (findWaitingPoints).
void updateJunctionPlans(const engine::LaneGraph& roads, const RightOfWay& row, const Car& car, Real carLength,
                         const WayAhead& way, uint64_t tick, double seconds, JunctionPlans& plans, PlanUpdate& out,
                         engine::TagMask tags = engine::kEveryVehicle);

// DROPS EVERY PLAN (the trip was routed again), noting each as `why`.
void dropJunctionPlans(uint64_t tick, double seconds, PlanChange why, JunctionPlans& plans, PlanUpdate& out);

// ONE NUMBER OVER A DRIVER'S PLANS, folded in a fixed order.
uint64_t junctionPlansHash(const JunctionPlans& plans);

// A DRIVER'S PLANS AS TEXT, for the driver card: for each junction ahead, nearest first, the junction, the kind of turn,
// the connector, how far ahead its start is, its crossing points in order with how far ahead each begins and who gives
// way there (for lights, in the phase they are in at `now`, and the light the car's turn shows), and its waiting points.
// Each plan also says what its driver decided (describeGiveWay); `now` (the clock, seconds) says how long ago it came
// to rest at the line.
std::string describeJunctionPlans(const engine::LaneGraph& roads, const RightOfWay& row, const Car& car,
                                  const JunctionPlans& plans, double now = -1);
// ITS CLAIM ON THE ROOM BEYOND one junction, in words: "granted: 6.8 m on lane 1083 (there were 14.2 m)", "refused: no
// room beyond, behind car 40 (I need 6.8 m; there are 1.6 m)".
std::string describeRoom(const JunctionPlan& plan);
// WHAT A DRIVER DECIDED about one junction, in words: "giving way to car 12 on the far turn from the north (connector
// 345), at crossing point 6789: a gap of 2.1 s; I need 4.1 s; its arrival at the line ... t=40.2 s, ...".
std::string describeGiveWay(const engine::LaneGraph& roads, const RightOfWay& row, const JunctionPlan& plan,
                            double now = -1);
// "straight on", "near turn", "far turn", "U-turn".
const char* turnWords(engine::Turn t);

}  // namespace driving

#endif
