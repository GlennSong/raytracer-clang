// Where trips can begin and end (trip_places.h, issue #255), routes that keep to the lanes a car's lane leads into
// (DrivingSettings::laneChanges), a car stranded at the end of a road (Driving::atRoadEnd), and the limit on a road with
// none posted (default_limits): all on the small hand-made roads of driving_test_roads.h.
//   1. Without lane changes, a two-lane grid falls apart into one part per lane: every place picked is a lane (never a
//      connector) of a part with enough lanes, and a route without lane changes exists between any two places of a part.
//   2. A road with dead ends offers no places at all; a ring offers every lane of it.
//   3. A pick is the same every time for the same four numbers, spread over the lanes in proportion to their length.
//   4. Room for a car: none beside another car or by a lane's ends, plenty further on.
//   5. Put down and sent on, again and again: twenty cars on a ring, none overlapping, every trip ending at its place
//      and a new one begun, the same state hash on a second run.
//   6. Routes without lane changes: a destination only a lane change reaches is refused; with lane changes it is
//      routed, and a car that cannot change lanes ends up stranded where its lane ends, with nothing after it
//      (atRoadEnd).
//   7. default_limits: read like the numbers, bound by road class, the limit a driver reads where none is posted; a road
//      class with no default is a problem named at load.

#include "test_framework.h"

#include "driving_test_roads.h"

#include "apps/driving/act.h"
#include "apps/driving/driving.h"
#include "apps/driving/road_rules.h"
#include "apps/driving/trip_places.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace driving;
using driving_test::Heading;
using driving_test::TestRoads;
using driving_test::TestRoadSpec;

namespace {

PathPoint at(engine::PathId path, Real along) {
    PathPoint p;
    p.path = path;
    p.along = along;
    return p;
}

VehicleClasses oneClass() {
    VehicleClasses c;
    c.add(driving_test::testCarClass());
    return c;
}

RouteClass routeClass(const engine::LaneGraph& g, bool laneChanges) {
    RouteClass rc = bindRouteClass(g, driving_test::testCarClass());
    rc.laneChanges = laneChanges;
    return rc;
}

}  // namespace

// ---- 1. parts without lane changes --------------------------------------------------------------------------------------

TEST_CASE(trip_places_1_parts_without_lane_changes_route_everywhere) {
    TestRoadSpec spec;
    spec.lanesEachWay = 2;
    const TestRoads t = driving_test::gridRoads(3, 3, spec);
    const RouteClass rc = routeClass(t.model.laneLevel, false);
    const TripPlaces places = findTripPlaces(t.model.laneLevel, rc, 2.0, 4);
    CHECK(!places.empty());
    // the offside lanes reach only offside lanes, the kerb lanes only kerb lanes: two parts
    CHECK(places.parts.size() == 2u);
    std::size_t lanes = 0;
    for (const TripPart& p : places.parts) {
        lanes += p.lanes.size();
        for (const engine::PathId l : p.lanes) CHECK(t.model.laneLevel.paths[l.index].kind == engine::PathKind::Lane);
    }
    CHECK(lanes == places.laneCount);
    // With lane changes, one part holds every lane of the grid, those two parts' and the rest.
    const TripPlaces joined = findTripPlaces(t.model.laneLevel, routeClass(t.model.laneLevel, true), 2.0, 4);
    CHECK(joined.parts.size() == 1u);
    CHECK(joined.laneCount >= lanes);
    std::size_t gridLanes = 0;
    for (const engine::Path& p : t.model.laneLevel.paths) gridLanes += p.kind == engine::PathKind::Lane ? 1 : 0;
    CHECK(joined.laneCount == gridLanes);
    // Any two places in one part: a route with no lane change between them.
    const RouteBounds bounds = studyRoads(t.model);
    int routes = 0, found = 0;
    for (uint32_t part = 0; part < places.parts.size(); ++part)
        for (uint32_t k = 0; k < 40; ++k) {
            const PathPoint from = placeInPart(places, part, placeDraw(7, k, PlaceQuestion::Start, 0));
            const PathPoint to = placeInPart(places, part, placeDraw(7, k, PlaceQuestion::Destination, 0));
            CHECK(places.partOf(from.path) == part && places.partOf(to.path) == part);
            CHECK(from.along >= places.margin && from.along <= t.model.laneLevel.paths[from.path.index].length - places.margin);
            ++routes;
            const Route r = findRoute(t.model, bounds, rc, from, to);
            if (!r.found) continue;
            ++found;
            // no lane change anywhere: the lane the car is in finishes the route from every leg
            for (const RouteLeg& leg : r.legs) {
                uint32_t changes = kCannotFinish;
                for (uint32_t i = leg.firstLane; i < leg.firstLane + leg.laneCount; ++i)
                    if (r.lanes[i].lane == leg.lane) changes = r.lanes[i].changes;
                CHECK(changes == 0u);
            }
        }
    std::printf("    without lane changes %zu parts, %zu lanes; with them %zu lanes; %d of %d routes found within a part\n",
                places.parts.size(), lanes, joined.laneCount, found, routes);
    CHECK(found == routes);
}

// ---- 2. dead ends offer nothing ----------------------------------------------------------------------------------------

TEST_CASE(trip_places_2_no_place_on_a_dead_end_every_place_on_a_ring) {
    const TestRoads straight = driving_test::straightRoad(4);
    CHECK(findTripPlaces(straight.model.laneLevel, routeClass(straight.model.laneLevel, true), 2.0, 1).empty());
    TestRoadSpec spec;
    spec.lanesEachWay = 2;
    const TestRoads ring = driving_test::ringRoad(6, spec);
    const TripPlaces places = findTripPlaces(ring.model.laneLevel, routeClass(ring.model.laneLevel, false), 2.0, 2);
    CHECK(places.parts.size() == 2u);   // each lane goes round on its own
    CHECK(places.laneCount == 12u);
    // too small a part (asking for more lanes than a ring lane has sides) is left out
    CHECK(findTripPlaces(ring.model.laneLevel, routeClass(ring.model.laneLevel, false), 2.0, 7).empty());
    // a class that may use none of the road has nowhere
    RouteClass none = routeClass(ring.model.laneLevel, false);
    none.tags = 0;
    CHECK(findTripPlaces(ring.model.laneLevel, none, 2.0, 1).empty());
}

// ---- 3. picks ------------------------------------------------------------------------------------------------------------

TEST_CASE(trip_places_3_picks_are_stable_and_spread_by_length) {
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) == placeDraw(1, 42, PlaceQuestion::Start, 3));
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) != placeDraw(1, 42, PlaceQuestion::Start, 4));
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) != placeDraw(1, 42, PlaceQuestion::Destination, 3));
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) != placeDraw(2, 42, PlaceQuestion::Start, 3));
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) != placeDraw(1, 43, PlaceQuestion::Start, 3));
    const TestRoads ring = driving_test::ringRoad(4);
    const TripPlaces places = findTripPlaces(ring.model.laneLevel, routeClass(ring.model.laneLevel, true), 2.0, 1);
    CHECK(places.parts.size() == 1u && places.laneCount == 4u);
    // the ends of the range: the first lane's first usable metre, the last lane's last
    const PathPoint first = placeAnywhere(places, 0);
    CHECK(first.path == places.parts[0].lanes.front() && std::fabs(first.along - 2.0) < 1e-9);
    const PathPoint last = placeAnywhere(places, 1);
    const Real lastLength = ring.model.laneLevel.paths[places.parts[0].lanes.back().index].length;
    CHECK(last.path == places.parts[0].lanes.back() && std::fabs(last.along - (lastLength - 2.0)) < 1e-9);
    // equal lanes get an equal share of 4,000 picks, within a few percent
    std::vector<int> hits(ring.model.laneLevel.paths.size(), 0);
    for (uint32_t k = 0; k < 4000; ++k) ++hits[placeAnywhere(places, placeDraw(9, k, PlaceQuestion::Destination, 0)).path.index];
    for (const engine::PathId l : places.parts[0].lanes) CHECK(hits[l.index] > 900 && hits[l.index] < 1100);
}

// ---- 4. room -------------------------------------------------------------------------------------------------------------

TEST_CASE(trip_places_4_room_for_a_car) {
    const TestRoads ring = driving_test::ringRoad(4);
    Driving d(&ring.model);
    d.setVehicleClasses(oneClass());
    const engine::PathId lane = ring.ringLane(0, 0);
    const Real length = driving_test::testCarClass().length;
    CHECK(roomFor(ring.model.laneLevel, d.pathLists(), at(lane, 50), length, 4));
    CHECK(!roomFor(ring.model.laneLevel, d.pathLists(), at(lane, length + 1), length, 4));   // its rear too near the start
    CHECK(!roomFor(ring.model.laneLevel, d.pathLists(), at(lane, 98), length, 4));           // its front too near the end
    CHECK(d.placeCar(CarId(1), 0, at(lane, 50)));
    CHECK(!roomFor(ring.model.laneLevel, d.pathLists(), at(lane, 50), length, 4));                    // on it
    CHECK(!roomFor(ring.model.laneLevel, d.pathLists(), at(lane, 50 + length + 3), length, 4));       // 3 m ahead of it
    CHECK(roomFor(ring.model.laneLevel, d.pathLists(), at(lane, 50 + length + 5), length, 4));        // 5 m ahead
    CHECK(!roomFor(ring.model.laneLevel, d.pathLists(), at(lane, 50 - length - 3), length, 4));       // 3 m behind
    CHECK(roomFor(ring.model.laneLevel, d.pathLists(), at(ring.ringLane(1, 0), 50), length, 4));      // the next side is clear
}

// ---- 5. put down and sent on ---------------------------------------------------------------------------------------------

namespace {

const char* kRulesLua = R"LUA(
rule_set = { name = 'trip_places_test', drives_on = 'left', ranks = {}, devices = {},
  control = { test_uncontrolled = { settle = { 'arrived_first', 'kerb_side', 'movement' } } },
  numbers = { standstill_gap = { 2, unit = 'm', source = 'test' },
              anticipation = { 0, unit = 's', source = 'test: the following written against it' },
              comfortable_side_accel = { 2, unit = 'm/s2', source = 'test' } },
  default_limits = { test_street = { 20, unit = 'mph', source = 'test' } },
  personality = { gap_kept = { usual = 1.5, low = 1.2, high = 1.8, unit = 's', source = 'test' },
                  speed_habit = { usual = 1, low = 0.95, high = 1.05, unit = 'ratio' } },
  rules = {
    { name = 'speed limit', give = { top_speed = 'my_limit' } },
    { name = 'bend ahead', give = { top_speed = 'corner_speed' } },
    { name = 'lane ends', when = 'lane_ends_ahead', give = { stop_at = 'lane_end' } },
    { name = 'arrive', give = { stop_at = 'destination' } },
  } }
)LUA";

}  // namespace

#ifdef RT_ENABLE_SCRIPTING

#include "apps/driving/scripting/rule_set_lua.h"
#include "apps/driving/scripting/rules_lua.h"
#include "engine/scripting/script_vm.h"

namespace {

// ...and with the lane change step (sitting 5): its numbers, the lab's own rule, and the step's facts registered.
const char* kLaneChangeLua = R"LUA(
rule_set.numbers.lane_change_safe_brake = { 4, unit = 'm/s2', source = 'test: MOBIL b_safe' }
rule_set.numbers.lane_change_seconds = { 4, unit = 's', source = 'test' }
table.insert(rule_set.rules, { name = 'change lanes for my route', when = { all = { 'wrong_lane', 'gap_beside' } },
                                give = { lane = 'change' } })
table.insert(rule_set.rules, { name = 'wait to change lanes', when = 'must_wait_to_change',
                                give = { stop_at = 'change_wait_line' } })
)LUA";

// The rules above, read and bound to `d`'s road the way a level reads them, handed to `d`.
bool giveRules(Driving& d, std::vector<std::string>* problems = nullptr, bool laneChangeStep = false) {
    FactRegistry facts;
    if (!registerBuiltInFacts(facts) || !registerMoveNeeds(facts)) return false;
    engine::ScriptVM vm;
    std::string err;
    RuleSet ruleSet;
    AnswerVocabulary vocabulary;
    CompiledRules rules;
    if (laneChangeStep &&
        (!registerLaneChangeFacts(facts, problems) || !registerLaneChangeAnswers(vocabulary, &err))) return false;
    if (!vm.doString(kRulesLua, &err) || (laneChangeStep && !vm.doString(kLaneChangeLua, &err)) ||
        !loadRuleSet(vm, "trip_places_test.lua", ruleSet, &err) ||
        !loadRules(vm, "trip_places_test.lua", facts, ruleSet, vocabulary, rules, &err)) {
        std::printf("    rules: %s\n", err.c_str());
        return false;
    }
    RoadRules roadRules;
    std::vector<std::string> found;
    if (!buildRoadRules(*d.model(), ruleSet, facts, roadRules, &found)) {
        if (problems) *problems = found;
        return false;
    }
    return d.useRules(ruleSet, facts, vocabulary, rules, roadRules, &err);
}

// Twenty cars put down on a two-lane ring and sent on trip after trip for `seconds`; the state hash at the end.
struct RingRun {
    uint64_t hash = 0;
    int placed = 0, arrivals = 0, renewed = 0, overlaps = 0;
};
RingRun runRing(Real seconds) {
    TestRoadSpec spec;
    spec.lanesEachWay = 2;
    spec.speedLimit = 0;   // no posted limit: the rule set's default for the road's class
    const TestRoads ring = driving_test::ringRoad(6, spec);
    Driving d(&ring.model);
    d.setVehicleClasses(oneClass());
    DrivingSettings settings;
    settings.laneChanges = false;
    d.setSettings(settings);
    RingRun out;
    if (!giveRules(d)) return out;
    const TripPlaces places = findTripPlaces(ring.model.laneLevel, routeClass(ring.model.laneLevel, false), 2.0, 2);
    std::vector<uint32_t> startDraws(20, 0), tripDraws(20, 0);
    for (uint32_t k = 0; k < 20; ++k) {
        const TripPlacing put = putCarSomewhere(d, places, CarId(k), 0, 4.0, 5, k, startDraws[k], 64);
        if (!put.done) continue;
        ++out.placed;
        sendCarSomewhere(d, places, CarId(k), k, 5, tripDraws[k], 64);
    }
    const Real dt = 1.0 / 60;
    for (int i = 0; i < static_cast<int>(seconds / dt); ++i) {
        d.tick(dt);
        for (const Event& e : d.drainEvents()) {
            if (e.kind != EventKind::Parked) continue;
            ++out.arrivals;
            const Car* c = d.car(e.car);
            if (c && !c->driven && std::fabs(c->speed) < 1e-9 &&
                sendCarSomewhere(d, places, e.car, e.person, 5, tripDraws[e.car.index], 64).done)
                ++out.renewed;
        }
        // no two bodies overlap on any path
        for (std::size_t p = 0; p < ring.model.laneLevel.paths.size(); ++p) {
            const PathList list = d.pathLists().on(engine::PathId(static_cast<uint32_t>(p)));
            for (std::size_t k = 0; k + 1 < list.size(); ++k)
                if (list[k + 1].rear < list[k].front - 1e-9) ++out.overlaps;
        }
    }
    out.hash = d.stateHash();
    return out;
}

}  // namespace

TEST_CASE(trip_places_5_put_down_and_sent_on_again_and_again) {
    const RingRun a = runRing(120);
    std::printf("    %d cars placed; %d arrivals, %d trips renewed, %d overlaps in two minutes\n", a.placed, a.arrivals,
                a.renewed, a.overlaps);
    CHECK(a.placed == 20);
    CHECK(a.arrivals > 20);
    CHECK(a.renewed == a.arrivals);
    CHECK(a.overlaps == 0);
    CHECK(runRing(120).hash == a.hash);
}

// ---- 6. routes without lane changes, and a car stranded at the end of a road -------------------------------------------

TEST_CASE(trip_places_6_no_lane_change_routes_and_the_road_end) {
    // Two sections of two lanes heading east; the kerb lane of the first ENDS (nothing after it), as a lane does where
    // it drops at a merge. A car in it bound for the second section must move over to the other lane on the way.
    TestRoadSpec spec;
    spec.lanesEachWay = 2;
    TestRoads t = driving_test::straightRoad(2, spec);
    const engine::PathId from = t.straightLane(0, true, 1), to = t.straightLane(1, true, 1);
    t.model.laneLevel.paths[from.index].next = engine::Range();
    for (const bool laneChanges : {false, true}) {
        Driving d(&t.model);
        d.setVehicleClasses(oneClass());
        DrivingSettings settings;
        settings.laneChanges = laneChanges;
        d.setSettings(settings);
        CHECK(giveRules(d));
        CHECK(d.placeCar(CarId(1), 0, at(from, 20)));
        const TripId trip = d.beginTrip(CarId(1), 1, at(to, 50));
        if (!laneChanges) {
            // only a lane change reaches the other lane: no route, and the host is told
            CHECK(!trip.valid());
            const std::vector<Event> events = d.drainEvents();
            CHECK(events.size() == 1u && events[0].kind == EventKind::NoRoute);
            continue;
        }
        // routed with a lane change the tick cannot make: the car keeps to its lane and stops where that lane ends
        CHECK(trip.valid());
        CHECK(!d.atRoadEnd(CarId(1)));
        for (int i = 0; i < 60 * 120 && !d.atRoadEnd(CarId(1)); ++i) d.tick(1.0 / 60);
        const Car* c = d.car(CarId(1));
        CHECK(d.atRoadEnd(CarId(1)));
        CHECK(c && c->path == from && c->speed < kStandingBelow);
        std::printf("    stranded %.2f m short of its lane's end after %.1f s\n",
                    c ? t.model.laneLevel.paths[c->path.index].length - c->along : -1.0, d.clockSeconds());
        // ended and taken off, it is no longer there
        CHECK(d.endTrip(trip));
        CHECK(!d.atRoadEnd(CarId(1)));
        CHECK(d.removeCar(CarId(1)));
    }
}

// ---- 6b. the same car, with the lane change step: it moves over and gets there (sitting 5) ------------------------------

TEST_CASE(trip_places_6b_a_car_changes_lanes_for_its_route) {
    TestRoadSpec spec;
    spec.lanesEachWay = 2;
    TestRoads t = driving_test::straightRoad(2, spec);
    const engine::PathId from = t.straightLane(0, true, 1), to = t.straightLane(1, true, 1);
    t.model.laneLevel.paths[from.index].next = engine::Range();
    const engine::PathId beside = laneBeside(t.model.laneLevel, from, false);
    CHECK(beside.valid());
    // A. AN EMPTY ROAD: it moves into the lane beside, listed in both while it does, and arrives
    {
        Driving d(&t.model);
        d.setVehicleClasses(oneClass());
        DrivingSettings settings;
        settings.laneChanges = true;
        d.setSettings(settings);
        CHECK(giveRules(d, nullptr, true));
        CHECK(d.laneChangeStep());
        CHECK(d.placeCar(CarId(1), 0, at(from, 20)));
        const TripId trip = d.beginTrip(CarId(1), 1, at(to, 50));
        CHECK(trip.valid());
        bool arrived = false, listedTwice = false;
        Real widest = 0, worstJump = 0, worstTurn = 0;
        bool had = false;
        CarPose was;
        Real wasSpeed = 0, wasOffset = 0;
        engine::PathId wasPath;
        for (int i = 0; i < 60 * 120 && !arrived; ++i) {
            d.tick(1.0 / 60);
            if (const Car* c = d.car(CarId(1))) {
                // (the drawn front never moves further than its speed and its sideways move carry it, nor turns on the spot)
                const CarPose pose = carPose(t.model.laneLevel, d.vehicleClasses(), d.ruleSet(), *c);
                if (had) {
                    const Real dx = pose.front.x - was.front.x, dz = pose.front.z - was.front.z;
                    const Real side = c->path == wasPath ? std::fabs(c->offset - wasOffset) : Real(0);
                    const Real along = std::sqrt(std::max(Real(0), dx * dx + dz * dz - side * side));
                    worstJump = std::max(worstJump, along - std::max(wasSpeed, c->speed) / 60);
                    const Real turn = std::fabs(was.body.x * pose.body.z - was.body.z * pose.body.x);
                    worstTurn = std::max(worstTurn, turn);
                }
                had = true;
                was = pose;
                wasSpeed = c->speed;
                wasOffset = c->offset;
                wasPath = c->path;
                if (c->besidePath.valid()) {
                    bool onMine = false, onOld = false;
                    for (const PathEntry& e : d.pathLists().on(c->path)) onMine = onMine || e.car == CarId(1);
                    for (const PathEntry& e : d.pathLists().on(c->besidePath)) onOld = onOld || e.car == CarId(1);
                    listedTwice = listedTwice || (onMine && onOld);
                }
                widest = std::max(widest, std::fabs(c->offset));
            }
            for (const Event& e : d.drainEvents()) arrived = arrived || e.kind == EventKind::Parked;
        }
        const Driving::LaneChangeCounts& n = d.laneChangeCounts();
        std::printf("    an empty road: begun %llu, finished %llu, missed %llu; listed in both lanes %d; widest offset %.2f m; "
                    "arrived %d at %.1f s\n",
                    static_cast<unsigned long long>(n.begun), static_cast<unsigned long long>(n.finished),
                    static_cast<unsigned long long>(n.missed), listedTwice ? 1 : 0, widest, arrived ? 1 : 0, d.clockSeconds());
        // (twice: out of the lane that ends, then back to the kerb lane its destination is on in the next stretch)
        std::printf("    ...its drawn front: the worst move beyond its speed %.4f m a tick; the most its body turned in a tick %.5f rad\n",
                    worstJump, worstTurn);
        CHECK(worstJump < 0.01);
        // (once: out of the lane that ends; in its destination's stretch, level with it in any lane is there)
        CHECK(n.begun == 1u && n.finished == 1u && n.missed == 0u);
        CHECK(listedTwice);
        CHECK(widest > 2.0 && widest < 4.5);   // (it started from its old lane's line: a lane's width over)
        CHECK(arrived);
        CHECK(!d.atRoadEnd(CarId(1)));
    }
    // B. A LANE OF STANDING CARS BESIDE IT (placed, never driven): no gap. As it nears where a move across must begin it
    // asks for a way on from its lane -- here there is none (its lane ends) -- so the host is told "no route" (once), as
    // when a trip is begun with nowhere to go, and the driver stops where a move across still fits and waits there for a
    // gap (never at its lane's end, too late to get across). C: the standing cars drive off, and it moves over from rest.
    for (const bool theyLeave : {false, true}) {
        Driving d(&t.model);
        d.setVehicleClasses(oneClass());
        DrivingSettings settings;
        settings.laneChanges = true;
        d.setSettings(settings);
        CHECK(giveRules(d, nullptr, true));
        const Real end = t.model.laneLevel.paths[from.index].length;
        CHECK(d.placeCar(CarId(1), 0, at(from, 20)));
        // (bumper to bumper from the lane's start to its end: no gap a car fits in anywhere)
        const Real spacing = d.vehicleClasses().at(0).length + 0.3;
        std::vector<CarId> standing;
        for (uint32_t k = 0; spacing * (k + 1) <= end; ++k)
            if (d.placeCar(CarId(10 + k), 0, at(beside, spacing * (k + 1)))) standing.push_back(CarId(10 + k));
        CHECK(d.beginTrip(CarId(1), 1, at(to, 50)).valid());
        int noRoutes = 0;
        bool arrived = false;
        Real restedAt = -1;
        for (int i = 0; i < 60 * 60 && !arrived; ++i) {
            d.tick(1.0 / 60);
            for (const Event& e : d.drainEvents()) {
                noRoutes += e.kind == EventKind::NoRoute && e.car == CarId(1) ? 1 : 0;
                arrived = arrived || (e.kind == EventKind::Parked && e.car == CarId(1));
            }
            const Car* me = d.car(CarId(1));
            if (me && me->path == from && me->speed < kStandingBelow && d.clockSeconds() > 5) restedAt = me->along;
            // (C: once it has waited a while, the lane beside empties)
            if (theyLeave && d.clockSeconds() > 30 && !standing.empty()) {
                for (const CarId c : standing) d.removeCar(c);
                standing.clear();
            }
        }
        const Driving::LaneChangeCounts& n = d.laneChangeCounts();
        std::printf("    %s: begun %llu, missed %llu (no route on %llu); told the host no route %d time(s); waited at %.1f m of "
                    "%.1f; at the road's end %d; arrived %d\n",
                    theyLeave ? "the standing cars beside it leave" : "a lane of standing cars beside it",
                    static_cast<unsigned long long>(n.begun), static_cast<unsigned long long>(n.missed),
                    static_cast<unsigned long long>(n.missedNoRoute), noRoutes, restedAt, end,
                    d.atRoadEnd(CarId(1)) ? 1 : 0, arrived ? 1 : 0);
        CHECK(n.missed == 1u && n.missedNoRoute == 1u);
        CHECK(noRoutes == 0);   // (the trip goes on: the host is not told "no route", #326 -- told, a resident got out)
        CHECK(restedAt > 0 && restedAt < end - 5.0);   // (stopped with room left to move across)
        CHECK(!d.atRoadEnd(CarId(1)));
        if (theyLeave) {
            CHECK(n.begun == 1u);
            CHECK(arrived);
        } else {
            CHECK(n.begun == 0u);
        }
    }
}

// ---- 7. the limit on a road with none posted ----------------------------------------------------------------------------

TEST_CASE(trip_places_7_default_limits_by_road_class) {
    TestRoadSpec spec;
    spec.speedLimit = 0;
    const TestRoads t = driving_test::straightRoad(2, spec);
    engine::ScriptVM vm;
    std::string err;
    RuleSet ruleSet;
    CHECK(vm.doString(kRulesLua, &err));
    CHECK(loadRuleSet(vm, "trip_places_test.lua", ruleSet, &err));
    const Number* limit = ruleSet.findDefaultLimit(driving_test::kTestRoadClass);
    CHECK(limit && std::fabs(limit->value - 20 * 0.44704) < 1e-9 && limit->unit == "mph" && limit->hasSource);
    const PersonalSetting* gap = ruleSet.findPersonal("gap_kept");
    CHECK(gap && gap->hasSource && gap->source == "test");
    // bound: every unposted path of the class takes it; a posted one keeps its own
    FactRegistry facts;
    CHECK(registerBuiltInFacts(facts) && registerMoveNeeds(facts));
    RoadRules rr;
    std::vector<std::string> problems;
    CHECK(buildRoadRules(t.model, ruleSet, facts, rr, &problems));
    const engine::PathId lane = t.straightLane(0, true, 0);
    CHECK(std::fabs(rr.baseLimit(lane) - 20 * 0.44704) < 1e-9);
    TestRoads posted = t;   // the same road with a limit posted on that lane's street (stated on the model)
    posted.model.streets[posted.model.laneLevel.paths[lane.index].street].speedLimit = 9;
    RoadRules postedRules;
    CHECK(buildRoadRules(posted.model, ruleSet, facts, postedRules, &problems));
    CHECK(postedRules.baseLimit(lane) == 9);
    // a class with no default: one problem, naming the class and the table
    RuleSet bare = ruleSet;
    bare.defaultLimits.clear();
    problems.clear();
    CHECK(!buildRoadRules(t.model, bare, facts, rr, &problems));
    CHECK(problems.size() == 1u && problems[0].find("default_limits.test_street") != std::string::npos);
    // a limit that is not a speed is refused when the file is read
    engine::ScriptVM bad;
    CHECK(bad.doString("rule_set = { name = 'x', drives_on = 'left', ranks = {}, devices = {}, numbers = {}, "
                       "personality = {}, default_limits = { test_street = { 20, unit = 's' } } }"));
    RuleSet refused;
    CHECK(!loadRuleSet(bad, "bad.lua", refused, &err));
    CHECK(err.find("bad.lua: rule_set.default_limits.test_street.unit") != std::string::npos);
    // a car on the road drives at that limit
    Driving d(&t.model);
    d.setVehicleClasses(oneClass());
    CHECK(giveRules(d));
    CHECK(d.placeCar(CarId(1), 0, at(lane, 10)));
    CHECK(d.beginTrip(CarId(1), 1, at(t.straightLane(1, true, 0), 90)).valid());
    Real fastest = 0;
    for (int i = 0; i < 60 * 12; ++i) {
        d.tick(1.0 / 60);
        if (const Car* c = d.car(CarId(1))) fastest = std::max(fastest, c->speed);
    }
    std::printf("    unposted road: fastest %.2f m/s against a default of %.2f m/s (habit 0.95-1.05)\n", fastest,
                20 * 0.44704);
    CHECK(fastest > 20 * 0.44704 * 0.9 && fastest < 20 * 0.44704 * 1.06);
}

#endif   // RT_ENABLE_SCRIPTING
