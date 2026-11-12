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
    const RouteClass rc = routeClass(t.graph, false);
    const TripPlaces places = findTripPlaces(t.graph, rc, 2.0, 4);
    CHECK(!places.empty());
    // the offside lanes reach only offside lanes, the kerb lanes only kerb lanes: two parts
    CHECK(places.parts.size() == 2u);
    std::size_t lanes = 0;
    for (const TripPart& p : places.parts) {
        lanes += p.lanes.size();
        for (const engine::PathId l : p.lanes) CHECK(t.graph.paths[l.index].kind == engine::PathKind::Lane);
    }
    CHECK(lanes == places.laneCount);
    // With lane changes, one part holds every lane of the grid, those two parts' and the rest.
    const TripPlaces joined = findTripPlaces(t.graph, routeClass(t.graph, true), 2.0, 4);
    CHECK(joined.parts.size() == 1u);
    CHECK(joined.laneCount >= lanes);
    std::size_t gridLanes = 0;
    for (const engine::Path& p : t.graph.paths) gridLanes += p.kind == engine::PathKind::Lane ? 1 : 0;
    CHECK(joined.laneCount == gridLanes);
    // Any two places in one part: a route with no lane change between them.
    const RouteBounds bounds = studyRoads(t.graph);
    int routes = 0, found = 0;
    for (uint32_t part = 0; part < places.parts.size(); ++part)
        for (uint32_t k = 0; k < 40; ++k) {
            const PathPoint from = placeInPart(places, part, placeDraw(7, k, PlaceQuestion::Start, 0));
            const PathPoint to = placeInPart(places, part, placeDraw(7, k, PlaceQuestion::Destination, 0));
            CHECK(places.partOf(from.path) == part && places.partOf(to.path) == part);
            CHECK(from.along >= places.margin && from.along <= t.graph.paths[from.path.index].length - places.margin);
            ++routes;
            const Route r = findRoute(t.graph, bounds, rc, from, to);
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
    CHECK(findTripPlaces(straight.graph, routeClass(straight.graph, true), 2.0, 1).empty());
    TestRoadSpec spec;
    spec.lanesEachWay = 2;
    const TestRoads ring = driving_test::ringRoad(6, spec);
    const TripPlaces places = findTripPlaces(ring.graph, routeClass(ring.graph, false), 2.0, 2);
    CHECK(places.parts.size() == 2u);   // each lane goes round on its own
    CHECK(places.laneCount == 12u);
    // too small a part (asking for more lanes than a ring lane has sides) is left out
    CHECK(findTripPlaces(ring.graph, routeClass(ring.graph, false), 2.0, 7).empty());
    // a class that may use none of the road has nowhere
    RouteClass none = routeClass(ring.graph, false);
    none.tags = 0;
    CHECK(findTripPlaces(ring.graph, none, 2.0, 1).empty());
}

// ---- 3. picks ------------------------------------------------------------------------------------------------------------

TEST_CASE(trip_places_3_picks_are_stable_and_spread_by_length) {
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) == placeDraw(1, 42, PlaceQuestion::Start, 3));
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) != placeDraw(1, 42, PlaceQuestion::Start, 4));
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) != placeDraw(1, 42, PlaceQuestion::Destination, 3));
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) != placeDraw(2, 42, PlaceQuestion::Start, 3));
    CHECK(placeDraw(1, 42, PlaceQuestion::Start, 3) != placeDraw(1, 43, PlaceQuestion::Start, 3));
    const TestRoads ring = driving_test::ringRoad(4);
    const TripPlaces places = findTripPlaces(ring.graph, routeClass(ring.graph, true), 2.0, 1);
    CHECK(places.parts.size() == 1u && places.laneCount == 4u);
    // the ends of the range: the first lane's first usable metre, the last lane's last
    const PathPoint first = placeAnywhere(places, 0);
    CHECK(first.path == places.parts[0].lanes.front() && std::fabs(first.along - 2.0) < 1e-9);
    const PathPoint last = placeAnywhere(places, 1);
    const Real lastLength = ring.graph.paths[places.parts[0].lanes.back().index].length;
    CHECK(last.path == places.parts[0].lanes.back() && std::fabs(last.along - (lastLength - 2.0)) < 1e-9);
    // equal lanes get an equal share of 4,000 picks, within a few percent
    std::vector<int> hits(ring.graph.paths.size(), 0);
    for (uint32_t k = 0; k < 4000; ++k) ++hits[placeAnywhere(places, placeDraw(9, k, PlaceQuestion::Destination, 0)).path.index];
    for (const engine::PathId l : places.parts[0].lanes) CHECK(hits[l.index] > 900 && hits[l.index] < 1100);
}

// ---- 4. room -------------------------------------------------------------------------------------------------------------

TEST_CASE(trip_places_4_room_for_a_car) {
    const TestRoads ring = driving_test::ringRoad(4);
    Driving d(&ring.graph);
    d.setVehicleClasses(oneClass());
    const engine::PathId lane = ring.ringLane(0, 0);
    const Real length = driving_test::testCarClass().length;
    CHECK(roomFor(ring.graph, d.pathLists(), at(lane, 50), length, 4));
    CHECK(!roomFor(ring.graph, d.pathLists(), at(lane, length + 1), length, 4));   // its rear too near the start
    CHECK(!roomFor(ring.graph, d.pathLists(), at(lane, 98), length, 4));           // its front too near the end
    CHECK(d.placeCar(CarId(1), 0, at(lane, 50)));
    CHECK(!roomFor(ring.graph, d.pathLists(), at(lane, 50), length, 4));                    // on it
    CHECK(!roomFor(ring.graph, d.pathLists(), at(lane, 50 + length + 3), length, 4));       // 3 m ahead of it
    CHECK(roomFor(ring.graph, d.pathLists(), at(lane, 50 + length + 5), length, 4));        // 5 m ahead
    CHECK(!roomFor(ring.graph, d.pathLists(), at(lane, 50 - length - 3), length, 4));       // 3 m behind
    CHECK(roomFor(ring.graph, d.pathLists(), at(ring.ringLane(1, 0), 50), length, 4));      // the next side is clear
}

// ---- 5. put down and sent on ---------------------------------------------------------------------------------------------

namespace {

const char* kRulesLua = R"LUA(
rule_set = { name = 'trip_places_test', drives_on = 'left', ranks = {}, devices = {},
  control = { test_uncontrolled = { settle = { 'arrived_first', 'kerb_side', 'movement' } } },
  numbers = { standstill_gap = { 2, unit = 'm', source = 'test' },
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

// The rules above, read and bound to `g` the way a level reads them, handed to `d`.
bool giveRules(Driving& d, const engine::LaneGraph& g, std::vector<std::string>* problems = nullptr) {
    FactRegistry facts;
    if (!registerBuiltInFacts(facts) || !registerMoveNeeds(facts)) return false;
    engine::ScriptVM vm;
    std::string err;
    RuleSet ruleSet;
    AnswerVocabulary vocabulary;
    CompiledRules rules;
    if (!vm.doString(kRulesLua, &err) || !loadRuleSet(vm, "trip_places_test.lua", ruleSet, &err) ||
        !loadRules(vm, "trip_places_test.lua", facts, ruleSet, vocabulary, rules, &err)) {
        std::printf("    rules: %s\n", err.c_str());
        return false;
    }
    RoadRules roadRules;
    std::vector<std::string> found;
    if (!buildRoadRules(g, ruleSet, facts, roadRules, &found)) {
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
    Driving d(&ring.graph);
    d.setVehicleClasses(oneClass());
    DrivingSettings settings;
    settings.laneChanges = false;
    d.setSettings(settings);
    RingRun out;
    if (!giveRules(d, ring.graph)) return out;
    const TripPlaces places = findTripPlaces(ring.graph, routeClass(ring.graph, false), 2.0, 2);
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
        for (std::size_t p = 0; p < ring.graph.paths.size(); ++p) {
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
    t.graph.paths[from.index].next = engine::Range();
    for (const bool laneChanges : {false, true}) {
        Driving d(&t.graph);
        d.setVehicleClasses(oneClass());
        DrivingSettings settings;
        settings.laneChanges = laneChanges;
        d.setSettings(settings);
        CHECK(giveRules(d, t.graph));
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
                    c ? t.graph.paths[c->path.index].length - c->along : -1.0, d.clockSeconds());
        // ended and taken off, it is no longer there
        CHECK(d.endTrip(trip));
        CHECK(!d.atRoadEnd(CarId(1)));
        CHECK(d.removeCar(CarId(1)));
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
    CHECK(buildRoadRules(t.graph, ruleSet, facts, rr, &problems));
    const engine::PathId lane = t.straightLane(0, true, 0);
    CHECK(std::fabs(rr.baseLimit(t.graph, lane) - 20 * 0.44704) < 1e-9);
    TestRoads posted = t;
    posted.graph.paths[lane.index].speedLimit = 9;
    CHECK(rr.baseLimit(posted.graph, lane) == 9);
    // a class with no default: one problem, naming the class and the table
    RuleSet bare = ruleSet;
    bare.defaultLimits.clear();
    problems.clear();
    CHECK(!buildRoadRules(t.graph, bare, facts, rr, &problems));
    CHECK(problems.size() == 1u && problems[0].find("default_limits.test_street") != std::string::npos);
    // a limit that is not a speed is refused when the file is read
    engine::ScriptVM bad;
    CHECK(bad.doString("rule_set = { name = 'x', drives_on = 'left', ranks = {}, devices = {}, numbers = {}, "
                       "personality = {}, default_limits = { test_street = { 20, unit = 's' } } }"));
    RuleSet refused;
    CHECK(!loadRuleSet(bad, "bad.lua", refused, &err));
    CHECK(err.find("bad.lua: rule_set.default_limits.test_street.unit") != std::string::npos);
    // a car on the road drives at that limit
    Driving d(&t.graph);
    d.setVehicleClasses(oneClass());
    CHECK(giveRules(d, t.graph));
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
