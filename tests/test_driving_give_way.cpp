// GIVING WAY AT JUNCTIONS (issue #262, task D3; docs/driving-briefs/D3-give-way.md): announcements on the board, the
// give-way judgement, patience. On the hand-made grids of driving_test_roads.h, whose junctions carry real crossing
// points from the road builder's own finder. Every scenario runs whole cars through the whole tick and watches their
// bodies on the plan: two bodies overlapping fails, whatever the cars were doing.
//
//   1. the shared estimate of "when", and a queue's following interval;
//   2. the board's order does not depend on the order things were written to it, nor on the order cars were placed;
//   3. two cars on crossing paths: the one that should give way stops at the line, the other never slows;
//   4. a T with a stop sign on the side road: a full stop first, then a queue empties into gaps on the main road, every
//      gap taken between the floor and the full gap, and the main road never slows;
//   5. two streams joining one lane zip without touching;
//   6. a far turn across oncoming traffic, at an uncontrolled four-way and on a green (lights at their first phase);
//   7. four cars reaching a four-way in the same tick (uncontrolled, and an all-way stop): one at a time where their
//      paths meet, in the same order every run;
//   8. a long vehicle: the car crossing behind it waits for its tail;
//   9. a car that can no longer stop is not stopped, and the other gives way to it;
//  10. a refused turn: routed another way, it gives way there;
//  11. a meeting filed under no junction (two junctions' connectors): asked while driving, never both at once;
//  12. a car with right of way that is held at its line announces "not going", and cross traffic does not wait for it;
//  13. patience over many people: every gap taken lies between the floor and the full gap;
//  14. ten minutes of random traffic at an uncontrolled four-way, an all-way stop and a T with a stop sign: no loop in
//      the wait-for graph, no two bodies overlapping, nobody changes their mind twice, traffic flows;
//  15. the driver card and the wait-for graph name the car given way to.

#include "test_framework.h"

#include "driving_test_roads.h"

#include "apps/driving/act.h"
#include "apps/driving/announce.h"
#include "apps/driving/driving.h"
#include "apps/driving/give_way.h"
#include "apps/driving/road_rules.h"
#include "apps/driving/wait_graph.h"
#include "engine/bundle/bundle.h"   // Fnv

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace driving;
using driving_test::Heading;
using driving_test::TestRoads;
using driving_test::TestRoadSpec;

namespace {

constexpr Heading kEast = Heading::East, kNorth = Heading::North, kWest = Heading::West, kSouth = Heading::South;
constexpr Real kStep = 1.0 / 60;

// The numbers the scenarios read back (the same as the rule set below writes).
constexpr Real kTestGapComfortable = 2.5, kTestGapHard = 1.0, kTestGapFloor = 1.5;

}  // namespace

// ---- 1. The estimate --------------------------------------------------------------------------------------------------

TEST_CASE(give_way_1_the_estimate_of_when) {
    Mover m;
    m.speedUp = 1.5;
    m.brake = 2.0;
    m.speedThrough = 10;
    // At the speed through already: distance over speed.
    m.speed = 10;
    CHECK_APPROX(secondsToCover(50, m), 5.0, 1e-9);
    // From rest: speeding up at 1.5 to 10 m/s takes 33.3 m and 6.67 s; the rest at 10 m/s.
    m.speed = 0;
    CHECK_APPROX(secondsToCover(12, m), 4.0, 1e-9);   // 12 = 0.5 * 1.5 * 4^2
    CHECK_APPROX(secondsToCover(43.333333333333336, m), 10.0 / 1.5 + 1.0, 1e-9);
    // Slowing toward a lower speed through.
    m.speed = 14;
    m.speedThrough = 6;
    const Real slowing = (14.0 * 14 - 36) / (2 * 2.0);   // 40 m
    CHECK_APPROX(secondsToCover(slowing + 12, m), 4.0 + 2.0, 1e-9);
    // Stopping first at a line 20 m off, from 10 m/s: 5 m at speed, 25 m braking... the line is nearer than that: an
    // even stop over 20 m at 10 m/s takes 4 s; then from rest.
    m.speed = 10;
    m.speedThrough = 10;
    CHECK_APPROX(secondsToStopAt(20, m), 4.0, 1e-9);
    Mover rest = m;
    rest.speed = 0;
    CHECK_APPROX(secondsToCoverStoppingFirst(32, 20, m), 4.0 + secondsToCover(12, rest), 1e-9);
    // Never zero speed: a car held at a top speed of zero is still assumed to cross at a crawl.
    Mover held = m;
    held.speed = 0;
    held.speedThrough = 0;
    CHECK(std::isfinite(secondsToCover(10, held)) && secondsToCover(10, held) > 0);
}

// ---- The scenarios' road, rules and watcher --------------------------------------------------------------------------

#ifdef RT_ENABLE_SCRIPTING

#include "apps/driving/scripting/rule_set_lua.h"
#include "apps/driving/scripting/rules_lua.h"
#include "engine/scripting/script_vm.h"

namespace {

const char* kFile = "d3_test.lua";

// The rule set: the control row the test asks for, the junction numbers, and the rules of the lab plus the two full
// stops (an all-way stop and a stop sign), written as data.
std::string rulesLua(const std::string& control) {
    return R"LUA(
rule_set = { name = 'd3', drives_on = 'left', ranks = { 'priority', 'give_way', 'stop' },
  devices = { stop = { means = { stop_line = true, rank = 'stop' } } },
  control = { test_uncontrolled = )LUA" + control + R"LUA( },
  movement_rank = { 'straight', 'near_turn', 'far_turn', 'u_turn' },
  numbers = {
    standstill_gap = { 2, unit = 'm', source = 'test' },
    comfortable_side_accel = { 2, unit = 'm/s2', source = 'test' },
    junction_look_ahead = { 10, unit = 's', source = 'test' },
    gap_comfortable = { 2.5, unit = 's', source = 'test' },
    gap_hard = { 1.0, unit = 's', source = 'test' },
    gap_floor = { 1.5, unit = 's', source = 'test' },
    gap_shrink = { 0.05, unit = 'ratio', source = 'test' },
    at_line_within = { 1.0, unit = 'm', source = 'test' } },
  personality = {
    gap_kept = { usual = 1.5, low = 1.2, high = 1.8, unit = 's', source = 'test' },
    speed_habit = { usual = 1, low = 0.95, high = 1.05, unit = 'ratio', source = 'test' },
    patience = { usual = 1, low = 0.5, high = 2, unit = 'ratio', source = 'test' } },
  rules = {
    { name = 'speed limit', give = { top_speed = 'my_limit' } },
    { name = 'bend ahead', give = { top_speed = 'corner_speed' } },
    { name = 'lane ends', when = 'lane_ends_ahead', give = { stop_at = 'lane_end' } },
    { name = 'arrive', give = { stop_at = 'destination' } },
    { name = 'held', give = { stop_at = 'hold_ahead' } },
    { name = 'give way', when = 'must_give_way', give = { stop_at = 'give_way_line' } },
    { name = 'all-way stop', when = { all = { 'everyone_stops', { 'not', 'stopped_at_line' } } },
      give = { stop_at = 'next_line' } },
    { name = 'stop sign', when = { all = { { 'sign_rank', 'is', 'stop' }, { 'not', 'stopped_at_line' } } },
      give = { stop_at = 'next_line' } },
  } }
)LUA";
}

const char* kUncontrolled = "{ settle = { 'arrived_first', 'kerb_side', 'movement' } }";
const char* kAllWayStop = "{ settle = { 'arrived_first', 'kerb_side', 'movement' }, everyone_stops = true }";
const char* kMinorStop = "{ settle = { 'sign_rank', 'movement', 'arrived_first' } }";
const char* kLights = "{ settle = { 'light', 'movement' }, gives_way_on_green = { 'far_turn' } }";

// A stop sign standing at the end of `lane` (a "stop" regulation), as the T test of D1 places one.
void standStopSign(engine::LaneGraph& g, engine::PathId lane) {
    g.regulationKindNames = {"stop"};
    engine::Regulation stop;
    stop.kind = 0;
    stop.path = lane;
    stop.at = g.paths[lane.index].length;
    g.regulations = {stop};
    g.paths[lane.index].regulations = engine::Range{0, 1};
}

PathPoint at(engine::PathId path, Real along) {
    PathPoint p;
    p.path = path;
    p.along = along;
    return p;
}

// A CAR'S BODY on the plan, for the overlap check.
struct Footprint {
    CarId car;
    engine::Vec2 rear, front;
    Real halfWidth = 0;
};
bool overlap(const Footprint& a, const Footprint& b) {
    const auto corners = [](const Footprint& f, engine::Vec2 out[4]) {
        engine::Vec2 d = f.front - f.rear;
        const Real len = d.length();
        d = len > 1e-9 ? d * (1.0 / len) : engine::Vec2(1, 0);
        const engine::Vec2 side(-d.y * f.halfWidth, d.x * f.halfWidth);
        out[0] = f.rear + side;
        out[1] = f.front + side;
        out[2] = f.front - side;
        out[3] = f.rear - side;
    };
    engine::Vec2 ca[4], cb[4];
    corners(a, ca);
    corners(b, cb);
    for (const engine::Vec2* poly : {ca, cb})
        for (int e = 0; e < 4; ++e) {
            const engine::Vec2 edge = poly[(e + 1) % 4] - poly[e];
            const engine::Vec2 axis(-edge.y, edge.x);
            Real loA = 1e30, hiA = -1e30, loB = 1e30, hiB = -1e30;
            for (int k = 0; k < 4; ++k) {
                const Real pa = ca[k].x * axis.x + ca[k].y * axis.y, pb = cb[k].x * axis.x + cb[k].y * axis.y;
                loA = std::min(loA, pa);
                hiA = std::max(hiA, pa);
                loB = std::min(loB, pb);
                hiB = std::max(hiB, pb);
            }
            if (hiA <= loB || hiB <= loA) return false;
        }
    return true;
}

// ONE SCENARIO: the roads, the model on them with the junction step, and what was seen while it ran.
struct Sim {
    TestRoads t;
    Driving d;
    VehicleClasses classes;
    bool ok = false;
    // Seen while running.
    std::set<std::pair<uint32_t, uint32_t>> overlapping;    // pairs of cars whose bodies overlapped on the plan
    std::map<uint32_t, Real> lowestSpeed, mostBraking;      // per car (braking: the most negative acceleration, until
                                                            // its front has left the junction for the lane beyond)
    std::map<uint32_t, uint64_t> enteredJunction;           // per car: the tick its front first came onto a connector
    std::map<uint32_t, bool> leftJunction;                  // per car: its front has come off a connector onto a lane
    std::map<uint32_t, bool> gotGoing, stoppedAfterMoving;  // per car: it reached 3 m/s; it came to rest after that
    std::set<uint32_t> dithered;                            // cars whose card was shown for changing their mind twice
    std::map<uint32_t, bool> restedAtLine;                  // per car: it came to rest at a junction's line
    std::vector<WaitLoop> loops;                            // every loop the wait-for graph showed
    uint64_t noReason = 0;
    uint64_t arrived = 0;

    Sim(TestRoads roads, const char* control, uint32_t decideEvery = DrivingSettings::kDefaultDecideEveryTicks)
        : t(std::move(roads)), d(&t.graph) {
        classes.add(driving_test::testCarClass());
        VehicleClass bus = deriveVehicleClass("test_bus", 12.0, 2.5, 6.0, 40);
        bus.tags = {driving_test::kTestVehicleTag};
        classes.add(bus);
        d.setVehicleClasses(classes);
        DrivingSettings s;
        s.decideEveryTicks = decideEvery;
        s.laneChanges = false;
        d.setSettings(s);
        ok = giveRules(control);
    }
    Sim(const Sim&) = delete;

    bool giveRules(const char* control) {
        FactRegistry facts;
        if (!registerBuiltInFacts(facts) || !registerMoveNeeds(facts)) return false;
        engine::ScriptVM vm;
        std::string err;
        RuleSet ruleSet;
        AnswerVocabulary vocabulary;
        CompiledRules rules;
        if (!vm.doString(rulesLua(control), &err) || !loadRuleSet(vm, kFile, ruleSet, &err)) {
            std::printf("    rule set: %s\n", err.c_str());
            return false;
        }
        std::vector<std::string> problems;
        if (!registerJunctionFacts(facts, ruleSet.ranks, &problems) ||
            !loadRules(vm, kFile, facts, ruleSet, vocabulary, rules, &err)) {
            std::printf("    rules: %s\n", err.c_str());
            return false;
        }
        RoadRules roadRules;
        if (!buildRoadRules(t.graph, ruleSet, facts, roadRules, &problems)) {
            for (const std::string& p : problems) std::printf("    %s\n", p.c_str());
            return false;
        }
        return d.useRules(ruleSet, facts, vocabulary, rules, roadRules, &err);
    }

    // A car of class `cls`, its front `along` metres along `lane`, bound for `dest` at `destAlong`.
    TripId add(uint32_t id, engine::PathId lane, Real along, engine::PathId dest, Real destAlong, uint32_t cls = 0,
               PersonId person = 0) {
        std::string why;
        if (!d.placeCar(CarId(id), cls, at(lane, along), {}, &why)) {
            std::printf("    car %u not placed: %s\n", id, why.c_str());
            CHECK(false);
            return TripId{};
        }
        const TripId trip = d.beginTrip(CarId(id), person ? person : id + 1, at(dest, destAlong));
        CHECK(trip.valid());
        return trip;
    }

    // One tick, and a look at everything.
    void step() {
        d.tick(kStep);
        // Cars that arrived are taken off the road (they would stand on the lane, parked, in everyone's way).
        std::vector<CarId> parked;
        for (const Event& e : d.drainEvents())
            if (e.kind == EventKind::Parked) {
                ++arrived;
                parked.push_back(e.car);
            }
        for (const CarId p : parked) d.removeCar(p);
        std::vector<Footprint> bodies;
        for (const Car& c : d.cars()) {
            const uint32_t id = c.id.index;
            const bool onConnector = t.graph.paths[c.path.index].kind == engine::PathKind::Connector;
            if (!onConnector && enteredJunction.count(id)) leftJunction[id] = true;
            if (!lowestSpeed.count(id) || c.speed < lowestSpeed[id]) lowestSpeed[id] = c.speed;
            if (!leftJunction[id] && (!mostBraking.count(id) || c.acceleration < mostBraking[id]))
                mostBraking[id] = c.acceleration;
            if (onConnector && !enteredJunction.count(id)) enteredJunction[id] = d.tickCount();
            if (c.speed > 3) gotGoing[id] = true;
            if (gotGoing[id] && !leftJunction[id] && c.speed < kStandingBelow) stoppedAfterMoving[id] = true;
            const JunctionPlans* plans = d.junctionPlans(c.id);
            for (std::size_t k = 0; plans && k < plans->count; ++k) {
                if (plans->plans[k].cameToRest) restedAtLine[id] = true;
                // RT_GIVE_WAY_DITHER: the card of a driver that changed its mind twice, once.
                if (plans->plans[k].mindChanges >= 2 && std::getenv("RT_GIVE_WAY_DITHER") && dithered.insert(id).second)
                    std::printf("%s", d.describeCar(c.id).c_str());
            }
            const CarPose p = carPose(t.graph, d.vehicleClasses(), d.ruleSet(), c);
            Footprint f;
            f.car = c.id;
            f.rear = engine::Vec2(p.rear.x, p.rear.z);
            f.front = engine::Vec2(p.front.x, p.front.z);
            f.halfWidth = d.vehicleClasses().at(c.vehicleClass).width * 0.5;
            bodies.push_back(f);
        }
        for (std::size_t a = 0; a < bodies.size(); ++a)
            for (std::size_t b = a + 1; b < bodies.size(); ++b) {
                if (((bodies[a].front - bodies[b].front).length()) > 30) continue;
                if (overlap(bodies[a], bodies[b])) overlapping.insert({bodies[a].car.index, bodies[b].car.index});
            }
        // RT_GIVE_WAY_WATCH=<car>,<from s>,<to s>: that car's card and every announcement on its connector's crossing
        // points, every tick of that window (for a person chasing one decision).
        if (const char* watch = std::getenv("RT_GIVE_WAY_WATCH")) {
            unsigned car = 0;
            double from = 0, to = 0;
            const Real now = static_cast<Real>(d.clockSeconds());
            if (std::sscanf(watch, "%u,%lf,%lf", &car, &from, &to) == 3 && now >= from && now <= to && d.car(CarId(car))) {
                std::printf("---- t=%.3f\n%s", now, d.describeCar(CarId(car)).c_str());
                const JunctionPlans* plans = d.junctionPlans(CarId(car));
                for (std::size_t k = 0; plans && k < plans->count; ++k) {
                    const engine::Path& p = t.graph.paths[plans->plans[k].connector.index];
                    for (uint32_t x = p.crossings.first; x < p.crossings.end(); ++x)
                        for (const Announcement& a : d.board().on(t.graph.pathCrossings[x]))
                            std::printf("  board: crossing %u car %llu path %u arrive %.2f clear %.2f going %d fixed %d at "
                                        "tick %llu notGoing %d\n",
                                        a.crossing.index, static_cast<unsigned long long>(a.who), a.onPath.index,
                                        a.arriveIn, a.clearIn, a.going, a.arrivalFixed,
                                        static_cast<unsigned long long>(a.arrivalTick), a.notGoing);
                }
            }
        }
        if (d.tickCount() % 60 == 0) {
            const WaitReport r = d.waitReport();
            noReason += r.noReason.size();
            for (const WaitLoop& l : r.loops) loops.push_back(l);
            // RT_GIVE_WAY_DEBUG=<seconds>: every driven car's card at that second, for a person to read.
            const char* debug = std::getenv("RT_GIVE_WAY_DEBUG");
            if (debug && static_cast<uint64_t>(std::atof(debug) * 60 + 0.5) == d.tickCount()) {
                for (const Car& c : d.cars())
                    if (c.driven) std::printf("%s", d.describeCar(c.id).c_str());
                std::printf("%s", d.describeWaiting(r).c_str());
            }
        }
    }
    void run(Real seconds) {
        const int steps = static_cast<int>(seconds / kStep + 0.5);
        for (int i = 0; i < steps; ++i) step();
    }
    // The cars in the order their fronts first came into a junction, with the tick.
    std::vector<std::pair<uint64_t, uint32_t>> entryOrder() const {
        std::vector<std::pair<uint64_t, uint32_t>> out;
        for (const std::pair<const uint32_t, uint64_t>& e : enteredJunction) out.push_back({e.second, e.first});
        std::sort(out.begin(), out.end());
        return out;
    }
    void report(const char* name) const {
        std::printf("    %s: %zu pairs overlapped, %zu loops, %llu waiting with no reason, %llu arrived; give way: %llu go, "
                    "%llu give way, %llu changed their mind twice\n",
                    name, overlapping.size(), loops.size(), static_cast<unsigned long long>(noReason),
                    static_cast<unsigned long long>(arrived),
                    static_cast<unsigned long long>(d.junctionPlanCounts().decidedGo),
                    static_cast<unsigned long long>(d.junctionPlanCounts().decidedGiveWay),
                    static_cast<unsigned long long>(d.junctionPlanCounts().changedMindTwice));
        for (const std::pair<uint32_t, uint32_t>& p : overlapping) std::printf("      cars %u and %u overlapped\n", p.first, p.second);
    }
    // No overlap, no loop, nobody waiting for no reason, and nobody changing their mind twice on one approach -- or, in
    // long random traffic (`mindChangesPerHundred` above zero), no more than that many in a hundred of the cars that
    // arrived: the hard gap catching an estimate that turned out wrong (a car further back whose guess jumped as the car
    // in front crossed) is the design working, not dithering, but it must stay rare.
    void checkClean(double mindChangesPerHundred = 0) const {
        CHECK(overlapping.empty());
        CHECK(loops.empty());
        CHECK(noReason == 0);
        const double twice = static_cast<double>(d.junctionPlanCounts().changedMindTwice);
        CHECK(twice <= mindChangesPerHundred * static_cast<double>(arrived) / 100.0);
    }
};

// A car `toLine` metres short of junction (r, c)'s line arriving heading `in`, bound `toward`: 10 m short of the end of
// the lane leaving that way, so it is well clear of the junction before it slows to park.
TripId approach(Sim& s, uint32_t id, uint32_t r, uint32_t c, Heading in, Real toLine, Heading toward, uint32_t cls = 0,
                PersonId person = 0) {
    const engine::PathId lane = s.t.laneArriving(r, c, in, 0);
    const engine::PathId out = s.t.laneLeaving(r, c, toward, 0);
    return s.add(id, lane, s.t.graph.paths[lane.index].length - toLine, out, s.t.graph.paths[out.index].length - 10, cls,
                 person);
}

// One number over every car's record (not the trips' handles, which the order of beginTrip numbers).
uint64_t carsHash(const Driving& d) {
    engine::bundle::Fnv h;
    for (const Car& c : d.cars()) {
        h.pod(c.id.index);
        h.pod(c.path.index);
        h.pod(c.along);
        h.pod(c.speed);
        h.pod(c.odometer);
        h.pod(static_cast<uint8_t>(c.binding.by));
        h.pod(c.binding.car.index);
    }
    return h.h;
}

}  // namespace

// ---- 1b. A queue's following interval ------------------------------------------------------------------------------

TEST_CASE(give_way_1b_a_queue_announces_one_behind_another) {
    // Five cars queued at rest on the road arriving at the four-way from the west, all going straight on: the fifth's
    // announcement on the first crossing point is no earlier than the fourth's plus a following interval, and so on.
    Sim s(driving_test::gridRoads(3, 3), kUncontrolled, 1);
    CHECK(s.ok);
    for (uint32_t i = 0; i < 5; ++i) approach(s, i, 1, 1, kEast, 1 + i * 6.5, kEast);
    s.run(2 * kStep);
    const engine::PathId conn = s.t.connector(1, 1, kEast, 0, kEast, 0);
    const engine::Path& p = s.t.graph.paths[conn.index];
    int checked = 0;
    for (uint32_t k = p.crossings.first; k < p.crossings.end() && checked == 0; ++k) {
        const engine::CrossingId cid = s.t.graph.pathCrossings[k];
        if (s.t.graph.crossings[cid.index].meeting == engine::Meeting::Split) continue;
        std::vector<Real> arrive;
        for (uint32_t i = 0; i < 5; ++i) {
            const Announcement* a = s.d.board().find(cid, i, conn);
            arrive.push_back(a ? a->arriveIn : kNever);
        }
        std::printf("    the queue's arrivals at crossing point %u:", cid.index);
        for (const Real t : arrive) std::printf(" %.2f", t);
        std::printf(" s\n");
        // A car within the look-ahead time announces; each one at least a following interval (gap_kept's low) after the
        // one in front.
        for (uint32_t i = 1; i < 5; ++i)
            if (std::isfinite(arrive[i])) CHECK(arrive[i] >= arrive[i - 1] + 1.2);
        CHECK(std::isfinite(arrive[0]) && std::isfinite(arrive[1]));
        ++checked;
    }
    CHECK(checked == 1);
}

// ---- 2. Order does not matter --------------------------------------------------------------------------------------

TEST_CASE(give_way_2_order_written_and_placed_changes_nothing) {
    // The board: the same announcements added in two orders file to the same board.
    AnnouncementBoard a, b;
    a.begin(7, 10);
    b.begin(7, 10);
    std::vector<Announcement> all;
    for (uint32_t i = 0; i < 12; ++i) {
        Announcement x;
        x.crossing = engine::CrossingId((i * 7) % 10);
        x.onPath = engine::PathId(i % 3);
        x.arriveIn = static_cast<Real>((i * 5) % 4);
        x.who = 100 - i;
        all.push_back(x);
    }
    for (const Announcement& x : all) a.add(x);
    for (std::size_t i = all.size(); i-- > 0;) b.add(all[i]);
    a.seal();
    b.seal();
    CHECK(boardHash(a) == boardHash(b));
    CHECK(a.on(engine::CrossingId(3)).size() == b.on(engine::CrossingId(3)).size());
    // The cars: eight cars at the middle four-way, placed and sent off in two different orders; after 40 s every car is
    // in the same place.
    const auto runOrder = [](const std::vector<uint32_t>& order) {
        Sim s(driving_test::gridRoads(3, 3), kUncontrolled);
        CHECK(s.ok);
        const Heading in[] = {kEast, kNorth, kWest, kSouth};
        for (const uint32_t i : order) {
            const Heading h = in[i % 4];
            approach(s, i, 1, 1, h, 20 + 9 * (i / 4), i < 4 ? h : driving_test::farTurnFrom(h, DrivesOn::Left));
        }
        s.run(40);
        s.checkClean();
        return std::make_pair(carsHash(s.d), s.d.stateHash());
    };
    const std::pair<uint64_t, uint64_t> one = runOrder({0, 1, 2, 3, 4, 5, 6, 7});
    const std::pair<uint64_t, uint64_t> two = runOrder({5, 2, 7, 0, 3, 6, 1, 4});
    CHECK(one.first == two.first);
    const std::pair<uint64_t, uint64_t> again = runOrder({0, 1, 2, 3, 4, 5, 6, 7});
    CHECK(one.second == again.second);
}

// ---- 3. Two cars on crossing paths -----------------------------------------------------------------------------------

TEST_CASE(give_way_3_crossing_paths_one_gives_way_the_other_does_not_slow) {
    Sim s(driving_test::gridRoads(3, 3), kUncontrolled);
    CHECK(s.ok);
    // Straight on from the south (car 0) and straight on from the west (car 1), both setting off from rest; car 0 is
    // a little nearer its line, so it arrives first; car 1 arrives a moment later and must give way.
    approach(s, 0, 1, 1, kNorth, 30, kNorth);
    approach(s, 1, 1, 1, kEast, 30.5, kEast);
    bool gaveWay = false;
    for (int i = 0; i < static_cast<int>(30 / kStep); ++i) {
        s.step();
        const JunctionPlans* plans = s.d.junctionPlans(CarId(1));
        if (plans && plans->count > 0 && plans->plans[0].decision == GiveWayDecision::GiveWay &&
            plans->plans[0].judged.other == CarId(0))
            gaveWay = true;
    }
    s.report("crossing paths");
    s.checkClean();
    CHECK(gaveWay);                                 // the one that should give way did, to the other
    std::printf("    the one giving way %s at its line\n", s.restedAtLine[1] ? "came to rest" : "slowed, and went behind");
    CHECK(!s.restedAtLine[0]);
    CHECK(s.mostBraking[0] > -0.05);                // the other never braked
    CHECK(s.entryOrder().size() == 2 && s.entryOrder()[0].second == 0);
    CHECK(s.arrived == 2);
}

// ---- 4. A T with a stop sign: a full stop first, then gaps on the main road ------------------------------------------

TEST_CASE(give_way_4_side_road_stop_sign_queue_takes_gaps) {
    // Junction (0, 1) of a two-by-three grid is a T: its side road arrives heading south.
    TestRoads roads = driving_test::gridRoads(2, 3);
    standStopSign(roads.graph, roads.laneArriving(0, 1, kSouth, 0));
    Sim s(std::move(roads), kMinorStop);
    CHECK(s.ok);
    // A queue of four on the side road, each turning across the main road (its far turn); the main road's traffic from
    // both sides, a car every 8-14 s from each, straight on.
    const Heading far = driving_test::farTurnFrom(kSouth, DrivesOn::Left);
    for (uint32_t i = 0; i < 4; ++i) approach(s, i, 0, 1, kSouth, 2 + i * 7, far);
    uint32_t next = 10;
    Real nextEast = 0, nextWest = 2;
    std::set<uint32_t> mainRoad;
    for (int i = 0; i < static_cast<int>(120 / kStep); ++i) {
        const Real now = static_cast<Real>(s.d.clockSeconds());
        // A main-road car put down 90 m back when there is room.
        const auto put = [&](Heading h, Real& when) {
            if (now < when || now > 70) return;
            const engine::PathId lane = s.t.laneArriving(0, 1, h, 0);
            if (!carsOnStretch(s.d.pathLists(), lane, 0, 25).empty()) return;
            s.add(next, lane, 10, s.t.laneLeaving(0, 1, h, 0), 90);
            mainRoad.insert(next);
            ++next;
            when = now + 8 + static_cast<Real>((next * 37) % 7);
        };
        put(kEast, nextEast);
        put(kWest, nextWest);
        s.step();
    }
    s.report("stop sign T");
    s.checkClean();
    // Every side-road car came to a full stop at the line, and all four got out.
    for (uint32_t i = 0; i < 4; ++i) CHECK(s.restedAtLine[i]);
    CHECK(s.enteredJunction.count(3) == 1);
    // The main road was never stopped for them, nor made to brake harder than comfortably (a side-road car that takes a
    // gap ahead of one may make it ease off a little, as it pulls away in front).
    Real hardest = 0;
    for (const uint32_t m : mainRoad) {
        hardest = std::min(hardest, s.mostBraking[m]);
        CHECK(!s.stoppedAfterMoving[m]);
        CHECK(s.mostBraking[m] > -2.0);
    }
    std::printf("    the main road's hardest braking before it was through: %.2f m/s2\n", hardest);
    // The gaps the side road took lie between the floor and the full gap.
    const JunctionPlanCounts& pc = s.d.junctionPlanCounts();
    std::printf("    gaps taken: %llu, %.2f-%.2f s, the smallest needed %.2f s\n",
                static_cast<unsigned long long>(pc.wentAfterGivingWay), pc.smallestGapTaken, pc.largestGapTaken,
                pc.smallestGapNeeded);
    CHECK(pc.wentAfterGivingWay > 0);
    CHECK(pc.smallestGapNeeded >= kTestGapFloor - 1e-9 && pc.smallestGapNeeded <= kTestGapComfortable + 1e-9);
    CHECK(pc.smallestGapTaken >= pc.smallestGapNeeded - 1e-9);
}

// ---- 5. Two streams joining one lane ---------------------------------------------------------------------------------

TEST_CASE(give_way_5_a_join_zips_without_touching) {
    Sim s(driving_test::gridRoads(3, 3), kUncontrolled);
    CHECK(s.ok);
    // Into the lane leaving the middle junction heading east: straight on from the west, and the near turn from the
    // road whose near turn leads east.
    Heading fromNear = kNorth;
    for (const Heading h : {kNorth, kSouth})
        if (driving_test::nearTurnFrom(h, DrivesOn::Left) == kEast) fromNear = h;
    CHECK(driving_test::nearTurnFrom(fromNear, DrivesOn::Left) == kEast);
    for (uint32_t i = 0; i < 4; ++i) {
        approach(s, i, 1, 1, kEast, 20 + i * 8, kEast);
        approach(s, 10 + i, 1, 1, fromNear, 18 + i * 8, kEast);
    }
    s.run(120);
    s.report("join");
    s.checkClean();
    CHECK(s.arrived == 8);
}

// ---- 6. A far turn across oncoming traffic ---------------------------------------------------------------------------

TEST_CASE(give_way_6_far_turn_across_oncoming) {
    for (const char* control : {kUncontrolled, kLights}) {
        Sim s(driving_test::gridRoads(3, 3), control);
        CHECK(s.ok);
        // Car 0 turns across the road from the west; car 1 comes straight on from the east (the oncoming traffic). On a
        // green the far turn gives way to it whatever (the table, by movement): car 0 starts nearer its line. At the
        // uncontrolled junction who gets there first goes first: car 1 starts a little nearer. Either way car 0 waits for
        // car 1, which never slows.
        const bool lights = control == kLights;
        approach(s, 0, 1, 1, kEast, lights ? 14 : 21, driving_test::farTurnFrom(kEast, DrivesOn::Left));
        approach(s, 1, 1, 1, kWest, 20, kWest);
        s.run(30);
        s.report(control == kLights ? "far turn, lights (first phase)" : "far turn, uncontrolled");
        s.checkClean();
        CHECK(s.mostBraking[1] > -0.05);
        CHECK(s.entryOrder().size() == 2);
        CHECK(s.arrived == 2);
    }
}

// ---- 7. Four cars reaching a four-way in the same tick --------------------------------------------------------------

namespace {

std::vector<std::pair<uint64_t, uint32_t>> fourAtOnce(const char* control, Sim** keep = nullptr) {
    // Deciding every tick and one person at every wheel, so all four move alike to the tick.
    static std::unique_ptr<Sim> last;
    last.reset(new Sim(driving_test::gridRoads(3, 3), control, 1));
    Sim& s = *last;
    CHECK(s.ok);
    const Heading in[] = {kEast, kNorth, kWest, kSouth};
    for (uint32_t i = 0; i < 4; ++i) approach(s, i, 1, 1, in[i], 25, in[i], 0, 7);
    s.run(40);
    if (keep) *keep = &s;
    return s.entryOrder();
}

}  // namespace

TEST_CASE(give_way_7_four_cars_reach_a_four_way_in_the_same_tick) {
    for (const char* control : {kUncontrolled, kAllWayStop}) {
        Sim* s = nullptr;
        const std::vector<std::pair<uint64_t, uint32_t>> order = fourAtOnce(control, &s);
        s->report(control == kAllWayStop ? "four at an all-way stop" : "four at an uncontrolled four-way");
        s->checkClean();
        CHECK(order.size() == 4u);
        std::printf("    entered:");
        for (const std::pair<uint64_t, uint32_t>& e : order) std::printf(" car %u at tick %llu;", e.second, static_cast<unsigned long long>(e.first));
        std::printf("\n");
        if (control == kAllWayStop)
            for (uint32_t i = 0; i < 4; ++i) CHECK(s->restedAtLine[i]);
        // They did not all go at once: at least two different ticks.
        CHECK(order.size() == 4u && order.front().first != order.back().first);
        // The same order, to the tick, every run.
        CHECK(fourAtOnce(control) == order);
    }
}

// ---- 8. A long vehicle's tail -----------------------------------------------------------------------------------------

TEST_CASE(give_way_8_a_long_vehicle_tail_blocks_the_crossing) {
    Sim s(driving_test::gridRoads(3, 3), kUncontrolled);
    CHECK(s.ok);
    // A 12 m bus straight on from the south arrives first; a car straight on from the west must wait for its tail.
    approach(s, 0, 1, 1, kNorth, 26, kNorth, 1);
    approach(s, 1, 1, 1, kEast, 30, kEast);
    // Its announcement on their crossing point clears much later than it arrives: by its length at least.
    Real busSpan = 0;
    for (int i = 0; i < static_cast<int>(30 / kStep); ++i) {
        s.step();
        const engine::PathId conn = s.t.connector(1, 1, kNorth, 0, kNorth, 0);
        const engine::Path& p = s.t.graph.paths[conn.index];
        for (uint32_t k = p.crossings.first; k < p.crossings.end(); ++k) {
            const engine::CrossingId cid = s.t.graph.pathCrossings[k];
            const Announcement* a = s.d.board().find(cid, 0, conn);
            if (!a || s.t.graph.crossings[cid.index].meeting != engine::Meeting::Cross) continue;
            if (a->arriveIn > 0 && a->speedThere > 0) busSpan = std::max(busSpan, (a->clearIn - a->arriveIn) * a->speedThere);
        }
    }
    s.report("long vehicle");
    s.checkClean();
    std::printf("    the bus's announced span over a crossing point, at its speed through: %.1f m\n", busSpan);
    CHECK(busSpan >= 12.0);
    CHECK(s.d.junctionPlanCounts().decidedGiveWay >= 1u);   // the car gave way
    CHECK(s.entryOrder().size() == 2 && s.entryOrder()[0].second == 0);
}

// ---- 9. A car that can no longer stop is not stopped -------------------------------------------------------------------

TEST_CASE(give_way_9_a_car_that_cannot_stop_is_not_stopped) {
    TestRoadSpec spec;
    spec.roadLength = 200;
    Sim s(driving_test::gridRoads(3, 3, spec), kUncontrolled);
    CHECK(s.ok);
    // Car 0 comes straight on from the west at speed (from rest 190 m back). Car 1, straight on from the south, is held
    // at its line by the host until car 0 is 15 m from its line, too close to stop: then let go, it must give way.
    approach(s, 0, 1, 1, kEast, 190, kEast);
    const TripId held = approach(s, 1, 1, 1, kNorth, 0.5, kNorth);
    const engine::PathId lane1 = s.t.laneArriving(1, 1, kNorth, 0);
    CHECK(s.d.placeHold(held, 1, at(lane1, s.t.graph.paths[lane1.index].length)));
    const engine::PathId lane0 = s.t.laneArriving(1, 1, kEast, 0);
    bool released = false;
    Real speedAtRelease = 0;
    for (int i = 0; i < static_cast<int>(60 / kStep); ++i) {
        const Car* c0 = s.d.car(CarId(0));
        if (!released && c0 && c0->path == lane0 && s.t.graph.paths[lane0.index].length - c0->along < 15) {
            CHECK(s.d.releaseHold(held, 1));
            released = true;
            speedAtRelease = c0->speed;
            s.mostBraking[0] = 0;   // watch its braking from here on
        }
        s.step();
    }
    s.report("cannot stop");
    s.checkClean();
    CHECK(released);
    std::printf("    car 0 was doing %.1f m/s, 15 m from its line, when car 1 was let go\n", speedAtRelease);
    CHECK(speedAtRelease * speedAtRelease / (2 * 2.0) > 15);   // too fast to stop braking comfortably
    CHECK(s.mostBraking[0] > -0.05);
    CHECK(s.entryOrder().size() == 2 && s.entryOrder()[0].second == 0);
}

// ---- 10. A refused turn -----------------------------------------------------------------------------------------------

TEST_CASE(give_way_10_a_refused_turn) {
    const Heading far = driving_test::farTurnFrom(kEast, DrivesOn::Left);
    TestRoadSpec spec;
    TestRoadSpec::RefusedTurn refused;
    refused.row = 1;
    refused.col = 1;
    refused.in = kEast;
    refused.out = far;
    spec.refused.push_back(refused);
    Sim s(driving_test::gridRoads(3, 3, spec), kUncontrolled);
    CHECK(s.ok);
    // Car 0 is bound where only the refused turn leads directly: it goes another way, and gives way there as anyone;
    // cross traffic from both sides keeps coming.
    approach(s, 0, 1, 1, kEast, 15, far);
    approach(s, 1, 1, 1, kSouth, 20, kSouth);
    approach(s, 2, 1, 1, kNorth, 24, kNorth);
    s.run(150);
    s.report("refused turn");
    s.checkClean();
    CHECK(s.arrived == 3);
}

// ---- 11. A meeting filed under no junction -----------------------------------------------------------------------------

TEST_CASE(give_way_11_a_meeting_spanning_two_junctions) {
    // A crossing point between the straight-on connector of junction (1, 1) heading east and that of junction (1, 2)
    // heading west, filed under no junction, as ADR-0400's carry-overs are (two junctions' connectors crowded together).
    // The geometry is the test's: the record is what the model reads. Asked while driving, falling back on the lower path
    // id: the two cars are never on their stretches at once.
    TestRoads roads = driving_test::gridRoads(3, 4);
    const engine::PathId x = roads.connector(1, 1, kEast, 0, kEast, 0);
    const engine::PathId y = roads.connector(1, 2, kWest, 0, kWest, 0);
    engine::CrossingPoint rec;
    rec.a = x < y ? x : y;
    rec.other = (x < y ? y : x).index;
    rec.meeting = engine::Meeting::Cross;
    rec.aFrom = 4;
    rec.aTo = 9;
    rec.otherFrom = 4;
    rec.otherTo = 9;
    roads.graph.crossings.push_back(rec);
    engine::indexCrossings(roads.graph);
    Sim s(std::move(roads), kUncontrolled);
    CHECK(s.ok);
    approach(s, 0, 1, 1, kEast, 20, kEast);
    approach(s, 1, 1, 2, kWest, 20, kWest);
    // Both on their stretch at once (the body anywhere within it), in ticks.
    int both = 0;
    const auto onStretch = [&](const Car* c, engine::PathId p) {
        if (!c) return false;
        const Body body = layBody(s.t.graph, *c, s.d.vehicleClasses().at(c->vehicleClass).length);
        for (std::size_t k = 0; k < body.count; ++k)
            if (body.pieces[k].path == p && body.pieces[k].front > 4 && body.pieces[k].rear < 9) return true;
        return false;
    };
    for (int i = 0; i < static_cast<int>(30 / kStep); ++i) {
        s.step();
        both += onStretch(s.d.car(CarId(0)), x) && onStretch(s.d.car(CarId(1)), y);
    }
    s.report("across two junctions");
    s.checkClean();
    std::printf("    ticks with both on the meeting: %d\n", both);
    CHECK(both == 0);
    CHECK(s.arrived == 2);
}

// ---- 12. Right of way but held: "not going" ---------------------------------------------------------------------------

TEST_CASE(give_way_12_held_at_the_line_does_not_hold_up_cross_traffic) {
    Sim s(driving_test::gridRoads(3, 3), kUncontrolled);
    CHECK(s.ok);
    // Car 0 waits at its line with right of way (it arrived first) but is held there by the host (no room beyond, say:
    // task D5 will give that as a stop). It announces "not going", so car 1 crossing its path does not wait for it.
    const TripId held = approach(s, 0, 1, 1, kNorth, 0.5, kNorth);
    const engine::PathId lane0 = s.t.laneArriving(1, 1, kNorth, 0);
    CHECK(s.d.placeHold(held, 1, at(lane0, s.t.graph.paths[lane0.index].length)));
    s.run(2);
    approach(s, 1, 1, 1, kEast, 40, kEast);
    s.run(20);
    s.report("held, not going");
    s.checkClean();
    CHECK(s.mostBraking[1] > -0.05);   // car 1 never slowed
    CHECK(s.enteredJunction.count(1) == 1 && s.enteredJunction.count(0) == 0);
}

// ---- 13. Patience: gaps taken between the floor and the full gap ------------------------------------------------------

TEST_CASE(give_way_13_patience_gaps_between_floor_and_full) {
    // Many people, each waiting at a stop sign for a gap in a steady stream (a car every 4 s, 3.5 s apart at the
    // conflict: too small for the full gap, big enough once patience has shrunk it). Every gap taken is at least what the
    // driver needed then, and what it needed lies between the floor and the full gap.
    Real smallestNeeded = 1e9, largestNeeded = 0, smallestMargin = 1e9;
    int taken = 0;
    for (PersonId person = 1; person <= 12; ++person) {
        TestRoads roads = driving_test::gridRoads(2, 3);
        standStopSign(roads.graph, roads.laneArriving(0, 1, kSouth, 0));
        Sim s(std::move(roads), kMinorStop);
        CHECK(s.ok);
        uint32_t next = 10;
        Real nextAt = 0;
        for (int i = 0; i < static_cast<int>(100 / kStep); ++i) {
            const Real now = static_cast<Real>(s.d.clockSeconds());
            // The side-road car comes once the stream is flowing.
            if (i == static_cast<int>(15 / kStep))
                approach(s, 0, 0, 1, kSouth, 2, driving_test::farTurnFrom(kSouth, DrivesOn::Left), 0, person * 7919);
            const engine::PathId lane = s.t.laneArriving(0, 1, kEast, 0);
            if (now >= nextAt && now < 80 && carsOnStretch(s.d.pathLists(), lane, 0, 20).empty()) {
                s.add(next++, lane, 10, s.t.laneLeaving(0, 1, kEast, 0), 90);
                // Gaps of 4, 5.5 and 7 s in turn: some only big enough once patience has shrunk the gap needed.
                nextAt = now + 4 + 1.5 * static_cast<Real>((next + person) % 3);
            }
            s.step();
            if (s.enteredJunction.count(0)) break;
        }
        CHECK(s.overlapping.empty());
        const CarTrace* tr = s.d.trace(CarId(0));
        for (std::size_t k = 0; tr && k < tr->giveWayNoteCount(); ++k) {
            const GiveWayNote& n = tr->giveWayNoteOldestFirst(k);
            if (n.decision != GiveWayDecision::Go || n.judged.cause != GiveWayCause::Gap) continue;
            ++taken;
            smallestNeeded = std::min(smallestNeeded, n.judged.need);
            largestNeeded = std::max(largestNeeded, n.judged.need);
            smallestMargin = std::min(smallestMargin, n.judged.have - n.judged.need);
        }
    }
    std::printf("    %d gaps taken; needed %.2f-%.2f s; each at least what was needed by %.3f s\n", taken, smallestNeeded,
                largestNeeded, smallestMargin);
    CHECK(taken > 0);
    CHECK(smallestNeeded >= kTestGapFloor - 1e-9);
    CHECK(largestNeeded <= kTestGapComfortable + 1e-9);
    CHECK(smallestMargin >= -1e-9);
    CHECK(kTestGapFloor >= kTestGapHard);
}

// ---- 14. Ten minutes of random traffic at each kind of junction -------------------------------------------------------

namespace {

// Random traffic through junction (r, c) of `s` from the headings `ins`, each car bound out by a hashed heading, a new
// one at an arm every few seconds when there is room; arrived cars are taken off. `seconds` of it.
void randomTraffic(Sim& s, uint32_t r, uint32_t c, const std::vector<Heading>& ins, Real seconds) {
    uint32_t next = 0;
    std::map<int, Real> nextAt;
    const int steps = static_cast<int>(seconds / kStep);
    for (int i = 0; i < steps; ++i) {
        const Real now = static_cast<Real>(s.d.clockSeconds());
        for (std::size_t a = 0; a < ins.size(); ++a) {
            if (now < nextAt[static_cast<int>(a)]) continue;
            const engine::PathId lane = s.t.laneArriving(r, c, ins[a], 0);
            if (!carsOnStretch(s.d.pathLists(), lane, 0, 25).empty()) continue;
            const uint32_t pick = engine::bundle::fnv1a(&next, sizeof next, 0x51u) % 4;
            const Heading outs[] = {ins[a], driving_test::nearTurnFrom(ins[a], DrivesOn::Left),
                                    driving_test::farTurnFrom(ins[a], DrivesOn::Left), ins[a]};
            const engine::PathId out = s.t.laneLeaving(r, c, outs[pick], 0);
            if (!out.valid() || !s.t.connector(r, c, ins[a], 0, outs[pick], 0).valid()) {
                nextAt[static_cast<int>(a)] = now + 1;
                continue;
            }
            s.add(next, lane, 10, out, 90);
            ++next;
            nextAt[static_cast<int>(a)] = now + 3 + static_cast<Real>((next * 2654435761u) % 6);
        }
        s.step();
    }
}

}  // namespace

// In ten minutes of random traffic, at most this many in a hundred cars may change their minds twice on one approach
// (the hard gap catching a guess that turned out wrong; see Sim::checkClean). Measured: 0, 2 and 1 in about 180.
constexpr double kRandomMindChangesPerHundred = 1.5;

TEST_CASE(give_way_14_ten_minutes_of_random_traffic_at_each_kind) {
    const Real minutes = 10 * 60;
    {
        Sim s(driving_test::gridRoads(3, 3), kUncontrolled);
        CHECK(s.ok);
        randomTraffic(s, 1, 1, {kEast, kNorth, kWest, kSouth}, minutes);
        s.report("ten minutes, uncontrolled four-way");
        s.checkClean(kRandomMindChangesPerHundred);
        CHECK(s.arrived > 150);
    }
    {
        Sim s(driving_test::gridRoads(3, 3), kAllWayStop);
        CHECK(s.ok);
        randomTraffic(s, 1, 1, {kEast, kNorth, kWest, kSouth}, minutes);
        s.report("ten minutes, all-way stop");
        s.checkClean(kRandomMindChangesPerHundred);
        CHECK(s.arrived > 120);
    }
    {
        TestRoads roads = driving_test::gridRoads(2, 3);
        standStopSign(roads.graph, roads.laneArriving(0, 1, kSouth, 0));
        Sim s(std::move(roads), kMinorStop);
        CHECK(s.ok);
        randomTraffic(s, 0, 1, {kEast, kWest, kSouth}, minutes);
        s.report("ten minutes, a T with a stop sign");
        s.checkClean(kRandomMindChangesPerHundred);
        CHECK(s.arrived > 120);
    }
}

// ---- 15. The driver card and the wait-for graph -------------------------------------------------------------------------

TEST_CASE(give_way_15_the_card_and_the_wait_graph_name_who) {
    TestRoadSpec spec;
    spec.roadLength = 200;
    Sim s(driving_test::gridRoads(3, 3, spec), kUncontrolled);
    CHECK(s.ok);
    // Car 1 is held at its line until car 0, coming straight on at speed, can no longer stop; let go, it gives way,
    // standing at its line.
    approach(s, 0, 1, 1, kEast, 190, kEast);
    const TripId held = approach(s, 1, 1, 1, kNorth, 0.5, kNorth);
    const engine::PathId lane1 = s.t.laneArriving(1, 1, kNorth, 0), lane0 = s.t.laneArriving(1, 1, kEast, 0);
    CHECK(s.d.placeHold(held, 1, at(lane1, s.t.graph.paths[lane1.index].length)));
    bool released = false, sawArrow = false, sawCard = false;
    for (int i = 0; i < static_cast<int>(40 / kStep) && !(sawArrow && sawCard); ++i) {
        const Car* c0 = s.d.car(CarId(0));
        if (!released && c0 && c0->path == lane0 && s.t.graph.paths[lane0.index].length - c0->along < 25) {
            CHECK(s.d.releaseHold(held, 1));
            released = true;
        }
        s.step();
        const Car* c1 = s.d.car(CarId(1));
        if (!c1 || !isWaiting(*c1)) continue;
        const WaitReason why = waitReasonOf(*c1);
        if (why.sort == ReasonSort::Arrow && why.arrow == ArrowKind::GivingWay && why.toCar == CarId(0)) {
            sawArrow = true;
            const std::string waiting = s.d.describeWaiting(s.d.waitReport());
            CHECK(waiting.find("no reason") != std::string::npos);
        }
        const std::string card = s.d.describeCar(CarId(1));
        if (card.find("giving way to car 0") != std::string::npos && card.find("I need") != std::string::npos) {
            sawCard = true;
            std::printf("%s", card.c_str());
            CHECK(card.find("give-way decisions") != std::string::npos);
        }
    }
    CHECK(sawArrow);
    CHECK(sawCard);
}

#endif  // RT_ENABLE_SCRIPTING
