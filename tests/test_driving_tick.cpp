// The three-step tick (#253): cars speed up, follow, stop at places to stop by and arrive, and can never run into the
// car ahead. First the move step's formulas alone (act.h, no world); then whole runs of Driving on the test roads
// (driving_test_roads.h) with a rule set read from Lua the way a level reads one:
//   1. one car on an empty road reaches its top speed along the curve the formula predicts;
//   2. a car stops at a place to stop by, short of it by no more than 0.3 m, and never passes it;
//   3. twenty cars behind a leader braking as hard as it can: no two overlap, at every step and decide interval;
//   4. a thousand seeded runs behind a leader whose speed changes at random: the smallest gap is always above zero;
//   5. the same seed twice: the same state hash at every one of 10,000 ticks;
//   6. the cars placed (and their trips begun) in a shuffled order: the same result;
//   7. a car crosses ten 0.5 m paths in one tick and ends in the right place with the right tail;
//   8. a queue released from a hold empties at a steady interval per car (printed);
//   9. a trip across a grid through several junctions ends with a Parked event, the car still placed and undriven;
//  10. a hold stops a car (its record names the rule "held"), and releasing it lets the car go on;
//  11. a car stops behind a parked car at the standstill gap, its record naming that car, and stays put;
//  12. a way that ends with no rule to stop the car: it stops short of the end and never passes it.
// Every tick of every run is watched (13): no driven car at rest held back by nothing, no two bodies overlapping on
// any path, no speed below zero. Last, what a tick costs.

#include "test_framework.h"

#include "driving_test_roads.h"

#include "apps/driving/act.h"
#include "apps/driving/driving.h"
#include "apps/driving/measure.h"
#include "apps/driving/road_rules.h"
#include "hash.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr Real kEps = 1e-9;

// An ordinary car's figures (vehicle_class.h's unauthored defaults, which testCarClass keeps).
Follower ordinaryFollower() {
    Follower f;
    f.speedUp = kUnauthoredSpeedUp;
    f.brake = kUnauthoredBrake;
    f.hardestBrake = kUnauthoredHardestBrake;
    f.gapKept = 1.5;
    f.standstillGap = 2;
    return f;
}

}  // namespace

// ---- The formulas alone ------------------------------------------------------------------------------------------------

TEST_CASE(tick_formulas_free_road_easing_and_hard_limit) {
    const Follower f = ordinaryFollower();
    // The empty road: all of a at rest, nothing at the top speed; a top speed of zero is never divided by.
    CHECK_APPROX(freeRoadAcceleration(f, 0, 10), f.speedUp, kEps);
    CHECK_APPROX(freeRoadAcceleration(f, 10, 10), 0, kEps);
    CHECK_APPROX(freeRoadAcceleration(f, 5, 10), f.speedUp * (1 - std::pow(0.5, 4)), kEps);
    CHECK_APPROX(freeRoadAcceleration(f, 3, 0), -f.brake, kEps);
    CHECK_APPROX(freeRoadAcceleration(f, 0, 0), 0, kEps);
    // The gap wanted: s0 + vT + closing; never below s0 when the thing ahead pulls away.
    CHECK_APPROX(gapWanted(f, 0, 0), f.standstillGap, kEps);
    CHECK_APPROX(gapWanted(f, 10, 10), f.standstillGap + 10 * f.gapKept, kEps);
    CHECK_APPROX(gapWanted(f, 10, 0), f.standstillGap + 15 + 100 / (2 * std::sqrt(f.speedUp * f.brake)), kEps);
    CHECK_APPROX(gapWanted(f, 1, 30), f.standstillGap, kEps);
    // Easing behind something: the whole formula; a gap at or below zero brakes as hard as allowed; a tiny gap is
    // read at the floor (finite).
    const Real s = 30;
    const Real expected = f.speedUp * (1 - std::pow(10.0 / 13.4, 4) - std::pow(gapWanted(f, 10, 8) / s, 2));
    CHECK_APPROX(easingAcceleration(f, 10, 13.4, s, 8), expected, kEps);
    CHECK_APPROX(easingAcceleration(f, 10, 13.4, 0, 8), -f.hardestBrake, kEps);
    CHECK_APPROX(easingAcceleration(f, 10, 13.4, -1, 8), -f.hardestBrake, kEps);
    CHECK(std::isfinite(easingAcceleration(f, 1, 13.4, 1e-12, 0)));
    CHECK_APPROX(easingAcceleration(f, 1, 13.4, 1e-12, 0), easingAcceleration(f, 1, 13.4, kGapFloor, 0), kEps);
    // The hard limit: zero against a standing thing at zero gap; Krauss's own form when both brake alike; the
    // reaction time is the decide interval or the step, whichever is longer.
    CHECK_APPROX(safeSpeed(7, 0.1, 0, 0, 0), 0, kEps);
    CHECK_APPROX(safeSpeed(7, 0.1, -3, 0, 0), 0, kEps);
    const Real krauss = -7 * 0.1 + std::sqrt(0.7 * 0.7 + 12 * 12 + 2 * 7 * 20);
    CHECK_APPROX(safeSpeed(7, 0.1, 20, 12, 7), krauss, kEps);
    // From the safe speed, reacting for tau and then braking hard stops within the gap plus the other's stop.
    const Real v = safeSpeed(7, 0.5, 20, 0, 0);
    CHECK_APPROX(v * 0.5 + v * v / 14, 20, 1e-9);
    CHECK_APPROX(reactionTime(0.1, 1.0 / 60), 0.1, kEps);
    CHECK_APPROX(reactionTime(0.1, 0.5), 0.5, kEps);
    CHECK_APPROX(lookAheadDistance(0, 2), kLookAheadFloor, kEps);
    CHECK_APPROX(lookAheadDistance(20, 2), 100 + kLookAheadMargin, kEps);
}

TEST_CASE(tick_formulas_step_rest_creep_and_binding) {
    const Follower f = ordinaryFollower();
    const Real dt = 0.1, tau = 0.1;
    // Free road from rest: speeds up at a; nothing held it back.
    StepOutcome s = stepSpeed(f, 0, 13.4, kNoRule, nullptr, 0, dt, tau);
    CHECK_APPROX(s.speed, f.speedUp * dt, kEps);
    CHECK(s.binding.by == Limit::Nothing);
    // Cruising at the top speed: the top speed holds it, with the rule that gave it.
    s = stepSpeed(f, 13.4, 13.4, 7, nullptr, 0, dt, tau);
    CHECK_APPROX(s.speed, 13.4, kEps);
    CHECK(s.binding.by == Limit::TopSpeed && s.binding.rule == 7u);
    // A top speed of zero: at rest it stays at rest, held by the top speed.
    s = stepSpeed(f, 0, 0, 3, nullptr, 0, dt, tau);
    CHECK(s.speed == 0);
    CHECK(s.binding.by == Limit::TopSpeed && s.binding.rule == 3u);
    // At rest behind a standing car at exactly the standstill gap, and a little beyond it: no creeping.
    Obstacle car;
    car.kind = Limit::CarAhead;
    car.distance = f.standstillGap;
    car.car = CarId(9);
    car.hardestBrake = 7;
    s = stepSpeed(f, 0, 13.4, kNoRule, &car, 1, dt, tau);
    CHECK(s.speed == 0);
    CHECK(s.binding.by == Limit::CarAhead && s.binding.car == CarId(9));
    car.distance = f.standstillGap + 0.01;
    s = stepSpeed(f, 0, 13.4, kNoRule, &car, 1, dt, tau);
    CHECK(s.speed == 0);
    // A car placed overlapping the car ahead: brakes as hard as allowed, no NaN.
    car.distance = -0.5;
    s = stepSpeed(f, 10, 13.4, kNoRule, &car, 1, dt, tau);
    CHECK(std::isfinite(s.speed));
    CHECK_APPROX(s.speed, 10 - f.hardestBrake * dt, kEps);
    CHECK(s.binding.by == Limit::CarAhead);
    // Where the driver only stopped looking, the end of the way only caps: far away it changes nothing, close it holds
    // the car back.
    Obstacle end;
    end.kind = Limit::WayEnd;
    end.hardOnly = true;
    end.distance = 500;
    s = stepSpeed(f, 13.4, 13.4, kNoRule, &end, 1, dt, tau);
    CHECK_APPROX(s.speed, 13.4, kEps);
    CHECK(s.binding.by == Limit::TopSpeed);
    end.distance = 5;
    s = stepSpeed(f, 10, 13.4, kNoRule, &end, 1, dt, tau);
    CHECK(s.speed < 10);
    CHECK(s.binding.by == Limit::WayEnd);
    // Where the way truly ends, it is eased to like a place to stop by, and a car at rest just short of it stays.
    end.hardOnly = false;
    end.distance = 60;
    Obstacle stop = end;
    stop.kind = Limit::Stop;
    CHECK_APPROX(stepSpeed(f, 13.4, 13.4, kNoRule, &end, 1, dt, tau).speed,
                 stepSpeed(f, 13.4, 13.4, kNoRule, &stop, 1, dt, tau).speed, kEps);
    end.distance = 0.02;
    s = stepSpeed(f, 0, 13.4, kNoRule, &end, 1, dt, tau);
    CHECK(s.speed == 0 && s.binding.by == Limit::WayEnd);
}

#ifdef RT_ENABLE_SCRIPTING

#include "apps/driving/scripting/rule_set_lua.h"
#include "apps/driving/scripting/rules_lua.h"
#include "engine/scripting/script_vm.h"

using driving_test::Heading;
using driving_test::TestRoads;
using driving_test::TestRoadSpec;

namespace {

constexpr const char* kFile = "c6_test.lua";

// The rules the runs use, each one line of a rule set's `rules`.
constexpr const char* kRuleSpeedLimit = "{ name = 'speed limit', give = { top_speed = 'my_limit' } }";
constexpr const char* kRuleBend = "{ name = 'bend ahead', give = { top_speed = 'corner_speed' } }";
constexpr const char* kRuleLaneEnds = "{ name = 'lane ends', when = 'lane_ends_ahead', give = { stop_at = 'lane_end' } }";
constexpr const char* kRuleArrive = "{ name = 'arrive', give = { stop_at = 'destination' } }";
constexpr const char* kRuleHeld = "{ name = 'held', give = { stop_at = 'hold_ahead' } }";
const std::vector<const char*> kAllRules = {kRuleSpeedLimit, kRuleBend, kRuleLaneEnds, kRuleArrive, kRuleHeld};

constexpr Real kStandstillGap = 2.0;   // the test rule set's standstill_gap
constexpr Real kGapKept = 1.5;         // ... and everyone's gap_kept unless a spread is given

// The personal settings' spreads: everyone alike unless a run asks for variety.
struct Spread {
    double gapLow = kGapKept, gapUsual = kGapKept, gapHigh = kGapKept;
    double habitLow = 1, habitUsual = 1, habitHigh = 1;
};

std::string num(double v) { return std::to_string(v); }

// A whole rule set around `rules`. `withMoveNeeds` false leaves out what the move step reads.
std::string ruleSetLua(const std::vector<const char*>& rules, const Spread& s = Spread(), bool withMoveNeeds = true) {
    std::string lua = "rule_set = { name = 'c6_test', drives_on = 'left', ranks = { 'stop' }, devices = {}, numbers = { "
                      "comfortable_side_accel = { 2, unit = 'm/s2', source = 'test' }, ";
    if (withMoveNeeds) lua += "standstill_gap = { " + num(kStandstillGap) + ", unit = 'm', source = 'test' }, ";
    lua += "}, personality = { speed_habit = { usual = " + num(s.habitUsual) + ", low = " + num(s.habitLow) +
           ", high = " + num(s.habitHigh) + ", unit = 'ratio' }, ";
    if (withMoveNeeds)
        lua += "gap_kept = { usual = " + num(s.gapUsual) + ", low = " + num(s.gapLow) + ", high = " + num(s.gapHigh) +
               ", unit = 's' }, ";
    lua += "}, rules = { ";
    for (const char* r : rules) lua += std::string(r) + ", ";
    return lua + "} }";
}

// A rule set read and compiled as a level would: the built-in facts, the move step's needs, the Lua readers.
struct TestRules {
    RuleSet ruleSet;
    FactRegistry facts;
    AnswerVocabulary vocab;
    CompiledRules compiled;
    bool ok = false;

    explicit TestRules(const std::string& lua) {
        CHECK(registerBuiltInFacts(facts));
        CHECK(registerMoveNeeds(facts));
        engine::ScriptVM vm;
        std::string err;
        if (!vm.doString(lua, &err) || !loadRuleSet(vm, kFile, ruleSet, &err) ||
            !loadRules(vm, kFile, facts, ruleSet, vocab, compiled, &err)) {
            std::printf("    rules: %s\n", err.c_str());
            return;
        }
        ok = true;
    }
    RuleIndex rule(const char* name) const { return compiled.findRule(name); }
};

// Binds the rules to the road and hands them to `d`.
bool giveRules(Driving& d, const engine::LaneGraph& g, const TestRules& r) {
    if (!r.ok) return false;
    RoadRules rr;
    std::vector<std::string> problems;
    const bool bound = buildRoadRules(g, r.ruleSet, r.facts, rr, &problems);
    for (const std::string& p : problems) std::printf("    %s\n", p.c_str());
    std::string why;
    const bool used = bound && d.useRules(r.ruleSet, r.facts, r.vocab, r.compiled, rr, &why);
    if (!why.empty()) std::printf("    useRules: %s\n", why.c_str());
    return used;
}

// The classes: 0 the ordinary test car (4.5 m); 1 a 20 m vehicle (an articulated bus or lorry); 2 one longer than the
// trail can hold over the shortest paths here (30 m over 0.5 m paths needs 60).
constexpr uint32_t kOrdinary = 0, kLong = 1, kOverlong = 2;
constexpr Real kLongLength = 20.0;
constexpr Real kOverlongLength = 30.0;
VehicleClass lengthened(const char* name, Real length) {
    VehicleClass c = driving_test::testCarClass();
    c.name = name;
    c.length = length;
    c.wheelbase = length / 2;   // only straight roads here: the turning circle is never asked
    return c;
}
VehicleClasses testClasses() {
    VehicleClasses c;
    c.add(driving_test::testCarClass());
    c.add(lengthened("test_long_vehicle", kLongLength));
    c.add(lengthened("test_overlong_vehicle", kOverlongLength));
    return c;
}

PathPoint at(engine::PathId path, Real along) {
    PathPoint p;
    p.path = path;
    p.along = along;
    return p;
}


// A STRAIGHT ROAD heading east, read as one line: a car's front is `x` metres from the west end.
struct Straight {
    TestRoads roads;
    uint32_t sections = 0;
    std::vector<Real> starts;         // per section: x at its west end; one more: the road's east end
    std::vector<int> sectionOfPath;   // per path: its section when it is the eastward lane, else -1

    Straight(uint32_t n, Real sectionLength) : Straight(std::vector<Real>(n, sectionLength)) {}
    explicit Straight(const std::vector<Real>& lengths)
        : roads(driving_test::straightRoadOf(lengths)), sections(static_cast<uint32_t>(lengths.size())) {
        starts.push_back(0);
        for (const Real l : lengths) starts.push_back(starts.back() + l);
        sectionOfPath.assign(roads.graph.paths.size(), -1);
        for (uint32_t i = 0; i < sections; ++i) sectionOfPath[lane(i).index] = static_cast<int>(i);
    }
    const engine::LaneGraph& graph() const { return roads.graph; }
    Real end() const { return starts.back(); }
    engine::PathId lane(uint32_t i) const { return roads.straightLane(i, true, 0); }
    // The section a front at `x` is on (a front exactly at a section's end is on that section).
    uint32_t sectionAt(Real x) const {
        const std::vector<Real>::const_iterator it = std::lower_bound(starts.begin() + 1, starts.end(), x);
        const uint32_t i = static_cast<uint32_t>(it - (starts.begin() + 1));
        return std::min(i, sections - 1);
    }
    PathPoint point(Real x) const {
        const uint32_t i = sectionAt(x);
        return at(lane(i), x - starts[i]);
    }
    // The earlier lanes a body `bodyLength` long with its front at `x` lies on, nearest first.
    std::vector<engine::PathId> trail(Real x, Real bodyLength) const {
        std::vector<engine::PathId> out;
        const uint32_t i = sectionAt(x);
        Real remaining = bodyLength - (x - starts[i]);
        for (uint32_t k = i; remaining > kEps && k > 0;) {
            out.push_back(lane(--k));
            remaining -= starts[k + 1] - starts[k];
        }
        return out;
    }
    // Where a car's front is, as x; -1 when it is not on the eastward lanes.
    Real x(const Car& c) const {
        const int i = c.path.index < sectionOfPath.size() ? sectionOfPath[c.path.index] : -1;
        return i < 0 ? Real(-1) : starts[static_cast<std::size_t>(i)] + c.along;
    }
    Real x(const Driving& d, uint32_t id) const {
        const Car* c = d.car(CarId(id));
        return c ? x(*c) : Real(-1);
    }
};

bool placeAt(Driving& d, const Straight& road, uint32_t id, Real x, uint32_t cls = kOrdinary) {
    std::string why;
    const bool ok = d.placeCar(CarId(id), cls, road.point(x), road.trail(x, d.vehicleClasses().at(cls).length), &why);
    if (!ok) std::printf("    placeCar %u at %.2f: %s\n", id, x, why.c_str());
    return ok;
}

// THE WATCH (done-when 13), looked at after every tick: no driven car at rest held back by nothing, no two bodies
// overlapping on a path, no speed below zero; and the smallest gap between two bodies on one path. It also reads
// Driving's cap counts: no tail ran off a trail and no look-ahead was cut short, unless a test allows tails.
constexpr Real kOverlapEps = 1e-9;
struct Watch {
    uint64_t looks = 0;
    uint64_t restingForNothing = 0;
    uint64_t overlaps = 0;
    uint64_t negativeSpeeds = 0;
    uint64_t tailsOffTrail = 0;
    uint64_t lookAheadsCut = 0;
    bool tailsAllowed = false;
    Real smallestGap = std::numeric_limits<Real>::infinity();
    std::string first;

    void look(const Driving& d) {
        ++looks;
        tailsOffTrail = d.capCounts().tailsOffTrail;
        lookAheadsCut = d.capCounts().lookAheadsCut;
        for (const Car& c : d.cars()) {
            if (c.speed < 0) note(negativeSpeeds, "car " + std::to_string(c.id.index) + " has a speed below zero");
            if (c.driven && c.speed == 0 && c.binding.by == Limit::Nothing)
                note(restingForNothing, "car " + std::to_string(c.id.index) + " is at rest held back by nothing");
            // The same, as the wait-for graph sees it (wait_graph.h): a waiting car with no reason would be on its
            // no-reason list.
            else if (isWaiting(c) && waitReasonOf(c).sort == ReasonSort::None)
                note(restingForNothing, "car " + std::to_string(c.id.index) + " is waiting for no reason");
        }
        const engine::LaneGraph& g = *d.roads();
        for (uint32_t p = 0; p < g.paths.size(); ++p) {
            const PathList list = d.pathLists().on(engine::PathId(p));
            for (std::size_t i = 0; i + 1 < list.size(); ++i) {
                if (list[i].car == list[i + 1].car) continue;
                const Real gap = list[i + 1].rear - list[i].front;
                smallestGap = std::min(smallestGap, gap);
                if (gap < -kOverlapEps)
                    note(overlaps, "cars " + std::to_string(list[i].car.index) + " and " +
                                       std::to_string(list[i + 1].car.index) + " overlap by " + std::to_string(-gap) +
                                       " m on path " + std::to_string(p));
            }
        }
    }
    void note(uint64_t& count, const std::string& what) {
        if (count == 0 && first.empty()) first = what + " (look " + std::to_string(looks) + ")";
        ++count;
    }
    bool clean() const {
        return restingForNothing == 0 && overlaps == 0 && negativeSpeeds == 0 && lookAheadsCut == 0 &&
               (tailsAllowed || tailsOffTrail == 0);
    }
    void check(const char* what) const {
        CHECK(clean());
        if (!clean())
            std::printf("    %s: %llu at rest for nothing, %llu overlaps, %llu negative speeds, %llu tails off a trail, "
                        "%llu look-aheads cut; first: %s\n",
                        what, static_cast<unsigned long long>(restingForNothing),
                        static_cast<unsigned long long>(overlaps), static_cast<unsigned long long>(negativeSpeeds),
                        static_cast<unsigned long long>(tailsOffTrail), static_cast<unsigned long long>(lookAheadsCut),
                        first.c_str());
    }
};

// Ticks `d` for `seconds` at `dt`, watching every tick.
void run(Driving& d, Real seconds, Real dt, Watch& w) {
    const int ticks = static_cast<int>(std::lround(seconds / dt));
    for (int t = 0; t < ticks; ++t) {
        d.tick(dt);
        w.look(d);
    }
}

// How many ticks a decide interval is at a step (at least one).
uint32_t ticksFor(Real interval, Real dt) {
    return std::max<uint32_t>(1, static_cast<uint32_t>(std::lround(interval / dt)));
}

void decideEvery(Driving& d, Real interval, Real dt) {
    DrivingSettings s;
    s.decideEveryTicks = ticksFor(interval, dt);
    d.setSettings(s);
}

// A Driving on `road` with the test classes and rules.
struct World {
    Driving d;
    explicit World(const engine::LaneGraph& g, const TestRules& rules) : d(&g) {
        d.setVehicleClasses(testClasses());
        CHECK(giveRules(d, g, rules));
    }
};

}  // namespace

// ---- The rules handed over ---------------------------------------------------------------------------------------------

TEST_CASE(tick_rules_need_gap_kept_and_standstill_gap) {
    const Straight road(2, 100);
    // A rule set without them: buildRoadRules names the move step and each.
    const TestRules lacking(ruleSetLua(kAllRules, Spread(), false));
    CHECK(lacking.ok);
    RoadRules rr;
    std::vector<std::string> problems;
    CHECK(!buildRoadRules(road.graph(), lacking.ruleSet, lacking.facts, rr, &problems));
    CHECK(problems.size() == 2u);
    for (const std::string& p : problems) {
        CHECK(p.find(kMoveReaderName) != std::string::npos);
        std::printf("      %s\n", p.c_str());
    }
    CHECK(problems.size() == 2u && problems[0].find(kNumberStandstillGap) != std::string::npos &&
          problems[1].find(kSettingGapKept) != std::string::npos);
    // ... and Driving refuses it rather than move cars by numbers it does not have.
    Driving d(&road.graph());
    std::string why;
    CHECK(!d.useRules(lacking.ruleSet, lacking.facts, lacking.vocab, lacking.compiled, rr, &why));
    CHECK(!d.hasRules() && !why.empty());
    // A registry without the built-in facts (no `arrived`) is refused too.
    const TestRules full(ruleSetLua(kAllRules));
    CHECK(!d.useRules(full.ruleSet, FactRegistry(), full.vocab, full.compiled, rr, &why));
    CHECK(why.find(kFactArrived) != std::string::npos);
    // Without rules, a driven car does not move.
    d.setVehicleClasses(testClasses());
    CHECK(placeAt(d, road, 1, 10));
    CHECK(d.beginTrip(CarId(1), 1, road.point(150)).valid());
    for (int t = 0; t < 10; ++t) d.tick(0.1);
    CHECK_APPROX(road.x(d, 1), 10, kEps);
    CHECK(d.car(CarId(1))->speed == 0);
}

// ---- 1. one car on an empty road -------------------------------------------------------------------------------------

TEST_CASE(tick_1_empty_road_reaches_top_speed_on_the_predicted_curve) {
    const Straight road(20, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    constexpr Real dt = 0.1;
    decideEvery(w.d, dt, dt);
    CHECK(placeAt(w.d, road, 1, 10));
    CHECK(w.d.beginTrip(CarId(1), 1, road.point(1990)).valid());
    // The curve: v' = v + a (1 - (v / v_top)^4) dt from rest, v_top the limit times a speed habit of 1.
    const VehicleClass& cls = w.d.vehicleClasses().at(kOrdinary);
    const Real top = road.roads.spec.speedLimit;
    Real predicted = 0, x = 10;
    Watch watch;
    uint32_t compared = 0;
    Real worst = 0;
    for (int t = 0; t < 600; ++t) {
        w.d.tick(dt);
        watch.look(w.d);
        predicted += cls.speedUp * (1 - std::pow(predicted / top, 4)) * dt;
        x += predicted * dt;
        const Car& c = *w.d.car(CarId(1));
        if (x < 1800) {
            worst = std::max(worst, std::fabs(c.speed - predicted));
            CHECK_APPROX(road.x(c), x, 1e-6);
            // (The first step starts at rest, where the top speed takes nothing off yet: nothing held it back.)
            if (t == 0) CHECK(c.binding.by == Limit::Nothing);
            else CHECK(c.binding.by == Limit::TopSpeed && c.binding.rule == rules.rule("speed limit"));
            ++compared;
        }
    }
    CHECK(compared > 400u);
    CHECK(worst < 1e-9);
    // 60 s on: within a hundredth of the top speed, and never above it.
    const Car& c = *w.d.car(CarId(1));
    CHECK(c.speed > 0.99 * top && c.speed <= top + kEps);
    std::printf("    after 60 s: %.4f m/s of %.4f; %u ticks on the predicted curve (worst %.2e m/s)\n", c.speed, top,
                compared, worst);
    watch.check("empty road");
}

// ---- 2. stopping at a place to stop by --------------------------------------------------------------------------------

TEST_CASE(tick_2_stops_short_of_a_stop_point_and_never_past) {
    const Straight road(6, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    const Real steps[] = {1.0 / 60, 1.0 / 30, 0.1, 0.5};
    const Real holdX = 300;
    for (const Real dt : steps) {
        World w(road.graph(), rules);
        decideEvery(w.d, 0.1, dt);
        CHECK(placeAt(w.d, road, 1, 10));
        const TripId trip = w.d.beginTrip(CarId(1), 1, road.point(590));
        CHECK(w.d.placeHold(trip, 1, road.point(holdX)));
        Watch watch;
        Real furthest = 0;
        const int ticks = static_cast<int>(std::lround(60 / dt));
        for (int t = 0; t < ticks; ++t) {
            w.d.tick(dt);
            watch.look(w.d);
            furthest = std::max(furthest, road.x(w.d, 1));
        }
        const Car& c = *w.d.car(CarId(1));
        const Real short_ = holdX - road.x(c);
        CHECK(furthest <= holdX + kEps);
        CHECK(c.speed == 0);
        CHECK(short_ >= -kEps && short_ <= 0.3);
        CHECK(c.binding.by == Limit::Stop && c.binding.rule == rules.rule("held"));
        std::printf("    step %.4f s: at rest %.4f m short of the hold\n", dt, short_);
        watch.check("stop point");
    }
}

// ---- 3. twenty followers behind a leader braking as hard as it can --------------------------------------------------

namespace {

void leaderBrakesHard(Real dt, Real decideInterval, const TestRules& rules, const Straight& road) {
    World w(road.graph(), rules);
    decideEvery(w.d, decideInterval, dt);
    constexpr uint32_t kCars = 21;   // the leader (id 0) and twenty followers
    TripId leader;
    for (uint32_t k = 0; k < kCars; ++k) {
        CHECK(placeAt(w.d, road, k, 400 - k * 10.0));
        const TripId t = w.d.beginTrip(CarId(k), 1000 + k, road.point(1490));
        CHECK(t.valid());
        if (k == 0) leader = t;
    }
    Watch watch;
    run(w.d, 40, dt, watch);
    // A hold just far enough ahead to stop for at the hardest braking, once the leader notices it (at its next
    // decision, up to one reaction time on).
    const Car& lead = *w.d.car(CarId(0));
    const Real tau = reactionTime(ticksFor(decideInterval, dt) * dt, dt);
    const Real hardest = w.d.vehicleClasses().at(kOrdinary).hardestBrake;
    const Real holdAt = road.x(lead) + 2 * lead.speed * tau + lead.speed * lead.speed / (2 * hardest) + 1;
    const Real speedBefore = lead.speed;
    CHECK(w.d.placeHold(leader, 1, road.point(holdAt)));
    Real hardestSeen = 0;
    // Long enough for the last of the twenty, still closing up from the start, to reach the queue and stop.
    const int ticks = static_cast<int>(std::lround(90 / dt));
    for (int t = 0; t < ticks; ++t) {
        w.d.tick(dt);
        watch.look(w.d);
        hardestSeen = std::min(hardestSeen, w.d.car(CarId(0))->acceleration);
    }
    bool allAtRest = true;
    for (const Car& c : w.d.cars()) {
        if (c.speed != 0)
            std::printf("    car %u not at rest: %.6f m/s, x %.3f, held back by %d (car %u)\n", c.id.index, c.speed,
                        road.x(c), static_cast<int>(c.binding.by), c.binding.car.index);
        allAtRest = allAtRest && c.speed == 0;
    }
    CHECK(allAtRest);
    CHECK(road.x(w.d, 0) <= holdAt + kEps);
    // It braked harder than comfortably, and no harder than it can.
    CHECK(hardestSeen < -w.d.vehicleClasses().at(kOrdinary).brake - 1);
    CHECK(hardestSeen >= -hardest - kEps);
    std::printf("    step %.4f s, deciding every %.2f s: leader from %.2f m/s, hardest braking %.2f m/s2; smallest "
                "gap %.3f m\n",
                dt, ticksFor(decideInterval, dt) * dt, speedBefore, -hardestSeen, watch.smallestGap);
    watch.check("leader braking hard");
    CHECK(watch.smallestGap > 0);
}

}  // namespace

TEST_CASE(tick_3_twenty_followers_behind_a_hard_stop_never_overlap) {
    const Straight road(15, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    const Real steps[] = {1.0 / 60, 1.0 / 30, 0.1, 0.5};
    const Real intervals[] = {0.1, 0.5};
    for (const Real dt : steps)
        for (const Real interval : intervals) leaderBrakesHard(dt, interval, rules, road);
}

// ---- 4 to 6. a leader whose speed changes at random --------------------------------------------------------------

namespace {

// A RANDOM-LEADER RUN: car 0 leads `followers` cars; every so often a hold is placed at a random distance ahead of it
// (sometimes closer than it can stop for, so it brakes as hard as it can and passes it) or released. The step, decide
// interval and the drivers (their gap kept and speed habit) all come from the seed. `placeOrder` / `tripOrder` give the
// order cars are placed / their trips begun (empty: by id).
struct RandomRun {
    uint32_t seed = 0;
    uint32_t followers = 5;
    Real dt = 0;
    uint32_t ticks = 0;
    std::vector<uint32_t> placeOrder;
    std::vector<uint32_t> tripOrder;
    std::vector<uint64_t>* hashes = nullptr;   // the state hash after every tick, when given
    std::vector<std::vector<Car>>* records = nullptr;   // every car's record after every tick, when given
};

constexpr uint32_t kHoldEveryTicks = 23;   // how often the leader's hold may change (ticks; prime, so it drifts)
constexpr Real kNearestHold = 2;           // a hold is placed this far ahead of the leader ...
constexpr Real kHoldReach = 60;            // ... plus up to this much

Real stepFor(uint32_t seed) {
    const Real steps[] = {1.0 / 60, 1.0 / 30, 0.1, 0.5};
    return steps[seed % 4];
}
Real intervalFor(uint32_t seed) { return (seed / 4) % 2 == 0 ? 0.1 : 0.5; }

void randomLeader(const RandomRun& r, const TestRules& rules, const Straight& road, Watch& watch) {
    World w(road.graph(), rules);
    decideEvery(w.d, intervalFor(r.seed), r.dt);
    const uint32_t cars = r.followers + 1;
    std::vector<uint32_t> placeOrder = r.placeOrder, tripOrder = r.tripOrder;
    for (uint32_t k = 0; r.placeOrder.empty() && k < cars; ++k) placeOrder.push_back(k);
    for (uint32_t k = 0; r.tripOrder.empty() && k < cars; ++k) tripOrder.push_back(k);
    // Spaced 9 to 20 m apart, from x = 300 back.
    std::vector<Real> xs(cars, 0);
    Real x = 300;
    for (uint32_t k = 0; k < cars; ++k) {
        xs[k] = x;
        x -= 9 + engine::unit24(engine::hashPair(r.seed, 100 + k)) * 11;
    }
    for (const uint32_t k : placeOrder) CHECK(placeAt(w.d, road, k, xs[k]));
    TripId leader;
    for (const uint32_t k : tripOrder) {
        const TripId t = w.d.beginTrip(CarId(k), static_cast<PersonId>(r.seed) * 1000 + k,
                                       road.point(road.end() - 10));
        CHECK(t.valid());
        if (k == 0) leader = t;
    }
    bool held = false;
    for (uint32_t t = 0; t < r.ticks; ++t) {
        if (t % kHoldEveryTicks == 0) {
            const uint32_t h = engine::hashPair(r.seed, t);
            if (held && (h & 1u)) {
                w.d.releaseHold(leader, 1);
                held = false;
            } else if (!held && (h & 2u)) {
                const Real ahead = kNearestHold + engine::unit24(engine::hashPair(h, 7)) * kHoldReach;
                const Real holdX = road.x(w.d, 0) + ahead;
                if (holdX < road.end() - 1) held = w.d.placeHold(leader, 1, road.point(holdX));
            }
        }
        w.d.tick(r.dt);
        watch.look(w.d);
        if (r.hashes) r.hashes->push_back(w.d.stateHash());
        if (r.records) r.records->push_back(w.d.cars());
    }
}

// The same records, field by field (what the move step writes, and the trip-independent rest).
bool sameRecords(const std::vector<Car>& a, const std::vector<Car>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Car& x = a[i];
        const Car& y = b[i];
        if (x.id != y.id || x.path != y.path || x.along != y.along || x.speed != y.speed ||
            x.acceleration != y.acceleration || x.odometer != y.odometer || x.held != y.held ||
            x.binding != y.binding || x.trailCount != y.trailCount || x.driven != y.driven)
            return false;
        for (std::size_t k = 0; k < x.trailCount; ++k)
            if (x.trail[k] != y.trail[k]) return false;
    }
    return true;
}

// A fixed shuffle of 0..n-1 from a seed (Fisher-Yates over hash.h).
std::vector<uint32_t> shuffled(uint32_t n, uint32_t seed) {
    std::vector<uint32_t> out(n);
    for (uint32_t i = 0; i < n; ++i) out[i] = i;
    for (uint32_t i = n; i > 1; --i) std::swap(out[i - 1], out[engine::hashPair(seed, i) % i]);
    return out;
}

}  // namespace

TEST_CASE(tick_4_a_thousand_random_leaders_smallest_gap_above_zero) {
    const Straight road(10, 100);
    Spread varied;
    varied.gapLow = 1.0;
    varied.gapUsual = 1.4;
    varied.gapHigh = 2.0;
    varied.habitLow = 0.9;
    varied.habitUsual = 1.0;
    varied.habitHigh = 1.15;
    const TestRules rules(ruleSetLua(kAllRules, varied));
    constexpr uint32_t kRuns = 1000;
    constexpr Real kSeconds = 40;
    Watch all;
    uint32_t clean = 0;
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    for (uint32_t seed = 1; seed <= kRuns; ++seed) {
        RandomRun r;
        r.seed = seed;
        r.dt = stepFor(seed);
        r.ticks = static_cast<uint32_t>(std::lround(kSeconds / r.dt));
        Watch watch;
        randomLeader(r, rules, road, watch);
        if (watch.clean() && watch.smallestGap > 0) ++clean;
        else if (all.first.empty()) all.first = "seed " + std::to_string(seed) + ": " + watch.first;
        all.smallestGap = std::min(all.smallestGap, watch.smallestGap);
        all.restingForNothing += watch.restingForNothing;
        all.overlaps += watch.overlaps;
        all.negativeSpeeds += watch.negativeSpeeds;
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    CHECK(clean == kRuns);
    CHECK(all.smallestGap > 0);
    std::printf("    %u runs (%.0f s each, steps 1/60 to 0.5 s, deciding every 0.1 or 0.5 s): smallest gap %.3f m, "
                "%u clean; %.0f ms\n",
                kRuns, kSeconds, all.smallestGap, clean, ms);
    all.check("random leaders");
}

TEST_CASE(tick_5_same_seed_same_hash_every_tick) {
    const Straight road(30, 100);
    Spread varied;
    varied.gapLow = 1.0;
    varied.gapHigh = 2.0;
    varied.habitLow = 0.9;
    varied.habitHigh = 1.15;
    const TestRules rules(ruleSetLua(kAllRules, varied));
    std::vector<uint64_t> first, second;
    for (int pass = 0; pass < 2; ++pass) {
        RandomRun r;
        r.seed = 5;
        r.followers = 9;
        r.dt = 1.0 / 60;
        r.ticks = 10000;
        r.hashes = pass == 0 ? &first : &second;
        Watch watch;
        randomLeader(r, rules, road, watch);
        watch.check("same seed");
    }
    CHECK(first.size() == 10000u);
    CHECK(first == second);
    // The hash moves: no two ticks in a row alike (the clock alone sees to it).
    uint32_t repeats = 0;
    for (std::size_t t = 1; t < first.size(); ++t)
        if (first[t] == first[t - 1]) ++repeats;
    CHECK(repeats == 0u);
}

TEST_CASE(tick_6_shuffled_storage_order_same_result) {
    const Straight road(10, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    constexpr uint32_t kFollowers = 9;
    RandomRun base;
    base.seed = 6;
    base.followers = kFollowers;
    base.dt = 1.0 / 30;
    base.ticks = 2000;
    std::vector<uint64_t> byId, placedShuffled;
    std::vector<std::vector<Car>> recordsById, recordsAllShuffled;
    {
        RandomRun r = base;
        r.hashes = &byId;
        r.records = &recordsById;
        Watch watch;
        randomLeader(r, rules, road, watch);
        watch.check("by id");
    }
    {
        // Placed in a shuffled order, trips begun by id: the very same state, hash for hash.
        RandomRun r = base;
        r.placeOrder = shuffled(kFollowers + 1, 61);
        r.hashes = &placedShuffled;
        Watch watch;
        randomLeader(r, rules, road, watch);
        watch.check("placed shuffled");
    }
    CHECK(byId == placedShuffled);
    {
        // Placed AND trips begun in shuffled orders: the trip handles differ (they are the host's), every car's
        // record does not.
        RandomRun r = base;
        r.placeOrder = shuffled(kFollowers + 1, 62);
        r.tripOrder = shuffled(kFollowers + 1, 63);
        r.records = &recordsAllShuffled;
        Watch watch;
        randomLeader(r, rules, road, watch);
        watch.check("all shuffled");
    }
    CHECK(recordsById.size() == recordsAllShuffled.size());
    uint32_t differing = 0;
    for (std::size_t t = 0; t < recordsById.size() && t < recordsAllShuffled.size(); ++t)
        if (!sameRecords(recordsById[t], recordsAllShuffled[t])) ++differing;
    CHECK(differing == 0u);
}

// ---- 7. ten short paths in one tick -------------------------------------------------------------------------------------

TEST_CASE(tick_7_crosses_ten_short_paths_in_one_tick) {
    constexpr Real kPath = 0.5;
    const Straight road(400, kPath);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    constexpr Real dt = 0.5;
    decideEvery(w.d, 0.1, dt);
    const Real startX = 5.0;
    const Real carLength = w.d.vehicleClasses().at(kOrdinary).length;   // an ordinary 4.5 m car: nine or ten paths
    CHECK(placeAt(w.d, road, 1, startX, kOrdinary));
    CHECK(w.d.beginTrip(CarId(1), 1, road.point(195)).valid());
    Watch watch;
    uint32_t mostCrossed = 0, ticksCrossingTen = 0;
    for (int t = 0; t < 40; ++t) {
        const Car before = *w.d.car(CarId(1));
        w.d.tick(dt);
        watch.look(w.d);
        const Car& c = *w.d.car(CarId(1));
        // The right place: the odometer and the place on the road agree.
        const Real x = startX + c.odometer;
        CHECK_APPROX(road.x(c), x, 1e-9);
        const uint32_t crossed = static_cast<uint32_t>(road.sectionOfPath[c.path.index] -
                                                       road.sectionOfPath[before.path.index]);
        mostCrossed = std::max(mostCrossed, crossed);
        if (crossed >= 10) ++ticksCrossingTen;
        // The right tail: the lanes the body still lies on behind its front, nearest first ...
        const std::vector<engine::PathId> expected = road.trail(road.x(c), carLength);
        CHECK(c.trailCount == expected.size());
        for (std::size_t k = 0; k < expected.size() && k < c.trailCount; ++k) CHECK(c.trail[k] == expected[k]);
        // ... each listed on its path as a tail, over the stretch the body covers there.
        Real remaining = carLength - c.along;
        for (std::size_t k = 0; k < c.trailCount; ++k) {
            bool listed = false;
            for (const PathEntry& e : w.d.pathLists().on(c.trail[k])) {
                if (e.car != c.id) continue;
                listed = e.why == OnPath::Tail && std::fabs(e.front - kPath) < kEps &&
                         std::fabs(e.rear - std::max(Real(0), kPath - remaining)) < 1e-9;
            }
            CHECK(listed);
            remaining -= kPath;
        }
    }
    CHECK(ticksCrossingTen > 0u);
    std::printf("    at most %u paths of %.1f m crossed in one %.1f s tick (%u ticks crossing ten or more)\n",
                mostCrossed, kPath, dt, ticksCrossingTen);
    watch.check("short paths");
}

// ---- 7b, 7c. long vehicles over short paths, and a follower behind ------------------------------------------------------

namespace {

// A LONG VEHICLE (id 1) and an ordinary car following it (id 2) over a run of short sections between two long ones.
// The vehicle is held at `holdX` until the follower has stopped behind it, then let go. Every tick the follower's
// front is checked against the vehicle's TRUE rear, worked out from where it started and its odometer (not from the
// lists). Returns the smallest gap seen between the two, and the gap when both stood.
struct LongRun {
    Real smallest = std::numeric_limits<Real>::infinity();
    Real standing = -1;
    CapCounts caps;   // the Driving's at the end
};
LongRun longVehicleAndFollower(uint32_t vehicleClass, Real shortPath, uint32_t shortCount, Real holdX, Watch& watch) {
    std::vector<Real> lengths(1, 100);
    lengths.insert(lengths.end(), shortCount, shortPath);
    lengths.push_back(200);
    const Straight road(lengths);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    constexpr Real dt = 1.0 / 60;
    const Real length = w.d.vehicleClasses().at(vehicleClass).length;
    const Real vehicleX = 95, followerX = vehicleX - length - 10;
    CHECK(placeAt(w.d, road, 1, vehicleX, vehicleClass));
    CHECK(placeAt(w.d, road, 2, followerX, kOrdinary));
    const TripId lead = w.d.beginTrip(CarId(1), 1, road.point(road.end() - 10));
    CHECK(lead.valid());
    CHECK(w.d.beginTrip(CarId(2), 2, road.point(road.end() - 40)).valid());
    CHECK(w.d.placeHold(lead, 1, road.point(holdX)));
    LongRun r;
    bool released = false;
    for (int t = 0; t < static_cast<int>(90 / dt); ++t) {
        w.d.tick(dt);
        watch.look(w.d);
        const Car& v = *w.d.car(CarId(1));
        const Car& f = *w.d.car(CarId(2));
        const Real trueFront = vehicleX + v.odometer;
        CHECK_APPROX(road.x(v), trueFront, 1e-6);
        const Real gap = (trueFront - length) - (followerX + f.odometer);
        r.smallest = std::min(r.smallest, gap);
        if (!released && v.speed == 0 && f.speed == 0 && f.odometer > 0) {
            r.standing = gap;
            released = w.d.releaseHold(lead, 1);
        }
    }
    CHECK(released);
    r.caps = w.d.capCounts();
    return r;
}

}  // namespace

TEST_CASE(tick_7b_twenty_metre_vehicle_over_short_paths_fits_its_trail) {
    std::printf("    a car's record is %zu bytes (a trail of %zu paths)\n", sizeof(Car), kMaxTrail);
    Watch watch;
    // 1.5 m paths: a 20 m body lies on 14 behind its front, inside the 16 kept. Held with its body over them.
    const LongRun r = longVehicleAndFollower(kLong, 1.5, 300, 300, watch);
    CHECK(r.smallest > 0);
    CHECK(r.standing >= kStandstillGap - 1e-6 && r.standing <= kStandstillGap + 0.3);
    std::printf("    20 m over 1.5 m paths: follower stood %.4f m behind its true rear, smallest gap %.4f m\n",
                r.standing, r.smallest);
    watch.check("20 m vehicle");   // tails not allowed: the counter stayed zero
}

TEST_CASE(tick_7c_body_longer_than_the_trail_is_counted_and_kept_clear) {
    Watch watch;
    watch.tailsAllowed = true;
    // 0.5 m paths: a 30 m body would lie on 60; the trail keeps 16. Held at 150 m, its whole body over short paths,
    // so the follower stops on a path the trail no longer records.
    const LongRun r = longVehicleAndFollower(kOverlong, 0.5, 300, 150, watch);
    CHECK(watch.tailsOffTrail > 0u);
    CHECK(r.caps.tailsOffTrail == watch.tailsOffTrail && r.caps.lastTailOffTrail == CarId(1));
    CHECK(r.caps.lookAheadsCut == 0u);
    CHECK(r.smallest > 0);
    CHECK(r.standing >= kStandstillGap - 1e-6 && r.standing <= kStandstillGap + 0.3);
    std::printf("    30 m over 0.5 m paths: %llu car-ticks with a tail off its trail; follower stood %.4f m behind the "
                "true rear, smallest gap %.4f m\n",
                static_cast<unsigned long long>(watch.tailsOffTrail), r.standing, r.smallest);
    watch.check("overlong vehicle");
}

// ---- 8. a queue released from a stop point -----------------------------------------------------------------------------

TEST_CASE(tick_8_released_queue_empties_at_a_steady_interval) {
    const Straight road(15, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    constexpr Real dt = 1.0 / 60;
    decideEvery(w.d, 0.1, dt);
    constexpr uint32_t kCars = 20;
    constexpr Real kLine = 300;
    TripId first;
    for (uint32_t k = 0; k < kCars; ++k) {
        CHECK(placeAt(w.d, road, k, kLine - 5 - k * 9.0));
        const TripId t = w.d.beginTrip(CarId(k), 2000 + k, road.point(1490));
        if (k == 0) first = t;
    }
    CHECK(w.d.placeHold(first, 1, road.point(kLine)));
    Watch watch;
    run(w.d, 90, dt, watch);
    // All queued at rest: the first at the line, each other behind the car ahead.
    for (const Car& c : w.d.cars()) {
        CHECK(c.speed == 0);
        if (c.speed != 0)
            std::printf("    car %u still moving at %.5f m/s, %.4f m behind the car ahead\n", c.id.index, c.speed,
                        c.binding.by == Limit::CarAhead
                            ? road.x(w.d, c.binding.car.index) - w.d.vehicleClasses().at(kOrdinary).length - road.x(c)
                            : -1.0);
    }
    CHECK(w.d.car(CarId(1))->binding.by == Limit::CarAhead && w.d.car(CarId(1))->binding.car == CarId(0));
    // Released: when does each front cross the line?
    CHECK(w.d.releaseHold(first, 1));
    std::vector<Real> crossedAt(kCars, -1);
    const double releasedAt = w.d.clockSeconds();
    for (int t = 0; t < static_cast<int>(60 / dt); ++t) {
        w.d.tick(dt);
        watch.look(w.d);
        for (uint32_t k = 0; k < kCars; ++k)
            if (crossedAt[k] < 0 && road.x(w.d, k) > kLine) crossedAt[k] = static_cast<Real>(w.d.clockSeconds() - releasedAt);
    }
    std::string line;
    std::vector<Real> intervals;
    for (uint32_t k = 0; k < kCars; ++k) {
        CHECK(crossedAt[k] >= 0);
        if (k > 0) {
            intervals.push_back(crossedAt[k] - crossedAt[k - 1]);
            char buf[32];
            std::snprintf(buf, sizeof buf, " %.2f", intervals.back());
            line += buf;
        }
    }
    // Steady: the last five intervals within a tenth of a second of their mean.
    Real mean = 0;
    for (std::size_t i = intervals.size() - 5; i < intervals.size(); ++i) mean += intervals[i] / 5;
    for (std::size_t i = intervals.size() - 5; i < intervals.size(); ++i) CHECK(std::fabs(intervals[i] - mean) < 0.1);
    std::printf("    first car over the line %.2f s after release; intervals (s):%s; steady %.2f s per car\n",
                crossedAt[0], line.c_str(), mean);
    watch.check("queue");
}

// ---- 9. a trip across a grid ---------------------------------------------------------------------------------------------

TEST_CASE(tick_9_grid_trip_corner_to_corner_parks) {
    const TestRoads t = driving_test::gridRoads(3, 3);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(t.graph, rules);
    constexpr Real dt = 1.0 / 60;
    std::string why;
    CHECK(w.d.placeCar(CarId(1), kOrdinary, at(t.laneLeaving(0, 0, Heading::East, 0), 10), {}, &why));
    const PathPoint destination = at(t.laneArriving(2, 2, Heading::North, 0), 90);
    const TripId trip = w.d.beginTrip(CarId(1), 1, destination);
    CHECK(trip.valid());
    const Route* route = w.d.route(trip);
    uint32_t junctions = 0;
    for (const RouteTurn& turn : route->turns)
        if (turn.junction.valid()) ++junctions;
    CHECK(junctions >= 3u);
    Watch watch;
    std::vector<uint32_t> connectorsDriven;
    std::vector<Event> events;
    Real topSpeedSeen = 0;
    for (int k = 0; k < static_cast<int>(200 / dt) && events.empty(); ++k) {
        w.d.tick(dt);
        watch.look(w.d);
        const Car& c = *w.d.car(CarId(1));
        topSpeedSeen = std::max(topSpeedSeen, c.speed);
        if (t.graph.paths[c.path.index].kind == engine::PathKind::Connector &&
            (connectorsDriven.empty() || connectorsDriven.back() != c.path.index))
            connectorsDriven.push_back(c.path.index);
        events = w.d.drainEvents();
    }
    CHECK(events.size() == 1u);
    CHECK(!events.empty() && events[0].kind == EventKind::Parked && events[0].car == CarId(1) &&
          events[0].trip == trip);
    CHECK(connectorsDriven.size() == junctions);
    // Still placed, undriven, at rest at its destination.
    const Car* c = w.d.car(CarId(1));
    CHECK(c != nullptr);
    CHECK(c && !c->driven && c->speed == 0);
    CHECK(c && c->path == destination.path && std::fabs(destination.along - c->along) <= kArrivedWithin);
    CHECK(w.d.tripCount() == 0u && w.d.trip(trip) == nullptr);
    std::printf("    through %zu junctions (top speed %.1f m/s), parked %.3f m short of the destination after %.1f s\n",
                connectorsDriven.size(), topSpeedSeen, c ? destination.along - c->along : -1.0, w.d.clockSeconds());
    // It stays put.
    const Car parked = *c;
    run(w.d, 5, dt, watch);
    CHECK(w.d.car(CarId(1))->along == parked.along && w.d.car(CarId(1))->path == parked.path);
    watch.check("grid trip");
}

// ---- 10. a hold, and its release ----------------------------------------------------------------------------------------

TEST_CASE(tick_10_hold_stops_names_held_release_goes_on) {
    const Straight road(6, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    constexpr Real dt = 1.0 / 60;
    CHECK(placeAt(w.d, road, 1, 10));
    const TripId trip = w.d.beginTrip(CarId(1), 1, road.point(590));
    Watch watch;
    run(w.d, 5, dt, watch);
    constexpr Real kHoldX = 250;
    CHECK(w.d.placeHold(trip, 42, road.point(kHoldX)));
    uint32_t atRestTicks = 0, namedHeld = 0;
    for (int k = 0; k < static_cast<int>(40 / dt); ++k) {
        w.d.tick(dt);
        watch.look(w.d);
        const Car& c = *w.d.car(CarId(1));
        CHECK(road.x(c) <= kHoldX + kEps);
        if (c.speed == 0) {
            ++atRestTicks;
            if (c.binding.by == Limit::Stop && c.binding.rule == rules.rule("held")) ++namedHeld;
        }
    }
    CHECK(atRestTicks > 0u && namedHeld == atRestTicks);
    CHECK(kHoldX - road.x(w.d, 1) <= 0.3);
    CHECK(w.d.releaseHold(trip, 42));
    run(w.d, 15, dt, watch);
    CHECK(road.x(w.d, 1) > kHoldX + 50);
    CHECK(w.d.car(CarId(1))->speed > 0);
    watch.check("hold");
}

// ---- 11. behind a parked car --------------------------------------------------------------------------------------------

TEST_CASE(tick_11_stops_behind_a_parked_car_and_stays_put) {
    const Straight road(10, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    constexpr Real dt = 1.0 / 60;
    constexpr Real kParkedX = 500;
    CHECK(placeAt(w.d, road, 2, kParkedX));   // nobody drives it
    CHECK(placeAt(w.d, road, 1, 10));
    CHECK(w.d.beginTrip(CarId(1), 1, road.point(990)).valid());
    Watch watch;
    run(w.d, 80, dt, watch);
    const Car& c = *w.d.car(CarId(1));
    const Real gap = kParkedX - w.d.vehicleClasses().at(kOrdinary).length - road.x(c);
    CHECK(c.speed == 0);
    CHECK(gap >= kStandstillGap - 1e-6 && gap <= kStandstillGap + 0.3);
    CHECK(c.binding.by == Limit::CarAhead && c.binding.car == CarId(2));
    std::printf("    at rest %.4f m behind the parked car (standstill gap %.1f m)\n", gap, kStandstillGap);
    // No creeping: two thousand ticks on, not a nanometre further.
    const Car stopped = c;
    run(w.d, 2000 * dt, dt, watch);
    CHECK(w.d.car(CarId(1))->along == stopped.along && w.d.car(CarId(1))->odometer == stopped.odometer);
    watch.check("parked car");
}

// ---- 12. a way that ends, with no rule for it --------------------------------------------------------------------------

TEST_CASE(tick_12_way_end_without_a_rule_is_a_wall) {
    const Straight road(3, 100);
    const TestRules rules(ruleSetLua({kRuleSpeedLimit, kRuleHeld}));   // no "lane ends", no "arrive"
    World w(road.graph(), rules);
    constexpr Real dt = 1.0 / 60;
    CHECK(placeAt(w.d, road, 1, 10));
    CHECK(w.d.beginTrip(CarId(1), 1, road.point(250)).valid());
    const Real end = road.end();
    Watch watch;
    Real furthest = 0;
    for (int k = 0; k < static_cast<int>(60 / dt); ++k) {
        w.d.tick(dt);
        watch.look(w.d);
        furthest = std::max(furthest, road.x(w.d, 1));
    }
    const Car& c = *w.d.car(CarId(1));
    CHECK(furthest <= end + kEps);
    CHECK(c.speed == 0 && c.driven);
    CHECK(c.binding.by == Limit::WayEnd);
    if (!(c.speed == 0 && c.driven && c.binding.by == Limit::WayEnd))
        std::printf("    speed %.6f, driven %d, held back by %d, path %u along %.6f\n", c.speed, c.driven ? 1 : 0,
                    static_cast<int>(c.binding.by), c.path.index, c.along);
    CHECK(w.d.drainEvents().empty());   // never arrived: it drove past its destination, as its rules let it
    std::printf("    at rest %.4f m short of the end of the road\n", end - road.x(c));
    watch.check("way end");
}

// ---- What a tick costs ------------------------------------------------------------------------------------------------

TEST_CASE(tick_timing_lists_and_a_thousand_driven_cars) {
    // Rebuilding every path's list (what each tick ends with), at 1,000 and 10,000 cars on a 20 by 20 grid.
    {
        const TestRoads t = driving_test::gridRoads(20, 20);
        const VehicleClasses classes = testClasses();
        std::vector<engine::PathId> lanes;
        for (uint32_t i = 0; i < t.graph.paths.size(); ++i)
            if (t.graph.paths[i].kind == engine::PathKind::Lane) lanes.push_back(engine::PathId(i));
        for (const uint32_t count : {1000u, 10000u}) {
            std::vector<Car> cars;
            for (uint32_t k = 0; k < count; ++k) {
                Car c;
                c.id = CarId(k);
                c.path = lanes[k % lanes.size()];
                c.along = 5 + static_cast<Real>(k / lanes.size()) * 12;
                cars.push_back(c);
            }
            PathLists lists;
            constexpr int kReps = 50;
            const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            for (int r = 0; r < kReps; ++r) rebuildPathLists(t.graph, classes, cars, lists);
            const double us =
                std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() / kReps;
            CHECK(lists.entries.size() >= count);
            std::printf("    rebuilding every path's list, %u cars on %zu paths: %.0f us\n", count,
                        t.graph.paths.size(), us);
        }
    }
    // One tick at 1,000 driven cars on a 10 by 10 grid, deciding every 6 ticks at 60 a second. (No junction logic
    // yet, so cars meeting at a junction are not watched here.)
    {
        const TestRoads t = driving_test::gridRoads(10, 10);
        const TestRules rules(ruleSetLua(kAllRules));
        World w(t.graph, rules);
        std::vector<engine::PathId> lanes;
        for (uint32_t i = 0; i < t.graph.paths.size(); ++i)
            if (t.graph.paths[i].kind == engine::PathKind::Lane) lanes.push_back(engine::PathId(i));
        constexpr uint32_t kCars = 1000;
        uint32_t trips = 0;
        for (uint32_t k = 0; k < kCars; ++k) {
            const engine::PathId lane = lanes[k % lanes.size()];
            CHECK(w.d.placeCar(CarId(k), kOrdinary, at(lane, 10 + static_cast<Real>(k / lanes.size()) * 30)));
            const engine::PathId to = lanes[engine::hashPair(253, k) % lanes.size()];
            if (w.d.beginTrip(CarId(k), k, at(to, 50)).valid()) ++trips;
        }
        constexpr Real dt = 1.0 / 60;
        constexpr int kWarm = 6, kTicks = 120;
        for (int k = 0; k < kWarm; ++k) w.d.tick(dt);
        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        for (int k = 0; k < kTicks; ++k) w.d.tick(dt);
        const double us =
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() / kTicks;
        uint32_t moving = 0;
        for (const Car& c : w.d.cars())
            if (c.speed > 0) ++moving;
        CHECK(trips == kCars);
        std::printf("    one tick, %u driven cars on a 10x10 grid (%zu paths): %.0f us (%u moving)\n", trips,
                    t.graph.paths.size(), us, moving);
    }
}

// ---- Seeing why (#256): the trace, the driver card and the wait-for graph on a live tick -----------------------------
//
// (The wait-for graph on hand-built lists, and the ring on its own, are in test_driving_trace.cpp.)
//   T1. a car drives off, catches a slower car, follows it and stops at a hold: its trace has one entry per change of
//       what limits it, in order, and none on the ticks between; ending the trip drops the trace;
//   T2. tracing on or off: the same state hash at every tick of 2,000 with 50 cars on a grid;
//   T3. the driver card for a free-road car, a following car, a car at a hold, a parked car and an unknown id;
//   T4. a closed ring full of cars at rest: one loop, gridlock, every car in it; with one car fewer the cars move up,
//       and no tick has a car waiting for no reason;
//   T5. a queue behind a held car: no loop, a chain as long as the queue, headed by the hold's rule by name.

namespace {

constexpr uint32_t kSlow = 3;            // a test class index: an ordinary car with a low top speed
constexpr Real kSlowTopSpeed = 6.0;      // m/s
VehicleClasses classesWithSlow() {
    VehicleClasses c = testClasses();
    VehicleClass slow = driving_test::testCarClass();
    slow.name = "test_slow_car";
    slow.topSpeed = kSlowTopSpeed;
    c.add(slow);
    return c;
}

// One change of what held a car back: the tick count after it, and the new binding.
struct Change {
    uint64_t tick = 0;
    Binding binding;
};

}  // namespace

TEST_CASE(trace_T1_one_entry_per_change_in_order) {
    const Straight road(10, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    w.d.setVehicleClasses(classesWithSlow());
    constexpr Real dt = 1.0 / 60;
    constexpr Real kHoldX = 700;
    CHECK(placeAt(w.d, road, 2, 200, kSlow));
    CHECK(placeAt(w.d, road, 1, 10));
    CHECK(w.d.beginTrip(CarId(2), 2, road.point(990)).valid());
    const TripId trip = w.d.beginTrip(CarId(1), 1, road.point(990));
    CHECK(trip.valid());
    CHECK(w.d.placeHold(trip, 7, road.point(kHoldX)));
    // Watch what limits car 1 after every tick, and keep each change.
    std::vector<Change> changes;
    Watch watch;
    for (int k = 0; k < static_cast<int>(150 / dt); ++k) {
        w.d.tick(dt);
        watch.look(w.d);
        const Car& c = *w.d.car(CarId(1));
        if (changes.empty() || changes.back().binding != c.binding) changes.push_back(Change{w.d.tickCount(), c.binding});
    }
    const Car& c = *w.d.car(CarId(1));
    CHECK(c.speed == 0 && c.binding.by == Limit::Stop && c.binding.rule == rules.rule("held"));
    const CarTrace* trace = w.d.trace(CarId(1));
    CHECK(trace != nullptr);
    if (!trace) return;
    std::string seen;
    for (const Change& ch : changes)
        seen += " " + std::to_string(static_cast<int>(ch.binding.by)) + "@" + std::to_string(ch.tick);
    std::printf("    changes (limit@tick):%s\n", seen.c_str());
    // The story, exactly: free road as it moves off, then the speed limit; the slower car once it comes in sight, far
    // off, eases it a little; the speed limit again (the easing formula weighs the two, and while the slower car is
    // still far the limit takes more off); the slower car for good as it closes; then the hold, once the slower car
    // has gone on past it.
    const std::vector<Binding> story = {
        Binding(),
        Binding{Limit::TopSpeed, CarId(), rules.rule("speed limit")},
        Binding{Limit::CarAhead, CarId(2), kNoRule},
        Binding{Limit::TopSpeed, CarId(), rules.rule("speed limit")},
        Binding{Limit::CarAhead, CarId(2), kNoRule},
        Binding{Limit::Stop, CarId(), rules.rule("held")},
    };
    CHECK(changes.size() == story.size());
    for (std::size_t i = 0; i < changes.size() && i < story.size(); ++i) CHECK(changes[i].binding == story[i]);
    // The first two come on the first two ticks.
    CHECK(changes.size() >= 2u && changes[0].tick == 1u && changes[1].tick == 2u);
    // The trace holds exactly those changes, at the ticks they happened, and nothing for the ticks between.
    CHECK(trace->written == changes.size());
    const std::size_t from = changes.size() > kTraceLength ? changes.size() - kTraceLength : 0;
    CHECK(trace->size() == changes.size() - from);
    for (std::size_t i = 0; i < trace->size() && from + i < changes.size(); ++i) {
        const TraceEntry& e = trace->oldestFirst(i);
        CHECK(e.tick == changes[from + i].tick && e.binding == changes[from + i].binding);
    }
    // The stop's entry carries the hold rule's reason, with the value it read.
    const TraceEntry* last = trace->latest();
    CHECK(last && last->why.rule == rules.rule("held") && last->why.kept > 0);
    std::printf("%s", w.d.describeCar(CarId(1)).c_str());
    watch.check("trace story");
    // Ending the trip drops the trace.
    CHECK(w.d.endTrip(trip));
    CHECK(w.d.trace(CarId(1)) == nullptr);
}

TEST_CASE(trace_T2_tracing_on_or_off_same_state_hash) {
    const TestRoads t = driving_test::gridRoads(4, 4);
    const TestRules rules(ruleSetLua(kAllRules));
    std::vector<engine::PathId> lanes;
    for (uint32_t i = 0; i < t.graph.paths.size(); ++i)
        if (t.graph.paths[i].kind == engine::PathKind::Lane) lanes.push_back(engine::PathId(i));
    constexpr uint32_t kCars = 50;
    constexpr int kTicks = 2000;
    constexpr Real dt = 1.0 / 60;
    std::vector<uint64_t> hashes[2];
    uint64_t traceHashes[2] = {0, 0};
    uint64_t entries[2] = {0, 0};
    for (int pass = 0; pass < 2; ++pass) {
        World w(t.graph, rules);
        DrivingSettings s;
        s.trace = pass == 0;
        w.d.setSettings(s);
        for (uint32_t k = 0; k < kCars; ++k) {
            const engine::PathId lane = lanes[k % lanes.size()];
            CHECK(w.d.placeCar(CarId(k), kOrdinary, at(lane, 10 + static_cast<Real>(k / lanes.size()) * 30)));
            const engine::PathId to = lanes[engine::hashPair(256, k) % lanes.size()];
            w.d.beginTrip(CarId(k), k, at(to, 50));
        }
        for (int k = 0; k < kTicks; ++k) {
            w.d.tick(dt);
            hashes[pass].push_back(w.d.stateHash());
        }
        traceHashes[pass] = w.d.traceHash();
        for (const Car& c : w.d.cars())
            if (const CarTrace* tr = w.d.trace(c.id)) entries[pass] += tr->written;
    }
    CHECK(hashes[0].size() == static_cast<std::size_t>(kTicks));
    CHECK(hashes[0] == hashes[1]);
    CHECK(entries[0] > 0u && entries[1] == 0u);
    CHECK(traceHashes[0] != traceHashes[1]);
    std::printf("    %u cars, %d ticks: the same state hash at every tick with tracing on and off (%llu entries "
                "written with it on)\n",
                kCars, kTicks, static_cast<unsigned long long>(entries[0]));
}

TEST_CASE(trace_T3_driver_cards) {
    const TestRules rules(ruleSetLua(kAllRules));
    constexpr Real dt = 1.0 / 60;
    const Straight road(6, 100);
    Watch watch;
    // A car on free road, at the speed limit.
    {
        World w(road.graph(), rules);
        CHECK(placeAt(w.d, road, 14, 10));
        CHECK(w.d.beginTrip(CarId(14), 5, road.point(590)).valid());
        run(w.d, 15, dt, watch);
        const std::string card = w.d.describeCar(CarId(14));
        std::printf("%s", card.c_str());
        CHECK(card == "car 14, class \"test_car\", driven by person 5, trip to lane 10 at 90.0 m (446 m, about 33 s "
                      "to go)\n"
                      "on lane 2 at 43.7 m, 13.2 m/s, speeding up\n"
                      "now: its top speed of 13.4 m/s, by rule \"speed limit\"\n"
                      "held: no stop; top speed 13.4 m/s (speed limit: my_limit is 13.4 m/s)\n"
                      "recent:\n"
                      "  t=0.0 s  free road; 0.0 m/s on lane 0 at 10.0 m\n"
                      "  t=0.0 s  a top speed (speed limit: my_limit is 13.4 m/s); 0.0 m/s on lane 0 at 10.0 m\n");
    }
    // A car following a slower one.
    {
        World w(road.graph(), rules);
        w.d.setVehicleClasses(classesWithSlow());
        CHECK(placeAt(w.d, road, 9, 60, kSlow));
        CHECK(placeAt(w.d, road, 14, 10));
        CHECK(w.d.beginTrip(CarId(9), 6, road.point(590)).valid());
        CHECK(w.d.beginTrip(CarId(14), 5, road.point(590)).valid());
        run(w.d, 30, dt, watch);
        const std::string card = w.d.describeCar(CarId(14));
        std::printf("%s", card.c_str());
        CHECK(card == "car 14, class \"test_car\", driven by person 5, trip to lane 10 at 90.0 m (379 m, about 28 s "
                      "to go)\n"
                      "on lane 4 at 10.8 m, 6.0 m/s, steady\n"
                      "now: following car 9, 11.2 m ahead, doing 6.0 m/s\n"
                      "held: no stop; top speed 13.4 m/s (speed limit: my_limit is 13.4 m/s)\n"
                      "recent:\n"
                      "  t=0.0 s  following car 9; 0.0 m/s on lane 0 at 10.0 m\n");
    }
    // A car held at a hold.
    {
        World w(road.graph(), rules);
        CHECK(placeAt(w.d, road, 14, 10));
        const TripId trip = w.d.beginTrip(CarId(14), 5, road.point(590));
        CHECK(w.d.placeHold(trip, 3, road.point(150)));
        run(w.d, 30, dt, watch);
        const std::string card = w.d.describeCar(CarId(14));
        std::printf("%s", card.c_str());
        CHECK(card == "car 14, class \"test_car\", driven by person 5, trip to lane 10 at 90.0 m (440 m, about 33 s "
                      "to go)\n"
                      "on lane 2 at 50.0 m, 0.0 m/s, at rest\n"
                      "now: a stop 0.0 m ahead, by rule \"held\"\n"
                      "held: stop in 0.0 m (held: hold_ahead is 0.0 m); top speed 13.4 m/s (speed limit: my_limit is "
                      "13.4 m/s)\n"
                      "recent:\n"
                      "  t=0.0 s  free road; 0.0 m/s on lane 0 at 10.0 m\n"
                      "  t=0.0 s  a top speed (speed limit: my_limit is 13.4 m/s); 0.0 m/s on lane 0 at 10.0 m\n"
                      "  t=10.4 s  a stop (held: hold_ahead is 66.5 m); 12.3 m/s on lane 0 at 83.7 m\n");
        // A parked car, and a car that is not there.
        CHECK(placeAt(w.d, road, 20, 400));
        const std::string parked = w.d.describeCar(CarId(20));
        const std::string unknown = w.d.describeCar(CarId(77));
        std::printf("%s%s", parked.c_str(), unknown.c_str());
        CHECK(parked == "car 20, class \"test_car\", parked on lane 6 at 100.0 m: nobody is driving it\n");
        CHECK(unknown == "no such car: 77 is not on the road in the new driving model\n");
    }
    watch.check("driver cards");
}

namespace {

// A RING of four 52 m sides (208 m round): room for exactly 32 ordinary cars bumper to bumper at the standstill gap.
constexpr uint32_t kRingSides = 4;
constexpr Real kRingSide = 52;
constexpr uint32_t kRingCars = 32;
constexpr Real kRingSpacing = kRingSide * kRingSides / kRingCars;   // 6.5 m: a 4.5 m car and the 2 m standstill gap

TestRoads ringOfFour() {
    TestRoadSpec spec;
    spec.roadLength = kRingSide;
    return driving_test::ringRoad(kRingSides, spec);
}

// Car k's front `k` spacings and one car length round the ring from the start of side 0; its trip is to the middle of
// the side two on from its own. Every car but `skip` (CarId::kNone: none skipped).
void fillRing(World& w, const TestRoads& ring, uint32_t skip) {
    const Real length = w.d.vehicleClasses().at(kOrdinary).length;
    for (uint32_t k = 0; k < kRingCars; ++k) {
        if (k == skip) continue;
        const Real s = k * kRingSpacing + length;
        const uint32_t side = static_cast<uint32_t>(s / kRingSide) % kRingSides;
        std::string why;
        CHECK(w.d.placeCar(CarId(k), kOrdinary, at(ring.ringLane(side, 0), s - side * kRingSide), {}, &why));
        if (!why.empty()) std::printf("    placeCar %u: %s\n", k, why.c_str());
        CHECK(w.d.beginTrip(CarId(k), k, at(ring.ringLane((side + 2) % kRingSides, 0), kRingSide / 2)).valid());
    }
}

}  // namespace

TEST_CASE(trace_T4_full_ring_is_one_gridlock_loop) {
    const TestRoads ring = ringOfFour();
    const TestRules rules(ruleSetLua(kAllRules));
    constexpr Real dt = 1.0 / 60;
    {
        World w(ring.graph, rules);
        fillRing(w, ring, CarId::kNone);
        Watch watch;
        run(w.d, 2, dt, watch);
        const WaitReport r = w.d.waitReport();
        CHECK(r.waiting == kRingCars && r.noReason.empty());
        CHECK(r.loops.size() == 1u && r.gridlocks == 1u && r.deadlocks == 0u);
        if (!r.loops.empty()) {
            CHECK(r.loops[0].cars.size() == kRingCars && r.loops[0].kind == LoopClass::Gridlock);
            for (uint32_t k = 0; k < r.loops[0].cars.size(); ++k) CHECK(r.loops[0].cars[k] == CarId(k));
        }
        const std::string text = w.d.describeWaiting(r);
        std::printf("%s", text.substr(0, text.find(':', text.find("loop 1")) + 1).c_str());
        std::printf(" ...\n");
        watch.check("full ring");
    }
    {
        // One car fewer: the gap travels back round the ring, every car moving up into it in turn.
        World w(ring.graph, rules);
        fillRing(w, ring, 0);
        Watch watch;
        uint64_t ticksWithNoReason = 0, ticksWithLoops = 0;
        for (int k = 0; k < static_cast<int>(30 / dt); ++k) {
            w.d.tick(dt);
            watch.look(w.d);
            const WaitReport r = w.d.waitReport();
            if (!r.noReason.empty()) ++ticksWithNoReason;
            if (!r.loops.empty()) ++ticksWithLoops;
        }
        uint32_t movedUp = 0;
        for (const Car& c : w.d.cars())
            if (c.odometer > 0) ++movedUp;
        CHECK(ticksWithNoReason == 0u);
        CHECK(ticksWithLoops == 0u);
        CHECK(movedUp >= kRingCars / 2);
        std::printf("    one car fewer: %u of %u cars moved up in 30 s; no tick with a car waiting for no reason, none "
                    "with a loop\n",
                    movedUp, kRingCars - 1);
        watch.check("ring with a gap");
    }
}

TEST_CASE(trace_T5_queue_behind_a_hold_is_a_chain) {
    const Straight road(6, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    constexpr Real dt = 1.0 / 60;
    constexpr uint32_t kQueue = 6;
    TripId head;
    for (uint32_t k = 0; k < kQueue; ++k) {
        CHECK(placeAt(w.d, road, k + 1, 200 - k * 15.0));
        const TripId t = w.d.beginTrip(CarId(k + 1), k + 1, road.point(590));
        if (k == 0) head = t;
    }
    CHECK(w.d.placeHold(head, 1, road.point(300)));
    Watch watch;
    run(w.d, 60, dt, watch);
    const WaitReport r = w.d.waitReport();
    CHECK(r.waiting == kQueue && r.noReason.empty() && r.loops.empty());
    CHECK(r.longestChain == kQueue && r.longestHead == CarId(1) && r.longestBack == CarId(kQueue));
    const std::string text = w.d.describeWaiting(r);
    std::printf("%s", text.c_str());
    CHECK(text.find("a stop by rule \"held\": 1 queue, 6 cars in all, the longest 6") != std::string::npos);
    watch.check("queue behind a hold");
}

// ---- The measurements table on a live tick (#257) ---------------------------------------------------------------------
//
// (The arithmetic on its own, with breaches staged by hand, is in test_driving_measure.cpp.)
//   M1. 60 cars on a grid driving trips (a new one each time one parks) for 3 simulated minutes: every row with a zero
//       target passes but two the model fails today (overlaps where two connectors join one lane, bends taken too
//       fast; see the test), which are only checked to be measured; the table is printed;
//   M2. a queue of 10 released from a hold on a straight road, a counting line at the hold: the zero-target rows pass,
//       and the queue figures are measured (and printed; both are outside the plan's bands today, see the test);
//   M3. the full ring: one gridlock loop, counted once however long it runs, every car in it; no deadlock;
//   M4. attached or not: the same state hash at every one of 2,000 ticks with 50 cars on a grid.

namespace {

// The rows whose target is zero. Each must pass; where the run gave nothing to measure (no bend on a straight road)
// it must at least not fail, and `needData` says which must have had something. `knownFailing` names rows the model
// fails today, which the caller checks on its own.
const std::vector<const char*> kZeroTargetRows = {"standing_no_reason", "deadlock_loops", "jumps",     "tight_turns",
                                                  "overlaps",           "over_own_limit", "bend_speed"};

bool listed(const std::vector<const char*>& keys, const char* key) {
    return std::find_if(keys.begin(), keys.end(), [&](const char* k) { return std::string(k) == key; }) != keys.end();
}

void checkZeroTargetRows(const MeasurementReport& rep, const std::vector<const char*>& needData,
                         const std::vector<const char*>& knownFailing = {}) {
    for (const char* key : kZeroTargetRows) {
        if (listed(knownFailing, key)) continue;
        const ReportRow* r = rep.row(key);
        CHECK(r != nullptr);
        if (!r) continue;
        const bool mustHaveData = listed(needData, key);
        const bool ok = r->status == RowStatus::Pass || (!mustHaveData && r->status == RowStatus::NoData);
        CHECK(ok);
        if (!ok) std::printf("    row %s: %s\n", key, rowStatusWords(r->status));
    }
}

}  // namespace

TEST_CASE(measure_M1_grid_city_three_minutes_clean) {
    const TestRoads t = driving_test::gridRoads(4, 4);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(t.graph, rules);
    std::vector<engine::PathId> lanes;
    for (uint32_t i = 0; i < t.graph.paths.size(); ++i)
        if (t.graph.paths[i].kind == engine::PathKind::Lane) lanes.push_back(engine::PathId(i));
    constexpr uint32_t kCars = 60;
    constexpr uint32_t kSeed = 257;
    constexpr Real kDestinationAlong = 50;
    for (uint32_t k = 0; k < kCars; ++k) {
        const engine::PathId lane = lanes[k % lanes.size()];
        CHECK(w.d.placeCar(CarId(k), kOrdinary, at(lane, 10 + static_cast<Real>(k / lanes.size()) * 30)));
        CHECK(w.d.beginTrip(CarId(k), k, at(lanes[engine::hashPair(kSeed, k) % lanes.size()], kDestinationAlong))
                  .valid());
    }
    Measurements m;
    w.d.attachMeasurements(&m);
    constexpr Real dt = 1.0 / 60;
    constexpr int kTicks = static_cast<int>(180 / dt);
    uint32_t tripsBegun = kCars, parked = 0;
    for (int k = 0; k < kTicks; ++k) {
        tickTimed(w.d, dt);
        // A car that has parked sets off again somewhere else (the host's part).
        for (const Event& e : w.d.drainEvents()) {
            if (e.kind != EventKind::Parked) continue;
            ++parked;
            const engine::PathId to = lanes[engine::hashPair(kSeed, e.car.index + kCars * (++tripsBegun)) % lanes.size()];
            w.d.beginTrip(e.car, e.person, at(to, kDestinationAlong));
        }
    }
    const MeasurementReport rep = m.report();
    std::printf("    %u trips parked in 3 minutes\n%s", parked, rep.describe().c_str());
    CHECK(rep.ticks == static_cast<uint64_t>(kTicks));
    CHECK(parked > 0u);
    checkZeroTargetRows(rep, {"standing_no_reason", "deadlock_loops", "jumps", "tight_turns", "over_own_limit"},
                        {"overlaps", "bend_speed"});
    // TWO ROWS THE MODEL FAILS TODAY, measured and printed but not asserted to pass (findings of #257, for the tasks
    // that fix them):
    //   - overlaps: two cars leaving a junction by different connectors into the same lane (a straight-on car and a
    //     turning one) meet at the lane's start and drive into each other, since nothing settles who goes first yet
    //     (the tick's "settle claims" step is empty until the junction tasks);
    //   - bend_speed: a driver's top speed for a bend (corner_speed) is approached through the free-road term, which
    //     brakes too gently when the speed is well above it, so cars reach a turn faster than the rule set's
    //     comfortable sideways acceleration and brake inside it.
    // Each is checked to have been measured; when the model is fixed, move them into the list above.
    const ReportRow* overlaps = rep.row("overlaps");
    const ReportRow* bends = rep.row("bend_speed");
    CHECK(overlaps && overlaps->status != RowStatus::NoData && overlaps->status != RowStatus::NotMeasured);
    CHECK(bends && bends->status != RowStatus::NoData && bends->status != RowStatus::NotMeasured);
    const ReportRow* cost = rep.row("tick_cost");
    CHECK(cost && cost->status == RowStatus::NoTarget && m.rows().cost.ticksTimed == static_cast<uint64_t>(kTicks));
}

TEST_CASE(measure_M2_released_queue_figures) {
    const Straight road(6, 100);
    const TestRules rules(ruleSetLua(kAllRules));
    World w(road.graph(), rules);
    constexpr Real dt = 1.0 / 60;
    decideEvery(w.d, 0.1, dt);
    constexpr uint32_t kCars = 10;
    constexpr Real kLine = 300;
    constexpr Real kSpacing = 9;
    TripId first;
    for (uint32_t k = 0; k < kCars; ++k) {
        CHECK(placeAt(w.d, road, k, kLine - 5 - k * kSpacing));
        const TripId trip = w.d.beginTrip(CarId(k), 3000 + k, road.point(590));
        if (k == 0) first = trip;
    }
    CHECK(w.d.placeHold(first, 1, road.point(kLine)));
    Measurements m;
    m.addCountingLine(road.point(kLine));
    w.d.attachMeasurements(&m);
    for (int k = 0; k < static_cast<int>(60 / dt); ++k) tickTimed(w.d, dt);
    CHECK(w.d.releaseHold(first, 1));
    for (int k = 0; k < static_cast<int>(60 / dt); ++k) tickTimed(w.d, dt);
    const MeasurementReport rep = m.report();
    std::printf("%s", rep.describe().c_str());
    checkZeroTargetRows(rep, {"standing_no_reason", "deadlock_loops", "jumps", "tight_turns", "overlaps",
                              "over_own_limit"});
    // The queue figures are measured and finite, whatever they are. About 2.4 s per car has been measured, outside the
    // plan's band of 1.7 to 2.1 s, so that row is EXPECTED to fail for now: only that it was measured is checked here.
    // The first car's delay fails too, for another reason: it is measured from the moment the car began to move, and a
    // car standing at the line crosses it a fraction of a second later. The plan's "about 2 s" counts the driver's
    // reaction to the green as well, and nothing in the model reacts late to a release yet.
    const ReportRow* headway = rep.row("queue_headway");
    const ReportRow* startUp = rep.row("queue_start_up");
    CHECK(headway && headway->status != RowStatus::NoData && std::isfinite(headway->values[0].value));
    CHECK(startUp && startUp->status != RowStatus::NoData && std::isfinite(startUp->values[0].value));
    CHECK(headway && headway->offenders.kept.size() == 1u && headway->offenders.kept[0].car == CarId(0));
    CHECK(headway && !headway->offenders.kept.empty() && headway->offenders.kept[0].numbers[0].value == kCars);
}

TEST_CASE(measure_M3_full_ring_is_one_gridlock_counted_once) {
    const TestRoads ring = ringOfFour();
    const TestRules rules(ruleSetLua(kAllRules));
    constexpr Real dt = 1.0 / 60;
    World w(ring.graph, rules);
    fillRing(w, ring, CarId::kNone);
    Measurements m;
    w.d.attachMeasurements(&m);
    for (int k = 0; k < static_cast<int>(30 / dt); ++k) w.d.tick(dt);
    const LoopRow after30 = m.rows().loops;
    for (int k = 0; k < static_cast<int>(90 / dt); ++k) w.d.tick(dt);
    const LoopRow& loops = m.rows().loops;
    CHECK(after30.gridlock.loops == 1u && loops.gridlock.loops == 1u);
    CHECK(loops.gridlock.mostCars == kRingCars);
    CHECK(loops.gridlock.longest > after30.gridlock.longest);
    CHECK(loops.deadlock.loops == 0u);
    const MeasurementReport rep = m.report();
    const ReportRow* grid = rep.row("gridlock_loops");
    CHECK(grid && grid->status == RowStatus::NoTarget && grid->offenders.total == 1u);
    CHECK(grid && !grid->offenders.kept.empty() && grid->offenders.kept[0].car == CarId(0));
    const ReportRow* dead = rep.row("deadlock_loops");
    CHECK(dead && dead->status == RowStatus::Pass);
    std::printf("    the full ring for 2 minutes: %llu gridlock loop of %llu cars, lasting %.0f s; %llu deadlocks\n",
                static_cast<unsigned long long>(loops.gridlock.loops),
                static_cast<unsigned long long>(loops.gridlock.mostCars), loops.gridlock.longest,
                static_cast<unsigned long long>(loops.deadlock.loops));
}

TEST_CASE(measure_M4_attached_or_not_same_state_hash) {
    const TestRoads t = driving_test::gridRoads(4, 4);
    const TestRules rules(ruleSetLua(kAllRules));
    std::vector<engine::PathId> lanes;
    for (uint32_t i = 0; i < t.graph.paths.size(); ++i)
        if (t.graph.paths[i].kind == engine::PathKind::Lane) lanes.push_back(engine::PathId(i));
    constexpr uint32_t kCars = 50;
    constexpr int kTicks = 2000;
    constexpr Real dt = 1.0 / 60;
    std::vector<uint64_t> hashes[2];
    Measurements m;
    for (int pass = 0; pass < 2; ++pass) {
        World w(t.graph, rules);
        for (uint32_t k = 0; k < kCars; ++k) {
            const engine::PathId lane = lanes[k % lanes.size()];
            CHECK(w.d.placeCar(CarId(k), kOrdinary, at(lane, 10 + static_cast<Real>(k / lanes.size()) * 30)));
            w.d.beginTrip(CarId(k), k, at(lanes[engine::hashPair(256, k) % lanes.size()], 50));
        }
        if (pass == 1) w.d.attachMeasurements(&m);
        for (int k = 0; k < kTicks; ++k) {
            w.d.tick(dt);
            hashes[pass].push_back(w.d.stateHash());
        }
    }
    CHECK(hashes[0].size() == static_cast<std::size_t>(kTicks));
    CHECK(hashes[0] == hashes[1]);
    CHECK(m.rows().ticks == static_cast<uint64_t>(kTicks));
    std::printf("    %u cars, %d ticks: the same state hash at every tick with the measurements attached and not\n",
                kCars, kTicks);
}

#endif  // RT_ENABLE_SCRIPTING
