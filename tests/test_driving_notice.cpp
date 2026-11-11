// What a driver notices (#251): the fact registry (register, find, refuse a repeat, slots in order), only the facts
// asked for are worked out, the four car-ahead facts share one lookup, each built-in fact against numbers worked by
// hand (and its absent case), the road rules binding and its plain-word refusals, and the same facts whatever order
// the cars are stored in. Every graph is a few straight paths built by hand; rule sets are built as structs.

#include "test_framework.h"

#include "apps/driving/notice.h"
#include "apps/driving/road_rules.h"
#include "apps/driving/rule_set.h"
#include "apps/driving/traffic_state.h"
#include "apps/driving/vehicle_class.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr Real kEps = 1e-9;

// The regulation kinds the test graphs carry, and what the test rule set makes of them: a top speed, and something
// that is not one (it must never change a limit).
constexpr const char* kTopSpeedKind = "top_speed_here";
constexpr const char* kOtherKind = "stop_here";
constexpr engine::NameIndex kTopSpeed = 0, kOther = 1;

// One class: 4.5 m long, braking comfortably at 2 m/s2.
constexpr Real kCarLength = 4.5;
constexpr Real kBrake = 2.0;
constexpr Real kSideAccel = 2.0;   // the rule set's comfortable_side_accel

VehicleClasses testClasses() {
    VehicleClasses c;
    VehicleClass k = deriveVehicleClass("test_car", kCarLength, 1.8, 2.7, 30);
    k.brake = kBrake;
    c.add(k);
    return c;
}

engine::PathId pid(uint32_t i) { return engine::PathId(i); }

// Straight flat paths end to end along +x, each leading into the next, each with its own limit and no bend.
engine::LaneGraph chain(const std::vector<Real>& lengths, const std::vector<Real>& limits) {
    engine::LaneGraph g;
    g.regulationKindNames = {kTopSpeedKind, kOtherKind};
    Real x = 0;
    for (std::size_t i = 0; i < lengths.size(); ++i) {
        engine::Path p;
        p.curve = g.addCurve({engine::Vec2(x, 0), engine::Vec2(x + lengths[i], 0)}, {0, 0});
        p.length = lengths[i];
        p.speedLimit = limits[i];
        p.tightestRadius = engine::kNoBend;
        g.paths.push_back(p);
        x += lengths[i];
    }
    for (std::size_t i = 0; i + 1 < g.paths.size(); ++i) {
        g.paths[i].next.first = static_cast<uint32_t>(g.nextPaths.size());
        g.paths[i].next.count = 1;
        g.nextPaths.push_back(pid(static_cast<uint32_t>(i + 1)));
    }
    return g;
}

// Puts regulations on one path (call path by path, each list sorted by distance along).
void addRegulations(engine::LaneGraph& g, uint32_t path, const std::vector<engine::Regulation>& regs) {
    g.paths[path].regulations.first = static_cast<uint32_t>(g.regulations.size());
    g.paths[path].regulations.count = static_cast<uint32_t>(regs.size());
    for (engine::Regulation r : regs) {
        r.path = pid(path);
        g.regulations.push_back(r);
    }
}
engine::Regulation regulation(engine::NameIndex kind, Real at, Real value = 0) {
    engine::Regulation r;
    r.kind = kind;
    r.at = at;
    r.value = value;
    return r;
}

Number number(const std::string& name, double value, const std::string& unit) {
    Number n;
    n.name = name;
    n.value = n.writtenValue = value;
    n.unit = unit;
    n.source = "test";
    n.hasSource = true;
    return n;
}
PersonalSetting setting(const std::string& name, double low, double usual, double high, const std::string& unit) {
    PersonalSetting s;
    s.name = name;
    s.low = s.writtenLow = low;
    s.usual = s.writtenUsual = usual;
    s.high = s.writtenHigh = high;
    s.unit = unit;
    return s;
}

// The test rule set: the two devices, comfortable_side_accel, and a speed habit everyone shares (`habit`) unless a
// spread is given.
RuleSet testRules(double habit = 1.0) {
    RuleSet r;
    r.name = "test_rules";
    Device stop;
    stop.name = kOtherKind;
    stop.stopLine = true;
    Device top;
    top.name = kTopSpeedKind;
    top.topSpeedIsValue = true;
    r.devices = {stop, top};   // sorted by name
    r.numbers = {number(kNumberComfortableSideAccel, kSideAccel, "m/s2")};
    r.personality = {setting(kSettingSpeedHabit, habit, habit, habit, "ratio")};
    return r;
}

Car record(uint32_t id, uint32_t path, Real along, Real speed = 0) {
    Car c;
    c.id = CarId(id);
    c.vehicleClass = 0;
    c.path = pid(path);
    c.along = along;
    c.speed = speed;
    return c;
}

bool says(const std::string& line, const std::string& words) { return line.find(words) != std::string::npos; }
bool anySays(const std::vector<std::string>& lines, const std::string& a, const std::string& b) {
    for (const std::string& l : lines)
        if (says(l, a) && says(l, b)) return true;
    return false;
}

// A small world: a graph, the classes, a rule set, the built-in facts and the binding, and the cars.
struct World {
    engine::LaneGraph g;
    VehicleClasses classes = testClasses();
    RuleSet rules;
    FactRegistry facts;
    RoadRules roadRules;
    std::vector<Car> cars;
    PathLists lists;

    World(engine::LaneGraph graph, RuleSet r) : g(std::move(graph)), rules(std::move(r)) {
        registerBuiltInFacts(facts);
        std::vector<std::string> problems;
        CHECK(buildRoadRules(g, rules, facts, roadRules, &problems));
        for (const std::string& p : problems) std::printf("    %s\n", p.c_str());
    }
    void place(const std::vector<Car>& c) {
        cars = c;
        rebuildPathLists(g, classes, cars, lists);
    }
    // Every built-in fact for car `id`, along `way` (followOnlyWay when empty), looking `lookAhead` metres ahead.
    std::vector<NoticedFact> notice(uint32_t id, Real lookAhead, PersonId person = 1, WayAhead way = WayAhead(),
                                    NoticeScratch* scratch = nullptr) {
        std::vector<NoticedFact> out;
        const Car* me = findCar(cars, lists, CarId(id));
        CHECK(me != nullptr);
        if (!me) return out;
        if (way.paths.empty()) way = followOnlyWay(g, me->path, me->along, lookAhead);
        const DriverView view{*me, classes.at(me->vehicleClass), person, way, g, cars, lists, rules, roadRules,
                              lookAhead};
        std::vector<FactSlot> all;
        for (std::size_t i = 0; i < facts.size(); ++i) all.push_back(static_cast<FactSlot>(i));
        NoticeScratch own;
        noticeFacts(view, facts, all, out, scratch ? *scratch : own);
        return out;
    }
    const NoticedFact& fact(const std::vector<NoticedFact>& out, const char* name) const {
        return out[facts.find(name)];
    }
};

// A counting fact for the registry tests.
int gCountA = 0, gCountB = 0;
void countA(const DriverView&, NoticeScratch&, NoticedFact& out) {
    ++gCountA;
    out.present = true;
}
void countB(const DriverView&, NoticeScratch&, NoticedFact& out) {
    ++gCountB;
    out.present = true;
}
FactSpec countingSpec(const char* name, NoticeFn fn) {
    FactSpec s;
    s.name = name;
    s.kind = FactKind::YesNo;
    s.notice = fn;
    s.meaning = "counts how often it is worked out";
    return s;
}

}  // namespace

// ---- 1. the registry ---------------------------------------------------------------------------------------------

TEST_CASE(notice_registry_register_find_refuse_in_order) {
    FactRegistry r;
    std::string why;
    CHECK(r.add(countingSpec("first", countA), &why) == 0u);
    CHECK(r.add(countingSpec("second", countB), &why) == 1u);
    CHECK(r.find("second") == 1u);
    CHECK(r.find("nothing") == kNoFact);
    CHECK(r.add(countingSpec("first", countB), &why) == kNoFact);
    CHECK(says(why, "\"first\" is already registered"));
    CHECK(r.add(countingSpec("", countB), &why) == kNoFact);
    CHECK(r.add(countingSpec("third", nullptr), &why) == kNoFact);
    CHECK(says(why, "no function"));
    CHECK(r.size() == 2u);

    // The built-in facts take slots in the order they are listed.
    FactRegistry b;
    CHECK(registerBuiltInFacts(b));
    const std::vector<const char*> order = {kFactCarAhead,   kFactGapAhead,      kFactSpeedAhead, kFactClosingSpeed,
                                            kFactRoadLimit,  kFactMyLimit,       kFactSlowerAhead, kFactCornerSpeed,
                                            kFactLaneEndsAhead, kFactLaneEnd,    kFactMySpeed,    kFactStanding};
    CHECK(b.size() == order.size());
    for (std::size_t i = 0; i < order.size(); ++i) CHECK(b.find(order[i]) == static_cast<FactSlot>(i));
    CHECK(b.at(b.find(kFactLaneEnd)).kind == FactKind::Place);
    CHECK(!b.at(b.find(kFactMyLimit)).meaning.empty());
    // A second registration of the same facts is refused, each by name.
    std::vector<std::string> problems;
    CHECK(!registerBuiltInFacts(b, &problems));
    CHECK(problems.size() == order.size());
    CHECK(b.size() == order.size());
}

TEST_CASE(notice_only_wanted_facts_are_worked_out) {
    World w(chain({50}, {10}), testRules());
    w.place({record(1, 0, 10, 5)});
    FactRegistry r;
    r.add(countingSpec("a", countA));
    r.add(countingSpec("b", countB));
    const Car& me = w.cars[0];
    const WayAhead way = followOnlyWay(w.g, me.path, me.along, 100);
    const DriverView view{me, w.classes.at(0), 1, way, w.g, w.cars, w.lists, w.rules, w.roadRules, 100};
    gCountA = gCountB = 0;
    std::vector<NoticedFact> out;
    noticeFacts(view, r, {1, 1, 7}, out);   // a repeat and an unknown slot are passed over
    CHECK(gCountA == 0 && gCountB == 1);
    CHECK(out.size() == 2u);
    CHECK(!out[0].present && out[1].present);
    CHECK(out[0].kind == FactKind::YesNo);
    noticeFacts(view, r, {}, out);
    CHECK(gCountA == 0 && gCountB == 1);
    CHECK(!out[0].present && !out[1].present);
}

// ---- 2. the car ahead: four facts, one lookup --------------------------------------------------------------------

TEST_CASE(notice_car_ahead_four_facts_one_lookup) {
    // One 100 m lane. Me (car 1) at 20 going 10 m/s; car 2's front at 50 going 6 m/s, so its rear at 50 - 4.5 = 45.5.
    // gap = 45.5 - 20 = 25.5 m; speed ahead 6; closing 10 - 6 = 4 m/s.
    World w(chain({100}, {20}), testRules());
    w.place({record(2, 0, 50, 6), record(1, 0, 20, 10)});
    NoticeScratch scratch;
    std::vector<NoticedFact> f = w.notice(1, 100, 1, WayAhead(), &scratch);
    CHECK(scratch.carAheadLookups == 1u);
    CHECK(w.fact(f, kFactCarAhead).present && w.fact(f, kFactCarAhead).yes);
    CHECK(w.fact(f, kFactCarAhead).aboutCar == CarId(2));
    CHECK_APPROX(w.fact(f, kFactGapAhead).number, 25.5, kEps);
    CHECK(w.fact(f, kFactGapAhead).aboutCar == CarId(2));
    CHECK(w.fact(f, kFactGapAhead).hasMetres);
    CHECK_APPROX(w.fact(f, kFactSpeedAhead).number, 6, kEps);
    CHECK_APPROX(w.fact(f, kFactClosingSpeed).number, 4, kEps);
    CHECK(w.fact(f, kFactClosingSpeed).aboutCar == CarId(2));

    // Absent: car 2 sees nobody ahead; car 1 looking only 20 m sees nobody either. Still one lookup each.
    for (int pass = 0; pass < 2; ++pass) {
        f = pass == 0 ? w.notice(2, 100, 1, WayAhead(), &scratch) : w.notice(1, 20, 1, WayAhead(), &scratch);
        CHECK(scratch.carAheadLookups == 1u);
        CHECK(w.fact(f, kFactCarAhead).present && !w.fact(f, kFactCarAhead).yes);
        CHECK(!w.fact(f, kFactCarAhead).aboutCar.valid());
        CHECK(!w.fact(f, kFactGapAhead).present);
        CHECK(!w.fact(f, kFactSpeedAhead).present);
        CHECK(!w.fact(f, kFactClosingSpeed).present);
    }
}

// ---- 3. limits --------------------------------------------------------------------------------------------------

TEST_CASE(notice_road_limit_two_regulations_on_one_path) {
    // One 100 m path, its own limit 20 m/s; a top speed of 15 at 30 m, something else (not a top speed) at 40 m, a top
    // speed of 10 at 60 m. Before the first: 20. At it (30) and between (45): 15. After the second (70): 10.
    engine::LaneGraph g = chain({100}, {20});
    addRegulations(g, 0, {regulation(kTopSpeed, 30, 15), regulation(kOther, 40, 99), regulation(kTopSpeed, 60, 10)});
    World w(std::move(g), testRules());
    const Real fronts[] = {10, 30, 45, 70};
    const Real limits[] = {20, 15, 15, 10};
    for (int i = 0; i < 4; ++i) {
        w.place({record(1, 0, fronts[i], 5)});
        const std::vector<NoticedFact> f = w.notice(1, 100);
        CHECK(w.fact(f, kFactRoadLimit).present);
        CHECK_APPROX(w.fact(f, kFactRoadLimit).number, limits[i], kEps);
        CHECK(w.fact(f, kFactRoadLimit).aboutPath == pid(0));
        // With everyone's habit 1.0, my_limit is the road limit.
        CHECK_APPROX(w.fact(f, kFactMyLimit).number, limits[i], kEps);
    }
    // Absent: a car whose path is not on the road (unknown path index) has no limit.
    w.place({record(1, 0, 10, 5)});
    w.cars[0].path = pid(9);
    const WayAhead none;
    const DriverView view{w.cars[0], w.classes.at(0), 1, none, w.g, w.cars, w.lists, w.rules, w.roadRules, 100};
    std::vector<NoticedFact> out;
    noticeFacts(view, w.facts, {w.facts.find(kFactRoadLimit), w.facts.find(kFactMyLimit)}, out);
    CHECK(!w.fact(out, kFactRoadLimit).present && !w.fact(out, kFactMyLimit).present);
}

TEST_CASE(notice_my_limit_differs_between_people_same_for_one) {
    // A spread of habits, 0.8 to 1.2 around 1.0. my_limit = 20 x personalValue(speed_habit, person).
    RuleSet rules = testRules();
    rules.personality = {setting(kSettingSpeedHabit, 0.8, 1.0, 1.2, "ratio")};
    World w(chain({100}, {20}), rules);
    w.place({record(1, 0, 10, 5)});
    const PersonId ann = 11, bob = 12;
    const double annHabit = personalValue(rules.personality[0], ann);
    const double bobHabit = personalValue(rules.personality[0], bob);
    CHECK(annHabit != bobHabit);
    const Real annLimit = w.fact(w.notice(1, 100, ann), kFactMyLimit).number;
    const Real bobLimit = w.fact(w.notice(1, 100, bob), kFactMyLimit).number;
    CHECK_APPROX(annLimit, 20 * annHabit, kEps);
    CHECK_APPROX(bobLimit, 20 * bobHabit, kEps);
    CHECK(annLimit != bobLimit);
    for (int i = 0; i < 5; ++i) CHECK(w.fact(w.notice(1, 100, ann), kFactMyLimit).number == annLimit);
}

TEST_CASE(notice_slower_ahead_formula_closer_is_lower_absent) {
    // Two 100 m paths: limit 20, then 10. Brake 2 m/s2, habit 1. From 40 m along, the lower limit starts 60 m ahead:
    // sqrt(10^2 + 2 x 2 x 60) = sqrt(340) = 18.439 m/s. From 80 m: 20 m ahead, sqrt(100 + 80) = sqrt(180) = 13.416.
    World w(chain({100, 100}, {20, 10}), testRules());
    w.place({record(1, 0, 40, 15)});
    std::vector<NoticedFact> f = w.notice(1, 200);
    const NoticedFact& far = w.fact(f, kFactSlowerAhead);
    CHECK(far.present);
    CHECK_APPROX(far.number, std::sqrt(340.0), kEps);
    CHECK(far.aboutPath == pid(1) && far.hasMetres);
    CHECK_APPROX(far.metres, 60, kEps);
    const Real farSpeed = far.number;
    w.place({record(1, 0, 80, 15)});
    f = w.notice(1, 200);
    CHECK_APPROX(w.fact(f, kFactSlowerAhead).number, std::sqrt(180.0), kEps);
    CHECK(w.fact(f, kFactSlowerAhead).number < farSpeed);

    // With a habit of 0.9 the speed wanted there is 9: from 40 m, sqrt(81 + 240) = sqrt(321) = 17.916.
    World h(chain({100, 100}, {20, 10}), testRules(0.9));
    h.place({record(1, 0, 40, 15)});
    CHECK_APPROX(h.fact(h.notice(1, 200), kFactSlowerAhead).number, std::sqrt(321.0), kEps);

    // A lower top speed on my own path, 12 at 70 m, is nearer and lower: from 40 m, 30 m ahead, sqrt(144 + 120) =
    // sqrt(264) = 16.248 < 18.439.
    engine::LaneGraph g = chain({100, 100}, {20, 10});
    addRegulations(g, 0, {regulation(kTopSpeed, 70, 12)});
    World r(std::move(g), testRules());
    r.place({record(1, 0, 40, 15)});
    f = r.notice(1, 200);
    CHECK_APPROX(r.fact(f, kFactSlowerAhead).number, std::sqrt(264.0), kEps);
    CHECK(r.fact(f, kFactSlowerAhead).aboutPath == pid(0));
    CHECK_APPROX(r.fact(f, kFactSlowerAhead).metres, 30, kEps);

    // Absent: the lower limit is past my look-ahead (60 m away, looking 50 m), or the limit ahead is higher.
    w.place({record(1, 0, 40, 15)});
    CHECK(!w.fact(w.notice(1, 50), kFactSlowerAhead).present);
    World up(chain({100, 100}, {20, 25}), testRules());
    up.place({record(1, 0, 40, 15)});
    CHECK(!up.fact(up.notice(1, 200), kFactSlowerAhead).present);
}

TEST_CASE(notice_corner_speed_formula_closer_is_lower_absent) {
    // Paths 50, 30, 50 m; the middle one bends at radius 20 m, the last at 200 m. Side accel 2 m/s2, brake 2 m/s2.
    // The 20 m bend: sqrt(2 x 20) = sqrt(40). From 10 m along the first path it starts 40 m ahead:
    // sqrt(40 + 2 x 2 x 40) = sqrt(200) = 14.142. The 200 m bend: sqrt(400) = 20, 70 m ahead: sqrt(400 + 280) = 26.08,
    // higher, so the 20 m bend is the fact. From 30 m: 20 m ahead, sqrt(40 + 80) = sqrt(120) = 10.954.
    engine::LaneGraph g = chain({50, 30, 50}, {20, 20, 20});
    g.paths[1].tightestRadius = 20;
    g.paths[2].tightestRadius = 200;
    World w(std::move(g), testRules());
    w.place({record(1, 0, 10, 15)});
    std::vector<NoticedFact> f = w.notice(1, 200);
    const NoticedFact& c = w.fact(f, kFactCornerSpeed);
    CHECK(c.present);
    CHECK_APPROX(c.number, std::sqrt(200.0), kEps);
    CHECK(c.aboutPath == pid(1));
    CHECK_APPROX(c.metres, 40, kEps);
    const Real farSpeed = c.number;
    w.place({record(1, 0, 30, 15)});
    f = w.notice(1, 200);
    CHECK_APPROX(w.fact(f, kFactCornerSpeed).number, std::sqrt(120.0), kEps);
    CHECK(w.fact(f, kFactCornerSpeed).number < farSpeed);
    // In the bend itself: distance zero, so the bend's own speed, sqrt(40) = 6.325.
    w.place({record(1, 1, 5, 6)});
    f = w.notice(1, 200);
    CHECK_APPROX(w.fact(f, kFactCornerSpeed).number, std::sqrt(40.0), kEps);
    CHECK(w.fact(f, kFactCornerSpeed).metres == 0);

    // Absent: no bend anywhere, or the bend beyond my look-ahead (40 m away, looking 30 m).
    World straight(chain({50, 30, 50}, {20, 20, 20}), testRules());
    straight.place({record(1, 0, 10, 15)});
    CHECK(!straight.fact(straight.notice(1, 200), kFactCornerSpeed).present);
    w.place({record(1, 0, 10, 15)});
    WayAhead upToBend;
    upToBend.paths = {pid(0), pid(1)};
    CHECK(!w.fact(w.notice(1, 30, 1, upToBend), kFactCornerSpeed).present);
}

// ---- 4. where my way runs out ------------------------------------------------------------------------------------

TEST_CASE(notice_lane_end_across_three_short_paths) {
    // Three 5 m paths; the last leads nowhere. From 2 m along the first: 3 + 5 + 5 = 13 m to the end, on path 2.
    World w(chain({5, 5, 5}, {10, 10, 10}), testRules());
    w.place({record(1, 0, 2, 3)});
    std::vector<NoticedFact> f = w.notice(1, 50);
    CHECK(w.fact(f, kFactLaneEndsAhead).present && w.fact(f, kFactLaneEndsAhead).yes);
    const NoticedFact& end = w.fact(f, kFactLaneEnd);
    CHECK(end.present && end.kind == FactKind::Place);
    CHECK_APPROX(end.number, 13, kEps);
    CHECK(end.aboutPath == pid(2));
    CHECK_APPROX(end.metres, 13, kEps);
    // Further on, on the second path at 4 m: 1 + 5 = 6 m.
    w.place({record(1, 1, 4, 3)});
    CHECK_APPROX(w.fact(w.notice(1, 50), kFactLaneEnd).number, 6, kEps);

    // Absent: the end is past my look-ahead (13 m away, looking 10 m) ...
    w.place({record(1, 0, 2, 3)});
    f = w.notice(1, 10);
    CHECK(w.fact(f, kFactLaneEndsAhead).present && !w.fact(f, kFactLaneEndsAhead).yes);
    CHECK(!w.fact(f, kFactLaneEnd).present);
    // ... or the supplied way stops at a path that does lead on (my route is just not given further).
    WayAhead shortWay;
    shortWay.paths = {pid(0), pid(1)};
    f = w.notice(1, 50, 1, shortWay);
    CHECK(!w.fact(f, kFactLaneEndsAhead).yes);
    CHECK(!w.fact(f, kFactLaneEnd).present);
}

// ---- 5. my own speed ---------------------------------------------------------------------------------------------

TEST_CASE(notice_my_speed_and_standing) {
    // 3 m/s is moving; 0.05 m/s is below kStandingBelow (0.1), standing.
    World w(chain({50}, {10}), testRules());
    w.place({record(1, 0, 10, 3)});
    std::vector<NoticedFact> f = w.notice(1, 50);
    CHECK_APPROX(w.fact(f, kFactMySpeed).number, 3, kEps);
    CHECK(w.fact(f, kFactStanding).present && !w.fact(f, kFactStanding).yes);
    w.place({record(1, 0, 10, 0.05)});
    f = w.notice(1, 50);
    CHECK(w.fact(f, kFactStanding).yes);
    CHECK_APPROX(w.fact(f, kFactMySpeed).number, 0.05, kEps);
}

// ---- 6. road rules ----------------------------------------------------------------------------------------------

TEST_CASE(notice_road_rules_binding_and_refusals) {
    FactRegistry facts;
    registerBuiltInFacts(facts);
    std::vector<std::string> problems;

    // Sound: each kind bound to its device.
    const engine::LaneGraph g = chain({10}, {10});
    RoadRules rr;
    CHECK(buildRoadRules(g, testRules(), facts, rr, &problems));
    CHECK(problems.empty());
    const RuleSet rules = testRules();
    CHECK(rr.device(rules, kTopSpeed) && rr.device(rules, kTopSpeed)->topSpeedIsValue);
    CHECK(rr.device(rules, kOther) && rr.device(rules, kOther)->name == kOtherKind);
    CHECK(rr.device(rules, 7) == nullptr);

    // A regulation kind with no device: refused naming both.
    engine::LaneGraph odd = g;
    odd.regulationKindNames.push_back("mystery_kind");
    CHECK(!buildRoadRules(odd, rules, facts, rr, &problems));
    CHECK(problems.size() == 1u && anySays(problems, "mystery_kind", "test_rules"));

    // Missing comfortable_side_accel: names corner_speed and the number.
    problems.clear();
    RuleSet noSide = testRules();
    noSide.numbers.clear();
    CHECK(!buildRoadRules(g, noSide, facts, rr, &problems));
    CHECK(problems.size() == 1u && anySays(problems, "corner_speed", "comfortable_side_accel"));

    // Missing speed_habit: names each fact that needs it.
    problems.clear();
    RuleSet noHabit = testRules();
    noHabit.personality.clear();
    CHECK(!buildRoadRules(g, noHabit, facts, rr, &problems));
    CHECK(problems.size() == 2u);
    CHECK(anySays(problems, "my_limit", "speed_habit") && anySays(problems, "slower_ahead", "speed_habit"));

    // The wrong kind of unit: a time where an acceleration is wanted.
    problems.clear();
    RuleSet wrongUnit = testRules();
    wrongUnit.numbers[0].unit = "s";
    CHECK(!buildRoadRules(g, wrongUnit, facts, rr, &problems));
    CHECK(problems.size() == 1u && anySays(problems, "comfortable_side_accel", "a time"));
    CHECK(anySays(problems, "corner_speed", "an acceleration"));

    // Three problems at once, all reported: the unknown kind, the wrong unit, and a fact's missing number.
    FactRegistry more;
    registerBuiltInFacts(more);
    FactSpec extra = countingSpec("test_needs_a_gap", countA);
    FactNeed gapNeed;
    gapNeed.name = "test_gap";
    gapNeed.family = UnitFamily::Length;
    extra.numbers.push_back(gapNeed);
    more.add(extra);
    problems.clear();
    CHECK(!buildRoadRules(odd, wrongUnit, more, rr, &problems));
    CHECK(problems.size() == 3u);
    CHECK(anySays(problems, "mystery_kind", "no device"));
    CHECK(anySays(problems, "corner_speed", "a time"));
    CHECK(anySays(problems, "test_needs_a_gap", "test_gap"));
    for (const std::string& p : problems) std::printf("      %s\n", p.c_str());
}

// ---- 7. the order cars are stored in changes nothing -------------------------------------------------------------

TEST_CASE(notice_same_facts_whatever_order_cars_are_stored) {
    // Two 60 m paths with a lower limit and a bend on the second; six cars over both, two level with each other.
    engine::LaneGraph g = chain({60, 60}, {20, 12});
    g.paths[1].tightestRadius = 40;
    addRegulations(g, 1, {regulation(kTopSpeed, 30, 8)});
    RuleSet rules = testRules();
    rules.personality = {setting(kSettingSpeedHabit, 0.8, 1.0, 1.2, "ratio")};
    World w(std::move(g), rules);
    const std::vector<Car> stored = {record(4, 0, 10, 12),  record(9, 0, 30, 11), record(2, 0, 50, 0.0),
                                     record(7, 1, 10, 9),   record(5, 1, 25, 4),  record(3, 1, 55, 0.05)};
    const std::vector<uint32_t> ids = {2, 3, 4, 5, 7, 9};

    w.place(stored);
    std::vector<std::vector<NoticedFact>> first;
    for (uint32_t id : ids) first.push_back(w.notice(id, 120, 100 + id));

    std::vector<Car> reversed = stored;
    std::reverse(reversed.begin(), reversed.end());
    std::vector<Car> shuffled = {stored[3], stored[0], stored[5], stored[1], stored[4], stored[2]};
    for (const std::vector<Car>& order : {reversed, shuffled}) {
        w.place(order);
        for (std::size_t i = 0; i < ids.size(); ++i) {
            const std::vector<NoticedFact> again = w.notice(ids[i], 120, 100 + ids[i]);
            CHECK(again.size() == first[i].size());
            for (std::size_t s = 0; s < again.size() && s < first[i].size(); ++s) CHECK(again[s] == first[i][s]);
        }
    }
    // And they say something: the first car sees car 9 ahead, the slower limit and the bend.
    const std::vector<NoticedFact>& car4 = first[2];
    CHECK(w.fact(car4, kFactCarAhead).aboutCar == CarId(9));
    CHECK(w.fact(car4, kFactSlowerAhead).present);
    CHECK(w.fact(car4, kFactCornerSpeed).present);
}
