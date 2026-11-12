// WHO GIVES WAY TO WHOM, WORKED OUT AT LOAD (right_of_way.h), and EACH DRIVER'S PLAN FOR THE JUNCTIONS AHEAD
// (junction_plan.h): issue #260, task D1, docs/driving-briefs/D1-junction-plan.md. On the hand-made grids of
// driving_test_roads.h, whose junctions carry real crossing points from the road builder's own finder.
//
// The table, against tables worked by hand (the brief's "done when"):
//   1. a four-way with lights, two phases;
//   2. the same with a far-turn arrow phase;
//   3. an all-way stop;
//   4. a side road with a stop sign meeting a main road (a T);
//   5. a four-way with no signs;
//   6. mirroring the junction's shape and flipping the rule set's side gives the same table (no fixed side slipped in);
//   7. "on my kerb side" at a five-way: only the nearest approach toward the kerb;
//   8. the table as text; the rule set's control table read from Lua, and its faults.
// The plans:
//   9. the plan for each turn kind at a four-way and at a T;
//  10. a four-way entered from each arm;
//  11. two junctions close together: both planned, the second kept (not rebuilt) when the first is cleared;
//  12. a plan dropped when the route changes (changeDestination, reroute), and made again;
//  13. a junction where a turn was refused (no connector);
//  14. waiting points: never inside a crossing stretch, fewer for a longer body;
//  15. the same plans every run, and the state hash the same with plans as the model had before them.
//
// Compass words: the test roads name North +y on the plan; the table's text names the world's compass (north -z), so
// the text is only checked for its shape here.

#include "test_framework.h"

#include "driving_test_roads.h"

#include "apps/driving/act.h"
#include "apps/driving/driving.h"
#include "apps/driving/junction_plan.h"
#include "apps/driving/right_of_way.h"
#include "apps/driving/road_rules.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace driving;
using driving_test::Heading;
using driving_test::TestRoads;
using driving_test::TestRoadSpec;

namespace {

constexpr Heading kEast = Heading::East, kNorth = Heading::North, kWest = Heading::West, kSouth = Heading::South;
constexpr Heading kHeadings[] = {kEast, kNorth, kWest, kSouth};

// ---- Rule sets made in C++ (no Lua needed for the table) ----------------------------------------------------------------

ControlKind control(const std::string& name, const std::vector<SettleQuestion>& settle) {
    ControlKind k;
    k.name = name;
    k.settle = settle;
    return k;
}

MovementMask mask(Movement m) { return static_cast<MovementMask>(1u << static_cast<uint8_t>(m)); }

// The design's rows (6.1 b).
ControlKind signal(bool arrows) {
    ControlKind k = control(arrows ? "signal_arrow" : "signal", {SettleQuestion::Light, SettleQuestion::Movement});
    k.givesWayOnGreen = mask(Movement::FarTurn);
    if (arrows) k.arrow = mask(Movement::FarTurn);
    k.turnOnRed = true;
    k.turnOnRedMovement = Movement::NearTurn;
    k.turnOnRedAfterFullStop = true;
    return k;
}
ControlKind allWayStop() {
    ControlKind k = control("all_way_stop", {SettleQuestion::ArrivedFirst, SettleQuestion::KerbSide, SettleQuestion::Movement});
    k.everyoneStops = true;
    return k;
}
ControlKind minorStop() {
    return control("minor_stop", {SettleQuestion::SignRank, SettleQuestion::Movement, SettleQuestion::ArrivedFirst});
}
ControlKind unmarked() { return control("unmarked", {SettleQuestion::ArrivedFirst, SettleQuestion::KerbSide}); }

RuleSet rulesWith(const ControlKind& k, DrivesOn side = DrivesOn::Left) {
    RuleSet r;
    r.name = "d1_test";
    r.drivesOn = side;
    r.ranks = {"priority", "give_way", "stop"};
    Device stop;
    stop.name = "stop";
    stop.stopLine = true;
    stop.rank = 2;
    r.devices = {stop};
    r.controls = {k};
    r.movementRank = {Movement::Straight, Movement::NearTurn, Movement::FarTurn, Movement::UTurn};
    return r;
}

// The table for `g` under `r`: every junction carries r's one control.
RightOfWay tableFor(engine::LaneGraph& g, const RuleSet& r) {
    g.controlNames = {r.controls[0].name};
    if (g.regulationKindNames.empty()) g.regulationKindNames = {"stop"};
    RoadRules rr;
    FactRegistry none;
    std::vector<std::string> problems;
    CHECK(buildRoadRules(g, r, none, rr, &problems));
    for (const std::string& p : problems) std::printf("    %s\n", p.c_str());
    return buildRightOfWay(g, r, rr);
}

// ---- The test junction's connectors by name ------------------------------------------------------------------------------

struct Turn {
    Heading in = kEast, out = kEast;
};
// Every connector of junction (row, col), lane 0 into lane 0, by id.
std::map<uint32_t, Turn> connectorsOf(const TestRoads& t, uint32_t row, uint32_t col) {
    std::map<uint32_t, Turn> out;
    for (const Heading in : kHeadings)
        for (const Heading o : kHeadings) {
            const engine::PathId c = t.connector(row, col, in, 0, o, 0);
            if (c.valid()) out[c.index] = Turn{in, o};
        }
    return out;
}

Movement movementOf(const Turn& t, DrivesOn side) {
    if (t.out == t.in) return Movement::Straight;
    if (t.out == driving_test::nearTurnFrom(t.in, side)) return Movement::NearTurn;
    return Movement::FarTurn;
}
uint32_t rankOf(Movement m) { return static_cast<uint32_t>(m); }   // straight, near, far, U: the test's movement_rank
// Approach a (arriving heading) is on approach b's kerb side: a's traffic comes from the road b's near turn goes into.
bool onKerbSide(Heading a, Heading b, DrivesOn side) { return a == driving_test::opposite(driving_test::nearTurnFrom(b, side)); }

// HOW A PAIR SHOULD COME OUT, worked by hand from the brief's questions, for turns a (the record's side A) and b.
struct Expect {
    GivesWay who = GivesWay::Ask;
    SettledBy by = SettledBy::ArrivedFirst;
    GivesWay fallback = GivesWay::A;
    SettledBy fallbackBy = SettledBy::LowerPathId;
};
bool same(const MeetingAnswer& got, const Expect& e) {
    if (got.who != e.who || got.by != e.by) return false;
    if (e.who != GivesWay::Ask) return true;
    return got.fallback == e.fallback && got.fallbackBy == e.fallbackBy;
}
// Kerb side, then movement, as the fall-back (or answer) of a pair; `kerb` and `movement` say which questions are in the
// list after the first.
bool kerbThenMovement(const Turn& a, const Turn& b, bool kerb, bool movement, GivesWay& who, SettledBy& by) {
    if (kerb && a.in != b.in) {
        if (onKerbSide(a.in, b.in, DrivesOn::Left) && !onKerbSide(b.in, a.in, DrivesOn::Left)) {
            who = GivesWay::Other;
            by = SettledBy::KerbSide;
            return true;
        }
        if (onKerbSide(b.in, a.in, DrivesOn::Left) && !onKerbSide(a.in, b.in, DrivesOn::Left)) {
            who = GivesWay::A;
            by = SettledBy::KerbSide;
            return true;
        }
    }
    if (movement) {
        const uint32_t ra = rankOf(movementOf(a, DrivesOn::Left)), rb = rankOf(movementOf(b, DrivesOn::Left));
        if (ra != rb) {
            who = ra > rb ? GivesWay::A : GivesWay::Other;
            by = SettledBy::Movement;
            return true;
        }
    }
    who = GivesWay::A;
    by = SettledBy::LowerPathId;
    return false;
}

// Checks every meeting of two connectors at a junction against `expect`, and that there is at least one cross and one
// join. Returns how many were checked.
template <typename Fn>
int checkMeetings(const TestRoads& t, const RightOfWay& row, uint32_t r, uint32_t c, Fn expect) {
    const std::map<uint32_t, Turn> names = connectorsOf(t, r, c);
    const engine::Junction& jn = t.graph.junctions[t.junction(r, c).index];
    int checked = 0, crosses = 0, joins = 0, wrong = 0;
    for (uint32_t k = jn.crossings.first; k < jn.crossings.end(); ++k) {
        const engine::CrossingPoint& rec = t.graph.crossings[k];
        if (rec.otherSide != engine::OtherSide::Path) continue;
        const Turn a = names.at(rec.a.index), b = names.at(rec.other);
        const uint32_t phases = row.junctions[t.junction(r, c).index].phaseCount();
        for (uint32_t ph = 0; ph < phases; ++ph) {
            const MeetingAnswer* got = row.answer(engine::CrossingId(k), ph);
            CHECK(got != nullptr);
            if (!got) continue;
            Expect e;
            if (rec.meeting == engine::Meeting::Split) {
                e.who = GivesWay::Neither;
                e.by = SettledBy::Split;
            } else {
                e = expect(a, b, ph);
            }
            if (!same(*got, e)) {
                ++wrong;
                std::printf("    meeting %u (connectors %u, %u) phase %u: got who %d by %d (fallback %d by %d), worked by "
                            "hand who %d by %d (fallback %d by %d)\n",
                            k, rec.a.index, rec.other, ph, static_cast<int>(got->who), static_cast<int>(got->by),
                            static_cast<int>(got->fallback), static_cast<int>(got->fallbackBy), static_cast<int>(e.who),
                            static_cast<int>(e.by), static_cast<int>(e.fallback), static_cast<int>(e.fallbackBy));
            }
        }
        ++checked;
        crosses += rec.meeting == engine::Meeting::Cross;
        joins += rec.meeting == engine::Meeting::Join;
    }
    CHECK(wrong == 0);
    CHECK(crosses > 0);
    CHECK(joins > 0);
    return checked;
}

// The lights of a junction's phase, as the connector ids that are green and those that may turn on red.
std::vector<uint32_t> lightsOf(const TestRoads& t, const RightOfWay& row, uint32_t r, uint32_t c, uint32_t phase,
                               Light which) {
    std::vector<uint32_t> out;
    const engine::Junction& jn = t.graph.junctions[t.junction(r, c).index];
    for (uint32_t i = 0; i < jn.connectors.count; ++i) {
        const engine::PathId p = t.graph.junctionConnectors[jn.connectors.first + i];
        if (row.light(t.graph, p, phase) == which) out.push_back(p.index);
    }
    return out;
}
// The connectors of junction (r, c) arriving on the given headings (and, when `only` is set, of that movement).
std::vector<uint32_t> turnsFrom(const TestRoads& t, uint32_t r, uint32_t c, std::vector<Heading> ins, bool onlyOne = false,
                                Movement only = Movement::Straight) {
    std::vector<uint32_t> out;
    for (const std::pair<const uint32_t, Turn>& n : connectorsOf(t, r, c)) {
        bool in = false;
        for (const Heading h : ins) in = in || n.second.in == h;
        if (in && (!onlyOne || movementOf(n.second, DrivesOn::Left) == only)) out.push_back(n.first);
    }
    return out;
}

// The lights expected at a two-phase four-way, by hand: phase `ph`'s green approaches.
Light handLight(const Turn& t, const std::vector<Heading>& green, bool nearOnRed, const std::vector<uint32_t>* arrowOnly,
                uint32_t id) {
    if (arrowOnly) {
        for (const uint32_t g : *arrowOnly)
            if (g == id) return Light::Green;
    } else {
        for (const Heading h : green)
            if (t.in == h) return Light::Green;
    }
    return nearOnRed && movementOf(t, DrivesOn::Left) == Movement::NearTurn ? Light::RedMayTurn : Light::Red;
}

// The signal rule worked by hand: a green before a red; a red that may turn gives way to a plain red; then movement;
// then the lower path id.
Expect bySignal(const Turn& a, const Turn& b, Light la, Light lb) {
    Expect e;
    e.fallback = GivesWay::A;
    if ((la == Light::Green) != (lb == Light::Green)) {
        e.who = la == Light::Green ? GivesWay::Other : GivesWay::A;
        e.by = SettledBy::Light;
        return e;
    }
    if (la == Light::RedMayTurn && lb == Light::Red) {
        e.who = GivesWay::A;
        e.by = SettledBy::Light;
        return e;
    }
    if (lb == Light::RedMayTurn && la == Light::Red) {
        e.who = GivesWay::Other;
        e.by = SettledBy::Light;
        return e;
    }
    kerbThenMovement(a, b, false, true, e.who, e.by);
    return e;
}

}  // namespace

// ---- 1, 2. a four-way with lights ----------------------------------------------------------------------------------------

TEST_CASE(junction_table_1_four_way_lights_two_phases) {
    TestRoads t = driving_test::gridRoads(3, 3);
    const RightOfWay row = tableFor(t.graph, rulesWith(signal(false)));
    const JunctionRules& jr = row.junctions[t.junction(1, 1).index];
    CHECK(jr.phases.count == 2u);
    // By hand: the approaches heading east and west share phase 0 (their far turns giving way), north and south phase 1;
    // nothing else fits either phase; the near turns may go on red.
    const std::vector<std::vector<Heading>> green = {{kEast, kWest}, {kNorth, kSouth}};
    for (uint32_t ph = 0; ph < 2 && ph < jr.phases.count; ++ph) {
        CHECK(lightsOf(t, row, 1, 1, ph, Light::Green) == turnsFrom(t, 1, 1, green[ph]));
        CHECK(lightsOf(t, row, 1, 1, ph, Light::RedMayTurn) ==
              turnsFrom(t, 1, 1, green[1 - ph], true, Movement::NearTurn));
        CHECK(!row.phases[jr.phases.first + ph].arrow);
    }
    const int n = checkMeetings(t, row, 1, 1, [&](const Turn& a, const Turn& b, uint32_t ph) {
        return bySignal(a, b, handLight(a, green[ph], true, nullptr, 0), handLight(b, green[ph], true, nullptr, 0));
    });
    std::printf("    %d meetings at the four-way, each checked at both phases\n", n);
}

TEST_CASE(junction_table_2_four_way_lights_with_far_turn_arrows) {
    TestRoads t = driving_test::gridRoads(3, 3);
    const RightOfWay row = tableFor(t.graph, rulesWith(signal(true)));
    const JunctionRules& jr = row.junctions[t.junction(1, 1).index];
    // By hand: an arrow for the east and west far turns, then east and west; an arrow for the north and south far turns,
    // then north and south.
    CHECK(jr.phases.count == 4u);
    if (jr.phases.count != 4u) return;
    const std::vector<uint32_t> arrowEW = turnsFrom(t, 1, 1, {kEast, kWest}, true, Movement::FarTurn);
    const std::vector<uint32_t> arrowNS = turnsFrom(t, 1, 1, {kNorth, kSouth}, true, Movement::FarTurn);
    const std::vector<Heading> greenOf[4] = {{}, {kEast, kWest}, {}, {kNorth, kSouth}};
    const std::vector<uint32_t>* arrowOf[4] = {&arrowEW, nullptr, &arrowNS, nullptr};
    for (uint32_t ph = 0; ph < 4; ++ph) {
        CHECK(row.phases[jr.phases.first + ph].arrow == (arrowOf[ph] != nullptr));
        CHECK(lightsOf(t, row, 1, 1, ph, Light::Green) == (arrowOf[ph] ? *arrowOf[ph] : turnsFrom(t, 1, 1, greenOf[ph])));
    }
    checkMeetings(t, row, 1, 1, [&](const Turn& a, const Turn& b, uint32_t ph) {
        const std::map<uint32_t, Turn> names = connectorsOf(t, 1, 1);
        uint32_t ia = 0, ib = 0;
        for (const std::pair<const uint32_t, Turn>& nm : names) {
            if (nm.second.in == a.in && nm.second.out == a.out) ia = nm.first;
            if (nm.second.in == b.in && nm.second.out == b.out) ib = nm.first;
        }
        return bySignal(a, b, handLight(a, greenOf[ph], true, arrowOf[ph], ia),
                        handLight(b, greenOf[ph], true, arrowOf[ph], ib));
    });
}

// ---- 3. an all-way stop --------------------------------------------------------------------------------------------------

TEST_CASE(junction_table_3_all_way_stop) {
    TestRoads t = driving_test::gridRoads(3, 3);
    const RightOfWay row = tableFor(t.graph, rulesWith(allWayStop()));
    CHECK(row.junctions[t.junction(1, 1).index].phases.count == 0u);
    // By hand: every pair is asked while driving; if that ties, the one from the other's kerb side goes first, and
    // between approaches facing each other, straight on before the near turn before the far turn.
    checkMeetings(t, row, 1, 1, [&](const Turn& a, const Turn& b, uint32_t) {
        Expect e;
        e.who = GivesWay::Ask;
        e.by = SettledBy::ArrivedFirst;
        kerbThenMovement(a, b, true, true, e.fallback, e.fallbackBy);
        return e;
    });
    // Nothing at an all-way stop is settled at load: every meeting but the splits is asked while driving.
    CHECK(row.counts.askWhileDriving + row.counts.splits == row.counts.meetings);
    CHECK(row.counts.settledAtLoad == 0u);
}

// ---- 4. a T with a stop sign on the side road ----------------------------------------------------------------------------

TEST_CASE(junction_table_4_side_road_stop_sign_at_a_t) {
    // A grid two junctions deep and three wide: junction (0, 1) is a T, its side road arriving heading south.
    TestRoads t = driving_test::gridRoads(2, 3);
    engine::LaneGraph& g = t.graph;
    const engine::PathId side = t.laneArriving(0, 1, kSouth, 0);
    g.regulationKindNames = {"stop"};
    engine::Regulation stop;
    stop.kind = 0;
    stop.path = side;
    stop.from = stop.to = g.paths[side.index].length;
    g.regulations = {stop};
    g.paths[side.index].regulations = engine::Range{0, 1};
    CHECK(engine::validate(g).empty());
    const RightOfWay row = tableFor(g, rulesWith(minorStop()));
    const JunctionRules& jr = row.junctions[t.junction(0, 1).index];
    CHECK(jr.approaches.count == 3u);
    // By hand: the side road gives way to the main road by its sign; on the main road, straight on before the near
    // turn before the far turn; two of a kind are asked while driving and fall back on the lower path id.
    checkMeetings(t, row, 0, 1, [&](const Turn& a, const Turn& b, uint32_t) {
        Expect e;
        const bool aSide = a.in == kSouth, bSide = b.in == kSouth;
        if (aSide != bSide) {
            e.who = aSide ? GivesWay::A : GivesWay::Other;
            e.by = SettledBy::SignRank;
            return e;
        }
        if (!kerbThenMovement(a, b, false, true, e.who, e.by)) {
            e.who = GivesWay::Ask;
            e.by = SettledBy::ArrivedFirst;
            e.fallback = GivesWay::A;
            e.fallbackBy = SettledBy::LowerPathId;
        }
        return e;
    });
}

// ---- 5. a four-way with no signs -----------------------------------------------------------------------------------------

TEST_CASE(junction_table_5_four_way_no_signs) {
    TestRoads t = driving_test::gridRoads(3, 3);
    const RightOfWay row = tableFor(t.graph, rulesWith(unmarked()));
    // By hand: asked while driving; if that ties, the one from the other's kerb side; approaches facing each other have
    // nothing more to ask, so the lower path id gives way (and the loader notes it).
    checkMeetings(t, row, 1, 1, [&](const Turn& a, const Turn& b, uint32_t) {
        Expect e;
        e.who = GivesWay::Ask;
        e.by = SettledBy::ArrivedFirst;
        kerbThenMovement(a, b, true, false, e.fallback, e.fallbackBy);
        return e;
    });
    CHECK(row.counts.byLowerPathId > 0);
    CHECK(!row.notes.empty());
}

// ---- 6. mirrored shape, flipped side: the same table ---------------------------------------------------------------------

TEST_CASE(junction_table_6_mirrored_and_flipped_is_the_same) {
    for (const ControlKind& k : {signal(true), allWayStop(), unmarked(), minorStop()}) {
        TestRoads t = driving_test::gridRoads(3, 3);
        engine::LaneGraph mirrored = t.graph;
        for (engine::Vec2& p : mirrored.points) p.x = -p.x;
        for (engine::Vec2& p : mirrored.planPoints) p.x = -p.x;
        const RightOfWay a = tableFor(t.graph, rulesWith(k, DrivesOn::Left));
        const RightOfWay b = tableFor(mirrored, rulesWith(k, DrivesOn::Right));
        CHECK(a.answers == b.answers);
        CHECK(a.lights == b.lights);
        CHECK(a.phases.size() == b.phases.size());
        CHECK(a.counts.byLowerPathId == b.counts.byLowerPathId);
        // ... and the kerb side really was asked: with the side left as it was, the mirror differs.
        if (k.name == "unmarked") {
            const RightOfWay wrong = tableFor(mirrored, rulesWith(k, DrivesOn::Left));
            CHECK(!(a.answers == wrong.answers));
        }
    }
}

// ---- 7. kerb side at a five-way ------------------------------------------------------------------------------------------

TEST_CASE(junction_table_7_kerb_side_at_a_five_way) {
    // A five-way, laid out by hand. My approach (0) heads +x. Turning from +x a quarter turn the way perp turns (the
    // kerb's way for traffic keeping to DrivesOn::Right), the other four roads lie at: 0 degrees (straight ahead: its
    // traffic faces mine), 60 and 120 degrees (both on my kerb half), and -90 degrees (on my offside). Each approach's
    // heading points from its road into the junction.
    const double kDeg = 3.14159265358979323846 / 180;
    const auto roadAt = [&](double degrees) {   // the heading of traffic arriving from the road at that angle
        return engine::Vec2(-std::cos(degrees * kDeg), -std::sin(degrees * kDeg));
    };
    const std::vector<engine::Vec2> heads = {engine::Vec2(1, 0), roadAt(0), roadAt(60), roadAt(120), roadAt(-90)};
    CHECK(engine::dot(engine::perp(heads[0]) * kerbwardPerpSign(DrivesOn::Right), engine::Vec2(0, 1)) > 0.99);
    // Keeping to DrivesOn::Right's side: of the two on my kerb half, only the first found turning toward the kerb (60 degrees) is on my kerb
    // side; the one facing me and the one on my offside are not.
    CHECK(onKerbSideOf(heads, 2, 0, DrivesOn::Right));
    CHECK(!onKerbSideOf(heads, 3, 0, DrivesOn::Right));
    CHECK(!onKerbSideOf(heads, 1, 0, DrivesOn::Right));
    CHECK(!onKerbSideOf(heads, 4, 0, DrivesOn::Right));
    // Keeping to DrivesOn::Left's side, the kerb is the other way: the road at -90 degrees is the only one on my kerb half, and so on my
    // kerb side.
    CHECK(onKerbSideOf(heads, 4, 0, DrivesOn::Left));
    for (const std::size_t i : {1u, 2u, 3u}) CHECK(!onKerbSideOf(heads, i, 0, DrivesOn::Left));
    // Two approaches facing each other: neither is on the other's kerb side.
    CHECK(!onKerbSideOf(heads, 0, 1, DrivesOn::Right) && !onKerbSideOf(heads, 1, 0, DrivesOn::Right));
}

// ---- 8. the table as text, and the rule set's control table ---------------------------------------------------------------

TEST_CASE(junction_table_8_as_text) {
    TestRoads t = driving_test::gridRoads(3, 3);
    const RuleSet r = rulesWith(signal(true));
    const RightOfWay row = tableFor(t.graph, r);
    const std::string text = describeJunctionRules(t.graph, r, row, t.junction(1, 1));
    std::printf("%s", text.substr(0, 900).c_str());
    CHECK(text.find("control \"signal_arrow\"") != std::string::npos);
    CHECK(text.find("phase 0 (arrow): green:") != std::string::npos);
    CHECK(text.find("the far turn from") != std::string::npos);
    CHECK(text.find("gives way to straight on from") != std::string::npos);
    CHECK(text.find(" left") == std::string::npos && text.find(" right") == std::string::npos);
    CHECK(describeJunctionRules(t.graph, r, row, t.junction(1, 1)) == text);   // same table, same text
    CHECK(describeRightOfWay(row).find("right of way: 9 junctions") == 0);
}

// ---- 14. waiting points (no rules needed) --------------------------------------------------------------------------------

TEST_CASE(junction_plan_14_waiting_points_clear_of_every_stretch) {
    // A wide junction (two lanes each way, generous corners), so some turns pass clear stretches inside it.
    TestRoadSpec spec;
    spec.lanesEachWay = 2;
    spec.cornerRadius = 25;
    TestRoads t = driving_test::gridRoads(3, 3, spec);
    const RightOfWay row = tableFor(t.graph, rulesWith(allWayStop()));
    const engine::Junction& jn = t.graph.junctions[t.junction(1, 1).index];
    int inside = 0;
    for (uint32_t i = 0; i < jn.connectors.count; ++i) {
        const engine::PathId c = t.graph.junctionConnectors[jn.connectors.first + i];
        const engine::Path& path = t.graph.paths[c.index];
        JunctionPlan shortCar, longCar;
        findWaitingPoints(t.graph, row, c, 4.5, shortCar);
        findWaitingPoints(t.graph, row, c, 18.0, longCar);
        CHECK(shortCar.waitingCount >= 1 && shortCar.waitingAt[0] == 0);
        CHECK(longCar.waitingCount <= shortCar.waitingCount);
        for (const JunctionPlan* p : {&shortCar, &longCar}) {
            const Real length = p == &shortCar ? 4.5 : 18.0;
            for (std::size_t w = 1; w < p->waitingCount; ++w) {
                const Real s = p->waitingAt[w];
                CHECK(s > p->waitingAt[w - 1]);
                for (uint32_t k = path.crossings.first; k < path.crossings.end(); ++k) {
                    const engine::CrossingPoint& rec = t.graph.crossings[t.graph.pathCrossings[k].index];
                    if (rec.meeting == engine::Meeting::Split) continue;
                    const Real from = rec.a == c ? rec.aFrom : rec.otherFrom, to = rec.a == c ? rec.aTo : rec.otherTo;
                    CHECK(!(from < s && to > s - length));   // no part of the body [s - length, s] in the stretch
                }
            }
            if (p == &shortCar) inside += static_cast<int>(p->waitingCount) - 1;
        }
    }
    std::printf("    a 4.5 m car can wait inside the two-lane four-way at %d places over its %u connectors\n", inside,
                jn.connectors.count);
    CHECK(inside > 0);
}

#ifdef RT_ENABLE_SCRIPTING

#include "apps/driving/scripting/rule_set_lua.h"
#include "apps/driving/scripting/rules_lua.h"
#include "engine/scripting/script_vm.h"

namespace {

const char* kFile = "d1_test.lua";

bool readRuleSet(const std::string& lua, RuleSet& out, std::string& err) {
    engine::ScriptVM vm;
    return vm.doString(lua, &err) && loadRuleSet(vm, kFile, out, &err);
}

const char* kBase = "rule_set = { name = 'd1', drives_on = 'left', ranks = { 'priority', 'give_way', 'stop' }, devices = {}, "
                    "numbers = {}, personality = {}, ";

}  // namespace

TEST_CASE(junction_table_8b_control_table_read_from_lua) {
    RuleSet r;
    std::string err;
    const std::string good = std::string(kBase) +
                             "control = { signal = { settle = { 'light', 'movement' }, gives_way_on_green = { 'far_turn' },"
                             " arrow = 'far_turn', turn_on_red = { movement = 'near_turn', after = 'full_stop' } },"
                             " all_way_stop = { settle = { 'arrived_first', 'kerb_side', 'movement' }, everyone_stops = true } },"
                             " movement_rank = { 'straight', 'near_turn', 'far_turn', 'u_turn' } }";
    CHECK(readRuleSet(good, r, err));
    if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(r.controls.size() == 2u);
    const ControlKind* s = r.findControl("signal");
    const ControlKind* a = r.findControl("all_way_stop");
    CHECK(s && s->hasLights() && s->settle.size() == 2u && s->turnOnRed && s->turnOnRedAfterFullStop);
    CHECK(s && hasMovement(s->givesWayOnGreen, Movement::FarTurn) && hasMovement(s->arrow, Movement::FarTurn));
    CHECK(a && !a->hasLights() && a->everyoneStops && a->settle[0] == SettleQuestion::ArrivedFirst);
    CHECK(r.movementRankOf(Movement::FarTurn) == 2u);
    // Faults, all reported together: an unknown question, an unknown movement, a lights table on a control with none.
    RuleSet bad;
    const std::string wrong = std::string(kBase) +
                              "control = { x = { settle = { 'who_is_bigger' } }, y = { settle = { 'arrived_first' }, "
                              "arrow = { 'far_turn' } } }, movement_rank = { 'sideways' } }";
    CHECK(!readRuleSet(wrong, bad, err));
    std::printf("%s\n", err.c_str());
    CHECK(err.find("rule_set.control.x.settle names \"who_is_bigger\"") != std::string::npos);
    CHECK(err.find("rule_set.control.y gives") != std::string::npos);
    CHECK(err.find("rule_set.movement_rank names \"sideways\"") != std::string::npos);
    // A junction whose control the rule set lacks is a problem at load.
    TestRoads t = driving_test::gridRoads(2, 2);
    RoadRules rr;
    FactRegistry none;
    std::vector<std::string> problems;
    CHECK(!buildRoadRules(t.graph, r, none, rr, &problems));
    CHECK(problems.size() == 1u && problems[0].find("controlled by \"test_uncontrolled\"") != std::string::npos);
}

// ---- The plans ------------------------------------------------------------------------------------------------------------

namespace {

const char* kRulesLua = R"LUA(
rule_set = { name = 'd1_plans', drives_on = 'left', ranks = {}, devices = {},
  control = { test_uncontrolled = { settle = { 'arrived_first', 'kerb_side', 'movement' } } },
  movement_rank = { 'straight', 'near_turn', 'far_turn', 'u_turn' },
  numbers = { standstill_gap = { 2, unit = 'm', source = 'test' },
              comfortable_side_accel = { 2, unit = 'm/s2', source = 'test' } },
  personality = { gap_kept = { usual = 1.5, low = 1.2, high = 1.8, unit = 's', source = 'test' },
                  speed_habit = { usual = 1, low = 0.95, high = 1.05, unit = 'ratio' } },
  rules = {
    { name = 'speed limit', give = { top_speed = 'my_limit' } },
    { name = 'bend ahead', give = { top_speed = 'corner_speed' } },
    { name = 'lane ends', when = 'lane_ends_ahead', give = { stop_at = 'lane_end' } },
    { name = 'arrive', give = { stop_at = 'destination' } },
  } }
)LUA";

bool giveRules(Driving& d, const engine::LaneGraph& g) {
    FactRegistry facts;
    if (!registerBuiltInFacts(facts) || !registerMoveNeeds(facts)) return false;
    engine::ScriptVM vm;
    std::string err;
    RuleSet ruleSet;
    AnswerVocabulary vocabulary;
    CompiledRules rules;
    if (!vm.doString(kRulesLua, &err) || !loadRuleSet(vm, kFile, ruleSet, &err) ||
        !loadRules(vm, kFile, facts, ruleSet, vocabulary, rules, &err)) {
        std::printf("    rules: %s\n", err.c_str());
        return false;
    }
    RoadRules roadRules;
    std::vector<std::string> found;
    if (!buildRoadRules(g, ruleSet, facts, roadRules, &found)) {
        for (const std::string& f : found) std::printf("    %s\n", f.c_str());
        return false;
    }
    return d.useRules(ruleSet, facts, vocabulary, rules, roadRules, &err);
}

PathPoint at(engine::PathId path, Real along) {
    PathPoint p;
    p.path = path;
    p.along = along;
    return p;
}

void setUp(Driving& d, const engine::LaneGraph& g) {
    VehicleClasses classes;
    classes.add(driving_test::testCarClass());
    d.setVehicleClasses(classes);
    CHECK(giveRules(d, g));
}

// One car, `from` its lane at `along`, bound for `to` at `toAlong`; one tick (its first decision). Its plans after it.
const JunctionPlans* planFor(Driving& d, engine::PathId from, Real along, engine::PathId to, Real toAlong, TripId* trip = nullptr) {
    CHECK(d.placeCar(CarId(1), 0, at(from, along)));
    const TripId id = d.beginTrip(CarId(1), 7, at(to, toAlong));
    CHECK(id.valid());
    if (trip) *trip = id;
    d.tick(1.0 / 60);
    return d.junctionPlans(CarId(1));
}

// Checks the one plan a car arriving at junction (r, c) heading `in` makes for leaving heading `out`.
void checkOnePlan(const TestRoads& t, uint32_t r, uint32_t c, Heading in, Heading out, engine::Turn turn) {
    Driving d(&t.graph);
    setUp(d, t.graph);
    const engine::PathId from = t.laneArriving(r, c, in, 0);
    const Real along = t.graph.paths[from.index].length - 30;
    const JunctionPlans* plans = planFor(d, from, along, t.laneLeaving(r, c, out, 0), 20);
    CHECK(plans && plans->count >= 1);
    if (!plans || plans->count == 0) return;
    const JunctionPlan& p = plans->plans[0];
    const engine::PathId conn = t.connector(r, c, in, 0, out, 0);
    CHECK(p.junction == t.junction(r, c));
    CHECK(p.connector == conn);
    CHECK(p.turn == turn && t.graph.paths[conn.index].turn == turn);
    CHECK(p.crossingCount == t.graph.paths[conn.index].crossings.count);
    CHECK(p.waitingCount >= 1 && p.waitingAt[0] == 0);
    const Car* car = d.car(CarId(1));
    CHECK(car && std::fabs((p.startOdometer - car->odometer) - (t.graph.paths[from.index].length - car->along)) < 1e-6);
    // The driver card shows it, and the trace noted it.
    const std::string card = d.describeCar(CarId(1));
    CHECK(card.find("junction " + std::to_string(p.junction.index) + ": " + turnWords(turn) + " by connector " +
                    std::to_string(conn.index)) != std::string::npos);
    CHECK(card.find("crossing points (" + std::to_string(p.crossingCount)) != std::string::npos);
    CHECK(card.find("waiting points: the line (0.0 m)") != std::string::npos);
    const CarTrace* tr = d.trace(CarId(1));
    CHECK(tr && tr->planNoteCount() >= 1 && tr->planNoteOldestFirst(0).change == PlanChange::Made);
}

}  // namespace

// ---- 9. each turn kind, at a four-way and at a T --------------------------------------------------------------------------

TEST_CASE(junction_plan_9_each_turn_at_a_four_way_and_a_t) {
    const DrivesOn side = DrivesOn::Left;
    {
        const TestRoads t = driving_test::gridRoads(3, 3);
        checkOnePlan(t, 1, 1, kEast, kEast, engine::Turn::Straight);
        checkOnePlan(t, 1, 1, kEast, driving_test::nearTurnFrom(kEast, side), engine::Turn::NearTurn);
        checkOnePlan(t, 1, 1, kEast, driving_test::farTurnFrom(kEast, side), engine::Turn::FarTurn);
        // The far turn's card, printed once for a person to read.
        Driving d(&t.graph);
        setUp(d, t.graph);
        const engine::PathId from = t.laneArriving(1, 1, kEast, 0);
        planFor(d, from, t.graph.paths[from.index].length - 30, t.laneLeaving(1, 1, driving_test::farTurnFrom(kEast, side), 0), 20);
        std::printf("%s", d.describeCar(CarId(1)).c_str());
    }
    {
        // The T at (0, 1): its side road arrives heading south; the main road runs east and west.
        const TestRoads t = driving_test::gridRoads(2, 3);
        for (const Heading in : {kSouth, kEast, kWest})
            for (const Heading out : kHeadings) {
                const engine::PathId c = t.connector(0, 1, in, 0, out, 0);
                if (!c.valid()) continue;
                checkOnePlan(t, 0, 1, in, out, t.graph.paths[c.index].turn);
            }
    }
}

// ---- 10. a four-way entered from each arm --------------------------------------------------------------------------------

TEST_CASE(junction_plan_10_four_way_from_each_arm) {
    const TestRoads t = driving_test::gridRoads(3, 3);
    for (const Heading in : kHeadings) checkOnePlan(t, 1, 1, in, in, engine::Turn::Straight);
}

// ---- 11. two junctions close together -------------------------------------------------------------------------------------

TEST_CASE(junction_plan_11_two_junctions_close_together) {
    TestRoadSpec spec;
    spec.roadLength = 12;   // 12 m between boxes: the second junction is in view before the first is cleared
    const TestRoads t = driving_test::gridRoads(3, 4, spec);
    Driving d(&t.graph);
    setUp(d, t.graph);
    const JunctionPlans* plans = planFor(d, t.laneArriving(1, 1, kEast, 0), 5, t.laneLeaving(1, 2, kEast, 0), 8);
    CHECK(plans && plans->count == 2);
    if (!plans || plans->count != 2) return;
    CHECK(plans->plans[0].junction == t.junction(1, 1) && plans->plans[1].junction == t.junction(1, 2));
    CHECK(plans->plans[0].startOdometer < plans->plans[1].startOdometer);
    const JunctionPlan second = plans->plans[1];
    // Drive on until the first is cleared: the second is the same plan, kept, not made again.
    bool cleared = false;
    for (int i = 0; i < 60 * 30 && !cleared; ++i) {
        d.tick(1.0 / 60);
        const JunctionPlans* now = d.junctionPlans(CarId(1));
        if (!now) break;
        cleared = now->count >= 1 && now->plans[0].junction == t.junction(1, 2);
        if (cleared) CHECK(now->plans[0] == second);
    }
    CHECK(cleared);
    const CarTrace* tr = d.trace(CarId(1));
    bool notedCleared = false;
    for (std::size_t i = 0; tr && i < tr->planNoteCount(); ++i)
        notedCleared = notedCleared || (tr->planNoteOldestFirst(i).change == PlanChange::Cleared &&
                                        tr->planNoteOldestFirst(i).junction == t.junction(1, 1));
    CHECK(notedCleared);
    CHECK(d.junctionPlanCounts().made == 2u && d.junctionPlanCounts().cleared == 1u);
    CHECK(d.junctionPlanCounts().mostAtOnce == 2u);
}

// ---- 12. dropped when the route changes ------------------------------------------------------------------------------------

TEST_CASE(junction_plan_12_dropped_when_the_route_changes) {
    const TestRoads t = driving_test::gridRoads(3, 3);
    Driving d(&t.graph);
    setUp(d, t.graph);
    const engine::PathId from = t.laneArriving(1, 1, kEast, 0);
    TripId trip;
    const JunctionPlans* plans = planFor(d, from, t.graph.paths[from.index].length - 30, t.laneLeaving(1, 1, kEast, 0), 20, &trip);
    CHECK(plans && plans->count == 1 && plans->plans[0].turn == engine::Turn::Straight);
    // A new destination, through the far turn: the plan goes at once, noted; the next decision makes the new one.
    const Heading far = driving_test::farTurnFrom(kEast, DrivesOn::Left);
    CHECK(d.changeDestination(trip, at(t.laneLeaving(1, 1, far, 0), 20)));
    CHECK(d.junctionPlans(CarId(1))->count == 0);
    const CarTrace* tr = d.trace(CarId(1));
    CHECK(tr && tr->planNoteCount() == 2 && tr->planNoteOldestFirst(1).change == PlanChange::RouteChanged);
    d.tick(1.0 / 60);
    plans = d.junctionPlans(CarId(1));
    CHECK(plans->count == 1 && plans->plans[0].connector == t.connector(1, 1, kEast, 0, far, 0));
    const uint64_t madeAt = plans->plans[0].madeAtTick;
    // Routed again to the same place: dropped, and made again (later).
    CHECK(d.reroute(trip));
    CHECK(d.junctionPlans(CarId(1))->count == 0);
    d.tick(1.0 / 60);
    plans = d.junctionPlans(CarId(1));
    CHECK(plans->count == 1 && plans->plans[0].connector == t.connector(1, 1, kEast, 0, far, 0));
    CHECK(plans->plans[0].madeAtTick > madeAt);
    CHECK(d.junctionPlanCounts().routeChanged == 2u && d.junctionPlanCounts().made == 3u);
}

// ---- 13. a refused turn --------------------------------------------------------------------------------------------------

TEST_CASE(junction_plan_13_a_refused_turn) {
    const Heading far = driving_test::farTurnFrom(kEast, DrivesOn::Left);
    TestRoadSpec spec;
    TestRoadSpec::RefusedTurn refused;
    refused.row = 1;
    refused.col = 1;
    refused.in = kEast;
    refused.out = far;
    spec.refused.push_back(refused);
    const TestRoads t = driving_test::gridRoads(3, 3, spec);
    CHECK(!t.connector(1, 1, kEast, 0, far, 0).valid());
    CHECK(t.graph.junctions[t.junction(1, 1).index].connectors.count == 11u);
    // The table has nothing to say about a turn that is not there, and the rest of the junction is settled as before.
    TestRoads whole = driving_test::gridRoads(3, 3);
    engine::LaneGraph g = t.graph;
    const RightOfWay row = tableFor(g, rulesWith(allWayStop()));
    const RightOfWay rowWhole = tableFor(whole.graph, rulesWith(allWayStop()));
    CHECK(row.counts.meetings < rowWhole.counts.meetings);
    // A car bound where only the refused turn leads directly is routed another way, and plans for the turn it takes.
    Driving d(&t.graph);
    setUp(d, t.graph);
    const engine::PathId from = t.laneArriving(1, 1, kEast, 0);
    const JunctionPlans* plans = planFor(d, from, t.graph.paths[from.index].length - 30, t.laneLeaving(1, 1, far, 0), 20);
    CHECK(plans && plans->count >= 1);
    if (!plans || plans->count == 0) return;
    const JunctionPlan& p = plans->plans[0];
    CHECK(p.junction == t.junction(1, 1));
    CHECK(p.connector.valid() && t.graph.paths[p.connector.index].junction == t.junction(1, 1));
    CHECK(p.turn != engine::Turn::FarTurn && t.graph.paths[p.connector.index].turn == p.turn);
}

// ---- 15. the same every run; the state hash untouched by plans -----------------------------------------------------------

namespace {

struct Run {
    uint64_t state = 0, plans = 0;
    JunctionPlanCounts counts;
};
Run runGrid(bool trace) {
    TestRoadSpec spec;
    spec.roadLength = 40;
    const TestRoads t = driving_test::gridRoads(3, 3, spec);
    Driving d(&t.graph);
    DrivingSettings settings;
    settings.trace = trace;
    d.setSettings(settings);
    setUp(d, t.graph);
    // Eight cars, two from each arm of the middle junction, each bound through it a different way.
    uint32_t id = 0;
    for (const Heading in : kHeadings)
        for (const Heading out : {in, driving_test::farTurnFrom(in, DrivesOn::Left)}) {
            const engine::PathId from = t.laneArriving(1, 1, in, 0);
            CHECK(d.placeCar(CarId(id), 0, at(from, 5 + 12 * (id % 2))));
            CHECK(d.beginTrip(CarId(id), id, at(t.laneLeaving(1, 1, out, 0), 30)).valid());
            ++id;
        }
    for (int i = 0; i < 60 * 30; ++i) d.tick(1.0 / 60);
    Run r;
    r.state = d.stateHash();
    r.plans = d.planHash();
    r.counts = d.junctionPlanCounts();
    return r;
}

}  // namespace

TEST_CASE(junction_plan_15_same_every_run) {
    const Run a = runGrid(true), b = runGrid(true), c = runGrid(false);
    std::printf("    %llu plans made, %llu cleared, the most crossing points on one %u\n",
                static_cast<unsigned long long>(a.counts.made), static_cast<unsigned long long>(a.counts.cleared),
                a.counts.mostCrossings);
    CHECK(a.counts.made >= 8u);
    CHECK(a.state == b.state && a.plans == b.plans);
    CHECK(a.state == c.state && a.plans == c.plans);   // tracing changes neither
}

#endif  // RT_ENABLE_SCRIPTING
