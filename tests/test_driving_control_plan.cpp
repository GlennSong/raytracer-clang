// THE CONTROL PLAN MEETS THE GIVE-WAY TABLE (issue #240, task B4). The lab's own rule set and placement policy
// (assets/scripts/rules/lab.lua) decide a junction of the synthetic grid from its facts -- which roads meet, of what
// class, running which way -- and the decision is put on the grid as the lane graph builder carries a plan through:
// the junction's control, and each regulation the decision names at the end of every lane arriving by that approach.
// The table built from it (right_of_way.h) is then checked against what the law says:
//
//   1. a local street crossing an arterial: a two-way stop; the local street's turns give way to the arterial's, by its
//      signs, and the arterial's own turns are settled without them;
//   2. two local streets crossing: an all-way stop; every approach carries the stop sign, everyone stops first, and
//      every meeting is asked while driving (who got there first);
//   3. two arterials crossing: lights; with no light clock yet, the junction is bound to its `when_dark` row (an all-way
//      stop), and says so.

#include "test_framework.h"

#include "driving_test_roads.h"

#include "apps/driving/right_of_way.h"
#include "apps/driving/road_rules.h"

#if defined(RT_ENABLE_SCRIPTING) && defined(RT_ROADS_LANES)

#include "apps/driving/scripting/rule_set_lua.h"
#include "engine/procgen/city/roads/lanes/control_plan.h"
#include "engine/scripting/script_vm.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace driving;
using driving_test::Heading;
using driving_test::TestRoads;
using engine::roads::lanes::ArmFacts;
using engine::roads::lanes::ControlChoice;
using engine::roads::lanes::ControlPolicy;
using engine::roads::lanes::JunctionFacts;
using engine::roads::lanes::PolicyRow;

namespace {

constexpr Heading kHeadings[] = {Heading::East, Heading::North, Heading::West, Heading::South};
constexpr uint32_t kRow = 1, kCol = 1;   // the grid's middle junction: a four-way

// The lab's rule set and placement policy, read from the file the lab ships.
bool labRules(RuleSet& rules, ControlPolicy& policy) {
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/scripts/rules/lab.lua");
    std::stringstream code;
    code << f.rdbuf();
    engine::ScriptVM vm;
    std::string err;
    const bool ok = vm.doString(code.str(), &err) && loadRuleSet(vm, "lab.lua", rules, &err) &&
                    loadPlacementPolicy(vm, "lab.lua", rules, policy, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    return ok;
}

// The middle junction's facts: an arm for every road arriving there (all of the grid's roads are two-way), the class
// of the east-west road and of the north-south one as the test says.
JunctionFacts gridJunction(const TestRoads& t, const std::string& eastWest, const std::string& northSouth,
                           const ControlPolicy& policy) {
    JunctionFacts j;
    j.id = "grid";
    for (const Heading h : kHeadings) {
        if (!t.laneArriving(kRow, kCol, h, 0).valid()) continue;
        ArmFacts a;
        a.roadClass = h == Heading::East || h == Heading::West ? eastWest : northSouth;
        a.road = a.roadClass;
        a.heading = driving_test::headingVector(h);
        a.arriving = a.leaving = static_cast<int>(t.spec.lanesEachWay);
        a.limit = policy.postedLimit(a.roadClass);
        j.arms.push_back(a);
    }
    engine::roads::lanes::settleRoles(j, policy);
    return j;
}

// THE DECISION PUT ON THE GRID as the lane graph builder carries a plan: every junction its control (the rest of the
// grid takes the same, so the rule set knows every junction's), and each regulation the row names for an approach at
// the end of every lane arriving by it. Returns the arms' headings that got a sign.
std::vector<Heading> carry(TestRoads& t, const JunctionFacts& j, const ControlChoice& choice, const ControlPolicy& policy) {
    engine::LaneGraph& g = t.graph;
    g.controlNames = {choice.control};
    for (engine::Junction& jn : g.junctions) jn.control = 0;
    const PolicyRow& row = policy.rows[static_cast<std::size_t>(choice.row)];
    std::map<uint32_t, std::vector<std::string>> kindsOn;   // lane -> kinds at its end
    std::vector<Heading> signed_;
    std::size_t armIndex = 0;
    for (const Heading h : kHeadings) {
        if (!t.laneArriving(kRow, kCol, h, 0).valid()) continue;
        const ArmFacts& a = j.arms[armIndex++];
        const std::vector<std::string>& kinds = a.major ? row.majorSigns : row.minorSigns;
        if (kinds.empty()) continue;
        signed_.push_back(h);
        for (uint32_t k = 0; k < t.spec.lanesEachWay; ++k)
            for (const std::string& kind : kinds) kindsOn[t.laneArriving(kRow, kCol, h, k).index].push_back(kind);
    }
    g.regulationKindNames.clear();
    for (const std::pair<const uint32_t, std::vector<std::string>>& l : kindsOn)
        g.regulationKindNames.insert(g.regulationKindNames.end(), l.second.begin(), l.second.end());
    std::sort(g.regulationKindNames.begin(), g.regulationKindNames.end());
    g.regulationKindNames.erase(std::unique(g.regulationKindNames.begin(), g.regulationKindNames.end()), g.regulationKindNames.end());
    g.regulations.clear();
    for (uint32_t p = 0; p < g.paths.size(); ++p) {
        g.paths[p].regulations = engine::Range{static_cast<uint32_t>(g.regulations.size()), 0};
        const std::map<uint32_t, std::vector<std::string>>::const_iterator it = kindsOn.find(p);
        if (it == kindsOn.end()) continue;
        for (const std::string& kind : it->second) {
            engine::Regulation reg;
            reg.kind = engine::LaneGraph::nameIndex(g.regulationKindNames, kind);
            reg.path = engine::PathId(p);
            reg.from = reg.to = g.paths[p].length;
            g.regulations.push_back(reg);
        }
        g.paths[p].regulations.count = static_cast<uint32_t>(g.regulations.size()) - g.paths[p].regulations.first;
    }
    CHECK(engine::validate(g).empty());
    return signed_;
}

// The heading each connector of the middle junction arrives on.
std::map<uint32_t, Heading> arrivingHeadingOf(const TestRoads& t) {
    std::map<uint32_t, Heading> out;
    for (const Heading in : kHeadings)
        for (const Heading o : kHeadings)
            for (uint32_t k = 0; k < t.spec.lanesEachWay; ++k)
                for (uint32_t m = 0; m < t.spec.lanesEachWay; ++m) {
                    const engine::PathId c = t.connector(kRow, kCol, in, k, o, m);
                    if (c.valid()) out[c.index] = in;
                }
    return out;
}

struct Built {
    RuleSet rules;
    RoadRules roadRules;
    RightOfWay row;
};
bool buildTable(engine::LaneGraph& g, Built& b) {
    FactRegistry none;
    std::vector<std::string> problems;
    const bool ok = buildRoadRules(g, b.rules, none, b.roadRules, &problems);
    for (const std::string& p : problems) std::printf("    %s\n", p.c_str());
    if (ok) b.row = buildRightOfWay(g, b.rules, b.roadRules);
    return ok;
}

}  // namespace

// ---- 1. a two-way stop ---------------------------------------------------------------------------------------------------

TEST_CASE(control_plan_meets_the_table_1_a_two_way_stop) {
    Built b;
    ControlPolicy policy;
    CHECK(labRules(b.rules, policy));
    TestRoads t = driving_test::gridRoads(3, 3);
    const JunctionFacts j = gridJunction(t, "arterial", "local", policy);
    const ControlChoice choice = engine::roads::lanes::chooseControl(j, policy, {});
    std::printf("    %s -> %s (\"%s\")\n", engine::roads::lanes::describeFacts(j).c_str(), choice.control.c_str(), choice.why.c_str());
    CHECK(choice.control == "stop");
    if (choice.row < 0) return;
    const std::vector<Heading> signed_ = carry(t, j, choice, policy);
    CHECK(signed_ == (std::vector<Heading>{Heading::North, Heading::South}));
    CHECK(buildTable(t.graph, b));
    const engine::Junction& jn = t.graph.junctions[t.junction(kRow, kCol).index];
    std::printf("%s", describeJunctionRules(t.graph, b.rules, b.row, t.junction(kRow, kCol)).c_str());
    const std::map<uint32_t, Heading> from = arrivingHeadingOf(t);
    auto minor = [&](uint32_t path) { const Heading h = from.at(path); return h == Heading::North || h == Heading::South; };
    int bySign = 0, majorPairs = 0;
    for (uint32_t k = jn.crossings.first; k < jn.crossings.end(); ++k) {
        const engine::CrossingPoint& rec = t.graph.crossings[k];
        if (rec.otherSide != engine::OtherSide::Path || rec.meeting == engine::Meeting::Split) continue;
        const MeetingAnswer* a = b.row.answer(engine::CrossingId(k));
        CHECK(a != nullptr);
        if (!a) continue;
        const bool aMinor = minor(rec.a.index), otherMinor = minor(rec.other);
        if (aMinor != otherMinor) {
            // the local street's turn gives way to the arterial's, by its sign
            CHECK(a->by == SettledBy::SignRank);
            CHECK(a->who == (aMinor ? GivesWay::A : GivesWay::Other));
            ++bySign;
        } else {
            CHECK(a->by != SettledBy::SignRank);
            majorPairs += aMinor ? 0 : 1;
        }
    }
    CHECK(bySign > 0 && majorPairs > 0);
    // the table's approaches: the local street's carry the stop sign's rank, the arterial's the first (no sign)
    const JunctionRules& jr = b.row.junctions[t.junction(kRow, kCol).index];
    for (uint32_t a = 0; a < jr.approaches.count; ++a) {
        const Approach& ap = b.row.approaches[jr.approaches.first + a];
        const bool northSouth = std::fabs(ap.heading.y) > std::fabs(ap.heading.x);   // the local street's
        CHECK(ap.signRank == (northSouth ? b.rules.rankIndex("stop") : 0));
    }
    CHECK(!b.rules.controls[jr.control].everyoneStops);
}

// ---- 2. an all-way stop --------------------------------------------------------------------------------------------------

TEST_CASE(control_plan_meets_the_table_2_an_all_way_stop) {
    Built b;
    ControlPolicy policy;
    CHECK(labRules(b.rules, policy));
    TestRoads t = driving_test::gridRoads(3, 3);
    const JunctionFacts j = gridJunction(t, "local", "local", policy);
    const ControlChoice choice = engine::roads::lanes::chooseControl(j, policy, {});
    std::printf("    %s -> %s (\"%s\")\n", engine::roads::lanes::describeFacts(j).c_str(), choice.control.c_str(), choice.why.c_str());
    CHECK(choice.control == "all_way_stop");
    if (choice.row < 0) return;
    CHECK(carry(t, j, choice, policy).size() == 4u);   // every approach stops
    CHECK(buildTable(t.graph, b));
    const JunctionRules& jr = b.row.junctions[t.junction(kRow, kCol).index];
    CHECK(b.rules.controls[jr.control].everyoneStops);
    for (uint32_t a = 0; a < jr.approaches.count; ++a)
        CHECK(b.row.approaches[jr.approaches.first + a].signRank == b.rules.rankIndex("stop"));
    const engine::Junction& jn = t.graph.junctions[t.junction(kRow, kCol).index];
    int asked = 0;
    for (uint32_t k = jn.crossings.first; k < jn.crossings.end(); ++k) {
        const engine::CrossingPoint& rec = t.graph.crossings[k];
        if (rec.otherSide != engine::OtherSide::Path || rec.meeting == engine::Meeting::Split) continue;
        const MeetingAnswer* a = b.row.answer(engine::CrossingId(k));
        CHECK(a && a->who == GivesWay::Ask && a->by == SettledBy::ArrivedFirst);
        ++asked;
    }
    CHECK(asked > 0);
}

// ---- 3. lights, dark -----------------------------------------------------------------------------------------------------

TEST_CASE(control_plan_meets_the_table_3_lights_with_no_clock_are_treated_as_their_dark_row) {
    Built b;
    ControlPolicy policy;
    CHECK(labRules(b.rules, policy));
    TestRoads t = driving_test::gridRoads(3, 3);
    const JunctionFacts j = gridJunction(t, "arterial", "arterial", policy);
    const ControlChoice choice = engine::roads::lanes::chooseControl(j, policy, {});
    CHECK(choice.control == "signal");
    if (choice.row < 0) return;
    CHECK(carry(t, j, choice, policy).empty());   // lights stand no sign on an approach
    CHECK(buildTable(t.graph, b));
    const ControlKind* signal = b.rules.findControl("signal");
    CHECK(signal && signal->hasLights() && !signal->whenDark.empty());
    CHECK(!kLightsRunAProgramme);
    CHECK(b.roadRules.darkOfName.size() == 1u && b.roadRules.darkOfName[0] == 1);
    const JunctionRules& jr = b.row.junctions[t.junction(kRow, kCol).index];
    CHECK(b.rules.controls[jr.control].name == signal->whenDark);
    CHECK(jr.phases.count == 0u);
    CHECK(b.row.counts.junctionsDark == static_cast<uint32_t>(t.graph.junctions.size()));
    const std::string text = describeJunctionRules(t.graph, b.rules, b.row, t.junction(kRow, kCol));
    CHECK(text.find("\"signal\", its lights dark: treated as control \"" + signal->whenDark + "\"") != std::string::npos);
}

// ---- 4. the placement policy's faults --------------------------------------------------------------------------------

TEST_CASE(control_plan_meets_the_table_4_a_faulty_placement_policy_is_refused) {
    const std::string lua = R"LUA(
rule_set = { name = 'p', drives_on = 'left', ranks = { 'priority', 'stop' },
  devices = { stop = { means = { stop_line = true, rank = 'stop' } } },
  control = { lights = { settle = { 'light', 'movement' }, when_dark = 'nothing' },
              nothing = { settle = { 'arrived_first' } } },
  numbers = {}, personality = {},
  placement = {
    class_order = { 'big', 'small' },
    speed_limits = { big = { 30, unit = 's', source = 'test' } },
    junctions = {
      { name = 'lights', when = { major = 'big' }, control = 'lights' },
      { name = 'unknown control', control = 'roundabout' },
      { name = 'unknown sign', control = 'nothing', signs = { minor = 'yield' } },
      { name = 'last asks', when = { arms = { min = 3 } }, control = 'nothing' },
    } } }
)LUA";
    engine::ScriptVM vm;
    std::string err;
    RuleSet rules;
    ControlPolicy policy;
    CHECK(vm.doString(lua, &err) && loadRuleSet(vm, "p.lua", rules, &err));
    CHECK(!loadPlacementPolicy(vm, "p.lua", rules, policy, &err));
    std::printf("    %s\n", err.c_str());
    CHECK(err.find("rule_set.placement.speed_limits.big.unit is \"s\", which is a time; a limit is a speed") != std::string::npos);
    CHECK(err.find("rule_set.placement.junctions[2].control names \"roundabout\", which rule_set.control has no row for") !=
          std::string::npos);
    CHECK(err.find("rule_set.placement.junctions[3].signs.minor names \"yield\", which rule_set.devices has no device for") !=
          std::string::npos);
    CHECK(err.find("rule_set.placement.junctions[4] asks something") != std::string::npos);
    CHECK(policy.rows.empty());   // left untouched
    // A dark row naming a control with lights of its own, or none at all, is refused with the rule set.
    const std::string darkLua = R"LUA(
rule_set = { name = 'q', drives_on = 'left', ranks = {}, devices = {}, numbers = {}, personality = {},
  control = { a = { settle = { 'light' }, when_dark = 'b' }, b = { settle = { 'light' } },
              c = { settle = { 'light' }, when_dark = 'zz' } } }
)LUA";
    engine::ScriptVM vm2;
    RuleSet dark;
    CHECK(vm2.doString(darkLua, &err));
    CHECK(!loadRuleSet(vm2, "q.lua", dark, &err));
    CHECK(err.find("rule_set.control.a.when_dark names \"b\", which has lights of its own") != std::string::npos);
    CHECK(err.find("rule_set.control.c.when_dark names \"zz\", which is not a control here") != std::string::npos);
}

#endif
