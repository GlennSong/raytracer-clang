// THE CONTROL PLAN (issue #240, task B4; engine/procgen/city/roads/lanes/control_plan.h): every junction's control
// and every road's posted limit, chosen by the planner from a placement policy, written into the road graph, carried
// through and checked by the lane graph builder.
//
//   1. the policy, on hand-made junction descriptions: lights where arterials cross, a stop on a local street meeting
//      an arterial, an all-way stop where local streets cross, a give-way on a T's stem, nothing at a bend; the major
//      road found by class, then by the one road running straight through; limits as conditions; lights kept apart;
//      no row holding;
//   2. a small road graph planned, written, built: each built junction carries its planned control, the stop signs stand
//      at the ends of the minor road's arriving lanes only, every lane and connector posts its road's limit, and the
//      builder reports nothing about the plan;
//   3. a junction the plan does not name, and a sign on an arm no lane arrives by: each reported with its place, the
//      junction left not planned (never filled in);
//   4. the regulations survive the lane graph's save and load;
//   5. the same plan every run, and the same graph and dump from it.
//
// The class, control and regulation names here are the test's own words: the planner compares them as text.

#include "test_framework.h"

#include "engine/bundle/binary_stream.h"
#include "engine/procgen/city/roads/lane_graph.h"
#include "engine/procgen/city/roads/lanes/control_plan.h"
#include "engine/procgen/city/roads/lanes/deck_mesh.h"     // junctionAreas: the stated junctions' stop lines
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/road_layout.h"   // the streets: where one only carries on
#include "engine/procgen/city/roads/road_model.h"          // assembleRoadModel: the model and its lane level

#include <algorithm>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <vector>

using namespace engine;
using namespace engine::roads::lanes;

namespace {

constexpr double kArterialLimit = 15.0, kLocalLimit = 11.0;   // m/s, the test policy's posted limits
constexpr double kLightsApart = 150.0;                        // m

// THE TEST POLICY: rows like the lab's, in the test's own words.
ControlPolicy testPolicy() {
    ControlPolicy p;
    p.name = "test";
    p.classOrder = {"arterial", "local", "oneway"};
    p.speedLimits = {{"arterial", kArterialLimit}, {"local", kLocalLimit}};
    PolicyRow bend;
    bend.name = "a bend";
    bend.arms.set = true;
    bend.arms.hi = 2;
    bend.control = "unmarked";
    PolicyRow lights;
    lights.name = "arterials cross";
    lights.major = {"arterial"};
    lights.minor = {"arterial"};
    lights.approaches.set = true;
    lights.approaches.lo = 3;
    lights.apart = kLightsApart;
    lights.control = "signal";
    PolicyRow stop;
    stop.name = "a local street meets an arterial";
    stop.major = {"arterial"};
    stop.minor = {"local"};
    stop.hasMinor = YesNo::Yes;
    stop.control = "stop";
    stop.minorSigns = {"stop"};
    PolicyRow allWay;
    allWay.name = "local streets cross";
    allWay.major = {"local"};
    allWay.minor = {"local"};
    allWay.approaches.set = true;
    allWay.approaches.lo = 4;
    allWay.control = "all_way_stop";
    allWay.majorSigns = allWay.minorSigns = {"stop"};
    PolicyRow giveWay;
    giveWay.name = "a local T";
    giveWay.major = {"local"};
    giveWay.hasMinor = YesNo::Yes;
    giveWay.control = "give_way";
    giveWay.minorSigns = {"give_way"};
    PolicyRow rest;
    rest.name = "anything else";
    rest.control = "unmarked";
    p.rows = {bend, lights, stop, allWay, giveWay, rest};
    return p;
}

// One arm arriving heading `heading` (a compass direction on the plan: the way its traffic goes into the junction).
struct TestArm {
    ArmFacts facts;
    Vec2 heading;
};
TestArm arm(const std::string& roadClass, const Vec2& heading, int arriving, int leaving, const ControlPolicy& p) {
    TestArm a;
    a.facts.road = roadClass + "_road";
    a.facts.roadClass = roadClass;
    a.heading = heading;
    a.facts.arriving = arriving;
    a.facts.leaving = leaving;
    a.facts.limit = p.postedLimit(roadClass);
    return a;
}
const Vec2 kEast(1, 0), kWest(-1, 0), kNorth(0, 1), kSouth(0, -1);

// A junction of these arms with its through pairs STATED: `through` as given, or (null) as the one rule states them
// from the ways the arms leave -- what the layout writes into a road file for it.
JunctionFacts junction(const std::vector<TestArm>& arms, const ControlPolicy& p, const Vec2& at = Vec2(),
                       const ThroughPairs* through = nullptr) {
    JunctionFacts j;
    j.id = "test";
    j.at = at;
    std::vector<Vec2> leaves;
    for (const TestArm& a : arms) {
        j.arms.push_back(a.facts);
        leaves.push_back(a.heading * -1.0);
    }
    j.through = through ? *through : throughPairs(leaves);
    settleRoles(j, p);
    return j;
}

std::string decide(const JunctionFacts& j, const ControlPolicy& p, const std::vector<Decided>& before = {}) {
    const ControlChoice c = chooseControl(j, p, before);
    std::printf("    %s -> %s (\"%s\")\n", describeFacts(j).c_str(), c.control.c_str(), c.why.c_str());
    return c.control;
}

}  // namespace

// ---- 1. the policy ---------------------------------------------------------------------------------------------------

TEST_CASE(control_plan_1_the_policy_decides_from_the_facts) {
    const ControlPolicy p = testPolicy();
    // Two arterials crossing: lights. Every approach major (no minor road), and the minor facts read over them all.
    JunctionFacts big = junction({arm("arterial", kEast, 2, 2, p), arm("arterial", kWest, 2, 2, p), arm("arterial", kNorth, 2, 2, p),
                                  arm("arterial", kSouth, 2, 2, p)}, p);
    CHECK(!big.hasMinor && big.majorClass == "arterial" && big.minorClass == "arterial" && big.majorLanes == 2);
    CHECK(decide(big, p) == "signal");
    // ... but not within kLightsApart of lights already placed: the next row that holds decides (here the last).
    CHECK(decide(big, p, {Decided{Vec2(100, 0), "signal"}}) == "unmarked");
    CHECK(decide(big, p, {Decided{Vec2(200, 0), "signal"}}) == "signal");
    CHECK(decide(big, p, {Decided{Vec2(100, 0), "stop"}}) == "signal");
    // A local street crossing an arterial: the arterial is the major road by class, the local street stops.
    JunctionFacts cross = junction({arm("arterial", kEast, 2, 2, p), arm("arterial", kWest, 2, 2, p), arm("local", kNorth, 1, 1, p),
                                    arm("local", kSouth, 1, 1, p)}, p);
    CHECK(cross.hasMinor && cross.minorClass == "local" && cross.minorLanes == 1);
    CHECK(cross.arms[0].major && cross.arms[1].major && !cross.arms[2].major && !cross.arms[3].major);
    CHECK(cross.majorLimit == kArterialLimit && cross.minorLimit == kLocalLimit);
    CHECK(decide(cross, p) == "stop");
    // Local streets crossing: an all-way stop.
    JunctionFacts locals = junction({arm("local", kEast, 1, 1, p), arm("local", kWest, 1, 1, p), arm("local", kNorth, 1, 1, p),
                                     arm("local", kSouth, 1, 1, p)}, p);
    CHECK(!locals.hasMinor);
    CHECK(decide(locals, p) == "all_way_stop");
    // A T of local streets: the one road running straight through is major, the stem gives way.
    JunctionFacts tee = junction({arm("local", kEast, 1, 1, p), arm("local", kWest, 1, 1, p), arm("local", kNorth, 1, 1, p)}, p);
    CHECK(tee.hasMinor && tee.arms[0].major && tee.arms[1].major && !tee.arms[2].major);
    CHECK(decide(tee, p) == "give_way");
    // A Y (no two arms running straight through): no minor road; not four approaches: nothing applies but the last row.
    const double r = std::sqrt(0.5);
    JunctionFacts y = junction({arm("local", kEast, 1, 1, p), arm("local", Vec2(-r, r), 1, 1, p), arm("local", Vec2(-r, -r), 1, 1, p)}, p);
    CHECK(!y.hasMinor);
    CHECK(decide(y, p) == "unmarked");
    // A bend: two arms.
    CHECK(decide(junction({arm("local", kEast, 1, 1, p), arm("local", kNorth, 1, 1, p)}, p), p) == "unmarked");
    // An arm nothing arrives by is no approach: a one-way street leaving a T of locals leaves a plain through road.
    JunctionFacts out = junction({arm("local", kEast, 1, 1, p), arm("local", kWest, 1, 1, p), arm("oneway", kNorth, 0, 1, p)}, p);
    CHECK(out.approaches == 2 && !out.hasMinor);
    CHECK(decide(out, p) == "unmarked");
    // A limit as a condition: a row asking for a faster major road holds only where one is posted.
    ControlPolicy fast = p;
    PolicyRow quick;
    quick.name = "a fast major road";
    quick.majorLimit.set = true;
    quick.majorLimit.lo = kArterialLimit;
    quick.control = "fast";
    fast.rows.insert(fast.rows.begin(), quick);
    CHECK(decide(cross, fast) == "fast");
    CHECK(decide(locals, fast) == "all_way_stop");
    ControlPolicy noPosted = fast;
    noPosted.speedLimits.clear();
    CHECK(decide(junction({arm("arterial", kEast, 2, 2, noPosted), arm("local", kNorth, 1, 1, noPosted)}, noPosted), noPosted) == "unmarked");
    // A policy with no row for a junction says so (-1): the plan reports it and leaves the junction out.
    ControlPolicy partial = p;
    partial.rows.pop_back();
    CHECK(chooseControl(y, partial, {}).row == -1);
}

// WHICH ROAD IS THE MAJOR ONE IS READ FROM THE STATED THROUGH PAIR (ADR-0432), never measured again. A T of three local
// streets: west and east lie in one straight line, the third comes up from the south. The road file STATES the west and
// south arms as one road carrying on (the main road turns the corner there). So the east arm is the minor road and gives
// way. Measuring the mouths instead (west and east head on) made the south arm give way: the old answer.
TEST_CASE(control_plan_1b_the_major_road_is_the_stated_through_pair) {
    const ControlPolicy p = testPolicy();
    // the facts alone: the same three arms, the pair stated two ways
    const std::vector<TestArm> arms = {arm("local", kEast, 1, 1, p), arm("local", kWest, 1, 1, p), arm("local", kNorth, 1, 1, p)};
    const ThroughPairs corner = {{0, 2}};   // the arm from the west (heading east) and the one from the south
    const JunctionFacts turned = junction(arms, p, Vec2(), &corner);
    CHECK(turned.hasMinor && turned.arms[0].major && turned.arms[2].major && !turned.arms[1].major);
    const ThroughPairs none = {};
    CHECK(!junction(arms, p, Vec2(), &none).hasMinor);   // no pair stated: no road runs through, none is minor

    // the whole way, through a road file: its layout written, then the pair restated by hand
    nlohmann::json g;
    g["name"] = "through_test";
    auto road = [](const std::string& id, const Vec2& a, const Vec2& b) {
        return nlohmann::json{{"id", id}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}};
    };
    g["edges"] = nlohmann::json::array({road("w", Vec2(-200, 0), Vec2(0, 0)), road("e", Vec2(0, 0), Vec2(200, 0)),
                                        road("s", Vec2(0, -150), Vec2(0, 0))});
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    CHECK(g["junctions"].size() == 1u);
    nlohmann::json& j = g["junctions"][0];
    std::map<std::string, int> armOf;
    for (std::size_t a = 0; a < j["arms"].size(); ++a) armOf[j["arms"][a]["road"].get<std::string>()] = static_cast<int>(a);
    // as the one rule states it: west and east
    CHECK(j["through"] == nlohmann::json::array({nlohmann::json::array({std::min(armOf["w"], armOf["e"]), std::max(armOf["w"], armOf["e"])})}));
    auto givesWay = [&](const nlohmann::json& scene) {
        const ControlPlan plan = planControls(RoadLabGraph::fromJson(scene), p);
        std::set<std::string> roads;
        for (const ModelJunction& pj : plan.junctions)
            for (const ModelArm& a : pj.arms)
                if (!a.regulations.empty() && a.regulations.front() == "give_way") roads.insert(a.road);
        return roads;
    };
    CHECK(givesWay(g) == std::set<std::string>({"s"}));
    // ... restated: the main road is west and south
    j["through"] = nlohmann::json::array({nlohmann::json::array({std::min(armOf["w"], armOf["s"]), std::max(armOf["w"], armOf["s"])})});
    CHECK(givesWay(g) == std::set<std::string>({"e"}));
}

// THE STOP-LINE SETBACK IS ONE RULE (issue #292): the grouping of road ends into junctions and the stated junctions'
// areas ask the same function (road_model_parts.h stopLineBack), so the same street carrying on is never "a road crossing
// it". The scene: a local street `a`, 32 m long, between two four-way crossings of local streets. At its east end it
// carries on as an ARTERIAL (`b`, half width 11 m); the streets crossing there are local (half width 6 m). Its stop line
// at each end stands back by the crossing local street's half width and 5.2 m: 11.2 m, so 32 - 22.4 = 9.6 m of road lie
// between its two stop lines, more than the 6.5 m a car needs to wait (rules.storage): TWO junctions. (The grouping's
// old copy of the rule took the arterial it carries on into as crossing it: 16.2 m back at that end, 4.6 m between,
// one junction.)
TEST_CASE(control_plan_1c_the_grouping_and_the_areas_ask_one_stop_line_rule) {
    nlohmann::json g;
    g["name"] = "stop_line_test";
    auto road = [](const std::string& id, const std::string& cls, const Vec2& a, const Vec2& b) {
        return nlohmann::json{{"id", id}, {"class", cls}, {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}};
    };
    g["edges"] = nlohmann::json::array(
        {road("p", "local", Vec2(-232, 0), Vec2(-32, 0)), road("a", "local", Vec2(-32, 0), Vec2(0, 0)),
         road("b", "arterial", Vec2(0, 0), Vec2(200, 0)), road("q0", "local", Vec2(-32, -150), Vec2(-32, 0)),
         road("q1", "local", Vec2(-32, 0), Vec2(-32, 150)), road("c0", "local", Vec2(0, -150), Vec2(0, 0)),
         road("c1", "local", Vec2(0, 0), Vec2(0, 150))});
    const RoadLabGraph plain = RoadLabGraph::fromJson(g);
    CHECK(approxEqual(plain.hw(*plain.find("b")), 11.0, kSamePoint) && approxEqual(plain.hw(*plain.find("c0")), 6.0, kSamePoint));
    CHECK(approxEqual(plain.rules.storage, 6.5, kSamePoint));
    const std::vector<JunctionFacts> found = findJunctions(plain, testPolicy());
    CHECK(found.size() == 2u);
    for (const JunctionFacts& j : found) CHECK(j.arms.size() == 4u && j.inside.empty());
    // the layout written, the stated junctions' areas give street `a` the same two stop lines
    writeLayout(g, deriveLayout(plain, LayoutSource::Inferred));
    const RoadLabGraph laid = RoadLabGraph::fromJson(g);
    CHECK(laid.junctions.size() == 2u);
    const int a = laid.index.at("a");
    std::vector<double> stopLines;
    for (const JunctionArea& A : junctionAreas(laid))
        for (const ArmStop& arm : A.arms)
            if (arm.edge == a) stopLines.push_back(arm.stopLine);
    std::sort(stopLines.begin(), stopLines.end());
    CHECK(stopLines.size() == 2u);
    if (stopLines.size() == 2u) {
        CHECK(approxEqual(stopLines[0], 11.2, kSamePoint));   // back from the west crossing: 6 + 5.2
        CHECK(approxEqual(stopLines[1], 20.8, kSamePoint));   // back from the east crossing: 32 - (6 + 5.2)
    }
    // AN ARM NAMING A STREET THE FILE DOES NOT HAVE is refused when the file is read, like any other unknown id (issue
    // #292: it once read as an arm with no street, and nothing said so)
    g["junctions"][0]["arms"][0]["street"] = "no_such_street";
    bool refused = false;
    try {
        RoadLabGraph::fromJson(g);
    } catch (const std::exception& e) {
        refused = std::string(e.what()).find("names unknown street no_such_street") != std::string::npos;
    }
    CHECK(refused);
}

// ---- 2. a road graph planned, written and built ------------------------------------------------------------------------

namespace {

// A SMALL ROAD GRAPH: an arterial west to east through two junctions; at (0, 0) a local street crosses it; at (200, 0)
// a local street ends on it from the south and a one-way street leaves it to the north; the crossing local street
// bends at (0, 150). Flat ground.
nlohmann::json smallGraph() {
    nlohmann::json g;
    g["name"] = "control_test";
    g["classes"]["oneway"] = {{"w", 3.5}, {"fwd", 1}, {"back", 0}, {"sidewalk", 2.0}, {"g_max", 0.09}, {"window", 40}, {"thick", 0.5}};
    auto road = [](const std::string& id, const std::string& cls, const Vec2& a, const Vec2& b) {
        return nlohmann::json{{"id", id}, {"class", cls}, {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}};
    };
    g["edges"] = nlohmann::json::array({road("a0", "arterial", Vec2(-200, 0), Vec2(0, 0)), road("a1", "arterial", Vec2(0, 0), Vec2(200, 0)),
                                        road("a2", "arterial", Vec2(200, 0), Vec2(400, 0)), road("l0", "local", Vec2(0, -150), Vec2(0, 0)),
                                        road("l1", "local", Vec2(0, 0), Vec2(0, 150)), road("l2", "local", Vec2(200, -150), Vec2(200, 0)),
                                        road("o0", "oneway", Vec2(200, 0), Vec2(200, 150)), road("l3", "local", Vec2(0, 150), Vec2(120, 220))});
    return g;
}

LaneGraphOptions testVehicles() {
    LaneGraphOptions o;
    VehicleSize car;
    car.tag = "test_car";
    car.turningRadius = 5.0;
    car.width = 1.9;
    o.vehicles = {car};
    return o;
}

// The small graph with its plan written in, and its layout (the streets, which say where one only carries on).
nlohmann::json plannedGraph(ControlPlan* planOut = nullptr) {
    nlohmann::json g = smallGraph();
    const ControlPlan plan = planControls(RoadLabGraph::fromJson(g), testPolicy());
    writeControlPlan(g, plan);
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    g["lane_rules"] = {{"solid_before_stop_line", 20.0}};   // the lane level's rules (ADR-0426)
    if (planOut) *planOut = plan;
    return g;
}

// The road model built from a road graph's JSON (its lane level is `model.laneLevel`; a junction's control and a
// path's posted limit are asked of the model), with what building the lane level reported.
struct BuiltLaneLevel : LaneGraphBuild {
    RoadModel model;
};
BuiltLaneLevel laneGraphFrom(const nlohmann::json& g) {
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    BuiltLaneLevel out;
    LaneGraphBuild report;
    out.model = assembleRoadModel(*r, testVehicles(), &report);
    static_cast<LaneGraphBuild&>(out) = std::move(report);
    return out;
}

const BuiltLaneLevel& plannedBuild() {
    static const BuiltLaneLevel lg = laneGraphFrom(plannedGraph());
    return lg;
}

bool hasProblem(const BuiltLaneLevel& lg, const std::string& words) {
    for (const std::string& p : engine::problemLines(lg.problems))
        if (p.find(words) != std::string::npos) return true;
    return false;
}

// The road (its id) a lane path was cut from: its section's road, through the build's records.
std::string roadOf(const BuiltLaneLevel& lg, const RoadLabGraph& g, PathId lane) {
    const uint32_t s = lg.model.laneLevel.paths[lane.index].section.index;
    return s < lg.sectionSources.size() ? g.edges[static_cast<std::size_t>(lg.sectionSources[s].edge)].id : std::string();
}

// The regulation kinds standing at the end of `lane`, as the road model answers (RoadModel::regulationsAlong).
std::vector<std::string> kindsAtEnd(const RoadModel& m, PathId lane) {
    std::vector<std::string> out;
    const Real length = m.laneLevel.paths[lane.index].length;
    for (const PathRegulation& r : m.regulationsAlong(lane))
        if (approxEqual(r.from, length, kSamePoint) && approxEqual(r.to, length, kSamePoint)) out.push_back(r.kind);
    return out;
}

}  // namespace

TEST_CASE(control_plan_2_a_planned_road_graph_is_carried_through) {
    ControlPlan plan;
    const nlohmann::json g = plannedGraph(&plan);
    std::printf("    %s", describeControlPlan(plan).c_str());
    CHECK(plan.problems.empty());
    // The plan as written: two junctions, the crossing and the T, each stopping the local street. The bend is no junction
    // (ADR-0419: a street carrying on, as the layout's streets say); the builder's own junction there takes the plan's
    // carry-on control, the policy's for a place of two arms: nothing.
    CHECK(g["junctions"].size() == 2u);
    CHECK(g["junctions"][0]["control"] == "stop" && g["junctions"][1]["control"] == "stop");
    CHECK(plan.carryOns == 1u && g.value("carry_on_control", std::string()) == "unmarked");
    // The one-way street's arm has nothing arriving, so no sign is planned on it; the local streets' arms carry "stop".
    for (const nlohmann::json& a : g["junctions"][1]["arms"]) {
        const bool signed_ = a.contains("regulations");
        CHECK(signed_ == (a["road"] == "l2"));
    }
    // Every road a car drives posts its class's limit; the one-way class has none in the test policy.
    for (const nlohmann::json& e : g["edges"]) {
        const std::string cls = e["class"];
        if (cls == "oneway") CHECK(!e.contains("speed_limit"));
        else CHECK(e.contains("speed_limit") && e["speed_limit"] == (cls == "arterial" ? kArterialLimit : kLocalLimit));
    }

    const BuiltLaneLevel& lg = plannedBuild();
    const LaneGraph& G = lg.model.laneLevel;
    const RoadLabGraph road = RoadLabGraph::fromJson(g);
    std::printf("    controls: %u stated junctions, %zu lane-level junctions\n", lg.controlPlan.planned, G.junctions.size());
    for (const std::string& p : engine::problemLines(lg.problems)) std::printf("    problem: %s\n", p.c_str());
    CHECK(lg.error.empty());
    CHECK(lg.controlPlan.planned == road.junctions.size());
    // ONE JUNCTION PER STATED JUNCTION (the lane level is a table built from the road model, ADR-0426): the bend is no
    // junction (its streets carry on, their lanes joined lane to lane), so two, each carrying its stated junction's
    // control; the stop signs stand where the local streets arrive.
    CHECK(G.junctions.size() == 2u);
    uint32_t stopsSeen = 0;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const Junction& jn = G.junctions[j];
        const ModelJunction& pj = road.junctions[jn.modelJunction];
        CHECK(lg.model.controlAt(JunctionId(j)) == pj.control);
        for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k) {
            const PathId lane = G.junctionLanes[k];
            const std::string by = roadOf(lg, road, lane);
            const std::vector<std::string> kinds = kindsAtEnd(lg.model, lane);
            const bool local = by == "l0" || by == "l1" || by == "l2";
            CHECK(kinds == (local && pj.control == "stop" ? std::vector<std::string>{"stop"} : std::vector<std::string>{}));
            stopsSeen += static_cast<uint32_t>(kinds.size());
        }
    }
    CHECK(stopsSeen == 3u);   // l0 and l1 at the crossing, l2 at the T (one lane each)
    // Posted limits: every lane its road's; every connector the lane it joins'; the one-way street none.
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        const Path& path = G.paths[p];
        const PathId lane = path.kind == PathKind::Lane ? PathId(p) : G.nextPaths[path.next.first];
        const std::string by = roadOf(lg, road, lane);
        const double want = by[0] == 'a' ? kArterialLimit : by[0] == 'l' ? kLocalLimit : 0.0;
        CHECK(lg.model.postedLimit(PathId(p)) == want);
    }
    CHECK(validate(G).empty());
}

// ---- 3. a missing control, an impossible sign ----------------------------------------------------------------------------

TEST_CASE(control_plan_3_a_junction_with_no_plan_or_an_impossible_sign_is_reported) {
    // The T at (200, 0) left out of the file's list: the lane level makes no junction there (it never makes one up): the
    // streets that ended at it end at no junction now, so their lanes there lead nowhere -- and that is REPORTED: the
    // streets' ends meet other roads where the road file states no junction (the planner's to state).
    {
        nlohmann::json g = plannedGraph();
        nlohmann::json kept = nlohmann::json::array();
        for (const nlohmann::json& j : g["junctions"])
            if (j["id"] != "j1") kept.push_back(j);
        g["junctions"] = kept;
        for (nlohmann::json& st : g["streets"])   // (the streets that ended at it end at no junction now)
            for (nlohmann::json& end : st["junctions"])
                if (end == "j1") end = nullptr;
        const BuiltLaneLevel lg = laneGraphFrom(g);
        CHECK(lg.error.empty());
        CHECK(lg.model.laneLevel.junctions.size() == 1u);
        CHECK(lg.model.controlAt(JunctionId(0)) == "stop");
        // the four street ends the left-out junction held: a1's and l2's ends there, a2's and o0's starts
        std::set<std::string> said;
        for (const std::string& p : engine::problemLines(lg.problems)) {
            if (p.find("but the road file states no junction there") == std::string::npos) continue;
            std::printf("    problem: %s\n", p.c_str());
            said.insert(p.substr(0, p.find(" at (")));
        }
        CHECK(said == std::set<std::string>({"street a1: its `to` end", "street a2: its `from` end", "street l2: its `to` end",
                                             "street o0: its `from` end"}));
    }
    // A stop sign written by hand on the one-way street's arm, which no lane arrives by: impossible, reported.
    {
        nlohmann::json g = plannedGraph();
        for (nlohmann::json& a : g["junctions"][1]["arms"])
            if (a["road"] == "o0") a["regulations"] = {"stop"};
        const BuiltLaneLevel lg = laneGraphFrom(g);
        for (const std::string& p : engine::problemLines(lg.problems)) std::printf("    problem: %s\n", p.c_str());
        CHECK(hasProblem(lg, "junction j1: its arm on road o0 carries \"stop\", but no lane arrives by it"));
        // THE PROBLEM IS A RECORD (ADR-0438): its kind, the junction, arm and road it concerns and the sign's name are
        // fields a tool can read, and its sentence is made from them by the one function
        uint32_t found = 0;
        for (const engine::RoadProblem& p : lg.problems) {
            if (p.kind != engine::RoadProblemKind::SignOnArmNoLane) continue;
            ++found;
            CHECK(p.junction == "j1" && p.road == "o0" && p.words == std::vector<std::string>{"stop"} && p.numbers.empty());
            CHECK(p.arm >= 0 && g["junctions"][1]["arms"][static_cast<std::size_t>(p.arm)]["road"] == "o0");
            CHECK(engine::problemLine(p) == "junction j1: its arm on road o0 carries \"stop\", but no lane arrives by it");
            CHECK(std::string(engine::problemKindRow(p.kind).name) == "sign on an arm no lane arrives by");
        }
        CHECK(found == 1u);
        const std::vector<std::pair<engine::RoadProblemKind, uint32_t>> byKind = engine::problemsByKind(lg.problems);
        uint32_t counted = 0;
        for (const std::pair<engine::RoadProblemKind, uint32_t>& k : byKind) counted += k.second;
        CHECK(counted == lg.problems.size());
    }
    // EVERY KIND HAS A NAME, A MEANING AND A SENTENCE, and a record with its fields filled writes that sentence with no
    // placeholder left in it (the kinds are listed in road_problem.cpp in the enum's own order)
    for (std::size_t k = 0; k < engine::kRoadProblemKindCount; ++k) {
        const engine::RoadProblemKindRow& row = engine::problemKindRow(static_cast<engine::RoadProblemKind>(k));
        CHECK(static_cast<std::size_t>(row.kind) == k && row.name[0] != '\0' && row.meaning[0] != '\0' && row.line[0] != '\0');
        engine::RoadProblem p(row.kind);
        p.atJunction("J").onStreet("S").andStreet("S2").onLane("L").andLane("L2").onRoad("R").onArm(1).onPath(2).andPath(3)
            .where(engine::Vec2(1.25, 2.5)).atStation(7.5);
        for (int i = 0; i < 10; ++i) p.number(i + 0.5).word("w" + std::to_string(i));
        const std::string line = engine::problemLine(p);
        CHECK(!line.empty() && line.find('{') == std::string::npos && line.find('}') == std::string::npos);
    }
    // two sentences written out by hand from their records: numbers keep the decimals their kind states
    {
        engine::RoadProblem fold(engine::RoadProblemKind::LaneFoldTidied);
        fold.onLane("s32.f1").onRoad("s32").where(engine::Vec2(-146.84, 102.46)).number(0.974).number(1.981);
        CHECK(engine::problemLine(fold) == "lane s32.f1 runs backwards for 0.97 m near (-146.8, 102.5): it lies on the inside of a corner of road "
                                           "s32 sharper than its offset from the road's spine; tidied (the fold cut out, 1.98 m shorter)");
        engine::RoadProblem slot(engine::RoadProblemKind::CarryOnOtherSlot);
        slot.onLane("a.f1").onStreet("a/0").andStreet("b/0").andLane("b.f0").number(1).number(-1);
        CHECK(engine::problemLine(slot) == "lane a.f1 (slot 1) carries on from street a/0 into b/0, which has no lane in its slot: into lane b.f0 "
                                           "(slot -1)");
    }
    // (Two stated junctions are two junctions of the lane level, whatever the paint draws: since ADR-0426 nothing groups
    // them, so the case of one built junction holding two planned ones of different controls cannot arise.)
    // A junction naming no control is no plan at all: the road graph is refused as it is read.
    {
        nlohmann::json g = plannedGraph();
        g["junctions"][0].erase("control");
        bool refused = false;
        try {
            RoadLabGraph::fromJson(g);
        } catch (const std::exception& e) {
            refused = std::string(e.what()).find("junction j0: names no control") != std::string::npos;
        }
        CHECK(refused);
    }
    // A hand-written junction survives re-planning, as written.
    {
        nlohmann::json g = plannedGraph();
        g["junctions"][1]["control"] = "signal";
        g["junctions"][1]["hand"] = true;
        const ControlPlan again = planControls(RoadLabGraph::fromJson(g), testPolicy());
        CHECK(again.handKept == 1u);
        nlohmann::json g2 = g;
        writeControlPlan(g2, again);
        bool kept = false;
        for (const nlohmann::json& j : g2["junctions"]) kept = kept || (j["id"] == "j1" && j["control"] == "signal" && j.value("hand", false));
        CHECK(kept);
        CHECK(g2["junctions"].size() == 2u);
    }
}

// ---- 4. save and load ----------------------------------------------------------------------------------------------------

TEST_CASE(control_plan_4_regulations_survive_save_and_load) {
    // The regulations are the road model's (ADR-0437): the signs its arms state and the rules stated on its streets.
    // A model saved and read back answers the same along every path, with one stated on a street too.
    RoadModel m = plannedBuild().model;
    ModelRegulation limit;
    limit.kind = "top_speed";
    limit.street = ModelStreetId(0);
    limit.from = m.streets[0].from;
    limit.to = m.streets[0].to;
    limit.value = 9;
    CHECK(stateRegulation(m, limit));
    bundle::BinWriter w;
    putRoadModel(w, m);
    bundle::BinReader r(w.bytes.data(), w.bytes.size());
    RoadModel back;
    std::string err;
    CHECK(getRoadModel(r, back, &err));
    CHECK(err.empty());
    CHECK(back == m);
    CHECK(back.laneLevel == m.laneLevel);
    CHECK(back.regulations.size() == 1u);
    std::size_t signs = 0, limits = 0;
    for (uint32_t p = 0; p < m.laneLevel.paths.size(); ++p) {
        const std::vector<PathRegulation> was = m.regulationsAlong(PathId(p)), is = back.regulationsAlong(PathId(p));
        CHECK(was.size() == is.size());
        for (std::size_t k = 0; k < was.size() && k < is.size(); ++k) {
            CHECK(was[k].kind == is[k].kind && was[k].from == is[k].from && was[k].to == is[k].to && was[k].value == is[k].value);
            ++(was[k].kind == "top_speed" ? limits : signs);
        }
    }
    CHECK(signs == 3u);    // the three stop signs of test 2
    CHECK(limits >= 1u);   // the limit, on each lane path of its street running along the spine
    // the model's dump shows what its arms state and the regulation on the street
    CHECK(dumpRoadModel(back).find("regulation: top_speed on street ") != std::string::npos);
    CHECK(dumpRoadModel(back).find(" [stop]") != std::string::npos);
}

// ---- 5. same plan, same graph ----------------------------------------------------------------------------------------------

TEST_CASE(control_plan_5_the_same_plan_and_graph_every_run) {
    ControlPlan a, b;
    const nlohmann::json ga = plannedGraph(&a), gb = plannedGraph(&b);
    CHECK(ga.dump(1) == gb.dump(1));
    // planning the planned graph again writes the very same plan
    nlohmann::json again = ga;
    writeControlPlan(again, planControls(RoadLabGraph::fromJson(ga), testPolicy()));
    CHECK(again.dump(1) == ga.dump(1));
    // ... through the one writer of a junction (road_layout.h junctionJson, issue #292): each junction in the scene is that
    // writer's writing of the junction as the road file's reader reads it back, every member of every arm
    const RoadLabGraph read = RoadLabGraph::fromJson(again);
    CHECK(read.junctions.size() == again["junctions"].size() && !read.junctions.empty());
    for (std::size_t i = 0; i < read.junctions.size() && i < again["junctions"].size(); ++i) {
        CHECK(again["junctions"][i] == junctionJson(read.junctions[i], read.streets));
        for (const nlohmann::json& arm : again["junctions"][i]["arms"]) CHECK(arm.contains("street") && arm.contains("heading"));
    }
    const BuiltLaneLevel second = laneGraphFrom(gb);
    const LaneGraph& first = plannedBuild().model.laneLevel;
    bundle::BinWriter wa, wb;
    putLaneGraph(wa, first);
    putLaneGraph(wb, second.model.laneLevel);
    CHECK(wa.bytes == wb.bytes);
    CHECK(dumpText(first) == dumpText(second.model.laneLevel));
}
