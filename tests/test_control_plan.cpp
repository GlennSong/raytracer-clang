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
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/lanes.h"

#include <memory>
#include <nlohmann/json.hpp>
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
ArmFacts arm(const std::string& roadClass, const Vec2& heading, int arriving, int leaving, const ControlPolicy& p) {
    ArmFacts a;
    a.road = roadClass + "_road";
    a.roadClass = roadClass;
    a.heading = heading;
    a.arriving = arriving;
    a.leaving = leaving;
    a.limit = p.postedLimit(roadClass);
    return a;
}
const Vec2 kEast(1, 0), kWest(-1, 0), kNorth(0, 1), kSouth(0, -1);

JunctionFacts junction(const std::vector<ArmFacts>& arms, const ControlPolicy& p, const Vec2& at = Vec2()) {
    JunctionFacts j;
    j.id = "test";
    j.at = at;
    j.arms = arms;
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

// The small graph with its plan written in.
nlohmann::json plannedGraph(ControlPlan* planOut = nullptr) {
    nlohmann::json g = smallGraph();
    const ControlPlan plan = planControls(RoadLabGraph::fromJson(g), testPolicy());
    writeControlPlan(g, plan);
    if (planOut) *planOut = plan;
    return g;
}

// The lane graph built from a road graph's JSON.
LaneGraphBuild laneGraphFrom(const nlohmann::json& g) {
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    return buildLaneGraph(*r, testVehicles());
}

const LaneGraphBuild& plannedBuild() {
    static const LaneGraphBuild lg = laneGraphFrom(plannedGraph());
    return lg;
}

bool hasProblem(const LaneGraphBuild& lg, const std::string& words) {
    for (const std::string& p : lg.problems)
        if (p.find(words) != std::string::npos) return true;
    return false;
}

// The road (its id) a lane path was cut from: its section's road, through the build's records.
std::string roadOf(const LaneGraphBuild& lg, const RoadLabGraph& g, PathId lane) {
    const uint32_t s = lg.graph.paths[lane.index].section.index;
    return s < lg.sectionSources.size() ? g.edges[static_cast<std::size_t>(lg.sectionSources[s].edge)].id : std::string();
}

// The regulation kinds standing at the end of `lane`.
std::vector<std::string> kindsAtEnd(const LaneGraph& g, PathId lane) {
    std::vector<std::string> out;
    const Path& p = g.paths[lane.index];
    for (uint32_t r = p.regulations.first; r < p.regulations.end(); ++r)
        if (g.regulations[r].from == p.length && g.regulations[r].to == p.length)
            out.push_back(g.regulationKindNames[g.regulations[r].kind]);
    return out;
}

}  // namespace

TEST_CASE(control_plan_2_a_planned_road_graph_is_carried_through) {
    ControlPlan plan;
    const nlohmann::json g = plannedGraph(&plan);
    std::printf("    %s", describeControlPlan(plan).c_str());
    CHECK(plan.problems.empty());
    // The plan as written: three junctions; the crossing and the T stop the local street; the bend has nothing.
    CHECK(g["junctions"].size() == 3u);
    CHECK(g["junctions"][0]["control"] == "stop" && g["junctions"][1]["control"] == "stop" && g["junctions"][2]["control"] == "unmarked");
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

    const LaneGraphBuild& lg = plannedBuild();
    const LaneGraph& G = lg.graph;
    const RoadLabGraph road = RoadLabGraph::fromJson(g);
    std::printf("    control plan: %u planned, %u not built, %u spread; %u not planned, %u conflicts, %u lanes off the plan, %u "
                "signs with no lane\n", lg.controlPlan.planned, lg.controlPlan.plannedNotBuilt, lg.controlPlan.plannedSpread,
                lg.controlPlan.notPlanned, lg.controlPlan.conflicts, lg.controlPlan.lanesOffPlan, lg.controlPlan.signsWithNoLane);
    for (const std::string& p : lg.problems) std::printf("    problem: %s\n", p.c_str());
    CHECK(lg.controlPlan.declared);
    CHECK(lg.controlPlan.notPlanned == 0 && lg.controlPlan.conflicts == 0 && lg.controlPlan.lanesOffPlan == 0 &&
          lg.controlPlan.signsWithNoLane == 0);
    CHECK(G.junctions.size() >= 2u);
    // Each built junction carries its planned junction's control; the stop signs stand where the local streets arrive.
    uint32_t stopsSeen = 0;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const Junction& jn = G.junctions[j];
        CHECK(lg.plannedOf[j].size() == 1u);
        if (lg.plannedOf[j].empty()) continue;
        const PlannedJunction& pj = road.junctions[lg.plannedOf[j][0]];
        CHECK(G.controlNames[jn.control] == pj.control);
        for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k) {
            const PathId lane = G.junctionLanes[k];
            const std::string by = roadOf(lg, road, lane);
            const std::vector<std::string> kinds = kindsAtEnd(G, lane);
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
        CHECK(path.speedLimit == want);
    }
    CHECK(validate(G).empty());
}

// ---- 3. a missing control, an impossible sign ----------------------------------------------------------------------------

TEST_CASE(control_plan_3_a_junction_with_no_plan_or_an_impossible_sign_is_reported) {
    // The T at (200, 0) left out of the plan: the builder names it and leaves it not planned (it never makes one up).
    {
        nlohmann::json g = plannedGraph();
        nlohmann::json kept = nlohmann::json::array();
        for (const nlohmann::json& j : g["junctions"])
            if (j["id"] != "j1") kept.push_back(j);
        g["junctions"] = kept;
        const LaneGraphBuild lg = laneGraphFrom(g);
        CHECK(lg.controlPlan.notPlanned == 1u);
        CHECK(hasProblem(lg, "no planned junction names it; it is left \"not_planned\""));
        uint32_t unplanned = 0;
        for (const Junction& jn : lg.graph.junctions) unplanned += lg.graph.controlNames[jn.control] == kControlNotPlanned ? 1 : 0;
        CHECK(unplanned == 1u);
    }
    // A stop sign written by hand on the one-way street's arm, which no lane arrives by: impossible, reported.
    {
        nlohmann::json g = plannedGraph();
        for (nlohmann::json& a : g["junctions"][1]["arms"])
            if (a["road"] == "o0") a["regulations"] = {"stop"};
        const LaneGraphBuild lg = laneGraphFrom(g);
        CHECK(lg.controlPlan.signsWithNoLane == 1u);
        CHECK(hasProblem(lg, "planned junction j1: its arm on road o0 (spine out at 0.0 m) carries \"stop\", but no lane arrives by it there"));
    }
    // Two planned junctions given to one built junction with different controls: neither is picked.
    {
        nlohmann::json g = plannedGraph();
        nlohmann::json split = g["junctions"][0];
        split["id"] = "j9";
        split["control"] = "signal";
        nlohmann::json keepArms = nlohmann::json::array(), moveArms = nlohmann::json::array();
        for (const nlohmann::json& a : g["junctions"][0]["arms"]) (a["road"] == "l0" ? moveArms : keepArms).push_back(a);
        g["junctions"][0]["arms"] = keepArms;
        split["arms"] = moveArms;
        g["junctions"].push_back(split);
        const LaneGraphBuild lg = laneGraphFrom(g);
        CHECK(lg.controlPlan.conflicts == 1u);
        CHECK(hasProblem(lg, "the builder made one junction of planned junctions j0 (stop), j9 (signal), whose controls differ"));
    }
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
        CHECK(g2["junctions"].size() == 3u);
    }
}

// ---- 4. save and load ----------------------------------------------------------------------------------------------------

TEST_CASE(control_plan_4_regulations_survive_save_and_load) {
    const LaneGraph& G = plannedBuild().graph;
    CHECK(!G.regulations.empty());
    bundle::BinWriter w;
    putLaneGraph(w, G);
    bundle::BinReader r(w.bytes.data(), w.bytes.size());
    LaneGraph back;
    std::string err;
    CHECK(getLaneGraph(r, back, &err));
    CHECK(err.empty());
    CHECK(back == G);
    CHECK(back.regulations == G.regulations && back.regulationKindNames == G.regulationKindNames);
    CHECK(back.controlNames == G.controlNames);
    CHECK(dumpText(back) == dumpText(G));
    // the dump shows each junction's control and what its arriving lanes face
    CHECK(dumpText(G).find("at the arriving lanes' ends: lane ") != std::string::npos);
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
    const LaneGraphBuild second = laneGraphFrom(gb);
    const LaneGraph& first = plannedBuild().graph;
    bundle::BinWriter wa, wb;
    putLaneGraph(wa, first);
    putLaneGraph(wb, second.graph);
    CHECK(wa.bytes == wb.bytes);
    CHECK(dumpText(first) == dumpText(second.graph));
}
