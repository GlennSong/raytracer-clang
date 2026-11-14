// THE ONE ROAD MODEL, steps 1 and 2 (docs/road-model-design.md, ADR-0417):
//
//   1. the planner and the one-off layout tool agree: for a small planned city, the junctions and streets the planner
//      writes (planToLanesScene) are the ones lanes_tool layout derives from the same scene without them -- the same
//      junctions with the same arms, the same streets with the same ends;
//   2. the bake does not see the layout: a road file with its layout written in (as the planner writes it, and in place as
//      the tool writes it, into a control plan's list too) strips back to the bytes it was, and the city's bake key is
//      the same before and after;
//   3. the builder carries the ids: every lane on a stated street knows its street, its slot and the junction at each
//      end; the paint's junction boxes are kept in the build's Result; the model holds what is stated (each arm's stop
//      line, the ramps' merges) and lists what is not;
//   4. the questions: the street, lane and station nearest a point; the streets at a junction in order; a street's lanes;
//      what a lane leads to; what is attached to a street or a junction;
//   5. saved and read back, and its own cache file, keyed;
//   6. the comparison with the old forms runs on a small scene and agrees where they must;
//   7. kerb parking is the road file's statement (ADR-0418): the twin's band, the model's section, and a statement that
//      cannot be built named.

#include "test_framework.h"
#include "planned_city.h"

#include "engine/bundle/bake.h"
#include "engine/procgen/city/plan/plan_scene.h"
#include "engine/procgen/city/roads/lanes/control_plan.h"     // findJunctions: the one grouping
#include "engine/procgen/city/roads/lanes/deck_mesh.h"        // kStopLineBack
#include "engine/procgen/city/roads/lanes/lane_graph_cache.h"   // levelRoadsKey: the city's bake key
#include "engine/procgen/city/roads/lanes/lane_expand.h"   // expand: the planned layer reads the ramps composed
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/road_layout.h"
#include "engine/procgen/city/roads/lanes/road_model_compare.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"   // roadTwin: the kerb parking band it hands the sim
#include "engine/procgen/city/roads/road_entity.h"      // navRoadGraph
#include "engine/procgen/city/roads/road_model.h"
#include "apps/citysim/control_policy_file.h"   // loadControlPolicy: the shared placement policy (ADR-0422)

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

using namespace engine;
using namespace engine::roads::lanes;

namespace {

// A SMALL ROAD GRAPH (as the control plan's test): an arterial west to east through two junctions; at (0, 0) a local
// street crosses it; at (200, 0) a local street ends on it from the south and a one-way street leaves it to the north;
// the crossing local street bends at (0, 150) into another. Flat ground.
nlohmann::json smallGraph() {
    nlohmann::json g;
    g["name"] = "road_model_test";
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

// The small graph with its layout, as the tool writes it.
nlohmann::json smallGraphWithLayout() {
    nlohmann::json g = smallGraph();
    const RoadLabGraph graph = RoadLabGraph::fromJson(g);
    RoadLayout layout = deriveLayout(graph, LayoutSource::Inferred);
    nameLayout(layout, graph);   // the streets' names, as the one-off tool writes them (ADR-0420)
    writeLayout(g, layout);
    return g;
}

// A junction compared by what it is: its arms as (road, station, way), sorted.
using ArmKey = std::tuple<std::string, double, bool>;
std::vector<ArmKey> armsOf(const PlannedJunction& j) {
    std::vector<ArmKey> out;
    for (const PlannedArm& a : j.arms) out.emplace_back(a.road, a.at, a.spineIn);
    std::sort(out.begin(), out.end());
    return out;
}

std::string slurp(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

std::string scratchDir(const std::string& name) {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / ("rt_road_model_" + name + "_" + std::to_string(static_cast<long>(::getpid())));
    std::filesystem::remove_all(p);
    std::filesystem::create_directories(p);
    return p.string();
}

// The build and the model of the small graph with its layout, once.
struct Built {
    std::unique_ptr<Result> r;
    RoadModel model;
};
const Built& smallBuilt() {
    static const Built b = [] {
        Built x;
        x.r = build(RoadLabGraph::fromJson(smallGraphWithLayout()));
        x.model = assembleRoadModel(*x.r);
        return x;
    }();
    return b;
}

}  // namespace

// ---- 0. a fresh road file states its controls (ADR-0422) -------------------------------------------------------------

#ifdef RT_ENABLE_SCRIPTING
TEST_CASE(road_model_0_a_planned_city_states_every_junctions_control) {
    // THE SHARED POLICY, through the default rule set, as the planner reads it
    ControlPolicy policy;
    std::string err;
    CHECK(citysim::loadControlPolicy(citysim::kDefaultRuleSet, policy, &err));
    if (policy.rows.empty()) {
        std::printf("    %s\n", err.c_str());
        return;
    }
    engine::plan::SceneOptions so;
    so.controls = &policy;
    const nlohmann::json scene = engine::plan::planToLanesScene(engine::plan::generatePlan(citytest::roundBrief(11)), so);
    const RoadLabGraph g = RoadLabGraph::fromJson(scene);
    CHECK(g.declaresJunctions && !g.junctions.empty());
    // every junction has a control, and the row that decided it (its `why`) is a row of the policy giving that control
    std::map<std::string, std::string> controlOfRow;
    for (const PolicyRow& r : policy.rows) controlOfRow[r.name] = r.control;
    std::map<std::string, uint32_t> byControl;
    uint32_t decided = 0;
    for (const PlannedJunction& pj : g.junctions) {
        CHECK(!pj.control.empty());
        const std::map<std::string, std::string>::const_iterator row = controlOfRow.find(pj.why);
        const bool byRow = row != controlOfRow.end() && row->second == pj.control;
        CHECK(byRow);
        decided += byRow ? 1u : 0u;
        ++byControl[pj.control];
        // a junction whose row stands signs carries them on some arm
        bool signs = false;
        for (const PlannedArm& a : pj.arms) signs = signs || !a.regulations.empty();
        for (const PolicyRow& r : policy.rows)
            if (r.name == pj.why && (!r.minorSigns.empty() || !r.majorSigns.empty())) CHECK(signs);
    }
    std::string counts;
    for (const std::pair<const std::string, uint32_t>& kv : byControl) counts += " " + std::to_string(kv.second) + " " + kv.first;
    std::printf("    a planned city: %zu junctions, %u decided by a row of the policy:%s\n", g.junctions.size(), decided, counts.c_str());
    CHECK(decided == g.junctions.size());
    // the layout the planner wrote stays with the controls: the model reads both
    const RoadModel m = plannedRoadModel(g);
    CHECK(m.junctions.size() == g.junctions.size());
    for (const ModelJunction& J : m.junctions) CHECK(!J.control.empty());
    for (const ModelJunction& J : m.junctions)
        for (const ModelArm& a : J.arms) CHECK(a.street.valid());
    // without a policy, the scene states none (a place's scene, which the island plans again as a whole)
    CHECK(!RoadLabGraph::fromJson(engine::plan::planToLanesScene(engine::plan::generatePlan(citytest::roundBrief(11)))).declaresJunctions);
}
#endif

// ---- 1. the planner and the tool agree --------------------------------------------------------------------------------

TEST_CASE(road_model_1_the_planner_and_the_layout_tool_agree) {
    const nlohmann::json scene = engine::plan::planToLanesScene(engine::plan::generatePlan(citytest::roundBrief(11)));
    CHECK(scene.contains("junctions") && scene.contains("streets"));
    const RoadLabGraph planned = RoadLabGraph::fromJson(scene);
    CHECK(planned.listsJunctions && planned.listsStreets && !planned.declaresJunctions);
    nlohmann::json bare = scene;
    bare.erase("junctions");
    bare.erase("streets");
    const RoadLayout tool = deriveLayout(RoadLabGraph::fromJson(bare), LayoutSource::Inferred);
    std::printf("    planner: %zu junctions, %zu streets; tool: %s", planned.junctions.size(), planned.streets.size(), describeLayout(tool).c_str());
    CHECK(!planned.junctions.empty());
    CHECK(planned.junctions.size() == tool.junctions.size());
    CHECK(planned.streets.size() == tool.streets.size());
    // junction for junction by their arms, and the ids that map one to the other
    std::map<std::vector<ArmKey>, std::string> toolByArms;
    for (const PlannedJunction& j : tool.junctions) toolByArms[armsOf(j)] = j.id;
    std::map<std::string, std::string> idOf;   // planner id -> tool id
    int unmatched = 0;
    for (const PlannedJunction& j : planned.junctions) {
        const std::map<std::vector<ArmKey>, std::string>::const_iterator it = toolByArms.find(armsOf(j));
        if (it == toolByArms.end()) {
            if (++unmatched <= 5) {
                std::printf("    planner junction %s at (%.1f, %.1f) with %zu arms: the tool has no junction with those arms\n", j.id.c_str(), j.at.x, j.at.y, j.arms.size());
                auto show = [](const PlannedJunction& x) {
                    std::string s;
                    for (const PlannedArm& a : x.arms) s += " " + a.road + "@" + std::to_string(a.at) + (a.spineIn ? "in" : "out");
                    return s;
                };
                std::printf("      planner:%s\n", show(j).c_str());
                for (const PlannedJunction& t : tool.junctions)
                    if (distance(t.at, j.at) < 40) std::printf("      tool %s at (%.1f, %.1f):%s\n", t.id.c_str(), t.at.x, t.at.y, show(t).c_str());
            }
            continue;
        }
        idOf[j.id] = it->second;
    }
    CHECK(unmatched == 0);
    // street for street: road, stations, and the junction at each end (as mapped)
    std::set<std::tuple<std::string, double, double, std::string, std::string>> toolStreets, plannerStreets;
    for (const PlannedStreet& s : tool.streets) toolStreets.emplace(s.road, s.from, s.to, s.junctions[0], s.junctions[1]);
    for (const PlannedStreet& s : planned.streets)
        plannerStreets.emplace(s.road, s.from, s.to, s.junctions[0].empty() ? "" : idOf[s.junctions[0]], s.junctions[1].empty() ? "" : idOf[s.junctions[1]]);
    CHECK(toolStreets == plannerStreets);
    // the planner's arms each name a street of its list, and the ids the planner gives are written in its own scene
    int armsWithoutStreet = 0;
    for (const PlannedJunction& j : planned.junctions)
        for (const PlannedArm& a : j.arms) armsWithoutStreet += a.street.empty() ? 1 : 0;
    CHECK(armsWithoutStreet == 0);
    // the planner names its streets from birth (ADR-0420): every street of a class that carries a name has one, and no
    // other does
    const std::vector<char> named = namedStreets(planned, planned.streets, streetNameBook());
    int unnamed = 0, misnamed = 0;
    for (std::size_t s = 0; s < planned.streets.size(); ++s) {
        unnamed += named[s] && planned.streets[s].name.empty() ? 1 : 0;
        misnamed += !named[s] && !planned.streets[s].name.empty() ? 1 : 0;
    }
    CHECK(unnamed == 0 && misnamed == 0);
}

// ---- 2. the bake does not see the layout --------------------------------------------------------------------------------

TEST_CASE(road_model_2_the_bake_key_does_not_see_the_layout) {
    // as the planner writes a file (one dump): the dump with its layout strips to the dump without
    const nlohmann::json bare = smallGraph();
    const nlohmann::json with = smallGraphWithLayout();
    CHECK(stripLayoutText(with.dump(1)) == bare.dump(1));
    CHECK(stripLayoutText(bare.dump(1)) == bare.dump(1));
    // as the tool writes one, in place: nothing else moves, and it strips back
    const RoadLayout layout = deriveLayout(RoadLabGraph::fromJson(bare), LayoutSource::Inferred);
    for (int indent : {1, 2, 4}) {
        std::string err;
        const std::string text = bare.dump(indent);
        const std::string written = insertLayoutText(text, layout, &err);
        CHECK(!written.empty());
        CHECK(stripLayoutText(written) == text);
        const RoadLabGraph back = RoadLabGraph::fromJson(nlohmann::json::parse(written));
        CHECK(back.streets.size() == layout.streets.size() && back.junctions.size() == layout.junctions.size());
    }
    // a control plan's list: the layout goes into it, and it strips back to the plan as it was
    {
        nlohmann::json planned = bare;
        nlohmann::json list = nlohmann::json::array();
        for (const PlannedJunction& j : layout.junctions) {
            nlohmann::json jj = {{"id", j.id}, {"at", {j.at.x, j.at.y}}, {"control", "stop"}};
            for (const PlannedArm& a : j.arms) jj["arms"].push_back({{"road", a.road}, {"at", a.at}, {"spine", a.spineIn ? "in" : "out"}});
            for (const PlannedStretch& st : j.inside) jj["inside"].push_back({{"road", st.road}, {"from", st.from}, {"to", st.to}});
            list.push_back(jj);
        }
        planned["junctions"] = list;
        const std::string text = planned.dump(1);
        const RoadLayout kept = deriveLayout(RoadLabGraph::fromJson(planned), LayoutSource::Inferred);
        CHECK(kept.keptList);
        std::string err;
        const std::string written = insertLayoutText(text, kept, &err);
        CHECK(!written.empty());
        CHECK(stripLayoutText(written) == text);
        const RoadLabGraph back = RoadLabGraph::fromJson(nlohmann::json::parse(written));
        CHECK(back.declaresJunctions);
        bool streets = true;
        for (const PlannedJunction& j : back.junctions)
            for (const PlannedArm& a : j.arms) streets = streets && !a.street.empty() && a.hasHeading;
        CHECK(streets);
    }
    // the city's bake key, through the producer: the same with the layout written in; another without an edge
    const std::string dir = scratchDir("key");
    const std::string graphPath = dir + "/graph.json";
    std::ofstream(graphPath) << bare.dump(1);
    bundle::LevelInputs in;
    in.levelPath = dir + "/level.json";
    in.levelDir = dir;
    in.level = {{"entities", nlohmann::json::array({{{"shape", "road"}, {"road", {{"graph", graphPath}}}}})}};
    const uint64_t before = levelRoadsKey(in);
    {
        std::string err;
        std::ofstream(graphPath) << insertLayoutText(bare.dump(1), layout, &err);
    }
    const uint64_t after = levelRoadsKey(in);
    nlohmann::json fewer = bare;
    fewer["edges"].erase(fewer["edges"].size() - 1);
    std::ofstream(graphPath) << fewer.dump(1);
    const uint64_t otherRoads = levelRoadsKey(in);
    std::printf("    the city's key: %016llx without the layout, %016llx with it, %016llx with a road fewer\n", static_cast<unsigned long long>(before),
                static_cast<unsigned long long>(after), static_cast<unsigned long long>(otherRoads));
    CHECK(before != 0 && before == after);
    CHECK(otherRoads != before);
    std::filesystem::remove_all(dir);
}

// ---- 3. the builder carries the ids; the model holds what is stated -------------------------------------------------

TEST_CASE(road_model_3_lanes_carry_their_streets_and_the_model_holds_what_is_stated) {
    const Built& b = smallBuilt();
    const Result& r = *b.r;
    const RoadModel& m = b.model;
    std::printf("    %s", reportRoadModel(m).c_str());
    // every lane on a road with streets knows them, and through lanes their slot from the centre line
    int lanesOnStreets = 0, wrongSlot = 0;
    for (const Lane& l : r.lanes.lanes) {
        if (l.parent < 0) continue;
        lanesOnStreets += l.streets.empty() ? 0 : 1;
        if (l.kind == "through") wrongSlot += l.slot == std::atoi(l.id.c_str() + l.id.rfind('.') + 2) ? 0 : 1;
    }
    CHECK(lanesOnStreets == static_cast<int>(std::count_if(r.lanes.lanes.begin(), r.lanes.lanes.end(), [](const Lane& l) { return l.parent >= 0; })));
    CHECK(wrongSlot == 0);
    // the arterial a1 runs from the crossing at (0, 0) to the T at (200, 0): its forward lanes start at the first and end at
    // the second, its backward lanes the other way round
    const ModelStreetId a1 = streetNamed(m, "a1");
    CHECK(a1.valid());
    const ModelJunctionId cross = m.streets[a1.index].junctions[0], tee = m.streets[a1.index].junctions[1];
    CHECK(cross.valid() && tee.valid());
    for (ModelLaneId l : lanesOf(m, a1)) {
        const ModelLane& L = m.lanes[l.index];
        CHECK(L.atStart == (L.dir > 0 ? cross : tee));
        CHECK(L.atEnd == (L.dir > 0 ? tee : cross));
    }
    // the paint's boxes are kept, and every arm of the crossing has its stop line from them, outside the box
    CHECK(r.hasJunctionBoxes && !r.junctionBoxes.empty());
    const ModelJunction& J = m.junctions[cross.index];
    CHECK(J.arms.size() == 4u);
    for (const ModelArm& a : J.arms) {
        CHECK(stated(a.stopLine) && stated(a.boxEdge));
        CHECK(a.spineIn ? a.stopLine < a.boxEdge : a.stopLine > a.boxEdge);
    }
    CHECK(J.through.size() == 2u);   // the arterial and the local street both carry on through
    // the bend at (0, 150) is NO junction (ADR-0419): the street l1 carries on into l3 there, and each says so
    int twoArm = 0;
    for (const ModelJunction& j : m.junctions) twoArm += j.arms.size() == 2 ? 1 : 0;
    CHECK(twoArm == 0);
    const ModelStreetId l1 = streetNamed(m, "l1"), l3 = streetNamed(m, "l3");
    CHECK(l1.valid() && l3.valid());
    if (l1.valid() && l3.valid()) {
        CHECK(m.streets[l1.index].carriesOn[1] == l3 && !m.streets[l1.index].junctions[1].valid());
        CHECK(m.streets[l3.index].carriesOn[0] == l1 && !m.streets[l3.index].junctions[0].valid());
    }
    // what is not stated is listed, and nothing is filled in: no control, no turn paths
    bool turns = false, controls = false;
    for (const std::string& s : m.notStated) {
        turns = turns || s.find("turn paths") != std::string::npos;
        controls = controls || s.find("controls") != std::string::npos;
    }
    CHECK(turns && controls);
    for (const ModelJunction& j : m.junctions) CHECK(j.control.empty());
    // the only problems: the paint still draws its box at the bend (both roads' ends), where the file states no junction
    CHECK(m.problems.size() == 2u);
    for (const std::string& p : m.problems) CHECK(p.find("carries on into") != std::string::npos);
}

TEST_CASE(road_model_3b_a_ramps_merge_is_its_stated_anchor) {
    nlohmann::json g = nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/lanelab/merge_taper.json"));
    g.erase("junctions");
    g.erase("streets");
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    const RoadModel m = assembleRoadModel(*r);
    // three ramps, each merging into the mainline's kerb lane at its anchor, with its stated auxiliary lane and taper
    int merges = 0;
    for (const ModelConnection& c : m.connections) {
        if (c.kind != ConnectionKind::Merge) continue;
        ++merges;
        CHECK(m.lanes[c.to.index].id == "mainline.f2");
        CHECK(c.length == 120.0 && c.taper == 108.0);
        // the anchor's x: 330, 780 or 1220 along a mainline starting at x = -150
        CHECK(std::fabs(c.hostStation - 480.0) < 0.01 || std::fabs(c.hostStation - 930.0) < 0.01 || std::fabs(c.hostStation - 1370.0) < 0.01);
    }
    CHECK(merges == 3);
    // each ramp's start is on its feeder street's side, an arm of a junction there
    for (const ModelRamp& R : m.ramps) CHECK(R.ends[0].kind == RampEndKind::OnStreet && R.ends[1].kind == RampEndKind::Anchored);
    // what the ramp lane leads to: its merge
    const LeadsTo to = leadsTo(m, laneNamed(m, "ramp_one.f0"));
    CHECK(to.connections.size() == 1u && m.connections[to.connections.front()].kind == ConnectionKind::Merge);
}

// ---- 4. the questions ------------------------------------------------------------------------------------------------------

TEST_CASE(road_model_4_the_questions) {
    const RoadModel& m = smallBuilt().model;
    // nearest to a point beside the arterial halfway between its junctions: street a1, a lane of it, station ~100
    const NearestOnRoad n = nearestOnRoad(m, Vec2(100, -3));
    CHECK(n.street.valid() && m.streets[n.street.index].id == "a1");
    CHECK(n.lane.valid() && m.lanes[n.lane.index].edge == m.streets[n.street.index].edge);
    CHECK(std::fabs(n.station - 100.0) < 1.0);
    CHECK(n.distance < 3.0);
    // nothing near a point far from every road
    CHECK(!nearestOnRoad(m, Vec2(1000, 1000)).lane.valid());
    // the crossing's streets in order round it: east (a1), north (l1), west (a0), south (l0)
    const ModelJunctionId cross = m.streets[streetNamed(m, "a1").index].junctions[0];
    const std::vector<ModelStreetId> round = streetsAt(m, cross);
    CHECK(round.size() == 4u);
    if (round.size() == 4u) {
        CHECK(m.streets[round[0].index].id == "a1");
        CHECK(m.streets[round[1].index].id == "l1");
        CHECK(m.streets[round[2].index].id == "a0");
        CHECK(m.streets[round[3].index].id == "l0");
    }
    // a street's lanes: forward then backward, by slot from the centre line
    const std::vector<ModelLaneId> lanes = lanesOf(m, streetNamed(m, "a1"));
    CHECK(lanes.size() == 6u);
    for (std::size_t i = 0; i + 1 < lanes.size(); ++i) {
        const ModelLane& A = m.lanes[lanes[i].index];
        const ModelLane& B = m.lanes[lanes[i + 1].index];
        CHECK(A.dir > B.dir || (A.dir == B.dir && A.slot < B.slot));
    }
    // what a lane leads to: the junction at its end, and nothing else stated
    const LeadsTo to = leadsTo(m, laneNamed(m, "a1.f0"));
    CHECK(to.junction.valid() && m.junctions[to.junction.index].at.x > 150);
    CHECK(to.connections.empty());
    // what is attached (ADR-0420): each named street carries its name; the crossing has nothing yet
    const std::vector<uint32_t> onO0 = attachedToStreet(m, streetNamed(m, "o0"));
    CHECK(onO0.size() == 1u);
    if (!onO0.empty()) CHECK(m.attachments[onO0.front()].text == streetName(m, streetNamed(m, "o0")) && !m.attachments[onO0.front()].text.empty());
    CHECK(attachedToJunction(m, cross).empty());
    // a name sign attached to an arm of the crossing: its street is the arm's; the street now lists it too
    RoadModel signed_ = m;
    ModelAttachment sign;
    sign.kind = attachmentKinds().use("name_sign");
    sign.junction = cross;
    sign.arm = 0;
    sign.text = "a blade";
    std::string why;
    CHECK(attach(signed_, sign, &why));
    CHECK(signed_.attachments.back().street == signed_.junctions[cross.index].arms[0].street);
    CHECK(attachedToJunction(signed_, cross).size() == 1u);
    CHECK(attachedToStreet(signed_, signed_.junctions[cross.index].arms[0].street).size() == 2u);   // its name, and the sign
    // refused: a kind the data does not list; an arm kind with no arm; a street kind with no street
    ModelAttachment bad = sign;
    bad.kind = "no_such_kind";
    CHECK(!attach(signed_, bad, &why));
    bad = sign;
    bad.arm = 99;
    CHECK(!attach(signed_, bad, &why));
    bad = ModelAttachment();
    bad.kind = attachmentKinds().use("street_name");
    CHECK(!attach(signed_, bad, &why));
    // the attachment kinds are data
    CHECK(attachmentKinds().find(attachmentKinds().use("street_name")) && attachmentKinds().find(attachmentKinds().use("street_name"))->names);
}

// ---- 4b. names: a run of streets that are one street carrying on has one name, as the road file states it -------------

TEST_CASE(road_model_4b_a_name_is_a_stated_run) {
    const RoadModel& m = smallBuilt().model;
    auto name = [&](const char* id) { return streetName(m, streetNamed(m, id)); };
    // the arterial runs straight through both junctions: one name; the local street crosses at (0, 0) and bends on into
    // l3 at (0, 150) (a carry-on): one name; at the T the local street from the south runs on as the one-way street north
    CHECK(!name("a0").empty() && name("a0") == name("a1") && name("a1") == name("a2"));
    CHECK(!name("l0").empty() && name("l0") == name("l1") && name("l1") == name("l3"));
    CHECK(!name("l2").empty() && name("l2") == name("o0"));
    CHECK(name("a0") != name("l0") && name("a0") != name("l2") && name("l0") != name("l2"));
    // the planned layer, read from the file alone, holds the same junctions, streets and names as the model of the build
    RoadLabGraph g = RoadLabGraph::fromJson(smallGraphWithLayout());
    (void)expand(g);
    const RoadModel planned = plannedRoadModel(g);
    CHECK(!planned.built && smallBuilt().model.built);
    CHECK(planned.junctions.size() == m.junctions.size() && planned.streets.size() == m.streets.size());
    CHECK(planned.lanes.empty() && planned.connections.empty());
    for (std::size_t s = 0; s < m.streets.size(); ++s) {
        CHECK(streetName(planned, ModelStreetId(static_cast<uint32_t>(s))) == streetName(m, ModelStreetId(static_cast<uint32_t>(s))));
        CHECK(planned.streets[s].line.count >= 2);
    }
    CHECK(!planned.notStated.empty() && planned.notStated.front().find("the built layer") != std::string::npos);
    // the name nearest a point beside the arterial
    CHECK(streetNameNear(planned, Vec2(100, -4)) == name("a1"));
    CHECK(streetNameNear(planned, Vec2(1000, 1000)).empty());
    // the named arms at the crossing: four, two names, in order round it
    const ModelJunctionId cross = planned.streets[streetNamed(planned, "a1").index].junctions[0];
    const std::vector<NamedArm> arms = namedArmsAt(planned, cross);
    CHECK(arms.size() == 4u);
    std::set<std::string> names;
    for (const NamedArm& a : arms) names.insert(a.name);
    CHECK(names.size() == 2u);
    // saved and read back with its attachments
    bundle::BinWriter w;
    putRoadModel(w, planned);
    bundle::BinReader rd(w.bytes.data(), w.bytes.size());
    RoadModel back;
    std::string err;
    CHECK(getRoadModel(rd, back, &err) && back == planned);
    // the one-off tool writes the names into a file in place; the bake key does not see them
    const nlohmann::json bare = smallGraph();
    nlohmann::json laidOut = bare;
    writeLayout(laidOut, deriveLayout(RoadLabGraph::fromJson(bare), LayoutSource::Inferred));
    const std::string text = laidOut.dump(1);
    const RoadLabGraph stated = RoadLabGraph::fromJson(laidOut);
    std::vector<PlannedStreet> streets = stated.streets;
    nameStreetRuns(streets, stated.junctions, stated.divided, stated, streetNameBook());
    const std::string named = insertStreetNamesText(text, streets, &err);
    CHECK(!named.empty() && stripLayoutText(named) == stripLayoutText(text));
    const RoadLabGraph reread = RoadLabGraph::fromJson(nlohmann::json::parse(named));
    for (std::size_t s = 0; s < streets.size(); ++s) CHECK(reread.streets[s].name == streets[s].name);
    // named again, every name the file states is kept
    std::vector<PlannedStreet> again = reread.streets;
    const NamingReport rep = nameStreetRuns(again, reread.junctions, reread.divided, reread, streetNameBook(), namedStretches(reread.streets));
    CHECK(rep.generated == 0u && rep.kept == rep.runs);
    for (std::size_t s = 0; s < again.size(); ++s) CHECK(again[s].name == streets[s].name);

    // EVERY JUNCTION'S CONTROL, stated once (ADR-0421): written in place; the bake key (keyedRoadFileText) does not see
    // it, the lane graph's key (junctionPlanKey) does, and the model reads it back
    std::vector<std::pair<std::string, std::string>> controls;
    for (std::size_t j = 0; j < reread.junctions.size(); ++j)
        controls.emplace_back(reread.junctions[j].id, j == 0 ? "signal" : "unmarked");
    const std::string controlled = insertJunctionControlsText(named, controls, &err);
    CHECK(!controlled.empty());
    CHECK(keyedRoadFileText(controlled) == keyedRoadFileText(named));
    CHECK(keyedRoadFileText(controlled) == keyedRoadFileText(bare.dump(1)));
    CHECK(junctionPlanKey(nlohmann::json::parse(controlled)) != junctionPlanKey(nlohmann::json::parse(named)));
    const RoadLabGraph withControls = RoadLabGraph::fromJson(nlohmann::json::parse(controlled));
    CHECK(withControls.declaresJunctions);
    const RoadModel cm = plannedRoadModel(withControls);
    CHECK(!cm.junctions.empty() && cm.junctions[0].control == "signal");
    for (std::size_t j = 1; j < cm.junctions.size(); ++j) CHECK(cm.junctions[j].control == "unmarked");
    // the signal poles stand for the stated "signal", by the data (road_attachments.json)
    const AttachmentKind* pole = attachmentKinds().find(attachmentKinds().use("signal_pole"));
    CHECK(pole && pole->on == "arm" && pole->standsFor("signal") && !pole->standsFor("unmarked"));
}

// ---- 5. saved and read back ----------------------------------------------------------------------------------------------

TEST_CASE(road_model_5_saved_and_read_back) {
    const RoadModel& m = smallBuilt().model;
    bundle::BinWriter w;
    putRoadModel(w, m);
    bundle::BinReader rd(w.bytes.data(), w.bytes.size());
    RoadModel back;
    std::string err;
    CHECK(getRoadModel(rd, back, &err));
    CHECK(back == m);
    RoadModel changed = m;
    changed.streets.front().speedLimit = 11.0;
    CHECK(!(changed == m));
    // cut short: refused, never half read
    bundle::BinReader shortReader(w.bytes.data(), w.bytes.size() / 2);
    RoadModel half;
    CHECK(!getRoadModel(shortReader, half, &err));
    // its own cache file: written, read back equal, refused for another key
    const std::string dir = scratchDir("cache");
    const uint64_t key = roadModelCacheKey(1234, 5678);
    CHECK(key != roadModelCacheKey(1234, 5679) && key != roadModelCacheKey(1235, 5678) && key != roadModelCacheKey(1234, 5678, kRoadModelVersion + 1));
    const std::string path = roadModelCachePath(dir, key);
    CHECK(writeRoadModelCache(path, key, m, &err));
    RoadModel read;
    CHECK(readRoadModelCache(path, key, read, &err) && read == m);
    CHECK(!readRoadModelCache(path, key + 1, read, &err));
    // a level's: built and saved the first time, read from the file the second
    const std::string graphPath = dir + "/graph.json";
    std::ofstream(graphPath) << smallGraphWithLayout().dump(1);
    bundle::LevelInputs in;
    in.levelPath = dir + "/level.json";
    in.levelDir = dir;
    in.level = {{"entities", nlohmann::json::array({{{"shape", "road"}, {"road", {{"graph", graphPath}}}}})}};
    const ObtainedRoadModel first = obtainLevelRoadModel(in, dir + "/roadmodel", &err);
    CHECK(first.model && first.saved && !first.fromCache);
    const ObtainedRoadModel second = obtainLevelRoadModel(in, dir + "/roadmodel", &err);
    CHECK(second.model && second.fromCache && second.key == first.key);
    if (first.model && second.model) CHECK(*first.model == *second.model);
    std::filesystem::remove_all(dir);
}

// ---- 6. the comparison with the old forms --------------------------------------------------------------------------------

TEST_CASE(road_model_6_compared_with_the_old_forms) {
    const Built& b = smallBuilt();
    const RoadModelComparison c = compareRoadModel(b.model, *b.r);
    std::printf("%s", c.text.c_str());
    // the crossing and the T are junctions of every form, one to one; the nav has no junction node the model has not (the
    // bend is no junction to the model either since ADR-0419: a street carrying on)
    CHECK(c.nav.matched >= 2u && c.paint.matched >= 2u && c.laneGraph.matched >= 2u);
    CHECK(c.modelTwoArm == 0u);
    CHECK(c.nav.onlyModel == 0u);
    CHECK(c.nav.onlyOther == 0u);
    // the paint (and the lane graph, built from its boxes) still makes the bend a junction, by its own rule until step 9
    CHECK(c.paint.onlyOther <= 1u && c.laneGraph.onlyOther <= 1u);
    CHECK(c.nav.otherKnots == 0u && c.nav.modelKnots == 0u);
    // every street cars drive has its nav link, with the model's lanes each way and its one-way
    CHECK(c.streetsCompared == static_cast<uint32_t>(b.model.streets.size()));
    CHECK(c.streetsWithoutLink == 0u && c.oneWayDiffers == 0u);
    CHECK(c.lanesWithPaths == c.modelRoadLanes);
    CHECK(c.text.find("WHAT THE MODEL DOES NOT STATE YET") != std::string::npos);
    // ONE LANE COUNT (ADR-0418): every drivable link gets its road's stated lanes, so no street's nav lanes differ from
    // the road file's; and the sim's kerbside bays lie only in a band a road states
    CHECK(c.linksStated == c.linksDrivable);
    CHECK(c.laneCountDiffers == 0u);
    CHECK(c.baysOffStatedParking == 0u);
}

// ---- 7. kerb parking is stated (ADR-0418) --------------------------------------------------------------------------------

namespace {
// The kerb parking bands the road twin hands the sim (navRoadGraph's edges: parkWidth), by the twin edge's class.
std::map<engine::RoadClass, std::pair<int, double>> twinBands(const Result& r) {
    std::map<engine::RoadClass, std::pair<int, double>> out;   // class -> (edges with a band, the widest band)
    for (const engine::RoadEdge& e : engine::navRoadGraph(roadTwin(r, 60.0, false, /*forNav=*/true), nullptr).edges) {
        if (e.parkWidth <= 0) continue;
        std::pair<int, double>& o = out[e.klass];
        ++o.first;
        o.second = std::max(o.second, static_cast<double>(e.parkWidth));
    }
    return out;
}
}  // namespace

TEST_CASE(road_model_7_kerb_parking_is_the_road_file_s_statement) {
    // the class table: a local street parks along both kerbs, 2.5 m over its 6 m lane; no other class parks
    CHECK(roadClass("local").lanes.parking == 2.5);
    for (const std::pair<const std::string, RoadClassSpec>& c : roadClasses())
        if (c.first != "local") CHECK(c.second.lanes.parking == 0.0);
    // the small graph (the table's local and arterial, and a one-way class of its own): the twin's bands are the locals'
    // stated 2.5 m and nothing else's; the model's street sections say the same
    {
        const Built& b = smallBuilt();
        const std::map<engine::RoadClass, std::pair<int, double>> bands = twinBands(*b.r);
        CHECK(bands.size() == 1u && bands.count(engine::RoadClass::Local) == 1u);
        if (bands.count(engine::RoadClass::Local)) CHECK_APPROX(bands.at(engine::RoadClass::Local).second, 2.5, 1e-9);
        for (const ModelStreet& s : b.model.streets) CHECK(s.section.parking == (s.section.roadClass == "local" ? 2.5 : 0.0));
        for (const std::string& p : b.model.problems) CHECK(p.find("kerb parking") == std::string::npos);
    }
    // the file says its locals do not park: no band at all
    {
        nlohmann::json g = smallGraphWithLayout();
        g["classes"]["local"] = {{"parking", 0.0}};
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        CHECK(twinBands(*r).empty());
        for (const ModelStreet& s : assembleRoadModel(*r).streets) CHECK(s.section.parking == 0.0);
    }
    // ...or that it parks 2.0 m: the band is 2.0 m
    {
        nlohmann::json g = smallGraphWithLayout();
        g["classes"]["local"] = {{"parking", 2.0}};
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        const std::map<engine::RoadClass, std::pair<int, double>> bands = twinBands(*r);
        CHECK(bands.count(engine::RoadClass::Local) == 1u);
        if (bands.count(engine::RoadClass::Local)) CHECK_APPROX(bands.at(engine::RoadClass::Local).second, 2.0, 1e-9);
    }
    // a local of 3.5 m lanes cannot hold the table's 2.5 m and a travel lane: no band, and the model says so by name
    {
        nlohmann::json g = smallGraphWithLayout();
        g["classes"]["local"] = {{"w", 3.5}};
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        CHECK(twinBands(*r).empty());
        const RoadModel m = assembleRoadModel(*r);
        int said = 0;
        for (const std::string& p : m.problems) said += p.find("kerb parking") != std::string::npos ? 1 : 0;
        CHECK(said == 4);   // l0..l3, once each
    }
}

// ---- 8. which road ends are ONE junction (ADR-0419) ---------------------------------------------------------------------

namespace {
// An arterial west to east, cut where two streets end on it from the north, `apart` metres apart (at x = 0 and x = apart):
// the classes' own sections (an arterial's half width 11 m, a boulevard's 5.5 m). `cls` the two streets' class; `pair`
// states them as a divided road's two carriageways.
nlohmann::json twoTees(double apart, const std::string& cls, bool pair) {
    auto road = [](const std::string& id, const std::string& c, const Vec2& a, const Vec2& b) {
        return nlohmann::json{{"id", id}, {"class", c}, {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}};
    };
    nlohmann::json g;
    g["name"] = "two_tees";
    g["edges"] = nlohmann::json::array({road("a0", "arterial", Vec2(-200, 0), Vec2(0, 0)), road("a1", "arterial", Vec2(0, 0), Vec2(apart, 0)),
                                        road("a2", "arterial", Vec2(apart, 0), Vec2(apart + 200, 0)),
                                        road("t_in", cls, Vec2(0, 0), Vec2(0, 150)), road("t_out", cls, Vec2(apart, 150), Vec2(apart, 0))});
    if (pair) g["divided"] = nlohmann::json::array({nlohmann::json::array({"t_in", "t_out"})});
    return g;
}
std::size_t junctionsOf(const nlohmann::json& g) { return findJunctions(RoadLabGraph::fromJson(g), ControlPolicy()).size(); }
}  // namespace

TEST_CASE(road_model_8_one_junction_is_no_room_for_a_car_between_stop_lines) {
    // between the two Ts' stop lines along the arterial: `apart` less (11 + 5.2) at each (the arterial is the widest other
    // road at both); one junction when that is under the storage length (6.5 m, a car and its gap)
    const double reach = 2.0 * (11.0 + kStopLineBack);
    CHECK(junctionsOf(twoTees(reach + 6.0, "local", false)) == 1u);   // 6 m between: no car fits
    CHECK(junctionsOf(twoTees(reach + 7.0, "local", false)) == 2u);   // 7 m: a car can wait
    // the storage length is the road file's to state
    nlohmann::json g = twoTees(reach + 7.0, "local", false);
    g["rules"]["storage"] = 8.0;
    CHECK(junctionsOf(g) == 1u);
    // a divided road's two carriageways meeting one street are one junction by construction, however far apart
    CHECK(junctionsOf(twoTees(60.0, "boulevard", false)) == 2u);
    CHECK(junctionsOf(twoTees(60.0, "boulevard", true)) == 1u);
    // the layout tool finds the pair from the planner's own naming when the file states none, and writes it in
    const RoadLayout L = deriveLayout(RoadLabGraph::fromJson(twoTees(60.0, "boulevard", false)), LayoutSource::Inferred);
    CHECK(L.divided.size() == 1u && L.junctions.size() == 1u);
    // where a street only carries on (two road ends meeting at a bend, nothing else there) is no junction, and the two
    // streets name each other
    nlohmann::json bend;
    bend["name"] = "bend";
    bend["edges"] = nlohmann::json::array({{{"id", "p"}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{0, 0}, {100, 0}}}}}},
                                           {{"id", "q"}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{100, 0}, {150, 80}}}}}}});
    CHECK(junctionsOf(bend) == 0u);
    const RoadLayout B = deriveLayout(RoadLabGraph::fromJson(bend), LayoutSource::Inferred);
    CHECK(B.junctions.empty() && B.streets.size() == 2u);
    if (B.streets.size() == 2u) CHECK(B.streets[0].carriesOn[1] == "q" && B.streets[1].carriesOn[0] == "p");
}
