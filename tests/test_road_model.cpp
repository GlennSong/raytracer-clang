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
#include "engine/procgen/city/roads/lane_turns.h"           // the stated turns (ADR-0426)
#include "engine/procgen/city/roads/lanes/lane_level.h"     // the lane level from the model (ADR-0426)
#include "lane_level_scene.h"                               // a scene with the lane level's facts, in memory
#include "engine/procgen/city/roads/control_signs.h"   // the signs that follow from the facts (ADR-0424)
#include "engine/ai/nav_graph.h"                        // rightOf: the kerb side
#include "engine/script_assets.h"                       // loadScriptCode: the rule set's file
#include "apps/citysim/control_policy_file.h"   // loadControlPolicy: the shared placement policy (ADR-0422)
#ifdef RT_ENABLE_SCRIPTING
#include "apps/driving/rule_set.h"
#include "apps/driving/scripting/rule_set_lua.h"   // loadRuleSet: what each sign means is the rule set's device
#include "engine/scripting/script_vm.h"
#endif

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

// ---- 2. the bake sees only where the stated arms stand --------------------------------------------------------------------------------

TEST_CASE(road_model_2_the_bake_key_sees_only_where_the_stated_arms_stand) {
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
    // the city's bake key, through the producer (ADR-0428: the paint draws the stated junctions, so where their arms stand
    // is keyed, and nothing else of the layout): a file stating its junctions' arms (the control plan's list) keys the
    // same with the layout written into it; moving one arm a metre, or a road fewer, keys otherwise; and a file stating
    // no junctions keys otherwise than one stating them
    nlohmann::json planned = bare;
    for (const PlannedJunction& j : layout.junctions) {
        nlohmann::json jj = {{"id", j.id}, {"at", {j.at.x, j.at.y}}, {"control", "stop"}};
        for (const PlannedArm& a : j.arms) jj["arms"].push_back({{"road", a.road}, {"at", a.at}, {"spine", a.spineIn ? "in" : "out"}});
        for (const PlannedStretch& st : j.inside) jj["inside"].push_back({{"road", st.road}, {"from", st.from}, {"to", st.to}});
        planned["junctions"].push_back(jj);
    }
    const std::string dir = scratchDir("key");
    const std::string graphPath = dir + "/graph.json";
    bundle::LevelInputs in;
    in.levelPath = dir + "/level.json";
    in.levelDir = dir;
    in.level = {{"entities", nlohmann::json::array({{{"shape", "road"}, {"road", {{"graph", graphPath}}}}})}};
    auto keyOf = [&](const std::string& text) {
        std::ofstream(graphPath) << text;
        return levelRoadsKey(in);
    };
    const uint64_t stated = keyOf(planned.dump(1));
    std::string err;
    const RoadLayout kept = deriveLayout(RoadLabGraph::fromJson(planned), LayoutSource::Inferred);
    const uint64_t laidOut = keyOf(insertLayoutText(planned.dump(1), kept, &err));
    nlohmann::json moved = planned;
    moved["junctions"][0]["arms"][0]["at"] = moved["junctions"][0]["arms"][0]["at"].get<double>() + 1.0;
    const uint64_t armMoved = keyOf(moved.dump(1));
    nlohmann::json fewer = bare;
    fewer["edges"].erase(fewer["edges"].size() - 1);
    const uint64_t otherRoads = keyOf(fewer.dump(1));
    const uint64_t none = keyOf(bare.dump(1));
    std::printf("    the city's key: %016llx stated, %016llx laid out, %016llx an arm moved, %016llx a road fewer, %016llx no junctions\n",
                static_cast<unsigned long long>(stated), static_cast<unsigned long long>(laidOut), static_cast<unsigned long long>(armMoved),
                static_cast<unsigned long long>(otherRoads), static_cast<unsigned long long>(none));
    CHECK(stated != 0 && stated == laidOut);
    CHECK(armMoved != stated);
    CHECK(otherRoads != stated);
    CHECK(none != stated);
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
    // every arm of the crossing has its stop line from the stated junction (junctionAreas, ADR-0428): its box edge back from
    // its mouth by the CROSSING street's half width (the default classes: the arterial's arms by the local street's one
    // 6 m lane each way, 6 m; the local street's by the arterial's three 22/6 m lanes each way, 11 m), its stop line
    // kStopLineBack further
    const ModelJunction& J = m.junctions[cross.index];
    CHECK(J.arms.size() == 4u);
    for (const ModelArm& a : J.arms) {
        CHECK(stated(a.stopLine) && stated(a.boxEdge));
        const bool arterial = a.road[0] == 'a';
        const Real back = arterial ? 6.0 : 11.0;
        std::printf("      arm %s: mouth %.2f, box edge %.2f, stop line %.2f\n", a.road.c_str(), a.station, a.boxEdge, a.stopLine);
        const Real away = a.spineIn ? -1.0 : 1.0;
        CHECK(approxEqual(a.boxEdge, a.station + away * back, kWrittenStation));
        CHECK(approxEqual(a.stopLine, a.station + away * (back + roads::lanes::kStopLineBack), kWrittenStation));
    }
    CHECK(J.boxes.size() == 4u);   // its area along each of its four roads (each arm a road of its own)
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
    // no problems: the paint draws only the stated junctions (ADR-0428), none at the bend where the street carries on
    for (const std::string& p : m.problems) std::printf("      problem: %s\n", p.c_str());
    CHECK(m.problems.empty());
}

TEST_CASE(road_model_3b_a_ramps_merge_is_its_stated_anchor) {
    nlohmann::json g = nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/lanelab/merge_taper.json"));
    g.erase("junctions");
    g.erase("streets");
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    const RoadModel m = assembleRoadModel(*r);
    // three ramps, each merging into the mainline's kerb lane at its anchor, with its stated auxiliary lane and taper (and,
    // since ADR-0426, a two-lane ramp's second lane merging into its first: no ramp's anchor, so not counted here)
    int merges = 0;
    for (const ModelConnection& c : m.connections) {
        if (c.kind != ConnectionKind::Merge || c.ramp.empty()) continue;
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
    // it, the lane graph's key (laneLevelKey) does, and the model reads it back
    std::vector<std::pair<std::string, std::string>> controls;
    for (std::size_t j = 0; j < reread.junctions.size(); ++j)
        controls.emplace_back(reread.junctions[j].id, j == 0 ? "signal" : "unmarked");
    const std::string controlled = insertJunctionControlsText(named, controls, &err);
    CHECK(!controlled.empty());
    CHECK(keyedRoadFileText(controlled) == keyedRoadFileText(named));
    CHECK(keyedRoadFileText(controlled) == keyedRoadFileText(text));   // (the junctions' arms as laid out: ADR-0428)
    CHECK(laneLevelKey(nlohmann::json::parse(controlled)) != laneLevelKey(nlohmann::json::parse(named)));
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
    // (the model's own cache file, which nothing read, went in ADR-0426's review)
}

// ---- 6. the comparison with the old forms --------------------------------------------------------------------------------

TEST_CASE(road_model_6_compared_with_the_old_forms) {
    // (the small graph with the facts the lane graph reads since ADR-0426: its junctions' controls and its lane rules)
    Built b;
    b.r = build(RoadLabGraph::fromJson(lane_level_test::withLaneLevelFacts(smallGraphWithLayout())));
    b.model = assembleRoadModel(*b.r);
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
// The kerb parking bands the road twin carries (navRoadGraph's edges: parkWidth; the lot pass's twin since ADR-0425), by
// the twin edge's class.
std::map<engine::RoadClass, std::pair<int, double>> twinBands(const Result& r) {
    std::map<engine::RoadClass, std::pair<int, double>> out;   // class -> (edges with a band, the widest band)
    for (const engine::RoadEdge& e : engine::navRoadGraph(roadTwin(r), nullptr).edges) {
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

// ---- 9. the signs that follow from the facts (ADR-0424) ----------------------------------------------------------------

namespace {
// The small graph with its one-way street turned to run INTO the T at (200, 0) (its exit end there), its layout written,
// and controls stated by hand: an all-way stop at the crossing (0, 0), the T giving way on its local street.
nlohmann::json signedGraph() {
    nlohmann::json g = smallGraph();
    for (nlohmann::json& e : g["edges"])
        if (e["id"] == "o0") e["path"]["points"] = {{200, 150}, {200, 0}};
    const RoadLabGraph graph = RoadLabGraph::fromJson(g);
    RoadLayout layout = deriveLayout(graph, LayoutSource::Inferred);
    writeLayout(g, layout);
    for (nlohmann::json& j : g["junctions"]) {
        const bool cross = std::fabs(j["at"][0].get<double>()) < 1.0;
        j["control"] = cross ? "all_way_stop" : "give_way";
        for (nlohmann::json& a : j["arms"]) {
            if (cross) a["regulations"] = {"stop"};
            else if (a["road"] == "l2") a["regulations"] = {"give_way"};
        }
    }
    return g;
}
}  // namespace

TEST_CASE(road_model_9_the_signs_follow_from_the_facts) {
    // THE ROWS are data: each kind with a `sign` row, in priority order, every one read
    std::vector<std::string> problems;
    const std::vector<engine::ControlSignRule> rules = engine::controlSignRules(attachmentKinds(), &problems);
    for (const std::string& p : problems) std::printf("    %s\n", p.c_str());
    CHECK(problems.empty());
    std::set<std::string> kinds;
    for (std::size_t i = 0; i < rules.size(); ++i) {
        kinds.insert(rules[i].kind);
        if (i) CHECK(rules[i - 1].priority <= rules[i].priority);
        CHECK(!rules[i].source.empty());
    }
    CHECK(kinds == std::set<std::string>({"stop_sign", "give_way_sign", "do_not_enter_sign", "one_way_sign"}));
#ifdef RT_ENABLE_SCRIPTING
    // ...and what each means is the rule set's device (rules/lab.lua `devices`)
    {
        engine::ScriptVM vm;
        driving::RuleSet rs;
        std::string err;
        const std::string code = engine::loadScriptCode(citysim::kDefaultRuleSet, "");
        CHECK(!code.empty() && vm.doString(code, &err) && driving::loadRuleSet(vm, citysim::kDefaultRuleSet, rs, &err));
        for (const engine::ControlSignRule& r : rules) {
            if (r.means.empty()) continue;
            bool device = false;
            for (const driving::Device& d : rs.devices) device = device || d.name == r.means;
            CHECK(device);
            CHECK(r.regulation == r.means);   // it stands where the arm states that sign
        }
    }
#endif

    const RoadModel m = plannedRoadModel(RoadLabGraph::fromJson(signedGraph()));
    const engine::ControlSignPlan plan = engine::placeControlSigns(m, rules, nullptr, {});
    for (const std::string& n : plan.notes) std::printf("    %s\n", n.c_str());
    CHECK(plan.notes.empty());
    std::map<std::string, int> count;
    for (const engine::PlacedSign& s : plan.signs) ++count[s.kind];
    // four STOP signs at the all-way stop, each with its ALL WAY plaque; one YIELD on the T's local street; one DO NOT
    // ENTER at the one-way street's exit end; a ONE WAY sign facing each of the two arterial approaches that cross it
    CHECK(count["stop_sign"] == 4);
    CHECK(count["give_way_sign"] == 1);
    CHECK(count["do_not_enter_sign"] == 1);
    CHECK(count["one_way_sign"] == 2);
    std::vector<std::string> arrows;
    for (const engine::PlacedSign& s : plan.signs) {
        const ModelJunction& J = m.junctions[static_cast<std::size_t>(s.junction)];
        const ModelArm& a = J.arms[static_cast<std::size_t>(s.arm)];
        const ModelStreet& S = m.streets[a.street.index];
        Vec2 mouth;
        CHECK(armMouth(m, J, static_cast<std::size_t>(s.arm), mouth));
        const Vec2 d(std::cos(a.heading * DEG_TO_RAD), std::sin(a.heading * DEG_TO_RAD));
        const Vec2 off = s.at - mouth;
        // on the pavement: past the kerb across the arm's line, never on the carriageway
        CHECK(std::fabs(dot(off, rightOf(d))) > kerbOffset(S) + 0.5);
        // attached to its arm through the model
        RoadModel withSign = m;
        ModelAttachment at;
        at.kind = s.kind;
        at.junction = ModelJunctionId(static_cast<uint32_t>(s.junction));
        at.arm = s.arm;
        at.at = s.at;
        CHECK(attach(withSign, at));
        if (s.kind == "stop_sign" || s.kind == "give_way_sign") {
            // at the stop line, 0.6 m toward the junction; on the kerb side of the traffic arriving by the arm, facing it
            CHECK(std::fabs(dot(off, d) - (stopLineFromMouth(m, J, static_cast<std::size_t>(s.arm)) - 0.6)) < 1e-6);
            CHECK(dot(off, rightOf(d * -1.0)) > 0);
            CHECK((s.face - d).length() < 1e-9);
            CHECK(s.look.value("sign", std::string()) == (s.kind == "stop_sign" ? "stop" : "yield"));
            CHECK(s.look.contains("plaques") == (J.control == "all_way_stop"));
        } else if (s.kind == "do_not_enter_sign") {
            // at the street's exit end, facing back into the junction, on the kerb side of a car that would drive in
            CHECK(a.road == "o0" && a.spineIn);
            CHECK((s.face + d).length() < 1e-9);
            CHECK(dot(off, rightOf(d)) > 0);
        } else {
            // parallel to the one-way street, facing an arterial approach; the street runs one way, so the two readers,
            // facing each other across it, see its arrow point opposite ways
            CHECK(a.road == "o0");
            CHECK(std::fabs(dot(s.face, d)) < 0.05);
            arrows.push_back(s.look.value("arrow", std::string()));
        }
    }
    CHECK(arrows.size() == 2u && arrows[0] != arrows[1]);
    // two signs on one spot are back to back, never one standing in the other
    for (std::size_t i = 0; i < plan.signs.size(); ++i)
        for (std::size_t k = i + 1; k < plan.signs.size(); ++k)
            if ((plan.signs[i].at - plan.signs[k].at).length() < 1.0) CHECK(dot(plan.signs[i].face, plan.signs[k].face) < -0.9);
    // NO ROOM: a sign whose spot is on the carriageway, or too near something already standing, is not placed, and said
    const engine::ControlSignPlan onRoad = engine::placeControlSigns(m, rules, [](const Vec2&) { return Real(1); }, {});
    CHECK(onRoad.signs.empty() && onRoad.notes.size() == plan.signs.size());
    std::vector<Vec2> standing;
    for (const engine::PlacedSign& s : plan.signs)
        if (s.kind == "give_way_sign") standing.push_back(s.at + Vec2(0.3, 0));
    const engine::ControlSignPlan crowded = engine::placeControlSigns(m, rules, nullptr, standing);
    CHECK(crowded.signs.size() + 1 == plan.signs.size() && crowded.refused.at("give_way_sign") == 1);
    // A ROAD FILE THAT STATES NO SIGNS gets none of the regulation signs (the one-way street's signs follow from its lanes)
    nlohmann::json bare = signedGraph();
    for (nlohmann::json& j : bare["junctions"]) {
        j.erase("control");
        for (nlohmann::json& a : j["arms"]) a.erase("regulations");
    }
    const engine::ControlSignPlan none = engine::placeControlSigns(plannedRoadModel(RoadLabGraph::fromJson(bare)), rules, nullptr, {});
    for (const engine::PlacedSign& s : none.signs) CHECK(s.kind == "do_not_enter_sign" || s.kind == "one_way_sign");
}

// ---- 10. the regions enclosed by streets (ADR-0423) ---------------------------------------------------------------------

namespace {
// A GRID OF STREETS for the regions: rows at y = 0, 100, 200, 300 and columns at x = 0 and 100 (local streets), and at
// x = 200 a DIVIDED ROAD: two one-way carriageways 12 m apart (x = 194 running +y, x = 206 running -y, stated as one road),
// each row cut where it meets them. The grid's outer corners are bends (a row carries on as a column). A local street
// leaves the junction at (100, 100) and ends inside the block beyond it, at (160, 160): a dead end. A column at x = 300
// closes the east side. Its layout and names written, as the one-off tool writes them.
nlohmann::json regionGrid() {
    nlohmann::json g;
    g["name"] = "region_grid";
    g["classes"]["oneway"] = {{"w", 3.5}, {"fwd", 1}, {"back", 0}, {"sidewalk", 2.0}, {"g_max", 0.09}, {"window", 40}, {"thick", 0.5}};
    auto road = [](const std::string& id, const std::string& cls, const std::vector<Vec2>& pts) {
        nlohmann::json p = nlohmann::json::array();
        for (const Vec2& q : pts) p.push_back({q.x, q.y});
        return nlohmann::json{{"id", id}, {"class", cls}, {"path", {{"type", "polyline"}, {"points", p}}}};
    };
    nlohmann::json edges = nlohmann::json::array();
    const double xs[] = {0, 100, 194, 206, 300};
    for (int j = 0; j < 4; ++j)   // the rows, one edge between each pair of crossings
        for (int i = 0; i + 1 < 5; ++i)
            edges.push_back(road("r" + std::to_string(j) + "_" + std::to_string(i), "local", {Vec2(xs[i], 100.0 * j), Vec2(xs[i + 1], 100.0 * j)}));
    for (int j = 0; j < 3; ++j) {
        const double y0 = 100.0 * j, y1 = y0 + 100;
        edges.push_back(road("c0_" + std::to_string(j), "local", {Vec2(0, y0), Vec2(0, y1)}));
        edges.push_back(road("c1_" + std::to_string(j), "local", {Vec2(100, y0), Vec2(100, y1)}));
        edges.push_back(road("c4_" + std::to_string(j), "local", {Vec2(300, y0), Vec2(300, y1)}));
        edges.push_back(road("dv_a" + std::to_string(j), "oneway", {Vec2(194, y0), Vec2(194, y1)}));   // +y
        edges.push_back(road("dv_b" + std::to_string(j), "oneway", {Vec2(206, y1), Vec2(206, y0)}));   // -y
    }
    edges.push_back(road("dead", "local", {Vec2(100, 100), Vec2(160, 160)}));
    g["edges"] = edges;
    nlohmann::json divided = nlohmann::json::array();
    for (int j = 0; j < 3; ++j) divided.push_back({"dv_a" + std::to_string(j), "dv_b" + std::to_string(j)});
    g["divided"] = divided;
    const RoadLabGraph graph = RoadLabGraph::fromJson(g);
    RoadLayout layout = deriveLayout(graph, LayoutSource::Inferred);
    nameLayout(layout, graph);
    writeLayout(g, layout);
    return g;
}
}  // namespace

TEST_CASE(road_model_10_the_regions_enclosed_by_streets) {
    RoadLabGraph g = RoadLabGraph::fromJson(regionGrid());
    (void)expand(g);
    const RoadModel m = plannedRoadModel(g);
    const EnclosedRegions er = enclosedRegions(m);
    std::printf("%s", reportEnclosedRegions(m, er).c_str());
    // THE LAND: three columns of blocks (0-100, 100-194, 206-300) in three rows; between the carriageways, three medians, no
    // region; one outer face; the walk closed everything as stated
    CHECK(er.regions.size() == 9);
    CHECK(er.medians.size() == 3);
    CHECK(er.outer.size() == 1 && er.pieces == 1);
    CHECK(er.problems.empty() && er.crossings.empty() && er.unplanar.empty() && er.nested.empty());
    CHECK(er.passedOver.empty() && er.walked == m.streets.size());
    // EACH BOUNDED IN ORDER, counter-clockwise: every step starts where the one before it ended (at the junction it named,
    // or on the street it carried on into, or back along itself where it turned round), and the region lies on the
    // side each bound says
    for (const EnclosedRegion& R : er.regions) {
        CHECK(R.area > 0 && R.bounds.size() >= 3);
        for (std::size_t i = 0; i < R.bounds.size(); ++i) {
            const RegionBound& a = R.bounds[i];
            const RegionBound& b = R.bounds[(i + 1) % R.bounds.size()];
            const ModelStreet& A = m.streets[a.street.index];
            const ModelStreet& B = m.streets[b.street.index];
            const int aEnd = a.spineForward ? 1 : 0, bStart = b.spineForward ? 0 : 1;
            if (a.end == RegionStepEnd::Junction) {
                CHECK(A.junctions[static_cast<std::size_t>(aEnd)] == a.junction);
                CHECK(B.junctions[static_cast<std::size_t>(bStart)] == a.junction);
            } else if (a.end == RegionStepEnd::CarriesOn) {
                CHECK(A.carriesOn[static_cast<std::size_t>(aEnd)] == b.street);
            } else {
                CHECK(b.street == a.street && b.spineForward != a.spineForward);
            }
            // the side: a point a metre off the middle of the street, on the stated side, lies inside the outline
            const Vec2 p0 = m.linePoints[A.line.first], p1 = m.linePoints[A.line.end() - 1];
            const Vec2 dir = normalize(p1 - p0), ccw(-dir.y, dir.x);
            const Vec2 off = (p0 + p1) * 0.5 + (a.side == RegionSide::Ccw ? ccw : ccw * -1.0);
            CHECK(pointInPolygon(R.outline, off));
        }
        CHECK(R.corners.size() >= 2);
    }
    // THE OUTER CORNER at (0, 0) is a bend, no junction: the block there is bounded by the row carrying on into the column
    {
        const int r = regionAt(er, Vec2(50, 50));
        CHECK(r >= 0);
        bool carries = false;
        if (r >= 0)
            for (const RegionBound& b : er.regions[static_cast<std::size_t>(r)].bounds) carries = carries || b.end == RegionStepEnd::CarriesOn;
        CHECK(carries);
    }
    // A DEAD END INSIDE A BLOCK splits nothing: the block it stands in is one region, walked down one side of it and back
    // up the other
    {
        const int r = regionAt(er, Vec2(170, 120));
        CHECK(r >= 0 && r == regionAt(er, Vec2(120, 180)));
        uint32_t sides = 0;
        if (r >= 0)
            for (const RegionBound& b : er.regions[static_cast<std::size_t>(r)].bounds) sides += m.streets[b.street.index].road == "dead" ? 1u : 0u;
        CHECK(sides == 2);
        CHECK(er.deadEnds.size() == 1);
    }
    // THE MEDIAN between the carriageways is no region
    CHECK(regionAt(er, Vec2(200, 50)) < 0);
    bool inMedian = false;
    for (const EnclosedRegion& f : er.medians) inMedian = inMedian || pointInPolygon(f.outline, Vec2(200, 150));
    CHECK(inMedian);
    // A LOOK-UP: asked twice, the same answer; the model itself unchanged by the asking
    const RoadModel before = m;
    const EnclosedRegions again = enclosedRegions(m);
    CHECK(again.regions.size() == er.regions.size() && m == before);
    for (std::size_t i = 0; i < er.regions.size() && i < again.regions.size(); ++i) CHECK(again.regions[i].outline.size() == er.regions[i].outline.size());
}

// ---- 11. the lane level: a table built from the road model (step 8, ADR-0426) ---------------------------------------------

namespace {
// The lane level's small scene: the signed graph (an all-way stop at (0, 0), a give-way T at (200, 0), a local street
// carrying on at a bend at (0, 150)) with its lane rules stated.
nlohmann::json laneLevelGraph() {
    nlohmann::json g = signedGraph();
    g["lane_rules"] = {{"solid_before_stop_line", 20.0}};
    return g;
}
LaneGraphOptions oneCar() {
    LaneGraphOptions o;
    VehicleSize car;
    car.tag = "car";
    car.turningRadius = 5.0;
    car.width = 1.9;
    car.length = 4.5;
    o.vehicles = {car};
    return o;
}
// The first lane path cut from model lane `lane` (in order along it), and its last.
std::vector<uint32_t> pathsOfLane(const LaneGraphBuild& lg, uint32_t lane) {
    std::vector<uint32_t> out;
    for (uint32_t p = 0; p < lg.graph.paths.size(); ++p)
        if (lg.graph.paths[p].kind == PathKind::Lane && lg.graph.paths[p].modelLane == lane) out.push_back(p);
    std::sort(out.begin(), out.end(), [&](uint32_t a, uint32_t b) { return lg.sources[a].from < lg.sources[b].from; });
    return out;
}
bool leadsInto(const LaneGraph& G, uint32_t from, uint32_t to) {
    for (uint32_t k = G.paths[from].next.first; k < G.paths[from].next.end(); ++k)
        if (G.nextPaths[k].index == to) return true;
    return false;
}
}  // namespace

TEST_CASE(road_model_11_which_turns_a_junction_offers_is_stated_by_its_arms_order) {
    // a four-way: arms leaving east (0), north (1), west (2), south (3); east-west and north-south are through pairs
    std::vector<TurnArm> arms(4);
    const double headings[4] = {0, 90, 180, 270};
    for (int a = 0; a < 4; ++a) {
        arms[a].heading = headings[a];
        arms[a].hasHeading = true;
    }
    const std::vector<std::array<uint32_t, 2>> pairs = {{0, 2}, {1, 3}};
    const bool kerbCcw = kerbSideCounterClockwise();
    const std::vector<Turn> fromWest = turnKindsFrom(arms, pairs, 2, kerbCcw);
    CHECK(fromWest[0] == Turn::Straight && fromWest[2] == Turn::None);
    // traffic arriving from the west runs east; its kerb side, where the near turn goes, is the side the one statement
    // of the kerb (kerbPerpSign) gives: the plan's -y (this world drives on the left, its plan mirrored into the world)
    CHECK(kerbCcw == (roads::lanes::kerbPerpSign() < 0));
    CHECK(fromWest[3] == (kerbCcw ? Turn::NearTurn : Turn::FarTurn));
    CHECK(fromWest[1] == (kerbCcw ? Turn::FarTurn : Turn::NearTurn));
    // the stem of a T (south, no partner): one arm on each side; and with no partner at a four-way, the arm halfway
    // round is a far turn, reported
    std::vector<TurnArm> tee = {arms[0], arms[2], arms[3]};
    const std::vector<Turn> fromStem = turnKindsFrom(tee, {{0, 1}}, 2, kerbCcw);
    CHECK(fromStem[0] != fromStem[1] && fromStem[0] != Turn::Straight && fromStem[1] != Turn::Straight);
    std::vector<uint32_t> halfway;
    const std::vector<Turn> unpaired = turnKindsFrom(arms, {}, 2, kerbCcw, &halfway);
    CHECK(unpaired[0] == Turn::FarTurn && halfway.size() == 1u && halfway.front() == 0u);
    // the table: two lanes each way on the through road, one on the cross street; the kerb lane takes the near turn, the
    // centre-line lane the far turn, both go straight on by slot
    for (TurnArm& a : arms) {
        a.arriving.clear();
        a.leaving.clear();
    }
    uint32_t id = 0;
    for (int a : {0, 2})
        for (int s = 0; s < 2; ++s) {
            arms[a].arriving.push_back(TurnArmLane{id++, "through", s});
            arms[a].leaving.push_back(TurnArmLane{id++, "through", s});
        }
    for (int a : {1, 3}) {
        arms[a].arriving.push_back(TurnArmLane{id++, "through", 0});
        arms[a].leaving.push_back(TurnArmLane{id++, "through", 0});
    }
    const std::vector<StatedTurn> turns = turnsAtJunction(arms, pairs, "all_way_stop", laneTurnRules());
    uint32_t fromWestCount = 0;
    for (const StatedTurn& t : turns) {
        if (t.fromArm != 2) continue;
        ++fromWestCount;
        const int slot = t.from == arms[2].arriving[0].lane ? 0 : 1;
        if (t.turn == Turn::Straight) CHECK((slot == 0 && t.to == arms[0].leaving[0].lane) || (slot == 1 && t.to == arms[0].leaving[1].lane));
        if (t.turn == Turn::NearTurn) CHECK(slot == 1);
        if (t.turn == Turn::FarTurn) CHECK(slot == 0);
    }
    CHECK(fromWestCount == 4u);   // two straight on, one near turn, one far turn
}

TEST_CASE(road_model_12_the_lane_level_is_a_table_built_from_the_model) {
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(laneLevelGraph()));
    const RoadModel m = assembleRoadModel(*r);
    const LaneGraphBuild lg = buildLaneLevel(*r, m, oneCar());
    const LaneGraph& G = lg.graph;
    CHECK(lg.error.empty());
    for (const std::string& p : validate(G)) std::printf("      [lane level] %s\n", p.c_str());
    CHECK(validate(G).empty());
    // one junction per stated junction, each carrying its id; every lane path its street and lane; every connector its turn
    CHECK(G.junctions.size() == m.junctions.size());
    for (uint32_t j = 0; j < G.junctions.size(); ++j) CHECK(G.junctions[j].modelJunction == j);
    for (const Path& p : G.paths) {
        if (p.kind == PathKind::Lane) {
            CHECK(p.street < m.streets.size() && p.modelLane < m.lanes.size());
            bool onIt = false;
            for (const ModelLaneSpan& sp : m.lanes[p.modelLane].streets) onIt = onIt || sp.street.index == p.street;
            CHECK(onIt);
            continue;
        }
        CHECK(p.modelTurn < m.turns.size());
        const ModelTurn& t = m.turns[p.modelTurn];
        CHECK(G.paths[p.from.index].modelLane == t.from.index);
        CHECK(G.paths[G.nextPaths[p.next.first].index].modelLane == t.to.index);
        CHECK(p.turn == t.turn);
        CHECK(G.junctions[p.junction.index].modelJunction == t.junction.index);
    }
    // the controls are the stated ones; the all-way stop's arriving lanes face its stop signs
    for (const Junction& jn : G.junctions) CHECK(G.controlNames[jn.control] == m.junctions[jn.modelJunction].control);
    // at the crossing, a lane of the arterial arriving from the west turns near into the street on its kerb side
    {
        uint32_t near = 0;
        for (const ModelTurn& t : m.turns)
            if (m.junctions[t.junction.index].id == m.junctions[G.junctions[0].modelJunction].id && t.turn == Turn::NearTurn) ++near;
        CHECK(near > 0);
    }
    // the bend at (0, 150): no junction, the local street's lane carries on into the next street's lane of its slot
    {
        const ModelLaneId l1 = laneNamed(m, "l1.f0"), l3 = laneNamed(m, "l3.f0");
        CHECK(l1.valid() && l3.valid());
        const std::vector<uint32_t> a = pathsOfLane(lg, l1.index), b = pathsOfLane(lg, l3.index);
        CHECK(!a.empty() && !b.empty());
        if (!a.empty() && !b.empty()) CHECK(leadsInto(G, a.back(), b.front()));
        CHECK(lg.carryOnJoins >= 2u);
    }
    // the lines before a stop line are solid within the stated stretch, and may be crossed further back
    {
        uint32_t solid = 0, open = 0;
        for (const Section& s : G.sections)
            for (uint32_t k = s.lines.first; k < s.lines.end(); ++k) (G.laneLines[k].crossFromOffside ? open : solid)++;
        CHECK(solid > 0 && open > 0);
    }
    // a road file that does not state its lane rules gets no lane level, and is told what to run
    nlohmann::json bare = laneLevelGraph();
    bare.erase("lane_rules");
    const std::unique_ptr<Result> rb = build(RoadLabGraph::fromJson(bare));
    const LaneGraphBuild none = buildLaneLevel(*rb, assembleRoadModel(*rb), oneCar());
    CHECK(!none.error.empty() && none.error.find("--lane-rules") != std::string::npos && none.graph.paths.empty());
}

TEST_CASE(road_model_13_a_ramp_is_a_way_on_at_the_lane_level) {
    // merge_taper: three ramps, each from a feeder street (an arm of a junction there) merging into the mainline's kerb
    // lane at its stated anchor
    nlohmann::json g = nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/lanelab/merge_taper.json"));
    g.erase("junctions");
    g.erase("streets");
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    for (nlohmann::json& j : g["junctions"]) j["control"] = "unmarked";
    g["lane_rules"] = {{"solid_before_stop_line", 20.0}};
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    const RoadModel m = assembleRoadModel(*r);
    const LaneGraphBuild lg = buildLaneLevel(*r, m, oneCar());
    CHECK(lg.error.empty());
    CHECK(validate(lg.graph).empty());
    std::printf("      [lane level] merge_taper: %u lanes merge into their hosts, %u leave them, %u joins with a gap\n", lg.mergeJoins,
                lg.divergeJoins, lg.joinsWithGap);
    CHECK(lg.mergeJoins >= 3u);   // each ramp into the mainline (and a two-lane ramp's second lane into its first)
    const ModelLaneId host = laneNamed(m, "mainline.f2");
    for (const char* ramp : {"ramp_one.f0", "ramp_short.f0", "ramp_long.f0"}) {
        const std::vector<uint32_t> ps = pathsOfLane(lg, laneNamed(m, ramp).index);
        CHECK(!ps.empty());
        if (ps.empty()) continue;
        // its end leads into the mainline's kerb lane, from where their lines meet
        const Path& last = lg.graph.paths[ps.back()];
        CHECK(last.next.count == 1u);
        if (last.next.count == 1u) CHECK(lg.graph.paths[lg.graph.nextPaths[last.next.first].index].modelLane == host.index);
        // and where the two join, the measured crossing points hold a join of the two lanes
        bool join = false;
        for (uint32_t k = last.crossings.first; k < last.crossings.end(); ++k) {
            const CrossingPoint& x = lg.graph.crossings[lg.graph.pathCrossings[k].index];
            join = join || (x.meeting == Meeting::Join && x.otherSide == OtherSide::Path);
        }
        CHECK(join);
    }
}

// ---- 14. the turn table, junction by junction, by hand (lane_turns.h, assets/data/lane_turns.json) -----------------------

namespace {
// A junction by hand: one arm per heading (degrees, plan coordinates), each with `lanes[a]` lanes arriving and leaving,
// all of kind `kind` (slots 0 from the centre-line side). Arm a's arriving lane of slot s is 100 a + s, its leaving lane
// of slot s 100 a + 50 + s.
std::vector<TurnArm> handJunction(const std::vector<double>& headings, const std::vector<int>& lanes, const std::string& kind = "through") {
    std::vector<TurnArm> arms(headings.size());
    for (std::size_t a = 0; a < headings.size(); ++a) {
        arms[a].heading = headings[a];
        arms[a].hasHeading = true;
        for (int s = 0; s < lanes[a]; ++s) {
            arms[a].arriving.push_back(TurnArmLane{static_cast<uint32_t>(100 * a + s), kind, s});
            arms[a].leaving.push_back(TurnArmLane{static_cast<uint32_t>(100 * a + 50 + s), kind, s});
        }
    }
    return arms;
}
using TurnSet = std::set<std::tuple<uint32_t, uint32_t, Turn>>;   // (from lane, into lane, kind)
TurnSet turnSet(const std::vector<StatedTurn>& ts) {
    TurnSet out;
    for (const StatedTurn& t : ts) out.insert(std::make_tuple(t.from, t.to, t.turn));
    return out;
}
void printTurns(const char* what, const TurnSet& s) {
    std::printf("      %s:", what);
    for (const std::tuple<uint32_t, uint32_t, Turn>& t : s) std::printf(" %u>%u %s;", std::get<0>(t), std::get<1>(t), turnKey(std::get<2>(t)));
    std::printf("\n");
}
}  // namespace

TEST_CASE(road_model_14_the_turn_table_by_hand) {
    const LaneTurnRules& rules = laneTurnRules();
    CHECK(rules.rows.size() == 5u);
    const Turn S = Turn::Straight, N = Turn::NearTurn, F = Turn::FarTurn;
    // THE KERB SIDE IN THIS WORLD: counter-clockwise from an arriving arm, in plan coordinates (kerbPerpSign < 0)
    CHECK(kerbSideCounterClockwise());
    // A FOUR-WAY, one lane each way, arms leaving east (0), north (1), west (2), south (3), east-west and north-south
    // through pairs: exactly twelve turns. From each arm: straight on into its partner; the near turn into the next arm
    // counter-clockwise (the kerb side); the far turn into the one clockwise.
    {
        const std::vector<TurnArm> arms = handJunction({0, 90, 180, 270}, {1, 1, 1, 1});
        const TurnSet got = turnSet(turnsAtJunction(arms, {{0, 2}, {1, 3}}, "all_way_stop", rules, nullptr, true));
        const TurnSet want = {{0, 250, S}, {0, 150, N}, {0, 350, F},     // from the east arm
                              {100, 350, S}, {100, 250, N}, {100, 50, F},  // from the north
                              {200, 50, S}, {200, 350, N}, {200, 150, F},  // from the west
                              {300, 150, S}, {300, 50, N}, {300, 250, F}}; // from the south
        if (got != want) printTurns("four-way", got);
        CHECK(got == want);
        // the mirrored world (kerb clockwise): near and far change places, straight on stays
        const TurnSet mirrored = turnSet(turnsAtJunction(arms, {{0, 2}, {1, 3}}, "all_way_stop", rules, nullptr, false));
        const TurnSet wantMirrored = {{0, 250, S}, {0, 350, N}, {0, 150, F},   {100, 350, S}, {100, 50, N}, {100, 250, F},
                                      {200, 50, S}, {200, 150, N}, {200, 350, F}, {300, 150, S}, {300, 250, N}, {300, 50, F}};
        if (mirrored != wantMirrored) printTurns("four-way mirrored", mirrored);
        CHECK(mirrored == wantMirrored);
    }
    // KERB-SIDE AND CENTRE-LINE-SIDE PAIRING: three lanes each way east-west, two north-south. From the west: every lane
    // straight on in its own slot; the near turn from the kerb lane (slot 2) into the south arm's kerb lane (slot 1); the
    // far turn from the centre-line lane (slot 0) into the north arm's centre-line lane (slot 0).
    {
        const std::vector<TurnArm> arms = handJunction({0, 90, 180, 270}, {3, 2, 3, 2});
        TurnSet fromWest;
        for (const StatedTurn& t : turnsAtJunction(arms, {{0, 2}, {1, 3}}, "signal", rules, nullptr, true))
            if (t.fromArm == 2) fromWest.insert(std::make_tuple(t.from, t.to, t.turn));
        const TurnSet want = {{200, 50, S}, {201, 51, S}, {202, 52, S}, {202, 351, N}, {200, 150, F}};
        if (fromWest != want) printTurns("from the west", fromWest);
        CHECK(fromWest == want);
    }
    // FEWER LANES LEAVING THAN ARRIVING: three lanes arrive from the west, two leave east: straight on by slot, the kerb lane
    // into the nearest slot there is (slot 1)
    {
        std::vector<TurnArm> arms = handJunction({0, 180}, {2, 3});
        TurnSet got = turnSet(turnsAtJunction(arms, {{0, 1}}, "unmarked", rules, nullptr, true));
        TurnSet fromWest;
        for (const std::tuple<uint32_t, uint32_t, Turn>& t : got)
            if (std::get<0>(t) >= 100) fromWest.insert(t);
        const TurnSet want = {{100, 50, S}, {101, 51, S}, {102, 51, S}};
        if (fromWest != want) printTurns("three into two", fromWest);
        CHECK(fromWest == want);
    }
    // A T's STEM (south, no way straight on): the arm counter-clockwise from it (east) is its near turn, the other (west)
    // its far turn. Both `no_straight_on` rows: with three stem lanes the kerb side's larger share (slots 2 and 1) take the
    // near turn, kerb lane into kerb lane, and the centre-line side's smaller (slot 0) the far turn; with one lane, it
    // takes both.
    {
        const std::vector<TurnArm> arms = handJunction({0, 180, 270}, {2, 2, 3});
        TurnSet fromStem;
        for (const StatedTurn& t : turnsAtJunction(arms, {{0, 1}}, "stop", rules, nullptr, true))
            if (t.fromArm == 2) fromStem.insert(std::make_tuple(t.from, t.to, t.turn));
        const TurnSet want = {{202, 51, N}, {201, 50, N}, {200, 150, F}};
        if (fromStem != want) printTurns("T stem, three lanes", fromStem);
        CHECK(fromStem == want);
        const std::vector<TurnArm> one = handJunction({0, 180, 270}, {1, 1, 1});
        TurnSet fromOne;
        for (const StatedTurn& t : turnsAtJunction(one, {{0, 1}}, "stop", rules, nullptr, true))
            if (t.fromArm == 2) fromOne.insert(std::make_tuple(t.from, t.to, t.turn));
        const TurnSet wantOne = {{200, 50, N}, {200, 150, F}};
        if (fromOne != wantOne) printTurns("T stem, one lane", fromOne);
        CHECK(fromOne == wantOne);
        // and the T's through road: from the east (arm 0) the stem lies clockwise, a far turn; from the west, a near turn
        TurnSet fromThrough;
        for (const StatedTurn& t : turnsAtJunction(one, {{0, 1}}, "stop", rules, nullptr, true))
            if (t.fromArm != 2) fromThrough.insert(std::make_tuple(t.from, t.to, t.turn));
        const TurnSet wantThrough = {{0, 150, S}, {0, 250, F}, {100, 50, S}, {100, 250, N}};
        if (fromThrough != wantThrough) printTurns("T through road", fromThrough);
        CHECK(fromThrough == wantThrough);
    }
    // A FIVE-WAY, one lane each way, arms at 0, 60, 150, 200 and 290 degrees, 0 and 2 a through pair. From arm 0: near into
    // 1 (counter-clockwise before its partner), far into 3 and 4. From arm 1, no partner: an odd ring has no halfway arm;
    // near into the two counter-clockwise (2, 3), far into the other two (4, 0), its one lane taking both kinds.
    {
        const std::vector<TurnArm> arms = handJunction({0, 60, 150, 200, 290}, {1, 1, 1, 1, 1});
        std::vector<std::string> notes;
        const std::vector<StatedTurn> ts = turnsAtJunction(arms, {{0, 2}}, "signal", rules, &notes, true);
        TurnSet from0, from1;
        for (const StatedTurn& t : ts) {
            if (t.fromArm == 0) from0.insert(std::make_tuple(t.from, t.to, t.turn));
            if (t.fromArm == 1) from1.insert(std::make_tuple(t.from, t.to, t.turn));
        }
        const TurnSet want0 = {{0, 250, S}, {0, 150, N}, {0, 350, F}, {0, 450, F}};
        const TurnSet want1 = {{100, 250, N}, {100, 350, N}, {100, 450, F}, {100, 50, F}};
        if (from0 != want0) printTurns("five-way from 0", from0);
        if (from1 != want1) printTurns("five-way from 1", from1);
        CHECK(from0 == want0);
        CHECK(from1 == want1);
        for (const std::string& n : notes) CHECK(n.find("halfway") == std::string::npos);
    }
    // A ROW FOR SOME CONTROLS ONLY: a table whose far turn is offered at a stop only; a u_turn row is refused when read (no
    // junction offers a U-turn: it could never apply)
    {
        const nlohmann::json doc = {{"turns", nlohmann::json::array({
            {{"turn", "straight"}, {"when", "any"}, {"kinds", {"through"}}, {"lanes", "every"}, {"into", "same_slot"}, {"source", "test"}},
            {{"turn", "far_turn"}, {"when", "any"}, {"controls", {"stop"}}, {"kinds", {"through"}}, {"lanes", {{"from", "centre"}, {"count", 1u}}},
             {"into", "same_side"}, {"source", "test"}},
            {{"turn", "u_turn"}, {"kinds", {"through"}}, {"lanes", "every"}, {"into", "same_slot"}, {"source", "test"}}})}};
        std::vector<std::string> problems;
        const LaneTurnRules table = parseLaneTurnRules(doc, &problems);
        CHECK(table.rows.size() == 2u && problems.size() == 1u);
        if (problems.size() == 1u) CHECK(problems.front().find("u_turn") != std::string::npos);
        const std::vector<TurnArm> arms = handJunction({0, 90, 180, 270}, {1, 1, 1, 1});
        const TurnSet atStop = turnSet(turnsAtJunction(arms, {{0, 2}, {1, 3}}, "stop", table, nullptr, true));
        const TurnSet atSignal = turnSet(turnsAtJunction(arms, {{0, 2}, {1, 3}}, "signal", table, nullptr, true));
        CHECK(atStop.size() == 8u && atSignal.size() == 4u);
        for (const std::tuple<uint32_t, uint32_t, Turn>& t : atSignal) CHECK(std::get<2>(t) == S);
        CHECK(atStop.count(std::make_tuple(200u, 150u, F)) == 1u);
    }
    // AN ARRIVING LANE NO ROW COVERS IS SAID, never dropped quietly: a ramp's lane (the table names no `ramp` kind) arriving
    // at a junction, and an arm with nowhere to go
    {
        std::vector<TurnArm> arms = handJunction({0, 90, 180, 270}, {1, 1, 1, 1});
        arms[3].arriving.front().kind = "ramp";
        arms[3].arriving.front().name = "offramp.f0";
        std::vector<std::string> notes;
        const std::vector<StatedTurn> ts = turnsAtJunction(arms, {{0, 2}, {1, 3}}, "stop", rules, &notes, true);
        for (const StatedTurn& t : ts) CHECK(t.from != 300u);
        bool said = false;
        for (const std::string& n : notes) said = said || (n.find("offramp.f0") != std::string::npos && n.find("takes no turn") != std::string::npos);
        CHECK(said);
        // two arms, the east one with no lane leaving by it: the west arm's lane has nowhere to go (said); the east's goes on
        std::vector<TurnArm> deadEnd = handJunction({0, 180}, {1, 1});
        deadEnd[0].leaving.clear();
        notes.clear();
        const TurnSet two = turnSet(turnsAtJunction(deadEnd, {{0, 1}}, "unmarked", rules, &notes, true));
        CHECK(two == TurnSet({{0, 150, S}}));
        CHECK(notes.size() == 1u);
        if (notes.size() == 1u) CHECK(notes.front().find("arriving lane #100") != std::string::npos && notes.front().find("no other arm") != std::string::npos);
    }
}

// ---- 15. a lane leaving its host and one merging into it, at the lane level (grid_city's freeway exit and entry) ----------

TEST_CASE(road_model_15_a_diverge_and_a_merge_at_the_lane_level) {
    const std::unique_ptr<Result> r = lane_level_test::buildWithLaneLevelFacts(std::string(RT_SOURCE_DIR) + "/assets/lanelab/grid_city.json");
    const RoadModel m = assembleRoadModel(*r);
    const LaneGraphBuild lg = buildLaneLevel(*r, m, oneCar());
    CHECK(lg.error.empty());
    const LaneGraph& G = lg.graph;
    uint32_t diverges = 0, merges = 0;
    for (const ModelConnection& c : m.connections) {
        if (c.ramp != "exit" && c.ramp != "entry") continue;
        if (c.kind == ConnectionKind::Diverge && c.ramp == "exit") {
            // THE EXIT: the host lane's path ending where their lines meet leads on along the host AND into the exit's
            // first path; the two paths' bodies split there (a measured crossing point)
            const std::vector<uint32_t> ramp = pathsOfLane(lg, c.to.index), host = pathsOfLane(lg, c.from.index);
            CHECK(!ramp.empty() && !host.empty());
            if (ramp.empty()) continue;
            int from = -1;
            for (uint32_t h : host)
                if (leadsInto(G, h, ramp.front())) from = static_cast<int>(h);
            CHECK(from >= 0);
            if (from < 0) continue;
            ++diverges;
            CHECK(G.paths[static_cast<std::size_t>(from)].next.count == 2u);
            bool onAlong = false;
            for (uint32_t k = G.paths[static_cast<std::size_t>(from)].next.first; k < G.paths[static_cast<std::size_t>(from)].next.end(); ++k)
                onAlong = onAlong || G.paths[G.nextPaths[k].index].modelLane == c.from.index;
            CHECK(onAlong);
            CHECK(distance(G.pointAt(PathId(static_cast<uint32_t>(from)), G.paths[static_cast<std::size_t>(from)].length),
                           G.pointAt(PathId(ramp.front()), 0)) <= 0.01);
        }
        if (c.kind == ConnectionKind::Merge && c.ramp == "entry" && c.to.index < m.lanes.size() && m.lanes[c.to.index].edge != m.lanes[c.from.index].edge) {
            // THE ENTRY: the ramp's last path leads into the host lane's path starting where their lines meet, and only there
            const std::vector<uint32_t> ramp = pathsOfLane(lg, c.from.index);
            CHECK(!ramp.empty());
            if (ramp.empty()) continue;
            const Path& last = G.paths[ramp.back()];
            CHECK(last.next.count == 1u);
            if (last.next.count != 1u) continue;
            ++merges;
            CHECK(G.paths[G.nextPaths[last.next.first].index].modelLane == c.to.index);
            CHECK(distance(G.pointAt(PathId(ramp.back()), last.length), G.pointAt(G.nextPaths[last.next.first], 0)) <= 0.01);
        }
    }
    std::printf("      [lane level] grid_city: %u diverges and %u merges checked\n", diverges, merges);
    // grid_city has one exit ramp and one entry ramp off its freeway, each of one lane (the file's edges `exit` and
    // `entry`, class ramp with one lane): exactly one diverge and one merge
    CHECK(diverges == 1u && merges == 1u);
    CHECK(lg.divergeJoins == 1u && lg.mergeJoins == 1u);
}

// ---- 16. what the lane level says instead of joining: a junction arm with no stop line, a turn naming no junction ----------

TEST_CASE(road_model_16_the_lane_level_reports_what_it_cannot_join) {
    // A STREET'S END AT A JUNCTION WITH NO ARM FOR IT: hill_junction's one junction, with both arms of the road that runs
    // through it (its lanes are one builder lane each, on the streets either side) taken out of the stated arms. Each of
    // that road's streets still names the junction at its end, so its lanes run on to the street's end there -- and are
    // NOT joined across it to the same lane on the far street (the join an equal station would have made), and each end
    // is a problem line.
    nlohmann::json g = lane_level_test::withLaneLevelFacts(nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/lanelab/hill_junction.json")));
    nlohmann::json& J = g["junctions"][0];
    const std::string road = J["arms"][0]["road"].get<std::string>();
    std::vector<int> newIndex;
    nlohmann::json arms = nlohmann::json::array();
    for (const nlohmann::json& a : J["arms"]) {
        newIndex.push_back(a["road"] == road ? -1 : static_cast<int>(arms.size()));
        if (a["road"] != road) arms.push_back(a);
    }
    nlohmann::json through = nlohmann::json::array();
    for (const nlohmann::json& t : J["through"]) {
        const int a = newIndex[t[0].get<std::size_t>()], b = newIndex[t[1].get<std::size_t>()];
        if (a >= 0 && b >= 0) through.push_back({a, b});
    }
    J["arms"] = arms;
    J["through"] = through;
    std::vector<std::string> streetsOfRoad;
    for (const nlohmann::json& st : g["streets"])
        if (st["road"] == road) streetsOfRoad.push_back(st["id"].get<std::string>());
    CHECK(streetsOfRoad.size() == 2u);   // the road runs through the junction: a street each side
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    const RoadModel m = assembleRoadModel(*r);
    const LaneGraphBuild lg = buildLaneLevel(*r, m, oneCar());
    CHECK(lg.error.empty());
    for (const std::string& st : streetsOfRoad) {
        bool said = false;
        for (const std::string& p : lg.problems)
            said = said || (p.find("street " + st + ": its ") == 0 && p.find("names junction " + J["id"].get<std::string>() + ", which has no arm with a "
                                                                              "stop line for it") != std::string::npos);
        CHECK(said);
    }
    uint32_t joinedAcross = 0, lanesOfRoad = 0;
    const LaneGraph& G = lg.graph;
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        if (G.paths[p].kind != PathKind::Lane || m.lanes[G.paths[p].modelLane].edge != r->graph.index.at(road)) continue;
        ++lanesOfRoad;
        for (uint32_t k = G.paths[p].next.first; k < G.paths[p].next.end(); ++k) {
            const Path& q = G.paths[G.nextPaths[k].index];
            joinedAcross += q.kind == PathKind::Lane && q.modelLane == G.paths[p].modelLane && q.street != G.paths[p].street ? 1 : 0;
        }
    }
    CHECK(lanesOfRoad > 0u);
    CHECK(joinedAcross == 0u);

    // A STATED TURN NAMING NO JUNCTION of the model: not built, and said (never dropped quietly)
    const std::unique_ptr<Result> r2 = lane_level_test::buildWithLaneLevelFacts(std::string(RT_SOURCE_DIR) + "/assets/lanelab/hill_junction.json");
    RoadModel m2 = assembleRoadModel(*r2);
    const std::size_t stated = m2.turns.size();
    CHECK(stated > 0u);
    ModelTurn stray = m2.turns.front();
    stray.junction = ModelJunctionId(static_cast<uint32_t>(m2.junctions.size() + 7));
    m2.turns.push_back(stray);
    const LaneGraphBuild lg2 = buildLaneLevel(*r2, m2, oneCar());
    const std::string expect = "stated turn " + std::to_string(stated) + " names no junction of the road model: not built";
    CHECK(std::find(lg2.problems.begin(), lg2.problems.end(), expect) != lg2.problems.end());
    uint32_t connectors = 0;
    for (const Path& p : lg2.graph.paths) connectors += p.kind == PathKind::Connector ? 1 : 0;
    CHECK(connectors + lg2.refusals.size() == stated);   // every turn at a junction built or refused; the stray neither
}

// ---- 17. one stated ring of a junction's arms (ADR-0427) ----------------------------------------------------------------

namespace {
// A road graph of straight roads, each a polyline from `a` to `b`, and one junction at (0, 0) whose arms are listed in the
// order given ({road, station, spine in}).
nlohmann::json ringScene(const std::vector<std::tuple<std::string, Vec2, Vec2>>& roads,
                         const std::vector<std::tuple<std::string, double, bool>>& arms) {
    nlohmann::json g;
    g["name"] = "ring_scene";
    g["classes"]["oneway"] = {{"w", 3.5}, {"fwd", 1}, {"back", 0}, {"sidewalk", 2.0}, {"g_max", 0.09}, {"window", 40}, {"thick", 0.5}};
    nlohmann::json edges = nlohmann::json::array();
    for (const std::tuple<std::string, Vec2, Vec2>& r : roads) {
        const Vec2 a = std::get<1>(r), b = std::get<2>(r);
        const bool oneWay = std::get<0>(r).find("bound") != std::string::npos;
        edges.push_back({{"id", std::get<0>(r)}, {"class", oneWay ? "oneway" : "local"},
                         {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}});
    }
    g["edges"] = edges;
    nlohmann::json list = nlohmann::json::array();
    for (const std::tuple<std::string, double, bool>& a : arms)
        list.push_back({{"road", std::get<0>(a)}, {"at", std::get<1>(a)}, {"spine", std::get<2>(a) ? "in" : "out"}});
    g["junctions"] = nlohmann::json::array({{{"id", "j"}, {"at", {0.0, 0.0}}, {"arms", list}}});
    return g;
}
std::vector<std::string> ringRoads(const nlohmann::json& g, std::vector<std::string>* notes = nullptr) {
    const RoadLabGraph rg = RoadLabGraph::fromJson(g);
    std::vector<std::string> out;
    for (std::size_t i : armRing(rg.junctions.front(), ComposedRoads(rg), notes)) out.push_back(rg.junctions.front().arms[i].road);
    return out;
}
}  // namespace

TEST_CASE(road_model_17_one_stated_ring_of_a_junctions_arms) {
    // A DIVIDED ROAD CROSSING A STREET, spread over its points (written out by hand): the street east-west (`east` from
    // (8, 0) out, `west` in to (-8, 0)); the divided road's two one-way carriageways 8 m apart, `northbound` at x = 4 and
    // `southbound` at x = -4, each cut at the street (their mouths 6 m either side of it). Counter-clockwise from the east
    // arm: the northbound carriageway leaving north (x = 4 first, on the clockwise side of the pair), the southbound one
    // arriving from the north (x = -4), the west arm, the southbound carriageway leaving south (x = -4), the northbound one
    // arriving from the south (x = 4). Listed scrambled; the ring is the one function's.
    const nlohmann::json divided = ringScene(
        {{"east", Vec2(8, 0), Vec2(100, 0)}, {"west", Vec2(-100, 0), Vec2(-8, 0)},
         {"northbound_n", Vec2(4, 6), Vec2(4, 100)}, {"northbound_s", Vec2(4, -100), Vec2(4, -6)},
         {"southbound_n", Vec2(-4, 100), Vec2(-4, 6)}, {"southbound_s", Vec2(-4, -6), Vec2(-4, -100)}},
        {{"west", 92.0, true}, {"southbound_s", 0.0, false}, {"east", 0.0, false}, {"northbound_s", 94.0, true},
         {"southbound_n", 94.0, true}, {"northbound_n", 0.0, false}});
    std::vector<std::string> notes;
    const std::vector<std::string> ring = ringRoads(divided, &notes);
    CHECK(notes.empty());
    const std::vector<std::string> want = {"east", "northbound_n", "southbound_n", "west", "southbound_s", "northbound_s"};
    std::printf("      divided crossing ring:");
    for (const std::string& r : ring) std::printf(" %s", r.c_str());
    std::printf("\n");
    CHECK(ring == want);
    // A SKEWED FIVE-WAY, spread (mouths on a 15 m circle): A east, Q north-east, P from the north mouth heading off at 200
    // degrees (south of west: it leaves the junction's area at about 155 degrees, between Q and C), C west, D south. By
    // heading P would come after C; the ring puts it where its road leaves: A, Q, P, C, D.
    const Vec2 pDir(std::cos(200.0 * 3.14159265358979 / 180.0), std::sin(200.0 * 3.14159265358979 / 180.0));
    const nlohmann::json fiveWay = ringScene(
        {{"A", Vec2(15, 0), Vec2(100, 0)}, {"Q", Vec2(10.61, 10.61), Vec2(80, 80)}, {"P", Vec2(0, 15), Vec2(0, 15) + pDir * 30.0},
         {"C", Vec2(-15, 0), Vec2(-100, 0)}, {"D", Vec2(0, -15), Vec2(0, -100)}},
        {{"D", 0.0, false}, {"C", 0.0, false}, {"A", 0.0, false}, {"P", 0.0, false}, {"Q", 0.0, false}});
    CHECK(ringRoads(fiveWay) == std::vector<std::string>({"A", "Q", "P", "C", "D"}));
    // A ROAD THAT NEVER LEAVES THE CIRCLE (15 m mouths, so a 20 m circle): a three-way whose south road C is 3 m long,
    // measured by its far end (0, -18) -- south, after A east and B west -- and said
    notes.clear();
    const nlohmann::json shortArm = ringScene({{"A", Vec2(15, 0), Vec2(100, 0)}, {"B", Vec2(-15, 0), Vec2(-100, 0)}, {"C", Vec2(0, -15), Vec2(0, -18)}},
                                              {{"C", 0.0, false}, {"B", 0.0, false}, {"A", 0.0, false}});
    CHECK(ringRoads(shortArm, &notes) == std::vector<std::string>({"A", "B", "C"}));
    CHECK(notes.size() == 1u && notes[0].find("(C) never leaves the circle of 20") != std::string::npos);

    // THE TURNS READ THE RING, lane by lane (one lane each way; arm k's arriving lane 100 k, leaving 100 k + 50; a one-way
    // arm only one of them), in this world (kerb side counter-clockwise) and the mirrored one
    using TurnSet3 = std::set<std::tuple<uint32_t, uint32_t, Turn>>;
    const Turn S = Turn::Straight, N = Turn::NearTurn, F = Turn::FarTurn;
    {
        // the divided crossing in ring order: east 0, northbound_n 1 (leaving), southbound_n 2 (arriving), west 3,
        // southbound_s 4 (leaving), northbound_s 5 (arriving); through: east-west, the northbound pair, the southbound pair
        std::vector<TurnArm> arms(6);
        for (uint32_t a = 0; a < 6; ++a) {
            arms[a].hasHeading = true;
            const bool arriving = a != 1 && a != 4, leaving = a != 2 && a != 5;
            if (arriving) arms[a].arriving.push_back(TurnArmLane{100 * a, "through", 0});
            if (leaving) arms[a].leaving.push_back(TurnArmLane{100 * a + 50, "through", 0});
        }
        const std::vector<std::array<uint32_t, 2>> through = {{0, 3}, {5, 1}, {2, 4}};
        auto from = [&](uint32_t arm, bool kerbCcw) {
            TurnSet3 out;
            for (const StatedTurn& t : turnsAtJunction(arms, through, "signal", laneTurnRules(), nullptr, kerbCcw))
                if (t.fromArm == arm) out.insert(std::make_tuple(t.from, t.to, t.turn));
            return out;
        };
        // from the west: straight on east; the near turn south (the southbound carriageway, counter-clockwise before
        // east); the far turn north (the northbound carriageway)
        CHECK(from(3, true) == (TurnSet3{{300, 50, S}, {300, 450, N}, {300, 150, F}}));
        CHECK(from(3, false) == (TurnSet3{{300, 50, S}, {300, 450, F}, {300, 150, N}}));
        // from the south on the northbound carriageway: straight on north; near east; far west, and far onto the
        // southbound carriageway going back south (the U-turn across the median: the table states it as a far turn)
        CHECK(from(5, true) == (TurnSet3{{500, 150, S}, {500, 50, N}, {500, 350, F}, {500, 450, F}}));
        CHECK(from(5, false) == (TurnSet3{{500, 150, S}, {500, 50, F}, {500, 350, N}, {500, 450, N}}));
    }
    {
        // the five-way in ring order A 0, Q 1, P 2, C 3, D 4, A and C a through pair, each arm with its stated heading (P's
        // 200 degrees, past C's 180): from A, straight on C, near Q and P (counter-clockwise before C), far D -- by heading
        // P would have been a far turn
        std::vector<TurnArm> arms(5);
        const double headings[5] = {0, 45, 200, 180, 270};
        for (uint32_t a = 0; a < 5; ++a) {
            arms[a].hasHeading = true;
            arms[a].heading = headings[a];
            arms[a].arriving.push_back(TurnArmLane{100 * a, "through", 0});
            arms[a].leaving.push_back(TurnArmLane{100 * a + 50, "through", 0});
        }
        TurnSet3 fromA, fromAMirrored;
        for (const StatedTurn& t : turnsAtJunction(arms, {{0, 3}}, "signal", laneTurnRules(), nullptr, true))
            if (t.fromArm == 0) fromA.insert(std::make_tuple(t.from, t.to, t.turn));
        for (const StatedTurn& t : turnsAtJunction(arms, {{0, 3}}, "signal", laneTurnRules(), nullptr, false))
            if (t.fromArm == 0) fromAMirrored.insert(std::make_tuple(t.from, t.to, t.turn));
        CHECK(fromA == (TurnSet3{{0, 350, S}, {0, 150, N}, {0, 250, N}, {0, 450, F}}));
        CHECK(fromAMirrored == (TurnSet3{{0, 350, S}, {0, 150, F}, {0, 250, F}, {0, 450, N}}));
    }

    // THE MARK (ADR-0427). hill_junction with the lane level's facts, its layout derived: written in ring order and marked
    // "arms_in_ring", the measure agreeing
    const nlohmann::json laid = lane_level_test::withLaneLevelFacts(nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/lanelab/hill_junction.json")));
    CHECK(laid.value("arms_in_ring", false));
    CHECK(checkArmRings(RoadLabGraph::fromJson(laid)).differ.empty());
    auto said = [](const RoadModel& m) {
        bool s = false;
        for (const std::string& p : m.problems) s = s || (p.find("\"arms_in_ring\"") != std::string::npos && p.find("--arm-order --write") != std::string::npos);
        return s;
    };
    {
        // A FILE NOT MARKED: the model says so, naming the command, and the lane level is not built
        nlohmann::json g = laid;
        g.erase("arms_in_ring");
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        const RoadModel m = assembleRoadModel(*r);
        CHECK(!m.armsInRing);
        CHECK(said(m));
        const LaneGraphBuild lg = buildLaneLevel(*r, m, oneCar());
        std::printf("      %s\n", lg.error.c_str());
        CHECK(lg.error.find("--arm-order --write") != std::string::npos && lg.graph.paths.empty());
    }
    {
        // A MARKED FILE IS TRUSTED, never measured or re-sorted at load: its one junction's arms listed backwards (the
        // through pairs following) load as listed, with no problem said and the lane level built; only the check (what
        // `lanes_tool layout --arm-order` prints) names the junction
        nlohmann::json g = laid;
        nlohmann::json& J = g["junctions"][0];
        const std::size_t n = J["arms"].size();
        nlohmann::json reversed = nlohmann::json::array();
        for (std::size_t k = n; k-- > 0;) reversed.push_back(J["arms"][k]);
        J["arms"] = reversed;
        for (nlohmann::json& t : J["through"])
            for (nlohmann::json& a : t) a = static_cast<int>(n - 1 - a.get<std::size_t>());
        CHECK(n >= 3u);
        CHECK(checkArmRings(RoadLabGraph::fromJson(g)).differ == std::vector<std::string>({J["id"].get<std::string>()}));
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        const RoadModel m = assembleRoadModel(*r);
        CHECK(m.armsInRing);
        CHECK(!said(m));
        CHECK(m.junctions[0].arms.size() == n && m.junctions[0].arms[0].road == reversed[0]["road"].get<std::string>());
        const LaneGraphBuild lg = buildLaneLevel(*r, m, oneCar());
        CHECK(lg.error.empty());
    }
}

// ---- 18. the shipped road files state their arms in their ring (ADR-0427) ----------------------------------------------
// Each shipped road file that states junctions and that the lane level reads, read raw (as the loader reads it, nothing put
// in order first): marked "arms_in_ring", and every junction's stated ring the one measure's (the check `lanes_tool layout
// <file> --arm-order` prints) but for the junctions written here as allowed to differ (a planner's stated ring the measure
// does not give; none today). Then the check is seen to bite: one junction of a copy put out of order is named.
TEST_CASE(road_model_18_the_shipped_road_files_state_their_arms_in_their_ring) {
    const std::vector<std::pair<std::string, std::vector<std::string>>> files = {
        {"planned/traffic_lab_lanelab.json", {}}, {"planned/river_town_lanelab.json", {}},
        {"planned/agent_lab_lanelab.json", {}},   {"ring_city.json", {}}};
    for (const std::pair<std::string, std::vector<std::string>>& f : files) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/" + f.first;
        const RoadLabGraph g = RoadLabGraph::fromJson(nlohmann::json::parse(slurp(path)), path.substr(0, path.find_last_of('/')));
        const ArmRingCheck c = checkArmRings(g);
        std::printf("      %s: %s, %zu junctions, %zu differ from the measure, %zu notes\n", f.first.c_str(),
                    g.armsInRing ? "marked" : "NOT marked", g.junctions.size(), c.differ.size(), c.notes.size());
        for (const std::string& id : c.differ) std::printf("        differs: %s\n", id.c_str());
        CHECK(g.armsInRing);
        CHECK(!g.junctions.empty());
        CHECK(c.differ == f.second);
    }
    // the bite: river_town's j129 (the boulevard's end, four arms) with its first two arms swapped, the through pairs
    // following -- a copy in memory, the file untouched
    const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/river_town_lanelab.json";
    nlohmann::json spec = nlohmann::json::parse(slurp(path));
    bool swapped = false;
    for (nlohmann::json& J : spec["junctions"]) {
        if (J["id"] != "j129") continue;
        std::swap(J["arms"][0], J["arms"][1]);
        for (nlohmann::json& t : J["through"])
            for (nlohmann::json& a : t) a = a.get<int>() == 0 ? 1 : a.get<int>() == 1 ? 0 : a.get<int>();
        swapped = true;
    }
    CHECK(swapped);
    const ArmRingCheck c = checkArmRings(RoadLabGraph::fromJson(spec, path.substr(0, path.find_last_of('/'))));
    CHECK(c.differ == std::vector<std::string>({"j129"}));
}
