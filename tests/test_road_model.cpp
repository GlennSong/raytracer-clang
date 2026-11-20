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
//   7. kerb parking is the road file's statement (ADR-0418): the twin's band, the model's section, and a statement that
//      cannot be built named.

#include "test_framework.h"
#include "planned_city.h"

#include "apps/citysim/brain/brain_driving.h"                 // RoadPlaces: the kerb spaces as the agent brain's places

#include "engine/bundle/bake.h"
#include "engine/procgen/city/plan/plan_scene.h"
#include "engine/procgen/city/roads/lanes/control_plan.h"     // findJunctions: the one grouping
#include "engine/procgen/city/roads/lanes/deck_mesh.h"        // kStopLineBack
#include "engine/procgen/city/roads/lanes/road_model_cache.h"   // levelRoadsKey: the city's bake key
#include "engine/procgen/city/roads/lanes/lane_expand.h"   // expand: the planned layer reads the ramps composed
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/walk_lanes.h"   // kWalkFromKerb: how far in from the kerb a corner walk runs
#include "engine/procgen/city/roads/lanes/lots_producer.h"   // lotRoadInputs: the roads as the lot pass reads them (ADR-0439)
#include "engine/procgen/city/city_lots.h"                   // growLotBuildings: the lot pass faces its doors
#include "engine/level_params.h"                             // readLotGrowParams
#include "engine/procgen/city/roads/lanes/road_layout.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"   // smoothTangentAtStation (ADR-0473)
#include "engine/procgen/city/road_network.h"               // buildingMayFront
#include "engine/procgen/city/roads/road_entity.h"      // navRoadGraph
#include "engine/procgen/city/roads/road_model.h"
#include "engine/procgen/city/roads/lane_turns.h"           // the stated turns (ADR-0426)
#include "engine/procgen/city/roads/lanes/lane_level.h"     // the lane level from the model (ADR-0426)
#include "lane_level_scene.h"                               // a scene with the lane level's facts, in memory
#include "engine/procgen/city/roads/control_signs.h"   // the signs that follow from the facts (ADR-0424)
#include "engine/procgen/city/roads/route.h"   // the one route-finder: cars and walkers (ADR-0467)
#include "engine/procgen/city/roads/street_map.h"      // appendStreetMap: what routing reads
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
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <random>
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
std::vector<ArmKey> armsOf(const ModelJunction& j) {
    std::vector<ArmKey> out;
    for (const ModelArm& a : j.arms) out.emplace_back(a.road, a.station, a.spineIn);
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
        x.model = assembleRoadModel(*x.r, LaneGraphOptions());
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
    for (const ModelJunction& pj : g.junctions) {
        CHECK(!pj.control.empty());
        const std::map<std::string, std::string>::const_iterator row = controlOfRow.find(pj.why);
        const bool byRow = row != controlOfRow.end() && row->second == pj.control;
        CHECK(byRow);
        decided += byRow ? 1u : 0u;
        ++byControl[pj.control];
        // a junction whose row stands signs carries them on some arm
        bool signs = false;
        for (const ModelArm& a : pj.arms) signs = signs || !a.regulations.empty();
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
    // THE SOLID STRETCH BEFORE A STOP LINE HAS ONE HOME (ADR-0435): the planner states in the road file the number its
    // ramps are laid by, and the lane level reads the file. As planned: 20 m. Planned for 35 m: the file says 35. (Before,
    // the planner laid its ramps by a constant of 20 m and wrote nothing, so a file could state any other number.)
    CHECK(g.statesLaneRules);
    CHECK(scene.contains("lane_rules") && scene["lane_rules"].value("solid_before_stop_line", -1.0) == 20.0);
    CHECK(g.laneRules.solidBeforeStopLine == 20.0);
    engine::plan::SceneOptions longer;
    longer.solidBeforeStopLine = 35.0;
    const RoadLabGraph g35 = RoadLabGraph::fromJson(engine::plan::planToLanesScene(engine::plan::generatePlan(citytest::roundBrief(11)), longer));
    CHECK(g35.statesLaneRules && g35.laneRules.solidBeforeStopLine == 35.0);
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
    for (const ModelJunction& j : tool.junctions) toolByArms[armsOf(j)] = j.id;
    std::map<std::string, std::string> idOf;   // planner id -> tool id
    int unmatched = 0;
    for (const ModelJunction& j : planned.junctions) {
        const std::map<std::vector<ArmKey>, std::string>::const_iterator it = toolByArms.find(armsOf(j));
        if (it == toolByArms.end()) {
            if (++unmatched <= 5) {
                std::printf("    planner junction %s at (%.1f, %.1f) with %zu arms: the tool has no junction with those arms\n", j.id.c_str(), j.at.x, j.at.y, j.arms.size());
                auto show = [](const ModelJunction& x) {
                    std::string s;
                    for (const ModelArm& a : x.arms) s += " " + a.road + "@" + std::to_string(a.station) + (a.spineIn ? "in" : "out");
                    return s;
                };
                std::printf("      planner:%s\n", show(j).c_str());
                for (const ModelJunction& t : tool.junctions)
                    if (distance(t.at, j.at) < 40) std::printf("      tool %s at (%.1f, %.1f):%s\n", t.id.c_str(), t.at.x, t.at.y, show(t).c_str());
            }
            continue;
        }
        idOf[j.id] = it->second;
    }
    CHECK(unmatched == 0);
    // street for street: road, stations, and the junction at each end (as mapped)
    std::set<std::tuple<std::string, double, double, std::string, std::string>> toolStreets, plannerStreets;
    // (the id of the junction a street names at an end; empty: none)
    auto endId = [](const std::vector<ModelJunction>& list, engine::ModelJunctionId j) { return j.valid() ? list[j.index].id : std::string(); };
    for (const ModelStreet& s : tool.streets)
        toolStreets.emplace(s.road, s.from, s.to, endId(tool.junctions, s.junctions[0]), endId(tool.junctions, s.junctions[1]));
    for (const ModelStreet& s : planned.streets)
        plannerStreets.emplace(s.road, s.from, s.to, s.junctions[0].valid() ? idOf[endId(planned.junctions, s.junctions[0])] : "",
                               s.junctions[1].valid() ? idOf[endId(planned.junctions, s.junctions[1])] : "");
    CHECK(toolStreets == plannerStreets);
    // the planner's arms each name a street of its list, and the ids the planner gives are written in its own scene
    int armsWithoutStreet = 0;
    for (const ModelJunction& j : planned.junctions)
        for (const ModelArm& a : j.arms) armsWithoutStreet += a.street.valid() ? 0 : 1;
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

// ---- 2. the bake sees where the stated arms stand and what the paint reads of the layout -----------------------------------------------

TEST_CASE(road_model_2_the_bake_key_sees_where_the_arms_stand_and_what_the_paint_reads) {
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
        for (const ModelJunction& j : layout.junctions) {
            nlohmann::json jj = {{"id", j.id}, {"at", {j.at.x, j.at.y}}, {"control", "stop"}};
            for (const ModelArm& a : j.arms) jj["arms"].push_back({{"road", a.road}, {"at", a.station}, {"spine", a.spineIn ? "in" : "out"}});
            for (const ModelStretch& st : j.inside) jj["inside"].push_back({{"road", st.road}, {"from", st.from}, {"to", st.to}});
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
        for (const ModelJunction& j : back.junctions)
            for (const ModelArm& a : j.arms) streets = streets && a.street.valid() && engine::stated(a.heading);
        CHECK(streets);
    }
    // the city's bake key, through the producer. ADR-0428: the paint draws the stated junctions, so where their arms stand
    // is keyed. ADR-0431: the paint's lane lines, stop bars and arrows are read off the road model, so what they read of
    // the layout is keyed too (deck_mesh.h paintFactsKey): the streets, the arms' streets and headings, the through
    // pairs, each control. So a file stating its junctions' arms (the control plan's list) keys OTHERWISE with the layout
    // written into it, and otherwise again with another control; a street's name or a junction's `why` keys nothing;
    // moving one arm a metre, or a road fewer, keys otherwise; and a file stating no junctions keys otherwise than one
    // stating them
    nlohmann::json planned = bare;
    for (const ModelJunction& j : layout.junctions) {
        nlohmann::json jj = {{"id", j.id}, {"at", {j.at.x, j.at.y}}, {"control", "stop"}};
        for (const ModelArm& a : j.arms) jj["arms"].push_back({{"road", a.road}, {"at", a.station}, {"spine", a.spineIn ? "in" : "out"}});
        for (const ModelStretch& st : j.inside) jj["inside"].push_back({{"road", st.road}, {"from", st.from}, {"to", st.to}});
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
    CHECK(stated != 0 && stated != laidOut);
    {
        const std::string laidOutText = insertLayoutText(planned.dump(1), kept, &err);
        nlohmann::json named = nlohmann::json::parse(laidOutText);
        CHECK(named.contains("streets") && !named["streets"].empty());
        if (named.contains("streets") && !named["streets"].empty()) named["streets"][0]["name"] = "Oak Street";
        named["junctions"][0]["why"] = "written by a test";
        CHECK(keyOf(named.dump(1)) == keyOf(nlohmann::json::parse(laidOutText).dump(1)));   // a name and a `why` paint nothing
        nlohmann::json lit = nlohmann::json::parse(laidOutText);
        lit["junctions"][0]["control"] = "signal";
        CHECK(keyOf(lit.dump(1)) != keyOf(nlohmann::json::parse(laidOutText).dump(1)));     // a control paints stop bars
    }
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
    // every lane on a road with streets knows them (the model's lane holds them: the one place), and through lanes their
    // slot from the centre line
    int lanesOnStreets = 0, wrongSlot = 0;
    CHECK(m.lanes.size() == r.lanes.lanes.size());
    for (const ModelLane& l : m.lanes) {
        if (l.edge == kNoIndex) continue;
        lanesOnStreets += l.streets.empty() ? 0 : 1;
        if (l.kind == "through") wrongSlot += l.slot == std::atoi(l.id.c_str() + l.id.rfind('.') + 2) ? 0 : 1;
    }
    CHECK(lanesOnStreets == static_cast<int>(std::count_if(r.lanes.lanes.begin(), r.lanes.lanes.end(), [](const Lane& l) { return l.road() >= 0; })));
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
    // 5.2 m further: 11.2 m and 16.2 m. The numbers are written out here, worked from the scene by hand: the rule itself
    // (road_model_parts.h boxEdgeBack / stopLineBack) is what is under test, so the test neither calls it nor adds its
    // constant.
    const ModelJunction& J = m.junctions[cross.index];
    CHECK(J.arms.size() == 4u);
    for (const ModelArm& a : J.arms) {
        CHECK(stated(a.stopLine) && stated(a.boxEdge));
        const bool arterial = a.road[0] == 'a';
        const Real boxBack = arterial ? 6.0 : 11.0, stopBack = arterial ? 11.2 : 16.2;
        std::printf("      arm %s: mouth %.2f, box edge %.2f, stop line %.2f\n", a.road.c_str(), a.station, a.boxEdge, a.stopLine);
        const Real away = a.spineIn ? -1.0 : 1.0;
        CHECK(approxEqual(a.boxEdge, a.station + away * boxBack, kWrittenStation));
        CHECK(approxEqual(a.stopLine, a.station + away * stopBack, kWrittenStation));
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
    const RoadModel m = assembleRoadModel(*r, LaneGraphOptions());
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

// WHERE A RAMP'S LANE MEETS ITS HOST LANE IS THE BUILDER'S RECORD (issue #292): the builder lays the auxiliary lane and its
// taper to a station on the host and records it; the model's connection copies it. The scene: merge_taper's one-lane
// ramp, its taper restated as 108.4 m: 120 + 108.4 = 228.4 m past the gore at x = 330. The builder lays the whole stated
// length (lane_expand.h rampEndDistances: a sample each metre and one at the length itself), so its lane ends at
// x = 558.4, station 708.4 of a mainline that starts at x = -150 along y = 0, on the kerb lane of its three 3.6 m lanes
// (3.6 m off the centre line). The model says 708.4, where the lane is drawn. (The builder once laid whole metres only
// and stopped at 708.0, 0.4 m before the stated meeting; the model once worked 708.4 out again where no lane ended.)
TEST_CASE(road_model_3c_a_ramp_lane_is_drawn_to_where_the_model_says_it_meets_its_host) {
    // the distances an end is laid at: whole metres, then the length; a whole length gets no extra step
    const std::vector<double> part = rampEndDistances(2.25), whole = rampEndDistances(3.0);
    CHECK(part.size() == 4u && part[2] == 2.0 && part[3] == 2.25);
    CHECK(whole.size() == 4u && whole[3] == 3.0);
    nlohmann::json g = nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/lanelab/merge_taper.json"));
    g.erase("junctions");
    g.erase("streets");
    for (nlohmann::json& e : g["edges"])
        if (e["id"] == "ramp_one") e["to"]["taper"] = 108.4;
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    const RoadModel m = assembleRoadModel(*r, LaneGraphOptions());
    // the builder's record, and the ramp's spine drawn to it, on the kerb lane's line
    const EdgeSpec* ramp = r->graph.find("ramp_one");
    CHECK(ramp != nullptr);
    if (!ramp) return;
    CHECK(approxEqual(ramp->anchorMeets[1], 708.4, kSamePoint));
    CHECK(approxEqual(ramp->xy.back().x, 558.4, kSamePoint));
    CHECK(approxEqual(std::fabs(ramp->xy.back().y), 3.6, kSamePoint));
    // the model's connection says the same station
    int found = 0;
    for (const ModelConnection& c : m.connections) {
        if (c.kind != ConnectionKind::Merge || c.ramp != "ramp_one") continue;
        ++found;
        CHECK(c.length == 120.0 && c.taper == 108.4);
        CHECK(approxEqual(c.hostStation, 480.0, kWrittenStation));
        CHECK(approxEqual(c.meets, 708.4, kSamePoint));
    }
    CHECK(found == 1);
    // a ramp whose file states no taper gets the default once, where the file is read: 30 of its host's 3.6 m lanes
    for (nlohmann::json& e : g["edges"])
        if (e["id"] == "ramp_one") e["to"].erase("taper");
    const RoadLabGraph plain = RoadLabGraph::fromJson(g);
    CHECK(approxEqual(plain.find("ramp_one")->to.taper, 108.0, kSamePoint));
}

// THE POINTS OF A LINE INSIDE A STRETCH ARE PICKED BY ONE FUNCTION (issue #292; polyline.h stretchAtStations): the road
// model's centre lines, the street map's runs and the lane level's slices all call it. A line along x with points at
// stations 0, 5, 9.996 and 10, and the stretch a file wrote as 2.5 to 10: the point 4 mm before the end IS the end, so the
// stretch is (2.5, 0), (5, 0), (10, 0). (The model's own copy asked with a millionth of a metre and kept the 4 mm point
// as a fourth; the street map's asked with the written-station tolerance and did not.)
TEST_CASE(road_model_3d_one_function_picks_a_lines_points_inside_a_stretch) {
    const std::vector<Vec2> xy = {Vec2(0, 0), Vec2(5, 0), Vec2(9.996, 0), Vec2(10, 0)};
    const std::vector<Real> s = {0.0, 5.0, 9.996, 10.0};
    std::vector<Vec2> pts;
    std::vector<Real> st;
    stretchAtStations(xy.data(), s.data(), xy.size(), 2.5, 10.0, kWrittenStation, pts, &st);
    CHECK(pts.size() == 3u && st.size() == 3u);
    if (pts.size() == 3u) {
        CHECK(approxEqual(pts[0].x, 2.5, kSamePoint) && approxEqual(pts[1].x, 5.0, kSamePoint) && approxEqual(pts[2].x, 10.0, kSamePoint));
        CHECK(st[0] == 2.5 && st[1] == 5.0 && st[2] == 10.0);
    }
    // a stretch worked out, not written, asks with the millimetre: the same point is then inside it
    std::vector<Vec2> fine;
    stretchAtStations(xy.data(), s.data(), xy.size(), 2.5, 10.0, kSamePoint, fine);
    CHECK(fine.size() == 4u);
    // the lane level's slice is the same function, its ends kept within the line
    const std::vector<Vec2> slice = sliceAtStations(xy, s, -3.0, 20.0, kWrittenStation);
    CHECK(slice.size() == 3u);
    if (slice.size() == 3u) CHECK(approxEqual(slice[0].x, 0.0, kSamePoint) && approxEqual(slice[1].x, 5.0, kSamePoint) && approxEqual(slice[2].x, 10.0, kSamePoint));
}

// ---- 4. the questions ------------------------------------------------------------------------------------------------------

TEST_CASE(road_model_4_the_questions) {
    const RoadModel& m = smallBuilt().model;
    const ModelJunctionId cross = m.streets[streetNamed(m, "a1").index].junctions[0];
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
    // what is attached (ADR-0420): each named street carries its name
    const std::vector<uint32_t> onO0 = attachedToStreet(m, streetNamed(m, "o0"));
    CHECK(onO0.size() == 1u);
    if (!onO0.empty()) CHECK(m.attachments[onO0.front()].text == streetName(m, streetNamed(m, "o0")) && !m.attachments[onO0.front()].text.empty());
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
    std::vector<ModelStreet> streets = stated.streets;
    nameStreetRuns(streets, stated.junctions, stated.divided, stated, streetNameBook());
    const std::string named = insertStreetNamesText(text, streets, &err);
    CHECK(!named.empty() && stripLayoutText(named) == stripLayoutText(text));
    const RoadLabGraph reread = RoadLabGraph::fromJson(nlohmann::json::parse(named));
    for (std::size_t s = 0; s < streets.size(); ++s) CHECK(reread.streets[s].name == streets[s].name);
    // named again, every name the file states is kept
    std::vector<ModelStreet> again = reread.streets;
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

// ---- 7. kerb parking is stated (ADR-0418) --------------------------------------------------------------------------------

namespace {
// The kerb parking bands the road model's streets carry (their cross-sections' `parking`: what the street map's links
// and the sim's bays read), by the street's class.
std::map<engine::RoadClass, std::pair<int, double>> streetBands(const RoadModel& m) {
    std::map<engine::RoadClass, std::pair<int, double>> out;   // class -> (streets with a band, the widest band)
    for (const ModelStreet& s : m.streets) {
        if (s.section.parking <= 0) continue;
        std::pair<int, double>& o = out[s.section.klass];
        ++o.first;
        o.second = std::max(o.second, static_cast<double>(s.section.parking));
    }
    return out;
}
std::map<engine::RoadClass, std::pair<int, double>> streetBands(const Result& r) { return streetBands(assembleRoadModel(r, LaneGraphOptions())); }
}  // namespace

TEST_CASE(road_model_7_kerb_parking_is_the_road_file_s_statement) {
    // the class table: a local street parks along both kerbs, 2.5 m over its 6 m lane; no other class parks
    CHECK(roadClass("local").lanes.parking == 2.5);
    for (const std::pair<const std::string, RoadClassSpec>& c : roadClasses())
        if (c.first != "local") CHECK(c.second.lanes.parking == 0.0);
    // the small graph (the table's local and arterial, and a one-way class of its own): the streets' bands are the
    // locals' stated 2.5 m and nothing else's
    {
        const Built& b = smallBuilt();
        const std::map<engine::RoadClass, std::pair<int, double>> bands = streetBands(b.model);
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
        CHECK(streetBands(*r).empty());
        for (const ModelStreet& s : assembleRoadModel(*r, LaneGraphOptions()).streets) CHECK(s.section.parking == 0.0);
    }
    // ...or that it parks 2.0 m: the band is 2.0 m
    {
        nlohmann::json g = smallGraphWithLayout();
        g["classes"]["local"] = {{"parking", 2.0}};
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        const std::map<engine::RoadClass, std::pair<int, double>> bands = streetBands(*r);
        CHECK(bands.count(engine::RoadClass::Local) == 1u);
        if (bands.count(engine::RoadClass::Local)) CHECK_APPROX(bands.at(engine::RoadClass::Local).second, 2.0, 1e-9);
    }
    // a local of 3.5 m lanes cannot hold the table's 2.5 m and a travel lane: no band, and the model says so by name
    {
        nlohmann::json g = smallGraphWithLayout();
        g["classes"]["local"] = {{"w", 3.5}};
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        CHECK(streetBands(*r).empty());
        const RoadModel m = assembleRoadModel(*r, LaneGraphOptions());
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
    // between the two Ts' stop lines along the arterial: `apart` less (6 + 5.2) at each -- the road CROSSING the arterial
    // at each T is the local street ending on it (half width 6 m); the arterial carrying on is not crossing itself. One
    // junction when that is under the storage length (6.5 m, a car and its gap). (Was 11 + 5.2 until issue #292: the
    // grouping's own copy of the stop-line rule counted the arterial's other half as the widest road there, which the
    // stated junctions' areas -- what is painted -- never did.)
    const double reach = 2.0 * (6.0 + kStopLineBack);
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
    if (B.streets.size() == 2u)
        CHECK(B.streets[0].carriesOn[1] == engine::ModelStreetId(1) && B.streets[1].id == "q" && B.streets[1].carriesOn[0] == engine::ModelStreetId(0) &&
              B.streets[0].id == "p");
}

// ---- 9. the signs that follow from the facts (ADR-0424) ----------------------------------------------------------------

namespace {
// The small graph with its one-way street turned to run INTO the T at (200, 0) (its exit end there), its layout written,
// and controls stated by hand: an all-way stop at the crossing (0, 0), the T giving way on its local street.
// `spineAgainstTraffic`: the same street, the same traffic, WRITTEN THE OTHER WAY ROUND -- its spine runs out of the T and
// its one lane is a backward lane, so its traffic still runs into the T.
nlohmann::json signedGraph(bool spineAgainstTraffic = false) {
    nlohmann::json g = smallGraph();
    for (nlohmann::json& e : g["edges"]) {
        if (e["id"] != "o0") continue;
        if (!spineAgainstTraffic) {
            e["path"]["points"] = {{200, 150}, {200, 0}};
        } else {
            e["path"]["points"] = {{200, 0}, {200, 150}};
            e["lanes"] = {{"fwd", 0}, {"back", 1}};
        }
    }
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

    // WHICH DEVICE PAINTS WHAT is this file's `marking` rows alone (issue #292: the rule set once listed a `paint` per
    // device too, read by nothing, and the two disagreed -- it gave a give-way arm "give_way_teeth", which no row draws,
    // and knew nothing of the bar at traffic lights). Two rows, by name: the lane arrows (no device's paint) and the stop
    // bar, the paint of the device "stop".
    std::vector<std::string> markProblems;
    const std::vector<engine::ArmMarkingRule> marks = engine::armMarkingRules(attachmentKinds(), &markProblems);
    CHECK(markProblems.empty());
    CHECK(marks.size() == 2u);
    if (marks.size() != 2u) return;
    CHECK(marks[0].kind == "lane_arrows" && marks[0].draws == "turn_arrows" && marks[0].means.empty());
    CHECK(marks[1].kind == "stop_bar" && marks[1].draws == "bar" && marks[1].means == "stop");
    // THE ARROWS' LAYOUT is that row's (issue #292: it was numbers inside the paint): where two or more lanes arrive, a
    // tail 12 m and one 35 m before the stop line, the second on a block of 110 m or more, on a road running 40 m back
    CHECK(marks[0].whenAny.size() == 1u && marks[0].whenAny[0].arrivingLanes == 2 && marks[0].whenAny[0].regulation.empty() &&
          marks[0].whenAny[0].controls.empty());
    CHECK(marks[0].beforeStopLine == std::vector<double>({12.0, 35.0}));
    CHECK(marks[0].repeatNeedsBlock == 110.0 && marks[0].needsRoad == 40.0);
#ifdef RT_ENABLE_SCRIPTING
    {
        // ...the device it means is the rule set's, one a stop line goes with, and the row is painted where an arm states it
        engine::ScriptVM vm;
        driving::RuleSet rs;
        std::string err;
        const std::string code = engine::loadScriptCode(citysim::kDefaultRuleSet, "");
        CHECK(!code.empty() && vm.doString(code, &err) && driving::loadRuleSet(vm, citysim::kDefaultRuleSet, rs, &err));
        for (const engine::ArmMarkingRule& r : marks) {
            if (r.means.empty()) continue;
            const driving::Device* d = rs.findDevice(r.means);
            CHECK(d != nullptr && d->stopLine);
            bool whereStated = false;
            for (const engine::ArmMarkingRule::When& w : r.whenAny) whereStated = whereStated || w.regulation == r.means;
            CHECK(whereStated);
        }
    }
#endif

    const RoadModel m = plannedRoadModel(RoadLabGraph::fromJson(signedGraph()));
    {
        // on the signed scene: a bar across every arm stated to stop, none across the arm stated to give way (the rule
        // set's old list would have painted teeth there); asked with one lane arriving, so no arrows row holds...
        int stopArms = 0, giveWayArms = 0;
        for (const ModelJunction& J : m.junctions)
            for (std::size_t a = 0; a < J.arms.size(); ++a) {
                const std::vector<const engine::ArmMarkingRule*> on = engine::markingsOnArm(J, a, marks, 1);
                // ...and with two arriving, the arrows row holds as well, at every arm whatever it states
                const std::vector<const engine::ArmMarkingRule*> two = engine::markingsOnArm(J, a, marks, 2);
                CHECK(two.size() == on.size() + 1u && two.front()->kind == "lane_arrows");
                const std::vector<std::string>& regs = J.arms[a].regulations;
                if (std::find(regs.begin(), regs.end(), "stop") != regs.end()) {
                    ++stopArms;
                    CHECK(on.size() == 1u && on.front()->kind == "stop_bar");
                }
                if (std::find(regs.begin(), regs.end(), "give_way") != regs.end()) {
                    ++giveWayArms;
                    CHECK(on.empty());
                }
            }
        CHECK(stopArms == 4 && giveWayArms == 1);
    }
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
        CHECK(std::fabs(dot(off, rightOf(d))) > S.section.carriagewayHalfWidth() + 0.5);
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
            CHECK(std::fabs(dot(off, d) - (std::fabs(a.stopLine - a.station) - 0.6)) < 1e-6);
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

// ONE-WAY IS ONE-WAY WHICHEVER WAY THE SPINE IS WRITTEN, AND A STREET HAS ONE WIDTH (ADR-0432). The signed graph's one-way
// street o0 written the other way round (its spine out of the T at (200, 0), its one lane a backward lane) is the same
// street with the same traffic. The model stated one-way only for forward lanes, so the signs took this street for
// two-way (no DO NOT ENTER, no ONE WAY) while routing took it for one-way.
TEST_CASE(road_model_9b_one_way_is_asked_of_the_cross_section_whichever_way_its_lanes_run) {
    const std::vector<engine::ControlSignRule> rules = engine::controlSignRules(attachmentKinds());
    const RoadModel written = plannedRoadModel(RoadLabGraph::fromJson(signedGraph()));
    const RoadModel turned = plannedRoadModel(RoadLabGraph::fromJson(signedGraph(true)));
    const ModelStreetId o0 = streetNamed(turned, "o0");
    CHECK(o0.valid());
    const ModelSection& sec = turned.streets[o0.index].section;
    CHECK(sec.lanesForward == 0 && sec.lanesBackward == 1);
    CHECK(sec.oneWay());
    CHECK(sec.lanesArriving(false) == 1 && sec.lanesArriving(true) == 0);   // its traffic arrives where the spine runs OUT
    CHECK_APPROX(sec.travelWidth(), 3.5, 1e-12);
    // THE SIGNS: the same signs on the same spots, facing the same way, with the same arrows
    const engine::ControlSignPlan a = engine::placeControlSigns(written, rules, nullptr, {});
    const engine::ControlSignPlan b = engine::placeControlSigns(turned, rules, nullptr, {});
    CHECK(b.placed.count("do_not_enter_sign") && b.placed.at("do_not_enter_sign") == 1);
    CHECK(b.placed.count("one_way_sign") && b.placed.at("one_way_sign") == 2);
    CHECK(a.signs.size() == b.signs.size());
    for (const engine::PlacedSign& s : a.signs) {
        bool same = false;
        for (const engine::PlacedSign& t : b.signs)
            same = same || (t.kind == s.kind && (t.at - s.at).length() < 1e-6 && (t.face - s.face).length() < 1e-9 &&
                            t.look.value("arrow", std::string()) == s.look.value("arrow", std::string()));
        CHECK(same);
    }
    for (const engine::PlacedSign& s : b.signs) {
        if (s.kind != "do_not_enter_sign") continue;
        const ModelArm& arm = turned.junctions[static_cast<std::size_t>(s.junction)].arms[static_cast<std::size_t>(s.arm)];
        CHECK(arm.road == "o0" && !arm.spineIn);   // at the T, its exit end: where its spine runs OUT of the junction
    }
    // ROUTING: one link, one way, from its far end (200, 150) into the T
    engine::NavGraph map;
    engine::StreetMapReport rep;
    CHECK(engine::appendStreetMap(map, turned, 0, nullptr, &rep));
    int along = 0;
    for (const engine::NavLink& L : map.links) {
        if (L.street != static_cast<int>(o0.index)) continue;
        ++along;
        CHECK(L.oneWay);
        CHECK_APPROX(L.width, 3.5, 1e-12);
        CHECK(map.nodes[static_cast<std::size_t>(L.from)].y > map.nodes[static_cast<std::size_t>(L.to)].y);   // toward the T at y = 0
    }
    CHECK(along >= 1);

    // ONE WIDTH. A road with two lanes forward and one back, 3.5 m lanes and a 1 m median gap: 3 x 3.5 + 1 = 11.5 m across
    // (the street map said 1 + 2 x 2 x 3.5 = 15). A one-way road that states a gap has no median to put it in: 2 x 3.5 = 7
    // (the model said 11). The builder lays exactly these (half widths 5.75 and 3.5).
    nlohmann::json g = smallGraph();
    for (nlohmann::json& e : g["edges"]) {
        if (e["id"] == "l2") e["lanes"] = {{"w", 3.5}, {"fwd", 2}, {"back", 1}, {"gap", 1.0}};
        if (e["id"] == "o0") e["lanes"] = {{"w", 3.5}, {"fwd", 2}, {"back", 0}, {"gap", 4.0}};
    }
    {
        const RoadLabGraph graph = RoadLabGraph::fromJson(g);
        RoadLayout layout = deriveLayout(graph, LayoutSource::Inferred);
        writeLayout(g, layout);
    }
    const RoadLabGraph graph = RoadLabGraph::fromJson(g);
    const RoadModel m = plannedRoadModel(graph);
    engine::NavGraph widths;
    CHECK(engine::appendStreetMap(widths, m, 0, nullptr, &rep));
    struct RoadWidth { const char* road; double width; };
    for (const RoadWidth& x : {RoadWidth{"l2", 11.5}, RoadWidth{"o0", 7.0}}) {
        const ModelStreetId s = streetNamed(m, x.road);
        CHECK(s.valid());
        CHECK_APPROX(m.streets[s.index].section.travelWidth(), x.width, 1e-12);
        CHECK_APPROX(graph.hw(*graph.find(x.road)), x.width / 2, 1e-12);
        double outermost = 0;   // the builder's lanes: the farthest lane's middle and half a lane
        for (const LaneSlot& slot : laneSlots(graph.cls(*graph.find(x.road)), graph.find(x.road)->lanes))
            outermost = std::max(outermost, std::fabs(slot.offset) + 1.75);
        CHECK_APPROX(outermost, x.width / 2, 1e-5);
        int links = 0;
        for (const engine::NavLink& L : widths.links) {
            if (L.street != static_cast<int>(s.index)) continue;
            ++links;
            CHECK_APPROX(L.width, x.width, 1e-12);
        }
        CHECK(links >= 1);
    }
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
std::vector<uint32_t> pathsOfLane(const LaneGraph& G, const LaneGraphBuild& lg, uint32_t lane) {
    std::vector<uint32_t> out;
    for (uint32_t p = 0; p < G.paths.size(); ++p)
        if (G.paths[p].kind == PathKind::Lane && G.paths[p].modelLane == lane) out.push_back(p);
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
    LaneGraphBuild lg;   // what building the model's lane level reported
    const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
    const LaneGraph& G = m.laneLevel;
    CHECK(lg.error.empty());
    for (const std::string& p : validate(G)) std::printf("      [lane level] %s\n", p.c_str());
    CHECK(validate(G).empty());
    // THE LANES' CENTRE LINES ARE HELD ONCE (ADR-0438): the model holds the very object the build laid its lanes into,
    // and a builder's lane and its model lane read the same points in memory, not two copies of them
    CHECK(m.curves && m.curves == r->lanes.curves);
    CHECK(r->lanes.lanes.size() == m.lanes.size() && !m.lanes.empty());
    for (std::size_t li = 0; li < m.lanes.size() && li < r->lanes.lanes.size(); ++li) {
        const Lane& built = r->lanes.lanes[li];
        CHECK(built.curve == m.lanes[li].curve && built.xy().size() == m.lanes[li].curve.count);
        CHECK(built.xy().data() == m.lanePoints().data() + m.lanes[li].curve.first);
        CHECK(built.s().data() == m.laneAlong().data() + m.lanes[li].curve.first);
    }
    // (a model read back from its saved bytes holds its own, equal, points)
    {
        bundle::BinWriter w;
        putRoadModel(w, m);
        bundle::BinReader rd(w.bytes.data(), w.bytes.size());
        RoadModel back;
        std::string why;
        CHECK(getRoadModel(rd, back, &why));
        CHECK(back.curves && back.curves != m.curves && back.lanePoints().size() == m.lanePoints().size() && back == m);
    }
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
        CHECK(G.junctions[p.junction.index].modelJunction == t.junction.index);
    }
    // WHAT THE ROAD IS ALONG A PATH IS ASKED OF THE MODEL (ADR-0436, issue #291): the lane table keeps no copy of a
    // junction's control, a path's posted limit or road class, or a connector's turn, so there is one answer, and
    // stating a fact again on the model changes it for every reader without the lane table being built again. (The
    // copies the table used to keep were written when it was built and would still say the old thing.)
    {
        RoadModel stated = m;
        uint32_t cross = kNoIndex, tee = kNoIndex;   // the lane level's junctions at (0, 0) and at (200, 0)
        for (uint32_t j = 0; j < G.junctions.size(); ++j) (m.junctions[G.junctions[j].modelJunction].at.length() < 1.0 ? cross : tee) = j;
        CHECK(cross != kNoIndex && tee != kNoIndex);
        CHECK(stated.controlAt(JunctionId(cross)) == "all_way_stop");
        CHECK(stated.controlAt(JunctionId(tee)) == "give_way");
        CHECK(stated.controlAt(JunctionId(99)).empty());   // no such junction: nothing stated
        // a lane path of the arterial a0 arriving at the crossing, and a connector leading out of it into the local
        // street l1 (a far or near turn off the arterial)
        const std::vector<uint32_t> a0 = pathsOfLane(G, lg, laneNamed(m, "a0.f0").index);
        CHECK(!a0.empty());
        if (a0.empty()) return;
        const PathId arriving(a0.back());
        PathId turning;   // the first connector from it that leads into a lane of l1
        for (uint32_t k = G.paths[arriving.index].next.first; k < G.paths[arriving.index].next.end() && !turning.valid(); ++k) {
            const PathId c = G.nextPaths[k];
            const ModelStreet* into = stated.streetOf(c);
            if (G.paths[c.index].kind == PathKind::Connector && into && into->road == "l1") turning = c;
        }
        CHECK(turning.valid());
        if (!turning.valid()) return;
        CHECK(stated.roadClassOf(arriving) == "arterial");
        CHECK(stated.roadClassOf(turning) == "local");          // a connector's: the lane's it leads into
        CHECK(stated.postedLimit(arriving) == 0.0 && stated.postedLimit(turning) == 0.0);   // the scene posts none
        CHECK(stated.turnOf(arriving) == Turn::None);           // a lane takes no turn
        CHECK(stated.turnOf(turning) != Turn::None && stated.turnOf(turning) != Turn::Straight);
        // stated again, on the model alone: 11 m/s on l1's street, 17 on a0's; lights at the crossing; the turn a U-turn
        const uint32_t l1Street = G.paths[G.nextPaths[G.paths[turning.index].next.first].index].street;
        stated.streets[l1Street].speedLimit = 11.0;
        stated.streets[G.paths[arriving.index].street].speedLimit = 17.0;
        stated.streets[l1Street].section.roadClass = "collector";
        stated.junctions[G.junctions[cross].modelJunction].control = "signal";
        stated.turns[G.paths[turning.index].modelTurn].turn = Turn::UTurn;
        CHECK(stated.postedLimit(arriving) == 17.0);
        CHECK(stated.postedLimit(turning) == 11.0);             // the limit of the lane it leads into
        CHECK(stated.roadClassOf(turning) == "collector");
        CHECK(stated.controlAt(JunctionId(cross)) == "signal");
        CHECK(stated.controlAt(JunctionId(tee)) == "give_way");
        CHECK(stated.turnOf(turning) == Turn::UTurn);
    }
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
        const std::vector<uint32_t> a = pathsOfLane(m.laneLevel, lg,l1.index), b = pathsOfLane(m.laneLevel, lg,l3.index);
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
    LaneGraphBuild none;
    const RoadModel bareModel = assembleRoadModel(*rb, oneCar(), &none);
    CHECK(!none.error.empty() && none.error.find("--lane-rules") != std::string::npos && bareModel.laneLevel.paths.empty());
}

// TWO APPROACHES FACE EACH OTHER WHERE THE JUNCTION STATES THEIR ARMS AS A THROUGH PAIR (ADR-0432): read from the pair,
// never from the angle between them. The crossing at (0, 0): the arterial west-east and the local street south-north,
// four approaches meeting exactly head on, two by two.
//   - As the layout states it (two through pairs): each approach's one far turn crosses the paths coming from the
//     approach facing it, so each of the four far turns has a place to wait inside.
//   - With no pair stated there: no approach is stated to face another, so no far turn has one. (The angle test called
//     these four head-on approaches facing whatever the junction said: the old answer.)
TEST_CASE(road_model_12b_approaches_face_each_other_where_a_through_pair_is_stated) {
    auto farTurnsAtTheCrossing = [](const nlohmann::json& g, uint32_t* waiting, uint32_t* facingLanes) {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        LaneGraphBuild lg;
        const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
        const LaneGraph& G = m.laneLevel;
        CHECK(lg.error.empty());
        uint32_t cross = kNoIndex;
        for (uint32_t j = 0; j < m.junctions.size(); ++j)
            if (m.junctions[j].at.length() < 1.0) cross = j;
        CHECK(cross != kNoIndex);
        uint32_t farTurns = 0;
        *waiting = 0;
        *facingLanes = 0;
        // the lanes arriving at the crossing that the model says face another lane arriving there
        uint32_t laneCross = kNoIndex;   // the lane level's junction that is the crossing
        for (uint32_t j = 0; j < G.junctions.size(); ++j)
            if (G.junctions[j].modelJunction == cross) laneCross = j;
        CHECK(laneCross != kNoIndex);
        std::vector<PathId> arriving;
        for (uint32_t p = 0; p < G.paths.size(); ++p)
            if (G.paths[p].kind == PathKind::Lane && G.paths[p].arm != kNoIndex && m.arrivesAt(PathId(p)) == &m.junctions[cross])
                arriving.push_back(PathId(p));
        for (const PathId a : arriving) {
            bool faces = false;
            for (const PathId b : arriving) faces = faces || m.facing(JunctionId(laneCross), a, b);
            *facingLanes += faces ? 1u : 0u;
        }
        for (const Path& p : G.paths) {
            if (p.kind != PathKind::Connector || G.junctions[p.junction.index].modelJunction != cross ||
                m.turnOf(PathId(static_cast<uint32_t>(&p - G.paths.data()))) != Turn::FarTurn)
                continue;
            ++farTurns;
            if (p.waitInsideAt != kNoWaitInside) ++*waiting;
        }
        return farTurns;
    };
    uint32_t waiting = 0, facingLanes = 0;
    const nlohmann::json stated = laneLevelGraph();
    CHECK(farTurnsAtTheCrossing(stated, &waiting, &facingLanes) == 4u);
    CHECK(waiting == 4u);
    CHECK(facingLanes > 0u);
    nlohmann::json unstated = stated;
    for (nlohmann::json& j : unstated["junctions"])
        if (std::fabs(j["at"][0].get<double>()) < 1.0 && std::fabs(j["at"][1].get<double>()) < 1.0) j.erase("through");
    CHECK(farTurnsAtTheCrossing(unstated, &waiting, &facingLanes) > 0u);
    CHECK(waiting == 0u);
    CHECK(facingLanes == 0u);

    // THE MODEL IS ASKED, AND HOLDS THE PAIR ALONE (ADR-0438): the lane table once kept a copy of each arriving lane's
    // through partner, written when it was built. Now RoadModel::facing reads the junction's stated pairs, so stating
    // them again on the model is the new answer for every reader, with nothing built again.
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(stated));
    LaneGraphBuild lg;
    const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
    const LaneGraph& G = m.laneLevel;
    uint32_t cross = kNoIndex;   // the lane level's junction at (0, 0)
    for (uint32_t j = 0; j < G.junctions.size(); ++j)
        if (m.junctions[G.junctions[j].modelJunction].at.length() < 1.0) cross = j;
    CHECK(cross != kNoIndex);
    if (cross == kNoIndex) return;
    // the lane paths arriving at the crossing: the arterial from the west (a0) and from the east (a1, its backward
    // lane), the local street from the south (l0) and from the north (l1, its backward lane)
    auto arrivingBy = [&](const char* lane) {
        const std::vector<uint32_t> paths = pathsOfLane(G, lg, laneNamed(m, lane).index);
        CHECK(!paths.empty());
        PathId found;
        for (uint32_t p : paths)
            if (G.paths[p].arm != kNoIndex && m.arrivesAt(PathId(p)) == m.junctionOf(JunctionId(cross))) found = PathId(p);
        CHECK(found.valid());
        return found;
    };
    const PathId west = arrivingBy("a0.f0"), east = arrivingBy("a1.b0"), south = arrivingBy("l0.f0"), north = arrivingBy("l1.b0");
    if (!west.valid() || !east.valid() || !south.valid() || !north.valid()) return;
    const JunctionId at(cross);
    CHECK(m.facing(at, west, east) && m.facing(at, east, west));
    CHECK(m.facing(at, south, north) && m.facing(at, north, south));
    CHECK(!m.facing(at, west, south) && !m.facing(at, west, north) && !m.facing(at, east, south) && !m.facing(at, west, west));
    CHECK(!m.facing(JunctionId(99), west, east));   // no such junction: nothing stated
    // the same lane table, the pairs stated again: none
    RoadModel restated = m;
    ModelJunction& J = restated.junctions[G.junctions[cross].modelJunction];
    J.through.clear();
    CHECK(!restated.facing(at, west, east));
    CHECK(!restated.facing(at, south, north));
    // ... and the arterial's west arm paired with the local street's south arm instead
    J.through.push_back({G.paths[west.index].arm, G.paths[south.index].arm});
    CHECK(restated.facing(at, west, south) && restated.facing(at, south, west));
    CHECK(!restated.facing(at, west, east));
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
    LaneGraphBuild lg;   // what building the model's lane level reported
    const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
    CHECK(lg.error.empty());
    CHECK(validate(m.laneLevel).empty());
    std::printf("      [lane level] merge_taper: %u lanes merge into their hosts, %u leave them, %u joins with a gap\n", lg.mergeJoins,
                lg.divergeJoins, lg.joinsWithGap);
    CHECK(lg.mergeJoins >= 3u);   // each ramp into the mainline (and a two-lane ramp's second lane into its first)
    const ModelLaneId host = laneNamed(m, "mainline.f2");
    for (const char* ramp : {"ramp_one.f0", "ramp_short.f0", "ramp_long.f0"}) {
        const std::vector<uint32_t> ps = pathsOfLane(m.laneLevel, lg,laneNamed(m, ramp).index);
        CHECK(!ps.empty());
        if (ps.empty()) continue;
        // its end leads into the mainline's kerb lane, from where their lines meet
        const LaneGraph& G = m.laneLevel;
        const Path& last = G.paths[ps.back()];
        CHECK(last.next.count == 1u);
        if (last.next.count == 1u) CHECK(G.paths[G.nextPaths[last.next.first].index].modelLane == host.index);
        // and where the two join, the measured crossing points hold a join of the two lanes
        bool join = false;
        for (uint32_t k = last.crossings.first; k < last.crossings.end(); ++k) {
            const CrossingPoint& x = G.crossings[G.pathCrossings[k].index];
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
        // (a table with no "lane_lines" says so, once: no line between lanes is solid)
        {
            std::vector<std::string> said;
            const LaneTurnRules none = parseLaneTurnRules(nlohmann::json{{"turns", nlohmann::json::array()}}, &said);
            CHECK(none.holdBeforeStopLine.empty() && said.size() == 1u);
            if (said.size() == 1u) CHECK(said.front().find("lane_lines") != std::string::npos);
        }
        const nlohmann::json doc = {{"lane_lines", {{"hold_before_stop_line", {"through"}}}}, {"turns", nlohmann::json::array({
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
    LaneGraphBuild lg;   // what building the model's lane level reported
    const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
    CHECK(lg.error.empty());
    const LaneGraph& G = m.laneLevel;
    uint32_t diverges = 0, merges = 0;
    for (const ModelConnection& c : m.connections) {
        if (c.ramp != "exit" && c.ramp != "entry") continue;
        if (c.kind == ConnectionKind::Diverge && c.ramp == "exit") {
            // THE EXIT: the host lane's path ending where their lines meet leads on along the host AND into the exit's
            // first path; the two paths' bodies split there (a measured crossing point)
            const std::vector<uint32_t> ramp = pathsOfLane(m.laneLevel, lg,c.to.index), host = pathsOfLane(m.laneLevel, lg,c.from.index);
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
            const std::vector<uint32_t> ramp = pathsOfLane(m.laneLevel, lg,c.from.index);
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
    LaneGraphBuild lg;   // what building the model's lane level reported
    const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
    CHECK(lg.error.empty());
    for (const std::string& st : streetsOfRoad) {
        bool said = false;
        // (by the problem's record, road_problem.h: its kind, street and junction)
        for (const RoadProblem& p : lg.problems)
            said = said || (p.kind == RoadProblemKind::StreetEndNoStopLine && p.street == st && p.junction == J["id"].get<std::string>());
        CHECK(said);
    }
    uint32_t joinedAcross = 0, lanesOfRoad = 0;
    const LaneGraph& G = m.laneLevel;
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
    RoadModel m2 = assembleRoadModel(*r2, oneCar());
    const std::size_t stated = m2.turns.size();
    CHECK(stated > 0u);
    ModelTurn stray = m2.turns.front();
    stray.junction = ModelJunctionId(static_cast<uint32_t>(m2.junctions.size() + 7));
    m2.turns.push_back(stray);
    const LaneGraphBuild lg2 = buildLaneLevel(*r2, m2, oneCar());   // the lane level again, of the model as changed
    const std::string expect = "stated turn " + std::to_string(stated) + " names no junction of the road model: not built";
    const std::vector<std::string> lines2 = problemLines(lg2.problems);
    CHECK(std::find(lines2.begin(), lines2.end(), expect) != lines2.end());
    uint32_t connectors = 0;
    for (const Path& p : m2.laneLevel.paths) connectors += p.kind == PathKind::Connector ? 1 : 0;
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
        LaneGraphBuild lg;
        const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
        CHECK(!m.armsInRing);
        CHECK(said(m));
        std::printf("      %s\n", lg.error.c_str());
        CHECK(lg.error.find("--arm-order --write") != std::string::npos && m.laneLevel.paths.empty());
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
        LaneGraphBuild lg;
        const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
        CHECK(m.armsInRing);
        CHECK(!said(m));
        CHECK(m.junctions[0].arms.size() == n && m.junctions[0].arms[0].road == reversed[0]["road"].get<std::string>());
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

// THE LAYOUT WRITTEN AGAIN INTO A ROAD FILE THAT HAS NOT CHANGED GIVES THE SAME BYTES (ADR-0435): what
// `lanes_tool layout <file> --write` does, on the three shipped files, in memory. Two things made it change a file
// before:
//   - it stated a carry-on at a street end where the file's own list states a junction (a second answer to "how does
//     this street run on", which no reader asks: traffic_lab, 27 streets, 40 ends);
//   - it dropped every junction's `nodes` (they came back only with a second command, `--nodes --write`).
// A street's end is at a junction or carries on, never both: counted here, with each file's own numbers.
TEST_CASE(road_model_19_the_layout_written_again_into_a_shipped_file_changes_nothing) {
    struct Shipped { const char* file; std::size_t streetsCarryingOn, carryOnEnds, junctionsWithNodes; };
    const Shipped files[] = {{"planned/traffic_lab_lanelab.json", 0, 0, 15},
                             {"planned/river_town_lanelab.json", 22, 32, 30},
                             {"planned/agent_lab_lanelab.json", 22, 32, 30}};
    for (const Shipped& f : files) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/" + f.file;
        const std::string text = slurp(path);
        const RoadLabGraph g = RoadLabGraph::fromJson(nlohmann::json::parse(text), path.substr(0, path.find_last_of('/')));
        RoadLayout layout = deriveLayout(g, LayoutSource::Inferred);
        const std::vector<NamedStretch> keep = namedStretches(g.streets);
        if (!keep.empty()) (void)nameLayout(layout, g, keep);
        std::string err;
        const std::string out = insertLayoutText(stripLayoutText(text, true), layout, &err);
        if (!err.empty()) std::printf("      %s: %s\n", f.file, err.c_str());
        CHECK(out == text);
        // THE LANE RULES WRITTEN ONCE ARE WHERE THE LAYOUT WRITER LEAVES THEM (ADR-0437): the shipped file with its
        // "lane_rules" member cut out (and the comma before it), then written by `--lane-rules 20`, is the shipped file
        // to the byte -- so the layout written after it moves nothing. (It once went after the last member, behind
        // "streets", and the next layout write moved it.)
        const std::size_t key = text.find("\"lane_rules\"");
        CHECK(key != std::string::npos);
        if (key != std::string::npos) {
            const std::size_t comma = text.rfind(',', key), close = text.find('}', key);
            std::string without = text;
            without.erase(comma, close + 1 - comma);
            CHECK(without.find("lane_rules") == std::string::npos);
            std::string why;
            const std::string again = insertLaneRulesText(without, 20.0, &why);
            if (!why.empty()) std::printf("      %s: %s\n", f.file, why.c_str());
            CHECK(again == text);
        }
        std::size_t streets = 0, ends = 0, endsAtAJunction = 0, withNodes = 0;
        for (const ModelStreet& st : layout.streets) {
            streets += st.carriesOn[0].valid() || st.carriesOn[1].valid() ? 1u : 0u;
            for (std::size_t k = 0; k < 2; ++k) {
                ends += st.carriesOn[k].valid() ? 1u : 0u;
                endsAtAJunction += st.carriesOn[k].valid() && st.junctions[k].valid() ? 1u : 0u;
            }
        }
        for (const ModelJunction& J : g.junctions) withNodes += J.nodes.empty() ? 0u : 1u;
        std::printf("      %s: %zu streets carry on at %zu ends (%zu of them at a junction); %zu junctions list their nodes; written again: %s\n",
                    f.file, streets, ends, endsAtAJunction, withNodes, out == text ? "the same bytes" : "CHANGED");
        CHECK(streets == f.streetsCarryingOn && ends == f.carryOnEnds);
        CHECK(endsAtAJunction == 0u);
        CHECK(withNodes == f.junctionsWithNodes);
    }
}

// ---- 20. the questions readers ask of the junctions and streets (ADR-0436) -----------------------------------------------
TEST_CASE(road_model_20_one_answer_for_the_arm_a_street_is_and_where_its_ends_are) {
    // the signed scene: the arterial a0 runs from (-200, 0) to the crossing at (0, 0); l1 leaves the crossing north
    RoadModel m = plannedRoadModel(RoadLabGraph::fromJson(signedGraph()));
    ModelStreetId a0, l1;
    for (uint32_t s = 0; s < m.streets.size(); ++s) {
        if (m.streets[s].road == "a0") a0 = ModelStreetId(s);
        if (m.streets[s].road == "l1") l1 = ModelStreetId(s);
    }
    CHECK(a0.valid() && l1.valid());
    if (!a0.valid() || !l1.valid()) return;
    // WHERE A STREET'S ENDS ARE, and its whole centre line
    Vec2 from, to;
    CHECK(m.streetEnd(m.streets[a0.index], 0, from) && m.streetEnd(m.streets[a0.index], 1, to));
    CHECK(approxEqual(from.x, -200.0, kSamePoint) && approxEqual(from.y, 0.0, kSamePoint));
    CHECK(approxEqual(to.x, 0.0, kSamePoint) && approxEqual(to.y, 0.0, kSamePoint));
    const std::vector<Vec2> line = m.centreLine(m.streets[a0.index].line);
    CHECK(line.size() == 2u);
    CHECK(m.centreLine(Range{static_cast<uint32_t>(m.linePoints.size()), 4}).empty());   // not a line of this model
    // THE ARM A STREET IS at each end: none at a0's dead end; at the crossing, the arm of road a0 whose spine runs in
    CHECK(m.armAt(a0, 0) == -1);
    const int arm = m.armAt(a0, 1);
    const ModelJunction& cross = m.junctions[m.streets[a0.index].junctions[1].index];
    CHECK(arm == 2);
    if (arm < 0) return;
    CHECK(cross.arms[static_cast<std::size_t>(arm)].road == "a0" && cross.arms[static_cast<std::size_t>(arm)].spineIn);
    CHECK(m.streetOf(cross.arms[static_cast<std::size_t>(arm)]) == &m.streets[a0.index]);
    const int l1Arm = m.armAt(l1, 0);   // l1 leaves the crossing: its spine runs out
    CHECK(l1Arm >= 0 && l1Arm != arm && cross.arms[static_cast<std::size_t>(l1Arm)].road == "l1");
    CHECK(m.armAt(l1, 1) == -1 || m.streets[l1.index].junctions[1].valid());
    CHECK(m.armAt(ModelStreetId(9999), 0) == -1 && m.armAt(a0, 2) == -1);
    // A JUNCTION THAT LISTS THE STREET TWICE (a broken file): every reader gets the same arm, the first listed. The
    // street map once took the last and the lane level the first.
    ModelJunction& broken = m.junctions[m.streets[a0.index].junctions[1].index];
    broken.arms.push_back(broken.arms[static_cast<std::size_t>(arm)]);
    CHECK(m.armAt(a0, 1) == 2);
    ModelLaneSpan span;
    span.street = a0;
    span.from = m.streets[a0.index].from;
    span.to = m.streets[a0.index].to;
    CHECK(laneSpanEnd(m, span, 1).arm == 2);
    // an arm that names no street of the model
    ModelArm stray;
    CHECK(m.streetOf(stray) == nullptr);
}

// THE ARM A LINK OF THE STREET MAP ARRIVES BY IS ASKED OF THE MODEL (ADR-0437): the link carries its street and the
// station at its far end, and the model says which arm that is (RoadModel::armReachedAt). The link once kept a copy of
// the arm, written when the map was built; a junction whose arms are stated again then left the copy pointing at
// another road's arm.
TEST_CASE(road_model_21_the_arm_a_street_map_link_arrives_by_is_the_models) {
    // the signed scene: the arterial a0 runs 200 m from its dead end at (-200, 0) to the crossing at (0, 0)
    RoadModel m = plannedRoadModel(RoadLabGraph::fromJson(signedGraph()));
    const ModelStreetId a0 = streetNamed(m, "a0");
    CHECK(a0.valid());
    if (!a0.valid()) return;
    engine::NavGraph map;
    engine::StreetMapReport rep;
    CHECK(engine::appendStreetMap(map, m, 0, nullptr, &rep));
    const auto armsReached = [&](std::vector<int>& atCrossing, std::vector<int>& elsewhere) {
        atCrossing.clear();
        elsewhere.clear();
        for (const engine::NavLink& L : map.links) {
            if (L.street != static_cast<int>(a0.index)) continue;
            const int arm = m.armReachedAt(ModelStreetId(static_cast<uint32_t>(L.street)), L.stationTo);
            (approxEqual(L.stationTo, 200.0, kWrittenStation) ? atCrossing : elsewhere).push_back(arm);
        }
    };
    std::vector<int> atCrossing, elsewhere;
    armsReached(atCrossing, elsewhere);
    // one link arrives at the crossing, by arm 2 (road a0, its spine running in); every other link of the street
    // arrives at no junction arm (along the street, or at its dead end)
    CHECK(atCrossing == std::vector<int>{2});
    CHECK(!elsewhere.empty());
    for (int arm : elsewhere) CHECK(arm == -1);
    CHECK(m.armReachedAt(a0, 100.0) == -1 && m.armReachedAt(a0, 0.0) == -1 && m.armReachedAt(a0, 200.004) == 2);
    CHECK(m.armReachedAt(ModelStreetId(9999), 200.0) == -1);
    // THE CROSSING'S ARMS STATED AGAIN, its first arm now listed last: a0's arm is 1, and the same links say so with no
    // new map built
    ModelJunction& cross = m.junctions[m.streets[a0.index].junctions[1].index];
    std::rotate(cross.arms.begin(), cross.arms.begin() + 1, cross.arms.end());
    CHECK(cross.arms[1].road == "a0" && cross.arms[1].spineIn);
    armsReached(atCrossing, elsewhere);
    CHECK(atCrossing == std::vector<int>{1});
}

// ---- 22. the regulations along a path are the model's (ADR-0437) -----------------------------------------------------------
// A rule standing on a stretch of a street is stated ON THE MODEL, by street and station, and a path's regulations are
// asked of the model (RoadModel::regulationsAlong). The lane table once held them: only each arm's signs, stood at the
// end of the arriving lanes when the table was built, with no home for a rule part-way along a street and no way to see
// one stated afterwards.
TEST_CASE(road_model_22_the_regulations_along_a_path_are_asked_of_the_model) {
    // The lane-level scene: the arterial a1 runs 200 m straight from the crossing j0 at (0, 0) to the T j1 at (200, 0),
    // three lanes each way. Its stop lines stand at stations 11.2 (at j0) and 188.8 (at j1), and each lane is cut 20 m
    // before the stop line it runs to (the solid lines). So along the spine a lane is a 157.6 m path from 11.2 to 168.8
    // and a 20 m path from 168.8 to 188.8; against it, a 157.6 m path from 188.8 to 31.2 and a 20 m path from 31.2 to 11.2.
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(laneLevelGraph()));
    RoadModel m = assembleRoadModel(*r, oneCar());
    const ModelStreetId a1 = streetNamed(m, "a1");
    CHECK(a1.valid());
    if (!a1.valid()) return;
    // the four kinds of path on a1, by the stretch of spine each carries
    const auto pathsFrom = [&](Real stationFrom, Real stationTo) {
        std::vector<PathId> out;
        for (uint32_t p = 0; p < m.laneLevel.paths.size(); ++p) {
            const Path& path = m.laneLevel.paths[p];
            if (path.kind == PathKind::Lane && path.street == a1.index && path.onSpine &&
                approxEqual(path.stationFrom, stationFrom, kWrittenStation) && approxEqual(path.stationTo, stationTo, kWrittenStation))
                out.push_back(PathId(p));
        }
        return out;
    };
    const std::vector<PathId> alongFirst = pathsFrom(11.2, 168.8), alongLast = pathsFrom(168.8, 188.8);
    const std::vector<PathId> againstFirst = pathsFrom(188.8, 31.2), againstLast = pathsFrom(31.2, 11.2);
    CHECK(alongFirst.size() == 3u && alongLast.size() == 3u && againstFirst.size() == 3u && againstLast.size() == 3u);
    // AS BUILT: nothing along a1 but the sign its arm at j0 states, at the end (20 m) of each lane arriving there
    for (PathId p : alongFirst) CHECK(m.regulationsAlong(p).empty());
    for (PathId p : alongLast) CHECK(m.regulationsAlong(p).empty());   // j1 states no sign on a1's arm
    for (PathId p : againstFirst) CHECK(m.regulationsAlong(p).empty());
    const std::string armSign = m.arrivesAt(againstLast.empty() ? PathId() : againstLast[0])
                                    ? m.arrivesAt(againstLast[0])->arms[0].regulations.front()
                                    : std::string();
    for (PathId p : againstLast) {
        const std::vector<PathRegulation> regs = m.regulationsAlong(p);
        CHECK(regs.size() == 1u);
        if (regs.size() != 1u) continue;
        CHECK(regs[0].kind == armSign && !armSign.empty());
        CHECK(approxEqual(regs[0].from, 20.0, kWrittenStation) && approxEqual(regs[0].to, 20.0, kWrittenStation));
    }
    // STATED ON THE STREET: a top speed of 9 m/s from station 60 to 180 for traffic running along the spine, and a
    // sign at station 100 for traffic running against it.
    ModelRegulation limit;
    limit.kind = "top_speed";
    limit.street = a1;
    limit.from = 60;
    limit.to = 180;
    limit.dir = 1;
    limit.value = 9;
    CHECK(stateRegulation(m, limit));
    ModelRegulation sign;
    sign.kind = "school";
    sign.street = a1;
    sign.from = sign.to = 100;
    sign.dir = -1;
    sign.value = 5;
    CHECK(stateRegulation(m, sign));
    // along the spine: on the long path from 60 - 11.2 = 48.8 m to its end (157.6 m); on the last path from its start to
    // 180 - 168.8 = 11.2 m
    for (PathId p : alongFirst) {
        const std::vector<PathRegulation> regs = m.regulationsAlong(p);
        CHECK(regs.size() == 1u);
        if (regs.size() != 1u) continue;
        CHECK(regs[0].kind == "top_speed");
        CHECK(approxEqual(regs[0].from, 48.8, kWrittenStation) && approxEqual(regs[0].to, 157.6, kWrittenStation));
        CHECK(approxEqual(regs[0].value, 9.0, kSamePoint));
    }
    for (PathId p : alongLast) {
        const std::vector<PathRegulation> regs = m.regulationsAlong(p);
        CHECK(regs.size() == 1u);
        if (regs.size() != 1u) continue;
        CHECK(approxEqual(regs[0].from, 0.0, kWrittenStation) && approxEqual(regs[0].to, 11.2, kWrittenStation));
    }
    // against the spine: the sign 188.8 - 100 = 88.8 m along the long path; the last path keeps its arm's sign alone
    for (PathId p : againstFirst) {
        const std::vector<PathRegulation> regs = m.regulationsAlong(p);
        CHECK(regs.size() == 1u);
        if (regs.size() != 1u) continue;
        CHECK(regs[0].kind == "school");
        CHECK(approxEqual(regs[0].from, 88.8, kWrittenStation) && approxEqual(regs[0].to, 88.8, kWrittenStation));
        CHECK(approxEqual(regs[0].value, 5.0, kSamePoint));
    }
    for (PathId p : againstLast) CHECK(m.regulationsAlong(p).size() == 1u);
    // a connector has none; nor a path the model does not have
    for (uint32_t p = 0; p < m.laneLevel.paths.size(); ++p)
        if (m.laneLevel.paths[p].kind == PathKind::Connector) CHECK(m.regulationsAlong(PathId(p)).empty());
    CHECK(m.regulationsAlong(PathId(99999)).empty());
    // THE ARM'S SIGN STATED AGAIN is the new answer with no lane level built again
    ModelJunction& j0 = m.junctions[m.streets[a1.index].junctions[0].index];
    j0.arms[0].regulations = {"give_way"};
    for (PathId p : againstLast) {
        const std::vector<PathRegulation> regs = m.regulationsAlong(p);
        CHECK(regs.size() == 1u && regs[0].kind == "give_way");
    }
    // WHAT THE MODEL REFUSES: no kind, no street, a way that is neither, a stretch backwards or off its street
    std::string why;
    ModelRegulation bad = limit;
    bad.kind.clear();
    CHECK(!stateRegulation(m, bad, &why));
    bad = limit;
    bad.street = ModelStreetId(9999);
    CHECK(!stateRegulation(m, bad, &why));
    bad = limit;
    bad.dir = 0;
    CHECK(!stateRegulation(m, bad, &why));
    bad = limit;
    bad.from = 190;
    CHECK(!stateRegulation(m, bad, &why) && why.find("runs backwards") != std::string::npos);
    bad = limit;
    bad.to = 250;
    CHECK(!stateRegulation(m, bad, &why) && why.find("lies off street") != std::string::npos);
    CHECK(m.regulations.size() == 2u);
}

// ---- 23. the road class table is a data file, and a class is what its row says (ADR-0438) --------------------------------

TEST_CASE(road_model_23_the_road_class_table_is_its_data_file_and_no_class_is_asked_by_name) {
    // THE SHIPPED TABLE, row by row, against the numbers the C++ table held before it moved to
    // assets/data/road_classes.json (written out here by hand)
    const std::map<std::string, RoadClassSpec>& t = roadClasses();
    CHECK(t.size() == 12u);
    CHECK(defaultRoadClassName() == "local");
    struct Row {
        const char* name;
        double w;
        int fwd, back;
        double shoulder, sidewalk, median;
        int rank;
        double gMax, window, thick;
        engine::RoadClass engineClass;
    };
    const Row rows[] = {
        {"local", 6.0, 1, 1, 0.0, 5.0, 0.0, 1, 0.12, 40.0, 0.5, engine::RoadClass::Local},
        {"collector", 4.0, 2, 2, 0.0, 5.0, 0.0, 1, 0.10, 80.0, 0.5, engine::RoadClass::Collector},
        {"arterial", 22.0 / 6, 3, 3, 0.0, 5.0, 0.0, 2, 0.08, 120.0, 0.6, engine::RoadClass::Arterial},
        {"boulevard", 22.0 / 6, 3, 0, 0.0, 5.0, 0.0, 2, 0.08, 120.0, 0.6, engine::RoadClass::Local},
        {"freeway", 3.75, 4, 0, 2.5, 0.0, 0.0, 3, 0.06, 200.0, 1.2, engine::RoadClass::Freeway},
        {"rural", 3.5, 1, 1, 2.5, 0.0, 0.0, 1, 0.10, 60.0, 0.5, engine::RoadClass::Collector},
        {"ramp", 4.5, 1, 0, 2.5, 0.0, 0.0, 0, 0.09, 40.0, 1.0, engine::RoadClass::Ramp},
        {"divider", 0.0, 0, 0, 0.0, 0.0, 1.5, -1, 1.0, 1.0, 0.0, engine::RoadClass::Local},
        {"mountain", 3.5, 1, 1, 2.5, 0.0, 0.0, 1, 0.15, 60.0, 0.5, engine::RoadClass::Local},
        {"rail", 4.0, 1, 1, 0.0, 0.0, 0.0, 3, 0.025, 400.0, 0.8, engine::RoadClass::Local},
        {"light_rail", 3.6, 1, 1, 0.0, 0.0, 0.0, 2, 0.05, 120.0, 0.7, engine::RoadClass::Local},
        {"footway", 3.0, 1, 0, 0.0, 0.0, 0.0, 0, 0.6, 20.0, 0.4, engine::RoadClass::Local},
    };
    for (const Row& x : rows) {
        CHECK(t.count(x.name) == 1u);
        if (!t.count(x.name)) continue;
        const RoadClassSpec& c = t.at(x.name);
        // (exactly: the file's numbers are the old table's to the last bit, so nothing built moves)
        const bool same = c.name == x.name && c.lanes.w == x.w && c.lanes.fwd == x.fwd && c.lanes.back == x.back && c.shoulder == x.shoulder &&
                          c.sidewalk == x.sidewalk && c.median == x.median && c.rank == x.rank && c.gMax == x.gMax && c.window == x.window &&
                          c.thick == x.thick && c.engineClass == x.engineClass && c.lanes.gap == 0.0 && c.lanes.dovetail == 60.0;
        if (!same) std::printf("    row %s differs from the numbers written here\n", x.name);
        CHECK(same);
        CHECK(c.lanes.parking == (std::string(x.name) == "local" ? 2.5 : 0.0));
        CHECK(c.balance == (std::string(x.name) == "mountain" ? 0.5 : 0.0));
        CHECK(c.track == (std::string(x.name) == "rail" || std::string(x.name) == "light_rail"));
        CHECK(c.footway == (std::string(x.name) == "footway"));
        CHECK(c.limitedAccess == (std::string(x.name) == "freeway"));
        CHECK(c.defaultBarriers == (std::string(x.name) == "freeway" || std::string(x.name) == "ramp"));
    }
    // the barriers the rows state: the mountain road's (none against pavement, a median or a street; a guardrail on a
    // bridge, and at grade only over a 2 m drop), a track's parapet on a bridge, a footway's
    auto barrier = [&](const char* cls, std::size_t role) { return t.at(cls).edges[role]; };
    for (std::size_t role = 0; role < 3; ++role) CHECK(barrier("mountain", role).set && barrier("mountain", role).kind == BarrierKind::None);
    CHECK(barrier("mountain", 3).set && barrier("mountain", 3).kind == BarrierKind::Guardrail && barrier("mountain", 3).h == 0.75 &&
          barrier("mountain", 3).thick == 0.1 && barrier("mountain", 3).minDrop == 0.0);
    CHECK(barrier("mountain", 4).kind == BarrierKind::Guardrail && barrier("mountain", 4).minDrop == 2.0);
    for (const char* track : {"rail", "light_rail"}) {
        CHECK(barrier(track, 3).set && barrier(track, 3).kind == BarrierKind::Wall && barrier(track, 3).h == 1.0 && barrier(track, 3).thick == 0.4);
        CHECK(!barrier(track, 0).set && !barrier(track, 4).set);
    }
    CHECK(barrier("footway", 3).kind == BarrierKind::Wall && barrier("footway", 3).h == 1.1 && barrier("footway", 3).thick == 0.2);
    CHECK(barrier("footway", 2).set && barrier("footway", 2).kind == BarrierKind::None && barrier("footway", 4).set && !barrier("footway", 0).set);
    for (const char* plain : {"local", "collector", "arterial", "boulevard", "freeway", "rural", "ramp", "divider"})
        for (std::size_t role = 0; role < 5; ++role) CHECK(!barrier(plain, role).set);
    // the plan picture's colours
    CHECK(t.at("freeway").planColour == "#2b2b30" && t.at("ramp").planColour == "#38383e" && t.at("arterial").planColour == "#3a3a3c" &&
          t.at("local").planColour == "#454547" && t.at("collector").planColour.empty());
    // the file's bytes key the city's bake
    CHECK(roadClassTableHash() == engine::bundle::fnv1aStr(slurp(std::string(RT_SOURCE_DIR) + "/" + kRoadClassTableFile)));

    // A CLASS IS WHAT ITS ROW SAYS, NOT WHAT IT IS CALLED. A road file states an "avenue" that is an arterial to the
    // engine and limited-access, and states its own "freeway" row as an ordinary collector street. (Asked by name, the
    // avenue was a local street that met other roads at one level, and the freeway a freeway whatever its row said.)
    nlohmann::json file = smallGraph();
    file["classes"]["avenue"] = {{"engine_class", "arterial"}, {"limited_access", true}, {"default_barriers", true}};
    file["classes"]["freeway"] = {{"engine_class", "collector"}, {"limited_access", false}};
    for (nlohmann::json& e : file["edges"]) {
        if (e["id"] == "a0") e["class"] = "avenue";
        if (e["id"] == "a2") e["class"] = "freeway";
    }
    const RoadLabGraph g = RoadLabGraph::fromJson(file);
    const EdgeSpec& avenue = *g.find("a0");
    const EdgeSpec& fw = *g.find("a2");
    const EdgeSpec& street = *g.find("l0");
    CHECK(classOf(g, avenue) == engine::RoadClass::Arterial);
    CHECK(classOf(g, fw) == engine::RoadClass::Collector);
    CHECK(classOf(g, street) == engine::RoadClass::Local);
    CHECK(mustSeparate(g, avenue, street));
    CHECK(!mustSeparate(g, fw, street));
    CHECK(!mustSeparate(g, street, *g.find("l1")));
    // a class a file adds starts from the default row: the local street's section and treatment, but no kerb parking
    // and no colour of its own
    CHECK(g.cls(avenue).lanes.w == 6.0 && g.cls(avenue).sidewalk == 5.0 && g.cls(avenue).lanes.parking == 0.0 && g.cls(avenue).planColour.empty());
    CHECK(g.cls(avenue).defaultBarriers && g.cls(avenue).junctionMarkings);
    // ...and a row of the table's name keeps what it does not restate
    CHECK(g.cls(fw).lanes.fwd == 4 && g.cls(fw).shoulder == 2.5 && g.cls(fw).defaultBarriers && !g.cls(fw).junctionMarkings);
    // written back only where a row differs from the one it would be read with
    CHECK(!classJson(t.at("freeway")).contains("limited_access") && !classJson(t.at("freeway")).contains("engine_class"));
    CHECK(classJson(g.cls(fw)).value("limited_access", true) == false);
    CHECK(classJson(g.cls(fw)).value("engine_class", std::string()) == "collector");
    CHECK(classJson(g.cls(avenue)).value("engine_class", std::string()) == "arterial");

    // THE TABLE'S OWN READER: `like` starts a row from another, in whatever order the file lists them; what it refuses
    const nlohmann::json doc = nlohmann::json::parse(R"({"default": "street", "classes": {
        "alley": {"like": "street", "w": 2.5, "engine_class": "alley"},
        "street": {"w": 3.0, "fwd": 1, "back": 1, "sidewalk": 2.0, "rank": 1}}})");
    std::string fallback;
    const std::map<std::string, RoadClassSpec> small = parseRoadClassTable(doc, &fallback);
    CHECK(fallback == "street" && small.size() == 2u);
    CHECK(small.at("alley").name == "alley" && small.at("alley").lanes.w == 2.5 && small.at("alley").sidewalk == 2.0 && small.at("alley").lanes.back == 1 &&
          small.at("alley").engineClass == engine::RoadClass::Alley && small.at("street").engineClass == engine::RoadClass::Local);
    auto refused = [](const char* text) {
        try {
            parseRoadClassTable(nlohmann::json::parse(text));
        } catch (const std::runtime_error&) {
            return true;
        }
        return false;
    };
    CHECK(refused(R"({"default": "street", "classes": {"street": {"like": "lane"}}})"));               // like a row it lacks
    CHECK(refused(R"({"default": "street", "classes": {"street": {"like": "street"}}})"));             // like itself
    CHECK(refused(R"({"default": "street", "classes": {"street": {"engine_class": "motorway"}}})"));   // no such engine class
    CHECK(refused(R"({"default": "lane", "classes": {"street": {}}})"));                               // no default row
    CHECK(refused(R"({"default": "street"})"));                                                        // no classes
}

// ---- 24. kerb parking spaces are stated in the model, once (ADR-0439) ----------------------------------------------------

namespace {
bool samePlace(const Vec2& a, const Vec2& b) { return approxEqual(a.x, b.x, kSamePoint) && approxEqual(a.y, b.y, kSamePoint); }
}  // namespace

TEST_CASE(road_model_24_kerb_parking_spaces_are_stated_once_and_asked_of_the_model) {
    // THE RULES are data: a space is 7 m of kerb, and 10 m are kept empty before a stop line (assets/data/kerb_parking.json)
    const KerbParkingRules& rules = kerbParkingRules();
    CHECK(rules.usable && rules.spaceLength == 7.0 && rules.clearOfStopLine == 10.0);
    CHECK(rules.hash == engine::bundle::fnv1aStr(slurp(std::string(RT_SOURCE_DIR) + "/assets/data/kerb_parking.json")));
    {   // what the reader refuses, and a file that reads
        KerbParkingRules read;
        std::string why;
        CHECK(!parseKerbParkingRules(nlohmann::json::parse(R"({"clear_of_stop_line": 10})"), read, &why) && !read.usable && !why.empty());
        CHECK(!parseKerbParkingRules(nlohmann::json::parse(R"({"space_length": 0, "clear_of_stop_line": 10})"), read) && !read.usable);
        CHECK(!parseKerbParkingRules(nlohmann::json::parse(R"({"space_length": 6})"), read) && !read.usable);
        CHECK(!parseKerbParkingRules(nlohmann::json::parse(R"({"space_length": 6, "clear_of_stop_line": -1})"), read) && !read.usable);
        CHECK(parseKerbParkingRules(nlohmann::json::parse(R"({"space_length": 6, "clear_of_stop_line": 0})"), read) && read.usable &&
              read.spaceLength == 6.0 && read.clearOfStopLine == 0.0);
    }

    // THE LANE-LEVEL SCENE. Its four local streets park (2.5 m of each 6 m lane, the band's middle 4.75 m from the centre
    // line); its arterials and its one-way street do not.
    //   l0 runs north up x = 0 from a dead end at (0, -150) to the crossing at (0, 0): station 150. The arterial it
    //      meets is 22 m wide, so l0's stop line stands 11 + 5.2 m back, at station 133.8, and spaces end by 123.8.
    //      17 spaces of 7 m fit in 0 to 123.8 (119 m), centred: from 2.4 to 121.4. The same on each kerb.
    //   l1 runs on north from the crossing to a bend at (0, 150) where it only carries on: its stop line is at 16.2, so
    //      spaces lie in 26.2 to 150: 17 again, from 28.6 to 147.6.
    //   l2 is l0's twin at x = 200. l3 runs from the bend to a dead end at (120, 220), 138.92 m with no junction: 19
    //      spaces from 2.96.
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(laneLevelGraph()));
    const RoadModel m = assembleRoadModel(*r, oneCar());
    const ModelStreetId l0 = streetNamed(m, "l0"), l1 = streetNamed(m, "l1"), l2 = streetNamed(m, "l2"), l3 = streetNamed(m, "l3");
    CHECK(l0.valid() && l1.valid() && l2.valid() && l3.valid());
    if (!l0.valid() || !l1.valid() || !l2.valid() || !l3.valid()) return;
    const KerbSpaceCounts counts = kerbSpaceCounts(m);
    CHECK(counts.streetsWithBand == 4u && counts.streets == 4u && counts.spaces == 140u);
    CHECK(approxEqual(counts.kerbMetres, 980.0, kSamePoint));
    CHECK(m.kerbRuns.size() == 8u);   // one run on each kerb of each of the four
    for (const char* none : {"a0", "a1", "a2", "o0"}) CHECK(m.kerbSpacesOf(streetNamed(m, none)).empty());
    for (const std::string& s : m.notStated) CHECK(s.find("the spaces along it are not") == std::string::npos);   // (the old line is gone)

    // l0: the kerb of the traffic running along the spine first (17), then the other kerb (17), each by index
    const std::vector<KerbSpace> onL0 = m.kerbSpacesOf(l0);
    CHECK(onL0.size() == 34u);
    if (onL0.size() != 34u) return;
    CHECK(onL0[0].street == l0 && onL0[0].dir == 1 && onL0[0].index == 0u);
    CHECK(approxEqual(onL0[0].from, 2.4, kWrittenStation) && approxEqual(onL0[0].to, 9.4, kWrittenStation));
    CHECK(onL0[0].length == 7.0 && onL0[0].width == 2.5);
    CHECK(onL0[16].dir == 1 && onL0[16].index == 16u && approxEqual(onL0[16].from, 114.4, kWrittenStation) &&
          approxEqual(onL0[16].to, 121.4, kWrittenStation));
    CHECK(onL0[17].dir == -1 && onL0[17].index == 0u && approxEqual(onL0[17].from, 2.4, kWrittenStation));
    CHECK(onL0[33].dir == -1 && onL0[33].index == 16u);
    CHECK(m.kerbSpaceName(onL0[0]) == "l0 along 0" && m.kerbSpaceName(onL0[33]) == "l0 against 16");
    // one space by its id; an index past the kerb's last is no space
    const KerbSpace byId = m.kerbSpace(l0, -1, 16);
    CHECK(byId.valid() && byId.dir == -1 && byId.index == 16u && approxEqual(byId.from, 114.4, kWrittenStation));
    CHECK(!m.kerbSpace(l0, 1, 17).valid() && !m.kerbSpace(streetNamed(m, "a1"), 1, 0).valid());
    // the other streets' first and last spaces
    const std::vector<KerbSpace> onL1 = m.kerbSpacesOf(l1), onL2 = m.kerbSpacesOf(l2), onL3 = m.kerbSpacesOf(l3);
    CHECK(onL1.size() == 34u && onL2.size() == 34u && onL3.size() == 38u);
    if (onL1.size() != 34u || onL3.size() != 38u) return;
    CHECK(approxEqual(onL1[0].from, 28.6, kWrittenStation) && approxEqual(onL1[16].to, 147.6, kWrittenStation));
    // (l3's street ends at station 138.92, as the layout writes it: (138.92 - 19 x 7) / 2 = 2.96)
    CHECK(approxEqual(onL3[0].from, 2.96, kWrittenStation) && onL3[18].dir == 1 && onL3[19].dir == -1);

    // WHERE A CAR IN A SPACE STANDS. l0's spine runs north, and the traffic running along it keeps to the east side (the
    // world drives on the left; the plan is its mirror image), so that kerb's band is at x = +4.75; the other kerb's at
    // -4.75, its cars facing south. The first space's middle is at station 5.9: y = -144.1.
    KerbSpacePose pose;
    CHECK(m.kerbSpacePose(onL0[0], pose) && samePlace(pose.at, Vec2(4.75, -144.1)) && samePlace(pose.facing, Vec2(0, 1)));
    CHECK(m.kerbSpacePose(onL0[17], pose) && samePlace(pose.at, Vec2(-4.75, -144.1)) && samePlace(pose.facing, Vec2(0, -1)));
    CHECK(m.kerbSpacePose(onL1[0], pose) && samePlace(pose.at, Vec2(4.75, 32.1)));
    // l3 runs up and to the east from (0, 150): along (120, 70) / 138.92
    CHECK(m.kerbSpacePose(onL3[0], pose) && samePlace(pose.facing, Vec2(120.0 / std::sqrt(19300.0), 70.0 / std::sqrt(19300.0))));
    CHECK(!m.kerbSpacePose(KerbSpace(), pose));

    // THE KERB LANE BESIDE A SPACE: the lane path a car stops on to turn into it. l0's one lane running north is one
    // path from the dead end (station 0) to its stop line (133.8). The lane is 6 m wide and the parking band (2.5 m, its
    // middle 4.75 m out) lies over its outer 2.5 m, so it is driven along the middle of the 3.5 m left (ADR-0450):
    // 1.75 m from the centre line. Beside the first space: 5.9 m along it. Beside the last (its middle at station
    // 117.9): 117.9 m along.
    const KerbLanePlace first = m.kerbLaneBeside(onL0[0]), last = m.kerbLaneBeside(onL0[16]);
    CHECK(first.valid() && last.valid() && first.path == last.path);
    if (!first.valid() || !last.valid()) return;
    const Path& firstPath = m.laneLevel.paths[first.path.index];
    CHECK(firstPath.street == l0.index && firstPath.alongSpine() && approxEqual(firstPath.stationFrom, 0.0, kWrittenStation) &&
          approxEqual(firstPath.stationTo, 133.8, kWrittenStation));
    CHECK(approxEqual(first.along, 5.9, kWrittenStation) && samePlace(first.at, Vec2(1.75, -144.1)));
    CHECK(approxEqual(last.along, 117.9, kWrittenStation) && samePlace(last.at, Vec2(1.75, -32.1)));
    // ...and where a road has several lanes one way, the path is the one nearest the kerb: the arterial a1 has three
    // lanes of 3.67 m each way, and the kerb lane of the traffic running east along its spine is the third out from
    // the centre line, its middle 9.17 m south of it (y = -9.17).
    const KerbLanePlace kerbOfThree = m.kerbLaneAt(streetNamed(m, "a1"), 1, 100.0);
    CHECK(kerbOfThree.valid() && samePlace(kerbOfThree.at, Vec2(100.0, -2.5 * 22.0 / 6.0)));
    if (kerbOfThree.valid()) CHECK(m.laneLevel.paths[kerbOfThree.path.index].indexInSection == 2u);
    // ...and beside the first space of the other kerb: the lane running south, 1.75 m the other side of the centre line,
    // which ends at the dead end (station 0), so the place is 5.9 m before that path's end
    const KerbLanePlace south = m.kerbLaneBeside(onL0[17]);
    CHECK(south.valid());
    if (!south.valid()) return;
    const Path& southPath = m.laneLevel.paths[south.path.index];
    CHECK(!southPath.alongSpine() && samePlace(south.at, Vec2(-1.75, -144.1)));
    CHECK(approxEqual(south.along, southPath.length - 5.9, kWrittenStation));
    // a station of the street where no lane path runs (inside the junction's area, past the stop line): none
    CHECK(!m.kerbLaneAt(l0, 1, 145.0).valid());
    // the one placing of a station on a path (what the regulations use too)
    CHECK(approxEqual(m.alongPathAt(first.path, 50.0), 50.0, kWrittenStation) && m.alongPathAt(first.path, 500.0) == firstPath.length);

    // THE SPACE NEAREST A POINT: 6 m east of l0's centre line, level with station 50 (y = -100). The spaces' middles on
    // that kerb are at y = -144.1 + 7 k; the nearest is k = 6 at (4.75, -102.1), 1.25 m across and 2.1 m along.
    const KerbSpaceNear near = m.nearestKerbSpace(Vec2(6, -100), 5.0);
    CHECK(near.space.valid() && m.kerbSpaceName(near.space) == "l0 along 6");
    CHECK(approxEqual(near.distance, std::sqrt(1.25 * 1.25 + 2.1 * 2.1), kSamePoint) && samePlace(near.at, Vec2(4.75, -102.1)));
    CHECK(!m.nearestKerbSpace(Vec2(6, -100), 2.0).space.valid());        // none within 2 m
    CHECK(!m.nearestKerbSpace(Vec2(100, 60), 30.0).space.valid());       // beside the arterial, far from every local street

    // THE PLANNED LAYER STATES THEM: the same runs with no road built; only the kerb lane needs the built layer
    const RoadModel planned = plannedRoadModel(r->graph);
    CHECK(planned.kerbRuns.size() == 8u && kerbSpaceCounts(planned).spaces == 140u);
    CHECK(planned.kerbSpacePose(planned.kerbSpacesOf(l0)[0], pose) && samePlace(pose.at, Vec2(4.75, -144.1)));
    CHECK(!planned.kerbLaneBeside(planned.kerbSpacesOf(l0)[0]).valid());

    // SAVED AND READ BACK: the runs are in the model's bytes (the format is 13)
    {
        bundle::BinWriter w;
        putRoadModel(w, m);
        bundle::BinReader rd(w.bytes.data(), w.bytes.size());
        RoadModel back;
        std::string err;
        CHECK(getRoadModel(rd, back, &err));
        CHECK(back.kerbRuns.size() == 8u);
        for (std::size_t i = 0; i < back.kerbRuns.size() && i < m.kerbRuns.size(); ++i) {
            const ModelKerbRun& a = back.kerbRuns[i];
            const ModelKerbRun& b = m.kerbRuns[i];
            CHECK(a.street == b.street && a.dir == b.dir && a.from == b.from && a.spaceLength == b.spaceLength && a.count == b.count);
        }
        RoadModel fewer = m;
        fewer.kerbRuns.pop_back();
        CHECK(!(fewer == m));
    }
    // the dump lists each run; the lines for a person list each space
    CHECK(dumpRoadModel(m).find("kerb parking: street l0, the kerb of the traffic running along the spine, 17 spaces of 7.00 m from station 2.40 to 121.40\n") !=
          std::string::npos);
    bool listed = false;
    for (const std::string& line : kerbSpaceLines(m)) listed = listed || line.find("  space l0 against 16: stations 114.40 to 121.40") == 0;
    CHECK(listed);

    // A ONE-WAY STREET THAT PARKS has spaces on the kerb its traffic runs beside, and none on the other. The scene's
    // one-way street o0 (down x = 200 from a dead end at (200, 150) into the T at (200, 0)) is given a 12 m lane with a
    // 2.5 m band. Its traffic runs south along its spine, so its kerb is on the west side: x = 200 - 4.75.
    {
        nlohmann::json g = laneLevelGraph();
        g["classes"]["oneway"]["w"] = 12.0;
        g["classes"]["oneway"]["parking"] = 2.5;
        const std::unique_ptr<Result> wide = build(RoadLabGraph::fromJson(g));
        const RoadModel w = assembleRoadModel(*wide, oneCar());
        const ModelStreetId o0 = streetNamed(w, "o0");
        const std::vector<KerbSpace> onO0 = w.kerbSpacesOf(o0);
        CHECK(w.streets[o0.index].section.oneWay() && w.streets[o0.index].section.parking == 2.5);
        CHECK(onO0.size() == 17u);
        for (const KerbSpace& s : onO0) CHECK(s.dir == 1);
        if (onO0.size() != 17u) return;
        CHECK(w.kerbSpacePose(onO0[0], pose) && samePlace(pose.at, Vec2(195.25, 144.1)) && samePlace(pose.facing, Vec2(0, -1)));
        // its one lane lies beside both kerbs: asked for either, the same lane path
        const KerbLanePlace own = w.kerbLaneAt(o0, 1, 50.0), other = w.kerbLaneAt(o0, -1, 50.0);
        CHECK(own.valid() && other.valid() && own.path == other.path && approxEqual(own.along, other.along, kWrittenStation));
        // ...but a two-way street's kerb is beside the lane running that way only
        CHECK(w.kerbLaneAt(streetNamed(w, "l0"), 1, 50.0).path != w.kerbLaneAt(streetNamed(w, "l0"), -1, 50.0).path);
    }
}

// ---- 25. from a building's door to a place on the road (ADR-0439) ------------------------------------------------------

TEST_CASE(road_model_25_where_a_door_fronts_the_road_is_asked_of_the_model) {
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(laneLevelGraph()));
    const RoadModel m = assembleRoadModel(*r, oneCar());
    const ModelStreetId l0 = streetNamed(m, "l0"), a1 = streetNamed(m, "a1");
    CHECK(l0.valid() && a1.valid());
    if (!l0.valid() || !a1.valid()) return;

    // A DOOR 9 m EAST OF l0 (which runs north up x = 0 from (0, -150)), level with its station 50: it fronts l0 there,
    // on the kerb of the traffic running along the spine (the east side).
    const RoadFrontage f = m.frontageAt(Vec2(9, -100));
    CHECK(f.valid() && f.street == l0 && !f.junction.valid() && f.dir == 1);
    CHECK(approxEqual(f.station, 50.0, kSamePoint) && approxEqual(f.distance, 9.0, kSamePoint));
    CHECK(samePlace(f.on, Vec2(0, -100)) && samePlace(f.towards, Vec2(1, 0)));
    // the nearest place to walk: the middle of the 5 m sidewalk beyond the 6 m of carriageway
    Vec2 walk;
    CHECK(m.sidewalkBeside(f, walk) && samePlace(walk, Vec2(8.5, -100)));
    // set down in the kerb lane beside it: the lane running north, driven 1.75 m from the centre line (clear of the
    // parking band, ADR-0450), 50 m along its first path
    const KerbLanePlace drop = m.kerbLaneBeside(f);
    CHECK(drop.valid() && approxEqual(drop.along, 50.0, kWrittenStation) && samePlace(drop.at, Vec2(1.75, -100)));
    // THE KERB SPACES NEAR IT, BY WALKING DISTANCE. On its own kerb the spaces' middles are at stations 5.9 + 7 k: 47.9
    // (2.1 m away), 54.9 (4.9), 40.9 (9.1), 61.9 (11.9). The space opposite the nearest is 2.1 m along and the 12 m of
    // road across: 14.1. Within 15 m on foot, those five in that order.
    const std::vector<KerbSpaceNear> near = m.kerbSpacesNear(f, 15.0, 10);
    const char* names[] = {"l0 along 6", "l0 along 7", "l0 along 5", "l0 along 8", "l0 against 6"};
    const Real walks[] = {2.1, 4.9, 9.1, 11.9, 14.1};
    CHECK(near.size() == 5u);
    for (std::size_t i = 0; i < near.size() && i < 5; ++i) {
        CHECK(m.kerbSpaceName(near[i].space) == names[i]);
        CHECK(approxEqual(near[i].distance, walks[i], kSamePoint));
    }
    if (near.size() == 5u) CHECK(samePlace(near[0].at, Vec2(4.75, -102.1)) && samePlace(near[4].at, Vec2(-4.75, -102.1)));
    CHECK(m.kerbSpacesNear(f, 15.0, 2).size() == 2u);
    CHECK(m.kerbSpacesNear(f, 2.0, 10).empty());
    // THE SAME FROM STATED FACTS, with no search: the street, the station and the kerb
    const RoadFrontage stated = m.frontageOn(l0, 50.0, 1);
    CHECK(stated.valid() && stated.street == l0 && stated.dir == 1 && samePlace(stated.on, f.on) && samePlace(stated.towards, f.towards));
    CHECK(m.kerbLaneBeside(stated).path == drop.path && m.kerbSpacesNear(stated, 15.0, 10).size() == 5u);
    CHECK(!m.frontageOn(l0, 50.0, 0).valid() && !m.frontageOn(ModelStreetId(), 50.0, 1).valid());
    // the door across the road fronts the other kerb
    const RoadFrontage west = m.frontageAt(Vec2(-9, -100));
    CHECK(west.valid() && west.street == l0 && west.dir == -1 && samePlace(west.towards, Vec2(-1, 0)));
    CHECK(m.sidewalkBeside(west, walk) && samePlace(walk, Vec2(-8.5, -100)));
    CHECK(samePlace(m.kerbLaneBeside(west).at, Vec2(-1.75, -100)));

    // A DOOR ON THE ARTERIAL, 20 m north of a1 (which runs east along y = 0) at its station 100. Traffic running along
    // a1's spine keeps to the south side, so the door is on the kerb of the traffic running against it. The arterial
    // has no kerb parking; the nearest spaces are round the corner, each 100 m along a1 to a junction and then 32.1 m
    // up or down a local street to the middle of its nearest space (l0's and l2's last, l1's first), on either kerb.
    const RoadFrontage onArterial = m.frontageAt(Vec2(100, 20));
    CHECK(onArterial.valid() && onArterial.street == a1 && onArterial.dir == -1 && approxEqual(onArterial.station, 100.0, kSamePoint));
    CHECK(m.sidewalkBeside(onArterial, walk) && samePlace(walk, Vec2(100, 13.5)));
    const std::vector<KerbSpaceNear> round = m.kerbSpacesNear(onArterial, 135.0, 20);
    CHECK(round.size() == 6u);
    std::set<std::string> found;
    for (const KerbSpaceNear& n : round) {
        CHECK(approxEqual(n.distance, 132.1, kSamePoint));
        found.insert(m.kerbSpaceName(n.space));
    }
    CHECK(found == (std::set<std::string>{"l0 along 16", "l0 against 16", "l1 along 0", "l1 against 0", "l2 along 16", "l2 against 16"}));
    CHECK(m.kerbSpacesNear(onArterial, 130.0, 20).empty());

    // THE SAME ANSWER AS THE LOT PASS. A hand-made model: a narrow freeway along y = 20 and a wide local street along
    // y = 60 (10 m of carriageway and 5 m of sidewalk each side). A building between them stands nearer the freeway's
    // centre line, but no building fronts a freeway: the lot pass faces its door at the street
    // (LotRoads::nearestFrontable), and the model answers the street too, by the same search over the same lines.
    // (The nearest centre line of ANY road would be the freeway's.)
    RoadModel hand;
    auto street = [&](const char* id, RoadClass klass, Real y, Real laneWidth, int lanes, Real sidewalk) {
        ModelStreet s;
        s.id = s.road = id;
        s.from = 0;
        s.to = 200;
        s.section.klass = klass;
        s.section.lanesForward = s.section.lanesBackward = lanes;
        s.section.laneWidth = laneWidth;
        s.section.sidewalk = sidewalk;
        s.line.first = static_cast<uint32_t>(hand.linePoints.size());
        s.line.count = 2;
        hand.linePoints.push_back(Vec2(0, y));
        hand.linePoints.push_back(Vec2(200, y));
        hand.lineStations.push_back(0);
        hand.lineStations.push_back(200);
        hand.streets.push_back(s);
    };
    street("fw", RoadClass::Freeway, 20, 3.0, 1, 0);
    street("st", RoadClass::Local, 60, 10.0, 1, 5);
    const RoadFrontage between = hand.frontageAt(Vec2(100, 30));   // 10 m from the freeway's line, 30 m from the street's
    CHECK(between.valid() && between.street == streetNamed(hand, "st") && approxEqual(between.distance, 30.0, kSamePoint));
    CHECK(between.dir == 1 && approxEqual(between.station, 100.0, kSamePoint));   // (south of an eastward spine: its own traffic's kerb)
    CHECK(hand.roadLines().size() == 2u);
    const LotRoadInputs roads = lotRoadInputs(hand, RoadDeckField());
    CHECK(roads.roads.size() == 2u && roads.roads[0].id == "fw" && roads.roads[1].id == "st");
    // one block from 4 m off the freeway's centre line (y = 24) to the back of the street's sidewalk (y = 45)
    const std::vector<Poly2> blocks = {{{20, 24}, {180, 24}, {180, 45}, {20, 45}}};
    nlohmann::json cityJson = {{"seed", 9}, {"sidewalk", 5.0}, {"buildChance", 1.0}};
    LotParams lp;
    readLotGrowParams(LotCitysim::of(cityJson), lp);
    lp.roadMargin = 0;
    LotPlanDebug plan;
    int faced = 0, nearerFreeway = 0, doorsStated = 0;
    for (const LotBuilding& b : growLotBuildings(blocks, lp, &plan, nullptr, &roads, 0.0)) {
        if (b.units.empty()) continue;
        // WHAT THE LOT PASS RECORDS (ADR-0474): the street the building was turned to face, and each door's street,
        // station and kerb, the same answer the model gives for the door's foot (both by frontageOnLine)
        CHECK(b.frontage.street == "st" && b.frontage.dir == 1 && !b.frontage.onWalkway());
        for (const BuildingUnit& u : b.units)
            for (const DoorSpec& d : u.doors) {
                const RoadFrontage at = hand.frontageAt(d.foot);
                CHECK(at.valid() && at.street.valid() && d.frontage.street == hand.streets[at.street.index].id);
                CHECK(approxEqual(d.frontage.station, at.station, kSamePoint) && d.frontage.dir == at.dir && !d.frontage.onWalkway());
                ++doorsStated;
            }
        const RoadFrontage front = hand.frontageAt(b.site);
        CHECK(front.valid() && front.street == streetNamed(hand, "st"));
        if (!front.valid()) continue;
        const Vec2 toStreet = normalize(front.on - b.site);
        const Vec3 face = b.units.front().params.faceDir;
        CHECK(approxEqual(static_cast<Real>(face.x), toStreet.x, kSamePoint) && approxEqual(static_cast<Real>(face.z), toStreet.y, kSamePoint));
        ++faced;
        nearerFreeway += b.site.y < 40.0 ? 1 : 0;   // (y = 40 is halfway between the two centre lines)
    }
    std::printf("    [frontage] %d buildings face the street the model answers, %d of them nearer the freeway's centre line; %d doors "
                "record the model's street, station and kerb\n",
                faced, nearerFreeway, doorsStated);
    CHECK(faced >= 2 && nearerFreeway >= 1 && doorsStated >= faced);
}

// ---- 26. the model's kerb spaces as places the agent brain can hold (ADR-0440) -------------------------------------------

TEST_CASE(road_model_26_kerb_spaces_are_handed_to_the_agent_brain_as_places) {
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(laneLevelGraph()));
    const RoadModel m = assembleRoadModel(*r, oneCar());
    constexpr uint32_t kTag = 0x40000000u;   // what tells these handles from a host's own (its doors)
    citysim::brain::RoadPlaces places(m, kTag);
    // THE SPACES NEAR THE DOOR OF TEST 25 (9 m east of l0, level with its station 50), in the model's walking order
    const std::vector<citysim::brain::PlaceRef> near = places.spacesNear(Vec2(9, -100), 15.0, 10);
    const char* names[] = {"l0 along 6", "l0 along 7", "l0 along 5", "l0 along 8", "l0 against 6"};
    CHECK(near.size() == 5u);
    if (near.size() != 5u) return;
    for (std::size_t i = 0; i < 5; ++i) {
        KerbSpace space;
        CHECK(places.owns(near[i]) && places.spaceOf(near[i], space) && m.kerbSpaceName(space) == names[i]);
        CHECK(near[i].id == (kTag | static_cast<uint32_t>(i)));   // numbered as they are handed out, under the tag
    }
    // where a car in the nearest stands: the parking band's middle, 4.75 m from the centre line, at station 47.9
    Vec2 at;
    CHECK(places.positionOf(near[0], at) && samePlace(at, Vec2(4.75, -102.1)));
    // the kerb lane beside it, as the driving is told it: the lane running north, driven 1.75 m from the centre line
    // (ADR-0450), 47.9 m along
    driving::PathPoint lane;
    CHECK(places.kerbLane(near[0], lane));
    KerbSpace nearest;
    places.spaceOf(near[0], nearest);
    const KerbLanePlace models = m.kerbLaneBeside(nearest);
    CHECK(models.valid() && lane.path == models.path && approxEqual(lane.along, 47.9, kWrittenStation));
    CHECK(samePlace(m.laneLevel.pointAt(lane.path, lane.along), Vec2(1.75, -102.1)));
    // the same space asked again is the same handle; a handle that is not one of these is refused
    CHECK(places.placeOf(nearest) == near[0]);
    CHECK(places.spacesNear(Vec2(9, -100), 15.0, 10) == near);
    citysim::brain::PlaceRef foreign;
    foreign.id = 3;
    CHECK(!places.owns(foreign) && !places.positionOf(foreign, at) && !places.kerbLane(foreign, lane));
    foreign.id = kTag | 99u;
    CHECK(!places.owns(foreign));
}

// THE CROSSWALKS ARE THE ROAD MODEL'S (ADR-0452): stated once, by the planned layer, across every marked or paved arm of
// traffic_lab's stated junctions; the paint draws its zebras from them; the lane level lays them, and the road build's
// crossing finder finds the connectors over each. Checked against the scene's own facts and against what the paint drew.
namespace {
// The two kerb ends lie the crosswalk's length apart, to this (m).
constexpr Real kKerbEndOnEdge = 0.05;
// Halfway between them is the street's centre line, to this (m): the model keeps a street's line to within 0.1 m of its
// spine (road_model.cpp kLineTolerance), and the band's middle is placed on the spine.
constexpr Real kMiddleOnLine = 0.12;
// A painted zebra's stripes lie inside the kerb-to-kerb span, on its line, to this (m).
constexpr Real kZebraOnLine = 0.05;
}  // namespace
TEST_CASE(road_model_27_the_crosswalks_are_stated_once_and_the_paint_and_the_lane_level_read_them) {
    const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/traffic_lab_lanelab.json";
    const std::unique_ptr<Result> r =
        build(RoadLabGraph::fromJson(nlohmann::json::parse(slurp(path)), path.substr(0, path.find_last_of('/'))));
    RoadPaint paint;
    buildMeshes(*r, &paint);
    const RoadModel m = assembleRoadModel(*r, oneCar());
    for (const std::string& s : m.notStated) CHECK(s.find("crosswalks") == std::string::npos);   // stated now
    CHECK(!m.crosswalks.empty());
    // at a lit junction, a stop-signed one (an arm with a stop sign) and a plain one (none)
    std::map<std::string, uint32_t> byJunctionControl;
    std::array<uint32_t, kCrosswalkControlCount> byControl{};
    Real shortest = 1e9, longest = 0, worstEdge = 0, worstBand = 0;
    uint32_t laid = 0, crossed = 0, inAnotherRoad = 0;
    for (uint32_t id = 0; id < m.crosswalks.size(); ++id) {
        const ModelCrosswalk& c = m.crosswalks[id];
        const ModelJunction& J = m.junctions[c.junction.index];
        const ModelArm& arm = J.arms[c.arm];
        ++byJunctionControl[J.control];
        ++byControl[static_cast<std::size_t>(c.control)];
        // its control is the junction's and the arm's stated facts
        if (J.control == "signal") CHECK(c.control == CrosswalkControl::Lights);
        else CHECK(c.control == (arm.regulations.empty() ? CrosswalkControl::None : CrosswalkControl::Signs));
        // its band: kCrosswalkNear to kCrosswalkFar back from the arm's stated box edge, toward the approach
        const Real back = arm.spineIn ? -1 : 1;
        worstBand = std::max(worstBand, std::fabs(c.nearStation - (arm.boxEdge + back * kCrosswalkNear)));
        worstBand = std::max(worstBand, std::fabs(c.farStation - (arm.boxEdge + back * kCrosswalkFar)));
        // kerb to kerb: the street's carriageway, its two ends that far apart, each on an edge
        const ModelStreet* s = m.streetOf(arm);
        CHECK(s != nullptr);
        if (!s) continue;
        CHECK(approxEqual(c.length, 2 * s->section.carriagewayHalfWidth(), kSamePoint));
        CHECK(approxEqual((c.ends[1] - c.ends[0]).length(), c.length, kKerbEndOnEdge));
        const std::vector<Vec2> line = m.centreLine(s->line);
        worstEdge = std::max(worstEdge, distToPolyline(c.middle(), line, false));
        shortest = std::min(shortest, c.length);
        longest = std::max(longest, c.length);
        CHECK(!c.pavements || s->section.sidewalk > 0);   // (a kerb end in another road's carriageway: not one people use)
        inAnotherRoad += s->section.sidewalk > 0 && !c.pavements ? 1u : 0u;
        // laid in the lane level, and crossed by the connectors there
        laid += c.laneCrosswalk != kNoIndex ? 1u : 0u;
        crossed += m.crossersOf(id).empty() ? 0u : 1u;
    }
    std::printf("      %zu crosswalks: %u under lights, %u at signs, %u with neither; by junction control:", m.crosswalks.size(),
                byControl[0], byControl[1], byControl[2]);
    for (const std::pair<const std::string, uint32_t>& k : byJunctionControl) std::printf(" %s %u;", k.first.c_str(), k.second);
    std::printf("\n      %.1f to %.1f m kerb to kerb; the middle off the centre line by at most %.3f m; band off by at most %.4f m; "
                "%u laid, %u crossed; %u with a kerb end in another road (not used by people)\n", shortest, longest, worstEdge,
                worstBand, laid, crossed, inAnotherRoad);
    CHECK(inAnotherRoad < m.crosswalks.size() / 10);   // (the spread junctions' few)
    // (traffic_lab's 20 "unmarked" junctions are two-arm carry-ons and freeway joints: each of their roads begins at the
    // mouth, so a band would lie within kPaintRoadEndMargin of its end, and none is stated, as the paint drew none. The
    // plain crosswalks -- no lights, no sign -- are the through road's at its stop-signed junctions.)
    for (const char* control : {"signal", "stop", "all_way_stop"}) CHECK(byJunctionControl[control] > 0u);
    CHECK(byJunctionControl["unmarked"] == 0u);
    for (const uint32_t n : byControl) CHECK(n > 0u);
    CHECK(worstEdge <= kMiddleOnLine);
    CHECK(worstBand <= kWrittenStation);
    CHECK(shortest > 0 && longest < 30.0);
    CHECK(laid == m.crosswalks.size());
    CHECK(crossed == m.crosswalks.size());
    // THE PAINT DRAWS THE MARKED ONES, each across its crosswalk: its stripes from kerb to kerb, on the line between the
    // kerb ends, inside them
    uint32_t marked = 0;
    for (const ModelCrosswalk& c : m.crosswalks) marked += c.marked ? 1u : 0u;
    CHECK(paint.zebras.size() == marked);
    Real worstOff = 0, shortestShare = 1;
    for (const PaintedZebra& z : paint.zebras) {
        CHECK(z.crosswalk < m.crosswalks.size());
        if (z.crosswalk >= m.crosswalks.size()) continue;
        const ModelCrosswalk& c = m.crosswalks[z.crosswalk];
        CHECK(c.marked);
        for (const Vec2& p : {z.from, z.to}) {
            worstOff = std::max(worstOff, distToSegment(p, c.ends[0], c.ends[1]));
            const Real t = dot(p - c.ends[0], c.across());
            CHECK(t > 0 && t < c.length);
        }
        shortestShare = std::min(shortestShare, (z.to - z.from).length() / c.length);
    }
    std::printf("      %zu zebras painted (of %u marked): stripes off the kerb-to-kerb line by at most %.3f m; the shortest spans "
                "%.0f%% of its crosswalk\n", paint.zebras.size(), marked, worstOff, shortestShare * 100);
    CHECK(worstOff <= kZebraOnLine);
    CHECK(shortestShare > 0.5);
}

// ---- 28. where people on foot can walk: corners, dead ends, sidewalk kerbs, the path users (ADR-0464) -------------------

namespace {
// A T (laid out by the one-off tool, as a planned file is): a local street east from (0, 0), one west into it, and one
// from the south into it; `southSidewalks` is the south road's "sidewalks" statement (empty: none stated). With `spur`,
// a local road from (50, -60) north that stops 6 m short of the east road's centre line, inside its paved band, with no
// junction stated: one street, its `from` a dead end, its `to` meeting another road.
nlohmann::json walkTee(const std::string& southSidewalks = std::string(), bool spur = false) {
    nlohmann::json g;
    g["name"] = "walk_tee";
    auto road = [](const std::string& id, const Vec2& a, const Vec2& b) {
        return nlohmann::json{{"id", id}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}};
    };
    g["edges"] = nlohmann::json::array({road("east", Vec2(0, 0), Vec2(100, 0)), road("west", Vec2(-100, 0), Vec2(0, 0)),
                                        road("south", Vec2(0, -100), Vec2(0, 0))});
    if (!southSidewalks.empty()) g["edges"][2]["sidewalks"] = southSidewalks;
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    if (spur) {
        g["edges"].push_back(road("spur", Vec2(50, -60), Vec2(50, -6)));
        g["streets"].push_back({{"id", "spur/1"}, {"road", "spur"}, {"from", 0.0}, {"to", 54.0}, {"junctions", {nullptr, nullptr}}});
    }
    return g;
}
// A DIVIDED ROAD CROSSING A STREET, spread over two stated points, as a planned file states one: the street `ew` runs
// east-west through it (its stretch between the points, stations 96 to 104, lies inside the junction); the divided road's
// one-way carriageways cross it at x = 4 (northbound, `nb_s` in and `nb_n` out) and x = -4 (southbound). Point A at (4, 0)
// holds the street's station 104 and the northbound's two ends, point B at (-4, 0) its station 96 and the southbound's.
// The ring, counter-clockwise from east: east (ew out), nb_n, sb_n, west (ew in), sb_s, nb_s. `nbsAt`: the station point A
// states for nb_s's end (100, its end; another: an arm no point holds).
nlohmann::json walkSpread(double nbsAt = 100.0) {
    nlohmann::json g;
    g["name"] = "walk_spread";
    g["classes"]["oneway"] = {{"w", 3.5}, {"fwd", 1}, {"back", 0}, {"sidewalk", 2.0}, {"g_max", 0.09}, {"window", 40}, {"thick", 0.5}};
    struct R { const char* id; const char* cls; Vec2 a, b; };
    const R roads[] = {{"ew", "local", Vec2(-100, 0), Vec2(100, 0)},
                       {"nb_n", "oneway", Vec2(4, 0), Vec2(4, 100)},   {"nb_s", "oneway", Vec2(4, -100), Vec2(4, 0)},
                       {"sb_n", "oneway", Vec2(-4, 100), Vec2(-4, 0)}, {"sb_s", "oneway", Vec2(-4, 0), Vec2(-4, -100)}};
    nlohmann::json edges = nlohmann::json::array();
    for (const R& r : roads)
        edges.push_back({{"id", r.id}, {"class", r.cls}, {"path", {{"type", "polyline"}, {"points", {{r.a.x, r.a.y}, {r.b.x, r.b.y}}}}}});
    g["edges"] = edges;
    const auto street = [](const std::string& id, const char* road, double from, double to, bool atFrom) {
        return nlohmann::json{{"id", id}, {"road", road}, {"from", from}, {"to", to},
                              {"junctions", atFrom ? nlohmann::json::array({"j", nullptr}) : nlohmann::json::array({nullptr, "j"})}};
    };
    g["streets"] = nlohmann::json::array({street("west", "ew", 0, 96, false), street("east", "ew", 104, 200, true),
                                          street("nb_n", "nb_n", 0, 100, true), street("nb_s", "nb_s", 0, 100, false),
                                          street("sb_n", "sb_n", 0, 100, false), street("sb_s", "sb_s", 0, 100, true)});
    auto arm = [](const char* road, double at, bool in, const char* street) {
        return nlohmann::json{{"road", road}, {"at", at}, {"spine", in ? "in" : "out"}, {"street", street}};
    };
    nlohmann::json j = {{"id", "j"}, {"at", {0.0, 0.0}}};
    j["arms"] = nlohmann::json::array({arm("ew", 104, false, "east"), arm("nb_n", 0, false, "nb_n"), arm("sb_n", 100, true, "sb_n"),
                                       arm("ew", 96, true, "west"), arm("sb_s", 0, false, "sb_s"), arm("nb_s", 100, true, "nb_s")});
    j["inside"] = nlohmann::json::array({{{"road", "ew"}, {"from", 96.0}, {"to", 104.0}}});
    j["nodes"] = nlohmann::json::array(
        {{{"at", {4.0, 0.0}}, {"points", {{{"road", "ew"}, {"at", 104.0}}, {{"road", "nb_n"}, {"at", 0.0}}, {{"road", "nb_s"}, {"at", nbsAt}}}}},
         {{"at", {-4.0, 0.0}}, {"points", {{{"road", "ew"}, {"at", 96.0}}, {{"road", "sb_n"}, {"at", 100.0}}, {{"road", "sb_s"}, {"at", 0.0}}}}}});
    g["junctions"] = nlohmann::json::array({j});
    g["arms_in_ring"] = true;
    return g;
}
// A road file's planned layer, its ramps composed (as the loader reads it); `out`, the graph it was read from.
RoadModel walkModel(const nlohmann::json& file, RoadLabGraph& out) {
    out = RoadLabGraph::fromJson(file);
    (void)expand(out);
    return plannedRoadModel(out);
}
std::map<CornerGap, uint32_t> gapsOf(const std::vector<RingPair>& pairs) {
    std::map<CornerGap, uint32_t> out;
    for (const RingPair& p : pairs) ++out[p.gap];
    return out;
}
// WHICH KERBS A CORNER JOINS, checked against where they lie: the end of each arm's crosswalk on the kerb the corner
// names (end 0 is the +1 kerb's; a crosswalk's ends are laid by where the kerbs lie, not by the ring) lies inside the
// corner -- counter-clockwise from arm A's stated heading and short of arm B's, seen from the junction's stated point.
// True when the arms have no crosswalk people use or no stated headings (nothing to check: `checked` counts the rest).
bool cornerKerbsFaceIt(const RoadModel& m, const ModelCorner& c, uint32_t& checked) {
    const ModelJunction& J = m.junctions[c.junction.index];
    const ModelCrosswalk* a = m.crosswalkAcross(c.junction, c.armA);
    const ModelCrosswalk* b = m.crosswalkAcross(c.junction, c.armB);
    const Real hA = J.arms[c.armA].heading, hB = J.arms[c.armB].heading;
    if (!a || !b || !a->pavements || !b->pavements || !stated(hA) || !stated(hB)) return true;
    ++checked;
    const auto ccwFrom = [](Real from, Real to) { return std::fmod(std::fmod(to - from, 360.0) + 360.0, 360.0); };
    const Real sweep = ccwFrom(hA, hB);
    bool inside = true;
    for (const Vec2& p : {a->ends[c.kerbA > 0 ? 0 : 1], b->ends[c.kerbB > 0 ? 0 : 1]}) {
        const Real at = ccwFrom(hA, std::atan2(p.y - J.at.y, p.x - J.at.x) * 180.0 / 3.14159265358979);
        if (!(at > 0 && at < sweep)) {
            std::printf("        (junction %s, arms %u-%u: the kerb end (%.2f, %.2f) lies %.1f degrees round from arm %u, outside the "
                        "corner's %.1f)\n", J.id.c_str(), c.armA, c.armB, p.x, p.y, at, c.armA, sweep);
            inside = false;
        }
    }
    return inside;
}
}  // namespace

TEST_CASE(road_model_28_corners_dead_ends_and_sidewalk_kerbs_are_stated_once) {
    // A T: three corners round its ring, each naming the kerbs that face it (checked against the crosswalks' own kerb
    // ends, which are laid by where the kerbs lie, not by the ring); its three far ends are dead ends, joined
    // across by the shipped table's join
    {
        RoadLabGraph g;
        const RoadModel m = walkModel(walkTee(), g);
        CHECK(m.armsInRing && m.junctions.size() == 1u && m.junctions[0].arms.size() == 3u);
        CHECK(m.corners.size() == 3u);
        uint32_t checked = 0;
        for (const ModelCorner& c : m.corners) {
            CHECK(c.armB == (c.armA + 1) % 3);
            CHECK(cornerKerbsFaceIt(m, c, checked));
        }
        CHECK(checked == 3u);
        CHECK(m.deadEnds.size() == 3u);
        for (const ModelDeadEnd& d : m.deadEnds) CHECK(d.turnRound);
        // the one derivation: the stated corners are the ring pairs with no gap, and the dead ends deadEndsOf's
        CHECK(gapsOf(ringPairs(m, g))[CornerGap::None] == 3u);
        // with no dead-end join in the table, no dead end is joined (the rule is the data's)
        PathUsers noJoin;
        std::string why;
        CHECK(parsePathUsers(nlohmann::json{{"movers", {{"walker", nlohmann::json::object()}}}, {"kinds", {{"sidewalk", {"walker"}}}},
                                            {"dead_end", {{"joined_by", nullptr}}}},
                             noJoin, &why));
        const std::vector<ModelDeadEnd> unjoined = deadEndsOf(m, g, noJoin);
        CHECK(unjoined.size() == 3u);
        for (const ModelDeadEnd& d : unjoined) CHECK(!d.turnRound);
        // saved and read back whole
        bundle::BinWriter w;
        putRoadModel(w, m);
        bundle::BinReader rd(w.bytes.data(), w.bytes.size());
        RoadModel back;
        CHECK(getRoadModel(rd, back, &why) && back == m && back.corners.size() == 3u && back.deadEnds.size() == 3u);
        for (std::size_t k = 0; k < back.corners.size() && k < m.corners.size(); ++k)
            CHECK(back.corners[k].kerbA == m.corners[k].kerbA && back.corners[k].kerbB == m.corners[k].kerbB);
        // A KERB FLIPPED IS CAUGHT: the check above bites
        ModelCorner flipped = m.corners.front();
        flipped.kerbA = -flipped.kerbA;
        uint32_t again = 0;
        CHECK(!cornerKerbsFaceIt(m, flipped, again));
    }
    // A SIDEWALK ON ONE KERB of the south road ("along": the kerb of the traffic running north along its spine): the
    // corner on its other kerb is not stated (no walk there), the one on that kerb is; its crosswalk is still reached; its
    // dead end is not joined across (only one sidewalk comes to it); the model says the build still paves both kerbs
    {
        RoadLabGraph g;
        const RoadModel m = walkModel(walkTee("along"), g);
        uint32_t southArm = kNoIndex;
        for (uint32_t a = 0; a < m.junctions[0].arms.size(); ++a)
            if (m.junctions[0].arms[a].road == "south") southArm = a;
        CHECK(southArm != kNoIndex);
        if (southArm == kNoIndex) return;
        const ModelStreet* s = m.streetOf(m.junctions[0].arms[southArm]);
        CHECK(s && s->section.sidewalkOnKerb(+1) && !s->section.sidewalkOnKerb(-1) && s->section.pavementOnKerb(+1) &&
              !s->section.pavementOnKerb(-1));
        CHECK(m.corners.size() == 2u);
        for (const RingPair& p : ringPairs(m, g)) {
            const int southKerb = p.armA == southArm ? p.kerbA : p.armB == southArm ? p.kerbB : 0;
            CHECK((p.gap == CornerGap::NoPavement) == (southKerb == -1));
        }
        const ModelCrosswalk* cw = m.crosswalkAcross(ModelJunctionId(0), southArm);
        CHECK(cw && cw->pavements);
        uint32_t joined = 0, ends = 0;
        for (const ModelDeadEnd& d : m.deadEnds) {
            ++ends;
            joined += d.turnRound ? 1u : 0u;
            if (m.streets[d.street.index].road == "south") CHECK(!d.turnRound);
        }
        CHECK(ends == 3u && joined == 2u);
        bool said = false;
        for (const std::string& p : m.problems) said = said || p.find("road south: it states a sidewalk on fewer than both kerbs") != std::string::npos;
        CHECK(said);
        // none at all: no corner touches the south arm, and nobody reaches its crosswalk
        const RoadModel none = walkModel(walkTee("none"), g);
        CHECK(none.corners.size() == 1u);
        const ModelCrosswalk* cn = none.crosswalkAcross(ModelJunctionId(0), southArm);
        CHECK(!cn || !cn->pavements);
        // a word the file does not know is refused when read
        bool refused = false;
        try {
            (void)RoadLabGraph::fromJson(walkTee("left"));
        } catch (const std::exception& e) {
            refused = std::string(e.what()).find("sidewalks is \"left\"") != std::string::npos;
        }
        CHECK(refused);
    }
    // THE STREET ENDS, one answer: a spur stopping inside the east road's band with no junction stated meets another road
    // (the planner's to state, never a dead end), and its far end is a dead end
    {
        RoadLabGraph g;
        const RoadModel m = walkModel(walkTee(std::string(), true), g);
        const ModelStreetId spur = streetNamed(m, "spur/1");
        CHECK(spur.valid());
        if (!spur.valid()) return;
        CHECK(streetEndKind(g, m.streets[spur.index], 0) == StreetEndKind::DeadEnd);
        CHECK(streetEndKind(g, m.streets[spur.index], 1) == StreetEndKind::MeetsAnother);
        uint32_t spurEnds = 0;
        for (const ModelDeadEnd& d : m.deadEnds)
            if (d.street == spur) {
                ++spurEnds;
                CHECK(d.end == 0);
            }
        CHECK(spurEnds == 1u && m.deadEnds.size() == 4u);
        // and a street end at the junction is the junction's
        for (const ModelStreet& S : m.streets)
            for (int end = 0; end < 2; ++end)
                if (S.junctions[static_cast<std::size_t>(end)].valid()) CHECK(streetEndKind(g, S, end) == StreetEndKind::Junction);
    }
    // A SPREAD JUNCTION: corners only between arms at one stated point; the two pairs whose arms meet it at different
    // points get none (nothing states where the kerb runs between the points), and say so
    {
        RoadLabGraph g;
        const RoadModel m = walkModel(walkSpread(), g);
        const std::vector<RingPair> pairs = ringPairs(m, g);
        std::map<CornerGap, uint32_t> gaps = gapsOf(pairs);
        CHECK(pairs.size() == 6u && gaps[CornerGap::None] == 4u && gaps[CornerGap::ApartAtSpread] == 2u);
        CHECK(m.corners.size() == 4u);
        const std::vector<std::pair<std::string, std::string>> apart = {{"nb_n", "sb_n"}, {"sb_s", "nb_s"}};
        for (const RingPair& p : pairs)
            if (p.gap == CornerGap::ApartAtSpread) {
                const std::pair<std::string, std::string> roads(m.junctions[0].arms[p.armA].road, m.junctions[0].arms[p.armB].road);
                CHECK(std::find(apart.begin(), apart.end(), roads) != apart.end());
            }
        // an arm the stated points do not hold (nb_s's end stated 4 m off): its two pairs say so, and get no corner
        RoadLabGraph g2;
        const RoadModel off = walkModel(walkSpread(90.0), g2);
        gaps = gapsOf(ringPairs(off, g2));
        CHECK(gaps[CornerGap::ArmAtNoPoint] == 2u && gaps[CornerGap::None] == 3u && off.corners.size() == 3u);
        // an unmarked file's ring is not trusted: no corner at all
        nlohmann::json unmarked = walkSpread();
        unmarked.erase("arms_in_ring");
        RoadLabGraph g3;
        const RoadModel um = walkModel(unmarked, g3);
        CHECK(um.corners.empty() && gapsOf(ringPairs(um, g3))[CornerGap::RingNotStated] == 6u);
    }
    // WHO CAN WALK A CROSS-SECTION'S KERB: a footway, all of it; a track, none of it
    {
        ModelSection footway, track;
        footway.footway = true;
        track.track = true;
        track.sidewalk = 2.0;
        CHECK(footway.pavementOnKerb(+1) && footway.pavementOnKerb(-1) && !track.pavementOnKerb(+1) && !track.pavementOnKerb(-1));
    }
}

TEST_CASE(road_model_28b_who_may_use_which_kind_of_path_is_its_data_file) {
    const PathUsers& u = pathUsers();
    CHECK(u.usable);
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/data/path_users.json", std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    CHECK(!text.empty() && u.hash == bundle::fnv1aStr(text));
    // the file as read: what each mover is, and that the walking kinds are the walker's alone
    const PathMover* vehicles = u.mover("vehicles");
    const PathMover* walker = u.mover("walker");
    CHECK(vehicles && vehicles->vehicles && walker && !walker->vehicles);
    for (const char* kind : {"sidewalk", "corner", "crossing", "turn_round", "footway", "park_path"})
        CHECK(u.mayUse("walker", kind) && !u.mayUse("vehicles", kind));
    CHECK(!u.deadEndJoin.empty() && u.usersOf(u.deadEndJoin));
    CHECK(!u.usersOf("no_such_kind") && !u.mayUse("walker", "no_such_kind"));
    // EVERY LANE KIND THE BUILDER LAYS is in it, open to the fleet's vehicles and to nobody on foot (traffic_lab's lanes,
    // ramps and pockets among them)
    const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/traffic_lab_lanelab.json";
    const std::unique_ptr<Result> r =
        build(RoadLabGraph::fromJson(nlohmann::json::parse(slurp(path)), path.substr(0, path.find_last_of('/'))));
    const RoadModel m = builtRoadModel(*r);
    std::set<std::string> kinds;
    for (const ModelLane& L : m.lanes) kinds.insert(L.kind);
    CHECK(!kinds.empty());
    for (const std::string& k : kinds) {
        std::printf("      lane kind %s: %s\n", k.c_str(), u.usersOf(k) ? "listed" : "NOT LISTED");
        CHECK(u.mayUse("vehicles", k) && !u.mayUse("walker", k));
    }
    // A FILE THAT IS NOT ONE is refused, saying why
    std::string why;
    PathUsers bad;
    CHECK(!parsePathUsers(nlohmann::json::object(), bad, &why) && why.find("movers") != std::string::npos);
    CHECK(!parsePathUsers(nlohmann::json{{"movers", {{"walker", nlohmann::json::object()}}}, {"kinds", {{"sidewalk", {"cyclist"}}}}}, bad, &why) &&
          why.find("\"cyclist\"") != std::string::npos && !bad.usable);
    CHECK(!parsePathUsers(nlohmann::json{{"movers", {{"walker", nlohmann::json::object()}}}, {"kinds", {{"sidewalk", {"walker"}}}},
                                         {"dead_end", {{"joined_by", "hop"}}}},
                          bad, &why) &&
          why.find("\"hop\"") != std::string::npos);
    // a new mover is a row: a table naming one is read, and the kinds it lists are its own
    PathUsers withBikes;
    CHECK(parsePathUsers(nlohmann::json{{"movers", {{"walker", nlohmann::json::object()}, {"cyclist", nlohmann::json::object()}}},
                                        {"kinds", {{"footway", {"walker", "cyclist"}}, {"sidewalk", {"walker"}}}}},
                         withBikes, &why));
    CHECK(withBikes.mayUse("cyclist", "footway") && !withBikes.mayUse("cyclist", "sidewalk") && withBikes.deadEndJoin.empty());
}

// THE SHIPPED LEVELS' CORNERS AND DEAD ENDS (ADR-0464), measured: how many corners each junction ring gives, which ring
// pairs get none and why, the dead ends, and the street ends the planner left unstated. The spread junctions' gaps are
// the walking paths' risk (task 4): listed by junction.
TEST_CASE(road_model_29_the_shipped_levels_corners_and_dead_ends) {
    for (const char* level : {"traffic_lab", "river_town", "agent_lab"}) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/" + level + "_lanelab.json";
        RoadLabGraph g = RoadLabGraph::load(path);
        (void)expand(g);
        const RoadModel m = plannedRoadModel(g);
        const std::vector<RingPair> pairs = ringPairs(m, g);
        std::map<CornerGap, uint32_t> gaps = gapsOf(pairs);
        CHECK(gaps[CornerGap::None] == m.corners.size());
        CHECK(!m.corners.empty());
        CHECK(gaps[CornerGap::RingNotStated] == 0u && gaps[CornerGap::OneArm] == 0u);
        uint32_t spreadJunctions = 0;
        for (const ModelJunction& J : m.junctions) spreadJunctions += J.nodes.empty() ? 0u : 1u;
        std::printf("      %s: %zu junctions (%u spread over points), %zu ring pairs: %zu corners", level, m.junctions.size(), spreadJunctions,
                    pairs.size(), m.corners.size());
        for (uint8_t k = 1; k < kCornerGapCount; ++k)
            if (gaps[static_cast<CornerGap>(k)]) std::printf("; %u %s", gaps[static_cast<CornerGap>(k)], cornerGapWord(static_cast<CornerGap>(k)));
        std::printf("\n");
        // the spread junctions' gaps, junction by junction
        std::map<std::string, std::vector<std::string>> byJunction;
        for (const RingPair& p : pairs) {
            if (p.gap != CornerGap::ApartAtSpread && p.gap != CornerGap::ArmAtNoPoint && p.gap != CornerGap::NoStreet) continue;
            const ModelJunction& J = m.junctions[p.junction.index];
            byJunction[J.id].push_back(J.arms[p.armA].road + "-" + J.arms[p.armB].road + (p.gap == CornerGap::ApartAtSpread ? "" : p.gap == CornerGap::ArmAtNoPoint ? " (no point)" : " (no street)"));
        }
        if (!byJunction.empty()) {
            std::printf("        corners not stated at %zu junctions:", byJunction.size());
            for (const std::pair<const std::string, std::vector<std::string>>& j : byJunction) {
                std::printf(" %s [", j.first.c_str());
                for (std::size_t k = 0; k < j.second.size(); ++k) std::printf("%s%s", k ? ", " : "", j.second[k].c_str());
                std::printf("];");
            }
            std::printf("\n");
        }
        // every stated corner names the kerbs that face it: its arms' crosswalks' kerb ends on those kerbs lie inside it,
        // where both have a crosswalk people use (seen from the junction's stated point, which at a spread junction can lie
        // off the point the arms meet at: counted apart)
        uint32_t checked = 0, wrong = 0, wrongSpread = 0;
        for (const ModelCorner& c : m.corners)
            if (!cornerKerbsFaceIt(m, c, checked)) {
                ++wrong;
                wrongSpread += m.junctions[c.junction.index].nodes.empty() ? 0u : 1u;
            }
        std::printf("        %u corners checked against their crosswalks' kerb ends, %u with a kerb end outside the corner (%u at spread "
                    "junctions)\n", checked, wrong, wrongSpread);
        CHECK(checked > 0u);
        CHECK(wrong == wrongSpread);   // (at a compact junction, never)
        // the street ends
        std::map<StreetEndKind, uint32_t> ends;
        for (const ModelStreet& S : m.streets)
            for (int end = 0; end < 2; ++end) ++ends[streetEndKind(g, S, end)];
        uint32_t joined = 0;
        for (const ModelDeadEnd& d : m.deadEnds) joined += d.turnRound ? 1u : 0u;
        std::printf("        %zu dead ends (%u joined across); street ends: %u at junctions, %u carrying on, %u ramp anchors, %u meeting another "
                    "road unstated, %u stopping part way unstated\n", m.deadEnds.size(), joined, ends[StreetEndKind::Junction],
                    ends[StreetEndKind::CarriesOn], ends[StreetEndKind::RampAnchored], ends[StreetEndKind::MeetsAnother],
                    ends[StreetEndKind::PartWay]);
        CHECK(ends[StreetEndKind::DeadEnd] == m.deadEnds.size());
    }
}

// ---- 30. the walks the road build lays: sidewalk lanes and corner walks along the drawn pavement (ADR-0465) --------------

namespace {
// A point counts as on the drawn walk within this (m): the sidewalk's polygons are snapped to a millimetre grid, and the
// shrunk walk's edge is cut to the same grid.
constexpr Real kOnPavement = 0.05;
// THE DRAWN WALK of a build: its sidewalk layer and its footways' paving, grown by kOnPavement.
PolySet drawnWalk(const Result& r) {
    PolySet out = r.pavement.sidewalk;
    for (size_t li = 0; li < r.lanes.lanes.size(); ++li)
        if (r.lanes.lanes[li].kind == "footway" && li < r.pavement.footprints.size()) out = unionSets(out, r.pavement.footprints[li]);
    return offsetSet(out, kOnPavement);
}
// How many of a walk's points, and of the middles of its segments, lie off the drawn walk.
uint32_t offTheWalk(const Lane& w, const PreparedSet& walk) {
    uint32_t off = 0;
    const Span<Vec2> xy = w.xy();
    for (size_t k = 0; k < xy.size(); ++k) {
        off += walk.contains(xy[k]) ? 0u : 1u;
        if (k + 1 < xy.size()) off += walk.contains((xy[k] + xy[k + 1]) * 0.5) ? 0u : 1u;
    }
    return off;
}
// THE KERB END A CORNER'S END SITS ACROSS FROM on arm `arm` (its crosswalk's end on the corner's kerb), and the width of
// the walk there: false where the arm has no crosswalk.
bool kerbEndOf(const RoadModel& m, ModelJunctionId j, uint32_t arm, int kerb, Vec2& end, Real& width) {
    const ModelCrosswalk* c = m.crosswalkAcross(j, arm);
    const ModelStreet* s = m.streetOf(m.junctions[j.index].arms[arm]);
    if (!c || !s) return false;
    end = c->ends[kerb > 0 ? 0 : 1];
    width = s->section.footway ? s->section.travelWidth() : s->section.sidewalk;
    return true;
}
// A corner's two ends against its arms' crosswalk kerb ends: each lies on that kerb's sidewalk by it, no farther than the
// sidewalk is wide and kWalkFromKerb (beside a turn pocket the drawn kerb stands out from the stated carriageway's edge).
// `checked` counts the ends with a crosswalk to check against.
bool cornerEndsAtKerbEnds(const RoadModel& m, const ModelCorner& c, uint32_t& checked, Real* worst = nullptr) {
    if (c.walk >= m.walks.size()) return false;
    const ModelLane& w = m.walks[c.walk];
    const Vec2 ends[2] = {m.lanePoints()[w.curve.first], m.lanePoints()[w.curve.end() - 1]};
    bool ok = true;
    for (int k = 0; k < 2; ++k) {
        Vec2 kerbEnd;
        Real width = 0;
        if (!kerbEndOf(m, c.junction, k == 0 ? c.armA : c.armB, k == 0 ? c.kerbA : c.kerbB, kerbEnd, width)) continue;
        ++checked;
        const Real d = distance(ends[k], kerbEnd);
        if (worst) *worst = std::max(*worst, d - width);
        ok = ok && d <= width + kWalkFromKerb;
    }
    return ok;
}
}  // namespace

TEST_CASE(road_model_30_the_build_lays_the_walks_from_its_drawn_pavement) {
    // A STRAIGHT STREET: a sidewalk lane along each kerb, the middle of its sidewalk (the carriageway's half width and half
    // the sidewalk out), the +1 kerb's on the side the kerb side is (kerbPerpSign: the plan's -y of an eastbound spine), and
    // every point of each on the drawn sidewalk
    {
        nlohmann::json g;
        g["name"] = "walk_street";
        g["edges"] = nlohmann::json::array(
            {{{"id", "st"}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{0.0, 0.0}, {100.0, 0.0}}}}}}});
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        CHECK(r->lanes.walks.size() == 2u);
        const PreparedSet walk(drawnWalk(*r));
        const RoadClassSpec& local = r->graph.cls(r->graph.edges[0]);
        const Real middle = r->graph.carriagewayHalfWidth(r->graph.edges[0]) + local.sidewalk * 0.5;
        for (const Lane& w : r->lanes.walks) {
            CHECK(w.kind == "sidewalk" && w.onRoad() && (w.dir == 1 || w.dir == -1));
            const Real wantY = (w.dir > 0 ? 1.0 : -1.0) * roads::lanes::kerbPerpSign() * middle;   // (perp of +x is +y)
            for (const Vec2& p : w.xy()) CHECK(approxEqual(p.y, wantY, kSamePoint));
            CHECK(offTheWalk(w, walk) == 0u);
        }
        // the model takes them as its own walks, at the deck's height, on its street
        const RoadModel m = assembleRoadModel(*r, oneCar());
        CHECK(m.walks.size() == 2u && m.lanes.size() == r->lanes.lanes.size());
        for (const ModelLane& w : m.walks) CHECK(w.kind == "sidewalk");
        // NOTHING DRAWN READS THEM: the lane level's paths are its car lanes' alone, as before
        for (const Path& p : m.laneLevel.paths) CHECK(p.modelLane == kNoIndex || p.modelLane < m.lanes.size());
    }
    // A T: a corner walk for each of its three corners, every point of each on the drawn sidewalk (never across the
    // carriageway), each end on its kerb's sidewalk across from the crosswalk's kerb end
    {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(walkTee()));
        const PreparedSet walk(drawnWalk(*r));
        uint32_t corners = 0;
        for (const WalkCorner& wc : r->walkCorners) {
            CHECK(wc.lane >= 0 && wc.why.empty() && !wc.spread);
            if (wc.lane < 0) continue;
            ++corners;
            CHECK(offTheWalk(r->lanes.walks[static_cast<size_t>(wc.lane)], walk) == 0u);
        }
        CHECK(corners == 3u);
        const RoadModel m = assembleRoadModel(*r, oneCar());
        uint32_t checked = 0;
        for (const ModelCorner& c : m.corners) CHECK(cornerEndsAtKerbEnds(m, c, checked));
        CHECK(m.corners.size() == 3u && checked == 6u);
        CHECK(m.walks.size() == 3u * 2u + 3u);   // three roads' two sidewalks, three corners
        // A CORNER OFF ITS PAVEMENT IS CAUGHT: the check above bites (a walk moved 6 m across, into the carriageway)
        Lane moved = r->lanes.walks[static_cast<size_t>(r->walkCorners.front().lane)];
        std::shared_ptr<LaneCurves> shifted = std::make_shared<LaneCurves>();
        for (const Vec2& p : moved.xy()) shifted->points.push_back(p + Vec2(0, 6));
        shifted->along.assign(shifted->points.size(), 0);
        moved.curves = shifted;
        moved.curve = Range{0, static_cast<uint32_t>(shifted->points.size())};
        CHECK(offTheWalk(moved, walk) > 0u);
    }
    // ONE KERB'S SIDEWALK on the T's south road: one sidewalk lane there, and no corner walk on its bare kerb
    {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(walkTee("along")));
        uint32_t south = 0;
        for (const Lane& w : r->lanes.walks)
            if (w.onRoad() && r->graph.edges[w.edge].id == "south") {
                ++south;
                CHECK(w.dir == 1);
            }
        CHECK(south == 1u && r->walkCorners.size() == 2u);
        for (const WalkCorner& wc : r->walkCorners) CHECK(wc.lane >= 0);
    }
    // A SPREAD JUNCTION: the two pairs across its points get a walk along the drawn pavement between them -- round by the
    // street's sidewalk, not across the median the chord between their kerb ends would cut -- and so it has every corner
    {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(walkSpread()));
        const PreparedSet walk(drawnWalk(*r));
        uint32_t spread = 0, laid = 0;
        for (const WalkCorner& wc : r->walkCorners) {
            if (wc.lane < 0) {
                std::printf("      no walk at %s arms %u-%u: %s\n", r->graph.junctions[wc.junction.index].id.c_str(), wc.armA, wc.armB, wc.why.c_str());
                continue;
            }
            ++laid;
            const Lane& w = r->lanes.walks[static_cast<size_t>(wc.lane)];
            CHECK(offTheWalk(w, walk) == 0u);
            if (!wc.spread) continue;
            ++spread;
            const Span<Vec2> xy = w.xy();
            const Real chord = distance(xy[0], xy[xy.size() - 1]);
            const Real along = w.s()[xy.size() - 1];
            std::printf("      across the points: %u-%u, %.1f m along the pavement, %.1f m straight\n", wc.armA, wc.armB, along, chord);
            CHECK(along > chord + 1.0);   // (it goes round)
        }
        CHECK(laid == 6u && spread == 2u);
        const RoadModel m = assembleRoadModel(*r, oneCar());
        uint32_t walked = 0, across = 0;
        for (const ModelCorner& c : m.corners) {
            walked += c.walk != kNoIndex ? 1u : 0u;
            across += c.spread ? 1u : 0u;
        }
        CHECK(m.corners.size() == 6u && walked == 6u && across == 2u);
        // saved and read back whole, walks and all
        bundle::BinWriter wr;
        putRoadModel(wr, m);
        bundle::BinReader rd(wr.bytes.data(), wr.bytes.size());
        RoadModel back;
        std::string why;
        CHECK(getRoadModel(rd, back, &why) && back == m && back.walks.size() == m.walks.size());
    }
}

// THE SHIPPED LEVELS' WALKS (ADR-0465): every ring pair's corner, now with the drawn pavement's walk (the spread
// junctions' too), each on the drawn sidewalk and its ends at its arms' kerb ends; any left without one, with why.
TEST_CASE(road_model_31_the_shipped_levels_walks) {
    for (const char* level : {"traffic_lab", "river_town"}) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/" + level + "_lanelab.json";
        const std::unique_ptr<Result> r =
            build(RoadLabGraph::fromJson(nlohmann::json::parse(slurp(path)), path.substr(0, path.find_last_of('/'))));
        const RoadModel m = builtRoadModel(*r);
        const std::vector<RingPair> pairs = ringPairs(m, r->graph);
        uint32_t walkable = 0, laid = 0, laidSpread = 0, off = 0, offWalks = 0;
        for (const RingPair& p : pairs) walkable += p.gap == CornerGap::None || p.gap == CornerGap::ApartAtSpread ? 1u : 0u;
        const PreparedSet walk(drawnWalk(*r));
        std::map<std::string, uint32_t> whys;
        for (const WalkCorner& wc : r->walkCorners) {
            if (wc.lane < 0) {
                ++whys[wc.why.find("kerb end lies") != std::string::npos ? std::string("a kerb end lies out in the paved junction") : wc.why];
                std::printf("        no walk: junction %s arms %u-%u%s: %s; kerb ends (%.2f, %.2f) and (%.2f, %.2f)\n",
                            m.junctions[wc.junction.index].id.c_str(), wc.armA, wc.armB, wc.spread ? " (across its points)" : "",
                            wc.why.c_str(), wc.anchorA.x, wc.anchorA.y, wc.anchorB.x, wc.anchorB.y);
                continue;
            }
            ++laid;
            laidSpread += wc.spread ? 1u : 0u;
            const uint32_t o = offTheWalk(r->lanes.walks[static_cast<size_t>(wc.lane)], walk);
            off += o;
            offWalks += o > 0 ? 1u : 0u;
        }
        uint32_t checked = 0, endsAway = 0, sidewalkLanes = 0;
        Real worst = -1e9;
        for (const ModelCorner& c : m.corners)
            if (c.walk != kNoIndex && !cornerEndsAtKerbEnds(m, c, checked, &worst)) ++endsAway;
        for (const ModelLane& w : m.walks) sidewalkLanes += w.onRoad() ? 1u : 0u;
        std::printf("      %s: %zu ring pairs, %u walkable (piece 1: corners and spread pairs); walks laid for %u (%u across a spread "
                    "junction's points); %zu ring pairs now corners; %u sidewalk lanes; %u corner walks with points off the drawn "
                    "walk (%u points); %u corner ends checked against their crosswalk kerb ends, %u corners with an end farther than "
                    "its sidewalk is wide (the worst %.2f m past it)\n",
                    level, pairs.size(), walkable, laid, laidSpread, m.corners.size(), sidewalkLanes, offWalks, off, checked, endsAway, worst);
        for (const std::pair<const std::string, uint32_t>& w : whys) std::printf("        %u with no walk: %s\n", w.second, w.first.c_str());
        // each corner with no walk says what kind of fault it is, and one with a walk none (ADR-0469: the lane level's lone
        // ends are counted by it)
        for (const WalkCorner& wc : r->walkCorners) {
            CHECK((wc.lane >= 0) == (wc.fault == WalkCornerFault::None));
            if (wc.fault == WalkCornerFault::KerbEndOut) CHECK(wc.why.find("kerb end lies") != std::string::npos);
            if (wc.fault == WalkCornerFault::TwoPieces) CHECK(wc.why.find("two pieces") != std::string::npos);
        }
        CHECK(laid + static_cast<uint32_t>(r->walkCorners.size() - laid) == walkable);
        CHECK(laid > 0u && laidSpread > 0u);
        CHECK(offWalks == 0u);
        CHECK(endsAway == 0u);
    }
}

// ---- 32. the walks in the lane level: one table of paths for walkers and drivers (ADR-0466) ----------------------------

namespace {
// The lane level's walks counted by kind (the model's answer: RoadModel::walkKind).
std::map<std::string, uint32_t> walkKinds(const RoadModel& m) {
    std::map<std::string, uint32_t> out;
    for (uint32_t w = 0; w < m.laneLevel.walks.size(); ++w) ++out[m.walkKind(WalkId(w))];
    return out;
}
// Whether walk end `a` lists walk end `b` among those that meet it.
bool walkEndsMeet(const LaneGraph& G, const WalkEnd& a, const WalkEnd& b) {
    if (a.walk.index >= G.walks.size()) return false;
    const Range r = G.walks[a.walk.index].joins(a.end);
    for (uint32_t k = r.first; k < r.end() && k < G.walkJoins.size(); ++k)
        if (G.walkJoins[k] == b) return true;
    return false;
}
// THE WALK ENDS AT A JUNCTION'S KERB END, found by the model's ids alone: the end of the arm's street's sidewalk on that
// kerb nearer the junction (its stretch's stations), each corner walk's end on that arm and kerb (its arm A end is its
// start), and the crossing's end on that kerb (end 0 on the +1 kerb). True when every one meets every other; `found`
// counts them.
bool kerbEndJoined(const RoadModel& m, ModelJunctionId J, uint32_t arm, int kerb, uint32_t& found) {
    const LaneGraph& G = m.laneLevel;
    const ModelArm& A = m.junctions[J.index].arms[arm];
    const Real at = kerbEndStation(m, J, arm);
    const ModelCrosswalk* cw = m.crosswalkAcross(J, arm);
    const uint32_t cwId = cw ? static_cast<uint32_t>(cw - m.crosswalks.data()) : kNoIndex;
    std::vector<WalkEnd> ends;
    for (uint32_t w = 0; w < G.walks.size(); ++w) {
        const Walk& k = G.walks[w];
        if (k.modelWalk < m.walks.size() && m.walks[k.modelWalk].onRoad()) {
            if (k.street == A.street.index && m.walks[k.modelWalk].dir == kerb)
                if (std::fabs(k.stationFrom - at) < 0.01 || std::fabs(k.stationTo - at) < 0.01)
                    ends.push_back(WalkEnd{WalkId(w), static_cast<uint8_t>(std::fabs(k.stationFrom - at) < 0.01 ? 0 : 1)});
        } else if (k.modelWalk < m.walks.size()) {
            for (const ModelCorner& c : m.corners) {
                if (c.walk != k.modelWalk || c.junction != J) continue;
                if (c.armA == arm && c.kerbA == kerb) ends.push_back(WalkEnd{WalkId(w), 0});
                if (c.armB == arm && c.kerbB == kerb) ends.push_back(WalkEnd{WalkId(w), 1});
            }
        } else if (k.modelCrosswalk != kNoIndex && k.modelCrosswalk == cwId) {
            ends.push_back(WalkEnd{WalkId(w), static_cast<uint8_t>(kerb > 0 ? 0 : 1)});
        }
    }
    found = static_cast<uint32_t>(ends.size());
    for (const WalkEnd& a : ends)
        for (const WalkEnd& b : ends)
            if (!(a == b) && !walkEndsMeet(G, a, b)) return false;
    return true;
}
// A path users' table: the shipped movers, with a `pedestrian` who may use only the kinds named.
PathUsers usersWithPedestrian(const std::vector<std::string>& kinds) {
    nlohmann::json doc = nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/data/path_users.json"));
    doc["movers"]["pedestrian"] = nlohmann::json::object();
    for (const std::string& k : kinds) doc["kinds"][k].push_back("pedestrian");
    PathUsers u;
    std::string why;
    CHECK(parsePathUsers(doc, u, &why));
    return u;
}
uint32_t openCount(const std::vector<uint8_t>& v) {
    uint32_t n = 0;
    for (const uint8_t x : v) n += x ? 1u : 0u;
    return n;
}
// Does a person who may use `open` get from walk a to walk b?
bool walksReach(const LaneGraph& G, const std::vector<uint8_t>& open, uint32_t a, uint32_t b) {
    const ReachableParts parts = reachableWalks(G, open);
    return a < parts.partOf.size() && b < parts.partOf.size() && parts.partOf[a] == parts.partOf[b] && open[a] && open[b];
}
// The model of a scene with the lane level's facts, built.
struct BuiltScene {
    std::unique_ptr<Result> r;
    RoadModel m;
    LaneGraphBuild report;
};
BuiltScene builtScene(const nlohmann::json& file) {
    BuiltScene s;
    s.r = build(RoadLabGraph::fromJson(lane_level_test::withLaneLevelFacts(file)));
    s.m = assembleRoadModel(*s.r, oneCar(), &s.report);
    return s;
}
// No walk's curve lies among the car paths' (they come after every path's), and no car path leads to a walk.
bool walksAfterThePaths(const LaneGraph& G) {
    uint32_t pathsEnd = 0;
    for (const Path& p : G.paths) pathsEnd = std::max(pathsEnd, p.curve.end());
    for (const Walk& k : G.walks)
        if (k.curve.first < pathsEnd) return false;
    return true;
}
}  // namespace

TEST_CASE(road_model_32_the_lane_level_lays_the_walks_and_joins_them) {
    // STRAIGHT STREETS AND DEAD ENDS: two straight roads crossing, each running on through the junction (a road file with
    // no junction at all gets no lane level, as before). Each road's sidewalk lane is cut at the kerb ends into one walk
    // per street; the four far ends are dead ends, each joined across by a turn-round from the +1 kerb's sidewalk end to
    // the -1 kerb's
    {
        nlohmann::json f;
        f["name"] = "walk_cross";
        auto road = [](const std::string& id, const Vec2& a, const Vec2& b) {
            return nlohmann::json{{"id", id}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}};
        };
        f["edges"] = nlohmann::json::array({road("ew", Vec2(-100, 0), Vec2(100, 0)), road("ns", Vec2(0, -100), Vec2(0, 100))});
        writeLayout(f, deriveLayout(RoadLabGraph::fromJson(f), LayoutSource::Inferred));
        BuiltScene s = builtScene(f);
        CHECK(s.report.error.empty());
        const LaneGraph& G = s.m.laneLevel;
        std::map<std::string, uint32_t> kinds = walkKinds(s.m);
        std::printf("      straight streets crossing: %zu streets; %u sidewalks, %u corners, %u crossings, %u turn-rounds; %zu ends alone\n",
                    s.m.streets.size(), kinds["sidewalk"], kinds["corner"], kinds["crossing"], kinds["turn_round"], s.report.walkEndsAlone.size());
        for (const LaneGraphBuild::WalkEndAlone& a : s.report.walkEndsAlone) std::printf("        alone: %s: %s\n", a.where.c_str(), a.why.c_str());
        CHECK(s.m.streets.size() == 4u && s.m.walks.size() == 4u + 4u);   // (two lanes along each road, four corners)
        CHECK(kinds["sidewalk"] == 8u && kinds["corner"] == 4u && kinds["crossing"] == 4u && kinds["turn_round"] == 4u);
        CHECK(s.report.walkEndsAlone.empty());
        CHECK(validate(G).empty());
        // each sidewalk's stretch runs from the kerb end at the junction to its street's far end, on its street
        for (const Walk& k : G.walks) {
            if (k.street == kNoIndex) continue;
            const ModelStreet& S = s.m.streets[k.street];
            CHECK(k.stationFrom >= S.from - kWrittenStation && k.stationTo <= S.to + kWrittenStation && k.stationTo > k.stationFrom);
        }
        // a turn-round: from the +1 kerb's sidewalk end to the -1 kerb's, each meeting just it
        uint32_t turnRounds = 0;
        for (uint32_t w = 0; w < G.walks.size(); ++w) {
            if (G.walks[w].modelDeadEnd == kNoIndex) continue;
            ++turnRounds;
            for (int end = 0; end < 2; ++end) {
                const Range r = G.walks[w].joins(end);
                CHECK(r.count == 1u);
                if (r.count != 1u) continue;
                const Walk& side = G.walks[G.walkJoins[r.first].walk.index];
                CHECK(side.modelWalk < s.m.walks.size() && s.m.walks[side.modelWalk].dir == (end == 0 ? 1 : -1));
            }
        }
        CHECK(turnRounds == 4u);
        for (uint32_t a = 0; a < s.m.junctions[0].arms.size(); ++a)
            for (const int kerb : {+1, -1}) {
                uint32_t found = 0;
                CHECK(kerbEndJoined(s.m, ModelJunctionId(0), a, kerb, found) && found == 3u);
            }
        CHECK(reachableWalks(G, s.m.openTo("walker").walks).sizes.size() == 1u);
        // NOT JOINED ACROSS (the model's dead ends told so, the lane level laid again): no turn-round, eight lone ends that
        // say why
        for (ModelDeadEnd& d : s.m.deadEnds) d.turnRound = false;
        const LaneGraphBuild again = buildLaneLevel(*s.r, s.m, oneCar());
        kinds = walkKinds(s.m);
        CHECK(kinds["turn_round"] == 0u && again.walkEndsAlone.size() == 8u);
        for (const LaneGraphBuild::WalkEndAlone& a : again.walkEndsAlone) CHECK(a.why == "a dead end not joined across");
    }
    // WHERE A STREET CARRIES ON AS ANOTHER (the signed scene's bend at (0, 150), no junction): each kerb's sidewalk ends
    // there meet, the kerb running on
    {
        BuiltScene s = builtScene(laneLevelGraph());
        CHECK(s.report.error.empty());
        const LaneGraph& G = s.m.laneLevel;
        uint32_t carryOns = 0, met = 0;
        for (uint32_t si = 0; si < s.m.streets.size(); ++si)
            for (int end = 0; end < 2; ++end) {
                if (!s.m.streets[si].carriesOn[static_cast<std::size_t>(end)].valid()) continue;
                ++carryOns;
                // this street's sidewalk ends there (by stations), each meeting one sidewalk end of the next street
                const Real at = end == 0 ? s.m.streets[si].from : s.m.streets[si].to;
                for (uint32_t w = 0; w < G.walks.size(); ++w) {
                    const Walk& k = G.walks[w];
                    if (k.street != si) continue;
                    const int e = std::fabs((end == 0 ? k.stationFrom : k.stationTo) - at) < 0.01 ? end : -1;
                    if (e < 0) continue;
                    const Range r = k.joins(e);
                    if (r.count == 1u && G.walks[G.walkJoins[r.first].walk.index].street ==
                                             s.m.streets[si].carriesOn[static_cast<std::size_t>(end)].index)
                        ++met;
                }
            }
        std::printf("      carry-ons: %u street ends carry on; %u sidewalk ends there meet the next street's\n", carryOns, met);
        for (const LaneGraphBuild::WalkEndAlone& a : s.report.walkEndsAlone) std::printf("        alone: %s: %s\n", a.where.c_str(), a.why.c_str());
        CHECK(carryOns >= 2u && met == carryOns * 2u);
        CHECK(validate(G).empty());
    }
    // A T: every walk there with its kind; at every kerb end its sidewalk, corner and crossing meet; across a crosswalk
    // to the corner on the arm's far kerb by walker paths alone; a car sees none of it
    {
        BuiltScene s = builtScene(walkTee());
        CHECK(s.report.error.empty());
        const LaneGraph& G = s.m.laneLevel;
        std::map<std::string, uint32_t> kinds = walkKinds(s.m);
        std::printf("      T: %u sidewalks, %u corners, %u crossings, %u turn-rounds; %zu joins; %zu ends alone\n", kinds["sidewalk"],
                    kinds["corner"], kinds["crossing"], kinds["turn_round"], G.walkJoins.size(), s.report.walkEndsAlone.size());
        for (const LaneGraphBuild::WalkEndAlone& a : s.report.walkEndsAlone) std::printf("        alone: %s: %s\n", a.where.c_str(), a.why.c_str());
        CHECK(kinds["sidewalk"] == 6u && kinds["corner"] == 3u && kinds["crossing"] == 3u && kinds["turn_round"] == 3u);
        CHECK(G.walks.size() == 15u && s.report.walkEndsAlone.empty());
        CHECK(validate(G).empty());
        CHECK(walksAfterThePaths(G));
        uint32_t kerbEnds = 0;
        for (uint32_t a = 0; a < 3; ++a)
            for (const int kerb : {+1, -1}) {
                uint32_t found = 0;
                CHECK(kerbEndJoined(s.m, ModelJunctionId(0), a, kerb, found));
                CHECK(found == 3u);   // its sidewalk, one corner, the crossing
                kerbEnds += found == 3u ? 1u : 0u;
            }
        CHECK(kerbEnds == 6u);
        // A JOIN TAKEN AWAY IS CAUGHT: the kerb-end check and the lane level's own check both bite
        {
            RoadModel cut = s.m;
            uint32_t corner = kNoIndex;
            for (uint32_t w = 0; w < cut.laneLevel.walks.size() && corner == kNoIndex; ++w)
                if (cut.walkKind(WalkId(w)) == "corner") corner = w;
            CHECK(corner != kNoIndex);
            if (corner == kNoIndex) return;
            cut.laneLevel.walks[corner].atStart.count = 0;
            bool any = false;
            for (uint32_t a = 0; a < 3; ++a)
                for (const int kerb : {+1, -1}) {
                    uint32_t found = 0;
                    any = any || !kerbEndJoined(cut, ModelJunctionId(0), a, kerb, found);
                }
            CHECK(any);
            CHECK(!validate(cut.laneLevel).empty());
        }
        // ACROSS A CROSSWALK: from the east road's +1 sidewalk to the corner on its -1 kerb, by a person who may use the
        // sidewalks, corners and crossings (not the turn-rounds, which would go round by the dead ends); not without crossings
        uint32_t eastArm = kNoIndex, eastWalk = kNoIndex, farCorner = kNoIndex;
        for (uint32_t a = 0; a < s.m.junctions[0].arms.size(); ++a)
            if (s.m.junctions[0].arms[a].road == "east") eastArm = a;
        CHECK(eastArm != kNoIndex);
        if (eastArm == kNoIndex) return;
        for (uint32_t w = 0; w < G.walks.size(); ++w) {
            const Walk& k = G.walks[w];
            if (k.modelWalk >= s.m.walks.size()) continue;
            const ModelLane& W = s.m.walks[k.modelWalk];
            if (W.onRoad() && k.street == s.m.junctions[0].arms[eastArm].street.index && W.dir == 1) eastWalk = w;
            for (const ModelCorner& c : s.m.corners)
                if (!W.onRoad() && c.walk == k.modelWalk && ((c.armA == eastArm && c.kerbA == -1) || (c.armB == eastArm && c.kerbB == -1)))
                    farCorner = w;
        }
        CHECK(eastWalk != kNoIndex && farCorner != kNoIndex);
        const RoadModel::OpenPaths onFoot = s.m.openTo("pedestrian", usersWithPedestrian({"sidewalk", "corner", "crossing"}));
        const RoadModel::OpenPaths noCrossing = s.m.openTo("pedestrian", usersWithPedestrian({"sidewalk", "corner"}));
        CHECK(openCount(onFoot.walks) == 12u && openCount(onFoot.paths) == 0u);
        CHECK(walksReach(G, onFoot.walks, eastWalk, farCorner));
        CHECK(!walksReach(G, noCrossing.walks, eastWalk, farCorner));
        // the shipped walker may use every walk, and the whole T is one part to them
        const RoadModel::OpenPaths walker = s.m.openTo("walker");
        CHECK(openCount(walker.walks) == G.walks.size() && openCount(walker.paths) == 0u);
        CHECK(reachableWalks(G, walker.walks).sizes.size() == 1u);
        // A CAR SEES NONE OF THEM: the vehicles' filter opens every lane path and connector, and no walk; it bites (a
        // table opening the sidewalks to the vehicles opens six walks)
        const RoadModel::OpenPaths cars = s.m.openTo("vehicles");
        CHECK(openCount(cars.walks) == 0u && openCount(cars.paths) == G.paths.size() && !G.paths.empty());
        nlohmann::json loose = nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/data/path_users.json"));
        loose["kinds"]["sidewalk"].push_back("vehicles");
        PathUsers looseUsers;
        CHECK(parsePathUsers(loose, looseUsers));
        CHECK(openCount(s.m.openTo("vehicles", looseUsers).walks) == 6u);
        // saved and read back whole, walks, joins and all
        bundle::BinWriter wr;
        putRoadModel(wr, s.m);
        bundle::BinReader rd(wr.bytes.data(), wr.bytes.size());
        RoadModel back;
        std::string why;
        CHECK(getRoadModel(rd, back, &why) && back == s.m && back.laneLevel.walks.size() == G.walks.size());
        CHECK(dumpText(back.laneLevel) == dumpText(G));
    }
    // A FOOTWAY ARM: the T's south road a footway. Its walk meets both corners at its one kerb end (its middle); it has
    // no crossing and no turn-round (its far end is a dead end nobody joins across)
    {
        nlohmann::json f = walkTee();
        f["edges"][2]["class"] = "footway";
        BuiltScene s = builtScene(f);
        CHECK(s.report.error.empty());
        const LaneGraph& G = s.m.laneLevel;
        std::map<std::string, uint32_t> kinds = walkKinds(s.m);
        std::printf("      T with a footway: %u footways, %u sidewalks, %u corners, %u crossings, %u turn-rounds; %zu ends alone\n",
                    kinds["footway"], kinds["sidewalk"], kinds["corner"], kinds["crossing"], kinds["turn_round"], s.report.walkEndsAlone.size());
        for (const LaneGraphBuild::WalkEndAlone& a : s.report.walkEndsAlone) std::printf("        alone: %s: %s\n", a.where.c_str(), a.why.c_str());
        CHECK(kinds["footway"] == 1u && kinds["sidewalk"] == 4u && kinds["corner"] == 3u && kinds["crossing"] == 2u && kinds["turn_round"] == 2u);
        CHECK(validate(G).empty());
        uint32_t footway = kNoIndex, south = kNoIndex;
        for (uint32_t w = 0; w < G.walks.size(); ++w)
            if (s.m.walkKind(WalkId(w)) == "footway") footway = w;
        for (uint32_t a = 0; a < s.m.junctions[0].arms.size(); ++a)
            if (s.m.junctions[0].arms[a].road == "south") south = a;
        CHECK(footway != kNoIndex && south != kNoIndex);
        if (footway == kNoIndex || south == kNoIndex) return;
        // (the footway runs north into the junction: its end there)
        uint32_t cornersMet = 0;
        const Range atJunction = G.walks[footway].atEnd;
        for (uint32_t k = atJunction.first; k < atJunction.end(); ++k) cornersMet += s.m.walkKind(G.walkJoins[k].walk) == "corner" ? 1u : 0u;
        CHECK(cornersMet == 2u && atJunction.count == 2u);
        CHECK(s.report.walkEndsAlone.size() == 1u);
        if (!s.report.walkEndsAlone.empty()) CHECK(s.report.walkEndsAlone[0].walk == footway && s.report.walkEndsAlone[0].end == 0);
        CHECK(reachableWalks(G, s.m.openTo("walker").walks).sizes.size() == 1u);
    }
    // A SPREAD JUNCTION: the corners across its two points join the street's sidewalks at their kerb ends, so a person
    // walks round it, every walk one part
    {
        BuiltScene s = builtScene(walkSpread());
        CHECK(s.report.error.empty());
        const LaneGraph& G = s.m.laneLevel;
        std::map<std::string, uint32_t> kinds = walkKinds(s.m);
        std::printf("      spread junction: %u sidewalks, %u corners, %u crossings, %u turn-rounds; %zu ends alone\n", kinds["sidewalk"],
                    kinds["corner"], kinds["crossing"], kinds["turn_round"], s.report.walkEndsAlone.size());
        for (const LaneGraphBuild::WalkEndAlone& a : s.report.walkEndsAlone) std::printf("        alone: %s: %s\n", a.where.c_str(), a.why.c_str());
        CHECK(kinds["corner"] == 6u && kinds["sidewalk"] == 2u * 2u + 4u * 2u);   // (the street's two stretches, the four one-ways)
        CHECK(kinds["crossing"] == 6u && kinds["turn_round"] == 6u && s.report.walkEndsAlone.empty());
        CHECK(validate(G).empty());
        uint32_t spreadJoined = 0;
        for (const ModelCorner& c : s.m.corners) {
            if (!c.spread) continue;
            uint32_t found = 0;
            bool joined = kerbEndJoined(s.m, c.junction, c.armA, c.kerbA, found);
            joined = kerbEndJoined(s.m, c.junction, c.armB, c.kerbB, found) && joined;
            spreadJoined += joined ? 1u : 0u;
        }
        CHECK(spreadJoined == 2u);
        for (uint32_t a = 0; a < s.m.junctions[0].arms.size(); ++a)
            for (const int kerb : {+1, -1}) {
                uint32_t found = 0;
                CHECK(kerbEndJoined(s.m, ModelJunctionId(0), a, kerb, found));
                CHECK(found >= 2u);
            }
        CHECK(reachableWalks(G, s.m.openTo("walker").walks).sizes.size() == 1u);
        CHECK(openCount(s.m.openTo("vehicles").walks) == 0u);
    }
}

// THE SHIPPED LEVELS' WALKS IN THE LANE LEVEL (ADR-0466): the walks by kind, the joins, every walk end alone with why, and
// how much of the sidewalks a person on foot can get round.
TEST_CASE(road_model_33_the_shipped_levels_walks_in_the_lane_level) {
    for (const char* level : {"traffic_lab", "river_town"}) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/" + level + "_lanelab.json";
        const std::unique_ptr<Result> r =
            build(RoadLabGraph::fromJson(nlohmann::json::parse(slurp(path)), path.substr(0, path.find_last_of('/'))));
        LaneGraphBuild report;
        const RoadModel m = assembleRoadModel(*r, oneCar(), &report);
        CHECK(report.error.empty());
        const LaneGraph& G = m.laneLevel;
        const std::map<std::string, uint32_t> kinds = walkKinds(m);
        std::printf("      %s: %zu walks:", level, G.walks.size());
        for (const std::pair<const std::string, uint32_t>& k : kinds) std::printf(" %u %s,", k.second, k.first.c_str());
        std::printf(" %zu joins (walk end to walk end, each counted from both ends); %zu car paths\n", G.walkJoins.size(), G.paths.size());
        CHECK(validate(G).empty());
        CHECK(walksAfterThePaths(G));
        // the filters: a car sees no walk and every path; a walker every walk and no path
        const RoadModel::OpenPaths cars = m.openTo("vehicles"), walker = m.openTo("walker");
        std::printf("        open to the vehicles: %u of %zu paths, %u walks; to the walker: %u paths, %u of %zu walks\n", openCount(cars.paths),
                    G.paths.size(), openCount(cars.walks), openCount(walker.paths), openCount(walker.walks), G.walks.size());
        CHECK(openCount(cars.walks) == 0u && openCount(walker.paths) == 0u && openCount(walker.walks) == G.walks.size());
        // the model's sidewalk lanes the lane level made no walk of, and why
        {
            std::vector<uint32_t> piecesOf(m.walks.size(), 0);
            for (const Walk& k : G.walks)
                if (k.modelWalk < m.walks.size()) ++piecesOf[k.modelWalk];
            std::map<std::string, uint32_t> none;
            uint32_t lanes = 0;
            for (uint32_t wi = 0; wi < m.walks.size(); ++wi) {
                const ModelLane& W = m.walks[wi];
                if (!W.onRoad()) continue;
                ++lanes;
                if (piecesOf[wi] > 0) continue;
                bool laid = true;
                for (uint32_t k = W.curve.first; k < W.curve.end(); ++k) laid = laid && stated(m.spineStations[k]);
                ++none[W.streets.empty() ? "its road has no street (a ramp, or a stretch inside a junction)"
                       : !laid           ? "not laid beside its spine"
                                         : "no room between its kerb ends"];
            }
            std::printf("        %u sidewalk lanes in the model; walks made of them:", lanes);
            for (const std::pair<const std::string, uint32_t>& n : none) std::printf(" %u none (%s);", n.second, n.first.c_str());
            std::printf("\n");
        }
        // every walk end alone, by why
        std::map<std::string, uint32_t> alone;
        for (const LaneGraphBuild::WalkEndAlone& a : report.walkEndsAlone) ++alone[a.why];
        std::printf("        %zu walk ends meet nothing:\n", report.walkEndsAlone.size());
        for (const std::pair<const std::string, uint32_t>& a : alone) std::printf("          %u: %s\n", a.second, a.first.c_str());
        for (const LaneGraphBuild::WalkEndAlone& a : report.walkEndsAlone)
            if (a.why.find("carries on") != std::string::npos || a.why.find("stops short") != std::string::npos)
                std::printf("            %s at (%.1f, %.1f): %s\n", a.where.c_str(), a.at.x, a.at.y, a.why.c_str());
        // how far apart the ends of two walks that meet lie, by the pair of kinds: the step off a kerb, a seam
        std::map<std::string, Real> widest;
        const auto endAt = [&](const WalkEnd& e) {
            const Walk& k = G.walks[e.walk.index];
            return G.points[e.end == 0 ? k.curve.first : k.curve.end() - 1];
        };
        for (uint32_t w = 0; w < G.walks.size(); ++w)
            for (uint8_t end = 0; end < 2; ++end) {
                const Range r = G.walks[w].joins(end);
                for (uint32_t j = r.first; j < r.end(); ++j) {
                    const WalkEnd other = G.walkJoins[j];
                    const std::string a = m.walkKind(WalkId(w)), b = m.walkKind(other.walk);
                    Real& wide = widest[a < b ? a + "-" + b : b + "-" + a];
                    const Real gap = distance(endAt(WalkEnd{WalkId(w), end}), endAt(other));
                    wide = std::max(wide, gap);
                    if (a == "sidewalk" && b == "sidewalk" && gap > 2.0 && w < other.walk.index)
                        std::printf("            a carry-on's sidewalks joined across a gap (the planner's): %s (kerb %d) - %s (kerb %d), %.2f m\n", m.streets[G.walks[w].street].id.c_str(),
                                    m.walks[G.walks[w].modelWalk].dir, m.streets[G.walks[other.walk.index].street].id.c_str(),
                                    m.walks[G.walks[other.walk.index].modelWalk].dir, gap);
                }
            }
        std::printf("        the widest gap between two walk ends that meet:");
        for (const std::pair<const std::string, Real>& w : widest) std::printf(" %s %.2f m;", w.first.c_str(), w.second);
        std::printf("\n");
        // the sidewalks a person on foot can get round: the largest part's share, and the rest by the lone ends of their parts
        const ReachableParts parts = reachableWalks(G, walker.walks);
        std::vector<uint32_t> sidewalksIn(parts.sizes.size(), 0);
        uint32_t sidewalks = 0;
        for (uint32_t w = 0; w < G.walks.size(); ++w)
            if (m.walkKind(WalkId(w)) == "sidewalk") {
                ++sidewalks;
                ++sidewalksIn[parts.partOf[w]];
            }
        uint32_t best = 0;
        for (uint32_t p = 0; p < sidewalksIn.size(); ++p)
            if (sidewalksIn[p] > sidewalksIn[best]) best = p;
        const uint32_t inLargest = sidewalksIn.empty() ? 0u : sidewalksIn[best];
        std::printf("        %zu parts; the largest holds %u of %u sidewalks (%.1f%%); %u sidewalks outside it\n", parts.sizes.size(), inLargest,
                    sidewalks, sidewalks ? 100.0 * inLargest / sidewalks : 0.0, sidewalks - inLargest);
        std::map<uint32_t, std::set<std::string>> whyOfPart;
        for (const LaneGraphBuild::WalkEndAlone& a : report.walkEndsAlone) whyOfPart[parts.partOf[a.walk]].insert(a.why);
        std::map<std::string, std::pair<uint32_t, uint32_t>> outside;   // why -> (parts, sidewalks)
        for (uint32_t p = 0; p < sidewalksIn.size(); ++p) {
            if (p == best || sidewalksIn[p] == 0) continue;
            std::string why;
            for (const std::string& w : whyOfPart[p]) why += (why.empty() ? "" : "; ") + w;
            if (why.empty()) why = "no lone end: every way out is a missing crossing or corner";
            ++outside[why].first;
            outside[why].second += sidewalksIn[p];
        }
        for (const std::pair<const std::string, std::pair<uint32_t, uint32_t>>& o : outside)
            std::printf("          %u sidewalks in %u parts bounded by: %s\n", o.second.second, o.second.first, o.first.c_str());
        CHECK(sidewalks > 0u && inLargest * 2 > sidewalks);
        // THE LONE ENDS BY CAUSE, pinned (ADR-0469; was, at ADR-0466: traffic_lab 61, river_town 126). The fixes left for the
        // one planned re-bake are named in the ADR:
        //   - a corner with no walk because a kerb end lies out in the paved junction: the box edge's one rule
        //     (road_model_parts.h boxEdgeBack) sets an arm's crosswalk back by the crossing road's half width as if the two
        //     met square, so where they meet at a sharp angle the crosswalk's kerb end stands in the other road;
        //   - a corner with no walk because the drawn sidewalk is in two pieces: each road's sidewalk band is cut square at
        //     the road's end, leaving a wedge of bare ground where the kerb kinks from one road into the next;
        //   - traffic_lab's s208, the fourth arm of j95, not stated in its ring (its two kerbs' ends; stated since ADR-0473);
        //   - river_town's freeway carry-ons: a boulevard's sidewalks end where it runs on as the freeway (bv0, bv1; tn0 and
        //     tn1 too until ADR-0473 made them ramps).
        // No end may say "no corner is stated" where the build tried a walk (a spread junction's pair: it was counted as a
        // freeway arm's), and none is a street end under the freeway (now a dead end, joined across: 16 and 24 turn-rounds).
        std::map<std::string, uint32_t> cause;
        for (const LaneGraphBuild::WalkEndAlone& a : report.walkEndsAlone)
            ++cause[a.why.find(walkCornerFaultWord(WalkCornerFault::KerbEndOut)) != std::string::npos ? "kerb end out"
                    : a.why.find(walkCornerFaultWord(WalkCornerFault::TwoPieces)) != std::string::npos ? "two pieces"
                    : a.why.find("meets another road") != std::string::npos                         ? "meets another road"
                    : a.why.find("carries on into") != std::string::npos                            ? "carry-on"
                                                                                                    : a.why];
        // (ADR-0471, the box edge at the angle the arms meet: kerb end out 24 -> 4 / 49 -> 22, two pieces 3 -> 1 / 13 -> 5 --
        // a lone end is counted where its corner has no walk AND no crosswalk people use, and more crosswalks now lie clear of
        // the other road; sidewalks cut off 4 -> 0 / 25 -> 14. The kerb ends still out are at the junctions the next one
        // stands too close to (road_model_39), and four river_town corners at j19, j116 and j119)
        // (ADR-0472, the bands mitred at a kink: two pieces 1 -> 0 / 5 -> 1. The one left is river_town j90 0-1, whose arm 0
        // kerb end lies on a scrap of s228's sidewalk inside s227's carriageway: road_model_41. Sidewalks cut off unchanged,
        // 0 / 14: the parts still cut off are bounded by kerb ends out.)
        // (ADR-0473, the road files: traffic_lab's meets another road 2 -> 0, s208 now j95's fourth arm; river_town's
        // carry-ons 16 -> 8 and sidewalks cut off 14 -> 10, tn0 and tn1 now ramps with no sidewalks, so only bv0's and bv1's
        // sidewalks end where they run on as the freeway.)
        const bool lab = std::string(level) == "traffic_lab";
        const std::map<std::string, uint32_t> pinned =
            lab ? std::map<std::string, uint32_t>{{"kerb end out", 4}}
                : std::map<std::string, uint32_t>{{"kerb end out", 22}, {"two pieces", 1}, {"carry-on", 8}};
        CHECK(cause == pinned);
        CHECK(kinds.count("turn_round") && kinds.at("turn_round") == (lab ? 16u : 31u));
        CHECK(sidewalks - inLargest == (lab ? 0u : 10u));
    }
}

// ---- 34. the one route-finder: a walker's route over the walks (ADR-0467) ----------------------------------------------

namespace {
namespace route = engine::roads;
// THE SIDEWALK of a road's street on kerb `kerb` (+1 / -1), by the model's ids; kNoIndex: none.
uint32_t sidewalkOf(const RoadModel& m, const std::string& road, int kerb) {
    const LaneGraph& G = m.laneLevel;
    for (uint32_t w = 0; w < G.walks.size(); ++w) {
        const Walk& k = G.walks[w];
        if (k.modelWalk >= m.walks.size() || !m.walks[k.modelWalk].onRoad() || k.street >= m.streets.size()) continue;
        if (m.walks[k.modelWalk].dir == kerb && m.streets[k.street].road == road) return w;
    }
    return kNoIndex;
}
// A place `back` metres along a walk from its end nearer `near` (a junction's point).
WalkPoint nearEnd(const LaneGraph& G, uint32_t walk, const Vec2& near, Real back) {
    const Walk& k = G.walks[walk];
    const bool startNearer = distance(G.pointAt(WalkId(walk), 0), near) < distance(G.pointAt(WalkId(walk), k.length), near);
    WalkPoint p;
    p.walk = WalkId(walk);
    p.along = startNearer ? std::min(back, k.length) : std::max(Real(0), k.length - back);
    return p;
}
uint32_t stepsOfKind(const RoadModel& m, const route::WalkRoute& r, const std::string& kind) {
    uint32_t n = 0;
    for (const route::WalkStep& s : r.steps) n += m.walkKind(s.walk) == kind ? 1u : 0u;
    return n;
}
bool sameWalkRoute(const route::WalkRoute& a, const route::WalkRoute& b) {
    return a.found == b.found && a.why == b.why && a.steps == b.steps && a.crossings == b.crossings && a.metres == b.metres &&
           a.seconds == b.seconds && a.waitSeconds == b.waitSeconds;
}
void printWalkRoute(const RoadModel& m, const route::WalkRoute& r) {
    std::printf("        %s: %.1f m, %.1f s (%.1f s waiting), %zu crossings:", r.found ? "found" : r.whyWords.c_str(), r.metres, r.seconds,
                r.waitSeconds, r.crossings.size());
    for (const route::WalkStep& s : r.steps)
        std::printf(" %s %u %.1f->%.1f;", m.walkKind(s.walk).c_str(), s.walk.index, s.from, s.to);
    std::printf("\n");
}
constexpr Real kTestPace = 1.4;   // m/s: a walker's pace in these tests (the router is given the person's)
}  // namespace

TEST_CASE(road_model_34_a_walkers_route_over_the_walks) {
    // A T: from the east road's +1 sidewalk to its -1 sidewalk, 10 m from the junction on each: across the east arm's
    // crosswalk (not round by the far dead end's turn-round, 170 m), the crossing's wait the path users' table states
    BuiltScene s = builtScene(walkTee());
    CHECK(s.report.error.empty());
    const LaneGraph& G = s.m.laneLevel;
    const Vec2 junction = s.m.junctions[0].at;
    uint32_t eastArm = kNoIndex;
    for (uint32_t a = 0; a < s.m.junctions[0].arms.size(); ++a)
        if (s.m.junctions[0].arms[a].road == "east") eastArm = a;
    const uint32_t eastNear = sidewalkOf(s.m, "east", +1), eastFar = sidewalkOf(s.m, "east", -1);
    CHECK(eastArm != kNoIndex && eastNear != kNoIndex && eastFar != kNoIndex);
    if (eastArm == kNoIndex || eastNear == kNoIndex || eastFar == kNoIndex) return;
    const ModelCrosswalk* eastCrosswalk = s.m.crosswalkAcross(ModelJunctionId(0), eastArm);
    CHECK(eastCrosswalk != nullptr);
    if (!eastCrosswalk) return;
    const uint32_t eastCw = static_cast<uint32_t>(eastCrosswalk - s.m.crosswalks.data());
    const WalkPoint a = nearEnd(G, eastNear, junction, 10), b = nearEnd(G, eastFar, junction, 10);
    const route::RouteWalker walker = route::bindWalker(s.m, "walker", kTestPace);
    const Real crossingWait = pathUsers().mover("walker")->waitOn("crossing");
    CHECK(crossingWait > 0 && pathUsers().mover("walker")->waitOn("sidewalk") == 0);
    // THE WAITS ARE CHECKED AS READ: refused on the vehicles' mover, below zero, or on a kind not open to the mover
    {
        const nlohmann::json shipped = nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/data/path_users.json"));
        PathUsers bad;
        std::string why;
        nlohmann::json onCars = shipped;
        onCars["movers"]["vehicles"]["waits"] = {{"connector", 5}};
        CHECK(!parsePathUsers(onCars, bad, &why) && why.find("vehicles") != std::string::npos);
        nlohmann::json negative = shipped;
        negative["movers"]["walker"]["waits"] = {{"crossing", -1}};
        CHECK(!parsePathUsers(negative, bad, &why) && why.find("seconds") != std::string::npos);
        nlohmann::json notOpen = shipped;
        notOpen["movers"]["walker"]["waits"] = {{"connector", 5}};
        CHECK(!parsePathUsers(notOpen, bad, &why) && why.find("does not open") != std::string::npos);
    }
    {
        const route::WalkRoute r = route::findWalkRoute(s.m, walker, a, b);
        std::printf("      T, east +1 sidewalk to east -1 sidewalk:\n");
        printWalkRoute(s.m, r);
        CHECK(r.found);
        CHECK(r.crossings.size() == 1u && stepsOfKind(s.m, r, "crossing") == 1u && stepsOfKind(s.m, r, "turn_round") == 0u);
        if (!r.crossings.empty()) {
            CHECK(r.crossings[0].crosswalk == eastCw);
            // stepped off from the +1 kerb's end (kerb end 0: the crossing walk starts there)
            CHECK(r.crossings[0].fromEnd == 0u && r.steps[r.crossings[0].step].forward);
        }
        CHECK(r.steps.size() >= 3u && r.steps.front().walk == a.walk && r.steps.back().walk == b.walk);
        CHECK(std::fabs(r.steps.front().from - a.along) < 1e-9 && std::fabs(r.steps.back().to - b.along) < 1e-9);
        CHECK(r.metres > 20 && r.metres < 45);
        CHECK(std::fabs(r.waitSeconds - crossingWait) < 1e-9);
        CHECK(std::fabs(r.seconds - (r.metres / kTestPace + r.waitSeconds)) < 1e-6);
        // the way back crosses the same crosswalk from its other end
        const route::WalkRoute back = route::findWalkRoute(s.m, walker, b, a);
        CHECK(back.found && back.crossings.size() == 1u && std::fabs(back.metres - r.metres) < 1e-6);
        if (back.crossings.size() == 1u) CHECK(back.crossings[0].crosswalk == eastCw && back.crossings[0].fromEnd == 1u);
        // SAME ANSWER EVERY RUN
        CHECK(sameWalkRoute(r, route::findWalkRoute(s.m, walker, a, b)));
        // the walker's start and end on one walk: one step, no wait
        WalkPoint c = a;
        c.along = a.along > 20 ? a.along - 15 : a.along + 15;
        const route::WalkRoute same = route::findWalkRoute(s.m, walker, a, c);
        CHECK(same.found && same.steps.size() == 1u && std::fabs(same.metres - 15) < 1e-9 && same.waitSeconds == 0);
    }
    // CROSSINGS CLOSED TO THE MOVER: it goes round by the dead end's turn-round; with turn-rounds closed too, no route
    {
        const route::RouteWalker noCrossing =
            route::bindWalker(s.m, "pedestrian", kTestPace, usersWithPedestrian({"sidewalk", "corner", "turn_round"}));
        const route::WalkRoute r = route::findWalkRoute(s.m, noCrossing, a, b);
        std::printf("      T, crossings closed:\n");
        printWalkRoute(s.m, r);
        CHECK(r.found && r.crossings.empty() && stepsOfKind(s.m, r, "crossing") == 0u && stepsOfKind(s.m, r, "turn_round") == 1u);
        CHECK(r.metres > 150);
        const route::RouteWalker stuck = route::bindWalker(s.m, "pedestrian", kTestPace, usersWithPedestrian({"sidewalk", "corner"}));
        const route::WalkRoute none = route::findWalkRoute(s.m, stuck, a, b);
        CHECK(!none.found && none.why == route::NoRouteWhy::NothingLeadsThere);
        // a destination on a walk the mover may not use
        WalkPoint onCrossing;
        for (uint32_t w = 0; w < G.walks.size(); ++w)
            if (G.walks[w].modelCrosswalk == eastCw) onCrossing.walk = WalkId(w);
        CHECK(route::findWalkRoute(s.m, noCrossing, a, onCrossing).why == route::NoRouteWhy::DestinationClosed);
        // a vehicle (the vehicles mover) may use no walk at all
        CHECK(route::findWalkRoute(s.m, route::bindWalker(s.m, "vehicles", kTestPace), a, b).why == route::NoRouteWhy::DestinationClosed);
    }
    // A FOOTWAY: the T's south road a footway. A walker from the east road's sidewalk reaches a point on it along the
    // footway's walk; a car may not drive it (the vehicles' filter shuts its lane path)
    {
        nlohmann::json f = walkTee();
        f["edges"][2]["class"] = "footway";
        BuiltScene fw = builtScene(f);
        CHECK(fw.report.error.empty());
        const LaneGraph& F = fw.m.laneLevel;
        uint32_t footway = kNoIndex;
        for (uint32_t w = 0; w < F.walks.size(); ++w)
            if (fw.m.walkKind(WalkId(w)) == "footway") footway = w;
        const uint32_t east = sidewalkOf(fw.m, "east", +1);
        CHECK(footway != kNoIndex && east != kNoIndex);
        if (footway == kNoIndex || east == kNoIndex) return;
        const WalkPoint from = nearEnd(F, east, fw.m.junctions[0].at, 30), to = nearEnd(F, footway, fw.m.junctions[0].at, 40);
        const route::WalkRoute r = route::findWalkRoute(fw.m, route::bindWalker(fw.m, "walker", kTestPace), from, to);
        std::printf("      T with a footway, east sidewalk to 40 m down the footway:\n");
        printWalkRoute(fw.m, r);
        CHECK(r.found && stepsOfKind(fw.m, r, "footway") == 1u && !r.steps.empty() && r.steps.back().walk == WalkId(footway));
        // A CAR NEVER: the lane level lays no lane path along the footway (no car path has its street), and the footway
        // walk is shut to the vehicles' mover, so neither of a car's searches can take it
        uint32_t carPathsOnIt = 0;
        for (const Path& p : F.paths)
            carPathsOnIt += p.street < fw.m.streets.size() && fw.m.streets[p.street].road == "south" ? 1u : 0u;
        CHECK(carPathsOnIt == 0u);
        CHECK(!fw.m.openTo("vehicles").walks[footway]);
        const route::WalkRoute byCar = route::findWalkRoute(fw.m, route::bindWalker(fw.m, "vehicles", kTestPace), from, to);
        CHECK(!byCar.found && byCar.why == route::NoRouteWhy::DestinationClosed);
        // ... and a walker on a table that shuts footways to them is refused it too
        const route::RouteWalker noFootway =
            route::bindWalker(fw.m, "pedestrian", kTestPace, usersWithPedestrian({"sidewalk", "corner", "crossing", "turn_round"}));
        CHECK(route::findWalkRoute(fw.m, noFootway, from, to).why == route::NoRouteWhy::DestinationClosed);
    }
    // ONE-WAY RULES ARE THE CARS': on the spread junction's one-way northbound road, a walker goes south along its
    // sidewalk (against the traffic), the car on its lane cannot
    {
        BuiltScene sp = builtScene(walkSpread());
        CHECK(sp.report.error.empty());
        const LaneGraph& S = sp.m.laneLevel;
        const route::RouteWalker w = route::bindWalker(sp.m, "walker", kTestPace);
        const uint32_t nb = sidewalkOf(sp.m, "nb_n", +1);
        CHECK(nb != kNoIndex);
        if (nb == kNoIndex) return;
        // from 80 m north of the junction to 20 m north, along nb_n's sidewalk, either way along it
        const WalkPoint north = nearEnd(S, nb, Vec2(4, 0), 80), south = nearEnd(S, nb, Vec2(4, 0), 20);
        const route::WalkRoute down = route::findWalkRoute(sp.m, w, north, south), up = route::findWalkRoute(sp.m, w, south, north);
        std::printf("      spread junction, nb_n's sidewalk south (against its one-way traffic), then north:\n");
        printWalkRoute(sp.m, down);
        printWalkRoute(sp.m, up);
        CHECK(down.found && up.found && down.steps.size() == 1u && up.steps.size() == 1u);
        CHECK(std::fabs(down.metres - 60) < 1e-6 && std::fabs(up.metres - 60) < 1e-6);
        if (!down.steps.empty() && !up.steps.empty()) CHECK(down.steps[0].forward != up.steps[0].forward);
        // the car, on the road's one lane at the same two places: north to south finds nothing (it is one-way, and its
        // far end leads nowhere), south to north is the 60 m drive
        PathId lane;
        for (uint32_t p = 0; p < S.paths.size(); ++p)
            if (S.paths[p].kind == PathKind::Lane && S.paths[p].street < sp.m.streets.size() && sp.m.streets[S.paths[p].street].road == "nb_n")
                lane = PathId(p);
        CHECK(lane.valid());
        if (!lane.valid()) return;
        route::RouteClass car;
        car.tags = ~TagMask(0);
        car.topSpeed = 20;
        const Vec2 jn(4, 0);
        const bool laneStartNear = distance(S.pointAt(lane, 0), jn) < distance(S.pointAt(lane, S.paths[lane.index].length), jn);
        const Real len = S.paths[lane.index].length;
        PathPoint carNorth, carSouth;
        carNorth.path = carSouth.path = lane;
        carNorth.along = laneStartNear ? std::min(len, Real(80)) : std::max(Real(0), len - 80);
        carSouth.along = laneStartNear ? std::min(len, Real(20)) : std::max(Real(0), len - 20);
        const route::Route carDown = route::findRoute(sp.m, route::studyRoads(sp.m), car, carNorth, carSouth);
        const route::Route carUp = route::findRoute(sp.m, route::studyRoads(sp.m), car, carSouth, carNorth);
        std::printf("        the car: south %s; north %s (%.1f m)\n", carDown.found ? "found" : carDown.whyWords.c_str(),
                    carUp.found ? "found" : carUp.whyWords.c_str(), carUp.metres);
        CHECK(!carDown.found && carUp.found);
    }
}

// ---- 35. the one route-finder on the shipped levels (ADR-0467) ---------------------------------------------------------

TEST_CASE(road_model_35_the_one_router_on_the_shipped_levels) {
    for (const char* level : {"traffic_lab", "river_town"}) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/" + level + "_lanelab.json";
        const std::unique_ptr<Result> r =
            build(RoadLabGraph::fromJson(nlohmann::json::parse(slurp(path)), path.substr(0, path.find_last_of('/'))));
        const RoadModel m = assembleRoadModel(*r, oneCar());
        const LaneGraph& G = m.laneLevel;
        // CARS: fixed-seed trips between points of the largest part, every route's hash folded (the move to roads/ kept
        // them all: traffic_lab 0x909f03a16d4a0ad3, river_town 0xf727e34512df5c49 before it, from the driving's router)
        {
            const ReachableParts parts = reachableParts(G);
            std::vector<uint32_t> inLargest;
            for (uint32_t p = 0; p < G.paths.size(); ++p)
                if (parts.partOf[p] == parts.largest && G.paths[p].length > 1) inLargest.push_back(p);
            route::RouteClass car;
            car.tags = ~TagMask(0);
            car.topSpeed = 20;
            // (the vehicles' filter by kind opens every lane path here: the car search need not ask it, ADR-0467)
            CHECK(openCount(m.openTo("vehicles").paths) == G.paths.size());
            const route::RouteBounds bounds = route::studyRoads(m);
            std::mt19937 rng(1234u);
            uint64_t fold = 1469598103934665603ull;
            uint32_t found = 0;
            const uint32_t trips = 500;
            for (uint32_t t = 0; t < trips; ++t) {
                PathPoint a, b;
                a.path = PathId(inLargest[rng() % inLargest.size()]);
                a.along = G.paths[a.path.index].length * (rng() % 1000) / 1000.0;
                b.path = PathId(inLargest[rng() % inLargest.size()]);
                b.along = G.paths[b.path.index].length * (rng() % 1000) / 1000.0;
                const route::Route rt = route::findRoute(m, bounds, car, a, b);
                found += rt.found ? 1u : 0u;
                fold = (fold ^ route::routeHash(rt)) * 1099511628211ull;
            }
            std::printf("      %s: cars: %u of %u trips routed; the routes folded: 0x%016llx\n", level, found, trips,
                        static_cast<unsigned long long>(fold));
            CHECK(found == trips);
        }
        // WALKERS: fixed-seed trips between random points on the sidewalks of the largest part on foot. Every one is found;
        // the time each search takes, and how far the route is against the straight line between its ends (the detour)
        {
            const route::RouteWalker walker = route::bindWalker(m, "walker", kTestPace);
            const ReachableParts parts = reachableWalks(G, walker.open);
            std::vector<uint32_t> sidewalks;
            std::vector<uint32_t> inPart(parts.sizes.size(), 0);
            for (uint32_t w = 0; w < G.walks.size(); ++w)
                if (m.walkKind(WalkId(w)) == "sidewalk") ++inPart[parts.partOf[w]];
            uint32_t largest = 0;
            for (uint32_t p = 0; p < inPart.size(); ++p)
                if (inPart[p] > inPart[largest]) largest = p;
            for (uint32_t w = 0; w < G.walks.size(); ++w)
                if (m.walkKind(WalkId(w)) == "sidewalk" && parts.partOf[w] == largest && G.walks[w].length > 0) sidewalks.push_back(w);
            std::mt19937 rng(4321u);
            const uint32_t trips = 1000;
            uint32_t found = 0, absurd = 0;
            std::vector<double> micros, detours, metres, crossings;
            const auto pointOn = [&]() {
                WalkPoint p;
                p.walk = WalkId(sidewalks[rng() % sidewalks.size()]);
                p.along = G.walks[p.walk.index].length * (rng() % 1000) / 1000.0;
                return p;
            };
            uint64_t fold = 1469598103934665603ull;
            for (uint32_t t = 0; t < trips; ++t) {
                const WalkPoint a = pointOn(), b = pointOn();
                const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
                const route::WalkRoute rt = route::findWalkRoute(m, walker, a, b);
                micros.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
                if (!rt.found) {
                    std::printf("        NOT FOUND: %s\n", rt.whyWords.c_str());
                    continue;
                }
                ++found;
                fold = (fold ^ static_cast<uint64_t>(rt.metres * 1000)) * 1099511628211ull;
                fold = (fold ^ rt.steps.size()) * 1099511628211ull;
                const Real straight = distance(G.pointAt(a.walk, a.along), G.pointAt(b.walk, b.along));
                metres.push_back(rt.metres);
                crossings.push_back(static_cast<double>(rt.crossings.size()));
                if (straight > 50) {
                    detours.push_back(rt.metres / straight);
                    if (rt.metres / straight > 4) {
                        ++absurd;
                        std::printf("        a long way round: %.0f m for %.0f m straight, (%.1f, %.1f) to (%.1f, %.1f), %zu crossings\n", rt.metres, straight,
                                    G.pointAt(a.walk, a.along).x, G.pointAt(a.walk, a.along).y, G.pointAt(b.walk, b.along).x,
                                    G.pointAt(b.walk, b.along).y, rt.crossings.size());
                    }
                }
            }
            // the same trips again: the same routes
            std::mt19937 again(4321u);
            rng = again;
            uint64_t fold2 = 1469598103934665603ull;
            for (uint32_t t = 0; t < trips; ++t) {
                const WalkPoint a = pointOn(), b = pointOn();
                const route::WalkRoute rt = route::findWalkRoute(m, walker, a, b);
                if (!rt.found) continue;
                fold2 = (fold2 ^ static_cast<uint64_t>(rt.metres * 1000)) * 1099511628211ull;
                fold2 = (fold2 ^ rt.steps.size()) * 1099511628211ull;
            }
            const auto at = [](std::vector<double> v, double q) {
                if (v.empty()) return 0.0;
                std::sort(v.begin(), v.end());
                return v[static_cast<std::size_t>(q * static_cast<double>(v.size() - 1))];
            };
            std::printf("      %s: walkers: %zu sidewalks in the largest part on foot; %u of %u trips routed (%.1f%%); folded 0x%016llx\n",
                        level, sidewalks.size(), found, trips, 100.0 * found / trips, static_cast<unsigned long long>(fold));
            std::printf("        search time: median %.0f us, 95th %.0f us, worst %.0f us\n", at(micros, 0.5), at(micros, 0.95), at(micros, 1.0));
            std::printf("        route length: median %.0f m, 95th %.0f m; crossings: median %.0f, 95th %.0f\n", at(metres, 0.5),
                        at(metres, 0.95), at(crossings, 0.5), at(crossings, 0.95));
            std::printf("        detour (route / straight line, ends over 50 m apart, %zu trips): median %.2f, 95th %.2f, worst %.2f; over 4: %u\n",
                        detours.size(), at(detours, 0.5), at(detours, 0.95), at(detours, 1.0), absurd);
            CHECK(found == trips);
            CHECK(fold == fold2);
        }
    }
}

// ---- 36. where a place joins the walks: one query (ADR-0468, issue #300) -------------------------------------------------

namespace {
// The first street of a road-file edge (none: it has none).
ModelStreetId streetOfRoad(const RoadModel& m, const std::string& road) {
    for (uint32_t s = 0; s < m.streets.size(); ++s)
        if (m.streets[s].road == road) return ModelStreetId(s);
    return ModelStreetId();
}
bool sameJoin(const WalkJoin& a, const WalkJoin& b) {
    return a.at == b.at && a.street == b.street && a.dir == b.dir && a.station == b.station && a.point.x == b.point.x &&
           a.point.y == b.point.y && a.held == b.held && a.across == b.across && a.searched == b.searched &&
           a.distance == b.distance && a.part == b.part && a.cutOff == b.cutOff && a.why == b.why;
}
void printJoin(const char* what, const RoadModel& m, const WalkJoin& j) {
    if (!j.valid()) {
        std::printf("      %s: none (%s)\n", what, walkJoinWhyWord(j.why));
        return;
    }
    std::printf("      %s: walk %u (%s) %.2f m along, street %s kerb %+d, station %.2f, at (%.2f, %.2f); held %.2f m%s%s, %.2f m away\n",
                what, j.at.walk.index, m.walkKind(j.at.walk).c_str(), j.at.along, m.streets[j.street.index].id.c_str(), j.dir,
                j.station, j.point.x, j.point.y, j.held, j.across ? ", across" : "", j.cutOff ? ", cut off" : "", j.distance);
}
}  // namespace

TEST_CASE(road_model_36_where_a_place_joins_the_walks_is_one_query) {
    // A T (walkTee): the east road runs from the junction at (0, 0) to (100, 0), its station x; local, 5 m sidewalks
    BuiltScene s = builtScene(walkTee());
    CHECK(s.report.error.empty());
    const LaneGraph& G = s.m.laneLevel;
    const ModelStreetId east = streetOfRoad(s.m, "east");
    const uint32_t eastNear = sidewalkOf(s.m, "east", +1), eastFar = sidewalkOf(s.m, "east", -1);
    CHECK(east.valid() && eastNear != kNoIndex && eastFar != kNoIndex);
    if (!east.valid() || eastNear == kNoIndex || eastFar == kNoIndex) return;
    const std::vector<uint8_t> open = s.m.openTo("walker").walks;
    const ReachableParts parts = reachableWalks(G, open);

    // A DOOR ON A STATED STREET: east, station 50, each kerb. It lands on that kerb's sidewalk, at station 50, where the
    // sidewalk's middle is (the model's sidewalkBeside), 50 m less the kerb end's station along the walk; in the one part.
    for (const int kerb : {+1, -1}) {
        const RoadFrontage f = s.m.frontageOn(east, 50.0, kerb);
        const WalkJoin j = s.m.walkJoinOn(f, open, &parts);
        printJoin(kerb > 0 ? "east 50 along" : "east 50 against", s.m, j);
        Vec2 middle;
        CHECK(s.m.sidewalkBeside(f, middle));
        CHECK(j.valid() && j.why == WalkJoinWhy::None);
        CHECK(j.at.walk.index == (kerb > 0 ? eastNear : eastFar));
        CHECK(j.street == east && j.dir == kerb && approxEqual(j.station, 50.0, kWrittenStation));
        CHECK(j.held == 0 && !j.across && !j.searched && j.distance == 0 && !j.cutOff && j.part == parts.largest);
        CHECK(approxEqual(j.at.along, 50.0 - G.walks[j.at.walk.index].stationFrom, kWrittenStation));
        CHECK(distance(j.point, middle) < 0.01);
        CHECK(distance(G.pointAt(j.at.walk, j.at.along), j.point) < kSamePoint);
    }
    // A DOOR AT THE CORNER: east, station 1, nearer the junction than where a person steps off the kerb. It joins the
    // sidewalk's end there (along 0), held by the metres to the kerb end, where the corner walk and the crossing meet it.
    {
        const WalkJoin j = s.m.walkJoinOn(s.m.frontageOn(east, 1.0, +1), open, &parts);
        printJoin("east 1 along (the corner)", s.m, j);
        const Walk& k = G.walks[eastNear];
        CHECK(j.valid() && j.at.walk.index == eastNear && j.at.along == 0 && !j.across);
        CHECK(approxEqual(j.station, k.stationFrom, kWrittenStation) && approxEqual(j.held, k.stationFrom - 1.0, kWrittenStation));
        CHECK(j.held > 1.0);
        uint32_t corners = 0;
        for (uint32_t e = k.atStart.first; e < k.atStart.end(); ++e) corners += s.m.walkKind(G.walkJoins[e].walk) == "corner" ? 1u : 0u;
        CHECK(corners >= 1u);
        // ...and one past the far end of the road, held back to the sidewalk's other end
        const WalkJoin far = s.m.walkJoinOn(s.m.frontageOn(east, 100.0, +1), open, &parts);
        CHECK(far.valid() && far.at.walk.index == eastNear && approxEqual(far.at.along, k.length, kWrittenStation));
    }
    // THE FALLBACK BY POSITION: a point 15 m out from east's centre line at station 50, on the +1 kerb's side, with nothing
    // stated: the same walk and station, `searched`, and the 6.5 m from the point to the sidewalk's middle reported.
    {
        const RoadFrontage f = s.m.frontageOn(east, 50.0, +1);
        const Vec2 p = f.on + f.towards * 15.0;
        const WalkJoin stated = s.m.walkJoinOn(f, open, &parts), near = s.m.walkJoinNear(p, open, &parts);
        printJoin("15 m out at east 50 (by position)", s.m, near);
        CHECK(near.valid() && near.searched && near.at.walk == stated.at.walk && approxEqual(near.at.along, stated.at.along, kWrittenStation));
        CHECK(approxEqual(near.distance, distance(p, near.point), kSamePoint) && near.distance > 6.4 && near.distance < 6.6);
        // DETERMINISM: the same answer every time it is asked, and on the same scene built again
        CHECK(sameJoin(near, s.m.walkJoinNear(p, open, &parts)) && sameJoin(stated, s.m.walkJoinOn(f, open, &parts)));
        BuiltScene again = builtScene(walkTee());
        const std::vector<uint8_t> open2 = again.m.openTo("walker").walks;
        const ReachableParts parts2 = reachableWalks(again.m.laneLevel, open2);
        CHECK(sameJoin(near, again.m.walkJoinNear(p, open2, &parts2)));
    }
    // ONLY WALKS THE MOVER MAY USE. Its own kerb's sidewalk shut: the other kerb's, `across` (the person crosses there).
    // Every walk shut: none, and why. A mover may use nothing: the vehicles.
    {
        std::vector<uint8_t> shut = open;
        shut[eastNear] = 0;
        const WalkJoin j = s.m.walkJoinOn(s.m.frontageOn(east, 50.0, +1), shut);
        CHECK(j.valid() && j.at.walk.index == eastFar && j.across && j.dir == -1);
        const WalkJoin none = s.m.walkJoinOn(s.m.frontageOn(east, 50.0, +1), std::vector<uint8_t>(open.size(), 0));
        CHECK(!none.valid() && none.why == WalkJoinWhy::NotOpen);
        CHECK(!s.m.walkJoinNear(Vec2(50, 15), s.m.openTo("vehicles").walks).valid());
        // a part cut off: only the sidewalks open, each its own part; a door on a sidewalk not in the largest says so
        std::vector<uint8_t> sidewalksOnly(open.size(), 0);
        for (uint32_t w = 0; w < G.walks.size(); ++w) sidewalksOnly[w] = s.m.walkKind(WalkId(w)) == "sidewalk" ? 1 : 0;
        const ReachableParts apart = reachableWalks(G, sidewalksOnly);
        const WalkJoin nearKerb = s.m.walkJoinOn(s.m.frontageOn(east, 50.0, +1), sidewalksOnly, &apart),
                       farKerb = s.m.walkJoinOn(s.m.frontageOn(east, 50.0, -1), sidewalksOnly, &apart);
        CHECK(nearKerb.valid() && farKerb.valid() && nearKerb.part != farKerb.part);
        CHECK(nearKerb.cutOff != (nearKerb.part == apart.largest) && farKerb.cutOff != (farKerb.part == apart.largest));
        CHECK(nearKerb.cutOff || farKerb.cutOff);
        // a frontage on no street (a stretch inside a junction, or none at all)
        CHECK(s.m.walkJoinOn(RoadFrontage(), open).why == WalkJoinWhy::NoStreet);
    }
    // A STREET WITH A SIDEWALK ON ONE KERB ONLY: the T's south road states "along". A door on its other kerb joins the
    // sidewalk across the road; one on its own kerb joins its own. With "none", no walk: none, and why.
    {
        BuiltScene one = builtScene(walkTee("along"));
        CHECK(one.report.error.empty());
        const ModelStreetId south = streetOfRoad(one.m, "south");
        const uint32_t along = sidewalkOf(one.m, "south", +1);
        CHECK(south.valid() && along != kNoIndex && sidewalkOf(one.m, "south", -1) == kNoIndex);
        const std::vector<uint8_t> o = one.m.openTo("walker").walks;
        const WalkJoin bare = one.m.walkJoinOn(one.m.frontageOn(south, 50.0, -1), o), own = one.m.walkJoinOn(one.m.frontageOn(south, 50.0, +1), o);
        printJoin("one-sided south 50, the bare kerb", one.m, bare);
        CHECK(bare.valid() && bare.at.walk.index == along && bare.across && bare.dir == +1 && approxEqual(bare.station, 50.0, kWrittenStation));
        CHECK(own.valid() && own.at.walk.index == along && !own.across && own.dir == +1);
        BuiltScene none = builtScene(walkTee("none"));
        const ModelStreetId southNone = streetOfRoad(none.m, "south");
        const WalkJoin j = none.m.walkJoinOn(none.m.frontageOn(southNone, 50.0, -1), none.m.openTo("walker").walks);
        printJoin("no sidewalk, south 50", none.m, j);
        CHECK(southNone.valid() && !j.valid() && j.why == WalkJoinWhy::NoWalk);
    }
    // A RAMP, WHICH HAS NO SIDEWALK (merge_taper: three ramps from local feeder streets onto a freeway). No walk is laid
    // along a ramp or the freeway, so nothing joins one. A point on the middle of ramp_one's lane: the search by position
    // never answers a ramp or a freeway (frontageAt: what a building may front), so it joins the nearest street a building
    // may front, a feeder, and says how far that is. A frontage stated on a ramp's street, if it has one: no walk.
    {
        nlohmann::json g = nlohmann::json::parse(slurp(std::string(RT_SOURCE_DIR) + "/assets/lanelab/merge_taper.json"));
        g.erase("junctions");
        g.erase("streets");
        writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
        for (nlohmann::json& jn : g["junctions"]) jn["control"] = "unmarked";   // (as road_model_13 lays its lane level)
        g["lane_rules"] = {{"solid_before_stop_line", 20.0}};
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        LaneGraphBuild lg;
        const RoadModel m = assembleRoadModel(*r, oneCar(), &lg);
        CHECK(lg.error.empty());
        if (!lg.error.empty()) std::printf("      merge_taper's lane level: %s\n", lg.error.c_str());
        const std::vector<uint8_t> o = m.openTo("walker").walks;
        uint32_t onFast = 0, laid = 0;
        for (const Walk& k : m.laneLevel.walks) {
            if (k.street >= m.streets.size()) continue;
            ++laid;
            const std::string& road = m.streets[k.street].road;
            onFast += road.rfind("ramp_", 0) == 0 || road == "mainline" ? 1u : 0u;
        }
        CHECK(laid > 0u && onFast == 0u);
        const ModelLane& ramp = m.lanes[laneNamed(m, "ramp_one.f0").index];
        const Vec2 mid = m.lanePoints()[ramp.curve.first + ramp.curve.count / 2];
        const WalkJoin j = m.walkJoinNear(mid, o);
        printJoin("on ramp_one's middle (by position)", m, j);
        CHECK(j.valid() && j.searched && m.streets[j.street.index].road.rfind("feed_", 0) == 0 && j.distance > 0);
        const ModelStreetId rampStreet = streetOfRoad(m, "ramp_one");
        CHECK(rampStreet.valid());
        if (rampStreet.valid()) {
            const WalkJoin onRamp = m.walkJoinOn(m.frontageOn(rampStreet, m.streets[rampStreet.index].from + 10.0, +1), o);
            printJoin("stated on ramp_one's street", m, onRamp);
            CHECK(!onRamp.valid() && onRamp.why == WalkJoinWhy::NoWalk);
        }
    }
}

// ---- 37. a street ending under the freeway is a dead end: the crossing policy says what meets at grade (ADR-0469) -------

TEST_CASE(road_model_37_a_street_ending_under_the_freeway_is_a_dead_end) {
    // THE CROSSING POLICY, one statement (road_design.h meetAtGrade), by what the two roads are
    {
        RoadLabGraph g = RoadLabGraph::fromJson(walkTee());
        const auto edgeOf = [&](const char* cls) {
            EdgeSpec e = g.edges.front();
            e.cls = cls;
            return e;
        };
        const EdgeSpec local = edgeOf("local"), freeway = edgeOf("freeway"), rail = edgeOf("rail"), footway = edgeOf("footway");
        CHECK(meetAtGrade(g, local, local, true, false));          // two streets: a junction
        CHECK(!meetAtGrade(g, local, freeway, true, false));       // a street at the freeway: a bridge
        CHECK(!meetAtGrade(g, freeway, local, false, true));
        CHECK(!meetAtGrade(g, local, rail, true, false));          // at a track: a bridge
        CHECK(meetAtGrade(g, footway, local, true, false));        // a footway ending on a road: its way in
        CHECK(!meetAtGrade(g, footway, local, false, false));      // a footway crossing it: a footbridge
    }
    // A SPUR ENDING INSIDE A ROAD'S BAND, with no junction stated (walkTee's spur, 6 m short of the east road's centre
    // line): a street the planner left unstated where the road is one it meets at grade; a dead end, joined across by a
    // turn-round, where that road is a freeway (above it, by the class's own `limited_access`). The design stage asks the
    // same one test (ADR-0470): under the freeway the spur's end is open (brought down to the ground), beside a street it
    // is met. Roads whose ends meet at a node meet whatever they are: east and west carry on into each other at (0, 0) as
    // freeways too (were the crossing policy asked there, the freeway's own ends would be open and brought down).
    for (const char* cls : {"local", "freeway"}) {
        nlohmann::json file = walkTee(std::string(), true);
        for (nlohmann::json& e : file["edges"])
            if (e["id"] == "east" || e["id"] == "west") e["class"] = cls;
        RoadLabGraph g;
        const RoadModel m = walkModel(file, g);
        const ModelStreetId spur = streetNamed(m, "spur/1");
        CHECK(spur.valid());
        if (!spur.valid()) return;
        const bool underFreeway = std::string(cls) == "freeway";
        const StreetEndKind kind = streetEndKind(g, m.streets[spur.index], 1);
        std::printf("      the spur's end inside the %s road's band: %s\n", cls, streetEndKindWord(kind));
        CHECK(kind == (underFreeway ? StreetEndKind::DeadEnd : StreetEndKind::MeetsAnother));
        const EdgeSpec& spurEdge = g.edges[static_cast<size_t>(g.index.at("spur"))];
        CHECK(roadEndMeetsAnother(g, spurEdge, 1) == !underFreeway);   // at grade: only the street
        CHECK(roadEndMeetsAnother(g, g.edges[static_cast<size_t>(g.index.at("east"))], 0));   // a node: met, either class
        CHECK(roadEndMeetsAnother(g, g.edges[static_cast<size_t>(g.index.at("west"))], 1));
        {   // the design stage's open ends: the same answer
            RoadLabGraph d = g;
            (void)designRoads(d, [](double, double) { return 0.0; });
            const EdgeSpec& ds = d.edges[static_cast<size_t>(d.index.at("spur"))];
            CHECK(ds.openEnd[1] == underFreeway);
            CHECK(ds.openEnd[0]);   // its far end meets nothing: open either way
            CHECK(!d.edges[static_cast<size_t>(d.index.at("east"))].openEnd[0]);
            CHECK(!d.edges[static_cast<size_t>(d.index.at("west"))].openEnd[1]);
        }
        uint32_t spurEnds = 0;
        bool joinedAtTheFreeway = false;
        for (const ModelDeadEnd& d : m.deadEnds)
            if (d.street == spur) {
                ++spurEnds;
                joinedAtTheFreeway = joinedAtTheFreeway || (d.end == 1 && d.turnRound);
            }
        CHECK(spurEnds == (underFreeway ? 2u : 1u));
        CHECK(joinedAtTheFreeway == underFreeway);
    }
}

// ---- 38. the box edge at the angle the arms meet (ADR-0471) -------------------------------------------------------------

namespace {
// A local street east-west through (0, 0), and one more local road `slant` leaving the junction at `degrees` below east
// (into it: its spine runs in), laid out by the one-off tool as walkTee is. With `crossAt` > 0, a local road north from
// (crossAt, 0): a second junction along the street, `crossAt` metres east.
nlohmann::json sharpTee(double degrees, double crossAt = 0) {
    nlohmann::json g;
    g["name"] = "sharp_tee";
    auto road = [](const std::string& id, const Vec2& a, const Vec2& b) {
        return nlohmann::json{{"id", id}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}};
    };
    const Vec2 far = directionOf(-degrees) * 100.0;
    g["edges"] = nlohmann::json::array({road("west", Vec2(-100, 0), Vec2(0, 0)), road("slant", far, Vec2(0, 0))});
    if (crossAt > 0) {
        g["edges"].push_back(road("east", Vec2(0, 0), Vec2(crossAt, 0)));
        g["edges"].push_back(road("east2", Vec2(crossAt, 0), Vec2(100, 0)));
        g["edges"].push_back(road("north", Vec2(crossAt, 0), Vec2(crossAt, 100)));
    } else {
        g["edges"].push_back(road("east", Vec2(0, 0), Vec2(100, 0)));
    }
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    return g;
}
// Metres from p to the east road's centre line (the segment from the junction at (0, 0) east).
Real fromEastLine(const Vec2& p) { return p.x >= 0 ? std::fabs(p.y) : p.length(); }
const ModelArm* armOn(const ModelJunction& J, const std::string& road) {
    for (const ModelArm& a : J.arms)
        if (a.road == road) return &a;
    return nullptr;
}
}  // namespace

TEST_CASE(road_model_38_the_box_edge_at_the_angle_the_arms_meet) {
    const Real W = 6.0, w = 6.0;   // a local road's half travel width (road_classes.json: two 6 m lanes)
    // THE RULE, arm by arm (road_model_parts.h boxEdgeBack). A square crossroads: each arm set back by the crossing road's
    // half width, with or without headings -- the square rule's number, to the bit
    {
        std::vector<ArmMouth> arms;
        for (const Vec2& d : {Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0), Vec2(0, -1)}) arms.push_back(ArmMouth{static_cast<int>(arms.size() % 2), Vec2(), W, d});
        const ThroughPairs through = {{0, 2}, {1, 3}};
        std::vector<ArmMouth> noHeadings = arms;
        for (ArmMouth& a : noHeadings) a.dir = Vec2();
        for (std::size_t k = 0; k < 4; ++k) {
            CHECK(boxEdgeBack(arms, through, k) == W);
            CHECK(boxEdgeBack(noHeadings, through, k) == W);
            CHECK(squareBoxEdgeBack(arms, through, k) == W);
        }
        // headings written to the hundredth of a degree (directionOf(90) is a hair off square): square all the same
        for (std::size_t k = 0; k < 4; ++k) arms[k].dir = directionOf(90.0 * static_cast<double>(k));
        for (std::size_t k = 0; k < 4; ++k) CHECK(boxEdgeBack(arms, through, k) == W);
    }
    // A SHARP T from one point: the street through (east and west), a stem leaving 30 degrees off east. The stem's kerb on
    // the east side clears the street's carriageway at W / sin 30 + w / tan 30; the street's east arm is set back as far
    // by the stem; its west arm (150 degrees off the stem) by the square rule's W
    {
        const Real a = 30 * DEG_TO_RAD;
        const std::vector<ArmMouth> arms = {ArmMouth{0, Vec2(), W, Vec2(1, 0)}, ArmMouth{0, Vec2(), W, Vec2(-1, 0)},
                                            ArmMouth{1, Vec2(), w, directionOf(-30)}};
        const ThroughPairs through = {{0, 1}};
        const Real want = W / std::sin(a) + w / std::tan(a);
        std::printf("      a 30-degree T: the stem set back %.2f m (the square rule: %.2f m)\n", boxEdgeBack(arms, through, 2), W);
        CHECK(approxEqual(boxEdgeBack(arms, through, 2), want, 1e-9));
        CHECK(approxEqual(boxEdgeBack(arms, through, 0), want, 1e-9));
        CHECK(boxEdgeBack(arms, through, 1) == W);
        CHECK(boxEdgeSqueeze(arms, through, 2) == 0);
        // THE ROOM CAP: with 3 m of room, the stem stands back the square rule and 3 m, short of clear by the rest
        std::vector<ArmMouth> tight = arms;
        tight[2].room = 3;
        CHECK(approxEqual(boxEdgeBack(tight, through, 2), W + 3, 1e-9));
        CHECK(approxEqual(boxEdgeSqueeze(tight, through, 2), want - W - 3, 1e-9));
        tight[2].room = 0;
        CHECK(boxEdgeBack(tight, through, 2) == W);
    }
    // THE ANGLE CAP: a stem leaving 5 degrees off east is set back as if it met at kSharpestMeet (20 degrees), not 5
    {
        const std::vector<ArmMouth> arms = {ArmMouth{0, Vec2(), W, Vec2(1, 0)}, ArmMouth{0, Vec2(), W, Vec2(-1, 0)},
                                            ArmMouth{1, Vec2(), w, directionOf(-5)}};
        const Real c = kSharpestMeet * DEG_TO_RAD;
        const Real capped = W / std::sin(c) + w / std::tan(c);
        CHECK(approxEqual(boxEdgeBack(arms, {{0, 1}}, 2), capped, 1e-9));
        CHECK(boxEdgeBack(arms, {{0, 1}}, 2) < W / std::sin(5 * DEG_TO_RAD));
    }
    // AN ARM BESIDE IT BUT APART (a divided road's other carriageway, 14 m off, leaving the same way): its strip never
    // overlaps the arm's, so it sets it back by the square rule's W only
    {
        const std::vector<ArmMouth> arms = {ArmMouth{0, Vec2(0, 0), w, Vec2(1, 0)}, ArmMouth{1, Vec2(0, 14), W, Vec2(1, 0)}};
        CHECK(boxEdgeBack(arms, {}, 0) == W);
    }
    // THE ROOM between two facing arms: half what the square rule's stop lines and the storage leave, less a millimetre
    CHECK(approxEqual(armRoom(50, 15, 15, 6.5), 6.75 - kSamePoint, 1e-9));
    CHECK(armRoom(30, 15, 15, 6.5) == 0);

    // A SHARP T BUILT (a 30-degree stem): every corner walked, the stem's crosswalk kerb ends on drawn pavement and clear
    // of the street's carriageway, its stop line wholly ahead of the street's edge. (The square rule left the stem's kerb
    // end out in the asphalt of the sharp corner: no walk there.)
    {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(sharpTee(30)));
        uint32_t laid = 0;
        for (const WalkCorner& wc : r->walkCorners) {
            if (wc.lane < 0) std::printf("      no walk: arms %u-%u: %s\n", wc.armA, wc.armB, wc.why.c_str());
            laid += wc.lane >= 0 ? 1u : 0u;
            CHECK(wc.fault == WalkCornerFault::None);
        }
        CHECK(laid == 3u);
        const RoadModel m = assembleRoadModel(*r, oneCar());
        CHECK(m.junctions.size() == 1u);
        if (m.junctions.size() != 1u) return;
        const ModelJunction& J = m.junctions[0];
        const ModelArm* stem = armOn(J, "slant");
        CHECK(stem != nullptr);
        if (!stem) return;
        const uint32_t stemArm = static_cast<uint32_t>(stem - J.arms.data());
        const ModelCrosswalk* cw = m.crosswalkAcross(ModelJunctionId(0), stemArm);
        CHECK(cw != nullptr && cw->pavements);
        if (cw)
            for (const Vec2& e : cw->ends) {
                std::printf("      the stem's kerb end (%.2f, %.2f): %.2f m from the street's centre line (its half width %.2f)\n", e.x, e.y,
                            fromEastLine(e), W);
                CHECK(fromEastLine(e) > W);
            }
        const EdgeSpec& slant = r->graph.edges[static_cast<std::size_t>(r->graph.index.at("slant"))];
        const Vec2 at = pointAt(slant.xy, slant.s, stem->stopLine);
        const Vec2 side = perp(tangentAtStation(slant.xy, slant.s, stem->stopLine)) * w;
        CHECK(fromEastLine(at + side) > W && fromEastLine(at - side) > W);
        for (const std::string& p : m.problems) CHECK(p.find("too close to junction") == std::string::npos);
    }
    // A SQUARE T (walkTee) BUILT: the box edges the square rule gives, to the bit
    {
        RoadLabGraph g;
        const RoadModel m = walkModel(walkTee(), g);
        for (const ModelArm& a : m.junctions[0].arms) CHECK(a.boxEdge == a.station + (a.spineIn ? -W : W));
    }
    // THE ROOM CAP ON A SCENE: the sharp T with a second junction 40 m east along the street. Kerbs clear, the street's east
    // arm would stand back about 22 m, leaving no room for a car before the next junction's stop line; the room caps it,
    // the grouping still finds two junctions (it asks the same rule), the two stop lines leave the storage between them,
    // and the arm is said as a planning fault (junctions too close for the angle their roads meet at)
    {
        const nlohmann::json file = sharpTee(30, 40);
        CHECK(file["junctions"].size() == 2u);
        RoadLabGraph g;
        const RoadModel m = walkModel(file, g);
        const ModelJunction* A = nullptr;
        const ModelJunction* B = nullptr;
        for (const ModelJunction& J : m.junctions) (armOn(J, "slant") ? A : B) = &J;
        CHECK(A && B);
        if (!A || !B) return;
        const ModelArm* east = armOn(*A, "east");
        const ModelArm* eastAtB = armOn(*B, "east");
        CHECK(east && eastAtB);
        if (!east || !eastAtB) return;
        const Real between = eastAtB->stopLine - east->stopLine;
        std::printf("      east arm: box edge %.2f, stop line %.2f; the next junction's stop line %.2f: %.2f m between (storage %.1f)\n",
                    east->boxEdge, east->stopLine, eastAtB->stopLine, between, g.rules.storage);
        CHECK(between >= g.rules.storage);
        CHECK(approxEqual(east->boxEdge, W + armRoom(40, W + kStopLineBack, W + kStopLineBack, g.rules.storage), 1e-9));
        uint32_t faults = 0;
        for (const std::string& p : m.problems) {
            if (p.find("too close to junction") == std::string::npos) continue;
            std::printf("      %s\n", p.c_str());
            faults += p.find("junction " + A->id + ", arm ") == 0 && p.find("(east): too close to junction " + B->id) != std::string::npos ? 1u : 0u;
        }
        CHECK(faults == 1u);
    }
}

// THE SHIPPED ROAD FILES' STOP LINES (ADR-0471): along every road, no arm's stop line reaches the next junction's facing
// one with less than the storage a car needs between them (so none passes it), and no stop line reaches past any point of
// another junction on its road (an arm's mouth or the end of a stretch inside it). The arms the room stops short of
// clear, the planning fault of two junctions too close for the angle their roads meet at, by count and by junction.
TEST_CASE(road_model_39_the_shipped_stop_lines_stand_short_of_the_next_junction) {
    const std::map<std::string, std::set<std::string>> knownSqueezed = {
        {"traffic_lab", {"j0:5", "j0:6", "j22:0", "j24:0", "j37:3", "j70:0", "j70:1", "j84:3", "j84:4"}},
        {"river_town", {"j135:4", "j135:5", "j14:0", "j35:2", "j35:3", "j37:0", "j37:1", "j43:0", "j43:4", "j56:0", "j56:1", "j68:0", "j6:3",
                        "j72:0", "j72:1", "j73:0", "j75:0", "j75:2", "j7:0", "j7:1", "j8:2", "j8:3", "j90:0", "j90:4", "j91:2", "j91:3"}}};
    for (const char* level : {"traffic_lab", "river_town", "agent_lab"}) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/" + level + "_lanelab.json";
        RoadLabGraph g = RoadLabGraph::fromJson(nlohmann::json::parse(slurp(path)), path.substr(0, path.find_last_of('/')));
        (void)expand(g);
        const RoadModel m = plannedRoadModel(g);
        struct Point { uint32_t junction; Real station; int arm; bool spineIn; };   // arm -1: an inside stretch's end
        std::map<std::string, std::vector<Point>> onRoad;
        for (uint32_t j = 0; j < m.junctions.size(); ++j) {
            const ModelJunction& J = m.junctions[j];
            for (uint32_t a = 0; a < J.arms.size(); ++a) onRoad[J.arms[a].road].push_back(Point{j, J.arms[a].station, static_cast<int>(a), J.arms[a].spineIn});
            for (const ModelStretch& in : J.inside)
                for (const Real s : {in.from, in.to}) onRoad[in.road].push_back(Point{j, s, -1, false});
        }
        uint32_t pairs = 0, short_ = 0, past = 0;
        for (uint32_t j = 0; j < m.junctions.size(); ++j)
            for (const ModelArm& a : m.junctions[j].arms) {
                if (!stated(a.stopLine)) continue;
                const Real away = a.spineIn ? -1 : 1, back = (a.stopLine - a.station) * away;
                const Point* facing = nullptr;
                for (const Point& p : onRoad[a.road]) {
                    const Real gap = (p.station - a.station) * away;
                    if (p.junction == j || !(gap > 0)) continue;
                    if (back >= gap) ++past;
                    if (p.arm >= 0 && p.spineIn != a.spineIn && (!facing || gap < (facing->station - a.station) * away)) facing = &p;
                }
                if (!facing) continue;
                ++pairs;
                const ModelArm& b = m.junctions[facing->junction].arms[static_cast<std::size_t>(facing->arm)];
                const Real backB = (b.stopLine - b.station) * (b.spineIn ? -1 : 1);
                const Real between = (facing->station - a.station) * away - back - backB;
                if (between < g.rules.storage - kSamePoint) ++short_;
            }
        std::set<std::string> squeezed;
        for (const std::string& p : m.problems) {
            const std::size_t at = p.find("): too close to junction ");
            if (at == std::string::npos) continue;
            const std::size_t comma = p.find(", arm ");
            squeezed.insert(p.substr(std::string("junction ").size(), comma - std::string("junction ").size()) + ":" +
                            p.substr(comma + 6, p.find(' ', comma + 6) - comma - 6));
        }
        std::printf("      %s: %u arms facing the next junction along their road, %u with less than the storage between the stop "
                    "lines, %u stop lines past another junction's point; %zu arms too close to the next junction for their angle\n",
                    level, pairs, short_, past, squeezed.size());
        CHECK(pairs > 500u && short_ == 0u && past == 0u);
        const std::string known = std::string(level) == "agent_lab" ? "river_town" : level;   // (agent_lab has river_town's roads)
        CHECK(squeezed == knownSqueezed.at(known));
    }
}

// ---- 40. the bands at a kink: each road's asphalt, shoulder and sidewalk mitred on the bisector (ADR-0472) ---------------

namespace {
// TWO LOCAL ROADS MEETING END TO END AT (0, 0): `a` from the west, `b` leaving `turnDeg` counter-clockwise of east, so the
// outside of the bend lies to the south (a's right, b's right). The file says a carries on as b. Both carry a planned
// profile, so their heights are as given: `a` at 0 m and `b` at `riseB`.
nlohmann::json kinkScene(double turnDeg, double riseB = 0) {
    nlohmann::json g;
    g["name"] = "kink";
    const Vec2 far = directionOf(turnDeg) * 100.0;
    auto road = [](const std::string& id, const Vec2& p, const Vec2& q, double z) {
        return nlohmann::json{{"id", id}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{p.x, p.y}, {q.x, q.y}}}}},
                              {"profile", {{"step", 1.0}, {"z", std::vector<double>(101, z)}}}};
    };
    g["edges"] = nlohmann::json::array({road("a", Vec2(-100, 0), Vec2(0, 0), 0.0), road("b", Vec2(0, 0), far, riseB)});
    g["junctions"] = nlohmann::json::array();
    g["streets"] = nlohmann::json::array(
        {{{"id", "a"}, {"road", "a"}, {"from", 0.0}, {"to", 100.0}, {"junctions", {nullptr, nullptr}}, {"carries_on", {nullptr, "b"}}},
         {{"id", "b"}, {"road", "b"}, {"from", 0.0}, {"to", 100.0}, {"junctions", {nullptr, nullptr}}, {"carries_on", {"a", nullptr}}}});
    return g;
}
// THE DRAWN PAVEMENT OF A BUILD AGAIN, from its own lanes and heights: as the build drew it, or (`stated` false) with the
// file's statements of where road ends meet -- its streets and junctions -- taken away, which is how every kink was drawn
// before ADR-0472 (each band cut square at its road's end).
Pavement redrawn(const Result& r, bool stated) {
    RoadLabGraph g = r.graph;
    if (!stated) {
        g.streets.clear();
        g.junctions.clear();
    }
    Pavement p;
    buildFootprints(g, r.lanes, *r.heights, p);
    return p;
}
// The way out of kinkScene's bend from (0, 0): half way round its outside, from a's way back west to b's way out.
Vec2 kinkOutside(double turnDeg) { return directionOf(-90.0 + turnDeg * 0.5); }
// A T whose street kinks at the junction: `west` ends at (0, 0) from the west, `east` leaves it `turnDeg` clockwise of
// east, and the stem `south` comes up from the south, on the inside of the bend. Nothing stands on the north side, the
// outside. Laid out as the layout tool would state it.
nlohmann::json kinkTee(double turnDeg) {
    nlohmann::json g;
    g["name"] = "kink_tee";
    auto road = [](const std::string& id, const Vec2& p, const Vec2& q) {
        return nlohmann::json{{"id", id}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{p.x, p.y}, {q.x, q.y}}}}}};
    };
    g["edges"] = nlohmann::json::array({road("west", Vec2(-100, 0), Vec2(0, 0)), road("east", Vec2(0, 0), directionOf(-turnDeg) * 100.0),
                                        road("south", Vec2(0, -100), Vec2(0, 0))});
    writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    return g;
}
// Two polygon sets the same to the bit (a byte-for-byte check of the drawn pavement, so an exact compare is the point).
bool samePolySets(const PolySet& x, const PolySet& y) {
    auto sameRing = [](const Ring& p, const Ring& q) {
        if (p.size() != q.size()) return false;
        for (std::size_t i = 0; i < p.size(); ++i)
            if (!(p[i].x == q[i].x && p[i].y == q[i].y)) return false;
        return true;
    };
    if (x.size() != y.size()) return false;
    for (std::size_t i = 0; i < x.size(); ++i) {
        if (!sameRing(x[i].outer, y[i].outer) || x[i].holes.size() != y[i].holes.size()) return false;
        for (std::size_t h = 0; h < x[i].holes.size(); ++h)
            if (!sameRing(x[i].holes[h], y[i].holes[h])) return false;
    }
    return true;
}
// How many points along the broken line `pts` (every 10 cm) lie off a polygon set.
uint32_t offAlong(const PolySet& s, const std::vector<Vec2>& pts) {
    uint32_t off = 0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const int n = std::max(1, static_cast<int>(distance(pts[i], pts[i + 1]) / 0.1));
        for (int k = 0; k <= n; ++k) off += contains(s, lerp(pts[i], pts[i + 1], static_cast<Real>(k) / n)) ? 0u : 1u;
    }
    return off;
}
}  // namespace

TEST_CASE(road_model_40_the_bands_are_mitred_at_a_kink) {
    // A CARRY-ON KINKED 40 DEGREES. The file says a carries on as b, so the two ends are one kink. Each band runs on to the
    // bisector on the outside of the bend: the carriageway is asphalt out to its mitre (no bay left for the asphalt's
    // rounding to bite out of the kerb), the sidewalk is unbroken from that kerb to its own mitre, the sidewalk is in one
    // piece each side, and a walk along the middle of the outside sidewalk stays on it across the kink. The same build drawn
    // again with nothing stated about where the ends meet shows the old wedge: bare ground across the bisector, the outside
    // sidewalk in two pieces.
    {
        const double turn = 40;
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(kinkScene(turn)));
        const Pavement drawn = redrawn(*r, true), unstated = redrawn(*r, false);
        const RoadLabGraph& g = r->graph;
        const std::vector<BandKink> kinks = bandKinks(g);
        CHECK(kinks.size() == 1u);
        if (kinks.size() != 1u) return;
        CHECK(samePolySets(drawn.sidewalk, r->pavement.sidewalk) && samePolySets(drawn.surface, r->pavement.surface));
        CHECK(g.edges[static_cast<std::size_t>(kinks[0].edgeA)].id == "a" && kinks[0].endA == 1);
        CHECK(g.edges[static_cast<std::size_t>(kinks[0].edgeB)].id == "b" && kinks[0].endB == 0);
        CHECK(approxEqual(kinks[0].turn, turn * DEG_TO_RAD, 1e-9));
        const EdgeSpec& a = g.edges[static_cast<std::size_t>(g.index.at("a"))];
        const Real kerb = g.carriagewayHalfWidth(a), outer = g.sidewalkOuterHalfWidth(a), middle = (kerb + outer) * 0.5;
        const Real reach = 1.0 / std::cos(turn * 0.5 * DEG_TO_RAD);   // how far out along the bisector a band's edge meets its twin's, per metre
        const Vec2 out = kinkOutside(turn);
        CHECK(contains(r->pavement.surface, out * (kerb * reach - 0.05)));
        CHECK(!contains(r->pavement.sidewalk, out * (kerb * reach - 0.05)));   // (the kerb is not drawn into the carriageway)
        CHECK(!contains(unstated.surface, out * (kerb * reach - 0.05)));
        std::vector<Vec2> across;
        for (Real d = kerb * reach + 0.05; d < outer * reach - 0.05; d += 0.1) across.push_back(out * d);
        uint32_t bare = 0, bareBefore = 0;
        for (const Vec2& p : across) {
            bare += contains(r->pavement.sidewalk, p) ? 0u : 1u;
            bareBefore += contains(unstated.sidewalk, p) ? 0u : 1u;
        }
        std::printf("      a 40-degree carry-on: %zu points across the bisector, kerb to back: %u off the sidewalk (stated nothing: %u)\n",
                    across.size(), bare, bareBefore);
        CHECK(!across.empty() && bare == 0u && bareBefore == across.size());
        CHECK(!contains(r->pavement.sidewalk, out * (outer * reach + 0.05)));
        std::printf("      sidewalk pieces: %zu (stated nothing: %zu)\n", r->pavement.sidewalk.size(), unstated.sidewalk.size());
        CHECK(r->pavement.sidewalk.size() == 2u && unstated.sidewalk.size() == 3u);
        // the outside sidewalk's middle: along a, to the bisector, along b
        const Vec2 bOut = directionOf(turn);
        const std::vector<Vec2> walkLine = {Vec2(-20, -middle), out * (middle * reach), bOut * 20.0 - perp(bOut) * middle};
        CHECK(offAlong(r->pavement.sidewalk, walkLine) == 0u);
        CHECK(offAlong(unstated.sidewalk, walkLine) > 0u);
    }
    // A T WHOSE STREET KINKS 25 DEGREES AT THE JUNCTION, the stem on the inside of the bend: the two street ends with
    // nothing between them on the outside are a kink, and the corner walk round the outside, which found the drawn sidewalk
    // in two pieces before (ADR-0469's B2), is laid. Every corner has its walk, on the drawn sidewalk.
    {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(kinkTee(25)));
        const std::vector<BandKink> kinks = bandKinks(r->graph);
        CHECK(kinks.size() == 1u);
        if (kinks.size() == 1u)
            CHECK(r->graph.edges[static_cast<std::size_t>(kinks[0].edgeA)].id == "east" &&
                  r->graph.edges[static_cast<std::size_t>(kinks[0].edgeB)].id == "west");
        const PreparedSet walk(drawnWalk(*r));
        uint32_t laid = 0;
        for (const WalkCorner& wc : r->walkCorners) {
            if (wc.lane < 0) std::printf("      no walk: arms %u-%u: %s\n", wc.armA, wc.armB, wc.why.c_str());
            CHECK(wc.fault == WalkCornerFault::None);
            if (wc.lane < 0) continue;
            ++laid;
            CHECK(offTheWalk(r->lanes.walks[static_cast<std::size_t>(wc.lane)], walk) == 0u);
        }
        CHECK(laid == 3u);
    }
    // A SHARP KINK, 150 DEGREES: the mitre's point would reach out 3.9 times the band's half width; it is cut off square
    // across the bisector at kMitreLimit times it. The sidewalk is drawn up to the cut and not past it.
    {
        const double turn = 150;
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(kinkScene(turn)));
        const RoadLabGraph& g = r->graph;
        const std::vector<BandKink> kinks = bandKinks(g);
        CHECK(kinks.size() == 1u);
        if (kinks.size() != 1u) return;
        const EdgeSpec& a = g.edges[static_cast<std::size_t>(g.index.at("a"))];
        const Real outer = g.sidewalkOuterHalfWidth(a), cap = kMitreLimit * outer;
        const Vec2 out = kinkOutside(turn);
        Real farthest = 0;
        for (const Ring& piece : mitrePieces(g, kinks[0], outer, outer))
            for (const Vec2& q : piece) farthest = std::max(farthest, dot(q, out));
        std::printf("      a 150-degree kink: the sidewalk's mitre reaches %.2f m out (cut at %.2f; uncut it would reach %.2f)\n", farthest,
                    cap, outer / std::cos(turn * 0.5 * DEG_TO_RAD));
        CHECK(approxEqual(farthest, cap, kSamePoint));
        CHECK(contains(r->pavement.sidewalk, out * (cap - 0.1)));
        CHECK(!contains(r->pavement.sidewalk, out * (cap + 0.1)));
    }
    // ENDS AT DIFFERENT HEIGHTS ARE NOT JOINED: the same 40-degree carry-on with b 5 m up (more than sameLevelDz) is no
    // kink, and the outside stays as each road draws it
    {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(kinkScene(40, 5.0)));
        CHECK(r->graph.rules.sameLevelDz < 5.0);
        CHECK(bandKinks(r->graph).empty());
        const EdgeSpec& a = r->graph.edges[static_cast<std::size_t>(r->graph.index.at("a"))];
        const Real middle = (r->graph.carriagewayHalfWidth(a) + r->graph.sidewalkOuterHalfWidth(a)) * 0.5;
        CHECK(!contains(r->pavement.sidewalk, kinkOutside(40) * (middle / std::cos(20 * DEG_TO_RAD))));
    }
    // A STRAIGHT CARRY-ON IS UNCHANGED: no kink, a mitre of nothing, and the drawn pavement the same to the bit as the same
    // build drawn with nothing stated about where the ends meet
    {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(kinkScene(0)));
        const Pavement drawn = redrawn(*r, true), unstated = redrawn(*r, false);
        CHECK(bandKinks(r->graph).empty());
        BandKink straight;
        straight.edgeA = r->graph.index.at("a");
        straight.endA = 1;
        straight.edgeB = r->graph.index.at("b");
        straight.endB = 0;
        for (const Ring& piece : mitrePieces(r->graph, straight, 9.0, 9.0)) CHECK(piece.empty());
        CHECK(samePolySets(drawn.surface, unstated.surface));
        CHECK(samePolySets(drawn.sidewalk, unstated.sidewalk));
        CHECK(samePolySets(drawn.shoulder, unstated.shoulder));
        CHECK(samePolySets(drawn.median, unstated.median));
        CHECK(drawn.footprints.size() == unstated.footprints.size());
        for (std::size_t li = 0; li < drawn.footprints.size() && li < unstated.footprints.size(); ++li)
            CHECK(samePolySets(drawn.footprints[li], unstated.footprints[li]));
        // (and the build drew the same)
        CHECK(samePolySets(drawn.sidewalk, r->pavement.sidewalk) && samePolySets(drawn.surface, r->pavement.surface));
    }
}

// THE SHIPPED ROAD FILES' KINKS (ADR-0472): how many there are, how many the mitre limit cuts off, what the mitres add to
// the drawn pavement (the same build drawn again with nothing stated about where ends meet), that no mitre piece crosses
// itself, and that the corners left with the drawn sidewalk in two pieces are the three known ones -- none at a kink.
TEST_CASE(road_model_41_the_shipped_kinks_are_mitred) {
    // the corners left with no walk because the drawn sidewalks by their kerb ends are two pieces (junction id, arms)
    const std::map<std::string, std::set<std::string>> knownTwoPieces = {
        {"traffic_lab", {}},
        // j0 6-0, j14 5-0: S2's box edges put their kerb ends 37-48 m out, and the walk round the block between them leaves
        // the corner window (kCornerWindow); j90 0-1: arm 0's kerb end lies on a scrap of s228's sidewalk inside s227's
        // carriageway (ADR-0471: j90 arm 0 too close to j120 for its angle)
        {"river_town", {"j0 6-0", "j14 5-0", "j90 0-1"}}};
    for (const char* level : {"traffic_lab", "river_town"}) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/" + level + "_lanelab.json";
        const std::unique_ptr<Result> r =
            build(RoadLabGraph::fromJson(nlohmann::json::parse(slurp(path)), path.substr(0, path.find_last_of('/'))));
        const RoadLabGraph& g = r->graph;
        const std::vector<BandKink> kinks = bandKinks(g);
        uint32_t capped = 0, crossed = 0;
        Real sharpest = 0;
        for (const BandKink& k : kinks) {
            sharpest = std::max(sharpest, k.turn * RAD_TO_DEG);
            capped += k.turn > 120 * DEG_TO_RAD ? 1u : 0u;
            const EdgeSpec& A = g.edges[static_cast<std::size_t>(k.edgeA)];
            const EdgeSpec& B = g.edges[static_cast<std::size_t>(k.edgeB)];
            if (k.turn > 120 * DEG_TO_RAD)
                std::printf("        past the mitre limit: %s and %s, turning %.1f degrees\n", A.id.c_str(), B.id.c_str(), k.turn * RAD_TO_DEG);
            for (const Ring& piece : mitrePieces(g, k, g.sidewalkOuterHalfWidth(A), g.sidewalkOuterHalfWidth(B))) {
                Vec2 at;
                if (piece.size() == 4u && (segmentsCross(piece[0], piece[1], piece[2], piece[3], at) || segmentsCross(piece[1], piece[2], piece[3], piece[0], at)))
                    ++crossed;
            }
        }
        const Pavement unstated = redrawn(*r, false);
        const Real sidewalkAdded = setArea(r->pavement.sidewalk) - setArea(unstated.sidewalk);
        const Real asphaltAdded = setArea(r->pavement.surface) - setArea(unstated.surface);
        const Real overlap = setArea(intersectSets(r->pavement.sidewalk, r->pavement.surface));
        const Real overlapBefore = setArea(intersectSets(unstated.sidewalk, unstated.surface));
        std::set<std::string> twoPieces;
        for (const WalkCorner& wc : r->walkCorners)
            if (wc.fault == WalkCornerFault::TwoPieces)
                twoPieces.insert(g.junctions[wc.junction.index].id + " " + std::to_string(wc.armA) + "-" + std::to_string(wc.armB));
        std::printf("      %s: %zu kinks (the sharpest %.1f degrees; %u past the mitre limit), %u mitre pieces crossing themselves; the "
                    "mitres add %.1f m2 of sidewalk (%.2f%%) and %.1f m2 of asphalt; sidewalk over asphalt %.1f m2 (stated nothing: %.1f); "
                    "%zu corners with the sidewalk in two pieces\n",
                    level, kinks.size(), sharpest, capped, crossed, sidewalkAdded, 100 * sidewalkAdded / setArea(unstated.sidewalk),
                    asphaltAdded, overlap, overlapBefore, twoPieces.size());
        CHECK(!kinks.empty() && crossed == 0u);
        CHECK(sidewalkAdded > 0 && asphaltAdded > 0);
        CHECK(overlap <= overlapBefore + 1.0);
        CHECK(twoPieces == knownTwoPieces.at(level));
    }
}

// ---- 42. the frozen road files' fixes (ADR-0473, #306 re-bake bundle S4) ------------------------------------------------

TEST_CASE(road_model_42_the_road_files_fixes_hold) {
    // THE SMOOTH FRAME (polyline_ops.h smoothTangentAtStation): a line turning 2 degrees every 20 m, a point set out 150 m
    // to its side, read 0.6 m before and after each of its points. On each segment's own direction (tangentAtStation, the
    // planner's frame before) the point jumps sideways by about 150 m x 2 degrees (4.0 m measured, inside the bend); on the
    // smooth direction it moves on by about the 1.2 m read along, scaled by the bend (0.9 m measured).
    std::vector<Vec2> line;
    Vec2 at(0, 0);
    for (int i = 0; i < 12; ++i) {
        line.push_back(at);
        at = at + Vec2(std::cos(2.0 * i * DEG_TO_RAD), std::sin(2.0 * i * DEG_TO_RAD)) * 20.0;
    }
    const std::vector<double> st = stations(line);
    auto setOut = [&](double s, bool smooth) {
        return pointAt(line, st, s) + perp(smooth ? smoothTangentAtStation(line, st, s) : tangentAtStation(line, st, s)) * 150.0;
    };
    double stepped = 0, smooth = 0;
    for (std::size_t k = 1; k + 1 < line.size(); ++k) {
        stepped = std::max(stepped, distance(setOut(st[k] - 0.6, false), setOut(st[k] + 0.6, false)));
        smooth = std::max(smooth, distance(setOut(st[k] - 0.6, true), setOut(st[k] + 0.6, true)));
    }
    std::printf("      150 m out, 1.2 m read across a point: %.2f m on the segments' own directions, %.2f m on the smooth one\n", stepped, smooth);
    CHECK(stepped > 3.0);
    CHECK(smooth < 1.2);

    for (const char* level : {"river_town", "agent_lab"}) {
        const std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/" + level + "_lanelab.json";
        const nlohmann::json spec = nlohmann::json::parse(slurp(path));
        // THE INTERCHANGE RAMPS, set out again (lanes_tool reframe): no turn from one stated point to the next sharper than
        // 18 degrees (it was 47-134 on the loops), and round a loop's arc none over 6 (it was 40-134: the steps)
        for (const nlohmann::json& e : spec["edges"]) {
            const std::string id = e["id"].get<std::string>();
            if (id.rfind("x0_", 0) != 0 && id.rfind("x1_", 0) != 0) continue;
            std::vector<Vec2> p;
            for (const nlohmann::json& q : e["path"]["points"]) p.push_back(Vec2(q.at(0).get<double>(), q.at(1).get<double>()));
            double sharpest = 0, sharpestOnArc = 0;
            for (std::size_t i = 1; i + 1 < p.size(); ++i) {
                const Vec2 a = p[i] - p[i - 1], b = p[i + 1] - p[i];
                const double turn = std::fabs(std::atan2(cross(a, b), dot(a, b))) * RAD_TO_DEG;
                sharpest = std::max(sharpest, turn);
                if (i >= 10 && i + 10 < p.size()) sharpestOnArc = std::max(sharpestOnArc, turn);
            }
            const bool loop = id.find("_loop") != std::string::npos;
            std::printf("      %s %s: sharpest turn between points %.1f degrees, %.1f away from its ends\n", level, id.c_str(), sharpest, sharpestOnArc);
            CHECK(sharpest < 18.0);
            if (loop) CHECK(sharpestOnArc < 6.0);
        }
        // THE FREEWAY'S TURN-ROUND LOOPS ARE RAMPS: one lane, no sidewalk, a ramp's barriers, and no building fronts them
        const RoadLabGraph g = RoadLabGraph::fromJson(spec, std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned");
        for (const char* id : {"tn0", "tn1"}) {
            const EdgeSpec* e = g.find(id);
            CHECK(e != nullptr);
            if (!e) continue;
            CHECK(e->cls == "ramp");
            CHECK(g.cls(*e).sidewalk == 0.0);
            CHECK(g.cls(*e).defaultBarriers);
            CHECK(!buildingMayFront(classOf(g, *e), false));
        }
    }

    // TRAFFIC_LAB'S j95 STATES s208 AS ITS FOURTH ARM, in its ring, through with s213; stating it again is refused
    const std::string labPath = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/traffic_lab_lanelab.json";
    const std::string labText = slurp(labPath);
    const RoadLabGraph lab = RoadLabGraph::fromJson(nlohmann::json::parse(labText), std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned");
    bool seen = false;
    for (const ModelJunction& j : lab.junctions) {
        if (j.id != "j95") continue;
        seen = true;
        CHECK(j.arms.size() == 4u && j.arms[3].road == "s208" && j.arms[3].spineIn);
        bool through = false;
        for (const std::array<uint32_t, 2>& t : j.through) through = through || (std::min(t[0], t[1]) == 1u && std::max(t[0], t[1]) == 3u);
        CHECK(through);
    }
    CHECK(seen);
    std::string err;
    CHECK(insertArmText(labText, lab, "j95", "s208", nullptr, &err).empty());
    CHECK(err.find("already states") != std::string::npos);
    // a road whose end is not at the junction is refused, and says how far it ends from it
    CHECK(insertArmText(labText, lab, "j95", "s182", nullptr, &err).empty());
    CHECK(err.find("endpoint tolerance") != std::string::npos);
}

// ---- 43. walkways: paths beside no street, stated with where their ends land (ADR-0475) ------------------------------------

namespace {
// A walkway end that lands on a street's kerb (the frontage's street id, station and kerb), on another walkway (its id and
// the metres along it), or a stated dead end.
WalkwayEnd endOnStreet(const RoadModel& m, const RoadFrontage& f) {
    WalkwayEnd e;
    e.meets.street = m.streets[f.street.index].id;
    e.meets.station = f.station;
    e.meets.dir = f.dir;
    return e;
}
WalkwayEnd endOnWalkway(const std::string& id, Real along) {
    WalkwayEnd e;
    e.meets.walkway = id;
    e.meets.along = along;
    return e;
}
WalkwayEnd deadEndHere() {
    WalkwayEnd e;
    e.deadEnd = true;
    return e;
}
constexpr Real kTestWalkwayWidth = 2.0;   // m
ModelWalkway walkwayOf(const std::string& id, const std::string& kind, const std::vector<Vec2>& line, const WalkwayEnd& a,
                       const WalkwayEnd& b) {
    ModelWalkway w;
    w.id = id;
    w.kind = kind;
    w.width = kTestWalkwayWidth;
    w.line = line;
    w.heights.assign(line.size(), 0.0);
    w.ends = {a, b};
    return w;
}
// The lane level's walks of model walkway `way`, in order along it.
std::vector<uint32_t> walksOfWalkway(const LaneGraph& G, uint32_t way) {
    std::vector<uint32_t> out;
    for (uint32_t w = 0; w < G.walks.size(); ++w)
        if (G.walks[w].modelWalkway == way) out.push_back(w);
    std::sort(out.begin(), out.end(), [&](uint32_t a, uint32_t b) { return G.walks[a].stationFrom < G.walks[b].stationFrom; });
    return out;
}
// The lane level's walks along one street's kerb that were one sidewalk walk (its model walk), in order along the street.
std::vector<uint32_t> sidewalkPieces(const LaneGraph& G, const Walk& whole) {
    std::vector<uint32_t> out;
    for (uint32_t w = 0; w < G.walks.size(); ++w)
        if (G.walks[w].modelWalk == whole.modelWalk && G.walks[w].street == whole.street) out.push_back(w);
    std::sort(out.begin(), out.end(), [&](uint32_t a, uint32_t b) { return G.walks[a].stationFrom < G.walks[b].stationFrom; });
    return out;
}
void printWalkwayProblems(const std::vector<WalkwayProblem>& problems) {
    for (const WalkwayProblem& p : problems) std::printf("      walkway %u end %d: %s\n", p.walkway, p.end, p.why.c_str());
}
constexpr Real kOnTheCut = 0.05;   // m: a cut walk's end stands on the stated station's sidewalk point (the slice's rounding)
}  // namespace

TEST_CASE(road_model_43_walkways_are_stated_and_laid_as_walks_joined_where_their_ends_land) {
    // THE T (walkTee) and two walkways north of it, where no road is. The BAR runs from the east road's north sidewalk at
    // station 50 up to y = 30, west to above the west road, and down to its north sidewalk at station 50 (where each
    // end lands: the street, station and kerb, as the lot pass will state it). The STEM leaves the bar where it passes
    // x = 0 and runs north 30 m to a stated dead end.
    BuiltScene s = builtScene(walkTee());
    CHECK(s.report.error.empty());
    RoadModel& m = s.m;
    const LaneGraph& G = m.laneLevel;
    const RoadFrontage eastN = m.frontageAt(Vec2(50, 20)), westN = m.frontageAt(Vec2(-50, 20));
    CHECK(eastN.valid() && westN.valid());
    if (!eastN.valid() || !westN.valid()) return;
    CHECK(m.streets[eastN.street.index].road == "east" && m.streets[westN.street.index].road == "west");
    Vec2 eastAt, westAt;
    CHECK(m.sidewalkBeside(eastN, eastAt) && m.sidewalkBeside(westN, westAt));
    const uint32_t eastSide = sidewalkOf(m, "east", eastN.dir), westSide = sidewalkOf(m, "west", westN.dir);
    CHECK(eastSide != kNoIndex && westSide != kNoIndex);
    if (eastSide == kNoIndex || westSide == kNoIndex) return;
    const std::size_t walksBefore = G.walks.size();
    const Walk eastWhole = G.walks[eastSide], westWhole = G.walks[westSide];

    const std::vector<Vec2> barLine = {eastAt, Vec2(eastAt.x, 30), Vec2(westAt.x, 30), westAt};
    const Real stemAt = stations(barLine)[1] + eastAt.x;   // the bar's metres along where it passes x = 0
    const Real barLength = stations(barLine).back();
    const std::vector<WalkwayProblem> problems =
        stateWalkways(m, {walkwayOf("bar", "footway", barLine, endOnStreet(m, eastN), endOnStreet(m, westN)),
                          walkwayOf("stem", "park_path", {Vec2(0, 30), Vec2(0, 60)}, endOnWalkway("bar", stemAt), deadEndHere())});
    printWalkwayProblems(problems);
    CHECK(problems.empty());
    for (const std::string& p : validate(G)) std::printf("      [lane level] %s\n", p.c_str());
    CHECK(validate(G).empty());
    // two walkways, the bar cut where the stem lands, each street's sidewalk cut where the bar lands
    CHECK(G.walks.size() == walksBefore + 5);
    const std::vector<uint32_t> bar = walksOfWalkway(G, 0), stem = walksOfWalkway(G, 1);
    const std::vector<uint32_t> eastPieces = sidewalkPieces(G, eastWhole), westPieces = sidewalkPieces(G, westWhole);
    CHECK(bar.size() == 2u && stem.size() == 1u && eastPieces.size() == 2u && westPieces.size() == 2u);
    if (bar.size() != 2u || stem.size() != 1u || eastPieces.size() != 2u || westPieces.size() != 2u) return;
    CHECK(m.walkKind(WalkId(bar[0])) == "footway" && m.walkKind(WalkId(stem[0])) == "park_path");
    CHECK(approxEqual(G.walks[bar[0]].stationTo, stemAt, kWrittenStation) && approxEqual(G.walks[bar[1]].stationFrom, stemAt, kWrittenStation));
    CHECK(approxEqual(G.walks[bar[0]].length + G.walks[bar[1]].length, barLength, kOnTheCut));

    // KERB JOINS: each street's sidewalk is cut at station 50, its two pieces meet each other and the bar's end there,
    // at the sidewalk point of that station; what met the whole sidewalk's ends meets the pieces' (none lost)
    const auto kerbJoin = [&](const std::vector<uint32_t>& pieces, const Walk& whole, const WalkEnd& barEnd, const Vec2& at) {
        const Walk& a = G.walks[pieces[0]];
        const Walk& b = G.walks[pieces[1]];
        CHECK(approxEqual(a.stationFrom, whole.stationFrom, kWrittenStation) && approxEqual(a.stationTo, 50.0, kWrittenStation));
        CHECK(approxEqual(b.stationFrom, 50.0, kWrittenStation) && approxEqual(b.stationTo, whole.stationTo, kWrittenStation));
        CHECK(distance(G.pointAt(WalkId(pieces[0]), a.length), at) < kOnTheCut && distance(G.pointAt(WalkId(pieces[1]), 0), at) < kOnTheCut);
        const WalkEnd aEnd{WalkId(pieces[0]), 1}, bStart{WalkId(pieces[1]), 0};
        CHECK(walkEndsMeet(G, aEnd, bStart) && walkEndsMeet(G, aEnd, barEnd) && walkEndsMeet(G, barEnd, bStart));
        CHECK(a.atStart.count == whole.atStart.count && b.atEnd.count == whole.atEnd.count && a.atEnd.count == 2u && b.atStart.count == 2u);
    };
    kerbJoin(eastPieces, eastWhole, WalkEnd{WalkId(bar[0]), 0}, eastAt);
    kerbJoin(westPieces, westWhole, WalkEnd{WalkId(bar[1]), 1}, westAt);
    CHECK(distance(G.pointAt(WalkId(bar[0]), 0), eastAt) < kOnTheCut);

    // A T OF TWO WALKWAYS: the stem's start meets both halves of the bar where they meet; its far end, a stated dead end,
    // meets nothing (and is no problem: none was returned)
    const WalkEnd stemStart{WalkId(stem[0]), 0};
    CHECK(walkEndsMeet(G, stemStart, WalkEnd{WalkId(bar[0]), 1}) && walkEndsMeet(G, stemStart, WalkEnd{WalkId(bar[1]), 0}) &&
          walkEndsMeet(G, WalkEnd{WalkId(bar[0]), 1}, WalkEnd{WalkId(bar[1]), 0}));
    CHECK(G.walks[stem[0]].atEnd.count == 0u);

    // CARS MAY NOT USE THEM; walkers may (the path users' table, by the walkways' kinds)
    const std::vector<uint8_t> cars = m.openTo("vehicles").walks, walker = m.openTo("walker").walks;
    for (const uint32_t w : {bar[0], bar[1], stem[0]}) CHECK(cars[w] == 0 && walker[w] == 1);
    CHECK(openCount(m.openTo("vehicles").paths) == G.paths.size());

    // THE ROUTER WALKS THEM AS IT WALKS ANY WALK (no case of their own). A pedestrian who may use sidewalks and the two
    // walkway kinds but no corner or crossing gets from the east road's north sidewalk to the west road's only over the
    // bar; with footways shut too, not at all. And from the stem's dead end to the east sidewalk, down the stem and the bar.
    {
        StatedFrontage from, to;
        from.street = m.streets[eastN.street.index].id;
        from.station = 80;
        from.dir = eastN.dir;
        to.street = m.streets[westN.street.index].id;
        to.station = 20;
        to.dir = westN.dir;
        const WalkJoin a = m.walkJoinOn(from), b = m.walkJoinOn(to);
        CHECK(a.valid() && b.valid() && a.at.walk.index == eastPieces[1] && b.at.walk.index == westPieces[0]);
        const route::RouteWalker pedestrian = route::bindWalker(m, "pedestrian", kTestPace, usersWithPedestrian({"sidewalk", "footway", "park_path"}));
        const route::WalkRoute r = route::findWalkRoute(m, pedestrian, a.at, b.at);
        std::printf("      east sidewalk to west sidewalk, sidewalks and walkways only:\n");
        printWalkRoute(m, r);
        CHECK(r.found && stepsOfKind(m, r, "footway") == 2u && stepsOfKind(m, r, "corner") == 0u && r.crossings.empty());
        CHECK(approxEqual(r.metres, 30.0 + barLength + 30.0, kOnTheCut));
        const route::RouteWalker noFootway = route::bindWalker(m, "pedestrian", kTestPace, usersWithPedestrian({"sidewalk", "park_path"}));
        CHECK(!route::findWalkRoute(m, noFootway, a.at, b.at).found);
        WalkPoint tip;
        tip.walk = WalkId(stem[0]);
        tip.along = G.walks[stem[0]].length;
        const route::WalkRoute up = route::findWalkRoute(m, pedestrian, tip, a.at);
        printWalkRoute(m, up);
        CHECK(up.found && stepsOfKind(m, up, "park_path") == 1u && stepsOfKind(m, up, "footway") == 1u);
    }

    // A DOOR STATED ON A WALKWAY (StatedFrontage's walkway slot): walkJoinOn answers that walkway's walk holding the metres
    // along it, on either side of the cut; held past its end; a walkway nobody stated: none, and why. A door stated on a
    // street answers as the street's question does.
    {
        StatedFrontage door;
        door.walkway = "bar";
        door.along = 10;
        WalkJoin j = m.walkJoinOn(door);
        CHECK(j.valid() && j.at.walk.index == bar[0] && approxEqual(j.at.along, 10.0, kWrittenStation) && j.walkway == 0u &&
              !j.street.valid() && j.held == 0);
        CHECK(distance(j.point, G.pointAt(WalkId(bar[0]), 10.0)) < kSamePoint);
        door.along = stemAt + 15;
        j = m.walkJoinOn(door);
        CHECK(j.valid() && j.at.walk.index == bar[1] && approxEqual(j.at.along, 15.0, kWrittenStation));
        door.along = barLength + 3;
        j = m.walkJoinOn(door);
        CHECK(j.valid() && j.at.walk.index == bar[1] && approxEqual(j.at.along, G.walks[bar[1]].length, kWrittenStation) &&
              approxEqual(j.held, 3.0, kWrittenStation));
        std::vector<uint8_t> shut = walker;
        shut[bar[0]] = shut[bar[1]] = 0;
        door.along = 10;
        CHECK(!m.walkJoinOn(door, shut).valid() && m.walkJoinOn(door, shut).why == WalkJoinWhy::NotOpen);
        door.walkway = "moat";
        CHECK(!m.walkJoinOn(door).valid() && m.walkJoinOn(door).why == WalkJoinWhy::NoWalkway);
        CHECK(m.walkJoinOn(StatedFrontage()).why == WalkJoinWhy::NoStreet);
        StatedFrontage onStreet;
        onStreet.street = m.streets[eastN.street.index].id;
        onStreet.station = 70;
        onStreet.dir = eastN.dir;
        CHECK(sameJoin(m.walkJoinOn(onStreet), m.walkJoinOn(m.frontageOn(eastN.street, 70, eastN.dir))));
    }

    // SAVED AND READ BACK whole: the walkways and their walks
    {
        bundle::BinWriter w;
        putRoadModel(w, m);
        bundle::BinReader rd(w.bytes.data(), w.bytes.size());
        RoadModel back;
        std::string err;
        CHECK(getRoadModel(rd, back, &err) && back == m);
        CHECK(back.walkways.size() == 2u && back.walkways[1].ends[0].meets.walkway == "bar" && back.walkways[1].ends[1].deadEnd &&
              back.walkways[0].ends[0].meets.street == m.streets[eastN.street.index].id);
        CHECK(back.laneLevel.walks.size() == G.walks.size() && back.laneLevel.walks[stem[0]].modelWalkway == 1u);
        RoadModel moved = m;
        moved.walkways[1].ends[0].meets.along += 1;
        CHECK(!(moved == m));
    }
    // STATED ONCE PER LANE LEVEL: stating again is refused and changes nothing
    {
        const RoadModel before = m;
        const std::vector<WalkwayProblem> again = stateWalkways(m, {});
        CHECK(again.size() == 1u && again[0].fault == WalkwayFault::AlreadyLaid && m == before);
    }
}

TEST_CASE(road_model_44_a_walkway_end_that_does_not_land_is_a_fault_with_why) {
    // THE T with the south road's sidewalk on its +1 kerb only (walkTee("along")): one good walkway with two dead ends,
    // and one per fault. A walkway wrong as a whole is not laid; one with a bad end is laid, its end meeting nothing.
    BuiltScene s = builtScene(walkTee("along"));
    CHECK(s.report.error.empty());
    RoadModel& m = s.m;
    const LaneGraph& G = m.laneLevel;
    const std::size_t walksBefore = G.walks.size();
    const ModelStreetId east = streetOfRoad(m, "east"), south = streetOfRoad(m, "south");
    CHECK(east.valid() && south.valid());
    if (!east.valid() || !south.valid()) return;
    const auto line = [](int i) { return std::vector<Vec2>{Vec2(200 + 10 * i, 0), Vec2(200 + 10 * i, 20)}; };
    const auto onStreet = [&](ModelStreetId st, Real station, int dir) {
        WalkwayEnd e;
        e.meets.street = m.streets[st.index].id;
        e.meets.station = station;
        e.meets.dir = dir;
        return e;
    };
    WalkwayEnd two = onStreet(east, 50, +1);
    two.deadEnd = true;
    struct Case {
        ModelWalkway w;
        int end;
        WalkwayFault fault;
    };
    std::vector<Case> cases = {
        {walkwayOf("ok", "footway", line(0), deadEndHere(), deadEndHere()), -1, WalkwayFault::None},
        {walkwayOf("no_street", "footway", line(1), onStreet(east, 50, +1), deadEndHere()), 0, WalkwayFault::NoSuchStreet},
        {walkwayOf("off_street", "footway", line(2), onStreet(east, 500, +1), deadEndHere()), 0, WalkwayFault::OffStreet},
        {walkwayOf("bare_kerb", "footway", line(3), deadEndHere(), onStreet(south, 50, -1)), 1, WalkwayFault::NoSidewalk},
        {walkwayOf("by_the_junction", "footway", line(4), onStreet(east, 1, +1), deadEndHere()), 0, WalkwayFault::PastSidewalk},
        {walkwayOf("no_walkway", "footway", line(5), endOnWalkway("nowhere", 5), deadEndHere()), 0, WalkwayFault::NoSuchWalkway},
        {walkwayOf("off_walkway", "footway", line(6), endOnWalkway("ok", 500), deadEndHere()), 0, WalkwayFault::OffWalkway},
        {walkwayOf("own", "footway", line(7), endOnWalkway("own", 5), deadEndHere()), 0, WalkwayFault::OwnWalkway},
        {walkwayOf("says_nothing", "footway", line(8), WalkwayEnd(), deadEndHere()), 0, WalkwayFault::StatesNothing},
        {walkwayOf("says_two", "footway", line(9), two, deadEndHere()), 0, WalkwayFault::StatesTwo},
        {walkwayOf("moat", "moat", line(10), deadEndHere(), deadEndHere()), -1, WalkwayFault::KindNotListed},
        {walkwayOf("", "footway", line(11), deadEndHere(), deadEndHere()), -1, WalkwayFault::NoId},
        {walkwayOf("ok", "footway", line(12), deadEndHere(), deadEndHere()), -1, WalkwayFault::SameId},
        {walkwayOf("one_point", "footway", {Vec2(400, 0)}, deadEndHere(), deadEndHere()), -1, WalkwayFault::NoLine},
        {walkwayOf("to_the_moat", "footway", line(14), endOnWalkway("moat", 5), deadEndHere()), 0, WalkwayFault::WalkwayNotLaid},
    };
    cases[1].w.ends[0].meets.street = "s_nowhere";
    std::vector<ModelWalkway> stated;
    for (const Case& c : cases) stated.push_back(c.w);
    const std::vector<WalkwayProblem> problems = stateWalkways(m, stated);
    printWalkwayProblems(problems);
    CHECK(problems.size() == cases.size() - 1);
    uint32_t laid = 0;
    for (uint32_t i = 0; i < cases.size(); ++i) {
        const Case& c = cases[i];
        bool said = c.fault == WalkwayFault::None;
        for (const WalkwayProblem& p : problems)
            if (p.walkway == i) said = p.end == c.end && p.fault == c.fault && !p.why.empty();
        CHECK(said);
        if (!said) std::printf("      case %u (%s): not said as expected\n", i, c.w.id.c_str());
        const std::vector<uint32_t> walks = walksOfWalkway(G, i);
        const bool whole = c.end < 0 && c.fault != WalkwayFault::None;
        CHECK(walks.size() == (whole ? 0u : 1u));
        laid += static_cast<uint32_t>(walks.size());
        // a laid walkway's ends meet nothing here: its good ends are dead ends, its bad one does not land
        for (const uint32_t w : walks) CHECK(G.walks[w].atStart.count == 0u && G.walks[w].atEnd.count == 0u);
    }
    // nothing else moved: no sidewalk was cut, and the lane level holds together
    CHECK(G.walks.size() == walksBefore + laid);
    CHECK(validate(G).empty());
    // A PLANNED-ONLY MODEL has no lane level to lay them in
    RoadLabGraph g;
    RoadModel planned = walkModel(walkTee(), g);
    const std::vector<WalkwayProblem> none = stateWalkways(planned, {cases[0].w});
    CHECK(none.size() == 1u && none[0].fault == WalkwayFault::NoLaneLevel && planned.laneLevel.walks.empty());
}

// ---- 45. a lot's paths are stated as walkways, their ends looked up once (ADR-0476) -----------------------------------

TEST_CASE(road_model_45_a_lots_paths_become_walkways_landing_where_the_lot_pass_states) {
    // THE T of test 43, and a lot north of it whose path runs, as the lot pass draws one, in three straight pieces: up
    // from the east road's north sidewalk at station 50, west along y = 30, and down to the west road's north sidewalk at
    // station 50; and a door on the middle piece, far from any street. The lot pass states where every end lands
    // (city_lots.h stateLotPaths), the model lays them (lots_producer.h stateLotWalkways), and a walker crosses over them.
    BuiltScene s = builtScene(walkTee());
    CHECK(s.report.error.empty());
    RoadModel& m = s.m;
    const RoadFrontage eastN = m.frontageAt(Vec2(50, 20)), westN = m.frontageAt(Vec2(-50, 20));
    Vec2 eastAt, westAt;
    CHECK(eastN.valid() && westN.valid() && m.sidewalkBeside(eastN, eastAt) && m.sidewalkBeside(westN, westAt));
    if (!eastN.valid() || !westN.valid()) return;
    const LotRoadInputs roads = lotRoadInputs(m, RoadDeckField());
    std::vector<LotBuilding> lots(1);
    LotBuilding& lot = lots[0];
    lot.type = "park";
    const Vec2 ne(eastAt.x, 30), nw(westAt.x, 30);
    for (const std::pair<Vec2, Vec2>& piece : {std::make_pair(eastAt, ne), std::make_pair(ne, nw), std::make_pair(nw, westAt)}) {
        ModelWalkway w;   // (as the lot pass draws a path: a kind, a width, two points; city_lots.cpp addPath)
        w.kind = "footway";
        w.width = 2.0;
        w.line = {piece.first, piece.second};
        lot.walkways.push_back(w);
    }
    BuildingUnit hut;
    DoorSpec door;
    door.foot = Vec2(0, 31.5);
    door.normal = Vec2(0, -1);
    hut.doors.push_back(door);
    lot.units.push_back(hut);

    const LotPathCounts counts = stateLotPaths(lots, &roads, nullptr);
    CHECK(counts.walkways == 3 && counts.onStreet == 2 && counts.onWalkway == 4 && counts.deadEnds == 0 && counts.doorsOnWalkway == 1);
    CHECK(lot.walkways.size() == 3u);
    if (lot.walkways.size() != 3u) return;
    const ModelWalkway &up = lot.walkways[0], &across = lot.walkways[1], &down = lot.walkways[2];
    CHECK(up.id == "lot0.p0" && across.id == "lot0.p1" && down.id == "lot0.p2" && up.heights.size() == 2u);
    // the two street ends: each street's id, the station level with the end, its own kerb (frontageOnLine's answer)
    const WalkwayEnd &east = up.ends[0], &west = down.ends[1];
    CHECK(east.meets.street == m.streets[eastN.street.index].id && approxEqual(east.meets.station, eastN.station, kWrittenStation) &&
          east.meets.dir == eastN.dir && !east.deadEnd);
    CHECK(west.meets.street == m.streets[westN.street.index].id && approxEqual(west.meets.station, westN.station, kWrittenStation) &&
          west.meets.dir == westN.dir && !west.deadEnd);
    // the corners: each piece's end on the next piece's start (or its end), by id and metres along
    CHECK(up.ends[1].meets.walkway == "lot0.p1" && approxEqual(up.ends[1].meets.along, 0.0, kWrittenStation));
    CHECK(down.ends[0].meets.walkway == "lot0.p1" && approxEqual(down.ends[0].meets.along, distance(ne, nw), kWrittenStation));
    CHECK(across.ends[0].meets.onWalkway() && across.ends[1].meets.onWalkway());
    // the door states the path and where along it, not a street
    const StatedFrontage& onPath = lot.units[0].doors[0].frontage;
    CHECK(onPath.walkway == "lot0.p1" && !onPath.onStreet() && approxEqual(onPath.along, ne.x, kWrittenStation));

    // HANDED TO THE MODEL: every end lands, no fault
    const std::vector<WalkwayProblem> problems = stateLotWalkways(m, lots);
    printWalkwayProblems(problems);
    CHECK(problems.empty() && m.walkways.size() == 3u);
    CHECK(validate(m.laneLevel).empty());
    // a walker who may use sidewalks and footways but no corner gets from one street to the other over the lot's path
    StatedFrontage from, to;
    from.street = east.meets.street;
    from.station = 80;
    from.dir = eastN.dir;
    to.street = west.meets.street;
    to.station = 20;
    to.dir = westN.dir;
    const WalkJoin a = m.walkJoinOn(from), b = m.walkJoinOn(to);
    CHECK(a.valid() && b.valid());
    const route::RouteWalker pedestrian = route::bindWalker(m, "pedestrian", kTestPace, usersWithPedestrian({"sidewalk", "footway"}));
    const route::WalkRoute r = route::findWalkRoute(m, pedestrian, a.at, b.at);
    printWalkRoute(m, r);
    CHECK(r.found && stepsOfKind(m, r, "footway") == 3u && stepsOfKind(m, r, "corner") == 0u);
    // and the door is answered on the path by the one query
    const WalkJoin j = m.walkJoinOn(onPath);
    CHECK(j.valid() && j.walkway == 1u && !j.street.valid() && j.held == 0);
}
