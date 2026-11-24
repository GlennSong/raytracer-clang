// BUS LINES ON THE ROAD MODEL (the buses plan, step 1; ~/.claude/plans/buses-on-road-model.md). A line is stated in the
// road file -- its stops in order, each a street by id, a station along it and a kerb -- and the road model holds it,
// each stop's street resolved. The claims under test:
//   - a stated line is held: each stop's bus has a kerb lane to pull into, and its people a walk on the stop's kerb;
//   - a line with a stop that names no street, or lies off its street, is not held, and a problem names it;
//   - the driving checks that every leg can be driven: a two-stop line on one street of a grid is (the bus goes round a
//     block to come back), the same line on a street that ends with nowhere to turn is not, and the fault names the
//     leg.
// The scenes are road files built as a level's roads are (model_scene.h).
#include "test_framework.h"

#include "model_scene.h"
#include "trip_driving.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/roads/bus_planner.h"
#include "../src/engine/procgen/city/roads/lanes/road_layout.h"

#include <cstdio>
#include <string>

using namespace engine;

namespace {

// A 3 by 3 grid of 200 m local streets (a block either side of the middle street, to go round).
RoadGraph gridOfNine() {
    RoadGraph g;
    int n[3][3];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) n[r][c] = g.addNode({200.0 * c, 200.0 * r});
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c + 1 < 3; ++c) g.addEdge(n[r][c], n[r][c + 1], 10, RoadClass::Local);
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r + 1 < 3; ++r) g.addEdge(n[r][c], n[r + 1][c], 10, RoadClass::Local);
    return g;
}

// The longest street of a model (its id, where it runs along its spine).
const ModelStreet& longestStreet(const RoadModel& m) {
    const ModelStreet* best = &m.streets.front();
    for (const ModelStreet& S : m.streets)
        if (S.to - S.from > best->to - best->from) best = &S;
    return *best;
}

// One line of two stops on `street`'s +1 kerb, at a quarter and three quarters of the way along it.
nlohmann::json twoStopLine(const ModelStreet& street, const std::string& lineId) {
    const Real len = street.to - street.from;
    nlohmann::json stops = nlohmann::json::array();
    stops.push_back({{"id", lineId + "-1"}, {"name", "First"}, {"street", street.id}, {"station", street.from + len * 0.25}, {"kerb", 1}});
    stops.push_back({{"id", lineId + "-2"}, {"name", "Second"}, {"street", street.id}, {"station", street.from + len * 0.75}, {"kerb", 1}});
    return {{"id", lineId}, {"name", "Test " + lineId}, {"stops", stops}};
}

// The scene of `g` again, with these bus lines stated in its road file.
const citytest::ModelScene& withLines(const std::string& name, const RoadGraph& g, const nlohmann::json& lines) {
    return citytest::sceneNamed("buslines_" + name, [&g, &lines, name]() {
        nlohmann::json file = citytest::roadFileOf(g, name);
        file["bus_lines"] = lines;
        return citytest::modelScene(file);
    });
}

}  // namespace

TEST_CASE(bus_line_stated_in_the_road_file_is_held_with_its_stops_on_the_roads) {
    const RoadGraph g = gridOfNine();
    const citytest::ModelScene& plain = citytest::graphScene("bus_grid", g);
    const ModelStreet& S = longestStreet(*plain.model);
    const citytest::ModelScene& s = withLines("grid_one", g, nlohmann::json::array({twoStopLine(S, "L1")}));
    const RoadModel& m = *s.model;
    CHECK(m.busLines.size() == 1u);
    if (m.busLines.size() != 1u) return;
    const ModelBusLine& line = m.busLines.front();
    std::printf("    [bus] line %s \"%s\": %zu stops on street %s (%.1f to %.1f)\n", line.id.c_str(), line.name.c_str(),
                line.stops.size(), S.id.c_str(), S.from, S.to);
    CHECK(line.id == "L1" && line.stops.size() == 2u);
    for (const ModelBusStop& stop : line.stops) {
        CHECK(stop.street.valid() && m.streets[stop.street.index].id == S.id);
        const KerbLanePlace k = m.busStopLane(stop);
        const WalkJoin w = m.busStopWalk(stop);
        CHECK(k.valid());
        CHECK(w.valid() && w.dir == stop.kerb && !w.across);   // waiting on its own kerb
        if (k.valid() && w.valid())
            std::printf("    [bus] stop %s at %.1f: its bus pulls in on path %u at %.1f m; people wait %.1f m from it\n", stop.id.c_str(),
                        stop.station, k.path.index, k.along, (w.point - k.at).length());
        // the bus pulls in beside where people wait: across the kerb from them, not down the street
        if (k.valid() && w.valid()) CHECK((w.point - k.at).length() < 12.0);
    }
}

TEST_CASE(bus_line_with_a_stop_that_cannot_stand_is_not_held_and_is_named) {
    const RoadGraph g = gridOfNine();
    const citytest::ModelScene& plain = citytest::graphScene("bus_grid", g);
    const ModelStreet& S = longestStreet(*plain.model);
    nlohmann::json noStreet = twoStopLine(S, "BAD1");
    noStreet["stops"][1]["street"] = "no_such_street";
    nlohmann::json offStreet = twoStopLine(S, "BAD2");
    offStreet["stops"][0]["station"] = S.to + 50.0;
    const citytest::ModelScene& s = withLines("grid_bad", g, nlohmann::json::array({noStreet, offStreet, twoStopLine(S, "GOOD")}));
    const RoadModel& m = *s.model;
    std::string said;
    for (const std::string& p : m.problems)
        if (p.find("bus line") != std::string::npos) said += "      " + p + "\n";
    std::printf("    [bus] held %zu of 3 lines; the problems:\n%s", m.busLines.size(), said.c_str());
    CHECK(m.busLines.size() == 1u && m.busLines.front().id == "GOOD");
    CHECK(said.find("BAD1") != std::string::npos && said.find("no_such_street") != std::string::npos);
    CHECK(said.find("BAD2") != std::string::npos && said.find("lies off street") != std::string::npos);
}

TEST_CASE(bus_line_legs_are_checked_by_the_driving_round_a_block_and_not_past_a_dead_end) {
    // ON THE GRID: from the second stop back to the first the bus drives round a block
    {
        const RoadGraph g = gridOfNine();
        const citytest::ModelScene& plain = citytest::graphScene("bus_grid", g);
        const ModelStreet& S = longestStreet(*plain.model);
        const citytest::ModelScene& s = withLines("grid_one", g, nlohmann::json::array({twoStopLine(S, "L1")}));
        const std::unique_ptr<driving::Driving> d = citytest::tripDriving(*s.model);
        const std::vector<std::string> problems = d->busLineProblems(0);
        for (const std::string& p : problems) std::printf("    [bus] grid: %s\n", p.c_str());
        CHECK(problems.empty());
    }
    // ON A STREET THAT ENDS AT A T AT ONE END AND NOWHERE AT THE OTHER (model_scene.h straightScene's road): the way
    // back from the second stop to the first has no route (no car turns round anywhere), and the fault names the leg
    {
        RoadGraph g;
        const int a = g.addNode({0, 0}), b = g.addNode({600, 0}), c = g.addNode({600, -30}), e = g.addNode({600, 30});
        g.addEdge(a, b, 10, RoadClass::Local);
        g.addEdge(b, c, 10, RoadClass::Local);
        g.addEdge(b, e, 10, RoadClass::Local);
        const citytest::ModelScene& plain = citytest::graphScene("bus_dead_end", g);
        const ModelStreet& S = longestStreet(*plain.model);
        const citytest::ModelScene& s = withLines("dead_end", g, nlohmann::json::array({twoStopLine(S, "D1")}));
        const std::unique_ptr<driving::Driving> d = citytest::tripDriving(*s.model);
        const std::vector<std::string> problems = d->busLineProblems(0);
        for (const std::string& p : problems) std::printf("    [bus] dead end: %s\n", p.c_str());
        bool named = false;
        for (const std::string& p : problems) named = named || p.find("no route") != std::string::npos;
        CHECK(named);
    }
}

// ---- THE PLANNER (ADR-0482): lays a town's lines over its road model, and they go into the road file and back --------

namespace {
// The router's class for the scenes' one vehicle (model_scene.h sceneVehicles: a car), standing in for the bus.
engine::roads::RouteClass sceneBus(const RoadModel& m) {
    engine::roads::RouteClass c;
    for (std::size_t t = 0; t < m.laneLevel.vehicleTagNames.size(); ++t)
        if (m.laneLevel.vehicleTagNames[t] == "car") c.tags |= TagMask(1) << t;
    c.tightestTurnRadius = 5.0;
    c.topSpeed = 13.9;
    return c;
}
}  // namespace

TEST_CASE(bus_planner_lays_loops_with_stops_along_them_that_can_be_driven) {
    const citytest::ModelScene& town = citytest::gridScene(1200.0, 200.0, 7u);
    const RoadModel& m = *town.model;
    engine::roads::BusPlanParams bp;
    bp.hubEvery = 2000;
    const engine::roads::BusPlan plan = engine::roads::planBusLines(m, sceneBus(m), bp);
    std::printf("    [planner] %d hubs, %zu lines; coverage %.1f%%, street share %.1f%%\n", plan.hubs, plan.lines.size(), plan.coverage * 100.0,
                plan.streetShare * 100.0);
    for (const std::string& n : plan.notes) std::printf("    [planner] note: %s\n", n.c_str());
    CHECK(plan.hubs >= 3 && !plan.lines.empty());
    CHECK(plan.coverage > 0.6 && plan.streetShare > 0.2);
    for (const ModelBusLine& L : plan.lines) {
        CHECK(L.stops.size() >= 3u);
        for (std::size_t i = 0; i < L.stops.size(); ++i) {
            const ModelBusStop& s = L.stops[i];
            CHECK(m.streets[s.street.index].section.sidewalkOnKerb(s.kerb));   // beside a sidewalk
            const KerbLanePlace a = m.busStopLane(s), b = m.busStopLane(L.stops[(i + 1) % L.stops.size()]);
            CHECK(a.valid() && b.valid());
            if (a.valid() && b.valid()) CHECK((a.at - b.at).length() > 60.0);   // not crowded together
        }
    }
    // INTO THE ROAD FILE AND BACK: the file with the lines written in, built again, holds them; the driving finds no
    // fault; the city's bake key does not see them
    nlohmann::json file = citytest::roadFileOf(citytest::gridCity([] {
                                                   citytest::GridCityParams p;
                                                   p.extent = 1200.0;
                                                   p.cellSize = 200.0;
                                                   p.seed = 7u;
                                                   return p;
                                               }()),
                                               "grid");
    const std::string text = file.dump(2);
    std::string err;
    const std::string written = roads::lanes::insertBusLinesText(text, engine::roads::busLinesJson(plan.lines), &err);
    CHECK(!written.empty());
    CHECK(roads::lanes::keyedRoadFileText(written) == roads::lanes::keyedRoadFileText(text));
    const std::string again = roads::lanes::insertBusLinesText(written, engine::roads::busLinesJson(plan.lines), &err);
    CHECK(again == written);   // written again: the same bytes
    const citytest::ModelScene& back = citytest::sceneNamed("buslines_planned_grid", [&written]() {
        return citytest::modelScene(nlohmann::json::parse(written));
    });
    CHECK(back.model->busLines.size() == plan.lines.size());
    const std::unique_ptr<driving::Driving> d = citytest::tripDriving(*back.model);
    const std::vector<std::string> problems = d->busLineProblems(0);
    for (const std::string& p : problems) std::printf("    [planner] fault: %s\n", p.c_str());
    CHECK(problems.empty());
}
