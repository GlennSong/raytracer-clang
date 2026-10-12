#include "../src/engine/procgen/world/island_rail.h"
#include "test_framework.h"

#include <cmath>

using namespace engine;
using json = nlohmann::json;

// THE ISLAND LOOP from a lanes graph (island_rail.h): a square freeway ring of two carriageway pieces, a road across
// it on one side, a river floor on another, two towns outside it.

namespace {

json squareIsland() {
    // the outer carriageway, counter-clockwise, 4 km a side, cut in two the way an island scene cuts it
    auto pts = [](std::initializer_list<std::pair<double, double>> l) { json a = json::array(); for (auto& p : l) a.push_back({p.first, p.second}); return a; };
    json g;
    g["edges"] = json::array();
    g["edges"].push_back({{"id", "fw0_a"}, {"class", "freeway"}, {"path", {{"points", pts({{-2000, -2000}, {2000, -2000}, {2000, 2000}})}}},
                          {"floor", json::array({json::array({2000.0, 0.0, 12.0, 80.0}), json::array({2000.0, 1000.0, 9.0, 30.0})})}});
    g["edges"].push_back({{"id", "fw0_a2"}, {"class", "freeway"}, {"path", {{"points", pts({{2000, 2000}, {-2000, 2000}, {-2000, -2000}})}}}});
    // the inner carriageway: a smaller ring the line must NOT follow
    g["edges"].push_back({{"id", "fw0_b"}, {"class", "freeway"}, {"path", {{"points", pts({{-1976, -1976}, {1976, -1976}, {1976, 1976}, {-1976, 1976}, {-1976, -1976}})}}}});
    // a road out of the island across the south side, at x = 500
    g["edges"].push_back({{"id", "link"}, {"class", "collector"}, {"path", {{"points", pts({{500, -1800}, {500, -2300}})}}}});
    return g;
}

}  // namespace

TEST_CASE(island_rail_loop_runs_outside_the_outer_carriageway_and_closes) {
    const IslandRailResult r = islandRailLoop(squareIsland(), {}, IslandRailOptions{});
    CHECK(r.edges.size() >= 3);
    // 4 km sides offset 45 m outward: about 16.18 km (the corners round off a little)
    CHECK(r.length > 16000 && r.length < 16500);
    // every point outside the outer ring, ~45 m from it
    for (const json& e : r.edges)
        for (const json& p : e["path"]["points"]) {
            const double x = std::fabs(p[0].get<double>()), z = std::fabs(p[1].get<double>());
            if (std::min(x, z) > 1900.0) continue;   // (a square's right-angle corners round off inside; a freeway has none)
            CHECK(std::max(x, z) > 2030.0 && std::max(x, z) < 2060.0);
        }
    // the edges join end to start, round to the first
    for (std::size_t i = 0; i < r.edges.size(); ++i) {
        const json& a = r.edges[i]["path"]["points"].back();
        const json& b = r.edges[(i + 1) % r.edges.size()]["path"]["points"].front();
        CHECK(std::hypot(a[0].get<double>() - b[0].get<double>(), a[1].get<double>() - b[1].get<double>()) < 1e-6);
        CHECK(r.edges[i]["class"] == "rail");
    }
    CHECK(r.crossings == 1);   // the link road
    CHECK(r.floors == 1);      // the river (80 m hold); the road under the freeway (30 m) is not the line's
}

TEST_CASE(island_rail_stations_stand_near_their_towns_clear_of_crossings_and_bridges) {
    // a town south, right where the link road crosses; one east where the river is; one north on open line
    const std::vector<RailTown> towns = {{"South", Vec2(500, -2600)}, {"East", Vec2(2600, 0)}, {"North", Vec2(0, 2600)}};
    const IslandRailResult r = islandRailLoop(squareIsland(), towns, IslandRailOptions{});
    CHECK(r.stations.size() == 3);
    int platforms = 0;
    for (const IslandRailStation& s : r.stations) {
        if (s.name == "South") CHECK(std::fabs(s.at.x - 500) >= 70 + 150 - 1);   // off the crossing by half a platform + 150
        if (s.name == "East") CHECK(std::fabs(s.at.y) >= 70 + 80 + 40 - 1);       // off the bridge
        if (s.name == "North") CHECK(std::fabs(s.at.x) < 20 && s.fromTown < 600);
        CHECK(std::fabs(std::max(std::fabs(s.at.x), std::fabs(s.at.y)) - 2045) < 10);   // on a straight, not a corner
    }
    for (const json& e : r.edges)
        if (e.contains("platforms"))
            for (const json& p : e["platforms"]) {
                ++platforms;
                CHECK(p["s1"].get<double>() - p["s0"].get<double>() == 140.0);
                CHECK(p["s0"].get<double>() >= 0);
            }
    CHECK(platforms == 3);
}

TEST_CASE(island_rail_every_station_gets_a_way_in_from_the_nearest_street_over_the_line) {
    // a street outside the line along the north side, 80 m beyond it: North's way leaves from it
    json g = squareIsland();
    g["edges"].push_back({{"id", "front"}, {"class", "local"}, {"path", {{"points", json::array({json::array({-600.0, 2125.0}), json::array({600.0, 2125.0})})}}}});
    const IslandRailResult r = islandRailLoop(g, {{"North", Vec2(0, 2600)}}, IslandRailOptions{});
    CHECK(r.stations.size() == 1);
    CHECK(r.ways.size() == 1);
    if (r.ways.empty() || r.stations.empty()) return;
    const json& w = r.ways[0];
    const IslandRailStation& s = r.stations[0];
    CHECK(w["class"] == "footway");
    CHECK(s.street == "front");
    const json& pts = w["path"]["points"];
    // it starts on the street's sidewalk (local: 3.5 m to the kerb, 1 m in), outside the line
    CHECK(std::fabs(pts.front()[1].get<double>() - (2125.0 - 4.5)) < 0.5);
    // the leg runs beside the line on the street's side, and the bridge ends past the far platform
    CHECK(std::fabs(pts[2][1].get<double>() - (s.at.y + 14.0)) < 1.0);
    CHECK(std::fabs(pts.back()[1].get<double>() - (s.at.y - 10.05)) < 0.5);
    CHECK(std::fabs(pts.back()[0].get<double>() - s.at.x) < 0.5);
    // the hold over the line, its height left to the builder
    CHECK(w["floor"].size() == 1);
    CHECK(w["floor"][0][2].get<double>() < -100.0);
    CHECK(w["floor"][0][3].get<double>() >= 10.05);
}
