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

TEST_CASE(a_city_light_rail_loop_follows_its_streets_set_in_and_smooth) {
    // a 2 km city on a 200 m grid of arterials, its hub in the middle, the main line's way in off its east edge
    json g;
    g["edges"] = json::array();
    int k = 0;
    for (double c = -1000; c <= 1000; c += 200)   // each street a block long, junction to junction, as a lanes graph has them
        for (double u = -1000; u < 1000; u += 200) {
            g["edges"].push_back({{"id", "ns" + std::to_string(k++)}, {"class", "arterial"}, {"path", {{"points", json::array({json::array({c, u}), json::array({c, u + 200})})}}}});
            g["edges"].push_back({{"id", "ew" + std::to_string(k++)}, {"class", "arterial"}, {"path", {{"points", json::array({json::array({u, c}), json::array({u + 200, c})})}}}});
        }
    const IslandRailResult r = cityRailLoop(g, RailTown{"Gridton", Vec2(0, 0)}, Vec2(1000, 120), CityRailOptions{});
    for (const std::string& w : r.skipped) std::printf("    [city rail] skipped: %s\n", w.c_str());
    CHECK(r.length > 3000 && r.length < 8000);
    CHECK(r.stations.size() >= 3);
    CHECK(r.crossings > 4);   // over its side streets
    // the edges join end to start round the loop, all light rail
    std::vector<Vec2> pts;
    for (std::size_t i = 0; i < r.edges.size(); ++i) {
        CHECK(r.edges[i]["class"] == "light_rail");
        const json& a = r.edges[i]["path"]["points"].back();
        const json& b = r.edges[(i + 1) % r.edges.size()]["path"]["points"].front();
        CHECK(std::hypot(a[0].get<double>() - b[0].get<double>(), a[1].get<double>() - b[1].get<double>()) < 1e-6);
        for (const json& p : r.edges[i]["path"]["points"]) pts.push_back(Vec2(p[0].get<double>(), p[1].get<double>()));
    }
    // set in beside its streets: running along an arterial, never on it (between crossings, >= 15 m off any centreline)
    int along = 0, on = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Vec2 p = pts[i], t = pts[(i + 1) % pts.size()] - pts[(i + pts.size() - 1) % pts.size()];
        const double gx = std::fabs(std::remainder(p.x, 200.0)), gz = std::fabs(std::remainder(p.y, 200.0));
        if (std::min(gx, gz) < 30.0) ++along;
        // ON a street: within 12 m of a centreline and running with it (crossing one square is what it should do)
        const double tl = std::max(1e-9, t.length());
        if ((gx < 12.0 && std::fabs(t.y) / tl > 0.9) || (gz < 12.0 && std::fabs(t.x) / tl > 0.9)) ++on;
    }
    CHECK(along > static_cast<int>(pts.size()) / 2);
    CHECK(on < static_cast<int>(pts.size()) / 20);
    // no curve a tram cannot take
    double rMin = 1e9;
    for (std::size_t i = 2; i + 2 < pts.size(); ++i) {
        const Vec2 a = pts[i - 2], b = pts[i], c = pts[i + 2];
        const double cr = std::fabs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
        if (cr > 1e-9) rMin = std::min(rMin, (a - b).length() * (b - c).length() * (c - a).length() / (2 * cr));
    }
    std::printf("    [city rail] %.0f m, %zu stations, %d crossings, tightest curve %.0f m\n", r.length, r.stations.size(), r.crossings, rMin);
    CHECK(rMin > 18.0);
    // its sections join clear of the streets it crosses (a joint beside a lift stepped on the island): 200 m blocks
    // leave room for 60 m along the line (the straight line to a crossing round a corner is a little less)
    std::vector<Vec2> crossAt;   // where the line crosses a grid street
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Vec2 a = pts[i], b = pts[(i + 1) % pts.size()];
        for (double c = -1000; c <= 1000; c += 200) {
            if ((a.x - c) * (b.x - c) < 0) crossAt.push_back(a + (b - a) * ((c - a.x) / (b.x - a.x)));
            if ((a.y - c) * (b.y - c) < 0) crossAt.push_back(a + (b - a) * ((c - a.y) / (b.y - a.y)));
        }
    }
    double jointMin = 1e9;
    for (const json& e : r.edges) {
        const Vec2 j(e["path"]["points"].front()[0].get<double>(), e["path"]["points"].front()[1].get<double>());
        for (const Vec2& x : crossAt) jointMin = std::min(jointMin, (x - j).length());
    }
    std::printf("    [city rail] nearest joint to a street it crosses: %.0f m\n", jointMin);
    CHECK(jointMin > 45.0);
    // every station reached from below: a way that ends past the line, and a level hold on the track over it
    CHECK(r.ways.size() == r.stations.size());
    for (const json& w : r.ways) {
        CHECK(!w.contains("floor"));   // the way stays on the street's level
        const json& last = w["path"]["points"].back();
        const Vec2 end(last[0].get<double>(), last[1].get<double>());
        bool held = false;
        for (const json& e : r.edges)
            if (e.contains("floor"))
                for (const json& f : e["floor"]) {
                    const Vec2 at(f[0].get<double>(), f[1].get<double>());
                    if ((at - end).length() < 20.0 && f[3].get<double>() >= 40.0 + 5.0) held = true;   // half the 80 m platform and more
                }
        CHECK(held);
    }
}
