#include "test_framework.h"

#include "../src/engine/procgen/city/city_svg.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/ai/nav_graph.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;

// THE CITY MAP (device: "an svg of all the sidewalks and how they hug the
// roads ... along with all of the planted street objects ... layered so we
// can turn on and off elements"). A small synthetic city: a square street
// loop, one curb loop around its asphalt, a block with a lot and a building,
// a hub, a place, two planted objects.

namespace {
CityMapData smallCity() {
    CityMapData m;
    RoadGraph& g = m.roads;
    g.nodes = {RoadNode{Vec2(0, 0)}, RoadNode{Vec2(200, 0)}, RoadNode{Vec2(200, 200)},
               RoadNode{Vec2(0, 200)}};
    g.addEdge(0, 1, 10.0);
    g.addEdge(1, 2, 10.0);
    g.addEdge(2, 3, 10.0);
    g.addEdge(3, 0, 10.0);
    m.nav = buildNavGraph(g);
    m.furniture = planStreetFurniture(m.nav, [](Real, Real) { return Real(0); });
    // The asphalt's outer outline (CCW in x/z), a curb loop 5 m outside the
    // centrelines, and the inner block hole.
    m.curbLoops.push_back({Vec2(-5, -5), Vec2(205, -5), Vec2(205, 205), Vec2(-5, 205)});
    m.curbLoops.push_back({Vec2(5, 5), Vec2(5, 195), Vec2(195, 195), Vec2(195, 5)});
    m.sidewalkWidth = 3.5;
    m.mouthGaps.push_back({Vec2(100, -5), Vec2(110, -5)});
    m.blocks.push_back({Vec2(9, 9), Vec2(191, 9), Vec2(191, 191), Vec2(9, 191)});
    m.lots.push_back({Vec2(9, 9), Vec2(100, 9), Vec2(100, 100), Vec2(9, 100)});
    m.buildings.push_back({{Vec2(15, 15), Vec2(90, 15), Vec2(90, 90), Vec2(15, 90)}, "financial", "office"});
    m.hubs.push_back({Vec2(100, 100), 0, "financial"});
    m.hubRadius = 150.0;
    m.places.push_back({Vec2(50, 50), "office", "Acme"});
    m.objects.push_back({Vec2(30, 30), CityMapData::ObjectKind::Scenery});
    m.objects.push_back({Vec2(31, 30), CityMapData::ObjectKind::Furniture});
    return m;
}
std::string readAll(const std::string& path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}
}

TEST_CASE(city_map_draws_every_layer_as_its_own_group) {
    const CityMapData m = smallCity();
    const std::string path = "city_map_test.svg";
    CHECK(writeCityMapSvg(path, m, CityMapLayers()));
    const std::string svg = readAll(path);
    std::remove(path.c_str());
    int n = 0;
    const char* const* names = CityMapLayers::names(&n);
    CHECK(n == 13);
    for (int i = 0; i < n; ++i)
        CHECK(svg.find("id='layer-" + std::string(names[i]) + "'") != std::string::npos);
    CHECK(svg.find("toggleLayer(") != std::string::npos);            // the in-file switches
    CHECK(svg.find("financial") != std::string::npos);               // the district label
    CHECK(svg.find("office Acme") != std::string::npos);             // the place
    CHECK(svg.find("stroke-width='10.0'") != std::string::npos);     // a street at its width
    CHECK(svg.find("stroke-width='3.5'") != std::string::npos);      // the sidewalk band
    CHECK(m.furniture.signals.size() > 0 || m.furniture.lampBases.size() > 0);
}

TEST_CASE(city_map_layer_selection_leaves_the_others_out) {
    const CityMapData m = smallCity();
    const std::string path = "city_map_test2.svg";
    const CityMapLayers sel = CityMapLayers::fromList("roads, sidewalks,furniture");
    CHECK(sel.roads && sel.sidewalks && sel.furniture);
    CHECK(!sel.buildings && !sel.districts && !sel.nav && !sel.legend);
    CHECK(sel.toList() == "roads,sidewalks,furniture");
    CHECK(writeCityMapSvg(path, m, sel));
    const std::string svg = readAll(path);
    std::remove(path.c_str());
    CHECK(svg.find("id='layer-roads'") != std::string::npos);
    CHECK(svg.find("id='layer-sidewalks'") != std::string::npos);
    CHECK(svg.find("id='layer-buildings'") == std::string::npos);
    CHECK(svg.find("id='layer-districts'") == std::string::npos);
    CHECK(svg.find("id='layer-legend'") == std::string::npos);
    // "all" and the empty list mean everything; the furniture preset is
    // the old RT_FURNITURE_SVG picture.
    CHECK(CityMapLayers::fromList("all").buildings);
    CHECK(CityMapLayers::fromList("").districts);
    const CityMapLayers f = furnitureMapLayers();
    CHECK(f.roads && f.nav && f.furniture && f.legend && !f.sidewalks && !f.buildings);
}

TEST_CASE(city_map_sidewalk_band_hugs_the_curb_from_outside) {
    // The band centreline sits half a sidewalk OUTSIDE the asphalt outline
    // all the way round — for the exterior loop that is outside the square,
    // for the block's hole loop it is inside the hole (away from the road).
    const CityMapData m = smallCity();
    const std::vector<Poly2> bands = sidewalkBandCentrelines(m.curbLoops, m.sidewalkWidth);
    CHECK(bands.size() == 2);
    const double half = m.sidewalkWidth * 0.5;
    // Exterior loop (CCW): every point outside [-5, 205]^2 by ~half (corners by half*sqrt2).
    for (const Vec2& p : bands[0]) {
        const bool outside = p.x < -5.0 || p.x > 205.0 || p.y < -5.0 || p.y > 205.0;
        CHECK(outside);
        const double dx = std::max({-5.0 - p.x, p.x - 205.0, 0.0});
        const double dy = std::max({-5.0 - p.y, p.y - 205.0, 0.0});
        CHECK_APPROX(std::max(dx, dy), half, 1e-6);
    }
    // Hole loop (CW as the mesher orients holes): every point inside (5, 195)^2, half in.
    for (const Vec2& p : bands[1]) {
        CHECK(p.x > 5.0 && p.x < 195.0 && p.y > 5.0 && p.y < 195.0);
        const double d = std::min({p.x - 5.0, 195.0 - p.x, p.y - 5.0, 195.0 - p.y});
        CHECK_APPROX(d, half, 1e-6);
    }
}
