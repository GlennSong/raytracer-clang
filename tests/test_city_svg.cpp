#include "test_framework.h"

#include "../src/engine/procgen/city/city_svg.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/road_net.h"   // buildRoadNetMesh: a real deck for the census
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
    CHECK(n == 14);
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

// WHERE THE SIDEWALK CUTS ACROSS A ROAD, measured against the BUILT deck: the
// mesher's own band around its own asphalt has no cuts (its curb loops are
// the asphalt's outline); a stray band loop through the middle of a street
// is found, on that street, ~4 m deep; a loop wrapping a dead end's cap is
// not a cut (the deck ends where the ribbon ends). Straight roads on
// purpose: the mesher SMOOTHS a closed 4-node loop into a rounded curve
// (the first fixture's "south street" ran 13 m off its graph edge).
TEST_CASE(city_map_finds_the_sidewalk_on_the_built_asphalt) {
    RoadEntity net;
    net.look.defaultWidth = 10.0;
    net.look.sidewalk = 3.5;
    net.graph.nodes = {RoadNode{Vec2(0, 0)}, RoadNode{Vec2(200, 0)}, RoadNode{Vec2(400, 0)},
                       RoadNode{Vec2(200, 200)}, RoadNode{Vec2(300, 0)}};
    net.graph.addEdge(0, 1, 10.0);   // a straight east-west street ...
    net.graph.addEdge(1, 4, 10.0);
    net.graph.addEdge(4, 2, 10.0);   // ... ending in a dead end at x = 400
    net.graph.addEdge(1, 3, 10.0);   // a T arm north
    CurbBandAudit audit;
    RoadDeckField deck;
    buildRoadNetMesh(net, nullptr, &audit, &deck);
    CHECK(!audit.loops.empty());
    CHECK(!deck.empty());
    CityMapData m;
    m.roads = net.graph;
    m.curbLoops = audit.loops;
    m.sidewalkWidth = audit.sidewalkWidth > 0 ? audit.sidewalkWidth : 3.5;
    const std::vector<const RoadDeckField*> decks{&deck};
    std::vector<SidewalkCrossing> clean = findSidewalkRoadCrossings(m, decks);
    std::printf("    [conflicts] the mesher's own band: %zu place(s) on its own asphalt\n", clean.size());
    CHECK(clean.empty());
    // The deck itself: the street's centre 5 m deep, its verge outside, and
    // past the dead end's cap nothing.
    CHECK_APPROX(deck.depthInside(100.0, 0.0), 5.0, 0.6);
    CHECK_APPROX(deck.depthInside(100.0, -7.0), 0.0, 1e-9);
    CHECK_APPROX(deck.depthInside(405.0, 0.0), 0.0, 1e-9);
    // A stray band loop straight through the south street.
    m.curbLoops.push_back({Vec2(90, -30), Vec2(110, -30), Vec2(110, 30), Vec2(90, 30)});
    std::vector<SidewalkCrossing> hits = findSidewalkRoadCrossings(m, decks);
    std::printf("    [conflicts] with a stray loop: %zu place(s); first at (%.1f, %.1f) %.2f m onto the deck near a %.0f m road\n",
                hits.size(), hits.empty() ? 0.0 : hits[0].pos.x, hits.empty() ? 0.0 : hits[0].pos.y,
                hits.empty() ? 0.0 : hits[0].depth, hits.empty() ? 0.0 : hits[0].width);
    CHECK(hits.size() >= 1);
    if (!hits.empty()) {
        CHECK(std::fabs(hits[0].pos.y) < 6.0);
        CHECK(hits[0].pos.x > 85.0 && hits[0].pos.x < 115.0);
        CHECK(hits[0].depth > 3.0);
        CHECK_APPROX(hits[0].width, 10.0, 1e-9);
    }
    // A loop hugging the street's END (a cul-de-sac cap): not a cut.
    m.curbLoops.pop_back();
    m.curbLoops.push_back({Vec2(404, -8), Vec2(412, -8), Vec2(412, 8), Vec2(404, 8)});
    CHECK(findSidewalkRoadCrossings(m, decks).empty());
    // Drawn: the conflicts layer carries the X and its caption; without decks it says so.
    m.curbLoops.pop_back();
    m.curbLoops.push_back({Vec2(90, -30), Vec2(110, -30), Vec2(110, 30), Vec2(90, 30)});
    const std::string path = "city_map_test3.svg";
    CHECK(writeCityMapSvg(path, m, CityMapLayers::fromList("roads,sidewalks,conflicts,legend"), decks));
    const std::string svg = readAll(path);
    std::remove(path.c_str());
    CHECK(svg.find("id='layer-conflicts'") != std::string::npos);
    CHECK(svg.find("m into local") != std::string::npos);
    CHECK(svg.find("CONFLICTS: sidewalk band on built asphalt (") != std::string::npos);
    CHECK(writeCityMapSvg(path, m, CityMapLayers::fromList("conflicts")));
    const std::string none = readAll(path);
    std::remove(path.c_str());
    CHECK(none.find("<line") == std::string::npos);   // nothing measured, nothing drawn
}

// A FILL IS NOT A BRIDGE (device: "those sidewalks cross roads — why?"). On
// metro_v2_test the four deepest sidewalk-on-asphalt places were the four
// ends of two locals that cross a valley: the mesher's "elevated" test (any
// point of the reconciled profile > 1.5 m above the RAW terrain) made each
// whole street a bridge — underside, piers, no sidewalk band — so the
// arterial's band swept straight across their mouths. Same T as above, with
// the north arm crossing a 12 m dip its grade-limited profile cannot follow:
// it is still an at-grade street, keeps its band, and the census stays clean.
TEST_CASE(city_map_a_street_over_a_dip_keeps_its_sidewalk) {
    RoadEntity net;
    net.look.defaultWidth = 10.0;
    net.look.sidewalk = 3.5;
    net.graph.nodes = {RoadNode{Vec2(0, 0)}, RoadNode{Vec2(200, 0)}, RoadNode{Vec2(400, 0)},
                       RoadNode{Vec2(200, 200)}, RoadNode{Vec2(300, 0)}, RoadNode{Vec2(200, 100)}};
    net.graph.addEdge(0, 1, 10.0);
    net.graph.addEdge(1, 4, 10.0);
    net.graph.addEdge(4, 2, 10.0);
    net.graph.addEdge(1, 5, 10.0);   // the T arm north, in two edges ...
    net.graph.addEdge(5, 3, 10.0);   // ... through a valley at z = 100
    auto valley = [](Real, Real z) -> Real {
        const Real d = std::fabs(z - 100.0);
        return d < 30.0 ? -12.0 * (1.0 - d / 30.0) : Real(0);
    };
    CurbBandAudit audit;
    RoadDeckField deck;
    buildRoadNetMesh(net, valley, &audit, &deck);
    CHECK(!audit.loops.empty());
    CityMapData m;
    m.roads = net.graph;
    m.curbLoops = audit.loops;
    m.sidewalkWidth = audit.sidewalkWidth > 0 ? audit.sidewalkWidth : 3.5;
    const std::vector<const RoadDeckField*> decks{&deck};
    std::vector<SidewalkCrossing> hits = findSidewalkRoadCrossings(m, decks);
    std::printf("    [valley] %zu sidewalk-on-asphalt place(s); first at (%.1f, %.1f) %.2f m deep\n",
                hits.size(), hits.empty() ? 0.0 : hits[0].pos.x, hits.empty() ? 0.0 : hits[0].pos.y,
                hits.empty() ? 0.0 : hits[0].depth);
    CHECK(hits.empty());
    // The arm carries a band: some curb-loop vertex runs along its side
    // through the valley (x ~ 200 +- (5 + 3.5), z ~ 100).
    bool bandAlongArm = false;
    for (const Poly2& L : audit.loops)
        for (const Vec2& v : L)
            if (std::fabs(v.y - 100.0) < 12.0 && std::fabs(std::fabs(v.x - 200.0) - 7.0) < 4.0)
                bandAlongArm = true;
    CHECK(bandAlongArm);
}
