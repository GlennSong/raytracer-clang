#include "test_framework.h"
#include "../src/engine/procgen/world/island_world.h"

#include <algorithm>
#include <cmath>

using namespace engine;

// LINK ROADS (ADR-0214): a link is a straight collector from a town's arterial up to the freeway, and
// the builder grades it at that class's limit. Ground that climbs steeper than the limit over a long
// stretch it can only cut into, and the cut runs on under the freeway to where the diamond's ramps land
// (Weyby on the rocky island: 11.7 % over 330 m, the link 16 m into the hill, its ramps 16.7 %). Such a
// candidate is passed over; the flat one beside it is kept. So is one that runs along a street (Saltwood's, 14 deg
// off its arterial and 5 m beside it: the two overlapped for 100 m and the builder made it an overpass).

namespace {
// 2 km square: a freeway north-south at x = 500; flat ground at 10 m, except north of z = 0, where the
// last 200 m up to the freeway climb at 15 %.
IslandWorld slopeWorld() {
    IslandWorld w;
    w.half = 1000.0;
    w.cell = 20.0;
    w.n = 101;
    w.height.resize(static_cast<std::size_t>(w.n) * w.n);
    for (int j = 0; j < w.n; ++j)
        for (int i = 0; i < w.n; ++i) {
            const double x = -w.half + i * w.cell, z = -w.half + j * w.cell;
            w.height[static_cast<std::size_t>(j) * w.n + i] = static_cast<float>(10.0 + (z > 0.0 ? 0.15 * std::clamp(x - 300.0, 0.0, 200.0) : 0.0));
        }
    IslandSite town;
    town.kind = "town";
    town.name = "Testby";
    town.at = Vec2(0, 0);
    w.sites.push_back(town);
    IslandRoad fw;
    fw.kind = "freeway";
    fw.points = {Vec2(500, -1000), Vec2(500, 1000)};
    w.roads.push_back(fw);
    return w;
}

int links(const IslandWorld& w) { return static_cast<int>(std::count_if(w.roads.begin(), w.roads.end(), [](const IslandRoad& r) { return r.kind == "link"; })); }
}  // namespace

TEST_CASE(a_link_too_steep_for_its_class_is_passed_over) {
    const std::vector<Vec2> nodes = {Vec2(300, -400), Vec2(300, 400)};   // both 200 m from the freeway; the second at the foot of the slope
    IslandWorld any = slopeWorld();
    linkCityToFreeway(any, 0, nodes, 4, 300.0, 900.0);
    CHECK(links(any) == 2);   // no grade limit: both
    IslandWorld held = slopeWorld();
    linkCityToFreeway(held, 0, nodes, 4, 300.0, 900.0, {}, 0.10);
    CHECK(links(held) == 1);   // a collector's 10 %: the flat one only
    for (const IslandRoad& r : held.roads)
        if (r.kind == "link") CHECK(r.points.front().y < 0.0);
}

TEST_CASE(a_link_leaving_along_a_street_is_passed_over) {
    const std::vector<Vec2> nodes = {Vec2(300, -400), Vec2(300, -800)};   // flat ground, both 200 m from the freeway
    const std::vector<std::pair<Vec2, Vec2>> streets = {{Vec2(300, -405), Vec2(480, -385)},    // 6 deg off the way to the freeway, 5 m beside it
                                                         {Vec2(300, -800), Vec2(300, -1000)}};  // square to it
    IslandWorld w = slopeWorld();
    linkCityToFreeway(w, 0, nodes, 4, 300.0, 900.0, {}, 0.10, streets);
    CHECK(links(w) == 1);
    for (const IslandRoad& r : w.roads)
        if (r.kind == "link") CHECK(r.points.front().y < -600.0);
}
