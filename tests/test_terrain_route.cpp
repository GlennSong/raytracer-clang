#include "test_framework.h"
#include "../src/engine/procgen/city/terrain_route.h"

#include <cmath>

using namespace engine;

// Flat ground: the road goes (nearly) straight.
TEST_CASE(terrain_route_is_straight_on_flat_ground) {
    const HeightField flat = [](double, double) { return 0.0; };
    const TerrainRoute r = routeOnTerrain(flat, Vec2(0, 0), Vec2(400, 150), {});
    CHECK(!r.points.empty());
    const double straight = std::hypot(400.0, 150.0);
    CHECK(r.length < straight * 1.03);
    CHECK(r.worstGrade < 1e-9);
}

// A mountain with 30 % flanks and a summit plateau 150 m up: the road may not climb it straight (8 %
// grade, 14 % hard), so it winds -- longer than the climb allows at the hard grade, never steeper.
// (A needle peak has no answer at all: near a point, any move climbs too fast. Real tops are flat.)
TEST_CASE(terrain_route_winds_up_a_mountain_within_its_grade) {
    const HeightField cone = [](double x, double y) { return std::min(150.0, std::max(0.0, 180.0 - 0.3 * std::hypot(x, y))); };
    TerrainRouteParams p;
    p.maxGrade = 0.08;
    p.hardGrade = 0.14;
    p.margin = 300.0;
    const Vec2 foot(700, 0), summit(0, 0);
    const TerrainRoute r = routeOnTerrain(cone, foot, summit, p);
    CHECK(!r.points.empty());
    CHECK(r.climb > 140.0);
    CHECK(r.length > 150.0 / 0.14);   // a straight 30 % line is 500 m; the road must be far longer
    CHECK(r.worstGrade < 0.16);       // simplified segments: the hard limit plus a little slack
    std::printf("[route] cone: %.0f m long, %.0f m climbed, worst %.3f, %zu points, %ld states\n",
                r.length, r.climb, r.worstGrade, r.points.size(), r.expanded);
}
