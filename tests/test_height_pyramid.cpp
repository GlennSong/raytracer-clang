#include "test_framework.h"
#include "../src/engine/procgen/height_pyramid.h"
#include "../src/job_system.h"
#include <algorithm>
#include <cmath>
using namespace engine;
using namespace engine::pyramid;

// The baked height pyramid (ADR-0095): tiles meet exactly, parents are their children's even
// samples, detail is stored only where it changes the drawn surface, and the codec round-trips.

TEST_CASE(pyramid_covering_spans_the_rectangle) {
    const PyramidSpec s = PyramidSpec::covering(-3000, -3000, 3000, 3000, 1.0, 0.02);
    CHECK(s.extent() >= 6000.0);
    CHECK(s.extent() < 12000.0 + 1e-9);   // the smallest that does
    CHECK(s.levels == 7);                  // 128 m tiles at level 0, 8192 m at the top
}

TEST_CASE(pyramid_of_flat_ground_is_one_tile) {
    const PyramidSpec s = PyramidSpec::covering(0, 0, 1000, 1000, 1.0, 0.02);
    const Pyramid p = buildPyramid(s, [](double, double, double) { return 12.5; });
    CHECK(p.tiles.size() == 1);
    CHECK(std::fabs(p.height(400, 700) - 12.5) < 1e-3);
}

TEST_CASE(pyramid_tiles_share_borders_and_parents_are_even_samples) {
    // Detail everywhere: a ripple finer than any coarse cell forces every tile down to level 0.
    const PyramidSpec s = PyramidSpec::covering(0, 0, 512, 512, 1.0, 0.001);
    auto h = [](double x, double z, double) { return 3.0 * std::sin(x * 0.9) * std::cos(z * 0.7); };
    const Pyramid p = buildPyramid(s, h);
    const HeightTile* a = p.find({0, 0, 0});
    const HeightTile* b = p.find({0, 1, 0});   // east neighbour
    const HeightTile* up = p.find({1, 0, 0});
    CHECK(a && b && up);
    if (!a || !b || !up) return;
    // a's last column is b's first column, sample for sample (to quantization).
    double worst = 0;
    for (int j = 0; j < kTileSamples; ++j) worst = std::max(worst, std::fabs(a->at(kTileCells, j) - b->at(0, j)));
    CHECK(worst < 2 * std::max(a->step, b->step));
    // The parent's sample (i, j) is the child's (2i, 2j) within the child quadrant.
    worst = 0;
    for (int j = 0; j <= kTileCells / 2; ++j)
        for (int i = 0; i <= kTileCells / 2; ++i) worst = std::max(worst, std::fabs(up->at(i, j) - a->at(2 * i, 2 * j)));
    CHECK(worst < 2 * std::max(up->step, a->step));
}

TEST_CASE(pyramid_refines_only_where_the_ground_changes) {
    // Flat but for a 30 m bump near one corner of a 2 km square.
    const PyramidSpec s = PyramidSpec::covering(0, 0, 2000, 2000, 1.0, 0.02);
    auto h = [](double x, double z, double) {
        const double d = std::hypot(x - 150.0, z - 150.0);
        return d < 30.0 ? 2.0 * std::cos(d / 30.0 * 1.5707963) : 0.0;
    };
    JobSystem jobs(2);
    const Pyramid p = buildPyramid(s, h, &jobs);
    int level0 = 0;
    for (const auto& kv : p.tiles) if (kv.first.level == 0) ++level0;
    std::printf("    [pyramid] %zu tiles, %d at level 0 (a full level 0 would be %d)\n", p.tiles.size(), level0, 16 * 16);
    CHECK(level0 >= 1 && level0 <= 4);                 // only around the bump
    CHECK(p.finestAt(1800, 1800)->key.level > 0);      // the far flat corner stays coarse
    // Wherever it is stored, the drawn surface matches the ground at the finest samples.
    double worst = 0;
    for (double z = 100; z <= 200; z += 1.0)
        for (double x = 100; x <= 200; x += 1.0) worst = std::max(worst, std::fabs(p.height(x, z) - h(x, z, 1.0)));
    CHECK(worst < 0.01);
}

TEST_CASE(pyramid_tile_codec_round_trips) {
    const PyramidSpec s = PyramidSpec::covering(0, 0, 300, 300, 1.0, 0.02);
    const Pyramid p = buildPyramid(s, [](double x, double z, double) { return x * 0.01 - z * 0.02; });
    const HeightTile& t = p.tiles.begin()->second;
    const std::vector<uint8_t> bytes = encodeTile(t);
    HeightTile back;
    CHECK(decodeTile(bytes.data(), bytes.size(), back));
    CHECK(back.key == t.key);
    CHECK(back.q == t.q);
    CHECK(back.minH == t.minH && back.step == t.step && back.hasChildren == t.hasChildren);
    CHECK(!decodeTile(bytes.data(), bytes.size() - 1, back));   // a truncated section is refused
    CHECK(tileSectionName({3, 4, -2}) == "terrain/L3/4_-2");
}

TEST_CASE(pyramid_bundle_round_trips_and_refuses_another_key) {
    const PyramidSpec s = PyramidSpec::covering(0, 0, 600, 600, 1.0, 0.02);
    const Pyramid p = buildPyramid(s, [](double x, double z, double) { return std::sin(x * 0.05) * 3 + z * 0.01; });
    const std::string path = "/tmp/rt_test_pyramid.bundle";
    std::string err;
    CHECK(writePyramidBundle(p, 0x1234, path, &err));
    Pyramid back;
    CHECK(readPyramidBundle(path, 0x1234, back, &err));
    CHECK(back.tiles.size() == p.tiles.size());
    CHECK(back.spec.levels == p.spec.levels && back.spec.cell0 == p.spec.cell0);
    double worst = 0;
    for (double z = 5; z < 600; z += 37) for (double x = 3; x < 600; x += 41) worst = std::max(worst, std::fabs(back.height(x, z) - p.height(x, z)));
    CHECK(worst < 1e-9);
    Pyramid other;
    CHECK(!readPyramidBundle(path, 0x9999, other, &err));   // another key: a miss, not a wrong ground
    std::remove(path.c_str());
}

// A must-refine rule (ADR-0134: the trails) refines flat ground the heights alone would leave at one
// tile, down to level 0 where it asks, and nowhere else.
TEST_CASE(pyramid_refines_where_a_rule_demands_even_on_flat_ground) {
    const auto spec = pyramid::PyramidSpec::covering(0, 0, 1000, 1000, 1.0, 0.02);
    const pyramid::Pyramid p = pyramid::buildPyramid(
        spec, [](double, double, double) { return 5.0; }, nullptr,
        [](double x0, double z0, double, double, int) { return x0 <= 10.0 && z0 <= 10.0; });   // the corner tiles
    int level0 = 0, level0Far = 0;
    for (const auto& [k, t] : p.tiles)
        if (k.level == 0) { ++level0; if (k.tx > 1 || k.tz > 1) ++level0Far; }   // a refined tile stores all four children
    CHECK(level0 >= 1);
    CHECK(level0Far == 0);
}
