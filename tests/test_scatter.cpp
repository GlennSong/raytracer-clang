#include "test_framework.h"

#include "../src/engine/scatter.h"
#include "../src/engine/terrain.h"
#include "../src/engine/noise.h"
#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

namespace {
TerrainParams flatTerrain() {
    TerrainParams t;
    t.heightScale = 0.0f;   // perfectly flat: slope 0, height 0 everywhere
    return t;
}
TerrainParams hillyTerrain() {
    TerrainParams t;
    t.size = 200; t.heightScale = 25; t.noiseScale = 0.03; t.octaves = 5;
    return t;
}
}  // namespace

TEST_CASE(scatter_is_deterministic_for_a_seed) {
    ScatterParams sp; sp.count = 300; sp.seed = 42;
    TerrainParams t = hillyTerrain();
    Noise n(1);
    auto a = scatterOnTerrain(sp, t, n);
    auto b = scatterOnTerrain(sp, t, n);
    CHECK(a.size() == b.size());
    bool same = a.size() == b.size();
    for (size_t i = 0; same && i < a.size(); i++)
        if ((a[i].position - b[i].position).lengthSquared() > 1e-18 ||
            std::fabs(a[i].yaw - b[i].yaw) > 1e-6 ||
            std::fabs(a[i].scale - b[i].scale) > 1e-6)
            same = false;
    CHECK(same);
}

TEST_CASE(scatter_respects_region_surface_scale_and_yaw) {
    ScatterParams sp; sp.count = 500; sp.seed = 7;
    sp.minScale = 0.5f; sp.maxScale = 2.0f;
    TerrainParams t = hillyTerrain();
    Noise n(2);
    auto ps = scatterOnTerrain(sp, t, n);
    CHECK(ps.size() > 0);
    CHECK(ps.size() <= 500);
    const float half = sp.regionSize * 0.5f;
    for (const Placement& p : ps) {
        CHECK(std::fabs(p.position.x) <= half + 1e-3);
        CHECK(std::fabs(p.position.z) <= half + 1e-3);
        // Rests on the surface.
        double h = terrainHeight(t, n, p.position.x, p.position.z);
        CHECK_APPROX(p.position.y, h, 1e-6);
        CHECK(p.scale >= 0.5f - 1e-4f && p.scale <= 2.0f + 1e-4f);
        CHECK(p.yaw >= 0.0f && p.yaw <= 2.0f * static_cast<float>(PI) + 1e-4f);
    }
}

TEST_CASE(scatter_altitude_band_filters) {
    // Flat terrain sits at y=0; a band entirely below 0 keeps nothing.
    ScatterParams sp; sp.count = 200; sp.seed = 3;
    sp.minHeight = -100.0f; sp.maxHeight = -1.0f;
    TerrainParams t = flatTerrain();
    Noise n(1);
    CHECK(scatterOnTerrain(sp, t, n).empty());
}

TEST_CASE(scatter_slope_filter_rejects_steep_ground) {
    // A near-zero max slope keeps far fewer placements on hilly terrain than a
    // permissive one (which keeps ~all candidates that pass density).
    TerrainParams t = hillyTerrain();
    Noise n(4);
    ScatterParams loose; loose.count = 500; loose.seed = 11; loose.maxSlopeDeg = 89.0f;
    ScatterParams strict = loose; strict.maxSlopeDeg = 0.5f;
    size_t many = scatterOnTerrain(loose, t, n).size();
    size_t few  = scatterOnTerrain(strict, t, n).size();
    CHECK(few < many);
}

TEST_CASE(scatter_density_threshold_filters) {
    // Raising the density threshold keeps strictly fewer placements.
    TerrainParams t = hillyTerrain();
    Noise n(6);
    ScatterParams open; open.count = 500; open.seed = 21;
    open.maxSlopeDeg = 89.0f; open.densityThreshold = -1.0f;   // keep ~all
    ScatterParams picky = open; picky.densityThreshold = 0.3f; // keep dense spots
    CHECK(scatterOnTerrain(picky, t, n).size() < scatterOnTerrain(open, t, n).size());
}
