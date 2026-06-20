#include "test_framework.h"

#include "../src/engine/procgen/surface_maps.h"
#include <algorithm>
#include <cstdlib>

using namespace engine;  // namespace migration (ADR-0015)
using Surface = RenderMaterial::Surface;

TEST_CASE(surface_maps_bakes_full_pbr_set) {
    SurfaceMaps m = surfaceMaps(Surface::Brick, 64, 7);
    CHECK(m.albedo.width == 64 && m.albedo.height == 64 && m.albedo.channels == 3);
    CHECK(m.normal.channels == 3);
    CHECK(m.mr.channels == 3);
    CHECK(m.ao.channels == 1);
    CHECK(m.height.channels == 1);
    CHECK(m.albedo.pixels.size() == static_cast<size_t>(64) * 64 * 3);

    // The normal map faces predominantly outward (+Z dominant, b ~ 0.9..1.0).
    double avgB = 0;
    for (size_t i = 2; i < m.normal.pixels.size(); i += 3) avgB += m.normal.pixels[i];
    avgB /= (64.0 * 64.0);
    CHECK(avgB > 170);

    // Brick albedo varies (mortar vs faces), not a flat fill.
    uint8_t mn = 255, mx = 0;
    for (size_t i = 0; i < m.albedo.pixels.size(); i += 3) {
        mn = std::min(mn, m.albedo.pixels[i]);
        mx = std::max(mx, m.albedo.pixels[i]);
    }
    CHECK(mx - mn > 30);
}

TEST_CASE(surface_maps_is_deterministic) {
    SurfaceMaps a = surfaceMaps(Surface::Cobblestone, 32, 3);
    SurfaceMaps b = surfaceMaps(Surface::Cobblestone, 32, 3);
    CHECK(a.albedo.pixels == b.albedo.pixels);
    CHECK(a.normal.pixels == b.normal.pixels);
}

TEST_CASE(surface_maps_tile_seamlessly) {
    // Column 0 and the last column are neighbours across the wrap, so a seamless
    // tile keeps them close (no visible seam when the texture repeats).
    const int size = 64;
    SurfaceMaps m = surfaceMaps(Surface::Brick, size, 1);
    long diff = 0;
    for (int y = 0; y < size; ++y)
        for (int c = 0; c < 3; ++c)
            diff += std::abs(int(m.albedo.pixels[(y * size + 0) * 3 + c]) -
                             int(m.albedo.pixels[(y * size + size - 1) * 3 + c]));
    CHECK(diff / static_cast<double>(size * 3) < 40.0);
}

TEST_CASE(surface_maps_metal_is_metallic) {
    // The corrugated-metal MR map carries real metalness (B channel high where
    // it isn't rusted); masonry stays dielectric.
    SurfaceMaps metal = surfaceMaps(Surface::CorrugatedMetal, 48, 2);
    SurfaceMaps brick = surfaceMaps(Surface::Brick, 48, 2);
    double mMetal = 0, bMetal = 0;
    for (size_t i = 2; i < metal.mr.pixels.size(); i += 3) mMetal += metal.mr.pixels[i];
    for (size_t i = 2; i < brick.mr.pixels.size(); i += 3) bMetal += brick.mr.pixels[i];
    CHECK(mMetal > bMetal * 4);   // metal far more metallic than brick (~0)
}
