#include "test_framework.h"

#include "../src/engine/terrain.h"
#include "../src/engine/noise.h"
#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(terrain_mesh_has_grid_topology) {
    TerrainParams p;
    p.resolution = 8;
    Noise noise(1);
    RenderMesh m = generateTerrain(p, noise);
    CHECK(m.vertices.size() == static_cast<size_t>((8 + 1) * (8 + 1)));
    CHECK(m.indices.size() == static_cast<size_t>(8 * 8 * 6));
    // No index is out of range.
    uint32_t maxIdx = 0;
    for (uint32_t i : m.indices) maxIdx = std::max(maxIdx, i);
    CHECK(maxIdx < m.vertices.size());
}

TEST_CASE(terrain_flat_when_height_zero) {
    TerrainParams p;
    p.resolution = 8;
    p.heightScale = 0.0f;
    Noise noise(1);
    RenderMesh m = generateTerrain(p, noise);
    for (const Vertex& v : m.vertices) {
        CHECK_APPROX(v.position.y, 0.0, 1e-6);
        CHECK_APPROX(v.normal.y, 1.0, 1e-4);   // flat ground faces up
    }
}

TEST_CASE(terrain_is_deterministic_for_a_seed) {
    TerrainParams p;
    p.resolution = 16;
    Noise a(7), b(7);
    RenderMesh ma = generateTerrain(p, a);
    RenderMesh mb = generateTerrain(p, b);
    CHECK(ma.vertices.size() == mb.vertices.size());
    bool allEqual = true;
    for (size_t i = 0; i < ma.vertices.size(); i++)
        if (std::fabs(ma.vertices[i].position.y - mb.vertices[i].position.y) > 1e-12)
            allEqual = false;
    CHECK(allEqual);
}

TEST_CASE(terrain_mesh_height_matches_field_query) {
    // The mesh and the scatter-facing height query share one source of truth.
    TerrainParams p;
    p.resolution = 4;
    p.size = 40.0f;
    Noise noise(3);
    RenderMesh m = generateTerrain(p, noise);
    for (const Vertex& v : m.vertices) {
        double h = terrainHeight(p, noise, v.position.x, v.position.z);
        CHECK_APPROX(v.position.y, h, 1e-9);
    }
}

TEST_CASE(terrain_height_bounded_by_scale) {
    TerrainParams p;
    p.heightScale = 12.0f;
    Noise noise(5);
    double maxAbs = 0.0;
    for (int i = 0; i < 500; i++) {
        double x = i * 1.7, z = i * 0.9;
        maxAbs = std::max(maxAbs, std::fabs(terrainHeight(p, noise, x, z)));
    }
    CHECK(maxAbs <= 12.0 * 1.05);
}
