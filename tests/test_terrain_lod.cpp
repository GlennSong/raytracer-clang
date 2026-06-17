#include "test_framework.h"

#include "../src/engine/procgen/terrain_lod.h"
#include "../src/engine/procgen/noise.h"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace engine;  // namespace migration (ADR-0015)

// --- LOD ranges & morph bands ---

TEST_CASE(lod_ranges_increase_and_double) {
    auto r = defaultLodRanges(5, 32.0f);
    CHECK(r.size() == 5u);
    for (size_t i = 1; i < r.size(); i++) {
        CHECK(r[i] > r[i - 1]);
        CHECK_APPROX(r[i], r[i - 1] * 2.0, 1e-4);
    }
    CHECK_APPROX(r[0], 32.0, 1e-4);
}

TEST_CASE(morph_band_is_between_consecutive_ranges) {
    auto r = defaultLodRanges(4, 50.0f);   // 50, 100, 200, 400
    MorphRange m2 = lodMorphRange(2, r);
    CHECK_APPROX(m2.start, 100.0, 1e-4);   // ranges[1]
    CHECK_APPROX(m2.end, 200.0, 1e-4);     // ranges[2]
    // Level 0 morphs from 0 (near camera) to its range.
    MorphRange m0 = lodMorphRange(0, r);
    CHECK_APPROX(m0.start, 0.0, 1e-4);
    CHECK_APPROX(m0.end, 50.0, 1e-4);
    CHECK(m0.end > m0.start);
}

// --- Nearest-point box distance ---

TEST_CASE(box_distance_zero_inside_and_correct_outside) {
    // Box [0,10] x [0,10].
    CHECK_APPROX(distanceToBoxXZ(5, 5, 0, 0, 10), 0.0, 1e-6);   // inside
    CHECK_APPROX(distanceToBoxXZ(0, 0, 0, 0, 10), 0.0, 1e-6);   // on corner
    CHECK_APPROX(distanceToBoxXZ(13, 5, 0, 0, 10), 3.0, 1e-6);  // +x face
    CHECK_APPROX(distanceToBoxXZ(5, -4, 0, 0, 10), 4.0, 1e-6);  // -z face
    CHECK_APPROX(distanceToBoxXZ(13, 14, 0, 0, 10), 5.0, 1e-6); // corner (3,4)
}

// --- Quadtree selection ---

namespace {
bool nodeContains(const LodNode& n, float x, float z) {
    return x >= n.minX && x < n.minX + n.size && z >= n.minZ && z < n.minZ + n.size;
}
// Two squares are edge-adjacent if they touch on one side with a positive overlap
// along the shared edge.
bool adjacent(const LodNode& a, const LodNode& b) {
    auto overlap = [](float lo1, float hi1, float lo2, float hi2) {
        return std::min(hi1, hi2) - std::max(lo1, lo2) > 1e-3f;
    };
    bool touchX = std::fabs((a.minX + a.size) - b.minX) < 1e-3f ||
                  std::fabs((b.minX + b.size) - a.minX) < 1e-3f;
    bool touchZ = std::fabs((a.minZ + a.size) - b.minZ) < 1e-3f ||
                  std::fabs((b.minZ + b.size) - a.minZ) < 1e-3f;
    if (touchX && overlap(a.minZ, a.minZ + a.size, b.minZ, b.minZ + b.size))
        return true;
    if (touchZ && overlap(a.minX, a.minX + a.size, b.minX, b.minX + b.size))
        return true;
    return false;
}
}  // namespace

TEST_CASE(selection_covers_world_area_exactly) {
    float worldHalf = 1024.0f;
    auto r = lodRangesForWorld(worldHalf, 6);
    auto nodes = selectLodNodes(worldHalf, 6, r, 0.0f, 0.0f);
    CHECK(!nodes.empty());
    double area = 0.0;
    for (const auto& n : nodes) area += static_cast<double>(n.size) * n.size;
    double world = static_cast<double>(worldHalf * 2.0) * (worldHalf * 2.0);
    CHECK_APPROX(area, world, world * 1e-6);
}

TEST_CASE(selection_each_point_in_exactly_one_node) {
    float worldHalf = 512.0f;
    auto r = lodRangesForWorld(worldHalf, 5);
    auto nodes = selectLodNodes(worldHalf, 5, r, 100.0f, -60.0f);
    // Sample an irregular grid (avoid landing exactly on node borders).
    for (int j = 0; j < 17; j++) {
        for (int i = 0; i < 17; i++) {
            float x = -worldHalf + (i + 0.37f) * (worldHalf * 2.0f / 17.0f);
            float z = -worldHalf + (j + 0.61f) * (worldHalf * 2.0f / 17.0f);
            int hits = 0;
            for (const auto& n : nodes) if (nodeContains(n, x, z)) hits++;
            CHECK(hits == 1);
        }
    }
}

TEST_CASE(selection_is_finest_at_camera_and_coarsens_away) {
    float worldHalf = 1024.0f;
    int numLods = 6;
    auto r = lodRangesForWorld(worldHalf, numLods);
    auto nodes = selectLodNodes(worldHalf, numLods, r, 0.0f, 0.0f);
    // The node under the camera (origin) is the finest level.
    int camLevel = -1, maxLevel = 0;
    for (const auto& n : nodes) {
        if (nodeContains(n, 0.0f, 0.0f)) camLevel = n.level;
        maxLevel = std::max(maxLevel, n.level);
    }
    CHECK(camLevel == 0);
    CHECK(maxLevel >= 1);                 // the far field coarsened
    CHECK(maxLevel <= numLods - 1);
}

TEST_CASE(selection_neighbors_differ_by_at_most_one_level) {
    // The nearest-point distance criterion guarantees this (the CDLOD invariant the
    // morph relies on). Verify across a few camera placements.
    float worldHalf = 1024.0f;
    auto r = lodRangesForWorld(worldHalf, 6);
    const float cams[][2] = {{0, 0}, {300, -200}, {900, 900}, {-512, 128}};
    for (auto& c : cams) {
        auto nodes = selectLodNodes(worldHalf, 6, r, c[0], c[1]);
        for (size_t a = 0; a < nodes.size(); a++)
            for (size_t b = a + 1; b < nodes.size(); b++)
                if (adjacent(nodes[a], nodes[b]))
                    CHECK(std::abs(nodes[a].level - nodes[b].level) <= 1);
    }
}

TEST_CASE(selection_single_lod_is_one_world_node) {
    auto nodes = selectLodNodes(256.0f, 1, {}, 0.0f, 0.0f);
    CHECK(nodes.size() == 1u);
    CHECK(nodes[0].level == 0);
    CHECK_APPROX(nodes[0].size, 512.0, 1e-4);
    CHECK_APPROX(nodes[0].minX, -256.0, 1e-4);
}

// --- Node mesh + morph targets ---

TEST_CASE(node_mesh_topology_and_even_res) {
    TerrainParams p;
    Noise noise(1);
    LodNode node{0.0f, 0.0f, 100.0f, 0};
    // Pass odd res -> forced even (8).
    LodNodeMesh out = generateLodNodeMesh(p, noise, node, 7);
    int res = 8;
    CHECK(out.mesh.vertices.size() == static_cast<size_t>((res + 1) * (res + 1)));
    CHECK(out.mesh.indices.size() == static_cast<size_t>(res * res * 6));
    uint32_t maxIdx = 0;
    for (uint32_t i : out.mesh.indices) maxIdx = std::max(maxIdx, i);
    CHECK(maxIdx < out.mesh.vertices.size());
    // AABB contains every vertex.
    for (const Vertex& v : out.mesh.vertices) {
        CHECK(v.position.x >= out.boundsMin.x - 1e-3 && v.position.x <= out.boundsMax.x + 1e-3);
        CHECK(v.position.y >= out.boundsMin.y - 1e-3 && v.position.y <= out.boundsMax.y + 1e-3);
        CHECK(v.position.z >= out.boundsMin.z - 1e-3 && v.position.z <= out.boundsMax.z + 1e-3);
    }
}

TEST_CASE(morph_target_is_identity_on_flat_terrain) {
    TerrainParams p;
    p.heightScale = 0.0f;          // perfectly flat
    Noise noise(3);
    LodNode node{-50.0f, -50.0f, 100.0f, 1};
    LodNodeMesh out = generateLodNodeMesh(p, noise, node, 8);
    for (const Vertex& v : out.mesh.vertices) {
        // tangent holds the morph-target position; on flat ground it equals position.
        CHECK_APPROX(v.tangent.x, v.position.x, 1e-4);
        CHECK_APPROX(v.tangent.y, v.position.y, 1e-4);
        CHECK_APPROX(v.tangent.z, v.position.z, 1e-4);
    }
}

TEST_CASE(morph_target_collapses_odd_vertices_to_coarse_height) {
    TerrainParams p;
    p.heightScale = 12.0f;
    p.octaves = 5;
    Noise noise(9);
    int res = 8;
    LodNode node{0.0f, 0.0f, 200.0f, 0};
    LodNodeMesh out = generateLodNodeMesh(p, noise, node, res);
    int n = res + 1;
    auto V = [&](int i, int j) -> const Vertex& {
        return out.mesh.vertices[static_cast<size_t>(j) * n + i];
    };
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            const Vertex& v = V(i, j);
            // XZ never moves under the collapse.
            CHECK_APPROX(v.tangent.x, v.position.x, 1e-4);
            CHECK_APPROX(v.tangent.z, v.position.z, 1e-4);
            bool ei = (i % 2 == 0), ej = (j % 2 == 0);
            if (ei && ej) {
                CHECK_APPROX(v.tangent.y, v.position.y, 1e-4);   // survives coarsening
            } else if (!ei && ej) {
                double mid = 0.5 * (V(i - 1, j).position.y + V(i + 1, j).position.y);
                CHECK_APPROX(v.tangent.y, mid, 1e-3);
            } else if (ei && !ej) {
                double mid = 0.5 * (V(i, j - 1).position.y + V(i, j + 1).position.y);
                CHECK_APPROX(v.tangent.y, mid, 1e-3);
            } else {
                double mid = 0.25 * (V(i - 1, j - 1).position.y + V(i + 1, j - 1).position.y +
                                     V(i - 1, j + 1).position.y + V(i + 1, j + 1).position.y);
                CHECK_APPROX(v.tangent.y, mid, 1e-3);
            }
        }
    }
}

TEST_CASE(consistent_eps_makes_lod_boundary_normals_seamless) {
    // A fine tile and a coarse tile sharing the edge x=100, built with the SAME
    // normalEps, must have identical normals at coincident edge vertices — that is
    // what removes the shading seam between different LOD levels.
    TerrainParams p;
    p.heightScale = 20.0f;
    p.octaves = 5;
    Noise noise(11);
    double eps = 3.0;
    LodNode fine{0.0f, 0.0f, 100.0f, 0};      // step 12.5 at gridRes 8
    LodNode coarse{100.0f, 0.0f, 200.0f, 1};  // step 25 at gridRes 8
    LodNodeMesh A = generateLodNodeMesh(p, noise, fine, 8, eps);
    LodNodeMesh B = generateLodNodeMesh(p, noise, coarse, 8, eps);
    auto findVert = [](const RenderMesh& m, float x, float z) -> const Vertex* {
        for (const Vertex& v : m.vertices)
            if (std::fabs(v.position.x - x) < 1e-3 && std::fabs(v.position.z - z) < 1e-3)
                return &v;
        return nullptr;
    };
    for (float z : {0.0f, 25.0f, 50.0f, 75.0f, 100.0f}) {
        const Vertex* a = findVert(A.mesh, 100.0f, z);
        const Vertex* b = findVert(B.mesh, 100.0f, z);
        CHECK(a != nullptr && b != nullptr);
        if (a && b) {
            CHECK_APPROX(a->normal.x, b->normal.x, 1e-4);
            CHECK_APPROX(a->normal.y, b->normal.y, 1e-4);
            CHECK_APPROX(a->normal.z, b->normal.z, 1e-4);
        }
    }
}

TEST_CASE(node_surface_matches_height_field) {
    // The node mesh samples the same height field as the rest of the terrain, so a
    // vertex's y is terrainHeight at its XZ (parity oracle with ADR-0035 chunks).
    TerrainParams p;
    p.heightScale = 8.0f;
    Noise noise(5);
    LodNode node{20.0f, -40.0f, 160.0f, 2};
    LodNodeMesh out = generateLodNodeMesh(p, noise, node, 8);
    for (const Vertex& v : out.mesh.vertices) {
        double h = terrainHeight(p, noise, v.position.x, v.position.z);
        CHECK_APPROX(v.position.y, h, 1e-3);
    }
}
