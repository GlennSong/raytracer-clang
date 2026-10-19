#include "test_framework.h"

#include "../src/engine/procgen/terrain_lod.h"
#include "../src/engine/procgen/noise.h"
#include "../src/engine/procgen/terrain_normal_map.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
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
        // tangent holds the morph target (packMorphTarget): its height, and the normal it blends to in x and z;
        // on flat ground the height is the vertex's own and the normal straight up.
        CHECK_APPROX(v.tangent.x, 0.0, 1e-4);
        CHECK_APPROX(v.tangent.y, v.position.y, 1e-4);
        CHECK_APPROX(v.tangent.z, 0.0, 1e-4);
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
            // the formula's levels share one normal eps: the morph's normal is the vertex's own
            CHECK_APPROX(v.tangent.x, v.normal.x, 1e-6);
            CHECK_APPROX(v.tangent.z, v.normal.z, 1e-6);
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

// NORMALS BY LEVEL (ADR-0204; Glenn: the far grassland "looks faceted"): a baked tile's vertex normals take the slope
// over the tile's own cell, so a far tile's 8 m vertices follow the hills rather than whatever 1 m bump each one
// stands on; and a tile's morph target carries its parent's normals, so the two shade alike where they meet.
TEST_CASE(baked_tile_normals_follow_their_level_and_morph_to_the_parents) {
    const pyramid::PyramidSpec s = pyramid::PyramidSpec::covering(0, 0, 1024, 1024, 1.0, 0.001);
    auto hills = [](double x, double z) { return 40.0 * std::sin(x * 0.004) * std::cos(z * 0.003); };
    auto h = [&](double x, double z, double) { return hills(x, z) + 0.6 * std::sin(x * 1.3 + 0.4) * std::sin(z * 1.1); };
    const pyramid::Pyramid p = pyramid::buildPyramid(s, h);
    const pyramid::HeightTile* child = p.find({2, 0, 0});
    const pyramid::HeightTile* parent = p.find({3, 0, 0});
    CHECK(child && parent);
    if (!child || !parent) return;
    TerrainParams params;
    Noise noise(1);
    const LodNodeMesh pm = generateBakedTileMesh(p, *parent, params, noise, 2.0);
    const auto t0 = std::chrono::steady_clock::now();
    const LodNodeMesh cm = generateBakedTileMesh(p, *child, params, noise, 2.0);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // the parent's (8 m) normals against the hills' own, and what a 2 m span would have given
    double offLevel = 0, off2m = 0;
    int count = 0;
    for (int j = 4; j < pyramid::kTileSamples - 4; j += 4)   // (inside: the border's spans reach past the pyramid)
        for (int i = 4; i < pyramid::kTileSamples - 4; i += 4) {
            const Vertex& v = pm.mesh.vertices[static_cast<std::size_t>(j) * pyramid::kTileSamples + i];
            const double x = v.position.x, z = v.position.z, e = 0.5;
            const Vec3 truth = normalize(Vec3(-(hills(x + e, z) - hills(x - e, z)) / (2 * e), 1.0, -(hills(x, z + e) - hills(x, z - e)) / (2 * e)));
            offLevel += std::acos(std::min(1.0, dot(v.normal, truth)));
            off2m += std::acos(std::min(1.0, dot(bakedTileNormal(p, x, z, 2.0), truth)));
            ++count;
        }
    offLevel /= count;
    off2m /= count;
    std::printf("    [baked normals] off the hills: %.2f deg over the 8 m cell, %.2f deg over 2 m; a tile in %.1f ms\n", offLevel * 57.2958,
                off2m * 57.2958, ms);
    CHECK(offLevel < 0.5 * off2m);
    // the child's even vertices morph to the parent's normals exactly
    double worst = 0;
    for (int j = 0; j < pyramid::kTileSamples; j += 2)
        for (int i = 0; i < pyramid::kTileSamples; i += 2) {
            const Vertex& c = cm.mesh.vertices[static_cast<std::size_t>(j) * pyramid::kTileSamples + i];
            const Vertex& q = pm.mesh.vertices[static_cast<std::size_t>(j / 2) * pyramid::kTileSamples + i / 2];
            worst = std::max({worst, std::fabs(c.tangent.x - q.normal.x), std::fabs(c.tangent.z - q.normal.z)});
        }
    CHECK(worst < 1e-9);
}

// THE TERRAIN NORMAL MAP (ADR-0205): a tilted plane comes back as its own normal; bumps finer than a texel (but above
// the bake's half-texel samples) average away -- the far ground shows the hills, not noise; the disk cache round-trips
// and refuses another key.
TEST_CASE(terrain_normal_map_holds_the_mean_slope_per_texel) {
    auto plane = [](double x, double z) { return 0.2 * x - 0.1 * z; };
    const TerrainNormalMap pm = bakeTerrainNormalMap(plane, -256, -256, 512, 64);
    const Vec3 want = normalize(Vec3(-0.2, 1.0, 0.1));
    double worst = 0;
    for (double z = -200; z <= 200; z += 37)
        for (double x = -200; x <= 200; x += 41) worst = std::max(worst, std::acos(std::min(1.0, dot(pm.at(x, z), want))));
    CHECK(worst < 0.01);   // (8-bit: ~0.5 deg)
    auto hills = [](double x, double z) { return 30.0 * std::sin(x * 0.006) * std::cos(z * 0.005); };
    auto bumpy = [&](double x, double z) { return hills(x, z) + 0.5 * std::sin(x * 1.0 + 1.0) * std::sin(z * 0.9); };   // (6-7 m)
    const TerrainNormalMap bm = bakeTerrainNormalMap(bumpy, -512, -512, 1024, 128);   // 8 m texels
    double off = 0;
    int n = 0;
    for (double z = -400; z <= 400; z += 23)
        for (double x = -400; x <= 400; x += 29) {
            const double cx = bm.originX + (std::floor((x - bm.originX) / bm.texel()) + 0.5) * bm.texel();
            const double cz = bm.originZ + (std::floor((z - bm.originZ) / bm.texel()) + 0.5) * bm.texel(), e = 0.5;
            const Vec3 truth = normalize(Vec3(-(hills(cx + e, cz) - hills(cx - e, cz)) / (2 * e), 1.0, -(hills(cx, cz + e) - hills(cx, cz - e)) / (2 * e)));
            off += std::acos(std::min(1.0, dot(bm.at(x, z), truth)));
            ++n;
        }
    off /= n;
    std::printf("    [terrain normal map] bumps under an 8 m texel: %.2f deg off the hills\n", off * 57.2958);
    CHECK(off < 1.0 / 57.2958);
    const std::string path = "/tmp/rt_test_terrain_normal_map.nmap";
    CHECK(writeTerrainNormalMap(bm, 0x1234u, path));
    TerrainNormalMap back;
    CHECK(readTerrainNormalMap(path, 0x1234u, back));
    CHECK(back.size == bm.size && back.xz == bm.xz && back.extent == bm.extent && back.originX == bm.originX);
    TerrainNormalMap other;
    CHECK(!readTerrainNormalMap(path, 0x9999u, other));
    std::remove(path.c_str());
}
