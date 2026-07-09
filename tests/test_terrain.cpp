#include "test_framework.h"

#include "../src/engine/procgen/terrain.h"
#include "../src/engine/procgen/erosion.h"
#include "../src/engine/procgen/noise.h"
#include <algorithm>
#include <cmath>
#include <vector>

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

TEST_CASE(terrain_flatten_pad_levels_ground) {
    TerrainParams p;
    p.heightScale = 20.0f;   // bumpy ground so flattening is visible
    p.noiseScale = 0.05;
    Noise noise(7);

    // A 40x40 pad centred at the origin, graded flat to y = 3.
    std::vector<Vec3> square = {{-20, 0, -20}, {20, 0, -20}, {20, 0, 20}, {-20, 0, 20}};
    p.flatten.push_back(makeFlattenPad(square, 3.0, 6.0));

    // Inside the pad the ground is exactly the target, regardless of the noise.
    CHECK_APPROX(terrainHeight(p, noise, 0, 0), 3.0, 1e-9);
    CHECK_APPROX(terrainHeight(p, noise, 10, -8), 3.0, 1e-9);

    // Far outside, the natural terrain is untouched.
    double natural = [&] {
        TerrainParams q = p; q.flatten.clear();
        return terrainHeight(q, noise, 100, 100);
    }();
    CHECK_APPROX(terrainHeight(p, noise, 100, 100), natural, 1e-9);

    // On the falloff skirt (3 m outside the edge) the height sits between the pad
    // and the natural ground — a blended ramp, not a cliff.
    double edge = terrainHeight(p, noise, 23, 0);
    double naturalEdge = [&] {
        TerrainParams q = p; q.flatten.clear();
        return terrainHeight(q, noise, 23, 0);
    }();
    CHECK(std::min(3.0, naturalEdge) - 1e-6 <= edge);
    CHECK(edge <= std::max(3.0, naturalEdge) + 1e-6);
}

TEST_CASE(terrain_flatten_ramp_follows_road_grade) {
    TerrainParams p;
    p.heightScale = 15.0f;
    Noise noise(3);

    // A road corridor from (0,0) climbing to (100,0), grade 2 -> 12.
    p.flatten.push_back(
        makeFlattenRamp(Vec3(0, 0, 0), Vec3(100, 0, 0), 2.0, 12.0, 6.0, 5.0));

    CHECK_APPROX(terrainHeight(p, noise, 0, 0), 2.0, 1e-6);    // start grade
    CHECK_APPROX(terrainHeight(p, noise, 50, 0), 7.0, 1e-6);   // midpoint
    CHECK_APPROX(terrainHeight(p, noise, 100, 0), 12.0, 1e-6); // end grade
    // Across the corridor (within the half-width) the grade holds.
    CHECK_APPROX(terrainHeight(p, noise, 50, 4), 7.0, 1e-6);
}

TEST_CASE(terrain_flatten_mesh_conforms) {
    TerrainParams p;
    p.resolution = 32;
    p.size = 100.0f;
    p.heightScale = 18.0f;
    p.noiseScale = 0.06;
    Noise noise(11);
    std::vector<Vec3> square = {{-15, 0, -15}, {15, 0, -15}, {15, 0, 15}, {-15, 0, 15}};
    p.flatten.push_back(makeFlattenPad(square, 1.5, 5.0));

    RenderMesh m = generateTerrain(p, noise);
    for (const Vertex& v : m.vertices) {
        if (std::abs(v.position.x) <= 14.0 && std::abs(v.position.z) <= 14.0)
            CHECK_APPROX(v.position.y, 1.5, 1e-4);   // vertices land on the pad
    }
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

TEST_CASE(terrain_color_grass_dirt_patches_and_rock_slopes) {
    Vec3 grass = terrainColor(0.0, 1.0, -1.0);   // green plain
    Vec3 plain = terrainColor(0.0, 1.0,  1.0);   // plain stays green (no dirt splatter)
    Vec3 rock  = terrainColor(0.0, 0.0,  0.0);   // vertical -> grey rock
    Vec3 peak  = terrainColor(120.0, 1.0, 1.0);  // high flat peak -> snow (bright)
    CHECK(grass.y > grass.x && grass.y > grass.z);   // grass is green-dominant
    CHECK(plain.y > plain.x);                        // the plain is green even at high noise
    // Grass is more green-SATURATED than rock (neutral grey rock can be brighter,
    // so compare green dominance, not the raw green channel).
    CHECK((grass.y - grass.x) > (rock.y - rock.x));
    // Rock is desaturated (channels close together), unlike green grass.
    CHECK(std::fabs(rock.x - rock.y) < 0.1);
    // A high, flat peak reads as snow: bright and near-neutral.
    CHECK(peak.x > 0.7 && peak.y > 0.7 && peak.z > 0.7);
    for (const Vec3& c : {grass, plain, rock, peak}) {
        CHECK(c.x >= 0.0 && c.x <= 1.0);
        CHECK(c.y >= 0.0 && c.y <= 1.0);
        CHECK(c.z >= 0.0 && c.z <= 1.0);
    }
}

TEST_CASE(terrain_bakes_nonwhite_vertex_colors) {
    TerrainParams p;
    p.resolution = 16;
    p.heightScale = 10.0f;
    Noise n(2);
    RenderMesh m = generateTerrain(p, n);
    bool anyTinted = false;
    for (const Vertex& v : m.vertices) {
        if ((v.color - Vec3(1, 1, 1)).lengthSquared() > 1e-6) anyTinted = true;
        CHECK(v.color.x >= 0.0 && v.color.y >= 0.0 && v.color.z >= 0.0);
    }
    CHECK(anyTinted);   // coloration was baked (not left default white)
}

TEST_CASE(terrain_mountain_layer_raises_relief) {
    // The ridged mountain layer only adds positive relief: max height rises,
    // and the flat-field minimum is not pushed below the base terrain.
    TerrainParams base; base.size = 400; base.heightScale = 5; base.octaves = 5;
    TerrainParams mtn = base; mtn.mountainHeight = 60; mtn.mountainScale = 0.004;
    Noise n(3);
    double baseMax = -1e9, mtnMax = -1e9;
    for (int i = 0; i < 60; i++) {
        double x = -200 + i * 6.7, z = 37 + i * 5.0;
        baseMax = std::max(baseMax, terrainHeight(base, n, x, z));
        mtnMax = std::max(mtnMax, terrainHeight(mtn, n, x, z));
    }
    CHECK(mtnMax > baseMax + 20.0);   // mountains add substantial elevation
}

TEST_CASE(terrain_ring_is_a_hollow_annulus) {
    TerrainParams p; p.size = 100; p.heightScale = 8;
    Noise n(1);
    RenderMesh ring = generateTerrainRing(p, n, /*inner=*/50.0f, /*outer=*/100.0f, 20);
    CHECK(!ring.vertices.empty());
    CHECK(ring.indices.size() % 3 == 0);
    // Covers out to the outer extent...
    float maxAbs = 0.0f;
    for (const Vertex& v : ring.vertices)
        maxAbs = std::max(maxAbs, std::max(std::abs((float)v.position.x),
                                           std::abs((float)v.position.z)));
    CHECK(maxAbs > 95.0f);
    // ...with a hole: fewer triangles than a full grid of the same resolution.
    CHECK(ring.indices.size() < 20u * 20u * 6u);
    // No triangle lies entirely inside the inner hole.
    bool allInsideSome = false;
    for (size_t i = 0; i + 2 < ring.indices.size(); i += 3) {
        bool inside = true;
        for (int k = 0; k < 3; k++) {
            const Vec3& q = ring.vertices[ring.indices[i + k]].position;
            if (std::abs(q.x) > 50.0 || std::abs(q.z) > 50.0) inside = false;
        }
        allInsideSome |= inside;
    }
    CHECK(!allInsideSome);
}

TEST_CASE(terrain_lod_rings_double_in_extent) {
    TerrainParams p; p.size = 100;
    Noise n(1);
    std::vector<RenderMesh> rings = generateTerrainLOD(p, n, 3, 24);
    CHECK(rings.size() == 3u);
    float prev = 0.0f;
    for (const RenderMesh& r : rings) {
        float maxAbs = 0.0f;
        for (const Vertex& v : r.vertices)
            maxAbs = std::max(maxAbs, std::max(std::abs((float)v.position.x),
                                               std::abs((float)v.position.z)));
        CHECK(maxAbs > prev * 1.5f);   // each ring extends well beyond the last
        prev = maxAbs;
    }
}

TEST_CASE(erosion_carves_the_heightfield_within_bounds) {
    TerrainParams p; p.size = 200; p.resolution = 96;
    p.heightScale = 10; p.octaves = 6; p.mountainHeight = 40; p.mountainScale = 0.01;
    Noise n(7);
    Heightmap baked = bakeHeightmap(p, n);

    Heightmap eroded = baked;
    ErosionParams ep; ep.droplets = 8000; ep.seed = 9; ep.thermalIterations = 6;
    erode(eroded, ep);

    CHECK(eroded.h.size() == baked.h.size());
    double diff = 0.0, bakedMax = -1e9, erodedMax = -1e9;
    bool finite = true;
    for (size_t i = 0; i < baked.h.size(); i++) {
        diff += std::fabs(eroded.h[i] - baked.h[i]);
        bakedMax = std::max(bakedMax, (double)baked.h[i]);
        erodedMax = std::max(erodedMax, (double)eroded.h[i]);
        if (!std::isfinite(eroded.h[i])) finite = false;
    }
    CHECK(finite);
    CHECK(diff / baked.h.size() > 1e-3);          // it actually changed the field
    CHECK(erodedMax <= bakedMax + 2.0);           // erosion wears down, not up
}

TEST_CASE(terrain_mountain_mask_creates_grassland_regions) {
    // Without a mask, mountains rise everywhere; with one, large areas stay low
    // (grassland) while a range sits elsewhere -> spatial biome variation.
    TerrainParams base; base.size = 600; base.heightScale = 5; base.octaves = 5;
    base.mountainHeight = 120; base.mountainScale = 0.006;
    TerrainParams masked = base;
    masked.mountainMaskScale = 0.003; masked.mountainMaskLo = -0.05f;
    masked.mountainMaskHi = 0.22f;
    Noise n(12);
    int baseLow = 0, maskedLow = 0, total = 0;
    const double grass = 12.0;   // ~ grassland height (hills, no mountains)
    for (int i = 0; i < 3000; i++) {
        double x = -300 + (i * 37 % 600), z = -300 + (i * 53 % 600);
        if (terrainHeight(base, n, x, z) < grass) baseLow++;
        if (terrainHeight(masked, n, x, z) < grass) maskedLow++;
        total++;
    }
    CHECK(maskedLow > baseLow * 2);     // mask opens up grassland
    CHECK(maskedLow > total / 6);       // a meaningful grassland fraction
}

TEST_CASE(terrain_spine_range_rises_and_falls_to_plains) {
    // A spine-driven range: high uplift on the axis, falling to grassland beyond
    // rangeWidth -> a range with neighboring plains.
    TerrainParams p; p.size = 600; p.heightScale = 5; p.octaves = 5;
    p.rangeHeight = 120; p.rangeWidth = 150; p.mountainScale = 0.006;
    p.rangeSpine = sampleRangeSpine(
        {Vec3(-300, 0, 0), Vec3(0, 0, 0), Vec3(300, 0, 0)});  // axis along x at z=0
    Noise n(3);
    double onSpine = -1e9, offSpine = -1e9;
    for (int i = 0; i < 60; i++) {
        double x = -290 + i * 9.8;
        onSpine = std::max(onSpine, terrainHeight(p, n, x, 0.0));      // on the axis
        offSpine = std::max(offSpine, terrainHeight(p, n, x, 290.0));  // far away
    }
    CHECK(onSpine > 50.0);            // range rises on the spine
    CHECK(offSpine < 25.0);           // plains far from it (just hills)
    CHECK(onSpine > offSpine * 2.0);
}

TEST_CASE(branching_range_builds_multi_depth_ridge_network) {
    // Consumer #2 of the Skeleton: a branching ridge network with varied crest
    // heights (main divide tall, spurs lower), driving terrain uplift.
    std::vector<RidgeSegment> ridges =
        buildRangeRidges(60, 38, 0.55f, 0.9f, 4, 130, 0.6f, 0.0f, 3);
    CHECK(ridges.size() > 4u);          // it branched (not a single ridge)
    float hmin = 1e9f, hmax = -1e9f;
    for (const RidgeSegment& s : ridges) {
        hmin = std::min({hmin, s.ha, s.hb});
        hmax = std::max({hmax, s.ha, s.hb});
    }
    CHECK(hmax > hmin * 1.3f);          // depth falloff -> varied heights

    TerrainParams p; p.size = 600; p.heightScale = 4; p.rangeWidth = 120;
    p.mountainScale = 0.01; p.rangeRidges = ridges;
    Noise n(3);
    double maxNear = -1e9, far = -1e9;
    for (int j = 0; j < 60; j++)
        for (int i = 0; i < 60; i++) {
            double x = -150 + i * 5.0, z = -10 + j * 5.0;   // around the network
            maxNear = std::max(maxNear, terrainHeight(p, n, x, z));
        }
    far = terrainHeight(p, n, 290.0, 290.0);   // far corner = plains
    CHECK(maxNear > 40.0);              // the range rises
    CHECK(far < 20.0);                  // plains beyond the network
}

TEST_CASE(terrain_chunks_tile_the_world) {
    TerrainParams p;
    p.heightScale = 5.0f;
    Noise noise(11);
    const int perSide = 4;
    const float chunkSize = 50.0f;
    const int res = 8;
    auto chunks = generateTerrainChunks(p, noise, perSide, chunkSize, res,
                                        /*colliderRadius=*/chunkSize);
    CHECK(chunks.size() == static_cast<size_t>(perSide * perSide));

    const float worldHalf = perSide * chunkSize * 0.5f;
    for (const TerrainChunk& c : chunks) {
        // Grid topology per chunk.
        CHECK(c.mesh.vertices.size() == static_cast<size_t>((res + 1) * (res + 1)));
        CHECK(c.mesh.indices.size() == static_cast<size_t>(res * res * 6));
        // Tight world AABB matches the chunk footprint.
        float expMinX = -worldHalf + c.cx * chunkSize;
        float expMinZ = -worldHalf + c.cz * chunkSize;
        CHECK_APPROX(c.boundsMin.x, expMinX, 1e-3);
        CHECK_APPROX(c.boundsMax.x, expMinX + chunkSize, 1e-3);
        CHECK_APPROX(c.boundsMin.z, expMinZ, 1e-3);
        CHECK_APPROX(c.boundsMax.z, expMinZ + chunkSize, 1e-3);
        CHECK(c.boundsMax.y >= c.boundsMin.y);   // height range non-degenerate
        // Every vertex is inside its chunk's AABB.
        for (const Vertex& v : c.mesh.vertices) {
            CHECK(v.position.x >= c.boundsMin.x - 1e-3 && v.position.x <= c.boundsMax.x + 1e-3);
            CHECK(v.position.y >= c.boundsMin.y - 1e-3 && v.position.y <= c.boundsMax.y + 1e-3);
        }
    }
}

TEST_CASE(terrain_chunks_match_the_height_field) {
    // A chunk's surface must equal the shared height-field query (one source of
    // truth), so chunked terrain is the same surface as the single mesh.
    TerrainParams p;
    p.heightScale = 8.0f;
    Noise noise(5);
    auto chunks = generateTerrainChunks(p, noise, 2, 40.0f, 8, 40.0f);
    for (const TerrainChunk& c : chunks)
        for (const Vertex& v : c.mesh.vertices) {
            double h = terrainHeight(p, noise, v.position.x, v.position.z);
            CHECK_APPROX(v.position.y, h, 1e-9);
        }
}

TEST_CASE(terrain_chunk_borders_are_seamless) {
    // Adjacent chunks share exact edge vertex positions AND normals (analytic
    // normals from a shared eps), so there are no cracks or lighting seams.
    TerrainParams p;
    p.heightScale = 6.0f;
    Noise noise(9);
    const int res = 8;
    const float chunkSize = 40.0f;
    auto chunks = generateTerrainChunks(p, noise, 2, chunkSize, res, chunkSize);
    auto at = [&](int cx, int cz) -> const TerrainChunk& {
        for (const TerrainChunk& c : chunks) if (c.cx == cx && c.cz == cz) return c;
        return chunks[0];
    };
    const TerrainChunk& left = at(0, 0);
    const TerrainChunk& right = at(1, 0);
    const int n = res + 1;
    // Left chunk's +x edge (i = res) vs right chunk's -x edge (i = 0), same z rows.
    for (int j = 0; j < n; j++) {
        const Vertex& a = left.mesh.vertices[j * n + res];
        const Vertex& b = right.mesh.vertices[j * n + 0];
        CHECK_APPROX(a.position.x, b.position.x, 1e-6);
        CHECK_APPROX(a.position.y, b.position.y, 1e-9);
        CHECK_APPROX(a.position.z, b.position.z, 1e-9);
        CHECK_APPROX(a.normal.x, b.normal.x, 1e-9);
        CHECK_APPROX(a.normal.y, b.normal.y, 1e-9);
        CHECK_APPROX(a.normal.z, b.normal.z, 1e-9);
    }
}

TEST_CASE(terrain_chunk_collider_flag_by_radius) {
    TerrainParams p;
    Noise noise(2);
    // 4x4 chunks of 50 units; only chunks whose centre is within 60 units of the
    // origin get a collider (the central four).
    auto chunks = generateTerrainChunks(p, noise, 4, 50.0f, 4, 60.0f);
    int withCollider = 0;
    for (const TerrainChunk& c : chunks) if (c.collider) withCollider++;
    CHECK(withCollider == 4);   // the four chunks straddling the origin
}

