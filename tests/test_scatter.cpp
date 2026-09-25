#include "../src/engine/procgen/grass.h"
#include "../src/engine/procgen/stylized_tree.h"
#include "../src/engine/procgen/ground_cover.h"
#include "../src/engine/procgen/ground_layers.h"
#include "../src/engine/procgen/stylized_rock.h"
#include "../src/engine/procgen/hydrology.h"
#include "test_framework.h"

#include "../src/engine/procgen/scatter.h"
#include "../src/engine/procgen/terrain.h"
#include "../src/engine/procgen/noise.h"
#include <algorithm>
#include <string>
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

TEST_CASE(scatter_min_spacing_keeps_instances_apart) {
    // Footprint spacing: no two accepted placements are closer than minSpacing
    // in XZ, and the constraint thins the result vs the unconstrained scatter.
    ScatterParams sp;
    sp.count = 600; sp.seed = 11; sp.regionSize = 60.0f;
    TerrainParams t = flatTerrain();
    Noise n(1);

    auto dense = scatterOnTerrain(sp, t, n);
    sp.minSpacing = 5.0f;
    auto spaced = scatterOnTerrain(sp, t, n);

    CHECK(spaced.size() < dense.size());   // spacing rejects crowded candidates
    CHECK(!spaced.empty());
    bool ok = true;
    for (size_t i = 0; i < spaced.size(); i++)
        for (size_t j = i + 1; j < spaced.size(); j++) {
            double dx = spaced[i].position.x - spaced[j].position.x;
            double dz = spaced[i].position.z - spaced[j].position.z;
            if (dx * dx + dz * dz < 5.0 * 5.0 - 1e-6) ok = false;
        }
    CHECK(ok);
}

TEST_CASE(scatter_focus_clears_center_and_steps_size_down_outward) {
    // A hero focal point: a clearing around it, and instances scaled bigger near
    // it (stepping down with distance) so the hero reads as the focus.
    ScatterParams sp;
    sp.count = 2000; sp.seed = 3; sp.regionSize = 100.0f;
    sp.minScale = 1.0f; sp.maxScale = 1.0f;        // isolate the focus multiplier
    sp.focusRadius = 40.0f; sp.focusScale = 2.0f; sp.focusClear = 10.0f;
    TerrainParams t = flatTerrain();
    Noise n(1);
    auto p = scatterOnTerrain(sp, t, n);
    CHECK(!p.empty());

    bool clear = true;
    double nearSum = 0, farSum = 0;
    int nearN = 0, farN = 0;
    for (const auto& pl : p) {
        double d = std::sqrt(pl.position.x * pl.position.x +
                             pl.position.z * pl.position.z);
        if (d < 10.0 - 1e-3) clear = false;        // keep-out honored
        if (d < 18.0) { nearSum += pl.scale; nearN++; }
        else if (d > 35.0) { farSum += pl.scale; farN++; }
    }
    CHECK(clear);
    CHECK(nearN > 0 && farN > 0);
    if (nearN && farN) CHECK(nearSum / nearN > farSum / farN);   // bigger near focus
}

TEST_CASE(scatter_clustering_clumps_placements) {
    // The Thomas process clumps candidates around parent points, so the mean
    // nearest-neighbour distance is far smaller than a uniform scatter of the
    // same count — natural groves rather than even spacing.
    auto avgNearest = [](const std::vector<Placement>& p) {
        double sum = 0.0;
        for (size_t i = 0; i < p.size(); i++) {
            double best = 1e30;
            for (size_t j = 0; j < p.size(); j++) {
                if (i == j) continue;
                double dx = p[i].position.x - p[j].position.x;
                double dz = p[i].position.z - p[j].position.z;
                best = std::min(best, dx * dx + dz * dz);
            }
            sum += std::sqrt(best);
        }
        return p.empty() ? 0.0 : sum / p.size();
    };

    ScatterParams sp;
    sp.count = 400; sp.seed = 5; sp.regionSize = 80.0f;
    TerrainParams t = flatTerrain();
    Noise n(1);
    double uniformNN = avgNearest(scatterOnTerrain(sp, t, n));

    sp.clusterCount = 6; sp.clusterRadius = 4.0f;
    auto clustered = scatterOnTerrain(sp, t, n);
    double clusterNN = avgNearest(clustered);

    CHECK(!clustered.empty());
    CHECK(clusterNN < 0.6 * uniformNN);   // clumped => neighbours much closer
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

// The grass clump (procgen/grass.h): three triangles a blade, every normal straight up (the
// soft carpet look), root colour at the ground and tip colour at the top, same seed same clump.
TEST_CASE(grass_clump_is_blades_lit_as_a_carpet) {
    engine::GrassClumpParams p;
    p.blades = 12;
    const engine::RenderMesh a = engine::grassClump(5, p), b = engine::grassClump(5, p), c = engine::grassClump(6, p);
    CHECK(a.vertices.size() == 12u * 5u && a.indices.size() == 12u * 9u);
    bool up = true, same = a.vertices.size() == b.vertices.size(), differs = false;
    double top = 0.0;
    for (std::size_t i = 0; i < a.vertices.size(); ++i) {
        const engine::Vertex& v = a.vertices[i];
        if (v.normal.y < 0.999) up = false;
        if (v.position.y < 1e-9 && (v.color - p.rootColor).length() > 1e-9) up = false;
        top = std::max(top, static_cast<double>(v.position.y));
        if (same && (v.position - b.vertices[i].position).length() > 1e-12) same = false;
        if (i < c.vertices.size() && (v.position - c.vertices[i].position).length() > 1e-6) differs = true;
    }
    CHECK(up && same && differs);
    CHECK(top > p.height * 0.6 && top < p.height * 1.4);
}

// Stylized trees (procgen/stylized_tree.h): every shape grows bark and canopy within the
// polygon budget, stands on its base, reaches about its height, and is the same for a seed.
TEST_CASE(stylized_trees_stay_in_budget_and_repeat) {
    for (const char* name : {"round", "spreading", "columnar", "flowering", "pine", "palm", "oak", "maple", "birch",
                             "shrub", "flowering_shrub"}) {
        engine::StylizedTreeParams p;
        CHECK(engine::stylizedShapeFromName(name, p.shape));
        p.height = 8.0;
        const engine::StylizedTree a = engine::stylizedTree(11, p), b = engine::stylizedTree(11, p);
        const std::size_t tris = (a.bark.indices.size() + a.canopy.indices.size()) / 3;
        double lo = 1e9, hi = -1e9;
        for (const auto* m : {&a.bark, &a.canopy})
            for (const engine::Vertex& v : m->vertices) { lo = std::min(lo, (double)v.position.y); hi = std::max(hi, (double)v.position.y); }
        const bool shrub = std::string(name).find("shrub") != std::string::npos;   // no trunk, and low
        const bool ok = (shrub || !a.bark.indices.empty()) && !a.canopy.indices.empty() && tris < 2500 &&
                        lo > -0.5 && lo < 0.5 && hi > 8.0 * (shrub ? 0.3 : 0.6) && hi < 8.0 * 1.45 &&
                        a.canopy.vertices.size() == b.canopy.vertices.size() &&
                        (a.canopy.vertices.back().position - b.canopy.vertices.back().position).length() < 1e-12;
        if (!ok) std::printf("    %s: %zu tris, y %.2f..%.2f\n", name, tris, lo, hi);
        CHECK(ok);
    }
    engine::StylizedShape s;
    CHECK(!engine::stylizedShapeFromName("baobab", s));
}

// The ground-cover map (procgen/ground_cover.h): weights sum to one; the sea floor is sea, the
// strip just above it beach and sandy, flat lowland mostly grass, a steep face rock, high
// ground mountain; and the same point always answers the same.
TEST_CASE(ground_cover_bands_the_land_by_height_and_slope) {
    engine::GroundCoverParams p;
    p.seaLevel = 0.0; p.beachHeight = 2.5; p.uplandHeight = 30; p.mountainHeight = 60; p.dirtPatches = 0.0;
    const engine::GroundCover gc(p);
    auto sum = [](const engine::Cover& c) { return c.grass + c.dirt + c.sand + c.rock + c.snow; };
    int sea = 0, beachSand = 0, lowGrass = 0, steepRock = 0, mountain = 0, n = 0;
    for (int i = 0; i < 40; ++i) {
        const double x = i * 17.3, z = i * -9.1;
        const engine::Cover a = gc.at(x, z, -5.0, 1.0), b = gc.at(x, z, 0.8, 1.0), c = gc.at(x, z, 12.0, 1.0),
                            d = gc.at(x, z, 12.0, std::cos(55.0 * 3.14159265 / 180.0)), e = gc.at(x, z, 90.0, 1.0);
        for (const auto* q : {&a, &b, &c, &d, &e}) CHECK(std::fabs(sum(*q) - 1.0) < 1e-9);
        sea += a.biome == engine::Biome::Sea;
        beachSand += b.biome == engine::Biome::Beach && b.sand > 0.5;
        lowGrass += c.biome == engine::Biome::Lowland && c.grass > 0.9;
        steepRock += d.rock > 0.8;
        mountain += e.biome == engine::Biome::Mountain;
        ++n;
    }
    CHECK(sea == n && beachSand == n && lowGrass >= n - 2 && steepRock == n && mountain == n);
    const engine::Cover r1 = gc.at(3.3, 4.4, 10.0, 0.95), r2 = gc.at(3.3, 4.4, 10.0, 0.95);
    CHECK(r1.grass == r2.grass && (r1.colour - r2.colour).length() == 0.0);
    engine::Biome b;
    CHECK(engine::biomeFromName("mountain", b) && b == engine::Biome::Mountain && !engine::biomeFromName("tundra", b));
}

// Terrain layer textures (procgen/ground_layers.h) TILE: the wrap from the last column (row)
// to the first is no rougher than the roughest pair of neighbouring columns (rows) inside the
// tile -- a seam would stand out above all of them -- and a seed repeats.
TEST_CASE(ground_layer_textures_tile_without_a_seam) {
    for (int L = 0; L < 4; ++L) {
        const auto layer = static_cast<engine::GroundLayer>(L);
        const engine::TextureData a = engine::groundLayerTexture(layer, engine::Vec3(0.1, 0.1, 0.1), 64, 3);
        const engine::TextureData b = engine::groundLayerTexture(layer, engine::Vec3(0.1, 0.1, 0.1), 64, 3);
        CHECK(a.width == 64 && a.pixels.size() == 64u * 64u * 4u && a.pixels == b.pixels);
        auto px = [&](int x, int y, int c) { return static_cast<int>(a.pixels[(static_cast<std::size_t>(y) * 64 + x) * 4 + c]); };
        auto colPair = [&](int x0, int x1) { double d = 0; for (int y = 0; y < 64; ++y) for (int c = 0; c < 4; ++c) d += std::abs(px(x0, y, c) - px(x1, y, c)); return d; };
        auto rowPair = [&](int y0, int y1) { double d = 0; for (int x = 0; x < 64; ++x) for (int c = 0; c < 4; ++c) d += std::abs(px(x, y0, c) - px(x, y1, c)); return d; };
        double worstCol = 0, worstRow = 0;
        for (int i = 0; i + 1 < 64; ++i) { worstCol = std::max(worstCol, colPair(i, i + 1)); worstRow = std::max(worstRow, rowPair(i, i + 1)); }
        const double sx = colPair(63, 0), sy = rowPair(63, 0);
        if (sx > worstCol || sy > worstRow) std::printf("    layer %d: x seam %.0f (worst inner %.0f), y seam %.0f (worst inner %.0f)\n", L, sx, worstCol, sy, worstRow);
        CHECK(sx <= worstCol && sy <= worstRow);
    }
}

// The rock library (procgen/stylized_rock.h): every family and stone grows a closed-ish, small
// mesh standing on y = 0, the same for a seed; unknown names are refused.
TEST_CASE(stylized_rocks_every_family_and_stone) {
    for (const char* fam : {"boulder", "slab", "pebbles", "outcrop"})
        for (const char* st : {"granite", "sandstone", "basalt", "mossy"}) {
            engine::StylizedRockParams p;
            CHECK(engine::rockFamilyFromName(fam, p.family) && engine::rockMaterialFromName(st, p.material));
            p.size = 2.0;
            const engine::RenderMesh a = engine::stylizedRock(9, p), b = engine::stylizedRock(9, p);
            double lo = 1e9, hi = -1e9;
            for (const engine::Vertex& v : a.vertices) { lo = std::min(lo, (double)v.position.y); hi = std::max(hi, (double)v.position.y); }
            const bool ok = !a.indices.empty() && a.indices.size() / 3 < 1500 && lo > -0.5 && lo < 0.3 && hi > 0.1 && hi < 4.0 &&
                            a.vertices.size() == b.vertices.size();
            if (!ok) std::printf("    %s/%s: %zu tris, y %.2f..%.2f\n", fam, st, a.indices.size() / 3, lo, hi);
            CHECK(ok);
        }
    engine::RockFamily f;
    CHECK(!engine::rockFamilyFromName("pumice", f));
}

// Hydrology (procgen/hydrology.h): on a slope falling to the sea, water drains into rivers that
// reach it; every river's level only falls downstream and never stands above the ground at its
// centre; the carve only ever lowers the ground, and lowers it inside a channel.
TEST_CASE(hydrology_rivers_run_downhill_to_the_sea) {
    auto ground = [](double x, double z) {   // a tilted, rumpled plain: high in the west, sea in the east
        return -0.04 * x + 6.0 * std::sin(z * 0.004) + 4.0 * std::sin(x * 0.006 + z * 0.003);
    };
    engine::HydroParams hp;
    hp.half = 600; hp.cell = 8; hp.seaLevel = 0; hp.riverArea = 60000;
    auto hy = engine::Hydrology::build(ground, hp);
    CHECK(!hy->rivers().empty());
    int toSea = 0;
    for (const auto& r : hy->rivers()) {
        for (std::size_t k = 1; k < r.nodes.size(); ++k) CHECK(r.nodes[k].level <= r.nodes[k - 1].level + 1e-9);
        for (const auto& nd : r.nodes) CHECK(nd.level <= ground(nd.p.x, nd.p.y) - 0.25 + 1e-9 || nd.level <= hp.seaLevel + 1e-9);
        if (r.mouth && r.nodes.back().level <= hp.seaLevel + 1e-6) ++toSea;
    }
    CHECK(toSea > 0);
    const auto& mid = hy->rivers().front().nodes[hy->rivers().front().nodes.size() / 2];
    const double h = ground(mid.p.x, mid.p.y);
    CHECK(hy->carve(mid.p.x, mid.p.y, h) < h - 0.5);            // cut into a channel at the river
    CHECK(hy->carve(-590, -590, ground(-590, -590)) <= ground(-590, -590));   // never raised anywhere
}
