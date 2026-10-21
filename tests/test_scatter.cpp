#include "../src/engine/procgen/grass.h"
#include "../src/engine/procgen/stylized_tree.h"
#include "../src/engine/procgen/real_tree.h"
#include "../src/engine/procgen/trails.h"
#include "../src/engine/procgen/forest.h"
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
    // RT_GROUND_LAYER_DUMP=dir: each layer at 512^2 in the default palette, raw RGBA (rgb gamma-encoded, a the height)
    if (const char* dir = std::getenv("RT_GROUND_LAYER_DUMP")) {
        const engine::GroundCoverParams gp;
        const engine::Vec3 base[4] = {gp.grass, gp.dirt, gp.sand, gp.rock};
        for (int L = 0; L < 7; ++L) {
            const engine::TextureData t = engine::groundLayerTexture(static_cast<engine::GroundLayer>(L), base[std::min(L, 3)], 512, 3);
            if (std::FILE* f = std::fopen((std::string(dir) + "/layer" + std::to_string(L) + ".rgba").c_str(), "wb")) {
                std::fwrite(t.pixels.data(), 1, t.pixels.size(), f);
                std::fclose(f);
            }
        }
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

// Real trees (procgen/real_tree.h, ADR-0129/0130): every species grows bark and foliage within a
// near-model budget, the same seed grows the same tree, and its impostor covers a real share of the
// picture (a crown, not a stick).
TEST_CASE(real_trees_stay_in_budget_repeat_and_fill_their_impostor) {
    for (int s = 0; s < static_cast<int>(engine::RealSpecies::Count); ++s) {
        const auto sp = static_cast<engine::RealSpecies>(s);
        const engine::RealTree a = engine::realTree(sp, 1234u, 0.0), b = engine::realTree(sp, 1234u, 0.0);
        CHECK(!a.bark.indices.empty());
        CHECK(!a.foliage.indices.empty());
        CHECK(a.bark.indices.size() / 3 + a.foliage.indices.size() / 3 < 16000);   // a near model, instanced
        CHECK(a.bark.vertices.size() == b.bark.vertices.size());
        CHECK(a.foliage.vertices.size() == b.foliage.vertices.size());
        CHECK(a.height > 1.5 && a.crownRadius > 0.3);
        const engine::TextureData fol = engine::realFoliageTexture(sp, 128, 7u);
        std::vector<uint8_t> img(64 * 128 * 4);
        engine::renderImpostor(a, fol, false, 64, 128, 3.0, img.data(), 64 * 4);
        int covered = 0;
        for (std::size_t i = 3; i < img.size(); i += 4) covered += img[i] > 127;
        CHECK(covered > 64 * 128 / 12);
    }
}

// The street trees (real_tree.h, Glenn 2026-10-10: "a variety of trees for the city"): each grows to its kind -- the
// palms a tall bare column under a head of fronds (no foliage low down, the crown about a frond's length across),
// the rain tree wider than it is tall, the frangipani a small tree, the flowering ones partly in their bloom colour
// (redder for the flame tree, bluer for the jacaranda) -- and the forest species are untouched by them.
TEST_CASE(street_tree_species_grow_to_their_kind) {
    using engine::RealSpecies;
    auto lowestLeaf = [](const engine::RealTree& t) {
        double lo = 1e30;
        for (const auto& v : t.foliage.vertices) lo = std::min(lo, static_cast<double>(v.position.y));
        return lo;
    };
    for (uint32_t seed : {11u, 222u, 3333u}) {
        for (RealSpecies sp : {RealSpecies::RoyalPalm, RealSpecies::CoconutPalm}) {
            const engine::RealTree t = engine::realTree(sp, seed, 0.0);
            CHECK(engine::realSpeciesIsPalm(sp));
            CHECK(t.height > 9.0 && t.height < 27.0);
            CHECK(lowestLeaf(t) > 0.45 * t.height);                 // a bare column under the head (old fronds hang)
            CHECK(t.crownRadius > 2.5 && t.crownRadius < 9.0);      // fronds of 3.4-5.4 m
            CHECK(t.trunkRadius > 0.15 && t.trunkRadius < 0.6);
        }
        const engine::RealTree mp = engine::realTree(RealSpecies::Monkeypod, seed, 0.0);
        CHECK(2.0 * mp.crownRadius > mp.height);                     // an umbrella wider than the tree is tall
        const engine::RealTree pl = engine::realTree(RealSpecies::Plumeria, seed, 0.0);
        CHECK(pl.height > 3.0 && pl.height < 9.5);
        CHECK(!engine::realSpeciesIsPalm(RealSpecies::Plumeria) && !engine::realSpeciesIsPalm(RealSpecies::Oak));
    }
    // in flower: some leaf cards wear the bloom colour
    auto bloomShare = [](RealSpecies sp, bool red) {
        const engine::RealTree t = engine::realTree(sp, 7u, 0.0);
        int n = 0, bloom = 0;
        for (std::size_t i = 0; i < t.foliage.vertices.size(); i += 4, ++n) {
            const engine::Vec3 c = t.foliage.vertices[i].color;
            bloom += red ? (c.x > 2.0 * c.y) : (c.z > c.y);
        }
        return n ? double(bloom) / n : 0.0;
    };
    CHECK(bloomShare(RealSpecies::Poinciana, true) > 0.2);
    CHECK(bloomShare(RealSpecies::Jacaranda, false) > 0.3);
    CHECK(bloomShare(RealSpecies::Oak, true) == 0.0);
    // names round-trip
    for (RealSpecies sp : {RealSpecies::RoyalPalm, RealSpecies::CoconutPalm, RealSpecies::Monkeypod, RealSpecies::Poinciana,
                           RealSpecies::Jacaranda, RealSpecies::Plumeria}) {
        RealSpecies back = RealSpecies::Spruce;
        CHECK(engine::realSpeciesFromName(engine::realSpeciesName(sp), back) && back == sp);
    }
}

// Trails (procgen/trails.h, ADR-0134): the distance to a path is the distance to its nearest segment,
// capped, from any side.
TEST_CASE(trail_network_measures_distance_to_the_nearest_path) {
    const engine::TrailNetwork t({{engine::Vec2(0, 0), engine::Vec2(100, 0), engine::Vec2(100, 100)}});
    CHECK(std::abs(t.distance(50, 3, 10) - 3.0) < 1e-9);
    CHECK(std::abs(t.distance(104, 50, 10) - 4.0) < 1e-9);
    CHECK(std::abs(t.distance(-2, 0, 10) - 2.0) < 1e-9);
    CHECK(t.distance(50, 40, 10) == 10.0);
    const engine::TrailNetwork e({});
    CHECK(e.empty() && e.distance(0, 0, 5) == 5.0);
}

// #54 (review): a rock is seated on its base plane, tilted with the ground -- on an even 35-degree slope
// exactly `bed` below the ground at its centre (not sunk by the downhill drop), and on bumpy ground nowhere
// floating over the footprint.
TEST_CASE(rock_seat_follows_the_slope_without_burying_or_floating) {
    const double g = std::tan(35.0 * 3.14159265 / 180.0);
    auto plane = [g](double x, double) { return 100.0 + g * x; };
    const engine::Seat s = engine::seatOnGround(plane, 10.0, 5.0, 1.5, 0.2);
    CHECK(std::abs(s.baseY - (plane(10.0, 5.0) - 0.2)) < 1e-6);
    CHECK(std::abs(s.normal.y - std::cos(35.0 * 3.14159265 / 180.0)) < 1e-3);
    auto bumpy = [g](double x, double z) { return 100.0 + g * x + 0.3 * std::sin(x * 2.1) * std::cos(z * 1.7); };
    const engine::Seat b = engine::seatOnGround(bumpy, 3.0, -2.0, 1.5, 0.1);
    for (int k = 0; k < 8; ++k) {   // the base plane is at or below the ground at every footprint sample
        const double a = k * 0.785398, dx = 1.5 * std::cos(a), dz = 1.5 * std::sin(a);
        const double planeY = b.baseY - (b.normal.x * dx + b.normal.z * dz) / b.normal.y;
        CHECK(planeY <= bumpy(3.0 + dx, -2.0 + dz) + 1e-9);
    }
}


// #57 (Glenn: "There's a wall along one of the rivers ... the wall has no thickness"). The quay is a wall with
// a section now: the face toward the water, a coping on top, a back face into the bank, caps where a run ends.
// Every triangle is wound to face along its own normal (drawn from the side it faces, culled from behind).
TEST_CASE(quay_wall_has_a_top_and_a_back_and_faces_the_way_it_is_wound) {
    auto ground = [](double x, double z) {
        return -0.04 * x + 6.0 * std::sin(z * 0.004) + 4.0 * std::sin(x * 0.006 + z * 0.003);
    };
    engine::HydroParams hp;
    hp.half = 600; hp.cell = 8; hp.seaLevel = 0; hp.riverArea = 60000;
    auto hy = engine::Hydrology::build(ground, hp);
    engine::RenderMesh coping;
    const engine::RenderMesh q = hy->quayMesh([](double, double) { return true; }, ground, 0.8, &coping);
    CHECK(!q.indices.empty());
    CHECK(!coping.indices.empty());
    // RECTILINEAR (Glenn: "brick texture but then sloped tops ... It should be rectilinear"): every top face is level
    // -- the coping's top and the wall's own top are horizontal, whatever the bank does
    int slopedTops = 0, copeTops = 0;
    for (const engine::RenderMesh* m : {&q, static_cast<const engine::RenderMesh*>(&coping)})
        for (std::size_t i = 0; i + 2 < m->indices.size(); i += 3) {
            const engine::Vertex& A = m->vertices[m->indices[i]];
            if (A.normal.y < 0.9) continue;
            const double y0 = A.position.y, y1 = m->vertices[m->indices[i + 1]].position.y, y2 = m->vertices[m->indices[i + 2]].position.y;
            if (std::fabs(y0 - y1) > 1e-6 || std::fabs(y0 - y2) > 1e-6) ++slopedTops;
            if (m == &coping) ++copeTops;
        }
    std::printf("    quay: %d coping tops, %d sloped tops\n", copeTops, slopedTops);
    CHECK(copeTops > 0);
    CHECK(slopedTops == 0);
    int up = 0, front = 0, back = 0, misWound = 0, tris = 0;
    for (std::size_t i = 0; i + 2 < q.indices.size(); i += 3) {
        const engine::Vertex& A = q.vertices[q.indices[i]];
        const engine::Vertex& B = q.vertices[q.indices[i + 1]];
        const engine::Vertex& C = q.vertices[q.indices[i + 2]];
        // the ENGINE's front: cross(c - a, b - a) along the normal (clockwise seen from the front; mesh_builder.h). This
        // test once counted the counter-clockwise rule, and the quay was wound to match it -- every face inside out
        // to the renderer, until the quay moved onto MeshBuilder::quadFacing.
        const engine::Vec3 w = engine::cross(C.position - A.position, B.position - A.position);
        if (w.length() < 1e-9) continue;
        ++tris;
        if (engine::dot(w, A.normal) <= 0) ++misWound;
        if (A.normal.y > 0.9) ++up;
        else if (std::fabs(A.normal.y) < 0.1) {
            // toward the water or the bank: does a step along the normal bring the face nearer the river?
            const engine::Vec3 c = (A.position + B.position + C.position) * (1.0 / 3.0);
            const double here = hy->distanceToRiver(c.x, c.z, 60.0);
            const double ahead = hy->distanceToRiver(c.x + A.normal.x * 0.3, c.z + A.normal.z * 0.3, 60.0);
            ++(ahead < here ? front : back);
        }
    }
    std::printf("    quay: %d triangles, %d coping, %d toward the water, %d toward the bank (end caps among them), %d mis-wound\n",
                tris, up, front, back, misWound);
    CHECK(misWound == 0);
    CHECK(up > 0);              // a top
    CHECK(front > 0 && back >= front * 3 / 4);   // a back face behind the front (caps land on either side)
}

// #93 (Glenn: "The tree cards aren't lit well. I can tell they're the cards. ... Normal maps?"): the impostor's
// normal picture carries the crown's shape in the card's frame -- its left half faces left, its right half
// right, its top up -- so the far card lights like the tree, not like a sheet.
TEST_CASE(impostor_normals_carry_the_crown_shape) {
    const RealTree tree = realTree(RealSpecies::Oak, 7, 12.0);
    const TextureData fol = realFoliageTexture(RealSpecies::Oak, 128, 7);
    const int w = 96, h = 192;
    std::vector<uint8_t> col(static_cast<std::size_t>(w) * h * 4), nrm(col.size());
    renderImpostor(tree, fol, false, w, h, 3.0, col.data(), w * 4, nrm.data());
    double leftX = 0, rightX = 0, topY = 0, lowY = 0;
    int nl = 0, nr = 0, nt = 0, nb = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 4;
            if (col[i + 3] < 128) continue;   // the silhouette only
            const double nx = nrm[i] / 255.0 * 2 - 1, ny = nrm[i + 1] / 255.0 * 2 - 1, nz = nrm[i + 2] / 255.0 * 2 - 1;
            CHECK(nz > 0.15);                                  // never facing into the card
            if (x < w / 2 - 8) { leftX += nx; ++nl; }
            if (x > w / 2 + 8) { rightX += nx; ++nr; }
            if (y < h / 5) { topY += ny; ++nt; }
            if (y > h / 3 && y < h / 2) { lowY += ny; ++nb; }
        }
    CHECK(nl > 50 && nr > 50 && nt > 20 && nb > 20);
    leftX /= std::max(1, nl); rightX /= std::max(1, nr); topY /= std::max(1, nt); lowY /= std::max(1, nb);
    std::printf("    [impostor normals] left x %.2f, right x %.2f; crown top y %.2f, lower crown y %.2f\n", leftX, rightX, topY, lowY);
    CHECK(leftX < -0.1 && rightX > 0.1);   // the crown's sides face outward
    CHECK(topY > lowY);                    // its top faces up more than its underside
    // no normal picture asked for: the colour is exactly as before
    std::vector<uint8_t> col2(col.size());
    renderImpostor(tree, fol, false, w, h, 3.0, col2.data(), w * 4);
    CHECK(col2 == col);
}

// MANY-VIEW IMPOSTORS (Glenn, 2026-10-18: "So can we make multi angle imposter cards"): every view's window holds
// the whole tree; the view from behind is the view from the front mirrored (the frames agree with mesh.frag's);
// a far tree is one card carrying its centre, window and yaw, and its shadow two crossed cards on its views.
TEST_CASE(impostor_views_frame_the_tree_from_every_side) {
    const RealTree tree = realTree(RealSpecies::Oak, 7, 12.0);
    const TextureData fol = realFoliageTexture(RealSpecies::Oak, 128, 7);
    const double cy = 0.5 * tree.height, maxEl = (kImpostorElevations - 1) * kImpostorElevationStep;
    double rb = 0.1;
    for (const RenderMesh* m : {&tree.bark, &tree.foliage})
        for (const Vertex& v : m->vertices) rb = std::max(rb, std::hypot(v.position.x, v.position.z));
    const double hh = impostorViewHalfHeight(tree, cy, maxEl);
    CHECK(hh >= 0.5 * tree.height - 1e-6);
    int outside = 0;
    for (int e = 0; e < kImpostorElevations; ++e)
        for (int a = 0; a < kImpostorAzimuths; ++a) {
            const double az = a * 2.0 * 3.14159265358979 / kImpostorAzimuths, el = e * kImpostorElevationStep;
            const Vec3 d(std::sin(az) * std::cos(el), std::sin(el), std::cos(az) * std::cos(el)), r(std::cos(az), 0, -std::sin(az));
            const Vec3 u = cross(d, r);
            for (const RenderMesh* m : {&tree.bark, &tree.foliage})
                for (const Vertex& v : m->vertices) {
                    const Vec3 p = v.position - Vec3(0, cy, 0);
                    outside += std::fabs(dot(p, r)) > rb + 1e-6 || std::fabs(dot(p, u)) > hh + 1e-6;
                }
        }
    CHECK(outside == 0);
    const int w = 64, h = 128;
    std::vector<uint8_t> nrm[2];
    auto view = [&](double az, std::vector<uint8_t>& n) {
        std::vector<uint8_t> col(static_cast<std::size_t>(w) * h * 4);
        n.assign(col.size(), 0);
        renderImpostorView(tree, fol, az, 0.0, cy, rb, hh, w, h, 3.0, col.data(), w * 4, n.data());
        return col;
    };
    auto view2 = [&](double az, double el, std::vector<uint8_t>& n) {
        std::vector<uint8_t> col(static_cast<std::size_t>(w) * h * 4);
        n.assign(col.size(), 0);
        renderImpostorView(tree, fol, az, el, cy, rb, hh, w, h, 3.0, col.data(), w * 4, n.data());
        return col;
    };
    const std::vector<uint8_t> front = view(0.0, nrm[0]), back = view(3.14159265358979, nrm[1]);
    int cover = 0, both = 0;
    double thick = 0.0;   // (the normal map's alpha is the nearest part's depth: front and back, the crown between)
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 4 + 3, j = (static_cast<std::size_t>(y) * w + (w - 1 - x)) * 4 + 3;
            const bool f = front[i] >= 128;
            const bool b = back[j] >= 128;
            cover += f;
            both += f && b;
            if (f && b) thick += ((nrm[0][i] + nrm[1][j]) * 2.0 / 255.0 - 2.0) * (rb + hh);
        }
    thick /= std::max(1, both);
    // ...and the views as the crown's shadow map (mesh.frag impostorSelfShadow): the tree's nearest part under each
    // pixel seen from the front, looked up in the view toward the sun -- against a sun 35 deg up only the crown's top
    // catches it (about a third of what the viewer sees), with the sun behind the viewer nearly all of it
    auto selfLit = [&](double sunAz, double sunEl) {
        const double el = sunEl;
        const Vec3 d(std::sin(sunAz) * std::cos(el), std::sin(el), std::cos(sunAz) * std::cos(el)), r(std::cos(sunAz), 0, -std::sin(sunAz));
        const Vec3 u = cross(d, r);
        std::vector<uint8_t> sn;
        const std::vector<uint8_t> sc = view2(sunAz, sunEl, sn);
        const double D = rb + hh, fall = 0.15 * rb + 0.5, bias = 0.25 + 0.05 * rb;
        double lit = 0.0;
        int n = 0;
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 4 + 3;
                if (front[i] < 128) continue;
                const Vec3 p(((x + 0.5) / w * 2 - 1) * rb, (1 - (y + 0.5) / h * 2) * hh, (nrm[0][i] * 2.0 / 255.0 - 1.0) * D);
                const double sx = dot(p, r) / rb, sy = dot(p, u) / hh;
                double l = 1.0;
                if (std::fabs(sx) < 1 && std::fabs(sy) < 1) {
                    const int px = std::clamp(static_cast<int>((sx * 0.5 + 0.5) * w), 0, w - 1), py = std::clamp(static_cast<int>((0.5 - sy * 0.5) * h), 0, h - 1);
                    const std::size_t j = (static_cast<std::size_t>(py) * w + px) * 4 + 3;
                    const double under = (sn[j] * 2.0 / 255.0 - 1.0) * D - dot(p, d);
                    if (sc[j] >= 128) l = std::exp(-std::max(under - bias, 0.0) / fall);
                }
                lit += l;
                ++n;
            }
        return lit / std::max(1, n);
    };
    const double against = selfLit(3.14159265358979, kImpostorElevationStep), behind = selfLit(0.0, kImpostorElevationStep);
    std::printf("    [impostor views] window %.1f x %.1f m; front covers %d px, %d of them in the back view mirrored, "
                "%.2f m thick there; %.0f%% of it sunlit against the sun, %.0f%% with it behind\n",
                2 * rb, 2 * hh, cover, both, thick, 100 * against, 100 * behind);
    CHECK(cover > 300);
    CHECK(both > cover * 8 / 10);
    CHECK(thick > 0.3 * rb);
    CHECK(against < 0.65);
    CHECK(behind > 0.8);
    CHECK(behind - against > 0.25);
    // the far card: one quad whose corners give back the tree's centre; its shadow, two quads inside its block
    ImpostorSlot sl;
    sl.vu0 = 0.25; sl.vv0 = 0.5; sl.cellU = 0.01; sl.cellV = 0.02; sl.Rb = rb; sl.Hh = hh; sl.centreY = cy;
    ForestTree t;
    t.pos = Vec3(10, 2, -5); t.yaw = 0.7f; t.scale = 1.3f;
    RenderMesh card, shadow;
    appendImpostorViews(card, t, sl, 0.2);
    appendImpostorShadow(shadow, t, sl);
    CHECK(card.tangentIsData && card.vertices.size() == 4 && card.indices.size() == 6);
    for (const Vertex& v : card.vertices) {
        const Vec3 c = v.position - Vec3(v.u * v.tangent.x, v.v * v.tangent.y, v.u * v.tangent.x);
        CHECK((c - (t.pos + Vec3(0, (cy - 0.15) * t.scale, 0))).length() < 1e-4);
        CHECK(std::fabs(v.tangent.z - t.yaw) < 1e-6 && std::fabs(v.normal.x - 0.25) < 1e-9);
    }
    CHECK(shadow.vertices.size() == 8 && shadow.indices.size() == 12);
    for (const Vertex& v : shadow.vertices)
        CHECK(v.u >= 0.25 - 1e-6 && v.u <= 0.25 + kImpostorAzimuths * 0.01 + 1e-6 && v.v >= 0.5 - 1e-6 && v.v <= 0.52 + 1e-6);
}

// A FOREST REGION (Glenn, 2026-10-19: "one of the forests near the mountains should be spruce/pine trees"): on flat
// ground at 100 m, below the spruce's own band, a region naming spruce makes its middle a spruce wood, and outside it
// the island's mix is as before (no spruce at all at this height)
TEST_CASE(a_forest_region_grows_its_own_mix_below_the_species_band) {
    const nlohmann::json j = nlohmann::json::parse(R"({
        "seed": 3, "spacing": 7.5, "coverage": 1.0,
        "species": [{"kind": "oak", "altLo": 0, "altHi": 200}, {"kind": "spruce", "altLo": 220, "altHi": 720}],
        "regions": [{"name": "pinewood", "at": [0, 0], "radius": 300, "edge": 60, "altLo": 0,
                     "species": {"spruce": 6}, "others": 0.08}]})");
    const ForestParams p = forestFromJson(j);
    CHECK(p.regions.size() == 1 && p.regions[0].named.size() == 2 && !p.regions[0].named[0] && p.regions[0].named[1]);
    CHECK(std::fabs(p.regions[0].weight[0] - 0.08) < 1e-12 && std::fabs(p.regions[0].weight[1] - 6.0) < 1e-12);
    const auto trees = placeForest(p, 700.0, 0.0, nullptr, nullptr, [](double, double) { return 100.0; }, {}, 4);
    int in = 0, inSpruce = 0, out = 0, outSpruce = 0;
    for (const ForestTree& t : trees) {
        const double r = std::hypot(t.pos.x, t.pos.z);
        const bool spruce = t.variant / 4 == 1;   // (4 variants a species)
        if (r < 150) { ++in; inSpruce += spruce; }
        if (r > 500) { ++out; outSpruce += spruce; }
    }
    std::printf("    [forest region] inside %d trees, %d spruce; outside %d trees, %d spruce\n", in, inSpruce, out, outSpruce);
    CHECK(in > 300 && out > 3000);
    CHECK(inSpruce > in * 9 / 10);
    CHECK(outSpruce == 0);
}

// A FOREST MIXED WITH THE RECIPE TREES (Glenn, 2026-10-20: "we should mix the new trees in there as well"): with
// "recipes": 0.4 every tree stands where it stood and stays its kind, about 0.4 of each kind's trees grow as its recipe
// (its twin, listed after the kinds), and the twin is never a kind's tree for itself
TEST_CASE(a_forest_mixed_with_recipe_trees_keeps_its_trees_and_grows_the_share_as_recipes) {
    const char* kForest = R"({
        "seed": 3, "spacing": 7.5, "coverage": 1.0,
        "species": [{"kind": "oak", "altLo": 0, "altHi": 400}, {"kind": "spruce", "altLo": 0, "altHi": 400},
                    {"kind": "shrub", "understory": true}]})";
    nlohmann::json j = nlohmann::json::parse(kForest);
    const ForestParams real = forestFromJson(j);
    j["recipes"] = 0.4;
    const ForestParams mixed = forestFromJson(j);
    CHECK(real.species.size() == 3 && mixed.species.size() == 6);
    for (int s = 0; s < 3; ++s) {
        CHECK(mixed.species[s].recipe.empty() && mixed.species[s].twin == 3 + s);
        CHECK(mixed.species[3 + s].twinOnly && !mixed.species[3 + s].recipe.empty());
        CHECK(mixed.species[3 + s].kind == mixed.species[s].kind);
    }
    CHECK(mixed.species[5].recipe == "hawthorn" && mixed.species[5].understory);
    auto vpsOf = [](const ForestParams& p) { int v = 1; for (const ForestSpecies& s : p.species) v = std::max(v, s.variants); return v; };
    const int vr = vpsOf(real), vm = vpsOf(mixed);
    auto flat = [](double, double) { return 100.0; };
    const auto a = placeForest(real, 700.0, 0.0, nullptr, nullptr, flat, {}, vr);
    const auto b = placeForest(mixed, 700.0, 0.0, nullptr, nullptr, flat, {}, vm);
    CHECK(a.size() == b.size() && a.size() > 3000);
    if (a.size() != b.size()) return;
    int moved = 0, changedKind = 0, n[3] = {}, asRecipe[3] = {};
    for (std::size_t i = 0; i < a.size(); ++i) {
        moved += (a[i].pos - b[i].pos).length() > 1e-9;
        const int ka = a[i].variant / vr, sb = b[i].variant / vm, kb = sb >= 3 ? sb - 3 : sb;
        changedKind += ka != kb;
        ++n[ka];
        asRecipe[ka] += sb >= 3;
    }
    for (int k = 0; k < 3; ++k)
        std::printf("    [forest mix] %s: %d trees, %d as the recipe (%.2f)\n", mixed.species[k].kind.c_str(), n[k], asRecipe[k],
                    n[k] ? double(asRecipe[k]) / n[k] : 0.0);
    CHECK(moved == 0 && changedKind == 0);
    for (int k = 0; k < 3; ++k) {
        CHECK(n[k] > 200);
        CHECK(std::fabs(double(asRecipe[k]) / n[k] - 0.4) < 0.05);
    }
}
