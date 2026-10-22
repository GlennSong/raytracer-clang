#include "forest.h"
#include "../mesh_builder.h"

#include "ground_cover.h"
#include "noise.h"
#include "terrain_maps.h"

#include <algorithm>
#include <cstdlib>
#include <map>
#include <cmath>
#include <memory>
#include <thread>

namespace engine {

namespace {
double smooth(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
uint32_t mix32(uint32_t h) {
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
    return h;
}
double unit(uint32_t h) { return (mix32(h) & 0xFFFFFFu) / 16777216.0; }
}  // namespace

ForestParams forestFromJson(const nlohmann::json& j) {
    ForestParams p;
    p.spacing = j.value("spacing", p.spacing);
    p.coverage = j.value("coverage", p.coverage);
    p.standScaleM = j.value("standScale", p.standScaleM);
    p.treelineM = j.value("treeline", p.treelineM);
    p.shoreClearM = j.value("shoreClear", p.shoreClearM);
    p.maxSlopeDeg = j.value("maxSlopeDeg", p.maxSlopeDeg);
    p.cellM = j.value("cell", p.cellM);
    p.nearM = j.value("near", p.nearM);
    p.nearFadeM = j.value("nearFade", p.nearFadeM);
    p.farM = j.value("far", p.farM);
    p.heroM = j.value("hero", p.heroM);
    p.heroFadeM = j.value("heroFade", p.heroFadeM);
    p.midM = j.value("mid", p.midM);
    p.midFadeM = j.value("midFade", p.midFadeM);
    p.lightM = j.value("light", p.lightM);
    p.lightFadeM = j.value("lightFade", p.lightFadeM);
    p.seed = j.value("seed", p.seed);
    p.litter = j.value("litter", p.litter);
    p.understoryM = j.value("understoryDraw", p.understoryM);
    p.understorySpacing = j.value("understorySpacing", p.understorySpacing);
    p.understoryDensity = j.value("understoryDensity", p.understoryDensity);
    if (j.contains("rocks") && j["rocks"].is_array())
        for (const auto& r : j["rocks"]) {
            RockLayer L;
            L.family = r.value("family", L.family);
            L.stone = r.value("stone", L.stone);
            L.size = r.value("size", L.size);
            L.variants = std::clamp(r.value("variants", L.variants), 1, 16);
            const std::string g = r.value("ground", std::string("scree"));
            L.ground = g == "outcrop" ? 1 : g == "field" ? 2 : g == "forest" ? 3 : g == "beach" ? 4 : g == "shore" ? 5 : 0;
            L.spacing = r.value("spacing", L.spacing);
            L.density = r.value("density", L.density);
            L.moss = r.value("moss", L.moss);
            L.drawM = r.value("draw", L.drawM);
            p.rocks.push_back(L);
        }
    // THE RECIPE TREES IN A real_tree FOREST (Glenn, 2026-10-18: "I guess the next step is to bring these trees to the
    // island"): "recipes": true -- or RT_FOREST_RECIPES=1 (=0 off: the A/B) -- grows each `kind` as its recipe
    // (tree_recipes.lua), at "recipeAge" (default mature: real sizes, ADR-0196; RT_FOREST_RECIPE_AGE overrides): the
    // willow a weeping willow, the shrub a hawthorn; beech, fir, aspen and alder their own recipes (derived from their
    // kin); a kind with no recipe stays real_tree.
    // ...or MIXED (Glenn, 2026-10-20: "we should mix the new trees in there as well"): "recipes": 0.4 (or
    // RT_FOREST_RECIPES=0.4) grows that share of each kind's trees as its recipe and the rest as real_tree. The kind
    // keeps one place in the pick (its band, weight, stands, regions), so the forest stands where it stood; its recipe
    // twin, listed after every kind, takes a tree where the twin's clumps fall (placeForest)
    double recipeShare = 0.0;
    if (j.contains("recipes"))
        recipeShare = j["recipes"].is_boolean() ? (j["recipes"].get<bool>() ? 1.0 : 0.0) : j["recipes"].get<double>();
    if (const char* e = std::getenv("RT_FOREST_RECIPES")) {
        const double v = std::atof(e);
        recipeShare = std::string(e) == "0" ? 0.0 : v > 0.0 && v < 1.0 ? v : 1.0;
    }
    recipeShare = std::clamp(recipeShare, 0.0, 1.0);
    std::string recipeAge = j.value("recipeAge", std::string("mature"));
    if (const char* e = std::getenv("RT_FOREST_RECIPE_AGE")) recipeAge = e;
    // (and in a stand, of mixed ages: "recipeForm" (default stand), "ageMix" { age = weight } (default young 1, mature 2,
    // old 1; RT_FOREST_RECIPE_AGE pins one age instead))
    const std::string recipeForm = j.value("recipeForm", std::string("stand"));
    std::vector<std::pair<std::string, double>> recipeMix;
    if (j.contains("ageMix") && j["ageMix"].is_object()) {
        for (auto it = j["ageMix"].begin(); it != j["ageMix"].end(); ++it) recipeMix.emplace_back(it.key(), it.value().get<double>());
    } else if (!std::getenv("RT_FOREST_RECIPE_AGE") && !j.contains("recipeAge")) {
        recipeMix = {{"young", 1.0}, {"mature", 2.0}, {"old", 1.0}};
    }
    static const std::map<std::string, std::string> kRecipeOf = {
        {"oak", "oak"}, {"maple", "maple"}, {"beech", "beech"}, {"birch", "birch"}, {"aspen", "aspen"},
        {"willow", "weeping_willow"}, {"alder", "alder"}, {"pine", "pine"}, {"spruce", "spruce"}, {"fir", "fir"},
        {"shrub", "hawthorn"}};
    // one level species, grown as its recipe (`asRecipe`, where its kind has one) or as the level wrote it
    auto speciesOf = [&](const nlohmann::json& s, bool asRecipe, ForestSpecies& fs) {
        fs.recipe = s.value("recipe", std::string());
        fs.age = s.value("age", std::string());
        if (asRecipe && fs.recipe.empty()) {
            const auto it = kRecipeOf.find(s.value("kind", std::string("spruce")));
            if (it != kRecipeOf.end()) {
                fs.recipe = it->second;
                if (fs.age.empty() && !s.value("understory", false)) {
                    fs.age = recipeAge;
                    fs.ageMix = recipeMix;
                    fs.form = recipeForm;
                    fs.formByDensity = !recipeForm.empty();
                }
            }
        }
        if (s.contains("form")) fs.form = s.value("form", std::string());
        if (s.contains("ageMix") && s["ageMix"].is_object()) {
            fs.ageMix.clear();
            for (auto it = s["ageMix"].begin(); it != s["ageMix"].end(); ++it) fs.ageMix.emplace_back(it.key(), it.value().get<double>());
        }
        if (fs.recipe.empty() && !realSpeciesFromName(s.value("kind", std::string("spruce")), fs.species)) return false;
        fs.scale = s.value("scale", fs.scale);
        fs.variants = std::clamp(s.value("variants", fs.variants), 1, 16);
        if (fs.formByDensity) fs.variants = std::min(16, 2 * fs.variants);   // (open-grown, then in the stand)
        fs.altLo = s.value("altLo", fs.altLo);
        fs.altHi = s.value("altHi", fs.altHi);
        fs.weight = s.value("weight", fs.weight);
        fs.wet = s.value("wet", fs.wet);
        fs.understory = s.value("understory", fs.understory);
        fs.kind = s.value("kind", fs.recipe);
        return true;
    };
    std::vector<std::pair<std::size_t, ForestSpecies>> twins;   // (the kind's index, its recipe twin)
    if (j.contains("species") && j["species"].is_array())
        for (const auto& s : j["species"]) {
            ForestSpecies fs;
            if (!speciesOf(s, recipeShare >= 1.0, fs)) continue;
            const bool hasRecipe = s.value("recipe", std::string()).empty() && kRecipeOf.count(s.value("kind", std::string("spruce")));
            if (recipeShare > 0.0 && recipeShare < 1.0 && hasRecipe) {
                ForestSpecies tw;
                if (speciesOf(s, true, tw)) {
                    tw.twinOnly = true;
                    fs.twinShare = recipeShare;
                    twins.emplace_back(p.species.size(), std::move(tw));
                }
            }
            p.species.push_back(fs);
        }
    for (auto& [of, tw] : twins) {
        p.species[of].twin = static_cast<int>(p.species.size());
        p.species.push_back(std::move(tw));
    }
    // "regions": [{ "name", "at": [x, z], "radius", "edge", "altLo", "species": { kind: weight }, "others" }]
    if (j.contains("regions") && j["regions"].is_array())
        for (const auto& r : j["regions"]) {
            ForestRegion R;
            R.name = r.value("name", std::string());
            if (r.contains("at") && r["at"].is_array() && r["at"].size() >= 2) { R.cx = r["at"][0].get<double>(); R.cz = r["at"][1].get<double>(); }
            R.radius = r.value("radius", R.radius);
            R.edge = std::max(1.0, r.value("edge", R.edge));
            R.altLo = r.value("altLo", R.altLo);
            const double others = r.value("others", 1.0);
            R.weight.assign(p.species.size(), others);
            R.named.assign(p.species.size(), false);
            if (r.contains("species") && r["species"].is_object())
                for (std::size_t s = 0; s < p.species.size(); ++s)
                    if (r["species"].contains(p.species[s].kind)) {
                        R.weight[s] = r["species"][p.species[s].kind].get<double>();
                        R.named[s] = true;
                    }
            p.regions.push_back(std::move(R));
        }
    return p;
}


namespace {
const Noise& forestNoise(uint32_t seed) {
    static thread_local std::unique_ptr<Noise> nz;
    static thread_local uint32_t nzSeed = 0;
    if (!nz || nzSeed != seed) { nz = std::make_unique<Noise>(seed * 2654435761u + 5u); nzSeed = seed; }
    return *nz;
}
// the part of the density that needs no ground sample: stands and clearings, soil, scree
double forestPrefilter(const ForestParams& p, const TerrainMaps* maps, double x, double z) {
    const Noise& nz = forestNoise(p.seed);
    const double stand = 0.5 + 0.5 * nz.fbm2(x / p.standScaleM + 7.7, z / p.standScaleM - 2.9, 4) + 0.08 * nz.noise2(x * 0.03, z * 0.03);
    const double t0 = 1.0 - p.coverage - 0.1;
    double d = smooth(t0, t0 + 0.2, stand);
    if (d > 0.0 && maps) {
        const TerrainMapSample m = maps->at(x, z);
        d *= smooth(0.12, 0.45, m.soil) * (1.0 - smooth(0.2, 0.55, m.scree));
    }
    return d;
}
// ...and the part that does: the shore, the treeline, the slope, what the ground is
double forestGrounded(const ForestParams& p, double sea, const GroundCover* cover, double x, double z, double y, double slopeDeg) {
    const Noise& nz = forestNoise(p.seed);
    const double alt = y - (sea > -1e29 ? sea : 0.0);
    double d = smooth(p.shoreClearM, p.shoreClearM + 8.0, alt);
    if (d <= 0.0) return 0.0;
    const double tl = p.treelineM + 90.0 * nz.fbm2(x * 0.0017 + 3.3, z * 0.0017 - 1.1, 3);
    d *= 1.0 - smooth(tl - 70.0, tl, alt);
    d *= 1.0 - smooth(p.maxSlopeDeg - 8.0, p.maxSlopeDeg, slopeDeg);
    if (d > 0.0 && cover) {
        const Cover c = cover->at(x, z, y, std::cos(slopeDeg * 3.14159265358979 / 180.0));
        d *= std::max(0.0, 1.0 - 1.6 * (c.rock + c.snow + c.sand));
    }
    return std::clamp(d, 0.0, 1.0);
}
}  // namespace

ForestWhy forestWhy(const ForestParams& p, double sea, const GroundCover* cover, const TerrainMaps* maps, double x,
                    double z, double y, double slopeDeg) {
    ForestWhy w;
    const Noise& nz = forestNoise(p.seed);
    const double stand = 0.5 + 0.5 * nz.fbm2(x / p.standScaleM + 7.7, z / p.standScaleM - 2.9, 4) + 0.08 * nz.noise2(x * 0.03, z * 0.03);
    const double t0 = 1.0 - p.coverage - 0.1;
    w.stand = smooth(t0, t0 + 0.2, stand);
    if (maps) {
        const TerrainMapSample m = maps->at(x, z);
        w.soil = smooth(0.12, 0.45, m.soil);
        w.scree = 1.0 - smooth(0.2, 0.55, m.scree);
    }
    const double alt = y - (sea > -1e29 ? sea : 0.0);
    w.shore = smooth(p.shoreClearM, p.shoreClearM + 8.0, alt);
    const double tl = p.treelineM + 90.0 * nz.fbm2(x * 0.0017 + 3.3, z * 0.0017 - 1.1, 3);
    w.treeline = 1.0 - smooth(tl - 70.0, tl, alt);
    w.slope = 1.0 - smooth(p.maxSlopeDeg - 8.0, p.maxSlopeDeg, slopeDeg);
    if (cover) {
        const Cover c = cover->at(x, z, y, std::cos(slopeDeg * 3.14159265358979 / 180.0));
        w.cover = std::max(0.0, 1.0 - 1.6 * (c.rock + c.snow + c.sand));
    }
    return w;
}

double forestRegionAt(const ForestRegion& R, uint32_t seed, double x, double z) {
    // a ragged edge: the radius wanders +-25% round the centre (a wood is not a disc)
    const Noise& nz = forestNoise(seed);
    const double dx = x - R.cx, dz = z - R.cz, d = std::hypot(dx, dz);
    if (d > 1.25 * R.radius + R.edge) return 0.0;
    const double a = std::atan2(dz, dx);
    const double r = R.radius * (1.0 + 0.25 * nz.fbm2(std::cos(a) * 1.7 + R.cx * 1e-3, std::sin(a) * 1.7 + R.cz * 1e-3, 3));
    return 1.0 - smooth(r - R.edge, r, d);
}

double forestCanopy(const ForestParams& p, double sea, const TerrainMaps* maps, double x, double z, double y,
                    double slopeDeg) {
    const double a = forestPrefilter(p, maps, x, z);
    return a <= 0.0 ? 0.0 : a * forestGrounded(p, sea, nullptr, x, z, y, slopeDeg);
}

std::vector<ForestTree> placeForest(const ForestParams& p, double half, double sea, const GroundCover* cover,
                                    const TerrainMaps* maps, const std::function<double(double, double)>& ground,
                                    const std::function<bool(double, double)>& exclude, int vps) {
    std::vector<ForestTree> out;
    if (p.species.empty()) return out;
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    const Noise sn(p.seed * 7919u + 13u);
    bool anyUnder = false;
    for (const ForestSpecies& fs : p.species) anyUnder = anyUnder || fs.understory;
    // TWO PASSES over jittered grids: the canopy at the stand's spacing, then the understory (bushes
    // under thin canopy and thickest along the edges), each choosing among its own species
    for (int pass = 0; pass < (anyUnder ? 2 : 1); ++pass) {
        const bool under = pass == 1;
        const double spacing = under ? p.understorySpacing : p.spacing;
        const uint32_t salt = under ? 0x5EED5u : 0u;
        const int n = static_cast<int>(std::ceil(2.0 * half / spacing));
        std::vector<std::vector<ForestTree>> parts(T);
        std::vector<std::thread> th;
        for (int t = 0; t < T; ++t)
            th.emplace_back([&, t] {
                for (int j = t; j < n; j += T)
                    for (int i = 0; i < n; ++i) {
                        const uint32_t h = (static_cast<uint32_t>(i) * 73856093u ^ static_cast<uint32_t>(j) * 19349663u ^ p.seed * 83492791u) + salt;
                        const double x = -half + (i + 0.1 + 0.8 * unit(h)) * spacing, z = -half + (j + 0.1 + 0.8 * unit(h ^ 0xA5A5u)) * spacing;
                        // the cheap part first: most of the island is rejected before a ground sample
                        const double roll = unit(h ^ 0x51ED27u);
                        const double pre = forestPrefilter(p, maps, x, z);
                        // the understory takes a share of the canopy's density, most at a half-covered edge
                        auto keep = [&](double d) { return under ? p.understoryDensity * (0.35 + 1.3 * 4.0 * d * (1.0 - d)) * (d > 0.05 ? 1.0 : 0.0) : d; };
                        if (roll >= keep(pre) && (!under || pre <= 0.0)) continue;
                        const double y = ground(x, z);
                        const double e = 1.5;
                        const double gx = (ground(x + e, z) - ground(x - e, z)) / (2 * e), gz = (ground(x, z + e) - ground(x, z - e)) / (2 * e);
                        const double slope = std::atan(std::sqrt(gx * gx + gz * gz)) * 180.0 / 3.14159265358979;
                        const double d = pre * forestGrounded(p, sea, cover, x, z, y, slope);
                        if (roll >= keep(d)) continue;
                        if (exclude && exclude(x, z)) continue;
                        // the species: its altitude band (soft), its weight, its taste for wet ground, and a
                        // patchy field of its own (stands, not a salad)
                        const double alt = y - (sea > -1e29 ? sea : 0.0);
                        const double wet = maps ? maps->at(x, z).wet : 0.0;
                        // ...and the regions it stands in, each with its own mix (ForestRegion)
                        double inRegion[8] = {};
                        const std::size_t nRegions = std::min<std::size_t>(8, p.regions.size());
                        for (std::size_t r = 0; r < nRegions; ++r) inRegion[r] = forestRegionAt(p.regions[r], p.seed, x, z);
                        int best = -1;
                        double bw = -1e9;
                        for (std::size_t s = 0; s < p.species.size(); ++s) {
                            const ForestSpecies& fs = p.species[s];
                            if (fs.understory != under || fs.twinOnly) continue;
                            double altLo = fs.altLo, regionK = 1.0;
                            for (std::size_t r = 0; r < nRegions; ++r) {
                                if (inRegion[r] <= 0.0) continue;
                                const ForestRegion& R = p.regions[r];
                                regionK *= 1.0 + (R.weight[s] - 1.0) * inRegion[r];
                                if (R.named[s]) altLo += (std::min(altLo, R.altLo) - altLo) * inRegion[r];
                            }
                            const double band = smooth(altLo - 60.0, altLo + 20.0, alt) * (1.0 - smooth(fs.altHi - 20.0, fs.altHi + 60.0, alt));
                            if (band <= 0.0) continue;
                            const double k = regionK * (1.0 + fs.wet * ((0.03 + smooth(0.45, 0.8, wet)) * 4.0 - 1.0));
                            const double patch = sn.fbm2(x * 0.004 + 17.0 * s, z * 0.004 - 11.0 * s, 3);
                            const double w = std::log(band * fs.weight * k + 1e-6) + 1.6 * patch + 0.35 * unit(h ^ (0x1234u + static_cast<uint32_t>(s)));
                            if (w > bw) { bw = w; best = static_cast<int>(s); }
                        }
                        if (best < 0) continue;
                        // (a kind grown both ways: its recipe twin takes the tree where the twin's clumps fall --
                        // groves of tens of metres, ragged at their edges, about `twinShare` of the kind's trees)
                        if (const ForestSpecies& k = p.species[best]; k.twin >= 0) {
                            // (the clumps' noise is near normal, sd 0.19, measured; its CDF makes the share exact)
                            const double c = sn.fbm2(x * 0.02 + 31.0 * best, z * 0.02 - 23.0 * best, 2) + 0.08 * (unit(h ^ 0x7E7Eu) - 0.5);
                            if (0.5 * std::erfc(-c / (0.191 * std::sqrt(2.0))) < k.twinShare) best = k.twin;
                        }
                        const ForestSpecies& fs = p.species[best];
                        ForestTree tr;
                        tr.pos = Vec3(x, y, z);
                        tr.yaw = static_cast<float>(unit(h ^ 0x77u) * 6.2831853);
                        // stunted toward the treeline and at a stand's thin edge
                        const double tl = p.treelineM;
                        const double stunt = 1.0 - 0.5 * smooth(tl - 220.0, tl, alt);
                        tr.scale = static_cast<float>((0.72 + 0.5 * unit(h ^ 0x99u)) * stunt * (0.8 + 0.2 * d) * fs.scale);
                        // (its variant -- in a species grown both ways, the stand-grown half as the stand is thick here)
                        int nv = fs.variants, first = 0;
                        if (fs.formByDensity && nv >= 2) {
                            nv /= 2;
                            if (unit(h ^ 0xF0F0u) < smooth(0.30, 0.65, d)) first = nv;
                        }
                        tr.variant = static_cast<uint16_t>(best * vps + first + static_cast<int>(unit(h ^ 0x3131u) * nv) % nv);
                        parts[t].push_back(tr);
                    }
            });
        for (auto& x : th) x.join();
        for (auto& v : parts) out.insert(out.end(), v.begin(), v.end());
    }
    return out;
}

std::vector<PlacedRock> placeRocks(const ForestParams& p, double half, double sea, const GroundCover* cover,
                                   const TerrainMaps* maps, const std::function<double(double, double)>& ground,
                                   const std::function<bool(double, double)>& exclude,
                                   const std::function<double(double, double, double)>& shore,
                                   const std::function<bool(double, double)>& excludeShore) {
    std::vector<PlacedRock> out;
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    for (std::size_t li = 0; li < p.rocks.size(); ++li) {
        const RockLayer& L = p.rocks[li];
        const int n = static_cast<int>(std::ceil(2.0 * half / L.spacing));
        std::vector<std::vector<PlacedRock>> parts(T);
        std::vector<std::thread> th;
        for (int t = 0; t < T; ++t)
            th.emplace_back([&, t] {
                for (int j = t; j < n; j += T)
                    for (int i = 0; i < n; ++i) {
                        const uint32_t h = (static_cast<uint32_t>(i) * 73856093u ^ static_cast<uint32_t>(j) * 19349663u ^ p.seed * 83492791u) + 0xB0u * static_cast<uint32_t>(li + 1);
                        const double roll = unit(h ^ 0x51ED27u);
                        if (roll >= L.density) continue;
                        const double x = -half + (i + 0.1 + 0.8 * unit(h)) * L.spacing, z = -half + (j + 0.1 + 0.8 * unit(h ^ 0xA5A5u)) * L.spacing;
                        // the maps first (no ground sample): most of the island is the wrong ground
                        TerrainMapSample m;
                        if (maps) m = maps->at(x, z);
                        const double pre = forestPrefilter(p, maps, x, z);
                        bool ok = false;
                        switch (L.ground) {
                            case 0: ok = maps && m.scree > 0.25 && roll < L.density * std::min(1.0, m.scree * 1.6); break;
                            case 1: ok = maps && m.convex > 0.1 && m.soil < 0.35; break;
                            case 2: ok = maps && m.soil > 0.5 && pre < 0.15; break;
                            case 3: ok = pre > 0.5; break;
                            case 5: ok = static_cast<bool>(shore); break;
                            default: ok = true; break;
                        }
                        if (!ok) continue;
                        const double y = ground(x, z);
                        const double alt = y - (sea > -1e29 ? sea : 0.0);
                        if (sea > -1e29 && alt < 0.3) continue;   // (above the sea, when there is one)
                        const double e = 1.5;
                        const double gx = (ground(x + e, z) - ground(x - e, z)) / (2 * e), gz = (ground(x, z + e) - ground(x, z - e)) / (2 * e);
                        const double slope = std::atan(std::sqrt(gx * gx + gz * gz)) * 57.2957795;
                        if (cover) {
                            const Cover c = cover->at(x, z, y, std::cos(slope / 57.2957795));
                            if (L.ground == 1 && c.rock < 0.4) continue;
                            if (L.ground == 4 && c.sand < 0.5) continue;
                            if (L.ground != 1 && L.ground != 0 && c.snow > 0.3) continue;
                        }
                        if (L.ground == 3 && forestGrounded(p, sea, cover, x, z, y, slope) < 0.3) continue;
                        if (L.ground == 5 && roll >= L.density * shore(x, z, y)) continue;
                        if (slope > 55.0) continue;
                        const auto& ex = L.ground == 5 && excludeShore ? excludeShore : exclude;
                        if (ex && ex(x, z)) continue;
                        PlacedRock r;
                        r.pos = Vec3(x, y, z);
                        r.yaw = static_cast<float>(unit(h ^ 0x77u) * 6.2831853);
                        r.scale = static_cast<float>(0.55 + 0.9 * std::pow(unit(h ^ 0x99u), 2.0));   // many small, a few big
                        r.tiltDir = static_cast<float>(unit(h ^ 0x55u) * 6.2831853);
                        r.tilt = static_cast<float>(unit(h ^ 0x66u) * (L.ground == 1 ? 0.1 : 0.25) + std::min(0.35, slope / 57.2957795 * 0.5));
                        r.layer = static_cast<uint16_t>(li);
                        r.variant = static_cast<uint16_t>(static_cast<int>(unit(h ^ 0x3131u) * L.variants) % L.variants);
                        parts[t].push_back(r);
                    }
            });
        for (auto& x : th) x.join();
        for (auto& v : parts) out.insert(out.end(), v.begin(), v.end());
    }
    return out;
}

Seat seatOnGround(const std::function<double(double, double)>& ground, double x, double z, double foot, double bed) {
    Seat st;
    const double e = std::max(0.5, 0.6 * foot);
    const double gx = (ground(x + e, z) - ground(x - e, z)) / (2 * e);
    const double gz = (ground(x, z + e) - ground(x, z - e)) / (2 * e);
    st.normal = normalize(Vec3(-gx, 1.0, -gz));
    double y0 = ground(x, z);
    for (int k = 0; k < 8; ++k) {
        const double a = k * 0.785398, dx = foot * std::cos(a), dz = foot * std::sin(a);
        const double rise = -(st.normal.x * dx + st.normal.z * dz) / std::max(0.2, st.normal.y);   // the plane above its centre
        y0 = std::min(y0, ground(x + dx, z + dz) - rise);
    }
    st.baseY = y0 - bed;
    return st;
}

void appendImpostor(RenderMesh& mesh, const ForestTree& t, const ImpostorSlot& s, double colour) {
    const double sc = t.scale;
    const double hw = s.halfW * sc, H = s.height * sc;
    const double cy = std::cos(t.yaw), sy = std::sin(t.yaw);
    const Vec3 base = t.pos - Vec3(0, 0.15 * sc, 0);
    const Vec3 col(colour, colour, colour);
    // each card's own frame for the baked normal map (#93, real_tree.h renderImpostor): N the card's plane,
    // T its u axis; the map carries the crown's shape, so the card lights clump by clump like the model
    auto quad = [&](const Vec3 p[4], const Vec3 n[4], const Vec3& tangent, const double uv[4][2]) {
        const uint32_t b = static_cast<uint32_t>(mesh.vertices.size());
        for (int k = 0; k < 4; ++k) {
            Vertex v(p[k], n[k], tangent, static_cast<float>(uv[k][0]), static_cast<float>(uv[k][1]));
            v.color = col;
            mesh.vertices.push_back(v);
        }
        MeshBuilder::quadFacing(mesh, b, b + 1, b + 2, b + 3, n[0] + n[1] + n[2] + n[3]);
    };
    // the side picture's rows run top (v0) to the foot (v1): image row 0 is the tree's top
    for (int k = 0; k < 2; ++k) {
        const double a = k * 1.5707963;
        const Vec3 side(cy * std::cos(a) - sy * std::sin(a), 0.0, sy * std::cos(a) + cy * std::sin(a));
        const Vec3 p[4] = {base - side * hw, base + side * hw, base + side * hw + Vec3(0, H, 0), base - side * hw + Vec3(0, H, 0)};
        // the plane's normal (side x up): the bend -- out to each side, up toward the crown's top -- is in the
        // normal map now, per texel, instead of four corner normals smoothed across the card
        const Vec3 plane = normalize(cross(side, Vec3(0, 1, 0)));
        const Vec3 n[4] = {plane, plane, plane, plane};
        // the crossed card shows the tree TURNED a quarter (Glenn, 2026-10-02: "the tree cards in the distance are
        // still pretty visibly tree cards"): one picture on both cards repeated the same silhouette twice
        const bool second = k == 1 && s.sv0 >= 0.0;
        const double va = second ? s.sv0 : s.v0, vb = second ? s.sv1 : s.v1;
        const double uv[4][2] = {{s.u0, vb}, {s.u1, vb}, {s.u1, va}, {s.u0, va}};
        quad(p, n, side, uv);
    }
    // the top card, at the crown's broadest, facing up (x -> u, z -> v, turned by the yaw)
    const double yTop = (s.crownBase + 0.55 * (s.height - s.crownBase)) * sc;
    const Vec3 ax(cy, 0, sy), az(-sy, 0, cy);
    const Vec3 c = base + Vec3(0, yTop, 0);
    const Vec3 p[4] = {c - ax * hw - az * hw, c + ax * hw - az * hw, c + ax * hw + az * hw, c - ax * hw + az * hw};
    const Vec3 up(0, 1, 0);
    const Vec3 n[4] = {up, up, up, up};
    const double uv[4][2] = {{s.tu0, s.tv0}, {s.tu1, s.tv0}, {s.tu1, s.tv1}, {s.tu0, s.tv1}};
    quad(p, n, ax, uv);
}

void appendImpostorViews(RenderMesh& mesh, const ForestTree& t, const ImpostorSlot& s, double colour) {
    mesh.tangentIsData = true;
    const double sc = t.scale;
    const Vec3 c = t.pos + Vec3(0, (s.centreY - 0.15) * sc, 0);
    const double rb = s.Rb * sc, hh = s.Hh * sc;
    const uint32_t b = static_cast<uint32_t>(mesh.vertices.size());
    const double corner[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    for (int k = 0; k < 4; ++k) {
        const double cx = corner[k][0], cy = corner[k][1];
        // (the centre plus a reach in x, z and y: mesh.vert takes it back off -- the bounds hold the crowns)
        Vertex v(c + Vec3(cx * rb, cy * hh, cx * rb), Vec3(s.vu0, s.vv0, s.cellU), Vec3(rb, hh, t.yaw),
                 static_cast<float>(cx), static_cast<float>(cy));
        v.color = Vec3(colour, s.cellV, 0.0);
        mesh.vertices.push_back(v);
    }
    MeshBuilder::quadFacing(mesh, b, b + 1, b + 2, b + 3, Vec3(-1, 0, 1));   // (the corners' plane; drawn two-sided)
}

void appendImpostorShadow(RenderMesh& mesh, const ForestTree& t, const ImpostorSlot& s) {
    // the views from +z (azimuth 0) and from +x (a quarter round) at the horizon, as two crossed cards through the
    // centre, turned by the yaw: the sun sees a crown's shape from either
    const double sc = t.scale, cy = std::cos(t.yaw), sy = std::sin(t.yaw);
    const Vec3 c = t.pos + Vec3(0, (s.centreY - 0.15) * sc, 0);
    const double rb = s.Rb * sc, hh = s.Hh * sc;
    for (int k = 0; k < 2; ++k) {
        const double az = k * 2.0 * 3.14159265358979323846 / kImpostorAzimuths * (kImpostorAzimuths / 4);
        const Vec3 rl(std::cos(az), 0.0, -std::sin(az));               // the view's r, the tree's own frame
        const Vec3 r(rl.x * cy - rl.z * sy, 0.0, rl.x * sy + rl.z * cy);   // ...turned by the yaw
        const Vec3 n = normalize(cross(r, Vec3(0, 1, 0)));
        const int col = k * (kImpostorAzimuths / 4);
        const double u0 = s.vu0 + col * s.cellU, u1 = u0 + s.cellU, v0 = s.vv0, v1 = s.vv0 + s.cellV;
        const uint32_t b = static_cast<uint32_t>(mesh.vertices.size());
        const Vec3 p[4] = {c - r * rb - Vec3(0, hh, 0), c + r * rb - Vec3(0, hh, 0), c + r * rb + Vec3(0, hh, 0), c - r * rb + Vec3(0, hh, 0)};
        const double uv[4][2] = {{u0, v1}, {u1, v1}, {u1, v0}, {u0, v0}};   // (row 0 the view's top)
        for (int q = 0; q < 4; ++q) {
            Vertex v(p[q], n, r, static_cast<float>(uv[q][0]), static_cast<float>(uv[q][1]));
            v.color = Vec3(1, 1, 1);
            mesh.vertices.push_back(v);
        }
        MeshBuilder::quadFacing(mesh, b, b + 1, b + 2, b + 3, n);
    }
}

}  // namespace engine
