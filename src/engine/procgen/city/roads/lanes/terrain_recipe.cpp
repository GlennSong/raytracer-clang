#include "engine/procgen/city/roads/lanes/terrain_recipe.h"

#include "engine/level_params.h"   // readTerrainParams: a level terrain block, read the way the level reads it
#include "engine/procgen/terrain.h"
#include "engine/procgen/noise.h"   // latticeValueNoise
#include "hash.h"   // frozenTerrainRecipeLattice
#include "engine/procgen/hydrology.h"   // the base's rivers: relief stands back from them

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

namespace engine {
namespace roads::lanes {

namespace {

// Value noise (procgen/noise.h) at `cell` metres, over the recipe terrain's own frozen lattice (hash.h: kept until the
// lanes' next tag)
double valueNoise(double x, double y, double cell, uint32_t seed) {
    return latticeValueNoise(x / cell, y / cell, [seed](int64_t i, int64_t j) { return frozenTerrainRecipeLattice(i, j, seed); });
}

}  // namespace


HeightField makeTerrain(const TerrainSpec& spec, const std::array<double, 4>& bounds) {
    (void)bounds;
    if (spec.type == "flat") return [](double, double) { return 0.0; };
    if (spec.type == "grid") {
        std::ifstream f(spec.file); if (!f) throw std::runtime_error("terrain grid file not found: " + spec.file);
        nlohmann::json j; f >> j; auto g = std::make_shared<HeightGrid>();
        g->x0 = j.at("x0").get<double>(); g->y0 = j.at("y0").get<double>(); g->res = j.at("res").get<double>(); g->nx = j.at("nx").get<int>(); g->ny = j.at("ny").get<int>();
        g->z = j.at("z").get<std::vector<double>>();
        return [g](double x, double y) { return g->sample(x, y); };
    }
    if (spec.type != "procedural") throw std::runtime_error("unknown terrain type " + spec.type);
    TerrainSpec s = spec;
    if (s.octaves.empty()) s.octaves = {{18, 140}, {7, 55}, {2, 22}};
    std::shared_ptr<const TerrainParams> baseParams;
    std::shared_ptr<const Noise> baseNoise;
    if (!s.base.is_null()) {
        TerrainParams bp = readTerrainParams(s.base);
        // an ERODED base is the ground the level renders (and the island planner sites on): the
        // city must stand on it too, not on the analytic relief it was eroded from
        if (s.base.value("erode", false)) bp.erodedBase = erodedForTerrain(s.base);
        baseParams = std::make_shared<const TerrainParams>(std::move(bp));
        baseNoise = std::make_shared<const Noise>(s.base.value("seed", 0u));   // as the level loader seeds it
    }
    return [s, baseParams, baseNoise](double x, double y) {
        double h = 0; uint32_t seed = static_cast<uint32_t>(s.seed);
        for (size_t k = 0; k < s.octaves.size(); ++k) h += s.octaves[k].first * valueNoise(x, y, s.octaves[k].second, seed + static_cast<uint32_t>(k));
        for (const auto& v : s.valleys) { double u = (v.alongY ? y : x) - v.c; h -= v.depth * std::exp(-(u / v.width) * (u / v.width)); }
        for (const auto& b : s.hills) h += b.h * std::exp(-((x - b.x) * (x - b.x) + (y - b.y) * (y - b.y)) / (b.r * b.r));
        if (s.hasFade) h *= 1.0 - smoothstep01((std::hypot(x - s.fadeX, y - s.fadeY) - s.fadeR0) / std::max(1e-6, s.fadeR1 - s.fadeR0));
        if (s.hasCalm) h *= 1.0 - smoothstep01((x * s.calmDx + y * s.calmDy - s.calmFrom) / std::max(1e-6, s.calmTo - s.calmFrom));
        // RIVERS (the base's hydrology, ADR-0099): the city's hills stand back from the water. The
        // base carves the channel and sets the water level; relief laid over it would put bumps in
        // the river bed and hills in the water. It fades out over the last 100 m to a bank, and a
        // HOLLOW (negative relief) further out still -- land below the water beside a river reads
        // as a river on a levee.
        if (baseParams && baseParams->hydro) {
            const double d = baseParams->hydro->distanceToRiver(x, y, 320.0);
            h *= smoothstep01((d - 10.0) / 100.0);
            if (h < 0.0) h *= smoothstep01((d - 10.0) / 300.0);
        }
        if (s.hasTilt) h += s.dzdx * (x - s.x0) + s.dzdy * (y - s.y0);
        if (baseParams) h += terrainHeight(*baseParams, *baseNoise, x, y);   // lanes y is world z
        return h;
    };
}

HeightGrid bakeGrid(const HeightField& h, const TerrainSpec& spec, const std::array<double, 4>& bounds) {
    HeightGrid g; g.x0 = bounds[0]; g.y0 = bounds[2]; g.res = spec.res;
    g.nx = static_cast<int>(std::floor((bounds[1] - bounds[0]) / g.res + 0.5)) + 1; g.ny = static_cast<int>(std::floor((bounds[3] - bounds[2]) / g.res + 0.5)) + 1;
    g.z.resize(static_cast<size_t>(g.nx) * g.ny);
    for (int j = 0; j < g.ny; ++j) for (int i = 0; i < g.nx; ++i) g.at(i, j) = h(g.x0 + i * g.res, g.y0 + j * g.res);
    return g;
}

}  // namespace roads::lanes
}  // namespace engine
