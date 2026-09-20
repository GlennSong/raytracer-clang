#include "engine/procgen/city/roads/lanes/terrain_recipe.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

namespace engine {
namespace roads::lanes {

namespace {

// Deterministic lattice value noise: a hash per lattice node, smoothstep-interpolated.
double hash01(int64_t ix, int64_t iy, uint32_t seed) {
    uint64_t h = static_cast<uint64_t>(ix) * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(iy) * 0xC2B2AE3D27D4EB4Full ^ (static_cast<uint64_t>(seed) + 0x165667B19E3779F9ull);
    h ^= h >> 31; h *= 0x7FB5D329728EA185ull; h ^= h >> 27; h *= 0x81DADEF4BC2DD44Dull; h ^= h >> 33;
    return static_cast<double>(h >> 11) / 9007199254740992.0;
}

double valueNoise(double x, double y, double cell, uint32_t seed) {
    double fx = x / cell, fy = y / cell; double ix = std::floor(fx), iy = std::floor(fy); double tx = fx - ix, ty = fy - iy;
    tx = tx * tx * (3 - 2 * tx); ty = ty * ty * (3 - 2 * ty);
    int64_t i = static_cast<int64_t>(ix), j = static_cast<int64_t>(iy);
    double a = hash01(i, j, seed), b = hash01(i + 1, j, seed), c = hash01(i, j + 1, seed), d = hash01(i + 1, j + 1, seed);
    return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
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
    return [s](double x, double y) {
        double h = 0; uint32_t seed = static_cast<uint32_t>(s.seed);
        for (size_t k = 0; k < s.octaves.size(); ++k) h += s.octaves[k].first * valueNoise(x, y, s.octaves[k].second, seed + static_cast<uint32_t>(k));
        for (const auto& v : s.valleys) { double u = (v.alongY ? y : x) - v.c; h -= v.depth * std::exp(-(u / v.width) * (u / v.width)); }
        for (const auto& b : s.hills) h += b.h * std::exp(-((x - b.x) * (x - b.x) + (y - b.y) * (y - b.y)) / (b.r * b.r));
        if (s.hasTilt) h += s.dzdx * (x - s.x0) + s.dzdy * (y - s.y0);
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
