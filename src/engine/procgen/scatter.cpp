#include "scatter.h"

#include <cmath>
#include <random>

namespace engine {

std::vector<Placement> scatterOnTerrain(const ScatterParams& params,
                                        const TerrainParams& terrain,
                                        const Noise& terrainNoise) {
    std::vector<Placement> placements;
    std::mt19937 gen(params.seed);
    std::uniform_real_distribution<float> coord(-params.regionSize * 0.5f,
                                                 params.regionSize * 0.5f);
    std::uniform_real_distribution<float> yawDist(0.0f, 2.0f * static_cast<float>(PI));
    std::uniform_real_distribution<float> scaleDist(params.minScale, params.maxScale);

    // A separate, seed-derived field for the density mask, so clumping is
    // independent of the terrain's own shape but still reproducible.
    Noise density(params.seed + 1u);

    const double minSlopeCos = std::cos(params.maxSlopeDeg * PI / 180.0);
    const double eps = 0.5;   // world-space step for the slope finite difference

    for (int i = 0; i < params.count; i++) {
        // Draw all four values every iteration so the RNG sequence depends only
        // on `count`, not on how many candidates are accepted.
        float x = coord(gen);
        float z = coord(gen);
        float yaw = yawDist(gen);
        float scale = scaleDist(gen);

        double h = terrainHeight(terrain, terrainNoise, x, z);
        if (h < params.minHeight || h > params.maxHeight) continue;

        // Surface slope from a finite-difference gradient of the height field;
        // the up-component of the normal is cos(slope).
        double hx = (terrainHeight(terrain, terrainNoise, x + eps, z) -
                     terrainHeight(terrain, terrainNoise, x - eps, z)) / (2.0 * eps);
        double hz = (terrainHeight(terrain, terrainNoise, x, z + eps) -
                     terrainHeight(terrain, terrainNoise, x, z - eps)) / (2.0 * eps);
        Vec3 normal = normalize(Vec3(-hx, 1.0, -hz));
        if (normal.y < minSlopeCos) continue;   // too steep

        double d = density.noise2(x * params.densityScale, z * params.densityScale);
        if (d < params.densityThreshold) continue;

        // Focal point: a clearing around the hero, and scale stepping down with
        // distance from it (bigger near the focus, base size out at focusRadius).
        if (params.focusClear > 0.0f || params.focusRadius > 0.0f) {
            double fx = x - params.focus.x, fz = z - params.focus.z;
            double fdist = std::sqrt(fx * fx + fz * fz);
            if (params.focusClear > 0.0f && fdist < params.focusClear) continue;
            if (params.focusRadius > 0.0f) {
                double t = std::min(fdist / params.focusRadius, 1.0);
                scale *= static_cast<float>(params.focusScale +
                                            (1.0 - params.focusScale) * t);
            }
        }

        // Footprint spacing (dart-throwing Poisson disk): reject if too close in
        // XZ to an accepted placement, so large instances don't jumble. The RNG
        // was already drawn above, so the sequence stays acceptance-independent.
        if (params.minSpacing > 0.0f) {
            const double minSq =
                static_cast<double>(params.minSpacing) * params.minSpacing;
            bool tooClose = false;
            for (const Placement& p : placements) {
                double dx = p.position.x - x, dz = p.position.z - z;
                if (dx * dx + dz * dz < minSq) { tooClose = true; break; }
            }
            if (tooClose) continue;
        }

        placements.push_back({Vec3(x, h, z), yaw, scale});
    }
    return placements;
}

std::vector<std::vector<Mat4>> bucketPlacementsBySpecies(
    const std::vector<Placement>& placements, std::size_t numSpecies, uint32_t seed) {
    std::vector<std::vector<Mat4>> buckets(numSpecies);
    if (numSpecies == 0) return buckets;

    std::mt19937 pick(seed);
    std::uniform_int_distribution<std::size_t> speciesPick(0, numSpecies - 1);
    for (const Placement& pl : placements) {
        std::size_t si = speciesPick(pick);   // every placement lands in a bucket
        Mat4 m = Mat4::trs(pl.position,
                           Quat::fromAxisAngle(Vec3(0, 1, 0), pl.yaw),
                           Vec3(pl.scale, pl.scale, pl.scale));
        buckets[si].push_back(m);
    }
    return buckets;
}

}  // namespace engine
