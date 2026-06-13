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

        placements.push_back({Vec3(x, h, z), yaw, scale});
    }
    return placements;
}

}  // namespace engine
