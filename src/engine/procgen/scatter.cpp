#include "scatter.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

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

    // Cluster parents (Thomas process): a few seed points the candidates clump
    // around. Drawn from a separate stream so the per-candidate sequence below is
    // unchanged when clustering is off.
    std::vector<std::pair<float, float>> parents;
    if (params.clusterCount > 0) {
        std::mt19937 pgen(params.seed + 99u);
        std::uniform_real_distribution<float> pc(-params.regionSize * 0.5f,
                                                 params.regionSize * 0.5f);
        for (int i = 0; i < params.clusterCount; i++)
            parents.push_back({pc(pgen), pc(pgen)});
    }

    for (int i = 0; i < params.count; i++) {
        // Draw a fixed number of values every iteration so the RNG sequence
        // depends only on `count` (and the cluster mode), not on acceptance.
        float x, z, yaw, scale;
        if (params.clusterCount > 0) {
            // Pick a parent and offset by a Box-Muller Gaussian (2 uniforms ->
            // 2 normals; a fixed draw count keeps it deterministic).
            std::uniform_real_distribution<float> u01(0.0f, 1.0f);
            int pi = std::min(static_cast<int>(u01(gen) * params.clusterCount),
                              params.clusterCount - 1);
            float a = u01(gen), b = u01(gen);
            float r = params.clusterRadius *
                      std::sqrt(-2.0f * std::log(std::max(a, 1e-6f)));
            x = parents[pi].first + r * std::cos(2.0f * static_cast<float>(PI) * b);
            z = parents[pi].second + r * std::sin(2.0f * static_cast<float>(PI) * b);
            yaw = yawDist(gen);
            scale = scaleDist(gen);
        } else {
            x = coord(gen);
            z = coord(gen);
            yaw = yawDist(gen);
            scale = scaleDist(gen);
        }

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
