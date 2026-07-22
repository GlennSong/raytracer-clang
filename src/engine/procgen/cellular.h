#ifndef RAYTRACER_ENGINE_PROCGEN_CELLULAR_H
#define RAYTRACER_ENGINE_PROCGEN_CELLULAR_H

#include <cstdint>

namespace engine {

// 3D Worley / cellular noise (procedural-planet-plan, the noise-library gap ADR-0076
// flags). Each integer cell carries one jittered feature point; the sample returns
// the nearest (F1) and second-nearest (F2) distances plus a hash id of the nearest
// feature's cell. Deterministic in `seed` (ADR-0021). The crater field reads F1 as
// distance-to-crater-centre and `cellId` as the per-crater random seed (radius,
// depth, whether the cell is a crater at all).
struct WorleyResult {
    double f1 = 0.0;        // distance to the nearest feature point (cell units)
    double f2 = 0.0;        // distance to the second-nearest (F2 >= F1)
    uint32_t cellId = 0;    // hash of the nearest feature's cell — a per-feature seed
};

WorleyResult worley3(uint32_t seed, double x, double y, double z);

}  // namespace engine

#endif
