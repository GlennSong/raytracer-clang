#include "cellular.h"

#include <cmath>

namespace engine {

namespace {

// A cheap integer avalanche (Murmur/`fmix`-style) so nearby cells decorrelate.
inline uint32_t hashU32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

inline uint32_t hashCell(int ix, int iy, int iz, uint32_t seed) {
    uint32_t h = seed * 0x9e3779b1u + 0x1234567u;
    h = hashU32(h ^ static_cast<uint32_t>(ix * 73856093));
    h = hashU32(h ^ static_cast<uint32_t>(iy * 19349663));
    h = hashU32(h ^ static_cast<uint32_t>(iz * 83492791));
    return h;
}

// Uniform in [0,1) from the top 24 bits (the low bits of an LCG-ish hash are the
// weakest).
inline double u01(uint32_t h) { return (h >> 8) * (1.0 / 16777216.0); }

}  // namespace

WorleyResult worley3(uint32_t seed, double x, double y, double z) {
    const int ix = static_cast<int>(std::floor(x));
    const int iy = static_cast<int>(std::floor(y));
    const int iz = static_cast<int>(std::floor(z));

    double f1 = 1e30, f2 = 1e30;
    uint32_t bestCell = 0;

    for (int dz = -1; dz <= 1; dz++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                const int cx = ix + dx, cy = iy + dy, cz = iz + dz;
                const uint32_t h = hashCell(cx, cy, cz, seed);
                // Jittered feature point inside this cell.
                const double fx = cx + u01(h);
                const double fy = cy + u01(hashU32(h + 0x9e3779b1u));
                const double fz = cz + u01(hashU32(h + 0x85ebca6bu));
                const double ex = fx - x, ey = fy - y, ez = fz - z;
                const double d = std::sqrt(ex * ex + ey * ey + ez * ez);
                if (d < f1) {
                    f2 = f1;
                    f1 = d;
                    bestCell = h;
                } else if (d < f2) {
                    f2 = d;
                }
            }
        }
    }
    return {f1, f2, bestCell};
}

}  // namespace engine
