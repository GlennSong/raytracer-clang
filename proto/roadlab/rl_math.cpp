#include "rl_math.h"

namespace roadlab {
namespace {

// Integer hash -> [0,1). Cheap, stable across platforms (no floating-point in
// the hash itself), which matters because the shader is a reference for a GPU
// port and the two must agree.
inline double hash2(int64_t x, int64_t y) {
    uint64_t h = uint64_t(x) * 0x9E3779B97F4A7C15ull ^ uint64_t(y) * 0xC2B2AE3D27D4EB4Full;
    h ^= h >> 29;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 32;
    return double(h >> 11) * (1.0 / 9007199254740992.0);
}

inline double fade(double t) { return t * t * (3.0 - 2.0 * t); }

}  // namespace

double valueNoise(double x, double y) {
    double fx = std::floor(x), fy = std::floor(y);
    int64_t ix = int64_t(fx), iy = int64_t(fy);
    double tx = fade(x - fx), ty = fade(y - fy);
    double a = hash2(ix, iy), b = hash2(ix + 1, iy);
    double c = hash2(ix, iy + 1), d = hash2(ix + 1, iy + 1);
    return lerp(lerp(a, b, tx), lerp(c, d, tx), ty);
}

double fbm(double x, double y, int octaves, double lacunarity, double gain) {
    double sum = 0.0, amp = 0.5, norm = 0.0, fx = x, fy = y;
    for (int i = 0; i < octaves; ++i) {
        sum += amp * valueNoise(fx, fy);
        norm += amp;
        fx *= lacunarity;
        fy *= lacunarity;
        amp *= gain;
    }
    return norm > 0 ? sum / norm : 0.0;
}

double worley(double x, double y) {
    double fx = std::floor(x), fy = std::floor(y);
    int64_t ix = int64_t(fx), iy = int64_t(fy);
    double best = 1e9;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int64_t cx = ix + dx, cy = iy + dy;
            double px = double(cx) + hash2(cx, cy);
            double py = double(cy) + hash2(cx + 7919, cy - 104729);
            double d = (px - x) * (px - x) + (py - y) * (py - y);
            best = std::min(best, d);
        }
    }
    return std::sqrt(std::min(best, 1e9));
}

}  // namespace roadlab
