#include "noise.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <random>

namespace engine {

namespace {
double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }
double lerp(double t, double a, double b) { return a + t * (b - a); }
}  // namespace

Noise::Noise(uint32_t seed) {
    std::array<int, 256> p;
    std::iota(p.begin(), p.end(), 0);
    std::mt19937 gen(seed);
    std::shuffle(p.begin(), p.end(), gen);
    for (int i = 0; i < 256; i++) perm_[i] = perm_[i + 256] = p[i];
}

// Ken Perlin's 12 edge-gradients, selected by the low bits of the hash.
double Noise::gradDot(int hash, double x, double y, double z) const {
    int h = hash & 15;
    double u = h < 8 ? x : y;
    double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

double Noise::noise3(double x, double y, double z) const {
    int X = static_cast<int>(std::floor(x)) & 255;
    int Y = static_cast<int>(std::floor(y)) & 255;
    int Z = static_cast<int>(std::floor(z)) & 255;
    x -= std::floor(x);
    y -= std::floor(y);
    z -= std::floor(z);
    double u = fade(x), v = fade(y), w = fade(z);

    int A = perm_[X] + Y, AA = perm_[A] + Z, AB = perm_[A + 1] + Z;
    int B = perm_[X + 1] + Y, BA = perm_[B] + Z, BB = perm_[B + 1] + Z;

    return lerp(w,
        lerp(v, lerp(u, gradDot(perm_[AA], x, y, z),
                        gradDot(perm_[BA], x - 1, y, z)),
                lerp(u, gradDot(perm_[AB], x, y - 1, z),
                        gradDot(perm_[BB], x - 1, y - 1, z))),
        lerp(v, lerp(u, gradDot(perm_[AA + 1], x, y, z - 1),
                        gradDot(perm_[BA + 1], x - 1, y, z - 1)),
                lerp(u, gradDot(perm_[AB + 1], x, y - 1, z - 1),
                        gradDot(perm_[BB + 1], x - 1, y - 1, z - 1))));
}

double Noise::fbm3(double x, double y, double z, int octaves,
                   double lacunarity, double gain) const {
    double sum = 0.0, amp = 0.5, freq = 1.0, norm = 0.0;
    for (int i = 0; i < octaves; i++) {
        sum += amp * noise3(x * freq, y * freq, z * freq);
        norm += amp;
        amp *= gain;
        freq *= lacunarity;
    }
    return norm > 0.0 ? sum / norm : 0.0;
}

double Noise::fbm2(double x, double y, int octaves, double lacunarity,
                   double gain) const {
    return fbm3(x, y, 0.0, octaves, lacunarity, gain);
}

double Noise::warpedFbm2(double x, double y, double warp, int octaves) const {
    // Offset coordinates so the two warp channels are decorrelated.
    double qx = fbm2(x, y, octaves);
    double qy = fbm2(x + 5.2, y + 1.3, octaves);
    return fbm2(x + warp * qx, y + warp * qy, octaves);
}

}  // namespace engine
