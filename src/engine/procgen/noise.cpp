#include "noise.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <random>

namespace engine {

namespace {
// THE TERRAIN'S PERMUTATION, FROZEN (ADR-0265): what libstdc++'s std::shuffle over std::mt19937 laid on the islands
// every bake so far grew from, written out so every host shuffles alike. mt19937 itself is fixed by the standard;
// shuffle's draws are not (libc++ and MSVC differ), and a new stream would move every hill under the planned cities.
// libstdc++ (GCC 11+): swaps in pairs, both positions from one draw of Lemire's nearly divisionless range.
uint32_t lemireBelow(std::mt19937& g, uint32_t range) {
    uint64_t product = static_cast<uint64_t>(g()) * range;
    uint32_t low = static_cast<uint32_t>(product);
    if (low < range) {
        const uint32_t threshold = static_cast<uint32_t>(-range) % range;
        while (low < threshold) {
            product = static_cast<uint64_t>(g()) * range;
            low = static_cast<uint32_t>(product);
        }
    }
    return static_cast<uint32_t>(product >> 32);
}
}  // namespace

void frozenShuffle256(std::array<int, 256>& p, uint32_t seed) {
    std::mt19937 g(seed);
    std::size_t i = 1;
    std::swap(p[i], p[lemireBelow(g, 2)]);   // an even count: the first swap on its own
    ++i;
    while (i < p.size()) {
        const uint32_t range = static_cast<uint32_t>(i) + 1;
        const uint32_t x = lemireBelow(g, range * (range + 1));
        std::swap(p[i], p[x / (range + 1)]);
        ++i;
        std::swap(p[i], p[x % (range + 1)]);
        ++i;
    }
}

namespace {
double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }
double lerp(double t, double a, double b) { return a + t * (b - a); }
}  // namespace

Noise::Noise(uint32_t seed) {
    std::array<int, 256> p;
    std::iota(p.begin(), p.end(), 0);
    frozenShuffle256(p, seed);
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
