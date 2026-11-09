#ifndef RAYTRACER_ENGINE_NOISE_H
#define RAYTRACER_ENGINE_NOISE_H

#include "../../hash.h"   // hashCell, unit24: the canonical lattice
#include <array>
#include <cmath>
#include <cstdint>

namespace engine {

// The Perlin table's shuffle: libstdc++'s std::shuffle over std::mt19937(seed), frozen so every host lays the same
// terrain (noise.cpp; a test holds it to std::shuffle on libstdc++).
void frozenShuffle256(std::array<int, 256>& p, uint32_t seed);

// THE BLEND of a lattice square's four corners a, b (along x), c, d (a row up) at smoothstepped u, v: the bilinear one
struct LerpBlend {
    template <class T>
    T operator()(T a, T b, T c, T d, T u, T v) const {
        return (a + (b - a) * u) + ((c + (d - c) * u) - (a + (b - a) * u)) * v;
    }
};
// FROZEN BLENDS: the same bilinear blend, its arithmetic in the order a baked noise was written in (the rounding is
// baked too: the lanes' recipe terrain, a tree's textures). Retire each with its frozen lattice (hash.h) at its bake's
// next tag bump.
struct FrozenTerrainRecipeBlend {
    template <class T>
    T operator()(T a, T b, T c, T d, T u, T v) const {
        return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + d * u) * v;
    }
};
struct FrozenTreeTextureBlend {
    template <class T>
    T operator()(T a, T b, T c, T d, T u, T v) const {
        return (a + (b - a) * u) + ((c - a) + (d - c) * u - (b - a) * u) * v;
    }
};

// VALUE NOISE (2026-11-08: it had been written three times -- the wind's gusts, a tree's textures, the lanes' recipe
// terrain): a value per lattice node (`node(i, j)`, in [0, 1)), smoothstep-blended between the four round a point.
// Cheaper and blockier than the gradient Noise below. T: float or double. `blend`: LerpBlend, but for a frozen one.
template <class T, class Node, class Blend = LerpBlend>
T latticeValueNoise(T x, T y, Node&& node, Blend blend = {}) {
    const T fx = std::floor(x), fy = std::floor(y);
    const int64_t i = static_cast<int64_t>(fx), j = static_cast<int64_t>(fy);
    T u = x - fx, v = y - fy;
    u = u * u * (T(3) - T(2) * u);
    v = v * v * (T(3) - T(2) * v);
    const T a = static_cast<T>(node(i, j)), b = static_cast<T>(node(i + 1, j));
    const T c = static_cast<T>(node(i, j + 1)), d = static_cast<T>(node(i + 1, j + 1));
    return blend(a, b, c, d, u, v);
}
// ...in 3D: trilinear, x first, then y, then z. P the position's type (the fraction is taken in it), T the value's.
template <class T, class P, class Node>
T latticeValueNoise3(P x, P y, P z, Node&& node) {
    const P fx = std::floor(x), fy = std::floor(y), fz = std::floor(z);
    const int64_t i = static_cast<int64_t>(fx), j = static_cast<int64_t>(fy), k = static_cast<int64_t>(fz);
    T u = static_cast<T>(x - fx), v = static_cast<T>(y - fy), w = static_cast<T>(z - fz);
    u = u * u * (T(3) - T(2) * u);
    v = v * v * (T(3) - T(2) * v);
    w = w * w * (T(3) - T(2) * w);
    auto L = [](T a, T b, T t) { return a + (b - a) * t; };
    T c[2][2];
    for (int dj = 0; dj < 2; ++dj)
        for (int dk = 0; dk < 2; ++dk)
            c[dj][dk] = L(static_cast<T>(node(i, j + dj, k + dk)), static_cast<T>(node(i + 1, j + dj, k + dk)), u);
    return L(L(c[0][0], c[1][0], v), L(c[0][1], c[1][1], v), w);
}
// ...over hash.h's grid hash: the one to use, in [0, 1)
inline double valueNoise2(double x, double y, uint32_t seed) {
    return latticeValueNoise(x, y, [seed](int64_t i, int64_t j) {
        return unit24(hashCell(static_cast<uint32_t>(i), static_cast<uint32_t>(j), seed));
    });
}

// Seedable coherent noise for procgen (ROADMAP 3.7). Deterministic by
// construction (same seed + coordinates -> same value), so generated worlds are
// reproducible — the determinism ADR-0021/0002 require. Improved Perlin gradient
// noise underneath; 2D is the z=0 slice of the 3D field.
class Noise {
public:
    explicit Noise(uint32_t seed = 0);

    // Gradient noise, roughly in [-1, 1], smooth (C1-continuous).
    double noise2(double x, double y) const { return noise3(x, y, 0.0); }
    double noise3(double x, double y, double z) const;

    // Fractal Brownian motion: sum `octaves` of noise at rising frequency
    // (×lacunarity) and falling amplitude (×gain), normalized to ~[-1, 1]. The
    // workhorse for natural-looking terrain and textures.
    double fbm2(double x, double y, int octaves = 4,
                double lacunarity = 2.0, double gain = 0.5) const;
    double fbm3(double x, double y, double z, int octaves = 4,
                double lacunarity = 2.0, double gain = 0.5) const;

    // Domain warp: displace the sample point by noise before sampling, for the
    // swirly, organic look (ridged terrain, marble). `warp` scales the offset.
    double warpedFbm2(double x, double y, double warp = 1.0,
                      int octaves = 4) const;

    // CURL NOISE (Bridson 2007): the 2D curl of the gradient noise as a stream function -- (dN/dy, -dN/dx), a
    // divergence-free flow: it swirls and shears what it carries, never piles it up or thins it. For eddies in the
    // air (the clouds' weather map, ADR-0386; the wind field to come). Central differences `h` apart; roughly in
    // [-2, 2] per component.
    void curl2(double x, double y, double& outX, double& outY, double h = 0.05) const;

private:
    double gradDot(int hash, double x, double y, double z) const;

    int perm_[512];   // 0..255 permutation, seeded and duplicated for wrap-free indexing
};

}  // namespace engine

#endif
