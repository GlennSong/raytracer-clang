#ifndef RAYTRACER_ENGINE_NOISE_H
#define RAYTRACER_ENGINE_NOISE_H

#include <cstdint>

namespace engine {

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

private:
    double gradDot(int hash, double x, double y, double z) const;

    int perm_[512];   // 0..255 permutation, seeded and duplicated for wrap-free indexing
};

}  // namespace engine

#endif
