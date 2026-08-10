#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_RNG_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_RNG_H

#include "../../../rt_math.h"   // Real
#include <cstdint>

namespace engine {

// The xorshift32 every procgen module reaches for. Deterministic for its seed,
// which is the whole contract: the same seed must rebuild the same city.
//
// NOTE: district.cpp, city.cpp and road_network.cpp each carry their own copy
// of this struct, differing ONLY in the constant substituted for a zero seed
// (0x2545f491 / 0x1234567 / 0x6c078965). They are deliberately NOT converted
// here: swapping their fallback constants would change the geometry those
// modules generate for a zero seed, which is a behavioural change to the
// shipped city and wants its own reviewed commit rather than a drive-by. New
// code uses this one.
struct Rng {
    std::uint32_t s;
    explicit Rng(std::uint32_t seed) : s(seed ? seed : 0x2545f491u) {}

    std::uint32_t next() {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return s;
    }
    Real unit() { return (next() >> 8) * (1.0 / 16777216.0); }
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
    int irange(int a, int b) {
        if (b < a) return a;
        return a + static_cast<int>(unit() * (b - a + 1));
    }
    bool chance(Real p) { return unit() < p; }
};

}  // namespace engine

#endif
