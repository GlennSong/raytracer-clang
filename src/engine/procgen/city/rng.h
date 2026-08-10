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

    explicit Rng(std::uint32_t seed) {
        // SCRAMBLE THE SEED before use. xorshift32's first outputs are strongly
        // correlated for nearby seeds, so seeding it raw and taking one number
        // — the "one RNG per building, ask it one question" pattern this whole
        // system is built on — gives every building the SAME first answer.
        // Measured: with raw seeding, forty buildings on a street and forty
        // different districts all chose the identical palette, because the
        // first unit() landed in the same bucket every time. A finalizer mix
        // costs three multiplies once per RNG and makes independent streams
        // actually independent.
        std::uint32_t h = seed ? seed : 0x2545f491u;
        h ^= h >> 16;
        h *= 0x7feb352du;
        h ^= h >> 15;
        h *= 0x846ca68bu;
        h ^= h >> 16;
        s = h ? h : 0x2545f491u;
    }

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
