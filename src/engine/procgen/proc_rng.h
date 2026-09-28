#ifndef RAYTRACER_ENGINE_PROCGEN_PROC_RNG_H
#define RAYTRACER_ENGINE_PROCGEN_PROC_RNG_H

// The procedural generators' shared seeded random stream: small, fast, deterministic on every
// host (no <random> distributions, whose output differs between standard libraries). One seed,
// one stream: generators draw in a fixed order, so a seed always grows the same shape.

#include <cstdint>

namespace engine {

struct ProcRng {
    uint64_t s;
    explicit ProcRng(uint32_t seed, uint64_t salt = 0x9E3779B97F4A7C15ull)
        : s(salt ^ (static_cast<uint64_t>(seed) * 0xBF58476D1CE4E5B9ull + 0x94D049BB133111EBull)) {
        next(); next();
    }
    double next() {   // [0, 1)
        s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
        return static_cast<double>((s * 0x2545F4914F6CDD1Dull) >> 11) * (1.0 / 9007199254740992.0);
    }
    double in(double a, double b) { return a + (b - a) * next(); }
    int below(int n) { return static_cast<int>(next() * n); }   // 0 .. n-1
};

}  // namespace engine

#endif
