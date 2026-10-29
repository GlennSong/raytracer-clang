#ifndef RAYTRACER_PROC_RNG_H
#define RAYTRACER_PROC_RNG_H

// THE ENGINE'S ONE SEEDED RANDOM STREAM (ADR-0265, #117 item 38): small, fast, deterministic on every host.
// <random>'s engines are fixed by the standard but its distributions are not, so the same seed grew a different tree
// on libc++ or MSVC than on libstdc++. One seed, one stream: a generator draws in a fixed order, so a seed always
// grows the same shape, on Linux, Mac and Windows alike. Procgen, audio, particles and the path tracer all draw from
// it. (The terrain's Perlin permutation alone keeps its own frozen shuffle: noise.cpp.) A fixed order needs one draw
// per expression: C++ leaves the order of `Vec2(r.next(), r.next())`'s arguments (or `a.next() + b.next()`'s operands)
// to the compiler. Clang, which builds the engine on all three, takes them left to right; ~100 older lines lean on
// that, and new code draws into named locals.

#include <cmath>
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
    int between(int a, int b) { return a + below(b - a + 1); }   // a .. b, both in
    uint32_t bits() { return static_cast<uint32_t>(next() * 4294967296.0); }   // 32 random bits
    uint32_t peek() const { ProcRng c = *this; return c.bits(); }   // the next bits(), without drawing them
};

// RANGES drawn from a ProcRng, called like <random>'s distributions (`jit(rng)`) so a generator reads the same.
template <typename T>
struct ProcUniform {   // [a, b)
    T a, b;
    ProcUniform(T a_, T b_) : a(a_), b(b_) {}
    T operator()(ProcRng& r) const {
        const T x = static_cast<T>(r.in(static_cast<double>(a), static_cast<double>(b)));
        return x < b ? x : a;   // (a float can round up onto b)
    }
};
template <typename T>
struct ProcUniformInt {   // [a, b], both ends in
    T a, b;
    ProcUniformInt(T a_, T b_) : a(a_), b(b_) {}
    T operator()(ProcRng& r) const {
        return a + static_cast<T>(r.next() * (static_cast<double>(b) - static_cast<double>(a) + 1.0));
    }
};
template <typename T>
struct ProcExponential {   // rate lambda: mean 1 / lambda
    T lambda;
    explicit ProcExponential(T l) : lambda(l) {}
    T operator()(ProcRng& r) const { return static_cast<T>(-std::log(1.0 - r.next()) / static_cast<double>(lambda)); }
};

template <typename T>
struct ProcPoisson {   // counts with mean `mean` (Knuth's product of uniforms: for the small means sound needs)
    double mean;
    explicit ProcPoisson(double m) : mean(m) {}
    T operator()(ProcRng& r) const {
        const double floor = std::exp(-mean);
        T k = 0;
        for (double p = r.next(); p > floor; p *= r.next()) ++k;
        return k;
    }
};

}  // namespace engine

#endif
