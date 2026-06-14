#include "test_framework.h"

#include "../src/engine/procgen/noise.h"
#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(noise_is_deterministic_for_a_seed) {
    Noise a(1234), b(1234);
    CHECK_APPROX(a.noise3(1.5, 2.5, 3.5), b.noise3(1.5, 2.5, 3.5), 1e-12);
    CHECK_APPROX(a.fbm2(0.3, 0.7, 5), b.fbm2(0.3, 0.7, 5), 1e-12);
}

TEST_CASE(noise_differs_by_seed) {
    Noise a(1), b(2);
    // Vanishingly unlikely to collide across several samples if seeds differ.
    bool anyDifferent = false;
    for (int i = 0; i < 8; i++) {
        double x = i * 0.37 + 0.1;
        if (std::fabs(a.noise2(x, x) - b.noise2(x, x)) > 1e-9) anyDifferent = true;
    }
    CHECK(anyDifferent);
}

TEST_CASE(noise_stays_in_unit_range) {
    Noise n(42);
    double maxAbs = 0.0;
    for (int i = 0; i < 2000; i++) {
        double x = i * 0.123, y = i * 0.077, z = i * 0.211;
        maxAbs = std::max(maxAbs, std::fabs(n.noise3(x, y, z)));
        maxAbs = std::max(maxAbs, std::fabs(n.fbm3(x, y, z, 5)));
    }
    CHECK(maxAbs <= 1.05);   // gradient noise + normalized fbm are bounded ~[-1,1]
}

TEST_CASE(noise_is_continuous) {
    // A tiny step in the input produces a tiny change in the output (no jumps),
    // the defining property of coherent noise vs. white noise.
    Noise n(7);
    double a = n.noise2(3.0, 3.0);
    double b = n.noise2(3.0001, 3.0);
    CHECK(std::fabs(a - b) < 0.01);
}

TEST_CASE(noise_at_integer_lattice_is_near_zero) {
    // Perlin gradient noise vanishes at integer lattice points.
    Noise n(99);
    CHECK_APPROX(n.noise3(4.0, 7.0, 2.0), 0.0, 1e-9);
}

TEST_CASE(noise2_matches_noise3_zero_slice) {
    Noise n(5);
    CHECK_APPROX(n.noise2(2.3, 4.1), n.noise3(2.3, 4.1, 0.0), 1e-12);
}
