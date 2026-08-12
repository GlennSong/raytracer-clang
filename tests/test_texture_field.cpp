#include "test_framework.h"

#include "../src/engine/procgen/texture_field.h"

using namespace engine;

namespace {
// Average of channel 0 over a baked image (all channels equal for gray).
double meanR(const TextureData& t) {
    double s = 0;
    for (std::size_t i = 0; i < t.pixels.size(); i += t.channels) s += t.pixels[i];
    return s / (static_cast<double>(t.pixels.size()) / t.channels);
}
uint8_t at(const TextureData& t, int x, int y, int c) {
    return t.pixels[(static_cast<std::size_t>(y) * t.width + x) * t.channels + c];
}
}  // namespace

TEST_CASE(texture_field_brick_built_from_primitives) {
    // ADR-0043: a brick texture is *composed from primitives* (a brick lattice ×
    // fbm grit), not requested as a preset. The mask reads as bricks + mortar.
    TexField2 brick = fieldBrick(/*cols*/6, /*rows*/12, /*mortar*/0.12,
                              /*variation*/0.4, /*seed*/7);
    TexField2 grit = fieldScaleBias(fieldFbm(7, 8.0, 4), 0.25, 0.75);   // ~[0.75,1]
    TexField2 albedoMask = fieldMul(brick, grit);

    TextureData gray = bakeFieldGray(brick, 128);
    CHECK(gray.width == 128 && gray.channels == 3);

    // The pattern is not uniform: mortar (0) and bricks (~1) both present.
    double lo = 1e9, hi = -1e9;
    for (int y = 0; y < 128; ++y)
        for (int x = 0; x < 128; ++x) {
            double v = at(gray, x, y, 0);
            lo = std::min(lo, v); hi = std::max(hi, v);
        }
    CHECK(lo < 20);                 // mortar lines are dark
    CHECK(hi > 200);                // brick faces are bright
    CHECK(meanR(gray) > 120);       // mostly brick, some mortar

    // The combined albedo mask drives a two-colour bake (mortar grey -> brick red).
    TextureData albedo = bakeFieldColor(albedoMask, Vec3(0.6, 0.6, 0.58),
                                        Vec3(0.62, 0.20, 0.14), 96);
    CHECK(albedo.width == 96 && albedo.channels == 3);
    // Red channel dominates where brick shows; some pixel is clearly brick-red.
    bool sawRed = false;
    for (int y = 0; y < 96 && !sawRed; ++y)
        for (int x = 0; x < 96; ++x)
            if (at(albedo, x, y, 0) > at(albedo, x, y, 2) + 40) { sawRed = true; break; }
    CHECK(sawRed);
}

TEST_CASE(texture_field_is_deterministic_and_combinators_work) {
    TexField2 a = fieldFbm(3, 5.0, 4);
    CHECK_APPROX(bakeFieldGray(a, 32).pixels[0],
                 bakeFieldGray(fieldFbm(3, 5.0, 4), 32).pixels[0], 0);  // deterministic

    // mix(0,1, 0.5) == 0.5; clamp bounds; scale_bias maps.
    TexField2 half = fieldMix(fieldConstant(0.0), fieldConstant(1.0), 0.5);
    CHECK_APPROX(half(0.3, 0.7), 0.5, 1e-9);
    TexField2 cl = fieldClamp(fieldConstant(2.0), 0.0, 1.0);
    CHECK_APPROX(cl(0, 0), 1.0, 1e-9);
    TexField2 sb = fieldScaleBias(fieldConstant(0.5), 2.0, 0.1);
    CHECK_APPROX(sb(0, 0), 1.1, 1e-9);
}
