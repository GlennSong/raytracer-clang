#include "ground_layers.h"
#include "texture_field.h"

#include <cstdio>

namespace engine {

namespace {

// A field centred on zero: [0, 1] -> [-1, 1] (the recipes below think in +/- variation).
Field2 signedOf(Field2 f) { return fieldScaleBias(std::move(f), 2.0, -1.0); }
// 1 + sum of weighted signed fields: a brightness multiplier.
Field2 brightness(std::initializer_list<std::pair<Field2, double>> terms) {
    Field2 acc = fieldConstant(1.0);
    for (const auto& [f, w] : terms) acc = fieldAdd(acc, fieldScaleBias(signedOf(f), w, 0.0));
    return acc;
}

// THE RECIPES: each layer composed from texture_field primitives (tileable noise, cells, bands,
// warps), not private code -- the same vocabulary a Lua recipe uses.
struct Recipe { ColorField2 color; Field2 height; };

Recipe grassRecipe(const Vec3& base, uint32_t s) {
    const Field2 blot = fieldTileFbm(s, 3, 3);            // soft painted blotches
    const Field2 warm = fieldTileFbm(s + 7, 2, 2);         // a warmer / cooler drift
    const Field2 stroke = fieldMix(fieldTileNoise(s + 13, 24), fieldTileNoise(s + 17, 48), 0.5);   // short strokes
    const ColorField2 hue = colorMixBy(colorConstant(Vec3(base.x * 0.82, base.y, base.z * 1.15)),
                                       colorConstant(Vec3(base.x * 1.18, base.y, base.z * 0.85)), warm);
    return {colorMul(hue, brightness({{blot, 0.16}, {stroke, 0.07}})),
            fieldAdd(fieldScaleBias(blot, 0.5, 0.2), fieldScaleBias(stroke, 0.4, 0.05))};
}

Recipe dirtRecipe(const Vec3& base, uint32_t s) {
    const Field2 blot = fieldTileFbm(s, 4, 3);
    const Field2 pebble = fieldInvert(fieldSmoothstep(fieldCells(s + 3, 22), 0.18, 0.34));   // a pebble's body
    const Field2 crevice = fieldMul(fieldInvert(fieldSmoothstep(fieldCellEdges(s + 3, 22), 0.0, 0.08)), fieldInvert(pebble));
    const ColorField2 earth = colorMul(colorConstant(base), fieldMul(brightness({{blot, 0.14}}), fieldScaleBias(crevice, -0.1, 1.0)));
    return {colorMixBy(earth, colorConstant(Vec3(0.20, 0.18, 0.15)), fieldScaleBias(pebble, 0.55, 0.0)),   // pale stones
            fieldAdd(fieldScaleBias(blot, 0.4, 0.1), fieldScaleBias(pebble, 0.5, 0.0))};
}

Recipe sandRecipe(const Vec3& base, uint32_t s) {
    const Field2 ripple = fieldWarp(fieldBands(11), fieldConstant(0.5), fieldTileFbm(s, 3, 2), 0.08);   // wind ripples
    const Field2 blot = fieldTileFbm(s + 5, 4, 3);
    return {colorMul(colorConstant(base), brightness({{ripple, 0.05}, {blot, 0.08}})),
            fieldAdd(fieldScaleBias(ripple, 0.25, 0.28), fieldScaleBias(blot, 0.5, 0.0))};
}

Recipe rockRecipe(const Vec3& base, uint32_t s) {
    // a few big slabs whose cracks WANDER (the cell lookup domain-warped) and break off
    const Field2 edges = fieldWarp(fieldCellEdges(s + 11, 3), fieldTileFbm(s + 29, 3, 3), fieldTileFbm(s + 31, 3, 3), 0.06);
    const Field2 crack = fieldMul(fieldInvert(fieldSmoothstep(edges, 0.0, 0.035)), fieldSmoothstep(fieldTileFbm(s + 37, 4, 2), 0.4, 0.65));
    const Field2 grain = fieldTileFbm(s + 19, 6, 4);
    const Field2 strata = fieldWarp(fieldBands(6), fieldConstant(0.5), fieldTileFbm(s + 23, 2, 2), 0.06);
    const Field2 tone = fieldTileFbm(s + 41, 2, 3);
    return {colorMul(colorConstant(base), fieldMul(brightness({{grain, 0.16}, {strata, 0.05}, {tone, 0.14}}), fieldScaleBias(crack, -0.45, 1.0))),
            fieldAdd(fieldScaleBias(grain, 0.4, 0.55), fieldScaleBias(crack, -0.6, 0.0))};
}

constexpr int kRecipeVersion = 3;   // 3: recipes over texture_field (was private noise)

}  // namespace

TextureData groundLayerTexture(GroundLayer layer, const Vec3& base, int size, uint32_t seed) {
    const uint32_t s = seed * 7919u + static_cast<uint32_t>(layer) * 104729u;
    Recipe r;
    switch (layer) {
        case GroundLayer::Grass: r = grassRecipe(base, s); break;
        case GroundLayer::Dirt:  r = dirtRecipe(base, s); break;
        case GroundLayer::Sand:  r = sandRecipe(base, s); break;
        case GroundLayer::Rock:  r = rockRecipe(base, s); break;
    }
    return bakeFieldRGBA(r.color, r.height, size, /*gamma=*/true);
}

TextureData groundLayerTextureCached(GroundLayer layer, const Vec3& base, int size, uint32_t seed) {
    char key[160];
    std::snprintf(key, sizeof key, "groundLayer/v%d/L%d/%d/%u/%.5f,%.5f,%.5f", kRecipeVersion, static_cast<int>(layer), size, seed,
                  static_cast<double>(base.x), static_cast<double>(base.y), static_cast<double>(base.z));
    return bakeCached(key, [&] { return groundLayerTexture(layer, base, size, seed); });
}

}  // namespace engine
