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
    // its height (ADR-0207, bumped by the surface relief): the sward's TUSSOCKS -- round clumps, some cells only -- on a
    // smooth fine roll, not the strokes' value noise (it bumped as square grain)
    const Field2 du = fieldTileFbm(s + 71, 6, 2), dv = fieldTileFbm(s + 73, 6, 2);
    const Field2 tussock = fieldMul(fieldInvert(fieldSmoothstep(fieldWarp(fieldCells(s + 61, 12), du, dv, 0.04), 0.0, 0.6)),
                                    fieldSmoothstep(fieldWarp(fieldCellId(s + 61, 12), du, dv, 0.04), 0.35, 0.37));
    const Field2 roll = fieldTileFbm(s + 67, 16, 3);
    return {colorMul(hue, brightness({{blot, 0.16}, {stroke, 0.07}, {tussock, 0.05}})),
            fieldAdd(fieldAdd(fieldScaleBias(blot, 0.3, 0.1), fieldScaleBias(tussock, 0.45, 0.0)), fieldScaleBias(roll, 0.2, 0.0))};
}

Recipe dirtRecipe(const Vec3& base, uint32_t s) {
    // (ADR-0207: the surface relief bumps this height, and one size of pale pebble in every cell read as polka dots)
    const Field2 blot = fieldTileFbm(s, 4, 3);
    const Field2 crumbs = fieldTileFbm(s + 5, 48, 2);                                          // soil crumbs, ~8 cm
    const Field2 du = fieldTileFbm(s + 7, 6, 2), dv = fieldTileFbm(s + 8, 6, 2);
    // STONES OF TWO SIZES, in some cells only, out of round (the cell lookup warped), and gathered in patches
    auto stones = [&](uint32_t seed, int period, double keep, double r0, double r1) {
        const Field2 body = fieldInvert(fieldSmoothstep(fieldWarp(fieldCells(seed, period), du, dv, 0.03), r0, r1));
        return fieldMul(body, fieldSmoothstep(fieldWarp(fieldCellId(seed, period), du, dv, 0.03), 1.0 - keep, 1.0 - keep + 0.02));
    };
    const Field2 patches = fieldSmoothstep(fieldTileFbm(s + 21, 3, 2), 0.40, 0.62);
    const Field2 pebble = fieldMul(fieldMax(stones(s + 3, 14, 0.45, 0.14, 0.30), fieldScaleBias(stones(s + 9, 36, 0.40, 0.16, 0.30), 0.8, 0.0)),
                                   fieldScaleBias(patches, 0.85, 0.15));
    const Field2 tone = fieldWarp(fieldCellId(s + 13, 14), du, dv, 0.03);                      // each stone its own shade
    const ColorField2 earth = colorMul(colorConstant(base), brightness({{blot, 0.14}, {crumbs, 0.10}}));
    const ColorField2 stone = colorMixBy(colorConstant(Vec3(0.060, 0.056, 0.050)), colorConstant(Vec3(0.14, 0.128, 0.11)), tone);
    return {colorMixBy(earth, stone, fieldScaleBias(pebble, 0.85, 0.0)),
            fieldAdd(fieldAdd(fieldScaleBias(blot, 0.35, 0.1), fieldScaleBias(crumbs, 0.15, 0.0)), fieldScaleBias(pebble, 0.5, 0.0))};
}

Recipe sandRecipe(const Vec3& base, uint32_t s) {
    const Field2 ripple = fieldWarp(fieldBands(11), fieldConstant(0.5), fieldTileFbm(s, 3, 2), 0.08);   // wind ripples
    const Field2 blot = fieldTileFbm(s + 5, 4, 3);
    return {colorMul(colorConstant(base), brightness({{ripple, 0.05}, {blot, 0.08}})),
            fieldAdd(fieldScaleBias(ripple, 0.25, 0.28), fieldScaleBias(blot, 0.5, 0.0))};
}

Recipe rockRecipe(const Vec3& base, uint32_t s) {
    // a few big slabs whose cracks WANDER (the cell lookup domain-warped) and break off
    const Field2 wu = fieldTileFbm(s + 29, 3, 3), wv = fieldTileFbm(s + 31, 3, 3);
    const Field2 edges = fieldWarp(fieldCellEdges(s + 11, 3), wu, wv, 0.06);
    const Field2 crack = fieldMul(fieldInvert(fieldSmoothstep(edges, 0.0, 0.035)), fieldSmoothstep(fieldTileFbm(s + 37, 4, 2), 0.4, 0.65));
    // (ADR-0207: lit by its height, the stone between the cracks was smooth plaster) each slab at its own height, so a
    // crack steps; grain at three scales down to ~4 cm; mineral flecks; lichen in patches
    // (eased to nothing toward its edges: a hard step bumped as a dotted seam a pixel wide)
    const Field2 slab = fieldMul(fieldWarp(fieldCellId(s + 11, 3), wu, wv, 0.06), fieldSmoothstep(edges, 0.0, 0.08));
    const Field2 grain = fieldTileFbm(s + 19, 6, 4);
    const Field2 fine = fieldTileFbm(s + 43, 24, 3);
    const Field2 fleck = fieldSmoothstep(fieldTileNoise(s + 47, 96), 0.72, 0.82);
    const Field2 lichen = fieldMul(fieldSmoothstep(fieldTileFbm(s + 53, 8, 3), 0.66, 0.74), fieldInvert(crack));
    const Field2 strata = fieldWarp(fieldBands(6), fieldConstant(0.5), fieldTileFbm(s + 23, 2, 2), 0.06);
    const Field2 tone = fieldTileFbm(s + 41, 2, 3);
    const ColorField2 stone = colorMul(colorConstant(base), fieldMul(brightness({{grain, 0.16}, {fine, 0.10}, {strata, 0.05}, {tone, 0.14},
                                                                                  {slab, 0.08}, {fleck, 0.12}}),
                                                                       fieldScaleBias(crack, -0.45, 1.0)));
    const ColorField2 lichenColor = colorMixBy(colorConstant(Vec3(base.x * 1.35, base.y * 1.42, base.z * 1.25)),   // pale grey-green
                                               colorConstant(Vec3(base.x * 1.4, base.y * 1.25, base.z * 0.95)), fieldTileFbm(s + 59, 8, 2));
    return {colorMixBy(stone, lichenColor, fieldScaleBias(lichen, 0.45, 0.0)),
            fieldAdd(fieldAdd(fieldAdd(fieldScaleBias(grain, 0.25, 0.35), fieldScaleBias(fine, 0.18, 0.0)), fieldScaleBias(slab, 0.2, 0.0)),
                     fieldScaleBias(crack, -0.6, 0.0))};
}

constexpr int kRecipeVersion = 4;   // 3: recipes over texture_field (was private noise); 4: stones and slabs (ADR-0207)

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
