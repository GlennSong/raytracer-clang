#include "ground_layers.h"

#include <algorithm>
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

// THE ROCK'S SUB-LAYERS (ADR-0210; Glenn: "break it up into sublayers"): one rock everywhere read as one tile repeated
// over every mountain. Four, each a recipe, which mesh.frag chooses among by the ground (layerRock):
//   FACE    a cliff's compact stone: broad gentle planes, a few long fractures, streaks running down (v is up on a
//           cliff's triplanar faces), fine grain
//   BROKEN  blocky rock: angular blocks of their own heights and tones, deep gaps between
//   SCREE   talus: packed angular stones a hand to a forearm across, dark gaps, gravel between
//   LICHEN  the face's stone patched with lichen and moss, where it is damp or turned from the sun
Recipe rockFaceRecipe(const Vec3& base, uint32_t s, bool lichenOn) {
    const Field2 wu = fieldTileFbm(s + 29, 3, 3), wv = fieldTileFbm(s + 31, 3, 3);
    const Field2 planes = fieldTileFbm(s + 1, 2, 2);
    const Field2 grain = fieldTileFbm(s + 19, 12, 3);
    const Field2 fine = fieldTileFbm(s + 43, 32, 2);
    const Field2 edges = fieldWarp(fieldCellEdges(s + 11, 2), wu, wv, 0.12);
    const Field2 crack = fieldMul(fieldInvert(fieldSmoothstep(edges, 0.0, 0.02)), fieldSmoothstep(fieldTileFbm(s + 37, 2, 2), 0.62, 0.74));
    const Field2 streak = fieldSmoothstep(fieldWarp(fieldStripes(7), fieldTileFbm(s + 61, 5, 3), fieldConstant(0.5), 0.14), 0.66, 0.94);
    const Field2 streakMask = fieldSmoothstep(fieldTileFbm(s + 63, 3, 2), 0.45, 0.65);
    ColorField2 stone = colorMul(colorConstant(base), fieldMul(fieldMul(brightness({{planes, 0.12}, {grain, 0.10}, {fine, 0.06}}),
                                                                         fieldScaleBias(crack, -0.4, 1.0)),
                                                                fieldScaleBias(fieldMul(streak, streakMask), -0.22, 1.0)));
    Field2 height = fieldAdd(fieldAdd(fieldScaleBias(planes, 0.35, 0.25), fieldScaleBias(grain, 0.2, 0.0)),
                             fieldAdd(fieldScaleBias(fine, 0.1, 0.0), fieldScaleBias(crack, -0.3, 0.0)));
    if (lichenOn) {
        // (lichen in small crusts, moss in the low places; muted: the first, saturated blotches read as camouflage)
        const Field2 lichen = fieldMul(fieldSmoothstep(fieldTileFbm(s + 53, 14, 3), 0.52, 0.62), fieldInvert(crack));
        const Field2 moss = fieldMul(fieldSmoothstep(fieldTileFbm(s + 67, 6, 3), 0.58, 0.7), fieldInvert(planes));
        const ColorField2 lichenColor = colorMixBy(colorConstant(Vec3(base.x * 1.3, base.y * 1.36, base.z * 1.22)),   // pale grey-green
                                                   colorConstant(Vec3(base.x * 1.4, base.y * 1.28, base.z * 1.0)), fieldTileFbm(s + 59, 8, 2));
        stone = colorMixBy(colorMixBy(stone, lichenColor, fieldScaleBias(lichen, 0.6, 0.0)),
                           colorConstant(Vec3(base.x * 0.62, base.y * 0.78, base.z * 0.52)), fieldScaleBias(moss, 0.6, 0.0));   // moss
        height = fieldAdd(height, fieldScaleBias(fieldMax(lichen, moss), 0.08, 0.0));
    }
    return {stone, height};
}

Recipe rockBrokenRecipe(const Vec3& base, uint32_t s) {
    // (blocks of one size in one cell grid read as paving: two sizes, mixed by region; the gaps of varied width)
    const Field2 wu = fieldTileFbm(s + 29, 3, 3), wv = fieldTileFbm(s + 31, 3, 3);
    const Field2 region = fieldSmoothstep(fieldTileFbm(s + 71, 2, 2), 0.4, 0.6);
    const Field2 edges = fieldMixBy(fieldWarp(fieldCellEdges(s + 3, 4), wu, wv, 0.07), fieldWarp(fieldCellEdges(s + 4, 9), wu, wv, 0.05), region);
    const Field2 id = fieldMixBy(fieldWarp(fieldCellId(s + 3, 4), wu, wv, 0.07), fieldWarp(fieldCellId(s + 4, 9), wu, wv, 0.05), region);
    const Field2 grain = fieldTileFbm(s + 19, 12, 3);
    const Field2 gapW = fieldScaleBias(fieldTileFbm(s + 73, 4, 2), 0.1, 0.02);
    // (soft, shallow gaps: dark, deep ones read as a net of lines over every face -- Glenn, 2026-10-20)
    const Field2 body = fieldSmoothstep(fieldAdd(edges, fieldScaleBias(gapW, -1.0, 0.07)), 0.0, 0.12);   // 0 in a gap, 1 on a block
    return {colorMul(colorConstant(base), fieldMul(brightness({{id, 0.16}, {grain, 0.12}}), fieldScaleBias(body, 0.22, 0.78))),
            fieldAdd(fieldMul(fieldScaleBias(id, 0.3, 0.45), fieldScaleBias(body, 0.5, 0.5)), fieldScaleBias(grain, 0.15, 0.0))};
}

Recipe screeRecipe(const Vec3& base, uint32_t s) {
    const Field2 wu = fieldTileFbm(s + 29, 4, 2), wv = fieldTileFbm(s + 31, 4, 2);
    const Field2 region = fieldSmoothstep(fieldTileFbm(s + 75, 3, 2), 0.4, 0.6);   // (two sizes of stone, by region)
    const Field2 edges = fieldMixBy(fieldWarp(fieldCellEdges(s + 5, 14), wu, wv, 0.03), fieldWarp(fieldCellEdges(s + 6, 26), wu, wv, 0.03), region);
    const Field2 id = fieldMixBy(fieldWarp(fieldCellId(s + 5, 14), wu, wv, 0.03), fieldWarp(fieldCellId(s + 6, 26), wu, wv, 0.03), region);
    const Field2 body = fieldSmoothstep(edges, 0.0, 0.07);
    const Field2 gravel = fieldSmoothstep(fieldCellEdges(s + 7, 44), 0.0, 0.1);
    const Field2 grain = fieldTileFbm(s + 19, 24, 2);
    const ColorField2 stoneC = colorMul(colorConstant(base), brightness({{id, 0.25}, {grain, 0.08}}));
    const ColorField2 gapC = colorMul(colorConstant(base), fieldScaleBias(gravel, 0.25, 0.35));
    return {colorMixBy(gapC, stoneC, body),
            fieldAdd(fieldMul(fieldScaleBias(id, 0.4, 0.4), body), fieldScaleBias(gravel, 0.12, 0.0))};
}

constexpr int kRecipeVersion = 7;   // 3: recipes over texture_field; 4: stones and slabs (ADR-0207); 6: the rock's sub-layers (ADR-0210)

}  // namespace

TextureData groundLayerTexture(GroundLayer layer, const Vec3& base, int size, uint32_t seed) {
    const uint32_t s = seed * 7919u + static_cast<uint32_t>(layer) * 104729u;
    Recipe r;
    switch (layer) {
        case GroundLayer::Grass: r = grassRecipe(base, s); break;
        case GroundLayer::Dirt:  r = dirtRecipe(base, s); break;
        case GroundLayer::Sand:  r = sandRecipe(base, s); break;
        case GroundLayer::Rock:  r = rockFaceRecipe(base, s, false); break;
        case GroundLayer::RockBroken: r = rockBrokenRecipe(base, s); break;
        case GroundLayer::Scree: r = screeRecipe(base, s); break;
        case GroundLayer::RockLichen: r = rockFaceRecipe(base, s, true); break;
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

namespace engine {
TextureData groundRockAtlasCached(const Vec3& base, int size, uint32_t seed) {
    // the four sub-layers, each size^2, in a 2x2 atlas: face (0, 0), broken (1, 0), scree (0, 1), lichen (1, 1)
    const GroundLayer order[4] = {GroundLayer::Rock, GroundLayer::RockBroken, GroundLayer::Scree, GroundLayer::RockLichen};
    TextureData atlas;
    atlas.width = atlas.height = 2 * size;
    atlas.channels = 4;
    atlas.pixels.assign(static_cast<std::size_t>(4) * size * size * 4, 0);
    for (int k = 0; k < 4; ++k) {
        const TextureData t = groundLayerTextureCached(order[k], base, size, seed);
        const int ox = (k % 2) * size, oy = (k / 2) * size;
        for (int y = 0; y < size; ++y)
            std::copy(t.pixels.begin() + static_cast<std::ptrdiff_t>(y) * size * 4, t.pixels.begin() + static_cast<std::ptrdiff_t>(y + 1) * size * 4,
                      atlas.pixels.begin() + (static_cast<std::ptrdiff_t>(oy + y) * 2 * size + ox) * 4);
    }
    return atlas;
}
}  // namespace engine
