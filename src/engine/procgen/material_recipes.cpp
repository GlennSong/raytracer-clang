#include "material_recipes.h"
#include "texture_field.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace engine {

bool stoneKindFromName(const std::string& n, StoneKind& out) {
    if (n == "granite") out = StoneKind::Granite;
    else if (n == "sandstone") out = StoneKind::Sandstone;
    else if (n == "basalt") out = StoneKind::Basalt;
    else if (n == "masonry") out = StoneKind::Masonry;
    else return false;
    return true;
}

namespace {
Field2 signedOf(Field2 f) { return fieldScaleBias(std::move(f), 2.0, -1.0); }

struct Recipe { ColorField2 color; Field2 height; double relief; };

// GRANITE: speckled grains (dark and pale cells), broad tone, faint wandering cracks.
Recipe granite(uint32_t s) {
    const Field2 tone = fieldTileFbm(s, 3, 3), grain = fieldTileFbm(s + 1, 10, 3);
    const Field2 speckDark = fieldMul(fieldInvert(fieldSmoothstep(fieldCells(s + 2, 44), 0.12, 0.26)),
                                      fieldSmoothstep(fieldCellId(s + 2, 44), 0.55, 0.7));
    const Field2 speckPale = fieldMul(fieldInvert(fieldSmoothstep(fieldCells(s + 3, 36), 0.10, 0.22)),
                                      fieldSmoothstep(fieldCellId(s + 3, 36), 0.75, 0.85));
    const Field2 cracks = fieldInvert(fieldSmoothstep(
        fieldWarp(fieldCellEdges(s + 4, 3), fieldTileFbm(s + 5, 3, 3), fieldTileFbm(s + 6, 3, 3), 0.07), 0.0, 0.02));
    ColorField2 c = colorMul(colorConstant(Vec3(0.185, 0.178, 0.165)),
                             fieldAdd(fieldConstant(1.0), fieldAdd(fieldScaleBias(signedOf(tone), 0.12, 0.0), fieldScaleBias(signedOf(grain), 0.06, 0.0))));
    c = colorMixBy(c, colorConstant(Vec3(0.07, 0.07, 0.068)), fieldScaleBias(speckDark, 0.7, 0.0));
    c = colorMixBy(c, colorConstant(Vec3(0.42, 0.41, 0.39)), fieldScaleBias(speckPale, 0.6, 0.0));
    c = colorMul(c, fieldScaleBias(cracks, -0.45, 1.0));
    return {c, fieldAdd(fieldAdd(fieldScaleBias(tone, 0.5, 0.2), fieldScaleBias(grain, 0.3, 0.0)), fieldScaleBias(cracks, -0.5, 0.0)), 3.0};
}

// SANDSTONE: warm strata warped by slow noise, fine grain.
Recipe sandstone(uint32_t s) {
    const Field2 strata = fieldWarp(fieldBands(7), fieldConstant(0.5), fieldTileFbm(s, 2, 3), 0.07);
    const Field2 grain = fieldTileFbm(s + 1, 12, 3), tone = fieldTileFbm(s + 2, 3, 2);
    ColorField2 c = colorMixBy(colorConstant(Vec3(0.36, 0.25, 0.155)), colorConstant(Vec3(0.26, 0.17, 0.10)), strata);
    c = colorMul(c, fieldAdd(fieldConstant(1.0), fieldAdd(fieldScaleBias(signedOf(grain), 0.07, 0.0), fieldScaleBias(signedOf(tone), 0.1, 0.0))));
    return {c, fieldAdd(fieldScaleBias(strata, 0.5, 0.15), fieldScaleBias(grain, 0.3, 0.0)), 2.5};
}

// BASALT: dark, fine-grained, broken by columnar cracks.
Recipe basalt(uint32_t s) {
    // cracks that wander and break off (regular cells read as paving)
    const Field2 cols = fieldMul(fieldInvert(fieldSmoothstep(
                                     fieldWarp(fieldCellEdges(s, 3), fieldTileFbm(s + 7, 3, 3), fieldTileFbm(s + 8, 3, 3), 0.08), 0.0, 0.025)),
                                 fieldSmoothstep(fieldTileFbm(s + 9, 4, 2), 0.42, 0.6));
    const Field2 grain = fieldTileFbm(s + 1, 16, 3), tone = fieldTileFbm(s + 2, 3, 2);
    ColorField2 c = colorMul(colorConstant(Vec3(0.10, 0.095, 0.095)),
                             fieldAdd(fieldConstant(1.0), fieldAdd(fieldScaleBias(signedOf(grain), 0.1, 0.0), fieldScaleBias(signedOf(tone), 0.14, 0.0))));
    c = colorMul(c, fieldScaleBias(cols, -0.5, 1.0));
    return {c, fieldAdd(fieldScaleBias(grain, 0.3, 0.45), fieldScaleBias(cols, -0.6, 0.0)), 4.0};
}

// MASONRY: dressed limestone blocks in running bond (a tile is 3 x 6 of them), recessed joints,
// each block its own tone, fine grain, and darker weathering where water runs down a quay.
Recipe masonry(uint32_t s) {
    const Field2 blocks = fieldBrick(3, 6, 0.035, 0.22, s);   // 0.78..1 inside a block, 0 in a joint
    const Field2 inBlock = fieldSmoothstep(blocks, 0.0, 0.05);
    const Field2 grain = fieldTileFbm(s + 1, 14, 3), tone = fieldTileFbm(s + 2, 3, 2);
    const Field2 streaks = fieldWarp(fieldBands(9), fieldTileFbm(s + 3, 3, 2), fieldConstant(0.5), 0.12);
    ColorField2 c = colorMul(colorConstant(Vec3(0.30, 0.285, 0.26)),
                             fieldAdd(fieldConstant(1.0), fieldAdd(fieldScaleBias(signedOf(grain), 0.06, 0.0), fieldScaleBias(signedOf(tone), 0.08, 0.0))));
    c = colorMul(c, fieldScaleBias(blocks, 0.45, 0.55));   // each block's own tone, joints darkest
    c = colorMixBy(c, colorConstant(Vec3(0.13, 0.125, 0.11)), fieldScaleBias(fieldMul(fieldSmoothstep(streaks, 0.55, 0.9), fieldTileFbm(s + 4, 4, 3)), 0.45, 0.0));
    return {c, fieldAdd(fieldScaleBias(inBlock, 0.6, 0.1), fieldScaleBias(grain, 0.2, 0.0)), 3.0};
}

constexpr int kStoneVersion = 2;   // 2: darker granite, broken basalt cracks

// THE BARKS: a brightness (the vertex colour tints it) and a height.
struct BarkRecipe { Field2 bright; Field2 height; double relief; };

// FURROWED: continuous ridges running up the limb, parting and merging, with WIDE dark furrows between -- noise
// squeezed six times across the limb (features long along it), warped so the ridges wander, cut to ridge / furrow;
// a fine grain over the ridges, the furrows darkest at their floor.
BarkRecipe furrowed(uint32_t s) {
    // (two squeezes at co-prime counts, 5 and 7 across, so no column repeats the one beside it)
    const Field2 squeezed = fieldAdd(fieldScaleBias(fieldRepeat(fieldTileFbm(s, 3, 3), 5, 1), 0.5, 0.0),
                                     fieldScaleBias(fieldRepeat(fieldTileFbm(s + 9, 3, 3), 7, 1), 0.5, 0.0));
    const Field2 n = fieldWarp(squeezed, fieldTileFbm(s + 1, 3, 3), fieldTileFbm(s + 2, 2, 2), 0.04);
    const Field2 ridge = fieldSmoothstep(n, 0.40, 0.62), fine = fieldTileFbm(s + 3, 14, 3);
    const Field2 floor = fieldSmoothstep(n, 0.25, 0.45);
    return {fieldAdd(fieldScaleBias(ridge, 0.42, 0.30), fieldAdd(fieldScaleBias(floor, 0.18, 0.0), fieldScaleBias(fine, 0.12, 0.0))),
            fieldAdd(fieldScaleBias(ridge, 0.7, 0.0), fieldAdd(fieldScaleBias(floor, 0.15, 0.0), fieldScaleBias(fine, 0.15, 0.0))), 7.0};
}
// PLATED: flat plates a little wider than tall, each its own tone, flaking at its edges, thin cracks between.
BarkRecipe plated(uint32_t s) {
    const Field2 edges = fieldWarp(fieldRepeat(fieldCellEdges(s, 5), 1, 2), fieldTileFbm(s + 1, 4, 2), fieldTileFbm(s + 2, 4, 2), 0.02);
    const Field2 plate = fieldSmoothstep(edges, 0.0, 0.07), tone = fieldRepeat(fieldCellId(s, 5), 1, 2);
    const Field2 flake = fieldTileFbm(s + 3, 12, 3);
    return {fieldAdd(fieldScaleBias(plate, 0.45, 0.3), fieldAdd(fieldScaleBias(tone, 0.18, 0.0), fieldScaleBias(flake, 0.1, 0.0))),
            fieldAdd(fieldScaleBias(plate, 0.65, 0.0), fieldAdd(fieldScaleBias(tone, 0.15, 0.0), fieldScaleBias(flake, 0.2, 0.0))), 4.5};
}
// SMOOTH: a thin skin, bright, with a soft mottle and a few faint wrinkles across it that break off.
BarkRecipe smooth(uint32_t s) {
    const Field2 mottle = fieldTileFbm(s, 3, 3);
    const Field2 wrinkle = fieldMul(fieldSmoothstep(fieldRepeat(fieldTileNoise(s + 1, 5), 1, 6), 0.72, 0.85),
                                    fieldSmoothstep(fieldTileFbm(s + 2, 4, 2), 0.5, 0.7));
    return {fieldAdd(fieldScaleBias(mottle, 0.16, 0.78), fieldScaleBias(wrinkle, -0.10, 0.0)),
            fieldAdd(fieldScaleBias(mottle, 0.3, 0.0), fieldScaleBias(wrinkle, 0.15, 0.0)), 1.2};
}
// PAPERY: white; LENTICELS -- dark dashes running round the trunk (noise squeezed along it, so its features lie
// across it, cut to its darkest), long and short; black patches where the noise's slow octave runs low; peeling bands
// paler and lifted.
BarkRecipe papery(uint32_t s) {
    // (many cells across the trunk -- a dash a few centimetres long -- squeezed along it so each is a thin line)
    const Field2 dash = fieldSmoothstep(fieldRepeat(fieldTileNoise(s, 18), 1, 7), 0.83, 0.89);
    const Field2 dash2 = fieldSmoothstep(fieldRepeat(fieldTileNoise(s + 6, 26), 1, 9), 0.87, 0.92);
    const Field2 patch = fieldInvert(fieldSmoothstep(fieldWarp(fieldTileFbm(s + 1, 2, 3), fieldTileFbm(s + 2, 4, 2), fieldTileFbm(s + 3, 4, 2), 0.06), 0.24, 0.30));
    const Field2 peel = fieldSmoothstep(fieldWarp(fieldRepeat(fieldTileNoise(s + 4, 3), 1, 5), fieldTileFbm(s + 5, 3, 2), fieldConstant(0.5), 0.08), 0.7, 0.8);
    const Field2 tone = fieldTileFbm(s + 7, 3, 3);
    Field2 b = fieldAdd(fieldScaleBias(tone, 0.08, 0.86), fieldScaleBias(peel, 0.06, 0.0));
    b = fieldMul(b, fieldScaleBias(fieldMax(dash, dash2), -0.8, 1.0));
    b = fieldMul(b, fieldScaleBias(patch, -0.85, 1.0));
    return {b, fieldAdd(fieldScaleBias(peel, 0.4, 0.3), fieldScaleBias(fieldMax(dash, dash2), -0.3, 0.0)), 2.0};
}
// RINGED: sharp rings round the limb, long fibres along it between.
BarkRecipe ringed(uint32_t s) {
    const Field2 rings = fieldSmoothstep(fieldWarp(fieldBands(12), fieldTileFbm(s, 4, 2), fieldConstant(0.5), 0.03), 0.82, 0.95);
    const Field2 fibre = fieldRepeat(fieldTileFbm(s + 1, 6, 3), 6, 1);
    return {fieldAdd(fieldScaleBias(fibre, 0.22, 0.6), fieldScaleBias(rings, -0.3, 0.0)),
            fieldAdd(fieldScaleBias(fibre, 0.4, 0.2), fieldScaleBias(rings, 0.5, 0.0)), 3.0};
}

constexpr int kBarkVersion = 4;   // 2: furrows, not scales; dashes, not dots
}  // namespace

bool barkKindFromName(const std::string& n, BarkKind& out) {
    if (n == "furrowed" || n == "oak") out = BarkKind::Furrowed;
    else if (n == "plated" || n == "pine" || n == "spruce") out = BarkKind::Plated;
    else if (n == "smooth" || n == "beech" || n == "aspen") out = BarkKind::Smooth;
    else if (n == "papery" || n == "birch") out = BarkKind::Papery;
    else if (n == "ringed" || n == "palm" || n == "cherry") out = BarkKind::Ringed;
    else return false;
    return true;
}

BarkTextures barkTextures(BarkKind kind, uint32_t seed, int size) {
    const uint32_t s = seed * 7451u + static_cast<uint32_t>(kind) * 92821u + 3u;
    const BarkRecipe r = kind == BarkKind::Plated ? plated(s) : kind == BarkKind::Smooth ? smooth(s)
                       : kind == BarkKind::Papery ? papery(s) : kind == BarkKind::Ringed ? ringed(s) : furrowed(s);
    char key[96];
    BarkTextures t;
    std::snprintf(key, sizeof key, "bark/v%d/%d/%u/%d/albedo", kBarkVersion, static_cast<int>(kind), seed, size);
    const Field2 b = fieldClamp(r.bright, 0.0, 1.0);
    t.albedo = bakeCached(key, [&] { return bakeFieldRGBA([b](double u, double v) { const double x = b(u, v); return Vec3(x, x, x); },
                                                           fieldConstant(1.0), size, false); });
    std::snprintf(key, sizeof key, "bark/v%d/%d/%u/%d/normal", kBarkVersion, static_cast<int>(kind), seed, size);
    t.normal = bakeCached(key, [&] { return bakeFieldNormal(r.height, r.relief, size); });
    return t;
}

BarkTextures bladeTextures(uint32_t seed, int size) {
    constexpr int kBladeVersion = 1;
    const Field2 veins = fieldMix(fieldStripes(23.0), fieldStripes(9.0), 0.35);
    const Field2 mid = [](double u, double) { const double d = std::fabs(u - 0.5); return d < 0.035 ? 1.0 - d / 0.035 : 0.0; };
    const Field2 edge = [](double u, double) { const double d = std::min(u, 1.0 - u); return d < 0.08 ? 1.0 - d / 0.08 : 0.0; };
    const Field2 mottle = fieldRepeat(fieldTileFbm(seed + 3u, 4, 3), 1, 3);
    const Field2 bright = [veins, mid, edge, mottle](double u, double v) {
        return 0.80 + 0.10 * veins(u, v) + 0.14 * mid(u, v) - 0.12 * edge(u, v) + 0.10 * (mottle(u, v) - 0.5);
    };
    const Field2 height = [veins, mid](double u, double v) { return 0.6 * veins(u, v) + 0.8 * mid(u, v); };
    char key[96];
    BarkTextures t;
    std::snprintf(key, sizeof key, "blade/v%d/%u/%d/albedo", kBladeVersion, seed, size);
    const Field2 b = fieldClamp(bright, 0.0, 1.0);
    t.albedo = bakeCached(key, [&] { return bakeFieldRGBA([b](double u, double v) { const double x = b(u, v); return Vec3(x, x, x); },
                                                           fieldConstant(1.0), size, false); });
    std::snprintf(key, sizeof key, "blade/v%d/%u/%d/normal", kBladeVersion, seed, size);
    t.normal = bakeCached(key, [&] { return bakeFieldNormal(height, 2.0, size); });
    return t;
}

StoneTextures stoneTextures(StoneKind kind, uint32_t seed, int size) {
    const uint32_t s = seed * 6007u + static_cast<uint32_t>(kind) * 31337u;
    const Recipe r = kind == StoneKind::Sandstone ? sandstone(s) : kind == StoneKind::Basalt ? basalt(s)
                   : kind == StoneKind::Masonry ? masonry(s) : granite(s);
    char key[96];
    StoneTextures t;
    std::snprintf(key, sizeof key, "stone/v%d/%d/%u/%d/albedo", kStoneVersion, static_cast<int>(kind), seed, size);
    t.albedo = bakeCached(key, [&] { return bakeFieldRGBA(r.color, r.height, size, true); });
    std::snprintf(key, sizeof key, "stone/v%d/%d/%u/%d/normal", kStoneVersion, static_cast<int>(kind), seed, size);
    t.normal = bakeCached(key, [&] { return bakeFieldNormal(r.height, r.relief, size); });
    return t;
}

}  // namespace engine
