#include "material_recipes.h"
#include "texture_field.h"

#include <cstdio>

namespace engine {

bool stoneKindFromName(const std::string& n, StoneKind& out) {
    if (n == "granite") out = StoneKind::Granite;
    else if (n == "sandstone") out = StoneKind::Sandstone;
    else if (n == "basalt") out = StoneKind::Basalt;
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

constexpr int kStoneVersion = 2;   // 2: darker granite, broken basalt cracks
}  // namespace

StoneTextures stoneTextures(StoneKind kind, uint32_t seed, int size) {
    const uint32_t s = seed * 6007u + static_cast<uint32_t>(kind) * 31337u;
    const Recipe r = kind == StoneKind::Sandstone ? sandstone(s) : kind == StoneKind::Basalt ? basalt(s) : granite(s);
    char key[96];
    StoneTextures t;
    std::snprintf(key, sizeof key, "stone/v%d/%d/%u/%d/albedo", kStoneVersion, static_cast<int>(kind), seed, size);
    t.albedo = bakeCached(key, [&] { return bakeFieldRGBA(r.color, r.height, size, true); });
    std::snprintf(key, sizeof key, "stone/v%d/%d/%u/%d/normal", kStoneVersion, static_cast<int>(kind), seed, size);
    t.normal = bakeCached(key, [&] { return bakeFieldNormal(r.height, r.relief, size); });
    return t;
}

}  // namespace engine
