#ifndef RAYTRACER_ENGINE_PROCGEN_TEXTURE_FIELD_H
#define RAYTRACER_ENGINE_PROCGEN_TEXTURE_FIELD_H

#include "../../rt_math.h"   // Vec3
#include "tree.h"            // TextureData
#include <cstdint>
#include <functional>
#include <string>

namespace engine {

// A 2D scalar field over (u, v) — the compositional substrate for procedural
// textures (ADR-0042/0043). Mirrors Sdf exactly: a field is just a function, so
// primitives are closures and combinators wrap them. This is what lets a brick
// texture be *built from primitives* (a brick lattice × noise, two colours mixed
// by the mask) rather than only requested as a baked preset (surface_maps.h).
// Fields conventionally live on the unit square and tile.
using Field2 = std::function<double(double u, double v)>;

// --- primitives ---
Field2 fieldConstant(double value);
// Value noise / fbm at `scale` cells across the unit square, remapped to [0,1].
Field2 fieldNoise(uint32_t seed, double scale);
Field2 fieldFbm(uint32_t seed, double scale, int octaves);
// 0/1 checkerboard of `cols`×`rows` cells.
Field2 fieldChecker(double cols, double rows);
// A running-bond brick lattice: 1 inside a brick, 0 in the mortar gaps, with a
// per-brick random darkening up to `variation` so individual bricks read.
Field2 fieldBrick(double cols, double rows, double mortar, double variation,
                  uint32_t seed);
// A vertical ramp (= v); handy as a gradient term.
Field2 fieldGradientY();

// --- combinators ---
Field2 fieldAdd(Field2 a, Field2 b);
Field2 fieldMul(Field2 a, Field2 b);
Field2 fieldMix(Field2 a, Field2 b, double t);          // lerp a→b by t
Field2 fieldScaleBias(Field2 a, double scale, double bias);
Field2 fieldClamp(Field2 a, double lo, double hi);

// --- TILEABLE primitives (the flora plan's terrain layers, rocks, leaves). Exactly periodic
// over the unit square: a lattice of `period` cells that wraps, so a baked tile repeats with no
// seam. (fieldNoise / fieldFbm above sample unbounded noise and do not tile.) All in [0, 1].
Field2 fieldTileNoise(uint32_t seed, int period);
Field2 fieldTileFbm(uint32_t seed, int period, int octaves);   // period doubles per octave
// Cellular (Worley) over a wrapping jittered grid of `period` cells a side:
Field2 fieldCells(uint32_t seed, int period);       // distance to the nearest cell point (0 at it, ~0.5-0.7 between)
Field2 fieldCellEdges(uint32_t seed, int period);   // distance to the nearest cell BOUNDARY (0 on a crack)
Field2 fieldCellId(uint32_t seed, int period);      // a random value per cell (plates, pebbles, tones)
// Sine bands across v, `count` a tile (ripples, strata): 0.5 + 0.5 sin(2 pi count v).
Field2 fieldBands(double count);

// --- shaping and composition ---
Field2 fieldSmoothstep(Field2 a, double lo, double hi);
Field2 fieldInvert(Field2 a);                          // 1 - a
Field2 fieldMin(Field2 a, Field2 b);
Field2 fieldMax(Field2 a, Field2 b);
Field2 fieldMixBy(Field2 a, Field2 b, Field2 t);        // lerp a->b by a FIELD
Field2 fieldPow(Field2 a, double e);
// Domain warp: sample `a` at (u, v) displaced by (du, dv) (fields in [0, 1], centred on 0.5)
// times `amount`, wrapped to the unit square -- tile-preserving when du, dv tile.
Field2 fieldWarp(Field2 a, Field2 du, Field2 dv, double amount);

// --- colour fields: an RGB value over (u, v), LINEAR albedo ---
using ColorField2 = std::function<Vec3(double u, double v)>;
ColorField2 colorConstant(const Vec3& c);
ColorField2 colorMixBy(ColorField2 a, ColorField2 b, Field2 t);   // lerp a->b by a field
ColorField2 colorMul(ColorField2 c, Field2 k);                    // brightness by a field
ColorField2 colorTint(ColorField2 c, const Vec3& tint);            // per-channel multiply

// --- bake ---
// Grayscale: the field's value (clamped to [0,1]) written to all three channels
// — a roughness / height / AO map. `size`×`size`, row-major RGB.
TextureData bakeFieldGray(const Field2& f, int size);
// Colour: lerp `a`→`b` by the field used as a mask — e.g. mortar→brick by the
// brick lattice — giving an albedo map. `size`×`size`, row-major RGB.
TextureData bakeFieldColor(const Field2& mask, const Vec3& a, const Vec3& b,
                           int size);
// Tangent-space normal map from a height field: the surface normal of the height
// gradient, `strength` exaggerating relief, encoded RGB ([-1,1]→[0,1]). For
// brick mortar recesses, fabric weave, etc.
TextureData bakeFieldNormal(const Field2& height, double strength, int size);
// RGBA: rgb the colour (gamma-encoded when `gamma` -- 8-bit linear darks band -- else raw),
// a the field (a height for height blending, a mask, an alpha cut-out).
TextureData bakeFieldRGBA(const ColorField2& color, const Field2& alpha, int size, bool gamma = true);
// Any bake through a disk cache: `recipeKey` names the recipe and every parameter (and a
// version -- a field is a closure, it cannot be hashed itself); the result is stored under
// cache/fields/<hash>.tex and read back while the key is unchanged. RT_NOCACHE=1 always bakes.
TextureData bakeCached(const std::string& recipeKey, const std::function<TextureData()>& bake);

}  // namespace engine

#endif
