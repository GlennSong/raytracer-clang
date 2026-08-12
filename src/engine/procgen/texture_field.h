#ifndef RAYTRACER_ENGINE_PROCGEN_TEXTURE_FIELD_H
#define RAYTRACER_ENGINE_PROCGEN_TEXTURE_FIELD_H

#include "../../rt_math.h"   // Vec3
#include "tree.h"            // TextureData
#include <cstdint>
#include <functional>

namespace engine {

// A 2D scalar field over (u, v) — the compositional substrate for procedural
// textures (ADR-0042/0043). Mirrors Sdf exactly: a field is just a function, so
// primitives are closures and combinators wrap them. This is what lets a brick
// texture be *built from primitives* (a brick lattice × noise, two colours mixed
// by the mask) rather than only requested as a baked preset (surface_maps.h).
// Fields conventionally live on the unit square and tile.
// NAMED TexField2, not Field2. `engine::Field2` is already taken — by a STRUCT
// in procgen/city/shape_ops.h, the Lot System's sampled distance field — and two
// different types under one name in one namespace is an ODR violation that costs
// nothing until some translation unit includes both headers. One finally did.
// (docs/lot-system-plan.md §5 lists this exact hazard among the invariants; it
// had already bitten twice inside the city module.)
using TexField2 = std::function<double(double u, double v)>;

// --- primitives ---
TexField2 fieldConstant(double value);
// Value noise / fbm at `scale` cells across the unit square, remapped to [0,1].
TexField2 fieldNoise(uint32_t seed, double scale);
TexField2 fieldFbm(uint32_t seed, double scale, int octaves);
// 0/1 checkerboard of `cols`×`rows` cells.
TexField2 fieldChecker(double cols, double rows);
// A running-bond brick lattice: 1 inside a brick, 0 in the mortar gaps, with a
// per-brick random darkening up to `variation` so individual bricks read.
TexField2 fieldBrick(double cols, double rows, double mortar, double variation,
                  uint32_t seed);
// A vertical ramp (= v); handy as a gradient term.
TexField2 fieldGradientY();

// --- combinators ---
TexField2 fieldAdd(TexField2 a, TexField2 b);
TexField2 fieldMul(TexField2 a, TexField2 b);
TexField2 fieldMix(TexField2 a, TexField2 b, double t);          // lerp a→b by t
TexField2 fieldScaleBias(TexField2 a, double scale, double bias);
TexField2 fieldClamp(TexField2 a, double lo, double hi);

// --- bake ---
// Grayscale: the field's value (clamped to [0,1]) written to all three channels
// — a roughness / height / AO map. `size`×`size`, row-major RGB.
TextureData bakeFieldGray(const TexField2& f, int size);
// Colour: lerp `a`→`b` by the field used as a mask — e.g. mortar→brick by the
// brick lattice — giving an albedo map. `size`×`size`, row-major RGB.
TextureData bakeFieldColor(const TexField2& mask, const Vec3& a, const Vec3& b,
                           int size);
// Tangent-space normal map from a height field: the surface normal of the height
// gradient, `strength` exaggerating relief, encoded RGB ([-1,1]→[0,1]). For
// brick mortar recesses, fabric weave, etc.
TextureData bakeFieldNormal(const TexField2& height, double strength, int size);

}  // namespace engine

#endif
