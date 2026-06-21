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

// --- bake ---
// Grayscale: the field's value (clamped to [0,1]) written to all three channels
// — a roughness / height / AO map. `size`×`size`, row-major RGB.
TextureData bakeFieldGray(const Field2& f, int size);
// Colour: lerp `a`→`b` by the field used as a mask — e.g. mortar→brick by the
// brick lattice — giving an albedo map. `size`×`size`, row-major RGB.
TextureData bakeFieldColor(const Field2& mask, const Vec3& a, const Vec3& b,
                           int size);

}  // namespace engine

#endif
