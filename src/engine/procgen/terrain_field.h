#ifndef RAYTRACER_ENGINE_PROCGEN_TERRAIN_FIELD_H
#define RAYTRACER_ENGINE_PROCGEN_TERRAIN_FIELD_H

#include "../../rt_math.h"            // Vec3
#include "../../renderer/renderer.h"  // RenderMesh
#include <cstdint>
#include <functional>

namespace engine {

// A world-space heightfield over (x, z) — the compositional substrate for
// procedural terrain (ADR-0043 extended to terrain). The 2.5D sibling of Field2:
// a heightfield is a function, so primitives are closures and combinators wrap
// them. This is what lets terrain be *built from primitives* (fbm + ridged ridges
// + domain warp + terraces) rather than only requested as a preset
// (generateTerrain). Heights are metres; coordinates are world metres.
using HeightField = std::function<double(double x, double z)>;

// --- primitives ---
HeightField heightConstant(double h);
// Single-octave value noise / fbm / ridged, at `freq` cycles per world unit and
// `amp` metres. Ridged sums (1-|noise|)^2 octaves for sharp ridgelines.
HeightField heightNoise(uint32_t seed, double freq, double amp);
HeightField heightFbm(uint32_t seed, double freq, double amp, int octaves);
HeightField heightRidged(uint32_t seed, double freq, double amp, int octaves);
// Domain warp: sample `base` at coordinates pushed by `by` * `strength` — the
// swirl that turns banded fbm into organic ridgelines and meander.
HeightField heightWarp(HeightField base, HeightField by, double strength);
// Quantise to flat steps of `step` metres (mesas / terraces / paddies).
HeightField heightTerrace(HeightField base, double step);

// --- combinators ---
HeightField heightAdd(HeightField a, HeightField b);
HeightField heightMul(HeightField a, HeightField b);          // modulate a by b
HeightField heightScale(HeightField a, double s);
HeightField heightMax(HeightField a, HeightField b);
HeightField heightMin(HeightField a, HeightField b);
HeightField heightMix(HeightField a, HeightField b, double t);
HeightField heightClamp(HeightField a, double lo, double hi);

// --- bake ---
// Tessellate the heightfield to a mesh: a `resolution`×`resolution`-cell grid
// over the [-size/2, size/2]² square centred at the origin, vertices sampled from
// the field, per-vertex normals from the height gradient, planar UVs, and a flat
// vertex colour. World-space, ready to drop into a ProcModel part.
RenderMesh bakeHeightMesh(const HeightField& h, double size, int resolution,
                          const Vec3& color);

}  // namespace engine

#endif
