#ifndef RAYTRACER_ENGINE_PROCGEN_TERRAIN_FIELD_H
#define RAYTRACER_ENGINE_PROCGEN_TERRAIN_FIELD_H

#include "../../rt_math.h"            // Vec3
#include "../../renderer/renderer.h"  // RenderMesh
#include <cstdint>
#include <functional>
#include <vector>

namespace engine {

struct ErosionParams;    // erosion.h
struct TerrainFlatten;   // terrain.h

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

// --- erosion (a stateful, grid-based bake pass — not a pointwise combinator) ---
// Bake `f` into a `resolution`×`resolution` heightmap over a `worldSize` square,
// run hydraulic (droplet) + thermal erosion (erosion.h), and return a HeightField
// that bilinearly samples the eroded grid — so it composes back into the field
// vocabulary. Erosion needs a grid (a point's eroded height depends on flow across
// the whole field), so unlike the analytic primitives it costs time + resolution.
HeightField erodeField(const HeightField& f, double worldSize, int resolution,
                       const ErosionParams& params);

// --- conform (cut/fill the terrain to a road network / block pads) ---
// Return a HeightField that blends `base` toward each flatten region's plane
// (applyFlatten): inside a footprint the height is forced to the plane (a flat
// pad, or a constant-grade road ramp), easing back to the natural terrain across
// the footprint's falloff. This is how an urban road stays flat (or a single
// incline) while the terrain conforms *around* it instead of the road draping
// over every bump. Pointwise — composes back into the field vocabulary, no grid.
HeightField conformField(HeightField base, std::vector<TerrainFlatten> regions);

// --- site selection (where a city can actually go) ---------------------------
// A buildable patch of ground: a centre and the radius of the largest flat disc
// that fits inside it. (ADR-0044: don't drape a city over mountain peaks — find
// the flat valleys first.)
struct FlatSite { double cx = 0, cz = 0, radius = 0; };
struct FlatSiteParams {
    double region = 1000;     // half-extent of the square searched, around `center`
    Vec3   center{0, 0, 0};   // search centre (uses .x/.z)
    double cell = 8;          // sample spacing (m): the heightmap "bitmap" pitch
    double maxSlope = 0.18;   // max |gradient| (rise/run) a cell may have to build
    double maxHeight = 1e9;   // ignore ground above this (keeps cities off peaks)
    double minRadius = 40;    // discard sites whose flat disc is smaller than this
    int    count = 4;         // return up to this many, largest disc first
    double minSeparation = 0; // drop a site within this of an already-kept one
};
// Sample `h` into a grid bitmap over the search square, mark cells buildable
// (slope <= maxSlope and height <= maxHeight), flood-fill connected buildable
// regions, and return each as a FlatSite — its largest inscribed disc (a distance
// transform to the nearest blocked cell), biggest first. The recipe plants a city
// at a site and sizes it to the radius, so it sits on flat ground that can hold it.
std::vector<FlatSite> findFlatSites(const HeightField& h, const FlatSiteParams& p);

// --- bake ---
// Tessellate the heightfield to a mesh: a `resolution`×`resolution`-cell grid
// over the [-size/2, size/2]² square centred at the origin, vertices sampled from
// the field, per-vertex normals from the height gradient, planar UVs, and a flat
// vertex colour. World-space, ready to drop into a ProcModel part.
RenderMesh bakeHeightMesh(const HeightField& h, double size, int resolution,
                          const Vec3& color);

}  // namespace engine

#endif
