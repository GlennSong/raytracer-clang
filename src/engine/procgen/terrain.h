#ifndef RAYTRACER_ENGINE_TERRAIN_H
#define RAYTRACER_ENGINE_TERRAIN_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "noise.h"
#include <vector>

namespace engine {

// One ridge axis segment of a (possibly branching) mountain range: a ground-plane
// segment a->b with per-endpoint crest height.
struct RidgeSegment {
    Vec3  a, b;          // ground positions (x, 0, z)
    float ha = 0.0f, hb = 0.0f;
};

// Heightfield terrain (ROADMAP 4 Phase B.2) — the first generator combining the
// noise field (3.7) and the mesh builder (3.3). Deterministic for a given Noise,
// so the same recipe rebuilds the same terrain (ADR-0021).
struct TerrainParams {
    float size = 100.0f;       // world units across, centered on the origin (XZ)
    int   resolution = 64;     // grid cells per side; vertices = (resolution+1)^2
    float heightScale = 10.0f; // peak height (noise is ~[-1,1] * this)
    double noiseScale = 0.02;  // frequency: multiplies world coords before noise
    int   octaves = 5;
    double warp = 0.0;         // domain-warp amount (0 = plain FBM)
    // Long-range relief: a ridged-MULTIFRACTAL layer (varied, sharp, irregular
    // peaks — not uniform bumps), domain-warped, added on top of the hill octaves.
    // 0 = off.
    float  mountainHeight = 0.0f;
    double mountainScale = 0.004;
    // Regional mask: where mountains rise vs grassland. A large-scale field gates
    // the mountain layer (smoothstep maskLo..maskHi over it), so a range sits in
    // part of the map and plains/foothills elsewhere. 0 scale = mountains
    // everywhere (no regions).
    double mountainMaskScale = 0.0;
    float  mountainMaskLo = -0.12f;
    float  mountainMaskHi = 0.30f;
    // A mountain RANGE driven by a spine curve (range axis), instead of (or on
    // top of) the regional mask: uplift falls off with distance from the spine
    // (range -> foothills -> plains) and varies along it (tall massifs, low
    // passes). `rangeSpine` is a polyline of (x,_,z) points (sampled from the
    // authored control curve by the loader); empty = no range.
    std::vector<Vec3> rangeSpine;
    float rangeWidth = 220.0f;     // cross-axis falloff distance (to plains)
    float rangeHeight = 0.0f;      // peak uplift on the spine (0 = off)
    float rangeVariation = 0.55f;  // 0..1 along-spine height variation

    // A BRANCHING range: a ridge network (compiled from an L-system skeleton by
    // the loader). Each segment is a ridge axis with per-endpoint height (tall
    // main divide -> lower spurs by branch depth). Empty = no branching range.
    std::vector<RidgeSegment> rangeRidges;
};

// Build a branching ridge network from a planar L-system skeleton (consumer #2
// of the shared Skeleton): a main divide throws off spur ridges, sub-spurs, etc.
// Height falls by branch depth (depthFalloff). Lays the turtle's growth plane
// (x, y) onto the ground (x, z). Feeds TerrainParams::rangeRidges.
std::vector<RidgeSegment> buildRangeRidges(float length, float branchAngle,
                                           float falloff, float leaderFalloff,
                                           int iterations, float height,
                                           float depthFalloff, float angleJitter,
                                           uint32_t seed);

// Sample the terrain height at a world (x, z). The single source of truth for
// the surface — the mesh is built from it, and scatter/placement queries it
// directly (height, and slope via finite differences) without needing the mesh.
double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ);

// Sample a smooth spine polyline from authored control points (Catmull-Rom) for
// TerrainParams::rangeSpine. Points are (x, 0, z).
std::vector<Vec3> sampleRangeSpine(const std::vector<Vec3>& controls,
                                   int samples = 80);

// Height/slope-based ground color (ROADMAP 4 Phase D): grass on low flats, rock
// on steep or high ground, plus a noise term to break up the bands. `normalUp`
// is the surface normal's y (1 flat, 0 vertical); `noiseValue` ~[-1,1] varies
// it. Pure and testable; baked into the terrain's per-vertex colors.
Vec3 terrainColor(double height, double normalUp, double noiseValue);

// Build the terrain mesh: a grid in the XZ plane with y = terrainHeight, smooth
// normals, planar UVs spanning [0,1], and per-vertex height/slope coloration
// (terrainColor) baked in. Centered on the origin.
RenderMesh generateTerrain(const TerrainParams& params, const Noise& noise);

// Build one square annular ring of terrain from inner to outer half-extent at
// `cells` resolution (coarse), with a hole for the inner (higher-detail) tile.
// Used to extend terrain to the horizon as concentric coarsening LOD rings —
// cheap distant mountains/hills. Same height field as generateTerrain.
RenderMesh generateTerrainRing(const TerrainParams& params, const Noise& noise,
                               float innerHalf, float outerHalf, int cells);

// Concentric LOD rings around the central tile: `levels` rings, each doubling
// the extent (so triangle count per ring stays ~constant while coverage grows
// geometrically). Ring 0 starts at the central tile's edge (size/2).
std::vector<RenderMesh> generateTerrainLOD(const TerrainParams& params,
                                           const Noise& noise, int levels,
                                           int cells = 40);

}  // namespace engine

#endif
