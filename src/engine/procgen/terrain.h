#ifndef RAYTRACER_ENGINE_TERRAIN_H
#define RAYTRACER_ENGINE_TERRAIN_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "noise.h"

namespace engine {

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
    // Long-range relief: a low-frequency ridged layer added on top, so distant
    // terrain reads as mountains / rolling hills while the high-frequency octaves
    // above give near detail. 0 = off.
    float  mountainHeight = 0.0f;
    double mountainScale = 0.004;
};

// Sample the terrain height at a world (x, z). The single source of truth for
// the surface — the mesh is built from it, and scatter/placement queries it
// directly (height, and slope via finite differences) without needing the mesh.
double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ);

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
