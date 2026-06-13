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
};

// Sample the terrain height at a world (x, z). The single source of truth for
// the surface — the mesh is built from it, and scatter/placement queries it
// directly (height, and slope via finite differences) without needing the mesh.
double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ);

// Build the terrain mesh: a grid in the XZ plane with y = terrainHeight, smooth
// normals, and planar UVs spanning [0,1]. Centered on the origin.
RenderMesh generateTerrain(const TerrainParams& params, const Noise& noise);

}  // namespace engine

#endif
