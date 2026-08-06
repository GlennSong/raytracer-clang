#ifndef RAYTRACER_ENGINE_EROSION_H
#define RAYTRACER_ENGINE_EROSION_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "terrain.h"
#include "noise.h"
#include <cstdint>
#include <vector>

namespace engine {

// A mutable heightfield grid — the substrate erosion needs (it iterates and
// rewrites heights, unlike the stateless analytic terrainHeight). `n` vertices
// per side over `worldSize` world units, centered on the origin.
struct Heightmap {
    int n = 0;
    float worldSize = 0.0f;
    std::vector<float> h;   // n*n, row-major

    float get(int x, int y) const { return h[static_cast<size_t>(y) * n + x]; }
    void  set(int x, int y, float v) { h[static_cast<size_t>(y) * n + x] = v; }
    float sampleWorld(float worldX, float worldZ) const;   // bilinear
};

// Bake the analytic terrain (FBM + mountain layer) into a grid at `resolution`.
Heightmap bakeHeightmap(const TerrainParams& params, const Noise& noise);

// Hydraulic (droplet) + thermal erosion. Droplets run downhill, eroding where
// flow is fast/steep and depositing where it slows — carving drainage networks
// (valleys, ridges). Thermal passes slump slopes past the talus angle.
struct ErosionParams {
    // Hydraulic (droplet simulation).
    int   droplets = 60000;
    int   maxLifetime = 32;
    float inertia = 0.05f;        // 0 = follow gradient, 1 = keep direction
    float sedimentCapacity = 4.0f;
    float depositSpeed = 0.3f;
    float erodeSpeed = 0.3f;
    float evaporation = 0.02f;
    float gravity = 4.0f;
    float minSlope = 0.01f;
    int   erodeRadius = 3;        // brush radius (spreads erosion, avoids spikes)
    // Thermal (talus slumping).
    int   thermalIterations = 12;
    float talus = 1.2f;           // max height drop per cell before material slips
    float thermalRate = 0.5f;
    uint32_t seed = 0;
};

// Runs the GPU (Metal) port when available — same model, seconds instead of
// minutes — else the reference CPU sim. RT_CPU_EROSION=1 pins the CPU path.
void erode(Heightmap& hm, const ErosionParams& params);

// Which backend erode() will use: "gpu-erosion-v1" (Metal compute) or "cpu".
// Folded into content-hash cache keys so CPU- and GPU-baked terrain caches
// never cross-contaminate, and a future kernel revision invalidates cleanly.
const char* erosionBackendTag();

// Build a terrain mesh from a (possibly eroded) heightmap: smooth normals,
// planar UVs, and the same height/slope/altitude vertex coloring as
// generateTerrain. Centered on the origin.
RenderMesh generateTerrainMesh(const Heightmap& hm);

}  // namespace engine

#endif
