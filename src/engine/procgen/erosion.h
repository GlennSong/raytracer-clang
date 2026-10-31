#ifndef RAYTRACER_ENGINE_EROSION_H
#define RAYTRACER_ENGINE_EROSION_H

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

// The weather bake's last, opt-in stage (ADR-0126's "water" block; ADR-0257 retired the droplet sim that
// once ran here): breaching, the water-and-sediment pipe model, then thermal slumping of slopes past the talus.
struct ErosionParams {
    // Thermal (talus slumping).
    int   thermalIterations = 12;
    float talus = 1.2f;           // max height drop per cell before material slips
    float thermalRate = 0.5f;
    // Vulkan builds only (ADR-0122): run on the GPU when asked (the weather bake always asks). The CPU runs
    // thermal alone; the two agree statistically, not bit for bit, so the backend is part of the cache key.
    bool vulkan = false;
    // WATER AND SEDIMENT (ADR-0123; Vulkan only, 0 steps = off): the Mei et al. 2007 pipe model -- water
    // that pools, spreads and floods, carries sediment and drops it where it slows. Units are metres and seconds on the grid's real cell size (Heightmap::worldSize / (n - 1)).
    int   waterSteps = 0;
    float waterDt = 0.0f;          // s a step; 0 = derived from the cell size
    float waterRain = 0.002f;      // m/s on land
    float waterCapacity = 0.6f;    // Kc
    float waterDissolve = 0.25f;   // Ks, 1/s
    float waterDeposit = 0.4f;     // Kd, 1/s
    float waterEvaporate = 0.02f;  // Ke, 1/s
    float waterMinTilt = 0.02f;
    float waterMaxCut = 0.05f;     // m a step
    float waterRockHardness = 0.15f;   // bedrock's pick-up rate as a fraction of loose sediment's (ADR-0124)
    float waterCreep = 1.0e-4f;        // soil creep a step (ADR-0125)
    // BREACHING (ADR-0124; 0 = off): before the water, every closed hollow whose rim stands less than this
    // (m) above its floor gets a gently falling channel cut out to the sea or the map's edge -- the way
    // overflow breaches real basins. Deeper hollows stay closed: lakes. Noise relief is full of pits, and
    // the water filled every one flat with sediment.
    float breachDepth = 0.0f;
    float seaLevel = -1e30f;       // below it: the sea, held at its level
};

// Runs on the GPU (Vulkan) when asked and available, else thermal alone on the CPU. RT_CPU_EROSION=1 pins
// the CPU path.
void erode(Heightmap& hm, const ErosionParams& params);

// The backend a bake with `p` will run on (null: the platform default): "vk-erosion-v1" or "cpu". Folded
// into the weathered ground's cache key, so CPU- and GPU-baked terrain never share a cache entry.
const char* erosionBackendTag(const ErosionParams* p = nullptr);

}  // namespace engine

#endif
