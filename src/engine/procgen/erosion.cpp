#include "erosion.h"
#include "../../log.h"
#include "erosion_gpu.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace engine {

float Heightmap::sampleWorld(float worldX, float worldZ) const {
    if (n < 2) return 0.0f;
    // World [-worldSize/2, worldSize/2] -> grid [0, n-1].
    float gx = (worldX / worldSize + 0.5f) * (n - 1);
    float gz = (worldZ / worldSize + 0.5f) * (n - 1);
    gx = std::clamp(gx, 0.0f, static_cast<float>(n - 1));
    gz = std::clamp(gz, 0.0f, static_cast<float>(n - 1));
    int x0 = static_cast<int>(gx), z0 = static_cast<int>(gz);
    int x1 = std::min(x0 + 1, n - 1), z1 = std::min(z0 + 1, n - 1);
    float fx = gx - x0, fz = gz - z0;
    float a = get(x0, z0), b = get(x1, z0), c = get(x0, z1), d = get(x1, z1);
    return (a * (1 - fx) + b * fx) * (1 - fz) + (c * (1 - fx) + d * fx) * fz;
}

Heightmap bakeHeightmap(const TerrainParams& params, const Noise& noise) {
    Heightmap hm;
    hm.n = std::max(2, params.resolution) + 1;
    hm.worldSize = params.size;
    hm.h.resize(static_cast<size_t>(hm.n) * hm.n);
    float half = params.size * 0.5f;
    float step = params.size / (hm.n - 1);
    for (int z = 0; z < hm.n; z++)
        for (int x = 0; x < hm.n; x++)
            hm.set(x, z, static_cast<float>(terrainHeight(
                             params, noise, -half + x * step, -half + z * step)));
    return hm;
}

namespace {

// RT_CPU_EROSION=1 pins the reference CPU sim (debugging, A/B comparisons,
// and machines where the GPU result is unwanted).
bool forceCpuErosion() {
    const char* e = std::getenv("RT_CPU_EROSION");
    return e && e[0] != '\0' && !(e[0] == '0' && e[1] == '\0');
}

}  // namespace

const char* erosionBackendTag(const ErosionParams* p) {
    // run-to-run deterministic, not bit-identical to the CPU (ADR-0122)
    return (p && p->vulkan && !forceCpuErosion() && erodeGpuAvailable()) ? "vk-erosion-v1" : "cpu";
}

void erode(Heightmap& hm, const ErosionParams& p) {
    if (hm.n < 3) return;
    // GPU first (ADR-0122). Any failure (no device, kernel missing, env pin) falls through to the CPU below.
    if (p.vulkan && !forceCpuErosion() && erodeGpu(hm, p)) return;
    // the water model and breaching exist only on the Vulkan backend: say so rather than bake without them
    if (p.waterSteps > 0 || p.breachDepth > 0.0f)
        LOG_WARN << "[erosion] water steps / breach asked of the CPU backend, which has neither: the ground bakes without them";
    // --- Thermal: slump slopes steeper than the talus drop ---
    for (int it = 0; it < p.thermalIterations; it++) {
        std::vector<float> delta(hm.h.size(), 0.0f);
        for (int z = 1; z < hm.n - 1; z++) {
            for (int x = 1; x < hm.n - 1; x++) {
                float here = hm.get(x, z);
                // Move material to the lowest neighbor if the drop exceeds talus.
                int lx = x, lz = z; float lowest = here;
                const int off[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (auto& o : off) {
                    float nh = hm.get(x + o[0], z + o[1]);
                    if (nh < lowest) { lowest = nh; lx = x + o[0]; lz = z + o[1]; }
                }
                float drop = here - lowest;
                if (drop > p.talus) {
                    float move = (drop - p.talus) * 0.5f * p.thermalRate;
                    delta[static_cast<size_t>(z) * hm.n + x] -= move;
                    delta[static_cast<size_t>(lz) * hm.n + lx] += move;
                }
            }
        }
        for (size_t k = 0; k < hm.h.size(); k++) hm.h[k] += delta[k];
    }
}

}  // namespace engine
