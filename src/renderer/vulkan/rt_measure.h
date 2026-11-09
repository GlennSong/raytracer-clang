#ifndef RAYTRACER_RENDERER_VULKAN_RT_MEASURE_H
#define RAYTRACER_RENDERER_VULKAN_RT_MEASURE_H

// RAY-TRACED REFLECTIONS, THE FEASIBILITY CHECK (renderer-realism phase 3c; Glenn, 2026-11-07: "Sure. Let's try it.
// And not blow up my gpu lol"). Before anything is built: what would the acceleration structures for the meshes now
// on the GPU cost? The driver answers (vkGetAccelerationStructureBuildSizesKHR) from counts alone -- no buffer, no
// build, no allocation. The renderer's own device is Vulkan 1.0 without the ray-tracing extensions, so the question
// is asked of a short-lived 1.2 device on the same GPU, made and destroyed here; the renderer's state is untouched.

#include <cstdint>
#include <string>
#include <vector>

namespace engine {

struct RtMeshCount {
    uint32_t vertices = 0;
    uint32_t triangles = 0;
    uint32_t stride = 32;   // bytes a vertex; the position is its first three floats in both layouts (ADR-0096)
};

struct RtMeasure {
    bool ok = false;
    std::string why;               // when not ok
    uint32_t meshes = 0;
    uint64_t triangles = 0;
    uint64_t blasBytes = 0;        // every mesh's bottom-level structure, uncompacted (fast trace)
    uint64_t blasScratchMax = 0;   // the largest build scratch (builds one at a time reuse it)
    uint64_t tlasBytes = 0;        // the top level over `instances`
    uint64_t tlasScratch = 0;
};

// `vendorID`/`deviceID`: the renderer's GPU (VkPhysicalDeviceProperties), so the same one answers.
RtMeasure measureRayTracingMemory(uint32_t vendorID, uint32_t deviceID, const std::vector<RtMeshCount>& meshes,
                                  uint32_t instances);

}  // namespace engine

#endif
