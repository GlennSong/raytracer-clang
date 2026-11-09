#ifndef RAYTRACER_RENDERER_VULKAN_RT_SCENE_H
#define RAYTRACER_RENDERER_VULKAN_RT_SCENE_H

// THE NEAR SCENE, FOR RAYS (ADR-0376 step 3b). What a reflection ray can hit: the meshes near the camera, each with its
// own bottom-level acceleration structure, under one top level rebuilt every frame. Glenn, 2026-11-07: "not blow up my
// gpu" -- so a hard budget (a share of the device memory free when it starts, capped), nearest first: a mesh that does
// not fit is left out of the rays (the probes answer for it), never allocated. Builds are spread over frames (a
// triangle budget a frame), and a structure no instance has used for a while is retired. The Vulkan renderer hands it
// the frame's near instances and its mesh buffers; it records the builds into the frame's command buffer.

#include <vulkan/vulkan.h>
#include "vk_mem_alloc.h"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine {

class RtScene {
public:
    // A near instance, nearest first: its mesh (`key`: the handle, unique while it lives), the mesh's buffers and
    // counts, its world transform (3 x 4, row-major) and the index the rays' hit shading looks its material up by.
    struct Instance {
        uint64_t key = 0;
        VkBuffer vertexBuffer = VK_NULL_HANDLE, indexBuffer = VK_NULL_HANDLE;
        uint32_t vertexCount = 0, indexCount = 0, stride = 32;
        float transform[12]{};
        uint32_t custom = 0;
    };

    bool init(VkDevice device, VmaAllocator allocator, VkPhysicalDevice gpu, VkDeviceSize budgetBytes);
    void shutdown();
    bool ready() const { return device_ != VK_NULL_HANDLE; }

    // This frame: builds what is missing (within the budgets), then the top level over every instance whose structure
    // is ready. `frame` is the frame-in-flight slot, `counter` the renderer's frame counter. Leaves the top level ready
    // for ray queries in compute and fragment shaders.
    void record(VkCommandBuffer cmd, int frame, uint64_t counter, const std::vector<Instance>& nearestFirst);
    // A mesh is gone: its structure retires
    void forget(uint64_t key, uint64_t counter);
    // Frees what retired long enough ago (no frame in flight can read it)
    void collect(uint64_t counter, int framesInFlight, bool all = false);

    VkAccelerationStructureKHR topLevel() const { return tlas_; }
    bool hasTopLevel() const { return tlas_ != VK_NULL_HANDLE && tlasInstances_ > 0; }

    struct Stats {
        uint32_t instances = 0;      // in the top level this frame
        uint32_t skipped = 0;        // near, but left out (over the budget, or still waiting to be built)
        uint32_t structures = 0;     // bottom levels held
        uint32_t builtThisFrame = 0;
        VkDeviceSize bytes = 0;      // all of it: bottom levels, scratch, the top level and its instances
        VkDeviceSize bottomBytes = 0, scratchBytes = 0;
        VkDeviceSize budget = 0;
    };
    Stats stats() const { return stats_; }

    static constexpr int kMaxFrames = 2;
    static constexpr uint64_t kRetireAfter = 300;          // frames unused before a bottom level retires
    static constexpr uint64_t kTrianglesPerFrame = 600000; // built a frame, at most (one mesh always goes)

private:
    struct Blas {
        VkAccelerationStructureKHR as = VK_NULL_HANDLE;
        VkBuffer buffer = VK_NULL_HANDLE;
        VmaAllocation alloc = nullptr;
        VkDeviceSize bytes = 0;
        VkDeviceAddress address = 0;
        uint64_t lastUsed = 0;
    };
    struct Retired {
        Blas b;
        uint64_t at = 0;
    };
    bool buffer(VkDeviceSize size, VkBufferUsageFlags usage, bool host, VkBuffer& out, VmaAllocation& alloc, void** mapped);
    void destroy(Blas& b);
    VkDeviceAddress addressOf(VkBuffer b) const;

    VkDevice device_ = VK_NULL_HANDLE;
    VmaAllocator allocator_ = nullptr;
    VkDeviceSize budget_ = 0, used_ = 0;
    VkDeviceSize scratchAlign_ = 128;
    PFN_vkCreateAccelerationStructureKHR create_ = nullptr;
    PFN_vkDestroyAccelerationStructureKHR destroy_ = nullptr;
    PFN_vkGetAccelerationStructureBuildSizesKHR sizes_ = nullptr;
    PFN_vkCmdBuildAccelerationStructuresKHR build_ = nullptr;
    PFN_vkGetAccelerationStructureDeviceAddressKHR asAddress_ = nullptr;

    std::unordered_map<uint64_t, Blas> blas_;
    std::vector<Retired> retired_;
    // the shared scratch (bottom levels, then the top level after a barrier)
    VkBuffer scratch_ = VK_NULL_HANDLE;
    VmaAllocation scratchAlloc_ = nullptr;
    VkDeviceSize scratchSize_ = 0;
    // the top level: one, rebuilt each frame (a barrier orders it after the last frame's reads); its instances per frame
    VkAccelerationStructureKHR tlas_ = VK_NULL_HANDLE;
    VkBuffer tlasBuffer_ = VK_NULL_HANDLE;
    VmaAllocation tlasAlloc_ = nullptr;
    VkDeviceSize tlasSize_ = 0;
    uint32_t tlasInstances_ = 0;
    std::vector<Retired> retiredTlas_;
    VkBuffer instBuf_[kMaxFrames]{};
    VmaAllocation instAlloc_[kMaxFrames]{};
    void* instMapped_[kMaxFrames]{};
    VkDeviceSize instCap_[kMaxFrames]{};
    Stats stats_;
};

}  // namespace engine

#endif
