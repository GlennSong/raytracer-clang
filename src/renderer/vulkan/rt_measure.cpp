#include "rt_measure.h"

#include <vulkan/vulkan.h>
#include <algorithm>
#include <cstring>

namespace engine {

RtMeasure measureRayTracingMemory(uint32_t vendorID, uint32_t deviceID, const std::vector<RtMeshCount>& meshes,
                                  uint32_t instances) {
    RtMeasure out;
    VkApplicationInfo app{};
    app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "rt_measure";
    app.apiVersion = VK_API_VERSION_1_2;
    VkInstanceCreateInfo ii{};
    ii.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ii.pApplicationInfo = &app;
    VkInstance instance = VK_NULL_HANDLE;
    if (vkCreateInstance(&ii, nullptr, &instance) != VK_SUCCESS) { out.why = "no Vulkan 1.2 instance"; return out; }
    VkDevice device = VK_NULL_HANDLE;
    auto finish = [&](const char* why) {
        if (device) vkDestroyDevice(device, nullptr);
        vkDestroyInstance(instance, nullptr);
        if (why) out.why = why;
        return out;
    };
    // the renderer's GPU
    uint32_t n = 0;
    vkEnumeratePhysicalDevices(instance, &n, nullptr);
    std::vector<VkPhysicalDevice> phys(n);
    vkEnumeratePhysicalDevices(instance, &n, phys.data());
    VkPhysicalDevice gpu = VK_NULL_HANDLE;
    for (VkPhysicalDevice p : phys) {
        VkPhysicalDeviceProperties pp;
        vkGetPhysicalDeviceProperties(p, &pp);
        if (pp.vendorID == vendorID && pp.deviceID == deviceID) { gpu = p; break; }
    }
    if (!gpu) return finish("the renderer's GPU is not on a 1.2 instance");
    uint32_t en = 0;
    vkEnumerateDeviceExtensionProperties(gpu, nullptr, &en, nullptr);
    std::vector<VkExtensionProperties> have(en);
    vkEnumerateDeviceExtensionProperties(gpu, nullptr, &en, have.data());
    const char* need[] = {VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME};
    for (const char* e : need) {
        bool found = false;
        for (const VkExtensionProperties& h : have) found = found || std::strcmp(h.extensionName, e) == 0;
        if (!found) return finish("no VK_KHR_acceleration_structure on this GPU");
    }
    // a queue, the acceleration-structure feature and buffer addresses (core in 1.2): nothing else
    float prio = 1.0f;
    VkDeviceQueueCreateInfo q{};
    q.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    q.queueFamilyIndex = 0;
    q.queueCount = 1;
    q.pQueuePriorities = &prio;
    VkPhysicalDeviceAccelerationStructureFeaturesKHR asf{};
    asf.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
    asf.accelerationStructure = VK_TRUE;
    VkPhysicalDeviceVulkan12Features v12{};
    v12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    v12.bufferDeviceAddress = VK_TRUE;
    v12.pNext = &asf;
    VkDeviceCreateInfo di{};
    di.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    di.pNext = &v12;
    di.queueCreateInfoCount = 1;
    di.pQueueCreateInfos = &q;
    di.enabledExtensionCount = 2;
    di.ppEnabledExtensionNames = need;
    if (vkCreateDevice(gpu, &di, nullptr, &device) != VK_SUCCESS) return finish("the measuring device was refused");
    PFN_vkGetAccelerationStructureBuildSizesKHR sizes = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(
        vkGetDeviceProcAddr(device, "vkGetAccelerationStructureBuildSizesKHR"));
    if (!sizes) return finish("no vkGetAccelerationStructureBuildSizesKHR");
    // each mesh's bottom level: its triangles, positions read from its own vertex buffer
    for (const RtMeshCount& m : meshes) {
        if (m.triangles == 0 || m.vertices == 0) continue;
        VkAccelerationStructureGeometryKHR geo{};
        geo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        geo.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
        geo.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
        geo.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
        geo.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
        geo.geometry.triangles.vertexStride = m.stride;
        geo.geometry.triangles.maxVertex = m.vertices - 1;
        geo.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
        VkAccelerationStructureBuildGeometryInfoKHR bi{};
        bi.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        bi.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        bi.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
        bi.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        bi.geometryCount = 1;
        bi.pGeometries = &geo;
        VkAccelerationStructureBuildSizesInfoKHR si{};
        si.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
        sizes(device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &bi, &m.triangles, &si);
        out.blasBytes += si.accelerationStructureSize;
        out.blasScratchMax = std::max<uint64_t>(out.blasScratchMax, si.buildScratchSize);
        out.triangles += m.triangles;
        ++out.meshes;
    }
    // the top level over the instances
    {
        VkAccelerationStructureGeometryKHR geo{};
        geo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        geo.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
        geo.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
        VkAccelerationStructureBuildGeometryInfoKHR bi{};
        bi.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        bi.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        bi.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        bi.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        bi.geometryCount = 1;
        bi.pGeometries = &geo;
        VkAccelerationStructureBuildSizesInfoKHR si{};
        si.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
        const uint32_t count = std::max(instances, 1u);
        sizes(device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &bi, &count, &si);
        out.tlasBytes = si.accelerationStructureSize + static_cast<uint64_t>(count) * sizeof(VkAccelerationStructureInstanceKHR);
        out.tlasScratch = si.buildScratchSize;
    }
    out.ok = true;
    return finish(nullptr);
}

}  // namespace engine
