// GPU erosion bake on Vulkan (ADR-0122): the Linux/Windows twin of erosion_gpu.mm. Same seam
// (erodeGpuAvailable / erodeGpu, erosion_gpu.h), same kernels (shaders/vulkan/erosion.comp, one SPIR-V
// module per kernel under RT_VULKAN_SHADER_DIR), same dispatch order. Self-contained: its own headless
// instance, device and compute queue -- a load-time content bake has no renderer in reach.
//
// Buffers are device-local (the droplet kernel reads the heightmap thousands of times per droplet);
// the heightmap goes up and comes back through one host-visible staging buffer. Every dispatch is
// recorded into ONE command buffer with a compute->compute barrier between them, so dispatch order is
// data order -- the determinism contract of the Metal port.
#include "erosion_gpu.h"

#if defined(RT_HAVE_VULKAN_EROSION)

#include "erosion.h"
#include "../../log.h"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace engine {
namespace {

// Mirrors the push_constant block in erosion.comp (and the Metal uniforms): 15 four-byte scalars.
struct ErosionPushVk {
    int32_t  n;
    int32_t  batchStart;
    int32_t  batchCount;
    int32_t  maxLifetime;
    int32_t  brushCount;
    uint32_t seed;
    float    inertia;
    float    sedimentCapacity;
    float    depositSpeed;
    float    erodeSpeed;
    float    evaporation;
    float    gravity;
    float    minSlope;
    float    talus;
    float    thermalRate;
};
static_assert(sizeof(ErosionPushVk) == 15 * 4, "push block must match erosion.comp");

int dropletBatchSize(int n) { return std::clamp(n * n / 64, 1024, 8192); }   // as erosion_gpu.mm

enum Kernel { kDroplets, kClamp, kApplyApplied, kThermal, kApply, kKernelCount };
const char* kKernelFile[kKernelCount] = {"erosion_droplets.comp.spv", "erosion_clamp.comp.spv",
                                         "erosion_apply_applied.comp.spv", "erosion_thermal.comp.spv",
                                         "erosion_apply.comp.spv"};

struct VkErosion {
    VkInstance instance = VK_NULL_HANDLE;
    VkPhysicalDevice phys = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t queueFamily = 0;
    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE;
    VkPipelineLayout pipeLayout = VK_NULL_HANDLE;
    VkPipeline pipes[kKernelCount] = {};
    VkCommandPool cmdPool = VK_NULL_HANDLE;
    VkDescriptorPool descPool = VK_NULL_HANDLE;
    bool tried = false, failed = false;
    bool ready() const { return pipes[kApply] != VK_NULL_HANDLE; }
};

VkErosion& ctx() { static VkErosion g; return g; }
std::mutex& ctxMutex() { static std::mutex m; return m; }

std::vector<uint32_t> readSpirv(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const std::streamsize size = f.tellg();
    if (size <= 0 || size % 4) return {};
    std::vector<uint32_t> words(static_cast<std::size_t>(size) / 4);
    f.seekg(0);
    f.read(reinterpret_cast<char*>(words.data()), size);
    return words;
}

bool ensureReady() {
    VkErosion& g = ctx();
    if (g.ready()) return true;
    if (g.tried) return false;   // one attempt a process: no device, no kernels -> the CPU sim
    g.tried = true;

    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "rt-erosion";
    app.apiVersion = VK_API_VERSION_1_0;
    VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ici.pApplicationInfo = &app;
    if (vkCreateInstance(&ici, nullptr, &g.instance) != VK_SUCCESS) return false;

    uint32_t count = 0;
    vkEnumeratePhysicalDevices(g.instance, &count, nullptr);
    std::vector<VkPhysicalDevice> devs(count);
    vkEnumeratePhysicalDevices(g.instance, &count, devs.data());
    int bestScore = -1;
    for (VkPhysicalDevice d : devs) {   // a discrete GPU with a compute queue, else anything with one
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(d, &props);
        uint32_t qn = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, nullptr);
        std::vector<VkQueueFamilyProperties> qs(qn);
        vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, qs.data());
        for (uint32_t q = 0; q < qn; ++q) {
            if (!(qs[q].queueFlags & VK_QUEUE_COMPUTE_BIT)) continue;
            const int score = props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ? 2
                            : props.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU ? 1 : 0;
            if (score > bestScore) { bestScore = score; g.phys = d; g.queueFamily = q; }
            break;
        }
    }
    if (!g.phys) return false;

    const float prio = 1.0f;
    VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    qci.queueFamilyIndex = g.queueFamily;
    qci.queueCount = 1;
    qci.pQueuePriorities = &prio;
    VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;
    if (vkCreateDevice(g.phys, &dci, nullptr, &g.device) != VK_SUCCESS) return false;
    vkGetDeviceQueue(g.device, g.queueFamily, 0, &g.queue);

    VkDescriptorSetLayoutBinding b[5]{};
    for (uint32_t i = 0; i < 5; ++i) {
        b[i].binding = i;
        b[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        b[i].descriptorCount = 1;
        b[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo lci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    lci.bindingCount = 5;
    lci.pBindings = b;
    if (vkCreateDescriptorSetLayout(g.device, &lci, nullptr, &g.setLayout) != VK_SUCCESS) return false;
    VkPushConstantRange pcr{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(ErosionPushVk)};
    VkPipelineLayoutCreateInfo plci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    plci.setLayoutCount = 1;
    plci.pSetLayouts = &g.setLayout;
    plci.pushConstantRangeCount = 1;
    plci.pPushConstantRanges = &pcr;
    if (vkCreatePipelineLayout(g.device, &plci, nullptr, &g.pipeLayout) != VK_SUCCESS) return false;

    for (int k = 0; k < kKernelCount; ++k) {
        const std::vector<uint32_t> code = readSpirv(std::string(RT_VULKAN_SHADER_DIR) + "/" + kKernelFile[k]);
        if (code.empty()) { LOG_WARN << "[erosion] missing kernel " << kKernelFile[k] << ": CPU erosion"; return false; }
        VkShaderModuleCreateInfo smci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        smci.codeSize = code.size() * 4;
        smci.pCode = code.data();
        VkShaderModule mod = VK_NULL_HANDLE;
        if (vkCreateShaderModule(g.device, &smci, nullptr, &mod) != VK_SUCCESS) return false;
        VkComputePipelineCreateInfo cpci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        cpci.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        cpci.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        cpci.stage.module = mod;
        cpci.stage.pName = "main";
        cpci.layout = g.pipeLayout;
        const VkResult r = vkCreateComputePipelines(g.device, VK_NULL_HANDLE, 1, &cpci, nullptr, &g.pipes[k]);
        vkDestroyShaderModule(g.device, mod, nullptr);
        if (r != VK_SUCCESS) return false;
    }
    VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    cpi.queueFamilyIndex = g.queueFamily;
    if (vkCreateCommandPool(g.device, &cpi, nullptr, &g.cmdPool) != VK_SUCCESS) return false;
    VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 5};
    VkDescriptorPoolCreateInfo dpci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    dpci.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    dpci.maxSets = 1;
    dpci.poolSizeCount = 1;
    dpci.pPoolSizes = &ps;
    if (vkCreateDescriptorPool(g.device, &dpci, nullptr, &g.descPool) != VK_SUCCESS) return false;
    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(g.phys, &props);
    LOG_INFO << "[erosion] Vulkan compute on " << props.deviceName;
    return true;
}

// A buffer and its memory, freed on scope exit.
struct Buf {
    VkDevice dev = VK_NULL_HANDLE;
    VkBuffer buf = VK_NULL_HANDLE;
    VkDeviceMemory mem = VK_NULL_HANDLE;
    VkDeviceSize size = 0;
    ~Buf() {
        if (buf) vkDestroyBuffer(dev, buf, nullptr);
        if (mem) vkFreeMemory(dev, mem, nullptr);
    }
};

bool makeBuffer(Buf& b, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags want) {
    VkErosion& g = ctx();
    b.dev = g.device;
    b.size = size;
    VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bci.size = size;
    bci.usage = usage;
    bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(g.device, &bci, nullptr, &b.buf) != VK_SUCCESS) return false;
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(g.device, b.buf, &req);
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(g.phys, &mp);
    uint32_t type = UINT32_MAX;
    for (uint32_t i = 0; i < mp.memoryTypeCount; ++i)
        if ((req.memoryTypeBits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & want) == want) { type = i; break; }
    if (type == UINT32_MAX) return false;
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.allocationSize = req.size;
    mai.memoryTypeIndex = type;
    if (vkAllocateMemory(g.device, &mai, nullptr, &b.mem) != VK_SUCCESS) return false;
    return vkBindBufferMemory(g.device, b.buf, b.mem, 0) == VK_SUCCESS;
}

}  // namespace

bool erodeGpuAvailable() {
    std::lock_guard<std::mutex> lock(ctxMutex());
    return ensureReady();
}

bool erodeGpu(Heightmap& hm, const ErosionParams& p) {
    if (hm.n < 3 || hm.h.size() != static_cast<std::size_t>(hm.n) * static_cast<std::size_t>(hm.n)) return false;
    std::lock_guard<std::mutex> lock(ctxMutex());   // one context, one bake at a time
    if (!ensureReady()) return false;
    VkErosion& g = ctx();
    const int n = hm.n;
    const VkDeviceSize cells = static_cast<VkDeviceSize>(n) * n;

    // the brush, exactly as the Metal host builds it
    std::vector<int32_t> brushOff;
    std::vector<float> brushW;
    {
        const int r = std::max(1, p.erodeRadius);
        float wsum = 0.0f;
        for (int dy = -r; dy <= r; dy++)
            for (int dx = -r; dx <= r; dx++) {
                const float d2 = static_cast<float>(dx * dx + dy * dy);
                if (d2 > r * r) continue;
                const float w = 1.0f - std::sqrt(d2) / r;
                brushOff.push_back(dx); brushOff.push_back(dy); brushW.push_back(w); wsum += w;
            }
        for (float& w : brushW) w /= wsum;
    }

    const VkBufferUsageFlags storage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    Buf height, delta, applied, bOff, bW, staging;
    const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    if (!makeBuffer(height, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
        !makeBuffer(delta, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
        !makeBuffer(applied, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
        !makeBuffer(bOff, brushOff.size() * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, host) ||
        !makeBuffer(bW, brushW.size() * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, host) ||
        !makeBuffer(staging, cells * 4, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, host)) {
        LOG_WARN << "[erosion] GPU allocation failed at n=" << n << ": CPU erosion";
        return false;
    }
    auto fill = [&](Buf& b, const void* src, std::size_t bytes) {
        void* dst = nullptr;
        vkMapMemory(g.device, b.mem, 0, bytes, 0, &dst);
        std::memcpy(dst, src, bytes);
        vkUnmapMemory(g.device, b.mem);
    };
    fill(bOff, brushOff.data(), brushOff.size() * 4);
    fill(bW, brushW.data(), brushW.size() * 4);
    fill(staging, hm.h.data(), cells * 4);

    VkDescriptorSetAllocateInfo dsai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    dsai.descriptorPool = g.descPool;
    dsai.descriptorSetCount = 1;
    dsai.pSetLayouts = &g.setLayout;
    VkDescriptorSet set = VK_NULL_HANDLE;
    if (vkAllocateDescriptorSets(g.device, &dsai, &set) != VK_SUCCESS) return false;
    struct FreeSet { VkDevice d; VkDescriptorPool p; VkDescriptorSet s; ~FreeSet() { vkFreeDescriptorSets(d, p, 1, &s); } } freeSet{g.device, g.descPool, set};
    VkDescriptorBufferInfo infos[5] = {{height.buf, 0, VK_WHOLE_SIZE}, {delta.buf, 0, VK_WHOLE_SIZE}, {applied.buf, 0, VK_WHOLE_SIZE},
                                       {bOff.buf, 0, VK_WHOLE_SIZE}, {bW.buf, 0, VK_WHOLE_SIZE}};
    VkWriteDescriptorSet writes[5]{};
    for (uint32_t i = 0; i < 5; ++i) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = set;
        writes[i].dstBinding = i;
        writes[i].descriptorCount = 1;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[i].pBufferInfo = &infos[i];
    }
    vkUpdateDescriptorSets(g.device, 5, writes, 0, nullptr);

    VkCommandBufferAllocateInfo cbai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cbai.commandPool = g.cmdPool;
    cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbai.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    if (vkAllocateCommandBuffers(g.device, &cbai, &cmd) != VK_SUCCESS) return false;
    struct FreeCmd { VkDevice d; VkCommandPool p; VkCommandBuffer c; ~FreeCmd() { vkFreeCommandBuffers(d, p, 1, &c); } } freeCmd{g.device, g.cmdPool, cmd};
    VkCommandBufferBeginInfo cbbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    cbbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &cbbi);

    auto barrier = [&](VkPipelineStageFlags src, VkAccessFlags srcA, VkPipelineStageFlags dst, VkAccessFlags dstA) {
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        mb.srcAccessMask = srcA;
        mb.dstAccessMask = dstA;
        vkCmdPipelineBarrier(cmd, src, dst, 0, 1, &mb, 0, nullptr, 0, nullptr);
    };
    const VkAccessFlags rw = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    auto computeBarrier = [&] { barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, rw, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, rw); };

    VkBufferCopy whole{0, 0, cells * 4};
    vkCmdCopyBuffer(cmd, staging.buf, height.buf, 1, &whole);
    vkCmdFillBuffer(cmd, delta.buf, 0, cells * 4, 0);
    vkCmdFillBuffer(cmd, applied.buf, 0, cells * 4, 0);
    barrier(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, rw);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, g.pipeLayout, 0, 1, &set, 0, nullptr);

    ErosionPushVk u{};
    u.n = n;
    u.maxLifetime = p.maxLifetime;
    u.brushCount = static_cast<int32_t>(brushW.size());
    u.seed = p.seed;
    u.inertia = p.inertia;
    u.sedimentCapacity = p.sedimentCapacity;
    u.depositSpeed = p.depositSpeed;
    u.erodeSpeed = p.erodeSpeed;
    u.evaporation = p.evaporation;
    u.gravity = p.gravity;
    u.minSlope = p.minSlope;
    u.talus = p.talus;
    u.thermalRate = p.thermalRate;
    const uint32_t cellGroups = static_cast<uint32_t>((cells + 63) / 64);
    auto dispatch = [&](Kernel k, uint32_t groups) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, g.pipes[k]);
        vkCmdPushConstants(cmd, g.pipeLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof u, &u);
        vkCmdDispatch(cmd, groups, 1, 1);
        computeBarrier();
    };
    auto clampApply = [&] { dispatch(kClamp, cellGroups); dispatch(kApplyApplied, cellGroups); };

    const int batchSize = dropletBatchSize(n);
    for (int start = 0; start < p.droplets; start += batchSize) {
        u.batchStart = start;
        u.batchCount = std::min(batchSize, p.droplets - start);
        dispatch(kDroplets, static_cast<uint32_t>((u.batchCount + 63) / 64));
        clampApply();
    }
    constexpr int kResidueFlushPasses = 8;   // as the Metal host: drain what the per-batch clamps held back
    for (int it = 0; it < kResidueFlushPasses; it++) clampApply();
    vkCmdFillBuffer(cmd, delta.buf, 0, cells * 4, 0);
    barrier(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, rw);
    for (int it = 0; it < p.thermalIterations; it++) { dispatch(kThermal, cellGroups); dispatch(kApply, cellGroups); }

    barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_READ_BIT);
    vkCmdCopyBuffer(cmd, height.buf, staging.buf, 1, &whole);
    barrier(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_HOST_BIT, VK_ACCESS_HOST_READ_BIT);
    vkEndCommandBuffer(cmd);

    VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    VkFence fence = VK_NULL_HANDLE;
    vkCreateFence(g.device, &fci, nullptr, &fence);
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd;
    const bool ok = vkQueueSubmit(g.queue, 1, &si, fence) == VK_SUCCESS &&
                    vkWaitForFences(g.device, 1, &fence, VK_TRUE, UINT64_MAX) == VK_SUCCESS;
    vkDestroyFence(g.device, fence, nullptr);
    if (!ok) { LOG_WARN << "[erosion] GPU submit failed: CPU erosion"; return false; }

    void* src = nullptr;
    vkMapMemory(g.device, staging.mem, 0, cells * 4, 0, &src);
    std::vector<float> out(static_cast<std::size_t>(cells));
    std::memcpy(out.data(), src, cells * 4);
    vkUnmapMemory(g.device, staging.mem);
    for (float v : out) if (!std::isfinite(v)) { LOG_WARN << "[erosion] GPU result not finite: CPU erosion"; return false; }
    hm.h = std::move(out);
    return true;
}

}  // namespace engine

#endif  // RT_HAVE_VULKAN_EROSION
