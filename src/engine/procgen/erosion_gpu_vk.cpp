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
#include <queue>
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

enum Kernel { kDroplets, kClamp, kApplyApplied, kThermal, kApply, kWFlux, kWWater, kWErode, kWTransport, kKernelCount };
constexpr int kFirstWater = kWFlux;
const char* kKernelFile[kKernelCount] = {"erosion_droplets.comp.spv", "erosion_clamp.comp.spv",
                                         "erosion_apply_applied.comp.spv", "erosion_thermal.comp.spv",
                                         "erosion_apply.comp.spv", "erosion_w_flux.comp.spv", "erosion_w_water.comp.spv",
                                         "erosion_w_erode.comp.spv", "erosion_w_transport.comp.spv"};

// Mirrors the push block of erosion_water.comp (ADR-0123).
struct WaterPushVk {
    int32_t n;
    int32_t parity;
    float cellSize, dt, gravity, rain, capacity, dissolve, deposit, evaporate, seaLevel, minTilt, maxErodeDepth, rockHardness;
};
static_assert(sizeof(WaterPushVk) == 14 * 4, "push block must match erosion_water.comp");
constexpr uint32_t kWaterBindings = 10;

struct VkErosion {
    VkInstance instance = VK_NULL_HANDLE;
    VkPhysicalDevice phys = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t queueFamily = 0;
    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE, waterSetLayout = VK_NULL_HANDLE;
    VkPipelineLayout pipeLayout = VK_NULL_HANDLE, waterPipeLayout = VK_NULL_HANDLE;
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
    {   // the water kernels' layout: eight storage buffers, their own push block
        VkDescriptorSetLayoutBinding wb[kWaterBindings]{};
        for (uint32_t i = 0; i < kWaterBindings; ++i) {
            wb[i].binding = i;
            wb[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            wb[i].descriptorCount = 1;
            wb[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo wl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        wl.bindingCount = kWaterBindings;
        wl.pBindings = wb;
        if (vkCreateDescriptorSetLayout(g.device, &wl, nullptr, &g.waterSetLayout) != VK_SUCCESS) return false;
        VkPushConstantRange wpr{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(WaterPushVk)};
        VkPipelineLayoutCreateInfo wp{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        wp.setLayoutCount = 1;
        wp.pSetLayouts = &g.waterSetLayout;
        wp.pushConstantRangeCount = 1;
        wp.pPushConstantRanges = &wpr;
        if (vkCreatePipelineLayout(g.device, &wp, nullptr, &g.waterPipeLayout) != VK_SUCCESS) return false;
    }

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
        cpci.layout = k >= kFirstWater ? g.waterPipeLayout : g.pipeLayout;
        const VkResult r = vkCreateComputePipelines(g.device, VK_NULL_HANDLE, 1, &cpci, nullptr, &g.pipes[k]);
        vkDestroyShaderModule(g.device, mod, nullptr);
        if (r != VK_SUCCESS) return false;
    }
    VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    cpi.queueFamilyIndex = g.queueFamily;
    if (vkCreateCommandPool(g.device, &cpi, nullptr, &g.cmdPool) != VK_SUCCESS) return false;
    VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 5 + kWaterBindings};
    VkDescriptorPoolCreateInfo dpci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    dpci.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    dpci.maxSets = 2;
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

// BREACH DEPRESSIONS (ADR-0124, after Lindsay 2016's least-cost breaching, simplified): a priority flood
// from the map's edge and the sea inward, lowest first. A cell reached from a HIGHER one is in a hollow:
// the path it was reached along is cut down to fall gently from it (grade `fall` per metre) out to the
// outlet -- unless that would cut deeper than maxCut anywhere, when the hollow is left closed (a lake).
// Returns the number of hollows breached.
int breachDepressions(std::vector<float>& h, int n, float cell, float seaLevel, float maxCut) {
    const std::size_t N = static_cast<std::size_t>(n) * n;
    std::vector<int32_t> parent(N, -1);
    std::vector<float> level(N, 0.0f);   // the flood's level: a filled hollow's spill height
    std::vector<uint8_t> seen(N, 0);
    using Item = std::pair<float, int32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x) {
            const int32_t i = z * n + x;
            if (x == 0 || z == 0 || x == n - 1 || z == n - 1 || h[static_cast<std::size_t>(i)] < seaLevel) {
                seen[static_cast<std::size_t>(i)] = 1;
                level[static_cast<std::size_t>(i)] = h[static_cast<std::size_t>(i)];
                q.push({h[static_cast<std::size_t>(i)], i});
            }
        }
    const float fall = 0.002f;   // the cut channel's grade: 0.2 %
    int breached = 0;
    const int dx[4] = {1, -1, 0, 0}, dz[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        const auto [lv, c] = q.top();
        q.pop();
        const int cx = c % n, cz = c / n;
        for (int k = 0; k < 4; ++k) {
            const int nx = cx + dx[k], nz = cz + dz[k];
            if (nx < 0 || nz < 0 || nx >= n || nz >= n) continue;
            const int32_t nb = nz * n + nx;
            if (seen[static_cast<std::size_t>(nb)]) continue;
            seen[static_cast<std::size_t>(nb)] = 1;
            parent[static_cast<std::size_t>(nb)] = c;
            float hn = h[static_cast<std::size_t>(nb)];
            if (hn >= lv) { level[static_cast<std::size_t>(nb)] = hn; q.push({hn, nb}); continue; }
            // a hollow: can the way out be cut so it falls all the way from here?
            float worst = 0.0f, target = hn;
            for (int32_t a = c; a >= 0; a = parent[static_cast<std::size_t>(a)]) {
                target -= fall * cell;
                const float ha = h[static_cast<std::size_t>(a)];
                if (ha <= target) break;   // from here on the ground already falls
                worst = std::max(worst, ha - target);
                if (worst > maxCut) break;
            }
            if (worst <= maxCut) {
                target = hn;
                for (int32_t a = c; a >= 0; a = parent[static_cast<std::size_t>(a)]) {
                    target -= fall * cell;
                    float& ha = h[static_cast<std::size_t>(a)];
                    if (ha <= target) break;
                    ha = target;
                }
                ++breached;
                level[static_cast<std::size_t>(nb)] = hn;
                q.push({hn, nb});
            } else {   // too deep to breach: it floods to the spill level (a lake, left for the water to fill)
                level[static_cast<std::size_t>(nb)] = lv;
                q.push({lv, nb});
            }
        }
    }
    return breached;
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
    VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    VkFence fence = VK_NULL_HANDLE;
    vkCreateFence(g.device, &fci, nullptr, &fence);
    struct FreeFence { VkDevice d; VkFence f; ~FreeFence() { vkDestroyFence(d, f, nullptr); } } freeFence{g.device, fence};
    // CHUNKED SUBMITS: a long bake (thousands of water steps) goes in pieces, each submitted and waited
    // on before the next is recorded -- one multi-minute submit would stall the desktop sharing the GPU.
    // Order is still data order: a chunk starts after the previous one finished.
    auto begin = [&] {
        vkResetCommandBuffer(cmd, 0);
        VkCommandBufferBeginInfo cbbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        cbbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(cmd, &cbbi);
    };
    auto submit = [&] {
        vkEndCommandBuffer(cmd);
        vkResetFences(g.device, 1, &fence);
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd;
        return vkQueueSubmit(g.queue, 1, &si, fence) == VK_SUCCESS &&
               vkWaitForFences(g.device, 1, &fence, VK_TRUE, UINT64_MAX) == VK_SUCCESS;
    };
    begin();

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
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, g.pipeLayout, 0, 1, &set, 0, nullptr);
        vkCmdPushConstants(cmd, g.pipeLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof u, &u);
        vkCmdDispatch(cmd, groups, 1, 1);
        computeBarrier();
    };
    auto clampApply = [&] { dispatch(kClamp, cellGroups); dispatch(kApplyApplied, cellGroups); };

    // 1. DROPLETS (erosion.comp)
    const int batchSize = dropletBatchSize(n);
    int batchesInChunk = 0;
    for (int start = 0; start < p.droplets; start += batchSize) {
        u.batchStart = start;
        u.batchCount = std::min(batchSize, p.droplets - start);
        dispatch(kDroplets, static_cast<uint32_t>((u.batchCount + 63) / 64));
        clampApply();
        if (++batchesInChunk == 256) { if (!submit()) return false; begin(); batchesInChunk = 0; }
    }
    constexpr int kResidueFlushPasses = 8;   // as the Metal host: drain what the per-batch clamps held back
    for (int it = 0; it < kResidueFlushPasses; it++) clampApply();

    // 1b. BREACH (CPU, ADR-0124): read the droplets' ground back, cut the shallow hollows open, send it up
    if (p.waterSteps > 0 && p.breachDepth > 0.0f) {
        barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_READ_BIT);
        vkCmdCopyBuffer(cmd, height.buf, staging.buf, 1, &whole);
        if (!submit()) return false;
        std::vector<float> hh(static_cast<std::size_t>(cells));
        void* m = nullptr;
        vkMapMemory(g.device, staging.mem, 0, cells * 4, 0, &m);
        std::memcpy(hh.data(), m, cells * 4);
        const float cellM = hm.worldSize > 0.0f ? hm.worldSize / static_cast<float>(n - 1) : 1.0f;
        const int opened = breachDepressions(hh, n, cellM, p.seaLevel, p.breachDepth);
        std::memcpy(m, hh.data(), cells * 4);
        vkUnmapMemory(g.device, staging.mem);
        LOG_INFO << "[erosion] breached " << opened << " hollows (cuts up to " << p.breachDepth << " m)";
        begin();
        vkCmdCopyBuffer(cmd, staging.buf, height.buf, 1, &whole);
        barrier(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, rw);
    }

    // 2. WATER AND SEDIMENT (erosion_water.comp, ADR-0123): Mei et al.'s pipe model on the same height
    //    buffer, in chunks of 500 steps
    Buf water, flux, vel, sed0, sed1, tilt, wet, relief, soil;
    VkDescriptorSet wset = VK_NULL_HANDLE;
    const bool withWater = p.waterSteps > 0;
    if (withWater) {
        if (!makeBuffer(water, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
            !makeBuffer(flux, cells * 32, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
            !makeBuffer(vel, cells * 8, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
            !makeBuffer(sed0, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
            !makeBuffer(sed1, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
            !makeBuffer(tilt, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
            !makeBuffer(wet, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
            !makeBuffer(relief, cells * 8, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) ||
            !makeBuffer(soil, cells * 4, storage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
            LOG_WARN << "[erosion] GPU water allocation failed at n=" << n << ": CPU erosion";
            return false;
        }
        VkDescriptorSetAllocateInfo wai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        wai.descriptorPool = g.descPool;
        wai.descriptorSetCount = 1;
        wai.pSetLayouts = &g.waterSetLayout;
        if (vkAllocateDescriptorSets(g.device, &wai, &wset) != VK_SUCCESS) return false;
        VkDescriptorBufferInfo wi[kWaterBindings] = {{height.buf, 0, VK_WHOLE_SIZE}, {water.buf, 0, VK_WHOLE_SIZE}, {flux.buf, 0, VK_WHOLE_SIZE},
                                                     {vel.buf, 0, VK_WHOLE_SIZE}, {sed0.buf, 0, VK_WHOLE_SIZE}, {sed1.buf, 0, VK_WHOLE_SIZE},
                                                     {tilt.buf, 0, VK_WHOLE_SIZE}, {wet.buf, 0, VK_WHOLE_SIZE}, {relief.buf, 0, VK_WHOLE_SIZE},
                                                     {soil.buf, 0, VK_WHOLE_SIZE}};
        VkWriteDescriptorSet ww[kWaterBindings]{};
        for (uint32_t i = 0; i < kWaterBindings; ++i) {
            ww[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            ww[i].dstSet = wset;
            ww[i].dstBinding = i;
            ww[i].descriptorCount = 1;
            ww[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            ww[i].pBufferInfo = &wi[i];
        }
        vkUpdateDescriptorSets(g.device, kWaterBindings, ww, 0, nullptr);
        for (Buf* bz : {&water, &flux, &vel, &sed0, &sed1, &tilt, &wet, &relief, &soil}) vkCmdFillBuffer(cmd, bz->buf, 0, bz->size, 0);
        barrier(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, rw);
        WaterPushVk w{};
        w.n = n;
        w.cellSize = hm.worldSize > 0.0f ? hm.worldSize / static_cast<float>(n - 1) : 1.0f;
        w.gravity = 9.81f;
        // a gravity wave crosses no more than a quarter cell a step over ~4 m of water
        w.dt = p.waterDt > 0.0f ? p.waterDt : 0.25f * w.cellSize / std::sqrt(9.81f * 4.0f);
        w.rain = p.waterRain;
        w.capacity = p.waterCapacity;
        w.dissolve = p.waterDissolve;
        w.deposit = p.waterDeposit;
        w.evaporate = p.waterEvaporate;
        w.seaLevel = p.seaLevel;
        w.minTilt = p.waterMinTilt;
        w.maxErodeDepth = p.waterMaxCut;
        w.rockHardness = p.waterRockHardness;
        auto wdispatch = [&](Kernel k) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, g.pipes[k]);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, g.waterPipeLayout, 0, 1, &wset, 0, nullptr);
            vkCmdPushConstants(cmd, g.waterPipeLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof w, &w);
            vkCmdDispatch(cmd, cellGroups, 1, 1);
            computeBarrier();
        };
        for (int step = 0; step < p.waterSteps; ++step) {
            w.parity = step & 1;
            wdispatch(kWFlux);        // the pipes' flow, from the surface drop
            wdispatch(kWTransport);   // sediment along those pipes (needs the depth the flow left from)
            wdispatch(kWWater);       // then the depth, velocity, rain, sea; slope and relief for the next
            wdispatch(kWErode);
            if ((step + 1) % 500 == 0) { if (!submit()) return false; begin(); }
        }
        LOG_INFO << "[erosion] water: " << p.waterSteps << " steps of " << w.dt << " s on " << n << "^2 at " << w.cellSize << " m";
    }

    // 3. THERMAL (erosion.comp): slump what the water and droplets left steeper than the talus
    vkCmdFillBuffer(cmd, delta.buf, 0, cells * 4, 0);
    barrier(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, rw);
    for (int it = 0; it < p.thermalIterations; it++) { dispatch(kThermal, cellGroups); dispatch(kApply, cellGroups); }

    barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_READ_BIT);
    vkCmdCopyBuffer(cmd, height.buf, staging.buf, 1, &whole);
    barrier(VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_HOST_BIT, VK_ACCESS_HOST_READ_BIT);
    const bool ok = submit();
    if (wset) vkFreeDescriptorSets(g.device, g.descPool, 1, &wset);
    if (!ok) { LOG_WARN << "[erosion] GPU submit failed: CPU erosion"; return false; }

    // debug: RT_EROSION_DUMP=<dir> writes the water model's maps as raw float32 n x n (water depth,
    // wetness, suspended sediment) beside the height, for previews
    if (withWater) if (const char* dir = std::getenv("RT_EROSION_DUMP")) {
        auto dump = [&](Buf& b, const char* name) {
            Buf st;
            if (!makeBuffer(st, b.size, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host)) return;
            begin();
            VkBufferCopy c{0, 0, b.size};
            vkCmdCopyBuffer(cmd, b.buf, st.buf, 1, &c);
            if (!submit()) return;
            void* m = nullptr;
            vkMapMemory(g.device, st.mem, 0, b.size, 0, &m);
            std::ofstream(std::string(dir) + "/" + name, std::ios::binary).write(static_cast<const char*>(m), static_cast<std::streamsize>(b.size));
            vkUnmapMemory(g.device, st.mem);
        };
        dump(water, "water.f32"); dump(wet, "wet.f32"); dump(sed0, "sed.f32"); dump(soil, "soil.f32");
    }

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
