#include "rt_scene.h"
#include "../../log.h"

#include <algorithm>
#include <cstring>

namespace engine {

namespace {
VkDeviceSize alignUp(VkDeviceSize v, VkDeviceSize a) { return (v + a - 1) / a * a; }
}  // namespace

bool RtScene::init(VkDevice device, VmaAllocator allocator, VkPhysicalDevice gpu, VkDeviceSize budgetBytes) {
    create_ = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(vkGetDeviceProcAddr(device, "vkCreateAccelerationStructureKHR"));
    destroy_ = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(vkGetDeviceProcAddr(device, "vkDestroyAccelerationStructureKHR"));
    sizes_ = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(vkGetDeviceProcAddr(device, "vkGetAccelerationStructureBuildSizesKHR"));
    build_ = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(device, "vkCmdBuildAccelerationStructuresKHR"));
    asAddress_ = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(
        vkGetDeviceProcAddr(device, "vkGetAccelerationStructureDeviceAddressKHR"));
    if (!create_ || !destroy_ || !sizes_ || !build_ || !asAddress_) return false;
    VkPhysicalDeviceAccelerationStructurePropertiesKHR asProps{};
    asProps.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR;
    VkPhysicalDeviceProperties2 props{};
    props.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    props.pNext = &asProps;
    vkGetPhysicalDeviceProperties2(gpu, &props);
    scratchAlign_ = std::max<VkDeviceSize>(asProps.minAccelerationStructureScratchOffsetAlignment, 1);
    device_ = device;
    allocator_ = allocator;
    budget_ = budgetBytes;
    stats_.budget = budget_;
    return true;
}

bool RtScene::buffer(VkDeviceSize size, VkBufferUsageFlags usage, bool host, VkBuffer& out, VmaAllocation& alloc, void** mapped) {
    VkBufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    info.size = size;
    info.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VmaAllocationCreateInfo ai{};
    ai.usage = VMA_MEMORY_USAGE_AUTO;
    if (host) ai.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    else ai.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    VmaAllocationInfo got{};
    if (vmaCreateBuffer(allocator_, &info, &ai, &out, &alloc, &got) != VK_SUCCESS) {
        out = VK_NULL_HANDLE;
        alloc = nullptr;
        return false;
    }
    if (mapped) *mapped = got.pMappedData;
    return true;
}

VkDeviceAddress RtScene::addressOf(VkBuffer b) const {
    VkBufferDeviceAddressInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
    ai.buffer = b;
    return vkGetBufferDeviceAddress(device_, &ai);
}

void RtScene::destroy(Blas& b) {
    if (b.as) destroy_(device_, b.as, nullptr);
    if (b.buffer) vmaDestroyBuffer(allocator_, b.buffer, b.alloc);
    b = Blas();
}

void RtScene::forget(uint64_t key, uint64_t counter) {
    auto it = blas_.find(key);
    if (it == blas_.end()) return;
    used_ -= it->second.bytes;
    retired_.push_back(Retired{it->second, counter});
    blas_.erase(it);
}

void RtScene::collect(uint64_t counter, int framesInFlight, bool all) {
    auto done = [&](uint64_t r) { return all || r + static_cast<uint64_t>(framesInFlight) + 1 <= counter; };
    std::size_t keep = 0;
    for (Retired& r : retired_) {
        if (done(r.at)) destroy(r.b);
        else retired_[keep++] = r;
    }
    retired_.resize(keep);
    keep = 0;
    for (Retired& r : retiredTlas_) {
        if (done(r.at)) destroy(r.b);
        else retiredTlas_[keep++] = r;
    }
    retiredTlas_.resize(keep);
}

void RtScene::shutdown() {
    if (!device_) return;
    collect(0, 0, true);
    for (auto& [key, b] : blas_) destroy(b);
    blas_.clear();
    Blas t{tlas_, tlasBuffer_, tlasAlloc_, 0, 0, 0};
    destroy(t);
    tlas_ = VK_NULL_HANDLE;
    tlasBuffer_ = VK_NULL_HANDLE;
    if (scratch_) vmaDestroyBuffer(allocator_, scratch_, scratchAlloc_);
    scratch_ = VK_NULL_HANDLE;
    for (int f = 0; f < kMaxFrames; ++f)
        if (instBuf_[f]) { vmaDestroyBuffer(allocator_, instBuf_[f], instAlloc_[f]); instBuf_[f] = VK_NULL_HANDLE; }
    device_ = VK_NULL_HANDLE;
}

void RtScene::record(VkCommandBuffer cmd, int frame, uint64_t counter, const std::vector<Instance>& nearestFirst) {
    stats_ = Stats();
    stats_.budget = budget_;
    if (!device_) return;
    // retire the bottom levels nothing has used for a while
    for (auto it = blas_.begin(); it != blas_.end();) {
        if (it->second.lastUsed + kRetireAfter < counter) {
            used_ -= it->second.bytes;
            retired_.push_back(Retired{it->second, counter});
            it = blas_.erase(it);
        } else {
            ++it;
        }
    }
    // the budget's fixed parts: the scratch and the top level, as they stand (they grow inside it below)
    auto total = [&]() { return used_ + scratchSize_ + tlasSize_ + instCap_[0] + instCap_[1]; };

    // 1. THE MISSING BOTTOM LEVELS, nearest first, while the budgets allow
    struct Pending {
        VkAccelerationStructureGeometryKHR geo{};
        VkAccelerationStructureBuildGeometryInfoKHR info{};
        VkAccelerationStructureBuildRangeInfoKHR range{};
        VkDeviceSize scratchBytes = 0;
    };
    std::vector<Pending> pending;
    pending.reserve(16);
    uint64_t triangles = 0;
    VkDeviceSize scratchNeed = 0;
    std::vector<char> ready(nearestFirst.size(), 0);
    for (std::size_t i = 0; i < nearestFirst.size(); ++i) {
        const Instance& in = nearestFirst[i];
        auto it = blas_.find(in.key);
        if (it != blas_.end()) {
            it->second.lastUsed = counter;
            ready[i] = 1;
            continue;
        }
        const uint32_t tris = in.indexCount / 3;
        if (tris == 0 || in.vertexCount == 0 || !in.vertexBuffer || !in.indexBuffer) continue;
        if (!pending.empty() && triangles + tris > kTrianglesPerFrame) continue;   // next frame
        Pending p;
        p.geo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        p.geo.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
        p.geo.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
        VkAccelerationStructureGeometryTrianglesDataKHR& t = p.geo.geometry.triangles;
        t.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
        t.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;   // the first three floats of both vertex layouts
        t.vertexData.deviceAddress = addressOf(in.vertexBuffer);
        t.vertexStride = in.stride;
        t.maxVertex = in.vertexCount - 1;
        t.indexType = VK_INDEX_TYPE_UINT32;
        t.indexData.deviceAddress = addressOf(in.indexBuffer);
        p.info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        p.info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        p.info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        p.info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        p.info.geometryCount = 1;
        VkAccelerationStructureBuildSizesInfoKHR si{};
        si.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
        p.info.pGeometries = &p.geo;
        sizes_(device_, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &p.info, &tris, &si);
        const VkDeviceSize scratch = alignUp(si.buildScratchSize, scratchAlign_);
        const VkDeviceSize growScratch = std::max<VkDeviceSize>(scratchNeed + scratch, scratchSize_) - scratchSize_;
        // THE BUDGET: what is held, plus this one, plus any scratch it adds -- never past it
        if (total() + si.accelerationStructureSize + growScratch > budget_) { ++stats_.skipped; continue; }
        Blas b;
        if (!buffer(si.accelerationStructureSize, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR, false, b.buffer, b.alloc, nullptr)) {
            ++stats_.skipped;
            continue;
        }
        VkAccelerationStructureCreateInfoKHR ci{};
        ci.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
        ci.buffer = b.buffer;
        ci.size = si.accelerationStructureSize;
        ci.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        if (create_(device_, &ci, nullptr, &b.as) != VK_SUCCESS) {
            destroy(b);
            ++stats_.skipped;
            continue;
        }
        VkAccelerationStructureDeviceAddressInfoKHR ai{};
        ai.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
        ai.accelerationStructure = b.as;
        b.address = asAddress_(device_, &ai);
        b.bytes = si.accelerationStructureSize;
        b.lastUsed = counter;
        used_ += b.bytes;
        p.info.dstAccelerationStructure = b.as;
        p.range.primitiveCount = tris;
        p.scratchBytes = scratch;
        scratchNeed += scratch;
        triangles += tris;
        blas_[in.key] = b;
        pending.push_back(p);
        ready[i] = 1;
    }
    // the scratch, grown to the frame's builds (kept: the next frames reuse it)
    auto growScratch = [&](VkDeviceSize need) {
        if (need <= scratchSize_) return true;
        if (scratch_) retiredTlas_.push_back(Retired{Blas{VK_NULL_HANDLE, scratch_, scratchAlloc_, 0, 0, 0}, counter});
        scratch_ = VK_NULL_HANDLE;
        scratchSize_ = 0;
        if (!buffer(need, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false, scratch_, scratchAlloc_, nullptr)) return false;
        scratchSize_ = need;
        return true;
    };
    if (!pending.empty() && !growScratch(scratchNeed)) {
        // no scratch: undo this frame's structures (they were never built)
        for (Pending& p : pending)
            for (auto it = blas_.begin(); it != blas_.end(); ++it)
                if (it->second.as == p.info.dstAccelerationStructure) { used_ -= it->second.bytes; destroy(it->second); blas_.erase(it); break; }
        pending.clear();
        std::fill(ready.begin(), ready.end(), 0);
        for (std::size_t i = 0; i < nearestFirst.size(); ++i) ready[i] = blas_.count(nearestFirst[i].key) ? 1 : 0;
    }
    // the last frame's reads of the top level and the scratch's last use come before this frame's writes
    VkMemoryBarrier mb{};
    mb.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    mb.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    mb.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR | VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR | VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                         VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1, &mb, 0, nullptr, 0, nullptr);
    if (!pending.empty()) {
        VkDeviceAddress base = addressOf(scratch_);
        VkDeviceSize off = 0;
        std::vector<VkAccelerationStructureBuildGeometryInfoKHR> infos;
        std::vector<const VkAccelerationStructureBuildRangeInfoKHR*> ranges;
        for (Pending& p : pending) {
            p.info.pGeometries = &p.geo;
            p.info.scratchData.deviceAddress = alignUp(base + off, scratchAlign_);
            off += p.scratchBytes + scratchAlign_;
            infos.push_back(p.info);
            ranges.push_back(&p.range);
        }
        if (off > scratchSize_) {
            // (alignment slack: build in two halves is not worth it -- grow and go)
            if (!growScratch(off)) return;
            base = addressOf(scratch_);
            off = 0;
            for (std::size_t k = 0; k < pending.size(); ++k) {
                infos[k].scratchData.deviceAddress = alignUp(base + off, scratchAlign_);
                off += pending[k].scratchBytes + scratchAlign_;
            }
        }
        build_(cmd, static_cast<uint32_t>(infos.size()), infos.data(), ranges.data());
        stats_.builtThisFrame = static_cast<uint32_t>(pending.size());
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                             0, 1, &mb, 0, nullptr, 0, nullptr);
    }

    // 2. THE TOP LEVEL over every ready instance
    std::vector<VkAccelerationStructureInstanceKHR> inst;
    inst.reserve(nearestFirst.size());
    for (std::size_t i = 0; i < nearestFirst.size(); ++i) {
        if (!ready[i]) { ++stats_.skipped; continue; }
        const Instance& in = nearestFirst[i];
        auto it = blas_.find(in.key);
        if (it == blas_.end()) continue;
        VkAccelerationStructureInstanceKHR v{};
        std::memcpy(&v.transform, in.transform, sizeof(v.transform));
        v.instanceCustomIndex = in.custom & 0xFFFFFFu;
        v.mask = 0xFF;
        v.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
        v.accelerationStructureReference = it->second.address;
        inst.push_back(v);
    }
    tlasInstances_ = static_cast<uint32_t>(inst.size());
    stats_.instances = tlasInstances_;
    stats_.structures = static_cast<uint32_t>(blas_.size());
    if (inst.empty()) { stats_.bytes = total(); stats_.bottomBytes = used_; stats_.scratchBytes = scratchSize_; return; }
    const VkDeviceSize instBytes = inst.size() * sizeof(VkAccelerationStructureInstanceKHR);
    if (instCap_[frame] < instBytes) {
        if (instBuf_[frame]) retiredTlas_.push_back(Retired{Blas{VK_NULL_HANDLE, instBuf_[frame], instAlloc_[frame], 0, 0, 0}, counter});
        instBuf_[frame] = VK_NULL_HANDLE;
        instCap_[frame] = 0;
        const VkDeviceSize cap = std::max<VkDeviceSize>(instBytes * 2, 64 * 1024);
        if (!buffer(cap, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR, true, instBuf_[frame], instAlloc_[frame],
                    &instMapped_[frame])) { tlasInstances_ = 0; return; }
        instCap_[frame] = cap;
    }
    std::memcpy(instMapped_[frame], inst.data(), static_cast<std::size_t>(instBytes));
    vmaFlushAllocation(allocator_, instAlloc_[frame], 0, instBytes);

    VkAccelerationStructureGeometryKHR geo{};
    geo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
    geo.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geo.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    geo.geometry.instances.data.deviceAddress = addressOf(instBuf_[frame]);
    VkAccelerationStructureBuildGeometryInfoKHR info{};
    info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    info.geometryCount = 1;
    info.pGeometries = &geo;
    VkAccelerationStructureBuildSizesInfoKHR si{};
    si.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    sizes_(device_, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &info, &tlasInstances_, &si);
    if (si.accelerationStructureSize > tlasSize_) {
        // grown (with room): the old one retires, the frames in flight may still read it
        if (tlas_) retiredTlas_.push_back(Retired{Blas{tlas_, tlasBuffer_, tlasAlloc_, 0, 0, 0}, counter});
        tlas_ = VK_NULL_HANDLE;
        tlasBuffer_ = VK_NULL_HANDLE;
        tlasSize_ = 0;
        const VkDeviceSize size = si.accelerationStructureSize * 3 / 2;
        if (!buffer(size, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR, false, tlasBuffer_, tlasAlloc_, nullptr)) { tlasInstances_ = 0; return; }
        VkAccelerationStructureCreateInfoKHR ci{};
        ci.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
        ci.buffer = tlasBuffer_;
        ci.size = size;
        ci.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        if (create_(device_, &ci, nullptr, &tlas_) != VK_SUCCESS) { tlasInstances_ = 0; return; }
        tlasSize_ = size;
    }
    if (!growScratch(alignUp(si.buildScratchSize, scratchAlign_) + scratchAlign_)) { tlasInstances_ = 0; return; }
    info.dstAccelerationStructure = tlas_;
    info.scratchData.deviceAddress = alignUp(addressOf(scratch_), scratchAlign_);
    VkAccelerationStructureBuildRangeInfoKHR range{};
    range.primitiveCount = tlasInstances_;
    const VkAccelerationStructureBuildRangeInfoKHR* pr = &range;
    // the host wrote the instances
    VkMemoryBarrier hb{};
    hb.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    hb.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
    hb.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_HOST_BIT, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1, &hb, 0, nullptr, 0, nullptr);
    build_(cmd, 1, &info, &pr);
    // ready for the rays
    VkMemoryBarrier rb{};
    rb.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    rb.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    rb.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 1, &rb, 0, nullptr, 0, nullptr);
    stats_.bytes = total(); stats_.bottomBytes = used_; stats_.scratchBytes = scratchSize_;
}

}  // namespace engine
