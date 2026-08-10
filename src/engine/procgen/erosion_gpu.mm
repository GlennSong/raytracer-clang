// Metal host for the GPU erosion bake (metropolis-scale-plan P0.5).
// Self-contained by design: this runs at content-bake time where no renderer
// exists, so it creates its own system device + queue and compiles
// shaders/metal/erosion.metal from disk (CWD-relative, the renderer's shader
// convention). Every failure path returns false so erode() falls through to
// the reference CPU sim in erosion.cpp.

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "erosion_gpu.h"
#include "erosion.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace engine {

namespace {

// Mirrors struct ErosionUniforms in shaders/metal/erosion.metal EXACTLY
// (order + 4-byte scalar types only, so C++ and MSL layouts agree).
struct ErosionUniformsCpu {
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
static_assert(sizeof(ErosionUniformsCpu) == 15 * 4,
              "uniforms must stay 4-byte scalars to match the MSL layout");

// Droplets per dispatch: a fixed function of the grid (never of scheduling),
// so batch boundaries — which are part of the simulation's definition, since
// each batch reads the heightmap as-of the previous one — stay run-to-run
// deterministic. Scaled with cell count because within a batch droplets can't
// see each other: too many per cell and the relief clamp dominates the result
// (under-carving vs the CPU sim). 1/64 cells per batch keeps the clamp a
// safety net; 8192 is the throughput sweet spot at metropolis scale (2048^2).
// Changing this changes the baked result: bump the cache backend tag with it.
int dropletBatchSize(int n) {
    return std::clamp(n * n / 64, 1024, 8192);
}

struct GpuErosion {
    id<MTLDevice>               device = nil;
    id<MTLCommandQueue>         queue = nil;
    id<MTLComputePipelineState> dropletsPso = nil;
    id<MTLComputePipelineState> clampPso = nil;
    id<MTLComputePipelineState> applyAppliedPso = nil;
    id<MTLComputePipelineState> thermalPso = nil;
    id<MTLComputePipelineState> applyPso = nil;
    bool deviceTried = false;
    bool compileFailed = false;   // real compile error: don't retry every bake
    bool ready() const { return applyPso != nil; }
};

GpuErosion& ctx() {
    static GpuErosion g;
    return g;
}

// Lazily bring up device + pipelines. A missing kernel file is NOT cached as
// failure (the process may simply be running from the wrong directory when
// first asked); an actual Metal error is.
bool ensureReady() {
    GpuErosion& g = ctx();
    if (g.ready()) return true;
    if (g.compileFailed) return false;
    @autoreleasepool {
        if (!g.deviceTried) {
            g.deviceTried = true;
            g.device = MTLCreateSystemDefaultDevice();
        }
        if (!g.device) return false;

        NSError* error = nil;
        NSString* path = @"shaders/metal/erosion.metal";
        NSString* src = [NSString stringWithContentsOfFile:path
                                                  encoding:NSUTF8StringEncoding
                                                     error:&error];
        if (!src) return false;   // retryable: see note above

        id<MTLLibrary> library = [g.device newLibraryWithSource:src
                                                        options:nil
                                                          error:&error];
        if (!library) {
            NSLog(@"erosion.metal compile error: %@", error);
            g.compileFailed = true;
            return false;
        }
        auto makePso = [&](NSString* name) -> id<MTLComputePipelineState> {
            id<MTLFunction> fn = [library newFunctionWithName:name];
            if (!fn) return nil;
            NSError* psoError = nil;
            id<MTLComputePipelineState> pso =
                [g.device newComputePipelineStateWithFunction:fn
                                                        error:&psoError];
            if (!pso) NSLog(@"erosion pipeline %@ failed: %@", name, psoError);
            return pso;
        };
        id<MTLComputePipelineState> droplets = makePso(@"erodeDroplets");
        id<MTLComputePipelineState> clampD = makePso(@"clampDelta");
        id<MTLComputePipelineState> applyA = makePso(@"applyApplied");
        id<MTLComputePipelineState> thermal = makePso(@"thermalStep");
        id<MTLComputePipelineState> apply = makePso(@"applyDelta");
        id<MTLCommandQueue> queue = [g.device newCommandQueue];
        if (!droplets || !clampD || !applyA || !thermal || !apply || !queue) {
            g.compileFailed = true;
            return false;
        }
        g.dropletsPso = droplets;
        g.clampPso = clampD;
        g.applyAppliedPso = applyA;
        g.thermalPso = thermal;
        g.applyPso = apply;
        g.queue = queue;
    }
    return true;
}

}  // namespace

bool erodeGpuAvailable() { return ensureReady(); }

bool erodeGpu(Heightmap& hm, const ErosionParams& p) {
    if (hm.n < 3 ||
        hm.h.size() != static_cast<size_t>(hm.n) * static_cast<size_t>(hm.n))
        return false;
    if (!ensureReady()) return false;
    GpuErosion& g = ctx();

    @autoreleasepool {
        const int n = hm.n;
        const NSUInteger cells = static_cast<NSUInteger>(n) * n;

        id<MTLBuffer> heightBuf =
            [g.device newBufferWithBytes:hm.h.data()
                                  length:cells * sizeof(float)
                                 options:MTLResourceStorageModeShared];
        id<MTLBuffer> deltaBuf =
            [g.device newBufferWithLength:cells * sizeof(int32_t)
                                  options:MTLResourceStorageModeShared];
        id<MTLBuffer> appliedBuf =
            [g.device newBufferWithLength:cells * sizeof(int32_t)
                                  options:MTLResourceStorageModeShared];
        if (!heightBuf || !deltaBuf || !appliedBuf) return false;
        std::memset(deltaBuf.contents, 0, cells * sizeof(int32_t));

        // Same circular brush construction as the CPU sim (offsets +
        // normalized weights), uploaded once as flat (dx,dz) pairs.
        std::vector<int32_t> brushOff;
        std::vector<float> brushW;
        {
            int r = std::max(1, p.erodeRadius);
            float wsum = 0.0f;
            for (int dy = -r; dy <= r; dy++)
                for (int dx = -r; dx <= r; dx++) {
                    float d2 = static_cast<float>(dx * dx + dy * dy);
                    if (d2 > r * r) continue;
                    float w = 1.0f - std::sqrt(d2) / r;
                    brushOff.push_back(dx);
                    brushOff.push_back(dy);
                    brushW.push_back(w);
                    wsum += w;
                }
            for (float& w : brushW) w /= wsum;
        }
        id<MTLBuffer> brushOffBuf =
            [g.device newBufferWithBytes:brushOff.data()
                                  length:brushOff.size() * sizeof(int32_t)
                                 options:MTLResourceStorageModeShared];
        id<MTLBuffer> brushWBuf =
            [g.device newBufferWithBytes:brushW.data()
                                  length:brushW.size() * sizeof(float)
                                 options:MTLResourceStorageModeShared];
        if (!brushOffBuf || !brushWBuf) return false;

        ErosionUniformsCpu u{};
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

        id<MTLCommandBuffer> cmd = [g.queue commandBuffer];
        // Default (serial) encoder: successive dispatches execute in order, so
        // each applyDelta sees the whole batch's atomics and the next batch
        // sees the applied heights — the batching contract in erosion.metal.
        id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
        if (!cmd || !enc) return false;

        const NSUInteger applyWidth =
            std::min<NSUInteger>(g.applyPso.maxTotalThreadsPerThreadgroup, 256);
        const MTLSize applyGroup = MTLSizeMake(applyWidth, 1, 1);
        const MTLSize applyGrid =
            MTLSizeMake((cells + applyWidth - 1) / applyWidth, 1, 1);
        const MTLSize cellGroup = MTLSizeMake(8, 8, 1);
        const MTLSize cellGrid = MTLSizeMake((n + 7) / 8, (n + 7) / 8, 1);

        // clampDelta splits the accumulated delta into a relief-clamped
        // applied part + a residue that stays for later batches; applyApplied
        // folds the applied part into the heights.
        auto encodeClampApply = [&] {
            [enc setComputePipelineState:g.clampPso];
            [enc setBuffer:heightBuf offset:0 atIndex:0];
            [enc setBuffer:deltaBuf offset:0 atIndex:1];
            [enc setBytes:&u length:sizeof u atIndex:2];
            [enc setBuffer:appliedBuf offset:0 atIndex:3];
            [enc dispatchThreadgroups:cellGrid threadsPerThreadgroup:cellGroup];
            [enc setComputePipelineState:g.applyAppliedPso];
            [enc setBuffer:heightBuf offset:0 atIndex:0];
            [enc setBytes:&u length:sizeof u atIndex:2];
            [enc setBuffer:appliedBuf offset:0 atIndex:3];
            [enc dispatchThreadgroups:applyGrid threadsPerThreadgroup:applyGroup];
        };

        // --- Hydraulic: fixed batches of droplets. Between batches the
        // accumulated delta is clamped to per-cell relief (the batch-granular
        // form of the CPU's erode<=-dh / deposit<=dh caps — without it,
        // droplets converging on one cell sum unboundedly) and applied; the
        // overflow stays pending and drains as the relief updates.
        const NSUInteger dropWidth = std::min<NSUInteger>(
            g.dropletsPso.maxTotalThreadsPerThreadgroup, 256);
        const int batchSize = dropletBatchSize(n);
        for (int start = 0; start < p.droplets; start += batchSize) {
            u.batchStart = start;
            u.batchCount = std::min(batchSize, p.droplets - start);
            [enc setComputePipelineState:g.dropletsPso];
            [enc setBuffer:heightBuf offset:0 atIndex:0];
            [enc setBuffer:deltaBuf offset:0 atIndex:1];
            [enc setBytes:&u length:sizeof u atIndex:2];
            [enc setBuffer:brushOffBuf offset:0 atIndex:3];
            [enc setBuffer:brushWBuf offset:0 atIndex:4];
            [enc dispatchThreadgroups:MTLSizeMake((u.batchCount + dropWidth - 1) /
                                                      dropWidth,
                                                  1, 1)
                threadsPerThreadgroup:MTLSizeMake(dropWidth, 1, 1)];
            encodeClampApply();
        }

        // Drain what the per-batch clamps left pending (a fixed pass count —
        // part of the simulation's definition, like the batch size).
        constexpr int kResidueFlushPasses = 8;
        for (int it = 0; it < kResidueFlushPasses; it++) encodeClampApply();

        // Whatever still hasn't fit after the flush is discarded, so thermal's
        // plain (unclamped) apply never folds hydraulic overflow into the map.
        [enc endEncoding];
        id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
        [blit fillBuffer:deltaBuf
                   range:NSMakeRange(0, cells * sizeof(int32_t))
                   value:0];
        [blit endEncoding];
        enc = [cmd computeCommandEncoder];
        if (!enc) return false;

        // --- Thermal: scatter + plain apply, one pass per iteration. No
        // clamp here: one thread per cell, the identical computation to the
        // CPU's per-iteration delta pass.
        for (int it = 0; it < p.thermalIterations; it++) {
            [enc setComputePipelineState:g.thermalPso];
            [enc setBuffer:heightBuf offset:0 atIndex:0];
            [enc setBuffer:deltaBuf offset:0 atIndex:1];
            [enc setBytes:&u length:sizeof u atIndex:2];
            [enc dispatchThreadgroups:cellGrid threadsPerThreadgroup:cellGroup];
            [enc setComputePipelineState:g.applyPso];
            [enc setBuffer:heightBuf offset:0 atIndex:0];
            [enc setBuffer:deltaBuf offset:0 atIndex:1];
            [enc setBytes:&u length:sizeof u atIndex:2];
            [enc dispatchThreadgroups:applyGrid threadsPerThreadgroup:applyGroup];
        }

        [enc endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];
        if (cmd.status != MTLCommandBufferStatusCompleted) {
            NSLog(@"erosion GPU command buffer failed: %@", cmd.error);
            return false;
        }
        std::memcpy(hm.h.data(), heightBuf.contents, cells * sizeof(float));
    }
    return true;
}

}  // namespace engine
