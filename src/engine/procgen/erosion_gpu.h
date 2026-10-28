#ifndef RAYTRACER_ENGINE_EROSION_GPU_H
#define RAYTRACER_ENGINE_EROSION_GPU_H

// GPU erosion seam (ADR-0122): the Vulkan compute host in erosion_gpu_vk.cpp -- its own device and queue,
// kernels loaded from the build's SPIR-V. A load-time content bake with no renderer handle in reach, hence
// no dependency on the renderer. Run-to-run deterministic (fixed-point integer scatter) but NOT
// bit-identical to the CPU, so the cache key's backend tag (erosionBackendTag) keeps the two apart.
// ADR-0257 retired the droplet sim and with it the Metal port; elsewhere the CPU runs thermal alone.

namespace engine {

struct Heightmap;
struct ErosionParams;

// RT_NO_GPU_EROSION: a build that doesn't compile erosion_gpu_vk.cpp (the plain Makefile -- offline
// tracer + Jolt-free unit tests) takes the CPU stubs instead of failing to link.
#if defined(RT_HAVE_VULKAN_EROSION) && !defined(RT_NO_GPU_EROSION)
// True when a Vulkan device exists and the erosion kernels loaded. A missing kernel file (e.g. run from
// outside the build) is probed again on the next call; a real failure is cached.
bool erodeGpuAvailable();

// Run breach + water + thermal on the GPU. Returns false on any failure (no device, missing kernel,
// allocation) with the heightmap untouched, so the caller can fall through to the CPU.
bool erodeGpu(Heightmap& hm, const ErosionParams& params);
#else
inline bool erodeGpuAvailable() { return false; }
inline bool erodeGpu(Heightmap&, const ErosionParams&) { return false; }
#endif

}  // namespace engine

#endif
