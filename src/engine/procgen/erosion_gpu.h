#ifndef RAYTRACER_ENGINE_EROSION_GPU_H
#define RAYTRACER_ENGINE_EROSION_GPU_H

// GPU erosion bake seam (metropolis-scale-plan P0.5). Pure C++ header — the
// Metal host code lives in erosion_gpu.mm (Apple only) and is fully
// self-contained: its own MTLDevice + queue, kernels compiled at runtime from
// shaders/metal/erosion.metal (read relative to CWD, the same convention as
// the renderer's shaders). This is a load-time content bake with no renderer
// handle in reach, hence no dependency on the Metal renderer.
//
// The GPU sim is run-to-run deterministic (fixed batches, snapshot reads,
// fixed-point integer scatter — see erosion.metal) but NOT bit-identical to
// the sequential CPU sim; the cache key's backend tag (erosionBackendTag)
// keeps the two bakes from ever sharing a cache entry.

namespace engine {

struct Heightmap;
struct ErosionParams;

// RT_NO_GPU_EROSION: a build that doesn't compile erosion_gpu.mm (the plain
// Makefile — offline tracer + Jolt-free unit tests) forces the CPU stubs even
// on Apple, instead of failing to link.
#if defined(__APPLE__) && !defined(RT_NO_GPU_EROSION)
// True when a Metal device exists and the erosion kernels compiled. A missing
// kernel file (e.g. run from outside the repo root) is probed again on the
// next call; a real compile failure is cached.
bool erodeGpuAvailable();

// Run the full droplet + thermal simulation on the GPU. Returns false on any
// failure (no device, missing/broken kernel source, allocation) with the
// heightmap untouched, so the caller can fall through to the CPU sim.
bool erodeGpu(Heightmap& hm, const ErosionParams& params);
#else
inline bool erodeGpuAvailable() { return false; }
inline bool erodeGpu(Heightmap&, const ErosionParams&) { return false; }
#endif

}  // namespace engine

#endif
