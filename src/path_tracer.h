#ifndef RAYTRACER_PATH_TRACER_H
#define RAYTRACER_PATH_TRACER_H

#include "image.h"
#include "scene.h"
#include "camera.h"
#include "job_system.h"
#include <atomic>
#include <functional>

namespace engine {

// Everything the render needs that isn't the scene or camera. Passed by value
// so a render is fully described by (scene, camera, config) — no global state,
// so the function is reentrant and safe to drive from any thread (the editor
// runs it on a worker; the CLI on main).
struct RenderConfig {
    int imageSize = 512;
    int samplesPerPixel = 128;
    double exposure = 1.0;      // linear scale applied before the ACES/gamma curve
    int maxBounces = 10;
};

// Fraction in [0,1], reported as scanlines complete. Invoked from the thread
// that called renderImage (not a worker), so a host can marshal it to its UI.
using ProgressFn = std::function<void(float)>;

// Render `scene` through `camera` into a fresh Image, parallelized by scanline
// on `jobs`. onProgress (if set) fires as rows finish; `cancel` (if set) is
// polled per row so a host can stop early — the returned image is then partial
// (unrendered rows stay black). Pure compute, headless: the offline CLI and the
// editor's in-process render share this one path.
Image renderImage(const Scene& scene, const Camera& camera,
                  const RenderConfig& config, JobSystem& jobs,
                  const ProgressFn& onProgress = {},
                  const std::atomic<bool>* cancel = nullptr);

}  // namespace engine

#endif
