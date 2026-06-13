#include "path_tracer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace engine {

namespace {

// One scanline of the image. Identical math to the original inline renderRow:
// per-pixel multisampling, optional chromatic aberration (one ray per channel),
// exposure, the stylized vignette, then the viewer's ACES + 2.2 gamma display
// transform so offline renders match the realtime look.
void renderRow(const Scene& scene, const Camera& camera, Image& image,
               const RenderConfig& cfg, int y) {
    const int size = cfg.imageSize;
    const double ca = camera.lens.chromaticAberration;
    for (int x = 0; x < size; x++) {
        Vec3 color(0, 0, 0);
        for (int s = 0; s < cfg.samplesPerPixel; s++) {
            double u = (x + randomDouble()) / (size - 1);
            double v = (size - 1 - y + randomDouble()) / (size - 1);
            double lensU = randomDouble();
            double lensV = randomDouble();
            if (camera.hasChromaticAberration()) {
                // Lateral CA: the channels image at slightly different radial
                // scales, so trace one ray per channel through the same lens
                // sample and keep each ray's own channel.
                Vec3 r = scene.tracePath(
                    camera.generateRay(u, v, lensU, lensV, 1.0 + ca), cfg.maxBounces);
                Vec3 g = scene.tracePath(
                    camera.generateRay(u, v, lensU, lensV, 1.0), cfg.maxBounces);
                Vec3 b = scene.tracePath(
                    camera.generateRay(u, v, lensU, lensV, 1.0 - ca), cfg.maxBounces);
                color += Vec3(r.x, g.y, b.z);
            } else {
                Ray ray = camera.generateRay(u, v, lensU, lensV);
                color += scene.tracePath(ray, cfg.maxBounces);
            }
        }
        color = (color / static_cast<double>(cfg.samplesPerPixel)) * cfg.exposure;

        // Stylized vignette: quadratic radial falloff scaled by the parameter.
        if (camera.lens.vignette != 0.0) {
            double sx = 2.0 * x / (size - 1) - 1.0;
            double sy = 2.0 * y / (size - 1) - 1.0;
            double fall = 1.0 - camera.lens.vignette * (sx * sx + sy * sy) * 0.5;
            color = color * std::max(fall, 0.0);
        }

        auto aces = [](double val) {
            val = (val * (2.51 * val + 0.03)) / (val * (2.43 * val + 0.59) + 0.14);
            return std::pow(std::clamp(val, 0.0, 1.0), 1.0 / 2.2);
        };
        color = Vec3(aces(color.x), aces(color.y), aces(color.z));

        image.setPixel(x, y, color);
    }
}

}  // namespace

Image renderImage(const Scene& scene, const Camera& camera,
                  const RenderConfig& config, JobSystem& jobs,
                  const ProgressFn& onProgress, const std::atomic<bool>* cancel) {
    const int size = config.imageSize;
    Image image(size, size);

    std::atomic<int> linesComplete(0);
    std::atomic<int> counter(0);
    for (int y = 0; y < size; y++) {
        jobs.run([&scene, &camera, &config, &image, y, &linesComplete, cancel] {
            // A cancelled render lets the remaining rows finish instantly
            // (leaving them black) so the wait below still drains cleanly.
            if (!cancel || !cancel->load())
                renderRow(scene, camera, image, config, y);
            linesComplete.fetch_add(1);
        }, &counter);
    }

    // Report progress from the calling thread while the pool renders. In
    // synchronous mode (0 workers) the rows already ran during dispatch, so this
    // loop falls straight through to the final 100% tick.
    while (linesComplete.load() < size) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (onProgress)
            onProgress(static_cast<float>(linesComplete.load()) / size);
    }
    jobs.wait(&counter);
    if (onProgress) onProgress(1.0f);
    return image;
}

}  // namespace engine
