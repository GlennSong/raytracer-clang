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

        // Fitted ACES filmic tonemap (Stephen Hill). The RGB input/output
        // matrices preserve saturation — a per-channel ACES curve washes colors
        // out (shifts hue, pulls brights toward white). Then sRGB-ish 2.2 gamma.
        auto mul = [](const double m[3][3], const Vec3& c) {
            return Vec3(m[0][0] * c.x + m[0][1] * c.y + m[0][2] * c.z,
                        m[1][0] * c.x + m[1][1] * c.y + m[1][2] * c.z,
                        m[2][0] * c.x + m[2][1] * c.y + m[2][2] * c.z);
        };
        static const double ACESin[3][3] = {{0.59719, 0.35458, 0.04823},
                                            {0.07600, 0.90834, 0.01566},
                                            {0.02840, 0.13383, 0.83777}};
        static const double ACESout[3][3] = {{1.60475, -0.53108, -0.07367},
                                             {-0.10208, 1.10813, -0.00605},
                                             {-0.00327, -0.07276, 1.07602}};
        auto fit = [](double v) {
            double a = v * (v + 0.0245786) - 0.000090537;
            double b = v * (0.983729 * v + 0.432951) + 0.238081;
            return a / b;
        };
        Vec3 c = mul(ACESin, color);
        c = Vec3(fit(c.x), fit(c.y), fit(c.z));
        c = mul(ACESout, c);
        auto gamma = [](double v) {
            return std::pow(std::clamp(v, 0.0, 1.0), 1.0 / 2.2);
        };
        color = Vec3(gamma(c.x), gamma(c.y), gamma(c.z));

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
