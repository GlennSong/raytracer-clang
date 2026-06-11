#include "rt_math.h"
#include "image.h"
#include "camera.h"
#include "scene.h"
#include "job_system.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdlib>
#include <string>

using namespace engine;  // namespace migration (ADR-0015)

const int IMAGE_SIZE = 512;
const int SAMPLES_PER_PIXEL = 128;
const int MAX_BOUNCES = 10;

Scene buildCornellBox() {
    Scene scene;

    int white  = scene.addMaterial(Material::diffuse(Vec3(0.73, 0.73, 0.73)));
    int red    = scene.addMaterial(Material::diffuse(Vec3(0.65, 0.05, 0.05)));
    int green  = scene.addMaterial(Material::diffuse(Vec3(0.12, 0.45, 0.15)));
    int light  = scene.addMaterial(Material::emissive(Vec3(1.0, 0.9, 0.7), 15.0));
    int mirror = scene.addMaterial(Material::metal(Vec3(0.95, 0.95, 0.95), 0.0));
    int glass  = scene.addMaterial(Material::glass(1.5));

    // Floor
    scene.addQuad(Vec3(0, 0, 0), Vec3(555, 0, 0), Vec3(0, 0, 555), white);
    // Ceiling
    scene.addQuad(Vec3(0, 555, 555), Vec3(555, 0, 0), Vec3(0, 0, -555), white);
    // Back wall
    scene.addQuad(Vec3(0, 0, 555), Vec3(555, 0, 0), Vec3(0, 555, 0), white);
    // Left wall (red)
    scene.addQuad(Vec3(0, 0, 0), Vec3(0, 0, 555), Vec3(0, 555, 0), red);
    // Right wall (green)
    scene.addQuad(Vec3(555, 0, 555), Vec3(0, 0, -555), Vec3(0, 555, 0), green);

    // Ceiling light
    scene.addQuad(Vec3(213, 554, 227), Vec3(130, 0, 0), Vec3(0, 0, 105), light);

    // Glass sphere (left side)
    scene.addSphere(Vec3(185, 100, 190), 100, glass);

    // Mirror sphere mesh (right side, tessellated — exercises the KD-tree)
    scene.addMeshSphere(Vec3(370, 100, 350), 100, mirror, 32, 64);

    scene.buildAccelerator();
    return scene;
}

void renderRow(const Scene& scene, const Camera& camera, Image& image,
               int y, std::atomic<int>& linesComplete) {
    double ca = camera.lens.chromaticAberration;
    for (int x = 0; x < IMAGE_SIZE; x++) {
        Vec3 color(0, 0, 0);
        for (int s = 0; s < SAMPLES_PER_PIXEL; s++) {
            double u = (x + randomDouble()) / (IMAGE_SIZE - 1);
            double v = (IMAGE_SIZE - 1 - y + randomDouble()) / (IMAGE_SIZE - 1);
            double lensU = randomDouble();
            double lensV = randomDouble();
            if (camera.hasChromaticAberration()) {
                // Lateral CA: the channels image at slightly different radial
                // scales, so trace one ray per channel through the same lens
                // sample and keep each ray's own channel.
                Vec3 r = scene.tracePath(
                    camera.generateRay(u, v, lensU, lensV, 1.0 + ca), MAX_BOUNCES);
                Vec3 g = scene.tracePath(
                    camera.generateRay(u, v, lensU, lensV, 1.0), MAX_BOUNCES);
                Vec3 b = scene.tracePath(
                    camera.generateRay(u, v, lensU, lensV, 1.0 - ca), MAX_BOUNCES);
                color += Vec3(r.x, g.y, b.z);
            } else {
                Ray ray = camera.generateRay(u, v, lensU, lensV);
                color += scene.tracePath(ray, MAX_BOUNCES);
            }
        }
        color = color / static_cast<double>(SAMPLES_PER_PIXEL);

        // Stylized vignette: quadratic radial falloff scaled by the parameter.
        if (camera.lens.vignette != 0.0) {
            double sx = 2.0 * x / (IMAGE_SIZE - 1) - 1.0;
            double sy = 2.0 * y / (IMAGE_SIZE - 1) - 1.0;
            double fall = 1.0 - camera.lens.vignette * (sx * sx + sy * sy) * 0.5;
            color = color * std::max(fall, 0.0);
        }

        color = Vec3(std::sqrt(color.x), std::sqrt(color.y), std::sqrt(color.z));
        color = clampVec(color, 0.0, 1.0);

        image.setPixel(x, y, color);
    }
    linesComplete.fetch_add(1);
}

// Lens flags (docs/virtual-camera-plan.md, Phase 3). The Cornell box is ~555
// units across, so --scale 100 maps meter-scale optics onto it; with that,
// try: ./raytracer --fstop 1.0 --focus 800 --scale 100
LensParams parseLensFlags(int argc, char** argv, double& worldUnitsPerMeter) {
    LensParams lens;
    lens.fStop = 0.0;  // pinhole unless --fstop is given
    lens.focusDistance = 800.0;
    auto next = [&](int& i) { return std::atof(argv[++i]); };
    for (int i = 1; i < argc - 1; i++) {
        std::string flag = argv[i];
        if (flag == "--fstop")         lens.fStop = next(i);
        else if (flag == "--focal")    lens.focalLength = next(i);
        else if (flag == "--focus")    lens.focusDistance = next(i);
        else if (flag == "--k1")       lens.distortionK1 = next(i);
        else if (flag == "--k2")       lens.distortionK2 = next(i);
        else if (flag == "--ca")       lens.chromaticAberration = next(i);
        else if (flag == "--vignette") lens.vignette = next(i);
        else if (flag == "--scale")    worldUnitsPerMeter = next(i);
    }
    return lens;
}

int main(int argc, char** argv) {
    Image image(IMAGE_SIZE, IMAGE_SIZE);

    double worldUnitsPerMeter = 100.0;
    LensParams lens = parseLensFlags(argc, argv, worldUnitsPerMeter);
    Camera camera(
        Vec3(278, 278, -800),
        Vec3(278, 278, 0),
        Vec3(0, 1, 0),
        40.0,
        1.0,
        lens,
        worldUnitsPerMeter
    );
    if (lens.fStop > 0.0)
        std::cerr << "Lens: f/" << lens.fStop << " @ " << lens.focalLength
                  << "mm, focus " << lens.focusDistance << "\n";

    std::cerr << "Building scene...\n";
    Scene scene = buildCornellBox();
    std::cerr << "  Triangles: " << scene.triangles.size() << "\n";
    std::cerr << "  Quads: " << scene.quads.size() << "\n";
    std::cerr << "  Spheres: " << scene.spheres.size() << "\n";

    JobSystem jobs;  // one shared pool owns the threads; rows are the tasks
    std::cerr << "Rendering with " << jobs.workerCount() << " worker threads, "
              << SAMPLES_PER_PIXEL << " spp...\n";

    std::atomic<int> linesComplete(0);
    std::atomic<int> renderCounter(0);
    for (int y = 0; y < IMAGE_SIZE; y++) {
        jobs.run([&scene, &camera, &image, y, &linesComplete] {
            renderRow(scene, camera, image, y, linesComplete);
        }, &renderCounter);
    }

    // The main thread reports progress while the pool renders. (In synchronous
    // mode the rows have already run during dispatch, so this loop is skipped.)
    while (linesComplete.load() < IMAGE_SIZE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int done = linesComplete.load();
        int pct = (done * 100) / IMAGE_SIZE;
        std::cerr << "\r  Progress: " << pct << "% (" << done << "/" << IMAGE_SIZE << ")  " << std::flush;
    }
    jobs.wait(&renderCounter);

    std::cerr << "\r  Progress: 100%                \n";
    std::cerr << "Done rendering.\n";

    image.writePpm("output_raw.ppm");
    std::cout << "Wrote output_raw.ppm (unfiltered)\n";

    std::cerr << "Applying bilateral filter...\n";
    Image denoised = image.bilateralFilter(5, 3.0, 0.1);
    denoised.writePpm("output.ppm");
    std::cout << "Wrote output.ppm (denoised, " << IMAGE_SIZE << "x" << IMAGE_SIZE
              << ", " << SAMPLES_PER_PIXEL << " spp)\n";
    return 0;
}
