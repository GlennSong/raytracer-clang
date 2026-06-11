#include "rt_math.h"
#include "image.h"
#include "camera.h"
#include "scene.h"
#include "level_scene.h"
#include "job_system.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdlib>
#include <string>
#include <algorithm>

using namespace engine;  // namespace migration (ADR-0015)

int IMAGE_SIZE = 512;          // --size
int SAMPLES_PER_PIXEL = 128;   // --spp
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

// CLI (docs/virtual-camera-plan.md, Phases 3 + 5). Two modes:
//   Cornell box (default): lens flags apply directly. The box is ~555 units
//   across, so --scale 100 maps meter-scale optics onto it; try:
//     ./raytracer --fstop 1.0 --focus 800 --scale 100
//   Level mode: renders a viewer level through a camera placed interactively
//   (the sidecar written on viewer quit / by the Cameras panel):
//     ./raytracer --level assets/levels/arena.json --camera "Camera 1"
struct CliOptions {
    std::string level;        // empty = Cornell box
    std::string cameraName;   // empty = first camera in the sidecar
    std::string outFile = "output.ppm";
    double worldUnitsPerMeter = 0.0;   // 0 = mode default (1 level, 100 Cornell)
    LensParams lens;
    bool showHelp = false;
};

CliOptions parseCli(int argc, char** argv) {
    CliOptions opt;
    opt.lens.fStop = 0.0;  // pinhole unless --fstop is given
    opt.lens.focusDistance = 800.0;
    auto next = [&](int& i) { return std::atof(argv[++i]); };
    for (int i = 1; i < argc; i++) {
        std::string flag = argv[i];
        bool hasValue = i < argc - 1;
        if (flag == "--help" || flag == "-h")     opt.showHelp = true;
        else if (!hasValue)                       continue;
        else if (flag == "--level")    opt.level = argv[++i];
        else if (flag == "--camera")   opt.cameraName = argv[++i];
        else if (flag == "--out")      opt.outFile = argv[++i];
        else if (flag == "--size")     IMAGE_SIZE = std::max(16, atoi(argv[++i]));
        else if (flag == "--spp")      SAMPLES_PER_PIXEL = std::max(1, atoi(argv[++i]));
        else if (flag == "--fstop")    opt.lens.fStop = next(i);
        else if (flag == "--focal")    opt.lens.focalLength = next(i);
        else if (flag == "--focus")    opt.lens.focusDistance = next(i);
        else if (flag == "--k1")       opt.lens.distortionK1 = next(i);
        else if (flag == "--k2")       opt.lens.distortionK2 = next(i);
        else if (flag == "--ca")       opt.lens.chromaticAberration = next(i);
        else if (flag == "--vignette") opt.lens.vignette = next(i);
        else if (flag == "--scale")    opt.worldUnitsPerMeter = next(i);
    }
    return opt;
}

void printUsage() {
    std::cerr <<
        "Usage: raytracer [options]\n"
        "  --level <file>    render a viewer level instead of the Cornell box\n"
        "  --camera <name>   placed camera from <level>.cameras.json (default: first)\n"
        "  --out <file>      output image (default output.ppm)\n"
        "  --size N          image size (default 512)\n"
        "  --spp N           samples per pixel (default 128)\n"
        "Lens (Cornell mode; level cameras carry their own lens):\n"
        "  --fstop N --focal MM --focus D --k1 K --k2 K --ca C --vignette V\n"
        "  --scale U         world units per meter (default: 1 level, 100 Cornell)\n";
}

int main(int argc, char** argv) {
    CliOptions opt = parseCli(argc, argv);
    if (opt.showHelp) {
        printUsage();
        return 0;
    }

    Scene scene;
    Vec3 lookFrom(278, 278, -800), lookAt(278, 278, 0);
    double fovDegrees = 40.0;
    LensParams lens = opt.lens;
    double scale = opt.worldUnitsPerMeter;

    if (!opt.level.empty()) {
        if (!LevelScene::load(opt.level, scene)) return 1;
        SidecarCamera cam;
        if (!loadSidecarCamera(opt.level, opt.cameraName, cam)) return 1;
        lookFrom = cam.position;
        lookAt = cam.position + cam.forward;
        lens = cam.lens;                       // the placed camera's lens wins
        fovDegrees = lens.verticalFovDegrees();
        if (scale <= 0.0) scale = 1.0;         // levels are meter-scale
        std::cerr << "Rendering \"" << cam.name << "\" in " << opt.level << "\n";
    } else {
        std::cerr << "Building scene...\n";
        scene = buildCornellBox();
        if (scale <= 0.0) scale = 100.0;       // Cornell is ~555 units across
    }

    Image image(IMAGE_SIZE, IMAGE_SIZE);
    Camera camera(lookFrom, lookAt, Vec3(0, 1, 0), fovDegrees, 1.0, lens, scale);
    if (lens.fStop > 0.0)
        std::cerr << "Lens: f/" << lens.fStop << " @ " << lens.focalLength
                  << "mm, focus " << lens.focusDistance << "\n";

    std::cerr << "  Triangles: " << scene.triangles.size()
              << ", quads: " << scene.quads.size()
              << ", spheres: " << scene.spheres.size() << "\n";

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

    std::cerr << "Applying bilateral filter...\n";
    Image denoised = image.bilateralFilter(5, 3.0, 0.1);
    denoised.writePpm(opt.outFile);
    std::cout << "Wrote " << opt.outFile << " (denoised, " << IMAGE_SIZE << "x"
              << IMAGE_SIZE << ", " << SAMPLES_PER_PIXEL << " spp)\n";
    return 0;
}
