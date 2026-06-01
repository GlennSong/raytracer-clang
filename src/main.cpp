#include "math.h"
#include "image.h"
#include "camera.h"
#include "scene.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>

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

void renderTile(const Scene& scene, const Camera& camera, Image& image,
                int yStart, int yEnd, std::atomic<int>& linesComplete) {
    for (int y = yStart; y < yEnd; y++) {
        for (int x = 0; x < IMAGE_SIZE; x++) {
            Vec3 color(0, 0, 0);
            for (int s = 0; s < SAMPLES_PER_PIXEL; s++) {
                double u = (x + randomDouble()) / (IMAGE_SIZE - 1);
                double v = (IMAGE_SIZE - 1 - y + randomDouble()) / (IMAGE_SIZE - 1);
                Ray ray = camera.generateRay(u, v);
                color += scene.tracePath(ray, MAX_BOUNCES);
            }
            color = color / static_cast<double>(SAMPLES_PER_PIXEL);

            color = Vec3(std::sqrt(color.x), std::sqrt(color.y), std::sqrt(color.z));
            color = clampVec(color, 0.0, 1.0);

            image.setPixel(x, y, color);
        }
        linesComplete.fetch_add(1);
    }
}

int main() {
    Image image(IMAGE_SIZE, IMAGE_SIZE);

    Camera camera(
        Vec3(278, 278, -800),
        Vec3(278, 278, 0),
        Vec3(0, 1, 0),
        40.0,
        1.0
    );

    std::cerr << "Building scene...\n";
    Scene scene = buildCornellBox();
    std::cerr << "  Triangles: " << scene.triangles.size() << "\n";
    std::cerr << "  Quads: " << scene.quads.size() << "\n";
    std::cerr << "  Spheres: " << scene.spheres.size() << "\n";

    int numThreads = std::max(1u, std::thread::hardware_concurrency());
    std::cerr << "Rendering with " << numThreads << " threads, "
              << SAMPLES_PER_PIXEL << " spp...\n";

    std::atomic<int> linesComplete(0);
    std::vector<std::thread> threads;
    int rowsPerThread = IMAGE_SIZE / numThreads;

    for (int t = 0; t < numThreads; t++) {
        int yStart = t * rowsPerThread;
        int yEnd = (t == numThreads - 1) ? IMAGE_SIZE : yStart + rowsPerThread;
        threads.emplace_back(renderTile, std::cref(scene), std::cref(camera),
                             std::ref(image), yStart, yEnd, std::ref(linesComplete));
    }

    while (linesComplete.load() < IMAGE_SIZE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int done = linesComplete.load();
        int pct = (done * 100) / IMAGE_SIZE;
        std::cerr << "\r  Progress: " << pct << "% (" << done << "/" << IMAGE_SIZE << ")  " << std::flush;
    }

    for (auto& t : threads) {
        t.join();
    }

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
