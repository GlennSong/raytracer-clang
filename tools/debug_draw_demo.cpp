// DebugDraw demo (ADR-0067), headless: scribbles the full gizmo set into a
// DebugDraw, bakes it with the SAME buildDebugLineMesh ribbon path the viewer
// draws, and renders those triangles as emissive geometry through the offline
// path tracer — the production line geometry, seen end to end without a GPU.
// Writes debug_draw_demo.png. Build: the `debug_draw_demo` CMake target.
#include "camera.h"
#include "engine/debug_draw.h"
#include "image.h"
#include "job_system.h"
#include "material.h"
#include "path_tracer.h"
#include "scene.h"

#include <cstdio>
#include <map>
#include <tuple>

using namespace engine;

int main() {
    // --- the gizmo set a system would scribble into ctx.debug ---------------
    DebugDraw dd;
    dd.axes(Mat4::trs(Vec3(0, 0.02, 0), Quat(), Vec3(1, 1, 1)), 1.8);
    dd.sphere(Vec3(-2.6, 1.2, 0), 1.1, Vec3(0.3, 1.0, 0.4));
    dd.box(Vec3(2.6, 1.0, 0), Vec3(0.85, 0.85, 0.85),
           Quat::fromAxisAngle(Vec3(0, 1, 0), 0.5), Vec3(1.0, 0.6, 0.15));
    dd.arrow(Vec3(-1.4, 2.2, 0), Vec3(1.6, 2.2, 0), Vec3(0.3, 0.8, 1.0));
    dd.circle(Vec3(0, 0.02, 0), Vec3(0, 1, 0), 4.2, Vec3(0.55, 0.55, 0.6));
    dd.aabb(Vec3(-0.6, 0.02, -3.4), Vec3(0.6, 1.2, -2.2), Vec3(1.0, 0.3, 0.9));
    std::printf("DebugDraw: %zu lines\n", dd.lineCount());

    // --- bake to the production camera-facing ribbon mesh -------------------
    Vec3 cameraPos(6.5, 5.0, 8.0);
    Vec3 cameraTarget(0, 1.0, -0.4);
    // A slightly fatter pixelScale than the in-engine default so the lines
    // read well at this render size.
    RenderMesh ribbons = buildDebugLineMesh(dd.lines(), cameraPos, 0.004);
    std::printf("Ribbon mesh: %zu vertices, %zu triangles\n",
                ribbons.vertices.size(), ribbons.indices.size() / 3);

    // --- path-trace the ribbons as emissive triangles ------------------------
    Scene scene;
    int backdrop = scene.addMaterial(Material::diffuse(Vec3(0.05, 0.05, 0.06)));
    scene.addQuad(Vec3(-40, 0, -40), Vec3(80, 0, 0), Vec3(0, 0, 80), backdrop);
    int skyLight = scene.addMaterial(Material::emissive(Vec3(0.5, 0.55, 0.65), 0.35));
    scene.addQuad(Vec3(-40, 18, -40), Vec3(80, 0, 0), Vec3(0, 0, 80), skyLight);

    // One emissive material per distinct line color (vertex color carries it).
    std::map<std::tuple<float, float, float>, int> materialByColor;
    for (size_t i = 0; i + 2 < ribbons.indices.size(); i += 3) {
        const Vertex& a = ribbons.vertices[ribbons.indices[i]];
        const Vertex& b = ribbons.vertices[ribbons.indices[i + 1]];
        const Vertex& c = ribbons.vertices[ribbons.indices[i + 2]];
        auto key = std::make_tuple(static_cast<float>(a.color.x),
                                   static_cast<float>(a.color.y),
                                   static_cast<float>(a.color.z));
        auto it = materialByColor.find(key);
        if (it == materialByColor.end()) {
            int mat = scene.addMaterial(Material::emissive(a.color, 3.0));
            it = materialByColor.emplace(key, mat).first;
        }
        scene.addTriangle(a.position, b.position, c.position, it->second);
    }
    scene.buildAccelerator();

    Camera camera(cameraPos, cameraTarget, Vec3(0, 1, 0), 42.0, 1.0);
    JobSystem jobs;
    RenderConfig cfg;
    cfg.imageSize = 640;
    cfg.samplesPerPixel = 48;
    Image img = renderImage(scene, camera, cfg, jobs);
    img.writePng("debug_draw_demo.png");
    std::printf("Wrote debug_draw_demo.png (%dx%d)\n", img.width, img.height);
    return 0;
}
