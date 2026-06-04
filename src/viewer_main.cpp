#include "math.h"
#include "renderer/renderer.h"
#include "engine/world.h"
#include "engine/components.h"
#include "engine/application.h"
#include "engine/systems/dev_control_system.h"
#include "engine/systems/camera_system.h"
#include "engine/systems/motion_system.h"
#include "engine/systems/render_system.h"
#include "engine/systems/debug_overlay_system.h"
#include "log.h"

RenderMesh createQuadMesh(Vec3 corner, Vec3 edge1, Vec3 edge2) {
    RenderMesh mesh;
    Vec3 normal = -normalize(cross(edge1, edge2));

    mesh.vertices = {
        Vertex(corner, normal),
        Vertex(corner + edge1, normal),
        Vertex(corner + edge1 + edge2, normal),
        Vertex(corner + edge2, normal)
    };
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return mesh;
}

RenderMesh createSphereMesh(float radius, int stacks, int slices) {
    RenderMesh mesh;
    for (int i = 0; i <= stacks; i++) {
        float theta = PI * i / stacks;
        for (int j = 0; j <= slices; j++) {
            float phi = 2.0 * PI * j / slices;
            float x = radius * std::sin(theta) * std::cos(phi);
            float y = radius * std::cos(theta);
            float z = radius * std::sin(theta) * std::sin(phi);
            Vec3 pos(x, y, z);
            Vec3 norm = normalize(pos);
            float u = static_cast<float>(j) / slices;
            float v = static_cast<float>(i) / stacks;
            mesh.vertices.push_back(Vertex(pos, norm, u, v));
        }
    }

    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < slices; j++) {
            int a = i * (slices + 1) + j;
            int b = a + slices + 1;
            mesh.indices.push_back(a);
            mesh.indices.push_back(b);
            mesh.indices.push_back(a + 1);
            mesh.indices.push_back(b);
            mesh.indices.push_back(b + 1);
            mesh.indices.push_back(a + 1);
        }
    }
    return mesh;
}

RenderMesh createBoxMesh(Vec3 size) {
    RenderMesh mesh;
    float hx = static_cast<float>(size.x * 0.5);
    float hy = static_cast<float>(size.y * 0.5);
    float hz = static_cast<float>(size.z * 0.5);

    struct Face { Vec3 normal; Vec3 verts[4]; };
    Face faces[] = {
        {Vec3(0,0,-1), {Vec3(-hx,-hy,-hz), Vec3(hx,-hy,-hz), Vec3(hx,hy,-hz), Vec3(-hx,hy,-hz)}},
        {Vec3(0,0,1),  {Vec3(hx,-hy,hz), Vec3(-hx,-hy,hz), Vec3(-hx,hy,hz), Vec3(hx,hy,hz)}},
        {Vec3(-1,0,0), {Vec3(-hx,-hy,hz), Vec3(-hx,-hy,-hz), Vec3(-hx,hy,-hz), Vec3(-hx,hy,hz)}},
        {Vec3(1,0,0),  {Vec3(hx,-hy,-hz), Vec3(hx,-hy,hz), Vec3(hx,hy,hz), Vec3(hx,hy,-hz)}},
        {Vec3(0,-1,0), {Vec3(-hx,-hy,hz), Vec3(hx,-hy,hz), Vec3(hx,-hy,-hz), Vec3(-hx,-hy,-hz)}},
        {Vec3(0,1,0),  {Vec3(-hx,hy,-hz), Vec3(hx,hy,-hz), Vec3(hx,hy,hz), Vec3(-hx,hy,hz)}}
    };

    for (auto& f : faces) {
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int i = 0; i < 4; i++)
            mesh.vertices.push_back(Vertex(f.verts[i], f.normal));
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 1);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base + 3);
    }
    return mesh;
}

// Populates the Cornell-box scene: static room + spheres and a spinning box,
// plus the lights the render system reads from the shared RenderView.
static void buildScene(World& world, Renderer& renderer, RenderView& view) {
    // Every renderable carries a PrevTransform so the render system can use one
    // uniform query; static entities simply never have it updated.
    auto addStatic = [&](MeshHandle mesh, const Vec3& position,
                         const RenderMaterial& material) {
        Entity e = world.create();
        Transform t;
        t.position = position;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable r;
        r.mesh = mesh;
        r.material = material;
        world.add<Renderable>(e, r);
    };

    RenderMaterial whiteMat;
    whiteMat.albedo = Vec3(0.73, 0.73, 0.73);
    whiteMat.roughness = 0.9f;

    RenderMaterial redWall;
    redWall.albedo = Vec3(0.65, 0.05, 0.05);
    redWall.roughness = 0.9f;

    RenderMaterial greenWall;
    greenWall.albedo = Vec3(0.12, 0.45, 0.15);
    greenWall.roughness = 0.9f;

    // Walls bake their position into the mesh geometry, so the entity transform
    // stays at the origin.
    addStatic(renderer.uploadMesh(createQuadMesh(Vec3(-5, 0, -5), Vec3(10, 0, 0), Vec3(0, 0, 10))),
              Vec3(0, 0, 0), whiteMat);
    addStatic(renderer.uploadMesh(createQuadMesh(Vec3(-5, 8, 5), Vec3(10, 0, 0), Vec3(0, 0, -10))),
              Vec3(0, 0, 0), whiteMat);
    addStatic(renderer.uploadMesh(createQuadMesh(Vec3(-5, 0, 5), Vec3(10, 0, 0), Vec3(0, 8, 0))),
              Vec3(0, 0, 0), whiteMat);
    addStatic(renderer.uploadMesh(createQuadMesh(Vec3(-5, 0, -5), Vec3(0, 0, 10), Vec3(0, 8, 0))),
              Vec3(0, 0, 0), redWall);
    addStatic(renderer.uploadMesh(createQuadMesh(Vec3(5, 0, 5), Vec3(0, 0, -10), Vec3(0, 8, 0))),
              Vec3(0, 0, 0), greenWall);

    MeshHandle sphereHandle = renderer.uploadMesh(createSphereMesh(1.0f, 32, 64));

    RenderMaterial redMat;
    redMat.albedo = Vec3(0.8, 0.1, 0.1);
    redMat.roughness = 0.3f;
    addStatic(sphereHandle, Vec3(-2.0, 1.0, 0.0), redMat);

    RenderMaterial metalMat;
    metalMat.albedo = Vec3(0.9, 0.9, 0.95);
    metalMat.metallic = 1.0f;
    metalMat.roughness = 0.1f;
    addStatic(sphereHandle, Vec3(0.0, 1.0, 0.0), metalMat);

    RenderMaterial glassMat;
    glassMat.albedo = Vec3(0.5, 0.8, 1.0);
    glassMat.roughness = 0.1f;
    glassMat.opacity = 0.3f;
    addStatic(sphereHandle, Vec3(2.0, 1.0, 0.0), glassMat);

    // Simulated: has a Velocity, so MotionSystem spins it in place — making the
    // fixed timestep and time controls visible.
    RenderMaterial greenMat;
    greenMat.albedo = Vec3(0.2, 0.7, 0.2);
    greenMat.roughness = 0.5f;
    Entity box = world.create();
    Transform boxTransform;
    boxTransform.position = Vec3(3.0, 1.5, 2.0);
    boxTransform.rotation = Vec3(0.0, 0.4, 0.0);
    world.add<Transform>(box, boxTransform);
    world.add<PrevTransform>(box, PrevTransform{boxTransform});
    Renderable boxRender;
    boxRender.mesh = renderer.uploadMesh(createBoxMesh(Vec3(1.5, 3.0, 1.5)));
    boxRender.material = greenMat;
    world.add<Renderable>(box, boxRender);
    Velocity boxVelocity;
    boxVelocity.angular = Vec3(0.0, 0.6, 0.3);
    world.add<Velocity>(box, boxVelocity);

    view.lights = {
        PointLight(Vec3(0, 7.0, 0), Vec3(1.0, 0.95, 0.9), 25.0f),
        PointLight(Vec3(-3, 5, -3), Vec3(0.4, 0.5, 1.0), 10.0f)
    };
}

int main() {
    Application app;
    if (!app.initialize({1024, 1024, "Raytracer Viewer", "settings.json"})) {
        LOG_ERROR << "Failed to initialize application";
        return 1;
    }

    buildScene(app.world(), app.renderer(), app.renderView());

    app.addSystem<DevControlSystem>();
    app.addSystem<CameraSystem>();
    app.addSystem<MotionSystem>();
    app.addSystem<RenderSystem>();
    app.addSystem<DebugOverlaySystem>();   // inert unless built with ImGui

    app.run();
    return 0;
}
