#include "rt_math.h"
#include "renderer/renderer.h"
#include "engine/world.h"
#include "engine/components.h"
#include "engine/application.h"
#include "engine/systems/dev_control_system.h"
#include "engine/systems/camera_system.h"
#include "engine/systems/motion_system.h"
#include "engine/systems/render_system.h"
#include "engine/systems/debug_overlay_system.h"
#ifdef RT_ENABLE_PHYSICS
#include "engine/systems/physics_system.h"
#include "engine/systems/player_system.h"
#endif
#include "log.h"

using namespace engine;

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

static void buildArena(World& world, Renderer& renderer, RenderView& view) {
    MeshHandle sphereHandle = renderer.uploadMesh(createSphereMesh(1.0f, 32, 64));

    constexpr Real ARENA = 25.0;
    constexpr Real WALL_HEIGHT = 4.0;
    constexpr Real WALL_THICK = 0.5;

    RenderMaterial floorMat;
    floorMat.albedo = Vec3(0.4, 0.4, 0.4);
    floorMat.roughness = 0.8f;

    RenderMaterial wallMat;
    wallMat.albedo = Vec3(0.6, 0.6, 0.65);
    wallMat.roughness = 0.7f;

    RenderMaterial rampMat;
    rampMat.albedo = Vec3(0.5, 0.35, 0.2);
    rampMat.roughness = 0.6f;

    RenderMaterial obstacleMat;
    obstacleMat.albedo = Vec3(0.3, 0.3, 0.7);
    obstacleMat.roughness = 0.5f;

    // Helper: static physics box with visual
    auto addStaticBox = [&](const Vec3& halfExt, const Vec3& pos,
                            const RenderMaterial& mat,
                            const Quat& orient = Quat::identity()) {
        MeshHandle mesh = renderer.uploadMesh(createBoxMesh(halfExt * 2.0));
        Entity e = world.create();
        Transform t;
        t.position = pos;
        t.orientation = orient;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable r;
        r.mesh = mesh;
        r.material = mat;
        world.add<Renderable>(e, r);
        Collider c;
        c.shape = ColliderShape::Box;
        c.halfExtent = halfExt;
        c.friction = 0.5;
        world.add<Collider>(e, c);
        world.add<RigidBody>(e, RigidBody{BodyMotion::Static, INVALID_PHYSICS_BODY});
        return e;
    };

    // Helper: dynamic physics sphere with visual
    auto addDynamicSphere = [&](Real radius, const Vec3& pos,
                                const RenderMaterial& mat) {
        Entity e = world.create();
        Transform t;
        t.position = pos;
        t.scale = Vec3(radius, radius, radius);
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable r;
        r.mesh = sphereHandle;
        r.material = mat;
        world.add<Renderable>(e, r);
        Collider c;
        c.shape = ColliderShape::Sphere;
        c.radius = radius;
        c.restitution = 0.5;
        c.friction = 0.3;
        world.add<Collider>(e, c);
        world.add<RigidBody>(e, RigidBody{BodyMotion::Dynamic, INVALID_PHYSICS_BODY});
        return e;
    };

    // Floor (top surface at y=0)
    addStaticBox(Vec3(ARENA, 1.0, ARENA), Vec3(0, -1.0, 0), floorMat);

    // 4 walls
    addStaticBox(Vec3(ARENA, WALL_HEIGHT * 0.5, WALL_THICK),
                 Vec3(0, WALL_HEIGHT * 0.5, -ARENA), wallMat);  // north
    addStaticBox(Vec3(ARENA, WALL_HEIGHT * 0.5, WALL_THICK),
                 Vec3(0, WALL_HEIGHT * 0.5, ARENA), wallMat);   // south
    addStaticBox(Vec3(WALL_THICK, WALL_HEIGHT * 0.5, ARENA),
                 Vec3(-ARENA, WALL_HEIGHT * 0.5, 0), wallMat);  // west
    addStaticBox(Vec3(WALL_THICK, WALL_HEIGHT * 0.5, ARENA),
                 Vec3(ARENA, WALL_HEIGHT * 0.5, 0), wallMat);   // east

    // Ramps (rotated boxes)
    Real rampAngle = degreesToRadians(15.0);
    addStaticBox(Vec3(3.0, 0.25, 2.0),
                 Vec3(-10, 0.8, -10),
                 rampMat,
                 Quat::fromAxisAngle(Vec3(1, 0, 0), rampAngle));

    addStaticBox(Vec3(3.0, 0.25, 2.0),
                 Vec3(10, 0.8, 10),
                 rampMat,
                 Quat::fromAxisAngle(Vec3(1, 0, 0), -rampAngle));

    // Static obstacles (pillars)
    addStaticBox(Vec3(1.0, 2.0, 1.0), Vec3(-6, 2.0, 5), obstacleMat);
    addStaticBox(Vec3(1.0, 2.0, 1.0), Vec3(6, 2.0, -5), obstacleMat);
    addStaticBox(Vec3(2.0, 1.0, 0.5), Vec3(0, 1.0, -8), obstacleMat);

    // Dynamic spheres (4 player-sized balls)
    RenderMaterial ballMats[4];
    ballMats[0].albedo = Vec3(0.9, 0.2, 0.2); ballMats[0].roughness = 0.3f;
    ballMats[1].albedo = Vec3(0.2, 0.7, 0.2); ballMats[1].roughness = 0.3f;
    ballMats[2].albedo = Vec3(0.2, 0.3, 0.9); ballMats[2].roughness = 0.3f;
    ballMats[3].albedo = Vec3(0.9, 0.8, 0.1); ballMats[3].roughness = 0.3f;

    addDynamicSphere(1.0, Vec3(-5, 1.5, -3), ballMats[0]);
    addDynamicSphere(1.0, Vec3(5, 1.5, 3), ballMats[1]);
    addDynamicSphere(1.0, Vec3(-3, 1.5, 8), ballMats[2]);
    addDynamicSphere(1.0, Vec3(8, 1.5, -8), ballMats[3]);

    // Player (capsule)
    {
        Entity player = world.create();
        Transform t;
        t.position = Vec3(0, 1.0, 5);
        world.add<Transform>(player, t);
        world.add<PrevTransform>(player, PrevTransform{t});
        Collider c;
        c.shape = ColliderShape::Capsule;
        c.halfHeight = 0.4;
        c.radius = 0.3;
        c.friction = 0.5;
        world.add<Collider>(player, c);
        RigidBody rb;
        rb.motion = BodyMotion::Dynamic;
        rb.lockRotation = true;
        world.add<RigidBody>(player, rb);
        world.add<ControlledBy>(player, ControlledBy{0});
    }

    // Lighting
    auto& lighting = view.lighting;
    lighting.sun = DirectionalLight(
        Vec3(0.4, 0.8, -0.3),      // sun direction (toward source)
        Vec3(1.0, 0.98, 0.92),     // warm white
        1.5f, true);               // intensity, casts shadow

    lighting.pointLights = {
        PointLight(Vec3(0, 12.0, 0), Vec3(1.0, 0.95, 0.9), 50.0f),
        PointLight(Vec3(-15, 8, -15), Vec3(0.4, 0.5, 1.0), 30.0f),
        PointLight(Vec3(15, 8, 15), Vec3(1.0, 0.7, 0.3), 30.0f),
    };

    lighting.spotLights = {
        SpotLight(Vec3(0, 10, 0), Vec3(0, -1, 0), Vec3(1.0, 0.9, 0.8),
                  40.0f, 0.3f, 0.5f),
    };

    lighting.exposure = 0.5f;
}

int main() {
    Application app;
    if (!app.initialize({1280, 720, "FPS Arena", "settings.json"})) {
        LOG_ERROR << "Failed to initialize application";
        return 1;
    }

    buildArena(app.world(), app.renderer(), app.renderView());

    app.settings().setString("cameraMode", "fly");

    app.addSystem<DevControlSystem>();
    auto& camSys = app.addSystem<CameraSystem>();
#ifdef RT_ENABLE_PHYSICS
    auto& physSys = app.addSystem<PhysicsSystem>();
    app.addSystem<PlayerSystem>(camSys.flyController(), physSys);
#endif
    app.addSystem<MotionSystem>();
    app.addSystem<RenderSystem>();
    app.addSystem<DebugOverlaySystem>();

    app.run();
    return 0;
}
