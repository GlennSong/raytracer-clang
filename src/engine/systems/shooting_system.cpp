#include "shooting_system.h"
#include "physics_system.h"
#include "../components.h"
#include "../../renderer/event.h"

namespace engine {

void ShootingSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("fire", MouseButton::Left);
    ctx.actions.bindButton("fire", GamepadButton::RightBumper);

    // Upload bullet mesh (small cube) once
    {
        float h = static_cast<float>(bulletSize * 0.5);
        RenderMesh box;
        struct Face { Vec3 normal; Vec3 verts[4]; };
        Face faces[] = {
            {Vec3(0,0,-1), {Vec3(-h,-h,-h), Vec3(h,-h,-h), Vec3(h,h,-h), Vec3(-h,h,-h)}},
            {Vec3(0,0,1),  {Vec3(h,-h,h), Vec3(-h,-h,h), Vec3(-h,h,h), Vec3(h,h,h)}},
            {Vec3(-1,0,0), {Vec3(-h,-h,h), Vec3(-h,-h,-h), Vec3(-h,h,-h), Vec3(-h,h,h)}},
            {Vec3(1,0,0),  {Vec3(h,-h,-h), Vec3(h,-h,h), Vec3(h,h,h), Vec3(h,h,-h)}},
            {Vec3(0,-1,0), {Vec3(-h,-h,h), Vec3(h,-h,h), Vec3(h,-h,-h), Vec3(-h,-h,-h)}},
            {Vec3(0,1,0),  {Vec3(-h,h,-h), Vec3(h,h,-h), Vec3(h,h,h), Vec3(-h,h,h)}}
        };
        for (auto& f : faces) {
            uint32_t base = static_cast<uint32_t>(box.vertices.size());
            for (int i = 0; i < 4; i++)
                box.vertices.push_back(Vertex(f.verts[i], f.normal));
            box.indices.insert(box.indices.end(),
                {base, base+1, base+2, base, base+2, base+3});
        }
        bulletMesh = rendererRef.uploadMesh(box);
    }

    // Create gun entity (a cylinder) — positioned relative to camera each frame
    {
        float gunRadius = 0.03f;
        float gunLength = 0.4f;
        RenderMesh cyl;
        int slices = 12;
        float halfH = gunLength * 0.5f;
        for (int i = 0; i <= slices; i++) {
            float phi = 2.0f * static_cast<float>(PI) * i / slices;
            float x = gunRadius * std::cos(phi);
            float z = gunRadius * std::sin(phi);
            Vec3 normal = normalize(Vec3(x, 0, z));
            cyl.vertices.push_back(Vertex(Vec3(x, -halfH, z), normal));
            cyl.vertices.push_back(Vertex(Vec3(x, halfH, z), normal));
        }
        for (int i = 0; i < slices; i++) {
            uint32_t a = i * 2, b = a + 1, c = a + 2, d = a + 3;
            cyl.indices.insert(cyl.indices.end(), {a, c, b, b, c, d});
        }
        gunMesh = rendererRef.uploadMesh(cyl);

        gunEntity = ctx.world.create();
        Transform t;
        ctx.world.add<Transform>(gunEntity, t);
        ctx.world.add<PrevTransform>(gunEntity, PrevTransform{t});
        Renderable r;
        r.mesh = gunMesh;
        r.material.albedo = Vec3(0.15, 0.15, 0.15);
        r.material.metallic = 0.9f;
        r.material.roughness = 0.2f;
        ctx.world.add<Renderable>(gunEntity, r);
    }
}

void ShootingSystem::spawnBullet(FrameContext& ctx) {
    Vec3 spawnPos = camera.eye + camera.forward() * 0.5 + camera.right() * 0.2
                    + Vec3(0, -0.15, 0);
    Vec3 velocity = camera.forward() * bulletSpeed;

    Entity e = ctx.world.create();
    Transform t;
    t.position = spawnPos;
    ctx.world.add<Transform>(e, t);
    ctx.world.add<PrevTransform>(e, PrevTransform{t});

    Collider c;
    c.shape = ColliderShape::Box;
    c.halfExtent = Vec3(bulletSize * 0.5, bulletSize * 0.5, bulletSize * 0.5);
    c.restitution = 0.7;
    c.friction = 0.3;
    ctx.world.add<Collider>(e, c);

    RigidBody rb;
    rb.motion = BodyMotion::Dynamic;
    // A bullet is small and fast, so discrete steps tunnel it straight through
    // thin static geometry (a building wall, a fence). Linear-cast CCD sweeps the
    // body between steps so it actually hits what it passes through.
    rb.continuousCollision = true;
    ctx.world.add<RigidBody>(e, rb);

    Renderable r;
    r.mesh = bulletMesh;
    r.material.albedo = Vec3(1.0, 0.85, 0.1);
    r.material.roughness = 0.4f;
    r.material.emission = Vec3(0.5, 0.3, 0.0);
    ctx.world.add<Renderable>(e, r);

    // Set velocity after physics creates the body next fixedUpdate
    // Store it in Velocity component; PhysicsSystem will apply it
    Velocity vel;
    vel.linear = velocity;
    ctx.world.add<Velocity>(e, vel);

    bulletCount++;
}

void ShootingSystem::update(FrameContext& ctx) {
    // Fire on press — but not when the click was aimed at a debug panel.
    if (ctx.actions.pressed("fire") && !ctx.input.uiWantsMouse &&
        bulletCount < maxBullets) {
        spawnBullet(ctx);
    }

    // Position gun relative to camera
    if (ctx.world.alive(gunEntity)) {
        auto* t = ctx.world.get<Transform>(gunEntity);
        auto* prev = ctx.world.get<PrevTransform>(gunEntity);
        if (t) {
            if (prev) prev->value = *t;
            // Gun barrel points forward, offset to the right and slightly down
            Vec3 gunPos = camera.eye
                + camera.forward() * 0.35
                + camera.right() * 0.18
                + Vec3(0, -0.18, 0);
            t->position = gunPos;
            // Orient the cylinder along the look direction
            // The cylinder is Y-up by default, we want it pointing forward
            // Build rotation: cylinder Y-axis → forward direction
            t->orientation = Quat::fromAxisAngle(camera.right(), -PI * 0.5)
                           * Quat::fromAxisAngle(Vec3(0,1,0), camera.yaw * PI / 180.0);
        }
    }
}

}  // namespace engine
