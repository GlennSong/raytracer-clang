#ifndef RAYTRACER_ENGINE_PHYSICS_SYSTEM_H
#define RAYTRACER_ENGINE_PHYSICS_SYSTEM_H

#include "../system.h"
#include "../physics/physics_world.h"

namespace engine {

class JobSystem;   // our shared thread pool (src/job_system.h)

// An ECS-level impact fact (ADR-0071), published on the EventBus after each
// fixed step for every NEW contact Jolt reported. `a`/`b` are the touching
// entities where the body is ECS-known (RigidBody / MeshCollider); an invalid
// Entity means the body belongs to something outside the ECS (a character
// proxy, an AI-car kinematic body). Subscribers react — impact sounds, decals,
// damage — without knowing physics exists.
struct Collision {
    Entity a, b;
    Vec3 position;   // world-space contact point
    Vec3 normal;
    Real speed = 0;  // closing speed along the normal (m/s) — severity
};

// Drives Jolt physics from the ECS (ADR-0012, ROADMAP 2.3 Step C). Creates a
// body for every entity with Transform + RigidBody + Collider, steps the world
// each fixed update (deterministic — fits ADR-0002), and writes simulated
// transforms back to the Transform component. Owns the Transform of physics
// entities; MotionSystem yields to it.
//
// The System hooks are thin wrappers over createBodies()/step(), which take a
// World directly so the integration is unit-testable without a FrameContext.
class PhysicsSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    bool initialize(engine::JobSystem* jobs = nullptr);
    void shutdown();
    void createBodies(World& world);
    void step(World& world, Real dt);
    // Drain the step's ContactEvents, map body ids to entities, and publish a
    // Collision per contact (synchronously, in recorded order). Called by
    // fixedUpdate after step(); takes World + EventBus directly so the bridge
    // is unit-testable headless.
    void publishContacts(World& world, EventBus& events);

    PhysicsWorld& physicsWorld() { return physics; }

private:
    PhysicsWorld physics;
};


}  // namespace engine

#endif
