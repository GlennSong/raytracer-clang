#ifndef RAYTRACER_ENGINE_PHYSICS_SYSTEM_H
#define RAYTRACER_ENGINE_PHYSICS_SYSTEM_H

#include "../system.h"
#include "../physics/physics_world.h"

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

    bool initialize();
    void shutdown();
    void createBodies(World& world);
    void step(World& world, Real dt);

    PhysicsWorld& physicsWorld() { return physics; }

private:
    PhysicsWorld physics;
};

#endif
