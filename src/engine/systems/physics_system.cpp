#include "physics_system.h"

#include "../components.h"

namespace engine {

bool PhysicsSystem::initialize(engine::JobSystem* jobs) { return physics.initialize(jobs); }

void PhysicsSystem::shutdown() { physics.shutdown(); }

void PhysicsSystem::createBodies(World& world) {
    world.each<Transform, RigidBody, Collider>(
        [this](Entity, Transform& t, RigidBody& rb, Collider& collider) {
            if (rb.bodyId != INVALID_PHYSICS_BODY) return;  // already created
            if (collider.shape == ColliderShape::Sphere) {
                rb.bodyId = physics.addSphere(collider.radius, t.position,
                                              t.orientation, rb.motion);
            } else {
                rb.bodyId = physics.addBox(collider.halfExtent, t.position,
                                           t.orientation, rb.motion);
            }
        });
    physics.optimizeBroadPhase();
}

void PhysicsSystem::step(World& world, Real dt) {
    // Snapshot previous transforms (moving bodies only) so the render system can
    // interpolate between fixed steps — the role MotionSystem played for its
    // entities (ADR-0006).
    world.each<Transform, RigidBody, PrevTransform>(
        [](Entity, Transform& t, RigidBody& rb, PrevTransform& prev) {
            if (rb.motion != BodyMotion::Static) prev.value = t;
        });

    physics.update(dt);

    // Write simulated transforms back to the ECS.
    world.each<Transform, RigidBody>(
        [this](Entity, Transform& t, RigidBody& rb) {
            if (rb.motion == BodyMotion::Static) return;
            if (rb.bodyId == INVALID_PHYSICS_BODY) return;
            t.position = physics.bodyPosition(rb.bodyId);
            t.orientation = physics.bodyOrientation(rb.bodyId);
        });
}

void PhysicsSystem::onStart(FrameContext& ctx) {
    initialize(&ctx.jobs);   // run Jolt's step on the engine's shared pool
    createBodies(ctx.world);
}

void PhysicsSystem::fixedUpdate(FrameContext& ctx) {
    step(ctx.world, ctx.clock.fixedStep());
}

void PhysicsSystem::onStop(FrameContext&) { shutdown(); }

}  // namespace engine

