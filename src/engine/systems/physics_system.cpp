#include "physics_system.h"

#include "../components.h"
#include <vector>

namespace engine {

bool PhysicsSystem::initialize(engine::JobSystem* jobs) { return physics.initialize(jobs); }

void PhysicsSystem::shutdown() { physics.shutdown(); }

void PhysicsSystem::createBodies(World& world) {
    bool created = false;
    // Collect newly created entities to set initial velocity after iteration
    struct NewBody { PhysicsBodyId id; Vec3 velocity; };
    std::vector<NewBody> newBodies;

    world.each<Transform, RigidBody, Collider>(
        [this, &created, &world, &newBodies](Entity e, Transform& t, RigidBody& rb, Collider& c) {
            if (rb.bodyId != INVALID_PHYSICS_BODY) return;
            switch (c.shape) {
                case ColliderShape::Sphere:
                    rb.bodyId = physics.addSphere(c.radius, t.position,
                                                  t.orientation, rb.motion,
                                                  c.restitution, c.friction,
                                                  rb.lockRotation);
                    break;
                case ColliderShape::Capsule:
                    rb.bodyId = physics.addCapsule(c.halfHeight, c.radius,
                                                   t.position, t.orientation,
                                                   rb.motion, c.restitution,
                                                   c.friction, rb.lockRotation,
                                                   rb.continuousCollision);
                    break;
                default:
                    rb.bodyId = physics.addBox(c.halfExtent, t.position,
                                               t.orientation, rb.motion,
                                               c.restitution, c.friction,
                                               rb.lockRotation);
                    break;
            }
            // If entity has a Velocity component, queue initial velocity
            auto* vel = world.get<Velocity>(e);
            if (vel && rb.bodyId != INVALID_PHYSICS_BODY) {
                newBodies.push_back({rb.bodyId, vel->linear});
            }
            created = true;
        });

    // Static triangle-mesh colliders (terrain): one static body each, made once.
    world.each<Transform, MeshCollider>(
        [this, &created](Entity, Transform& t, MeshCollider& mc) {
            if (mc.bodyId != INVALID_PHYSICS_BODY) return;
            mc.bodyId = physics.addMesh(mc.vertices, mc.indices, t.position, mc.friction);
            if (mc.bodyId != INVALID_PHYSICS_BODY) created = true;
        });

    // Apply initial velocities outside iteration
    for (auto& nb : newBodies) {
        physics.setLinearVelocity(nb.id, nb.velocity);
    }

    if (created) physics.optimizeBroadPhase();
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
    createBodies(ctx.world);
    step(ctx.world, ctx.clock.fixedStep());
}

void PhysicsSystem::onStop(FrameContext&) { shutdown(); }

}  // namespace engine

