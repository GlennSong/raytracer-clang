#ifndef RAYTRACER_ENGINE_PHYSICS_SYSTEM_H
#define RAYTRACER_ENGINE_PHYSICS_SYSTEM_H

#include "../system.h"
#include "../physics/physics_world.h"
#include <unordered_map>
#include <unordered_set>

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
    // still touching (a vehicle's scrape, ADR-0353), and how fast the surfaces slide past each other
    bool persisted = false;
    Real slide = 0;
    // the two physics bodies (a body outside the ECS -- a streamed tree or hydrant -- is known by its id alone), and a
    // breakable one knocked loose (ADR-0354)
    PhysicsBodyId bodyA = INVALID_PHYSICS_BODY, bodyB = INVALID_PHYSICS_BODY;
    bool broke = false;
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
    // the entity a body belongs to (invalid: none in the ECS), from a kept map: an entry is checked against the
    // entity's component before it is trusted, and a body not found rebuilds the map once (then is remembered as
    // nobody's until the next rebuild, or a few seconds)
    Entity entityOf(World& world, PhysicsBodyId id);
    void rebuildBodyMap(World& world);

    PhysicsWorld physics;
    std::unordered_map<PhysicsBodyId, Entity> byBody_;
    std::unordered_set<PhysicsBodyId> nobodys_;
    int nobodysAge_ = 0;   // fixed steps since the nobody set was cleared
};


}  // namespace engine

#endif
