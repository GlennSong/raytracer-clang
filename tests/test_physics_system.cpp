#include "test_framework.h"

#include "../src/engine/systems/motion_system.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/world.h"
#include "../src/engine/components.h"

using namespace engine;  // namespace migration (ADR-0014)

namespace {

Entity addFloor(World& world) {
    Entity e = world.create();
    Transform t;
    t.position = Vec3(0, -1, 0);
    world.add<Transform>(e, t);
    Collider c;
    c.shape = ColliderShape::Box;
    c.halfExtent = Vec3(50, 1, 50);
    world.add<Collider>(e, c);
    world.add<RigidBody>(e, RigidBody{BodyMotion::Static, INVALID_PHYSICS_BODY});
    return e;
}

Entity addDynamicSphere(World& world, const Vec3& position) {
    Entity e = world.create();
    Transform t;
    t.position = position;
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    Collider c;
    c.shape = ColliderShape::Sphere;
    c.radius = 0.5;
    world.add<Collider>(e, c);
    world.add<RigidBody>(e, RigidBody{BodyMotion::Dynamic, INVALID_PHYSICS_BODY});
    return e;
}

}  // namespace

TEST_CASE(physics_system_creates_bodies) {
    World world;
    Entity sphere = addDynamicSphere(world, Vec3(0, 5, 0));

    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);

    RigidBody* rb = world.get<RigidBody>(sphere);
    CHECK(rb != nullptr);
    if (rb) CHECK(rb->bodyId != INVALID_PHYSICS_BODY);
    physics.shutdown();
}

TEST_CASE(physics_system_drives_transform) {
    World world;
    addFloor(world);
    Entity sphere = addDynamicSphere(world, Vec3(0, 5, 0));

    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);

    Real startY = world.get<Transform>(sphere)->position.y;
    for (int i = 0; i < 240; i++) physics.step(world, 1.0 / 60.0);
    Real restY = world.get<Transform>(sphere)->position.y;

    CHECK(restY < startY);                 // it fell
    CHECK_APPROX(restY, 0.5, 0.05);        // and settled at radius height
    physics.shutdown();
}

TEST_CASE(physics_system_updates_prev_transform) {
    World world;
    addFloor(world);
    Entity sphere = addDynamicSphere(world, Vec3(0, 5, 0));

    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);
    for (int i = 0; i < 6; i++) physics.step(world, 1.0 / 60.0);

    // While falling, prev should trail current (interpolation source is live).
    Transform* t = world.get<Transform>(sphere);
    PrevTransform* prev = world.get<PrevTransform>(sphere);
    CHECK(t != nullptr && prev != nullptr);
    if (t && prev) CHECK(prev->value.position.y > t->position.y);
    physics.shutdown();
}

TEST_CASE(physics_system_leaves_static_body) {
    World world;
    Entity floor = addFloor(world);

    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);

    Vec3 before = world.get<Transform>(floor)->position;
    for (int i = 0; i < 60; i++) physics.step(world, 1.0 / 60.0);
    Vec3 after = world.get<Transform>(floor)->position;
    CHECK(approxEqual(before, after));
    physics.shutdown();
}

TEST_CASE(motion_system_moves_velocity_entities) {
    World world;
    Entity e = world.create();
    Transform t;
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    Velocity v;
    v.linear = Vec3(1, 0, 0);
    world.add<Velocity>(e, v);

    MotionSystem motion;
    motion.integrate(world, 1.0);
    CHECK_APPROX(world.get<Transform>(e)->position.x, 1.0, 1e-9);
}

TEST_CASE(motion_system_yields_to_physics) {
    World world;
    Entity e = world.create();
    Transform t;
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    Velocity v;
    v.linear = Vec3(1, 0, 0);
    world.add<Velocity>(e, v);
    // Same entity also has a RigidBody -> physics owns it; motion must not move it.
    world.add<RigidBody>(e, RigidBody{BodyMotion::Dynamic, INVALID_PHYSICS_BODY});

    MotionSystem motion;
    motion.integrate(world, 1.0);
    CHECK_APPROX(world.get<Transform>(e)->position.x, 0.0, 1e-9);
}
