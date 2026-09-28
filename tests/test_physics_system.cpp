#include "test_framework.h"

#include "../src/engine/systems/motion_system.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/world.h"
#include "../src/engine/components.h"

using namespace engine;  // namespace migration (ADR-0015)

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

TEST_CASE(physics_system_scene_gravity_zero_stops_the_fall) {
    // A space level (planet_demo) sets "gravity": [0,0,0]; the loader adds a
    // SceneGravity singleton, PhysicsSystem applies it, and a body with no floor
    // under it floats instead of falling forever.
    World world;
    Entity g = world.create();
    world.add<SceneGravity>(g, SceneGravity{Vec3(0, 0, 0)});
    Entity sphere = addDynamicSphere(world, Vec3(0, 5, 0));   // nothing beneath it

    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);                              // applies SceneGravity
    Real startY = world.get<Transform>(sphere)->position.y;
    for (int i = 0; i < 240; i++) physics.step(world, 1.0 / 60.0);
    Real endY = world.get<Transform>(sphere)->position.y;

    CHECK_APPROX(endY, startY, 0.05);   // did NOT fall (contrast: default gravity above)
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

TEST_CASE(physics_system_publishes_collision_events_with_entities) {
    World world;
    Entity floor = addFloor(world);
    Entity ball = addDynamicSphere(world, Vec3(0, 4, 0));

    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);

    EventBus events;
    std::vector<Collision> hits;
    events.subscribe<Collision>([&](const Collision& c) { hits.push_back(c); });

    for (int i = 0; i < 180 && hits.empty(); i++) {
        physics.step(world, 1.0 / 60.0);
        physics.publishContacts(world, events);
    }
    CHECK(!hits.empty());
    if (!hits.empty()) {
        // Both bodies are ECS-known, so the event names the actual entities.
        bool mapped = (hits[0].a == floor && hits[0].b == ball) ||
                      (hits[0].a == ball && hits[0].b == floor);
        CHECK(mapped);
        CHECK(hits[0].speed > 4.0);
    }
    physics.shutdown();
}

// FOOTSTEPS (#62): the character knows which body it stands on, and that body carries its collider's
// surface tag -- an asphalt road strip beside a terrain patch, walked from one onto the other.
TEST_CASE(the_character_stands_on_a_tagged_surface) {
    World world;
    auto quad = [&](double x0, double x1, ColliderSurface s) {
        Entity e = world.create();
        world.add<Transform>(e, Transform{});
        MeshCollider mc;
        mc.vertices = {Vec3(x0, 0, -10), Vec3(x1, 0, -10), Vec3(x1, 0, 10), Vec3(x0, 0, 10)};
        mc.indices = {0, 2, 1, 0, 3, 2};
        mc.surface = s;
        world.add<MeshCollider>(e, mc);
    };
    quad(-10, 0, ColliderSurface::Asphalt);
    quad(0, 10, ColliderSurface::Terrain);
    Entity player = world.create();
    Transform pt; pt.position = Vec3(-5, 1.0, 0);
    world.add<Transform>(player, pt);
    world.add<CharacterController>(player, CharacterController{});
    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);
    PhysicsWorld& pw = physics.physicsWorld();
    const CharacterId id = world.get<CharacterController>(player)->characterId;
    CHECK(id != INVALID_CHARACTER);
    auto settle = [&](const Vec3& vel, int ticks) {
        for (int i = 0; i < ticks; ++i) { pw.moveCharacter(id, vel, 1.0 / 60.0); physics.step(world, 1.0 / 60.0); }
    };
    settle(Vec3(0, 0, 0), 60);
    CHECK(pw.characterGroundState(id) == GroundState::OnGround);
    CHECK(pw.bodySurface(pw.characterGroundBody(id)) == static_cast<uint8_t>(ColliderSurface::Asphalt));
    settle(Vec3(3, 0, 0), 150);   // walk 7.5 m east, off the road onto the ground
    CHECK(pw.characterPosition(id).x > 1.0);
    CHECK(pw.bodySurface(pw.characterGroundBody(id)) == static_cast<uint8_t>(ColliderSurface::Terrain));
    // in the air: no ground body, untagged
    pw.setCharacterPosition(id, Vec3(5, 20, 0));
    settle(Vec3(0, 0, 0), 2);
    CHECK(pw.characterGroundBody(id) == INVALID_PHYSICS_BODY);
    CHECK(pw.bodySurface(pw.characterGroundBody(id)) == 0);
    physics.shutdown();
}
