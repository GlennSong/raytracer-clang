// A stamped prefab is solid where its model's collision shapes are (#191, step 7): the proof the design note asks
// for, "a physics test that the boom's shape stops a box".
#include "test_framework.h"

#include "../src/engine/prefab_stamp.h"
#include "../src/engine/scripting/prefab_list.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/components.h"
#include "../src/engine/world.h"

using namespace engine;

namespace {

Entity addFloor(World& world) {
    Entity e = world.create();
    Transform t;
    t.position = Vec3(0, -1, 0);
    world.add<Transform>(e, t);
    Collider c;
    c.shape = ColliderShape::Box;
    c.halfExtent = Vec3(80, 1, 80);
    world.add<Collider>(e, c);
    world.add<RigidBody>(e, RigidBody{BodyMotion::Static, INVALID_PHYSICS_BODY});
    return e;
}

Entity addCrate(World& world, const Vec3& position) {
    Entity e = world.create();
    Transform t;
    t.position = position;
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    Collider c;
    c.shape = ColliderShape::Box;
    c.halfExtent = Vec3(0.25, 0.25, 0.25);
    c.friction = 0.8;
    world.add<Collider>(e, c);
    world.add<RigidBody>(e, RigidBody{BodyMotion::Dynamic, INVALID_PHYSICS_BODY});
    return e;
}

}  // namespace

TEST_CASE(a_stamped_cranes_boom_stops_a_falling_box) {
    World world;
    ModelStore store;
    addFloor(world);
    Prefab crane;
    CHECK(findPrefab("crane", crane));
    // The crane stands away from the origin and is turned a quarter, so the boom is where only the node tree and
    // the entity's place together put it: 3.1 m along the world's x from the mast, 5.4 m up.
    Transform at;
    at.position = Vec3(20, 0, -6);
    at.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), 3.14159265358979 / 2);
    std::string err;
    const Entity e = stampPrefab(world, nullptr, crane, at, std::string(), &err, &store);
    CHECK(e.valid());
    if (!e.valid()) { std::printf("    %s\n", err.c_str()); return; }
    const Vec3 boomMiddle = at.matrix().transformPoint(Vec3(0, 5.4, 3.1));
    CHECK(std::abs(boomMiddle.x - at.position.x) > 3.0);     // (the turn took the boom off the model's z)

    const Entity onBoom = addCrate(world, boomMiddle + Vec3(0, 3, 0));        // above the boom's middle
    const Entity beside = addCrate(world, at.position + Vec3(0, 8.4, 3.1));   // where the boom would be unturned
    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);
    for (Entity be : world.get<ModelInstance>(e)->bodies) CHECK(world.get<RigidBody>(be)->bodyId != INVALID_PHYSICS_BODY);
    for (int i = 0; i < 360; i++) physics.step(world, 1.0 / 60.0);

    // The boom is 0.3 m deep: its top is 5.55 m up, and the crate (0.5 m) rests on it.
    const Vec3 rested = world.get<Transform>(onBoom)->position;
    std::printf("    the crate over the boom rests at %.3f %.3f %.3f; the one beside it at y %.3f\n", rested.x, rested.y, rested.z,
                world.get<Transform>(beside)->position.y);
    CHECK_APPROX(rested.y, 5.55 + 0.25, 0.05);
    CHECK((Vec3(rested.x, 0, rested.z) - Vec3(boomMiddle.x, 0, boomMiddle.z)).length() < 0.3);
    // The other crate met nothing and is on the floor.
    CHECK_APPROX(world.get<Transform>(beside)->position.y, 0.25, 0.05);

    // Taking the crane away takes its bodies out of the physics: a crate dropped where the boom was meets nothing.
    // (The first crate is asleep where it rested, as a resting body is until something touches it.)
    const int before = physics.physicsWorld().bodyCount();
    unstampPrefab(world, e, [&](PhysicsBodyId id) { physics.physicsWorld().removeBody(id); });
    CHECK(physics.physicsWorld().bodyCount() == before - 4);
    const Entity through = addCrate(world, boomMiddle + Vec3(0.8, 3, 0));
    physics.createBodies(world);
    for (int i = 0; i < 360; i++) physics.step(world, 1.0 / 60.0);
    CHECK_APPROX(world.get<Transform>(through)->position.y, 0.25, 0.05);
    physics.shutdown();
}
