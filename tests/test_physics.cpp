#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include "../src/job_system.h"

using namespace engine;  // namespace migration (ADR-0014)

namespace {

// A ground box centered at y=-1 with half-height 1, so its top is y=0.
PhysicsBodyId addFloor(PhysicsWorld& world) {
    return world.addBox(Vec3(50, 1, 50), Vec3(0, -1, 0), Quat::identity(),
                        BodyMotion::Static);
}

void step(PhysicsWorld& world, int frames) {
    for (int i = 0; i < frames; i++) world.update(1.0 / 60.0);
}

}  // namespace

TEST_CASE(physics_sphere_falls_under_gravity) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    PhysicsBodyId sphere =
        world.addSphere(0.5, Vec3(0, 5, 0), Quat::identity(), BodyMotion::Dynamic);
    world.optimizeBroadPhase();

    Real startY = world.bodyPosition(sphere).y;
    step(world, 12);
    Real laterY = world.bodyPosition(sphere).y;
    CHECK(laterY < startY);  // gravity pulled it down
    world.shutdown();
}

TEST_CASE(physics_sphere_rests_on_floor) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    PhysicsBodyId sphere =
        world.addSphere(0.5, Vec3(0, 5, 0), Quat::identity(), BodyMotion::Dynamic);
    world.optimizeBroadPhase();

    step(world, 240);  // ~4 seconds: long enough to settle
    Real restY = world.bodyPosition(sphere).y;
    // Floor top is y=0, sphere radius 0.5, so it settles near y=0.5.
    CHECK_APPROX(restY, 0.5, 0.05);
    // And it did not tunnel through the floor.
    CHECK(restY > 0.4);
    world.shutdown();
}

TEST_CASE(physics_static_body_does_not_move) {
    PhysicsWorld world;
    world.initialize();
    PhysicsBodyId floor = addFloor(world);
    world.optimizeBroadPhase();

    Vec3 before = world.bodyPosition(floor);
    step(world, 60);
    Vec3 after = world.bodyPosition(floor);
    CHECK(approxEqual(before, after));
    world.shutdown();
}

TEST_CASE(physics_is_deterministic) {
    auto run = []() {
        PhysicsWorld world;
        world.initialize();
        addFloor(world);
        PhysicsBodyId sphere = world.addSphere(0.5, Vec3(0.1, 4, -0.2),
                                               Quat::identity(), BodyMotion::Dynamic);
        world.optimizeBroadPhase();
        step(world, 90);
        Vec3 p = world.bodyPosition(sphere);
        world.shutdown();
        return p;
    };

    Vec3 a = run();
    Vec3 b = run();
    CHECK(approxEqual(a, b, 1e-6));
}

TEST_CASE(physics_initial_velocity_carries) {
    PhysicsWorld world;
    world.initialize();
    PhysicsBodyId ball =
        world.addSphere(0.5, Vec3(0, 10, 0), Quat::identity(), BodyMotion::Dynamic);
    world.setLinearVelocity(ball, Vec3(5, 0, 0));
    world.optimizeBroadPhase();

    step(world, 30);
    // Half a second of +x velocity (no x forces) carries it clearly positive.
    CHECK(world.bodyPosition(ball).x > 1.0);
    world.shutdown();
}

TEST_CASE(physics_invalid_body_is_safe) {
    PhysicsWorld world;
    world.initialize();
    // Querying a bad handle must not crash and returns a default.
    CHECK(approxEqual(world.bodyPosition(INVALID_PHYSICS_BODY), Vec3()));
    world.removeBody(INVALID_PHYSICS_BODY);
    world.shutdown();
}

// Run the same scene on our JobSystem (the JoltJobAdapter path) and confirm the
// result matches the single-threaded path bit-for-bit, and is itself
// repeatable. This exercises the adapter end-to-end and pins down that routing
// Jolt's jobs through our pool changes nothing observable (ADR-0002 / ADR-0012).
namespace {
Vec3 dropAndSettle(PhysicsWorld& world) {
    addFloor(world);
    PhysicsBodyId sphere = world.addSphere(0.5, Vec3(0.1, 4, -0.2),
                                           Quat::identity(), BodyMotion::Dynamic);
    world.optimizeBroadPhase();
    step(world, 120);
    return world.bodyPosition(sphere);
}
}  // namespace

TEST_CASE(physics_threaded_matches_single_threaded) {
    Vec3 single;
    {
        PhysicsWorld world;
        world.initialize();                 // single-threaded
        single = dropAndSettle(world);
        world.shutdown();
    }

    JobSystem pool(3);                       // 3 workers -> the adapter path
    Vec3 threaded;
    {
        PhysicsWorld world;
        world.initialize(&pool);
        threaded = dropAndSettle(world);
        world.shutdown();
    }

    // Same machine, same Jolt: multi-threaded simulation stays deterministic.
    CHECK(approxEqual(single, threaded, 1e-6));
}

TEST_CASE(physics_threaded_is_repeatable) {
    JobSystem pool(4);
    auto run = [&]() {
        PhysicsWorld world;
        world.initialize(&pool);
        Vec3 p = dropAndSettle(world);
        world.shutdown();
        return p;
    };
    CHECK(approxEqual(run(), run(), 1e-6));
}

// Synchronous pool (0 workers): the adapter must still run the sim correctly,
// executing Jolt's jobs inline like the single-threaded system does.
TEST_CASE(physics_synchronous_pool_runs_sim) {
    JobSystem pool(0);
    CHECK(pool.workerCount() == 0);

    PhysicsWorld world;
    world.initialize(&pool);
    addFloor(world);
    PhysicsBodyId sphere =
        world.addSphere(0.5, Vec3(0, 5, 0), Quat::identity(), BodyMotion::Dynamic);
    world.optimizeBroadPhase();
    step(world, 240);
    CHECK_APPROX(world.bodyPosition(sphere).y, 0.5, 0.05);
    world.shutdown();
}
