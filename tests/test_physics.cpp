#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include "../src/job_system.h"
#include <vector>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// A ground box centered at y=-1 with half-height 1, so its top is y=0.
PhysicsBodyId addFloor(PhysicsWorld& world) {
    return world.addBox(Vec3(50, 1, 50), Vec3(0, -1, 0), Quat::identity(),
                        BodyMotion::Static);
}

// A flat triangle-mesh floor at y=0 (the terrain-collider path).
PhysicsBodyId addMeshFloor(PhysicsWorld& world) {
    std::vector<Vec3> verts = {{-50, 0, -50}, {50, 0, -50}, {50, 0, 50}, {-50, 0, 50}};
    std::vector<uint32_t> idx = {0, 1, 2, 0, 2, 3};
    return world.addMesh(verts, idx, Vec3(0, 0, 0));
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

TEST_CASE(physics_sphere_rests_on_mesh_floor) {
    PhysicsWorld world;
    world.initialize();
    PhysicsBodyId floor = addMeshFloor(world);
    CHECK(floor != INVALID_PHYSICS_BODY);   // the mesh shape was created
    PhysicsBodyId sphere =
        world.addSphere(0.5, Vec3(0, 5, 0), Quat::identity(), BodyMotion::Dynamic);
    world.optimizeBroadPhase();

    step(world, 240);   // ~4s to settle
    Real y = world.bodyPosition(sphere).y;
    CHECK(y > 0.3);     // did not fall through the triangle mesh
    CHECK(y < 0.7);     // rests ~radius (0.5) above the surface
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

// --- Character controller (CharacterVirtual) ---------------------------------
namespace {
// Walk a character with a fixed desired velocity for N frames (gravity is
// applied inside moveCharacter — this mirrors PlayerSystem::fixedUpdate).
void walk(PhysicsWorld& world, CharacterId c, const Vec3& vel, int frames) {
    for (int i = 0; i < frames; i++) world.moveCharacter(c, vel, 1.0 / 60.0);
}
}  // namespace

TEST_CASE(character_settles_on_floor) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);   // top at y=0
    // Capsule: halfHeight 0.4, radius 0.3 -> bottom is 0.7 below the centre, so a
    // grounded centre rests at y=0.7. Spawn it a little above and let it drop.
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(0, 2.0, 0));
    CHECK(c != INVALID_CHARACTER);
    world.optimizeBroadPhase();

    walk(world, c, Vec3(), 180);   // ~3s of no input
    Real y = world.characterPosition(c).y;
    CHECK_APPROX(y, 0.7, 0.1);     // resting on the floor, not through it
    CHECK(world.characterGroundState(c) == GroundState::OnGround);
    world.shutdown();
}

TEST_CASE(character_steps_up_a_curb) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);   // top at y=0, walkable everywhere
    // A 0.3 m kerb (top at y=0.3) starting at x=5 — lower than the 0.4 m default
    // step height, so the controller should climb onto it.
    world.addBox(Vec3(5, 0.15, 5), Vec3(10, 0.15, 0), Quat::identity(),
                 BodyMotion::Static);
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(0, 0.9, 0));
    world.optimizeBroadPhase();

    walk(world, c, Vec3(), 60);          // settle on the floor first
    CHECK_APPROX(world.characterPosition(c).y, 0.7, 0.1);

    walk(world, c, Vec3(2.5, 0, 0), 240);   // stroll toward and onto the kerb
    Vec3 p = world.characterPosition(c);
    CHECK(p.x > 6.0);    // walked past the kerb edge at x=5
    CHECK(p.y > 0.95);   // climbed the 0.3 m step (centre ~1.0 on top)
    world.shutdown();
}

TEST_CASE(character_blocked_by_tall_wall) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    // An 0.8 m step — well above the 0.4 m step height, so it acts as a wall.
    world.addBox(Vec3(5, 0.4, 5), Vec3(10, 0.4, 0), Quat::identity(),
                 BodyMotion::Static);
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(0, 0.9, 0));
    world.optimizeBroadPhase();

    walk(world, c, Vec3(), 60);
    walk(world, c, Vec3(2.5, 0, 0), 240);
    Vec3 p = world.characterPosition(c);
    CHECK(p.x < 5.0);    // stopped at the wall face (kerb edge at x=5)
    CHECK(p.y < 0.9);    // did not climb onto the 0.8 m ledge
    world.shutdown();
}

TEST_CASE(character_invalid_handle_is_safe) {
    PhysicsWorld world;
    world.initialize();
    CHECK(approxEqual(world.characterPosition(INVALID_CHARACTER), Vec3()));
    CHECK(world.characterGroundState(INVALID_CHARACTER) == GroundState::InAir);
    world.moveCharacter(INVALID_CHARACTER, Vec3(1, 0, 0), 1.0 / 60.0);
    world.removeCharacter(INVALID_CHARACTER);
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

TEST_CASE(physics_ccd_box_does_not_tunnel_thin_wall) {
    // A bullet is a small, fast dynamic box. With discrete stepping it leaps
    // past a thin wall between frames (tunnelling — "I can shoot through
    // buildings"); linear-cast CCD sweeps the body so it actually stops. Same
    // shot, CCD off vs on.
    auto shoot = [](bool continuous) {
        PhysicsWorld world;
        world.initialize();
        // A thin static wall (10 cm) standing at x = 0.
        world.addBox(Vec3(0.05, 3, 3), Vec3(0, 0, 0), Quat::identity(),
                     BodyMotion::Static, 0.0, 0.5);
        // A small fast box fired from x = -3 straight at it (~5 m per 1/60 step,
        // far more than the wall is thick).
        PhysicsBodyId b =
            world.addBox(Vec3(0.05, 0.05, 0.05), Vec3(-3, 0, 0), Quat::identity(),
                         BodyMotion::Dynamic, 0.0, 0.3, false, continuous);
        world.optimizeBroadPhase();
        world.setLinearVelocity(b, Vec3(300, 0, 0));
        step(world, 6);
        Real x = world.bodyPosition(b).x;
        world.shutdown();
        return x;
    };
    CHECK(shoot(false) > 0.5);   // discrete: tunnelled clean through the wall
    CHECK(shoot(true) < 0.0);    // CCD: stopped on the near side
}

// --- Fall-respawn safety net (FallRespawnTracker + character teleport) --------
// Integration of the tracker with the real character controller: walking off a
// ledge triggers exactly one respawn and the character re-settles; a level with
// no floor triggers the give-up path instead of teleport-cycling forever. Also
// pins the teleport contract: setCharacterPosition drops carried velocity.
#include "../src/engine/systems/player_system.h"

TEST_CASE(character_fall_respawn_recovers_off_ledge) {
    PhysicsWorld world;
    world.initialize();
    // A narrow floor: top at y=0, only 1m wide in x.
    world.addBox(Vec3(1, 1, 50), Vec3(0, -1, 0), Quat::identity(),
                 BodyMotion::Static);
    const Vec3 spawn(0, 2.0, 0);
    CharacterId c = world.addCharacter(0.4, 0.3, spawn);
    world.optimizeBroadPhase();

    FallRespawnTracker fall;
    fall.onSpawnCaptured(spawn.y);
    int respawns = 0;
    auto tick = [&](const Vec3& vel) {
        world.moveCharacter(c, vel, 1.0 / 60.0);
        Real y = world.characterPosition(c).y;
        GroundState gs = world.characterGroundState(c);
        if (gs == GroundState::OnGround || gs == GroundState::OnSteepGround)
            fall.onGrounded(y);
        if (fall.shouldRespawn(y)) {
            world.setCharacterPosition(c, spawn);
            fall.onRespawn(false);
            ++respawns;
        }
    };

    for (int i = 0; i < 60; ++i) tick(Vec3());            // settle on the ledge
    CHECK(world.characterGroundState(c) == GroundState::OnGround);
    for (int i = 0; i < 120; ++i) tick(Vec3(4, 0, 0));    // walk off the edge
    // Falls 40m below the footing (~2.9s), respawns once, settles again.
    for (int i = 0; i < 400; ++i) tick(Vec3());
    CHECK(respawns == 1);
    CHECK_APPROX(world.characterPosition(c).y, 0.7, 0.1);
    CHECK(world.characterGroundState(c) == GroundState::OnGround);
    world.shutdown();
}

TEST_CASE(character_fall_respawn_gives_up_without_floor) {
    PhysicsWorld world;
    world.initialize();          // no bodies at all: nothing to land on
    const Vec3 spawn(0, 200.0, 0);
    CharacterId c = world.addCharacter(0.4, 0.3, spawn);
    world.optimizeBroadPhase();

    FallRespawnTracker fall;
    fall.onSpawnCaptured(spawn.y);
    int respawns = 0;
    // Simulate long enough for several 300m fall cycles (~8s each without the
    // give-up; ~7.8s per cycle at 1/60 steps).
    for (int i = 0; i < 60 * 40 && respawns <= FallRespawnTracker::kMaxFailedRespawns + 1; ++i) {
        world.moveCharacter(c, Vec3(), 1.0 / 60.0);
        Real y = world.characterPosition(c).y;
        if (fall.shouldRespawn(y)) {
            world.setCharacterPosition(c, spawn);
            fall.onRespawn(false);
            ++respawns;
        }
    }
    CHECK(respawns == FallRespawnTracker::kMaxFailedRespawns);   // then disarmed
    CHECK(world.characterPosition(c).y < spawn.y - 300.0);       // falling free, no more snaps
    world.shutdown();
}

TEST_CASE(character_teleport_zeroes_velocity) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);   // top at y=0
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(0, 60.0, 0));
    world.optimizeBroadPhase();

    walk(world, c, Vec3(), 90);   // 1.5s of free fall: ~14.7 m/s downward
    CHECK(world.characterPosition(c).y < 55.0);
    world.setCharacterPosition(c, Vec3(0, 2.0, 0));
    // One step after the teleport: with velocity zeroed the drop is ~g*dt*dt
    // (millimetres); with the old carried velocity it would be ~0.25m.
    world.moveCharacter(c, Vec3(), 1.0 / 60.0);
    CHECK(world.characterPosition(c).y > 1.9);
    world.shutdown();
}
