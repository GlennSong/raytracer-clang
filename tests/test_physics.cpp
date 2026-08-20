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

TEST_CASE(character_jump_leaves_the_ground_and_lands_again) {
    // The integration that code-reading cannot settle: moveCharacter OWNS the
    // vertical axis and zeroes it on the ground, so a jump only works because
    // it is staged and consumed inside that same step (and releases the
    // stick-to-floor that smooths kerbs). Prove the capsule actually rises.
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(0, 2.0, 0));
    world.optimizeBroadPhase();

    walk(world, c, Vec3(), 180);   // settle
    const Real rest = world.characterPosition(c).y;
    CHECK(world.characterGroundState(c) == GroundState::OnGround);

    CHECK(world.jumpCharacter(c, 4.3));
    Real peak = rest;
    for (int i = 0; i < 60; i++) {
        world.moveCharacter(c, Vec3(), 1.0 / 60.0);
        peak = std::max(peak, world.characterPosition(c).y);
    }
    CHECK(peak > rest + 0.5);   // a real leap, not a twitch

    walk(world, c, Vec3(), 240);   // and it comes back down
    CHECK_APPROX(world.characterPosition(c).y, rest, 0.12);
    CHECK(world.characterGroundState(c) == GroundState::OnGround);
    world.shutdown();
}

TEST_CASE(character_cannot_jump_in_mid_air) {
    // The grounded gate lives in the physics seam, so no caller can get
    // "am I allowed to jump" wrong — and nobody can double-jump.
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(0, 2.0, 0));
    world.optimizeBroadPhase();
    walk(world, c, Vec3(), 180);

    CHECK(world.jumpCharacter(c, 4.3));
    walk(world, c, Vec3(), 12);   // now airborne
    CHECK(world.characterGroundState(c) != GroundState::OnGround);
    CHECK(!world.jumpCharacter(c, 4.3));   // refused mid-flight
    world.shutdown();
}

TEST_CASE(character_crouch_shrinks_but_keeps_its_feet_planted) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(0, 2.0, 0));
    world.optimizeBroadPhase();
    walk(world, c, Vec3(), 180);

    const Real standCentre = world.characterPosition(c).y;
    const Real feetBefore = standCentre - (0.4 + 0.3);
    CHECK(world.setCharacterHeight(c, 0.14, 0.3));
    const Real crouchCentre = world.characterPosition(c).y;
    const Real feetAfter = crouchCentre - (0.14 + 0.3);
    // The centre drops; the SOLES stay put (a crouch settles, it does not
    // sink into the floor or hop up off it).
    CHECK(crouchCentre < standCentre - 0.2);
    CHECK_APPROX(feetAfter, feetBefore, 0.05);

    walk(world, c, Vec3(), 60);   // stays standing on the floor, crouched
    CHECK(world.characterGroundState(c) == GroundState::OnGround);
    CHECK(world.setCharacterHeight(c, 0.4, 0.3));   // clear overhead: stands
    CHECK_APPROX(world.characterPosition(c).y, standCentre, 0.12);
    world.shutdown();
}

TEST_CASE(character_cannot_stand_up_under_a_ledge) {
    // The payoff of doing crouch as a real capsule swap: the fit test refuses
    // to stand where there is no headroom, so a crouched player under a ledge
    // stays crouched instead of popping through it.
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(0, 2.0, 0));
    world.optimizeBroadPhase();
    walk(world, c, Vec3(), 180);
    CHECK(world.setCharacterHeight(c, 0.14, 0.3));   // crouch first
    walk(world, c, Vec3(), 30);

    // A slab just above the crouched head: the crouched capsule (0.88 m tall)
    // fits under it, the standing one (1.4 m) does not.
    world.addBox(Vec3(4, 0.1, 4), Vec3(0, 1.05, 0), Quat::identity(),
                 BodyMotion::Static);
    world.optimizeBroadPhase();

    const Real crouchY = world.characterPosition(c).y;
    CHECK(!world.setCharacterHeight(c, 0.4, 0.3));           // refused
    CHECK_APPROX(world.characterPosition(c).y, crouchY, 0.02);   // and unmoved
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

// Tearing a PhysicsWorld down immediately after stepping must not outrun the
// jobs it queued onto the shared pool.
//
// The bug this pins down: JoltJobAdapter had no destructor, so it could be
// destroyed while a worker sat between Job::Execute() — which signals the
// barrier the stepping thread was waiting on — and Job::Release(), which frees
// the job out of the adapter's own free list. The worker then wrote into
// destroyed storage. It crashed physics_tests twice under CPU contention
// (SIGSEGV, null+0x48) while passing every time the machine was idle.
//
// Many short-lived worlds sharing one pool is the shape that provokes it: each
// teardown races that window, and oversubscribing the pool (more workers than
// cores) keeps threads getting descheduled inside it. Timing-dependent by
// nature, so this is a probabilistic gate — but with the destructor removed it
// fails readily, and it costs a fraction of a second to run.
TEST_CASE(physics_world_teardown_waits_for_queued_jobs) {
    JobSystem pool(8);   // oversubscribed on purpose: maximises preemption

    for (int i = 0; i < 40; i++) {
        PhysicsWorld world;
        world.initialize(&pool);
        addFloor(world);
        world.addSphere(0.5, Vec3(0, 3, 0), Quat::identity(), BodyMotion::Dynamic);
        world.optimizeBroadPhase();
        // Just enough steps to have real jobs in flight, then drop the world
        // straight away — the teardown is the thing under test, not the sim.
        step(world, 2);
        world.shutdown();
    }

    // Reaching here without a crash IS the assertion; the CHECK keeps the case
    // from looking assertion-free to a reader skimming the file.
    CHECK(pool.workerCount() == 8);
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

TEST_CASE(physics_contact_events_fire_on_impact) {
    PhysicsWorld world;
    world.initialize();
    PhysicsBodyId floor = addFloor(world);
    PhysicsBodyId sphere =
        world.addSphere(0.5, Vec3(0, 4, 0), Quat::identity(), BodyMotion::Dynamic);
    world.optimizeBroadPhase();

    CHECK(world.drainContactEvents().empty());   // nothing has touched yet

    std::vector<ContactEvent> contacts;
    for (int i = 0; i < 180 && contacts.empty(); i++) {
        world.update(1.0 / 60.0);
        std::vector<ContactEvent> batch = world.drainContactEvents();
        contacts.insert(contacts.end(), batch.begin(), batch.end());
    }
    CHECK(!contacts.empty());
    if (contacts.empty()) return;

    const ContactEvent& hit = contacts.front();
    bool pairMatches = (hit.bodyA == floor && hit.bodyB == sphere) ||
                       (hit.bodyA == sphere && hit.bodyB == floor);
    CHECK(pairMatches);
    CHECK(hit.approachSpeed > 4.0);            // ~8 m/s after a 3.5 m fall
    CHECK(std::fabs(hit.position.y) < 0.25);   // at the floor top (y = 0)
}

TEST_CASE(physics_resting_contact_does_not_respam_events) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    world.addSphere(0.5, Vec3(0, 2, 0), Quat::identity(), BodyMotion::Dynamic);
    world.optimizeBroadPhase();

    step(world, 240);              // fall, land, settle, fall asleep
    world.drainContactEvents();    // discard the landing burst
    step(world, 60);
    // Resting on the floor is a persisted contact — no new ContactEvents.
    CHECK(world.drainContactEvents().empty());
}

// --- AgentDriver drives real Jolt (possession round, ADR-0079) ---------------
// The missing integration: computeDriverInput was harness-tested against a
// kinematic bicycle model only, and its steering sign shipped MIRRORED without
// anything noticing (nothing spawned an AgentDriver). This closes the loop the
// possession system depends on: route polyline -> LaneFollower -> pursuit ->
// computeDriverInput -> setVehicleInput -> Jolt, on a real vehicle.

#include "../src/engine/ai/lane_follow.h"

namespace {

PhysicsWorld::VehicleConfig sedanConfig() {
    PhysicsWorld::VehicleConfig cfg;
    cfg.chassisHalfExtent = Vec3(0.9, 0.65, 2.1);
    cfg.mass = 1500.0;
    cfg.maxSteerDegrees = 32.0;
    cfg.engineTorque = 650.0;
    cfg.brakeTorque = 1600.0;
    cfg.handBrakeTorque = 4200.0;
    const Real axleY = -0.35, halfTrack = 0.80, axleZ = 1.35, r = 0.31;
    for (int i = 0; i < 4; ++i) {
        PhysicsWorld::VehicleWheel w;
        const bool front = i < 2;
        w.position = Vec3(i % 2 ? -halfTrack : halfTrack, axleY,
                          front ? axleZ : -axleZ);
        w.radius = r;
        w.width = 0.22;
        w.suspensionMin = 0.05;
        w.suspensionMax = 0.25;
        w.steered = front;
        w.driven = true;
        w.handBrake = !front;
        cfg.wheels.push_back(w);
    }
    return cfg;
}

}  // namespace

TEST_CASE(agent_driver_tracks_a_lane_on_real_physics_and_stops_at_its_end) {
    PhysicsWorld world;
    world.initialize();
    // The stock 50 m test floor ends mid-route (first run: the car tracked the
    // lane, crossed z=50, and "arrived" in free fall at y=-231). The lane is
    // 100 m, so the ground must outlast it.
    world.addBox(Vec3(30, 1, 120), Vec3(0, -1, 50), Quat::identity(),
                 BodyMotion::Static);

    // Face +Z at the origin; the lane runs straight down +Z for 100 m.
    PhysicsWorld::VehicleId car =
        world.addVehicle(sedanConfig(), Vec3(0, 1.0, 0), Quat::identity());
    CHECK(car != PhysicsWorld::INVALID_VEHICLE);

    LaneFollower lf;
    {
        std::vector<Vec2> path;
        for (int z = 0; z <= 100; z += 5)
            path.push_back(Vec2(0, static_cast<Real>(z)));
        lf.setPath(path);
    }

    Real worstLateral = 0;
    bool reachedEnd = false;
    for (int i = 0; i < 60 * 45 && !reachedEnd; ++i) {
        const Vec3 pos = world.vehiclePosition(car);
        const Quat q = world.vehicleOrientation(car);
        const Vec3 fwd3 = q.rotate(Vec3(0, 0, 1));
        const Vec3 vel = world.vehicleVelocity(car);

        const Vec2 pos2(pos.x, pos.z);
        const Real lateral = lf.update(pos2);
        // Ignore the first settle second (suspension drop); after that the
        // car must hold the lane. 2 m is a full lane-width miss.
        if (i > 60) worstLateral = std::max(worstLateral, std::fabs(lateral));

        DriverState s;
        s.forward = Vec2(fwd3.x, fwd3.z);
        s.speed = vel.x * fwd3.x + vel.y * fwd3.y + vel.z * fwd3.z;
        const DriverCommand cmd = pursuitCommand(
            lf, pos2, /*desiredSpeed=*/10.0, pursuitLookahead(s.speed));
        const DriverInput in = computeDriverInput(s, cmd);
        world.setVehicleInput(car, in.throttle, in.steer, in.brake,
                              in.handBrake);
        world.update(1.0 / 60.0);

        if (lf.remaining() < 1.5 && std::fabs(s.speed) < 0.5) reachedEnd = true;
    }

    const Vec3 endPos = world.vehiclePosition(car);
    // Probe print (house style): the trajectory summary that explains any
    // failure without rerunning under a debugger.
    std::printf("    [agentdrive] end=(%.1f, %.1f, %.1f) worstLat=%.2f "
                "reached=%d\n",
                endPos.x, endPos.y, endPos.z, worstLateral, reachedEnd ? 1 : 0);
    CHECK(reachedEnd);                 // covered the 100 m and came to rest
    CHECK(worstLateral < 2.0);         // held the lane the whole way
    CHECK(endPos.z > 92.0);            // stopped AT the end, not short of it
    CHECK(std::fabs(endPos.x) < 2.0);  // and on the line, not beside it
    world.shutdown();
}

// --- JoltJobAdapter under exhaustion (the 0x27ff0 crash class) --------------
// Three live crashes shared one address: the adapter's job free list came up
// empty (whole budget in flight during a SimClock catch-up burst) and the
// assert-only guard let Get(cInvalidObjectIndex) ship a wild Job*. CreateJob
// now waits for a slot, Jolt-thread-pool style. This hammer drives a TINY pool
// (32 slots) with 40x that many jobs from Jolt's own barrier machinery — the
// pre-fix adapter dies here in seconds; the fix makes exhaustion mean
// "briefly wait", never "wild pointer".
#include "../src/engine/physics/jolt_job_adapter.h"
#include <atomic>
#include <chrono>
#include <thread>

TEST_CASE(jolt_job_adapter_survives_transient_pool_exhaustion) {
    // The RUNTIME pattern, miniaturized: PhysicsSystem::Update runs one
    // barrier per step, back to back — and WaitForJobs returns when jobs have
    // EXECUTED, while their Release() (which frees the slot) happens a beat
    // later on a worker. So the next step's CreateJob burst races a free list
    // that is transiently short — with a step-burst frame running 8 barriers
    // in a row, transiently EMPTY. 500 rounds at 24 jobs against a 32-slot
    // pool makes that window a certainty many times over; the pre-fix adapter
    // ships a wild Job* out of one of these rounds, the fixed one briefly
    // waits and completes every job. (True intra-barrier exhaustion is NOT
    // retryable by design — the construction site carries 4x headroom for
    // that; see jolt_job_adapter.cpp.)
    engine::JobSystem pool(4);
    {
        JoltJobAdapter adapter(pool, /*maxJobs=*/32, /*maxBarriers=*/4);
        std::atomic<int> executed{0};
        constexpr int kRounds = 500;
        constexpr int kJobsPerRound = 24;
        for (int r = 0; r < kRounds; ++r) {
            JPH::JobSystem::Barrier* barrier = adapter.CreateBarrier();
            for (int i = 0; i < kJobsPerRound; ++i) {
                JPH::JobSystem::JobHandle handle = adapter.CreateJob(
                    "hammer", JPH::Color::sWhite,
                    [&executed] {
                        executed.fetch_add(1, std::memory_order_relaxed);
                    });
                barrier->AddJob(handle);
            }
            adapter.WaitForJobs(barrier);
            adapter.DestroyBarrier(barrier);
        }
        CHECK(executed.load() == kRounds * kJobsPerRound);
    }   // ~JoltJobAdapter drains its outstanding count before the pool dies
}
