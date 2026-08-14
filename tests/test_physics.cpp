#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include "../src/job_system.h"
#include <vector>
#include <cstdio>

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

// --- XR grasp primitives: grip springs, joints, layers, contacts ----------

TEST_CASE(grip_spring_holds_object_at_grabbed_offset) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    // Grip anchor (kinematic, no collisions) 20cm above a dynamic crate:
    // hook, then move the grip — the crate must follow, keeping the offset.
    PhysicsBodyId grip = world.addSphere(0.01, Vec3(0, 1.2, 0),
                                         Quat::identity(),
                                         BodyMotion::Kinematic);
    world.setBodyLayer(grip, PhysicsWorld::BodyLayer::Grip);
    PhysicsBodyId crate = world.addBox(Vec3(0.075, 0.075, 0.075),
                                       Vec3(0, 1.0, 0), Quat::identity(),
                                       BodyMotion::Dynamic, 0.0, 0.7);
    const ConstraintId spring = world.addGripSpring(grip, crate);
    CHECK(spring != INVALID_CONSTRAINT);

    // Carry sideways a metre over a second, then HOLD (kinematic bodies
    // keep their last moveKinematic velocity, so the hold must keep
    // commanding the rest position).
    for (int i = 0; i <= 60; i++) {
        world.moveKinematic(grip, Vec3(i / 60.0, 1.2, 0), Quat::identity(),
                            1.0 / 60.0);
        world.update(1.0 / 60.0);
    }
    for (int i = 0; i < 30; i++) {
        world.moveKinematic(grip, Vec3(1.0, 1.2, 0), Quat::identity(),
                            1.0 / 60.0);
        world.update(1.0 / 60.0);
    }
    const Vec3 p = world.bodyPosition(crate);
    CHECK(std::abs(p.x - 1.0) < 0.05);        // followed the carry
    CHECK(std::abs(p.y - 1.0) < 0.05);        // kept the 20cm-below offset

    // Unhook: the crate is dynamic and simply falls to the floor.
    world.removeConstraint(spring);
    step(world, 120);
    CHECK(world.bodyPosition(crate).y < 0.2);
    world.shutdown();
}

TEST_CASE(hinge_respects_limits) {
    PhysicsWorld world;
    world.initialize();
    // A static base and a dynamic lid hinged at the base's edge; gravity
    // swings the lid down but the 0..110deg limits stop it.
    PhysicsBodyId base = world.addBox(Vec3(0.1, 0.02, 0.1), Vec3(0, 1, 0),
                                      Quat::identity(), BodyMotion::Static);
    PhysicsBodyId lid = world.addBox(Vec3(0.1, 0.01, 0.1), Vec3(0, 1.03, 0.2),
                                     Quat::identity(), BodyMotion::Dynamic);
    const ConstraintId hinge = world.addHinge(
        base, lid, Vec3(0, 1.03, 0.1), Vec3(1, 0, 0), 0.0, 110.0 * PI / 180.0);
    CHECK(hinge != INVALID_CONSTRAINT);
    step(world, 240);
    // The lid hangs at a limit rather than spinning free: it stays within
    // its travel circle around the pivot (radius ~0.1 + slop).
    const Vec3 p = world.bodyPosition(lid);
    const Vec3 pivot(0, 1.03, 0.1);
    CHECK((p - pivot).length() < 0.16);
    world.shutdown();
}

TEST_CASE(slider_stays_within_travel) {
    PhysicsWorld world;
    world.initialize();
    PhysicsBodyId base = world.addBox(Vec3(0.1, 0.02, 0.02), Vec3(0, 1, 0),
                                      Quat::identity(), BodyMotion::Static);
    PhysicsBodyId bolt = world.addBox(Vec3(0.02, 0.02, 0.02), Vec3(0, 1.05, 0),
                                      Quat::identity(), BodyMotion::Dynamic);
    // Vertical slider, 0..6cm of travel: gravity pulls the bolt to the low
    // stop; a kick upward may not exceed the high stop.
    const ConstraintId slider = world.addSlider(
        base, bolt, Vec3(0, 1.05, 0), Vec3(0, 1, 0), -0.03, 0.03);
    CHECK(slider != INVALID_CONSTRAINT);
    step(world, 120);
    CHECK(world.bodyPosition(bolt).y > 1.05 - 0.05);   // held by low stop
    world.setLinearVelocity(bolt, Vec3(0, 3.0, 0));
    step(world, 30);
    CHECK(world.bodyPosition(bolt).y < 1.05 + 0.05);   // capped by high stop
    world.shutdown();
}

TEST_CASE(hand_layer_ignores_static_hits_dynamic) {
    PhysicsWorld world;
    world.initialize();
    addMeshFloor(world);
    // A kinematic "fingertip" on the HAND layer sweeps down THROUGH the
    // static mesh floor (no grinding on the real room)...
    PhysicsBodyId tip = world.addSphere(0.012, Vec3(0, 0.5, 0),
                                        Quat::identity(),
                                        BodyMotion::Kinematic);
    world.setBodyLayer(tip, PhysicsWorld::BodyLayer::Hand);
    for (int i = 0; i <= 60; i++) {
        world.moveKinematic(tip, Vec3(0, 0.5 - i / 60.0, 0),
                            Quat::identity(), 1.0 / 60.0);
        world.update(1.0 / 60.0);
    }
    CHECK(world.bodyPosition(tip).y < -0.4);   // passed the floor unimpeded

    // ...but the same layer SHOVES a dynamic crate.
    PhysicsBodyId crate = world.addBox(Vec3(0.075, 0.075, 0.075),
                                       Vec3(2, 0.075, 0), Quat::identity(),
                                       BodyMotion::Dynamic, 0.0, 0.3);
    world.teleport(tip, Vec3(1.7, 0.075, 0), Quat::identity());
    for (int i = 0; i <= 30; i++) {
        world.moveKinematic(tip, Vec3(1.7 + 0.4 * i / 30.0, 0.075, 0),
                            Quat::identity(), 1.0 / 60.0);
        world.update(1.0 / 60.0);
    }
    CHECK(world.bodyPosition(crate).x > 2.02);   // got pushed
    world.shutdown();
}

TEST_CASE(active_contacts_track_touch_and_orient_the_normal) {
    PhysicsWorld world;
    world.initialize();
    addFloor(world);
    PhysicsBodyId crate = world.addBox(Vec3(0.075, 0.075, 0.075),
                                       Vec3(0, 0.5, 0), Quat::identity(),
                                       BodyMotion::Dynamic, 0.0, 0.7);
    // Fall (~0.3s) + a short rest — but NOT long enough for the island to
    // SLEEP: Jolt drops sleeping pairs from the contact cache, so
    // activeContacts covers awake bodies (the grasp stack's hands are
    // kinematic and keep everything they touch awake — the caveat never
    // bites there).
    step(world, 40);

    // Resting contact must be VISIBLE (persisted, not just the add edge),
    // with the normal pointing AWAY from the queried body: query the crate
    // and the floor pushes up at it -> normal.y < 0 from the crate's view?
    // No — away from the crate means toward the floor: normal points from
    // the crate outward through the contact, i.e. DOWN.
    std::vector<ContactEvent> touches;
    world.activeContacts(crate, touches);
    CHECK(!touches.empty());
    bool foundFloorTouch = false;
    for (const ContactEvent& t : touches) {
        CHECK(t.bodyA == crate);
        if (t.normal.y < -0.9) foundFloorTouch = true;
    }
    CHECK(foundFloorTouch);

    // Lift the crate away: the touch entry must clear.
    world.teleport(crate, Vec3(0, 2, 0), Quat::identity());
    step(world, 5);
    touches.clear();
    world.activeContacts(crate, touches);
    CHECK(touches.empty());
    world.shutdown();
}
