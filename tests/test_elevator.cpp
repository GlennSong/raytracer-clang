// Elevators (skyscrapers v2 M5/M6, elevator_system.h): the bank of the
// building the player is in, headless — a call from the lobby opens the
// nearest cab, a pick inside carries it to the floor, leaving frees it all.
// The walker's ride itself is test_physics's character_rides_a_kinematic_platform.
#include "test_framework.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/physics/physics_world.h"
#include "../src/engine/procgen/city/building_records.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/systems/elevator_system.h"
#include "../src/engine/world.h"

#include <cmath>

using namespace engine;

namespace {
struct StubUploader : MeshUploader {
    uint32_t next = 1;
    MeshHandle uploadMesh(const RenderMesh&) override { return MeshHandle{next++, 1}; }
    void removeMesh(MeshHandle) override {}
    BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
};
}  // namespace

TEST_CASE(elevator_answers_a_call_and_carries_a_pick_to_its_floor) {
    World world;
    PhysicsWorld phys;
    phys.initialize();
    StubUploader uploader;
    AssetManager assets(uploader);

    CityBuildings cb;
    BuildingRecord r;
    r.plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    r.baseY = 0;
    r.groundY = -0.45;
    r.params.floors = 30;
    r.params.curtainWall = true;
    r.params.walkableGround = true;
    r.params.openDoorway = true;
    r.params.seed = 5;
    r.height = r.params.groundHeight + 30 * r.params.floorHeight;
    r.doors.push_back({Vec2(20, 40), Vec2(0, 1), 2.0, 2.7});
    r.enterable = true;
    cb.records.push_back(r);
    cb.buildIndex();
    world.add<CityBuildings>(world.create(), std::move(cb));

    const CorePlan core = coreFor(r.plan, r.params, entranceEdgeFor(r.plan, r.params));
    CHECK(core.valid);
    CHECK(core.hoistways.size() == 3);
    const CoreShaft& hw = core.hoistways[0];
    const Real dt = 1.0 / 60.0;
    const int baseline = phys.bodyCount();

    ElevatorSystem sys(nullptr);
    // In front of hoistway 0's door on the ground floor: E calls the cab.
    const Vec2 front = hw.frame.toWorld({hw.doorX, -1.2});
    const Vec3 pFront(front.x, 0.05 + 0.7, front.y);
    sys.step(world, &phys, assets, pFront, dt, true, 0);
    CHECK(sys.bankCount() == 1);
    CHECK(sys.status().atDoor);
    CHECK(!sys.status().inCab);
    CHECK(phys.bodyCount() > baseline);   // cabs and door leaves are bodies
    for (int i = 0; i < 90; ++i) sys.step(world, &phys, assets, pFront, dt, false, 0);
    // Step in, pick five floors up, go.
    const Vec2 in = hw.frame.toWorld({hw.width * 0.5, 1.0});
    Vec3 pIn(in.x, 0.05 + 0.7, in.y);
    sys.step(world, &phys, assets, pIn, dt, false, 5);
    CHECK(sys.status().inCab);
    CHECK(sys.status().selected == 5);
    CHECK(sys.status().floor == 0);
    sys.step(world, &phys, assets, pIn, dt, true, 0);
    const Real target = 0.05 + r.params.groundHeight + 4 * r.params.floorHeight;
    int steps = 0;
    bool arrived = false;
    for (; steps < 60 * 40; ++steps) {
        pIn.y = sys.cabY(0, 0) + 0.7;   // ride along (the physics transfer is tested elsewhere)
        sys.step(world, &phys, assets, pIn, dt, false, 0);
        if (std::fabs(sys.cabY(0, 0) - target) < 1e-6 && !sys.status().moving) { arrived = true; break; }
    }
    CHECK(arrived);
    CHECK(sys.status().inCab);
    CHECK(sys.status().floor == 5);
    // Doors close 1.2 s, 17.3 m at 3 m/s with 1.5 m/s^2 ramps ~7.8 s: well under 12 s.
    CHECK(steps < 60 * 12);
    // Leaving the building frees the bank and its bodies.
    sys.step(world, &phys, assets, Vec3(500, 1, 500), dt, false, 0);
    CHECK(sys.bankCount() == 0);
    CHECK(phys.bodyCount() == baseline);
    phys.shutdown();
}

// A REAL walker through the core: in the lobby, through stairwell A's door,
// up flight A to the half landing, up flight B to storey 1's landing — and
// then through a hoistway door into the cab once the bank has opened it.
TEST_CASE(character_walks_the_stairwell_door_and_climbs_the_dog_leg) {
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    BuildingParams p;
    p.floors = 12;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 5;
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    CHECK(core.valid);
    RenderMesh collider;
    growInterior(plan, p, 0.0, &collider, 0, 3);
    CHECK(!collider.indices.empty());

    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(60, 0.5, 60), Vec3(20, -0.45, 20), Quat::identity(), BodyMotion::Static);   // the lobby slab top at 0.05
    std::vector<Vec3> verts;
    for (const Vertex& v : collider.vertices) verts.push_back(v.position);
    std::vector<uint32_t> idx = collider.indices;
    const std::size_t oneSided = idx.size();
    for (std::size_t i = 0; i + 2 < oneSided; i += 3) { idx.push_back(idx[i]); idx.push_back(idx[i + 2]); idx.push_back(idx[i + 1]); }
    world.addMesh(verts, idx, Vec3(), 0.85);

    const CoreStair& st = core.stairs[0];
    const CoreShaft& sh = st.shaft;
    auto at = [&](Real u, Real v) { const Vec2 w = sh.frame.toWorld({u, v}); return Vec3(w.x, 0, w.y); };
    const Vec3 start = at(sh.doorX, -2.0);
    CharacterId c = world.addCharacter(0.4, 0.3, Vec3(start.x, 0.05 + 0.7 + 0.2, start.z));
    world.optimizeBroadPhase();
    for (int i = 0; i < 60; ++i) world.moveCharacter(c, Vec3(), 1.0 / 60.0);
    CHECK_APPROX(world.characterPosition(c).y, 0.75, 0.15);
    auto walkTo = [&](const Vec3& target, int maxFrames) {
        for (int i = 0; i < maxFrames; ++i) {
            const Vec3 q = world.characterPosition(c);
            const Real dx = target.x - q.x, dz = target.z - q.z;
            const Real len = std::sqrt(dx * dx + dz * dz);
            if (len < 0.12) break;
            world.moveCharacter(c, Vec3(dx / len * 1.2, 0, dz / len * 1.2), 1.0 / 60.0);
        }
    };
    auto frameOf = [&](const Vec3& q) { return sh.frame.toFrame(Vec2(q.x, q.z)); };
    // Through the door onto the landing.
    walkTo(at(sh.doorX, 0.8), 600);
    Vec3 q = world.characterPosition(c);
    Vec2 f = frameOf(q);
    std::printf("    [core-walk] landing u=%.2f v=%.2f y=%.2f\n", f.x, f.y, q.y);
    CHECK(f.y > 0.5);
    // Up flight A (u = 0.6, v climbing) to the half landing.
    const int nR = halfFlightRisers(4.5);
    const Real run = nR * st.tread;
    walkTo(at(0.6, st.landing + run + 0.6), 1500);
    q = world.characterPosition(c);
    f = frameOf(q);
    std::printf("    [core-walk] half landing u=%.2f v=%.2f y=%.2f (expect y ~ %.2f)\n", f.x, f.y, q.y, 0.05 + 2.25 + 0.7);
    CHECK(f.y > st.landing + run - 0.3);
    CHECK_APPROX(q.y, 0.05 + 2.25 + 0.7, 0.3);
    // Across to flight B and down... up to storey 1's landing.
    walkTo(at(2.0, st.landing + run + 0.6), 400);
    walkTo(at(2.0, 0.6), 1500);
    q = world.characterPosition(c);
    f = frameOf(q);
    std::printf("    [core-walk] storey 1 landing u=%.2f v=%.2f y=%.2f (expect y ~ %.2f)\n", f.x, f.y, q.y, 4.55 + 0.7);
    CHECK(f.y < 1.2);
    CHECK_APPROX(q.y, 4.55 + 0.7, 0.3);
    // Out through storey 1's stair door into the corridor.
    walkTo(at(sh.doorX, -1.5), 600);
    q = world.characterPosition(c);
    f = frameOf(q);
    std::printf("    [core-walk] corridor u=%.2f v=%.2f y=%.2f\n", f.x, f.y, q.y);
    CHECK(f.y < -1.0);
    CHECK_APPROX(q.y, 4.55 + 0.7, 0.3);
    world.shutdown();
}

TEST_CASE(character_enters_the_cab_through_an_open_hoistway_door) {
    World world;
    PhysicsWorld phys;
    phys.initialize();
    StubUploader uploader;
    AssetManager assets(uploader);
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    CityBuildings cb;
    BuildingRecord r;
    r.plan = plan;
    r.baseY = 0;
    r.groundY = -0.45;
    r.params.floors = 12;
    r.params.curtainWall = true;
    r.params.walkableGround = true;
    r.params.openDoorway = true;
    r.params.seed = 5;
    r.height = r.params.groundHeight + 12 * r.params.floorHeight;
    r.doors.push_back({Vec2(20, 40), Vec2(0, 1), 2.0, 2.7});
    r.enterable = true;
    cb.records.push_back(r);
    cb.buildIndex();
    world.add<CityBuildings>(world.create(), std::move(cb));
    const CorePlan core = coreFor(plan, r.params, entranceEdgeFor(plan, r.params));
    CHECK(core.valid);
    RenderMesh collider;
    growInterior(plan, r.params, 0.0, &collider, 0, 3);
    phys.addBox(Vec3(60, 0.5, 60), Vec3(20, -0.45, 20), Quat::identity(), BodyMotion::Static);
    std::vector<Vec3> verts;
    for (const Vertex& v : collider.vertices) verts.push_back(v.position);
    std::vector<uint32_t> idx = collider.indices;
    const std::size_t oneSided = idx.size();
    for (std::size_t i = 0; i + 2 < oneSided; i += 3) { idx.push_back(idx[i]); idx.push_back(idx[i + 2]); idx.push_back(idx[i + 1]); }
    phys.addMesh(verts, idx, Vec3(), 0.85);

    const CoreShaft& hw = core.hoistways[0];
    auto at = [&](Real u, Real v) { const Vec2 w = hw.frame.toWorld({u, v}); return Vec3(w.x, 0, w.y); };
    const Vec3 start = at(hw.doorX, -1.5);
    CharacterId c = phys.addCharacter(0.4, 0.3, Vec3(start.x, 0.95, start.z));
    phys.optimizeBroadPhase();
    const Real dt = 1.0 / 60.0;
    ElevatorSystem sys(nullptr);
    auto stepAll = [&](const Vec3& vel, bool call) {
        phys.moveCharacter(c, vel, dt);
        const Vec3 pp = phys.characterPosition(c);
        sys.step(world, &phys, assets, pp, dt, call, 0);
        phys.update(dt);
    };
    for (int i = 0; i < 30; ++i) stepAll(Vec3(), false);
    // Closed doors: walking at the hoistway gets nowhere.
    auto toward = [&](const Vec3& target) {
        const Vec3 q = phys.characterPosition(c);
        const Real dx = target.x - q.x, dz = target.z - q.z;
        const Real len = std::sqrt(dx * dx + dz * dz);
        return len < 0.05 ? Vec3() : Vec3(dx / len * 1.2, 0, dz / len * 1.2);
    };
    const Vec3 inCab = at(hw.width * 0.5, 1.0);
    for (int i = 0; i < 120; ++i) stepAll(toward(inCab), false);
    Vec2 f = hw.frame.toFrame(Vec2(phys.characterPosition(c).x, phys.characterPosition(c).z));
    std::printf("    [cab-walk] against closed doors: u=%.2f v=%.2f\n", f.x, f.y);
    {   // What is in the way? Probe rays at knee, waist and head height.
        const Vec3 q = phys.characterPosition(c);
        const Vec3 dir = toward(inCab);
        for (Real dy : {-0.5, 0.0, 0.5}) {
            Vec3 hit;
            const bool h = phys.castRay(Vec3(q.x, q.y + dy, q.z), Vec3(dir.x, 0, dir.z) * 3.0, hit);
            const Vec2 hf = hw.frame.toFrame(Vec2(hit.x, hit.z));
            std::printf("    [cab-walk] ray dy=%.1f hit=%d at u=%.2f v=%.2f y=%.2f\n", dy, h ? 1 : 0, hf.x, hf.y, hit.y);
        }
        const Vec2 s0 = hw.frame.toFrame(Vec2(start.x, start.z));
        std::printf("    [cab-walk] start u=%.2f v=%.2f; cab floor y=%.2f bodies=%d\n", s0.x, s0.y, sys.cabY(0, 0), phys.bodyCount());
        // Where are the leaves? Rays along +v at several u across the door, from v = -1.0.
        const Vec3 vdir(hw.frame.v.x, 0, hw.frame.v.y);
        for (Real u : {0.7, 0.95, 1.2, 1.45, 1.7}) {
            const Vec3 o = hw.at(u, 1.0, -1.0);
            Vec3 hit;
            const bool h = phys.castRay(o, vdir * 2.0, hit);
            const Vec2 hf = hw.frame.toFrame(Vec2(hit.x, hit.z));
            std::printf("    [cab-walk] door ray u=%.2f hit=%d at u=%.2f v=%.2f\n", u, h ? 1 : 0, hf.x, hf.y);
        }
        // Lateral rays from the stuck spot.
        const Vec3 udir(hw.frame.u.x, 0, hw.frame.u.y);
        for (Real sgn : {-1.0, 1.0}) {
            Vec3 hit;
            const bool h = phys.castRay(Vec3(q.x, q.y, q.z), udir * (sgn * 2.0), hit);
            const Vec2 hf = hw.frame.toFrame(Vec2(hit.x, hit.z));
            std::printf("    [cab-walk] side ray sgn=%.0f hit=%d at u=%.2f v=%.2f y=%.2f\n", sgn, h ? 1 : 0, hf.x, hf.y, hit.y);
        }
        // Forward rays at the capsule's flanks.
        for (Real du : {-0.25, 0.25}) {
            const Vec3 o = hw.at(f.x + du, q.y, f.y);
            Vec3 hit;
            const bool h = phys.castRay(o, vdir * 3.0, hit);
            const Vec2 hf = hw.frame.toFrame(Vec2(hit.x, hit.z));
            std::printf("    [cab-walk] flank ray du=%.2f hit=%d at u=%.2f v=%.2f y=%.2f\n", du, h ? 1 : 0, hf.x, hf.y, hit.y);
        }
    }
    CHECK(f.y < -0.05);
    // Call, wait for the doors, walk in.
    stepAll(Vec3(), true);
    CHECK(sys.status().atDoor);
    for (int i = 0; i < 100; ++i) stepAll(Vec3(), false);
    for (int i = 0; i < 240; ++i) stepAll(toward(inCab), false);
    const Vec3 q = phys.characterPosition(c);
    f = hw.frame.toFrame(Vec2(q.x, q.z));
    std::printf("    [cab-walk] in the cab: u=%.2f v=%.2f y=%.2f\n", f.x, f.y, q.y);
    CHECK(f.y > 0.6);
    CHECK(sys.status().inCab);
    CHECK_APPROX(q.y, 0.05 + 0.7, 0.15);
    phys.shutdown();
}
