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
