// RIDING A BUS (Glenn, 2026-09-18: "when I'm in the bus there's a lot of crazy
// jittering. The bus isn't stable. Should the player be able to move freely
// around the bus interior and find a seat?"). The passenger lives in the bus's
// own frame and is placed through the exact matrix the bus body is drawn with,
// so, standing still, the player must not move RELATIVE TO THE BUS at all --
// however the bus drives. Then: the aisle is walkable, a free seat takes you,
// and the doors only let you off at a stop.
#include "test_framework.h"
#include "../src/apps/citysim/city_player_transit.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/camera/fly_camera_controller.h"
#include "../src/engine/components.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/world.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;
using namespace citysim;

namespace {

RoadEntity rideGrid() {
    RoadEntity net;
    net.look.defaultWidth = 10.0;
    net.look.sidewalk = 2.5;
    const int N = 5;
    const Real pitch = 90;
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i)
            net.graph.nodes.push_back(RoadNode{Vec2(i * pitch, j * pitch)});
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) {
            if (i + 1 < N) net.graph.addEdge(j * N + i, j * N + i + 1, net.look.defaultWidth);
            if (j + 1 < N) net.graph.addEdge(j * N + i, (j + 1) * N + i, net.look.defaultWidth);
        }
    return net;
}

std::string readAsset(const std::string& name) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/" + name);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

struct StubUploader : engine::MeshUploader {
    uint32_t next = 1;
    engine::MeshHandle uploadMesh(const engine::RenderMesh&) override {
        return engine::MeshHandle{next++, 1};
    }
    void removeMesh(engine::MeshHandle) override {}
    engine::BoundingSphere getMeshBounds(engine::MeshHandle) const override { return {}; }
};

Vec3 inFrame(const Mat4& pose, const Vec3& p) { return pose.inverse().transformPoint(p); }

}  // namespace

TEST_CASE(a_bus_passenger_rides_in_the_bus_frame) {
    World world;
    world.add<RoadEntity>(world.create(), rideGrid());
    CityRenderParams params;
    params.cars = 12;
    params.pedestrians = 0;
    params.seed = 7;
    params.wander = true;
    params.busRoutes = 1;
    params.busStops = 8;
    params.buses = 2;
    params.busMaxWalk = 200;
    params.vehicleScript = readAsset("vehicles.lua");
    CityRenderSystem city(params);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    CHECK(city.build(world, &assets));
    CHECK(!city.busSeats().empty());
    CHECK(city.busDoors().size() == 2);

    Entity player = world.create();
    Transform pt;
    pt.position = Vec3(180, 1.1, 180);
    world.add<Transform>(player, pt);
    world.add<PrevTransform>(player, PrevTransform{pt});
    world.add<CharacterController>(player, CharacterController{});
    world.add<ControlledBy>(player, ControlledBy{});
    PhysicsSystem physics;
    physics.initialize();
    physics.physicsWorld().addBox(Vec3(600, 1, 600), Vec3(180, -1, 180), Quat::identity(),
                                  BodyMotion::Static);
    physics.createBodies(world);
    FlyCameraController fly;
    CityPlayerTransitSystem transit(city, physics, fly);

    const Real dt = 1.0 / 60.0;
    auto tick = [&](const CityPlayerTransitSystem::RideInput& in) {
        city.step(world, dt);
        physics.step(world, dt);
        transit.step(world, dt, in);
    };
    // A bus standing at a stop, and the player beside it.
    int bus = -1;
    for (int i = 0; i < 60 * 120 && bus < 0; ++i) {
        tick({});
        for (int a = 0; a < static_cast<int>(city.sim().agents().size()); ++a)
            if (city.sim().isBus(a) && city.sim().agents()[static_cast<std::size_t>(a)].speed <= 0.5 &&
                city.sim().agents()[static_cast<std::size_t>(a)].busDwell > 1.0) { bus = a; break; }
    }
    CHECK(bus >= 0);
    if (bus < 0) return;
    const Vec2 bp = city.sim().agents()[static_cast<std::size_t>(bus)].pos;
    world.get<Transform>(player)->position = Vec3(bp.x + 3, 1.1, bp.y);
    CityPlayerTransitSystem::RideInput board;
    board.request = 1.0;
    tick(board);
    CHECK(transit.riding() == bus);
    CHECK(world.has<Passenger>(player));

    // STANDING STILL FOR 30 s: fixed in the bus's drawn frame.
    Mat4 pose;
    CHECK(city.busFrame(bus, &pose));
    const Vec3 start = inFrame(pose, world.get<Transform>(player)->position);
    const Vec3 busStart = pose.transformPoint(Vec3(0, 0, 0));
    Real drift = 0, travel = 0;
    for (int i = 0; i < 60 * 30; ++i) {
        tick({});
        if (transit.riding() != bus) break;
        CHECK(city.busFrame(bus, &pose));
        const Vec3 here = inFrame(pose, world.get<Transform>(player)->position);
        drift = std::max(drift, (here - start).length());
        travel = std::max(travel, (pose.transformPoint(Vec3(0, 0, 0)) - busStart).length());
    }
    std::printf("    [ride] bus travelled %.0f m; passenger drift in the bus frame %.6f m\n",
                travel, drift);
    CHECK(transit.riding() == bus);
    CHECK(travel > 40.0);
    CHECK(drift < 1e-3);

    // WALKING the aisle: forward for 2 s moves you along the saloon and keeps
    // you inside it.
    const Vec3 before = transit.aboardAt();
    fly.yaw = 0;
    for (int i = 0; i < 120; ++i) {
        CityPlayerTransitSystem::RideInput walk;
        walk.forward = 1.0;
        tick(walk);
    }
    const Vec3 after = transit.aboardAt();
    std::printf("    [ride] walked from (%.2f, %.2f) to (%.2f, %.2f) in the bus frame\n",
                before.x, before.z, after.x, after.z);
    CHECK((after - before).length() > 0.5);
    CHECK(std::fabs(after.x) <= 0.31 + 0.01 || std::fabs(after.x) < 1.3);

    // SITTING: walk to a free seat's row and press E; E again stands you up.
    CityPlayerTransitSystem::RideInput e;
    e.interact = true;
    tick(e);
    std::printf("    [ride] E in the aisle -> seat %d\n", transit.seatHeld());
    CHECK(transit.seatHeld() >= 0);
    tick(e);
    CHECK(transit.seatHeld() == -1);

    // THE DOORS: not while moving; yes at a stop. Walk to the middle door.
    const Vec3 door = city.busDoors()[1];
    int doorTries = 0;
    bool off = false;
    for (int i = 0; i < 60 * 180 && !off; ++i) {
        CityPlayerTransitSystem::RideInput in;
        const Vec3 at = transit.aboardAt();
        // steer toward the door in the bus frame (fly yaw tracks the bus, so
        // map the local direction through the current frame)
        CHECK(city.busFrame(bus, &pose));
        const Vec3 wantW = pose.transformPoint(door) - pose.transformPoint(at);
        const Vec3 f = fly.forward(), r = fly.right();
        in.forward = std::clamp(Real(wantW.x * f.x + wantW.z * f.z), Real(-1), Real(1));
        in.right = std::clamp(Real(wantW.x * r.x + wantW.z * r.z), Real(-1), Real(1));
        if ((Vec3(at.x, 0, at.z) - Vec3(door.x, 0, door.z)).length() < 0.8 && i % 30 == 0) {
            in.interact = true;
            ++doorTries;
        }
        const bool moving = city.sim().agents()[static_cast<std::size_t>(bus)].speed > 2.0;
        tick(in);
        if (transit.riding() < 0) {
            off = true;
            CHECK(!moving);   // it let us off only while standing
        }
    }
    std::printf("    [ride] got off after %d tries at the door\n", doorTries);
    CHECK(off);
    CHECK(!world.has<Passenger>(player));
}
