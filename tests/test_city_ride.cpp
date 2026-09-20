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
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/world.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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

// HOW SMOOTH IS THE BUS ITSELF (Glenn, 2026-09-18: "there's still a lot of
// jitter from the bus movement"). The passenger is locked to the bus, so any
// unevenness in the bus's own drawn motion is the whole world shaking. At
// metro's sim rate (localHz 30: a tick every other fixed step), on the drawn
// matrix, per fixed step: jerk |p[k+1] - 2p[k] + p[k-1]| (an even glide is ~0)
// and the jump in yaw rate. Measured before: jerk 0.0127 m mean / 0.19 max, yaw
// jump 0.013 rad -- from the pull-out easing advancing in 30 Hz chunks (and on
// alternate ticks only: the stop-line approach skipped its counter), and a bus
// arriving at its stop at 5.6 m/s and standing still the next tick.
TEST_CASE(a_bus_glides_at_the_city_s_sim_rate) {
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
    params.localHz = 30;
    params.vehicleScript = readAsset("vehicles.lua");
    CityRenderSystem city(params);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    CHECK(city.build(world, &assets));
    int bus = -1;
    for (int a = 0; a < static_cast<int>(city.sim().agents().size()); ++a)
        if (city.sim().isBus(a)) { bus = a; break; }
    CHECK(bus >= 0);
    std::vector<Vec3> p;
    std::vector<Real> yaw, speed;
    for (int i = 0; i < 60 * 60; ++i) {
        city.step(world, 1.0 / 60.0);
        Mat4 pose;
        if (!city.busFrame(bus, &pose)) continue;
        p.push_back(pose.transformPoint(Vec3(0, 0, 0)));
        yaw.push_back(std::atan2(pose.m[0][2], pose.m[2][2]));
        speed.push_back(city.sim().agents()[static_cast<std::size_t>(bus)].speed);
    }
    auto wrap = [](Real d) {
        while (d > PI) d -= 2 * PI;
        while (d < -PI) d += 2 * PI;
        return d;
    };
    Real jerkMax = 0, jerkSum = 0, yawJump = 0;
    int n = 0;
    for (std::size_t k = 1; k + 1 < p.size(); ++k) {
        if (speed[k] < 2.0) continue;
        const Vec3 j = p[k + 1] - p[k] * 2.0 + p[k - 1];
        const Real jh = std::sqrt(j.x * j.x + j.z * j.z);
        jerkMax = std::max(jerkMax, jh);
        jerkSum += jh;
        yawJump = std::max(yawJump, std::fabs(wrap(yaw[k + 1] - yaw[k]) - wrap(yaw[k] - yaw[k - 1])));
        ++n;
    }
    std::printf("    [smooth] %d moving steps: jerk mean %.4f max %.4f m/step^2, yaw-rate jump "
                "max %.4f rad/step\n", n, n ? jerkSum / n : 0.0, jerkMax, yawJump);
    CHECK(n > 1000);
    CHECK(jerkSum / n < 0.002);
    CHECK(jerkMax < 0.06);
    CHECK(yawJump < 0.005);
}
