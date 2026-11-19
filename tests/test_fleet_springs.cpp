// EVERY FLEET CAR ON ITS OWN SPRINGS (#172): each fleet slot's handling, put down by PhysicsWorld::spawnHeight. Moved
// here from test_city_phys_tier.cpp when the old driving's physical tier and its tests went (ADR-0461).
#include "test_framework.h"

#include "../src/apps/citysim/city_render.h"
#include "../src/engine/components.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/physics/physics_world.h"
#include "../src/engine/world.h"
#include "street_map_test_util.h"   // a test world's street map, from its road entities (ADR-0425)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;
using namespace citysim;

namespace {

// A 4x4 junction grid, 80 m pitch: a city to load the fleet in.
RoadEntity springsGrid() {
    RoadEntity net;
    net.look.defaultWidth = 10.0;
    net.look.sidewalk = 2.5;
    const int N = 4;
    const Real pitch = 80;
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i)
            net.graph.nodes.push_back(RoadNode{Vec2(i * pitch, j * pitch)});
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) {
            if (i + 1 < N)
                net.graph.addEdge(j * N + i, j * N + i + 1, net.look.defaultWidth);
            if (j + 1 < N)
                net.graph.addEdge(j * N + i, (j + 1) * N + i, net.look.defaultWidth);
        }
    return net;
}

// The shipped vehicle catalogue (cars are CONTENT: a city with no recipes has no fleet).
std::string readFleetAsset(const std::string& name) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/" + name);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

// WHERE A CAR IS PUT DOWN (#172): every fleet slot, on its own handling, placed at PhysicsWorld::spawnHeight over flat
// ground, touches its tyres from the first step and settles onto its springs -- no fall, whatever its travel. The city
// spawn and the snap back onto a ghost used one 0.35 m drop for every car, a sedan's guess a bus or a truck does not
// share.
TEST_CASE(every_fleet_car_is_put_down_on_its_own_springs) {
    World world;
    testmap::addRoads(world, springsGrid());
    CityRenderParams params;
    params.cars = 4;
    params.pedestrians = 0;
    params.vehicleScript = readFleetAsset("vehicles.lua");
    CityRenderSystem city(params);
    CHECK(city.build(world, nullptr));
    const int slots = city.sim().fleetSize();
    CHECK(slots >= 14);
    for (int slot = 0; slot < slots; ++slot) {
        const PhysicsWorld::VehicleConfig* cfg = city.carConfig(slot);
        CHECK(cfg != nullptr);
        if (!cfg) continue;
        PhysicsWorld w;
        CHECK(w.initialize());
        w.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
        w.optimizeBroadPhase();
        const Real at = PhysicsWorld::spawnHeight(*cfg);
        const PhysicsWorld::VehicleId car = w.addVehicle(*cfg, Vec3(0, at, 0), Quat::identity());
        CHECK(car != PhysicsWorld::INVALID_VEHICLE);
        Real lowest = at, fastest = 0;
        for (int i = 0; i < 120; ++i) {
            w.update(1.0 / 60.0);
            lowest = std::min(lowest, w.vehiclePosition(car).y);
            fastest = std::max(fastest, std::fabs(w.vehicleVelocity(car).y));
        }
        const Real rest = w.vehiclePosition(car).y;
        std::printf("    [put down] slot %2d at %.3f m: rests %.3f, lowest %.3f, fastest fall %.2f m/s\n", slot, at, rest,
                    lowest, fastest);
        CHECK(at >= cfg->chassisHalfExtent.y);
        CHECK(at - rest < 0.12);       // settles onto its springs
        CHECK(at - rest > -0.01);      // and was not put down under them
        CHECK(fastest < 1.0);          // no fall (0.35 m is 2.6 m/s)
        CHECK(rest - lowest < 0.05);   // one gentle bounce at most
        w.shutdown();
    }
}
