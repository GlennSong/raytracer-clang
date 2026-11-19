// A BUS'S OWN DRAWN GLIDE (Glenn, 2026-09-18: "when I'm in the bus there's a lot of crazy jittering"). The player's
// passenger seat on a bus or cab (CityPlayerTransitSystem) was deleted with the old driving (ADR-0459, #306 piece C1);
// buses go with piece C2, and this case with them.
#include "test_framework.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/world.h"
#include "street_map_test_util.h"   // a test world's street map, from its road entities (ADR-0425)
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

}  // namespace

// HOW SMOOTH IS THE BUS ITSELF (Glenn, 2026-09-18: "there's still a lot of
// jitter from the bus movement"). Any unevenness in the bus's own drawn motion
// is the whole world shaking for someone aboard. At
// metro's sim rate (localHz 30: a tick every other fixed step), on the drawn
// matrix, per fixed step: jerk |p[k+1] - 2p[k] + p[k-1]| (an even glide is ~0)
// and the jump in yaw rate. Measured before: jerk 0.0127 m mean / 0.19 max, yaw
// jump 0.013 rad -- from the pull-out easing advancing in 30 Hz chunks (and on
// alternate ticks only: the stop-line approach skipped its counter), and a bus
// arriving at its stop at 5.6 m/s and standing still the next tick.
TEST_CASE(a_bus_glides_at_the_city_s_sim_rate) {
    World world;
    testmap::addRoads(world, rideGrid());
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
    auto wrap = [](Real d) { return wrapRadians(d); };
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
