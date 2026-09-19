#include "test_framework.h"
#include "../src/apps/citysim/city_map_tool.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/camera/fly_camera_controller.h"
#include "../src/engine/components.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/world.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace engine;
using namespace citysim;

// THE MAP TOOL'S FRAME (Glenn, 2026-09-19: "We should be able to see where we
// are, which direction we are facing, bus stops and bus lines"). The picture
// itself is CityMapRaster's (tested against metro in level_tests); this pins
// what the tool lays over it: the you-are-here chevron at your position,
// pointing the way you look (and pinned to the edge when you are off the
// panel), a dot per stop in view, a square per bus.

namespace {

RoadEntity mapGrid() {
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

bool sameColour(const Renderer::UiQuad& q, const float* c) {
    return std::fabs(q.r - c[0]) < 1e-4f && std::fabs(q.g - c[1]) < 1e-4f &&
           std::fabs(q.b - c[2]) < 1e-4f && std::fabs(q.a - c[3]) < 1e-4f;
}

}  // namespace

TEST_CASE(city_map_tool_draws_you_facing_the_way_you_look) {
    World world;
    world.add<RoadEntity>(world.create(), mapGrid());
    CityRenderParams params;
    params.cars = 12;
    params.pedestrians = 0;
    params.seed = 7;
    params.wander = true;
    params.busRoutes = 1;
    params.busStops = 8;
    params.buses = 2;
    params.vehicleScript = readAsset("vehicles.lua");
    CityRenderSystem city(params);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    CHECK(city.build(world, &assets));

    Entity player = world.create();
    Transform pt;
    pt.position = Vec3(180, 1.1, 180);
    world.add<Transform>(player, pt);
    world.add<ControlledBy>(player, ControlledBy{});

    FlyCameraController fly;
    CityMapToolSystem map(city, fly);
    const int W = 1600, H = 900;
    const auto p = CityMapToolSystem::panel(W, H);
    const float pcx = (p.x0 + p.x1) * 0.5f, pcy = (p.y0 + p.y1) * 0.5f;

    // The chevron: two quads in the you colour/white; the coloured one's
    // centre is where you stand, and its top edge's midpoint is ahead of it.
    auto you = [&](float& cx, float& cy, float& fx, float& fy) {
        const auto quads = map.composeFrame(world, W, H);
        for (const auto& q : quads) {
            if (!sameColour(q, CityMapToolSystem::youColour())) continue;
            cx = (q.x[0] + q.x[1] + q.x[2] + q.x[3]) * 0.25f;
            cy = (q.y[0] + q.y[1] + q.y[2] + q.y[3]) * 0.25f;
            const float tx = (q.x[0] + q.x[1]) * 0.5f - cx, ty = (q.y[0] + q.y[1]) * 0.5f - cy;
            const float l = std::sqrt(tx * tx + ty * ty);
            fx = tx / l;
            fy = ty / l;
            return true;
        }
        return false;
    };
    float cx, cy, fx, fy;

    // Centred on you, facing -Z (yaw 0): the chevron is mid-panel, pointing UP.
    map.showAt(180, 180, 1.0);
    fly.yaw = 0;
    CHECK(you(cx, cy, fx, fy));
    CHECK(std::fabs(cx - pcx) < 0.5f && std::fabs(cy - pcy) < 0.5f);
    CHECK(fy < -0.99f);
    // Facing +X (yaw 90): it points RIGHT. 30 m east of you on the map is 30 px.
    fly.yaw = 90;
    map.showAt(150, 180, 1.0);
    CHECK(you(cx, cy, fx, fy));
    CHECK(std::fabs(cx - (pcx + 30.0f)) < 0.5f && std::fabs(cy - pcy) < 0.5f);
    CHECK(fx > 0.99f);
    // Far off the panel: pinned to its edge (still pointing), not gone.
    map.showAt(-5000, 180, 1.0);
    CHECK(you(cx, cy, fx, fy));
    CHECK(cx <= p.x1 && cx >= p.x1 - 20.0f);

    // Stops and buses: at a zoom that shows the whole grid, a dot per stop
    // and a square per bus inside the panel.
    map.showAt(180, 180, 0.6);
    const auto quads = map.composeFrame(world, W, H);
    int inside = 0;
    for (const auto& q : quads) {
        const float mx = (q.x[0] + q.x[2]) * 0.5f, my = (q.y[0] + q.y[2]) * 0.5f;
        if (mx >= p.x0 && mx <= p.x1 && my >= p.y0 && my <= p.y1) ++inside;
    }
    int stops = 0;
    for (int r = 0; r < city.sim().buses().routeCount(); ++r)
        stops += static_cast<int>(city.sim().buses().route(r).stops.size());
    std::printf("    [map] %zu quads (%d inside the panel); %d stops, %d buses\n", quads.size(),
                inside, stops, params.buses);
    CHECK(stops > 0);
    // frame + paper + >= 2 per stop + 2 per bus + 2 for you + legend
    CHECK(static_cast<int>(quads.size()) >= 2 + 2 * stops + 2 * params.buses + 2);
}
