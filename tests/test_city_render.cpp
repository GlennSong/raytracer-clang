#include "test_framework.h"

#include "../src/apps/citysim/city_render.h"
#include "../src/engine/components.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/world.h"

using namespace engine;
using namespace citysim;

namespace {

// A square loop of road: every node routes to every other (degree-2 corners, so
// no junction signals — good for the plain car/ped bake tests).
RoadNet squareLoop() {
    RoadNet net;
    net.nodes = { Vec2(0, 0), Vec2(40, 0), Vec2(40, 40), Vec2(0, 40) };
    net.edges = { {0, 1}, {1, 2}, {2, 3}, {3, 0} };
    net.width = 10.0;
    net.sidewalk = 2.5;
    return net;
}

// A plus/cross: the centre node has degree 4, so it becomes a signalled junction.
RoadNet crossRoads() {
    RoadNet net;
    net.nodes = { Vec2(0, 0), Vec2(60, 0), Vec2(-60, 0), Vec2(0, 60), Vec2(0, -60) };
    net.edges = { {0, 1}, {0, 2}, {0, 3}, {0, 4} };
    net.width = 10.0;
    net.sidewalk = 2.5;
    return net;
}

std::size_t groupCount(World& world, Entity e) {
    InstanceGroup* g = world.get<InstanceGroup>(e);
    return g ? g->transforms.size() : 0;
}

}  // namespace

TEST_CASE(city_render_builds_from_roadnet) {
    World world;
    world.add<RoadNet>(world.create(), squareLoop());

    CityRenderParams p;
    p.cars = 8;
    p.pedestrians = 6;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));   // null AssetManager: transforms only
    CHECK(city.built());
    CHECK(city.nav().linkCount() > 0);
    CHECK(city.sim().agents().size() == 14u);

    CHECK(groupCount(world, city.carGroup()) == 8u);
    CHECK(groupCount(world, city.pedGroup()) == 6u);
}

TEST_CASE(city_render_build_fails_without_roads) {
    World world;
    CityRenderSystem city;
    CHECK(!city.build(world, nullptr));
    CHECK(!city.built());
}

TEST_CASE(city_render_agents_move_when_stepped) {
    World world;
    world.add<RoadNet>(world.create(), squareLoop());

    CityRenderParams p;
    p.cars = 12;
    p.pedestrians = 0;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));

    InstanceGroup* cars = world.get<InstanceGroup>(city.carGroup());
    CHECK(cars != nullptr);
    std::vector<Vec3> start;
    for (const Mat4& t : cars->transforms)
        start.push_back(Vec3(t.m[0][3], t.m[1][3], t.m[2][3]));

    bool anyMoved = false;
    for (int i = 0; i < 3000; ++i) {
        city.step(world, 0.1);
        cars = world.get<InstanceGroup>(city.carGroup());
        CHECK(cars->transforms.size() == 12u);   // count stable across steps
        for (std::size_t k = 0; k < cars->transforms.size(); ++k) {
            Vec3 now(cars->transforms[k].m[0][3], cars->transforms[k].m[1][3],
                     cars->transforms[k].m[2][3]);
            if ((now - start[k]).length() > 1.0) anyMoved = true;
        }
    }
    CHECK(anyMoved);
}

TEST_CASE(city_render_signals_light_up_and_change_state) {
    World world;
    world.add<RoadNet>(world.create(), crossRoads());

    CityRenderParams p;
    p.cars = 6;
    p.pedestrians = 0;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));

    auto sigCount = [&](SignalState s) {
        return groupCount(world, city.signalGroup(s));
    };

    // The signalled junction puts at least one lens in some state group, and the
    // total number of lenses is conserved across phases (each approach is in
    // exactly one state group at a time).
    std::size_t total0 = sigCount(SignalState::Green) + sigCount(SignalState::Yellow) +
                         sigCount(SignalState::Red);
    CHECK(total0 > 0);
    // One static signal-head assembly per signalled approach; the lit lenses
    // (across the three state groups) always sum to that same count.
    CHECK(groupCount(world, city.signalPostGroup()) == total0);

    bool sawGreen = false, sawRed = false;
    bool totalStable = true;
    for (int i = 0; i < 2000; ++i) {
        city.step(world, 0.1);
        if (sigCount(SignalState::Green) > 0) sawGreen = true;
        if (sigCount(SignalState::Red) > 0) sawRed = true;
        std::size_t total = sigCount(SignalState::Green) + sigCount(SignalState::Yellow) +
                            sigCount(SignalState::Red);
        if (total != total0) totalStable = false;
    }
    CHECK(sawGreen);        // the lens glows green on a green phase...
    CHECK(sawRed);          // ...and red on a red phase (it changes state)
    CHECK(totalStable);     // and every approach is always represented exactly once
}
