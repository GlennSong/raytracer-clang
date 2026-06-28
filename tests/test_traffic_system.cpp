#include "test_framework.h"

#include "../src/engine/systems/traffic_system.h"
#include "../src/engine/components.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/world.h"

using namespace engine;

namespace {

// A square loop of road: four corners joined into a cycle, so every node can
// route to every other (a real commute exists between any home/work pair).
RoadNet squareLoop() {
    RoadNet net;
    net.nodes = { Vec2(0, 0), Vec2(40, 0), Vec2(40, 40), Vec2(0, 40) };
    net.edges = { {0, 1}, {1, 2}, {2, 3}, {3, 0} };
    net.width = 10.0;
    net.sidewalk = 2.5;
    return net;
}

// Count InstanceGroup transforms on an entity (0 if it has no group).
std::size_t groupCount(World& world, Entity e) {
    InstanceGroup* g = world.get<InstanceGroup>(e);
    return g ? g->transforms.size() : 0;
}

}  // namespace

TEST_CASE(traffic_builds_from_roadnet) {
    World world;
    world.add<RoadNet>(world.create(), squareLoop());

    TrafficParams p;
    p.cars = 8;
    p.pedestrians = 6;
    TrafficSystem traffic(p);
    CHECK(traffic.build(world, nullptr));
    CHECK(traffic.built());
    CHECK(traffic.nav().linkCount() > 0);
    CHECK(traffic.sim().agents().size() == 14u);

    // One InstanceGroup per kind, sized to its agent count.
    CHECK(groupCount(world, traffic.carGroup()) == 8u);
    CHECK(groupCount(world, traffic.pedGroup()) == 6u);
}

TEST_CASE(traffic_build_fails_without_roads) {
    World world;
    TrafficSystem traffic;
    CHECK(!traffic.build(world, nullptr));
    CHECK(!traffic.built());
}

TEST_CASE(traffic_agents_move_when_stepped) {
    World world;
    world.add<RoadNet>(world.create(), squareLoop());

    TrafficParams p;
    p.cars = 12;
    p.pedestrians = 0;
    TrafficSystem traffic(p);
    CHECK(traffic.build(world, nullptr));

    // Snapshot initial car positions (translation column of each instance).
    InstanceGroup* cars = world.get<InstanceGroup>(traffic.carGroup());
    CHECK(cars != nullptr);
    std::vector<Vec3> start;
    for (const Mat4& t : cars->transforms)
        start.push_back(Vec3(t.m[0][3], t.m[1][3], t.m[2][3]));

    // Step through the morning commute window, watching for motion AS IT HAPPENS:
    // agents leave home, drive to work, then return — so a final-vs-initial check
    // would miss them back home. Sample displacement every step instead.
    bool anyMoved = false;
    for (int i = 0; i < 3000; ++i) {
        traffic.step(world, 0.1);
        cars = world.get<InstanceGroup>(traffic.carGroup());
        CHECK(cars->transforms.size() == 12u);   // count is stable across steps
        for (std::size_t k = 0; k < cars->transforms.size(); ++k) {
            Vec3 now(cars->transforms[k].m[0][3], cars->transforms[k].m[1][3],
                     cars->transforms[k].m[2][3]);
            if ((now - start[k]).length() > 1.0) anyMoved = true;
        }
    }
    CHECK(anyMoved);
}

TEST_CASE(traffic_agents_stay_near_roads) {
    World world;
    world.add<RoadNet>(world.create(), squareLoop());

    TrafficSystem traffic;   // defaults
    CHECK(traffic.build(world, nullptr));
    for (int i = 0; i < 2000; ++i) traffic.step(world, 0.1);

    // Every instance must sit within the road network's XZ bounds plus a margin
    // (lane + sidewalk offset), i.e. nobody wanders off into the void.
    auto within = [](World& w, Entity e) {
        InstanceGroup* g = w.get<InstanceGroup>(e);
        if (!g) return true;
        for (const Mat4& t : g->transforms) {
            Real x = t.m[0][3], z = t.m[2][3];
            if (x < -15 || x > 55 || z < -15 || z > 55) return false;
        }
        return true;
    };
    CHECK(within(world, traffic.carGroup()));
    CHECK(within(world, traffic.pedGroup()));
}
