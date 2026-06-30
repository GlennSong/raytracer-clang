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

// Total car instances across every variant group.
std::size_t carTotal(World& world, const CityRenderSystem& city) {
    std::size_t n = 0;
    for (Entity e : city.carGroups()) n += groupCount(world, e);
    return n;
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

    CHECK(carTotal(world, city) == 8u);     // 8 cars, spread across variant groups
    CHECK(groupCount(world, city.pedGroup()) == 6u);
    CHECK(city.carGroups().size() > 1u);    // there really are multiple variants
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

    // Gather every car's translation across all variant groups (group order +
    // agent order are stable, so index k tracks the same car each step).
    auto gather = [&]() {
        std::vector<Vec3> v;
        for (Entity e : city.carGroups()) {
            InstanceGroup* g = world.get<InstanceGroup>(e);
            if (!g) continue;
            for (const Mat4& t : g->transforms)
                v.push_back(Vec3(t.m[0][3], t.m[1][3], t.m[2][3]));
        }
        return v;
    };
    std::vector<Vec3> start = gather();
    CHECK(start.size() == 12u);

    bool anyMoved = false;
    for (int i = 0; i < 3000; ++i) {
        city.step(world, 0.1);
        std::vector<Vec3> now = gather();
        CHECK(now.size() == 12u);   // count stable across steps
        for (std::size_t k = 0; k < now.size(); ++k)
            if ((now[k] - start[k]).length() > 1.0) anyMoved = true;
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

// Distance from point p to segment [a,b] (XZ).
static double segDist(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab(b.x - a.x, b.y - a.y);
    double L2 = ab.x * ab.x + ab.y * ab.y;
    double t = L2 > 1e-9 ? ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / L2 : 0.0;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    double cx = a.x + ab.x * t, cy = a.y + ab.y * t;
    double dx = p.x - cx, dy = p.y - cy;
    return std::sqrt(dx * dx + dy * dy);
}

TEST_CASE(signal_poles_stand_outside_the_carriageway) {
    // A WIDE cross: a fixed pole setback would land inside the carriageway ("in
    // the middle of the road"). Each pole must sit beyond the kerb of every road.
    RoadNet net;
    net.nodes = { Vec2(0, 0), Vec2(80, 0), Vec2(-80, 0), Vec2(0, 80), Vec2(0, -80) };
    net.edges = { {0, 1}, {0, 2}, {0, 3}, {0, 4} };
    net.width = 16.0;   // half-width 8 — a fixed ~6.6 m setback would be INSIDE
    net.sidewalk = 2.5;

    World world;
    world.add<RoadNet>(world.create(), net);
    CityRenderSystem city;
    CHECK(city.build(world, nullptr));

    const NavGraph& nav = city.nav();
    Real halfW = 16.0 * 0.5;
    InstanceGroup* posts = world.get<InstanceGroup>(city.signalPostGroup());
    CHECK(posts != nullptr);
    CHECK(!posts->transforms.empty());
    for (const Mat4& t : posts->transforms) {
        Vec2 p(t.m[0][3], t.m[2][3]);
        double dmin = 1e9;
        for (const auto& link : nav.links)
            dmin = std::min(dmin, segDist(p, nav.nodes[link.from], nav.nodes[link.to]));
        CHECK(dmin > halfW * 0.9);   // clear of every carriageway (not in the road)
    }
}

TEST_CASE(crosswalks_sit_at_junction_mouths) {
    World world;
    world.add<RoadNet>(world.create(), crossRoads());   // 4-arm cross, width 10
    CityRenderSystem city;
    CHECK(city.build(world, nullptr));

    const NavGraph& nav = city.nav();
    // One crosswalk band per approach into the junction (4 arms here).
    const auto& centers = city.crosswalkCenters();
    CHECK(centers.size() == 4u);

    Real halfW = 10.0 * 0.5;
    for (const Vec2& c : centers) {
        // Each band sits just outside the junction node, across the road mouth —
        // beyond the carriageway edge but not far down the arm.
        double dNode = 1e9;
        for (const Vec2& n : nav.nodes) {
            double dx = c.x - n.x, dy = c.y - n.y;
            dNode = std::min(dNode, std::sqrt(dx * dx + dy * dy));
        }
        CHECK(dNode > halfW * 0.8);    // outside the intersection box...
        CHECK(dNode < halfW + 6.0);    // ...but right at the mouth, not mid-block
    }
    CHECK(world.get<InstanceGroup>(city.crosswalkGroup()) != nullptr);
}
