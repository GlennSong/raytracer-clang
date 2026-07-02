#include "test_framework.h"

#include "../src/apps/citysim/city_render.h"
#include "../src/engine/components.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/world.h"

#include <algorithm>
#include <cmath>
#include <vector>

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

TEST_CASE(city_render_debug_widgets) {
    World world;
    world.add<RoadNet>(world.create(), squareLoop());

    CityRenderParams p;
    p.cars = 5;
    p.pedestrians = 5;
    p.debugWidgets = true;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));

    // Step so agents move (footprints + forward arrows populate). Track the arrow
    // count over time — arrows exist only while an agent is actually moving.
    std::size_t maxArrows = 0;
    for (int i = 0; i < 3000; ++i) {
        city.step(world, 0.1);
        std::size_t arrows = groupCount(world, city.forwardGroup());
        maxArrows = std::max(maxArrows, arrows);
        CHECK(arrows <= 10u);   // never more than the agent count
    }
    // One footprint per agent, spread across the behaviour-state groups (peds use
    // the first four; drivers use the FSM states after them).
    std::size_t foot = 0;
    for (int s = 0; s < static_cast<int>(Agent::State::Count); ++s)
        foot += groupCount(world, city.footprintGroup(static_cast<Agent::State>(s)));
    CHECK(foot == 10u);
    CHECK(maxArrows >= 1u);     // agents did move at some point, drawing a trajectory
}

TEST_CASE(debug_widgets_ring_the_real_car_when_cars_are_external) {
    // ADR-0061: with cars owned by the vehicle bridge, a driver's debug ring must
    // circle the PHYSICAL car (the bridge-reported pose), not the planner ghost —
    // a ring around the ghost is an empty circle on the ground. Drivers with no
    // reported car draw no ring; pedestrian rings are unaffected.
    World world;
    world.add<RoadNet>(world.create(), squareLoop());
    CityRenderParams p;
    p.cars = 4;
    p.pedestrians = 3;
    p.debugWidgets = true;
    CityRenderSystem city(p);
    city.setCarsExternallyOwned(true);
    CHECK(city.build(world, nullptr));

    // Report a real pose for agent 0 only, far from any ghost.
    const Vec2 realPos(500.0, -500.0);
    city.setExternalCarPoses({ CityRenderSystem::ExternalAgentPose{0, realPos, Vec2(0, 1)} });
    city.step(world, 0.1);

    std::size_t rings = 0;
    bool ringAtReal = false;
    for (int s = 0; s < static_cast<int>(Agent::State::Count); ++s) {
        InstanceGroup* g = world.get<InstanceGroup>(
            city.footprintGroup(static_cast<Agent::State>(s)));
        if (!g) continue;
        for (const Mat4& m : g->transforms) {
            ++rings;
            if (std::fabs(m.m[0][3] - realPos.x) < 1e-6 &&
                std::fabs(m.m[2][3] - realPos.y) < 1e-6)
                ringAtReal = true;
        }
    }
    CHECK(rings == 4u);      // 3 ped ghosts + the 1 reported car; 3 unreported drivers skip
    CHECK(ringAtReal);       // and the car's ring sits at the REAL pose
}

TEST_CASE(external_peds_stop_the_instanced_bake_and_ring_real_walkers) {
    // ADR-0061 walkers: with peds owned by CityWalkerSystem, the render bridge
    // must not bake instanced ped boxes (they'd draw twice), and a ped's debug
    // ring follows the bridge-reported REAL body — unreported walkers draw none.
    World world;
    world.add<RoadNet>(world.create(), squareLoop());
    CityRenderParams p;
    p.cars = 2;
    p.pedestrians = 3;
    p.debugWidgets = true;
    CityRenderSystem city(p);
    city.setCarsExternallyOwned(true);
    city.setPedsExternallyOwned(true);
    CHECK(city.build(world, nullptr));

    const Vec2 realPed(321.0, 654.0);
    // Ped agents follow the drivers in the agent array: ids 2..4 here.
    city.setExternalPedPoses({ CityRenderSystem::ExternalAgentPose{2, realPed, Vec2(1, 0), realPed} });
    city.step(world, 0.1);

    CHECK(groupCount(world, city.pedGroup()) == 0u);   // no instanced ghost peds

    std::size_t rings = 0;
    bool ringAtReal = false;
    for (int s = 0; s < static_cast<int>(Agent::State::Count); ++s) {
        InstanceGroup* g = world.get<InstanceGroup>(
            city.footprintGroup(static_cast<Agent::State>(s)));
        if (!g) continue;
        for (const Mat4& m : g->transforms) {
            ++rings;
            if (std::fabs(m.m[0][3] - realPed.x) < 1e-6 &&
                std::fabs(m.m[2][3] - realPed.y) < 1e-6)
                ringAtReal = true;
        }
    }
    CHECK(rings == 1u);   // ONLY the reported walker (no cars/peds reported besides it)
    CHECK(ringAtReal);
}

TEST_CASE(city_render_car_colliders_scale_with_the_fleet) {
    World world;
    world.add<RoadNet>(world.create(), squareLoop());
    CityRenderSystem city;
    CHECK(city.build(world, nullptr));

    // One collider extent per car group, and they follow the shared fleet: a group
    // is a body slot, so a box-truck slot's box is bigger than a sedan slot's.
    std::vector<Vec3> ext = city.carGroupHalfExtents();
    CHECK(ext.size() == city.carGroups().size());
    Real sedanZ = 0, maxZ = 0;
    for (std::size_t v = 0; v < ext.size(); ++v) {
        CHECK(ext[v].x > 0 && ext[v].y > 0 && ext[v].z > 0);
        // Half-extent mirrors the fleet body's half-length.
        CHECK(std::fabs(ext[v].z - vehicleFleetBody(static_cast<int>(v)).length * 0.5) < 1e-6);
        if (vehicleFleetBody(static_cast<int>(v)).type == VehicleType::Sedan) sedanZ = ext[v].z;
        maxZ = std::max(maxZ, ext[v].z);
    }
    CHECK(maxZ > sedanZ);   // a bigger body has a bigger collider
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
    // The bars are painted into the ROAD TEXTURE now (ADR-0061), not a decal group:
    // the group still exists (a stable anchor) but carries no overlay instances.
    InstanceGroup* g = world.get<InstanceGroup>(city.crosswalkGroup());
    CHECK(g != nullptr);
    CHECK(g->transforms.empty());
}
