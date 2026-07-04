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
    // ADR-0062: with cars owned by the vehicle bridge, a driver's debug ring must
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
    // ADR-0062 walkers: with peds owned by CityWalkerSystem, the render bridge
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

    // Body-truth override: reporting the walker as Waiting (blocked/knocked) puts
    // its ring in the RED group regardless of what the ghost thinks it's doing.
    city.setExternalPedPoses({ CityRenderSystem::ExternalAgentPose{
        2, realPed, Vec2(1, 0), realPed, static_cast<int>(Agent::State::Waiting)} });
    city.step(world, 0.1);
    InstanceGroup* red = world.get<InstanceGroup>(city.footprintGroup(Agent::State::Waiting));
    bool redAtReal = false;
    if (red)
        for (const Mat4& m : red->transforms)
            if (std::fabs(m.m[0][3] - realPed.x) < 1e-6 &&
                std::fabs(m.m[2][3] - realPed.y) < 1e-6)
                redAtReal = true;
    CHECK(redAtReal);
}

TEST_CASE(commandeered_car_leaves_the_instanced_fleet) {
    // Promotion (ADR-0062 rethink): when the player commandeers an ambient car,
    // the sim releases its agent and the instanced fleet must drop that car (the
    // real Vehicle takes its visual place). Widgets drop it too.
    World world;
    world.add<RoadNet>(world.create(), squareLoop());
    CityRenderParams p;
    p.cars = 5;
    p.pedestrians = 0;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));
    city.step(world, 0.1);
    CHECK(carTotal(world, city) == 5u);

    city.simMutable().releaseDriver(2);
    city.step(world, 0.1);
    CHECK(carTotal(world, city) == 4u);   // the commandeered car vanished
}

TEST_CASE(fleet_mesh_wheelless_variant_drops_the_wheels) {
    // A promoted physical car renders VehicleSystem's physics wheels; its body
    // mesh must not bake a second set (the "double wheels" device bug).
    RenderMesh with = fleetCarMesh(0, true);
    RenderMesh without = fleetCarMesh(0, false);
    CHECK(without.vertices.size() < with.vertices.size());
    CHECK(!without.vertices.empty());
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

// --- car lamps (ADR-0065 follow-up) ------------------------------------------
// The lamp DECISION logic (carLampState) is unit-tested headless in
// test_car_lamps.cpp; these exercise the instanced RENDER BAKE — that the right
// emissive lamp transforms appear on drawn cars — which is otherwise viewer-gated.
// The bake is deterministic (sim clock drives even the turn-signal blink), so
// these are stable, not flaky.

TEST_CASE(car_headlights_track_the_night_clock) {
    World world;
    world.add<RoadNet>(world.create(), squareLoop());
    CityRenderParams p;
    p.cars = 6;
    p.pedestrians = 0;
    p.hoursPerSecond = 3.0;   // sweep a full day in a few hundred steps
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));

    bool nightLit = false, dayDark = false;
    for (int i = 0; i < 2000; ++i) {
        city.step(world, 0.1);
        Real t = city.sim().timeOfDay();
        std::size_t heads = groupCount(world, city.headlightGroup());
        std::size_t cars = carTotal(world, city);
        if (t > 20.0 || t < 5.0) {          // deep night: BOTH headlights per car
            if (cars > 0 && heads == cars * 2) nightLit = true;
        } else if (t > 9.0 && t < 16.0) {   // midday: none
            if (heads == 0) dayDark = true;
        }
    }
    CHECK(nightLit);   // every drawn car lights two headlights at night
    CHECK(dayDark);    // and none by day
}

TEST_CASE(car_brake_lights_come_on_when_holding) {
    World world;
    world.add<RoadNet>(world.create(), crossRoads());   // signalled junction
    CityRenderParams p;
    p.cars = 8;
    p.pedestrians = 0;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));

    bool sawBrake = false;
    for (int i = 0; i < 3000; ++i) {
        city.step(world, 0.1);
        if (groupCount(world, city.brakeLightGroup()) > 0) sawBrake = true;
    }
    CHECK(sawBrake);   // cars hold at the red / yield and light their brakes
}

TEST_CASE(car_turn_signals_flash_on_turns) {
    World world;
    world.add<RoadNet>(world.create(), crossRoads());
    CityRenderParams p;
    p.cars = 8;
    p.pedestrians = 0;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));

    bool sawTurn = false;
    for (int i = 0; i < 4000; ++i) {
        city.step(world, 0.1);
        // A turning car's indicator flashes (sim-clock blink), so over many steps
        // a Turning-state car coincides with a blink-on window.
        if (groupCount(world, city.turnSignalGroup()) > 0) sawTurn = true;
    }
    CHECK(sawTurn);
}

TEST_CASE(externally_owned_cars_draw_no_lamps) {
    // ADR-0062: a car owned by the vehicle bridge isn't drawn here, so it lights no
    // lamps here either (its real Vehicle owns them) — even at night.
    World world;
    world.add<RoadNet>(world.create(), squareLoop());
    CityRenderParams p;
    p.cars = 5;
    p.pedestrians = 0;
    p.hoursPerSecond = 3.0;   // pass through night too
    CityRenderSystem city(p);
    city.setCarsExternallyOwned(true);
    CHECK(city.build(world, nullptr));

    bool everLit = false;
    for (int i = 0; i < 500; ++i) {
        city.step(world, 0.1);
        if (groupCount(world, city.headlightGroup()) > 0 ||
            groupCount(world, city.brakeLightGroup()) > 0 ||
            groupCount(world, city.turnSignalGroup()) > 0)
            everLit = true;
    }
    CHECK(!everLit);   // no instanced lamps while the cars are externally owned
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
    // The bars are painted into the ROAD TEXTURE now (ADR-0062), not a decal group:
    // the group still exists (a stable anchor) but carries no overlay instances.
    InstanceGroup* g = world.get<InstanceGroup>(city.crosswalkGroup());
    CHECK(g != nullptr);
    CHECK(g->transforms.empty());
}

TEST_CASE(level_citysim_config_overrides_build_params) {
    // ADR-0063: a level's top-level "citysim" block (parsed into a CitySimConfig
    // entity) decides the population — the agent lab runs 1 car + 1 walker while
    // grown.json keeps the default bustle. The entity must beat the constructor.
    World world;
    world.add<RoadNet>(world.create(), squareLoop());
    CitySimConfig cfg;
    cfg.cars = 1;
    cfg.pedestrians = 1;
    cfg.seed = 7;
    cfg.debugWidgets = true;
    world.add<CitySimConfig>(world.create(), cfg);

    CityRenderParams p;          // the constructor asks for a crowd...
    p.cars = 40;
    p.pedestrians = 40;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));
    CHECK(city.sim().agents().size() == 2u);   // ...but the LEVEL's block wins
    CHECK(carTotal(world, city) == 1u);
    CHECK(groupCount(world, city.pedGroup()) == 1u);
}

TEST_CASE(agent_lab_theta_circuit_is_navigable) {
    // The agent-lab level (assets/levels/agent_lab.json) is a theta circuit: a
    // 120x80 ring with a middle bar, meeting at two 3-way junctions. Mirror its
    // geometry here and pin what the lab depends on: both junctions carry
    // signals, every node routes to every other, and a lone driver + walker
    // actually get moving on it.
    RoadNet net;
    net.nodes = { Vec2(-60, -40), Vec2(60, -40), Vec2(60, 40), Vec2(-60, 40),
                  Vec2(-60, 0), Vec2(60, 0) };
    net.edges = { {0, 1}, {1, 5}, {5, 2}, {2, 3}, {3, 4}, {4, 0}, {4, 5} };
    net.width = 7.0;
    net.sidewalk = 1.8;

    World world;
    world.add<RoadNet>(world.create(), net);
    CitySimConfig cfg;
    cfg.cars = 1;
    cfg.pedestrians = 1;
    cfg.seed = 7;
    cfg.wander = true;   // the lab keeps its agents perpetually on the move
    world.add<CitySimConfig>(world.create(), cfg);
    CityRenderSystem city;
    CHECK(city.build(world, nullptr));
    CHECK(city.sim().wander());

    const NavGraph& nav = city.nav();
    int junctions = 0;
    for (int v = 0; v < nav.nodeCount(); ++v)
        if (nav.isJunction(v)) ++junctions;
    CHECK(junctions == 2);   // the two theta mid-side meets; corners just bend

    // Fully connected: every ordered pair routes.
    bool allRoute = true;
    for (int a = 0; a < nav.nodeCount(); ++a)
        for (int b = 0; b < nav.nodeCount(); ++b)
            if (a != b && !findRoute(nav, a, b).valid()) allRoute = false;
    CHECK(allRoute);

    // Both lab agents come alive: the driver rolls and the walker walks.
    bool carMoved = false, pedMoved = false;
    for (int i = 0; i < 6000; ++i) {
        city.step(world, 0.1);
        for (const Agent& a : city.sim().agents()) {
            if (!a.moving || a.speed <= 0.5) continue;
            (a.mode == Agent::Mode::Driver ? carMoved : pedMoved) = true;
        }
    }
    CHECK(carMoved);
    CHECK(pedMoved);

    // Wander keeps the lab ALIVE: the driver strings together trip after trip
    // instead of parking until the evening commute (device round 3 — "I would
    // expect the car to be running around the track").
    int carTrips = 0;
    for (const Agent& a : city.sim().agents())
        if (a.mode == Agent::Mode::Driver) carTrips = a.trips;
    CHECK(carTrips >= 3);
}

TEST_CASE(wedge_mesh_is_a_flat_outline_cone) {
    // The vision-cone wedge (city_meshes): a ground-facing OUTLINE — two straight
    // edges from the origin plus the arc — of unit radius, built from thin quads
    // like ringXZ so it draws with the same widgetMaterial ground-paint technique.
    RenderMesh w = wedgeXZ(0.45);
    CHECK(!w.vertices.empty());
    CHECK(w.vertices.size() == (2u + 12u) * 4u);   // 2 edge quads + 12 arc quads
    CHECK(w.indices.size() == (2u + 12u) * 6u);
    Real maxR = 0, minR = 1e9;
    bool flat = true;
    for (const Vertex& v : w.vertices) {
        if (std::fabs(v.position.y) > 1e-9) flat = false;
        Real r = std::sqrt(v.position.x * v.position.x + v.position.z * v.position.z);
        maxR = std::max(maxR, r);
        minR = std::min(minR, r);
    }
    CHECK(flat);                          // all y == 0: lies in the ground plane
    CHECK(std::fabs(maxR - 1.0) < 0.05);  // rim (arc + edge tips) at unit radius
    CHECK(minR < 0.05);                   // edges really start at the origin
    // The half-angle is baked into the mesh, so cones of different widths are
    // different geometry (a walker's 1.2 rad wedge vs a driver's 0.45 rad).
    RenderMesh wide = wedgeXZ(1.2);
    CHECK(wide.vertices.size() == w.vertices.size());
    bool differs = false;
    for (std::size_t i = 0; i < w.vertices.size(); ++i)
        if ((wide.vertices[i].position - w.vertices[i].position).length() > 0.01)
            differs = true;
    CHECK(differs);
}

TEST_CASE(debug_navgraph_and_vision_cones_draw_with_the_hud) {
    // The `j` HUD's two new views: the NAVGRAPH (lane-centreline strips + a ring
    // per junction node, baked once at build) and the VISION CONES (one wedge per
    // moving agent, in its mode's group).
    World world;
    world.add<RoadNet>(world.create(), crossRoads());
    CityRenderParams p;
    p.cars = 4;
    p.pedestrians = 3;
    p.debugWidgets = true;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));

    // One strip per lane per link, lifted just off the road like the rings.
    const NavGraph& nav = city.nav();
    std::size_t lanes = 0;
    for (const NavLink& l : nav.links)
        lanes += static_cast<std::size_t>(l.lanes < 1 ? 1 : l.lanes);
    CHECK(groupCount(world, city.navLinkGroup()) == lanes);
    InstanceGroup* strips = world.get<InstanceGroup>(city.navLinkGroup());
    CHECK(strips != nullptr);
    for (const Mat4& m : strips->transforms)
        CHECK(std::fabs(m.m[1][3] - (0.3 + 0.04)) < 1e-6);   // road lift (0.3) + 0.04
    // One ring per junction node (the cross has exactly one).
    int junctions = 0;
    for (int n = 0; n < nav.nodeCount(); ++n)
        if (nav.isJunction(n)) ++junctions;
    CHECK(junctions == 1);
    CHECK(groupCount(world, city.navNodeGroup()) == static_cast<std::size_t>(junctions));

    // Cones: exactly one per MOVING agent every step, split by mode; the
    // navgraph stays put (static bake, not rebaked per frame).
    bool sawCarCone = false, sawPedCone = false;
    for (int i = 0; i < 3000; ++i) {
        city.step(world, 0.1);
        std::size_t movingCars = 0, movingPeds = 0;
        for (const Agent& a : city.sim().agents()) {
            if (!a.moving) continue;
            (a.mode == Agent::Mode::Driver ? movingCars : movingPeds) += 1;
        }
        CHECK(groupCount(world, city.visionGroup(Agent::Mode::Driver)) == movingCars);
        CHECK(groupCount(world, city.visionGroup(Agent::Mode::Pedestrian)) == movingPeds);
        CHECK(groupCount(world, city.navLinkGroup()) == lanes);
        if (movingCars > 0) sawCarCone = true;
        if (movingPeds > 0) sawPedCone = true;
    }
    CHECK(sawCarCone);   // agents really moved, so both views really drew
    CHECK(sawPedCone);
}

TEST_CASE(debug_navgraph_and_cones_vanish_when_the_hud_is_off) {
    World world;
    world.add<RoadNet>(world.create(), crossRoads());
    CityRenderParams p;
    p.cars = 4;
    p.pedestrians = 3;
    p.debugWidgets = false;   // HUD off: every debug group must stay empty
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));
    for (int i = 0; i < 500; ++i) city.step(world, 0.1);
    CHECK(groupCount(world, city.navLinkGroup()) == 0u);
    CHECK(groupCount(world, city.navNodeGroup()) == 0u);
    CHECK(groupCount(world, city.visionGroup(Agent::Mode::Driver)) == 0u);
    CHECK(groupCount(world, city.visionGroup(Agent::Mode::Pedestrian)) == 0u);
}

TEST_CASE(person_mesh_stands_and_swings) {
    // The simple articulated person (device ask: "people should look like
    // people"): six vertex-coloured boxes, 1.8 m tall, centred at mid-height
    // like the old walker box. A swing pose moves the limbs but never changes
    // topology — the walk cycle hops between shared pose meshes.
    RenderMesh stand = buildPersonMesh(0.0, 0);
    RenderMesh step = buildPersonMesh(0.5, 0);
    CHECK(stand.vertices.size() == 6u * 24u);   // head/torso/2 legs/2 arms
    CHECK(step.vertices.size() == stand.vertices.size());
    CHECK(step.indices.size() == stand.indices.size());

    Real lo = 1e9, hi = -1e9;
    bool moved = false;
    for (std::size_t i = 0; i < stand.vertices.size(); ++i) {
        lo = std::min(lo, stand.vertices[i].position.y);
        hi = std::max(hi, stand.vertices[i].position.y);
        if ((stand.vertices[i].position - step.vertices[i].position).length() > 0.05)
            moved = true;
    }
    CHECK(lo >= -0.95);
    CHECK(lo <= -0.85);      // feet at the bottom of the old box envelope...
    CHECK(hi <= 0.90);       // ...head inside the top
    CHECK(moved);            // the pose really articulates
    // Outfits differ (deterministic palette) and wrap safely.
    CHECK(personOutfitCount() > 4);
    RenderMesh other = buildPersonMesh(0.0, 1);
    bool recolored = false;
    for (std::size_t i = 0; i < stand.vertices.size(); ++i)
        if ((stand.vertices[i].color - other.vertices[i].color).length() > 0.01)
            recolored = true;
    CHECK(recolored);
}

TEST_CASE(player_outfit_is_reserved_and_distinct) {
    // The player IS a person: a fixed outfit slot past the walker-dealable
    // range (walkers deal `hash % personOutfitCount()`), coloured unlike every
    // walker outfit so the player reads instantly in a crowd.
    CHECK(playerOutfit() >= personOutfitCount());
    RenderMesh body = buildPersonMesh(0.0, playerOutfit());
    for (int i = 0; i < personOutfitCount(); ++i) {
        RenderMesh walker = buildPersonMesh(0.0, i);
        bool differs = false;
        for (std::size_t v = 0; v < body.vertices.size(); ++v)
            if ((body.vertices[v].color - walker.vertices[v].color).length() > 0.01)
                differs = true;
        CHECK(differs);
    }
}

TEST_CASE(walk_pose_index_tracks_real_speed) {
    // The shared walk cycle (walkers AND the third-person player body): below
    // the stand threshold the phase holds and the body stands (pose 0)...
    Real phase = 0.0;
    CHECK(walkPoseIndex(phase, 0.1, 0.1) == 0);
    CHECK(walkPoseIndex(phase, 0.0, 0.1) == 0);
    CHECK(phase == 0.0);

    // ...and at stride speed (kStrideLength m/s = one cycle per second) the
    // pose index sweeps every pose and the phase stays wrapped to [0, 1).
    bool seen[kWalkPoses] = {};
    for (int i = 0; i < 10; ++i) {
        int pose = walkPoseIndex(phase, kStrideLength, 0.1);
        CHECK(pose >= 0);
        CHECK(pose < kWalkPoses);
        seen[pose] = true;
        CHECK(phase >= 0.0);
        CHECK(phase < 1.0);
    }
    for (int p = 0; p < kWalkPoses; ++p) CHECK(seen[p]);

    // Pose 0 stands; the others really articulate (walkPoseSwing feeds the
    // person-mesh builder).
    CHECK_APPROX(walkPoseSwing(0), 0.0, 1e-12);
    CHECK(std::fabs(walkPoseSwing(1)) > 0.1);
}
