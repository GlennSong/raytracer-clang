#include "test_framework.h"

#include "../src/engine/procgen/city/polyline.h"   // distToSegment, distToPolyline
#include "../src/apps/citysim/city_render.h"
#include "../src/apps/citysim/city_vehicles.h"   // promotedLamps (pure mapping)
#include "../src/engine/asset_manager.h"
#include "../src/engine/block_body.h"   // skinnedLimb
#include "../src/engine/components.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/vehicle_lamps.h"   // the lit-lamp mask bits (ADR-0342)
#include "../src/engine/world.h"
#include "street_map_test_util.h"   // a test world's street map, from its road entities (ADR-0425)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace engine;
using namespace citysim;

namespace {

// A square loop of road: every node routes to every other (degree-2 corners, so
// no junction signals — good for the plain car/ped bake tests).
RoadEntity squareLoop() {
    RoadEntity net;
    net.look.defaultWidth = 10.0;
    net.look.sidewalk = 2.5;
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(40, 0)},
                        RoadNode{Vec2(40, 40)}, RoadNode{Vec2(0, 40)} };
    net.graph.addEdge(0, 1, net.look.defaultWidth);
    net.graph.addEdge(1, 2, net.look.defaultWidth);
    net.graph.addEdge(2, 3, net.look.defaultWidth);
    net.graph.addEdge(3, 0, net.look.defaultWidth);
    return net;
}

// A plus/cross: the centre node has degree 4, so it becomes a signalled junction.
RoadEntity crossRoads() {
    RoadEntity net;
    net.look.defaultWidth = 10.0;
    net.look.sidewalk = 2.5;
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(60, 0)},
                        RoadNode{Vec2(-60, 0)}, RoadNode{Vec2(0, 60)},
                        RoadNode{Vec2(0, -60)} };
    net.graph.addEdge(0, 1, net.look.defaultWidth);
    net.graph.addEdge(0, 2, net.look.defaultWidth);
    net.graph.addEdge(0, 3, net.look.defaultWidth);
    net.graph.addEdge(0, 4, net.look.defaultWidth);
    return net;
}

// The shipped vehicle catalogue. Cars are CONTENT: a city with no recipes draws
// none at all now (the built-in box fleet is gone), so any test that expects to
// see cars has to say which cars — exactly as a level does.
std::string readAsset(const std::string& name) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/" + name);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Lamp markers are published by a car's RECIPE, so a test that asserts on lamps
// needs the geometry actually built — which needs somewhere to upload it. This
// is that somewhere: a stub backend, no GPU.
struct StubUploader : MeshUploader {
    uint32_t next = 1;
    MeshHandle uploadMesh(const RenderMesh&) override { return MeshHandle{next++, 1}; }
    void removeMesh(MeshHandle) override {}
    BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
};

CityRenderParams cityParams() {
    CityRenderParams p;
    p.vehicleScript = readAsset("vehicles.lua");
    p.hoursPerSecond = 0.05;   // a twenty-minute day: these tests watch traffic come and go in a few seconds
    return p;
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
    testmap::addRoads(world, squareLoop());

    CityRenderParams p = cityParams();
    p.cars = 8;
    p.pedestrians = 6;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));   // null AssetManager: transforms only
    CHECK(city.built());
    CHECK(city.nav().linkCount() > 0);
    // THE CARS ARE THE DRIVING'S (ADR-0461): with no rule set named there is no driving, so the eight cars asked for
    // are not made, and the sim's agents are the six walkers, every one drawn at most once.
    CHECK(city.sim().agents().size() == 6u);
    CHECK(carTotal(world, city) == 0u);
    const std::size_t peds = groupCount(world, city.pedGroup());
    CHECK(peds <= 6u);
    CHECK(city.carGroups().size() > 1u);    // the fleet's models are still built, one group each
}

TEST_CASE(city_render_debug_widgets) {
    World world;
    testmap::addRoads(world, squareLoop());

    CityRenderParams p = cityParams();
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
        CHECK(arrows <= 5u);   // never more than the agent count (the five walkers: no driving, no cars)
    }
    // One footprint per agent, spread across the behaviour-state groups.
    std::size_t foot = 0;
    for (int s = 0; s < static_cast<int>(Agent::State::Count); ++s)
        foot += groupCount(world, city.footprintGroup(static_cast<Agent::State>(s)));
    CHECK(foot == 5u);
    CHECK(maxArrows >= 1u);     // agents did move at some point, drawing a trajectory
}

TEST_CASE(external_peds_stop_the_instanced_bake_and_ring_real_walkers) {
    // ADR-0062 walkers: with peds owned by CityWalkerSystem, the render bridge
    // must not bake instanced ped boxes (they'd draw twice), and a ped's debug
    // ring follows the bridge-reported REAL body — unreported walkers draw none.
    World world;
    testmap::addRoads(world, squareLoop());
    CityRenderParams p = cityParams();
    p.cars = 0;
    p.pedestrians = 3;
    p.debugWidgets = true;
    CityRenderSystem city(p);
    city.setPedsExternallyOwned(true);
    CHECK(city.build(world, nullptr));

    const Vec2 realPed(321.0, 654.0);
    // The walkers are the agents: ids 0..2 here.
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

TEST_CASE(city_render_car_colliders_scale_with_the_fleet) {
    World world;
    testmap::addRoads(world, squareLoop());
    CityRenderSystem city(cityParams());
    CHECK(city.build(world, nullptr));

    // One collider extent per car group, and they follow the shared fleet: a group
    // is a body slot, so a box-truck slot's box is bigger than a sedan slot's.
    std::vector<Vec3> ext = city.carGroupHalfExtents();
    CHECK(ext.size() == city.carGroups().size());
    CHECK(!ext.empty());
    Real minZ = 1e30, maxZ = 0;
    for (std::size_t v = 0; v < ext.size(); ++v) {
        CHECK(ext[v].x > 0 && ext[v].y > 0 && ext[v].z > 0);
        // Half-extent mirrors the ADOPTED fleet body's half-length — the sizes
        // the catalogue published, not a table transcribed into C++.
        const VehicleBody& b = city.sim().fleetBody(static_cast<int>(v));
        CHECK(std::fabs(ext[v].z - b.length * 0.5) < 1e-6);
        minZ = std::min(minZ, ext[v].z);
        maxZ = std::max(maxZ, ext[v].z);
    }
    CHECK(maxZ > minZ);   // a bigger body has a bigger collider
}

TEST_CASE(city_render_build_fails_without_roads) {
    World world;
    CityRenderSystem city(cityParams());
    CHECK(!city.build(world, nullptr));
    CHECK(!city.built());
}

// THE POLES ARE THE CITY'S (ADR-0456): on a hand-made road graph the loader placed no signal poles, and the render
// stands none of its own (it once did, by the old signal clock's links) and lights nothing. Where poles stand, their
// heads show the new driving's lamps: level_traffic_lab_v2_the_signal_heads_show_the_new_drivings_lamps.
TEST_CASE(city_render_stands_no_signal_of_its_own) {
    World world;
    testmap::addRoads(world, crossRoads());

    CityRenderParams p = cityParams();
    p.cars = 6;
    p.pedestrians = 0;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));
    const InstanceGroup* posts = world.get<InstanceGroup>(city.signalPostGroup());
    CHECK(posts != nullptr);
    CHECK(posts && posts->transforms.empty());
    CHECK(city.signalHeadCount() == 0u);
    for (int i = 0; i < 50; ++i) city.step(world, 0.1);
    for (int c = 0; c < kLensColourCount; ++c) {
        const InstanceGroup* lens = world.get<InstanceGroup>(city.signalGroup(static_cast<LensColour>(c)));
        CHECK(lens && lens->transforms.empty());
    }
}

// --- car lamps (ADR-0065 follow-up) ------------------------------------------
// When a lamp lights is the shared core's (engine::vehicleLampState, test_vehicle_lamps.cpp). The city's cars are
// entities lit from the driving's braking (CityRenderSystem::syncCarLamps); where no render view has staged the sun
// (headless), darkness is the sim's clock. (The old driving's cars' lamp cases -- headlights by the night clock, brake
// lights holding at a light, indicators on a turn, none on a car owned elsewhere -- went with it: ADR-0461.)
TEST_CASE(city_headlights_follow_the_clock_outside_dawn_to_dusk) {
    CHECK(CityRenderSystem::clockIsDark(2.0));    // deep night: on
    CHECK(CityRenderSystem::clockIsDark(6.0));    // before dawn: on
    CHECK(!CityRenderSystem::clockIsDark(6.5));   // AT dawn: off (not before it)
    CHECK(!CityRenderSystem::clockIsDark(12.0));  // midday: off
    CHECK(!CityRenderSystem::clockIsDark(19.0));  // AT dusk: off (not after it)
    CHECK(CityRenderSystem::clockIsDark(19.1));   // just past dusk: on
    CHECK(CityRenderSystem::clockIsDark(23.5));
}

// (Where a signal pole stands clear of every carriageway is the furniture planner's, the one that places them:
// test_street_furniture.cpp signal_poles_clear_every_arm_at_an_acute_corner, signal_poles_stand_clear_of_the_junction_pad.
// The render's own pole placer, and its test here, went with the old signal clock, ADR-0456.)

// (Where crosswalks ARE is the road model's since ADR-0452, ModelCrosswalk, tested in test_road_model.cpp; the render
// system's own guessed "crosswalk centres", which nothing read, are gone.)
TEST_CASE(crosswalks_are_painted_into_the_road_not_drawn_as_decals) {
    World world;
    testmap::addRoads(world, crossRoads());   // 4-arm cross, width 10
    CityRenderSystem city(cityParams());
    CHECK(city.build(world, nullptr));
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
    testmap::addRoads(world, squareLoop());
    CitySimConfig cfg;
    cfg.cars = 1;
    cfg.pedestrians = 1;
    cfg.seed = 7;
    cfg.debugWidgets = true;
    cfg.vehicleScript = readAsset("vehicles.lua");   // the block wins, catalogue included
    world.add<CitySimConfig>(world.create(), cfg);

    CityRenderParams p = cityParams();          // the constructor asks for a crowd...
    p.cars = 40;
    p.pedestrians = 40;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));
    // ...but the LEVEL's block wins: its one walker (its one car is the driving's, and the block names no rule set:
    // no driving, so no car, ADR-0461)
    CHECK(city.sim().agents().size() == 1u);
    CHECK(carTotal(world, city) == 0u);
    CHECK(groupCount(world, city.pedGroup()) == 1u);
}

TEST_CASE(city_places_carry_where_their_doors_front) {
    // ADR-0480: the street, station and kerb (or the walkway and metres along it) the lot pass recorded for a place's
    // door (AuthoredPlace::frontage, ADR-0474) reach the sim's place unchanged, so a walk can start and end where the door
    // states. A place that states nothing says nothing. (This hand world has no lane-built city: the sim holds no road
    // model.)
    World world;
    testmap::addRoads(world, crossRoads());
    CitySimConfig cfg;
    cfg.pedestrians = 1;
    cfg.seed = 7;
    AuthoredPlace home;
    home.type = "home";
    home.x = 20;
    home.z = 12;
    home.frontage.street = "s3";
    home.frontage.station = 41.25;
    home.frontage.dir = -1;
    AuthoredPlace shop;
    shop.type = "shop";
    shop.x = -20;
    shop.z = -12;
    shop.openHour = 9;
    shop.closeHour = 17;
    shop.frontage.walkway = "lot4.p2";
    shop.frontage.along = 7.5;
    AuthoredPlace office;
    office.type = "office";
    office.x = 12;
    office.z = -20;
    cfg.places = {home, shop, office};
    world.add<CitySimConfig>(world.create(), cfg);
    CityRenderSystem city(cityParams());
    CHECK(city.build(world, nullptr));
    const std::vector<Place>& places = city.places().places();
    CHECK(places.size() == 3u);
    if (places.size() != 3u) return;
    CHECK(places[0].frontage.street == "s3" && places[0].frontage.station == 41.25 && places[0].frontage.dir == -1);
    CHECK(!places[0].frontage.onWalkway());
    CHECK(places[1].frontage.walkway == "lot4.p2" && places[1].frontage.along == 7.5 && !places[1].frontage.onStreet());
    CHECK(!places[2].frontage.onStreet() && !places[2].frontage.onWalkway());
    CHECK(city.sim().roads() == nullptr);
}

TEST_CASE(agent_lab_theta_circuit_is_navigable) {
    // The agent-lab level (assets/levels/agent_lab.json) is a theta circuit: a
    // 120x80 ring with a middle bar, meeting at two 3-way junctions. Mirror its
    // geometry here and pin what the lab depends on: both junctions carry
    // signals, every node routes to every other, and a lone driver + walker
    // actually get moving on it.
    RoadEntity net;
    net.look.defaultWidth = 7.0;
    net.look.sidewalk = 1.8;
    net.graph.nodes = { RoadNode{Vec2(-60, -40)}, RoadNode{Vec2(60, -40)},
                        RoadNode{Vec2(60, 40)}, RoadNode{Vec2(-60, 40)},
                        RoadNode{Vec2(-60, 0)}, RoadNode{Vec2(60, 0)} };
    net.graph.addEdge(0, 1, net.look.defaultWidth);
    net.graph.addEdge(1, 5, net.look.defaultWidth);
    net.graph.addEdge(5, 2, net.look.defaultWidth);
    net.graph.addEdge(2, 3, net.look.defaultWidth);
    net.graph.addEdge(3, 4, net.look.defaultWidth);
    net.graph.addEdge(4, 0, net.look.defaultWidth);
    net.graph.addEdge(4, 5, net.look.defaultWidth);

    World world;
    testmap::addRoads(world, net);
    CitySimConfig cfg;
    cfg.cars = 1;
    cfg.pedestrians = 1;
    cfg.seed = 7;
    cfg.wander = true;   // the lab keeps its agents perpetually on the move
    world.add<CitySimConfig>(world.create(), cfg);
    CityRenderSystem city(cityParams());
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

    // The lab's walker comes alive and, wandering, strings trip after trip together. (Its car was the old driving's:
    // with no rule set named the level has no driving, so no car, ADR-0461.)
    bool pedMoved = false;
    for (int i = 0; i < 6000; ++i) {
        city.step(world, 0.1);
        for (const Agent& a : city.sim().agents())
            if (a.moving && a.speed > 0.5) pedMoved = true;
    }
    CHECK(pedMoved);
    CHECK(city.sim().agents().size() == 1u);
    CHECK(!city.sim().agents().empty() && city.sim().agents().front().trips >= 3);
}

TEST_CASE(wedge_mesh_is_a_flat_outline_cone) {
    // The vision-cone wedge (city_meshes): a ground-facing OUTLINE — two straight
    // edges from the origin plus the arc — of unit radius, built from thin quads
    // like ringXZ so it draws with the same widgetMaterial ground-paint technique.
    RenderMesh w = wedgeXZ(0.45);
    CHECK(!w.vertices.empty());
    CHECK(w.vertices.size() == 2u * 4u + 13u * 2u);   // 2 edge quads + the arc's 12 quads on 13 shared pairs
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
    testmap::addRoads(world, crossRoads());
    CityRenderParams p = cityParams();
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
    const double lift = crossRoads().look.lift;   // strips ride the net's own lift
    for (const Mat4& m : strips->transforms)
        CHECK(std::fabs(m.m[1][3] - (lift + 0.04)) < 1e-6);   // road lift + 0.04
    // One ring per junction node (the cross has exactly one).
    int junctions = 0;
    for (int n = 0; n < nav.nodeCount(); ++n)
        if (nav.isJunction(n)) ++junctions;
    CHECK(junctions == 1);
    CHECK(groupCount(world, city.navNodeGroup()) == static_cast<std::size_t>(junctions));

    // Cones: exactly one per MOVING walker every step; the navgraph stays put (static bake, not rebaked per frame).
    bool sawPedCone = false;
    for (int i = 0; i < 3000; ++i) {
        city.step(world, 0.1);
        std::size_t movingPeds = 0;
        for (const Agent& a : city.sim().agents())
            if (a.moving) ++movingPeds;
        CHECK(groupCount(world, city.visionGroup()) == movingPeds);
        CHECK(groupCount(world, city.navLinkGroup()) == lanes);
        if (movingPeds > 0) sawPedCone = true;
    }
    CHECK(sawPedCone);   // walkers really moved, so both views really drew
}

TEST_CASE(debug_navgraph_and_cones_vanish_when_the_hud_is_off) {
    World world;
    testmap::addRoads(world, crossRoads());
    CityRenderParams p = cityParams();
    p.cars = 4;
    p.pedestrians = 3;
    p.debugWidgets = false;   // HUD off: every debug group must stay empty
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));
    for (int i = 0; i < 500; ++i) city.step(world, 0.1);
    CHECK(groupCount(world, city.navLinkGroup()) == 0u);
    CHECK(groupCount(world, city.navNodeGroup()) == 0u);
    CHECK(groupCount(world, city.visionGroup()) == 0u);
}

TEST_CASE(person_mesh_stands_and_swings) {
    // The simple articulated person (device ask: "people should look like
    // people"): the head and torso as vertex-coloured boxes and each limb one
    // skinned tube bent at the knee or elbow, 1.8 m tall, centred at mid-height
    // like the old walker box. A swing pose moves the limbs but never changes
    // topology — the walk cycle hops between shared pose meshes.
    RenderMesh stand = buildPersonMesh(0.0, 0);
    RenderMesh step = buildPersonMesh(0.5, 0);
    const std::size_t tube = engine::skinnedLimb(0, 0.0, Vec3(1, 1, 1)).vertices.size();
    CHECK(stand.vertices.size() == 2u * 24u + 4u * tube);   // head/torso + 2 arms/2 legs
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

TEST_CASE(density_population_scales_with_the_graph) {
    // Roads-v2.1 4c: pedestrians = -1 asks for DENSITY population — a count derived from sidewalk-km, so a bigger city
    // gets more people from the same recipe, and explicit counts stay overrides. (The cars a density asks for are the
    // driving's, made only where the level names a rule set: none here, ADR-0461.)
    auto build = [](RoadEntity net, int peds, Real perKm, World& world) {
        testmap::addRoads(world, std::move(net));
        CityRenderParams p = cityParams();
        p.cars = -1;
        p.pedestrians = peds;
        p.pedsPerKm = perKm;
        p.seed = 5;
        CityRenderSystem city(p);
        CHECK(city.build(world, nullptr));
        return static_cast<int>(city.sim().agents().size());
    };
    auto grid = []() {
        RoadEntity net;
        net.look.defaultWidth = 10.0;
        net.look.sidewalk = 2.5;
        const int N = 4;
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i)
                net.graph.nodes.push_back(RoadNode{Vec2(i * 80.0, j * 80.0)});
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                if (i + 1 < N)
                    net.graph.addEdge(j * N + i, j * N + i + 1, net.look.defaultWidth);
                if (j + 1 < N)
                    net.graph.addEdge(j * N + i, (j + 1) * N + i, net.look.defaultWidth);
            }
        return net;
    };

    World w1, w2, w3, w4;
    const int tiny = build(squareLoop(), -1, 30.0, w1);   // 0.16 km loop
    const int big = build(grid(), -1, 30.0, w2);          // ~1.9 km grid
    const int denser = build(grid(), -1, 60.0, w3);       // same city, 2x density
    const int fixed_ = build(grid(), 9, 30.0, w4);        // explicit count wins
    std::printf("    [density] walkers tiny=%d big=%d denser=%d fixed=%d\n", tiny, big, denser, fixed_);
    CHECK(tiny >= 6);              // floor keeps a hamlet alive
    CHECK(big > tiny);             // more city -> more people
    CHECK(denser > big);           // knob scales it
    CHECK(fixed_ == 9);            // explicit count is an override
}

// --- P8.2d: sim rate below the fixed step -----------------------------------
// Traffic is not physics, so it may tick slower than 60 Hz — but the DRAWN
// world must not: a body that freezes for a tick then jumps both judders and
// (because Jolt reads a kinematic body's velocity from its position delta)
// shoves the player. These pin the three properties that make the rate ladder
// safe: the historical default is untouched, a slower rate still covers the
// same ground, and poses keep moving between ticks.

namespace {
// Sum of |position| deltas across every drawn walker, over `steps` calls. (Cars until ADR-0461: the city's agents are
// all walkers now, its cars the driving's entities.)
Real driveDistance(World& world, CityRenderSystem& city, int steps, Real dt) {
    auto snapshot = [&]() {
        std::vector<Vec3> out;
        if (InstanceGroup* g = world.get<InstanceGroup>(city.pedGroup()))
            for (const Mat4& m : g->transforms)
                out.push_back(Vec3(m.m[0][3], m.m[1][3], m.m[2][3]));
        return out;
    };
    std::vector<Vec3> prev = snapshot();
    Real total = 0;
    for (int i = 0; i < steps; ++i) {
        city.step(world, dt);
        std::vector<Vec3> now = snapshot();
        for (std::size_t k = 0; k < now.size() && k < prev.size(); ++k)
            total += (now[k] - prev[k]).length();
        prev = std::move(now);
    }
    return total;
}
}  // namespace

TEST_CASE(city_sim_rate_default_is_every_step) {
    // localHz 0 must reproduce the historical tick exactly — every existing
    // level and gate depends on it.
    World a, b;
    testmap::addRoads(a, squareLoop());
    testmap::addRoads(b, squareLoop());
    CityRenderParams p = cityParams();
    p.cars = 0; p.pedestrians = 6; p.seed = 5;
    p.wander = true;              // move continuously (no schedule window)
    CityRenderParams q = p;
    q.localHz = 0.0;              // explicit default
    CityRenderSystem ca(p), cb(q);
    CHECK(ca.build(a, nullptr));
    CHECK(cb.build(b, nullptr));
    for (int i = 0; i < 120; ++i) { ca.step(a, 1.0 / 60.0); cb.step(b, 1.0 / 60.0); }
    const auto& A = ca.sim().agents();
    const auto& B = cb.sim().agents();
    CHECK(A.size() == B.size());
    bool identical = true;
    for (std::size_t i = 0; i < A.size() && i < B.size(); ++i)
        identical = identical && A[i].pos.x == B[i].pos.x && A[i].pos.y == B[i].pos.y;
    CHECK(identical);
}

TEST_CASE(city_sim_rate_30hz_covers_the_same_ground) {
    // Half the ticks, twice the dt: the fleet must travel a comparable
    // distance (coarser integration, not slower traffic).
    World a, b;
    testmap::addRoads(a, squareLoop());
    testmap::addRoads(b, squareLoop());
    CityRenderParams p = cityParams();
    p.cars = 0; p.pedestrians = 8; p.seed = 5;
    p.wander = true;              // move continuously (no schedule window)
    CityRenderParams q = p;
    q.localHz = 30.0;
    q.adaptiveRate = false;       // pin the rate: this asserts 30 Hz, not load
    CityRenderSystem ca(p), cb(q);
    CHECK(ca.build(a, nullptr));
    CHECK(cb.build(b, nullptr));
    const Real full = driveDistance(a, ca, 600, 1.0 / 60.0);
    const Real half = driveDistance(b, cb, 600, 1.0 / 60.0);
    std::printf("        [sim rate] 60Hz travel %.1f m, 30Hz travel %.1f m\n",
                full, half);
    CHECK(full > 1.0);            // the fixture actually drives
    CHECK(half > full * 0.75);    // and 30 Hz covers the same ground
    CHECK(half < full * 1.25);
}

TEST_CASE(city_sim_rate_poses_keep_moving_between_ticks) {
    // THE judder/shove gate: on a fixed step where the sim does NOT tick, the
    // baked poses must still advance (extrapolated along heading), or cars
    // freeze for a step and then teleport.
    World world;
    testmap::addRoads(world, squareLoop());
    CityRenderParams p = cityParams();
    p.cars = 0; p.pedestrians = 8; p.seed = 5;
    p.wander = true;              // move continuously (no schedule window)
    p.localHz = 30.0;
    p.adaptiveRate = false;
    CityRenderSystem city(p);
    CHECK(city.build(world, nullptr));
    for (int i = 0; i < 240; ++i) city.step(world, 1.0 / 60.0);   // get going

    auto poses = [&]() {
        std::vector<Vec3> out;
        if (InstanceGroup* g = world.get<InstanceGroup>(city.pedGroup()))
            for (const Mat4& m : g->transforms)
                out.push_back(Vec3(m.m[0][3], m.m[1][3], m.m[2][3]));
        return out;
    };
    // Two consecutive steps: one of them ticks the sim, the other must still
    // move the drawn walkers. Neither may be a frozen frame.
    int frozen = 0;
    std::vector<Vec3> prev = poses();
    for (int i = 0; i < 8; ++i) {
        city.step(world, 1.0 / 60.0);
        std::vector<Vec3> now = poses();
        Real moved = 0;
        for (std::size_t k = 0; k < now.size() && k < prev.size(); ++k)
            moved = std::max(moved, (now[k] - prev[k]).length());
        if (moved < 1e-6) ++frozen;
        prev = std::move(now);
    }
    std::printf("        [sim rate] frozen steps out of 8: %d\n", frozen);
    CHECK(frozen == 0);
}

TEST_CASE(city_sim_far_tier_refreshes_on_a_clock_not_a_tick_count) {
    // The bug this pins: the far tier's refresh was budgeted in TICKS ("60
    // buckets, one per tick"), which only means ~1 Hz while 60 ticks take a
    // second. Halve the sim rate and distant agents refreshed half as often,
    // went stale near the bubble edge, and the tier pass over-promoted them.
    // Same wall time, two rates: the far population must be comparable.
    auto activeCount = [](CityRenderSystem& c) {
        int k = 0;
        for (const Agent& a : c.sim().agents())
            if (a.tier != Agent::Tier::V) ++k;
        return k;
    };
    auto run = [&](Real hz, int& active, int& far) {
        World world;
        // A loop big enough that traffic actually LEAVES the bubble (promote
        // 500 m / demote 620 m) — on a 40 m fixture everything stays K and the
        // gate would prove nothing.
        RoadEntity big;
        big.look.defaultWidth = 10.0;
        big.look.sidewalk = 2.5;
        big.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(1600, 0)},
                            RoadNode{Vec2(1600, 1600)}, RoadNode{Vec2(0, 1600)} };
        big.graph.addEdge(0, 1, big.look.defaultWidth);
        big.graph.addEdge(1, 2, big.look.defaultWidth);
        big.graph.addEdge(2, 3, big.look.defaultWidth);
        big.graph.addEdge(3, 0, big.look.defaultWidth);
        testmap::addRoads(world, big);
        CityRenderParams p = cityParams();
        p.cars = 0; p.pedestrians = 60; p.seed = 5;
        p.wander = true;
        p.tieredAgents = true;
        p.localHz = hz;
        p.adaptiveRate = false;
        CityRenderSystem city(p);
        CHECK(city.build(world, nullptr));
        // A player entity IS the bubble centre: the bridge feeds its position
        // to the sim every step (no player => everything stays K).
        Entity player = world.create();
        world.add<Transform>(player);
        world.add<CharacterController>(player);
        for (int i = 0; i < 600; ++i) city.step(world, 1.0 / 60.0);   // 10 s
        active = activeCount(city);
        far = static_cast<int>(city.sim().agents().size()) - active;
    };
    int a60 = 0, f60 = 0, a30 = 0, f30 = 0;
    run(0.0, a60, f60);     // every step
    run(30.0, a30, f30);    // half rate
    std::printf("        [far tier] 60Hz active %d/far %d | 30Hz active %d/far %d\n",
                a60, f60, a30, f30);
    // The tier split must not depend on the sim's rate.
    //
    // HONEST SCOPE: this pins the INVARIANT, not the piedmont regression. On
    // this fixture the old tick-count budget passes too — 60 agents on a 6.4 km
    // loop put only a handful near the 500 m boundary, where two seconds of
    // staleness moves a car ~26 m. Piedmont's K/P inflation (950 -> 2650) needs
    // a dense population against that boundary to reproduce, so the mechanism
    // there may be more than sample staleness. Enable localHz on a real level
    // only behind a measurement.
    CHECK(a30 <= a60 + 6);
}

// The fleet bridge end to end (scripting builds only): a level whose citysim
// block names a vehicles.lua-style script must leave the render bridge holding
// a WHEEL-LESS body and a wheel layout for every variant. That pair is what a
// commandeered car (CityVehicleSystem, ADR-0062) is dressed and wheeled from;
// without it, promotion silently falls back to the retired C++ box body, which
// is exactly the skew that shipped — a whole city of mesh.car traffic and a box
// in the player's hands.
TEST_CASE(fleet_bridge_holds_a_promotable_body_per_variant) {
    struct StubUploader : MeshUploader {
        uint32_t next = 1;
        MeshHandle uploadMesh(const RenderMesh&) override { return MeshHandle{next++, 1}; }
        void removeMesh(MeshHandle) override {}
        BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
    };
    StubUploader up;
    AssetManager assets(up);

    World world;
    testmap::addRoads(world, squareLoop());
    CityRenderParams p = cityParams();
    p.cars = 8;
    p.pedestrians = 0;
    p.vehicleScript = readAsset("vehicles.lua");
    CHECK(!p.vehicleScript.empty());
    CityRenderSystem city(p);
    CHECK(city.build(world, &assets));

    for (int v = 0; v < static_cast<int>(city.carGroups().size()); ++v) {
        CHECK(city.carChassisMesh(v).valid());
        const engine::PhysicsWorld::VehicleConfig* c = city.carConfig(v);   // (a motorcycle's slot: two wheels)
        CHECK(city.carWheels(v).size() == (c && c->twoWheeler ? 2u : 4u));
    }
    // Slots wrap the way a driver's vehicle index does, so a car in any slot can
    // be promoted (the sim hands out indices unbounded by the fleet size).
    CHECK(city.carChassisMesh(static_cast<int>(city.carGroups().size())).valid());

    // DOUBLED TAILLIGHTS. VehicleSystem lights a promoted car from the markers
    // its Vehicle carries, and falls back to four lenses at guessed chassis
    // CORNERS when it carries none. Those corners sat almost exactly on the
    // retired box car's baked lamp boxes, so the guess was invisible; against a
    // real generated body, whose housings are inset from the extremes, it lit a
    // second pair out beyond the bodywork. Every promoted car must therefore
    // arrive with its own four lamps.
    for (int v = 0; v < static_cast<int>(city.carGroups().size()); ++v) {
        engine::Vec3 seat(0, 0, 0);
        bool hasSeat = false;
        const std::vector<PromotedLamp> lamps =
            promotedLamps(city.carLamps(v), &seat, &hasSeat);
        const engine::PhysicsWorld::VehicleConfig* c = city.carConfig(v);
        const bool bike = c && c->twoWheeler;   // one lamp each end, on its centre line
        CHECK(lamps.size() == (bike ? 2u : 4u));
        int front = 0, left = 0;
        for (const PromotedLamp& l : lamps) {
            if (l.front) ++front;
            if (l.left) ++left;
        }
        CHECK(front == (bike ? 1 : 2));   // two at the nose...
        CHECK(bike || left == 2);         // ...and one of each pair per indicator side
        // ON the bodywork, which is the whole difference from the corner guess:
        // that fallback puts its lenses at ±(halfLength + 0.05), i.e. floating
        // just PAST the tail. A real marker sits on the nose/tail ring itself
        // (so |z| reaches the half-length but never exceeds it) and is inset
        // laterally to the lamp housing.
        const Vec3 he = city.carGroupHalfExtents()[v];
        for (const PromotedLamp& l : lamps) {
            CHECK(std::fabs(l.local.z) <= he.z + 1e-3);
            CHECK(std::fabs(l.local.x) < he.x);
        }
    }
}

TEST_CASE(promoted_lamp_mapping_reads_names_not_positions) {
    // The mapping itself, on hand-written markers: prefix picks the end, the
    // trailing l/r picks the indicator side, driver_seat is extracted rather
    // than lit, and anything else is ignored.
    // Marker x-signs follow the physical chirality: the car's left ('_l') is
    // +X with forward +Z (see car_mesh.cpp's lamp block).
    std::vector<CityRenderSystem::LampMarker> markers = {
        {"headlight_l", Vec3(0.7, -0.1, 2.0)},
        {"headlight_r", Vec3(-0.7, -0.1, 2.0)},
        {"taillight_l", Vec3(0.7, -0.1, -2.0)},
        {"taillight_r", Vec3(-0.7, -0.1, -2.0)},
        {"driver_seat", Vec3(0.35, 0.1, 0.2)},
        {"badge", Vec3(0, 0, 2.1)},
    };
    Vec3 seat(0, 0, 0);
    bool hasSeat = false;
    std::vector<PromotedLamp> lamps = promotedLamps(markers, &seat, &hasSeat);
    CHECK(lamps.size() == 4u);        // the badge is not a lamp
    CHECK(hasSeat);
    CHECK(std::fabs(seat.x - 0.35) < 1e-9);
    CHECK(lamps[0].front && lamps[0].left);
    CHECK(lamps[1].front && !lamps[1].left);
    CHECK(!lamps[2].front && lamps[2].left);
    CHECK(!lamps[3].front && !lamps[3].left);

    // A seat marker at the body origin still reports as PRESENT (the reason
    // presence is a flag and not a non-zero test).
    std::vector<CityRenderSystem::LampMarker> atOrigin = {
        {"driver_seat", Vec3(0, 0, 0)}};
    promotedLamps(atOrigin, &seat, &hasSeat);
    CHECK(hasSeat);

    // No markers at all: no lamps, no seat — the caller leaves VehicleSystem to
    // its corner fallback rather than fabricating one.
    promotedLamps({}, &seat, &hasSeat);
    CHECK(!hasSeat);
}

// THE ASSET SETS THE FLEET'S LENGTH. This was a C++ constant (the size of
// city_meshes' colour table), so a level whose recipes declared any other number
// of cars got silently corrected: extra slots were never built, missing ones
// each fell back to a box. A three-car fleet must produce three variants, and
// every driver must map into them.
TEST_CASE(fleet_length_comes_from_the_asset_not_from_cpp) {
    struct StubUploader : MeshUploader {
        uint32_t next = 1;
        MeshHandle uploadMesh(const RenderMesh&) override { return MeshHandle{next++, 1}; }
        void removeMesh(MeshHandle) override {}
        BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
    };
    StubUploader up;
    AssetManager assets(up);

    World world;
    testmap::addRoads(world, squareLoop());
    CityRenderParams p = cityParams();
    p.cars = 10;               // more drivers than slots: the mapping must wrap
    p.pedestrians = 0;
    // Three slots, each a plain box body with a size — no mesh.car needed.
    p.vehicleScript = R"lua(
        local function slot(w, h, l)
          return { size = { w, h, l },
                   parts = { { pos = {0,0,0}, size = {w,h,l}, color = {1,0,0} } } }
        end
        vehicle = { fleet = { slot(1.8,1.4,4.0), slot(2.0,1.9,5.2), slot(2.3,2.7,6.4) } }
    )lua";
    CityRenderSystem city(p);
    CHECK(city.build(world, &assets));

    CHECK(city.carGroups().size() == 3u);
    CHECK(city.sim().fleetSize() == 3);
    // The adopted sizes are the recipe's, in slot order.
    CHECK(std::fabs(city.sim().fleetBody(0).length - 4.0) < 1e-9);
    CHECK(std::fabs(city.sim().fleetBody(2).height - 2.7) < 1e-9);
    // ...and they wrap, so driver 3 wears slot 0's body.
    CHECK(std::fabs(city.sim().fleetBody(3).length - 4.0) < 1e-9);
    // Collider proxies follow the same three sizes.
    CHECK(city.carGroupHalfExtents().size() == 3u);
    CHECK(std::fabs(city.carGroupHalfExtents()[2].z - 6.4 * 0.5) < 1e-9);
}

// The other half of the contract: NO vehicle script means no promotable body,
// and the caller must be able to see that rather than receive a wheeled mesh.
TEST_CASE(fleet_bridge_reports_no_promotable_body_without_a_script) {
    struct StubUploader : MeshUploader {
        uint32_t next = 1;
        MeshHandle uploadMesh(const RenderMesh&) override { return MeshHandle{next++, 1}; }
        void removeMesh(MeshHandle) override {}
        BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
    };
    StubUploader up;
    AssetManager assets(up);

    World world;
    testmap::addRoads(world, squareLoop());
    CityRenderParams p;                // deliberately NO vehicle catalogue
    p.cars = 4;
    p.pedestrians = 0;
    CityRenderSystem city(p);
    CHECK(city.build(world, &assets));
    // No recipes, no cars — not a substitute car. The built-in box fleet that
    // used to stand in here is gone; a level says `"vehicles": "..."` to have
    // traffic, and its absence is a decision the level made.
    CHECK(city.carGroups().empty());
    CHECK(!city.carChassisMesh(0).valid());
    CHECK(city.carWheels(0).empty());
}

// LOCKSTEP WITH THE SKY (device: "I'd like to keep the simulation in lock
// step with day night"). The city opens at the sky's hour, runs at the sky's
// rate, follows a sky JUMP by re-placing everyone from their schedules at the
// new hour, and holds its clock while the sky is held.
TEST_CASE(city_clock_follows_the_sky_in_lockstep) {
    StubUploader up;
    AssetManager assets(up);
    World world;
    testmap::addRoads(world, squareLoop());
    CityRenderParams p = cityParams();
    p.cars = 12;
    p.pedestrians = 8;
    p.hoursPerSecond = 0.05;   // the level's own clock: overridden by the sky's
    p.startHour = 10.5;
    CityRenderSystem city(p);
    CHECK(!city.setWorldClock(6.0, 3.0));       // staged before build: opens here
    CHECK(city.build(world, &assets));
    CHECK_APPROX(city.sim().timeOfDay(), 6.0, 1e-6);
    for (int i = 0; i < 10; ++i) city.step(world, 0.1);   // 1 s at 3 h/s
    CHECK_APPROX(city.sim().timeOfDay(), 9.0, 1e-6);       // the sky's rate, not 0.05

    // A JUMP: the sky goes to 22:00. The sim re-seeds at 22:00 and every
    // resting AI agent is where its own schedule puts it at that hour. At a
    // LIVABLE rate: at 3 h/s a one-minute commute is 180 in-world hours and
    // the whole city is "in flight" all day (the first cut checked 0 agents).
    CHECK(city.setWorldClock(22.0, 0.005));
    CHECK_APPROX(city.sim().timeOfDay(), 22.0, 1e-6);
    int checked = 0, agree = 0;
    for (const Agent& a : city.sim().agents()) {
        if (a.playerControlled || a.released || a.home == a.work) continue;
        const CitySim::Snapshot s = city.sim().scheduleSnapshot(a, 22.0);
        Agent::Activity want;
        if (s.where == CitySim::Snapshot::Where::AtHome) want = Agent::Activity::AtHome;
        else if (s.where == CitySim::Snapshot::Where::AtWork) want = Agent::Activity::AtWork;
        else continue;   // in flight: placed a measured distance along the trip
        ++checked;
        if (a.activity == want) ++agree;
    }
    std::printf("    [lockstep] %d/%d resting agents where 22:00 puts them\n",
                agree, checked);
    CHECK(checked > 0);
    CHECK(agree == checked);
    // Drift inside the tolerance is not a jump (no re-seed churn).
    CHECK(!city.setWorldClock(22.01, 0.005));
    CHECK_APPROX(city.sim().timeOfDay(), 22.0, 1e-6);

    // A HELD sky: rate 0 holds the clock; the sim itself keeps running.
    CHECK(!city.setWorldClock(22.0, 0.0));
    const Real before = city.sim().seconds();
    for (int i = 0; i < 10; ++i) city.step(world, 0.1);
    CHECK_APPROX(city.sim().timeOfDay(), 22.0, 1e-6);
    CHECK(city.sim().seconds() > before);
    // Released: it runs again at the sky's rate from where it was held.
    CHECK(!city.setWorldClock(22.0, 3.0));
    for (int i = 0; i < 10; ++i) city.step(world, 0.1);
    CHECK_APPROX(city.sim().timeOfDay(), 1.0, 1e-6);   // 22 + 3 wraps
}
