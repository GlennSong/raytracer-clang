// THE ONE LIGHT SYSTEM (ADR-0362): lights.lua reads into kinds and pools (and a broken file is refused), and the one
// picker chooses as every producer used to on its own -- per pool the player's own first, then the nearest in reach
// up to the count, a car's lamps together, the set fading toward its moving edge, the dusk ramp on the kinds that
// ride it, a kind with no pool always lit.

#include "test_framework.h"
#include "../src/engine/lights/light_picker.h"
#include "../src/engine/lights/probe_sites.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/components.h"   // drawClassName
#include "../src/engine/scripting/lights_lua.h"
#include <cmath>
#include <fstream>
#include <sstream>

using namespace engine;

namespace {
std::string readLightsFile() {
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/scripts/lights.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
// a small book: lamp (point, pool street: 3 within 100, fading), head (spot, pool cars: 2 groups), torch (no pool)
LightBook testBook() {
    LightBook b;
    b.pools.push_back({"street", 3, 100.0f, true, false});
    b.pools.push_back({"cars", 2, 0.0f, false, true});
    LightKind lamp;
    lamp.name = "lamp"; lamp.intensity = 10.0f; lamp.range = 30.0f; lamp.dusk = true; lamp.pool = 0;
    lamp.offset = Vec3(0, -0.5, 0);
    LightKind head;
    head.name = "head"; head.spot = true; head.intensity = 20.0f; head.inner = 0.3f; head.outer = 0.5f; head.pool = 1;
    LightKind torch;
    torch.name = "torch"; torch.spot = true; torch.intensity = 100.0f;
    b.kinds = {lamp, head, torch};
    return b;
}
LightCandidate at(int kind, Real x, uint32_t group = 0, bool mine = false) {
    LightCandidate c;
    c.kind = kind;
    c.position = Vec3(x, 0, 0);
    c.group = group;
    c.mine = mine;
    return c;
}
}  // namespace

TEST_CASE(lights_lua_reads_every_kind_the_producers_name) {
    LightBook book;
    std::string err;
    CHECK(loadLightsSource(readLightsFile(), book, &err));
    if (!err.empty()) std::printf("    %s\n", err.c_str());
    for (const char* name : {"street_lamp", "station_lamp", "headlight", "flashlight", "room_fill", "room_panel", "room_round",
                             "room_pendant", "cab", "beacon"})
        CHECK(book.kind(name) >= 0);
    // the street pool's defaults are the ones the render system kept (20 within 160 m), the headlights' 3 cars
    const int street = book.pool("street"), heads = book.pool("headlights"), beacons = book.pool("beacons");
    CHECK(street >= 0 && heads >= 0 && beacons >= 0);
    if (street >= 0) CHECK(book.pools[static_cast<std::size_t>(street)].count == 20 && book.pools[static_cast<std::size_t>(street)].within == 160.0f);
    if (heads >= 0) CHECK(book.pools[static_cast<std::size_t>(heads)].count == 3 && book.pools[static_cast<std::size_t>(heads)].groups);
    const LightKind& hl = book.kinds[static_cast<std::size_t>(book.kind("headlight"))];
    CHECK(hl.spot && hl.intensity == 22.0f && hl.range == 45.0f);
}

TEST_CASE(lights_lua_refuses_a_broken_file) {
    LightBook book;
    std::string err;
    CHECK(!loadLightsSource("return { kinds = { x = { type = 'beam' } } }", book, &err));
    CHECK(!err.empty());
    CHECK(!loadLightsSource("return { kinds = { x = { type = 'point', pool = 'nowhere' } } }", book, &err));
    CHECK(!loadLightsSource("return { kinds = { x = { type = 'spot', cone = { 0.5, 0.3 } } } }", book, &err));
    CHECK(book.kinds.empty());   // untouched
}

TEST_CASE(light_picker_keeps_the_nearest_in_reach_up_to_the_count) {
    const LightBook b = testBook();
    std::vector<LightCandidate> c = {at(0, 90), at(0, 10), at(0, 150), at(0, 30), at(0, 60), at(0, 20)};
    std::vector<PointLight> pts;
    std::vector<SpotLight> sps;
    pickLights(b, b.pools, c, Vec3(0, 0, 0), 1.0f, pts, sps);
    // 150 is out of reach; of the five in reach the nearest three (10, 20, 30); the 30 is the moving edge, so dark
    CHECK(pts.size() == 2);
    for (const PointLight& p : pts) CHECK(p.position.x == 10.0 || p.position.x == 20.0);
    for (const PointLight& p : pts) CHECK(std::fabs(p.position.y - (-0.5)) < 1e-9);   // the kind's offset
    CHECK(sps.empty());
}

TEST_CASE(light_picker_fades_the_set_toward_its_edge) {
    CHECK(lightEdgeFade(10.0f, 100.0f) == 1.0f);
    CHECK(lightEdgeFade(75.0f, 100.0f) == 1.0f);
    CHECK(lightEdgeFade(100.0f, 100.0f) == 0.0f);
    const float mid = lightEdgeFade(87.5f, 100.0f);
    CHECK(mid > 0.45f && mid < 0.55f);
    CHECK(lightEdgeFade(5000.0f, 0.0f) == 1.0f);   // no edge: no fade
    // under the count, the reach is the edge: a lamp at 90 of 100 is part faded
    const LightBook b = testBook();
    std::vector<PointLight> pts;
    std::vector<SpotLight> sps;
    pickLights(b, b.pools, {at(0, 90)}, Vec3(0, 0, 0), 1.0f, pts, sps);
    CHECK(pts.size() == 1);
    if (!pts.empty()) CHECK(pts[0].intensity > 0.0f && pts[0].intensity < 10.0f);
}

TEST_CASE(light_picker_takes_a_cars_lamps_together_and_the_players_first) {
    const LightBook b = testBook();
    // three cars (two lamps each) at 5, 40 and 80 m; the player's own is the farthest
    std::vector<LightCandidate> c = {at(1, 5, 7), at(1, 6, 7), at(1, 40, 8), at(1, 41, 8), at(1, 80, 9, true), at(1, 81, 9, true)};
    std::vector<PointLight> pts;
    std::vector<SpotLight> sps;
    pickLights(b, b.pools, c, Vec3(0, 0, 0), 0.0f, pts, sps);
    // two cars: the player's (first, whatever its distance) and the nearest; both lamps of each; not dusk-gated
    CHECK(sps.size() == 4);
    int mine = 0, near = 0;
    for (const SpotLight& s : sps) {
        if (s.position.x >= 80.0) ++mine;
        if (s.position.x <= 6.0) ++near;
        CHECK(s.intensity == 20.0f);   // the pool does not fade
    }
    CHECK(mine == 2 && near == 2);
    CHECK(pts.empty());
}

TEST_CASE(light_picker_lights_a_pool_less_kind_always_and_rides_the_dusk) {
    const LightBook b = testBook();
    std::vector<LightCandidate> c = {at(2, 5000, 0, true), at(0, 10)};
    c[0].range = 12.0f;
    c[1].scale = 0.5f;
    std::vector<PointLight> pts;
    std::vector<SpotLight> sps;
    pickLights(b, b.pools, c, Vec3(0, 0, 0), 0.5f, pts, sps);
    CHECK(sps.size() == 1);
    if (!sps.empty()) CHECK(sps[0].range == 12.0f && sps[0].intensity == 100.0f);   // its own range; no dusk
    CHECK(pts.size() == 1);
    if (!pts.empty()) CHECK(std::fabs(pts[0].intensity - 10.0f * 0.5f * 0.5f) < 1e-5f);   // scale x dusk
    // by day the dusk kinds come out dark and are left out
    pts.clear();
    sps.clear();
    pickLights(b, b.pools, c, Vec3(0, 0, 0), 0.0f, pts, sps);
    CHECK(pts.empty() && sps.size() == 1);
}

TEST_CASE(light_picker_trims_to_the_renderers_capacity_by_importance) {
    const LightBook b = testBook();
    std::vector<LightPool> pools = b.pools;
    pools[0].count = 100;   // the street pool would light all of them
    std::vector<LightCandidate> c;
    for (int i = 0; i < 10; ++i) c.push_back(at(0, 5 + 8 * i));   // lamps at 5, 13, .. 77 m
    c.push_back(at(2, 300, 0, true));                               // the player's torch, far off: still kept
    std::vector<PointLight> pts;
    std::vector<SpotLight> sps;
    pickLights(b, pools, c, Vec3(0, 0, 0), 1.0f, pts, sps, 4);
    CHECK(pts.size() + sps.size() == 4);
    CHECK(sps.size() == 1);   // the player's own first
    for (const PointLight& p : pts) CHECK(p.position.x <= 21.0);   // then the brightest where the camera is: the nearest
    CHECK(lightImportance(10.0f, 30.0f, 31.0f) == 0.0f);          // past its reach it lights nothing here
    CHECK(lightImportance(10.0f, 30.0f, 0.2f) == 10.0f);          // nearer than a metre counts as one
}

TEST_CASE(lights_lua_gives_every_pool_a_many_light_budget) {
    LightBook book;
    std::string err;
    CHECK(loadLightsSource(readLightsFile(), book, &err));
    for (const LightPool& p : book.pools) {
        CHECK(p.hasMany);
        CHECK(p.manyCount >= p.count);   // a clustered renderer never lights fewer than a forward one
    }
    for (const char* name : {"tail_light", "brake_light"}) CHECK(book.kind(name) >= 0);
    // the reflection probes' rules (ADR-0374) are read, and sane
    CHECK(book.probes.spacing >= 5.0 && book.probes.height > 0.0 && book.probes.above > 0.0);
    LightBook bad;
    CHECK(!loadLightsSource("return { probes = { spacing = 1 } }", bad, &err));
    // the ray-traced reflections' rules (ADR-0376): read, every hit a draw class's name, the memory share bounded
    CHECK(book.raytrace.near > 0.0 && book.raytrace.fade <= book.raytrace.near);
    CHECK(book.raytrace.memoryShare > 0.0 && book.raytrace.memoryShare <= 0.5);
    CHECK(!book.raytrace.hits.empty());
    for (const std::string& h : book.raytrace.hits) {
        bool known = false;
        for (int c = 0; c < static_cast<int>(DrawClass::Count); ++c)
            known = known || (drawClassName(static_cast<DrawClass>(c)) && h == drawClassName(static_cast<DrawClass>(c)));
        CHECK(known);
    }
    CHECK(!loadLightsSource("return { raytrace = { memory_share = 0.9 } }", bad, &err));
}

// THE REFLECTION PROBES' SITES (ADR-0374): along a street every `spacing` metres at `height` over the carriageway (a
// deck's own height when it is one), one site where two streets meet, and each box holding its stretch of street and
// the frontages either side.
TEST_CASE(probe_sites_line_the_streets) {
    RoadGraph g;
    RoadNode a, b, c, d;
    a.pos = Vec2(0, 0);
    b.pos = Vec2(200, 0);
    c.pos = Vec2(0, 160);
    d.pos = Vec2(400, 0);
    d.elev = 12.0;   // a ramp up to a deck
    d.elevAbsolute = true;
    g.nodes = {a, b, c, d};
    RoadEdge ab, ac, bd;
    ab.a = 0; ab.b = 1; ab.width = 10;
    ac.a = 0; ac.b = 2; ac.width = 10;
    bd.a = 1; bd.b = 3; bd.width = 10;
    g.edges = {ab, ac, bd};
    const ProbeGroundFn ground = [](double, double) { return 2.0; };
    ProbeRules rules;
    std::vector<ProbeSite> sites;
    placeProbeSites(g, ground, rules, sites);
    CHECK(sites.size() == 5u + 4u + 5u);
    for (const ProbeSite& s : sites) {
        CHECK(s.boxMin.x < s.position.x && s.position.x < s.boxMax.x);
        CHECK(s.boxMin.z < s.position.z && s.position.z < s.boxMax.z);
        if (s.position.x < 200.0 && s.position.z == 0.0) {
            CHECK(std::fabs(s.position.y - (2.0 + rules.height)) < 1e-9);   // at grade: the ground's
            CHECK(s.boxMax.z - s.boxMin.z > 10.0 + 2 * rules.side - 1e-6);  // past both kerbs to the frontages
        }
    }
    // the ramp: from the ground at b (2) to the deck at d (12), 5 sites at 0.1 .. 0.9 of the way
    int ramp = 0;
    for (const ProbeSite& s : sites)
        if (s.position.x > 200.0) {
            ++ramp;
            const double t = (s.position.x - 200.0) / 200.0;
            CHECK(std::fabs(s.position.y - (2.0 + 10.0 * t + rules.height)) < 1e-6);
        }
    CHECK(ramp == 5);
    // the junction: a short arm's middle lands within `merge` of the other's: one site
    RoadGraph j;
    RoadNode o, e, n;
    o.pos = Vec2(0, 0);
    e.pos = Vec2(20, 0);
    n.pos = Vec2(0, 20);
    j.nodes = {o, e, n};
    RoadEdge oe, on;
    oe.a = 0; oe.b = 1;
    on.a = 0; on.b = 2;
    j.edges = {oe, on};
    std::vector<ProbeSite> js;
    placeProbeSites(j, ground, rules, js);
    CHECK(js.size() == 1u);
}

// THE NEAREST 64, STABLY (ADR-0374): a slot keeps its probe while it stays near (within the hysteresis past the
// nearest set's edge), so walking down a street re-captures only the probes it passes.
TEST_CASE(probe_residency_is_the_nearest_and_keeps_its_slots) {
    std::vector<ProbeSite> sites;
    for (int i = 0; i < 100; ++i) {
        ProbeSite s;
        s.position = Vec3(i * 40.0, 5.0, 0.0);
        sites.push_back(s);
    }
    std::vector<int> resident(8, -1);
    CHECK(pickResidentProbes(sites, Vec3(0, 0, 0), 15.0, resident));
    for (int r : resident) CHECK(r >= 0 && r < 8);   // the nearest 8
    CHECK(!pickResidentProbes(sites, Vec3(0, 0, 0), 15.0, resident));   // nothing moved: nothing changes
    const std::vector<int> before = resident;
    // a little way on: the edge moves 10 m, inside the hysteresis -- still nothing changes
    CHECK(!pickResidentProbes(sites, Vec3(10, 0, 0), 15.0, resident));
    // 300 m on: the four farthest behind leave, the four new ones ahead take their slots; the rest keep theirs
    CHECK(pickResidentProbes(sites, Vec3(300, 0, 0), 15.0, resident));
    int kept = 0;
    for (std::size_t i = 0; i < resident.size(); ++i) {
        CHECK(resident[i] >= 0);
        if (resident[i] == before[i]) ++kept;
        const double d = std::fabs(sites[static_cast<std::size_t>(resident[i])].position.x - 300.0);
        CHECK(d <= 140.0 + 1e-9);   // all eight now the nearest (20, 60, 100, 140 m either side)
    }
    CHECK(kept == 4);
    // the level's sites gone: every slot empties
    std::vector<ProbeSite> none;
    CHECK(pickResidentProbes(none, Vec3(0, 0, 0), 15.0, resident));
    for (int r : resident) CHECK(r == -1);
}

// A STOREY'S PROBES (ADR-0380): a grid over the footprint (an oriented box), one probe a cell at its height over the
// floor, each box the cell's from floor to ceiling, every one marked indoor -- and a footprint smaller than a cell
// still gets one.
TEST_CASE(storey_probes_grid_the_floor) {
    const Vec2 axis[2] = {Vec2(1, 0), Vec2(0, 1)};
    const Real half[2] = {20, 12};
    std::vector<ProbeSite> s;
    placeStoreyProbeSites(Vec2(100, 50), axis, half, 30.0, 34.0, 8.0, 1.6, s);
    CHECK(s.size() == 5u * 3u);   // 40 m / 8 = 5 across, 24 m / 8 = 3 deep
    for (const ProbeSite& p : s) {
        CHECK(p.indoor);
        CHECK(std::fabs(p.position.y - 31.6) < 1e-9);
        CHECK(p.boxMin.y < 30.0 && p.boxMax.y > 34.0);
        CHECK(p.position.x > 80.0 && p.position.x < 120.0 && p.position.z > 38.0 && p.position.z < 62.0);
        CHECK(p.boxMin.x < p.position.x && p.position.x < p.boxMax.x);
    }
    // turned 45 degrees: the boxes are the turned cells' bounds, still holding their probes
    const Real r = std::sqrt(0.5);
    const Vec2 turned[2] = {Vec2(r, r), Vec2(-r, r)};
    std::vector<ProbeSite> t;
    placeStoreyProbeSites(Vec2(0, 0), turned, half, 0.0, 3.5, 8.0, 1.6, t);
    CHECK(t.size() == 15u);
    for (const ProbeSite& p : t) CHECK(p.boxMin.z < p.position.z && p.position.z < p.boxMax.z);
    // smaller than a cell: one, in the middle; a low ceiling keeps it under the ceiling
    const Real tiny[2] = {2, 1.5};
    std::vector<ProbeSite> u;
    placeStoreyProbeSites(Vec2(5, 5), axis, tiny, 10.0, 11.5, 8.0, 1.6, u);
    CHECK(u.size() == 1u);
    CHECK(u[0].position.y <= 11.5 - 0.3 + 1e-9);
}
