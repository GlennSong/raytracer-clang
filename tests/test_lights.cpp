// THE ONE LIGHT SYSTEM (ADR-0362): lights.lua reads into kinds and pools (and a broken file is refused), and the one
// picker chooses as every producer used to on its own -- per pool the player's own first, then the nearest in reach
// up to the count, a car's lamps together, the set fading toward its moving edge, the dusk ramp on the kinds that
// ride it, a kind with no pool always lit.

#include "test_framework.h"
#include "../src/engine/lights/light_picker.h"
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
    for (const char* name : {"street_lamp", "station_lamp", "headlight", "flashlight", "room", "cab", "beacon"})
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
}
