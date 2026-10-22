// THE DECK-HEIGHT GATE (#25: "parked cars sunk into the street... hovering at
// all kinds of different heights... the parking markers floating above the
// road").
//
// The city render bridge places a lot of things ON the road: parked cars, the
// painted outlines of their bays, crosswalk decals, and at-grade traffic. It
// used to take their height from the RoadEntity's own `heightAt` sampler — which,
// for a road the level graded its terrain to, is deliberately the NATURAL,
// pre-carve ground. On a hill that is the bare hillside, metres away from the
// asphalt the mesher actually built, so parked cars sank into cuts and hovered
// over fills.
//
// This test builds one road on real relief, carves the terrain to it exactly as
// the level loader does, and then compares what the bridge places against the
// height of the ROAD MESH itself — two independent producers, sampled at the
// same spot. Flat ground would hide the whole bug, so the fixture is a slope.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/apps/citysim/city_render.h"
#include "../src/engine/components.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/procgen/city/road_spec.h"
#include "../src/engine/procgen/noise.h"
#include "../src/engine/procgen/terrain.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/world.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

using namespace engine;
using namespace citysim;

namespace {

// A wide 4-way cross with long arms, carrying a REAL street cross-section:
// sidewalk | curb | parking 2.5 | travel 3.5 | travel 3.5 | parking 2.5 | curb |
// sidewalk. The Parking BAND is what makes a road park-able (the old "Local and
// >= 9 m wide" heuristic is gone), so the fixture resolves one exactly the way a
// generated street does — same as tests/test_city_parking.cpp.
RoadEntity slopedCross() {
    RoadEntity net;
    net.graph.specs = { roadSpecStreetParking(12.0, 1, 3.5, 0.25) };
    net.look.defaultWidth = net.graph.specs[0].carriagewayWidth();
    net.look.sidewalk = 2.5;
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(0, 140)},
                        RoadNode{Vec2(0, -140)}, RoadNode{Vec2(140, 0)},
                        RoadNode{Vec2(-140, 0)} };
    for (int arm = 1; arm <= 4; ++arm) {
        RoadEdge e{0, arm, static_cast<Real>(net.look.defaultWidth),
                   RoadClass::Local, 0};
        e.spec = 0;
        net.graph.edges.push_back(e);
    }
    return net;
}

// Real relief, not a plane: a broad rise plus a cross-ripple, so the road's
// smoothed, grade-limited profile and the raw ground disagree by metres in
// places and by centimetres in others.
TerrainParams hillParams() {
    TerrainParams p;
    p.heightScale = 40.0;
    p.noiseScale = 0.0022;
    p.octaves = 4;
    return p;
}

// The shipped vehicle catalogue. Cars are CONTENT: a city with no recipes draws
// none at all (the built-in box fleet is gone), so the scenery arm of this gate
// has to say which cars — exactly as a level does.
std::string readAsset(const std::string& name) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/" + name);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// A stub mesh backend: the fleet must actually be built for parked cars to
// exist, and building needs somewhere to upload to. No GPU.
struct StubUploader : engine::MeshUploader {
    uint32_t next = 1;
    engine::MeshHandle uploadMesh(const engine::RenderMesh&) override {
        return engine::MeshHandle{next++, 1};
    }
    void removeMesh(engine::MeshHandle) override {}
    engine::BoundingSphere getMeshBounds(engine::MeshHandle) const override { return {}; }
};

// The highest road surface under (x, z), or NaN where the mesh has none.
double meshDeckAt(const RenderMesh& mesh, double x, double z) {
    std::vector<double> hits = driveprobe::surfacesAt(mesh, x, z);
    if (hits.empty()) return std::nan("");
    return *std::max_element(hits.begin(), hits.end());
}

}  // namespace


// #35: traffic on a ramp asks for the drawn deck NEAR its own level. A ramp crests over a street (the
// ramp's drawn profile is a curve; its nav link only knows the two ends, so the straight line between them
// cuts under the crest). heightNear must find the ramp's curve from the lerped height, never the street
// below; heightAt (for things at grade) still answers the street.
TEST_CASE(deck_field_height_near_finds_the_ramp_curve_not_the_street_under_it) {
    using namespace engine;
    RoadDeckField d;
    {   // the street: flat at y = 0, along x
        UnionSpine s;
        for (int i = 0; i <= 10; ++i) { s.points.push_back(Vec2(i * 10.0, 0.0)); s.yAbs.push_back(0.0); s.hw.push_back(4.0); }
        s.halfWidth = 4.0;
        s.layer = 0;
        d.spines.push_back(std::move(s));
    }
    {   // the ramp over it: ends at 6 m, cresting at 7 m mid-way (a parabola)
        UnionSpine s;
        for (int i = 0; i <= 10; ++i) {
            const double u = i / 10.0;
            s.points.push_back(Vec2(i * 10.0, 0.0));
            s.yAbs.push_back(6.0 + 4.0 * u * (1.0 - u));
            s.hw.push_back(4.0);
        }
        s.halfWidth = 4.0;
        s.layer = 1;
        d.spines.push_back(std::move(s));
    }
    d.buildIndex();
    double y = -1;
    // the nav lerp says 6.0 at the crest; the drawn ramp is at 7.0 there
    CHECK(d.heightNear(50.0, 0.0, 0.5, 6.0, 2.0, &y));
    CHECK_APPROX(y, 7.0, 1e-6);
    // a car at the street's level finds the street
    CHECK(d.heightNear(50.0, 0.0, 0.5, 0.3, 2.0, &y));
    CHECK_APPROX(y, 0.0, 1e-6);
    // nothing within the window: no answer (the caller keeps its own height)
    CHECK(!d.heightNear(50.0, 0.0, 0.5, 3.5, 2.0, &y));
    // off every road: no answer
    CHECK(!d.heightNear(50.0, 30.0, 0.5, 6.0, 2.0, &y));
    // heightAt is unchanged: the lower layer (the street) wins
    CHECK(d.heightAt(50.0, 0.0, 0.5, &y));
    CHECK_APPROX(y, 0.0, 1e-6);
}

// #35, measured on island 8: a ramp runs alongside the freeway and their drawn decks overlap at the edge.
// The car's lerped height (1.46 m under the ramp's crest) was nearer the FREEWAY's surface, and a
// nearest-height rule put the car on the freeway, 1.3 m inside the ramp. The road the car is inside wins.
TEST_CASE(deck_field_height_near_prefers_the_road_the_car_is_inside_over_a_nearer_height) {
    using namespace engine;
    RoadDeckField d;
    auto straight = [&](double z, double y, double hw, int layer) {
        UnionSpine s;
        for (int i = 0; i <= 10; ++i) { s.points.push_back(Vec2(i * 10.0, z)); s.yAbs.push_back(y); s.hw.push_back(hw); }
        s.halfWidth = hw;
        s.layer = layer;
        d.spines.push_back(std::move(s));
    };
    straight(0.0, 28.41, 4.75, 1);    // the ramp, the car in its lane
    straight(-12.0, 27.00, 8.0, 1);   // the freeway beside it; its edge reaches under the ramp's lane
    d.buildIndex();
    double y = -1;
    // the car 1.5 m right of the ramp's centre: inside both decks (freeway edge at z = -4 + 0.5 margin)
    CHECK(d.heightNear(50.0, -3.8, 0.5, 26.95, 3.0, &y));
    CHECK_APPROX(y, 28.41, 1e-6);
    // on the freeway proper it is the freeway
    CHECK(d.heightNear(50.0, -12.0, 0.5, 27.1, 3.0, &y));
    CHECK_APPROX(y, 27.00, 1e-6);
}

// Wheel lab: in a lanes scene the street and the freeway over it are both layer 1, so "the lower layer
// wins" cannot tell them apart and a street car under the overpass was drawn 8.6 m up on it. A car's own
// road CLASS picks the road; a junction pad counts only near that road's height.
TEST_CASE(deck_field_height_on_picks_the_car_s_own_road_class_under_an_overpass) {
    using namespace engine;
    RoadDeckField d;
    auto straight = [&](Vec2 a, Vec2 b, double y, double hw, RoadClass k) {
        UnionSpine s;
        for (int i = 0; i <= 10; ++i) { s.points.push_back(a + (b - a) * (i / 10.0)); s.yAbs.push_back(y); s.hw.push_back(hw); }
        s.halfWidth = hw;
        s.layer = 1;
        s.klass = k;
        d.spines.push_back(std::move(s));
    };
    straight(Vec2(0, -50), Vec2(0, 50), 0.2, 6.0, RoadClass::Arterial);    // the street
    straight(Vec2(-50, 1), Vec2(50, 1), 8.8, 10.0, RoadClass::Freeway);    // the freeway over it, nearer the car
    d.pads.push_back(RoadDeckField::Tri{Vec3(-5, 8.8, -5), Vec3(5, 8.8, -5), Vec3(0, 8.8, 5)});   // a freeway pad above
    d.buildIndex();
    const RoadClass art = RoadClass::Arterial, fwy = RoadClass::Freeway;
    double y = -1;
    CHECK(d.heightOn(0.5, 0.8, 0.5, &art, 0.0, 1e9, &y));   // a street car, no height to go by
    CHECK_APPROX(y, 0.2, 1e-6);
    CHECK(d.heightOn(0.5, 0.8, 0.5, &fwy, 8.5, 3.0, &y));   // a freeway car on the deck above
    CHECK_APPROX(y, 8.8, 1e-6);
}
