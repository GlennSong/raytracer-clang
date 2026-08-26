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
#include "../src/engine/procgen/city/road_net.h"
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

TEST_CASE(parked_cars_and_bay_paint_sit_on_the_road_deck) {
    // 1. Natural ground, exactly what the loader hands a pre-pass road.
    TerrainParams natural = hillParams();
    auto naturalNoise = std::make_shared<Noise>(7u);
    auto naturalCopy = std::make_shared<TerrainParams>(natural);
    std::function<double(double, double)> naturalGround =
        [naturalCopy, naturalNoise](double x, double z) {
            return terrainHeight(*naturalCopy, *naturalNoise, x, z);
        };

    RoadEntity net = slopedCross();
    // (Roads no longer store a sampler: the NATURAL ground the loader knows is
    // passed to every consumer below, exactly as the loader now does.)

    // 2. Carve the terrain to the road, exactly as the level loader does.
    TerrainParams carved = natural;
    carved.flatten = roadNetConformRegions(net, naturalGround);
    CHECK(!carved.flatten.empty());
    rebuildFlattenIndex(carved);

    // 3. A world holding the road and the carved CDLOD terrain config.
    World world;
    world.add<RoadEntity>(world.create(), net);
    TerrainLodConfig lod;
    lod.params = carved;
    lod.seed = 7u;
    world.add<TerrainLodConfig>(world.create(), lod);
    // 4. The road mesh: the surface a player actually stands on — and the DECK
    // it rode, stored exactly as the loader stores it (RoadDeck). Built before
    // the run so the DRIVEN cars can be measured while they are driving (at
    // the end of the run every driver has reached a dead-end tip and pulled
    // off the carriageway to wait — off the mesh, nothing to measure).
    RoadDeckField deck;
    const RenderMesh roadMesh = buildRoadNetMesh(net, naturalGround, nullptr, &deck);
    CHECK(!roadMesh.vertices.empty());
    CHECK(!deck.empty());
    world.add<RoadDeck>(world.create(), RoadDeck{deck});

    CityRenderParams p;
    p.cars = 6;
    p.pedestrians = 0;
    p.seed = 3;
    p.vehicleScript = readAsset("vehicles.lua");
    CHECK(!p.vehicleScript.empty());
    CityRenderSystem city(p);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    // The NATURAL sampler goes to build the way the loader hands it on (the
    // constrained graph is built over it); the carved TerrainLodConfig above
    // still wins for everything the bridge PLACES (#25).
    CHECK(city.build(world, &assets, naturalGround));
    // A PARKED car is a SimVehicle whose driver has left it — not a bay flag —
    // so the fixture has to run until somebody actually parks. Drivers rest on
    // arrival, which on this cross takes a few in-world minutes.
    const std::vector<Vec3> he = city.carGroupHalfExtents();

    // 7 (accumulated below). Signed bottom-minus-deck for every drawn driver
    // that is ON the mesh, sampled every 25 steps: a sink and a hover cannot
    // cancel, the mean is the systematic term, the spread is the noise.
    double drivenWorst = 0, drivenSum = 0, drivenMin = 1e9, drivenMax = -1e9;
    int drivenChecked = 0, drivenSeen = 0, drivenNoDeck = 0;
    auto sampleDriven = [&]() {
        const auto& idsNow = city.carAgentIds();
        for (std::size_t v = 0; v < city.carGroups().size(); ++v) {
            InstanceGroup* g = world.get<InstanceGroup>(city.carGroups()[v]);
            if (!g || v >= idsNow.size()) continue;
            for (std::size_t i = 0; i < g->transforms.size() && i < idsNow[v].size(); ++i) {
                if (idsNow[v][i] < 0) continue;          // parked: arm 6 below
                ++drivenSeen;
                const Mat4& m = g->transforms[i];
                const double deck = meshDeckAt(roadMesh, m.m[0][3], m.m[2][3]);
                if (std::isnan(deck)) { ++drivenNoDeck; continue; }
                ++drivenChecked;
                const double bottom = m.m[1][3] - (v < he.size() ? he[v].y : 0.65);
                const double d = bottom - deck;
                drivenSum += d;
                drivenMin = std::min(drivenMin, d);
                drivenMax = std::max(drivenMax, d);
                drivenWorst = std::max(drivenWorst, std::fabs(d));
            }
        }
    };

    int parkedSeen = 0;
    for (int i = 0; i < 6000 && parkedSeen == 0; ++i) {
        city.step(world, 0.1);
        if (i % 25 == 0) sampleDriven();
        for (const SimVehicle& sv : city.sim().vehicles())
            if (sv.driver < 0) { ++parkedSeen; break; }
    }
    CHECK(parkedSeen > 0);

    // How far the natural sampler is from the deck at these bays — the size of
    // the error the bridge used to inherit. Reported so a future flattening of
    // the fixture (which would make this test vacuous) is visible.
    double worstNaturalError = 0;
    const auto& bays = city.sim().parkingBays();
    CHECK(!bays.empty());
    int sampled = 0;
    for (const CitySim::ParkingBay& b : bays) {
        const double deck = meshDeckAt(roadMesh, b.pos.x, b.pos.y);
        if (std::isnan(deck)) continue;
        ++sampled;
        worstNaturalError =
            std::max(worstNaturalError,
                     std::fabs(naturalGround(b.pos.x, b.pos.y) - deck));
    }
    CHECK(sampled > 0);

    // 5. The painted bay outlines ride the asphalt.
    double worstPaint = 0;
    int paintChecked = 0;
    if (InstanceGroup* g = world.get<InstanceGroup>(city.parkBayGroup())) {
        CHECK(g->transforms.size() == bays.size());
        for (std::size_t i = 0; i < g->transforms.size(); ++i) {
            const double deck = meshDeckAt(roadMesh, bays[i].pos.x, bays[i].pos.y);
            if (std::isnan(deck)) continue;
            ++paintChecked;
            // The outline is authored at deck + the mesher's own stripe lift
            // (kRoadMarkLift, 2 cm — the same one the lane paint uses); measure
            // the residual against the mesh.
            const double resid =
                std::fabs((g->transforms[i].m[1][3] - engine::kRoadMarkLift) - deck);
            worstPaint = std::max(worstPaint, resid);
        }
    }
    CHECK(paintChecked > 0);

    // 6. The parked cars rest wheels-on-deck. Their instances are the ones the
    // bake tagged with a NEGATIVE id (-2 - vehicleIndex); a driver carries its
    // agent index.
    double worstCar = 0;
    int carsChecked = 0;
    const auto& ids = city.carAgentIds();
    for (std::size_t v = 0; v < city.carGroups().size(); ++v) {
        InstanceGroup* g = world.get<InstanceGroup>(city.carGroups()[v]);
        if (!g || v >= ids.size()) continue;
        for (std::size_t i = 0; i < g->transforms.size() && i < ids[v].size(); ++i) {
            // A scenery parked car carries a NEGATIVE bake id (-2 - bayIndex,
            // stable across the distance cull); a real driver carries its agent
            // index.
            if (ids[v][i] >= 0) continue;
            const Mat4& m = g->transforms[i];
            const double deck = meshDeckAt(roadMesh, m.m[0][3], m.m[2][3]);
            if (std::isnan(deck)) continue;
            ++carsChecked;
            const double bottom =
                m.m[1][3] - (v < he.size() ? he[v].y : 0.65);
            worstCar = std::max(worstCar, std::fabs(bottom - deck));
        }
    }

    const double drivenMean = drivenChecked ? drivenSum / drivenChecked : 0.0;

    std::printf("    [deck] bays=%d naturalVsDeck=%.2fm worstPaint=%.3fm "
                "cars=%d worstCar=%.3fm\n",
                sampled, worstNaturalError, worstPaint, carsChecked, worstCar);
    std::printf("    [deck] driven=%d (seen %d, off-mesh %d) bottom-deck mean=%.3fm "
                "min=%.3fm max=%.3fm worst=%.3fm\n",
                drivenChecked, drivenSeen, drivenNoDeck, drivenMean, drivenMin,
                drivenMax, drivenWorst);

    // The fixture must actually be hilly enough to expose the bug: if the raw
    // ground and the deck agree everywhere, this test proves nothing.
    CHECK(worstNaturalError > 0.5);
    // Paint and parked cars are placed from the RoadDeck the mesh rode, so
    // they land ON the mesh surface (measured 0.000 m). The old 0.30 m bound
    // was sized to tolerate the 0.14 m sink this gate never noticed — the
    // terrain is carved 0.22 m under the deck and the bridge added a
    // vestigial 0.08 "lift" — and it must not be widened again.
    CHECK(worstPaint < 0.05);
    CHECK(carsChecked > 0);    // the scenery arm must not pass vacuously
    CHECK(worstCar < 0.05);
    // The driven arm: cars must not be IN the road. Bound both ways, tight:
    // a tyre is 0.31 m, so a 0.05 m sink is a visible flat spot and a 0.05 m
    // hover a visible shadow gap.
    CHECK(drivenChecked > 0);
    CHECK(drivenMin > -0.05);
    CHECK(drivenMax < 0.12);
}
