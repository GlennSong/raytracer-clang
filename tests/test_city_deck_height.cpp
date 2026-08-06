// THE DECK-HEIGHT GATE (#25: "parked cars sunk into the street... hovering at
// all kinds of different heights... the parking markers floating above the
// road").
//
// The city render bridge places a lot of things ON the road: parked cars, the
// painted outlines of their bays, crosswalk decals, and at-grade traffic. It
// used to take their height from the RoadNet's own `heightAt` sampler — which,
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
RoadNet slopedCross() {
    RoadNet net;
    net.nodes = { Vec2(0, 0), Vec2(0, 140), Vec2(0, -140), Vec2(140, 0),
                  Vec2(-140, 0) };
    net.edges = { {0, 1}, {0, 2}, {0, 3}, {0, 4} };
    net.specs = { roadSpecStreetParking(12.0, 1, 3.5, 0.25) };
    net.edgeSpecs.assign(net.edges.size(), 0);
    net.width = net.specs[0].carriagewayWidth();
    net.sidewalk = 2.5;
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

    RoadNet net = slopedCross();
    net.heightAt = naturalGround;   // the loader leaves the NATURAL sampler here

    // 2. Carve the terrain to the road, exactly as the level loader does.
    TerrainParams carved = natural;
    carved.flatten = roadNetConformRegions(net);
    CHECK(!carved.flatten.empty());
    rebuildFlattenIndex(carved);

    // 3. A world holding the road and the carved CDLOD terrain config.
    World world;
    world.add<RoadNet>(world.create(), net);
    TerrainLodConfig lod;
    lod.params = carved;
    lod.seed = 7u;
    world.add<TerrainLodConfig>(world.create(), lod);

    CityRenderParams p;
    p.cars = 6;
    p.pedestrians = 0;
    p.seed = 3;
    p.vehicleScript = readAsset("vehicles.lua");
    CHECK(!p.vehicleScript.empty());
    CityRenderSystem city(p);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    CHECK(city.build(world, &assets));
    // A PARKED car is a SimVehicle whose driver has left it — not a bay flag —
    // so the fixture has to run until somebody actually parks. Drivers rest on
    // arrival, which on this cross takes a few in-world minutes.
    int parkedSeen = 0;
    for (int i = 0; i < 6000 && parkedSeen == 0; ++i) {
        city.step(world, 0.1);
        for (const SimVehicle& sv : city.sim().vehicles())
            if (sv.driver < 0) { ++parkedSeen; break; }
    }
    CHECK(parkedSeen > 0);

    // 4. The road mesh: the surface a player actually stands on.
    const RenderMesh roadMesh = buildRoadNetMesh(net);
    CHECK(!roadMesh.vertices.empty());

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
    const std::vector<Vec3> he = city.carGroupHalfExtents();
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

    std::printf("    [deck] bays=%d naturalVsDeck=%.2fm worstPaint=%.3fm "
                "cars=%d worstCar=%.3fm\n",
                sampled, worstNaturalError, worstPaint, carsChecked, worstCar);

    // The fixture must actually be hilly enough to expose the bug: if the raw
    // ground and the deck agree everywhere, this test proves nothing.
    CHECK(worstNaturalError > 0.5);
    // The road slab stands `net.lift` (8 cm) proud of the carved ground, so the
    // bridge's placements land within roughly that of the mesh surface.
    CHECK(worstPaint < 0.30);
    CHECK(carsChecked > 0);    // the scenery arm must not pass vacuously
    CHECK(worstCar < 0.30);
}
