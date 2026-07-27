// Curbside parking (roads-v2.1 R6b, plan 4d phase 1): marked parallel bays
// mid-link on at-grade Local streets, seeded ~half full with scenery cars;
// an arriving driver claims a free bay on its arrival link (the old grass
// verge remains the fallback), and pulling out frees it. Gates, per the
// plan: bays never crowd junction mouths, arrivals really park and depart,
// scenery bays are never stolen, and no bay ever holds two cars.
#include "test_framework.h"

#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/road_spec.h"
#include "city_test_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <nlohmann/json.hpp>

using namespace engine;
using namespace citysim;

namespace {

// A wide 4-way cross (12 m carriageways, 100 m arms) whose streets carry a REAL
// cross-section: sidewalk | curb | parking 2.5 | travel 3.5 | travel 3.5 |
// parking 2.5 | curb | sidewalk. The Parking band is what makes a road park-able
// now (the old "Local and >= 9 m wide" heuristic is gone), so the fixture has to
// resolve one exactly the way a generated street does.
NavGraph wideCross() {
    const RoadSpec spec = roadSpecStreetParking(12.0, 1, 3.5, 0.25);
    double off = 0, w = 0;
    roadSpecParkingBand(spec, +1, off, w);
    RoadGraph g;
    g.nodes = { { Vec2(0, 0) }, { Vec2(0, 100) }, { Vec2(0, -100) },
                { Vec2(100, 0) }, { Vec2(-100, 0) } };
    for (int arm = 1; arm <= 4; ++arm) {
        RoadEdge e{ 0, arm, static_cast<Real>(spec.carriagewayWidth()),
                    RoadClass::Local, 0 };
        e.parkOffset = static_cast<Real>(off);
        e.parkWidth = static_cast<Real>(w);
        g.edges.push_back(e);
    }
    return buildNavGraph(g);
}

}  // namespace

TEST_CASE(parking_bays_stay_clear_of_junction_mouths) {
    NavGraph nav = wideCross();
    CitySim sim;
    sim.build(nav, 0, 0, 5);
    const auto& bays = sim.parkingBays();
    CHECK(!bays.empty());

    int scenery = 0;
    for (const CitySim::ParkingBay& b : bays) {
        // Clear of every junction node (the mouth machinery owns that zone).
        for (int ni = 0; ni < nav.nodeCount(); ++ni)
            CHECK((b.pos - nav.nodes[ni]).length() >= 12.0);
        // Sits in the road's PARKING BAND: the bay centre is the band centre,
        // so the whole stall lies between the last travel lane and the kerb.
        const NavLink& L = nav.links[b.link];
        const Vec2 dir = nav.direction(b.link);
        const Vec2 rel = b.pos - nav.nodes[L.from];
        const Real lat = rel.x * dir.y - rel.y * dir.x;   // + = right of dir
        CHECK_APPROX(lat, L.parkOffset, 1e-9);
        CHECK_APPROX(b.width, L.parkWidth, 1e-9);
        CHECK(lat - b.width * 0.5 >= L.width * 0.5 - L.parkWidth - 1e-9);
        CHECK(lat + b.width * 0.5 <= L.width * 0.5 + 1e-9);
        if (b.occupant == CitySim::kBayScenery) ++scenery;
    }
    const Real frac = Real(scenery) / Real(bays.size());
    std::printf("[parking] bays=%zu sceneryFrac=%.2f\n", bays.size(), frac);
    CHECK(frac > 0.3);
    CHECK(frac < 0.8);
}

TEST_CASE(parking_arrivals_claim_bays_and_departures_free_them) {
    NavGraph nav = wideCross();
    CitySim sim;
    sim.build(nav, 10, 0, 9);   // scheduled day: commutes arrive + rest

    long parkTicks = 0, freedAfterPark = 0;
    bool sceneryStolen = false, doubleParked = false, wrongPose = false;
    std::vector<int> lastOccupant(sim.parkingBays().size(), -1);
    for (std::size_t i = 0; i < sim.parkingBays().size(); ++i)
        lastOccupant[i] = sim.parkingBays()[i].occupant;

    for (int i = 0; i < 6000; ++i) {
        sim.step(0.1, 0.5);
        const auto& ag = sim.agents();
        const auto& bays = sim.parkingBays();
        std::vector<int> users(bays.size(), 0);
        for (std::size_t ai = 0; ai < ag.size(); ++ai) {
            const Agent& a = ag[ai];
            if (a.parkedBay < 0) continue;
            ++parkTicks;
            ++users[a.parkedBay];
            const CitySim::ParkingBay& b = bays[a.parkedBay];
            if (b.occupant != static_cast<int>(ai)) doubleParked = true;
            if ((a.pos - b.pos).length() > 0.5) wrongPose = true;
            if (a.moving) wrongPose = true;
        }
        for (std::size_t bi = 0; bi < bays.size(); ++bi) {
            if (users[bi] > 1) doubleParked = true;
            if (lastOccupant[bi] == CitySim::kBayScenery &&
                bays[bi].occupant != CitySim::kBayScenery)
                sceneryStolen = true;
            // A bay that held an agent and is free again = a real departure.
            if (lastOccupant[bi] >= 0 && bays[bi].occupant == -1)
                ++freedAfterPark;
            lastOccupant[bi] = bays[bi].occupant;
        }
    }
    std::printf("[parking] parkTicks=%ld departures=%ld\n", parkTicks,
                freedAfterPark);
    CHECK(parkTicks > 0);          // drivers really rest in bays
    CHECK(freedAfterPark > 0);     // and pull out again
    CHECK(!sceneryStolen);
    CHECK(!doubleParked);
    CHECK(!wrongPose);
}

// THE REGRESSION THAT WOULD HAVE CAUGHT "half on the sidewalk" (parking round).
// Run the REAL recipe pipeline — applyGenerateRecipe, its street specs, the
// constrained nav graph the sim actually drives — and demand that every bay it
// mints lives inside the Parking band of its own road:
//   * the road it sits on really HAS a Parking band (never a bare travel lane);
//   * the painted stall lies between the last travel lane and the kerb;
//   * the widest car in the fleet still fits inside the band;
//   * the width the mesher/lots/nav read equals the spec's carriageway width,
//     so a widened road can never swallow the ground buildings kept clear of.
TEST_CASE(parking_bays_live_inside_the_road_s_parking_band) {
    // piedmont_mini's recipe, trimmed of the terrain knobs (no heightAt here).
    nlohmann::json recipe = nlohmann::json::parse(R"({
        "kind": "metro", "seed": 7, "backbone": "arterial", "freeways": false,
        "artery_width": 17, "collector_width": 13, "street_width": 12.0,
        "block_size": 220, "min_block_edge": 150, "min_road_len": 30,
        "min_road_clearance": 0.9, "seg_length": 120, "influence": 800,
        "kill_radius": 300, "merge_radius": 200, "corridor_spacing": 240,
        "ambient_per_500": 5, "loop_min": 550, "loop_max": 1300,
        "curviness": 0.25,
        "sites": [
          { "center": { "x": 0, "z": 0 }, "radius": 700, "hotspots": 6,
            "block_size": 220, "density": 1.0 },
          { "center": { "x": 950, "z": -200 }, "radius": 250, "hotspots": 2,
            "block_size": 180, "density": 0.6, "kind_bias": "oldtown" }
        ]
    })");
    RoadNet net;
    net.width = 12.0;
    net.sidewalk = 3.5;
    net.curb = 0.16;
    applyGenerateRecipe(net, recipe);
    CHECK(!net.edges.empty());
    CHECK(!net.specs.empty());

    // (B) ONE source of truth for width: every edge that got a spec carries that
    // spec's carriageway width, which is what roadNetEdgeWidth hands the mesher,
    // the lot clearance pass and the nav graph.
    int specced = 0, withPark = 0;
    for (std::size_t ei = 0; ei < net.edges.size(); ++ei) {
        const int si = net.edgeSpecs[ei];
        const RoadClass k = net.edgeClasses[ei];
        // Arterials and boulevards keep today's look: no spec, no parking band.
        if (k != RoadClass::Local && k != RoadClass::Collector) CHECK(si < 0);
        if (si < 0) continue;
        ++specced;
        const RoadSpec& s = net.specs[si];
        CHECK_APPROX(roadNetEdgeWidth(net, static_cast<int>(ei)),
                     s.carriagewayWidth(), 1e-9);
        CHECK_APPROX(roadNetEdgeSpec(net, static_cast<int>(ei)).carriagewayWidth(),
                     s.carriagewayWidth(), 1e-9);
        double off = 0, bw = 0;
        if (roadSpecParkingBand(s, +1, off, bw)) ++withPark;
    }
    CHECK(specced > 0);
    CHECK(withPark > 0);

    // A generated net saves as its RECIPE (roadRecipeForSave keeps the "generate"
    // block and never bakes nodes/edges), so the specs must come back by
    // REGENERATION — same table, same per-edge index, same widths.
    {
        nlohmann::json doc;
        doc["generate"] = recipe;
        const nlohmann::json saved = roadRecipeForSave(doc.dump(), net);
        CHECK(saved.contains("generate"));
        RoadNet again = roadNetFromJson(saved);
        applyGenerateRecipe(again, saved["generate"]);
        CHECK(again.specs.size() == net.specs.size());
        CHECK(again.edgeSpecs == net.edgeSpecs);
        CHECK(again.edgeWidths == net.edgeWidths);
        for (std::size_t i = 0; i < net.specs.size(); ++i)
            CHECK_APPROX(again.specs[i].carriagewayWidth(),
                         net.specs[i].carriagewayWidth(), 1e-9);
    }

    NavGraph nav = buildNavGraph(navRoadGraph(net));
    CitySim sim;
    sim.build(nav, 0, 0, 5);
    const auto& bays = sim.parkingBays();
    std::printf("[parking] generated: %d specced edges, %zu specs, %zu bays\n",
                specced, net.specs.size(), bays.size());
    CHECK(bays.size() >= 40u);   // the streets really are parked-along

    Real widestCar = 0;
    for (int i = 0; i < vehicleFleetSize(); ++i)
        widestCar = std::max(widestCar, vehicleFleetBody(i).width);

    Real worstOut = 0, worstIn = 0;
    for (const CitySim::ParkingBay& b : bays) {
        const NavLink& L = nav.links[b.link];
        CHECK(L.parkWidth > 0);                       // never a bare travel lane
        CHECK_APPROX(b.width, L.parkWidth, 1e-9);
        const Vec2 dir = nav.direction(b.link);
        const Vec2 rel = b.pos - nav.nodes[L.from];
        const Real lat = rel.x * dir.y - rel.y * dir.x;   // + = right of travel
        const Real edge = L.width * 0.5;                  // kerb line
        const Real inner = edge - L.parkWidth;            // travel-lane boundary
        // Bay centre IS the band centre; the stall never crosses either side.
        CHECK_APPROX(lat, (inner + edge) * 0.5, 1e-6);
        worstOut = std::max(worstOut, lat + b.width * 0.5 - edge);
        worstIn = std::max(worstIn, inner - (lat - b.width * 0.5));
        // ...and a real car parked on that centre fits in the band too.
        CHECK(lat + widestCar * 0.5 <= edge + 1e-9);
        CHECK(lat - widestCar * 0.5 >= inner - 1e-9);
    }
    std::printf("[parking] widest fleet car %.2f m; worst overhang: kerb %.4f m, "
                "travel lane %.4f m\n", widestCar, worstOut, worstIn);
    CHECK(worstOut <= 1e-9);   // NOT half on the sidewalk
    CHECK(worstIn <= 1e-9);    // NOT half in the travel lane
}
