// HOW THE SIM SCALES (Glenn, 2026-09-16: "I'd like a prototype... see if we
// can push a few thousand agents in the world"). Before designing anything,
// measure: step a real CitySim headlessly at rising populations, flat and
// tiered, and print the cost per tick with the tier census beside it.
//
// Print-only by design. Wall-clock on a shared desktop is not a gate, and a
// timing assertion here would flake; the NUMBERS are the deliverable and
// they travel with the run.
#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/metro.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>

using namespace engine;
using citysim::CitySim;

namespace {

struct Row {
    int agents = 0;
    double msPerTick = 0;
    int k = 0, v = 0, d = 0, sleeping = 0;
};

Row run(const NavGraph& nav, int agents, bool tiered, int ticks) {
    CitySim sim;
    sim.build(nav, agents / 2, agents - agents / 2, 7);
    if (tiered) {
        sim.tieringEnabled = true;
        sim.dormancyEnabled = true;
        sim.setTierCenter(Vec2(0, 0));   // the player at the middle of the grid
    }
    // Warm: let the population place itself and the tier bubble settle before
    // the clock starts, so the measurement is steady state, not spawn.
    for (int i = 0; i < 30; ++i) sim.step(1.0 / 60.0, 0.05);

    sim.resetPhaseTimes();
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < ticks; ++i) sim.step(1.0 / 60.0, 0.05);
    const auto t1 = std::chrono::steady_clock::now();
    {   // WHERE the step time goes. The three passes left of the bar touch the
        // WHOLE population every tick regardless of tier; the three right of it
        // walk only the active list. Stage 1 of the 100k plan needs to know
        // which side dominates -- capping the sensing radius query at 50k moved
        // nothing, which killed two guesses.
        const auto& ph = sim.phaseTimes();
        const double n = ph.steps > 0 ? ph.steps : 1;
        std::printf("    [phase] %6d agents | rehash %8.1f tier %8.1f active %8.1f "
                    "| goals %8.1f gaps %8.1f advance %8.1f | total %9.1f us/step\n",
                    static_cast<int>(sim.agents().size()), ph.rehash / n, ph.tierPass / n,
                    ph.activeList / n, ph.goals / n, ph.gaps / n, ph.advance / n,
                    ph.total / n);
        std::printf("    [adv]   %6d agents | move %8.1f pairs %8.1f "
                    "pop %8.1f solver %8.1f tail %8.1f us/step\n",
                    static_cast<int>(sim.agents().size()), ph.advMove / n,
                    ph.advPairs / n, ph.advPop / n, ph.advSolver / n,
                    ph.advance / n);
    }

    Row r;
    r.agents = static_cast<int>(sim.agents().size());
    r.msPerTick = std::chrono::duration<double, std::milli>(t1 - t0).count() / ticks;
    for (const auto& a : sim.agents()) {
        if (a.tier == citysim::Agent::Tier::K) ++r.k;
        else if (a.tier == citysim::Agent::Tier::V) ++r.v;
        else ++r.d;
    }
    r.sleeping = sim.sleepingAgents();
    return r;
}

}  // namespace

TEST_CASE(sim_scale_census_prints) {
    // A 1.2 km grid on 120 m blocks: a small city, not a test stub.
    const NavGraph nav = citytest::cityNav(1200, 120, 11);
    std::printf("    [scale] nav: %zu nodes, %zu links\n", nav.nodes.size(), nav.links.size());
    std::printf("    [scale] %-7s %-7s %9s %7s %7s %7s %9s\n",
                "agents", "mode", "ms/tick", "K", "V", "D", "sleeping");
    // Glenn, 2026-09-16: "can we reach say 50k agents walking around?" The
    // answer has to be MEASURED, not extrapolated off the 4k row. Flat stops
    // at 4k -- it is the known-bad path and would dominate the runtime to
    // prove something already proven.
    for (int n : {500, 1000, 2000, 4000, 10000, 25000, 50000}) {
        for (int tiered = 0; tiered < 2; ++tiered) {
            if (n > 4000 && tiered == 0) continue;
            const Row r = run(nav, n, tiered == 1, 120);
            std::printf("    [scale] %-7d %-7s %9.3f %7d %7d %7d %9d\n",
                        r.agents, tiered ? "tiered" : "flat", r.msPerTick, r.k, r.v, r.d, r.sleeping);
            CHECK(r.agents > 0);
        }
    }
}


// METRO V2'S OWN NETWORK (Glenn, 2026-09-16: "enough to make metro v2 feel
// like a bustling urban environment"). The synthetic grid above answers how
// the sim scales; this answers what THIS city asks for. The parameters are
// metro_v2_test.json's own `generate` block, mapped the way road_net.cpp maps
// it, and the density rule is city_render.cpp's: lane-km x carsPerLaneKm and
// sidewalk-km x pedsPerKm, each currently clamped to 400.
//
// Heavy (it grows a 1 km metro), so it runs only under RT_SIM_SCALE=1.
TEST_CASE(metro_v2_density_target_prints) {
    if (!std::getenv("RT_SIM_SCALE")) return;
    MetroParams mp;
    mp.radius = 1075;
    mp.hotspots = 9;
    mp.blockSize = 180;
    mp.minBlockEdge = 120;
    mp.arteryWidth = 17;
    mp.collectorWidth = 13;
    mp.streetWidth = 12;
    mp.freewayWidth = 22;
    mp.freeways = true;                 // radius >= 550
    mp.corridorFreeways = true;
    mp.interchangeSpacing = 700;
    mp.segLength = 100;
    mp.influence = 700;
    mp.killRadius = 260;
    mp.mergeRadius = 170;
    mp.corridorSpacing = 200;
    mp.ambientPer500 = 40;
    mp.loopMin = 450;
    mp.loopMax = 1100;
    mp.fabric = "mix";
    mp.fabricCoreLen = 100;
    mp.skeleton = "footprint";
    mp.districtLen = 900;
    mp.gateSpacing = 900;
    mp.seed = 9;

    const auto tb = std::chrono::steady_clock::now();
    const RoadGraph g = buildMetro(mp, nullptr);   // freeway corridors not built headlessly
    const NavGraph nav = buildNavGraph(g);
    const double buildMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tb).count();

    // city_render.cpp's own formula, including the halving for directed links.
    Real laneKm = 0, walkKm = 0;
    for (std::size_t li = 0; li < nav.links.size(); ++li) {
        const NavLink& L = nav.links[li];
        laneKm += L.length * std::max(1, L.lanes) * 1e-3;
        if (L.klass != RoadClass::Freeway && L.klass != RoadClass::Ramp)
            walkKm += L.length * 2.0 * 1e-3;
    }
    laneKm *= 0.5;
    walkKm *= 0.5;
    const int wantCars = static_cast<int>(laneKm * 10.0);   // carsPerLaneKm
    const int wantPeds = static_cast<int>(walkKm * 6.0);    // pedsPerKm

    std::printf("    [metro] built in %.0f ms: %zu nodes, %zu links\n", buildMs,
                nav.nodes.size(), nav.links.size());
    std::printf("    [metro] %.1f lane-km, %.1f sidewalk-km\n", laneKm, walkKm);
    std::printf("    [metro] density asks for %d cars + %d walkers = %d agents\n",
                wantCars, wantPeds, wantCars + wantPeds);
    std::printf("    [metro] today's clamp gives %d + %d = %d agents\n",
                std::min(wantCars, 400), std::min(wantPeds, 400),
                std::min(wantCars, 400) + std::min(wantPeds, 400));

    // What that population costs on this network, flat and tiered.
    for (int tiered = 0; tiered < 2; ++tiered) {
        CitySim sim;
        sim.build(nav, wantCars, wantPeds, 9);
        if (tiered) {
            sim.tieringEnabled = true;
            sim.dormancyEnabled = true;
            sim.setTierCenter(Vec2(0, 0));
        }
        for (int i = 0; i < 30; ++i) sim.step(1.0 / 60.0, 0.05);
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 60; ++i) sim.step(1.0 / 60.0, 0.05);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 60.0;
        int k = 0, v = 0, d = 0;
        for (const auto& a : sim.agents()) {
            if (a.tier == citysim::Agent::Tier::K) ++k;
            else if (a.tier == citysim::Agent::Tier::V) ++v;
            else ++d;
        }
        std::printf("    [metro] %-6s %d agents: %.3f ms/tick  K %d  V %d  D %d\n",
                    tiered ? "tiered" : "flat", static_cast<int>(sim.agents().size()), ms, k, v, d);
        CHECK(sim.agents().size() > 0);
    }
}
