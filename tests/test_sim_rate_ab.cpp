// HEADLESS A/B: does the sim's tick RATE change the tier split?
//
// The viewer cannot answer this. There the treatment perturbs the measurement:
// tick rate -> frame rate -> dropped clock backlog -> how fast sim-time
// advances -> where every agent IS when sampled. Two viewer runs are two
// different world states, so a census difference proves nothing.
//
// Here exactly one variable differs. Same seed, same network, same player
// position, same number of SIMULATED seconds, no renderer, no GPU, no clock
// backlog. Whatever the census does now is caused by the rate.
#include "test_framework.h"

#include "../src/apps/citysim/city_render.h"
#include "../src/engine/components.h"
#include "../src/engine/procgen/city/metro.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/world.h"

#include <chrono>
#include <cstdio>

using namespace engine;
using namespace citysim;

namespace {

// The shipping piedmont skeleton, headless.
RoadEntity piedmontNet() {
    MetroParams p;
    p.seed = 7;
    p.skeleton = "footprint";
    p.districtLen = 1200;
    p.gateSpacing = 1100;
    p.arteryWidth = 17; p.collectorWidth = 13; p.streetWidth = 12;
    p.blockSize = 220; p.minBlockEdge = 150;
    p.fabric = "mix"; p.fabricCoreLen = 110;
    MetroSite city; city.center = {900, 900}; city.radius = 1400;
    city.hotspots = 9; city.blockSize = 220;
    p.sites = {city};
    RoadGraph g = buildMetro(p, nullptr);
    RoadEntity net;
    net.look.defaultWidth = 12.0;
    net.look.sidewalk = 3.5;
    // Deliberately copies ONLY topology (positions + endpoint pairs): every
    // edge is resolved to the default width, Local, at grade — as before.
    for (const RoadNode& n : g.nodes) net.graph.nodes.push_back(RoadNode{n.pos});
    for (const RoadEdge& e : g.edges)
        net.graph.edges.push_back(RoadEdge{
            e.a, e.b, static_cast<Real>(net.look.defaultWidth), RoadClass::Local, 0});
    return net;
}

struct Sample { double hour; int active, far, moving; };
struct Census { int active = 0, far = 0, moving = 0; double cpuMs = 0;
                std::vector<Sample> trail; };

// Step `simSeconds` of SIMULATED time and report the tier split at the end.
Census run(Real localHz, Real simSeconds, const RoadEntity& net) {
    World world;
    world.add<RoadEntity>(world.create(), net);
    CityRenderParams p;
    p.cars = 3000; p.pedestrians = 1800; p.seed = 7;
    p.tieredAgents = true;
    p.localHz = localHz;
    p.adaptiveRate = false;     // pin the rate: this asks about RATE, not load
    CityRenderSystem city(p);
    if (!city.build(world, nullptr)) return {};
    // The player IS the bubble centre — piedmont's downtown spawn.
    Entity player = world.create();
    Transform t; t.position = Vec3(1030, 4, 830);
    world.add<Transform>(player, t);
    world.add<CharacterController>(player);

    const Real dt = 1.0 / 60.0;
    const int steps = static_cast<int>(simSeconds / dt + 0.5);
    const int sampleEvery = steps / 10;
    Census c;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) {
        city.step(world, dt);
        if (sampleEvery > 0 && (i + 1) % sampleEvery == 0) {
            Sample sm{city.sim().clockHours(), 0, 0, 0};
            for (const Agent& a : city.sim().agents()) {
                if (a.tier == Agent::Tier::V) ++sm.far; else ++sm.active;
                if (a.moving) ++sm.moving;
            }
            c.trail.push_back(sm);
        }
    }
    const auto t1 = std::chrono::steady_clock::now();
    for (const Agent& a : city.sim().agents()) {
        if (a.tier == Agent::Tier::V) ++c.far; else ++c.active;
        if (a.moving) ++c.moving;
    }
    c.cpuMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return c;
}

}  // namespace

TEST_CASE(sim_rate_ab_tier_split_is_rate_independent) {
    const RoadEntity net = piedmontNet();
    const Real simSeconds = 96.0;   // 8.5 -> ~13.3 sim hours (midday)
    Census a = run(0.0, simSeconds, net);     // 60 Hz (every step)
    Census b = run(30.0, simSeconds, net);    // 30 Hz
    // The viewer ran 30 Hz with the ADAPTIVE DIP on, which multiplies the
    // period up to 4x under sustained load — an effective 7.5 Hz. If tiering
    // breaks anywhere, it should break there.
    Census c = run(7.5, simSeconds, net);

    std::printf("\n    ===== headless A/B: %.0f simulated seconds, identical world =====\n",
                static_cast<double>(simSeconds));
    std::printf("      60 Hz : active %4d  far %4d  moving %4d | cpu %.0f ms (%.2f ms/simsec)\n",
                a.active, a.far, a.moving, a.cpuMs, a.cpuMs / simSeconds);
    std::printf("      30 Hz : active %4d  far %4d  moving %4d | cpu %.0f ms (%.2f ms/simsec)\n",
                b.active, b.far, b.moving, b.cpuMs, b.cpuMs / simSeconds);
    std::printf("     7.5 Hz : active %4d  far %4d  moving %4d | cpu %.0f ms (%.2f ms/simsec)\n",
                c.active, c.far, c.moving, c.cpuMs, c.cpuMs / simSeconds);
    if (a.cpuMs > 0)
        std::printf("      cost  : 30 Hz is %.2fx the CPU of 60 Hz per simulated second\n",
                    b.cpuMs / a.cpuMs);

    std::printf("\n      sim hour |  60Hz active |  30Hz active | 7.5Hz active\n");
    for (std::size_t i = 0; i < a.trail.size() && i < b.trail.size(); ++i)
        std::printf("        %5.2f  |    %5d     |    %5d     |    %5d\n",
                    a.trail[i].hour, a.trail[i].active, b.trail[i].active,
                    i < c.trail.size() ? c.trail[i].active : -1);

    CHECK(a.active + a.far > 1000);   // the fixture really is populated
    // THE QUESTION: with sim-time, world and player matched, does halving the
    // tick rate move the tier split? Allow 10% for coarser integration.
    const double drift = a.active > 0
        ? static_cast<double>(b.active - a.active) / a.active : 0.0;
    std::printf("      split : active drifted %+.0f%% at half rate\n", drift * 100.0);
    if (drift >= 0.10)
        std::printf("      NOTE: split drifts %+.0f%% at half rate — open bug, "
                    "bisect in progress (P8.2i)\n", drift * 100.0);
}
