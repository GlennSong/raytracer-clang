// THE BRAIN ON THE ISLAND (ADR-0496; the one-brain plan, step 4): island_8_nature with every household run by the
// agent brain (citysim.brainPeople -1, set here before the city is built), the island's households driving their own
// cars (ADR-0533: the island names river_town's rule set; before, its people only walked and rode, ADR-0494), and the
// player standing in Saltwood (the level's own start), so the brain's attention (ADR-0493) has most of the island far:
// far drives on the clock, their cars away (ADR-0516..0518).
//
// What it measures: how many people, how long the brain takes to start, what a tick costs, how many are far, and
// whether their days go on -- out, to work inside their shift, nobody at a shut place, nothing erring. The island's
// towns are joined by rail and the brain has no trains yet, so a trip to another town finds no way (counted).
//
//   RT_TEST_FILTER=level_island_brain ./build-viewer/level_tests   (RT_ISLAND_MINUTES: how many minutes, default 5)
//
// A HEAVY level case: the island's lots must be baked (rt_bake assets/levels/island_8_nature.json, about 35 GB).

#include "test_framework.h"

#include "crowd_test_rig.h"
#include "../src/apps/citysim/city_car.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

using namespace engine;

namespace {

constexpr Real kStep = 1.0 / 60.0;

double residentMbIsland() {
    std::ifstream f("/proc/self/status");
    std::string key;
    while (f >> key) {
        if (key == "VmRSS:") {
            double kb = 0;
            f >> kb;
            return kb / 1024.0;
        }
        std::getline(f, key);
    }
    return 0;
}

}  // namespace

TEST_CASE(level_island_brain_runs_every_household) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    crowdtest::LoadedCrowd lc("island_8_nature.json", [](CitySimConfig& c) { c.brainPeople = -1; });
    const double loadS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    CHECK(lc.ok);
    if (!lc.ok) return;
    citysim::CitySim& sim = lc.city.simMutable();
    citysim::CityBrain* bp = sim.brain();
    CHECK(bp != nullptr && bp->hasResident());
    if (!bp || !bp->hasResident()) return;
    citysim::CityBrain& b = *bp;
    std::printf("    [island] loaded and built in %.1f s; %d brain people of the sim's %zu agents, %zu cars, %zu markers; %.0f MB\n",
                loadS, b.residentCount(), sim.agents().size(), b.cars().size(), b.markers().size(), residentMbIsland());
    const engine::Vec2 player(-4057.9, -439.6);   // (the level's player start, in Saltwood)
    const int ticksPerMinute = static_cast<int>(60.0 / kStep + 0.5);
    const int minutes = std::getenv("RT_ISLAND_MINUTES") ? std::atoi(std::getenv("RT_ISLAND_MINUTES")) : 5;
    double stepMs = 0, stepWorst = 0;
    // THE FAR CAR TIER (ADR-0516..0518), looked at every minute: no car away is drawn, every car away is a far driver's,
    // and no far person on a leg by car has their car on the road
    int awayDrawn = 0, awayNotDriving = 0, farOnRoad = 0, mostOnRoad = 0;
    for (int minute = 1; minute <= minutes; ++minute) {
        for (int t = 0; t < ticksPerMinute; ++t) {
            sim.setTierCenter(player);
            const std::chrono::steady_clock::time_point s0 = std::chrono::steady_clock::now();
            lc.city.step(lc.world, kStep);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s0).count();
            stepMs += ms;
            stepWorst = std::max(stepWorst, ms);
        }
        int away = 0, onRoad = 0;
        lc.world.each<citysim::CityCarKept>([&](engine::Entity e, citysim::CityCarKept& k) {
            const int holder = k.heldBy < static_cast<citysim::brain::AgentId>(b.residentCount()) ? static_cast<int>(k.heldBy) : -1;
            const bool far = holder >= 0 && b.residentFar(holder);
            const std::string st = holder >= 0 ? b.stateName(holder) : std::string();
            const bool carLeg = st == "pulling_out" || st == "driving" || st == "parking";
            if (k.phase == citysim::CityCarPhase::Away) {
                ++away;
                awayDrawn += lc.world.has<citysim::CityCarPose>(e) ? 1 : 0;
                awayNotDriving += far && st == "driving" ? 0 : 1;
            }
            onRoad += lc.world.has<citysim::CityCarOnLane>(e) ? 1 : 0;
            farOnRoad += far && carLeg && lc.world.has<citysim::CityCarOnLane>(e) ? 1 : 0;
        });
        mostOnRoad = std::max(mostOnRoad, onRoad);
        const citysim::CityBrainCounts& c = b.counts();
        std::printf("    [island] %2d min: cars on the road %d, away %d | trips by car %u, far drives %u (stood in their space %u, "
                    "given up %u) | lowered driving %u, raised mid-drive %u (no room yet %u, at the start %u) | away and drawn %d, "
                    "away and not a far driver's %d, a far driver's on the road %d\n",
                    minute, onRoad, away, c.tripsBegun, c.drivenByClock, c.farParked, c.awayLeft, c.loweredDriving,
                    c.raisedOnRoute, c.raiseWaited, c.raisedAtStart, awayDrawn, awayNotDriving, farOnRoad);
        {
            // WHAT THE DRIVING COSTS a tick (ADR-0533): its model, the measurements watching it, the hand-over after it
            const citysim::DrivingHost::Cost& dc = sim.drivingHost().cost();
            const double n = static_cast<double>(std::max<uint64_t>(1, dc.ticks));
            std::printf("    [island] %2d min: the driving %.3f ms a tick (worst %.3f), watching it %.3f, the hand-over %.3f (worst "
                        "%.3f), over %llu ticks\n",
                        minute, dc.modelMs / n, dc.modelWorstMs, dc.watchMs / n, dc.hostMs / n, dc.hostWorstMs,
                        static_cast<unsigned long long>(dc.ticks));
        }
        std::printf("    [island] %2d min: %05.2f h | %s | the day: set out %u, arrived %u, nowhere %u, no way %u, at a shut "
                    "place %u, at work %u (%u outside the shift) | brain %.3f ms a tick (worst %.3f), the whole step %.3f ms "
                    "(worst %.3f) | %.0f MB\n",
                    minute, sim.clockHours(), b.attentionText().c_str(), c.daySetOut, c.dayArrivals, c.dayNowhere, c.dayNoWay,
                    c.dayArrivedShut, c.dayToWork, c.dayToWorkOffShift, b.cost().ms / std::max<uint64_t>(1, b.cost().ticks),
                    b.cost().worstMs, stepMs / (static_cast<double>(minute) * ticksPerMinute), stepWorst, residentMbIsland());
    }
    const citysim::CityBrainCounts& c = b.counts();
    std::printf("    [island] the brain's cost: %s\n", b.cost().describe().c_str());
    std::printf("    [island] the machine's last error: %s\n", b.machine()->lastError());
    std::printf("    [island] at the end: arrivals %llu, plans %llu, no plans %llu, errors %llu, refused %llu\n",
                static_cast<unsigned long long>(b.machine()->counts().arrivals),
                static_cast<unsigned long long>(b.machine()->counts().plans),
                static_cast<unsigned long long>(b.machine()->counts().noPlans),
                static_cast<unsigned long long>(b.machine()->counts().errors),
                static_cast<unsigned long long>(b.machine()->counts().refused));
    // BY TRAIN (step 4f): the island's towns are joined by rail. Whose plan now has a train in it, an example, and the
    // near riders the train riders carried.
    int byTrain = 0;
    std::string example;
    for (int who = 0; who < b.residentCount(); ++who) {
        const std::string plan = b.planText(who);
        if (plan.find("train>") == std::string::npos) continue;
        if (byTrain++ == 0) example = plan + " (" + b.planReason(who) + ")";
    }
    const citysim::TrainRiders::Counts& tc = sim.trainRiders().counts();
    std::printf("    [island] trains: %zu lines, %zu stations; %d people's plans go by train, e.g. %s; near riders: %u went "
                "in, %u came out; the riders: boarded %llu, got off %llu, a full train left someone %llu times\n",
                sim.rail().lines().size(), sim.trainRiders().stations().size(), byTrain, example.c_str(), c.trainWaits,
                c.trainRides, static_cast<unsigned long long>(tc.boardings), static_cast<unsigned long long>(tc.alightings),
                static_cast<unsigned long long>(tc.leftFull));
    CHECK(b.residentCount() > static_cast<int>(sim.agents().size()) * 9 / 10);   // (the brain runs nearly everyone)
    // THE ISLAND DRIVES (ADR-0533): its households have cars in kerb spaces, and the far car tier holds
    std::printf("    [island] cars: %zu household cars; the most on the road at once %d\n", b.cars().size(), mostOnRoad);
    CHECK(!b.cars().empty());
    if (minutes >= 3) CHECK(c.drivenByClock > 0u);   // (the first drives set out in the third minute)
    CHECK(awayDrawn == 0 && awayNotDriving == 0 && farOnRoad == 0);
    CHECK(c.raisedAtStart == 0u);
    CHECK(b.farCount() > b.residentCount() / 2);                                 // (most of the island is far)
    CHECK(c.dayArrivals > 0u);
    CHECK(c.dayArrivedShut == 0u && c.dayToWorkOffShift == 0u);
    CHECK(b.machine()->counts().errors == 0u && b.machine()->counts().refused == 0u);
}
