// RIVER TOWN ON THE NEW DRIVING (ADR-0457; #306, deleting the old driving, piece B).
//
// Loads assets/levels/river_town.json through the real LevelLoader and CityRenderSystem (a NullRenderer underneath): the
// town's roads and lots, and a crowd made when the level loads from assets/scripts/crowd_river_town.lua and the road model
// (city_crowd.h, the same reader and maker as brain_crowd's crowd.lua): residents in households, the households' cars in
// kerb spaces near home, each person a day round places spread along the streets. The new driving drives their cars;
// the old driving drives nothing (no v1 cars, buses or cabs). The city is stepped as the viewer steps it, 60 times a
// second, for kRiverMinutes of the day (RT_CROWD_MINUTES for another stretch; the pins then do not apply).
//
//   RT_TEST_FILTER=level_river_town_crowd ./build-viewer/level_tests
//
// Every simulated minute it reports: the residents by state, the cars on the road and those waiting, rings of waits,
// trips begun, parked, plans that found no way and walks blocked, the longest a car has stood still on the road (and
// its driver card at the end: why), bodies touching, what a tick cost (the brain, the driving and the whole city step)
// and the process's memory. Held: no ring of waits at any minute, nothing driven by the old driving, the signal heads
// tied to the new driving's lamps, and the counts and state hash of the run (pinned from a run that was looked at).

#include "test_framework.h"

#include "crowd_test_rig.h"
#include "../src/apps/citysim/city_car.h"
#include "../src/apps/driving/measure.h"
#include "../src/apps/driving/wait_graph.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

using namespace engine;

namespace {

constexpr Real kStep = 1.0 / 60.0;   // the viewer's fixed step, and the sim's tick
constexpr int kRiverMinutes = 30;    // the stretch of the day the test runs: everybody's first trip out and most back

// The process's resident memory, MB (Linux /proc; 0 where there is none).
double residentMb() {
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

TEST_CASE(level_river_town_crowd_drives_on_the_new_driving) {
    const double mb0 = residentMb();
    crowdtest::LoadedCrowd lc("river_town.json");
    CHECK(lc.ok);
    if (!lc.ok) return;
    citysim::CitySim& sim = lc.city.simMutable();
    // (the sim is built with nobody driving since ADR-0461: its agents are the people on foot)
    const int walkers = static_cast<int>(sim.agents().size());
    CHECK(sim.drivingV2() != nullptr && sim.drivingHost().active());
    citysim::CityBrain* bp = sim.brain();
    CHECK(bp != nullptr && bp->hasResident());
    if (!sim.drivingV2() || !bp || !bp->hasResident()) return;
    citysim::CityBrain& b = *bp;
    const driving::Driving& d = *sim.drivingV2();
    World& world = lc.world;
    const int minutes = std::getenv("RT_CROWD_MINUTES") ? std::atoi(std::getenv("RT_CROWD_MINUTES")) : kRiverMinutes;
    const double mbLoaded = residentMb();

    // WHAT THE TOWN AND THE CROWD ARE
    int households = 0;
    {
        std::map<std::string, int> houses;
        for (int who = 0; who < b.residentCount(); ++who) ++houses[b.householdName(who)];
        households = static_cast<int>(houses.size());
    }
    int withCar = 0;
    for (int who = 0; who < b.residentCount(); ++who) withCar += b.residentCar(who) >= 0 ? 1 : 0;
    std::printf("    [river] %d residents in %d households, %zu cars (%d residents in a household with one), %zu markers; the old "
                "sim's agents: %zu (%d walkers); the driving's cars %zu; memory %.0f MB loaded "
                "(%.0f before)\n",
                b.residentCount(), households, b.cars().size(), withCar, b.markers().size(), sim.agents().size(), walkers,
                d.carCount(), mbLoaded, mb0);
    // THE ROADS THE NEW DRIVING HAS: the model's junctions and streets, and its lane level's paths
    std::printf("    [river] the road model: %zu junctions, %zu streets, %zu lane-level paths; the sim's walk points %d\n",
                d.model()->junctions.size(), d.model()->streets.size(), d.roads()->paths.size(), sim.walkPoints().nodeCount());

    // THE DAY, minute by minute
    const int ticksPerMinute = static_cast<int>(60.0 / kStep + 0.5);
    int mostOnRoad = 0, mostOnRoadAt = 0, ringsSeen = 0;
    double stepMsSum = 0, stepWorst = 0;
    std::map<uint32_t, int> standingSince;   // car id -> the tick it last stood still from
    int longestStandTicks = 0;
    uint32_t longestStandCar = 0;
    std::string longestStandWhy;
    for (int minute = 1; minute <= minutes; ++minute) {
        for (int t = 0; t < ticksPerMinute; ++t) {
            const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
            lc.city.step(world, kStep);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            stepMsSum += ms;
            stepWorst = std::max(stepWorst, ms);
            const int tick = (minute - 1) * ticksPerMinute + t;
            for (const driving::Car& c : d.cars()) {
                if (!driving::isWaiting(c)) {
                    standingSince.erase(c.id.index);
                    continue;
                }
                const std::map<uint32_t, int>::iterator it = standingSince.emplace(c.id.index, tick).first;
                if (tick - it->second > longestStandTicks) {
                    longestStandTicks = tick - it->second;
                    longestStandCar = c.id.index;
                    if (t % 60 == 0) longestStandWhy = d.describeCar(c.id);   // (once a second: the card is long)
                }
            }
        }
        std::map<std::string, int> byState;
        for (int who = 0; who < b.residentCount(); ++who) ++byState[b.stateName(who)];
        int onRoad = 0;
        world.each<citysim::CityCarOnLane>([&](Entity, citysim::CityCarOnLane& lane) { onRoad += lane.driven ? 1 : 0; });
        if (onRoad > mostOnRoad) {
            mostOnRoad = onRoad;
            mostOnRoadAt = minute;
        }
        const driving::WaitReport wr = d.waitReport();
        ringsSeen += static_cast<int>(wr.loops.size());
        std::string states;
        for (const std::pair<const std::string, int>& s : byState) states += " " + s.first + " " + std::to_string(s.second);
        const citysim::CityBrainCounts& c = b.counts();
        const citysim::brain::BrainCounts& mc = b.machine()->counts();
        std::printf("    [river] %2d min:%s | %d cars on the road, %u waiting, %zu rings | trips begun %u, parked %u; plans %llu, "
                    "no way %llu, walks blocked %u | longest stand %.1f s (car %u) | touching spells %llu | brain %.3f ms a "
                    "tick (worst %.3f), driving %.3f ms (worst %.3f), the whole step %.3f ms (worst %.3f) | %.0f MB\n",
                    minute, states.c_str(), onRoad, wr.waiting, wr.loops.size(), c.tripsBegun, c.parkedReports,
                    static_cast<unsigned long long>(mc.plans), static_cast<unsigned long long>(mc.noPlans), c.walksBlocked,
                    longestStandTicks * kStep, longestStandCar,
                    static_cast<unsigned long long>(sim.drivingHost().measurements().rows().touches.spells),
                    b.cost().ms / std::max<uint64_t>(1, b.cost().ticks), b.cost().worstMs,
                    sim.drivingHost().cost().modelMs / std::max<uint64_t>(1, sim.drivingHost().cost().ticks),
                    sim.drivingHost().cost().modelWorstMs, stepMsSum / (minute * ticksPerMinute), stepWorst, residentMb());
    }
    const driving::WaitReport end = d.waitReport();
    const uint64_t hash = crowdtest::crowdHash(b, sim, world);
    std::printf("    [river] at the end: %zu rings of waits (%u gridlocks, %u deadlocks); the most cars on the road at once %d "
                "(minute %d); another place of the kind chosen %u times; arrivals %llu; errors %llu, refused %llu; "
                "final state hash %016llx\n",
                end.loops.size(), end.gridlocks, end.deadlocks, mostOnRoad, mostOnRoadAt, b.counts().otherPlaces,
                static_cast<unsigned long long>(b.machine()->counts().arrivals),
                static_cast<unsigned long long>(b.machine()->counts().errors),
                static_cast<unsigned long long>(b.machine()->counts().refused),
                static_cast<unsigned long long>(hash));
    std::printf("    [river] the longest stand, car %u, %.1f s; its card then:\n%s\n", longestStandCar, longestStandTicks * kStep,
                longestStandWhy.c_str());
    if (!end.loops.empty()) std::printf("%s", d.describeWaiting(end).c_str());
    // WHY THE CARS STANDING AT THE END STAND: the rule each one's last decision named (its card's "now:" line)
    {
        std::map<std::string, int> why;
        for (const driving::Car& c : d.cars()) {
            if (!driving::isWaiting(c)) continue;
            const std::string card = d.describeCar(c.id);
            const std::size_t at = card.find("by rule \"");
            const std::size_t to = at == std::string::npos ? at : card.find('"', at + 9);
            ++why[at == std::string::npos ? std::string("(no rule named)") : card.substr(at + 9, to - at - 9)];
        }
        for (const std::pair<const std::string, int>& w : why) std::printf("    [river] standing at the end: %d by \"%s\"\n", w.second, w.first.c_str());
    }
    // ...and what is in the longest-standing car's way: the things on the road within reach of its front
    if (const driving::Car* car = d.car(driving::CarId(longestStandCar))) {
        constexpr Real kLookReach = 8.0;
        const driving::CarPose pose = driving::carPose(*d.roads(), d.vehicleClasses(), d.ruleSet(), *car);
        const Vec2 front(pose.front.x, pose.front.z);
        for (const driving::RoadThing& th : d.thingsOnRoad()) {
            if ((th.at - front).length() > kLookReach) continue;
            int agent = -1;
            for (std::size_t i = 0; i < sim.agents().size() && agent < 0; ++i)
                if (sim.agents()[i].uid == th.id) agent = static_cast<int>(i);
            const citysim::CitySim::CrossingAhead x =
                agent >= 0 ? sim.nextCrossing(sim.agents()[static_cast<std::size_t>(agent)]) : citysim::CitySim::CrossingAhead();
            std::printf("    [river]   in its way: thing %u at %.1f %.1f (%.1f m from its front, crosswalk %d; hold why %d; next "
                        "crossing found %d crosswalk %u to hold %.2f; resident %d): %s\n",
                        th.id, th.at.x, th.at.y, (th.at - front).length(),
                        th.crosswalk == engine::kNoIndex ? -1 : static_cast<int>(th.crosswalk),
                        agent >= 0 ? sim.holdWhyOf(agent) : -1, x.found ? 1 : 0, x.crosswalk, x.toHold,
                        agent >= 0 && agent < b.residentCount() ? 1 : 0,
                        agent >= 0 ? sim.describeAgent(agent).substr(260, 300).c_str() : "(not a walker of the sim)");
        }
    }

    // THE SIGNAL HEADS show the new driving's lamps (ADR-0456): every head the loader placed is tied to its junction
    std::size_t tied = 0;
    for (std::size_t k = 0; k < lc.city.signalHeadCount(); ++k) tied += lc.city.signalHeadJunction(k).valid() ? 1 : 0;
    std::printf("    [river] signal heads %zu, tied to the new driving's junctions %zu\n", lc.city.signalHeadCount(), tied);
    CHECK(lc.city.signalHeadCount() > 0u);
    CHECK(tied == lc.city.signalHeadCount());

    // NO RING OF WAITS AT ANY MINUTE
    CHECK(ringsSeen == 0);
    // EVERY WALK SIGNAL WALKS in some phase (a crosswalk that never shows WALK strands its walkers behind the kerb)
    CHECK(d.rightOfWay().counts.walksNever == 0u);
    CHECK(d.signalProblems().empty());
    CHECK(b.machine()->counts().errors == 0u);
    CHECK(b.machine()->counts().refused == 0u);

    // WHAT WAS MADE AND WHAT HAPPENED, pinned from the run of 2026-11-18 (ADR-0457; looked at: the way to work builds from
    // minute 3, the most on the road 117 at minute 11, back to 55 at minute 19 and 117 again at 30 as the way home and a
    // jam at lane junction 29 build up: 99 of the 117 standing at the end, car 91 behind about 40 residents queued at
    // crosswalk 123's kerb end from minute 5, a known fault not yet diagnosed; 8 residents in no_way all run, planning
    // again each minute, most of the 225 no-plans). Another stretch of the day (RT_CROWD_MINUTES) holds only the rings.
    // RE-PINNED at the S7 re-bake (ADR-0478; the roads of ADR-0470..0473 and 0477, the lots' frontage and walkways of
    // ADR-0474..0476): places in a cut-off part on foot 97 -> 23 (level_walk_join), so nobody is left in no_way (8 -> 0)
    // and no-plans fall 225 -> 9; trips 259 -> 322, parked 141 -> 210, arrivals 621 -> 753, the most on the road 117 ->
    // 135 (still minute 11). Fewer households hold a car: cars 388 -> 364 (with one 415 -> 393), the kerb spaces having
    // moved with the stop lines. Worse, small: walks blocked 7 -> 11, another place 6 -> 9. The known jam is still there,
    // another car: car 237 stands 1540.6 s from minute 5 giving way to people crossing (was car 76, 1480.9 s). Rings still
    // 0, touching spells 0. Hash 0x83b99189cfc16fbd -> 0xc02460bd06f44189.
    // RE-PINNED for ADR-0479 (the pavement layers' rounding slivers and spikes taken off, the sidewalk's necks kept): the
    // lots regrown on the new pavement gaps and the walks laid on the cleaned sidewalk. Better: walks blocked 11 -> 0,
    // another place 9 -> 0, no-plans 9 -> 0, the longest stand 1540.6 s (car 237) -> 734.7 s (car 19). Worse, small:
    // arrivals 753 -> 730, trips 322 -> 318, parked 210 -> 209. Cars 364 -> 383 (with one 393 -> 407), the most on the
    // road 135 -> 139 (still minute 11). Rings 0, touching spells 0. (An opening alone, without keeping the necks, cut
    // sidewalks in two and went the wrong way: arrivals 692, walks blocked 21, no way 42; not kept.)
    // RE-PINNED at the street map's deletion (#306; ADR-0480 W3, W3b and the sweep), whatever the new system gives
    // (Glenn: "kill the old system and replace it ... iron out bugs afterwards"): people walk the road model's walks by
    // the one router (W3), river_town's sliver streets went (s8, s9, s10, s169), the crowd maker holds each home's kerb
    // spaces and puts places only where the walks reach (W3b), and the sim's own walkers rest at walk points, not
    // street-map nodes. Cars 383 -> 391 (with one 407 -> 413); the most on the road 139 at minute 11 -> 98 at minute 10;
    // trips 318 -> 292; parked 209 -> 240; ARRIVALS 730 -> 472 (on the known-bugs list: about 410 of the 630 are
    // walking at any minute from minute 8, and why the walks take longer is not known); the longest stand 734.7 s
    // (car 19) -> 450.2 s (car 20). No plan, blocked, another place, rings, touching spells: 0.
    // (At the sweep's second half: the most on the road 99 at minute 11, trips 290, parked 237, arrivals 475.)
    // RE-PINNED when every walk signal got a phase to walk in (ADR-0480, the sweep's bugs, 1): 14 of the 432 lit
    // crosswalks showed WALK in no phase -- a near turn out of their arm meets no car, so the phase builder's single-turn
    // fill made it green in every phase -- and only jaywalkers crossed them; queues stood behind their kerbs all run
    // (crosswalk 36 at junction 8 held about 15 people). Walkers standing still fell from about 190 at any moment to about
    // 95. ARRIVALS 475 -> 675; trips 290 -> 316; parked 237 -> 245; the most on the road 99 -> 102 (still minute 11).
    // Walking at minute 30 416 -> 369 (most of them on 900 m-plus walks to work: the crowd's days). No plan, blocked,
    // another place, rings, touching spells: still 0. The longest stand is the same, 450.2 s (car 20).
    if (minutes == kRiverMinutes) {
        CHECK(b.residentCount() == 630);
        CHECK(households == 582);
        CHECK(b.cars().size() == 391u);
        CHECK(withCar == 413);
        CHECK(b.markers().size() == 799u);
        CHECK(mostOnRoad == 102);
        CHECK(mostOnRoadAt == 11);
        CHECK(b.counts().tripsBegun == 316u);
        CHECK(b.counts().parkedReports == 245u);
        CHECK(b.counts().walksBlocked == 0u);
        CHECK(b.counts().otherPlaces == 0u);
        CHECK(b.machine()->counts().arrivals == 675u);
        CHECK(b.machine()->counts().noPlans == 0u);
        CHECK(sim.drivingHost().measurements().rows().touches.spells == 0u);
        // 0x9757add914fdb773 -> 0x4b8f868f3d5e05e0 (ADR-0458): 27 paths' crossings of 5 crosswalks on sloping junctions
        // are each one whole record now, so who gives way to whom there moved; every count above is unchanged
        // 0x4b8f868f3d5e05e0 -> 0xde1f57edbda09ebc (ADR-0460): river_town's old walkers no longer hail cabs that never
        // existed (hailChance 0.25, no cabs: a quarter of the walkers facing 350 m or more stood at the kerb all run);
        // they walk. Put back, the old hash returns exactly. Trips 263 -> 259, parked 145 -> 141, arrivals 623 -> 621.
        // 0xde1f57edbda09ebc -> 0x83b99189cfc16fbd (ADR-0461): the 15 fire engines parked in river_town's fire stations
        // were rows 0-14 of the old sim's vehicle list, and a car entity is numbered past every row (nextCarNumber), so
        // every household car's number is 15 lower now (the longest stand: car 91 -> 76). Every count, every minute,
        // is the same.
        // 0x83b99189cfc16fbd -> 0xc02460bd06f44189 (ADR-0478, the S7 re-bake): the counts above.
        // 0xc02460bd06f44189 -> 0xa15a1ec8afaff85d (ADR-0479, the pavement layers cleaned): the counts above.
        // 0xa15a1ec8afaff85d -> 0x9362ecbc934c82cf (#306, the street map's deletion, its first half): the counts above.
        // 0x9362ecbc934c82cf -> 0xf9c704ff73207220 (#306, its second half): the street furniture planned on the road
        // model -- one signal pole per arriving arm, a sidewalk's width back from its mouth; lamps and kerb life along
        // each kerb -- so the walkers' obstacles (the poles and the furniture) stand elsewhere: the counts above.
        // 0xf9c704ff73207220 -> 0x12277ea4937a87e4 (every crosswalk's walk signal walks in some phase): the counts above.
        CHECK(hash == 0x12277ea4937a87e4ull);
    }
}
