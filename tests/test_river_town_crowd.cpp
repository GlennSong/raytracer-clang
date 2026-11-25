// RIVER TOWN ON THE NEW DRIVING (ADR-0457; #306, deleting the old driving, piece B).
//
// Loads assets/levels/river_town.json through the real LevelLoader and CityRenderSystem (a NullRenderer underneath): the
// town's roads and lots, and 630 of the people its dwellings were filled with, whole households of them, run by the agent
// brain (ADR-0490, brain_people.h; the level's `citysim.brainPeople`): each at home in their real building, the households'
// cars in kerb spaces near home, a worker's day to their own job and home again. The new driving drives their cars;
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
    const std::size_t homeMarkers = b.markers().size();   // (at the start: the homes; a day makes more as it goes)
    std::printf("    [river] %d residents in %d households, %zu cars (%d residents in a household with one), %zu markers; the old "
                "sim's agents: %zu (%d walkers); the driving's cars %zu; memory %.0f MB loaded "
                "(%.0f before)\n",
                b.residentCount(), households, b.cars().size(), withCar, b.markers().size(), sim.agents().size(), walkers,
                d.carCount(), mbLoaded, mb0);
    // THE PEOPLE ARE THE CITY'S OWN (ADR-0490): every brain person is the sim agent of a household the city's dwellings
    // were filled with -- the brain's household of the same number -- and that household's home is a home: a dwelling of
    // its building, or a hall's bed
    {
        int notTheirs = 0, notAHome = 0;
        const citysim::HouseholdPlan& plan = sim.households();
        const citysim::PlaceMap& places = lc.city.places();
        for (int who = 0; who < b.residentCount(); ++who) {
            const int agent = b.residentAgent(who);
            const int h = agent >= 0 ? sim.agents()[static_cast<std::size_t>(agent)].household : -1;
            if (h < 0 || b.householdName(who) != "h" + std::to_string(h)) {
                ++notTheirs;
                continue;
            }
            const citysim::Household& hh = plan.households[static_cast<std::size_t>(h)];
            const bool home = hh.place != citysim::kNoPlace && places[hh.place].type == citysim::PlaceType::Home &&
                              (hh.dwelling != citysim::Household::kNoDwelling || hh.type == citysim::Household::kHall);
            notAHome += home ? 0 : 1;
        }
        std::printf("    [river] the brain's people: %d not a household's own agent, %d whose household's home is no dwelling\n",
                    notTheirs, notAHome);
        CHECK(notTheirs == 0);
        CHECK(notAHome == 0);
    }
    // THE ROADS THE NEW DRIVING HAS: the model's junctions and streets, and its lane level's paths
    std::printf("    [river] the road model: %zu junctions, %zu streets, %zu lane-level paths; the sim's walk points %d\n",
                d.model()->junctions.size(), d.model()->streets.size(), d.roads()->paths.size(), sim.walkPoints().nodeCount());

    // THE BUS LINES its road file states (ADR-0481, laid by `lanes_tool buslines`, ADR-0482): held, and every leg can be
    // driven by the fleet's bus
    {
        const int busClass = d.vehicleClasses().indexOf("bus");
        CHECK(busClass >= 0);
        std::size_t stops = 0;
        for (const engine::ModelBusLine& bl : d.model()->busLines) stops += bl.stops.size();
        const std::vector<std::string> faults = busClass >= 0 ? d.busLineProblems(static_cast<uint32_t>(busClass)) : std::vector<std::string>();
        std::printf("    [river] bus lines %zu (%zu stops), %zu faults\n", d.model()->busLines.size(), stops, faults.size());
        for (const std::string& f : faults) std::printf("    [river]   %s\n", f.c_str());
    }
    // THE DAY, minute by minute
    const int ticksPerMinute = static_cast<int>(60.0 / kStep + 0.5);
    int mostOnRoad = 0, mostOnRoadAt = 0, ringsSeen = 0;
    double stepMsSum = 0, stepWorst = 0;
    std::map<uint32_t, int> standingSince;   // car id -> the tick it last stood still from
    int longestStandTicks = 0;
    uint32_t longestStandCar = 0;
    std::string longestStandWhy;
    // ...and THE BUSES' longest stand in the road, not at a stop (its doors shut): which bus, where, and its card
    std::map<uint32_t, engine::Entity> busOf;   // car id -> the bus's entity
    world.each<citysim::CityBusRuns, citysim::CityCarBody>(
        [&](engine::Entity e, citysim::CityBusRuns&, citysim::CityCarBody& body) { busOf[body.id] = e; });
    std::map<uint32_t, int> busStandSince;
    int busStandTicks = 0;
    uint32_t busStandCar = 0;
    std::string busStandWhy;
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
                    busStandSince.erase(c.id.index);
                    continue;
                }
                const std::map<uint32_t, int>::iterator it = standingSince.emplace(c.id.index, tick).first;
                if (tick - it->second > longestStandTicks) {
                    longestStandTicks = tick - it->second;
                    longestStandCar = c.id.index;
                    if (t % 60 == 0) longestStandWhy = d.describeCar(c.id);   // (once a second: the card is long)
                }
                const std::map<uint32_t, engine::Entity>::const_iterator bus = busOf.find(c.id.index);
                if (bus == busOf.end()) continue;
                const citysim::CityBusRuns* runs = world.get<citysim::CityBusRuns>(bus->second);
                if (!runs || runs->standing) {
                    busStandSince.erase(c.id.index);
                    continue;
                }
                const int since = busStandSince.emplace(c.id.index, tick).first->second;
                if (tick - since > busStandTicks) {
                    busStandTicks = tick - since;
                    busStandCar = c.id.index;
                    if (t % 60 == 0) busStandWhy = d.describeCar(c.id);
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
        {
            // THE DAY (ADR-0491): the clock, and what the role days did so far
            int atWork = 0, out = 0;
            for (int r = 0; r < b.residentCount(); ++r) {
                const citysim::Agent& a = sim.agents()[static_cast<std::size_t>(b.residentAgent(r))];
                atWork += a.activity == citysim::Agent::Activity::AtWork ? 1 : 0;
                out += a.activity != citysim::Agent::Activity::AtHome && a.activity != citysim::Agent::Activity::AtWork ? 1 : 0;
            }
            std::printf("    [river] %2d min: %05.2f h | the day: set out %u, arrived %u, nowhere %u, no way %u, at a shut place %u, at work %u "
                        "(%u outside the shift) | now at work %d, out %d\n",
                        minute, sim.clockHours(), c.daySetOut, c.dayArrivals, c.dayNowhere, c.dayNoWay, c.dayArrivedShut, c.dayToWork,
                        c.dayToWorkOffShift, atWork, out);
        }
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
    // THE BUSES ON THEIR LINES (ADR-0483): how many run, and the stops they have called at
    std::printf("    [river] buses %u on %zu lines; %llu stops called at; their legs took %.2f times their free-flow time\n",
                sim.drivingHost().counts().buses, d.model()->busLines.size(),
                static_cast<unsigned long long>(sim.drivingHost().counts().busCalls),
                sim.drivingHost().counts().busFreeFlow > 0 ? sim.drivingHost().counts().busDriven / sim.drivingHost().counts().busFreeFlow : 0.0);
    // THE PEOPLE ON THE BUSES (ADR-0484): the residents' rides and the sim's own walkers' together
    {
        const citysim::BusRiders::Counts& rc = sim.buses().counts();
        std::printf("    [river] riders: %llu waits, %llu boardings, %llu got off, a full bus left someone %llu times; residents rode "
                    "%u times (%u waits)\n",
                    static_cast<unsigned long long>(rc.waits), static_cast<unsigned long long>(rc.boardings),
                    static_cast<unsigned long long>(rc.alightings), static_cast<unsigned long long>(rc.leftFull), b.counts().busRides,
                    b.counts().busWaits);
    }
    // WHAT THE PLANS PUT ON THE FREEWAY (#306 sweep bug 4): legs by car priced and chosen, and those with metres on it
    {
        const citysim::brain::PlanRoadCounts rc = b.planRoadCounts();
        std::printf("    [river] legs by car: priced %llu (on the freeway %llu, %.1f km), chosen %llu (on the freeway %llu, %.1f km)\n",
                    static_cast<unsigned long long>(rc.pricedLegs), static_cast<unsigned long long>(rc.pricedOnFreeway),
                    rc.pricedFreewayMetres / 1000.0, static_cast<unsigned long long>(rc.chosenLegs),
                    static_cast<unsigned long long>(rc.chosenOnFreeway), rc.chosenFreewayMetres / 1000.0);
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
    if (!busOf.empty())
        std::printf("    [river] the longest a bus stood in the road (not at a stop): bus %u, %.1f s; its card then:\n%s\n", busStandCar,
                    busStandTicks * kStep, busStandWhy.c_str());
    std::printf("    [river] contact: %llu new contacts, %llu crashes, %llu wrecks, %llu cleared\n",
                static_cast<unsigned long long>(d.crashCounts().contacts), static_cast<unsigned long long>(d.crashCounts().crashes),
                static_cast<unsigned long long>(d.crashCounts().wrecks), static_cast<unsigned long long>(d.crashCounts().cleared));
    if (!end.loops.empty()) std::printf("%s", d.describeWaiting(end).c_str());
    // RT_RIVER_WHO=365: every resident whose brain's description names that car, described (who left it where)
    if (const char* who = std::getenv("RT_RIVER_WHO")) {
        const std::string want = std::string("car ") + who;
        for (int r = 0; r < b.residentCount(); ++r) {
            const std::string text = b.describe(static_cast<citysim::brain::AgentId>(r));
            if (text.find(want + ",") != std::string::npos || text.find(want + " ") != std::string::npos ||
                text.find(want + ")") != std::string::npos)
                std::printf("    [river] resident %d:\n%s\n%s\n", r, text.substr(0, 3000).c_str(), b.traceText(r).c_str());
        }
    }
    // RT_RIVER_ROUTE=365: where that car's lane leads, and whether a route is found from it and from what it leads into
    if (const char* which = std::getenv("RT_RIVER_ROUTE")) {
        const driving::CarId id(static_cast<uint32_t>(std::atoi(which)));
        const driving::Car* c = d.car(id);
        const driving::Trip* t = c ? d.trip(d.tripOf(id)) : nullptr;
        if (c && t) {
            const engine::LaneGraph& g = *d.roads();
            const engine::Path& P = g.paths[c->path.index];
            std::printf("    [river] car %u on path %u (kind %d, %.1f m long, section %u) at %.1f; leads into:", id.index, c->path.index,
                        static_cast<int>(P.kind), P.length, P.section.index, c->along);
            for (uint32_t n = P.next.first; n < P.next.end(); ++n) {
                const engine::PathId q = g.nextPaths[n];
                driving::PathPoint from;
                from.path = q;
                from.along = 0;
                const engine::roads::Route r = d.routeBetween(c->vehicleClass, from, t->destination);
                std::printf(" %u (kind %d, %.1f m; route from it %s)", q.index, static_cast<int>(g.paths[q.index].kind),
                            g.paths[q.index].length, r.found ? "found" : "NOT found");
            }
            driving::PathPoint here;
            here.path = c->path;
            here.along = c->along;
            std::printf("; a route from where it is: %s\n", d.routeBetween(c->vehicleClass, here, t->destination).found ? "found" : "NOT found");
        }
    }
    // RT_RIVER_CARDS=365,12: those cars' driver cards at the end (following a chain of waits by hand)
    if (const char* cards = std::getenv("RT_RIVER_CARDS")) {
        std::string list(cards);
        for (std::size_t at = 0; at < list.size();) {
            const std::size_t comma = std::min(list.find(',', at), list.size());
            const int id = std::atoi(list.substr(at, comma - at).c_str());
            if (d.car(driving::CarId(static_cast<uint32_t>(id))))
                std::printf("    [river] card:\n%s\n", d.describeCar(driving::CarId(static_cast<uint32_t>(id))).substr(0, 1800).c_str());
            at = comma + 1;
        }
    }
    // WHY THE CARS STANDING AT THE END STAND: the rule each one's last decision named (its card's "now:" line)
    {
        std::map<std::string, int> why;
        for (const driving::Car& c : d.cars()) {
            if (!driving::isWaiting(c)) continue;
            const std::string card = d.describeCar(c.id);
            const std::size_t at = card.find("by rule \"");
            const std::size_t to = at == std::string::npos ? at : card.find('"', at + 9);
            ++why[at == std::string::npos ? std::string("(no rule named)") : card.substr(at + 9, to - at - 9)];
            static int shown = 0;
            if (card.find("now: the end of its way") != std::string::npos && shown++ < 3)
                std::printf("    [river] standing at the end of its way:\n%s\n", card.substr(0, 1500).c_str());
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
    // RE-PINNED when a drive is priced by the driving's router, kerb lane to kerb lane at the limit each road is driven
    // at (ADR-0480, the sweep's bugs, 4; it was priced along the sidewalks): the most on the road 102 -> 111 (still
    // minute 11); parked 245 -> 229; arrivals 675 -> 663; trips the same, 316. 566 car legs priced, none on the freeway:
    // river_town's trips never pay for its ramps (brain_crowd checks a drive that does).
    // RE-PINNED when the buses drive their lines (ADR-0483): 6 buses on the 3 lines of its road file, each driving to
    // its next stop, standing there 20 s, and on. The most on the road 111 -> 120 (still minute 11: the buses among
    // them); arrivals 663 -> 665; trips and parked the same (316, 229). Nobody rides them yet (step 4).
    // RE-PINNED when the brain took its people from the city's own households (ADR-0490; the one-brain plan's step 1):
    // the crowd generator's 630 invented residents at invented places became 630 people of 343 of the households its
    // dwellings were filled with (drawn in a seeded order; 84 more were passed over, their home with too few kerb spaces
    // for their cars or walked to from nowhere), at home in their real buildings, a worker going to their own job (329 of
    // them) and home, everyone else staying home. Households 582 -> 343, cars 397 -> 180 (a child or a hall student
    // drives none), residents with one 427 -> 263, markers 799 -> 284 (one per real home or job used). The crowd's days
    // sent 94% of people to work at least 900 m away; real jobs are mostly near, so most walk: trips by car 433 -> 92,
    // parked 277 -> 69, the most on the road 206 at minute 10 -> 37 at minute 21, arrivals 597 -> 1282; walks blocked
    // 2 -> 0, another place 1 -> 0, no plan 1 -> 0. Touching spells 2 -> 0 (#327's check passes on this run).
    // A NEW FAILURE, the bus's: a motorcycle (car 118) stands at junction 62's far turn from minute 6 with bus 6 behind
    // it, for the sim's own walkers standing at rest in the carriageway across the turn (#333).
    // RE-PINNED when the brain's people keep their ROLE'S DAY (ADR-0491; the one-brain plan's step 2a): the sim's role
    // table on the sim's clock, going where the sim's own choosers send them (lunch, an outing, an errand, a campus
    // break), in place of people.lua's stand-in round of minutes. The test's clock runs 10:31 to 11:00, so at minute 1
    // every worker is due at work and every day-out role goes out: 288 set out at once (spread over ticks, eight a tick,
    // so the brain's worst tick is 5.6 ms, not 129). The stand-in's draws are gone, so the seeded households are others:
    // households 343 -> 337, cars 180 -> 204, residents with one 263 -> 296; markers at the start 284 -> 115 (only the
    // homes now: a job's or a venue's marker is made when a day first goes there). Trips by car 92 -> 72, parked 69 ->
    // 65, the most on the road 37 at minute 21 -> 63 at minute 5 (the morning's late start, all at once); arrivals 1282
    // -> 392 (the stand-in sent workers back and forth every few minutes; a real day stays at work).
    // RE-PINNED when the brain runs EVERY household (ADR-0493; the one-brain plan, step 4; river_town's brainPeople
    // 630 -> -1): 5115 people of 2686 households, of the sim's 5149 (16 households left out, their home with no street
    // beside its door). No home needs 24 kerb spaces any more (a household has the cars it can park: 899 cars found no
    // free space), a car names its space, and people at home or work are indoors (people.lua intent be_at), sharing a
    // tower's door. Residents 630 -> 5115, households 337 -> 2686, cars 204 -> 824 (with one 296 -> 1212), home markers
    // 115 -> 278; trips by car 72 -> 164, parked 65 -> 151, the most on the road 63 at minute 5 -> 117 at minute 6;
    // arrivals 392 -> 3281. The day: set out 436 -> 3611, arrived 392 -> 3247, at work 218 -> 1767, still nobody at a
    // shut place or outside their shift; nowhere 2 -> 8 and no way 0 -> 62 (places no walk reaches: a day gives up on
    // one and wants to be where it is), plans with no way 0 -> 317 (nine people stay in no_way, their own last place cut
    // off on foot from where they stand).
    // RE-PINNED when a resting day sleeps until its next event, near people with nothing to do are not stepped and
    // trips due at once wait their turn first come first served (ADR-0496): the most on the road 117 -> 120 (still
    // minute 6), trips by car 164 -> 169, parked 151 -> 156, arrivals 3281 -> 3249; set out 3611 -> 3604, arrived 3247
    // -> 3215, at work 1767 the same; no way 62 -> 64, plans with no way 317 -> 332. The brain 0.87 -> 0.37 ms a tick.
    // RE-PINNED when the old sim's people went (ADR-0499; the one-brain plan, step 2b): its goal layer, tiers and sleep
    // queue deleted, the sim's step only moves bodies out on the street. The 34 people the brain does not run (16
    // households whose home has no street beside its door) no longer walk the old sim's days: trips by car 169 -> 170,
    // parked 156 -> 153, arrivals 3249 -> 3259; set out 3604 -> 3600, arrived 3215 -> 3224, at work 1767 -> 1766, no
    // way 64 -> 63, plans with no way 332 -> 318. The whole step 2.9 ms a tick (the brain 0.29).
    // RE-PINNED when a block's and a lot's dice came from their own outlines and the blocks were taken in the order they
    // stand (ADR-0503, #317 step 6a; lots tag 2026-11-25.1): every lot regrown once, so other buildings, dwellings and
    // jobs. Residents 5115 -> 5134, households 2686 -> 2699, cars 824 -> 787, residents with one 1212 -> 1159, home
    // markers 278 -> 291; the most on the road 120 -> 127 (still minute 6); trips by car 170 -> 165, parked 153 -> 155,
    // arrivals 3259 -> 3357; set out 3600 -> 3667, arrived 3224 -> 3317, at work 1766 -> 1782, nowhere 8 -> 4, no way
    // 63 -> 50, plans with no way 318 -> 178. The longest stand 1456.0 s (car 482, waiting to change lanes at the end of
    // its way, #328's kind).
    // RE-PINNED when the lots were grown, cut and written a tile at a time (ADR-0504, #317 step 6b; lots tag
    // 2026-11-25.2): every block's number now 31 bits (so other dice), the blocks' records in tile order. Residents 5134
    // -> 5106, households 2699 -> 2715, cars 787 -> 865, residents with one 1159 -> 1280, home markers 291 -> 280; the
    // most on the road 127 -> 137 (still minute 6); trips by car 165 -> 179, parked 155 -> 163, arrivals 3357 -> 3227;
    // set out 3667 -> 3520, arrived 3317 -> 3200, at work 1782 -> 1759, nowhere 4 the same, no way 50 -> 42, plans with
    // no way 178 -> 238. A NEW FAILURE, the buses' check below: bus 11 stood 192.3 s at junction 50's red light
    // (its far turn), over the 180 s it allows -- not re-pinned, for the driving's owner to judge.
    if (minutes == kRiverMinutes) {
        CHECK(b.residentCount() == 5106);
        CHECK(households == 2715);
        CHECK(b.cars().size() == 865u);
        CHECK(withCar == 1280);
        CHECK(homeMarkers == 280u);
        CHECK(mostOnRoad == 137);
        CHECK(mostOnRoadAt == 6);
        CHECK(b.counts().tripsBegun == 179u);
        CHECK(b.counts().parkedReports == 163u);
        CHECK(b.counts().walksBlocked == 0u);
        CHECK(b.counts().otherPlaces == 0u);
        CHECK(b.machine()->counts().arrivals == 3227u);
        // THE DAY (ADR-0491): nobody arrives at a place that is shut; every worker reaching their job does so inside their
        // shift; 436 trips begun, 392 arrived, 218 at work. Nowhere to go twice (one person's job has no street beside
        // it: their day tries again each quarter hour), no way 0.
        CHECK(b.counts().dayArrivedShut == 0u);
        CHECK(b.counts().dayToWorkOffShift == 0u);
        CHECK(b.counts().daySetOut == 3520u);
        CHECK(b.counts().dayArrivals == 3200u);
        CHECK(b.counts().dayToWork == 1759u);
        CHECK(b.counts().dayNowhere == 4u);
        CHECK(b.counts().dayNoWay == 42u);
        CHECK(b.machine()->counts().noPlans == 238u);
        // (failing with lane changes on, 2 spells, #327; none since ADR-0490's people)
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
        // 0x12277ea4937a87e4 -> 0x331264b35e85398b (a drive priced by the driving's router): the counts above.
        // 0x331264b35e85398b -> 0x96738d598af1e6e2 (ADR-0483: the buses drive their lines): the counts above.
        // 0x96738d598af1e6e2 -> 0x84834e8b10e27894 (ADR-0484: people ride the buses, and the fleet is sized by the
        // measured traffic factor): most on the road 120 -> 126, trips 316 -> 312, parked 229 -> 223, arrivals 665 -> 658
        // (a bus trip is a trip the brain counts as one arrival, after a wait and a ride, where a drive was before)
        // 0x84834e8b10e27894 -> 0xf5c3d679818d2ee3 (ADR-0485: a post and a bench at each of the 38 stops, and the
        // street trees within 3 m of one gone, so the walkers' obstacles moved): parked 223 -> 225, arrivals 658 -> 660;
        // the riders' counts are the same
        // 0xf5c3d679818d2ee3 -> 0xd99d3758b0039615 (a walker's crossing state goes with the near tier, city_sim.cpp
        // setTier): a walker that went far kept "on crosswalk N" from its last near step, and the drivers gave way to it
        // for ever -- bus 7 stood 410 s in junction 53 for two children a block past crosswalk 219. The longest a bus
        // stood in the road 410 -> 87 s (a red light); most on the road 126 -> 124 (minute 11 -> 10), trips 312 -> 321,
        // parked 225 -> 245, arrivals 660 -> 672,
        // the buses' calls 154 -> 167, their legs 4.17 -> 3.79 times the free-flow time
        // 0xd99d3758b0039615 -> 0xc5fa9c993e9148d7 (sitting 5: cars pull out of and into kerb spaces on the driving's
        // curve, no drawn 2.5 s slide; the space a driver heads for is held for it): most on the road 124 -> 125 (still
        // minute 10), trips 321 -> 320, parked 245 -> 247, arrivals 672 -> 676, boardings 40 -> 45; rings and touching
        // spells still 0; the longest a bus stood in the road 87 -> 102 s (under the 180 s check); legs 3.79 -> 3.83 times
        // 0xc5fa9c993e9148d7 -> 0xf4b923be54a7b75b (sitting 5: LANE CHANGES ON BY DEFAULT): homes placed where routes
        // reach, so cars 391 -> 397; more drive: trips 320 -> 430, the most on the road 125 -> 200 (minute 10), parked
        // 247 -> 275, arrivals 676 -> 590, the buses' calls 167 -> 152, riders 45 -> 20 boardings. WORSE, and two checks
        // fail: 2 touching spells and bus 7 stood 1150 s (#326: a car left parked in a lane past junction 11; #327)
        // 0xf4b923be54a7b75b -> 0x67a1b7581e4e1dac (sitting 6: junctions by judgement): most on the road 200 -> 201,
        // trips 430 -> 419, parked 275 -> 250, arrivals 590 -> 570, bus calls 152 -> 144, boardings 20 -> 18; rings 0;
        // touching spells 2 -> 1. The same two failures (#326, #327).
        // 0x67a1b7581e4e1dac -> 0x86dd6401bb40c85c (sitting 7: contact and crashes; a car nobody drives standing on a
        // lane is a thing in drivers' way): every count above the same; 1 new contact, no crash rolled.
        // 0x86dd6401bb40c85c -> 0xeb213119d367acfd (#326: a car missing its turn with no way on from its lane is no longer
        // reported "no route" to the host -- its resident got out and left it standing in the lane past junction 11, and
        // the queue behind it, a bus among them, stood for good): most on the road 201 -> 203, trips 419 -> 444, parked
        // 250 -> 303, arrivals 570 -> 612, bus calls 144 -> 166, bus rides 10 -> 19; the longest a bus stood in the road
        // 1150 s -> 99 s (passes). Car 365 still stands at the end of a 20 m lane piece waiting to change lanes with no
        // room to (#328).
        // 0xeb213119d367acfd -> 0x537a4261659664d2 (the alpha review, P1-1): the lane-change safety check read the
        // follower behind a gap by id as if it were a record index (the crowd's ids are sparse) -- now the real one, so
        // more changes wait for a safe gap: most on the road 203 -> 206, trips 444 -> 433, parked 303 -> 277, arrivals
        // 612 -> 597, bus calls 166 -> 165; the longest a bus stood in the road 99 s -> 117 s (passes)
        // 0x537a4261659664d2 -> 0x982f4b5f055d37a8 (#328: as brain_crowd's; buses end at their stops in the kerb lane):
        // every count the same
        // 0x982f4b5f055d37a8 -> 0x0c184c57be58cc6a (ADR-0490: the brain's people are the city's own households): the
        // counts above, and the buses' below
        // 0x0c184c57be58cc6a -> 0x4d453ad8fdb637d9 (ADR-0491: the brain's people keep their role's day): the counts above
        // and the buses' below
        // 0x4d453ad8fdb637d9 -> 0x7ed5a8a9fec97fa4 (ADR-0493: the brain runs every household): the counts above, and
        // the buses' below
        // 0x7ed5a8a9fec97fa4 -> 0x0e41f1185e8239f8 (ADR-0496: resting days sleep, and the rest): the counts above
        // 0x0e41f1185e8239f8 -> 0x5ab02c088a376691 (ADR-0499: the old sim's people gone): the counts above
        // 0x5ab02c088a376691 -> 0xf535cd1ac2925c02 (ADR-0503: every lot regrown once, seeds from outlines): the counts
        // above and the buses' below
        // 0xf535cd1ac2925c02 -> 0xb25113df86e3d71d (ADR-0504: the lots grown a tile at a time): the counts above
        CHECK(hash == 0xb25113df86e3d71dull);
        // THE BUSES (ADR-0483, ADR-0484): 13 on the 3 lines (the service file's headway over each round trip at the
        // measured traffic factor), calling at 154 stops in the 30 minutes
        CHECK(sim.drivingHost().counts().buses == 13u);
        // (165 -> 147, ADR-0490: bus 6 stood behind car 118, #333; 147 -> 160, ADR-0491: bus 4 stood there instead)
        // (160 -> 182, ADR-0493: nobody stands in junction 62's turn now that the old sim walks nobody, #333)
        CHECK(sim.drivingHost().counts().busCalls == 181u);   // (182 -> 180, ADR-0496; 180 -> 184, ADR-0503; 184 -> 181, ADR-0504)
        // ...and no bus stands in the road, away from a stop, for minutes: a red light and a queue are the most it waits
        // (87 s); a bus held by something that never goes was 410 s
        // (FAILING from ADR-0490 to ADR-0491, the sim's own walkers standing in junction 62's turn, #333; passes since
        // ADR-0493, the old sim walking nobody: bus 9, 87.2 s)
        CHECK(busStandTicks * kStep < 180.0);
        // THE RIDERS (ADR-0484): residents and the sim's own walkers wait, board and get off; no full bus left anyone
        {
            const citysim::BusRiders::Counts& rc = sim.buses().counts();
            // (ADR-0490: fewer far trips, so fewer by bus: waits 25 -> 11, boardings 20 -> 3, got off 19 -> 2; the
            // residents' waits 21 -> 3, rides 19 -> 2)
            // (ADR-0491: nobody: waits 11 -> 0, boardings 3 -> 0, got off 2 -> 0; the residents' waits 3 -> 0, rides
            // 2 -> 0. The sim's own walkers began 2482 trips, a ride found for each, none beating the walk by
            // kBusBeatsWalk; none of the brain's 218 to work went by bus: #335)
            CHECK(rc.waits == 0u && rc.boardings == 0u && rc.alightings == 0u && rc.leftFull == 0u);
            CHECK(b.counts().busWaits == 0u && b.counts().busRides == 0u);
            // (the check that some of the sim's own walkers rode too went with them, ADR-0499: every rider is the brain's)
        }
    }
}

// FAR PEOPLE KEEP THEIR DAY (ADR-0493; the one-brain plan, step 4). With the player standing north of the river by
// junction 62 (the busiest place, test_near_tier's kRiverTownBusy), the brain's people within the level's
// pedPromoteRadius are near and the rest far: no skill moves a far person, the brain's clock ends their legs, and they
// are visited only then. The same day goes on: they set out, arrive, and get to work inside their shift, nobody at a
// shut place; people come near and go far as the attention allows; nothing errs; and the brain costs less a tick.
TEST_CASE(level_river_town_far_people_keep_their_day) {
    crowdtest::LoadedCrowd lc(std::getenv("RT_FAR_LEVEL") ? std::getenv("RT_FAR_LEVEL") : "river_town.json");
    CHECK(lc.ok);
    if (!lc.ok) return;
    citysim::CitySim& sim = lc.city.simMutable();
    citysim::CityBrain* bp = sim.brain();
    CHECK(bp != nullptr && bp->hasResident());
    if (!bp || !bp->hasResident()) return;
    citysim::CityBrain& b = *bp;
    const engine::Vec2 player(-50, 750);
    const int ticksPerMinute = static_cast<int>(60.0 / kStep + 0.5);
    const int kFarMinutes = std::getenv("RT_FAR_MINUTES") ? std::atoi(std::getenv("RT_FAR_MINUTES")) : 15;
    int mostFar = 0;
    for (int minute = 1; minute <= kFarMinutes; ++minute) {
        for (int t = 0; t < ticksPerMinute; ++t) {
            sim.setTierCenter(player);
            lc.city.step(lc.world, kStep);
        }
        mostFar = std::max(mostFar, b.farCount());
        std::printf("    [far] %2d min: %s\n", minute, b.attentionText().c_str());
        const citysim::CityBrainCounts& c = b.counts();
        std::printf("    [far] %2d min: far %d of %d | lowered %u, raised %u, far visits %llu | the day: set out %u, arrived %u, "
                    "nowhere %u, no way %u, at a shut place %u, at work %u (%u outside the shift) | brain %.3f ms a tick "
                    "(worst %.3f)\n",
                    minute, b.farCount(), b.residentCount(), c.lowered, c.raised, static_cast<unsigned long long>(c.farVisits),
                    c.daySetOut, c.dayArrivals, c.dayNowhere, c.dayNoWay, c.dayArrivedShut, c.dayToWork, c.dayToWorkOffShift,
                    b.cost().ms / std::max<uint64_t>(1, b.cost().ticks), b.cost().worstMs);
    }
    const citysim::CityBrainCounts& c = b.counts();
    const uint64_t hash = crowdtest::crowdHash(b, sim, lc.world);
    std::printf("    [far] at the end: arrivals %llu, errors %llu, refused %llu; final state hash %016llx\n",
                static_cast<unsigned long long>(b.machine()->counts().arrivals),
                static_cast<unsigned long long>(b.machine()->counts().errors),
                static_cast<unsigned long long>(b.machine()->counts().refused), static_cast<unsigned long long>(hash));
    CHECK(c.dayArrivedShut == 0u);
    CHECK(c.dayToWorkOffShift == 0u);
    CHECK(b.machine()->counts().errors == 0u);
    CHECK(b.machine()->counts().refused == 0u);
    // PINNED from the run of 2026-11-24 (ADR-0493; the whole town, 5115 people): about half far at any minute (the most
    // 2724; the player stands in the town's busiest, densest part: 1453 to 1836 within the 300 m ring, 736 to 809
    // between the rings, 39 to 135 near for a car); 3062 times lowered, 558 raised, 2906 far visits; 2823 set out, 2296
    // arrived, 1528 at work; nowhere 4, no way 30; the brain 0.98 ms a tick (worst 6.7).
    // RE-PINNED when far walkers are drawn by the GPU crowd along their route (ADR-0495): one lowered part-way along a
    // walk goes on from where they were (not the straight line between the leg's ends), and one raised is put where the
    // crowd had them. The most far 2724 -> 2718; lowered 3062 -> 3070, raised 558 -> 568, far visits 2906 -> 2889; set
    // out 2823 -> 2820, arrived 2296 -> 2293, at work 1528 -> 1521; 0x17ca8289b402619b -> 0xd898a2ec3bb6bbe9.
    // RE-PINNED when resting days sleep, far people are visited from compact arrays in two passes (the distance share,
    // then the scan) and trips due at once wait their turn first come first served (ADR-0496): the most far 2718 ->
    // 2719; lowered 3070 the same, raised 568 -> 569, far visits 2889 -> 2887; set out 2820, arrived 2293 -> 2292, at
    // work 1521 -> 1520; 0xd898a2ec3bb6bbe9 -> 0xe2c8701064b9badc. The brain 1.05 -> 0.53 ms a tick.
    // (ADR-0499, the old sim's people gone: lowered 3070 -> 3068, raised 569 -> 571, far visits 2887 -> 2878; set out
    // 2820 -> 2812, arrived 2292 -> 2283, at work 1520 -> 1519; 0xe2c8701064b9badc -> 0xba9eaef6e5933eb5)
    if (kFarMinutes == 15) {
        CHECK(mostFar == 2719);
        CHECK(c.lowered == 3068u && c.raised == 571u && c.farVisits == 2878u);
        CHECK(c.daySetOut == 2812u && c.dayArrivals == 2283u && c.dayToWork == 1519u);
        CHECK(hash == 0xba9eaef6e5933eb5ull);
    }
}


// WITH NO DRIVING (ADR-0494): the same town with its rule set taken away, as the island and every level the new driving
// does not drive yet are. The brain runs every household all the same: they walk (and would ride), nobody has a car,
// and their role's day goes on -- out, to work inside their shift, home.
TEST_CASE(level_river_town_without_the_driving_the_brain_still_runs_everyone) {
    crowdtest::LoadedCrowd lc("river_town.json", [](engine::CitySimConfig& c) { c.drivingRules.clear(); });
    CHECK(lc.ok);
    if (!lc.ok) return;
    citysim::CitySim& sim = lc.city.simMutable();
    CHECK(!sim.drivingHost().active());
    citysim::CityBrain* bp = sim.brain();
    CHECK(bp != nullptr && bp->hasResident());
    if (!bp || !bp->hasResident()) return;
    citysim::CityBrain& b = *bp;
    const int ticksPerMinute = static_cast<int>(60.0 / kStep + 0.5);
    constexpr int kMinutes = 10;
    for (int minute = 1; minute <= kMinutes; ++minute) {
        for (int t = 0; t < ticksPerMinute; ++t) lc.city.step(lc.world, kStep);
        const citysim::CityBrainCounts& c = b.counts();
        std::printf("    [no-driving] %2d min: the day: set out %u, arrived %u, nowhere %u, no way %u, at a shut place %u, at "
                    "work %u (%u outside the shift) | brain %.3f ms a tick (worst %.3f)\n",
                    minute, c.daySetOut, c.dayArrivals, c.dayNowhere, c.dayNoWay, c.dayArrivedShut, c.dayToWork,
                    c.dayToWorkOffShift, b.cost().ms / std::max<uint64_t>(1, b.cost().ticks), b.cost().worstMs);
    }
    const citysim::CityBrainCounts& c = b.counts();
    std::printf("    [no-driving] %d people, %zu cars; arrivals %llu, errors %llu, refused %llu\n", b.residentCount(), b.cars().size(),
                static_cast<unsigned long long>(b.machine()->counts().arrivals),
                static_cast<unsigned long long>(b.machine()->counts().errors),
                static_cast<unsigned long long>(b.machine()->counts().refused));
    CHECK(b.residentCount() > 5000);
    CHECK(b.cars().empty());
    CHECK(c.tripsBegun == 0u);
    CHECK(c.dayArrivals > 1000u && c.dayToWork > 500u);
    CHECK(c.dayArrivedShut == 0u && c.dayToWorkOffShift == 0u);
    CHECK(b.machine()->counts().errors == 0u && b.machine()->counts().refused == 0u);
    // PINNED from the run of 2026-11-24 (ADR-0494): 2568 set out, 2087 arrived, 1419 at work in ten minutes; 2104
    // arrivals in all; the brain 0.84 ms a tick (worst 3.2). RE-PINNED when a resting day sleeps until its next event
    // (ADR-0496; its time in the state caught up in one sum on waking, not tick by tick, so a stay ends a tick either
    // way): 2569 set out, 2100 arrived, 1433 at work, 2117 arrivals; the brain 0.84 -> 0.41 ms a tick.
    // (ADR-0499, the old sim's people gone: set out 2569 -> 2567, arrived 2100 -> 2095, arrivals 2117 -> 2112)
    CHECK(c.daySetOut == 2567u && c.dayArrivals == 2095u && c.dayToWork == 1433u);
    CHECK(b.machine()->counts().arrivals == 2112u);
}
