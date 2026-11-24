// A CROWD OF RESIDENTS DRIVING ON THE TRAFFIC LAB'S ROADS (ADR-0446; docs/two-drivers-meeting-design.md section 12,
// sitting 2).
//
// Loads assets/levels/brain_crowd.json through the real LevelLoader and CityRenderSystem (a NullRenderer underneath):
// the traffic lab's roads, and a crowd made when the level loads from assets/scripts/crowd.lua and the road model
// (city_crowd.h): about 350 residents in households (200 at ADR-0446, 350 since sitting 2b), the households' cars in kerb spaces near home, each person a day
// round places spread along the streets. The city is stepped as the viewer steps it, 60 times a second, for
// kCrowdMinutes of the lab's clock (RT_CROWD_MINUTES for another stretch; the pinned numbers then do not apply).
//
//   RT_TEST_FILTER=level_brain_crowd ./build-viewer/level_tests
//
// Every simulated minute it reports: the residents by state, the cars on the road, trips begun, parked and failed to
// plan, the longest a car has stood still on the road (reported, NOT a limit: Glenn ruled out timeouts), rings of waits
// in the wait-for graph, and what a tick cost. Held: no ring of waits at any minute (a ring of cars each waiting on the
// next, which nothing can end); what the crowd is made of, the busiest minute, the counts and the state hash of the run
// (pinned from a run that was looked at), with how many minutes were busy (kBusyCars or more on the road) and how
// often somebody with no way to a place chose another of its kind. The busy period has 59-74 cars on the road for six
// minutes, not the hundred asked for (ADR-0446 says why).

#include "test_framework.h"

#include "crowd_test_rig.h"
#include "../src/apps/citysim/city_brain.h"
#include "../src/apps/citysim/city_car.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/apps/citysim/trip_plan.h"
#include "../src/apps/driving/wait_graph.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/world.h"
#include "../src/hash.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

constexpr Real kStep = 1.0 / 60.0;   // the viewer's fixed step, and the sim's tick
// The stretch of the day the test runs (simulated minutes): everybody's first trip out and most of the trips back.
constexpr int kCrowdMinutes = 30;
// A minute is counted busy with this many cars on the road at its end (reported and pinned, not a rule).
constexpr int kBusyCars = 50;
// A walker standing at a crosswalk's kerb is this near (m) its kerb end's waiting place (ADR-0452): a few people waiting
// at one kerb end are spread round it by the crowd's own spacing (CitySim's overlap push), so not all on the one point.
constexpr Real kAtTheKerbEnd = 2.5;
// ...and fewer than one sample in this many is not (the spread junctions' known few).
constexpr uint64_t kAwayShareOneIn = 20;

using crowdtest::LoadedCrowd;   // (crowd_test_rig.h: shared with test_river_town_crowd.cpp)
using crowdtest::crowdHash;

}  // namespace

TEST_CASE(level_brain_crowd_the_residents_make_their_day) {
    LoadedCrowd lc("brain_crowd.json");
    CHECK(lc.ok);
    if (!lc.ok) return;
    citysim::CitySim& sim = lc.city.simMutable();
    citysim::CityBrain* bp = sim.brain();
    CHECK(bp != nullptr && bp->hasResident());
    if (!bp || !bp->hasResident()) return;
    citysim::CityBrain& b = *bp;
    const driving::Driving& d = *sim.drivingV2();
    World& world = lc.world;
    const int minutes = std::getenv("RT_CROWD_MINUTES") ? std::atoi(std::getenv("RT_CROWD_MINUTES")) : kCrowdMinutes;

    // WHAT THE CROWD IS MADE OF
    int households = 0;
    {
        std::map<std::string, int> houses;
        for (int who = 0; who < b.residentCount(); ++who) ++houses[b.householdName(who)];
        households = static_cast<int>(houses.size());
    }
    int withBoard = 0, withScooter = 0, withCar = 0;
    for (int who = 0; who < b.residentCount(); ++who) {
        const std::string line = b.describe(who);
        withBoard += line.find("traits: has_board") != std::string::npos || line.find(", has_board") != std::string::npos ? 1 : 0;
        withScooter += line.find("has_scooter") != std::string::npos ? 1 : 0;
        withCar += b.residentCar(who) >= 0 ? 1 : 0;
    }
    std::printf("    [crowd] %d residents in %d households, %zu cars (%d residents in a household with one), %zu markers; "
                "%d with a board, %d with a scooter\n",
                b.residentCount(), households, b.cars().size(), withCar, b.markers().size(), withBoard, withScooter);

    // A DRIVE ACROSS TOWN BETWEEN THE FREEWAY'S RAMPS IS PRICED ON THE FREEWAY (#306 sweep bug 4: a drive was priced over
    // the sidewalks, and no walk runs along a freeway). From the street feeding one on-ramp to the street an off-ramp
    // leads to, the farthest such pair: the trip planner's car leg (planLeg, by the driving's router) has freeway metres.
    {
        const engine::RoadModel& m = *d.model();
        const engine::LaneGraph& g = m.laneLevel;
        const auto klassOf = [&](uint32_t path) {
            const engine::ModelStreet* st = m.streetOf(engine::PathId(path));
            return st ? st->section.klass : engine::RoadClass::Local;
        };
        std::vector<uint32_t> feeders, outlets;   // a lane leading onto an on-ramp; a lane an off-ramp leads onto
        for (uint32_t r = 0; r < g.paths.size(); ++r) {
            if (klassOf(r) != engine::RoadClass::Ramp || g.paths[r].kind != engine::PathKind::Lane) continue;
            for (uint32_t n = g.paths[r].next.first; n < g.paths[r].next.end() && n < g.nextPaths.size(); ++n) {
                const uint32_t o = g.nextPaths[n].index;
                if (klassOf(o) != engine::RoadClass::Freeway) {
                    outlets.push_back(o);
                    continue;
                }
                for (uint32_t j = 0; j < g.paths.size(); ++j)
                    for (uint32_t k = g.paths[j].next.first; k < g.paths[j].next.end() && k < g.nextPaths.size(); ++k)
                        if (g.nextPaths[k].index == r && klassOf(j) != engine::RoadClass::Ramp && klassOf(j) != engine::RoadClass::Freeway)
                            feeders.push_back(j);
            }
        }
        engine::Vec2 from, to;
        engine::Real apart = -1;
        for (const uint32_t f : feeders)
            for (const uint32_t o : outlets) {
                const engine::Vec2 a = g.pointAt(engine::PathId(f), g.paths[f].length * 0.5);
                const engine::Vec2 c = g.pointAt(engine::PathId(o), g.paths[o].length * 0.5);
                if ((a - c).length() > apart) {
                    apart = (a - c).length();
                    from = a;
                    to = c;
                }
            }
        citysim::TravelCatalog cat;
        citysim::TravelMode car;
        car.name = "car";
        car.roads = true;
        cat.modes = {car};
        citysim::RoadKind streets, freeway;
        streets.name = "streets";
        streets.classes = {engine::RoadClass::Arterial, engine::RoadClass::Collector, engine::RoadClass::Local, engine::RoadClass::Ramp};
        freeway.name = "freeway";
        freeway.classes = {engine::RoadClass::Freeway};
        cat.roadKinds = {streets, freeway};
        const engine::roads::RouteWalker walker = engine::roads::bindWalker(m, "walker", 1.4);
        citysim::TripLeg leg;
        const bool priced = apart > 0 && citysim::planLeg(citysim::TripNetwork{&m, &walker, &d}, cat, 0, from, to, leg);
        const engine::Real onFreeway = priced ? citysim::kindMetres(cat, leg, "freeway") : 0;
        std::printf("    [crowd] across town by the freeway: %zu on-ramp feeders, %zu off-ramp outlets; (%.0f %.0f) -> (%.0f %.0f), "
                    "%.0f m apart: a drive of %.0f m in %.2f min, %.0f m of it on the freeway\n",
                    feeders.size(), outlets.size(), from.x, from.y, to.x, to.y, apart, leg.metres, leg.minutes, onFreeway);
        CHECK(priced && onFreeway > 0);
    }
    // THE DAY, minute by minute
    const int ticksPerMinute = static_cast<int>(60.0 / kStep + 0.5);
    int mostOnRoad = 0, mostOnRoadAt = 0, ringsSeen = 0, busyMinutes = 0;
    double stepMsSum = 0, stepWorst = 0;
    std::map<uint32_t, int> standingSince;   // car id -> the tick it last stood still from
    int longestStandTicks = 0;
    uint32_t longestStandCar = 0;
    // ...AT A LIT JUNCTION (ADR-0452): standing on a lane into one, or inside one
    std::map<uint32_t, int> litSince;
    int longestLitTicks = 0;
    uint32_t longestLitCar = 0, longestLitJunction = 0;
    // ...WALKERS STANDING AT A CROSSWALK'S KERB (ADR-0452): how far from the model's waiting place
    Real farthestFromKerb = 0;
    int farthestFromKerbWho = -1;
    std::string farthestFromKerbWhy;
    uint64_t kerbWaitSamples = 0, kerbWaitAway = 0;
    const auto litJunctionOf = [&](const driving::Car& c) -> int {
        const engine::LaneGraph& roads = *d.roads();
        const engine::Path& P = roads.paths[c.path.index];
        engine::JunctionId j = P.junction;
        if (P.kind == engine::PathKind::Lane && P.next.count > 0) j = roads.paths[roads.nextPaths[P.next.first].index].junction;
        return j.valid() && j.index < d.rightOfWay().junctions.size() && d.rightOfWay().junctions[j.index].running()
                   ? static_cast<int>(j.index) : -1;
    };
    for (int minute = 1; minute <= minutes; ++minute) {
        for (int t = 0; t < ticksPerMinute; ++t) {
            const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
            lc.city.step(world, kStep);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            stepMsSum += ms;
            stepWorst = std::max(stepWorst, ms);
            const int tick = (minute - 1) * ticksPerMinute + t;
            // the longest a car on the road has stood still (the wait-for graph's "waiting": at rest, not moving off)
            for (const driving::Car& c : d.cars()) {
                if (!driving::isWaiting(c)) {
                    standingSince.erase(c.id.index);
                    continue;
                }
                const std::map<uint32_t, int>::iterator it = standingSince.emplace(c.id.index, tick).first;
                if (tick - it->second > longestStandTicks) {
                    longestStandTicks = tick - it->second;
                    longestStandCar = c.id.index;
                }
            }
            for (const driving::Car& c : d.cars()) {
                const int lit = driving::isWaiting(c) ? litJunctionOf(c) : -1;
                if (lit < 0) {
                    litSince.erase(c.id.index);
                    continue;
                }
                const std::map<uint32_t, int>::iterator it = litSince.emplace(c.id.index, tick).first;
                if (tick - it->second > longestLitTicks) {
                    longestLitTicks = tick - it->second;
                    longestLitCar = c.id.index;
                    longestLitJunction = static_cast<uint32_t>(lit);
                }
            }
            // EVERY WALKER STANDING AT A CROSSWALK'S KERB stands at its kerb end (ADR-0452): the farthest any is from
            // the nearer of the model's two waiting places, every tick
            for (const citysim::Agent& a : sim.agents()) {
                if (!a.moving || a.crosswalkAt < 0 || a.onCrosswalk || a.speed > 0.05) continue;
                const engine::ModelCrosswalk* cw = d.model()->crosswalk(static_cast<uint32_t>(a.crosswalkAt));
                if (!cw) continue;
                const Real off = std::min((a.pos - cw->kerbWait(0, citysim::CitySim::kCrossingKerbStep)).length(),
                                          (a.pos - cw->kerbWait(1, citysim::CitySim::kCrossingKerbStep)).length());
                if (off > farthestFromKerb) {
                    farthestFromKerb = off;
                    farthestFromKerbWho = static_cast<int>(&a - sim.agents().data());
                    const citysim::CitySim::CrossingAhead x = sim.nextCrossing(a);
                    char line[200];
                    std::snprintf(line, sizeof line, "minute %d: hold why %d, crosswalk %d, to hold %.2f m, to far %.2f m", minute,
                                  sim.holdWhyOf(farthestFromKerbWho), a.crosswalkAt, x.toHold, x.toFar);
                    farthestFromKerbWhy = line;
                }
                ++kerbWaitSamples;
                kerbWaitAway += off > kAtTheKerbEnd ? 1u : 0u;
            }
        }
        std::map<std::string, int> byState;
        for (int who = 0; who < b.residentCount(); ++who) ++byState[b.stateName(who)];
        int onRoad = 0;
        world.each<citysim::CityCarOnLane>([&](Entity, citysim::CityCarOnLane& lane) { onRoad += lane.driven ? 1 : 0; });
        busyMinutes += onRoad >= kBusyCars ? 1 : 0;
        if (onRoad > mostOnRoad) {
            mostOnRoad = onRoad;
            mostOnRoadAt = minute;
        }
        const driving::WaitReport wr = d.waitReport();
        ringsSeen += static_cast<int>(wr.loops.size());
        // RT_CROWD_LOG=<minute>: WHERE TO LOOK at that minute (tools/brain_lab_probe.py --scenario crowd): the junctions of
        // each control and the ramps' merges, with the cars on the road within kLookReach of each, the busiest first
        if (std::getenv("RT_CROWD_LOG") && std::atoi(std::getenv("RT_CROWD_LOG")) == minute) {
            constexpr Real kLookReach = 40.0;
            std::vector<Vec2> carsAt;
            world.each<citysim::CityCarOnLane>([&](Entity e, citysim::CityCarOnLane& lane) {
                const Transform* t = world.get<Transform>(e);
                if (lane.driven && t) carsAt.push_back(Vec2(t->position.x, t->position.z));
            });
            const auto near = [&](Vec2 p) {
                int n = 0;
                for (const Vec2& c : carsAt) n += (c - p).length() < kLookReach ? 1 : 0;
                return n;
            };
            std::vector<std::pair<int, std::string>> looks;
            for (const engine::ModelJunction& J : d.model()->junctions)
                looks.push_back({near(J.at), "junction " + J.id + " (" + (J.control.empty() ? "none" : J.control) + ") at " +
                                                 std::to_string(J.at.x) + " " + std::to_string(J.at.y)});
            const std::vector<Vec2>& pts = d.model()->lanePoints();
            for (const engine::ModelConnection& c : d.model()->connections) {
                if (c.kind != engine::ConnectionKind::Merge || c.ramp.empty()) continue;
                const engine::ModelLane& L = d.model()->lanes[c.from.index];
                if (L.curve.count == 0) continue;
                const Vec2 gore = pts[L.curve.end() - 1];
                looks.push_back({near(gore), "merge of ramp " + c.ramp + " at " + std::to_string(gore.x) + " " + std::to_string(gore.y)});
            }
            std::sort(looks.begin(), looks.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
            for (const std::pair<int, std::string>& l : looks)
                if (l.first > 0) std::printf("    [crowd] look: %d cars near %s\n", l.first, l.second.c_str());
        }
        std::string states;
        for (const std::pair<const std::string, int>& s : byState) states += " " + s.first + " " + std::to_string(s.second);
        const citysim::CityBrainCounts& c = b.counts();
        const citysim::brain::BrainCounts& mc = b.machine()->counts();
        std::printf("    [crowd] %2d min:%s | %d cars on the road, %u waiting, %zu rings | trips begun %u, parked %u; plans %llu, "
                    "no way %llu, walks blocked %u | longest stand %.1f s (car %u) | brain %.3f ms a tick (worst %.3f), "
                    "driving %.3f ms (worst %.3f), the whole step %.3f ms (worst %.3f)\n",
                    minute, states.c_str(), onRoad, wr.waiting, wr.loops.size(), c.tripsBegun, c.parkedReports,
                    static_cast<unsigned long long>(mc.plans), static_cast<unsigned long long>(mc.noPlans), c.walksBlocked,
                    longestStandTicks * kStep, longestStandCar, b.cost().ms / std::max<uint64_t>(1, b.cost().ticks), b.cost().worstMs,
                    sim.drivingHost().cost().modelMs / std::max<uint64_t>(1, sim.drivingHost().cost().ticks),
                    sim.drivingHost().cost().modelWorstMs, stepMsSum / (minute * ticksPerMinute), stepWorst);
        // CROSSWALKS (ADR-0452): people waiting at kerbs now and crossing now; crossings begun so far by kind; drivers
        // giving way at crosswalks so far by the crosswalk's control, and decisions standing for someone only waiting
        // (both hesitating); and the longest a car has stood at a lit junction
        const citysim::CitySim::WalkStats& ws = sim.walkStats();
        const driving::Driving::ThingCounts& tc = d.thingCounts();
        std::printf("    [crowd]        crosswalks: at kerbs %d, crossing %d (waited %.0f agent-s) | crossings: on walk %llu, at signs "
                    "%llu, marked %llu, unmarked %llu, against the lights %llu | drivers gave way: lights %llu, signs %llu, "
                    "none %llu; stood for a waiter %llu decisions | longest stand at a lit junction %.1f s (car %u, junction %u)\n",
                    sim.walkersWaitingAtKerbs(), sim.walkersOnCrosswalks(), ws.waitingTicks * kStep,
                    static_cast<unsigned long long>(ws.crossings[0]), static_cast<unsigned long long>(ws.crossings[1]),
                    static_cast<unsigned long long>(ws.crossings[2]), static_cast<unsigned long long>(ws.crossings[3]),
                    static_cast<unsigned long long>(ws.crossings[4]), static_cast<unsigned long long>(tc.crosswalkGiveWays[0]),
                    static_cast<unsigned long long>(tc.crosswalkGiveWays[1]), static_cast<unsigned long long>(tc.crosswalkGiveWays[2]),
                    static_cast<unsigned long long>(tc.crosswalkStandOffs), longestLitTicks * kStep, longestLitCar,
                    longestLitJunction);
    }
    // WHAT THE PLANS PUT ON THE FREEWAY (#306 sweep bug 4): legs by car priced and chosen, and those with metres on it
    {
        const citysim::brain::PlanRoadCounts rc = b.planRoadCounts();
        std::printf("    [crowd] legs by car: priced %llu (on the freeway %llu, %.1f km), chosen %llu (on the freeway %llu, %.1f km)\n",
                    static_cast<unsigned long long>(rc.pricedLegs), static_cast<unsigned long long>(rc.pricedOnFreeway),
                    rc.pricedFreewayMetres / 1000.0, static_cast<unsigned long long>(rc.chosenLegs),
                    static_cast<unsigned long long>(rc.chosenOnFreeway), rc.chosenFreewayMetres / 1000.0);
    }
    const driving::WaitReport end = d.waitReport();
    const uint64_t hash = crowdHash(b, sim, world);
    std::printf("    [crowd] at the end: %zu rings of waits (%u gridlocks, %u deadlocks); the most cars on the road at once %d "
                "(minute %d), %d minutes with %d or more; another place of the kind chosen %u times; arrivals %llu; errors "
                "%llu, refused %llu; final state hash %016llx\n",
                end.loops.size(), end.gridlocks, end.deadlocks, mostOnRoad, mostOnRoadAt, busyMinutes, kBusyCars,
                b.counts().otherPlaces,
                static_cast<unsigned long long>(b.machine()->counts().arrivals),
                static_cast<unsigned long long>(b.machine()->counts().errors),
                static_cast<unsigned long long>(b.machine()->counts().refused), static_cast<unsigned long long>(hash));
    {
        const driving::Driving::KerbMoveCounts& k = d.kerbMoveCounts();
        std::printf("    [crowd] kerb spaces: pulled out of %llu, pulled into %llu, trips that ended in the lane instead %llu; "
                    "handed between space and driving, the farthest moved %.3f m (put in a space, not driven in: %u)\n",
                    static_cast<unsigned long long>(k.pulledOut), static_cast<unsigned long long>(k.pulledIn),
                    static_cast<unsigned long long>(k.stoppedInLane), static_cast<double>(b.counts().worstKerbHandOver), b.counts().putInSpace);
    }
    if (!end.loops.empty()) std::printf("%s", d.describeWaiting(end).c_str());

    // NO RING OF WAITS AT ANY MINUTE: a ring is cars each waiting on the next, which nothing can end (the wait-for graph,
    // driving::WaitReport::loops)
    CHECK(ringsSeen == 0);
    // PEOPLE WAITING AT A CROSSWALK STAND AT ITS KERB END (ADR-0452), never in the carriageway: every sample of a walker
    // standing at a kerb, within a hand of the model's waiting place
    std::printf("    [crowd] walkers standing at a crosswalk's kerb: %llu samples, the farthest %.2f m from its kerb end (agent %d)\n",
                static_cast<unsigned long long>(kerbWaitSamples), farthestFromKerb, farthestFromKerbWho);
    if (farthestFromKerb > kAtTheKerbEnd && farthestFromKerbWho >= 0) {
        const citysim::Agent& a = sim.agents()[static_cast<std::size_t>(farthestFromKerbWho)];
        const citysim::CitySim::CrossingAhead x = sim.nextCrossing(a);
        std::printf("    [crowd]   then: %s\n", farthestFromKerbWhy.c_str());
        std::printf("    [crowd]   it: %s | crossing ahead: found %d crosswalk %u from end %d, to hold %.2f, to far %.2f; at %d on %d; hold why %d\n",
                    sim.describeAgent(farthestFromKerbWho).substr(0, 200).c_str(), x.found ? 1 : 0, x.crosswalk, x.fromEnd, x.toHold,
                    x.toFar, a.crosswalkAt, a.onCrosswalk ? 1 : 0, sim.holdWhyOf(farthestFromKerbWho));
    }
    // KNOWN (ADR-0452): a walker whose route line passes a crosswalk far from its kerb end (at a junction spread over
    // several points) is not stepped across to it and waits on its route, at the arriving arm's box edge at most; those
    // samples are counted, and held to a small share
    std::printf("    [crowd]   of them %llu (%.2f%%) not at a kerb end\n", static_cast<unsigned long long>(kerbWaitAway),
                kerbWaitSamples ? 100.0 * static_cast<double>(kerbWaitAway) / static_cast<double>(kerbWaitSamples) : 0.0);
    CHECK(kerbWaitSamples > 0u);
    CHECK(kerbWaitAway * kAwayShareOneIn < kerbWaitSamples);

    // WHAT WAS MADE AND WHAT HAPPENED, pinned from the run of 2026-11-17, sitting 2b (looked at: 350 residents in 323
    // households and 226 cars; the way to work builds from minute 3, 59-74 cars on the road over minutes 6-11, the most
    // 74 at minute 9, and the way home from about minute 20, 41-46 over minutes 25-30; r326 found no way to work_51 and
    // went to work_75 instead; 6 plans found no way, r9's 5 of them to home_7, which has no kind to swap: the pavements
    // end 26.6 m short of it, issue #300). Another stretch of the day (RT_CROWD_MINUTES) holds only the rings.
    // (At ADR-0446, 200 residents: households 154, cars 98, with one 126, markers 242, the most 41 at minute 5, trips
    // 61, parked 56, walks blocked 7, arrivals 234, no way 7, hash 0x2fe6b3a18cc9239f.)
    // RE-PINNED (ADR-0450, 2026-11-17) when kerb lanes came to be driven clear of the parking band: cars 226 -> 231
    // (with one 244 -> 249), the most on the road 74 at minute 9 -> 79 at minute 10, busy minutes 7 -> 10, trips 252 ->
    // 269, parked 208 -> 225, walks blocked 7 -> 13, another place 1 -> 3, arrivals 452 -> 451, no plan 6 -> 11, hash
    // 0x51e27f907adb580e -> 0x1d2715902b8559a2. Every car's line moved, so every car's day did; rings of waits still 0.
    // AND AGAIN (ADR-0451) when drivers came to stop for people on foot in their way: the most on the road 79 -> 83,
    // trips 269 -> 265, parked 225 -> 216, walks blocked 13 -> 17, arrivals 451 -> 444, no plan 11 -> 15, hash
    // 0x1d2715902b8559a2 -> 0x0c538d7f7a9fb3fa; rings still 0.
    // AND AGAIN (ADR-0452) when crosswalks came into the road model and people came to wait at their kerb ends and cross
    // on WALK or in a gap, and drivers to give way to them there: the most on the road 83 -> 91 (still at minute 10),
    // busy minutes 10 -> 13, trips 265 -> 260, parked 216 -> 204, walks blocked 17 -> 1, another place 3 -> 1, arrivals
    // 444 -> 415, no plan 15 -> 1, hash 0x0c538d7f7a9fb3fa -> 0xfd98526fa3b9d920; rings still 0.
    // AND AGAIN (ADR-0456) when the old sim's signal clock went: a walker at a junction the model's crosswalks do not rule
    // no longer waits for that clock's WALK and judges the gap instead (the runs part at minute 3). Trips 260 -> 261,
    // parked 204 -> 205, arrivals 415 -> 422, hash 0xfd98526fa3b9d920 -> 0xf0003605a4cb6ba2. (The one pole obstacle the
    // old clock's links stood where no pole is, link 561, went too; put back, the hash is the same.)
    // AND AGAIN at the S7 re-bake (ADR-0478; the roads of ADR-0470..0473): kerb spaces moved with the stop lines set back
    // at sharp angles (the lab's 1098 -> 1080), so cars 231 -> 229 (with one 249 -> 246); the walks join up (no place in
    // a cut-off part, 4 -> 0, level_walk_join), so more people walk: trips 261 -> 233, parked 205 -> 179, the most on the
    // road 91 at minute 10 -> 88 at minute 11, busy minutes 13 -> 12, another place 1 -> 0, no plan 1 -> 0, arrivals 422
    // -> 426, hash 0xf0003605a4cb6ba2 -> 0xb67879f5b6ec7210; rings still 0. Walkers waiting away from a kerb end 75.45%
    // -> 62.68% (still failing, above). Not pinned, looked at: the longest stand 341 s (car 123) -> 563 s (car 159,
    // junction 93, from minute 15).
    // AND AGAIN at the switch to the one router (#306 piece 6 W3, ADR-0480): people walk the road model's walks by
    // findWalkRoute, from and to their places' frontage, and wait at a crosswalk's kerb end where their route crosses, so
    // every walker's day and every driver's stops for them moved. The most on the road 88 at minute 11 -> 80 at minute
    // 10, busy minutes 12 -> 8, trips 233 -> 263, parked 179 -> 217, walks blocked 1 -> 0, arrivals 426 -> 455, hash
    // 0xb67879f5b6ec7210 -> 0xcb6bf4524e6ee584; rings, no plan, another place, errors, refused still 0. Walkers waiting
    // away from a kerb end 62.68% -> 4.38%, so the under-5% check above passes for the first time. Not pinned, looked
    // at: the longest stand 563 s -> 166 s (car 169).
    // AND AGAIN when the crowd maker was made to ask what the brain and the sim ask (ADR-0480, W3b): a place is put
    // only where its stand joins the largest part of the walks (CitySim::walkJoin; it asked the street map, which
    // nobody walks any more), and a home only where its household's cars have kerb spaces free in the brain's own list
    // (kerbSpacesOffered, from the marker's stated frontage), which it then holds. Places drawn again draw the seeded
    // stream on, so the households' car counts moved: cars 229 -> 209 (with one 246 -> 222). The most on the road 80 ->
    // 57, both at minute 10; busy minutes 8 -> 3; trips 263 -> 201; parked 217 -> 164; arrivals 455 -> 487; hash
    // 0xcb6bf4524e6ee584 -> 0xbacfb8b215f04aa1 (with walkers on a crossing never held behind a queue past it,
    // computeGaps; without that, the same counts and 0xe7299b584c61b745). Rings, no plan, another place, blocked,
    // errors, refused still 0. Away from a kerb end 4.38% -> 3.24%. Not pinned, looked at: the longest stand 166 s ->
    // 193 s (car 176).
    // AND AGAIN when the street map went (#306, the sweep): the sim's own walkers -- the city's people besides the
    // brain's residents -- rest at and go to walk points on the road model's walks (city_walk_points.h), not
    // street-map nodes, so their days, and the drivers' stops for them, moved. The most on the road 57 at minute 10
    // -> 87 at minute 9; busy minutes 3 -> 16; trips 201 -> 313; parked 164 -> 238; arrivals 487 -> 470; hash
    // 0xbacfb8b215f04aa1 -> 0xf0e62a8d7fffa45b. Away from a kerb end 3.24% -> 1.93%. STILL FAILING, on the known-bugs
    // list: ONE RING OF WAITS (a deadlock) from about minute 30 (the rings checks above), and the longest stand
    // 193 s -> 1247.6 s (car 109).
    // And its second half (the street furniture planned on the road model: one signal pole per arriving arm, a
    // sidewalk's width back from its mouth; lamps and kerb life along each kerb, so the walkers' obstacles stand
    // elsewhere): the most on the road 87 at minute 9 -> 89 at minute 10; parked 238 -> 234; arrivals 470 -> 472; hash
    // 0xf0e62a8d7fffa45b -> 0xec0a6a6e55e90ddc; away from a kerb end 1.93% -> 1.95%. The ring of waits and the longest
    // stand (1247.6 s, car 109) are the same.
    // RE-PINNED (ADR-0480, the sweep's bugs, 1: every crosswalk's walk signal walks in some phase): trips 313 -> 315,
    // parked 234 -> 237, arrivals 472 -> 476, hash 0xec0a6a6e55e90ddc -> 0xd88b08b351744711; away from a kerb end 1.95% ->
    // 2.06%; the most on the road (89 at minute 10) and busy minutes (16) the same. The ring of waits and the longest
    // stand (1247.6 s, car 109) are the same (the next bug on the list).
    // RE-PINNED (ADR-0480, the sweep's bugs, 3): the ring is gone. Car 102 turning far from the south waited inside
    // junction 7 to give way, further on, to car 136 going straight on, which stood behind it on the stretch the two
    // connectors share as they leave one lane; car 109 then waited for car 102. A car following another along a split's
    // stretch is now an arrow the resolver never turns round. Rings 1 -> 0 (from minute 11); the longest stand 1247.6 s
    // (car 109) -> 685.6 s (car 68: junction 7's north arm, losing its greens to people crossing, a give-way and no room
    // beyond -- no ring); trips 315 -> 321; parked 237 -> 257; arrivals 476 -> 490; busy minutes 16 -> 14 (the most on
    // the road the same, 89 at minute 10); away from a kerb end 2.06% -> 2.31%; hash 0xd88b08b351744711 ->
    // 0x108c0a6b2babd46b.
    // RE-PINNED (ADR-0480, the sweep's bugs, 4: a drive priced by the driving's router, kerb lane to kerb lane, at the
    // limit each road is driven at): the most on the road 89 -> 90 (still minute 10); trips 321 -> 322; parked 257 ->
    // 261; arrivals the same, 490; away from a kerb end 2.31% -> 2.16%; hash 0x108c0a6b2babd46b -> 0xa7a50cd8f2bb2c29.
    // 463 car legs priced, 365 chosen, none on the freeway: the day's trips never pay for its 420 m ramps (the drive
    // across town between them, checked above, does).
    // RE-PINNED (sitting 5: pulling out of and into kerb spaces on the driving's own curve, not a drawn 2.5 s slide): a car
    // is placed standing in its space and sets off at once, driving out on the lane change's curve; a trip ends with the
    // car driven into its space and left where it came to rest. Trips 322 -> 326, arrivals 490 -> 494 (no 2.5 s slide
    // before each trip); parked the same, 261; the most on the road (90 at minute 10) and busy minutes (14) the same;
    // rings still 0; away from a kerb end 2.16% -> 2.30%; hash 0xa7a50cd8f2bb2c29 -> 0xe8da33de4e20c514. Pinned now too:
    // every trip ended in its space (none in the lane) and no car moved more than a centimetre between its space and the
    // driving (4 mm). Not pinned, looked at: the longest stand 685.6 s -> 690.4 s (the same car 68, junction 7).
    // RE-PINNED (sitting 5: lane changes on by default): homes are placed where a class's routes reach, which lane
    // changes widen, so cars 209 -> 215 (with one 222 -> 233); trips 326 -> 337, parked 261 -> 272, arrivals 494 -> 473
    // (more drive, fewer walk), the most on the road 90 at minute 10 -> 98 at minute 8, busy minutes 14; rings 0; every
    // trip still ends in its space; hash 0xe8da33de4e20c514 -> 0x76038c965dbef7af. Longest stand 690.4 -> 457.3 s.
    // RE-PINNED (sitting 6: junctions by judgement, no ring order, no room grants): parked 272 -> 271, the rest the same;
    // rings 0; hash 0x76038c965dbef7af -> 0xf7e7889934759abd.
    if (minutes == kCrowdMinutes) {
        CHECK(b.residentCount() == 350);
        CHECK(households == 323);
        CHECK(b.cars().size() == 215u);
        CHECK(withCar == 233);
        CHECK(b.markers().size() == 451u);
        CHECK(mostOnRoad == 98);
        CHECK(mostOnRoadAt == 8);
        CHECK(busyMinutes == 14);
        CHECK(b.counts().tripsBegun == 337u);
        CHECK(b.counts().parkedReports == 271u);
        CHECK(b.counts().walksBlocked == 0u);
        CHECK(b.counts().otherPlaces == 0u);
        CHECK(b.machine()->counts().arrivals == 473u);
        CHECK(b.machine()->counts().noPlans == 0u);
        CHECK(b.machine()->counts().errors == 0u);
        CHECK(b.machine()->counts().refused == 0u);
        CHECK(hash == 0xf7e7889934759abdull);
        CHECK(d.kerbMoveCounts().pulledIn == 271u && d.kerbMoveCounts().stoppedInLane == 0u);
        CHECK(b.counts().putInSpace == 0u && b.counts().worstKerbHandOver < 0.01);
    }
    // RT_CROWD_LOG=1: why the first twenty planned as they did, everybody with no way, and the car that has stood
    // longest, in full
    if (std::getenv("RT_CROWD_LOG")) {
        for (int who = 0; who < std::min(std::max(20, std::atoi(std::getenv("RT_CROWD_LOG"))), b.residentCount()); ++who)
            std::printf("    [crowd] %s: %s | %s\n", b.residentName(who).c_str(), b.planText(who).c_str(), b.planReason(who).c_str());
        for (int who = 0; who < b.residentCount(); ++who)
            if (b.stateName(who) == "no_way") {
                const Vec2 at = sim.agents()[static_cast<std::size_t>(b.residentAgent(who))].pos;
                const engine::RoadFrontage f = d.model()->frontageAt(at);
                engine::WalkPoint joined;
                bool cutOff = false;
                const bool joins = sim.walkJoin(at, joined, &cutOff);
                std::printf("    [crowd] where it stands joins %s%s\n",
                            joins ? ("walk " + std::to_string(joined.walk.index)).c_str() : "no walk",
                            joins && cutOff ? " (cut off from the largest part)" : "");
                std::printf("    [crowd] no way: %s | street %s, class %s\n      trace: %s\n", b.describe(who).c_str(),
                            f.street.valid() ? d.model()->streets[f.street.index].id.c_str() : "?",
                            f.section ? f.section->roadClass.c_str() : "?", b.traceText(who).c_str());
            }
        if (d.car(driving::CarId(longestStandCar))) std::printf("    [crowd] stood longest:\n%s", d.describeCar(driving::CarId(longestStandCar)).c_str());
        // ...the crosswalk it gives way at (ADR-0452), and who is on it or waiting at it
        const uint32_t cwAt = d.crosswalkGivingWayAt(driving::CarId(longestStandCar));
        if (const engine::ModelCrosswalk* cw = d.model()->crosswalk(cwAt)) {
            std::printf("    [crowd] it gives way at crosswalk %u (junction %s, arm %u, %s, walk signal %s), ends (%.1f %.1f) (%.1f %.1f)\n",
                        cwAt, d.model()->junctions[cw->junction.index].id.c_str(), cw->arm, engine::crosswalkControlWord(cw->control),
                        driving::walkWord(driving::walkAtCrosswalk(d.rightOfWay(), cwAt, d.clockSeconds())), cw->ends[0].x,
                        cw->ends[0].y, cw->ends[1].x, cw->ends[1].y);
            for (std::size_t i = 0; i < sim.agents().size(); ++i)
                if (sim.agents()[i].crosswalkAt == static_cast<int>(cwAt)) {
                    std::string why;
                    sim.gapToCross(sim.agents()[i], cwAt, sim.agents()[i].crossWaited, false, &why);
                    std::printf("    [crowd]   %s it: agent %zu (waited %.0f s; held by: %s): %s\n", sim.agents()[i].onCrosswalk ? "on" : "at",
                                i, sim.agents()[i].crossWaited, why.empty() ? "nothing" : why.c_str(),
                                sim.describeAgent(static_cast<int>(i)).substr(0, 120).c_str());
                }
        }
        // ...and what is in its way (ADR-0451/0452), as the driving last saw it, and who that is
        if (const driving::ThingAhead* t = d.thingAhead(driving::CarId(longestStandCar))) {
            std::printf("    [crowd] in its way: %d, thing %u, %.2f m ahead, %.2f m to the side, moving %.2f m/s, on path %u\n",
                        t->found ? 1 : 0, t->id, t->distance, t->side, t->moves, t->path.index);
            for (const driving::RoadThing& th : d.thingsOnRoad())
                if (th.id == t->id) std::printf("    [crowd]   it is at %.1f %.1f\n", th.at.x, th.at.y);
            for (std::size_t i = 0; i < sim.agents().size(); ++i)
                if (sim.agents()[i].uid == t->id) {
                    const citysim::Agent& a = sim.agents()[i];
                    std::printf("    [crowd]   agent %zu: %s\n", i, sim.describeAgent(static_cast<int>(i)).c_str());
                    const citysim::CitySim::CrossingAhead x = sim.nextCrossing(a);
                    std::printf("    [crowd]   its next crossing: found %d crosswalk %u from end %d, to hold %.2f m, to far %.2f m; "
                                "hold why %d\n", x.found ? 1 : 0, x.crosswalk, x.fromEnd, x.toHold, x.toFar,
                                sim.holdWhyOf(static_cast<int>(i)));
                    for (std::size_t k = 0; k < a.route.steps.size(); ++k) {
                        const engine::roads::WalkStep& s = a.route.steps[k];
                        std::printf("    [crowd]     step %zu: walk %u (%s), %.1f to %.1f m%s\n", k, s.walk.index,
                                    sim.roads()->walkKind(s.walk).c_str(), s.from, s.to, s.forward ? "" : ", walked back");
                    }
                }
        }
        for (const driving::Car& c : d.cars())
            if (!c.driven) {
                std::printf("    [crowd] on the road and not driven: %s\n", b.describeCar(static_cast<int>(c.id.index)).c_str());
                for (const int who : b.membersOfCar(static_cast<int>(c.id.index)))
                    std::printf("      %s: %s | %s\n", b.residentName(who).c_str(), b.traceText(who).c_str(), b.planReason(who).c_str());
            }
        std::printf("    [crowd] waiting?: %s\n", d.describeWaiting(end).c_str());
    }

    // NO RING OF WAITS STANDS AT THE END: a ring is cars each waiting on the next, which nothing ends
    CHECK(end.loops.empty());
    std::string why;
    CHECK(b.holdsSound(&why));
    if (!why.empty()) std::printf("    [crowd] holds: %s\n", why.c_str());
}
