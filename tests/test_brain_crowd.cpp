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

#include "../src/apps/citysim/city_brain.h"
#include "../src/apps/citysim/city_car.h"
#include "../src/apps/citysim/city_render.h"
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

struct LoadedCrowd {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    std::unique_ptr<RendererMeshUploader> uploader;
    std::unique_ptr<AssetManager> assets;
    World world;
    RenderView view;
    citysim::CityRenderSystem city;
    bool ok = false;
    LoadedCrowd() {
        uploader = std::make_unique<RendererMeshUploader>(*renderer);
        assets = std::make_unique<AssetManager>(*uploader);
        if (!LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/brain_crowd.json", world, *renderer, view, *assets, false))
            return;
        ok = city.build(world, assets.get(), nullptr);
    }
};

// One number over the run's end state: the driving's own state hash, then every resident's brain record byte for byte
// and their place to the centimetre, then every household car's place to the centimetre.
uint64_t crowdHash(const citysim::CityBrain& b, const citysim::CitySim& sim, World& world) {
    uint64_t h = sim.drivingV2()->stateHash();
    const auto mix = [&](uint32_t v) { h = h * 1099511628211ull ^ hashPair(static_cast<uint32_t>(h >> 32), v); };
    for (int who = 0; who < b.residentCount(); ++who) {
        const citysim::brain::AgentBrain& a = b.machine()->agent(static_cast<citysim::brain::AgentId>(who));
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&a);
        for (std::size_t i = 0; i < sizeof(a); i += 4) {
            uint32_t w = 0;
            for (std::size_t k = 0; k < 4 && i + k < sizeof(a); ++k) w |= static_cast<uint32_t>(bytes[i + k]) << (8 * k);
            mix(w);
        }
        const Vec2 p = sim.agents()[static_cast<std::size_t>(b.residentAgent(who))].pos;
        mix(static_cast<uint32_t>(static_cast<int32_t>(std::lround(p.x * 100))));
        mix(static_cast<uint32_t>(static_cast<int32_t>(std::lround(p.y * 100))));
    }
    for (const int car : b.cars()) {
        const Transform* t = world.get<Transform>(b.carEntity(car));
        if (!t) continue;
        mix(static_cast<uint32_t>(static_cast<int32_t>(std::lround(t->position.x * 100))));
        mix(static_cast<uint32_t>(static_cast<int32_t>(std::lround(t->position.z * 100))));
    }
    return h;
}

}  // namespace

TEST_CASE(level_brain_crowd_the_residents_make_their_day) {
    LoadedCrowd lc;
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

    // THE DAY, minute by minute
    const int ticksPerMinute = static_cast<int>(60.0 / kStep + 0.5);
    int mostOnRoad = 0, mostOnRoadAt = 0, ringsSeen = 0, busyMinutes = 0;
    double stepMsSum = 0, stepWorst = 0;
    std::map<uint32_t, int> standingSince;   // car id -> the tick it last stood still from
    int longestStandTicks = 0;
    uint32_t longestStandCar = 0;
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
    if (!end.loops.empty()) std::printf("%s", d.describeWaiting(end).c_str());

    // NO RING OF WAITS AT ANY MINUTE: a ring is cars each waiting on the next, which nothing can end (the wait-for graph,
    // driving::WaitReport::loops)
    CHECK(ringsSeen == 0);

    // WHAT WAS MADE AND WHAT HAPPENED, pinned from the run of 2026-11-17, sitting 2b (looked at: 350 residents in 323
    // households and 226 cars; the way to work builds from minute 3, 59-74 cars on the road over minutes 6-11, the most
    // 74 at minute 9, and the way home from about minute 20, 41-46 over minutes 25-30; r326 found no way to work_51 and
    // went to work_75 instead; 6 plans found no way, r9's 5 of them to home_7, which has no kind to swap: the pavements
    // end 26.6 m short of it, issue #300). Another stretch of the day (RT_CROWD_MINUTES) holds only the rings.
    // (At ADR-0446, 200 residents: households 154, cars 98, with one 126, markers 242, the most 41 at minute 5, trips
    // 61, parked 56, walks blocked 7, arrivals 234, no way 7, hash 0x2fe6b3a18cc9239f.)
    if (minutes == kCrowdMinutes) {
        CHECK(b.residentCount() == 350);
        CHECK(households == 323);
        CHECK(b.cars().size() == 226u);
        CHECK(withCar == 244);
        CHECK(b.markers().size() == 451u);
        CHECK(mostOnRoad == 74);
        CHECK(mostOnRoadAt == 9);
        CHECK(busyMinutes == 7);
        CHECK(b.counts().tripsBegun == 252u);
        CHECK(b.counts().parkedReports == 208u);
        CHECK(b.counts().walksBlocked == 7u);
        CHECK(b.counts().otherPlaces == 1u);
        CHECK(b.machine()->counts().arrivals == 452u);
        CHECK(b.machine()->counts().noPlans == 6u);
        CHECK(b.machine()->counts().errors == 0u);
        CHECK(b.machine()->counts().refused == 0u);
        CHECK(hash == 0x51e27f907adb580eull);
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
                const int node = sim.graph()->nearestNode(at);
                int walkLinks = 0, links = 0;
                for (const engine::NavLink& L : sim.graph()->links)
                    if (L.from == node || L.to == node) {
                        ++links;
                        walkLinks += L.walkable ? 1 : 0;
                    }
                std::printf("    [crowd] nearest node %d at (%.1f, %.1f): %d links, %d walkable\n", node,
                            sim.graph()->nodes[static_cast<std::size_t>(node)].x, sim.graph()->nodes[static_cast<std::size_t>(node)].y, links,
                            walkLinks);
                std::printf("    [crowd] no way: %s | street %s, class %s\n      trace: %s\n", b.describe(who).c_str(),
                            f.street.valid() ? d.model()->streets[f.street.index].id.c_str() : "?",
                            f.section ? f.section->roadClass.c_str() : "?", b.traceText(who).c_str());
            }
        if (d.car(driving::CarId(longestStandCar))) std::printf("    [crowd] stood longest:\n%s", d.describeCar(driving::CarId(longestStandCar)).c_str());
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
