// ONE RESIDENT WITH THE AGENT BRAIN MAKES A WHOLE TRIP (docs/agent-brain-design.md section 9; ADR-0441).
//
// Loads assets/levels/brain_lab.json through the real LevelLoader and CityRenderSystem (a NullRenderer underneath, as
// every level test): the traffic lab's roads and markers, one resident driven by the brain library, their car in a
// kerb space by the marker `home`, and no other traffic. The city is stepped the way the viewer steps it, 60 times a
// second. Run one case at a time:
//
//   RT_TEST_FILTER=level_brain_lab_one_resident_makes_the_trip ./build-viewer/level_tests
//
// level_brain_lab_one_resident_makes_the_trip: told to be at `work`, about 760 m away, the resident plans walk, car,
// walk; walks to the car, gets in, pulls out, drives there on the new driving, slides into the kerb space held for
// them, gets out and walks to the marker. Held: the plan, the states in order, that the car left its space and the
// driving reported "parked", where the car and the person end, that the car never went faster than the limit posted
// where it was, that it entered no junction on a red light, and that it came to rest at every stop line it met.
//
// level_brain_lab_a_board_makes_the_short_hop_one_leg: with a skateboard, told to be at `shop` (about 370 m): one leg
// on the board, and the car never moves.
//
// level_brain_lab_a_goto_while_driving_is_taken_up_on_arriving: told to be back at `home` while driving to `work`: they
// finish the trip to `work` (the drive is not redirected: Driving::changeDestination is not used), and from the marker
// plan again, back to the car and home.
//
// level_brain_lab_a_taken_space_is_answered_with_the_next: another car takes the space held for them while they drive;
// beside it they are told the next nearest free space and drive on to that one.
//
// RT_BRAIN_LAB_LOG=1 prints `brain?` at every change of state and every 30 s.

#include "test_framework.h"

#include "../src/apps/citysim/city_brain.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/apps/driving/signal_programme.h"
#include "../src/apps/driving/traffic_state.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/city/roads/road_model.h"
#include "../src/engine/world.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

constexpr Real kStep = 1.0 / 60.0;   // the viewer's fixed step, and the sim's tick on the lab
// THE LIMIT A DRIVER KEEPS TO is the posted one times their own habit, which the lab's rule set deals between 0.92 and
// 1.08 (assets/scripts/rules/lab.lua `speed_habit`): the fastest any driver means to go is 1.08 times the posted limit.
constexpr Real kHighestSpeedHabit = 1.08;
// A car is over the limit when it is faster than that, where its front is, by more than this: the driving's own
// measure of it (measure.h kOverLimitToleranceMps, about 1 mph).
constexpr Real kOverLimitTolerance = 0.5;
// A parked car is where its space says, and a person where their marker says, to this: both are put there exactly.
constexpr Real kParkedWithin = 0.01;
// A walk ends within this of where it was going (city_brain.h kWalkArrivedWithin, 0.3 m).
constexpr Real kStandsWithin = citysim::kWalkArrivedWithin;
// A car has stopped at a stop line when, within this far of the end of the lane it arrived by, it was slower than the
// driving's own "standing" speed (traffic_state.h kStandingBelow).
constexpr Real kStopLineWithin = 15.0;
// The longest a case runs, in simulated seconds: three times the longest trip here.
constexpr int kLongestRun = 30 * 60;

struct LoadedCity {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    std::unique_ptr<RendererMeshUploader> uploader;
    std::unique_ptr<AssetManager> assets;
    World world;
    RenderView view;
    citysim::CityRenderSystem city;
    bool ok = false;
    explicit LoadedCity(const std::string& level) {
        uploader = std::make_unique<RendererMeshUploader>(*renderer);
        assets = std::make_unique<AssetManager>(*uploader);
        if (!LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/" + level, world, *renderer, view, *assets, false))
            return;
        ok = city.build(world, assets.get(), nullptr);
    }
};

// ONE JUNCTION THE CAR WENT INTO: what its light showed as the tick began, what the lane it arrived by is regulated
// with, and how slow it got within kStopLineWithin of that lane's end.
struct JunctionEntry {
    double at = 0;
    uint32_t connector = 0;
    std::string control;
    driving::Lamp lamp = driving::Lamp::Dark;
    std::string regulations;   // the kinds met along the lane it arrived by, each followed by a space
    Real slowestAtLine = 1e9;
    Real entrySpeed = 0;
};

// WHAT THE CAR DID, tick by tick.
struct CarWatch {
    uint64_t ticksOnRoad = 0, ticksDriven = 0;
    Real worstOverLimit = -1e9;       // speed less the posted limit, at its worst (m/s)
    double worstAt = 0;               // ...when, on which path, how far along it, and the limit there
    uint32_t worstPath = engine::kNoIndex;
    Real worstAlong = 0, worstLimit = 0;
    Real fastest = 0;
    Real farthestFromStart = 0;       // metres from where it stood at the start
    std::vector<JunctionEntry> entries;
    uint32_t lastPath = engine::kNoIndex;
    Real slowestOnLane = 1e9;         // within kStopLineWithin of the end of the lane it is on
    std::string lastStates;

    void look(const citysim::CitySim& sim, const citysim::CityBrain& b, Vec2 start) {
        const driving::Driving& d = *sim.drivingV2();
        const citysim::SimVehicle& v = sim.vehicles()[static_cast<std::size_t>(b.residentVehicle())];
        farthestFromStart = std::max(farthestFromStart, (v.pos - start).length());
        const driving::Car* c = d.car(driving::CarId(static_cast<uint32_t>(b.residentVehicle())));
        if (!c) {
            lastPath = engine::kNoIndex;
            return;
        }
        ++ticksOnRoad;
        if (c->driven) ++ticksDriven;
        const engine::RoadModel& model = *d.model();
        const engine::LaneGraph& roads = model.laneLevel;
        const Real limit = model.postedLimit(c->path);
        fastest = std::max(fastest, c->speed);
        if (limit > 0 && c->speed - limit * kHighestSpeedHabit > worstOverLimit) {
            worstOverLimit = c->speed - limit * kHighestSpeedHabit;
            worstAt = d.clockSeconds();
            worstPath = c->path.index;
            worstAlong = c->along;
            worstLimit = limit;
        }
        const engine::Path& path = roads.paths[c->path.index];
        if (c->path.index != lastPath) {
            if (path.kind == engine::PathKind::Connector && lastPath != engine::kNoIndex) {
                JunctionEntry e;
                e.at = d.clockSeconds();
                e.connector = c->path.index;
                e.control = model.controlAt(path.junction);
                // (what the driver decided by: the lamp as the tick began)
                e.lamp = driving::lampAt(roads, d.rightOfWay(), c->path, d.clockSeconds() - kStep);
                for (const engine::PathRegulation& r : model.regulationsAlong(engine::PathId(lastPath))) e.regulations += r.kind + " ";
                e.slowestAtLine = slowestOnLane;
                e.entrySpeed = c->speed;
                entries.push_back(e);
            }
            lastPath = c->path.index;
            slowestOnLane = 1e9;
        }
        if (path.kind == engine::PathKind::Lane && path.length - c->along <= kStopLineWithin)
            slowestOnLane = std::min(slowestOnLane, c->speed);
    }
};

// The states in the order they were entered, as "indoors walking ...".
std::string stateNames(const citysim::CityBrain& b) {
    std::string s;
    for (const citysim::brain::Transition& t : b.trace())
        s += (s.empty() ? "" : " ") + b.machine()->table().states[t.state].name;
    return s;
}
// How long the n-th state entered lasted, in milliseconds (the last: until now).
uint32_t stateLasted(const citysim::CityBrain& b, std::size_t n) {
    const std::vector<citysim::brain::Transition>& t = b.trace();
    if (n >= t.size()) return 0;
    return (n + 1 < t.size() ? t[n + 1].at : b.now()) - t[n].at;
}

struct Run {
    LoadedCity lc;
    citysim::CityBrain* b = nullptr;
    CarWatch watch;
    Vec2 carStart;
    int ticks = 0;
    std::string logged;
    bool log = std::getenv("RT_BRAIN_LAB_LOG") != nullptr;

    Run() : lc("brain_lab.json") {
        CHECK(lc.ok);
        if (!lc.ok) return;
        b = lc.city.simMutable().brain();
        CHECK(b != nullptr && b->hasResident());
        if (b && !b->hasResident()) b = nullptr;
        if (b) carStart = lc.city.sim().vehicles()[static_cast<std::size_t>(b->residentVehicle())].pos;
    }
    const citysim::CitySim& sim() const { return lc.city.sim(); }
    const citysim::Agent& person() const { return sim().agents()[static_cast<std::size_t>(b->residentAgent())]; }
    const citysim::SimVehicle& car() const { return sim().vehicles()[static_cast<std::size_t>(b->residentVehicle())]; }
    void step() {
        lc.city.step(lc.world, kStep);
        ++ticks;
        watch.look(sim(), *b, carStart);
        if (log && (b->stateName() != logged || ticks % (60 * 30) == 0)) {
            logged = b->stateName();
            std::printf("    [brain-lab] t=%.1f brain? %s\n", ticks * kStep, b->describe().c_str());
        }
    }
    // Steps until `done` says so; false when kLongestRun passes first.
    template <typename Done>
    bool until(Done done) {
        for (int i = 0; i < kLongestRun * 60; ++i) {
            step();
            if (done()) return true;
        }
        return false;
    }
    // The resident is where they wanted to be, with nothing left to do.
    bool there(const std::string& marker) const {
        const citysim::brain::AgentBrain& a = b->machine()->agent(0);
        return !a.travelling() && static_cast<int>(a.intentTarget) == b->markerNamed(marker) && b->stateName() == "standing" &&
               b->trace().size() > 2;
    }
    void printTrip(const char* what) const {
        std::printf("    [brain-lab] %s: plan \"%s\"\n    [brain-lab] states: %s\n", what, b->planText().c_str(), b->traceText().c_str());
        const citysim::CityBrainCounts& c = b->counts();
        std::printf("    [brain-lab] got in %u, pulled out %u (waited %u ticks for room), trips begun %u, \"parked\" reports %u, slid in %u, "
                    "got out %u; walks %u begun, %u arrived, %u blocked; next spaces %u\n",
                    c.boarded, c.pulledOut, c.waitedForRoom, c.tripsBegun, c.parkedReports, c.slidIn, c.alighted, c.walksBegun,
                    c.walksArrived, c.walksBlocked, c.nextSpaces);
        std::printf("    [brain-lab] the car: %llu ticks on the road, fastest %.2f m/s, worst over the posted limit %.2f m/s (at "
                    "t=%.1f, %.1f m along path %u, limit %.2f), farthest from its first space %.1f m; %zu junctions entered:\n",
                    static_cast<unsigned long long>(watch.ticksOnRoad), watch.fastest, watch.worstOverLimit, watch.worstAt,
                    watch.worstAlong, watch.worstPath, watch.worstLimit, watch.farthestFromStart, watch.entries.size());
        static const char* const kLamps[] = {"dark", "green", "yellow", "red", "red-may-turn"};
        for (const JunctionEntry& e : watch.entries)
            std::printf("      t=%.1f connector %u, control %s, lamp %s, the lane's regulations: %s, slowest within %.0f m of the line "
                        "%.2f m/s, entered at %.2f m/s\n",
                        e.at, e.connector, e.control.c_str(), kLamps[static_cast<int>(e.lamp)],
                        e.regulations.empty() ? "none" : e.regulations.c_str(), kStopLineWithin, e.slowestAtLine, e.entrySpeed);
        std::printf("    [brain-lab] brain? %s\n", b->describe().c_str());
    }
};

// What the drive must have kept to: the limit, the lights, the stop lines.
void checkDrove(const CarWatch& w) {
    CHECK(w.worstOverLimit <= kOverLimitTolerance);
    for (const JunctionEntry& e : w.entries) {
        CHECK(e.lamp != driving::Lamp::Red);
        // a lane that ends at a stop line (the regulation kinds the road model names with "stop"): the car came to rest
        if (e.regulations.find("stop") != std::string::npos) CHECK(e.slowestAtLine < driving::kStandingBelow);
    }
}

}  // namespace

TEST_CASE(level_brain_lab_one_resident_makes_the_trip) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    const engine::RoadModel& model = *r.sim().drivingV2()->model();

    // AT HOME: standing at the marker, the car in the kerb space nearest it on foot
    CHECK(b.markers().size() == 8u);
    const int home = b.markerNamed("home"), work = b.markerNamed("work");
    CHECK(home == 0 && work == 1);
    if (home < 0 || work < 0) return;
    CHECK(approxEqual(r.person().pos.x, b.markers()[0].stand.x, kParkedWithin) &&
          approxEqual(r.person().pos.y, b.markers()[0].stand.y, kParkedWithin));
    CHECK(b.carSpaceName() == "s19 along 3");
    CHECK(b.stateName() == "indoors" && b.planText().empty());
    CHECK(r.sim().drivingV2()->carCount() == 0u);
    // THE SIM PLACING ITS POPULATION AGAIN (the viewer opens a level at its hour; a `daynight` jump) does not carry the
    // resident off: they are the brain's to move, and stand where it left them. (Seen in the viewer before this was
    // held: the resident stood 500 m from home, where the day of a role they do not keep had them.)
    r.lc.city.simMutable().seedFromSchedule(15.0);
    r.step();
    CHECK(approxEqual(r.person().pos.x, b.markers()[0].stand.x, kParkedWithin) &&
          approxEqual(r.person().pos.y, b.markers()[0].stand.y, kParkedWithin));
    CHECK(!r.person().indoors && !r.person().moving);

    std::string why;
    CHECK(b.goTo("work", &why));
    r.step();
    // THE PLAN: walk to the car, drive to the space nearest work, walk to the marker. That space is held from now.
    CHECK(b.planText() == "walk>space s19 along 3, car>space s182 against 3, walk>work");
    CHECK(model.kerbSpaceName(b.heldSpace()) == "s182 against 3");
    const bool arrived = r.until([&] { return r.there("work"); });
    r.printTrip("home to work");
    CHECK(arrived);
    if (!arrived) return;

    // THE STATES, in order; pulling out and parking last the way's own minutes (travel.lua: 2 and 3)
    CHECK(stateNames(b) == "indoors walking pulling_out driving parking walking standing");
    CHECK(stateLasted(b, 2) == 120000u && stateLasted(b, 4) == 180000u);
    const citysim::CityBrainCounts& c = b.counts();
    CHECK(c.boarded == 1u && c.alighted == 1u);
    // THE CAR LEFT ITS SPACE, was driven, and the driving reported "parked" once
    CHECK(c.pulledOut == 1u && c.tripsBegun == 1u && c.parkedReports == 1u && c.slidIn == 1u);
    CHECK(r.watch.ticksDriven > 60u * 60u);            // more than a minute at the wheel
    CHECK(r.watch.farthestFromStart > 400.0);
    CHECK(c.walksBegun == 2u && c.walksArrived == 2u && c.walksBlocked == 0u);
    CHECK(b.machine()->counts().plans == 1u && b.machine()->counts().errors == 0u && b.machine()->counts().refused == 0u);
    // IT KEPT TO THE ROAD'S RULES: the limit, the lights (two junctions with lights at least), every stop line
    checkDrove(r.watch);
    int lit = 0, stopLines = 0;
    for (const JunctionEntry& e : r.watch.entries) {
        lit += e.lamp != driving::Lamp::Dark ? 1 : 0;
        stopLines += e.regulations.find("stop") != std::string::npos ? 1 : 0;
    }
    CHECK(lit >= 2);
    CHECK(stopLines >= 1);
    // WHERE THE CAR ENDS: in the space that was held, exactly where a car in it stands and faces, off the driving's
    // road, and nobody holds a space any more. It is the nearest space to work on foot.
    CHECK(b.carSpaceName() == "s182 against 3");
    CHECK(!b.heldSpace().valid());
    engine::KerbSpacePose pose;
    CHECK(model.kerbSpacePose(b.carSpace(), pose));
    CHECK(approxEqual(r.car().pos.x, pose.at.x, kParkedWithin) && approxEqual(r.car().pos.y, pose.at.y, kParkedWithin));
    CHECK(approxEqual(r.car().heading.x, pose.facing.x, kParkedWithin) && approxEqual(r.car().heading.y, pose.facing.y, kParkedWithin));
    CHECK(r.car().driver < 0);
    CHECK(r.sim().drivingV2()->carCount() == 0u);
    const std::vector<engine::KerbSpaceNear> nearWork = model.kerbSpacesNear(b.markers()[1].frontage, citysim::kSpaceReach, 1);
    CHECK(nearWork.size() == 1u && model.kerbSpaceName(nearWork[0].space) == "s182 against 3");
    // WHERE THE PERSON ENDS: on foot, standing at the marker
    CHECK(r.person().mode == citysim::Agent::Mode::Pedestrian && r.person().vehicle < 0 && !r.person().moving);
    CHECK((r.person().pos - b.markers()[1].stand).length() <= kStandsWithin);
    CHECK(!r.sim().drivenByV2(b.residentAgent()));
}

TEST_CASE(level_brain_lab_a_board_makes_the_short_hop_one_leg) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    std::string why;
    // without a board the short hop is walked; with one it is one leg on the board
    CHECK(b.goTo("shop", &why));
    r.step();
    CHECK(b.planText() == "walk>shop");
    CHECK(b.setTrait("has_board", true, &why));
    CHECK(!b.setTrait("has_wings", true, &why));
    CHECK(b.goTo("shop", &why));   // (a new intent on foot plans again at once)
    r.step();
    CHECK(b.planText() == "skateboard>shop");
    const uint16_t planned = b.machine()->agent(0).legSeconds[0];
    bool onBoard = false;
    const bool arrived = r.until([&] {
        onBoard = onBoard || r.sim().rideOf(b.residentAgent()) == citysim::Ride::Skateboard;
        return r.there("shop");
    });
    r.printTrip("home to shop on a board");
    CHECK(arrived);
    if (!arrived) return;
    CHECK(stateNames(b) == "indoors walking on_small_wheels standing");
    CHECK(onBoard);
    // the leg took about what the plan said: within a fifth of it (the planner prices the footpaths' route at the
    // board's speed; the walk follows the pavements and takes the last steps off them)
    const double took = stateLasted(b, 2) / 1000.0;
    std::printf("    [brain-lab] the board leg: planned %u s, took %.1f s\n", planned, took);
    CHECK(took > planned * 0.8 && took < planned * 1.2);
    CHECK((r.person().pos - b.markers()[static_cast<std::size_t>(b.markerNamed("shop"))].stand).length() <= kStandsWithin);
    // NO CAR MOVED: nobody got in, nothing was ever on the driving's road, and the car stands where it stood
    const citysim::CityBrainCounts& c = b.counts();
    CHECK(c.boarded == 0u && c.pulledOut == 0u && c.tripsBegun == 0u);
    CHECK(r.watch.ticksOnRoad == 0u);
    CHECK(approxEqual(r.watch.farthestFromStart, 0.0, kParkedWithin));
    CHECK(b.carSpaceName() == "s19 along 3" && !b.heldSpace().valid());
}

TEST_CASE(level_brain_lab_a_goto_while_driving_is_taken_up_on_arriving) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    std::string why;
    CHECK(b.goTo("work", &why));
    CHECK(r.until([&] { return b.stateName() == "driving" && r.watch.ticksDriven > 30u * 60u; }));
    // TOLD TO BE BACK HOME WHILE DRIVING TO WORK: nothing changes at the wheel
    const std::size_t statesBefore = b.trace().size();
    CHECK(b.goTo("home", &why));
    for (int i = 0; i < 120; ++i) r.step();
    CHECK(b.stateName() == "driving" && b.trace().size() == statesBefore);
    CHECK(b.planText() == "walk>space s19 along 3, car>space s182 against 3, walk>work");
    CHECK(b.machine()->counts().plans == 1u);
    // ...they park at work, walk to its marker, and from there plan again: back to the car and home, to the space they
    // left (free again, and the nearest to home)
    const bool arrived = r.until([&] { return r.there("home"); });
    r.printTrip("home to work, told `goto home` at the wheel");
    CHECK(arrived);
    if (!arrived) return;
    CHECK(stateNames(b) == "indoors walking pulling_out driving parking walking walking pulling_out driving parking walking standing");
    CHECK(b.planText() == "walk>space s182 against 3, car>space s19 along 3, walk>home");
    CHECK(b.machine()->counts().plans == 2u && b.machine()->counts().arrivals == 1u);
    CHECK(b.counts().parkedReports == 2u && b.counts().boarded == 2u && b.counts().alighted == 2u);
    CHECK(b.carSpaceName() == "s19 along 3");
    CHECK((r.person().pos - b.markers()[0].stand).length() <= kStandsWithin);
    checkDrove(r.watch);
}

TEST_CASE(level_brain_lab_a_taken_space_is_answered_with_the_next) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    const engine::RoadModel& model = *r.sim().drivingV2()->model();
    std::string why;
    CHECK(b.goTo("work", &why));
    CHECK(r.until([&] { return b.stateName() == "driving"; }));
    const engine::KerbSpace first = b.heldSpace();
    CHECK(model.kerbSpaceName(first) == "s182 against 3");
    b.occupyForTest(first);   // another car takes it while they drive
    const bool arrived = r.until([&] { return r.there("work"); });
    r.printTrip("home to work, the held space taken on the way");
    CHECK(arrived);
    if (!arrived) return;
    // beside the taken space they are told the next nearest free one, drive on, and park there
    CHECK(stateNames(b) == "indoors walking pulling_out driving parking driving parking walking standing");
    CHECK(b.counts().nextSpaces == 1u && b.counts().parkedReports == 2u && b.counts().slidIn == 1u);
    CHECK(b.machine()->counts().plans == 1u);
    const std::vector<engine::KerbSpaceNear> nearWork = model.kerbSpacesNear(b.markers()[1].frontage, citysim::kSpaceReach, 2);
    CHECK(nearWork.size() == 2u);
    if (nearWork.size() == 2u) CHECK(b.carSpaceName() == model.kerbSpaceName(nearWork[1].space));
    CHECK(b.carSpaceName() != "s182 against 3" && !b.carSpaceName().empty());
    engine::KerbSpacePose pose;
    CHECK(model.kerbSpacePose(b.carSpace(), pose));
    CHECK(approxEqual(r.car().pos.x, pose.at.x, kParkedWithin) && approxEqual(r.car().pos.y, pose.at.y, kParkedWithin));
    CHECK((r.person().pos - b.markers()[1].stand).length() <= kStandsWithin);
    checkDrove(r.watch);
}
