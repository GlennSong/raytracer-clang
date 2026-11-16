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
// level_brain_lab_a_goto_while_driving_redirects_the_drive (ADR-0442): told to be at `shop` while driving to `work`: the
// same trip of the same car is given a new end (Driving::changeDestination), the plan becomes "drive to the space
// nearest the shop, walk", and the space held at work is let go for the new one. Also: the sim placing its population
// again while they drive moves neither the driver nor the car.
//
// level_traffic_lab_first_cars_stand_at_their_stop_lines: the 300-car lab, where the first car of each queue at a red
// light stands against the model's stop line.
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

// ONE TIME THE CAR STOOD AT THE END OF A LANE THAT ARRIVES AT A JUNCTION: where three things say its stop line is and
// where the car was. Every distance is metres ALONG THE LANE, measured from the model's stop line (the paint's bar):
// below zero is before the line (the side the car arrives from), above zero is past it, into the junction.
struct LineStop {
    double at = 0;                  // the driving's clock when it first stood
    uint32_t path = engine::kNoIndex;
    driving::Lamp lamp = driving::Lamp::Dark;   // of the connector its route takes next, when it first stood
    Real modelLine = 0;             // the model's stop line (ModelArm::stopLine), as a station of its road's spine
    Real laneEndStation = 0;        // the station the lane path ends at (Path::stationTo): the driving's line
    Real drivingLine = 0;           // the driving's line: the lane path's end (the junction plan's startOdometer)
    Real front = 0, rear = 0;       // the driving's car: its front bumper (Car::along) and its rear (a class length back)
    Real drawnFront = 0, drawnRear = 0;   // the body as drawn: the sim vehicle's middle, half its drawn length each way
    Real classLength = 0, drawnLength = 0;
};
// A car at a red light stands with its front AT OR BEFORE its stop line and no more than this far behind it, both as
// the driving has it and as it is drawn. (The driving brings a front to rest within its rule set's at_line_within of
// the line; 1.5 m is a third of a car, near enough that nobody would say it stopped short.)
constexpr Real kStandsBehindLineWithin = 1.5;
// "At" the line allows this much past it: a drawn body is laid on a chord of the lane, a few millimetres off its curve.
constexpr Real kAtTheLine = 0.02;

// Is `c` standing at the end of a lane that arrives at a junction? Then `s` is where the three say the line is and where
// the car is (its `at`, `path` and `lamp` are the caller's: they belong to the moment the stand began).
bool measureAtLine(const driving::Driving& d, const engine::RoadModel& model, const driving::Car& c,
                   const citysim::SimVehicle& v, LineStop& s) {
    const engine::LaneGraph& roads = model.laneLevel;
    const engine::Path& path = roads.paths[c.path.index];
    const engine::ModelJunction* junction = model.arrivesAt(c.path);
    if (!c.driven || !(c.speed < driving::kStandingBelow) || path.kind != engine::PathKind::Lane || !junction ||
        path.arm >= junction->arms.size() || path.length - c.along > kStopLineWithin)
        return false;
    s.modelLine = junction->arms[path.arm].stopLine;
    s.laneEndStation = path.stationTo;
    // the model's line as metres along this path (RoadModel::alongPathAt: the one placing of a station on a path)
    const Real lineAlong = model.alongPathAt(c.path, s.modelLine);
    s.drivingLine = path.length - lineAlong;
    s.classLength = d.vehicleClasses().at(c.vehicleClass).length;
    s.drawnLength = v.length;
    s.front = c.along - lineAlong;
    s.rear = s.front - s.classLength;
    // the drawn body, laid along the lane's direction at the line
    const engine::Vec2 lineAt = roads.pointAt(c.path, lineAlong), way = roads.directionAt(c.path, lineAlong);
    const Real middle = engine::dot(v.pos - lineAt, way);
    s.drawnFront = middle + v.length * Real(0.5);
    s.drawnRear = middle - v.length * Real(0.5);
    return true;
}
// The lamp of the connector `c`'s way takes next (dark: no trip, or no junction with lights ahead of it on its way).
driving::Lamp lampAhead(const driving::Driving& d, const driving::Car& c) {
    const engine::LaneGraph& roads = *d.roads();
    const driving::Trip* trip = d.trip(d.tripOf(c.id));
    if (!trip) return driving::Lamp::Dark;
    for (std::size_t k = trip->wayAt; k < trip->way.paths.size(); ++k) {
        const engine::PathId next = trip->way.paths[k];
        if (next.index < roads.paths.size() && roads.paths[next.index].kind == engine::PathKind::Connector)
            return driving::lampAt(roads, d.rightOfWay(), next, d.clockSeconds());
    }
    return driving::Lamp::Dark;
}
const char* lampName(driving::Lamp lamp) {
    static const char* const kLamps[] = {"dark", "green", "yellow", "red", "red-may-turn"};
    return kLamps[static_cast<int>(lamp)];
}

// WHAT THE CAR DID, tick by tick.
struct CarWatch {
    std::vector<LineStop> lineStops;
    bool standing = false;            // it stood at a line on the last tick (one LineStop a stand)
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
        lookAtLine(d, model, v, *c);
    }

    // AT REST NEAR THE END OF A LANE THAT ARRIVES AT A JUNCTION: one LineStop a stand, measured as the stand begins and
    // moved on while it lasts (a car that creeps forward while standing is measured where it got to).
    void lookAtLine(const driving::Driving& d, const engine::RoadModel& model, const citysim::SimVehicle& v, const driving::Car& c) {
        LineStop now;
        if (!measureAtLine(d, model, c, v, now)) {
            standing = false;
            return;
        }
        if (!standing) {
            now.at = d.clockSeconds();
            now.path = c.path.index;
            now.lamp = lampAhead(d, c);
            lineStops.push_back(now);
        }
        standing = true;
        LineStop& s = lineStops.back();
        now.at = s.at;
        now.path = s.path;
        now.lamp = s.lamp;
        s = now;
    }
    void printLineStops() const {
        std::printf("    [brain-lab] %zu stands at the end of a lane arriving at a junction (metres along the lane from the model's stop "
                    "line; below 0 is before it):\n",
                    lineStops.size());
        for (const LineStop& s : lineStops)
            std::printf("      t=%.1f path %u lamp %s | model line: station %.2f; lane ends at station %.2f; the driving's line %+.2f | "
                        "driving's car: front %+.2f rear %+.2f (class %.2f m) | drawn body: front %+.2f rear %+.2f (drawn %.2f m)\n",
                        s.at, s.path, lampName(s.lamp), s.modelLine, s.laneEndStation, s.drivingLine, s.front, s.rear,
                        s.classLength, s.drawnFront, s.drawnRear, s.drawnLength);
    }
};

// The states in the order they were entered, as "standing walking ...".
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
        watch.printLineStops();
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
    // WHERE IT STOOD AT EACH RED LIGHT: its front at or before the model's stop line (where the paint draws the bar) and
    // no more than kStandsBehindLineWithin behind it, as the driving has the car AND as the body is drawn; and the
    // driving's line (the end of the lane path) is the model's.
    for (const LineStop& s : w.lineStops) {
        if (s.lamp != driving::Lamp::Red) continue;
        CHECK(approxEqual(s.drivingLine, 0, kWrittenStation));
        CHECK(atOrBefore(s.front, 0, kAtTheLine) && atOrPast(s.front, -kStandsBehindLineWithin, kAtTheLine));
        CHECK(atOrBefore(s.drawnFront, 0, kAtTheLine) && atOrPast(s.drawnFront, -kStandsBehindLineWithin, kAtTheLine));
    }
}
int redLightStands(const CarWatch& w) {
    int n = 0;
    for (const LineStop& s : w.lineStops) n += s.lamp == driving::Lamp::Red ? 1 : 0;
    return n;
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
    // (a marker has no building: they are in their intent's arriving state, standing outside, and the screen says where)
    CHECK(b.stateName() == "standing" && b.stateLabel() == "standing outside" && b.standingAt() == "home");
    CHECK(b.planText().empty());
    // the marker's post is drawn beside them, not through them: kMarkerPostOffset away, further from the road than they
    CHECK(approxEqual((b.markers()[0].post - b.markers()[0].stand).length(), citysim::kMarkerPostOffset, kSamePoint));
    CHECK((b.markers()[0].post - b.markers()[0].frontage.on).length() > (b.markers()[0].stand - b.markers()[0].frontage.on).length());
    CHECK(r.sim().drivingV2()->carCount() == 0u);
    // THE SIM PLACING ITS POPULATION AGAIN (the viewer opens a level at its hour; a `daynight` jump) does not carry the
    // resident off: they are the brain's (Agent::brainOwned, ADR-0442), and the placing leaves them exactly as they
    // were -- looked at straight after it, before any tick could put them back, and again a tick later. (Seen in the
    // viewer before this was held: the resident stood 500 m from home, where the day of a role they do not keep had
    // them.)
    CHECK(r.person().brainOwned);
    r.lc.city.simMutable().seedFromSchedule(15.0);
    CHECK(approxEqual(r.person().pos.x, b.markers()[0].stand.x, kParkedWithin) &&
          approxEqual(r.person().pos.y, b.markers()[0].stand.y, kParkedWithin));
    CHECK(!r.person().indoors && !r.person().moving);
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
    CHECK(stateNames(b) == "standing walking pulling_out driving parking walking standing");
    CHECK(b.standingAt() == "work");
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
    // (two of the lights were red when it reached them: checkDrove held where it stood at each)
    CHECK(redLightStands(r.watch) == 2);
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
    {
        // THE PARKED BODY IS INSIDE ITS PAINTED BAY AND OFF THE SIDEWALK, AND FACES THE WAY ITS KERB'S TRAFFIC RUNS. The
        // bay is the street's parking band (its middle `parkingOffset` from the centre line, `parking` wide) over the
        // space's stretch of the street; the kerb is at the carriageway's half width. Measured across the road from
        // the centre line, toward the kerb.
        const engine::KerbSpace space = b.carSpace();
        const engine::ModelSection& section = model.streets[space.street.index].section;
        const Real kerbFlank = section.parkingOffset + r.car().width * Real(0.5);
        const Real roadFlank = section.parkingOffset - r.car().width * Real(0.5);
        const Real bandKerbEdge = section.parkingOffset + section.parking * Real(0.5);
        const Real bandRoadEdge = section.parkingOffset - section.parking * Real(0.5);
        const engine::KerbLanePlace lane = model.kerbLaneBeside(space);
        const Vec2 traffic = lane.valid() ? model.laneLevel.directionAt(lane.path, lane.along) : Vec2(0, 0);
        std::printf("    [brain-lab] parked in %s: the body %.2f m wide and %.2f m long in a bay %.2f m wide and %.2f m long; across "
                    "the road from the centre line: the body from %.2f to %.2f m, the bay from %.2f to %.2f m, the kerb at %.2f m; "
                    "it faces %.3f %.3f and its kerb lane's traffic runs %.3f %.3f\n",
                    model.kerbSpaceName(space).c_str(), r.car().width, r.car().length, space.width, space.length, roadFlank, kerbFlank,
                    bandRoadEdge, bandKerbEdge, section.carriagewayHalfWidth(), r.car().heading.x, r.car().heading.y, traffic.x,
                    traffic.y);
        CHECK(atOrBefore(r.car().length, space.length, kSamePoint));
        CHECK(atOrPast(roadFlank, bandRoadEdge, kSamePoint) && atOrBefore(kerbFlank, bandKerbEdge, kSamePoint));
        CHECK(atOrBefore(kerbFlank, section.carriagewayHalfWidth(), kSamePoint));   // no wheel on the kerb or the sidewalk
        CHECK(lane.valid() && approxEqual(engine::dot(r.car().heading, traffic), 1, kParkedWithin));
    }
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
    CHECK(stateNames(b) == "standing walking on_small_wheels standing");
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

TEST_CASE(level_brain_lab_a_goto_while_driving_redirects_the_drive) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    const engine::RoadModel& model = *r.sim().drivingV2()->model();
    const driving::Driving& d = *r.sim().drivingV2();
    std::string why;
    CHECK(b.goTo("work", &why));
    CHECK(r.until([&] { return b.stateName() == "driving" && r.watch.ticksDriven > 30u * 60u; }));
    CHECK(b.planText() == "walk>space s19 along 3, car>space s182 against 3, walk>work");
    CHECK(model.kerbSpaceName(b.heldSpace()) == "s182 against 3");
    const driving::CarId car(static_cast<uint32_t>(b.residentVehicle()));
    const driving::TripId trip = d.tripOf(car);
    CHECK(trip.valid());
    // THE SIM PLACING ITS POPULATION AGAIN WHILE THEY DRIVE leaves the driver and the car where they are (the per-tick
    // repair this replaced only ever looked at a person standing still)
    {
        const Vec2 carWas = r.car().pos, personWas = r.person().pos;
        r.lc.city.simMutable().seedFromSchedule(9.0);
        CHECK(approxEqual((r.car().pos - carWas).length(), 0, kSamePoint) &&
              approxEqual((r.person().pos - personWas).length(), 0, kSamePoint));
        CHECK(r.person().mode == citysim::Agent::Mode::Driver && r.sim().drivingV2()->car(car) != nullptr);
    }
    // TOLD TO BE AT THE SHOP WHILE DRIVING TO WORK: the drive is redirected at the next tick. The same trip of the same
    // car now ends beside the space nearest the shop; the plan is "drive there, walk"; the space at work is let go and
    // the new one held; nobody got out and nothing was parked.
    const std::size_t statesBefore = b.trace().size();
    CHECK(b.goTo("shop", &why));
    r.step();
    CHECK(b.stateName() == "driving" && b.trace().size() == statesBefore + 1);
    CHECK(b.planText() == "car>space s106 along 2, walk>shop");
    CHECK(model.kerbSpaceName(b.heldSpace()) == "s106 along 2");
    {
        // (that space is the model's nearest to the shop on foot)
        const int shopMarker = b.markerNamed("shop");
        CHECK(shopMarker == 2);
        if (shopMarker < 0) return;
        const std::vector<engine::KerbSpaceNear> nearShop =
            model.kerbSpacesNear(b.markers()[static_cast<std::size_t>(shopMarker)].frontage, citysim::kSpaceReach, 1);
        CHECK(nearShop.size() == 1u && model.kerbSpaceName(nearShop[0].space) == "s106 along 2");
    }
    CHECK(b.machine()->agent(0).legNow == 0 && b.machine()->agent(0).legCount == 2);
    CHECK(b.machine()->counts().plans == 2u && b.machine()->counts().redirects == 1u && b.machine()->counts().redirectsLeft == 0u);
    CHECK(b.counts().redirected == 1u && b.counts().tripsBegun == 1u && b.counts().parkedReports == 0u && b.counts().alighted == 0u);
    CHECK(d.tripOf(car) == trip);   // the same trip, steered: not ended and begun again
    {
        // ...and it ends where a car beside that space stands: on the kerb lane beside it
        const engine::KerbLanePlace beside = model.kerbLaneBeside(b.heldSpace());
        CHECK(beside.valid() && d.trip(trip) != nullptr && d.trip(trip)->destination.path == beside.path);
    }
    const bool arrived = r.until([&] { return r.there("shop"); });
    r.printTrip("home to work, told `goto shop` at the wheel");
    CHECK(arrived);
    if (!arrived) return;
    // one drive, entered twice (the second time is the redirect); parked once, at the shop; never at work
    CHECK(stateNames(b) == "standing walking pulling_out driving driving parking walking standing");
    CHECK(b.planText() == "car>space s106 along 2, walk>shop");
    CHECK(b.machine()->counts().plans == 2u && b.machine()->counts().arrivals == 1u);
    CHECK(b.machine()->counts().errors == 0u && b.machine()->counts().refused == 0u);
    CHECK(b.counts().parkedReports == 1u && b.counts().boarded == 1u && b.counts().alighted == 1u && b.counts().slidIn == 1u);
    CHECK(b.carSpaceName() == "s106 along 2" && !b.heldSpace().valid());
    const int shop = b.markerNamed("shop");
    CHECK(shop == 2);
    if (shop < 0) return;
    CHECK((r.person().pos - b.markers()[static_cast<std::size_t>(shop)].stand).length() <= kStandsWithin);
    CHECK(b.standingAt() == "shop");
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
    CHECK(stateNames(b) == "standing walking pulling_out driving parking driving parking walking standing");
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

// THE 300-CAR LAB, THE SAME QUESTION: where does the FIRST car of each queue at a red light stand, against the model's
// stop line? Two minutes of the traffic lab (RT_LAB_LINE_SECONDS for more), looked at ten times a second: every driven
// car at rest at the end of a lane arriving at a junction, with no car between it and the line, whose light is red.
// Held: the driving's line is the model's, and the front the driving has is at or before it. Reported and not held: how
// far behind it the fronts stand, and the body AS DRAWN (a lab car is drawn at its fleet slot's length, which may differ
// from its driving class's: the hand-over draws the middle of the class's body).
TEST_CASE(level_traffic_lab_first_cars_stand_at_their_stop_lines) {
    LoadedCity lc("traffic_lab.json");
    CHECK(lc.ok);
    if (!lc.ok) return;
    const citysim::CitySim& sim = lc.city.sim();
    const driving::Driving* d = sim.drivingV2();
    CHECK(d != nullptr);
    if (!d) return;
    const engine::RoadModel& model = *d->model();
    const Real seconds = std::getenv("RT_LAB_LINE_SECONDS") ? std::atof(std::getenv("RT_LAB_LINE_SECONDS")) : 120.0;
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    constexpr int kLookEvery = 6;
    constexpr Real kJustAhead = 0.01;   // a car's own front is not "a car ahead of it"
    uint64_t looks = 0, pastLine = 0, drawnPastLine = 0, farBehind = 0, linesApart = 0, sizesDiffer = 0;
    Real frontMost = -1e9, frontLeast = 1e9, drawnMost = -1e9, drawnLeast = 1e9, frontSum = 0;
    LineStop worst, worstDrawn;
    for (int i = 0; i < steps; ++i) {
        lc.city.step(lc.world, kStep);
        if (i % kLookEvery != 0) continue;
        for (const driving::Car& c : d->cars()) {
            if (c.id.index >= sim.vehicles().size()) continue;
            LineStop s;
            if (!measureAtLine(*d, model, c, sim.vehicles()[c.id.index], s)) continue;
            const engine::Path& path = model.laneLevel.paths[c.path.index];
            bool first = true;
            for (const driving::PathEntry& e : driving::carsOnStretch(d->pathLists(), c.path, c.along + kJustAhead, path.length))
                first = first && e.car == c.id;
            if (!first || lampAhead(*d, c) != driving::Lamp::Red) continue;
            s.at = d->clockSeconds();
            s.path = c.path.index;
            ++looks;
            linesApart += approxEqual(s.drivingLine, 0, kWrittenStation) ? 0 : 1;
            sizesDiffer += approxEqual(s.classLength, s.drawnLength, kWrittenStation) ? 0 : 1;
            pastLine += atOrBefore(s.front, 0, kAtTheLine) ? 0 : 1;
            drawnPastLine += atOrBefore(s.drawnFront, 0, kAtTheLine) ? 0 : 1;
            farBehind += atOrPast(s.front, -kStandsBehindLineWithin, kAtTheLine) ? 0 : 1;
            frontSum += s.front;
            if (s.front > frontMost) worst = s;
            frontMost = std::max(frontMost, s.front);
            frontLeast = std::min(frontLeast, s.front);
            if (s.drawnFront > drawnMost) worstDrawn = s;
            drawnMost = std::max(drawnMost, s.drawnFront);
            drawnLeast = std::min(drawnLeast, s.drawnFront);
        }
    }
    std::printf("    [lab-lines] %.0f s, %zu cars: %llu looks at a first car standing at a red light (metres along the lane from the "
                "model's stop line; below 0 is before it)\n",
                seconds, d->carCount(), static_cast<unsigned long long>(looks));
    std::printf("    [lab-lines] the driving's line apart from the model's: %llu looks. The driving's front: furthest on %+.2f, "
                "furthest back %+.2f, mean %+.2f; past the line %llu looks; more than %.1f m behind it %llu looks\n",
                static_cast<unsigned long long>(linesApart), frontMost, frontLeast, looks ? frontSum / static_cast<Real>(looks) : Real(0),
                static_cast<unsigned long long>(pastLine), kStandsBehindLineWithin, static_cast<unsigned long long>(farBehind));
    std::printf("    [lab-lines] the body as drawn: front furthest on %+.2f (t=%.1f path %u: class %.2f m, drawn %.2f m), furthest "
                "back %+.2f; past the line %llu looks; drawn at another length than its class %llu looks\n",
                drawnMost, worstDrawn.at, worstDrawn.path, worstDrawn.classLength, worstDrawn.drawnLength, drawnLeast,
                static_cast<unsigned long long>(drawnPastLine), static_cast<unsigned long long>(sizesDiffer));
    CHECK(looks > 100u);
    CHECK(linesApart == 0u);
    CHECK(pastLine == 0u);
    // (every lab car is drawn at its class's length today, so the body as drawn is held too; a fleet slot drawn longer
    // than its class would put a drawn nose past the line and fail here, which is the fault to find)
    CHECK(sizesDiffer > 0u || drawnPastLine == 0u);
}
