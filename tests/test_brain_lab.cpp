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
// level_brain_lab_a_household_car_is_one_entity_drawn_from_its_own_pose (ADR-0444): a household's car is one entity,
// no row of the sim's vehicles, and is drawn once, from its own pose, with its driver inside it.
//
// Every case reads a household's car from its entity (carView below), never from the sim's vehicle rows.
//
// RT_BRAIN_LAB_LOG=1 prints `brain?` at every change of state and every 30 s.

#include "test_framework.h"

#include "../src/apps/citysim/city_brain.h"
#include "../src/apps/citysim/city_car.h"
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

// A HOUSEHOLD'S CAR AS THE TESTS READ IT (ADR-0444): its entity's components, never a row of the sim's vehicles. Where
// it is and the way it points (its Transform), its drawn size (CityCarBody), who sits at its wheel and whether the
// player has it (CityCarLink). `found` false: the brain has no such car, or its entity lacks a component.
struct CarView {
    bool found = false;
    Vec2 pos, heading;
    Real length = 0, width = 0;
    int driver = -1;
    bool away = false;
};
CarView carView(World& world, const citysim::CityBrain& b, int car) {
    CarView v;
    const Entity e = b.carEntity(car);
    const Transform* t = world.get<Transform>(e);
    const citysim::CityCarBody* body = world.get<citysim::CityCarBody>(e);
    const citysim::CityCarLink* link = world.get<citysim::CityCarLink>(e);
    if (!t || !body || !link) return v;
    v.found = true;
    v.pos = citysim::carPosition(*t);
    v.heading = citysim::carHeading(*t);
    v.length = body->length;
    v.width = body->width;
    v.driver = link->driver;
    v.away = link->away;
    return v;
}

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
// (`drawnAt` and `drawnLength`: the middle of the body as drawn, and its drawn length: a sim vehicle row's for the lab's
// cars, a household car entity's for the brain's)
bool measureAtLine(const driving::Driving& d, const engine::RoadModel& model, const driving::Car& c, Vec2 drawnAt,
                   Real drawnLength, LineStop& s) {
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
    s.drawnLength = drawnLength;
    s.front = c.along - lineAlong;
    s.rear = s.front - s.classLength;
    // the drawn body, laid along the lane's direction at the line
    const engine::Vec2 lineAt = roads.pointAt(c.path, lineAlong), way = roads.directionAt(c.path, lineAlong);
    const Real middle = engine::dot(drawnAt - lineAt, way);
    s.drawnFront = middle + drawnLength * Real(0.5);
    s.drawnRear = middle - drawnLength * Real(0.5);
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

    void look(World& world, const citysim::CitySim& sim, const citysim::CityBrain& b, Vec2 start) {
        const driving::Driving& d = *sim.drivingV2();
        const CarView v = carView(world, b, b.residentCar());
        farthestFromStart = std::max(farthestFromStart, (v.pos - start).length());
        const driving::Car* c = d.car(driving::CarId(static_cast<uint32_t>(b.residentCar())));
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
    void lookAtLine(const driving::Driving& d, const engine::RoadModel& model, const CarView& v, const driving::Car& c) {
        LineStop now;
        if (!measureAtLine(d, model, c, v.pos, v.length, now)) {
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
std::string stateNames(const citysim::CityBrain& b, int who = 0) {
    std::string s;
    for (const citysim::brain::Transition& t : b.trace(who))
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
    std::vector<std::string> logged;
    bool log = std::getenv("RT_BRAIN_LAB_LOG") != nullptr;
    // HOLDS ON KERB SPACES, looked at every tick (ADR-0443; CityBrain::holdsSound): how many ticks found a fault, and
    // the first one found.
    int holdFaults = 0;
    std::string firstHoldFault;
    // A RIDER WHERE THEIR CAR IS (ADR-0445, #296), looked at every tick: how many (person, tick) pairs found a person
    // in a household car whose own place was not the car's.
    int riderApart = 0;

    // The level's SIX residents (ADR-0443), every day table switched off: a case says who goes where. The first
    // resident (ana, number 0) and her household's car are what the one-resident cases below are about; the other
    // five stand at home unless a case sends them.
    Run() : lc("brain_lab.json") {
        CHECK(lc.ok);
        if (!lc.ok) return;
        b = lc.city.simMutable().brain();
        CHECK(b != nullptr && b->hasResident());
        if (b && !b->hasResident()) b = nullptr;
        if (!b) return;
        b->setDay(false);
        carStart = car().pos;
        logged.resize(static_cast<std::size_t>(b->residentCount()));
    }
    const citysim::CitySim& sim() const { return lc.city.sim(); }
    const citysim::Agent& person(int who = 0) const { return sim().agents()[static_cast<std::size_t>(b->residentAgent(who))]; }
    // the car `who` would plan with, read from its entity
    CarView car(int who = 0) { return carView(lc.world, *b, b->residentCar(who)); }
    double seconds() const { return ticks * kStep; }
    void step() {
        lc.city.step(lc.world, kStep);
        ++ticks;
        watch.look(lc.world, sim(), *b, carStart);
        for (const citysim::Agent& a : sim().agents())
            if (a.inCar >= 0 && !approxEqual((a.pos - carView(lc.world, *b, a.inCar).pos).length(), 0, kSamePoint)) ++riderApart;
        std::string why;
        if (!b->holdsSound(&why)) {
            if (holdFaults++ == 0) firstHoldFault = "t=" + std::to_string(seconds()) + " " + why;
        }
        for (int who = 0; log && who < b->residentCount(); ++who)
            if (b->stateName(who) != logged[static_cast<std::size_t>(who)] || ticks % (60 * 60) == 0) {
                logged[static_cast<std::size_t>(who)] = b->stateName(who);
                std::printf("    [brain-lab] t=%.1f brain? %s\n", seconds(), b->describe(who).c_str());
            }
    }
    void run(double forSeconds) {
        const int n = static_cast<int>(forSeconds / kStep + 0.5);
        for (int i = 0; i < n; ++i) step();
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
    bool there(const std::string& marker, int who = 0) const {
        const citysim::brain::AgentBrain& a = b->machine()->agent(static_cast<citysim::brain::AgentId>(who));
        return !a.travelling() && b->wantedMarker(who) == b->markerNamed(marker) && b->stateName(who) == "standing" &&
               b->trace(who).size() > 2;
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

    // THE STATES, in order; pulling out and parking last the way's own minutes (travel.lua: 2 and 2)
    CHECK(stateNames(b) == "standing walking pulling_out driving parking walking standing");
    CHECK(b.standingAt() == "work");
    CHECK(stateLasted(b, 2) == 120000u && stateLasted(b, 4) == 120000u);
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
    // (three of the lights were red when it reached them: checkDrove held where it stood at each. 2 -> 3 at ADR-0452:
    // drivers give way to people at crosswalks now, and the drive's timing moved so it met a third red, at path 875)
    CHECK(redLightStands(r.watch) == 3);
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
    const driving::CarId car(static_cast<uint32_t>(b.residentCar()));
    const driving::TripId trip = d.tripOf(car);
    CHECK(trip.valid());
    // THE SIM PLACING ITS POPULATION AGAIN WHILE THEY DRIVE leaves the driver and the car where they are (the per-tick
    // repair this replaced only ever looked at a person standing still)
    {
        const Vec2 carWas = r.car().pos, personWas = r.person().pos;
        r.lc.city.simMutable().seedFromSchedule(9.0);
        CHECK(approxEqual((r.car().pos - carWas).length(), 0, kSamePoint) &&
              approxEqual((r.person().pos - personWas).length(), 0, kSamePoint));
        CHECK(b.inCar(0) && r.sim().drivingV2()->car(car) != nullptr);
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
            const citysim::SimVehicle& v = sim.vehicles()[c.id.index];
            if (!measureAtLine(*d, model, c, v.pos, v.length, s)) continue;
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

// ---- SEVERAL RESIDENTS, A SHARED HOUSEHOLD CAR, AND INTERRUPTIONS (ADR-0443; design sections 7 and 12) -----------------------
//
// The residents by number: 0 ana and 1 ben (household east, one car, at `home`); 2 cat, 3 dan (a board) and 4 eve
// (household trio, one car, at `west`); 5 fay (household solo, no car, a scooter, at `shop`).

namespace {
constexpr int kAna = 0, kBen = 1, kCat = 2, kDan = 3, kEve = 4, kFay = 5;

// The sim agent that is no resident: the level's seventh walker, for the possession channel to take over.
int bystander(const Run& r) {
    const std::vector<citysim::Agent>& agents = r.sim().agents();
    for (std::size_t i = 0; i < agents.size(); ++i)
        if (agents[i].mode == citysim::Agent::Mode::Pedestrian && r.b->residentOfAgent(static_cast<int>(i)) < 0) return static_cast<int>(i);
    return -1;
}
// How far a point is from the centre line of the road nearest it, less that road's carriageway half width: above zero
// is beyond the kerb line (on the sidewalk side), below is on the carriageway.
Real beyondTheKerb(const engine::RoadModel& model, Vec2 at) {
    const engine::RoadFrontage f = model.frontageAt(at);
    return f.valid() ? f.distance - f.section->carriagewayHalfWidth() : Real(-1e9);
}
}  // namespace

TEST_CASE(level_brain_lab_six_residents_in_three_households) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    CHECK(b.residentCount() == 6);
    if (b.residentCount() != 6) return;
    const char* const names[6] = {"ana", "ben", "cat", "dan", "eve", "fay"};
    const char* const houses[6] = {"east", "east", "trio", "trio", "trio", "solo"};
    const char* const homes[6] = {"home", "home", "west", "west", "west", "shop"};
    for (int who = 0; who < 6; ++who) {
        CHECK(b.residentName(who) == names[who] && b.householdName(who) == houses[who]);
        CHECK(b.residentNamed(names[who]) == who && b.residentNamed(std::to_string(who)) == who);
        // each on their own spot at their home marker, standing outside, wanting to be there
        CHECK(b.standingAt(who) == homes[who] && b.stateName(who) == "standing" && b.planText(who).empty());
        CHECK(b.wantedMarker(who) == b.markerNamed(homes[who]));
        CHECK((r.person(who).pos - b.standSpot(who, b.markerNamed(homes[who]))).length() <= kParkedWithin);
        CHECK(r.person(who).brainOwned && b.residentOfAgent(b.residentAgent(who)) == who);
    }
    CHECK(b.residentNamed("zed") == -1 && b.residentNamed("6") == -1 && b.residentNamed("") == -1);
    // NOBODY STANDS IN ANYBODY: every two residents are at least kStandApart apart, less a centimetre
    for (int x = 0; x < 6; ++x)
        for (int y = x + 1; y < 6; ++y)
            CHECK((r.person(x).pos - r.person(y).pos).length() >= citysim::kStandApart - kParkedWithin);
    // the first on the marker's own spot; the second kStandApart along the sidewalk from it
    CHECK((b.standSpot(kAna, 0) - b.markers()[0].stand).length() <= kParkedWithin);
    CHECK(approxEqual((b.standSpot(kBen, 0) - b.markers()[0].stand).length(), citysim::kStandApart, kParkedWithin));
    // THE HOUSEHOLDS AND THEIR CARS: two cars, each in the free kerb space nearest its marker; fay's household has none
    CHECK(b.householdOf(kAna) == std::vector<int>({kAna, kBen}) && b.householdOf(kDan) == std::vector<int>({kCat, kDan, kEve}));
    CHECK(b.householdOf(kFay) == std::vector<int>({kFay}));
    CHECK(b.cars().size() == 2u);
    if (b.cars().size() != 2u) return;
    CHECK(b.residentCar(kAna) == b.cars()[0] && b.residentCar(kBen) == b.cars()[0]);
    CHECK(b.residentCar(kCat) == b.cars()[1] && b.residentCar(kDan) == b.cars()[1] && b.residentCar(kEve) == b.cars()[1]);
    CHECK(b.residentCar(kFay) == -1);
    CHECK(b.membersOfCar(b.cars()[1]) == std::vector<int>({kCat, kDan, kEve}));
    CHECK(b.carHolder(b.cars()[0]) == citysim::CarHolder::Nobody && b.carHolder(b.cars()[1]) == citysim::CarHolder::Nobody);
    std::printf("    [brain-lab] east's car in %s; trio's car in %s\n", b.carSpaceName(kAna).c_str(), b.carSpaceName(kCat).c_str());
    CHECK(b.carSpaceName(kAna) == "s19 along 3");
    CHECK(b.carSpaceName(kCat) == "s229 along 3");
    CHECK(r.sim().drivingV2()->carCount() == 0u);
    // EACH PLANS WITH WHAT THEY HAVE, told by name: fay (no car, a scooter) rides it home; dan (a board) rides his to
    // the shop; cat, with neither, takes the household's car to `far`
    std::string why;
    CHECK(b.goTo(kFay, "home", &why) && b.goTo(kDan, "home", &why) && b.goTo(kCat, "far", &why));
    CHECK(!b.goTo(7, "home", &why) && !b.goTo(kAna, "nowhere", &why));
    r.step();
    std::printf("    [brain-lab] fay: %s (%s)\n    [brain-lab] dan: %s (%s)\n    [brain-lab] cat: %s (%s)\n", b.planText(kFay).c_str(),
                b.planReason(kFay).c_str(), b.planText(kDan).c_str(), b.planReason(kDan).c_str(), b.planText(kCat).c_str(),
                b.planReason(kCat).c_str());
    CHECK(b.planText(kFay) == "scooter>home");
    CHECK(b.planText(kDan) == "skateboard>home");
    CHECK(b.planText(kCat) == "walk>space s229 along 3, car>space s288 along 1, walk>far");
    CHECK(b.planReason(kFay) == "walk 4.6 min, car -- no car, scooter 1.9 min: scooter");
    CHECK(b.heldSpace(kCat).valid() && !b.heldSpace(kDan).valid() && !b.heldSpace(kFay).valid());
    CHECK(b.stateName(kAna) == "standing" && b.stateName(kBen) == "standing" && b.stateName(kEve) == "standing");
    CHECK(r.holdFaults == 0);
    CHECK(r.riderApart == 0);   // (ADR-0445: every rider, every tick, where their car is)
}

// WHOEVER REACHES THE HOUSEHOLD'S CAR FIRST TAKES IT (design section 12). Ana and ben, of one household, are told a
// second apart to be at far-apart places (`work`, east; `ramp`, by the east freeway ramps). Both plan with the car, which
// is free and stands beside them. BEN, told second, reaches it first (his spot at `home` is the nearer to it by
// kStandApart) and drives off; ana finds it gone at the kerb and goes on foot. The car stays where ben parks it (by
// `ramp`). Later ana, sent to `ramp`, is where the car is and takes it home; ben, left at `ramp`, then walks home,
// since the car is no longer there.
TEST_CASE(level_brain_lab_whoever_reaches_the_household_car_first_takes_it) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    const engine::RoadModel& model = *r.sim().drivingV2()->model();
    std::string why;
    CHECK(b.goTo(kAna, "work", &why));
    r.run(1.0);
    CHECK(b.goTo(kBen, "ramp", &why));
    r.step();
    // BOTH PLANS PRICE THE CAR: it is free at that moment and stands in its space beside home. Each holds a space of
    // their own at the far end.
    CHECK(b.planText(kAna) == "walk>space s19 along 3, car>space s182 against 3, walk>work");
    CHECK(b.planText(kBen) == "walk>space s19 along 3, car>space s223 against 3, walk>ramp");
    CHECK(b.planReason(kBen) == "walk 10.2 min, car 6.3 min (0.2 walk + 6.0 car + 0.2 walk): car");
    CHECK(model.kerbSpaceName(b.heldSpace(kAna)) == "s182 against 3" && model.kerbSpaceName(b.heldSpace(kBen)) == "s223 against 3");
    // BEN IS AT THE CAR FIRST and takes it; ANA, at the kerb a second later, finds it taken: `vehicle_gone`, and a
    // plan without it
    CHECK(r.until([&] { return b.stateName(kBen) == "pulling_out"; }));
    const double benTook = r.seconds();
    CHECK(b.carHolder(b.cars()[0]) == citysim::CarHolder::Resident && b.stateName(kAna) == "walking");
    CHECK(r.until([&] { return b.trace(kAna).size() >= 4u; }));
    const double anaFound = r.seconds();
    std::printf("    [brain-lab] ben took the car at %.2f s; ana found it gone at %.2f s: %s (%s)\n", benTook, anaFound,
                b.planText(kAna).c_str(), b.planReason(kAna).c_str());
    CHECK(approxEqual(benTook, 3.0, 0.02) && approxEqual(anaFound, 4.07, 0.02));
    CHECK(stateNames(b, kAna) == "standing walking pulling_out walking");
    CHECK(b.planText(kAna) == "walk>work");
    CHECK(b.planReason(kAna) == "walk 9.3 min, car -- the car is in use: walk");
    CHECK(!b.heldSpace(kAna).valid() && model.kerbSpaceName(b.heldSpace(kBen)) == "s223 against 3");
    CHECK(r.person(kAna).mode == citysim::Agent::Mode::Pedestrian && !b.inCar(kAna) && b.inCar(kBen));
    // BOTH ARRIVE: ben by car, ana on foot. The car is where ben left it, free.
    CHECK(r.until([&] { return r.there("work", kAna) && r.there("ramp", kBen); }));
    std::printf("    [brain-lab] t=%.1f ana: %s\n    [brain-lab] ben: %s\n", r.seconds(), b.traceText(kAna).c_str(), b.traceText(kBen).c_str());
    CHECK(stateNames(b, kBen) == "standing walking pulling_out driving parking walking standing");
    CHECK(stateNames(b, kAna) == "standing walking pulling_out walking standing");
    CHECK(b.carSpaceName(kAna) == "s223 against 3" && b.carHolder(b.cars()[0]) == citysim::CarHolder::Nobody);
    CHECK(b.standingAt(kAna) == "work" && b.standingAt(kBen) == "ramp");
    CHECK(b.counts().boarded == 1u && b.counts().pulledOut == 1u && b.counts().slidIn == 1u);
    // ANA GOES TO WHERE THE CAR IS (on foot: it is not worth walking to a car to drive five minutes), and from there
    // takes it home
    CHECK(b.goTo(kAna, "ramp", &why));
    r.step();
    std::printf("    [brain-lab] ana to the ramp: %s (%s)\n", b.planText(kAna).c_str(), b.planReason(kAna).c_str());
    CHECK(b.planText(kAna) == "walk>ramp");
    CHECK(r.until([&] { return r.there("ramp", kAna); }));
    CHECK(b.counts().boarded == 1u);
    CHECK(b.goTo(kAna, "home", &why));
    r.step();
    std::printf("    [brain-lab] ana home from the ramp: %s (%s)\n", b.planText(kAna).c_str(), b.planReason(kAna).c_str());
    CHECK(b.planText(kAna) == "walk>space s223 against 3, car>space s19 along 3, walk>home");
    CHECK(r.until([&] { return r.there("home", kAna); }));
    std::printf("    [brain-lab] t=%.1f ana: %s\n", r.seconds(), b.traceText(kAna).c_str());
    CHECK(b.counts().boarded == 2u && b.counts().slidIn == 2u && b.carSpaceName(kAna) == "s19 along 3");
    // ...AND BEN, STILL AT THE RAMP, walks home: the car is no longer where he left it
    CHECK(b.goTo(kBen, "home", &why));
    r.step();
    std::printf("    [brain-lab] ben home from the ramp: %s (%s)\n", b.planText(kBen).c_str(), b.planReason(kBen).c_str());
    CHECK(b.planText(kBen) == "walk>home");
    CHECK(r.holdFaults == 0);
    CHECK(r.riderApart == 0);   // (ADR-0445: every rider, every tick, where their car is)
    if (r.holdFaults > 0) std::printf("    [brain-lab] holds: %s\n", r.firstHoldFault.c_str());
    CHECK(b.machine()->counts().errors == 0u && b.machine()->counts().refused == 0u);
    checkDrove(r.watch);
}

// SOMEBODY ELSE TAKES A HOUSEHOLD'S PARKED CAR (the possession channel's `board`: CityBrain::board, ADR-0444). The car
// is no longer free and no longer in its space; a resident planning later does not price it; one already walking to it
// is told `vehicle_gone` at the kerb. Left again (`alight`: CityBrain::alight), it stands where it was left, free.
TEST_CASE(level_brain_lab_a_parked_car_taken_by_somebody_else) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    const int other = bystander(r);
    CHECK(other >= 0);
    if (other < 0) return;
    const int car = b.cars()[0];
    std::string why;
    // ANA IS WALKING TO THE CAR when it is taken
    CHECK(b.goTo(kAna, "work", &why));
    r.run(1.0);
    CHECK(b.planText(kAna) == "walk>space s19 along 3, car>space s182 against 3, walk>work" && b.stateName(kAna) == "walking");
    CHECK(b.board(other, car));
    // (in the car: not drawn on foot, and at its wheel)
    CHECK(r.sim().agents()[static_cast<std::size_t>(other)].inCar == car && !r.sim().pedVisible(other) && r.car(kAna).driver == other);
    CHECK(b.carHolder(car) == citysim::CarHolder::Outsider && b.carSpaceName(kAna).empty());
    CHECK(b.counts().takenStanding == 1u);
    // BEN, PLANNING NOW, does not price it
    CHECK(b.goTo(kBen, "ramp", &why));
    r.step();
    std::printf("    [brain-lab] ben, the car taken: %s (%s)\n", b.planText(kBen).c_str(), b.planReason(kBen).c_str());
    CHECK(b.planText(kBen) == "walk>ramp");
    CHECK(b.planReason(kBen).find("car -- no car") != std::string::npos);
    // ANA, AT THE KERB: the car is gone; on foot from there, and the space she held at work is let go
    CHECK(r.until([&] { return b.trace(kAna).size() >= 4u; }));
    std::printf("    [brain-lab] t=%.2f ana at the kerb: %s | %s (%s)\n", r.seconds(), b.traceText(kAna).c_str(), b.planText(kAna).c_str(),
                b.planReason(kAna).c_str());
    CHECK(stateNames(b, kAna) == "standing walking pulling_out walking");
    CHECK(b.planText(kAna) == "walk>work" && !b.heldSpace(kAna).valid());
    CHECK(b.counts().boarded == 0u && r.sim().drivingV2()->carCount() == 0u);
    // LEFT AGAIN WHERE IT STOOD: free, in its space, and a plan made now prices it
    CHECK(b.alight(other));
    CHECK(r.sim().agents()[static_cast<std::size_t>(other)].inCar == -1 && r.car(kAna).driver == -1);
    CHECK(b.carHolder(car) == citysim::CarHolder::Nobody && b.carSpaceName(kAna) == "s19 along 3" && b.counts().leftAgain == 1u);
    CHECK(b.goTo(kBen, "ramp", &why));
    r.step();
    CHECK(b.planText(kBen) == "walk>space s19 along 3, car>space s223 against 3, walk>ramp");
    CHECK(r.holdFaults == 0);
    CHECK(r.riderApart == 0);   // (ADR-0445: every rider, every tick, where their car is)
    CHECK(b.machine()->counts().errors == 0u && b.machine()->counts().refused == 0u);
}

// THE PLAYER TAKES THE CAR WHILE A RESIDENT DRIVES IT (CityBrain::takenByPlayer: the brain's side of it. ADR-0444: the
// player's key does not reach it yet; CityVehicleSystem's promotion of a household car is sitting 1b). The resident is
// put out on the kerb side and told `vehicle_gone`; the trip is ended and the car is off the driving's road; they plan
// again on foot from where they stand. Then the player leaves the car: first in a kerb space, then at a point in a
// lane, then in a field, and a household member plans with it, or not.
TEST_CASE(level_brain_lab_the_player_takes_the_car_from_under_its_driver) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    const driving::Driving& d = *r.sim().drivingV2();
    const engine::RoadModel& model = *d.model();
    const int car = b.cars()[0];
    std::string why;
    CHECK(b.goTo(kAna, "work", &why));
    CHECK(r.until([&] { return b.stateName(kAna) == "driving" && r.watch.ticksDriven > 30u * 60u; }));
    CHECK(d.carCount() == 1u && model.kerbSpaceName(b.heldSpace(kAna)) == "s182 against 3");
    const Vec2 carWas = r.car(kAna).pos;
    CHECK(beyondTheKerb(model, carWas) < 0);   // (the car is on the carriageway)
    // TAKEN
    CHECK(b.takenByPlayer(car));
    CHECK(r.person(kAna).mode == citysim::Agent::Mode::Pedestrian && r.person(kAna).vehicle < 0 && !b.inCar(kAna));
    CHECK(r.car(kAna).away && r.car(kAna).driver == -1);   // (the player's own car now: not drawn here)
    // out on the kerb side: beyond the kerb line of the road the car was on, and within a road's width of the car
    std::printf("    [brain-lab] t=%.1f taken at %.1f %.1f; ana put out at %.1f %.1f, %.2f m beyond the kerb line, %.1f m from the car\n",
                r.seconds(), carWas.x, carWas.y, r.person(kAna).pos.x, r.person(kAna).pos.y, beyondTheKerb(model, r.person(kAna).pos),
                (r.person(kAna).pos - carWas).length());
    CHECK(beyondTheKerb(model, r.person(kAna).pos) > 0);
    {
        // ...ON THE CAR'S OWN SIDE of the road it was on (the kerb of its own traffic), level with it: the middle of
        // that sidewalk, which is the carriageway's half width and half a sidewalk from the centre line
        const engine::RoadFrontage carOn = model.frontageAt(carWas), personOn = model.frontageAt(r.person(kAna).pos);
        std::printf("    [brain-lab] the car was %.2f m from the centre line of a road %.2f m to the kerb, on the kerb `dir` %d; "
                    "ana is on `dir` %d\n",
                    carOn.distance, carOn.section->carriagewayHalfWidth(), carOn.dir, personOn.dir);
        CHECK(carOn.street == personOn.street && carOn.dir == personOn.dir);
        CHECK(approxEqual(carOn.station, personOn.station, 0.5));
        CHECK((r.person(kAna).pos - carWas).length() < carOn.section->carriagewayHalfWidth() + 5.0);
    }
    // the trip is ended and the car is off the driving's road, at once
    CHECK(d.carCount() == 0u && d.car(driving::CarId(static_cast<uint32_t>(car))) == nullptr);
    CHECK(b.carHolder(car) == citysim::CarHolder::Outsider && b.carSpaceName(kAna).empty());
    CHECK(b.counts().takenDriven == 1u && b.counts().putOut == 1u);
    CHECK(!r.sim().drivenByV2(b.residentAgent(kAna)));
    r.step();
    // THE BRAIN DECIDES WHAT FOLLOWS: `vehicle_gone` at the wheel is "plan again from here": on foot, the hold let go
    CHECK(stateNames(b, kAna) == "standing walking pulling_out driving walking");
    CHECK(b.planText(kAna) == "walk>work" && !b.heldSpace(kAna).valid());
    CHECK(b.planReason(kAna).find("car -- no car") != std::string::npos);
    const bool arrived = r.until([&] { return r.there("work", kAna); });
    std::printf("    [brain-lab] t=%.1f ana: %s\n", r.seconds(), b.traceText(kAna).c_str());
    CHECK(arrived);
    CHECK(stateNames(b, kAna) == "standing walking pulling_out driving walking standing");
    CHECK(b.counts().alighted == 0u && b.counts().parkedReports == 0u);   // nobody parked anything
    // THE PLAYER LEAVES IT IN A KERB SPACE (the second space by home): it stands there, free, and ben plans with it
    {
        const std::vector<engine::KerbSpaceNear> nearHome = model.kerbSpacesNear(b.markers()[0].frontage, citysim::kSpaceReach, 2);
        CHECK(nearHome.size() == 2u);
        if (nearHome.size() != 2u) return;
        engine::KerbSpacePose pose;
        CHECK(model.kerbSpacePose(nearHome[1].space, pose));
        const std::string name = model.kerbSpaceName(nearHome[1].space);
        std::printf("    [brain-lab] left in %s\n", name.c_str());
        // (half a metre off where a car in the space stands, and a little askew: a player parks roughly)
        CHECK(b.carLeftAt(car, pose.at + pose.facing * 0.5, pose.facing));
        CHECK(b.carHolder(car) == citysim::CarHolder::Nobody && b.carSpaceName(kBen) == name && b.counts().leftAgain == 1u);
        CHECK(!r.car(kBen).away);   // (drawn again)
        CHECK(b.goTo(kBen, "work", &why));
        r.step();
        std::printf("    [brain-lab] ben, the car left in a space: %s (%s)\n", b.planText(kBen).c_str(), b.planReason(kBen).c_str());
        CHECK(b.planText(kBen).rfind("walk>space " + name + ", car>space ", 0) == 0);
        CHECK(r.until([&] { return b.stateName(kBen) == "driving" && d.carCount() == 1u; }));
        CHECK(b.carHolder(car) == citysim::CarHolder::Resident);
    }
    // TAKEN FROM BEN TOO, AND LEFT AT A POINT IN A LANE: it is put into the kerb lane beside that point, and cat's
    // household is not asked; ana, of its own household, plans with it from there
    {
        CHECK(b.takenByPlayer(car));
        const Vec2 at = r.person(kBen).pos;   // the kerb side of where it was taken
        const engine::RoadFrontage f = model.frontageAt(at);
        const engine::KerbLanePlace lane = model.kerbLaneBeside(f);
        CHECK(lane.valid());
        if (!lane.valid()) return;
        const Vec2 facing = model.laneLevel.directionAt(lane.path, lane.along);
        CHECK(b.carLeftAt(car, lane.at, facing));
        std::printf("    [brain-lab] left at a point: %s\n", b.describeCar(car).c_str());
        CHECK(b.carHolder(car) == citysim::CarHolder::Nobody && b.carSpaceName(kAna).empty());
        CHECK(d.car(driving::CarId(static_cast<uint32_t>(car))) != nullptr);   // the driving knows it stands in the lane
        CHECK(b.describeCar(car).find("left at the point") != std::string::npos);
        r.step();
        // ben stands on the kerb side beside it, still wanting to be at work: he plans with it from that point
        CHECK(b.goTo(kBen, "work", &why));
        r.step();
        std::printf("    [brain-lab] ben, the car left at a point: %s (%s)\n", b.planText(kBen).c_str(), b.planReason(kBen).c_str());
        CHECK(b.planText(kBen).rfind("walk>the point ", 0) == 0 &&
              b.planText(kBen).find(", car>space s182 against 3, walk>work") != std::string::npos);
        const uint64_t plans = b.machine()->counts().plans;
        CHECK(r.until([&] { return b.stateName(kBen) == "driving" && d.tripOf(driving::CarId(static_cast<uint32_t>(car))).valid(); }));
        std::printf("    [brain-lab] t=%.1f ben: %s\n", r.seconds(), b.traceText(kBen).c_str());
        // the few steps to the car, into it, and off: by that one plan
        const std::string states = stateNames(b, kBen);
        const std::string tail = " walking pulling_out driving";
        CHECK(states.size() > tail.size() && states.compare(states.size() - tail.size(), tail.size(), tail) == 0);
        // ...getting in took the way's two minutes (travel.lua), not ended early by the few steps' walk (ADR-0444: a
        // walk of no length, which the machine ends on the spot, went on being stepped from inside the car)
        const std::vector<citysim::brain::Transition>& trace = b.trace(kBen);
        CHECK(trace.size() >= 2u && trace.back().at - trace[trace.size() - 2].at == 120000u);
        CHECK(b.machine()->counts().plans == plans);
        CHECK(b.carHolder(car) == citysim::CarHolder::Resident && b.inCar(kBen));
    }
    // TAKEN ONCE MORE AND LEFT IN A FIELD (the player's spawn, off every road): nobody can drive it from there, and
    // nobody plans with it
    {
        CHECK(b.takenByPlayer(car));
        r.step();
        CHECK(b.carLeftAt(car, Vec2(-40.0, -75.0), Vec2(0, 1)));
        std::printf("    [brain-lab] left in a field: %s\n", b.describeCar(car).c_str());
        CHECK(b.describeCar(car).find("off the road") != std::string::npos);
        CHECK(d.car(driving::CarId(static_cast<uint32_t>(car))) == nullptr);
        CHECK(b.goTo(kBen, "work", &why));
        r.step();
        CHECK(b.planText(kBen) == "walk>work" && b.planReason(kBen).find("car -- no car") != std::string::npos);
    }
    CHECK(r.holdFaults == 0);
    CHECK(r.riderApart == 0);   // (ADR-0445: every rider, every tick, where their car is)
    if (r.holdFaults > 0) std::printf("    [brain-lab] holds: %s\n", r.firstHoldFault.c_str());
    CHECK(b.machine()->counts().errors == 0u);
}

// A DIRECTOR TAKES A RESIDENT OVER (the possession channel: CitySim::brainNoteHeld, what `possess walker` and `release`
// call). The brain's `possessed` state holds them: the intent is kept, the record does not change by a byte, and the
// director moves them. Let go, they plan again from where they stand and carry on with the intent they had.
TEST_CASE(level_brain_lab_a_possessed_resident_is_held_and_carries_on_when_let_go) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    citysim::CitySim& sim = r.lc.city.simMutable();
    std::string why;
    CHECK(b.goTo(kAna, "shop", &why));
    r.run(20.0);
    CHECK(b.stateName(kAna) == "walking" && b.planText(kAna) == "walk>shop");
    const int agent = b.residentAgent(kAna);
    sim.brainNoteHeld(agent, true);
    r.step();
    CHECK(stateNames(b, kAna) == "standing walking possessed" && b.stateLabel(kAna) == "held by the player");
    CHECK(b.counts().held == 1u);
    // THE DIRECTOR WALKS THEM BACK TOWARD HOME for 30 s; the brain's record is the same bytes throughout
    const citysim::brain::AgentBrain held = b.machine()->agent(kAna);
    const Vec2 from = r.person(kAna).pos;
    CHECK(sim.sendAgentTo(agent, b.standSpot(kAna, b.markerNamed("home"))));
    bool same = true;
    for (int i = 0; i < 30 * 60; ++i) {
        r.step();
        const citysim::brain::AgentBrain& now = b.machine()->agent(kAna);
        same = same && std::equal(reinterpret_cast<const unsigned char*>(&held), reinterpret_cast<const unsigned char*>(&held) + sizeof(held),
                                  reinterpret_cast<const unsigned char*>(&now));
    }
    CHECK(same);
    const Vec2 movedTo = r.person(kAna).pos;
    std::printf("    [brain-lab] held at %.1f %.1f; the director walked them %.1f m to %.1f %.1f\n", from.x, from.y,
                (movedTo - from).length(), movedTo.x, movedTo.y);
    CHECK((movedTo - from).length() > 15.0);   // (the director really moved them: about 1.4 m/s for 30 s, less the turn)
    CHECK(b.wantedMarker(kAna) == b.markerNamed("shop") && b.stateName(kAna) == "possessed");
    CHECK(r.person(kAna).brainOwned);
    // LET GO: released, then at once planning from where they stand, for the shop they still want
    sim.setAgentDirected(agent, false);   // (what the possession channel does on release, before it tells the brain)
    sim.brainNoteHeld(agent, false);
    r.step();
    CHECK(stateNames(b, kAna) == "standing walking possessed released walking");
    CHECK(b.planText(kAna) == "walk>shop" && b.counts().letGo == 1u);
    CHECK(r.sim().agentDirected(agent));
    {
        const citysim::brain::AgentBrain& a = b.machine()->agent(kAna);
        const Vec2 origin(citysim::brain::fromCentimetres(a.originX), citysim::brain::fromCentimetres(a.originZ));
        CHECK((origin - movedTo).length() < 0.5);   // planned from where the director left them
    }
    const bool arrived = r.until([&] { return r.there("shop", kAna); });
    std::printf("    [brain-lab] t=%.1f ana: %s\n", r.seconds(), b.traceText(kAna).c_str());
    CHECK(arrived);
    CHECK(stateNames(b, kAna) == "standing walking possessed released walking standing");
    CHECK(b.machine()->counts().errors == 0u && b.machine()->counts().refused == 0u);
}

// ONE CAR, ONE REPRESENTATION (ADR-0444; docs/two-drivers-meeting-design.md section 2, principle 3). A household's car
// is one entity of the World and nothing else: no row of the sim's vehicles stands for it, and it is drawn from its own
// pose, not from its driver's. Held, at the start and 30 s into ana's drive to work:
//   - every household car's number is past the sim's last vehicle row (a car that was also a row would be one of them),
//     and no vehicle row stands where a household car stands;
//   - the World holds exactly the two cars, each one entity with its body, its link and, on the road only, its lane
//     component naming its car in the driving;
//   - on the road its pose is the driving's (the middle of the body), and its driver sits in it: not drawn on foot, her
//     own place and heading the car's (ADR-0445, #296), and at no tick of the whole run was any rider's place apart from
//     their car's;
//   - it is drawn ONCE in the city's car groups, by its own name, where its entity is, listed under its driver (who is
//     drawn in its seat); nothing else is drawn where it is; the parked car is drawn once where its entity is, listed
//     under its own number.
TEST_CASE(level_brain_lab_a_household_car_is_one_entity_drawn_from_its_own_pose) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    World& world = r.lc.world;
    const driving::Driving& d = *r.sim().drivingV2();
    const std::vector<int> cars = b.cars();
    CHECK(cars.size() == 2u);
    if (cars.size() != 2u) return;
    // (the lab's level makes no traffic: the sim's vehicle rows are the fire engines and parked cars of the build, if any)
    const std::size_t rows = r.sim().vehicles().size();
    std::printf("    [brain-lab] %zu vehicle rows; household cars %d and %d\n", rows, cars[0], cars[1]);
    CHECK(cars[0] == static_cast<int>(rows) && cars[1] == static_cast<int>(rows) + 1);
    // the cars are entities, and only these two
    int entities = 0;
    world.each<citysim::CityCarBody, citysim::CityCarLink, Transform>(
        [&](Entity e, citysim::CityCarBody& body, citysim::CityCarLink&, Transform&) {
            ++entities;
            CHECK(b.carEntity(static_cast<int>(body.id)) == e);
            CHECK(!world.has<citysim::CityCarOnLane>(e));   // (parked, both of them)
        });
    CHECK(entities == 2);
    for (const int car : cars) {
        const CarView v = carView(world, b, car);
        CHECK(v.found);
        for (const citysim::SimVehicle& row : r.sim().vehicles()) CHECK((row.pos - v.pos).length() > 1.0);
    }

    // ANA DRIVES TO WORK, 30 s into the drive
    std::string why;
    CHECK(b.goTo(kAna, "work", &why));
    CHECK(r.until([&] { return b.stateName(kAna) == "driving" && r.watch.ticksDriven > 30u * 60u; }));
    const int car = cars[0];
    const Entity e = b.carEntity(car);
    const CarView v = carView(world, b, car);
    const citysim::CityCarOnLane* lane = world.get<citysim::CityCarOnLane>(e);
    CHECK(lane != nullptr && lane->car == driving::CarId(static_cast<uint32_t>(car)) && lane->driven);
    const driving::Car* placed = d.car(driving::CarId(static_cast<uint32_t>(car)));
    CHECK(placed != nullptr);
    if (!lane || !placed) return;
    {
        // its pose is the driving's: the middle of the body
        const driving::CarPose pose = driving::carPose(*d.roads(), d.vehicleClasses(), d.ruleSet(), *placed);
        const Vec3 mid = (pose.front + pose.rear) * 0.5;
        CHECK(approxEqual(v.pos.x, mid.x, kSamePoint) && approxEqual(v.pos.y, mid.z, kSamePoint));
    }
    // the driver is in it: at its wheel, not drawn on foot; her own place and heading are the car's (ADR-0445, #296:
    // the car is the authority, and the person follows it every tick)
    const int agent = b.residentAgent(kAna);
    const citysim::Agent& ana = r.person(kAna);
    CHECK(ana.inCar == car && v.driver == agent && !r.sim().pedVisible(agent));
    CHECK(ana.mode == citysim::Agent::Mode::Pedestrian && ana.vehicle < 0);
    std::printf("    [brain-lab] the car at %.1f %.1f; ana's own place %.1f %.1f, %.3f m from it; ticks a rider stood apart from "
                "their car: %d\n",
                v.pos.x, v.pos.y, ana.pos.x, ana.pos.y, (ana.pos - v.pos).length(), r.riderApart);
    CHECK(approxEqual((ana.pos - v.pos).length(), 0, kSamePoint));
    CHECK(approxEqual(engine::dot(ana.heading, v.heading), 1, kParkedWithin));
    CHECK(r.riderApart == 0);
    // DRAWN once, by its own name, where its entity is, under its driver; the parked one under its own number
    const uint64_t name = (uint64_t(1) << 61) | (static_cast<uint64_t>(car) << 8);
    const uint64_t parkedName = (uint64_t(1) << 61) | (static_cast<uint64_t>(cars[1]) << 8);
    const CarView parked = carView(world, b, cars[1]);
    int drawn = 0, drawnParked = 0, atTheCar = 0;
    const std::vector<Entity>& groups = r.lc.city.carGroups();
    const std::vector<std::vector<int>>& keys = r.lc.city.carAgentIds();
    for (std::size_t g = 0; g < groups.size(); ++g) {
        const InstanceGroup* group = world.get<InstanceGroup>(groups[g]);
        if (!group) continue;
        for (std::size_t k = 0; k < group->transforms.size(); ++k) {
            const Vec2 at(group->transforms[k].m[0][3], group->transforms[k].m[2][3]);
            atTheCar += (at - v.pos).length() < 2.0 ? 1 : 0;
            const uint64_t id = k < group->ids.size() ? group->ids[k] : 0;
            const int key = g < keys.size() && k < keys[g].size() ? keys[g][k] : -1;
            if (id == name) {
                ++drawn;
                CHECK(approxEqual(at.x, v.pos.x, kParkedWithin) && approxEqual(at.y, v.pos.y, kParkedWithin));
                CHECK(key == agent);
            }
            if (id == parkedName) {
                ++drawnParked;
                CHECK(approxEqual(at.x, parked.pos.x, kParkedWithin) && approxEqual(at.y, parked.pos.y, kParkedWithin));
                CHECK(key == -2 - cars[1]);
            }
        }
    }
    CHECK(drawn == 1 && drawnParked == 1);
    CHECK(atTheCar == 1);   // (one car drawn where the car is: nothing else, its driver's included)
    for (const std::vector<int>& walkers : r.lc.city.pedAgentIds())
        for (const int w : walkers) CHECK(w != agent);
}

// EVERY SYSTEM THAT LOOKS FOR CARS SEES A HOUSEHOLD'S CAR (ADR-0445): carsNear (city_car.h) is the one question the
// engine note, the walkers' dodge, the possessed car's sensing and the player's key ask. Held: parked, the car is in
// it, standing, off the road, with its body's size; 30 s into a drive it is in it on the road, on its trip, at the
// driving's speed, with its driver; once the player holds it (the same entity with an engine::Vehicle, as
// CityVehicleSystem makes it) it is NOT in it (VehicleSystem's, found as a Vehicle: one car, one representation) and
// the city bake no longer draws it.
TEST_CASE(level_brain_lab_every_system_that_looks_for_cars_sees_a_household_car) {
    Run r;
    if (!r.b) return;
    citysim::CityBrain& b = *r.b;
    World& world = r.lc.world;
    const int car = b.cars()[0];
    const Entity e = b.carEntity(car);
    const auto find = [&](Vec2 at) {
        std::vector<citysim::CarSeen> seen;
        citysim::carsNear(r.sim(), world, at, 5.0, seen);
        citysim::CarSeen out;
        for (const citysim::CarSeen& c : seen)
            if (c.entity == e) out = c;
        return out;
    };
    {
        const CarView v = carView(world, b, car);
        const citysim::CarSeen c = find(v.pos);
        CHECK(c.entity == e && c.id == static_cast<uint32_t>(car) && c.agent == -1 && c.driver == -1);
        CHECK(!c.onRoad && !c.moving && approxEqual(c.speed, 0, kSamePoint));
        CHECK(approxEqual(c.length, v.length, kSamePoint) && approxEqual(c.width, v.width, kSamePoint));
    }
    std::string why;
    CHECK(b.goTo(kAna, "work", &why));
    CHECK(r.until([&] { return b.stateName(kAna) == "driving" && r.watch.ticksDriven > 30u * 60u; }));
    {
        const CarView v = carView(world, b, car);
        const citysim::CarSeen c = find(v.pos);
        const driving::Car* placed = r.sim().drivingV2()->car(driving::CarId(static_cast<uint32_t>(car)));
        CHECK(c.entity == e && c.onRoad && c.moving && c.driver == b.residentAgent(kAna));
        CHECK(placed != nullptr && approxEqual(c.speed, placed ? placed->speed : -1, kSamePoint));
        std::printf("    [brain-lab] carsNear finds the household car on the road at %.1f m/s\n", c.speed);
    }
    // THE PLAYER HOLDS IT (as CityVehicleSystem does: the brain told, then the same entity gains its Vehicle)
    CHECK(b.takenByPlayer(car));
    world.add<engine::Vehicle>(e, engine::Vehicle{});
    const Vec2 heldAt = carView(world, b, car).pos;
    r.step();
    CHECK(!find(heldAt).entity.valid());
    CHECK(!world.has<citysim::CityCarOnLane>(e) && r.sim().drivingV2()->car(driving::CarId(static_cast<uint32_t>(car))) == nullptr);
    int drawn = 0;
    for (const Entity g : r.lc.city.carGroups())
        if (const InstanceGroup* group = world.get<InstanceGroup>(g))
            for (const uint64_t id : group->ids) drawn += id == ((uint64_t(1) << 61) | (static_cast<uint64_t>(car) << 8)) ? 1 : 0;
    CHECK(drawn == 0);
    CHECK(b.counts().takenDriven == 1u && b.counts().putOut == 1u && !b.inCar(kAna));
}
