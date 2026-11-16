#include "brain_fakes.h"

#include "apps/citysim/brain/scripting/brain_lua.h"
#include "engine/ai/pathfind.h"
#include "engine/procgen/city/road_network.h"
#include "engine/scripting/script_vm.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace brain_test {

namespace {

BrainTime millisFor(Real metres, Real speed) { return static_cast<BrainTime>(std::lround(metres / speed * 1000.0)); }

// Where something is between two points, by the clock.
Vec2 between(Vec2 from, Vec2 to, BrainTime start, BrainTime end, BrainTime now) {
    if (end <= start || now >= end) return to;
    if (now <= start) return from;
    return from + (to - from) * (static_cast<Real>(now - start) / static_cast<Real>(end - start));
}

// The start and end of something that takes `total` and has `fractionDone` of it behind it at `now`.
void spanFrom(BrainTime now, BrainTime total, Real fractionDone, BrainTime& start, BrainTime& end) {
    const BrainTime gone = static_cast<BrainTime>(std::lround(static_cast<Real>(total) * fractionDone));
    start = now >= gone ? now - gone : 0;
    end = start + total;
}

}  // namespace

// ---- the world ---------------------------------------------------------------------------------------------------------
PlaceRef FakeWorld::addDoor(Vec2 at) {
    places.push_back(at);
    spaceCar.push_back(kNotSpace);
    PlaceRef p;
    p.id = static_cast<uint32_t>(places.size() - 1);
    return p;
}

PlaceRef FakeWorld::addSpace(Vec2 at) {
    const PlaceRef p = addDoor(at);
    spaceCar[p.id] = CarRef::kNone;
    return p;
}

CarRef FakeWorld::addCar(PlaceRef space) {
    Car c;
    c.standing = space;
    cars.push_back(c);
    CarRef car;
    car.id = static_cast<uint32_t>(cars.size() - 1);
    spaceCar[space.id] = car.id;
    return car;
}

void FakeWorld::setAgents(std::size_t count) {
    bodyAt.resize(count);
    bodyKnown.resize(count, 0);
    carOfPerson.resize(count, CarRef::kNone);
}

void FakeWorld::noteBody(AgentId who, Vec2 at) {
    if (who >= bodyAt.size()) return;
    bodyAt[who] = at;
    bodyKnown[who] = 1;
}

bool FakeWorld::positionOf(PlaceRef place, Vec2& out) const {
    if (place.id >= places.size()) return false;
    out = places[place.id];
    return true;
}

bool FakeWorld::bodyPosition(AgentId who, Vec2& out) const {
    if (who >= bodyKnown.size()) return false;
    CarRef car;
    car.id = carOfPerson[who];
    if (car.valid() && drive && drive->where(car, out)) return true;
    if (walk && walk->where(who, out)) return true;
    if (!bodyKnown[who]) return false;
    out = bodyAt[who];
    return true;
}

bool FakeWorld::carStanding(CarRef car, PlaceRef& out) const {
    if (car.id >= cars.size()) return false;
    out = cars[car.id].standing;
    return true;
}

bool FakeWorld::carFreeFor(CarRef car, AgentId who) const {
    return car.id < cars.size() && (cars[car.id].heldBy == kNoAgent || cars[car.id].heldBy == who);
}

bool FakeWorld::claimCar(CarRef car, AgentId who) {
    if (!carFreeFor(car, who)) return false;
    cars[car.id].heldBy = who;
    if (who < carOfPerson.size()) carOfPerson[who] = car.id;
    return true;
}

void FakeWorld::releaseCar(CarRef car, AgentId who) {
    if (car.id >= cars.size() || cars[car.id].heldBy != who) return;
    cars[car.id].heldBy = kNoAgent;
    if (who < carOfPerson.size()) carOfPerson[who] = CarRef::kNone;
}

void FakeWorld::leaveCar(CarRef car, AgentId who) {
    if (car.id >= cars.size() || cars[car.id].heldBy != who) return;
    Vec2 at;
    if (!(drive && drive->where(car, at))) positionOf(cars[car.id].standing, at);
    // it stands where it stopped, in no kerb space: a place of its own, and the space it came from is empty
    for (uint32_t& occupant : spaceCar)
        if (occupant == car.id) occupant = CarRef::kNone;
    cars[car.id].standing = addDoor(at);
    cars[car.id].heldBy = kNoAgent;
    if (who < carOfPerson.size()) carOfPerson[who] = CarRef::kNone;
    noteBody(who, at);
}

void FakeWorld::parkCar(CarRef car, AgentId who, PlaceRef space) {
    if (car.id >= cars.size()) return;
    cars[car.id].standing = space;
    cars[car.id].heldBy = kNoAgent;
    if (who < carOfPerson.size()) carOfPerson[who] = CarRef::kNone;
    Vec2 at;
    if (positionOf(space, at)) noteBody(who, at);
}

bool FakeWorld::spaceNear(PlaceRef door, CarRef car, PlaceRef& out) const {
    Vec2 at;
    if (!positionOf(door, at)) return false;
    bool found = false;
    Real best = 0;
    for (std::size_t i = 0; i < places.size(); ++i) {
        if (spaceCar[i] == kNotSpace || (spaceCar[i] != CarRef::kNone && spaceCar[i] != car.id)) continue;
        const Real d = (places[i] - at).lengthSquared();
        if (!found || d < best) {   // (a tie keeps the earlier space)
            found = true;
            best = d;
            out.id = static_cast<uint32_t>(i);
        }
    }
    return found;
}

bool FakeWorld::takeSpace(PlaceRef space, CarRef car) {
    if (space.id >= spaceCar.size() || spaceCar[space.id] == kNotSpace) return false;
    if (spaceCar[space.id] != CarRef::kNone && spaceCar[space.id] != car.id) return false;
    for (uint32_t& occupant : spaceCar)
        if (occupant == car.id) occupant = CarRef::kNone;
    spaceCar[space.id] = car.id;
    return true;
}

bool FakeWorld::nextSpace(PlaceRef door, CarRef, AgentId, PlaceRef taken, PlaceRef& out) const {
    Vec2 at;
    if (!offersNextSpace || !positionOf(door, at)) return false;
    bool found = false;
    Real best = 0;
    for (std::size_t i = 0; i < places.size(); ++i) {
        if (spaceCar[i] != CarRef::kNone || i == taken.id) continue;   // (kNotSpace, or a car stands in it)
        const Real d = (places[i] - at).lengthSquared();
        if (!found || d < best) {
            found = true;
            best = d;
            out.id = static_cast<uint32_t>(i);
        }
    }
    return found;
}

// ---- walking -------------------------------------------------------------------------------------------------------------
FakeWalk::FakeWalk(FakeWorld& world, const TravelCatalog& travel, EventId arrived) : world_(world), arrived_(arrived) {
    for (const TravelMode& m : travel.modes) speeds_.push_back(m.speed);
}

void FakeWalk::setAgents(std::size_t count) { walks.resize(count); }

bool FakeWalk::walk(AgentId who, WayId way, Vec2 from, PlaceRef to, Real fractionDone) {
    Vec2 end;
    if (who >= walks.size() || way >= speeds_.size() || !(speeds_[way] > 0) || !world_.positionOf(to, end)) return false;
    Walk& w = walks[who];
    w.active = true;
    w.from = from;
    w.to = end;
    w.way = way;
    w.fraction = fractionDone;
    spanFrom(world_.clock, millisFor((end - from).length(), speeds_[way]), fractionDone, w.start, w.end);
    ++started;
    return true;
}

void FakeWalk::cancel(AgentId who) {
    if (who >= walks.size() || !walks[who].active) return;
    Walk& w = walks[who];
    world_.noteBody(who, between(w.from, w.to, w.start, w.end, world_.clock));
    w.active = false;
    ++cancelled;
}

bool FakeWalk::where(AgentId who, Vec2& out) const {
    if (who >= walks.size() || !walks[who].active) return false;
    const Walk& w = walks[who];
    out = between(w.from, w.to, w.start, w.end, world_.clock);
    return true;
}

void FakeWalk::tick(Machine& machine) {
    const std::size_t n = walks.size();
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t i = reversed ? n - 1 - k : k;
        Walk& w = walks[i];
        if (!w.active || w.end > world_.clock) continue;
        w.active = false;
        world_.noteBody(static_cast<AgentId>(i), w.to);
        machine.post(static_cast<AgentId>(i), arrived_);
    }
}

// ---- driving -------------------------------------------------------------------------------------------------------------
FakeDrive::FakeDrive(FakeWorld& world, EventId parked) : world_(world), parkedEvent_(parked) {}

FakeDrive::Trip& FakeDrive::tripOf(CarRef car) {
    if (car.id >= trips.size()) trips.resize(car.id + 1);
    return trips[car.id];
}

bool FakeDrive::pullOut(CarRef car, PlaceRef space) {
    Vec2 at;
    if (!world_.positionOf(space, at)) return false;
    Trip& t = tripOf(car);
    t.placed = true;
    t.touched = true;
    t.at = at;
    ++pulledOut;
    return true;
}

bool FakeDrive::beginTrip(CarRef car, AgentId person, PlaceRef fromSpace, PlaceRef toSpace, Real fractionDone) {
    if (noRoute) return false;
    Trip& t = tripOf(car);
    Vec2 from = t.at, to;
    // a car the drive has not been given yet, or one taken up part-way, is measured from the space it set out from
    if ((!t.placed || fractionDone > 0) && !world_.positionOf(fromSpace, from)) return false;
    if (!world_.positionOf(toSpace, to)) return false;
    t.placed = true;
    t.touched = true;
    t.person = person;
    t.from = from;
    t.to = to;
    t.fraction = fractionDone;
    ++begun;
    if (laneBusy) {
        t.at = from;
        t.waiting = true;
        return true;
    }
    t.onTrip = true;
    const Real speed = engine::classSpeed(engine::RoadClass::Local);
    spanFrom(world_.clock, millisFor((to - from).length(), speed), fractionDone, t.start, t.end);
    return true;
}

void FakeDrive::endTrip(CarRef car) {
    if (car.id >= trips.size()) return;
    Trip& t = trips[car.id];
    if (t.waiting) {
        t.waiting = false;
        ++ended;
        return;
    }
    if (!t.onTrip) return;
    t.at = between(t.from, t.to, t.start, t.end, world_.clock);
    t.onTrip = false;
    ++ended;
}

bool FakeDrive::changeDestination(CarRef car, PlaceRef toSpace) {
    Vec2 to;
    if (noRoute || car.id >= trips.size() || !world_.positionOf(toSpace, to)) return false;
    Trip& t = trips[car.id];
    if (t.waiting) {
        t.to = to;
        ++redirected;
        return true;
    }
    if (!t.onTrip) return false;
    t.from = between(t.from, t.to, t.start, t.end, world_.clock);
    t.to = to;
    t.fraction = 0;
    t.start = world_.clock;
    t.end = t.start + millisFor((t.to - t.from).length(), engine::classSpeed(engine::RoadClass::Local));
    ++redirected;
    return true;
}

bool FakeDrive::waitingToSetOff(CarRef car) const { return car.id < trips.size() && trips[car.id].waiting; }

void FakeDrive::park(CarRef car, PlaceRef space) {
    Trip& t = tripOf(car);
    world_.positionOf(space, t.at);
    t.placed = false;
    t.onTrip = false;
    t.touched = true;
    ++parked;
}

bool FakeDrive::where(CarRef car, Vec2& out) const {
    if (car.id >= trips.size() || !trips[car.id].touched) return false;
    const Trip& t = trips[car.id];
    out = t.onTrip ? between(t.from, t.to, t.start, t.end, world_.clock) : t.at;
    return true;
}

void FakeDrive::tick(Machine& machine) {
    const std::size_t n = trips.size();
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t i = reversed ? n - 1 - k : k;
        Trip& t = trips[i];
        if (t.waiting && !laneBusy) {
            // the lane has cleared: it sets off now, and the drive takes its whole time from here
            t.waiting = false;
            t.onTrip = true;
            spanFrom(world_.clock, millisFor((t.to - t.from).length(), engine::classSpeed(engine::RoadClass::Local)), t.fraction,
                     t.start, t.end);
            continue;
        }
        if (!t.onTrip || t.end > world_.clock) continue;
        t.onTrip = false;
        t.at = t.to;
        machine.post(t.person, parkedEvent_);
    }
}

// ---- the lab -----------------------------------------------------------------------------------------------------------
engine::NavGraph longStreet() {
    engine::RoadGraph g;
    for (int i = 0; i <= 20; ++i) g.nodes.push_back({Vec2(100.0 * i, 0)});
    for (int i = 0; i < 20; ++i) g.edges.push_back(engine::RoadEdge{i, i + 1, 8, engine::RoadClass::Local, 0});
    return engine::buildNavGraph(g);
}

TravelCatalog testTravel() {
    TravelCatalog c;
    c.traits = {"has_board", "has_scooter"};
    TravelMode walk;
    walk.name = "walk";
    walk.speed = 5.0 / 3.6;
    walk.beginsIn = "walking";
    TravelMode car;
    car.name = "car";
    car.roads = true;
    car.speed = 0;
    car.startMinutes = 1.0;
    car.endMinutes = 1.5;
    car.setOffWaitMinutes = 2.0;
    car.ownVehicle = true;
    car.beginsIn = "pulling_out";
    TravelMode board;
    board.name = "skateboard";
    board.speed = 10.0 / 3.6;
    board.needs = "has_board";
    board.needsTrait = 0;
    board.longestLegMinutes = 6.0;
    board.weight = 1.2;
    board.beginsIn = "on_small_wheels";
    TravelMode scooter = board;
    scooter.name = "scooter";
    scooter.speed = 12.0 / 3.6;
    scooter.needs = "has_scooter";
    scooter.needsTrait = 1;
    c.modes = {walk, car, board, scooter};
    RoadKind streets;
    streets.name = "streets";
    streets.classes = {engine::RoadClass::Arterial, engine::RoadClass::Collector, engine::RoadClass::Local};
    c.roadKinds = {streets};
    return c;
}

bool shippedBrainSpec(BrainSpec& out) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/brain.lua");
    std::stringstream text;
    text << in.rdbuf();
    engine::ScriptVM vm;
    std::string err;
    if (!vm.doString(text.str(), &err) || !loadBrainSpec(vm, out, &err)) {
        std::printf("    brain.lua did not read: %s\n", err.c_str());
        return false;
    }
    return true;
}

Lab::Lab(bool skills, const BrainSpec* given, const TravelCatalog* ways) {
    nav = longStreet();
    travel = ways ? *ways : testTravel();
    registerBehaviours(behaviours);
    BrainSpec spec;
    if (given) spec = *given;
    ok = (given || shippedBrainSpec(spec)) && compileBrainTable(spec, behaviours, table, &error) &&
         bindWays(travel, table, &error);
    if (!ok) {
        std::printf("    the lab's table did not compile: %s\n", error.c_str());
        return;
    }
    walk.reset(new FakeWalk(world, travel, static_cast<EventId>(table.event("arrived"))));
    drive.reset(new FakeDrive(world, static_cast<EventId>(table.event("parked"))));
    if (skills) {
        world.walk = walk.get();
        world.drive = drive.get();
    }
    planner.reset(new TripPlanner(nav, travel, world));
    machine.reset(new Machine(table, world, *planner, skills ? walk.get() : nullptr, skills ? drive.get() : nullptr));
    machine->traceTo(&trace);
}

Lab::~Lab() = default;

AgentId Lab::person(PlaceRef home, uint8_t traits, CarRef car, Detail detail) {
    const AgentId who = machine->addAgent(home, 0, traits, car, detail);
    world.setAgents(machine->agentCount());
    walk->setAgents(machine->agentCount());
    return who;
}

void Lab::tick() {
    if (world.walk) walk->tick(*machine);
    if (world.drive) drive->tick(*machine);
    machine->tick();
}

void Lab::runTo(BrainTime t) {
    while (world.clock < t) {
        world.clock += kTick;
        tick();
    }
}

std::string Lab::traceOf(AgentId who) const {
    std::string s;
    for (const Transition& t : trace)
        if (t.agent == who) s += (s.empty() ? "" : " ") + table.states[t.state].name + "@" + std::to_string(t.at);
    return s;
}

std::string Lab::planOf(AgentId who) const {
    const AgentBrain& a = machine->agent(who);
    std::string s;
    for (int i = 0; i < a.legCount; ++i)
        s += (i ? " " : "") + table.ways[a.legWay(i)].name + ">" + std::to_string(a.legEnd[i]);
    return s;
}

}  // namespace brain_test
