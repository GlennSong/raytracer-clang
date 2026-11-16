#ifndef RAYTRACER_TESTS_BRAIN_FAKES_H
#define RAYTRACER_TESTS_BRAIN_FAKES_H

// STAND-INS FOR THE WORLD AND THE SKILLS, for the agent brain's tests (ADR-0440). Each does what the real thing will,
// by the clock alone and with no chance in it: a walk takes its straight-line metres at the way's speed, a drive its
// straight-line metres at a local street's speed (the router's own, engine::classSpeed), and each reports to the
// machine on the first tick at or after the moment it ends. On the test street (one straight road with a node every
// 100 m, every place on a node) those times are the trip planner's own, so a near person (skills) and a far one (the
// clock) keep the same times.
//
// A LAB is the whole set-up: the street, the ways of travelling (written here as numbers, so every expected time can be
// worked out by hand), the shipped brain.lua compiled against the real behaviours, the stand-ins, the real trip planner
// and the machine.

#include "apps/citysim/brain/brain.h"
#include "apps/citysim/brain/brain_planner.h"
#include "apps/citysim/brain/brain_states.h"
#include "apps/citysim/trip_plan.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace brain_test {

using namespace citysim;
using namespace citysim::brain;

constexpr BrainTime kTick = 500;   // the test clock's step, milliseconds

class FakeWalk;
class FakeDrive;

class FakeWorld : public World {
public:
    BrainTime clock = 0;
    FakeWalk* walk = nullptr;
    FakeDrive* drive = nullptr;

    PlaceRef addDoor(Vec2 at);
    PlaceRef addSpace(Vec2 at);
    CarRef addCar(PlaceRef space);       // a car standing in a kerb space
    void setAgents(std::size_t count);   // room for this many people's bodies

    BrainTime now() const override { return clock; }
    bool positionOf(PlaceRef place, Vec2& out) const override;
    bool bodyPosition(AgentId who, Vec2& out) const override;
    bool carStanding(CarRef car, PlaceRef& out) const override;
    bool carFreeFor(CarRef car, AgentId who) const override;
    bool claimCar(CarRef car, AgentId who) override;
    void releaseCar(CarRef car, AgentId who) override;
    void leaveCar(CarRef car, AgentId who) override;
    void parkCar(CarRef car, AgentId who, PlaceRef space) override;
    bool spaceNear(PlaceRef door, CarRef car, PlaceRef& out) const override;
    bool takeSpace(PlaceRef space, CarRef car) override;

    struct Car {
        PlaceRef standing;
        AgentId heldBy = kNoAgent;
    };
    std::vector<Vec2> places;            // by PlaceRef id
    std::vector<uint32_t> spaceCar;      // by PlaceRef id: the car in it; kNotSpace for a place that is no kerb space
    std::vector<Car> cars;
    std::vector<Vec2> bodyAt;            // by person: where they last stopped
    std::vector<uint8_t> bodyKnown;
    std::vector<uint32_t> carOfPerson;   // by person: the car they have taken (CarRef::kNone: none)
    static constexpr uint32_t kNotSpace = 0xFFFFFFFEu;
    void noteBody(AgentId who, Vec2 at);
};

class FakeWalk : public WalkSkill {
public:
    FakeWalk(FakeWorld& world, const TravelCatalog& travel, EventId arrived);
    void setAgents(std::size_t count);
    bool walk(AgentId who, WayId way, Vec2 from, PlaceRef to, Real fractionDone) override;
    void cancel(AgentId who) override;
    // Reports every walk that has ended, lowest person first (highest first with `reversed`, to show the machine's
    // answers do not depend on the order reports are posted in).
    void tick(Machine& machine);
    bool where(AgentId who, Vec2& out) const;   // false: not walking

    struct Walk {
        bool active = false;
        Vec2 from, to;
        BrainTime start = 0, end = 0;
        WayId way = kNoId;
        Real fraction = 0;   // what the last walk() was told was already done
    };
    std::vector<Walk> walks;
    bool reversed = false;
    int started = 0, cancelled = 0;

private:
    FakeWorld& world_;
    std::vector<Real> speeds_;   // m/s by way
    EventId arrived_;
};

class FakeDrive : public DriveSkill {
public:
    FakeDrive(FakeWorld& world, EventId parked);
    bool pullOut(CarRef car, PlaceRef space) override;
    bool beginTrip(CarRef car, AgentId person, PlaceRef fromSpace, PlaceRef toSpace, Real fractionDone) override;
    void endTrip(CarRef car) override;
    void park(CarRef car, PlaceRef space) override;
    void tick(Machine& machine);
    bool where(CarRef car, Vec2& out) const;    // false: the drive has not touched this car

    struct Trip {
        bool placed = false, onTrip = false, touched = false;
        Vec2 at, from, to;
        BrainTime start = 0, end = 0;
        AgentId person = kNoAgent;
        Real fraction = 0;   // what the last beginTrip() was told was already done
    };
    std::vector<Trip> trips;   // by car
    bool reversed = false;
    bool noRoute = false;      // every beginTrip is refused
    int pulledOut = 0, begun = 0, ended = 0, parked = 0;

private:
    Trip& tripOf(CarRef car);
    FakeWorld& world_;
    EventId parkedEvent_;
};

// The test street: along x from 0 to 2000 m, a node every 100 m, walkable and drivable.
engine::NavGraph longStreet();
// The ways of travelling, as numbers: walk 5 km/h (100 m in 72 s); car at the router's speeds (a local street 8 m/s),
// 1 minute to pull out, 1.5 to park; skateboard 10 km/h (100 m in 36 s), for someone with a board, a leg of at most 6
// minutes (1000 m), a minute of it minded as 1.2; scooter the same at 12 km/h, at most 6 minutes. Each begins in the
// state travel.lua says.
TravelCatalog testTravel();
constexpr uint8_t kHasBoard = 1, kHasScooter = 2;   // the traits' bits, in testTravel's order
constexpr WayId kWalk = 0, kCar = 1, kSkateboard = 2, kScooter = 3;

// The shipped brain.lua, read. False (with the reason printed) when it does not read.
bool shippedBrainSpec(BrainSpec& out);

struct Lab {
    engine::NavGraph nav;
    TravelCatalog travel;
    BehaviourSet behaviours;
    BrainTable table;
    FakeWorld world;
    std::unique_ptr<FakeWalk> walk;
    std::unique_ptr<FakeDrive> drive;
    std::unique_ptr<TripPlanner> planner;
    std::unique_ptr<Machine> machine;
    std::vector<Transition> trace;
    bool ok = false;         // the data read and compiled
    std::string error;

    // `skills` false: a machine with no skills at all (every person must stay far). `spec` and `ways`, when given,
    // stand in for the shipped brain.lua and for testTravel() (a test that edits one row).
    explicit Lab(bool skills = true, const BrainSpec* spec = nullptr, const TravelCatalog* ways = nullptr);
    ~Lab();
    Lab(const Lab&) = delete;
    Lab& operator=(const Lab&) = delete;

    PlaceRef door(Real x) { return world.addDoor(Vec2(x, 0)); }
    PlaceRef space(Real x) { return world.addSpace(Vec2(x, 0)); }
    AgentId person(PlaceRef home, uint8_t traits, CarRef car, Detail detail);
    // The skills report, then the machine ticks, at the clock's time.
    void tick();
    // The clock stepped by kTick, with a tick at each step, until it reads `t`.
    void runTo(BrainTime t);
    StateId state(const char* name) const { return static_cast<StateId>(table.state(name)); }
    const std::string& stateName(AgentId who) const { return table.states[machine->agent(who).state].name; }
    // The trace as text: "walking@0 pulling_out@72000 ..." for one person.
    std::string traceOf(AgentId who) const;
    // A person's plan as text: "walk>3 car>5 walk>2".
    std::string planOf(AgentId who) const;
};

}  // namespace brain_test

#endif
