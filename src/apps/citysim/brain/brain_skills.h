#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_SKILLS_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_SKILLS_H

// WHAT THE BRAIN ASKS OF THE WORLD (docs/agent-brain-design.md section 6; ADR-0440). A SKILL is a system that moves a
// body: walking, driving. A state asks a skill to do something and waits for its report, which comes back as an event
// posted to the machine (brain.h Machine::post). The WORLD answers the questions states ask: where a place is, where the
// household's car stands and whether it is free, the nearest kerb parking space to a door, the time. The PLANNER turns
// "from here to that door" into legs.
//
// These are interfaces so the machine can be driven by stand-ins in a test and by the real systems in the sim. There is
// ONE object of each for the whole crowd: nothing here is made per person.

#include "brain_record.h"

#include "../../../engine/procgen/city/polygon.h"

namespace citysim {
namespace brain {

using engine::Real;
using engine::Vec2;

// WALKING, and the small wheels that are a faster way of doing a leg on foot (a board, a scooter: the way says which,
// so the host can show the prop and move at its speed).
class WalkSkill {
public:
    virtual ~WalkSkill() = default;
    // `who` goes from `from` to the place `to` by `way`, with `fractionDone` of the leg already behind them (0: from
    // the start; more: a person raised from far part-way along, or carrying on after being knocked down). Reports the
    // event `arrived` when there. False: it cannot be begun (nowhere to walk).
    virtual bool walk(AgentId who, WayId way, Vec2 from, PlaceRef to, Real fractionDone) = 0;
    // Stop where they are; no report follows. Harmless for someone not walking.
    virtual void cancel(AgentId who) = 0;
};

// DRIVING. Each call maps onto one call of the driving library's front door (apps/driving/driving.h), named beside it.
// The driving's reports come back as the events `parked`, `no_route`, `cannot_park`, `missed_turn`, `crashed` and
// `ended_early`. (`parked` is the driving's word for "stopped at the destination, in the kerb lane beside the space";
// getting into the space and out of the car is the brain's parking state.)
class DriveSkill {
public:
    virtual ~DriveSkill() = default;
    // placeCar: the car is put on the road in the kerb lane beside the space it stands in. False: it cannot be.
    virtual bool pullOut(CarRef car, PlaceRef space) = 0;
    // beginTrip: `person` drives `car` to the kerb lane beside `toSpace`. With `fractionDone` above 0 (a person raised
    // from far part-way through the drive from `fromSpace`) the car is first put that far along. False: no route.
    virtual bool beginTrip(CarRef car, AgentId person, PlaceRef fromSpace, PlaceRef toSpace, Real fractionDone) = 0;
    // endTrip: stop driving where the car is; no report follows. Harmless for a car on no trip.
    virtual void endTrip(CarRef car) = 0;
    // removeCar: the car leaves the lane for `space` and stands in it.
    virtual void park(CarRef car, PlaceRef space) = 0;
    // changeDestination (ADR-0442): the trip `car` is on (or is waiting to set off on) now ends in the kerb lane beside
    // `toSpace`, by the route from where the car is. False: it cannot be (no trip, or no route there: then the trip
    // keeps its old end, and no report follows).
    virtual bool changeDestination(CarRef car, PlaceRef toSpace) = 0;
    // IS THE CAR STILL WAITING TO SET OFF on the trip it was given: in its space, the lane beside it not yet clear?
    // The driving state asks it once a tick, to give the wait up after the way's limit (travel.lua
    // `set_off_wait_minutes`). A skill that puts a car in the lane at once answers no.
    virtual bool waitingToSetOff(CarRef car) const {
        (void)car;
        return false;
    }
};

// THE WORLD'S ANSWERS.
class World {
public:
    virtual ~World() = default;
    virtual BrainTime now() const = 0;
    // Where a place is, in plan. False: the world does not know it.
    virtual bool positionOf(PlaceRef place, Vec2& out) const = 0;
    // Where a person's body is, when one exists (a near person). False: none, and the brain works it out from the plan.
    virtual bool bodyPosition(AgentId who, Vec2& out) const = 0;

    // WHICH CAR `who` WOULD PLAN WITH, asked each time they plan (ADR-0443). `has` is the car their record names. A
    // world whose households share several cars answers the one of theirs that is free and nearest `from` (or `has`
    // when none is free); the default is `has`. The answer becomes the record's car for the plan that follows.
    virtual CarRef carOffered(AgentId who, CarRef has, Vec2 from) const {
        (void)who;
        (void)from;
        return has;
    }
    // THE HOUSEHOLD'S CAR. Where it stands now (false: no such car, or it is nowhere a person could take it from: the
    // player has it).
    virtual bool carStanding(CarRef car, PlaceRef& out) const = 0;
    // Whether `who` could take it this moment: nobody has it, or `who` already does.
    virtual bool carFreeFor(CarRef car, AgentId who) const = 0;
    // `who` takes it. False: someone else has it. Taking it twice is the same as once.
    virtual bool claimCar(CarRef car, AgentId who) = 0;
    // `who` gives it up without having moved it: it stands where it stood.
    virtual void releaseCar(CarRef car, AgentId who) = 0;
    // `who` gets out wherever the car is now and leaves it there, free.
    virtual void leaveCar(CarRef car, AgentId who) = 0;
    // `who` leaves it in `space`, free.
    virtual void parkCar(CarRef car, AgentId who, PlaceRef space) = 0;

    // TRIP ENDS. The nearest kerb parking space to a door that `car` could be left in this moment (false: none near).
    virtual bool spaceNear(PlaceRef door, CarRef car, PlaceRef& out) const = 0;
    // `car` takes the space. False: another car is in it.
    virtual bool takeSpace(PlaceRef space, CarRef car) = 0;

    // A SPACE HELD FOR A DRIVER (ADR-0441). `who`'s plan ends a drive in `space`: from now until they park there or
    // give the plan up, spaceNear tells nobody else that space. One person holds at most one space; holding another
    // lets the first go. A world that keeps no holds (the default) leaves two drivers free to be told one space.
    virtual void holdSpace(PlaceRef space, AgentId who) {
        (void)space;
        (void)who;
    }
    // `who` holds no space any more (their plan was dropped, or they have parked).
    virtual void dropHold(AgentId who) { (void)who; }
    // THE SPACE TO TRY NEXT: `car` stands in the lane beside `taken`, which another car is in. The next nearest space
    // to `door` that is free for `who` this moment. False (the default): none, and the drive ends in "cannot park".
    virtual bool nextSpace(PlaceRef door, CarRef car, AgentId who, PlaceRef taken, PlaceRef& out) const {
        (void)door;
        (void)car;
        (void)who;
        (void)taken;
        (void)out;
        return false;
    }
};

// A PLAN ASKED FOR: this person, standing here, wants to be at that place, with these traits and this car, and these
// ways ruled out.
struct PlanAsk {
    AgentId agent = kNoAgent;
    Vec2 from;
    PlaceRef to;
    uint8_t traits = 0;
    uint16_t ruledOut = 0;
    CarRef car;
    // AT THE WHEEL (the verb `redirect`, ADR-0442): the person is in `car` now, on the road at `from`. The plan is
    // made in it: its first leg is that car from here to where it will be left.
    bool atTheWheel = false;
};
// ...and the answer, in the record's own form.
struct PlanLegs {
    uint32_t end[kMaxLegs] = {};
    uint32_t use[kMaxLegs] = {};
    uint16_t seconds[kMaxLegs] = {};
    uint8_t count = 0;
};
class Planner {
public:
    virtual ~Planner() = default;
    // The cheapest way there that fits in kMaxLegs legs. False: there is no way.
    virtual bool plan(const PlanAsk& ask, PlanLegs& out) = 0;
};

}  // namespace brain
}  // namespace citysim

#endif
