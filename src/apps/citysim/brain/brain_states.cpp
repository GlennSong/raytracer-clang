#include "brain_states.h"

#include "brain.h"

#include "../../../engine/procgen/city/roads/lane_graph.h"   // engine::PathPoint (World::carOnLane)

namespace citysim {
namespace brain {

namespace {

// ---- shared by the states of a leg ---------------------------------------------------------------------------------------
BrainTime legMillis(const AgentBrain& a) {
    return a.travelling() ? static_cast<BrainTime>(a.legSeconds[a.legNow]) * kMillisPerSecond : kNever;
}
const WayRow* legWayRow(const StepContext& ctx, const AgentBrain& a) {
    if (!a.travelling()) return nullptr;
    const WayId way = a.legWay(a.legNow);
    return way < ctx.table->ways.size() ? &ctx.table->ways[way] : nullptr;
}
PlaceRef legEndPlace(const AgentBrain& a) {
    PlaceRef p;
    if (a.travelling()) p.id = a.legEnd[a.legNow];
    return p;
}
CarRef carOf(const AgentBrain& a) {
    CarRef c;
    c.id = a.car;
    return c;
}
EventId raised(const StepContext& ctx, const AgentBrain& a, int which) {
    return ctx.table->states[a.state].raises[which];
}

// ---- WALKING, and small wheels (the walk skill is told the way, so it moves at the way's speed) ----------------------------
// raises: 0 blocked
BrainTime walkLasts(const StepContext&, const AgentBrain& a) { return legMillis(a); }
EventId walkFrom(StepContext& ctx, AgentBrain& a, Real fractionDone) {
    if (!ctx.skillsRun) return kNoId;   // far: nobody walks; the clock stands for the walk
    Vec2 from;
    if (!ctx.walk || !ctx.machine->legStart(a, from)) return raised(ctx, a, 0);
    return ctx.walk->walk(ctx.agent, a.legWay(a.legNow), from, legEndPlace(a), fractionDone) ? kNoId : raised(ctx, a, 0);
}
EventId walkEnter(StepContext& ctx, AgentBrain& a) { return walkFrom(ctx, a, 0); }
void walkExit(StepContext& ctx, AgentBrain&, ExitWhy why) {
    if (ctx.skillsRun && ctx.walk && why != ExitWhy::Done) ctx.walk->cancel(ctx.agent);
}

// ---- PULLING OUT: the person is at the household's car. If it is still there and free they take it, and the way's
// start minutes pass (unlocking, pulling out of the space). raises: 0 vehicle_gone, 1 no_route
BrainTime pullOutLasts(const StepContext& ctx, const AgentBrain& a) {
    const WayRow* way = legWayRow(ctx, a);
    return way ? way->startMillis : kNever;
}
EventId pullOutFrom(StepContext& ctx, AgentBrain& a) {
    const CarRef car = carOf(a);
    PlaceRef standing;
    if (!car.valid() || !ctx.world->carStanding(car, standing)) return raised(ctx, a, 0);
    // the plan walked them to where the car stood when it was made: if the car has since been moved, it is gone
    const PlaceRef planned = ctx.machine->legStartPlace(a);
    if (planned.valid() && planned != standing) return raised(ctx, a, 0);
    if (!ctx.world->claimCar(car, ctx.agent)) return raised(ctx, a, 0);
    if (ctx.skillsRun && ctx.drive && !ctx.drive->pullOut(car, standing)) {
        ctx.world->releaseCar(car, ctx.agent);
        return raised(ctx, a, 1);
    }
    return kNoId;
}
EventId pullOutEnter(StepContext& ctx, AgentBrain& a) { return pullOutFrom(ctx, a); }
EventId pullOutMidLeg(StepContext& ctx, AgentBrain& a, Real) { return pullOutFrom(ctx, a); }
void pullOutExit(StepContext& ctx, AgentBrain& a, ExitWhy why) {
    if (why == ExitWhy::Done) return;
    const CarRef car = carOf(a);
    PlaceRef standing;
    engine::PathPoint lane;
    // it never left its space: back off the lane it goes, and, unless the person is only no longer watched, it is
    // given up. (One an earlier trip left in a lane stays there, on the road: taken off it there, it stood in the lane
    // where nothing drove round it.)
    if (ctx.skillsRun && ctx.drive && ctx.world->carStanding(car, standing) && !ctx.world->carOnLane(car, lane))
        ctx.drive->park(car, standing);
    if (why != ExitWhy::Lowered) ctx.world->releaseCar(car, ctx.agent);
}

// ---- DRIVING: from the kerb lane beside the car's space to the kerb lane beside the space near the door. Its planned
// time is the leg's less the way's start and end minutes. raises: 0 no_route, 1 vehicle_gone, 2 blocked
//
// SETTING OFF MAY HAVE TO WAIT (ADR-0442): the skill keeps the car in its space until the lane beside it is clear. Once
// a tick the state asks whether it is still waiting, and after the way's limit (travel.lua `set_off_wait_minutes`)
// raises `blocked`, which the table answers (the failed-leg rule: plan again without this way).
//
// SENT SOMEWHERE ELSE (the verb `redirect`): the leg now ends at another space, and the skill is told.
BrainTime driveLasts(const StepContext& ctx, const AgentBrain& a) {
    const WayRow* way = legWayRow(ctx, a);
    const BrainTime leg = legMillis(a);
    if (!way || leg == kNever) return kNever;
    const BrainTime fixed = way->startMillis + way->endMillis;
    return leg > fixed ? leg - fixed : 0;
}
EventId driveFrom(StepContext& ctx, AgentBrain& a, Real fractionDone) {
    if (!ctx.skillsRun) {
        // far: nobody drives; the clock stands for the drive, and the car is away with it (ADR-0516)
        ctx.world->carDrivenByClock(carOf(a), ctx.agent);
        return kNoId;
    }
    if (!ctx.drive) return raised(ctx, a, 0);
    const CarRef car = carOf(a);
    PlaceRef from = ctx.machine->legStartPlace(a);
    if (!from.valid()) ctx.world->carStanding(car, from);
    return ctx.drive->beginTrip(car, ctx.agent, from, legEndPlace(a), fractionDone) ? kNoId : raised(ctx, a, 0);
}
EventId driveEnter(StepContext& ctx, AgentBrain& a) { return driveFrom(ctx, a, 0); }
EventId driveMidLeg(StepContext& ctx, AgentBrain& a, Real fractionDone) {
    if (!ctx.world->claimCar(carOf(a), ctx.agent)) return raised(ctx, a, 1);
    return driveFrom(ctx, a, fractionDone);
}
EventId driveRetarget(StepContext& ctx, AgentBrain& a) {
    if (!ctx.skillsRun) return kNoId;   // far: the clock stands for the drive, to wherever it now ends
    if (!ctx.drive) return raised(ctx, a, 0);
    return ctx.drive->changeDestination(carOf(a), legEndPlace(a)) ? kNoId : raised(ctx, a, 0);
}
void driveStep(StepContext& ctx, AgentBrain& a) {
    const WayRow* way = legWayRow(ctx, a);
    if (!ctx.drive || !way || way->setOffWaitMillis == 0) return;
    if (ctx.now - a.stateEntered < way->setOffWaitMillis) return;
    if (ctx.drive->waitingToSetOff(carOf(a))) ctx.machine->post(ctx.agent, raised(ctx, a, 2));
}
void driveExit(StepContext& ctx, AgentBrain& a, ExitWhy why) {
    if (why == ExitWhy::Done) return;
    if (why == ExitWhy::Lowered) {
        // no longer watched (ADR-0517): the clock drives the rest of the leg, and the car is away with it -- off the
        // road (its trip ended there), or out of its space if it had not set off yet
        ctx.world->carDrivenByClock(carOf(a), ctx.agent);
        return;
    }
    if (ctx.skillsRun && ctx.drive) ctx.drive->endTrip(carOf(a));
    // the plan is dropped or the person interrupted: they get out where the car is and leave it there
    ctx.world->leaveCar(carOf(a), ctx.agent);
}

// ---- PARKING: the car is in the kerb lane beside its space. The space is taken if it is still free, and the way's end
// minutes pass (into the space, out of the car, locking up). raises: 0 cannot_park, 1 space_taken
//
// THE SPACE IS TAKEN (ADR-0441): the world is asked for the next nearest free space to the door the walk after this
// leg goes to. When it names one, the leg now ends there, it is held for this person, and `space_taken` is raised: the
// table sends the person back to driving, to that space. When it names none, `cannot_park`, as before. The state's own
// data says the car took no space, so leaving the state does not park it.
constexpr uint32_t kTookNoSpace = 1;
BrainTime parkLasts(const StepContext& ctx, const AgentBrain& a) {
    const WayRow* way = legWayRow(ctx, a);
    return way ? way->endMillis : kNever;
}
EventId parkEnter(StepContext& ctx, AgentBrain& a) {
    const PlaceRef space = legEndPlace(a);
    if (ctx.world->takeSpace(space, carOf(a))) return kNoId;
    a.payload.word = kTookNoSpace;
    PlaceRef door, next;
    door.id = a.legCount > 0 ? a.legEnd[a.legCount - 1] : a.intentTarget;
    if (!ctx.world->nextSpace(door, carOf(a), ctx.agent, space, next) || !next.valid() || next == space)
        return raised(ctx, a, 0);
    a.legEnd[a.legNow] = next.id;
    ctx.world->holdSpace(next, ctx.agent);
    return raised(ctx, a, 1);
}
EventId parkMidLeg(StepContext& ctx, AgentBrain& a, Real) {
    // raised from far part-way through parking: the car stands in the kerb lane beside its space
    if (ctx.skillsRun && ctx.drive) ctx.drive->pullOut(carOf(a), legEndPlace(a));
    return kNoId;
}
void parkExit(StepContext& ctx, AgentBrain& a, ExitWhy why) {
    if (why == ExitWhy::Lowered) return;
    if (why == ExitWhy::Done) {
        if (a.payload.word == kTookNoSpace) return;   // on to another space: the car stays in the lane, still theirs
        if (ctx.skillsRun && ctx.drive) ctx.drive->park(carOf(a), legEndPlace(a));
        ctx.world->parkCar(carOf(a), ctx.agent, legEndPlace(a));
        return;
    }
    ctx.world->leaveCar(carOf(a), ctx.agent);
}

// ---- WAITING FOR A BUS OR A TRAIN (ADR-0484; step 4f): at the stop the leg before ended at, for one of the leg's line
// (its handle). Its planned time is the line's mean wait at that stop (World::rideWait: half the headway, and a
// station's way in) and the way's start minutes. A near person is the ride skill's: it reports `boarded`.
// raises: 0 missed_ride
BrainTime rideWaitOf(const StepContext& ctx, const AgentBrain& a) {
    return ctx.world && ctx.machine ? ctx.world->rideWait(a.legHandle(a.legNow), ctx.machine->legStartPlace(a)) : 0;
}
BrainTime busWaitLasts(const StepContext& ctx, const AgentBrain& a) {
    const WayRow* way = legWayRow(ctx, a);
    return way ? way->startMillis + rideWaitOf(ctx, a) : kNever;
}
EventId busWaitFrom(StepContext& ctx, AgentBrain& a, Real) {
    if (!ctx.skillsRun) return kNoId;   // far: nobody waits at a real stop; the clock stands for the wait
    if (!ctx.ride) return raised(ctx, a, 0);
    const PlaceRef board = ctx.machine->legStartPlace(a);
    return ctx.ride->waitFor(ctx.agent, a.legHandle(a.legNow), board, legEndPlace(a)) ? kNoId : raised(ctx, a, 0);
}
EventId busWaitEnter(StepContext& ctx, AgentBrain& a) { return busWaitFrom(ctx, a, 0); }
void busWaitExit(StepContext& ctx, AgentBrain&, ExitWhy why) {
    // (boarded: the skill carries on with them aboard, into riding)
    if (ctx.skillsRun && ctx.ride && why != ExitWhy::Done) ctx.ride->cancel(ctx.agent);
}

// ---- RIDING A BUS OR A TRAIN: aboard, to the leg's last stop. Its planned time is the leg's less the wait. A near person who
// boarded is already the skill's; one raised from far part-way through is put aboard (`aboard`). The skill reports
// `alighted`. raises: 0 missed_ride
BrainTime busRideLasts(const StepContext& ctx, const AgentBrain& a) {
    const WayRow* way = legWayRow(ctx, a);
    const BrainTime leg = legMillis(a);
    if (!way || leg == kNever) return kNever;
    const BrainTime fixed = way->startMillis + way->endMillis + rideWaitOf(ctx, a);
    return leg > fixed ? leg - fixed : 0;
}
EventId busRideEnter(StepContext&, AgentBrain&) { return kNoId; }   // (boarded: the skill has them aboard already)
EventId busRideMidLeg(StepContext& ctx, AgentBrain& a, Real) {
    if (!ctx.skillsRun) return kNoId;
    if (!ctx.ride) return raised(ctx, a, 0);
    return ctx.ride->aboard(ctx.agent, a.legHandle(a.legNow), legEndPlace(a)) ? kNoId : raised(ctx, a, 0);
}
void busRideExit(StepContext& ctx, AgentBrain&, ExitWhy why) {
    if (ctx.skillsRun && ctx.ride && why != ExitWhy::Done) ctx.ride->cancel(ctx.agent);
}

// ---- RELEASED: the player or a director has let go. The intent takes over again at once. raises: 0 resume_intent
EventId releasedEnter(StepContext& ctx, AgentBrain& a) { return raised(ctx, a, 0); }

}  // namespace

void registerBehaviours(BehaviourSet& set) {
    set.addSkill("walk");
    set.addSkill("drive");
    set.addSkill("ride");

    // at a place, and held: the state itself does nothing; its row of the table says what ends it
    const StateFns waits{};
    set.add("indoors", waits);
    set.add("standing", waits);
    // on a seat or a track, in a group (ADR-0512): waiting, as at a place; the body is the sim's (CitySim::stepSpots)
    set.add("using_spot", waits);
    set.add("group_activity", waits);
    // wanting to be somewhere with no way there (ADR-0443): the data says how long they wait before trying again
    // (`seconds`) and what makes them try sooner
    set.add("no_way", waits);
    set.add("possessed", waits);
    set.add("knocked_down", waits);
    set.add("stuck", waits);
    set.add("crashed", waits);

    StateFns walking;
    walking.lasts = walkLasts;
    walking.enter = walkEnter;
    walking.enterMidLeg = walkFrom;
    walking.exit = walkExit;
    set.add("walking", walking, {"blocked"});
    set.add("on_small_wheels", walking, {"blocked"});

    StateFns pullingOut;
    pullingOut.lasts = pullOutLasts;
    pullingOut.enter = pullOutEnter;
    pullingOut.enterMidLeg = pullOutMidLeg;
    pullingOut.exit = pullOutExit;
    set.add("pulling_out", pullingOut, {"vehicle_gone", "no_route"});

    StateFns driving;
    driving.lasts = driveLasts;
    driving.enter = driveEnter;
    driving.enterMidLeg = driveMidLeg;
    driving.exit = driveExit;
    driving.retarget = driveRetarget;
    driving.step = driveStep;
    set.add("driving", driving, {"no_route", "vehicle_gone", "blocked"});

    StateFns parking;
    parking.lasts = parkLasts;
    parking.enter = parkEnter;
    parking.enterMidLeg = parkMidLeg;
    parking.exit = parkExit;
    set.add("parking", parking, {"cannot_park", "space_taken"});

    StateFns busWaiting;
    busWaiting.lasts = busWaitLasts;
    busWaiting.enter = busWaitEnter;
    busWaiting.enterMidLeg = busWaitFrom;
    busWaiting.exit = busWaitExit;
    set.add("waiting", busWaiting, {"missed_ride"});

    StateFns busRiding;
    busRiding.lasts = busRideLasts;
    busRiding.enter = busRideEnter;
    busRiding.enterMidLeg = busRideMidLeg;
    busRiding.exit = busRideExit;
    set.add("riding", busRiding, {"missed_ride"});

    StateFns released;
    released.enter = releasedEnter;
    set.add("released", released, {"resume_intent"});

    // in the data, not built yet
    for (const char* name : {"boarding", "alighting", "hailing", "working_post", "asleep"})
        set.addNotBuilt(name);
}

}  // namespace brain
}  // namespace citysim
