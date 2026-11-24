#include "brain_planner.h"

namespace citysim {
namespace brain {

static_assert(kMaxLegs == kMaxPlanLegs, "the brain's record and the trip planner hold the same number of legs");

bool TripPlanner::plan(const PlanAsk& ask, PlanLegs& out) {
    lastReason_.clear();
    lastError_.clear();
    TripRequest req;
    req.from = ask.from;
    if (!world_.positionOf(ask.to, req.to)) {
        lastError_ = "the world does not know place " + std::to_string(ask.to.id);
        return false;
    }
    req.traits = ask.traits;
    for (int way = 0; way < static_cast<int>(travel_.modes.size()); ++way)
        if ((ask.ruledOut >> way) & 1u) req.ruledOut.push_back(way);
    // THE HOUSEHOLD'S CAR: where it stands now, whether it is free now, and the space near the door it would be left
    // in. It serves every way that needs a vehicle of one's own (today the one such way; a second kind of owned thing,
    // a bike, needs the record to say which handle is which).
    PlaceRef carAt, leaveAt;
    Vec2 carPoint, leavePoint;
    // AT THE WHEEL (ADR-0442) the car is where the person is, on the road, and it is theirs: the plan is made in it.
    if (ask.atTheWheel) carPoint = ask.from;
    const bool carKnown = ask.car.valid() && ask.car.id <= kMaxHandle &&
                          (ask.atTheWheel || (world_.carStanding(ask.car, carAt) && world_.positionOf(carAt, carPoint)));
    const bool haveCar = carKnown && world_.spaceNear(ask.to, ask.car, leaveAt) && world_.positionOf(leaveAt, leavePoint);
    if (ask.atTheWheel && !haveCar) {
        lastError_ = "at the wheel, and there is no free kerb space near place " + std::to_string(ask.to.id);
        return false;
    }
    if (haveCar) {
        const bool isFree = ask.atTheWheel || world_.carFreeFor(ask.car, ask.agent);
        for (std::size_t i = 0; i < travel_.modes.size(); ++i)
            if (travel_.modes[i].ownVehicle) {
                OwnedVehicle v{static_cast<int>(i), carPoint, leavePoint, isFree, ask.atTheWheel, world_.carClass(ask.car)};
                v.onLane = ask.atTheWheel && world_.carOnLane(ask.car, v.lane);
                req.vehicles.push_back(v);
            }
    }
    const TripPlan trip = planTrip(nav_, travel_, req);
    lastReason_ = trip.reason;
    for (std::size_t o = 0; o < trip.options.size(); ++o) {
        if (!trip.options[o].possible) continue;
        const bool chosen = static_cast<int>(o) == trip.chosen;
        for (const TripLeg& leg : trip.options[o].legs) {
            if (leg.mode < 0 || !travel_.modes[static_cast<std::size_t>(leg.mode)].roads) continue;
            const Real onFreeway = kindMetres(travel_, leg, "freeway");
            ++roadCounts_.pricedLegs;
            roadCounts_.pricedOnFreeway += onFreeway > 0 ? 1 : 0;
            roadCounts_.pricedFreewayMetres += onFreeway;
            if (!chosen) continue;
            ++roadCounts_.chosenLegs;
            roadCounts_.chosenOnFreeway += onFreeway > 0 ? 1 : 0;
            roadCounts_.chosenFreewayMetres += onFreeway;
        }
    }
    if (!trip.valid()) {
        lastError_ = "no way there (" + trip.reason + ")";
        return false;
    }
    FixedLegs fixed;
    if (!fixedLegs(travel_, *trip.choice(), fixed, &lastError_)) return false;
    PlanLegs legs;
    for (int i = 0; i < fixed.count; ++i) {
        const FixedLeg& leg = fixed.legs[i];
        const bool usesCar = travel_.modes[leg.way].ownVehicle;
        // what each leg ends at, as the place the world named
        PlaceRef end = ask.to;
        if (leg.end == LegEnd::Vehicle) end = carAt;
        else if (leg.end == LegEnd::VehicleLeft) end = leaveAt;
        else if (leg.end == LegEnd::Stop && !world_.stopPlace(leg.busLine, leg.busStop, end)) {
            lastError_ = "the plan names bus stop " + std::to_string(leg.busStop) + " of line " + std::to_string(leg.busLine) +
                         ", which the world does not know";
            return false;
        }
        legs.end[i] = end.id;
        // (a bus leg's handle is its line: the ride skill is told which bus to get on)
        const bool byBus = travel_.modes[leg.way].bus;
        legs.use[i] = packLegUse(leg.way, usesCar ? ask.car.id : byBus ? static_cast<uint32_t>(leg.busLine) : kNoHandle);
        legs.seconds[i] = leg.seconds;
    }
    legs.count = fixed.count;
    out = legs;
    return true;
}

}  // namespace brain
}  // namespace citysim
