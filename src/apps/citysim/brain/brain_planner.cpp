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
    const bool haveCar = ask.car.valid() && ask.car.id <= kMaxHandle && world_.carStanding(ask.car, carAt) &&
                         world_.positionOf(carAt, carPoint) && world_.spaceNear(ask.to, ask.car, leaveAt) &&
                         world_.positionOf(leaveAt, leavePoint);
    if (haveCar) {
        const bool isFree = world_.carFreeFor(ask.car, ask.agent);
        for (std::size_t i = 0; i < travel_.modes.size(); ++i)
            if (travel_.modes[i].ownVehicle)
                req.vehicles.push_back(OwnedVehicle{static_cast<int>(i), carPoint, leavePoint, isFree});
    }
    const TripPlan trip = planTrip(nav_, travel_, req);
    lastReason_ = trip.reason;
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
        legs.end[i] = end.id;
        legs.use[i] = packLegUse(leg.way, usesCar ? ask.car.id : kNoHandle);
        legs.seconds[i] = leg.seconds;
    }
    legs.count = fixed.count;
    out = legs;
    return true;
}

}  // namespace brain
}  // namespace citysim
