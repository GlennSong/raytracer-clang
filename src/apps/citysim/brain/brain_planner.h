#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_PLANNER_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_PLANNER_H

// THE PLAN LAYER (docs/agent-brain-design.md section 4; ADR-0440): the brain's Planner, answered by THE trip planner
// (../trip_plan.h, ADR-0415) -- the same pricing the one-agent lab uses, not a copy. It asks the world where the
// household's car stands and whether it is free at this moment, and which kerb parking space near the door the car
// would be left in; hands the planner the person's traits and the ways ruled out; and turns the cheapest option into
// the record's fixed legs, each ending at a place the world named.
//
// It allocates (the planner's routes are vectors). A person plans when their intent changes or a leg fails, a few times
// a day, never per tick.

#include "brain_skills.h"

#include "../trip_plan.h"

#include <string>

namespace citysim {
namespace brain {

// WHAT THE PLANS PUT ON THE ROADS, counted over every plan made: the legs by a way on the roads in every option priced
// (whether or not it was chosen), and in the plans chosen; of each, those with metres on the freeway kind of road
// (travel.lua road_kinds "freeway"), and those metres.
struct PlanRoadCounts {
    uint64_t pricedLegs = 0, pricedOnFreeway = 0;
    uint64_t chosenLegs = 0, chosenOnFreeway = 0;
    double pricedFreewayMetres = 0, chosenFreewayMetres = 0;
};

class TripPlanner : public Planner {
public:
    TripPlanner(const TripNetwork& net, const TravelCatalog& travel, const World& world)
        : nav_(net), travel_(travel), world_(world) {}

    bool plan(const PlanAsk& ask, PlanLegs& out) override;

    // The last plan's one-line reason ("walk 18.0 min, car 8.6 min (...): car"), and, when a plan was refused, why.
    const std::string& lastReason() const { return lastReason_; }
    const std::string& lastError() const { return lastError_; }
    const PlanRoadCounts& roadCounts() const { return roadCounts_; }

private:
    TripNetwork nav_;   // the road model and the walker every leg is routed on (trip_plan.h)
    const TravelCatalog& travel_;
    const World& world_;
    std::string lastReason_, lastError_;
    PlanRoadCounts roadCounts_;
};

}  // namespace brain
}  // namespace citysim

#endif
