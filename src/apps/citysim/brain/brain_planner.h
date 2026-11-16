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

class TripPlanner : public Planner {
public:
    TripPlanner(const engine::NavGraph& nav, const TravelCatalog& travel, const World& world)
        : nav_(nav), travel_(travel), world_(world) {}

    bool plan(const PlanAsk& ask, PlanLegs& out) override;

    // The last plan's one-line reason ("walk 18.0 min, car 8.6 min (...): car"), and, when a plan was refused, why.
    const std::string& lastReason() const { return lastReason_; }
    const std::string& lastError() const { return lastError_; }

private:
    const engine::NavGraph& nav_;
    const TravelCatalog& travel_;
    const World& world_;
    std::string lastReason_, lastError_;
};

}  // namespace brain
}  // namespace citysim

#endif
