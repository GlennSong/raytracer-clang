#ifndef RAYTRACER_APPS_DRIVING_ROAD_RULES_H
#define RAYTRACER_APPS_DRIVING_ROAD_RULES_H

// BINDING THE ROAD TO THE RULES, once at load (driving-model-design.md 6.4; issue #251).
//
// The lane graph names the kinds of regulation standing on its paths in its own table (regulationKindNames); the rule
// set says what each device means, by name. RoadRules is the join of the two: for each regulation kind in the graph,
// the rule set's device of the same name. Facts read it to learn what a regulation means without looking up a name
// while cars drive.
//
// Building it is also where the facts are checked against the rule set: every number and personal setting a
// registered fact says it needs must be there, in the kind of unit it asks for (a speed is not a time); and so must
// those of every other reader the registry holds (the move step, once registerMoveNeeds has added it). Every problem is
// collected and reported together, one plain line each.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/notice.h"
#include "apps/driving/rule_set.h"

#include <cstdint>
#include <string>
#include <vector>

namespace driving {

// Not bound: a regulation kind with no device (only in a RoadRules that was refused).
constexpr uint32_t kNoDevice = 0xFFFFFFFFu;

struct RoadRules {
    // One per entry of the graph's regulationKindNames: the index of its device in RuleSet::devices, or kNoDevice.
    std::vector<uint32_t> deviceOfKind;

    // What a regulation of kind `kind` means: its device in `rules` (the rule set this was built from), or null for a
    // kind the graph does not have or that is not bound.
    const Device* device(const RuleSet& rules, engine::NameIndex kind) const;
};

// BUILDS THE BINDING and checks the registered facts' needs. True when there is no problem. Otherwise false, `out`
// holds what could be bound, and `problems` (when given) gets every problem, in this order: regulation kinds in the
// graph's table order, then facts in slot order, then the other readers in the order added, each one's numbers then
// its settings in the order it declares them.
bool buildRoadRules(const engine::LaneGraph& roads, const RuleSet& rules, const FactRegistry& facts, RoadRules& out,
                    std::vector<std::string>* problems = nullptr);

}  // namespace driving

#endif
