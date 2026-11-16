#ifndef RAYTRACER_APPS_DRIVING_ROAD_RULES_H
#define RAYTRACER_APPS_DRIVING_ROAD_RULES_H

// BINDING THE ROAD TO THE RULES, once at load (driving-model-design.md 6.4; issue #251).
//
// The road model states what stands on the road by NAME: the regulations along each path (RoadModel::regulationsAlong:
// the signs a junction's arm states and the rules stated on a stretch of a street, ADR-0437), each junction's control
// (RoadModel::controlAt), each road's class (RoadModel::roadClassOf) and the limit posted on it
// (RoadModel::postedLimit). The rule set says what each name means. RoadRules is the join of the two, looked up once:
// for each path, its regulations in order along it, each with the rule set's device of its kind's name; for each
// junction, the rule set's row for its stated control; for each path, the limit in force on it (the posted one, or the
// rule set's default for its road's class). Facts read it to learn what stands along a path and what it means, and what
// limit holds, without looking up a name or asking the model while cars drive. It holds no road fact of its own: it is
// made from the model each time rules are given, and never saved -- a look-up, not a second home.
//
// Building it is also where the facts are checked against the rule set: every number and personal setting a
// registered fact says it needs must be there, in the kind of unit it asks for (a speed is not a time); and so must
// those of every other reader the registry holds (the move step, once registerMoveNeeds has added it). Every problem is
// collected and reported together, one plain line each.

#include "engine/procgen/city/roads/road_model.h"
#include "apps/driving/notice.h"
#include "apps/driving/rule_set.h"

#include <cstdint>
#include <string>
#include <vector>

namespace driving {

// Not bound: a regulation whose kind has no device, a junction whose control has no row (only in a RoadRules that was
// refused).
constexpr uint32_t kNoDevice = 0xFFFFFFFFu;

// ONE REGULATION ALONG A PATH, BOUND (the model's PathRegulation with its kind's name looked up): the rule set's device
// it is (an index into RuleSet::devices; kNoDevice: the rule set has none of that name), the stretch of the path where
// it holds (metres along it; equal: it stands at one place) and its value.
struct BoundRegulation {
    uint32_t device = kNoDevice;
    Real from = 0, to = 0;
    Real value = 0;
};
// A path's bound regulations, to walk in order along it.
struct BoundRegulations {
    const BoundRegulation* first = nullptr;
    const BoundRegulation* last = nullptr;
    const BoundRegulation* begin() const { return first; }
    const BoundRegulation* end() const { return last; }
};

struct RoadRules {
    // WHETHER TRAFFIC LIGHTS RUN A PROGRAMME (issue #261): when the registry the binding was checked against has the
    // light clock (signal_programme.h registerLightClock). When they do not, the lights are DARK: a junction whose control
    // has lights and names a `when_dark` row (rule_set.h ControlKind::whenDark) is bound to that row, as a dark or
    // flashing-red signal is; one naming none keeps its lights, fixed at their first phase.
    bool lightsRun = false;
    // THE REGULATIONS ALONG EACH PATH, as the road model answers them (RoadModel::regulationsAlong), each bound to its
    // device: path by path, each path's in the model's order along it. `regulationsOfPath[p]` is path p's stretch.
    std::vector<BoundRegulation> regulations;
    std::vector<engine::Range> regulationsOfPath;
    // THE LIMIT IN FORCE ON EACH PATH before any regulation on it (m/s): the limit the road model posts on it
    // (RoadModel::postedLimit: a lane its street's, a connector that of the lane it leads into), or else the rule set's
    // default limit for its road's class (RuleSet::defaultLimits over RoadModel::roadClassOf). 0: neither.
    std::vector<Real> limitOfPath;
    // One per junction of the lane level: the index in RuleSet::controls of the row for the control the road model
    // states there (RoadModel::controlAt), or kNoDevice when the rule set has none (issue #260: right_of_way.h reads
    // it). A control with lights while no light runs a programme: the row its `when_dark` names, when it names one.
    std::vector<uint32_t> controlOfJunction;
    // One per junction of the lane level: 1 where controlOfJunction is the row its control is treated as while its
    // lights are dark, not its own.
    std::vector<uint8_t> darkJunction;

    // The regulations along `path`, in order along it (none for a path it does not know).
    BoundRegulations regulationsOn(engine::PathId path) const {
        if (path.index >= regulationsOfPath.size()) return BoundRegulations();
        const engine::Range& r = regulationsOfPath[path.index];
        return BoundRegulations{regulations.data() + r.first, regulations.data() + r.end()};
    }
    // What a regulation means: its device in `rules` (the rule set this was built from), or null for one whose kind
    // the rule set has no device for.
    const Device* device(const RuleSet& rules, const BoundRegulation& regulation) const {
        return regulation.device < rules.devices.size() ? &rules.devices[regulation.device] : nullptr;
    }
    // The limit in force on `path` before any regulation on it (limitOfPath; 0 for a path it does not know).
    Real baseLimit(engine::PathId path) const {
        return path.index < limitOfPath.size() ? limitOfPath[path.index] : Real(0);
    }
};

// BUILDS THE BINDING and checks the registered facts' needs. True when there is no problem. Otherwise false, `out`
// holds what could be bound, and `problems` (when given) gets every problem, in this order: regulation kinds the
// rule set has no device for (in name order), then road classes with unposted paths and no default limit (in name order), then junction
// controls some junction states that the rule set's `control` table lacks (in name order), then facts in slot order,
// then the other readers in the order added, each one's numbers then its settings in the order it declares them.
bool buildRoadRules(const engine::RoadModel& model, const RuleSet& rules, const FactRegistry& facts, RoadRules& out,
                    std::vector<std::string>* problems = nullptr);

}  // namespace driving

#endif
