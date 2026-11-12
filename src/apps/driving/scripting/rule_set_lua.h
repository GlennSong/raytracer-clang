#ifndef RAYTRACER_APPS_DRIVING_SCRIPTING_RULE_SET_LUA_H
#define RAYTRACER_APPS_DRIVING_SCRIPTING_RULE_SET_LUA_H

// READS A RULE SET OUT OF LUA (driving-model-design.md 6.1 a, c, d and 6.4; issue #250). The caller has already run the
// rule-set file in `vm`, which defined a global table `rule_set`. This reads the parts C3 owns (name, drives_on,
// ranks, devices, numbers, personality, and the optional default_limits, written like the numbers) into plain structs
// and checks them. It also reads the optional right-of-way tables (issue #260): `control`, each kind of junction
// control as { settle = { question, ... }, everyone_stops = bool, gives_way_on_green = { movement, ... },
// arrow = { movement, ... }, turn_on_red = { movement = ..., after = "full_stop" }, when_dark = "<another control>" }
// (when_dark: what a control with lights is treated as while no light runs a programme, issue #240), and
// `movement_rank`, a list of movements, earlier goes first. Other top-level keys (the planner's `placement`, read by
// loadPlacementPolicy below) are left for the later readers and are not errors; unknown keys inside the parts read here
// ARE, so a typo is caught.
//
// A value in a Lua table carries no line number, so an error names the file and the full key path and says what was
// wrong and what was expected. The reader collects up to kMaxRuleSetErrors and reports them all, one per line. On any
// error `out` is left untouched. There is no built-in rule set and no fallback: a missing `rule_set` is an error.
//
// This is the only driving file allowed to include Lua; the rest of the library never sees it.

#include "apps/driving/rule_set.h"

#include <string>

namespace engine {
class ScriptVM;
namespace roads::lanes {
struct ControlPolicy;
}
}  // namespace engine

namespace driving {

// The most faults one load reports (a file that bad needs fixing from the top anyway).
constexpr std::size_t kMaxRuleSetErrors = 20;

// `fileName` (for example "us_left.lua") is only used to name the file in messages.
bool loadRuleSet(engine::ScriptVM& vm, const std::string& fileName, RuleSet& out, std::string* err);

// READS THE PLACEMENT POLICY (issue #240; engine/procgen/city/roads/lanes/control_plan.h) out of the same file: the
// rule set's `placement` table, which the city planner uses to give every junction a control and every road a posted
// limit. `rules` is the rule set loadRuleSet read from that file; the policy is checked against it, so a row naming a
// control the rule set has no row for, or a sign it has no device for, is an error. The table:
//   placement = {
//     class_order = { "freeway", "arterial", ... },          -- road classes, the most important first
//     speed_limits = { arterial = { 35, unit = "mph", source = "..." }, ... },   -- posted per class (optional)
//     junctions = {                                          -- rows, the first that holds decides
//       { name = "...", when = { <conditions> }, apart = { 200, unit = "m", source = "..." },
//         control = "...", signs = { major = ..., minor = ..., all = ... } },
//       ...,
//       { name = "...", control = "..." },                   -- the last row asks nothing: it holds for every junction
//     },
//   }
// Conditions: arms, approaches, major_lanes, minor_lanes = { min = n, max = n }; major, minor = a class or a list of
// classes; has_minor, ramp_end = true or false; major_limit, minor_limit = { min = { v, unit = ... }, max = ... }. Faults
// are reported like loadRuleSet's, every one, with the key path; on any, `out` is left untouched.
bool loadPlacementPolicy(engine::ScriptVM& vm, const std::string& fileName, const RuleSet& rules,
                         engine::roads::lanes::ControlPolicy& out, std::string* err);

}  // namespace driving

#endif
