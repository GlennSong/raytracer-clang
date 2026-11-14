#ifndef RAYTRACER_APPS_CITYSIM_CONTROL_POLICY_FILE_H
#define RAYTRACER_APPS_CITYSIM_CONTROL_POLICY_FILE_H

// THE SHARED CONTROL PLACEMENT POLICY, READ FROM ITS FILES (ADR-0422). A rule set (assets/scripts/rules/lab.lua) names
// the policy every city shares (`placement = { policy = "rules/control_policy.lua" }`); the driving library's reader
// opens no files, so this host does: it runs the rule set's file, then the policy's file it names, in one VM, and reads
// the policy checked against the rule set (driving/scripting/rule_set_lua.h loadPlacementPolicy). What a planner calls
// (tools/city_plan.cpp: a planned city's scene, the island's, and the `controls` verb).

#include "engine/procgen/city/roads/lanes/control_plan.h"

#include <string>

namespace citysim {

// The rule set a city is planned with when nothing names another: today the one there is.
constexpr const char* kDefaultRuleSet = "rules/lab.lua";

// The rule set at `ruleSetFile` (under assets/scripts), then its placement policy -- the shared file it names, or its own
// table -- checked against it. False (with `err`) when a file is missing or a check fails; `out` is then untouched.
// Without scripting in the build: false.
bool loadControlPolicy(const std::string& ruleSetFile, engine::roads::lanes::ControlPolicy& out, std::string* err);

}  // namespace citysim

#endif
