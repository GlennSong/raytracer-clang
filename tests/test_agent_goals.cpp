#include "test_framework.h"

#include "../src/apps/citysim/city_goals.h"
#include "../src/engine/scripting/agent_goals.h"
#include "../src/engine/scripting/script_vm.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;
using namespace citysim;

// The agents.lua reader (ADR-0064): the shipped asset must rebuild the C++
// default goal tables EXACTLY (describe() is the canonical dump — identical
// text means identical behaviour), custom tables must parse with their params,
// and a broken script must fail loudly at load, never misbehave at tick time.

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// A VM with the shipped assets/scripts/agents.lua loaded (global `agents`).
struct AgentsVM {
    ScriptVM vm;
    bool loaded = false;
    AgentsVM() {
        std::string src =
            readFile(std::string(RT_SOURCE_DIR) + "/assets/scripts/agents.lua");
        std::string err;
        loaded = !src.empty() && vm.doString(src, &err);
        if (!loaded) std::printf("    agents.lua load error: %s\n", err.c_str());
    }
};

}  // namespace

TEST_CASE(agents_lua_rebuilds_the_default_schedule_table) {
    AgentsVM a;
    CHECK(a.loaded);
    GoalTable t;
    std::string err;
    bool ok = loadGoalTable(a.vm, "schedule", t, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    CHECK(t.describe() == defaultScheduleGoals().describe());
}

TEST_CASE(agents_lua_rebuilds_the_wander_tables) {
    AgentsVM a;
    CHECK(a.loaded);
    GoalTable d, p;
    CHECK(loadGoalTable(a.vm, "wander_driver", d));
    CHECK(d.describe() == wanderGoals(true).describe());
    CHECK(loadGoalTable(a.vm, "wander_pedestrian", p));
    CHECK(p.describe() == wanderGoals(false).describe());
}

TEST_CASE(agents_lua_custom_table_carries_action_params) {
    // A behaviour authored from scratch in Lua — "go somewhere random, browse
    // two hours, repeat" — arrives with its params (target, activity, dwell).
    ScriptVM vm;
    std::string err;
    CHECK(vm.doString(R"lua(
        agents = {
            errands = {
                entry = "Errand",
                states = {
                    { name = "Errand", action = "goto", target = "random",
                      activity = "Commuting" },
                    { name = "Browse", action = "rest", activity = "AtWork",
                      dwell = 2 },
                },
                transitions = {
                    { from = "Errand", event = "arrived",   to = "Browse" },
                    { from = "Browse", event = "dwellDone", to = "Errand" },
                },
            },
        }
    )lua", &err));
    GoalTable t;
    bool ok = loadGoalTable(vm, "errands", t, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    int errand = t.findState("Errand");
    int browse = t.findState("Browse");
    CHECK(t.entry() == errand);
    CHECK(t.state(errand).action == GoalAction::GoTo);
    CHECK(t.state(errand).target == GoalTarget::Random);
    CHECK(t.state(browse).action == GoalAction::Rest);
    CHECK(t.state(browse).activity == Activity::AtWork);
    CHECK(t.state(browse).dwellHours == 2.0);
    CHECK(t.onEvent(errand, GoalEvent::Arrived) == browse);
    CHECK(t.onEvent(browse, GoalEvent::DwellDone) == errand);
}

TEST_CASE(agents_lua_reader_rejects_malformed_tables) {
    ScriptVM vm;
    std::string err;
    CHECK(vm.doString(R"lua(
        agents = {
            noentry  = { states = { { name = "A", action = "rest" } } },
            badevent = { entry = "A",
                         states = { { name = "A", action = "rest" } },
                         transitions = { { from = "A", event = "teleported",
                                           to = "A" } } },
            badstate = { entry = "A",
                         states = { { name = "A", action = "levitate" } } },
        }
    )lua", &err));
    GoalTable t;
    CHECK(!loadGoalTable(vm, "missing", t, &err));    // no such archetype
    CHECK(!err.empty());
    CHECK(!loadGoalTable(vm, "noentry", t, &err));    // entry is mandatory
    CHECK(!loadGoalTable(vm, "badevent", t, &err));   // unknown event name
    CHECK(!loadGoalTable(vm, "badstate", t, &err));   // unknown action name
}
