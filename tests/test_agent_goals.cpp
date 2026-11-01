#include "test_framework.h"

#include "../src/apps/citysim/city_goals.h"
#include "../src/apps/citysim/activities.h"
#include "../src/apps/citysim/scripting/agent_goals.h"
#include "../src/apps/citysim/scripting/roles_lua.h"
#include "../src/apps/citysim/scripting/signals_lua.h"
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

// THE ROLES (city_roles.h): roles.lua is the built-in catalog, and the days it builds are the built-in days -- the
// residents' tables come from roles, not from hand-written tables in agents.lua.
TEST_CASE(roles_lua_is_the_built_in_role_catalog) {
    ScriptVM vm;
    std::string err;
    std::ifstream f("assets/scripts/roles.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    CHECK(!ss.str().empty());
    CHECK(vm.doString(ss.str(), &err));
    citysim::RoleCatalog lua;
    const bool ok = loadRoleCatalog(vm, lua, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    const std::string a = lua.describe(), b = citysim::defaultRoleCatalog().describe();
    if (a != b) std::printf("    LUA:\n%s\n    C++:\n%s\n", a.c_str(), b.c_str());
    CHECK(a == b);
    for (const citysim::ResidentRole& r : lua.roles)
        CHECK(buildDayTable(r.day).describe() == citysim::builtinDayTable(r.name).describe());
    // the first four are the built-ins (kRoleCommuter...)
    CHECK(lua.find("commuter") == citysim::kRoleCommuter && lua.find("shopkeeper") == citysim::kRoleShopkeeper &&
          lua.find("stroller") == citysim::kRoleStroller && lua.find("student") == citysim::kRoleStudent);
    CHECK(citysim::builtinRolesLead(lua).empty());
    CHECK(std::string(lua.nameOf(citysim::kRoleStudent)) == "student");
    // a catalog that reorders the built-ins is refused: an agent's role id would name the wrong day
    ScriptVM swapped;
    CHECK(swapped.doString(ss.str() + "\nroles.order[1], roles.order[3] = roles.order[3], roles.order[1]", &err));
    citysim::RoleCatalog wrong;
    CHECK(!loadRoleCatalog(swapped, wrong, &err));
    CHECK(err.find("commuter") != std::string::npos);
    ScriptVM bad;
    CHECK(bad.doString("roles = { order = { 'x' }, x = { day = { pause = { target = 'moon' } } } }", &err));
    citysim::RoleCatalog none;
    CHECK(!loadRoleCatalog(bad, none, &err));
}

// THE SIGNALS' CYCLE: signals.lua is SignalTiming's defaults, and a time that is not positive is refused.
TEST_CASE(signals_lua_is_the_built_in_signal_timing) {
    ScriptVM vm;
    std::string err;
    std::ifstream f("assets/scripts/signals.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    CHECK(!ss.str().empty());
    CHECK(vm.doString(ss.str(), &err));
    citysim::SignalTiming lua;
    lua.green = -1;
    const bool ok = loadSignalTiming(vm, lua, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    const citysim::SignalTiming cpp;
    CHECK(lua.green == cpp.green && lua.yellow == cpp.yellow && lua.allRed == cpp.allRed && lua.walk == cpp.walk &&
          lua.arrow == cpp.arrow && lua.arrowYellow == cpp.arrowYellow);
    ScriptVM bad;
    CHECK(bad.doString("signals = { green = 0 }", &err));
    citysim::SignalTiming t;
    CHECK(!loadSignalTiming(bad, t, &err));
    CHECK(err.find("green") != std::string::npos);
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

// ACTIVITY SPOTS (behaviour plan, step 2): a behaviour that asks for "an activity of these kinds, with these tags" is
// pure Lua -- target "activity" with spot = { kinds, tags } -- and an unknown kind or tag is refused, not guessed.
TEST_CASE(agents_lua_asks_for_an_activity_by_kind_and_tag) {
    ScriptVM vm;
    std::string err;
    CHECK(vm.doString(R"lua(
        agents = {
            loafer = {
                entry = "Find",
                states = {
                    { name = "Find", action = "goto", target = "activity", activity = "Outing",
                      spot = { kinds = { "sit", "watch" }, tags = { "campus" } } },
                    { name = "Sit", action = "rest", activity = "Outing", dwell = 0.2 },
                },
                transitions = {
                    { from = "Find", event = "arrived", to = "Sit" },
                    { from = "Find", event = "noRoute", to = "Sit" },
                    { from = "Sit", event = "dwellDone", to = "Find" },
                },
            },
            bad = {
                entry = "Find",
                states = { { name = "Find", action = "goto", target = "activity", spot = { kinds = { "juggle" } } } },
            },
        }
    )lua", &err));
    GoalTable t;
    const bool ok = loadGoalTable(vm, "loafer", t, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    const GoalState& f = t.state(t.findState("Find"));
    CHECK(f.target == GoalTarget::Activity);
    CHECK(f.spotKinds == (citysim::spotKindBit(citysim::SpotKind::Sit) | citysim::spotKindBit(citysim::SpotKind::Watch)));
    CHECK(f.spotTags == citysim::spot_tag::kCampus);
    GoalTable b;
    CHECK(!loadGoalTable(vm, "bad", b, &err));
}

// THE ACTIVITY CATALOG IN LUA (activities framework, step A): assets/scripts/activities.lua describes exactly the
// catalog built into C++ -- the outings, lunch and students' breaks the town ran on before -- so the script can take
// over without a change in behaviour; a menu naming an unknown activity, or an unknown site kind, is refused.
#include "../src/apps/citysim/scripting/activities_lua.h"
TEST_CASE(activities_lua_is_the_built_in_catalog) {
    ScriptVM vm;
    std::string err;
    std::ifstream f("assets/scripts/activities.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    CHECK(!ss.str().empty());
    CHECK(vm.doString(ss.str(), &err));
    citysim::ActivityCatalog lua;
    const bool ok = loadActivityCatalog(vm, lua, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    const std::string a = lua.describe(), b = citysim::defaultActivityCatalog().describe();
    if (a != b) std::printf("    LUA:\n%s\n    C++:\n%s\n", a.c_str(), b.c_str());
    CHECK(a == b);
    ScriptVM bad;
    CHECK(bad.doString("activities = { x = { sites = { \"moon\" } } }", &err));
    citysim::ActivityCatalog c;
    CHECK(!loadActivityCatalog(bad, c, &err));
    ScriptVM bad2;
    CHECK(bad2.doString("activities = { x = { sites = { \"cafe\" } } } menus = { m = { { first = { { \"y\", 1 } } } } }", &err));
    CHECK(!loadActivityCatalog(bad2, c, &err));
}
