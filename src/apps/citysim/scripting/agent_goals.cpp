#include "../activities.h"
#include "agent_goals.h"

#include "../../../engine/scripting/lua_state.h"   // luaState() + the Lua C API (scripting-internal)

namespace engine {

using citysim::Activity;
using citysim::GoalAction;
using citysim::GoalEvent;
using citysim::GoalTable;
using citysim::GoalTarget;

namespace {

// String field of the table at absolute stack index `t` ("" when absent).
std::string strField(lua_State* L, int t, const char* key) {
    lua_getfield(L, t, key);
    const char* s = lua_isstring(L, -1) ? lua_tostring(L, -1) : nullptr;
    std::string v = s ? s : "";
    lua_pop(L, 1);
    return v;
}

double numField(lua_State* L, int t, const char* key, double def) {
    lua_getfield(L, t, key);
    double v = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : def;
    lua_pop(L, 1);
    return v;
}

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

bool parseAction(const std::string& s, GoalAction& out) {
    if (s == "rest" || s.empty()) { out = GoalAction::Rest; return true; }
    if (s == "goto") { out = GoalAction::GoTo; return true; }
    return false;
}

bool parseTarget(const std::string& s, GoalTarget& out) {
    if (s.empty() || s == "none") { out = GoalTarget::None; return true; }
    if (s == "work") { out = GoalTarget::Work; return true; }
    if (s == "home") { out = GoalTarget::Home; return true; }
    if (s == "random") { out = GoalTarget::Random; return true; }

    if (s == "shop") { out = GoalTarget::Shop; return true; }
    if (s == "fare") { out = GoalTarget::Fare; return true; }
    if (s == "drop") { out = GoalTarget::Drop; return true; }
    if (s == "stop") { out = GoalTarget::Stop; return true; }
    if (s == "depot") { out = GoalTarget::Depot; return true; }
    if (s == "lunch") { out = GoalTarget::Lunch; return true; }
    if (s == "outing") { out = GoalTarget::Outing; return true; }
    if (s == "campus") { out = GoalTarget::Campus; return true; }
    if (s == "activity") { out = GoalTarget::Activity; return true; }
    return false;
}

bool parseActivity(const std::string& s, Activity& out) {
    if (s.empty() || s == "AtHome") { out = Activity::AtHome; return true; }
    if (s == "Commuting") { out = Activity::Commuting; return true; }
    if (s == "AtWork") { out = Activity::AtWork; return true; }
    if (s == "Returning") { out = Activity::Returning; return true; }

    if (s == "Shopping") { out = Activity::Shopping; return true; }
    if (s == "Outing") { out = Activity::Outing; return true; }
    if (s == "Lunch") { out = Activity::Lunch; return true; }
    return false;
}

}  // namespace

bool loadGoalTable(ScriptVM& vm, const std::string& archetype, GoalTable& out,
                   std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);

    lua_getglobal(L, "agents");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, "agents.lua: no global `agents` table (load it first)");
    }
    lua_getfield(L, -1, archetype.c_str());
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, "agents." + archetype + ": no such archetype table");
    }
    const int t = lua_gettop(L);
    GoalTable table;

    // states = { { name=, action=, target=, activity=, dwell= }, ... }
    lua_getfield(L, t, "states");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, "agents." + archetype + ": missing `states` array");
    }
    const int st = lua_gettop(L);
    const int nStates = static_cast<int>(luaL_len(L, st));
    for (int i = 1; i <= nStates; ++i) {
        lua_rawgeti(L, st, i);
        const int si = lua_gettop(L);
        if (!lua_istable(L, si)) {
            lua_settop(L, base);
            return fail(err, "agents." + archetype + ": states[" +
                                 std::to_string(i) + "] is not a table");
        }
        std::string name = strField(L, si, "name");
        GoalAction action;
        GoalTarget target;
        Activity activity;
        bool ok = !name.empty() && parseAction(strField(L, si, "action"), action) &&
                  parseTarget(strField(L, si, "target"), target) &&
                  parseActivity(strField(L, si, "activity"), activity);
        double dwell = numField(L, si, "dwell", 0.0);
        const std::string menuName = strField(L, si, "menu");   // an Activity state may pick from a catalog menu
        // an ACTIVITY state's ask: spot = { kinds = { "sit", ... }, tags = { "campus", ... } }
        uint32_t spotKinds = 0, spotTags = 0;
        lua_getfield(L, si, "spot");
        if (lua_istable(L, -1)) {
            const int sp = lua_gettop(L);
            for (const char* field : {"kinds", "tags"}) {
                lua_getfield(L, sp, field);
                if (lua_istable(L, -1)) {
                    const int arr = lua_gettop(L);
                    const int n = static_cast<int>(luaL_len(L, arr));
                    for (int k = 1; k <= n; ++k) {
                        lua_rawgeti(L, arr, k);
                        const std::string v = lua_isstring(L, -1) ? lua_tostring(L, -1) : "";
                        lua_pop(L, 1);
                        if (field[0] == 'k') {
                            citysim::SpotKind sk;
                            if (citysim::spotKindFromName(v, sk)) spotKinds |= citysim::spotKindBit(sk);
                            else ok = false;
                        } else {
                            const uint32_t tg = citysim::spotTagFromName(v);
                            if (tg) spotTags |= tg; else ok = false;
                        }
                    }
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);   // spot
        // (a seat by default -- unless it picks from a menu, which names its own sites)
        if (target == GoalTarget::Activity && spotKinds == 0 && menuName.empty()) spotKinds = citysim::spotKindBit(citysim::SpotKind::Sit);
        lua_pop(L, 1);
        if (!ok || dwell < 0) {
            lua_settop(L, base);
            return fail(err, "agents." + archetype + ": bad state `" + name + "`");
        }
        table.addState(name, action, target, activity, dwell);
        if (target == GoalTarget::Activity) table.setActivity(name, spotKinds, spotTags);
        if (target == GoalTarget::Activity && !menuName.empty()) table.setMenu(name, menuName);
    }
    lua_pop(L, 1);   // states

    // transitions = { { from=, event=, to= }, ... } — insertion order kept
    // (the first matching row wins at run time, exactly like the C++ tables).
    lua_getfield(L, t, "transitions");
    if (lua_istable(L, -1)) {
        const int tr = lua_gettop(L);
        const int nRows = static_cast<int>(luaL_len(L, tr));
        for (int i = 1; i <= nRows; ++i) {
            lua_rawgeti(L, tr, i);
            const int ri = lua_gettop(L);
            std::string from = lua_istable(L, ri) ? strField(L, ri, "from") : "";
            std::string event = lua_istable(L, ri) ? strField(L, ri, "event") : "";
            std::string to = lua_istable(L, ri) ? strField(L, ri, "to") : "";
            lua_pop(L, 1);
            bool evOk = false;
            GoalEvent ev = citysim::goalEventFromName(event, &evOk);
            if (!evOk || !table.addTransition(from, ev, to)) {
                lua_settop(L, base);
                return fail(err, "agents." + archetype + ": bad transition `" +
                                     from + " " + event + " -> " + to + "`");
            }
        }
    }
    lua_pop(L, 1);   // transitions (or non-table)

    std::string entry = strField(L, t, "entry");
    if (entry.empty() || !table.setEntry(entry)) {
        lua_settop(L, base);
        return fail(err, "agents." + archetype + ": bad or missing `entry`");
    }

    lua_settop(L, base);
    out = std::move(table);
    return true;
}

}  // namespace engine
