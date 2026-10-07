#include "roles_lua.h"

#include "../../../engine/scripting/lua_state.h"
#include "../../../engine/scripting/script_vm.h"

namespace engine {

namespace {

bool fail(std::string* err, const std::string& why) { if (err) *err = why; return false; }
std::string str(lua_State* L, int t, const char* key) {
    lua_getfield(L, t, key);
    std::string s = lua_isstring(L, -1) ? lua_tostring(L, -1) : "";
    lua_pop(L, 1);
    return s;
}
double num(lua_State* L, int t, const char* key, double def) {
    lua_getfield(L, t, key);
    const double v = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : def;
    lua_pop(L, 1);
    return v;
}
bool flag(lua_State* L, int t, const char* key, bool def) {
    lua_getfield(L, t, key);
    const bool v = lua_isnil(L, -1) ? def : lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    return v;
}
bool target(const std::string& s, citysim::GoalTarget& out) {
    if (s == "lunch") { out = citysim::GoalTarget::Lunch; return true; }
    if (s == "campus") { out = citysim::GoalTarget::Campus; return true; }
    if (s == "activity") { out = citysim::GoalTarget::Activity; return true; }
    return false;
}
bool label(const std::string& s, citysim::Activity& out) {
    static const char* names[] = {"AtHome", "Commuting", "AtWork", "Returning", "Shopping", "Outing", "Lunch"};
    for (int i = 0; i < 7; ++i)
        if (s == names[i]) { out = static_cast<citysim::Activity>(i); return true; }
    return false;
}

}  // namespace

bool loadRoleCatalog(ScriptVM& vm, citysim::RoleCatalog& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "roles");
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "roles.lua: no global `roles` table"); }
    const int rt = lua_gettop(L);
    std::vector<std::string> names;
    lua_getfield(L, rt, "order");
    if (lua_istable(L, -1)) {
        const int n = static_cast<int>(luaL_len(L, -1));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, -1, i);
            if (lua_isstring(L, -1)) names.push_back(lua_tostring(L, -1));
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    if (names.empty()) { lua_settop(L, base); return fail(err, "roles.lua: `roles.order` must name the roles"); }
    citysim::RoleCatalog c;
    for (const std::string& name : names) {
        lua_getfield(L, rt, name.c_str());
        if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "roles." + name + ": not a table"); }
        const int t = lua_gettop(L);
        citysim::ResidentRole r;
        r.name = name;
        r.nightOutShare = num(L, t, "night_out", 0);
        citysim::RoleDay& d = r.day;
        lua_getfield(L, t, "day");
        if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "roles." + name + ": no `day`"); }
        const int dt = lua_gettop(L);
        d.commute = flag(L, dt, "commute", true);
        d.outingPause = num(L, dt, "outing_pause", d.outingPause);
        d.workBeforeBreak = num(L, dt, "work_before_break", 0);
        d.errand = flag(L, dt, "errand", false);
        d.eveningMenu = str(L, dt, "evening");
        lua_getfield(L, dt, "pause");   // the break (`break` is a Lua keyword)
        if (lua_istable(L, -1)) {
            const int bt = lua_gettop(L);
            if (!target(str(L, bt, "target"), d.breakTarget)) {
                lua_settop(L, base);
                return fail(err, "roles." + name + ".day.pause: target must be lunch, campus or activity");
            }
            d.breakMenu = str(L, bt, "menu");
            const std::string lb = str(L, bt, "label");
            if (!lb.empty() && !label(lb, d.breakActivity)) {
                lua_settop(L, base);
                return fail(err, "roles." + name + ".day.pause: unknown label `" + lb + "`");
            }
            d.breakHours = num(L, bt, "hours", 0);
            d.breaksRepeat = flag(L, bt, "repeats", false);
        }
        lua_pop(L, 1);   // pause
        lua_pop(L, 1);   // day
        lua_pop(L, 1);   // role
        c.roles.push_back(r);
    }
    lua_settop(L, base);
    out = std::move(c);
    return true;
}

}  // namespace engine
