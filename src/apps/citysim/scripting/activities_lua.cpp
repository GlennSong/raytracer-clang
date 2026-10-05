#include "activities_lua.h"

#include "../../../engine/scripting/lua_state.h"
#include "../../../engine/scripting/script_vm.h"

#include <algorithm>
#include <map>

namespace engine {

namespace {

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
bool flag(lua_State* L, int t, const char* key) {
    lua_getfield(L, t, key);
    const bool v = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    return v;
}
// {lo, hi} pair field
bool pair(lua_State* L, int t, const char* key, double& lo, double& hi) {
    lua_getfield(L, t, key);
    bool ok = true;
    if (lua_istable(L, -1)) {
        lua_rawgeti(L, -1, 1); lo = lua_tonumber(L, -1); lua_pop(L, 1);
        lua_rawgeti(L, -1, 2); hi = lua_tonumber(L, -1); lua_pop(L, 1);
    } else if (!lua_isnil(L, -1)) ok = false;
    lua_pop(L, 1);
    return ok;
}
std::vector<std::string> strings(lua_State* L, int t, const char* key) {
    std::vector<std::string> out;
    lua_getfield(L, t, key);
    if (lua_isstring(L, -1)) out.push_back(lua_tostring(L, -1));
    else if (lua_istable(L, -1)) {
        const int n = static_cast<int>(luaL_len(L, -1));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, -1, i);
            if (lua_isstring(L, -1)) out.push_back(lua_tostring(L, -1));
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    return out;
}
bool knownSite(const std::string& k) {
    static const char* kinds[] = {"cafe", "restaurant", "shop", "supermarket", "civic", "park", "library", "teaching", "quad",
                                  "field", "office", "seat", "bed", "stand", "loop", "pitch", "watch", "street"};
    for (const char* x : kinds) if (k == x) return true;
    return false;
}
// sorted keys of a table at the top of the stack (Lua iteration order is not stable; the catalog's must be)
std::vector<std::string> keys(lua_State* L, int t) {
    std::vector<std::string> ks;
    lua_pushnil(L);
    while (lua_next(L, t)) {
        if (lua_type(L, -2) == LUA_TSTRING) ks.push_back(lua_tostring(L, -2));
        lua_pop(L, 1);
    }
    std::sort(ks.begin(), ks.end());
    return ks;
}
bool fail(std::string* err, const std::string& why) { if (err) *err = why; return false; }

}  // namespace

bool loadActivityCatalog(ScriptVM& vm, citysim::ActivityCatalog& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    citysim::ActivityCatalog c;
    lua_getglobal(L, "activities");
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "activities.lua: no global `activities` table"); }
    const int at = lua_gettop(L);
    // ORDER: an `order` array names them in the order they are listed (the C++ catalog's); otherwise sorted names
    std::vector<std::string> names = strings(L, at, "order");
    if (names.empty()) names = keys(L, at);
    for (const std::string& name : names) {
        if (name == "order") continue;
        lua_getfield(L, at, name.c_str());
        if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "activities." + name + ": not a table"); }
        const int dt = lua_gettop(L);
        citysim::ActivityDef d;
        d.name = name;
        d.sites = strings(L, dt, "sites");
        for (const std::string& k : d.sites)
            if (!knownSite(k)) { lua_settop(L, base); return fail(err, "activities." + name + ": unknown site kind `" + k + "`"); }
        for (const std::string& tg : strings(L, dt, "tags")) {
            const uint32_t b = citysim::spotTagFromName(tg);
            if (!b) { lua_settop(L, base); return fail(err, "activities." + name + ": unknown tag `" + tg + "`"); }
            d.tags |= b;
        }
        if (!pair(L, dt, "hours", d.hourLo, d.hourHi) || !pair(L, dt, "minutes", d.minutesLo, d.minutesHi) ||
            !pair(L, dt, "distance", d.distLo, d.distHi)) {
            lua_settop(L, base);
            return fail(err, "activities." + name + ": hours / minutes / distance must be { lo, hi }");
        }
        d.nearest = static_cast<int>(num(L, dt, "nearest", 0));
        d.perSite = flag(L, dt, "per_site");
        d.walkersOnly = flag(L, dt, "walkers_only");
        d.inShift = flag(L, dt, "in_shift");
        d.bringOwnEighths = static_cast<int>(num(L, dt, "bring_own_eighths", 0));
        const std::string pf = str(L, dt, "perform");
        if (pf.empty() || pf == "inside") d.perform = citysim::Perform::Inside;
        else if (pf == "outside") d.perform = citysim::Perform::Outside;
        else if (pf == "spot") d.perform = citysim::Perform::Spot;
        else if (pf == "wander") d.perform = citysim::Perform::Wander;
        else { lua_settop(L, base); return fail(err, "activities." + name + ": unknown perform `" + pf + "`"); }
        if (d.sites.empty()) { lua_settop(L, base); return fail(err, "activities." + name + ": no sites"); }
        c.defs.push_back(d);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    lua_getglobal(L, "menus");
    if (lua_istable(L, -1)) {
        const int mt = lua_gettop(L);
        std::vector<std::string> menuNames = strings(L, mt, "order");
        if (menuNames.empty()) menuNames = keys(L, mt);
        for (const std::string& mname : menuNames) {
            if (mname == "order") continue;
            lua_getfield(L, mt, mname.c_str());
            if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "menus." + mname + ": not a table"); }
            const int bt = lua_gettop(L);
            citysim::Menu m;
            m.name = mname;
            const int nb = static_cast<int>(luaL_len(L, bt));
            for (int i = 1; i <= nb; ++i) {
                lua_rawgeti(L, bt, i);
                const int band = lua_gettop(L);
                citysim::MenuBand b;
                if (!pair(L, band, "hours", b.hourLo, b.hourHi)) { lua_settop(L, base); return fail(err, "menus." + mname + ": bad hours"); }
                for (const char* field : {"first", "pick"}) {
                    lua_getfield(L, band, field);
                    if (lua_istable(L, -1)) {
                        const int ft = lua_gettop(L);
                        const int ne = static_cast<int>(luaL_len(L, ft));
                        for (int k = 1; k <= ne; ++k) {   // { "activity", number }
                            lua_rawgeti(L, ft, k);
                            citysim::MenuEntry e;
                            lua_rawgeti(L, -1, 1); e.activity = lua_isstring(L, -1) ? lua_tostring(L, -1) : ""; lua_pop(L, 1);
                            lua_rawgeti(L, -1, 2); const double v = lua_tonumber(L, -1); lua_pop(L, 1);
                            lua_pop(L, 1);
                            if (c.find(e.activity) < 0) { lua_settop(L, base); return fail(err, "menus." + mname + ": unknown activity `" + e.activity + "`"); }
                            if (field[0] == 'f') { e.chance = v; b.first.push_back(e); } else { e.weight = v; b.pick.push_back(e); }
                        }
                    }
                    lua_pop(L, 1);
                }
                m.bands.push_back(b);
                lua_pop(L, 1);
            }
            c.menus.push_back(m);
            lua_pop(L, 1);
        }
    }
    lua_settop(L, base);
    out = std::move(c);
    return true;
}

}  // namespace engine
