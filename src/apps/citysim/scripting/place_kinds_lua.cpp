#include "place_kinds_lua.h"

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
// The strings of the list at t[key] (none when it is absent).
std::vector<std::string> strings(lua_State* L, int t, const char* key) {
    std::vector<std::string> out;
    lua_getfield(L, t, key);
    if (lua_istable(L, -1)) {
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
// The { beds, m2, share } list at t[key].
std::vector<citysim::DwellingSize> sizes(lua_State* L, int t, const char* key) {
    std::vector<citysim::DwellingSize> out;
    lua_getfield(L, t, key);
    if (lua_istable(L, -1)) {
        const int n = static_cast<int>(luaL_len(L, -1));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, -1, i);
            if (lua_istable(L, -1)) {
                const int st = lua_gettop(L);
                citysim::DwellingSize d;
                d.beds = static_cast<int>(num(L, st, "beds", 1));
                d.m2 = num(L, st, "m2", 0);
                d.share = num(L, st, "share", 0);
                out.push_back(d);
            }
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    return out;
}

}  // namespace

bool loadPlaceKindCatalog(ScriptVM& vm, citysim::PlaceKindCatalog& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "places");
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "places.lua: no global `places` table"); }
    const int pt = lua_gettop(L);
    const std::vector<std::string> names = strings(L, pt, "order");
    if (names.empty()) { lua_settop(L, base); return fail(err, "places.lua: `places.order` must name the kinds"); }
    citysim::PlaceKindCatalog c;
    lua_getfield(L, pt, "kinds");
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "places.lua: no `places.kinds`"); }
    const int kt = lua_gettop(L);
    for (const std::string& name : names) {
        lua_getfield(L, kt, name.c_str());
        if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "places.kinds." + name + ": not a table"); }
        const int t = lua_gettop(L);
        citysim::PlaceKind k;
        k.name = name;
        const std::string type = str(L, t, "type");
        if (!citysim::parsePlaceType(type, k.type)) {
            lua_settop(L, base);
            return fail(err, "places.kinds." + name + ": unknown type `" + type + "`");
        }
        k.recipes = strings(L, t, "recipes");
        k.tagNames = strings(L, t, "tags");
        k.m2PerBed = num(L, t, "bed_m2", 0);
        lua_pop(L, 1);   // kind
        c.kinds.push_back(std::move(k));
    }
    lua_pop(L, 1);   // kinds
    c.internTags();
    lua_getfield(L, pt, "dwellings");
    if (lua_istable(L, -1)) {
        const int dt = lua_gettop(L);
        lua_pushnil(L);
        while (lua_next(L, dt)) {
            if (lua_type(L, -2) == LUA_TSTRING && lua_istable(L, -1)) {
                const int rt = lua_gettop(L);
                citysim::DwellingRule r;
                lua_getfield(L, rt, "per_unit");
                r.perUnit = lua_toboolean(L, -1) != 0;
                lua_pop(L, 1);
                r.usable = num(L, rt, "usable", r.usable);
                r.mix = sizes(L, rt, "mix");
                c.dwellings[lua_tostring(L, -2)] = std::move(r);
            }
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);   // dwellings
    c.houseSizes = sizes(L, pt, "house_sizes");
    lua_settop(L, base);
    if (const std::string wrong = c.check(); !wrong.empty()) return fail(err, "places.lua: " + wrong);
    out = std::move(c);
    return true;
}

}  // namespace engine
