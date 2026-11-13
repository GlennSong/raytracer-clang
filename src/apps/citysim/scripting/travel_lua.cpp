#include "travel_lua.h"

#include "../../../engine/scripting/lua_helpers.h"
#include "../../../engine/scripting/script_vm.h"

#include <algorithm>

namespace engine {

namespace {

bool fail(lua_State* L, int base, std::string* err, const std::string& why) {
    lua_settop(L, base);
    if (err) *err = "travel.lua: " + why;
    return false;
}

// A {lo, hi} band of metres; absent keeps the default.
bool band(lua_State* L, int t, const char* key, Real& lo, Real& hi) {
    lua_getfield(L, t, key);
    bool ok = true;
    if (lua_istable(L, -1)) {
        lua_rawgeti(L, -1, 1);
        lua_rawgeti(L, -2, 2);
        ok = lua_isnumber(L, -2) && lua_isnumber(L, -1) && lua_tonumber(L, -2) >= 0 &&
             lua_tonumber(L, -1) > lua_tonumber(L, -2);
        if (ok) { lo = lua_tonumber(L, -2); hi = lua_tonumber(L, -1); }
        lua_pop(L, 2);
    }
    lua_pop(L, 1);
    return ok;
}

}  // namespace

bool loadTravelCatalog(ScriptVM& vm, citysim::TravelCatalog& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "travel");
    if (!lua_istable(L, -1)) return fail(L, base, err, "no global `travel` table");
    const int tt = lua_gettop(L);
    citysim::TravelCatalog cat;
    std::vector<std::string> access;
    lua_getfield(L, tt, "modes");
    if (!lua_istable(L, -1)) return fail(L, base, err, "no `modes` list");
    const lua_Integer n = luaL_len(L, -1);
    for (lua_Integer i = 1; i <= n; ++i) {
        lua_rawgeti(L, -1, i);
        const int mt = lua_gettop(L);
        if (!lua_istable(L, mt)) return fail(L, base, err, "modes[" + std::to_string(i) + "] is not a table");
        citysim::TravelMode m;
        m.name = strField(L, mt, "name");
        if (m.name.empty()) return fail(L, base, err, "modes[" + std::to_string(i) + "] has no name");
        if (cat.find(m.name) >= 0) return fail(L, base, err, "`" + m.name + "` is listed twice");
        const std::string net = strField(L, mt, "network");
        if (net != "roads" && net != "footpaths")
            return fail(L, base, err, "`" + m.name + "`: network must be \"roads\" or \"footpaths\", not \"" + net + "\"");
        m.roads = net == "roads";
        // a walk's speed is the table's; a drive's minutes are the router's time for its route (ADR-0416)
        const double kmh = numField(L, mt, "speed_kmh", 0.0);
        if (m.roads) {
            if (kmh != 0.0)
                return fail(L, base, err, "`" + m.name + "`: a way on the roads has no speed_kmh -- its minutes are the "
                                              "router's own time by road class (pathfind.h classSpeed)");
            m.speed = 0;
        } else {
            if (!(kmh > 0)) return fail(L, base, err, "`" + m.name + "`: speed_kmh must be positive");
            m.speed = kmh / 3.6;
        }
        m.startMinutes = numField(L, mt, "start_minutes", 0.0);
        m.endMinutes = numField(L, mt, "end_minutes", 0.0);
        if (m.startMinutes < 0 || m.endMinutes < 0)
            return fail(L, base, err, "`" + m.name + "`: start_minutes and end_minutes cannot be negative");
        m.ownVehicle = boolField(L, mt, "own_vehicle", false);
        m.weight = numField(L, mt, "weight", 1.0);
        if (!(m.weight > 0)) return fail(L, base, err, "`" + m.name + "`: weight must be positive");
        m.colour = vec3Field(L, mt, "colour", m.colour);
        access.push_back(strField(L, mt, "access"));
        if (m.ownVehicle && access.back().empty())
            return fail(L, base, err, "`" + m.name + "` needs a vehicle, so it needs an `access` way to reach it");
        cat.modes.push_back(m);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    if (cat.modes.empty()) return fail(L, base, err, "`modes` is empty");
    for (std::size_t i = 0; i < cat.modes.size(); ++i) {
        if (access[i].empty()) continue;
        const int a = cat.find(access[i]);
        if (a < 0) return fail(L, base, err, "`" + cat.modes[i].name + "`: access `" + access[i] + "` is not a way listed");
        if (cat.modes[static_cast<std::size_t>(a)].ownVehicle)
            return fail(L, base, err, "`" + cat.modes[i].name + "`: its access way `" + access[i] +
                                          "` needs a vehicle itself");
        cat.modes[i].access = a;
    }
    // the kinds of road: each a name and the road classes it covers, a class in one kind at most
    lua_getfield(L, tt, "road_kinds");
    if (lua_istable(L, -1)) {
        const lua_Integer nk = luaL_len(L, -1);
        for (lua_Integer i = 1; i <= nk; ++i) {
            lua_rawgeti(L, -1, i);
            const int kt = lua_gettop(L);
            const std::string at = "road_kinds[" + std::to_string(i) + "]";
            if (!lua_istable(L, kt)) return fail(L, base, err, at + " is not a table");
            citysim::RoadKind k;
            k.name = strField(L, kt, "name");
            if (k.name.empty()) return fail(L, base, err, at + " has no name");
            k.one = strField(L, kt, "one");
            k.count = boolField(L, kt, "count", false);
            lua_getfield(L, kt, "colour");
            k.hasColour = lua_istable(L, -1);
            lua_pop(L, 1);
            k.colour = vec3Field(L, kt, "colour", k.colour);
            lua_getfield(L, kt, "classes");
            if (!lua_istable(L, -1)) return fail(L, base, err, "road kind `" + k.name + "` has no `classes` list");
            const lua_Integer nc = luaL_len(L, -1);
            for (lua_Integer c = 1; c <= nc; ++c) {
                lua_rawgeti(L, -1, c);
                const std::string cn = lua_isstring(L, -1) ? lua_tostring(L, -1) : "";
                lua_pop(L, 1);
                RoadClass rc;
                if (!parseRoadClass(cn, rc))
                    return fail(L, base, err, "road kind `" + k.name + "`: `" + cn + "` is not a road class");
                if (cat.roadKindOf(rc) >= 0 || std::find(k.classes.begin(), k.classes.end(), rc) != k.classes.end())
                    return fail(L, base, err, "road class `" + cn + "` is listed twice in road_kinds");
                k.classes.push_back(rc);
            }
            lua_pop(L, 1);
            cat.roadKinds.push_back(std::move(k));
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    if (!band(L, tt, "near", cat.nearMin, cat.nearMax)) return fail(L, base, err, "`near` must be {lo, hi}, lo < hi");
    if (!band(L, tt, "far", cat.farMin, cat.farMax)) return fail(L, base, err, "`far` must be {lo, hi}, lo < hi");
    lua_settop(L, base);
    out = std::move(cat);
    return true;
}

}  // namespace engine
