#include "crowd_lua.h"

#include "../../../engine/scripting/lua_helpers.h"
#include "../../../engine/scripting/script_vm.h"

#include <functional>

namespace engine {

bool loadCrowdSpec(ScriptVM& vm, const std::string& file, citysim::CrowdSpec& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    const auto fail = [&](const std::string& why) {
        lua_settop(L, base);
        if (err) *err = file + ": crowd." + why;
        return false;
    };
    lua_getglobal(L, "crowd");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        if (err) *err = file + ": no global `crowd` table";
        return false;
    }
    const int ct = lua_gettop(L);
    citysim::CrowdSpec spec;
    spec.residents = static_cast<int>(numField(L, ct, "residents", 0));
    if (spec.residents <= 0) return fail("residents must be a count above zero");
    spec.intent = strField(L, ct, "intent");
    if (spec.intent.empty()) return fail("intent: a row of brain.lua's `intents` is needed");
    spec.keepFromEnds = numField(L, ct, "keep_from_ends", 0);
    if (spec.keepFromEnds < 0) return fail("keep_from_ends cannot be negative");
    // each row of the list-field `key` in turn, the row's table on top of the stack (false from `row`: refused)
    const auto rows = [&](const char* key, const std::function<bool(int, const std::string&)>& row) {
        lua_getfield(L, ct, key);
        if (!lua_istable(L, -1)) return false;
        const lua_Integer n = luaL_len(L, -1);
        for (lua_Integer i = 1; i <= n; ++i) {
            lua_rawgeti(L, -1, i);
            const std::string at = std::string(key) + "[" + std::to_string(i) + "]";
            if (!lua_istable(L, -1) || !row(lua_gettop(L), at)) return false;
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
        return n > 0;
    };
    std::string bad;
    const auto refuse = [&](const std::string& why) {
        bad = why;
        return false;
    };
    const auto share = [&](int t, const char* valueKey, std::vector<citysim::CrowdShare>& into, const std::string& at) {
        citysim::CrowdShare s;
        s.value = static_cast<int>(numField(L, t, valueKey, -1));
        s.share = numField(L, t, "share", -1);
        if (s.value < 0 || s.share < 0) return refuse(at + " needs `" + valueKey + "` and `share`, neither below zero");
        into.push_back(s);
        return true;
    };
    if (!rows("places", [&](int t, const std::string& at) {
            citysim::CrowdPlaceKind k;
            k.kind = strField(L, t, "kind");
            k.homes = boolField(L, t, "homes", false);
            k.parking = boolField(L, t, "parking", false);
            k.count = static_cast<int>(numField(L, t, "count", 0));
            if (k.kind.empty()) return refuse(at + " has no kind");
            if (k.count < 0) return refuse(at + ": count cannot be negative");
            spec.places.push_back(k);
            return true;
        }))
        return fail(bad.empty() ? "places: a list of kinds of place is needed" : bad);
    if (!rows("household_sizes", [&](int t, const std::string& at) { return share(t, "size", spec.householdSizes, at); }))
        return fail(bad.empty() ? "household_sizes: a list is needed" : bad);
    if (!rows("cars", [&](int t, const std::string& at) { return share(t, "cars", spec.carsPerHousehold, at); }))
        return fail(bad.empty() ? "cars: a list is needed" : bad);
    lua_getfield(L, ct, "traits");
    const bool hasTraits = lua_istable(L, -1);
    lua_pop(L, 1);
    if (hasTraits && !rows("traits", [&](int t, const std::string& at) {
            citysim::CrowdTrait tr;
            tr.name = strField(L, t, "name");
            tr.share = numField(L, t, "share", -1);
            if (tr.name.empty() || tr.share < 0) return refuse(at + " needs a `name` and a `share` not below zero");
            spec.traits.push_back(tr);
            return true;
        }) && !bad.empty())
        return fail(bad);
    if (!rows("days", [&](int t, const std::string& at) {
            citysim::CrowdDay d;
            d.share = numField(L, t, "share", -1);
            if (d.share < 0) return refuse(at + ": share cannot be below zero");
            lua_getfield(L, t, "stops");
            if (!lua_istable(L, -1)) return refuse(at + " has no stops");
            const lua_Integer n = luaL_len(L, -1);
            for (lua_Integer i = 1; i <= n; ++i) {
                lua_rawgeti(L, -1, i);
                const int st = lua_gettop(L);
                const std::string where = at + ".stops[" + std::to_string(i) + "]";
                citysim::CrowdStop s;
                if (lua_istable(L, st)) {
                    s.kind = strField(L, st, "kind");
                    s.home = boolField(L, st, "home", false);
                    s.beyond = numField(L, st, "beyond", 0);
                    lua_getfield(L, st, "minutes");
                    if (lua_istable(L, -1)) {
                        lua_rawgeti(L, -1, 1);
                        lua_rawgeti(L, -2, 2);
                        s.minutesLo = lua_isnumber(L, -2) ? lua_tonumber(L, -2) : -1;
                        s.minutesHi = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : -1;
                        lua_pop(L, 2);
                    }
                    lua_pop(L, 1);
                }
                lua_pop(L, 1);
                if (s.kind.empty() == !s.home) return refuse(where + " needs a `kind` or `home = true`, not both");
                if (s.minutesLo < 0 || s.minutesHi < s.minutesLo) return refuse(where + ": minutes = {lo, hi}, 0 <= lo <= hi");
                if (s.beyond < 0) return refuse(where + ": beyond cannot be negative");
                d.stops.push_back(s);
            }
            lua_pop(L, 1);
            if (d.stops.empty()) return refuse(at + " has no stops");
            spec.days.push_back(d);
            return true;
        }))
        return fail(bad.empty() ? "days: a list is needed" : bad);
    lua_settop(L, base);
    out = std::move(spec);
    return true;
}

}  // namespace engine
