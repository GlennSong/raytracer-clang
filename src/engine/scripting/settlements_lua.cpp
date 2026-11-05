#include "settlements_lua.h"

#include "lua_helpers.h"
#include "script_vm.h"
#include "../script_assets.h"

namespace engine {

namespace {

bool fail(std::string* err, const std::string& why) { if (err) *err = why; return false; }

// t[key] = { a, b, ... } read into `out` (as many as it has, up to n); false when it is there but not a list
bool numbers(lua_State* L, int t, const char* key, double* out, int n) {
    lua_getfield(L, t, key);
    const bool absent = lua_isnil(L, -1), ok = absent || lua_istable(L, -1);
    if (lua_istable(L, -1))
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, -1, i);
            if (lua_isnumber(L, -1)) out[i - 1] = lua_tonumber(L, -1);
            lua_pop(L, 1);
        }
    lua_pop(L, 1);
    return ok;
}

}  // namespace

bool loadSettlementCatalog(ScriptVM& vm, SettlementCatalog& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    auto bail = [&](const std::string& why) { lua_settop(L, base); return fail(err, "settlements.lua: " + why); };
    lua_getglobal(L, "settlements");
    if (!lua_istable(L, -1)) return bail("no global `settlements` table");
    const int st = lua_gettop(L);
    const std::vector<std::string> names = stringsField(L, st, "order");
    if (names.empty()) return bail("`settlements.order` must name the kinds");
    SettlementCatalog c;
    for (const std::string& name : names) {
        lua_getfield(L, st, name.c_str());
        if (!lua_istable(L, -1)) return bail("settlements[\"" + name + "\"]: not a table");
        const int t = lua_gettop(L);
        SettlementKind k;
        k.name = name;
        if (const std::string hub = strField(L, t, "hub"); !hub.empty()) k.hub = hub;
        double radius[3] = {k.radiusScale, k.radiusMin, k.radiusMax};
        if (!numbers(L, t, "radius", radius, 3)) return bail(name + ".radius = { scale, min, max }");
        k.radiusScale = radius[0]; k.radiusMin = radius[1]; k.radiusMax = radius[2];
        k.coastReach = numField(L, t, "coast_reach", k.coastReach);
        k.onRing = boolField(L, t, "on_ring", k.onRing);
        k.passEnd = boolField(L, t, "pass_end", k.passEnd);
        k.mountainRoad = boolField(L, t, "mountain_road", k.mountainRoad);
        k.highRoad = boolField(L, t, "high_road", k.highRoad);
        k.interchanges = static_cast<int>(numField(L, t, "interchanges", k.interchanges));
        double links[2] = {static_cast<double>(k.linkCandidates), k.linkSpacing};
        if (!numbers(L, t, "links", links, 2)) return bail(name + ".links = { candidates, spacing }");
        k.linkCandidates = static_cast<int>(links[0]); k.linkSpacing = links[1];
        double buses[2] = {static_cast<double>(k.busRoutes), static_cast<double>(k.busStops)};
        if (!numbers(L, t, "buses", buses, 2)) return bail(name + ".buses = { routes, stops }");
        k.busRoutes = static_cast<int>(buses[0]); k.busStops = static_cast<int>(buses[1]);
        k.signedFar = boolField(L, t, "signed_far", k.signedFar);
        if (const std::string ls = strField(L, t, "limit_sign"); !ls.empty()) k.limitSign = ls;
        if (const std::string nm = strField(L, t, "naming"); !nm.empty()) k.naming = nm;
        lua_getfield(L, t, "brief");
        if (lua_istable(L, -1)) {
            const int b = lua_gettop(L);
            if (const std::string px = strField(L, b, "prefix"); !px.empty()) k.briefPrefix = px;
            k.areaScale = numField(L, b, "area_scale", k.areaScale);
            k.sizeMin = numField(L, b, "size_min", k.sizeMin);
            k.spokeSpacing = numField(L, b, "spoke_spacing", k.spokeSpacing);
            if (!numbers(L, b, "core_block", k.coreBlock, 2) || !numbers(L, b, "midtown_block", k.midtownBlock, 2))
                return bail(name + ".brief: blocks are { long, short }");
            k.warp = numField(L, b, "warp", k.warp);
            k.ringSpacing = numField(L, b, "ring_spacing", k.ringSpacing);
            k.spokes = static_cast<int>(numField(L, b, "spokes", k.spokes));
        }
        lua_pop(L, 1);   // brief
        lua_getfield(L, t, "strip");
        if (lua_istable(L, -1)) {
            const int sp = lua_gettop(L);
            k.strip = true;
            k.stripLength = numField(L, sp, "length", k.stripLength);
            k.stripRows = static_cast<int>(numField(L, sp, "rows", k.stripRows));
            k.stripRow = numField(L, sp, "row", k.stripRow);
            k.stripCross = numField(L, sp, "cross", k.stripCross);
            k.stripSea = numField(L, sp, "sea", k.stripSea);
            k.stripPromenade = numField(L, sp, "promenade", k.stripPromenade);
            if (const std::string al = strField(L, sp, "along"); !al.empty()) k.stripAlong = al;
            k.stripBoth = boolField(L, sp, "both_sides", k.stripBoth);
            if (const std::string mn = strField(L, sp, "main"); !mn.empty()) k.stripMain = mn;
        }
        lua_pop(L, 1);   // strip
        lua_getfield(L, t, "regrade");   // { blur, max_slope, terrace, bank, feather, margin } (procgen/regrade.h)
        if (lua_istable(L, -1)) {
            const int rg = lua_gettop(L);
            k.regrade = true;
            k.regradeBlur = numField(L, rg, "blur", k.regradeBlur);
            k.regradeMaxSlope = numField(L, rg, "max_slope", k.regradeMaxSlope);
            k.regradeTerrace = numField(L, rg, "terrace", k.regradeTerrace);
            k.regradeBank = numField(L, rg, "bank", k.regradeBank);
            k.regradeFeather = numField(L, rg, "feather", k.regradeFeather);
            k.regradeMargin = numField(L, rg, "margin", k.regradeMargin);
        }
        lua_pop(L, 1);   // regrade
        k.hubReach = numField(L, t, "hub_reach", k.hubReach);
        lua_getfield(L, t, "mixes");   // { <district> = { { "recipe", weight }, ... } }, in the district order below
        if (lua_istable(L, -1)) {
            const int mx = lua_gettop(L);
            for (const char* d : {"financial", "commercial", "residential", "oldtown", "industrial"}) {
                lua_getfield(L, mx, d);
                if (lua_istable(L, -1)) {
                    const lua_Integer n = luaL_len(L, -1);
                    std::vector<std::pair<std::string, double>> es;
                    for (lua_Integer i = 1; i <= n; ++i) {
                        lua_rawgeti(L, -1, i);
                        lua_rawgeti(L, -1, 1);
                        lua_rawgeti(L, -2, 2);
                        if (!lua_isstring(L, -2) || !lua_isnumber(L, -1)) return bail(name + ".mixes." + d + ": { \"recipe\", weight } pairs");
                        es.push_back({lua_tostring(L, -2), lua_tonumber(L, -1)});
                        lua_pop(L, 3);
                    }
                    k.mixes[d] = std::move(es);
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);   // mixes
        lua_getfield(L, t, "civic");
        if (lua_isboolean(L, -1)) k.civic = lua_toboolean(L, -1) != 0;
        lua_pop(L, 1);
        lua_getfield(L, t, "landmarks");   // { { "school", n, min_lots = n, min_old_town = n, any = bool }, ... }
        if (lua_istable(L, -1)) {
            const int lt = lua_gettop(L);
            k.landmarks.clear();
            const lua_Integer n = luaL_len(L, lt);
            for (lua_Integer i = 1; i <= n; ++i) {
                lua_rawgeti(L, lt, i);
                const int e = lua_gettop(L);
                SettlementLandmark lm;
                lua_rawgeti(L, e, 1);
                lua_rawgeti(L, e, 2);
                if (!lua_istable(L, e) || !lua_isstring(L, -2)) return bail(name + ".landmarks: { \"landmark\", count, ... } entries");
                lm.name = lua_tostring(L, -2);
                if (lua_isnumber(L, -1)) lm.count = static_cast<int>(lua_tointeger(L, -1));
                lua_pop(L, 2);
                lm.minLots = static_cast<int>(numField(L, e, "min_lots", lm.minLots));
                lm.minOldTown = static_cast<int>(numField(L, e, "min_old_town", lm.minOldTown));
                lua_getfield(L, e, "any");
                if (lua_isboolean(L, -1)) lm.any = lua_toboolean(L, -1) != 0;
                lua_pop(L, 2);   // any, the entry
                k.landmarks.push_back(std::move(lm));
            }
        }
        lua_pop(L, 1);   // landmarks
        lua_getfield(L, t, "map");
        if (lua_istable(L, -1)) {
            const int m = lua_gettop(L);
            double rgb[3] = {k.rgb[0], k.rgb[1], k.rgb[2]};
            if (!numbers(L, m, "rgb", rgb, 3)) return bail(name + ".map.rgb = { r, g, b }");
            for (int i = 0; i < 3; ++i) k.rgb[i] = static_cast<float>(rgb[i]);
            k.dot = numField(L, m, "dot", k.dot);
            k.label = numField(L, m, "label", k.label);
        }
        lua_pop(L, 1);   // map
        lua_pop(L, 1);   // the kind
        c.kinds.push_back(std::move(k));
    }
    lua_settop(L, base);
    if (const std::string wrong = c.check(); !wrong.empty()) return fail(err, "settlements.lua: " + wrong);
    out = std::move(c);
    return true;
}

SettlementCatalog shippedSettlements(std::string* warn) {
    const std::string code = loadScriptCode("settlements.lua", "");
    if (code.empty()) {
        if (warn) *warn = "settlements.lua: not found (the built-in kinds)";
        return defaultSettlements();
    }
    ScriptVM vm;
    std::string err;
    SettlementCatalog c;
    if (vm.doString(code, &err) && loadSettlementCatalog(vm, c, &err)) return c;
    if (warn) *warn = err + " (the built-in kinds)";
    return defaultSettlements();
}

}  // namespace engine
