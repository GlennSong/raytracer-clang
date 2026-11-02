#include "people_lua.h"

#include "../../../engine/scripting/lua_helpers.h"
#include "../../../engine/scripting/script_vm.h"

namespace engine {

namespace {

bool fail(std::string* err, const std::string& why) { if (err) *err = why; return false; }

// The { <name> = share } table at t[key], in the order of `names` (lua_next's order is not one); a name not in
// `names` is an error naming it.
bool shares(lua_State* L, int t, const char* key, const std::vector<std::string>& names,
            std::vector<std::pair<int, double>>& out, std::string& unknown) {
    out.clear();
    lua_getfield(L, t, key);
    if (!lua_istable(L, -1)) { lua_pop(L, 1); return true; }
    const int st = lua_gettop(L);
    for (std::size_t i = 0; i < names.size(); ++i) {
        const double s = numField(L, st, names[i].c_str(), 0);
        if (s > 0) out.push_back({static_cast<int>(i), s});
    }
    lua_pushnil(L);
    while (lua_next(L, st)) {
        lua_pop(L, 1);
        const std::string k = lua_type(L, -1) == LUA_TSTRING ? lua_tostring(L, -1) : "?";
        bool known = false;
        for (const std::string& n : names) known = known || n == k;
        if (!known && unknown.empty()) unknown = k;
    }
    lua_pop(L, 1);
    return unknown.empty();
}

// t[key] as { lo, hi } or one number for both; false when it is neither.
bool range(lua_State* L, int t, const char* key, int& lo, int& hi) {
    lua_getfield(L, t, key);
    bool ok = true;
    if (lua_isnumber(L, -1)) {
        lo = hi = static_cast<int>(lua_tointeger(L, -1));
    } else if (lua_istable(L, -1)) {
        lua_rawgeti(L, -1, 1);
        lua_rawgeti(L, -2, 2);
        ok = lua_isnumber(L, -2) && lua_isnumber(L, -1);
        if (ok) { lo = static_cast<int>(lua_tointeger(L, -2)); hi = static_cast<int>(lua_tointeger(L, -1)); }
        lua_pop(L, 2);
    } else {
        ok = false;
    }
    lua_pop(L, 1);
    return ok;
}

}  // namespace

bool loadPeopleCatalog(ScriptVM& vm, citysim::PeopleCatalog& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    auto bail = [&](const std::string& why) { lua_settop(L, base); return fail(err, "people.lua: " + why); };
    lua_getglobal(L, "people");
    if (!lua_istable(L, -1)) return bail("no global `people` table");
    const int pt = lua_gettop(L);
    citysim::PeopleCatalog c;

    // a section's table and its order list
    auto section = [&](const char* name, std::vector<std::string>& order) {
        lua_getfield(L, pt, name);
        if (!lua_istable(L, -1)) return false;
        order = stringsField(L, lua_gettop(L), "order");
        return !order.empty();
    };
    std::vector<std::string> stageNames, bandNames, typeNames;
    if (!section("stages", stageNames)) return bail("`people.stages` needs an order");
    for (const std::string& n : stageNames) {
        lua_getfield(L, -1, n.c_str());
        if (!lua_istable(L, -1)) return bail("stages." + n + ": not a table");
        const int t = lua_gettop(L);
        citysim::LifeStage s;
        s.name = n;
        s.job = boolField(L, t, "job", false);
        s.walks = boolField(L, t, "walks", false);
        s.beds = boolField(L, t, "beds", false);
        s.role = strField(L, t, "role");
        c.stages.push_back(s);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);   // stages

    if (!section("bands", bandNames)) return bail("`people.bands` needs an order");
    for (const std::string& n : bandNames) {
        lua_getfield(L, -1, n.c_str());
        if (!lua_istable(L, -1)) return bail("bands." + n + ": not a table");
        const int t = lua_gettop(L);
        citysim::AgeBand b;
        b.name = n;
        if (!range(L, t, "ages", b.lo, b.hi)) return bail("bands." + n + ": ages = { lo, hi }");
        std::string unknown;
        if (!shares(L, t, "stages", stageNames, b.stages, unknown)) return bail("bands." + n + ": no stage `" + unknown + "`");
        c.bands.push_back(std::move(b));
        lua_pop(L, 1);
    }
    lua_pop(L, 1);   // bands

    // THE TIES (before the households, which name them)
    std::vector<std::string> tieNames;
    if (section("ties", tieNames)) {
        std::vector<std::string> others;
        for (const std::string& n : tieNames) {
            lua_getfield(L, -1, n.c_str());
            if (!lua_istable(L, -1)) return bail("ties." + n + ": not a table");
            const int t = lua_gettop(L);
            citysim::Tie tie;
            tie.name = n;
            tie.older = static_cast<int>(numField(L, t, "older", 0));
            others.push_back(strField(L, t, "other"));
            c.ties.push_back(tie);
            lua_pop(L, 1);
        }
        for (std::size_t i = 0; i < c.ties.size(); ++i) {
            c.ties[i].other = c.tie(others[i]);
            if (c.ties[i].other < 0) return bail("ties." + tieNames[i] + ": no tie `" + others[i] + "` for its other side");
        }
    }
    lua_pop(L, 1);   // ties

    if (!section("households", typeNames)) return bail("`people.households` needs an order");
    for (const std::string& n : typeNames) {
        lua_getfield(L, -1, n.c_str());
        if (!lua_istable(L, -1)) return bail("households." + n + ": not a table");
        const int t = lua_gettop(L);
        citysim::HouseholdType h;
        h.name = n;
        h.share = numField(L, t, "share", 0);
        if (!range(L, t, "beds", h.bedsLo, h.bedsHi)) return bail("households." + n + ": beds = { lo, hi }");
        lua_getfield(L, t, "members");
        if (lua_istable(L, -1)) {
            const lua_Integer m = luaL_len(L, -1);
            for (lua_Integer i = 1; i <= m; ++i) {
                lua_rawgeti(L, -1, i);
                if (!lua_istable(L, -1)) return bail("households." + n + ": a member group is not a table");
                const int g = lua_gettop(L);
                citysim::MemberGroup grp;
                if (!range(L, g, "count", grp.lo, grp.hi)) return bail("households." + n + ": count = n or { lo, hi }");
                grp.perBed = boolField(L, g, "per_bed", false);
                std::string unknown;
                if (!shares(L, g, "bands", bandNames, grp.bands, unknown)) return bail("households." + n + ": no band `" + unknown + "`");
                h.groups.push_back(std::move(grp));
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);   // members
        // its ties: { from, "<tie>", to } -- group `from` is the <tie> of group `to` (groups count from 1)
        lua_getfield(L, t, "ties");
        if (lua_istable(L, -1)) {
            const lua_Integer m = luaL_len(L, -1);
            for (lua_Integer i = 1; i <= m; ++i) {
                lua_rawgeti(L, -1, i);
                if (!lua_istable(L, -1)) return bail("households." + n + ": a tie is not { from, \"<tie>\", to }");
                lua_rawgeti(L, -1, 1);
                lua_rawgeti(L, -2, 2);
                lua_rawgeti(L, -3, 3);
                if (!lua_isnumber(L, -3) || !lua_isstring(L, -2) || !lua_isnumber(L, -1))
                    return bail("households." + n + ": a tie is not { from, \"<tie>\", to }");
                citysim::HouseholdTie ht;
                ht.from = static_cast<int>(lua_tointeger(L, -3)) - 1;
                ht.to = static_cast<int>(lua_tointeger(L, -1)) - 1;
                const std::string tn = lua_tostring(L, -2);
                ht.tie = c.tie(tn);
                if (ht.tie < 0) return bail("households." + n + ": no tie `" + tn + "`");
                h.ties.push_back(ht);
                lua_pop(L, 4);
            }
        }
        lua_pop(L, 1);   // ties
        c.households.push_back(std::move(h));
        lua_pop(L, 1);
    }
    lua_pop(L, 1);   // households

    lua_getfield(L, pt, "halls");
    if (lua_istable(L, -1)) {
        const int t = lua_gettop(L);
        c.hallBand = c.band(strField(L, t, "band"));
        c.hallStage = c.stage(strField(L, t, "stage"));
        c.hallShare = numField(L, t, "share_of_walkers", c.hallShare);
    }
    lua_pop(L, 1);   // halls
    {
        std::string unknown;
        if (!shares(L, pt, "lodgers", bandNames, c.lodgers, unknown)) return bail("lodgers: no band `" + unknown + "`");
    }
    lua_settop(L, base);
    if (const std::string wrong = c.check(); !wrong.empty()) return fail(err, "people.lua: " + wrong);
    out = std::move(c);
    return true;
}

}  // namespace engine
