#include "lights_lua.h"
#include "lua_state.h"     // the Lua C API (scripting-internal)
#include "lua_helpers.h"   // numField / strField / boolField
#include "script_vm.h"
#include "../script_assets.h"
#include "../../log.h"
#include <algorithm>
#include <mutex>

namespace engine {

namespace {

// a list of up to `n` numbers at t[key] into `v`; false when present but not a table of n numbers
bool numbers(lua_State* L, int t, const char* key, float* v, int n) {
    lua_getfield(L, t, key);
    if (lua_isnil(L, -1)) { lua_pop(L, 1); return true; }
    bool ok = lua_istable(L, -1) && luaL_len(L, -1) == n;
    for (int i = 0; ok && i < n; ++i) {
        lua_rawgeti(L, -1, i + 1);
        ok = lua_isnumber(L, -1);
        if (ok) v[i] = static_cast<float>(lua_tonumber(L, -1));
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return ok;
}

// every string key of the table at `t`, sorted (Lua's own order is not stable)
std::vector<std::string> keysOf(lua_State* L, int t) {
    std::vector<std::string> keys;
    lua_pushnil(L);
    while (lua_next(L, t) != 0) {
        if (lua_type(L, -2) == LUA_TSTRING) keys.push_back(lua_tostring(L, -2));
        lua_pop(L, 1);
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}

}  // namespace

bool loadLightsSource(const std::string& code, LightBook& out, std::string* err) {
    ScriptVM vm;
    lua_State* L = luaState(vm);
    auto fail = [&](const std::string& why) { if (err) *err = "lights.lua: " + why; return false; };
    if (luaL_loadstring(L, code.c_str()) != LUA_OK || lua_pcall(L, 0, 1, 0) != LUA_OK)
        return fail(lua_tostring(L, -1) ? lua_tostring(L, -1) : "did not run");
    if (!lua_istable(L, -1)) return fail("returns no table");
    const int root = lua_gettop(L);
    LightBook book;
    book.emissionScale = static_cast<float>(numField(L, root, "emission_scale", 1.0));
    book.cityGlowNits = static_cast<float>(numField(L, root, "city_glow_nits", 0.0));
    book.moonNits = static_cast<float>(numField(L, root, "moon_nits", 2500.0));
    lua_getfield(L, root, "pools");
    if (lua_istable(L, -1)) {
        const int pt = lua_gettop(L);
        for (const std::string& name : keysOf(L, pt)) {
            lua_getfield(L, pt, name.c_str());
            if (!lua_istable(L, -1)) return fail("pool `" + name + "` is not a table");
            const int p = lua_gettop(L);
            LightPool pool;
            pool.name = name;
            pool.count = static_cast<int>(numField(L, p, "count", 0));
            pool.within = static_cast<float>(numField(L, p, "within", 0));
            pool.fade = boolField(L, p, "fade", false);
            pool.groups = boolField(L, p, "groups", false);
            lua_getfield(L, p, "many");
            if (lua_istable(L, -1)) {
                const int m = lua_gettop(L);
                pool.hasMany = true;
                pool.manyCount = static_cast<int>(numField(L, m, "count", pool.count));
                pool.manyWithin = static_cast<float>(numField(L, m, "within", pool.within));
                pool.manyFade = boolField(L, m, "fade", pool.fade);
            }
            lua_pop(L, 1);
            if (pool.count < 0 || pool.within < 0 || pool.manyCount < 0 || pool.manyWithin < 0)
                return fail("pool `" + name + "`: count and within are >= 0");
            book.pools.push_back(pool);
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    lua_getfield(L, root, "glows");
    if (lua_istable(L, -1)) {
        const int gt = lua_gettop(L);
        for (const std::string& name : keysOf(L, gt)) {
            lua_getfield(L, gt, name.c_str());
            if (!lua_istable(L, -1)) return fail("glow `" + name + "` is not a table");
            LightBook::Glow g;
            g.name = name;
            g.nits = static_cast<float>(numField(L, lua_gettop(L), "nits", 0.0));
            if (g.nits < 0.0f) return fail("glow `" + name + "`: nits >= 0");
            book.glows.push_back(g);
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    lua_getfield(L, root, "kinds");
    if (!lua_istable(L, -1)) return fail("no `kinds` table");
    const int kt = lua_gettop(L);
    for (const std::string& name : keysOf(L, kt)) {
        lua_getfield(L, kt, name.c_str());
        if (!lua_istable(L, -1)) return fail("kind `" + name + "` is not a table");
        const int t = lua_gettop(L);
        LightKind k;
        k.name = name;
        const std::string type = strField(L, t, "type");
        if (type != "point" && type != "spot") return fail("kind `" + name + "`: type is point or spot");
        k.spot = type == "spot";
        float c[3] = {1, 1, 1}, o[3] = {0, 0, 0}, cone[2] = {k.inner, k.outer};
        if (!numbers(L, t, "color", c, 3)) return fail("kind `" + name + "`: color is { r, g, b }");
        if (!numbers(L, t, "offset", o, 3)) return fail("kind `" + name + "`: offset is { x, y, z }");
        if (!numbers(L, t, "cone", cone, 2)) return fail("kind `" + name + "`: cone is { inner, outer }");
        k.color = Vec3(c[0], c[1], c[2]);
        k.offset = Vec3(o[0], o[1], o[2]);
        k.inner = cone[0];
        k.outer = cone[1];
        k.intensity = static_cast<float>(numField(L, t, "intensity", 1.0));
        k.candela = static_cast<float>(numField(L, t, "candela", 0.0));
        k.range = static_cast<float>(numField(L, t, "range", 25.0));
        k.dusk = boolField(L, t, "dusk", false);
        const std::string pool = strField(L, t, "pool");
        if (!pool.empty()) {
            k.pool = book.pool(pool);
            if (k.pool < 0) return fail("kind `" + name + "`: no pool `" + pool + "`");
        }
        if (k.range <= 0 || k.intensity < 0) return fail("kind `" + name + "`: range > 0, intensity >= 0");
        if (k.spot && !(k.inner > 0 && k.inner < k.outer)) return fail("kind `" + name + "`: cone needs 0 < inner < outer");
        book.kinds.push_back(k);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    if (book.kinds.empty()) return fail("no kinds");
    out = std::move(book);
    return true;
}

void ensureLightsLoaded() {
    static std::once_flag once;
    std::call_once(once, [] {
        const std::string code = loadScriptCode("lights.lua", "");
        if (code.empty()) { LOG_WARN << "[lights] no lights.lua: one plain white light kind stands"; return; }
        LightBook book;
        std::string err;
        if (!loadLightsSource(code, book, &err)) { LOG_WARN << "[lights] " << err << " (one plain white light kind stands)"; return; }
        setLightBook(book);
        LOG_INFO << "[lights] " << book.kinds.size() << " kinds, " << book.pools.size() << " pools (lights.lua)";
    });
}

}  // namespace engine
