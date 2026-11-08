#include "clouds_lua.h"
#include "lua_state.h"     // the Lua C API (scripting-internal)
#include "lua_helpers.h"   // numField / strField
#include "script_vm.h"
#include "../script_assets.h"
#include "../../log.h"
#include <mutex>

namespace engine {

namespace {
// a list of exactly `n` numbers at t[key] into `v`; absent: untouched (true); otherwise false
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
}  // namespace

bool loadCloudsSource(const std::string& code, CloudBook& out, std::string* err) {
    ScriptVM vm;
    lua_State* L = luaState(vm);
    auto fail = [&](const std::string& why) { if (err) *err = "clouds.lua: " + why; return false; };
    if (luaL_loadstring(L, code.c_str()) != LUA_OK || lua_pcall(L, 0, 1, 0) != LUA_OK)
        return fail(lua_tostring(L, -1) ? lua_tostring(L, -1) : "did not run");
    if (!lua_istable(L, -1)) return fail("returns no table");
    const int root = lua_gettop(L);
    CloudBook book;
    book.shapeScale = static_cast<float>(numField(L, root, "shape_scale", book.shapeScale));
    book.shadow = static_cast<float>(numField(L, root, "shadow", book.shadow));
    if (book.shadow < 0.0f || book.shadow > 1.0f) return fail("shadow: 0..1");
    lua_getfield(L, root, "map");
    if (lua_istable(L, -1)) {
        const int m = lua_gettop(L);
        book.mapExtent = static_cast<float>(numField(L, m, "extent", book.mapExtent));
        book.mapSize = static_cast<int>(numField(L, m, "size", book.mapSize));
        book.rebuildMetres = static_cast<float>(numField(L, m, "rebuild_m", book.rebuildMetres));
    }
    lua_pop(L, 1);
    if (book.mapExtent < 1000.0f || book.mapSize < 64 || book.mapSize > 4096) return fail("map: extent >= 1000, size 64..4096");
    lua_getfield(L, root, "types");
    if (!lua_istable(L, -1) || luaL_len(L, -1) != 4) return fail("types: exactly four, mild to severe");
    const int types = lua_gettop(L);
    for (int i = 0; i < 4; ++i) {
        lua_rawgeti(L, types, i + 1);
        if (!lua_istable(L, -1)) return fail("types[" + std::to_string(i + 1) + "] is not a table");
        const int t = lua_gettop(L);
        CloudTypeSpec& c = book.types[static_cast<std::size_t>(i)];
        c.name = strField(L, t, "name");
        if (c.name.empty()) c.name = "type" + std::to_string(i + 1);
        c.base = static_cast<float>(numField(L, t, "base", c.base));
        c.top = static_cast<float>(numField(L, t, "top", c.top));
        c.density = static_cast<float>(numField(L, t, "density", c.density));
        c.erosion = static_cast<float>(numField(L, t, "erosion", c.erosion));
        c.anvil = static_cast<float>(numField(L, t, "anvil", c.anvil));
        c.aspect = static_cast<float>(numField(L, t, "aspect", c.aspect));
        c.low = static_cast<float>(numField(L, t, "low", c.low));
        float r[2] = {c.radiusMin, c.radiusMax};
        if (!numbers(L, t, "profile", c.profile.data(), 4) || !numbers(L, t, "radius", r, 2))
            return fail("type `" + c.name + "`: profile is four numbers, radius two");
        c.radiusMin = r[0];
        c.radiusMax = r[1];
        if (!(c.top > c.base) || !(c.radiusMin > 0.0f && c.radiusMax >= c.radiusMin) || c.aspect < 1.0f ||
            !(c.low > 0.0f && c.low <= 1.0f))
            return fail("type `" + c.name + "`: top > base, 0 < radius min <= max, aspect >= 1, 0 < low <= 1");
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    book.loaded = true;
    out = std::move(book);
    return true;
}

void ensureCloudsLoaded() {
    static std::once_flag once;
    std::call_once(once, [] {
        const std::string code = loadScriptCode("clouds.lua", "");
        if (code.empty()) { LOG_WARN << "[clouds] no clouds.lua: one cumulus type stands"; return; }
        CloudBook book;
        std::string err;
        if (!loadCloudsSource(code, book, &err)) { LOG_WARN << "[clouds] " << err << " (one cumulus type stands)"; return; }
        setCloudBook(book);
        LOG_INFO << "[clouds] 4 types, a " << book.mapExtent / 1000.0f << " km map (clouds.lua)";
    });
}

}  // namespace engine
