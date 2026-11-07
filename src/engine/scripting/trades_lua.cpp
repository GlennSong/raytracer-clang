#include "trades_lua.h"
#include "lua_state.h"     // the Lua C API (scripting-internal)
#include "lua_helpers.h"   // numField / strField / boolField
#include "script_vm.h"
#include "../script_assets.h"
#include "../../log.h"
#include <mutex>

namespace engine {

namespace {

const char* const kLights[] = {"panels", "round", "dim", "none"};
const char* const kMixes[] = {"everyday", "high_street", "nightlife", "mall"};

Vec3 colourAt(lua_State* L, int idx, const Vec3& def) {
    if (!lua_istable(L, idx)) return def;
    Real c[3] = {def.x, def.y, def.z};
    for (int i = 0; i < 3; ++i) {
        lua_rawgeti(L, idx, i + 1);
        if (lua_isnumber(L, -1)) c[i] = lua_tonumber(L, -1);
        lua_pop(L, 1);
    }
    return Vec3(c[0], c[1], c[2]);
}
Vec3 colourField(lua_State* L, int t, const char* key, const Vec3& def) {
    lua_getfield(L, t, key);
    const Vec3 v = colourAt(L, lua_gettop(L), def);
    lua_pop(L, 1);
    return v;
}

// room = { walls = {{r,g,b} x3}, dado = { height, color }, floor = {r,g,b}, checker = { color, size }, ceiling, lights }
bool readRoom(lua_State* L, int t, TradeInterior& in, std::string& why) {
    lua_getfield(L, t, "walls");
    if (!lua_istable(L, -1) || luaL_len(L, -1) != 3) { lua_pop(L, 1); why = "walls must be three colours"; return false; }
    for (int i = 0; i < 3; ++i) {
        lua_rawgeti(L, -1, i + 1);
        in.walls[i] = colourAt(L, lua_gettop(L), Vec3(0.9, 0.9, 0.9));
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    lua_getfield(L, t, "dado");
    if (lua_istable(L, -1)) {
        const int d = lua_gettop(L);
        in.dadoH = numField(L, d, "height", 0.0);
        in.dado = colourField(L, d, "color", Vec3());
    }
    lua_pop(L, 1);
    in.floorA = colourField(L, t, "floor", Vec3(0.55, 0.55, 0.56));
    lua_getfield(L, t, "checker");
    if (lua_istable(L, -1)) {
        const int c = lua_gettop(L);
        in.floorB = colourField(L, c, "color", Vec3());
        in.tile = numField(L, c, "size", 0.0);
    }
    lua_pop(L, 1);
    in.ceiling = colourField(L, t, "ceiling", Vec3(0.92, 0.92, 0.90));
    const std::string lights = strField(L, t, "lights");
    in.lights = 0;
    bool known = lights.empty();
    for (int i = 0; i < 4; ++i) if (lights == kLights[i]) { in.lights = i; known = true; }
    if (!known) { why = "lights `" + lights + "` is not panels, round, dim or none"; return false; }
    return true;
}

}  // namespace

bool loadTradesSource(const std::string& code, TradeBook& out, std::string* err) {
    ScriptVM vm;
    lua_State* L = luaState(vm);
    auto fail = [&](const std::string& why) { if (err) *err = "trades.lua: " + why; return false; };
    if (luaL_loadstring(L, code.c_str()) != LUA_OK || lua_pcall(L, 0, 1, 0) != LUA_OK)
        return fail(lua_tostring(L, -1) ? lua_tostring(L, -1) : "did not run");
    if (!lua_istable(L, -1)) return fail("returns no table");
    const int root = lua_gettop(L);
    TradeBook book;
    std::string why;
    lua_getfield(L, root, "plain_room");
    if (!lua_istable(L, -1) || !readRoom(L, lua_gettop(L), book.plain, why)) return fail("plain_room: " + (why.empty() ? "missing" : why));
    lua_pop(L, 1);
    lua_getfield(L, root, "trades");
    if (!lua_istable(L, -1)) return fail("no `trades` list");
    const int tl = lua_gettop(L), n = static_cast<int>(luaL_len(L, tl));
    for (int i = 1; i <= n; ++i) {
        lua_rawgeti(L, tl, i);
        const int t = lua_gettop(L);
        TradeInfo tr;
        tr.id = static_cast<uint8_t>(numField(L, t, "id", -1));
        tr.name = strField(L, t, "name");
        tr.placeType = strField(L, t, "place");
        if (tr.name.empty() || tr.placeType.empty()) return fail("trade " + std::to_string(i) + " needs a name and a place");
        for (const TradeInfo& o : book.trades)
            if (o.id == tr.id || o.name == tr.name) return fail("trade `" + tr.name + "`: its id or name is taken");
        lua_getfield(L, t, "hours");
        if (!lua_istable(L, -1)) return fail("trade `" + tr.name + "` needs hours = { open, close }");
        lua_rawgeti(L, -1, 1); tr.openHour = static_cast<float>(lua_tonumber(L, -1)); lua_pop(L, 1);
        lua_rawgeti(L, -1, 2); tr.closeHour = static_cast<float>(lua_tonumber(L, -1)); lua_pop(L, 1);
        lua_pop(L, 1);
        lua_getfield(L, t, "weight");
        if (lua_istable(L, -1))
            for (int m = 0; m < 4; ++m) tr.weight[m] = static_cast<int>(numField(L, lua_gettop(L), kMixes[m], 0));
        lua_pop(L, 1);
        lua_getfield(L, t, "bays");
        if (!lua_istable(L, -1)) return fail("trade `" + tr.name + "` needs bays = { min, max }");
        lua_rawgeti(L, -1, 1); tr.minBays = static_cast<int>(lua_tonumber(L, -1)); lua_pop(L, 1);
        lua_rawgeti(L, -1, 2); tr.maxBays = static_cast<int>(lua_tonumber(L, -1)); lua_pop(L, 1);
        lua_pop(L, 1);
        if (tr.minBays < 1 || tr.maxBays < tr.minBays) return fail("trade `" + tr.name + "`: bays must be 1 <= min <= max");
        tr.fascia = colourField(L, t, "fascia", tr.fascia);
        tr.terrace = boolField(L, t, "terrace", false);
        lua_getfield(L, t, "room");
        if (lua_istable(L, -1)) {
            TradeBook::Room r;
            r.id = tr.id;
            if (!readRoom(L, lua_gettop(L), r.in, why)) return fail("trade `" + tr.name + "` room: " + why);
            book.rooms.push_back(r);
        }
        lua_pop(L, 1);
        book.trades.push_back(tr);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    // rooms that are not trades (the malls' concourses), by style id
    lua_getfield(L, root, "rooms");
    if (lua_istable(L, -1)) {
        const int rl = lua_gettop(L), m = static_cast<int>(luaL_len(L, rl));
        for (int i = 1; i <= m; ++i) {
            lua_rawgeti(L, rl, i);
            TradeBook::Room r;
            r.id = static_cast<uint8_t>(numField(L, lua_gettop(L), "id", 0));
            if (!readRoom(L, lua_gettop(L), r.in, why)) return fail("room " + std::to_string(r.id) + ": " + why);
            book.rooms.push_back(r);
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    if (book.trades.empty()) return fail("no trades");
    out = std::move(book);
    return true;
}

void ensureTradesLoaded() {
    static std::once_flag once;
    std::call_once(once, [] {
        const std::string code = loadScriptCode("trades.lua", "");
        if (code.empty()) { LOG_WARN << "[trades] no trades.lua: the built-in trades stand"; return; }
        TradeBook book;
        std::string err;
        if (!loadTradesSource(code, book, &err)) { LOG_WARN << "[trades] " << err << " (the built-in trades stand)"; return; }
        setTradeBook(book);
        LOG_INFO << "[trades] " << book.trades.size() << " trades, " << book.rooms.size() << " rooms (trades.lua)";
    });
}

}  // namespace engine
