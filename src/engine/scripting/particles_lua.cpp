#include "particles_lua.h"
#include "lua_state.h"
#include "lua_helpers.h"
#include "script_vm.h"
#include "../script_assets.h"
#include "../../log.h"
#include <mutex>
#include <set>

namespace engine {

namespace {
bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

// a {a, b} pair field (life = {min, max}, size = {start, end}); a lone number sets both
bool pairField(lua_State* L, int t, const char* key, Real& a, Real& b) {
    lua_getfield(L, t, key);
    bool ok = true;
    if (lua_isnumber(L, -1)) {
        a = b = lua_tonumber(L, -1);
    } else if (lua_istable(L, -1)) {
        const int pt = lua_gettop(L);
        lua_rawgeti(L, pt, 1);
        lua_rawgeti(L, pt, 2);
        ok = lua_isnumber(L, -2) && lua_isnumber(L, -1);
        if (ok) { a = lua_tonumber(L, -2); b = lua_tonumber(L, -1); }
        lua_pop(L, 2);
    } else if (!lua_isnil(L, -1)) {
        ok = false;
    }
    lua_pop(L, 1);
    return ok;
}
}  // namespace

bool readParticleBook(ScriptVM& vm, ParticleBook& book, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "particles");
    if (lua_isnil(L, -1)) { lua_settop(L, base); return true; }   // none: the built-ins stand
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "`particles` is not a table"); }
    const int all = lua_gettop(L);
    static const std::set<std::string> known = {"gravity", "drag",  "wobble", "life",  "shape", "colour", "glow",
                                                 "size",    "alpha", "fade_in", "fade_out", "stretch", "count",
                                                 "speed",   "up",    "spread", "jitter"};
    lua_pushnil(L);
    while (lua_next(L, all) != 0) {
        if (!lua_isstring(L, -2) || !lua_istable(L, -1)) {
            lua_settop(L, base);
            return fail(err, "a particle effect is not name = {...}");
        }
        const std::string name = lua_tostring(L, -2);
        const std::string where = "particles." + name;
        const int t = lua_gettop(L);
        // every key must be one we read (a typo is an error, not a setting quietly lost)
        lua_pushnil(L);
        while (lua_next(L, t) != 0) {
            const std::string key = lua_type(L, -2) == LUA_TSTRING ? lua_tostring(L, -2) : "?";
            lua_pop(L, 1);
            if (!known.count(key)) {
                lua_settop(L, base);
                return fail(err, where + ": unknown field `" + key + "`");
            }
        }
        const int existing = book.find(name);
        ParticleEffect e = existing >= 0 ? book.at(existing) : ParticleEffect{};   // a partial entry edits a built-in
        e.name = name;
        e.gravity = numField(L, t, "gravity", e.gravity);
        e.drag = numField(L, t, "drag", e.drag);
        e.wobble = numField(L, t, "wobble", e.wobble);
        e.alpha = numField(L, t, "alpha", e.alpha);
        e.fadeIn = numField(L, t, "fade_in", e.fadeIn);
        e.fadeOut = numField(L, t, "fade_out", e.fadeOut);
        e.stretch = numField(L, t, "stretch", e.stretch);
        e.count = static_cast<int>(numField(L, t, "count", e.count));
        e.speed = numField(L, t, "speed", e.speed);
        e.up = numField(L, t, "up", e.up);
        e.spread = numField(L, t, "spread", e.spread);
        e.jitter = numField(L, t, "jitter", e.jitter);
        e.colour = vec3Field(L, t, "colour", e.colour);
        e.glow = vec3Field(L, t, "glow", e.glow);
        if (!pairField(L, t, "life", e.lifeMin, e.lifeMax)) { lua_settop(L, base); return fail(err, where + ".life: a number or {min, max}"); }
        if (!pairField(L, t, "size", e.sizeStart, e.sizeEnd)) { lua_settop(L, base); return fail(err, where + ".size: a number or {start, end}"); }
        const std::string shape = strField(L, t, "shape");
        if (shape == "puff") e.shape = ParticleShape::Puff;
        else if (shape == "ring") e.shape = ParticleShape::Ring;
        else if (shape == "glow") e.shape = ParticleShape::Glow;
        else if (!shape.empty()) { lua_settop(L, base); return fail(err, where + ".shape: puff, ring or glow, not " + shape); }
        if (e.lifeMax < e.lifeMin || e.lifeMin <= 0) { lua_settop(L, base); return fail(err, where + ".life: 0 < min <= max"); }
        book.set(e);
        lua_pop(L, 1);
    }
    lua_settop(L, base);
    return true;
}

bool loadParticleBookSource(const std::string& source, ParticleBook& book, std::string* err) {
    ScriptVM vm;
    std::string e;
    if (!vm.doString(source, &e)) return fail(err, "particles.lua: " + e);
    return readParticleBook(vm, book, err);
}

bool ensureParticleBookLoaded() {
    static std::once_flag once;
    static bool ok = false;
    std::call_once(once, [] {
        const std::string src = loadScriptCode("particles.lua", "");
        if (src.empty()) { LOG_WARN << "[particles] no particles.lua: the built-in effects stand"; return; }
        std::string err;
        ParticleBook book = ParticleBook::global();
        if (!loadParticleBookSource(src, book, &err)) { LOG_WARN << "[particles] " << err; return; }
        ParticleBook::global() = book;
        ok = true;
        LOG_INFO << "[particles] " << book.size() << " effects";
    });
    return ok;
}

}  // namespace engine
