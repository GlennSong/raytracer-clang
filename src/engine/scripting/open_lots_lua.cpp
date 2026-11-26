#include "open_lots_lua.h"
#include "lua_state.h"     // the Lua C API (scripting-internal)
#include "lua_helpers.h"   // numField / strField
#include "script_vm.h"
#include "../script_assets.h"
#include "../../log.h"
#include <mutex>

namespace engine {

bool loadOpenLotsSource(const std::string& code, OpenLotBook& out, std::string* err) {
    ScriptVM vm;
    lua_State* L = luaState(vm);
    auto fail = [&](const std::string& why) { if (err) *err = "open_lots.lua: " + why; return false; };
    if (luaL_loadstring(L, code.c_str()) != LUA_OK || lua_pcall(L, 0, 1, 0) != LUA_OK)
        return fail(lua_tostring(L, -1) ? lua_tostring(L, -1) : "did not run");
    if (!lua_istable(L, -1)) return fail("returns no table");
    const int root = lua_gettop(L);
    OpenLotBook book;
    lua_getfield(L, root, "furniture");
    if (lua_istable(L, -1)) {
        const int t = lua_gettop(L);
        lua_pushnil(L);
        while (lua_next(L, t) != 0) {
            if (lua_type(L, -2) == LUA_TSTRING && lua_istable(L, -1)) {
                OpenLotFurniture f;
                const int row = lua_gettop(L);
                lua_getfield(L, row, "prefab");
                if (lua_type(L, -1) == LUA_TSTRING) f.prefabs.push_back(lua_tostring(L, -1));
                else if (lua_istable(L, -1)) {
                    const int n = static_cast<int>(lua_rawlen(L, -1));
                    for (int i = 1; i <= n; ++i) {
                        lua_rawgeti(L, -1, i);
                        if (lua_type(L, -1) == LUA_TSTRING) f.prefabs.push_back(lua_tostring(L, -1));
                        lua_pop(L, 1);
                    }
                }
                lua_pop(L, 1);
                f.back = numField(L, row, "back", 0.0);
                if (f.prefabs.empty()) return fail(std::string("furniture.") + lua_tostring(L, -2) + " names no prefab");
                book.furniture[lua_tostring(L, -2)] = f;
            }
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    lua_getfield(L, root, "playground");
    if (lua_istable(L, -1)) {
        lua_getfield(L, -1, "equipment");
        if (lua_istable(L, -1)) {
            const int list = lua_gettop(L);
            const int n = static_cast<int>(lua_rawlen(L, list));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, list, i);
                if (lua_istable(L, -1)) {
                    const int t = lua_gettop(L);
                    OpenLotPiece p;
                    p.prefab = strField(L, t, "prefab");
                    p.along = numField(L, t, "along", p.along);
                    p.deep = numField(L, t, "deep", p.deep);
                    p.turn = numField(L, t, "turn", 0.0);
                    if (p.prefab.empty() || p.along <= 0 || p.deep <= 0)
                        return fail("playground.equipment[" + std::to_string(i) + "] needs a prefab and its room (along, deep)");
                    book.playground.push_back(p);
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    out = std::move(book);
    return true;
}

void ensureOpenLotsLoaded() {
    static std::once_flag once;
    std::call_once(once, [] {
        const std::string code = loadScriptCode("open_lots.lua", "");
        if (code.empty()) { LOG_WARN << "[lots] no open_lots.lua: a playground has nothing to stand"; return; }
        OpenLotBook book;
        std::string err;
        if (!loadOpenLotsSource(code, book, &err)) { LOG_WARN << "[lots] " << err << " (a playground has nothing to stand)"; return; }
        LOG_INFO << "[lots] a playground chooses from " << book.playground.size() << " prefabs (open_lots.lua)";
        setOpenLotBook(book);
    });
}

}  // namespace engine
