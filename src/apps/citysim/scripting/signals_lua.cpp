#include "signals_lua.h"

#include "../../../engine/scripting/lua_state.h"
#include "../../../engine/scripting/script_vm.h"

namespace engine {

bool loadSignalTiming(ScriptVM& vm, citysim::SignalTiming& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "signals");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        if (err) *err = "signals.lua: no global `signals` table";
        return false;
    }
    citysim::SignalTiming t;
    struct Field { const char* key; double* v; };
    const Field fields[] = {
        {"green", &t.green}, {"yellow", &t.yellow}, {"all_red", &t.allRed},
        {"walk", &t.walk},   {"arrow", &t.arrow},   {"arrow_yellow", &t.arrowYellow},
    };
    for (const Field& f : fields) {
        lua_getfield(L, -1, f.key);
        if (!lua_isnil(L, -1)) {
            if (!lua_isnumber(L, -1) || lua_tonumber(L, -1) <= 0) {
                lua_settop(L, base);
                if (err) *err = std::string("signals.lua: `") + f.key + "` must be a positive number of seconds";
                return false;
            }
            *f.v = lua_tonumber(L, -1);
        }
        lua_pop(L, 1);
    }
    lua_settop(L, base);
    out = t;
    return true;
}

}  // namespace engine
