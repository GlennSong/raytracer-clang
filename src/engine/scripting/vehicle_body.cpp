#include "vehicle_body.h"

#include "lua_state.h"            // luaState() + the Lua C API (scripting-internal)
#include "../mesh_builder.h"      // MeshBuilder::box (same primitive as addBox)

namespace engine {

namespace {

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

// String field of the table at absolute stack index `t` ("" when absent).
std::string strField(lua_State* L, int t, const char* key) {
    lua_getfield(L, t, key);
    const char* s = lua_isstring(L, -1) ? lua_tostring(L, -1) : nullptr;
    std::string v = s ? s : "";
    lua_pop(L, 1);
    return v;
}

// A {x,y,z} array-field of the table at absolute stack index `t` (like
// vehicle_spec's reader): missing components keep `def`.
Vec3 vec3Field(lua_State* L, int t, const char* key, Vec3 def) {
    lua_getfield(L, t, key);
    Vec3 r = def;
    if (lua_istable(L, -1)) {
        int st = lua_gettop(L);
        lua_rawgeti(L, st, 1);
        if (lua_isnumber(L, -1)) r.x = lua_tonumber(L, -1);
        lua_pop(L, 1);
        lua_rawgeti(L, st, 2);
        if (lua_isnumber(L, -1)) r.y = lua_tonumber(L, -1);
        lua_pop(L, 1);
        lua_rawgeti(L, st, 3);
        if (lua_isnumber(L, -1)) r.z = lua_tonumber(L, -1);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return r;
}

// Append a vertex-coloured box (centred at `c`, dimensions `size`) into `out` —
// the EXACT primitive citysim's addBox uses (MeshBuilder::box, offset, tint), so
// a Lua-authored body winds identically to the C++ fleetCarMesh it replaces.
void addColoredBox(RenderMesh& out, Vec3 size, Vec3 c, Vec3 color) {
    RenderMesh b = MeshBuilder::box(size);
    uint32_t base = static_cast<uint32_t>(out.vertices.size());
    for (Vertex v : b.vertices) {
        v.position = v.position + c;
        v.color = color;
        out.vertices.push_back(v);
    }
    for (uint32_t i : b.indices) out.indices.push_back(base + i);
}

}  // namespace

bool loadFleetCarBody(ScriptVM& vm, int slot, CarBodyRecipe& out,
                      std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    const std::string where =
        "vehicle.fleet[" + std::to_string(slot + 1) + "]";

    lua_getglobal(L, "vehicle");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, "vehicles.lua: no global `vehicle` table (load it first)");
    }
    lua_getfield(L, -1, "fleet");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, "vehicles.lua: no `vehicle.fleet` array");
    }
    const int fleet = lua_gettop(L);
    const int n = static_cast<int>(luaL_len(L, fleet));
    if (slot < 0 || slot >= n) {
        lua_settop(L, base);
        return fail(err, where + ": slot out of range (fleet has " +
                             std::to_string(n) + ")");
    }
    lua_rawgeti(L, fleet, slot + 1);
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, where + " is not a table");
    }
    const int rec = lua_gettop(L);
    CarBodyRecipe body;

    // parts = { { pos={x,y,z}, size={w,h,l}, color={r,g,b} }, ... }
    lua_getfield(L, rec, "parts");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, where + ": missing `parts` array");
    }
    const int parts = lua_gettop(L);
    const int nParts = static_cast<int>(luaL_len(L, parts));
    if (nParts <= 0) {
        lua_settop(L, base);
        return fail(err, where + ": empty `parts` array");
    }
    for (int i = 1; i <= nParts; ++i) {
        lua_rawgeti(L, parts, i);
        const int pi = lua_gettop(L);
        if (!lua_istable(L, pi)) {
            lua_settop(L, base);
            return fail(err, where + ": parts[" + std::to_string(i) +
                                 "] is not a table");
        }
        Vec3 size = vec3Field(L, pi, "size", Vec3(0, 0, 0));
        Vec3 pos = vec3Field(L, pi, "pos", Vec3(0, 0, 0));
        Vec3 color = vec3Field(L, pi, "color", Vec3(1, 1, 1));
        lua_pop(L, 1);
        if (size.x <= 0 || size.y <= 0 || size.z <= 0) {
            lua_settop(L, base);
            return fail(err, where + ": parts[" + std::to_string(i) +
                                 "] has a non-positive size");
        }
        addColoredBox(body.mesh, size, pos, color);
    }
    lua_pop(L, 1);   // parts

    // lights = { { name=, pos={x,y,z} }, ... } — named lamp attachment markers.
    lua_getfield(L, rec, "lights");
    if (lua_istable(L, -1)) {
        const int lights = lua_gettop(L);
        const int nLights = static_cast<int>(luaL_len(L, lights));
        for (int i = 1; i <= nLights; ++i) {
            lua_rawgeti(L, lights, i);
            const int li = lua_gettop(L);
            if (!lua_istable(L, li)) {
                lua_settop(L, base);
                return fail(err, where + ": lights[" + std::to_string(i) +
                                     "] is not a table");
            }
            std::string name = strField(L, li, "name");
            Vec3 pos = vec3Field(L, li, "pos", Vec3(0, 0, 0));
            lua_pop(L, 1);
            if (name.empty()) {
                lua_settop(L, base);
                return fail(err, where + ": lights[" + std::to_string(i) +
                                     "] has no `name`");
            }
            body.lights.push_back({name, pos});
        }
    }
    lua_pop(L, 1);   // lights (or non-table)

    lua_settop(L, base);
    out = std::move(body);
    return true;
}

}  // namespace engine
