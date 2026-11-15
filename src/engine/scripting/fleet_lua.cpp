#include "fleet_lua.h"

#include "lua_helpers.h"   // strField, vec3Field
#include "lua_state.h"     // luaState() + the Lua C API

#include <algorithm>

namespace engine {

namespace {

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

constexpr const char* kDrivingBlock = "driving";   // a slot's block for the driving model
constexpr const char* kTagsKey = "tags";           // ...and its list of tag names

}  // namespace

bool pushFleetSlot(lua_State* L, int base, int slot, const std::string& where, std::string* err) {
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
        return fail(err, where + ": slot out of range (fleet has " + std::to_string(n) + ")");
    }
    lua_rawgeti(L, fleet, slot + 1);
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, where + " is not a table");
    }
    return true;
}

bool selectFleet(ScriptVM& vm, const std::string& name, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "vehicle");
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "no global `vehicle` table"); }
    lua_getfield(L, -1, name.c_str());
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "no `vehicle." + name + "` table"); }
    lua_setfield(L, -2, "fleet");   // vehicle.fleet = vehicle[name]
    lua_settop(L, base);
    return true;
}

int fleetSlotCount(ScriptVM& vm) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    int n = 0;
    lua_getglobal(L, "vehicle");
    if (lua_istable(L, -1)) {
        lua_getfield(L, -1, "fleet");
        if (lua_istable(L, -1)) n = static_cast<int>(luaL_len(L, lua_gettop(L)));
    }
    lua_settop(L, base);
    return n;
}

bool loadFleetCatalogueEntry(ScriptVM& vm, int slot, Vec3& size, std::string& className, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    const std::string where = "vehicle.fleet[" + std::to_string(slot + 1) + "]";
    if (!pushFleetSlot(L, base, slot, where, err)) return false;
    const int rec = lua_gettop(L);

    // Deliberately does NOT touch `build` — reading a car's measurements must
    // not cost a car's geometry.
    size = vec3Field(L, rec, "size", Vec3(0, 0, 0));
    className = strField(L, rec, "class");
    lua_settop(L, base);

    if (size.x < 0 || size.y < 0 || size.z < 0)
        return fail(err, where + ": `size` has a negative component");
    return true;
}

bool loadFleetHandling(ScriptVM& vm, int slot, VehicleSpec& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    const std::string where = "vehicle.fleet[" + std::to_string(slot + 1) + "]";
    if (!pushFleetSlot(L, base, slot, where, err)) return false;
    lua_getfield(L, lua_gettop(L), "handling");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, where + ": no `handling` table");
    }
    VehicleSpec spec;
    readVehicleHandling(L, lua_gettop(L), spec);
    lua_settop(L, base);
    if (spec.config.wheels.size() < 2) return fail(err, where + ": `handling` has no wheels");
    out = std::move(spec);
    return true;
}

bool loadFleetSlotBody(ScriptVM& vm, int slot, FleetSlotBody& out, std::string* err) {
    Vec3 size(0, 0, 0);
    std::string className;
    VehicleSpec handling;
    if (!loadFleetCatalogueEntry(vm, slot, size, className, err) || !loadFleetHandling(vm, slot, handling, err))
        return false;
    const std::string where = "vehicle.fleet[" + std::to_string(slot + 1) + "]";
    std::vector<roads::lanes::WheelSpot> spots;
    for (const PhysicsWorld::VehicleWheel& w : handling.config.wheels) spots.push_back({w.position.z, w.steered});
    Real wheelbase = 0;
    if (!roads::lanes::wheelbaseFromWheels(spots, wheelbase))
        return fail(err, where + ": its wheels give no wheelbase (none of them steers, or all do)");
    out.className = className;
    out.length = size.z;
    out.width = size.x;
    out.wheelbase = wheelbase;
    out.maxSteerDegrees = handling.config.maxSteerDegrees;
    return true;
}

void readTagNames(lua_State* L, int list, std::vector<std::string>& names, std::vector<int>* notNames) {
    list = lua_absindex(L, list);
    names.clear();
    const lua_Integer n = static_cast<lua_Integer>(luaL_len(L, list));
    for (lua_Integer i = 1; i <= n; ++i) {
        lua_rawgeti(L, list, i);
        if (lua_type(L, -1) == LUA_TSTRING && lua_rawlen(L, -1) > 0) names.push_back(lua_tostring(L, -1));
        else if (notNames) notNames->push_back(static_cast<int>(i));
        lua_pop(L, 1);
    }
    std::sort(names.begin(), names.end());
    names.erase(std::unique(names.begin(), names.end()), names.end());
}

std::vector<roads::lanes::VehicleSize> fleetVehicleSizes(ScriptVM& vm, std::vector<std::string>* notes) {
    std::vector<roads::lanes::VehicleSize> sizes;
    std::vector<std::string> seen;   // class names taken (a class is its first slot)
    lua_State* L = luaState(vm);
    const int slots = fleetSlotCount(vm);
    for (int slot = 0; slot < slots; ++slot) {
        const std::string where = "vehicle.fleet[" + std::to_string(slot + 1) + "]";
        FleetSlotBody body;
        std::string err;
        if (!loadFleetSlotBody(vm, slot, body, &err)) {
            if (notes) notes->push_back(err + " (the roads are not checked for it)");
            continue;
        }
        if (std::find(seen.begin(), seen.end(), body.className) != seen.end()) continue;
        std::vector<std::string> tags;
        const int base = lua_gettop(L);
        bool listed = false;
        if (pushFleetSlot(L, base, slot, where, &err)) {
            lua_getfield(L, lua_gettop(L), kDrivingBlock);
            if (lua_istable(L, -1)) {
                lua_getfield(L, lua_gettop(L), kTagsKey);
                listed = lua_istable(L, -1);
                if (listed) readTagNames(L, -1, tags);
            }
        }
        lua_settop(L, base);
        if (!listed) {
            if (notes) notes->push_back(where + ": no `" + kDrivingBlock + "." + kTagsKey + "` list (the roads are not checked for it)");
            continue;
        }
        seen.push_back(body.className);
        const Real radius = roads::lanes::tightestTurnRadius(body.wheelbase, body.maxSteerDegrees);
        for (const std::string& tag : tags) roads::lanes::addVehicleSize(sizes, tag, radius, body.width, body.length);
    }
    return sizes;
}

}  // namespace engine
