#include "vehicle_spec.h"

#include "lua_state.h"        // luaState() + the Lua C API (scripting-internal)
#include "procgen_mesh.h"     // luaToMesh
#include "../components.h"
#include "../asset_manager.h"

namespace engine {

namespace {

// Field readers over a table at absolute stack index `t`.
double numField(lua_State* L, int t, const char* key, double def) {
    lua_getfield(L, t, key);
    double v = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : def;
    lua_pop(L, 1);
    return v;
}

bool boolField(lua_State* L, int t, const char* key, bool def) {
    lua_getfield(L, t, key);
    bool v = lua_isboolean(L, -1) ? (lua_toboolean(L, -1) != 0) : def;
    lua_pop(L, 1);
    return v;
}

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

}  // namespace

bool loadVehicleSpec(ScriptVM& vm, const std::string& recipe, uint32_t seed,
                     VehicleSpec& out, std::string* err) {
    lua_State* L = luaState(vm);
    vm.setGlobalNumber("seed", static_cast<double>(seed));

    if (luaL_loadstring(L, recipe.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 1, 0) != LUA_OK) {
        if (err) {
            const char* m = lua_tostring(L, -1);
            *err = m ? m : "vehicle recipe error";
        }
        lua_pop(L, 1);
        return false;
    }

    int t = lua_gettop(L);
    if (!lua_istable(L, t)) {
        if (err) *err = "vehicle recipe did not return a table";
        lua_pop(L, 1);
        return false;
    }

    // Body mesh (procgen Mesh userdata).
    lua_getfield(L, t, "body");
    out.body = luaToMesh(L, -1);
    lua_pop(L, 1);

    out.albedo = vec3Field(L, t, "albedo", out.albedo);
    out.metallic = static_cast<float>(numField(L, t, "metallic", out.metallic));
    out.roughness = static_cast<float>(numField(L, t, "roughness", out.roughness));

    PhysicsWorld::VehicleConfig& c = out.config;
    c.mass = numField(L, t, "mass", c.mass);
    c.engineTorque = numField(L, t, "engine_torque", c.engineTorque);
    c.maxRPM = numField(L, t, "max_rpm", c.maxRPM);
    c.maxSteerDegrees = numField(L, t, "max_steer_deg", c.maxSteerDegrees);
    c.brakeTorque = numField(L, t, "brake_torque", c.brakeTorque);
    c.handBrakeTorque = numField(L, t, "hand_brake_torque", c.handBrakeTorque);
    c.comOffsetY = numField(L, t, "com_offset", c.comOffsetY);

    // chassis = { half = {x,y,z} }
    lua_getfield(L, t, "chassis");
    if (lua_istable(L, -1))
        c.chassisHalfExtent = vec3Field(L, lua_gettop(L), "half", c.chassisHalfExtent);
    lua_pop(L, 1);

    // wheel = { radius=, width= } — defaults for each wheel.
    double wheelRadius = 0.34, wheelWidth = 0.24;
    lua_getfield(L, t, "wheel");
    if (lua_istable(L, -1)) {
        int wt = lua_gettop(L);
        wheelRadius = numField(L, wt, "radius", wheelRadius);
        wheelWidth = numField(L, wt, "width", wheelWidth);
    }
    lua_pop(L, 1);

    // wheels = { {x,y,z, steered=, driven=, hand_brake=, radius=, width=}, ... }
    lua_getfield(L, t, "wheels");
    if (lua_istable(L, -1)) {
        int wt = lua_gettop(L);
        int n = static_cast<int>(luaL_len(L, wt));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, wt, i);
            int wi = lua_gettop(L);
            if (lua_istable(L, wi)) {
                PhysicsWorld::VehicleWheel w;
                w.position = Vec3(numField(L, wi, "x", 0), numField(L, wi, "y", 0),
                                  numField(L, wi, "z", 0));
                w.radius = numField(L, wi, "radius", wheelRadius);
                w.width = numField(L, wi, "width", wheelWidth);
                w.steered = boolField(L, wi, "steered", false);
                w.driven = boolField(L, wi, "driven", false);
                w.handBrake = boolField(L, wi, "hand_brake", false);
                c.wheels.push_back(w);
            }
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);

    lua_pop(L, 1);   // the returned spec table
    return out.body != nullptr || !c.wheels.empty();
}

Entity spawnVehicle(World& world, AssetManager& assets, const VehicleSpec& spec,
                    const Vec3& position, Real yawDegrees) {
    Entity e = world.create();

    Transform t;
    t.position = position;
    t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), degreesToRadians(yawDegrees));
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, {t});

    if (spec.body) {
        Renderable r;
        r.mesh = assets.acquireMesh(*spec.body);   // unique upload (no dedup key)
        r.material.albedo = spec.albedo;
        r.material.metallic = spec.metallic;
        r.material.roughness = spec.roughness;
        r.material.opacity = 1.0f;
        world.add<Renderable>(e, r);
    }

    Vehicle v;
    v.config = spec.config;
    world.add<Vehicle>(e, v);
    return e;
}

}  // namespace engine
