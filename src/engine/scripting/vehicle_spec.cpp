#include "vehicle_spec.h"

#include "lua_state.h"        // luaState() + the Lua C API (scripting-internal)
#include "lua_helpers.h"      // numField/boolField/vec3Field spec readers
#include "procgen_mesh.h"     // luaToMesh
#include "../components.h"
#include "../asset_manager.h"

namespace engine {

namespace {

// Field readers (numField/boolField/vec3Field) come from lua_helpers.h.

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

    // parts = { { mesh=, albedo=, metallic=, roughness=, opacity=, emission= }, ... }
    // Each becomes a child Renderable so glass/interior keep their own material.
    lua_getfield(L, t, "parts");
    if (lua_istable(L, -1)) {
        const int pt = lua_gettop(L);
        const int n = static_cast<int>(luaL_len(L, pt));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, pt, i);
            const int pi = lua_gettop(L);
            if (lua_istable(L, pi)) {
                VehicleSpec::Part part;
                lua_getfield(L, pi, "mesh");
                part.mesh = luaToMesh(L, -1);
                lua_pop(L, 1);
                if (part.mesh) {
                    part.albedo = vec3Field(L, pi, "albedo", part.albedo);
                    part.metallic = static_cast<float>(numField(L, pi, "metallic", part.metallic));
                    part.roughness = static_cast<float>(numField(L, pi, "roughness", part.roughness));
                    part.opacity = static_cast<float>(numField(L, pi, "opacity", part.opacity));
                    part.emission = vec3Field(L, pi, "emission", part.emission);
                    out.parts.push_back(std::move(part));
                }
            }
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);

    PhysicsWorld::VehicleConfig& c = out.config;
    c.mass = numField(L, t, "mass", c.mass);
    c.engineTorque = numField(L, t, "engine_torque", c.engineTorque);
    c.maxRPM = numField(L, t, "max_rpm", c.maxRPM);
    c.maxSteerDegrees = numField(L, t, "max_steer_deg", c.maxSteerDegrees);
    c.brakeTorque = numField(L, t, "brake_torque", c.brakeTorque);
    c.handBrakeTorque = numField(L, t, "hand_brake_torque", c.handBrakeTorque);
    c.comOffsetY = numField(L, t, "com_offset", c.comOffsetY);

    // lights = { { name = "headlight_l", pos = {x,y,z} }, ... }
    // Same shape citysim's fleet recipes use, so `mesh.car`'s output drops
    // straight into either consumer.
    lua_getfield(L, t, "lights");
    if (lua_istable(L, -1)) {
        const int lt = lua_gettop(L);
        const int n = static_cast<int>(lua_rawlen(L, lt));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, lt, i);
            if (lua_istable(L, -1)) {
                const int et = lua_gettop(L);
                VehicleSpec::LampMarker m;
                lua_getfield(L, et, "name");
                if (lua_isstring(L, -1)) m.name = lua_tostring(L, -1);
                lua_pop(L, 1);
                m.pos = vec3Field(L, et, "pos", m.pos);
                // The markers array carries more than lamps — the interior's
                // driver hip point rides along in it. Route by name rather than
                // taking everything as a lamp: "driver_seat" neither starts with
                // "headlight" nor ends in 'l', so a blind reader would have made
                // it a right-hand TAIL light.
                if (m.name == "driver_seat") {
                    out.driverSeat = m.pos;
                    out.hasDriverSeat = true;
                } else if (m.name.rfind("headlight", 0) == 0 ||
                           m.name.rfind("taillight", 0) == 0) {
                    out.lights.push_back(m);
                }
            }
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);

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
    v.driverSeat = spec.driverSeat;
    v.hasDriverSeat = spec.hasDriverSeat;

    // One child entity per extra part, rigidly pinned to the chassis by
    // VehicleSystem::writeBack. Separate entities because each carries its own
    // material — glass transparent, interior matte — which a single Renderable
    // cannot express.
    for (const VehicleSpec::Part& part : spec.parts) {
        if (!part.mesh) continue;
        Entity pe = world.create();
        Transform pt = t;
        world.add<Transform>(pe, pt);
        world.add<PrevTransform>(pe, {pt});
        Renderable pr;
        pr.mesh = assets.acquireMesh(*part.mesh);
        pr.material.albedo = part.albedo;
        pr.material.metallic = part.metallic;
        pr.material.roughness = part.roughness;
        pr.material.opacity = part.opacity;
        pr.material.emission = part.emission;
        world.add<Renderable>(pe, pr);
        v.bodyParts.push_back(pe);
    }
    // Marker slots only: VehicleSystem creates the lens entities on its first
    // fixed step (it owns the shared mesh and the asset manager), so a car
    // spawned before systems start still ends up lit.
    for (const VehicleSpec::LampMarker& m : spec.lights) {
        Vehicle::Lamp lamp;
        lamp.local = m.pos;
        lamp.front = m.name.rfind("headlight", 0) == 0;
        lamp.left = !m.name.empty() && m.name.back() == 'l';
        v.lamps.push_back(lamp);
    }
    world.add<Vehicle>(e, v);
    return e;
}

}  // namespace engine
