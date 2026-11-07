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

// THE ONE HANDLING READER (item 23, #117): a recipe's handling table -- every field a drivable spec carries
// besides its meshes and lamps -- into `out`'s Jolt config, suspension rig and drivetrain flags. A drivable
// spec's own table and a fleet slot's `handling` (vehicles.lua: class_handling / kit_handling) are both read
// here, so a commandeered traffic car and the same car dropped by the player get one config.
void readVehicleHandling(lua_State* L, int t, VehicleSpec& out) {
    PhysicsWorld::VehicleConfig& c = out.config;
    c.mass = numField(L, t, "mass", c.mass);
    c.engineTorque = numField(L, t, "engine_torque", c.engineTorque);
    c.maxRPM = numField(L, t, "max_rpm", c.maxRPM);
    c.maxSteerDegrees = numField(L, t, "max_steer_deg", c.maxSteerDegrees);
    c.brakeTorque = numField(L, t, "brake_torque", c.brakeTorque);
    c.handBrakeTorque = numField(L, t, "hand_brake_torque", c.handBrakeTorque);
    c.comOffsetY = numField(L, t, "com_offset", c.comOffsetY);
    // The forgiving knobs (physics_world.h): a recipe may loosen or drop them.
    c.maxPitchRollDegrees = numField(L, t, "max_roll_deg", c.maxPitchRollDegrees);
    c.antiRollStiffness = numField(L, t, "anti_roll", c.antiRollStiffness);
    lua_getfield(L, t, "drift");   // { rear_grip, slip_deg, min_speed }: the car's own drift (physics_world.h)
    if (lua_istable(L, -1)) {
        const int d = lua_gettop(L);
        c.driftRearGrip = numField(L, d, "rear_grip", c.driftRearGrip);
        c.driftSlipDegrees = numField(L, d, "slip_deg", c.driftSlipDegrees);
        c.driftMinSpeed = numField(L, d, "min_speed", c.driftMinSpeed);
    }
    lua_pop(L, 1);
    c.yawAssist = numField(L, t, "yaw_assist", c.yawAssist);
    c.lateralGrip = numField(L, t, "grip", c.lateralGrip);
    lua_getfield(L, t, "pedals");   // true: pedalled (a bicycle) -- heard as a freewheel and tyres, not an engine
    if (lua_isboolean(L, -1)) out.pedalled = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    lua_getfield(L, t, "two_wheeler");   // true: a motorcycle, moped or bicycle (Jolt's MotorcycleController)
    if (lua_isboolean(L, -1)) c.twoWheeler = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    lua_getfield(L, t, "open_seat");
    out.openSeat = c.twoWheeler || lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    lua_getfield(L, t, "seat");
    if (lua_istable(L, -1)) {
        const int st = lua_gettop(L);
        out.seat.grip = vec3Field(L, st, "grip", out.seat.grip);
        out.seat.foot = vec3Field(L, st, "foot", out.seat.foot);
        out.seat.lean = numField(L, st, "lean", out.seat.lean);
        out.seat.crank = numField(L, st, "crank", out.seat.crank);
        out.seat.wheel = !boolField(L, st, "bars", !out.seat.wheel);
        out.seat.pedals = boolField(L, st, "pedals", out.seat.pedals);
    }
    lua_pop(L, 1);
    c.maxLeanDegrees = numField(L, t, "max_lean_deg", c.maxLeanDegrees);
    c.engineDamping = numField(L, t, "engine_damping", c.engineDamping);
    c.engineInertia = numField(L, t, "engine_inertia", c.engineInertia);
    // The collision's underside: floor_clearance (m off the body box's floor), approach / departure angles
    c.floorClearance = numField(L, t, "floor_clearance", c.floorClearance);
    c.approachDegrees = numField(L, t, "approach_deg", c.approachDegrees);
    c.departureDegrees = numField(L, t, "departure_deg", c.departureDegrees);
    // DRIVETRAIN: drive = "fwd" | "rwd" | "awd" | "4wd" (part-time: starts in 2WD, rear, the player
    // switches -- VehicleSystem), or front_share = 0..1 directly; axle_lsd / center_lsd limited-slip ratios
    lua_getfield(L, t, "drive");
    if (lua_isstring(L, -1)) {
        const std::string d = lua_tostring(L, -1);
        if (d == "fwd") c.frontDriveShare = 1.0;
        else if (d == "rwd") c.frontDriveShare = 0.0;
        else if (d == "awd") c.frontDriveShare = 0.5;
        else if (d == "4wd") { c.frontDriveShare = 0.0; out.partTime4wd = true; }
    }
    lua_pop(L, 1);
    c.frontDriveShare = numField(L, t, "front_share", c.frontDriveShare);
    c.axleLimitedSlip = numField(L, t, "axle_lsd", c.axleLimitedSlip);
    c.centerLimitedSlip = numField(L, t, "center_lsd", c.centerLimitedSlip);
    lua_getfield(L, t, "traction_split");   // true: in 4WD a hanging axle hands its torque to the grounded one
    if (lua_isboolean(L, -1)) c.tractionSplit = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    // AERO and GEARBOX
    c.dragArea = numField(L, t, "drag_area", c.dragArea);
    c.downforce = numField(L, t, "downforce", c.downforce);
    c.shiftTime = numField(L, t, "shift_time", c.shiftTime);
    c.clutchReleaseTime = numField(L, t, "clutch_time", c.clutchReleaseTime);
    c.shiftLatency = numField(L, t, "shift_latency", c.shiftLatency);
    lua_getfield(L, t, "gear_ratios");
    if (lua_istable(L, -1)) {
        const int gt = lua_gettop(L), n = static_cast<int>(luaL_len(L, gt));
        for (int i = 1; i <= n; ++i) { lua_rawgeti(L, gt, i); c.gearRatios.push_back(lua_tonumber(L, -1)); lua_pop(L, 1); }
    }
    lua_pop(L, 1);
    // SUSPENSION = { min=, max=, freq=, damp=, rest_drop= } -- the street rig by default; an off-roader's
    // long travel and soft springs
    lua_getfield(L, t, "suspension");
    if (lua_istable(L, -1)) {
        const int st = lua_gettop(L);
        out.suspensionMin = numField(L, st, "min", out.suspensionMin);
        out.suspensionMax = numField(L, st, "max", out.suspensionMax);
        out.suspensionFrequency = numField(L, st, "freq", out.suspensionFrequency);
        out.suspensionDamping = numField(L, st, "damp", out.suspensionDamping);
        out.suspensionRestDrop = numField(L, st, "rest_drop", out.suspensionRestDrop);
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
                // The spec publishes the wheel's DRAWN resting centre
                // (vehicles.lua: axleY = -H/2 + r, the same datum the fleet
                // bakes its wheels at). Lift the attach point by the settle
                // and use the rig's travel -- every car this reader builds, a
                // dropped one or a traffic car made physical -- so the car rests
                // exactly where it is drawn instead of 0.30 m above it. (Jolt hangs
                // the wheel BELOW its attach point, and the frequency spring's rest
                // drop is mass-independent: tests/test_driving_lab.cpp pins it.)
                w.position = Vec3(numField(L, wi, "x", 0),
                                  numField(L, wi, "y", 0) + out.suspensionRestDrop,
                                  numField(L, wi, "z", 0));
                w.suspensionMin = out.suspensionMin;
                w.suspensionMax = out.suspensionMax;
                w.suspensionFrequency = out.suspensionFrequency;
                w.suspensionDamping = out.suspensionDamping;
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

}

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

    // panels = { { name=, pivot={x,y,z}, axis={x,y,z}, open=deg }, ... } (ADR-0352), before the parts that name them
    lua_getfield(L, t, "panels");
    if (lua_istable(L, -1)) {
        const int nt = lua_gettop(L);
        const int n = static_cast<int>(luaL_len(L, nt));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, nt, i);
            const int pi = lua_gettop(L);
            if (lua_istable(L, pi)) {
                VehicleSpec::Panel panel;
                lua_getfield(L, pi, "name");
                if (lua_isstring(L, -1)) panel.name = lua_tostring(L, -1);
                lua_pop(L, 1);
                panel.pivot = vec3Field(L, pi, "pivot", panel.pivot);
                panel.axis = normalize(vec3Field(L, pi, "axis", panel.axis));
                panel.openDeg = static_cast<float>(numField(L, pi, "open", panel.openDeg));
                panel.centre = vec3Field(L, pi, "centre", panel.pivot);
                panel.lo = vec3Field(L, pi, "lo", panel.centre);
                panel.hi = vec3Field(L, pi, "hi", panel.centre);
                panel.mass = static_cast<float>(numField(L, pi, "mass", panel.mass));
                if (!panel.name.empty()) out.panels.push_back(std::move(panel));
            }
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);

    // parts = { { mesh=, albedo=, metallic=, roughness=, opacity=, emission=, panel= }, ... }
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
                    lua_getfield(L, pi, "panel");
                    if (lua_isstring(L, -1)) {
                        const std::string pn = lua_tostring(L, -1);
                        for (std::size_t k = 0; k < out.panels.size(); ++k)
                            if (out.panels[k].name == pn) part.panel = static_cast<int>(k);
                    }
                    lua_pop(L, 1);
                    out.parts.push_back(std::move(part));
                }
            }
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);

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

    readVehicleHandling(L, t, out);

    // damage = { knock_from=, knock_full=, ..., dent = {...}, smoke = {...}, kick = {...}, rough = {...} } (ADR-0355)
    lua_getfield(L, t, "damage");
    if (lua_istable(L, -1)) {
        const int d = lua_gettop(L);
        DamageRule& r = out.damage;
        r.knockFrom = numField(L, d, "knock_from", r.knockFrom);
        r.knockFull = numField(L, d, "knock_full", r.knockFull);
        r.scrapePerMetre = numField(L, d, "scrape_per_metre", r.scrapePerMetre);
        r.refMass = numField(L, d, "ref_mass", r.refMass);
        r.sparkKnock = numField(L, d, "spark_knock", r.sparkKnock);
        r.sparkSlide = numField(L, d, "spark_slide", r.sparkSlide);
        r.crashWindow = numField(L, d, "crash_window", r.crashWindow);
        r.groundShare = numField(L, d, "ground_share", r.groundShare);
        r.breakShare = numField(L, d, "break_share", r.breakShare);
        r.zoneScale = numField(L, d, "zone_scale", r.zoneScale);
        r.panelScale = numField(L, d, "panel_scale", r.panelScale);
        auto sub = [&](const char* name, auto&& read) {
            lua_getfield(L, d, name);
            if (lua_istable(L, -1)) read(lua_gettop(L));
            lua_pop(L, 1);
        };
        sub("dent", [&](int x) {
            r.dentFrom = numField(L, x, "from", r.dentFrom);
            r.dentRadiusLo = numField(L, x, "radius_lo", r.dentRadiusLo);
            r.dentRadiusHi = numField(L, x, "radius_hi", r.dentRadiusHi);
            r.dentDepthLo = numField(L, x, "depth_lo", r.dentDepthLo);
            r.dentDepthPer = numField(L, x, "depth_per", r.dentDepthPer);
            r.dentDepthMax = numField(L, x, "depth_max", r.dentDepthMax);
            r.scuffLo = numField(L, x, "scuff_lo", r.scuffLo);
            r.scuffPer = numField(L, x, "scuff_per", r.scuffPer);
        });
        sub("smoke", [&](int x) {
            r.steamBelow = numField(L, x, "steam_below", r.steamBelow);
            r.blackBelow = numField(L, x, "black_below", r.blackBelow);
        });
        sub("kick", [&](int x) {
            r.kickCar = numField(L, x, "car", r.kickCar);
            r.kickOut = numField(L, x, "out", r.kickOut);
            r.kickUp = numField(L, x, "up", r.kickUp);
        });
        sub("rough", [&](int x) {
            r.roughBelow = numField(L, x, "below", r.roughBelow);
            r.roughWobble = numField(L, x, "wobble", r.roughWobble);
            r.roughMisfire = numField(L, x, "misfire", r.roughMisfire);
        });
    }
    lua_pop(L, 1);

    lua_pop(L, 1);   // the returned spec table
    return out.body != nullptr || !out.config.wheels.empty();
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
    v.partTime4wd = spec.partTime4wd;
    v.pedalled = spec.pedalled;
    v.driverSeat = spec.driverSeat;
    v.hasDriverSeat = spec.hasDriverSeat;
    v.openSeat = spec.openSeat;
    v.seat = spec.seat;
    v.bodyMesh = spec.body;   // (shared with the spec until the first dent copies it)
    v.damage = spec.damage;

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
        v.bodyPartPanel.push_back(part.panel);
        v.partMeshes.push_back(part.mesh);
        v.partDented.push_back(0);
    }
    for (const VehicleSpec::Panel& sp : spec.panels) {
        Vehicle::Panel panel;
        panel.name = sp.name;
        panel.pivot = sp.pivot;
        panel.axis = sp.axis;
        panel.open = sp.openDeg * static_cast<float>(3.14159265358979 / 180.0);
        panel.centre = sp.centre;
        panel.lo = sp.lo;
        panel.hi = sp.hi;
        panel.mass = sp.mass;
        v.panels.push_back(panel);
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

namespace engine {

bool loadDrivableCatalogue(ScriptVM& vm, std::vector<DrivableEntry>& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "vehicle");
    if (!lua_istable(L, -1)) { lua_settop(L, base); if (err) *err = "no global `vehicle`"; return false; }
    lua_getfield(L, -1, "drivable");
    if (!lua_istable(L, -1)) { lua_settop(L, base); if (err) *err = "no `vehicle.drivable`"; return false; }
    const int t = lua_gettop(L), n = static_cast<int>(luaL_len(L, t));
    out.clear();
    for (int i = 1; i <= n; ++i) {
        lua_rawgeti(L, t, i);
        if (lua_istable(L, -1)) {
            DrivableEntry e;
            lua_getfield(L, -1, "recipe"); if (lua_isstring(L, -1)) e.recipe = lua_tostring(L, -1); lua_pop(L, 1);
            lua_getfield(L, -1, "label"); if (lua_isstring(L, -1)) e.label = lua_tostring(L, -1); lua_pop(L, 1);
            lua_getfield(L, -1, "drive"); if (lua_isstring(L, -1)) e.drive = lua_tostring(L, -1); lua_pop(L, 1);
            lua_getfield(L, -1, "quick"); e.quick = lua_toboolean(L, -1) != 0; lua_pop(L, 1);
            if (!e.recipe.empty()) out.push_back(std::move(e));
        }
        lua_pop(L, 1);
    }
    lua_settop(L, base);
    return !out.empty();
}

std::string driveLabel(const std::string& d) {
    if (d == "fwd") return "2WD (front)";
    if (d == "rwd") return "2WD (rear)";
    if (d == "awd") return "AWD";
    if (d == "4wd") return "4WD part-time (Z)";
    return d;
}

}  // namespace engine
