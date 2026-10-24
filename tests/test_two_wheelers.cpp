// The two-wheelers (ADR-0235): vehicles.lua's motorcycle, moped and bicycle, as their specs read back, on Jolt's
// MotorcycleController -- they ride off, reach their pace, lean into a turn and stay up.
#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/script_assets.h"
#include "../src/engine/scripting/vehicle_spec.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;

namespace {
struct BikesVM {
    ScriptVM vm;
    bool loaded = false;
    BikesVM() {
        openProcgenLibrary(vm);
        openModuleLoader(vm, makeModuleSource(std::string(RT_SOURCE_DIR) + "/assets/scripts"));
        std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/scripts/vehicles.lua");
        std::stringstream ss;
        ss << f.rdbuf();
        std::string err;
        loaded = vm.doString(ss.str(), &err);
        if (!loaded) std::printf("    vehicles.lua load error: %s\n", err.c_str());
    }
};
}  // namespace

TEST_CASE(two_wheelers_ride_off_lean_into_a_turn_and_stay_up) {
    BikesVM v;
    CHECK(v.loaded);
    struct Want { const char* recipe; Real minKmh, maxKmh; };
    for (const Want& want : {Want{"motorcycle", 60, 200}, Want{"moped", 30, 70}, Want{"bicycle", 18, 45}}) {
        VehicleSpec spec;
        std::string err;
        CHECK(loadVehicleSpec(v.vm, std::string("return vehicle.") + want.recipe + "(seed, {})", 1u, spec, &err));
        CHECK(spec.config.twoWheeler);
        CHECK(spec.config.wheels.size() == 2);
        PhysicsWorld w;
        w.initialize();
        w.addBox(Vec3(2000, 1, 2000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static);
        const PhysicsWorld::VehicleId id = w.addVehicle(spec.config, Vec3(0, 1.0, 0), Quat::identity());
        CHECK(id != PhysicsWorld::INVALID_VEHICLE);
        w.optimizeBroadPhase();
        for (int i = 0; i < 60; ++i) { w.setVehicleInput(id, 0, 0, 1.0, 1.0); w.update(1.0 / 60.0); }
        // full throttle, straight, 12 s
        Real top = 0, worstUp = 1;
        for (int i = 0; i < 12 * 60; ++i) {
            w.setVehicleInput(id, 1.0, 0, 0);
            w.update(1.0 / 60.0);
            top = std::max(top, w.vehicleVelocity(id).length());
            worstUp = std::min(worstUp, w.vehicleOrientation(id).rotate(Vec3(0, 1, 0)).y);
        }
        // into a turn at 80% lock on a little throttle, 4 s (the lean limits the steering at speed: a fast bike turns
        // wide; and a bicycle does not freewheel yet -- off the throttle its "engine" brakes it)
        const Vec3 f0 = w.vehicleOrientation(id).rotate(Vec3(0, 0, 1));
        for (int i = 0; i < 4 * 60; ++i) {
            w.setVehicleInput(id, 0.3, 0.8, 0);
            w.update(1.0 / 60.0);
            const Quat q = w.vehicleOrientation(id);
            worstUp = std::min(worstUp, q.rotate(Vec3(0, 1, 0)).y);
        }
        const Vec3 f1 = w.vehicleOrientation(id).rotate(Vec3(0, 0, 1));
        const Real turned = std::acos(std::clamp(dot(normalize(Vec3(f0.x, 0, f0.z)), normalize(Vec3(f1.x, 0, f1.z))), Real(-1), Real(1))) * 57.2958;
        // the lean: how far the up axis tips off vertical, in the turn
        const Real tilt = std::acos(std::clamp(w.vehicleOrientation(id).rotate(Vec3(0, 1, 0)).y, Real(-1), Real(1))) * 57.2958;
        std::printf("    %-10s top %.0f km/h, turned %.0f deg, leaning %.0f deg at the end, never below %.2f up\n",
                    want.recipe, top * 3.6, turned, tilt, worstUp);
        CHECK(top * 3.6 > want.minKmh && top * 3.6 < want.maxKmh);
        CHECK(worstUp > 0.5);      // never fell over
        CHECK(turned > 30);        // it turned
        CHECK(tilt > 5);           // ...leaning into it
        w.shutdown();
    }
}


TEST_CASE(a_bicycle_freewheels_off_the_pedals) {
    // Off the throttle a bicycle rolls on (a freewheel); a motorcycle's engine brakes it.
    BikesVM v;
    auto coastKeeps = [&](const char* recipe) {
        VehicleSpec spec;
        std::string err;
        loadVehicleSpec(v.vm, std::string("return vehicle.") + recipe + "(seed, {})", 1u, spec, &err);
        PhysicsWorld w;
        w.initialize();
        w.addBox(Vec3(2000, 1, 2000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static);
        const PhysicsWorld::VehicleId id = w.addVehicle(spec.config, Vec3(0, 0.6, 0), Quat::identity());
        w.optimizeBroadPhase();
        for (int i = 0; i < 8 * 60; ++i) { w.setVehicleInput(id, 1.0, 0, 0); w.update(1.0 / 60.0); }
        const Real before = w.vehicleVelocity(id).length();
        for (int i = 0; i < 3 * 60; ++i) { w.setVehicleInput(id, 0, 0, 0); w.update(1.0 / 60.0); }
        const Real after = w.vehicleVelocity(id).length();
        std::printf("    %-10s %.0f -> %.0f km/h after 3 s off the throttle\n", recipe, before * 3.6, after * 3.6);
        w.shutdown();
        return after / std::max(before, Real(0.1));
    };
    CHECK(coastKeeps("bicycle") > 0.7);
}
