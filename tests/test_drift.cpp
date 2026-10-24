// THE DRIFT (ADR-0236): a sedan thrown into a slide by a handbrake tap while steering at speed holds a slip angle
// set by the steering, keeps most of its speed on the throttle, turns hard, and grips again once straightened.
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

TEST_CASE(a_handbrake_flick_throws_a_car_into_a_held_drift_and_it_grips_again) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(std::string(RT_SOURCE_DIR) + "/assets/scripts"));
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/scripts/vehicles.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    std::string err;
    CHECK(vm.doString(ss.str(), &err));
    VehicleSpec spec;
    CHECK(loadVehicleSpec(vm, "return vehicle.kit_sedan(seed, {})", 1u, spec, &err));
    PhysicsWorld w;
    w.initialize();
    w.addBox(Vec3(3000, 1, 3000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static);
    const PhysicsWorld::VehicleId id = w.addVehicle(spec.config, Vec3(0, 1.2, 0), Quat::identity());
    w.setVehicleDriftEnabled(id, true);
    w.optimizeBroadPhase();
    const Real dt = 1.0 / 60.0;
    auto yawOf = [&]() {
        const Vec3 fw = w.vehicleOrientation(id).rotate(Vec3(0, 0, 1));
        return std::atan2(fw.x, fw.z);
    };
    // up to ~75 km/h
    for (int i = 0; i < 20 * 60 && w.vehicleVelocity(id).length() < 21.0; ++i) {
        w.setVehicleInput(id, 1.0, 0, 0);
        w.update(dt);
    }
    const Real entry = w.vehicleVelocity(id).length();
    CHECK(entry > 19.0);
    // the flick: steer right, handbrake for a beat
    for (int i = 0; i < 12; ++i) { w.setVehicleInput(id, 0.6, 0.8, 0, 1.0); w.update(dt); }
    CHECK(w.vehicleDrift(id).drifting);
    // hold it: steer 0.8, throttle on, 3 s
    const Real yaw0 = yawOf();
    Real turned = 0, last = yaw0, slipSum = 0, maxSlip = 0;
    int n = 0;
    for (int i = 0; i < 3 * 60; ++i) {
        w.setVehicleInput(id, 0.8, 0.8, 0, 0);
        w.update(dt);
        const Real y = yawOf();
        turned += std::remainder(y - last, 2 * 3.14159265358979);
        last = y;
        const PhysicsWorld::DriftState d = w.vehicleDrift(id);
        if (i > 60) { slipSum += std::fabs(d.slipDeg); ++n; }
        maxSlip = std::max(maxSlip, std::fabs(d.slipDeg));
    }
    const Real held = w.vehicleVelocity(id).length();
    const PhysicsWorld::DriftState mid = w.vehicleDrift(id);
    std::printf("    entry %.0f km/h, held %.0f km/h, slip %.0f deg average (max %.0f), turned %.0f deg, still drifting %d, "
                "boost %.2f\n", entry * 3.6, held * 3.6, slipSum / std::max(n, 1), maxSlip,
                std::fabs(turned) * 57.2958, mid.drifting, mid.boost);
    CHECK(mid.drifting);
    CHECK(slipSum / std::max(n, 1) > 12 && slipSum / std::max(n, 1) < 45);   // a slide, held near what's asked
    CHECK(maxSlip < 80);                                                       // not a spin
    CHECK(held > entry * 0.55);                                                // the speed held through it
    CHECK(std::fabs(turned) > 1.2);                                            // ~70°+ of turn
    CHECK(mid.boost > 0.5);
    // straighten up: it grips again
    for (int i = 0; i < 90; ++i) { w.setVehicleInput(id, 0.5, 0, 0, 0); w.update(dt); }
    CHECK(!w.vehicleDrift(id).drifting);
    CHECK(std::fabs(w.vehicleDrift(id).slipDeg) < 12);
    w.shutdown();
}
