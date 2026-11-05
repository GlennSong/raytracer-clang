// THE ROLLOVER PROBE (Glenn, 2026-11-05: "I think the cars are also too light. They should hug the road a little more.
// The big yellow SUV topples over easily and is unstable when taking turns"). Every four-wheeled drivable recipe, as the
// player gets it (vehicles.lua through loadVehicleSpec), brought to a speed on flat ground and then held at full lock
// through the player's own steering shaping (vehicle_steering.h) with the throttle on: a hard turn, the worst a player
// does on purpose. It may slide; it must not go over.
#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/script_assets.h"
#include "../src/engine/scripting/vehicle_spec.h"
#include "../src/engine/vehicle_steering.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;

namespace {

struct Turn { Real minUp = 1, maxRollDeg = 0; };

// up to `kmh`, then 5 s of full lock through the shaping, throttle held at 0.6
Turn hardTurn(const PhysicsWorld::VehicleConfig& cfg, Real kmh) {
    PhysicsWorld w;
    w.initialize();
    w.addBox(Vec3(4000, 1, 4000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static);
    const PhysicsWorld::VehicleId id = w.addVehicle(cfg, Vec3(0, 1.5, 0), Quat::identity());
    w.optimizeBroadPhase();
    const Real dt = 1.0 / 60.0;
    for (int i = 0; i < 30 * 60 && w.vehicleVelocity(id).length() < kmh / 3.6; ++i) {
        w.setVehicleInput(id, 1.0, 0, 0);
        w.update(dt);
    }
    Turn t;
    SteerShaper shaper;
    for (int i = 0; i < 5 * 60; ++i) {
        const Vec3 fwd = w.vehicleOrientation(id).rotate(Vec3(0, 0, 1));
        const Real steer = shaper.step(1.0, dot(w.vehicleVelocity(id), fwd), dt);
        w.setVehicleInput(id, 0.6, steer, 0);
        w.update(dt);
        const Vec3 up = w.vehicleOrientation(id).rotate(Vec3(0, 1, 0));
        t.minUp = std::min(t.minUp, up.y);
        t.maxRollDeg = std::max(t.maxRollDeg, std::acos(std::clamp(up.y, Real(-1), Real(1))) * 57.29578);
    }
    w.shutdown();
    return t;
}

// sliding sideways at `ms` into a 0.15 m kerb (a lot's, a sidewalk's): the trip that rolls a car
Turn kerbTrip(const PhysicsWorld::VehicleConfig& cfg, Real ms) {
    PhysicsWorld w;
    w.initialize();
    w.addBox(Vec3(4000, 1, 4000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static);
    w.addBox(Vec3(1, 0.075, 50), Vec3(6, 0.075, 0), Quat::identity(), BodyMotion::Static);
    const PhysicsWorld::VehicleId id = w.addVehicle(cfg, Vec3(0, 1.5, 0), Quat::identity());
    w.optimizeBroadPhase();
    const Real dt = 1.0 / 60.0;
    for (int i = 0; i < 60; ++i) { w.setVehicleInput(id, 0, 0, 0); w.update(dt); }
    w.setLinearVelocity(w.vehicleBody(id), Vec3(ms, 0, 0));
    Turn t;
    for (int i = 0; i < 3 * 60; ++i) {
        w.setVehicleInput(id, 0, 0, 0);
        w.update(dt);
        const Vec3 up = w.vehicleOrientation(id).rotate(Vec3(0, 1, 0));
        t.minUp = std::min(t.minUp, up.y);
        t.maxRollDeg = std::max(t.maxRollDeg, std::acos(std::clamp(up.y, Real(-1), Real(1))) * 57.29578);
    }
    w.shutdown();
    return t;
}

// a hard turn across a 12 % side slope (a hill town's cross street, a cambered mountain road), downhill to the outside
Turn slopeTurn(const PhysicsWorld::VehicleConfig& cfg, Real kmh) {
    PhysicsWorld w;
    w.initialize();
    const Quat tilt = Quat::fromAxisAngle(Vec3(0, 0, 1), std::atan(0.12));
    w.addBox(Vec3(4000, 1, 4000), Vec3(0, -1, 0), tilt, BodyMotion::Static);
    const PhysicsWorld::VehicleId id = w.addVehicle(cfg, Vec3(0, 1.5, 0), tilt);
    w.optimizeBroadPhase();
    const Real dt = 1.0 / 60.0;
    for (int i = 0; i < 30 * 60 && w.vehicleVelocity(id).length() < kmh / 3.6; ++i) {
        w.setVehicleInput(id, 1.0, 0, 0);
        w.update(dt);
    }
    Turn t;
    SteerShaper shaper;
    for (int i = 0; i < 4 * 60; ++i) {
        const Vec3 fwd = w.vehicleOrientation(id).rotate(Vec3(0, 0, 1));
        const Real steer = shaper.step(-1.0, dot(w.vehicleVelocity(id), fwd), dt);
        w.setVehicleInput(id, 0.6, steer, 0);
        w.update(dt);
        const Vec3 up = w.vehicleOrientation(id).rotate(Vec3(0, 1, 0));
        t.minUp = std::min(t.minUp, up.y);
        t.maxRollDeg = std::max(t.maxRollDeg, std::acos(std::clamp(up.y, Real(-1), Real(1))) * 57.29578);
    }
    w.shutdown();
    return t;
}

}  // namespace

TEST_CASE(no_drivable_car_tips_over_in_a_hard_turn) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(std::string(RT_SOURCE_DIR) + "/assets/scripts"));
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/scripts/vehicles.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    std::string err;
    CHECK(vm.doString(ss.str(), &err));
    int tipped = 0;
    for (const char* recipe : {"offroad", "kit_jeep", "kit_pickup", "kit_suv", "kit_sedan", "kit_hatchback", "kit_taxi", "kit_convertible",
                               "kit_step_van", "kit_small_truck", "sedan", "pickup", "van"}) {
        VehicleSpec spec;
        if (!loadVehicleSpec(vm, std::string("return vehicle.") + recipe + "(seed, {})", 1u, spec, &err)) {
            std::printf("    [rollover] %s: %s\n", recipe, err.c_str());
            CHECK(false);
            continue;
        }
        const PhysicsWorld::VehicleConfig& c = spec.config;
        std::printf("    [rollover] %-16s %5.0f kg, com %+.2f of a %.2f m half height:", recipe, c.mass, c.comOffsetY, c.chassisHalfExtent.y);
        for (const Real kmh : {40.0, 60.0, 80.0}) {
            const Turn t = hardTurn(c, kmh);
            const bool over = t.minUp < 0.5;
            tipped += over ? 1 : 0;
            std::printf("  %2.0f km/h roll %4.1f%s", kmh, t.maxRollDeg, over ? " OVER" : "");
        }
        const Turn k = kerbTrip(c, 8.0), sl = slopeTurn(c, 50.0);
        tipped += (k.minUp < 0.5 ? 1 : 0) + (sl.minUp < 0.5 ? 1 : 0);
        std::printf(" | kerb at 8 m/s roll %4.1f%s | 12%% slope at 50 roll %4.1f%s\n", k.maxRollDeg, k.minUp < 0.5 ? " OVER" : "", sl.maxRollDeg,
                    sl.minUp < 0.5 ? " OVER" : "");
    }
    CHECK(tipped == 0);
}
