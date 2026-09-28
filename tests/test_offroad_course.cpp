// THE OFF-ROAD COURSE, DRIVEN (#41, ADR-0141). physics_tests: it needs Jolt and the Lua vehicle catalogue.
#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/script_assets.h"
#include "../src/engine/scripting/vehicle_spec.h"
#include "../src/engine/vehicle_steering.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;

namespace {
struct VehiclesVM {
    ScriptVM vm;
    bool loaded = false;
    VehiclesVM() {
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

// THE COURSE, DRIVEN (Glenn, after the first drive: "if I go over slabs and rocks even with 4WD I get stuck. I
// thought the idea was that I didn't."). The REAL vehicle.offroad spec on the REAL course geometry
// (assets/levels/offroad_course.json: ground, ledges, rock garden, twister, logs), full throttle -- the key is
// digital -- down each lane from before the ledges to past the logs. (The logs were added after Glenn parked
// across a 0.56 m one: the body on it, the tyres unloaded, spinning in place both ways.) The first cut failed twice over:
// boulders up to 1.35 m (walls), and a square collision box whose floor sat 14 cm under the drawn belly and a
// full overhang ahead of the tyres, so the bumper caught what the wheels could climb.
namespace {
struct CourseRun { Real reachedZ; bool through; };
CourseRun driveCourseLane(const VehicleSpec& spec, bool fourWheel, Real laneX, Real startZ = -30.0,
                          Real goalZ = -170.0, Real seconds = 45.0, Real throttle = 1.0) {
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/levels/offroad_course.json");
    const nlohmann::json level = nlohmann::json::parse(f);
    PhysicsWorld w;
    w.initialize();
    for (const auto& e : level["entities"]) {
        const std::string shape = e["shape"];
        const Vec3 pos(e["position"][0], e["position"][1], e["position"][2]);
        Quat q = Quat::identity();
        if (e.contains("orientation")) {
            const auto& o = e["orientation"];
            q = Quat::fromAxisAngle(Vec3(o["axis"][0], o["axis"][1], o["axis"][2]),
                                    Real(o["angleDeg"]) * 3.14159265358979 / 180.0);
        }
        const Real mu = e["physics"]["friction"];
        if (shape == "box")
            w.addBox(Vec3(e["size"][0], e["size"][1], e["size"][2]) * 0.5, pos, q, BodyMotion::Static, 0.0, mu);
        else if (shape == "sphere")
            w.addSphere(e["size"][0], pos, q, BodyMotion::Static, 0.0, mu);
        else if (shape == "capsule")   // size = {r, length, r}; the length along the capsule's axis, caps included
            w.addCapsule(Real(e["size"][1]) * 0.5 - Real(e["size"][0]), e["size"][0], pos, q, BodyMotion::Static, 0.0, mu);
    }
    w.optimizeBroadPhase();
    PhysicsWorld::VehicleConfig cfg = spec.config;
    cfg.frontDriveShare = fourWheel ? 0.5 : 0.0;
    // facing -z (the course runs that way), before the ledges
    const auto id = w.addVehicle(cfg, Vec3(laneX, 1.4, startZ), Quat::fromAxisAngle(Vec3(0, 1, 0), 3.14159265358979));
    for (int i = 0; i < 90; ++i) w.update(1.0 / 60.0);
    Real best = 0.0;
    for (int i = 0; i < static_cast<int>(60 * seconds); ++i) {
        // hold the lane: steer back toward laneX, as a driver would
        const Vec3 p = w.vehiclePosition(id);
        const Vec3 fwd = w.vehicleOrientation(id).rotate(Vec3(0, 0, 1));
        const Real want = std::clamp((p.x - laneX) * 0.4, Real(-0.5), Real(0.5));   // heading -z: +x error steers right
        const Real headingErr = fwd.x;   // facing -z, fwd.x > 0 means drifting toward +x
        w.setVehicleInput(id, throttle, std::clamp(want + headingErr * 1.5, Real(-1), Real(1)) * -1.0, 0);
        w.update(1.0 / 60.0);
        best = std::min(best, p.z);
        if (p.z < goalZ) return { p.z, true };
    }
    return { best, false };
}
}  // namespace

TEST_CASE(offroader_drives_the_course_ledges_rocks_and_twister_in_four_wheel_drive) {
    VehiclesVM v;
    CHECK(v.loaded);
    VehicleSpec spec;
    std::string err;
    CHECK(loadVehicleSpec(v.vm, "return vehicle.offroad(seed, {})", 3u, spec, &err));
    CHECK(spec.config.approachDegrees > 30.0);
    CHECK(spec.config.floorClearance > 0.35);
    int through4 = 0;
    // one lane per ledge height (0.2 0.3 0.4 0.5 m at x = -12 -4 4 12), each on through the rocks
    for (const Real x : {-12.0, -4.0, 4.0, 12.0}) {
        const CourseRun r4 = driveCourseLane(spec, true, x);
        std::printf("    lane x %+5.1f: 4WD %s (z %.1f)\n", x, r4.through ? "THROUGH" : "stuck", r4.reachedZ);
        std::fflush(stdout);
        through4 += r4.through;
    }
    CHECK(through4 == 4);
    // and a CRAWL over the logs (bellies catch at low speed, where no bounce carries them)
    const CourseRun crawl = driveCourseLane(spec, true, 0.0, -144.0, -170.0, 40.0, 0.35);
    std::printf("    logs at a crawl: %s (z %.1f)\n", crawl.through ? "OVER" : "stuck", crawl.reachedZ);
    CHECK(crawl.through);
    // THE HILLS, over the top: up the ramp, across the 6 m top, down the far side -- the crests are where a
    // long belly high-centres, which an endless test slope never asks. FROM REST AT THE STOP LINE (the course's
    // start for a traction test): with a run-up, momentum carries even 2WD over 30 degrees of mud, which tests speed, not
    // traction.
    struct Hill { Real x, deg; const char* ground; };
    const Hill hills[] = { {-60, 15, "dirt"}, {-51, 20, "dirt"}, {-42, 25, "dirt"}, {-33, 30, "dirt"}, {-24, 35, "dirt"},
                           {20, 10, "mud"}, {29, 15, "mud"}, {38, 20, "mud"}, {47, 25, "mud"}, {56, 30, "mud"} };
    bool dirt30 = false, mudAll4 = true, mud25in2 = true;
    for (const Hill& h : hills) {
        const Real a = h.deg * 3.14159265358979 / 180.0, run = 6.0 / std::tan(a);
        const Real beyond = -(2.0 * run + 6.0) - 3.0;   // past the foot of the down ramp
        const Real line = 3.2;   // the stop line at the foot (a mud hill's apron under the rear tyres)
        const CourseRun r2 = driveCourseLane(spec, false, h.x, line - 2.4, beyond, 25.0);
        const CourseRun r4 = driveCourseLane(spec, true, h.x, line - 2.4, beyond, 25.0);
        std::printf("    %2.0f deg %-4s: 2WD %s  4WD %s\n", h.deg, h.ground, r2.through ? "OVER " : "stuck",
                    r4.through ? "OVER" : "stuck");
        const bool mud = h.ground[0] == 'm';
        if (!mud && h.deg == 30) dirt30 = r4.through;
        if (mud) mudAll4 = mudAll4 && r4.through;
        if (mud && h.deg == 25) mud25in2 = r2.through;
    }
    // (35 degrees of dirt is the rung past the limit: neither mode climbs it from rest on an endless slope,
    // and on the course a run is decided at the crest)
    CHECK(dirt30);      // a 30 degree dirt hill, crest and all
    CHECK(mudAll4);     // every mud hill, to 30 degrees, in four-wheel drive...
    CHECK(!mud25in2);   // ...where two-wheel drive stalls on 25
}

// ONE AXLE HANGING (Glenn: "I have two wheels down (front) and they're not moving in 4wd mode ... if I try to
// reverse they don't spin"). The rear axle over a hole, its tyres in the air, the fronts on the dirt. A real
// transfer case locks the shafts, so the grounded wheels still turn; the traction split hands the hanging axle's
// torque to them (VehicleConfig::tractionSplit). The first cut's plain fixed split did not: the hanging rears spun up, the engine hit its limiter and cut its torque, and the
// grounded pair sat at 0 rad/s. (Whether the truck then gets OUT is geometry: here its tail drops below the
// hole's lip and wedges against the wall -- a winch job, not a drivetrain one -- so only the spin is asserted.)
namespace {
struct HoleRun { Real moved; std::vector<Real> spin; std::vector<char> contact0; };
HoleRun driveOutOfHole(PhysicsWorld::VehicleConfig cfg, Real throttle, bool print) {
    PhysicsWorld w;
    w.initialize();
    // dirt ahead (z > -1.0) and far behind (z < -4.0); a 1.5 m deep hole between, under the rear axle
    w.addBox(Vec3(20, 0.5, 20), Vec3(0, -0.5, 19.0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    w.addBox(Vec3(20, 0.5, 20), Vec3(0, -0.5, -24.0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    w.addBox(Vec3(20, 0.5, 1.5), Vec3(0, -2.0, -2.5), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    w.optimizeBroadPhase();
    const auto id = w.addVehicle(cfg, Vec3(0, 1.4, 0.4), Quat::identity());
    for (int i = 0; i < 120; ++i) w.update(1.0 / 60.0);
    HoleRun r;
    r.contact0 = w.vehicleTelemetry(id).wheelContact;
    const Vec3 p0 = w.vehiclePosition(id);
    for (int i = 0; i < 60 * 4; ++i) { w.setVehicleInput(id, throttle, 0, 0); w.update(1.0 / 60.0); }
    const auto t1 = w.vehicleTelemetry(id);
    r.moved = w.vehiclePosition(id).z - p0.z;
    r.spin = t1.wheelSpin;
    if (print) {
        std::printf("      contact at rest");
        for (char c : r.contact0) std::printf(" %d", c);
        std::printf("  | after 4 s: rpm %.0f spin", t1.rpm);
        for (Real sp : t1.wheelSpin) std::printf(" %+.1f", sp);
        std::printf("  moved %+.2f m\n", r.moved);
    }
    return r;
}
}  // namespace

TEST_CASE(offroader_with_an_axle_hanging_still_drives_the_wheels_on_the_ground) {
    VehiclesVM v;
    VehicleSpec spec;
    std::string err;
    CHECK(loadVehicleSpec(v.vm, "return vehicle.offroad(seed, {})", 3u, spec, &err));
    PhysicsWorld::VehicleConfig cfg = spec.config;
    cfg.frontDriveShare = 0.5;   // Z pressed
    std::printf("    4WD with the traction split:\n");
    const HoleRun fwd = driveOutOfHole(cfg, 1.0, true);
    int down = 0;
    for (char c : fwd.contact0) down += c;
    CHECK(fwd.contact0.size() == 4 && down == 2);   // one axle on the dirt, one hanging
    auto groundedSpin = [](const HoleRun& r) {
        Real least = 1e9;
        for (std::size_t i = 0; i < r.spin.size(); ++i)
            if (r.contact0[i]) least = std::min(least, std::fabs(r.spin[i]));
        return least;
    };
    CHECK(groundedSpin(fwd) > 5.0);   // the wheels on the ground are driven
    CHECK(cfg.tractionSplit);
    PhysicsWorld::VehicleConfig fixedSplit = cfg;
    fixedSplit.tractionSplit = false;
    std::printf("    4WD, a plain fixed 50/50 split (the first cut):\n");
    const HoleRun fixedRun = driveOutOfHole(fixedSplit, 1.0, true);
    CHECK(groundedSpin(fixedRun) < 1.0);   // what Glenn saw: starved
}


// ROLLING IT (Glenn: "how come I can't flip my car? ... if I go off a ramp with half my wheels on it I should be
// able to flip my vehicle", then "rolling over should be allowed for active drivers, everyone"). No car has a
// roll cone now (ADR-0087's 65 degrees held every one there). A ramp under one track, hit at speed.
namespace {
Real maxRollOffHalfRamp(PhysicsWorld::VehicleConfig cfg, Real kmh) {
    PhysicsWorld w;
    w.initialize();
    w.addBox(Vec3(60, 0.5, 200), Vec3(0, -0.5, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    // a 25 degree kicker, 1.2 m wide, under one track (x = +0.9) only
    const Real a = 25.0 * 3.14159265358979 / 180.0, len = 3.0;
    w.addBox(Vec3(0.6, 0.1, len * 0.5), Vec3(0.9, len * 0.5 * std::sin(a) - 0.1, 40.0),
             Quat::fromAxisAngle(Vec3(1, 0, 0), -a), BodyMotion::Static, 0.0, 0.85);
    w.optimizeBroadPhase();
    const auto id = w.addVehicle(cfg, Vec3(0, 1.4, 0), Quat::identity());
    for (int i = 0; i < 60; ++i) w.update(1.0 / 60.0);
    Real maxRoll = 0.0;
    for (int i = 0; i < 60 * 12; ++i) {
        const Vec3 p = w.vehiclePosition(id);
        const Real v = w.vehicleTelemetry(id).speed * 3.6;
        w.setVehicleInput(id, p.z < 38.0 && v < kmh ? 1.0 : 0.0, 0, p.z < 38.0 && v > kmh + 3 ? 0.5 : 0.0);
        w.update(1.0 / 60.0);
        const Vec3 up = w.vehicleOrientation(id).rotate(Vec3(0, 1, 0));
        maxRoll = std::max(maxRoll, std::acos(std::clamp(up.y, Real(-1), Real(1))) * 57.2958);
    }
    return maxRoll;
}
}  // namespace

TEST_CASE(cars_can_roll_over_off_a_half_ramp) {
    VehiclesVM v;
    VehicleSpec off, sedan;
    std::string err;
    CHECK(loadVehicleSpec(v.vm, "return vehicle.offroad(seed, {})", 3u, off, &err));
    CHECK(loadVehicleSpec(v.vm, "return vehicle.kit_sedan(seed, {})", 3u, sedan, &err));
    CHECK(off.config.maxPitchRollDegrees >= 180.0 && sedan.config.maxPitchRollDegrees >= 180.0);
    bool offRolled = false, sedanRolled = false;
    for (const Real kmh : {40.0, 60.0, 80.0}) {
        const Real r4 = maxRollOffHalfRamp(off.config, kmh), rs = maxRollOffHalfRamp(sedan.config, kmh);
        std::printf("    %2.0f km/h off a one-side kicker: off-roader max roll %5.1f deg, sedan %5.1f deg\n", kmh, r4, rs);
        offRolled = offRolled || r4 > 85.0;       // off its wheels: on its side or further
        sedanRolled = sedanRolled || rs > 85.0;
    }
    CHECK(offRolled);     // (the tall off-roader goes onto its roof at 80; the low sedan onto its side)
    CHECK(sedanRolled);
}

// CORNERING WITHOUT THE CONE (Glenn: "The sedan was rolling a lot when I would turn ... turning doesn't feel
// good"). Every drivable spec: full steering held from speed, and the two kerb trips the cone was added for.
// Prints the lean (max roll) and whether it went over.
namespace {
struct RollProbe { Real maxRoll = 0.0; bool over = false; };
RollProbe rollProbe(const PhysicsWorld::VehicleConfig& cfg, int kind, Real kmh) {
    PhysicsWorld w;
    w.initialize();
    w.addBox(Vec3(3000, 1, 3000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    if (kind == 1) {   // the slanted kerb
        const Quat k = Quat::fromAxisAngle(Vec3(0, 1, 0), 25.0 * 3.14159265358979 / 180.0);
        w.addBox(Vec3(80, 0.075, 40), k.rotate(Vec3(0, 0, 40)) + Vec3(0, 0.075, 60), k, BodyMotion::Static, 0.0, 0.85);
    } else if (kind == 2) {   // a kerb face across a sideways slide
        w.addBox(Vec3(40, 0.075, 40), Vec3(46, 0.075, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    }
    w.optimizeBroadPhase();
    const auto id = w.addVehicle(cfg, Vec3(0, 1.2, 0), Quat::identity());
    for (int i = 0; i < 90; ++i) w.update(1.0 / 60.0);
    const Real v = kmh / 3.6;
    w.setLinearVelocity(w.vehicleBody(id), kind == 2 ? Vec3(v, 0, 0) : Vec3(0, 0, v));
    RollProbe r;
    for (int i = 0; i < 60 * 5; ++i) {
        if (kind == 0) w.setVehicleInput(id, 0.3, 1.0, 0);        // full lock, a little throttle
        else if (kind == 1) w.setVehicleInput(id, 0.3, 0, 0);
        else w.setVehicleInput(id, 0, 0, 0);
        w.update(1.0 / 60.0);
        const Vec3 up = w.vehicleOrientation(id).rotate(Vec3(0, 1, 0));
        const Real tilt = std::acos(std::clamp(up.y, Real(-1), Real(1))) * 57.2958;
        r.maxRoll = std::max(r.maxRoll, tilt);
        if (tilt > 85.0) r.over = true;
    }
    return r;
}
}  // namespace

TEST_CASE(drivable_cars_probe_cornering_roll_prints) {
    VehiclesVM v;
    for (const char* recipe : {"sedan", "kit_sedan", "kit_hatchback", "kit_suv", "kit_step_van", "offroad"}) {
        VehicleSpec spec;
        std::string err;
        if (!loadVehicleSpec(v.vm, std::string("return vehicle.") + recipe + "(seed, {})", 3u, spec, &err)) continue;
        std::printf("    %-14s turn:", recipe);
        for (const Real kmh : {40.0, 60.0, 80.0, 100.0, 120.0}) {
            const RollProbe r = rollProbe(spec.config, 0, kmh);
            std::printf(" %3.0f:%4.1f%s", kmh, r.maxRoll, r.over ? "OVER" : "");
        }
        const RollProbe k1 = rollProbe(spec.config, 1, 65.0), k2 = rollProbe(spec.config, 2, 29.0);
        std::printf("  | slanted kerb 65: %4.1f%s  side trip 29: %4.1f%s\n", k1.maxRoll, k1.over ? "OVER" : "",
                    k2.maxRoll, k2.over ? "OVER" : "");
    }
}

// BRAKES AND TURNING (Glenn: "the brakes are really soft and the turn radius feels really wide"). Each drivable
// spec, through the player's own input path (the keyboard steering assist, vehicle_steering.h):
//   stop     from 100 km/h: Space (brake pedal = 1) and S (throttle -1), metres to a stop. A real car: ~36-40 m.
//   radius   full key held at a steady speed: the circle's radius (speed / yaw rate). A real sedan turns in a
//            ~5.5 m radius at walking pace; the tyres cap it near v^2 / (0.9 g) at speed.
namespace {
PhysicsWorld::VehicleId flatCar(PhysicsWorld& w, const PhysicsWorld::VehicleConfig& cfg) {
    w.initialize();
    w.addBox(Vec3(4000, 1, 4000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    w.optimizeBroadPhase();
    const auto id = w.addVehicle(cfg, Vec3(0, 1.2, 0), Quat::identity());
    for (int i = 0; i < 90; ++i) w.update(1.0 / 60.0);
    return id;
}
Real stopDistance(const PhysicsWorld::VehicleConfig& cfg, bool pedal) {
    PhysicsWorld w;
    const auto id = flatCar(w, cfg);
    for (int i = 0; i < 60 * 30 && w.vehicleTelemetry(id).speed < 100.0 / 3.6; ++i) {
        w.setVehicleInput(id, 1.0, 0, 0);
        w.update(1.0 / 60.0);
    }
    const Vec3 p0 = w.vehiclePosition(id);
    for (int i = 0; i < 60 * 30 && w.vehicleTelemetry(id).speed > 0.3; ++i) {
        w.setVehicleInput(id, pedal ? 0.0 : -1.0, 0, pedal ? 1.0 : 0.0);
        w.update(1.0 / 60.0);
    }
    return (w.vehiclePosition(id) - p0).length();
}
Real turnRadius(const PhysicsWorld::VehicleConfig& cfg, Real kmh) {
    PhysicsWorld w;
    const auto id = flatCar(w, cfg);
    Real steer = 0.0;
    Real yawRate = 0.0, speed = 0.0;
    for (int i = 0; i < 60 * 14; ++i) {
        speed = w.vehicleTelemetry(id).speed;
        const bool turning = i > 60 * 6;
        const Real thr = std::clamp((kmh / 3.6 - speed) * 0.5, Real(-0.3), Real(1.0));
        steer = shapeSteer(steer, turning ? 1.0 : 0.0, speed, 1.0 / 60.0);
        const Vec3 f0 = w.vehicleOrientation(id).rotate(Vec3(0, 0, 1));
        w.setVehicleInput(id, thr, steer, 0);
        w.update(1.0 / 60.0);
        const Vec3 f1 = w.vehicleOrientation(id).rotate(Vec3(0, 0, 1));
        if (i > 60 * 11) yawRate = 0.9 * yawRate + 0.1 * std::fabs(std::atan2(f0.x * f1.z - f0.z * f1.x, f0.x * f1.x + f0.z * f1.z)) * 60.0;
    }
    return yawRate > 1e-4 ? std::fabs(speed) / yawRate : 1e9;
}
}  // namespace

TEST_CASE(drivable_cars_probe_brakes_and_turning_prints) {
    VehiclesVM v;
    for (const char* recipe : {"sedan", "kit_sedan", "kit_suv", "kit_convertible", "offroad"}) {
        VehicleSpec spec;
        std::string err;
        if (!loadVehicleSpec(v.vm, std::string("return vehicle.") + recipe + "(seed, {})", 3u, spec, &err)) continue;
        std::printf("    %-16s stop 100->0: Space %5.1f m  S %5.1f m  | radius:", recipe, stopDistance(spec.config, true),
                    stopDistance(spec.config, false));
        for (const Real kmh : {10.0, 30.0, 50.0, 80.0})
            std::printf("  %2.0f km/h %5.1f m", kmh, turnRadius(spec.config, kmh));
        std::printf("   [brake %.0f Nm, steer %.0f deg, mass %.0f]\n", spec.config.brakeTorque, spec.config.maxSteerDegrees, spec.config.mass);
    }
}

