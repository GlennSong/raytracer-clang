// THE OFF-ROAD COURSE, DRIVEN (#41, ADR-0141). physics_tests: it needs Jolt and the Lua vehicle catalogue.
#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/script_assets.h"
#include "../src/engine/scripting/vehicle_spec.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
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
// (assets/levels/offroad_course.json: ground, ledges, rock garden, twister), full throttle -- the key is
// digital -- down each lane from before the ledges to past the twister. The first cut failed twice over:
// boulders up to 1.35 m (walls), and a square collision box whose floor sat 14 cm under the drawn belly and a
// full overhang ahead of the tyres, so the bumper caught what the wheels could climb.
namespace {
struct CourseRun { Real reachedZ; bool through; };
CourseRun driveCourseLane(const VehicleSpec& spec, bool fourWheel, Real laneX, Real startZ = -30.0,
                          Real goalZ = -142.0, Real seconds = 45.0) {
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
        w.setVehicleInput(id, 1.0, std::clamp(want + headingErr * 1.5, Real(-1), Real(1)) * -1.0, 0);
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
        through4 += r4.through;
    }
    CHECK(through4 == 4);
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
