// The fleet v2 recipes (assets/scripts/vehicle_kit.lua on the poly.* kit, ADR-0139): every spec builds a
// closed body, inside a triangle budget per LOD, with no sliver faces (Glenn: "we definitely don't want
// wasted triangles in the runtime"): none under roundness 0.015, a 1:200 strip), and the square-windowed
// ones really carry corners.
#include "test_framework.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/script_assets.h"   // makeModuleSource

#include <cstdio>
#include <string>

using namespace engine;

namespace {
struct KitVM {
    ScriptVM vm;
    bool ok = false;
    KitVM() {
        openProcgenLibrary(vm);
        openModuleLoader(vm, makeModuleSource(std::string(RT_SOURCE_DIR) + "/assets/scripts"));
        std::string err;
        ok = vm.doString("kit = require 'vehicle_kit'", &err);
        if (!ok) std::printf("    vehicle_kit load error: %s\n", err.c_str());
    }
};
}  // namespace

TEST_CASE(fleet_v2_every_body_is_closed_lean_and_sliver_free) {
    KitVM k;
    CHECK(k.ok);
    std::string err;
    const bool ran = k.vm.doString(R"LUA(
      report = {}
      local names = { "sedan", "taxi", "hatchback", "suv", "jeep", "convertible", "pickup", "step_van", "small_truck", "fire_engine", "semi", "semi_tanker" }
      for _, name in ipairs(names) do
        local P = kit.SPECS[name]()
        local cage, s1 = kit.poly(P, 0)   -- the faceted fleet (level 0)
        local st = s1:stats(0.015)
        local cs = cage:stats(0.015)
        report[#report + 1] = { name = name, closed = cage:closed(), tris = st.triangles, slivers = st.slivers,
                                worst = st.worst, corners = cs.corners, cage_worst = cs.worst,
                                at = cs.worst_at, grp = cs.worst_group, cage_slivers = cs.slivers,
                                at1 = st.worst_at, grp1 = st.worst_group, pts1 = st.worst_pts, cpts = cs.worst_pts }
        if name == "convertible" then
          for _, q in ipairs(st.worst_pts or {}) do print(string.format("      lod1 worst pt %.4f %.4f %.4f", q[1], q[2], q[3])) end
        end
      end
    )LUA", &err);
    if (!ran) std::printf("    run error: %s\n", err.c_str());
    CHECK(ran);
    if (!ran) return;
    const bool checked = k.vm.doString(R"LUA(
      failures = 0
      for _, r in ipairs(report) do
        print(string.format("    %-12s closed %-5s lod0 %6d tris  slivers %d  worst %.3f in %s at (%.2f %.2f %.2f)  corners %d | cage slivers %d worst %.4f in %s", r.name,
                            tostring(r.closed), r.tris, r.slivers, r.worst, r.grp1 or "?",
                            r.at1 and r.at1[1] or 0, r.at1 and r.at1[2] or 0, r.at1 and r.at1[3] or 0,
                            r.corners, r.cage_slivers, r.cage_worst, r.grp or "?"))
        if not r.closed then failures = failures + 1 end
        if r.tris > 6000 then failures = failures + 1 end         -- faceted traffic budget per body
        if r.slivers > 0 then failures = failures + 1 end
        if (r.name == "jeep" or r.name == "step_van") and r.corners == 0 then failures = failures + 1 end
      end
      assert(failures == 0, failures .. " fleet body checks failed")
    )LUA", &err);
    if (!checked) std::printf("    %s\n", err.c_str());
    CHECK(checked);
}

// THE FIRE ENGINE (Glenn, 2026-10-13: "a fire truck should be an actual truck"): a kit vehicle like the rest of the
// fleet, so the sim parks it in the fire station's bays and a player can drive it. A US Type 1 pumper's size: 9-10 m
// long, about 2.5 m wide, 3.1-3.4 m to the light bar; dual wheels on its rear axle.
TEST_CASE(the_fire_engine_is_fire_engine_sized) {
    KitVM k;
    CHECK(k.ok);
    std::string err;
    const bool ran = k.vm.doString(R"LUA(
      local car = kit.build(kit.SPECS.fire_engine(), 0)
      fe = { L = car.size[3], W = car.size[1], H = car.size[2], wheels = #car.wheels, duals = 0 }
      for _, w in ipairs(car.wheels) do if w.dual then fe.duals = fe.duals + 1 end end
      print(string.format("    [fire engine] %.2f x %.2f x %.2f m, %d wheel stations, %d dual", fe.L, fe.W, fe.H, fe.wheels, fe.duals))
      assert(fe.L > 9.0 and fe.L < 10.0, "length")
      assert(fe.W > 2.4 and fe.W < 2.62, "width")
      assert(fe.H > 3.1 and fe.H < 3.4, "height")
      assert(fe.wheels == 4 and fe.duals == 2, "wheels")
    )LUA", &err);
    if (!ran) std::printf("    %s\n", err.c_str());
    CHECK(ran);
}
