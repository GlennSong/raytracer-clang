// The openings library (assets/scripts/openings.lua, #191): a hole through a wall with a frame and a pane.
#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include "../src/engine/procgen/proc_model.h"
#include "../src/engine/script_assets.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/scripting/script_vm.h"
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

bool runModel(const std::string& code, ProcModel& out, std::string* error = nullptr) {
    ScriptVM vm;
    openProcgenLibrary(vm);
#ifdef RT_SOURCE_DIR
    openModuleLoader(vm, makeModuleSource(std::string(RT_SOURCE_DIR) + "/assets/scripts"));
#else
    openModuleLoader(vm, makeModuleSource("assets/scripts"));
#endif
    return runProcgenModelValue(vm, code, out, error);
}

}  // namespace

TEST_CASE(openings_cut_a_wall_and_give_back_a_frame_and_a_pane) {
    ProcModel m;
    std::string err;
    const bool ok = runModel(R"LUA(
        local openings = require "openings"
        local wall = poly.box{ 6, 3, 0.25 }
        wall:translate{ 0, 1.5, 0 }
        local function near(a, b) return math.abs(a - b) < 1e-6 end

        -- a window: the wall stays one closed solid, less exactly the opening
        local w = openings.window(wall, { at = { -1.8, 1.6, 0.125 }, normal = { 0, 0, 1 }, width = 1.2, height = 1.3,
                                          thickness = 0.25, frame = 0.07, set_back = 0.06, bars = { 1, 1 } })
        assert(wall:closed(), "closed after a window")
        assert(near(wall:volume(), 4.5 - 1.2 * 1.3 * 0.25), "the wall lost the opening's volume: " .. wall:volume())
        -- the frame fills the opening's outline and has its own hole; the pane sits inside the frame, in the wall
        assert(w.frame and w.pane)
        local lo, hi = w.frame:bounds()
        assert(near(hi[1] - lo[1], 1.2) and near(hi[2] - lo[2], 1.3), "the frame is the opening's size")
        assert(lo[3] > -0.125 and hi[3] < 0.125 + 1e-9, "the frame is inside the wall's thickness")
        assert(near(hi[3], 0.125 - 0.06), "its front is set back 0.06 from the face")
        local plo, phi = w.pane:bounds()
        assert(plo[1] > lo[1] and phi[1] < hi[1] and plo[2] > lo[2] and phi[2] < hi[2], "the pane is inside the frame")
        assert(w.pane:closed())

        -- a second window at the same height (its cuts run through the first one's corners), and a doorway down to
        -- the floor, which is open at the bottom
        openings.window(wall, { at = { 0.2, 1.6, 0.125 }, normal = { 0, 0, 1 }, width = 1.2, height = 1.3, thickness = 0.25 })
        assert(wall:closed(), "closed after a second window")
        openings.hole(wall, { at = { 2.1, 1.05, 0.125 }, normal = { 0, 0, 1 }, width = 0.95, height = 2.1, thickness = 0.25 })
        assert(wall:closed(), "closed after a doorway")
        assert(near(wall:volume(), 4.5 - 2 * 1.2 * 1.3 * 0.25 - 0.95 * 2.1 * 0.25), "the wall less three openings: " .. wall:volume())

        -- no frame, no pane, when asked for none
        local bare = openings.window(wall, { at = { -1.8, 0.5, 0.125 }, normal = { 0, 0, 1 }, width = 0.5, height = 0.4,
                                             thickness = 0.25, frame = 0, pane = 0 })
        assert(bare.frame == nil and bare.pane == nil)
        assert(wall:closed())

        -- an opening in a wall that faces another way, cut from its other side: a wall along z, looking down -x
        local side = poly.box{ 0.3, 3, 4 }
        local s = openings.window(side, { at = { -0.15, 0.2, 0.5 }, normal = { -1, 0, 0 }, width = 1.0, height = 0.8, thickness = 0.3 })
        assert(side:closed(), "closed, cut from the -x side")
        assert(near(side:volume(), 0.3 * 3 * 4 - 1.0 * 0.8 * 0.3))
        local flo, fhi = s.frame:bounds()
        assert(near(fhi[3] - flo[3], 1.0) and near(fhi[2] - flo[2], 0.8), "the frame's width runs along the wall")

        -- what is wrong is said
        local ok1, why1 = pcall(openings.hole, wall, { at = { 0, 1, 0.125 }, normal = { 0, 0, 1 }, width = 1, height = 1 })
        assert(not ok1 and why1:find("give at, normal, width, height and thickness"), tostring(why1))
        local ok2, why2 = pcall(openings.hole, wall, { at = { 0, 9, 0.125 }, normal = { 0, 0, 1 }, width = 1, height = 1, thickness = 0.25 })
        assert(not ok2 and why2:find("no wall face"), tostring(why2))

        local out = model.new()
        for _, part in pairs(wall:to_parts(20)) do out:add(part) end
        return out
    )LUA", m, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    CHECK(!m.parts.empty());
}
