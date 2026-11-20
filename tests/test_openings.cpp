// The modelling libraries written in Lua over the mesh words (#191): openings.lua, a hole through a wall with a
// frame and a pane; truss.lua, a lattice of members along a path.
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

TEST_CASE(truss_builds_a_lattice_along_a_path) {
    ProcModel m;
    std::string err;
    const bool ok = runModel(R"LUA(
        local truss = require "truss"
        local function near(a, b, eps) return math.abs(a - b) < (eps or 1e-6) end

        -- a square mast 8 m up in 1 m bays: 4 chords, a ring of 4 at each of 9 frames, a diagonal on each of 4
        -- sides in each of 8 bays
        local mast, info = truss.build{ path = { { 0, 0, 0 }, { 0, 8, 0 } }, up = { 0, 0, 1 }, bay = 1.0, size = { 1.2, 1.2 },
                                        chord = 0.05, brace = 0.02 }
        assert(info.bays == 8 and info.chords == 4 and info.frames == 36 and info.diagonals == 32)
        assert(near(info.length, 8))
        assert(#info.corners == 9 and #info.corners[1] == 4)
        -- the joints are where the section's corners are: 0.6 m out at the foot, 8 m up at the top
        local foot, top = info.corners[1][1], info.corners[9][3]
        assert(near(math.abs(foot[1]), 0.6) and near(math.abs(foot[3]), 0.6) and near(foot[2], 0))
        assert(near(top[2], 8))
        local audit = mesh.audit(mast)
        assert(audit.tris > 0 and audit.degenerate == 0)

        -- crossed diagonals are two a side; none is none; no braces is chords only
        local _, cross = truss.build{ path = { { 0, 0, 0 }, { 0, 0, 4 } }, bay = 1.0, pattern = "cross" }
        assert(cross.diagonals == 4 * 4 * 2)
        local _, bare = truss.build{ path = { { 0, 0, 0 }, { 0, 0, 4 } }, bay = 1.0, pattern = "none", frames = false }
        assert(bare.diagonals == 0 and bare.frames == 0 and bare.chords == 4)
        local _, chords = truss.build{ path = { { 0, 0, 0 }, { 0, 0, 4 } }, bay = 1.0, brace = 0 }
        assert(chords.diagonals == 0 and chords.frames == 0)

        -- a taper: the tip's section is the end size; a triangle has three chords
        local _, boom = truss.build{ path = { { 0, 0, 0 }, { 0, 0, 6 } }, bay = 1.5, section = "triangle", size = { 1, 1 }, size_end = { 0.2, 0.2 } }
        assert(boom.bays == 4 and boom.chords == 3)
        local tip = boom.corners[5]
        assert(near(tip[2][1] - tip[1][1], 0.2), "the tip is 0.2 m across")
        assert(near(boom.corners[1][2][1] - boom.corners[1][1][1], 1.0), "the foot is 1 m across")

        -- a bent path keeps its length in bays and does not twist: the section's first corner stays on one side
        local _, bent = truss.build{ path = { { 0, 0, 0 }, { 0, 2, 3 }, { 0, 2, 7 } }, bay = 1.0 }
        assert(near(bent.length, math.sqrt(13) + 4))
        for i = 1, #bent.corners do assert(bent.corners[i][1][1] < 0, "corner 1 stays on -x") end

        -- what is wrong is said
        local function refused(o, says)
          local ok, why = pcall(truss.build, o)
          assert(not ok and tostring(why):find(says, 1, true), tostring(why))
        end
        refused({}, "give path")
        refused({ path = { { 0, 0, 0 }, { 0, 0, 0 } } }, "two different points")
        refused({ path = { { 0, 0, 0 }, { 0, 5, 0 } } }, "runs along `up`")
        refused({ path = { { 0, 0, 0 }, { 0, 0, 5 } }, pattern = "howe" }, "pattern is")
        refused({ path = { { 0, 0, 0 }, { 0, 0, 5 } }, section = "hexagon" }, "section is")

        local out = model.new()
        out:add(mast)
        return out
    )LUA", m, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    CHECK(!m.parts.empty());
}
