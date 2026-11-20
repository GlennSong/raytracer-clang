// The modelling libraries written in Lua over the mesh words (#191): openings.lua, a hole through a wall with a
// frame and a pane; truss.lua, a lattice of members along a path; scatter.lua, where to put many copies of a piece.
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

TEST_CASE(scatter_gives_placements_that_stand_each_copy_along_the_surface) {
    ProcModel m;
    std::string err;
    const bool ok = runModel(R"LUA(
        local scatter = require "scatter"
        local function near(a, b) return math.abs(a - b) < 1e-9 end
        -- a ring: `count` places, all `radius` from the middle, flat on the surface
        local ring = scatter.ring{ at = { 1, 2, 3 }, normal = { 0, 0, 1 }, radius = 0.5, count = 8 }
        assert(#ring == 8)
        for _, p in ipairs(ring) do
          local dx, dy, dz = p.pos[1] - 1, p.pos[2] - 2, p.pos[3] - 3
          assert(near(math.sqrt(dx * dx + dy * dy), 0.5) and near(dz, 0) and near(p.up[3], 1))
        end
        -- along a line: both ends, or neither; round a closed outline, each corner once
        local line = { { 0, 0, 0 }, { 1, 0, 0 } }
        assert(#scatter.along{ path = line, spacing = 0.25 } == 5)
        assert(#scatter.along{ path = line, spacing = 0.25, ends = false } == 3)
        local square = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 1 }, { 0, 0, 1 } }
        local round = scatter.along{ path = square, spacing = 0.5, closed = true }
        assert(#round == 8)
        -- a grid: rows by columns, a margin in from the edges, standing along u x v
        local grid = scatter.grid{ origin = { 0, 0, 0 }, u = { 2, 0, 0 }, v = { 0, 0, -1 }, count = { 3, 2 }, margin = 0.1 }
        assert(#grid == 6)
        assert(near(grid[1].pos[1], 0.1) and near(grid[3].pos[1], 1.9) and near(grid[1].pos[3], -0.1) and near(grid[6].pos[3], -0.9))
        assert(near(grid[1].up[2], 1))
        -- the middles of chosen faces of a quad mesh
        local box = poly.box{ 2, 1, 1 }
        local tops = scatter.faces(box, box:select{ normal = { 0, 1, 0 }, min_dot = 0.9 })
        assert(#tops == 1 and near(tops[1].pos[2], 0.5) and near(tops[1].up[2], 1))
        assert(#scatter.faces(box, box:select{ all = true }) == 6)
        assert(#scatter.join(ring, grid) == 14)
        local ok1, why = pcall(scatter.ring, { at = { 0, 0, 0 } })
        assert(not ok1 and why:find("give at, radius and count"))

        -- used: one bolt, stored once, standing along each placement's `up`
        local out = model.new()
        local bolt = mesh.cylinder(0.02, 0.01, 6)
        out:use(bolt, { { pos = { 0, 0, 0 }, up = { 1, 0, 0 } }, { pos = { 0, 0, 0 }, up = { 0, -1, 0 } },
                        { pos = { 0, 0, 0 }, up = { 0, 3, 4 } }, { pos = { 0, 0, 0 } },
                        { pos = { 0, 0, 0 }, up = { 0, 1, 0 }, spin = 90 } })
        return out
    )LUA", m, &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    CHECK(m.uses.size() == 1);
    if (m.uses.size() != 1 || m.uses[0].placements.size() != 5) { CHECK(false); return; }
    auto upOf = [&](int i) {
        const Mat4 x = m.uses[0].placements[static_cast<std::size_t>(i)].matrix();
        return Vec3(x.m[0][1], x.m[1][1], x.m[2][1]);       // where the piece's own +y goes
    };
    CHECK((upOf(0) - Vec3(1, 0, 0)).length() < 1e-9);
    CHECK((upOf(1) - Vec3(0, -1, 0)).length() < 1e-9);     // straight down
    CHECK((upOf(2) - Vec3(0, 0.6, 0.8)).length() < 1e-9);  // any length of `up` will do
    CHECK((upOf(3) - Vec3(0, 1, 0)).length() < 1e-9);      // none given: as it was built
    // spin turns the piece about its own up: its +x goes round to -z
    const Mat4 spun = m.uses[0].placements[4].matrix();
    CHECK((Vec3(spun.m[0][0], spun.m[1][0], spun.m[2][0]) - Vec3(0, 0, -1)).length() < 1e-9);
}
