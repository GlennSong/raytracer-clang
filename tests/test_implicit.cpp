// THE IMPLICIT MASS (implicit.h, issue #105 step 6): a building as a signed distance field, sliced storey by
// storey by marching squares; the plan extruded, the wave warp, the wavy tower through the whole grammar.
#include "test_framework.h"

#include "../src/engine/procgen/city/implicit.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/room_plan.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/space_graph.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {
BuildingParams towerParams(int floors) {
    BuildingParams p;
    p.floors = floors;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 77;
    return p;
}
}  // namespace

TEST_CASE(a_plan_extruded_slices_back_to_its_own_rectangle) {
    const Poly2 plan = {{-12, -8}, {12, -8}, {12, 8}, {-12, 8}};
    const Sdf f = sdfExtrude(plan, 0, 30);
    CHECK(f(Vec3(0, 10, 0)) < 0);
    CHECK(std::fabs(f(Vec3(0, 10, 0)) + 8.0) < 1e-9);    // 8 m to the nearest long side
    CHECK(f(Vec3(14, 10, 0)) > 0 && std::fabs(f(Vec3(14, 10, 0)) - 2.0) < 1e-9);
    CHECK(f(Vec3(0, 31, 0)) > 0);                         // above the slab
    SdfBounds b{Vec3(-15, 0, -11), Vec3(15, 30, 11)};
    const Poly2 slice = sliceSdf(f, 10.0, b, 0.5, 0.3);
    for (const Vec2& q : slice) std::printf("    [slice] (%.2f, %.2f)\n", q.x, q.y);
    CHECK(slice.size() == 4);   // simplified back to the four corners
    CHECK(signedArea(slice) > 0);
    CHECK(std::fabs(area(slice) - 24.0 * 16.0) < 24.0 * 16.0 * 0.03);
    Vec2 lo, hi;
    bounds(slice, lo, hi);
    CHECK(std::fabs(lo.x + 12) < 0.3 && std::fabs(hi.x - 12) < 0.3 && std::fabs(lo.y + 8) < 0.3 && std::fabs(hi.y - 8) < 0.3);
    std::printf("    [slice] %zu corners, area %.1f of %.1f\n", slice.size(), area(slice), 24.0 * 16.0);
}

TEST_CASE(the_wave_warp_undulates_the_long_faces_and_climbs) {
    const Poly2 plan = {{-20, -8}, {20, -8}, {20, 8}, {-20, 8}};
    const Sdf f = sdfWave(sdfExtrude(plan, 0, 100), 1.5, 20.0, Vec2(0, 1), 20.0);
    SdfBounds b{Vec3(-23, 0, -11), Vec3(23, 100, 11)};
    const Poly2 s0 = sliceSdf(f, 5.0, b, 0.5, 0.25);
    const Poly2 s1 = sliceSdf(f, 15.0, b, 0.5, 0.25);
    CHECK(s0.size() > 4 && s1.size() > 4);   // no longer a rectangle
    Vec2 lo0, hi0, lo1, hi1;
    bounds(s0, lo0, hi0);
    bounds(s1, lo1, hi1);
    // across the short axis the skin reaches amplitude past the plan; along the long axis it keeps its ends
    CHECK(hi0.y > 8.0 + 1.0 && lo0.y < -8.0 - 1.0);
    CHECK(std::fabs(hi0.x - 20.0) < 0.6 && std::fabs(lo0.x + 20.0) < 0.6);
    CHECK(std::fabs(area(s0) - area(plan)) < area(plan) * 0.05);   // a wave moves area, it does not add any
    // the wave has climbed: the two heights are not the same outline
    Real moved = 0;
    for (const Vec2& q : s0) {
        Real best = 1e9;
        for (const Vec2& r : s1) best = std::min(best, (q - r).length());
        moved = std::max(moved, best);
    }
    CHECK(moved > 0.5);
    std::printf("    [wave] %zu and %zu vertices, across %.1f..%.1f, moved %.2f m between heights\n", s0.size(), s1.size(), lo0.y, hi0.y, moved);
}

TEST_CASE(a_wavy_tower_runs_through_the_whole_grammar) {
    // THE WAVE IS IN THE PLATES (Glenn, 2026-10-15: "the interior of those offices should just be a straight edge
    // versus the exterior which can be wavy"): the storeys are the plan as drawn, the glass straight on it; each
    // floor plate's edge is the field's contour at its floor line, running past the glass -- Aqua Tower.
    const Poly2 plan = {{-20, -10}, {20, -10}, {20, 10}, {-20, 10}};
    BuildingParams p = towerParams(20);
    p.envelope = BuildingParams::Envelope::Wave;
    p.baseFloors = 1;
    p.wave = 1.5;
    p.waveLen = 18.0;
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    CHECK(storeys.size() == 21);
    for (const StoreyPlan& sp : storeys) { CHECK(sp.plan.size() == 4); CHECK(sp.tier == 0); }   // straight glass, one tier
    CHECK(massStack(plan, p).size() == 1);
    const std::vector<Poly2> plates = wavePlates(plan, p);
    CHECK(plates.size() == 21);
    CHECK(plates[0].empty() && plates[1].empty());   // the base storeys: no plate past the glass
    int differing = 0;
    for (int k = 2; k <= 20; ++k) {
        const Poly2& pl = plates[static_cast<std::size_t>(k)];
        CHECK(pl.size() > 4);
        Vec2 lo, hi;
        bounds(pl, lo, hi);
        CHECK(hi.y <= 10.0 + 1.5 + 0.4 && lo.y >= -10.0 - 1.5 - 0.4);   // the wave's reach past the glass
        CHECK(hi.y > 10.0 + 0.8 || lo.y < -10.0 - 0.8);                 // and it does reach past it
        if (k > 2) {
            const Poly2& prev = plates[static_cast<std::size_t>(k - 1)];
            bool same = prev.size() == pl.size();
            for (std::size_t i = 0; same && i < pl.size(); ++i) same = (pl[i] - prev[i]).length() < 1e-9;
            if (!same) ++differing;
        }
    }
    CHECK(differing >= 15);   // the wave climbs: (almost) every plate its own outline
    CHECK(coreFor(plan, p, entranceEdgeFor(plan, p)).valid);
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    CHECK(unreachableCells(g).empty());
    const ImplicitMass im = implicitOf(plan, p);
    const RenderMesh shell = im.shell(1, Vec3(0.5, 0.5, 0.5));
    CHECK(shell.vertices.size() > 100);
    std::printf("    [wavy] %zu storeys on the plan, %d plates differing, %zu cells, %zu unreachable; polygonised shell %zu vertices\n", storeys.size(),
                differing, g.cells.size(), unreachableCells(g).size(), shell.vertices.size());
}
