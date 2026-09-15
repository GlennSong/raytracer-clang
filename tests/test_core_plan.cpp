// The core (skyscrapers v2 M5, core_plan.h): the elevator bank and the two
// dog-leg stairwells a tall building climbs by — pure geometry from the plan,
// the mass stack and the params, shared by the exterior grow, the streamed
// interior and the runtime.
#include "test_framework.h"

#include "../src/engine/procgen/city/core_plan.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {
BuildingParams towerParams(int floors, bool curtain) {
    BuildingParams p;
    p.floors = floors;
    p.curtainWall = curtain;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 77;
    return p;
}
bool inside(const Poly2& poly, const Poly2& q) {
    for (const Vec2& v : q)
        if (!pointInPolygon(poly, v)) return false;
    return true;
}
}  // namespace

TEST_CASE(core_plan_seats_a_bank_and_two_stairs_facing_the_entrance) {
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const BuildingParams p = towerParams(30, true);
    const std::size_t e = entranceEdgeFor(plan, p);
    const CorePlan core = coreFor(plan, p, e);
    CHECK(core.valid);
    CHECK(core.hoistways.size() == 3);   // 21-40 floors
    CHECK(core.stairs.size() == 2);
    CHECK(core.hasService);
    // Every shaft sits inside the plan and inside the core's own outline
    // (its corners lie ON the outline, so the centroid is the inside test).
    const Poly2 outline = core.rect();
    for (const CoreShaft& h : core.hoistways) { CHECK(inside(plan, h.rect())); CHECK(pointInPolygon(outline, centroid(h.rect()))); }
    for (const CoreStair& s : core.stairs) { CHECK(inside(plan, s.shaft.rect())); CHECK(pointInPolygon(outline, centroid(s.shaft.rect()))); }
    // The doors face the entrance: the door normal points toward the
    // entrance edge's midpoint from the core's centre.
    Poly2 ccw = plan;
    ensureCCW(ccw);
    const Vec2 mid = (ccw[e] + ccw[(e + 1) % ccw.size()]) * 0.5;
    const Vec2 c = centroid(outline);
    for (const CoreShaft& h : core.hoistways) CHECK(dot(h.doorNormal(), normalize(mid - c)) > 0.5);
    for (const CoreStair& s : core.stairs) CHECK(dot(s.shaft.doorNormal(), normalize(mid - c)) > 0.5);
    // Shafts never overlap each other.
    std::vector<Poly2> holes = coreSlabHoles(core);
    CHECK(holes.size() == 5);
    for (std::size_t i = 0; i < holes.size(); ++i)
        for (std::size_t j = i + 1; j < holes.size(); ++j)
            CHECK(!pointInPolygon(holes[j], centroid(holes[i])));
}

TEST_CASE(core_plan_scales_the_bank_with_height_and_refuses_a_small_plan) {
    CHECK(hoistwaysFor(6) == 1);
    CHECK(hoistwaysFor(20) == 2);
    CHECK(hoistwaysFor(21) == 3);
    CHECK(hoistwaysFor(41) == 4);
    // A 4.5 m ground storey: 12 risers of 0.1875 per half flight, 3.12 m run.
    CHECK(halfFlightRisers(4.5) == 12);
    CHECK(std::fabs(halfFlightRun(4.5) - 12 * 0.26) < 1e-9);
    CHECK(halfFlightRisers(3.2) == 8);
    // Too small for a core with its corridor: the building keeps the old stair.
    const Poly2 small = {{0, 0}, {8, 0}, {8, 10}, {0, 10}};
    const BuildingParams p = towerParams(6, false);
    CHECK(!coreFor(small, p, entranceEdgeFor(small, p)).valid);
    // The policy: under four floors no core; "never" and "always" override.
    BuildingParams low = towerParams(3, false);
    CHECK(!wantsCore(low));
    low.core = 2;
    CHECK(wantsCore(low));
    BuildingParams tall = towerParams(30, true);
    CHECK(wantsCore(tall));
    tall.core = 1;
    CHECK(!wantsCore(tall));
    CHECK(!coreFor({{0, 0}, {40, 0}, {40, 40}, {0, 40}}, tall, 0).valid);
}

TEST_CASE(core_plan_fits_inside_every_tier_of_a_setback_tower) {
    const Poly2 plan = {{0, 0}, {36, 0}, {36, 44}, {0, 44}};
    BuildingParams p = towerParams(40, false);
    p.envelope = BuildingParams::Envelope::StreetWallSetback;
    p.baseFloors = 5;
    p.setback1 = 5;
    p.stepFloors = 10;
    p.stepDepth = 3;
    p.towerFrac = 0.35;
    p.towerFloor = 20;
    const std::vector<MassTier> tiers = massStack(plan, p);
    CHECK(tiers.size() >= 2);
    const CorePlan core = corePlan(plan, tiers, p, entranceEdgeFor(plan, p));
    CHECK(core.valid);
    for (const MassTier& t : tiers) CHECK(inside(t.plan, core.rect()));
}

TEST_CASE(core_storey_is_enclosed_climbable_and_lands_on_the_next_floor) {
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const BuildingParams p = towerParams(12, true);
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    CHECK(core.valid);
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    CoreMeshes cm;
    RenderMesh col;
    const Real baseY = 10.0;
    emitCoreStorey(cm, &col, core, storeys[3], baseY, p, true, true);
    CHECK(!cm.drywall.vertices.empty());
    CHECK(!cm.floor.vertices.empty());
    CHECK(!cm.stair.vertices.empty());
    CHECK(!col.indices.empty());
    // The stair's highest tread is the next storey's slab top; nothing of it
    // rises above that, nothing sits below this storey's slab top.
    const Real y0 = baseY + storeys[3].y0, h = storeys[3].h;
    Real hi = -1e9, lo = 1e9;
    for (const Vertex& v : cm.stair.vertices) { hi = std::max(hi, (Real)v.position.y); lo = std::min(lo, (Real)v.position.y); }
    CHECK(std::fabs(hi - (y0 + h + 0.05)) < 1e-6);
    CHECK(lo >= y0 + 0.05 - 1e-6);
    // Walls span the storey: the drywall reaches the ceiling and the floor.
    Real whi = -1e9, wlo = 1e9;
    for (const Vertex& v : cm.drywall.vertices) { whi = std::max(whi, (Real)v.position.y); wlo = std::min(wlo, (Real)v.position.y); }
    CHECK(std::fabs(whi - (y0 + h)) < 1e-6);
    CHECK(wlo <= y0 + 1e-6);
    // Every riser is a code riser: the emitted tread tops (up-facing quads in
    // the stair mesh) climb in steps of at most 0.2 m from the slab to the
    // next slab, with no level skipped.
    {
        std::vector<Real> tops;
        for (const Vertex& v : cm.stair.vertices)
            if (v.normal.y > 0.99) tops.push_back(v.position.y);
        std::sort(tops.begin(), tops.end());
        tops.erase(std::unique(tops.begin(), tops.end(), [](Real a, Real b) { return std::fabs(a - b) < 1e-6; }), tops.end());
        CHECK(tops.size() >= 6);
        CHECK(std::fabs(tops.front() - (y0 + 0.05 + (h * 0.5) / halfFlightRisers(h))) < 1e-6);
        for (std::size_t i = 1; i < tops.size(); ++i) CHECK(tops[i] - tops[i - 1] <= 0.2 + 1e-9);
        CHECK(std::fabs(tops.back() - (y0 + h + 0.05)) < 1e-6);
    }
    // The top storey grows no flights but guards the well: a floor-to-ceiling
    // wall at the landing's inner edge across flight A's half and the spine.
    CoreMeshes top;
    emitCoreStorey(top, nullptr, core, storeys.back(), baseY, p, false, true);
    CHECK(top.stair.vertices.empty());
    {
        const CoreStair& st = core.stairs[0];
        const Real yT0 = baseY + storeys.back().y0, hT = storeys.back().h;
        bool guard = false;
        for (std::size_t i = 0; i + 3 < top.drywall.vertices.size(); i += 4) {
            bool onLine = true, spans = true;
            Real uMin = 1e9, uMax = -1e9;
            for (std::size_t j = 0; j < 4; ++j) {
                const Vertex& v = top.drywall.vertices[i + j];
                const Vec2 q = st.shaft.frame.toFrame(Vec2(v.position.x, v.position.z));
                if (std::fabs(q.y - st.landing) > 1e-6) onLine = false;
                uMin = std::min(uMin, q.x);
                uMax = std::max(uMax, q.x);
                if (v.position.y < yT0 - 1e-6 || v.position.y > yT0 + hT + 1e-6) spans = false;
            }
            if (onLine && spans && uMin < 1e-6 && std::fabs(uMax - (st.flightWidth + st.spine)) < 1e-6) guard = true;
        }
        CHECK(guard);
    }
    // The ground storey grows flights but no floor landing: nothing of the
    // floor part lies at the ground slab's top (the half landing is higher).
    CoreMeshes ground;
    emitCoreStorey(ground, nullptr, core, storeys[0], baseY, p, true, false);
    CHECK(!ground.stair.vertices.empty());
    for (const Vertex& v : ground.floor.vertices) CHECK(v.position.y > baseY + 0.05 + 0.5);
}

TEST_CASE(core_plan_refuses_a_notch_through_the_bank) {
    // An L/notched plan whose notch cuts the corridor ring's edge while the
    // ring's four corners stay inside — the corner-only test would seat the
    // bank through the notch.
    const Poly2 notched = {{0, 0}, {40, 0}, {40, 40}, {25, 40}, {25, 22}, {15, 22}, {15, 40}, {0, 40}};
    const Poly2 square = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const BuildingParams p = towerParams(30, true);
    CHECK(coreFor(square, p, entranceEdgeFor(square, p)).valid);
    CHECK(!coreFor(notched, p, entranceEdgeFor(notched, p)).valid);
    // The policy's threshold is pinned: four floors open, three do not.
    CHECK(wantsCore(towerParams(4, false)));
    CHECK(!wantsCore(towerParams(3, false)));
}

TEST_CASE(grow_interior_with_a_core_punches_the_shafts_and_streams_a_window) {
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const BuildingParams p = towerParams(12, true);
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    CHECK(core.valid);
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    const Real baseY = 0;
    RenderMesh col;
    const BuildingMesh all = growInterior(plan, p, baseY, &col);
    // A point in a hoistway at storey 4's slab height has no floor
    // triangle over it; a point in the corridor beside the core does.
    const Vec2 inShaft = core.hoistways[0].frame.toWorld({1.2, 1.3});
    const Vec2 corridor = core.frame.toWorld({-1.0, core.depth * 0.5});
    const Real ySlab = baseY + storeys[4].y0 + 0.05;
    auto floorOver = [&](const BuildingMesh& bm, const Vec2& q) {
        for (const RenderMesh& part : bm.parts) {
            if (part.materialIndex == static_cast<int>(PartId::Interior)) continue;
            for (std::size_t i = 0; i + 2 < part.indices.size(); i += 3) {
                const Vertex& a = part.vertices[part.indices[i]];
                const Vertex& b = part.vertices[part.indices[i + 1]];
                const Vertex& c = part.vertices[part.indices[i + 2]];
                if (std::fabs(a.position.y - ySlab) > 1e-4 || std::fabs(b.position.y - ySlab) > 1e-4 ||
                    std::fabs(c.position.y - ySlab) > 1e-4)
                    continue;
                const Poly2 tri = {{a.position.x, a.position.z}, {b.position.x, b.position.z}, {c.position.x, c.position.z}};
                if (pointInPolygon(tri, q)) return true;
            }
        }
        return false;
    };
    CHECK(!floorOver(all, inShaft));
    CHECK(floorOver(all, corridor));
    // The window [3, 6) holds storeys 3-5 only: nothing below storey 3's
    // floor, nothing above storey 5's ceiling, and it is a strict subset.
    RenderMesh colW;
    const BuildingMesh win = growInterior(plan, p, baseY, &colW, 3, 6);
    Real lo = 1e9, hi = -1e9;
    std::size_t nWin = 0, nAll = 0;
    for (const RenderMesh& part : win.parts) {
        nWin += part.vertices.size();
        for (const Vertex& v : part.vertices) { lo = std::min(lo, (Real)v.position.y); hi = std::max(hi, (Real)v.position.y); }
    }
    for (const RenderMesh& part : all.parts) nAll += part.vertices.size();
    CHECK(nWin > 0);
    CHECK(nWin < nAll);
    CHECK(lo >= baseY + storeys[3].y0 - 0.3);
    CHECK(hi <= baseY + storeys[5].y0 + storeys[5].h + 0.06);
    CHECK(floorOver(win, corridor));
    CHECK(!colW.indices.empty());
}

TEST_CASE(core_window_grow_cost_is_bounded) {
    // The performance census (Glenn: "we do have to figure out how to keep
    // the city performant"): one streamed window of a 40-storey curtain-wall
    // tower — five storeys with their core — must stay small. Printed so the
    // numbers travel with the run.
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const BuildingParams p = towerParams(40, true);
    CHECK(coreFor(plan, p, entranceEdgeFor(plan, p)).valid);
    RenderMesh col;
    const auto t0 = std::chrono::steady_clock::now();
    const BuildingMesh win = growInterior(plan, p, 0.0, &col, 18, 23);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::size_t tris = 0, bytes = 0;
    for (const RenderMesh& part : win.parts) {
        tris += part.indices.size() / 3;
        bytes += part.vertices.size() * sizeof(Vertex) + part.indices.size() * sizeof(uint32_t);
    }
    std::printf("    [core-census] window [18, 23) of 41 storeys: %zu tris, %zu KB, collider %zu tris, grow %.2f ms\n",
                tris, bytes / 1024, col.indices.size() / 3, ms);
    CHECK(tris > 0);
    CHECK(tris < 4000);                   // ~2x the measured 1870
    CHECK(col.indices.size() / 3 < 3500);   // ~2x the measured 1700
    (void)ms;   // printed, not asserted: wall-clock on a shared desktop is not a gate
}

TEST_CASE(lobby_gets_a_desk_and_call_buttons) {
    // The lobby's storey carries a reception desk with a collider between the
    // entrance and the bank, and every hoistway door a call button plate; an
    // upper storey has the plates but no desk.
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const BuildingParams p = towerParams(20, true);
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    CHECK(core.valid);
    RenderMesh col0, col5;
    const BuildingMesh lobby = growInterior(plan, p, 0.0, &col0, 0, 1);
    const BuildingMesh upper = growInterior(plan, p, 0.0, &col5, 5, 6);
    auto countAt = [](const BuildingMesh& bm, Real y0, Real y1) {
        std::size_t n = 0;
        for (const RenderMesh& part : bm.parts)
            if (part.materialIndex == static_cast<int>(PartId::Interior))
                for (const Vertex& v : part.vertices)
                    if (v.position.y > y0 && v.position.y < y1) ++n;
        return n;
    };
    // The call button plate at 1.05-1.22 over the storey base — on both.
    CHECK(countAt(lobby, 1.04, 1.06) > 0);
    CHECK(countAt(upper, 4.5 + 4 * 3.2 + 1.04, 4.5 + 4 * 3.2 + 1.06) > 0);
    // The desk has a COLLIDER (the plates do not): triangles at its counter
    // top (1.12 over the 0.07 overlay) in the lobby, none at that height on
    // an upper storey.
    auto trisAt = [](const RenderMesh& col, Real y0, Real y1) {
        std::size_t n = 0;
        for (std::size_t i = 0; i + 2 < col.indices.size(); i += 3) {
            const Real y = col.vertices[col.indices[i]].position.y;
            if (y > y0 && y < y1) ++n;
        }
        return n;
    };
    CHECK(trisAt(col0, 1.15, 1.25) >= 2);
    CHECK(trisAt(col5, 4.5 + 4 * 3.2 + 1.15, 4.5 + 4 * 3.2 + 1.25) == 0);
}
