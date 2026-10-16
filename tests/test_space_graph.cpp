// THE SPACE GRAPH (architectural toolkit, issue #105, foundation A): cells and portals read off today's growers, the
// storey view that must equal the storey stack, and the GATE -- every cell reachable from the street -- over every
// recipe the architect knows.
#include "test_framework.h"
#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/space_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <string>

using namespace engine;

namespace {

Poly2 rectPlan(Real w, Real d) { return {{-w / 2, -d / 2}, {w / 2, -d / 2}, {w / 2, d / 2}, {-w / 2, d / 2}}; }
Poly2 lPlan(Real w, Real d) {
    return {{-w / 2, -d / 2}, {w / 2, -d / 2}, {w / 2, 0}, {0, 0}, {0, d / 2}, {-w / 2, d / 2}};
}

BuildingParams towerParams(int floors) {
    BuildingParams p;
    p.floors = floors;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 5;
    return p;
}

int countKind(const SpaceGraph& g, CellKind k) {
    int n = 0;
    for (const Cell& c : g.cells) if (c.kind == k) ++n;
    return n;
}

bool samePlan(const Poly2& a, const Poly2& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::fabs(a[i].x - b[i].x) > 1e-9 || std::fabs(a[i].y - b[i].y) > 1e-9) return false;
    return true;
}

// Every recipe the architect knows, on a rectangle and an L, at two seeds -- the enumeration the mesh audit uses.
template <class F>
void everyRecipe(F&& f) {
    for (int i = 0; i < architectRecipeCount(); ++i) {
        const char* name = architectRecipeName(i);
        for (uint32_t seed : {3u, 17u}) {
            BuildingRecipe r;
            if (!architectRecipeByName(name, seed, 0.6, 24, 600, r)) continue;
            if (r.massing == BuildingRecipe::Massing::Park || r.massing == BuildingRecipe::Massing::Plaza) continue;
            r.params.openDoorway = true;   // (a lot's building is entered; the lineup grows them so)
            for (const Poly2& plan : {rectPlan(24, 20), lPlan(28, 24), rectPlan(40, 40)}) f(name, seed, plan, r.params);
        }
    }
}

std::string describe(const SpaceGraph& g, int i) {
    const Cell& c = g.cells[static_cast<std::size_t>(i)];
    char buf[96];
    std::snprintf(buf, sizeof buf, "%s%s%s@%d", cellKindName(c.kind), c.kind == CellKind::Room ? ":" : "",
                  c.kind == CellKind::Room ? cellUseName(c.use) : "", c.k0);
    return buf;
}

}  // namespace

TEST_CASE(space_graph_of_a_cored_tower_has_halls_rooms_shafts_and_a_roof) {
    const Poly2 plan = rectPlan(40, 40);
    const BuildingParams p = towerParams(12);
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    CHECK(g.modelled);
    const std::vector<StoreyPlan> sps = storeyPlans(plan, p);
    CHECK(g.storeys == static_cast<int>(sps.size()));
    CHECK(countKind(g, CellKind::Hall) == g.storeys);
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    CHECK(core.valid);
    CHECK(countKind(g, CellKind::Stair) == static_cast<int>(core.stairs.size()));
    CHECK(countKind(g, CellKind::Lift) == static_cast<int>(core.hoistways.size()));
    CHECK(countKind(g, CellKind::Room) > g.storeys);   // offices round every upper storey
    CHECK(coreReachesRoof(p));
    CHECK(g.roof >= 0);
    CHECK(g.entrance >= 0);
    // The street door opens into the ground hall; every storey has a stair door and a lift door; the roof has its
    // bulkhead doors from the stairs.
    int street = 0, stairDoors = 0, liftDoors = 0, roofDoors = 0, roomDoors = 0;
    for (const Portal& po : g.portals) {
        if (po.b < 0) { ++street; CHECK(po.a == g.entrance || g.cells[static_cast<std::size_t>(po.a)].kind == CellKind::Room); }
        if (po.kind == Portal::Kind::Stair && po.storey < g.storeys) ++stairDoors;
        if (po.kind == Portal::Kind::Stair && po.storey == g.storeys) { ++roofDoors; CHECK(po.b == g.roof); }
        if (po.kind == Portal::Kind::Lift) ++liftDoors;
        if (po.kind == Portal::Kind::Door && po.b >= 0) ++roomDoors;
    }
    CHECK(street >= 1);
    CHECK(stairDoors == static_cast<int>(core.stairs.size()) * g.storeys);
    CHECK(liftDoors == static_cast<int>(core.hoistways.size()) * g.storeys);
    CHECK(roofDoors == static_cast<int>(core.stairs.size()));
    CHECK(roomDoors > 0);
    // The derived view is the storey stack.
    const std::vector<StoreyPlan> view = storeyView(g, 0.0);
    CHECK(view.size() == sps.size());
    for (std::size_t k = 0; k < sps.size() && k < view.size(); ++k) {
        CHECK(samePlan(view[k].plan, sps[k].plan));
        CHECK(std::fabs(view[k].y0 - sps[k].y0) < 1e-9);
        CHECK(std::fabs(view[k].h - sps[k].h) < 1e-9);
        CHECK(view[k].tier == sps[k].tier);
    }
    // The gate on this tower.
    const std::vector<int> bad = unreachableCells(g);
    std::printf("    [graph] tower: %zu cells, %zu portals, %zu unreachable", g.cells.size(), g.portals.size(), bad.size());
    for (std::size_t i = 0; i < bad.size() && i < 6; ++i) std::printf(" %s", describe(g, bad[i]).c_str());
    std::printf("\n");
    CHECK(bad.empty());
}

TEST_CASE(walker_caps_decide_what_the_graph_reaches) {
    const Poly2 plan = rectPlan(40, 40);
    const BuildingParams p = towerParams(12);
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    CHECK(g.roof >= 0);
    int topHall = -1;
    for (std::size_t i = 0; i < g.cells.size(); ++i)
        if (g.cells[i].kind == CellKind::Hall && g.cells[i].k0 == g.storeys - 1) topHall = static_cast<int>(i);
    CHECK(topHall >= 0);
    WalkerCaps none;
    none.stairs = none.lift = none.escalator = none.ramp = false;
    CHECK(reachable(g, -1, g.entrance, none));      // the lobby, on foot
    CHECK(!reachable(g, -1, topHall, none));         // not the top: no way up
    WalkerCaps liftOnly = none;
    liftOnly.lift = true;
    CHECK(reachable(g, -1, topHall, liftOnly));      // the lift serves every storey...
    CHECK(!reachable(g, -1, g.roof, liftOnly));      // ...but the roof is the stairs' alone
    WalkerCaps stairsOnly = none;
    stairsOnly.stairs = true;
    CHECK(reachable(g, -1, g.roof, stairsOnly));
    CHECK(reachable(g, -1, g.roof, WalkerCaps{}));
}

TEST_CASE(storey_view_equals_storey_plans_every_recipe) {
    int buildings = 0, mismatches = 0;
    everyRecipe([&](const char* name, uint32_t seed, const Poly2& plan, const BuildingParams& params) {
        const SpaceGraph g = spaceGraphOf(plan, params, 0.0);
        if (!g.modelled) return;
        ++buildings;
        Poly2 ccw = plan;
        ensureCCW(ccw);
        const std::vector<StoreyPlan> sps = storeyPlans(ccw, params);
        const std::vector<StoreyPlan> view = storeyView(g, 0.0);
        bool ok = view.size() == sps.size();
        for (std::size_t k = 0; ok && k < sps.size(); ++k)
            ok = samePlan(view[k].plan, sps[k].plan) && std::fabs(view[k].y0 - sps[k].y0) < 1e-9 &&
                 std::fabs(view[k].h - sps[k].h) < 1e-9 && view[k].tier == sps[k].tier;
        if (!ok) { ++mismatches; std::printf("    [view] MISMATCH %s seed %u (%zu-gon)\n", name, seed, plan.size()); }
    });
    std::printf("    [view] %d buildings, %d mismatches\n", buildings, mismatches);
    CHECK(buildings > 0);
    CHECK(mismatches == 0);
}

// THE GATE over every recipe (the brief on #105, step 1b): every cell of every building reachable from the street.
// THE FIX LIST (step 1c) is the recipes it fails today, each with its one cause, written here so the suite stays
// green and honest: a recipe not on the list must pass; a recipe on it must still fail, and leaves the list the day
// its cause is fixed. What the gate found on day one (2026-10-15), all fixed the same day: apartment end units whose
// hall faced a bedroom across a solid divider (layoutApartment); coreless setback towers whose straight stair ended
// at the first shrunken tier (stairRuns: the stair moves inboard, the transfer storey an open floor); a ring office
// on an L's narrow arm that spanned the arm and opened its door on the exterior wall (ringPlan measures into the
// plan). The list is empty; it stays here for the next thing the gate finds.
namespace {
const std::map<std::string, const char*> kFixList = {};
}  // namespace

TEST_CASE(graph_gate_every_recipe) {
    struct Tally { int buildings = 0, failing = 0, cells = 0, portals = 0; std::map<std::string, int> why; };
    std::map<std::string, Tally> byRecipe;
    everyRecipe([&](const char* name, uint32_t seed, const Poly2& plan, const BuildingParams& params) {
        const SpaceGraph g = spaceGraphOf(plan, params, 0.0);
        if (!g.modelled) return;
        Tally& t = byRecipe[name];
        ++t.buildings;
        t.cells += static_cast<int>(g.cells.size());
        t.portals += static_cast<int>(g.portals.size());
        const std::vector<int> bad = unreachableCells(g);
        if (bad.empty()) return;
        ++t.failing;
        for (int i : bad) ++t.why[describe(g, i)];
        (void)seed;
    });
    int failingRecipes = 0, unlisted = 0;
    for (const auto& [name, t] : byRecipe) {
        if (t.failing == 0) {
            if (kFixList.count(name)) { std::printf("    [gate] %s passes now: take it off the fix list\n", name.c_str()); ++unlisted; }
            continue;
        }
        ++failingRecipes;
        const auto fix = kFixList.find(name);
        if (fix == kFixList.end()) ++unlisted;
        std::printf("    [gate] %s %-22s %d/%d buildings fail:", fix == kFixList.end() ? "NEW" : "fix list", name.c_str(), t.failing, t.buildings);
        int shown = 0;
        for (const auto& [why, n] : t.why) { if (shown++ >= 5) { std::printf(" ..."); break; } std::printf(" %s x%d", why.c_str(), n); }
        std::printf("\n");
    }
    std::set<std::string> causes;
    for (const auto& [name, t] : byRecipe) if (t.failing && kFixList.count(name)) causes.insert(kFixList.at(name));
    for (const std::string& c : causes) std::printf("    [gate] FIX LIST cause: %s\n", c.c_str());
    int passing = 0, total = 0;
    for (const auto& [name, t] : byRecipe) { total += t.buildings; passing += t.buildings - t.failing; }
    std::printf("    [gate] %d recipes, %d buildings, %d pass, %d recipes on the fix list\n",
                static_cast<int>(byRecipe.size()), total, passing, failingRecipes);
    CHECK(total > 0);
    CHECK(unlisted == 0);   // nothing fails that the fix list does not name, and the list names nothing that passes
}

// THE STAIR MOVES INBOARD (issue #105, step 1c): a coreless setback tower's straight stair ran out at the first
// shrunken tier, and every storey above had no way up -- ten recipes on the gate's day-one fix list. stairRuns lays
// a new run on the shrunken tier, its first flight leaving the storey the last run arrived at, and the graph climbs
// to the top.
TEST_CASE(straight_stair_climbs_through_a_setback) {
    const Poly2 plan = rectPlan(24, 20);
    BuildingParams p = towerParams(16);
    p.core = 1;   // never: the straight stair is the way up
    p.setbackFloors = 5;
    p.setbackEvery = 2.0;
    Poly2 ccw = plan;
    ensureCCW(ccw);
    const std::vector<StoreyPlan> sps = storeyPlans(ccw, p);
    CHECK(sps.back().tier >= 2);
    const std::vector<StairRun> runs = stairRuns(ccw, p, entranceEdgeFor(ccw, p), sps);
    CHECK(runs.size() >= 2);
    for (std::size_t i = 0; i < runs.size(); ++i) {
        CHECK(runs[i].k1 > runs[i].k0);
        if (i > 0) CHECK(runs[i].k0 == runs[i - 1].k1);   // the next run leaves where the last arrived
        if (i > 0) CHECK(runs[i].il.edge < runs[i].sides);
    }
    CHECK(runs.front().k0 == 0);
    CHECK(runs.back().k1 == static_cast<int>(sps.size()) - 1);   // ...to the top
    for (int k = 0; k + 1 < static_cast<int>(sps.size()); ++k) CHECK(runLeaving(runs, k) != nullptr);
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    int stairPortals = 0;
    for (const Portal& po : g.portals) if (po.kind == Portal::Kind::Stair) ++stairPortals;
    CHECK(stairPortals == static_cast<int>(sps.size()) - 1);
    const std::vector<int> bad = unreachableCells(g);
    std::printf("    [runs] %zu runs over %zu storeys, %zu unreachable\n", runs.size(), sps.size(), bad.size());
    for (std::size_t i = 0; i < runs.size(); ++i)
        std::printf("    [runs]   run %zu storeys %d..%d edge %zu of %zu foot (%.1f, %.1f)\n", i, runs[i].k0, runs[i].k1,
                    runs[i].il.edge, runs[i].sides, runs[i].il.stairFoot.x, runs[i].il.stairFoot.y);
    CHECK(bad.empty());
}

// THE TRANSFER FLOOR IS A LOUNGE (issue #105; the open storey between two stair runs was bare): sofas along the
// glass on every edge but the stairs', every strip clear of both wells, no walls; the furniture lands and the gate
// stays green.
TEST_CASE(transfer_floor_is_a_lounge) {
    const Poly2 plan = rectPlan(24, 20);
    BuildingParams p = towerParams(16);
    p.core = 1;
    p.setbackFloors = 5;
    p.setbackEvery = 2.0;
    Poly2 ccw = plan;
    ensureCCW(ccw);
    const std::vector<StoreyPlan> sps = storeyPlans(ccw, p);
    const std::vector<StairRun> runs = stairRuns(ccw, p, entranceEdgeFor(ccw, p), sps);
    CHECK(runs.size() >= 2);
    if (runs.size() < 2) return;
    const int kT = runs[0].k1;
    const CorePlan core = coreFor(ccw, p, entranceEdgeFor(ccw, p));
    const StoreyRooms sr = storeyRooms(ccw, p, 0.0, sps, kT, core, runs, entranceEdgeFor(ccw, p));
    CHECK(!sr.rp.rooms.empty());
    CHECK(sr.rp.walls.empty());
    for (const Room& r : sr.rp.rooms) {
        CHECK(r.kind == RoomKind::Lounge);
        for (const StairRun& run : {runs[0], runs[1]}) {
            for (const Vec2& q : r.rect) CHECK(!pointInPolygon(run.il.well, q));
            for (const Vec2& q : run.il.well) CHECK(!pointInPolygon(r.rect, q));
        }
    }
    const BuildingMesh in = growInterior(ccw, p, 0.0, nullptr, kT, kT + 1);
    std::printf("    [lounge] transfer storey %d: %zu lounge strips, %zu pieces of furniture\n", kT, sr.rp.rooms.size(), in.furniture.size());
    CHECK(in.furniture.size() > 4);
    CHECK(unreachableCells(spaceGraphOf(plan, p, 0.0)).empty());
}

TEST_CASE(blueprint_svg_draws_every_storey_and_the_roof) {
    const Poly2 plan = rectPlan(40, 40);
    const BuildingParams p = towerParams(12);
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    const std::string svg = blueprintSvg(g, "tower", unreachableCells(g));
    CHECK(svg.find("<svg") == 0);
    CHECK(svg.find(">storey 0<") != std::string::npos);
    CHECK(svg.find(">storey " + std::to_string(g.storeys - 1) + "<") != std::string::npos);
    CHECK(svg.find(">roof<") != std::string::npos);
    CHECK(svg.find("</svg>") != std::string::npos);
}

// (debug dump: RT_GRAPH_DUMP=recipe[,recipe...] prints one building's cells and portals)
TEST_CASE(graph_dump_debug) {
    const char* want = std::getenv("RT_GRAPH_DUMP");
    if (!want) return;
    const std::string wants = want;
    everyRecipe([&](const char* name, uint32_t seed, const Poly2& plan, const BuildingParams& params) {
        const char* wp = std::getenv("RT_GRAPH_DUMP_PLAN");
        const std::string wantPlan = wp ? wp : "rect24";
        const std::string planName = plan.size() == 6 ? "L" : plan[1].x > 15 ? "rect40" : "rect24";
        if ((std::string(",") + wants + ",").find(std::string(",") + name + ",") == std::string::npos || seed != (std::getenv("RT_GRAPH_DUMP_SEED") ? static_cast<uint32_t>(std::atoi(std::getenv("RT_GRAPH_DUMP_SEED"))) : 3u) || planName != wantPlan) return;
        const SpaceGraph g = spaceGraphOf(plan, params, 0.0);
        const std::vector<bool> seen = reachableFrom(g, -1, WalkerCaps{});
        std::printf("=== %s seed %u: %d storeys, %zu cells, %zu portals, core %s\n", name, seed, g.storeys, g.cells.size(), g.portals.size(),
                    coreFor(plan, params, entranceEdgeFor(plan, params)).valid ? "yes" : "no");
        const std::vector<StoreyPlan> sps = storeyPlans(plan, params);
        const std::vector<StairRun> runs = stairRuns(plan, params, entranceEdgeFor(plan, params), sps);
        for (const StairRun& r : runs)
            std::printf("  stair run %d..%d foot (%.1f,%.1f) dir (%.1f,%.1f) run %.1f edge %zu/%zu\n", r.k0, r.k1, r.il.stairFoot.x, r.il.stairFoot.y, r.il.stairDir.x, r.il.stairDir.y, r.il.run, r.il.edge, r.sides);
        const std::vector<int> bad = unreachableCells(g);
        std::printf("  unreachable (%zu):", bad.size());
        for (std::size_t i = 0; i < bad.size() && i < 14; ++i) std::printf(" %d=%s", bad[i], describe(g, bad[i]).c_str());
        std::printf("\n");
        int lowK = 99, highK = -1;
        for (int i : bad) { lowK = std::min(lowK, g.cells[static_cast<std::size_t>(i)].k0); highK = std::max(highK, g.cells[static_cast<std::size_t>(i)].k0); }
        const int showK = bad.empty() ? 0 : lowK;
        for (std::size_t i = 0; i < g.cells.size(); ++i) {
            const Cell& c = g.cells[i];
            if (c.k0 != showK && !(c.kind == CellKind::Stair || c.kind == CellKind::Lift)) continue;
            Vec2 lo, hi; bounds(c.plan, lo, hi);
            std::printf("  cell %2zu %-14s k%d-%d [%.1f %.1f]-[%.1f %.1f] free %.1f %s\n", i, describe(g, static_cast<int>(i)).c_str(), c.k0, c.k1, lo.x, lo.y, hi.x, hi.y, c.freeArea, seen[i] ? "" : "UNREACHABLE");
        }
        for (const Portal& p : g.portals) {
            if (p.storey != showK && !(p.storey == showK - 1 && p.kind != Portal::Kind::Door)) continue;
            std::printf("  portal %-9s s%d  %d <-> %d  at (%.1f,%.1f)\n", portalKindName(p.kind), p.storey, p.a, p.b, (p.p0.x + p.p1.x) * 0.5, (p.p0.y + p.p1.y) * 0.5);
        }
        // the storey's outline and every unreachable cell's own polygon (a wavy skin, a room laid past it)
        for (std::size_t i = 0; i < g.cells.size(); ++i) {
            const Cell& c = g.cells[i];
            if (c.k0 != showK || (c.kind != CellKind::Hall && seen[i])) continue;
            std::printf("  poly %zu %s:", i, describe(g, static_cast<int>(i)).c_str());
            for (const Vec2& q : c.plan) std::printf(" (%.1f,%.1f)", q.x, q.y);
            std::printf("\n");
        }
        // the walls with doors on storey 1, and what the planner put either side
        const CorePlan core = coreFor(plan, params, entranceEdgeFor(plan, params));
        for (int k = showK; k < std::min(showK + 1, g.storeys); ++k) {
            const StoreyRooms sr = storeyRooms(plan, params, 0.0, sps, k, core, runs, entranceEdgeFor(plan, params));
            std::printf("  storey %d: topology %d rooms %zu walls %zu (doors %zu) openings %zu\n", k, static_cast<int>(sr.rp.topology), sr.rp.rooms.size(), sr.rp.walls.size(),
                        static_cast<std::size_t>(std::count_if(sr.rp.walls.begin(), sr.rp.walls.end(), [](const RoomWall& w) { return w.doorAt >= 0; })),
                        sr.rp.openings.size());
            for (const Room& r : sr.rp.rooms) { Vec2 lo, hi; bounds(r.rect, lo, hi); std::printf("    room %-10s [%.1f %.1f]-[%.1f %.1f]\n", cellUseName(r.kind), lo.x, lo.y, hi.x, hi.y); }
            for (const RoomWall& w : sr.rp.walls) if (w.doorAt >= 0) {
                const Vec2 p = w.a + (w.b - w.a) * w.doorAt;
                Vec2 d = w.b - w.a; d = d * (1.0 / d.length()); const Vec2 n(-d.y, d.x);
                auto side = [&](const Vec2& q) {
                    for (std::size_t ri = 0; ri < sr.rp.rooms.size(); ++ri) if (pointInPolygon(sr.rp.rooms[ri].rect, q)) return std::string("room") + std::to_string(ri);
                    return pointInPolygon(sps[static_cast<std::size_t>(k)].plan, q) ? std::string("hall") : std::string("OUTSIDE");
                };
                std::printf("    door at (%.1f,%.1f) on wall (%.1f,%.1f)-(%.1f,%.1f): sides %s / %s\n", p.x, p.y, w.a.x, w.a.y, w.b.x, w.b.y,
                            side(p + n * 0.45).c_str(), side(p - n * 0.45).c_str());
            }
            for (const RoomWall& w : sr.rp.walls) if (w.doorAt < 0) std::printf("    wall (%.1f,%.1f)-(%.1f,%.1f)%s\n", w.a.x, w.a.y, w.b.x, w.b.y, w.glass ? " glass" : "");
            { Vec2 lo, hi; bounds(sps[static_cast<std::size_t>(k)].plan, lo, hi); std::printf("    plan %zu-gon [%.1f %.1f]-[%.1f %.1f]:", sps[static_cast<std::size_t>(k)].plan.size(), lo.x, lo.y, hi.x, hi.y); for (const Vec2& v : sps[static_cast<std::size_t>(k)].plan) std::printf(" (%.1f,%.1f)", v.x, v.y); std::printf("\n"); }
        }
    });
}
