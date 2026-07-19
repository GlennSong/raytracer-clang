// Roads-v2.1 R4 ground-truth gates (drive feedback): the terrain the city
// stands on must be WALKABLE truth —
//   PIT PROBE: roads on hills used to corral terrain into bowls a player
//     falls into and cannot leave ("the road makes it into a hole"). With
//     block grading, every block interior blends to the plane of its road
//     boundary: no enclosed basin deeper than a curb-step remains.
//   RISER PROBE: the sidewalk's OUTER face used to stand as a wall where
//     the ungraded interior fell away ("player can't walk up onto a
//     sidewalk") — the same grading pins the interior to the street, so the
//     outer riser stays a step, not a wall.
#include "test_framework.h"

#include "../src/engine/procgen/city/block_grade.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/structure_set.h"
#include "../src/engine/procgen/terrain.h"
#include <cmath>
#include <cstdio>
#include <nlohmann/json.hpp>

using namespace engine;

namespace {

// Rolling hills: enough relief that an ungraded block interior forms a
// real bowl between filled road edges.
double hills(double x, double z) {
    return 7.0 * std::sin(x * 0.045) + 5.5 * std::cos(z * 0.06) +
           2.5 * std::sin((x + z) * 0.03);
}

// TRAPPED, honestly: a grid cell the player cannot REACH from any road by
// walking (each 2 m step limited to a comfortable climb). A pit is exactly
// the set of low cells no walkable path serves — local ray tests miss
// block-sized bowls whose walls are far from the bottom.
template <typename F>
int trappedCells(F&& ground, double lo, double hi) {
    const double cell = 2.0;
    const int N = static_cast<int>((hi - lo) / cell) + 1;
    std::vector<double> h(N * N);
    for (int iz = 0; iz < N; ++iz)
        for (int ix = 0; ix < N; ++ix)
            h[iz * N + ix] = ground(lo + ix * cell, lo + iz * cell);
    std::vector<char> reach(N * N, 0);
    std::vector<int> stack;
    for (int i = 0; i < N; ++i) {   // the boundary ring rides the roads
        for (int j : { i, (N - 1) * N + i, i * N, i * N + (N - 1) })
            if (!reach[j]) { reach[j] = 1; stack.push_back(j); }
    }
    const double kStep = 1.0;   // per-2 m climb a player comfortably walks
    while (!stack.empty()) {
        const int c = stack.back();
        stack.pop_back();
        const int ix = c % N, iz = c / N;
        const int nb[4] = { c - 1, c + 1, c - N, c + N };
        const bool ok[4] = { ix > 0, ix < N - 1, iz > 0, iz < N - 1 };
        for (int k = 0; k < 4; ++k) {
            if (!ok[k] || reach[nb[k]]) continue;
            // SAFE means the road is reachable CLIMBING OUT: expand cell c
            // to neighbour nb only if nb could step UP into c — descending
            // into a bowl is always possible and proves nothing.
            if (h[c] - h[nb[k]] > kStep) continue;
            reach[nb[k]] = 1;
            stack.push_back(nb[k]);
        }
    }
    int trapped = 0;
    for (int i = 0; i < N * N; ++i)
        if (!reach[i]) {
            ++trapped;
            if (std::getenv("RT_PIT_DEBUG"))
                std::printf("[pit]   trapped cell (%.0f, %.0f) h=%.2f\n",
                            lo + (i % N) * cell, lo + (i / N) * cell, h[i]);
        }
    return trapped;
}

}  // namespace

TEST_CASE(block_grading_leaves_no_pits_between_roads) {
    // A closed ring of streets around a naturally-low interior — the exact
    // corral geometry from the drive. Conform carves the roads; the block
    // grade must lift/blend the interior to the boundary plane.
    RoadNet net;
    net.width = 8.0;
    net.sidewalk = 2.0;
    net.autoRoundabout = false;
    net.heightAt = hills;
    net.nodes = { Vec2(-60, -60), Vec2(60, -60), Vec2(60, 60), Vec2(-60, 60) };
    net.edges = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 } };

    std::vector<TerrainFlatten> flatten = roadNetConformRegions(net);
    // The block: the ring interior, inset off the road (as the lot plan's
    // blocks are).
    // The grade polygon OVERLAPS the road conform band on purpose: the
    // road's higher priority wins inside its own footprint, and everywhere
    // else the block plane owns the ground — no raw-terrain notch can
    // survive between the two systems (the seam gap was the pit).
    Poly2 block{ Vec2(-58, -58), Vec2(58, -58), Vec2(58, 58), Vec2(-58, 58) };
    auto roadCarved = [&](double x, double z) {
        return applyFlatten(flatten, x, z, hills(x, z));
    };
    // CONTROL: without grading, the road ring stands over an interior that
    // falls metres away — the sidewalk-edge WALL (and the raw notch between
    // road band and interior) is exactly the 'can't get back up' pit
    // mechanism from the drive. If this stops failing, the fixture lost
    // its teeth.
    {
        double wallUngraded = 0;
        for (double t = -50; t <= 50; t += 5.0) {
            const double deckY = roadCarved(t, -60.0);
            const double insideY = roadCarved(t, -60.0 + 7.0);
            wallUngraded = std::max(wallUngraded, deckY + 0.15 - insideY);
        }
        std::printf("[pit] control wallUngraded=%.2f\n", wallUngraded);
        CHECK(wallUngraded > 0.55);
    }
    std::vector<TerrainFlatten> graded = gradeBlocks({ block }, roadCarved);
    CHECK(!graded.empty());
    std::vector<TerrainFlatten> all = flatten;
    all.insert(all.end(), graded.begin(), graded.end());
    auto finalGround = [&](double x, double z) {
        return applyFlatten(all, x, z, hills(x, z));
    };

    if (std::getenv("RT_PIT_DEBUG"))
        for (double z = 50; z <= 60; z += 2)
            std::printf("[pit]   col x=-38 z=%.0f h=%.2f\n", z,
                        finalGround(-38, z));
    // PIT PROBE: with grading, every cell is reachable on foot from a road.
    const int basins = trappedCells(finalGround, -58.0, 58.0);
    std::printf("[pit] trapped=%d\n", basins);
    CHECK(basins == 0);

    // RISER PROBE:    // RISER PROBE: just outside each road's sidewalk line, the ground sits
    // within a stepable 0.35 m of the road deck (the sidewalk outer face is
    // a curb-step, not a wall).
    double worstRiser = 0;
    for (double t = -50; t <= 50; t += 5.0) {
        // north edge road runs z=-60; sample the block-side sidewalk outer line
        const double deckY = finalGround(t, -60.0);
        const double insideY = finalGround(t, -60.0 + net.width * 0.5 +
                                                  net.sidewalk + 1.0);
        worstRiser = std::max(worstRiser, deckY + 0.15 - insideY);
    }
    std::printf("[pit] worstRiser=%.2f\n", worstRiser);
    CHECK(worstRiser < 0.35 + 0.20);   // curb + small tolerance
}

#include "../src/engine/procgen/city/city_lots.h"

TEST_CASE(metro_blocks_reach_city_scale_and_aspect) {
    // Drive feedback: "streets need more space between them and need to be
    // longer... Those blocks don't even feel like NYC blocks." NYC blocks
    // run ~80 x 274 m — LONG: square-celled subdivision structurally cannot
    // produce that. The generator now cuts each district face with an
    // ANISOTROPIC cell (short axis ~lot depth, long axis city-scale), so
    // long blocks exist at all.
    RoadNet net;
    net.width = 11.0;
    net.autoRoundabout = false;
    nlohmann::json gen = { { "kind", "metro" },    { "radius", 700 },
                           { "hotspots", 5 },      { "block_size", 150 },
                           { "seed", 11 },         { "freeways", false },
                           { "min_road_len", 24 }, { "terrain_aware", false } };
    applyGenerateRecipe(net, gen);
    RoadGraph g = navRoadGraph(net);
    std::vector<Poly2> blocks = extractBlocks(g, 400.0);
    CHECK(blocks.size() > 30u);

    int longBlocks = 0;
    std::vector<double> areas;
    for (const Poly2& b : blocks) {
        OBB2 o = orientedBoundingBox(b);
        const double longSide = 2.0 * std::max(o.half[0], o.half[1]);
        const double shortSide = 2.0 * std::min(o.half[0], o.half[1]);
        if (shortSide < 20.0) continue;              // slivers aren't blocks
        areas.push_back(longSide * shortSide);
        if (longSide / std::max(1.0, shortSide) >= 2.0 && longSide >= 150.0)
            ++longBlocks;
    }
    std::sort(areas.begin(), areas.end());
    const double median = areas.empty() ? 0 : areas[areas.size() / 2];
    std::printf("[scale] blocks=%zu median=%.0f m2 long=%d\n", blocks.size(),
                median, longBlocks);
    CHECK(median >= 7000.0);        // city-scale, not courtyard-scale
    CHECK(longBlocks >= 8);         // real long blocks exist
}

TEST_CASE(park_pads_and_fences_stay_out_of_carriageways) {
    // Drive feedback E1/B4: "parks don't sit on their lots and explode into
    // the street including fences... park fences sunk into the road." Every
    // park/green pad vertex must clear every sampled road ribbon — the same
    // protection buildings always had.
    RoadNet net;
    net.width = 13.0;   // arterial width: the case the old 4 m assumption broke
    net.autoRoundabout = false;
    nlohmann::json gen = { { "kind", "metro" },    { "radius", 600 },
                           { "hotspots", 4 },      { "block_size", 130 },
                           { "seed", 21 },         { "freeways", false },
                           { "min_road_len", 24 }, { "terrain_aware", false } };
    applyGenerateRecipe(net, gen);
    RoadGraph rg = navRoadGraph(net);

    LotParams lp;
    lp.seed = 7;
    lp.roadMargin = 8.0;
    NetLotResult grown =
        growLotBuildingsOnNets({ net }, lp, EdgeBlockParams{}, 5.0, &rg);
    int parks = 0, violations = 0;
    for (const LotBuilding& lb : grown.lots) {
        if (lb.type != "park" && lb.type != "green") continue;
        if (lb.recipe.rfind("underfreeway", 0) == 0) continue;   // in the ROW by design
        ++parks;
        for (const Vec2& v : lb.pad) {
            for (const RoadEdge& e : rg.edges) {
                if (e.a < 0 || e.b < 0) continue;
                const Vec2& a = rg.nodes[e.a].pos;
                const Vec2& b = rg.nodes[e.b].pos;
                Vec2 ab = b - a;
                const Real l2 = ab.lengthSquared();
                Real t = l2 > 1e-12 ? dot(v - a, ab) / l2 : 0.0;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                if ((a + ab * t - v).length() < e.width * 0.5) ++violations;
            }
        }
    }
    std::printf("[park] parks=%d violations=%d\n", parks, violations);
    CHECK(parks > 0);
    CHECK(violations == 0);
}

TEST_CASE(retaining_walls_front_deep_cut_faces) {
    // R6a (hillcity finding): the smoothstep conform leaves a near-vertical
    // face where a road cuts DEEP into a hill. buildRoadWalls (co-designed
    // with that feather) fronts each deep face with a retaining wall just
    // outside the graded corridor; shallow cuts and flats stay open ground.
    auto ridge = [](double x, double z) {
        (void)x;   // a steep ridge north of the road line
        return z > 0 ? std::min(24.0, z * 0.8) : 0.0;
    };
    RoadNet net;
    net.width = 8.0;
    net.sidewalk = 2.0;
    net.autoRoundabout = false;
    net.heightAt = ridge;
    net.nodes = { Vec2(-120, 6), Vec2(120, 6) };   // hugs the ridge foot
    net.edges = { { 0, 1 } };

    StructureParams p;
    p.minWall = 3.5;
    StructureSet ws = buildRoadWalls(net, p);
    std::printf("[walls] segments=%zu\n", ws.walls.size());
    CHECK(!ws.walls.empty());          // the deep cut face gets its wall
    CHECK(!ws.mesh.vertices.empty());

    const double flatHalf = 4.0 + 2.0 + 2.0;   // halfWidth + sidewalk + 2
    double worstTopGap = 0;
    for (const WallSegment& w : ws.walls) {
        // Walls only on the CUT (ridge) side, outside the graded corridor,
        // never in the walkway.
        CHECK(w.topA.z > 6.0 + flatHalf - 0.5);
        CHECK(w.retaining);
        // The top edge meets the hill line at the feather's end (the wall
        // FRONTS the steep feather face; terrain behind it climbs on).
        const double natural = ridge(w.topA.x, 6.0 + flatHalf + 8.0);
        worstTopGap = std::max(worstTopGap, std::fabs(w.topA.y - natural));
        CHECK(w.dropA > 0 || w.dropB > 0);
    }
    std::printf("[walls] worstTopGap=%.2f\n", worstTopGap);
    CHECK(worstTopGap < 1.5);   // top tracks the hill line (grade-limited deck)

    // Control: a flat site grows no walls at all.
    RoadNet flat = net;
    flat.heightAt = [](double, double) { return 2.0; };
    CHECK(buildRoadWalls(flat, p).walls.empty());
}
