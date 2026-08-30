#include "test_framework.h"

#include "../src/engine/procgen/city/surface_field.h"
#include "../src/engine/procgen/city/block_grade.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/road_constraints.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/terrain.h"

#include <cmath>
#include <vector>

using namespace engine;

namespace {

// A deterministic LCG so the parity sweep is reproducible (no <random> global state).
struct Lcg {
    uint64_t s;
    explicit Lcg(uint64_t seed) : s(seed) {}
    double next() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return ((s >> 11) & 0xFFFFFFFF) / double(0xFFFFFFFF); }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

// A spread of pads + ramps across a patch, so the grid holds many overlapping bins.
std::vector<TerrainFlatten> sampleRegions() {
    std::vector<TerrainFlatten> regions;
    Lcg rng(12345);
    for (int i = 0; i < 60; ++i) {
        double cx = rng.range(-200, 200), cz = rng.range(-200, 200);
        double h = rng.range(-5, 5);
        if (rng.next() < 0.5) {
            double r = rng.range(6, 22);
            std::vector<Vec3> poly = {Vec3(cx - r, 0, cz - r), Vec3(cx + r, 0, cz - r),
                                      Vec3(cx + r, 0, cz + r), Vec3(cx - r, 0, cz + r)};
            regions.push_back(makeFlattenPad(std::move(poly), h, rng.range(3, 10)));
        } else {
            double ang = rng.range(0, 6.28), len = rng.range(20, 80);
            Vec3 a(cx, 0, cz), b(cx + std::cos(ang) * len, 0, cz + std::sin(ang) * len);
            regions.push_back(makeFlattenRamp(a, b, h, h + rng.range(-4, 4),
                                              rng.range(4, 8), rng.range(3, 9)));
        }
    }
    return regions;
}

int maxDegree(const RoadGraph& g) {
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) {
        if (e.a >= 0 && e.a < (int)deg.size()) deg[e.a]++;
        if (e.b >= 0 && e.b < (int)deg.size()) deg[e.b]++;
    }
    int m = 0;
    for (int d : deg) m = std::max(m, d);
    return m;
}

}  // namespace

// The spatial index must be a PURE accelerator: same height as the linear scan
// at every point, with and without dilation (ADR-0075 Phase 0 correctness gate).
TEST_CASE(flatten_grid_matches_linear_scan) {
    std::vector<TerrainFlatten> regions = sampleRegions();
    FlattenGrid grid = buildFlattenGrid(regions);
    CHECK(!grid.empty());

    Lcg rng(999);
    int mismatches = 0;
    for (int i = 0; i < 4000; ++i) {
        double x = rng.range(-260, 260), z = rng.range(-260, 260);
        double base = std::sin(x * 0.01) * 3.0;
        for (double dil : {0.0, 4.0, 20.0}) {
            double lin = applyFlatten(regions, x, z, base, dil);
            double idx = applyFlatten(grid, regions, x, z, base, dil);
            if (std::fabs(lin - idx) > 1e-9) ++mismatches;
        }
    }
    CHECK(mismatches == 0);
}

// A point far outside every footprint returns the base untouched, via the index.
TEST_CASE(flatten_grid_returns_base_outside) {
    std::vector<TerrainFlatten> regions = sampleRegions();
    FlattenGrid grid = buildFlattenGrid(regions);
    double base = 7.5;
    CHECK_APPROX(applyFlatten(grid, regions, 10000.0, -8000.0, base, 0.0), base, 1e-9);
}

// Empty region set => empty grid => queries fall back cleanly to the base.
TEST_CASE(flatten_grid_empty_is_safe) {
    std::vector<TerrainFlatten> none;
    FlattenGrid grid = buildFlattenGrid(none);
    CHECK(grid.empty());
    CHECK_APPROX(applyFlatten(grid, none, 3.0, 4.0, 2.0, 0.0), 2.0, 1e-9);
}

// SurfaceField.height reproduces base + applyFlatten; the indexed and unindexed
// paths agree; and it hands out a working sampler closure.
TEST_CASE(surface_field_height_and_sampler) {
    HeightField base = [](double x, double z) { return 0.2 * x - 0.1 * z; };
    std::vector<TerrainFlatten> regions = sampleRegions();

    SurfaceField field(base);
    field.setEdits(regions);        // index stale -> linear fold
    SurfaceField indexed(base);
    indexed.setEdits(regions);
    indexed.rebuildIndex();         // index live

    Lcg rng(77);
    for (int i = 0; i < 1500; ++i) {
        double x = rng.range(-220, 220), z = rng.range(-220, 220);
        double want = applyFlatten(regions, x, z, base(x, z), 0.0);
        CHECK_APPROX(field.height(x, z), want, 1e-9);
        CHECK_APPROX(indexed.height(x, z), want, 1e-9);
        CHECK_APPROX(indexed.sampler()(x, z), want, 1e-9);
    }
}

// Normal of a planar tilt points the expected way (finite differences on height).
TEST_CASE(surface_field_normal_on_slope) {
    // Ground rises toward +x at grade 0.5: n.x < 0, n.y > 0, |n|=1.
    HeightField base = [](double x, double) { return 0.5 * x; };
    SurfaceField field(base);
    Vec3 n = field.normal(10.0, 10.0);
    CHECK(n.y > 0.0f);
    CHECK(n.x < 0.0f);
    CHECK_APPROX(std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z), 1.0, 1e-5);
}

// DaylightBatter earthwork (ADR-0075 Phase 1): outside a flat corridor deck the
// ground cuts INTO the uphill slope and FILLS OUT the downhill one, each batter
// daylighting where it meets natural terrain — never proud of the hill on the cut
// side nor sunk below it on the fill side.
TEST_CASE(daylight_batter_cuts_uphill_fills_downhill) {
    auto natural = [](double x) { return 0.3 * x; };   // hillside rising toward +x
    std::vector<Vec3> poly = {Vec3(-4, 0, -50), Vec3(4, 0, -50),
                              Vec3(4, 0, 50), Vec3(-4, 0, 50)};
    TerrainFlatten f = makeFlattenPad(std::move(poly), 0.0, 6.0);  // flat deck at y=0
    f.falloffMode = TerrainFlatten::Falloff::DaylightBatter;
    f.cutBatter = 1.0; f.fillBatter = 0.6; f.falloff = 26.0;
    std::vector<TerrainFlatten> regs = {f};
    auto H = [&](double x) { return applyFlatten(regs, x, 0.0, natural(x), 0.0); };

    CHECK_APPROX(H(0.0), 0.0, 1e-9);                     // deck is flat inside the corridor
    CHECK(H(5.5) < natural(5.5) - 0.05);                 // uphill: cut below natural
    CHECK_APPROX(H(9.0), natural(9.0), 1e-6);            // cut daylights back to natural
    CHECK(H(-6.0) > natural(-6.0) + 0.05);               // downhill: fill above natural
    CHECK_APPROX(H(-12.0), natural(-12.0), 1e-6);        // fill daylights back to natural
    // Invariants: the cut never rises above the hill, the fill never sinks below it.
    for (double x = 4.5; x < 30; x += 0.5)  CHECK(H(x) <= natural(x) + 1e-6);
    for (double x = -30; x < -4.5; x += 0.5) CHECK(H(x) >= natural(x) - 1e-6);
}

// The spatial index reproduces batter footprints too (not just smoothstep pads).
TEST_CASE(daylight_batter_index_parity) {
    auto natural = [](double x, double z) { return 0.25 * x + 0.1 * z; };
    std::vector<TerrainFlatten> regs;
    Lcg rng(4242);
    for (int i = 0; i < 20; ++i) {
        double cx = rng.range(-150, 150), cz = rng.range(-150, 150), len = rng.range(30, 90);
        Vec3 a(cx, 0, cz), b(cx + rng.range(-1, 1) * len, 0, cz + len);
        TerrainFlatten f = makeFlattenRamp(a, b, natural(cx, cz), natural(cx, cz + len), 5.0, 6.0);
        f.falloffMode = TerrainFlatten::Falloff::DaylightBatter;
        f.cutBatter = 1.0; f.fillBatter = 0.6; f.falloff = 24.0;
        regs.push_back(std::move(f));
    }
    FlattenGrid grid = buildFlattenGrid(regs);
    int mismatches = 0;
    for (int i = 0; i < 3000; ++i) {
        double x = rng.range(-200, 200), z = rng.range(-200, 200);
        double b = natural(x, z);
        if (std::fabs(applyFlatten(regs, x, z, b, 0.0) - applyFlatten(grid, regs, x, z, b, 0.0)) > 1e-9)
            ++mismatches;
    }
    CHECK(mismatches == 0);
}

// Block grading (ADR-0075 Phase 2): a block on a gentle slope tilts with its
// bounding ground into ONE graded plane; a block on a steep slope TERRACES into
// flat steps so no single grade break is a cliff.
TEST_CASE(block_grades_to_a_plane_then_terraces) {
    Poly2 block = {Vec2(0, 0), Vec2(60, 0), Vec2(60, 60), Vec2(0, 60)};

    // Gentle: 5% grade across 60 m = 3 m drop < maxDrop -> one tilted plane.
    HeightSampler gentle = [](double x, double) { return 0.05 * x; };
    std::vector<TerrainFlatten> g = gradeBlock(block, gentle);
    CHECK(g.size() == 1);
    CHECK_APPROX(g[0].dx, 0.05, 0.01);          // tilts with the ground
    CHECK(std::fabs(g[0].dz) < 0.01);
    CHECK_APPROX(g[0].planeY(30, 30), 0.05 * 30, 0.2);   // meets the ground mid-block

    // Steep: 30% grade across 60 m = 18 m drop > maxDrop(6) -> terrace into steps.
    // Each step keeps a REDUCED tilt (at most maxBandRise across its width) and
    // the steps still climb — not dead-flat stairs (the device: "I don't like
    // the harsh stair steps"), not the full plane either (then no steps).
    HeightSampler steep = [](double x, double) { return 0.30 * x; };
    BlockGradeParams bp;
    std::vector<TerrainFlatten> t = gradeBlock(block, steep, bp);
    CHECK(t.size() >= 2);                        // several terraces
    const double bw = 60.0 / static_cast<double>(t.size());   // band width along x
    double prev = -1e30; int rising = 0;
    for (const TerrainFlatten& f : t) {
        CHECK(f.dx > 1e-6);                                  // tilted WITH the hill...
        CHECK(f.dx * bw <= bp.maxBandRise + 1e-6);           // ...by at most the budget
        CHECK(f.dx < 0.30 - 1e-6);                           // ...and less than the plane
        CHECK(std::fabs(f.dz) < 1e-6);
        double lvl = f.planeY(30, 30);           // the band's height on its own line
        if (lvl > prev) ++rising;
        prev = lvl;
    }
    CHECK(rising >= 1);                          // steps climb the slope
    // The riser: consecutive bands still differ by roughly a step's worth, so
    // the terraces are terraces — the rise across a band plus its riser spans
    // the block's slope, and the riser carries most of it.
    const double lvl0 = t.front().planeY(bw * 0.5, 30), lvl1 = t[1].planeY(bw * 1.5, 30);
    CHECK(lvl1 - lvl0 > bp.maxBandRise);         // more than one band's own rise
}

// Overlap priority (ADR-0075 Phase 2): a flat building pad (priority 0) stays flat
// over the tilted block grade (priority -1) it sits on — the higher priority wins
// inside the overlap — while the block still grades its own yards.
TEST_CASE(flatten_priority_pad_beats_block_grade) {
    std::vector<Vec3> blockPoly = {Vec3(-30, 0, -30), Vec3(30, 0, -30),
                                   Vec3(30, 0, 30), Vec3(-30, 0, 30)};
    TerrainFlatten block = makeFlattenPad(blockPoly, 0.0, 4.0);
    block.dx = 0.3; block.priority = -1;              // tilted, low priority
    std::vector<Vec3> padPoly = {Vec3(-10, 0, -10), Vec3(10, 0, -10),
                                 Vec3(10, 0, 10), Vec3(-10, 0, 10)};
    TerrainFlatten pad = makeFlattenPad(padPoly, 5.0, 4.0);   // flat at 5, priority 0
    std::vector<TerrainFlatten> regs = {block, pad};

    CHECK_APPROX(applyFlatten(regs, 0, 0, 100.0, 0.0), 5.0, 1e-9);   // pad centre: flat
    CHECK_APPROX(applyFlatten(regs, 8, 8, 100.0, 0.0), 5.0, 1e-9);   // block tilt(2.4) < pad, pad wins
    CHECK_APPROX(applyFlatten(regs, 20, 0, 100.0, 0.0),              // outside pad, in block
                 block.planeY(20, 0), 1e-9);                          // -> block grade applies
}

// StructureSet walls (ADR-0075 Phase 1b): a steep sidehill cut that can't daylight
// within reach gets a retaining wall; flat ground gets none.
TEST_CASE(road_walls_only_where_cuts_are_steep) {
    RoadEntity net;
    net.look.defaultWidth = 12.0; net.look.sidewalk = 3.5;
    net.graph.nodes = {RoadNode{Vec2(0, -100)}, RoadNode{Vec2(0, 100)}};
    net.graph.addEdge(0, 1, 12.0);

    const RoadGroundFn flatGround = [](double, double) { return 5.0; };   // flat: batter daylights at once
    StructureSet flat = buildRoadWalls(net, flatGround);
    CHECK(flat.walls.empty());
    CHECK(flat.mesh.vertices.empty());

    const RoadGroundFn steepGround = [](double x, double) { return 2.0 * x; };  // steep sidehill (grade 2.0)
    StructureSet steep = buildRoadWalls(net, steepGround);
    CHECK(!steep.walls.empty());
    CHECK(!steep.mesh.vertices.empty());
    CHECK(steep.mesh.indices.size() % 3 == 0);
    CHECK(steep.colliderEdges.size() == steep.walls.size());
    bool anyRetain = false; double maxDrop = 0;
    for (const WallSegment& w : steep.walls) {
        if (w.retaining) anyRetain = true;
        maxDrop = std::max(maxDrop, std::max(w.dropA, w.dropB));
    }
    CHECK(anyRetain);          // the uphill cut needs holding back
    CHECK(maxDrop > 0.5);      // and it's a real wall, not a sliver
}

// The autoRoundabout policy on a net is honoured by the conform/mesh graph pass
// (ADR-0075 Phase 0): a degree-5 star is promoted to a roundabout ring only when
// the net asks for it — the generator's autoRoundabout=false must survive into
// constrainedNetGraph (it used to be overridden by default rules).
TEST_CASE(net_autoroundabout_policy_is_honoured) {
    RoadEntity star;
    star.look.defaultWidth = 10.0;
    star.look.sidewalk = 2.5;
    star.graph.nodes = {RoadNode{Vec2(0, 0)}, RoadNode{Vec2(30, 0)}, RoadNode{Vec2(9, 28)},
                        RoadNode{Vec2(-24, 17)}, RoadNode{Vec2(-24, -17)}, RoadNode{Vec2(9, -28)}};
    star.graph.addEdge(0, 1, 10.0);  // centre is degree 5
    star.graph.addEdge(0, 2, 10.0);
    star.graph.addEdge(0, 3, 10.0);
    star.graph.addEdge(0, 4, 10.0);
    star.graph.addEdge(0, 5, 10.0);

    // navRoadGraph wraps constrainedNetGraph — the shared mesh+conform graph pass.
    // With autoRoundabout on, the degree-5 centre is promoted to a ring.
    RoadGraph rawish = navRoadGraph(
        [&] { RoadEntity n = star; n.look.autoRoundabout = true; return n; }(), nullptr);
    CHECK(maxDegree(rawish) <= 4);   // promoted: every survivor is degree <= 3 (ring) or <= 4

    star.look.autoRoundabout = false;
    RoadGraph kept = navRoadGraph(star, nullptr);
    CHECK(maxDegree(kept) >= 5);     // NOT promoted: the busy centre survives intact

    // And the mesher runs on the unpromoted net without crashing.
    RenderMesh m = buildRoadNetMesh(star, nullptr);
    CHECK(!m.vertices.empty());
}


// The overlap CONTRACT, pinned directly (it was only exercised indirectly
// through block grading): the priority ladder is a hard override — grades -1,
// generic 0, roads 1, building pads 2 — and among equal priorities the LOWEST
// plane wins (the junction rule). Dilation grows footprints for coarse-LOD
// queries: a point outside a narrow band adopts the band's plane once the
// query dilation reaches it. These are the rules every conform consumer
// (CDLOD tiles, lot pads, walkway ramps, vegetation keep-out) builds on.
TEST_CASE(flatten_priority_ladder_and_dilation_contract) {
    auto square = [](double cx, double cz, double h) {
        return std::vector<Vec3>{{cx - h, 0, cz - h}, {cx + h, 0, cz - h},
                                 {cx + h, 0, cz + h}, {cx - h, 0, cz + h}};
    };
    const double base = 20.0;

    // Nested overlap: block grade (-1, plane 2) under a road (1, plane 5)
    // under a building pad (2, plane 10).
    std::vector<TerrainFlatten> r;
    r.push_back(makeFlattenPad(square(0, 0, 40), 2.0, 4.0));   // grade
    r.back().priority = -1;
    r.push_back(makeFlattenPad(square(0, 0, 20), 5.0, 4.0));   // road
    r.back().priority = 1;
    r.push_back(makeFlattenPad(square(0, 0, 8), 10.0, 4.0));   // pad
    r.back().priority = 2;
    CHECK_APPROX(applyFlatten(r, 0, 0, base), 10.0, 1e-9);     // pad wins inside all
    CHECK_APPROX(applyFlatten(r, 14, 0, base), 5.0, 1e-9);     // road wins past the pad
    CHECK_APPROX(applyFlatten(r, 30, 0, base), 2.0, 1e-9);     // grade owns the rest

    // Equal priority: the LOWEST plane wins.
    std::vector<TerrainFlatten> eq;
    eq.push_back(makeFlattenPad(square(0, 0, 10), 7.0, 4.0));
    eq.push_back(makeFlattenPad(square(0, 0, 10), 4.0, 4.0));
    CHECK_APPROX(applyFlatten(eq, 0, 0, base), 4.0, 1e-9);

    // Dilation: a 2 m-wide ramp band; a point 2.5 m off its edge sees natural
    // ground at dilate 0 and the band's plane once dilation reaches it.
    std::vector<TerrainFlatten> band;
    band.push_back(makeFlattenRamp(Vec3(-30, 0, 0), Vec3(30, 0, 0), 6.0, 6.0,
                                   1.0, 0.75));
    const double off = applyFlatten(band, 0.0, 3.5, base, 0.0);
    CHECK(std::fabs(off - base) < 1e-9);                       // untouched
    CHECK_APPROX(applyFlatten(band, 0.0, 3.5, base, 4.0), 6.0, 1e-9);
    FlattenGrid g2 = buildFlattenGrid(band);
    CHECK(!flattenCovers(g2, band, 0.0, 3.5, 0.0));
    CHECK(flattenCovers(g2, band, 0.0, 3.5, 4.0));
}
