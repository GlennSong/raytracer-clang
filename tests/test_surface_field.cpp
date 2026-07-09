#include "test_framework.h"

#include "../src/engine/procgen/city/surface_field.h"
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

// The autoRoundabout policy on a net is honoured by the conform/mesh graph pass
// (ADR-0075 Phase 0): a degree-5 star is promoted to a roundabout ring only when
// the net asks for it — the generator's autoRoundabout=false must survive into
// constrainedNetGraph (it used to be overridden by default rules).
TEST_CASE(net_autoroundabout_policy_is_honoured) {
    RoadNet star;
    star.nodes = {Vec2(0, 0), Vec2(30, 0), Vec2(9, 28), Vec2(-24, 17),
                  Vec2(-24, -17), Vec2(9, -28)};
    star.edges = {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}};  // centre is degree 5
    star.width = 10.0;
    star.sidewalk = 2.5;

    // navRoadGraph wraps constrainedNetGraph — the shared mesh+conform graph pass.
    // With autoRoundabout on, the degree-5 centre is promoted to a ring.
    RoadGraph rawish = navRoadGraph([&] { RoadNet n = star; n.autoRoundabout = true; return n; }());
    CHECK(maxDegree(rawish) <= 4);   // promoted: every survivor is degree <= 3 (ring) or <= 4

    star.autoRoundabout = false;
    RoadGraph kept = navRoadGraph(star);
    CHECK(maxDegree(kept) >= 5);     // NOT promoted: the busy centre survives intact

    // And the mesher runs on the unpromoted net without crashing.
    RenderMesh m = buildRoadNetMesh(star);
    CHECK(!m.vertices.empty());
}
