// P8 footprint-first city, stage A gates: the footprint polygon is the
// contract every later stage builds against, so its guarantees are proven
// here on synthetic terrains — simple/CCW/contains-center, terrain respected
// (no vertices over the sea), skinny lobes killed (the isthmus fixture),
// hostile ground degrades honestly, gates evenly spaced, and the whole stage
// bit-identical across runs. Plus insetRingArcs at the NEW arterial depth
// regime (it was only ever exercised at block scale).
#include "test_framework.h"

#include "../src/engine/procgen/city/city_footprint.h"
#include "../src/engine/procgen/city/patch_fabric.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {

constexpr double kPi = 3.14159265358979323846;

double segDist(const Vec2& a, const Vec2& b, const Vec2& p) {
    Vec2 ab = b - a;
    double len2 = ab.lengthSquared();
    double t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return (p - (a + ab * t)).length();
}

double boundaryDist(const Poly2& poly, const Vec2& p) {
    double best = 1e30;
    for (std::size_t i = 0; i < poly.size(); ++i)
        best = std::min(best,
                        segDist(poly[i], poly[(i + 1) % poly.size()], p));
    return best;
}

// Flat plain: everything buildable.
double plainHeight(double, double) { return 10.0; }

// West half is deep sea (sharp coast at x = 0; the transition band reads as
// Steep to the slope sampler, which is fine — it is unbuildable either way).
double coastHeight(double x, double) { return x < 0.0 ? -40.0 : 10.0; }

// Two buildable basins joined by a narrow neck: land sits at height 10 inside
// the region, deep water outside. Neck half-width 60m — far below the
// minWidth the city passes, so the opening must cut it.
double isthmusHeight(double x, double z) {
    const double da = std::hypot(x + 600.0, z);
    const double db = std::hypot(x - 600.0, z);
    const bool inNeck = std::abs(z) < 60.0 && std::abs(x) < 600.0;
    return (da < 500.0 || db < 500.0 || inNeck) ? 10.0 : -40.0;
}

BuildabilityConfig seaConfig() {
    BuildabilityConfig cfg;
    cfg.seaLevel = 0.0;
    return cfg;
}

bool polysIdentical(const Poly2& a, const Poly2& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].x != b[i].x || a[i].y != b[i].y) return false;
    return true;
}

}  // namespace

TEST_CASE(footprint_on_open_plain_fills_the_site_disc) {
    FootprintParams fp;
    fp.minWidth = 1200.0;
    Footprint f = buildFootprint(Vec2(900, 900), 1400.0, plainHeight,
                                 BuildabilityConfig{}, fp);
    CHECK(!f.degraded);
    CHECK(f.polygon.size() >= 12);
    CHECK(isCCW(f.polygon));
    CHECK(pointInPolygon(f.polygon, f.center));
    // The whole disc is buildable: the footprint should claim most of it.
    const double discArea = kPi * 1400.0 * 1400.0;
    CHECK(area(f.polygon) > 0.60 * discArea);
    CHECK(f.coreDepth > 800.0);
    // Nothing escapes the site radius (plus a cell of contour slack).
    for (const Vec2& v : f.polygon)
        CHECK((v - Vec2(900, 900)).length() < 1400.0 + 2.0 * fp.cell);
}

TEST_CASE(footprint_respects_the_coast) {
    FootprintParams fp;
    fp.minWidth = 800.0;
    Footprint f = buildFootprint(Vec2(500, 0), 1200.0, coastHeight,
                                 seaConfig(), fp);
    CHECK(!f.degraded);
    CHECK(f.polygon.size() >= 8);
    // No vertex over the sea (x < 0), give-or-take contour discretization.
    for (const Vec2& v : f.polygon) CHECK(v.x > -2.0 * fp.cell);
    CHECK(area(f.polygon) > 0.15 * kPi * 1200.0 * 1200.0);
    CHECK(pointInPolygon(f.polygon, f.center));
}

TEST_CASE(footprint_kills_the_skinny_isthmus) {
    FootprintParams fp;
    fp.cell = 40.0;          // resolve the 120m neck
    fp.minWidth = 400.0;     // neck (120m) is far below -> must be cut
    Footprint f = buildFootprint(Vec2(-600, 0), 900.0, isthmusHeight,
                                 seaConfig(), fp);
    CHECK(!f.degraded);
    // Only the western basin survives: nothing east of the neck.
    for (const Vec2& v : f.polygon) CHECK(v.x < 250.0);
    // The kept basin is the ~r500 disc around (-600, 0): full-bodied, not a
    // remnant, with a deep core — the direct observables of lobe-killing.
    const double basinArea = kPi * 500.0 * 500.0;
    CHECK(area(f.polygon) > 0.55 * basinArea);
    CHECK(area(f.polygon) < 1.30 * basinArea);
    CHECK(f.coreDepth > 300.0);
}

TEST_CASE(footprint_hostile_terrain_degrades_to_disc) {
    // All sea: nothing buildable anywhere.
    auto hostile = [](double, double) { return -40.0; };
    FootprintParams fp;
    Footprint f = buildFootprint(Vec2(0, 0), 800.0, hostile, seaConfig(), fp);
    CHECK(f.degraded);
    CHECK(f.polygon.size() == 24);
    CHECK(pointInPolygon(f.polygon, f.center));
    Footprint f2 = buildFootprint(Vec2(0, 0), 800.0, hostile, seaConfig(), fp);
    CHECK(polysIdentical(f.polygon, f2.polygon));
}

TEST_CASE(footprint_gates_are_evenly_spaced) {
    FootprintParams fp;
    fp.minWidth = 1200.0;
    Footprint f = buildFootprint(Vec2(900, 900), 1400.0, plainHeight,
                                 BuildabilityConfig{}, fp);
    placeFootprintGates(f, 1100.0, 7u);
    const double P = perimeter(f.polygon);
    const int expect =
        std::max(3, static_cast<int>(std::llround(P / 1100.0)));
    std::printf("        [footprint gates] perimeter %.0f m -> %zu gates\n", P,
                f.gates.size());
    CHECK(static_cast<int>(f.gates.size()) == expect);
    // Even spacing: every consecutive arc gap within 1.35x of the mean.
    const double mean = P / f.gates.size();
    for (std::size_t i = 0; i < f.gates.size(); ++i) {
        const double a = f.gates[i].s;
        const double b = i + 1 < f.gates.size() ? f.gates[i + 1].s
                                                : f.gates[0].s + P;
        const double gap = b - a;
        CHECK(gap > mean / 1.35 && gap < mean * 1.35);
        // Gates sit on the boundary, outward normals point out.
        CHECK(boundaryDist(f.polygon, f.gates[i].pos) < 1.0);
        CHECK(!pointInPolygon(f.polygon,
                              f.gates[i].pos + f.gates[i].outward * 25.0));
    }
}

TEST_CASE(footprint_connector_gates_face_their_target) {
    FootprintParams fp;
    fp.minWidth = 600.0;
    std::vector<Footprint> sites;
    sites.push_back(buildFootprint(Vec2(0, 0), 1200.0, plainHeight,
                                   BuildabilityConfig{}, fp));
    sites.push_back(buildFootprint(Vec2(3000, 0), 500.0, plainHeight,
                                   BuildabilityConfig{}, fp));
    placeFootprintGates(sites[0], 1100.0, 7u);
    placeFootprintGates(sites[1], 600.0, 7u);
    pairConnectorGates(sites);
    int cityConn = 0, townConn = 0;
    for (const auto& gate : sites[0].gates)
        if (gate.toSite == 1) {
            ++cityConn;
            // The chosen gate faces east, toward the town.
            CHECK(gate.outward.x > 0.3);
        }
    for (const auto& gate : sites[1].gates)
        if (gate.toSite == 0) {
            ++townConn;
            CHECK(gate.outward.x < -0.3);
        }
    CHECK(cityConn == 1);
    CHECK(townConn == 1);
}

TEST_CASE(footprint_is_deterministic) {
    FootprintParams fp;
    fp.minWidth = 800.0;
    Footprint a = buildFootprint(Vec2(500, 0), 1200.0, coastHeight,
                                 seaConfig(), fp);
    Footprint b = buildFootprint(Vec2(500, 0), 1200.0, coastHeight,
                                 seaConfig(), fp);
    CHECK(polysIdentical(a.polygon, b.polygon));
    CHECK(a.center.x == b.center.x && a.center.y == b.center.y);
    CHECK(a.coreDepth == b.coreDepth);
    placeFootprintGates(a, 900.0, 11u);
    placeFootprintGates(b, 900.0, 11u);
    CHECK(a.gates.size() == b.gates.size());
    for (std::size_t i = 0; i < a.gates.size(); ++i) {
        CHECK(a.gates[i].pos.x == b.gates[i].pos.x);
        CHECK(a.gates[i].pos.y == b.gates[i].pos.y);
        CHECK(a.gates[i].s == b.gates[i].s);
    }
}

TEST_CASE(inset_ring_arcs_hold_at_arterial_depth) {
    // A 3km-long kilometer-scale banana — the NEW depth regime for
    // insetRingArcs (block-scale fabric only ever ran it below ~200m).
    Poly2 banana;
    for (int k = 0; k <= 24; ++k) {
        const double t = kPi * k / 24;
        banana.push_back(Vec2(std::cos(t) * 1500.0, std::sin(t) * 1500.0));
    }
    for (int k = 24; k >= 0; --k) {
        const double t = kPi * k / 24;
        banana.push_back(Vec2(std::cos(t) * 700.0, std::sin(t) * 700.0));
    }
    ensureCCW(banana);
    const double depth = 850.0;
    auto arcs = insetRingArcs(banana, depth);
    // The 800m-thick limb cannot host an 850m inset ring... but whatever
    // comes back must satisfy the medial-validity contract exactly.
    const double eps = depth * 0.02 + 0.25;
    int samples = 0;
    for (const auto& arc : arcs)
        for (const Vec2& p : arc) {
            ++samples;
            CHECK(pointInPolygon(banana, p));
            CHECK(boundaryDist(banana, p) >= depth - eps - 1.0);
            CHECK(std::isfinite(p.x) && std::isfinite(p.y));
        }
    std::printf("        [inset@850] %zu arcs, %d samples on the banana\n",
                arcs.size(), samples);
    // And at a depth the banana CAN host, a valid inset appears.
    auto thin = insetRingArcs(banana, 250.0);
    CHECK(!thin.empty());
}
