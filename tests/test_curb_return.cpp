#include "test_framework.h"

#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/triangulate.h"
#include <cmath>

using namespace engine;

namespace {
double len(const Vec2& v) { return std::sqrt(v.x * v.x + v.y * v.y); }
// The tightest bend along a polyline as a radius (circumradius of each triple; +inf for a
// straight run). Mirrors the metric fairHermite caps against.
double minCircum(const std::vector<Vec2>& poly) {
    double worst = 1e30;
    for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
        const Vec2 &a = poly[i - 1], &b = poly[i], &c = poly[i + 1];
        double ab = len(b - a), bc = len(c - b), ca = len(a - c);
        double area2 = std::fabs(cross(b - a, c - a));
        if (area2 < 1e-9) continue;
        worst = std::min(worst, (ab * bc * ca) / (2.0 * area2));
    }
    return worst;
}
}  // namespace

TEST_CASE(fair_hermite_relaxes_an_over_tight_curve_to_the_min_radius) {
    // A short edge with huge tangents loops back on itself (radius far below 6 m).
    Vec2 p0(0, 0), p1(4, 0), m0(0, 30), m1(0, -30);
    std::vector<Vec2> tight = fairHermite(p0, m0, p1, m1, 24, 0.0);   // no cap: stays tight
    CHECK(minCircum(tight) < 6.0);
    std::vector<Vec2> fair = fairHermite(p0, m0, p1, m1, 24, 6.0);    // capped at 6 m
    CHECK(minCircum(fair) >= 6.0 - 0.5);                              // no longer over-tight
    CHECK(fair.front().x == 0.0 && fair.back().x == 4.0);            // endpoints preserved
}

TEST_CASE(fair_hermite_leaves_a_gentle_curve_untouched) {
    // Gentle tangents -> a wide curve already well above the limit: returned as-is.
    Vec2 p0(0, 0), p1(40, 0), m0(20, 8), m1(20, -8);
    std::vector<Vec2> ref = fairHermite(p0, m0, p1, m1, 16, 0.0);
    std::vector<Vec2> capped = fairHermite(p0, m0, p1, m1, 16, 6.0);
    CHECK(capped.size() == ref.size());
    CHECK(minCircum(capped) > 6.0);
    for (std::size_t i = 0; i < ref.size(); ++i)
        CHECK(len(capped[i] - ref[i]) < 1e-9);          // identical samples (scale stayed 1)
}

namespace {
double triArea(const Vec2& a, const Vec2& b, const Vec2& c) {
    return std::fabs(cross(b - a, c - a)) * 0.5;
}
double shoelace(const std::vector<Vec2>& p) {
    double a = 0.0;
    int n = static_cast<int>(p.size());
    for (int i = 0; i < n; ++i) a += cross(p[i], p[(i + 1) % n]);
    return std::fabs(a) * 0.5;
}
double sumTriAreas(const std::vector<Vec2>& p, const std::vector<std::array<int, 3>>& tris) {
    double a = 0.0;
    for (const auto& t : tris) a += triArea(p[t[0]], p[t[1]], p[t[2]]);
    return a;
}
}

namespace {
}

TEST_CASE(triangulate_convex_tiles_the_polygon) {
    std::vector<Vec2> sq = { Vec2(0, 0), Vec2(4, 0), Vec2(4, 4), Vec2(0, 4) };
    auto tris = triangulatePolygon(sq);
    CHECK(tris.size() == 2);                                  // n-2 triangles
    CHECK_APPROX(sumTriAreas(sq, tris), 16.0, 1e-9);         // exact tiling, no overlap/gap
}

TEST_CASE(triangulate_nonconvex_notch_tiles_without_overlap) {
    // A square with the top edge pushed into a V — (3,3) is a reflex vertex, exactly the
    // case where fanning from an interior point self-overlaps. Ear clipping must tile it.
    std::vector<Vec2> notch = { Vec2(0, 0), Vec2(6, 0), Vec2(6, 6), Vec2(3, 3), Vec2(0, 6) };
    auto tris = triangulatePolygon(notch);
    CHECK(static_cast<int>(tris.size()) == static_cast<int>(notch.size()) - 2);
    CHECK_APPROX(sumTriAreas(notch, tris), shoelace(notch), 1e-9);   // tiles exactly (=27)
    for (const auto& t : tris)
        CHECK(triArea(notch[t[0]], notch[t[1]], notch[t[2]]) > 1e-9);   // none degenerate
}

TEST_CASE(triangulate_dedupes_a_ring_with_coincident_vertices) {
    // A sharp junction's ring repeats coincident mouth corners (cornerRadius 0). The
    // triangulator must dedupe and still tile the underlying square (covers the centre).
    std::vector<Vec2> ring = { Vec2(5, -5), Vec2(5, 5), Vec2(5, 5), Vec2(-5, 5),
                               Vec2(-5, 5), Vec2(-5, -5), Vec2(-5, -5), Vec2(5, -5) };
    auto tris = triangulatePolygon(ring);
    CHECK(tris.size() == 2);                                  // dedups to a 4-vert square
    CHECK_APPROX(sumTriAreas(ring, tris), 100.0, 1e-9);     // tiles the full 10x10
}

TEST_CASE(triangulate_handles_cw_and_degenerate) {
    // Clockwise input is normalized to CCW internally and still tiles.
    std::vector<Vec2> cw = { Vec2(0, 0), Vec2(0, 4), Vec2(4, 4), Vec2(4, 0) };
    CHECK_APPROX(sumTriAreas(cw, triangulatePolygon(cw)), 16.0, 1e-9);
    CHECK(triangulatePolygon({ Vec2(0, 0), Vec2(1, 1) }).empty());     // < 3 verts
}

// A slab round a lift core: rectangular holes in rows, their edges on the same lines and 0.15 m apart -- in plan and
// mirrored (the frame emitPolygon projects an upward face into). Every case tiles the outline less its holes exactly:
// the port sorted the holes by their first vertex rather than their leftmost and kept an outer node the collinear
// filter had removed, and a mirrored core slab paved over its shafts (Glenn, 2026-10-28: "the hole for the floor
// doesn't get cut and you end up going through a plane of geometry").
TEST_CASE(triangulate_with_rows_of_aligned_holes_leaves_them_open) {
    auto rect = [](double x0, double y0, double x1, double y1) { return Poly2{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; };
    auto mirror = [](Poly2 p) { for (Vec2& v : p) std::swap(v.x, v.y); return p; };
    auto tiled = [](const Poly2& outer, const std::vector<Poly2>& holes) {
        double want = std::fabs(area(outer)), got = 0;
        for (const Poly2& h : holes) want -= std::fabs(area(h));
        for (const auto& t : triangulateWithHoles(outer, holes)) got += std::fabs(area(Poly2{t[0], t[1], t[2]}));
        return std::fabs(got - want) < 1e-6 * std::max(1.0, want);
    };
    const std::vector<Poly2> core = {rect(14.975, 21.24, 17.375, 23.84), rect(17.525, 21.24, 19.925, 23.84),
                                     rect(20.075, 21.24, 22.475, 23.84), rect(22.625, 21.24, 25.025, 23.84),
                                     rect(12.225, 16.16, 14.825, 23.84), rect(25.175, 16.16, 27.775, 23.84)};
    const Poly2 square = rect(0, 0, 40, 40);
    const Poly2 octagon = {{0.70, 0.34}, {39.30, 0.34}, {39.66, 0.70}, {39.66, 39.30}, {39.30, 39.66}, {0.70, 39.66}, {0.34, 39.30}, {0.34, 0.70}};
    int cases = 0, bad = 0;
    for (const Poly2& outer : {square, octagon})
        for (bool m : {false, true}) {
            std::vector<Poly2> hs;
            for (const Poly2& h : core) hs.push_back(m ? mirror(h) : h);
            ++cases;
            if (!tiled(m ? mirror(outer) : outer, hs)) { ++bad; std::printf("    [earcut] the core slab, %s, mirrored %d\n", outer.size() == 4 ? "square" : "octagon", m ? 1 : 0); }
        }
    // ...and rows of 1-6 shafts in 1-3 rows, either stair-high or not, in both frames
    for (int rows = 1; rows <= 3; ++rows)
        for (int n = 1; n <= 6; ++n)
            for (bool stairs : {false, true})
                for (bool m : {false, true}) {
                    std::vector<Poly2> hs;
                    for (int r = 0; r < rows; ++r) {
                        const double y0 = 8 + r * 9.0;
                        double x = 8;
                        if (stairs) { hs.push_back(rect(x, y0 - 5.1, x + 2.6, y0 + 2.6)); x += 2.75; }
                        for (int i = 0; i < n; ++i) { hs.push_back(rect(x, y0, x + 2.4, y0 + 2.6)); x += 2.55; }
                        if (stairs) hs.push_back(rect(x, y0 - 5.1, x + 2.6, y0 + 2.6));
                    }
                    if (m) for (Poly2& h : hs) h = mirror(h);
                    ++cases;
                    if (!tiled(m ? mirror(square) : square, hs)) { ++bad; std::printf("    [earcut] %d rows of %d%s, mirrored %d\n", rows, n, stairs ? " with stairs" : "", m ? 1 : 0); }
                }
    std::printf("    [earcut] %d slabs, %d paved over a hole\n", cases, bad);
    CHECK(bad == 0);
}

