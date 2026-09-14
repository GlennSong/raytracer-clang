#include "test_framework.h"
#include "../src/engine/procgen/city/site_plan.h"
#include <cmath>

using namespace engine;

// Skyscrapers v2 (ADR-0086): buildings are rectilinear, lots are not. The site
// plan is the layer that makes that true — a frame on the frontage, the largest
// aligned rectangle the lot holds with its yards, and the leftover ground
// classified by where it lies. Pure geometry, pinned here.

namespace {
bool rectilinearIn(const Poly2& rect, const SiteFrame& f, Real tol = 1e-6) {
    if (rect.size() != 4) return false;
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec2 d = normalize(rect[(i + 1) % 4] - rect[i]);
        const Real au = std::fabs(dot(d, f.u)), av = std::fabs(dot(d, f.v));
        if (!(au > 1 - tol || av > 1 - tol)) return false;
    }
    return true;
}
bool allInside(const Poly2& rect, const Poly2& lot) {
    const Vec2 c = centroid(rect);
    for (const Vec2& q : rect)
        if (!pointInPolygon(lot, q + (c - q) * 0.001)) return false;
    return true;
}
Real frameExtent(const Poly2& rect, const SiteFrame& f, bool alongV, bool max) {
    Real best = max ? -1e30 : 1e30;
    for (const Vec2& w : rect) {
        const Vec2 q = f.toFrame(w);
        const Real c = alongV ? q.y : q.x;
        best = max ? std::max(best, c) : std::min(best, c);
    }
    return best;
}
}  // namespace

TEST_CASE(site_frame_follows_the_frontage_edge) {
    // A CCW rectangle whose street is to the south (frontage points -y).
    const Poly2 lot = {{0, 0}, {20, 0}, {20, 30}, {0, 30}};
    const SiteFrame f = siteFrame(lot, {0, -1});
    CHECK(std::fabs(f.u.x - 1) < 1e-9 && std::fabs(f.u.y) < 1e-9);
    CHECK(std::fabs(f.v.x) < 1e-9 && std::fabs(f.v.y - 1) < 1e-9);
    CHECK(std::fabs(f.origin.x) < 1e-9 && std::fabs(f.origin.y) < 1e-9);
    // The same lot handed in CW order and with the street to the east.
    const Poly2 cw = {{0, 30}, {20, 30}, {20, 0}, {0, 0}};
    const SiteFrame g = siteFrame(cw, {1, 0});
    CHECK(std::fabs(dot(g.v, Vec2(-1, 0)) - 1) < 1e-9);   // inward = west
    CHECK(std::fabs(g.origin.x - 20) < 1e-9);
    // Sides: the classification the yards key on.
    CHECK(lotSideOf({0, -1}, {0, -1}) == LotSide::Front);
    CHECK(lotSideOf({0, 1}, {0, -1}) == LotSide::Rear);
    CHECK(lotSideOf({1, 0}, {0, -1}) == LotSide::Side);
}

TEST_CASE(largest_aligned_rect_in_a_trapezoid_is_inside_and_squared_to_the_street) {
    // A frontage-walk trapezoid: 30 m on the street, 20 m at the back, 25 deep.
    const Poly2 lot = {{0, 0}, {30, 0}, {25, 25}, {5, 25}};
    const SiteFrame f = siteFrame(lot, {0, -1});
    const Poly2 r = largestAlignedRect(lot, f, Yards{}, 0.5, 6.0);
    CHECK(r.size() == 4u);
    CHECK(rectilinearIn(r, f));
    CHECK(allInside(r, lot));
    // It takes most of the lot: the exact optimum is 20 x 25 = 500 of 625 m².
    CHECK(area(r) > 0.70 * area(lot));
    CHECK(area(r) <= area(lot));
    // Yards pull the rectangle off the lines: a 3 m rear yard ends it at 22.
    const Poly2 y = largestAlignedRect(lot, f, Yards{0, 0, 3}, 0.5, 6.0);
    CHECK(y.size() == 4u);
    CHECK(frameExtent(y, f, true, true) <= 22.0 + 1e-6);
    CHECK(frameExtent(y, f, true, true) >= 21.0);
}

TEST_CASE(largest_aligned_rect_honours_every_yard) {
    const Poly2 lot = {{0, 0}, {20, 0}, {20, 30}, {0, 30}};
    const SiteFrame f = siteFrame(lot, {0, -1});
    const Yards yards{2.0, 1.0, 4.0};
    const Poly2 r = largestAlignedRect(lot, f, yards, 0.5, 6.0);
    CHECK(r.size() == 4u);
    // Exact answer is x in [1, 19], y in [2, 26]; the raster sits within half a cell.
    CHECK(std::fabs(frameExtent(r, f, false, false) - 1.0) <= 0.5 + 1e-6);
    CHECK(std::fabs(frameExtent(r, f, false, true) - 19.0) <= 0.5 + 1e-6);
    CHECK(std::fabs(frameExtent(r, f, true, false) - 2.0) <= 0.5 + 1e-6);
    CHECK(std::fabs(frameExtent(r, f, true, true) - 26.0) <= 0.5 + 1e-6);
    // No yards: the rectangle fills the lot to within the raster.
    const Poly2 full = largestAlignedRect(lot, f, Yards{}, 0.5, 6.0);
    CHECK(area(full) > 0.90 * area(lot));
    // A lot too narrow for minSide yields nothing rather than a sliver.
    const Poly2 narrow = {{0, 0}, {5, 0}, {5, 30}, {0, 30}};
    CHECK(largestAlignedRect(narrow, siteFrame(narrow, {0, -1}), Yards{}, 0.5, 6.0).empty());
}

TEST_CASE(largest_aligned_rect_in_an_l_lot_takes_the_fat_leg) {
    // An L: 30 wide on the street, the back-right 15x15 missing.
    const Poly2 lot = {{0, 0}, {30, 0}, {30, 15}, {15, 15}, {15, 30}, {0, 30}};
    const SiteFrame f = siteFrame(lot, {0, -1});
    const Poly2 r = largestAlignedRect(lot, f, Yards{}, 0.5, 6.0);
    CHECK(r.size() == 4u);
    CHECK(allInside(r, lot));
    CHECK(rectilinearIn(r, f));
    // Both legs are 450 m²; the answer is one of them, not the 900 m² bbox.
    CHECK(area(r) > 380.0);
    CHECK(area(r) < 470.0);
}

TEST_CASE(open_space_pieces_tile_the_lot_around_the_building) {
    const Poly2 lot = {{0, 0}, {30, 0}, {25, 25}, {5, 25}};
    const Yards yards{2.0, 0.0, 3.0};
    const SitePlan sp = planSite(lot, {0, -1}, yards, 0.5, 6.0);
    CHECK(sp.buildable.size() == 4u);
    Real openArea = 0;
    int forecourts = 0, sides = 0, rears = 0;
    for (const OpenSpace& o : sp.open) {
        openArea += area(o.poly);
        if (o.kind == OpenKind::Forecourt) ++forecourts;
        if (o.kind == OpenKind::SideYard) ++sides;
        if (o.kind == OpenKind::RearYard) ++rears;
    }
    // Building + open ground = the lot (the cuts are exact on a convex lot).
    CHECK(std::fabs(area(sp.buildable) + openArea - area(lot)) < 0.02 * area(lot));
    CHECK(forecourts == 1);   // the front yard strip
    CHECK(rears == 1);        // the rear yard strip
    CHECK(sides == 2);        // the trapezoid's two tapered slivers
    // The frame's names mean what they say: the forecourt lies in front.
    for (const OpenSpace& o : sp.open) {
        const Real cy = sp.frame.toFrame(centroid(o.poly)).y;
        const Real by0 = frameExtent(sp.buildable, sp.frame, true, false);
        const Real by1 = frameExtent(sp.buildable, sp.frame, true, true);
        if (o.kind == OpenKind::Forecourt) CHECK(cy < by0);
        if (o.kind == OpenKind::RearYard) CHECK(cy > by1);
    }
    // Deterministic.
    const SitePlan again = planSite(lot, {0, -1}, yards, 0.5, 6.0);
    CHECK(again.buildable.size() == sp.buildable.size());
    for (std::size_t i = 0; i < again.buildable.size() && i < sp.buildable.size(); ++i)
        CHECK((again.buildable[i] - sp.buildable[i]).length() < 1e-9);
    CHECK(again.open.size() == sp.open.size());
}
