#include "parcel.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace {

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 0x2545f491u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    Real unit() { return (next() >> 8) * (1.0 / 16777216.0); }
};

// Recursively bisect `poly` along the long OBB axis. Appends finished lots.
void subdivide(const Poly2& poly, const ParcelParams& p, Rng& rng,
               std::vector<Poly2>& out, int depth) {
    Real a = area(poly);
    if (poly.size() < 3 || a < p.minArea || depth > 24) {
        if (a >= 1.0) out.push_back(poly);
        return;
    }
    OBB2 obb = orientedBoundingBox(poly);
    int la = obb.longAxis();
    Real longHalf = obb.half[la];
    Real shortHalf = obb.half[1 - la];

    // Stop when both the area is near target and the lot isn't over-long, or when
    // a further split would create a sliver thinner than minEdge.
    bool smallEnough = a <= p.targetArea;
    bool tooThinToSplit = (longHalf < p.minEdge) || (shortHalf * 2 < p.minEdge);
    if (smallEnough || tooThinToSplit) {
        out.push_back(poly);
        return;
    }

    // Split perpendicular to the long axis, through the center with a little jitter,
    // so we cut the lot into two shorter halves that still front the same streets.
    Vec2 dirLong = obb.axis[la];
    Real offset = (rng.unit() - 0.5) * 2.0 * p.jitter * longHalf;
    Vec2 splitPoint = obb.center + dirLong * offset;
    // The cut line runs along the short axis (perpendicular to the long axis).
    Vec2 cutDir = obb.axis[1 - la];

    Poly2 left, right;
    splitByLine(poly, splitPoint, cutDir, left, right);
    if (left.size() < 3 || right.size() < 3) {     // degenerate cut: keep whole
        out.push_back(poly);
        return;
    }
    subdivide(left, p, rng, out, depth + 1);
    subdivide(right, p, rng, out, depth + 1);
}

// Clip a convex cell polygon to a (convex) block by each block edge's inward
// half-plane. Grid sub-blocks are convex, so the result stays a clean convex
// piece (a rectangle where the cell is interior, trimmed at the block edge).
Poly2 clipToBlock(const Poly2& cell, const Poly2& block) {
    Poly2 piece = cell;
    const int n = static_cast<int>(block.size());
    for (int i = 0; i < n && piece.size() >= 3; ++i) {
        const Vec2 a = block[i], b = block[(i + 1) % n];
        const Vec2 d = b - a;
        const Vec2 outward(d.y, -d.x);   // CCW block: interior is left of a->b
        piece = clipHalfPlane(piece, outward, outward.x * a.x + outward.y * a.y);
    }
    return piece;
}

// Frontage-first parceling (city-pipeline v2 step 10, RECTANGULAR): lay lots as
// rows down the block's short axis (each row fronts a long street), each row
// divided into ~frontWidth rectangles down the long axis — the classic
// back-to-back city block. A block deep enough for a middle court keeps two
// edge rows + an interior court (reachable from the cross streets at the block
// ends); otherwise the two rows meet in the middle. Rectangles are OBB-aligned
// and clipped to the block, so they read as real lots, not trapezoids.
// A CCW polygon is convex when every consecutive turn is a left turn.
bool parcelFrontage(const Poly2& b, const ParcelParams& p, Rng& rng, int district,
                    std::vector<Lot>& out) {
    if (b.size() < 3) return false;
    const OBB2 o = orientedBoundingBox(b);
    const int la = o.longAxis(), sa = 1 - la;
    const Vec2 U = o.axis[la];    // long axis (rows run along this)
    const Vec2 V = o.axis[sa];    // short axis (rows stack along this)
    const Real HL = o.half[la], HS = o.half[sa];
    if (HL < 5.0 || HS < 5.0) return false;

    // Rows across the short axis. Deep block: two lotDepth rows + a court
    // between them; otherwise two rows meeting in the middle (or one if thin).
    const bool court = 2.0 * HS > 2.0 * p.lotDepth + 22.0;
    const int rows = (2.0 * HS < 1.35 * p.lotDepth) ? 1 : 2;
    const int cols = std::max(1, static_cast<int>(std::lround(2.0 * HL / p.frontWidth)));

    auto emitCell = [&](Real u0, Real u1, Real v0, Real v1, const Vec2& fdir,
                        bool isCourt) {
        Poly2 rect{o.center + U * u0 + V * v0, o.center + U * u1 + V * v0,
                   o.center + U * u1 + V * v1, o.center + U * u0 + V * v1};
        Poly2 piece = clipToBlock(rect, b);
        if (piece.size() < 3) return;
        ensureCCW(piece);
        const Real full = std::fabs((u1 - u0) * (v1 - v0));   // U,V orthonormal
        const Real a = area(piece);
        if (isCourt) {
            if (a < 1.0) return;   // the central plaza: whatever interior remains
        } else {
            // AABB-ONLY (device: "AABB lots ... no trapezoids; the empty
            // middle is a plaza"): keep a lot ONLY where the rectangular cell
            // clips to a (near-)full rectangle. A significantly clipped cell
            // would be a trapezoid, so it's left as open ground that reads as
            // setback/plaza rather than a wedge lot. Emit the clean RECT (not
            // the clipped piece) so the lot is a true axis-aligned rectangle.
            if (a < 0.90 * full) return;
            if (!pointInPolygon(b, o.center + U * ((u0 + u1) * 0.5) +
                                       V * ((v0 + v1) * 0.5)))
                return;
            piece = rect;
        }
        Lot lot;
        lot.area = area(piece);
        lot.footprint = std::move(piece);
        lot.district = district;
        lot.frontage = fdir;
        lot.court = isCourt;
        out.push_back(std::move(lot));
    };

    // v-bands: two street-fronting rows + a central PLAZA between them (device
    // model). Rows shrink to keep a real plaza even on a medium block; a thin
    // block gets a single row and no plaza.
    struct Band { Real v0, v1; Vec2 fdir; bool court; };
    std::vector<Band> bands;
    (void)court;
    if (rows == 1) {
        bands.push_back({-HS, HS, V, false});
    } else {
        const Real rd = std::min(p.lotDepth, HS * 0.82);   // >= ~36% stays plaza
        bands.push_back({-HS, -HS + rd, V * -1.0, false});   // front row
        bands.push_back({HS - rd, HS, V, false});            // back row
        bands.push_back({-HS + rd, HS - rd, V, true});       // central plaza
    }
    for (const Band& bd : bands) {
        if (bd.court) { emitCell(-HL, HL, bd.v0, bd.v1, V, true); continue; }
        for (int c = 0; c < cols; ++c) {
            const Real u0 = -HL + 2.0 * HL * c / cols;
            const Real u1 = -HL + 2.0 * HL * (c + 1) / cols;
            emitCell(u0, u1, bd.v0, bd.v1, bd.fdir, false);
        }
    }
    (void)rng;
    return !out.empty();
}

}  // namespace

std::vector<Lot> subdivideBlock(const Poly2& block, const ParcelParams& params,
                                int district) {
    std::vector<Lot> lots;
    if (block.size() < 3) return lots;
    Poly2 b = block;
    ensureCCW(b);

    Rng rng(params.seed ? params.seed : 1u);

    // FRONTAGE-FIRST (v2 step 10): the primary path — every lot faces a street,
    // the deep core becomes a reachable court. Falls back to bisection for
    // shallow/degenerate blocks (which bisection still street-fronts, being
    // thin enough that both halves reach opposite streets).
    if (parcelFrontage(b, params, rng, district, lots)) return lots;

    std::vector<Poly2> pieces;
    subdivide(b, params, rng, pieces, 0);
    Vec2 blockCenter = centroid(b);
    for (Poly2& piece : pieces) {
        ensureCCW(piece);
        Lot lot;
        lot.footprint = piece;
        lot.area = area(piece);
        lot.district = district;
        Vec2 dir = centroid(piece) - blockCenter;
        lot.frontage = (dir.lengthSquared() > 1e-6) ? normalize(dir) : Vec2(0, 1);
        lots.push_back(std::move(lot));
    }
    return lots;
}

}  // namespace engine
