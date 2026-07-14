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
bool isConvexCCW(const Poly2& poly) {
    const int n = static_cast<int>(poly.size());
    if (n < 3) return false;
    for (int i = 0; i < n; ++i) {
        const Vec2 a = poly[i], b = poly[(i + 1) % n], c = poly[(i + 2) % n];
        if (cross(b - a, c - b) < -1e-6) return false;   // a right turn => concave
    }
    return true;
}

// Concave-block parceling: a ring of street-fronting strips inset one lotDepth
// from each edge, plus an interior court. Trapezoidal, not rectangular — but
// every lot touches a block edge (a street), so NOTHING is landlocked. Used
// only for the rare non-convex arterial off-cut; convex blocks get the crisp
// rectangular row-split. This is what makes the parceler court-robust for ANY
// block, so the grid-cell cap can relax and big districts keep big blocks.
bool parcelRing(const Poly2& b, const ParcelParams& p, int district,
                std::vector<Lot>& out) {
    const int n = static_cast<int>(b.size());
    if (n < 3) return false;
    Poly2 I = inset(b, p.lotDepth);
    // inset must survive with the same edge count (edge i pairs with inset
    // edge i); a collapse/self-intersection means too-deep for this block.
    if (I.size() != static_cast<std::size_t>(n) || area(I) < 1.0) {
        // Shallow concave block: inset one HALF the depth so the ring still
        // forms, meeting near the middle (no court).
        I = inset(b, p.lotDepth * 0.5);
        if (I.size() != static_cast<std::size_t>(n) || area(I) < 1.0) return false;
    }
    ensureCCW(I);
    for (int i = 0; i < n; ++i) {
        const Vec2 B0 = b[i], B1 = b[(i + 1) % n];
        const Vec2 I0 = I[i], I1 = I[(i + 1) % n];
        const Real flen = (B1 - B0).length();
        if (flen < 1.0) continue;
        const Vec2 edir = normalize(B1 - B0);
        const Vec2 fwd(edir.y, -edir.x);   // outward (CCW: interior is left)
        const int k = std::max(1, static_cast<int>(std::lround(flen / p.frontWidth)));
        for (int j = 0; j < k; ++j) {
            const Real t0 = static_cast<Real>(j) / k, t1 = static_cast<Real>(j + 1) / k;
            Poly2 lotP{lerp(B0, B1, t0), lerp(B0, B1, t1),
                       lerp(I0, I1, t1), lerp(I0, I1, t0)};
            ensureCCW(lotP);
            if (area(lotP) < p.minArea * 0.4) continue;
            Lot lot;
            lot.footprint = std::move(lotP);
            lot.area = area(lot.footprint);
            lot.district = district;
            lot.frontage = fwd;
            out.push_back(std::move(lot));
        }
    }
    if (area(I) >= p.courtMinArea) {   // a real interior => a court
        Lot court;
        court.footprint = I;
        court.area = area(I);
        court.district = district;
        court.court = true;
        court.frontage = Vec2(0, 1);
        out.push_back(std::move(court));
    }
    return !out.empty();
}

bool parcelFrontage(const Poly2& b, const ParcelParams& p, Rng& rng, int district,
                    std::vector<Lot>& out) {
    if (b.size() < 3) return false;
    // The row-split clips cells by each block edge's half-plane, valid only for
    // a CONVEX face. Concave blocks take the ring method (also accessible) —
    // never the landlocking bisection.
    if (!isConvexCCW(b)) return parcelRing(b, p, district, out);
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
        if (area(piece) < (isCourt ? 1.0 : p.minArea * 0.5)) return;
        if (!isCourt) {
            // Reject THIN diagonal slivers a corner clip can leave — their
            // axis-aligned bbox looks fine but the real (OBB) short side is
            // knife-thin, and a building shrink-fit to one collapses to a
            // degenerate box far from its site.
            const OBB2 lo = orientedBoundingBox(piece);
            if (lo.half[1 - lo.longAxis()] * 2.0 < p.minEdge) return;
        }
        Lot lot;
        lot.area = area(piece);
        lot.footprint = std::move(piece);
        lot.district = district;
        lot.frontage = fdir;
        lot.court = isCourt;
        out.push_back(std::move(lot));
    };

    // v-bands of the rows (and the court) in OBB short-axis coordinates.
    struct Band { Real v0, v1; Vec2 fdir; bool court; };
    std::vector<Band> bands;
    if (rows == 1) {
        bands.push_back({-HS, HS, V, false});
    } else if (court) {
        bands.push_back({-HS, -HS + p.lotDepth, V * -1.0, false});   // front row
        bands.push_back({HS - p.lotDepth, HS, V, false});            // back row
        bands.push_back({-HS + p.lotDepth, HS - p.lotDepth, V, true}); // court
    } else {
        bands.push_back({-HS, 0, V * -1.0, false});
        bands.push_back({0, HS, V, false});
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
