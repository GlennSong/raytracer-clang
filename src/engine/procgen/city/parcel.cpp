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

}  // namespace

std::vector<Lot> subdivideBlock(const Poly2& block, const ParcelParams& params,
                                int district) {
    std::vector<Lot> lots;
    if (block.size() < 3) return lots;
    Poly2 b = block;
    ensureCCW(b);

    Rng rng(params.seed ? params.seed : 1u);
    std::vector<Poly2> pieces;
    subdivide(b, params, rng, pieces, 0);

    Vec2 blockCenter = centroid(b);
    for (Poly2& piece : pieces) {
        ensureCCW(piece);
        Lot lot;
        lot.footprint = piece;
        lot.area = area(piece);
        lot.district = district;
        // Frontage heuristic (city-plan §3.3): a building faces away from the block
        // interior, toward the surrounding street ring.
        Vec2 dir = centroid(piece) - blockCenter;
        lot.frontage = (dir.lengthSquared() > 1e-6) ? normalize(dir) : Vec2(0, 1);
        lots.push_back(std::move(lot));
    }
    return lots;
}

}  // namespace engine
