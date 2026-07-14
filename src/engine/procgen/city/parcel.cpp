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

// Frontage-first parceling (city-pipeline v2 step 10): cut a ring of
// street-fronting lots inward from the block edges; the leftover core becomes a
// court. Returns false (caller falls back to bisection) when the block is too
// shallow to leave a court, or the inset is degenerate.
bool parcelFrontage(const Poly2& b, const ParcelParams& p, Rng& rng, int district,
                    std::vector<Lot>& out) {
    const int n = static_cast<int>(b.size());
    if (n < 3) return false;
    Poly2 I = inset(b, p.lotDepth);
    // The inset must survive with the SAME edge count (so block edge i pairs
    // with inset edge i) and leave a real interior — otherwise it's a shallow
    // block, handled by the bisection fallback.
    if (I.size() != static_cast<std::size_t>(n) || area(I) < p.courtMinArea)
        return false;
    ensureCCW(I);   // inset() preserves winding, but normalize to be safe

    // Pick ONE alley: a gap in the longest street strip so the court is
    // reachable (city-pipeline v2 "reachable court"). Skip the middle lot of
    // that strip.
    int alleyEdge = 0;
    Real longest = 0;
    for (int i = 0; i < n; ++i) {
        Real l = (b[(i + 1) % n] - b[i]).length();
        if (l > longest) { longest = l; alleyEdge = i; }
    }

    for (int i = 0; i < n; ++i) {
        const Vec2 B0 = b[i], B1 = b[(i + 1) % n];
        const Vec2 I0 = I[i], I1 = I[(i + 1) % n];
        const Real flen = (B1 - B0).length();
        if (flen < 1.0) continue;
        // Outward frontage normal (CCW block: interior is left of B0->B1, so
        // outward is the right normal).
        Vec2 edir = normalize(B1 - B0);
        Vec2 fwd(edir.y, -edir.x);
        const int k = std::max(1, static_cast<int>(std::lround(flen / p.frontWidth)));
        const int alleyLot = (i == alleyEdge && k >= 3) ? k / 2 : -1;
        for (int j = 0; j < k; ++j) {
            if (j == alleyLot) continue;   // the alley gap
            const Real t0 = static_cast<Real>(j) / k;
            const Real t1 = static_cast<Real>(j + 1) / k;
            Poly2 lotP{lerp(B0, B1, t0), lerp(B0, B1, t1),
                       lerp(I0, I1, t1), lerp(I0, I1, t0)};
            ensureCCW(lotP);
            if (area(lotP) < 1.0) continue;
            Lot lot;
            lot.footprint = std::move(lotP);
            lot.area = area(lot.footprint);
            lot.district = district;
            lot.frontage = fwd;
            out.push_back(std::move(lot));
        }
    }
    // The court: the interior polygon, flagged non-buildable.
    Lot court;
    court.footprint = I;
    court.area = area(I);
    court.district = district;
    court.court = true;
    court.frontage = Vec2(0, 1);
    out.push_back(std::move(court));
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
