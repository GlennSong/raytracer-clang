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

// First boundary hit of the ray o + t*dir (t in (tMin, inf)) against the
// polygon's edges. Returns tMin-clamped infinity when nothing is hit.
Real rayExit(const Poly2& poly, const Vec2& o, const Vec2& dir, Real tMin) {
    Real best = Real(1e30);
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        const Vec2 a = poly[i], b = poly[(i + 1) % n];
        const Vec2 e = b - a;
        const Real den = cross(dir, e);
        if (std::fabs(den) < Real(1e-9)) continue;
        const Real t = cross(a - o, e) / den;   // along the ray
        const Real u = cross(a - o, dir) / den; // along the edge
        if (t > tMin && t < best && u >= -1e-6 && u <= 1.0 + 1e-6) best = t;
    }
    return best;
}

// SAT overlap test for two convex polygons; touching (within eps) is NOT
// overlap, so lots that share an edge line pass.
bool convexOverlap(const Poly2& A, const Poly2& B, Real eps) {
    for (const Poly2* P : {&A, &B}) {
        const int n = static_cast<int>(P->size());
        for (int i = 0; i < n; ++i) {
            Vec2 e = (*P)[(i + 1) % n] - (*P)[i];
            Real l = e.length();
            if (l < Real(1e-9)) continue;
            const Vec2 ax(-e.y / l, e.x / l);
            Real lo0 = 1e30, hi0 = -1e30, lo1 = 1e30, hi1 = -1e30;
            for (const Vec2& v : A) {
                const Real t = dot(ax, v);
                lo0 = std::min(lo0, t); hi0 = std::max(hi0, t);
            }
            for (const Vec2& v : B) {
                const Real t = dot(ax, v);
                lo1 = std::min(lo1, t); hi1 = std::max(hi1, t);
            }
            if (hi0 <= lo1 + eps || hi1 <= lo0 + eps) return false;
        }
    }
    return true;
}

// How much of the candidate's ORIGINAL front line (through `a` along `d`,
// inward normal `nrm`) the piece still touches: the span along `d` of piece
// vertices within 0.6 m of the front line. A clip that strips the front would
// landlock the lot, so callers reject pieces whose span drops too low.
Real frontSpan(const Poly2& piece, const Vec2& a, const Vec2& d, const Vec2& nrm) {
    Real lo = 1e30, hi = -1e30;
    for (const Vec2& v : piece) {
        if (dot(v - a, nrm) > 0.6) continue;
        const Real t = dot(v - a, d);
        lo = std::min(lo, t);
        hi = std::max(hi, t);
    }
    return hi > lo ? hi - lo : Real(0);
}

// Frontage-first parceling, BOUNDARY-WALK edition (Glenn's density round): walk
// the block polygon's ACTUAL edges and lay a ring of lots along each one —
// ~frontWidth of street frontage, lotDepth deep, extending along the edge's
// inward normal. Works for ANY simple polygon (convex, trapezoidal, concave):
// every lot fronts a real boundary edge BY CONSTRUCTION.
//   * corners: candidates are clipped by the corner BISECTOR half-planes
//     (nearest-edge wins), so adjacent edges' lots miter into each other
//     instead of overlapping — the classic corner-lot cut;
//   * depth: inward rays from the front clamp the depth where the polygon is
//     shallow — a narrow block gets one full-depth row (first edge wins), a
//     medium one an even split, a deep one lotDepth rows;
//   * backstop: any candidate still overlapping an already-placed lot is
//     clipped to the placed lot's far side (largest surviving piece that keeps
//     real frontage) or dropped — lots never overlap;
//   * the interior remainder becomes ONE court lot (inset by lotDepth), the
//     sculpted mid-block green downstream.
// Deterministic: edges walk in polygon order, one rng draw per edge.
bool parcelFrontage(const Poly2& b, const ParcelParams& p, Rng& rng, int district,
                    std::vector<Lot>& out) {
    const int n = static_cast<int>(b.size());
    if (n < 3) return false;
    const Real fw = std::max(p.frontWidth, Real(4.0));
    const Real ld = std::max(p.lotDepth, Real(6.0));
    const Real minDepth = std::max(p.minEdge, Real(0.55) * ld);
    const Real minFront = std::max(p.minEdge, Real(0.35) * fw);

    std::vector<Vec2> dir(n), inr(n);
    std::vector<Real> len(n);
    for (int i = 0; i < n; ++i) {
        const Vec2 e = b[(i + 1) % n] - b[i];
        len[i] = e.length();
        dir[i] = len[i] > Real(1e-9) ? e / len[i] : Vec2(1, 0);
        inr[i] = Vec2(-dir[i].y, dir[i].x);   // CCW: interior left of a->b
    }

    std::vector<Poly2> placed;   // convex lot polys, for the overlap backstop
    const std::size_t firstLot = out.size();
    for (int ei = 0; ei < n; ++ei) {
        const Real L = len[ei];
        if (L < Real(0.55) * fw) { rng.unit(); continue; }   // keep the stream
        const Vec2 A = b[ei], B = b[(ei + 1) % n];
        const Vec2 d = dir[ei], nrm = inr[ei];
        // Corner bisectors (only at CONVEX corners — at a reflex corner the
        // neighbouring strips diverge and need no miter).
        const int pi2 = (ei + n - 1) % n, ni = (ei + 1) % n;
        const bool miterA = cross(dir[pi2], d) > Real(1e-6);
        const bool miterB = cross(d, dir[ni]) > Real(1e-6);
        const Vec2 mnA = nrm - inr[pi2];       // keep dot(q-A, mnA) <= 0
        const Vec2 mnB = nrm - inr[ni];        // keep dot(q-B, mnB) <= 0
        // Slot the edge into ~frontWidth lots (a light per-edge width jitter
        // so streets don't read as one metronome rhythm).
        const Real wj = fw * (Real(1) + (rng.unit() - Real(0.5)) * Real(0.25));
        const int slots = std::max(1, static_cast<int>(std::lround(L / wj)));
        const Real w = L / slots;
        for (int k = 0; k < slots; ++k) {
            const Vec2 f0 = A + d * (w * k);
            const Vec2 f1 = A + d * (w * (k + 1));
            // Clamp depth by casting inward rays from the front: how far the
            // block actually extends behind this frontage.
            Real dmax = Real(1e30);
            const int ns = std::max(3, static_cast<int>(w / 8.0) + 1);
            for (int s = 0; s < ns; ++s) {
                const Vec2 q = f0 + d * (w * (s + 0.5) / ns);
                dmax = std::min(dmax, rayExit(b, q, nrm, Real(0.05)));
            }
            if (dmax > Real(1e29)) dmax = ld;
            // Narrow span (a rim rectangle): ONE full-depth row — the first
            // edge wins, the opposite row clips away. Anything wider splits
            // EVENLY (capped at lotDepth), so facing rows meet in the middle
            // and the authored lotDepth grain survives the clamp — a block
            // 2x lotDepth deep parcels 2x-deep lots, not one deep row plus
            // clipped remnants.
            Real depth = dmax < Real(1.35) * ld
                             ? std::min(ld, dmax - Real(0.4))
                             : std::min(ld, Real(0.5) * dmax);
            Poly2 piece;
            for (int attempt = 0; attempt < 3; ++attempt, depth *= Real(0.75)) {
                if (depth < minDepth) { piece.clear(); break; }
                piece = {f0, f1, f1 + nrm * depth, f0 + nrm * depth};
                if (miterA && mnA.length() > Real(1e-6))
                    piece = clipHalfPlane(piece, mnA, dot(mnA, A));
                if (miterB && mnB.length() > Real(1e-6) && piece.size() >= 3)
                    piece = clipHalfPlane(piece, mnB, dot(mnB, B));
                if (piece.size() < 3 ||
                    frontSpan(piece, A, d, nrm) < minFront) {
                    piece.clear();
                    break;   // mitered away at a sharp corner: no lot here
                }
                // Backstop: clip clear of every already-placed lot. Clipping
                // to the OUTER side of one placed edge guarantees disjointness
                // (both are convex); keep the largest piece that still fronts.
                bool ok = true;
                for (const Poly2& q : placed) {
                    if (!convexOverlap(piece, q, Real(0.05))) continue;
                    Poly2 best;
                    Real bestA = -1;
                    const int qn = static_cast<int>(q.size());
                    for (int j = 0; j < qn; ++j) {
                        const Vec2 qa = q[j];
                        const Vec2 qe = q[(j + 1) % qn] - qa;
                        const Vec2 outw(qe.y, -qe.x);   // CCW: outward right
                        Poly2 cut = clipHalfPlane(
                            piece, Vec2(-outw.x, -outw.y),
                            -(outw.x * qa.x + outw.y * qa.y));
                        if (cut.size() < 3) continue;
                        if (frontSpan(cut, A, d, nrm) < minFront) continue;
                        const Real ar = area(cut);
                        if (ar > bestA) { bestA = ar; best = std::move(cut); }
                    }
                    if (bestA <= 0) { ok = false; break; }
                    piece = std::move(best);
                }
                if (!ok) { piece.clear(); break; }
                // Containment: a concave block can still cut across the rear
                // corners the rays missed — retry shallower, then give up.
                const Vec2 pc = centroid(piece);
                bool inside = true;
                for (const Vec2& v : piece)
                    if (!pointInPolygon(b, v + (pc - v) * Real(0.02))) {
                        inside = false;
                        break;
                    }
                if (inside) break;
                piece.clear();
            }
            if (piece.size() < 3) continue;
            if (area(piece) < std::max(p.minArea, Real(25))) continue;
            // EFFECTIVE depth after the miter/backstop clips: a piece whose
            // area-per-frontage collapsed (the shallow band squeezed between
            // two full rows) is a back-alley strip, not a lot — leaving it
            // out keeps the parcel grain honest (clipped remnants used to
            // drag the whole distribution down).
            {
                const Real fs = frontSpan(piece, A, d, nrm);
                if (fs < Real(0.5) || area(piece) / fs < minDepth * Real(0.6))
                    continue;
            }
            ensureCCW(piece);
            Lot lot;
            lot.area = area(piece);
            lot.footprint = piece;
            lot.district = district;
            lot.frontage = Vec2(-nrm.x, -nrm.y);   // outward: toward the street
            lot.court = false;
            placed.push_back(std::move(piece));
            out.push_back(std::move(lot));
        }
    }
    if (out.size() == firstLot) return false;   // walk failed: caller bisects

    // The interior remainder: ONE court lot (the sculpted mid-block green).
    // inset(lotDepth) sits behind every ring lot by construction; when it
    // collapses (shallow/odd block) the rows already meet, so no court.
    Poly2 court = inset(b, ld);
    if (court.size() >= 3 && area(court) >= Real(40)) {
        Lot lot;
        lot.area = area(court);
        ensureCCW(court);
        lot.footprint = std::move(court);
        lot.district = district;
        lot.frontage = Vec2(0, 1);
        lot.court = true;
        out.push_back(std::move(lot));
    }
    return true;
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
