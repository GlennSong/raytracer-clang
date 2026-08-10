#include "parcel_block.h"

#include "rng.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

Real totalArea(const std::vector<Shape2>& v) {
    Real a = 0;
    for (const Shape2& s : v) a += area(s);
    return a;
}

// Cut a shape by the infinite line through `p` along `dir`. Implemented with the
// boolean, so it works on any region — holes, arcs and all — rather than only on
// the convex rings a Sutherland-Hodgman clip can handle.
void splitShape(const Shape2& s, const Vec2& p, const Vec2& dir,
                std::vector<Shape2>& left, std::vector<Shape2>& right) {
    Vec2 lo, hi;
    bounds(s, lo, hi);
    const Real span = (hi - lo).length() + 10;
    const Vec2 d = normalize(dir);
    const Vec2 n(-d.y, d.x);
    // Two overlapping half-plane rectangles, each comfortably larger than the
    // shape, so the cut reaches all the way across whatever it is given.
    Poly2 a = {p - d * span, p + d * span, p + d * span + n * span,
               p - d * span + n * span};
    Poly2 b = {p - d * span, p + d * span, p + d * span - n * span,
               p - d * span - n * span};
    left = shapeBool({s}, {shapeFromPoly(a)}, BoolOp::Intersect);
    right = shapeBool({s}, {shapeFromPoly(b)}, BoolOp::Intersect);
}

// The longest run of street-tagged edge, and its direction — the frontage a
// split has to preserve.
bool frontageAxis(const Shape2& s, Vec2& along, Vec2& mid, Real& length) {
    length = 0;
    for (std::size_t i = 0; i < s.outer.size(); ++i) {
        if (s.outer.edges[i].tag != EdgeTag::Street) continue;
        const Vec2 a = s.outer.start(i), b = s.outer.end(i);
        const Real len = (b - a).length();
        if (len <= length) continue;
        length = len;
        along = normalize(b - a);
        mid = (a + b) * 0.5;
    }
    return length > 1e-6;
}

// Total street frontage of a region.
Real frontageLength(const Shape2& s) {
    Real f = 0;
    for (std::size_t i = 0; i < s.outer.size(); ++i)
        if (s.outer.edges[i].tag == EdgeTag::Street)
            f += (s.outer.end(i) - s.outer.start(i)).length();
    return f;
}

}  // namespace

Real ParcelledBlock::lotArea() const {
    Real a = 0;
    for (const ParcelledLot& l : lots) a += area(l.shape);
    return a;
}

Real ParcelledBlock::openArea() const { return totalArea(openSpace); }

// ---------------------------------------------------------------------------
// Edge tagging
// ---------------------------------------------------------------------------

void tagLotEdges(Shape2& lot, const Shape2& reference, Real partyMaxFront) {
    if (lot.outer.size() < 3 || reference.outer.size() < 3) return;

    // Distance from a point to the reference boundary, and WHICH reference edge
    // it landed on — the tag is inherited from that edge, so a block side
    // fronting a park does not turn into a street.
    auto nearestRefEdge = [&](const Vec2& p, Real& dist) {
        std::size_t best = 0;
        dist = 1e30;
        for (std::size_t i = 0; i < reference.outer.size(); ++i) {
            const Vec2 a = reference.outer.start(i), b = reference.outer.end(i);
            const Vec2 d = b - a;
            const Real len2 = d.lengthSquared();
            Real u = len2 > 1e-12 ? dot(p - a, d) / len2 : 0;
            u = std::max(Real(0), std::min(Real(1), u));
            const Real dd = (p - (a + d * u)).length();
            if (dd < dist) { dist = dd; best = i; }
        }
        return best;
    };

    // Pass 1: an edge whose whole length lies on the reference boundary faces
    // whatever that boundary faces.
    std::vector<char> isStreet(lot.outer.size(), 0);
    for (std::size_t i = 0; i < lot.outer.size(); ++i) {
        const Vec2 a = lot.outer.start(i), b = lot.outer.end(i);
        const Vec2 m = (a + b) * 0.5;
        Real da, db, dm;
        const std::size_t ref = nearestRefEdge(m, dm);
        nearestRefEdge(a, da);
        nearestRefEdge(b, db);
        if (std::max({da, db, dm}) >= 0.35) continue;
        const EdgeTag inherited = reference.outer.edges[ref].tag;
        lot.outer.edges[i].tag =
            inherited == EdgeTag::None ? EdgeTag::Street : inherited;
        if (lot.outer.edges[i].tag == EdgeTag::Street) isStreet[i] = 1;
    }

    const Real front = frontageLength(lot);
    // Pass 2: of the remaining edges, the one whose midpoint is FURTHEST from
    // any street edge is the rear; the rest are sides. On a lot too narrow to
    // leave a gap between neighbours, those sides are party walls — blank by
    // construction, which is what stops a terrace's windows looking into next
    // door's bedroom.
    Real worst = -1;
    std::size_t rear = lot.outer.size();
    for (std::size_t i = 0; i < lot.outer.size(); ++i) {
        if (isStreet[i]) continue;
        const Vec2 m = (lot.outer.start(i) + lot.outer.end(i)) * 0.5;
        Real nearestStreet = 1e30;
        for (std::size_t j = 0; j < lot.outer.size(); ++j) {
            if (!isStreet[j]) continue;
            const Vec2 a = lot.outer.start(j), b = lot.outer.end(j);
            const Vec2 d = b - a;
            const Real len2 = d.lengthSquared();
            Real u = len2 > 1e-12 ? dot(m - a, d) / len2 : 0;
            u = std::max(Real(0), std::min(Real(1), u));
            nearestStreet = std::min(nearestStreet, (m - (a + d * u)).length());
        }
        if (nearestStreet > worst) { worst = nearestStreet; rear = i; }
    }
    const bool terraced = front > 1e-6 && front <= partyMaxFront * 1.35;
    for (std::size_t i = 0; i < lot.outer.size(); ++i) {
        if (isStreet[i]) continue;
        if (i == rear) {
            lot.outer.edges[i].tag = EdgeTag::Rear;
        } else {
            lot.outer.edges[i].tag = terraced ? EdgeTag::Party : EdgeTag::Side;
        }
    }
    // A lot with NO street edge at all is an interior court parcel; nothing
    // fronts a road, so nothing should claim to.
    if (front <= 1e-6)
        for (Edge2& e : lot.outer.edges)
            if (e.tag == EdgeTag::Street) e.tag = EdgeTag::Court;
}

// ---------------------------------------------------------------------------
// The recursive cut
// ---------------------------------------------------------------------------

namespace {

struct Cutter {
    ProgramSet* programs = nullptr;
    const ParcelParams* params = nullptr;
    const Shape2* reference = nullptr;   // the PARCELLABLE region, not the block
    bool enclosed = true;
    Real coreness = 0;
    StreetClass klass = StreetClass::Street;
    Rng rng{1};
    std::vector<ParcelledLot> lots;
    std::vector<Shape2> open;

    // What this region is FOR, and therefore how finely to cut it. Sizing the
    // split by the smallest eligible program instead would let the smallest
    // entry in a mix drive every decision, and a rim block with a cottage in
    // its programme list would be cut into cottages.
    const LotProgram* target(const LotTags& tags) const {
        const int i = programs->bestFor(tags);
        return i >= 0 ? &programs->programs[i] : nullptr;
    }

    void cut(Shape2 region, int depth) {
        if (region.outer.size() < 3 || area(region) < params->minLotArea) {
            if (region.outer.size() >= 3) open.push_back(std::move(region));
            return;
        }
        tagLotEdges(region, *reference, params->partyMaxFront);
        LotTags tags = measureLot(region, {}, klass, enclosed, coreness);

        std::vector<std::pair<int, Real>> el = programs->eligible(tags);
        if (el.empty()) {
            // NOTHING can stand here. That is a legitimate outcome — open space
            // by design — not a rejection to be counted.
            open.push_back(std::move(region));
            return;
        }
        if (depth >= params->maxDepth) {
            emit(std::move(region), tags);
            return;
        }

        // Would splitting leave two halves that can still host something? Ask
        // BEFORE cutting, which is the whole inversion in one line.
        Vec2 along, mid;
        Real frontLen = 0;
        const bool hasFront = frontageAxis(region, along, mid, frontLen);
        const LotProgram* want = target(tags);
        const Real useful = want ? std::min(want->minW, want->minD) : 8.0;
        // Stop splitting once the land is close to what the target actually
        // needs — cutting a campus parcel in half again serves nothing.
        const bool roomToSplit = !want || area(region) > want->minArea * 2.0;

        if (hasFront && roomToSplit && frontLen > useful * 2.2) {
            // FRONTAGE-FIRST: cut perpendicular to the street so both halves
            // keep a frontage. A split parallel to it would leave the back half
            // landlocked, which is how a parceller produces lots with no way in.
            const Vec2 cutDir(-along.y, along.x);
            const Real jitter = rng.range(0.42, 0.58);
            const Vec2 at = mid + along * (frontLen * (jitter - 0.5));
            std::vector<Shape2> a, b;
            splitShape(region, at, cutDir, a, b);
            if (!a.empty() && !b.empty()) {
                for (Shape2& s : a) cut(std::move(s), depth + 1);
                for (Shape2& s : b) cut(std::move(s), depth + 1);
                return;
            }
        }

        // DEPTH: a region far deeper than anything eligible wants gets a front
        // row and a back region. The back may be landlocked, in which case the
        // recursion finds no eligible program for it and it becomes the block's
        // interior green — which is exactly what a real block has.
        Vec2 lo, hi;
        bounds(region, lo, hi);
        const Real deep = std::max(hi.x - lo.x, hi.y - lo.y);
        const Real wantDepth =
            want ? std::max(want->minW, want->minD) : 14.0;
        if (hasFront && roomToSplit && deep > wantDepth * 2.1) {
            const Vec2 inward(-along.y, along.x);
            const Vec2 at = mid + inward * (wantDepth * rng.range(1.05, 1.35));
            std::vector<Shape2> a, b;
            splitShape(region, at, along, a, b);
            if (!a.empty() && !b.empty()) {
                for (Shape2& s : a) cut(std::move(s), depth + 1);
                for (Shape2& s : b) cut(std::move(s), depth + 1);
                return;
            }
        }
        emit(std::move(region), tags);
    }

    void emit(Shape2 region, const LotTags& tags) {
        ParcelledLot lot;
        lot.shape = std::move(region);
        lot.tags = tags;
        lot.program = programs->pick(tags, rng.next());
        if (lot.program < 0) {
            open.push_back(std::move(lot.shape));
            return;
        }
        lots.push_back(std::move(lot));
    }
};

}  // namespace

ParcelledBlock parcelBlock(const Shape2& block, ProgramSet& programs,
                           const ParcelParams& params, bool enclosed,
                           Real coreness, StreetClass klass,
                           std::uint32_t seed) {
    ParcelledBlock out;
    out.block = block;
    if (block.outer.size() < 3) return out;

    // A block is bounded by roads on every side, so every boundary edge faces
    // the street. Saying so once here is what lets every later layer stop
    // guessing.
    Shape2 tagged = block;
    for (Edge2& e : tagged.outer.edges) e.tag = EdgeTag::Street;

    std::vector<Shape2> inner = offsetShape(tagged, -params.roadMargin);
    if (inner.empty()) {
        out.parcellable = Shape2{};
        out.openSpace.push_back(block);
        return out;
    }
    // The margin can split a thin block into several pieces; each is parcelled
    // in its own right.
    Cutter c;
    c.programs = &programs;
    c.params = &params;
    c.enclosed = enclosed;
    c.coreness = coreness;
    c.klass = klass;
    c.rng = Rng(seed ? seed : 5u);
    Real best = 0;
    for (const Shape2& piece : inner)
        if (area(piece) > best) { best = area(piece); out.parcellable = piece; }
    // Each piece is its OWN reference: a lot cut from it fronts the street
    // across the margin wherever it touches that piece's boundary. The offset
    // carried the block's edge tags through, so a non-street block side stays a
    // non-street lot side.
    for (const Shape2& piece : inner) {
        c.reference = &piece;
        c.cut(piece, 0);
    }
    out.lots = std::move(c.lots);
    out.openSpace = std::move(c.open);
    return out;
}

}  // namespace engine
