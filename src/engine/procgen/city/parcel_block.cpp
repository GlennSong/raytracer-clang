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

// Is this edge a way in? A LANE counts: a mews lot fronts a lane, and a
// frontage-first split has to preserve lane frontage exactly as it preserves
// street frontage. Counting only streets leaves a lane-served strip reporting
// no frontage at all, so it is never split and ships as one 150 m slab.
bool isFrontage(EdgeTag t) {
    return t == EdgeTag::Street || t == EdgeTag::Lane;
}

// The longest run of frontage edge, and its direction — what a split preserves.
bool frontageAxis(const Shape2& s, Vec2& along, Vec2& mid, Real& length) {
    length = 0;
    for (std::size_t i = 0; i < s.outer.size(); ++i) {
        if (!isFrontage(s.outer.edges[i].tag)) continue;
        const Vec2 a = s.outer.start(i), b = s.outer.end(i);
        const Real len = (b - a).length();
        if (len <= length) continue;
        length = len;
        along = normalize(b - a);
        mid = (a + b) * 0.5;
    }
    return length > 1e-6;
}

// Total frontage of a region, street and lane together.
Real frontageLength(const Shape2& s) {
    Real f = 0;
    for (std::size_t i = 0; i < s.outer.size(); ++i)
        if (isFrontage(s.outer.edges[i].tag))
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

Real ParcelledBlock::laneArea() const {
    Real a = 0;
    for (const BlockLane& l : lanes) a += area(l.area);
    return a;
}

bool ParcelledBlock::openSpaceIsReachable(Real minArea, Real tol) const {
    // A shared green nobody can walk into is a bug, not a park. Reachable means
    // it touches the block's own boundary (the street) or any lane/passage.
    for (const Shape2& g : openSpace) {
        if (area(g) < std::max(Real(1.0), minArea)) continue;
        bool ok = false;
        const Poly2 ring = tessellate(g.outer, 0.5);
        auto touches = [&](const Shape2& other) {
            const Poly2 o = tessellate(other.outer, 0.5);
            for (const Vec2& p : ring)
                for (std::size_t i = 0; i < o.size(); ++i) {
                    const Vec2 a = o[i], b = o[(i + 1) % o.size()];
                    const Vec2 d = b - a;
                    const Real len2 = d.lengthSquared();
                    Real u = len2 > 1e-12 ? dot(p - a, d) / len2 : 0;
                    u = std::max(Real(0), std::min(Real(1), u));
                    if ((p - (a + d * u)).length() < tol) return true;
                }
            return false;
        };
        if (parcellable.outer.size() >= 3 && touches(parcellable)) ok = true;
        for (const BlockLane& l : lanes)
            if (!ok && l.area.outer.size() >= 3 && touches(l.area)) ok = true;
        if (!ok) return false;
    }
    return true;
}

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
    // whatever that boundary faces. `settled` records that pass 2 must leave it
    // alone — marking only STREETS here lets pass 2 overwrite an inherited LANE
    // with Rear/Side, which silently strips a mews strip of all its frontage
    // and ships it as one 273 m slab.
    std::vector<char> isStreet(lot.outer.size(), 0);
    std::vector<char> settled(lot.outer.size(), 0);
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
        settled[i] = 1;
        if (lot.outer.edges[i].tag == EdgeTag::Street) isStreet[i] = 1;
    }

    // For the party-wall decision, only STREET frontage counts: a mews lot with
    // a long lane face is not a terrace.
    Real front = 0;
    for (std::size_t i = 0; i < lot.outer.size(); ++i)
        if (lot.outer.edges[i].tag == EdgeTag::Street)
            front += (lot.outer.end(i) - lot.outer.start(i)).length();
    // Pass 2: of the remaining edges, the one whose midpoint is FURTHEST from
    // any street edge is the rear; the rest are sides. On a lot too narrow to
    // leave a gap between neighbours, those sides are party walls — blank by
    // construction, which is what stops a terrace's windows looking into next
    // door's bedroom.
    Real worst = -1;
    std::size_t rear = lot.outer.size();
    for (std::size_t i = 0; i < lot.outer.size(); ++i) {
        if (settled[i]) continue;
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
        if (settled[i]) continue;
        if (i == rear) {
            lot.outer.edges[i].tag = EdgeTag::Rear;
        } else {
            lot.outer.edges[i].tag = terraced ? EdgeTag::Party : EdgeTag::Side;
        }
    }
    // A lot with no frontage of ANY kind is an interior court parcel; nothing
    // fronts a road or lane, so nothing should claim to.
    if (frontageLength(lot) <= 1e-6)
        for (Edge2& e : lot.outer.edges)
            if (e.tag == EdgeTag::Street) e.tag = EdgeTag::Court;
}

// ---------------------------------------------------------------------------
// The recursive cut
// ---------------------------------------------------------------------------

namespace {

struct Cutter {
    ProgramSet* programs = nullptr;
    const BlockParcelParams* params = nullptr;
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

    // Does every piece have a street or lane edge once tagged? Access is what
    // makes a region a lot rather than a void.
    bool hasAccess(const std::vector<Shape2>& pieces) const {
        for (const Shape2& p : pieces) {
            if (area(p) < params->minLotArea) continue;
            Shape2 probe = p;
            tagLotEdges(probe, *reference, params->partyMaxFront);
            Real access = 0;
            for (std::size_t i = 0; i < probe.outer.size(); ++i) {
                const EdgeTag t = probe.outer.edges[i].tag;
                if (t == EdgeTag::Street || t == EdgeTag::Lane)
                    access += (probe.outer.end(i) - probe.outer.start(i)).length();
            }
            if (access < 1.0) return false;
        }
        return true;
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
        // Stop splitting once the land is close to the lot the target is BUILT
        // on — cutting a campus parcel in half again serves nothing.
        const bool roomToSplit = !want || area(region) > want->targetArea() * 1.5;

        if (hasFront && roomToSplit && frontLen > useful * 2.2) {
            // FRONTAGE-FIRST: cut perpendicular to the street so both halves
            // keep a frontage. A split parallel to it would leave the back half
            // landlocked, which is how a parceller produces lots with no way in.
            const Vec2 cutDir(-along.y, along.x);
            // STATION the cut on whole lots rather than halving. Splitting near
            // the middle makes every lot width `frontage / 2^k`, so the grain is
            // a power-of-two artefact of the block's dimensions — a wider block
            // does not give wider lots, it gives one more halving. Rounding the
            // frontage to a whole number of target-width lots and cutting on a
            // lot boundary makes both halves whole, so the recursion bottoms out
            // at the width the program actually asked for.
            const Real targetFront = want ? want->targetWidth() : useful * 1.35;
            const int lots = std::max(2, static_cast<int>(
                                             std::lround(frontLen / targetFront)));
            // Off-centre by up to one lot, so a street does not read as though
            // every block were folded down the middle.
            const int cutAt = std::max(1, std::min(lots - 1,
                                                   lots / 2 + rng.irange(-1, 1)));
            const Real frac = static_cast<Real>(cutAt) / lots;
            const Vec2 at = mid + along * (frontLen * (frac - 0.5));
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
        // The depth a lot of this program is BUILT at, not the shallowest one
        // it would accept — same reason as the frontage above.
        const Real wantDepth = want ? want->targetDepth() : 14.0;
        const Vec2 inward(-along.y, along.x);
        // Measured ALONG the frontage normal, not from an axis-aligned box: a
        // block at an angle to the axes is the normal case, and its bounding
        // box is deeper than the block is.
        Real dlo = 1e30, dhi = -1e30;
        for (const Vec2& v : tessellate(region.outer, 0.25)) {
            const Real t = dot(v, inward);
            dlo = std::min(dlo, t);
            dhi = std::max(dhi, t);
        }
        const Real deep = dhi - dlo;
        if (hasFront && roomToSplit && deep > wantDepth * 2.1) {
            // ROWS, evenly: peeling one target-deep row off the front leaves a
            // remainder of whatever is left over, and that remainder is the
            // odd lot out. Dividing the depth into a whole number of rows makes
            // every row the same — the same stationing argument as the
            // frontage, in the other direction.
            const int rows = std::max(2, static_cast<int>(
                                             std::lround(deep / wantDepth)));
            const Real step = deep / rows;
            const Vec2 at = mid + inward * (step - (dot(mid, inward) - dlo));
            std::vector<Shape2> a, b;
            splitShape(region, at, along, a, b);
            // Only take the split if BOTH halves still have a way in. A depth
            // split whose back half is landlocked does not create a lot, it
            // creates stranded green — and that green is the whole complaint
            // about big blocks. Better a deeper lot than an unreachable void.
            if (!a.empty() && !b.empty() && hasAccess(a) && hasAccess(b)) {
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

// A corridor rectangle centred on `at`, running along `dir`, long enough to
// cross anything it is given. Its edges carry EdgeTag::Lane, and the boolean
// propagates tags — so the sub-blocks either side come back already knowing
// they front a lane, with nothing re-deriving it.
Shape2 laneCorridor(const Shape2& region, const Vec2& at, const Vec2& dir,
                    Real width) {
    Vec2 lo, hi;
    bounds(region, lo, hi);
    const Real span = (hi - lo).length() + 20;
    const Vec2 d = normalize(dir);
    const Vec2 n(-d.y, d.x);
    Poly2 ring = {at - d * span - n * (width * 0.5),
                  at + d * span - n * (width * 0.5),
                  at + d * span + n * (width * 0.5),
                  at - d * span + n * (width * 0.5)};
    Shape2 c = shapeFromPoly(ring);
    for (Edge2& e : c.outer.edges) e.tag = EdgeTag::Lane;
    return c;
}

// Split a region with service lanes until no piece is deeper than its programs
// can serve from the perimeter. Returns the sub-blocks; appends the corridors.
void insertLanes(const Shape2& region, Real lotDepth, const BlockParcelParams& params,
                 int lanesLeft, std::vector<Shape2>& out,
                 std::vector<BlockLane>& lanes) {
    Vec2 lo, hi;
    bounds(region, lo, hi);
    const Poly2 ring = tessellate(region.outer, 0.3);
    const OBB2 box = orientedBoundingBox(ring);
    const Real depth = std::min(box.half[0], box.half[1]) * 2;
    // What a perimeter ring leaves stranded in the middle.
    const Real core = depth - 2 * lotDepth * params.ringDepthFactor;
    if (!params.lanes || lanesLeft <= 0 || core <= params.strandedCoreDepth ||
        area(region) < params.minLotArea * 6) {
        out.push_back(region);
        return;
    }
    // Cut ALONG the long axis, through the middle: that halves the depth while
    // leaving both halves their full street frontage.
    const int longAxis = box.half[0] >= box.half[1] ? 0 : 1;
    const Vec2 dir = box.axis[longAxis];
    Shape2 corridor = laneCorridor(region, box.center, dir, params.laneWidth);

    std::vector<Shape2> pieces = shapeBool({region}, {corridor}, BoolOp::Subtract);
    std::vector<Shape2> cut = shapeBool({region}, {corridor}, BoolOp::Intersect);
    if (pieces.size() < 2 || cut.empty()) {
        out.push_back(region);
        return;
    }
    for (Shape2& c : cut) {
        if (area(c) < 1.0) continue;
        BlockLane bl;
        bl.area = c;
        Vec2 clo, chi;
        bounds(c, clo, chi);
        bl.from = box.center - dir * ((chi - clo).length() * 0.5);
        bl.to = box.center + dir * ((chi - clo).length() * 0.5);
        bl.width = params.laneWidth;
        bl.paved = true;
        lanes.push_back(std::move(bl));
    }
    for (Shape2& p : pieces)
        insertLanes(p, lotDepth, params, lanesLeft - 1, out, lanes);
}

}  // namespace

ParcelledBlock parcelBlock(const Shape2& block, ProgramSet& programs,
                           const BlockParcelParams& params, bool enclosed,
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

    // LANES FIRST. How deep a block can be before it needs one comes from the
    // programs that would live on it, so a campus block is not cut up by lanes
    // sized for terraced houses.
    std::vector<Shape2> subBlocks;
    for (const Shape2& piece : inner) {
        Shape2 probe = piece;
        tagLotEdges(probe, piece, params.partyMaxFront);
        const LotTags pt = measureLot(probe, {}, klass, enclosed, coreness);
        const int bi = programs.bestFor(pt);
        const Real lotDepth =
            bi >= 0 ? std::max(programs.programs[bi].minW, programs.programs[bi].minD)
                    : 24.0;
        insertLanes(piece, lotDepth, params, params.maxLanes, subBlocks, out.lanes);
    }

    // Each sub-block is its OWN reference: a lot cut from it fronts whatever
    // that boundary faces — the street across the margin, or a lane. The
    // boolean carried the tags through, so nothing re-derives them.
    for (const Shape2& piece : subBlocks) {
        c.reference = &piece;
        c.cut(piece, 0);
    }
    out.lots = std::move(c.lots);
    out.openSpace = std::move(c.open);

    // PASSAGES. Any shared green still cut off from everything gets a footpath
    // to the nearest access. A green nobody can walk into is a bug, not a park.
    if (params.passageWidth > 0) {
        // Snapshot the greens that need one BEFORE touching anything. Retiring
        // a lot pushes it back into openSpace, and iterating that vector while
        // it reallocates would leave the loop holding a dangling reference —
        // the classic version of this bug, and it was here.
        std::vector<Shape2> needPassage;
        for (const Shape2& g : out.openSpace) {
            if (area(g) < params.sharedGreenMinArea) continue;   // a back garden
            ParcelledBlock probe;
            probe.parcellable = out.parcellable;
            probe.lanes = out.lanes;
            probe.openSpace = {g};
            if (!probe.openSpaceIsReachable(params.sharedGreenMinArea))
                needPassage.push_back(g);
        }
        for (const Shape2& green : needPassage) {
            // Re-check: an earlier passage may already have opened this green.
            ParcelledBlock probe;
            probe.parcellable = out.parcellable;
            probe.lanes = out.lanes;
            probe.openSpace = {green};
            if (probe.openSpaceIsReachable(params.sharedGreenMinArea)) continue;

            // Shortest route from the green to the nearest access boundary.
            const Vec2 from = centroid(green);
            Vec2 to = from;
            Real bestD = 1e30;
            auto consider = [&](const Shape2& target) {
                for (const Vec2& p : tessellate(target.outer, 0.6)) {
                    const Real d = (p - from).length();
                    if (d < bestD) { bestD = d; to = p; }
                }
            };
            consider(out.parcellable);
            for (const BlockLane& l : out.lanes) consider(l.area);
            if (bestD > 1e29) continue;

            const Vec2 dir = normalize(to - from);
            if (dir.lengthSquared() < 0.5) continue;
            Shape2 corridor = laneCorridor(out.parcellable, (from + to) * 0.5, dir,
                                           params.passageWidth);
            std::vector<Shape2> path =
                shapeBool({corridor}, {out.parcellable}, BoolOp::Intersect);
            if (path.empty()) continue;

            // The passage is carved OUT of whatever it crosses. A lot it cuts
            // through gives up that ground; if the remainder can no longer hold
            // its program the lot is retired to open space rather than shipped
            // in a state the inversion promises never happens.
            std::vector<ParcelledLot> keep;
            for (ParcelledLot& lot : out.lots) {
                std::vector<Shape2> rest =
                    shapeBool({lot.shape}, path, BoolOp::Subtract);
                if (rest.empty()) { out.openSpace.push_back(lot.shape); continue; }
                Shape2 biggest = rest[0];
                for (Shape2& r : rest)
                    if (area(r) > area(biggest)) biggest = r;
                if (std::fabs(area(biggest) - area(lot.shape)) < 0.5) {
                    keep.push_back(std::move(lot));               // untouched
                    continue;
                }
                tagLotEdges(biggest, out.parcellable, params.partyMaxFront);
                LotTags t = measureLot(biggest, {}, klass, enclosed, coreness);
                if (lot.program >= 0 &&
                    lotFitsProgram(t, programs.programs[lot.program])) {
                    lot.shape = std::move(biggest);
                    lot.tags = t;
                    keep.push_back(std::move(lot));
                } else {
                    out.openSpace.push_back(std::move(biggest));
                }
            }
            out.lots = std::move(keep);
            for (Shape2& pth : path) {
                BlockLane bl;
                bl.area = pth;
                bl.from = from;
                bl.to = to;
                bl.width = params.passageWidth;
                bl.paved = false;              // a soft footpath, not a service lane
                bl.passage = true;
                out.lanes.push_back(std::move(bl));
            }
        }
    }
    return out;
}

}  // namespace engine
