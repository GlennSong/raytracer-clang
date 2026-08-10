#include "shape_ops.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace engine {
namespace {

constexpr Real kTau = 6.28318530717958647692;
constexpr Real kEps = 1e-9;

// Even-odd containment across a whole loop set, so holes count correctly: a
// point inside an outer ring but also inside its hole crosses twice and reads
// as outside, which is exactly right.
bool inLoops(const std::vector<Poly2>& loops, const Vec2& p) {
    int c = 0;
    for (const Poly2& l : loops)
        if (l.size() >= 3 && pointInPolygon(l, p)) c ^= 1;
    return c != 0;
}

Real distToSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 d = b - a;
    const Real len2 = d.lengthSquared();
    if (len2 < kEps) return (p - a).length();
    Real t = dot(p - a, d) / len2;
    t = std::max(Real(0), std::min(Real(1), t));
    return (p - (a + d * t)).length();
}

bool segCross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d,
              Real& t) {
    const Vec2 r = b - a, s = d - c;
    const Real den = cross(r, s);
    if (std::fabs(den) < 1e-12) return false;
    const Real tt = cross(c - a, s) / den;
    const Real uu = cross(c - a, r) / den;
    if (tt < -1e-9 || tt > 1 + 1e-9 || uu < -1e-9 || uu > 1 + 1e-9) return false;
    t = std::max(Real(0), std::min(Real(1), tt));
    return true;
}

// Does `p` lie ON segment a->b (within tol)? Returns its parameter. Needed for
// T-junctions and, critically, for COLLINEAR OVERLAP: two shapes that share a
// wall have no proper crossing at all, and without this the shared run is never
// split and the boolean silently produces nonsense. Uniting a wing onto a box
// shares an edge BY CONSTRUCTION, so this is the common case, not a corner one.
bool pointOnSeg(const Vec2& a, const Vec2& b, const Vec2& p, Real tol, Real& t) {
    const Vec2 d = b - a;
    const Real len2 = d.lengthSquared();
    if (len2 < 1e-18) return false;
    const Real u = dot(p - a, d) / len2;
    if (u < 1e-9 || u > 1 - 1e-9) return false;         // interior only
    const Vec2 q = a + d * u;
    if ((p - q).length() > tol) return false;
    t = u;
    return true;
}

// A straight CHORD of some source edge, carrying enough provenance to be put
// back on that edge as a true sub-arc afterwards.
struct Piece {
    Vec2 a, b;
    int set = 0;          // 0 = operand A, 1 = operand B
    int loop = 0;         // index into the flattened loop list
    int edge = 0;         // edge index within that loop
    Real t0 = 0, t1 = 1;  // parameter span within the source edge
    Real bulge = 0;       // the source edge's bulge (sign follows traversal)
    EdgeTag tag = EdgeTag::None;
};

// Flatten a shape list into loops, remembering nothing about which shape a loop
// came from — for the boolean, only the geometry and its winding matter.
std::vector<Loop2> flattenLoops(const std::vector<Shape2>& shapes) {
    std::vector<Loop2> out;
    for (const Shape2& s : shapes) {
        if (s.outer.size() >= 3) out.push_back(s.outer);
        for (const Loop2& h : s.holes)
            if (h.size() >= 3) out.push_back(h);
    }
    return out;
}

std::vector<Poly2> tessLoops(const std::vector<Loop2>& loops, Real tol) {
    std::vector<Poly2> out;
    out.reserve(loops.size());
    for (const Loop2& l : loops) out.push_back(tessellate(l, tol));
    return out;
}

// Chop every edge into chords fine enough that crossings found on them are
// within `tol` of crossings on the true curve. Straight edges stay whole.
void emitPieces(const std::vector<Loop2>& loops, int set, Real tol,
                std::vector<Piece>& out) {
    for (std::size_t li = 0; li < loops.size(); ++li) {
        const Loop2& loop = loops[li];
        for (std::size_t ei = 0; ei < loop.size(); ++ei) {
            const Vec2 A = loop.start(ei), B = loop.end(ei);
            const Real bulge = loop.bulge(ei);
            const ArcGeom g = arcGeom(A, B, bulge);
            int segs = 1;
            if (!g.straight) {
                const Real ratio =
                    std::max(Real(-1), std::min(Real(1), 1.0 - tol / g.radius));
                const Real maxStep = 2.0 * std::acos(ratio);
                segs = std::max(2, static_cast<int>(std::ceil(
                           std::fabs(g.sweep) / std::max(maxStep, Real(1e-3)))));
            }
            for (int k = 0; k < segs; ++k) {
                Piece p;
                p.t0 = static_cast<Real>(k) / segs;
                p.t1 = static_cast<Real>(k + 1) / segs;
                p.a = arcPoint(A, B, bulge, p.t0);
                p.b = arcPoint(A, B, bulge, p.t1);
                p.set = set;
                p.loop = static_cast<int>(li);
                p.edge = static_cast<int>(ei);
                p.bulge = bulge;
                p.tag = loop.edges[ei].tag;
                out.push_back(p);
            }
        }
    }
}

// Walk a chained run of pieces and put every contiguous same-source run back
// together as ONE edge — a true sub-arc when the source was curved. This is
// what keeps a curved wall curved through a boolean; without it the output is
// permanently the chord fan the crossing test happened to need.
Loop2 mergeRun(const std::vector<Piece>& run) {
    Loop2 loop;
    std::size_t i = 0;
    while (i < run.size()) {
        std::size_t j = i;
        // Extend while the pieces come from the same source edge AND their
        // parameters are contiguous (a boolean can hand back two disjoint runs
        // of the SAME edge — they must stay two edges).
        while (j + 1 < run.size() && run[j + 1].set == run[i].set &&
               run[j + 1].loop == run[i].loop && run[j + 1].edge == run[i].edge &&
               std::fabs(run[j + 1].t0 - run[j].t1) < 1e-9)
            ++j;
        Edge2 e;
        e.a = run[i].a;
        e.bulge = subBulge(run[i].bulge, run[i].t0, run[j].t1);
        e.tag = run[i].tag;
        loop.edges.push_back(e);
        i = j + 1;
    }
    return loop;
}

Real loopDepthArea(const Loop2& l) { return area(l); }

// Where a fragment sits relative to the OTHER operand. A plain inside/outside
// boolean cannot describe a fragment lying exactly along the other's boundary,
// and that ambiguity is what makes naive versions of this algorithm fail on
// shared walls: point-in-polygon on a boundary point answers arbitrarily.
enum class Side { Outside, Inside, OnSame, OnOpposite };

Side classifyPiece(const Vec2& mid, const Vec2& dir,
                   const std::vector<Piece>& other,
                   const std::vector<Poly2>& otherTess, Real tol) {
    for (const Piece& o : other) {
        Real t;
        const Vec2 od = o.b - o.a;
        if (od.lengthSquared() < 1e-18) continue;
        // "mid lies on o" — endpoints included, since a fragment's midpoint can
        // coincide with a chord end after splitting.
        const Real len2 = od.lengthSquared();
        const Real u = dot(mid - o.a, od) / len2;
        if (u < -1e-9 || u > 1 + 1e-9) continue;
        if ((mid - (o.a + od * u)).length() > tol) continue;
        (void)t;
        // Coincident boundary: which way does the other operand run here? Same
        // direction means the two shapes lie on the same side of this wall;
        // opposite means they face each other across it.
        return dot(dir, normalize(od)) > 0 ? Side::OnSame : Side::OnOpposite;
    }
    return inLoops(otherTess, mid) ? Side::Inside : Side::Outside;
}

}  // namespace

// ---------------------------------------------------------------------------
// Assembly
// ---------------------------------------------------------------------------

std::vector<Shape2> assembleShapes(std::vector<Loop2> loops) {
    // Drop degenerate loops before anything reasons about nesting.
    loops.erase(std::remove_if(loops.begin(), loops.end(),
                               [](const Loop2& l) {
                                   return l.size() < 3 || area(l) < 1e-6;
                               }),
                loops.end());
    const std::size_t n = loops.size();
    std::vector<Poly2> tess(n);
    for (std::size_t i = 0; i < n; ++i) tess[i] = tessellate(loops[i], 0.02);

    // Nesting depth by containment of a representative point. Area ordering
    // breaks ties safely: a loop can only be inside a strictly larger one.
    std::vector<int> depth(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j) continue;
            if (loopDepthArea(loops[j]) > loopDepthArea(loops[i]) &&
                tess[i].size() >= 3 && pointInPolygon(tess[j], tess[i][0]))
                ++depth[i];
        }
    }

    // Even depth is a boundary, odd is a hole. Force the winding to match, so
    // downstream never has to ask again.
    std::vector<Shape2> shapes;
    std::vector<int> outerOf(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        if (depth[i] % 2 != 0) continue;
        if (signedArea(loops[i]) < 0) reverseLoop(loops[i]);
        Shape2 s;
        s.outer = loops[i];
        outerOf[i] = static_cast<int>(shapes.size());
        shapes.push_back(std::move(s));
    }
    // Each hole joins the SMALLEST boundary that contains it — with nested
    // rings (a court inside a building inside a block) the largest container is
    // the wrong answer.
    for (std::size_t i = 0; i < n; ++i) {
        if (depth[i] % 2 == 0) continue;
        int best = -1;
        Real bestArea = 0;
        for (std::size_t j = 0; j < n; ++j) {
            if (outerOf[j] < 0 || tess[i].size() < 3) continue;
            if (!pointInPolygon(tess[j], tess[i][0])) continue;
            const Real aj = loopDepthArea(loops[j]);
            if (best < 0 || aj < bestArea) { best = outerOf[j]; bestArea = aj; }
        }
        if (best < 0) continue;                 // orphan hole: drop it
        if (signedArea(loops[i]) > 0) reverseLoop(loops[i]);
        shapes[best].holes.push_back(loops[i]);
    }
    return shapes;
}

// ---------------------------------------------------------------------------
// Boolean
// ---------------------------------------------------------------------------

std::vector<Shape2> shapeBool(const std::vector<Shape2>& aIn,
                              const std::vector<Shape2>& bIn, BoolOp op,
                              Real chordTol) {
    std::vector<Loop2> A = flattenLoops(aIn), B = flattenLoops(bIn);
    if (A.empty() && B.empty()) return {};
    // Degenerate operands: the answer is a plain copy, no algebra needed.
    if (B.empty()) return op == BoolOp::Intersect ? std::vector<Shape2>{} : aIn;
    if (A.empty()) {
        if (op == BoolOp::Unite) return bIn;
        return {};
    }
    const std::vector<Poly2> tessA = tessLoops(A, chordTol);
    const std::vector<Poly2> tessB = tessLoops(B, chordTol);

    std::vector<Piece> pieces;
    emitPieces(A, 0, chordTol, pieces);
    emitPieces(B, 1, chordTol, pieces);
    std::vector<Piece> setPieces[2];
    for (const Piece& p : pieces) setPieces[p.set].push_back(p);

    // Split every chord at its crossings with the OTHER operand, then classify
    // each surviving fragment by where its midpoint lies. One algorithm; only
    // the keep rule changes with the op.
    const Real onTol = std::max(chordTol * 0.5, Real(1e-6));
    std::vector<Piece> kept;
    for (const Piece& p : pieces) {
        std::vector<Real> cuts{0.0, 1.0};
        for (const Piece& o : setPieces[1 - p.set]) {
            Real t;
            if (segCross(p.a, p.b, o.a, o.b, t)) cuts.push_back(t);
            // T-junctions and collinear overlap produce no proper crossing —
            // split at the other edge's endpoints where they land on this one,
            // or a shared wall is never divided into its inside/outside runs.
            if (pointOnSeg(p.a, p.b, o.a, onTol, t)) cuts.push_back(t);
            if (pointOnSeg(p.a, p.b, o.b, onTol, t)) cuts.push_back(t);
        }
        std::sort(cuts.begin(), cuts.end());
        for (std::size_t k = 0; k + 1 < cuts.size(); ++k) {
            if (cuts[k + 1] - cuts[k] < 1e-7) continue;
            const Vec2 P = lerp(p.a, p.b, cuts[k]);
            const Vec2 Q = lerp(p.a, p.b, cuts[k + 1]);
            const Vec2 mid = (P + Q) * 0.5;
            const Vec2 dir = normalize(Q - P);
            const Side side = classifyPiece(mid, dir, setPieces[1 - p.set],
                                            p.set == 0 ? tessB : tessA, onTol);
            // The four-way rule set. A shared wall is kept exactly ONCE (from
            // operand A) when both shapes lie on the same side of it, and
            // dropped entirely when they face each other across it — which is
            // what makes two boxes sharing a wall unite into one clean outline
            // instead of a self-overlapping figure of eight.
            bool keep = false, flip = false;
            switch (op) {
                case BoolOp::Unite:
                    keep = (side == Side::Outside) ||
                           (side == Side::OnSame && p.set == 0);
                    break;
                case BoolOp::Intersect:
                    keep = (side == Side::Inside) ||
                           (side == Side::OnSame && p.set == 0);
                    break;
                case BoolOp::Subtract:
                    // A survives where B is not; B survives inside A, REVERSED,
                    // so the clip's boundary winds as a hole. A coincident wall
                    // with B on the far side (OnOpposite) still bounds A.
                    if (p.set == 0)
                        keep = (side == Side::Outside) || (side == Side::OnOpposite);
                    else
                        keep = (side == Side::Inside);
                    flip = (p.set == 1);
                    break;
            }
            if (!keep) continue;
            Piece q = p;
            // Map the chord-local cut back onto the SOURCE edge's parameter.
            const Real s0 = p.t0 + (p.t1 - p.t0) * cuts[k];
            const Real s1 = p.t0 + (p.t1 - p.t0) * cuts[k + 1];
            if (flip) {
                // Reversing an arc negates its bulge and mirrors its parameter,
                // so the merge-back downstream stays uniform.
                q.a = Q; q.b = P;
                q.t0 = 1.0 - s1; q.t1 = 1.0 - s0;
                q.bulge = -p.bulge;
            } else {
                q.a = P; q.b = Q;
                q.t0 = s0; q.t1 = s1;
            }
            kept.push_back(q);
        }
    }
    if (kept.empty()) return {};

    // Chain head-to-tail; at a shared vertex take the most-clockwise turn so
    // the walk hugs the boundary instead of cutting across it.
    const Real tol = 1e-4;
    auto key = [&](const Vec2& v) {
        return std::make_pair(static_cast<long long>(std::llround(v.x / tol)),
                              static_cast<long long>(std::llround(v.y / tol)));
    };
    std::map<std::pair<long long, long long>, std::vector<int>> byStart;
    for (int i = 0; i < static_cast<int>(kept.size()); ++i)
        byStart[key(kept[i].a)].push_back(i);

    std::vector<char> used(kept.size(), 0);
    std::vector<Loop2> loops;
    for (int s = 0; s < static_cast<int>(kept.size()); ++s) {
        if (used[s]) continue;
        std::vector<Piece> run;
        int cur = s;
        while (cur != -1 && !used[cur]) {
            used[cur] = 1;
            run.push_back(kept[cur]);
            const Vec2 din = normalize(kept[cur].b - kept[cur].a);
            int next = -1;
            Real best = -1e9;
            auto it = byStart.find(key(kept[cur].b));
            if (it != byStart.end()) {
                for (int cand : it->second) {
                    if (used[cand]) continue;
                    const Vec2 dout = normalize(kept[cand].b - kept[cand].a);
                    const Real ang = std::atan2(cross(din, dout), dot(din, dout));
                    if (-ang > best) { best = -ang; next = cand; }
                }
            }
            cur = next;
        }
        if (run.size() < 2) continue;
        Loop2 loop = mergeRun(run);
        if (loop.size() >= 3 && area(loop) > 1e-6) loops.push_back(std::move(loop));
    }
    return assembleShapes(std::move(loops));
}

std::vector<Shape2> unite(const Shape2& a, const Shape2& b) {
    return shapeBool({a}, {b}, BoolOp::Unite);
}
std::vector<Shape2> subtract(const Shape2& a, const Shape2& b) {
    return shapeBool({a}, {b}, BoolOp::Subtract);
}
std::vector<Shape2> intersect(const Shape2& a, const Shape2& b) {
    return shapeBool({a}, {b}, BoolOp::Intersect);
}

std::vector<Shape2> uniteAll(const std::vector<Shape2>& shapes) {
    if (shapes.empty()) return {};
    std::vector<Shape2> acc{shapes[0]};
    for (std::size_t i = 1; i < shapes.size(); ++i)
        acc = shapeBool(acc, {shapes[i]}, BoolOp::Unite);
    return acc;
}

// ---------------------------------------------------------------------------
// Offset
// ---------------------------------------------------------------------------

Real TagOffsets::forTag(EdgeTag t) const {
    switch (t) {
        case EdgeTag::Street: return street;
        case EdgeTag::Side:   return side;
        case EdgeTag::Rear:   return rear;
        case EdgeTag::Court:  return court;
        case EdgeTag::Party:  return party;
        case EdgeTag::Lane:   return lane;
        case EdgeTag::None:   break;
    }
    return none;
}

namespace {

// The offset of one edge: a parallel LINE for a straight edge, a CONCENTRIC
// CIRCLE for an arc. Treating an arc's chord as its geometry — as a naive
// polygon offset must — offsets the INSCRIBED POLYGON instead of the curve, and
// a circle grown by 3 m comes back the wrong size and shape.
struct OffCurve {
    bool arc = false;
    Vec2 pt, dir;              // line: a point on it and its direction
    Vec2 centre;               // arc: concentric circle
    Real radius = 0;
    Real sweepSign = 1;        // the source arc's rotation direction
    bool dead = false;         // the offset consumed this arc entirely
};

// Intersections, picking the root nearest `hint` (the naively-offset corner),
// which is what disambiguates the two solutions of the curved cases.
bool intersectCurves(const OffCurve& a, const OffCurve& b, const Vec2& hint,
                     Real miter, Vec2& out) {
    if (!a.arc && !b.arc) {
        const Real den = cross(a.dir, b.dir);
        if (std::fabs(den) < 1e-9) { out = b.pt; return true; }
        out = a.pt + a.dir * (cross(b.pt - a.pt, b.dir) / den);
        // MITER LIMIT. Without it the chord joints of a rounded ring fly off
        // and the offset grows spikes — the failure offsetPlan documents.
        if ((out - hint).length() > miter) out = b.pt;
        return true;
    }
    if (a.arc && b.arc) {
        const Vec2 d = b.centre - a.centre;
        const Real dist = d.length();
        if (dist < 1e-9) {
            // Same centre (consecutive arcs of one circle): every point of the
            // offset circle is a solution, so keep the original vertex's angle.
            const Vec2 r = normalize(hint - a.centre);
            out = a.centre + r * a.radius;
            return true;
        }
        const Real r0 = a.radius, r1 = b.radius;
        if (dist > r0 + r1 || dist < std::fabs(r0 - r1)) return false;
        const Real x = (dist * dist - r1 * r1 + r0 * r0) / (2 * dist);
        const Real h2 = r0 * r0 - x * x;
        if (h2 < 0) return false;
        const Real h = std::sqrt(h2);
        const Vec2 u = d / dist, v(-u.y, u.x);
        const Vec2 c1 = a.centre + u * x + v * h;
        const Vec2 c2 = a.centre + u * x - v * h;
        out = (c1 - hint).lengthSquared() <= (c2 - hint).lengthSquared() ? c1 : c2;
        return true;
    }
    // Line/circle.
    const OffCurve& L = a.arc ? b : a;
    const OffCurve& C = a.arc ? a : b;
    const Vec2 f = L.pt - C.centre;
    const Real bq = 2 * dot(f, L.dir);
    const Real cq = dot(f, f) - C.radius * C.radius;
    const Real disc = bq * bq - 4 * cq;
    if (disc < 0) return false;
    const Real sq = std::sqrt(disc);
    const Vec2 p1 = L.pt + L.dir * ((-bq + sq) * 0.5);
    const Vec2 p2 = L.pt + L.dir * ((-bq - sq) * 0.5);
    out = (p1 - hint).lengthSquared() <= (p2 - hint).lengthSquared() ? p1 : p2;
    return true;
}

// Exact miter offset of one loop. `dist[i]` moves edge i along its OUTWARD
// normal (positive grows). Returns an empty loop when the result collapses.
Loop2 offsetLoopExact(const Loop2& loop, const std::vector<Real>& dist) {
    const std::size_t n = loop.size();
    if (n < 3 || dist.size() != n) return Loop2{};
    std::vector<OffCurve> curves(n);
    std::vector<Vec2> outwardAt(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 A = loop.start(i), B = loop.end(i);
        const Vec2 chord = B - A;
        if (chord.lengthSquared() < 1e-18) return Loop2{};
        const Vec2 cd = normalize(chord);
        const Vec2 outward(cd.y, -cd.x);          // right of a CCW edge
        outwardAt[i] = outward;
        const ArcGeom g = arcGeom(A, B, loop.bulge(i));
        if (g.straight) {
            curves[i] = OffCurve{false, A + outward * dist[i], cd, {}, 0, 1, false};
            continue;
        }
        OffCurve c;
        c.arc = true;
        c.centre = g.centre;
        c.sweepSign = g.sweep >= 0 ? 1.0 : -1.0;
        // A CCW-sweeping arc on a CCW loop bulges outward, so its centre is on
        // the inside and growing outward INCREASES the radius; a concave arc's
        // centre is outside, so growing decreases it.
        c.radius = g.radius + dist[i] * c.sweepSign;
        if (c.radius <= 1e-6) { c.dead = true; c.radius = 1e-6; }
        curves[i] = c;
    }
    Real maxD = 0;
    for (Real v : dist) maxD = std::max(maxD, std::fabs(v));
    const Real miter = maxD * 4.0 + 0.5;

    Loop2 out;
    out.edges.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        // The naive corner — the average of the two edges' outward moves — is
        // the hint that picks the right root of a curved intersection.
        const Vec2 nAvg = normalize(outwardAt[prev] + outwardAt[i]);
        const Vec2 hint = loop.start(i) +
                          (nAvg.lengthSquared() > kEps ? nAvg : outwardAt[i]) *
                              ((dist[prev] + dist[i]) * 0.5);
        Vec2 q;
        if (!intersectCurves(curves[prev], curves[i], hint, miter, q)) q = hint;
        out.edges[i].a = q;
        out.edges[i].tag = loop.edges[i].tag;
        out.edges[i].bulge = 0;
    }
    // Recompute each arc's bulge from where its endpoints actually LANDED on
    // the concentric circle. For a whole circle the sweep is unchanged and the
    // bulge comes back identical; where an arc meets a straight wall the
    // tangent point moves, and the sweep genuinely changes.
    for (std::size_t i = 0; i < n; ++i) {
        if (!curves[i].arc || curves[i].dead) continue;
        const Vec2 A = out.edges[i].a, B = out.edges[(i + 1) % n].a;
        const Vec2 c = curves[i].centre;
        Real a0 = std::atan2(A.y - c.y, A.x - c.x);
        Real a1 = std::atan2(B.y - c.y, B.x - c.x);
        Real sweep = a1 - a0;
        // Normalize into the source arc's rotational direction.
        while (sweep <= -kTau * 0.5) sweep += kTau;
        while (sweep > kTau * 0.5) sweep -= kTau;
        if (curves[i].sweepSign > 0 && sweep < 0) sweep += kTau;
        if (curves[i].sweepSign < 0 && sweep > 0) sweep -= kTau;
        out.edges[i].bulge = std::tan(sweep * 0.25);
    }
    return out;
}

// Is this actually the offset it claims to be? By DEFINITION every point of an
// offset by d sits |d| from the original boundary, on the correct side. Nothing
// weaker catches a symmetric over-shrink: pushing a 10 m square in by 6 m sends
// each wall past its opposite number, and the result is the same square
// rotated 180 degrees — which preserves winding AND has a plausible positive
// area, so both the area test and the sign test wave it through. Measuring the
// distance is the only check that sees the shape has collapsed.
bool offsetIsSane(const Shape2& src, const Shape2& result, Real d) {
    std::vector<Poly2> bnd;
    bnd.push_back(tessellate(src.outer, 0.05));
    for (const Loop2& h : src.holes) bnd.push_back(tessellate(h, 0.05));
    const Real want = std::fabs(d) * 0.98;
    auto ok = [&](const Loop2& l) {
        for (const Edge2& e : l.edges) {
            Real best = 1e30;
            for (const Poly2& p : bnd)
                for (std::size_t i = 0; i < p.size(); ++i)
                    best = std::min(best,
                                    distToSegment(e.a, p[i], p[(i + 1) % p.size()]));
            if (best < want) return false;
            const bool inside = contains(src, e.a);
            if ((d < 0) != inside) return false;   // shrink inward, grow outward
        }
        return true;
    };
    if (!ok(result.outer)) return false;
    for (const Loop2& h : result.holes)
        if (!ok(h)) return false;
    return true;
}

// Did the exact offset stay a simple polygon? Cheap self-intersection test on
// the tessellation — the signal that the answer needs the field path.
bool selfIntersects(const Loop2& loop) {
    const Poly2 p = tessellate(loop, 0.05);
    const std::size_t n = p.size();
    if (n < 4) return false;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue;   // adjacent at the wrap
            Real t;
            if (segCross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n], t)) {
                // Endpoint touches are not crossings.
                const Vec2 x = lerp(p[i], p[(i + 1) % n], t);
                if ((x - p[i]).length() > 1e-6 &&
                    (x - p[(i + 1) % n]).length() > 1e-6)
                    return true;
            }
        }
    }
    return false;
}

}  // namespace

Shape2 offsetEdges(const Shape2& shape, const std::vector<Real>& dist) {
    if (shape.outer.size() != dist.size()) return Shape2{};
    const Real sign0 = signedArea(shape.outer) >= 0 ? 1.0 : -1.0;
    Shape2 out;
    out.outer = offsetLoopExact(shape.outer, dist);
    if (out.outer.size() < 3 || area(out.outer) < 1e-6 ||
        (signedArea(out.outer) >= 0 ? 1.0 : -1.0) != sign0)
        return Shape2{};
    // A hole boundary carries its own tags, and a hole moves the SAME way the
    // outer does under this convention: on a CW hole, "right of travel" points
    // into the void, so a positive offset grows the solid into the court and
    // the court shrinks. A donut grown outward has a smaller hole, which is
    // what the Minkowski sum says too.
    Real mean = 0;
    for (Real v : dist) mean += v;
    mean /= static_cast<Real>(std::max<std::size_t>(1, dist.size()));
    for (const Loop2& h : shape.holes) {
        Loop2 oh = offsetLoopExact(h, std::vector<Real>(h.size(), mean));
        if (oh.size() >= 3 && area(oh) > 1e-6) out.holes.push_back(std::move(oh));
    }
    orient(out);
    return out;
}

Shape2 offsetByTag(const Shape2& shape, const TagOffsets& offsets) {
    auto perEdge = [&](const Loop2& l) {
        std::vector<Real> d(l.size());
        for (std::size_t i = 0; i < l.size(); ++i)
            d[i] = offsets.forTag(l.edges[i].tag);
        return d;
    };
    const Real sign0 = signedArea(shape.outer) >= 0 ? 1.0 : -1.0;
    Shape2 out;
    out.outer = offsetLoopExact(shape.outer, perEdge(shape.outer));
    if (out.outer.size() < 3 || area(out.outer) < 1e-6 ||
        (signedArea(out.outer) >= 0 ? 1.0 : -1.0) != sign0)
        return Shape2{};
    // Holes get their own tags honoured too — a courtyard wall is a Court edge
    // and can carry a different setback from the street frontage.
    for (const Loop2& h : shape.holes) {
        Loop2 oh = offsetLoopExact(h, perEdge(h));
        if (oh.size() >= 3 && area(oh) > 1e-6) out.holes.push_back(std::move(oh));
    }
    orient(out);
    return out;
}

std::vector<Shape2> offsetShape(const Shape2& shape, Real d) {
    if (shape.outer.size() < 3) return {};
    if (std::fabs(d) < 1e-9) return {shape};
    // EXACT first: fast, keeps corners sharp, keeps arcs as arcs.
    const Real sign0 = signedArea(shape.outer) >= 0 ? 1.0 : -1.0;
    Shape2 exact;
    exact.outer = offsetLoopExact(shape.outer,
                                  std::vector<Real>(shape.outer.size(), d));
    // A shrink that overshoots turns the loop INSIDE OUT — it comes back with
    // real area but the opposite winding. |area| alone cannot see that, so an
    // over-shrunk 10 m square would otherwise report a healthy little shape
    // instead of "this collapsed".
    bool ok = exact.outer.size() >= 3 && area(exact.outer) > 1e-6 &&
              (signedArea(exact.outer) >= 0 ? 1.0 : -1.0) == sign0 &&
              !selfIntersects(exact.outer);
    if (ok) {
        for (const Loop2& h : shape.holes) {
            const Real hs = signedArea(h) >= 0 ? 1.0 : -1.0;
            Loop2 oh = offsetLoopExact(h, std::vector<Real>(h.size(), d));
            if (oh.size() < 3 || area(oh) < 1e-6 || selfIntersects(oh) ||
                (signedArea(oh) >= 0 ? 1.0 : -1.0) != hs) {
                ok = false;
                break;
            }
            exact.holes.push_back(std::move(oh));
        }
    }
    if (ok) {
        const Real a0 = area(shape), a1 = area(exact);
        const bool grew = d > 0;
        if (((grew && a1 >= a0) || (!grew && a1 <= a0)) &&
            offsetIsSane(shape, exact, d)) {
            orient(exact);
            return {exact};
        }
    }
    // The exact path could not answer — self-intersected, collapsed, or the
    // topology changed. This is exactly the case the field exists for.
    return fieldOffset(shape, d);
}

// ---------------------------------------------------------------------------
// Corners
// ---------------------------------------------------------------------------

Loop2 filletCorners(const Loop2& loop, const std::vector<Real>& radii) {
    const std::size_t n = loop.size();
    if (n < 3 || radii.size() != n) return loop;
    Loop2 out;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        const Real r = radii[i];
        // Only straight-to-straight corners are filleted: an arc already
        // meeting a wall is either tangent-continuous or deliberately not, and
        // rounding it again would restate a curve the author chose.
        if (r <= 1e-6 || std::fabs(loop.bulge(prev)) > kEps ||
            std::fabs(loop.bulge(i)) > kEps) {
            out.edges.push_back(loop.edges[i]);
            continue;
        }
        const Vec2 P = loop.edges[prev].a, V = loop.edges[i].a, N = loop.end(i);
        const Vec2 d0 = normalize(V - P), d1 = normalize(N - V);
        const Real phi = std::atan2(cross(d0, d1), dot(d0, d1));
        if (std::fabs(phi) < 1e-4 || std::fabs(phi) > kTau * 0.5 - 1e-4) {
            out.edges.push_back(loop.edges[i]);
            continue;
        }
        // Tangent set-back, clamped so neighbouring fillets cannot overrun each
        // other (which would turn the loop inside out at that corner). The
        // clamp is exactly HALF the shorter neighbouring edge, not a hair less:
        // at half, two fillets from opposite ends meet precisely and the wall
        // between them vanishes — which is what turns a square into a true
        // circle at full radius. Backing the clamp off to 0.48 leaves a 4%
        // stub on every side and roundToCircle can never actually arrive.
        Real t = r * std::fabs(std::tan(phi * 0.5));
        t = std::min(t, std::min((V - P).length(), (N - V).length()) * 0.5);
        Edge2 arc;
        arc.a = V - d0 * t;
        arc.bulge = std::tan(phi * 0.25);          // the fillet arc itself
        arc.tag = loop.edges[prev].tag;            // it belongs to the wall it cut
        out.edges.push_back(arc);
        Edge2 wall;
        wall.a = V + d1 * t;
        wall.bulge = loop.bulge(i);
        wall.tag = loop.edges[i].tag;
        out.edges.push_back(wall);
    }
    return out;
}

Shape2 fillet(const Shape2& shape, std::size_t vertex, Real r) {
    Shape2 out = shape;
    if (shape.outer.size() < 3) return out;
    std::vector<Real> radii(shape.outer.size(), 0.0);
    radii[vertex % radii.size()] = r;
    out.outer = filletCorners(shape.outer, radii);
    return out;
}

Shape2 filletAll(const Shape2& shape, Real r) {
    Shape2 out;
    out.outer = filletCorners(shape.outer,
                              std::vector<Real>(shape.outer.size(), r));
    for (const Loop2& h : shape.holes)
        out.holes.push_back(filletCorners(h, std::vector<Real>(h.size(), r)));
    return out;
}

Loop2 chamferCorners(const Loop2& loop, const std::vector<Real>& cuts) {
    const std::size_t n = loop.size();
    if (n < 3 || cuts.size() != n) return loop;
    Loop2 out;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        const Real c = cuts[i];
        if (c <= 1e-6 || std::fabs(loop.bulge(prev)) > kEps ||
            std::fabs(loop.bulge(i)) > kEps) {
            out.edges.push_back(loop.edges[i]);
            continue;
        }
        const Vec2 P = loop.edges[prev].a, V = loop.edges[i].a, N = loop.end(i);
        const Vec2 d0 = normalize(V - P), d1 = normalize(N - V);
        if (std::fabs(cross(d0, d1)) < 1e-6) {
            out.edges.push_back(loop.edges[i]);
            continue;
        }
        const Real t = std::min(c, std::min((V - P).length(), (N - V).length()) * 0.48);
        Edge2 cut;
        cut.a = V - d0 * t;
        cut.bulge = 0;                             // a straight cut, not an arc
        cut.tag = loop.edges[prev].tag;
        out.edges.push_back(cut);
        Edge2 wall;
        wall.a = V + d1 * t;
        wall.bulge = loop.bulge(i);
        wall.tag = loop.edges[i].tag;
        out.edges.push_back(wall);
    }
    return out;
}

Shape2 chamferAll(const Shape2& shape, Real cut) {
    Shape2 out;
    out.outer = chamferCorners(shape.outer,
                               std::vector<Real>(shape.outer.size(), cut));
    for (const Loop2& h : shape.holes)
        out.holes.push_back(chamferCorners(h, std::vector<Real>(h.size(), cut)));
    return out;
}

Shape2 roundToCircle(const Shape2& shape, Real t) {
    const std::size_t n = shape.outer.size();
    if (n < 3) return shape;
    t = std::max(Real(0), std::min(Real(1), t));
    std::vector<Real> radii(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        const Vec2 P = shape.outer.edges[prev].a;
        const Vec2 V = shape.outer.edges[i].a;
        const Vec2 N = shape.outer.end(i);
        const Vec2 d0 = normalize(V - P), d1 = normalize(N - V);
        const Real phi = std::atan2(cross(d0, d1), dot(d0, d1));
        const Real tanHalf = std::fabs(std::tan(phi * 0.5));
        if (tanHalf < 1e-6) continue;
        // The inscribed limit: the largest radius whose tangent set-back reaches
        // halfway along the shorter neighbouring edge. At t = 1 every corner is
        // at its limit and a regular polygon closes into a true circle.
        const Real half = std::min((V - P).length(), (N - V).length()) * 0.5;
        radii[i] = t * half / tanHalf;
    }
    Shape2 out = shape;
    out.outer = filletCorners(shape.outer, radii);
    return out;
}

// ---------------------------------------------------------------------------
// Sampled field
// ---------------------------------------------------------------------------

Real Field2::at(int x, int y) const {
    x = std::max(0, std::min(nx - 1, x));
    y = std::max(0, std::min(ny - 1, y));
    return d[static_cast<std::size_t>(y) * nx + x];
}

Real Field2::sample(const Vec2& p) const {
    if (nx < 2 || ny < 2) return 1e30;
    const Real fx = (p.x - lo.x) / cell, fy = (p.y - lo.y) / cell;
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    const Real tx = fx - x0, ty = fy - y0;
    const Real a = at(x0, y0), b = at(x0 + 1, y0);
    const Real c = at(x0, y0 + 1), e = at(x0 + 1, y0 + 1);
    return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + e * tx) * ty;
}

Field2 rasterize(const std::vector<Shape2>& shapes, Real cell, Real pad) {
    Field2 f;
    f.cell = std::max(cell, Real(1e-3));
    // Boundary polylines, once, at a tolerance finer than the grid — the field
    // can never be more accurate than its own sampling anyway.
    std::vector<Poly2> loops;
    for (const Shape2& s : shapes) {
        Poly2 o = tessellate(s.outer, f.cell * 0.5);
        if (o.size() >= 3) loops.push_back(std::move(o));
        for (const Loop2& h : s.holes) {
            Poly2 hp = tessellate(h, f.cell * 0.5);
            if (hp.size() >= 3) loops.push_back(std::move(hp));
        }
    }
    if (loops.empty()) return f;

    Vec2 lo(1e30, 1e30), hi(-1e30, -1e30);
    for (const Poly2& l : loops)
        for (const Vec2& v : l) {
            lo.x = std::min(lo.x, v.x); lo.y = std::min(lo.y, v.y);
            hi.x = std::max(hi.x, v.x); hi.y = std::max(hi.y, v.y);
        }
    f.lo = Vec2(lo.x - pad, lo.y - pad);
    f.hi = Vec2(hi.x + pad, hi.y + pad);
    f.nx = std::max(2, static_cast<int>(std::ceil((f.hi.x - f.lo.x) / f.cell)) + 1);
    f.ny = std::max(2, static_cast<int>(std::ceil((f.hi.y - f.lo.y) / f.cell)) + 1);
    // Guard rail: a tiny cell on a big shape would allocate without bound.
    const long long cells = static_cast<long long>(f.nx) * f.ny;
    if (cells > 4'000'000) {
        const Real scale = std::sqrt(static_cast<Real>(cells) / 4'000'000.0);
        f.cell *= scale;
        f.nx = std::max(2, static_cast<int>(std::ceil((f.hi.x - f.lo.x) / f.cell)) + 1);
        f.ny = std::max(2, static_cast<int>(std::ceil((f.hi.y - f.lo.y) / f.cell)) + 1);
    }
    f.d.assign(static_cast<std::size_t>(f.nx) * f.ny, 1e30);

    for (int y = 0; y < f.ny; ++y) {
        for (int x = 0; x < f.nx; ++x) {
            const Vec2 p(f.lo.x + x * f.cell, f.lo.y + y * f.cell);
            Real best = 1e30;
            for (const Poly2& l : loops)
                for (std::size_t i = 0; i < l.size(); ++i)
                    best = std::min(best, distToSegment(p, l[i], l[(i + 1) % l.size()]));
            // Sign from even-odd containment across ALL loops, so a hole reads
            // as outside exactly like it does everywhere else in the kernel.
            const bool inside = inLoops(loops, p);
            f.d[static_cast<std::size_t>(y) * f.nx + x] = inside ? -best : best;
        }
    }
    return f;
}

std::vector<Poly2> contour(const Field2& f, Real level) {
    if (f.nx < 2 || f.ny < 2) return {};
    // Marching squares -> a soup of segments -> chained loops. The winding a
    // cell walk produces carries no information, so nesting decides at the end.
    std::vector<std::pair<Vec2, Vec2>> segs;
    auto interp = [&](const Vec2& p0, Real v0, const Vec2& p1, Real v1) {
        const Real den = (v1 - v0);
        const Real t = std::fabs(den) < 1e-12 ? 0.5 : (level - v0) / den;
        return lerp(p0, p1, std::max(Real(0), std::min(Real(1), t)));
    };
    for (int y = 0; y + 1 < f.ny; ++y) {
        for (int x = 0; x + 1 < f.nx; ++x) {
            const Real v[4] = {f.at(x, y), f.at(x + 1, y), f.at(x + 1, y + 1),
                               f.at(x, y + 1)};
            const Vec2 p[4] = {Vec2(f.lo.x + x * f.cell, f.lo.y + y * f.cell),
                               Vec2(f.lo.x + (x + 1) * f.cell, f.lo.y + y * f.cell),
                               Vec2(f.lo.x + (x + 1) * f.cell, f.lo.y + (y + 1) * f.cell),
                               Vec2(f.lo.x + x * f.cell, f.lo.y + (y + 1) * f.cell)};
            int code = 0;
            for (int i = 0; i < 4; ++i) if (v[i] < level) code |= (1 << i);
            if (code == 0 || code == 15) continue;
            // Edge crossings: e0 = bottom, e1 = right, e2 = top, e3 = left.
            Vec2 e[4];
            bool has[4] = {false, false, false, false};
            for (int i = 0; i < 4; ++i) {
                const int j = (i + 1) % 4;
                if (((code >> i) & 1) != ((code >> j) & 1)) {
                    e[i] = interp(p[i], v[i], p[j], v[j]);
                    has[i] = true;
                }
            }
            std::vector<int> hit;
            for (int i = 0; i < 4; ++i) if (has[i]) hit.push_back(i);
            if (hit.size() == 2) {
                segs.push_back({e[hit[0]], e[hit[1]]});
            } else if (hit.size() == 4) {
                // Saddle: pair by the cell-centre value so the two branches do
                // not cross each other.
                const Real centre = (v[0] + v[1] + v[2] + v[3]) * 0.25;
                if ((centre < level) == ((code & 1) != 0)) {
                    segs.push_back({e[0], e[1]});
                    segs.push_back({e[2], e[3]});
                } else {
                    segs.push_back({e[1], e[2]});
                    segs.push_back({e[3], e[0]});
                }
            }
        }
    }
    if (segs.empty()) return {};

    const Real tol = f.cell * 0.05 + 1e-6;
    auto key = [&](const Vec2& v) {
        return std::make_pair(static_cast<long long>(std::llround(v.x / tol)),
                              static_cast<long long>(std::llround(v.y / tol)));
    };
    std::map<std::pair<long long, long long>, std::vector<int>> byPoint;
    for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
        byPoint[key(segs[i].first)].push_back(i);
        byPoint[key(segs[i].second)].push_back(i);
    }
    std::vector<char> used(segs.size(), 0);
    std::vector<Poly2> loops;
    for (int s = 0; s < static_cast<int>(segs.size()); ++s) {
        if (used[s]) continue;
        Poly2 loop;
        int cur = s;
        Vec2 at = segs[s].first;
        while (cur != -1 && !used[cur]) {
            used[cur] = 1;
            loop.push_back(at);
            const Vec2 other = (key(segs[cur].first) == key(at)) ? segs[cur].second
                                                                 : segs[cur].first;
            at = other;
            int next = -1;
            auto it = byPoint.find(key(at));
            if (it != byPoint.end())
                for (int cand : it->second)
                    if (!used[cand]) { next = cand; break; }
            cur = next;
        }
        if (loop.size() >= 3) loops.push_back(std::move(loop));
    }
    return orientByNesting(std::move(loops));
}

namespace {

std::vector<Shape2> shapesFromContour(std::vector<Poly2> loops) {
    std::vector<Loop2> ls;
    ls.reserve(loops.size());
    for (const Poly2& p : loops)
        if (p.size() >= 3) ls.push_back(loopFromPoly(p));
    return assembleShapes(std::move(ls));
}

}  // namespace

std::vector<Shape2> fieldOffset(const Shape2& shape, Real d, Real cell) {
    if (shape.outer.size() < 3) return {};
    // The grid must resolve the offset itself, or the answer is noise.
    const Real c = std::min(cell, std::max(std::fabs(d) / 4.0, Real(0.05)));
    Field2 f = rasterize({shape}, c, std::fabs(d) * 1.5 + c * 4);
    // sdf < 0 inside, so the offset region is {sdf <= d}: positive d grows.
    std::vector<Poly2> loops = contour(f, d);
    return shapesFromContour(std::move(loops));
}

std::vector<Shape2> fieldBool(const std::vector<Shape2>& a,
                              const std::vector<Shape2>& b, BoolOp op,
                              Real cell) {
    std::vector<Shape2> all = a;
    all.insert(all.end(), b.begin(), b.end());
    if (all.empty()) return {};
    Field2 fa = rasterize(a, cell, cell * 4);
    Field2 fb = rasterize(b, cell, cell * 4);
    // One shared grid spanning both operands, so the two fields can be combined
    // sample by sample.
    Field2 f = rasterize(all, cell, cell * 4);
    for (int y = 0; y < f.ny; ++y)
        for (int x = 0; x < f.nx; ++x) {
            const Vec2 p(f.lo.x + x * f.cell, f.lo.y + y * f.cell);
            const Real da = fa.nx >= 2 ? fa.sample(p) : Real(1e30);
            const Real db = fb.nx >= 2 ? fb.sample(p) : Real(1e30);
            Real v = da;
            switch (op) {
                case BoolOp::Unite:     v = std::min(da, db); break;
                case BoolOp::Intersect: v = std::max(da, db); break;
                case BoolOp::Subtract:  v = std::max(da, -db); break;
            }
            f.d[static_cast<std::size_t>(y) * f.nx + x] = v;
        }
    return shapesFromContour(contour(f, 0.0));
}

std::vector<Shape2> smoothUnion(const Shape2& a, const Shape2& b, Real k,
                                Real cell) {
    if (k <= 0) return unite(a, b);
    std::vector<Shape2> all{a, b};
    Field2 fa = rasterize({a}, cell, k * 2 + cell * 4);
    Field2 fb = rasterize({b}, cell, k * 2 + cell * 4);
    Field2 f = rasterize(all, cell, k * 2 + cell * 4);
    for (int y = 0; y < f.ny; ++y)
        for (int x = 0; x < f.nx; ++x) {
            const Vec2 p(f.lo.x + x * f.cell, f.lo.y + y * f.cell);
            const Real da = fa.sample(p), db = fb.sample(p);
            // Polynomial smooth-min (the 2-D sibling of sdfSmoothUnion). Note
            // it ADDS material in the fillet — a blended join is bigger than
            // either operand, by design.
            const Real h = std::max(Real(0), std::min(Real(1),
                               0.5 + 0.5 * (db - da) / k));
            f.d[static_cast<std::size_t>(y) * f.nx + x] =
                db * (1 - h) + da * h - k * h * (1 - h);
        }
    return shapesFromContour(contour(f, 0.0));
}

}  // namespace engine
