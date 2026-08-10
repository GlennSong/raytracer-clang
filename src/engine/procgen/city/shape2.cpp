#include "shape2.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace {

constexpr Real kTau = 6.28318530717958647692;
constexpr Real kEps = 1e-9;

}  // namespace

// Bulge and tag belong to the EDGE, not the vertex, so reversing is not just
// reversing the point list: after the reversal, output edge j traverses
// original edge (n-2-j) BACKWARDS, which negates its bulge (a CCW sweep walked
// the other way is CW) and carries its tag. Getting this wrong is the classic
// arc-loop bug — the shape looks right until something reverses it and every
// curve inverts.
void reverseLoop(Loop2& loop) {
    const std::size_t n = loop.size();
    if (n < 2) return;
    std::vector<Edge2> out(n);
    for (std::size_t j = 0; j < n; ++j) {
        const std::size_t src = (2 * n - 2 - j) % n;
        out[j].a = loop.edges[(n - 1 - j) % n].a;
        out[j].bulge = -loop.edges[src].bulge;
        out[j].tag = loop.edges[src].tag;
    }
    loop.edges = std::move(out);
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Loop2 loopFromPoly(const Poly2& poly, EdgeTag tag) {
    Loop2 loop;
    loop.edges.reserve(poly.size());
    for (const Vec2& v : poly) loop.edges.push_back(Edge2{v, 0, tag});
    return loop;
}

Shape2 shapeFromPoly(const Poly2& poly, EdgeTag tag) {
    Shape2 s;
    s.outer = loopFromPoly(poly, tag);
    orient(s);
    return s;
}

Shape2 shapeFromPolys(const Poly2& outer, const std::vector<Poly2>& holes,
                      EdgeTag tag) {
    Shape2 s;
    s.outer = loopFromPoly(outer, tag);
    for (const Poly2& h : holes) {
        if (h.size() < 3) continue;
        s.holes.push_back(loopFromPoly(h, tag));
    }
    orient(s);
    return s;
}

Shape2 rectShape(Real x0, Real y0, Real x1, Real y1, EdgeTag tag) {
    if (x1 < x0) std::swap(x0, x1);
    if (y1 < y0) std::swap(y0, y1);
    Poly2 p = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};   // CCW
    return shapeFromPoly(p, tag);
}

Shape2 circleShape(const Vec2& centre, Real radius) {
    Shape2 s;
    // Four quarter-turn arcs. bulge = tan(sweep/4) = tan(90°/4) = tan(22.5°).
    const Real b = std::tan(kTau / 16.0);
    const Vec2 pts[4] = {{centre.x + radius, centre.y},
                         {centre.x, centre.y + radius},
                         {centre.x - radius, centre.y},
                         {centre.x, centre.y - radius}};
    for (const Vec2& p : pts) s.outer.edges.push_back(Edge2{p, b, EdgeTag::None});
    return s;
}

// ---------------------------------------------------------------------------
// Arc geometry
// ---------------------------------------------------------------------------

ArcGeom arcGeom(const Vec2& a, const Vec2& b, Real bulge) {
    ArcGeom g;
    if (std::fabs(bulge) < kEps) return g;
    const Vec2 d = b - a;
    const Real chord = d.length();
    if (chord < kEps) return g;
    g.straight = false;
    g.sweep = 4.0 * std::atan(bulge);                    // DXF: bulge = tan(s/4)
    g.radius = chord / (2.0 * std::sin(std::fabs(g.sweep) * 0.5));
    // The centre sits off the chord midpoint along the perpendicular. WHICH
    // SIDE is the subtle part, and it is invisible in an SVG (an `A` command
    // re-derives the centre from radius + flags) — it only shows up when the
    // loop is tessellated and the curve bows the wrong way. A positive
    // (counter-clockwise) sweep puts the centre on the LEFT of a->b, so the arc
    // itself swells to the RIGHT; on a CCW loop, right is outward.
    const Vec2 m = (a + b) * 0.5;
    const Vec2 n = normalize(Vec2(-d.y, d.x));           // left of a->b
    const Real h = std::sqrt(std::max(Real(0),
                                      g.radius * g.radius - chord * chord * 0.25));
    const Real major = (std::fabs(g.sweep) > kTau * 0.5) ? -1.0 : 1.0;
    g.centre = m + n * (h * major * (bulge > 0 ? 1.0 : -1.0));
    g.startAngle = std::atan2(a.y - g.centre.y, a.x - g.centre.x);
    return g;
}

Vec2 arcPoint(const Vec2& a, const Vec2& b, Real bulge, Real t) {
    const ArcGeom g = arcGeom(a, b, bulge);
    if (g.straight) return lerp(a, b, t);
    const Real ang = g.startAngle + g.sweep * t;
    return Vec2(g.centre.x + g.radius * std::cos(ang),
                g.centre.y + g.radius * std::sin(ang));
}

Real subBulge(Real bulge, Real t0, Real t1) {
    if (std::fabs(bulge) < kEps) return 0;
    // A sub-arc of a circle is an arc ON THE SAME CIRCLE, so this is exact:
    // the sweep scales with the parameter span and the bulge follows.
    const Real sweep = 4.0 * std::atan(bulge) * (t1 - t0);
    return std::tan(sweep * 0.25);
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

Real signedArea(const Loop2& loop) {
    const std::size_t n = loop.size();
    if (n < 3) return 0;
    Poly2 pts;
    pts.reserve(n);
    for (const Edge2& e : loop.edges) pts.push_back(e.a);
    Real a = signedArea(pts);
    // Add each arc's circular SEGMENT — the crescent between the chord and the
    // true arc. Without this a circle measures as its inscribed square and
    // every area-conservation check in the kernel is quietly wrong.
    for (std::size_t i = 0; i < n; ++i) {
        const ArcGeom g = arcGeom(loop.start(i), loop.end(i), loop.bulge(i));
        if (g.straight) continue;
        a += 0.5 * g.radius * g.radius * (g.sweep - std::sin(g.sweep));
    }
    return a;
}

Real area(const Shape2& shape) {
    Real a = area(shape.outer);
    for (const Loop2& h : shape.holes) a -= area(h);
    return std::max(Real(0), a);
}

Real perimeter(const Loop2& loop) {
    Real p = 0;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const ArcGeom g = arcGeom(loop.start(i), loop.end(i), loop.bulge(i));
        p += g.straight ? (loop.end(i) - loop.start(i)).length()
                        : std::fabs(g.radius * g.sweep);
    }
    return p;
}

bool contains(const Shape2& shape, const Vec2& p) {
    const TessShape t = tessellate(shape, 0.01);
    if (t.outer.size() < 3 || !pointInPolygon(t.outer, p)) return false;
    for (const Poly2& h : t.holes)
        if (h.size() >= 3 && pointInPolygon(h, p)) return false;
    return true;
}

Vec2 centroid(const Shape2& shape) {
    const TessShape t = tessellate(shape, 0.02);
    if (t.outer.size() < 3) return Vec2();
    Real aOut = std::abs(signedArea(t.outer));
    Vec2 acc = centroid(t.outer) * aOut;
    Real total = aOut;
    for (const Poly2& h : t.holes) {
        if (h.size() < 3) continue;
        const Real ah = std::abs(signedArea(h));
        acc = acc - centroid(h) * ah;
        total -= ah;
    }
    return total > kEps ? acc / total : centroid(t.outer);
}

void bounds(const Shape2& shape, Vec2& lo, Vec2& hi) {
    // Tessellated so an arc that bows past its chord endpoints is included.
    const Poly2 t = tessellate(shape.outer, 0.01);
    if (t.empty()) { lo = hi = Vec2(); return; }
    bounds(t, lo, hi);
}

// ---------------------------------------------------------------------------
// Normalization
// ---------------------------------------------------------------------------

void orient(Shape2& shape) {
    if (shape.outer.size() >= 3 && signedArea(shape.outer) < 0)
        reverseLoop(shape.outer);
    for (Loop2& h : shape.holes)
        if (h.size() >= 3 && signedArea(h) > 0) reverseLoop(h);
}

std::vector<Poly2> orientByNesting(std::vector<Poly2> loops) {
    for (std::size_t i = 0; i < loops.size(); ++i) {
        if (loops[i].size() < 3) continue;
        int depth = 0;
        for (std::size_t j = 0; j < loops.size(); ++j) {
            if (i == j || loops[j].size() < 3) continue;
            if (area(loops[j]) > area(loops[i]) &&
                pointInPolygon(loops[j], loops[i][0]))
                ++depth;
        }
        const bool wantCCW = (depth % 2) == 0;
        if ((signedArea(loops[i]) > 0) != wantCCW)
            std::reverse(loops[i].begin(), loops[i].end());
    }
    return loops;
}

Loop2 simplify(const Loop2& loop, Real tol) {
    const std::size_t n = loop.size();
    if (n < 4 || tol <= 0) return loop;

    // PASS 1 — weld duplicate vertices against the previously KEPT one. This
    // has to be a running comparison: judging each vertex against its ORIGINAL
    // neighbours lets a pair of coincident vertices delete the real corner
    // between them (each looks collinear with the other's copy) and the shape
    // loses a whole side.
    Loop2 welded;
    for (std::size_t i = 0; i < n; ++i) {
        const Edge2& e = loop.edges[i];
        if (!welded.empty() && std::fabs(welded.edges.back().bulge) < kEps &&
            (e.a - welded.edges.back().a).length() < tol) {
            // Same point: the survivor takes THIS vertex's outgoing edge, so an
            // arc leaving a welded vertex is never lost.
            welded.edges.back().bulge = e.bulge;
            welded.edges.back().tag = e.tag;
            continue;
        }
        welded.edges.push_back(e);
    }
    while (welded.size() >= 4 && std::fabs(welded.edges.back().bulge) < kEps &&
           (welded.edges.back().a - welded.edges.front().a).length() < tol)
        welded.edges.pop_back();
    if (welded.size() < 4) return welded.size() >= 3 ? welded : loop;

    // PASS 2 — drop vertices lying on the straight line between their
    // neighbours. An ARC endpoint is never dropped: "nearly collinear" is
    // meaningless for a curve the author asked for on purpose, and removing its
    // endpoint would silently restate the curve as a different one.
    Loop2 out;
    const std::size_t m = welded.size();
    for (std::size_t i = 0; i < m; ++i) {
        const Edge2& e = welded.edges[i];
        const std::size_t prev = (i + m - 1) % m;
        if (std::fabs(e.bulge) > kEps || std::fabs(welded.bulge(prev)) > kEps) {
            out.edges.push_back(e);
            continue;
        }
        const Vec2 A = welded.edges[prev].a, B = e.a, C = welded.end(i);
        const Vec2 d = C - A;
        if (d.length() > kEps && std::fabs(cross(normalize(d), B - A)) < tol)
            continue;
        out.edges.push_back(e);
    }
    return out.size() >= 3 ? out : welded;
}

Shape2 simplify(const Shape2& shape, Real tol) {
    Shape2 s;
    s.outer = simplify(shape.outer, tol);
    for (const Loop2& h : shape.holes) {
        Loop2 sh = simplify(h, tol);
        if (sh.size() >= 3) s.holes.push_back(std::move(sh));
    }
    return s;
}

// ---------------------------------------------------------------------------
// Tessellation
// ---------------------------------------------------------------------------

Poly2 tessellate(const Loop2& loop, Real chordTol) {
    Poly2 out;
    const std::size_t n = loop.size();
    out.reserve(n * 2);
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back(loop.start(i));
        const ArcGeom g = arcGeom(loop.start(i), loop.end(i), loop.bulge(i));
        if (g.straight) continue;
        // A chord subtending angle w on radius r sits r*(1-cos(w/2)) from the
        // arc; invert that for the largest step meeting the tolerance.
        const Real ratio = std::max(Real(-1),
                                    std::min(Real(1), 1.0 - chordTol / g.radius));
        const Real maxStep = 2.0 * std::acos(ratio);
        const int segs = std::max(2, static_cast<int>(std::ceil(
                             std::fabs(g.sweep) / std::max(maxStep, Real(1e-3)))));
        for (int k = 1; k < segs; ++k) {
            const Real ang = g.startAngle +
                             g.sweep * (static_cast<Real>(k) / segs);
            out.push_back(Vec2(g.centre.x + g.radius * std::cos(ang),
                               g.centre.y + g.radius * std::sin(ang)));
        }
    }
    return out;
}

TessShape tessellate(const Shape2& shape, Real chordTol) {
    TessShape t;
    t.outer = tessellate(shape.outer, chordTol);
    for (const Loop2& h : shape.holes) {
        Poly2 p = tessellate(h, chordTol);
        if (p.size() >= 3) t.holes.push_back(std::move(p));
    }
    return t;
}

// ---------------------------------------------------------------------------
// Editing verbs
// ---------------------------------------------------------------------------

void bowEdge(Loop2& loop, std::size_t i, Real sagitta) {
    if (loop.size() < 3) return;
    i %= loop.size();
    const Real chord = (loop.end(i) - loop.start(i)).length();
    if (chord < 1e-6) return;
    // sagitta s on chord c: bulge = 2s/c (exactly, by the DXF definition).
    loop.edges[i].bulge = 2.0 * sagitta / chord;
}

void apseEdge(Loop2& loop, std::size_t i, Real sign) {
    if (loop.size() < 3) return;
    loop.edges[i % loop.size()].bulge = sign >= 0 ? 1.0 : -1.0;   // half turn
}

void tagFacing(Loop2& loop, const Vec2& dir, Real deg, EdgeTag tag) {
    const std::size_t n = loop.size();
    if (n < 3) return;
    const Vec2 want = normalize(dir);
    const Real cosLimit = std::cos(std::max(Real(0), deg) * kTau / 360.0);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 d = loop.end(i) - loop.start(i);
        if (d.lengthSquared() < kEps) continue;
        // Interior is LEFT of a CCW edge, so outward is the RIGHT normal.
        const Vec2 outward = normalize(Vec2(d.y, -d.x));
        if (dot(outward, want) >= cosLimit) loop.edges[i].tag = tag;
    }
}

// ---------------------------------------------------------------------------
// The pen
// ---------------------------------------------------------------------------

Pen& Pen::moveTo(Real x, Real y) {
    cur_ = Vec2(x, y);
    edges_.clear();
    edges_.push_back(Edge2{cur_, 0, tag_});
    started_ = true;
    return *this;
}

Pen& Pen::lineTo(Real x, Real y) {
    if (!started_) return moveTo(x, y);
    const Vec2 next(x, y);
    edges_.back().bulge = 0;
    edges_.back().tag = tag_;
    dir_ = std::atan2(next.y - cur_.y, next.x - cur_.x);
    cur_ = next;
    edges_.push_back(Edge2{cur_, 0, tag_});
    return *this;
}

Pen& Pen::arcTo(Real x, Real y, Real bulge) {
    if (!started_) return moveTo(x, y);
    const Vec2 next(x, y);
    edges_.back().bulge = bulge;
    edges_.back().tag = tag_;
    // The heading LEAVING an arc is the chord direction turned by half the
    // sweep on each side — entering it turns by sweep/2, and so does leaving.
    const Real sweep = 4.0 * std::atan(bulge);
    dir_ = std::atan2(next.y - cur_.y, next.x - cur_.x) + sweep * 0.5;
    cur_ = next;
    edges_.push_back(Edge2{cur_, 0, tag_});
    return *this;
}

Pen& Pen::forward(Real d) {
    if (!started_) moveTo(0, 0);
    return lineTo(cur_.x + std::cos(dir_) * d, cur_.y + std::sin(dir_) * d);
}

Pen& Pen::turn(Real deg) {
    dir_ += deg * kTau / 360.0;
    return *this;
}

Pen& Pen::sweep(Real radius, Real deg) {
    if (!started_) moveTo(0, 0);
    if (radius <= 0 || std::fabs(deg) < 1e-6) return *this;
    const Real theta = deg * kTau / 360.0;              // signed sweep
    // ONE true arc edge, not a fan of chords — the reason the type carries
    // bulge at all. The chord of a sweep theta on radius r is 2r sin(theta/2),
    // and it leaves at heading + theta/2.
    const Real chord = 2.0 * radius * std::sin(std::fabs(theta) * 0.5);
    const Real chordDir = dir_ + theta * 0.5;
    const Vec2 next(cur_.x + std::cos(chordDir) * chord,
                    cur_.y + std::sin(chordDir) * chord);
    edges_.back().bulge = std::tan(theta * 0.25);
    edges_.back().tag = tag_;
    cur_ = next;
    dir_ += theta;
    edges_.push_back(Edge2{cur_, 0, tag_});
    return *this;
}

Pen& Pen::tag(EdgeTag t) {
    tag_ = t;
    return *this;
}

Loop2 Pen::closeLoop() const {
    Loop2 loop;
    loop.edges = edges_;
    // The pen leaves a trailing vertex at the current position; if it coincides
    // with the start, it IS the start and the closing edge is already implied.
    if (loop.size() > 1 &&
        (loop.edges.front().a - loop.edges.back().a).length() < 1e-6)
        loop.edges.pop_back();
    if (loop.size() >= 3 && signedArea(loop) < 0) reverseLoop(loop);
    return loop;
}

Shape2 Pen::close() const {
    Shape2 s;
    s.outer = closeLoop();
    return s;
}

}  // namespace engine
