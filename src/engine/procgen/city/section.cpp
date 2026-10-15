#include "section.h"

#include "shape_grammar.h"   // offsetPlanPublic
#include <cmath>

namespace engine {

namespace {

constexpr Real kPi = 3.14159265358979323846;

// An arc from its chord and sagitta: the centre, radius and the signed sweep (left of a->b positive).
struct ArcGeom { Vec2 c; Real r = 0; Real a0 = 0, sweep = 0; bool ok = false; };
ArcGeom arcOf(const Edge& e) {
    ArcGeom g;
    const Vec2 ch = e.b - e.a;
    const Real L = ch.length();
    if (L < 1e-9 || std::fabs(e.bulge) < 1e-9) return g;
    const Real s = std::fabs(e.bulge);
    g.r = (L * L * 0.25 + s * s) / (2.0 * s);
    const Vec2 mid = (e.a + e.b) * 0.5;
    const Vec2 left(-ch.y / L, ch.x / L);
    const Real sign = e.bulge > 0 ? 1.0 : -1.0;
    // the centre sits on the far side of the chord from the bulge, r - s from the mid point
    g.c = mid - left * (sign * (g.r - s));
    const Vec2 da = e.a - g.c;
    g.a0 = std::atan2(da.y, da.x);
    // The sweep: an arc bulging LEFT of a->b has its centre to the right (a minor arc) and runs clockwise from a to
    // b -- negative; bulging right, counter-clockwise. Past a half circle the centre crosses to the bulge's side and
    // the arc goes the long way round, the same way.
    const Real half = std::asin(std::min(Real(1), L * 0.5 / g.r));
    g.sweep = -sign * 2.0 * half;
    if (s > L * 0.5) g.sweep = -sign * (2.0 * kPi - 2.0 * half);
    g.ok = true;
    return g;
}

Section fromPoints(const Poly2& pts) {
    Section s;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        Edge e;
        e.a = pts[i];
        e.b = pts[(i + 1) % pts.size()];
        s.outer.push_back(e);
    }
    return s;
}

void ring(const std::vector<Edge>& edges, Real tol, Poly2& pts, std::vector<int>* edgeOf) {
    for (std::size_t i = 0; i < edges.size(); ++i) {
        const Edge& e = edges[i];
        pts.push_back(e.a);
        if (edgeOf) edgeOf->push_back(static_cast<int>(i));
        if (e.kind != Edge::Kind::Arc) continue;
        const ArcGeom g = arcOf(e);
        if (!g.ok) continue;
        // chords rising at most `tol` off the arc: the half angle each may span
        const Real halfMax = std::acos(std::max(Real(-1), std::min(Real(1), 1.0 - std::max(tol, Real(1e-3)) / g.r)));
        const int n = std::max(1, static_cast<int>(std::ceil(std::fabs(g.sweep) / (2.0 * halfMax))));
        for (int k = 1; k < n; ++k) {
            const Real ang = g.a0 + g.sweep * (static_cast<Real>(k) / n);
            pts.push_back(g.c + Vec2(std::cos(ang), std::sin(ang)) * g.r);
            if (edgeOf) edgeOf->push_back(static_cast<int>(i));
        }
    }
}

}  // namespace

Real Edge::length() const {
    if (kind != Kind::Arc) return (b - a).length();
    const ArcGeom g = arcOf(*this);
    return g.ok ? std::fabs(g.sweep) * g.r : (b - a).length();
}

Poly2 Section::tessellate(Real tol) const {
    Poly2 pts;
    ring(outer, tol, pts, nullptr);
    return pts;
}

Tessellation Section::tessellation(Real tol) const {
    Tessellation t;
    ring(outer, tol, t.points, &t.edgeOf);
    return t;
}

std::vector<Poly2> Section::holesTessellated(Real tol) const {
    std::vector<Poly2> out;
    for (const std::vector<Edge>& h : holes) { Poly2 pts; ring(h, tol, pts, nullptr); out.push_back(pts); }
    return out;
}

Section Section::rect(Real w, Real d, const Vec2& c) {
    return fromPoints({{c.x - w / 2, c.y - d / 2}, {c.x + w / 2, c.y - d / 2}, {c.x + w / 2, c.y + d / 2}, {c.x - w / 2, c.y + d / 2}});
}

Section Section::ngon(int n, Real r, const Vec2& c) {
    Poly2 pts;
    for (int k = 0; k < n; ++k) {
        const Real a2 = 2.0 * 3.14159265358979323846 * k / n;
        pts.push_back(c + Vec2(std::cos(a2), std::sin(a2)) * r);
    }
    return fromPoints(pts);
}

Section Section::fromLot(const Poly2& plan) {
    Poly2 p = plan;
    if (p.size() >= 3) ensureCCW(p);
    return fromPoints(p);
}

Section Section::offset(Real d) const {
    Section s = fromPoints(offsetPlanPublic(tessellate(), d));
    s.holes = holes;
    return s;
}

Section Section::withHole(const Section& hole) const {
    Section s = *this;
    s.holes.push_back(hole.outer);
    return s;
}

Section Section::rounded(Real r) const {
    const std::size_t n = outer.size();
    if (n < 3 || r <= 0) return *this;
    // Each corner's trim: r tan(phi/2) along both edges, phi the turn there; never past half an edge.
    std::vector<Real> trim(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const Edge& in = outer[(i + n - 1) % n];
        const Edge& out = outer[i];
        if (in.kind != Edge::Kind::Line || out.kind != Edge::Kind::Line) continue;
        const Vec2 d0 = normalize(in.b - in.a), d1 = normalize(out.b - out.a);
        const Real phi = std::acos(std::max(Real(-1), std::min(Real(1), dot(d0, d1))));   // the turn
        if (phi < 1e-3 || phi > kPi - 1e-3) continue;
        trim[i] = r * std::tan(phi * 0.5);
    }
    for (std::size_t i = 0; i < n; ++i) {   // an edge can give each end at most half its length
        const Real L = (outer[i].b - outer[i].a).length();
        const Real give = L * 0.5;
        if (trim[i] > give) trim[i] = give;
        if (trim[(i + 1) % n] > give) trim[(i + 1) % n] = give;
    }
    Section s;
    s.holes = holes;
    for (std::size_t i = 0; i < n; ++i) {
        const Edge& e = outer[i];
        const Vec2 d = normalize(e.b - e.a);
        const Real tA = trim[i], tB = trim[(i + 1) % n];
        Edge line = e;
        line.a = e.a + d * tA;
        line.b = e.b - d * tB;
        if ((line.b - line.a).length() > 1e-4) s.outer.push_back(line);   // (an edge the fillets consumed whole is gone)
        if (tB > 1e-9) {
            // the fillet from this edge's trimmed end to the next edge's trimmed start, bulging toward the corner
            const Edge& nx = outer[(i + 1) % n];
            const Vec2 dn = normalize(nx.b - nx.a);
            const Real phi = std::acos(std::max(Real(-1), std::min(Real(1), dot(d, dn))));
            const Real rr = tB / std::tan(phi * 0.5);   // the radius the trim could afford
            Edge arc;
            arc.kind = Edge::Kind::Arc;
            arc.a = e.b - d * tB;
            arc.b = nx.a + dn * tB;
            const Real sag = rr * (1.0 - std::cos(phi * 0.5));
            arc.bulge = cross(d, dn) > 0 ? -sag : sag;   // a left turn (convex, CCW): the arc bulges right, toward the corner
            s.outer.push_back(arc);
        }
    }
    return s;
}

Section Section::transformed(const Vec2& o, const Vec2& u, const Vec2& v) const {
    auto map = [&](const Vec2& p) { return o + u * p.x + v * p.y; };
    Section s = *this;
    for (Edge& e : s.outer) { e.a = map(e.a); e.b = map(e.b); for (Vec2& c : e.ctrl) c = map(c); }
    for (std::vector<Edge>& h : s.holes) for (Edge& e : h) { e.a = map(e.a); e.b = map(e.b); }
    // a mirrored frame flips the bulges' side
    if (cross(u, v) < 0) { for (Edge& e : s.outer) e.bulge = -e.bulge; for (std::vector<Edge>& h : s.holes) for (Edge& e : h) e.bulge = -e.bulge; }
    return s;
}

}  // namespace engine
