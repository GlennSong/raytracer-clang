#include "section.h"

#include "shape_grammar.h"   // offsetPlanPublic
#include <cmath>

namespace engine {

namespace {

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

Poly2 ring(const std::vector<Edge>& edges, Real /*tol*/) {
    Poly2 out;
    out.reserve(edges.size());
    for (const Edge& e : edges) out.push_back(e.a);   // (2b: an arc's or a spline's chords follow its corner)
    return out;
}

}  // namespace

Poly2 Section::tessellate(Real tol) const { return ring(outer, tol); }

std::vector<Poly2> Section::holesTessellated(Real tol) const {
    std::vector<Poly2> out;
    for (const std::vector<Edge>& h : holes) out.push_back(ring(h, tol));
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

}  // namespace engine
